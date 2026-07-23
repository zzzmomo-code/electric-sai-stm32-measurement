# ADS8688 Continuous Frame Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 修正 ADS8688 的确定性掉电唤醒顺序，并使采样率、FFT配置和文档与 SPI3 两周期帧间空闲一致。

**Architecture:** 保留已验证的 SPI3 32位循环DMA架构和所有外部接口。`ads8688_initialize_attempt()` 明确执行 PWR_DN、AUTO_RST唤醒、15ms稳定、寄存器配置和最终AUTO_RST；采样率继续由 `ads8688_get_effective_sample_rate_hz()` 统一提供，但帧周期改为34个SPI时钟。

**Tech Stack:** STM32H743 HAL、SPI3 + DMA1、ADS8688、Python `unittest`源码契约测试、STM32CubeIDE GNU Arm工具链。

## Global Constraints

- 不修改用户刚重新生成的 `.ioc` 和 CubeMX生成初始化代码。
- 所有用户代码修改仅位于 `Core/User/`，使用小写下划线命名并补齐中文注释。
- 中断回调继续只设置一个目的明确的标志。
- 不改变ADS8688外部接口、DMA缓冲区、storage容量或65536点FFT结构。
- 保留工作区中全部既有用户改动，不提交无关文件或Debug构建产物。
- 未经用户许可不推送远程仓库。

---

### Task 1: Add failing recovery and frame timing contracts

**Files:**
- Modify: `tests/test_ads8688_input_contract.py`

**Interfaces:**
- Consumes: `Core/User/ads8688.c`中的初始化实现和 `h743_pre1.ioc`中的SPI3配置。
- Produces: 对MIDI=2、34周期帧、PWR_DN唤醒顺序的可重复契约。

- [ ] **Step 1: Extend the IOC contract**

Add this required line to `test_spi3_and_dma_match_ads8688()`:

```python
"SPI3.MasterInterDataIdleness=SPI_MASTER_INTERDATA_IDLENESS_02CYCLE",
```

- [ ] **Step 2: Replace the old runtime-rate expectations**

Replace `test_runtime_rate_math()` with:

```python
def test_runtime_rate_math(self):
    self.assertAlmostEqual(16_000_000.0 / 34.0, 470_588.2353, places=3)
    self.assertAlmostEqual(16_000_000.0 / 68.0, 235_294.1176, places=3)
    self.assertAlmostEqual(34.0 / 16_000_000.0, 2.125e-6, places=12)
```

- [ ] **Step 3: Add the recovery-order contract**

Add to `Ads8688HardwareContract`:

```python
def test_power_down_wakeup_precedes_configuration(self):
    source = read("Core/User/ads8688.c")
    self.assertIn("#define ADS8688_SPI_FRAME_CYCLES           34.0f", source)
    body = source.split(
        "static ads8688_status_t ads8688_initialize_attempt(void)", 1
    )[1].split("\n}", 1)[0]
    power_down_delay = body.index("HAL_Delay(1u);")
    wake_command = body.index(
        "status = ads8688_send_command(ADS8688_COMMAND_AUTO_RST);"
    )
    reference_delay = body.index("HAL_Delay(15u);")
    first_configuration = body.index(
        "status = ads8688_write_and_verify_register("
    )
    final_auto_reset = body.rindex(
        "ads8688_send_command(ADS8688_COMMAND_AUTO_RST)"
    )
    self.assertLess(power_down_delay, wake_command)
    self.assertLess(wake_command, reference_delay)
    self.assertLess(reference_delay, first_configuration)
    self.assertLess(first_configuration, final_auto_reset)
```

- [ ] **Step 4: Run the focused tests and verify RED**

Run:

```powershell
python -m unittest tests.test_ads8688_input_contract -v
```

Expected: `test_power_down_wakeup_precedes_configuration` fails because the frame constant is still `33.0f` and the wake command still occurs after the 15ms delay; `test_runtime_rate_math` itself passes because it expresses the new required arithmetic.

- [ ] **Step 5: Commit the failing contract**

```powershell
git add tests/test_ads8688_input_contract.py
git commit -m "test: define ADS8688 recovery timing contract"
```

---

### Task 2: Implement deterministic wakeup and corrected sample rate

**Files:**
- Modify: `Core/User/ads8688.c`
- Modify: `Core/User/ads8688.h`
- Modify: `Core/User/measurement_input.c`

**Interfaces:**
- Consumes: `ads8688_send_command(uint16_t)`、现有寄存器写入校验和初始化重试机制。
- Produces: 保持原签名的 `ads8688_init()` 和按34周期计算的 `ads8688_get_effective_sample_rate_hz()`。

- [ ] **Step 1: Replace the frame constant and remove the stale duplicate**

In `Core/User/ads8688.c`:

```c
#define ADS8688_SPI_FRAME_CYCLES           34.0f
```

Delete the unused line from `Core/User/measurement_input.c`:

```c
#define MEASUREMENT_INPUT_ADS_FRAME_CYCLES 33.0f
```

- [ ] **Step 2: Replace the ambiguous RST/PD pulse**

Remove `ADS8688_RESET_HOLD_NOP_COUNT` and `nop_index`. Replace the reset section in `ads8688_initialize_attempt()` with:

```c
HAL_GPIO_WritePin(ADS8688_DAISY_GPIO_Port,
                  ADS8688_DAISY_Pin,
                  GPIO_PIN_RESET);
HAL_GPIO_WritePin(ADS8688_RST_GPIO_Port,
                  ADS8688_RST_Pin,
                  GPIO_PIN_RESET);
HAL_Delay(1u);
HAL_GPIO_WritePin(ADS8688_RST_GPIO_Port,
                  ADS8688_RST_Pin,
                  GPIO_PIN_SET);

status = ads8688_send_command(ADS8688_COMMAND_AUTO_RST);
if (status != ADS8688_STATUS_OK)
{
    return status;
}
HAL_Delay(15u);
```

Keep the existing register writes and final:

```c
return ads8688_send_command(ADS8688_COMMAND_AUTO_RST);
```

- [ ] **Step 3: Update Chinese comments**

Update module and function notes to state:

- SPI3 uses two inter-data idle cycles.
- RST/PD is held low for 1ms to enter PWR_DN intentionally.
- The first AUTO_RST exits PWR_DN, followed by 15ms reference settling.
- Each frame is 32 data clocks plus two idle clocks.

Update the sample-rate note in `Core/User/ads8688.h` to the same 34-cycle model.

- [ ] **Step 4: Run the focused tests and verify GREEN**

Run:

```powershell
python -m unittest tests.test_ads8688_input_contract -v
```

Expected: all ADS8688 input contract tests pass.

- [ ] **Step 5: Commit the implementation**

```powershell
git add Core/User/ads8688.c Core/User/ads8688.h Core/User/measurement_input.c
git commit -m "fix: recover ADS8688 with framed wakeup"
```

---

### Task 3: Synchronize documentation and verify the complete project

**Files:**
- Modify: `README.md`

**Interfaces:**
- Consumes: verified MIDI=2 IOC setting and 34-cycle driver calculation.
- Produces: accurate CubeMX setup, sample-rate values and hardware validation instructions.

- [ ] **Step 1: Update README timing and rates**

Replace the ADS8688 configuration and rate text with:

```text
Master Inter Data Idleness 2 Cycles
总帧率约 470.59 kframe/s
AIN0/AIN1双通道时每通道约235.29 kSPS
单通道时约470.59 kSPS
```

Document that H743 hardware NSS inter-frame pulses require `MIDI>1`, and that the driver intentionally enters PWR_DN for 1ms, sends AUTO_RST, waits15ms, configures registers and sends AUTO_RST again.

- [ ] **Step 2: Run all source-contract tests**

Run:

```powershell
python -m unittest discover -s tests -p "test_*.py" -v
```

Expected: all tests pass, except any explicitly documented host-toolchain skip.

- [ ] **Step 3: Build the Debug target**

Run the available STM32CubeIDE make tool against the generated Debug build:

```powershell
make -C Debug -j4 all
```

If `make` is not on PATH, use the `make.exe` located under the installed STM32CubeIDE 1.19.0 toolchain directory with the same arguments.

Expected: exit code 0 and no new warnings originating from `ads8688.c`, `ads8688.h` or `measurement_input.c`.

- [ ] **Step 4: Check project rules and staged scope**

Run:

```powershell
git diff --check
git status --short
```

Verify that `main.c` still calls `system_process()` in the loop, callbacks still assign one flag, and only the intended source, test, documentation and plan files are staged for this fix.

- [ ] **Step 5: Commit documentation**

```powershell
git add README.md
git commit -m "docs: update ADS8688 continuous frame timing"
```

- [ ] **Step 6: Report hardware validation**

Report the user-confirmed result that raw DMA data became nonzero after MIDI changed from1 to2. Leave voltage accuracy and long-duration FFT stability as hardware follow-up checks unless separately measured.
