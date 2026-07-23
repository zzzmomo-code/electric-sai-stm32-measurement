# ADS8688 DMA Backlog Handling Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让 ADS8688 DMA 半满和全满标志同时待处理时按既定半区顺序连续送入 FFT，避免正常积压反复清空 65536 点窗口。

**Architecture:** 保留现有两个中断标志和 `ads8688_expected_half` 状态机。删除“双标志直接丢弃并恢复”的提前返回，使后续已有分支在一次 `ads8688_process()` 中按期望顺序领取并处理两个半区；非期望的单标志和 SPI/DMA 错误仍走恢复路径。

**Tech Stack:** STM32H743、STM32 HAL SPI/DMA、C11、Python `unittest` 静态契约测试、STM32CubeIDE 1.19.0 GNU Arm 工具链。

## Global Constraints

- 不修改 `h743_pre1.ioc`、CubeMX 生成的 SPI3/DMA/GPIO 初始化代码或 SPI3 时钟。
- 不改变 `ADS8688_DMA_WORD_COUNT`、FFT 点数、公共接口、通道模式和量程。
- 中断回调继续只给各自唯一的 `xxx_flag` 赋值。
- 修改的用户代码继续位于 `Core/User/`，并使用小写下划线命名和中文注释。
- 保留用户现有未提交改动；只提交本计划列出的文件。
- 不推送远程仓库。

---

### Task 1: 用契约测试复现双标志错误恢复

**Files:**
- Modify: `tests/test_ads8688_input_contract.py:66-79`
- Test: `tests/test_ads8688_input_contract.py`

**Interfaces:**
- Consumes: `void ads8688_process(void)` 的源码文本。
- Produces: `test_driver_processes_both_pending_halves_without_resync` 回归测试。

- [ ] **Step 1: 将旧积压恢复断言替换为目标行为测试**

```python
def test_driver_processes_both_pending_halves_without_resync(self):
    source = read("Core/User/ads8688.c")
    process_body = source.split(
        "void ads8688_process(void)", 1
    )[1].split("\n}", 1)[0]
    self.assertNotIn("if (pending_flags == 3u)", process_body)
    self.assertNotIn(
        "ads8688_diagnostics.lost_samples += ADS8688_DMA_WORD_COUNT;",
        process_body,
    )
    self.assertNotIn("measurement_fft_resynchronize();", process_body)
    self.assertIn("ads8688_expected_half == 0u", process_body)
    self.assertIn("ads8688_process_dma_half(0u);", process_body)
    self.assertIn("ads8688_process_dma_half(1u);", process_body)
    self.assertIn(
        "ADS8688_DMA_HALF_WORD_COUNT;",
        process_body,
    )
    self.assertIn("ads8688_recovery_pending = 1u;", process_body)

    public_stop = source.split(
        "ads8688_status_t ads8688_stop(void)", 1
    )[1].split("\n}", 1)[0]
    self.assertIn("ads8688_recovery_pending = 1u;", public_stop)
    self.assertIn("ads8688_initialized = 0u;", public_stop)
    self.assertIn("enabled_channel_count++", source)
    self.assertIn(
        "frame_rate_hz / (float)enabled_channel_count",
        source,
    )
```

- [ ] **Step 2: 运行测试并确认 RED**

Run:

```powershell
python -m unittest tests.test_ads8688_input_contract.Ads8688HardwareContract.test_driver_processes_both_pending_halves_without_resync -v
```

Expected: `FAIL`，原因是当前 `ads8688_process()` 仍包含
`if (pending_flags == 3u)`、整缓冲区丢样计数和
`measurement_fft_resynchronize()`。

- [ ] **Step 3: 提交失败测试**

```powershell
git add -- tests/test_ads8688_input_contract.py
git commit -m "test: define ADS8688 dual DMA flag handling"
```

---

### Task 2: 允许两个完整 DMA 半区连续处理

**Files:**
- Modify: `Core/User/ads8688.c:750-859`
- Test: `tests/test_ads8688_input_contract.py`

**Interfaces:**
- Consumes: `ads8688_snapshot_dma_flags()`、`ads8688_claim_flag()`、
  `ads8688_expected_half` 和 `ads8688_process_dma_half(uint8_t half_index)`。
- Produces: 保持签名不变的 `void ads8688_process(void)`。

- [ ] **Step 1: 删除双标志直接恢复分支**

从 `ads8688_process()` 删除以下代码：

```c
    if (pending_flags == 3u)
    {
        (void)ads8688_claim_flag(&ads8688_dma_half_flag);
        (void)ads8688_claim_flag(&ads8688_dma_full_flag);
        ads8688_diagnostics.lost_samples += ADS8688_DMA_WORD_COUNT;
        measurement_fft_resynchronize();
        ads8688_recovery_pending = 1u;
        (void)ads8688_recover();
        return;
    }
```

保留下方已有逻辑。它在 `ads8688_expected_half == 0u` 时先处理前半区再处理
后半区，在 `ads8688_expected_half == 1u` 时先处理后半区再处理前半区。

- [ ] **Step 2: 补充中文行为说明**

将 `ads8688_process()` 的 `@note` 更新为：

```c
 * @note 双半区同时待处理时按期望顺序连续消费；仅在单半区乱序或硬件错误时恢复。
```

- [ ] **Step 3: 运行专项测试并确认 GREEN**

Run:

```powershell
python -m unittest tests.test_ads8688_input_contract -v
```

Expected: `Ran 13 tests`，`OK`。

- [ ] **Step 4: 检查修改差异并提交**

```powershell
git diff --check -- Core/User/ads8688.c tests/test_ads8688_input_contract.py
git add -- Core/User/ads8688.c
git commit -m "fix: process both ADS8688 DMA halves"
```

---

### Task 3: 回归、编译与实板验收

**Files:**
- Verify: `Core/User/ads8688.c`
- Verify: `tests/test_ads8688_input_contract.py`
- Build output: `Debug/h743_pre1.elf`

**Interfaces:**
- Consumes: Task 2 修改后的 `ads8688_process()`。
- Produces: 可烧录 Debug 固件和明确的实板观察条件。

- [ ] **Step 1: 运行全量契约测试**

Run:

```powershell
python -m unittest discover -s tests -v
```

Expected: ADS8688 相关测试全部通过；允许保留用户已确认的
`test_main_user_regions_remain_thin` 失败和主机 C 编译器测试跳过。

- [ ] **Step 2: 使用 CubeIDE 工具链编译**

Run:

```powershell
$toolchain = 'D:\CubeIDE\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344\tools\bin'
$make_dir = 'D:\CubeIDE\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845\tools\bin'
$env:Path = "$toolchain;$make_dir;$env:Path"
& "$make_dir\make.exe" -C Debug -j4 all
```

Expected: exit code `0`，生成 `Debug/h743_pre1.elf`。

- [ ] **Step 3: 检查本次提交范围**

Run:

```powershell
git diff --check HEAD~2..HEAD -- Core/User/ads8688.c tests/test_ads8688_input_contract.py
git log -3 --oneline
```

Expected: 无空白错误；日志包含测试提交和驱动修复提交。

- [ ] **Step 4: 烧录后观察实板状态**

在不暂停 DMA 处理循环的情况下观察：

```text
measurement_fft_capture_index
measurement_fft_diagnostics.window_count
measurement_fft_diagnostics.fft_count
measurement_fft_diagnostics.capture_resync_count
ads8688_diagnostics.lost_samples
ads8688_diagnostics.recoveries
```

Expected: `measurement_fft_capture_index` 在约 283 ms 内到达 65536，
`window_count` 和 `fft_count` 增加；正常采样时
`capture_resync_count`、`lost_samples` 和 `recoveries` 不再按 DMA 周期持续增加。

