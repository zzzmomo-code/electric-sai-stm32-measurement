# FFT 双通道频率校准 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为双通道 FFT 原始插值频率增加指定分段校准，诊断中同时保存 raw 和校准结果，并向下游发布校准频率。

**Architecture:** `measurement_fft.h` 提供可调校准系数、公开纯函数和四个双通道诊断字段。`measurement_fft.c` 先保存两路 raw 插值频率，再独立调用校准函数；现有 `measurement_result` 与 HMI 接口继续使用原字段名接收校准值。

**Tech Stack:** STM32H743VIT6、GNU11 C、F32 65536 点 FFT、Python `unittest` 合同测试、STM32CubeIDE 1.19.0 ARM GCC 13.3。

## Global Constraints

- 分段公式必须严格为：`f_raw <= 40000` 时 `0.9999807 * f_raw - 0.2414`，否则 `0.99995854 * f_raw - 0.3226`。
- 输入非有限、零或负数返回 `0.0f`；公式结果小于零时钳位到 `0.0f`。
- CH1 和 CH2 独立保存 raw 与校准值，不平均、不互相覆盖。
- `measurement_result_t.frequency_hz`、`secondary_frequency_hz` 及 HMI 接口不改名，内容改为校准后频率。
- 不修改 TIM5 的 `frequency_measure_hz`、FFT 长度、采样率、插值、相位、THD、频谱和电压算法。
- 所有用户代码保持在 `Core/User/`，不修改 CubeMX、ADC、DMA、GPIO 或时钟配置。
- 保留并不暂存当前 `Core/Src/main.c` 中的 `vga_control_set_level(2);` 及 Debug 构建产物变化。
- 未经用户再次明确许可，不推送远端。

## File Structure

- Modify: `tests/test_600ksps_fft_contract.py` — 定义校准公式、raw 命名和双通道数据流合同。
- Modify: `Core/User/measurement_fft.h` — 提供校准宏、公开函数和 raw/校准诊断字段。
- Modify: `Core/User/measurement_fft.c` — 实现纯函数并校准双通道输出。
- Modify: `README.md` — 记录公式、边界、字段含义和下游行为。

---

### Task 1: 用失败测试定义频率校准合同

**Files:**
- Modify: `tests/test_600ksps_fft_contract.py`
- Test: `tests/test_600ksps_fft_contract.py`

**Interfaces:**
- Consumes: 现有 `measurement_fft_diagnostics_t` 和 `measurement_fft_process()` 数据流。
- Produces: 校准宏、`measurement_fft_calibrate_frequency(float)`、四个频率诊断字段的可执行合同。

- [ ] **Step 1: 更新诊断字段合同**

在 `test_existing_diagnostics_fields_remain()` 的字段元组中，将原来的两个频率字段替换为：

```python
            "raw_peak_frequency_hz",
            "peak_frequency_hz",
            "secondary_raw_peak_frequency_hz",
            "secondary_peak_frequency_hz",
```

- [ ] **Step 2: 增加公式和公开接口合同测试**

在 `SourceContractTest` 中加入：

```python
    def test_fft_frequency_calibration_formula_is_exposed(self):
        header = (ROOT / "Core/User/measurement_fft.h").read_text(
            encoding="utf-8"
        )
        source = (ROOT / "Core/User/measurement_fft.c").read_text(
            encoding="utf-8"
        )
        required_header = (
            "#define MEASUREMENT_FFT_FREQUENCY_SPLIT_HZ 40000.0f",
            "#define MEASUREMENT_FFT_LOW_FREQUENCY_GAIN 0.9999807f",
            "#define MEASUREMENT_FFT_LOW_FREQUENCY_OFFSET_HZ (-0.2414f)",
            "#define MEASUREMENT_FFT_HIGH_FREQUENCY_GAIN 0.99995854f",
            "#define MEASUREMENT_FFT_HIGH_FREQUENCY_OFFSET_HZ (-0.3226f)",
            "float measurement_fft_calibrate_frequency(float raw_frequency_hz);",
        )
        for text in required_header:
            self.assertIn(text, header)

        self.assertIn(
            "float measurement_fft_calibrate_frequency(float raw_frequency_hz)",
            source,
        )
        self.assertIn(
            "raw_frequency_hz <= MEASUREMENT_FFT_FREQUENCY_SPLIT_HZ",
            source,
        )
        self.assertIn("MEASUREMENT_FFT_LOW_FREQUENCY_GAIN", source)
        self.assertIn("MEASUREMENT_FFT_HIGH_FREQUENCY_GAIN", source)

    def test_fft_frequency_calibration_math_and_boundary(self):
        def calibrate(raw_frequency_hz):
            if raw_frequency_hz <= 0.0:
                return 0.0
            if raw_frequency_hz <= 40000.0:
                calibrated = 0.9999807 * raw_frequency_hz - 0.2414
            else:
                calibrated = 0.99995854 * raw_frequency_hz - 0.3226
            return max(calibrated, 0.0)

        self.assertEqual(0.0, calibrate(0.0))
        self.assertEqual(0.0, calibrate(-1.0))
        self.assertAlmostEqual(999.7393, calibrate(1000.0), places=4)
        self.assertAlmostEqual(39998.9866, calibrate(40000.0), places=4)
        self.assertAlmostEqual(39999.01895854, calibrate(40001.0), places=5)
```

- [ ] **Step 3: 增加双通道 raw 到校准结果的数据流合同**

```python
    def test_both_fft_channels_preserve_raw_and_publish_calibrated_frequency(self):
        source = (ROOT / "Core/User/measurement_fft.c").read_text(
            encoding="utf-8"
        )
        required_source = (
            "measurement_fft_diagnostics.raw_peak_frequency_hz =",
            "measurement_fft_diagnostics.secondary_raw_peak_frequency_hz =",
            "measurement_fft_calibrate_frequency(\n"
            "                measurement_fft_diagnostics.raw_peak_frequency_hz)",
            "measurement_fft_calibrate_frequency(\n"
            "                measurement_fft_diagnostics.secondary_raw_peak_frequency_hz)",
            "result.frequency_hz = measurement_fft_diagnostics.peak_frequency_hz;",
            "measurement_fft_diagnostics.secondary_peak_frequency_hz;",
        )
        for text in required_source:
            self.assertIn(text, source)

        self.assertGreaterEqual(
            source.count("raw_peak_frequency_hz = 0.0f;"), 2
        )
        self.assertGreaterEqual(
            source.count("peak_frequency_hz = 0.0f;"), 4
        )
```

- [ ] **Step 4: 运行 FFT 合同测试并确认按预期失败**

Run:

```powershell
python -m unittest tests.test_600ksps_fft_contract -v
```

Expected: FAIL；原因是缺少校准宏、公开函数、raw 诊断字段或双通道校准数据流，而不是 Python 导入或语法错误。

- [ ] **Step 5: 提交失败测试**

```powershell
git add -- tests/test_600ksps_fft_contract.py
git commit -m "test: define FFT frequency calibration contract"
```

---

### Task 2: 实现分段校准函数和双通道数据流

**Files:**
- Modify: `Core/User/measurement_fft.h`
- Modify: `Core/User/measurement_fft.c`
- Test: `tests/test_600ksps_fft_contract.py`

**Interfaces:**
- Consumes: `analysis[channel].peak_position`、`measurement_fft_diagnostics.bin_width_hz`。
- Produces: `float measurement_fft_calibrate_frequency(float raw_frequency_hz)`；双通道 raw 与校准诊断字段。

- [ ] **Step 1: 在头文件中增加校准宏**

紧跟 `MEASUREMENT_FFT_SPECTRUM_FLOOR_DB_X10` 后加入：

```c
/** 分段频率校准的切换点，等于此值时使用低频段公式。 */
#define MEASUREMENT_FFT_FREQUENCY_SPLIT_HZ 40000.0f
/** 低频段频率校准增益。 */
#define MEASUREMENT_FFT_LOW_FREQUENCY_GAIN 0.9999807f
/** 低频段频率校准偏置，单位为 Hz。 */
#define MEASUREMENT_FFT_LOW_FREQUENCY_OFFSET_HZ (-0.2414f)
/** 高频段频率校准增益。 */
#define MEASUREMENT_FFT_HIGH_FREQUENCY_GAIN 0.99995854f
/** 高频段频率校准偏置，单位为 Hz。 */
#define MEASUREMENT_FFT_HIGH_FREQUENCY_OFFSET_HZ (-0.3226f)
```

- [ ] **Step 2: 扩充诊断结构并声明公开函数**

把诊断结构的两个频率字段替换为：

```c
    float raw_peak_frequency_hz;        /**< AIN0 未经公式校准的插值主频率，单位为 Hz。 */
    float peak_frequency_hz;            /**< AIN0 经分段公式校准的主频率，单位为 Hz。 */
    float secondary_raw_peak_frequency_hz; /**< AIN1 未经公式校准的插值主频率，单位为 Hz。 */
    float secondary_peak_frequency_hz;  /**< AIN1 经分段公式校准的主频率，单位为 Hz。 */
```

在 `measurement_fft_init()` 声明之前加入：

```c
/**
 * @brief 使用分段线性公式校准 FFT 插值得到的频率。
 * @param raw_frequency_hz 未经本公式校准的 FFT 插值频率，单位为 Hz。
 * @return 校准后的非负频率；输入无效、非正或结果为负时返回 0 Hz。
 * @note 纯数值计算，不访问外设，也不修改模块状态；40000 Hz 使用低频段公式。
 */
float measurement_fft_calibrate_frequency(float raw_frequency_hz);
```

- [ ] **Step 3: 在源文件中实现公开纯函数**

在第一个静态数值辅助函数之前加入：

```c
float measurement_fft_calibrate_frequency(float raw_frequency_hz)
{
    float calibrated_frequency_hz;

    if ((!isfinite(raw_frequency_hz)) || (raw_frequency_hz <= 0.0f))
    {
        return 0.0f;
    }

    if (raw_frequency_hz <= MEASUREMENT_FFT_FREQUENCY_SPLIT_HZ)
    {
        calibrated_frequency_hz =
            MEASUREMENT_FFT_LOW_FREQUENCY_GAIN * raw_frequency_hz
            + MEASUREMENT_FFT_LOW_FREQUENCY_OFFSET_HZ;
    }
    else
    {
        calibrated_frequency_hz =
            MEASUREMENT_FFT_HIGH_FREQUENCY_GAIN * raw_frequency_hz
            + MEASUREMENT_FFT_HIGH_FREQUENCY_OFFSET_HZ;
    }

    return (calibrated_frequency_hz > 0.0f)
               ? calibrated_frequency_hz
               : 0.0f;
}
```

- [ ] **Step 4: 保存双通道 raw 频率并生成校准频率**

把现有两行频率赋值替换为：

```c
        measurement_fft_diagnostics.raw_peak_frequency_hz =
            analysis[0].peak_position * measurement_fft_diagnostics.bin_width_hz;
        measurement_fft_diagnostics.peak_frequency_hz =
            measurement_fft_calibrate_frequency(
                measurement_fft_diagnostics.raw_peak_frequency_hz);
        measurement_fft_diagnostics.secondary_raw_peak_frequency_hz =
            analysis[1].peak_position * measurement_fft_diagnostics.bin_width_hz;
        measurement_fft_diagnostics.secondary_peak_frequency_hz =
            measurement_fft_calibrate_frequency(
                measurement_fft_diagnostics.secondary_raw_peak_frequency_hz);
```

`measurement_fft_publish()` 保持发布 `peak_frequency_hz` 和 `secondary_peak_frequency_hz`，从而让 `measurement_result` 与 HMI 自动获得校准值。

- [ ] **Step 5: 直流通道同时清零 raw 和校准频率**

CH1 直流分支使用：

```c
                    measurement_fft_diagnostics.raw_peak_frequency_hz = 0.0f;
                    measurement_fft_diagnostics.peak_frequency_hz = 0.0f;
```

CH2 直流分支使用：

```c
                    measurement_fft_diagnostics.secondary_raw_peak_frequency_hz =
                        0.0f;
                    measurement_fft_diagnostics.secondary_peak_frequency_hz =
                        0.0f;
```

- [ ] **Step 6: 运行 FFT 合同测试并确认转绿**

Run:

```powershell
python -m unittest tests.test_600ksps_fft_contract -v
```

Expected: `OK`，所有 FFT 合同通过。

- [ ] **Step 7: 执行 ARM GCC 语法检查**

Run:

```powershell
& 'D:\CubeIDE\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344\tools\bin\arm-none-eabi-gcc.exe' `
  -mcpu=cortex-m7 -std=gnu11 -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx `
  -ICore/Inc -ICore/User -IDrivers/STM32H7xx_HAL_Driver/Inc `
  -IDrivers/STM32H7xx_HAL_Driver/Inc/Legacy `
  -IDrivers/CMSIS/Device/ST/STM32H7xx/Include -IDrivers/CMSIS/Include `
  -IDrivers/CMSIS/DSP/Include -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb `
  -Wall -fsyntax-only Core/User/measurement_fft.c
```

Expected: exit code 0，无新增编译错误或警告。

- [ ] **Step 8: 提交实现**

```powershell
git add -- Core/User/measurement_fft.h Core/User/measurement_fft.c
git commit -m "feat: calibrate dual-channel FFT frequency"
```

---

### Task 3: 更新文档并完成回归验证

**Files:**
- Modify: `README.md`
- Modify: `tests/test_600ksps_fft_contract.py`
- Test: `tests/test_600ksps_fft_contract.py`

**Interfaces:**
- Consumes: Task 2 的公开校准函数、raw/校准诊断字段和双通道发布行为。
- Produces: 面向实板调试的公式、边界和字段说明。

- [ ] **Step 1: 先增加 README 合同测试**

在 `SourceContractTest` 中加入：

```python
    def test_readme_documents_fft_frequency_calibration(self):
        readme = (ROOT / "README.md").read_text(encoding="utf-8")
        required = (
            "0.9999807",
            "0.99995854",
            "40000 Hz",
            "raw_peak_frequency_hz",
            "secondary_raw_peak_frequency_hz",
            "measurement_fft_calibrate_frequency",
        )
        for text in required:
            self.assertIn(text, readme)
```

- [ ] **Step 2: 运行 README 合同并确认按预期失败**

Run:

```powershell
python -m unittest tests.test_600ksps_fft_contract.SourceContractTest.test_readme_documents_fft_frequency_calibration -v
```

Expected: FAIL；README 尚未包含频率校准公式或 raw 字段说明。

- [ ] **Step 3: 在 README 中记录公式和数据流**

在 FFT 测量说明中加入：

````markdown
双通道 FFT 三点插值得到的原始频率分别保存在 `raw_peak_frequency_hz` 和 `secondary_raw_peak_frequency_hz`。模块通过 `measurement_fft_calibrate_frequency()` 独立校准两路频率：

```text
f_raw <= 40000 Hz: f_cal = 0.9999807 * f_raw - 0.2414
f_raw >  40000 Hz: f_cal = 0.99995854 * f_raw - 0.3226
```

40000 Hz 使用第一段。`measurement_result` 与 HMI 发布校准结果；raw 字段只供调试和重新标定使用。该公式不影响 TIM5 独立测频。
````

- [ ] **Step 4: 重新运行 FFT 合同并确认转绿**

Run:

```powershell
python -m unittest tests.test_600ksps_fft_contract -v
```

Expected: `OK`。

- [ ] **Step 5: 运行全量合同测试**

Run:

```powershell
python -m unittest discover -s tests -v
```

Expected: FFT 校准相关测试全部通过。当前未提交的 `main.c` 含 `vga_control_set_level(2);`，若 `test_main_user_regions_remain_thin` 因此失败，只报告该已知无关失败，不删除或暂存用户改动。

- [ ] **Step 6: 检查任务差异范围**

Run:

```powershell
git diff --check -- Core/User/measurement_fft.h Core/User/measurement_fft.c README.md tests/test_600ksps_fft_contract.py
git diff -- Core/User/measurement_fft.h Core/User/measurement_fft.c README.md tests/test_600ksps_fft_contract.py
git status --short
```

Expected: 四个任务文件无格式错误；`main.c` 和 Debug 构建产物仍保持用户现状且不在暂存区。

- [ ] **Step 7: 提交 README 和最终合同**

```powershell
git add -- README.md tests/test_600ksps_fft_contract.py
git commit -m "docs: document FFT frequency calibration"
```

- [ ] **Step 8: 核对本地提交且不推送**

Run:

```powershell
git log -6 --oneline --decorate
git show --stat --oneline HEAD
git status --short
```

Expected: 频率校准测试、实现和文档形成三个范围明确的本地提交；远端引用不变。
