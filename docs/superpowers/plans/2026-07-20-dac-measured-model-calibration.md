# DAC 实测模型校准 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 保持六档 DAC 指令及输出码不变，新增 PA4 实测电压配置，并让 VG、AV、VOUT 和诊断模型使用实测值。

**Architecture:** `vga_control.h` 分别保存 DAC 指令电压与 PA4 实测电压。`vga_control_set_level()` 使用前者生成 DAC 码、使用后者计算 VG/AV；`vga_control_gain_from_level()` 只读取实测值。现有公开函数签名、CubeMX 配置和非法档位/HAL 错误行为保持不变。

**Tech Stack:** STM32H743VIT6、STM32 HAL DAC、C11/GNU11、STM32CubeIDE 1.19.0、Python `unittest` 合同测试。

## Global Constraints

- 所有用户代码仍位于 `Core/User/`，不修改 CubeMX 生成的 DAC 初始化代码和 `.ioc`。
- 所有新增自定义标识符使用小写下划线命名法，宏沿用现有大写命名风格。
- 保留 DAC 指令电压 `0.00/0.66/1.32/1.98/2.64/3.30 V`，自动输出码必须仍为 `0/819/1638/2457/3276/4095`。
- PA4 实测电压固定为 `0.023/0.683/1.362/2.040/2.720/3.370 V`，VG、AV、VOUT 模型使用这些值。
- 第 0 档仍输出 DAC 码 0，不尝试用软件消除 23 mV 零点偏置。
- 保留用户当前未提交的无关改动；每次提交只暂存本计划列出的文件。
- 未经用户再次明确许可，不推送远端分支。

## File Structure

- Modify: `tests/test_vga_control_contract.py` — 先定义实测模型与 DAC 码不变的可执行合同。
- Modify: `Core/User/vga_control.h` — 声明六档实测电压宏和新的诊断字段。
- Modify: `Core/User/vga_control.c` — 在两个显式 `switch` 中分离 DAC 指令值与模型实测值。
- Modify: `README.md` — 记录实测表、计算公式、诊断字段和重新校准方法。

---

### Task 1: 用失败测试定义 DAC 指令与实测模型的分离行为

**Files:**
- Modify: `tests/test_vga_control_contract.py`
- Test: `tests/test_vga_control_contract.py`

**Interfaces:**
- Consumes: 现有 `VGA_CONTROL_LEVEL_x_VOLTAGE_V`、`vga_control_set_level()`、`vga_control_gain_from_level()`。
- Produces: 六个 `VGA_CONTROL_LEVEL_x_MEASURED_VOLTAGE_V` 宏和 `measured_voltage_v` 诊断字段的合同。

- [ ] **Step 1: 扩充宏与模型测试，使其表达新需求**

把 `test_module_exposes_six_voltage_levels_and_formulas()` 中的六档检查扩充为：

```python
        command_voltages = ("0.00f", "0.66f", "1.32f", "1.98f", "2.64f", "3.30f")
        measured_voltages = (
            "0.023f",
            "0.683f",
            "1.362f",
            "2.040f",
            "2.720f",
            "3.370f",
        )
        for index, voltage in enumerate(command_voltages):
            self.assertIn(
                f"#define VGA_CONTROL_LEVEL_{index}_VOLTAGE_V {voltage}",
                header,
            )
        for index, voltage in enumerate(measured_voltages):
            self.assertIn(
                f"#define VGA_CONTROL_LEVEL_{index}_MEASURED_VOLTAGE_V {voltage}",
                header,
            )
```

把 `test_six_level_math_matches_the_design()` 替换为：

```python
    def test_dac_codes_stay_original_while_model_uses_measured_voltage(self):
        command_voltages = (0.00, 0.66, 1.32, 1.98, 2.64, 3.30)
        measured_voltages = (0.023, 0.683, 1.362, 2.040, 2.720, 3.370)
        expected_codes = (0, 819, 1638, 2457, 3276, 4095)
        expected_vg = (
            -0.986061,
            -0.586061,
            -0.174545,
            0.236364,
            0.648485,
            1.042424,
        )
        expected_gain = (
            0.013939,
            0.413939,
            0.825455,
            1.236364,
            1.648485,
            2.042424,
        )

        for index, command_voltage in enumerate(command_voltages):
            code = int(command_voltage / 3.30 * ((1 << 12) - 1) + 0.5)
            vg = (20.0 / 33.0) * measured_voltages[index] - 1.0
            gain = (1.0 + vg) * 1.0 / 1.0
            self.assertEqual(expected_codes[index], code)
            self.assertAlmostEqual(expected_vg[index], vg, places=6)
            self.assertAlmostEqual(expected_gain[index], gain, places=6)
```

在 `test_public_api_and_diagnostics_are_declared()` 中增加：

```python
        self.assertIn("float measured_voltage_v;", header)
```

新增数据流合同测试：

```python
    def test_dac_write_uses_command_voltage_and_model_uses_measured_voltage(self):
        source = (ROOT / "Core/User/vga_control.c").read_text(encoding="utf-8")
        set_body = source.split(
            "vga_control_status_t vga_control_set_level", 1
        )[1].split("vga_control_status_t vga_control_gain_from_level", 1)[0]
        gain_body = source.split(
            "vga_control_status_t vga_control_gain_from_level", 1
        )[1]

        self.assertIn(
            "vga_control_voltage_to_dac_code(dac_voltage_v)", set_body
        )
        self.assertIn(
            "VGA_CONTROL_VG_FROM_DAC_VOLTAGE(measured_voltage_v)", set_body
        )
        self.assertIn(
            "vga_control_diagnostics.measured_voltage_v = measured_voltage_v;",
            set_body,
        )
        self.assertIn(
            "VGA_CONTROL_VG_FROM_DAC_VOLTAGE(measured_voltage_v)", gain_body
        )
        for level in range(6):
            measured_macro = (
                f"VGA_CONTROL_LEVEL_{level}_MEASURED_VOLTAGE_V"
            )
            self.assertIn(measured_macro, set_body)
            self.assertIn(measured_macro, gain_body)
```

- [ ] **Step 2: 运行定向测试并确认按预期失败**

Run:

```powershell
python -m unittest tests.test_vga_control_contract -v
```

Expected: FAIL；失败原因应包括缺少 `VGA_CONTROL_LEVEL_0_MEASURED_VOLTAGE_V`、缺少 `float measured_voltage_v;` 或源文件仍使用 `dac_voltage_v` 计算 VG，而不是 Python 导入或语法错误。

- [ ] **Step 3: 提交失败测试**

```powershell
git add -- tests/test_vga_control_contract.py
git commit -m "test: define measured DAC voltage model"
```

---

### Task 2: 实现实测电压宏、诊断字段和 VG/AV 数据流

**Files:**
- Modify: `Core/User/vga_control.h`
- Modify: `Core/User/vga_control.c`
- Test: `tests/test_vga_control_contract.py`

**Interfaces:**
- Consumes: `VGA_CONTROL_VG_FROM_DAC_VOLTAGE(float)`、`VGA_CONTROL_GAIN_FROM_VG(float)` 和现有 HAL DAC 写入接口。
- Produces: `VGA_CONTROL_LEVEL_0_MEASURED_VOLTAGE_V` 至 `VGA_CONTROL_LEVEL_5_MEASURED_VOLTAGE_V`；`vga_control_diagnostics_t.measured_voltage_v`。

- [ ] **Step 1: 在头文件中增加 PA4 实测电压宏**

紧跟六个现有指令电压宏之后加入：

```c
/** 第 0 档 PA4/DAC1_OUT1 对地实测电压，用于 VG 与 AV 模型计算。 */
#define VGA_CONTROL_LEVEL_0_MEASURED_VOLTAGE_V 0.023f
/** 第 1 档 PA4/DAC1_OUT1 对地实测电压，用于 VG 与 AV 模型计算。 */
#define VGA_CONTROL_LEVEL_1_MEASURED_VOLTAGE_V 0.683f
/** 第 2 档 PA4/DAC1_OUT1 对地实测电压，用于 VG 与 AV 模型计算。 */
#define VGA_CONTROL_LEVEL_2_MEASURED_VOLTAGE_V 1.362f
/** 第 3 档 PA4/DAC1_OUT1 对地实测电压，用于 VG 与 AV 模型计算。 */
#define VGA_CONTROL_LEVEL_3_MEASURED_VOLTAGE_V 2.040f
/** 第 4 档 PA4/DAC1_OUT1 对地实测电压，用于 VG 与 AV 模型计算。 */
#define VGA_CONTROL_LEVEL_4_MEASURED_VOLTAGE_V 2.720f
/** 第 5 档 PA4/DAC1_OUT1 对地实测电压，用于 VG 与 AV 模型计算。 */
#define VGA_CONTROL_LEVEL_5_MEASURED_VOLTAGE_V 3.370f
```

把现有六个 `VGA_CONTROL_LEVEL_x_VOLTAGE_V` 的注释统一改成“DAC 指令电压，仅用于自动生成片上 DAC 数字码”，避免与实测值混淆。

- [ ] **Step 2: 扩充诊断结构并保持初始化顺序一致**

在 `vga_control_diagnostics_t` 的 `dac_voltage_v` 后加入：

```c
    float measured_voltage_v;              /**< 当前档位 PA4 实测电压，用于 VG 与 AV 模型。 */
```

把 `vga_control.c` 的全局初始化器改为七个字段顺序：

```c
vga_control_diagnostics_t vga_control_diagnostics = {
    0xffu,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    vga_control_status_dac_error,
    (uint32_t)HAL_ERROR
};
```

- [ ] **Step 3: 在设置函数的 switch 中同时选择指令值与实测值**

在 `vga_control_set_level()` 中加入局部变量：

```c
    /** 当前档位 PA4 的实测电压，仅用于 VG 与 AV 模型计算。 */
    float measured_voltage_v;
```

将 switch 六个有效分支改为以下完整映射，default 分支保持原样：

```c
        case 0u:
            dac_voltage_v = VGA_CONTROL_LEVEL_0_VOLTAGE_V;
            measured_voltage_v = VGA_CONTROL_LEVEL_0_MEASURED_VOLTAGE_V;
            break;
        case 1u:
            dac_voltage_v = VGA_CONTROL_LEVEL_1_VOLTAGE_V;
            measured_voltage_v = VGA_CONTROL_LEVEL_1_MEASURED_VOLTAGE_V;
            break;
        case 2u:
            dac_voltage_v = VGA_CONTROL_LEVEL_2_VOLTAGE_V;
            measured_voltage_v = VGA_CONTROL_LEVEL_2_MEASURED_VOLTAGE_V;
            break;
        case 3u:
            dac_voltage_v = VGA_CONTROL_LEVEL_3_VOLTAGE_V;
            measured_voltage_v = VGA_CONTROL_LEVEL_3_MEASURED_VOLTAGE_V;
            break;
        case 4u:
            dac_voltage_v = VGA_CONTROL_LEVEL_4_VOLTAGE_V;
            measured_voltage_v = VGA_CONTROL_LEVEL_4_MEASURED_VOLTAGE_V;
            break;
        case 5u:
            dac_voltage_v = VGA_CONTROL_LEVEL_5_VOLTAGE_V;
            measured_voltage_v = VGA_CONTROL_LEVEL_5_MEASURED_VOLTAGE_V;
            break;
```

HAL 成功后，保留 `vga_control_voltage_to_dac_code(dac_voltage_v)`，把模型更新改为：

```c
    vg_voltage_v = VGA_CONTROL_VG_FROM_DAC_VOLTAGE(measured_voltage_v);
    gain = VGA_CONTROL_GAIN_FROM_VG(vg_voltage_v);
    vga_control_diagnostics.current_level = level;
    vga_control_diagnostics.dac_voltage_v = dac_voltage_v;
    vga_control_diagnostics.measured_voltage_v = measured_voltage_v;
    vga_control_diagnostics.vg_voltage_v = vg_voltage_v;
    vga_control_diagnostics.vga_gain = gain;
    vga_control_diagnostics.last_status = vga_control_status_ok;
```

- [ ] **Step 4: 让档位增益查询只使用实测值**

把 `vga_control_gain_from_level()` 的 `dac_voltage_v` 局部变量替换为：

```c
    /** 当前档位 PA4 的实测电压，用于 VG 与 AV 模型计算。 */
    float measured_voltage_v;
```

将 switch 六个有效分支改为：

```c
        case 0u:
            measured_voltage_v = VGA_CONTROL_LEVEL_0_MEASURED_VOLTAGE_V;
            break;
        case 1u:
            measured_voltage_v = VGA_CONTROL_LEVEL_1_MEASURED_VOLTAGE_V;
            break;
        case 2u:
            measured_voltage_v = VGA_CONTROL_LEVEL_2_MEASURED_VOLTAGE_V;
            break;
        case 3u:
            measured_voltage_v = VGA_CONTROL_LEVEL_3_MEASURED_VOLTAGE_V;
            break;
        case 4u:
            measured_voltage_v = VGA_CONTROL_LEVEL_4_MEASURED_VOLTAGE_V;
            break;
        case 5u:
            measured_voltage_v = VGA_CONTROL_LEVEL_5_MEASURED_VOLTAGE_V;
            break;
```

函数结尾改为：

```c
    vg_voltage_v = VGA_CONTROL_VG_FROM_DAC_VOLTAGE(measured_voltage_v);
    *gain = VGA_CONTROL_GAIN_FROM_VG(vg_voltage_v);
```

- [ ] **Step 5: 运行定向测试并确认转绿**

Run:

```powershell
python -m unittest tests.test_vga_control_contract -v
```

Expected: `Ran 13 tests`，`OK`。

- [ ] **Step 6: 对修改的用户模块执行 ARM GCC 语法检查**

Run:

```powershell
& 'D:\CubeIDE\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344\tools\bin\arm-none-eabi-gcc.exe' `
  -mcpu=cortex-m7 -std=gnu11 -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx `
  -ICore/Inc -ICore/User -IDrivers/STM32H7xx_HAL_Driver/Inc `
  -IDrivers/STM32H7xx_HAL_Driver/Inc/Legacy `
  -IDrivers/CMSIS/Device/ST/STM32H7xx/Include -IDrivers/CMSIS/Include `
  -IDrivers/CMSIS/DSP/Include -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb `
  -Wall -fsyntax-only Core/User/vga_control.c
```

Expected: exit code 0；不出现新的编译错误或警告。CubeIDE 编辑器可能继续对浮点 `_Static_assert` 显示索引器黄线，该已知 GNU11/严格 C11 差异不属于本任务。

- [ ] **Step 7: 提交模型实现**

```powershell
git add -- Core/User/vga_control.h Core/User/vga_control.c
git commit -m "feat: use measured DAC voltage for VGA model"
```

---

### Task 3: 更新说明文档并完成全量验证

**Files:**
- Modify: `README.md`
- Test: `tests/test_vga_control_contract.py`

**Interfaces:**
- Consumes: Task 2 的六档指令电压、实测电压、VG/AV 结果与诊断字段。
- Produces: 面向硬件调试的实测模型说明和重新校准入口。

- [ ] **Step 1: 先更新 README 合同测试**

在 `test_readme_documents_dac_vga_control()` 的 `required` 中加入：

```python
            "PA4 实测电压",
            "VPA4_MEASURED",
            "measured_voltage_v",
```

- [ ] **Step 2: 运行 README 合同测试并确认按预期失败**

Run:

```powershell
python -m unittest tests.test_vga_control_contract.VgaControlContractTest.test_readme_documents_dac_vga_control -v
```

Expected: FAIL；失败原因是当前 README 缺少 `PA4 实测电压`、`VPA4_MEASURED` 或 `measured_voltage_v`，而不是 Python 导入或语法错误。

- [ ] **Step 3: 更新 README 的 DAC/VGA 表格和公式说明**

把原六档表替换为：

```markdown
| 档位 | DAC 指令电压 | 自动换算的 12 位码 | PA4 实测电压 | VG | 默认 AV |
|---:|---:|---:|---:|---:|---:|
| 0 | 0.00 V | 0 | 0.023 V | -0.986061 V | 0.013939 |
| 1 | 0.66 V | 819 | 0.683 V | -0.586061 V | 0.413939 |
| 2 | 1.32 V | 1638 | 1.362 V | -0.174545 V | 0.825455 |
| 3 | 1.98 V | 2457 | 2.040 V | 0.236364 V | 1.236364 |
| 4 | 2.64 V | 3276 | 2.720 V | 0.648485 V | 1.648485 |
| 5 | 3.30 V | 4095 | 3.370 V | 1.042424 V | 2.042424 |
```

把公式中的 `VDAC` 说明改为 PA4 实测值：

```text
VG = (20 / 33) * VPA4_MEASURED - 1
AV = (1 + VG) * Rf / RG
VOUT = (+VIN - -VIN) * AV
```

明确写明原六档宏只生成 DAC 码，六个 `VGA_CONTROL_LEVEL_x_MEASURED_VOLTAGE_V` 用于 VG、AV、VOUT；重新测量后只更新对应实测宏。

- [ ] **Step 4: 更新 README 的诊断字段**

把诊断说明更新为：

```markdown
- `vga_control_diagnostics.current_level`：最近一次成功写入的 DAC 档位。
- `vga_control_diagnostics.dac_voltage_v`：当前档位用于生成 DAC 码的指令电压。
- `vga_control_diagnostics.measured_voltage_v`：当前档位用于模型计算的 PA4 实测电压。
- `vga_control_diagnostics.vg_voltage_v`：基于 PA4 实测电压计算的 VG。
- `vga_control_diagnostics.vga_gain`：基于 PA4 实测电压计算的 VGA 电压增益 AV。
- `vga_control_diagnostics.last_hal_status`：最近一次 DAC HAL 操作状态。
```

- [ ] **Step 5: 重新运行定向测试并确认转绿**

Run:

```powershell
python -m unittest tests.test_vga_control_contract -v
```

Expected: `Ran 13 tests`，`OK`。

- [ ] **Step 6: 运行全量合同测试**

Run:

```powershell
python -m unittest discover -s tests -v
```

Expected: 所有测试通过，最终输出 `OK`；记录实际测试数量，不假定工作区后来新增的测试数。

- [ ] **Step 7: 检查差异范围与格式**

Run:

```powershell
git diff --check
git diff -- Core/User/vga_control.h Core/User/vga_control.c tests/test_vga_control_contract.py README.md
git status --short
```

Expected: `git diff --check` 无错误；本任务差异仅涉及上述四个文件。状态中已有的其他修改和 Debug 构建产物保持原样，不暂存、不回滚。

- [ ] **Step 8: 提交文档和最终测试合同**

```powershell
git add -- README.md tests/test_vga_control_contract.py
git commit -m "docs: document measured DAC voltage model"
```

- [ ] **Step 9: 核对提交内容且不推送**

Run:

```powershell
git log -5 --oneline --decorate
git show --stat --oneline HEAD
git status --short
```

Expected: 本实现新增三个范围明确的本地提交；`origin/h743_pre1` 不变，当前工作区的无关改动仍保留。
