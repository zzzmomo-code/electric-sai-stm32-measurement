# DAC 与 VGA 增益控制 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 STM32H743VIT6 上实现 DAC1 经 OPAMP1/PC4 输出六档校准电压，并按档位计算外部 VGA 的 VG 与差分电压增益。

**Architecture:** `dac_output` 用户模块封装电压档位、校准、12 位码值换算、HAL 启停及 VGA 数学模型。CubeMX 继续负责 DAC1、OPAMP1 和 PC4 的底层配置，`system_init()` 只调用用户模块初始化；Python 契约测试验证配置、接口和计算常量，STM32CubeIDE Debug 构建验证目标端集成。

**Tech Stack:** STM32CubeIDE 1.19.0、STM32H7 HAL、C11/GNU C、Python 3 `unittest`、GNU Arm Embedded Toolchain 13.3.rel1。

## Global Constraints

- 目标 MCU 固定为 STM32H743VIT6，外部晶振固定为 25 MHz，PLL1 系统时钟固定为 480 MHz。
- 所有新增用户 `.c/.h` 文件必须位于 `Core/User/`，用户 `.c` 只包含 `system.h`。
- 不修改 `.ioc`、CubeMX 生成的 DAC/OPAMP/时钟初始化代码或 PC4 配置。
- `main.c` 用户初始化区仍然只有 `system_init();`，主循环用户区仍然只有 `system_process();`。
- 六档标称电压固定为 `0、660、1320、1980、2640、3300 mV`，档位编号固定为 `0..5`。
- `RF` 与 `RG` 默认均为 `10000 ohm`；`VG=(20/33)*VDAC-1`；`gain=(1+VG)*RF/RG`。
- 比例校准默认 `1.0f`，偏移校准默认 `0.0f V`；校准结果在换算前钳位到 `0..DAC_OUTPUT_REFERENCE_MV`。
- 所有新增或修改的用户代码最终补齐中文模块说明、函数注释、重要变量注释和 PC4 映射。
- 保留用户当前所有未提交改动；每次提交只暂存本计划列出的文件；未经用户明确许可不得 push。

---

## File Structure

- Create `Core/User/dac_output.h`: 配置宏、状态类型和公开 API。
- Create `Core/User/dac_output.c`: 档位表、校准/钳位/码值换算、HAL 操作及增益计算。
- Create `tests/test_dac_output_contract.py`: CubeMX 配置、时钟、接口、计算结果、集成位置和错误行为的主机端契约测试。
- Modify `Core/User/system.h`: 纳入 CubeMX DAC/OPAMP 句柄和 `dac_output.h`。
- Modify `Core/User/system.c`: 在统一初始化入口调用 `dac_output_init()`，更新模块 GPIO/依赖说明。
- Modify `README.md`: 增加 DAC/OPAMP/PC4、六档电压、校准宏、VGA 公式和验证限制。

---

### Task 1: 建立失败的 DAC/VGA 契约测试

**Files:**
- Create: `tests/test_dac_output_contract.py`

**Interfaces:**
- Consumes: `h743_pre1.ioc`、`Core/Src/main.c`、`Core/Inc/stm32h7xx_hal_conf.h`。
- Produces: 对 `dac_output.h/.c`、`system.h/.c` 和 README 集成的可重复验收契约。

- [ ] **Step 1: 写入配置与时钟契约测试**

在 `tests/test_dac_output_contract.py` 创建 `unittest.TestCase`，使用 UTF-8 读取工程文件，并精确断言：

```python
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def read_text(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")

class DacOutputContractTest(unittest.TestCase):
    def test_cube_configuration_routes_dac_to_opamp_pc4(self) -> None:
        ioc = read_text("h743_pre1.ioc")
        for line in (
            "DAC1.DAC_Channel-DAC_OUT1_Int=DAC_CHANNEL_1",
            "PC4.Mode=Follower-DAC_OUT1-INP",
            "PC4.Signal=OPAMP1_VOUT",
            "VP_DAC1_VS_DACI1.Mode=DAC_OUT1_Int",
        ):
            self.assertIn(line, ioc)

    def test_hse_pll_keeps_480mhz_system_clock(self) -> None:
        main = read_text("Core/Src/main.c")
        hal_conf = read_text("Core/Inc/stm32h7xx_hal_conf.h")
        self.assertIn("#define HSE_VALUE    (25000000UL)", hal_conf)
        self.assertIn("RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;", main)
        self.assertIn("RCC_OscInitStruct.PLL.PLLM = 5;", main)
        self.assertIn("RCC_OscInitStruct.PLL.PLLN = 192;", main)
        self.assertIn("RCC_OscInitStruct.PLL.PLLP = 2;", main)
        self.assertIn("FLASH_LATENCY_4", main)
```

- [ ] **Step 2: 写入驱动 API、宏和集成契约测试**

在同一测试类追加测试，要求头文件存在下列宏与签名，源文件调用 HAL，系统统一初始化模块：

```python
    def test_driver_exposes_levels_calibration_and_gain_api(self) -> None:
        header = read_text("Core/User/dac_output.h")
        for token in (
            "DAC_OUTPUT_LEVEL_COUNT 6u",
            "DAC_OUTPUT_REFERENCE_MV 3300u",
            "DAC_OUTPUT_RF_OHM 10000.0f",
            "DAC_OUTPUT_RG_OHM 10000.0f",
            "DAC_OUTPUT_GAIN_CALIBRATION 1.0f",
            "DAC_OUTPUT_OFFSET_CALIBRATION_V 0.0f",
            "dac_output_init(void)",
            "dac_output_set_level(uint8_t level)",
            "dac_output_get_voltage(uint8_t level, float *voltage_v)",
            "dac_output_get_vg(uint8_t level, float *vg)",
            "dac_output_get_vga_gain(uint8_t level, float *gain)",
        ):
            self.assertIn(token, header)

    def test_driver_starts_opamp_and_dac_and_system_initializes_it(self) -> None:
        source = read_text("Core/User/dac_output.c")
        system_h = read_text("Core/User/system.h")
        system_c = read_text("Core/User/system.c")
        self.assertIn("HAL_OPAMP_Start(&hopamp1)", source)
        self.assertIn("HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1", source)
        self.assertIn("HAL_DAC_Start(&hdac1, DAC_CHANNEL_1)", source)
        self.assertIn('#include "dac_output.h"', system_h)
        self.assertIn("dac_output_init();", system_c)
```

- [ ] **Step 3: 写入六档数学结果和错误处理契约测试**

追加纯数学期望，确保档位电压、码值、VG 与默认 `RF/RG` 增益没有歧义，同时用源码契约要求越界检查先于 HAL 写入：

```python
    def test_six_default_levels_have_expected_codes_vg_and_gain(self) -> None:
        millivolts = (0, 660, 1320, 1980, 2640, 3300)
        expected_codes = (0, 819, 1638, 2457, 3276, 4095)
        for index, mv in enumerate(millivolts):
            code = (mv * 4095 + 1650) // 3300
            vg = (20.0 / 33.0) * (mv / 1000.0) - 1.0
            gain = (1.0 + vg) * (10000.0 / 10000.0)
            self.assertEqual(expected_codes[index], code)
            self.assertAlmostEqual(index * 0.4 - 1.0, vg, places=6)
            self.assertAlmostEqual(index * 0.4, gain, places=6)

    def test_set_level_validates_before_writing_hal(self) -> None:
        source = read_text("Core/User/dac_output.c")
        validation = source.index("level >= DAC_OUTPUT_LEVEL_COUNT")
        write = source.index("HAL_DAC_SetValue")
        self.assertLess(validation, write)

if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 4: 运行测试并确认因驱动缺失而失败**

Run: `python -m unittest tests.test_dac_output_contract -v`

Expected: 配置与时钟测试通过；驱动测试以 `FileNotFoundError: Core/User/dac_output.h` 失败，证明测试在实现前能够捕获缺失功能。

- [ ] **Step 5: 提交失败测试**

```powershell
git add -- tests/test_dac_output_contract.py
git commit -m "test: define DAC VGA output contract"
```

---

### Task 2: 实现 DAC 输出与 VGA 增益模块

**Files:**
- Create: `Core/User/dac_output.h`
- Create: `Core/User/dac_output.c`
- Modify: `Core/User/system.h`
- Modify: `Core/User/system.c`
- Test: `tests/test_dac_output_contract.py`

**Interfaces:**
- Consumes: CubeMX 全局句柄 `hdac1`、`hopamp1`；HAL 的 `HAL_DAC_*`、`HAL_OPAMP_Start`。
- Produces: `dac_output_status_t dac_output_init(void)`、`dac_output_status_t dac_output_set_level(uint8_t)`、`uint8_t dac_output_get_level(void)`、三个带输出指针的查询函数。

- [ ] **Step 1: 创建公开头文件并定义精确配置**

创建带中文模块说明的 `Core/User/dac_output.h`。模块说明明确：用途为六档 DAC/VGA 控制；GPIO 为 PC4/OPAMP1_VOUT；依赖 DAC1 Channel 1 内部连接和 OPAMP1 Follower；由 `system_init()` 初始化。核心声明为：

```c
#ifndef DAC_OUTPUT_H
#define DAC_OUTPUT_H

#include <stdint.h>

#define DAC_OUTPUT_LEVEL_COUNT              6u
#define DAC_OUTPUT_REFERENCE_MV             3300u
#define DAC_OUTPUT_LEVEL_0_MV               0u
#define DAC_OUTPUT_LEVEL_1_MV               660u
#define DAC_OUTPUT_LEVEL_2_MV               1320u
#define DAC_OUTPUT_LEVEL_3_MV               1980u
#define DAC_OUTPUT_LEVEL_4_MV               2640u
#define DAC_OUTPUT_LEVEL_5_MV               3300u
#define DAC_OUTPUT_RF_OHM                   10000.0f
#define DAC_OUTPUT_RG_OHM                   10000.0f
#define DAC_OUTPUT_GAIN_CALIBRATION         1.0f
#define DAC_OUTPUT_OFFSET_CALIBRATION_V     0.0f

typedef enum
{
    dac_output_status_ok = 0,
    dac_output_status_invalid_argument,
    dac_output_status_hal_error
} dac_output_status_t;

dac_output_status_t dac_output_init(void);
dac_output_status_t dac_output_set_level(uint8_t level);
uint8_t dac_output_get_level(void);
dac_output_status_t dac_output_get_voltage(uint8_t level, float *voltage_v);
dac_output_status_t dac_output_get_vg(uint8_t level, float *vg);
dac_output_status_t dac_output_get_vga_gain(uint8_t level, float *gain);

#endif
```

为每个宏、枚举和函数补充中文用途、参数、返回值与副作用说明。

- [ ] **Step 2: 实现纯计算与查询函数**

在只包含 `system.h` 的 `Core/User/dac_output.c` 中定义只读毫伏表和当前档位。使用以下计算顺序：

```c
static const uint16_t dac_output_level_mv[DAC_OUTPUT_LEVEL_COUNT] = {
    DAC_OUTPUT_LEVEL_0_MV, DAC_OUTPUT_LEVEL_1_MV,
    DAC_OUTPUT_LEVEL_2_MV, DAC_OUTPUT_LEVEL_3_MV,
    DAC_OUTPUT_LEVEL_4_MV, DAC_OUTPUT_LEVEL_5_MV
};

static uint8_t dac_output_current_level = 0u;

static float dac_output_calculate_voltage(uint8_t level)
{
    float voltage_v = ((float)dac_output_level_mv[level] / 1000.0f)
                      * DAC_OUTPUT_GAIN_CALIBRATION
                      + DAC_OUTPUT_OFFSET_CALIBRATION_V;
    const float reference_v = (float)DAC_OUTPUT_REFERENCE_MV / 1000.0f;
    if (voltage_v < 0.0f) voltage_v = 0.0f;
    if (voltage_v > reference_v) voltage_v = reference_v;
    return voltage_v;
}
```

三个查询函数先检查 `level >= DAC_OUTPUT_LEVEL_COUNT` 和空指针，再分别计算：

```c
*voltage_v = dac_output_calculate_voltage(level);
*vg = (20.0f / 33.0f) * voltage_v - 1.0f;
*gain = (1.0f + vg) * DAC_OUTPUT_RF_OHM / DAC_OUTPUT_RG_OHM;
```

`dac_output_get_vga_gain()` 必须通过局部变量调用 `dac_output_get_vg()`，并在编译期用 `#if` 或运行前检查保证 `DAC_OUTPUT_RG_OHM > 0.0f`。

- [ ] **Step 3: 实现 12 位码值换算与安全设置**

将校准电压转换为 mV 后按最近整数换算；浮点写法保持清晰并在转换前钳位：

```c
static uint32_t dac_output_voltage_to_code(float voltage_v)
{
    const float reference_v = (float)DAC_OUTPUT_REFERENCE_MV / 1000.0f;
    float code = voltage_v * 4095.0f / reference_v + 0.5f;
    if (code < 0.0f) code = 0.0f;
    if (code > 4095.0f) code = 4095.0f;
    return (uint32_t)code;
}
```

`dac_output_set_level()` 必须先验证档位，再计算电压与码值，调用：

```c
if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, code) != HAL_OK)
{
    return dac_output_status_hal_error;
}
dac_output_current_level = level;
return dac_output_status_ok;
```

HAL 失败时不得修改 `dac_output_current_level`。

- [ ] **Step 4: 实现初始化顺序**

`dac_output_init()` 先启动 OPAMP，再写入 0 档，然后启动 DAC；任何 HAL 失败均返回 `dac_output_status_hal_error`：

```c
if (HAL_OPAMP_Start(&hopamp1) != HAL_OK) return dac_output_status_hal_error;
if (dac_output_set_level(0u) != dac_output_status_ok) return dac_output_status_hal_error;
if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK) return dac_output_status_hal_error;
return dac_output_status_ok;
```

注释说明初始化会将输出目标置为 0 V；不增加中断或主循环处理函数。

- [ ] **Step 5: 接入统一头文件和统一初始化**

在 `Core/User/system.h` 的 CubeMX 可用性区加入 `dac.h`、`opamp.h`，并在用户模块包含区加入：

```c
#include "dac_output.h"
```

在 `Core/User/system.c` 的 `system_init()` 中，其他可能启动采集/输出的模块之前调用：

```c
(void)dac_output_init();
```

更新 `system.c` 顶部中文模块说明，加入 `PC4/OPAMP1_VOUT` 和 DAC1/OPAMP1 依赖。保持 `main.c` 不变。

- [ ] **Step 6: 运行契约测试并修正为全绿**

Run: `python -m unittest tests.test_dac_output_contract -v`

Expected: 所有 DAC 契约测试 PASS；若精确源码契约与格式化后的等价实现不匹配，只调整契约为语义稳定的正则表达式，不降低行为要求。

- [ ] **Step 7: 运行全部主机端回归测试**

Run: `python -m unittest discover -s tests -p "test_*.py" -v`

Expected: 现有 ADC/FFT、DDS、频率测量和新增 DAC 测试全部 PASS。

- [ ] **Step 8: 提交驱动实现**

```powershell
git add -- Core/User/dac_output.h Core/User/dac_output.c Core/User/system.h Core/User/system.c tests/test_dac_output_contract.py
git commit -m "feat: add calibrated DAC VGA gain control"
```

---

### Task 3: 文档、目标构建与可靠性复核

**Files:**
- Modify: `README.md`
- Verify: `Core/Src/main.c`
- Verify: `Core/Src/dac.c`
- Verify: `Core/Src/opamp.c`
- Verify: `h743_pre1.ioc`

**Interfaces:**
- Consumes: Task 2 完成的公开宏和 API。
- Produces: 用户可执行的配置/调用/校准说明，以及构建和静态可靠性证据。

- [ ] **Step 1: 更新 README 的硬件和调用说明**

在 README 对应章节写明：STM32CubeIDE 1.19.0、STM32H743VIT6、25 MHz HSE/480 MHz SYSCLK、DAC1 OUT1 内部连接到 OPAMP1、PC4 输出、六档电压表、默认 `RF=RG=10 kΩ`、两个校准宏、VG/VGA 公式、`dac_output_set_level(0..5)` 调用示例，以及 3.3 V 档受 VDDA/OPAMP 摆幅/负载影响的限制。

- [ ] **Step 2: 执行时钟与生成配置静态复核**

Run:

```powershell
Select-String -Path Core/Src/main.c -Pattern 'RCC_PLLSOURCE_HSE|PLLM = 5|PLLN = 192|PLLP = 2|RCC_HCLK_DIV2|FLASH_LATENCY_4'
Select-String -Path Core/Src/dac.c -Pattern 'DAC_CHIPCONNECT_ENABLE|DAC_TRIGGER_NONE|DAC_CHANNEL_1'
Select-String -Path Core/Src/opamp.c -Pattern 'OPAMP_FOLLOWER_MODE|OPAMP_NONINVERTINGINPUT_DAC_CH|GPIO_PIN_4'
```

Expected: HSE=25 MHz 经 PLL 得到 480 MHz；AHB=240 MHz、各 APB=120 MHz；DAC 内部连接启用；OPAMP 跟随 DAC 并在 PC4 模拟输出。记录 `.ioc` 初始化排序字段中若仍出现 `MX_TIM6_Init` 而非 OPAMP 的异常，但不直接修改 `.ioc`。

- [ ] **Step 3: 运行 STM32 Debug 构建**

先定位 STM32CubeIDE bundled make 与工具链；若当前终端已有工具链，执行：

```powershell
make -C Debug -j4 all
```

Expected: `h743_pre1.elf` 生成，编译/链接返回码为 0，无新增 warning。若 `make` 或 `arm-none-eabi-gcc` 不在 PATH，使用 STM32CubeIDE 1.19.0 安装目录内的对应可执行文件重试，并在交付中记录实际命令。

- [ ] **Step 4: 执行最终规则检查**

Run:

```powershell
python -m unittest discover -s tests -p "test_*.py" -v
git diff --check
git status --short
```

Expected: 全部测试通过；本次新增/修改源文件没有空白错误；`main.c` 用户区结构不变；所有用户模块仍在 `Core/User/`；Git 状态中用户原有 CubeMX/Debug 改动得到保留。

- [ ] **Step 5: 提交 README**

```powershell
git add -- README.md
git commit -m "docs: document DAC VGA output control"
```

- [ ] **Step 6: 汇报硬件未验证项并等待确认**

汇报目标构建与主机测试结果、时钟计算、六档理论值、当前未进行板上测量。要求用户用万用表测量 PC4 每档实际电压，并据此调整 `DAC_OUTPUT_GAIN_CALIBRATION` 与 `DAC_OUTPUT_OFFSET_CALIBRATION_V`；未经明确许可不 push。
