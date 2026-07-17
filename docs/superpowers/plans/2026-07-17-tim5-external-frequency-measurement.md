# TIM5 External Frequency Measurement Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 使用 PA0/TIM5_CH1 对最高 30 MHz 的外部 0～3.3 V 方波连续计数，由 TIM3 以 1 Hz 置位调度标志，并用 DWT 实际周期差计算独立频率变量。

**Architecture:** CubeMX 生成 TIM3、TIM5、GPIO 和 NVIC 初始化；`Core/User/frequency_measure.c` 独立负责启动定时器、领取 TIM3 标志、读取 TIM5/DWT 快照并计算平均频率。TIM3 HAL 回调只置一个标志，现有 FFT 频率结果和串口屏链路保持不变。

**Tech Stack:** STM32CubeIDE 1.19.0、STM32H743VIT6、STM32 HAL、Cortex-M7 DWT、Python `unittest` 源码契约测试、Git。

## Global Constraints

- 所有新增用户 `.c/.h` 文件必须位于 `Core/User/`，并通过 `Core/User/system.h` 统一包含。
- `main.c` 用户初始化区只能调用 `system_init();`，主循环只能调用功能实现函数。
- TIM3 回调只给 `frequency_measure_flag` 赋值；读取计数器、清标志和计算均在主循环。
- 用户标识符使用小写下划线命名，新增和修改的用户代码交付前补全中文注释。
- 不手工修改 CubeMX 生成的 `MX_*`、MSP、IRQ 初始化代码。
- 保留用户现有工作区改动；每次提交只暂存本任务明确列出的文件。
- 未经用户明确许可不得推送远程仓库。

## File Structure

- Create `Core/User/frequency_measure.h`: 模块公开接口、独立频率结果和 TIM3 共享标志声明。
- Create `Core/User/frequency_measure.c`: 初始化、DWT/TIM5 快照、频率计算、TIM3 HAL 回调。
- Create `tests/test_frequency_measure_contract.py`: CubeMX 配置、回调约束、回绕数学和系统集成契约。
- Modify `Core/User/system.h`: 纳入频率测量模块统一入口。
- Modify `Core/User/system.c`: 在统一初始化和主循环处理中调用新模块。
- Modify `README.md`: 增加 PA0 映射、TIM3/TIM5 配置、模块调用和验证方法。
- Preserve generated changes in `h743_pre1.ioc`, `Core/Inc/tim.h`, `Core/Inc/stm32h7xx_it.h`, `Core/Src/tim.c`, `Core/Src/stm32h7xx_it.c`, `Core/Src/main.c`, and `Core/Src/gpio.c`; verify and commit them without hand editing.

---

### Task 1: Lock Down the CubeMX Timer Configuration

**Files:**
- Create: `tests/test_frequency_measure_contract.py`
- Verify/commit: `h743_pre1.ioc`
- Verify/commit: `Core/Inc/tim.h`
- Verify/commit: `Core/Inc/stm32h7xx_it.h`
- Verify/commit: `Core/Src/tim.c`
- Verify/commit: `Core/Src/stm32h7xx_it.c`
- Verify/commit: `Core/Src/main.c`
- Verify/commit: `Core/Src/gpio.c`

**Interfaces:**
- Consumes: CubeMX 生成的 `htim3`、`htim5`、`MX_TIM3_Init()`、`MX_TIM5_Init()` 和 `TIM3_IRQHandler()`。
- Produces: 受测试保护的 TIM3 1 Hz 配置和 TIM5 TI1FP1 外部计数配置。

- [ ] **Step 1: Write configuration characterization tests**

```python
from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]


class TimerConfigurationTest(unittest.TestCase):
    def test_tim3_generates_one_hz_and_has_low_priority(self):
        timer = (ROOT / "Core/Src/tim.c").read_text(encoding="utf-8")
        self.assertIn("htim3.Init.Prescaler = 23999;", timer)
        self.assertIn("htim3.Init.Period = 9999;", timer)
        self.assertIn("HAL_NVIC_SetPriority(TIM3_IRQn, 6, 0);", timer)
        self.assertEqual(240_000_000 // (23_999 + 1) // (9_999 + 1), 1)

    def test_tim5_counts_pa0_rising_edges_without_filter(self):
        timer = (ROOT / "Core/Src/tim.c").read_text(encoding="utf-8")
        self.assertIn("htim5.Init.Prescaler = 0;", timer)
        self.assertIn("htim5.Init.Period = 4294967295;", timer)
        self.assertIn("TIM_SLAVEMODE_EXTERNAL1", timer)
        self.assertIn("TIM_TS_TI1FP1", timer)
        self.assertIn("TIM_TRIGGERPOLARITY_RISING", timer)
        self.assertIn("sSlaveConfig.TriggerFilter = 0;", timer)
        self.assertIn("GPIO_PIN_0", timer)
        self.assertIn("GPIO_AF2_TIM5", timer)

    def test_generated_main_initializes_timers_before_system(self):
        main = (ROOT / "Core/Src/main.c").read_text(encoding="utf-8")
        order = [
            main.index("MX_TIM3_Init();"),
            main.index("MX_TIM5_Init();"),
            main.index("system_init();"),
        ]
        self.assertEqual(order, sorted(order))
```

- [ ] **Step 2: Run the generated-configuration tests**

Run: `python -m unittest tests.test_frequency_measure_contract.TimerConfigurationTest -v`

Expected: 3 tests pass. If any fail, stop and correct the setting in STM32CubeIDE Device Configuration Tool, regenerate, and rerun; do not patch generated initialization by hand.

- [ ] **Step 3: Check generated diffs for unrelated changes**

Run:

```powershell
git diff -- h743_pre1.ioc Core/Inc/tim.h Core/Inc/stm32h7xx_it.h Core/Src/tim.c Core/Src/stm32h7xx_it.c Core/Src/main.c Core/Src/gpio.c
```

Expected: only TIM3, TIM5, PA0, NVIC, generated function ordering, and harmless GPIO clock ordering changes. Do not stage `.cproject` or `.settings/language.settings.xml` because they contain unrelated IDE-local changes.

- [ ] **Step 4: Commit generated configuration and its contract**

```powershell
git add -- tests/test_frequency_measure_contract.py h743_pre1.ioc Core/Inc/tim.h Core/Inc/stm32h7xx_it.h Core/Src/tim.c Core/Src/stm32h7xx_it.c Core/Src/main.c Core/Src/gpio.c
git commit -m "config: add TIM5 external counter and TIM3 scheduler"
```

Expected: commit succeeds and `.cproject` plus `.settings/language.settings.xml` remain unstaged.

---

### Task 2: Implement the Frequency Measurement Module with TDD

**Files:**
- Modify: `tests/test_frequency_measure_contract.py`
- Create: `Core/User/frequency_measure.h`
- Create: `Core/User/frequency_measure.c`

**Interfaces:**
- Consumes: `extern TIM_HandleTypeDef htim3`, `extern TIM_HandleTypeDef htim5`, `SystemCoreClock`, `DWT`, `HAL_TIM_Base_Start()`, and `HAL_TIM_Base_Start_IT()` from HAL/generated headers.
- Produces: `void frequency_measure_init(void)`, `void frequency_measure_process(void)`, `volatile uint8_t frequency_measure_flag`, and `volatile float frequency_measure_hz`.

- [ ] **Step 1: Add failing module and arithmetic contract tests**

Append to `tests/test_frequency_measure_contract.py`:

```python
class FrequencyModuleTest(unittest.TestCase):
    def test_unsigned_delta_math_handles_wrap_and_30mhz(self):
        previous_counter = 0xFFF0_0000
        counter_delta = 30_000_000
        current_counter = (previous_counter + counter_delta) & 0xFFFF_FFFF
        measured_delta = (current_counter - previous_counter) & 0xFFFF_FFFF
        cycle_delta = 480_000_000
        frequency_hz = measured_delta * 480_000_000.0 / cycle_delta
        self.assertEqual(measured_delta, counter_delta)
        self.assertAlmostEqual(frequency_hz, 30_000_000.0)

    def test_module_exposes_independent_result_and_process_api(self):
        header = (ROOT / "Core/User/frequency_measure.h").read_text(encoding="utf-8")
        self.assertIn("void frequency_measure_init(void);", header)
        self.assertIn("void frequency_measure_process(void);", header)
        self.assertIn("extern volatile uint8_t frequency_measure_flag;", header)
        self.assertIn("extern volatile float frequency_measure_hz;", header)

    def test_tim3_callback_only_assigns_its_flag(self):
        source = (ROOT / "Core/User/frequency_measure.c").read_text(encoding="utf-8")
        body = source.split("void HAL_TIM_PeriodElapsedCallback", 1)[1].split("\n}", 1)[0]
        assigned_flags = re.findall(r"\b([a-z0-9_]+_flag)\s*=", body)
        self.assertEqual(assigned_flags, ["frequency_measure_flag"])
        self.assertNotIn("frequency_measure_process();", body)
        self.assertNotIn("__HAL_TIM_GET_COUNTER", body)

    def test_runtime_uses_counter_and_dwt_deltas_without_resetting_dwt(self):
        source = (ROOT / "Core/User/frequency_measure.c").read_text(encoding="utf-8")
        self.assertIn("__HAL_TIM_GET_COUNTER(&htim5)", source)
        self.assertIn("DWT->CYCCNT", source)
        self.assertIn("SystemCoreClock", source)
        self.assertNotIn("DWT->CYCCNT = 0", source)
```

- [ ] **Step 2: Run tests to verify the module contract fails**

Run: `python -m unittest tests.test_frequency_measure_contract.FrequencyModuleTest -v`

Expected: arithmetic test passes; module tests error with `FileNotFoundError` for `Core/User/frequency_measure.h` or `frequency_measure.c`.

- [ ] **Step 3: Create the public header**

Create `Core/User/frequency_measure.h` with include guards, `#include <stdint.h>`, full Chinese module header, and these declarations:

```c
#ifndef FREQUENCY_MEASURE_H
#define FREQUENCY_MEASURE_H

#include <stdint.h>

extern volatile uint8_t frequency_measure_flag;
extern volatile float frequency_measure_hz;

void frequency_measure_init(void);
void frequency_measure_process(void);

#endif /* FREQUENCY_MEASURE_H */
```

The module header must state: purpose; `PA0/TIM5_CH1` GPIO; dependency on TIM5 external clock mode 1, TIM3 1 Hz interrupt, and DWT; initialization through `system_init()`; processing through `system_process()`.

- [ ] **Step 4: Implement initialization, processing, and callback**

Create `Core/User/frequency_measure.c` with Chinese function/variable comments and this minimal logic:

```c
#include "system.h"

volatile uint8_t frequency_measure_flag = 0u;
volatile float frequency_measure_hz = 0.0f;

static uint32_t frequency_measure_previous_counter = 0u;
static uint32_t frequency_measure_previous_cycles = 0u;
static uint8_t frequency_measure_ready = 0u;

void frequency_measure_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    frequency_measure_flag = 0u;
    frequency_measure_hz = 0.0f;
    frequency_measure_ready = 0u;
    __HAL_TIM_SET_COUNTER(&htim5, 0u);

    if (HAL_TIM_Base_Start(&htim5) != HAL_OK)
    {
        return;
    }
    if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
    {
        return;
    }

    frequency_measure_previous_counter = __HAL_TIM_GET_COUNTER(&htim5);
    frequency_measure_previous_cycles = DWT->CYCCNT;
    frequency_measure_ready = 1u;
}

void frequency_measure_process(void)
{
    uint32_t current_counter;
    uint32_t current_cycles;
    uint32_t counter_delta;
    uint32_t cycle_delta;

    if ((frequency_measure_flag == 0u) || (frequency_measure_ready == 0u))
    {
        return;
    }

    frequency_measure_flag = 0u;
    current_counter = __HAL_TIM_GET_COUNTER(&htim5);
    current_cycles = DWT->CYCCNT;
    counter_delta = current_counter - frequency_measure_previous_counter;
    cycle_delta = current_cycles - frequency_measure_previous_cycles;

    if (cycle_delta != 0u)
    {
        frequency_measure_hz = (float)counter_delta
                             * ((float)SystemCoreClock / (float)cycle_delta);
    }

    frequency_measure_previous_counter = current_counter;
    frequency_measure_previous_cycles = current_cycles;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        frequency_measure_flag = 1u;
    }
}
```

- [ ] **Step 5: Run module tests**

Run: `python -m unittest tests.test_frequency_measure_contract.FrequencyModuleTest -v`

Expected: 4 tests pass.

- [ ] **Step 6: Commit the isolated module**

```powershell
git add -- Core/User/frequency_measure.c Core/User/frequency_measure.h tests/test_frequency_measure_contract.py
git commit -m "feat: add TIM5 frequency measurement module"
```

Expected: commit succeeds without staging unrelated IDE files.

---

### Task 3: Integrate the Module and Document Its Use

**Files:**
- Modify: `tests/test_frequency_measure_contract.py`
- Modify: `Core/User/system.h`
- Modify: `Core/User/system.c`
- Modify: `README.md`

**Interfaces:**
- Consumes: `frequency_measure_init()` and `frequency_measure_process()` from Task 2.
- Produces: automatic module initialization/processing through existing `system_init()` and `system_process()`; documented hardware and debug interface.

- [ ] **Step 1: Add failing system-integration tests**

Append to `tests/test_frequency_measure_contract.py`:

```python
class SystemIntegrationTest(unittest.TestCase):
    def test_system_header_is_the_unified_entry(self):
        header = (ROOT / "Core/User/system.h").read_text(encoding="utf-8")
        self.assertIn('#include "frequency_measure.h"', header)

    def test_system_initializes_and_processes_frequency_module(self):
        source = (ROOT / "Core/User/system.c").read_text(encoding="utf-8")
        init_body = source.split("void system_init(void)", 1)[1].split("\n}", 1)[0]
        process_body = source.split("void system_process(void)", 1)[1].split("\n}", 1)[0]
        self.assertIn("frequency_measure_init();", init_body)
        self.assertIn("frequency_measure_process();", process_body)

    def test_external_frequency_does_not_replace_fft_result(self):
        source = (ROOT / "Core/User/frequency_measure.c").read_text(encoding="utf-8")
        self.assertNotIn("measurement_result_publish", source)
        self.assertNotIn("measurement_result_t", source)
```

- [ ] **Step 2: Run integration tests to verify they fail**

Run: `python -m unittest tests.test_frequency_measure_contract.SystemIntegrationTest -v`

Expected: the independence test passes; header and system-call tests fail because integration is absent.

- [ ] **Step 3: Integrate through the unified system entry**

In `Core/User/system.h`, add alongside the other user module headers:

```c
#include "frequency_measure.h"
```

In `Core/User/system.c`, call the module after `measurement_fft_init()` so the existing DWT setup occurs first:

```c
measurement_fft_init();
frequency_measure_init();
```

In `system_process()`, add one function call near the beginning:

```c
frequency_measure_process();
```

Update the affected Chinese function comments to mention external frequency processing and PA0/TIM5/TIM3 dependencies.

- [ ] **Step 4: Update README with exact operating information**

Add the following facts to the existing relevant sections, preserving the current README organization:

```text
PA0：TIM5_CH1，0～3.3 V CMOS 外部频率输入，最高 30 MHz。
TIM5：External Clock Mode 1，TI1FP1，上升沿，PSC=0，ARR=0xFFFFFFFF，Filter=0。
TIM3：240 MHz / 24000 / 10000 = 1 Hz，NVIC 优先级 6。
frequency_measure_init() 由 system_init() 调用；frequency_measure_process() 由 system_process() 调用。
frequency_measure_hz 是独立调试变量，不替换 FFT 频率结果。
频率按 TIM5 计数差和 DWT 实际周期差计算；TIM3 只提供约 1 Hz 更新调度。
板级验证点：0 Hz、1 Hz、1 kHz、1 MHz、10 MHz、30 MHz。
```

- [ ] **Step 5: Run integration and full Python tests**

Run:

```powershell
python -m unittest tests.test_frequency_measure_contract.SystemIntegrationTest -v
python -m unittest discover -s tests -p 'test_*.py' -v
```

Expected: integration tests pass; complete suite reports `OK` with no failures or errors.

- [ ] **Step 6: Commit integration and documentation**

```powershell
git add -- Core/User/system.h Core/User/system.c README.md tests/test_frequency_measure_contract.py
git commit -m "docs: integrate external frequency measurement"
```

Expected: commit succeeds without staging `.cproject` or `.settings/language.settings.xml`.

---

### Task 4: Build and Final Verification

**Files:**
- Verify: all files changed in Tasks 1–3
- Do not modify generated code to conceal configuration or build errors

**Interfaces:**
- Consumes: complete TIM3/TIM5 frequency measurement feature.
- Produces: evidence-backed verification report and clean feature commits.

- [ ] **Step 1: Run source-policy checks**

Run:

```powershell
rg -n "HAL_TIM_PeriodElapsedCallback|frequency_measure_flag|frequency_measure_hz|frequency_measure_(init|process)" Core/User Core/Src/main.c
git diff --check
git status --short
```

Expected: callback exists only once; its only `_flag` assignment is `frequency_measure_flag = 1u`; APIs are integrated through `system.c`; no whitespace errors; only known unrelated IDE-local changes remain.

- [ ] **Step 2: Run the complete offline test suite again**

Run: `python -m unittest discover -s tests -p 'test_*.py' -v`

Expected: all tests pass and final line is `OK`.

- [ ] **Step 3: Perform STM32CubeIDE Clean Build**

In STM32CubeIDE 1.19.0 select `Project > Clean...`, clean `h743_pre1`, then select `Project > Build Project` using the existing H743 Debug configuration.

Expected: build completes with 0 errors. Record the warning count exactly. If CLI build tooling is discoverable in the existing environment, the same generated build may be invoked non-interactively; otherwise report that this step requires the IDE.

- [ ] **Step 4: Inspect final diff and commit history**

Run:

```powershell
git status --short
git log -5 --oneline
git diff --stat HEAD~3..HEAD
```

Expected: three implementation commits cover configuration, module, and integration; `.cproject` and `.settings/language.settings.xml` remain preserved if still modified; no feature file is left unstaged.

- [ ] **Step 5: Report board verification still required**

Report that static tests and compilation do not verify 30 MHz signal integrity or absolute oscillator accuracy. Request board-level comparison at 0 Hz, 1 Hz, 1 kHz, 1 MHz, 10 MHz, and 30 MHz for at least 60 seconds, including observation across a TIM5 wrap if practical. Do not push unless the user explicitly authorizes it.
