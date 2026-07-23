# Dynamic ADC and ADS8688 Input Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 保留内部双 ADC，并增加可动态切换、支持 ADS8688 单/双通道 FFT 的 SPI3 DMA 采集链路。

**Architecture:** `measurement_input` 统一管理采集源生命周期和 FFT 输入配置；`adc_dual` 与 `ads8688` 分别管理底层硬件；`measurement_fft` 通过运行时 profile 处理不同通道数、采样率、量程和顺序采样时差。

**Tech Stack:** STM32H743 HAL、SPI3 + DMA1、ADC1/ADC2 + TIM2、Cortex-M7 D-Cache、Python `unittest` 源码契约测试、STM32CubeIDE GNU Arm 工具链。

## Global Constraints

- 所有新增用户 `.c/.h` 必须位于 `Core/User/`，用户 `.c` 只包含 `system.h`。
- `main.c` 用户初始化区只调用 `system_init()`，主循环只调用 `system_process()`。
- HAL 回调只能给一个目的明确的 `xxx_flag` 赋值。
- 用户标识符使用小写下划线命名法，交付前补齐中文注释和模块 GPIO/依赖说明。
- 不修改 `.ioc` 或 CubeMX 生成初始化代码；使用用户已生成的 SPI3/DMA/GPIO 配置。
- 默认输入源为内部 ADC；ADS8688 默认 AIN0/AIN1、双极性 ±5.12 V。
- DDS2 迁移 SPI6 不在本次范围；不推送远程仓库。
- 现有未提交改动属于用户，不提交无关文件。

---

### Task 1: Add failing input-selection contracts

**Files:**
- Create: `tests/test_ads8688_input_contract.py`

**Interfaces:**
- Consumes: 已生成的 `.ioc`、SPI3/DMA/GPIO 初始化文件。
- Produces: ADS8688、FFT profile、采集源管理和系统集成的可重复契约。

- [ ] **Step 1: Write the failing tests**

```python
from pathlib import Path
import math
import re
import unittest

ROOT = Path(__file__).resolve().parents[1]

def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")

class Ads8688HardwareContract(unittest.TestCase):
    def test_spi3_and_dma_match_ads8688(self):
        ioc = read("h743_pre1.ioc")
        for line in (
            "PA15\\ (JTDI).Signal=SPI3_NSS",
            "PC10.Signal=SPI3_SCK",
            "PC11.Signal=SPI3_MISO",
            "PC12.Signal=SPI3_MOSI",
            "PD0.GPIO_Label=ADS8688_DAISY",
            "PD1.GPIO_Label=ADS8688_RST",
            "SPI3.DataSize=SPI_DATASIZE_32BIT",
            "SPI3.CLKPhase=SPI_PHASE_2EDGE",
            "SPI3.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_4",
            "Dma.SPI3_RX.1.MemInc=DMA_MINC_ENABLE",
            "Dma.SPI3_TX.2.MemInc=DMA_MINC_DISABLE",
        ):
            self.assertIn(line, ioc)

    def test_ads8688_public_lifecycle_and_channel_api(self):
        header = read("Core/User/ads8688.h")
        for name in (
            "ads8688_start", "ads8688_stop", "ads8688_set_single_channel",
            "ads8688_set_dual_channel", "ads8688_get_effective_sample_rate_hz",
        ):
            self.assertIn(name, header)

    def test_callbacks_only_set_one_ads8688_flag(self):
        source = read("Core/User/ads8688.c")
        expected = (
            ("HAL_SPI_TxRxHalfCpltCallback", "ads8688_dma_half_flag"),
            ("HAL_SPI_TxRxCpltCallback", "ads8688_dma_full_flag"),
            ("HAL_SPI_ErrorCallback", "ads8688_error_flag"),
        )
        for function, flag in expected:
            body = source.split(f"void {function}", 1)[1].split("\n}", 1)[0]
            self.assertEqual(re.findall(r"\b([a-z0-9_]+_flag)\s*=", body), [flag])

class MeasurementInputContract(unittest.TestCase):
    def test_fft_profile_and_single_ingest_exist(self):
        header = read("Core/User/measurement_fft.h")
        for name in (
            "measurement_fft_input_profile_t",
            "measurement_fft_configure_input",
            "measurement_fft_ingest_single",
        ):
            self.assertIn(name, header)

    def test_runtime_rate_math(self):
        self.assertAlmostEqual(16_000_000.0 / 33.0, 484_848.4848, places=3)
        self.assertAlmostEqual(16_000_000.0 / 66.0, 242_424.2424, places=3)
        self.assertAlmostEqual(33.0 / 16_000_000.0, 2.0625e-6, places=12)

    def test_manager_and_system_integration_exist(self):
        header = read("Core/User/measurement_input.h")
        source = read("Core/User/system.c")
        for name in (
            "measurement_input_select",
            "measurement_input_set_ads8688_single_channel",
            "measurement_input_set_ads8688_dual_channel",
            "measurement_input_get_diagnostics",
        ):
            self.assertIn(name, header)
        self.assertIn("measurement_input_init();", source)
        self.assertIn("measurement_input_process();", source)
        self.assertNotIn("adc_dual_init();", source)

    def test_ads8688_is_in_unified_header_and_active_build(self):
        header = read("Core/User/system.h")
        project = read(".cproject")
        self.assertIn('#include "ads8688.h"', header)
        self.assertIn('#include "measurement_input.h"', header)
        self.assertNotIn("User/ads8688.c|User/ads8688_storage.c", project)
```

- [ ] **Step 2: Run tests and verify RED**

Run:

```powershell
python -m unittest tests.test_ads8688_input_contract -v
```

Expected: failures for missing lifecycle APIs, FFT profile, manager module, SPI callbacks, and build inclusion.

- [ ] **Step 3: Commit the failing contract**

```powershell
git add tests/test_ads8688_input_contract.py
git commit -m "test: define dynamic ADS8688 input contract"
```

---

### Task 2: Make measurement_fft runtime-configurable

**Files:**
- Modify: `Core/User/measurement_fft.h`
- Modify: `Core/User/measurement_fft.c`
- Modify: `tests/test_ads8688_input_contract.py`

**Interfaces:**
- Consumes: `measurement_fft_calibration_t`.
- Produces: `measurement_fft_input_profile_t`, `measurement_fft_configure_input()`, `measurement_fft_ingest_single()`.

- [ ] **Step 1: Extend the failing tests for profile validation and phase correction**

Add source-contract assertions:

```python
def test_fft_uses_runtime_rate_and_phase_delay(self):
    source = read("Core/User/measurement_fft.c")
    self.assertIn("measurement_fft_input_profile.sample_rate_hz", source)
    self.assertIn("measurement_fft_input_profile.ch2_delay_seconds", source)
    self.assertIn("measurement_fft_ingest_single", source)
    self.assertIn("MEASUREMENT_FFT_INPUT_SINGLE_CHANNEL", source)
    self.assertIn("secondary_valid_mask = 0u", source)
```

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```powershell
python -m unittest tests.test_ads8688_input_contract.MeasurementInputContract -v
```

Expected: failure because runtime profile implementation is absent.

- [ ] **Step 3: Add the public profile**

Add to `measurement_fft.h`:

```c
typedef enum
{
    MEASUREMENT_FFT_INPUT_SINGLE_CHANNEL = 1,
    MEASUREMENT_FFT_INPUT_DUAL_CHANNEL = 2
} measurement_fft_input_channel_mode_t;

typedef struct
{
    float sample_rate_hz;
    float ch2_delay_seconds;
    measurement_fft_calibration_t calibration[2];
    measurement_fft_input_channel_mode_t channel_mode;
} measurement_fft_input_profile_t;

uint8_t measurement_fft_configure_input(
    const measurement_fft_input_profile_t *profile);

uint8_t measurement_fft_ingest_single(uint16_t raw_code);
```

- [ ] **Step 4: Implement runtime profile behavior**

In `measurement_fft.c`:

```c
static measurement_fft_input_profile_t measurement_fft_input_profile;

uint8_t measurement_fft_configure_input(
    const measurement_fft_input_profile_t *profile)
{
    if ((profile == 0) || !isfinite(profile->sample_rate_hz)
        || (profile->sample_rate_hz <= 0.0f)
        || !isfinite(profile->ch2_delay_seconds)
        || (profile->ch2_delay_seconds < 0.0f)
        || ((profile->channel_mode != MEASUREMENT_FFT_INPUT_SINGLE_CHANNEL)
            && (profile->channel_mode != MEASUREMENT_FFT_INPUT_DUAL_CHANNEL)))
    {
        return 0u;
    }

    measurement_fft_resynchronize();
    measurement_fft_input_profile = *profile;
    measurement_fft_calibration[0] = profile->calibration[0];
    measurement_fft_calibration[1] = profile->calibration[1];
    measurement_fft_update_rate_diagnostics();
    return 1u;
}

uint8_t measurement_fft_ingest_single(uint16_t raw_code)
{
    if (measurement_fft_input_profile.channel_mode
        != MEASUREMENT_FFT_INPUT_SINGLE_CHANNEL)
    {
        return 0u;
    }
    return measurement_fft_store_sample(raw_code, 0u);
}
```

Replace all frequency/bin/Nyquist calculations that use
`MEASUREMENT_FFT_SAMPLE_RATE_HZ` with
`measurement_fft_input_profile.sample_rate_hz`. In single-channel mode skip the
second FFT, clear all secondary result fields and masks, and clear phase validity.
For dual-channel phase apply:

```c
measurement_fft_diagnostics.phase_deg = measurement_fft_wrap_phase(
    measurement_fft_diagnostics.raw_phase_deg
    - 360.0f * measurement_fft_diagnostics.peak_frequency_hz
      * measurement_fft_input_profile.ch2_delay_seconds);
```

- [ ] **Step 5: Run focused and existing FFT tests**

Run:

```powershell
python -m unittest tests.test_ads8688_input_contract tests.test_600ksps_fft_contract -v
```

Expected: all focused and legacy FFT contracts pass.

- [ ] **Step 6: Commit**

```powershell
git add Core/User/measurement_fft.c Core/User/measurement_fft.h tests/test_ads8688_input_contract.py
git commit -m "feat: configure FFT input at runtime"
```

---

### Task 3: Add explicit adc_dual lifecycle

**Files:**
- Modify: `Core/User/adc_dual.h`
- Modify: `Core/User/adc_dual.c`
- Modify: `tests/test_ads8688_input_contract.py`

**Interfaces:**
- Produces: `adc_dual_start()`, `adc_dual_stop()`, idempotent lifecycle.

- [ ] **Step 1: Add failing lifecycle assertions**

```python
def test_internal_adc_has_explicit_lifecycle(self):
    header = read("Core/User/adc_dual.h")
    source = read("Core/User/adc_dual.c")
    self.assertIn("adc_dual_status_t", header)
    self.assertIn("adc_dual_start(void)", header)
    self.assertIn("adc_dual_stop(void)", header)
    self.assertIn("HAL_TIM_Base_Stop(&htim2)", source)
```

- [ ] **Step 2: Run and verify RED**

Run:

```powershell
python -m unittest tests.test_ads8688_input_contract -v
```

Expected: lifecycle assertion fails.

- [ ] **Step 3: Implement lifecycle**

Add:

```c
typedef enum
{
    ADC_DUAL_STATUS_OK = 0,
    ADC_DUAL_STATUS_NOT_READY,
    ADC_DUAL_STATUS_HAL_ERROR
} adc_dual_status_t;

adc_dual_status_t adc_dual_start(void);
adc_dual_status_t adc_dual_stop(void);
```

Refactor calibration/DMA loading into `adc_dual_init()`, leave the module stopped,
and move TIM2 start to `adc_dual_start()`. `adc_dual_stop()` stops TIM2, clears
the three flags in a short critical section, and remains idempotent.

- [ ] **Step 4: Verify**

Run:

```powershell
python -m unittest tests.test_ads8688_input_contract tests.test_600ksps_fft_contract -v
```

Expected: pass.

- [ ] **Step 5: Commit**

```powershell
git add Core/User/adc_dual.c Core/User/adc_dual.h tests/test_ads8688_input_contract.py
git commit -m "refactor: expose internal ADC lifecycle"
```

---

### Task 4: Port ADS8688 to SPI3 circular DMA

**Files:**
- Modify: `Core/User/ads8688.h`
- Modify: `Core/User/ads8688.c`
- Modify: `Core/User/ads8688_storage.h`
- Modify: `Core/User/ads8688_storage.c`
- Modify: `Core/User/system.h`
- Modify: `STM32H743VITX_FLASH.ld`
- Modify: `STM32H743VITX_RAM.ld`
- Modify without committing because it already contains user changes: `.cproject`
- Modify: `tests/test_ads8688_input_contract.py`

**Interfaces:**
- Consumes: `hspi3`, generated ADS8688 pin macros, SPI3 DMA.
- Produces: ADS8688 init/start/stop/process, single/dual selection, sample rate, diagnostics.

- [ ] **Step 1: Run current ADS contract and verify RED**

Run:

```powershell
python -m unittest tests.test_ads8688_input_contract.Ads8688HardwareContract -v
```

Expected: missing APIs/callbacks/build inclusion.

- [ ] **Step 2: Update header and unified includes**

Declare lifecycle, channel mode and diagnostics:

```c
ads8688_status_t ads8688_start(void);
ads8688_status_t ads8688_stop(void);
ads8688_status_t ads8688_set_single_channel(uint8_t channel);
ads8688_status_t ads8688_set_dual_channel(
    uint8_t primary_channel, uint8_t secondary_channel);
float ads8688_get_effective_sample_rate_hz(void);
```

Include `ads8688.h`, `ads8688_storage.h`, and later `measurement_input.h` from
`system.h`; declare the three ADS8688 flags there.

- [ ] **Step 3: Port hardware bindings and frame parsing**

Replace `hspi2`, PD8/PD9 and literal pins with:

```c
&hspi3
ADS8688_DAISY_GPIO_Port, ADS8688_DAISY_Pin
ADS8688_RST_GPIO_Port, ADS8688_RST_Pin
```

Use a 32-bit TX constant with `HAL_SPI_TransmitReceive_DMA()`. Parse conversion
data from bits `[15:0]` of each received 32-bit frame. Use auto sequence for two
channels and manual channel commands for one channel. Track actual channel IDs,
not only alternating array positions.

- [ ] **Step 4: Place buffers in D2 SRAM and maintain D-Cache**

Use:

```c
static uint32_t ads8688_dma_rx[ADS8688_DMA_WORD_COUNT]
    __attribute__((section(".ads8688_dma"), aligned(32)));
static uint32_t ads8688_dma_tx_word
    __attribute__((section(".ads8688_dma"), aligned(32)));
```

Add `.ads8688_dma (NOLOAD)` to both linker scripts targeting D2 SRAM, invalidate
each completed RX half before reading, and clean the TX cache line before DMA.

- [ ] **Step 5: Add callbacks and lifecycle**

```c
void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi3)
    {
        ads8688_dma_half_flag = 1u;
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi3)
    {
        ads8688_dma_full_flag = 1u;
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi3)
    {
        ads8688_error_flag = 1u;
    }
}
```

`ads8688_init()` configures but does not start DMA. `ads8688_start()` and
`ads8688_stop()` are idempotent. `ads8688_process()` claims flags, processes
halves in order, submits single samples or channel pairs, and performs bounded
recovery.

- [ ] **Step 6: Enable active build**

Remove `User/ads8688.c|User/ads8688_storage.c` from both Core source exclusions
in `.cproject`. Do not edit generated Debug make fragments by hand.

- [ ] **Step 7: Verify ADS contracts**

Run:

```powershell
python -m unittest tests.test_ads8688_input_contract -v
```

Expected: ADS hardware/API/callback contracts pass; manager tests may remain red.

- [ ] **Step 8: Commit**

```powershell
git add Core/User/ads8688.c Core/User/ads8688.h Core/User/ads8688_storage.c Core/User/ads8688_storage.h Core/User/system.h STM32H743VITX_FLASH.ld STM32H743VITX_RAM.ld tests/test_ads8688_input_contract.py
git commit -m "feat: drive ADS8688 with SPI3 DMA"
```

---

### Task 5: Implement measurement_input manager and system integration

**Files:**
- Create: `Core/User/measurement_input.h`
- Create: `Core/User/measurement_input.c`
- Modify: `Core/User/system.h`
- Modify: `Core/User/system.c`
- Modify: `tests/test_ads8688_input_contract.py`

**Interfaces:**
- Consumes: both driver lifecycles and FFT runtime profile.
- Produces: transactional source selection, ADS single/dual configuration, diagnostics.

- [ ] **Step 1: Run manager contract and verify RED**

Run:

```powershell
python -m unittest tests.test_ads8688_input_contract.MeasurementInputContract -v
```

Expected: manager file/API assertions fail.

- [ ] **Step 2: Create manager header**

Define:

```c
typedef enum
{
    MEASUREMENT_INPUT_INTERNAL_ADC = 0,
    MEASUREMENT_INPUT_ADS8688
} measurement_input_source_t;

typedef enum
{
    MEASUREMENT_INPUT_STATUS_OK = 0,
    MEASUREMENT_INPUT_STATUS_INVALID_ARGUMENT,
    MEASUREMENT_INPUT_STATUS_STOP_FAILED,
    MEASUREMENT_INPUT_STATUS_START_FAILED,
    MEASUREMENT_INPUT_STATUS_ROLLBACK_FAILED
} measurement_input_status_t;

void measurement_input_init(void);
measurement_input_status_t measurement_input_select(
    measurement_input_source_t source);
measurement_input_source_t measurement_input_get_source(void);
measurement_input_status_t measurement_input_set_ads8688_single_channel(
    uint8_t channel);
measurement_input_status_t measurement_input_set_ads8688_dual_channel(
    uint8_t primary_channel, uint8_t secondary_channel);
void measurement_input_process(void);
uint8_t measurement_input_get_diagnostics(
    measurement_input_diagnostics_t *diagnostics);
```

- [ ] **Step 3: Implement profiles and transactional switching**

Internal profile:

```c
sample_rate_hz = 600000.0f;
ch2_delay_seconds = 0.0f;
channel_mode = MEASUREMENT_FFT_INPUT_DUAL_CHANNEL;
```

ADS dual profile:

```c
sample_rate_hz = ads8688_get_effective_sample_rate_hz();
ch2_delay_seconds = 1.0f / (2.0f * sample_rate_hz);
channel_mode = MEASUREMENT_FFT_INPUT_DUAL_CHANNEL;
```

ADS single profile:

```c
sample_rate_hz = ads8688_get_effective_sample_rate_hz();
ch2_delay_seconds = 0.0f;
channel_mode = MEASUREMENT_FFT_INPUT_SINGLE_CHANNEL;
```

Use source-specific calibrations. ADS ±5.12 V defaults:

```c
volts_per_code = 0.00015625f;
offset_v = -5.12f;
valid = 1u;
```

On any switch/configuration failure, restore the previous driver and FFT profile.

- [ ] **Step 4: Integrate system**

In `system_init()` initialize FFT, then manager; remove direct ADC init and
hard-coded active FFT calibration writes. In `system_process()` replace direct
ADC calls with one `measurement_input_process()` call while retaining frequency,
DDS and HMI sequencing.

- [ ] **Step 5: Verify all tests**

Run:

```powershell
python -m unittest discover -s tests -p "test_*.py" -v
```

Expected: all tests pass, with only the pre-existing host compiler skip allowed.

- [ ] **Step 6: Commit**

```powershell
git add Core/User/measurement_input.c Core/User/measurement_input.h Core/User/system.c Core/User/system.h tests/test_ads8688_input_contract.py
git commit -m "feat: switch measurement input at runtime"
```

---

### Task 6: Documentation and full verification

**Files:**
- Modify: `README.md`
- Modify: any touched `Core/User/*.c` and `Core/User/*.h` comments only

**Interfaces:**
- Produces: complete Chinese documentation and reproducible validation evidence.

- [ ] **Step 1: Update README**

Document:

- internal ADC and ADS8688 input paths;
- SPI3 pin map and CubeMX/DMA/NVIC settings;
- dynamic selection and ADS single/dual APIs;
- default ±5.12 V range and remaining range APIs;
- 600 kSPS, approximately 242.42 kSPS, and approximately 484.85 kSPS profiles;
- sequential-channel phase correction;
- DDS2-to-SPI6 migration remains pending;
- oscilloscope checks for 32 SCLK and FSYNC high time.

- [ ] **Step 2: Complete Chinese comments**

Ensure every changed user function documents purpose, parameters, return value,
and side effects. Ensure module headers state purpose, GPIO mapping, dependencies,
initialization and call method.

- [ ] **Step 3: Run contract tests**

Run:

```powershell
python -m unittest discover -s tests -p "test_*.py" -v
```

Expected: all tests pass; only documented compiler-dependent skip allowed.

- [ ] **Step 4: Regenerate or locate STM32CubeIDE build tools and build**

Locate `stm32cubeidec.exe` or bundled `make.exe` under the installed
STM32CubeIDE 1.19.0 directory. Run a clean Debug build so the IDE regenerates
source lists and includes `measurement_input.c`, `ads8688.c`, and
`ads8688_storage.c`.

Expected: build exit code 0 and no new compiler warnings from touched files.

- [ ] **Step 5: Verify project rules**

Run read-only checks for:

- all user `.c/.h` under `Core/User`;
- `main.c` user regions remain thin;
- callbacks assign only one flag;
- no unrelated files staged;
- diff contains no accidental generated Debug artifacts.

- [ ] **Step 6: Commit documentation**

```powershell
git add README.md Core/User/ads8688.c Core/User/ads8688.h Core/User/ads8688_storage.c Core/User/ads8688_storage.h Core/User/adc_dual.c Core/User/adc_dual.h Core/User/measurement_fft.c Core/User/measurement_fft.h Core/User/measurement_input.c Core/User/measurement_input.h Core/User/system.c Core/User/system.h
git commit -m "docs: explain selectable ADS8688 measurement input"
```

- [ ] **Step 7: Report hardware-only checks**

Report as not yet verified until measured on hardware:

- PA15 FSYNC high time at current 1-cycle inter-data idleness;
- exactly 32 PC10 SCLK pulses per frame;
- ADS8688 register read-back;
- known DC/sine voltage accuracy;
- same-phase dual-input phase compensation;
- repeated live source switching.
