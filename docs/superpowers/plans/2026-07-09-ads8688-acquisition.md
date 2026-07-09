# ADS8688 High-Speed Acquisition Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement ADS8688 initialization, automatic eight-channel scanning, manual single-channel acquisition, conversion storage, and approximately 393 kSPS circular-DMA acquisition on STM32H750VBT6.

**Architecture:** SPI2 transmits one 32-bit NO_OP frame per conversion with hardware NSS pulses while two circular DMA streams move repeated TX words and received frames. Interrupt callbacks only set flags; `ads8688_process()` parses completed DMA halves, updates per-channel latest values, and appends records to a 4096-entry ring buffer.

**Tech Stack:** STM32CubeIDE 1.19.0, STM32Cube FW_H7 V1.12.1 HAL, STM32H750VBT6, SPI2, DMA1, C11, Python `unittest`, Git.

## Global Constraints

- All user-authored `.c` and `.h` files live under `Core/User/`.
- `Core/User/system.h` is the only user header included by `main.c` and other user `.c` files.
- Every user `.c` file includes only `system.h`; module headers are collected by `system.h`.
- `main.c` user initialization contains only `system_init();`.
- `main.c` user loop contains only functional calls, specifically `ads8688_process();`.
- Each interrupt callback assigns only one purpose-specific `xxx_flag`; parsing, recovery, and flag clearing occur in `ads8688_process()`.
- User-defined identifiers use lowercase snake case.
- Every changed user module and function receives the required Chinese documentation before delivery.
- Do not modify `.ioc` or generated peripheral initialization settings during implementation.
- Preserve SPI2 at 16.125 MHz, 32-bit, CPHA second edge, hardware NSS, 9-cycle inter-data idleness, and circular RX/TX DMA.
- Do not push to a remote repository without explicit user permission.

---

## File Map

- Create `Core/User/system.h`: sole user-header entry point and cross-module extern declarations.
- Create `Core/User/system.c`: `system_init()` and user-module initialization order.
- Create `Core/User/ads8688.h`: public types, status codes, modes, ranges, diagnostics, and APIs.
- Create `Core/User/ads8688.c`: ADS8688 transport, register access, initialization, mode changes, DMA processing, recovery, and HAL callbacks.
- Create `Core/User/ads8688_storage.h`: internal storage interface used by the driver.
- Create `Core/User/ads8688_storage.c`: latest-channel storage and 4096-entry ring buffer.
- Modify `Core/Src/main.c`: include `system.h`, call `system_init()`, and call `ads8688_process()`.
- Create `tests/test_project_contract.py`: automated checks for file placement, public contracts, callback restrictions, and `main.c` structure.
- Create `README.md`: hardware mapping, CubeIDE settings, APIs, build/run instructions, limitations, and validation procedure.

### Task 1: Lock Down Project and API Contracts

**Files:**
- Create: `tests/test_project_contract.py`
- Create: `Core/User/ads8688.h`
- Create: `Core/User/ads8688_storage.h`
- Create: `Core/User/system.h`

**Interfaces:**
- Produces: `ads8688_status_t`, `ads8688_mode_t`, `ads8688_range_t`, `ads8688_sample_t`, `ads8688_latest_t`, `ads8688_diagnostics_t`.
- Produces: `ads8688_init()`, `ads8688_process()`, `ads8688_set_auto_mode()`, `ads8688_set_manual_mode()`, `ads8688_set_channel_range()`, `ads8688_get_latest()`, `ads8688_read_history()`, `ads8688_clear_history()`, and `system_init()`.

- [ ] **Step 1: Write the failing contract test**

Create `tests/test_project_contract.py` with `unittest` checks that:

```python
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[1]

class ProjectContractTests(unittest.TestCase):
    def test_required_user_headers_exist(self):
        for name in ("system.h", "ads8688.h", "ads8688_storage.h"):
            self.assertTrue((ROOT / "Core" / "User" / name).is_file())

    def test_public_ads8688_api_is_declared(self):
        text = (ROOT / "Core" / "User" / "ads8688.h").read_text(encoding="utf-8")
        for symbol in (
            "ads8688_init", "ads8688_process", "ads8688_set_auto_mode",
            "ads8688_set_manual_mode", "ads8688_set_channel_range",
            "ads8688_get_latest", "ads8688_read_history",
            "ads8688_clear_history",
        ):
            self.assertRegex(text, rf"\b{symbol}\s*\(")

    def test_sample_record_contract(self):
        text = (ROOT / "Core" / "User" / "ads8688.h").read_text(encoding="utf-8")
        for field in ("sample_index", "raw_code", "channel", "reserved"):
            self.assertIn(field, text)

if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the contract test and verify failure**

Run:

```powershell
python -m unittest tests.test_project_contract -v
```

Expected: FAIL because `Core/User/ads8688.h` and the other user headers do not exist.

- [ ] **Step 3: Add the public contracts**

Define the exact public values in `Core/User/ads8688.h`:

```c
typedef enum
{
    ADS8688_STATUS_OK = 0,
    ADS8688_STATUS_INVALID_ARGUMENT,
    ADS8688_STATUS_NOT_INITIALIZED,
    ADS8688_STATUS_HAL_ERROR,
    ADS8688_STATUS_VERIFY_ERROR
} ads8688_status_t;

typedef enum
{
    ADS8688_MODE_AUTO = 0,
    ADS8688_MODE_MANUAL
} ads8688_mode_t;

typedef enum
{
    ADS8688_RANGE_BIPOLAR_10V24 = 0x00,
    ADS8688_RANGE_BIPOLAR_5V12 = 0x01,
    ADS8688_RANGE_BIPOLAR_2V56 = 0x02,
    ADS8688_RANGE_UNIPOLAR_10V24 = 0x05,
    ADS8688_RANGE_UNIPOLAR_5V12 = 0x06
} ads8688_range_t;

typedef struct
{
    uint32_t sample_index;
    uint16_t raw_code;
    uint8_t channel;
    uint8_t reserved;
} ads8688_sample_t;
```

Declare `system_init(void)` in `system.h`, include `main.h`, `spi.h`, `ads8688.h`, and `ads8688_storage.h`, and use normal header guards.

- [ ] **Step 4: Run the contract test**

Run:

```powershell
python -m unittest tests.test_project_contract -v
```

Expected: PASS.

- [ ] **Step 5: Commit the contracts**

```powershell
git add Core/User tests/test_project_contract.py
git commit -m "feat: define ADS8688 module contracts"
```

### Task 2: Implement Storage and Conversion Logic

**Files:**
- Modify: `Core/User/ads8688.h`
- Modify: `Core/User/ads8688_storage.h`
- Create: `Core/User/ads8688_storage.c`
- Modify: `tests/test_project_contract.py`

**Interfaces:**
- Consumes: `ads8688_sample_t`, `ads8688_latest_t`.
- Produces: `void ads8688_storage_init(void)`.
- Produces: `void ads8688_storage_push(uint8_t channel, uint16_t raw_code, ads8688_range_t range, uint32_t sample_index)`.
- Produces: `ads8688_status_t ads8688_storage_get_latest(uint8_t channel, ads8688_latest_t *latest)`.
- Produces: `uint32_t ads8688_storage_read(ads8688_sample_t *samples, uint32_t max_count)`.
- Produces: `void ads8688_storage_clear(void)`.

- [ ] **Step 1: Add failing storage checks**

Add checks for:

```python
def test_storage_capacity_and_channel_count(self):
    text = (ROOT / "Core" / "User" / "ads8688_storage.h").read_text(encoding="utf-8")
    self.assertRegex(text, r"#define\s+ADS8688_HISTORY_CAPACITY\s+4096u")
    self.assertRegex(text, r"#define\s+ADS8688_CHANNEL_COUNT\s+8u")

def test_storage_api_is_declared(self):
    text = (ROOT / "Core" / "User" / "ads8688_storage.h").read_text(encoding="utf-8")
    for symbol in ("ads8688_storage_init", "ads8688_storage_push",
                   "ads8688_storage_get_latest", "ads8688_storage_read",
                   "ads8688_storage_clear"):
        self.assertRegex(text, rf"\b{symbol}\s*\(")
```

Run the test and expect failure because the constants and implementation are absent.

- [ ] **Step 2: Implement the ring buffer**

Use these invariants in `ads8688_storage.c`:

```c
static ads8688_sample_t ads8688_history[ADS8688_HISTORY_CAPACITY];
static ads8688_latest_t ads8688_latest[ADS8688_CHANNEL_COUNT];
static uint32_t ads8688_history_head;
static uint32_t ads8688_history_count;
static uint32_t ads8688_history_overwrite_count;

static uint32_t ads8688_next_index(uint32_t index)
{
    return (index + 1u) % ADS8688_HISTORY_CAPACITY;
}
```

`ads8688_storage_push()` writes at `head`, advances `head`, increments `count` until full, then increments the overwrite counter. `ads8688_storage_read()` copies oldest-first and removes the returned records.

- [ ] **Step 3: Implement voltage conversion**

Use `VREF = 4.096f` and straight-binary conversion:

```c
static float ads8688_code_to_voltage(uint16_t raw_code,
                                     ads8688_range_t range)
{
    switch (range)
    {
        case ADS8688_RANGE_BIPOLAR_10V24:
            return ((float)raw_code * 20.48f / 65536.0f) - 10.24f;
        case ADS8688_RANGE_BIPOLAR_5V12:
            return ((float)raw_code * 10.24f / 65536.0f) - 5.12f;
        case ADS8688_RANGE_BIPOLAR_2V56:
            return ((float)raw_code * 5.12f / 65536.0f) - 2.56f;
        case ADS8688_RANGE_UNIPOLAR_10V24:
            return (float)raw_code * 10.24f / 65536.0f;
        case ADS8688_RANGE_UNIPOLAR_5V12:
            return (float)raw_code * 5.12f / 65536.0f;
        default:
            return 0.0f;
    }
}
```

- [ ] **Step 4: Run contract checks and ARM syntax compilation**

Run:

```powershell
python -m unittest tests.test_project_contract -v
& 'D:\CubeIDE\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344\tools\bin\arm-none-eabi-gcc.exe' -fsyntax-only -std=gnu11 -DSTM32H750xx -DUSE_HAL_DRIVER -ICore/User -ICore/Inc -IDrivers/STM32H7xx_HAL_Driver/Inc -IDrivers/STM32H7xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32H7xx/Include -IDrivers/CMSIS/Include Core/User/ads8688_storage.c
```

Expected: tests PASS and compiler exits 0.

- [ ] **Step 5: Commit storage**

```powershell
git add Core/User/ads8688.h Core/User/ads8688_storage.h Core/User/ads8688_storage.c tests/test_project_contract.py
git commit -m "feat: add ADS8688 sample storage"
```

### Task 3: Implement ADS8688 Register Transport and Initialization

**Files:**
- Create: `Core/User/ads8688.c`
- Modify: `Core/User/ads8688.h`
- Modify: `tests/test_project_contract.py`

**Interfaces:**
- Consumes: generated `hspi2`, PD8 RST/PD, PD9 DAISY.
- Produces: verified program-register reads and writes, `ads8688_init()`, range state, mode state, and diagnostic state.

- [ ] **Step 1: Add failing protocol constant checks**

Check the driver declares these exact values:

```python
expected = {
    "ADS8688_COMMAND_NO_OP": "0x0000u",
    "ADS8688_COMMAND_AUTO_RST": "0xa000u",
    "ADS8688_REGISTER_AUTO_SEQUENCE": "0x01u",
    "ADS8688_REGISTER_FEATURE_SELECT": "0x03u",
    "ADS8688_REGISTER_RANGE_CH0": "0x05u",
}
text = (ROOT / "Core" / "User" / "ads8688.c").read_text(encoding="utf-8")
for name, value in expected.items():
    self.assertRegex(text.lower(), rf"{name.lower()}\s+{value}")
```

Run tests and expect failure because `ads8688.c` is absent.

- [ ] **Step 2: Implement 32-bit command and register transfers**

Use the ADS8688 program-write format `ADDR[6:0], WR=1, DATA[7:0]` in the first 16 clocks:

```c
static uint32_t ads8688_make_register_write(uint8_t address, uint8_t value)
{
    uint16_t command = ((uint16_t)(address & 0x7fu) << 9)
                     | (1u << 8)
                     | value;
    return (uint32_t)command << 16;
}

static uint32_t ads8688_make_register_read(uint8_t address)
{
    return (uint32_t)((uint16_t)(address & 0x7fu) << 9) << 16;
}
```

Use `HAL_SPI_TransmitReceive(&hspi2, ...)` for one 32-bit data unit while DMA is stopped. Extract register readback from the received frame according to logic-analyzer-confirmed bit placement; the initial implementation uses the low byte after the 16 command clocks.

- [ ] **Step 3: Implement deterministic reset and verified initialization**

Implement:

```c
HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
for (volatile uint32_t delay = 0u; delay < 1000u; ++delay) { __NOP(); }
HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);
HAL_Delay(15u);
```

Then write and verify:

- AUTO sequence register `0x01 = 0xff`;
- feature register `0x03 = 0x00`;
- range registers `0x05` through `0x0c = 0x00`;
- AUTO_RST command `0xa000`.

Retry initialization at most three times and leave the driver stopped after the third failure.

- [ ] **Step 4: Run tests and syntax compilation**

Run contract tests and compile both ADS8688 modules with the ARM compiler command from Task 2, adding `Core/User/ads8688.c`.

Expected: tests PASS and compilation exits 0.

- [ ] **Step 5: Commit transport and initialization**

```powershell
git add Core/User/ads8688.c Core/User/ads8688.h tests/test_project_contract.py
git commit -m "feat: initialize and configure ADS8688"
```

### Task 4: Add Circular DMA Acquisition and Mode Switching

**Files:**
- Modify: `Core/User/ads8688.c`
- Modify: `Core/User/ads8688.h`
- Modify: `tests/test_project_contract.py`

**Interfaces:**
- Consumes: storage APIs and initialized SPI2/DMA handles.
- Produces: `ads8688_process()`, automatic and manual mode switching, DMA recovery, and diagnostics.

- [ ] **Step 1: Add failing DMA and callback contract checks**

Assert these constants and flags exist:

```python
self.assertRegex(text, r"#define\s+ADS8688_DMA_WORD_COUNT\s+1024u")
for flag in ("ads8688_dma_half_flag", "ads8688_dma_full_flag",
             "ads8688_error_flag"):
    self.assertIn(flag, text)
```

Add a callback-body check that strips comments and verifies each of:

- `HAL_SPI_TxRxHalfCpltCallback`
- `HAL_SPI_TxRxCpltCallback`
- `HAL_SPI_ErrorCallback`

contains exactly one assignment to its corresponding flag and no loop, delay, print, register parsing, or storage call.

- [ ] **Step 2: Start circular DMA**

Declare aligned buffers:

```c
static uint32_t ads8688_dma_rx[ADS8688_DMA_WORD_COUNT]
    __attribute__((aligned(32)));
static uint32_t ads8688_dma_tx_word;
```

Set `ads8688_dma_tx_word = 0u` and call:

```c
HAL_SPI_TransmitReceive_DMA(&hspi2,
                            (uint8_t *)&ads8688_dma_tx_word,
                            (uint8_t *)ads8688_dma_rx,
                            ADS8688_DMA_WORD_COUNT);
```

TX memory increment is disabled, so DMA repeatedly reads the same NO_OP word.

- [ ] **Step 3: Process DMA halves in the main-loop function**

For each frame:

```c
uint16_t raw_code = (uint16_t)(frame & 0xffffu);
ads8688_storage_push(channel, raw_code, range, sample_index);
sample_index++;
channel = ads8688_next_enabled_channel(channel, channel_mask);
```

Track the expected half. If a half/full flag arrives out of order, increment `lost_sample_count`, discard the ambiguous half, stop DMA, resend the active mode command, reset channel tracking, and restart DMA.

- [ ] **Step 4: Implement mode and range changes**

`ads8688_set_auto_mode(mask)` rejects `mask == 0`, stops DMA, writes register `0x01`, verifies it, sends AUTO_RST, selects the lowest enabled channel, and restarts DMA.

`ads8688_set_manual_mode(channel)` rejects channels above 7, stops DMA, sends:

```c
uint16_t command = (uint16_t)(0xc000u + ((uint16_t)channel << 10));
```

then fixes the software channel tag to that channel and restarts DMA.

`ads8688_set_channel_range(channel, range)` validates both values, stops DMA, writes and verifies register `0x05 + channel`, restores the active acquisition command, and restarts DMA without clearing history.

- [ ] **Step 5: Run tests, syntax compilation, and inspect callbacks**

Run:

```powershell
python -m unittest tests.test_project_contract -v
git diff --check
```

Compile `ads8688.c` and `ads8688_storage.c` with the ARM syntax command. Expected: all checks pass and every callback remains flag-only.

- [ ] **Step 6: Commit acquisition**

```powershell
git add Core/User/ads8688.c Core/User/ads8688.h tests/test_project_contract.py
git commit -m "feat: acquire ADS8688 samples with circular DMA"
```

### Task 5: Integrate the User System Entry Point

**Files:**
- Create: `Core/User/system.c`
- Modify: `Core/User/system.h`
- Modify: `Core/Src/main.c`
- Modify: `tests/test_project_contract.py`

**Interfaces:**
- Consumes: generated GPIO, DMA, and SPI initialization already called by CubeMX.
- Produces: application startup and continuous processing.

- [ ] **Step 1: Add failing `main.c` structure checks**

Check:

```python
main = (ROOT / "Core" / "Src" / "main.c").read_text(encoding="utf-8")
self.assertIn('#include "system.h"', main)
self.assertRegex(main, r"/\* USER CODE BEGIN 2 \*/\s*system_init\(\);\s*/\* USER CODE END 2 \*/")
self.assertRegex(main, r"/\* USER CODE BEGIN 3 \*/\s*ads8688_process\(\);\s*\}")
```

Run tests and expect failure before integration.

- [ ] **Step 2: Implement `system_init()`**

`Core/User/system.c` contains only:

```c
#include "system.h"

void system_init(void)
{
    (void)ads8688_init();
}
```

The final commented version documents the ignored return value and exposes initialization status through diagnostics.

- [ ] **Step 3: Modify only CubeMX user sections in `main.c`**

- Add `#include "system.h"` in `USER CODE BEGIN Includes`.
- Add only `system_init();` in `USER CODE BEGIN 2`.
- Add only `ads8688_process();` in `USER CODE BEGIN 3`.

- [ ] **Step 4: Run contract checks**

Run `python -m unittest tests.test_project_contract -v`.

Expected: PASS.

- [ ] **Step 5: Commit integration**

```powershell
git add Core/User/system.c Core/User/system.h Core/Src/main.c tests/test_project_contract.py
git commit -m "feat: integrate ADS8688 acquisition loop"
```

### Task 6: Add Chinese Documentation and README

**Files:**
- Modify: every file under `Core/User/`
- Create: `README.md`
- Modify: `tests/test_project_contract.py`

**Interfaces:**
- Documents all public and internal interfaces delivered by Tasks 1–5.

- [ ] **Step 1: Add failing documentation checks**

Check every `Core/User/*.c` and `Core/User/*.h` begins with module-purpose text and contains:

- `模块用途`
- `GPIO 引脚映射`
- `依赖的外设和 CubeIDE 配置`
- `初始化方法`
- `调用方法`

Check `README.md` contains:

- `STM32H750VBT6`
- `STM32CubeIDE 1.19.0`
- `PB12`, `PB13`, `PB14`, `PB15`, `PD8`, `PD9`
- automatic scan and manual acquisition usage
- build, flash, run, validation, known limitations, and no-board-verification disclosure.

- [ ] **Step 2: Add Chinese module and function comments**

For every user function, document:

```c
/**
 * @brief  中文功能说明。
 * @param  参数名 中文参数说明。
 * @retval 中文返回值说明。
 * @note   重要副作用，包括 DMA 状态变化或历史缓冲区变化。
 */
```

Document every important variable at its declaration. Mark callback flags as interrupt-shared `volatile` variables and explain their concurrency purpose.

- [ ] **Step 3: Write README**

Document the exact wiring, internal-reference selection, CubeIDE clock/SPI/DMA/NVIC settings, directory layout, API examples, build and flash procedure, expected 393 kSPS rate, 4096-record storage behavior, 200 kSPS fallback, and logic-analyzer validation steps.

- [ ] **Step 4: Run documentation and contract tests**

Run:

```powershell
python -m unittest tests.test_project_contract -v
git diff --check
```

Expected: PASS and no whitespace errors.

- [ ] **Step 5: Commit documentation**

```powershell
git add Core/User README.md tests/test_project_contract.py
git commit -m "docs: document ADS8688 acquisition module"
```

### Task 7: Build and Final Verification

**Files:**
- Modify only files required to resolve verified build defects.

**Interfaces:**
- Verifies the complete firmware artifact and repository state.

- [ ] **Step 1: Run all contract tests**

```powershell
python -m unittest discover -s tests -p "test_*.py" -v
```

Expected: all tests PASS.

- [ ] **Step 2: Run a clean STM32CubeIDE Debug build**

```powershell
$workspace = Join-Path $env:TEMP 'h7_pre-cubeide-workspace'
& 'D:\CubeIDE\STM32CubeIDE_1.19.0\STM32CubeIDE\stm32cubeidec.exe' `
  --launcher.suppressErrors -nosplash `
  -application org.eclipse.cdt.managedbuilder.core.headlessbuild `
  -data $workspace `
  -import 'D:\CubeIDE\h7_pre' `
  -cleanBuild 'h7_pre/Debug'
```

Expected: exit code 0, no compiler errors, and `Debug/h7_pre.elf` generated.

- [ ] **Step 3: Run structural audits**

Verify:

```powershell
git status --short
Get-ChildItem Core\User -Recurse -Include *.c,*.h
Select-String Core\Src\main.c -Pattern 'system_init|ads8688_process'
Select-String Core\User\ads8688.c -Pattern 'HAL_SPI_TxRxHalfCpltCallback|HAL_SPI_TxRxCpltCallback|HAL_SPI_ErrorCallback'
```

Expected: only intended files are changed, all user C/H files are under `Core/User`, and callbacks contain only one flag assignment.

- [ ] **Step 4: Record hardware validation as pending**

Do not claim board-level success without the target board. Report these pending checks:

- 16.125 MHz SCLK;
- 32 SCLK edges per NSS frame;
- NSS high time and approximately 393 kSPS total frame rate;
- ground input near `0x8000` at bipolar ±10.24 V;
- all-channel order, manual-mode tagging, and voltage accuracy.

- [ ] **Step 5: Commit build fixes if any**

If build fixes were necessary:

```powershell
git add Core/User Core/Src/main.c README.md tests/test_project_contract.py
git commit -m "fix: resolve ADS8688 integration build issues"
```

If no fixes were needed, make no empty commit.

- [ ] **Step 6: Report completion without pushing**

Report tests, clean-build result, commit list, hardware checks still pending, and explicitly state that no remote push was performed.
