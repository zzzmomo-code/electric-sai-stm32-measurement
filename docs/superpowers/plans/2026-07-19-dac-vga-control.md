# DAC and VGA Six-Level Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add safe six-level PA4/DAC1 output control and calculate the external differential VGA gain from a selected level.

**Architecture:** A focused `vga_control` user module owns DAC start/write operations, level validation, voltage-to-code conversion, gain calculation, and debugger-visible state. Both public level operations use explicit `switch` statements; voltage and analog formulas remain configurable macros, while the on-chip DAC always uses HAL 12-bit right alignment and derives its code automatically.

**Tech Stack:** STM32H743VIT6, STM32CubeIDE 1.19.0, STM32 HAL DAC, C11, Python `unittest` contract tests.

## Global Constraints

- Put all new user-authored `.c` and `.h` files in `Core/User/`.
- Every user `.c` file includes only `system.h`; `system.h` remains the unified user header.
- Do not modify `.ioc`, `Core/Inc/dac.h`, `Core/Src/dac.c`, or generated `MX_*` initialization.
- PA4 is already configured as DAC1_OUT1 with no trigger and output buffer enabled.
- `main.c` user initialization remains only `system_init();`; its loop remains only `system_process();`.
- User identifiers use lower snake case; all delivered user code has complete Chinese module/function/variable comments.
- Preserve all existing uncommitted changes and never stage unrelated hunks, especially the existing `system.c` GPIO-toggle removal and generated clock/DAC changes.
- Default level is 0. Invalid levels return an error and do not call HAL or change recorded output state.
- User-adjustable macros include six voltages, DAC reference voltage, VG scale/offset, Rf, RG, gain formula, and VOUT formula. Six manual DAC-code macros must not exist.
- Do not push any commit.

---

### Task 1: VGA control module and level math

**Files:**
- Create: `tests/test_vga_control_contract.py`
- Create: `Core/User/vga_control.h`
- Create: `Core/User/vga_control.c`

**Interfaces:**
- Consumes: CubeMX handle `hdac1`; HAL APIs `HAL_DAC_Start()` and `HAL_DAC_SetValue()`.
- Produces: `void vga_control_init(void)`, `vga_control_status_t vga_control_set_level(uint8_t level)`, `vga_control_status_t vga_control_gain_from_level(uint8_t level, float *gain)`, and `vga_control_diagnostics`.

- [ ] **Step 1: Write the failing module contract tests**

Create `tests/test_vga_control_contract.py` with tests that read the real production files and validate the requested API, formulas, switch behavior, and automatic 12-bit conversion:

```python
from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]


class VgaControlContractTest(unittest.TestCase):
    def test_module_exposes_six_voltage_levels_and_formulas(self):
        header = (ROOT / "Core/User/vga_control.h").read_text(encoding="utf-8")
        for index, voltage in enumerate(("0.00f", "0.66f", "1.32f", "1.98f", "2.64f", "3.30f")):
            self.assertIn(f"#define VGA_CONTROL_LEVEL_{index}_VOLTAGE_V {voltage}", header)
        self.assertIn("#define VGA_CONTROL_DAC_REFERENCE_VOLTAGE_V 3.30f", header)
        self.assertIn("#define VGA_CONTROL_VG_SCALE (20.0f / 33.0f)", header)
        self.assertIn("#define VGA_CONTROL_VG_OFFSET_V (-1.0f)", header)
        self.assertIn("#define VGA_CONTROL_RF 1.0f", header)
        self.assertIn("#define VGA_CONTROL_RG 1.0f", header)
        self.assertIn("VGA_CONTROL_VG_FROM_DAC_VOLTAGE", header)
        self.assertIn("VGA_CONTROL_GAIN_FROM_VG", header)
        self.assertIn("VGA_CONTROL_VOUT_FROM_INPUTS", header)
        self.assertNotRegex(header, r"LEVEL_[0-5]_CODE")

    def test_six_level_math_matches_the_design(self):
        voltages = (0.00, 0.66, 1.32, 1.98, 2.64, 3.30)
        expected_codes = (0, 819, 1638, 2457, 3276, 4095)
        expected_vg = (-1.0, -0.6, -0.2, 0.2, 0.6, 1.0)
        expected_gain = (0.0, 0.4, 0.8, 1.2, 1.6, 2.0)
        for index, voltage in enumerate(voltages):
            code = int(voltage / 3.30 * ((1 << 12) - 1) + 0.5)
            vg = (20.0 / 33.0) * voltage - 1.0
            gain = (1.0 + vg) * 1.0 / 1.0
            self.assertEqual(expected_codes[index], code)
            self.assertAlmostEqual(expected_vg[index], vg, places=6)
            self.assertAlmostEqual(expected_gain[index], gain, places=6)

    def test_both_level_functions_use_switch_and_default(self):
        source = (ROOT / "Core/User/vga_control.c").read_text(encoding="utf-8")
        self.assertGreaterEqual(source.count("switch (level)"), 2)
        for level in range(6):
            self.assertGreaterEqual(source.count(f"case {level}u:"), 2)
        self.assertGreaterEqual(source.count("default:"), 2)

    def test_on_chip_dac_uses_automatic_12bit_right_aligned_code(self):
        source = (ROOT / "Core/User/vga_control.c").read_text(encoding="utf-8")
        self.assertIn("(1u << 12u) - 1u", source)
        self.assertIn("DAC_ALIGN_12B_R", source)
        self.assertIn("HAL_DAC_Start(&hdac1, DAC_CHANNEL_1)", source)
        self.assertIn("HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_code)", source)

    def test_invalid_set_level_returns_before_hal_write(self):
        source = (ROOT / "Core/User/vga_control.c").read_text(encoding="utf-8")
        body = source.split("vga_control_status_t vga_control_set_level", 1)[1]
        switch_body = body.split("HAL_DAC_SetValue", 1)[0]
        default_body = switch_body.rsplit("default:", 1)[1]
        self.assertIn("return vga_control_status_invalid_level;", default_body)

    def test_public_api_and_diagnostics_are_declared(self):
        header = (ROOT / "Core/User/vga_control.h").read_text(encoding="utf-8")
        self.assertIn("void vga_control_init(void);", header)
        self.assertIn("vga_control_status_t vga_control_set_level(uint8_t level);", header)
        self.assertIn("vga_control_status_t vga_control_gain_from_level(uint8_t level, float *gain);", header)
        self.assertIn("extern vga_control_diagnostics_t vga_control_diagnostics;", header)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the new test and verify RED**

Run:

```powershell
python -m unittest tests.test_vga_control_contract -v
```

Expected: failures with `FileNotFoundError` for `Core/User/vga_control.h` and `Core/User/vga_control.c`. This proves the test is red because the module is absent.

- [ ] **Step 3: Create the public header**

Create `Core/User/vga_control.h` with complete Chinese module and API comments and this functional content:

```c
#ifndef VGA_CONTROL_H
#define VGA_CONTROL_H

#include <stdint.h>

#define VGA_CONTROL_DAC_REFERENCE_VOLTAGE_V 3.30f
#define VGA_CONTROL_LEVEL_0_VOLTAGE_V 0.00f
#define VGA_CONTROL_LEVEL_1_VOLTAGE_V 0.66f
#define VGA_CONTROL_LEVEL_2_VOLTAGE_V 1.32f
#define VGA_CONTROL_LEVEL_3_VOLTAGE_V 1.98f
#define VGA_CONTROL_LEVEL_4_VOLTAGE_V 2.64f
#define VGA_CONTROL_LEVEL_5_VOLTAGE_V 3.30f

#define VGA_CONTROL_VG_SCALE (20.0f / 33.0f)
#define VGA_CONTROL_VG_OFFSET_V (-1.0f)
#define VGA_CONTROL_RF 1.0f
#define VGA_CONTROL_RG 1.0f

#define VGA_CONTROL_VG_FROM_DAC_VOLTAGE(dac_voltage_v) \
    ((VGA_CONTROL_VG_SCALE * (dac_voltage_v)) + VGA_CONTROL_VG_OFFSET_V)
#define VGA_CONTROL_GAIN_FROM_VG(vg_voltage_v) \
    (((1.0f + (vg_voltage_v)) * VGA_CONTROL_RF) / VGA_CONTROL_RG)
#define VGA_CONTROL_VOUT_FROM_INPUTS(vin_positive_v, vin_negative_v, vg_voltage_v) \
    (((vin_positive_v) - (vin_negative_v)) * VGA_CONTROL_GAIN_FROM_VG(vg_voltage_v))

typedef enum
{
    vga_control_status_ok = 0,
    vga_control_status_invalid_level,
    vga_control_status_null_pointer,
    vga_control_status_dac_error
} vga_control_status_t;

typedef struct
{
    uint8_t current_level;
    float dac_voltage_v;
    float vg_voltage_v;
    float vga_gain;
    vga_control_status_t last_status;
    uint32_t last_hal_status;
} vga_control_diagnostics_t;

extern vga_control_diagnostics_t vga_control_diagnostics;

void vga_control_init(void);
vga_control_status_t vga_control_set_level(uint8_t level);
vga_control_status_t vga_control_gain_from_level(uint8_t level, float *gain);

#endif /* VGA_CONTROL_H */
```

The Chinese comments must state: module purpose; PA4/DAC1_OUT1 mapping; DAC1 channel 1, no-trigger, buffered-output dependency; `system_init()` initialization; public calling examples; every function's parameters, return value, and side effects; every diagnostics field's purpose.

- [ ] **Step 4: Create the switch-based implementation**

Create `Core/User/vga_control.c`. Include only `system.h`, add the required Chinese module header, and implement this behavior:

```c
#include "system.h"

_Static_assert(VGA_CONTROL_DAC_REFERENCE_VOLTAGE_V > 0.0f,
               "DAC reference voltage must be positive");
_Static_assert(VGA_CONTROL_RG > 0.0f, "VGA RG must be positive");

vga_control_diagnostics_t vga_control_diagnostics = {
    0xffu,
    0.0f,
    0.0f,
    0.0f,
    vga_control_status_dac_error,
    (uint32_t)HAL_ERROR
};

static uint32_t vga_control_voltage_to_dac_code(float dac_voltage_v)
{
    const uint32_t dac_max_code = (1u << 12u) - 1u;
    uint32_t dac_code = (uint32_t)(((dac_voltage_v
                                    / VGA_CONTROL_DAC_REFERENCE_VOLTAGE_V)
                                   * (float)dac_max_code) + 0.5f);

    if (dac_code > dac_max_code)
    {
        dac_code = dac_max_code;
    }

    return dac_code;
}

void vga_control_init(void)
{
    HAL_StatusTypeDef hal_status;

    hal_status = HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
    vga_control_diagnostics.last_hal_status = (uint32_t)hal_status;
    if (hal_status != HAL_OK)
    {
        vga_control_diagnostics.last_status = vga_control_status_dac_error;
        return;
    }

    vga_control_diagnostics.last_status = vga_control_set_level(0u);
}

vga_control_status_t vga_control_set_level(uint8_t level)
{
    float dac_voltage_v;
    float gain;
    float vg_voltage_v;
    uint32_t dac_code;
    HAL_StatusTypeDef hal_status;

    switch (level)
    {
        case 0u: dac_voltage_v = VGA_CONTROL_LEVEL_0_VOLTAGE_V; break;
        case 1u: dac_voltage_v = VGA_CONTROL_LEVEL_1_VOLTAGE_V; break;
        case 2u: dac_voltage_v = VGA_CONTROL_LEVEL_2_VOLTAGE_V; break;
        case 3u: dac_voltage_v = VGA_CONTROL_LEVEL_3_VOLTAGE_V; break;
        case 4u: dac_voltage_v = VGA_CONTROL_LEVEL_4_VOLTAGE_V; break;
        case 5u: dac_voltage_v = VGA_CONTROL_LEVEL_5_VOLTAGE_V; break;
        default: return vga_control_status_invalid_level;
    }

    dac_code = vga_control_voltage_to_dac_code(dac_voltage_v);
    hal_status = HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_code);
    vga_control_diagnostics.last_hal_status = (uint32_t)hal_status;
    if (hal_status != HAL_OK)
    {
        vga_control_diagnostics.last_status = vga_control_status_dac_error;
        return vga_control_status_dac_error;
    }

    vg_voltage_v = VGA_CONTROL_VG_FROM_DAC_VOLTAGE(dac_voltage_v);
    (void)vga_control_gain_from_level(level, &gain);
    vga_control_diagnostics.current_level = level;
    vga_control_diagnostics.dac_voltage_v = dac_voltage_v;
    vga_control_diagnostics.vg_voltage_v = vg_voltage_v;
    vga_control_diagnostics.vga_gain = gain;
    vga_control_diagnostics.last_status = vga_control_status_ok;
    return vga_control_status_ok;
}

vga_control_status_t vga_control_gain_from_level(uint8_t level, float *gain)
{
    float dac_voltage_v;
    float vg_voltage_v;

    if (gain == NULL)
    {
        return vga_control_status_null_pointer;
    }

    switch (level)
    {
        case 0u: dac_voltage_v = VGA_CONTROL_LEVEL_0_VOLTAGE_V; break;
        case 1u: dac_voltage_v = VGA_CONTROL_LEVEL_1_VOLTAGE_V; break;
        case 2u: dac_voltage_v = VGA_CONTROL_LEVEL_2_VOLTAGE_V; break;
        case 3u: dac_voltage_v = VGA_CONTROL_LEVEL_3_VOLTAGE_V; break;
        case 4u: dac_voltage_v = VGA_CONTROL_LEVEL_4_VOLTAGE_V; break;
        case 5u: dac_voltage_v = VGA_CONTROL_LEVEL_5_VOLTAGE_V; break;
        default: return vga_control_status_invalid_level;
    }

    vg_voltage_v = VGA_CONTROL_VG_FROM_DAC_VOLTAGE(dac_voltage_v);
    *gain = VGA_CONTROL_GAIN_FROM_VG(vg_voltage_v);
    return vga_control_status_ok;
}
```

Add Chinese Doxygen comments before the private helper and all three public functions. Important variables (`dac_max_code`, `dac_code`, diagnostics state) receive declaration-site Chinese comments.

- [ ] **Step 5: Run the module test and verify GREEN**

Run:

```powershell
python -m unittest tests.test_vga_control_contract -v
```

Expected: all six `VgaControlContractTest` tests pass.

- [ ] **Step 6: Commit only the new module and its test**

```powershell
git add -- Core/User/vga_control.c Core/User/vga_control.h tests/test_vga_control_contract.py
git diff --cached --check
git commit -m "feat: add switch-based DAC VGA control"
```

Expected: the commit contains exactly the three listed files.

---

### Task 2: Unified-header and startup integration

**Files:**
- Modify: `tests/test_vga_control_contract.py`
- Modify: `Core/User/system.h`
- Modify: `Core/User/system.c`

**Interfaces:**
- Consumes: Task 1's `vga_control.h` and `vga_control_init()`.
- Produces: DAC declarations and VGA API through `system.h`; safe level-0 initialization through `system_init()`.

- [ ] **Step 1: Add failing integration tests**

Append these methods to `VgaControlContractTest`:

```python
    def test_system_header_is_the_unified_dac_and_vga_entry(self):
        header = (ROOT / "Core/User/system.h").read_text(encoding="utf-8")
        self.assertIn('#include "dac.h"', header)
        self.assertIn('#include "vga_control.h"', header)

    def test_system_initializes_vga_after_cubemx_initializes_dac(self):
        main = (ROOT / "Core/Src/main.c").read_text(encoding="utf-8")
        system = (ROOT / "Core/User/system.c").read_text(encoding="utf-8")
        self.assertLess(main.index("MX_DAC1_Init();"), main.index("system_init();"))
        init_body = re.search(
            r"void system_init\(void\)\s*\{(?P<body>.*?)\n\}",
            system,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(init_body)
        self.assertIn("vga_control_init();", init_body.group("body"))

    def test_main_user_regions_remain_thin(self):
        main = (ROOT / "Core/Src/main.c").read_text(encoding="utf-8")
        init_region = main.split("/* USER CODE BEGIN 2 */", 1)[1].split("/* USER CODE END 2 */", 1)[0]
        loop_region = main.split("/* USER CODE BEGIN 3 */", 1)[1].split("/* USER CODE END 3 */", 1)[0]
        self.assertEqual(["system_init();"], [line.strip() for line in init_region.splitlines() if line.strip()])
        self.assertEqual(["system_process();"], [line.strip() for line in loop_region.splitlines() if line.strip()])
```

- [ ] **Step 2: Run the integration tests and verify RED**

Run:

```powershell
python -m unittest tests.test_vga_control_contract.VgaControlContractTest.test_system_header_is_the_unified_dac_and_vga_entry tests.test_vga_control_contract.VgaControlContractTest.test_system_initializes_vga_after_cubemx_initializes_dac -v
```

Expected: failure because `system.h` does not include `vga_control.h`/`dac.h` and `system_init()` does not call `vga_control_init()`.

- [ ] **Step 3: Integrate the module without touching generated files**

In `Core/User/system.h`, directly after `#include "main.h"`, add:

```c
#include "dac.h"
```

After the existing user-module includes, add:

```c
#include "vga_control.h"
```

Update the Chinese dependency comment at the top of `system.h` to mention DAC1 and PA4 through the VGA module.

In the `Core/User/system.c` module header, add PA4/DAC1_OUT1 to the GPIO mapping and DAC1 to the peripheral dependency list. Add this as the first operation inside `system_init()` so the VGA powers up at its safe zero-gain level before the other functional modules:

```c
    vga_control_init();
```

Do not alter the user's existing removal of `HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);` from `system_process()`.

- [ ] **Step 4: Run the integration tests and verify GREEN**

Run:

```powershell
python -m unittest tests.test_vga_control_contract -v
```

Expected: all VGA contract and integration tests pass.

- [ ] **Step 5: Stage only owned integration hunks and commit**

Stage `system.h` and the test normally. Stage only the VGA header-comment/init hunks from the already-dirty `system.c`; explicitly exclude the pre-existing GPIO-toggle deletion. Verify the cached diff before committing:

```powershell
git add -- Core/User/system.h tests/test_vga_control_contract.py
git add -p -- Core/User/system.c
git diff --cached -- Core/User/system.c Core/User/system.h tests/test_vga_control_contract.py
git commit -m "feat: initialize VGA control at startup"
```

At the interactive `git add -p`, accept only hunks adding PA4/DAC1 documentation and `vga_control_init();`; reject any hunk deleting `HAL_GPIO_TogglePin`. If a hunk combines owned and unrelated lines, split it with `s` or edit it with `e` before staging.

---

### Task 3: README documentation

**Files:**
- Modify: `tests/test_vga_control_contract.py`
- Modify: `README.md`

**Interfaces:**
- Consumes: Task 1 public macros and APIs.
- Produces: Hardware/configuration/use/verification instructions for the six VGA levels.

- [ ] **Step 1: Add a failing README contract test**

Append:

```python
    def test_readme_documents_dac_vga_control(self):
        readme = (ROOT / "README.md").read_text(encoding="utf-8")
        required = (
            "PA4 / DAC1_OUT1",
            "DAC_TRIGGER_NONE",
            "DAC_OUTPUTBUFFER_ENABLE",
            "vga_control_set_level",
            "vga_control_gain_from_level",
            "VG = (20 / 33) * VDAC - 1",
            "0、0.66、1.32、1.98、2.64、3.3 V",
        )
        for text in required:
            self.assertIn(text, readme)
```

- [ ] **Step 2: Run the README test and verify RED**

Run:

```powershell
python -m unittest tests.test_vga_control_contract.VgaControlContractTest.test_readme_documents_dac_vga_control -v
```

Expected: failure because README does not yet contain `PA4 / DAC1_OUT1`.

- [ ] **Step 3: Document the hardware and API**

Update `README.md` in Chinese with:

- Hardware list entry: `PA4 / DAC1_OUT1` connects to the external control-voltage amplifier input; the amplifier output is `VG` and controls the external differential VGA.
- CubeMX DAC section: DAC1 OUT1, `DAC_TRIGGER_NONE`, `DAC_OUTPUTBUFFER_ENABLE`, PA4 analog/no-pull, no DMA, no interrupt; generated `MX_DAC1_Init()` remains before `system_init()`.
- One six-row table containing level, VDAC, automatic 12-bit code, VG, and default gain values from the design.
- Formula block exactly containing `VG = (20 / 33) * VDAC - 1` and the differential `VOUT` formula.
- Macro location: `Core/User/vga_control.h`; explain that voltage, VDDA reference, Rf, and RG are adjustable while six DAC codes are automatic and not manually configured.
- Usage example:

```c
float vga_gain;

if (vga_control_set_level(3u) == vga_control_status_ok)
{
    (void)vga_control_gain_from_level(3u, &vga_gain);
}
```

- Verification: measure PA4 at all levels, then measure amplifier VG and differential VGA gain. Note VDDA, resistor tolerance, amplifier offset/gain error, and VGA characteristics as sources of real-world deviation.

- [ ] **Step 4: Run the README test and verify GREEN**

Run:

```powershell
python -m unittest tests.test_vga_control_contract.VgaControlContractTest.test_readme_documents_dac_vga_control -v
```

Expected: pass.

- [ ] **Step 5: Commit README and its test**

```powershell
git add -- README.md tests/test_vga_control_contract.py
git diff --cached --check
git commit -m "docs: document DAC VGA gain levels"
```

Expected: commit contains only README and the VGA test file.

---

### Task 4: Full verification and delivery audit

**Files:**
- Verify only; repair only owned files if a check identifies a defect.

**Interfaces:**
- Consumes: all earlier tasks.
- Produces: test/build evidence and a clean scope audit without pushing.

- [ ] **Step 1: Run all offline contract tests**

Run:

```powershell
python -m unittest discover -s tests -p 'test_*.py' -v
```

Expected: all tests pass with no failures or errors.

- [ ] **Step 2: Run focused static checks**

Run PowerShell searches to confirm:

```powershell
Select-String -Path Core/User/vga_control.c -Pattern 'switch \(level\)|case [0-5]u:|default:|DAC_ALIGN_12B_R'
Select-String -Path Core/User/vga_control.h -Pattern 'LEVEL_[0-5]_CODE'
git diff --check
```

Expected: two switches, twelve total level cases, at least two defaults, `DAC_ALIGN_12B_R` present, no `LEVEL_x_CODE` match, and no whitespace errors in owned changes.

- [ ] **Step 3: Build the Debug configuration**

Use the verified STM32CubeIDE 1.19.0 headless-build entry. Create an isolated temporary IDE workspace and run a clean Debug build:

```powershell
$build_workspace = Join-Path ([System.IO.Path]::GetTempPath()) ('codex-h743-pre1-' + [guid]::NewGuid().ToString('N'))
& 'D:\CubeIDE\STM32CubeIDE_1.19.0\STM32CubeIDE\headless-build.bat' -data $build_workspace -import 'D:\CubeIDE\h743_pre1' -cleanBuild 'h743_pre1/Debug'
```

Expected: exit code 0, 0 compile errors, and no new warnings from `vga_control.c`.

- [ ] **Step 4: Audit project rules and Git scope**

Verify:

```powershell
git status --short
git log -5 --oneline --decorate
git show --stat --oneline HEAD
```

Confirm:

- `vga_control.c/.h` are under `Core/User/`.
- `main.c` remains unchanged by this feature and retains thin user regions.
- No interrupt callback was added.
- All new/modified user code has Chinese comments.
- README contains hardware mapping, CubeMX setup, APIs, limits, and verification.
- Existing unrelated working-tree changes remain present and uncommitted.
- No remote push occurred.

- [ ] **Step 5: Report software verification and hardware follow-up**

Report exact test totals and build result. State that PA4 voltage, external amplifier VG, and physical VGA gain still require real-board measurement unless the user provides those measurements during this task. Do not claim hardware validation from software-only evidence.
