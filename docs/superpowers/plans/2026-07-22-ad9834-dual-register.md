# AD9834 Dual Register Support Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add independent programming of AD9834 `FREQ0`, `FREQ1`, `PHASE0`, and `PHASE1` while preserving every existing public function and the current `dds_control` behavior.

**Architecture:** Extend the existing `ad9834` module with register-parameterized setters. Keep `ad9834_set_frequency_hz()` as a compatibility wrapper for `FREQ0`, keep output selection on the existing FSELECT/PSELECT GPIO functions, and extend diagnostics without removing existing fields.

**Tech Stack:** STM32H7 HAL C, STM32CubeIDE 1.19.0, AD9834 16-bit SPI protocol, Python `unittest` contract tests, GNU Make generated Debug build.

## Global Constraints

- All modified user C and header files remain under `Core/User/` and include only `system.h` from user C files.
- Do not modify `.ioc`, CubeMX-generated peripheral initialization, GPIO, DMA, NVIC, or clock configuration.
- Preserve all existing AD9834 public functions, enum values, and default `FREQ0 + PHASE0` behavior.
- Use lower snake case for all new user identifiers and complete Chinese comments before delivery.
- Preserve the user's existing `Core/User/dds_control.h` change and all unrelated working-tree changes.
- Do not stage or commit Debug build artifacts.
- Do not push without explicit user permission.

---

### Task 1: Define failing dual-register contract tests

**Files:**
- Modify: `tests/test_dds_contract.py`
- Test: `tests/test_dds_contract.py`

**Interfaces:**
- Consumes: existing `read_text()` helper and `unittest.TestCase` test class in `tests/test_dds_contract.py`.
- Produces: failing contracts for `ad9834_set_frequency_register_hz()`, `ad9834_set_phase_register_degrees()`, register addresses, validation, initialization, and the compatibility wrapper.

- [ ] **Step 1: Add a failing public-interface contract test**

Add a test that reads `Core/User/ad9834.h` and asserts these exact declarations and appended status values exist:

```python
def test_driver_declares_dual_frequency_and_phase_writers(self) -> None:
    header = read_text("Core/User/ad9834.h")

    self.assertIn("ad9834_status_invalid_register", header)
    self.assertIn("ad9834_status_invalid_phase", header)
    self.assertIn("ad9834_set_frequency_register_hz", header)
    self.assertIn("ad9834_set_phase_register_degrees", header)
```

- [ ] **Step 2: Add failing register-address and conversion contract tests**

Add assertions that source defines `0x4000`, `0x8000`, `0xC000`, and `0xE000` addresses, validates `phase_degrees > 359u`, and performs rounded integer conversion using `4096u` and `180u` before division by `360u`.

```python
def test_driver_uses_all_ad9834_register_addresses(self) -> None:
    source = read_text("Core/User/ad9834.c")

    for text in (
        "AD9834_FREQ0_ADDRESS  0x4000u",
        "AD9834_FREQ1_ADDRESS  0x8000u",
        "AD9834_PHASE0_ADDRESS 0xC000u",
        "AD9834_PHASE1_ADDRESS 0xE000u",
        "phase_degrees > 359u",
        "((uint32_t)phase_degrees * 4096u) + 180u",
    ):
        self.assertIn(text, source)
```

- [ ] **Step 3: Add failing compatibility and initialization contract tests**

Assert the old setter delegates to the new setter with register 0, both register pairs are initialized, and the final selected pair remains register 0:

```python
def test_driver_preserves_freq0_compatibility_and_initializes_both_banks(self) -> None:
    source = read_text("Core/User/ad9834.c")

    self.assertIn(
        "ad9834_set_frequency_register_hz(ad9834_frequency_register_0, frequency_hz)",
        source,
    )
    self.assertGreaterEqual(source.count("ad9834_frequency_register_0"), 3)
    self.assertGreaterEqual(source.count("ad9834_frequency_register_1"), 2)
    self.assertGreaterEqual(source.count("ad9834_phase_register_0"), 3)
    self.assertGreaterEqual(source.count("ad9834_phase_register_1"), 2)
```

- [ ] **Step 4: Run the new tests and verify RED**

Run: `python -m unittest tests.test_dds_contract.DdsContractTests.test_driver_declares_dual_frequency_and_phase_writers tests.test_dds_contract.DdsContractTests.test_driver_uses_all_ad9834_register_addresses tests.test_dds_contract.DdsContractTests.test_driver_preserves_freq0_compatibility_and_initializes_both_banks -v`

Expected: FAIL because the new status values, setters, three register addresses, and dual-bank initialization do not yet exist.

- [ ] **Step 5: Commit the failing tests**

```powershell
git add -- tests/test_dds_contract.py
git commit -m "test: define AD9834 dual register contract"
```

### Task 2: Implement the public types and dual-register writers

**Files:**
- Modify: `Core/User/ad9834.h`
- Modify: `Core/User/ad9834.c`
- Test: `tests/test_dds_contract.py`

**Interfaces:**
- Consumes: `ad9834_frequency_register_t`, `ad9834_phase_register_t`, `ad9834_write_word()`, and `ad9834_calculate_tuning_word()`.
- Produces: `ad9834_set_frequency_register_hz(ad9834_frequency_register_t, uint32_t)` and `ad9834_set_phase_register_degrees(ad9834_phase_register_t, uint16_t)` returning `ad9834_status_t`.

- [ ] **Step 1: Extend status and diagnostic types in the header**

Append statuses so existing numeric values remain unchanged:

```c
typedef enum
{
    ad9834_status_ok = 0,
    ad9834_status_invalid_frequency,
    ad9834_status_spi_error,
    ad9834_status_invalid_register,
    ad9834_status_invalid_phase
} ad9834_status_t;
```

Append these fields to `ad9834_diagnostics_t`:

```c
uint32_t frequency_hz[2];
uint32_t frequency_tuning_word[2];
uint16_t phase_degrees[2];
uint16_t phase_word[2];
uint8_t last_frequency_register;
uint8_t last_phase_register;
```

Declare both new public setters with complete Chinese documentation for function, parameters, return value, and SPI side effects.

- [ ] **Step 2: Add register addresses and validated frequency writing**

Define all data addresses:

```c
#define AD9834_FREQ0_ADDRESS  0x4000u
#define AD9834_FREQ1_ADDRESS  0x8000u
#define AD9834_PHASE0_ADDRESS 0xC000u
#define AD9834_PHASE1_ADDRESS 0xE000u
```

Implement `ad9834_set_frequency_register_hz()` so it validates the enum and range before any SPI operation, selects the address by enum, sends low then high 14-bit words, and updates both legacy and per-bank diagnostics only after both writes succeed.

- [ ] **Step 3: Add validated integer-degree phase writing**

Implement `ad9834_set_phase_register_degrees()` so it validates the enum and `0~359` range before SPI, computes:

```c
phase_word = (uint16_t)((((uint32_t)phase_degrees * 4096u) + 180u) / 360u);
```

Select `AD9834_PHASE0_ADDRESS` or `AD9834_PHASE1_ADDRESS`, send one word, and update that bank's phase diagnostics only after success.

- [ ] **Step 4: Preserve the existing frequency setter as a wrapper**

Replace its body with delegation while keeping the exact public signature:

```c
ad9834_status_t ad9834_set_frequency_hz(uint32_t frequency_hz)
{
    return ad9834_set_frequency_register_hz(
        ad9834_frequency_register_0,
        frequency_hz);
}
```

- [ ] **Step 5: Make invalid selection enums leave GPIO unchanged**

Add an explicit validation guard at the start of each existing `void` selection function. Valid enum values continue to write the same PB14/PD8 levels and diagnostics; invalid values return without changing state.

- [ ] **Step 6: Run focused tests and verify GREEN**

Run: `python -m unittest tests.test_dds_contract -v`

Expected: all DDS contract tests PASS.

- [ ] **Step 7: Commit the driver implementation**

```powershell
git add -- Core/User/ad9834.h Core/User/ad9834.c
git commit -m "feat: support both AD9834 frequency and phase banks"
```

### Task 3: Initialize both banks and document usage

**Files:**
- Modify: `Core/User/ad9834.c`
- Modify: `Core/User/ad9834.h`
- Modify: `README.md`
- Test: `tests/test_dds_contract.py`

**Interfaces:**
- Consumes: both new register-parameterized setters from Task 2.
- Produces: deterministic dual-bank initialization with final `FREQ0 + PHASE0` selection and documented call examples.

- [ ] **Step 1: Extend diagnostic reset and initialization sequence**

Zero every newly added diagnostic array and last-register field. While RESET is active, call the frequency setter for registers 0 and 1 with `initial_frequency_hz`, call the phase setter for registers 0 and 1 with `0u`, then issue RUN. Finally select register 0 for both frequency and phase and set `initialized = 1u`.

Return immediately on each failed write, retaining the existing error propagation behavior.

- [ ] **Step 2: Complete module-level Chinese documentation**

Update both AD9834 file headers and public function comments to state:

- GPIO mapping remains PB12/PB13/PB15/PB14/PD8;
- SPI2 configuration remains unchanged;
- initialization programs both banks and selects bank 0;
- register programming does not automatically change FSELECT/PSELECT;
- selection is intended for main-loop context and has no SPI side effect.

- [ ] **Step 3: Update README register behavior and examples**

Replace the statement that formal mode only uses `FREQ0/PHASE0` with a compatibility statement, then add this usage sequence:

```c
ad9834_set_frequency_register_hz(ad9834_frequency_register_1, 2000000u);
ad9834_set_phase_register_degrees(ad9834_phase_register_1, 90u);
ad9834_select_frequency_register(ad9834_frequency_register_1);
ad9834_select_phase_register(ad9834_phase_register_1);
```

Explain that current `dds_control` continues to update `FREQ0` through `ad9834_set_frequency_hz()`.

- [ ] **Step 4: Run the DDS contract tests**

Run: `python -m unittest tests.test_dds_contract -v`

Expected: all DDS contract tests PASS.

- [ ] **Step 5: Commit initialization and documentation**

```powershell
git add -- Core/User/ad9834.c Core/User/ad9834.h README.md
git commit -m "docs: explain AD9834 bank programming and selection"
```

### Task 4: Full verification and repository hygiene

**Files:**
- Verify: `Core/User/ad9834.c`
- Verify: `Core/User/ad9834.h`
- Verify: `Core/User/dds_control.c`
- Verify: `Core/Src/main.c`
- Verify: `README.md`

**Interfaces:**
- Consumes: complete implementation from Tasks 1-3.
- Produces: fresh test/build evidence and a clean scoped source diff ready for user confirmation.

- [ ] **Step 1: Run the complete contract suite**

Run: `python -m unittest discover -s tests -p "test_*.py" -v`

Expected: all tests PASS with zero failures and zero errors.

- [ ] **Step 2: Build the STM32 Debug target**

Run: `mingw32-make -C Debug -j8 all`

If `mingw32-make` is unavailable on PATH, locate the STM32CubeIDE 1.19.0 bundled make executable and run the same generated Debug makefile. Expected: exit code 0 and no compiler or linker error.

- [ ] **Step 3: Check structural project rules**

Confirm `main.c` initialization user block still contains only `system_init();`, its loop user block still calls only `system_process();`, all modified user C/H files remain in `Core/User/`, and no interrupt callback was changed.

- [ ] **Step 4: Check diffs and preserve unrelated changes**

Run:

```powershell
git diff --check
git status --short
git diff -- Core/User/ad9834.c Core/User/ad9834.h README.md tests/test_dds_contract.py
```

Expected: no whitespace errors; source changes are limited to the AD9834 feature and README/tests; `Core/User/dds_control.h`, `AGENTS.md`, and Debug artifacts remain unstaged and unmodified by this implementation.

- [ ] **Step 5: Report results and wait for push permission**

Report the new APIs, compatibility behavior, test count, build result, commits, and any hardware behavior not verified on-device. Do not push.
