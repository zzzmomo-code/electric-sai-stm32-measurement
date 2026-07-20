# Current Project README Sync Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rewrite README and synchronize stale source-contract tests with the current, hardware-tested STM32 project before committing and pushing the complete workspace to `origin/h743_pre1`.

**Architecture:** Treat `h743_pre1.ioc` and the current `Core/User` interfaces as the source of truth. Documentation describes the existing implementation; tests assert the existing calibrated frequency formula and HMI data boundaries without changing production behavior.

**Tech Stack:** STM32CubeIDE 1.19.0, STM32H743 HAL/CMSIS, C11, Python `unittest`, Git.

## Global Constraints

- Do not modify current production behavior.
- PA4 is DAC1_OUT1; PC4 is ADC1_INP4/CH1; PB1 is ADC2_INP5/CH2.
- Use the current inverse FFT correction coefficients from `measurement_fft.h`.
- Commit the current workspace, including CubeIDE configuration and Debug output, as explicitly requested.
- Push only with `git push origin HEAD:h743_pre1`; never push remote `main` and never force-push.

---

### Task 1: Synchronize the FFT and HMI contracts

**Files:**
- Modify: `tests/test_600ksps_fft_contract.py`

**Interfaces:**
- Consumes: `measurement_fft_calibrate_frequency(float)` and the current HMI use of `measurement_result` plus ADC health statistics.
- Produces: static assertions matching current frequency coefficients and HMI data-source boundaries.

- [ ] **Step 1: Update the expected FFT constants and reference math**

Replace the old forward-fit constants with:

```python
"#define MEASUREMENT_FFT_LOW_FREQUENCY_GAIN 1.0000193004f"
"#define MEASUREMENT_FFT_LOW_FREQUENCY_OFFSET_HZ (0.24140466f)"
"#define MEASUREMENT_FFT_HIGH_FREQUENCY_GAIN 1.0000414617f"
"#define MEASUREMENT_FFT_HIGH_FREQUENCY_OFFSET_HZ (0.3261338f)"
```

Use the same constants in the Python boundary calculation and keep 40000 Hz in the low-frequency branch.

- [ ] **Step 2: Narrow the HMI ADC-stat assertion to the actual contract**

Retain assertions that both channels' DC voltage and Vpp come from `measurement_result`. Replace the blanket ban on `adc_dual_get_stats` with assertions proving ADC statistics are only used for `ADC_DUAL_STATE_ERROR` and `overflow_count`, not as voltage/amplitude fields.

- [ ] **Step 3: Run the focused tests**

Run:

```powershell
python -m unittest tests.test_600ksps_fft_contract -v
```

Expected: failures remain only where README has not yet been rewritten; frequency formula and HMI source tests pass.

### Task 2: Rewrite README from current project state

**Files:**
- Modify: `README.md`
- Modify: `tests/test_vga_control_contract.py`

**Interfaces:**
- Consumes: `h743_pre1.ioc`, `Core/User/*.h`, and the current system initialization/process flow.
- Produces: one authoritative setup, operation, calibration, testing, and troubleshooting document.

- [ ] **Step 1: Update README contract expectations**

Require the current facts, including:

```python
"PA4 / DAC1_OUT1"
"PC4 / ADC1_INP4"
"PB1 / ADC2_INP5"
"vga_control_set_level"
"vga_control_gain_from_level"
"VG = (20 / 33) * VPA4_MEASURED - 1"
"measurement_fft_calibrate_frequency"
"1.0000193004"
"1.0000414617"
```

- [ ] **Step 2: Rewrite README**

Document project purpose, toolchain, pin mapping, CubeMX peripheral values, system data flow, FFT raw/calibrated fields, VGA command/measured voltage separation, HMI controls, module layout, initialization, build/flash procedure, offline tests, board verification, diagnostics, and known limitations.

- [ ] **Step 3: Run README-related tests**

Run:

```powershell
python -m unittest tests.test_600ksps_fft_contract.SourceContractTest.test_readme_documents_fft_frequency_calibration tests.test_vga_control_contract.VgaControlContractTest.test_readme_documents_dac_vga_control -v
```

Expected: both tests pass.

### Task 3: Verify and publish the complete workspace

**Files:**
- Stage: all current tracked and untracked project files, excluding files already ignored by Git.

**Interfaces:**
- Consumes: completed documentation/test synchronization and all existing user changes.
- Produces: a verified local commit and updated `origin/h743_pre1`.

- [ ] **Step 1: Run the full contract suite**

```powershell
python -m unittest discover -s tests -p 'test_*.py' -v
```

Expected: 42 tests pass with zero failures.

- [ ] **Step 2: Run compiler/build verification**

Run the existing STM32CubeIDE Debug build or the available ARM GCC verification and require exit code 0. Record any warnings without claiming a warning-free build unless the output proves it.

- [ ] **Step 3: Review and commit all current changes**

```powershell
git diff --check
git add -A
git diff --cached --check
git commit -m "feat: integrate measurement runtime and refresh documentation"
```

Expected: commit succeeds and `git status --short` is empty.

- [ ] **Step 4: Push only the requested remote branch**

```powershell
git fetch origin h743_pre1
git push origin HEAD:h743_pre1
```

Expected: normal fast-forward push succeeds. If rejected as non-fast-forward, stop without force-pushing.

- [ ] **Step 5: Verify the remote commit**

```powershell
git ls-remote origin refs/heads/h743_pre1
git rev-parse HEAD
```

Expected: both commit IDs are identical.
