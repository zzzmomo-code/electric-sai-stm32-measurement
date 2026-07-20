# DDS ADC Frequency Compensation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an HMI-triggered DDS control mode that performs one TIM5 coarse setting followed by five ADC/FFT-guided frequency corrections, then holds the final DDS frequency.

**Architecture:** `dds_control` owns a sequence-driven state machine and consumes only new valid `measurement_result` snapshots. The HMI `M` command requests both immediate TIM5 measurement and DDS compensation; every DDS change resynchronizes FFT capture so the next result belongs to the new output frequency.

**Tech Stack:** STM32CubeIDE 1.19.0, STM32H743 HAL/CMSIS, C11, Python `unittest`, Git.

## Global Constraints

- The low-side relation is `fADC = fEXTERNAL - fDDS`.
- Each correction is `fDDS_new = fDDS_current + (fADC - 100000)`.
- The TIM5 coarse write does not count toward the five ADC-guided corrections.
- Only successful DDS writes count; invalid and duplicate FFT results do not count.
- After the fifth correction, hold the final DDS frequency until the next `M` command.
- Do not add work to UART, TIM3, ADC, or DMA interrupt callbacks.
- Do not modify `.ioc`, GPIO, clocks, DMA, or NVIC configuration.
- All new or modified user code receives Chinese comments and remains under `Core/User`.
- Implement and verify locally; do not push without a new explicit push instruction.

---

### Task 1: Define failing DDS compensation contracts

**Files:**
- Modify: `tests/test_dds_contract.py`

**Interfaces:**
- Consumes: proposed `void dds_control_request_compensation(void)` and current `measurement_result_t`.
- Produces: regression coverage for API, formula, state transitions, FFT sequence gating, HMI trigger, and five-step hold behavior.

- [ ] **Step 1: Add source-contract tests**

Add tests requiring these public/state tokens:

```python
self.assertIn("#define DDS_CONTROL_COMPENSATION_LIMIT 5u", header)
self.assertIn("dds_control_state_coarse", header)
self.assertIn("dds_control_state_compensating", header)
self.assertIn("dds_control_state_holding", header)
self.assertIn("void dds_control_request_compensation(void);", header)
self.assertIn("compensation_count", header)
self.assertIn("compensation_request_count", header)
self.assertIn("adc_frequency_hz", header)
self.assertIn("frequency_error_hz", header)
self.assertIn("last_result_sequence", header)
```

Require `dds_control.c` to read `measurement_result_get_snapshot`, test
`MEASUREMENT_VALID_FREQUENCY`, reject repeated `result.sequence`, calculate signed error from
`DDS_CONTROL_TARGET_IF_HZ`, call `measurement_fft_resynchronize()`, increment only after
`dds_control_apply_output()` succeeds, and select `dds_control_state_holding` at count 5.

- [ ] **Step 2: Add mathematical direction and boundary tests**

Use a Python reference helper:

```python
def compensate(current_dds_hz, adc_hz):
    error_hz = round(adc_hz - 100_000.0)
    return current_dds_hz + error_hz

self.assertEqual(900_250, compensate(900_000, 100_250.0))
self.assertEqual(899_750, compensate(900_000, 99_750.0))
self.assertEqual(900_000, compensate(900_000, 100_000.0))
```

- [ ] **Step 3: Add HMI trigger contract**

Extract the `M`/`m` branch from `Core/User/hmi_tjc.c` and assert it contains both:

```c
frequency_measure_request_now();
dds_control_request_compensation();
```

- [ ] **Step 4: Run tests and verify RED**

Run:

```powershell
python -m unittest tests.test_dds_contract -v
```

Expected: new compensation tests fail because the API, states, diagnostics, and HMI call do not exist.

- [ ] **Step 5: Commit the failing contracts**

```powershell
git add tests/test_dds_contract.py
git commit -m "test: define ADC-guided DDS compensation contract"
```

### Task 2: Implement the DDS compensation state machine

**Files:**
- Modify: `Core/User/dds_control.h`
- Modify: `Core/User/dds_control.c`
- Modify: `Core/User/hmi_tjc.c`
- Test: `tests/test_dds_contract.py`

**Interfaces:**
- Consumes: `frequency_measure_hz`, `measurement_result_get_snapshot()`, `measurement_fft_resynchronize()`, and `ad9834_set_frequency_hz()`.
- Produces: `dds_control_request_compensation()`, five-step compensation states, and diagnostics.

- [ ] **Step 1: Extend the public header**

Define:

```c
#define DDS_CONTROL_COMPENSATION_LIMIT 5u

void dds_control_request_compensation(void);
```

Add `coarse`, `compensating`, and `holding` enum states. Add these diagnostics with exact types:

```c
uint8_t compensation_count;
uint32_t compensation_request_count;
float adc_frequency_hz;
int32_t frequency_error_hz;
uint32_t last_result_sequence;
```

- [ ] **Step 2: Add internal request and safe correction helpers**

Maintain a module-private `uint8_t dds_control_compensation_requested`. Implement signed target
calculation using `int64_t` and reject values below 1 or above `AD9834_MAX_OUTPUT_HZ` before
converting to `uint32_t`.

Change `dds_control_apply_output()` to return `uint8_t`: return 1 only after a successful
`ad9834_set_frequency_hz()`, otherwise record the existing error diagnostics and return 0.

- [ ] **Step 3: Implement request initialization**

`dds_control_request_compensation()` sets the request flag and increments
`compensation_request_count`. `dds_control_init()` clears all new fields and the private flag.

- [ ] **Step 4: Implement coarse-start processing**

When the request flag is observed in formal mode:

1. clear the request flag and compensation count;
2. round `frequency_measure_hz` and validate it through `dds_control_calculate_output_hz()`;
3. if unavailable, remain in `coarse` so a later loop can consume the requested TIM5 result;
4. on successful DDS write, snapshot the current result sequence, clear ADC/error diagnostics,
   call `measurement_fft_resynchronize()`, and enter `compensating`.

Do not run the old automatic 500 Hz TIM5 tracking path while no request is active.

- [ ] **Step 5: Implement sequence-driven ADC corrections**

In `compensating`:

1. read a measurement result snapshot;
2. ignore missing, invalid, frequency-invalid, non-finite, non-positive, or repeated sequences;
3. record the consumed sequence, ADC frequency, and rounded signed error;
4. calculate and validate the new DDS target;
5. write DDS and increment `compensation_count` only on success;
6. at count 5 enter `holding`; otherwise resynchronize FFT and await another result.

- [ ] **Step 6: Connect the existing M command**

In the existing `M`/`m` command branch, preserve `frequency_measure_request_now()` and add:

```c
dds_control_request_compensation();
```

Keep command processing in `hmi_tjc_process_input()` and do not change the UART callback.

- [ ] **Step 7: Run focused tests and verify GREEN**

```powershell
python -m unittest tests.test_dds_contract -v
```

Expected: all DDS contract tests pass.

- [ ] **Step 8: Commit the DDS state machine**

```powershell
git add Core/User/dds_control.c Core/User/dds_control.h Core/User/hmi_tjc.c
git commit -m "feat: compensate DDS from ADC intermediate frequency"
```

### Task 3: Document the compensation behavior

**Files:**
- Modify: `README.md`
- Test: `tests/test_dds_contract.py`

**Interfaces:**
- Consumes: the completed coarse-plus-five-correction workflow.
- Produces: user-facing setup and runtime documentation.

- [ ] **Step 1: Update README**

Document the exact trigger and sequence:

```text
M -> TIM5 immediate coarse frequency -> DDS = fTIM5 - 100 kHz
  -> five new valid ADC/FFT frames
  -> DDS += fADC - 100 kHz after each frame
  -> hold final DDS frequency
```

Document that another `M` restarts the sequence and that invalid FFT frames do not count.

- [ ] **Step 2: Run DDS and full contract tests**

```powershell
python -m unittest tests.test_dds_contract -v
python -m unittest discover -s tests -p 'test_*.py' -v
```

Expected: all DDS tests and the complete suite pass.

- [ ] **Step 3: Commit documentation**

```powershell
git add README.md
git commit -m "docs: explain ADC-guided DDS compensation"
```

### Task 4: Build and final verification

**Files:**
- Verify: all modified files and generated Debug artifacts.

**Interfaces:**
- Consumes: completed compensation implementation.
- Produces: evidence that the local branch is ready for user review.

- [ ] **Step 1: Run fresh complete tests**

```powershell
python -m unittest discover -s tests -p 'test_*.py' -v
```

Expected: zero failures.

- [ ] **Step 2: Build the Debug target**

Add the STM32CubeIDE ARM GCC and Make directories to the process PATH, then run:

```powershell
make -C Debug -j4 all
```

Expected: exit code 0 and a size summary for `h743_pre1.elf`.

- [ ] **Step 3: Review repository state**

```powershell
git diff --check
git status --short
git log -6 --oneline
```

Expected: only build artifacts generated by the verified build may remain modified. Commit those
artifacts only if they are already tracked and belong to this build; do not push.
