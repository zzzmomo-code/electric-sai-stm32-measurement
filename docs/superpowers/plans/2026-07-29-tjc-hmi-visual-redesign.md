# TJC HMI Visual Redesign Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce `fpga1_codex_ui_v1.HMI` with the approved 800×480 instrument layout while preserving the original HMI, STM32 object-name contract, serial commands, and 350×210 plot areas.

**Architecture:** Treat `fpga1.HMI` as an immutable binary baseline, duplicate it, and perform all GUI changes only in the copy through USART HMI. Validate the proprietary binary through source hashes, editor compilation, a frozen-control checklist, simulator commands, and screenshots rather than text diffs.

**Tech Stack:** 淘晶驰 USART HMI editor, 800×480 TJC screen project, PowerShell, Git

---

### Task 1: Protect and duplicate the baseline

**Files:**
- Read only: `fpga1.HMI`
- Create: `fpga1_codex_ui_v1.HMI`
- Create: `docs/hmi/fpga1_codex_ui_v1_validation.md`

- [ ] **Step 1: Close the source project in USART HMI**

Close the currently open `fpga1.HMI` window so Windows releases the file lock. Do not use “Save As” from a modified editor state.

- [ ] **Step 2: Record the immutable source hash**

Run:

```powershell
$source = 'fpga1.HMI'
$copy = 'fpga1_codex_ui_v1.HMI'
$sourceHashBefore = (Get-FileHash -Algorithm SHA256 -LiteralPath $source).Hash
$sourceHashBefore
```

Expected: one 64-character hexadecimal SHA-256 value and no file-lock error.

- [ ] **Step 3: Create the independent HMI copy**

Run:

```powershell
Copy-Item -LiteralPath $source -Destination $copy
$copyHashInitial = (Get-FileHash -Algorithm SHA256 -LiteralPath $copy).Hash
if ($copyHashInitial -ne $sourceHashBefore) { throw 'Initial HMI copy hash mismatch' }
```

Expected: command completes without output; both files initially have identical hashes.

- [ ] **Step 4: Create the validation record**

Create `docs/hmi/fpga1_codex_ui_v1_validation.md` with:

```markdown
# fpga1_codex_ui_v1.HMI validation

## File protection

- Source: `fpga1.HMI`
- Editable copy: `fpga1_codex_ui_v1.HMI`
- Source SHA-256 before editing: `<recorded value>`
- Source SHA-256 after editing: `<record after final validation>`

## Frozen interface

- `t_vpp`, `t_vrms`, `t_freq`, `t_status`
- `t_comp1`, `t_comp2`, `t_comp3`
- `b_t1`: `printh A5 01 5A`
- `b_t3`: `printh A5 02 5A`
- `s_t1`, `s_t3`: overlapping 350×210 time-domain charts
- `s_spec`: independent 350×210 spectrum chart
- `b_rsv1`, `b_rsv2`: no send-key and no event code

## USART HMI result

- Compile errors: not run
- Compile warnings: not run
- Simulator regression: not run
- Hardware download: not run
```

- [ ] **Step 5: Commit only the safety copy and validation record**

Run:

```powershell
git add -- fpga1_codex_ui_v1.HMI docs/hmi/fpga1_codex_ui_v1_validation.md
git diff --cached --name-only
git commit -m "chore: create protected HMI redesign copy"
```

Expected staged names: only the HMI copy and validation record.

### Task 2: Rebuild the approved visual hierarchy in the copy

**Files:**
- Modify: `fpga1_codex_ui_v1.HMI`

- [ ] **Step 1: Open only the copied HMI**

Open `fpga1_codex_ui_v1.HMI` in USART HMI and confirm the editor title bar shows the copied filename.

- [ ] **Step 2: Preserve project-level behavior**

Before moving controls, inspect and retain:

```text
page0 only
current baud setting
current bkcmd and recmod values
current backlight setting
page post-initialization visibility script
t_status.txt="WAIT FPGA"
```

Do not add a second page, a start-measurement control, or a spectrum-toggle control.

- [ ] **Step 3: Create the compact header**

Use the existing title/status controls where possible:

```text
Header target: x=24, y=10, w=752, h=40
Title: 周期信号测量分析装置, left aligned, about 20 px
t_status: right side of header, readable green status styling
```

Keep `t_status` as the firmware-written object.

- [ ] **Step 4: Arrange the primary measurement cards**

Place the existing value objects inside three white cards:

```text
t_vpp:  x=24,  y=60,  card w=170, h=50
t_vrms: x=204, y=60,  card w=170, h=50
t_freq: x=384, y=60,  card w=170, h=50
```

Use a 14–16 px label and about 20 px value. Keep units inside the existing firmware-provided text.

- [ ] **Step 5: Arrange the component cards**

Place:

```text
t_comp1: x=24,  y=120, card w=170, h=46
t_comp2: x=204, y=120, card w=170, h=46
t_comp3: x=384, y=120, card w=170, h=46
```

Use the largest existing GBK/ASCII font that displays `H1 12.345 kHz 1.650 V` on one line without truncation.

- [ ] **Step 6: Arrange four medium buttons**

Place:

```text
b_t1:   x=570, y=60,  w=96, h=44, text=1周期
b_t3:   x=680, y=60,  w=96, h=44, text=3周期
b_rsv1: x=570, y=120, w=96, h=44, text=预留
b_rsv2: x=680, y=120, w=96, h=44, text=预留
```

Set button text to about 17–18 px. For `b_rsv1` and `b_rsv2`, leave press/release event editors empty and keep “发送键值” unchecked.

- [ ] **Step 7: Preserve button protocol events**

Confirm the press events remain exactly:

```text
b_t1: printh A5 01 5A
b_t3: printh A5 02 5A
```

- [ ] **Step 8: Arrange the two chart regions**

Place chart titles near `y=179`, then preserve:

```text
left data area:  350×210, top y=209
right data area: 350×210, top y=209
s_t1 and s_t3: identical left coordinates
s_spec: right coordinates only
```

Keep channel configuration unchanged. Keep percentage labels outside the 350×210 control rectangles.

- [ ] **Step 9: Apply the approved palette**

Use editor color pickers to approximate:

```text
page: light gray-white
cards: white
primary/button: restrained navy blue
status: green
chart background: dark navy/black
time waveform: cyan
spectrum: amber
grid: subdued blue-gray/green with less emphasis than the curve
```

Do not add gradients, large images, animated decorations, or unsupported transparency.

- [ ] **Step 10: Save the copied HMI**

Use Save and confirm the title still shows `fpga1_codex_ui_v1.HMI`.

### Task 3: Compile and run HMI regression checks

**Files:**
- Modify: `fpga1_codex_ui_v1.HMI`
- Modify: `docs/hmi/fpga1_codex_ui_v1_validation.md`

- [ ] **Step 1: Compile in USART HMI**

Click “编译”.

Expected:

```text
编译成功
0 个错误
```

Record warning count separately; inspect every warning before acceptance.

- [ ] **Step 2: Check text-object compatibility in the simulator**

Send these existing-format commands with the normal TJC `FF FF FF` terminators:

```text
t_vpp.txt="3.300 V"
t_vrms.txt="1.166 V"
t_freq.txt="12.345 kHz"
t_status.txt="READY #4294 DROP 12"
t_comp1.txt="H1 12.345 kHz 1.650 V"
t_comp2.txt="H2 24.690 kHz 0.825 V"
t_comp3.txt="H3 37.035 kHz 0.412 V"
```

Expected: all strings remain on one line, remain legible, and do not overlap buttons.

- [ ] **Step 3: Check chart loading**

For each of `s_t1`, `s_t3`, and `s_spec`, clear channel 0 and add 350 test values.

Expected:

```text
350 points fill the entire horizontal data region
s_t1 and s_t3 occupy exactly the same left plot
s_spec remains visible in the independent right plot
```

- [ ] **Step 4: Check visibility switching**

Run:

```text
vis s_t1,1
vis s_t3,0
vis s_spec,1
```

Then:

```text
vis s_t1,0
vis s_t3,1
vis s_spec,1
```

Expected: only the left time-domain curve changes; the right spectrum remains present.

- [ ] **Step 5: Check button serial output**

Use the editor simulator serial monitor:

```text
b_t1 press -> A5 01 5A
b_t3 press -> A5 02 5A
b_rsv1 press -> no bytes
b_rsv2 press -> no bytes
```

- [ ] **Step 6: Update the validation record**

Replace the “not run” entries with the actual compile, warning, simulator, and hardware status. Add a screenshot path for the final compiled page.

- [ ] **Step 7: Commit the completed HMI**

Run:

```powershell
git add -- fpga1_codex_ui_v1.HMI docs/hmi/fpga1_codex_ui_v1_validation.md
git diff --cached --name-only
git commit -m "feat: add clean instrument-style TJC HMI"
```

Expected staged names: only the copied HMI and its validation record.

### Task 4: Prove the source was not changed

**Files:**
- Read only: `fpga1.HMI`
- Modify: `docs/hmi/fpga1_codex_ui_v1_validation.md`

- [ ] **Step 1: Recompute both hashes**

Run:

```powershell
$sourceHashAfter = (Get-FileHash -Algorithm SHA256 -LiteralPath 'fpga1.HMI').Hash
$copyHashFinal = (Get-FileHash -Algorithm SHA256 -LiteralPath 'fpga1_codex_ui_v1.HMI').Hash
$sourceHashAfter
$copyHashFinal
```

Expected: source hash equals the value recorded before copying; final copy hash differs from the source hash.

- [ ] **Step 2: Record the final hashes**

Update `docs/hmi/fpga1_codex_ui_v1_validation.md` with both final values and explicitly state:

```text
Original fpga1.HMI unchanged: PASS
```

- [ ] **Step 3: Verify repository scope**

Run:

```powershell
git status --short -- fpga1.HMI fpga1_codex_ui_v1.HMI docs/hmi/fpga1_codex_ui_v1_validation.md
git log -3 --oneline
```

Expected: `fpga1.HMI` has no new modification, while the copy and validation record are committed.

- [ ] **Step 4: Commit the final validation record**

Run:

```powershell
git add -- docs/hmi/fpga1_codex_ui_v1_validation.md
git commit -m "docs: record TJC HMI redesign validation"
```

Do not stage unrelated CubeIDE settings, firmware sources, build artifacts, or `.superpowers/` scratch files.
