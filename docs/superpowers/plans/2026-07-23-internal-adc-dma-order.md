# Internal ADC DMA Half Ordering Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让片上双 ADC 在 DMA 前后半区标志同时积压时按真实期望顺序连续送入 FFT，避免 65536 点窗口反复重同步。

**Architecture:** 在 `adc_dual.c` 内增加 `adc_dual_expected_half`，并用一个私有半区处理函数统一更新统计、复制样本和推进期望状态。`adc_dual_process()` 同时收到两个标志时按期望顺序处理两块；只有非期望的单标志才记录丢样并重同步。

**Tech Stack:** STM32H743、STM32 HAL ADC/DMA/TIM、C11、Python `unittest` 静态契约测试、STM32CubeIDE 1.19.0 GNU Arm 工具链。

## Global Constraints

- 保持片上 ADC 每通道 600 kSPS。
- 保持 `ADC_DUAL_DMA_WORD_COUNT` 为 1024 对样本。
- 保持 FFT 长度为 65536 点。
- 不修改 `.ioc` 或 CubeMX 生成的 ADC、DMA、TIM2 初始化代码。
- ADC 中断回调继续只给各自唯一的 `xxx_flag` 赋值。
- 不改变片上 ADC、采集源管理器或 FFT 的公共接口。
- 用户代码继续位于 `Core/User/`，新增函数和变量使用小写下划线命名并补充中文注释。
- 只提交本计划列出的文件，不推送远程仓库。

---

### Task 1: 用契约测试定义半区顺序行为

**Files:**
- Modify: `tests/test_600ksps_fft_contract.py:174-185`
- Test: `tests/test_600ksps_fft_contract.py`

**Interfaces:**
- Consumes: `Core/User/adc_dual.c` 源码文本。
- Produces: `test_adc_dual_orders_both_pending_halves_without_resync`。

- [ ] **Step 1: 增加失败回归测试**

在 `SourceContractTest` 中、回调测试之前增加：

```python
def test_adc_dual_orders_both_pending_halves_without_resync(self):
    source = (ROOT / "Core/User/adc_dual.c").read_text(encoding="utf-8")
    self.assertIn("static uint8_t adc_dual_expected_half;", source)
    self.assertGreaterEqual(
        source.count("adc_dual_expected_half = 0u;"),
        3,
    )
    self.assertIn(
        "static void adc_dual_process_completed_half(uint8_t half_index)",
        source,
    )

    process_body = source.split(
        "void adc_dual_process(void)", 1
    )[1].split("\n}", 1)[0]
    dual_branch = process_body.split(
        "if ((half_flag != 0u) && (full_flag != 0u))", 1
    )[1].split(
        "else if (adc_dual_expected_half == 0u)", 1
    )[0]
    self.assertIn("adc_dual_stats.backlog_count++;", dual_branch)
    self.assertNotIn(
        "adc_dual_stats.dropped_pair_count += ADC_DUAL_DMA_WORD_COUNT;",
        dual_branch,
    )
    self.assertNotIn("measurement_fft_resynchronize();", dual_branch)
    self.assertGreaterEqual(
        dual_branch.count("adc_dual_process_completed_half("),
        4,
    )
    self.assertIn(
        "adc_dual_stats.dropped_pair_count +=\n"
        "                ADC_DUAL_DMA_HALF_WORD_COUNT;",
        process_body,
    )
    self.assertIn("measurement_fft_resynchronize();", process_body)
```

- [ ] **Step 2: 运行单项测试并确认 RED**

Run:

```powershell
python -m unittest tests.test_600ksps_fft_contract.SourceContractTest.test_adc_dual_orders_both_pending_halves_without_resync -v
```

Expected: `FAIL`，首先报告找不到
`static uint8_t adc_dual_expected_half;`。

- [ ] **Step 3: 提交失败测试**

```powershell
git add -- tests/test_600ksps_fft_contract.py
git commit -m "test: define internal ADC DMA half ordering"
```

---

### Task 2: 实现片上 ADC 半区期望状态机

**Files:**
- Modify: `Core/User/adc_dual.c:16-425`
- Test: `tests/test_600ksps_fft_contract.py`

**Interfaces:**
- Consumes: `adc_dual_process_block(uint32_t start_index, uint32_t word_count)`。
- Produces: 私有函数 `static void adc_dual_process_completed_half(uint8_t half_index)`；
  公共 `adc_dual_process()` 签名不变。

- [ ] **Step 1: 增加期望半区状态**

在三个 DMA 标志之后增加：

```c
/** 下一个可信 DMA 半区，0 表示前半区，1 表示后半区。 */
static uint8_t adc_dual_expected_half;
```

在 `adc_dual_init()`、`adc_dual_start()` 和 `adc_dual_stop()` 清除 DMA 标志的位置
分别增加：

```c
adc_dual_expected_half = 0u;
```

- [ ] **Step 2: 增加统一半区处理函数**

在 `adc_dual_process_block()` 之后增加：

```c
/**
 * @brief 处理一个已经确认完成的 DMA 半区并推进期望顺序。
 * @param half_index 半区编号，0 为前半区，1 为后半区。
 * @return 无。
 * @note 只由主循环调用；调用前必须确认对应 DMA 标志已经被领取。
 */
static void adc_dual_process_completed_half(uint8_t half_index)
{
    if (half_index == 0u)
    {
        adc_dual_stats.dma_half_count++;
        adc_dual_process_block(0u, ADC_DUAL_DMA_HALF_WORD_COUNT);
        adc_dual_expected_half = 1u;
    }
    else
    {
        adc_dual_stats.dma_full_count++;
        adc_dual_process_block(ADC_DUAL_DMA_HALF_WORD_COUNT,
                               ADC_DUAL_DMA_HALF_WORD_COUNT);
        adc_dual_expected_half = 0u;
    }
}
```

- [ ] **Step 3: 用期望顺序替换积压分支**

将 `adc_dual_process()` 的半区处理部分替换为：

```c
    if ((half_flag != 0u) && (full_flag != 0u))
    {
        adc_dual_stats.backlog_count++;
        if (adc_dual_expected_half == 0u)
        {
            adc_dual_process_completed_half(0u);
            adc_dual_process_completed_half(1u);
        }
        else
        {
            adc_dual_process_completed_half(1u);
            adc_dual_process_completed_half(0u);
        }
    }
    else if (adc_dual_expected_half == 0u)
    {
        if (half_flag != 0u)
        {
            adc_dual_process_completed_half(0u);
        }
        else if (full_flag != 0u)
        {
            adc_dual_stats.dropped_pair_count +=
                ADC_DUAL_DMA_HALF_WORD_COUNT;
            if (measurement_fft_sampling_required() != 0u)
            {
                measurement_fft_resynchronize();
            }
        }
    }
    else
    {
        if (full_flag != 0u)
        {
            adc_dual_process_completed_half(1u);
        }
        else if (half_flag != 0u)
        {
            adc_dual_stats.dropped_pair_count +=
                ADC_DUAL_DMA_HALF_WORD_COUNT;
            if (measurement_fft_sampling_required() != 0u)
            {
                measurement_fft_resynchronize();
            }
        }
    }
```

- [ ] **Step 4: 运行片上 ADC/FFT 契约测试并确认 GREEN**

Run:

```powershell
python -m unittest tests.test_600ksps_fft_contract -v
```

Expected: 该模块全部测试通过。

- [ ] **Step 5: 检查差异并提交**

```powershell
git diff --check -- Core/User/adc_dual.c tests/test_600ksps_fft_contract.py
git add -- Core/User/adc_dual.c
git commit -m "fix: preserve internal ADC DMA half order"
```

---

### Task 3: 回归、编译与实板验证

**Files:**
- Verify: `Core/User/adc_dual.c`
- Verify: `tests/test_600ksps_fft_contract.py`
- Build output: `Debug/h743_pre1.elf`

**Interfaces:**
- Consumes: Task 2 的内部 ADC 半区状态机。
- Produces: 可烧录固件和实板验收结果。

- [ ] **Step 1: 运行 ADS8688 与片上 ADC 专项测试**

Run:

```powershell
python -m unittest tests.test_600ksps_fft_contract tests.test_ads8688_input_contract -v
```

Expected: 两组测试全部通过。

- [ ] **Step 2: 运行全量测试**

Run:

```powershell
python -m unittest discover -s tests -v
```

Expected: 允许保留用户已确认的 `test_main_user_regions_remain_thin` 失败和主机
C 编译器测试跳过；其余测试通过。

- [ ] **Step 3: 使用 CubeIDE 工具链编译**

Run:

```powershell
$toolchain = 'D:\CubeIDE\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344\tools\bin'
$make_dir = 'D:\CubeIDE\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845\tools\bin'
$env:Path = "$toolchain;$make_dir;$env:Path"
& "$make_dir\make.exe" -C Debug -j4 all
```

Expected: exit code `0` 并生成 `Debug/h743_pre1.elf`。

- [ ] **Step 4: 检查提交范围**

Run:

```powershell
git diff --check HEAD~2..HEAD -- Core/User/adc_dual.c tests/test_600ksps_fft_contract.py
git log -4 --oneline
```

Expected: 无空白错误，日志包含失败测试与驱动修复提交。

- [ ] **Step 5: 烧录后观察实板**

全速运行，不在 DMA 处理循环中设置断点，观察：

```text
adc_dual_expected_half
adc_dual_stats.backlog_count
adc_dual_stats.dropped_pair_count
measurement_fft_capture_index
measurement_fft_diagnostics.capture_resync_count
measurement_fft_diagnostics.window_count
measurement_fft_diagnostics.fft_count
```

Expected: `backlog_count` 可以增加，但正常双标志积压不再同步增加
`capture_resync_count` 或 `dropped_pair_count`；约 111 ms 后 `window_count` 和
`fft_count` 增加。

