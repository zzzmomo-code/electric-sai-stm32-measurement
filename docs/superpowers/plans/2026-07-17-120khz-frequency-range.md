# 120 kHz FFT Frequency Range Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将主峰搜索和串口屏 64 点频谱的统一上限从 100 kHz 扩展到 120 kHz。

**Architecture:** 保留现有单一上限常量作为主峰搜索与 HMI 频谱压缩的共同数据源，只修改常量值并同步注释、测试和 README。采样率、FFT 点数、Nyquist 边界、诊断结构体与外设配置保持不变。

**Tech Stack:** STM32H743VIT6、STM32CubeIDE 1.19.0、C11、Python unittest。

## Global Constraints

- 所有用户代码继续位于 `Core/User/`，不修改 CubeMX 生成的外设初始化代码或 `.ioc`。
- `MEASUREMENT_FFT_SAMPLE_RATE_HZ` 保持 `600000.0f`，`MEASUREMENT_FFT_LENGTH` 保持 `65536u`。
- 主峰搜索与 HMI 频谱必须共同使用 `MEASUREMENT_FFT_SPECTRUM_MAX_HZ`，其值为 `120000.0f`。
- 保留全部 `measurement_fft_diagnostics_t` 字段和现有公共接口。
- 64 点频谱覆盖 0 至 120 kHz，每点宽度为 1875 Hz；FFT 本征频点间隔仍约为 9.155273 Hz。
- 120 kHz 输入只保证二次谐波 240 kHz 位于 Nyquist 内，不承诺完整三次及以上谐波、THD或非正弦波分类。
- 不处理或提交工作区内与本功能无关的已有改动。

---

### Task 1: 扩展统一频率范围并同步契约

**Files:**
- Modify: `tests/test_600ksps_fft_contract.py:40-45`
- Modify: `Core/User/measurement_fft.h:33-36,54,194`
- Modify: `Core/User/measurement_fft.c:281,542`
- Modify: `README.md:35,69-71,92`

**Interfaces:**
- Consumes: `MEASUREMENT_FFT_SAMPLE_RATE_HZ`、`MEASUREMENT_FFT_LENGTH` 和现有 `MEASUREMENT_FFT_SPECTRUM_MAX_HZ`。
- Produces: `MEASUREMENT_FFT_SPECTRUM_MAX_HZ = 120000.0f`，供主峰搜索与 `measurement_fft_build_spectrum()` 共同使用；不新增函数或结构体字段。

- [ ] **Step 1: 先把契约测试期望改为 120 kHz**

将 `test_measurement_constants_are_updated()` 中的上限断言改为：

```python
self.assertIn("#define MEASUREMENT_FFT_SPECTRUM_MAX_HZ 120000.0f", header)
```

- [ ] **Step 2: 运行单项测试并确认按预期失败**

Run:

```powershell
python -m unittest tests.test_600ksps_fft_contract.SourceContractTest.test_measurement_constants_are_updated -v
```

Expected: `FAIL`，失败信息显示头文件仍包含 `100000.0f` 而不是 `120000.0f`。

- [ ] **Step 3: 最小修改统一频率上限**

在 `Core/User/measurement_fft.h` 中改为：

```c
/** 压缩频谱和主峰搜索覆盖的最高频率，单位为 Hz。 */
#define MEASUREMENT_FFT_SPECTRUM_MAX_HZ 120000.0f
```

同步把 `measurement_fft.h` 和 `measurement_fft.c` 中三处“0 至 100 kHz”或“1 Hz 至 100 kHz”中文说明改为 120 kHz，不改变任何计算表达式。

- [ ] **Step 4: 更新 README 的范围和板上验证说明**

将功能范围改为“0 至 120 kHz”，把验证频点列表扩展为“20 Hz、1 kHz、20 kHz、100 kHz、120 kHz”，并明确加入：

```markdown
- 120 kHz 时每周期约 5 个样本；二次谐波 240 kHz 仍低于 300 kHz Nyquist，三次及以上谐波越界，因此高频端 THD 和波形分类仅供参考。
```

- [ ] **Step 5: 运行单项测试确认转绿**

Run:

```powershell
python -m unittest tests.test_600ksps_fft_contract.SourceContractTest.test_measurement_constants_are_updated -v
```

Expected: `OK`，1 test passed。

- [ ] **Step 6: 运行全部主机测试**

Run:

```powershell
python -m unittest discover -s tests -p 'test_*.py' -v
```

Expected: 全部测试通过，0 failures，0 errors。

- [ ] **Step 7: 检查旧范围残留和差异格式**

Run:

```powershell
Select-String -Path 'Core\User\measurement_fft.c','Core\User\measurement_fft.h','tests\test_600ksps_fft_contract.py','README.md' -Pattern '100000.0f|0 至 100 kHz|1 Hz 至 100 kHz'
git diff --check -- Core/User/measurement_fft.h Core/User/measurement_fft.c tests/test_600ksps_fft_contract.py README.md
```

Expected: 第一个命令无匹配；`git diff --check` 无输出且退出码为 0。

- [ ] **Step 8: 执行 Debug Clean Build**

在 STM32CubeIDE 1.19.0 中选择 Debug 配置，执行 `Project -> Clean...` 后 `Project -> Build Project`。

Expected: Build Finished，0 errors；确认用户模块仍以 `-O2` 编译，防止 600 kSPS 下 `backlog_count` 增长。

- [ ] **Step 9: 板上验收**

输入双通道同频 120 kHz、1.0 Vpp、1.65 V 偏置正弦波，预期：

```text
peak_bin                           ≈ 13107
secondary_peak_bin                 ≈ 13107
peak_frequency_hz                  ≈ 120000 Hz
secondary_peak_frequency_hz        ≈ 120000 Hz
adc_dual_stats.backlog_count       = 0
adc_dual_stats.overflow_count      = 0
adc_dual_stats.error_count         = 0
```

- [ ] **Step 10: 仅在Git身份配置完成后提交本功能文件**

```powershell
git add Core/User/measurement_fft.h Core/User/measurement_fft.c tests/test_600ksps_fft_contract.py README.md docs/superpowers/specs/2026-07-17-120khz-frequency-range-design.md docs/superpowers/plans/2026-07-17-120khz-frequency-range.md
git commit -m "feat: extend FFT measurement range to 120 kHz"
```

Expected: 仅提交上述文件；如果 `user.name` 或 `user.email` 未配置，停止提交并如实报告，不擅自设置Git身份。
