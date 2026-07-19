# ADC1 PA6/INP3 Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 完成 ADC1 从 PC4/INP4 到 PA6/INP3 的一致性迁移，并验证 CubeMX 配置、用户代码注释、README 和固件构建。

**Architecture:** CubeMX 生成区是 ADC 通道与 GPIO 初始化的唯一事实来源；用户模块不执行运行时重配置，只消费原有双 ADC DMA 数据。新增契约测试锁定 `.ioc`、`adc.c`、用户模块说明和 README 的引脚映射，避免以后重新生成时回退。

**Tech Stack:** STM32CubeIDE 1.19.0、STM32CubeMX 6.15.0、STM32H743VIT6、STM32 HAL、Python `unittest`、Git。

## Global Constraints

- 用户 `.c/.h` 只放在 `Core/User`，统一包含入口为 `Core/User/system.h`。
- 不手工修改 `.ioc` 或 CubeMX 自动生成初始化代码；当前生成结果只做审查。
- ADC1 保持 16 位、8.5 cycles、TIM2 TRGO 上升沿、Dual Regular Simultaneous 和 DMA Circular。
- ADC2 保持 PB1/ADC2_INP5；DMA 数据低 16 位仍为 ADC1，高 16 位仍为 ADC2。
- 所有新增或修改的用户代码使用小写下划线标识符并补齐中文注释。
- 保留用户现有 `.settings` 修改；未经确认不 push。

---

### Task 1: 锁定 ADC1 迁移契约并审查 CubeMX 生成结果

**Files:**
- Modify: `tests/test_600ksps_fft_contract.py`
- Inspect: `h743_pre1.ioc`
- Inspect: `Core/Src/adc.c`

**Interfaces:**
- Consumes: CubeMX 生成的 ADC1 Channel 3、PA6/ADC1_INP3 配置。
- Produces: `SourceContractTest.test_adc1_uses_pa6_inp3` 契约测试。

- [ ] **Step 1: 写入迁移契约测试**

在 `SourceContractTest` 中加入：

```python
    def test_adc1_uses_pa6_inp3(self):
        ioc = (ROOT / "h743_pre1.ioc").read_text(encoding="utf-8")
        adc_source = (ROOT / "Core/Src/adc.c").read_text(encoding="utf-8")
        self.assertIn("ADC1.Channel-0\\#ChannelRegularConversion=ADC_CHANNEL_3", ioc)
        self.assertIn("PA6.Signal=ADCx_INP3", ioc)
        self.assertIn("SH.ADCx_INP3.0=ADC1_INP3,IN3-Single-Ended", ioc)
        self.assertNotIn("PC4.Signal=ADCx_INP4", ioc)
        self.assertIn("sConfig.Channel = ADC_CHANNEL_3;", adc_source)
        self.assertIn("PA6     ------> ADC1_INP3", adc_source)
        self.assertIn("HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);", adc_source)
        self.assertIn("HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);", adc_source)
```

- [ ] **Step 2: 运行单项测试确认当前生成结果满足契约**

Run: `python -m unittest tests.test_600ksps_fft_contract.SourceContractTest.test_adc1_uses_pa6_inp3 -v`

Expected: `OK`。若失败，只报告不匹配的 CubeMX 参数，不手工修复生成区。

- [ ] **Step 3: 审查迁移无关的 CubeMX 文件变化**

Run: `git diff -- .cproject Core/Inc/spi.h .settings/language.settings.xml .settings/stm32cubeide.project.prefs`

Expected: 明确区分 CubeMX 自动刷新与用户既有修改；ADC 迁移提交只纳入经确认需要的文件。

- [ ] **Step 4: 提交契约测试和已生成的 ADC 配置**

```powershell
git add -- tests/test_600ksps_fft_contract.py h743_pre1.ioc Core/Src/adc.c
git commit -m "feat: migrate ADC1 input to PA6 INP3"
```

### Task 2: 更新用户模块的中文引脚说明

**Files:**
- Modify: `Core/User/adc_dual.c:7`
- Modify: `Core/User/adc_dual.h:7-8`
- Modify: `Core/User/measurement_fft.c:7`
- Modify: `Core/User/system.c:6`
- Test: `tests/test_600ksps_fft_contract.py`

**Interfaces:**
- Consumes: ADC1 CH1 为 PA6/ADC1_INP3，ADC2 CH2 为 PB1/ADC2_INP5。
- Produces: 与实际硬件一致的用户模块说明；不改变任何 C 接口或行为。

- [ ] **Step 1: 扩展契约测试检查用户说明**

在 `test_adc1_uses_pa6_inp3` 末尾加入：

```python
        documented_files = (
            "Core/User/adc_dual.c",
            "Core/User/adc_dual.h",
            "Core/User/measurement_fft.c",
            "Core/User/system.c",
        )
        for relative_path in documented_files:
            text = (ROOT / relative_path).read_text(encoding="utf-8")
            self.assertIn("PA6/ADC1_INP3", text)
            self.assertNotIn("PC4/ADC1_INP4", text)
```

- [ ] **Step 2: 运行测试并确认旧注释导致失败**

Run: `python -m unittest tests.test_600ksps_fft_contract.SourceContractTest.test_adc1_uses_pa6_inp3 -v`

Expected: FAIL，提示至少一个文件缺少 `PA6/ADC1_INP3`。

- [ ] **Step 3: 最小修改四处模块头说明**

将活动映射统一改为：

```c
 * GPIO 引脚映射：PA6/ADC1_INP3 为 CH1，PB1/ADC2_INP5 为 CH2。
```

`system.c` 保留其余引脚列表，只将 `PC4/ADC1_INP4` 替换为 `PA6/ADC1_INP3`。删除 `adc_dual.h` 中“计划使用”和“CubeMX 尚未生成”的过时描述，明确配置已经就绪。

- [ ] **Step 4: 运行测试确认通过**

Run: `python -m unittest tests.test_600ksps_fft_contract.SourceContractTest.test_adc1_uses_pa6_inp3 -v`

Expected: `OK`。

- [ ] **Step 5: 提交用户代码注释**

```powershell
git add -- Core/User/adc_dual.c Core/User/adc_dual.h Core/User/measurement_fft.c Core/User/system.c tests/test_600ksps_fft_contract.py
git commit -m "docs: update ADC1 PA6 module mappings"
```

### Task 3: 更新 README 的硬件、MX 配置和验证说明

**Files:**
- Modify: `README.md`
- Modify: `tests/test_600ksps_fft_contract.py`

**Interfaces:**
- Consumes: 已验证的 ADC1 PA6/INP3 CubeMX 参数。
- Produces: 可重复操作的 STM32CubeIDE 1.19.0 配置和实板验证步骤。

- [ ] **Step 1: 扩展 README 契约测试**

在 `test_adc1_uses_pa6_inp3` 末尾加入：

```python
        readme = (ROOT / "README.md").read_text(encoding="utf-8")
        self.assertIn("PA6：ADC1_INP3", readme)
        self.assertIn("IN3 Single-ended", readme)
        self.assertIn("ADC_CHANNEL_3", readme)
        self.assertNotIn("PC4/ADC1_INP4", readme)
```

- [ ] **Step 2: 运行测试并确认 README 尚未覆盖迁移**

Run: `python -m unittest tests.test_600ksps_fft_contract.SourceContractTest.test_adc1_uses_pa6_inp3 -v`

Expected: FAIL，提示 README 缺少 PA6/INP3 配置说明。

- [ ] **Step 3: 更新 README**

在“硬件”中增加 `PA6：ADC1_INP3，CH1 模拟输入` 和 `PB1：ADC2_INP5，CH2 模拟输入`；在“CubeMX 配置”中增加 ADC1/ADC2 双规则同步、Rank 1 Channel 3、8.5 cycles、TIM2 TRGO、DMA1 Stream0 Word/Word Circular Very High、NVIC 影响及重新生成步骤；在软件流程中说明 32 位 DMA 字拆包；在验证步骤中增加 PA6 输入、CH1 码值、DMA 计数和双通道同步检查；在限制中注明输入范围受 VDDA 和模拟前端驱动能力约束。

- [ ] **Step 4: 运行契约测试确认通过**

Run: `python -m unittest tests.test_600ksps_fft_contract.SourceContractTest.test_adc1_uses_pa6_inp3 -v`

Expected: `OK`。

- [ ] **Step 5: 提交 README**

```powershell
git add -- README.md tests/test_600ksps_fft_contract.py
git commit -m "docs: document ADC1 PA6 CubeMX setup"
```

### Task 4: 全量验证和交付审查

**Files:**
- Verify: `Core/Src/main.c`
- Verify: `Core/User/*.c`
- Verify: `Core/User/*.h`
- Verify: `README.md`

**Interfaces:**
- Consumes: Tasks 1-3 的配置、注释、文档和测试。
- Produces: 可复核的测试、编译和 Git 状态证据。

- [ ] **Step 1: 运行全部离线测试**

Run: `python -m unittest discover -s tests -p 'test_*.py' -v`

Expected: 所有测试 `OK`，无 failure 或 error。

- [ ] **Step 2: 检查格式和残留旧映射**

Run: `git diff --check`

Expected: 无输出。

Run: `Get-ChildItem Core\User,README.md -Recurse -File | Select-String -Pattern 'PC4/ADC1_INP4|PC4.*ADC1_INP4'`

Expected: 无活动代码或当前文档命中；历史设计文档不在此扫描范围。

- [ ] **Step 3: 检查 main.c 和中断约束**

Run: `python -m unittest tests.test_600ksps_fft_contract.SourceContractTest.test_adc_callbacks_only_set_their_flag -v`

Expected: `OK`；人工确认 `main.c` 用户初始化区仅调用 `system_init()`，主循环仅调用 `system_process()`。

- [ ] **Step 4: 执行 STM32CubeIDE Debug 构建**

优先使用工程现有 headless build 命令或 STM32CubeIDE 的 `Project > Clean...`、`Project > Build Project`。

Expected: `0 errors`；记录 warnings 数量和任何因本机 IDE 环境无法验证的项目。

- [ ] **Step 5: 检查最终提交范围**

Run: `git status --short` 和 `git log -5 --oneline`

Expected: 只剩用户原有或明确排除的 `.settings`/IDE 刷新文件；不 push 远端。
