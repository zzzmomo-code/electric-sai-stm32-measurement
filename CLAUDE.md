# Claude Code 项目入口

进入本仓库后，先读取：

@AGENTS.md
@README.md
@docs/PROJECT_HANDOFF.md

## 本项目最高优先级约束

- 这是 STM32H743VIT6 + AD9834 工程，不套用全局旧 F103/F407/AD7606 假设。
- 先检查真实 `.ioc`、生成代码、Git状态和硬件连接，再提出修改。
- 用户代码只放 `Core/User`；`main.c` 只通过 `system_init()` 和 `system_process()` 进入用户逻辑。
- 不直接改 CubeMX 的 `MX_*_Init()`；需要改 `.ioc` 时先逐项指导用户在 GUI 完成并生成代码。
- 保持代码简单直观。除非能明显减少重复或错误，不增加新抽象层、通用框架或复杂状态机。
- ISR只设置一个对应标志；计算、SPI、打印和协议处理放在主循环。
- 硬件结论必须区分实测、编译验证和待验证。
- 开始前运行 `git status --short --branch`，不得回滚或提交用户、队友和CubeIDE生成的无关改动。
- 默认不 push。用户明确要求后，只推指定分支；不得覆盖 `h743_pre1` 队友分支。

## 当前事实

- 已验证：PA0方波输入经TIM5测频，AD9834输出 `fin-100kHz` 正弦波。
- AD9834：75 MHz MCLK，SPI2 8 MHz，16 bit，Mode 2，手动FSYNC。
- 正式模式：`DDS_CONTROL_FIXED_TEST_ENABLE=0u`。
- 当前远程默认分支：`main`；稳定基线提交：`8c0c43b`。

## 最小验证

```powershell
python -m unittest discover -s tests -p 'test_*.py' -v
```

工程编译后还必须检查实板。诊断优先观察 `frequency_measure_hz`、`dds_control_diagnostics` 和 `ad9834_diagnostics`。
