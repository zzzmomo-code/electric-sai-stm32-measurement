# H743 外差测量项目协作与交接指南

## 1. 当前可用基线

- 工程：`h743_pre1`
- MCU：STM32H743VIT6
- IDE：STM32CubeIDE 1.19.0
- DDS：AD9834，MCLK 75 MHz
- GitHub：`zzzmomo-code/electric-sai-stm32-measurement`
- 正式分支：`main`
- 当前稳定提交：`8c0c43b`
- 实板验证日期：2026-07-18

已验证链路：

```text
0~3.3 V同频方波
  -> PA0 / TIM5_CH1外部计数
  -> DWT测量实际门时间
  -> 得到输入频率fin
  -> 计算fLO = fin - 100 kHz
  -> SPI2写AD9834
  -> 输出对应频率正弦波
```

队友分支 `h743_pre1` 保留，不直接覆盖。上一道 H750 双 ADC 工程已备份到 `legacy/h750-onchip-adc-20260718`，两套工程没有共同 Git 历史，不做普通合并。

## 2. 已确认硬件与配置

| 功能 | 引脚/参数 |
|---|---|
| 方波输入 | PA0 / TIM5_CH1 |
| AD9834 FSYNC | PB12，空闲高 |
| AD9834 SCLK | PB13 / SPI2_SCK |
| AD9834 SDATA | PB15 / SPI2_MOSI |
| AD9834 FSELECT | PB14，默认低 |
| AD9834 PSELECT | PD8，默认低 |
| SPI2 | 16 bit、MSB、CPOL High、CPHA 1 Edge |
| SPI123内核时钟 | PER_CK 64 MHz |
| SPI2分频 | /8，SCLK 8 MHz |
| 中频目标 | 100 kHz |

AD9834 使用控制寄存器 RESET 位做软件复位。没有由 MCU 控制的 RESET、SLEEP 等硬件脚必须按模块原理图固定到有效电平，不能悬空。

## 3. 软件边界

- `Core/User/frequency_measure.*`：TIM5计数与DWT时间测量。
- `Core/User/dds_control.*`：输入检查、`fin-100kHz` 频率规划和更新阈值。
- `Core/User/ad9834.*`：SPI字发送、FTW计算、频率/相位寄存器选择。
- `Core/User/system.*`：统一初始化与主循环调度。
- `tests/test_dds_contract.py`：DDS配置与调用契约。

正式模式在 `Core/User/dds_control.h`：

```c
#define DDS_CONTROL_FIXED_TEST_ENABLE 0u
```

改为 `1u` 后进入固定通信测试模式，默认输出100 kHz。中断只置标志，SPI和计算都在主循环执行。

## 4. 推荐协作流程

1. 先读 `AGENTS.md`、`README.md`、本文件和根目录 `.ioc`。
2. 执行 `git status --short --branch`，区分用户源码改动、CubeIDE索引文件和 `Debug` 构建产物。
3. 先确认硬件事实；引脚、时钟、外设实例不清楚时查 `.ioc`、原理图和已生成代码。
4. 涉及 CubeMX 时，先逐项指导用户在 GUI 配置并检查截图，用户生成代码后再编写依赖代码。
5. 新功能先做最小固定测试，使通信和引脚状态可观察；通过后接入真实数据流。
6. 代码保持简单：一层驱动、一层控制、一层统一调度已经足够，不为“架构感”增加无实际收益的抽象。
7. 暴露少量稳定诊断结构，在 Expressions 中观察状态、频率、FTW、写次数和 HAL 错误。
8. 完成后依次执行契约测试、工程编译、实板验证和 README 更新。
9. Git 只暂存本次相关文件；不提交 `Debug`、索引缓存或不明来源的脏文件。
10. 未经用户明确许可不 push；替换无共同历史的主线前必须先保存旧历史。

## 5. 用户偏好

- 中文沟通，说明要具体，但代码和最终结论保持精简。
- 希望协作者主动执行、持续追到根因，不停在“可能是”。
- 不喜欢复杂、冗余、难看懂的代码；优先直接函数、清楚状态和短调用链。
- 硬件步骤一次只推进一个可验证节点，明确断点、Expressions变量和预期值。
- 对“已验证”“待验证”“推测”严格区分，不用软件测试代替实板结论。
- 保留队友改动和团队分支；执行危险 Git 操作前先留可恢复备份。

## 6. 已踩过的坑

- CubeIDE 打开的 `.ioc` 必须是项目 active `.ioc`；项目名与 `.ioc` 名不一致会报 `Must be project's active .ioc file`。
- SPI123 时钟曾错误选到 480 MHz 并标红；当前正确设置为 PER_CK 64 MHz，SPI2 /8 得8 MHz。
- `undefined reference to HAL_SPI_Init/HAL_SPI_Transmit` 是 HAL SPI 源文件未参与链接，不是 AD9834 驱动算法错误。
- `Setup exceptions` 属于 Debug launch 配置问题，应检查实际 ELF、芯片型号、SWD和启动脚本。
- CubeIDE生成的 `Debug/subdir.mk` 可能加入 GCC 13 不支持的 `-fcyclomatic-complexity`；这是本机生成文件问题，不能为绕开它改业务源码。
- CubeIDE可能生成大量 `Debug`、`.cproject` 和语言索引变化。提交前必须精确暂存，不能 `git add -A`。

## 7. 后续实施顺序

1. 完成过零比较器和输入保护，验证1~30 MHz方波质量。
2. 联调 AD835，将差频稳定到100 kHz。
3. 完成中频隔直/滤波和VGA。
4. 复用片上ADC测量能力，优先实现100 kHz中频RMS或I/Q测幅。
5. 建立频率、增益档位和输入幅度校准表。
6. 最后实现自动增益与1~5次谐波顺序外差测量。

每一步都应保留示波器截图、输入条件、代码提交和误差记录。
