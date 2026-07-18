# STM32H743 + AD9834 外差本振控制

## 当前状态

本工程用于大学生电子设计竞赛测量组的简易外差式幅频测量装置。当前已完成并通过实板全流程测试：

```text
输入同频方波
    -> PA0 / TIM5 外部脉冲计数
    -> 计算输入频率 fin
    -> 计算低侧本振 fLO = fin - 100 kHz
    -> SPI2 配置 AD9834
    -> AD9834 输出对应频率的正弦波
```

2026-07-18 实测确认：方波输入、TIM5 粗测频、频率规划、SPI 通信和 AD9834 正弦输出均正常。当前代码默认运行正式跟随模式，是项目现阶段的稳定基线。

开发环境：STM32CubeIDE 1.19.0、STM32Cube FW_H7 V1.12.1。

## 硬件

- MCU：STM32H743VIT6。
- DDS：AD9834，外部 MCLK 为 75 MHz。
- PA0：TIM5_CH1，接过零比较器输出的 0～3.3 V 方波。
- PB12：AD9834 FSYNC，空闲为高电平。
- PB13：SPI2_SCK，连接 AD9834 SCLK。
- PB15：SPI2_MOSI，连接 AD9834 SDATA。
- PB14：AD9834 FSELECT，低电平选择 FREQ0，高电平选择 FREQ1。
- PD8：AD9834 PSELECT，低电平选择 PHASE0，高电平选择 PHASE1。
- MCU、比较器和 AD9834 必须共地。

AD9834 初始化使用控制寄存器的 RESET 位完成软件复位，不需要 MCU 单独控制硬件 RESET 引脚。硬件 RESET、SLEEP 等未由本工程控制的引脚应按实际模块原理图固定到有效电平，不得悬空。

## CubeMX 配置

### TIM5 输入计数

- `PA0 -> TIM5_CH1`。
- External Clock Mode 1，Trigger Source 为 TI1FP1。
- 上升沿计数，Prescaler `0`，Period `0xFFFFFFFF`，Input Filter `0`。

### TIM3 测量调度

- Prescaler `23999`，Period `9999`。
- 约每秒产生一次更新中断，NVIC 优先级为 6。
- 中断回调只设置 `frequency_measure_flag`，频率计算在主循环执行。

### SPI2 / AD9834

- Transmit Only Master，Motorola，16 bit，MSB First。
- CPOL High，CPHA 1 Edge，软件 NSS，NSS Pulse Disabled。
- SPI123 内核时钟选择 PER_CK，频率 64 MHz。
- SPI2 Prescaler `8`，SCLK 为 8 MHz。
- PB12/FSYNC 初始为高；PB14/FSELECT、PD8/PSELECT 初始为低。
- 不使用 SPI DMA 和 SPI 中断。

CubeMX 重新生成代码前启用 `Project Manager > Code Generator > Keep User Code when re-generating`。不要手工修改自动生成的 `MX_*_Init()`，用户模块统一放在 `Core/User`。

## 软件流程

`frequency_measure.c` 连续读取 TIM5 的 32 位累计计数，同时使用 Cortex-M7 DWT 周期计数器获得真实测量时间：

```text
fin = TIM5计数差 * SystemCoreClock / DWT周期差
```

`dds_control.c` 接收粗测频结果，并在 1～30 MHz 有效输入范围内计算：

```text
fLO = fin - 100000 Hz
```

只有目标本振相对上次成功输出变化至少 500 Hz 时才重新写 AD9834，避免输入测量的微小波动导致 DDS 频繁更新。SPI 写操作始终在主循环中执行。

`ad9834.c` 使用 75 MHz MCLK 计算 28 位频率控制字：

```text
FTW = round(fLO * 2^28 / 75 MHz)
```

初始化顺序为软件复位、写 FREQ0、写 PHASE0、退出复位。FSYNC 在每个 16 位 SPI 字发送前拉低，发送结束后恢复高电平。

## 模式切换

模式开关位于 `Core/User/dds_control.h`：

```c
#define DDS_CONTROL_FIXED_TEST_ENABLE 0u
```

- `0`：正式模式，读取 TIM5 粗测频结果并输出 `fin - 100 kHz`，当前默认值。
- `1`：固定通信测试模式，AD9834 输出 `DDS_CONTROL_TEST_OUTPUT_HZ`，当前为 100 kHz，并每秒重写一次。

修改模式后重新编译和烧录。正式运行时 FSELECT 和 PSELECT 都保持低电平，使用 FREQ0 和 PHASE0。

## 代码结构

- `Core/User/system.c`：用户模块统一初始化和主循环处理。
- `Core/User/frequency_measure.c`：TIM5 外部计数及 DWT 时间测量。
- `Core/User/dds_control.c`：输入频率检查、低侧本振计算和更新阈值控制。
- `Core/User/ad9834.c`：AD9834 SPI 驱动、FTW 计算及 FSELECT/PSELECT 控制。
- `tests/test_dds_contract.py`：检查 IOC、SPI 时序约定、频率公式和模块调用关系。

`main.c` 用户初始化区只调用 `system_init()`，主循环只调用 `system_process()`。

## 编译与烧录

1. 在 STM32CubeIDE 1.19.0 中打开 `h743_pre1`。
2. 选择 Debug 配置，执行 `Project > Clean...`。
3. 执行 `Project > Build Project`，确认 0 errors、0 warnings。
4. 使用 ST-LINK 下载并运行。
5. 将过零比较器输出接到 PA0，示波器连接 AD9834 输出端并共地。
6. 改变输入方波频率，检查 DDS 输出是否满足 `fDDS = fin - 100 kHz`。

离线契约测试：

```powershell
python -m unittest discover -s tests -p 'test_*.py' -v
```

## 调试变量

可在 STM32CubeIDE Expressions 中观察：

- `frequency_measure_hz`：TIM5 粗测得到的输入频率。
- `dds_control_diagnostics.state`：`waiting`、`test`、`tracking` 或 `error`。
- `dds_control_diagnostics.input_frequency_hz`：当前采用的输入频率。
- `dds_control_diagnostics.output_frequency_hz`：目标 DDS 输出频率。
- `dds_control_diagnostics.update_count`：成功更新次数。
- `ad9834_diagnostics.output_frequency_hz`：底层驱动最近一次成功设置的频率。
- `ad9834_diagnostics.write_count`：成功写入的 16 位字数。
- `ad9834_diagnostics.error_count`：SPI 写入错误次数，正常应保持 0。
- `ad9834_diagnostics.last_hal_status`：最近一次 HAL SPI 状态，正常为 `HAL_OK`。

## 已知限制与后续工作

- 输入方波必须满足 STM32H743 GPIO 电平范围；模拟信号应先经过可靠的过零比较器整形。
- 当前频率规划采用低侧本振，输入有效范围为 1～30 MHz，目标中频固定为 100 kHz。
- 频率绝对精度受输入边沿质量、H743 系统时钟和 AD9834 75 MHz 参考时钟误差影响。
- 当前只完成 DDS 控制全流程；AD835 混频、中频滤波、VGA 自动增益、片上 ADC 幅度测量和整机校准仍需后续联调。
- 仓库中保留了前一训练题的双 ADC、FFT 和串口屏模块，当前 DDS 链路不以这些模块的测量结果作为验收依据。
