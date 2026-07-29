# G题周期信号测量分析装置 STM32 工程

## 项目用途

本工程是 2026 电赛 G 题装置的 STM32H743 显示与通信端。STM32 只负责：

1. 通过 SPI3 从 FPGA 读取完整测量帧；
2. 校验状态、长度、序号和 CRC16；
3. 将时域和频谱数据转换为三组 700 点显示缓存；
4. 通过 USART1 驱动淘晶驰串口屏；
5. 提前把三组曲线写入重叠的隐藏控件，按键时只切换可见性。

FPGA 负责 ADC 采集、滤波、FFT 和参数测量。旧工程中的片内 ADC、DAC、DDS、
TIM 测频和 USART2 FPGA 链路不参与当前正式运行。

开发环境：

- MCU：STM32H743VIT6；
- IDE：STM32CubeIDE 1.19.0；
- HAL：STM32Cube FW_H7 V1.12.1；
- 系统时钟：480 MHz；
- FPGA SPI：20 MHz、Mode 0、8 bit、MSB first；
- 串口屏：512000 baud、8N1。

## 硬件连接

FPGA 复用底板 ADS8688 外接插座：

| 插座脚 | STM32 | 信号 | 方向 |
|---:|---|---|---|
| 1 | PD1 | FPGA_DATA_READY | FPGA → STM32 |
| 2 | PC12 / SPI3_MOSI | MOSI | STM32 → FPGA |
| 3 | PA15 | FPGA_CS_N，低有效 | STM32 → FPGA |
| 4 | PC10 / SPI3_SCK | SCK | STM32 → FPGA |
| 5、7 | GND | 公共地 | - |
| 6 | PC11 / SPI3_MISO | MISO | FPGA → STM32 |
| 8 | PD0 | 未使用 | - |

淘晶驰串口屏：

| STM32 | 屏幕 |
|---|---|
| PA9 / USART1_TX | RX |
| PA10 / USART1_RX | TX |
| GND | GND |

FPGA、STM32 和串口屏必须共地，FPGA 接口必须是 3.3 V 逻辑。复用期间不得同时
连接 ADS8688。

## IOC 配置

当前 `h743_task2_20260727.ioc` 已生成下列配置：

### SPI3

- Full-Duplex Master；
- Hardware NSS Disable，PA15 软件控制 CS；
- Motorola、8 bit、MSB first；
- CPOL Low、CPHA 1 Edge，即 SPI Mode 0；
- PLL2P 80 MHz，Prescaler 4，SCK 20 MHz；
- PC10/PC11/PC12：AF Push-Pull、No Pull、Very High Speed；
- SPI3 RX DMA：Normal、Byte、Memory Increment Enable、Very High；
- SPI3 TX DMA：Normal、Byte、Memory Increment Disable、High；
- DMA 中断优先级 5。

TX DMA 关闭内存递增是有意设计：读取响应时重复发送同一个 `0x00` dummy 字节。

### 软件 CS 与 DATA_READY

- PA15：GPIO Output，初始 High，标签 `FPGA_CS_N`；
- PD1：GPIO EXTI1，Rising Edge、Pull-down，标签 `FPGA_DATA_READY`；
- EXTI1 中断优先级 6。

主循环同时检查 PD1 实际电平，避免 DATA_READY 持续为高时只依赖上升沿而漏帧。

### USART1

- Asynchronous、512000 baud、8N1；
- RX/TX DMA 均为 Normal Mode；
- USART1 中断优先级 7；
- 使用 `HAL_UARTEx_ReceiveToIdle_DMA()` 接收按键命令。

CubeMX 重新生成前必须勾选
`Project Manager > Code Generator > Keep User Code when re-generating`。
不要手改 `MX_*_Init()`；用户模块全部位于 `Core/User`。

## FPGA SPI V1 协议

最高参考是 `D:\QQ\FPGA_STM32_SPI_PROTOCOL_V1.md`。

每条读命令在同一次 CS 事务中完成：

```text
CS_N       \_________________________________________/
MOSI        A5       CMD       00       00       ...
MISO        --       --       RESP0    RESP1    ...
```

`A5 CMD` 阶段收到的 MISO 字节丢弃；CS 保持低电平，第一个 dummy 字节对应
`RESP0`，不存在额外 turnaround 字节。

| CMD | 功能 |
|---:|---|
| `0x01` | GET_STATUS，读取 16 字节状态 |
| `0x02` | READ_FRAME，读取 `frame_length` 字节完整帧 |
| `0x03` | ACK_FRAME，确认正确帧序号 |

多字节整数均为小端。CRC 使用 CRC-16/CCITT-FALSE：

```text
poly=0x1021
init=0xFFFF
refin=false
refout=false
xorout=0x0000
```

完整帧内存布局：

```text
128字节帧头
+ time_count × int16_t 时域数据
+ spectrum_count × uint16_t 频谱数据
+ 2字节CRC16
```

V1 固定约束：

- 帧头 magic：ASCII `G26F`；
- `header_bytes=128`；
- `time_count<=3750`，时域数据为 10 µV/LSB；
- `spectrum_count=1312`，频谱数据为 10 µV_peak/LSB；
- `fft_length=4096`，`bin_spacing_mHz=381470`；
- 最大帧长 10254 字节。

CRC 或长度错误时不发送 ACK，最多重新读取三次。`RESULT_INVALID` 帧会 ACK 释放
FPGA 缓冲区，但不会覆盖上一份有效显示快照。

## 串口屏页面合同

当前 C 代码要求同一页面包含以下对象，名称必须完全一致：

| 名称 | 类型 | 用途 |
|---|---|---|
| `s_t1` | Waveform | 一周期时域 |
| `s_t3` | Waveform | 三周期时域 |
| `s_spec` | Waveform | 频谱 |
| `t_vpp` | Text | 峰峰值 |
| `t_vrms` | Text | 真有效值 |
| `t_freq` | Text | 基频 |
| `t_comp1`～`t_comp3` | Text | 三个分量 |
| `t_status` | Text | 帧序号和丢帧状态 |

三个 Waveform 控件放在相同位置，宽度按 700 点设计。横纵轴、单位和刻度使用页面
静态控件绘制。

三个按钮的按下事件分别发送：

```text
printh A5 01 5A  // 一周期
printh A5 02 5A  // 三周期
printh A5 03 5A  // 频谱
```

上电后 STM32 先隐藏三图。收到有效 FPGA 帧后，先生成三组缓存，再依次把默认优先
模式、其他两图和参数文本预装到屏幕；在收到第一次有效按键命令前始终不显示曲线。
新 FPGA 帧到来时保留一份稳定工作快照，完成当前三图预装后才切换到更新快照，
避免连续新帧造成预装饥饿。

当前使用普通 `cle` + 700 条 `add` 指令，一条曲线最坏约 18 KB；512000 baud 下
线缆传输约 352 ms。按键后的正常路径只发送可见性命令，因此响应远小于 2 秒。

注意：仓库中的旧 `fpga1.HMI` 仍可能只有旧对象，必须先在 USART HMI 工具中按上表
创建控件并重新下载页面。隐藏 Waveform 控件是否能可靠接收并保留数据仍需实板验证。

## 软件结构

- `Core/User/fpga_protocol.c/.h`：小端字段读取、CRC、状态帧和测量帧校验；
- `Core/User/fpga_link.c/.h`：SPI3 事务、DMA、重试、ACK 和双测量快照；
- `Core/User/measurement_conversion.c/.h`：一周期、三周期和频谱三组 700 点缓存；
- `Core/User/hmi_chart.c/.h`：Waveform 清空、写点和可见性命令构建；
- `Core/User/hmi_task2.c/.h`：USART1 DMA、按键解析、预装调度和参数文本；
- `Core/User/system.c/.h`：唯一用户初始化和主循环入口。

`main.c` 用户初始化区只调用：

```c
system_init();
```

`while (1)` 用户区只调用：

```c
system_process();
```

HAL 回调只设置一个对应的共享标志，协议解析、重试、转换和发送均在主循环执行。
所有大缓冲区静态分配并按 32 字节对齐；启用 D-Cache 时会执行 Clean/Invalidate。

## 编译、烧录和联调

1. 在 CubeIDE 中右键工程选择 `Refresh`，让新文件 `fpga_protocol.c` 加入构建；
2. 执行 `Project > Clean...`；
3. 执行 `Project > Build Project`，确认生成 `.elf`；
4. 使用 ST-LINK 烧录；
5. 先只连接串口屏，确认页面对象名称和 512000 baud；
6. 再连接 FPGA，逻辑分析仪检查 Mode 0、20 MHz 和同一 CS 立即响应；
7. 在 Expressions 观察 `fpga_link_diagnostics` 和 `hmi_task2_diagnostics`。

关键诊断量：

- `status_valid_count`、`frame_valid_count`、`ack_count` 应持续增加；
- `status_crc_error_count`、`frame_crc_error_count`、`timeout_count` 应保持 0；
- `retry_count` 只应在坏帧/超时后增加；
- `preload_complete_count` 应随完整屏幕快照增加；
- `command_count` 应随三个按钮操作增加；
- `tx_error_count` 应保持 0。

## 当前验证边界

- 协议和 HMI 当前合同测试已通过；
- 新增模块已使用 CubeIDE 随附 GCC 完成编译和 ELF 链接；
- 旧 ADC/DAC/DDS/TIM 合同测试仍对应历史运行链路，不代表本 G 题链路失败；
- SPI 电气时序、DMA/D-Cache、隐藏控件保留行为和整机 2 秒指标均待实板验证；
- 当前正常模式 `HMI_CHART_SELF_TEST_ENABLE=0`，不会发送自检图；
- 用户本次 CubeMX 生成的 IOC/生成文件保留在工作区，不随本功能提交覆盖；
- 未经明确许可不推送远程分支。

更完整的设计和验收步骤见
`docs/G26_FPGA_SPI_HMI_IMPLEMENTATION_PLAN.md`。
