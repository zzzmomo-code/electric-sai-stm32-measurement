# 2026 G题 FPGA-SPI-串口屏实施计划

## 文档定位

本文档是 STM32H743 侧的当前实施方案和验收依据。项目只负责连接 FPGA
与淘晶驰串口屏，不再使用旧工程中的片内 ADC、DAC、OPAMP、TIM5 测频、
AD9834、SPI2 扫频控制或 USART2 FPGA 链路。

FPGA 与 STM32 的线上事务以
`D:\QQ\FPGA_STM32_SPI_PROTOCOL_V1.md` 为最高通信协议参考。协议中的
128 字节测量帧头字段和完整帧 CRC 覆盖范围补齐后，才冻结数据解析接口。

## 总体数据流

```text
FPGA持续采集和计算
  -> DATA_READY置高
  -> STM32通过SPI3读取完整测量帧
  -> 校验状态、长度、序号和CRC16
  -> 生成1周期、3周期、频谱三组700点显示缓存
  -> 提前更新串口屏隐藏曲线和参数文本
  -> 用户按键只切换控件可见性
```

STM32 不重新执行 FPGA 的采集、FFT 或测量算法。按键切换不得触发重新测量，
目标是按键后立即显示；即使隐藏控件实测不能保留数据，也应从 STM32
缓存重画并确保小于 2 秒。

## 硬件接口

### FPGA SPI3

复用底板 ADS8688 外接插座。插座信号名称原本以 ADS8688 为参照，复用后
按下表理解：

| 插座脚 | 底板网络 | STM32H743 | 新用途 | 方向 |
|---:|---|---|---|---|
| 1 | D1 | PD1 | FPGA_DATA_READY | FPGA -> STM32 |
| 2 | C12 | PC12 / SPI3_MOSI | MOSI | STM32 -> FPGA |
| 3 | A15 | PA15 / GPIO_Output | FPGA_CS_N | STM32 -> FPGA |
| 4 | C10 | PC10 / SPI3_SCK | SCK | STM32 -> FPGA |
| 5 | GND | GND | 公共地 | - |
| 6 | C11 | PC11 / SPI3_MISO | MISO | FPGA -> STM32 |
| 7 | GND | GND | 公共地 | - |
| 8 | D0 | PD0 | 不使用 | - |

约束：

- FPGA 与 STM32 均使用 3.3 V 逻辑电平并必须共地。
- 该插座不提供 FPGA 电源，FPGA 由自身电源供电。
- 复用期间不得同时连接 ADS8688。
- PA15 使用软件控制的低有效 CS，不使用 SPI3 硬件 NSS。
- PD1 只作为 DATA_READY 输入，不再承担 ADS8688 复位功能。
- PD0/DAISY 保持未使用。

### 淘晶驰串口屏

| 信号 | STM32H743 | 参数 |
|---|---|---|
| 屏幕RX | PA9 / USART1_TX | 512000 baud、8N1 |
| 屏幕TX | PA10 / USART1_RX | 512000 baud、8N1 |
| GND | GND | 必须共地 |

## IOC最终配置

### 系统时钟

- HSE：25 MHz。
- CPU/SYSCLK：480 MHz。
- PLL2：`M=5`、`N=64`、`P=4`，PLL2P 输出 80 MHz。
- SPI123 Kernel Clock Source：PLL2P。
- SPI3 Baud Rate Prescaler：`4`，得到 20 MHz SCK。
- SYS Debug：Serial Wire，释放 PA15 的 JTAG 功能。

### SPI3参数

- Mode：Full-Duplex Master。
- Hardware NSS Signal：Disable。
- Frame Format：Motorola。
- Data Size：8 bit。
- First Bit：MSB First。
- Clock Polarity：Low。
- Clock Phase：1 Edge。
- NSS Pulse：Disable。
- Master Keep IO State：Enable。
- CRC Calculation：Disable，协议 CRC16 由软件计算。
- GPIO PC10、PC11、PC12：Alternate Function Push-Pull、No Pull、
  Very High Speed。

### 软件CS与DATA_READY

- PA15：GPIO Output Push-Pull、No Pull、Very High Speed，标签
  `FPGA_CS_N`，初始电平 High。
- PD1：GPIO EXTI1、Rising Edge、Pull-down，标签
  `FPGA_DATA_READY`。
- EXTI 回调只置 `fpga_data_ready_flag`；读取、协议处理和标志清除放在
  主循环处理函数中。
- 主循环同时检查 PD1 实际电平，不能只依赖上升沿，因为 ACK 后下一帧
  已就绪时 DATA_READY 可以持续为高。

### SPI3 DMA

SPI3 RX：

- Peripheral to Memory。
- Normal Mode。
- Peripheral Increment Disable。
- Memory Increment Enable。
- Peripheral/Memory Data Alignment：Byte。
- Priority：Very High。

SPI3 TX：

- Memory to Peripheral。
- Normal Mode。
- Peripheral Increment Disable。
- Memory Increment Disable。
- Peripheral/Memory Data Alignment：Byte。
- Priority：High。

TX 关闭内存递增，DMA 读取响应期间反复发送同一个 `0x00` dummy 字节。
RX/TX DMA 中断优先级使用 5，DMA 回调只置一个完成标志。

### USART1

- Asynchronous、512000 baud、8 data bits、No parity、1 stop bit。
- 启用 USART1 RX 和 TX DMA，均使用 Normal Mode。
- 屏幕指令发送必须有超时和错误计数，不能在中断中发送曲线。

### 清理旧外设

新链路完成基础收发后，从 IOC 和初始化顺序中移除不再需要的 SPI1、SPI2、
USART2、ADC1、ADC2、DAC1、OPAMP1、TIM2、TIM3、TIM5。删除前先确认
USART1、SPI3、DMA、GPIO、系统时钟可独立生成并编译。

2026-07-28 已检查当前 IOC 和生成代码：SPI3 已为 8 bit、Mode 0、20 MHz，
PA15 已改为初始高电平的软件 CS，PD1 已改为上升沿 EXTI 输入；SPI3 RX/TX DMA、
USART1 RX/TX DMA 和相应中断均已生成。旧外设目前保留但不进入
`system_init()`/`system_process()` 正式链路，待 FPGA 与串口屏基础联调通过后再按需清理。

## SPI协议实现

### 固定参数

- Mode 0：CPOL=0、CPHA=0。
- SCK：20 MHz。
- 每字节 MSB-first。
- 多字节整数 little-endian。
- 软件 CRC：CRC-16/CCITT-FALSE，`poly=0x1021`、`init=0xFFFF`、
  `refin=false`、`refout=false`、`xorout=0x0000`。
- 状态、ACK 和完整帧中的 CRC 在线上均低字节先传。

### 同一CS立即返回

每条读命令独立占用一次 CS 事务：

```text
CS_N       \_______________________________________________/
MOSI        A5       CMD       00       00       ...
MISO        00       00       RESP0    RESP1    ...
```

发送 `A5 CMD` 时同时收到的两个字节丢弃。CS 保持为低，从紧接着的
第一个 dummy 字节读取 `RESP0`，不插入 turnaround 字节。响应读完后
再拉高 CS。

### 状态机

```text
IDLE
  -> DATA_READY为高
  -> GET_STATUS
  -> 校验magic、版本、状态CRC、FRAME_READY和frame_length
  -> READ_FRAME
  -> 校验G26F、字段范围、序号、总长度和完整帧CRC
     -> 成功：解析快照并ACK_FRAME
     -> 失败：不ACK，最多重读3次
  -> ACK后重新读取DATA_READY电平
     -> 高：继续GET_STATUS
     -> 低：返回IDLE
```

命令：

| CMD | 名称 | 行为 |
|---:|---|---|
| `0x01` | GET_STATUS | 同一CS读取16字节状态 |
| `0x02` | READ_FRAME | 同一CS读取 `frame_length` 字节 |
| `0x03` | ACK_FRAME | 发送帧序号及ACK CRC |

完整帧最大 10254 字节。`frame_length` 必须满足：

```text
frame_length == total_bytes
             == header_bytes
              + time_count * 2
              + spectrum_count * 2
              + 2
```

V1 中 `header_bytes=128`、`time_count<=3750`、
`spectrum_count=1312`。

帧字段采用显式小端读取函数解析，不直接把DMA缓冲区强制转换成结构体。
这样避免未对齐访问、编译器填充和协议版本变化造成错误。

CRC正确但 `RESULT_INVALID=1` 的帧仍然 ACK 释放，但不覆盖上一份有效显示
快照。ADC超量程、FPGA丢帧、CRC错误、长度错误、DMA错误和重试次数均保留
32位诊断计数。

## H7 DMA与缓存

- SPI DMA 缓冲区不得放入 DTCM。
- DMA缓冲区使用 32 字节对齐，并按 STM32H743 D-Cache 规则执行
  Clean/Invalidate。
- 使用一个最大帧接收缓冲区和两份测量快照：后台写入非活动快照，
  完整校验并转换后再原子切换活动索引。
- 不使用动态内存分配。
- 中断与主循环共享的完成标志使用 `volatile`。

## 数据转换与显示缓存

STM32长期保存：

```c
uint8_t waveform_1cycle[350];
uint8_t waveform_3cycle[350];
uint8_t spectrum_display[350];
```

以及：

- Vpp、Vrms、基频、直流偏置。
- 最多三个分量的频率、峰值幅度、FFT bin、谐波次数和有效标志。
- 当前帧序号、更新时间和通信诊断。

时域处理：

- 输入为 FPGA 低通后的 `int16_t` 三周期数据，单位 10 µV/LSB。
- 三周期显示对完整数据做分段重采样，输出 350 点。
- 一周期显示从完整三周期中选择一个连续完整周期，再输出 350 点。
- 纵轴按当前有效快照的最小值和最大值映射，并留出上下边距。

频谱处理：

- 输入为 1312 个 `uint16_t` 幅值，单位 10 µV_peak/LSB。
- 频率为 `k * 381.4697265625 Hz`。
- 1312 点压缩至 350 点时，每个区间取最大值，避免窄谱线被平均掉。
- 显示参数使用 FPGA 帧头中的精测结果，不从 350 点显示数组反推。

## 串口屏页面

串口屏使用一个页面。左侧放置两个位置和尺寸完全重叠的时域 Waveform 控件，
右侧独立放置频谱 Waveform 控件：

- `s_t1`：一周期时域。
- `s_t3`：三周期时域。
- `s_spec`：频谱。

三个控件宽度统一按 350 个显示点设计。`s_t1` 与 `s_t3` 只在左侧二选一显示，
`s_spec` 在右侧持续显示，不与时域波形重叠。页面同时放置 Vpp、Vrms、基频、
分量频率/幅值和通信状态文本。

两个按键向 STM32 返回：

```text
A5 01 5A  // 显示一周期
A5 02 5A  // 显示三周期
```

固件保留历史命令 `A5 03 5A` 的兼容解析，但当前页面不需要单独的频谱按钮。

后台每收到一份有效快照：

1. 更新三组 STM32 显示缓存。
2. 尝试把三组曲线写入对应的隐藏 Waveform 控件。
3. 更新参数文本。
4. 记录每个控件已装载的快照序号。

按键时只在 `s_t1` 与 `s_t3` 之间切换，`s_spec` 保持可见。如果实测发现淘晶驰隐藏控件
不能可靠接收或保留 Waveform 数据，则使用备用策略：

```text
按键 -> 选择STM32已有缓存 -> 清空对应时域曲线 -> 发送 350 点 -> 显示
```

当前第一版使用可读性和兼容性更高的 350 条 ASCII `add` 指令，而不是 `addt`
透传。单条曲线最坏不超过 7.8 KB，在 512000 baud、8N1 下纯线缆传输约 151 ms；
三条曲线在后台依次预装。按键只发送可见性命令，因此预装成功后的切换时间不受
整条曲线传输时间影响，仍远小于 2 秒。

## 当前实现状态

2026-07-28 已完成：

- `fpga_protocol`：V1 状态帧、测量帧、小端字段和 CRC-16/CCITT-FALSE 校验。
- `fpga_link`：同一 CS 立即返回、SPI3 DMA、超时、三次重试、ACK 和双快照。
- `measurement_conversion`：一周期、三周期和频谱三组 350 点显示缓存。
- `hmi_task2`：USART1 DMA、三图后台预装、参数文本和按键可见性切换。
- `system`：正式主循环只执行 FPGA 接收、转换和串口屏刷新。

当前串口屏工程仍需按本文档创建 `s_t1`、`s_t3`、`s_spec` 和参数文本控件；
在控件名称对齐前，MCU 会发送数据但页面无法正确显示。隐藏 Waveform 控件能否在
当前屏幕固件中持续接收并保留数据仍属于待实板验证项。

## 软件模块

所有用户模块放在 `Core\User`，由 `system.h` 统一包含：

- `fpga_link.c/.h`：SPI事务、DATA_READY状态机、DMA调度、超时和诊断。
- `fpga_protocol.c/.h`：小端字段读取、CRC16、状态和完整帧校验。
- `measurement_conversion.c/.h`：快照解析、时域重采样和频谱压缩。
- `hmi_chart.c/.h`：Waveform清空、批量写点和可见性切换。
- `hmi_task2.c/.h`：屏幕按键解析、文本更新、显示模式和刷新调度。
- `system.c/.h`：统一初始化和主循环入口。

`main.c` 用户初始化区只调用 `system_init()`；主循环用户区只调用
`system_process()`。HAL回调只设置与中断目的对应的单个标志变量。

## 验证与验收

### 软件检查

- 使用固定字节数组验证小端字段和CRC16。
- 使用正确帧、错误magic、错误长度、错误CRC、重复序号、
  `RESULT_INVALID` 和最大10254字节帧测试状态机。
- 确认协议解析不会访问接收缓冲区之外的地址。
- CubeIDE Debug和Release均完成实际编译并生成ELF。

### 逻辑分析仪

- SCK空闲低，20 MHz。
- 上升沿采样、下降沿更新。
- `A5 CMD` 与响应之间CS不抬高。
- 第一个dummy时钟返回 `RESP0`。
- READ_FRAME从 `47 32 36 46` 开始。
- ACK正确后DATA_READY按协议更新。

### 串口屏

- USART1实测为512000、8N1。
- 一周期/三周期两个按键均能在2秒内切换，目标响应小于100 ms。
- 改变FPGA输入后，三种缓存和参数均更新到同一帧序号。
- 隐藏控件保留测试通过后才启用“预写隐藏控件”作为正式路径。
- 备用重画路径必须独立验证。

### 交付边界

- 主机编译和协议单元测试通过不代表实板SPI、DMA Cache或串口屏已验证。
- FPGA、底板插座和串口屏的接线、时序、显示内容均需实板确认。
- 未经用户明确许可，不推送远程分支。
