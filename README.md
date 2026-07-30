# G题周期信号测量分析装置 STM32 工程

## 项目用途

本工程是 2026 电赛 G 题装置的 STM32H743 显示与通信端。STM32 只负责：

1. 通过 SPI3 从 FPGA 读取完整测量帧；
2. 校验状态、长度、序号和 CRC16；
3. 将时域和频谱数据转换为三组 350 点显示缓存；
4. 通过 USART1 驱动淘晶驰串口屏；
5. 首帧立即显示并持续刷新频谱；波形首次按键后显示，随后在一周期/三周期之间实时切换。

FPGA 负责 ADC 采集、滤波、FFT 和参数测量。旧工程中的片内 ADC、DAC、DDS、
TIM 测频和 USART2 FPGA 链路不参与当前正式运行。

开发环境：

- MCU：STM32H743VIT6；
- IDE：STM32CubeIDE 1.19.0；
- HAL：STM32Cube FW_H7 V1.12.1；
- 系统时钟：480 MHz；
- FPGA SPI：625 kHz、Mode 0、8 bit、MSB first；
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
- PLL2P 80 MHz，Prescaler 128，SCK 625 kHz；
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

最高参考是
`docs/FPGA_STM32_SPI_PROTOCOL_V1_0_FROZEN_20260730.md`。该文档已经按 FPGA
队友最新的 `D:\QQ\FPGA_STM32_SPI_PROTOCOL_V1_0_STM32.md`（STM32-R2）
再次对齐，并保留双方联调 CRC 测试向量。

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

三个 `component` 是 FPGA 独立候选槽位，有效槽位不保证连续。STM32 遍历
`component[0..2]`，要求 `component_count` 等于三个 `VALID` 位的置位数，
只把 `VALID=1` 的槽位压紧后交给 `t_comp1`～`t_comp3` 显示。例如
`VALID=101、component_count=2` 是合法帧，不会被拒收。
槽位也不保证按频率、幅度或谐波次数排序，代码不会把 `component[0]` 当作
基波；基频文本直接采用帧头 `fundamental_mHz`。

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

`s_t1` 与 `s_t3` 放在左侧相同位置并相互重叠；`s_spec` 独立放在右侧，不与时域
波形重叠。三个 Waveform 控件宽度统一为 350，横纵轴、单位和刻度使用页面静态控件绘制。

两个按钮的按下事件分别发送：

```text
printh A5 01 5A  // 一周期
printh A5 02 5A  // 三周期
```

固件仍兼容历史命令 `A5 03 5A`，但当前页面不需要频谱按钮，因为右侧 `s_spec`
在显示一周期或三周期时都会保持可见。

上电后 STM32 先隐藏三个曲线控件。收到第一帧有效 FPGA 数据后，立即显示并持续更新
右侧频谱和参数文本；一周期、三周期两个时域控件仍保持隐藏。第一次收到一周期或
三周期按键后，程序先切换可见性，再把 STM32 中相应的最新 350 点缓存重画到可见
控件。此后每个新 FPGA 快照都会继续更新参数、频谱和当前选中的时域波形。

当前使用普通 `cle` + 350 条 `add` 指令，一条曲线最坏不超过 7.8 KB；512000 baud 下
纯线缆传输约 151 ms。第一次或切换波形按键需要发送可见性命令并重画一条 350 点
时域曲线，仍远小于题目要求的 2 秒。

仓库根目录中的 `fpga1.HMI` 是 2026-07-29 保存的当前页面工程：`s_t1` 与 `s_t3`
在左侧同位置重叠，`s_spec` 独立位于右侧，三个控件宽度均为 350。实板自检已确认
STM32 可以接收两个按键命令、切换并重画一周期/三周期波形；频谱在上电取得首帧后
立即显示，并在两种时域模式下保持独立实时更新。

## 软件结构

- `Core/User/fpga_protocol.c/.h`：小端字段读取、CRC、状态帧和测量帧校验；
- `Core/User/fpga_link.c/.h`：SPI3 事务、DMA、重试、ACK 和双测量快照；
- `Core/User/measurement_conversion.c/.h`：一周期、三周期和频谱三组 350 点缓存；
- `Core/User/hmi_chart.c/.h`：Waveform 清空、写点和可见性命令构建；
- `Core/User/hmi_task2.c/.h`：USART1 DMA、按键解析、实时刷新调度和参数文本；
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

## 代码阅读教程

### 先记住一条主数据流

```text
PD1 DATA_READY
    ↓
fpga_link_process()
    ↓  SPI3读取、CRC校验、ACK
fpga_measurement_snapshot_t
    ↓
measurement_conversion_update()
    ↓  截周期、重采样、纵轴映射
measurement_display_snapshot_t
    ↓
hmi_task2_process()
    ↓  cle/add/vis命令 + USART1 DMA
淘晶驰 s_t1 / s_t3 / s_spec
```

工程没有在中断里做大计算。`main.c` 不断调用 `system_process()`，每次只让各状态机
向前走一步，因此 FPGA SPI 和屏幕 UART 可以并行等待 DMA，不会因某一条曲线阻塞
整个 CPU。

### 推荐阅读顺序

第一次看代码时不要从 900 多行的 `hmi_task2.c` 开始，按下面顺序更容易理解：

1. `Core/Src/main.c`
   - 只看 `system_init();` 和 `system_process();` 两个调用；
   - CubeMX 生成的 `MX_*_Init()` 不需要逐行研究，也不要手改。
2. `Core/User/system.c`
   - 这是全部用户业务的目录；
   - 先收 FPGA，再对新帧换算，最后推进屏幕发送。
3. `Core/User/fpga_protocol.h`
   - 看清 FPGA 状态帧、128 字节帧头和最终快照包含哪些字段。
4. `Core/User/fpga_protocol.c`
   - 看小端字段怎样读取、CRC 覆盖到哪里、哪些条件会判定帧无效。
5. `Core/User/fpga_link.c`
   - 看同一 CS 的 GET_STATUS/READ_FRAME/ACK，以及 DMA 重试和双缓冲发布。
6. `Core/User/measurement_conversion.c`
   - 看最多 3750 点时域和 1312 点频谱怎样变成三组 350 点。
7. `Core/User/hmi_chart.c`
   - 看 `cle`、`add`、`vis` 如何变成以三个 `0xFF` 结尾的命令。
8. `Core/User/hmi_task2.c`
   - 最后看实时刷新调度、按键解析和 UART DMA 状态机。

### 三种数据形态

代码中同一份测量结果会经过三种形态。分清这三种结构，就不会把 FPGA 单位、
屏幕坐标和串口字节混在一起。

#### 1. FPGA 原始字节

`fpga_link_frame_storage[]` 保存 READ_FRAME 收到的完整字节流：

```text
128字节头 + int16_t时域数组 + uint16_t频谱数组 + CRC16
```

该缓冲区只是通信原料，DMA 写完并完成 D-Cache 维护后才允许 CPU 解析。

#### 2. 测量快照

`fpga_measurement_snapshot_t` 是已经校验并拆字段后的测量数据：

- 头部参数已经从小端字节转换成 MCU 整数；
- `time_samples[]` 仍是 FPGA 的有符号时域码；
- `spectrum[]` 仍是 FPGA 的无符号幅值；
- Vpp、Vrms、基频和三个分量保留 FPGA 约定的 µV、mHz 单位。

这一层不含淘晶驰控件名，也不关心 350 像素。

#### 3. 显示快照

`measurement_display_snapshot_t` 是屏幕专用缓存：

```c
waveform_1cycle[350]
waveform_3cycle[350]
spectrum_display[350]
```

三个数组都已经映射为 `8~201` 的单字节纵坐标，适配当前高度为 210 像素的曲线
控件，并在上下各保留约 8 个像素，避免曲线被边框裁剪。参数值仍以 µV、mHz
保存，发送前才格式化成 `mV`、`V`、`Hz` 或 `kHz` 文本。

### 一帧数据从 FPGA 到屏幕的完整过程

#### 第一步：发现新帧

FPGA 拉高 PD1。EXTI 回调只设置 `fpga_data_ready_flag`，不读 SPI。主循环进入
`fpga_link_process()` 后，同时检查这个标志和 PD1 当前电平，所以即使上升沿发生在
系统初始化前，只要 DATA_READY 仍保持高电平，也不会漏掉帧。

#### 第二步：读取状态

STM32 拉低 PA15，在同一个 CS 内发送：

```text
A5 01 + 16个dummy 00
```

命令阶段同时收到的 MISO 字节丢弃，从第一个 dummy 字节开始收 16 字节状态。
随后检查：

- magic 是否为 `5A A5`；
- version 是否为 1；
- FRAME_READY 是否置位；
- `frame_length` 是否在合法范围；
- 状态 CRC16 是否正确。

#### 第三步：DMA 读取完整帧

状态正确后，STM32 再拉低 CS，先发送 `A5 02`，保持 CS 为低，启动 SPI3 全双工 DMA。
TX DMA 重复读取一个固定的 `0x00`，RX DMA 将 FPGA 返回的
`frame_length` 字节写入对齐缓冲区。

DMA 完成回调只设置 `fpga_spi_dma_complete_flag`。下一轮主循环才会拉高 CS、维护
D-Cache、解析头部并校验完整帧 CRC。超时、SPI 错误或坏 CRC 都不会 ACK；软件最多
重新 READ_FRAME 三次。

#### 第四步：ACK 和发布快照

有效帧通过后发送：

```text
A5 03 + frame_seq小端4字节 + CRC16小端2字节
```

只有 ACK 成功后 FPGA 才能释放当前缓冲区。STM32 发送 ACK 后会实际读取 PD1，
只有观察到 DATA_READY 拉低才确认本次 ACK 完成；10 ms 内仍为高会记录超时并
重新执行 GET_STATUS，而不会假定固定两个 FPGA 时钟后已经释放。代码把解析结果
先写入“非活动”快照，
执行 `__DMB()` 后再切换活动索引。读者因此只会看到完整旧快照或完整新快照，
不会看到复制到一半的数据。

FPGA 标记 `RESULT_INVALID` 时仍发送 ACK，避免 FPGA 一直卡住，但不会覆盖上一份
有效显示结果。

#### 连续测量约定

STM32 不发送“开始测量”命令。正常运行时双方按下面的闭环持续工作：

```text
FPGA 完成新测量并发布新 frame_seq
→ DATA_READY 拉高
→ STM32 GET_STATUS、READ_FRAME、校验并 ACK
→ FPGA 将 DATA_READY 拉低并释放本帧
→ FPGA 完成下一次测量后，用递增的新 frame_seq 再次拉高 DATA_READY
```

因此固件不是“上电只取一次”。只有 FPGA 持续发布新序号的完整帧，数字、频谱和已显示
波形才会持续变化。若输入改变后画面不更新，先比较
`frame_valid_count`、`ack_count`、`published_snapshot_count` 和
`last_frame_sequence` 是否继续增加；不要用重复旧序号绕过 STM32 的防重发布检查。

#### 第五步：转换为三组 350 点

`measurement_conversion_update()` 对同一个 FPGA 快照生成：

- 一周期：从三周期捕获数据中取中间一个完整周期，再重采样为 350 点；
- 三周期：使用整段有效时域数据，重采样为 350 点；
- 频谱：把 1312 个谱点分成 350 个横向区间，每个区间取最大值。

时域降采样采用分桶平均，防止单点抽取造成波形抖动；原始点少于 350 时采用线性
插值，保证曲线占满横轴。频谱采用区间最大值而不是平均值，避免很窄的谱峰被稀释。

时域纵轴用本帧最小值和最大值线性映射，频谱用本帧最大值线性映射。FPGA 发来的
频谱已经是幅值结果，STM32 不再次开平方。

#### 第六步：持续刷新屏幕

`hmi_task2_refresh_work_snapshot()` 取得一份稳定显示快照。随后调度器每次只启动一个
UART TX DMA 动作：

```text
上电且尚无有效帧：隐藏三图
→ 首帧到达：只显示独立频谱控件，一周期/三周期继续隐藏
→ 先更新参数文本
→ 重画独立频谱
→ 用户首次按键后，显示并重画所选的一周期或三周期波形
→ 标记本快照显示完成
```

程序不向隐藏的一周期/三周期控件后台写 `add`，因为实板验证表明隐藏 Waveform
控件不一定可靠保留曲线数据。三组 350 点数据始终缓存在 STM32；第一次按键前只持续
更新数字和频谱，第一次按键后再持续更新所选时域波形。当前显示完成后直接取得 FPGA
的最新快照，中间过期帧不排队。

#### 第七步：按键切换并重画最新缓存

屏幕按钮返回 `A5 CMD 5A`。解析器可以跨多次 DMA 接收识别这三个字节。收到一周期
或三周期命令后执行：

```text
s_t1/s_t3 中目标控件 vis=1
s_t1/s_t3 中另一个控件 vis=0
s_spec vis=1
→ 清空并重画目标时域控件的 350 个最新缓存点
```

因此按键不触发重新采样、重新 FFT 或重新换算，只重发 STM32 已准备好的 350 个点。
按钮切换后，后续每个 FPGA 新快照都会继续更新当前时域模式和频谱。

### 代码中几个容易看不懂的写法

#### `static`

文件内的状态变量和辅助函数使用 `static`，表示只允许本模块访问。例如
`hmi_task2_work_snapshot` 不会被其他模块直接修改，模块之间通过公开函数交换数据。

#### `volatile`

中断和主循环共同访问的标志使用 `volatile`，告诉编译器每次都必须真的读写内存。
它不等于线程安全，因此主循环取走多个 UART 事件时仍使用一个极短临界区。

#### `const ... *`

`fpga_link_get_snapshot()` 和 `measurement_conversion_get_snapshot()` 返回只读指针。
调用方可以读取已发布快照，但不能越过模块边界修改它。

#### `active_index ^ 1u`

索引只有 0 和 1。异或 1 可以在两个缓冲区之间切换：

```text
0 ^ 1 = 1
1 ^ 1 = 0
```

代码永远向非活动缓冲区写，完成后才发布，减少大数组复制和数据撕裂。

#### `__DMB()`

内存屏障保证“先写完数组，后更新活动索引”的顺序不会被 CPU 或编译器打乱。
它解决的是 CPU 可见顺序；DMA 与 D-Cache 的一致性仍需单独 Clean/Invalidate。

#### 32 字节对齐和 D-Cache

H743 的 D-Cache 行是 32 字节。DMA 接收前清理/失效缓冲区，完成后再次失效，确保
CPU 读到 DMA 写入的新数据；DMA 发送前 Clean，确保 DMA 读到 CPU 刚构建的命令。
因此 DMA 缓冲区都使用 `aligned(32)`，维护范围也向 32 字节边界取整。

#### 无符号超时比较

类似下面的写法允许 `HAL_GetTick()` 在约 49.7 天后回绕：

```c
(uint32_t)(HAL_GetTick() - start_ms) > timeout_ms
```

只要单次超时远小于 2³¹ ms，减法回绕后仍能正确比较。

### 常见修改应该去哪里

| 想修改的内容 | 文件或宏 |
|---|---|
| SPI 命令、帧字段、CRC 规则 | `fpga_protocol.h/.c` |
| SPI 超时、重试次数、DMA事务 | `fpga_link.c` |
| 曲线点数、纵轴上下限 | `measurement_conversion.h` |
| 一周期截取、重采样、频谱压缩 | `measurement_conversion.c` |
| 屏幕对象名称 | `hmi_chart.h` |
| 淘晶驰 `cle/add/vis` 语法 | `hmi_chart.c` |
| 按键 `A5 CMD 5A` 和实时刷新顺序 | `hmi_task2.c` |
| 临时打开三图自检 | `system.c` 中 `HMI_CHART_SELF_TEST_ENABLE` |

不要在 `main.c` 堆业务逻辑，也不要直接修改 `MX_SPI3_Init()`、`MX_USART1_UART_Init()`
等 CubeMX 生成函数。需要改变外设时，应先修改 `.ioc`，重新生成代码，再检查用户区。

## Debug 观察与故障定位

建议在 CubeIDE 的 Expressions 中依次添加：

```text
fpga_link_diagnostics
measurement_conversion_diagnostics
hmi_task2_diagnostics
```

### FPGA SPI 诊断

| 变量 | 正常现象 | 异常说明 |
|---|---|---|
| `data_ready_irq_count` | 每次 PD1 上升增加 | 始终为 0：检查 PD1、EXTI 和共地 |
| `status_read_count` | 有数据时增加 | 不增加：主循环或 DATA_READY 未进入链路 |
| `status_valid_count` | 状态正确时增加 | 不增加：检查同一 CS 返回时序和状态格式 |
| `status_crc_error_count` | 应保持 0 | 增加：字节错位、端序或信号完整性问题 |
| `status_not_ready_count` | DATA_READY/状态竞争时可偶增 | 状态有效但 FRAME_READY=0，不属于 CRC/格式错误 |
| `frame_dma_start_count` | 每个读取尝试增加 | 不增加：状态未通过或 DMA 启动失败 |
| `frame_dma_complete_count` | 应跟随启动数 | 落后：SPI/DMA 中断或时钟存在问题 |
| `frame_dma_timeout_count` | 应保持 0 | 增加：FPGA 未持续输出、DMA 未完成 |
| `frame_format_error_count` | 应保持 0 | 增加：头字段、长度、点数与 V1 不一致 |
| `frame_crc_error_count` | 应保持 0 | 增加：帧损坏、错位或 CRC 实现不一致 |
| `frame_retry_count` | 正常应为 0 | 增加：STM32 正在不 ACK 地重读坏帧 |
| `frame_valid_count` | 每个正确帧增加 | 表示完整格式和 CRC 已通过 |
| `ack_count` | 通常跟随已处理帧 | 不增加：ACK SPI 发送失败 |
| `ack_ready_low_count` | 应跟随 `ack_count` | 表示实际观察到 DATA_READY 拉低 |
| `ack_ready_low_timeout_count` | 应保持 0 | 增加：ACK 未被 FPGA 接受、接线或时序异常 |
| `published_snapshot_count` | 有效结果时增加 | 帧有效但不增加：检查 RESULT_INVALID |
| `last_protocol_result` | 正常为 OK | 可定位最近一次具体协议错误 |

`data_ready_irq_count` 只统计上升沿；如果 PD1 上电时已经为高，软件仍能通过电平轮询
读取状态，因此它为 0 不一定表示完全收不到数据。要结合 `status_read_count` 判断。

### 显示换算诊断

| 变量 | 含义 |
|---|---|
| `conversion_count` | 成功生成 350 点显示快照的次数 |
| `invalid_source_count` | 输入点数或字段不合法次数 |
| `last_frame_sequence` | 最近成功换算的 FPGA 帧序号 |
| `last_time_min` / `last_time_max` | 最近时域原始数据范围 |
| `last_spectrum_max` | 最近频谱最大原始幅值 |
| `last_one_cycle_samples` | 本次估算的一周期原始点数 |

如果 SPI 已收到有效帧但图形像一条直线，先看 `last_time_min` 是否等于
`last_time_max`，或 `last_spectrum_max` 是否为 0，再判断是 FPGA 数据本身恒定还是
屏幕映射问题。

### 串口屏诊断

| 变量 | 正常现象 | 异常说明 |
|---|---|---|
| `tx_start_count` / `tx_complete_count` | 二者长期接近 | 完成数落后：UART DMA/中断有问题 |
| `tx_error_count` | 应保持 0 | 增加：启动失败、超时或 UART 错误 |
| `preload_complete_count` | 每份完整显示快照增加 | 不增加：查看发送是否卡在某一动作 |
| `last_source_sequence` | 跟随换算序号 | 不变：屏幕层没有取得新快照 |
| `command_count` | 每按一次有效按钮增加 | 不增加：检查 PA10、按钮事件和波特率 |
| `invalid_command_count` | 应保持 0 | 增加：按钮返回格式或串口数据不正确 |
| `requested_mode` | 上电为 1，按键后为 1/2 | 当前用户请求的一周期/三周期；3 为历史频谱命令 |
| `visible_mode` | 首帧为 3，按键后为 1/2 | 3 表示仅频谱可见；1/2 表示对应时域与频谱同时可见 |
| `last_visible_sequence` | 当前可见曲线的数据序号 | 可判断显示的是不是最新已装帧 |

## 资源占用与实时性

- 不使用 `malloc()`，所有大数组静态分配，运行时间和内存占用可预测；
- 最大 FPGA 帧约 10.3 KB，SPI 625 kHz 纯线缆时间约 131.3 ms；
- 每条 350 点普通曲线命令最坏不超过 7.8 KB，512000 baud 纯线缆时间约 151 ms；
- 第一次波形按键前每个显示快照发送参数和频谱，理论最高刷新率约 6 Hz；
- 第一次波形按键后每个显示快照发送参数、当前波形和频谱，理论最高刷新率约 3 Hz；
- FPGA 帧快于屏幕刷新时不排队旧显示帧，当前发送结束后直接采用最新 STM32 快照；
- 最近一次完整 ELF 链接结果：`text=64252`、`data=472`、`bss=47616` 字节；
- `.bss` 主要来自双测量快照、原始帧和 HMI TX 缓冲区，H743 RAM 仍有余量。

## 编译、烧录和联调

1. 在 CubeIDE 中右键工程选择 `Refresh`，让新文件 `fpga_protocol.c` 加入构建；
2. 执行 `Project > Clean...`；
3. 执行 `Project > Build Project`，确认生成 `.elf`；
4. 使用 ST-LINK 烧录；
5. 先只连接串口屏，确认页面对象名称和 512000 baud；
6. 再连接 FPGA，逻辑分析仪检查 Mode 0、625 kHz 和同一 CS 立即响应；
7. 在 Expressions 观察 `fpga_link_diagnostics` 和 `hmi_task2_diagnostics`。

关键诊断量：

- `status_valid_count`、`frame_valid_count`、`ack_count`、
  `ack_ready_low_count` 应持续增加；
- `status_crc_error_count`、`frame_crc_error_count`、`frame_dma_timeout_count` 应保持 0；
- `ack_ready_low_timeout_count` 应保持 0；
- `frame_retry_count` 只应在坏帧/超时后增加；
- `preload_complete_count` 应随完整屏幕快照增加；
- `command_count` 应随两个按钮操作增加；
- `tx_error_count` 应保持 0。

## 当前验证边界

- 2026-07-30 使用 CubeIDE 随附 GNU Tools for STM32 13.3 完成 Debug 实际编译和 ELF
  链接：`text=64252`、`data=472`、`bss=47616`，构建成功；
- SPI V1.0/R2 专项合同测试 16 项全部通过；其中主机测试直接编译并运行
  `fpga_protocol.c`，验证 `000`～`111` 全部 8 种 component VALID 排列均按
  `component_count=popcount(VALID)` 正确接收；
- `python -m unittest discover -s tests -v` 共执行 57 项，52 项通过；
  5 项未通过仍检查当前 FPGA+HMI 运行路径之外的历史 ADC/DAC/DDS/TIM 链路，
  本次未修改这些非活动模块；
- 串口屏 USART1 收发、按键命令、一周期/三周期切换及独立频谱显示已用固件自检数据
  完成实板验证；本次“首帧显示频谱、首次按键后实时刷新波形”逻辑仍待重新烧录验证；
- 曲线纵坐标已限制在 210 像素控件的安全范围内，当前映射范围为 8～201，避免
  淘晶驰单字节点超过控件高度后产生削顶；
- FPGA SPI 电气时序、DMA/D-Cache、正式帧解析、测量精度和整机 2 秒指标仍待
  FPGA 完成后进行实板验证；
- 当前 `HMI_CHART_SELF_TEST_ENABLE=0`，已切回 FPGA 正式数据主链路；
- 当前 `.ioc`、SPI3/USART1/DMA/GPIO 生成配置已作为本功能基线保留；
- IDE 工作区元数据、Debug 构建产物和本机启动配置不属于功能源码，不应混入提交；
- 本功能只在 `codex/fpga-hmi-bode` 分支开发和推送，不覆盖 `main`、`h743_pre1`
  或其他稳定分支。

## G 题要求对照

| 题目要求 | 当前实现 | 验证状态 |
|---|---|---|
| 按键显示 1 个或 3 个完整周期 | 从 FPGA 捕获的 3 周期数据生成两组 350 点缓存；按键切换并重画最新缓存，之后持续刷新 | 自检链路通过，真实周期数待 FPGA 数据验证 |
| 峰峰值、真有效值、基频 | 直接显示完整测量帧的 `vpp_uV`、`vrms_uV`、`fundamental_mHz` | 显示链路具备，5 mV/1 kHz 误差待校准实测 |
| 定性电压频谱 | 1312 个正频率谱点按区间最大值压缩为右侧 350 点频谱 | 自检显示通过，多分量相对位置/高度待 FPGA 实测 |
| 频率分辨率不大于 500 Hz | 协议固定 `bin_spacing_mHz=381470`，即 381.47 Hz/bin | 设计满足 |
| 最多 3 个频率分量及幅值 | 协议和页面均支持 `component[3]`、`t_comp1`～`t_comp3` | 字段具备，需确认实测长文本不被截断 |
| 抑制不低于 1 MHz 的 200 mVpp 干扰 | STM32 只接收 FPGA 滤波后的时域、频谱和参数 | 待模拟前端/FPGA 联调 |
| 每项上电后 2 秒内显示 | SPI 最大帧纯传输约 4.1 ms；按键切换并发送一条 350 点曲线约 151 ms | 理论满足，整机上电计时待测 |
| 单 5 V、BNC、50 Ω、屏幕不小于 6 英寸 | 当前使用 7 英寸淘晶驰屏 | 屏幕尺寸已确认，其余为整机硬件验收项 |

更完整的设计和验收步骤见
`docs/G26_FPGA_SPI_HMI_IMPLEMENTATION_PLAN.md`；本次实板状态和下一步联调清单见
`docs/G26_HMI_BOARD_TEST_HANDOFF_20260729.md`。
