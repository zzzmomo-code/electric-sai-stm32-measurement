# FPGA—STM32 SPI 实板联调指南

## 1. 本次联调目标

本指南用于第一次验证 FPGA 与 STM32H743 之间的 SPI V1.0 链路，先证明以下闭环正确，再接入真实 ADC、FFT 和测量算法：

```text
FPGA 冻结固定测试帧并拉高 DATA_READY
→ STM32 发送 GET_STATUS
→ STM32 发送 READ_FRAME 并用 DMA 读取完整帧
→ STM32 校验帧格式和 CRC
→ STM32 发送 ACK_FRAME
→ FPGA 校验 ACK 后拉低 DATA_READY
```

本次优先使用“最小无效帧”。它不会更新串口屏测量结果，但可以完整验证 SPI 电气连接、响应时序、状态帧、长帧 DMA、大小端、CRC、ACK 和 DATA_READY 握手。

协议字段和帧格式的唯一依据是：

`docs/FPGA_STM32_SPI_PROTOCOL_V1_FINAL_20260729.md`

## 2. 切回 FPGA 主模式

使用工程：

```text
h743_task2_20260727
```

这个工程当前就是 FPGA 主模式：

- `system_init()` 初始化 FPGA SPI 链路、测量换算和串口屏；
- `system_process()` 持续执行 FPGA 取帧、350 点显示换算和串口屏后台预装；
- 不启动片上 ADC、TIM2 采样、FFT 或无 FPGA 算法。

操作步骤：

1. 在 STM32CubeIDE 中选中 `h743_task2_20260727`。
2. 直接选择 `Build Project`，不要执行工作区级的 `Clean All`。
3. 检查 Build Console：最终目标必须是 `h743_task2_20260727.elf`，不是历史目标 `h743_pre1.elf`。
4. 烧录 `Debug/h743_task2_20260727.elf`。
5. FPGA 配置完成后复位 STM32，或两块板一起重新上电。

工程仍保留若干同名历史构建配置。若 Build Console 显示 `h743_pre1.elf`，立即停止烧录，回到 `Build Configurations → Set Active`，选择当前能生成 `h743_task2_20260727.elf` 的 Debug 配置。首次 SPI 联调期间不要使用命令行对所有历史配置执行全局 clean-build。

注意：STM32 固件无法代替 FPGA 配置。烧录 STM32 后仍需确认 FPGA 已加载用于本次联调的固定测试 bitstream。

## 3. 硬件接线

| 信号 | STM32H743 | STM32 方向 | FPGA 方向 | 说明 |
|---|---|---:|---:|---|
| `SPI3_SCK` | `PC10` | 输出 | 输入 | 20 MHz，空闲低 |
| `SPI3_MOSI` | `PC12` | 输出 | 输入 | MSB first |
| `SPI3_MISO` | `PC11` | 输入 | 输出 | CS 为高时建议高阻 |
| `FPGA_CS_N` | `PA15` | 输出 | 输入 | 低有效，软件控制 |
| `FPGA_DATA_READY` | `PD1` | 输入 | 输出 | 帧冻结完成后拉高 |
| `GND` | 任一可靠地端 | — | — | 两块板必须共地 |

电气要求：

- 默认按 3.3 V CMOS 电平连接；
- 未确认 FPGA Bank 电压前不得直接连接 5 V 信号；
- 建议线长尽量短，SCK、MOSI、MISO 旁边各安排可靠地回流；
- 首次上电前用万用表检查电源、地和相邻引脚是否短路；
- 不要把 `DATA_READY` 接成脉冲。它必须保持高电平直到收到正确 ACK。

## 4. SPI 固定参数

FPGA 端必须配置为：

| 参数 | 固定值 |
|---|---|
| 主从关系 | STM32 Master，FPGA Slave |
| SPI 模式 | Mode 0 |
| CPOL | 0 |
| CPHA | 0，第一个边沿采样 |
| 位序 | MSB first |
| 数据宽度 | 8 bit |
| SCK | 20 MHz |
| CS | 低有效 |
| 多字节字段 | 小端 |
| CRC | CRC-16/CCITT-FALSE |
| CRC 多项式 | `0x1021` |
| CRC 初值 | `0xFFFF` |
| 反射/XOROUT | 均关闭/0 |
| CRC 线上顺序 | 低字节在前 |

特别注意：STM32 发送两字节命令 `A5 CMD` 后，在同一个 CS 低电平事务内继续产生时钟。FPGA 必须从紧接着的第一个 dummy 字节开始返回响应 byte0，不允许额外插入周转 dummy。

## 5. FPGA 第一个固定测试帧

FPGA 先不要接真实测量算法，固定提供以下“最小无效帧”：

```text
frame_seq       = 0x01020304
time_count      = 1
spectrum_count  = 1312
component_count = 1
total_bytes     = 2756 = 0x00000AC4
time_samples[0] = 0
spectrum[0..1311] = 0
status          = FRAME_READY | RESULT_INVALID = 0x11
其余测量字段       = 0
```

必须先把完整帧和帧尾 CRC 写好并冻结，再拉高 `DATA_READY`。

### 5.1 GET_STATUS 期望

STM32 MOSI 命令：

```text
A5 01
```

之后 STM32继续发送 16 个 dummy 字节。FPGA 从第一个 dummy 对应的 MISO 字节开始返回：

```text
5A A5 01 11 04 03 02 01 C4 0A 00 00 00 00 05 46
```

其中状态 CRC 为 `0x4605`，线上顺序为 `05 46`。

### 5.2 READ_FRAME 期望

STM32 MOSI 命令：

```text
A5 02
```

在同一个 CS 低电平事务内，FPGA 从紧接着的第一个 dummy 字节开始返回 2756 字节。前四字节必须是：

```text
47 32 36 46
```

即 ASCII `G26F`。

整个帧除最后两个 CRC 字节外参与 CRC 计算，期望帧 CRC：

```text
0x0360
```

帧尾线上字节：

```text
60 03
```

READ_FRAME 结束后 FPGA 仍应保持：

- 当前 pending 帧内容不变；
- `frame_seq` 不变；
- `DATA_READY` 为高。

### 5.3 ACK_FRAME 期望

STM32 校验成功后发送：

```text
A5 03 04 03 02 01 09 A7
```

其中 ACK CRC 为 `0xA709`，线上顺序为 `09 A7`。

FPGA 只有在以下条件全部满足后才能释放 pending 帧并拉低 `DATA_READY`：

- ACK 长度为 8 字节；
- 前缀和命令正确；
- 序号为 `0x01020304`；
- CRC 正确。

## 6. 逻辑分析仪或 FPGA ILA 设置

同时观察：

```text
CS_N
SCK
MOSI
MISO
DATA_READY
```

建议：

- 逻辑分析仪采样率至少 100 MS/s，优先 200 MS/s；
- SPI 解码设置为 Mode 0、MSB first、8 bit、CS 低有效；
- 若外部逻辑分析仪无法可靠采集 20 MHz，请使用示波器确认电气质量，并用 FPGA ILA 检查字节序列；
- 第一轮不要为了适配低速分析仪擅自改变正式 20 MHz 配置。

正确时序应满足：

1. `DATA_READY` 拉高；
2. STM32 拉低 CS；
3. MOSI 出现 `A5 01`；
4. 第 3 个 SPI 字节开始，MISO 出现 `5A A5 01 11...`；
5. CS 拉高结束 GET_STATUS；
6. 下一次 CS 拉低，MOSI 出现 `A5 02`；
7. 第 3 个 SPI 字节开始，MISO 出现 `47 32 36 46...`；
8. 2756 字节读取完成后 CS 拉高，但 `DATA_READY` 仍为高；
9. 下一次 CS 拉低，MOSI 出现完整 8 字节 ACK；
10. CS 拉高后，FPGA 校验 ACK 并拉低 `DATA_READY`。

最大正式帧为 10254 字节，在 20 MHz 下纯移位时间约为 4.10 ms。STM32 DMA 超时设置为 20 ms。

## 7. STM32CubeIDE 诊断变量

烧录后暂停在主循环或使用实时 Expressions，添加：

```text
fpga_link_diagnostics
fpga_data_ready_flag
fpga_spi_dma_complete_flag
fpga_spi_dma_error_flag
hspi3.ErrorCode
hdma_spi3_rx.State
hdma_spi3_tx.State
```

完成一帧“最小无效帧”闭环后，关键诊断量应满足：

| 诊断量 | 期望 |
|---|---:|
| `status_read_count` | 至少 1 |
| `status_valid_count` | 至少 1 |
| `status_error_count` | 0 |
| `status_crc_error_count` | 0 |
| `frame_read_count` | 至少 1 |
| `frame_dma_start_count` | 至少 1 |
| `frame_dma_complete_count` | 至少 1 |
| `frame_dma_error_count` | 0 |
| `frame_dma_timeout_count` | 0 |
| `frame_valid_count` | 至少 1 |
| `frame_format_error_count` | 0 |
| `frame_crc_error_count` | 0 |
| `result_invalid_count` | 至少 1 |
| `ack_count` | 至少 1 |
| `ack_error_count` | 0 |
| `published_snapshot_count` | 0 |
| `last_frame_sequence` | `0x01020304` |
| `last_frame_length` | `2756` |
| `last_protocol_result` | `FPGA_PROTOCOL_OK` |

`data_ready_irq_count` 不要求必须为 1：如果 FPGA 在 STM32 启动前已经把 `DATA_READY` 拉高，STM32 可能没有收到上升沿，但固件还会读取 PD1 当前电平并正常取帧。

## 8. 分层排障

### 8.1 DATA_READY 始终为低

先检查：

- FPGA 固定帧是否真正打包完成；
- PD1 与 FPGA 引脚是否接反；
- 两板是否共地；
- FPGA 引脚约束和 Bank 电压；
- FPGA 是否把 DATA_READY 做成了短脉冲。

### 8.2 DATA_READY 为高，但没有 CS/SCK

检查：

- STM32 是否烧录了 `h743_task2_20260727`；
- PC10、PA15 是否被其他外设占用；
- `fpga_link_diagnostics.status_read_count` 是否增加；
- `hspi3.ErrorCode` 是否非零；
- 程序是否持续执行 `system_process()`。

### 8.3 MISO 响应整体晚一个字节

如果 `5A` 出现在第 4 个 SPI 字节，而不是第 3 个字节，说明 FPGA 加了一个额外周转 dummy。修正 FPGA SPI 从机：收到 `A5 01` 或 `A5 02` 后，下一个字节立即装载响应 byte0。

### 8.4 状态 CRC 错误

依次检查：

- SPI 是否确实为 Mode 0；
- 是否 MSB first；
- 状态字段是否小端；
- CRC 是否使用 CCITT-FALSE；
- CRC 计算是否覆盖状态前 14 字节；
- 线上 CRC 是否低字节在前。

### 8.5 DMA 超时

检查：

- 状态中的 `frame_length` 是否与 FPGA 实际可输出字节数一致；
- FPGA 是否在长帧中途停止移位或重置读指针；
- CS 低期间帧缓存是否保持不变；
- STM32 SCK 是否连续产生；
- FPGA MISO 输出时序是否满足 20 MHz。

### 8.6 帧格式错误

重点检查：

- 帧头是否为 `47 32 36 46`；
- `version=1`；
- `header_bytes=128`；
- `total_bytes=128 + time_count×2 + 1312×2 + 2`；
- `time_count` 是否为 1～3750；
- `spectrum_count` 是否固定为 1312；
- 状态中的序号、长度与帧头完全一致。

### 8.7 帧 CRC 错误

检查：

- CRC 是否覆盖从帧 byte0 到倒数第 3 字节；
- 最后两字节是否只存放 CRC；
- CRC 线上是否为低字节在前；
- pending 帧是否在 READ_FRAME 或重读期间被算法覆盖。

STM32 遇到坏帧不会 ACK，会对同一 pending 帧最多重读 3 次。因此此时 FPGA 必须保持 `DATA_READY` 和帧内容不变。

### 8.8 ACK 已出现，但 DATA_READY 不下降

检查 FPGA ACK 解析：

- 是否按 8 字节接收；
- 是否按小端还原 `frame_seq`；
- 是否使用前 6 字节计算 CRC；
- 是否把线上 `09 A7` 还原为 `0xA709`；
- 是否只在 CS 上升沿或完整 ACK 校验后原子释放 pending 帧。

## 9. 第二阶段：固定有效帧

最小无效帧闭环通过后，再发送固定有效合成帧，例如：

```text
基频        = 12.5 kHz
Vpp         = 3.300 V
Vrms        = 1.166 V
时域        = 3 周期固定正弦波或三角波
频谱        = 对应频率桶一个固定主峰
component 0 = 有效
RESULT_INVALID = 0
```

验收：

- `published_snapshot_count` 增加；
- 串口屏参数与固定字段一致；
- 一周期图约显示 1 个周期；
- 三周期图约显示 3 个周期；
- 频谱图主峰位置正确；
- 正确 ACK 后 DATA_READY 下降。

只有固定有效帧通过后，才接入真实 ADC、FFT、I/Q、谐波识别和测量计算。

## 10. 最终验收清单

- [ ] 两板共地，电平和 FPGA Bank 电压正确；
- [ ] SPI Mode 0、8 bit、MSB first、20 MHz；
- [ ] DATA_READY 在 ACK 前始终保持高；
- [ ] GET_STATUS 第一个响应字节无额外 dummy；
- [ ] 状态固定 16 字节且 CRC 正确；
- [ ] READ_FRAME 第一个响应字节为 `G`；
- [ ] 最小帧 2756 字节连续完成；
- [ ] 状态、帧头和 ACK 的序号一致；
- [ ] 多字节字段全部小端；
- [ ] 三类 CRC 均为 CCITT-FALSE，线上低字节在前；
- [ ] 错误帧不 ACK，重复读取内容不变；
- [ ] 正确 ACK 后 DATA_READY 下降；
- [ ] STM32 所有错误计数为 0；
- [ ] 固定有效帧能更新串口屏；
- [ ] 保存逻辑分析仪/ILA 波形和诊断量截图；
- [ ] 记录 STM32 提交号、FPGA 提交号和 bitstream 文件名。

## 11. 联调记录模板

```text
日期：
STM32 工程：h743_task2_20260727
STM32 Git 提交：
STM32 ELF：
FPGA Git 提交：
FPGA bitstream：
电源电压：
逻辑分析仪/ILA 文件：

阶段 A 最小无效帧：通过 / 未通过
阶段 B 固定有效帧：通过 / 未通过
阶段 C 实时算法帧：通过 / 未通过

首次异常位置：
对应诊断量：
逻辑分析仪现象：
修改内容：
复测结果：
```
