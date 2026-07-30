# FPGA—STM32 SPI 通信协议 V1.0（2026-07-30 最终冻结版）

## 0. 文档地位

本文是 FPGA 与 STM32H743 之间的最终字节级接口约定，已经与 FPGA 队友提供的
`FPGA-SPI.md` 逐字段对齐，并与当前 STM32 固件实现一致。

- 协议版本：V1.0；
- 状态：冻结；
- 本文替代 STM32 工程中 2026-07-29 及以前的旧帧头说明；
- 禁止在不升级版本号的情况下修改字段、偏移、字节序、CRC 范围或 SPI 时序；
- FPGA 发送的是测量数据，不发送淘晶驰指令或屏幕坐标。

## 1. 硬件连接

| 信号 | FPGA 方向 | FPGA XC7Z020 管脚 | STM32H743 管脚 | 说明 |
|---|---:|---:|---:|---|
| `spi_sck` | 输入 | U19 | PC10 / SPI3_SCK | Mode 0 时钟 |
| `spi_mosi` | 输入 | M19 | PC12 / SPI3_MOSI | STM32 命令和 dummy |
| `spi_miso` | 输出 | J14 | PC11 / SPI3_MISO | FPGA 返回数据 |
| `spi_cs_n` | 输入 | N20 | PA15 / GPIO 输出 | 软件片选，低有效 |
| `data_ready` | 输出 | M20 | PD1 / EXTI1 | 高有效，帧就绪 |
| GND | — | GND | GND | 必须共地 |

电气与 SPI 参数：

- 3.3 V 逻辑电平；
- STM32H743 为 SPI Master，FPGA 为 SPI Slave；
- SPI Mode 0：CPOL=0、CPHA=0；
- 初始 SCK 为 20 MHz；
- 8 bit 数据宽度，MSB-first；
- 多字节整数在字节流中采用 little-endian；
- `CS_N=1` 时 FPGA 的 MISO 必须为高阻态；
- Mode 0：下降沿更新数据，上升沿采样数据。

## 2. 同一 CS 内立即返回

所有读命令必须在发送命令的同一次 CS 事务内返回：

```text
CS_N       \________________________________________________/
MOSI        0xA5        CMD         0x00        0x00   ...
MISO        0x00        0x00        RESP[0]     RESP[1] ...
字节序号       0           1            2           3
```

硬性规则：

1. STM32 拉低 `CS_N`；
2. STM32 依次发送 `A5` 和 `CMD`；
3. 命令阶段同时收到的两个 MISO 字节丢弃；
4. `CS_N` 保持为低；
5. 紧接着发送的第一个 dummy `00` 对应 `RESP[0]`；
6. 不允许插入额外 turnaround dummy；
7. 响应完成后 STM32 才拉高 `CS_N`；
8. dummy 全部为 `00`；
9. 命令发送与 DMA 收发之间不得拉高 CS。

## 3. 命令

每条命令以 `A5 CMD` 开始。

| CMD | 名称 | 作用 |
|---:|---|---|
| `01` | `GET_STATUS` | 查询帧状态、序号和长度 |
| `02` | `READ_FRAME` | 从帧首读取完整测量帧 |
| `03` | `ACK_FRAME` | 确认并释放指定序号的帧 |

## 4. GET_STATUS

完整 CS 事务共 18 字节：

```text
MOSI:
A5 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00

MISO:
00 00 5A A5 VER STATE SEQ0 SEQ1 SEQ2 SEQ3
      LEN0 LEN1 LEN2 LEN3 RSV0 RSV1 CRC0 CRC1
```

16 字节响应布局：

| 响应偏移 | 长度 | 字段 | V1.0 要求 |
|---:|---:|---|---|
| 0 | 2 | magic | `5A A5` |
| 2 | 1 | version | `01` |
| 3 | 1 | state | 见下表 |
| 4 | 4 | frame_seq | little-endian |
| 8 | 4 | frame_length | 含帧末 CRC16 |
| 12 | 2 | reserved | 固定为 `0000` |
| 14 | 2 | crc16 | 前 14 字节 CRC，低字节先传 |

`state`：

| bit | 名称 | 含义 |
|---:|---|---|
| 0 | `FRAME_READY` | 当前有稳定完整帧 |
| 1 | `FPGA_BUSY` | FPGA 正在计算 |
| 2 | `ADC_OTR` | ADC 出现超量程 |
| 3 | `FRAME_DROPPED` | 出现过快照丢弃 |
| 4 | `RESULT_INVALID` | 当前测量结果无效 |
| 5～7 | reserved | 固定为 0 |

## 5. READ_FRAME

STM32 先从 GET_STATUS 得到 `frame_length`，然后开启新的 CS 事务：

```text
MOSI: A5 02 + frame_length 个 00
MISO: 00 00 + frame[0] ... frame[frame_length-1]
```

- 第一个 dummy 必须立即返回 `frame[0]`；
- 正常帧前四字节必须为 `47 32 36 46`，即 ASCII `"G26F"`；
- CS 提前变高时，本次读取取消，FPGA 保留快照；
- 下次 READ_FRAME 必须重新从 `frame[0]` 开始；
- STM32 只在完整帧 CRC 与全部格式检查通过后发送 ACK。

## 6. ACK_FRAME

```text
MOSI: A5 03 SEQ0 SEQ1 SEQ2 SEQ3 CRC0 CRC1
MISO: 00 00 00   00   00   00   00   00
```

- `SEQ0` 是序号最低字节；
- ACK CRC 覆盖 `A5 03 SEQ0 SEQ1 SEQ2 SEQ3`；
- CRC 低字节先传；
- 只有序号匹配且 ACK CRC 正确时，FPGA 才释放当前快照；
- 错误 ACK 不得修改当前快照和 DATA_READY。

## 7. DATA_READY 时序

- 高电平表示存在完整、稳定、可重复读取的帧；
- 发布帧后保持高电平，直到收到正确 ACK；
- STM32 同时使用 EXTI 上升沿和 GPIO 电平检查，避免漏掉上电前已经拉高的情况；
- 正确 ACK 后，FPGA 必须在 2 个 50 MHz 时钟周期内拉低 DATA_READY；
- 即使内部已有下一帧，重新拉高前也必须保持低至少 50 个 50 MHz 周期，即 1 µs；
- STM32 读取期间 FPGA 不得覆盖当前快照；
- `CS_N` 上升只结束事务，不释放快照；只有正确 ACK 能释放快照。

## 8. 完整测量帧

```text
offset 0:
128-byte header
+ time_count × int16_t 时域样点
+ 1312 × uint16_t 频谱幅值
+ 2-byte CRC16
```

严格长度：

```text
frame_length = 128 + time_count×2 + spectrum_count×2 + 2
spectrum_count = 1312
frame_length = 2754 + time_count×2
```

因此：

- `time_count=75` 时，最小帧长为 2904 字节；
- `time_count=3750` 时，最大帧长为 10254 字节。

### 8.1 128 字节 header

| 偏移 | 长度 | 字段 | V1.0 值或含义 |
|---:|---:|---|---|
| `0x00` | 4 | magic | `47 32 36 46` / `"G26F"` |
| `0x04` | 1 | version_major | `1` |
| `0x05` | 1 | version_minor | `0` |
| `0x06` | 2 | header_length | `128` |
| `0x08` | 4 | frame_length | 完整帧总字节数 |
| `0x0C` | 4 | frame_seq | 帧序号 |
| `0x10` | 8 | timestamp_50mhz | 50 MHz 时钟计数 |
| `0x18` | 4 | flags | 见 header flags |
| `0x1C` | 4 | time_sample_rate_hz | `12500000` |
| `0x20` | 2 | time_count | `75～3750` |
| `0x22` | 2 | time_uV_per_lsb | `10` |
| `0x24` | 4 | fft_sample_rate_hz | `1562500` |
| `0x28` | 2 | fft_length | `4096` |
| `0x2A` | 2 | spectrum_count | `1312` |
| `0x2C` | 4 | bin_spacing_mHz | `381470` |
| `0x30` | 2 | spectrum_uV_per_lsb | `10`，峰值幅度 |
| `0x32` | 1 | component_count | `0～3` |
| `0x33` | 1 | reserved0 | `0` |
| `0x34` | 4 | vpp_uV | 峰峰值，µV |
| `0x38` | 4 | vrms_uV | 真有效值，µV |
| `0x3C` | 4 | dc_uV | 有符号直流分量，µV |
| `0x40` | 4 | fundamental_mHz | 基频，mHz |
| `0x44` | 16 | component[0] | 第一个分量 |
| `0x54` | 16 | component[1] | 第二个分量 |
| `0x64` | 16 | component[2] | 第三个分量 |
| `0x74` | 2 | calibration_version | 校准参数版本 |
| `0x76` | 2 | reserved1 | `0` |
| `0x78` | 4 | dropped_frame_count | 累计丢弃快照数 |
| `0x7C` | 4 | reserved2 | 全部为 `0` |

header `flags`：

| bit | 名称 | 含义 |
|---:|---|---|
| 0 | `MEASUREMENT_VALID` | 本帧主要测量结果有效 |
| 1 | `ADC_OTR` | 本窗口 ADC 超量程 |
| 2 | `ADC_SATURATION` | 数字链路检测到饱和 |
| 3 | `FFT_ERROR` | FFT 帧或 AXI 事件异常 |
| 4 | `IQ_UNSTABLE` | 至少一个分量 IQ 精测不稳定 |
| 5 | `FRAME_DROPPED` | 上次发布后发生过快照丢弃 |
| 6～31 | reserved | 固定为 0 |

### 8.2 每个分量的 16 字节布局

| 分量内偏移 | 长度 | 字段 | 含义 |
|---:|---:|---|---|
| `+0x00` | 4 | frequency_mHz | 精测频率，mHz |
| `+0x04` | 4 | amplitude_peak_uV | 正弦峰值幅度，µV |
| `+0x08` | 2 | fft_bin | FFT 粗定位 bin |
| `+0x0A` | 2 | fft_delta_q15 | 有符号 Q1.15 插值偏移 |
| `+0x0C` | 1 | harmonic_order | 1=基波，2/3/...=谐波 |
| `+0x0D` | 1 | flags | 见下表 |
| `+0x0E` | 2 | reserved | 固定为 0 |

分量 `flags`：

| bit | 名称 | 含义 |
|---:|---|---|
| 0 | `VALID` | 该分量记录有效 |
| 1 | `IQ_REFINED` | 频率已经 IQ 精修 |
| 2 | `AMPLITUDE_VALID` | 峰值幅度有效 |
| 3～7 | reserved | 固定为 0 |

STM32 V1.0 的严格接收要求：

- 前 `component_count` 个分量必须设置 `VALID`；
- `component_count` 之后的未使用分量 `flags` 必须为 0；
- 三个分量的 `reserved` 均必须为 0；
- 所有未定义 header/component flag 均必须为 0。

### 8.3 时域数组

- 固定采样率 12.5 MSPS；
- 内容为 FPGA 已完成数字低通的连续 3 周期数据；
- 每点是 little-endian `int16_t` 二进制补码；
- 比例为 10 µV/LSB；
- 数组从偏移 128 开始；
- 样点 `i` 的偏移：`128 + i×2`。

### 8.4 频谱数组

- 4096 点 FFT，采样率 1.5625 MSPS；
- 发送 bin 0～1311，共 1312 点；
- 每点是 little-endian `uint16_t`；
- 比例为 10 µV_peak/LSB；
- bin 间隔精确值为 381.4697265625 Hz，头中四舍五入写 `381470 mHz`；
- 首个频谱点偏移：`128 + time_count×2`。

## 9. CRC

状态、ACK 和完整帧统一使用：

```text
CRC-16/CCITT-FALSE
poly    = 0x1021
init    = 0xFFFF
refin   = false
refout  = false
xorout  = 0x0000
```

- 状态 CRC 覆盖状态响应前 14 字节；
- ACK CRC 覆盖 `A5 03 + 4 字节 frame_seq`；
- 完整帧 CRC 覆盖 `frame[0]` 到 `frame[frame_length-3]`；
- CRC 自身不参与计算；
- 在线发送均为低字节在前、高字节在后；
- 标准检查向量：ASCII `"123456789"` 的结果必须为 `0x29B1`。

## 10. 有效性与 ACK 规则

STM32 对完整帧执行以下顺序：

1. 校验状态 magic/version/reserved/state/length/CRC；
2. 校验帧 magic、版本、固定字段、reserved、分量布局和精确长度；
3. 校验 `frame_seq` 与 GET_STATUS 一致；
4. 校验完整帧 CRC；
5. 以上任一失败：不 ACK，最多重新 READ_FRAME 3 次；
6. 格式和 CRC 正确：无论测量是否有效，都发送 ACK；
7. 只有同时满足以下两项，才发布给串口屏：
   - `GET_STATUS.state.RESULT_INVALID = 0`；
   - `header.flags.MEASUREMENT_VALID = 1`。

推荐 FPGA 保持两者一致：

```text
有效测量：
status.RESULT_INVALID = 0
header.MEASUREMENT_VALID = 1

无效测量：
status.RESULT_INVALID = 1
header.MEASUREMENT_VALID = 0
```

## 11. 双方联调测试向量

### 11.1 ACK 向量

条件：

```text
frame_seq = 0x01020304
```

STM32 应发送：

```text
A5 03 04 03 02 01 09 A7
```

其中 ACK CRC 为 `0xA709`，线上为 `09 A7`。

### 11.2 最小有效状态响应

条件：

```text
state        = 0x01  // FRAME_READY
frame_seq    = 0x01020304
time_count   = 75
frame_length = 2904 = 0x00000B58
```

FPGA 的 16 字节状态响应必须为：

```text
5A A5 01 01 04 03 02 01 58 0B 00 00 00 00 2E 7C
```

状态 CRC 为 `0x7C2E`。

若使用同一长度的无效结果，`state=0x11`，对应：

```text
5A A5 01 11 04 03 02 01 58 0B 00 00 00 00 13 29
```

### 11.3 最小有效完整帧向量

固定条件：

```text
frame_seq             = 0x01020304
timestamp_50mhz       = 0x0000000011223344
flags                 = MEASUREMENT_VALID
time_count            = 75
component_count       = 1
vpp_uV                = 3300000
vrms_uV               = 1166000
dc_uV                 = 0
fundamental_mHz       = 12345000
component[0].frequency_mHz     = 12345000
component[0].amplitude_peak_uV = 1650000
component[0].fft_bin           = 32
component[0].fft_delta_q15     = 0
component[0].harmonic_order    = 1
component[0].flags             = 0x07
calibration_version   = 1
75 个时域点全部为 0
1312 个频谱点全部为 0
```

128 字节 header：

```text
47 32 36 46 01 00 80 00 58 0B 00 00 04 03 02 01
44 33 22 11 00 00 00 00 01 00 00 00 20 BC BE 00
4B 00 0A 00 84 D7 17 00 00 10 20 05 1E D2 05 00
0A 00 01 00 A0 5A 32 00 B0 CA 11 00 00 00 00 00
A8 5E BC 00 A8 5E BC 00 50 2D 19 00 20 00 00 00
01 07 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
00 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00
```

该帧总长 2904 字节，帧末 CRC 必须为：

```text
CRC = 0xDB1E
线上尾字节 = 1E DB
```

## 12. FPGA 实现验收清单

- [ ] Mode 0、20 MHz、8 bit、MSB-first；
- [ ] 多字节字段 little-endian；
- [ ] 第一个 dummy 立即返回响应第 0 字节；
- [ ] GET_STATUS 固定返回 16 字节；
- [ ] READ_FRAME 每次从 `"G26F"` 开始；
- [ ] CS 中断后下次仍从帧首开始；
- [ ] 在正确 ACK 前不覆盖当前快照；
- [ ] ACK 序号和 CRC 均正确才释放；
- [ ] DATA_READY 正确 ACK 后先拉低，再发布下一帧；
- [ ] 128 字节头偏移与本文完全一致；
- [ ] component 固定为 16 字节；
- [ ] `dc_uV` 位于 `0x3C`，`fundamental_mHz` 位于 `0x40`；
- [ ] 所有 reserved 和保留 flag 发送为 0；
- [ ] 最小/最大长度公式正确；
- [ ] 三类 CRC 均为 CCITT-FALSE 且低字节先传；
- [ ] 通过第 11 节三个冻结测试向量。
