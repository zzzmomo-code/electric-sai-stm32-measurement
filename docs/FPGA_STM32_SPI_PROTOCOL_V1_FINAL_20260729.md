# FPGA—STM32 SPI 通信协议 V1.0 最终对齐文档

> 日期：2026-07-29
> 面向对象：FPGA 端 RTL、仿真和联调负责人
> 对端：STM32H743，当前工程 `h743_task2_20260727`
> 状态：**V1.0 最终实现合同**

## 0. 文档效力

FPGA 端请严格按照本文实现。本文已经逐项对齐 STM32 当前实际代码：

- `Core/User/fpga_protocol.h`
- `Core/User/fpga_protocol.c`
- `Core/User/fpga_link.h`
- `Core/User/fpga_link.c`

若旧聊天记录、旧 UART 协议、`D:\QQ\FPGA_STM32_SPI_PROTOCOL_V1.md` 或其他草稿与本文冲突，**以本文和上述 STM32 源码为准**。

V1.0 不再使用以下旧设计：

- 不使用 FPGA→STM32 UART；
- 不传旧的 `AA 55 ... 0D 0A` 帧；
- 不传 `mag2_hi + phase` 扫频数据；
- 不传屏幕像素或淘晶驰指令；
- 不在命令和返回数据之间增加额外“周转 dummy 字节”；
- 不允许 FPGA 在收到正确 ACK 前覆盖当前帧。

FPGA 只负责产生完整测量结果并作为 SPI 从机提供数据；STM32 负责校验、缓存、压缩、坐标映射和串口屏显示。

---

## 1. 硬件接口

### 1.1 引脚约定

复用底板 ADS8688 外接排针：

| 排针脚 | STM32 引脚 | 信号名 | 方向 | 说明 |
|---|---|---|---|---|
| 1 | PD1 | `FPGA_DATA_READY` | FPGA→STM32 | 高电平表示一帧完整数据已经稳定 |
| 2 | PC12 / SPI3_MOSI | `MOSI` | STM32→FPGA | 命令和 ACK |
| 3 | PA15 | `FPGA_CS_N` | STM32→FPGA | 软件片选，低有效 |
| 4 | PC10 / SPI3_SCK | `SCK` | STM32→FPGA | SPI 时钟 |
| 5 | GND | `GND` | 公共地 | 必须连接 |
| 6 | PC11 / SPI3_MISO | `MISO` | FPGA→STM32 | 状态和完整帧 |
| 7 | GND | `GND` | 公共地 | 必须连接 |
| 8 | PD0 | 未使用 | — | V1.0 保留 |

电气要求：

- 所有信号均为 **3.3 V CMOS**；
- STM32 和 FPGA 必须共地；
- FPGA 与 ADS8688 不得同时占用这一组排针；
- `CS_N=1` 时 FPGA 的 `MISO` 必须进入高阻态；
- `DATA_READY` 复位默认低电平。

### 1.2 SPI 参数

| 参数 | 固定值 |
|---|---|
| 主从关系 | STM32 Master，FPGA Slave |
| SPI 模式 | Mode 0 |
| CPOL | 0，SCK 空闲低 |
| CPHA | 0，第一个上升沿采样 |
| 数据位宽 | 8 bit |
| 位序 | MSB first |
| 多字节整数 | Little Endian |
| 初始 SCK | 20 MHz |
| CS | STM32 软件控制，低有效 |

Mode 0 的 FPGA 动作：

- 在 SCK 上升沿采样 MOSI；
- 在 SCK 下降沿更新 MISO；
- STM32 在下一上升沿采样该 MISO 位。

---

## 2. 最关键时序：同一 CS 内立即返回

所有读取命令都在**同一次 CS 拉低期间**完成：

```text
CS_N  ──────┐________________________________________┌──────
            │                                        │
MOSI        A5    CMD    00    00    00    ...       │
MISO        xx    xx    R0    R1    R2    ...        │
字节序号     0     1     2     3     4
```

规则：

1. STM32 先发送 `0xA5` 和 `CMD`；
2. FPGA 在这两个命令字节期间返回的 MISO 数据会被 STM32丢弃；
3. **紧接着的第一个 dummy 字节就必须返回响应字节 R0**；
4. 命令和响应之间没有额外周转字节；
5. CS 全程保持低电平；
6. STM32 在命令发送结束与后续 DMA 开始之间可能短暂停止 SCK，FPGA 必须保持事务状态和读指针；
7. 只有 CS 上升沿才能结束本次事务并复位命令解析器/读指针。

### 2.1 FPGA 字节发送的半周期要求

接收 `CMD` 的最后一位时：

1. FPGA 在第 8 个 SCK 上升沿采到 `CMD[0]`；
2. FPGA 必须在紧接着的下降沿把 `R0[7]` 放到 MISO；
3. STM32 在下一个上升沿采到 `R0[7]`。

因此不能等到下一个完整字节后才准备返回数据。SPI 从机的发送移位寄存器必须在命令字节最后一个采样沿之后立即装载。

---

## 3. 总体通信流程

```text
FPGA完成一帧
    ↓
冻结当前待发送缓冲区
    ↓
DATA_READY = 1
    ↓
STM32: GET_STATUS
    ↓
状态、序号、长度、状态CRC正确？
    ├─否：稍后重新查询
    └─是
       ↓
STM32: READ_FRAME
       ↓
帧头、长度、固定字段、序号、帧CRC正确？
    ├─否：不ACK，重新READ_FRAME，最多再读3次
    └─是
       ↓
STM32发布或丢弃RESULT_INVALID结果
       ↓
STM32: ACK_FRAME
       ↓
FPGA核对ACK序号和CRC
    ├─错误：保持缓冲区和DATA_READY不变
    └─正确：释放当前帧；无下一帧则DATA_READY=0
```

重要：

- STM32 对同一帧最多可能发起 4 次 `READ_FRAME`；
- 每次 `READ_FRAME` 都必须从帧第 0 字节重新开始；
- CRC 错误或传输中断时 STM32 不发 ACK；
- 正确 ACK 前，当前帧的任何字节都不得改变。

---

## 4. SPI 命令

每条命令均以 `0xA5 + CMD` 开始：

| CMD | 名称 | 功能 |
|---|---|---|
| `0x01` | `GET_STATUS` | 查询状态、当前帧序号和帧长度 |
| `0x02` | `READ_FRAME` | 从帧首读取完整测量帧 |
| `0x03` | `ACK_FRAME` | 通知 FPGA 当前帧已经正确接收 |

未知前缀或未知命令：

- 不修改任何帧状态；
- MISO 可以输出 `0x00`；
- 忽略到 CS 上升沿，再回到等待新命令状态。

---

## 5. GET_STATUS：查询状态

### 5.1 总线事务

STM32 在同一个 CS 内发送：

```text
MOSI: A5 01 + 16个00
MISO: xx xx + 16字节fpga_status
```

总事务长度为 18 字节。MISO 前两个字节被忽略。

### 5.2 16 字节状态结构

| 偏移 | 长度 | 字段 | 固定/含义 |
|---:|---:|---|---|
| 0 | 1 | `magic[0]` | `0x5A` |
| 1 | 1 | `magic[1]` | `0xA5` |
| 2 | 1 | `version` | `0x01` |
| 3 | 1 | `state` | 状态位 |
| 4 | 4 | `frame_seq` | `uint32_t`，小端 |
| 8 | 4 | `frame_length` | `uint32_t`，小端，**包含帧尾 CRC16** |
| 12 | 2 | `reserved` | 小端，必须为 0 |
| 14 | 2 | `crc16` | 小端；覆盖状态字节 0～13 |

状态位：

| 位 | 名称 | 含义 |
|---:|---|---|
| bit0 | `FRAME_READY` | 当前完整帧可读取 |
| bit1 | `FPGA_BUSY` | FPGA 正在计算下一帧 |
| bit2 | `ADC_OTR` | ADC 曾发生超量程 |
| bit3 | `FRAME_DROPPED` | 缓冲区忙导致新结果丢弃/溢出 |
| bit4 | `RESULT_INVALID` | 当前测量结果无效，但帧结构仍必须正确 |
| bit5～7 | 保留 | V1.0 必须为 0 |

### 5.3 状态与待发送帧必须是同一个快照

当 `DATA_READY=1` 且 `FRAME_READY=1`：

- `frame_seq` 必须等于完整帧头中的 `frame_seq`；
- `frame_length` 必须等于完整帧头中的 `total_bytes`；
- `frame_length = 2754 + 2 × time_count`；
- 合法范围是 2756～10254 字节；
- 在收到正确 ACK 前，状态中的序号和长度不得改变。

STM32 正常只在 `DATA_READY=1` 时查询状态。为了增强调试容错，如果没有帧时仍收到查询：

- 返回格式和 CRC 仍必须正确；
- 清除 `FRAME_READY`；
- `frame_length` 建议返回上一帧合法长度；复位后尚无历史帧时返回 2756；
- 不要返回 0，因为 STM32 状态解析器会把过小长度统计为协议错误。

---

## 6. READ_FRAME：读取完整帧

### 6.1 总线事务

STM32 新拉低一次 CS：

```text
MOSI: A5 02 + frame_length个00
MISO: xx xx + frame[0] ... frame[frame_length-1]
```

要求：

- 第一个 dummy 字节对应完整帧 `frame[0]`；
- `frame[0..3]` 必须是 ASCII `"G26F"`，即 `47 32 36 46`；
- CS 中途拉高表示本次读取取消；
- 取消后当前帧仍保留；
- 下次 `READ_FRAME` 必须重新从 `frame[0]` 返回；
- FPGA 不得因为读过一次就移动或释放缓冲区。

STM32 完整帧 DMA 超时为 20 ms。最大帧在 20 MHz 下的纯传输时间约为：

```text
10254 × 8 / 20 MHz ≈ 4.10 ms
```

FPGA 必须允许事务中 SCK 暂停，但有 SCK 时应连续提供数据。

---

## 7. ACK_FRAME：释放当前帧

### 7.1 总线事务

ACK 是单次 8 字节写事务，不要求 FPGA 返回有效数据：

```text
偏移:  0   1   2     3     4     5     6     7
MOSI:  A5  03  SEQ0  SEQ1  SEQ2  SEQ3  CRC0  CRC1
```

- `SEQ0` 是 `frame_seq` 最低字节；
- ACK CRC 覆盖前 6 字节：

```text
A5 03 SEQ0 SEQ1 SEQ2 SEQ3
```

- CRC 结果仍按低字节、再高字节发送。

FPGA 只有同时满足以下条件才接受 ACK：

1. ACK 共收到完整 8 字节；
2. CRC 正确；
3. ACK 序号等于当前待确认帧序号；
4. 当前确实存在待确认帧。

接受后：

- 释放当前待发送缓冲区；
- 没有下一帧时拉低 `DATA_READY`；
- 有下一帧排队时可原子切换到下一帧，并保持 `DATA_READY=1`；
- 不得让 STM32 看到“新状态 + 旧帧”或“旧状态 + 新帧”的混合状态。

ACK CRC 错误、序号不匹配或字节不完整时：

- 当前帧保持不变；
- `DATA_READY` 保持高；
- 下一次查询/读取仍返回相同序号和相同帧。

STM32 也会对以下帧发送 ACK：

- CRC 正确的重复帧；
- 结构正确但状态含 `RESULT_INVALID` 的帧。

这用于防止链路死锁。FPGA 不得因为结果无效而拒绝 ACK。

---

## 8. 完整测量帧布局

```text
┌──────────────────────────────┐
│ 128字节 fpga_frame_header    │
├──────────────────────────────┤
│ time_count × int16_t 时域数据│
├──────────────────────────────┤
│ 1312 × uint16_t 频谱数据     │
├──────────────────────────────┤
│ CRC16，2字节，小端           │
└──────────────────────────────┘
```

长度：

```text
total_bytes
= 128 + time_count×2 + 1312×2 + 2
= 2754 + time_count×2
```

约束：

```text
1 <= time_count <= 3750
2756 <= total_bytes <= 10254
```

帧尾 CRC 覆盖：

```text
从 frame[0] 的字符 'G'
到最后一个频谱数据字节
不包含 CRC 自身
```

CRC 结果在帧尾按低字节、高字节发送。

---

## 9. 128 字节帧头逐字节定义

下表中的多字节字段全部为小端：

| 偏移 | 长度 | 字段 | V1.0 要求 |
|---:|---:|---|---|
| 0 | 4 | `magic` | ASCII `"G26F"` = `47 32 36 46` |
| 4 | 1 | `protocol_version` | 固定 1 |
| 5 | 1 | `frame_type` | 固定 1，完整测量帧 |
| 6 | 2 | `header_bytes` | 固定 128 |
| 8 | 4 | `total_bytes` | 头+时域+频谱+CRC 的总长度 |
| 12 | 4 | `frame_seq` | 与 GET_STATUS 相同 |
| 16 | 8 | `timestamp_50m` | FPGA 50 MHz 自由运行计数 |
| 24 | 4 | `flags` | V1.0 保留，必须为 0 |
| 28 | 4 | `time_sample_rate_hz` | 固定 12500000 |
| 32 | 2 | `time_count` | 1～3750 |
| 34 | 1 | `captured_cycles` | 固定 3 |
| 35 | 1 | `time_format` | 固定 1：`int16_t`，10 µV/LSB |
| 36 | 4 | `fft_sample_rate_hz` | 固定 1562500 |
| 40 | 2 | `fft_length` | 固定 4096 |
| 42 | 2 | `spectrum_count` | 固定 1312 |
| 44 | 4 | `bin_spacing_mHz` | 固定 381470，即 381.470 Hz/bin |
| 48 | 1 | `spectrum_format` | 固定 1：`uint16_t`，10 µV_peak/LSB |
| 49 | 1 | `window_type` | 固定 1：Hann |
| 50 | 1 | `component_count` | 1～3 |
| 51 | 1 | `reserved0` | 必须为 0 |
| 52 | 4 | `vpp_uV` | 峰峰值，单位 µV |
| 56 | 4 | `vrms_uV` | 真有效值，单位 µV |
| 60 | 4 | `fundamental_mHz` | 基频，单位 0.001 Hz |
| 64 | 4 | `dc_offset_uV` | `int32_t`，输入信号直流，单位 µV |
| 68 | 12 | `component[0]` | 第 1 个频率分量 |
| 80 | 12 | `component[1]` | 第 2 个频率分量 |
| 92 | 12 | `component[2]` | 第 3 个频率分量 |
| 104 | 2 | `calibration_revision` | 校准版本；无校准时为 0 |
| 106 | 2 | `reserved1` | 必须为 0 |
| 108 | 4 | `dropped_frames` | 累计丢帧计数 |
| 112 | 2 | `adc_min_code` | `int16_t`，本次 ADC 最小码 |
| 114 | 2 | `adc_max_code` | `int16_t`，本次 ADC 最大码 |
| 116 | 12 | `reserved2` | 12 字节全部为 0 |

### 9.1 STM32 会严格检查的字段

以下任意一项不匹配，STM32 都会判定整帧格式错误、不 ACK 并重新读取：

- magic；
- protocol version；
- frame type；
- header bytes；
- total bytes；
- frame sequence；
- time sample rate；
- time count 范围；
- captured cycles；
- time format；
- FFT sample rate；
- FFT length；
- spectrum count；
- bin spacing；
- spectrum format；
- window type；
- component count；
- 帧尾 CRC。

因此不要把固定字段当作“说明性字段”随意填写。

---

## 10. 每个频率分量的 12 字节结构

每个 `component[i]`：

| 相对偏移 | 长度 | 字段 | 含义 |
|---:|---:|---|---|
| +0 | 4 | `frequency_mHz` | 频率，单位 0.001 Hz |
| +4 | 4 | `amplitude_peak_uV` | 正弦峰值幅度，单位 µV；不是 Vpp、不是 RMS |
| +8 | 2 | `fft_bin` | 对应 4096 点 FFT 谱线序号 |
| +10 | 1 | `harmonic_order` | 1=基波，2=二次谐波，3=三次谐波…… |
| +11 | 1 | `flags` | bit0=有效，bit1=IQ/精测完成，bit2～7=0 |

填写规则：

- 建议按 `harmonic_order` 从小到大排列；
- `component[0]` 应为基波，`harmonic_order=1`；
- `fundamental_mHz` 表示基频，不是“幅度最大的频率”；
- 有效分量的 `flags.bit0` 必须为 1；
- 完成 IQ 或其他高精度频率/幅值估计后置 `flags.bit1=1`；
- 未使用的 component 槽位全部清零；
- `component_count` 是实际分量个数，范围固定为 1～3。

若测量无效：

- 状态置 `RESULT_INVALID=1`；
- 帧结构仍必须完整且 CRC 正确；
- `component_count` 仍填 1，因为 STM32 V1.0 不接受 0；
- `component[0].flags=0`；
- 其他测量值可以清零；
- 所有固定字段仍必须填写正确。

---

## 11. 时域数据定义

位置：

```text
frame[128] 开始
长度 = time_count × 2
格式 = int16_t little-endian
```

数据要求：

- 已经经过 FPGA 数字低通，抑制 ≥1 MHz 干扰；
- 已经去除 ADC 中点偏置；
- 是输入端等效电压，量化单位 10 µV/LSB；
- 有符号，负电压用 int16 二补码；
- 连续覆盖 3 个基波周期；
- 推荐从稳定的同方向过零点或固定相位点开始，减少画面跳动；
- 不传未经处理的 50 MSPS ADC 原始码流。

建议点数：

```text
time_count ≈ round(3 × 12,500,000 / fundamental_Hz)
```

并限制在 1～3750。典型值：

| 基频 | 3 周期点数 |
|---:|---:|
| 10 kHz | 3750 |
| 50 kHz | 750 |
| 100 kHz | 375 |
| 500 kHz | 75 |

STM32 会利用基频和采样率分别重采样出“1 周期”和“3 周期”显示缓存，所以 FPGA 不需要发送两套波形。

---

## 12. 频谱数据定义

位置：

```text
frame[128 + time_count×2] 开始
固定长度 = 1312 × 2
格式 = uint16_t little-endian
```

频率轴：

```text
frequency[k] = k × 381.4697265625 Hz
k = 0 ... 1311
```

含义：

- 4096 点 FFT 的单边幅度谱；
- FFT 输入采样率固定 1.5625 MSPS；
- 使用 Hann 窗；
- 每个点是输入端等效的**正弦峰值幅度**；
- 量化单位为 10 µV_peak/LSB；
- 建议完成 Hann 相干增益和单边谱系数修正；
- 不是复数实部/虚部；
- 不是功率；
- 不是幅度平方 `I²+Q²`；
- STM32 不会再次开平方，只做线性压缩和显示映射。

若内部算法使用功率谱或 `mag²`，必须在 FPGA 内部先换算成线性峰值幅度再写入本协议。

---

## 13. 参数定义

### 13.1 `vpp_uV`

- 数字低通后的输入端时域信号最大值减最小值；
- 单位 µV；
- 不含 ADC 中点偏置。

### 13.2 `vrms_uV`

- 周期信号的真有效值；
- 单位 µV；
- 必须包含基波和谐波的综合贡献；
- 不要用 `Vpp/(2√2)` 代替非正弦信号真有效值。

### 13.3 `dc_offset_uV`

- 被测输入信号自身的直流分量；
- 单位 µV，有符号；
- 不包含 ADC 前端人为加入的中点偏置。

### 13.4 `timestamp_50m`

- FPGA 50 MHz 自由运行计数器；
- 当前帧测量快照对应的时间戳；
- 小端写入 64 位字段；
- 允许自然回绕。

---

## 14. CRC-16/CCITT-FALSE

所有 CRC 统一采用：

| 参数 | 值 |
|---|---|
| Name | CRC-16/CCITT-FALSE |
| Polynomial | `0x1021` |
| Initial | `0xFFFF` |
| RefIn | false |
| RefOut | false |
| XorOut | `0x0000` |
| `"123456789"` 检查值 | `0x29B1` |

注意：

- CRC 算法不反射；
- 但 CRC 字段在 SPI 字节流中仍按协议整数的小端格式发送；
- 例如 CRC 结果为 `0xA709`，线上发送 `09 A7`。

### 14.1 SystemVerilog 参考函数

```systemverilog
function automatic logic [15:0] crc16_ccitt_byte(
    input logic [15:0] crc_in,
    input logic [7:0]  data
);
    logic [15:0] crc;
    int i;
    begin
        crc = crc_in ^ {data, 8'h00};
        for (i = 0; i < 8; i++) begin
            if (crc[15])
                crc = {crc[14:0], 1'b0} ^ 16'h1021;
            else
                crc = {crc[14:0], 1'b0};
        end
        return crc;
    end
endfunction
```

帧生成时：

```systemverilog
crc = 16'hFFFF;
for (每个被覆盖字节)
    crc = crc16_ccitt_byte(crc, byte_data);

frame[total_bytes-2] = crc[7:0];
frame[total_bytes-1] = crc[15:8];
```

---

## 15. FPGA 推荐内部结构

```text
ADC/滤波/FFT/IQ/参数计算
        │
        ▼
测量结果打包器 + 帧CRC
        │ 写
        ▼
双缓冲BRAM
  work buffer / pending buffer
        │ 读
        ▼
SPI从机命令解析器
  GET_STATUS / READ_FRAME / ACK_FRAME
        │
        ├── MISO
        └── DATA_READY
```

### 15.1 缓冲区所有权

至少区分：

- `work_buffer`：计算/打包模块正在写；
- `pending_buffer`：SPI 正在提供给 STM32，完全只读；
- 可选 `next_buffer`：等待当前帧 ACK 后切换。

核心规则：

1. 一帧所有字段和 CRC 写完后才能成为 pending；
2. 切换 pending 时同时锁存 `frame_seq`、`frame_length` 和状态；
3. pending 在正确 ACK 前不可写；
4. 新结果到来但没有空缓冲区时：
   - 可以丢弃新结果；
   - `dropped_frames++`；
   - 设置状态 bit3；
   - 绝不能覆盖 pending。

### 15.2 跨时钟域建议

FPGA 系统时钟为 50 MHz，SPI SCK 为 20 MHz。**不建议只用 50 MHz 对 SCK 做普通双触发器过采样并检测边沿**，因为采样倍率只有 2.5 倍，时序裕量不足。

建议二选一：

1. 使用 SCK/CS 作为 SPI 移位逻辑时钟，字节完成后通过握手同步到 50 MHz 域；
2. 使用 FPGA 厂商 SPI 从机/IP、SERDES 或可靠的异步 FIFO/双口 RAM 结构。

大帧存储建议使用真双口 BRAM：

- 50 MHz 域写完整帧；
- SPI 读侧顺序读取；
- 在缓冲区所有权切换前完成跨时钟域握手。

若 BRAM 是同步读：

- 必须提前预取 `frame[0]`；
- 后续始终预取下一字节；
- 确保 `READ_FRAME` 命令后第一个 dummy 字节立刻输出 `frame[0]`。

---

## 16. SPI 从机状态机建议

事务级状态：

```text
CS上升/复位
    ↓
WAIT_PREFIX
    ├─收到A5 → WAIT_CMD
    └─其他   → IGNORE

WAIT_CMD
    ├─01 → STATUS_STREAM
    ├─02 → FRAME_STREAM
    ├─03 → ACK_RECEIVE
    └─其他 → IGNORE

STATUS_STREAM
    └─依次输出16字节；CS上升结束

FRAME_STREAM
    └─从frame[0]依次输出；CS上升结束

ACK_RECEIVE
    └─收满8字节后校验；CS上升结束

IGNORE
    └─等待CS上升
```

注意：

- 每次 CS 下降时，命令字节计数从 0 开始；
- 每次 CS 上升时，帧读指针重置为 0；
- `FRAME_STREAM` 读完最后一字节也不能自动释放帧；
- 只有正确 ACK 才释放；
- 若 CS 上升发生在半个字节中间，也应放弃该事务；
- ACK 可以在收满第 8 字节时校验，最终缓冲区切换建议与 CS 上升沿配合，避免同一事务内状态突变。

---

## 17. DATA_READY 行为

必须满足：

- 复位后为低；
- 一帧完整打包、CRC 写入并冻结后才能拉高；
- 高电平期间当前帧可重复查询和重复读取；
- 正确 ACK 前保持高；
- STM32 启动前已经为高也没关系，STM32 同时检查上升沿和当前电平；
- 没有下一帧时，接受 ACK 后拉低；
- 有下一帧时，可原子切换并继续保持高电平。

不允许：

- 计算刚开始就提前拉高；
- 读到一半修改帧；
- READ_FRAME 完成后立即拉低；
- 错误 ACK 后拉低；
- 先更新状态序号、后更新帧缓冲区。

---

## 18. 确定性测试向量

### 18.1 CRC 算法自检

```text
输入ASCII: "123456789"
CRC结果:   0x29B1
线上字节:  B1 29
```

### 18.2 ACK 测试向量

帧序号：

```text
frame_seq = 0x01020304
```

CRC 输入：

```text
A5 03 04 03 02 01
```

期望：

```text
CRC结果 = 0xA709
线上完整ACK = A5 03 04 03 02 01 09 A7
```

### 18.3 最大长度状态测试向量

输入状态前 14 字节：

```text
5A A5 01 01 04 03 02 01 0E 28 00 00 00 00
```

解释：

- `FRAME_READY=1`；
- `frame_seq=0x01020304`；
- `frame_length=10254=0x0000280E`。

期望：

```text
CRC结果 = 0x673D
完整状态 = 5A A5 01 01 04 03 02 01 0E 28 00 00 00 00 3D 67
```

### 18.4 最小无效帧联调向量

用于先验证 SPI/CRC/ACK，不要求测量算法已经完成：

```text
frame_seq          = 0x01020304
time_count         = 1
total_bytes        = 2756 = 0x00000AC4
component_count    = 1
其他测量字段       = 0
时域唯一一个样点   = 0
1312个频谱点       = 全0
状态               = FRAME_READY | RESULT_INVALID = 0x11
```

状态前 14 字节：

```text
5A A5 01 11 04 03 02 01 C4 0A 00 00 00 00
```

期望状态 CRC：

```text
0x4605
线上完整状态：
5A A5 01 11 04 03 02 01 C4 0A 00 00 00 00 05 46
```

期望完整帧 CRC：

```text
0x0360
帧尾线上字节 = 60 03
```

该帧应被 STM32 判断为：

- SPI 状态正确；
- 帧格式正确；
- 帧 CRC 正确；
- 结果无效，不发布到显示缓存；
- STM32 仍发送正确 ACK；
- FPGA 收到 ACK 后拉低 `DATA_READY`。

这是一条非常适合第一次硬件联调的闭环用例。

---

## 19. FPGA 仿真必须覆盖的用例

### 19.1 SPI 字节与时序

- Mode 0，MSB first；
- `A5 01` 后第一个 dummy 返回状态 byte0=`5A`；
- `A5 02` 后第一个 dummy 返回帧 byte0=`47`；
- 命令后暂停 SCK，再继续，返回指针不丢失；
- CS 在任意 bit/byte 位置上升，下一事务重新开始；
- CS 高时 MISO 高阻；
- 未知命令不改变任何帧状态。

### 19.2 状态和 CRC

- 状态 CRC 与测试向量一致；
- 帧 CRC 与软件参考模型一致；
- ACK CRC 正确时接受；
- ACK CRC 错、序号错、长度不足时拒绝；
- 所有 CRC 在线字节序均为低字节在前。

### 19.3 缓冲区

- 未 ACK 时重复 GET_STATUS 内容一致；
- 未 ACK 时重复 READ_FRAME 每个字节一致；
- READ_FRAME 中断后重读从帧首开始；
- 新测量完成时不覆盖 pending；
- 正确 ACK 后才释放；
- 有下一帧时原子切换；
- 序号、长度、帧内容不存在交叉混合。

### 19.4 边界长度

- `time_count=1`，帧长 2756；
- `time_count=3750`，帧长 10254；
- 1～3750 中随机点数；
- 最大帧在 20 MHz 下可连续读完。

---

## 20. 实板联调顺序

### 阶段 A：只验证电气和 SPI

FPGA 固定提供第 18.4 节“最小无效帧”：

1. 拉高 `DATA_READY`；
2. STM32 GET_STATUS；
3. STM32 READ_FRAME；
4. STM32 校验成功；
5. STM32 发送 ACK；
6. FPGA 拉低 `DATA_READY`。

预期 STM32：

- status CRC 无错误；
- frame format/CRC 无错误；
- ACK 计数增加；
- result-invalid 计数增加；
- 屏幕仍可显示“等待有效测量”。

### 阶段 B：固定有效合成帧

FPGA 发送一组确定数据：

- 12.5 kHz 基频；
- 3.300 Vpp；
- 1.166 V RMS；
- 时域数组为 3 周期正弦或三角波；
- 频谱在对应 bin 有单个峰；
- `component[0]` 有效；
- 不置 `RESULT_INVALID`。

预期：

- STM32 发布新快照；
- 屏幕参数更新；
- 1 周期显示约 1 个周期；
- 3 周期显示约 3 个周期；
- 频谱图独立显示一个主峰。

### 阶段 C：实时算法

接入 ADC、低通、FFT、IQ 和参数计算，逐项比较：

- 示波器时域波形与 STM32 屏幕波形；
- 信号源频率与 `fundamental_mHz`；
- 信号源/万用表电压与 Vpp、Vrms；
- FFT 峰位置与 component 频率；
- 输入变化后 2 s 内的显示更新；
- 超量程、无效结果和丢帧状态。

---

## 21. 逻辑分析仪验收点

建议同时观察：

- `CS_N`
- `SCK`
- `MOSI`
- `MISO`
- `DATA_READY`

关键判据：

1. SCK 空闲为低；
2. `DATA_READY` 高后 STM32 拉低 CS；
3. GET_STATUS MOSI 为 `A5 01`；
4. 第 3 个 SPI 字节开始，MISO 为 `5A A5 01 ...`；
5. READ_FRAME MOSI 为 `A5 02`；
6. 第 3 个 SPI 字节开始，MISO 为 `47 32 36 46 ...`；
7. ACK 为 8 字节；
8. 正确 ACK 后 `DATA_READY` 才下降。

若 GET_STATUS 的 `5A` 出现在第 4 个而不是第 3 个 SPI 字节，说明 FPGA 多加了一个周转字节，必须修正。

---

## 22. FPGA 交付清单

FPGA 队友交付前请确认：

- [ ] 引脚与第 1 节一致；
- [ ] SPI Mode 0、MSB first、20 MHz 通过时序；
- [ ] 同一 CS 内无额外周转字节；
- [ ] GET_STATUS 固定返回 16 字节；
- [ ] READ_FRAME 第一个返回字节是 `G`；
- [ ] ACK 严格检查 CRC 和序号；
- [ ] 多字节字段全部小端；
- [ ] 状态 CRC、帧 CRC、ACK CRC 均为 CCITT-FALSE；
- [ ] CRC 在线发送低字节在前；
- [ ] 帧头 128 字节偏移全部一致；
- [ ] 时域为 `int16_t`、10 µV/LSB、3 周期；
- [ ] 频谱为线性峰值幅度，不是幅度平方；
- [ ] 频谱固定 1312 点；
- [ ] `component_count` 为 1～3；
- [ ] 无效结果仍生成结构和 CRC 正确的帧；
- [ ] 正确 ACK 前 pending buffer 不可改变；
- [ ] 支持重复读取和中途取消后从头重读；
- [ ] `DATA_READY` 行为符合第 17 节；
- [ ] 通过第 18 节所有测试向量；
- [ ] 仿真覆盖第 19 节用例；
- [ ] 提供一份 ILA/逻辑分析仪波形和测试帧十六进制文件。

---

## 23. 最终一句话合同

> FPGA 在完整测量帧被冻结后拉高 `DATA_READY`；STM32 用同一 CS 内立即返回的 `GET_STATUS` 和 `READ_FRAME` 读取小端、带 CCITT-FALSE CRC 的完整帧；FPGA 在收到序号和 CRC 均正确的 `ACK_FRAME` 前，不得改变或释放当前帧。
