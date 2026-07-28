# FPGA ↔ STM32 幅频/相频 UART 双向通信指导

本文给 FPGA 与 STM32 联调使用。FPGA 向 STM32 发送固定幅相数据帧，STM32
向 FPGA 发送单字节扫频步进命令。不要自行增加版本号、CRC、功率、频率起点
等字段，否则 STM32 会按错误位置解析。

## 1. 硬件和串口参数

| 项目 | 要求 |
|---|---|
| FPGA J15/TX | 接 STM32 `PA3/USART2_RX` |
| FPGA T19/RX | 接 STM32 `PA2/USART2_TX`，接收扫频步进命令 |
| 电平 | 3.3 V TTL UART，不是 RS-232 电平 |
| 地 | FPGA 与 STM32 必须共地 |
| 波特率 | `1,000,000 bit/s` |
| 格式 | 8 data bits、No parity、1 stop bit，MSB/LSB 仅按下面字段定义 |
| 空闲电平 | TX 保持高电平 |

FPGA 复位释放后建议等待至少 `100 ms` 再发送第一帧，避免 STM32 尚未完成
USART2 DMA 初始化。

## 2. 固定帧格式

```text
AA 55  N_H N_L  [MAG_H MAG_L PHASE_H PHASE_L] × N  0D 0A
```

| 偏移 | 长度 | 内容 |
|---:|---:|---|
| 0 | 1 | 帧头 `0xAA` |
| 1 | 1 | 帧头 `0x55` |
| 2 | 1 | 点数 N 的高字节 |
| 3 | 1 | 点数 N 的低字节 |
| 4 | `4×N` | N 组幅度平方高16位和相位 |
| `4+4N` | 1 | 帧尾 `0x0D` |
| `5+4N` | 1 | 帧尾 `0x0A` |

要求：

- `1 ≤ N ≤ 1024`；N 使用本次实际扫频点数，例如 `N=249`。
- 总帧长必须严格等于 `4 + 4×N + 2` 字节。
- 所有 16 位字段均为大端发送，即高字节先发。
- `0x0D 0x0A` 是两个二进制字节，不是字符 `'0' 'D' '0' 'A'`。
- 一帧内部必须连续发送，不能出现超过约 `10 µs` 的额外空闲。
- 两帧不能粘在一起发送。建议完整帧发送完后保持 TX 高电平至少
  `50 ms`；联调初期建议每 `500 ms` 发送一帧。

总长度为 `4N+6` 字节。在 1 Mbaud、8N1 下，`N=249` 时为 `1002`
字节、约 `10.02 ms`；`N=1024` 时为 `4102` 字节、约 `41.02 ms`。

## 3. 每个频点的数据

每个频率点严格占 4 字节：

```text
MAG_H MAG_L PHASE_H PHASE_L
```

### 3.1 幅度 `mag2_hi`

- 类型：`uint16_t`，范围 `0…65535`。
- 精确定义：`mag2_hi = (I² + Q²)[63:48]`，即64位幅度平方的高16位。
- STM32 对每组 `mag2_hi` 求均值后执行整数平方根，再按本帧幅值最小值到
  最大值自动映射到 `0…255`；最小值位于 s0 纵轴底部。
- FPGA 不要再对该字段开方或使用逐频点变化的缩放系数。
- 建议整次扫频以及相邻帧使用相同量程，否则曲线高度会跳变。

例如：

```verilog
mag2_hi = mag2[63:48];
```

### 3.2 相位 `phase`

- 类型：`int16_t` 二进制补码。
- 格式：Q1.15 对应 `-π…+π`。
- STM32 的解释公式：

```text
phase_rad = phase / 32768 × π
phase_deg = phase / 32768 × 180°
```

典型值：

| 相位 | int16 数值 | 发送字节 |
|---:|---:|---|
| `-180°` | `-32768` | `80 00` |
| `-90°` | `-16384` | `C0 00` |
| `0°` | `0` | `00 00` |
| `+90°` | `16384` | `40 00` |
| 接近 `+180°` | `32767` | `7F FF` |

如果 FPGA 内部使用角度，可按下式量化并限制到 int16 范围：

```text
phase_q15 = round(phase_deg × 32768 / 180)
```

幅度和相位必须来自同一个频率索引。STM32 降采样时会分别计算每组频点的
幅度算术平均值和有符号相位算术平均值。

## 4. 频率点顺序

- `i=0…N-1` 必须按频率从低到高发送。
- 当前帧里没有 `f_start`、`f_step` 或频率单位字段。
- 串口屏横轴是静态绘制的，因此 FPGA 的扫频范围必须与屏幕标签保持一致。
- 目前交接约定使用：

```text
freq[i] = (f_start + i × f_step) × 762.94 Hz
```

如果 FPGA 修改 `f_start`、`f_step` 或扫频范围，必须同步修改串口屏横轴；
否则曲线形状能显示，但横轴标注会错误。

STM32 最终向每张串口屏图发送256点。N大于256时按比例分组并计算幅度平方
和相位均值；N小于256时按频率顺序等比例重复相邻点以铺满横轴。因此
`N=249` 可以直接接收并铺满256点宽的 Waveform。

## 5. 发送状态机参考

下面只描述帧组织逻辑。UART TX 分频和移位模块应使用 FPGA 工程现有实现，
并配置为 1 Mbaud；不要直接照搬未知系统时钟的分频常数。

```text
IDLE:
    等待一帧扫频数据准备完成
    锁存 N，并冻结本次发送使用的数据缓冲区
    byte_index = 0
    进入 SEND

SEND:
    当 uart_tx_ready == 1 时，根据 byte_index 选择 tx_byte：

    byte_index == 0             -> 8'hAA
    byte_index == 1             -> 8'h55
    byte_index == 2             -> N[15:8]
    byte_index == 3             -> N[7:0]

    4 <= byte_index < 4+4*N:
        point_index = (byte_index - 4) / 4
        field_index = (byte_index - 4) % 4

        field_index == 0 -> magnitude[point_index][15:8]
        field_index == 1 -> magnitude[point_index][7:0]
        field_index == 2 -> phase[point_index][15:8]
        field_index == 3 -> phase[point_index][7:0]

    byte_index == 4+4*N         -> 8'h0D
    byte_index == 5+4*N         -> 8'h0A

    对 tx_byte 产生一次 uart_tx_start
    最后一个字节完成后进入 GAP，否则 byte_index++

GAP:
    TX 保持高电平
    等待至少 50 ms，建议500 ms周期到达后回到 IDLE
```

实现注意：

- `uart_tx_busy=1` 时不能重复触发 `uart_tx_start`。
- 发送过程中不能修改 N 或当前发送缓冲区，建议使用双缓冲。
- 不要在一帧中途等待下一次 FFT；必须先准备完整快照再开始发送。
- 帧内不能插入调试字符串、换行或其他 UART 数据。
- 当前没有硬件流控和 ACK；FPGA按固定周期发送，STM32通过诊断计数判断结果。

## 6. 最小测试帧

下面是 `N=2` 的完整测试帧：

```text
AA 55 00 02  40 00 00 00  FF FF C0 00  0D 0A
```

含义：

- 点0：`mag2_hi=0x4000`，相位 `0°`。
- 点1：`mag2_hi=0xFFFF`，相位 `-90°`。

联调建议先发送一帧固定斜坡/峰形数据；STM32确认 `last_point_count` 等于
FPGA实际N、`frame_valid_count` 增加后，再切换到真实扫频数据。

## 7. STM32 侧验收计数

在 STM32CubeIDE Expressions 中观察：

```c
fpga_link_diagnostics.rx_event_count
fpga_link_diagnostics.frame_found_count
fpga_link_diagnostics.frame_valid_count
fpga_link_diagnostics.frame_error_count
fpga_link_diagnostics.uart_error_count
fpga_link_diagnostics.last_frame_bytes
fpga_link_diagnostics.last_point_count
hmi_task2_diagnostics.bode_frame_count
```

以当前 `N=249` 为例，正确帧应满足：

```text
last_frame_bytes  = 1002
last_point_count  = 249
frame_valid_count 增加1
frame_error_count = 0
uart_error_count  = 0
```

## 8. STM32 → FPGA 扫频步进命令

STM32 通过 `PA2/USART2_TX` 向 FPGA `T19/RX` 发送单字节命令：

| STM32 字节 | ASCII | FPGA 动作 | STM32 接口 |
|---:|---:|---|---|
| `0x2B` | `+` | `f_step + 1` | `fpga_link_send_step_increase()` |
| `0x2D` | `-` | `f_step - 1` | `fpga_link_send_step_decrease()` |

命令没有帧头、帧尾或 ACK。USART2 是全双工接口，STM32发送单字节命令时
RX DMA可以继续接收FPGA数据帧。可在调试器中观察：

```c
fpga_link_diagnostics.command_tx_count
fpga_link_diagnostics.command_tx_error_count
fpga_link_diagnostics.last_tx_command
```

## 9. 当前协议没有包含的内容

- 功率值不在该帧中。STM32当前通过 `measurement_result_set_power_w()` 接口
  单独写入功率。
- 当前没有 CRC、序号、时间戳、频率起点和频率步进。
- 如果以后需要 FPGA 同时发送功率或动态频率轴，必须先升级 STM32 协议解析，
  不能直接在现有帧中追加字段。
