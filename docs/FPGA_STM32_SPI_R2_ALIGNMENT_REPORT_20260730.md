# FPGA—STM32 SPI V1.0 STM32-R2 对齐报告

## 1. 对齐依据

- 队友交付文档：`D:\QQ\FPGA_STM32_SPI_PROTOCOL_V1_0_STM32.md`
- 文档修订：STM32-R2
- 对齐日期：2026-07-30
- STM32 工程：`h743_task2_20260727`
- 传输外设：SPI3，STM32 主机、FPGA 从机

STM32-R2 明确声明与既有 V1.0 二进制线协议兼容，因此本次没有修改
命令值、响应起始位置、字段偏移、端序或 CRC 算法。

## 2. 已逐项一致的线协议

| 项目 | STM32 当前实现 |
|---|---|
| SPI 模式 | Mode 0，8 bit，MSB-first，软件 NSS |
| SCK | PLL2P 80 MHz / 4 = 20 MHz |
| 命令 | `A5 01`、`A5 02`、`A5 03` |
| 响应时序 | 同一 CS；命令阶段 RX 丢弃；第一个 dummy 立即返回 `RESP[0]` |
| GET_STATUS | 16 字节，magic `5A A5`，小端序号/长度，CRC 覆盖前 14 字节 |
| READ_FRAME | 128 字节头 + `time_count*2` + `1312*2` + CRC16 |
| 帧头 | `G26F`、V1.0、固定偏移、16 字节 component |
| ACK | `A5 03 + frame_seq_le + crc_le` |
| CRC | CRC-16/CCITT-FALSE，线上低字节先传 |
| 最大帧 | 10254 字节 |
| DMA/Cache | 32 字节对齐静态缓冲区，DMA 前清理，DMA 后失效 |

READ_FRAME 在线上的完整事务长度始终是 `frame_length+2`。当前 STM32 工程在
同一 CS 低电平内先阻塞发送 `A5 02`，再用 DMA 接收 `frame_length` 字节响应；
RX 缓冲区从下标 0 保存 `frame[0]`。这与 FPGA 看到的一次连续总线事务完全一致，
也符合“命令和响应之间不得抬高 CS”的冻结约定。当前没有修改 `.ioc`。

## 3. 本次补齐的 STM32-R2 行为

1. `FRAME_READY=0` 时允许合法 GET_STATUS 使用 `frame_length=0`，可用于
   DATA_READY 未拉高时的通信自检。
2. 合法状态但 `FRAME_READY=0` 不再统计为协议错误，改记
   `status_not_ready_count`。
3. ACK 发送后不再直接认定 FPGA 已释放帧：
   - 先连续读取 PD1，捕获 FPGA 保证至少 1 us 的低电平；
   - 未立即观察到低电平则进入 `WAIT_DATA_READY_LOW`；
   - 10 ms 内仍为高，增加 `ack_ready_low_timeout_count`；
   - 超时后回到 IDLE，下一轮重新 GET_STATUS。
4. 新增 `ack_ready_low_count`，用于确认 STM32 实际观察到 ACK 后低电平。
5. FPGA 三个分量槽位按独立 VALID 处理：
   - `component_count` 必须等于三个 VALID 位的置位数；
   - 允许 `101`、`010` 等非连续有效排列；
   - 不假定槽位按频率、幅度或谐波次数排序，也不把槽位 0 固定视为基波；
   - 解析层遍历全部三个槽位；
   - 显示转换层只提取 VALID 槽位并压紧到 `t_comp1`～`t_comp3`。
6. `FPGA_BUSY` 仅解释为帧构建/发布管理忙；`fft_delta_q15` 在当前
   RTL V1.0 中按字段解析，但预期值为 0。

## 4. 联调时应观察

正常一帧的诊断顺序：

```text
data_ready_irq_count
status_read_count
status_valid_count
frame_read_count
frame_dma_start_count
frame_dma_complete_count
frame_valid_count
ack_count
ack_ready_low_count
```

下列计数应保持 0：

```text
status_crc_error_count
frame_crc_error_count
frame_dma_timeout_count
frame_dma_error_count
ack_error_count
ack_ready_low_timeout_count
```

## 5. 尚需实板确认

- 逻辑分析仪实测 SCK 为 20 MHz、周期 50 ns、空闲低；
- 每个命令的 CS 从 `A5` 到完整响应连续为低；
- 第三个总线字节开始返回 `5A A5 01`；
- 连续 1000 次 GET_STATUS 的 magic、CRC、HAL 错误均为 0；
- 连续 100 帧 READ_FRAME/ACK 的 CRC、序号、长度和 ACK 超时均为 0；
- ACK 后 DATA_READY 确实拉低，并在下一帧前保持低至少 1 us。

以上硬件项目未经过本次主机测试，必须标注“待 FPGA—STM32 实板验证”。

## 6. 本次软件验证结果

- SPI V1.0/R2 专项合同测试：16/16 通过；
- 实际 C 解析器主机测试：8 种 component VALID 排列全部通过；
- 完整测试集：56 项中 51 项通过，其余 5 项属于当前 FPGA+HMI 路径之外的历史
  ADC/DAC/DDS/TIM 检查；
- STM32 Debug 实际编译、链接成功：
  `text=63192`、`data=472`、`bss=47616`；
- 生成固件：`Debug/h743_task2_20260727.elf`。
