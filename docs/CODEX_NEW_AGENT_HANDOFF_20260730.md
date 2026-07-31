# h743_task2_20260727 新 Agent 完整交接文档

> 更新时间：2026-07-30  
> 适用工程：`C:\Users\48747\STM32CubeIDE\workspace_1.19.0\h743_task2_20260727`  
> 当前分支：`codex/fpga-hmi-bode`  
> 当前基线提交：`180c7bd fix: restore HMI after reconnect and apply calibration`

## 0. 给新 Agent 的第一条指令

请先完整阅读本文件，再检查 Git 状态、当前源码和协议文件。不要从旧工程、旧 UART/Bode 协议或聊天截图反推当前实现。

当前用户最优先的新要求是：

1. **取消时域波形上电隐藏机制。**
2. 上电后 `s_t1`（一周期）和 `s_t3`（三周期）都处于可显示、可接收数据状态。
3. 默认让一周期波形处于前景。
4. 每次接受新的稳定 FPGA 测量快照后，同时更新：
   - 一周期波形；
   - 三周期波形；
   - 频谱；
   - 数字参数。
5. 一周期和三周期按钮只负责切换哪个重叠控件位于前景，不再承担“首次显示/解除隐藏”的职责。
6. `启动`键不能控制或暂停 FPGA SPI，只允许用于串口屏重画/刷新等显示行为。
7. 用户刚刚明确要求：**先写交接文档，不要立即修改上述逻辑。**因此本提交只增加本文件，源码尚未按这项新要求修改。

注意：淘晶驰两个完全重叠的波形控件是否能通过重复 `vis ...,1` 动态改变前后层级，需要实屏验证。如果控件层级固定，则必须向用户说明限制，并在以下方案中选一个：

- 切换时短暂隐藏非选中控件（最可靠）；
- HMI中把两个波形控件分开放置；
- 改成一个共享时域波形控件，STM32按按钮重发对应缓存。

不要未经实测就声称“两控件始终可见且可动态置顶”已经成立。

---

## 1. 项目目标

这是电赛 G 题《周期信号测量分析装置》的 STM32H743 工程。

系统主链路：

```mermaid
flowchart LR
    ADC["ADC/模拟前端"] --> FPGA["FPGA持续采集、FFT和参数计算"]
    FPGA -->|"SPI3测量帧"| STM32["STM32H743"]
    STM32 -->|"USART1 512000"| HMI["淘晶驰串口屏"]
```

职责边界：

- FPGA负责测量和生成完整测量帧。
- STM32负责SPI协议、CRC、缓存、周期提取、三分量频谱生成、校准和HMI驱动。
- 串口屏只负责控件显示和发送按键命令。
- STM32不应因为串口屏按钮而停止或重启FPGA SPI通信。

---

## 2. 当前硬件和 `.ioc`

### 2.1 MCU

- STM32H743
- STM32CubeIDE 1.19.0
- CPU主频当前为480 MHz

### 2.2 FPGA—STM32 SPI3

| 信号 | STM32引脚 | 配置 |
|---|---|---|
| SCK | PC10 | SPI3_SCK |
| MISO | PC11 | SPI3_MISO |
| MOSI | PC12 | SPI3_MOSI |
| CS_N | PA15 | 普通推挽GPIO输出，软件片选 |
| DATA_READY | PD1 | GPIO输入/EXTI1 |

配置：

- SPI主机、Full Duplex；
- Mode 0：CPOL=0、CPHA=0；
- 8 bit；
- MSB first；
- Software NSS；
- NSS pulse关闭；
- 当前SPI3 kernel clock为80 MHz；
- 当前分频128，即 **625 kbit/s**；
- 冻结协议目标速率为20 MHz，但此前因接线联调主动降速；
- 硬件换成杜邦线后通信已成功，旧牛角线/线束曾导致完全无法通信。

不要直接手改CubeMX生成的 `MX_SPI3_Init()`。需要恢复20 MHz时，应在 `.ioc` 中修改并重新生成，同时确认实际SCK波形。

### 2.3 串口屏

| 信号 | STM32引脚 |
|---|---|
| USART1_TX | PA9，接屏幕RX |
| USART1_RX | PA10，接屏幕TX |

- 512000 baud；
- 8N1；
- TX/RX DMA；
- Receive-to-IDLE DMA接收屏幕按键；
- 屏幕与STM32必须共地。

---

## 3. FPGA SPI冻结协议

最高参考：

`docs/FPGA_STM32_SPI_PROTOCOL_V1_0_FROZEN_20260730.md`

不要单方面改变dummy数量、响应偏移、CS边界、端序或CRC范围。

### 3.1 命令

| 命令 | 字节 | 作用 |
|---|---|---|
| GET_STATUS | `A5 01` | 读取状态、帧序号和长度 |
| READ_FRAME | `A5 02` | 读取完整帧 |
| ACK_FRAME | `A5 03 + seq[4] + crc[2]` | 确认并释放当前帧 |

关键规则：

- 命令和响应在同一次CS事务内；
- 命令后第一个dummy字节立即得到 `response[0]`；
- 不增加turnaround dummy；
- 多字节整数小端；
- CRC低字节先传。

### 3.2 CRC

CRC-16/CCITT-FALSE：

```text
Polynomial = 0x1021
Initial    = 0xFFFF
RefIn      = false
RefOut     = false
XorOut     = 0x0000
```

### 3.3 完整帧

```text
128字节头
+ time_count × int16_t 时域数据
+ 1312 × uint16_t 频谱数据
+ CRC16
```

最大帧长10254字节。

组件规则：

- `component[0..2]` 的VALID槽位可以不连续；
- `component_count` 等于三个VALID位的popcount；
- STM32必须遍历全部三个槽位；
- 格式和CRC正确但 `MEASUREMENT_VALID=0` 的帧仍要ACK，只是不发布到屏幕。

ACK成功后等待DATA_READY实际拉低，不依赖固定时钟周期。

---

## 4. 当前软件模块

所有主要用户代码位于 `Core/User/`。

| 模块 | 作用 |
|---|---|
| `fpga_protocol.c/.h` | 字节协议、端序、字段校验、CRC |
| `fpga_link.c/.h` | SPI事务、DMA、重试、ACK、双缓冲、Cache维护 |
| `measurement_conversion.c/.h` | 精确一周期/三周期提取、350点重采样、三分量频谱生成 |
| `measurement_calibration.c/.h` | 电压/频率校准和校准开关 |
| `hmi_chart.c/.h` | 淘晶驰 `vis`、`cle`、`add` 命令构建 |
| `hmi_task2.c/.h` | HMI状态机、按钮、DMA、稳定锁存、断线重放 |
| `system.c/.h` | 总流程编排 |

主循环：

```text
fpga_link_process()
→ 获取最新有效快照
→ measurement_conversion_update()
→ hmi_task2_process()
```

`fpga_link_process()`是无条件持续执行的，不得受HMI启动键控制。

---

## 5. 当前测量和显示算法

### 5.1 时域

- 显示宽度固定350点；
- 根据 `fundamental_mHz` 和时域采样率计算一个周期样本数；
- 搜索稳定的上升过零附近作为周期起点；
- 使用Q16.16线性插值重采样为严格一周期350点；
- 三周期缓存由同一个周期模板重复三次；
- 严格按小端 `int16_t` 二补码解析；协议仍接受完整 `-32768～+32767` 码域。
- 显示纵轴按完整单周期最小值/最大值自动选中心和单边范围，增加约12.5%余量，
  并保留64码最小单边范围；一周期和三周期共用同一比例。

主要实现：

`Core/User/measurement_conversion.c`

2026-07-30实板诊断证据：500.004 kHz帧声明采样率12.5 MHz，STM32计算的一周期
为25点，周期长度计算正确；但FPGA原始 `time_samples[]` 已出现
`min=-8529`、`max=32767`、384/599点接近16位满量程且连续样点重复。FPGA随后确认
V1.0载荷为小端二补码，合法范围是 `-32768～+32767`，约±15000只是常见幅度而非协议边界。
STM32已移除编码自动猜测并按完整int16范围显示；若新FPGA仍大量出现±满量程点，应结合
`ADC_SATURATION`、`last_time_rail_sample_count`继续检查FPGA数字链路是否真实限幅。

### 5.2 频谱

- FPGA仍发送1312个 `uint16_t` 频谱点，但它们只用于协议和饱和诊断；
- 屏幕频谱只取帧头中与文字区相同的最多三个有效 `component[]`；
- `frequency_mhz` 决定横坐标，`amplitude_peak_uv` 按三者最大值归一化决定高度；
- 三个独立分量幅值相同时，屏幕谱线高度必然相同；
- 横轴左右各强制保留8个0%基线点，分量位置限制在第8～341点，避免谱线贴边；
- 当前为了适配淘晶驰滚动方向，存储顺序已反转，使低频在左、高频在右；
- 不再对频谱幅值二次开根号。

公开诊断量 `last_spectrum_rail_bin_count` 直接统计等于65535的饱和bin数量，
`last_component_spectrum_raw[3]` 保存三个有效分量各自FFT bin的原始频谱值。
2026-07-30旧FPGA实板曾观察到 `last_spectrum_max=65535`。2026-07-31新协议把
频谱比例改为125 µV_peak/LSB，理论满量程变为8.191875 V_peak；重新烧写新FPGA后
必须复测饱和计数，不能沿用旧10 µV/LSB结论。

### 5.3 当前启动锁存

- 第一份有效帧上电后立即锁存，并装载参数、一周期、三周期和频谱；
- FPGA与显示换算在后台继续更新，但未按启动键时屏幕工作快照保持不变；
- 启动键锁存当时最新完整快照，不向FPGA发送重测命令；
- 启动帧计作第1帧，800 ms内最多累计5帧小变化数据，只平均文字参数；
- 微调期间一旦出现大变化，丢弃该大变化帧并冻结在此前最新平均值；
- 一周期和三周期控件都已装载，两个周期键只切换重叠控件的前景层级。

---

## 6. 电压校准

模块：

`Core/User/measurement_calibration.c/.h`

当前默认“已校准”。

2026-07-31新FPGA协议已经把Vpp、Vrms、直流和分量峰值定义为物理uV，旧打表的
raw/mV公式不再参与SPI解码。当前只保留用户确认的约2倍前端参考面补偿：

```text
Upp_input_uV  = fpga_vpp_uV / 2.0
Urms_input_uV = fpga_vrms_uV / 2.0
Ui_input_uV   = fpga_peak_uV / 2.0
```

代码中的三个前端增益统一为：

```c
MEASUREMENT_FRONTEND_VPP_GAIN        2.0
MEASUREMENT_FRONTEND_VRMS_GAIN       2.0
MEASUREMENT_FRONTEND_COMPONENT_GAIN  2.0
```

频谱 `Ui` 是正弦峰值幅度，不是Vpp。若FPGA已经补偿同一个2倍模拟增益，STM32
三个增益必须改成1.0，避免重复补偿。

---

## 7. HMI工程和控件

用户确认的最新HMI原文件：

`C:\Users\48747\Downloads\fpga1_codex_ui_v1 (1).HMI`

不要直接覆盖该文件。若需修改：

1. 先复制到项目或新文件名；
2. 保存前后计算哈希；
3. 明确告诉用户烧录的是哪一个HMI。

### 7.1 控件

| 控件 | 用途 |
|---|---|
| `s_t1`，id=3 | 一周期时域波形 |
| `s_t3`，id=4 | 三周期时域波形 |
| `s_spec`，id=5 | 独立频谱 |
| `t_vpp` | 峰峰值 |
| `t_vrms` | 真有效值 |
| `t_freq` | 基频 |
| `t_comp1/2/3` | 三个频率分量 |
| `t_status` | 状态 |
| `t_nihe` | 已校准/未校准 |
| `t_mode` | 当前周期模式 |
| `t_range` | 频率范围 |

`s_t1`与`s_t3`完全重叠，`s_spec`位于右侧独立区域。曲线尺寸为350×210。

### 7.2 按键协议

| 按键 | HMI按下事件 |
|---|---|
| 一周期 | `printh A5 01 5A` |
| 三周期 | `printh A5 02 5A` |
| 频谱历史命令 | `printh A5 03 5A` |
| 启动 | `printh A5 04 5A` |
| 保留模式键 | `printh A5 10 5A`，当前忽略 |
| 校准切换 | `printh A5 20 5A` |

### 7.3 曲线命令

```text
vis s_t1,1     显示一周期控件
vis s_t1,0     隐藏一周期控件
cle s_t1.id,0  清空s_t1的0号通道
add s_t1.id,0,80
```

每条淘晶驰指令后必须带 `FF FF FF`。

当前使用普通 `cle + add`，不要擅自换成 `addt`。

---

## 8. 串口屏断电重连

当前STM32每1秒发送：

```text
sendme
```

屏幕返回：

```text
66 PAGE FF FF FF
```

超过2500 ms没有回复标记为离线。重新收到页面回复后：

- 清除“已经发送”的软件缓存标志；
- 重放文字；
- 重放频谱；
- 重放校准状态；
- 重放应显示的时域波形；
- 不影响FPGA SPI和测量快照。

这部分刚修复，但仍属于**待实板验证**。

---

## 9. 用户最新要求与待实现设计

### 9.1 现有旧逻辑

当前源码中，上电初始化调用：

```c
hmi_chart_build_visibility(HMI_CHART_MODE_SPECTRUM, ...)
```

实际生成：

```text
vis s_t1,0
vis s_t3,0
vis s_spec,1
```

因此仅在HMI编辑器把控件默认设为可见并不能解决问题，STM32上电后仍会主动隐藏。

涉及位置：

- `Core/User/hmi_task2.c` 的 `hmi_task2_build_initialize()`；
- `Core/User/hmi_chart.c` 的 `hmi_chart_build_visibility()`；
- `hmi_task2_display_requested`；
- `hmi_task2_waveform_redraw_pending`；
- `hmi_task2_select_action()`；
- `hmi_task2_refresh_work_snapshot()`。

### 9.2 下一任Agent要实现的新状态机

建议顺序：

```text
上电
→ s_t1、s_t3、s_spec全部可见
→ 默认一周期作为前景
→ 等待第一份有效快照
→ 依次发送文字、一周期、三周期、频谱
→ 恢复一周期为前景
```

新稳定快照：

```text
接受新快照
→ 数字缓存失效
→ s_t1缓存失效
→ s_t3缓存失效
→ s_spec缓存失效
→ 三条曲线各完整发送
→ 最后重新确认用户选中的时域控件处于前景
```

按钮：

```text
A5 01 5A → 一周期前景
A5 02 5A → 三周期前景
A5 04 5A → 可作为强制重画三条曲线；不得影响SPI
A5 20 5A → 切换校准，仅重画数字文字
```

必须防止：

- 新帧到达速度高于UART绘图速度造成旧帧排队；
- 一周期绘制一半时切到三周期导致状态机混乱；
- 每个32点小批次都反复切换层级造成闪烁；
- 为了置顶而清空另一条已缓存曲线；
- HMI按钮影响 `fpga_link_process()`。

建议保持“只保留最新稳定快照”：当前三图发送结束后直接处理最新快照，不排队发送过期快照。

---

## 10. Debug应观察的变量

### 10.1 FPGA SPI

```text
fpga_link_diagnostics.data_ready_irq_count
fpga_link_diagnostics.status_read_count
fpga_link_diagnostics.status_valid_count
fpga_link_diagnostics.status_error_count
fpga_link_diagnostics.status_crc_error_count
fpga_link_diagnostics.frame_read_count
fpga_link_diagnostics.frame_dma_start_count
fpga_link_diagnostics.frame_dma_complete_count
fpga_link_diagnostics.frame_dma_timeout_count
fpga_link_diagnostics.frame_dma_error_count
fpga_link_diagnostics.frame_valid_count
fpga_link_diagnostics.frame_format_error_count
fpga_link_diagnostics.frame_crc_error_count
fpga_link_diagnostics.ack_count
fpga_link_diagnostics.last_frame_sequence
fpga_link_diagnostics.last_frame_length
fpga_link_diagnostics.last_protocol_result
fpga_link_diagnostics.state
```

### 10.2 数据转换

```text
measurement_conversion_diagnostics.conversion_count
measurement_conversion_diagnostics.invalid_source_count
measurement_conversion_diagnostics.last_frame_sequence
measurement_conversion_diagnostics.last_time_min
measurement_conversion_diagnostics.last_time_max
measurement_conversion_diagnostics.last_spectrum_max
measurement_conversion_diagnostics.last_one_cycle_samples
```

### 10.3 HMI

```text
hmi_task2_diagnostics.rx_event_count
hmi_task2_diagnostics.rx_byte_count
hmi_task2_diagnostics.command_count
hmi_task2_diagnostics.invalid_command_count
hmi_task2_diagnostics.tx_start_count
hmi_task2_diagnostics.tx_complete_count
hmi_task2_diagnostics.tx_error_count
hmi_task2_diagnostics.chart_chunk_count
hmi_task2_diagnostics.chart_pass_count
hmi_task2_diagnostics.stable_accept_count
hmi_task2_diagnostics.stable_reject_count
hmi_task2_diagnostics.probe_count
hmi_task2_diagnostics.probe_reply_count
hmi_task2_diagnostics.reconnect_count
hmi_task2_diagnostics.screen_online
hmi_task2_diagnostics.current_page
hmi_task2_diagnostics.last_source_sequence
hmi_task2_diagnostics.last_visible_sequence
hmi_task2_diagnostics.requested_mode
hmi_task2_diagnostics.visible_mode
hmi_task2_diagnostics.state
```

### 10.4 校准

```text
measurement_calibration_enabled
measurement_calibration_diagnostics.apply_count
measurement_calibration_diagnostics.toggle_count
measurement_calibration_diagnostics.enabled
```

不要继续监视旧协议中的 `fpga_link_bode`、`mag2_hi`、`phase` 等变量，它们已不存在，CubeIDE会显示MI错误。

---

## 11. 常见故障树

### 数字正常、图表不显示

依次检查：

1. `measurement_conversion_diagnostics.conversion_count` 是否增加；
2. `last_time_min != last_time_max`；
3. `last_spectrum_max` 是否非零；
4. `hmi_task2_diagnostics.chart_chunk_count` 是否增加；
5. `chart_pass_count` 是否完成；
6. `tx_error_count` 是否为0；
7. HMI控件名是否严格为 `s_t1/s_t3/s_spec`；
8. 控件ID是否仍为3/4/5；
9. UART线上是否存在 `cle` 和 `add` 指令；
10. 屏幕是否下载了正确的最新HMI。

### 屏幕断电重插后不恢复

检查：

1. 屏幕TX是否接PA10；
2. `probe_count`是否增加；
3. `probe_reply_count`是否增加；
4. `screen_online`是否重新变为1；
5. `reconnect_count`是否增加；
6. 重连后 `tx_start_count/chart_pass_count` 是否继续增加。

### SPI没有数据

检查：

1. PD1 DATA_READY电平；
2. EXTI计数；
3. PA15是否拉低；
4. PC10是否有SCK；
5. MOSI是否为 `A5 01`；
6. MISO第一个响应字节是否紧跟命令；
7. 接线和共地；
8. 烧录目标是否为当前工程ELF。

### CRC错误

优先检查：

- CS是否在命令和DMA之间出现高脉冲；
- 是否错误增加dummy；
- MISO/MOSI是否接反；
- DMA RX地址递增；
- H7 D-Cache失效范围是否32字节对齐；
- DMA缓冲区是否错误放入DTCM。

---

## 12. 测试和构建状态

最近针对HMI重连和校准的合同测试：

```powershell
python -m unittest `
  tests.test_hmi_runtime_contract `
  tests.test_measurement_calibration_contract `
  tests.test_fpga_hmi_bode_contract
```

最近结果：42项通过。

完整测试曾为74项，其中69项通过，5项失败来自与当前FPGA/HMI主链路无关的历史片上ADC/DDS/DAC/TIM预期。不要把这5项误判为SPI/HMI回归。

最近Debug工程已成功生成：

`Debug\h743_task2_20260727.elf`

但本文件加入后未重新编译，因为只改了文档。

烧录前必须确认CubeIDE实际构建目标是：

```text
h743_task2_20260727.elf
```

不要误烧旧的 `h743_pre1.elf`。

---

## 13. Git和文件保护

当前工作树包含大量用户/IDE/Debug未提交改动。

新 Agent 开始前必须执行：

```powershell
git status --short
git diff --stat
git log -5 --oneline
```

规则：

- 不得 `git reset --hard`；
- 不得回滚用户已有改动；
- 不得把大量Debug产物与功能代码混在一个提交；
- 不得覆盖最新HMI原文件；
- 不得force push；
- 未经用户明确要求不得push；
- 每次只暂存本任务实际修改的文件。

当前分支：

```text
codex/fpga-hmi-bode
```

当前基线：

```text
180c7bd fix: restore HMI after reconnect and apply calibration
```

---

## 14. 建议的接手执行顺序

1. 阅读本文件和冻结协议。
2. 检查Git状态，确认没有覆盖用户改动。
3. 阅读 `system.c`、`fpga_link.c`、`measurement_conversion.c`、`hmi_chart.c`、`hmi_task2.c`。
4. 先为“两个时域缓存始终更新、按钮只切前景”修改合同测试。
5. 再修改HMI状态机，不触碰FPGA SPI协议。
6. 运行HMI/转换/协议测试。
7. 完整构建正确工程。
8. 烧录STM32和正确HMI。
9. 实测：
   - 上电默认一周期前景；
   - 一周期、三周期、频谱都有数据；
   - 按钮切换不清空缓存；
   - 输入变化后三图更新；
   - 屏幕断电重插后自动恢复；
   - FPGA SPI计数持续增长；
   - HMI启动键不影响SPI计数。
10. 只有实板通过后，才考虑把SPI3从625 kHz恢复到20 MHz并做压力测试。

---

## 15. 交付时必须如实区分

- “代码已实现”不等于“工程已编译”；
- “合同测试通过”不等于“实板验证通过”；
- “SPI能通信”不等于“20 MHz压力测试通过”；
- “控件收到数据”不等于“两个重叠控件可以动态置顶”；
- “参数显示正确”不等于“FPGA字段单位已永久冻结”。

下一任Agent应在每次汇报中分别写明：

1. 修改了什么；
2. 测试结果；
3. 编译结果；
4. 尚未完成的实板验证；
5. 下一步让用户观察哪些诊断变量或波形。
