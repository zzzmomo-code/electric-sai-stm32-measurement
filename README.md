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

2026-07-18 实测确认：方波输入、TIM5 粗测频、频率规划、SPI 通信和 AD9834 正弦输出均正常。

2026-07-21 实测确认：淘晶驰串口屏通过 USART1 正常收发，十项数据显示、G0～G5 六档按键和立即测量按键均能被 MCU 正确处理；`tx_count`、`rx_count`、`command_count` 会随实际通信更新，通信错误计数保持为 0。当前代码默认运行正式跟随模式，是项目现阶段的稳定基线。

开发环境：STM32CubeIDE 1.19.0、STM32Cube FW_H7 V1.12.1。

## 硬件

- MCU：STM32H743VIT6。
- DDS：AD9834，外部 MCLK 为 75 MHz。
- PA0：TIM5_CH1，接过零比较器输出的 0～3.3 V 方波。
- PA6：ADC1_INP3，作为双 ADC 同步采集的 CH1 模拟输入。
- PB1：ADC2_INP5，作为双 ADC 同步采集的 CH2 模拟输入。
- PB12：AD9834 FSYNC，空闲为高电平。
- PB13：SPI2_SCK，连接 AD9834 SCLK。
- PB15：SPI2_MOSI，连接 AD9834 SDATA。
- PB14：AD9834 FSELECT，低电平选择 FREQ0，高电平选择 FREQ1。
- PD8：AD9834 PSELECT，低电平选择 PHASE0，高电平选择 PHASE1。
- PC4: DAC1输出直流信号
- MCU、比较器和 AD9834 必须共地。

AD9834 初始化使用控制寄存器的 RESET 位完成软件复位，不需要 MCU 单独控制硬件 RESET 引脚。硬件 RESET、SLEEP 等未由本工程控制的引脚应按实际模块原理图固定到有效电平，不得悬空。

## CubeMX 配置

### ADC1 / ADC2 双路同步采集

1. 打开工程根目录的 `h743_pre1.ioc`，进入 `Pinout & Configuration`。
2. 在芯片引脚图中单击 PA6，选择 `ADC1_INP3`；在 ADC1 通道设置中确认显示 `IN3 Single-ended`。PC4 不再分配给 ADC1。
3. 打开 `Analog > ADC1 > Parameter Settings`：
   - Resolution：`16 Bits`；
   - Scan Conversion Mode：Disabled；
   - Regular Conversion 数量：`1`；
   - Rank 1 Channel：`ADC_CHANNEL_3`；
   - Sampling Time：`8.5 Cycles`；
   - External Trigger Conversion Source：`Timer 2 Trigger Out event`；
   - External Trigger Conversion Edge：`Rising Edge`；
   - Conversion Data Management Mode：`DMA Circular Mode`；
   - Overrun：`Overwritten`。
4. 在 ADC1 的 Multi-mode 设置中选择 `Dual regular simultaneous mode`。ADC1 为主 ADC；ADC2 保持 PB1、`ADC2_INP5`、Rank 1、单端输入和 `8.5 Cycles`。
5. 打开 `Analog > ADC1 > DMA Settings`，确认：
   - DMA：`DMA1 Stream 0`，Request 为 `ADC1`；
   - Direction：Peripheral to Memory；
   - Peripheral/Memory Increment：Disable/Enable；
   - Peripheral/Memory Data Width：Word/Word；
   - Mode：Circular；Priority：Very High；FIFO：Disabled。
6. 打开 `System Core > NVIC`，保留 `DMA1 Stream0 global interrupt` 和共享 `ADC1 and ADC2 global interrupt`，抢占优先级均为 5。换脚不需要新增 DMA 或中断。
7. 打开 `Clock Configuration`，确认 ADC 内核时钟仍为 64 MHz，ADC1/ADC2 Clock Prescaler 均为 Asynchronous clock divided by 2。PA6 改为模拟模式后不使用 GPIO 上拉、下拉、输出速度或复用功能。
8. 打开 `Project Manager > Code Generator`，勾选 `Keep User Code when re-generating`，按 `Alt+K` 或点击工具栏 `GENERATE CODE` 重新生成。生成后检查 `Core/Src/adc.c` 包含 `ADC_CHANNEL_3`、GPIOA Pin 6 初始化及反初始化。

该迁移只改变 ADC1 的物理输入和规则通道；TIM2 TRGO、ADC2、DMA 数据格式、NVIC 优先级和 FFT 数据顺序均保持不变。

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

### USART1 / 淘晶驰串口屏

- `PA9 -> USART1_TX`，连接串口屏 RX。
- `PA10 -> USART1_RX`，连接串口屏 TX。
- Asynchronous，`9600 bit/s`，8 data bits，No parity，1 stop bit。
- 发送使用 HAL 轮询，不使用 DMA；接收使用单字节中断。
- 启用 `USART1 global interrupt`，抢占优先级为 7，子优先级为 0。

串口屏页面需要以下文本控件，控件名必须完全一致：

| 控件名 | 显示内容 |
|---|---|
| `t_timer_freq` | TIM5 粗测输入频率 |
| `t_adc_freq` | ADC/FFT 测得的中频频率 |
| `t_adc_amp` | ADC/FFT 测得的 CH1 峰峰值 |
| `t_real_freq` | 换算后的被测信号频率 |
| `t_real_amp` | 换算后的被测信号幅度 |
| `t_wave` | `SINE`、`SQUARE` 或 `UNKNOWN` |
| `t_dds_freq` | AD9834 当前输出频率 |
| `t_vga` | 当前 VGA 档位 `G0` 至 `G5` |
| `t_status` | 测量状态 |
| `t_overflow` | `adc_dual_stats.overflow_count` |

VGA 六个按键的按下或松开事件分别发送一个 ASCII 字节：

```text
printh 30
printh 31
printh 32
printh 33
printh 34
printh 35
```

立即测量按键的松开事件发送：

```text
printh 4D
```

其中 `30` 至 `35` 对应字符 `'0'` 至 `'5'`，`4D` 对应字符 `'M'`。按键事件不要再附加控件 ID 或页面 ID，否则 MCU 会把多余字节计为无效命令。

状态含义：

- `WAIT`：还没有可显示的测量数据。
- `EST`：已有部分数据，但结果仍是估算值。
- `LOCK`：ADC 中频位于 90 kHz 至 110 kHz，认为已锁定在目标中频附近。
- `CLIP`：CH1 FFT 诊断发现削顶。
- `ERROR`：ADC 或 DDS 控制模块报告错误。

数值前的 `~` 表示当前值仍是估算值；`--` 表示暂时没有可用数据。串口屏输入处理、VGA 切档和 TIM5 强制测量都在主循环执行，USART1 回调只设置接收标志。

CubeMX 重新生成代码前启用 `Project Manager > Code Generator > Keep User Code when re-generating`。不要手工修改自动生成的 `MX_*_Init()`，用户模块统一放在 `Core/User`。

## 软件流程

TIM2 以 600 kHz TRGO 同时触发 ADC1 的 PA6/INP3 和 ADC2 的 PB1/INP5。ADC1 继续作为双模式主 ADC，DMA1 Stream0 循环读取 32 位公共数据：低 16 位是 ADC1/CH1，高 16 位是 ADC2/CH2。DMA 半满、满和 ADC 错误回调只设置标志，拆包、统计及 FFT 输入均由主循环处理。

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

`hmi_task2.c` 是本题的活动串口屏模块，每 500 ms 读取 TIM5、DDS、ADC/FFT、VGA 和溢出计数的快照并刷新页面。旧 `hmi_tjc.c` 仅作上一训练题留档，整个实现已用条件编译关闭。当前换算接口位于 `measurement_conversion.c`：被测频率优先按 `DDS频率 + ADC中频` 计算，ADC 中频缺失时暂用 TIM5 粗测值；被测幅度暂时直接使用 ADC 峰峰值。整机幅频标定完成后，只需在该模块中替换拟合公式，不需要修改显示协议。

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
- `Core/User/measurement_conversion.c`：被测频率和幅度的标定换算接口。
- `Core/User/hmi_task2.c`：本题串口屏十项数据显示及单字节按键处理。
- `Core/User/hmi_tjc.c`：上一训练题留档，当前不参与编译。
- `tests/test_dds_contract.py`：检查 IOC、SPI 时序约定、频率公式和模块调用关系。
- `tests/test_hmi_runtime_contract.py`：检查串口屏控件、按键命令、中断边界和换算接口。

`main.c` 用户初始化区只调用 `system_init()`，主循环只调用 `system_process()`。

## 编译与烧录

1. 在 STM32CubeIDE 1.19.0 中打开 `h743_pre1`。
2. 选择 Debug 配置，执行 `Project > Clean...`。
3. 执行 `Project > Build Project`，确认 0 errors、0 warnings。
4. 使用 ST-LINK 下载并运行。
5. 将过零比较器输出接到 PA0，示波器连接 AD9834 输出端并共地。
6. 改变输入方波频率，检查 DDS 输出是否满足 `fDDS = fin - 100 kHz`。
7. 向 PA6 输入位于 `VSSA`～`VDDA` 范围内且与 MCU 共地的模拟信号，在 Expressions 中观察 `adc_dual_stats.ch1_recent_min_code`、`ch1_recent_max_code` 和 `ch1_recent_mean_code` 随信号变化。
8. 同时向 PB1 输入测试信号，确认 `dma_half_count`、`dma_full_count` 持续增加、`error_count` 保持 0，并验证 CH1/CH2 FFT 结果对应同一 TIM2 触发时刻。

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
- PA6 和 PB1 的 ADC 模拟输入必须保持在 `VSSA`～`VDDA` 允许范围内；高源阻抗信号需要缓冲或增大采样时间，当前 `8.5 Cycles` 配置应结合模拟前端驱动能力进行实板验证。
- 当前频率规划采用低侧本振，输入有效范围为 1～30 MHz，目标中频固定为 100 kHz。
- 频率绝对精度受输入边沿质量、H743 系统时钟和 AD9834 75 MHz 参考时钟误差影响。
- DDS 控制全流程和串口屏双向通信已经通过实板测试；AD835 混频、中频滤波、VGA 自动增益、片上 ADC 幅度精度和整机校准仍需后续联调。
- 串口屏实板测试采用 PA9/PA10 交叉接线，页面控件刷新、G0～G5 六档命令和立即测量命令均已验证。屏幕工程修改控件名或按键发送字节后，必须同步更新 `hmi_task2.c` 中的协议常量。
- `measurement_conversion.c` 中的幅度换算仍是占位实现；整机增益及频率响应拟合完成后再替换为校准公式。
- 仓库中保留了前一训练题的双 ADC、FFT 和串口屏模块，当前 DDS 链路不以这些模块的测量结果作为验收依据。
## DAC 与 VGA 增益控制（2026-07-19）

本工程使用 STM32H743VIT6 的 DAC1 Channel 1 产生 VGA 控制电压。DAC1_OUT1
通过片内模拟连接进入 OPAMP1，OPAMP1 工作于电压跟随器模式，最终从
`PC4/OPAMP1_VOUT` 输出。开发环境为 STM32CubeIDE 1.19.0；系统使用 25 MHz
外部晶振，PLL1 配置保持 480 MHz CPU 主频。

CubeIDE/CubeMX 配置入口为 `Pinout & Configuration > Analog`：

1. 在 `DAC1` 中启用 `OUT1 connected to on chip peripherals only`，保持无触发、
   Sample and Hold Disabled、Connect to external peripheral Enabled。
2. 在 `OPAMP1` 中选择 `Follower-DAC_OUT1-INP`，Power Mode 使用 Normal，
   Trimming 使用 Factory。
3. 在 Pinout 中确认 PC4 为 `OPAMP1_VOUT`，GPIO 模式为 Analog、No pull。
4. 在 `Clock Configuration` 中确认 HSE 为 25 MHz，PLL1 参数为 M=5、N=192、
   P=2，SYSCLK 为 480 MHz，HCLK 为 240 MHz。
5. 在 `Project Manager > Code Generator` 勾选 `Keep User Code when re-generating`，
   使用 `Alt+K` 或工具栏 `GENERATE CODE` 重新生成。生成后不要手工修改
   `MX_DAC1_Init()`、`MX_OPAMP1_Init()` 或 `SystemClock_Config()`。

用户驱动位于 `Core/User/dac_output.c` 和 `Core/User/dac_output.h`，由
`system_init()` 自动启动 OPAMP1、设置安全的 0 档并启动 DAC1。六档标称输出为：

| 档位 | DAC 标称电压/V | 默认 VG | 默认 VGA 差分增益 |
|---:|---:|---:|---:|
| 0 | 0.00 | -1.0 | 0.0 |
| 1 | 0.66 | -0.6 | 0.4 |
| 2 | 1.32 | -0.2 | 0.8 |
| 3 | 1.98 | 0.2 | 1.2 |
| 4 | 2.64 | 0.6 | 1.6 |
| 5 | 3.30 | 1.0 | 2.0 |

外部电路模型为：

```text
VG = (20 / 33) * VDAC - 1
VOUT = ((+VIN) - (-VIN)) * (1 + VG) * RF / RG
gain = (1 + VG) * RF / RG
```

`RF` 与 `RG` 默认均为 10 kohm。可在 `dac_output.h` 中修改以下宏：

```c
#define DAC_OUTPUT_RF_OHM                   10000.0f
#define DAC_OUTPUT_RG_OHM                   10000.0f
#define DAC_OUTPUT_GAIN_CALIBRATION         1.0f
#define DAC_OUTPUT_OFFSET_CALIBRATION_V     0.0f
```

设置和查询示例：

```c
float gain;

if (dac_output_set_level(3u) == dac_output_status_ok)
{
    (void)dac_output_get_vga_gain(3u, &gain);
}
```

档位超出 `0..5`、查询输出指针为空或 HAL 操作失败时，函数返回错误；设置失败
不会更新软件记录的当前档位。校准关系为
`VDAC=标称电压*DAC_OUTPUT_GAIN_CALIBRATION+DAC_OUTPUT_OFFSET_CALIBRATION_V`，
结果在换算 12 位 DAC 码值前限制到 0 V 至参考电压。

烧录后应使用高输入阻抗万用表或示波器测量 PC4 六档电压，再调整比例和偏移
校准宏。DAC 参考电压取决于实际 VDDA，片内 OPAMP 的输出摆幅和外部负载也会
影响结果，因此 3.30 V 档不保证在真实硬件上精确达到 3.300 V。当前只完成理论
换算、静态契约和目标构建验证，板上六档电压与外部 VGA 增益仍需实测确认。

---
