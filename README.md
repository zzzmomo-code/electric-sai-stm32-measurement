# phase_locking_codex 使用指南

## 1. 项目目标

本工程使用 STM32H743VIT6，以同一个 TIM2 更新事件同时触发 ADC1 和 DAC1：

- PC0 采集 0～3.3 V 单极性输入；
- PA4 输出 0～3.3 V 单极性信号；
- 采样率和输出更新率均为 1 MHz；
- 支持“完整波形同步还原”和“正弦信号数字锁相”两种模式。

两种模式解决的问题不同：

- **直接模式**：保留输入波形的直流、幅度和谐波，适合任意带限波形；输出相对输入有固定缓冲延迟，因此相位不会持续漂移，但不保证零相位差。
- **DPLL 模式**：提取输入正弦基波的频率、幅度、偏置和相位，生成相位可调的干净正弦波；适合要求示波器上相位稳定的正弦信号。

任意复杂波形的“一比一还原”和“所有频率分量都独立校成零相位”不能同时完成，所以工程明确分成以上两种模式。

## 2. 硬件连接

| 引脚 | 外设功能 | 连接 |
| --- | --- | --- |
| PC0 | ADC1_INP10 | 函数发生器输出，必须为 0～3.3 V |
| PA4 | DAC1_OUT1 | 示波器 CH1 |
| PB14 | USART1_TX | USB-TTL 的 RX |
| PB15 | USART1_RX | USB-TTL 的 TX |
| PA13/PA14 | SWDIO/SWCLK | ST-Link |

注意：

- 函数发生器、开发板、示波器和 USB-TTL 必须共地。
- PC0 不允许输入负电压或高于 VDDA 的电压；双极性信号需要先经过偏置和限幅电路。
- PA4 的片内 DAC 带载能力有限，接示波器时建议使用高阻输入；需要带负载时应增加运放缓冲。

## 3. 已使用的 CubeMX 配置

当前 `.ioc` 已配置并通过编译：

- 系统时钟 480 MHz，HCLK 240 MHz；
- TIM2：240 MHz / 240 = 1 MHz，TRGO 为 Update Event；
- ADC1：16 位、PC0/INP10、16.5 cycles、外部触发 TIM2_TRGO、DMA Circular、Overrun 覆盖旧数据；
- DAC1 CH1：PA4、TIM2_TRGO 触发、12 位右对齐、输出缓冲开启；
- ADC DMA：DMA1 Stream0，Half Word，Circular；
- DAC DMA：DMA1 Stream1，Half Word，Circular；
- USART1：PB14/PB15，115200、8-N-1；
- CPU I-Cache、D-Cache 开启。

以后从 CubeMX 重新生成代码时：

1. 保留 `Core/User` 目录；
2. 确认 `main.c` 用户初始化区仍只有 `system_init();`；
3. 确认 `while (1)` 用户区仍只有 `system_process();`；
4. 确认 `.cproject` 仍包含 `../Core/User` 头文件路径；
5. 不要删除两个链接脚本中的 `.dma_buffer` 段。

## 4. 编译、烧录和运行

1. 用 STM32CubeIDE 1.19.0 打开工程：

   `C:\Users\48747\STM32CubeIDE\workspace_1.19.0\phase_locking_codex`

2. 选择 `Debug` 配置并编译。
3. 用 ST-Link 烧录并复位。
4. 函数发生器接 PC0，示波器同时观察输入和 PA4 输出。
5. 上电默认直接进入 DPLL 数字锁相模式，无需串口命令。
6. 如需完整波形直通，通过 USART1 发送字符 `0`。

当前已完成无界面的 CubeIDE Debug 编译：`0 errors, 0 warnings`，生成
`Debug/phase_locking_codex.elf`。实物输入范围、锁定时间、幅相误差和最高可靠频率仍需上板测量。

## 5. 串口命令

串口参数：115200 baud、8 数据位、无校验、1 停止位。

| 命令 | 功能 |
| --- | --- |
| `0` | 切换到直接同步还原模式 |
| `1` | 切换到 I/Q 二阶 DPLL 模式 |
| `r` | 清除 DPLL 状态并重新捕获 |
| `+` | 目标输出相位增加 5° |
| `-` | 目标输出相位减少 5° |
| `s` 或 `?` | 输出一次状态 |

状态示例：

```text
mode=dpll lock=locked run=1 f_mHz=10000000 phase_mdeg=800 target_mdeg=0 amp=12000 offset=32768 blocks=100 err=0/0/0
```

主要字段：

- `lock`：`no_signal`、`acquiring`、`tracking` 或 `locked`；
- `f_mHz`：频率，单位 mHz，除以 1000 得到 Hz；
- `phase_mdeg`：当前相位误差，单位千分之一度；
- `target_mdeg`：设置的目标相位；
- `amp`、`offset`：ADC 码值下的幅值和直流偏置；
- `err`：ADC 错误、DAC 错误、处理积压计数。

## 6. 两种模式怎么选

### 直接模式

发送 `0`。程序将 ADC 16 位采样按比例转换为 DAC 12 位输出。

优点：

- 能保留正弦、方波、三角波及其谐波；
- 算法简单，CPU 占用低；
- ADC 和 DAC 共用同一硬件时钟，延迟固定，不会随时间漂移。

限制：

- 双缓冲产生固定 4096 点延迟，即 4.096 ms；
- DAC 只有 12 位，无法保留 ADC 的全部 16 位量化精度；
- STM32H743 片内 ADC/DAC 的增益、偏置及模拟带宽误差需要实板校准。

### DPLL 模式

发送 `1`。程序先用带迟滞的插值上升过零估计粗频率，再用 I/Q 相关得到相位误差，二阶 PI 环路调整 NCO 频率，最后提前预测到 DAC 实际播放时刻并生成正弦波。过零迟滞用于避免噪声在零点附近制造假过零。

优点：

- 输入和输出频率相同，锁定后相位差稳定；
- 可用 `+`、`-` 调整目标相位；
- 输出比直接模式更干净。

限制：

- 只还原正弦基波，不保留方波、三角波的全部谐波；
- 默认设计范围为 1 kHz～100 kHz；
- 输入幅度过小、削顶、噪声很大或含有多个强频率时可能无法可靠锁定。

## 7. 代码结构和实现思路

所有自定义代码均放在 `Core/User`：

| 文件 | 作用 |
| --- | --- |
| `config.h` | 采样率、DMA、DPLL、NCO 参数 |
| `system.c/.h` | 统一初始化、主循环调度、串口命令 |
| `signal_chain.c/.h` | ADC/DAC DMA、Cache、模式切换 |
| `dpll.c/.h` | 粗测频、I/Q 检相、二阶环路、相位预测 |
| `nco.c/.h` | 2048 点正弦表、线性插值、Q32 相位累加 |
| `uart_debug.c/.h` | 简单的非阻塞串口收发 |
| `dpll_host_test.c` | 可在电脑上运行的算法测试 |

数据流程：

```text
TIM2 1 MHz
   ├─触发 ADC1 → DMA 双缓冲 → 主循环处理
   └─触发 DAC1 ← DMA 双缓冲 ← 直接换算或 DPLL 预测波形
```

H743 开启了 D-Cache。DMA 缓冲区被固定放入 D2 RAM 的 `.dma_buffer` 段并按 32 字节对齐；CPU 读取 ADC 数据前失效 Cache，写完 DAC 数据后清理 Cache，避免 DMA 与 CPU 看到不同数据。

中断回调只设置一个标志，计算、数组处理和串口控制均在主循环完成。

## 8. 常用参数

需要调参数时只改 `Core/User/config.h`：

- `DPLL_LOOP_BANDWIDTH_HZ`：增大后跟踪更快但抗噪变差；
- `DPLL_MIN_AMPLITUDE_ADC_COUNTS`：无信号判断阈值；
- `DPLL_CROSSING_HYSTERESIS_RATIO`：粗测频过零迟滞比例；
- `DPLL_LOCK_PHASE_THRESHOLD_DEG`：锁定相位门限；
- `SIGNAL_DIRECT_GAIN_Q15`：直接模式增益校准；
- `SIGNAL_DIRECT_OFFSET_ADC_COUNTS`：直接模式偏置校准。

改动后必须重新编译。增益和偏置建议根据示波器或高精度万用表实测结果校准，不要凭空填写。

## 9. 已完成的软件验证

- CubeIDE Debug 完整编译：通过，0 errors、0 warnings；
- ELF 已生成；
- DMA 缓冲区地址：DAC `0x30000000`，ADC `0x30002000`，均位于 D2 RAM；
- 主机算法测试：10 kHz 捕获通过；
- 输入跳变到 12.345 kHz 后重新锁定通过，结果为 12345.149 Hz、相位误差 -0.331°；
- 9973.25 Hz、约 8% 幅度噪声的最终 DAC 相位测试通过：相位 0.280°，约 2 秒累计漂移 -0.016°；
- 无信号识别测试通过；
- 用户模块最大静态栈占用 104 字节。

以上是软件结果。最终的相位抖动、幅度误差、频率范围和长期稳定性必须接实板与示波器验证。
