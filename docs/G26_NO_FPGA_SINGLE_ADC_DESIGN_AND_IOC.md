# G 题无 FPGA 单 ADC 测量方案与 IOC 配置指南

> 状态：`.ioc`、无 FPGA 业务代码和主机数值测试已完成；Debug ELF 已成功编译，实板精度与耗时待验证。
> 目标 MCU：STM32H743VIT6，LQFP100，480 MHz。
> IDE：STM32CubeIDE 1.19.0，STM32Cube FW_H7 V1.12.1。
> 日期：2026-07-29。

## 1. 已确认的输入条件

- 题目只有一个 50 Ω BNC 输入端，装置始终只采集一个模拟通道。
- `u_a` 和 `u_b` 不是同时接到装置上的两路信号，而是测试时先后使用的两类单通道周期信号。
- 第 3 项中的相加是信号源输出 `u = u_b + u_J`，即有效信号与单频干扰已经在进入装置前相加。
- ADC 输入直流中心约为 1.65 V。
- 当前预计 ADC 引脚电压范围约为 0.2～3.1 V。
- 不使用 VGA，软件不能依赖自动量程切换。
- 需要报告的是去除直流偏置和不低于 1 MHz 单频干扰后的有效信号参数。
- `Upp` 是组合有效信号的总峰峰值，不是把各分量峰峰值直接相加。
- 高频干扰源最高可能接近 20 MHz。

题目三种输入工况如下：

| 工况 | 装置单路输入 `u` | 有效信号范围 | 正式显示对象 |
|---|---|---|---|
| 1 | `u = u_a` | 总 Upp 100～250 mV，全部分量 10～200 kHz | `u_a` |
| 2 | `u = u_b` | 总 Upp 50～250 mV，全部分量 10～500 kHz | `u_b` |
| 3 | `u = u_b + u_J` | `u_b` 同上；`u_J` 为 200 mVpp、`f_J ≥ 1 MHz` 单频干扰 | 抑制 `u_J` 后的 `u_b` |

`u_a`、`u_b` 都由一个基波和 1 个或 2 个谐波组成。题面例子中频谱分量幅值 `U1`、`U3`、`U4` 是正弦项的峰值幅度：

```text
u(t) = U1*sin(...) + U3*sin(...) + U4*sin(...)
```

因此串口屏的“各频率分量幅值”应显示峰值幅度，不是单分量峰峰值，也不是单分量 RMS。

本方案选择：

| BNC 输入 | MCU 引脚 | ADC 通道 |
|---|---|---|
| 唯一被测通道 `u` | PA6 | ADC1_INP3 |

PB1/ADC2_INP5 不用于本题正式采样。

## 2. 工程隔离边界

FPGA 方案优先级最高。无 FPGA 方案不得修改或删除以下现有模块：

```text
Core/User/fpga_link.c
Core/User/fpga_link.h
Core/User/fpga_protocol.c
Core/User/fpga_protocol.h
Core/User/hmi_tjc.c
Core/User/hmi_tjc.h
Core/User/hmi_chart.c
Core/User/hmi_chart.h
Core/User/hmi_task2.c
Core/User/hmi_task2.h
```

SPI3、FPGA `DATA_READY` 和 USART1 串口屏的 IOC 设置也不应为无 FPGA 方案重新设计。

无 FPGA 版本应从当前已确认的 FPGA/HMI 提交创建独立分支和独立工作目录，建议名称：

```text
分支：codex/no-fpga-onchip-adc
目录：h743_task2_no_fpga_20260729
```

当前工作区存在未提交源码和大量构建产物变化。在确认这些变化的归属并形成可恢复提交前，不要直接切换分支，也不要在当前 FPGA 工作目录修改 `.ioc`。

无 FPGA 版本只允许通过新增适配模块调用现有显示公开接口。最终数据流为：

```text
单 ADC 测量结果
→ onchip_measurement 生成兼容快照
→ 现有 measurement_conversion
→ 现有 hmi_task2 / hmi_chart
→ USART1 串口屏
```

## 3. 总体硬件数据链

```text
BNC单路输入u
→ 50 Ω端接、保护、模拟低通、1.65 V偏置和ADC驱动
→ PA6 / ADC1_INP3
→ TIM2 TRGO以3.2 MHz触发ADC1
→ ADC1数据寄存器
→ DMA1 Stream0，每次写入一个16位样本
→ 8192点突发采样缓冲区
→ 主循环分析当前单路信号
→ 现有350点显示快照与串口屏
```

信号源输出阻抗和电缆特性阻抗均为 50 Ω。装置 BNC 端的端接方式必须与信号发生器的负载设置一起标定，否则发生器面板幅值与 ADC 实际幅值可能相差一倍。

## 4. 为什么选择 12 位、3.2 MSPS

STM32H743VIT6 是 LQFP100 封装。ST 的 AN5354 给出的单 ADC 典型上限如下：

| 分辨率 | LQFP100 单 ADC 最大典型采样率 |
|---|---:|
| 16 bit | 1.9 MSPS |
| 14 bit | 2.67 MSPS |
| 12 bit | 直接通道 4.88 MSPS；快速通道 4.33 MSPS |

因此不能把“16 位、3.2 MSPS”作为可靠设计。首版选用：

```text
分辨率：12 bit
单路采样率：3.2 MSPS
样本数量：8192
采样时间：8192 / 3.2 MHz = 2.56 ms
FFT频率间隔：3.2 MHz / 8192 = 390.625 Hz
```

当前 ADC 外设时钟源为 64 MHz，异步二分频后 ADC 内核时钟为 32 MHz。12 位转换、2.5 周期采样时间时：

```text
单次转换周期约为 2.5 + 12/2 + 0.5 = 9 个 ADC 时钟
理论转换能力约为 32 MHz / 9 = 3.556 MSPS
```

高于 3.2 MSPS 触发率，留有约 11% 的时序余量。

12 位、3.3 V 满量程时一个理想 LSB 约为 0.806 mV。如果 ADC 引脚上的 50 mVpp 信号未被固定增益前端放大，只占约 62 LSB；8192 点联合拟合可以降低随机量化和噪声对幅值估计的影响，但不能弥补削顶、模拟失真或严重的 ADC 驱动不足。由于题目要求 Upp、Urms 和分量幅值绝对误差不超过 5 mV，实板必须做 50 mVpp、100 mVpp、250 mVpp 及干扰叠加工况标定。

## 5. 20 MHz 干扰与抗混叠边界

3.2 MSPS 的奈奎斯特频率只有 1.6 MHz。1.6 MHz 以上的模拟频率会折叠到 0～1.6 MHz。例如某些高频干扰可能刚好混叠到有效谐波位置，纯软件无法判断它原来来自 20 MHz。

因此本方案把处理顺序定义为：

```text
模拟低通负责抑制可能混叠的高频能量
→ ADC采样
→ FFT识别仍然残留且未混叠的窄带干扰
→ 联合拟合进一步扣除可辨认的单频残留
→ 只用已确认的基波/谐波重建有效信号
```

模拟前端建议目标：

| 指标 | 建议 |
|---|---|
| 有效通带 | 10～500 kHz |
| 通带保护范围 | 至少覆盖到 550 kHz |
| 过渡带 | 约 550 kHz～1.6 MHz |
| 1～1.55 MHz | 允许有残留，软件将独立单频项加入联合拟合并剔除 |
| 1.6 MHz 及以上 | 建议至少 40 dB，避免折叠进 0～550 kHz 有效带 |
| 可行起点 | 约 700 kHz 截止的 6 阶 Butterworth；需按实际运放重新计算并实测 |
| ADC 驱动 | 低输出阻抗、能在 2.5 个 ADC 时钟采样时间内稳定 |
| ADC 引脚储能电容 | 47 pF 可作为起始值，最终结合驱动器和稳定性实测 |

滤波器阶数和器件参数必须根据实际运放型号、输出稳定性、源阻抗和 PCB 重新计算，不能只凭软件方案确定。

软件只能把 1～1.6 MHz 内仍可见的独立谱线识别为“高频干扰候选”。对已经混叠的谱线不得声称测出了原始干扰频率。题目若只要求在干扰存在时正确测量有效信号，应优先保证模拟抑制，而不是强行显示干扰频率。

## 6. 采集调度

首版使用“突发采样”，不要求 ADC 永久连续运行：

```text
主循环启动ADC1单通道DMA
→ 启动TIM2
→ DMA自动采满8192个16位样本
→ DMA进入完成状态，不再覆盖缓冲区
→ 回调只置adc_frame_ready_flag
→ 主循环停止TIM2、领取完整帧并处理
→ 发布显示快照
→ 再启动下一次突发采样
```

因此 IOC 中建议使用 DMA Normal，而不是 Circular。这样算法即使需要几十毫秒，也不会在 2.56 ms 后被 DMA 覆盖。

缓冲区资源：

```text
8192 × uint16_t = 16384 bytes
```

缓冲区必须 32 字节对齐，位于 DMA1 可访问的 SRAM。CPU 在读取 DMA 已写入的数据前必须对对应缓存行执行 D-Cache invalidate。

## 7. 单路信号的算法主线

三种题目工况复用同一条分析链，不需要两个 ADC：

```text
原始ADC码值
→ 通道偏置/增益校准
→ 削顶和异常点检查
→ 去直流
→ Hann窗与8192点RFFT
→ 候选谱峰检测
→ FFT三点抛物线插值
→ 谐波倍频约束确定基频
→ 基频附近局部最小二乘精修
→ 多正弦联合I/Q/最小二乘拟合
→ 分离可辨认的单频干扰
→ 有效信号重建
→ Upp、Urms、分量参数和显示缓存
```

### 7.1 频谱候选

建议内部搜索范围：

```text
有效分量候选：8～550 kHz
题目合法结果：10～500 kHz
高频残留候选：650 kHz～1.55 MHz
```

边界外只用于避免 FFT 泄漏造成漏检，最后报告仍按题目范围裁定。

不能只取最大峰，因为谐波可能强于基波。先保留 6～10 个通过局部噪声门限的候选峰，再检查整数倍关系。

### 7.2 基频识别

对候选峰 `f_i` 枚举可能的谐波次数 `m_i`：

```text
f0_candidate = f_i / m_i
```

评分同时考虑：

- 能解释多少个显著谱峰；
- 各峰与整数倍频点的误差；
- 被解释峰的能量和信噪比；
- 是否存在题面要求的一次分量；
- 候选基频是否位于合法范围。

确认谐波编号后使用加权最小二乘：

```text
f0 = sum(w_i * m_i * f_i) / sum(w_i * m_i * m_i)
```

不对原始组合波形直接过零测频。基波带通后的过零结果只作为诊断，不参与首版正式结果。

### 7.3 多正弦联合拟合

在未加窗的原始数据上拟合：

```text
x[n] = c
     + sum(a_i*cos(2*pi*f_i*n/Fs) + b_i*sin(2*pi*f_i*n/Fs))
     + a_j*cos(2*pi*f_j*n/Fs) + b_j*sin(2*pi*f_j*n/Fs)
     + residual[n]
```

其中 `j` 是确认存在时才加入的单频干扰项。

每个有效分量的峰值和相位：

```text
A_i   = sqrt(a_i*a_i + b_i*b_i)
phase = atan2(-b_i, a_i)
```

相位不用于总 RMS，但用于组合波形重建、总 Upp 和干扰相减。

## 8. Urms 的正式定义

本方案的正式结果是：去除 1.65 V 直流偏置和不低于 1 MHz 干扰后，有效基波与谐波之和的真有效值。

如果拟合得到的是峰值幅度 `A_i`：

```text
Urms_model = sqrt(sum(A_i * A_i / 2))
```

如果得到的是单分量峰峰值 `Vpp_i`：

```text
Urms_model = sqrt(sum(Vpp_i * Vpp_i / 8))
```

各分量相位不影响完整基波周期上的 RMS，但不能把各频谱幅度直接相加后开根。若从 FFT 主瓣积分计算 RMS，必须对 `|X[k]|²` 求和，并完成 FFT 归一化、单边谱倍增和 Hann 窗功率补偿。

已实现正式值采用联合拟合幅值公式。它与对重建后的有效波形做整周期平方平均完全等价，但无需再遍历一遍重建点。FFT 主瓣积分不是当前正式值，因为 Hann 窗主瓣积分还需要窗功率、单边谱和泄漏补偿；在题目只有 2～3 个有效分量时，联合最小二乘更直接。

## 9. Upp 的正式定义

总 Upp 不能由各分量 Upp 直接相加，因为谐波相对相位决定组合波形极值。

正式结果采用有效分量重建：

```text
x_rebuild(t) = sum(A_i*cos(2*pi*f_i*t + phase_i))
```

在一个基波周期内生成 4096 个软件点：

```text
Upp_primary = max(x_rebuild) - min(x_rebuild)
```

复核值包括：

- 去干扰并低通后的时域分位数极差；
- 多周期相位折叠得到的实测周期波形极差。

500 kHz 在 3.2 MSPS 下每周期只有 6.4 个 ADC 点，所以不能把原始采样的简单 `max-min` 作为正式 Upp。

## 10. 数字滤波

已实现版本不在原始 ADC 点上串接普通 FIR，而采用“频率选择 + 联合投影 + 有效分量重建”作为正式数字滤波：

```text
原始去直流数据
→ FFT只在8～550 kHz内寻找有效候选
→ 在650 kHz～1.55 MHz内寻找独立单频干扰候选
→ 对有效谐波和干扰同时做正余弦最小二乘
→ 只用有效基波/谐波系数重建波形
→ 干扰项和其他频带残差不进入Upp、Urms及显示波形
```

这种处理相当于只保留题目允许的几条窄带正弦分量，不会引入普通低通在 500 kHz 附近的幅相误差，也直接利用了“基波 + 1 或 2 个谐波”的题设先验。原始 FFT 仍用于找峰，重建波形用于 Upp 和一周期/三周期显示。

不使用移动平均或中值滤波。它们会改变 500 kHz 附近幅值、谐波比例和相位，从而直接破坏 Upp。后续若实板发现宽带噪声导致谱峰门限不稳，可增加仅供诊断/复核的线性相位 FIR，但不得替代模拟抗混叠网络。

## 11. 显示输出

继续复用现有串口屏数据结构和 350 点压缩：

- 一周期波形：优先使用相位折叠的实测周期波形；
- 三周期波形：按同一相位基准生成；
- 频谱：有效显示范围按题目要求，内部保留更宽诊断范围；
- 参数：总 Upp、有效信号 Urms、基频、各有效分量频率和峰值幅度；
- 干扰：作为诊断状态，不混入有效分量列表。

系统继续后台采集、计算和预装显示缓存。按键只切换已经准备好的显示内容，不启动新的 ADC 或 FFT。

## 12. 已实现模块

所有新增 `.c/.h` 均位于 `Core/User`：

```text
onchip_fft_8192.c/.h
  8192点Hann实数FFT；固定32 KiB工作区，无动态内存。

onchip_measurement.c/.h
  ADC1突发DMA、D-Cache维护、谱峰检测、倍频约束、
  联合I/Q最小二乘、干扰分离、Upp/Urms和兼容显示快照。
```

`system.c` 只调度片上测量、现有 `measurement_conversion` 和现有 HMI 接口。三个 ADC HAL 回调各自只设置一个目的明确的标志；全部 FFT、拟合、重建和显示处理均在主循环执行。旧 `adc_dual`、`measurement_fft` 和 FPGA 模块仍保留在目录中，但本固件不启动其硬件路径。

## 13. STM32CubeIDE 1.19.0 IOC 配置步骤

以下步骤只能在独立的无 FPGA 工程副本中执行。

### 13.1 打开配置

1. 在 Project Explorer 中双击无 FPGA 工程的 `.ioc`。
2. 等待 Device Configuration Tool 完全加载。
3. 先打开 `Project Manager > Code Generator`。
4. 确认勾选 `Keep User Code when re-generating`。
5. 不删除 SPI3、USART1、EXTI1 及其 DMA 设置。

### 13.2 配置单 ADC 引脚

在 `Pinout & Configuration` 页面：

1. 将 `PA6` 设为 `ADC1_INP3`。
2. PA6 设置为 `Analog`、`No pull`。
3. PB1/ADC2_INP5 不用于正式采样。
4. 不使用 PC4/OPAMP1 作为本题 ADC 输入。

### 13.3 配置 ADC1 单通道模式

在 `Analog > ADC1`：

1. ADC Mode 选择 `Independent mode`。
2. 不选择 Dual Regular Simultaneous。
3. 不开启双 ADC 数据打包。

ADC1 建议参数：

| 参数 | 值 |
|---|---|
| Clock Prescaler | Asynchronous clock divided by 2 |
| Resolution | 12 Bits |
| Scan Conversion Mode | Disable |
| Continuous Conversion Mode | Disable |
| Number Of Conversion | 1 |
| External Trigger Conversion Source | Timer 2 Trigger Out event |
| External Trigger Edge | Rising edge |
| Conversion Data Management | DMA One Shot |
| Overrun | Data overwritten |
| Left Bit Shift | None |
| Oversampling | Disable |
| Regular Channel | Channel 3 |
| Rank | 1 |
| Sampling Time | 2.5 Cycles |
| Single/Differential | Single-ended |
| Offset | None |

本独立工程已经禁用 ADC2。`system.h` 不定义 `SYSTEM_ADC_DUAL_AVAILABLE`，因此历史 `adc_dual.c` 仍可保留并编译，但其双 ADC 硬件路径和 HAL 回调不会进入本固件；正式回调由 `onchip_measurement.c` 提供。

### 13.4 配置 ADC1 DMA

打开 `ADC1 > DMA Settings`，保留一条 ADC1 请求：

| 参数 | 值 |
|---|---|
| DMA Request | ADC1 |
| DMA Instance | DMA1 Stream0 |
| Direction | Peripheral to Memory |
| Peripheral Increment | Disable |
| Memory Increment | Enable |
| Peripheral Data Width | Half Word |
| Memory Data Width | Half Word |
| Mode | Normal |
| Priority | Very High |
| FIFO | Disable |

单 ADC 12 位结果使用 Half Word/Half Word，DMA 缓冲区类型使用 `uint16_t`。

### 13.5 配置 TIM2

打开 `Timers > TIM2`：

1. Clock Source 选择 `Internal Clock`。
2. Prescaler 设为 `0`。
3. Counter Mode 选择 `Up`。
4. Counter Period 设为 `74`。
5. Clock Division 设为 `DIV1`。
6. Auto-reload preload 设为 `Disable`。
7. `Trigger Event Selection / Master Output Trigger` 选择 `Update Event`。
8. Master/Slave Mode 设为 `Disable`。
9. 不开启 TIM2 Update 中断。

当前 TIM2 时钟为 240 MHz，因此：

```text
Fs = 240 MHz / (0 + 1) / (74 + 1) = 3.2 MHz
```

TIM2 TRGO 只负责命令 ADC1 采一个点，不接输入比较器，也不参与过零测频。

### 13.6 检查时钟

打开 `Clock Configuration`：

- CPU Clock 保持 480 MHz；
- HCLK 保持 240 MHz；
- APB1 Timer Clock 保持 240 MHz；
- CKPER/ADC 外设输入时钟保持 64 MHz；
- ADC1 Asynchronous Divide by 2 后应为 32 MHz；
- 不为无 FPGA 方案改动 SPI3 和 USART1 的时钟源。

如果 CubeMX 显示 ADC 时钟错误或红色冲突，先截图，不要自行改动 PLL。

### 13.7 配置 NVIC

打开 `System Core > NVIC`：

| 中断 | 使能 | 抢占优先级 | 子优先级 |
|---|---|---:|---:|
| DMA1 Stream0 global interrupt | Enable | 5 | 0 |
| ADC1 and ADC2 global interrupt | Enable | 5 | 0 |
| TIM2 global interrupt | Disable | - | - |

保留现有 SPI3、USART1 和 EXTI1 中断配置，不调整 FPGA/HMI 相关优先级。

### 13.8 生成代码前后的检查

生成前：

1. 保存 `.ioc`。
2. 截图 Pinout、ADC1、DMA、TIM2、Clock Configuration 和 NVIC 六个页面。
3. 确认 PA6 没有引脚冲突。
4. 确认 SPI3 和 USART1 仍然存在。

生成：

```text
Project > Generate Code
```

生成后不要手改 `MX_ADC1_Init()`、`MX_TIM2_Init()` 或 `MX_DMA_Init()`。先核对生成结果应包含：

```c
hadc1.Init.Resolution = ADC_RESOLUTION_12B;

sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;

multimode.Mode = ADC_MODE_INDEPENDENT;

htim2.Init.Prescaler = 0;
htim2.Init.Period = 74;
sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;

hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
hdma_adc1.Init.Mode = DMA_NORMAL;
```

本工程已经按上述参数完成生成。以后若手动重新生成代码，必须再次核对这些字段，并确认 SPI3、USART1 和 FPGA `DATA_READY` 引脚配置没有变化。

## 14. 分阶段上板验证

代码实现后不直接运行完整算法，按以下顺序验证：

1. PA6 接 1.65 V，确认均值、噪声和偏置。
2. 输入单频 100 kHz，确认 DMA 采样率、码值范围和频率峰位置。
3. 输入 `u_a` 工况，覆盖 10 kHz、100 kHz、200 kHz。
4. 输入 `u_b` 工况，覆盖 10 kHz、100 kHz、500 kHz。
5. 分别使用基波加 1 个谐波、基波加 2 个谐波，检查谐波关系识别。
6. 50 mVpp、100 mVpp、250 mVpp 检查 Upp、Urms 和分量幅值绝对误差是否不超过 5 mV。
7. 形成 `u = u_b + u_J`，依次加入 1 MHz、1.5 MHz、5 MHz、20 MHz、200 mVpp 干扰，观察 BNC 端、ADC 引脚和 FFT。
8. 记录采样、FFT、拟合、滤波和显示耗时，确认每项启动后不超过 2 s。

所有频率、幅值、RMS、Upp 和抗干扰精度结论在完成上述测试前均标记为“待硬件验证”。

## 15. 参考资料

- ST AN5354：Getting started with 16-bit ADC on STM32H7 MCUs
  https://www.st.com/resource/en/application_note/an5354-getting-started-with-16bit-adc-on-stm32h7-mcus-stmicroelectronics.pdf
- STM32H742/743/753/750 Reference Manual RM0433。
- 当前工程 `h743_task2_20260727.ioc`、`README.md` 与 `docs/PROJECT_HANDOFF.md`。
