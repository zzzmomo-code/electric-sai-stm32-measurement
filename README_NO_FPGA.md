# STM32H743 无 FPGA 单 ADC 版本

本工程是 G 题“周期信号测量分析装置”的独立无 FPGA 固件。它只采集一个 BNC 合成输入，使用 PA6/ADC1_INP3 完成片上 ADC 测量，继续复用原来的 USART1 串口屏页面、按键、350 点图形缓存和数据换算接口。

## 工程隔离

- 无 FPGA 工程目录：`h743_task2_no_fpga_20260729`
- 本地分支：`codex/no-fpga-onchip-adc`
- 原 FPGA 工程和分支未被修改。
- `fpga_link.*`、`fpga_protocol.*`、`hmi_tjc.*`、`hmi_chart.*`、`hmi_task2.*` 均未修改。
- SPI3、FPGA `DATA_READY` 和 USART1 配置仍保留；无 FPGA 固件只是不启动 FPGA 接收链路。

不要把本工程的 `.ioc` 或 `system.c` 覆盖回 FPGA 主力工程。比赛时两套固件分别从各自目录编译和烧录。

## 硬件与 IOC

| 项目 | 当前值 |
|---|---|
| MCU / IDE | STM32H743VIT6 / STM32CubeIDE 1.19.0 |
| 唯一模拟输入 | PA6 / ADC1_INP3，单端，中心约 1.65 V |
| ADC | 12 bit，异步时钟 64 MHz / 2 = 32 MHz，采样时间 2.5 cycles |
| 触发 | TIM2 TRGO Update，PSC=0，ARR=74 |
| 采样率 | 240 MHz / 75 = 3.2 MSPS |
| DMA | ADC1 → DMA1 Stream0，Halfword，Normal，Very High |
| 每帧 | 8192 点，2.56 ms |
| FFT 间隔 | 390.625 Hz |
| 串口屏 | USART1，512000 baud，原接口不变 |

TIM2 的 3.2 MHz TRGO 是 ADC1 的等间隔“采一个点”命令，不接比较器，也不直接测频。组合波形可能因强谐波多次过零，因此正式频率不使用原始波形过零法。

`.ioc` 已经配置和生成完毕。需要在 CubeIDE 中核对或重新生成时：

1. 双击 `h743_task2_no_fpga_20260729.ioc`。
2. `ADC1`：Independent、12 bit、TIM2 TRGO rising、DMA One Shot、Channel 3、2.5 cycles。
3. `ADC1 > DMA Settings`：DMA1 Stream0、Halfword/Halfword、Normal、Very High。
4. `TIM2`：Internal Clock、Prescaler 0、Period 74、Master Output Trigger=Update Event，不开 TIM2 中断。
5. `NVIC`：DMA1 Stream0 和 ADC IRQ 为 5/0。
6. 确认 PA6=ADC1_INP3；ADC2 不存在；SPI3、USART1、EXTI1 仍存在。
7. `Project Manager > Code Generator` 勾选 `Keep User Code when re-generating`，保存后执行 `Project > Generate Code`。

不要手改 `MX_ADC1_Init()`、`MX_TIM2_Init()` 或 `MX_DMA_Init()`；参数变化应回到 `.ioc` 修改后重新生成。

更完整的逐页设置和设计依据见 `docs/G26_NO_FPGA_SINGLE_ADC_DESIGN_AND_IOC.md`。

## 软件数据链

```text
PA6/ADC1
→ TIM2触发的8192点DMA突发采集
→ 去均值 + Hann窗 + 8192点实数FFT
→ 8～550 kHz候选峰
→ “基波必须存在 + 其他峰为整数倍”约束
→ FFT抛物线初值 + 五点残差精修
→ 基波/谐波/可见单频干扰联合I/Q最小二乘
→ 只用有效基波和谐波重建
→ Upp、Urms、基频、2～3个分量
→ 原measurement_conversion
→ 原hmi_task2/hmi_chart
→ USART1串口屏
```

强谐波允许高于基波。程序不会把全谱最大峰直接当基波，而是保留多个峰，枚举谐波阶次，并要求一次分量真实存在。

相位不需要单独显示，但联合拟合必须求正余弦系数。总峰峰值取决于各谐波相对相位；代码用这些系数在一个基波周期内重建 4096 点，再求最大值减最小值。

## 有效值和峰峰值

联合拟合得到每个有效分量的峰值幅度 `A_i` 后，正式有效值为：

```text
Urms = sqrt(sum(A_i² / 2))
```

这就是对去直流、去干扰后的有效波形做整周期“平方平均再开根号”。不能把几个 FFT 幅度直接相加后开根号。若改用主峰附近功率积分，必须积分 `|X[k]|²`，并补偿 Hann 窗功率、单边谱倍增和频谱泄漏；当前题目只有 2～3 个有效正弦分量，联合最小二乘的幅值结果更合适。

总 Upp 不等于各分量 Upp 之和。代码使用拟合出的全部有效分量及其相对相位重建波形，干扰项不参与重建，然后计算 `max-min`。

## 数字滤波与模拟抗混叠

当前正式“数字滤波”不是普通移动平均或低阶 IIR，而是模型投影：

- 只接受 8～550 kHz 且满足倍频关系的 2～3 个有效分量；
- 650 kHz～1.55 MHz 的独立单频峰作为干扰项与有效分量同时拟合；
- 输出波形、Upp 和 Urms 只由有效分量重建，干扰和宽带残差自动丢弃。

这样不会让 500 kHz 附近的幅值和相位被普通低通滤波器扭曲。

但 3.2 MSPS 的奈奎斯特频率只有 1.6 MHz。1.6～20 MHz 模拟干扰可能折叠进有效频带，折叠后纯软件无法知道原频率，因此模拟前端仍必须有抗混叠低通。可把“约 700 kHz 截止、6 阶 Butterworth，1.6 MHz 以上约 40 dB 或更高衰减”作为起点；实际阶数、器件值和 ADC 驱动网络必须结合运放型号、50 Ω 端接和实测重新确定。

## 代码入口

- `Core/User/onchip_fft_8192.c/.h`：实际 8192 点 Hann 实数 FFT。
- `Core/User/onchip_measurement.c/.h`：ADC DMA、缓存维护、检测、拟合、重建和显示兼容快照。
- `Core/User/system.c`：只调度片上测量、原换算模块和原 HMI。
- `Core/User/system.h`：用户代码统一头文件入口。
- `Core/Src/main.c`：初始化区只调用 `system_init()`，主循环只调用 `system_process()`。

8192 点 `uint16_t` DMA 缓冲区和 FFT 工作区均为静态分配；DMA 缓冲区 32 字节对齐，并在 CPU/DMA 交换前后执行 D-Cache 维护。ADC HAL 回调只置一个事件标志，FFT 和拟合均在主循环执行。

联合拟合函数的编译器静态栈估计约 760 bytes，分析函数约 728 bytes；考虑嵌套调用和中断现场，IOC/链接脚本已把主栈保留量从 1 KiB 提高到 8 KiB。

电压换算目前采用：

```c
ADC Vref = 3.300 V
ADC中心 = 1.650 V
模拟前端增益 = 1.0
```

如果实板固定增益不是 1，修改 `onchip_measurement.c` 顶部的 `ONCHIP_MEASUREMENT_FRONT_END_GAIN`，随后必须用标准源做多频点、多幅值校准。

## 编译与验证

Debug 已用 STM32CubeIDE 1.19.0 自带 GNU Arm 工具链成功生成：

```text
Debug/h743_task2_no_fpga_20260729.elf
text = 68076 bytes
data = 472 bytes
bss  = 93384 bytes（含 8 KiB 主栈保留）
编译警告 = 0
```

主机测试：

```powershell
gcc -std=c11 -O2 -Wall -Wextra -Werror `
  -I Core/User `
  tests/host/test_onchip_fft_8192.c `
  Core/User/onchip_fft_8192.c -lm -o test_onchip_fft_8192.exe
.\test_onchip_fft_8192.exe

python tests/host/test_measurement_math.py
```

已通过场景包括：实际 C FFT 的整数/非整数频点、幅度高于基波的三次谐波、五次谐波、1.1234 MHz/200 mVpp 单频干扰、联合拟合 RMS 和重建 Upp。主机测试只证明算法与代码构建，不证明 ADC 硬件精度。

## 上板顺序

1. PA6 输入稳定 1.65 V，检查平均码、噪声和是否触发超时。
2. 输入 100 kHz 单频，确认 8192 点采集频率和谱峰位置。
3. 输入“弱基波 + 强谐波”，确认基频没有被强谐波替代。
4. 覆盖 10 kHz、200 kHz、500 kHz 和频带边缘。
5. 覆盖总 Upp 50、100、250 mV，校准 Upp、Urms、各峰值幅度误差。
6. 加入 1 MHz、1.5 MHz、5 MHz、20 MHz 的 200 mVpp 干扰，分别观察 BNC、滤波器输出、PA6 和最终结果。
7. 查看 `onchip_measurement_diagnostics.last_analysis_time_ms` 和 `last_total_time_ms`，确认每项启动后小于 2 s。
8. 检查串口屏一周期、三周期、频谱和文本，确认切换页面时后台预装正常。

在这些测试完成前，幅值误差、抗干扰能力和实时性结论均为“待硬件验证”。
