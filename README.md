# h743_pre1 双 ADC 测量工程

## 项目用途

本工程面向大学生电子设计竞赛测量组，使用 STM32H743VIT6 的 ADC1/ADC2 双重规则同步模式采集两路信号。每通道采样率为 600 kSPS，每帧保存 65536 个同步样本对；帧满后暂停 TIM2，顺序执行两个通道的自定义 65536 点 F32 实数 FFT，再向串口屏发布 DC、Vpp、RMS、频率、THD、相位和波形分类结果。

开发环境为 STM32CubeIDE 1.19.0，STM32Cube FW_H7 V1.12.1。

## 硬件连接

- PC4：ADC1_INP4，CH1 模拟输入。
- PB1：ADC2_INP5，CH2 模拟输入。
- PA0：TIM5_CH1，0～3.3 V CMOS 外部频率输入，最高 30 MHz。
- PB12：AD9834 FSYNC，普通 GPIO 输出，空闲为高电平。
- PB13：SPI2_SCK，连接 AD9834 SCLK。
- PB15：SPI2_MOSI，连接 AD9834 SDATA。
- PA9：USART1_TX，连接串口屏 RX。
- PA10：USART1_RX，连接串口屏 TX（当前显示链路主要使用 TX）。
- ADC 输入必须处于 0 至 VDDA 范围内；前端衰减、偏置和保护电路应按实际硬件确认。

## CubeMX/CubeIDE 配置

- ADC1/ADC2：Dual Regular Simultaneous，ADC1 为主 ADC，ADC2 为从 ADC。
- ADC1 外部触发：TIM2 TRGO 上升沿。
- ADC DMA：DMA1 Stream0、Circular、Word/Word、Memory Increment、Very High Priority。
- TIM2：240 MHz 输入时钟，Prescaler `0`，Counter Period `399`，TRGO 为 Update Event，得到 600000 次/秒触发。
- TIM5：External Clock Mode 1，触发源 TI1FP1，上升沿，Prescaler `0`，Counter Period `0xFFFFFFFF`，输入滤波 `0`。
- TIM3：240 MHz 输入时钟，Prescaler `23999`，Counter Period `9999`，产生 1 Hz 更新中断；NVIC 优先级为 6。
- SPI2：Transmit Only Master、16 bit、MSB First、CPOL High、CPHA 1 Edge、软件 NSS；SPI123 内核时钟为 64 MHz，Prescaler `8`，SCLK 为 8 MHz。
- ADC kernel clock：当前 `.ioc` 为 64 MHz；采样时间 8.5 cycles。更改 ADC 分辨率、时钟或采样时间后必须重新核算转换时间。
- USART1：9600 8N1，用于串口屏刷新。

在 STM32CubeIDE 中重新生成代码前，启用 `Project Manager > Code Generator > Keep User Code when re-generating`。不要手工修改 CubeMX 生成的 `MX_*_Init()`；用户代码统一位于 `Core/User`。

## 采集与 FFT 参数

- 每通道采样率：600000 sample/s。
- 帧长：65536 点。
- 单帧采集时间：约 109.227 ms。
- FFT 频点间隔：约 9.155273 Hz。
- Nyquist 频率：300 kHz。
- 测量主峰和串口屏压缩频谱范围：0 至 120 kHz。
- 实数 FFT 输出格式：`[DC, Nyquist, Re(1), Im(1), ...]`。

自定义 FFT 使用偶奇打包、32768 点原地基 2 复数 FFT 和实数后处理。Hann 窗与逐级旋转因子通过递推生成，不保存完整查找表。CH1、CH2 顺序复用同一工作区。

## SRAM 布局

- `.adc_sample_pairs`：65536 个 `adc_dual_sample_pair_t`，共 256 KiB，放入 RAM_D2。
- `.fft_f32_work`：65536 个 `float32_t`，共 256 KiB，放入 RAM_D1；RAM Debug 脚本中对应 RAM_EXEC。
- DMA packed-word 小循环缓冲区保持独立，并按 Cortex-M7 D-Cache 缓存行执行失效处理。

链接后应在 map 文件中检查上述两个段的地址和大小，避免后续新增大数组挤占对应 SRAM。

## 目录与调用关系

- `Core/User/system.c`：统一调用 `measurement_result_init()`、`measurement_fft_init()`、`adc_dual_init()` 和 HMI 初始化。
- `Core/User/adc_dual.c`：领取 DMA 标志、维护 Cache、拆分 32 位双 ADC packed word 并提交同步样本对。
- `Core/User/fft_f32_65536.c`：无 HAL 依赖的 65536 点 F32 实数 FFT。
- `Core/User/measurement_fft.c`：整帧存储、时域统计、两通道 FFT、诊断和结果发布。
- `Core/User/frequency_measure.c`：连续读取 TIM5 外部计数与 DWT 周期差，计算独立的外部信号平均频率。
- `Core/User/ad9834.c`：用 SPI2 和手动 FSYNC 写入 AD9834 控制字、频率字和相位字。
- `Core/User/dds_control.c`：将固定测试频率或 TIM5 粗测频率换算为低侧本振 `fDDS = fin - 100 kHz`。
- `Core/User/hmi_tjc.c`：只读取 `measurement_result_t`，不读取 DMA 缓冲区或 `adc_dual_stats_t`。

`frequency_measure_init()` 由 `system_init()` 调用，`frequency_measure_process()` 由 `system_process()` 调用。结果保存在独立调试变量 `frequency_measure_hz`，不替换 FFT 生成的频率结果。TIM3 只负责约 1 Hz 更新调度，频率按 TIM5 计数差和 DWT 实际周期差计算，从而补偿 FFT 造成的主循环延迟。

`main.c` 的用户初始化区只调用 `system_init()`，主循环只调用 `system_process()`。新增 `Core/User/fft_f32_65536.c` 和 `Core/User/frequency_measure.c` 后，需要按工程当前管理方式将它们加入 CubeIDE 构建；用户已选择自行处理 include path 和工程编译配置。

## AD9834 无比较器上板自检

当前 `Core/User/dds_control.h` 中 `DDS_CONTROL_FIXED_TEST_ENABLE` 默认为 `1`。程序把输入频率固定为 1 MHz，按低侧本振规划输出 900 kHz，并每秒重写一次频率寄存器，便于用示波器或逻辑分析仪同时检查 DDS 输出和 SPI 通信。

下载后可在调试器 Expressions 中观察：

- `ad9834_diagnostics.initialized`：应为 `1`。
- `ad9834_diagnostics.output_frequency_hz`：应为 `900000`。
- `ad9834_diagnostics.error_count`：应保持 `0`。
- `ad9834_diagnostics.write_count`：初始化后为 `5`，之后每秒增加 `2`。
- `dds_control_diagnostics.state`：应为 `dds_control_state_test`。

过零比较器接入 PA0/TIM5_CH1 后，将 `DDS_CONTROL_FIXED_TEST_ENABLE` 改为 `0`。此时 `dds_control_process()` 读取 `frequency_measure_hz`，输入处于 1～30 MHz 且目标本振变化不少于 500 Hz 时才更新 AD9834，SPI 操作始终在主循环内执行。

## 编译、烧录和运行

1. 在 STM32CubeIDE 1.19.0 中导入或打开 `h743_pre1`。
2. 确认 `Core/User` 和 CMSIS-DSP Include 已加入编译器 include path，并确认 `fft_f32_65536.c` 参与构建。
3. 选择 H743 Debug，执行 `Project > Clean...`，再执行 `Project > Build Project`。
4. 检查 map 文件中的 `.adc_sample_pairs` 与 `.fft_f32_work`。
5. 使用 ST-LINK 下载，先在无输入或限流信号下确认 DMA、TIM2 和串口屏状态，再接入测试信号。

## 验证方法与已知限制

- 用示波器或定时器诊断确认实际触发率为 600 kSPS，并观察 `adc_dual_stats_t` 的 DMA 次数、积压、溢出和丢弃计数。
- 向 PA0 依次输入 0 Hz、1 Hz、1 kHz、1 MHz、10 MHz 和 30 MHz，使用已校准信号源或频率计对比 `frequency_measure_hz`，并连续观察至少 60 秒。
- 外部频率结果是相邻两次主循环实际快照之间的平均值；快速扫频时测量窗口不严格对齐 TIM3 更新边沿。绝对精度还受 HSE 晶振和输入边沿质量影响。
- 依次输入 20 Hz、1 kHz、20 kHz、100 kHz、120 kHz，记录频率、Vpp、RMS、THD、FFT 周期数及长时间稳定性。
- 用两路相位可控信号检查 0°、±90°、180°；只有两通道主峰位置匹配时相位有效。
- 100 kHz 时每周期只有 6 个样本；二次谐波仍低于 Nyquist，三次谐波位于边界而不纳入 THD。
- 120 kHz 时每周期约 5 个样本；二次谐波 240 kHz 仍低于 300 kHz Nyquist，三次及以上谐波越界，因此高频端 THD 和波形分类仅供参考。
- 20 Hz 在单帧内约 2.18 个周期，虽然频率分辨率约 9.155 Hz，但低频幅度、THD和窗泄漏必须以真实硬件结果评估。
- 未完成电压校准时使用 `3.3 V / 65535` 标称比例，并通过估算标志对外说明；正式测量应设置每通道 `measurement_fft_calibration_t`。
- 当前仓库曾存在与本功能无关的 CMSIS-DSP 构建配置问题；若 Clean Build 首个错误仍来自旧工程源目录或 include 设置，应先修正工程构建清单，不要改动本测量算法规避错误。

离线源代码契约测试运行方式：

```powershell
python -m unittest discover -s tests -p 'test_*.py' -v
```

## 2026-07-17 板级验证记录

- STM32CubeIDE 1.19.0 H743 Debug 完整 Clean Build：0 errors、0 warnings。
- STM32CubeProgrammer 2.20.0 通过 ST-LINK V2 连接，目标电压 3.26 V；识别到 Device ID `0x450`、Rev V、Cortex-M7、2 MiB Flash。
- `h743_pre1.elf` 下载、回读校验和复位运行成功。
- 运行时 `SystemCoreClock=480 MHz`、D2 clock=240 MHz；TIM2 寄存器为 PSC=0、ARR=399、TRGO=Update，得到 600 kSPS 触发率。
- ADC12 common `MULTI=0x6`，确认为 Dual Regular Simultaneous。
- 连续运行快照：收齐 140 帧，完成并发布 139 帧；DMA half/full 均为 9099，错误、溢出、积压和采集重同步计数均为 0。
- 最近一次双通道 65536 点 FFT 与特征提取耗时为 321493154 个 Cortex-M7 周期，约 669.8 ms。
- 当前板上输入来源未知且两个通道尚未校准，`voltage_estimated_mask=0x03`；观察到的约 100 Hz 主峰、电压、THD 和相位只能证明数据链路持续运行，不能作为幅频或相位精度验收结果。
- 仍需使用已校准信号源和示波器/频率计人工完成 20 Hz、1 kHz、20 kHz、100 kHz、120 kHz、Vpp、RMS、THD 和 0°/±90°/180° 相位精度验证。
