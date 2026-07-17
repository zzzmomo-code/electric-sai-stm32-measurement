# TIM5 外部计数频率测量设计

## 1. 目标与范围

为 STM32H743VIT6 工程增加一路独立的外部数字信号频率测量功能。被测信号从 `PA0/TIM5_CH1` 输入，最高频率为 30 MHz，信号已经整形成 0～3.3 V CMOS 方波。

测量结果只保存到独立变量 `frequency_measure_hz`，不替换现有 FFT 生成的 `measurement_result.frequency_hz`，也不直接发送到串口屏。

## 2. 已确认方案

采用 TIM5 连续外部计数、TIM3 产生 1 Hz 调度中断、Cortex-M7 DWT 补偿主循环处理延迟的方案。

TIM3 只负责通知主循环进行一次更新，不作为严格的一秒硬件门控。实际频率按相邻两次主循环快照之间的 TIM5 计数差和 DWT 周期差计算：

```text
frequency_hz = counter_delta * SystemCoreClock / cycle_delta
```

因此，即使双通道 FFT 造成主循环延迟，稳定信号的平均频率仍按实际测量间隔计算。测量窗口起止位置会随主循环延迟移动，不承诺严格覆盖 TIM3 更新事件之间的整整一秒。

## 3. CubeIDE/CubeMX 配置

开发环境为 STM32CubeIDE 1.19.0，当前 APB1 定时器时钟为 240 MHz。

### 3.1 TIM5 外部计数

- GPIO：`PA0`，复用为 `TIM5_CH1/AF2`。
- GPIO 上下拉：`No Pull`。
- GPIO 速度：`Very High`。
- Slave Mode：`External Clock Mode 1`。
- Trigger Source：`TI1FP1`。
- 触发极性：上升沿。
- 输入滤波：`0`。
- Prescaler：`0`。
- Counter Mode：向上计数。
- Counter Period：`0xFFFFFFFF`。
- Auto-reload preload：关闭。
- 不开启 TIM5 中断和 DMA。
- TIM5 TRGO 保持 `Reset`，Master/Slave Mode 保持关闭。

TI1FP1 被外部时钟模式占用后，CubeMX 中 Channel 1 选项呈灰色属于正常现象。TIM5 的 CNT 连续运行，软件不在每次测量后清零。

### 3.2 TIM3 一赫兹调度

- Clock Source：内部时钟。
- Prescaler：`23999`。
- Counter Period：`9999`。
- Counter Mode：向上计数。
- Clock Division：不分频。
- Auto-reload preload：关闭。
- 开启 TIM3 Global Interrupt。
- NVIC 抢占优先级：`6`。
- NVIC 子优先级：`0`。

一赫兹更新频率为：

```text
240000000 / (23999 + 1) / (9999 + 1) = 1 Hz
```

现有 ADC 和 DMA 中断优先级为 5，TIM3 优先级较低，不抢占采样中断。

### 3.3 代码生成约束

- 保持 `Project Manager > Code Generator > Keep User Code when re-generating` 开启。
- TIM3、TIM5、GPIO 和 NVIC 初始化由 CubeMX 生成。
- 不手工修改 `MX_TIM3_Init()`、`MX_TIM5_Init()`、MSP 初始化或 `TIM3_IRQHandler()`。
- CubeMX 生成的 `MX_TIM3_Init()` 和 `MX_TIM5_Init()` 位于 `system_init()` 之前。
- `main.c` 用户初始化区仍只调用 `system_init()`，主循环仍只调用 `system_process()`。

## 4. 用户模块设计

新建以下文件：

- `Core/User/frequency_measure.h`
- `Core/User/frequency_measure.c`

模块对外接口：

```c
void frequency_measure_init(void);
void frequency_measure_process(void);
```

共享变量：

```c
extern volatile uint8_t frequency_measure_flag;
extern volatile float frequency_measure_hz;
```

`frequency_measure_flag` 由 TIM3 回调和主循环共享；`frequency_measure_hz` 是最新一次有效测量结果，供调试器或后续用户模块读取。

所有用户代码均位于 `Core/User`，函数和变量使用小写下划线命名，并在交付前补全中文模块说明、GPIO 映射、依赖、初始化方法和调用方法。

## 5. 初始化与运行流程

`frequency_measure_init()` 执行以下操作：

1. 确认 DWT 周期计数器已启用；若未启用则使能，但不清零 `DWT->CYCCNT`。
2. 将 TIM5 CNT 清零一次。
3. 启动 TIM5 基本计数。
4. 启动 TIM3 更新中断。
5. 保存初始 TIM5 CNT 和 DWT 周期值作为第一组基准。
6. 将频率结果初始化为 0 Hz，并清除更新标志。

现有 `measurement_fft_init()` 已启用并清零 DWT。频率模块必须复用同一 DWT 计数器，不得在运行中再次清零，以免破坏 FFT 耗时诊断。

`frequency_measure_process()` 由 `system_process()` 调用：

1. 检查并领取 `frequency_measure_flag`；没有更新请求时立即返回。
2. 在主循环中清除标志。
3. 获取当前 TIM5 CNT 和 DWT 周期值。
4. 通过 32 位无符号减法计算计数差和周期差，以支持一次自然回绕。
5. 周期差非零时计算频率并更新 `frequency_measure_hz`。
6. 保存当前快照，供下一次计算使用。

TIM5 在 30 MHz 输入下约 143 秒回绕一次；DWT 在 480 MHz 下约 8.95 秒回绕一次。当前 FFT 最长处理时间约 670 ms，正常快照间隔明显小于 DWT 回绕周期。无符号差值能够处理一次回绕，但不支持两次快照之间跨越多个 DWT 回绕周期。

## 6. 中断与并发约束

在 `frequency_measure.c` 中实现 HAL 定时器周期回调。回调识别 TIM3 后只执行一次与该中断目的对应的标志赋值：

```c
frequency_measure_flag = 1u;
```

回调内不读取 TIM5、不读取 DWT、不计算频率、不打印，也不清除业务标志。TIM3 硬件中断状态仍由 CubeMX 生成的 `TIM3_IRQHandler()` 调用 HAL 处理，这是 HAL 正常清除中断状态所必需的生成代码。

若主循环超过一个 TIM3 周期未领取标志，多次更新请求可能合并为一次，但下一次计算仍使用实际计数差和实际 DWT 时间差，不会把较长间隔误认为一秒。

## 7. 精度与限制

- 30 MHz、约一秒测量窗口下，脉冲计数量化误差约为一个计数，即约 0.033 ppm。
- DWT 时间分辨率在 480 MHz 下约为 2.08 ns。
- 最终绝对精度主要受 HSE 晶振精度、PLL 时基、输入信号抖动、边沿质量和 PA0 信号完整性影响。
- 结果是相邻两次实际快照之间的平均频率；快速扫频或突变信号的时间窗口不会严格对齐 TIM3 更新边沿。
- 输入必须保持在 MCU 允许的数字电平范围内，不允许负压或超过器件允许的最高输入电压。
- 输入为 0 Hz 时，下一次有效计算结果为 `0.0f`。
- DWT 周期差为零时不更新频率结果。

## 8. 验证计划

### 8.1 离线检查

- 检查新增用户文件全部位于 `Core/User`。
- 检查 `system.h` 是用户模块统一头文件入口。
- 检查 `main.c` 用户初始化区和主循环未增加业务逻辑。
- 检查 TIM3 回调只修改 `frequency_measure_flag`。
- 检查 TIM5 只在初始化时清零，测量过程中连续计数。
- 检查频率计算使用计数差和 DWT 实际周期差，而不是固定除以一秒。
- 检查 32 位无符号回绕计算。
- 运行现有 Python 契约测试及新增频率模块契约测试。
- 在 STM32CubeIDE 1.19.0 中执行 Clean Build，记录错误和警告数量。

### 8.2 板级验证

依次输入并记录以下频率：

- 0 Hz
- 1 Hz
- 1 kHz
- 1 MHz
- 10 MHz
- 30 MHz

使用已校准信号源、示波器或频率计作为参考，至少连续观察 60 秒，检查 `frequency_measure_hz` 的准确度、更新稳定性和 TIM5 回绕附近的连续性。同时确认 ADC DMA、FFT 和串口屏功能仍正常运行。

## 9. 完成条件

- CubeMX 生成配置与本规格一致。
- 新模块、`system.c`、`system.h` 和 README 均完成中文说明。
- 独立频率变量可由调试器稳定读取，不修改现有 FFT 频率结果。
- 当前条件允许的测试和编译均已执行并如实报告。
- 完成后等待用户确认；未经明确许可不推送远程仓库。
