# 内部 ADC 与 ADS8688 动态测量输入设计

## 1. 目标

在保留现有 STM32H743 内部 ADC1/ADC2 双路同步采集链路的同时，恢复并完善
ADS8688 外部 ADC 驱动。系统通过统一接口在运行期间动态选择测量输入源，并让
`measurement_fft` 根据当前来源、通道数、采样率、量程和通道采样时间差正确分析。

系统上电默认启用内部 ADC。DDS2 从 SPI3 迁移到 SPI6 不属于本次范围。

## 2. 硬件映射与 CubeMX 配置

ADS8688 使用 SPI3：

| 信号 | STM32H743 引脚 | CubeMX 功能 |
|---|---|---|
| SCK | PC10 | SPI3_SCK |
| CS | PA15 | SPI3_NSS |
| MISO | PC11 | SPI3_MISO |
| MOSI | PC12 | SPI3_MOSI |
| DAISY | PD0 | GPIO_Output，初始低电平 |
| RST | PD1 | GPIO_Output，初始高电平 |

SPI3 参数：

- Full-Duplex Master；
- Motorola 帧格式；
- 32 位数据帧，MSB First；
- CPOL Low，CPHA 2 Edge；
- Hardware NSS Output，NSS Pulse Enabled，NSS Polarity Low；
- SPI1/2/3 内核时钟保持 CKPER 64 MHz；
- 分频系数为 4，SCLK 为 16 MHz；
- Master SS Idleness 为 00 Cycle；
- Master Inter-Data Idleness 保持 01 Cycle；
- Master Keep IO State Enabled；
- CRC Disabled，FIFO Threshold 01 Data。

DMA 参数：

| 请求 | Stream | 方向 | 模式 | 数据宽度 | 内存递增 | 优先级 |
|---|---|---|---|---|---|---|
| SPI3_RX | DMA1 Stream1 | Peripheral-to-Memory | Circular | Word/Word | Enable | Very High |
| SPI3_TX | DMA1 Stream2 | Memory-to-Peripheral | Circular | Word/Word | Disable | Very High |

DMA1 Stream1、DMA1 Stream2 和 SPI3 全局中断使用抢占优先级 5、子优先级 0。
SPI3 引脚使用 Very High Speed、No Pull；PD0/PD1 使用推挽输出、No Pull、Low Speed。

SPI3 RX/TX DMA 缓冲区使用专用链接段放入 DMA1 可访问的 D2 SRAM，并按 32 字节
对齐。发送缓冲区在 DMA 启动前清理 D-Cache，接收半区在主循环读取前执行
D-Cache 失效。

当前 `MX_GPIO_Init()`、`MX_DMA_Init()` 和 `MX_SPI3_Init()` 均在 `system_init()`
之前执行。CubeMX 生成的外设初始化代码保持在生成位置，不迁入用户模块。

## 3. 模块架构

新增 `Core/User/measurement_input.c/.h`，作为采集源选择和切换的唯一业务入口：

```c
typedef enum
{
    MEASUREMENT_INPUT_INTERNAL_ADC = 0,
    MEASUREMENT_INPUT_ADS8688
} measurement_input_source_t;

typedef enum
{
    MEASUREMENT_INPUT_STATUS_OK = 0,
    MEASUREMENT_INPUT_STATUS_INVALID_ARGUMENT,
    MEASUREMENT_INPUT_STATUS_STOP_FAILED,
    MEASUREMENT_INPUT_STATUS_START_FAILED,
    MEASUREMENT_INPUT_STATUS_ROLLBACK_FAILED
} measurement_input_status_t;

void measurement_input_init(void);

measurement_input_status_t measurement_input_select(
    measurement_input_source_t source);

measurement_input_source_t measurement_input_get_source(void);

void measurement_input_process(void);
```

数据流：

```text
内部 ADC DMA -> adc_dual ------+
                               +-> measurement_input -> measurement_fft
SPI3 DMA -> ADS8688 驱动 ------+
```

任意时刻只有一个采集源运行 DMA。`system_init()` 不再直接启动 `adc_dual`，而是
调用 `measurement_input_init()` 并默认选择内部 ADC。`system_process()` 通过
`measurement_input_process()` 调度当前来源，不再直接重复调用
`adc_dual_process()`。

## 4. 动态切换事务

`measurement_input_select()` 按以下顺序切换：

1. 停止当前采集源；
2. 清除当前来源尚未处理的 DMA 和错误标志；
3. 清除 `measurement_fft` 未完成的采样窗口；
4. 设置目标来源的通道数、采样率、通道时间差和电压校准；
5. 启动目标采集源；
6. 全部成功后更新当前来源和诊断计数。

重复选择当前来源返回成功且不重复启停硬件。目标来源启动失败时尝试恢复原来源；
只有恢复成功后才继续报告原来源为活动状态。回滚也失败时进入明确的错误状态。

运行期间发生 ADS8688 SPI/DMA 错误时，由主循环执行有限次数的停止、硬件复位、
寄存器重配和重启。恢复失败后保持错误状态，不静默切换到内部 ADC。用户可显式
选择内部 ADC。

## 5. 底层采集生命周期

ADS8688 驱动提供：

```c
ads8688_status_t ads8688_init(void);
ads8688_status_t ads8688_start(void);
ads8688_status_t ads8688_stop(void);
void ads8688_process(void);
```

内部 ADC 驱动补充：

```c
adc_dual_status_t adc_dual_start(void);
adc_dual_status_t adc_dual_stop(void);
```

`init()` 只完成状态和硬件配置，`start()` 启动 DMA 或触发定时器，`stop()` 停止采集
并清除待处理标志。启动和停止接口应为幂等操作。内部 ADC 停止时关闭 TIM2 触发，
但保留 ADC 多模式 DMA 的可恢复配置。

HAL DMA 半完成、完成和错误回调分别只给各自的一个 `xxx_flag` 标志赋值。缓存维护、
样本解析、FFT 提交、故障恢复和标志清除全部在主循环调用的功能函数中执行。

## 6. ADS8688 初始化、量程和模式

硬件初始化将 DAISY 保持低电平，执行 RST 低脉冲和器件恢复等待，配置自动扫描和
输入量程，并通过寄存器读回校验配置。

默认双通道为 AIN0 和 AIN1，默认量程均为双极性 ±5.12 V。继续支持五种量程：

- 双极性 ±10.24 V；
- 双极性 ±5.12 V；
- 双极性 ±2.56 V；
- 单极性 0 V 至 10.24 V；
- 单极性 0 V 至 5.12 V。

保留并完善以下接口：

```c
ads8688_status_t ads8688_set_auto_mode(uint8_t channel_mask);
ads8688_status_t ads8688_set_manual_mode(uint8_t channel);

ads8688_status_t ads8688_set_single_channel(uint8_t channel);

ads8688_status_t ads8688_set_dual_channel(
    uint8_t primary_channel,
    uint8_t secondary_channel);

ads8688_status_t ads8688_set_channel_range(
    uint8_t channel,
    ads8688_range_t range);

ads8688_status_t ads8688_get_channel_range(
    uint8_t channel,
    ads8688_range_t *range);

ads8688_status_t ads8688_get_diagnostics(
    ads8688_diagnostics_t *diagnostics);
```

活动采集中修改模式或量程时，驱动停止 DMA，执行写入和读回校验，再恢复 DMA；
配置失败时恢复旧模式、旧量程和原采集状态。

供测量业务使用的高层接口为：

```c
measurement_input_status_t
measurement_input_set_ads8688_single_channel(uint8_t channel);

measurement_input_status_t
measurement_input_set_ads8688_dual_channel(
    uint8_t primary_channel,
    uint8_t secondary_channel);
```

高层接口负责同步更新 FFT 输入配置和重同步采样窗口。直接调用 ADS8688 底层模式
接口只用于不接入测量链路的底层控制或测试。

## 7. measurement_fft 动态输入配置

新增运行时输入配置：

```c
typedef enum
{
    MEASUREMENT_FFT_INPUT_SINGLE_CHANNEL = 1,
    MEASUREMENT_FFT_INPUT_DUAL_CHANNEL = 2
} measurement_fft_input_channel_mode_t;

typedef struct
{
    float sample_rate_hz;
    float ch2_delay_seconds;
    measurement_fft_calibration_t calibration[2];
    measurement_fft_input_channel_mode_t channel_mode;
} measurement_fft_input_profile_t;

uint8_t measurement_fft_configure_input(
    const measurement_fft_input_profile_t *profile);
```

FFT 频点宽度、峰值频率、频谱横轴、奈奎斯特频率、THD 频点范围和频率校准输入均
使用当前 `sample_rate_hz`，不再固定使用 600 kSPS。

内部 ADC 配置：

- 双通道；
- 每通道 600 kSPS；
- 两路同步采样，CH2 时间差为 0；
- 使用内部 ADC 当前的独立校准参数。

ADS8688 双通道配置：

- AIN0/AIN1 默认为主、次通道，也允许通过高层接口选择其他两个有效通道；
- 当前 16 MHz、32 位帧和 1 周期帧间空闲配置的名义总转换率约为
  484.85 kSPS；
- 每通道名义采样率约为 242.42 kSPS；
- 次通道比主通道晚一个转换帧，名义时间差约为 2.0625 us；
- 双极性 ±5.12 V 的理想换算参数为
  `volts_per_code = 0.00015625 V`、`offset_v = -5.12 V`。

SPI3 内核时钟通过 `HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI3)` 读取，并结合
当前分频、数据位宽和帧间空闲计算名义采样率，避免将时钟源写死在算法中。

当前相位定义为 CH2 相位减 CH1 相位。ADS8688 顺序采样的修正公式为：

```text
corrected_phase =
    wrap(raw_phase - 360 deg * signal_frequency_hz * ch2_delay_seconds)
```

内部同步 ADC 的 `ch2_delay_seconds` 为 0，因此不产生额外修正。

## 8. ADS8688 单通道 FFT

ADS8688 单通道模式只采集指定通道，并将其提交为 FFT 主通道。当前 SPI3 配置下，
单通道名义采样率约为 484.85 kSPS。

单通道模式只执行第一路 FFT，输出以下主通道结果：

- 直流电压；
- 峰峰值；
- RMS；
- 频率；
- THD；
- 波形类型；
- 频谱。

第二路结果必须满足：

- `secondary_valid_mask = 0`；
- `secondary_mode = MEASUREMENT_MODE_UNKNOWN`；
- 第二路数值清零；
- 不设置 `MEASUREMENT_VALID_PHASE`；
- `phase_deg` 清零。

从单通道切回双通道时清除未完成窗口，恢复双通道采样率和顺序采样相位补偿。

## 9. 诊断接口

`measurement_input` 诊断快照至少包括：

- 当前来源和最近请求来源；
- 当前输入状态；
- 成功切换次数；
- 停止、启动和回滚失败次数；
- 当前通道模式和 ADS8688 通道选择；
- 内部 ADC 与 ADS8688 的配置采样率；
- 最近底层错误状态；
- 因输入切换产生的 FFT 重同步次数。

ADS8688 诊断继续统计初始化失败、SPI/DMA 错误、丢失样本、历史覆盖和恢复次数，
并补充当前运行状态、DMA 半区处理计数、单/双通道模式和有效采样率。

## 10. 验证

主机测试覆盖：

- ADS8688 命令和程序寄存器帧构造；
- 寄存器读写、读回校验和失败回滚；
- 五种量程的原始码电压换算；
- AIN0/AIN1 及其他有效双通道选择的顺序配对；
- 单通道样本提交；
- 内部 ADC 与 ADS8688 动态切换；
- 目标来源启动失败后的原来源恢复；
- 输入切换时 FFT 未完成窗口清除；
- 600 kSPS、约 242.42 kSPS 和约 484.85 kSPS 下的 FFT 频率计算；
- ADS8688 双通道时间偏移相位补偿；
- 单通道第二路和相位无效化。

工程验证覆盖：

- STM32CubeIDE Debug 工程完整编译；
- 检查全部用户 `.c/.h` 位于 `Core/User/`；
- 检查 `main.c` 用户初始化区只调用 `system_init()`；
- 检查主循环用户区只调用 `system_process()`；
- 检查每个中断回调只写一个目的明确的标志；
- 检查中文模块说明、函数注释和 GPIO 映射；
- 更新 README 中的引脚、CubeMX、动态选择、量程和限制说明。

硬件验证覆盖：

- 示波器检查 PA15 CS 与 PC10 SCLK；
- 每个 ADS8688 帧包含 32 个 SCLK；
- 相邻帧之间 CS 确实回到高电平且高电平不少于 30 ns；
- 若当前 1 周期帧间空闲不能满足 CS 时序，硬件配置必须改为 2 周期并重新计算
  实际采样率；
- 通过寄存器读回确认 AIN0/AIN1 默认 ±5.12 V；
- 通过已知直流和正弦输入验证电压、频率和 THD；
- 向两路输入同相信号验证顺序采样相位补偿；
- 在采集过程中往返切换两种输入源，确认没有混合窗口或失控 DMA。

## 11. 文档、版本管理与范围限制

实现完成后更新 README，并只提交本次相关源码、测试、链接脚本和文档。保留用户
当前其他未提交改动，不覆盖或提交无关文件。

本次不实现 DDS2 到 SPI6 的迁移，不直接修改 `.ioc` 或 CubeMX 生成初始化代码，
不推送远程仓库。完成代码、测试、编译和 README 后等待用户确认。
