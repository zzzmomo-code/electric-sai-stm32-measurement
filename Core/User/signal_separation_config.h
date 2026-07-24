#ifndef SIGNAL_SEPARATION_CONFIG_H
#define SIGNAL_SEPARATION_CONFIG_H

/*
 * 模块用途：STM32H743 单/双信号识别、数字锁相和 DAC 再生参数。
 * GPIO 映射：PC0=ADC1_INP10，PA4=DAC1_OUT1，PA5=DAC1_OUT2。
 * 外设依赖：ADC1、DAC1 CH1/CH2、TIM2 TRGO、DMA1 Stream0/1/2。
 * 初始化方法：由 signal_separation_start() 自动初始化并启动。
 * 调用方法：主循环持续调用 signal_separation_process()。
 */

/*
 * 编译期工作模式，只需要修改 SIGSEP_OPERATION_MODE 后重新编译下载。
 *
 * single：
 *   PC0 输入一路正弦波、方波或三角波，PA4 重建并锁相输出，PA5 保持中点。
 * dual_mixed：
 *   PC0 输入两路信号的模拟叠加，PA4/PA5 分别重建低频/高频分量。
 */
#define SIGSEP_MODE_DUAL_MIXED                 1U
#define SIGSEP_MODE_SINGLE                     2U
#ifndef SIGSEP_OPERATION_MODE
#define SIGSEP_OPERATION_MODE                  SIGSEP_MODE_DUAL_MIXED
#endif

/* TIM2 同时触发 ADC 和两路 DAC，采样率为 2.5 MSPS。 */
#define SIGSEP_SAMPLE_RATE_HZ                  2500000U

/*
 * 每帧只分析前 500 点，维持 2.5 MHz / 500 = 5 kHz 的正交频点间隔。
 * DMA 半区使用 512 点，使每个半区恰好占 1024 字节并与 D-Cache 行对齐。
 */
#define SIGSEP_ANALYSIS_FRAME_LEN              500U
#define SIGSEP_ADC_DMA_HALF_LEN                512U
#define SIGSEP_ADC_DMA_LEN                     (SIGSEP_ADC_DMA_HALF_LEN * 2U)

/*
 * DAC 每半区 1024 点、每通道共 2048 点。每个半区和整个缓冲区均为
 * 32 字节整数倍，便于在 M7 开启 D-Cache 时安全地按半区清理缓存。
 */
#define SIGSEP_DAC_DMA_HALF_LEN                1024U
#define SIGSEP_DAC_DMA_LEN                     (SIGSEP_DAC_DMA_HALF_LEN * 2U)

#define SIGSEP_SINE_LUT_BITS                   10U
#define SIGSEP_SINE_LUT_SIZE                   (1U << SIGSEP_SINE_LUT_BITS)
#define SIGSEP_SINE_LUT_SHIFT                  (32U - SIGSEP_SINE_LUT_BITS)

/* 输入只搜索 10 kHz～100 kHz，候选间隔 5 kHz，共 19 个频点。 */
#define SIGSEP_FREQ_MIN_HZ                     10000U
#define SIGSEP_FREQ_STEP_HZ                    5000U
#define SIGSEP_FREQ_COUNT                      19U
#define SIGSEP_MAX_BIN                         100U
#define SIGSEP_IDENTIFY_FRAMES                 4U

/* 16 位 ADC 幅值到 12 位 DAC 幅值的换算和安全限幅。 */
#define SIGSEP_DAC_MID                         2048U
#define SIGSEP_DAC_MAX                         4095U
#define SIGSEP_ADC_TO_DAC_SCALE                (4095.0f / 65535.0f)
#define SIGSEP_DEFAULT_DAC_AMP                 700.0f
#define SIGSEP_MAX_DAC_AMP                     1850.0f

/* 两路输出的固定相位偏置，正值表示输出超前。 */
#define SIGSEP_DAC1_PHASE_OFFSET_DEG           0
#define SIGSEP_DAC2_PHASE_OFFSET_DEG           0

/*
 * 两分量来自同一相干源时，仅让通道 0 的 PLL 作为主环路，通道 1 按
 * 频率比例跟随主环路的时间误差，避免两个环路各自漂移。
 */
#define SIGSEP_COMMON_SOURCE_LOCK              1U
#define SIGSEP_PHASE_MASTER_CH                 0U

/*
 * 识别有效性和谐波法波形分类参数。
 * 双信号模式除固定门限外，还要求较弱分量至少达到较强分量的 5%，避免把
 * 单音输入产生的频谱泄漏或底噪误当成第二路信号。
 */
#define SIGSEP_MIN_VALID_ADC_AMP               120.0f
#define SIGSEP_DUAL_MIN_SECOND_RATIO           0.050f
#define SIGSEP_TRI_H3_RATIO                    0.060f
#define SIGSEP_TRI_H5_RATIO                    0.025f
#define SIGSEP_SQUARE_H3_RATIO                 0.220f
#define SIGSEP_SQUARE_H5_RATIO                 0.120f

/*
 * Q32 NCO/PLL 参数。
 * H743 当前使用 HSI，器件初始频差和温漂明显大于原 HSE 工程，因此把最大步进修正
 * 从 ±0.05% 扩大到 ±2%。积分限幅同时扩大，避免长期频差只能靠静态相位误差维持。
 */
#define SIGSEP_PLL_PHASE_KP_SHIFT              1U
#define SIGSEP_PLL_STEP_KP_DIV                 20U
#define SIGSEP_PLL_STEP_KI_DIV                 200U
#define SIGSEP_PLL_MAX_CORR_DIV                50U
#define SIGSEP_PLL_INTEGRATOR_LIMIT            1099511627776LL
#define SIGSEP_PLL_INTEGRATOR_LEAK_NUM         65535U
#define SIGSEP_AMP_SMOOTH_SHIFT                3U

/* DAC2 运行时额外相位偏移，当前默认 150°，按 5°量化。 */
#define SIGSEP_PHASE_OFFSET_DEFAULT_DEG        150
#define SIGSEP_PHASE_OFFSET_MIN_DEG            0
#define SIGSEP_PHASE_OFFSET_MAX_DEG            180
#define SIGSEP_PHASE_OFFSET_STEP_DEG           5

#endif
