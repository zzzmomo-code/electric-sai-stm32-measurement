#ifndef SIGNAL_SEPARATION_CONFIG_H
#define SIGNAL_SEPARATION_CONFIG_H

/*
 * 模块用途：STM32H743 双信号识别、数字锁相和双路 DAC 再生参数。
 * GPIO 映射：PC0=ADC1_INP10，PA4=DAC1_OUT1，PA5=DAC1_OUT2。
 * 外设依赖：ADC1、DAC1 CH1/CH2、TIM2 TRGO、DMA1 Stream0/1/2。
 * 初始化方法：由 signal_separation_start() 自动初始化并启动。
 * 调用方法：主循环持续调用 signal_separation_process()。
 */

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

/* 谐波法波形分类参数。 */
#define SIGSEP_MIN_VALID_ADC_AMP               120.0f
#define SIGSEP_TRI_H3_RATIO                    0.060f
#define SIGSEP_TRI_H5_RATIO                    0.025f

/* Q32 NCO/PLL 参数，沿用 2023H 原工程的环路带宽设计。 */
#define SIGSEP_PLL_PHASE_KP_SHIFT              1U
#define SIGSEP_PLL_STEP_KP_DIV                 20U
#define SIGSEP_PLL_STEP_KI_DIV                 200U
#define SIGSEP_PLL_MAX_CORR_DIV                2000U
#define SIGSEP_PLL_INTEGRATOR_LIMIT            8589934592LL
#define SIGSEP_PLL_INTEGRATOR_LEAK_NUM         65535U
#define SIGSEP_AMP_SMOOTH_SHIFT                3U

/* DAC2 运行时额外相位偏移，当前默认 150°，按 5°量化。 */
#define SIGSEP_PHASE_OFFSET_DEFAULT_DEG        150
#define SIGSEP_PHASE_OFFSET_MIN_DEG            0
#define SIGSEP_PHASE_OFFSET_MAX_DEG            180
#define SIGSEP_PHASE_OFFSET_STEP_DEG           5

#endif
