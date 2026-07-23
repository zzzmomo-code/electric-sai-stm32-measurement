/**
 * @file config.h
 * @brief 数字锁相与同步波形重建的统一参数配置。
 *
 * 模块用途：集中保存采样率、DMA 缓冲区、DPLL、NCO 和串口状态输出参数。
 * GPIO 引脚：无直接 GPIO 引脚。
 * 依赖外设：TIM2、ADC1、DAC1、DMA1、USART1。
 * 初始化方法：无需单独初始化，由 system_init() 使用。
 * 调用方法：其他用户模块只引用本文件中的宏，不在运行时修改。
 */

#ifndef USER_CONFIG_H
#define USER_CONFIG_H

#define PHASE_PI_F                         (3.14159265358979323846f)
#define PHASE_TWO_PI_F                     (6.28318530717958647692f)

/* TIM2 以 240 MHz / 240 产生 1 MHz ADC/DAC 公共采样时钟。 */
#define SIGNAL_SAMPLE_RATE_HZ               (1000000.0f)
#define SIGNAL_DMA_BUFFER_SAMPLES           (4096u)
#define SIGNAL_DMA_HALF_SAMPLES             (SIGNAL_DMA_BUFFER_SAMPLES / 2u)
#define SIGNAL_DMA_CACHE_LINE_BYTES         (32u)

/* ADC 16 位到 DAC 12 位的默认电压比例；实板标定后可调整。 */
#define SIGNAL_DIRECT_GAIN_Q15              (32768u)
#define SIGNAL_DIRECT_OFFSET_ADC_COUNTS     (0)
#define SIGNAL_DAC_MID_CODE                 (2048u)
#define SIGNAL_DAC_MAX_CODE                 (4095u)

/* NCO 使用 2048 点正弦表和相邻点线性插值。 */
#define NCO_TABLE_BITS                      (11u)
#define NCO_TABLE_SIZE                      (1u << NCO_TABLE_BITS)

/* 默认覆盖 1 kHz～100 kHz 周期信号；更低频率需要增大分析块。 */
#define DPLL_DEFAULT_FREQUENCY_HZ           (10000.0f)
#define DPLL_MIN_FREQUENCY_HZ               (1000.0f)
#define DPLL_MAX_FREQUENCY_HZ               (100000.0f)
#define DPLL_LOOP_BANDWIDTH_HZ              (20.0f)
#define DPLL_DAMPING_FACTOR                 (0.70710678f)
#define DPLL_COARSE_FILTER_ALPHA            (0.20f)
#define DPLL_MEASUREMENT_FILTER_ALPHA       (0.20f)
#define DPLL_MIN_AMPLITUDE_ADC_COUNTS       (256.0f)
#define DPLL_LOCK_PHASE_THRESHOLD_DEG       (4.0f)
#define DPLL_UNLOCK_PHASE_THRESHOLD_DEG     (20.0f)
#define DPLL_LOCK_CONFIRM_BLOCKS            (12u)
#define DPLL_FREQUENCY_CORRECTION_LIMIT_HZ  (5000.0f)

/* 上电先用波形直通验证模拟链路，串口命令 1 切换到 DPLL。 */
#define SIGNAL_DEFAULT_TARGET_PHASE_DEG     (0.0f)
#define UART_DEBUG_TX_BUFFER_SIZE           (256u)

#if ((SIGNAL_DMA_BUFFER_SAMPLES % 2u) != 0u)
#error "SIGNAL_DMA_BUFFER_SAMPLES 必须是偶数"
#endif

#if (((SIGNAL_DMA_HALF_SAMPLES * 2u) % SIGNAL_DMA_CACHE_LINE_BYTES) != 0u)
#error "DMA 半缓冲区字节数必须是 32 字节的整数倍"
#endif

#endif /* USER_CONFIG_H */
