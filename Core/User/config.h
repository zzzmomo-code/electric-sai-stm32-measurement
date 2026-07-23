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
#define PHASE_LOCK_FIRMWARE_ID              "lfdpll_fundamental_v2"

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
/* 第一阶段验收范围：500 Hz～3 kHz；上限留到 5 kHz 便于调试。 */
#define DPLL_DEFAULT_FREQUENCY_HZ                 (1000.0f)
#define DPLL_MIN_FREQUENCY_HZ                     (500.0f)
#define DPLL_MAX_FREQUENCY_HZ                     (5000.0f)
#define DPLL_MEASUREMENT_FILTER_ALPHA             (0.10f)
#define DPLL_MIN_AMPLITUDE_ADC_COUNTS             (256.0f)
#define DPLL_NO_SIGNAL_CONFIRM_BLOCKS             (4u)
#define DPLL_CROSSING_HYSTERESIS_RATIO            (0.10f)
#define DPLL_CROSSING_HYSTERESIS_MIN_COUNTS       (64.0f)
#define DPLL_CROSSING_ARM_SAMPLES                 (16u)
#define DPLL_CROSSING_BLANKING_RATIO              (0.20f)
#define DPLL_CROSSING_BLANKING_MIN_SAMPLES        (80.0f)
#define DPLL_PERIOD_TOLERANCE_RATIO               (0.30f)

/* 连续 16 个同方向且周期一致的完整周期后才形成一次粗频率候选。 */
#define DPLL_ACQUISITION_PERIODS                  (16u)
#define DPLL_REACQUIRE_THRESHOLD_RATIO            (0.02f)
#define DPLL_REACQUIRE_THRESHOLD_MIN_HZ           (5.0f)
#define DPLL_NOMINAL_SLEW_LIMIT_HZ_PER_S           (10000.0f)

/* f/2、f、2f 相关比较使用抽取后的长窗，不额外保存样本数组。 */
#define DPLL_VALIDATION_DECIMATION                (16u)
#define DPLL_VALIDATION_CYCLES                    (16u)
#define DPLL_VALIDATION_MIN_RAW_SAMPLES           (8192u)
#define DPLL_VALIDATION_MAX_RAW_SAMPLES           (65536u)
/* 从低到高选择能量不低于最强候选 5% 的候选，优先保留真实基波。 */
#define DPLL_FUNDAMENTAL_ENERGY_RATIO              (0.05f)

/* I/Q 相位窗至少跨 4 个 DMA 半块、至少覆盖 8 个周期。 */
#define DPLL_PHASE_WINDOW_CYCLES                  (8u)
#define DPLL_PHASE_MIN_RAW_SAMPLES                (8192u)
#define DPLL_PHASE_MAX_RAW_SAMPLES                (32768u)

/* 捕获态慢拉相；锁定后进一步降低带宽，并限制最大拉相速度。 */
#define DPLL_CAPTURE_KP_HZ_PER_RAD                (0.45f)
#define DPLL_CAPTURE_KI_HZ_PER_RAD_S              (0.32f)
#define DPLL_LOCKED_KP_HZ_PER_RAD                 (0.05f)
#define DPLL_LOCKED_KI_HZ_PER_RAD_S               (0.004f)
#define DPLL_CAPTURE_CORRECTION_LIMIT_HZ          (0.10f)
#define DPLL_LOCKED_CORRECTION_LIMIT_HZ           (0.02f)
#define DPLL_LOCK_PHASE_THRESHOLD_DEG             (5.0f)
#define DPLL_UNLOCK_PHASE_THRESHOLD_DEG           (20.0f)
#define DPLL_LOCK_CONFIRM_WINDOWS                 (8u)
#define DPLL_UNLOCK_CONFIRM_WINDOWS               (3u)
#define DPLL_OUTPUT_RAMP_TIME_MS                  (20u)

/* 本项目以锁相为主目标，上电直接进入 DPLL；串口命令 0 可切换到直通。 */
#define SIGNAL_DEFAULT_TARGET_PHASE_DEG     (0.0f)
#define UART_DEBUG_TX_BUFFER_SIZE           (256u)

#if ((SIGNAL_DMA_BUFFER_SAMPLES % 2u) != 0u)
#error "SIGNAL_DMA_BUFFER_SAMPLES 必须是偶数"
#endif

#if (((SIGNAL_DMA_HALF_SAMPLES * 2u) % SIGNAL_DMA_CACHE_LINE_BYTES) != 0u)
#error "DMA 半缓冲区字节数必须是 32 字节的整数倍"
#endif

#endif /* USER_CONFIG_H */
