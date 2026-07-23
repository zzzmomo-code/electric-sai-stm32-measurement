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
#define PHASE_LOCK_FIRMWARE_ID              "fft32768_dpll_v4"

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

/*
 * 1 MHz ADC 先按 64 点平均抽取到 15.625 kS/s，再做 32768 点 FFT。
 * 捕获窗约 2.097 s，频点间隔约 0.477 Hz；三点插值提供亚频点初值。
 * 两个 float 数组共占 256 KiB，适配 H743 的 512 KiB D1 SRAM。
 */
#define FFT_ANALYZER_BITS                         (15u)
#define FFT_ANALYZER_SIZE                         (1u << FFT_ANALYZER_BITS)
#define FFT_ANALYZER_DECIMATION                   (64u)
#define FFT_ANALYZER_SAMPLE_RATE_HZ               \
    (SIGNAL_SAMPLE_RATE_HZ / (float)FFT_ANALYZER_DECIMATION)
#define FFT_ANALYZER_WORK_BUDGET                  (2048u)
#define FFT_ANALYZER_FUNDAMENTAL_ENERGY_RATIO     (0.05f)

/* 第一阶段验收范围：500 Hz～4 kHz；上限留到 5 kHz 便于调试。 */
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

/* 过零诊断连续统计 32 个同方向且周期一致的完整周期。 */
#define DPLL_ACQUISITION_PERIODS                  (32u)
#define DPLL_REACQUIRE_THRESHOLD_RATIO            (0.02f)
#define DPLL_REACQUIRE_THRESHOLD_MIN_HZ           (5.0f)
#define DPLL_NOMINAL_SLEW_LIMIT_HZ_PER_S           (10000.0f)

/* FFT 得到频率后，用单频长窗 I/Q 初始化相位并继续细化。 */
#define DPLL_VALIDATION_CANDIDATE_COUNT           (1u)
#define DPLL_VALIDATION_DECIMATION                (16u)
#define DPLL_VALIDATION_CYCLES                    (16u)
#define DPLL_VALIDATION_MIN_RAW_SAMPLES           (8192u)
#define DPLL_VALIDATION_MAX_RAW_SAMPLES           (65536u)

/*
 * I/Q 相位窗优先覆盖 16 个完整周期，减少非整周期截断造成的二倍频
 * 混频泄漏；只设置一个 DMA 半块的下限，不再强行改变窗口周期数。
 */
#define DPLL_PHASE_WINDOW_CYCLES                  (16u)
#define DPLL_PHASE_MIN_RAW_SAMPLES                (2048u)
#define DPLL_PHASE_MAX_RAW_SAMPLES                (32768u)
#define DPLL_FINE_FREQUENCY_ALPHA                 (0.25f)
#define DPLL_FINE_FREQUENCY_STEP_LIMIT_HZ         (0.005f)
#define DPLL_LOCK_FREQUENCY_THRESHOLD_HZ          (0.02f)

/* 捕获态慢拉相；锁定后进一步降低带宽，并限制最大拉相速度。 */
#define DPLL_CAPTURE_KP_HZ_PER_RAD                (0.45f)
#define DPLL_CAPTURE_KI_HZ_PER_RAD_S              (0.32f)
#define DPLL_LOCKED_KP_HZ_PER_RAD                 (0.05f)
#define DPLL_LOCKED_KI_HZ_PER_RAD_S               (0.004f)
#define DPLL_CAPTURE_CORRECTION_LIMIT_HZ          (0.10f)
#define DPLL_LOCKED_CORRECTION_LIMIT_HZ           (0.10f)
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
