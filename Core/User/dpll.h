/**
 * @file dpll.h
 * @brief FFT 基波捕获、数字锁相和连续相位 DAC 输出接口。
 *
 * 模块用途：以 FFT 确定基波，用单频 I/Q 初始化和细化频率与相位，
 *           再以限速 PI 环路缓慢拉相；施密特过零只保留为诊断量。
 * GPIO 引脚：无直接 GPIO 引脚。
 * 依赖外设：无；输入为 ADC 采样数组，输出交给 DAC DMA 缓冲区。
 * 初始化方法：先调用 nco_init()，再调用 dpll_init()。
 * 调用方法：每个 ADC 半缓冲依次调用 dpll_process_block() 和 dpll_generate_dac()。
 */

#ifndef USER_DPLL_H
#define USER_DPLL_H

#include <stddef.h>
#include <stdint.h>

#include "config.h"

typedef enum
{
    DPLL_STATE_NO_SIGNAL = 0,
    DPLL_STATE_ACQUIRING,
    DPLL_STATE_TRACKING,
    DPLL_STATE_LOCKED
} dpll_lock_state_t;

typedef struct
{
    /* 输入时间轴上的连续 NCO 相位，以及下一段 DAC 数据的连续输出相位。 */
    uint64_t phase_accumulator_q32;
    uint64_t output_phase_accumulator_q32;
    uint32_t phase_increment_q32;
    uint64_t total_samples;

    /* 32 周期同向过零测量状态。 */
    double period_window_start_sample;
    double last_crossing_sample;
    float previous_raw_sample;
    float period_estimate_samples;
    uint32_t period_count;

    /* FFT 频点的单频 I/Q 初始化器，仅保存累加量，不保存长窗样本。 */
    uint32_t validation_phase_q32[DPLL_VALIDATION_CANDIDATE_COUNT];
    uint32_t validation_increment_q32[DPLL_VALIDATION_CANDIDATE_COUNT];
    double validation_i[DPLL_VALIDATION_CANDIDATE_COUNT];
    double validation_q[DPLL_VALIDATION_CANDIDATE_COUNT];
    float validation_frequency_hz[DPLL_VALIDATION_CANDIDATE_COUNT];
    uint32_t validation_raw_count;
    uint32_t validation_target_raw_count;
    uint32_t validation_decimation_count;

    /* 跨多个 DMA 块的 I/Q 相位检测累加器。 */
    double phase_i_sum;
    double phase_q_sum;
    uint32_t phase_raw_count;
    uint32_t phase_target_raw_count;

    /* 频率、相位、幅度和直流偏置的运行估计。 */
    float candidate_frequency_hz;
    float nominal_frequency_hz;
    float coarse_frequency_hz;
    float output_frequency_hz;
    float frequency_integrator_hz;
    float fine_frequency_error_hz;
    float phase_error_rad;
    float previous_phase_error_rad;
    float amplitude_adc_counts;
    float offset_adc_counts;
    float output_envelope;

    uint32_t lock_confirm_count;
    uint32_t unlock_confirm_count;
    uint32_t no_signal_block_count;
    uint32_t below_hysteresis_count;
    uint8_t previous_sample_valid;
    uint8_t crossing_armed;
    uint8_t crossing_valid;
    uint8_t frequency_valid;
    uint8_t validation_active;
    uint8_t output_started;
    uint8_t phase_error_history_valid;
    dpll_lock_state_t lock_state;
} dpll_t;

void dpll_init(dpll_t *dpll, float initial_frequency_hz);
void dpll_reset(dpll_t *dpll);
void dpll_process_block(dpll_t *dpll,
                        const uint16_t *samples,
                        size_t sample_count,
                        float target_phase_deg);
void dpll_generate_dac(dpll_t *dpll,
                       uint16_t *dac_samples,
                       size_t sample_count,
                       uint32_t lead_samples);
float dpll_get_phase_error_deg(const dpll_t *dpll);

#endif /* USER_DPLL_H */
