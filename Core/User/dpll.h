/**
 * @file dpll.h
 * @brief 二阶数字锁相环、粗频率捕获和 I/Q 相位检测接口。
 *
 * 模块用途：用插值上升过零完成粗频率捕获，并用 I/Q 相关检测相位。
 * GPIO 引脚：无直接 GPIO 引脚。
 * 依赖外设：无；输入为 ADC 采样数组，输出交给 DAC 缓冲区。
 * 初始化方法：先调用 nco_init()，再调用 dpll_init()。
 * 调用方法：每个 ADC 半缓冲调用 dpll_process_block()，随后可预测生成 DAC 波形。
 */

#ifndef USER_DPLL_H
#define USER_DPLL_H

#include <stddef.h>
#include <stdint.h>

typedef enum
{
    DPLL_STATE_NO_SIGNAL = 0,
    DPLL_STATE_ACQUIRING,
    DPLL_STATE_TRACKING,
    DPLL_STATE_LOCKED
} dpll_lock_state_t;

typedef struct
{
    uint64_t phase_accumulator_q32;
    uint32_t phase_increment_q32;
    uint64_t total_samples;
    double last_crossing_sample;
    float previous_raw_sample;
    float nominal_frequency_hz;
    float coarse_frequency_hz;
    float output_frequency_hz;
    float frequency_integrator_hz;
    float phase_error_rad;
    float amplitude_adc_counts;
    float offset_adc_counts;
    float loop_kp_hz_per_rad;
    float loop_ki_hz_per_rad_s;
    uint32_t lock_confirm_count;
    uint8_t previous_sample_valid;
    uint8_t crossing_valid;
    uint8_t frequency_valid;
    dpll_lock_state_t lock_state;
} dpll_t;

void dpll_init(dpll_t *dpll, float initial_frequency_hz);
void dpll_reset(dpll_t *dpll);
void dpll_process_block(dpll_t *dpll, const uint16_t *samples, size_t sample_count);
void dpll_generate_dac(const dpll_t *dpll,
                       uint16_t *dac_samples,
                       size_t sample_count,
                       uint32_t lead_samples,
                       float target_phase_deg);
float dpll_get_phase_error_deg(const dpll_t *dpll);

#endif /* USER_DPLL_H */
