/**
 * @file dpll.c
 * @brief 二阶数字锁相环和相位预测 DAC 波形生成实现。
 *
 * 模块用途：对 ADC 块做均值/幅值测量、迟滞过零捕获、I/Q 检相和 PI 环路控制。
 * GPIO 引脚：无直接 GPIO 引脚。
 * 依赖外设：无；算法可在主机端用 PHASE_LOCK_HOST_TEST 独立测试。
 * 初始化方法：nco_init() 后调用 dpll_init()。
 * 调用方法：每个 ADC 半缓冲调用 dpll_process_block()。
 */

#include "system.h"

/**
 * @brief 将浮点值限制在给定闭区间内。
 * @param value 输入值。
 * @param lower 下限。
 * @param upper 上限。
 * @return 限幅后的值，无外部副作用。
 */
static float dpll_clamp(float value, float lower, float upper)
{
    if (value < lower)
    {
        return lower;
    }
    if (value > upper)
    {
        return upper;
    }
    return value;
}

/**
 * @brief 根据环路带宽和阻尼系数计算二阶环路增益。
 * @param dpll DPLL 状态对象。
 * @return 无；更新对象内的 PI 增益。
 */
static void dpll_update_loop_gains(dpll_t *dpll)
{
    const float natural_frequency_rad_s = PHASE_TWO_PI_F * DPLL_LOOP_BANDWIDTH_HZ;

    dpll->loop_kp_hz_per_rad =
        (DPLL_DAMPING_FACTOR * natural_frequency_rad_s) / PHASE_PI_F;
    dpll->loop_ki_hz_per_rad_s =
        (natural_frequency_rad_s * natural_frequency_rad_s) / PHASE_TWO_PI_F;
}

/**
 * @brief 初始化数字锁相环的全部状态。
 * @param dpll DPLL 状态对象。
 * @param initial_frequency_hz 初始频率，单位 Hz。
 * @return 无。
 * @note 清除锁定状态，并把输出相位置零。
 */
void dpll_init(dpll_t *dpll, float initial_frequency_hz)
{
    memset(dpll, 0, sizeof(*dpll));
    dpll->nominal_frequency_hz =
        dpll_clamp(initial_frequency_hz, DPLL_MIN_FREQUENCY_HZ, DPLL_MAX_FREQUENCY_HZ);
    dpll->coarse_frequency_hz = dpll->nominal_frequency_hz;
    dpll->output_frequency_hz = dpll->nominal_frequency_hz;
    dpll->phase_increment_q32 =
        nco_increment_from_hz(dpll->output_frequency_hz, SIGNAL_SAMPLE_RATE_HZ);
    dpll->offset_adc_counts = 32767.5f;
    dpll->last_crossing_sample = -1.0;
    dpll->lock_state = DPLL_STATE_NO_SIGNAL;
    dpll_update_loop_gains(dpll);
}

/**
 * @brief 清除当前状态并重新捕获输入。
 * @param dpll DPLL 状态对象。
 * @return 无。
 * @note 频率恢复为默认值，相位和统计量清零。
 */
void dpll_reset(dpll_t *dpll)
{
    dpll_init(dpll, DPLL_DEFAULT_FREQUENCY_HZ);
}

/**
 * @brief 处理一个 ADC 采样块并更新二阶数字锁相环。
 * @param dpll DPLL 状态对象。
 * @param samples 16 位 ADC 采样数组。
 * @param sample_count 采样点数。
 * @return 无。
 * @note 更新频率、相位、幅值、直流偏置和锁定状态。
 */
void dpll_process_block(dpll_t *dpll, const uint16_t *samples, size_t sample_count)
{
    uint64_t sum = 0u;
    double squared_sum = 0.0;
    float block_mean;
    float block_amplitude;
    float correlation_i = 0.0f;
    float correlation_q = 0.0f;
    float crossing_hysteresis;
    uint64_t phase_accumulator;
    size_t index;

    if ((samples == NULL) || (sample_count == 0u))
    {
        return;
    }

    for (index = 0u; index < sample_count; ++index)
    {
        sum += samples[index];
    }
    block_mean = (float)((double)sum / (double)sample_count);

    for (index = 0u; index < sample_count; ++index)
    {
        const double centered = (double)samples[index] - (double)block_mean;
        squared_sum += centered * centered;
    }
    block_amplitude = (float)sqrt((2.0 * squared_sum) / (double)sample_count);
    crossing_hysteresis = fmaxf(
        block_amplitude * DPLL_CROSSING_HYSTERESIS_RATIO,
        DPLL_CROSSING_HYSTERESIS_MIN_COUNTS);

    if (dpll->total_samples == 0u)
    {
        dpll->offset_adc_counts = block_mean;
        dpll->amplitude_adc_counts = block_amplitude;
    }
    else
    {
        dpll->offset_adc_counts +=
            DPLL_MEASUREMENT_FILTER_ALPHA * (block_mean - dpll->offset_adc_counts);
        dpll->amplitude_adc_counts +=
            DPLL_MEASUREMENT_FILTER_ALPHA * (block_amplitude - dpll->amplitude_adc_counts);
    }

    phase_accumulator = dpll->phase_accumulator_q32;

    if (block_amplitude < DPLL_MIN_AMPLITUDE_ADC_COUNTS)
    {
        dpll->offset_adc_counts = block_mean;
        dpll->amplitude_adc_counts = 0.0f;
        phase_accumulator += (uint64_t)dpll->phase_increment_q32 * (uint64_t)sample_count;
        dpll->phase_accumulator_q32 = phase_accumulator;
        dpll->previous_raw_sample = (float)samples[sample_count - 1u];
        dpll->previous_sample_valid = 1u;
        dpll->total_samples += sample_count;
        dpll->lock_confirm_count = 0u;
        dpll->crossing_armed = 0u;
        dpll->crossing_valid = 0u;
        dpll->frequency_valid = 0u;
        dpll->lock_state = DPLL_STATE_NO_SIGNAL;
        return;
    }

    for (index = 0u; index < sample_count; ++index)
    {
        const float raw_sample = (float)samples[index];
        const float centered_sample = raw_sample - block_mean;
        const uint32_t phase_q32 = (uint32_t)phase_accumulator;
        const float normalized_sample = centered_sample / block_amplitude;
        const float sine_value = nco_sin_q32(phase_q32);
        const float cosine_value = nco_sin_q32(phase_q32 + 0x40000000u);

        correlation_i += normalized_sample * sine_value;
        correlation_q += normalized_sample * cosine_value;

        if (centered_sample <= -crossing_hysteresis)
        {
            dpll->crossing_armed = 1u;
        }

        if ((dpll->crossing_armed != 0u) && (dpll->previous_sample_valid != 0u))
        {
            const float previous_centered = dpll->previous_raw_sample - block_mean;

            if ((previous_centered <= 0.0f) && (centered_sample > 0.0f))
            {
                const float denominator = centered_sample - previous_centered;
                const float fraction = (denominator > 0.0f)
                                     ? (-previous_centered / denominator)
                                     : 0.0f;
                const double previous_index =
                    (double)dpll->total_samples + (double)index - 1.0;
                const double crossing_sample = previous_index + (double)fraction;
                if (dpll->crossing_valid != 0u)
                {
                    const double period_samples = crossing_sample - dpll->last_crossing_sample;
                    const double minimum_period =
                        (double)SIGNAL_SAMPLE_RATE_HZ / (double)DPLL_MAX_FREQUENCY_HZ;
                    const double maximum_period =
                        (double)SIGNAL_SAMPLE_RATE_HZ / (double)DPLL_MIN_FREQUENCY_HZ;

                    if ((period_samples >= minimum_period)
                        && (period_samples <= maximum_period))
                    {
                        const float measured_frequency =
                            (float)((double)SIGNAL_SAMPLE_RATE_HZ / period_samples);

                        if (dpll->frequency_valid == 0u)
                        {
                            dpll->coarse_frequency_hz = measured_frequency;
                            dpll->frequency_valid = 1u;
                        }
                        else
                        {
                            dpll->coarse_frequency_hz +=
                                DPLL_COARSE_FILTER_ALPHA
                                * (measured_frequency - dpll->coarse_frequency_hz);
                        }
                    }
                }

                dpll->last_crossing_sample = crossing_sample;
                dpll->crossing_valid = 1u;
                dpll->crossing_armed = 0u;
            }
        }

        dpll->previous_raw_sample = raw_sample;
        dpll->previous_sample_valid = 1u;
        phase_accumulator += dpll->phase_increment_q32;
    }

    dpll->phase_accumulator_q32 = phase_accumulator;
    dpll->total_samples += sample_count;

    if (dpll->frequency_valid != 0u)
    {
        if (dpll->lock_state == DPLL_STATE_NO_SIGNAL)
        {
            dpll->nominal_frequency_hz = dpll->coarse_frequency_hz;
            dpll->frequency_integrator_hz = 0.0f;
        }
        else
        {
            const float reacquire_threshold_hz = fmaxf(
                dpll->nominal_frequency_hz * DPLL_REACQUIRE_THRESHOLD_RATIO,
                DPLL_REACQUIRE_THRESHOLD_MIN_HZ);
            const float coarse_alpha =
                (fabsf(dpll->coarse_frequency_hz - dpll->output_frequency_hz)
                 > reacquire_threshold_hz)
                ? DPLL_COARSE_FILTER_ALPHA
                : DPLL_TRACKING_COARSE_ALPHA;

            dpll->nominal_frequency_hz +=
                coarse_alpha
                * (dpll->coarse_frequency_hz - dpll->nominal_frequency_hz);
        }
    }

    dpll->phase_error_rad = atan2f(correlation_q, correlation_i);

    {
        const float block_time_s = (float)sample_count / SIGNAL_SAMPLE_RATE_HZ;
        const float correction_limit = dpll_clamp(
            0.25f * dpll->nominal_frequency_hz,
            10.0f,
            DPLL_FREQUENCY_CORRECTION_LIMIT_HZ);
        float commanded_frequency;
        const float absolute_phase_error = fabsf(dpll->phase_error_rad);
        const float lock_threshold =
            DPLL_LOCK_PHASE_THRESHOLD_DEG * PHASE_PI_F / 180.0f;
        const float unlock_threshold =
            DPLL_UNLOCK_PHASE_THRESHOLD_DEG * PHASE_PI_F / 180.0f;

        dpll->frequency_integrator_hz +=
            dpll->loop_ki_hz_per_rad_s * block_time_s * dpll->phase_error_rad;
        dpll->frequency_integrator_hz = dpll_clamp(
            dpll->frequency_integrator_hz, -correction_limit, correction_limit);

        commanded_frequency =
            dpll->nominal_frequency_hz
            + dpll->frequency_integrator_hz
            + (dpll->loop_kp_hz_per_rad * dpll->phase_error_rad);
        dpll->output_frequency_hz = dpll_clamp(
            commanded_frequency, DPLL_MIN_FREQUENCY_HZ, DPLL_MAX_FREQUENCY_HZ);
        dpll->phase_increment_q32 =
            nco_increment_from_hz(dpll->output_frequency_hz, SIGNAL_SAMPLE_RATE_HZ);

        if ((dpll->frequency_valid != 0u) && (absolute_phase_error <= lock_threshold))
        {
            if (dpll->lock_confirm_count < DPLL_LOCK_CONFIRM_BLOCKS)
            {
                ++dpll->lock_confirm_count;
            }
            dpll->lock_state =
                (dpll->lock_confirm_count >= DPLL_LOCK_CONFIRM_BLOCKS)
                ? DPLL_STATE_LOCKED
                : DPLL_STATE_TRACKING;
        }
        else
        {
            if (absolute_phase_error >= unlock_threshold)
            {
                dpll->lock_confirm_count = 0u;
            }
            dpll->lock_state =
                (dpll->frequency_valid != 0u)
                ? DPLL_STATE_TRACKING
                : DPLL_STATE_ACQUIRING;
        }
    }
}

/**
 * @brief 根据已跟踪相位预测未来时刻并生成 DAC 正弦波。
 * @param dpll DPLL 状态对象。
 * @param dac_samples DAC 12 位右对齐输出数组。
 * @param sample_count 需要生成的采样点数。
 * @param lead_samples 从当前输入块末端到实际 DAC 播放起点的预测点数。
 * @param target_phase_deg 输出相对输入的目标相位，单位度。
 * @return 无。
 * @note 不修改 DPLL 输入时间轴，只在局部变量中预测未来相位。
 */
void dpll_generate_dac(const dpll_t *dpll,
                       uint16_t *dac_samples,
                       size_t sample_count,
                       uint32_t lead_samples,
                       float target_phase_deg)
{
    uint64_t output_phase;
    float dac_offset;
    float dac_amplitude;
    float maximum_amplitude;
    size_t index;

    if ((dac_samples == NULL) || (sample_count == 0u))
    {
        return;
    }

    output_phase =
        dpll->phase_accumulator_q32
        + ((uint64_t)dpll->phase_increment_q32 * (uint64_t)lead_samples)
        + (uint64_t)nco_phase_offset_from_deg(target_phase_deg);

    dac_offset = dpll_clamp(dpll->offset_adc_counts / 16.0f, 0.0f, 4095.0f);
    dac_amplitude = dpll->amplitude_adc_counts / 16.0f;
    maximum_amplitude = fminf(dac_offset, 4095.0f - dac_offset);
    dac_amplitude = dpll_clamp(dac_amplitude, 0.0f, maximum_amplitude);

    for (index = 0u; index < sample_count; ++index)
    {
        float output_code =
            dac_offset + (dac_amplitude * nco_sin_q32((uint32_t)output_phase));

        output_code = dpll_clamp(output_code, 0.0f, 4095.0f);
        dac_samples[index] = (uint16_t)(output_code + 0.5f);
        output_phase += dpll->phase_increment_q32;
    }
}

/**
 * @brief 获取当前相位误差。
 * @param dpll DPLL 状态对象。
 * @return 相位误差，单位度，范围约为 -180～180。
 * @note 只读访问，不改变锁相状态。
 */
float dpll_get_phase_error_deg(const dpll_t *dpll)
{
    return dpll->phase_error_rad * 180.0f / PHASE_PI_F;
}
