/**
 * @file dpll.c
 * @brief 低频连续相位数字锁相环实现。
 *
 * 模块用途：完成长周期粗测频、f/2/f/2f 谐波判别、跨 DMA 块 I/Q 检相和慢速拉相。
 * GPIO 引脚：无直接 GPIO 引脚。
 * 依赖外设：无；采样率由 config.h 中 SIGNAL_SAMPLE_RATE_HZ 指定。
 * 初始化方法：nco_init() 后调用 dpll_init()。
 * 调用方法：每个 ADC 半缓冲先处理输入，再连续生成下一段 DAC 数据。
 */

#include "dpll.h"

#include <math.h>
#include <string.h>

#include "config.h"
#include "fft_analyzer.h"
#include "nco.h"

#define DPLL_QUARTER_TURN_Q32 (0x40000000u)

/**
 * @brief 将浮点值限制到闭区间。
 * @param value 待限制值。
 * @param lower 下限。
 * @param upper 上限。
 * @return 限制后的值。
 * @note 无副作用。
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
 * @brief 将相位角归一化到 -π～π。
 * @param phase_rad 原始相位角。
 * @return 归一化相位角。
 * @note 无副作用。
 */
static float dpll_wrap_phase(float phase_rad)
{
    while (phase_rad > PHASE_PI_F)
    {
        phase_rad -= PHASE_TWO_PI_F;
    }
    while (phase_rad < -PHASE_PI_F)
    {
        phase_rad += PHASE_TWO_PI_F;
    }
    return phase_rad;
}

/**
 * @brief 将采样点数限制到配置窗口范围。
 * @param value 原始点数。
 * @param lower 最小点数。
 * @param upper 最大点数。
 * @return 限制后的点数。
 * @note 无副作用。
 */
static uint32_t dpll_clamp_samples(uint32_t value, uint32_t lower, uint32_t upper)
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
 * @brief 按当前输出频率更新 NCO 相位增量。
 * @param dpll DPLL 状态对象。
 * @return 无。
 * @note 只修改 phase_increment_q32。
 */
static void dpll_update_phase_increment(dpll_t *dpll)
{
    dpll->output_frequency_hz =
        dpll_clamp(dpll->output_frequency_hz,
                   DPLL_MIN_FREQUENCY_HZ,
                   DPLL_MAX_FREQUENCY_HZ);
    dpll->phase_increment_q32 =
        nco_increment_from_hz(dpll->output_frequency_hz, SIGNAL_SAMPLE_RATE_HZ);
}

/**
 * @brief 根据当前频率设置跨块 I/Q 检相窗口。
 * @param dpll DPLL 状态对象。
 * @return 无。
 * @note 清空上一窗口累加量。
 */
static void dpll_reset_phase_window(dpll_t *dpll)
{
    float frequency_hz = dpll->nominal_frequency_hz;
    uint32_t raw_samples;

    frequency_hz = dpll_clamp(frequency_hz,
                              DPLL_MIN_FREQUENCY_HZ,
                              DPLL_MAX_FREQUENCY_HZ);
    raw_samples =
        (uint32_t)(((float)DPLL_PHASE_WINDOW_CYCLES * SIGNAL_SAMPLE_RATE_HZ
                    / frequency_hz) + 0.5f);
    dpll->phase_target_raw_count =
        dpll_clamp_samples(raw_samples,
                           DPLL_PHASE_MIN_RAW_SAMPLES,
                           DPLL_PHASE_MAX_RAW_SAMPLES);
    dpll->phase_i_sum = 0.0;
    dpll->phase_q_sum = 0.0;
    dpll->phase_raw_count = 0u;
}

/**
 * @brief 清除粗测频与相关验证的临时状态。
 * @param dpll DPLL 状态对象。
 * @return 无。
 * @note 不改写连续输出相位。
 */
static void dpll_reset_acquisition(dpll_t *dpll)
{
    uint32_t candidate_index;

    dpll->period_count = 0u;
    dpll->period_estimate_samples = 0.0f;
    dpll->below_hysteresis_count = 0u;
    dpll->crossing_valid = 0u;
    dpll->crossing_armed = 0u;
    dpll->validation_active = 0u;
    dpll->phase_error_history_valid = 0u;
    dpll->fine_frequency_error_hz = 0.0f;
    dpll->validation_raw_count = 0u;
    dpll->validation_decimation_count = 0u;

    for (candidate_index = 0u;
         candidate_index < DPLL_VALIDATION_CANDIDATE_COUNT;
         ++candidate_index)
    {
        dpll->validation_i[candidate_index] = 0.0;
        dpll->validation_q[candidate_index] = 0.0;
    }
}

/**
 * @brief 对 FFT 基波结果启动单频长窗 I/Q 相位初始化。
 * @param dpll DPLL 状态对象。
 * @param measured_frequency_hz FFT 插值得到的基波频率。
 * @return 无。
 * @note 验证过程只累加 I/Q，不申请长窗数组。
 */
static void dpll_start_validation(dpll_t *dpll, float measured_frequency_hz)
{
    uint32_t candidate_index;
    uint32_t target_samples;

    dpll->candidate_frequency_hz = measured_frequency_hz;
    dpll->validation_frequency_hz[0] = measured_frequency_hz;

    for (candidate_index = 0u;
         candidate_index < DPLL_VALIDATION_CANDIDATE_COUNT;
         ++candidate_index)
    {
        const float candidate_hz = dpll->validation_frequency_hz[candidate_index];

        dpll->validation_phase_q32[candidate_index] = 0u;
        dpll->validation_i[candidate_index] = 0.0;
        dpll->validation_q[candidate_index] = 0.0;

        dpll->validation_increment_q32[candidate_index] =
            nco_increment_from_hz(candidate_hz, SIGNAL_SAMPLE_RATE_HZ);
    }

    target_samples =
        (uint32_t)(((float)DPLL_VALIDATION_CYCLES * SIGNAL_SAMPLE_RATE_HZ
                    / measured_frequency_hz) + 0.5f);
    dpll->validation_target_raw_count =
        dpll_clamp_samples(target_samples,
                           DPLL_VALIDATION_MIN_RAW_SAMPLES,
                           DPLL_VALIDATION_MAX_RAW_SAMPLES);
    dpll->validation_raw_count = 0u;
    dpll->validation_decimation_count = 0u;
    dpll->validation_active = 1u;

    if (dpll->frequency_valid == 0u)
    {
        dpll->lock_state = DPLL_STATE_ACQUIRING;
    }
}

/**
 * @brief 接受谐波判别结果并更新粗频率。
 * @param dpll DPLL 状态对象。
 * @param winner_index 最强相关能量候选下标。
 * @param target_phase_deg 输出相对输入的目标相位。
 * @return 无。
 * @note 首次捕获可在静音期间设置初始相位；运行后绝不硬改输出相位。
 */
static void dpll_accept_validation(dpll_t *dpll,
                                   uint32_t winner_index,
                                   float target_phase_deg)
{
    const float accepted_frequency_hz =
        dpll->validation_frequency_hz[winner_index];
    const uint8_t first_acquisition = (uint8_t)(dpll->frequency_valid == 0u);
    const float previous_coarse_hz = dpll->coarse_frequency_hz;

    if (first_acquisition != 0u)
    {
        const float input_phase_rad =
            atan2f((float)dpll->validation_q[winner_index],
                   (float)dpll->validation_i[winner_index]);
        const uint32_t input_phase_offset_q32 =
            nco_phase_offset_from_deg(input_phase_rad * 180.0f / PHASE_PI_F);
        const uint32_t target_phase_offset_q32 =
            nco_phase_offset_from_deg(target_phase_deg);
        const uint32_t aligned_phase_q32 =
            dpll->validation_phase_q32[winner_index]
            + input_phase_offset_q32
            + target_phase_offset_q32;

        dpll->phase_accumulator_q32 =
            (dpll->phase_accumulator_q32 & 0xFFFFFFFF00000000ULL)
            | (uint64_t)aligned_phase_q32;
        dpll->nominal_frequency_hz = accepted_frequency_hz;
        dpll->output_frequency_hz = accepted_frequency_hz;
        dpll->frequency_integrator_hz = 0.0f;
        dpll->phase_error_history_valid = 0u;
        dpll->frequency_valid = 1u;
        dpll->lock_state = DPLL_STATE_TRACKING;
        dpll_update_phase_increment(dpll);
        dpll_reset_phase_window(dpll);
    }
    else if (fabsf(accepted_frequency_hz - previous_coarse_hz)
             > fmaxf(previous_coarse_hz * DPLL_REACQUIRE_THRESHOLD_RATIO,
                     DPLL_REACQUIRE_THRESHOLD_MIN_HZ))
    {
        dpll->lock_state = DPLL_STATE_TRACKING;
        dpll->lock_confirm_count = 0u;
        dpll->unlock_confirm_count = 0u;
        dpll->frequency_integrator_hz = 0.0f;
        dpll->phase_error_history_valid = 0u;
        dpll_reset_phase_window(dpll);
    }

    dpll->coarse_frequency_hz = accepted_frequency_hz;
    dpll->validation_active = 0u;
}

/**
 * @brief 对一个采样点更新 FFT 频点的单频 I/Q 初始化器。
 * @param dpll DPLL 状态对象。
 * @param centered_sample 已去直流的 ADC 采样值。
 * @param target_phase_deg 输出相对输入的目标相位。
 * @return 无。
 * @note 每 16 点才调用三角查表，所有候选相位仍逐点连续推进。
 */
static void dpll_process_validation_sample(dpll_t *dpll,
                                           float centered_sample,
                                           float target_phase_deg)
{
    uint32_t candidate_index;

    if (dpll->validation_decimation_count == 0u)
    {
        for (candidate_index = 0u;
             candidate_index < DPLL_VALIDATION_CANDIDATE_COUNT;
             ++candidate_index)
        {
            if (dpll->validation_increment_q32[candidate_index] != 0u)
            {
                const uint32_t phase_q32 =
                    dpll->validation_phase_q32[candidate_index];
                const float sine_value = nco_sin_q32(phase_q32);
                const float cosine_value =
                    nco_sin_q32(phase_q32 + DPLL_QUARTER_TURN_Q32);

                dpll->validation_i[candidate_index] +=
                    (double)(centered_sample * sine_value);
                dpll->validation_q[candidate_index] +=
                    (double)(centered_sample * cosine_value);
            }
        }
    }

    ++dpll->validation_raw_count;
    ++dpll->validation_decimation_count;
    if (dpll->validation_decimation_count >= DPLL_VALIDATION_DECIMATION)
    {
        dpll->validation_decimation_count = 0u;
    }

    if (dpll->validation_raw_count >= dpll->validation_target_raw_count)
    {
        dpll_accept_validation(dpll, 0u, target_phase_deg);
        return;
    }

    for (candidate_index = 0u;
         candidate_index < DPLL_VALIDATION_CANDIDATE_COUNT;
         ++candidate_index)
    {
        dpll->validation_phase_q32[candidate_index] +=
            dpll->validation_increment_q32[candidate_index];
    }
}

/**
 * @brief 处理一次 FFT 基波频率结果。
 * @param dpll DPLL 状态对象。
 * @param measured_frequency_hz FFT 插值基波频率。
 * @return 无。
 * @note 大变化重新做单频 I/Q 初始化，小漂移由相位斜率细调。
 */
static void dpll_handle_fft_frequency(dpll_t *dpll,
                                      float measured_frequency_hz)
{
    const float fft_bin_width_hz =
        FFT_ANALYZER_SAMPLE_RATE_HZ / (float)FFT_ANALYZER_SIZE;
    float reacquire_threshold_hz;

    /*
     * 边界频点经窗函数插值后可能略越过量程，例如 500 Hz 得到
     * 499.94 Hz。允许一个 FFT 频点的数值余量，再钳位回有效范围。
     */
    if ((measured_frequency_hz
         < (DPLL_MIN_FREQUENCY_HZ - fft_bin_width_hz))
        || (measured_frequency_hz
            > (DPLL_MAX_FREQUENCY_HZ + fft_bin_width_hz)))
    {
        return;
    }
    measured_frequency_hz =
        dpll_clamp(measured_frequency_hz,
                   DPLL_MIN_FREQUENCY_HZ,
                   DPLL_MAX_FREQUENCY_HZ);

    if (dpll->frequency_valid == 0u)
    {
        if (dpll->validation_active == 0u)
        {
            dpll_start_validation(dpll, measured_frequency_hz);
        }
        return;
    }

    reacquire_threshold_hz =
        fmaxf(dpll->coarse_frequency_hz * DPLL_REACQUIRE_THRESHOLD_RATIO,
              DPLL_REACQUIRE_THRESHOLD_MIN_HZ);
    if (fabsf(measured_frequency_hz - dpll->coarse_frequency_hz)
        > reacquire_threshold_hz)
    {
        if (dpll->validation_active == 0u)
        {
            dpll->frequency_valid = 0u;
            dpll->lock_state = DPLL_STATE_ACQUIRING;
            dpll->frequency_integrator_hz = 0.0f;
            dpll->lock_confirm_count = 0u;
            dpll->unlock_confirm_count = 0u;
            dpll->phase_error_history_valid = 0u;
            dpll_reset_phase_window(dpll);
            dpll_start_validation(dpll, measured_frequency_hz);
        }
    }
}

/**
 * @brief 用一个上升过零位置更新 32 周期测频器。
 * @param dpll DPLL 状态对象。
 * @param crossing_sample 带线性插值的小数采样位置。
 * @return 无。
 * @note 只接收同方向上升过零。
 */
static void dpll_record_rising_crossing(dpll_t *dpll, double crossing_sample)
{
    if (dpll->crossing_valid == 0u)
    {
        dpll->period_window_start_sample = crossing_sample;
        dpll->last_crossing_sample = crossing_sample;
        dpll->period_count = 0u;
        dpll->period_estimate_samples = 0.0f;
        dpll->crossing_valid = 1u;
        return;
    }

    {
        const double interval_samples =
            crossing_sample - dpll->last_crossing_sample;
        const float interval_frequency_hz =
            (float)((double)SIGNAL_SAMPLE_RATE_HZ / interval_samples);
        float blanking_samples = DPLL_CROSSING_BLANKING_MIN_SAMPLES;

        if (dpll->frequency_valid != 0u)
        {
            blanking_samples =
                fmaxf(blanking_samples,
                      DPLL_CROSSING_BLANKING_RATIO
                      * SIGNAL_SAMPLE_RATE_HZ
                      / dpll->coarse_frequency_hz);
        }

        if (interval_samples < (double)blanking_samples)
        {
            return;
        }

        if ((interval_samples <= 0.0)
            || (interval_frequency_hz < (DPLL_MIN_FREQUENCY_HZ * 0.5f)))
        {
            dpll->period_window_start_sample = crossing_sample;
            dpll->period_count = 0u;
            dpll->period_estimate_samples = 0.0f;
        }
        else if (interval_frequency_hz > (DPLL_MAX_FREQUENCY_HZ * 2.0f))
        {
            return;
        }
        else
        {
            if (dpll->period_count == 0u)
            {
                dpll->period_estimate_samples = (float)interval_samples;
                dpll->period_count = 1u;
            }
            else if (fabsf((float)interval_samples
                           - dpll->period_estimate_samples)
                     <= (dpll->period_estimate_samples
                         * DPLL_PERIOD_TOLERANCE_RATIO))
            {
                dpll->period_estimate_samples +=
                    0.20f
                    * ((float)interval_samples
                       - dpll->period_estimate_samples);
                ++dpll->period_count;
            }
            else
            {
                dpll->period_window_start_sample = crossing_sample;
                dpll->period_count = 0u;
                dpll->period_estimate_samples = 0.0f;
            }

            if (dpll->period_count >= DPLL_ACQUISITION_PERIODS)
            {
                const double measured_samples =
                    crossing_sample - dpll->period_window_start_sample;
                const float measured_frequency_hz =
                    (float)(((double)DPLL_ACQUISITION_PERIODS
                             * (double)SIGNAL_SAMPLE_RATE_HZ)
                            / measured_samples);

                /* 过零结果只作诊断，绝不再驱动输出频率。 */
                dpll->candidate_frequency_hz = measured_frequency_hz;
                dpll->period_window_start_sample = crossing_sample;
                dpll->period_count = 0u;
                dpll->period_estimate_samples = 0.0f;
            }
        }

        dpll->last_crossing_sample = crossing_sample;
    }
}

/**
 * @brief 完成一个跨块 I/Q 窗口并更新限速 PI 环路。
 * @param dpll DPLL 状态对象。
 * @return 无。
 * @note 带输出限幅和条件积分，锁定态自动降低环路带宽。
 */
static void dpll_finish_phase_window(dpll_t *dpll)
{
    const float window_seconds =
        (float)dpll->phase_raw_count / SIGNAL_SAMPLE_RATE_HZ;
    const float phase_error_rad =
        atan2f((float)dpll->phase_q_sum, (float)dpll->phase_i_sum);
    const float absolute_error_rad = fabsf(phase_error_rad);
    const float lock_threshold_rad =
        DPLL_LOCK_PHASE_THRESHOLD_DEG * PHASE_PI_F / 180.0f;
    const float unlock_threshold_rad =
        DPLL_UNLOCK_PHASE_THRESHOLD_DEG * PHASE_PI_F / 180.0f;
    float kp;
    float ki;
    float correction_limit_hz;
    float nominal_step_limit_hz;
    float nominal_error_hz;
    float proposed_integrator_hz;
    float unsaturated_correction_hz;
    float correction_hz;

    dpll->phase_error_rad = phase_error_rad;
    dpll->fine_frequency_error_hz = 0.0f;

    /*
     * 相邻长窗相位误差的斜率就是剩余频差。FFT 只给初值，这里每窗最多
     * 修正 0.005 Hz，避免直接跳相或快速拉动输出频率。
     */
    if (absolute_error_rad > lock_threshold_rad)
    {
        /*
         * 大相位误差通常来自目标相位改变。此时只让限速 PI 拉相，禁止
         * 频率估计器把这段有意的相位运动误判成输入频率漂移。
         */
        dpll->phase_error_history_valid = 0u;
    }
    else if (dpll->phase_error_history_valid != 0u)
    {
        const float phase_delta_rad =
            dpll_wrap_phase(phase_error_rad - dpll->previous_phase_error_rad);
        const float observed_frequency_error_hz =
            phase_delta_rad / (PHASE_TWO_PI_F * window_seconds);
        const float fine_step_hz =
            dpll_clamp(DPLL_FINE_FREQUENCY_ALPHA
                       * observed_frequency_error_hz,
                       -DPLL_FINE_FREQUENCY_STEP_LIMIT_HZ,
                       DPLL_FINE_FREQUENCY_STEP_LIMIT_HZ);

        dpll->fine_frequency_error_hz = observed_frequency_error_hz;
        dpll->coarse_frequency_hz =
            dpll_clamp(dpll->coarse_frequency_hz + fine_step_hz,
                       DPLL_MIN_FREQUENCY_HZ,
                       DPLL_MAX_FREQUENCY_HZ);
    }
    if (absolute_error_rad <= lock_threshold_rad)
    {
        dpll->previous_phase_error_rad = phase_error_rad;
        dpll->phase_error_history_valid = 1u;
    }

    nominal_step_limit_hz =
        DPLL_NOMINAL_SLEW_LIMIT_HZ_PER_S * window_seconds;
    nominal_error_hz = dpll->coarse_frequency_hz - dpll->nominal_frequency_hz;
    dpll->nominal_frequency_hz +=
        dpll_clamp(nominal_error_hz,
                   -nominal_step_limit_hz,
                   nominal_step_limit_hz);

    if (dpll->lock_state == DPLL_STATE_LOCKED)
    {
        kp = DPLL_LOCKED_KP_HZ_PER_RAD;
        ki = DPLL_LOCKED_KI_HZ_PER_RAD_S;
        correction_limit_hz = DPLL_LOCKED_CORRECTION_LIMIT_HZ;
    }
    else
    {
        kp = DPLL_CAPTURE_KP_HZ_PER_RAD;
        ki = DPLL_CAPTURE_KI_HZ_PER_RAD_S;
        correction_limit_hz = DPLL_CAPTURE_CORRECTION_LIMIT_HZ;
    }

    proposed_integrator_hz =
        dpll->frequency_integrator_hz
        + (ki * phase_error_rad * window_seconds);
    unsaturated_correction_hz =
        (kp * phase_error_rad) + proposed_integrator_hz;
    correction_hz =
        dpll_clamp(unsaturated_correction_hz,
                   -correction_limit_hz,
                   correction_limit_hz);

    if (!(((unsaturated_correction_hz > correction_limit_hz)
           && (phase_error_rad > 0.0f))
          || ((unsaturated_correction_hz < -correction_limit_hz)
              && (phase_error_rad < 0.0f))))
    {
        dpll->frequency_integrator_hz = proposed_integrator_hz;
    }

    dpll->frequency_integrator_hz =
        dpll_clamp(dpll->frequency_integrator_hz,
                   -correction_limit_hz,
                   correction_limit_hz);
    correction_hz =
        dpll_clamp((kp * phase_error_rad) + dpll->frequency_integrator_hz,
                   -correction_limit_hz,
                   correction_limit_hz);
    dpll->output_frequency_hz =
        dpll->nominal_frequency_hz + correction_hz;
    dpll_update_phase_increment(dpll);

    if (dpll->lock_state == DPLL_STATE_LOCKED)
    {
        if (absolute_error_rad >= unlock_threshold_rad)
        {
            ++dpll->unlock_confirm_count;
            if (dpll->unlock_confirm_count >= DPLL_UNLOCK_CONFIRM_WINDOWS)
            {
                dpll->lock_state = DPLL_STATE_TRACKING;
                dpll->unlock_confirm_count = 0u;
                dpll->lock_confirm_count = 0u;
            }
        }
        else
        {
            dpll->unlock_confirm_count = 0u;
        }
    }
    else if ((absolute_error_rad <= lock_threshold_rad)
             && (fabsf(dpll->fine_frequency_error_hz)
                 <= DPLL_LOCK_FREQUENCY_THRESHOLD_HZ))
    {
        ++dpll->lock_confirm_count;
        if (dpll->lock_confirm_count >= DPLL_LOCK_CONFIRM_WINDOWS)
        {
            /*
             * 切换到低带宽参数时重配积分项，使总频率修正连续，避免
             * Kp 变小的一瞬间产生新的频率台阶和相位漂移。
             */
            dpll->frequency_integrator_hz =
                dpll_clamp(correction_hz
                           - (DPLL_LOCKED_KP_HZ_PER_RAD
                              * phase_error_rad),
                           -DPLL_LOCKED_CORRECTION_LIMIT_HZ,
                           DPLL_LOCKED_CORRECTION_LIMIT_HZ);
            dpll->lock_state = DPLL_STATE_LOCKED;
            dpll->lock_confirm_count = 0u;
            dpll->unlock_confirm_count = 0u;
        }
    }
    else
    {
        dpll->lock_confirm_count = 0u;
        dpll->lock_state = DPLL_STATE_TRACKING;
    }

    dpll_reset_phase_window(dpll);
}

/**
 * @brief 初始化 DPLL 状态。
 * @param dpll DPLL 状态对象。
 * @param initial_frequency_hz 未捕获信号前的安全初始频率。
 * @return 无。
 * @note 不启动输出，首次相关验证通过后才使 frequency_valid 有效。
 */
void dpll_init(dpll_t *dpll, float initial_frequency_hz)
{
    if (dpll == NULL)
    {
        return;
    }

    memset(dpll, 0, sizeof(*dpll));
    initial_frequency_hz =
        dpll_clamp(initial_frequency_hz,
                   DPLL_MIN_FREQUENCY_HZ,
                   DPLL_MAX_FREQUENCY_HZ);
    dpll->nominal_frequency_hz = initial_frequency_hz;
    dpll->coarse_frequency_hz = initial_frequency_hz;
    dpll->output_frequency_hz = initial_frequency_hz;
    dpll->offset_adc_counts = 32768.0f;
    dpll->lock_state = DPLL_STATE_NO_SIGNAL;
    fft_analyzer_init();
    dpll_update_phase_increment(dpll);
    dpll_reset_phase_window(dpll);
}

/**
 * @brief 将 DPLL 恢复到等待信号状态。
 * @param dpll DPLL 状态对象。
 * @return 无。
 * @note 使用配置的默认初始频率。
 */
void dpll_reset(dpll_t *dpll)
{
    dpll_init(dpll, DPLL_DEFAULT_FREQUENCY_HZ);
}

/**
 * @brief 处理一个 ADC 半缓冲并更新 DPLL。
 * @param dpll DPLL 状态对象。
 * @param samples ADC 16 位采样数组。
 * @param sample_count 采样点数。
 * @param target_phase_deg 输出相对输入的目标相位。
 * @return 无。
 * @note 连续相位状态跨调用保存；I/Q 累加窗口也允许跨多个 DMA 块。
 */
void dpll_process_block(dpll_t *dpll,
                        const uint16_t *samples,
                        size_t sample_count,
                        float target_phase_deg)
{
    double sum = 0.0;
    double squared_sum = 0.0;
    float block_mean;
    float block_amplitude;
    float hysteresis;
    fft_analyzer_result_t fft_result;
    size_t index;

    if ((dpll == NULL) || (samples == NULL) || (sample_count == 0u))
    {
        return;
    }

    for (index = 0u; index < sample_count; ++index)
    {
        sum += samples[index];
    }
    block_mean = (float)(sum / (double)sample_count);

    for (index = 0u; index < sample_count; ++index)
    {
        const float centered_sample = (float)samples[index] - block_mean;
        squared_sum += (double)(centered_sample * centered_sample);
    }
    block_amplitude =
        sqrtf((float)(2.0 * squared_sum / (double)sample_count));

    if (dpll->total_samples == 0u)
    {
        dpll->offset_adc_counts = block_mean;
        dpll->amplitude_adc_counts = block_amplitude;
    }
    else
    {
        dpll->offset_adc_counts +=
            DPLL_MEASUREMENT_FILTER_ALPHA
            * (block_mean - dpll->offset_adc_counts);
        dpll->amplitude_adc_counts +=
            DPLL_MEASUREMENT_FILTER_ALPHA
            * (block_amplitude - dpll->amplitude_adc_counts);
    }

    if (dpll->amplitude_adc_counts < DPLL_MIN_AMPLITUDE_ADC_COUNTS)
    {
        ++dpll->no_signal_block_count;
        if (dpll->no_signal_block_count >= DPLL_NO_SIGNAL_CONFIRM_BLOCKS)
        {
            dpll->frequency_valid = 0u;
            dpll->lock_state = DPLL_STATE_NO_SIGNAL;
            dpll->previous_sample_valid = 0u;
            dpll->lock_confirm_count = 0u;
            dpll->unlock_confirm_count = 0u;
            fft_analyzer_reset();
            dpll_reset_acquisition(dpll);
            dpll_reset_phase_window(dpll);
        }

        dpll->phase_accumulator_q32 +=
            (uint64_t)dpll->phase_increment_q32 * (uint64_t)sample_count;
        dpll->total_samples += sample_count;
        return;
    }

    dpll->no_signal_block_count = 0u;
    if (dpll->lock_state == DPLL_STATE_NO_SIGNAL)
    {
        dpll->lock_state = DPLL_STATE_ACQUIRING;
    }

    if (fft_analyzer_process_block(samples,
                                   sample_count,
                                   dpll->offset_adc_counts,
                                   &fft_result) != 0u)
    {
        dpll_handle_fft_frequency(dpll, fft_result.frequency_hz);
    }

    hysteresis =
        fmaxf(dpll->amplitude_adc_counts * DPLL_CROSSING_HYSTERESIS_RATIO,
              DPLL_CROSSING_HYSTERESIS_MIN_COUNTS);

    for (index = 0u; index < sample_count; ++index)
    {
        const float raw_sample = (float)samples[index];
        const float centered_sample = raw_sample - dpll->offset_adc_counts;
        const float previous_centered =
            dpll->previous_raw_sample - dpll->offset_adc_counts;

        if (dpll->validation_active != 0u)
        {
            dpll_process_validation_sample(dpll,
                                           centered_sample,
                                           target_phase_deg);
        }

        if (dpll->frequency_valid != 0u)
        {
            const uint32_t reference_phase_q32 =
                (uint32_t)dpll->phase_accumulator_q32
                - nco_phase_offset_from_deg(target_phase_deg);
            const float sine_value = nco_sin_q32(reference_phase_q32);
            const float cosine_value =
                nco_sin_q32(reference_phase_q32 + DPLL_QUARTER_TURN_Q32);

            dpll->phase_i_sum += (double)(centered_sample * sine_value);
            dpll->phase_q_sum += (double)(centered_sample * cosine_value);
            ++dpll->phase_raw_count;
            if (dpll->phase_raw_count >= dpll->phase_target_raw_count)
            {
                dpll_finish_phase_window(dpll);
            }
        }

        if (dpll->previous_sample_valid != 0u)
        {
            if (centered_sample <= -hysteresis)
            {
                if (dpll->below_hysteresis_count < DPLL_CROSSING_ARM_SAMPLES)
                {
                    ++dpll->below_hysteresis_count;
                }
                if (dpll->below_hysteresis_count
                    >= DPLL_CROSSING_ARM_SAMPLES)
                {
                    dpll->crossing_armed = 1u;
                }
            }
            else if ((dpll->crossing_armed != 0u)
                     && (previous_centered <= 0.0f)
                     && (centered_sample > 0.0f))
            {
                const float denominator =
                    centered_sample - previous_centered;
                const float fraction =
                    (denominator > 0.0f)
                    ? (-previous_centered / denominator)
                    : 0.0f;
                const double crossing_sample =
                    (double)dpll->total_samples
                    + (double)index - 1.0 + (double)fraction;

                dpll_record_rising_crossing(dpll, crossing_sample);
                dpll->crossing_armed = 0u;
                dpll->below_hysteresis_count = 0u;
            }
            else if (dpll->crossing_armed == 0u)
            {
                dpll->below_hysteresis_count = 0u;
            }
        }

        dpll->previous_raw_sample = raw_sample;
        dpll->previous_sample_valid = 1u;
        dpll->phase_accumulator_q32 += dpll->phase_increment_q32;
    }

    dpll->total_samples += sample_count;
}

/**
 * @brief 从持久化相位生成下一段 DAC 正弦波。
 * @param dpll DPLL 状态对象。
 * @param dac_samples DAC 12 位右对齐输出数组。
 * @param sample_count 需生成的采样点数。
 * @param lead_samples 当前输入块末端到该 DAC 块实际播放起点的预测点数。
 * @return 无。
 * @note 仅首次静音起振时对齐未来相位，随后每块都从上一块末相位继续。
 */
void dpll_generate_dac(dpll_t *dpll,
                       uint16_t *dac_samples,
                       size_t sample_count,
                       uint32_t lead_samples)
{
    const float envelope_step =
        1.0f
        / ((float)DPLL_OUTPUT_RAMP_TIME_MS
           * SIGNAL_SAMPLE_RATE_HZ / 1000.0f);
    float dac_offset;
    float dac_amplitude;
    float maximum_amplitude;
    size_t index;

    if ((dpll == NULL) || (dac_samples == NULL) || (sample_count == 0u))
    {
        return;
    }

    if ((dpll->frequency_valid != 0u) && (dpll->output_started == 0u))
    {
        dpll->output_phase_accumulator_q32 =
            dpll->phase_accumulator_q32
            + ((uint64_t)dpll->phase_increment_q32 * (uint64_t)lead_samples);
        dpll->output_started = 1u;
    }

    dac_offset = dpll_clamp(dpll->offset_adc_counts / 16.0f, 0.0f, 4095.0f);
    dac_amplitude = dpll->amplitude_adc_counts / 16.0f;
    maximum_amplitude = fminf(dac_offset, 4095.0f - dac_offset);
    dac_amplitude = dpll_clamp(dac_amplitude, 0.0f, maximum_amplitude);

    for (index = 0u; index < sample_count; ++index)
    {
        float output_code;

        if (dpll->frequency_valid != 0u)
        {
            dpll->output_envelope =
                dpll_clamp(dpll->output_envelope + envelope_step, 0.0f, 1.0f);
        }
        else
        {
            dpll->output_envelope =
                dpll_clamp(dpll->output_envelope - envelope_step, 0.0f, 1.0f);
        }

        output_code =
            dac_offset
            + (dac_amplitude
               * dpll->output_envelope
               * nco_sin_q32((uint32_t)dpll->output_phase_accumulator_q32));
        output_code = dpll_clamp(output_code, 0.0f, 4095.0f);
        dac_samples[index] = (uint16_t)(output_code + 0.5f);
        dpll->output_phase_accumulator_q32 += dpll->phase_increment_q32;
    }

    if ((dpll->frequency_valid == 0u) && (dpll->output_envelope <= 0.0f))
    {
        dpll->output_started = 0u;
        dpll->output_phase_accumulator_q32 = 0u;
    }
}

/**
 * @brief 获取当前相位误差。
 * @param dpll DPLL 状态对象。
 * @return 相位误差，单位度，范围约为 -180～180。
 * @note 只读访问。
 */
float dpll_get_phase_error_deg(const dpll_t *dpll)
{
    if (dpll == NULL)
    {
        return 0.0f;
    }
    return dpll->phase_error_rad * 180.0f / PHASE_PI_F;
}
