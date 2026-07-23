/**
 * @file dpll_host_test.c
 * @brief DPLL 最小主机端回归测试。
 *
 * 模块用途：验证稳态、频率阶跃、无信号识别及带噪声的最终 DAC 相位。
 * GPIO 引脚：无直接 GPIO 引脚。
 * 依赖外设：无；仅在定义 PHASE_LOCK_HOST_TEST 时生成 main()。
 * 初始化方法：主机 GCC 编译时加入 -DPHASE_LOCK_HOST_TEST。
 * 调用方法：直接运行生成的测试程序，返回 0 表示通过。
 */

#include "system.h"

#ifdef PHASE_LOCK_HOST_TEST

static uint16_t test_samples[SIGNAL_DMA_HALF_SAMPLES];
static uint16_t test_dac_samples[SIGNAL_DMA_HALF_SAMPLES];
static double test_input_phase = 0.7;
static uint32_t test_random_state = 1u;

/**
 * @brief 生成一块带可控噪声的 16 位正弦测试数据。
 * @param frequency_hz 输入频率。
 * @param amplitude 正弦峰值码数。
 * @param noise_amplitude 均匀噪声峰值码数。
 * @return 无；更新测试缓冲和连续输入相位。
 */
static void test_make_sine(float frequency_hz, float amplitude, float noise_amplitude)
{
    const double phase_step =
        (double)PHASE_TWO_PI_F * (double)frequency_hz / (double)SIGNAL_SAMPLE_RATE_HZ;
    uint32_t index;

    for (index = 0u; index < SIGNAL_DMA_HALF_SAMPLES; ++index)
    {
        float sample;
        float noise;

        test_random_state = (test_random_state * 1664525u) + 1013904223u;
        noise = ((float)((test_random_state >> 16u) & 0xffffu) / 32767.5f) - 1.0f;
        sample = 32768.0f
               + (amplitude * sinf((float)test_input_phase))
               + (noise_amplitude * noise);

        if (sample < 0.0f)
        {
            sample = 0.0f;
        }
        if (sample > 65535.0f)
        {
            sample = 65535.0f;
        }
        test_samples[index] = (uint16_t)(sample + 0.5f);

        test_input_phase += phase_step;
        if (test_input_phase >= (double)PHASE_TWO_PI_F)
        {
            test_input_phase -= (double)PHASE_TWO_PI_F;
        }
    }
}

/**
 * @brief 连续处理若干数据块并检查频率和锁定状态。
 * @param dpll 待测试的 DPLL 对象。
 * @param frequency_hz 合成输入频率。
 * @param block_count 处理块数。
 * @return 0 表示通过，非零表示频率或锁定状态失败。
 */
static int test_track_frequency(dpll_t *dpll, float frequency_hz, uint32_t block_count)
{
    uint32_t block;

    for (block = 0u; block < block_count; ++block)
    {
        test_make_sine(frequency_hz, 12000.0f, 100.0f);
        dpll_process_block(dpll, test_samples, SIGNAL_DMA_HALF_SAMPLES);
    }

    if (fabsf(dpll->output_frequency_hz - frequency_hz) > 5.0f)
    {
        return 1;
    }
    if (dpll->lock_state != DPLL_STATE_LOCKED)
    {
        return 2;
    }
    return 0;
}

/**
 * @brief 测量一个最终 DAC 缓冲区相对未来输入信号的相位差。
 * @param frequency_hz 输入频率。
 * @param future_input_phase DAC 播放首点对应的输入相位。
 * @return DAC 相对输入的相位差，单位度。
 */
static float test_measure_dac_phase(float frequency_hz, double future_input_phase)
{
    const double phase_step =
        (double)PHASE_TWO_PI_F * (double)frequency_hz / (double)SIGNAL_SAMPLE_RATE_HZ;
    double correlation_i = 0.0;
    double correlation_q = 0.0;
    uint32_t index;

    for (index = 0u; index < SIGNAL_DMA_HALF_SAMPLES; ++index)
    {
        const double output = (double)test_dac_samples[index] - 2048.0;
        const double reference_phase = future_input_phase + (phase_step * (double)index);

        correlation_i += output * sin(reference_phase);
        correlation_q += output * cos(reference_phase);
    }

    return (float)(atan2(correlation_q, correlation_i) * 180.0 / (double)PHASE_PI_F);
}

/**
 * @brief 验证最终 DAC 波形在非整数周期输入下不会持续漂移。
 * @param dpll 待测试的 DPLL 对象。
 * @param frequency_hz 合成输入频率。
 * @return 0 表示通过，非零表示未锁定、相位误差大或持续漂移。
 */
static int test_dac_phase_lock(dpll_t *dpll, float frequency_hz)
{
    const double phase_step =
        (double)PHASE_TWO_PI_F * (double)frequency_hz / (double)SIGNAL_SAMPLE_RATE_HZ;
    float previous_phase_deg = 0.0f;
    float maximum_phase_step_deg = 0.0f;
    float final_phase_deg = 0.0f;
    float accumulated_phase_drift_deg = 0.0f;
    uint32_t block;

    for (block = 0u; block < 5000u; ++block)
    {
        double future_input_phase;
        float phase_deg;
        float phase_step_deg;

        test_make_sine(frequency_hz, 12000.0f, 1000.0f);
        dpll_process_block(dpll, test_samples, SIGNAL_DMA_HALF_SAMPLES);
        dpll_generate_dac(dpll,
                          test_dac_samples,
                          SIGNAL_DMA_HALF_SAMPLES,
                          SIGNAL_DMA_HALF_SAMPLES,
                          0.0f);

        future_input_phase =
            test_input_phase + (phase_step * (double)SIGNAL_DMA_HALF_SAMPLES);
        phase_deg = test_measure_dac_phase(frequency_hz, future_input_phase);

        if (block >= 4000u)
        {
            phase_step_deg = phase_deg - previous_phase_deg;
            if (phase_step_deg > 180.0f)
            {
                phase_step_deg -= 360.0f;
            }
            if (phase_step_deg < -180.0f)
            {
                phase_step_deg += 360.0f;
            }
            if (fabsf(phase_step_deg) > maximum_phase_step_deg)
            {
                maximum_phase_step_deg = fabsf(phase_step_deg);
            }
            accumulated_phase_drift_deg += phase_step_deg;
        }

        previous_phase_deg = phase_deg;
        final_phase_deg = phase_deg;
    }

    printf("DAC phase test: f=%.3f Hz phase=%.3f deg max_step=%.3f deg "
           "drift=%.3f deg state=%d\n",
           dpll->output_frequency_hz,
           final_phase_deg,
           maximum_phase_step_deg,
           accumulated_phase_drift_deg,
           (int)dpll->lock_state);

    if (dpll->lock_state != DPLL_STATE_LOCKED)
    {
        return 1;
    }
    if (fabsf(final_phase_deg) > 5.0f)
    {
        return 2;
    }
    if (maximum_phase_step_deg > 2.0f)
    {
        return 3;
    }
    if (fabsf(accumulated_phase_drift_deg) > 5.0f)
    {
        return 4;
    }
    return 0;
}

/**
 * @brief 执行稳态、频率阶跃、无信号和最终 DAC 相位回归测试。
 * @param 无。
 * @return 0 表示全部通过，非零表示失败项。
 */
int main(void)
{
    dpll_t dpll;
    dpll_t output_dpll;
    int result;

    nco_init();
    dpll_init(&dpll, DPLL_DEFAULT_FREQUENCY_HZ);

    result = test_track_frequency(&dpll, 10000.0f, 150u);
    if (result != 0)
    {
        printf("steady lock failed: %d f=%.3f phase=%.3f state=%d\n",
               result,
               dpll.output_frequency_hz,
               dpll_get_phase_error_deg(&dpll),
               (int)dpll.lock_state);
        return result;
    }

    result = test_track_frequency(&dpll, 12345.0f, 250u);
    if (result != 0)
    {
        printf("step lock failed: %d f=%.3f phase=%.3f state=%d\n",
               result,
               dpll.output_frequency_hz,
               dpll_get_phase_error_deg(&dpll),
               (int)dpll.lock_state);
        return result + 10;
    }

    memset(test_samples, 0x80, sizeof(test_samples));
    dpll_process_block(&dpll, test_samples, SIGNAL_DMA_HALF_SAMPLES);
    if (dpll.lock_state != DPLL_STATE_NO_SIGNAL)
    {
        printf("no-signal detection failed\n");
        return 20;
    }

    test_input_phase = 0.7;
    test_random_state = 1u;
    dpll_init(&output_dpll, DPLL_DEFAULT_FREQUENCY_HZ);
    result = test_dac_phase_lock(&output_dpll, 9973.25f);
    if (result != 0)
    {
        printf("DAC phase lock failed: %d\n", result);
        return result + 20;
    }

    printf("DPLL tests passed: f=%.3f Hz phase=%.3f deg\n",
           dpll.output_frequency_hz,
           dpll_get_phase_error_deg(&dpll));
    return 0;
}

#endif /* PHASE_LOCK_HOST_TEST */
