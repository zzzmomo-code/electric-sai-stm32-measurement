/**
 * @file dpll_host_test.c
 * @brief DPLL 的主机端确定性测试。
 *
 * 模块用途：验证 500 Hz～4 kHz 测频、倍频抑制、跨块锁相、连续 DAC 相位和失信号处理。
 * GPIO 引脚：无直接 GPIO 引脚。
 * 依赖外设：无；仅在定义 PHASE_LOCK_HOST_TEST 时参与主机编译。
 * 初始化方法：测试入口中调用 nco_init() 和 dpll_init()。
 * 调用方法：由主机 C 编译器生成并运行测试程序。
 */

#ifdef PHASE_LOCK_HOST_TEST

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "config.h"
#include "dpll.h"
#include "fft_analyzer.h"
#include "nco.h"

static uint16_t test_adc_samples[SIGNAL_DMA_HALF_SAMPLES];
static uint16_t test_dac_samples[SIGNAL_DMA_HALF_SAMPLES];
static double test_input_phase_rad;
static double test_noise_phase_rad;
static double test_noise_amplitude;
static double test_fourth_harmonic_amplitude;

#define TEST_CHECK(condition, message)                                      \
    do                                                                      \
    {                                                                       \
        if (!(condition))                                                   \
        {                                                                   \
            (void)printf("FAIL line=%d: %s\n", __LINE__, (message));         \
            return 0;                                                       \
        }                                                                   \
    } while (0)

/**
 * @brief 将浮点采样值安全转换为 ADC 16 位码。
 * @param value 浮点 ADC 码。
 * @return 0～65535 范围内的整数码。
 * @note 无副作用。
 */
static uint16_t test_to_adc_code(double value)
{
    if (value < 0.0)
    {
        value = 0.0;
    }
    if (value > 65535.0)
    {
        value = 65535.0;
    }
    return (uint16_t)(value + 0.5);
}

/**
 * @brief 生成一块带可选二次谐波的连续输入。
 * @param frequency_hz 基波频率。
 * @param fundamental_amplitude 基波峰值 ADC 码。
 * @param second_harmonic_amplitude 二次谐波峰值 ADC 码。
 * @return 无。
 * @note test_input_phase_rad 跨块连续保存。
 */
static void test_generate_input(double frequency_hz,
                                double fundamental_amplitude,
                                double second_harmonic_amplitude)
{
    const double phase_step =
        2.0 * (double)PHASE_PI_F * frequency_hz
        / (double)SIGNAL_SAMPLE_RATE_HZ;
    const double noise_phase_step =
        2.0 * (double)PHASE_PI_F * 70000.0
        / (double)SIGNAL_SAMPLE_RATE_HZ;
    uint32_t index;

    for (index = 0u; index < SIGNAL_DMA_HALF_SAMPLES; ++index)
    {
        const double value =
            32768.0
            + (fundamental_amplitude * sin(test_input_phase_rad))
            + (second_harmonic_amplitude
               * sin((2.0 * test_input_phase_rad) + 0.35))
            + (test_fourth_harmonic_amplitude
               * sin((4.0 * test_input_phase_rad) + 0.21))
            + (test_noise_amplitude * sin(test_noise_phase_rad));

        test_adc_samples[index] = test_to_adc_code(value);
        test_input_phase_rad += phase_step;
        if (test_input_phase_rad >= (2.0 * (double)PHASE_PI_F))
        {
            test_input_phase_rad -= 2.0 * (double)PHASE_PI_F;
        }
        test_noise_phase_rad += noise_phase_step;
        if (test_noise_phase_rad >= (2.0 * (double)PHASE_PI_F))
        {
            test_noise_phase_rad -= 2.0 * (double)PHASE_PI_F;
        }
    }
}

/**
 * @brief 单独验证 FFT 基波频率输出。
 * @param frequency_hz 输入基波频率。
 * @param second_harmonic_amplitude 二次谐波幅度。
 * @param fourth_harmonic_amplitude 四次谐波幅度。
 * @return 通过返回 1，否则返回 0。
 * @note 直接检查 FFT 结果，不经过 DPLL。
 */
static int test_fft_frequency_point(double frequency_hz,
                                    double second_harmonic_amplitude,
                                    double fourth_harmonic_amplitude)
{
    fft_analyzer_result_t result = {0};
    uint32_t block;
    uint8_t result_valid = 0u;

    test_input_phase_rad = 0.37;
    test_noise_phase_rad = 0.11;
    test_noise_amplitude = 0.0;
    test_fourth_harmonic_amplitude = fourth_harmonic_amplitude;
    fft_analyzer_init();

    for (block = 0u; block < 2000u; ++block)
    {
        test_generate_input(frequency_hz,
                            12000.0,
                            second_harmonic_amplitude);
        result_valid =
            fft_analyzer_process_block(test_adc_samples,
                                       SIGNAL_DMA_HALF_SAMPLES,
                                       32768.0f,
                                       &result);
        if (result_valid != 0u)
        {
            break;
        }
    }
    test_fourth_harmonic_amplitude = 0.0;

    TEST_CHECK(result_valid != 0u, "FFT did not produce a result");
    TEST_CHECK(fabs((double)result.frequency_hz - frequency_hz) < 0.50,
               "FFT selected the wrong fundamental");
    (void)printf("PASS FFT input=%7.1fHz result=%9.4fHz bin=%lu blocks=%lu\n",
                 frequency_hz,
                 (double)result.frequency_hz,
                 (unsigned long)result.peak_bin,
                 (unsigned long)(block + 1u));
    return 1;
}

/**
 * @brief 运行 DPLL 直到锁定或达到时限。
 * @param dpll DPLL 状态对象。
 * @param frequency_hz 输入基波频率。
 * @param second_harmonic_amplitude 二次谐波峰值。
 * @param target_phase_deg 目标相位。
 * @param maximum_blocks 最大 DMA 半块数。
 * @return 实际处理块数；超时返回 maximum_blocks。
 * @note 同时调用 DAC 生成器，覆盖真实运行顺序。
 */
static uint32_t test_run_until_locked(dpll_t *dpll,
                                      double frequency_hz,
                                      double second_harmonic_amplitude,
                                      float target_phase_deg,
                                      uint32_t maximum_blocks)
{
    uint32_t block;

    for (block = 0u; block < maximum_blocks; ++block)
    {
        test_generate_input(frequency_hz, 12000.0, second_harmonic_amplitude);
        dpll_process_block(dpll,
                           test_adc_samples,
                           SIGNAL_DMA_HALF_SAMPLES,
                           target_phase_deg);
        dpll_generate_dac(dpll,
                          test_dac_samples,
                          SIGNAL_DMA_HALF_SAMPLES,
                          SIGNAL_DMA_HALF_SAMPLES);
        if (dpll->lock_state == DPLL_STATE_LOCKED)
        {
            return block + 1u;
        }
    }

    return maximum_blocks;
}

/**
 * @brief 直接对最终 DAC 采样数组计数，测量实际生成频率。
 * @param dpll 已捕获的 DPLL 状态对象。
 * @param input_frequency_hz 持续输入的基波频率。
 * @param second_harmonic_amplitude 输入二次谐波峰值。
 * @param target_phase_deg 目标相位。
 * @param block_count 测量 DMA 半块数。
 * @return DAC 上升过零测得的频率；过零不足时返回 0。
 * @note 不使用 output_frequency_hz 状态量作为测试结果。
 */
static double test_measure_dac_frequency(dpll_t *dpll,
                                         double input_frequency_hz,
                                         double second_harmonic_amplitude,
                                         float target_phase_deg,
                                         uint32_t block_count)
{
    double first_crossing = 0.0;
    double last_crossing = 0.0;
    float previous_sample = 0.0f;
    uint32_t crossing_count = 0u;
    uint32_t block;
    uint8_t previous_valid = 0u;

    for (block = 0u; block < block_count; ++block)
    {
        uint32_t index;

        test_generate_input(input_frequency_hz,
                            12000.0,
                            second_harmonic_amplitude);
        dpll_process_block(dpll,
                           test_adc_samples,
                           SIGNAL_DMA_HALF_SAMPLES,
                           target_phase_deg);
        dpll_generate_dac(dpll,
                          test_dac_samples,
                          SIGNAL_DMA_HALF_SAMPLES,
                          SIGNAL_DMA_HALF_SAMPLES);

        for (index = 0u; index < SIGNAL_DMA_HALF_SAMPLES; ++index)
        {
            const float current_sample = (float)test_dac_samples[index];
            const float threshold = dpll->offset_adc_counts / 16.0f;

            if ((previous_valid != 0u)
                && (previous_sample <= threshold)
                && (current_sample > threshold))
            {
                const float denominator = current_sample - previous_sample;
                const float fraction =
                    (denominator > 0.0f)
                    ? ((threshold - previous_sample) / denominator)
                    : 0.0f;
                const double crossing =
                    (double)(block * SIGNAL_DMA_HALF_SAMPLES + index)
                    - 1.0 + (double)fraction;

                if (crossing_count == 0u)
                {
                    first_crossing = crossing;
                }
                last_crossing = crossing;
                ++crossing_count;
            }

            previous_sample = current_sample;
            previous_valid = 1u;
        }
    }

    if (crossing_count < 2u)
    {
        return 0.0;
    }
    return ((double)(crossing_count - 1u) * (double)SIGNAL_SAMPLE_RATE_HZ)
           / (last_crossing - first_crossing);
}

/**
 * @brief 验证一个简化验收频点。
 * @param frequency_hz 输入频率。
 * @return 通过返回 1，否则返回 0。
 * @note 锁定时间上限按 5 秒计算。
 */
static int test_frequency_point(double frequency_hz)
{
    const uint32_t maximum_blocks =
        (uint32_t)(5.0 * (double)SIGNAL_SAMPLE_RATE_HZ
                   / (double)SIGNAL_DMA_HALF_SAMPLES);
    dpll_t dpll;
    uint32_t used_blocks;
    double elapsed_seconds;
    double measured_dac_frequency_hz;

    test_input_phase_rad = 0.61;
    test_noise_amplitude = 0.0;
    dpll_init(&dpll, DPLL_DEFAULT_FREQUENCY_HZ);
    used_blocks =
        test_run_until_locked(&dpll,
                              frequency_hz,
                              0.0,
                              0.0f,
                              maximum_blocks);
    elapsed_seconds =
        (double)used_blocks * (double)SIGNAL_DMA_HALF_SAMPLES
        / (double)SIGNAL_SAMPLE_RATE_HZ;
    measured_dac_frequency_hz =
        test_measure_dac_frequency(&dpll,
                                   frequency_hz,
                                   0.0,
                                   0.0f,
                                   64u);

    if ((dpll.lock_state != DPLL_STATE_LOCKED)
        || (fabs((double)dpll_get_phase_error_deg(&dpll))
            > DPLL_LOCK_PHASE_THRESHOLD_DEG))
    {
        (void)printf(
            "DIAG frequency=%7.1fHz state=%u valid=%u coarse=%9.4fHz "
            "nominal=%9.4fHz output=%9.4fHz fine=%8.4fHz "
            "phase=%8.3fdeg blocks=%lu\n",
            frequency_hz,
            (unsigned int)dpll.lock_state,
            (unsigned int)dpll.frequency_valid,
            (double)dpll.coarse_frequency_hz,
            (double)dpll.nominal_frequency_hz,
            (double)dpll.output_frequency_hz,
            (double)dpll.fine_frequency_error_hz,
            (double)dpll_get_phase_error_deg(&dpll),
            (unsigned long)used_blocks);
    }

    TEST_CHECK(dpll.lock_state == DPLL_STATE_LOCKED,
               "frequency point did not lock");
    TEST_CHECK(fabs((double)dpll.coarse_frequency_hz - frequency_hz) < 0.25,
               "coarse frequency is outside 0.25 Hz");
    TEST_CHECK(fabs((double)dpll.output_frequency_hz - frequency_hz) < 0.60,
               "output frequency is outside 0.60 Hz");
    TEST_CHECK(fabs(measured_dac_frequency_hz - frequency_hz) < 0.60,
               "actual DAC samples have the wrong frequency");
    TEST_CHECK(fabs((double)dpll_get_phase_error_deg(&dpll))
               <= DPLL_LOCK_PHASE_THRESHOLD_DEG,
               "locked phase error exceeds threshold");
    TEST_CHECK(elapsed_seconds <= 5.0, "lock time exceeds 5 seconds");

    (void)printf("PASS frequency=%7.1fHz dac=%9.3fHz lock=%6.3fs phase=%7.3fdeg\n",
                 frequency_hz,
                 measured_dac_frequency_hz,
                 elapsed_seconds,
                 (double)dpll_get_phase_error_deg(&dpll));
    return 1;
}

/**
 * @brief 验证含强二次谐波时不会锁到二倍频。
 * @return 通过返回 1，否则返回 0。
 * @note 二次谐波幅度高于基波，专门验证“最低可信基波优先”策略。
 */
static int test_second_harmonic_rejection(void)
{
    dpll_t dpll;
    uint32_t used_blocks;
    double measured_dac_frequency_hz;

    test_input_phase_rad = 0.23;
    test_noise_amplitude = 0.0;
    dpll_init(&dpll, DPLL_DEFAULT_FREQUENCY_HZ);
    used_blocks =
        test_run_until_locked(&dpll, 1000.0, 16000.0, 0.0f, 2442u);
    measured_dac_frequency_hz =
        test_measure_dac_frequency(&dpll,
                                   1000.0,
                                   16000.0,
                                   0.0f,
                                   64u);

    TEST_CHECK(dpll.lock_state == DPLL_STATE_LOCKED,
               "harmonic input did not lock");
    TEST_CHECK(fabsf(dpll.coarse_frequency_hz - 1000.0f) < 0.50f,
               "harmonic input locked to the wrong candidate");
    TEST_CHECK(fabs(measured_dac_frequency_hz - 1000.0) < 0.60,
               "harmonic input produced a doubled DAC frequency");
    TEST_CHECK(used_blocks < 2442u, "harmonic rejection exceeded timeout");

    (void)printf("PASS harmonic selected=%7.3fHz dac=%9.3fHz\n",
                 (double)dpll.coarse_frequency_hz,
                 measured_dac_frequency_hz);
    return 1;
}

/**
 * @brief 验证四次谐波制造多个过零时不会只回退一档而输出二倍频。
 * @return 通过返回 1，否则返回 0。
 * @note 四次谐波幅度高于基波，用来复现“粗测到 4f、旧候选最低为 2f”。
 */
static int test_fourth_harmonic_rejection(void)
{
    dpll_t dpll;
    uint32_t used_blocks;
    double measured_dac_frequency_hz;

    test_input_phase_rad = 0.39;
    test_noise_amplitude = 0.0;
    test_fourth_harmonic_amplitude = 16000.0;
    dpll_init(&dpll, DPLL_DEFAULT_FREQUENCY_HZ);
    used_blocks =
        test_run_until_locked(&dpll, 1000.0, 0.0, 0.0f, 2442u);
    measured_dac_frequency_hz =
        test_measure_dac_frequency(&dpll,
                                   1000.0,
                                   0.0,
                                   0.0f,
                                   64u);
    test_fourth_harmonic_amplitude = 0.0;

    TEST_CHECK(dpll.lock_state == DPLL_STATE_LOCKED,
               "fourth harmonic input did not lock");
    TEST_CHECK(fabsf(dpll.coarse_frequency_hz - 1000.0f) < 0.50f,
               "fourth harmonic input selected 2f or 4f");
    TEST_CHECK(fabs(measured_dac_frequency_hz - 1000.0) < 0.60,
               "fourth harmonic input doubled the DAC frequency");
    TEST_CHECK(used_blocks < 2442u,
               "fourth harmonic rejection exceeded timeout");
    (void)printf("PASS fourth harmonic selected=%7.3fHz dac=%9.3fHz\n",
                 (double)dpll.coarse_frequency_hz,
                 measured_dac_frequency_hz);
    return 1;
}

/**
 * @brief 验证高频噪声造成短时多次过零时仍能捕获基波。
 * @return 通过返回 1，否则返回 0。
 * @note 70 kHz 噪声幅度约为基波的 42%。
 */
static int test_noisy_crossing_rejection(void)
{
    dpll_t dpll;
    uint32_t used_blocks;

    test_input_phase_rad = 0.47;
    test_noise_phase_rad = 0.19;
    test_noise_amplitude = 5000.0;
    dpll_init(&dpll, DPLL_DEFAULT_FREQUENCY_HZ);
    used_blocks =
        test_run_until_locked(&dpll, 1000.0, 0.0, 0.0f, 2442u);
    test_noise_amplitude = 0.0;

    TEST_CHECK(dpll.lock_state == DPLL_STATE_LOCKED,
               "noisy crossing input did not lock");
    TEST_CHECK(fabsf(dpll.coarse_frequency_hz - 1000.0f) < 0.50f,
               "short noisy crossings corrupted coarse frequency");
    TEST_CHECK(used_blocks < 2442u, "noisy crossing rejection timed out");
    (void)printf("PASS noisy crossings selected=%7.3fHz\n",
                 (double)dpll.coarse_frequency_hz);
    return 1;
}

/**
 * @brief 验证 DAC 相位累加器跨缓冲区严格连续。
 * @return 通过返回 1，否则返回 0。
 * @note 直接检查 64 位相位状态，避免用量化后的 DAC 码间接判断。
 */
static int test_persistent_output_phase(void)
{
    dpll_t dpll;
    uint64_t first_end_phase;
    uint64_t expected_second_end_phase;

    dpll_init(&dpll, 1000.0f);
    dpll.frequency_valid = 1u;
    dpll.phase_accumulator_q32 = 0x12345678u;
    dpll.output_frequency_hz = 1000.0f;
    dpll.phase_increment_q32 =
        nco_increment_from_hz(1000.0f, SIGNAL_SAMPLE_RATE_HZ);
    dpll.amplitude_adc_counts = 12000.0f;
    dpll.offset_adc_counts = 32768.0f;

    dpll_generate_dac(&dpll,
                      test_dac_samples,
                      SIGNAL_DMA_HALF_SAMPLES,
                      SIGNAL_DMA_HALF_SAMPLES);
    first_end_phase = dpll.output_phase_accumulator_q32;
    expected_second_end_phase =
        first_end_phase
        + ((uint64_t)dpll.phase_increment_q32
           * (uint64_t)SIGNAL_DMA_HALF_SAMPLES);

    dpll_generate_dac(&dpll,
                      test_dac_samples,
                      SIGNAL_DMA_HALF_SAMPLES,
                      SIGNAL_DMA_HALF_SAMPLES);

    TEST_CHECK(dpll.output_phase_accumulator_q32 == expected_second_end_phase,
               "DAC phase was reset at a block boundary");
    (void)printf("PASS persistent DAC phase across DMA blocks\n");
    return 1;
}

/**
 * @brief 验证目标相位改变时不直接改写 DAC 相位。
 * @return 通过返回 1，否则返回 0。
 * @note 相位目标只通过受限频偏缓慢拉回。
 */
static int test_slow_target_phase_pull(void)
{
    dpll_t dpll;
    uint32_t block;
    uint64_t output_phase_before_process;

    test_input_phase_rad = 1.07;
    test_noise_amplitude = 0.0;
    dpll_init(&dpll, DPLL_DEFAULT_FREQUENCY_HZ);
    TEST_CHECK(test_run_until_locked(&dpll, 1000.0, 0.0, 0.0f, 2442u)
               < 2442u,
               "initial phase lock failed");

    output_phase_before_process = dpll.output_phase_accumulator_q32;
    test_generate_input(1000.0, 12000.0, 0.0);
    dpll_process_block(&dpll,
                       test_adc_samples,
                       SIGNAL_DMA_HALF_SAMPLES,
                       90.0f);
    TEST_CHECK(dpll.output_phase_accumulator_q32 == output_phase_before_process,
               "target change directly rewrote DAC phase");

    dpll_generate_dac(&dpll,
                      test_dac_samples,
                      SIGNAL_DMA_HALF_SAMPLES,
                      SIGNAL_DMA_HALF_SAMPLES);
    for (block = 0u; block < 8790u; ++block)
    {
        test_generate_input(1000.0, 12000.0, 0.0);
        dpll_process_block(&dpll,
                           test_adc_samples,
                           SIGNAL_DMA_HALF_SAMPLES,
                           90.0f);
        dpll_generate_dac(&dpll,
                          test_dac_samples,
                          SIGNAL_DMA_HALF_SAMPLES,
                          SIGNAL_DMA_HALF_SAMPLES);
    }

    (void)printf("INFO slow target state=%d phase=%7.3fdeg freq=%9.4fHz\n",
                 (int)dpll.lock_state,
                 (double)dpll_get_phase_error_deg(&dpll),
                 (double)dpll.output_frequency_hz);
    TEST_CHECK(dpll.lock_state == DPLL_STATE_LOCKED,
               "90 degree target did not relock");
    TEST_CHECK(fabsf(dpll_get_phase_error_deg(&dpll)) <= 5.0f,
               "90 degree target phase error exceeds 5 degrees");
    (void)printf("PASS slow target pull duration=18.0s phase=%7.3fdeg\n",
                 (double)dpll_get_phase_error_deg(&dpll));
    return 1;
}

/**
 * @brief 验证连续低幅输入最终进入失信号状态。
 * @return 通过返回 1，否则返回 0。
 * @note 低幅确认块数由 config.h 配置。
 */
static int test_no_signal_detection(void)
{
    dpll_t dpll;
    uint32_t block;
    uint32_t index;

    dpll_init(&dpll, DPLL_DEFAULT_FREQUENCY_HZ);
    dpll.frequency_valid = 1u;
    dpll.lock_state = DPLL_STATE_LOCKED;
    dpll.amplitude_adc_counts = 1000.0f;

    for (index = 0u; index < SIGNAL_DMA_HALF_SAMPLES; ++index)
    {
        test_adc_samples[index] = 32768u;
    }
    for (block = 0u; block < 40u; ++block)
    {
        dpll_process_block(&dpll,
                           test_adc_samples,
                           SIGNAL_DMA_HALF_SAMPLES,
                           0.0f);
    }

    TEST_CHECK(dpll.lock_state == DPLL_STATE_NO_SIGNAL,
               "DC input was not classified as no signal");
    TEST_CHECK(dpll.frequency_valid == 0u,
               "frequency remained valid after signal loss");
    (void)printf("PASS no-signal detection\n");
    return 1;
}

int main(void)
{
    nco_init();

    TEST_CHECK(test_fft_frequency_point(500.0, 0.0, 0.0) != 0,
               "500 Hz FFT test failed");
    TEST_CHECK(test_fft_frequency_point(1000.0, 0.0, 0.0) != 0,
               "1 kHz FFT test failed");
    TEST_CHECK(test_fft_frequency_point(2000.0, 0.0, 0.0) != 0,
               "2 kHz FFT test failed");
    TEST_CHECK(test_fft_frequency_point(3000.0, 0.0, 0.0) != 0,
               "3 kHz FFT test failed");
    TEST_CHECK(test_fft_frequency_point(4000.0, 0.0, 0.0) != 0,
               "4 kHz FFT test failed");
    TEST_CHECK(test_fft_frequency_point(1000.0, 16000.0, 0.0) != 0,
               "strong second harmonic FFT test failed");
    TEST_CHECK(test_fft_frequency_point(1000.0, 0.0, 16000.0) != 0,
               "strong fourth harmonic FFT test failed");
    TEST_CHECK(sizeof(dpll_t) <= 512u, "dpll_t exceeds 512-byte RAM budget");
    TEST_CHECK(test_frequency_point(500.0) != 0, "500 Hz test failed");
    TEST_CHECK(test_frequency_point(1000.0) != 0, "1 kHz test failed");
    TEST_CHECK(test_frequency_point(2000.0) != 0, "2 kHz test failed");
    TEST_CHECK(test_frequency_point(3000.0) != 0, "3 kHz test failed");
    TEST_CHECK(test_frequency_point(4000.0) != 0, "4 kHz test failed");
    TEST_CHECK(test_second_harmonic_rejection() != 0,
               "second harmonic rejection test failed");
    TEST_CHECK(test_fourth_harmonic_rejection() != 0,
               "fourth harmonic rejection test failed");
    TEST_CHECK(test_noisy_crossing_rejection() != 0,
               "noisy crossing rejection test failed");
    TEST_CHECK(test_persistent_output_phase() != 0,
               "persistent output phase test failed");
    TEST_CHECK(test_slow_target_phase_pull() != 0,
               "slow phase pull test failed");
    TEST_CHECK(test_no_signal_detection() != 0,
               "no signal test failed");

    (void)printf("PASS all DPLL host tests sizeof(dpll_t)=%lu bytes\n",
                 (unsigned long)sizeof(dpll_t));
    return 0;
}

#endif /* PHASE_LOCK_HOST_TEST */
