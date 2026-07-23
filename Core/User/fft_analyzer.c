/**
 * @file fft_analyzer.c
 * @brief 自包含、分块执行的 32768 点浮点 FFT。
 *
 * 模块用途：使用 64 点盒式低通抽取降低采样率，以 FFT 全频段搜索替代过零主判决。
 * GPIO 引脚：无直接 GPIO 引脚。
 * 依赖外设：无；使用 Cortex-M7 单精度浮点运算。
 * 初始化方法：调用 fft_analyzer_init()。
 * 调用方法：主循环每处理一个 ADC DMA 半缓冲调用一次。
 */

#include "fft_analyzer.h"

#include <math.h>
#include <string.h>

#include "config.h"

typedef enum
{
    FFT_ANALYZER_CAPTURE = 0,
    FFT_ANALYZER_WINDOW,
    FFT_ANALYZER_BIT_REVERSE,
    FFT_ANALYZER_TRANSFORM,
    FFT_ANALYZER_SEARCH_MAX,
    FFT_ANALYZER_SEARCH_FUNDAMENTAL
} fft_analyzer_state_t;

/* 两个数组共 256 KiB，放在 D1 RAM；不占用 DMA 专用 D2 RAM。 */
static float fft_real[FFT_ANALYZER_SIZE];
static float fft_imag[FFT_ANALYZER_SIZE];

static fft_analyzer_state_t fft_state;
static uint32_t fft_capture_index;
static uint32_t fft_decimation_count;
static float fft_decimation_sum;
static uint32_t fft_work_index;
static float fft_window_real;
static float fft_window_imag;
static float fft_window_step_real;
static float fft_window_step_imag;
static uint32_t fft_transform_length;
static uint32_t fft_transform_group;
static uint32_t fft_transform_j;
static float fft_twiddle_real;
static float fft_twiddle_imag;
static float fft_twiddle_step_real;
static float fft_twiddle_step_imag;
static uint32_t fft_search_min_bin;
static uint32_t fft_search_max_bin;
static uint32_t fft_peak_bin;
static float fft_peak_energy;

/**
 * @brief 将浮点值限制到闭区间。
 * @param value 待限制值。
 * @param lower 下限。
 * @param upper 上限。
 * @return 限制结果。
 * @note 无副作用。
 */
static float fft_clamp(float value, float lower, float upper)
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
 * @brief 反转 FFT 下标的有效二进制位。
 * @param value 原下标。
 * @return 位反转后的下标。
 * @note 位数由 FFT_ANALYZER_BITS 决定。
 */
static uint32_t fft_reverse_bits(uint32_t value)
{
    uint32_t reversed = 0u;
    uint32_t bit;

    for (bit = 0u; bit < FFT_ANALYZER_BITS; ++bit)
    {
        reversed = (reversed << 1u) | (value & 1u);
        value >>= 1u;
    }
    return reversed;
}

/**
 * @brief 读取某频点的复数能量。
 * @param bin FFT 频点下标。
 * @return 实部平方加虚部平方。
 * @note 无副作用。
 */
static float fft_bin_energy(uint32_t bin)
{
    return (fft_real[bin] * fft_real[bin])
           + (fft_imag[bin] * fft_imag[bin]);
}

/**
 * @brief 将递推得到的单位复数重新归一化。
 * @param real_value 复数实部地址。
 * @param imag_value 复数虚部地址。
 * @return 无。
 * @note 每隔固定步数调用，抑制长 FFT 中单精度旋转因子的累计漂移。
 */
static void fft_normalize_unit_complex(float *real_value, float *imag_value)
{
    const float magnitude =
        sqrtf((*real_value * *real_value) + (*imag_value * *imag_value));

    if (magnitude > 1.0e-12f)
    {
        *real_value /= magnitude;
        *imag_value /= magnitude;
    }
}

/**
 * @brief 初始化一个新的 FFT 变换阶段。
 * @param transform_length 当前蝶形长度。
 * @return 无。
 * @note 设置阶段旋转因子，不遍历数组。
 */
static void fft_start_transform_stage(uint32_t transform_length)
{
    const float angle =
        -PHASE_TWO_PI_F / (float)transform_length;

    fft_transform_length = transform_length;
    fft_transform_group = 0u;
    fft_transform_j = 0u;
    fft_twiddle_real = 1.0f;
    fft_twiddle_imag = 0.0f;
    fft_twiddle_step_real = cosf(angle);
    fft_twiddle_step_imag = sinf(angle);
}

/**
 * @brief 完成频率抛物线插值并返回结果。
 * @param result 接收 FFT 结果。
 * @return 有效结果返回 1，否则返回 0。
 * @note 使用峰值左右相邻能量的对数进行亚频点抛物线插值。
 */
static uint8_t fft_finish_result(fft_analyzer_result_t *result)
{
    const float left_magnitude =
        logf(fmaxf(fft_bin_energy(fft_peak_bin - 1u), 1.0e-30f));
    const float center_magnitude =
        logf(fmaxf(fft_bin_energy(fft_peak_bin), 1.0e-30f));
    const float right_magnitude =
        logf(fmaxf(fft_bin_energy(fft_peak_bin + 1u), 1.0e-30f));
    const float denominator =
        left_magnitude
        - (2.0f * center_magnitude)
        + right_magnitude;
    float bin_offset = 0.0f;

    if (fabsf(denominator) > 1.0e-12f)
    {
        bin_offset =
            0.5f
            * (left_magnitude - right_magnitude)
            / denominator;
        bin_offset = fft_clamp(bin_offset, -0.5f, 0.5f);
    }

    result->peak_bin = fft_peak_bin;
    result->peak_energy = fft_peak_energy;
    result->frequency_hz =
        ((float)fft_peak_bin + bin_offset)
        * FFT_ANALYZER_SAMPLE_RATE_HZ
        / (float)FFT_ANALYZER_SIZE;

    fft_analyzer_reset();
    return 1u;
}

/**
 * @brief 初始化 FFT 分析器。
 * @param 无。
 * @return 无。
 * @note 不清空大型数组，因为捕获阶段会完整覆盖实部数组。
 */
void fft_analyzer_init(void)
{
    fft_analyzer_reset();
}

/**
 * @brief 丢弃当前捕获或计算并重新开始。
 * @param 无。
 * @return 无。
 * @note 只重置索引和状态，执行时间固定。
 */
void fft_analyzer_reset(void)
{
    fft_state = FFT_ANALYZER_CAPTURE;
    fft_capture_index = 0u;
    fft_decimation_count = 0u;
    fft_decimation_sum = 0.0f;
    fft_work_index = 0u;
    fft_peak_bin = 0u;
    fft_peak_energy = 0.0f;
}

/**
 * @brief 捕获或分片处理一个 ADC 半缓冲。
 * @param samples ADC 16 位采样数组。
 * @param sample_count 采样点数。
 * @param offset_adc_counts 当前 ADC 直流偏置估计。
 * @param result 接收有效 FFT 结果。
 * @return 本次产生结果返回 1，否则返回 0。
 * @note 每次计算最多执行 FFT_ANALYZER_WORK_BUDGET 个工作单元。
 */
uint8_t fft_analyzer_process_block(const uint16_t *samples,
                                   size_t sample_count,
                                   float offset_adc_counts,
                                   fft_analyzer_result_t *result)
{
    uint32_t work_done = 0u;

    if ((samples == NULL) || (result == NULL))
    {
        return 0u;
    }

    if (fft_state == FFT_ANALYZER_CAPTURE)
    {
        size_t sample_index;

        for (sample_index = 0u; sample_index < sample_count; ++sample_index)
        {
            fft_decimation_sum +=
                (float)samples[sample_index] - offset_adc_counts;
            ++fft_decimation_count;

            if (fft_decimation_count >= FFT_ANALYZER_DECIMATION)
            {
                fft_real[fft_capture_index] =
                    fft_decimation_sum
                    / (float)FFT_ANALYZER_DECIMATION;
                ++fft_capture_index;
                fft_decimation_count = 0u;
                fft_decimation_sum = 0.0f;

                if (fft_capture_index >= FFT_ANALYZER_SIZE)
                {
                    const float window_angle =
                        PHASE_TWO_PI_F
                        / (float)(FFT_ANALYZER_SIZE - 1u);

                    fft_state = FFT_ANALYZER_WINDOW;
                    fft_work_index = 0u;
                    fft_window_real = 1.0f;
                    fft_window_imag = 0.0f;
                    fft_window_step_real = cosf(window_angle);
                    fft_window_step_imag = sinf(window_angle);
                    break;
                }
            }
        }
        return 0u;
    }

    if (fft_state == FFT_ANALYZER_WINDOW)
    {
        while ((fft_work_index < FFT_ANALYZER_SIZE)
               && (work_done < FFT_ANALYZER_WORK_BUDGET))
        {
            const float window_value =
                0.5f - (0.5f * fft_window_real);
            const float next_window_real =
                (fft_window_real * fft_window_step_real)
                - (fft_window_imag * fft_window_step_imag);
            const float next_window_imag =
                (fft_window_imag * fft_window_step_real)
                + (fft_window_real * fft_window_step_imag);

            fft_real[fft_work_index] *= window_value;
            fft_imag[fft_work_index] = 0.0f;
            fft_window_real = next_window_real;
            fft_window_imag = next_window_imag;
            ++fft_work_index;
            ++work_done;
            if ((fft_work_index & 0xFFu) == 0u)
            {
                fft_normalize_unit_complex(&fft_window_real,
                                           &fft_window_imag);
            }
        }

        if (fft_work_index >= FFT_ANALYZER_SIZE)
        {
            fft_state = FFT_ANALYZER_BIT_REVERSE;
            fft_work_index = 0u;
        }
        return 0u;
    }

    if (fft_state == FFT_ANALYZER_BIT_REVERSE)
    {
        while ((fft_work_index < FFT_ANALYZER_SIZE)
               && (work_done < FFT_ANALYZER_WORK_BUDGET))
        {
            const uint32_t reversed_index =
                fft_reverse_bits(fft_work_index);

            if (reversed_index > fft_work_index)
            {
                const float real_swap = fft_real[fft_work_index];

                fft_real[fft_work_index] = fft_real[reversed_index];
                fft_real[reversed_index] = real_swap;
            }
            ++fft_work_index;
            ++work_done;
        }

        if (fft_work_index >= FFT_ANALYZER_SIZE)
        {
            fft_state = FFT_ANALYZER_TRANSFORM;
            fft_start_transform_stage(2u);
        }
        return 0u;
    }

    if (fft_state == FFT_ANALYZER_TRANSFORM)
    {
        while (work_done < FFT_ANALYZER_WORK_BUDGET)
        {
            const uint32_t half_length = fft_transform_length / 2u;
            const uint32_t even_index =
                fft_transform_group + fft_transform_j;
            const uint32_t odd_index = even_index + half_length;
            const float odd_real =
                (fft_twiddle_real * fft_real[odd_index])
                - (fft_twiddle_imag * fft_imag[odd_index]);
            const float odd_imag =
                (fft_twiddle_real * fft_imag[odd_index])
                + (fft_twiddle_imag * fft_real[odd_index]);
            const float even_real = fft_real[even_index];
            const float even_imag = fft_imag[even_index];
            const float next_twiddle_real =
                (fft_twiddle_real * fft_twiddle_step_real)
                - (fft_twiddle_imag * fft_twiddle_step_imag);
            const float next_twiddle_imag =
                (fft_twiddle_imag * fft_twiddle_step_real)
                + (fft_twiddle_real * fft_twiddle_step_imag);

            fft_real[even_index] = even_real + odd_real;
            fft_imag[even_index] = even_imag + odd_imag;
            fft_real[odd_index] = even_real - odd_real;
            fft_imag[odd_index] = even_imag - odd_imag;
            fft_twiddle_real = next_twiddle_real;
            fft_twiddle_imag = next_twiddle_imag;
            ++fft_transform_j;
            ++work_done;
            if (((fft_transform_j & 0xFFu) == 0u)
                && (fft_transform_j < half_length))
            {
                fft_normalize_unit_complex(&fft_twiddle_real,
                                           &fft_twiddle_imag);
            }

            if (fft_transform_j >= half_length)
            {
                fft_transform_j = 0u;
                fft_transform_group += fft_transform_length;
                fft_twiddle_real = 1.0f;
                fft_twiddle_imag = 0.0f;

                if (fft_transform_group >= FFT_ANALYZER_SIZE)
                {
                    const uint32_t next_length =
                        fft_transform_length * 2u;

                    if (next_length > FFT_ANALYZER_SIZE)
                    {
                        fft_search_min_bin =
                            (uint32_t)(DPLL_MIN_FREQUENCY_HZ
                                       * (float)FFT_ANALYZER_SIZE
                                       / FFT_ANALYZER_SAMPLE_RATE_HZ);
                        if (fft_search_min_bin > 2u)
                        {
                            fft_search_min_bin -= 2u;
                        }
                        fft_search_max_bin =
                            (uint32_t)(DPLL_MAX_FREQUENCY_HZ
                                       * (float)FFT_ANALYZER_SIZE
                                       / FFT_ANALYZER_SAMPLE_RATE_HZ)
                            + 2u;
                        if (fft_search_max_bin
                            >= (FFT_ANALYZER_SIZE / 2u))
                        {
                            fft_search_max_bin =
                                (FFT_ANALYZER_SIZE / 2u) - 2u;
                        }
                        fft_state = FFT_ANALYZER_SEARCH_MAX;
                        fft_work_index = fft_search_min_bin;
                        fft_peak_bin = fft_search_min_bin;
                        fft_peak_energy = 0.0f;
                        break;
                    }
                    fft_start_transform_stage(next_length);
                }
            }
        }
        return 0u;
    }

    if (fft_state == FFT_ANALYZER_SEARCH_MAX)
    {
        while ((fft_work_index <= fft_search_max_bin)
               && (work_done < FFT_ANALYZER_WORK_BUDGET))
        {
            const float energy = fft_bin_energy(fft_work_index);

            if (energy > fft_peak_energy)
            {
                fft_peak_energy = energy;
                fft_peak_bin = fft_work_index;
            }
            ++fft_work_index;
            ++work_done;
        }

        if (fft_work_index > fft_search_max_bin)
        {
            fft_state = FFT_ANALYZER_SEARCH_FUNDAMENTAL;
            fft_work_index = fft_search_min_bin + 1u;
        }
        return 0u;
    }

    while ((fft_work_index < fft_search_max_bin)
           && (work_done < FFT_ANALYZER_WORK_BUDGET))
    {
        const float energy = fft_bin_energy(fft_work_index);

        if ((energy >= (fft_peak_energy
                        * FFT_ANALYZER_FUNDAMENTAL_ENERGY_RATIO))
            && (energy >= fft_bin_energy(fft_work_index - 1u))
            && (energy >= fft_bin_energy(fft_work_index + 1u)))
        {
            fft_peak_bin = fft_work_index;
            fft_peak_energy = energy;
            return fft_finish_result(result);
        }
        ++fft_work_index;
        ++work_done;
    }

    if (fft_work_index >= fft_search_max_bin)
    {
        return fft_finish_result(result);
    }
    return 0u;
}
