/**
 * @file onchip_measurement.c
 * @brief STM32H743 片上 ADC 单通道周期信号测量实现。
 *
 * 模块用途：完成“ADC1 DMA -> 8192 点 Hann FFT -> 谐波倍频约束 ->
 *          多正弦联合 I/Q 最小二乘 -> 去干扰后 Vpp/RMS/分量 -> 串口屏快照”。
 * GPIO 引脚映射：PA6/ADC1_INP3 接 1.65 V 中心偏置后的唯一模拟输入。
 * 依赖的外设和 CubeIDE 配置：ADC1 12 位、32 MHz ADC 内核时钟、2.5 周期采样，
 *          TIM2 PSC=0/ARR=74/TRGO Update，DMA1 Stream0 Normal Halfword。
 * 初始化方法：system_init() 调用 onchip_measurement_init()。
 * 调用方法：system_process() 持续调用 onchip_measurement_process()。
 */

#include "system.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define ONCHIP_MEASUREMENT_PI 3.14159265358979323846f
#define ONCHIP_MEASUREMENT_ADC_FULL_SCALE_CODE 4095.0f
#define ONCHIP_MEASUREMENT_ADC_VREF_UV 3300000.0f
#define ONCHIP_MEASUREMENT_ADC_CENTER_UV 1650000.0f
/*
 * 模拟前端当前按单位增益换算。实板若加入固定增益，只改这一项并用标准源校准，
 * 不得用显示端比例系数掩盖硬件增益误差。
 */
#define ONCHIP_MEASUREMENT_FRONT_END_GAIN 1.0f
#define ONCHIP_MEASUREMENT_MIN_SIGNAL_HZ 8000.0f
#define ONCHIP_MEASUREMENT_MAX_SIGNAL_HZ 550000.0f
#define ONCHIP_MEASUREMENT_INTERFERENCE_MIN_HZ 650000.0f
#define ONCHIP_MEASUREMENT_INTERFERENCE_MAX_HZ 1550000.0f
#define ONCHIP_MEASUREMENT_CAPTURE_TIMEOUT_MS 20u
#define ONCHIP_MEASUREMENT_RESTART_DELAY_MS 100u
#define ONCHIP_MEASUREMENT_MAX_PEAKS 12u
#define ONCHIP_MEASUREMENT_MAX_TONES 4u
#define ONCHIP_MEASUREMENT_MAX_MATRIX 8u
#define ONCHIP_MEASUREMENT_MAX_HARMONIC_ORDER 50u
#define ONCHIP_MEASUREMENT_NOISE_SAMPLES 128u
#define ONCHIP_MEASUREMENT_RECONSTRUCTION_POINTS 4096u
#define ONCHIP_MEASUREMENT_TIME_POINTS 1050u
#define ONCHIP_MEASUREMENT_FIT_GRID_COUNT 5u
#define ONCHIP_MEASUREMENT_PEAK_MATCH_BINS 1.75f
#define ONCHIP_MEASUREMENT_MIN_COMPONENT_UV 500.0f
#define ONCHIP_MEASUREMENT_MIN_INTERFERENCE_UV 1000.0f
#define ONCHIP_MEASUREMENT_CLIP_LOW_CODE 8u
#define ONCHIP_MEASUREMENT_CLIP_HIGH_CODE 4087u

/** FFT 候选谱峰。 */
typedef struct
{
    uint16_t bin;
    float refined_bin;
    float frequency_hz;
    float power;
} onchip_measurement_peak_t;

/** 一个待拟合的有效谐波。 */
typedef struct
{
    uint8_t harmonic_order;
    uint16_t fft_bin;
    float seed_frequency_hz;
    float power;
} onchip_measurement_component_seed_t;

/** 联合最小二乘拟合结果。 */
typedef struct
{
    float frequency_hz[ONCHIP_MEASUREMENT_MAX_TONES];
    float cosine_code[ONCHIP_MEASUREMENT_MAX_TONES];
    float sine_code[ONCHIP_MEASUREMENT_MAX_TONES];
    float amplitude_code[ONCHIP_MEASUREMENT_MAX_TONES];
    float residual_energy;
    uint8_t tone_count;
    uint8_t valid;
} onchip_measurement_fit_t;

volatile onchip_measurement_diagnostics_t
    onchip_measurement_diagnostics;

/** ADC1 DMA 原始帧；32 字节对齐，长度也是缓存行整数倍。 */
static uint16_t onchip_measurement_adc_samples[
    ONCHIP_MEASUREMENT_SAMPLE_COUNT] __attribute__((aligned(32)));

/** 8192 个 float 的 FFT 原地工作区，占用 32 KiB。 */
static float onchip_measurement_fft_work[
    ONCHIP_MEASUREMENT_SAMPLE_COUNT] __attribute__((aligned(32)));

/** 两份输出快照，写非活动项后原子切换。 */
static fpga_measurement_snapshot_t
    onchip_measurement_snapshots[2];

/** 当前活动快照索引。 */
static volatile uint8_t onchip_measurement_active_index;
/** 本帧采集启动时刻。 */
static uint32_t onchip_measurement_capture_started_ms;
/** 允许启动下一帧的时刻。 */
static uint32_t onchip_measurement_next_capture_ms;
/** 已发布快照序号。 */
static uint32_t onchip_measurement_sequence;
/** 最近一帧平均 ADC 码。 */
static float onchip_measurement_mean_code;
/** ADC DMA 半帧事件，只由 HAL 回调置位。 */
static volatile uint8_t onchip_measurement_dma_half_flag;
/** ADC DMA 完整帧事件，只由 HAL 回调置位。 */
static volatile uint8_t onchip_measurement_dma_full_flag;
/** ADC DMA 错误事件，只由 HAL 回调置位。 */
static volatile uint8_t onchip_measurement_dma_error_flag;

/**
 * @brief 判断数据缓存是否已启用。
 * @param 无。
 * @return 已启用返回 1，否则返回 0。
 */
static uint8_t onchip_measurement_dcache_enabled(void)
{
#if (__DCACHE_PRESENT == 1U)
    return ((SCB->CCR & SCB_CCR_DC_Msk) != 0u) ? 1u : 0u;
#else
    return 0u;
#endif
}

/**
 * @brief 在 DMA 写入前清理并失效样本缓冲区。
 * @param 无。
 * @return 无。
 */
static void onchip_measurement_prepare_dma_buffer(void)
{
#if (__DCACHE_PRESENT == 1U)
    if (onchip_measurement_dcache_enabled() != 0u)
    {
        SCB_CleanInvalidateDCache_by_Addr(
            (uint32_t *)onchip_measurement_adc_samples,
            (int32_t)sizeof(onchip_measurement_adc_samples));
    }
#endif
}

/**
 * @brief DMA 完成后失效 CPU 中对应缓存行。
 * @param 无。
 * @return 无。
 */
static void onchip_measurement_finish_dma_buffer(void)
{
#if (__DCACHE_PRESENT == 1U)
    if (onchip_measurement_dcache_enabled() != 0u)
    {
        SCB_InvalidateDCache_by_Addr(
            (uint32_t *)onchip_measurement_adc_samples,
            (int32_t)sizeof(onchip_measurement_adc_samples));
    }
#endif
}

/**
 * @brief 原子领取并清除 ADC HAL 回调写入的三个事件标志。
 * @param half_flag 半帧标志输出；本链路只清除不处理。
 * @param full_flag 完整帧标志输出。
 * @param error_flag ADC 错误标志输出。
 * @return 无。
 */
static void onchip_measurement_claim_adc_flags(
    uint8_t *half_flag,
    uint8_t *full_flag,
    uint8_t *error_flag)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    *half_flag = onchip_measurement_dma_half_flag;
    onchip_measurement_dma_half_flag = 0u;
    *full_flag = onchip_measurement_dma_full_flag;
    onchip_measurement_dma_full_flag = 0u;
    *error_flag = onchip_measurement_dma_error_flag;
    onchip_measurement_dma_error_flag = 0u;
    if (primask == 0u)
    {
        __enable_irq();
    }
}

/**
 * @brief 停止 TIM2 触发和 ADC1 DMA。
 * @param 无。
 * @return 无。
 */
static void onchip_measurement_stop_capture(void)
{
    (void)HAL_TIM_Base_Stop(&htim2);
    (void)HAL_ADC_Stop_DMA(&hadc1);
}

/**
 * @brief 启动一帧 8192 点定时触发采集。
 * @param 无。
 * @return 成功返回 1，失败返回 0。
 */
static uint8_t onchip_measurement_start_capture(void)
{
    HAL_StatusTypeDef status;

    onchip_measurement_dma_half_flag = 0u;
    onchip_measurement_dma_full_flag = 0u;
    onchip_measurement_dma_error_flag = 0u;
    onchip_measurement_prepare_dma_buffer();
    __HAL_TIM_SET_COUNTER(&htim2, 0u);

    status = HAL_ADC_Start_DMA(
        &hadc1,
        (uint32_t *)onchip_measurement_adc_samples,
        ONCHIP_MEASUREMENT_SAMPLE_COUNT);
    onchip_measurement_diagnostics.last_hal_status = (int32_t)status;
    if (status != HAL_OK)
    {
        onchip_measurement_diagnostics.capture_error_count++;
        onchip_measurement_diagnostics.state =
            ONCHIP_MEASUREMENT_STATE_ERROR;
        return 0u;
    }

    status = HAL_TIM_Base_Start(&htim2);
    onchip_measurement_diagnostics.last_hal_status = (int32_t)status;
    if (status != HAL_OK)
    {
        (void)HAL_ADC_Stop_DMA(&hadc1);
        onchip_measurement_diagnostics.capture_error_count++;
        onchip_measurement_diagnostics.state =
            ONCHIP_MEASUREMENT_STATE_ERROR;
        return 0u;
    }

    onchip_measurement_capture_started_ms = HAL_GetTick();
    onchip_measurement_diagnostics.capture_start_count++;
    onchip_measurement_diagnostics.state =
        ONCHIP_MEASUREMENT_STATE_CAPTURING;
    return 1u;
}

/**
 * @brief 返回指定 FFT 频点功率。
 * @param bin 频点索引，范围 1~4095。
 * @return 实部平方加虚部平方。
 */
static float onchip_measurement_bin_power(uint16_t bin)
{
    float real = onchip_measurement_fft_work[2u * bin];
    float imaginary =
        onchip_measurement_fft_work[2u * bin + 1u];
    return real * real + imaginary * imaginary;
}

/**
 * @brief 对谱峰执行对数功率抛物线插值。
 * @param bin 局部最大频点。
 * @return 修正后的非整数频点。
 */
static float onchip_measurement_refine_bin(uint16_t bin)
{
    float left = logf(onchip_measurement_bin_power(
        (uint16_t)(bin - 1u)) + FLT_MIN);
    float center = logf(onchip_measurement_bin_power(bin)
                        + FLT_MIN);
    float right = logf(onchip_measurement_bin_power(
        (uint16_t)(bin + 1u)) + FLT_MIN);
    float denominator = left - 2.0f * center + right;
    float offset = 0.0f;

    if (fabsf(denominator) > 1.0e-12f)
    {
        offset = 0.5f * (left - right) / denominator;
        if (offset > 0.5f)
        {
            offset = 0.5f;
        }
        else if (offset < -0.5f)
        {
            offset = -0.5f;
        }
    }
    return (float)bin + offset;
}

/**
 * @brief 对小型 float 数组升序排序。
 * @param values 待排序数组。
 * @param count 元素数量。
 * @return 无。
 */
static void onchip_measurement_sort_float(float *values,
                                          uint16_t count)
{
    uint16_t index;

    for (index = 1u; index < count; index++)
    {
        float value = values[index];
        uint16_t position = index;

        while ((position > 0u)
               && (values[position - 1u] > value))
        {
            values[position] = values[position - 1u];
            position--;
        }
        values[position] = value;
    }
}

/**
 * @brief 估计指定频带的中位功率噪声底。
 * @param first_bin 首频点。
 * @param last_bin 末频点。
 * @return 128 个均匀抽样频点的中位功率。
 */
static float onchip_measurement_noise_floor(uint16_t first_bin,
                                            uint16_t last_bin)
{
    float samples[ONCHIP_MEASUREMENT_NOISE_SAMPLES];
    uint16_t index;

    for (index = 0u;
         index < ONCHIP_MEASUREMENT_NOISE_SAMPLES;
         index++)
    {
        uint32_t numerator = (uint32_t)index
                             * (last_bin - first_bin);
        uint16_t bin = (uint16_t)(
            first_bin
            + numerator
              / (ONCHIP_MEASUREMENT_NOISE_SAMPLES - 1u));
        samples[index] = onchip_measurement_bin_power(bin);
    }

    onchip_measurement_sort_float(
        samples, ONCHIP_MEASUREMENT_NOISE_SAMPLES);
    return samples[ONCHIP_MEASUREMENT_NOISE_SAMPLES / 2u]
           + FLT_MIN;
}

/**
 * @brief 将局部峰按功率插入固定长度候选表。
 * @param peaks 候选表。
 * @param peak_count 当前候选数地址。
 * @param bin 峰值频点。
 * @param power 峰值功率。
 * @return 无。
 */
static void onchip_measurement_insert_peak(
    onchip_measurement_peak_t *peaks,
    uint8_t *peak_count,
    uint16_t bin,
    float power)
{
    uint8_t position;
    uint8_t limit = *peak_count;
    const float bin_spacing =
        (float)ONCHIP_MEASUREMENT_SAMPLE_RATE_HZ
        / (float)ONCHIP_MEASUREMENT_SAMPLE_COUNT;

    if (limit < ONCHIP_MEASUREMENT_MAX_PEAKS)
    {
        (*peak_count)++;
    }
    else if (power <= peaks[limit - 1u].power)
    {
        return;
    }

    position = (limit < ONCHIP_MEASUREMENT_MAX_PEAKS)
               ? limit
               : (ONCHIP_MEASUREMENT_MAX_PEAKS - 1u);
    while ((position > 0u)
           && (peaks[position - 1u].power < power))
    {
        if (position < ONCHIP_MEASUREMENT_MAX_PEAKS)
        {
            peaks[position] = peaks[position - 1u];
        }
        position--;
    }

    peaks[position].bin = bin;
    peaks[position].refined_bin =
        onchip_measurement_refine_bin(bin);
    peaks[position].frequency_hz =
        peaks[position].refined_bin * bin_spacing;
    peaks[position].power = power;
}

/**
 * @brief 在给定频带寻找局部最大谱峰。
 * @param minimum_hz 搜索下限。
 * @param maximum_hz 搜索上限。
 * @param minimum_uv 最小近似峰值幅度。
 * @param peaks 输出候选表。
 * @return 输出候选数量。
 */
static uint8_t onchip_measurement_find_peaks(
    float minimum_hz,
    float maximum_hz,
    float minimum_uv,
    onchip_measurement_peak_t *peaks)
{
    const float bin_spacing =
        (float)ONCHIP_MEASUREMENT_SAMPLE_RATE_HZ
        / (float)ONCHIP_MEASUREMENT_SAMPLE_COUNT;
    const float uv_per_code =
        ONCHIP_MEASUREMENT_ADC_VREF_UV
        / (ONCHIP_MEASUREMENT_ADC_FULL_SCALE_CODE
           * ONCHIP_MEASUREMENT_FRONT_END_GAIN);
    uint16_t first_bin = (uint16_t)ceilf(
        minimum_hz / bin_spacing);
    uint16_t last_bin = (uint16_t)floorf(
        maximum_hz / bin_spacing);
    float noise_floor;
    float minimum_code = minimum_uv / uv_per_code;
    float minimum_fft = minimum_code
                        * (float)ONCHIP_MEASUREMENT_SAMPLE_COUNT
                        * 0.25f;
    float threshold;
    uint8_t peak_count = 0u;
    uint16_t bin;

    if (first_bin < 2u)
    {
        first_bin = 2u;
    }
    if (last_bin > ((ONCHIP_MEASUREMENT_SAMPLE_COUNT / 2u) - 2u))
    {
        last_bin =
            (ONCHIP_MEASUREMENT_SAMPLE_COUNT / 2u) - 2u;
    }

    noise_floor = onchip_measurement_noise_floor(
        first_bin, last_bin);
    threshold = noise_floor * 16.0f;
    if (threshold < (minimum_fft * minimum_fft))
    {
        threshold = minimum_fft * minimum_fft;
    }

    memset(peaks, 0,
           sizeof(onchip_measurement_peak_t)
           * ONCHIP_MEASUREMENT_MAX_PEAKS);
    for (bin = first_bin; bin <= last_bin; bin++)
    {
        float power = onchip_measurement_bin_power(bin);

        if ((power >= threshold)
            && (power > onchip_measurement_bin_power(
                    (uint16_t)(bin - 1u)))
            && (power >= onchip_measurement_bin_power(
                    (uint16_t)(bin + 1u))))
        {
            onchip_measurement_insert_peak(
                peaks, &peak_count, bin, power);
        }
    }

    return peak_count;
}

/**
 * @brief 在候选峰中寻找给定频率附近的峰。
 * @param peaks 候选峰。
 * @param peak_count 候选数量。
 * @param frequency_hz 目标频率。
 * @param tolerance_hz 最大频差。
 * @return 匹配索引；未找到返回 -1。
 */
static int32_t onchip_measurement_nearest_peak(
    const onchip_measurement_peak_t *peaks,
    uint8_t peak_count,
    float frequency_hz,
    float tolerance_hz)
{
    int32_t best = -1;
    float best_error = tolerance_hz;
    uint8_t index;

    for (index = 0u; index < peak_count; index++)
    {
        float error = fabsf(peaks[index].frequency_hz
                            - frequency_hz);
        if (error <= best_error)
        {
            best_error = error;
            best = (int32_t)index;
        }
    }
    return best;
}

/**
 * @brief 用“基波必须存在 + 其余峰为整数倍”的约束估计基频。
 * @param peaks 信号带候选谱峰。
 * @param peak_count 候选数。
 * @param fundamental_hz 输出基频。
 * @return 成功返回 1，不能形成有效倍频关系返回 0。
 */
static uint8_t onchip_measurement_estimate_fundamental(
    const onchip_measurement_peak_t *peaks,
    uint8_t peak_count,
    float *fundamental_hz)
{
    const float bin_spacing =
        (float)ONCHIP_MEASUREMENT_SAMPLE_RATE_HZ
        / (float)ONCHIP_MEASUREMENT_SAMPLE_COUNT;
    const float tolerance =
        ONCHIP_MEASUREMENT_PEAK_MATCH_BINS * bin_spacing;
    float best_frequency = 0.0f;
    float best_score = -1.0f;
    uint8_t best_matches = 0u;
    uint8_t peak_index;

    if ((peaks == NULL) || (fundamental_hz == NULL)
        || (peak_count == 0u))
    {
        return 0u;
    }

    for (peak_index = 0u;
         peak_index < peak_count;
         peak_index++)
    {
        uint8_t divisor;

        for (divisor = 1u;
             divisor <= ONCHIP_MEASUREMENT_MAX_HARMONIC_ORDER;
             divisor++)
        {
            float candidate =
                peaks[peak_index].frequency_hz / (float)divisor;
            float weighted_sum = 0.0f;
            float weight_sum = 0.0f;
            float score = 0.0f;
            uint8_t matches = 0u;
            uint8_t index;

            if ((candidate < ONCHIP_MEASUREMENT_MIN_SIGNAL_HZ)
                || (candidate
                    > ONCHIP_MEASUREMENT_MAX_SIGNAL_HZ))
            {
                continue;
            }
            if (onchip_measurement_nearest_peak(
                    peaks, peak_count, candidate,
                    tolerance) < 0)
            {
                continue;
            }

            for (index = 0u; index < peak_count; index++)
            {
                float order_float =
                    peaks[index].frequency_hz / candidate;
                uint32_t order = (uint32_t)lroundf(order_float);
                float expected;
                float error;

                if ((order < 1u)
                    || (order
                        > ONCHIP_MEASUREMENT_MAX_HARMONIC_ORDER))
                {
                    continue;
                }
                expected = candidate * (float)order;
                error = fabsf(peaks[index].frequency_hz
                              - expected);
                if (error <= tolerance)
                {
                    float amplitude_weight =
                        sqrtf(peaks[index].power);
                    float frequency_weight =
                        amplitude_weight
                        * (float)(order * order);

                    weighted_sum +=
                        (peaks[index].frequency_hz
                         / (float)order)
                        * frequency_weight;
                    weight_sum += frequency_weight;
                    score += amplitude_weight
                             * (1.0f
                                - 0.4f * error / tolerance);
                    matches++;
                }
            }

            if ((matches >= 2u) && (weight_sum > 0.0f))
            {
                float refined = weighted_sum / weight_sum;

                if ((matches > best_matches)
                    || ((matches == best_matches)
                        && (score > best_score)))
                {
                    best_matches = matches;
                    best_score = score;
                    best_frequency = refined;
                }
            }
        }
    }

    if (best_matches < 2u)
    {
        return 0u;
    }
    *fundamental_hz = best_frequency;
    return 1u;
}

/**
 * @brief 从候选峰中选出最多三个不重复谐波阶次。
 * @param peaks 候选谱峰。
 * @param peak_count 候选数量。
 * @param fundamental_hz 已估计基频。
 * @param seeds 输出分量种子。
 * @return 输出分量数量。
 */
static uint8_t onchip_measurement_select_components(
    const onchip_measurement_peak_t *peaks,
    uint8_t peak_count,
    float fundamental_hz,
    onchip_measurement_component_seed_t *seeds)
{
    const float tolerance =
        ONCHIP_MEASUREMENT_PEAK_MATCH_BINS
        * (float)ONCHIP_MEASUREMENT_SAMPLE_RATE_HZ
        / (float)ONCHIP_MEASUREMENT_SAMPLE_COUNT;
    onchip_measurement_component_seed_t candidates[
        ONCHIP_MEASUREMENT_MAX_PEAKS];
    uint8_t candidate_count = 0u;
    uint8_t index;
    int32_t fundamental_candidate = -1;

    memset(candidates, 0, sizeof(candidates));
    for (index = 0u; index < peak_count; index++)
    {
        uint32_t order = (uint32_t)lroundf(
            peaks[index].frequency_hz / fundamental_hz);
        float error;
        uint8_t existing;

        if ((order < 1u)
            || (order > ONCHIP_MEASUREMENT_MAX_HARMONIC_ORDER))
        {
            continue;
        }
        error = fabsf(peaks[index].frequency_hz
                      - fundamental_hz * (float)order);
        if (error > tolerance)
        {
            continue;
        }

        existing = 0xffu;
        {
            uint8_t candidate_index;
            for (candidate_index = 0u;
                 candidate_index < candidate_count;
                 candidate_index++)
            {
                if (candidates[candidate_index].harmonic_order
                    == (uint8_t)order)
                {
                    existing = candidate_index;
                    break;
                }
            }
        }
        if (existing != 0xffu)
        {
            if (peaks[index].power > candidates[existing].power)
            {
                candidates[existing].fft_bin = peaks[index].bin;
                candidates[existing].seed_frequency_hz =
                    peaks[index].frequency_hz;
                candidates[existing].power = peaks[index].power;
            }
        }
        else if (candidate_count < ONCHIP_MEASUREMENT_MAX_PEAKS)
        {
            candidates[candidate_count].harmonic_order =
                (uint8_t)order;
            candidates[candidate_count].fft_bin = peaks[index].bin;
            candidates[candidate_count].seed_frequency_hz =
                peaks[index].frequency_hz;
            candidates[candidate_count].power = peaks[index].power;
            if (order == 1u)
            {
                fundamental_candidate = candidate_count;
            }
            candidate_count++;
        }
    }

    if (fundamental_candidate < 0)
    {
        return 0u;
    }

    seeds[0] = candidates[fundamental_candidate];
    {
        uint8_t selected = 1u;
        while ((selected < FPGA_PROTOCOL_COMPONENT_MAX)
               && (selected < candidate_count))
        {
            int32_t best = -1;
            float best_power = -1.0f;
            uint8_t candidate_index;

            for (candidate_index = 0u;
                 candidate_index < candidate_count;
                 candidate_index++)
            {
                uint8_t already_selected = 0u;
                uint8_t selected_index;

                for (selected_index = 0u;
                     selected_index < selected;
                     selected_index++)
                {
                    if (seeds[selected_index].harmonic_order
                        == candidates[candidate_index].harmonic_order)
                    {
                        already_selected = 1u;
                    }
                }
                if ((already_selected == 0u)
                    && (candidates[candidate_index].power
                        > best_power))
                {
                    best = candidate_index;
                    best_power =
                        candidates[candidate_index].power;
                }
            }
            if (best < 0)
            {
                break;
            }
            seeds[selected] = candidates[best];
            selected++;
        }

        for (index = 1u; index < selected; index++)
        {
            onchip_measurement_component_seed_t value = seeds[index];
            uint8_t position = index;

            while ((position > 0u)
                   && (seeds[position - 1u].harmonic_order
                       > value.harmonic_order))
            {
                seeds[position] = seeds[position - 1u];
                position--;
            }
            seeds[position] = value;
        }
        return selected;
    }
}

/**
 * @brief 用带部分主元的高斯消元求解小型线性方程。
 * @param augmented dim 行、dim+1 列增广矩阵。
 * @param dim 方程维数，最大 8。
 * @param solution 输出解向量。
 * @return 成功返回 1，矩阵病态返回 0。
 */
static uint8_t onchip_measurement_solve(
    float augmented[ONCHIP_MEASUREMENT_MAX_MATRIX]
                   [ONCHIP_MEASUREMENT_MAX_MATRIX + 1u],
    uint8_t dim,
    float *solution)
{
    uint8_t pivot;

    for (pivot = 0u; pivot < dim; pivot++)
    {
        uint8_t best_row = pivot;
        float best_value = fabsf(augmented[pivot][pivot]);
        uint8_t row;

        for (row = (uint8_t)(pivot + 1u); row < dim; row++)
        {
            float value = fabsf(augmented[row][pivot]);
            if (value > best_value)
            {
                best_value = value;
                best_row = row;
            }
        }
        if (best_value < 1.0e-5f)
        {
            return 0u;
        }
        if (best_row != pivot)
        {
            uint8_t column;
            for (column = pivot; column <= dim; column++)
            {
                float temporary = augmented[pivot][column];
                augmented[pivot][column] =
                    augmented[best_row][column];
                augmented[best_row][column] = temporary;
            }
        }

        {
            float inverse = 1.0f / augmented[pivot][pivot];
            uint8_t column;
            for (column = pivot; column <= dim; column++)
            {
                augmented[pivot][column] *= inverse;
            }
        }

        for (row = 0u; row < dim; row++)
        {
            if (row != pivot)
            {
                float factor = augmented[row][pivot];
                uint8_t column;
                for (column = pivot; column <= dim; column++)
                {
                    augmented[row][column] -=
                        factor * augmented[pivot][column];
                }
            }
        }
    }

    for (pivot = 0u; pivot < dim; pivot++)
    {
        solution[pivot] = augmented[pivot][dim];
    }
    return 1u;
}

/**
 * @brief 对多个已知频率的正弦/余弦基执行联合最小二乘。
 * @param frequencies_hz 各音调频率。
 * @param tone_count 音调数量，最大 4。
 * @param fit 输出拟合结果。
 * @return 成功返回 1，参数或矩阵异常返回 0。
 */
static uint8_t onchip_measurement_fit_tones(
    const float *frequencies_hz,
    uint8_t tone_count,
    onchip_measurement_fit_t *fit)
{
    float matrix[ONCHIP_MEASUREMENT_MAX_MATRIX]
                [ONCHIP_MEASUREMENT_MAX_MATRIX];
    float vector[ONCHIP_MEASUREMENT_MAX_MATRIX];
    float augmented[ONCHIP_MEASUREMENT_MAX_MATRIX]
                   [ONCHIP_MEASUREMENT_MAX_MATRIX + 1u];
    float solution[ONCHIP_MEASUREMENT_MAX_MATRIX];
    float oscillator_real[ONCHIP_MEASUREMENT_MAX_TONES];
    float oscillator_imaginary[ONCHIP_MEASUREMENT_MAX_TONES];
    float root_real[ONCHIP_MEASUREMENT_MAX_TONES];
    float root_imaginary[ONCHIP_MEASUREMENT_MAX_TONES];
    float sample_energy = 0.0f;
    uint8_t dim = (uint8_t)(tone_count * 2u);
    uint8_t tone;
    uint32_t sample_index;

    if ((frequencies_hz == NULL) || (fit == NULL)
        || (tone_count == 0u)
        || (tone_count > ONCHIP_MEASUREMENT_MAX_TONES))
    {
        return 0u;
    }

    memset(fit, 0, sizeof(*fit));
    memset(matrix, 0, sizeof(matrix));
    memset(vector, 0, sizeof(vector));
    for (tone = 0u; tone < tone_count; tone++)
    {
        float angle = 2.0f * ONCHIP_MEASUREMENT_PI
                      * frequencies_hz[tone]
                      / (float)ONCHIP_MEASUREMENT_SAMPLE_RATE_HZ;
        root_real[tone] = cosf(angle);
        root_imaginary[tone] = sinf(angle);
        oscillator_real[tone] = 1.0f;
        oscillator_imaginary[tone] = 0.0f;
        fit->frequency_hz[tone] = frequencies_hz[tone];
    }

    for (sample_index = 0u;
         sample_index < ONCHIP_MEASUREMENT_SAMPLE_COUNT;
         sample_index++)
    {
        float basis[ONCHIP_MEASUREMENT_MAX_MATRIX];
        float sample = (float)onchip_measurement_adc_samples[
            sample_index] - onchip_measurement_mean_code;
        uint8_t row;

        for (tone = 0u; tone < tone_count; tone++)
        {
            basis[2u * tone] = oscillator_real[tone];
            basis[2u * tone + 1u] =
                oscillator_imaginary[tone];
        }
        sample_energy += sample * sample;
        for (row = 0u; row < dim; row++)
        {
            uint8_t column;
            vector[row] += basis[row] * sample;
            for (column = row; column < dim; column++)
            {
                matrix[row][column] +=
                    basis[row] * basis[column];
            }
        }

        for (tone = 0u; tone < tone_count; tone++)
        {
            float next_real =
                oscillator_real[tone] * root_real[tone]
                - oscillator_imaginary[tone]
                  * root_imaginary[tone];
            oscillator_imaginary[tone] =
                oscillator_real[tone] * root_imaginary[tone]
                + oscillator_imaginary[tone]
                  * root_real[tone];
            oscillator_real[tone] = next_real;
        }

        if (((sample_index + 1u) & 255u) == 0u)
        {
            for (tone = 0u; tone < tone_count; tone++)
            {
                float magnitude = sqrtf(
                    oscillator_real[tone]
                    * oscillator_real[tone]
                    + oscillator_imaginary[tone]
                      * oscillator_imaginary[tone]);
                if ((!isfinite(magnitude))
                    || (magnitude <= 0.0f))
                {
                    return 0u;
                }
                oscillator_real[tone] /= magnitude;
                oscillator_imaginary[tone] /= magnitude;
            }
        }
    }

    {
        uint8_t row;
        for (row = 0u; row < dim; row++)
        {
            uint8_t column;
            for (column = 0u; column < dim; column++)
            {
                augmented[row][column] =
                    (column >= row)
                    ? matrix[row][column]
                    : matrix[column][row];
            }
            augmented[row][dim] = vector[row];
        }
    }

    if (onchip_measurement_solve(
            augmented, dim, solution) == 0u)
    {
        return 0u;
    }

    fit->residual_energy = sample_energy;
    {
        uint8_t coefficient;
        for (coefficient = 0u; coefficient < dim; coefficient++)
        {
            fit->residual_energy -=
                solution[coefficient] * vector[coefficient];
        }
    }
    if (fit->residual_energy < 0.0f)
    {
        fit->residual_energy = 0.0f;
    }

    for (tone = 0u; tone < tone_count; tone++)
    {
        fit->cosine_code[tone] = solution[2u * tone];
        fit->sine_code[tone] = solution[2u * tone + 1u];
        fit->amplitude_code[tone] = sqrtf(
            fit->cosine_code[tone] * fit->cosine_code[tone]
            + fit->sine_code[tone] * fit->sine_code[tone]);
        if (!isfinite(fit->amplitude_code[tone]))
        {
            return 0u;
        }
    }
    fit->tone_count = tone_count;
    fit->valid = 1u;
    return 1u;
}

/**
 * @brief 在 FFT 基频附近用五点残差网格细化，并完成最终联合 I/Q 拟合。
 * @param initial_fundamental_hz FFT/倍频约束的初值。
 * @param seeds 有效谐波阶次。
 * @param component_count 有效谐波数量。
 * @param interference_hz 高频干扰频率，零表示不拟合干扰项。
 * @param fit 输出最终拟合。
 * @param refined_fundamental_hz 输出细化基频。
 * @return 成功返回 1，否则返回 0。
 */
static uint8_t onchip_measurement_refine_and_fit(
    float initial_fundamental_hz,
    const onchip_measurement_component_seed_t *seeds,
    uint8_t component_count,
    float interference_hz,
    onchip_measurement_fit_t *fit,
    float *refined_fundamental_hz)
{
    const float bin_spacing =
        (float)ONCHIP_MEASUREMENT_SAMPLE_RATE_HZ
        / (float)ONCHIP_MEASUREMENT_SAMPLE_COUNT;
    const float grid_step = bin_spacing * 0.25f;
    float residual[ONCHIP_MEASUREMENT_FIT_GRID_COUNT];
    onchip_measurement_fit_t grid_fit;
    uint8_t include_interference =
        (interference_hz > 0.0f) ? 1u : 0u;
    uint8_t tone_count =
        (uint8_t)(component_count + include_interference);
    uint8_t grid;
    uint8_t best_grid = 0u;
    float best_residual = FLT_MAX;
    float best_frequency = initial_fundamental_hz;

    if ((seeds == NULL) || (fit == NULL)
        || (refined_fundamental_hz == NULL)
        || (component_count == 0u)
        || (tone_count > ONCHIP_MEASUREMENT_MAX_TONES))
    {
        return 0u;
    }

    for (grid = 0u;
         grid < ONCHIP_MEASUREMENT_FIT_GRID_COUNT;
         grid++)
    {
        float frequencies[ONCHIP_MEASUREMENT_MAX_TONES];
        float fundamental =
            initial_fundamental_hz
            + ((float)grid - 2.0f) * grid_step;
        uint8_t index;

        for (index = 0u; index < component_count; index++)
        {
            frequencies[index] =
                fundamental
                * (float)seeds[index].harmonic_order;
        }
        if (include_interference != 0u)
        {
            frequencies[component_count] = interference_hz;
        }

        if (onchip_measurement_fit_tones(
                frequencies, tone_count, &grid_fit) == 0u)
        {
            residual[grid] = FLT_MAX;
            continue;
        }
        residual[grid] = grid_fit.residual_energy;
        if (residual[grid] < best_residual)
        {
            best_residual = residual[grid];
            best_grid = grid;
            best_frequency = fundamental;
            *fit = grid_fit;
        }
    }

    if ((best_residual == FLT_MAX) || (fit->valid == 0u))
    {
        return 0u;
    }

    if ((best_grid > 0u)
        && (best_grid
            < (ONCHIP_MEASUREMENT_FIT_GRID_COUNT - 1u)))
    {
        float left = residual[best_grid - 1u];
        float center = residual[best_grid];
        float right = residual[best_grid + 1u];
        float denominator = left - 2.0f * center + right;

        if (isfinite(denominator)
            && (fabsf(denominator) > 1.0e-6f))
        {
            float offset = 0.5f * (left - right) / denominator;
            if (offset > 1.0f)
            {
                offset = 1.0f;
            }
            else if (offset < -1.0f)
            {
                offset = -1.0f;
            }
            best_frequency += offset * grid_step;
        }
    }

    {
        float frequencies[ONCHIP_MEASUREMENT_MAX_TONES];
        uint8_t index;
        for (index = 0u; index < component_count; index++)
        {
            frequencies[index] =
                best_frequency
                * (float)seeds[index].harmonic_order;
        }
        if (include_interference != 0u)
        {
            frequencies[component_count] = interference_hz;
        }
        if (onchip_measurement_fit_tones(
                frequencies, tone_count, fit) == 0u)
        {
            return 0u;
        }
    }

    *refined_fundamental_hz = best_frequency;
    return 1u;
}

/**
 * @brief 由有效谐波的联合拟合系数重建一个基波周期并求峰峰值。
 * @param fit 联合拟合结果。
 * @param seeds 有效谐波阶次。
 * @param component_count 有效谐波数量，不含干扰项。
 * @return 重建峰峰值，单位 ADC 码。
 */
static float onchip_measurement_reconstructed_vpp(
    const onchip_measurement_fit_t *fit,
    const onchip_measurement_component_seed_t *seeds,
    uint8_t component_count)
{
    float minimum = FLT_MAX;
    float maximum = -FLT_MAX;
    uint32_t point;

    for (point = 0u;
         point < ONCHIP_MEASUREMENT_RECONSTRUCTION_POINTS;
         point++)
    {
        float base_angle =
            2.0f * ONCHIP_MEASUREMENT_PI * (float)point
            / (float)ONCHIP_MEASUREMENT_RECONSTRUCTION_POINTS;
        float value = 0.0f;
        uint8_t component;

        for (component = 0u;
             component < component_count;
             component++)
        {
            float angle = base_angle
                          * (float)seeds[component].harmonic_order;
            value += fit->cosine_code[component] * cosf(angle)
                     + fit->sine_code[component] * sinf(angle);
        }
        if (value < minimum)
        {
            minimum = value;
        }
        if (value > maximum)
        {
            maximum = value;
        }
    }
    return maximum - minimum;
}

/**
 * @brief 将有效信号模型重建为三周期 int16 显示数据。
 * @param fit 最终拟合结果。
 * @param seeds 有效谐波阶次。
 * @param component_count 有效谐波数量。
 * @param output 输出数组。
 * @return 无。
 */
static void onchip_measurement_build_time_display(
    const onchip_measurement_fit_t *fit,
    const onchip_measurement_component_seed_t *seeds,
    uint8_t component_count,
    int16_t *output)
{
    uint16_t point;

    for (point = 0u;
         point < ONCHIP_MEASUREMENT_TIME_POINTS;
         point++)
    {
        float base_angle =
            6.0f * ONCHIP_MEASUREMENT_PI * (float)point
            / (float)ONCHIP_MEASUREMENT_TIME_POINTS;
        float value = 0.0f;
        uint8_t component;

        for (component = 0u;
             component < component_count;
             component++)
        {
            float angle = base_angle
                          * (float)seeds[component].harmonic_order;
            value += fit->cosine_code[component] * cosf(angle)
                     + fit->sine_code[component] * sinf(angle);
        }
        if (value > 32767.0f)
        {
            value = 32767.0f;
        }
        else if (value < -32768.0f)
        {
            value = -32768.0f;
        }
        output[point] = (int16_t)lroundf(value);
    }
}

/**
 * @brief 将 0~550 kHz FFT 幅度压缩为协议固定的 1312 点。
 * @param output 输出频谱。
 * @return 无。
 */
static void onchip_measurement_build_spectrum(uint16_t *output)
{
    const float bin_spacing =
        (float)ONCHIP_MEASUREMENT_SAMPLE_RATE_HZ
        / (float)ONCHIP_MEASUREMENT_SAMPLE_COUNT;
    uint16_t point;

    for (point = 0u;
         point < FPGA_PROTOCOL_SPECTRUM_COUNT;
         point++)
    {
        float begin_hz =
            (float)point * ONCHIP_MEASUREMENT_MAX_SIGNAL_HZ
            / (float)FPGA_PROTOCOL_SPECTRUM_COUNT;
        float end_hz =
            (float)(point + 1u)
            * ONCHIP_MEASUREMENT_MAX_SIGNAL_HZ
            / (float)FPGA_PROTOCOL_SPECTRUM_COUNT;
        uint16_t begin_bin = (uint16_t)floorf(
            begin_hz / bin_spacing);
        uint16_t end_bin = (uint16_t)ceilf(
            end_hz / bin_spacing);
        float maximum = 0.0f;
        uint16_t bin;

        if (begin_bin < 1u)
        {
            begin_bin = 1u;
        }
        if (end_bin <= begin_bin)
        {
            end_bin = (uint16_t)(begin_bin + 1u);
        }
        for (bin = begin_bin; bin <= end_bin; bin++)
        {
            float magnitude = sqrtf(
                onchip_measurement_bin_power(bin));
            if (magnitude > maximum)
            {
                maximum = magnitude;
            }
        }

        maximum *= 400.0f
                   / (float)ONCHIP_MEASUREMENT_SAMPLE_COUNT;
        if (maximum > 65535.0f)
        {
            maximum = 65535.0f;
        }
        output[point] = (uint16_t)lroundf(maximum);
    }
}

/**
 * @brief 分析当前 DMA 帧并发布一份兼容显示层的完整快照。
 * @param 无。
 * @return 成功返回 1，否则返回 0。
 */
static uint8_t onchip_measurement_analyze(void)
{
    const float uv_per_code =
        ONCHIP_MEASUREMENT_ADC_VREF_UV
        / (ONCHIP_MEASUREMENT_ADC_FULL_SCALE_CODE
           * ONCHIP_MEASUREMENT_FRONT_END_GAIN);
    onchip_measurement_peak_t peaks[
        ONCHIP_MEASUREMENT_MAX_PEAKS];
    onchip_measurement_peak_t interference_peaks[
        ONCHIP_MEASUREMENT_MAX_PEAKS];
    onchip_measurement_component_seed_t seeds[
        FPGA_PROTOCOL_COMPONENT_MAX];
    onchip_measurement_fit_t fit;
    fpga_measurement_snapshot_t *snapshot;
    uint64_t sum = 0u;
    uint16_t minimum = 4095u;
    uint16_t maximum = 0u;
    uint32_t sample_index;
    uint8_t peak_count;
    uint8_t interference_peak_count;
    uint8_t component_count;
    float fundamental_hz;
    float refined_fundamental_hz;
    float interference_hz = 0.0f;
    float rms_code_square = 0.0f;
    float vpp_code;
    float dc_uv;
    uint8_t target_index;
    uint8_t component;

    for (sample_index = 0u;
         sample_index < ONCHIP_MEASUREMENT_SAMPLE_COUNT;
         sample_index++)
    {
        uint16_t sample = onchip_measurement_adc_samples[
            sample_index];
        sum += sample;
        if (sample < minimum)
        {
            minimum = sample;
        }
        if (sample > maximum)
        {
            maximum = sample;
        }
    }
    onchip_measurement_mean_code =
        (float)sum / (float)ONCHIP_MEASUREMENT_SAMPLE_COUNT;
    onchip_measurement_diagnostics.last_adc_min = minimum;
    onchip_measurement_diagnostics.last_adc_max = maximum;

    if ((minimum <= ONCHIP_MEASUREMENT_CLIP_LOW_CODE)
        || (maximum >= ONCHIP_MEASUREMENT_CLIP_HIGH_CODE))
    {
        onchip_measurement_diagnostics.clipped_frame_count++;
    }

    if (onchip_fft_8192_forward(
            onchip_measurement_adc_samples,
            onchip_measurement_mean_code,
            onchip_measurement_fft_work)
        != ONCHIP_FFT_8192_STATUS_OK)
    {
        return 0u;
    }

    peak_count = onchip_measurement_find_peaks(
        ONCHIP_MEASUREMENT_MIN_SIGNAL_HZ,
        ONCHIP_MEASUREMENT_MAX_SIGNAL_HZ,
        ONCHIP_MEASUREMENT_MIN_COMPONENT_UV,
        peaks);
    if (onchip_measurement_estimate_fundamental(
            peaks, peak_count, &fundamental_hz) == 0u)
    {
        return 0u;
    }
    component_count = onchip_measurement_select_components(
        peaks, peak_count, fundamental_hz, seeds);
    if (component_count < 2u)
    {
        return 0u;
    }

    interference_peak_count = onchip_measurement_find_peaks(
        ONCHIP_MEASUREMENT_INTERFERENCE_MIN_HZ,
        ONCHIP_MEASUREMENT_INTERFERENCE_MAX_HZ,
        ONCHIP_MEASUREMENT_MIN_INTERFERENCE_UV,
        interference_peaks);
    if (interference_peak_count > 0u)
    {
        interference_hz =
            interference_peaks[0].frequency_hz;
    }

    memset(&fit, 0, sizeof(fit));
    if (onchip_measurement_refine_and_fit(
            fundamental_hz, seeds, component_count,
            interference_hz, &fit,
            &refined_fundamental_hz) == 0u)
    {
        return 0u;
    }

    for (component = 0u;
         component < component_count;
         component++)
    {
        rms_code_square +=
            fit.amplitude_code[component]
            * fit.amplitude_code[component] * 0.5f;
    }
    vpp_code = onchip_measurement_reconstructed_vpp(
        &fit, seeds, component_count);
    dc_uv = onchip_measurement_mean_code
            * ONCHIP_MEASUREMENT_ADC_VREF_UV
            / ONCHIP_MEASUREMENT_ADC_FULL_SCALE_CODE
            - ONCHIP_MEASUREMENT_ADC_CENTER_UV;
    dc_uv /= ONCHIP_MEASUREMENT_FRONT_END_GAIN;

    target_index =
        (uint8_t)(onchip_measurement_active_index ^ 1u);
    snapshot = &onchip_measurement_snapshots[target_index];
    memset(snapshot, 0, sizeof(*snapshot));

    onchip_measurement_sequence++;
    snapshot->header.protocol_version = FPGA_PROTOCOL_VERSION;
    snapshot->header.frame_type = FPGA_PROTOCOL_FRAME_TYPE_FULL;
    snapshot->header.header_bytes = FPGA_PROTOCOL_HEADER_BYTES;
    snapshot->header.frame_seq = onchip_measurement_sequence;
    snapshot->header.timestamp_50m =
        (uint64_t)HAL_GetTick() * 50000u;
    if ((minimum <= ONCHIP_MEASUREMENT_CLIP_LOW_CODE)
        || (maximum >= ONCHIP_MEASUREMENT_CLIP_HIGH_CODE))
    {
        snapshot->header.flags |= FPGA_PROTOCOL_STATUS_ADC_OTR;
    }
    snapshot->header.time_sample_rate_hz =
        (uint32_t)lroundf(refined_fundamental_hz * 350.0f);
    snapshot->header.time_count = ONCHIP_MEASUREMENT_TIME_POINTS;
    snapshot->header.captured_cycles = 3u;
    snapshot->header.time_format = 1u;
    snapshot->header.fft_sample_rate_hz =
        ONCHIP_MEASUREMENT_SAMPLE_RATE_HZ;
    snapshot->header.fft_length =
        ONCHIP_MEASUREMENT_SAMPLE_COUNT;
    snapshot->header.spectrum_count =
        FPGA_PROTOCOL_SPECTRUM_COUNT;
    snapshot->header.bin_spacing_mhz =
        (uint32_t)lroundf(
            ONCHIP_MEASUREMENT_MAX_SIGNAL_HZ * 1000.0f
            / (float)(FPGA_PROTOCOL_SPECTRUM_COUNT - 1u));
    snapshot->header.spectrum_format = 1u;
    snapshot->header.window_type = 1u;
    snapshot->header.component_count = component_count;
    snapshot->header.vpp_uv =
        (uint32_t)lroundf(vpp_code * uv_per_code);
    snapshot->header.vrms_uv =
        (uint32_t)lroundf(sqrtf(rms_code_square)
                          * uv_per_code);
    snapshot->header.fundamental_mhz =
        (uint32_t)lroundf(refined_fundamental_hz * 1000.0f);
    snapshot->header.dc_offset_uv = (int32_t)lroundf(dc_uv);
    snapshot->header.calibration_revision = 1u;
    snapshot->header.adc_min_code = (int16_t)minimum;
    snapshot->header.adc_max_code = (int16_t)maximum;

    for (component = 0u;
         component < component_count;
         component++)
    {
        fpga_protocol_component_t *output =
            &snapshot->header.component[component];
        output->frequency_mhz = (uint32_t)lroundf(
            refined_fundamental_hz
            * (float)seeds[component].harmonic_order
            * 1000.0f);
        output->amplitude_peak_uv = (uint32_t)lroundf(
            fit.amplitude_code[component] * uv_per_code);
        output->fft_bin = (uint16_t)lroundf(
            output->frequency_mhz
            * (float)ONCHIP_MEASUREMENT_SAMPLE_COUNT
            / ((float)ONCHIP_MEASUREMENT_SAMPLE_RATE_HZ
               * 1000.0f));
        output->harmonic_order =
            seeds[component].harmonic_order;
        output->flags = FPGA_PROTOCOL_COMPONENT_VALID
                        | FPGA_PROTOCOL_COMPONENT_IQ_READY;
    }

    onchip_measurement_build_time_display(
        &fit, seeds, component_count, snapshot->time_samples);
    onchip_measurement_build_spectrum(snapshot->spectrum);
    snapshot->valid = 1u;

    __DMB();
    onchip_measurement_active_index = target_index;

    onchip_measurement_diagnostics.last_fundamental_mhz =
        snapshot->header.fundamental_mhz;
    onchip_measurement_diagnostics.last_vpp_uv =
        snapshot->header.vpp_uv;
    onchip_measurement_diagnostics.last_vrms_uv =
        snapshot->header.vrms_uv;
    onchip_measurement_diagnostics.last_component_count =
        component_count;
    onchip_measurement_diagnostics.interference_removed =
        (interference_hz > 0.0f) ? 1u : 0u;
    onchip_measurement_diagnostics.last_interference_mhz =
        (uint32_t)lroundf(interference_hz * 1000.0f);
    return 1u;
}

uint8_t onchip_measurement_init(void)
{
    HAL_StatusTypeDef status;

    memset((void *)&onchip_measurement_diagnostics, 0,
           sizeof(onchip_measurement_diagnostics));
    memset(onchip_measurement_snapshots, 0,
           sizeof(onchip_measurement_snapshots));
    onchip_measurement_active_index = 0u;
    onchip_measurement_sequence = 0u;
    onchip_measurement_stop_capture();

    status = HAL_ADCEx_Calibration_Start(
        &hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
    onchip_measurement_diagnostics.last_hal_status =
        (int32_t)status;
    if (status != HAL_OK)
    {
        onchip_measurement_diagnostics.capture_error_count++;
        onchip_measurement_diagnostics.state =
            ONCHIP_MEASUREMENT_STATE_ERROR;
        return 0u;
    }

    return onchip_measurement_start_capture();
}

void onchip_measurement_process(void)
{
    uint8_t half_flag;
    uint8_t full_flag;
    uint8_t error_flag;
    uint32_t now = HAL_GetTick();

    onchip_measurement_claim_adc_flags(
        &half_flag, &full_flag, &error_flag);
    (void)half_flag;

    if (error_flag != 0u)
    {
        onchip_measurement_stop_capture();
        onchip_measurement_diagnostics.capture_error_count++;
        onchip_measurement_diagnostics.state =
            ONCHIP_MEASUREMENT_STATE_ERROR;
        onchip_measurement_next_capture_ms =
            now + ONCHIP_MEASUREMENT_RESTART_DELAY_MS;
    }

    if ((onchip_measurement_diagnostics.state
         == ONCHIP_MEASUREMENT_STATE_CAPTURING)
        && (full_flag != 0u))
    {
        uint32_t analysis_started_ms;

        onchip_measurement_stop_capture();
        onchip_measurement_finish_dma_buffer();
        onchip_measurement_diagnostics.capture_complete_count++;
        onchip_measurement_diagnostics.state =
            ONCHIP_MEASUREMENT_STATE_PROCESSING;
        analysis_started_ms = HAL_GetTick();

        if (onchip_measurement_analyze() != 0u)
        {
            onchip_measurement_diagnostics.analysis_success_count++;
        }
        else
        {
            onchip_measurement_diagnostics.analysis_error_count++;
        }
        now = HAL_GetTick();
        onchip_measurement_diagnostics.last_analysis_time_ms =
            now - analysis_started_ms;
        onchip_measurement_diagnostics.last_total_time_ms =
            now - onchip_measurement_capture_started_ms;
        onchip_measurement_next_capture_ms =
            now + ONCHIP_MEASUREMENT_RESTART_DELAY_MS;
        onchip_measurement_diagnostics.state =
            ONCHIP_MEASUREMENT_STATE_WAITING;
    }
    else if ((onchip_measurement_diagnostics.state
              == ONCHIP_MEASUREMENT_STATE_CAPTURING)
             && ((now - onchip_measurement_capture_started_ms)
                 > ONCHIP_MEASUREMENT_CAPTURE_TIMEOUT_MS))
    {
        onchip_measurement_stop_capture();
        onchip_measurement_diagnostics.capture_timeout_count++;
        onchip_measurement_diagnostics.state =
            ONCHIP_MEASUREMENT_STATE_ERROR;
        onchip_measurement_next_capture_ms =
            now + ONCHIP_MEASUREMENT_RESTART_DELAY_MS;
    }

    if (((onchip_measurement_diagnostics.state
          == ONCHIP_MEASUREMENT_STATE_WAITING)
         || (onchip_measurement_diagnostics.state
             == ONCHIP_MEASUREMENT_STATE_ERROR))
        && ((int32_t)(now - onchip_measurement_next_capture_ms)
            >= 0))
    {
        (void)onchip_measurement_start_capture();
    }
}

uint8_t onchip_measurement_get_snapshot(
    const fpga_measurement_snapshot_t **snapshot)
{
    const fpga_measurement_snapshot_t *active;

    if (snapshot == NULL)
    {
        return 0u;
    }
    active = &onchip_measurement_snapshots[
        onchip_measurement_active_index];
    *snapshot = active;
    return active->valid;
}

/**
 * @brief ADC1 DMA 半帧完成回调，仅置位一个事件标志。
 * @param hadc ADC 句柄。
 * @return 无。
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
    {
        onchip_measurement_dma_half_flag = 1u;
    }
}

/**
 * @brief ADC1 DMA 完整帧完成回调，仅置位一个事件标志。
 * @param hadc ADC 句柄。
 * @return 无。
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
    {
        onchip_measurement_dma_full_flag = 1u;
    }
}

/**
 * @brief ADC1 错误回调，仅置位一个事件标志。
 * @param hadc ADC 句柄。
 * @return 无。
 */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
    {
        onchip_measurement_dma_error_flag = 1u;
    }
}
