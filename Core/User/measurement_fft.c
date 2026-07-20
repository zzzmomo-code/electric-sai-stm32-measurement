/**
 * @file measurement_fft.c
 * @brief 双通道 600 kSPS、65536 点 F32 FFT 测量与诊断实现。
 *
 * 模块用途：保存 ADC1/ADC2 同步样本对，暂停 TIM2 后顺序完成两个通道的
 * 时域测量和自定义 F32 FFT，发布电压、Vpp、RMS、频率、THD、相位及波形类型。
 * GPIO 引脚映射：PC4/ADC1_INP4 为 CH1，PB1/ADC2_INP5 为 CH2；本模块不直接操作 GPIO。
 * 依赖的外设和 CubeIDE 配置：ADC1/ADC2 Dual Regular Simultaneous、TIM2 TRGO
 * 600 kHz、DMA1 Stream0 Circular Word/Word；工作区分别映射到 D1/D2 SRAM。
 * 初始化方法：system_init() 调用 measurement_fft_init()。
 * 调用方法：adc_dual_process() 提交样本对，主循环调用 measurement_fft_process()。
 */

#include "system.h"

#include <math.h>
#include <string.h>

#define MEASUREMENT_FFT_CHANNEL_COUNT 2u
#define MEASUREMENT_FFT_SECOND_CHANNEL 1u
#define MEASUREMENT_FFT_PI 3.14159265358979323846f
#define MEASUREMENT_FFT_SQRT_2 1.41421356237309504880f
#define MEASUREMENT_FFT_SQRT_3 1.73205080756887729353f
#define MEASUREMENT_FFT_ESTIMATED_ADC_VREF_V 3.3f
#define MEASUREMENT_FFT_ADC_FULL_SCALE_CODE 65535.0f
#define MEASUREMENT_FFT_ESTIMATED_VOLTS_PER_CODE \
    (MEASUREMENT_FFT_ESTIMATED_ADC_VREF_V / MEASUREMENT_FFT_ADC_FULL_SCALE_CODE)
#define MEASUREMENT_FFT_DECIMATION_FACTOR 1u
#define MEASUREMENT_FFT_MAX_HARMONIC_ORDER 10u
#define MEASUREMENT_FFT_HARMONIC_RADIUS 2u
#define MEASUREMENT_FFT_DISPLAY_GUARD_MS 500u
#define MEASUREMENT_FFT_SETTLE_SAMPLE_COUNT 1024u
#define MEASUREMENT_FFT_MINIMUM_SPAN_CODE 128u
#define MEASUREMENT_FFT_CLIP_MARGIN_CODE 64u
#define MEASUREMENT_FFT_CHANNEL_MATCH_TOLERANCE_BINS 1.5f
#define MEASUREMENT_FFT_SINE_H3_MAX 0.055f
#define MEASUREMENT_FFT_SINE_H5_MAX 0.040f
#define MEASUREMENT_FFT_TRIANGLE_H3_MIN 0.055f
#define MEASUREMENT_FFT_TRIANGLE_H3_MAX 0.200f
#define MEASUREMENT_FFT_TRIANGLE_H5_MAX 0.080f
#define MEASUREMENT_FFT_SQUARE_H3_MIN 0.200f
#define MEASUREMENT_FFT_SQUARE_H5_MIN 0.080f

/** 采集、处理、显示和过渡状态。 */
typedef enum
{
    MEASUREMENT_FFT_STATE_CAPTURE = 0,
    MEASUREMENT_FFT_STATE_READY,
    MEASUREMENT_FFT_STATE_DISPLAY,
    MEASUREMENT_FFT_STATE_SETTLING
} measurement_fft_state_t;

/** 电压换算来源。 */
typedef enum
{
    MEASUREMENT_FFT_VOLTAGE_INVALID = 0,
    MEASUREMENT_FFT_VOLTAGE_CALIBRATED,
    MEASUREMENT_FFT_VOLTAGE_ESTIMATED
} measurement_fft_voltage_status_t;

/** 单通道整帧时域统计。 */
typedef struct
{
    uint16_t minimum_code;
    uint16_t maximum_code;
    float mean_raw_code;
    float rms_ac_code;
} measurement_fft_time_metrics_t;

/** 单通道频域分析中需要跨通道保留的紧凑特征。 */
typedef struct
{
    float peak_power;
    float peak_offset;
    float peak_position;
    float peak_real;
    float peak_imaginary;
    float harmonic_ratio_3;
    float harmonic_ratio_5;
    float thd_percent;
    uint16_t peak_bin;
    uint8_t thd_harmonic_count;
    measurement_wave_type_t wave_type;
} measurement_fft_channel_analysis_t;

/** 65536 个同步样本对，占用 256 KiB，固定映射到 D2 SRAM。 */
static adc_dual_sample_pair_t measurement_fft_sample_pairs[MEASUREMENT_FFT_LENGTH]
    __attribute__((section(".adc_sample_pairs"), aligned(32)));

/** 两个通道顺序复用的 65536 元素 F32 原地 FFT 工作区，占用 256 KiB。 */
static float32_t measurement_fft_work[MEASUREMENT_FFT_LENGTH]
    __attribute__((section(".fft_f32_work"), aligned(32)));

/** 当前整帧已经写入的同步样本对数量。 */
static uint32_t measurement_fft_capture_index;
/** 非零表示样本帧已收齐并等待主循环处理。 */
static uint8_t measurement_fft_frame_ready;
/** 非零表示主循环正在读取样本帧并执行 FFT。 */
static uint8_t measurement_fft_processing;
/** 显示结束后主动丢弃的模拟前端稳定样本数。 */
static uint32_t measurement_fft_settle_count;
/** 当前测量状态。 */
static measurement_fft_state_t measurement_fft_state;
/** HMI 显示空档开始时刻。 */
static uint32_t measurement_fft_display_start_ms;
/** 结果发布序号。 */
static uint32_t measurement_fft_result_sequence;
/** 最新一次完整测量的诊断快照，保留原有全部字段。 */
measurement_fft_diagnostics_t measurement_fft_diagnostics;
/** 面向串口屏的 64 点相对幅度频谱。 */
static measurement_fft_spectrum_t measurement_fft_spectrum;
/** 两个通道的软件电压校准参数。 */
static measurement_fft_calibration_t
    measurement_fft_calibration[MEASUREMENT_FFT_CHANNEL_COUNT];
/** 兼容旧 ADS8688 顺序提交接口的一对临时样本。 */
static uint16_t measurement_fft_legacy_pair[MEASUREMENT_FFT_CHANNEL_COUNT];
static uint8_t measurement_fft_legacy_pair_mask;

/**
 * @brief 使用分段线性公式校准 FFT 插值得到的频率。
 * @param raw_frequency_hz 未经本公式校准的 FFT 插值频率，单位为 Hz。
 * @return 校准后的非负频率；输入无效、非正或结果为负时返回 0 Hz。
 * @note 纯数值计算，不访问外设，也不修改模块状态；40000 Hz 使用低频段公式。
 */
float measurement_fft_calibrate_frequency(float raw_frequency_hz)
{
    float calibrated_frequency_hz;

    if ((!isfinite(raw_frequency_hz)) || (raw_frequency_hz <= 0.0f))
    {
        return 0.0f;
    }

    if (raw_frequency_hz <= MEASUREMENT_FFT_FREQUENCY_SPLIT_HZ)
    {
        calibrated_frequency_hz =
            MEASUREMENT_FFT_LOW_FREQUENCY_GAIN * raw_frequency_hz
            + MEASUREMENT_FFT_LOW_FREQUENCY_OFFSET_HZ;
    }
    else
    {
        calibrated_frequency_hz =
            MEASUREMENT_FFT_HIGH_FREQUENCY_GAIN * raw_frequency_hz
            + MEASUREMENT_FFT_HIGH_FREQUENCY_OFFSET_HZ;
    }

    return (calibrated_frequency_hz > 0.0f)
               ? calibrated_frequency_hz
               : 0.0f;
}

/**
 * @brief 使能 Cortex-M7 DWT 周期计数器。
 * @param 无。
 * @return 无。
 * @note 只用于记录两通道 FFT 与分析耗时，不改变外设时钟。
 */
static void measurement_fft_enable_cycle_counter(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief 取得样本对中指定通道的首样本地址。
 * @param channel 通道号，0 为 CH1，1 为 CH2。
 * @return 指向首个 uint16_t 原始码的指针。
 */
static const uint16_t *measurement_fft_channel_samples(uint8_t channel)
{
    return (channel == 0u) ? &measurement_fft_sample_pairs[0].ch1_code
                           : &measurement_fft_sample_pairs[0].ch2_code;
}

/**
 * @brief 计算指定通道的整帧最值、均值和去直流 RMS。
 * @param channel 通道号。
 * @return 当前完整帧的时域统计。
 * @note 使用双精度平方累加，避免 65536 点大直流偏置造成明显消减误差。
 */
static measurement_fft_time_metrics_t measurement_fft_analyze_time_domain(
    uint8_t channel)
{
    const uint16_t *samples = measurement_fft_channel_samples(channel);
    uint64_t sum = 0u;
    double square_sum = 0.0;
    uint32_t index;
    measurement_fft_time_metrics_t metrics;

    metrics.minimum_code = UINT16_MAX;
    metrics.maximum_code = 0u;
    for (index = 0u; index < MEASUREMENT_FFT_LENGTH; index++)
    {
        const uint16_t code = samples[index * 2u];
        sum += code;
        if (code < metrics.minimum_code)
        {
            metrics.minimum_code = code;
        }
        if (code > metrics.maximum_code)
        {
            metrics.maximum_code = code;
        }
    }

    metrics.mean_raw_code = (float)sum / (float)MEASUREMENT_FFT_LENGTH;
    for (index = 0u; index < MEASUREMENT_FFT_LENGTH; index++)
    {
        const double centered =
            (double)samples[index * 2u] - (double)metrics.mean_raw_code;
        square_sum += centered * centered;
    }
    metrics.rms_ac_code =
        (float)sqrt(square_sum / (double)MEASUREMENT_FFT_LENGTH);
    return metrics;
}

/**
 * @brief 将原始码统计换算为电压统计。
 * @param metrics 整帧时域统计。
 * @param channel 通道号。
 * @param span_vpp 返回原始最大最小差对应的 Vpp。
 * @param dc_voltage 返回平均电压。
 * @param rms_voltage 返回去直流 RMS 电压。
 * @return 校准、估算或无效状态。
 */
static measurement_fft_voltage_status_t measurement_fft_convert_time_metrics(
    measurement_fft_time_metrics_t metrics,
    uint8_t channel,
    float *span_vpp,
    float *dc_voltage,
    float *rms_voltage)
{
    float volts_per_code;
    float offset_v;
    measurement_fft_voltage_status_t status;

    if ((channel >= MEASUREMENT_FFT_CHANNEL_COUNT) || (span_vpp == 0)
        || (dc_voltage == 0) || (rms_voltage == 0))
    {
        return MEASUREMENT_FFT_VOLTAGE_INVALID;
    }

    if (measurement_fft_calibration[channel].valid != 0u)
    {
        volts_per_code = measurement_fft_calibration[channel].volts_per_code;
        offset_v = measurement_fft_calibration[channel].offset_v;
        status = MEASUREMENT_FFT_VOLTAGE_CALIBRATED;
    }
    else
    {
        volts_per_code = MEASUREMENT_FFT_ESTIMATED_VOLTS_PER_CODE;
        offset_v = 0.0f;
        status = MEASUREMENT_FFT_VOLTAGE_ESTIMATED;
    }

    *span_vpp = (float)(metrics.maximum_code - metrics.minimum_code)
                * fabsf(volts_per_code);
    *dc_voltage = metrics.mean_raw_code * volts_per_code + offset_v;
    *rms_voltage = metrics.rms_ac_code * fabsf(volts_per_code);
    if ((!isfinite(*span_vpp)) || (!isfinite(*dc_voltage))
        || (!isfinite(*rms_voltage)))
    {
        *span_vpp = 0.0f;
        *dc_voltage = 0.0f;
        *rms_voltage = 0.0f;
        return MEASUREMENT_FFT_VOLTAGE_INVALID;
    }
    return status;
}

/**
 * @brief 判断整帧是否接近 ADC 上下量程。
 * @param metrics 整帧时域统计。
 * @return 接近任一端量程返回 1，否则返回 0。
 */
static uint8_t measurement_fft_is_clipped(
    measurement_fft_time_metrics_t metrics)
{
    return ((metrics.minimum_code <= MEASUREMENT_FFT_CLIP_MARGIN_CODE)
            || (metrics.maximum_code
                >= (uint16_t)(65535u - MEASUREMENT_FFT_CLIP_MARGIN_CODE)))
               ? 1u
               : 0u;
}

/**
 * @brief 读取压缩实数频谱中一个频点的功率。
 * @param spectrum F32 packed 频谱。
 * @param bin 频点号。
 * @return 实部平方与虚部平方之和；越界时返回零。
 */
static float measurement_fft_bin_power(const float32_t *spectrum,
                                        uint32_t bin)
{
    float real;
    float imaginary;

    if ((spectrum == 0) || (bin >= (MEASUREMENT_FFT_LENGTH / 2u)))
    {
        return 0.0f;
    }
    if (bin == 0u)
    {
        return spectrum[0] * spectrum[0];
    }
    real = spectrum[2u * bin];
    imaginary = spectrum[2u * bin + 1u];
    return real * real + imaginary * imaginary;
}

/**
 * @brief 在 1 Hz 至 120 kHz 范围内寻找最大频谱功率点。
 * @param spectrum packed 频谱。
 * @param peak_power 返回最大功率。
 * @return 最大功率频点号。
 */
static uint16_t measurement_fft_find_peak_bin(const float32_t *spectrum,
                                               float *peak_power)
{
    uint32_t maximum_bin =
        (uint32_t)ceilf(MEASUREMENT_FFT_SPECTRUM_MAX_HZ
                        / (MEASUREMENT_FFT_SAMPLE_RATE_HZ
                           / (float)MEASUREMENT_FFT_LENGTH));
    uint32_t bin;
    uint16_t peak_bin = 0u;
    float maximum_power = 0.0f;

    if (maximum_bin >= (MEASUREMENT_FFT_LENGTH / 2u - 1u))
    {
        maximum_bin = MEASUREMENT_FFT_LENGTH / 2u - 2u;
    }
    for (bin = 1u; bin <= maximum_bin; bin++)
    {
        const float power = measurement_fft_bin_power(spectrum, bin);
        if (power > maximum_power)
        {
            maximum_power = power;
            peak_bin = (uint16_t)bin;
        }
    }
    if (peak_power != 0)
    {
        *peak_power = maximum_power;
    }
    return peak_bin;
}

/**
 * @brief 用三点抛物线插值计算峰值相对整数频点的偏移。
 * @param spectrum packed 频谱。
 * @param peak_bin 整数峰值频点。
 * @return 限制在 -0.5 至 0.5 的亚频点偏移。
 */
static float measurement_fft_interpolate_peak(const float32_t *spectrum,
                                               uint16_t peak_bin)
{
    float left;
    float center;
    float right;
    float denominator;
    float offset;

    if ((peak_bin == 0u)
        || (peak_bin >= (MEASUREMENT_FFT_LENGTH / 2u - 1u)))
    {
        return 0.0f;
    }
    left = measurement_fft_bin_power(spectrum, peak_bin - 1u);
    center = measurement_fft_bin_power(spectrum, peak_bin);
    right = measurement_fft_bin_power(spectrum, peak_bin + 1u);
    denominator = left - 2.0f * center + right;
    if ((!isfinite(denominator)) || (fabsf(denominator) < 1.0e-20f))
    {
        return 0.0f;
    }
    offset = 0.5f * (left - right) / denominator;
    if (!isfinite(offset))
    {
        return 0.0f;
    }
    if (offset > 0.5f)
    {
        offset = 0.5f;
    }
    else if (offset < -0.5f)
    {
        offset = -0.5f;
    }
    return offset;
}

/**
 * @brief 累加目标频点附近固定半径内的频谱功率。
 * @param spectrum packed 频谱。
 * @param center_bin 中心频点。
 * @return 邻域功率和。
 */
static float measurement_fft_band_power(const float32_t *spectrum,
                                         uint32_t center_bin)
{
    uint32_t first;
    uint32_t last;
    uint32_t bin;
    float total = 0.0f;

    first = (center_bin > MEASUREMENT_FFT_HARMONIC_RADIUS)
                ? center_bin - MEASUREMENT_FFT_HARMONIC_RADIUS
                : 1u;
    last = center_bin + MEASUREMENT_FFT_HARMONIC_RADIUS;
    if (last >= (MEASUREMENT_FFT_LENGTH / 2u))
    {
        last = MEASUREMENT_FFT_LENGTH / 2u - 1u;
    }
    for (bin = first; bin <= last; bin++)
    {
        total += measurement_fft_bin_power(spectrum, bin);
    }
    return total;
}

/**
 * @brief 计算指定次数谐波相对于基波的幅度比。
 * @param spectrum packed 频谱。
 * @param peak_position 插值后的基波频点位置。
 * @param order 谐波次数。
 * @param fundamental_power 基波邻域功率。
 * @return 谐波幅度比；谐波越过 Nyquist 时返回零。
 */
static float measurement_fft_harmonic_ratio(const float32_t *spectrum,
                                             float peak_position,
                                             uint8_t order,
                                             float fundamental_power)
{
    const float target = peak_position * (float)order;
    uint32_t target_bin;
    float power;

    if ((fundamental_power <= 0.0f) || (!isfinite(target))
        || (target >= (float)(MEASUREMENT_FFT_LENGTH / 2u
                              - MEASUREMENT_FFT_HARMONIC_RADIUS)))
    {
        return 0.0f;
    }
    target_bin = (uint32_t)(target + 0.5f);
    power = measurement_fft_band_power(spectrum, target_bin);
    return (power > 0.0f) ? sqrtf(power / fundamental_power) : 0.0f;
}

/**
 * @brief 计算 Nyquist 以下可用谐波的总谐波失真。
 * @param spectrum packed 频谱。
 * @param peak_position 插值后的基波频点位置。
 * @param harmonic_count 返回实际纳入的谐波数。
 * @return THD 百分比。
 */
static float measurement_fft_calculate_thd(const float32_t *spectrum,
                                            float peak_position,
                                            uint8_t *harmonic_count)
{
    const uint32_t fundamental_bin = (uint32_t)(peak_position + 0.5f);
    const float fundamental_power =
        measurement_fft_band_power(spectrum, fundamental_bin);
    float harmonic_power = 0.0f;
    uint8_t count = 0u;
    uint8_t order;

    if (harmonic_count != 0)
    {
        *harmonic_count = 0u;
    }
    if (fundamental_power <= 0.0f)
    {
        return 0.0f;
    }
    for (order = 2u; order <= MEASUREMENT_FFT_MAX_HARMONIC_ORDER; order++)
    {
        const float target = peak_position * (float)order;
        if (target >= (float)(MEASUREMENT_FFT_LENGTH / 2u
                              - MEASUREMENT_FFT_HARMONIC_RADIUS))
        {
            break;
        }
        harmonic_power += measurement_fft_band_power(
            spectrum, (uint32_t)(target + 0.5f));
        count++;
    }
    if (harmonic_count != 0)
    {
        *harmonic_count = count;
    }
    return (count != 0u) ? 100.0f * sqrtf(harmonic_power / fundamental_power)
                         : 0.0f;
}

/**
 * @brief 根据三次、五次谐波幅度比分类常见波形。
 * @param ratio_3 三次谐波幅度比。
 * @param ratio_5 五次谐波幅度比。
 * @return 正弦、方波、三角波或未知。
 */
static measurement_wave_type_t measurement_fft_classify_wave(float ratio_3,
                                                               float ratio_5)
{
    if ((ratio_3 >= MEASUREMENT_FFT_SQUARE_H3_MIN)
        && (ratio_5 >= MEASUREMENT_FFT_SQUARE_H5_MIN))
    {
        return MEASUREMENT_WAVE_SQUARE;
    }
    if ((ratio_3 >= MEASUREMENT_FFT_TRIANGLE_H3_MIN)
        && (ratio_3 < MEASUREMENT_FFT_TRIANGLE_H3_MAX)
        && (ratio_5 <= MEASUREMENT_FFT_TRIANGLE_H5_MAX))
    {
        return MEASUREMENT_WAVE_TRIANGLE;
    }
    if ((ratio_3 < MEASUREMENT_FFT_SINE_H3_MAX)
        && (ratio_5 < MEASUREMENT_FFT_SINE_H5_MAX))
    {
        return MEASUREMENT_WAVE_SINE;
    }
    return MEASUREMENT_WAVE_UNKNOWN;
}

/**
 * @brief 将角度约束到 [-180, 180) 区间。
 * @param phase_deg 任意角度。
 * @return 约束后的角度；非有限输入返回零。
 */
static float measurement_fft_wrap_phase(float phase_deg)
{
    if (!isfinite(phase_deg))
    {
        return 0.0f;
    }
    while (phase_deg >= 180.0f)
    {
        phase_deg -= 360.0f;
    }
    while (phase_deg < -180.0f)
    {
        phase_deg += 360.0f;
    }
    return phase_deg;
}

/**
 * @brief 根据 RMS 和波形类型换算 Vpp。
 * @param rms_voltage 去直流 RMS 电压。
 * @param wave_type 波形分类。
 * @param fallback_vpp 未知波形时使用的整帧最大最小差。
 * @return 有限的 Vpp。
 */
static float measurement_fft_rms_to_vpp(float rms_voltage,
                                         measurement_wave_type_t wave_type,
                                         float fallback_vpp)
{
    float value = fallback_vpp;
    if (wave_type == MEASUREMENT_WAVE_SINE)
    {
        value = 2.0f * MEASUREMENT_FFT_SQRT_2 * rms_voltage;
    }
    else if (wave_type == MEASUREMENT_WAVE_SQUARE)
    {
        value = 2.0f * rms_voltage;
    }
    else if (wave_type == MEASUREMENT_WAVE_TRIANGLE)
    {
        value = 2.0f * MEASUREMENT_FFT_SQRT_3 * rms_voltage;
    }
    return isfinite(value) ? value : 0.0f;
}

/**
 * @brief 将 CH1 频谱压缩为 0 至 120 kHz 的 64 点相对功率快照。
 * @param spectrum packed 频谱。
 * @param reference_power CH1 基波邻域功率。
 * @return 无。
 */
static void measurement_fft_build_spectrum(const float32_t *spectrum,
                                            float reference_power)
{
    const float point_width = MEASUREMENT_FFT_SPECTRUM_MAX_HZ
                              / (float)MEASUREMENT_FFT_SPECTRUM_POINT_COUNT;
    const float bin_width = MEASUREMENT_FFT_SAMPLE_RATE_HZ
                            / (float)MEASUREMENT_FFT_LENGTH;
    uint32_t point;

    measurement_fft_spectrum.point_width_hz = point_width;
    measurement_fft_spectrum.nyquist_hz = MEASUREMENT_FFT_SAMPLE_RATE_HZ * 0.5f;
    measurement_fft_spectrum.sequence++;
    measurement_fft_spectrum.valid = (reference_power > 0.0f) ? 1u : 0u;
    for (point = 0u; point < MEASUREMENT_FFT_SPECTRUM_POINT_COUNT; point++)
    {
        uint32_t first_bin = (uint32_t)((float)point * point_width / bin_width);
        uint32_t last_bin =
            (uint32_t)((float)(point + 1u) * point_width / bin_width);
        uint32_t bin;
        float maximum_power = 0.0f;
        float relative_db;

        if (first_bin == 0u)
        {
            first_bin = 1u;
        }
        if (last_bin >= (MEASUREMENT_FFT_LENGTH / 2u))
        {
            last_bin = MEASUREMENT_FFT_LENGTH / 2u - 1u;
        }
        for (bin = first_bin; bin <= last_bin; bin++)
        {
            const float power = measurement_fft_bin_power(spectrum, bin);
            if (power > maximum_power)
            {
                maximum_power = power;
            }
        }
        if ((reference_power <= 0.0f) || (maximum_power <= 0.0f))
        {
            measurement_fft_spectrum.relative_db_x10[point] =
                MEASUREMENT_FFT_SPECTRUM_FLOOR_DB_X10;
            continue;
        }
        relative_db = 100.0f * log10f(maximum_power / reference_power);
        if (!isfinite(relative_db)
            || (relative_db < (float)MEASUREMENT_FFT_SPECTRUM_FLOOR_DB_X10))
        {
            relative_db = (float)MEASUREMENT_FFT_SPECTRUM_FLOOR_DB_X10;
        }
        if (relative_db > 0.0f)
        {
            relative_db = 0.0f;
        }
        measurement_fft_spectrum.relative_db_x10[point] =
            (int16_t)lrintf(relative_db);
    }
}

/**
 * @brief 执行一个通道的 FFT 并提取紧凑频域特征。
 * @param channel 通道号。
 * @param mean_code 该通道整帧平均原始码。
 * @param build_display_spectrum 非零时同时生成 64 点显示频谱。
 * @param analysis 返回通道频域特征。
 * @return FFT 执行状态。
 */
static fft_f32_65536_status_t measurement_fft_analyze_channel(
    uint8_t channel,
    float mean_code,
    uint8_t build_display_spectrum,
    measurement_fft_channel_analysis_t *analysis)
{
    fft_f32_65536_status_t status;
    float fundamental_power;

    memset(analysis, 0, sizeof(*analysis));
    status = fft_f32_65536_forward(measurement_fft_channel_samples(channel),
                                   2u,
                                   mean_code,
                                   measurement_fft_work);
    if (status != FFT_F32_65536_STATUS_OK)
    {
        return status;
    }

    analysis->peak_bin = measurement_fft_find_peak_bin(
        measurement_fft_work, &analysis->peak_power);
    analysis->peak_offset = measurement_fft_interpolate_peak(
        measurement_fft_work, analysis->peak_bin);
    analysis->peak_position =
        (float)analysis->peak_bin + analysis->peak_offset;
    fundamental_power = measurement_fft_band_power(
        measurement_fft_work, analysis->peak_bin);
    analysis->harmonic_ratio_3 = measurement_fft_harmonic_ratio(
        measurement_fft_work, analysis->peak_position, 3u, fundamental_power);
    analysis->harmonic_ratio_5 = measurement_fft_harmonic_ratio(
        measurement_fft_work, analysis->peak_position, 5u, fundamental_power);
    analysis->wave_type = measurement_fft_classify_wave(
        analysis->harmonic_ratio_3, analysis->harmonic_ratio_5);
    analysis->thd_percent = measurement_fft_calculate_thd(
        measurement_fft_work,
        analysis->peak_position,
        &analysis->thd_harmonic_count);
    if (analysis->peak_bin != 0u)
    {
        analysis->peak_real = measurement_fft_work[2u * analysis->peak_bin];
        analysis->peak_imaginary =
            measurement_fft_work[2u * analysis->peak_bin + 1u];
    }
    if (build_display_spectrum != 0u)
    {
        measurement_fft_build_spectrum(measurement_fft_work,
                                       fundamental_power);
    }
    return FFT_F32_65536_STATUS_OK;
}

/**
 * @brief 更新固定采样率和频点间隔诊断字段。
 * @param 无。
 * @return 无。
 */
static void measurement_fft_update_timing_diagnostics(void)
{
    measurement_fft_diagnostics.raw_sample_rate_hz =
        MEASUREMENT_FFT_SAMPLE_RATE_HZ;
    measurement_fft_diagnostics.effective_sample_rate_hz =
        MEASUREMENT_FFT_SAMPLE_RATE_HZ;
    measurement_fft_diagnostics.bin_width_hz =
        MEASUREMENT_FFT_SAMPLE_RATE_HZ / (float)MEASUREMENT_FFT_LENGTH;
    measurement_fft_diagnostics.decimation_factor =
        MEASUREMENT_FFT_DECIMATION_FACTOR;
}

/**
 * @brief 将诊断快照发布为串口屏使用的测量结果。
 * @param valid 双通道交流核心结果是否完整。
 * @param quality 本帧质量。
 * @param mode CH1 模式。
 * @param valid_mask CH1 有效字段。
 * @param estimated_mask CH1 估算字段。
 * @param secondary_mode CH2 模式。
 * @param secondary_valid_mask CH2 有效字段。
 * @param secondary_estimated_mask CH2 估算字段。
 * @param fault_mask 双通道故障位。
 * @return 无。
 */
static void measurement_fft_publish(uint8_t valid,
                                    measurement_fft_quality_t quality,
                                    measurement_mode_t mode,
                                    uint16_t valid_mask,
                                    uint16_t estimated_mask,
                                    measurement_mode_t secondary_mode,
                                    uint16_t secondary_valid_mask,
                                    uint16_t secondary_estimated_mask,
                                    uint8_t fault_mask)
{
    measurement_result_t result;

    measurement_fft_result_sequence++;
    result.dc_voltage = measurement_fft_diagnostics.dc_voltage;
    result.amplitude_vpp = measurement_fft_diagnostics.amplitude_vpp;
    result.rms_voltage = measurement_fft_diagnostics.rms_voltage;
    result.frequency_hz = measurement_fft_diagnostics.peak_frequency_hz;
    result.thd_percent = measurement_fft_diagnostics.thd_percent;
    result.secondary_dc_voltage = measurement_fft_diagnostics.secondary_dc_voltage;
    result.secondary_amplitude_vpp =
        measurement_fft_diagnostics.secondary_amplitude_vpp;
    result.secondary_rms_voltage =
        measurement_fft_diagnostics.secondary_rms_voltage;
    result.secondary_frequency_hz =
        measurement_fft_diagnostics.secondary_peak_frequency_hz;
    result.secondary_thd_percent =
        measurement_fft_diagnostics.secondary_thd_percent;
    result.phase_deg = measurement_fft_diagnostics.phase_deg;
    result.wave_type = measurement_fft_diagnostics.wave_type;
    result.secondary_wave_type = measurement_fft_diagnostics.secondary_wave_type;
    result.mode = mode;
    result.secondary_mode = secondary_mode;
    result.valid_mask = valid_mask;
    result.secondary_valid_mask = secondary_valid_mask;
    result.estimated_mask = estimated_mask;
    result.secondary_estimated_mask = secondary_estimated_mask;
    result.fault_mask = fault_mask;
    result.valid = valid;
    result.sequence = measurement_fft_result_sequence;
    measurement_result_publish(&result);
    measurement_fft_diagnostics.publish_count++;
    measurement_fft_diagnostics.quality = quality;
    measurement_fft_diagnostics.result_valid = valid;
}

void measurement_fft_init(void)
{
    uint32_t index;

    memset(&measurement_fft_diagnostics, 0, sizeof(measurement_fft_diagnostics));
    memset(&measurement_fft_spectrum, 0, sizeof(measurement_fft_spectrum));
    memset(measurement_fft_calibration, 0, sizeof(measurement_fft_calibration));
    measurement_fft_capture_index = 0u;
    measurement_fft_frame_ready = 0u;
    measurement_fft_processing = 0u;
    measurement_fft_settle_count = 0u;
    measurement_fft_state = MEASUREMENT_FFT_STATE_CAPTURE;
    measurement_fft_display_start_ms = 0u;
    measurement_fft_result_sequence = 0u;
    measurement_fft_legacy_pair_mask = 0u;
    measurement_fft_diagnostics.init_status =
        ((MEASUREMENT_FFT_LENGTH == FFT_F32_65536_LENGTH)
         && ((MEASUREMENT_FFT_LENGTH & (MEASUREMENT_FFT_LENGTH - 1u)) == 0u))
            ? (int32_t)FFT_F32_65536_STATUS_OK
            : (int32_t)FFT_F32_65536_STATUS_INVALID_ARGUMENT;
    measurement_fft_diagnostics.quality =
        (measurement_fft_diagnostics.init_status == 0)
            ? MEASUREMENT_FFT_QUALITY_NOT_READY
            : MEASUREMENT_FFT_QUALITY_INIT_ERROR;
    measurement_fft_update_timing_diagnostics();
    measurement_fft_spectrum.point_width_hz =
        MEASUREMENT_FFT_SPECTRUM_MAX_HZ
        / (float)MEASUREMENT_FFT_SPECTRUM_POINT_COUNT;
    measurement_fft_spectrum.nyquist_hz =
        MEASUREMENT_FFT_SAMPLE_RATE_HZ * 0.5f;
    for (index = 0u; index < MEASUREMENT_FFT_SPECTRUM_POINT_COUNT; index++)
    {
        measurement_fft_spectrum.relative_db_x10[index] =
            MEASUREMENT_FFT_SPECTRUM_FLOOR_DB_X10;
    }
    measurement_fft_enable_cycle_counter();
}

uint8_t measurement_fft_ingest_pair(uint16_t ch1_raw_code,
                                    uint16_t ch2_raw_code)
{
    if (measurement_fft_diagnostics.init_status != 0)
    {
        return 0u;
    }
    if (measurement_fft_state == MEASUREMENT_FFT_STATE_SETTLING)
    {
        measurement_fft_diagnostics.discarded_sample_count++;
        measurement_fft_settle_count++;
        if (measurement_fft_settle_count >= MEASUREMENT_FFT_SETTLE_SAMPLE_COUNT)
        {
            measurement_fft_capture_index = 0u;
            measurement_fft_settle_count = 0u;
            measurement_fft_state = MEASUREMENT_FFT_STATE_CAPTURE;
        }
        return 0u;
    }
    if ((measurement_fft_state != MEASUREMENT_FFT_STATE_CAPTURE)
        || (measurement_fft_frame_ready != 0u)
        || (measurement_fft_processing != 0u)
        || (measurement_fft_capture_index >= MEASUREMENT_FFT_LENGTH))
    {
        measurement_fft_diagnostics.discarded_sample_count++;
        return 0u;
    }

    measurement_fft_sample_pairs[measurement_fft_capture_index].ch1_code =
        ch1_raw_code;
    measurement_fft_sample_pairs[measurement_fft_capture_index].ch2_code =
        ch2_raw_code;
    measurement_fft_capture_index++;
    if (measurement_fft_capture_index == MEASUREMENT_FFT_LENGTH)
    {
        measurement_fft_frame_ready = 1u;
        measurement_fft_state = MEASUREMENT_FFT_STATE_READY;
        measurement_fft_diagnostics.window_count++;
    }
    return 1u;
}

void measurement_fft_ingest_sample(uint8_t channel, uint16_t raw_code)
{
    if (channel > MEASUREMENT_FFT_SECOND_CHANNEL)
    {
        return;
    }
    measurement_fft_legacy_pair[channel] = raw_code;
    measurement_fft_legacy_pair_mask |= (uint8_t)(1u << channel);
    if (measurement_fft_legacy_pair_mask == 0x03u)
    {
        measurement_fft_legacy_pair_mask = 0u;
        (void)measurement_fft_ingest_pair(measurement_fft_legacy_pair[0],
                                          measurement_fft_legacy_pair[1]);
    }
}

uint8_t measurement_fft_sampling_required(void)
{
    return ((measurement_fft_state == MEASUREMENT_FFT_STATE_CAPTURE)
            || (measurement_fft_state == MEASUREMENT_FFT_STATE_SETTLING))
               ? 1u
               : 0u;
}

void measurement_fft_resynchronize(void)
{
    if ((measurement_fft_diagnostics.init_status != 0)
        || (measurement_fft_processing != 0u))
    {
        return;
    }
    measurement_fft_capture_index = 0u;
    measurement_fft_frame_ready = 0u;
    measurement_fft_settle_count = 0u;
    measurement_fft_state = MEASUREMENT_FFT_STATE_SETTLING;
    measurement_fft_diagnostics.capture_resync_count++;
}

void measurement_fft_process(void)
{
    measurement_fft_time_metrics_t time_metrics[MEASUREMENT_FFT_CHANNEL_COUNT];
    measurement_fft_channel_analysis_t analysis[MEASUREMENT_FFT_CHANNEL_COUNT];
    measurement_fft_voltage_status_t voltage_status[MEASUREMENT_FFT_CHANNEL_COUNT];
    float span_vpp[MEASUREMENT_FFT_CHANNEL_COUNT];
    uint16_t span_code[MEASUREMENT_FFT_CHANNEL_COUNT];
    fft_f32_65536_status_t fft_status[MEASUREMENT_FFT_CHANNEL_COUNT];
    measurement_fft_quality_t quality;
    measurement_mode_t mode[MEASUREMENT_FFT_CHANNEL_COUNT];
    uint16_t valid_mask[MEASUREMENT_FFT_CHANNEL_COUNT];
    uint16_t estimated_mask[MEASUREMENT_FFT_CHANNEL_COUNT];
    uint8_t fault_mask;
    uint8_t valid;
    uint8_t channel;
    uint32_t cycle_start;

    if ((measurement_fft_state == MEASUREMENT_FFT_STATE_READY)
        && (measurement_fft_frame_ready != 0u))
    {
        HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);//反转开发板LED
        measurement_fft_processing = 1u;
        for (channel = 0u; channel < MEASUREMENT_FFT_CHANNEL_COUNT; channel++)
        {
            time_metrics[channel] = measurement_fft_analyze_time_domain(channel);
            span_code[channel] =
                time_metrics[channel].maximum_code - time_metrics[channel].minimum_code;
        }
        measurement_fft_diagnostics.ch1_frame_min_code = time_metrics[0].minimum_code;
        measurement_fft_diagnostics.ch1_frame_max_code = time_metrics[0].maximum_code;
        measurement_fft_diagnostics.ch1_frame_mean_code =
            (uint16_t)(time_metrics[0].mean_raw_code + 0.5f);
        measurement_fft_diagnostics.ch2_frame_min_code = time_metrics[1].minimum_code;
        measurement_fft_diagnostics.ch2_frame_max_code = time_metrics[1].maximum_code;
        measurement_fft_diagnostics.ch2_frame_mean_code =
            (uint16_t)(time_metrics[1].mean_raw_code + 0.5f);
        measurement_fft_diagnostics.clipping_mask =
            (uint8_t)((measurement_fft_is_clipped(time_metrics[0]) != 0u ? 0x01u : 0u)
                      | (measurement_fft_is_clipped(time_metrics[1]) != 0u ? 0x02u : 0u));
        voltage_status[0] = measurement_fft_convert_time_metrics(
            time_metrics[0], 0u, &span_vpp[0],
            &measurement_fft_diagnostics.dc_voltage,
            &measurement_fft_diagnostics.rms_voltage);
        voltage_status[1] = measurement_fft_convert_time_metrics(
            time_metrics[1], 1u, &span_vpp[1],
            &measurement_fft_diagnostics.secondary_dc_voltage,
            &measurement_fft_diagnostics.secondary_rms_voltage);
        measurement_fft_diagnostics.voltage_calibrated_mask =
            (uint8_t)((voltage_status[0] == MEASUREMENT_FFT_VOLTAGE_CALIBRATED ? 0x01u : 0u)
                      | (voltage_status[1] == MEASUREMENT_FFT_VOLTAGE_CALIBRATED ? 0x02u : 0u));
        measurement_fft_diagnostics.voltage_estimated_mask =
            (uint8_t)((voltage_status[0] == MEASUREMENT_FFT_VOLTAGE_ESTIMATED ? 0x01u : 0u)
                      | (voltage_status[1] == MEASUREMENT_FFT_VOLTAGE_ESTIMATED ? 0x02u : 0u));

        cycle_start = DWT->CYCCNT;
        fft_status[0] = measurement_fft_analyze_channel(
            0u, time_metrics[0].mean_raw_code, 1u, &analysis[0]);
        fft_status[1] = measurement_fft_analyze_channel(
            1u, time_metrics[1].mean_raw_code, 0u, &analysis[1]);
        measurement_fft_diagnostics.last_fft_cycles = DWT->CYCCNT - cycle_start;
        measurement_fft_diagnostics.peak_bin = analysis[0].peak_bin;
        measurement_fft_diagnostics.secondary_peak_bin = analysis[1].peak_bin;
        measurement_fft_diagnostics.peak_offset_bins = analysis[0].peak_offset;
        measurement_fft_diagnostics.raw_peak_frequency_hz =
            analysis[0].peak_position * measurement_fft_diagnostics.bin_width_hz;
        measurement_fft_diagnostics.peak_frequency_hz =
            measurement_fft_calibrate_frequency(
                measurement_fft_diagnostics.raw_peak_frequency_hz);
        measurement_fft_diagnostics.secondary_raw_peak_frequency_hz =
            analysis[1].peak_position * measurement_fft_diagnostics.bin_width_hz;
        measurement_fft_diagnostics.secondary_peak_frequency_hz =
            measurement_fft_calibrate_frequency(
                measurement_fft_diagnostics.secondary_raw_peak_frequency_hz);
        measurement_fft_diagnostics.harmonic_ratio_3 = analysis[0].harmonic_ratio_3;
        measurement_fft_diagnostics.harmonic_ratio_5 = analysis[0].harmonic_ratio_5;
        measurement_fft_diagnostics.secondary_harmonic_ratio_3 =
            analysis[1].harmonic_ratio_3;
        measurement_fft_diagnostics.secondary_harmonic_ratio_5 =
            analysis[1].harmonic_ratio_5;
        measurement_fft_diagnostics.wave_type = analysis[0].wave_type;
        measurement_fft_diagnostics.secondary_wave_type = analysis[1].wave_type;
        measurement_fft_diagnostics.thd_percent = analysis[0].thd_percent;
        measurement_fft_diagnostics.thd_harmonic_count = analysis[0].thd_harmonic_count;
        measurement_fft_diagnostics.secondary_thd_percent = analysis[1].thd_percent;
        measurement_fft_diagnostics.secondary_thd_harmonic_count =
            analysis[1].thd_harmonic_count;
        measurement_fft_diagnostics.amplitude_vpp = measurement_fft_rms_to_vpp(
            measurement_fft_diagnostics.rms_voltage,
            analysis[0].wave_type,
            span_vpp[0]);
        measurement_fft_diagnostics.secondary_amplitude_vpp =
            measurement_fft_rms_to_vpp(
                measurement_fft_diagnostics.secondary_rms_voltage,
                analysis[1].wave_type,
                span_vpp[1]);

        measurement_fft_diagnostics.raw_phase_deg = 0.0f;
        measurement_fft_diagnostics.phase_deg = 0.0f;
        if ((analysis[0].peak_bin != 0u)
            && (fabsf(analysis[0].peak_position - analysis[1].peak_position)
                <= MEASUREMENT_FFT_CHANNEL_MATCH_TOLERANCE_BINS))
        {
            const uint32_t reference_bin = analysis[0].peak_bin;
            const float ch2_real = measurement_fft_work[2u * reference_bin];
            const float ch2_imaginary = measurement_fft_work[2u * reference_bin + 1u];
            measurement_fft_diagnostics.raw_phase_deg =
                (atan2f(ch2_imaginary, ch2_real)
                 - atan2f(analysis[0].peak_imaginary, analysis[0].peak_real))
                * 180.0f / MEASUREMENT_FFT_PI;
            measurement_fft_diagnostics.phase_deg = measurement_fft_wrap_phase(
                measurement_fft_diagnostics.raw_phase_deg);
        }

        quality = MEASUREMENT_FFT_QUALITY_OK;
        fault_mask = 0u;
        for (channel = 0u; channel < MEASUREMENT_FFT_CHANNEL_COUNT; channel++)
        {
            mode[channel] = MEASUREMENT_MODE_UNKNOWN;
            valid_mask[channel] = 0u;
            estimated_mask[channel] = 0u;
            if (voltage_status[channel] != MEASUREMENT_FFT_VOLTAGE_INVALID)
            {
                valid_mask[channel] |= MEASUREMENT_VALID_DC_VOLTAGE;
            }
            if ((measurement_fft_diagnostics.clipping_mask & (1u << channel)) != 0u)
            {
                fault_mask |= (uint8_t)(1u << channel);
                quality = MEASUREMENT_FFT_QUALITY_CLIPPED;
            }
            else if (span_code[channel] < MEASUREMENT_FFT_MINIMUM_SPAN_CODE)
            {
                mode[channel] = MEASUREMENT_MODE_DC;
                if (channel == 0u)
                {
                    quality = MEASUREMENT_FFT_QUALITY_DC_INPUT;
                    measurement_fft_diagnostics.raw_peak_frequency_hz = 0.0f;
                    measurement_fft_diagnostics.peak_frequency_hz = 0.0f;
                    measurement_fft_diagnostics.thd_percent = 0.0f;
                    measurement_fft_diagnostics.thd_harmonic_count = 0u;
                    measurement_fft_diagnostics.wave_type =
                        MEASUREMENT_WAVE_UNKNOWN;
                    measurement_fft_spectrum.valid = 0u;
                }
                else
                {
                    measurement_fft_diagnostics.secondary_raw_peak_frequency_hz =
                        0.0f;
                    measurement_fft_diagnostics.secondary_peak_frequency_hz =
                        0.0f;
                    measurement_fft_diagnostics.secondary_thd_percent = 0.0f;
                    measurement_fft_diagnostics.secondary_thd_harmonic_count =
                        0u;
                    measurement_fft_diagnostics.secondary_wave_type =
                        MEASUREMENT_WAVE_UNKNOWN;
                }
            }
            else if ((fft_status[channel] != FFT_F32_65536_STATUS_OK)
                     || (analysis[channel].peak_power <= 0.0f)
                     || (!isfinite(analysis[channel].peak_position)))
            {
                fault_mask |= (uint8_t)(1u << channel);
                quality = MEASUREMENT_FFT_QUALITY_SIGNAL_TOO_SMALL;
            }
            else
            {
                mode[channel] = MEASUREMENT_MODE_AC;
                valid_mask[channel] |= MEASUREMENT_VALID_FREQUENCY
                                       | MEASUREMENT_VALID_THD;
                if (voltage_status[channel] != MEASUREMENT_FFT_VOLTAGE_INVALID)
                {
                    valid_mask[channel] |= MEASUREMENT_VALID_AMPLITUDE
                                           | MEASUREMENT_VALID_RMS;
                }
                if (analysis[channel].wave_type != MEASUREMENT_WAVE_UNKNOWN)
                {
                    valid_mask[channel] |= MEASUREMENT_VALID_WAVE_TYPE;
                }
                if ((channel == 0u) && (measurement_fft_spectrum.valid != 0u))
                {
                    valid_mask[channel] |= MEASUREMENT_VALID_SPECTRUM;
                }
            }
            if (voltage_status[channel] == MEASUREMENT_FFT_VOLTAGE_ESTIMATED)
            {
                estimated_mask[channel] = valid_mask[channel]
                    & (MEASUREMENT_VALID_DC_VOLTAGE
                       | MEASUREMENT_VALID_AMPLITUDE | MEASUREMENT_VALID_RMS);
            }
        }
        if ((mode[0] == MEASUREMENT_MODE_AC)
            && (mode[1] == MEASUREMENT_MODE_AC)
            && (fabsf(analysis[0].peak_position - analysis[1].peak_position)
                <= MEASUREMENT_FFT_CHANNEL_MATCH_TOLERANCE_BINS)
            && isfinite(measurement_fft_diagnostics.phase_deg))
        {
            valid_mask[0] |= MEASUREMENT_VALID_PHASE;
        }
        else if ((mode[0] == MEASUREMENT_MODE_AC)
                 && (mode[1] == MEASUREMENT_MODE_AC))
        {
            quality = MEASUREMENT_FFT_QUALITY_CHANNEL_MISMATCH;
        }
        valid = (((valid_mask[0] & (MEASUREMENT_VALID_AMPLITUDE
                                    | MEASUREMENT_VALID_FREQUENCY
                                    | MEASUREMENT_VALID_PHASE))
                  == (MEASUREMENT_VALID_AMPLITUDE
                      | MEASUREMENT_VALID_FREQUENCY | MEASUREMENT_VALID_PHASE))
                 && ((valid_mask[1] & (MEASUREMENT_VALID_AMPLITUDE
                                       | MEASUREMENT_VALID_FREQUENCY))
                     == (MEASUREMENT_VALID_AMPLITUDE
                         | MEASUREMENT_VALID_FREQUENCY)))
                    ? 1u
                    : 0u;
        measurement_fft_diagnostics.fft_count++;
        measurement_fft_diagnostics.fft_ready = 1u;
        measurement_fft_publish(valid, quality, mode[0], valid_mask[0],
                                estimated_mask[0], mode[1], valid_mask[1],
                                estimated_mask[1], fault_mask);
        measurement_fft_processing = 0u;
        measurement_fft_frame_ready = 0u;
        measurement_fft_display_start_ms = HAL_GetTick();
        measurement_fft_state = MEASUREMENT_FFT_STATE_DISPLAY;
        return;
    }

    if ((measurement_fft_state == MEASUREMENT_FFT_STATE_DISPLAY)
        && ((uint32_t)(HAL_GetTick() - measurement_fft_display_start_ms)
            >= MEASUREMENT_FFT_DISPLAY_GUARD_MS))
    {
        measurement_fft_capture_index = 0u;
        measurement_fft_settle_count = 0u;
        measurement_fft_state = MEASUREMENT_FFT_STATE_SETTLING;
    }
}

uint8_t measurement_fft_hmi_refresh_allowed(void)
{
    return (measurement_fft_state == MEASUREMENT_FFT_STATE_DISPLAY) ? 1u : 0u;
}

uint8_t measurement_fft_get_diagnostics(measurement_fft_diagnostics_t *diagnostics)
{
    if (diagnostics == 0)
    {
        return 0u;
    }
    *diagnostics = measurement_fft_diagnostics;
    return 1u;
}

uint8_t measurement_fft_get_spectrum(measurement_fft_spectrum_t *spectrum)
{
    if (spectrum == 0)
    {
        return 0u;
    }
    *spectrum = measurement_fft_spectrum;
    return 1u;
}

uint8_t measurement_fft_set_calibration(
    uint8_t channel,
    const measurement_fft_calibration_t *calibration)
{
    if ((channel >= MEASUREMENT_FFT_CHANNEL_COUNT) || (calibration == 0)
        || (calibration->valid > 1u) || (!isfinite(calibration->offset_v))
        || ((calibration->valid != 0u)
            && ((!isfinite(calibration->volts_per_code))
                || (calibration->volts_per_code == 0.0f))))
    {
        return 0u;
    }
    measurement_fft_calibration[channel] = *calibration;
    if (calibration->valid != 0u)
    {
        measurement_fft_diagnostics.voltage_calibrated_mask |=
            (uint8_t)(1u << channel);
        measurement_fft_diagnostics.voltage_estimated_mask &=
            (uint8_t)~(1u << channel);
    }
    else
    {
        measurement_fft_diagnostics.voltage_calibrated_mask &=
            (uint8_t)~(1u << channel);
        measurement_fft_diagnostics.voltage_estimated_mask |=
            (uint8_t)(1u << channel);
    }
    return 1u;
}

uint8_t measurement_fft_get_calibration(
    uint8_t channel,
    measurement_fft_calibration_t *calibration)
{
    if ((channel >= MEASUREMENT_FFT_CHANNEL_COUNT) || (calibration == 0))
    {
        return 0u;
    }
    *calibration = measurement_fft_calibration[channel];
    return 1u;
}
