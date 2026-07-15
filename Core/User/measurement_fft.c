/**
 * @file measurement_fft.c
 * @brief 片上双 ADC 同步采样的直流、幅频、失真、频谱、相位差和波形类型测量实现。
 *
 * 模块用途：以 ADC1/CH1、ADC2/CH2 的同步 16 位采样构成双缓冲 8192 点窗口，计算平均直流、
 * 峰峰值和交流有效值，去直流并加 Hann 窗后执行 8192 点 Q15 FFT，再完成亚频点插值、
 * THD、压缩频谱、双通道相位差和谐波分类。
 * GPIO 引脚映射：无直接 GPIO 引脚。
 * 依赖的外设和 CubeIDE 配置：依赖 adc_dual 在主循环提交同步样本对；
 * 使用 CMSIS Q15 数据类型，旋转因子在初始化时生成到 RAM，避免静态表超出 128 KiB Flash；
 * 当前软件目标采样率为每通道 80 kSPS，8192 点频点间隔为 9.765625 Hz。
 * 初始化方法：system_init() 调用 measurement_fft_init()。
 * 调用方法：adc_dual_process() 调用 measurement_fft_ingest_pair()，主循环随后调用
 * measurement_fft_process()；测量结果只通过 measurement_result_publish() 对外发布。
 */

#include "system.h"
#include "arm_math.h"

#define MEASUREMENT_FFT_CHANNEL_COUNT 2u
#define MEASUREMENT_FFT_BUFFER_COUNT 2u
#define MEASUREMENT_FFT_SECOND_CHANNEL 1u
#define MEASUREMENT_FFT_HANN_Q15_MAX 32767.0f
#define MEASUREMENT_FFT_PI 3.14159265358979323846f
#define MEASUREMENT_FFT_SQRT_2 1.41421356237309504880f
#define MEASUREMENT_FFT_SQRT_3 1.73205080756887729353f

/** TIM2 TRGO 驱动 ADC1/ADC2 同步转换的目标每通道采样率。 */
#define MEASUREMENT_FFT_RAW_SAMPLE_RATE_HZ 80000.0f

/** 固定使用全部同步样本，不执行未经低通滤波的跳点抽取。 */
#define MEASUREMENT_FFT_DECIMATION_FACTOR 1u

/** THD 最多统计到十次谐波，超出当前奈奎斯特频率的谐波自动忽略。 */
#define MEASUREMENT_FFT_MAX_HARMONIC_ORDER 10u

/** Hann 窗谱线能量统计时在目标频点左右覆盖的频点数。 */
#define MEASUREMENT_FFT_HARMONIC_RADIUS 2u

/** 9600 波特率轮询显示空档，覆盖一次刷新等待和最长一次阻塞发送。 */
#define MEASUREMENT_FFT_DISPLAY_GUARD_MS 500u

/** HMI 阻塞发送后，开始下一窗口前每通道主动丢弃的过渡样本数量。 */
#define MEASUREMENT_FFT_SETTLE_SAMPLE_COUNT 1024u

/** 小于该峰峰原始码跨度的输入不用于频率和相位发布。 */
#define MEASUREMENT_FFT_MINIMUM_SPAN_CODE 128u

/** 距离正负满量程小于该码值时判定存在削顶风险。 */
#define MEASUREMENT_FFT_CLIP_MARGIN_CODE 64u

/** 双通道主峰位置允许的最大差异，单位为 FFT 频点。 */
#define MEASUREMENT_FFT_CHANNEL_MATCH_TOLERANCE_BINS 1.5f

/** 初版谐波波形分类阈值，后续需用信号源实测数据校准。 */
#define MEASUREMENT_FFT_SINE_H3_MAX 0.055f
#define MEASUREMENT_FFT_SINE_H5_MAX 0.040f
#define MEASUREMENT_FFT_TRIANGLE_H3_MIN 0.055f
#define MEASUREMENT_FFT_TRIANGLE_H3_MAX 0.200f
#define MEASUREMENT_FFT_TRIANGLE_H5_MAX 0.080f
#define MEASUREMENT_FFT_SQUARE_H3_MIN 0.200f
#define MEASUREMENT_FFT_SQUARE_H5_MIN 0.080f

typedef enum
{
    MEASUREMENT_FFT_STATE_CAPTURE = 0,
    MEASUREMENT_FFT_STATE_READY,
    MEASUREMENT_FFT_STATE_DISPLAY,
    MEASUREMENT_FFT_STATE_SETTLING
} measurement_fft_state_t;

/** 单通道一帧原始码的最小值与最大值。 */
typedef struct
{
    uint16_t minimum_code;
    uint16_t maximum_code;
} measurement_fft_raw_span_t;

/** 单通道未加窗时域统计量，单位仍为 ADC 原始码。 */
typedef struct
{
    measurement_fft_raw_span_t span;
    float mean_raw_code;
    float rms_ac_code;
} measurement_fft_time_metrics_t;

/** 双缓冲原始采样窗口：第一维为缓冲区，第二维为 AIN0/AIN1。 */
static q15_t measurement_fft_input[MEASUREMENT_FFT_BUFFER_COUNT]
                                    [MEASUREMENT_FFT_CHANNEL_COUNT]
                                    [MEASUREMENT_FFT_LENGTH]
    __attribute__((aligned(32)));

/**
 * AIN0 与 AIN1 的复数 FFT 输出缓冲区。
 * 每个频点按实部、虚部交错保存，因此元素数量为输入长度的两倍。
 */
static q15_t measurement_fft_output[MEASUREMENT_FFT_CHANNEL_COUNT]
                                     [MEASUREMENT_FFT_OUTPUT_LENGTH]
    __attribute__((aligned(32)));

/** Q15 Hann 窗系数，初始化阶段计算一次。 */
static q15_t measurement_fft_hann_window[MEASUREMENT_FFT_LENGTH]
    __attribute__((aligned(32)));

/** 8192 点基 2 FFT 的 Q15 复数旋转因子，在启动时生成到 RAM。 */
static q15_t measurement_fft_twiddle[MEASUREMENT_FFT_LENGTH]
    __attribute__((aligned(32)));

/** 当前正在填充的双缓冲区编号。 */
static uint8_t measurement_fft_active_buffer;

/** 已收齐、等待 FFT 处理的双缓冲区编号。 */
static uint8_t measurement_fft_ready_buffer;

/** AIN0 与 AIN1 在当前窗口内已经写入的样本数量。 */
static uint16_t measurement_fft_sample_count[MEASUREMENT_FFT_CHANNEL_COUNT];

/** 显示结束后两个通道已经丢弃的过渡样本数量。 */
static uint16_t measurement_fft_settle_count[MEASUREMENT_FFT_CHANNEL_COUNT];

/** 固定抽取因子；保留在诊断接口中用于确认当前采样策略。 */
static uint8_t measurement_fft_decimation_factor;

/** 当前采集、处理、显示或过渡状态。 */
static measurement_fft_state_t measurement_fft_state;

/** 显示空档开始时间，用于在固定时间后恢复采集。 */
static uint32_t measurement_fft_display_start_ms;

/** 发布结果序号，每完成一帧 FFT 增加一次。 */
static uint32_t measurement_fft_result_sequence;

/** FFT 运行状态和最近一次完整测量诊断快照，供调试器直接观察。 */
measurement_fft_diagnostics_t measurement_fft_diagnostics;

/** 最近一次面向串口屏压缩得到的 64 点频谱。 */
static measurement_fft_spectrum_t measurement_fft_spectrum;

/** AIN0/AIN1 的运行时软件校准系数，由主循环配置和读取。 */
static measurement_fft_calibration_t
    measurement_fft_calibration[MEASUREMENT_FFT_CHANNEL_COUNT];

/** 迁移期间旧 ADS8688 顺序接口暂存的一对通道样本。 */
static uint16_t measurement_fft_legacy_pair[MEASUREMENT_FFT_CHANNEL_COUNT];
static uint8_t measurement_fft_legacy_pair_mask;

/**
 * @brief 使能 Cortex-M7 DWT 周期计数器。
 * @param 无。
 * @return 无。
 * @note 只用于记录双通道 FFT 耗时，不修改时钟树或外设配置。
 */
static void measurement_fft_enable_cycle_counter(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief 将有符号整数限制到 Q15 表示范围。
 * @param value 待限制的整数。
 * @return 限制后的 Q15 数值。
 * @note 无副作用。
 */
static q15_t measurement_fft_clamp_q15(int32_t value)
{
    if (value > 32767)
    {
        return 32767;
    }
    if (value < -32768)
    {
        return -32768;
    }
    return (q15_t)value;
}

/**
 * @brief 生成 8192 点基 2 FFT 所需的前向 Q15 旋转因子。
 * @param 无。
 * @return 无。
 * @note 启动时占用短暂计算时间；表存放在 RAM，不占用额外 Flash 常量表。
 */
static void measurement_fft_initialize_twiddle(void)
{
    uint32_t index;

    for (index = 0u; index < (MEASUREMENT_FFT_LENGTH / 2u); index++)
    {
        float angle = -2.0f * MEASUREMENT_FFT_PI * (float)index
                      / (float)MEASUREMENT_FFT_LENGTH;
        float real_value = cosf(angle) * MEASUREMENT_FFT_HANN_Q15_MAX;
        float imaginary_value = sinf(angle) * MEASUREMENT_FFT_HANN_Q15_MAX;
        int32_t real_q15 = (int32_t)(real_value >= 0.0f
                                         ? real_value + 0.5f
                                         : real_value - 0.5f);
        int32_t imaginary_q15 = (int32_t)(imaginary_value >= 0.0f
                                              ? imaginary_value + 0.5f
                                              : imaginary_value - 0.5f);

        measurement_fft_twiddle[index * 2u] =
            measurement_fft_clamp_q15(real_q15);
        measurement_fft_twiddle[index * 2u + 1u] =
            measurement_fft_clamp_q15(imaginary_q15);
    }
}

/**
 * @brief 反转固定 8192 点复数序列的索引位顺序。
 * @param value 原始索引。
 * @return 13 位反转后的索引。
 * @note 无副作用。
 */
static uint16_t measurement_fft_reverse_index(uint16_t value)
{
    uint16_t reversed = 0u;
    uint8_t bit;

    for (bit = 0u; bit < 13u; bit++)
    {
        reversed = (uint16_t)((reversed << 1u) | (value & 1u));
        value >>= 1u;
    }

    return reversed;
}

/**
 * @brief 对一通道实数 Q15 窗口执行缩放基 2 前向 FFT。
 * @param input 已去直流并加 Hann 窗的 8192 点实数输入。
 * @param output 接收 8192 个复数频点的交错 Q15 数组。
 * @return 无。
 * @note 每级蝶形缩小 1 位防止溢出，总缩放与 8192 点 Q15 FFT 一致。
 */
static void measurement_fft_execute_q15(const q15_t *input, q15_t *output)
{
    uint32_t index;
    uint32_t stage_size;

    for (index = 0u; index < MEASUREMENT_FFT_LENGTH; index++)
    {
        uint32_t reversed = measurement_fft_reverse_index((uint16_t)index);

        output[reversed * 2u] = input[index];
        output[reversed * 2u + 1u] = 0;
    }

    for (stage_size = 2u;
         stage_size <= MEASUREMENT_FFT_LENGTH;
         stage_size <<= 1u)
    {
        uint32_t half_size = stage_size / 2u;
        uint32_t twiddle_step = MEASUREMENT_FFT_LENGTH / stage_size;
        uint32_t block;

        for (block = 0u; block < MEASUREMENT_FFT_LENGTH; block += stage_size)
        {
            uint32_t offset;

            for (offset = 0u; offset < half_size; offset++)
            {
                uint32_t even_index = (block + offset) * 2u;
                uint32_t odd_index = (block + offset + half_size) * 2u;
                uint32_t twiddle_index = offset * twiddle_step * 2u;
                int32_t even_real = output[even_index];
                int32_t even_imaginary = output[even_index + 1u];
                int32_t odd_real = output[odd_index];
                int32_t odd_imaginary = output[odd_index + 1u];
                int32_t twiddle_real = measurement_fft_twiddle[twiddle_index];
                int32_t twiddle_imaginary =
                    measurement_fft_twiddle[twiddle_index + 1u];
                int32_t product_real = (int32_t)(
                    ((int64_t)twiddle_real * odd_real
                     - (int64_t)twiddle_imaginary * odd_imaginary)
                    >> 15);
                int32_t product_imaginary = (int32_t)(
                    ((int64_t)twiddle_real * odd_imaginary
                     + (int64_t)twiddle_imaginary * odd_real)
                    >> 15);

                output[even_index] = measurement_fft_clamp_q15(
                    (even_real + product_real) / 2);
                output[even_index + 1u] = measurement_fft_clamp_q15(
                    (even_imaginary + product_imaginary) / 2);
                output[odd_index] = measurement_fft_clamp_q15(
                    (even_real - product_real) / 2);
                output[odd_index + 1u] = measurement_fft_clamp_q15(
                    (even_imaginary - product_imaginary) / 2);
            }
        }
    }
}

/**
 * @brief 统计未加窗时域样本的范围、平均值和去直流有效值。
 * @param samples 一帧以 0x8000 为中心转换后的 Q15 原始样本。
 * @return 该帧的原始码时域统计量。
 * @note 必须在 measurement_fft_prepare_window() 修改输入数组前调用。
 */
static measurement_fft_time_metrics_t measurement_fft_analyze_time_domain(
    const q15_t *samples)
{
    int64_t sum = 0;
    int64_t centered_square_sum = 0;
    int32_t mean;
    int32_t minimum = 32767;
    int32_t maximum = -32768;
    uint32_t index;
    measurement_fft_time_metrics_t metrics;

    for (index = 0u; index < MEASUREMENT_FFT_LENGTH; index++)
    {
        int32_t sample = samples[index];

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

    mean = (int32_t)(sum / (int64_t)MEASUREMENT_FFT_LENGTH);
    for (index = 0u; index < MEASUREMENT_FFT_LENGTH; index++)
    {
        int32_t centered = (int32_t)samples[index] - mean;
        centered_square_sum += (int64_t)centered * centered;
    }

    metrics.span.minimum_code = (uint16_t)(minimum + 32768);
    metrics.span.maximum_code = (uint16_t)(maximum + 32768);
    metrics.mean_raw_code =
        (float)sum / (float)MEASUREMENT_FFT_LENGTH + 32768.0f;
    metrics.rms_ac_code = sqrtf(
        (float)centered_square_sum / (float)MEASUREMENT_FFT_LENGTH);
    return metrics;
}

/**
 * @brief 将一帧原始码统计量换算为电压统计量。
 * @param metrics 原始码范围、平均值和交流有效值。
 * @param channel 本地同步通道号，只允许 0 或 1。
 * @param amplitude_vpp 用于接收峰峰值电压的指针。
 * @param dc_voltage 用于接收平均直流电压的指针。
 * @param rms_voltage 用于接收去直流交流有效值的指针。
 * @return 校准有效且换算成功返回 1，否则返回 0。
 * @note 前级比例未确认时输出 NAN，FFT 频率和相位分析仍可继续。
 */
static uint8_t measurement_fft_convert_time_metrics(
    measurement_fft_time_metrics_t metrics,
    uint8_t channel,
    float *amplitude_vpp,
    float *dc_voltage,
    float *rms_voltage)
{
    const measurement_fft_calibration_t *calibration;
    float volts_per_code;

    if ((channel >= MEASUREMENT_FFT_CHANNEL_COUNT)
        || (amplitude_vpp == 0)
        || (dc_voltage == 0)
        || (rms_voltage == 0))
    {
        return 0u;
    }

    calibration = &measurement_fft_calibration[channel];
    if ((calibration->valid == 0u)
        || (!isfinite(calibration->volts_per_code))
        || (calibration->volts_per_code == 0.0f)
        || (!isfinite(calibration->offset_v)))
    {
        *amplitude_vpp = NAN;
        *dc_voltage = NAN;
        *rms_voltage = NAN;
        return 0u;
    }

    volts_per_code = calibration->volts_per_code;
    *amplitude_vpp =
        (float)(metrics.span.maximum_code - metrics.span.minimum_code)
        * fabsf(volts_per_code);
    *dc_voltage = metrics.mean_raw_code * volts_per_code
                  + calibration->offset_v;
    *rms_voltage = metrics.rms_ac_code * fabsf(volts_per_code);
    return 1u;
}

/**
 * @brief 判断一帧是否接近 ADC 正负满量程。
 * @param span 原始码最小值和最大值。
 * @return 接近任一满量程端点时返回 1，否则返回 0。
 * @note 该判断用于避免把削顶波形作为有效测量结果发布。
 */
static uint8_t measurement_fft_span_is_clipped(
    measurement_fft_raw_span_t span)
{
    if ((span.minimum_code <= MEASUREMENT_FFT_CLIP_MARGIN_CODE)
        || (span.maximum_code
            >= (uint16_t)(65535u - MEASUREMENT_FFT_CLIP_MARGIN_CODE)))
    {
        return 1u;
    }

    return 0u;
}

/**
 * @brief 对单通道窗口去直流并施加 Hann 窗。
 * @param samples 待处理的 Q15 样本数组。
 * @return 无。
 * @note 原地修改输入窗口，处理后数组仅供 FFT 使用。
 */
static void measurement_fft_prepare_window(q15_t *samples)
{
    int64_t sum = 0;
    int32_t mean;
    uint32_t index;

    for (index = 0u; index < MEASUREMENT_FFT_LENGTH; index++)
    {
        sum += samples[index];
    }
    mean = (int32_t)(sum / (int64_t)MEASUREMENT_FFT_LENGTH);

    for (index = 0u; index < MEASUREMENT_FFT_LENGTH; index++)
    {
        int32_t centered = (int32_t)samples[index] - mean;
        int32_t windowed =
            (centered * (int32_t)measurement_fft_hann_window[index]) >> 15;
        samples[index] = measurement_fft_clamp_q15(windowed);
    }
}

/**
 * @brief 计算一个复数频点的平方幅值。
 * @param spectrum Q15 复数 FFT 的交错实部、虚部输出数组。
 * @param bin 待计算的正频率频点。
 * @return 实部平方与虚部平方之和。
 * @note 使用 64 位整数避免两个 Q15 平方相加时溢出。
 */
static int64_t measurement_fft_bin_power(const q15_t *spectrum,
                                         uint16_t bin)
{
    int32_t real = spectrum[(uint32_t)bin * 2u];
    int32_t imaginary = spectrum[(uint32_t)bin * 2u + 1u];

    return (int64_t)real * real + (int64_t)imaginary * imaginary;
}

/**
 * @brief 查找最大非直流正频率峰值。
 * @param spectrum Q15 复数 FFT 的交错实部、虚部输出数组。
 * @param peak_power 用于接收主峰平方幅值的指针。
 * @return 最大峰值对应的整数频点；没有能量时返回 0。
 * @note 跳过直流和奈奎斯特端点，为三点插值保留左右相邻频点。
 */
static uint16_t measurement_fft_find_peak_bin(const q15_t *spectrum,
                                              int64_t *peak_power)
{
    int64_t largest_power = 0;
    uint16_t peak_bin = 0u;
    uint16_t bin;

    for (bin = 1u; bin < (MEASUREMENT_FFT_LENGTH / 2u - 1u); bin++)
    {
        int64_t power = measurement_fft_bin_power(spectrum, bin);

        if (power > largest_power)
        {
            largest_power = power;
            peak_bin = bin;
        }
    }

    if (peak_power != 0)
    {
        *peak_power = largest_power;
    }
    return peak_bin;
}

/**
 * @brief 用主峰左右三个对数平方幅值进行抛物线亚频点插值。
 * @param spectrum Q15 复数 FFT 的交错实部、虚部输出数组。
 * @param peak_bin 主峰整数频点。
 * @return 限制在 -0.5 至 0.5 之间的亚频点偏移。
 * @note 该插值提高非整频点信号的频率读数精度，但不改变 8192 点本征频点间隔。
 */
static float measurement_fft_interpolate_peak(const q15_t *spectrum,
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

    left = (float)measurement_fft_bin_power(spectrum, peak_bin - 1u);
    center = (float)measurement_fft_bin_power(spectrum, peak_bin);
    right = (float)measurement_fft_bin_power(spectrum, peak_bin + 1u);
    if ((left <= 0.0f) || (center <= 0.0f) || (right <= 0.0f))
    {
        return 0.0f;
    }

    left = logf(left);
    center = logf(center);
    right = logf(right);
    denominator = left - 2.0f * center + right;
    if (fabsf(denominator) < 1.0f)
    {
        return 0.0f;
    }

    offset = 0.5f * (left - right) / denominator;
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
 * @brief 计算目标谐波附近的最大幅值与基波幅值之比。
 * @param spectrum Q15 复数 FFT 的交错实部、虚部输出数组。
 * @param fundamental_bin 插值后的基波频点位置。
 * @param harmonic_order 谐波次数。
 * @param fundamental_power 基波主峰平方幅值。
 * @return 谐波幅值比；目标谐波超出奈奎斯特范围时返回 -1。
 * @note 在目标频点左右各搜索一个频点，降低非整频点泄漏对分类的影响。
 */
static float measurement_fft_harmonic_ratio(const q15_t *spectrum,
                                             float fundamental_bin,
                                             uint8_t harmonic_order,
                                             int64_t fundamental_power)
{
    float target_bin = fundamental_bin * (float)harmonic_order;
    int64_t harmonic_power = 0;
    uint16_t center_bin;
    uint16_t bin;

    if ((fundamental_power <= 0)
        || (target_bin < 2.0f)
        || (target_bin >= (float)(MEASUREMENT_FFT_LENGTH / 2u - 2u)))
    {
        return -1.0f;
    }

    center_bin = (uint16_t)(target_bin + 0.5f);
    for (bin = center_bin - 1u; bin <= center_bin + 1u; bin++)
    {
        int64_t power = measurement_fft_bin_power(spectrum, bin);

        if (power > harmonic_power)
        {
            harmonic_power = power;
        }
    }

    return sqrtf((float)harmonic_power / (float)fundamental_power);
}

/**
 * @brief 汇总目标频点附近 Hann 主瓣的平方幅值。
 * @param spectrum Q15 复数 FFT 的交错实部、虚部输出数组。
 * @param center_bin 目标中心频点。
 * @param radius 左右覆盖频点数量。
 * @return 指定频带内平方幅值之和；越界时返回零。
 */
static int64_t measurement_fft_band_power(const q15_t *spectrum,
                                          uint16_t center_bin,
                                          uint8_t radius)
{
    int64_t total_power = 0;
    uint16_t start_bin;
    uint16_t end_bin;
    uint16_t bin;

    if ((center_bin <= radius)
        || ((uint32_t)center_bin + radius
            >= (MEASUREMENT_FFT_LENGTH / 2u)))
    {
        return 0;
    }

    start_bin = center_bin - radius;
    end_bin = center_bin + radius;
    for (bin = start_bin; bin <= end_bin; bin++)
    {
        total_power += measurement_fft_bin_power(spectrum, bin);
    }
    return total_power;
}

/**
 * @brief 计算当前可观测谐波范围内的总谐波失真。
 * @param spectrum AIN0 的 Q15 复数 FFT 输出。
 * @param fundamental_bin 插值后的基波频点位置。
 * @param harmonic_count 用于接收实际纳入计算的谐波数量。
 * @return THD 百分比；无足够频带或基波能量时返回零。
 * @note 自动忽略超过奈奎斯特频率和与基波 Hann 主瓣重叠的谐波。
 */
static float measurement_fft_calculate_thd(const q15_t *spectrum,
                                           float fundamental_bin,
                                           uint8_t *harmonic_count)
{
    int64_t fundamental_power;
    int64_t harmonic_power_sum = 0;
    uint16_t fundamental_center;
    uint8_t count = 0u;
    uint8_t order;

    if (harmonic_count == 0)
    {
        return 0.0f;
    }
    *harmonic_count = 0u;
    if (fundamental_bin < 1.0f)
    {
        return 0.0f;
    }

    fundamental_center = (uint16_t)(fundamental_bin + 0.5f);
    fundamental_power = measurement_fft_band_power(
        spectrum,
        fundamental_center,
        MEASUREMENT_FFT_HARMONIC_RADIUS);
    if (fundamental_power <= 0)
    {
        return 0.0f;
    }

    for (order = 2u; order <= MEASUREMENT_FFT_MAX_HARMONIC_ORDER; order++)
    {
        float target_bin = fundamental_bin * (float)order;
        uint16_t center_bin;
        int64_t harmonic_power;

        if (target_bin
            >= (float)(MEASUREMENT_FFT_LENGTH / 2u
                       - MEASUREMENT_FFT_HARMONIC_RADIUS))
        {
            break;
        }
        center_bin = (uint16_t)(target_bin + 0.5f);
        if (center_bin
            <= (uint16_t)(fundamental_center
                          + 2u * MEASUREMENT_FFT_HARMONIC_RADIUS))
        {
            continue;
        }

        harmonic_power = measurement_fft_band_power(
            spectrum,
            center_bin,
            MEASUREMENT_FFT_HARMONIC_RADIUS);
        if (harmonic_power > 0)
        {
            harmonic_power_sum += harmonic_power;
            count++;
        }
    }

    *harmonic_count = count;
    if (count == 0u)
    {
        return 0.0f;
    }
    return sqrtf((float)harmonic_power_sum / (float)fundamental_power)
           * 100.0f;
}

/**
 * @brief 根据波形类型用交流有效值计算抗噪声峰峰值。
 * @param rms_voltage 去直流后的交流有效值。
 * @param wave_type 已识别波形。
 * @param fallback_vpp 无法识别波形时使用的时域跨度。
 * @return 对应波形的峰峰值。
 */
static float measurement_fft_rms_to_vpp(float rms_voltage,
                                        measurement_wave_type_t wave_type,
                                        float fallback_vpp)
{
    switch (wave_type)
    {
        case MEASUREMENT_WAVE_SINE:
            return 2.0f * MEASUREMENT_FFT_SQRT_2 * rms_voltage;
        case MEASUREMENT_WAVE_SQUARE:
            return 2.0f * rms_voltage;
        case MEASUREMENT_WAVE_TRIANGLE:
            return 2.0f * MEASUREMENT_FFT_SQRT_3 * rms_voltage;
        default:
            return fallback_vpp;
    }
}

/**
 * @brief 更新固定采样率和频点间隔诊断。
 * @param 无。
 * @return 无。
 */
static void measurement_fft_update_timing_diagnostics(void)
{
    measurement_fft_diagnostics.raw_sample_rate_hz =
        MEASUREMENT_FFT_RAW_SAMPLE_RATE_HZ;
    measurement_fft_diagnostics.decimation_factor =
        measurement_fft_decimation_factor;
    measurement_fft_diagnostics.effective_sample_rate_hz =
        MEASUREMENT_FFT_RAW_SAMPLE_RATE_HZ
        / (float)measurement_fft_decimation_factor;
    measurement_fft_diagnostics.bin_width_hz =
        measurement_fft_diagnostics.effective_sample_rate_hz
        / (float)MEASUREMENT_FFT_LENGTH;
}

/**
 * @brief 将 AIN0 FFT 压缩为固定 64 点相对幅度频谱。
 * @param spectrum AIN0 的 Q15 复数 FFT 输出。
 * @param reference_power 本帧主峰平方幅值。
 * @return 无。
 * @note 输出固定覆盖 0 至 20 kHz，超过当前奈奎斯特频率的点写入 -80 dB。
 */
static void measurement_fft_build_spectrum(const q15_t *spectrum,
                                           int64_t reference_power)
{
    const float point_width =
        MEASUREMENT_FFT_SPECTRUM_MAX_HZ
        / (float)MEASUREMENT_FFT_SPECTRUM_POINT_COUNT;
    const float bin_width = measurement_fft_diagnostics.bin_width_hz;
    const float nyquist =
        measurement_fft_diagnostics.effective_sample_rate_hz * 0.5f;
    uint32_t point;

    measurement_fft_spectrum.point_width_hz = point_width;
    measurement_fft_spectrum.nyquist_hz = nyquist;
    measurement_fft_spectrum.sequence++;
    measurement_fft_spectrum.valid = (reference_power > 0) ? 1u : 0u;

    for (point = 0u; point < MEASUREMENT_FFT_SPECTRUM_POINT_COUNT; point++)
    {
        float start_hz = (float)point * point_width;
        float end_hz = start_hz + point_width;
        int64_t largest_power = 0;
        uint16_t start_bin;
        uint16_t end_bin;
        uint16_t bin;

        if ((reference_power <= 0) || (start_hz >= nyquist)
            || (bin_width <= 0.0f))
        {
            measurement_fft_spectrum.relative_db_x10[point] =
                MEASUREMENT_FFT_SPECTRUM_FLOOR_DB_X10;
            continue;
        }

        start_bin = (uint16_t)(start_hz / bin_width);
        end_bin = (uint16_t)(end_hz / bin_width);
        if (start_bin < 1u)
        {
            start_bin = 1u;
        }
        if (end_bin >= (MEASUREMENT_FFT_LENGTH / 2u))
        {
            end_bin = MEASUREMENT_FFT_LENGTH / 2u - 1u;
        }

        for (bin = start_bin; bin <= end_bin; bin++)
        {
            int64_t power = measurement_fft_bin_power(spectrum, bin);
            if (power > largest_power)
            {
                largest_power = power;
            }
        }

        if (largest_power <= 0)
        {
            measurement_fft_spectrum.relative_db_x10[point] =
                MEASUREMENT_FFT_SPECTRUM_FLOOR_DB_X10;
        }
        else
        {
            float relative_db =
                10.0f * log10f((float)largest_power / (float)reference_power);
            if (relative_db > 0.0f)
            {
                relative_db = 0.0f;
            }
            else if (relative_db < -80.0f)
            {
                relative_db = -80.0f;
            }
            measurement_fft_spectrum.relative_db_x10[point] =
                (int16_t)(relative_db * 10.0f);
        }
    }
}

/**
 * @brief 根据三次和五次谐波幅值比进行初步波形分类。
 * @param harmonic_ratio_3 三次谐波与基波幅值比。
 * @param harmonic_ratio_5 五次谐波与基波幅值比。
 * @return 正弦波、方波、三角波或未知波形枚举。
 * @note 阈值为离线初值，必须在实板上用标准信号源覆盖不同幅度和频率后再校准。
 */
static measurement_wave_type_t measurement_fft_classify_wave(
    float harmonic_ratio_3,
    float harmonic_ratio_5)
{
    if ((harmonic_ratio_3 < 0.0f) || (harmonic_ratio_5 < 0.0f))
    {
        return MEASUREMENT_WAVE_UNKNOWN;
    }

    if ((harmonic_ratio_3 >= MEASUREMENT_FFT_SQUARE_H3_MIN)
        && (harmonic_ratio_5 >= MEASUREMENT_FFT_SQUARE_H5_MIN))
    {
        return MEASUREMENT_WAVE_SQUARE;
    }

    if ((harmonic_ratio_3 >= MEASUREMENT_FFT_TRIANGLE_H3_MIN)
        && (harmonic_ratio_3 < MEASUREMENT_FFT_TRIANGLE_H3_MAX)
        && (harmonic_ratio_5 <= MEASUREMENT_FFT_TRIANGLE_H5_MAX))
    {
        return MEASUREMENT_WAVE_TRIANGLE;
    }

    if ((harmonic_ratio_3 < MEASUREMENT_FFT_SINE_H3_MAX)
        && (harmonic_ratio_5 < MEASUREMENT_FFT_SINE_H5_MAX))
    {
        return MEASUREMENT_WAVE_SINE;
    }

    return MEASUREMENT_WAVE_UNKNOWN;
}

/**
 * @brief 将角度折返到 -180 至 180 度区间。
 * @param phase_deg 待折返角度。
 * @return 折返后的角度。
 * @note 无副作用。
 */
static float measurement_fft_wrap_phase(float phase_deg)
{
    while (phase_deg > 180.0f)
    {
        phase_deg -= 360.0f;
    }
    while (phase_deg <= -180.0f)
    {
        phase_deg += 360.0f;
    }
    return phase_deg;
}

/**
 * @brief 计算指定频点处 AIN1 相对 AIN0 的相位差。
 * @param first_spectrum AIN0 复数 FFT 输出。
 * @param second_spectrum AIN1 复数 FFT 输出。
 * @param bin 用于相位计算的基波整数频点。
 * @return AIN1-AIN0 相位，单位为度，范围为 -180 至 180 度。
 * @note 使用 X1 乘以 X0 共轭的交叉频谱，不受两个通道公共窗相位影响。
 */
static float measurement_fft_calculate_phase(const q15_t *first_spectrum,
                                             const q15_t *second_spectrum,
                                             uint16_t bin)
{
    int32_t first_real = first_spectrum[(uint32_t)bin * 2u];
    int32_t first_imaginary = first_spectrum[(uint32_t)bin * 2u + 1u];
    int32_t second_real = second_spectrum[(uint32_t)bin * 2u];
    int32_t second_imaginary = second_spectrum[(uint32_t)bin * 2u + 1u];
    int64_t cross_real =
        (int64_t)second_real * first_real
        + (int64_t)second_imaginary * first_imaginary;
    int64_t cross_imaginary =
        (int64_t)second_imaginary * first_real
        - (int64_t)second_real * first_imaginary;

    return measurement_fft_wrap_phase(
        atan2f((float)cross_imaginary, (float)cross_real)
        * 180.0f / MEASUREMENT_FFT_PI);
}

/**
 * @brief 发布当前帧的测量结果并同步诊断状态。
 * @param valid 非零表示结果可作为 LIVE 数据显示。
 * @param quality 当前帧质量状态。
 * @param mode 当前输入的直流或交流模式。
 * @param valid_mask 本帧各测量字段有效位。
 * @return 无。
 * @note 无效结果也会发布并递增序号，使 HMI 回到 WAIT 而不是保留过期有效值。
 */
static void measurement_fft_publish(uint8_t valid,
                                    measurement_fft_quality_t quality,
                                    measurement_mode_t mode,
                                    uint16_t valid_mask)
{
    measurement_result_t result;

    measurement_fft_result_sequence++;
    result.dc_voltage = measurement_fft_diagnostics.dc_voltage;
    result.amplitude_vpp = measurement_fft_diagnostics.amplitude_vpp;
    result.rms_voltage = measurement_fft_diagnostics.rms_voltage;
    result.frequency_hz = measurement_fft_diagnostics.peak_frequency_hz;
    result.thd_percent = measurement_fft_diagnostics.thd_percent;
    result.phase_deg = measurement_fft_diagnostics.phase_deg;
    result.wave_type = measurement_fft_diagnostics.wave_type;
    result.mode = mode;
    result.valid_mask = valid_mask;
    result.valid = valid;
    result.sequence = measurement_fft_result_sequence;
    measurement_result_publish(&result);

    measurement_fft_diagnostics.publish_count++;
    measurement_fft_diagnostics.quality = quality;
    measurement_fft_diagnostics.result_valid = valid;
}

/**
 * @brief 初始化 8192 点 Q15 FFT、Hann 窗和测量状态。
 * @param 无。
 * @return 无。
 * @note 不启动 ADC；采样从 adc_dual 主循环按同步样本对输入开始。
 */
void measurement_fft_init(void)
{
    uint32_t index;

    measurement_fft_active_buffer = 0u;
    measurement_fft_ready_buffer = 0u;
    measurement_fft_sample_count[0] = 0u;
    measurement_fft_sample_count[1] = 0u;
    measurement_fft_settle_count[0] = 0u;
    measurement_fft_settle_count[1] = 0u;
    measurement_fft_decimation_factor = MEASUREMENT_FFT_DECIMATION_FACTOR;
    measurement_fft_state = MEASUREMENT_FFT_STATE_CAPTURE;
    measurement_fft_display_start_ms = 0u;
    measurement_fft_result_sequence = 0u;
    measurement_fft_legacy_pair[0] = 0u;
    measurement_fft_legacy_pair[1] = 0u;
    measurement_fft_legacy_pair_mask = 0u;

    measurement_fft_initialize_twiddle();
    measurement_fft_diagnostics.init_status = (int32_t)ARM_MATH_SUCCESS;
    measurement_fft_diagnostics.window_count = 0u;
    measurement_fft_diagnostics.discarded_sample_count = 0u;
    measurement_fft_diagnostics.fft_count = 0u;
    measurement_fft_diagnostics.publish_count = 0u;
    measurement_fft_diagnostics.last_fft_cycles = 0u;
    measurement_fft_update_timing_diagnostics();
    measurement_fft_diagnostics.peak_bin = 0u;
    measurement_fft_diagnostics.secondary_peak_bin = 0u;
    measurement_fft_diagnostics.peak_offset_bins = 0.0f;
    measurement_fft_diagnostics.peak_frequency_hz = 0.0f;
    measurement_fft_diagnostics.amplitude_vpp = 0.0f;
    measurement_fft_diagnostics.secondary_amplitude_vpp = 0.0f;
    measurement_fft_diagnostics.dc_voltage = 0.0f;
    measurement_fft_diagnostics.secondary_dc_voltage = 0.0f;
    measurement_fft_diagnostics.rms_voltage = 0.0f;
    measurement_fft_diagnostics.secondary_rms_voltage = 0.0f;
    measurement_fft_diagnostics.thd_percent = 0.0f;
    measurement_fft_diagnostics.thd_harmonic_count = 0u;
    measurement_fft_diagnostics.raw_phase_deg = 0.0f;
    measurement_fft_diagnostics.phase_deg = 0.0f;
    measurement_fft_diagnostics.harmonic_ratio_3 = 0.0f;
    measurement_fft_diagnostics.harmonic_ratio_5 = 0.0f;
    measurement_fft_diagnostics.wave_type = MEASUREMENT_WAVE_UNKNOWN;
    measurement_fft_diagnostics.quality =
        (measurement_fft_diagnostics.init_status == (int32_t)ARM_MATH_SUCCESS)
            ? MEASUREMENT_FFT_QUALITY_NOT_READY
            : MEASUREMENT_FFT_QUALITY_INIT_ERROR;
    measurement_fft_diagnostics.clipping_mask = 0u;
    measurement_fft_diagnostics.result_valid = 0u;
    measurement_fft_diagnostics.fft_ready = 0u;
    measurement_fft_diagnostics.voltage_calibrated_mask = 0u;

    measurement_fft_spectrum.point_width_hz =
        MEASUREMENT_FFT_SPECTRUM_MAX_HZ
        / (float)MEASUREMENT_FFT_SPECTRUM_POINT_COUNT;
    measurement_fft_spectrum.nyquist_hz =
        measurement_fft_diagnostics.effective_sample_rate_hz * 0.5f;
    measurement_fft_spectrum.sequence = 0u;
    measurement_fft_spectrum.valid = 0u;
    measurement_fft_calibration[0].volts_per_code = 0.0f;
    measurement_fft_calibration[0].offset_v = 0.0f;
    measurement_fft_calibration[0].valid = 0u;
    measurement_fft_calibration[1].volts_per_code = 0.0f;
    measurement_fft_calibration[1].offset_v = 0.0f;
    measurement_fft_calibration[1].valid = 0u;
    for (index = 0u; index < MEASUREMENT_FFT_SPECTRUM_POINT_COUNT; index++)
    {
        measurement_fft_spectrum.relative_db_x10[index] =
            MEASUREMENT_FFT_SPECTRUM_FLOOR_DB_X10;
    }

    for (index = 0u; index < MEASUREMENT_FFT_LENGTH; index++)
    {
        float phase =
            (2.0f * MEASUREMENT_FFT_PI * (float)index) /
            (float)(MEASUREMENT_FFT_LENGTH - 1u);
        float coefficient = 0.5f * (1.0f - cosf(phase));
        measurement_fft_hann_window[index] =
            (q15_t)(coefficient * MEASUREMENT_FFT_HANN_Q15_MAX + 0.5f);
    }

    measurement_fft_enable_cycle_counter();
}

/**
 * @brief 接收同一触发时刻的双通道原始采样对。
 * @param ch1_raw_code ADC1/CH1 的 16 位原始码。
 * @param ch2_raw_code ADC2/CH2 的 16 位原始码。
 * @return 无。
 * @note 两个通道共用抽取计数和写入索引，从软件边界保证窗口严格对齐。
 */
void measurement_fft_ingest_pair(uint16_t ch1_raw_code,
                                 uint16_t ch2_raw_code)
{
    uint16_t write_index;

    if (measurement_fft_diagnostics.init_status != (int32_t)ARM_MATH_SUCCESS)
    {
        return;
    }

    if (measurement_fft_state == MEASUREMENT_FFT_STATE_SETTLING)
    {
        measurement_fft_diagnostics.discarded_sample_count += 2u;
        if (measurement_fft_settle_count[0]
            < MEASUREMENT_FFT_SETTLE_SAMPLE_COUNT)
        {
            measurement_fft_settle_count[0]++;
            measurement_fft_settle_count[1]++;
        }

        if (measurement_fft_settle_count[0]
            == MEASUREMENT_FFT_SETTLE_SAMPLE_COUNT)
        {
            measurement_fft_sample_count[0] = 0u;
            measurement_fft_sample_count[1] = 0u;
            measurement_fft_update_timing_diagnostics();
            measurement_fft_state = MEASUREMENT_FFT_STATE_CAPTURE;
        }
        return;
    }

    if (measurement_fft_state != MEASUREMENT_FFT_STATE_CAPTURE)
    {
        measurement_fft_diagnostics.discarded_sample_count += 2u;
        return;
    }

    write_index = measurement_fft_sample_count[0];
    if ((write_index >= MEASUREMENT_FFT_LENGTH)
        || (measurement_fft_sample_count[1] != write_index))
    {
        measurement_fft_diagnostics.discarded_sample_count += 2u;
        return;
    }

    measurement_fft_input[measurement_fft_active_buffer][0][write_index] =
        (q15_t)((int32_t)ch1_raw_code - 32768);
    measurement_fft_input[measurement_fft_active_buffer][1][write_index] =
        (q15_t)((int32_t)ch2_raw_code - 32768);
    measurement_fft_sample_count[0] = write_index + 1u;
    measurement_fft_sample_count[1] = write_index + 1u;

    if (measurement_fft_sample_count[0] == MEASUREMENT_FFT_LENGTH)
    {
        measurement_fft_ready_buffer = measurement_fft_active_buffer;
        measurement_fft_active_buffer ^= 1u;
        measurement_fft_sample_count[0] = 0u;
        measurement_fft_sample_count[1] = 0u;
        measurement_fft_state = MEASUREMENT_FFT_STATE_READY;
        measurement_fft_diagnostics.window_count++;
    }
}

/**
 * @brief 迁移期间接收一条旧 ADS8688 顺序采样记录。
 * @param channel 旧通道号，仅接收 0 与 1。
 * @param raw_code 旧 ADS8688 直二进制原始码。
 * @return 无。
 * @note 收齐两个通道后转交同步样本对入口；片上 ADC 正式链路不调用本函数。
 */
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
        measurement_fft_ingest_pair(measurement_fft_legacy_pair[0],
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

/**
 * @brief 处理已收齐窗口，计算并发布幅度、频率、相位差和波形类型。
 * @param 无。
 * @return 无。
 * @note FFT 完成后进入显示空档，避免 9600 波特率 HMI 阻塞发送污染下一连续采样窗口。
 */
void measurement_fft_process(void)
{
    int64_t peak_power[MEASUREMENT_FFT_CHANNEL_COUNT];
    measurement_fft_time_metrics_t time_metrics[MEASUREMENT_FFT_CHANNEL_COUNT];
    float span_vpp[MEASUREMENT_FFT_CHANNEL_COUNT];
    float peak_offset[MEASUREMENT_FFT_CHANNEL_COUNT];
    float peak_position[MEASUREMENT_FFT_CHANNEL_COUNT];
    uint16_t span_code[MEASUREMENT_FFT_CHANNEL_COUNT];
    measurement_fft_quality_t quality;
    measurement_mode_t mode;
    uint32_t cycle_start;
    uint16_t valid_mask;
    uint8_t voltage_status[MEASUREMENT_FFT_CHANNEL_COUNT];
    uint8_t channel;
    uint8_t valid;

    if (measurement_fft_state == MEASUREMENT_FFT_STATE_READY)
    {
        for (channel = 0u; channel < MEASUREMENT_FFT_CHANNEL_COUNT; channel++)
        {
            time_metrics[channel] = measurement_fft_analyze_time_domain(
                measurement_fft_input[measurement_fft_ready_buffer][channel]);
            span_code[channel] =
                time_metrics[channel].span.maximum_code
                - time_metrics[channel].span.minimum_code;
            span_vpp[channel] = NAN;
        }

        measurement_fft_diagnostics.clipping_mask = 0u;
        if (measurement_fft_span_is_clipped(time_metrics[0].span) != 0u)
        {
            measurement_fft_diagnostics.clipping_mask |= 0x01u;
        }
        if (measurement_fft_span_is_clipped(time_metrics[1].span) != 0u)
        {
            measurement_fft_diagnostics.clipping_mask |= 0x02u;
        }

        voltage_status[0] = measurement_fft_convert_time_metrics(
            time_metrics[0],
            0u,
            &span_vpp[0],
            &measurement_fft_diagnostics.dc_voltage,
            &measurement_fft_diagnostics.rms_voltage);
        voltage_status[1] = measurement_fft_convert_time_metrics(
            time_metrics[1],
            1u,
            &span_vpp[1],
            &measurement_fft_diagnostics.secondary_dc_voltage,
            &measurement_fft_diagnostics.secondary_rms_voltage);
        measurement_fft_diagnostics.voltage_calibrated_mask =
            (uint8_t)((voltage_status[0] != 0u ? 0x01u : 0u)
                      | (voltage_status[1] != 0u ? 0x02u : 0u));
        measurement_fft_diagnostics.amplitude_vpp = span_vpp[0];
        measurement_fft_diagnostics.secondary_amplitude_vpp = span_vpp[1];

        for (channel = 0u; channel < MEASUREMENT_FFT_CHANNEL_COUNT; channel++)
        {
            measurement_fft_prepare_window(
                measurement_fft_input[measurement_fft_ready_buffer][channel]);
        }

        cycle_start = DWT->CYCCNT;
        for (channel = 0u; channel < MEASUREMENT_FFT_CHANNEL_COUNT; channel++)
        {
            measurement_fft_execute_q15(
                measurement_fft_input[measurement_fft_ready_buffer][channel],
                measurement_fft_output[channel]);
        }
        measurement_fft_diagnostics.last_fft_cycles = DWT->CYCCNT - cycle_start;

        measurement_fft_diagnostics.peak_bin =
            measurement_fft_find_peak_bin(measurement_fft_output[0],
                                          &peak_power[0]);
        measurement_fft_diagnostics.secondary_peak_bin =
            measurement_fft_find_peak_bin(measurement_fft_output[1],
                                          &peak_power[1]);
        peak_offset[0] = measurement_fft_interpolate_peak(
            measurement_fft_output[0], measurement_fft_diagnostics.peak_bin);
        peak_offset[1] = measurement_fft_interpolate_peak(
            measurement_fft_output[1],
            measurement_fft_diagnostics.secondary_peak_bin);
        peak_position[0] =
            (float)measurement_fft_diagnostics.peak_bin + peak_offset[0];
        peak_position[1] =
            (float)measurement_fft_diagnostics.secondary_peak_bin + peak_offset[1];

        measurement_fft_diagnostics.peak_offset_bins = peak_offset[0];
        measurement_fft_diagnostics.peak_frequency_hz =
            peak_position[0] * measurement_fft_diagnostics.bin_width_hz;
        measurement_fft_diagnostics.harmonic_ratio_3 =
            measurement_fft_harmonic_ratio(measurement_fft_output[0],
                                           peak_position[0],
                                           3u,
                                           peak_power[0]);
        measurement_fft_diagnostics.harmonic_ratio_5 =
            measurement_fft_harmonic_ratio(measurement_fft_output[0],
                                           peak_position[0],
                                           5u,
                                           peak_power[0]);
        measurement_fft_diagnostics.wave_type =
            measurement_fft_classify_wave(
                measurement_fft_diagnostics.harmonic_ratio_3,
                measurement_fft_diagnostics.harmonic_ratio_5);
        if (voltage_status[0] != 0u)
        {
            measurement_fft_diagnostics.amplitude_vpp =
                measurement_fft_rms_to_vpp(
                    measurement_fft_diagnostics.rms_voltage,
                    measurement_fft_diagnostics.wave_type,
                    span_vpp[0]);
        }
        else
        {
            measurement_fft_diagnostics.amplitude_vpp = NAN;
        }
        measurement_fft_diagnostics.thd_percent =
            measurement_fft_calculate_thd(
                measurement_fft_output[0],
                peak_position[0],
                &measurement_fft_diagnostics.thd_harmonic_count);
        measurement_fft_build_spectrum(measurement_fft_output[0],
                                       peak_power[0]);

        if (measurement_fft_diagnostics.peak_bin != 0u)
        {
            measurement_fft_diagnostics.raw_phase_deg =
                measurement_fft_calculate_phase(measurement_fft_output[0],
                                                measurement_fft_output[1],
                                                measurement_fft_diagnostics.peak_bin);
            measurement_fft_diagnostics.phase_deg = measurement_fft_wrap_phase(
                measurement_fft_diagnostics.raw_phase_deg);
        }
        else
        {
            measurement_fft_diagnostics.raw_phase_deg = 0.0f;
            measurement_fft_diagnostics.phase_deg = 0.0f;
        }

        quality = MEASUREMENT_FFT_QUALITY_OK;
        mode = MEASUREMENT_MODE_UNKNOWN;
        valid_mask = 0u;
        valid = 0u;
        if ((measurement_fft_diagnostics.clipping_mask & 0x01u) != 0u)
        {
            quality = MEASUREMENT_FFT_QUALITY_CLIPPED;
        }
        else
        {
            if (voltage_status[0] != 0u)
            {
                valid_mask |= MEASUREMENT_VALID_DC_VOLTAGE;
            }
            if (span_code[0] < MEASUREMENT_FFT_MINIMUM_SPAN_CODE)
            {
                mode = MEASUREMENT_MODE_DC;
                quality = MEASUREMENT_FFT_QUALITY_DC_INPUT;
                measurement_fft_diagnostics.peak_frequency_hz = 0.0f;
                measurement_fft_diagnostics.thd_percent = 0.0f;
                measurement_fft_diagnostics.thd_harmonic_count = 0u;
                measurement_fft_diagnostics.raw_phase_deg = 0.0f;
                measurement_fft_diagnostics.phase_deg = 0.0f;
                measurement_fft_diagnostics.wave_type =
                    MEASUREMENT_WAVE_UNKNOWN;
                measurement_fft_spectrum.valid = 0u;
            }
            else if ((peak_power[0] <= 0)
                     || (!isfinite(
                         measurement_fft_diagnostics.peak_frequency_hz)))
            {
                quality = MEASUREMENT_FFT_QUALITY_SIGNAL_TOO_SMALL;
            }
            else
            {
                mode = MEASUREMENT_MODE_AC;
                valid_mask |= MEASUREMENT_VALID_FREQUENCY;
                if ((voltage_status[0] != 0u)
                    && isfinite(measurement_fft_diagnostics.amplitude_vpp)
                    && isfinite(measurement_fft_diagnostics.rms_voltage))
                {
                    valid_mask |= MEASUREMENT_VALID_AMPLITUDE
                                  | MEASUREMENT_VALID_RMS;
                }
                if ((measurement_fft_diagnostics.thd_harmonic_count != 0u)
                    && isfinite(measurement_fft_diagnostics.thd_percent))
                {
                    valid_mask |= MEASUREMENT_VALID_THD;
                }
                if (measurement_fft_diagnostics.wave_type
                    != MEASUREMENT_WAVE_UNKNOWN)
                {
                    valid_mask |= MEASUREMENT_VALID_WAVE_TYPE;
                }
                if (measurement_fft_spectrum.valid != 0u)
                {
                    valid_mask |= MEASUREMENT_VALID_SPECTRUM;
                }

                if (((measurement_fft_diagnostics.clipping_mask & 0x02u)
                        != 0u)
                    || (span_code[1] < MEASUREMENT_FFT_MINIMUM_SPAN_CODE)
                    || (peak_power[1] <= 0)
                    || (fabsf(peak_position[0] - peak_position[1])
                        > MEASUREMENT_FFT_CHANNEL_MATCH_TOLERANCE_BINS)
                    || (!isfinite(measurement_fft_diagnostics.phase_deg)))
                {
                    quality = MEASUREMENT_FFT_QUALITY_CHANNEL_MISMATCH;
                }
                else
                {
                    valid_mask |= MEASUREMENT_VALID_PHASE;
                    quality = MEASUREMENT_FFT_QUALITY_OK;
                }
            }
        }

        valid =
            ((valid_mask & (MEASUREMENT_VALID_AMPLITUDE
                            | MEASUREMENT_VALID_FREQUENCY
                            | MEASUREMENT_VALID_PHASE))
             == (MEASUREMENT_VALID_AMPLITUDE
                 | MEASUREMENT_VALID_FREQUENCY
                 | MEASUREMENT_VALID_PHASE))
                ? 1u
                : 0u;

        measurement_fft_diagnostics.fft_count++;
        measurement_fft_diagnostics.fft_ready = 1u;
        measurement_fft_publish(valid, quality, mode, valid_mask);
        measurement_fft_display_start_ms = HAL_GetTick();
        measurement_fft_state = MEASUREMENT_FFT_STATE_DISPLAY;
        return;
    }

    if ((measurement_fft_state == MEASUREMENT_FFT_STATE_DISPLAY)
        && ((uint32_t)(HAL_GetTick() - measurement_fft_display_start_ms)
            >= MEASUREMENT_FFT_DISPLAY_GUARD_MS))
    {
        measurement_fft_settle_count[0] = 0u;
        measurement_fft_settle_count[1] = 0u;
        measurement_fft_state = MEASUREMENT_FFT_STATE_SETTLING;
    }
}

/**
 * @brief 判断当前是否允许执行串口屏刷新。
 * @param 无。
 * @return 非零表示处于 FFT 完成后的显示空档；零表示正在采样、处理或过渡。
 * @note 显示空档结束后自动开始下一帧采集。
 */
uint8_t measurement_fft_hmi_refresh_allowed(void)
{
    return (measurement_fft_state == MEASUREMENT_FFT_STATE_DISPLAY) ? 1u : 0u;
}

/**
 * @brief 获取最近一次 FFT 测量诊断快照。
 * @param diagnostics 用于接收诊断信息的指针。
 * @return 指针有效时返回 1，否则返回 0。
 * @note 仅供主循环或调试器读取，不修改 FFT 状态。
 */
uint8_t measurement_fft_get_diagnostics(measurement_fft_diagnostics_t *diagnostics)
{
    if (diagnostics == 0)
    {
        return 0u;
    }

    *diagnostics = measurement_fft_diagnostics;
    return 1u;
}

/**
 * @brief 获取最近一次压缩频谱快照。
 * @param spectrum 用于接收 64 点相对幅度频谱的指针。
 * @return 指针有效时返回 1，否则返回 0。
 * @note 只复制主循环维护的数据，不读取 FFT 工作缓冲区。
 */
uint8_t measurement_fft_get_spectrum(measurement_fft_spectrum_t *spectrum)
{
    if (spectrum == 0)
    {
        return 0u;
    }

    *spectrum = measurement_fft_spectrum;
    return 1u;
}

/**
 * @brief 设置 ADC1/CH1 或 ADC2/CH2 的软件校准系数。
 * @param channel 通道号，只允许 0 或 1。
 * @param calibration 码值比例、零码偏置和有效状态。
 * @return 参数有效时返回 1，否则返回 0。
 * @note 应在主循环上下文调用；新系数从下一帧时域换算开始生效。
 */
uint8_t measurement_fft_set_calibration(
    uint8_t channel,
    const measurement_fft_calibration_t *calibration)
{
    if ((channel >= MEASUREMENT_FFT_CHANNEL_COUNT)
        || (calibration == 0)
        || (calibration->valid > 1u)
        || (!isfinite(calibration->offset_v)))
    {
        return 0u;
    }
    if ((calibration->valid != 0u)
        && ((!isfinite(calibration->volts_per_code))
            || (calibration->volts_per_code == 0.0f)))
    {
        return 0u;
    }

    measurement_fft_calibration[channel] = *calibration;
    if (calibration->valid != 0u)
    {
        measurement_fft_diagnostics.voltage_calibrated_mask |=
            (uint8_t)(1u << channel);
    }
    else
    {
        measurement_fft_diagnostics.voltage_calibrated_mask &=
            (uint8_t)~(1u << channel);
    }
    return 1u;
}

/**
 * @brief 读取 ADC1/CH1 或 ADC2/CH2 当前的软件校准系数。
 * @param channel 通道号，只允许 0 或 1。
 * @param calibration 用于接收校准系数的指针。
 * @return 参数有效时返回 1，否则返回 0。
 * @note 只复制主循环维护的数据，不访问 ADC 或 FFT 缓冲区。
 */
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
