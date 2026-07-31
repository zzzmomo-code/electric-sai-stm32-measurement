/**
 * @file measurement_conversion.c
 * @brief FPGA 测量快照到串口屏显示缓存的转换实现。
 *
 * 模块用途：根据采样率和基频定位上升过零点，从一个完整基波周期周期重采样出
 *          严格的一周期/三周期；
 *          对频谱分桶取最大值，并映射成淘晶驰 Waveform 可接受的 8 位纵轴数据。
 * GPIO 引脚映射：无直接 GPIO 引脚。
 * 依赖的外设和 CubeIDE 配置：无直接外设依赖。
 * 初始化方法：system_init() 调用 measurement_conversion_init()。
 * 调用方法：主循环收到新的 FPGA 快照后调用一次 update。
 */

#include "system.h"

#include <string.h>

/** 两份显示快照交替写入，避免 HMI 读取到半更新数组。 */
static measurement_display_snapshot_t measurement_display_snapshots[2];
static uint8_t measurement_display_active_index;

volatile measurement_conversion_diagnostics_t
    measurement_conversion_diagnostics;

/*
 * 本模块只改变“显示点数和纵轴坐标”，不重新计算 Vpp、Vrms、频率或谐波。
 *
 * FPGA快照                           显示快照
 * time_samples[最多3750]  ────────> waveform_3cycle[350]
 * 同相位起点一个周期     ─────────> waveform_1cycle[350]
 * spectrum[1312]          ─────────> spectrum_display[350]
 * 帧头参数                ─────────> Vpp/Vrms/基频/三个分量
 *
 * 三个输出数组带同一个 frame_sequence；HMI 只有看到完整发布的显示快照后
 * 才开始预装，因此不会把不同 FPGA 帧的曲线和参数混在一起。
 */

/**
 * @brief 按 FPGA 固定±15000量程映射一个时域原始值。
 * @param value 当前样点。
 * @return 8~201；超量程输入先限幅，零值固定落在纵轴中部。
 */
static uint8_t measurement_conversion_map_time(int32_t value)
{
    uint32_t scaled;

    if (value < -MEASUREMENT_TIME_DISPLAY_LIMIT)
    {
        value = -MEASUREMENT_TIME_DISPLAY_LIMIT;
    }
    else if (value > MEASUREMENT_TIME_DISPLAY_LIMIT)
    {
        value = MEASUREMENT_TIME_DISPLAY_LIMIT;
    }

    scaled = (((uint32_t)(value + MEASUREMENT_TIME_DISPLAY_LIMIT)
               * (MEASUREMENT_DISPLAY_Y_MAX
                  - MEASUREMENT_DISPLAY_Y_MIN))
              + MEASUREMENT_TIME_DISPLAY_LIMIT)
             / (2u * MEASUREMENT_TIME_DISPLAY_LIMIT);
    return (uint8_t)(MEASUREMENT_DISPLAY_Y_MIN + scaled);
}

/**
 * @brief 将任意长度时域片段重采样为屏幕控件宽度对应的点数。
 * @param source 原始 int16 样点。
 * @param source_count 原始点数。
 * @param period_q16 单周期 Q16.16 样点长度。
 * @param cycle_count 横轴需要显示的基波周期数。
 * @param minimum 单周期窗口最小值。
 * @param maximum 单周期窗口最大值。
 * @param output 固定宽度显示点输出。
 * @return 无。
 */
#define MEASUREMENT_TIME_POSITION_FRACTION_BITS 16u
#define MEASUREMENT_TIME_POSITION_ONE \
    (1ul << MEASUREMENT_TIME_POSITION_FRACTION_BITS)

/**
 * @brief 按本帧识别出的编码方式取得一个有符号时域样点。
 * @param source FPGA 时域数组。
 * @param index 样点下标。
 * @param offset_binary 非零表示载荷实际为偏移二进制。
 * @return 以零为中心的有符号样点。
 *
 * @note 冻结协议规定载荷应为二补码，但实板曲线表明当前 FPGA 可能直接发送
 *       ADC 偏移二进制码。异或 0x8000 可完成偏移二进制到二补码的转换。
 */
static int16_t measurement_conversion_decode_time_sample(
    const int16_t *source,
    uint16_t index,
    uint8_t offset_binary)
{
    uint16_t raw = (uint16_t)source[index];

    if (offset_binary != 0u)
    {
        raw ^= 0x8000u;
    }
    return (int16_t)raw;
}

/**
 * @brief 比较两种解码结果的相邻点总跳变量，自动识别时域样点编码。
 * @param source FPGA 时域数组。
 * @param source_count 样点数量。
 * @return 1=偏移二进制，0=协议规定的二补码。
 *
 * @note 连续波形若编码解释错误，会在过零附近产生约满量程的跳变。只有偏移
 *       二进制方案的总跳变量至少小一半时才切换，模糊情况仍遵守冻结协议。
 */
static uint8_t measurement_conversion_detect_time_encoding(
    const int16_t *source,
    uint16_t source_count)
{
    uint64_t twos_complement_variation = 0u;
    uint64_t offset_binary_variation = 0u;
    int32_t previous_twos_complement;
    int32_t previous_offset_binary;
    uint16_t index;

    if ((source == NULL) || (source_count < 2u))
    {
        return 0u;
    }

    previous_twos_complement = source[0];
    previous_offset_binary = (int16_t)(
        ((uint16_t)source[0]) ^ 0x8000u);
    for (index = 1u; index < source_count; index++)
    {
        int32_t current_twos_complement = source[index];
        int32_t current_offset_binary = (int16_t)(
            ((uint16_t)source[index]) ^ 0x8000u);
        int32_t twos_complement_delta =
            current_twos_complement - previous_twos_complement;
        int32_t offset_binary_delta =
            current_offset_binary - previous_offset_binary;

        twos_complement_variation += (uint32_t)(
            (twos_complement_delta >= 0)
                ? twos_complement_delta : -twos_complement_delta);
        offset_binary_variation += (uint32_t)(
            (offset_binary_delta >= 0)
                ? offset_binary_delta : -offset_binary_delta);
        previous_twos_complement = current_twos_complement;
        previous_offset_binary = current_offset_binary;
    }

    return ((offset_binary_variation * 2u)
            < twos_complement_variation) ? 1u : 0u;
}

/**
 * @brief 按 Q16.16 样点位置对时域数组做线性插值。
 * @param source 原始时域样点。
 * @param source_count 原始样点数。
 * @param position_q16 Q16.16 样点位置。
 * @param offset_binary 非零表示载荷实际为偏移二进制。
 * @return 插值后的有符号样点。
 */
static int32_t measurement_conversion_interpolate_time(
    const int16_t *source,
    uint16_t source_count,
    uint32_t position_q16,
    uint8_t offset_binary)
{
    uint32_t left_index =
        position_q16 >> MEASUREMENT_TIME_POSITION_FRACTION_BITS;
    uint32_t fraction =
        position_q16 & (MEASUREMENT_TIME_POSITION_ONE - 1u);
    int32_t left;
    int32_t right;

    if (left_index >= source_count)
    {
        left_index = source_count - 1u;
    }
    left = measurement_conversion_decode_time_sample(
        source, (uint16_t)left_index, offset_binary);
    if ((left_index + 1u) >= source_count)
    {
        return left;
    }

    right = measurement_conversion_decode_time_sample(
        source, (uint16_t)(left_index + 1u), offset_binary);
    return left
        + (int32_t)(((int64_t)(right - left) * fraction)
                    / MEASUREMENT_TIME_POSITION_ONE);
}

/**
 * @brief 从指定相位起点把精确的分数样点区间重采样为350点。
 * @param source 原始时域样点。
 * @param source_count 原始点数。
 * @param start_q16 起始 Q16.16 样点位置。
 * @param span_q16 覆盖的 Q16.16 样点跨度。
 * @param offset_binary 非零表示载荷实际为偏移二进制。
 * @param output 350点显示缓存。
 * @return 无。
 */
static void measurement_conversion_resample_periodic(
    const int16_t *source,
    uint16_t source_count,
    uint32_t start_q16,
    uint32_t period_q16,
    uint8_t cycle_count,
    uint8_t offset_binary,
    uint8_t *output)
{
    uint16_t output_index;
    uint64_t span_q16 = (uint64_t)period_q16 * cycle_count;

    for (output_index = 0u;
         output_index < MEASUREMENT_DISPLAY_POINT_COUNT;
         output_index++)
    {
        uint32_t phase_q16 = (uint32_t)(
            (span_q16 * output_index)
            / (MEASUREMENT_DISPLAY_POINT_COUNT - 1u));
        uint32_t position_q16;

        /*
         * 三周期图以同一个已对齐的完整基波周期为模板做周期延拓。
         * 这样即使FPGA缓存只有N个样点而没有第N+1个闭合端点，也不会拒绝
         * 400~500 kHz合法帧；最后一点恰好回到相同相位。
         */
        if (period_q16 != 0u)
        {
            phase_q16 %= period_q16;
        }
        position_q16 = start_q16 + phase_q16;
        int32_t interpolated =
            measurement_conversion_interpolate_time(
                source, source_count, position_q16,
                offset_binary);

        output[output_index] =
            measurement_conversion_map_time(interpolated);
    }
}

/**
 * @brief 按采样率和基频计算周期，并寻找能容纳一个完整周期的上升过零点。
 * @param source 已校验的 FPGA 测量快照。
 * @param start_q16 输出相位对齐后的 Q16.16 起点。
 * @param period_q16 输出单周期 Q16.16 长度。
 * @param offset_binary 非零表示载荷实际为偏移二进制。
 * @param minimum 输出单周期窗口最小值。
 * @param maximum 输出单周期窗口最大值。
 * @return 找到合法单周期模板返回1，否则返回0。
 *
 * @note 不再相信 time_count/captured_cycles 等于一个周期长度。实际周期只由
 *       time_sample_rate_hz / fundamental_mhz 决定，因此400 kHz等高频输入
 *       也会在横轴上精确显示一个或三个周期。
 */
static uint8_t measurement_conversion_find_cycle_window(
    const fpga_measurement_snapshot_t *source,
    uint32_t *start_q16,
    uint32_t *period_q16,
    uint8_t offset_binary,
    int16_t *minimum,
    int16_t *maximum)
{
    uint64_t calculated_period;
    uint32_t maximum_position_q16;
    uint32_t preferred_position_q16;
    uint32_t first_valid_start = UINT32_MAX;
    uint32_t selected_start = UINT32_MAX;
    int64_t sum = 0;
    int32_t center;
    uint16_t index;
    uint16_t begin_index;
    uint16_t end_index;

    if ((source->header.time_sample_rate_hz == 0u)
        || (source->header.fundamental_mhz == 0u)
        || (source->header.time_count < 2u))
    {
        return 0u;
    }

    calculated_period =
        (((uint64_t)source->header.time_sample_rate_hz * 1000u)
         << MEASUREMENT_TIME_POSITION_FRACTION_BITS)
        / source->header.fundamental_mhz;
    if ((calculated_period == 0u)
        || (calculated_period > UINT32_MAX))
    {
        return 0u;
    }

    *period_q16 = (uint32_t)calculated_period;
    maximum_position_q16 =
        ((uint32_t)source->header.time_count - 1u)
        << MEASUREMENT_TIME_POSITION_FRACTION_BITS;
    if (*period_q16 > maximum_position_q16)
    {
        return 0u;
    }

    for (index = 0u; index < source->header.time_count; index++)
    {
        sum += measurement_conversion_decode_time_sample(
            source->time_samples, index, offset_binary);
    }
    center = (int32_t)(sum / source->header.time_count);
    preferred_position_q16 =
        ((uint32_t)source->header.time_count / 4u)
        << MEASUREMENT_TIME_POSITION_FRACTION_BITS;

    for (index = 1u; index < source->header.time_count; index++)
    {
        int32_t left = measurement_conversion_decode_time_sample(
            source->time_samples, (uint16_t)(index - 1u),
            offset_binary);
        int32_t right = measurement_conversion_decode_time_sample(
            source->time_samples, index, offset_binary);

        if ((left <= center) && (right > center))
        {
            uint32_t fraction = (uint32_t)(
                ((uint64_t)(uint32_t)(center - left)
                 << MEASUREMENT_TIME_POSITION_FRACTION_BITS)
                / (uint32_t)(right - left));
            uint32_t crossing_q16 =
                ((uint32_t)(index - 1u)
                 << MEASUREMENT_TIME_POSITION_FRACTION_BITS)
                + fraction;

            if ((crossing_q16 + *period_q16)
                <= maximum_position_q16)
            {
                if (first_valid_start == UINT32_MAX)
                {
                    first_valid_start = crossing_q16;
                }
                if (crossing_q16 >= preferred_position_q16)
                {
                    selected_start = crossing_q16;
                    break;
                }
            }
        }
    }

    if (selected_start == UINT32_MAX)
    {
        selected_start = first_valid_start;
    }
    if (selected_start == UINT32_MAX)
    {
        /* 没找到可靠上升过零点时仍按真实周期截取，但从缓冲区首点开始。 */
        selected_start = 0u;
    }
    *start_q16 = selected_start;

    begin_index = (uint16_t)(
        selected_start >> MEASUREMENT_TIME_POSITION_FRACTION_BITS);
    end_index = (uint16_t)(
        (selected_start + *period_q16
         + MEASUREMENT_TIME_POSITION_ONE - 1u)
        >> MEASUREMENT_TIME_POSITION_FRACTION_BITS);
    if (end_index >= source->header.time_count)
    {
        end_index = (uint16_t)(source->header.time_count - 1u);
    }

    *minimum = measurement_conversion_decode_time_sample(
        source->time_samples, begin_index, offset_binary);
    *maximum = *minimum;
    for (index = (uint16_t)(begin_index + 1u);
         index <= end_index;
         index++)
    {
        int16_t sample = measurement_conversion_decode_time_sample(
            source->time_samples, index, offset_binary);

        if (sample < *minimum)
        {
            *minimum = sample;
        }
        if (sample > *maximum)
        {
            *maximum = sample;
        }
    }
    return 1u;
}

/**
 * @brief 将 1312 点频谱分桶取最大值并映射到屏幕控件宽度。
 * @param source 原始频谱。
 * @param source_count 原始点数。
 * @param output 输出固定宽度显示点。
 * @return 原始频谱最大值。
 */
static uint16_t measurement_conversion_compress_spectrum(
    const uint16_t *source,
    uint16_t source_count,
    uint8_t *output)
{
    uint16_t source_maximum = 0u;
    uint16_t output_index;
    uint16_t input_index;

    for (input_index = 0u; input_index < source_count; input_index++)
    {
        if (source[input_index] > source_maximum)
        {
            source_maximum = source[input_index];
        }
    }

    /*
     * 频谱与时域不同：每个横向桶取最大值而不是均值，避免很窄的谐波谱线
     * 在 1312 点频谱压缩过程中被平均掉。
     */
    for (output_index = 0u;
         output_index < MEASUREMENT_DISPLAY_POINT_COUNT;
         output_index++)
    {
        uint16_t display_index = (uint16_t)(
            MEASUREMENT_DISPLAY_POINT_COUNT - 1u - output_index);
        uint32_t begin =
            ((uint32_t)output_index * source_count)
            / MEASUREMENT_DISPLAY_POINT_COUNT;
        uint32_t end =
            ((uint32_t)(output_index + 1u) * source_count)
            / MEASUREMENT_DISPLAY_POINT_COUNT;
        uint16_t bucket_maximum = 0u;
        uint32_t index;

        if (end <= begin)
        {
            end = begin + 1u;
        }
        if (end > source_count)
        {
            end = source_count;
        }
        for (index = begin; index < end; index++)
        {
            if (source[index] > bucket_maximum)
            {
                bucket_maximum = source[index];
            }
        }

        if (source_maximum == 0u)
        {
            output[display_index] = MEASUREMENT_DISPLAY_Y_MIN;
        }
        else
        {
            /*
             * 淘晶驰曲线控件使用add逐点滚动，最终屏幕上的横向排列与发送数组相反。
             * 因此这里反向存储，使低频落在左侧、高频落在右侧。
             */
            output[display_index] = (uint8_t)(
                MEASUREMENT_DISPLAY_Y_MIN
                + (((uint32_t)bucket_maximum
                    * (MEASUREMENT_DISPLAY_Y_MAX
                       - MEASUREMENT_DISPLAY_Y_MIN))
                   / source_maximum));
        }
    }

    return source_maximum;
}

/**
 * @brief 将有效频率分量按谐波次数、再按频率从小到大排序。
 * @param snapshot 待整理的显示快照。
 * @return 无。
 *
 * @note FPGA 槽位允许无序；稳定排序后，连续多帧可按相同下标安全累加平均。
 */
static void measurement_conversion_sort_components(
    measurement_display_snapshot_t *snapshot)
{
    uint8_t index;

    for (index = 1u; index < snapshot->component_count; index++)
    {
        fpga_protocol_component_t value = snapshot->component[index];
        uint8_t position = index;

        while (position > 0u)
        {
            fpga_protocol_component_t *previous =
                &snapshot->component[position - 1u];
            uint8_t value_before_previous = (uint8_t)(
                (value.harmonic_order < previous->harmonic_order)
                || ((value.harmonic_order == previous->harmonic_order)
                    && (value.frequency_mhz
                        < previous->frequency_mhz)));

            if (value_before_previous == 0u)
            {
                break;
            }
            snapshot->component[position] = *previous;
            position--;
        }
        snapshot->component[position] = value;
    }
}

/**
 * @brief 清零双显示快照和诊断量。
 * @param 无。
 * @return 无。
 */
void measurement_conversion_init(void)
{
    memset((void *)measurement_display_snapshots, 0,
           sizeof(measurement_display_snapshots));
    memset((void *)&measurement_conversion_diagnostics, 0,
           sizeof(measurement_conversion_diagnostics));
    measurement_display_active_index = 0u;
}

/**
 * @brief 把一份完整 FPGA 快照转换为三组 350 点显示数据并原子发布。
 * @param source 已通过协议、长度和 CRC 校验的只读快照。
 * @return 转换成功返回 1，输入字段无效返回 0。
 */
uint8_t measurement_conversion_update(
    const fpga_measurement_snapshot_t *source)
{
    measurement_display_snapshot_t *target;
    uint8_t target_index;
    uint16_t index;
    uint32_t cycle_start_q16;
    uint32_t period_q16;
    int16_t time_minimum;
    int16_t time_maximum;
    uint16_t time_rail_sample_count = 0u;
    uint16_t time_display_clip_count = 0u;
    uint8_t time_offset_binary;
    uint8_t output_component_index = 0u;

    if ((source == NULL) || (source->valid == 0u)
        || (source->header.time_count == 0u)
        || (source->header.time_count
            > FPGA_PROTOCOL_MAX_TIME_SAMPLES)
        || (source->header.spectrum_count
            != FPGA_PROTOCOL_SPECTRUM_COUNT))
    {
        measurement_conversion_diagnostics.invalid_source_count++;
        return 0u;
    }

    /* 和 FPGA 层相同：先写非活动快照，完成后再一次性切换活动索引。 */
    target_index = (uint8_t)(measurement_display_active_index ^ 1u);
    target = &measurement_display_snapshots[target_index];
    memset((void *)target, 0, sizeof(*target));

    time_offset_binary = measurement_conversion_detect_time_encoding(
        source->time_samples, source->header.time_count);
    if (measurement_conversion_find_cycle_window(
            source, &cycle_start_q16, &period_q16,
            time_offset_binary,
            &time_minimum, &time_maximum) == 0u)
    {
        measurement_conversion_diagnostics.invalid_source_count++;
        return 0u;
    }

    measurement_conversion_resample_periodic(
        source->time_samples,
        source->header.time_count,
        cycle_start_q16,
        period_q16,
        3u,
        time_offset_binary,
        target->waveform_3cycle);

    measurement_conversion_resample_periodic(
        source->time_samples,
        source->header.time_count,
        cycle_start_q16,
        period_q16,
        1u,
        time_offset_binary,
        target->waveform_1cycle);
    measurement_conversion_diagnostics.last_spectrum_max =
        measurement_conversion_compress_spectrum(
            source->spectrum,
            source->header.spectrum_count,
            target->spectrum_display);

    target->frame_sequence = source->header.frame_seq;
    target->timestamp_50m = source->header.timestamp_50mhz;
    target->source_flags = source->header.flags;
    target->vpp_uv = source->header.vpp_uv;
    target->vrms_uv = source->header.vrms_uv;
    target->fundamental_mhz = source->header.fundamental_mhz;
    target->dc_offset_uv = source->header.dc_uv;
    target->dropped_frames = source->header.dropped_frame_count;
    target->calibration_revision =
        source->header.calibration_version;
    for (index = 0u;
         index < FPGA_PROTOCOL_COMPONENT_MAX;
         index++)
    {
        const fpga_protocol_component_t *component =
            &source->header.component[index];

        /*
         * FPGA 三个候选槽位独立有效，可能出现 101 这种非连续排列。
         * 显示快照把所有 VALID 槽位依次压紧，HMI 才能稳定使用
         * t_comp1~t_comp3；无效槽位完全不参与参数显示。
         */
        if ((component->flags & FPGA_PROTOCOL_COMPONENT_VALID) != 0u)
        {
            target->component[output_component_index] = *component;
            output_component_index++;
        }
    }
    target->component_count = output_component_index;
    measurement_conversion_sort_components(target);

    target->valid = 1u;
    /* 保证数组和参数先写完，再让读取者看到新的活动索引。 */
    __DMB();
    measurement_display_active_index = target_index;

    measurement_conversion_diagnostics.conversion_count++;
    measurement_conversion_diagnostics.last_frame_sequence =
        target->frame_sequence;
    measurement_conversion_diagnostics.last_time_min = time_minimum;
    measurement_conversion_diagnostics.last_time_max = time_maximum;
    for (index = 0u; index < source->header.time_count; index++)
    {
        int16_t sample = measurement_conversion_decode_time_sample(
            source->time_samples, index, time_offset_binary);

        if ((sample <= -32700) || (sample >= 32700))
        {
            time_rail_sample_count++;
        }
        if ((sample < -MEASUREMENT_TIME_DISPLAY_LIMIT)
            || (sample > MEASUREMENT_TIME_DISPLAY_LIMIT))
        {
            time_display_clip_count++;
        }
    }
    measurement_conversion_diagnostics.last_time_rail_sample_count =
        time_rail_sample_count;
    measurement_conversion_diagnostics.last_time_display_clip_count =
        time_display_clip_count;
    measurement_conversion_diagnostics.last_time_offset_binary =
        time_offset_binary;
    if (time_offset_binary != 0u)
    {
        measurement_conversion_diagnostics.offset_binary_frame_count++;
    }
    measurement_conversion_diagnostics.last_one_cycle_samples =
        (uint16_t)((period_q16
                    + (MEASUREMENT_TIME_POSITION_ONE / 2u))
                   >> MEASUREMENT_TIME_POSITION_FRACTION_BITS);
    return 1u;
}

/**
 * @brief 获取当前完整发布的显示快照。
 * @param snapshot 输出只读快照地址。
 * @return 已有有效快照返回 1，否则返回 0。
 */
uint8_t measurement_conversion_get_snapshot(
    const measurement_display_snapshot_t **snapshot)
{
    const measurement_display_snapshot_t *active;

    if (snapshot == NULL)
    {
        return 0u;
    }

    active = &measurement_display_snapshots[
        measurement_display_active_index];
    *snapshot = active;
    return active->valid;
}

/**
 * @brief 旧页面兼容的频率换算函数。
 * @param timer_frequency_hz 定时器测得频率。
 * @param dds_frequency_hz DDS 频率。
 * @param adc_frequency_hz ADC 频率。
 * @return 旧页面所需频率；G 题正式链路不调用。
 */
float measurement_conversion_frequency_hz(float timer_frequency_hz,
                                           float dds_frequency_hz,
                                           float adc_frequency_hz)
{
    if ((dds_frequency_hz > 0.0f) && (adc_frequency_hz > 0.0f))
    {
        return dds_frequency_hz + adc_frequency_hz;
    }

    return timer_frequency_hz;
}

/**
 * @brief 旧页面兼容的幅度换算函数。
 * @param adc_amplitude_vpp ADC 峰峰值。
 * @param vga_level VGA 档位，当前兼容实现未使用。
 * @return 原样返回 adc_amplitude_vpp；G 题正式链路不调用。
 */
float measurement_conversion_amplitude_vpp(float adc_amplitude_vpp,
                                            uint8_t vga_level)
{
    (void)vga_level;
    return adc_amplitude_vpp;
}
