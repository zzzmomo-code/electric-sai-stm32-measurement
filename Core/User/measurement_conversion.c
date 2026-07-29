/**
 * @file measurement_conversion.c
 * @brief FPGA 测量快照到串口屏显示缓存的转换实现。
 *
 * 模块用途：对时域数据分段平均或线性插值，对频谱分桶取最大值，并映射
 *          成淘晶驰 Waveform 可接受的 8 位纵轴数据。
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

/**
 * @brief 将一个时域原始值映射到保留上下边距的 8 位纵轴。
 * @param value 当前样点。
 * @param minimum 当前快照最小值。
 * @param maximum 当前快照最大值。
 * @return 8~247；常量输入返回中点。
 */
static uint8_t measurement_conversion_map_time(
    int32_t value,
    int32_t minimum,
    int32_t maximum)
{
    uint32_t span;
    uint32_t scaled;

    if (maximum <= minimum)
    {
        return (uint8_t)(
            (MEASUREMENT_DISPLAY_Y_MIN + MEASUREMENT_DISPLAY_Y_MAX)
            / 2u);
    }

    span = (uint32_t)(maximum - minimum);
    scaled = ((uint32_t)(value - minimum)
              * (MEASUREMENT_DISPLAY_Y_MAX
                 - MEASUREMENT_DISPLAY_Y_MIN))
             / span;
    return (uint8_t)(MEASUREMENT_DISPLAY_Y_MIN + scaled);
}

/**
 * @brief 将任意长度时域片段重采样为 700 点。
 * @param source 原始 int16 样点。
 * @param source_count 原始点数。
 * @param minimum 整帧最小值。
 * @param maximum 整帧最大值。
 * @param output 700 点输出。
 * @return 无。
 */
static void measurement_conversion_resample_time(
    const int16_t *source,
    uint16_t source_count,
    int16_t minimum,
    int16_t maximum,
    uint8_t *output)
{
    uint16_t output_index;

    if (source_count >= MEASUREMENT_DISPLAY_POINT_COUNT)
    {
        for (output_index = 0u;
             output_index < MEASUREMENT_DISPLAY_POINT_COUNT;
             output_index++)
        {
            uint32_t begin =
                ((uint32_t)output_index * source_count)
                / MEASUREMENT_DISPLAY_POINT_COUNT;
            uint32_t end =
                ((uint32_t)(output_index + 1u) * source_count)
                / MEASUREMENT_DISPLAY_POINT_COUNT;
            int64_t sum = 0;
            uint32_t input_index;
            int32_t average;

            if (end <= begin)
            {
                end = begin + 1u;
            }
            if (end > source_count)
            {
                end = source_count;
            }
            for (input_index = begin; input_index < end; input_index++)
            {
                sum += source[input_index];
            }
            average = (int32_t)(sum / (int64_t)(end - begin));
            output[output_index] = measurement_conversion_map_time(
                average, minimum, maximum);
        }
    }
    else
    {
        for (output_index = 0u;
             output_index < MEASUREMENT_DISPLAY_POINT_COUNT;
             output_index++)
        {
            uint32_t position_numerator;
            uint32_t left_index;
            uint32_t remainder;
            int32_t interpolated;

            if (source_count <= 1u)
            {
                interpolated = source[0];
            }
            else
            {
                position_numerator =
                    (uint32_t)output_index * (source_count - 1u);
                left_index = position_numerator
                    / (MEASUREMENT_DISPLAY_POINT_COUNT - 1u);
                remainder = position_numerator
                    % (MEASUREMENT_DISPLAY_POINT_COUNT - 1u);

                if ((left_index + 1u) >= source_count)
                {
                    interpolated = source[left_index];
                }
                else
                {
                    int32_t left = source[left_index];
                    int32_t right = source[left_index + 1u];

                    interpolated = left
                        + ((right - left) * (int32_t)remainder)
                          / (int32_t)(
                              MEASUREMENT_DISPLAY_POINT_COUNT - 1u);
                }
            }

            output[output_index] = measurement_conversion_map_time(
                interpolated, minimum, maximum);
        }
    }
}

/**
 * @brief 将 1312 点频谱分桶取最大值并映射到 700 点。
 * @param source 原始频谱。
 * @param source_count 原始点数。
 * @param output 输出 700 点。
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

    for (output_index = 0u;
         output_index < MEASUREMENT_DISPLAY_POINT_COUNT;
         output_index++)
    {
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
            output[output_index] = MEASUREMENT_DISPLAY_Y_MIN;
        }
        else
        {
            output[output_index] = (uint8_t)(
                MEASUREMENT_DISPLAY_Y_MIN
                + (((uint32_t)bucket_maximum
                    * (MEASUREMENT_DISPLAY_Y_MAX
                       - MEASUREMENT_DISPLAY_Y_MIN))
                   / source_maximum));
        }
    }

    return source_maximum;
}

void measurement_conversion_init(void)
{
    memset((void *)measurement_display_snapshots, 0,
           sizeof(measurement_display_snapshots));
    memset((void *)&measurement_conversion_diagnostics, 0,
           sizeof(measurement_conversion_diagnostics));
    measurement_display_active_index = 0u;
}

uint8_t measurement_conversion_update(
    const fpga_measurement_snapshot_t *source)
{
    measurement_display_snapshot_t *target;
    uint8_t target_index;
    uint16_t index;
    uint16_t one_cycle_count;
    uint16_t one_cycle_start;
    int16_t time_minimum;
    int16_t time_maximum;

    if ((source == NULL) || (source->valid == 0u)
        || (source->header.time_count == 0u)
        || (source->header.time_count
            > FPGA_PROTOCOL_MAX_TIME_SAMPLES)
        || (source->header.spectrum_count
            != FPGA_PROTOCOL_SPECTRUM_COUNT)
        || (source->header.captured_cycles == 0u))
    {
        measurement_conversion_diagnostics.invalid_source_count++;
        return 0u;
    }

    target_index = (uint8_t)(measurement_display_active_index ^ 1u);
    target = &measurement_display_snapshots[target_index];
    memset((void *)target, 0, sizeof(*target));

    time_minimum = source->time_samples[0];
    time_maximum = source->time_samples[0];
    for (index = 1u; index < source->header.time_count; index++)
    {
        if (source->time_samples[index] < time_minimum)
        {
            time_minimum = source->time_samples[index];
        }
        if (source->time_samples[index] > time_maximum)
        {
            time_maximum = source->time_samples[index];
        }
    }

    measurement_conversion_resample_time(
        source->time_samples,
        source->header.time_count,
        time_minimum,
        time_maximum,
        target->waveform_3cycle);

    one_cycle_count = (uint16_t)(
        source->header.time_count / source->header.captured_cycles);
    if (one_cycle_count == 0u)
    {
        measurement_conversion_diagnostics.invalid_source_count++;
        return 0u;
    }
    one_cycle_start = (uint16_t)(
        ((uint16_t)source->header.captured_cycles / 2u)
        * one_cycle_count);
    if (((uint32_t)one_cycle_start + one_cycle_count)
        > source->header.time_count)
    {
        one_cycle_start = (uint16_t)(
            source->header.time_count - one_cycle_count);
    }

    measurement_conversion_resample_time(
        &source->time_samples[one_cycle_start],
        one_cycle_count,
        time_minimum,
        time_maximum,
        target->waveform_1cycle);
    measurement_conversion_diagnostics.last_spectrum_max =
        measurement_conversion_compress_spectrum(
            source->spectrum,
            source->header.spectrum_count,
            target->spectrum_display);

    target->frame_sequence = source->header.frame_seq;
    target->timestamp_50m = source->header.timestamp_50m;
    target->source_flags = source->header.flags;
    target->vpp_uv = source->header.vpp_uv;
    target->vrms_uv = source->header.vrms_uv;
    target->fundamental_mhz = source->header.fundamental_mhz;
    target->dc_offset_uv = source->header.dc_offset_uv;
    target->component_count = source->header.component_count;
    target->dropped_frames = source->header.dropped_frames;
    target->calibration_revision =
        source->header.calibration_revision;
    for (index = 0u;
         index < FPGA_PROTOCOL_COMPONENT_MAX;
         index++)
    {
        target->component[index] = source->header.component[index];
    }

    target->valid = 1u;
    __DMB();
    measurement_display_active_index = target_index;

    measurement_conversion_diagnostics.conversion_count++;
    measurement_conversion_diagnostics.last_frame_sequence =
        target->frame_sequence;
    measurement_conversion_diagnostics.last_time_min = time_minimum;
    measurement_conversion_diagnostics.last_time_max = time_maximum;
    measurement_conversion_diagnostics.last_one_cycle_samples =
        one_cycle_count;
    return 1u;
}

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

float measurement_conversion_amplitude_vpp(float adc_amplitude_vpp,
                                            uint8_t vga_level)
{
    (void)vga_level;
    return adc_amplitude_vpp;
}
