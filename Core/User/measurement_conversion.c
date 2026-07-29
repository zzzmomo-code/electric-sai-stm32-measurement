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

/*
 * 本模块只改变“显示点数和纵轴坐标”，不重新计算 Vpp、Vrms、频率或谐波。
 *
 * FPGA快照                           显示快照
 * time_samples[最多3750]  ────────> waveform_3cycle[350]
 * 中间一个完整周期       ─────────> waveform_1cycle[350]
 * spectrum[1312]          ─────────> spectrum_display[350]
 * 帧头参数                ─────────> Vpp/Vrms/基频/三个分量
 *
 * 三个输出数组带同一个 frame_sequence；HMI 只有看到完整发布的显示快照后
 * 才开始预装，因此不会把不同 FPGA 帧的曲线和参数混在一起。
 */

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
 * @brief 将任意长度时域片段重采样为屏幕控件宽度对应的点数。
 * @param source 原始 int16 样点。
 * @param source_count 原始点数。
 * @param minimum 整帧最小值。
 * @param maximum 整帧最大值。
 * @param output 固定宽度显示点输出。
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
        /*
         * 原始点较多时，把输入分成固定数量的连续区间，每区间取均值。
         * 这样横轴始终铺满控件，并抑制单个采样毛刺。
         */
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
        /*
         * 原始点少于显示点数时，相邻样点做线性插值。不能简单重复最后一个点，
         * 否则有效波形只会挤在横轴左侧。
         */
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

    /* 和 FPGA 层相同：先写非活动快照，完成后再一次性切换活动索引。 */
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

    /*
     * FPGA 发送固定三周期数据。这里取位于中间的完整周期，减少首尾截取点
     * 可能存在的滤波过渡或边界误差。
     */
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
    /* 保证数组和参数先写完，再让读取者看到新的活动索引。 */
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
