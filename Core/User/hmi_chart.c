/**
 * @file hmi_chart.c
 * @brief TJC 串口屏幅频和相频曲线构帧实现。
 *
 * 模块用途：把最多 1024 点 FPGA Bode 数据压缩为 256 点，构建 s0/s1
 *          Waveform 控件的 cle + add ASCII 指令。
 * GPIO 引脚映射：无直接 GPIO。
 * 依赖的外设和 CubeIDE 配置：HMI 当前页面必须存在 s0 和 s1，通道数 ch=1。
 * 初始化方法：无状态，无需初始化。
 * 调用方法：由 hmi_task2 在主循环构建最新一帧曲线。
 */

#include "system.h"

#include <stdio.h>

#define HMI_CHART_AMPLITUDE_OBJECT "s0.id"
#define HMI_CHART_PHASE_OBJECT     "s1.id"
#define HMI_CHART_TERMINATOR       0xffu

/**
 * 256 点降采样工作区。
 * 使用静态存储，避免 512 字节临时数组占用主循环栈；构帧函数不可重入。
 */
static uint8_t hmi_chart_amplitude_workspace[HMI_CHART_POINT_COUNT];
static uint8_t hmi_chart_phase_workspace[HMI_CHART_POINT_COUNT];

/**
 * @brief 计算 16 位无符号整数平方根的向下取整值。
 * @param value FPGA 发送的幅度平方高 16 位。
 * @return 0~255 的整数平方根。
 */
static uint16_t hmi_chart_isqrt_u16(uint16_t value)
{
    uint32_t operand = value;
    uint32_t result = 0u;
    uint32_t bit = 1uL << 14;

    while (bit > operand)
    {
        bit >>= 2;
    }

    while (bit != 0u)
    {
        if (operand >= (result + bit))
        {
            operand -= result + bit;
            result = (result >> 1) + bit;
        }
        else
        {
            result >>= 1;
        }
        bit >>= 2;
    }

    return (uint16_t)result;
}

/**
 * @brief 追加一条以 FF FF FF 结尾的格式化 TJC 指令。
 * @param frame 输出缓冲区。
 * @param capacity 缓冲区总容量。
 * @param used 当前已使用字节数，成功后更新。
 * @param format 只允许本模块内部固定格式字符串。
 * @param object_expression s0.id 或 s1.id。
 * @param channel Waveform 通道。
 * @param value add 数值；cle 调用时传入负数。
 * @return 成功返回 1，空间不足返回 0。
 */
static uint8_t hmi_chart_append_command(uint8_t *frame,
                                        uint16_t capacity,
                                        uint16_t *used,
                                        const char *format,
                                        const char *object_expression,
                                        uint8_t channel,
                                        int16_t value)
{
    uint16_t remaining;
    int length;

    if (*used >= capacity)
    {
        return 0u;
    }
    remaining = (uint16_t)(capacity - *used);

    if (value < 0)
    {
        length = snprintf((char *)&frame[*used], remaining,
                          format, object_expression,
                          (unsigned int)channel);
    }
    else
    {
        length = snprintf((char *)&frame[*used], remaining,
                          format, object_expression,
                          (unsigned int)channel,
                          (unsigned int)value);
    }

    if ((length < 0) || (((uint16_t)length + 3u) > remaining))
    {
        return 0u;
    }

    *used = (uint16_t)(*used + (uint16_t)length);
    frame[(*used)++] = HMI_CHART_TERMINATOR;
    frame[(*used)++] = HMI_CHART_TERMINATOR;
    frame[(*used)++] = HMI_CHART_TERMINATOR;
    return 1u;
}

/**
 * @brief 将原始 Bode 点按频率顺序压缩到 256 点。
 * @param bode 原始数据。
 * @param amplitude 输出幅频纵轴值。
 * @param phase 输出相频纵轴值。
 * @return 无。
 */
static void hmi_chart_downsample(const fpga_link_bode_t *bode,
                                 uint8_t *amplitude,
                                 uint8_t *phase)
{
    uint16_t output_index;

    for (output_index = 0u;
         output_index < HMI_CHART_POINT_COUNT;
         output_index++)
    {
        uint32_t begin =
            ((uint32_t)output_index * bode->point_count)
            / HMI_CHART_POINT_COUNT;
        uint32_t end =
            ((uint32_t)(output_index + 1u) * bode->point_count)
            / HMI_CHART_POINT_COUNT;
        uint32_t input_index;
        uint32_t sample_count;
        uint32_t magnitude_sum = 0u;
        int32_t phase_sum = 0;
        uint16_t mean_magnitude;
        int16_t mean_phase;
        uint16_t magnitude_root;
        int32_t phase_shifted;

        if (begin >= bode->point_count)
        {
            begin = (uint32_t)bode->point_count - 1u;
        }
        if (end <= begin)
        {
            end = begin + 1u;
        }
        if (end > bode->point_count)
        {
            end = bode->point_count;
        }

        sample_count = end - begin;
        for (input_index = begin; input_index < end; input_index++)
        {
            magnitude_sum += bode->mag2_hi[input_index];
            phase_sum += bode->phase[input_index];
        }
        mean_magnitude = (uint16_t)(magnitude_sum / sample_count);
        mean_phase = (int16_t)(phase_sum / (int32_t)sample_count);

        /* mag2_hi 是 (I²+Q²)[47:32]，均值开方后得到幅度响应。 */
        magnitude_root = hmi_chart_isqrt_u16(mean_magnitude);
        amplitude[output_index] = (uint8_t)(
            ((uint32_t)magnitude_root * HMI_CHART_VALUE_MAX) / 255u);

        /* 相位按 int16 有符号值求均值，再将 -32768~32767 映射到 0~255。 */
        phase_shifted = (int32_t)mean_phase + 32768;
        phase[output_index] = (uint8_t)(
            ((uint32_t)phase_shifted * HMI_CHART_VALUE_MAX) / 65535u);
    }
}

hmi_chart_status_t hmi_chart_build_bode_frame(
    const fpga_link_bode_t *bode,
    uint8_t *frame,
    uint16_t frame_capacity,
    uint16_t *frame_size)
{
    uint16_t index;

    if ((bode == NULL) || (frame == NULL) || (frame_size == NULL)
        || (bode->valid == 0u) || (bode->point_count == 0u)
        || (bode->point_count > FPGA_LINK_MAX_POINTS))
    {
        return HMI_CHART_STATUS_INVALID_ARGUMENT;
    }
    if (frame_capacity < HMI_CHART_FRAME_SIZE_TOTAL)
    {
        return HMI_CHART_STATUS_BUFFER_TOO_SMALL;
    }

    *frame_size = 0u;
    hmi_chart_downsample(
        bode, hmi_chart_amplitude_workspace, hmi_chart_phase_workspace);

    if (hmi_chart_append_command(
            frame, frame_capacity, frame_size,
            "cle %s,%u", HMI_CHART_AMPLITUDE_OBJECT, 0u, -1) == 0u)
    {
        return HMI_CHART_STATUS_BUFFER_TOO_SMALL;
    }
    for (index = 0u; index < HMI_CHART_POINT_COUNT; index++)
    {
        if (hmi_chart_append_command(
            frame, frame_capacity, frame_size,
            "add %s,%u,%u", HMI_CHART_AMPLITUDE_OBJECT,
            0u, (int16_t)hmi_chart_amplitude_workspace[index]) == 0u)
        {
            return HMI_CHART_STATUS_BUFFER_TOO_SMALL;
        }
    }

    if (hmi_chart_append_command(
            frame, frame_capacity, frame_size,
            "cle %s,%u", HMI_CHART_PHASE_OBJECT, 0u, -1) == 0u)
    {
        return HMI_CHART_STATUS_BUFFER_TOO_SMALL;
    }
    for (index = 0u; index < HMI_CHART_POINT_COUNT; index++)
    {
        if (hmi_chart_append_command(
            frame, frame_capacity, frame_size,
            "add %s,%u,%u", HMI_CHART_PHASE_OBJECT,
            0u, (int16_t)hmi_chart_phase_workspace[index]) == 0u)
        {
            return HMI_CHART_STATUS_BUFFER_TOO_SMALL;
        }
    }

    return HMI_CHART_STATUS_OK;
}
