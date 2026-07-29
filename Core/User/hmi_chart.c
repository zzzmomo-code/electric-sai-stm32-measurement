/**
 * @file hmi_chart.c
 * @brief 淘晶驰三个重叠 Waveform 控件构帧实现。
 *
 * 模块用途：生成 cle/add/vis ASCII 指令，每条命令自动追加 FF FF FF。
 * GPIO 引脚映射：无直接 GPIO。
 * 依赖的外设和 CubeIDE 配置：页面存在 s_t1、s_t3、s_spec。
 * 初始化方法：无需初始化。
 * 调用方法：由 hmi_task2 在主循环调用，函数不可在中断中调用。
 */

#include "system.h"

#include <stdio.h>
#include <string.h>

#define HMI_CHART_TERMINATOR 0xffu

/**
 * @brief 追加三个 FF 结束符。
 * @param frame 输出缓冲区。
 * @param capacity 总容量。
 * @param used 当前长度并在成功时更新。
 * @return 成功返回 1，否则返回 0。
 */
static uint8_t hmi_chart_append_terminator(
    uint8_t *frame,
    uint16_t capacity,
    uint16_t *used)
{
    if (((uint32_t)*used + 3u) > capacity)
    {
        return 0u;
    }

    frame[(*used)++] = HMI_CHART_TERMINATOR;
    frame[(*used)++] = HMI_CHART_TERMINATOR;
    frame[(*used)++] = HMI_CHART_TERMINATOR;
    return 1u;
}

/**
 * @brief 追加一条 cle 或 add 指令。
 * @param frame 输出缓冲区。
 * @param capacity 总容量。
 * @param used 当前长度。
 * @param object_name 控件名。
 * @param value 小于零时生成 cle，否则生成 add。
 * @return 成功返回 1，否则返回 0。
 */
static uint8_t hmi_chart_append_curve_command(
    uint8_t *frame,
    uint16_t capacity,
    uint16_t *used,
    const char *object_name,
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
        length = snprintf(
            (char *)&frame[*used],
            remaining,
            "cle %s.id,0",
            object_name);
    }
    else
    {
        length = snprintf(
            (char *)&frame[*used],
            remaining,
            "add %s.id,0,%u",
            object_name,
            (unsigned int)value);
    }

    if ((length < 0) || ((uint16_t)length >= remaining))
    {
        return 0u;
    }
    *used = (uint16_t)(*used + (uint16_t)length);
    return hmi_chart_append_terminator(
        frame, capacity, used);
}

/**
 * @brief 追加一条 vis 指令。
 * @param frame 输出缓冲区。
 * @param capacity 总容量。
 * @param used 当前长度。
 * @param object_name 控件名。
 * @param visible 非零显示，零隐藏。
 * @return 成功返回 1，否则返回 0。
 */
static uint8_t hmi_chart_append_visibility(
    uint8_t *frame,
    uint16_t capacity,
    uint16_t *used,
    const char *object_name,
    uint8_t visible)
{
    uint16_t remaining;
    int length;

    if (*used >= capacity)
    {
        return 0u;
    }
    remaining = (uint16_t)(capacity - *used);
    length = snprintf(
        (char *)&frame[*used],
        remaining,
        "vis %s,%u",
        object_name,
        (unsigned int)((visible != 0u) ? 1u : 0u));
    if ((length < 0) || ((uint16_t)length >= remaining))
    {
        return 0u;
    }

    *used = (uint16_t)(*used + (uint16_t)length);
    return hmi_chart_append_terminator(
        frame, capacity, used);
}

hmi_chart_status_t hmi_chart_build_waveform(
    const char *object_name,
    const uint8_t *points,
    uint16_t point_count,
    uint8_t *frame,
    uint16_t frame_capacity,
    uint16_t *frame_size)
{
    uint16_t index;

    if ((object_name == NULL) || (points == NULL)
        || (frame == NULL) || (frame_size == NULL)
        || (point_count != MEASUREMENT_DISPLAY_POINT_COUNT))
    {
        return HMI_CHART_STATUS_INVALID_ARGUMENT;
    }
    if (frame_capacity < HMI_CHART_FRAME_MAX_BYTES)
    {
        return HMI_CHART_STATUS_BUFFER_TOO_SMALL;
    }

    *frame_size = 0u;
    if (hmi_chart_append_curve_command(
            frame, frame_capacity, frame_size,
            object_name, -1) == 0u)
    {
        return HMI_CHART_STATUS_BUFFER_TOO_SMALL;
    }

    for (index = 0u; index < point_count; index++)
    {
        if (hmi_chart_append_curve_command(
                frame, frame_capacity, frame_size,
                object_name, (int16_t)points[index]) == 0u)
        {
            return HMI_CHART_STATUS_BUFFER_TOO_SMALL;
        }
    }

    return HMI_CHART_STATUS_OK;
}

hmi_chart_status_t hmi_chart_build_visibility(
    hmi_chart_mode_t mode,
    uint8_t *frame,
    uint16_t frame_capacity,
    uint16_t *frame_size)
{
    uint8_t one_visible;
    uint8_t three_visible;
    uint8_t spectrum_visible;

    if ((frame == NULL) || (frame_size == NULL)
        || ((mode != HMI_CHART_MODE_ONE_CYCLE)
            && (mode != HMI_CHART_MODE_THREE_CYCLE)
            && (mode != HMI_CHART_MODE_SPECTRUM)))
    {
        return HMI_CHART_STATUS_INVALID_ARGUMENT;
    }

    one_visible = (mode == HMI_CHART_MODE_ONE_CYCLE) ? 1u : 0u;
    three_visible = (mode == HMI_CHART_MODE_THREE_CYCLE) ? 1u : 0u;
    spectrum_visible = (mode == HMI_CHART_MODE_SPECTRUM) ? 1u : 0u;
    *frame_size = 0u;

    if ((hmi_chart_append_visibility(
            frame, frame_capacity, frame_size,
            HMI_CHART_T1_OBJECT, one_visible) == 0u)
        || (hmi_chart_append_visibility(
            frame, frame_capacity, frame_size,
            HMI_CHART_T3_OBJECT, three_visible) == 0u)
        || (hmi_chart_append_visibility(
            frame, frame_capacity, frame_size,
            HMI_CHART_SPECTRUM_OBJECT, spectrum_visible) == 0u))
    {
        return HMI_CHART_STATUS_BUFFER_TOO_SMALL;
    }

    return HMI_CHART_STATUS_OK;
}

hmi_chart_status_t hmi_chart_build_hide_all(
    uint8_t *frame,
    uint16_t frame_capacity,
    uint16_t *frame_size)
{
    if ((frame == NULL) || (frame_size == NULL))
    {
        return HMI_CHART_STATUS_INVALID_ARGUMENT;
    }

    *frame_size = 0u;
    if ((hmi_chart_append_visibility(
            frame, frame_capacity, frame_size,
            HMI_CHART_T1_OBJECT, 0u) == 0u)
        || (hmi_chart_append_visibility(
            frame, frame_capacity, frame_size,
            HMI_CHART_T3_OBJECT, 0u) == 0u)
        || (hmi_chart_append_visibility(
            frame, frame_capacity, frame_size,
            HMI_CHART_SPECTRUM_OBJECT, 0u) == 0u))
    {
        return HMI_CHART_STATUS_BUFFER_TOO_SMALL;
    }

    return HMI_CHART_STATUS_OK;
}
