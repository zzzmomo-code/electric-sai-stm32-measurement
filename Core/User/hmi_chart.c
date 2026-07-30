/**
 * @file hmi_chart.c
 * @brief 淘晶驰两个重叠时域控件常显和一个独立频谱控件的构帧实现。
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

/*
 * 淘晶驰普通指令的共同格式：
 *
 *   ASCII 命令 + 0xFF 0xFF 0xFF
 *
 * 本模块只负责“把一条命令正确编码到字节缓冲区”，不直接操作 UART。
 * 这样协议构帧可以在 PC 单元测试中验证，UART DMA 则集中由 hmi_task2 管理。
 */

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

/**
 * @brief 根据曲线模式返回对应的淘晶驰控件名。
 * @param mode 曲线模式。
 * @return 合法模式返回控件名，非法模式返回 NULL。
 */
static const char *hmi_chart_get_object_name(hmi_chart_mode_t mode)
{
    switch (mode)
    {
        case HMI_CHART_MODE_ONE_CYCLE:
            return HMI_CHART_T1_OBJECT;

        case HMI_CHART_MODE_THREE_CYCLE:
            return HMI_CHART_T3_OBJECT;

        case HMI_CHART_MODE_SPECTRUM:
            return HMI_CHART_SPECTRUM_OBJECT;

        default:
            return NULL;
    }
}

/**
 * @brief 在曲线分批发送的首尾再次确认目标控件可见。
 * @param mode 目标曲线模式。
 * @param frame 输出缓冲区。
 * @param capacity 总容量。
 * @param used 当前长度。
 * @return 成功返回 1，否则返回 0。
 *
 * @brief 构建一整条 350 点曲线的清空和追加命令。
 * @param object_name 淘晶驰 Waveform 控件名。
 * @param points 已映射到 8~201 的 350 个纵坐标。
 * @param point_count 点数，必须等于 MEASUREMENT_DISPLAY_POINT_COUNT。
 * @param frame 输出命令字节流。
 * @param frame_capacity 输出缓冲区容量。
 * @param frame_size 输出实际字节数。
 * @return 构帧状态；本函数不启动 UART 发送。
 *
 * @note 先发送 cle，再为每个点发送 add。350 个点对应控件的 350 个横向位置，
 *       因此不会出现只占横轴左侧一部分的问题。
 */
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

/**
 * @brief 把一条 350 点曲线拆成小批量 cle/add 指令。
 * @param mode 目标曲线模式。
 * @param points 完整的纵轴数组。
 * @param first_point 本批第一点下标。
 * @param requested_points 本批最多发送点数。
 * @param frame 输出命令字节流。
 * @param frame_capacity 输出缓冲区容量。
 * @param frame_size 输出实际字节数。
 * @param emitted_points 本批实际发送点数。
 * @return 构帧状态；本函数不直接启动 UART。
 *
 * @note 分批绘图期间不发送 vis，避免两个完全重叠的时域控件在每个小批次之间
 *       反复争抢前景。三条曲线完成后由 hmi_task2 单独恢复用户选择的前景。
 */
hmi_chart_status_t hmi_chart_build_waveform_chunk(
    hmi_chart_mode_t mode,
    const uint8_t *points,
    uint16_t first_point,
    uint16_t requested_points,
    uint8_t *frame,
    uint16_t frame_capacity,
    uint16_t *frame_size,
    uint16_t *emitted_points)
{
    const char *object_name = hmi_chart_get_object_name(mode);
    uint16_t remaining_points;
    uint16_t batch_points;
    uint16_t index;

    if ((object_name == NULL) || (points == NULL)
        || (frame == NULL) || (frame_size == NULL)
        || (emitted_points == NULL)
        || (first_point >= MEASUREMENT_DISPLAY_POINT_COUNT)
        || (requested_points == 0u))
    {
        return HMI_CHART_STATUS_INVALID_ARGUMENT;
    }

    *frame_size = 0u;
    *emitted_points = 0u;
    remaining_points = (uint16_t)(
        MEASUREMENT_DISPLAY_POINT_COUNT - first_point);
    batch_points = (requested_points < remaining_points)
        ? requested_points : remaining_points;

    if (first_point == 0u)
    {
        if (hmi_chart_append_curve_command(
                frame, frame_capacity, frame_size,
                object_name, -1) == 0u)
        {
            return HMI_CHART_STATUS_BUFFER_TOO_SMALL;
        }
    }

    for (index = 0u; index < batch_points; index++)
    {
        if (hmi_chart_append_curve_command(
                frame, frame_capacity, frame_size,
                object_name,
                (int16_t)points[first_point + index]) == 0u)
        {
            return HMI_CHART_STATUS_BUFFER_TOO_SMALL;
        }
    }
    *emitted_points = batch_points;

    return HMI_CHART_STATUS_OK;
}

/**
 * @brief 构建双时域常显与指定时域控件前景切换命令。
 * @param mode 需要置于前景的一周期或三周期模式。
 * @param frame 输出命令字节流。
 * @param frame_capacity 输出缓冲区容量。
 * @param frame_size 输出实际字节数。
 * @return 构帧状态。
 *
 * @note 默认先保持非选中控件可见，再把选中控件快速隐藏/显示一次，尝试利用
 *       淘晶驰重绘顺序把它置于前景；两份曲线数据都不会被清空。该层级行为仍需
 *       实屏验证。若启用后备开关，则直接隐藏非选中控件。
 */
hmi_chart_status_t hmi_chart_build_visibility(
    hmi_chart_mode_t mode,
    uint8_t *frame,
    uint16_t frame_capacity,
    uint16_t *frame_size)
{
    const char *background_object;
    const char *foreground_object;

    if ((frame == NULL) || (frame_size == NULL)
        || ((mode != HMI_CHART_MODE_ONE_CYCLE)
            && (mode != HMI_CHART_MODE_THREE_CYCLE)))
    {
        return HMI_CHART_STATUS_INVALID_ARGUMENT;
    }

    background_object = (mode == HMI_CHART_MODE_ONE_CYCLE)
        ? HMI_CHART_T3_OBJECT : HMI_CHART_T1_OBJECT;
    foreground_object = (mode == HMI_CHART_MODE_ONE_CYCLE)
        ? HMI_CHART_T1_OBJECT : HMI_CHART_T3_OBJECT;
    *frame_size = 0u;

    if ((hmi_chart_append_visibility(
            frame, frame_capacity, frame_size,
            HMI_CHART_SPECTRUM_OBJECT, 1u) == 0u)
        || (hmi_chart_append_visibility(
            frame, frame_capacity, frame_size,
            background_object,
#if (HMI_CHART_HIDE_BACKGROUND_FALLBACK != 0u)
            0u) == 0u)
#else
            1u) == 0u)
#endif
#if (HMI_CHART_HIDE_BACKGROUND_FALLBACK == 0u)
        || (hmi_chart_append_visibility(
            frame, frame_capacity, frame_size,
            foreground_object, 0u) == 0u)
#endif
        || (hmi_chart_append_visibility(
            frame, frame_capacity, frame_size,
            foreground_object, 1u) == 0u))
    {
        return HMI_CHART_STATUS_BUFFER_TOO_SMALL;
    }

    return HMI_CHART_STATUS_OK;
}
