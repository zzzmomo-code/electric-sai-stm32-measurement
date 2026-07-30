/**
 * @file hmi_chart.h
 * @brief 淘晶驰双时域重叠控件常显和独立频谱控件的构帧接口。
 *
 * 模块用途：把已生成的 350 点缓存构建为 cle/add 指令，并生成一周期、
 *          三周期重叠控件的前景切换及独立频谱常显指令。
 * GPIO 引脚映射：无直接 GPIO；字节流由 hmi_task2 经 USART1 发送。
 * 依赖的外设和 CubeIDE 配置：页面包含 s_t1、s_t3、s_spec，均为单通道
 *          Waveform 控件且横向容纳 350 点；s_t1 与 s_t3 重叠，s_spec 独立放置。
 * 初始化方法：无状态，无需初始化。
 * 调用方法：hmi_task2 在后台刷新曲线或按键切换前景时调用。
 */

#ifndef HMI_CHART_H
#define HMI_CHART_H

#include <stdint.h>

#include "measurement_conversion.h"

#define HMI_CHART_T1_OBJECT       "s_t1"
#define HMI_CHART_T3_OBJECT       "s_t3"
#define HMI_CHART_SPECTRUM_OBJECT "s_spec"
#define HMI_CHART_FRAME_MAX_BYTES 9216u
#define HMI_CHART_POINTS_PER_CHUNK 32u
/**
 * 重叠控件切换策略：0=两个控件常显并尝试把选中控件置前；
 * 1=隐藏未选中控件作为可靠后备。实屏确认层级固定时只需改为 1。
 */
#define HMI_CHART_HIDE_BACKGROUND_FALLBACK 0u

/** 曲线数据模式；一周期和三周期同时装载，频谱位于独立区域。 */
typedef enum
{
    HMI_CHART_MODE_ONE_CYCLE = 1,
    HMI_CHART_MODE_THREE_CYCLE = 2,
    HMI_CHART_MODE_SPECTRUM = 3
} hmi_chart_mode_t;

/** HMI 构帧结果。 */
typedef enum
{
    HMI_CHART_STATUS_OK = 0,
    HMI_CHART_STATUS_INVALID_ARGUMENT,
    HMI_CHART_STATUS_BUFFER_TOO_SMALL
} hmi_chart_status_t;

/**
 * @brief 构建单个 Waveform 的 cle 加 350 条 add 指令。
 * @param object_name 控件名，不含 .id。
 * @param points 350 点 8 位纵轴数组。
 * @param point_count 必须为 350。
 * @param frame 输出 UART 字节流。
 * @param frame_capacity 输出容量。
 * @param frame_size 输出实际长度。
 * @return 构帧状态。
 */
hmi_chart_status_t hmi_chart_build_waveform(
    const char *object_name,
    const uint8_t *points,
    uint16_t point_count,
    uint8_t *frame,
    uint16_t frame_capacity,
    uint16_t *frame_size);

/**
 * @brief 分批构建曲线指令，避免一次突发 350 条 add 压垮屏幕命令解析器。
 * @param mode 目标曲线模式。
 * @param points 完整的 350 点纵轴数组。
 * @param first_point 本批第一点下标；为零时先生成 cle。
 * @param requested_points 本批最多发送的点数。
 * @param frame 输出 UART 字节流。
 * @param frame_capacity 输出容量。
 * @param frame_size 输出实际长度。
 * @param emitted_points 本批实际发送的点数。
 * @return 构帧状态。
 */
hmi_chart_status_t hmi_chart_build_waveform_chunk(
    hmi_chart_mode_t mode,
    const uint8_t *points,
    uint16_t first_point,
    uint16_t requested_points,
    uint8_t *frame,
    uint16_t frame_capacity,
    uint16_t *frame_size,
    uint16_t *emitted_points);

/**
 * @brief 构建双时域常显和指定时域控件前景切换指令。
 * @param mode 需要置于前景的时域模式，只允许一周期或三周期。
 * @param frame 输出 UART 字节流。
 * @param frame_capacity 输出容量。
 * @param frame_size 输出实际长度。
 * @return 构帧状态。
 */
hmi_chart_status_t hmi_chart_build_visibility(
    hmi_chart_mode_t mode,
    uint8_t *frame,
    uint16_t frame_capacity,
    uint16_t *frame_size);

#endif /* HMI_CHART_H */
