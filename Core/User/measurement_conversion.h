/**
 * @file measurement_conversion.h
 * @brief FPGA 测量快照到串口屏显示缓存的转换接口。
 *
 * 模块用途：把 FPGA 三周期时域和 1312 点频谱转换成三个 350 点显示缓存，
 *          同时保存 Vpp、Vrms、基频和最多三个分量参数。
 * GPIO 引脚映射：无直接 GPIO 引脚。
 * 依赖的外设和 CubeIDE 配置：无直接外设依赖。
 * 初始化方法：system_init() 调用 measurement_conversion_init()。
 * 调用方法：收到新 FPGA 快照后调用 measurement_conversion_update()。
 */

#ifndef MEASUREMENT_CONVERSION_H
#define MEASUREMENT_CONVERSION_H

#include <stdint.h>

#include "fpga_link.h"

/** 每个 Waveform 恰好发送 350 个横向显示点，与页面控件宽度一致。 */
#define MEASUREMENT_DISPLAY_POINT_COUNT 350u
/** 纵轴最低显示值，保留 8 个单位下边距。 */
#define MEASUREMENT_DISPLAY_Y_MIN       8u
/** 210 像素高控件的纵轴最高显示值，保留约 8 个像素上边距。 */
#define MEASUREMENT_DISPLAY_Y_MAX       201u
/** FPGA 时域数据的固定显示量程：-15000~+15000。 */
#define MEASUREMENT_TIME_DISPLAY_LIMIT  15000

/** 已转换的完整显示快照。 */
typedef struct
{
    uint8_t waveform_1cycle[MEASUREMENT_DISPLAY_POINT_COUNT]; /**< 一周期纵轴点。 */
    uint8_t waveform_3cycle[MEASUREMENT_DISPLAY_POINT_COUNT]; /**< 三周期纵轴点。 */
    uint8_t spectrum_display[MEASUREMENT_DISPLAY_POINT_COUNT]; /**< 频谱纵轴点。 */
    uint32_t frame_sequence; /**< 三组数组共同对应的 FPGA 帧序号。 */
    uint64_t timestamp_50m; /**< FPGA 50 MHz 时钟时间戳。 */
    uint32_t source_flags; /**< FPGA 完整测量帧 flags。 */
    uint32_t vpp_uv; /**< 峰峰值，单位 µV。 */
    uint32_t vrms_uv; /**< 真有效值，单位 µV。 */
    uint32_t fundamental_mhz; /**< 基频，单位 0.001 Hz。 */
    int32_t dc_offset_uv; /**< 直流偏置，单位 µV。 */
    fpga_protocol_component_t component[FPGA_PROTOCOL_COMPONENT_MAX]; /**< 已按 VALID 压紧的分量参数。 */
    uint32_t dropped_frames; /**< FPGA 累计丢帧计数。 */
    uint16_t calibration_revision; /**< FPGA 校准版本。 */
    uint8_t component_count; /**< 有效分量个数，范围 0~3。 */
    uint8_t valid; /**< 非零表示整份显示快照已完整发布。 */
} measurement_display_snapshot_t;

/** 显示转换诊断量。 */
typedef struct
{
    uint32_t conversion_count; /**< 成功生成并发布显示快照的次数。 */
    uint32_t invalid_source_count; /**< 输入快照字段不合法的次数。 */
    uint32_t last_frame_sequence; /**< 最近一次成功转换的 FPGA 帧序号。 */
    int16_t last_time_min; /**< 最近时域帧的最小原始码。 */
    int16_t last_time_max; /**< 最近时域帧的最大原始码。 */
    uint16_t last_spectrum_max; /**< 最近 1312 点频谱的最大值。 */
    uint16_t last_one_cycle_samples; /**< 最近一次截取的一周期原始点数。 */
    uint16_t last_time_rail_sample_count; /**< 最近一帧接近正负满量程的样点数。 */
    uint16_t last_time_display_clip_count; /**< 最近一帧超出±15000显示量程的样点数。 */
    uint8_t last_time_offset_binary; /**< 1=按偏移二进制解码，0=按二补码解码。 */
    uint32_t offset_binary_frame_count; /**< 自动修正偏移二进制的累计帧数。 */
} measurement_conversion_diagnostics_t;

/** 转换层公开诊断量，可加入 CubeIDE Expressions。 */
extern volatile measurement_conversion_diagnostics_t
    measurement_conversion_diagnostics;

/**
 * @brief 清零两份显示快照和转换诊断量。
 * @param 无。
 * @return 无。
 */
void measurement_conversion_init(void);

/**
 * @brief 将完整 FPGA 快照转换成三组 350 点显示数据。
 * @param source 已通过协议和 CRC 校验的测量快照。
 * @return 成功发布显示快照返回 1，输入无效返回 0。
 */
uint8_t measurement_conversion_update(
    const fpga_measurement_snapshot_t *source);

/**
 * @brief 获取当前活动显示快照。
 * @param snapshot 输出只读指针。
 * @return 已有有效显示快照返回 1，否则返回 0。
 */
uint8_t measurement_conversion_get_snapshot(
    const measurement_display_snapshot_t **snapshot);

/** 旧页面兼容频率换算接口；新 G 题正式主流程不调用。 */
float measurement_conversion_frequency_hz(float timer_frequency_hz,
                                           float dds_frequency_hz,
                                           float adc_frequency_hz);
/** 旧页面兼容幅度换算接口；新 G 题正式主流程不调用。 */
float measurement_conversion_amplitude_vpp(float adc_amplitude_vpp,
                                            uint8_t vga_level);

#endif /* MEASUREMENT_CONVERSION_H */
