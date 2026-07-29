/**
 * @file measurement_conversion.h
 * @brief FPGA 测量快照到串口屏显示缓存的转换接口。
 *
 * 模块用途：把 FPGA 三周期时域和 1312 点频谱转换成三个 700 点显示缓存，
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

#define MEASUREMENT_DISPLAY_POINT_COUNT 700u
#define MEASUREMENT_DISPLAY_Y_MIN       8u
#define MEASUREMENT_DISPLAY_Y_MAX       247u

/** 已转换的完整显示快照。 */
typedef struct
{
    uint8_t waveform_1cycle[MEASUREMENT_DISPLAY_POINT_COUNT];
    uint8_t waveform_3cycle[MEASUREMENT_DISPLAY_POINT_COUNT];
    uint8_t spectrum_display[MEASUREMENT_DISPLAY_POINT_COUNT];
    uint32_t frame_sequence;
    uint64_t timestamp_50m;
    uint32_t source_flags;
    uint32_t vpp_uv;
    uint32_t vrms_uv;
    uint32_t fundamental_mhz;
    int32_t dc_offset_uv;
    fpga_protocol_component_t component[FPGA_PROTOCOL_COMPONENT_MAX];
    uint32_t dropped_frames;
    uint16_t calibration_revision;
    uint8_t component_count;
    uint8_t valid;
} measurement_display_snapshot_t;

/** 显示转换诊断量。 */
typedef struct
{
    uint32_t conversion_count;
    uint32_t invalid_source_count;
    uint32_t last_frame_sequence;
    int16_t last_time_min;
    int16_t last_time_max;
    uint16_t last_spectrum_max;
    uint16_t last_one_cycle_samples;
} measurement_conversion_diagnostics_t;

extern volatile measurement_conversion_diagnostics_t
    measurement_conversion_diagnostics;

void measurement_conversion_init(void);

/**
 * @brief 将完整 FPGA 快照转换成三组 700 点显示数据。
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

/*
 * 旧页面仍参与编译时使用的兼容接口；新 G 题主流程不调用。
 */
float measurement_conversion_frequency_hz(float timer_frequency_hz,
                                           float dds_frequency_hz,
                                           float adc_frequency_hz);
float measurement_conversion_amplitude_vpp(float adc_amplitude_vpp,
                                            uint8_t vga_level);

#endif /* MEASUREMENT_CONVERSION_H */
