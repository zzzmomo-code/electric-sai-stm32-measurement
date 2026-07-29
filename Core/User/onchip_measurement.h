/**
 * @file onchip_measurement.h
 * @brief STM32H743 片上 ADC 单通道测量链路公共接口。
 *
 * 模块用途：管理 PA6/ADC1_INP3 的 3.2 MSPS 单次 DMA 采集，执行 FFT、谐波约束、
 *          联合 I/Q 最小二乘、干扰剔除以及 Vpp/RMS 重建，并发布兼容显示层的快照。
 * GPIO 引脚映射：PA6/ADC1_INP3 为唯一信号输入；PA9/PA10 串口屏由 hmi_task2 管理。
 * 依赖的外设和 CubeIDE 配置：ADC1 12 位、TIM2 TRGO 3.2 MHz、
 *          DMA1 Stream0 Normal Halfword/Halfword。
 * 初始化方法：system_init() 调用 onchip_measurement_init()。
 * 调用方法：system_process() 持续调用 onchip_measurement_process()。
 */

#ifndef ONCHIP_MEASUREMENT_H
#define ONCHIP_MEASUREMENT_H

#include <stdint.h>

#include "fpga_link.h"

/** 片上 ADC 标称采样率。 */
#define ONCHIP_MEASUREMENT_SAMPLE_RATE_HZ 3200000u
/** 一帧 FFT/拟合样本数。 */
#define ONCHIP_MEASUREMENT_SAMPLE_COUNT 8192u

/** 采集和处理状态。 */
typedef enum
{
    ONCHIP_MEASUREMENT_STATE_UNINITIALIZED = 0,
    ONCHIP_MEASUREMENT_STATE_CAPTURING,
    ONCHIP_MEASUREMENT_STATE_PROCESSING,
    ONCHIP_MEASUREMENT_STATE_WAITING,
    ONCHIP_MEASUREMENT_STATE_ERROR
} onchip_measurement_state_t;

/** 可在 STM32CubeIDE Expressions 中观察的诊断量。 */
typedef struct
{
    uint32_t capture_start_count; /**< 成功启动 DMA 采集的次数。 */
    uint32_t capture_complete_count; /**< DMA 完整帧完成次数。 */
    uint32_t capture_timeout_count; /**< 超过 20 ms 未完成的次数。 */
    uint32_t capture_error_count; /**< ADC、DMA 或 TIM2 错误次数。 */
    uint32_t analysis_success_count; /**< 成功发布测量快照的次数。 */
    uint32_t analysis_error_count; /**< FFT、寻峰或拟合失败次数。 */
    uint32_t clipped_frame_count; /**< ADC 靠近 0 或 4095 的帧数。 */
    uint32_t last_analysis_time_ms; /**< 最近一次算法处理耗时。 */
    uint32_t last_total_time_ms; /**< 最近一次从启动采集到发布结果的耗时。 */
    uint32_t last_fundamental_mhz; /**< 最近基频，单位 0.001 Hz。 */
    uint32_t last_vpp_uv; /**< 最近去干扰后峰峰值，单位 uV。 */
    uint32_t last_vrms_uv; /**< 最近去干扰后真有效值，单位 uV。 */
    uint32_t last_interference_mhz; /**< 检出的高频单音，单位 0.001 Hz。 */
    uint16_t last_adc_min; /**< 最近一帧 ADC 最小码。 */
    uint16_t last_adc_max; /**< 最近一帧 ADC 最大码。 */
    uint8_t last_component_count; /**< 最近有效分量数。 */
    uint8_t interference_removed; /**< 最近拟合是否包含高频干扰项。 */
    int32_t last_hal_status; /**< 最近一次关键 HAL 返回值。 */
    onchip_measurement_state_t state; /**< 当前状态。 */
} onchip_measurement_diagnostics_t;

/** 片上 ADC 测量诊断量。 */
extern volatile onchip_measurement_diagnostics_t
    onchip_measurement_diagnostics;

/**
 * @brief 校准 ADC1、清零状态并启动第一帧采集。
 * @param 无。
 * @return 初始化成功返回 1，HAL 校准或启动失败返回 0。
 */
uint8_t onchip_measurement_init(void);

/**
 * @brief 推进 DMA 采集、FFT 分析、快照发布和下一帧调度。
 * @param 无。
 * @return 无。
 * @note 中断仅由已有 ADC HAL 回调置标志，所有计算均在本函数所在主循环完成。
 */
void onchip_measurement_process(void);

/**
 * @brief 获取最近一次完整片上 ADC 测量快照。
 * @param snapshot 输出只读快照指针。
 * @return 已有有效快照返回 1，否则返回 0。
 */
uint8_t onchip_measurement_get_snapshot(
    const fpga_measurement_snapshot_t **snapshot);

#endif /* ONCHIP_MEASUREMENT_H */
