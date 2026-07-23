/**
 * @file measurement_input.h
 * @brief 内部双 ADC 与 ADS8688 动态采集源管理公共接口。
 *
 * 模块用途：统一管理采集源生命周期、FFT 运行时采样参数、ADS8688 单双通道及量程。
 * GPIO 引脚映射：内部 ADC 使用 PC4/ADC1_INP4、PB1/ADC2_INP5；ADS8688 使用
 * PA15/SPI3_NSS、PC10/SPI3_SCK、PC11/SPI3_MISO、PC12/SPI3_MOSI、
 * PD0/DAISY 和 PD1/RST。
 * 依赖的外设和 CubeIDE 配置：ADC1/ADC2、TIM2、SPI3 32 位主机硬件 NSS、
 * DMA1 Stream1 RX Circular Word/递增及 Stream2 TX Circular Word/不递增。
 * 初始化方法：system_init() 调用 measurement_input_init()，默认启动内部双 ADC。
 * 调用方法：主循环调用 measurement_input_process()；配置界面调用选择和 ADS 设置接口。
 */

#ifndef MEASUREMENT_INPUT_H
#define MEASUREMENT_INPUT_H

#include <stdint.h>

#include "ads8688.h"

/** 可选测量采集源。 */
typedef enum
{
    MEASUREMENT_INPUT_SOURCE_INTERNAL_ADC = 0,
    MEASUREMENT_INPUT_SOURCE_ADS8688
} measurement_input_source_t;

/** 采集源管理操作结果。 */
typedef enum
{
    MEASUREMENT_INPUT_STATUS_OK = 0,
    MEASUREMENT_INPUT_STATUS_INVALID_ARGUMENT,
    MEASUREMENT_INPUT_STATUS_DRIVER_ERROR,
    MEASUREMENT_INPUT_STATUS_ROLLBACK_ERROR
} measurement_input_status_t;

/** 采集源运行诊断快照。 */
typedef struct
{
    measurement_input_source_t active_source; /**< 当前正在运行的采集源。 */
    uint32_t switch_count;                    /**< 成功切换采集源的次数。 */
    uint32_t switch_failure_count;            /**< 切换或配置失败次数。 */
    float sample_rate_hz;                     /**< 当前每通道 FFT 采样率。 */
    uint8_t ads8688_single_channel;           /**< 非零表示 ADS 为单通道模式。 */
    uint8_t ads8688_channel;                  /**< ADS 单通道模式所选物理通道。 */
    int32_t last_driver_status;                /**< 最近一次底层驱动返回值。 */
} measurement_input_diagnostics_t;

/**
 * @brief 初始化两个采集驱动并默认启动内部双 ADC。
 * @param 无。
 * @return 初始化结果。
 * @note ADS8688 只完成器件配置，不在默认路径启动 SPI3 DMA。
 */
measurement_input_status_t measurement_input_init(void);

/**
 * @brief 动态选择内部双 ADC 或 ADS8688。
 * @param source 目标采集源。
 * @return 切换结果。
 * @note 先停止旧源、重配 FFT，再启动新源；失败时尝试恢复旧源。
 */
measurement_input_status_t measurement_input_select(
    measurement_input_source_t source);

/**
 * @brief 设置 ADS8688 单通道采样。
 * @param channel ADS8688 物理通道，范围为 0 至 7。
 * @return 配置结果。
 * @note 若 ADS 当前活动，将同步重配 FFT 并恢复采样。
 */
measurement_input_status_t measurement_input_set_ads8688_single_channel(
    uint8_t channel);

/**
 * @brief 设置 ADS8688 AIN0/AIN1 双通道采样。
 * @param 无。
 * @return 配置结果。
 * @note 双通道相位会按相邻轮询帧间隔进行软件补偿。
 */
measurement_input_status_t measurement_input_set_ads8688_dual_channel(void);

/**
 * @brief 设置 ADS8688 指定通道量程并同步 FFT 电压换算。
 * @param channel ADS8688 物理通道，范围为 0 至 7。
 * @param range 目标 ADS8688 硬件量程。
 * @return 配置结果。
 */
measurement_input_status_t measurement_input_set_ads8688_channel_range(
    uint8_t channel,
    ads8688_range_t range);

/**
 * @brief 执行当前采集源和 FFT 主循环处理。
 * @param 无。
 * @return 无。
 */
void measurement_input_process(void);

/**
 * @brief 获取采集源管理诊断快照。
 * @param diagnostics 接收诊断数据的指针。
 * @return 指针有效返回 1，否则返回 0。
 */
uint8_t measurement_input_get_diagnostics(
    measurement_input_diagnostics_t *diagnostics);

#endif /* MEASUREMENT_INPUT_H */
