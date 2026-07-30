/**
 * @file measurement_calibration.h
 * @brief G 题测量结果拟合校准接口。
 *
 * 模块用途：保存打表拟合参数，并在“已校准/未校准”两种显示模式之间切换。
 * GPIO 引脚映射：无直接 GPIO 引脚。
 * 依赖的外设和 CubeIDE 配置：无直接外设依赖。
 * 初始化方法：system_init() 调用 measurement_calibration_init()。
 * 调用方法：串口屏文字构建前调用对应的 measurement_calibration_apply_*() 接口。
 */

#ifndef MEASUREMENT_CALIBRATION_H
#define MEASUREMENT_CALIBRATION_H

#include <stdint.h>

/** 上电默认使用拟合后的校准结果；改为 0 可默认显示 FPGA 原始结果。 */
#define MEASUREMENT_CALIBRATION_DEFAULT_ENABLED 1u

/** 校准层诊断量，可直接加入 STM32CubeIDE Expressions。 */
typedef struct
{
    uint32_t apply_count;  /**< 拟合函数实际参与换算的累计次数。 */
    uint32_t toggle_count; /**< 已校准/未校准模式切换次数。 */
    uint8_t enabled;       /**< 当前模式：1=已校准，0=未校准。 */
} measurement_calibration_diagnostics_t;

/** 当前校准开关，1 表示显示拟合结果，0 表示显示 FPGA 原始结果。 */
extern volatile uint8_t measurement_calibration_enabled;

/** 校准层公开诊断量。 */
extern volatile measurement_calibration_diagnostics_t
    measurement_calibration_diagnostics;

/**
 * @brief 初始化校准状态和诊断量。
 * @param 无。
 * @return 无。
 * @note 默认状态由 MEASUREMENT_CALIBRATION_DEFAULT_ENABLED 决定。
 */
void measurement_calibration_init(void);

/**
 * @brief 设置是否使用拟合后的校准结果。
 * @param enable 非零表示已校准，零表示未校准。
 * @return 无。
 */
void measurement_calibration_set_enabled(uint8_t enable);

/**
 * @brief 切换已校准/未校准状态。
 * @param 无。
 * @return 切换后的状态：1=已校准，0=未校准。
 */
uint8_t measurement_calibration_toggle(void);

/**
 * @brief 查询当前是否使用拟合后的校准结果。
 * @param 无。
 * @return 1=已校准，0=未校准。
 */
uint8_t measurement_calibration_is_enabled(void);

/**
 * @brief 对峰峰值原始码执行打表比例校准。
 * @param raw_uv FPGA 原始峰峰值码值。
 * @return 已校准模式返回 uV；未校准模式原样返回码值。
 */
uint32_t measurement_calibration_apply_vpp_uv(uint32_t raw_uv);

/**
 * @brief 对真有效值原始码执行打表比例校准。
 * @param raw_uv FPGA 原始真有效值码值。
 * @return 已校准模式返回 uV；未校准模式原样返回码值。
 */
uint32_t measurement_calibration_apply_vrms_uv(uint32_t raw_uv);

/**
 * @brief 对频率执行拟合，供基频和各频率分量共用。
 * @param raw_mhz FPGA 原始频率，单位 0.001 Hz。
 * @return 当前显示模式对应的频率，单位 0.001 Hz。
 */
uint32_t measurement_calibration_apply_frequency_mhz(uint32_t raw_mhz);

/**
 * @brief 对频率分量的峰值幅度原始码执行打表比例校准。
 * @param raw_uv FPGA 原始分量峰值幅度码值；它是正弦峰值而非峰峰值。
 * @return 已校准模式返回峰值幅度 uV；未校准模式原样返回码值。
 */
uint32_t measurement_calibration_apply_component_amplitude_uv(
    uint32_t raw_uv);

/**
 * @brief 对直流偏置执行拟合，预留给后续页面显示。
 * @param raw_uv FPGA 原始直流偏置，单位 uV。
 * @return 当前显示模式对应的直流偏置，单位 uV。
 */
int32_t measurement_calibration_apply_dc_offset_uv(int32_t raw_uv);

#endif /* MEASUREMENT_CALIBRATION_H */
