/**
 * @file dac_output.c
 * @brief DAC 六档直流输出与外部 VGA 增益计算模块实现。
 *
 * 模块用途：将六档标称电压校准并换算为 DAC1 12 位码值，同时提供 VG 和 VGA 增益计算。
 * GPIO 引脚映射：PC4/OPAMP1_VOUT；DAC1_OUT1 通过片内模拟连接送入 OPAMP1。
 * 依赖的外设和 CubeIDE 配置：依赖 DAC1 Channel 1、OPAMP1 Follower 模式及 PC4 模拟输出。
 * 初始化方法：由 system_init() 调用 dac_output_init()。
 * 调用方法：业务代码调用 dac_output_set_level()，查询函数不改变硬件输出。
 */

#include "system.h"

/** 六档标称电压查找表，单位为 mV。 */
static const uint16_t dac_output_level_mv[DAC_OUTPUT_LEVEL_COUNT] =
{
    DAC_OUTPUT_LEVEL_0_MV,
    DAC_OUTPUT_LEVEL_1_MV,
    DAC_OUTPUT_LEVEL_2_MV,
    DAC_OUTPUT_LEVEL_3_MV,
    DAC_OUTPUT_LEVEL_4_MV,
    DAC_OUTPUT_LEVEL_5_MV
};

/** 最近一次成功写入 DAC 的档位，仅在主循环上下文访问。 */
static uint8_t dac_output_current_level = 0u;

/**
 * @brief 计算指定档位经校准和限幅后的理论电压。
 * @param level 已确认有效的输出档位。
 * @return 校准并限制后的理论电压，单位为 V。
 * @note 本函数不检查档位范围，也不访问硬件。
 */
static float dac_output_calculate_voltage(uint8_t level)
{
    const float reference_v = (float)DAC_OUTPUT_REFERENCE_MV / 1000.0f;
    float voltage_v = ((float)dac_output_level_mv[level] / 1000.0f)
                      * DAC_OUTPUT_GAIN_CALIBRATION
                      + DAC_OUTPUT_OFFSET_CALIBRATION_V;

    if (voltage_v < 0.0f)
    {
        voltage_v = 0.0f;
    }
    else if (voltage_v > reference_v)
    {
        voltage_v = reference_v;
    }

    return voltage_v;
}

/**
 * @brief 将已限幅的目标电压换算为 12 位 DAC 码值。
 * @param voltage_v 目标电压，单位为 V。
 * @return 右对齐的 12 位 DAC 码值，范围为 0 至 4095。
 * @note 换算使用四舍五入，并进行防御性上下限保护。
 */
static uint32_t dac_output_voltage_to_code(float voltage_v)
{
    const float reference_v = (float)DAC_OUTPUT_REFERENCE_MV / 1000.0f;
    float code = voltage_v * 4095.0f / reference_v + 0.5f;

    if (code < 0.0f)
    {
        code = 0.0f;
    }
    else if (code > 4095.0f)
    {
        code = 4095.0f;
    }

    return (uint32_t)code;
}

dac_output_status_t dac_output_init(void)
{
    if (HAL_OPAMP_Start(&hopamp1) != HAL_OK)
    {
        return dac_output_status_hal_error;
    }

    if (dac_output_set_level(0u) != dac_output_status_ok)
    {
        return dac_output_status_hal_error;
    }

    if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK)
    {
        return dac_output_status_hal_error;
    }

    return dac_output_status_ok;
}

dac_output_status_t dac_output_set_level(uint8_t level)
{
    uint32_t code;

    if (level >= DAC_OUTPUT_LEVEL_COUNT)
    {
        return dac_output_status_invalid_argument;
    }

    code = dac_output_voltage_to_code(dac_output_calculate_voltage(level));
    if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, code) != HAL_OK)
    {
        return dac_output_status_hal_error;
    }

    dac_output_current_level = level;
    return dac_output_status_ok;
}

uint8_t dac_output_get_level(void)
{
    return dac_output_current_level;
}

dac_output_status_t dac_output_get_voltage(uint8_t level, float *voltage_v)
{
    if ((level >= DAC_OUTPUT_LEVEL_COUNT) || (voltage_v == NULL))
    {
        return dac_output_status_invalid_argument;
    }

    *voltage_v = dac_output_calculate_voltage(level);
    return dac_output_status_ok;
}

dac_output_status_t dac_output_get_vg(uint8_t level, float *vg)
{
    float voltage_v;

    if (vg == NULL)
    {
        return dac_output_status_invalid_argument;
    }

    if (dac_output_get_voltage(level, &voltage_v) != dac_output_status_ok)
    {
        return dac_output_status_invalid_argument;
    }

    *vg = (20.0f / 33.0f) * voltage_v - 1.0f;
    return dac_output_status_ok;
}

dac_output_status_t dac_output_get_vga_gain(uint8_t level, float *gain)
{
    float vg;

    if ((gain == NULL) || (DAC_OUTPUT_RG_OHM <= 0.0f))
    {
        return dac_output_status_invalid_argument;
    }

    if (dac_output_get_vg(level, &vg) != dac_output_status_ok)
    {
        return dac_output_status_invalid_argument;
    }

    *gain = (1.0f + vg) * DAC_OUTPUT_RF_OHM / DAC_OUTPUT_RG_OHM;
    return dac_output_status_ok;
}
