/**
 * @file measurement_calibration.c
 * @brief G 题打表拟合参数及校准实现。
 *
 * 模块用途：集中保存所有可修改的拟合系数，并为串口屏数字显示提供校准结果。
 * GPIO 引脚映射：无直接 GPIO 引脚。
 * 依赖的外设和 CubeIDE 配置：无直接外设依赖。
 * 初始化方法：system_init() 调用 measurement_calibration_init()。
 * 调用方法：HMI 构建参数文字前调用对应的 apply 接口。
 */

#include "system.h"

#include <limits.h>
#include <string.h>

/**
 * 打表后只需要修改下面这一段系数。
 *
 * 统一拟合公式：
 *     y = c0 + c1*x + c2*x*x + c3*x*x*x
 *
 * 电压类曲线的 x、y 单位都是 V；频率曲线的 x、y 单位都是 Hz。
 * 拟合时应把“FPGA 原始读数”作为 x，把“标准仪器参考值”作为 y。
 * 当前全部为 y=x：c0=0、c1=1、c2=0、c3=0。
 */
typedef struct
{
    double c0;
    double c1;
    double c2;
    double c3;
} measurement_calibration_curve_t;

/* 峰峰值 UPP 拟合系数，输入/输出单位 V。 */
static const measurement_calibration_curve_t calibration_vpp_curve =
{
    0.0, 1.0, 0.0, 0.0
};

/* 真有效值 URMS 拟合系数，输入/输出单位 V。 */
static const measurement_calibration_curve_t calibration_vrms_curve =
{
    0.0, 1.0, 0.0, 0.0
};

/* 基频及各频率分量共用的拟合系数，输入/输出单位 Hz。 */
static const measurement_calibration_curve_t calibration_frequency_curve =
{
    0.0, 1.0, 0.0, 0.0
};

/* 各频率分量峰值幅度的拟合系数，输入/输出单位 V。 */
static const measurement_calibration_curve_t
    calibration_component_amplitude_curve =
{
    0.0, 1.0, 0.0, 0.0
};

/* 直流偏置拟合系数，输入/输出单位 V；当前页面未显示，先保留接口。 */
static const measurement_calibration_curve_t calibration_dc_offset_curve =
{
    0.0, 1.0, 0.0, 0.0
};

/** 当前是否启用拟合；由主循环按键处理修改，调试器可直接观察。 */
volatile uint8_t measurement_calibration_enabled;
/** 校准层累计诊断量。 */
volatile measurement_calibration_diagnostics_t
    measurement_calibration_diagnostics;

/**
 * @brief 使用霍纳法计算三次多项式。
 * @param curve 拟合系数。
 * @param input 工程单位输入值。
 * @return 工程单位拟合输出值。
 */
static double measurement_calibration_evaluate(
    const measurement_calibration_curve_t *curve,
    double input)
{
    return (((curve->c3 * input + curve->c2) * input + curve->c1)
            * input) + curve->c0;
}

/**
 * @brief 把非负 double 结果四舍五入并安全限制到 uint32_t。
 * @param value 待转换值。
 * @return 饱和后的无符号整数。
 */
static uint32_t measurement_calibration_saturate_u32(double value)
{
    if ((value != value) || (value <= 0.0))
    {
        return 0u;
    }
    if (value >= (double)UINT32_MAX)
    {
        return UINT32_MAX;
    }
    return (uint32_t)(value + 0.5);
}

/**
 * @brief 把 double 结果四舍五入并安全限制到 int32_t。
 * @param value 待转换值。
 * @return 饱和后的有符号整数。
 */
static int32_t measurement_calibration_saturate_i32(double value)
{
    if (value != value)
    {
        return 0;
    }
    if (value >= (double)INT32_MAX)
    {
        return INT32_MAX;
    }
    if (value <= (double)INT32_MIN)
    {
        return INT32_MIN;
    }
    return (value >= 0.0)
        ? (int32_t)(value + 0.5) : (int32_t)(value - 0.5);
}

/**
 * @brief 对 uV 电压值使用指定曲线拟合。
 * @param curve 电压曲线，输入/输出单位 V。
 * @param raw_uv 原始电压，单位 uV。
 * @return 拟合电压，单位 uV。
 */
static uint32_t measurement_calibration_apply_voltage(
    const measurement_calibration_curve_t *curve,
    uint32_t raw_uv)
{
    double input_v;
    double output_v;

    if (measurement_calibration_enabled == 0u)
    {
        return raw_uv;
    }

    input_v = (double)raw_uv / 1000000.0;
    output_v = measurement_calibration_evaluate(curve, input_v);
    measurement_calibration_diagnostics.apply_count++;
    return measurement_calibration_saturate_u32(output_v * 1000000.0);
}

/**
 * @brief 初始化校准开关和诊断量。
 * @param 无。
 * @return 无。
 * @note 上电默认值由 MEASUREMENT_CALIBRATION_DEFAULT_ENABLED 决定。
 */
void measurement_calibration_init(void)
{
    memset((void *)&measurement_calibration_diagnostics, 0,
           sizeof(measurement_calibration_diagnostics));
    measurement_calibration_enabled =
        (MEASUREMENT_CALIBRATION_DEFAULT_ENABLED != 0u) ? 1u : 0u;
    measurement_calibration_diagnostics.enabled =
        measurement_calibration_enabled;
}

/**
 * @brief 显式设置是否启用拟合值。
 * @param enable 非零启用拟合，零直接返回原始值。
 * @return 无。
 */
void measurement_calibration_set_enabled(uint8_t enable)
{
    measurement_calibration_enabled = (enable != 0u) ? 1u : 0u;
    measurement_calibration_diagnostics.enabled =
        measurement_calibration_enabled;
}

/**
 * @brief 在已校准和未校准两种显示模式之间切换。
 * @param 无。
 * @return 切换后的状态：1=已校准，0=未校准。
 */
uint8_t measurement_calibration_toggle(void)
{
    measurement_calibration_set_enabled(
        (measurement_calibration_enabled == 0u) ? 1u : 0u);
    measurement_calibration_diagnostics.toggle_count++;
    return measurement_calibration_enabled;
}

/**
 * @brief 查询当前校准显示状态。
 * @param 无。
 * @return 1=已校准，0=未校准。
 */
uint8_t measurement_calibration_is_enabled(void)
{
    return measurement_calibration_enabled;
}

/**
 * @brief 对峰峰值应用当前校准策略。
 * @param raw_uv FPGA 原始峰峰值，单位 uV。
 * @return 拟合值或原始值，单位 uV。
 */
uint32_t measurement_calibration_apply_vpp_uv(uint32_t raw_uv)
{
    return measurement_calibration_apply_voltage(
        &calibration_vpp_curve, raw_uv);
}

/**
 * @brief 对真有效值应用当前校准策略。
 * @param raw_uv FPGA 原始真有效值，单位 uV。
 * @return 拟合值或原始值，单位 uV。
 */
uint32_t measurement_calibration_apply_vrms_uv(uint32_t raw_uv)
{
    return measurement_calibration_apply_voltage(
        &calibration_vrms_curve, raw_uv);
}

/**
 * @brief 对基频或分量频率应用当前校准策略。
 * @param raw_mhz FPGA 原始频率，单位 0.001 Hz。
 * @return 拟合值或原始值，单位 0.001 Hz。
 */
uint32_t measurement_calibration_apply_frequency_mhz(uint32_t raw_mhz)
{
    double input_hz;
    double output_hz;

    if (measurement_calibration_enabled == 0u)
    {
        return raw_mhz;
    }

    input_hz = (double)raw_mhz / 1000.0;
    output_hz = measurement_calibration_evaluate(
        &calibration_frequency_curve, input_hz);
    measurement_calibration_diagnostics.apply_count++;
    return measurement_calibration_saturate_u32(output_hz * 1000.0);
}

/**
 * @brief 对频率分量峰值幅度应用当前校准策略。
 * @param raw_uv FPGA 原始分量峰值幅度，单位 uV。
 * @return 拟合值或原始值，单位 uV。
 */
uint32_t measurement_calibration_apply_component_amplitude_uv(
    uint32_t raw_uv)
{
    return measurement_calibration_apply_voltage(
        &calibration_component_amplitude_curve, raw_uv);
}

/**
 * @brief 对直流偏置应用当前校准策略。
 * @param raw_uv FPGA 原始直流偏置，单位 uV。
 * @return 拟合值或原始值，单位 uV。
 */
int32_t measurement_calibration_apply_dc_offset_uv(int32_t raw_uv)
{
    double input_v;
    double output_v;

    if (measurement_calibration_enabled == 0u)
    {
        return raw_uv;
    }

    input_v = (double)raw_uv / 1000000.0;
    output_v = measurement_calibration_evaluate(
        &calibration_dc_offset_curve, input_v);
    measurement_calibration_diagnostics.apply_count++;
    return measurement_calibration_saturate_i32(output_v * 1000000.0);
}
