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
 * FPGA 新协议中的 vpp_uv、vrms_uv 和 component.amplitude_peak_uv
 * 已经是物理微伏值，不再是 ADC 或 FFT 原始码。
 *
 * 2026-07-30 实板复核确认旧打表参考面存在约 2 倍阻抗匹配误差，因此已校准
 * 模式仍按用户要求把三类电压除以 2；这里把它明确描述成“uV 到 uV 的前端
 * 逆增益”，不能再使用旧的 raw/mV 或 /12800 解释。
 *
 * 若 FPGA 已经补偿同一个 2 倍增益，应把下面三个增益统一改为 1.0，避免重复补偿。
 */
#define MEASUREMENT_FRONTEND_VPP_GAIN        2.0
#define MEASUREMENT_FRONTEND_VRMS_GAIN       2.0
#define MEASUREMENT_FRONTEND_COMPONENT_GAIN  2.0

/**
 * 频率和直流偏置继续保留通用多项式接口。
 *
 * 统一拟合公式：
 *     y = c0 + c1*x + c2*x*x + c3*x*x*x
 *
 * 当前频率和直流偏置均为 y=x；频率字段保持 mHz 约定，
 * 即串口屏格式化后仍满足 f_Hz = raw_freq / 1000。
 */
typedef struct
{
    double c0;
    double c1;
    double c2;
    double c3;
} measurement_calibration_curve_t;

/* 基频及各频率分量共用的拟合系数，输入/输出单位 Hz。 */
static const measurement_calibration_curve_t calibration_frequency_curve =
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
 * @brief 对 FPGA 已换算的微伏值应用模拟前端逆增益。
 * @param fpga_uv FPGA 发送的物理电压，单位 uV。
 * @param frontend_gain 从信号输入参考面到 FPGA 测量参考面的电压增益。
 * @return 校准后的输入端电压，单位 uV；未校准模式原样返回 FPGA 微伏值。
 */
static uint32_t measurement_calibration_apply_inverse_frontend_gain(
    uint32_t fpga_uv,
    double frontend_gain)
{
    double output_uv;

    if (measurement_calibration_enabled == 0u)
    {
        return fpga_uv;
    }
    if (frontend_gain <= 0.0)
    {
        return fpga_uv;
    }

    output_uv = (double)fpga_uv / frontend_gain;
    measurement_calibration_diagnostics.apply_count++;
    return measurement_calibration_saturate_u32(output_uv);
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
 * @param fpga_uv FPGA 已换算的峰峰值，单位 uV。
 * @return 已校准模式返回输入端峰峰值；未校准模式原样返回 FPGA 微伏值。
 */
uint32_t measurement_calibration_apply_vpp_uv(uint32_t fpga_uv)
{
    return measurement_calibration_apply_inverse_frontend_gain(
        fpga_uv, MEASUREMENT_FRONTEND_VPP_GAIN);
}

/**
 * @brief 对真有效值应用当前校准策略。
 * @param fpga_uv FPGA 已换算的真有效值，单位 uV。
 * @return 已校准模式返回输入端有效值；未校准模式原样返回 FPGA 微伏值。
 */
uint32_t measurement_calibration_apply_vrms_uv(uint32_t fpga_uv)
{
    return measurement_calibration_apply_inverse_frontend_gain(
        fpga_uv, MEASUREMENT_FRONTEND_VRMS_GAIN);
}

/**
 * @brief 对基频或分量频率应用当前校准策略。
 * @param fpga_mhz FPGA 频率，单位 0.001 Hz。
 * @return 拟合值或 FPGA 原值，单位 0.001 Hz。
 */
uint32_t measurement_calibration_apply_frequency_mhz(uint32_t fpga_mhz)
{
    double input_hz;
    double output_hz;

    if (measurement_calibration_enabled == 0u)
    {
        return fpga_mhz;
    }

    input_hz = (double)fpga_mhz / 1000.0;
    output_hz = measurement_calibration_evaluate(
        &calibration_frequency_curve, input_hz);
    measurement_calibration_diagnostics.apply_count++;
    return measurement_calibration_saturate_u32(output_hz * 1000.0);
}

/**
 * @brief 对频率分量峰值幅度应用当前校准策略。
 * @param fpga_peak_uv FPGA 已换算的正弦峰值幅度，单位 uV；不是峰峰值。
 * @return 已校准模式返回输入端峰值；未校准模式原样返回 FPGA 微伏值。
 */
uint32_t measurement_calibration_apply_component_amplitude_uv(
    uint32_t fpga_peak_uv)
{
    return measurement_calibration_apply_inverse_frontend_gain(
        fpga_peak_uv, MEASUREMENT_FRONTEND_COMPONENT_GAIN);
}

/**
 * @brief 对直流偏置应用当前校准策略。
 * @param fpga_uv FPGA 已换算的直流偏置，单位 uV。
 * @return 拟合值或 FPGA 原值，单位 uV。
 */
int32_t measurement_calibration_apply_dc_offset_uv(int32_t fpga_uv)
{
    double input_v;
    double output_v;

    if (measurement_calibration_enabled == 0u)
    {
        return fpga_uv;
    }

    input_v = (double)fpga_uv / 1000000.0;
    output_v = measurement_calibration_evaluate(
        &calibration_dc_offset_curve, input_v);
    measurement_calibration_diagnostics.apply_count++;
    return measurement_calibration_saturate_i32(output_v * 1000000.0);
}
