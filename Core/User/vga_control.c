/**
 * @file vga_control.c
 * @brief 片上 DAC 六档输出与外接 VCA821 程控放大器控制实现。
 *
 * 模块用途：通过 PA4/DAC1_OUT1 输出六档直流电压，并按模块手册分段曲线计算 VCA821 理论增益。
 * GPIO 引脚映射：PA4/DAC1_OUT1，连接 VCA821 模块的外接 DA 控制输入端。
 * 依赖的外设和 CubeIDE 配置：DAC1 Channel 1、无触发、输出缓冲开启，PA4 为模拟无上下拉。
 * 初始化方法：CubeMX 完成 MX_DAC1_Init() 后，由 system_init() 调用 vga_control_init()。
 * 调用方法：使用 vga_control_set_level() 切换档位，使用
 * vga_control_gain_from_level() 查询指定档位的理论差分增益。
 */

#include "system.h"

_Static_assert(VGA_CONTROL_DAC_REFERENCE_VOLTAGE_V > 0.0f,
               "DAC reference voltage must be positive");
_Static_assert(VGA_CONTROL_LEVEL_0_VOLTAGE_V >= VGA_CONTROL_VCA821_CONTROL_MIN_V,
               "VCA821 level 0 is below the documented control range");
_Static_assert(VGA_CONTROL_LEVEL_5_VOLTAGE_V <= VGA_CONTROL_VCA821_CONTROL_MAX_V,
               "VCA821 level 5 is above the documented control range");

/** 模块状态快照；初始档位 0xff 表示尚未成功设置任何有效档位。 */
vga_control_diagnostics_t vga_control_diagnostics = {
    0xffu,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    vga_control_status_dac_error,
    (uint32_t)HAL_ERROR
};

/**
 * @brief 按 VCA821 模块手册的三段拟合公式，由控制电压计算增益。
 * @param control_voltage_v PA4 实测控制电压，单位为 V。
 * @return 限制在 0 至 20 dB 内的理论增益。
 * @note 以各段增益上限对应的控制电压作为切换点，避免实测偏差跨档后仍使用错误分段。
 */
static float vga_control_gain_db_from_voltage(float control_voltage_v)
{
    float control_voltage_mv = control_voltage_v * 1000.0f;
    float gain_db;

    if (control_voltage_mv <= 830.226f)
    {
        gain_db = (control_voltage_mv - 606.38f) / 15.989f;
    }
    else if (control_voltage_mv <= 941.14f)
    {
        gain_db = (control_voltage_mv - 247.2f) / 40.82f;
    }
    else
    {
        gain_db = (control_voltage_mv + 1137.8f) / 115.48f;
    }

    if (gain_db < VGA_CONTROL_VCA821_GAIN_MIN_DB)
    {
        gain_db = VGA_CONTROL_VCA821_GAIN_MIN_DB;
    }
    else if (gain_db > VGA_CONTROL_VCA821_GAIN_MAX_DB)
    {
        gain_db = VGA_CONTROL_VCA821_GAIN_MAX_DB;
    }

    return gain_db;
}

/**
 * @brief 把分贝增益换算为幅度换算所需的线性电压增益。
 * @param gain_db 理论增益，单位为 dB。
 * @return 线性电压增益。
 */
static float vga_control_linear_gain_from_db(float gain_db)
{
    return powf(10.0f, gain_db / 20.0f);
}

/**
 * @brief 把目标电压自动换算为片上 DAC 的 12 位数字码。
 * @param dac_voltage_v 目标 DAC 输出电压，单位为伏。
 * @return 四舍五入并限制在 12 位范围内的 DAC 数字码。
 * @note 仅执行数值换算，不访问硬件；12 位量程由片上 DAC 固定，不对外提供手动档位码。
 */
static uint32_t vga_control_voltage_to_dac_code(float dac_voltage_v)
{
    /** 片上 DAC 固定 12 位右对齐时的最大数字码。 */
    const uint32_t dac_max_code = (1u << 12u) - 1u;
    /** 根据目标电压和标称参考电压四舍五入得到的数字码。 */
    uint32_t dac_code;

    if (dac_voltage_v <= 0.0f)
    {
        return 0u;
    }

    dac_code = (uint32_t)(((dac_voltage_v
                            / VGA_CONTROL_DAC_REFERENCE_VOLTAGE_V)
                           * (float)dac_max_code) + 0.5f);
    if (dac_code > dac_max_code)
    {
        dac_code = dac_max_code;
    }

    return dac_code;
}

/**
 * @brief 启动 DAC1_OUT1，并把系统安全初始化为第 0 档。
 * @param 无。
 * @return 无；执行结果写入 vga_control_diagnostics。
 * @note 必须在 MX_DAC1_Init() 完成后调用；会启动 DAC 通道并写入第 0 档。
 */
void vga_control_init(void)
{
    /** DAC 预装和启动操作的 HAL 返回状态。 */
    HAL_StatusTypeDef hal_status;

    hal_status = HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0u);
    vga_control_diagnostics.last_hal_status = (uint32_t)hal_status;
    if (hal_status != HAL_OK)
    {
        vga_control_diagnostics.last_status = vga_control_status_dac_error;
        return;
    }

    hal_status = HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
    vga_control_diagnostics.last_hal_status = (uint32_t)hal_status;
    if (hal_status != HAL_OK)
    {
        vga_control_diagnostics.last_status = vga_control_status_dac_error;
        return;
    }

    vga_control_diagnostics.last_status = vga_control_set_level(0u);
}

/**
 * @brief 使用 switch 选择并输出指定 DAC 电压档位。
 * @param level 电压档位，有效范围为 0 至 5。
 * @return 成功返回 vga_control_status_ok；非法档位或 HAL 失败返回对应错误。
 * @note 非法档位在 HAL 写入前返回，不改变当前输出和有效档位记录。
 */
vga_control_status_t vga_control_set_level(uint8_t level)
{
    /** 当前档位用于自动生成 DAC 数字码的指令电压。 */
    float dac_voltage_v;
    /** 当前档位 PA4 的实测电压，仅用于 VG 与 AV 模型计算。 */
    float measured_voltage_v;
    /** 当前档位对应的外部放大器控制电压。 */
    float vg_voltage_v;
    /** 当前档位对应的 VCA821 理论线性增益。 */
    float gain;
    /** 当前档位对应的 VCA821 理论增益，单位为 dB。 */
    float gain_db;
    /** 自动换算得到的片上 DAC 12 位数字码。 */
    uint32_t dac_code;
    /** DAC 数据写入操作的 HAL 返回状态。 */
    HAL_StatusTypeDef hal_status;

    switch (level)
    {
        case 0u:
            dac_voltage_v = VGA_CONTROL_LEVEL_0_VOLTAGE_V;
            measured_voltage_v = VGA_CONTROL_LEVEL_0_MEASURED_VOLTAGE_V;
            break;
        case 1u:
            dac_voltage_v = VGA_CONTROL_LEVEL_1_VOLTAGE_V;
            measured_voltage_v = VGA_CONTROL_LEVEL_1_MEASURED_VOLTAGE_V;
            break;
        case 2u:
            dac_voltage_v = VGA_CONTROL_LEVEL_2_VOLTAGE_V;
            measured_voltage_v = VGA_CONTROL_LEVEL_2_MEASURED_VOLTAGE_V;
            break;
        case 3u:
            dac_voltage_v = VGA_CONTROL_LEVEL_3_VOLTAGE_V;
            measured_voltage_v = VGA_CONTROL_LEVEL_3_MEASURED_VOLTAGE_V;
            break;
        case 4u:
            dac_voltage_v = VGA_CONTROL_LEVEL_4_VOLTAGE_V;
            measured_voltage_v = VGA_CONTROL_LEVEL_4_MEASURED_VOLTAGE_V;
            break;
        case 5u:
            dac_voltage_v = VGA_CONTROL_LEVEL_5_VOLTAGE_V;
            measured_voltage_v = VGA_CONTROL_LEVEL_5_MEASURED_VOLTAGE_V;
            break;
        default:
            vga_control_diagnostics.last_status = vga_control_status_invalid_level;
            return vga_control_status_invalid_level;
    }

    dac_code = vga_control_voltage_to_dac_code(dac_voltage_v);
    hal_status = HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_code);
    vga_control_diagnostics.last_hal_status = (uint32_t)hal_status;
    if (hal_status != HAL_OK)
    {
        vga_control_diagnostics.last_status = vga_control_status_dac_error;
        return vga_control_status_dac_error;
    }

    vg_voltage_v = measured_voltage_v;
    gain_db = vga_control_gain_db_from_voltage(measured_voltage_v);
    gain = vga_control_linear_gain_from_db(gain_db);
    vga_control_diagnostics.current_level = level;
    vga_control_diagnostics.dac_voltage_v = dac_voltage_v;
    vga_control_diagnostics.measured_voltage_v = measured_voltage_v;
    vga_control_diagnostics.vg_voltage_v = vg_voltage_v;
    vga_control_diagnostics.gain_db = gain_db;
    vga_control_diagnostics.vga_gain = gain;
    vga_control_diagnostics.last_status = vga_control_status_ok;

    return vga_control_status_ok;
}

/**
 * @brief 使用 switch 由档位计算 VCA821 模块的理论线性增益。
 * @param level 电压档位，有效范围为 0 至 5。
 * @param gain 用于接收理论增益的非空指针。
 * @return 成功返回 vga_control_status_ok；非法档位或空指针返回对应错误。
 * @note 失败时不写入 gain 指向的存储位置，也不访问 DAC 硬件。
 */
vga_control_status_t vga_control_gain_from_level(uint8_t level, float *gain)
{
    /** 当前档位 PA4 的实测电压，用于 VG 与 AV 模型计算。 */
    float measured_voltage_v;
    /** 根据 PA4 实测电压计算得到的 VCA821 理论增益，单位为 dB。 */
    float gain_db;

    if (gain == NULL)
    {
        return vga_control_status_null_pointer;
    }

    switch (level)
    {
        case 0u:
            measured_voltage_v = VGA_CONTROL_LEVEL_0_MEASURED_VOLTAGE_V;
            break;
        case 1u:
            measured_voltage_v = VGA_CONTROL_LEVEL_1_MEASURED_VOLTAGE_V;
            break;
        case 2u:
            measured_voltage_v = VGA_CONTROL_LEVEL_2_MEASURED_VOLTAGE_V;
            break;
        case 3u:
            measured_voltage_v = VGA_CONTROL_LEVEL_3_MEASURED_VOLTAGE_V;
            break;
        case 4u:
            measured_voltage_v = VGA_CONTROL_LEVEL_4_MEASURED_VOLTAGE_V;
            break;
        case 5u:
            measured_voltage_v = VGA_CONTROL_LEVEL_5_MEASURED_VOLTAGE_V;
            break;
        default:
            return vga_control_status_invalid_level;
    }

    gain_db = vga_control_gain_db_from_voltage(measured_voltage_v);
    *gain = vga_control_linear_gain_from_db(gain_db);

    return vga_control_status_ok;
}
