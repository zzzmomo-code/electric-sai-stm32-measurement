/**
 * @file vga_control.h
 * @brief 片上 DAC 六档输出与外接差分 VGA 增益控制模块接口。
 *
 * 模块用途：控制 DAC1_OUT1 输出六档直流电压，并由档位计算外接 VGA 的理论增益。
 * GPIO 引脚映射：PA4/DAC1_OUT1，连接外部控制电压放大器的输入端。
 * 依赖的外设和 CubeIDE 配置：DAC1 Channel 1、无触发、输出缓冲开启，PA4 为模拟无上下拉。
 * 初始化方法：CubeMX 完成 MX_DAC1_Init() 后，由 system_init() 调用 vga_control_init()。
 * 调用方法：调用 vga_control_set_level() 设置 0 至 5 档；调用
 * vga_control_gain_from_level() 查询指定档位对应的 VGA 理论增益。
 */

#ifndef VGA_CONTROL_H
#define VGA_CONTROL_H

#include <stdint.h>

/** 片上 DAC 的标称参考电压；实测 VDDA 不同时可修改此宏进行校准。 */
#define VGA_CONTROL_DAC_REFERENCE_VOLTAGE_V 3.30f

/** 第 0 档 DAC 目标输出电压。 */
#define VGA_CONTROL_LEVEL_0_VOLTAGE_V 0.00f
/** 第 1 档 DAC 目标输出电压。 */
#define VGA_CONTROL_LEVEL_1_VOLTAGE_V 0.66f
/** 第 2 档 DAC 目标输出电压。 */
#define VGA_CONTROL_LEVEL_2_VOLTAGE_V 1.32f
/** 第 3 档 DAC 目标输出电压。 */
#define VGA_CONTROL_LEVEL_3_VOLTAGE_V 1.98f
/** 第 4 档 DAC 目标输出电压。 */
#define VGA_CONTROL_LEVEL_4_VOLTAGE_V 2.64f
/** 第 5 档 DAC 目标输出电压。 */
#define VGA_CONTROL_LEVEL_5_VOLTAGE_V 3.30f

/** 外部控制电压放大器公式 VG=(20/33)*VDAC-1 的比例系数。 */
#define VGA_CONTROL_VG_SCALE (20.0f / 33.0f)
/** 外部控制电压放大器公式中的偏置，单位为伏。 */
#define VGA_CONTROL_VG_OFFSET_V (-1.0f)

/** VGA 反馈电阻 Rf 的归一化值；实际阻值确定后在此修改。 */
#define VGA_CONTROL_RF 1.0f
/** VGA 增益电阻 RG 的归一化值；必须大于 0，实际阻值确定后在此修改。 */
#define VGA_CONTROL_RG 1.0f

/** 根据 DAC 输出电压计算外部放大器控制电压 VG。 */
#define VGA_CONTROL_VG_FROM_DAC_VOLTAGE(dac_voltage_v) \
    ((VGA_CONTROL_VG_SCALE * (dac_voltage_v)) + VGA_CONTROL_VG_OFFSET_V)

/** 根据控制电压 VG 计算差分 VGA 理论增益。 */
#define VGA_CONTROL_GAIN_FROM_VG(vg_voltage_v) \
    (((1.0f + (vg_voltage_v)) * VGA_CONTROL_RF) / VGA_CONTROL_RG)

/** 根据正、负输入端电压与 VG 计算 VGA 理论输出电压。 */
#define VGA_CONTROL_VOUT_FROM_INPUTS(vin_positive_v, vin_negative_v, vg_voltage_v) \
    (((vin_positive_v) - (vin_negative_v)) * VGA_CONTROL_GAIN_FROM_VG(vg_voltage_v))

/** VGA 控制模块公开状态码。 */
typedef enum
{
    vga_control_status_ok = 0,
    vga_control_status_invalid_level,
    vga_control_status_null_pointer,
    vga_control_status_dac_error
} vga_control_status_t;

/** VGA 控制模块调试快照。 */
typedef struct
{
    uint8_t current_level;                 /**< 最近一次成功写入的档位。 */
    float dac_voltage_v;                   /**< 最近一次成功设置的 DAC 目标电压。 */
    float vg_voltage_v;                    /**< 最近一次成功档位对应的 VG 理论值。 */
    float vga_gain;                        /**< 最近一次成功档位对应的 VGA 理论增益。 */
    vga_control_status_t last_status;      /**< 最近一次设置或初始化的模块状态。 */
    uint32_t last_hal_status;              /**< 最近一次 DAC HAL 操作的返回状态。 */
} vga_control_diagnostics_t;

/** 供主循环和调试器观察的 VGA 控制状态，不在中断中访问。 */
extern vga_control_diagnostics_t vga_control_diagnostics;

/**
 * @brief 启动 DAC1_OUT1，并把系统安全初始化为第 0 档。
 * @param 无。
 * @return 无；执行结果写入 vga_control_diagnostics。
 * @note 依赖 MX_DAC1_Init() 已完成；会启动 DAC 通道并写入 0 V 目标值。
 */
void vga_control_init(void);

/**
 * @brief 使用 switch 选择并输出指定 DAC 电压档位。
 * @param level 电压档位，有效范围为 0 至 5。
 * @return 成功返回 vga_control_status_ok；非法档位或 HAL 失败返回对应错误。
 * @note 非法档位不会调用 HAL，也不会改变当前 DAC 输出和有效档位记录。
 */
vga_control_status_t vga_control_set_level(uint8_t level);

/**
 * @brief 使用 switch 由档位计算外接差分 VGA 的理论增益。
 * @param level 电压档位，有效范围为 0 至 5。
 * @param gain 用于接收理论增益的非空指针。
 * @return 成功返回 vga_control_status_ok；非法档位或空指针返回对应错误。
 * @note 仅执行浮点公式计算，不访问 DAC，也不改变硬件输出。
 */
vga_control_status_t vga_control_gain_from_level(uint8_t level, float *gain);

#endif /* VGA_CONTROL_H */
