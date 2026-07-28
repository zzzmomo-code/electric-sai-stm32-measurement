/**
 * @file vga_control.h
 * @brief 片上 DAC 六档输出与外接 VCA821 程控放大器模块接口。
 *
 * 模块用途：控制 DAC1_OUT1 输出六档直流电压，并按模块手册分段曲线计算 VCA821 理论增益。
 * GPIO 引脚映射：PA4/DAC1_OUT1，连接 VCA821 模块的外接 DA 控制输入端。
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

/** 第 0 档 DAC 指令电压，仅用于自动生成片上 DAC 数字码。 */
#define VGA_CONTROL_LEVEL_0_VOLTAGE_V 0.617f
/** 第 1 档 DAC 指令电压，仅用于自动生成片上 DAC 数字码。 */
#define VGA_CONTROL_LEVEL_1_VOLTAGE_V 0.717f
/** 第 2 档 DAC 指令电压，仅用于自动生成片上 DAC 数字码。 */
#define VGA_CONTROL_LEVEL_2_VOLTAGE_V 0.817f
/** 第 3 档 DAC 指令电压，仅用于自动生成片上 DAC 数字码。 */
#define VGA_CONTROL_LEVEL_3_VOLTAGE_V 0.917f
/** 第 4 档 DAC 指令电压，仅用于自动生成片上 DAC 数字码。 */
#define VGA_CONTROL_LEVEL_4_VOLTAGE_V 1.017f
/** 第 5 档 DAC 指令电压，仅用于自动生成片上 DAC 数字码。 */
#define VGA_CONTROL_LEVEL_5_VOLTAGE_V 1.117f

/*
 * 下列六个值先与 DAC 指令值保持一致，作为尚未完成实板标定时的默认值。
 * 实测 PA4 后只修改对应宏，DAC 指令值保持不变。
 */
#define VGA_CONTROL_LEVEL_0_MEASURED_VOLTAGE_V 0.617f
#define VGA_CONTROL_LEVEL_1_MEASURED_VOLTAGE_V 0.717f
#define VGA_CONTROL_LEVEL_2_MEASURED_VOLTAGE_V 0.817f
#define VGA_CONTROL_LEVEL_3_MEASURED_VOLTAGE_V 0.917f
#define VGA_CONTROL_LEVEL_4_MEASURED_VOLTAGE_V 1.017f
#define VGA_CONTROL_LEVEL_5_MEASURED_VOLTAGE_V 1.117f

/** VCA821 模块手册给出的外接 DA 控制电压安全工作下限。 */
#define VGA_CONTROL_VCA821_CONTROL_MIN_V 0.617f
/** VCA821 模块手册给出的外接 DA 控制电压安全工作上限。 */
#define VGA_CONTROL_VCA821_CONTROL_MAX_V 1.220f
/** VCA821 模块可调增益下限，单位为 dB。 */
#define VGA_CONTROL_VCA821_GAIN_MIN_DB 0.0f
/** VCA821 模块可调增益上限，单位为 dB。 */
#define VGA_CONTROL_VCA821_GAIN_MAX_DB 20.0f

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
    float dac_voltage_v;                   /**< 当前档位用于生成 DAC 码的指令电压。 */
    float measured_voltage_v;              /**< 当前档位 PA4 实测电压，用于 VG 与 AV 模型。 */
    float vg_voltage_v;                    /**< VCA821 模块外接 DA 控制电压，单位为 V。 */
    float gain_db;                         /**< 按模块分段标定曲线计算的理论增益，单位为 dB。 */
    float vga_gain;                        /**< 由 dB 换算得到的理论线性电压增益。 */
    vga_control_status_t last_status;      /**< 最近一次设置或初始化的模块状态。 */
    uint32_t last_hal_status;              /**< 最近一次 DAC HAL 操作的返回状态。 */
} vga_control_diagnostics_t;

/** 供主循环和调试器观察的 VGA 控制状态，不在中断中访问。 */
extern vga_control_diagnostics_t vga_control_diagnostics;

/**
 * @brief 启动 DAC1_OUT1，并把系统安全初始化为第 0 档。
 * @param 无。
 * @return 无；执行结果写入 vga_control_diagnostics。
 * @note 依赖 MX_DAC1_Init() 已完成；先预装 0 V 目标值，再启动 DAC 通道。
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
 * @brief 使用 switch 由档位计算 VCA821 模块的理论线性增益。
 * @param level 电压档位，有效范围为 0 至 5。
 * @param gain 用于接收理论增益的非空指针。
 * @return 成功返回 vga_control_status_ok；非法档位或空指针返回对应错误。
 * @note 仅执行浮点公式计算，不访问 DAC，也不改变硬件输出。
 */
vga_control_status_t vga_control_gain_from_level(uint8_t level, float *gain);

#endif /* VGA_CONTROL_H */
