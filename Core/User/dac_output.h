/**
 * @file dac_output.h
 * @brief DAC 六档直流输出与外部 VGA 增益计算模块。
 *
 * 模块用途：控制 DAC1 通道 1，经片内 OPAMP1 电压跟随器输出六档直流电压，
 *          并根据外部电路公式计算 VG 与 VGA 差分电压增益。
 * GPIO 引脚映射：PC4/OPAMP1_VOUT；DAC1_OUT1 仅使用片内连接，不直接输出到引脚。
 * 依赖的外设和 CubeIDE 配置：DAC1 Channel 1 选择内部连接、无触发；OPAMP1 选择
 *          Follower-DAC_OUT1-INP 模式；PC4 配置为 OPAMP1_VOUT 模拟输出。
 * 初始化方法：CubeMX 的 MX_DAC1_Init()、MX_OPAMP1_Init() 完成后调用 dac_output_init()。
 * 调用方法：调用 dac_output_set_level() 设置 0 至 5 档，使用查询函数取得理论电压、
 *          VG 和 VGA 差分电压增益。
 */

#ifndef DAC_OUTPUT_H
#define DAC_OUTPUT_H

#include <stdint.h>

/** DAC 输出档位总数。 */
#define DAC_OUTPUT_LEVEL_COUNT              6u

/** DAC 参考电压，单位为 mV，应按实际 VDDA 调整。 */
#define DAC_OUTPUT_REFERENCE_MV             3300u

/** 六档标称输出电压，单位为 mV。 */
#define DAC_OUTPUT_LEVEL_0_MV               0u
#define DAC_OUTPUT_LEVEL_1_MV               660u
#define DAC_OUTPUT_LEVEL_2_MV               1320u
#define DAC_OUTPUT_LEVEL_3_MV               1980u
#define DAC_OUTPUT_LEVEL_4_MV               2640u
#define DAC_OUTPUT_LEVEL_5_MV               3300u

/** 外部 VGA 反馈电阻 RF，单位为 ohm。 */
#define DAC_OUTPUT_RF_OHM                   10000.0f

/** 外部 VGA 增益电阻 RG，单位为 ohm，必须大于 0。 */
#define DAC_OUTPUT_RG_OHM                   10000.0f

/** DAC 比例校准系数，默认不修正比例误差。 */
#define DAC_OUTPUT_GAIN_CALIBRATION         1.0f

/** DAC 偏移校准电压，单位为 V，默认不修正零点误差。 */
#define DAC_OUTPUT_OFFSET_CALIBRATION_V     0.0f

/** DAC 输出模块操作状态。 */
typedef enum
{
    dac_output_status_ok = 0,
    dac_output_status_invalid_argument,
    dac_output_status_hal_error
} dac_output_status_t;

/**
 * @brief 启动 OPAMP1 和 DAC1 通道 1，并将输出目标设置为 0 档。
 * @param 无。
 * @return 成功返回 dac_output_status_ok，HAL 启动或写入失败返回 dac_output_status_hal_error。
 * @note 会启动片内模拟外设并更新 PC4 输出；必须在 CubeMX 外设初始化之后调用。
 */
dac_output_status_t dac_output_init(void);

/**
 * @brief 设置 DAC 输出档位。
 * @param level 输出档位，有效范围为 0 至 5。
 * @return 成功返回 dac_output_status_ok；参数越界或 HAL 写入失败时返回对应错误。
 * @note 成功时立即更新 DAC 数据寄存器和当前档位；失败时保持当前档位记录不变。
 */
dac_output_status_t dac_output_set_level(uint8_t level);

/**
 * @brief 获取最近一次成功设置的 DAC 输出档位。
 * @param 无。
 * @return 当前档位，范围为 0 至 5。
 * @note 仅返回软件记录，不读取 DAC 硬件寄存器或 PC4 实测电压。
 */
uint8_t dac_output_get_level(void);

/**
 * @brief 获取指定档位经过增益和偏移校准后的理论 DAC 电压。
 * @param level 输出档位，有效范围为 0 至 5。
 * @param voltage_v 接收理论 DAC 电压的指针，单位为 V。
 * @return 成功返回 dac_output_status_ok；档位越界或指针为空时返回参数错误。
 * @note 结果已限制在 0 V 至 DAC 参考电压范围内。
 */
dac_output_status_t dac_output_get_voltage(uint8_t level, float *voltage_v);

/**
 * @brief 根据指定档位计算外部放大器控制电压 VG。
 * @param level 输出档位，有效范围为 0 至 5。
 * @param vg 接收 VG 理论值的指针。
 * @return 成功返回 dac_output_status_ok；档位越界或指针为空时返回参数错误。
 * @note 使用公式 VG=(20/33)*VDAC-1，VDAC 为校准后的理论电压。
 */
dac_output_status_t dac_output_get_vg(uint8_t level, float *vg);

/**
 * @brief 根据指定档位计算 VGA 差分电压增益。
 * @param level 输出档位，有效范围为 0 至 5。
 * @param gain 接收理论差分电压增益的指针。
 * @return 成功返回 dac_output_status_ok；参数无效时返回 dac_output_status_invalid_argument。
 * @note 使用公式 gain=(1+VG)*RF/RG，不包含输入差分电压本身。
 */
dac_output_status_t dac_output_get_vga_gain(uint8_t level, float *gain);

#endif /* DAC_OUTPUT_H */
