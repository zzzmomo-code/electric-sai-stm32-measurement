/**
 * @file dds_control.h
 * @brief 输入频率到AD9834本振频率的控制接口。
 *
 * 模块用途：测试阶段使用固定输入频率，正式阶段读取TIM5粗测频结果，
 * 按fDDS=fin-100kHz设置低侧本振。
 * GPIO引脚映射：无直接GPIO引脚，底层映射见ad9834.h。
 * 依赖的外设和CubeIDE配置：依赖frequency_measure和ad9834模块。
 * 初始化方法：由system_init()调用dds_control_init()。
 * 调用方法：由system_process()持续调用dds_control_process()。
 */

#ifndef DDS_CONTROL_H
#define DDS_CONTROL_H

#include <stdint.h>

/** 无过零比较器时保持为1；比较器接入PA0后改为0。 */
#define DDS_CONTROL_FIXED_TEST_ENABLE 0u

/** 固定通信测试模拟的输入频率，仅用于诊断记录。 */
#define DDS_CONTROL_TEST_INPUT_HZ 1000000u

/** 无比较器上板测试时的AD9834固定输出频率，单位Hz。 */
#define DDS_CONTROL_TEST_OUTPUT_HZ 100000u

/** 外差方案的目标中频，单位Hz。 */
#define DDS_CONTROL_TARGET_IF_HZ 100000u

/** 题目允许的输入频率范围，单位Hz。 */
#define DDS_CONTROL_MIN_INPUT_HZ 1000000u
#define DDS_CONTROL_MAX_INPUT_HZ 30000000u

/** DDS控制层状态。 */
typedef enum
{
    dds_control_state_waiting = 0,
    dds_control_state_test,
    dds_control_state_tracking,
    dds_control_state_error
} dds_control_state_t;

/** DDS控制层运行诊断，便于上板时在调试器中观察。 */
typedef struct
{
    dds_control_state_t state; /**< 当前等待、测试、跟随或错误状态。 */
    uint32_t input_frequency_hz;  /**< 当前采用的输入频率。 */
    uint32_t output_frequency_hz; /**< 当前目标DDS输出频率。 */
    uint32_t update_count;        /**< 成功更新DDS频率次数。 */
    uint32_t error_count;         /**< DDS更新失败次数。 */
} dds_control_diagnostics_t;

/** DDS控制层运行诊断快照。 */
extern volatile dds_control_diagnostics_t dds_control_diagnostics;

/**
 * @brief 初始化DDS控制层和AD9834。
 * @param 无。
 * @return 无，结果通过dds_control_diagnostics查看。
 */
void dds_control_init(void);

/**
 * @brief 执行固定通信测试或跟随TIM5粗测频结果。
 * @param 无。
 * @return 无，结果通过dds_control_diagnostics查看。
 */
void dds_control_process(void);

/**
 * @brief 根据输入频率计算低侧DDS本振频率。
 * @param input_frequency_hz 输入信号频率，单位Hz。
 * @return 合法时返回input-100kHz，超出题目范围时返回0。
 */
uint32_t dds_control_calculate_output_hz(uint32_t input_frequency_hz);

#endif /* DDS_CONTROL_H */
