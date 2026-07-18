/**
 * @file dds_control.c
 * @brief 输入频率到AD9834本振频率的控制实现。
 *
 * 模块用途：固定测试模式每秒重写一次900kHz，正式模式读取TIM5粗测频结果
 * 并在频率变化达到500Hz时更新DDS。
 * GPIO引脚映射：无直接GPIO引脚，底层映射见ad9834.h。
 * 依赖的外设和CubeIDE配置：依赖frequency_measure和ad9834模块。
 * 初始化方法：由system_init()调用dds_control_init()。
 * 调用方法：由system_process()持续调用dds_control_process()。
 */

#include "system.h"

#define DDS_CONTROL_TEST_REFRESH_MS 1000u
#define DDS_CONTROL_UPDATE_THRESHOLD_HZ 500u

/** DDS控制层运行诊断快照。 */
volatile dds_control_diagnostics_t dds_control_diagnostics;

/** 最近一次成功更新DDS的系统毫秒时刻。 */
static uint32_t dds_control_last_update_ms = 0u;

/**
 * @brief 计算两个无符号频率值的绝对差。
 * @param first_hz 第一个频率。
 * @param second_hz 第二个频率。
 * @return 频率绝对差，单位Hz。
 */
#if (DDS_CONTROL_FIXED_TEST_ENABLE == 0u)
static uint32_t dds_control_frequency_difference(uint32_t first_hz,
                                                 uint32_t second_hz)
{
    return (first_hz >= second_hz) ? (first_hz - second_hz)
                                   : (second_hz - first_hz);
}
#endif

/**
 * @brief 根据输入频率计算低侧DDS本振频率。
 * @param input_frequency_hz 输入信号频率，单位Hz。
 * @return 合法时返回input-100kHz，超出题目范围时返回0。
 */
uint32_t dds_control_calculate_output_hz(uint32_t input_frequency_hz)
{
    if ((input_frequency_hz < DDS_CONTROL_MIN_INPUT_HZ)
        || (input_frequency_hz > DDS_CONTROL_MAX_INPUT_HZ))
    {
        return 0u;
    }

    return input_frequency_hz - DDS_CONTROL_TARGET_IF_HZ;
}

/**
 * @brief 设置DDS并更新控制层诊断。
 * @param input_frequency_hz 当前采用的输入频率。
 * @return 无，失败时状态切换为error。
 */
static void dds_control_apply_frequency(uint32_t input_frequency_hz)
{
    uint32_t output_frequency_hz;

    output_frequency_hz = dds_control_calculate_output_hz(input_frequency_hz);
    if ((output_frequency_hz == 0u)
        || (ad9834_set_frequency_hz(output_frequency_hz) != ad9834_status_ok))
    {
        dds_control_diagnostics.error_count++;
        dds_control_diagnostics.state = dds_control_state_error;
        return;
    }

    dds_control_diagnostics.input_frequency_hz = input_frequency_hz;
    dds_control_diagnostics.output_frequency_hz = output_frequency_hz;
    dds_control_diagnostics.update_count++;
    dds_control_last_update_ms = HAL_GetTick();
}

/**
 * @brief 初始化DDS控制层和AD9834。
 * @param 无。
 * @return 无，结果通过dds_control_diagnostics查看。
 */
void dds_control_init(void)
{
    uint32_t initial_output_hz;

    dds_control_diagnostics.state = dds_control_state_waiting;
    dds_control_diagnostics.input_frequency_hz = 0u;
    dds_control_diagnostics.output_frequency_hz = 0u;
    dds_control_diagnostics.update_count = 0u;
    dds_control_diagnostics.error_count = 0u;
    dds_control_last_update_ms = HAL_GetTick();

    initial_output_hz = dds_control_calculate_output_hz(
        DDS_CONTROL_TEST_INPUT_HZ);
    if (ad9834_init(initial_output_hz) != ad9834_status_ok)
    {
        dds_control_diagnostics.error_count = 1u;
        dds_control_diagnostics.state = dds_control_state_error;
        return;
    }

    dds_control_diagnostics.input_frequency_hz = DDS_CONTROL_TEST_INPUT_HZ;
    dds_control_diagnostics.output_frequency_hz = initial_output_hz;
    dds_control_diagnostics.update_count = 1u;
#if (DDS_CONTROL_FIXED_TEST_ENABLE != 0u)
    dds_control_diagnostics.state = dds_control_state_test;
#else
    dds_control_diagnostics.state = dds_control_state_waiting;
#endif
}

/**
 * @brief 执行固定通信测试或跟随TIM5粗测频结果。
 * @param 无。
 * @return 无，结果通过dds_control_diagnostics查看。
 */
void dds_control_process(void)
{
#if (DDS_CONTROL_FIXED_TEST_ENABLE != 0u)
    uint32_t now_ms = HAL_GetTick();

    if ((uint32_t)(now_ms - dds_control_last_update_ms)
        >= DDS_CONTROL_TEST_REFRESH_MS)
    {
        dds_control_apply_frequency(DDS_CONTROL_TEST_INPUT_HZ);
        if (dds_control_diagnostics.state != dds_control_state_error)
        {
            dds_control_diagnostics.state = dds_control_state_test;
        }
    }
#else
    uint32_t measured_input_hz;
    uint32_t desired_output_hz;

    measured_input_hz = (uint32_t)(frequency_measure_hz + 0.5f);
    desired_output_hz = dds_control_calculate_output_hz(measured_input_hz);
    if (desired_output_hz == 0u)
    {
        dds_control_diagnostics.state = dds_control_state_waiting;
        return;
    }

    if (dds_control_frequency_difference(
            desired_output_hz,
            dds_control_diagnostics.output_frequency_hz)
        < DDS_CONTROL_UPDATE_THRESHOLD_HZ)
    {
        return;
    }

    dds_control_apply_frequency(measured_input_hz);
    if (dds_control_diagnostics.state != dds_control_state_error)
    {
        dds_control_diagnostics.state = dds_control_state_tracking;
    }
#endif
}
