/**
 * @file dds_control.c
 * @brief TIM5 粗调与 ADC 中频闭环修正 AD9834 输出频率的控制实现。
 *
 * 模块用途：固定测试模式周期重写指定频率；正式模式由串口屏立即测量命令触发，
 * 先按 TIM5 粗测频率设置低侧本振，再根据新的 ADC/FFT 中频连续修正五次并保持。
 * GPIO 引脚映射：无直接 GPIO 引脚，AD9834 底层映射见 ad9834.h。
 * 依赖的外设和 CubeIDE 配置：依赖 frequency_measure、measurement_result、
 * measurement_fft 和 ad9834 模块；不在中断中执行计算或 SPI 写入。
 * 初始化方法：由 system_init() 调用 dds_control_init()。
 * 调用方法：HMI 命令调用 dds_control_request_compensation()，主循环持续调用
 * dds_control_process()。
 */

#include "system.h"

#include <math.h>

#define DDS_CONTROL_TEST_REFRESH_MS 1000u

/** DDS 控制层运行诊断快照，仅在主循环上下文中修改。 */
volatile dds_control_diagnostics_t dds_control_diagnostics;

/** 最近一次成功更新 DDS 的系统毫秒时刻，供固定测试模式使用。 */
static uint32_t dds_control_last_update_ms;

/** 非零表示主循环需要重新开始一次粗调和五次 ADC 闭环补偿。 */
static uint8_t dds_control_compensation_requested;

/**
 * @brief 根据输入频率计算低侧 DDS 本振频率。
 * @param input_frequency_hz 输入信号频率，单位为 Hz。
 * @return 合法时返回 input-100 kHz，超出题目范围时返回 0。
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
 * @brief 写入 DDS 输出并更新公共诊断。
 * @param input_frequency_hz 当前采用的外界输入频率，单位为 Hz。
 * @param output_frequency_hz 需要写入 AD9834 的频率，单位为 Hz。
 * @return 写入成功返回 1；目标越界或底层写入失败返回 0。
 * @note 失败时增加错误计数并终止本轮补偿。
 */
static uint8_t dds_control_apply_output(uint32_t input_frequency_hz,
                                        uint32_t output_frequency_hz)
{
    if ((output_frequency_hz == 0u)
        || (output_frequency_hz > AD9834_MAX_OUTPUT_HZ)
        || (ad9834_set_frequency_hz(output_frequency_hz) != ad9834_status_ok))
    {
        dds_control_diagnostics.error_count++;
        dds_control_diagnostics.state = dds_control_state_error;
        return 0u;
    }

    dds_control_diagnostics.input_frequency_hz = input_frequency_hz;
    dds_control_diagnostics.output_frequency_hz = output_frequency_hz;
    dds_control_diagnostics.update_count++;
    dds_control_last_update_ms = HAL_GetTick();
    return 1u;
}

/**
 * @brief 记录当前结果序号，使粗调前的 FFT 结果不能参与补偿。
 * @param 无。
 * @return 无。
 */
static void dds_control_skip_existing_result(void)
{
    measurement_result_t result;

    if (measurement_result_get_snapshot(&result) != 0u)
    {
        dds_control_diagnostics.last_result_sequence = result.sequence;
    }
    else
    {
        dds_control_diagnostics.last_result_sequence = 0u;
    }
}

/**
 * @brief 尝试使用本次 TIM5 粗测结果设置 DDS 初始频率。
 * @param 无。
 * @return 粗调成功返回 1；粗测结果尚不可用返回 0。
 * @note 粗调不计入五次 ADC 补偿；写入失败会进入错误状态。
 */
static uint8_t dds_control_start_coarse_adjustment(void)
{
    uint32_t measured_input_hz;
    uint32_t desired_output_hz;

    if ((!isfinite(frequency_measure_hz)) || (frequency_measure_hz <= 0.0f))
    {
        return 0u;
    }

    measured_input_hz = (uint32_t)lroundf(frequency_measure_hz);
    desired_output_hz = dds_control_calculate_output_hz(measured_input_hz);
    if (desired_output_hz == 0u)
    {
        return 0u;
    }

    if (dds_control_apply_output(measured_input_hz, desired_output_hz) == 0u)
    {
        return 0u;
    }

    dds_control_diagnostics.compensation_count = 0u;
    dds_control_diagnostics.adc_frequency_hz = 0.0f;
    dds_control_diagnostics.frequency_error_hz = 0;
    dds_control_skip_existing_result();
    measurement_fft_resynchronize();
    dds_control_diagnostics.state = dds_control_state_compensating;
    return 1u;
}

/**
 * @brief 使用一帧新的有效 ADC 中频修正 DDS。
 * @param 无。
 * @return 本次成功消费并写入 DDS 返回 1，否则返回 0。
 * @note 无效或重复帧只被忽略；目标越界或 SPI 失败会进入错误状态。
 */
static uint8_t dds_control_apply_adc_compensation(void)
{
    measurement_result_t result;
    int64_t desired_output_hz;

    if ((measurement_result_get_snapshot(&result) == 0u)
        || (result.valid == 0u)
        || ((result.valid_mask & MEASUREMENT_VALID_FREQUENCY) == 0u)
        || (!isfinite(result.frequency_hz))
        || (result.frequency_hz <= 0.0f)
        || (result.sequence == dds_control_diagnostics.last_result_sequence))
    {
        return 0u;
    }

    dds_control_diagnostics.last_result_sequence = result.sequence;
    dds_control_diagnostics.adc_frequency_hz = result.frequency_hz;
    dds_control_diagnostics.frequency_error_hz = (int32_t)lroundf(
        result.frequency_hz - (float)DDS_CONTROL_TARGET_IF_HZ);
    desired_output_hz = (int64_t)dds_control_diagnostics.output_frequency_hz
                        + (int64_t)dds_control_diagnostics.frequency_error_hz;
    if ((desired_output_hz < 1)
        || (desired_output_hz > (int64_t)AD9834_MAX_OUTPUT_HZ))
    {
        dds_control_diagnostics.error_count++;
        dds_control_diagnostics.state = dds_control_state_error;
        return 0u;
    }

    if (dds_control_apply_output(
            dds_control_diagnostics.input_frequency_hz,
            (uint32_t)desired_output_hz) == 0u)
    {
        return 0u;
    }

    dds_control_diagnostics.compensation_count++;
    if (dds_control_diagnostics.compensation_count
        >= DDS_CONTROL_COMPENSATION_LIMIT)
    {
        dds_control_diagnostics.state = dds_control_state_holding;
    }
    else
    {
        measurement_fft_resynchronize();
    }
    return 1u;
}

/**
 * @brief 请求重新执行一次 TIM5 粗调和五次 ADC 中频闭环补偿。
 * @param 无。
 * @return 无。
 * @note 仅设置主循环消费的请求，不直接读取计数器或写入 DDS。
 */
void dds_control_request_compensation(void)
{
    dds_control_compensation_requested = 1u;
    dds_control_diagnostics.compensation_request_count++;
}

/**
 * @brief 初始化 DDS 控制层和 AD9834。
 * @param 无。
 * @return 无，结果通过 dds_control_diagnostics 查看。
 */
void dds_control_init(void)
{
    uint32_t initial_output_hz;

    dds_control_diagnostics.state = dds_control_state_waiting;
    dds_control_diagnostics.input_frequency_hz = 0u;
    dds_control_diagnostics.output_frequency_hz = 0u;
    dds_control_diagnostics.update_count = 0u;
    dds_control_diagnostics.error_count = 0u;
    dds_control_diagnostics.compensation_count = 0u;
    dds_control_diagnostics.compensation_request_count = 0u;
    dds_control_diagnostics.adc_frequency_hz = 0.0f;
    dds_control_diagnostics.frequency_error_hz = 0;
    dds_control_diagnostics.last_result_sequence = 0u;
    dds_control_last_update_ms = HAL_GetTick();
    dds_control_compensation_requested = 0u;

#if (DDS_CONTROL_FIXED_TEST_ENABLE != 0u)
    initial_output_hz = DDS_CONTROL_TEST_OUTPUT_HZ;
#else
    initial_output_hz = dds_control_calculate_output_hz(
        DDS_CONTROL_TEST_INPUT_HZ);
#endif
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
 * @brief 执行固定通信测试或一次由 HMI 请求的粗调与五次 ADC 闭环补偿。
 * @param 无。
 * @return 无，结果通过 dds_control_diagnostics 查看。
 */
void dds_control_process(void)
{
#if (DDS_CONTROL_FIXED_TEST_ENABLE != 0u)
    uint32_t now_ms = HAL_GetTick();

    if ((uint32_t)(now_ms - dds_control_last_update_ms)
        >= DDS_CONTROL_TEST_REFRESH_MS)
    {
        if (dds_control_apply_output(DDS_CONTROL_TEST_INPUT_HZ,
                                     DDS_CONTROL_TEST_OUTPUT_HZ) != 0u)
        {
            dds_control_diagnostics.state = dds_control_state_test;
        }
    }
#else
    if (dds_control_compensation_requested != 0u)
    {
        dds_control_compensation_requested = 0u;
        dds_control_diagnostics.compensation_count = 0u;
        dds_control_diagnostics.adc_frequency_hz = 0.0f;
        dds_control_diagnostics.frequency_error_hz = 0;
        dds_control_diagnostics.state = dds_control_state_coarse;
    }

    if (dds_control_diagnostics.state == dds_control_state_coarse)
    {
        (void)dds_control_start_coarse_adjustment();
        return;
    }

    if (dds_control_diagnostics.state == dds_control_state_compensating)
    {
        (void)dds_control_apply_adc_compensation();
    }
#endif
}
