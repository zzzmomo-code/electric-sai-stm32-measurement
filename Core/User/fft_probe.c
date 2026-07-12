/**
 * @file fft_probe.c
 * @brief CMSIS-DSP Q15 RFFT 链接与资源占用验证实现。
 *
 * 模块用途：执行一次不含真实输入数据的 8192 点 Q15 RFFT，以验证 CMSIS-DSP
 * 依赖、查表宏和目标芯片 Flash 预算。
 * GPIO 引脚映射：无直接 GPIO 引脚。
 * 依赖的外设和 CubeIDE 配置：无直接外设依赖；依赖 CMSIS-DSP Q15 RFFT 源文件
 * 和 ARM_DSP_CONFIG_TABLES 相关编译宏。
 * 初始化方法：由 system_init() 调用 fft_probe_init()。
 * 调用方法：完成资源验证后可整体移除本模块；不参与运行期测量。
 */

#include "system.h"
#include "arm_math.h"

#define FFT_PROBE_LENGTH 8192u

/** CMSIS-DSP 的 8192 点 RFFT 实例。 */
static arm_rfft_instance_q15 fft_probe_instance;

/** RFFT 会原地修改输入，因此保留独立的全零输入缓冲区。 */
static q15_t fft_probe_input[FFT_PROBE_LENGTH];

/** RFFT 结果缓冲区，长度与实数输入长度相同。 */
static q15_t fft_probe_output[FFT_PROBE_LENGTH];

/** CMSIS-DSP 初始化返回值，供调试器确认查表是否齐全。 */
int32_t fft_probe_init_status = (int32_t)ARM_MATH_ARGUMENT_ERROR;

/** RFFT 已执行标志，供调试器确认变换路径实际运行。 */
uint8_t fft_probe_executed;

/**
 * @brief 执行一次全零 8192 点 Q15 实数 FFT 验证。
 * @param 无。
 * @return 无。
 * @note 全零输入的频谱应全零；本函数仅验证 CMSIS-DSP 链接和资源占用，不产生测量结果。
 */
void fft_probe_init(void)
{
    fft_probe_init_status = (int32_t)arm_rfft_init_q15(&fft_probe_instance,
                                                        FFT_PROBE_LENGTH,
                                                        0u,
                                                        1u);
    if (fft_probe_init_status != (int32_t)ARM_MATH_SUCCESS)
    {
        return;
    }

    arm_rfft_q15(&fft_probe_instance, fft_probe_input, fft_probe_output);
    fft_probe_executed = 1u;
}
