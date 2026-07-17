/**
 * @file fft_f32_65536.h
 * @brief 65536 点单精度浮点实数 FFT 公共接口。
 *
 * 模块用途：将等间隔的 65536 个无符号 16 位样本去直流、加 Hann 窗后，
 * 执行原地实数 FFT，并输出 0 至 Nyquist 的压缩复数频谱。
 * GPIO 引脚映射：无直接 GPIO 引脚。
 * 依赖的外设和 CubeIDE 配置：无外设依赖；要求 Cortex-M7 单精度 FPU 与数学库。
 * 初始化方法：无需单独初始化。
 * 调用方法：在停止采样后调用 fft_f32_65536_forward()，输入与输出不得重叠。
 */

#ifndef FFT_F32_65536_H
#define FFT_F32_65536_H

#include <stdint.h>

#include "arm_math.h"

/** 实数 FFT 固定输入长度。 */
#define FFT_F32_65536_LENGTH 65536u

/** 实数 FFT 执行状态。 */
typedef enum
{
    FFT_F32_65536_STATUS_OK = 0,
    FFT_F32_65536_STATUS_INVALID_ARGUMENT,
    FFT_F32_65536_STATUS_NONFINITE
} fft_f32_65536_status_t;

/**
 * @brief 对 65536 个实数样本执行 Hann 加窗的单精度浮点 FFT。
 * @param samples 第一个无符号 16 位样本的地址。
 * @param stride 相邻样本之间跨越的 uint16_t 元素数，必须大于零。
 * @param mean_code 整帧样本的平均原始码，用于去除直流分量。
 * @param packed_spectrum 65536 个 float32_t 的原地工作区兼输出区。
 * @return 成功返回 FFT_F32_65536_STATUS_OK；参数或数值异常返回对应状态。
 * @note 输出格式为 [DC, Nyquist, Re(1), Im(1), ... Re(32767), Im(32767)]。
 *       函数会完全覆盖 packed_spectrum，且不会访问任何外设。
 */
fft_f32_65536_status_t fft_f32_65536_forward(
    const uint16_t *samples,
    uint32_t stride,
    float32_t mean_code,
    float32_t *packed_spectrum);

#endif /* FFT_F32_65536_H */
