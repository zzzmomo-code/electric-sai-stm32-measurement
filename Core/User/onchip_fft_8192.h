/**
 * @file onchip_fft_8192.h
 * @brief 片上 ADC 单通道 8192 点实数 FFT 公共接口。
 *
 * 模块用途：对一帧等间隔的 12 位 ADC 样本去直流、加 Hann 窗并执行原地实数 FFT。
 * GPIO 引脚映射：无直接 GPIO 引脚。
 * 依赖的外设和 CubeIDE 配置：无外设依赖；使用 Cortex-M7 单精度 FPU 和 libm。
 * 初始化方法：无需初始化。
 * 调用方法：采样停止后调用 onchip_fft_8192_forward()。
 */

#ifndef ONCHIP_FFT_8192_H
#define ONCHIP_FFT_8192_H

#include <stdint.h>

/** 实数 FFT 固定输入长度。 */
#define ONCHIP_FFT_8192_LENGTH 8192u

/** FFT 执行状态。 */
typedef enum
{
    ONCHIP_FFT_8192_STATUS_OK = 0,
    ONCHIP_FFT_8192_STATUS_INVALID_ARGUMENT,
    ONCHIP_FFT_8192_STATUS_NONFINITE
} onchip_fft_8192_status_t;

/**
 * @brief 对 8192 个 ADC 样本执行 Hann 加窗的单精度实数 FFT。
 * @param samples 8192 个无符号 ADC 样本。
 * @param mean_code 整帧平均码，用于去除 1.65 V 直流偏置。
 * @param packed_spectrum 8192 个 float 的工作区和输出区。
 * @return 成功返回 ONCHIP_FFT_8192_STATUS_OK，否则返回错误状态。
 * @note 输出格式为 [DC, Nyquist, Re(1), Im(1), ... Re(4095), Im(4095)]。
 */
onchip_fft_8192_status_t onchip_fft_8192_forward(
    const uint16_t *samples,
    float mean_code,
    float *packed_spectrum);

#endif /* ONCHIP_FFT_8192_H */
