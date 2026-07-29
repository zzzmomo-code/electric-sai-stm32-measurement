/**
 * @file test_onchip_fft_8192.c
 * @brief 片上 8192 点 FFT 的主机数值测试。
 *
 * 模块用途：用整数频点和非整数频点合成信号验证实际 C FFT 内核。
 * GPIO 引脚映射：无直接 GPIO 引脚。
 * 依赖：主机 C 编译器和 libm，不依赖 STM32 HAL。
 * 运行方法：见 README_NO_FPGA.md 的“验证”章节。
 */

#include "onchip_fft_8192.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define TEST_PI 3.14159265358979323846

static uint16_t test_samples[ONCHIP_FFT_8192_LENGTH];
static float test_spectrum[ONCHIP_FFT_8192_LENGTH];

/**
 * @brief 返回频点功率。
 * @param bin 单边谱频点。
 * @return 实部平方与虚部平方之和。
 */
static float test_bin_power(uint32_t bin)
{
    float real = test_spectrum[2u * bin];
    float imaginary = test_spectrum[2u * bin + 1u];
    return real * real + imaginary * imaginary;
}

/**
 * @brief 在指定范围内寻找最大谱线。
 * @param first_bin 起始频点。
 * @param last_bin 结束频点。
 * @return 最大谱线频点。
 */
static uint32_t test_find_peak(uint32_t first_bin, uint32_t last_bin)
{
    uint32_t bin;
    uint32_t best_bin = first_bin;
    float best_power = test_bin_power(first_bin);

    for (bin = first_bin + 1u; bin <= last_bin; bin++)
    {
        float power = test_bin_power(bin);
        if (power > best_power)
        {
            best_power = power;
            best_bin = bin;
        }
    }
    return best_bin;
}

/**
 * @brief 使用对数功率抛物线插值细化谱峰位置。
 * @param bin 局部最大频点。
 * @return 非整数频点位置。
 */
static double test_refine_bin(uint32_t bin)
{
    double left = log((double)test_bin_power(bin - 1u) + 1.0e-30);
    double center = log((double)test_bin_power(bin) + 1.0e-30);
    double right = log((double)test_bin_power(bin + 1u) + 1.0e-30);
    double denominator = left - 2.0 * center + right;
    double delta = 0.0;

    if (fabs(denominator) > 1.0e-12)
    {
        delta = 0.5 * (left - right) / denominator;
    }
    if (delta > 0.5)
    {
        delta = 0.5;
    }
    else if (delta < -0.5)
    {
        delta = -0.5;
    }
    return (double)bin + delta;
}

/**
 * @brief 验证幅度较大的高次谐波不会破坏 FFT 谱峰位置与幅度标度。
 * @param 无。
 * @return 成功返回 0，失败返回非零。
 */
static int test_integer_bin_tones(void)
{
    const double fundamental_bin = 101.0;
    const double harmonic_bin = 303.0;
    const double fundamental_amplitude = 360.0;
    const double harmonic_amplitude = 620.0;
    uint32_t index;
    uint32_t fundamental_peak;
    uint32_t harmonic_peak;
    double fundamental_magnitude;
    double harmonic_magnitude;

    for (index = 0u; index < ONCHIP_FFT_8192_LENGTH; index++)
    {
        double sample = 2048.0
                        + fundamental_amplitude
                              * cos(2.0 * TEST_PI
                                    * fundamental_bin * (double)index
                                    / (double)ONCHIP_FFT_8192_LENGTH)
                        + harmonic_amplitude
                              * cos(2.0 * TEST_PI
                                    * harmonic_bin * (double)index
                                    / (double)ONCHIP_FFT_8192_LENGTH);
        test_samples[index] = (uint16_t)llround(sample);
    }

    if (onchip_fft_8192_forward(test_samples, 2048.0f,
                                test_spectrum)
        != ONCHIP_FFT_8192_STATUS_OK)
    {
        return 1;
    }

    fundamental_peak = test_find_peak(95u, 107u);
    harmonic_peak = test_find_peak(297u, 309u);
    fundamental_magnitude = sqrt((double)test_bin_power(
        fundamental_peak));
    harmonic_magnitude = sqrt((double)test_bin_power(harmonic_peak));

    if ((fundamental_peak != 101u) || (harmonic_peak != 303u))
    {
        return 2;
    }
    if (fabs(fundamental_magnitude
             - fundamental_amplitude
                   * (double)ONCHIP_FFT_8192_LENGTH / 4.0)
        > fundamental_amplitude
              * (double)ONCHIP_FFT_8192_LENGTH * 0.015)
    {
        return 3;
    }
    if (fabs(harmonic_magnitude
             - harmonic_amplitude
                   * (double)ONCHIP_FFT_8192_LENGTH / 4.0)
        > harmonic_amplitude
              * (double)ONCHIP_FFT_8192_LENGTH * 0.015)
    {
        return 4;
    }
    return 0;
}

/**
 * @brief 验证非整数频点的抛物线频率细化。
 * @param 无。
 * @return 成功返回 0，失败返回非零。
 */
static int test_fractional_bin_refinement(void)
{
    const double target_bin = 437.31;
    uint32_t index;
    uint32_t peak;
    double refined;

    for (index = 0u; index < ONCHIP_FFT_8192_LENGTH; index++)
    {
        double sample = 2048.0
                        + 500.0
                              * sin(2.0 * TEST_PI
                                    * target_bin * (double)index
                                    / (double)ONCHIP_FFT_8192_LENGTH);
        test_samples[index] = (uint16_t)llround(sample);
    }

    if (onchip_fft_8192_forward(test_samples, 2048.0f,
                                test_spectrum)
        != ONCHIP_FFT_8192_STATUS_OK)
    {
        return 1;
    }
    peak = test_find_peak(430u, 445u);
    refined = test_refine_bin(peak);
    return (fabs(refined - target_bin) <= 0.06) ? 0 : 2;
}

/**
 * @brief 运行全部 FFT 主机测试。
 * @param 无。
 * @return 全部通过返回 0。
 */
int main(void)
{
    int integer_result = test_integer_bin_tones();
    int fractional_result = test_fractional_bin_refinement();

    if ((integer_result != 0) || (fractional_result != 0))
    {
        fprintf(stderr, "FFT test failed: integer=%d fractional=%d\n",
                integer_result, fractional_result);
        return EXIT_FAILURE;
    }
    puts("FFT test passed");
    return EXIT_SUCCESS;
}
