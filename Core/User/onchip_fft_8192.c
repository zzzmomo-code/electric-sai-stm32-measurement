/**
 * @file onchip_fft_8192.c
 * @brief 8192 点 Hann 窗单精度实数 FFT 实现。
 *
 * 模块用途：用 4096 点原地基 2 复数 FFT 和实数后处理完成 8192 点变换，
 *          不保存窗函数表和旋转因子表，以 32 KiB 固定工作区控制 SRAM 占用。
 * GPIO 引脚映射：无直接 GPIO 引脚。
 * 依赖的外设和 CubeIDE 配置：无外设依赖；链接 libm。
 * 初始化方法：无需初始化。
 * 调用方法：由 onchip_measurement_process() 在 DMA 停止后调用。
 */

#include "onchip_fft_8192.h"

#include <math.h>

#define ONCHIP_FFT_PI 3.14159265358979323846f
#define ONCHIP_FFT_COMPLEX_LENGTH (ONCHIP_FFT_8192_LENGTH / 2u)
#define ONCHIP_FFT_NORMALIZE_INTERVAL 256u

/**
 * @brief 将递推得到的单位复数重新归一化。
 * @param real 实部地址。
 * @param imaginary 虚部地址。
 * @return 数值有效返回 1，否则返回 0。
 */
static uint8_t onchip_fft_normalize(float *real, float *imaginary)
{
    float magnitude_square = (*real * *real)
                             + (*imaginary * *imaginary);
    float inverse_magnitude;

    if ((!isfinite(magnitude_square)) || (magnitude_square <= 0.0f))
    {
        return 0u;
    }

    inverse_magnitude = 1.0f / sqrtf(magnitude_square);
    *real *= inverse_magnitude;
    *imaginary *= inverse_magnitude;
    return (uint8_t)(isfinite(*real) && isfinite(*imaginary));
}

/**
 * @brief 对 4096 个交错复数执行原地正向基 2 FFT。
 * @param data 8192 个 float 的交错复数工作区。
 * @return 成功返回 OK，数值异常返回 NONFINITE。
 */
static onchip_fft_8192_status_t onchip_fft_complex_forward(float *data)
{
    uint32_t index;
    uint32_t reversed = 0u;
    uint32_t stage_size;

    for (index = 1u; index < ONCHIP_FFT_COMPLEX_LENGTH; index++)
    {
        uint32_t bit = ONCHIP_FFT_COMPLEX_LENGTH >> 1u;

        while ((reversed & bit) != 0u)
        {
            reversed ^= bit;
            bit >>= 1u;
        }
        reversed ^= bit;

        if (index < reversed)
        {
            float temporary = data[2u * index];
            data[2u * index] = data[2u * reversed];
            data[2u * reversed] = temporary;
            temporary = data[2u * index + 1u];
            data[2u * index + 1u] =
                data[2u * reversed + 1u];
            data[2u * reversed + 1u] = temporary;
        }
    }

    for (stage_size = 2u;
         stage_size <= ONCHIP_FFT_COMPLEX_LENGTH;
         stage_size <<= 1u)
    {
        const uint32_t half_size = stage_size >> 1u;
        const float angle = -2.0f * ONCHIP_FFT_PI
                            / (float)stage_size;
        const float root_real = cosf(angle);
        const float root_imaginary = sinf(angle);
        float twiddle_real = 1.0f;
        float twiddle_imaginary = 0.0f;
        uint32_t column;

        for (column = 0u; column < half_size; column++)
        {
            uint32_t block;

            for (block = column;
                 block < ONCHIP_FFT_COMPLEX_LENGTH;
                 block += stage_size)
            {
                const uint32_t odd = block + half_size;
                const float odd_real = data[2u * odd];
                const float odd_imaginary = data[2u * odd + 1u];
                const float product_real =
                    twiddle_real * odd_real
                    - twiddle_imaginary * odd_imaginary;
                const float product_imaginary =
                    twiddle_real * odd_imaginary
                    + twiddle_imaginary * odd_real;
                const float even_real = data[2u * block];
                const float even_imaginary = data[2u * block + 1u];

                data[2u * block] = even_real + product_real;
                data[2u * block + 1u] =
                    even_imaginary + product_imaginary;
                data[2u * odd] = even_real - product_real;
                data[2u * odd + 1u] =
                    even_imaginary - product_imaginary;
            }

            {
                const float next_real =
                    twiddle_real * root_real
                    - twiddle_imaginary * root_imaginary;
                twiddle_imaginary =
                    twiddle_real * root_imaginary
                    + twiddle_imaginary * root_real;
                twiddle_real = next_real;
            }

            if (((column + 1u)
                 % ONCHIP_FFT_NORMALIZE_INTERVAL) == 0u)
            {
                if (onchip_fft_normalize(&twiddle_real,
                                         &twiddle_imaginary) == 0u)
                {
                    return ONCHIP_FFT_8192_STATUS_NONFINITE;
                }
            }
        }
    }

    return ONCHIP_FFT_8192_STATUS_OK;
}

/**
 * @brief 由两个对称的复数频点重建一个实数 FFT 频点。
 * @param first_real Z[k] 实部。
 * @param first_imaginary Z[k] 虚部。
 * @param mirror_real Z[M-k] 实部。
 * @param mirror_imaginary Z[M-k] 虚部。
 * @param twiddle_real exp(-j*2*pi*k/N) 实部。
 * @param twiddle_imaginary exp(-j*2*pi*k/N) 虚部。
 * @param output_real X[k] 实部输出地址。
 * @param output_imaginary X[k] 虚部输出地址。
 * @return 无。
 */
static void onchip_fft_reconstruct_bin(
    float first_real,
    float first_imaginary,
    float mirror_real,
    float mirror_imaginary,
    float twiddle_real,
    float twiddle_imaginary,
    float *output_real,
    float *output_imaginary)
{
    const float even_real = 0.5f * (first_real + mirror_real);
    const float even_imaginary =
        0.5f * (first_imaginary - mirror_imaginary);
    const float odd_real =
        0.5f * (first_imaginary + mirror_imaginary);
    const float odd_imaginary =
        -0.5f * (first_real - mirror_real);

    *output_real = even_real
                   + twiddle_real * odd_real
                   - twiddle_imaginary * odd_imaginary;
    *output_imaginary = even_imaginary
                        + twiddle_real * odd_imaginary
                        + twiddle_imaginary * odd_real;
}

onchip_fft_8192_status_t onchip_fft_8192_forward(
    const uint16_t *samples,
    float mean_code,
    float *packed_spectrum)
{
    const float hann_step =
        2.0f * ONCHIP_FFT_PI
        / (float)(ONCHIP_FFT_8192_LENGTH - 1u);
    const float hann_root_real = cosf(hann_step);
    const float hann_root_imaginary = sinf(hann_step);
    float hann_cosine = 1.0f;
    float hann_sine = 0.0f;
    uint32_t index;
    onchip_fft_8192_status_t status;

    if ((samples == NULL) || (packed_spectrum == NULL)
        || (!isfinite(mean_code)))
    {
        return ONCHIP_FFT_8192_STATUS_INVALID_ARGUMENT;
    }

    for (index = 0u; index < ONCHIP_FFT_8192_LENGTH; index++)
    {
        const float window = 0.5f - 0.5f * hann_cosine;
        const float sample =
            ((float)samples[index] - mean_code) * window;
        const float next_cosine =
            hann_cosine * hann_root_real
            - hann_sine * hann_root_imaginary;

        if (!isfinite(sample))
        {
            return ONCHIP_FFT_8192_STATUS_NONFINITE;
        }
        packed_spectrum[index] = sample;

        hann_sine = hann_cosine * hann_root_imaginary
                    + hann_sine * hann_root_real;
        hann_cosine = next_cosine;
        if (((index + 1u)
             % ONCHIP_FFT_NORMALIZE_INTERVAL) == 0u)
        {
            if (onchip_fft_normalize(&hann_cosine,
                                     &hann_sine) == 0u)
            {
                return ONCHIP_FFT_8192_STATUS_NONFINITE;
            }
        }
    }

    status = onchip_fft_complex_forward(packed_spectrum);
    if (status != ONCHIP_FFT_8192_STATUS_OK)
    {
        return status;
    }

    {
        const float zero_real = packed_spectrum[0];
        const float zero_imaginary = packed_spectrum[1];
        packed_spectrum[0] = zero_real + zero_imaginary;
        packed_spectrum[1] = zero_real - zero_imaginary;
    }

    {
        const float angle_step =
            -2.0f * ONCHIP_FFT_PI
            / (float)ONCHIP_FFT_8192_LENGTH;
        const float root_real = cosf(angle_step);
        const float root_imaginary = sinf(angle_step);
        float twiddle_real = root_real;
        float twiddle_imaginary = root_imaginary;

        for (index = 1u;
             index < (ONCHIP_FFT_COMPLEX_LENGTH / 2u);
             index++)
        {
            const uint32_t mirror =
                ONCHIP_FFT_COMPLEX_LENGTH - index;
            const float first_real =
                packed_spectrum[2u * index];
            const float first_imaginary =
                packed_spectrum[2u * index + 1u];
            const float mirror_real =
                packed_spectrum[2u * mirror];
            const float mirror_imaginary =
                packed_spectrum[2u * mirror + 1u];
            float first_output_real;
            float first_output_imaginary;
            float mirror_output_real;
            float mirror_output_imaginary;
            float next_real;

            onchip_fft_reconstruct_bin(
                first_real, first_imaginary,
                mirror_real, mirror_imaginary,
                twiddle_real, twiddle_imaginary,
                &first_output_real, &first_output_imaginary);
            onchip_fft_reconstruct_bin(
                mirror_real, mirror_imaginary,
                first_real, first_imaginary,
                -twiddle_real, twiddle_imaginary,
                &mirror_output_real, &mirror_output_imaginary);

            packed_spectrum[2u * index] =
                first_output_real;
            packed_spectrum[2u * index + 1u] =
                first_output_imaginary;
            packed_spectrum[2u * mirror] =
                mirror_output_real;
            packed_spectrum[2u * mirror + 1u] =
                mirror_output_imaginary;

            next_real = twiddle_real * root_real
                        - twiddle_imaginary * root_imaginary;
            twiddle_imaginary =
                twiddle_real * root_imaginary
                + twiddle_imaginary * root_real;
            twiddle_real = next_real;
            if (((index + 1u)
                 % ONCHIP_FFT_NORMALIZE_INTERVAL) == 0u)
            {
                if (onchip_fft_normalize(&twiddle_real,
                                         &twiddle_imaginary) == 0u)
                {
                    return ONCHIP_FFT_8192_STATUS_NONFINITE;
                }
            }
        }
    }

    index = ONCHIP_FFT_COMPLEX_LENGTH / 2u;
    packed_spectrum[2u * index + 1u] =
        -packed_spectrum[2u * index + 1u];

    for (index = 0u; index < ONCHIP_FFT_8192_LENGTH; index++)
    {
        if (!isfinite(packed_spectrum[index]))
        {
            return ONCHIP_FFT_8192_STATUS_NONFINITE;
        }
    }

    return ONCHIP_FFT_8192_STATUS_OK;
}
