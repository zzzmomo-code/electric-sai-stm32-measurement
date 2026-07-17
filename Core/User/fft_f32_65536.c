/**
 * @file fft_f32_65536.c
 * @brief 65536 点 Hann 加窗 F32 实数 FFT 实现。
 *
 * 模块用途：以 32768 点原地基 2 复数 FFT 和实数后处理完成 65536 点变换，
 * 不保存完整窗函数表或旋转因子表，以固定 256 KiB 工作区控制 SRAM 占用。
 * GPIO 引脚映射：无直接 GPIO 引脚。
 * 依赖的外设和 CubeIDE 配置：无外设依赖；使用单精度 FPU 和 libm。
 * 初始化方法：无需初始化。
 * 调用方法：由 measurement_fft_process() 在整帧采样停止后顺序处理两个通道。
 */

#include "fft_f32_65536.h"

#include <math.h>

#define FFT_F32_65536_PI 3.14159265358979323846f
#define FFT_F32_65536_COMPLEX_LENGTH (FFT_F32_65536_LENGTH / 2u)
#define FFT_F32_65536_HANN_RENORMALIZE_INTERVAL 1024u
#define FFT_F32_65536_TWIDDLE_RENORMALIZE_INTERVAL 256u

/**
 * @brief 将复数乘法递推量归一化到单位圆。
 * @param real 实部地址。
 * @param imaginary 虚部地址。
 * @return 归一化成功返回 1，否则返回 0。
 * @note 仅修正浮点递推累计误差，不改变旋转方向。
 */
static uint8_t fft_f32_65536_normalize(float32_t *real,
                                        float32_t *imaginary)
{
    float32_t magnitude_square;
    float32_t inverse_magnitude;

    magnitude_square = (*real * *real) + (*imaginary * *imaginary);
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
 * @brief 对工作区中的 32768 个交错复数执行原地正向基 2 FFT。
 * @param data 65536 个 float32_t 的交错复数工作区。
 * @return 成功返回 OK，出现非有限数返回 NONFINITE。
 * @note 先执行位反转，再完成 15 级蝶形运算；正向变换不缩放。
 */
static fft_f32_65536_status_t fft_f32_65536_complex_forward(
    float32_t *data)
{
    uint32_t index;
    uint32_t reversed = 0u;
    uint32_t stage_size;

    for (index = 1u; index < FFT_F32_65536_COMPLEX_LENGTH; index++)
    {
        uint32_t bit = FFT_F32_65536_COMPLEX_LENGTH >> 1u;
        float32_t temporary;

        while ((reversed & bit) != 0u)
        {
            reversed ^= bit;
            bit >>= 1u;
        }
        reversed ^= bit;

        if (index < reversed)
        {
            temporary = data[2u * index];
            data[2u * index] = data[2u * reversed];
            data[2u * reversed] = temporary;
            temporary = data[2u * index + 1u];
            data[2u * index + 1u] = data[2u * reversed + 1u];
            data[2u * reversed + 1u] = temporary;
        }
    }

    for (stage_size = 2u;
         stage_size <= FFT_F32_65536_COMPLEX_LENGTH;
         stage_size <<= 1u)
    {
        const uint32_t half_size = stage_size >> 1u;
        const float32_t angle = -2.0f * FFT_F32_65536_PI
                                / (float32_t)stage_size;
        const float32_t root_real = cosf(angle);
        const float32_t root_imaginary = sinf(angle);
        float32_t twiddle_real = 1.0f;
        float32_t twiddle_imaginary = 0.0f;
        uint32_t column;

        for (column = 0u; column < half_size; column++)
        {
            uint32_t block;

            for (block = column;
                 block < FFT_F32_65536_COMPLEX_LENGTH;
                 block += stage_size)
            {
                const uint32_t odd = block + half_size;
                const float32_t odd_real = data[2u * odd];
                const float32_t odd_imaginary = data[2u * odd + 1u];
                const float32_t product_real =
                    twiddle_real * odd_real
                    - twiddle_imaginary * odd_imaginary;
                const float32_t product_imaginary =
                    twiddle_real * odd_imaginary
                    + twiddle_imaginary * odd_real;
                const float32_t even_real = data[2u * block];
                const float32_t even_imaginary = data[2u * block + 1u];

                data[2u * block] = even_real + product_real;
                data[2u * block + 1u] =
                    even_imaginary + product_imaginary;
                data[2u * odd] = even_real - product_real;
                data[2u * odd + 1u] =
                    even_imaginary - product_imaginary;
            }

            {
                const float32_t next_real =
                    twiddle_real * root_real
                    - twiddle_imaginary * root_imaginary;
                twiddle_imaginary = twiddle_real * root_imaginary
                                    + twiddle_imaginary * root_real;
                twiddle_real = next_real;
            }

            if (((column + 1u)
                 % FFT_F32_65536_TWIDDLE_RENORMALIZE_INTERVAL) == 0u)
            {
                if (fft_f32_65536_normalize(&twiddle_real,
                                             &twiddle_imaginary) == 0u)
                {
                    return FFT_F32_65536_STATUS_NONFINITE;
                }
            }
        }
    }

    return FFT_F32_65536_STATUS_OK;
}

/**
 * @brief 由两个对称的复数 FFT 频点计算一个实数 FFT 频点。
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
static void fft_f32_65536_reconstruct_bin(
    float32_t first_real,
    float32_t first_imaginary,
    float32_t mirror_real,
    float32_t mirror_imaginary,
    float32_t twiddle_real,
    float32_t twiddle_imaginary,
    float32_t *output_real,
    float32_t *output_imaginary)
{
    const float32_t even_real = 0.5f * (first_real + mirror_real);
    const float32_t even_imaginary =
        0.5f * (first_imaginary - mirror_imaginary);
    const float32_t odd_real =
        0.5f * (first_imaginary + mirror_imaginary);
    const float32_t odd_imaginary =
        -0.5f * (first_real - mirror_real);

    *output_real = even_real
                   + twiddle_real * odd_real
                   - twiddle_imaginary * odd_imaginary;
    *output_imaginary = even_imaginary
                        + twiddle_real * odd_imaginary
                        + twiddle_imaginary * odd_real;
}

fft_f32_65536_status_t fft_f32_65536_forward(
    const uint16_t *samples,
    uint32_t stride,
    float32_t mean_code,
    float32_t *packed_spectrum)
{
    const float32_t hann_step =
        2.0f * FFT_F32_65536_PI
        / (float32_t)(FFT_F32_65536_LENGTH - 1u);
    const float32_t hann_root_real = cosf(hann_step);
    const float32_t hann_root_imaginary = sinf(hann_step);
    float32_t hann_cosine = 1.0f;
    float32_t hann_sine = 0.0f;
    uint32_t index;
    fft_f32_65536_status_t status;

    if ((samples == 0) || (packed_spectrum == 0) || (stride == 0u)
        || (!isfinite(mean_code)))
    {
        return FFT_F32_65536_STATUS_INVALID_ARGUMENT;
    }

    for (index = 0u; index < FFT_F32_65536_LENGTH; index++)
    {
        const float32_t window = 0.5f - 0.5f * hann_cosine;
        const float32_t sample =
            ((float32_t)samples[index * stride] - mean_code) * window;
        const float32_t next_cosine =
            hann_cosine * hann_root_real
            - hann_sine * hann_root_imaginary;

        if (!isfinite(sample))
        {
            return FFT_F32_65536_STATUS_NONFINITE;
        }
        packed_spectrum[index] = sample;

        hann_sine = hann_cosine * hann_root_imaginary
                    + hann_sine * hann_root_real;
        hann_cosine = next_cosine;
        if (((index + 1u)
             % FFT_F32_65536_HANN_RENORMALIZE_INTERVAL) == 0u)
        {
            if (fft_f32_65536_normalize(&hann_cosine, &hann_sine) == 0u)
            {
                return FFT_F32_65536_STATUS_NONFINITE;
            }
        }
    }

    status = fft_f32_65536_complex_forward(packed_spectrum);
    if (status != FFT_F32_65536_STATUS_OK)
    {
        return status;
    }

    {
        const float32_t zero_real = packed_spectrum[0];
        const float32_t zero_imaginary = packed_spectrum[1];
        packed_spectrum[0] = zero_real + zero_imaginary;
        packed_spectrum[1] = zero_real - zero_imaginary;
    }

    {
        const float32_t angle_step = -2.0f * FFT_F32_65536_PI
                                     / (float32_t)FFT_F32_65536_LENGTH;
        const float32_t root_real = cosf(angle_step);
        const float32_t root_imaginary = sinf(angle_step);
        float32_t twiddle_real = root_real;
        float32_t twiddle_imaginary = root_imaginary;

        for (index = 1u;
             index < (FFT_F32_65536_COMPLEX_LENGTH / 2u);
             index++)
        {
            const uint32_t mirror = FFT_F32_65536_COMPLEX_LENGTH - index;
            const float32_t first_real = packed_spectrum[2u * index];
            const float32_t first_imaginary =
                packed_spectrum[2u * index + 1u];
            const float32_t mirror_real = packed_spectrum[2u * mirror];
            const float32_t mirror_imaginary =
                packed_spectrum[2u * mirror + 1u];
            float32_t first_output_real;
            float32_t first_output_imaginary;
            float32_t mirror_output_real;
            float32_t mirror_output_imaginary;
            float32_t next_real;

            fft_f32_65536_reconstruct_bin(first_real,
                                          first_imaginary,
                                          mirror_real,
                                          mirror_imaginary,
                                          twiddle_real,
                                          twiddle_imaginary,
                                          &first_output_real,
                                          &first_output_imaginary);
            fft_f32_65536_reconstruct_bin(
                mirror_real,
                mirror_imaginary,
                first_real,
                first_imaginary,
                -twiddle_real,
                twiddle_imaginary,
                &mirror_output_real,
                &mirror_output_imaginary);

            packed_spectrum[2u * index] = first_output_real;
            packed_spectrum[2u * index + 1u] = first_output_imaginary;
            packed_spectrum[2u * mirror] = mirror_output_real;
            packed_spectrum[2u * mirror + 1u] = mirror_output_imaginary;

            next_real = twiddle_real * root_real
                        - twiddle_imaginary * root_imaginary;
            twiddle_imaginary = twiddle_real * root_imaginary
                                + twiddle_imaginary * root_real;
            twiddle_real = next_real;
            if (((index + 1u)
                 % FFT_F32_65536_TWIDDLE_RENORMALIZE_INTERVAL) == 0u)
            {
                if (fft_f32_65536_normalize(&twiddle_real,
                                             &twiddle_imaginary) == 0u)
                {
                    return FFT_F32_65536_STATUS_NONFINITE;
                }
            }
        }
    }

    index = FFT_F32_65536_COMPLEX_LENGTH / 2u;
    {
        const float32_t real = packed_spectrum[2u * index];
        const float32_t imaginary = packed_spectrum[2u * index + 1u];
        packed_spectrum[2u * index] = real;
        packed_spectrum[2u * index + 1u] = -imaginary;
    }

    for (index = 0u; index < FFT_F32_65536_LENGTH; index++)
    {
        if (!isfinite(packed_spectrum[index]))
        {
            return FFT_F32_65536_STATUS_NONFINITE;
        }
    }

    return FFT_F32_65536_STATUS_OK;
}
