#if defined(FREQUENCY_ESTIMATOR_HOST_TEST)
#include "signal_separation_config.h"
#include "frequency_estimator.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#else
#include "system.h"
#endif

/*
 * 模块用途：高精度首次判频，使用 32768 点连续采样、Hann 窗、基 2 FFT、
 *           对数谱峰插值和前后半段相位斜率细化。
 * GPIO 映射：无直接 GPIO 引脚。
 * 外设依赖：无直接外设依赖；由 signal_separation.c 提供已经完成 Cache 维护的 ADC 副本。
 * 初始化方法：signal_separation_start() 或重新识别时调用 frequency_estimator_reset()。
 * 调用方法：frequency_estimator_push() 每次接收一个连续 ADC DMA 半区。
 */

/*
 * 本模块只负责“首次频率初值”，不直接完成持续锁相：
 * 连续样点 -> 去直流 -> Hann（汉宁）窗 -> FFT（快速傅里叶变换）
 * -> 主峰三点插值 -> 前后半段相位斜率细化 -> 毫赫兹频率。
 * 返回后由 signal_separation.c 测量当前短帧相位，并交给 Q32 NCO/PLL 闭环。
 */

#if (SIGSEP_FREQUENCY_MODE == SIGSEP_FREQ_MODE_PRECISE_FFT)

#define frequency_estimator_pi_f 3.14159265358979323846f
#define frequency_estimator_pi_d 3.14159265358979323846

_Static_assert((SIGSEP_PRECISE_FFT_LEN &
                (SIGSEP_PRECISE_FFT_LEN - 1U)) == 0U,
               "precise FFT length must be a power of two");
_Static_assert((SIGSEP_PRECISE_FFT_LEN %
                SIGSEP_ADC_DMA_HALF_LEN) == 0U,
               "precise FFT capture must contain complete DMA halves");
_Static_assert(SIGSEP_PRECISE_FREQ_MIN_HZ > 0U,
               "precise minimum frequency must be positive");
_Static_assert(SIGSEP_PRECISE_FREQ_MIN_HZ <
               SIGSEP_PRECISE_FREQ_MAX_HZ,
               "precise frequency range is invalid");
_Static_assert(SIGSEP_PRECISE_FREQ_MAX_HZ <
               (SIGSEP_SAMPLE_RATE_HZ / 2U),
               "precise maximum frequency must be below Nyquist");

/*
 * 这些缓冲区只由 CPU 使用，放在 D1 SRAM 的普通 .bss 中，不参与 DMA。
 * 16 位捕获记录占 64 KiB，两组浮点 FFT 数组共占 256 KiB。
 */
static uint16_t frequency_capture[SIGSEP_PRECISE_FFT_LEN];
static float fft_real[SIGSEP_PRECISE_FFT_LEN];
static float fft_imag[SIGSEP_PRECISE_FFT_LEN];
static uint32_t frequency_capture_count;

/**
 * @brief 交换两个浮点数。
 * @param left 第一个数。
 * @param right 第二个数。
 * @return 无。
 */
static void swap_float(float *left, float *right)
{
  float temporary = *left;
  *left = *right;
  *right = temporary;
}

/**
 * @brief 对复数数组执行原地基 2 时间抽取 FFT。
 * @param real 复数实部数组。
 * @param imag 复数虚部数组。
 * @param length FFT 长度，必须是 2 的幂。
 * @return 无。
 */
static void fft_radix2(float *real, float *imag, uint32_t length)
{
  uint32_t index;
  uint32_t reverse = 0U;
  uint32_t stage_length;

  for (index = 1U; index < length; index++)
  {
    uint32_t bit = length >> 1U;

    while ((reverse & bit) != 0U)
    {
      reverse ^= bit;
      bit >>= 1U;
    }
    reverse ^= bit;

    if (index < reverse)
    {
      swap_float(&real[index], &real[reverse]);
      swap_float(&imag[index], &imag[reverse]);
    }
  }

  for (stage_length = 2U; stage_length <= length;
       stage_length <<= 1U)
  {
    uint32_t half_length = stage_length >> 1U;
    float angle = -2.0f * frequency_estimator_pi_f /
                  (float)stage_length;
    float step_real = cosf(angle);
    float step_imag = sinf(angle);
    uint32_t block;

    for (block = 0U; block < length; block += stage_length)
    {
      float twiddle_real = 1.0f;
      float twiddle_imag = 0.0f;
      uint32_t offset;

      for (offset = 0U; offset < half_length; offset++)
      {
        uint32_t even = block + offset;
        uint32_t odd = even + half_length;
        float odd_real =
          (twiddle_real * real[odd]) - (twiddle_imag * imag[odd]);
        float odd_imag =
          (twiddle_real * imag[odd]) + (twiddle_imag * real[odd]);
        float next_twiddle_real =
          (twiddle_real * step_real) - (twiddle_imag * step_imag);

        real[odd] = real[even] - odd_real;
        imag[odd] = imag[even] - odd_imag;
        real[even] += odd_real;
        imag[even] += odd_imag;

        twiddle_imag =
          (twiddle_real * step_imag) + (twiddle_imag * step_real);
        twiddle_real = next_twiddle_real;

        /*
         * 最大级需要连续递推 16384 次。M7 单精度乘加会让旋转因子模长缓慢
         * 偏离 1，每 256 点归一化一次，避免长 FFT 的谱峰位置和幅值受累积
         * 舍入误差影响。
         */
        if ((offset & 255U) == 255U)
        {
          float twiddle_norm =
            1.0f / sqrtf((twiddle_real * twiddle_real) +
                         (twiddle_imag * twiddle_imag));
          twiddle_real *= twiddle_norm;
          twiddle_imag *= twiddle_norm;
        }
      }
    }

    if (stage_length == length)
    {
      break;
    }
  }
}

/**
 * @brief 返回单个 FFT 频点的功率。
 * @param bin 频点索引。
 * @return 实部平方与虚部平方之和。
 */
static float bin_power(uint32_t bin)
{
  return (fft_real[bin] * fft_real[bin]) +
         (fft_imag[bin] * fft_imag[bin]);
}

/**
 * @brief 判断候选频点是否落在已选主峰的保护区内。
 * @param candidate 候选频点。
 * @param blocked 已选主峰；UINT32_MAX 表示不屏蔽。
 * @param guard_bins 双信号最小频率间隔对应的频点数。
 * @return 允许返回 1，落入保护区返回 0。
 */
static uint8_t peak_bin_allowed(uint32_t candidate, uint32_t blocked,
                                uint32_t guard_bins)
{
  uint32_t distance;

  if (blocked == UINT32_MAX)
  {
    return 1U;
  }

  distance = (candidate >= blocked) ?
             (candidate - blocked) : (blocked - candidate);
  return (distance >= guard_bins) ? 1U : 0U;
}

/**
 * @brief 在指定范围寻找最强局部谱峰。
 * @param first_bin 搜索起始频点。
 * @param last_bin 搜索结束频点。
 * @param blocked_bin 需要屏蔽的另一主峰，UINT32_MAX 表示无。
 * @param guard_bins 屏蔽保护区半径。
 * @param peak_power_out 非空峰值功率输出。
 * @return 找到的频点；不存在有效局部峰时返回 UINT32_MAX。
 */
static uint32_t find_strongest_peak(uint32_t first_bin, uint32_t last_bin,
                                    uint32_t blocked_bin,
                                    uint32_t guard_bins,
                                    float *peak_power_out)
{
  uint32_t bin;
  uint32_t best_bin = UINT32_MAX;
  float best_power = 0.0f;

  for (bin = first_bin; bin <= last_bin; bin++)
  {
    float current_power;

    if (peak_bin_allowed(bin, blocked_bin, guard_bins) == 0U)
    {
      continue;
    }

    current_power = bin_power(bin);
    if ((current_power >= bin_power(bin - 1U)) &&
        (current_power > bin_power(bin + 1U)) &&
        (current_power > best_power))
    {
      best_power = current_power;
      best_bin = bin;
    }
  }

  *peak_power_out = best_power;
  return best_bin;
}

/**
 * @brief 用相邻三个对数功率点做抛物线峰顶插值并换算为毫赫兹。
 * @param peak_bin 整数谱峰索引。
 * @return 插值后的频率，单位 0.001 Hz。
 */
static uint32_t interpolate_peak_millihz(uint32_t peak_bin)
{
  float power_left = bin_power(peak_bin - 1U);
  float power_center = bin_power(peak_bin);
  float power_right = bin_power(peak_bin + 1U);
  float log_left = logf(power_left + 1.0f);
  float log_center = logf(power_center + 1.0f);
  float log_right = logf(power_right + 1.0f);
  float denominator =
    log_left - (2.0f * log_center) + log_right;
  float offset = 0.0f;
  double frequency_millihz;

  if (fabsf(denominator) > 1.0e-12f)
  {
    offset = 0.5f * (log_left - log_right) / denominator;
    if (offset > 0.5f)
    {
      offset = 0.5f;
    }
    else if (offset < -0.5f)
    {
      offset = -0.5f;
    }
  }

  frequency_millihz =
    ((double)peak_bin + (double)offset) *
    (double)SIGSEP_SAMPLE_RATE_HZ * 1000.0 /
    (double)SIGSEP_PRECISE_FFT_LEN;

  if (frequency_millihz <
      ((double)SIGSEP_PRECISE_FREQ_MIN_HZ * 1000.0))
  {
    frequency_millihz =
      (double)SIGSEP_PRECISE_FREQ_MIN_HZ * 1000.0;
  }
  else if (frequency_millihz >
           ((double)SIGSEP_PRECISE_FREQ_MAX_HZ * 1000.0))
  {
    frequency_millihz =
      (double)SIGSEP_PRECISE_FREQ_MAX_HZ * 1000.0;
  }

  return (uint32_t)(frequency_millihz + 0.5);
}

/**
 * @brief 利用长记录前后两半的基波相位差进一步细化 FFT 初值。
 * @param initial_millihz FFT 峰值插值得到的无歧义初值。
 * @return 相位斜率细化后的频率，单位 0.001 Hz。
 * @note FFT 初值误差远小于半段相位差的无歧义范围，因此相位差可提供比频点间隔
 *       更细的估计；两段使用相同 Hann 窗以降低另一分量和谐波的泄漏影响。
 */
static uint32_t refine_frequency_phase_millihz(uint32_t initial_millihz)
{
  const uint32_t half_length = SIGSEP_PRECISE_FFT_LEN / 2U;
  float frequency_hz = (float)initial_millihz * 0.001f;
  float oscillator_angle =
    2.0f * frequency_estimator_pi_f * frequency_hz /
    (float)SIGSEP_SAMPLE_RATE_HZ;
  float oscillator_step_real = cosf(oscillator_angle);
  float oscillator_step_imag = sinf(oscillator_angle);
  float window_angle =
    2.0f * frequency_estimator_pi_f / (float)(half_length - 1U);
  float window_step_real = cosf(window_angle);
  float window_step_imag = sinf(window_angle);
  float phase[2];
  uint64_t sample_sum = 0ULL;
  float sample_mean;
  uint32_t block;

  for (block = 0U; block < SIGSEP_PRECISE_FFT_LEN; block++)
  {
    sample_sum += frequency_capture[block];
  }
  sample_mean =
    (float)((double)sample_sum / (double)SIGSEP_PRECISE_FFT_LEN);

  for (block = 0U; block < 2U; block++)
  {
    float oscillator_real = 1.0f;
    float oscillator_imag = 0.0f;
    float window_real = 1.0f;
    float window_imag = 0.0f;
    float correlation_real = 0.0f;
    float correlation_imag = 0.0f;
    uint32_t index;

    for (index = 0U; index < half_length; index++)
    {
      float sample =
        (float)frequency_capture[(block * half_length) + index] -
        sample_mean;
      float window = 0.5f - (0.5f * window_real);
      float weighted_sample = sample * window;
      float next_oscillator_real =
        (oscillator_real * oscillator_step_real) -
        (oscillator_imag * oscillator_step_imag);
      float next_window_real =
        (window_real * window_step_real) -
        (window_imag * window_step_imag);

      correlation_real += weighted_sample * oscillator_real;
      correlation_imag -= weighted_sample * oscillator_imag;

      oscillator_imag =
        (oscillator_real * oscillator_step_imag) +
        (oscillator_imag * oscillator_step_real);
      oscillator_real = next_oscillator_real;
      window_imag =
        (window_real * window_step_imag) +
        (window_imag * window_step_real);
      window_real = next_window_real;

      /*
       * 单精度递推每 256 点归一化一次，限制长记录累计的幅值漂移，同时避免
       * 在每个样点调用代价较高的三角函数。
       */
      if ((index & 255U) == 255U)
      {
        float oscillator_norm =
          1.0f / sqrtf((oscillator_real * oscillator_real) +
                       (oscillator_imag * oscillator_imag));
        float window_norm =
          1.0f / sqrtf((window_real * window_real) +
                       (window_imag * window_imag));
        oscillator_real *= oscillator_norm;
        oscillator_imag *= oscillator_norm;
        window_real *= window_norm;
        window_imag *= window_norm;
      }
    }

    phase[block] = atan2f(correlation_imag, correlation_real);
  }

  {
    double expected_phase =
      2.0 * frequency_estimator_pi_d *
      ((double)initial_millihz * 0.001) *
      (double)half_length / (double)SIGSEP_SAMPLE_RATE_HZ;
    double phase_error =
      remainder((double)phase[1] - (double)phase[0] - expected_phase,
                2.0 * frequency_estimator_pi_d);
    double refined_millihz =
      (double)initial_millihz +
      phase_error * (double)SIGSEP_SAMPLE_RATE_HZ * 1000.0 /
      (2.0 * frequency_estimator_pi_d * (double)half_length);

    if (refined_millihz <
        ((double)SIGSEP_PRECISE_FREQ_MIN_HZ * 1000.0))
    {
      refined_millihz =
        (double)SIGSEP_PRECISE_FREQ_MIN_HZ * 1000.0;
    }
    else if (refined_millihz >
             ((double)SIGSEP_PRECISE_FREQ_MAX_HZ * 1000.0))
    {
      refined_millihz =
        (double)SIGSEP_PRECISE_FREQ_MAX_HZ * 1000.0;
    }

    return (uint32_t)(refined_millihz + 0.5);
  }
}

/**
 * @brief 对完整长记录去直流、加 Hann 窗并执行 FFT。
 * @param 无。
 * @return 无。
 */
static void prepare_spectrum(void)
{
  uint64_t sum = 0ULL;
  float mean;
  float window_angle =
    2.0f * frequency_estimator_pi_f /
    (float)(SIGSEP_PRECISE_FFT_LEN - 1U);
  float window_step_real = cosf(window_angle);
  float window_step_imag = sinf(window_angle);
  float window_real = 1.0f;
  float window_imag = 0.0f;
  uint32_t index;

  for (index = 0U; index < SIGSEP_PRECISE_FFT_LEN; index++)
  {
    sum += frequency_capture[index];
  }
  mean = (float)((double)sum / (double)SIGSEP_PRECISE_FFT_LEN);

  for (index = 0U; index < SIGSEP_PRECISE_FFT_LEN; index++)
  {
    float window = 0.5f - (0.5f * window_real);
    float next_window_real =
      (window_real * window_step_real) -
      (window_imag * window_step_imag);

    fft_real[index] =
      ((float)frequency_capture[index] - mean) * window;
    fft_imag[index] = 0.0f;

    window_imag =
      (window_real * window_step_imag) +
      (window_imag * window_step_real);
    window_real = next_window_real;

    /* 限制 32768 点 Hann 窗单精度递推的模长漂移。 */
    if ((index & 255U) == 255U)
    {
      float window_norm =
        1.0f / sqrtf((window_real * window_real) +
                     (window_imag * window_imag));
      window_real *= window_norm;
      window_imag *= window_norm;
    }
  }

  fft_radix2(fft_real, fft_imag, SIGSEP_PRECISE_FFT_LEN);
}

void frequency_estimator_reset(void)
{
  frequency_capture_count = 0U;
}

uint8_t frequency_estimator_push(const uint16_t *samples,
                                 uint32_t sample_count,
                                 uint8_t requested_components,
                                 frequency_estimator_result_t *result)
{
  uint32_t remaining;
  uint32_t first_bin;
  uint32_t last_bin;
  uint32_t guard_bins;
  uint32_t peak_bin[2] = {UINT32_MAX, UINT32_MAX};
  float peak_power[2] = {0.0f, 0.0f};
  float minimum_peak_power;
  uint32_t index;

  if ((samples == NULL) || (result == NULL) ||
      ((requested_components != 1U) && (requested_components != 2U)))
  {
    return 0U;
  }

  remaining = SIGSEP_PRECISE_FFT_LEN - frequency_capture_count;
  if (sample_count > remaining)
  {
    frequency_estimator_reset();
    return 0U;
  }

  memcpy(&frequency_capture[frequency_capture_count], samples,
         sample_count * sizeof(uint16_t));
  frequency_capture_count += sample_count;
  if (frequency_capture_count < SIGSEP_PRECISE_FFT_LEN)
  {
    return 0U;
  }
  frequency_capture_count = 0U;

  prepare_spectrum();

  first_bin =
    (uint32_t)(((uint64_t)SIGSEP_PRECISE_FREQ_MIN_HZ *
                SIGSEP_PRECISE_FFT_LEN) /
               SIGSEP_SAMPLE_RATE_HZ);
  last_bin =
    (uint32_t)((((uint64_t)SIGSEP_PRECISE_FREQ_MAX_HZ *
                 SIGSEP_PRECISE_FFT_LEN) +
                SIGSEP_SAMPLE_RATE_HZ - 1ULL) /
               SIGSEP_SAMPLE_RATE_HZ);
  if (first_bin < 1U)
  {
    first_bin = 1U;
  }
  if (last_bin >= ((SIGSEP_PRECISE_FFT_LEN / 2U) - 1U))
  {
    last_bin = (SIGSEP_PRECISE_FFT_LEN / 2U) - 2U;
  }

  guard_bins =
    (uint32_t)(((uint64_t)SIGSEP_DUAL_MIN_SEPARATION_HZ *
                SIGSEP_PRECISE_FFT_LEN) /
               SIGSEP_SAMPLE_RATE_HZ);
  if (guard_bins < 1U)
  {
    guard_bins = 1U;
  }
  peak_bin[0] = find_strongest_peak(first_bin, last_bin, UINT32_MAX,
                                    guard_bins, &peak_power[0]);

  /*
   * Hann 窗的相干增益约为 0.5；单边正弦峰值约为 amplitude*N/4。
   * 先换算为功率阈值，避免把底噪当作输入分量。
   */
  minimum_peak_power =
    SIGSEP_MIN_VALID_ADC_AMP * (float)SIGSEP_PRECISE_FFT_LEN * 0.25f;
  minimum_peak_power *= minimum_peak_power;
  if ((peak_bin[0] == UINT32_MAX) ||
      (peak_power[0] < minimum_peak_power))
  {
    return 0U;
  }

  if (requested_components == 2U)
  {
    peak_bin[1] = find_strongest_peak(first_bin, last_bin, peak_bin[0],
                                      guard_bins, &peak_power[1]);
    if ((peak_bin[1] == UINT32_MAX) ||
        (peak_power[1] < minimum_peak_power))
    {
      return 0U;
    }
  }

  result->component_count = requested_components;
  for (index = 0U; index < requested_components; index++)
  {
    result->frequency_millihz[index] =
      refine_frequency_phase_millihz(
        interpolate_peak_millihz(peak_bin[index]));
  }

  if ((requested_components == 2U) &&
      (result->frequency_millihz[0] > result->frequency_millihz[1]))
  {
    uint32_t temporary = result->frequency_millihz[0];
    result->frequency_millihz[0] = result->frequency_millihz[1];
    result->frequency_millihz[1] = temporary;
  }

  /*
   * FFT 保护区只用于避免第二次峰值搜索重复选中同一主瓣；最终仍以插值和
   * 相位斜率细化后的物理频率复核最小间隔。使用向下取整的保护频点数，可让
   * 恰好相差 4 kHz、但峰值整数 bin 只相差 52 的有效双音进入细化步骤。
   */
  if ((requested_components == 2U) &&
      ((result->frequency_millihz[1] -
        result->frequency_millihz[0]) <
       ((uint32_t)SIGSEP_DUAL_MIN_SEPARATION_HZ * 1000U)))
  {
    return 0U;
  }

  return 1U;
}

#else

void frequency_estimator_reset(void)
{
}

uint8_t frequency_estimator_push(const uint16_t *samples,
                                 uint32_t sample_count,
                                 uint8_t requested_components,
                                 frequency_estimator_result_t *result)
{
  (void)samples;
  (void)sample_count;
  (void)requested_components;
  (void)result;
  return 0U;
}

#endif
