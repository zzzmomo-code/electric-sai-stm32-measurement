#include "system.h"

/*
 * 模块用途：H743 上的 ADC 混合信号识别、Q32 NCO/PLL 和双 DAC 波形再生。
 * GPIO 映射：PC0=ADC1_INP10，PA4=DAC1_OUT1，PA5=DAC1_OUT2。
 * 外设依赖：ADC1、DAC1 CH1/CH2、TIM2 TRGO、DMA1 Stream0/1/2。
 * 初始化方法：由 system_init() 调用 signal_separation_start()。
 * 调用方法：由 system_process() 高频调用 signal_separation_process()。
 *
 * 与 2023H/H723 原工程的关键差异：
 * 1. H743 没有 H723 使用的硬件 CORDIC，初始化表和相位反正切改用 M7 FPU。
 * 2. DMA1 不访问 DTCM，三个 DMA 缓冲区经链接脚本放在 D2 SRAM。
 * 3. D-Cache 保持开启；ADC 读取前失效缓存，DAC 写完后清理缓存。
 * 4. DMA 半区为 32 字节整数倍，分析仍只取 500 点以保持 5 kHz 正交频点。
 */

#define signal_two_pi_f                  6.28318530717958647692f
#define signal_q32_scale_d               4294967296.0

_Static_assert(SIGSEP_ANALYSIS_FRAME_LEN <= SIGSEP_ADC_DMA_HALF_LEN,
               "analysis frame must fit in one ADC DMA half");
_Static_assert(((SIGSEP_ADC_DMA_HALF_LEN * sizeof(uint16_t)) % 32U) == 0U,
               "ADC DMA half must contain complete D-Cache lines");
_Static_assert(((SIGSEP_DAC_DMA_HALF_LEN * sizeof(uint16_t)) % 32U) == 0U,
               "DAC DMA half must contain complete D-Cache lines");
_Static_assert((SIGSEP_MAX_BIN * SIGSEP_FREQ_STEP_HZ) <=
               (SIGSEP_SAMPLE_RATE_HZ / 2U),
               "correlation table must not exceed Nyquist");
_Static_assert((SIGSEP_OPERATION_MODE == SIGSEP_MODE_DUAL_MIXED) ||
               (SIGSEP_OPERATION_MODE == SIGSEP_MODE_SINGLE),
               "invalid signal separation operation mode");
_Static_assert(SIGSEP_FREQ_COUNT >= 2U,
               "frequency search requires at least two bins");

typedef struct
{
  uint32_t frequency_hz;       /* 分量频率，单位 Hz。 */
  uint8_t frequency_index;     /* 在候选频率表中的索引。 */
  signal_wave_type_t wave;     /* 正弦波或三角波。 */
  float amplitude_adc;         /* 输入分量 ADC 峰值估计。 */
  float amplitude_dac;         /* 限幅后的 DAC 输出峰值。 */
  uint32_t phase_q32;          /* 输入分量在帧起点的 Q32 相位。 */
} signal_component_t;

typedef struct
{
  uint32_t nominal_step;       /* 理想 Q32 每采样点相位步进。 */
  int32_t step_correction;     /* PLL 叠加到步进上的修正量。 */
  int64_t integrator;          /* 带泄漏的相位误差积分项。 */
  uint32_t phase_reference;    /* 参考采样点对应的 Q32 相位。 */
  uint64_t sample_reference;   /* 相位参考的绝对采样点编号。 */
  int32_t last_error;          /* 最近一次有符号 Q32 相位误差。 */
} signal_nco_state_t;

/*
 * 三个 DMA 缓冲区必须：
 * - 位于 DMA1 可访问的 D2 SRAM；
 * - 起始地址按 32 字节对齐；
 * - 半区长度是 32 字节整数倍。
 * 具体地址由 STM32H743VITX_FLASH.ld 的 .dma_buffer 段分配。
 */
static uint16_t adc_dma_buffer[SIGSEP_ADC_DMA_LEN]
  __attribute__((section(".dma_buffer"), aligned(32)));
static uint16_t dac1_dma_buffer[SIGSEP_DAC_DMA_LEN]
  __attribute__((section(".dma_buffer"), aligned(32)));
static uint16_t dac2_dma_buffer[SIGSEP_DAC_DMA_LEN]
  __attribute__((section(".dma_buffer"), aligned(32)));

/*
 * ADC DMA 完成后先把最新安全半区复制到 CPU 专用缓冲区，再执行耗时相关检测。
 * 这样即使下一次 DMA 事件在算法计算期间到达，也不会改写正在分析的数据。
 */
static uint16_t adc_analysis_buffer[SIGSEP_ANALYSIS_FRAME_LEN];

/* H743 用 FPU 只在启动阶段生成下列查找表，实时路径不调用 sinf/cosf。 */
static int16_t sine_lut[SIGSEP_SINE_LUT_SIZE];
static float step_sine[SIGSEP_MAX_BIN + 1U];
static float step_cosine[SIGSEP_MAX_BIN + 1U];
static float identify_amplitude_accumulator[SIGSEP_FREQ_COUNT];

/* 主循环拥有的分量、NCO 和统计状态。 */
static signal_component_t active_component[2];
static signal_nco_state_t nco_state[2];
static uint32_t identify_frame_count;
static uint8_t separation_identified;
static int32_t output_phase_offset_deg = SIGSEP_PHASE_OFFSET_DEFAULT_DEG;
static uint64_t adc_sample_count;
static uint64_t dac_sample_count;
static uint32_t adc_event_handled;
static uint32_t dac_event_handled;
static uint32_t adc_frame_count;
static uint32_t adc_frame_overrun;
static uint32_t dac_half_overrun;

/*
 * 每个 DMA 回调只递增一个目的明确的事件标志。32 位读写在对齐的 Cortex-M7 上
 * 是原子的；主循环短暂关闭中断后取得三个一致快照。
 */
volatile uint32_t adc_dma_event_flag;
volatile uint32_t dac1_dma_event_flag;
volatile uint32_t dac2_dma_event_flag;

/**
 * @brief 将指定 DMA 地址范围对应的 D-Cache 行失效。
 * @param address 32 字节对齐的缓冲区地址。
 * @param byte_count 32 字节整数倍的长度。
 * @return 无。
 * @note 用于 DMA 写、CPU 读的 ADC 缓冲区。
 */
static void dma_invalidate_from_cpu(const void *address, uint32_t byte_count)
{
  SCB_InvalidateDCache_by_Addr((uint32_t *)address, (int32_t)byte_count);
  __DSB();
  __ISB();
}

/**
 * @brief 将指定 DMA 地址范围的脏 D-Cache 行写回内存。
 * @param address 32 字节对齐的缓冲区地址。
 * @param byte_count 32 字节整数倍的长度。
 * @return 无。
 * @note 用于 CPU 写、DMA 读的 DAC 缓冲区。
 */
static void dma_clean_to_peripheral(const void *address, uint32_t byte_count)
{
  SCB_CleanDCache_by_Addr((uint32_t *)address, (int32_t)byte_count);
  __DSB();
  __ISB();
}

/**
 * @brief 获取包含 APB1 定时器倍频规则的 TIM2 输入时钟。
 * @param 无。
 * @return TIM2 输入时钟，单位 Hz。
 */
static uint32_t timer2_get_clock_hz(void)
{
  uint32_t clock_hz = HAL_RCC_GetPCLK1Freq();

  if ((RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) != RCC_D2CFGR_D2PPRE1_DIV1)
  {
    clock_hz *= 2U;
  }

  return clock_hz;
}

/**
 * @brief 核对 CubeMX 生成的 TIM2 参数是否确实得到 2.5 MSPS。
 * @param 无。
 * @return 无。配置不一致时进入 Error_Handler()。
 * @note 不修改 MX_TIM2_Init()，只验证用户保存的 IOC 结果。
 */
static void timer2_verify_sample_rate(void)
{
  uint32_t timer_clock_hz = timer2_get_clock_hz();
  uint32_t expected_period;

  if ((timer_clock_hz < SIGSEP_SAMPLE_RATE_HZ) ||
      ((timer_clock_hz % SIGSEP_SAMPLE_RATE_HZ) != 0U))
  {
    Error_Handler();
  }

  expected_period = (timer_clock_hz / SIGSEP_SAMPLE_RATE_HZ) - 1U;
  if ((htim2.Init.Prescaler != 0U) || (htim2.Init.Period != expected_period))
  {
    Error_Handler();
  }
}

/**
 * @brief 把频率转换为每个采样点的 Q32 相位步进。
 * @param frequency_hz 目标频率，单位 Hz。
 * @return Q32 相位步进，2^32 代表一个完整周期。
 */
static uint32_t phase_step_q32(uint32_t frequency_hz)
{
  return (uint32_t)(((uint64_t)frequency_hz * 4294967296ULL) /
                    SIGSEP_SAMPLE_RATE_HZ);
}

/**
 * @brief 把角度偏移转换为 Q32 相位。
 * @param degree 角度，允许有符号输入。
 * @return 模 2^32 的 Q32 相位。
 */
static uint32_t phase_offset_degree_to_q32(int32_t degree)
{
  return (uint32_t)(((int64_t)degree * 4294967296LL) / 360LL);
}

/**
 * @brief 限制并量化 DAC2 的用户相位偏移。
 * @param degree 待处理角度。
 * @return 0～180°范围内、按 5°量化后的角度。
 */
static int32_t normalize_phase_offset_degree(int32_t degree)
{
  int32_t remainder;

  if (degree < SIGSEP_PHASE_OFFSET_MIN_DEG)
  {
    degree = SIGSEP_PHASE_OFFSET_MIN_DEG;
  }
  if (degree > SIGSEP_PHASE_OFFSET_MAX_DEG)
  {
    degree = SIGSEP_PHASE_OFFSET_MAX_DEG;
  }

  remainder = degree % SIGSEP_PHASE_OFFSET_STEP_DEG;
  degree -= remainder;
  if (remainder >= ((SIGSEP_PHASE_OFFSET_STEP_DEG + 1) / 2))
  {
    degree += SIGSEP_PHASE_OFFSET_STEP_DEG;
  }
  if (degree > SIGSEP_PHASE_OFFSET_MAX_DEG)
  {
    degree = SIGSEP_PHASE_OFFSET_MAX_DEG;
  }

  return degree;
}

/**
 * @brief 使用 H743 M7 FPU 预计算相关检测步进表和 DAC 正弦表。
 * @param 无。
 * @return 无。
 * @note 仅启动时调用；替代 H723 原工程的硬件 CORDIC 外设。
 */
static void prepare_math_tables(void)
{
  uint32_t index;

  step_sine[0] = 0.0f;
  step_cosine[0] = 1.0f;
  for (index = 1U; index <= SIGSEP_MAX_BIN; index++)
  {
    float angle = signal_two_pi_f *
                  ((float)(index * SIGSEP_FREQ_STEP_HZ) /
                   (float)SIGSEP_SAMPLE_RATE_HZ);
    step_sine[index] = sinf(angle);
    step_cosine[index] = cosf(angle);
  }

  for (index = 0U; index < SIGSEP_SINE_LUT_SIZE; index++)
  {
    float angle = signal_two_pi_f *
                  ((float)index / (float)SIGSEP_SINE_LUT_SIZE);
    sine_lut[index] = (int16_t)(sinf(angle) * 32767.0f);
  }
}

/**
 * @brief 计算一帧 ADC 数据的直流平均值。
 * @param samples 至少包含 500 点的 ADC 数据。
 * @return ADC 码均值。
 */
static float frame_mean(const uint16_t *samples)
{
  float mean = 0.0f;
  uint32_t index;

  for (index = 0U; index < SIGSEP_ANALYSIS_FRAME_LEN; index++)
  {
    mean += (float)samples[index];
  }

  return mean / (float)SIGSEP_ANALYSIS_FRAME_LEN;
}

/**
 * @brief 把正弦/余弦相关向量转换为输入正弦的 Q32 相位。
 * @param sine_part 与正弦参考的相关和，相当于向量 x 分量。
 * @param cosine_part 与余弦参考的相关和，相当于向量 y 分量。
 * @return 0～2^32-1 的 Q32 相位。
 * @note 对 x=A*sin(wt+phi)，atan2(cosine_part,sine_part)=phi。
 */
static uint32_t phase_from_iq(float sine_part, float cosine_part)
{
  float angle;
  double scaled;

  if ((fabsf(sine_part) < 1.0f) && (fabsf(cosine_part) < 1.0f))
  {
    return 0U;
  }

  angle = atan2f(cosine_part, sine_part);
  if (angle < 0.0f)
  {
    angle += signal_two_pi_f;
  }

  scaled = ((double)angle * signal_q32_scale_d) / (double)signal_two_pi_f;
  if (scaled >= signal_q32_scale_d)
  {
    scaled -= signal_q32_scale_d;
  }

  return (uint32_t)scaled;
}

/**
 * @brief 测量指定 5 kHz 栅格频率的幅值和相位。
 * @param samples ADC 分析帧。
 * @param mean 已计算的直流均值。
 * @param frequency_hz 待测频率。
 * @param amplitude 非空的幅值输出。
 * @param phase_q32 非空的 Q32 相位输出。
 * @return 无。
 */
static void measure_component(const uint16_t *samples, float mean,
                              uint32_t frequency_hz, float *amplitude,
                              uint32_t *phase_q32)
{
  uint32_t bin = frequency_hz / SIGSEP_FREQ_STEP_HZ;
  float sine_value = 0.0f;
  float cosine_value = 1.0f;
  float sine_sum = 0.0f;
  float cosine_sum = 0.0f;
  uint32_t index;

  if ((bin == 0U) || (bin > SIGSEP_MAX_BIN) ||
      ((frequency_hz % SIGSEP_FREQ_STEP_HZ) != 0U))
  {
    *amplitude = 0.0f;
    *phase_q32 = 0U;
    return;
  }

  for (index = 0U; index < SIGSEP_ANALYSIS_FRAME_LEN; index++)
  {
    float sample = (float)samples[index] - mean;
    float next_cosine;

    sine_sum += sample * sine_value;
    cosine_sum += sample * cosine_value;
    next_cosine = (cosine_value * step_cosine[bin]) -
                  (sine_value * step_sine[bin]);
    sine_value = (sine_value * step_cosine[bin]) +
                 (cosine_value * step_sine[bin]);
    cosine_value = next_cosine;
  }

  *amplitude = (2.0f * sqrtf((sine_sum * sine_sum) +
                             (cosine_sum * cosine_sum))) /
               (float)SIGSEP_ANALYSIS_FRAME_LEN;
  *phase_q32 = phase_from_iq(sine_sum, cosine_sum);
}

/**
 * @brief 只测量指定频率分量的幅值。
 * @param samples ADC 分析帧。
 * @param mean 已计算的直流均值。
 * @param frequency_hz 待测频率。
 * @return 该频率分量的 ADC 峰值估计。
 */
static float measure_amplitude_only(const uint16_t *samples, float mean,
                                    uint32_t frequency_hz)
{
  float amplitude;
  uint32_t unused_phase;

  measure_component(samples, mean, frequency_hz, &amplitude, &unused_phase);
  return amplitude;
}

/**
 * @brief 根据三次/五次谐波含量判断正弦波、三角波或方波。
 * @param samples ADC 分析帧。
 * @param mean ADC 均值。
 * @param frequency_hz 当前分量基波频率。
 * @param other_frequency_hz 另一个主分量频率。
 * @param fundamental_amplitude 当前基波幅值。
 * @return 识别出的波形类型。
 * @note 方波奇次谐波明显强于三角波；若三次谐波与双信号模式的另一分量
 *       重合，则改用五次谐波，避免把另一输入误当成本分量谐波。
 */
static signal_wave_type_t detect_wave_type(const uint16_t *samples, float mean,
                                           uint32_t frequency_hz,
                                           uint32_t other_frequency_hz,
                                           float fundamental_amplitude)
{
  float harmonic3_amplitude = 0.0f;
  float harmonic5_amplitude = 0.0f;

  if (fundamental_amplitude < SIGSEP_MIN_VALID_ADC_AMP)
  {
    return signal_wave_sine;
  }

  if ((frequency_hz * 3U) <= (SIGSEP_SAMPLE_RATE_HZ / 2U))
  {
    harmonic3_amplitude = measure_amplitude_only(samples, mean,
                                                 frequency_hz * 3U);
  }
  if ((frequency_hz * 5U) <= (SIGSEP_SAMPLE_RATE_HZ / 2U))
  {
    harmonic5_amplitude = measure_amplitude_only(samples, mean,
                                                 frequency_hz * 5U);
  }

#if (SIGSEP_OPERATION_MODE == SIGSEP_MODE_SINGLE)
  /*
   * 单信号扩展模式额外区分方波；参考工程的双信号题目模式只区分正弦波和
   * 三角波，避免改变其既有判据和输出流程。
   */
  if (other_frequency_hz == (frequency_hz * 3U))
  {
    if (harmonic5_amplitude >
        (fundamental_amplitude * SIGSEP_SQUARE_H5_RATIO))
    {
      return signal_wave_square;
    }
    if (harmonic5_amplitude >
        (fundamental_amplitude * SIGSEP_TRI_H5_RATIO))
    {
      return signal_wave_triangle;
    }
  }
  else
  {
    if (harmonic3_amplitude >
        (fundamental_amplitude * SIGSEP_SQUARE_H3_RATIO))
    {
      return signal_wave_square;
    }
    if (harmonic3_amplitude >
        (fundamental_amplitude * SIGSEP_TRI_H3_RATIO))
    {
      return signal_wave_triangle;
    }
  }
#else
  if ((other_frequency_hz == (frequency_hz * 3U)) &&
      (harmonic5_amplitude >
       (fundamental_amplitude * SIGSEP_TRI_H5_RATIO)))
  {
    return signal_wave_triangle;
  }
  if ((other_frequency_hz != (frequency_hz * 3U)) &&
      (harmonic3_amplitude >
       (fundamental_amplitude * SIGSEP_TRI_H3_RATIO)))
  {
    return signal_wave_triangle;
  }
#endif

  return signal_wave_sine;
}

/**
 * @brief 按当前编译期模式搜索单个主分量或两个混合分量。
 * @param samples ADC 分析帧。
 * @param output 最多两个分量的输出数组；单信号模式只使用 output[0]。
 * @return 累积到指定帧数并获得当前模式所需分量时返回 1，否则返回 0。
 */
static uint8_t analyze_frame(const uint16_t *samples,
                             signal_component_t output[2])
{
  float mean = frame_mean(samples);
  float amplitude[SIGSEP_FREQ_COUNT];
  uint32_t phase[SIGSEP_FREQ_COUNT];
  uint32_t best0 = 0U;
  uint32_t index;
  uint32_t component_count;

  for (index = 0U; index < SIGSEP_FREQ_COUNT; index++)
  {
    uint32_t frequency_hz = SIGSEP_FREQ_MIN_HZ +
                            (index * SIGSEP_FREQ_STEP_HZ);
    measure_component(samples, mean, frequency_hz,
                      &amplitude[index], &phase[index]);
    identify_amplitude_accumulator[index] += amplitude[index];
  }

  identify_frame_count++;
  if (identify_frame_count < SIGSEP_IDENTIFY_FRAMES)
  {
    return 0U;
  }

  for (index = 0U; index < SIGSEP_FREQ_COUNT; index++)
  {
    amplitude[index] = identify_amplitude_accumulator[index] /
                       (float)identify_frame_count;
    identify_amplitude_accumulator[index] = 0.0f;
  }
  identify_frame_count = 0U;

#if (SIGSEP_OPERATION_MODE == SIGSEP_MODE_SINGLE)
  /*
   * 单信号模式只寻找全频段最强基波，不再强制从底噪中凑出第二个分量。
   * 这样一路干净输入只驱动 PA4，PA5 始终保持 DAC 中点。
   */
  for (index = 1U; index < SIGSEP_FREQ_COUNT; index++)
  {
    if (amplitude[index] > amplitude[best0])
    {
      best0 = index;
    }
  }
  if (amplitude[best0] < SIGSEP_MIN_VALID_ADC_AMP)
  {
    return 0U;
  }

  output[0].frequency_index = (uint8_t)best0;
  output[0].frequency_hz = SIGSEP_FREQ_MIN_HZ +
                           (best0 * SIGSEP_FREQ_STEP_HZ);
  output[0].amplitude_adc = amplitude[best0];
  output[0].phase_q32 = phase[best0];
  output[0].wave = detect_wave_type(samples, mean,
                                    output[0].frequency_hz, 0U,
                                    output[0].amplitude_adc);

  output[1].frequency_index = 0U;
  output[1].frequency_hz = 0U;
  output[1].wave = signal_wave_sine;
  output[1].amplitude_adc = 0.0f;
  output[1].amplitude_dac = 0.0f;
  output[1].phase_q32 = 0U;
  component_count = 1U;
#else
  {
    uint32_t best1 = 1U;
    uint32_t temporary;

    if (amplitude[best1] > amplitude[best0])
    {
      best0 = 1U;
      best1 = 0U;
    }

    for (index = 2U; index < SIGSEP_FREQ_COUNT; index++)
    {
      if (amplitude[index] > amplitude[best0])
      {
        best1 = best0;
        best0 = index;
      }
      else if (amplitude[index] > amplitude[best1])
      {
        best1 = index;
      }
    }

    /* 两路 DAC 固定按低频到高频排序，与峰值强弱无关。 */
    if (best0 > best1)
    {
      temporary = best0;
      best0 = best1;
      best1 = temporary;
    }

    output[0].frequency_index = (uint8_t)best0;
    output[0].frequency_hz = SIGSEP_FREQ_MIN_HZ +
                             (best0 * SIGSEP_FREQ_STEP_HZ);
    output[0].amplitude_adc = amplitude[best0];
    output[0].phase_q32 = phase[best0];

    output[1].frequency_index = (uint8_t)best1;
    output[1].frequency_hz = SIGSEP_FREQ_MIN_HZ +
                             (best1 * SIGSEP_FREQ_STEP_HZ);
    output[1].amplitude_adc = amplitude[best1];
    output[1].phase_q32 = phase[best1];

    output[0].wave = detect_wave_type(samples, mean,
                                      output[0].frequency_hz,
                                      output[1].frequency_hz,
                                      output[0].amplitude_adc);
    output[1].wave = detect_wave_type(samples, mean,
                                      output[1].frequency_hz,
                                      output[0].frequency_hz,
                                      output[1].amplitude_adc);
    component_count = 2U;
  }
#endif

  for (index = 0U; index < component_count; index++)
  {
    output[index].amplitude_dac =
      output[index].amplitude_adc * SIGSEP_ADC_TO_DAC_SCALE;
  }

  return 1U;
}

/**
 * @brief 初始化一个输出通道的 Q32 NCO/PLL。
 * @param channel 通道索引 0 或 1。
 * @param component 已识别分量。
 * @param sample_start 当前 ADC 帧的绝对起始采样点。
 * @return 无。
 */
static void nco_init(uint32_t channel, const signal_component_t *component,
                     uint64_t sample_start)
{
  nco_state[channel].nominal_step =
    phase_step_q32(component->frequency_hz);
  nco_state[channel].step_correction = 0;
  nco_state[channel].integrator = 0;
  nco_state[channel].phase_reference = component->phase_q32;
  nco_state[channel].sample_reference = sample_start;
  nco_state[channel].last_error = 0;
}

/**
 * @brief 获取包含 PLL 修正的实际 NCO 步进。
 * @param channel 通道索引 0 或 1。
 * @return 实际 Q32 相位步进。
 */
static uint32_t nco_step(uint32_t channel)
{
  return nco_state[channel].nominal_step +
         (uint32_t)nco_state[channel].step_correction;
}

/**
 * @brief 预测某个绝对采样点的 NCO 相位。
 * @param channel 通道索引 0 或 1。
 * @param sample 绝对采样点编号。
 * @return 该采样点的 Q32 相位。
 */
static uint32_t nco_phase_at_sample(uint32_t channel, uint64_t sample)
{
  uint64_t delta = 0U;

  if (sample >= nco_state[channel].sample_reference)
  {
    delta = sample - nco_state[channel].sample_reference;
  }

  return nco_state[channel].phase_reference +
         (uint32_t)((uint64_t)nco_step(channel) * delta);
}

/**
 * @brief 用当前帧测得相位更新一个 PI 型数字 PLL。
 * @param channel 通道索引 0 或 1。
 * @param measured_phase 当前帧起点的测量相位。
 * @param frame_start_sample 当前帧绝对起始采样点。
 * @return 当前有符号 Q32 相位误差。
 */
static int32_t nco_update_lock(uint32_t channel, uint32_t measured_phase,
                               uint64_t frame_start_sample)
{
  uint32_t predicted_phase =
    nco_phase_at_sample(channel, frame_start_sample);
  int32_t phase_error = (int32_t)(measured_phase - predicted_phase);
  int64_t integrator =
    ((nco_state[channel].integrator *
      (int64_t)SIGSEP_PLL_INTEGRATOR_LEAK_NUM) / 65536LL) +
    (int64_t)phase_error;
  int32_t max_correction =
    (int32_t)(nco_state[channel].nominal_step /
              SIGSEP_PLL_MAX_CORR_DIV);
  int64_t integrator_limit;
  int64_t correction;

  if (max_correction < 1)
  {
    max_correction = 1;
  }

  integrator_limit =
    (int64_t)max_correction *
    (int64_t)SIGSEP_ADC_DMA_HALF_LEN *
    (int64_t)SIGSEP_PLL_STEP_KI_DIV;
  if (integrator_limit > SIGSEP_PLL_INTEGRATOR_LIMIT)
  {
    integrator_limit = SIGSEP_PLL_INTEGRATOR_LIMIT;
  }
  if (integrator > integrator_limit)
  {
    integrator = integrator_limit;
  }
  if (integrator < -integrator_limit)
  {
    integrator = -integrator_limit;
  }

  correction =
    ((int64_t)phase_error /
     (int64_t)(SIGSEP_ADC_DMA_HALF_LEN * SIGSEP_PLL_STEP_KP_DIV)) +
    (integrator /
     (int64_t)(SIGSEP_ADC_DMA_HALF_LEN * SIGSEP_PLL_STEP_KI_DIV));
  if (correction > (int64_t)max_correction)
  {
    correction = max_correction;
  }
  if (correction < -(int64_t)max_correction)
  {
    correction = -(int64_t)max_correction;
  }

  nco_state[channel].integrator = integrator;
  nco_state[channel].step_correction = (int32_t)correction;
  nco_state[channel].phase_reference =
    predicted_phase +
    (uint32_t)(phase_error /
               (int32_t)(1UL << SIGSEP_PLL_PHASE_KP_SHIFT));
  nco_state[channel].sample_reference = frame_start_sample;
  nco_state[channel].last_error = phase_error;

  return phase_error;
}

#if ((SIGSEP_OPERATION_MODE == SIGSEP_MODE_DUAL_MIXED) && \
     (SIGSEP_COMMON_SOURCE_LOCK != 0U))
/**
 * @brief 按频率比例把主通道相位时间误差换算到跟随通道。
 * @param phase_error 主通道有符号 Q32 相位误差。
 * @param destination_hz 跟随通道频率。
 * @param source_hz 主通道频率。
 * @return 跟随通道的有符号 Q32 相位误差。
 */
static int32_t scale_phase_error_by_frequency(int32_t phase_error,
                                              uint32_t destination_hz,
                                              uint32_t source_hz)
{
  int64_t scaled;

  if (source_hz == 0U)
  {
    return 0;
  }

  scaled = ((int64_t)phase_error * (int64_t)destination_hz) /
           (int64_t)source_hz;
  return (int32_t)((uint32_t)scaled);
}

/**
 * @brief 先在宽整数域缩放主相位误差并施加环路增益，最后才折回 Q32。
 * @param phase_error 主通道有符号 Q32 相位误差。
 * @param destination_hz 跟随通道频率。
 * @param source_hz 主通道频率。
 * @return 跟随通道本帧应施加的有符号 Q32 相位修正。
 * @note 不可先把倍频后的结果折回 int32_t 再除以 2，否则大相位误差在整数倍频
 *       场景会产生 180° 分支错误。
 */
static int32_t scale_phase_adjustment_by_frequency(
  int32_t phase_error, uint32_t destination_hz, uint32_t source_hz)
{
  int64_t scaled_adjustment;

  if (source_hz == 0U)
  {
    return 0;
  }

  scaled_adjustment =
    ((int64_t)phase_error * (int64_t)destination_hz) /
    (int64_t)source_hz;
  scaled_adjustment /=
    (int64_t)(1UL << SIGSEP_PLL_PHASE_KP_SHIFT);

  return (int32_t)((uint32_t)scaled_adjustment);
}

/**
 * @brief 把主通道频率步进修正按名义步进比例映射到跟随通道。
 * @param follower_channel 跟随通道索引。
 * @param master_channel 主通道索引。
 * @return 跟随通道的限幅步进修正。
 */
static int32_t scale_step_correction(uint32_t follower_channel,
                                     uint32_t master_channel)
{
  int64_t scaled;
  int32_t max_correction;

  if (nco_state[master_channel].nominal_step == 0U)
  {
    return 0;
  }

  scaled =
    ((int64_t)nco_state[master_channel].step_correction *
     (int64_t)nco_state[follower_channel].nominal_step) /
    (int64_t)nco_state[master_channel].nominal_step;
  max_correction =
    (int32_t)(nco_state[follower_channel].nominal_step /
              SIGSEP_PLL_MAX_CORR_DIV);
  if (scaled > (int64_t)max_correction)
  {
    scaled = max_correction;
  }
  if (scaled < -(int64_t)max_correction)
  {
    scaled = -(int64_t)max_correction;
  }

  return (int32_t)scaled;
}

/**
 * @brief 让同源跟随通道继承主 PLL 的时间修正。
 * @param follower_channel 跟随通道索引。
 * @param master_channel 主通道索引。
 * @param master_phase_error 主通道本帧相位误差。
 * @param frame_start_sample 当前 ADC 帧起始采样点。
 * @return 无。
 */
static void nco_follow_common_source(uint32_t follower_channel,
                                     uint32_t master_channel,
                                     int32_t master_phase_error,
                                     uint64_t frame_start_sample)
{
  uint32_t predicted_phase =
    nco_phase_at_sample(follower_channel, frame_start_sample);
  int32_t follower_error =
    scale_phase_error_by_frequency(
      master_phase_error,
      active_component[follower_channel].frequency_hz,
      active_component[master_channel].frequency_hz);
  int32_t follower_phase_adjust =
    scale_phase_adjustment_by_frequency(
      master_phase_error,
      active_component[follower_channel].frequency_hz,
      active_component[master_channel].frequency_hz);

  nco_state[follower_channel].integrator = 0;
  nco_state[follower_channel].step_correction =
    scale_step_correction(follower_channel, master_channel);
  nco_state[follower_channel].phase_reference =
    predicted_phase + (uint32_t)follower_phase_adjust;
  nco_state[follower_channel].sample_reference = frame_start_sample;
  nco_state[follower_channel].last_error = follower_error;
}
#endif

/**
 * @brief 生成一个通道的一段 DAC 采样数据。
 * @param destination 目标 DAC 缓冲区。
 * @param component 当前分量参数。
 * @param start_phase 第一个输出样点的 Q32 相位。
 * @param phase_step 每输出样点的 Q32 相位步进。
 * @param length 输出样点数。
 * @return 无。
 */
static void build_dac_samples(uint16_t *destination,
                              const signal_component_t *component,
                              uint32_t start_phase, uint32_t phase_step,
                              uint32_t length)
{
  uint32_t phase = start_phase;
  int32_t amplitude = (int32_t)(component->amplitude_dac + 0.5f);
  uint32_t index;

  for (index = 0U; index < length; index++)
  {
    int32_t waveform_q15;
    int32_t code;

    if (component->wave == signal_wave_square)
    {
      /* 与正弦基波保持同一零交叉相位：前半周期为高，后半周期为低。 */
      waveform_q15 = ((phase & 0x80000000UL) == 0U) ? 32767 : -32767;
    }
    else if (component->wave == signal_wave_triangle)
    {
      uint32_t quadrant = phase >> 30;
      uint32_t fraction = (phase & 0x3FFFFFFFUL) >> 15;

      if (quadrant == 0U)
      {
        waveform_q15 = (int32_t)fraction;
      }
      else if (quadrant == 1U)
      {
        waveform_q15 = 32767 - (int32_t)fraction;
      }
      else if (quadrant == 2U)
      {
        waveform_q15 = -(int32_t)fraction;
      }
      else
      {
        waveform_q15 = -32767 + (int32_t)fraction;
      }
    }
    else
    {
      waveform_q15 = sine_lut[phase >> SIGSEP_SINE_LUT_SHIFT];
    }

    code = (int32_t)SIGSEP_DAC_MID +
           ((amplitude * waveform_q15) >> 15);
    if (code < 0)
    {
      code = 0;
    }
    if (code > (int32_t)SIGSEP_DAC_MAX)
    {
      code = (int32_t)SIGSEP_DAC_MAX;
    }

    destination[index] = (uint16_t)code;
    phase += phase_step;
  }
}

/**
 * @brief 把一个已释放的 DAC 半区填为中点电平并清理缓存。
 * @param half_index 半区索引 0 或 1。
 * @return 无。
 */
static void fill_dac_half_midscale(uint32_t half_index)
{
  uint32_t offset = half_index * SIGSEP_DAC_DMA_HALF_LEN;
  uint32_t index;
  uint32_t byte_count =
    SIGSEP_DAC_DMA_HALF_LEN * (uint32_t)sizeof(uint16_t);

  for (index = 0U; index < SIGSEP_DAC_DMA_HALF_LEN; index++)
  {
    dac1_dma_buffer[offset + index] = SIGSEP_DAC_MID;
    dac2_dma_buffer[offset + index] = SIGSEP_DAC_MID;
  }

  dma_clean_to_peripheral(&dac1_dma_buffer[offset], byte_count);
  dma_clean_to_peripheral(&dac2_dma_buffer[offset], byte_count);
}

/**
 * @brief 按当前模式填充 DAC 信号半区并写回 D-Cache。
 * @param half_index 半区索引 0 或 1。
 * @param play_sample 该半区下次播放时第一个样点的绝对编号。
 * @return 无。
 */
static void fill_dac_half_signal(uint32_t half_index, uint64_t play_sample)
{
  uint32_t offset = half_index * SIGSEP_DAC_DMA_HALF_LEN;
  uint32_t phase0 =
    nco_phase_at_sample(0U, play_sample) +
    phase_offset_degree_to_q32(SIGSEP_DAC1_PHASE_OFFSET_DEG);
  uint32_t byte_count =
    SIGSEP_DAC_DMA_HALF_LEN * (uint32_t)sizeof(uint16_t);

  build_dac_samples(&dac1_dma_buffer[offset], &active_component[0],
                    phase0, nco_step(0U), SIGSEP_DAC_DMA_HALF_LEN);
#if (SIGSEP_OPERATION_MODE == SIGSEP_MODE_SINGLE)
  {
    uint32_t index;

    for (index = 0U; index < SIGSEP_DAC_DMA_HALF_LEN; index++)
    {
      dac2_dma_buffer[offset + index] = SIGSEP_DAC_MID;
    }
  }
#else
  {
    uint32_t phase1 =
      nco_phase_at_sample(1U, play_sample) +
      phase_offset_degree_to_q32(SIGSEP_DAC2_PHASE_OFFSET_DEG +
                                 output_phase_offset_deg);

    build_dac_samples(&dac2_dma_buffer[offset], &active_component[1],
                      phase1, nco_step(1U), SIGSEP_DAC_DMA_HALF_LEN);
  }
#endif
  dma_clean_to_peripheral(&dac1_dma_buffer[offset], byte_count);
  dma_clean_to_peripheral(&dac2_dma_buffer[offset], byte_count);
}

/**
 * @brief 清零所有事件标志并初始化主循环统计状态。
 * @param 无。
 * @return 无。
 */
static void reset_runtime_state(void)
{
  uint32_t index;

  adc_dma_event_flag = 0U;
  dac1_dma_event_flag = 0U;
  dac2_dma_event_flag = 0U;
  adc_sample_count = 0U;
  dac_sample_count = 0U;
  adc_event_handled = 0U;
  dac_event_handled = 0U;
  adc_frame_count = 0U;
  adc_frame_overrun = 0U;
  dac_half_overrun = 0U;
  identify_frame_count = 0U;
  separation_identified = 0U;
  memset(active_component, 0, sizeof(active_component));
  memset(nco_state, 0, sizeof(nco_state));

  for (index = 0U; index < SIGSEP_FREQ_COUNT; index++)
  {
    identify_amplitude_accumulator[index] = 0.0f;
  }
}

/**
 * @brief 根据两路 DAC 的单调事件序号回填当前确定安全的最新半区。
 * @param dac1_event_count DAC CH1 已完成的半区事件总数。
 * @param dac2_event_count DAC CH2 已完成的半区事件总数。
 * @return 无。
 * @note 两个计数不相等时表示其中一路 IRQ 尚未到达，暂不触碰任何半区。若一次
 *       积压多代，只回填当前最新安全半区，并把中间未及时回填的代次记为丢失。
 */
static void service_dac_halves(uint32_t dac1_event_count,
                               uint32_t dac2_event_count)
{
  uint32_t pending_event_count;
  uint32_t half_index;
  uint64_t play_sample;

  if (dac1_event_count != dac2_event_count)
  {
    return;
  }

  pending_event_count = dac1_event_count - dac_event_handled;
  if (pending_event_count == 0U)
  {
    return;
  }

  if (pending_event_count > 1U)
  {
    dac_half_overrun += pending_event_count - 1U;
  }

  dac_sample_count +=
    (uint64_t)pending_event_count * SIGSEP_DAC_DMA_HALF_LEN;
  dac_event_handled = dac1_event_count;

  /*
   * 第 1、3、5... 个事件是前半区释放，第 2、4、6... 个事件是后半区释放。
   * 此刻安全半区将在再经过一个半区后重新播放。
   */
  half_index = ((dac1_event_count & 1U) != 0U) ? 0U : 1U;
  play_sample = dac_sample_count + SIGSEP_DAC_DMA_HALF_LEN;

  if (separation_identified != 0U)
  {
    fill_dac_half_signal(half_index, play_sample);
  }
  else
  {
    fill_dac_half_midscale(half_index);
  }
}

/**
 * @brief 把平滑后的 ADC 幅值换算并限制为 DAC 输出幅值。
 * @param channel 通道索引 0 或 1。
 * @param smooth_output 非零时对 DAC 幅值继续做一阶平滑。
 * @return 无。
 */
static void update_dac_amplitude(uint32_t channel, uint8_t smooth_output)
{
  float waveform_peak_adc = active_component[channel].amplitude_adc;
  float target;

  /*
   * 相关检测得到的是基波峰值。方波基波为原波峰值的 4/pi，三角波基波为
   * 原波峰值的 8/pi^2，先换回时域峰值再映射到 DAC，避免不同波形幅度失真。
   */
  if (active_component[channel].wave == signal_wave_square)
  {
    waveform_peak_adc *= 0.7853981634f;
  }
  else if (active_component[channel].wave == signal_wave_triangle)
  {
    waveform_peak_adc *= 1.2337005501f;
  }

  target = waveform_peak_adc * SIGSEP_ADC_TO_DAC_SCALE;

  if (target < SIGSEP_DEFAULT_DAC_AMP)
  {
    target = SIGSEP_DEFAULT_DAC_AMP;
  }
  if (target > SIGSEP_MAX_DAC_AMP)
  {
    target = SIGSEP_MAX_DAC_AMP;
  }

  if (smooth_output != 0U)
  {
    active_component[channel].amplitude_dac +=
      (target - active_component[channel].amplitude_dac) /
      (float)(1UL << SIGSEP_AMP_SMOOTH_SHIFT);
  }
  else
  {
    active_component[channel].amplitude_dac = target;
  }
}

#if ((SIGSEP_OPERATION_MODE == SIGSEP_MODE_SINGLE) || \
     (SIGSEP_COMMON_SOURCE_LOCK == 0U))
/**
 * @brief 用与参考工程双通道独立 PLL 完全相同的流程跟踪一个分量。
 * @param channel 待跟踪的输出通道索引。
 * @param samples 当前 ADC 分析帧。
 * @param mean 当前分析帧的直流均值。
 * @param frame_start_sample 当前帧第一个样点的绝对采样点编号。
 * @return 无。
 * @note 固定执行“相关测幅/测相→PI 型 PLL 更新→状态与幅度平滑”顺序。
 *       单通道模式调用通道 0；双通道独立锁相依次调用通道 0 和通道 1。
 */
static void track_component_independent(uint32_t channel,
                                        const uint16_t *samples,
                                        float mean,
                                        uint64_t frame_start_sample)
{
  float measured_amplitude;
  uint32_t measured_phase;

  measure_component(samples, mean, active_component[channel].frequency_hz,
                    &measured_amplitude, &measured_phase);
  (void)nco_update_lock(channel, measured_phase, frame_start_sample);
  active_component[channel].phase_q32 = measured_phase;
  active_component[channel].amplitude_adc +=
    (measured_amplitude - active_component[channel].amplitude_adc) /
    (float)(1UL << SIGSEP_AMP_SMOOTH_SHIFT);
  update_dac_amplitude(channel, 1U);
}
#endif

/**
 * @brief 把一个刚完成的 ADC DMA 半区复制到 CPU 专用分析缓冲区。
 * @param offset 半区在 adc_dma_buffer 中的起始索引。
 * @param expected_event_count 选择该半区时看到的 ADC 事件序号。
 * @return 复制期间 DMA 未切换到下一半区返回 1，否则返回 0 并丢弃副本。
 */
static uint8_t copy_adc_frame(uint32_t offset,
                              uint32_t expected_event_count)
{
  const uint16_t *source = &adc_dma_buffer[offset];
  uint32_t byte_count =
    SIGSEP_ADC_DMA_HALF_LEN * (uint32_t)sizeof(uint16_t);
  uint32_t interrupt_state;
  uint32_t event_count_after_copy;

  dma_invalidate_from_cpu(source, byte_count);
  memcpy(adc_analysis_buffer, source,
         SIGSEP_ANALYSIS_FRAME_LEN * sizeof(uint16_t));
  __DMB();

  interrupt_state = __get_PRIMASK();
  __disable_irq();
  event_count_after_copy = adc_dma_event_flag;
  __set_PRIMASK(interrupt_state);

  return (event_count_after_copy == expected_event_count) ? 1U : 0U;
}

/**
 * @brief 处理一个已经安全复制、不会再被 DMA 改写的 ADC 分析帧。
 * @param samples CPU 专用的 500 点分析缓冲区。
 * @param frame_start_sample 此半区第一个样点的绝对采样点编号。
 * @return 无。
 */
static void process_adc_frame(const uint16_t *samples,
                              uint64_t frame_start_sample)
{
  adc_frame_count++;

  if (separation_identified == 0U)
  {
    signal_component_t result[2];

    if (analyze_frame(samples, result) == 0U)
    {
      return;
    }

    active_component[0] = result[0];
    nco_init(0U, &active_component[0], frame_start_sample);
#if (SIGSEP_OPERATION_MODE == SIGSEP_MODE_DUAL_MIXED)
    active_component[1] = result[1];
    nco_init(1U, &active_component[1], frame_start_sample);
#endif
    separation_identified = 1U;
    update_dac_amplitude(0U, 0U);
#if (SIGSEP_OPERATION_MODE == SIGSEP_MODE_DUAL_MIXED)
    update_dac_amplitude(1U, 0U);
#endif
  }
  else
  {
    float mean = frame_mean(samples);

#if (SIGSEP_OPERATION_MODE == SIGSEP_MODE_SINGLE)
    /*
     * 单信号模式与双通道独立 PLL 共用同一个通道跟踪函数，因此除只使用
     * 通道 0、只输出 PA4 外，测相、环路状态更新和幅度平滑顺序完全一致。
     */
    track_component_independent(0U, samples, mean, frame_start_sample);
#else
#if (SIGSEP_COMMON_SOURCE_LOCK != 0U)
    float measured_amplitude[2];
    uint32_t measured_phase[2];
    uint32_t master_channel =
      (SIGSEP_PHASE_MASTER_CH == 0U) ? 0U : 1U;
    uint32_t follower_channel = master_channel ^ 1U;
    int32_t master_phase_error;

    measure_component(samples, mean,
                      active_component[master_channel].frequency_hz,
                      &measured_amplitude[master_channel],
                      &measured_phase[master_channel]);
    measured_amplitude[follower_channel] =
      measure_amplitude_only(
        samples, mean,
        active_component[follower_channel].frequency_hz);
    measured_phase[follower_channel] =
      nco_phase_at_sample(follower_channel, frame_start_sample);
    master_phase_error =
      nco_update_lock(master_channel, measured_phase[master_channel],
                      frame_start_sample);
    nco_follow_common_source(follower_channel, master_channel,
                             master_phase_error, frame_start_sample);

    active_component[0].phase_q32 = measured_phase[0];
    active_component[1].phase_q32 = measured_phase[1];
    active_component[0].amplitude_adc +=
      (measured_amplitude[0] - active_component[0].amplitude_adc) /
      (float)(1UL << SIGSEP_AMP_SMOOTH_SHIFT);
    active_component[1].amplitude_adc +=
      (measured_amplitude[1] - active_component[1].amplitude_adc) /
      (float)(1UL << SIGSEP_AMP_SMOOTH_SHIFT);
    update_dac_amplitude(0U, 1U);
    update_dac_amplitude(1U, 1U);
#else
    track_component_independent(0U, samples, mean, frame_start_sample);
    track_component_independent(1U, samples, mean, frame_start_sample);
#endif
#endif
  }
}

/**
 * @brief 初始化并同步启动信号分离链路。
 * @param 无。
 * @return 无。
 */
void signal_separation_start(void)
{
  timer2_verify_sample_rate();
  prepare_math_tables();
  reset_runtime_state();
  fill_dac_half_midscale(0U);
  fill_dac_half_midscale(1U);

  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET,
                                  ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
                       SIGSEP_DAC_MID) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R,
                       SIGSEP_DAC_MID) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,
                        (uint32_t *)dac1_dma_buffer,
                        SIGSEP_DAC_DMA_LEN,
                        DAC_ALIGN_12B_R) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2,
                        (uint32_t *)dac2_dma_buffer,
                        SIGSEP_DAC_DMA_LEN,
                        DAC_ALIGN_12B_R) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer,
                        SIGSEP_ADC_DMA_LEN) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief 在主循环处理所有 DMA 事件和信号算法。
 * @param 无。
 * @return 无。
 */
void signal_separation_process(void)
{
  uint32_t adc_event_count;
  uint32_t dac1_event_count;
  uint32_t dac2_event_count;
  uint32_t pending_adc_event_count;
  uint32_t adc_offset;
  uint32_t interrupt_state;
  uint64_t frame_start_sample;

  interrupt_state = __get_PRIMASK();
  __disable_irq();
  adc_event_count = adc_dma_event_flag;
  dac1_event_count = dac1_dma_event_flag;
  dac2_event_count = dac2_dma_event_flag;
  __set_PRIMASK(interrupt_state);

  /* DAC 回填优先于耗时 ADC 分析，尽量扩大输出半区的时间裕量。 */
  service_dac_halves(dac1_event_count, dac2_event_count);

  pending_adc_event_count = adc_event_count - adc_event_handled;
  if (pending_adc_event_count == 0U)
  {
    return;
  }

  if (pending_adc_event_count > 1U)
  {
    adc_frame_overrun += pending_adc_event_count - 1U;
  }

  /*
   * 跳过已经过期的旧事件，只复制最新完成且此刻未被 DMA 写入的半区。绝对样点
   * 时间仍按全部事件推进，因此一次主循环超时不会永久破坏 PLL 时间轴。
   */
  adc_sample_count +=
    (uint64_t)pending_adc_event_count * SIGSEP_ADC_DMA_HALF_LEN;
  adc_event_handled = adc_event_count;
  adc_offset = ((adc_event_count & 1U) != 0U) ?
               0U : SIGSEP_ADC_DMA_HALF_LEN;
  frame_start_sample = adc_sample_count - SIGSEP_ADC_DMA_HALF_LEN;

  if (copy_adc_frame(adc_offset, adc_event_count) == 0U)
  {
    adc_frame_overrun++;
    return;
  }

  process_adc_frame(adc_analysis_buffer, frame_start_sample);
}

/**
 * @brief 清除识别结果并回到搜索状态。
 * @param 无。
 * @return 无。
 */
void signal_separation_restart_identify(void)
{
  uint32_t index;

  separation_identified = 0U;
  identify_frame_count = 0U;
  for (index = 0U; index < SIGSEP_FREQ_COUNT; index++)
  {
    identify_amplitude_accumulator[index] = 0.0f;
  }
}

/**
 * @brief 设置 DAC2 额外输出相位偏移。
 * @param degree 目标角度。
 * @return 无。
 */
void signal_separation_set_phase_offset_deg(int32_t degree)
{
  output_phase_offset_deg = normalize_phase_offset_degree(degree);
}

/**
 * @brief 获取 DAC2 当前额外输出相位偏移。
 * @param 无。
 * @return 当前角度。
 */
int32_t signal_separation_get_phase_offset_deg(void)
{
  return output_phase_offset_deg;
}

/**
 * @brief 获取当前识别、锁相与 DMA 统计快照。
 * @param status 非空状态输出。
 * @return 参数有效返回 1，否则返回 0。
 */
uint8_t signal_separation_get_status(signal_separation_status_t *status)
{
  uint32_t channel;

  if (status == NULL)
  {
    return 0U;
  }

  status->identified = separation_identified;
  status->mode = (signal_operation_mode_t)SIGSEP_OPERATION_MODE;
#if (SIGSEP_OPERATION_MODE == SIGSEP_MODE_SINGLE)
  status->output_count = 1U;
#else
  status->output_count = 2U;
#endif
  for (channel = 0U; channel < 2U; channel++)
  {
    status->frequency_hz[channel] = active_component[channel].frequency_hz;
    status->wave[channel] = active_component[channel].wave;
    status->amplitude_adc[channel] =
      (uint32_t)(active_component[channel].amplitude_adc + 0.5f);
    status->phase_error_mdeg[channel] =
      (int32_t)(((int64_t)nco_state[channel].last_error * 360000LL) /
                4294967296LL);
  }
  status->adc_frame_count = adc_frame_count;
  status->adc_frame_overrun = adc_frame_overrun;
  status->dac_half_overrun = dac_half_overrun;

  return 1U;
}

/**
 * @brief ADC1 DMA 前半区完成回调。
 * @param hadc ADC 句柄。
 * @return 无。
 * @note 中断中只递增一个事件标志，算法在主循环完成。
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    adc_dma_event_flag++;
  }
}

/**
 * @brief ADC1 DMA 后半区完成回调。
 * @param hadc ADC 句柄。
 * @return 无。
 * @note 与前半区共享同一单调事件序号；奇数代表前半区，偶数代表后半区。
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    adc_dma_event_flag++;
  }
}

/**
 * @brief DAC1 CH1 DMA 前半区完成回调。
 * @param hdac DAC 句柄。
 * @return 无。
 */
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
  if (hdac->Instance == DAC1)
  {
    dac1_dma_event_flag++;
  }
}

/**
 * @brief DAC1 CH1 DMA 后半区完成回调。
 * @param hdac DAC 句柄。
 * @return 无。
 */
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
  if (hdac->Instance == DAC1)
  {
    dac1_dma_event_flag++;
  }
}

/**
 * @brief DAC1 CH2 DMA 前半区完成回调。
 * @param hdac DAC 句柄。
 * @return 无。
 */
void HAL_DACEx_ConvHalfCpltCallbackCh2(DAC_HandleTypeDef *hdac)
{
  if (hdac->Instance == DAC1)
  {
    dac2_dma_event_flag++;
  }
}

/**
 * @brief DAC1 CH2 DMA 后半区完成回调。
 * @param hdac DAC 句柄。
 * @return 无。
 */
void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef *hdac)
{
  if (hdac->Instance == DAC1)
  {
    dac2_dma_event_flag++;
  }
}
