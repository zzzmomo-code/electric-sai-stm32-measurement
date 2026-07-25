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
 * 4. DMA 半区为 32 字节整数倍；5 kHz 原方案仍取 500 点，高精度模式额外
 *    收集 32768 个连续样点做 Hann 窗 FFT 和非栅格峰值插值。
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
_Static_assert((SIGSEP_FREQUENCY_MODE == SIGSEP_FREQ_MODE_GRID_5KHZ) ||
               (SIGSEP_FREQUENCY_MODE == SIGSEP_FREQ_MODE_CONTINUOUS) ||
               (SIGSEP_FREQUENCY_MODE == SIGSEP_FREQ_MODE_PRECISE_FFT),
               "invalid frequency identification mode");
_Static_assert(SIGSEP_FREQ_COUNT >= 2U,
               "frequency search requires at least two bins");
_Static_assert((SIGSEP_CONTINUOUS_CAPTURE_LEN %
                SIGSEP_ADC_DMA_HALF_LEN) == 0U,
               "continuous capture must contain complete ADC DMA halves");
_Static_assert(SIGSEP_FINE_FREQ_STEP_HZ > 0U,
               "fine frequency step must be positive");
_Static_assert(SIGSEP_FINE_FREQ_STEP_HZ < SIGSEP_FREQ_STEP_HZ,
               "fine frequency step must be smaller than coarse grid");

typedef struct
{
  uint32_t frequency_hz;       /* 分量频率，单位 Hz。 */
  uint32_t frequency_millihz;  /* 高精度初始频率，单位 0.001 Hz。 */
  uint8_t frequency_index;     /* 在候选频率表中的索引。 */
  signal_wave_type_t wave;     /* 正弦波或三角波。 */
  float amplitude_adc;         /* 输入分量 ADC 峰值估计。 */
  float amplitude_dac;         /* 限幅后的 DAC 输出峰值。 */
  uint32_t phase_q32;          /* 输入分量在帧起点的 Q32 相位。 */
  float mean_adc;              /* 长记录直流中心，仅供单信号低频过零锁相使用。 */
  uint8_t low_frequency_path;  /* 1 表示首次识别使用了抽取低频长记录。 */
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

#if (SIGSEP_OPERATION_MODE == SIGSEP_MODE_SINGLE)
typedef struct
{
  uint32_t center_adc;             /* 自适应输入中心电平。 */
  uint32_t hysteresis_adc;         /* 上升过零的施密特迟滞半宽。 */
  uint16_t previous_sample;        /* 上一个连续原始 ADC 样点。 */
  uint16_t cycle_minimum;          /* 当前完整周期的最小样点。 */
  uint16_t cycle_maximum;          /* 当前完整周期的最大样点。 */
  uint8_t previous_valid;          /* previous_sample 是否连续有效。 */
  uint8_t armed;                   /* 已经越过下门限，允许寻找下一次上升过零。 */
  uint8_t candidate_valid;         /* 已记录中心上升穿越，等待越过上门限确认。 */
  uint8_t last_crossing_valid;     /* 是否已有上一周期的确认过零时刻。 */
  uint64_t previous_sample_index;  /* previous_sample 的绝对样点编号。 */
  uint64_t monitor_start_sample;   /* 本次过零有效性监视的起始原始样点。 */
  uint64_t candidate_crossing_q16; /* 待确认中心上升穿越，单位 1/65536 样点。 */
  uint64_t last_crossing_q16;      /* 上一次确认过零，单位 1/65536 样点。 */
  uint64_t filtered_period_q16;    /* 平滑后的周期，单位 1/65536 样点。 */
} signal_low_lock_state_t;
#endif

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
static uint16_t adc_analysis_buffer[SIGSEP_ADC_DMA_HALF_LEN];

#if (SIGSEP_FREQUENCY_MODE == SIGSEP_FREQ_MODE_CONTINUOUS)
/*
 * 连续频率模式只在首次识别期间使用该 CPU 缓冲区。它位于普通 SRAM，不参与
 * DMA，因此不需要额外 Cache 维护；8 个完整半区提供比 500 点粗帧更高的频率
 * 分辨率，并避免把同一离栅格主峰的旁瓣误认成第二路信号。
 */
static uint16_t identify_sample_buffer[SIGSEP_CONTINUOUS_CAPTURE_LEN];
static uint32_t identify_sample_count;
#if (SIGSEP_OPERATION_MODE == SIGSEP_MODE_DUAL_MIXED)
/* 第一主分量消除后的长记录，用于避免把其谱泄漏误判为第二路输入。 */
static uint16_t identify_residual_buffer[SIGSEP_CONTINUOUS_CAPTURE_LEN];
#endif
#endif

/* H743 用 FPU 只在启动阶段生成下列查找表，实时路径不调用 sinf/cosf。 */
static int16_t sine_lut[SIGSEP_SINE_LUT_SIZE];
static float step_sine[SIGSEP_MAX_BIN + 1U];
static float step_cosine[SIGSEP_MAX_BIN + 1U];
static float identify_amplitude_accumulator[SIGSEP_FREQ_COUNT];
static float tracking_step_sine[2];
static float tracking_step_cosine[2];

/* 主循环拥有的分量、NCO 和统计状态。 */
static signal_component_t active_component[2];
static signal_nco_state_t nco_state[2];
#if (SIGSEP_OPERATION_MODE == SIGSEP_MODE_SINGLE)
static signal_low_lock_state_t low_lock_state;
#endif
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
 * @param frequency_millihz 目标频率，单位 0.001 Hz。
 * @return Q32 相位步进，2^32 代表一个完整周期。
 */
static uint32_t phase_step_q32_millihz(uint32_t frequency_millihz)
{
  return (uint32_t)(((uint64_t)frequency_millihz * 4294967296ULL) /
                    ((uint64_t)SIGSEP_SAMPLE_RATE_HZ * 1000ULL));
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
 * @brief 生成任意整数频率对应的单样点正弦/余弦递推系数。
 * @param frequency_hz 目标频率，单位 Hz。
 * @param sine_step 非空的正弦系数输出。
 * @param cosine_step 非空的余弦系数输出。
 * @return 无。
 * @note 对原 5 kHz 栅格直接复用启动查找表；只有连续频率和谐波测量才调用
 *       sinf()/cosf()，活动通道的系数会在 nco_init() 中缓存。
 */
static void frequency_step_coefficients(uint32_t frequency_hz,
                                        float *sine_step,
                                        float *cosine_step)
{
  uint32_t bin = frequency_hz / SIGSEP_FREQ_STEP_HZ;

  if ((bin > 0U) && (bin <= SIGSEP_MAX_BIN) &&
      ((frequency_hz % SIGSEP_FREQ_STEP_HZ) == 0U))
  {
    *sine_step = step_sine[bin];
    *cosine_step = step_cosine[bin];
  }
  else
  {
    float angle = signal_two_pi_f *
                  ((float)frequency_hz / (float)SIGSEP_SAMPLE_RATE_HZ);

    *sine_step = sinf(angle);
    *cosine_step = cosf(angle);
  }
}

/**
 * @brief 生成毫赫兹频率对应的单样点正弦/余弦递推系数。
 * @param frequency_millihz 目标频率，单位 0.001 Hz。
 * @param sine_step 非空的正弦系数输出。
 * @param cosine_step 非空的余弦系数输出。
 * @return 无。
 * @note 整数赫兹直接复用原函数；小数频率只在识别完成和重新识别时计算一次，
 *       不进入每个采样点的实时路径。
 */
static void frequency_step_coefficients_millihz(
  uint32_t frequency_millihz, float *sine_step, float *cosine_step)
{
  if ((frequency_millihz % 1000U) == 0U)
  {
    frequency_step_coefficients(frequency_millihz / 1000U,
                                sine_step, cosine_step);
  }
  else
  {
    float angle = (float)(((double)signal_two_pi_f *
                           (double)frequency_millihz) /
                          ((double)SIGSEP_SAMPLE_RATE_HZ * 1000.0));

    *sine_step = sinf(angle);
    *cosine_step = cosf(angle);
  }
}

/**
 * @brief 计算指定长度 ADC 数据的直流平均值。
 * @param samples ADC 数据。
 * @param sample_count 样点数量。
 * @return ADC 码均值。
 */
static float sample_mean(const uint16_t *samples, uint32_t sample_count)
{
  float mean = 0.0f;
  uint32_t index;

  for (index = 0U; index < sample_count; index++)
  {
    mean += (float)samples[index];
  }

  return mean / (float)sample_count;
}

/**
 * @brief 计算一帧 500 点 ADC 分析数据的直流平均值。
 * @param samples 至少包含 500 点的 ADC 数据。
 * @return ADC 码均值。
 */
static float frame_mean(const uint16_t *samples)
{
  return sample_mean(samples, SIGSEP_ANALYSIS_FRAME_LEN);
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
 * @brief 使用给定递推系数测量任意长度数据的幅值和相位。
 * @param samples ADC 数据。
 * @param sample_count 样点数量。
 * @param mean 已计算的直流均值。
 * @param sine_step 单样点正弦递推系数。
 * @param cosine_step 单样点余弦递推系数。
 * @param amplitude 非空的幅值输出。
 * @param phase_q32 非空的 Q32 相位输出。
 * @return 无。
 */
static void measure_component_with_step(const uint16_t *samples,
                                        uint32_t sample_count,
                                        float mean,
                                        float sine_step,
                                        float cosine_step,
                                        float *amplitude,
                                        uint32_t *phase_q32)
{
  float sine_value = 0.0f;
  float cosine_value = 1.0f;
  float sample_sum = 0.0f;
  float sine_sum = 0.0f;
  float cosine_sum = 0.0f;
  float sine_square_sum = 0.0f;
  float cosine_square_sum = 0.0f;
  float sine_cosine_sum = 0.0f;
  float sample_sine_sum = 0.0f;
  float sample_cosine_sum = 0.0f;
  float inverse_count;
  float centered_sine_square;
  float centered_cosine_square;
  float centered_sine_cosine;
  float centered_sample_sine;
  float centered_sample_cosine;
  float determinant;
  float sine_coefficient;
  float cosine_coefficient;
  uint32_t index;

  if ((samples == NULL) || (amplitude == NULL) ||
      (phase_q32 == NULL) || (sample_count == 0U))
  {
    return;
  }

  for (index = 0U; index < sample_count; index++)
  {
    float sample = (float)samples[index] - mean;
    float next_cosine;

    sample_sum += sample;
    sine_sum += sine_value;
    cosine_sum += cosine_value;
    sine_square_sum += sine_value * sine_value;
    cosine_square_sum += cosine_value * cosine_value;
    sine_cosine_sum += sine_value * cosine_value;
    sample_sine_sum += sample * sine_value;
    sample_cosine_sum += sample * cosine_value;
    next_cosine = (cosine_value * cosine_step) -
                  (sine_value * sine_step);
    sine_value = (sine_value * cosine_step) +
                 (cosine_value * sine_step);
    cosine_value = next_cosine;
  }

  /*
   * 同时拟合 x=a*sin(wt)+b*cos(wt)+dc。与直接假设正弦/余弦在窗口内正交相比，
   * 该 2x2 中心化最小二乘在 1 kHz、500 点仅 0.2 周期时仍能正确剥离直流并
   * 得到幅相；在原 5 kHz 正交栅格上会自然退化为原相关公式。
   */
  inverse_count = 1.0f / (float)sample_count;
  centered_sine_square =
    sine_square_sum - (sine_sum * sine_sum * inverse_count);
  centered_cosine_square =
    cosine_square_sum - (cosine_sum * cosine_sum * inverse_count);
  centered_sine_cosine =
    sine_cosine_sum - (sine_sum * cosine_sum * inverse_count);
  centered_sample_sine =
    sample_sine_sum - (sample_sum * sine_sum * inverse_count);
  centered_sample_cosine =
    sample_cosine_sum - (sample_sum * cosine_sum * inverse_count);
  determinant =
    (centered_sine_square * centered_cosine_square) -
    (centered_sine_cosine * centered_sine_cosine);

  if (determinant <=
      (1.0e-6f *
       ((centered_sine_square * centered_cosine_square) + 1.0f)))
  {
    *amplitude = 0.0f;
    *phase_q32 = 0U;
    return;
  }

  sine_coefficient =
    ((centered_sample_sine * centered_cosine_square) -
     (centered_sample_cosine * centered_sine_cosine)) /
    determinant;
  cosine_coefficient =
    ((centered_sample_cosine * centered_sine_square) -
     (centered_sample_sine * centered_sine_cosine)) /
    determinant;

  *amplitude = sqrtf((sine_coefficient * sine_coefficient) +
                     (cosine_coefficient * cosine_coefficient));
  *phase_q32 = phase_from_iq(sine_coefficient, cosine_coefficient);
}

#if (SIGSEP_OPERATION_MODE == SIGSEP_MODE_DUAL_MIXED)
/**
 * @brief 用带部分主元的高斯消元求解 4x4 线性方程。
 * @param matrix 4x4 系数矩阵。
 * @param vector 4x1 右端向量。
 * @param solution 非空的 4x1 解向量。
 * @return 矩阵可解返回 1，接近奇异返回 0。
 */
static uint8_t solve_linear_system_4x4(const float matrix[4][4],
                                       const float vector[4],
                                       float solution[4])
{
  float augmented[4][5];
  uint32_t column;
  uint32_t row;

  for (row = 0U; row < 4U; row++)
  {
    for (column = 0U; column < 4U; column++)
    {
      augmented[row][column] = matrix[row][column];
    }
    augmented[row][4] = vector[row];
  }

  for (column = 0U; column < 4U; column++)
  {
    uint32_t pivot_row = column;
    float pivot_magnitude = fabsf(augmented[column][column]);

    for (row = column + 1U; row < 4U; row++)
    {
      float candidate_magnitude = fabsf(augmented[row][column]);

      if (candidate_magnitude > pivot_magnitude)
      {
        pivot_magnitude = candidate_magnitude;
        pivot_row = row;
      }
    }
    if (pivot_magnitude < 1.0e-6f)
    {
      return 0U;
    }

    if (pivot_row != column)
    {
      for (row = column; row < 5U; row++)
      {
        float temporary = augmented[column][row];
        augmented[column][row] = augmented[pivot_row][row];
        augmented[pivot_row][row] = temporary;
      }
    }

    {
      float inverse_pivot = 1.0f / augmented[column][column];

      for (row = column; row < 5U; row++)
      {
        augmented[column][row] *= inverse_pivot;
      }
    }

    for (row = 0U; row < 4U; row++)
    {
      uint32_t element;
      float factor;

      if (row == column)
      {
        continue;
      }

      factor = augmented[row][column];
      for (element = column; element < 5U; element++)
      {
        augmented[row][element] -=
          factor * augmented[column][element];
      }
    }
  }

  for (row = 0U; row < 4U; row++)
  {
    solution[row] = augmented[row][4];
  }
  return 1U;
}

/**
 * @brief 在同一混合帧中联合拟合两个已知频率的幅值和相位。
 * @param samples ADC 混合信号。
 * @param sample_count 样点数量。
 * @param mean 当前帧直流均值。
 * @param sine_step 两个频率的正弦递推系数。
 * @param cosine_step 两个频率的余弦递推系数。
 * @param amplitude 两路幅值输出。
 * @param phase_q32 两路 Q32 相位输出。
 * @return 联合方程求解成功返回 1；输入无效或矩阵接近奇异返回 0。
 * @note 联合求解四个正交系数和一个隐式直流项，避免任意双音不再满足 500 点
 *       正交条件时，一个分量泄漏到另一个 PLL 的相位检测器。
 */
static uint8_t measure_components_dual_with_step(
  const uint16_t *samples, uint32_t sample_count, float mean,
  const float sine_step[2], const float cosine_step[2],
  float amplitude[2], uint32_t phase_q32[2])
{
  float sine_value[2] = {0.0f, 0.0f};
  float cosine_value[2] = {1.0f, 1.0f};
  float reference_sum[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float sample_reference_sum[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float gram[4][4] = {{0.0f}};
  float centered_gram[4][4];
  float centered_vector[4];
  float coefficient[4];
  float sample_sum = 0.0f;
  float inverse_count;
  uint32_t sample_index;
  uint32_t row;
  uint32_t column;

  if ((samples == NULL) || (sine_step == NULL) ||
      (cosine_step == NULL) || (amplitude == NULL) ||
      (phase_q32 == NULL) || (sample_count == 0U))
  {
    return 0U;
  }
  amplitude[0] = 0.0f;
  amplitude[1] = 0.0f;
  phase_q32[0] = 0U;
  phase_q32[1] = 0U;

  for (sample_index = 0U; sample_index < sample_count; sample_index++)
  {
    float reference[4] =
    {
      sine_value[0], cosine_value[0],
      sine_value[1], cosine_value[1]
    };
    float sample = (float)samples[sample_index] - mean;
    uint32_t channel;

    sample_sum += sample;
    for (row = 0U; row < 4U; row++)
    {
      reference_sum[row] += reference[row];
      sample_reference_sum[row] += sample * reference[row];
      for (column = row; column < 4U; column++)
      {
        gram[row][column] += reference[row] * reference[column];
      }
    }

    for (channel = 0U; channel < 2U; channel++)
    {
      float next_cosine =
        (cosine_value[channel] * cosine_step[channel]) -
        (sine_value[channel] * sine_step[channel]);

      sine_value[channel] =
        (sine_value[channel] * cosine_step[channel]) +
        (cosine_value[channel] * sine_step[channel]);
      cosine_value[channel] = next_cosine;
    }
  }

  inverse_count = 1.0f / (float)sample_count;
  for (row = 0U; row < 4U; row++)
  {
    centered_vector[row] =
      sample_reference_sum[row] -
      (sample_sum * reference_sum[row] * inverse_count);
    for (column = 0U; column < 4U; column++)
    {
      float raw_gram = (row <= column) ?
                       gram[row][column] : gram[column][row];

      centered_gram[row][column] =
        raw_gram -
        (reference_sum[row] * reference_sum[column] * inverse_count);
    }
  }

  if (solve_linear_system_4x4(centered_gram, centered_vector,
                              coefficient) == 0U)
  {
    /*
     * 不退回两次独立拟合：矩阵奇异时独立测量会重新引入双音串扰。返回无效
     * 幅度可让首次识别重新收集，或让运行中的两路 PLL 冻结在上次可靠状态。
     */
    return 0U;
  }

  amplitude[0] = sqrtf((coefficient[0] * coefficient[0]) +
                       (coefficient[1] * coefficient[1]));
  amplitude[1] = sqrtf((coefficient[2] * coefficient[2]) +
                       (coefficient[3] * coefficient[3]));
  phase_q32[0] = phase_from_iq(coefficient[0], coefficient[1]);
  phase_q32[1] = phase_from_iq(coefficient[2], coefficient[3]);
  return 1U;
}
#endif

#if !((SIGSEP_OPERATION_MODE == SIGSEP_MODE_SINGLE) && \
      (SIGSEP_FREQUENCY_MODE == SIGSEP_FREQ_MODE_PRECISE_FFT))
/**
 * @brief 测量指定整数频率在任意长度数据中的幅值和相位。
 * @param samples ADC 分析帧。
 * @param sample_count 样点数量。
 * @param mean 已计算的直流均值。
 * @param frequency_hz 待测频率。
 * @param amplitude 非空的幅值输出。
 * @param phase_q32 非空的 Q32 相位输出。
 * @return 无。
 */
static void measure_component_length(const uint16_t *samples,
                                     uint32_t sample_count,
                                     float mean,
                                     uint32_t frequency_hz,
                                     float *amplitude,
                                     uint32_t *phase_q32)
{
  float sine_step;
  float cosine_step;

  if ((frequency_hz == 0U) ||
      (frequency_hz > (SIGSEP_SAMPLE_RATE_HZ / 2U)) ||
      (sample_count == 0U))
  {
    *amplitude = 0.0f;
    *phase_q32 = 0U;
    return;
  }

  frequency_step_coefficients(frequency_hz, &sine_step, &cosine_step);
  measure_component_with_step(samples, sample_count, mean,
                              sine_step, cosine_step,
                              amplitude, phase_q32);
}

/**
 * @brief 测量一个 500 点分析帧内的任意整数频率分量。
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
  measure_component_length(samples, SIGSEP_ANALYSIS_FRAME_LEN, mean,
                           frequency_hz, amplitude, phase_q32);
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
 * @brief 计算两个无符号频率的绝对差。
 * @param first_hz 第一个频率。
 * @param second_hz 第二个频率。
 * @return 绝对频差，单位 Hz。
 */
static uint32_t frequency_distance_hz(uint32_t first_hz,
                                      uint32_t second_hz)
{
  return (first_hz >= second_hz) ?
         (first_hz - second_hz) : (second_hz - first_hz);
}
#endif

#if (SIGSEP_FREQUENCY_MODE == SIGSEP_FREQ_MODE_CONTINUOUS)
/**
 * @brief 测量长采样记录中指定频率的幅值。
 * @param samples ADC 长采样记录。
 * @param sample_count 样点数量。
 * @param mean 长记录直流均值。
 * @param frequency_hz 待测频率。
 * @return ADC 峰值估计。
 */
static float measure_amplitude_length(const uint16_t *samples,
                                      uint32_t sample_count,
                                      float mean,
                                      uint32_t frequency_hz)
{
  float amplitude;
  uint32_t unused_phase;

  measure_component_length(samples, sample_count, mean, frequency_hz,
                           &amplitude, &unused_phase);
  return amplitude;
}

/**
 * @brief 判断细搜索频点是否避开已经识别的另一分量。
 * @param frequency_hz 当前候选频率。
 * @param blocked_frequency_hz 需要避开的已识别频率；0 表示不屏蔽。
 * @return 可以使用返回 1，否则返回 0。
 */
static uint8_t fine_frequency_allowed(uint32_t frequency_hz,
                                      uint32_t blocked_frequency_hz)
{
  if (blocked_frequency_hz == 0U)
  {
    return 1U;
  }

  return (frequency_distance_hz(frequency_hz, blocked_frequency_hz) >=
          SIGSEP_DUAL_MIN_SEPARATION_HZ) ? 1U : 0U;
}

/**
 * @brief 在一个 5 kHz 粗峰附近细化任意输入频率。
 * @param samples 连续长采样记录。
 * @param mean 长记录直流均值。
 * @param coarse_frequency_hz 5 kHz 粗搜索频率。
 * @param blocked_frequency_hz 双信号时需要避开的另一主峰，0 表示不屏蔽。
 * @return 细化后的整数 Hz 频率。
 * @note 先按 250 Hz 扫描，再对离散最大值及左右邻点做抛物线插值。最终残差
 *       由原 PI 型 PLL 消除，不改变其增益、积分泄漏和限幅逻辑。
 */
static uint32_t refine_frequency(const uint16_t *samples,
                                 float mean,
                                 uint32_t coarse_frequency_hz,
                                 uint32_t blocked_frequency_hz)
{
  uint32_t lower_hz;
  uint32_t upper_hz;
  uint32_t frequency_hz;
  uint32_t best_frequency_hz = coarse_frequency_hz;
  float best_amplitude = -1.0f;

  lower_hz = (coarse_frequency_hz > SIGSEP_FINE_SEARCH_RADIUS_HZ) ?
             (coarse_frequency_hz - SIGSEP_FINE_SEARCH_RADIUS_HZ) :
             SIGSEP_FREQ_MIN_HZ;
  if (lower_hz < SIGSEP_FREQ_MIN_HZ)
  {
    lower_hz = SIGSEP_FREQ_MIN_HZ;
  }

  upper_hz = coarse_frequency_hz + SIGSEP_FINE_SEARCH_RADIUS_HZ;
  if (upper_hz > (SIGSEP_FREQ_MIN_HZ +
                  ((SIGSEP_FREQ_COUNT - 1U) * SIGSEP_FREQ_STEP_HZ)))
  {
    upper_hz = SIGSEP_FREQ_MIN_HZ +
               ((SIGSEP_FREQ_COUNT - 1U) * SIGSEP_FREQ_STEP_HZ);
  }

  for (frequency_hz = lower_hz; frequency_hz <= upper_hz;
       frequency_hz += SIGSEP_FINE_FREQ_STEP_HZ)
  {
    float amplitude;

    if (fine_frequency_allowed(frequency_hz,
                               blocked_frequency_hz) == 0U)
    {
      continue;
    }

    amplitude = measure_amplitude_length(
      samples, SIGSEP_CONTINUOUS_CAPTURE_LEN, mean, frequency_hz);
    if (amplitude > best_amplitude)
    {
      best_amplitude = amplitude;
      best_frequency_hz = frequency_hz;
    }

    if ((upper_hz - frequency_hz) < SIGSEP_FINE_FREQ_STEP_HZ)
    {
      break;
    }
  }

  if ((best_amplitude >= 0.0f) &&
      (best_frequency_hz >= (lower_hz + SIGSEP_FINE_FREQ_STEP_HZ)) &&
      (best_frequency_hz <= (upper_hz - SIGSEP_FINE_FREQ_STEP_HZ)) &&
      (fine_frequency_allowed(
         best_frequency_hz - SIGSEP_FINE_FREQ_STEP_HZ,
         blocked_frequency_hz) != 0U) &&
      (fine_frequency_allowed(
         best_frequency_hz + SIGSEP_FINE_FREQ_STEP_HZ,
         blocked_frequency_hz) != 0U))
  {
    float left_amplitude = measure_amplitude_length(
      samples, SIGSEP_CONTINUOUS_CAPTURE_LEN, mean,
      best_frequency_hz - SIGSEP_FINE_FREQ_STEP_HZ);
    float right_amplitude = measure_amplitude_length(
      samples, SIGSEP_CONTINUOUS_CAPTURE_LEN, mean,
      best_frequency_hz + SIGSEP_FINE_FREQ_STEP_HZ);
    float denominator = left_amplitude -
                        (2.0f * best_amplitude) +
                        right_amplitude;

    if (fabsf(denominator) > 0.001f)
    {
      float offset_bins =
        0.5f * (left_amplitude - right_amplitude) / denominator;
      float refined_hz;

      if (offset_bins > 0.5f)
      {
        offset_bins = 0.5f;
      }
      if (offset_bins < -0.5f)
      {
        offset_bins = -0.5f;
      }

      refined_hz = (float)best_frequency_hz +
                   (offset_bins * (float)SIGSEP_FINE_FREQ_STEP_HZ);
      if (refined_hz < (float)SIGSEP_FREQ_MIN_HZ)
      {
        refined_hz = (float)SIGSEP_FREQ_MIN_HZ;
      }
      if (refined_hz >
          (float)(SIGSEP_FREQ_MIN_HZ +
                  ((SIGSEP_FREQ_COUNT - 1U) * SIGSEP_FREQ_STEP_HZ)))
      {
        refined_hz =
          (float)(SIGSEP_FREQ_MIN_HZ +
                  ((SIGSEP_FREQ_COUNT - 1U) * SIGSEP_FREQ_STEP_HZ));
      }
      best_frequency_hz = (uint32_t)(refined_hz + 0.5f);
    }
  }

  return best_frequency_hz;
}

#if (SIGSEP_OPERATION_MODE == SIGSEP_MODE_DUAL_MIXED)
/**
 * @brief 从长记录中消除已细化的第一主分量基波。
 * @param samples 原始连续采样记录。
 * @param mean 原始记录直流均值。
 * @param frequency_hz 第一主分量频率。
 * @param residual_buffer 非空的残差输出缓冲区。
 * @return 无。
 * @note 只消除第一分量基波，不抹掉其三/五次谐波，后续波形分类仍使用原始数据。
 */
static void build_residual_capture(const uint16_t *samples,
                                   float mean,
                                   uint32_t frequency_hz,
                                   uint16_t *residual_buffer)
{
  float amplitude;
  uint32_t phase_q32;
  float phase_angle;
  float sine_value;
  float cosine_value;
  float sine_step;
  float cosine_step;
  uint32_t index;

  measure_component_length(samples, SIGSEP_CONTINUOUS_CAPTURE_LEN, mean,
                           frequency_hz, &amplitude, &phase_q32);
  phase_angle = (float)(((double)phase_q32 *
                         (double)signal_two_pi_f) /
                        signal_q32_scale_d);
  sine_value = sinf(phase_angle);
  cosine_value = cosf(phase_angle);
  frequency_step_coefficients(frequency_hz, &sine_step, &cosine_step);

  for (index = 0U; index < SIGSEP_CONTINUOUS_CAPTURE_LEN; index++)
  {
    float residual =
      ((float)samples[index] - mean) -
      (amplitude * sine_value) + 32768.0f;
    float next_cosine;

    if (residual < 0.0f)
    {
      residual = 0.0f;
    }
    if (residual > 65535.0f)
    {
      residual = 65535.0f;
    }
    residual_buffer[index] = (uint16_t)(residual + 0.5f);

    next_cosine = (cosine_value * cosine_step) -
                  (sine_value * sine_step);
    sine_value = (sine_value * cosine_step) +
                 (cosine_value * sine_step);
    cosine_value = next_cosine;
  }
}

/**
 * @brief 在残差长记录的 8 个 500 点片段上非相干平均粗频点幅值。
 * @param samples 第一主分量消除后的连续记录。
 * @param frequency_hz 5 kHz 粗搜索频率。
 * @return 平均 ADC 峰值估计。
 */
static float measure_segmented_amplitude(const uint16_t *samples,
                                         uint32_t frequency_hz)
{
  float amplitude_sum = 0.0f;
  uint32_t frame;

  for (frame = 0U; frame < SIGSEP_CONTINUOUS_CAPTURE_FRAMES; frame++)
  {
    const uint16_t *frame_samples =
      &samples[frame * SIGSEP_ADC_DMA_HALF_LEN];
    float mean = frame_mean(frame_samples);

    amplitude_sum +=
      measure_amplitude_only(frame_samples, mean, frequency_hz);
  }

  return amplitude_sum / (float)SIGSEP_CONTINUOUS_CAPTURE_FRAMES;
}
#endif
#endif

#if !((SIGSEP_OPERATION_MODE == SIGSEP_MODE_SINGLE) && \
      (SIGSEP_FREQUENCY_MODE == SIGSEP_FREQ_MODE_PRECISE_FFT))
/**
 * @brief 判断另一分量是否与当前基波的指定谐波重合。
 * @param other_frequency_hz 另一主分量频率。
 * @param fundamental_frequency_hz 当前基波频率。
 * @param harmonic_order 谐波次数。
 * @return 重合返回 1，否则返回 0。
 */
static uint8_t frequency_overlaps_harmonic(uint32_t other_frequency_hz,
                                           uint32_t fundamental_frequency_hz,
                                           uint32_t harmonic_order)
{
  uint32_t harmonic_frequency_hz =
    fundamental_frequency_hz * harmonic_order;

#if (SIGSEP_FREQUENCY_MODE == SIGSEP_FREQ_MODE_CONTINUOUS)
  return (frequency_distance_hz(other_frequency_hz,
                                harmonic_frequency_hz) <=
          SIGSEP_FINE_FREQ_STEP_HZ) ? 1U : 0U;
#elif (SIGSEP_FREQUENCY_MODE == SIGSEP_FREQ_MODE_PRECISE_FFT)
  /*
   * 高精度主频以毫赫兹保存，但波形分类接口使用四舍五入后的整数 Hz。三倍频
   * 两端各自取整最多产生约 2 Hz 组合误差，因此用 2 Hz 容差识别谐波重合。
   */
  return (frequency_distance_hz(other_frequency_hz,
                                harmonic_frequency_hz) <= 2U) ? 1U : 0U;
#else
  return (frequency_distance_hz(other_frequency_hz,
                                harmonic_frequency_hz) == 0U) ? 1U : 0U;
#endif
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
  if (frequency_overlaps_harmonic(other_frequency_hz,
                                  frequency_hz, 3U) != 0U)
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
  if ((frequency_overlaps_harmonic(other_frequency_hz,
                                   frequency_hz, 3U) != 0U) &&
      (harmonic5_amplitude >
       (fundamental_amplitude * SIGSEP_TRI_H5_RATIO)))
  {
    return signal_wave_triangle;
  }
  if ((frequency_overlaps_harmonic(other_frequency_hz,
                                   frequency_hz, 3U) == 0U) &&
      (harmonic3_amplitude >
       (fundamental_amplitude * SIGSEP_TRI_H3_RATIO)))
  {
    return signal_wave_triangle;
  }
#endif

  return signal_wave_sine;
}
#endif

#if ((SIGSEP_OPERATION_MODE == SIGSEP_MODE_SINGLE) && \
     (SIGSEP_FREQUENCY_MODE == SIGSEP_FREQ_MODE_PRECISE_FFT))
/**
 * @brief 用完整长记录的三次/五次谐波比例判别单信号波形。
 * @param harmonic3_ratio 三次谐波/基波幅值比；超出奈奎斯特时为 0。
 * @param harmonic5_ratio 五次谐波/基波幅值比；超出奈奎斯特时为 0。
 * @return 正弦波、三角波或方波。
 * @note 理想方波的 H3/H1=1/3、H5/H1=1/5；理想三角波分别为 1/9、1/25。
 *       两个判据取“任一可靠谐波超过门限”，从而在高频五次谐波不可测时仍可使用三次谐波。
 */
static signal_wave_type_t detect_wave_type_from_long_record(
  float harmonic3_ratio, float harmonic5_ratio)
{
  if ((harmonic3_ratio >= SIGSEP_SQUARE_H3_RATIO) ||
      ((harmonic5_ratio >= SIGSEP_SQUARE_H5_RATIO) &&
       (harmonic3_ratio >= (SIGSEP_SQUARE_H3_RATIO * 0.5f))))
  {
    return signal_wave_square;
  }
  if ((harmonic3_ratio >= SIGSEP_TRI_H3_RATIO) ||
      ((harmonic5_ratio >= SIGSEP_TRI_H5_RATIO) &&
       (harmonic3_ratio >= (SIGSEP_TRI_H3_RATIO * 0.5f))))
  {
    return signal_wave_triangle;
  }
  return signal_wave_sine;
}
#endif

#if (SIGSEP_FREQUENCY_MODE == SIGSEP_FREQ_MODE_GRID_5KHZ)
/**
 * @brief 使用原 5 kHz 栅格方案搜索单个主分量或两个混合分量。
 * @param samples ADC 分析帧。
 * @param output 最多两个分量的输出数组；单信号模式只使用 output[0]。
 * @return 累积到指定帧数并获得当前模式所需分量时返回 1，否则返回 0。
 */
static uint8_t analyze_grid_frame(const uint16_t *samples,
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
  output[0].frequency_millihz = output[0].frequency_hz * 1000U;
  output[0].amplitude_adc = amplitude[best0];
  output[0].phase_q32 = phase[best0];
  output[0].wave = detect_wave_type(samples, mean,
                                    output[0].frequency_hz, 0U,
                                    output[0].amplitude_adc);

  output[1].frequency_index = 0U;
  output[1].frequency_hz = 0U;
  output[1].frequency_millihz = 0U;
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
    output[0].frequency_millihz = output[0].frequency_hz * 1000U;
    output[0].amplitude_adc = amplitude[best0];
    output[0].phase_q32 = phase[best0];

    output[1].frequency_index = (uint8_t)best1;
    output[1].frequency_hz = SIGSEP_FREQ_MIN_HZ +
                             (best1 * SIGSEP_FREQ_STEP_HZ);
    output[1].frequency_millihz = output[1].frequency_hz * 1000U;
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
#endif

#if (SIGSEP_FREQUENCY_MODE == SIGSEP_FREQ_MODE_CONTINUOUS)
/**
 * @brief 用连续长采样记录细化粗搜索结果并生成活动分量。
 * @param current_samples 当前最后一个 ADC 半区，供相位和波形检测使用。
 * @param coarse_amplitude 8 帧非相干平均后的 5 kHz 粗搜索幅值。
 * @param output 最多两个分量的输出数组。
 * @return 获得当前模式所需有效分量返回 1，否则返回 0。
 */
static uint8_t analyze_continuous_capture(
  const uint16_t *current_samples,
  const float coarse_amplitude[SIGSEP_FREQ_COUNT],
  signal_component_t output[2])
{
  float long_mean = sample_mean(identify_sample_buffer,
                                SIGSEP_CONTINUOUS_CAPTURE_LEN);
  float current_mean = frame_mean(current_samples);
  uint32_t best0 = 0U;
#if (SIGSEP_OPERATION_MODE == SIGSEP_MODE_DUAL_MIXED)
  uint32_t best1 = UINT32_MAX;
  float best1_coarse_amplitude = -1.0f;
#endif
  uint32_t frequency0_hz;
#if (SIGSEP_OPERATION_MODE == SIGSEP_MODE_DUAL_MIXED)
  uint32_t frequency1_hz = 0U;
#endif
  uint32_t index;
  uint32_t component_count;

  for (index = 1U; index < SIGSEP_FREQ_COUNT; index++)
  {
    if (coarse_amplitude[index] > coarse_amplitude[best0])
    {
      best0 = index;
    }
  }
  if (coarse_amplitude[best0] < SIGSEP_MIN_VALID_ADC_AMP)
  {
    return 0U;
  }

  frequency0_hz = refine_frequency(
    identify_sample_buffer, long_mean,
    SIGSEP_FREQ_MIN_HZ + (best0 * SIGSEP_FREQ_STEP_HZ), 0U);

#if (SIGSEP_OPERATION_MODE == SIGSEP_MODE_DUAL_MIXED)
  /*
   * 先细化并消除最强主峰，再在残差记录上寻找第二峰。这样一个离开 5 kHz
   * 栅格的强信号不会因谱泄漏同时占据 best0/best1，也不会压住较弱的第二路。
   */
  build_residual_capture(identify_sample_buffer, long_mean,
                         frequency0_hz, identify_residual_buffer);
  for (index = 0U; index < SIGSEP_FREQ_COUNT; index++)
  {
    uint32_t candidate_hz =
      SIGSEP_FREQ_MIN_HZ + (index * SIGSEP_FREQ_STEP_HZ);
    float candidate_amplitude;

    if (frequency_distance_hz(candidate_hz, frequency0_hz) <
        SIGSEP_DUAL_MIN_SEPARATION_HZ)
    {
      continue;
    }

    candidate_amplitude =
      measure_segmented_amplitude(identify_residual_buffer, candidate_hz);
    if ((best1 == UINT32_MAX) ||
        (candidate_amplitude > best1_coarse_amplitude))
    {
      best1 = index;
      best1_coarse_amplitude = candidate_amplitude;
    }
  }

  if ((best1 == UINT32_MAX) ||
      (best1_coarse_amplitude < SIGSEP_MIN_VALID_ADC_AMP))
  {
    return 0U;
  }

  frequency1_hz = refine_frequency(
    identify_residual_buffer,
    sample_mean(identify_residual_buffer, SIGSEP_CONTINUOUS_CAPTURE_LEN),
    SIGSEP_FREQ_MIN_HZ + (best1 * SIGSEP_FREQ_STEP_HZ),
    frequency0_hz);
  if (frequency_distance_hz(frequency0_hz, frequency1_hz) <
      SIGSEP_DUAL_MIN_SEPARATION_HZ)
  {
    return 0U;
  }

  /* 两路 DAC 固定按低频到高频排序，与分量幅值强弱无关。 */
  if (frequency0_hz > frequency1_hz)
  {
    uint32_t temporary_frequency = frequency0_hz;
    uint32_t temporary_index = best0;

    frequency0_hz = frequency1_hz;
    frequency1_hz = temporary_frequency;
    best0 = best1;
    best1 = temporary_index;
  }
  component_count = 2U;
#else
  component_count = 1U;
#endif

  output[0].frequency_index = (uint8_t)best0;
  output[0].frequency_hz = frequency0_hz;
  output[0].frequency_millihz = frequency0_hz * 1000U;

#if (SIGSEP_OPERATION_MODE == SIGSEP_MODE_DUAL_MIXED)
  {
    float sine_step[2];
    float cosine_step[2];
    float measured_amplitude[2];
    uint32_t measured_phase[2];

    frequency_step_coefficients(frequency0_hz,
                                &sine_step[0], &cosine_step[0]);
    frequency_step_coefficients(frequency1_hz,
                                &sine_step[1], &cosine_step[1]);
    if (measure_components_dual_with_step(
          current_samples, SIGSEP_ANALYSIS_FRAME_LEN, current_mean,
          sine_step, cosine_step,
          measured_amplitude, measured_phase) == 0U)
    {
      return 0U;
    }
    output[0].amplitude_adc = measured_amplitude[0];
    output[0].phase_q32 = measured_phase[0];
    output[1].amplitude_adc = measured_amplitude[1];
    output[1].phase_q32 = measured_phase[1];
  }
  output[1].frequency_index = (uint8_t)best1;
  output[1].frequency_hz = frequency1_hz;
  output[1].frequency_millihz = frequency1_hz * 1000U;
  output[0].wave = detect_wave_type(current_samples, current_mean,
                                    output[0].frequency_hz,
                                    output[1].frequency_hz,
                                    output[0].amplitude_adc);
  output[1].wave = detect_wave_type(current_samples, current_mean,
                                    output[1].frequency_hz,
                                    output[0].frequency_hz,
                                    output[1].amplitude_adc);
#else
  measure_component(current_samples, current_mean, frequency0_hz,
                    &output[0].amplitude_adc, &output[0].phase_q32);
  output[0].wave = detect_wave_type(current_samples, current_mean,
                                    output[0].frequency_hz, 0U,
                                    output[0].amplitude_adc);
  output[1].frequency_index = 0U;
  output[1].frequency_hz = 0U;
  output[1].frequency_millihz = 0U;
  output[1].wave = signal_wave_sine;
  output[1].amplitude_adc = 0.0f;
  output[1].amplitude_dac = 0.0f;
  output[1].phase_q32 = 0U;
#endif

  for (index = 0U; index < component_count; index++)
  {
    output[index].amplitude_dac =
      output[index].amplitude_adc * SIGSEP_ADC_TO_DAC_SCALE;
  }

  return 1U;
}
#endif

/**
 * @brief 按频率识别超参数执行原栅格方案或连续频率粗到细方案。
 * @param samples 当前完整 ADC DMA 半区；前 500 点供原相关检测使用。
 * @param output 最多两个分量的输出数组。
 * @return 当前模式完成识别返回 1，否则返回 0。
 */
static uint8_t analyze_frame(const uint16_t *samples,
                             signal_component_t output[2])
{
  memset(output, 0, 2U * sizeof(signal_component_t));
#if (SIGSEP_FREQUENCY_MODE == SIGSEP_FREQ_MODE_GRID_5KHZ)
  return analyze_grid_frame(samples, output);
#elif (SIGSEP_FREQUENCY_MODE == SIGSEP_FREQ_MODE_CONTINUOUS)
  float mean = frame_mean(samples);
  float coarse_amplitude[SIGSEP_FREQ_COUNT];
  uint32_t unused_phase;
  uint32_t index;
  uint8_t result;

  for (index = 0U; index < SIGSEP_FREQ_COUNT; index++)
  {
    uint32_t frequency_hz = SIGSEP_FREQ_MIN_HZ +
                            (index * SIGSEP_FREQ_STEP_HZ);

    measure_component(samples, mean, frequency_hz,
                      &coarse_amplitude[index], &unused_phase);
    identify_amplitude_accumulator[index] += coarse_amplitude[index];
  }

  memcpy(&identify_sample_buffer[identify_sample_count], samples,
         SIGSEP_ADC_DMA_HALF_LEN * sizeof(uint16_t));
  identify_sample_count += SIGSEP_ADC_DMA_HALF_LEN;
  identify_frame_count++;
  if (identify_sample_count < SIGSEP_CONTINUOUS_CAPTURE_LEN)
  {
    return 0U;
  }

  for (index = 0U; index < SIGSEP_FREQ_COUNT; index++)
  {
    coarse_amplitude[index] =
      identify_amplitude_accumulator[index] /
      (float)identify_frame_count;
    identify_amplitude_accumulator[index] = 0.0f;
  }
  identify_frame_count = 0U;
  identify_sample_count = 0U;

  result = analyze_continuous_capture(samples, coarse_amplitude, output);
  return result;
#else
  frequency_estimator_result_t estimate;
#if (SIGSEP_OPERATION_MODE == SIGSEP_MODE_DUAL_MIXED)
  float mean;
#endif
  uint32_t channel;
#if (SIGSEP_OPERATION_MODE == SIGSEP_MODE_SINGLE)
  const uint8_t requested_components = 1U;
#else
  const uint8_t requested_components = 2U;
#endif

  if (frequency_estimator_push(samples, SIGSEP_ADC_DMA_HALF_LEN,
                               requested_components, &estimate) == 0U)
  {
    return 0U;
  }

#if (SIGSEP_OPERATION_MODE == SIGSEP_MODE_DUAL_MIXED)
  mean = frame_mean(samples);
#endif
  for (channel = 0U; channel < requested_components; channel++)
  {
    output[channel].frequency_millihz =
      estimate.frequency_millihz[channel];
    output[channel].frequency_hz =
      (estimate.frequency_millihz[channel] + 500U) / 1000U;
  }

#if (SIGSEP_OPERATION_MODE == SIGSEP_MODE_DUAL_MIXED)
  {
    float sine_step[2];
    float cosine_step[2];
    float measured_amplitude[2];
    uint32_t measured_phase[2];

    for (channel = 0U; channel < 2U; channel++)
    {
      frequency_step_coefficients_millihz(
        output[channel].frequency_millihz,
        &sine_step[channel], &cosine_step[channel]);
    }
    if (measure_components_dual_with_step(
          samples, SIGSEP_ANALYSIS_FRAME_LEN, mean,
          sine_step, cosine_step,
          measured_amplitude, measured_phase) == 0U)
    {
      return 0U;
    }
    for (channel = 0U; channel < 2U; channel++)
    {
      output[channel].amplitude_adc = measured_amplitude[channel];
      output[channel].phase_q32 = measured_phase[channel];
    }
  }
  output[0].wave = detect_wave_type(samples, mean,
                                    output[0].frequency_hz,
                                    output[1].frequency_hz,
                                    output[0].amplitude_adc);
  output[1].wave = detect_wave_type(samples, mean,
                                    output[1].frequency_hz,
                                    output[0].frequency_hz,
                                    output[1].amplitude_adc);
#else
  /*
   * 单信号首次幅相和波形类型全部使用与判频相同的长记录。不能退回最后 500 点：
   * 1 kHz 在 500 点内仅有 0.2 周期，会把纯基波投影成虚假的三次谐波。
   */
  output[0].amplitude_adc = estimate.amplitude_adc[0];
  output[0].phase_q32 = estimate.phase_q32[0];
  output[0].mean_adc = estimate.mean_adc;
  output[0].low_frequency_path = estimate.low_frequency_path;
  output[0].wave =
    detect_wave_type_from_long_record(estimate.harmonic3_ratio[0],
                                      estimate.harmonic5_ratio[0]);
#endif

  for (channel = 0U; channel < requested_components; channel++)
  {
    output[channel].amplitude_dac =
      output[channel].amplitude_adc * SIGSEP_ADC_TO_DAC_SCALE;
  }
  return 1U;
#endif
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
  frequency_step_coefficients_millihz(
    component->frequency_millihz,
    &tracking_step_sine[channel], &tracking_step_cosine[channel]);
  nco_state[channel].nominal_step =
    phase_step_q32_millihz(component->frequency_millihz);
  nco_state[channel].step_correction = 0;
  nco_state[channel].integrator = 0;
  nco_state[channel].phase_reference = component->phase_q32;
  nco_state[channel].sample_reference = sample_start;
  nco_state[channel].last_error = 0;

#if (SIGSEP_OPERATION_MODE == SIGSEP_MODE_SINGLE)
  if ((channel == 0U) &&
      (component->low_frequency_path != 0U) &&
      (component->frequency_hz <= SIGSEP_LOW_LOCK_MAX_HZ))
  {
    float hysteresis = component->amplitude_adc * 0.08f;

    memset(&low_lock_state, 0, sizeof(low_lock_state));
    if (component->mean_adc < 0.0f)
    {
      low_lock_state.center_adc = 0U;
    }
    else if (component->mean_adc > 65535.0f)
    {
      low_lock_state.center_adc = 65535U;
    }
    else
    {
      low_lock_state.center_adc =
        (uint32_t)(component->mean_adc + 0.5f);
    }
    if (hysteresis < 64.0f)
    {
      hysteresis = 64.0f;
    }
    else if (hysteresis > 4096.0f)
    {
      hysteresis = 4096.0f;
    }
    low_lock_state.hysteresis_adc = (uint32_t)(hysteresis + 0.5f);
    low_lock_state.cycle_minimum = UINT16_MAX;
    low_lock_state.monitor_start_sample = sample_start;
  }
#endif
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
 * @param destination_millihz 跟随通道频率，单位 0.001 Hz。
 * @param source_millihz 主通道频率，单位 0.001 Hz。
 * @return 跟随通道的有符号 Q32 相位误差。
 */
static int32_t scale_phase_error_by_frequency(int32_t phase_error,
                                              uint32_t destination_millihz,
                                              uint32_t source_millihz)
{
  int64_t scaled;

  if (source_millihz == 0U)
  {
    return 0;
  }

  scaled = ((int64_t)phase_error * (int64_t)destination_millihz) /
           (int64_t)source_millihz;
  return (int32_t)((uint32_t)scaled);
}

/**
 * @brief 先在宽整数域缩放主相位误差并施加环路增益，最后才折回 Q32。
 * @param phase_error 主通道有符号 Q32 相位误差。
 * @param destination_millihz 跟随通道频率，单位 0.001 Hz。
 * @param source_millihz 主通道频率，单位 0.001 Hz。
 * @return 跟随通道本帧应施加的有符号 Q32 相位修正。
 * @note 不可先把倍频后的结果折回 int32_t 再除以 2，否则大相位误差在整数倍频
 *       场景会产生 180° 分支错误。
 */
static int32_t scale_phase_adjustment_by_frequency(
  int32_t phase_error, uint32_t destination_millihz,
  uint32_t source_millihz)
{
  int64_t scaled_adjustment;

  if (source_millihz == 0U)
  {
    return 0;
  }

  scaled_adjustment =
    ((int64_t)phase_error * (int64_t)destination_millihz) /
    (int64_t)source_millihz;
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
      active_component[follower_channel].frequency_millihz,
      active_component[master_channel].frequency_millihz);
  int32_t follower_phase_adjust =
    scale_phase_adjustment_by_frequency(
      master_phase_error,
      active_component[follower_channel].frequency_millihz,
      active_component[master_channel].frequency_millihz);

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
#if (SIGSEP_FREQUENCY_MODE == SIGSEP_FREQ_MODE_CONTINUOUS)
  identify_sample_count = 0U;
#endif
  frequency_estimator_reset();
  memset(active_component, 0, sizeof(active_component));
  memset(nco_state, 0, sizeof(nco_state));
#if (SIGSEP_OPERATION_MODE == SIGSEP_MODE_SINGLE)
  memset(&low_lock_state, 0, sizeof(low_lock_state));
#endif
  memset(tracking_step_sine, 0, sizeof(tracking_step_sine));
  memset(tracking_step_cosine, 0, sizeof(tracking_step_cosine));

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

/**
 * @brief 把一次可靠的幅相测量送入指定通道的 PLL 和幅度平滑器。
 * @param channel 待更新的输出通道索引。
 * @param measured_amplitude 当前帧最小二乘拟合得到的 ADC 峰值。
 * @param measured_phase 当前帧起点的 Q32 输入相位。
 * @param frame_start_sample 当前帧第一个样点的绝对采样点编号。
 * @return 无。
 * @note 幅度低于有效阈值时冻结 PLL 和相位状态，避免拔掉输入后把噪声相位积分
 *       进 NCO；幅度显示仍缓慢衰减，便于诊断信号丢失。
 */
static void update_tracked_component(uint32_t channel,
                                     float measured_amplitude,
                                     uint32_t measured_phase,
                                     uint64_t frame_start_sample)
{
  if (measured_amplitude >= SIGSEP_MIN_VALID_ADC_AMP)
  {
    (void)nco_update_lock(channel, measured_phase, frame_start_sample);
    active_component[channel].phase_q32 = measured_phase;
  }

  active_component[channel].amplitude_adc +=
    (measured_amplitude - active_component[channel].amplitude_adc) /
    (float)(1UL << SIGSEP_AMP_SMOOTH_SHIFT);
  update_dac_amplitude(channel, 1U);
}

#if (SIGSEP_OPERATION_MODE == SIGSEP_MODE_SINGLE)
/**
 * @brief 确认一次低频上升中点过零，并更新周期、NCO 频率和相位。
 * @param crossing_q16 过零绝对时刻，单位为 1/65536 个原始 ADC 样点。
 * @return 无。
 * @note 正弦、三角和方波在上升中点处都定义为相位 0，因此该相位检测器与三种
 *       DAC 重建波形共用同一相位基准，不依赖不足一周期的短帧正交拟合。
 */
static void low_lock_accept_crossing(uint64_t crossing_q16)
{
  uint8_t period_valid = 0U;
  int64_t target_correction = 0LL;
  int32_t maximum_correction = 0;

  if (low_lock_state.last_crossing_valid != 0U)
  {
    uint64_t measured_period_q16 =
      crossing_q16 - low_lock_state.last_crossing_q16;
    uint64_t expected_period_q16 =
      (((uint64_t)SIGSEP_SAMPLE_RATE_HZ * 1000ULL) << 16U) /
      (uint64_t)active_component[0].frequency_millihz;

    if ((measured_period_q16 >= (expected_period_q16 / 2ULL)) &&
        (measured_period_q16 <=
         (expected_period_q16 + (expected_period_q16 / 2ULL))))
    {
      uint64_t measured_step;
      maximum_correction =
        (int32_t)(nco_state[0].nominal_step /
                  SIGSEP_PLL_MAX_CORR_DIV);

      if (low_lock_state.filtered_period_q16 == 0ULL)
      {
        low_lock_state.filtered_period_q16 = measured_period_q16;
      }
      else
      {
        low_lock_state.filtered_period_q16 =
          ((low_lock_state.filtered_period_q16 * 3ULL) +
           measured_period_q16) / 4ULL;
      }

      measured_step =
        (1ULL << 48U) / low_lock_state.filtered_period_q16;
      target_correction =
        (int64_t)measured_step -
        (int64_t)nco_state[0].nominal_step;
      if (maximum_correction < 1)
      {
        maximum_correction = 1;
      }
      if (target_correction > (int64_t)maximum_correction)
      {
        target_correction = maximum_correction;
      }
      else if (target_correction < -(int64_t)maximum_correction)
      {
        target_correction = -(int64_t)maximum_correction;
      }
      period_valid = 1U;
    }
  }

  /*
   * 首次交越只建立相位和周期基准；已有基准后，只有落入 0.5T～1.5T
   * 合法窗口的交越才能修正相位并刷新超时计时。这样噪声或振铃产生的
   * 假交越不会让 DAC 瞬时跳相，也不会掩盖真正的失锁超时。
   */
  if ((low_lock_state.last_crossing_valid != 0U) &&
      (period_valid == 0U))
  {
    return;
  }

  {
    uint64_t crossing_sample = crossing_q16 >> 16U;
    uint32_t crossing_fraction = (uint32_t)(crossing_q16 & 0xFFFFULL);
    uint32_t predicted_at_sample =
      nco_phase_at_sample(0U, crossing_sample);
    uint32_t predicted_at_crossing =
      predicted_at_sample +
      (uint32_t)(((uint64_t)nco_step(0U) * crossing_fraction) >> 16U);
    int32_t phase_error = (int32_t)(0U - predicted_at_crossing);

    nco_state[0].integrator = 0;
    nco_state[0].phase_reference =
      predicted_at_sample + (uint32_t)(phase_error / 2);
    nco_state[0].sample_reference = crossing_sample;
    nco_state[0].last_error = phase_error;
    active_component[0].phase_q32 = 0U;
  }

  /*
   * 先用旧步进把 NCO 重基准到本次过零，再修改频率修正；若先改步进，
   * nco_phase_at_sample() 会把新步进错误地追溯到整个旧参考区间并制造相位跳变。
   */
  if (period_valid != 0U)
  {
    nco_state[0].step_correction +=
      (int32_t)((target_correction -
                 (int64_t)nco_state[0].step_correction) / 4LL);
  }

  if ((period_valid != 0U) &&
      (low_lock_state.cycle_maximum > low_lock_state.cycle_minimum))
  {
    float measured_amplitude =
      ((float)low_lock_state.cycle_maximum -
       (float)low_lock_state.cycle_minimum) * 0.5f;
    float measured_center =
      ((float)low_lock_state.cycle_maximum +
       (float)low_lock_state.cycle_minimum) * 0.5f;
    float hysteresis = measured_amplitude * 0.08f;

    active_component[0].amplitude_adc +=
      (measured_amplitude - active_component[0].amplitude_adc) /
      (float)(1UL << SIGSEP_AMP_SMOOTH_SHIFT);
    low_lock_state.center_adc = (uint32_t)(measured_center + 0.5f);
    if (hysteresis < 64.0f)
    {
      hysteresis = 64.0f;
    }
    else if (hysteresis > 4096.0f)
    {
      hysteresis = 4096.0f;
    }
    low_lock_state.hysteresis_adc = (uint32_t)(hysteresis + 0.5f);
    update_dac_amplitude(0U, 1U);
  }

  low_lock_state.last_crossing_q16 = crossing_q16;
  low_lock_state.last_crossing_valid = 1U;
  low_lock_state.monitor_start_sample = crossing_q16 >> 16U;
  low_lock_state.cycle_minimum = UINT16_MAX;
  low_lock_state.cycle_maximum = 0U;
}

/**
 * @brief 用带迟滞确认和线性插值的上升中点过零持续锁定 40 Hz～1 kHz 单信号。
 * @param samples 当前完整 512 点原始 ADC DMA 半区。
 * @param frame_start_sample 当前块首点绝对编号。
 * @return 无。
 */
static void track_component_low_frequency(const uint16_t *samples,
                                          uint64_t frame_start_sample)
{
  uint32_t lower_threshold =
    (low_lock_state.center_adc > low_lock_state.hysteresis_adc) ?
    (low_lock_state.center_adc - low_lock_state.hysteresis_adc) : 0U;
  uint32_t upper_threshold =
    low_lock_state.center_adc + low_lock_state.hysteresis_adc;
  uint32_t index;

  if (upper_threshold > UINT16_MAX)
  {
    upper_threshold = UINT16_MAX;
  }
  if (low_lock_state.previous_valid == 0U)
  {
    /*
     * 首次识别的两次 FFT 可能让主循环积压很多 DMA 事件；超时监视必须从恢复
     * 实时处理的第一块数据开始，而不是从执行 FFT 前的旧帧开始。
     */
    low_lock_state.monitor_start_sample = frame_start_sample;
  }
  if ((low_lock_state.previous_valid != 0U) &&
      (frame_start_sample !=
       (low_lock_state.previous_sample_index + 1ULL)))
  {
    low_lock_state.previous_valid = 0U;
    low_lock_state.armed = 0U;
    low_lock_state.candidate_valid = 0U;
    low_lock_state.last_crossing_valid = 0U;
    low_lock_state.filtered_period_q16 = 0ULL;
    low_lock_state.monitor_start_sample = frame_start_sample;
  }

  for (index = 0U; index < SIGSEP_ADC_DMA_HALF_LEN; index++)
  {
    uint16_t sample = samples[index];
    uint64_t sample_index = frame_start_sample + index;

    if (sample < low_lock_state.cycle_minimum)
    {
      low_lock_state.cycle_minimum = sample;
    }
    if (sample > low_lock_state.cycle_maximum)
    {
      low_lock_state.cycle_maximum = sample;
    }

    if (sample <= lower_threshold)
    {
      low_lock_state.armed = 1U;
      low_lock_state.candidate_valid = 0U;
    }

    if ((low_lock_state.previous_valid != 0U) &&
        (low_lock_state.armed != 0U) &&
        (low_lock_state.candidate_valid == 0U) &&
        (low_lock_state.previous_sample < low_lock_state.center_adc) &&
        (sample >= low_lock_state.center_adc) &&
        (sample > low_lock_state.previous_sample))
    {
      uint32_t sample_delta =
        (uint32_t)sample - (uint32_t)low_lock_state.previous_sample;
      uint32_t center_delta =
        low_lock_state.center_adc -
        (uint32_t)low_lock_state.previous_sample;
      uint32_t fraction_q16 =
        (uint32_t)(((uint64_t)center_delta << 16U) / sample_delta);

      low_lock_state.candidate_crossing_q16 =
        ((sample_index - 1ULL) << 16U) + fraction_q16;
      low_lock_state.candidate_valid = 1U;
    }

    if ((low_lock_state.candidate_valid != 0U) &&
        (sample >= upper_threshold))
    {
      low_lock_accept_crossing(low_lock_state.candidate_crossing_q16);
      low_lock_state.armed = 0U;
      low_lock_state.candidate_valid = 0U;
    }

    low_lock_state.previous_sample = sample;
    low_lock_state.previous_sample_index = sample_index;
    low_lock_state.previous_valid = 1U;
  }

  {
    uint64_t current_end_sample =
      frame_start_sample + SIGSEP_ADC_DMA_HALF_LEN;
    uint64_t expected_period_samples =
      ((uint64_t)SIGSEP_SAMPLE_RATE_HZ * 1000ULL) /
      (uint64_t)active_component[0].frequency_millihz;

    /*
     * 连续约 2.5 个周期没有确认过零，视为拔掉输入或波形超出门限，回到重新识别；
     * 避免 PA4 永久沿用最后一次 NCO 频率继续输出一个看似“锁定”的旧波形。
     */
    if ((current_end_sample - low_lock_state.monitor_start_sample) >
        ((expected_period_samples * 5ULL) / 2ULL))
    {
      signal_separation_restart_identify();
    }
  }
}

/**
 * @brief 用与双通道独立 PLL 相同的顺序跟踪单信号分量。
 * @param channel 待跟踪的输出通道索引，单信号模式固定为 0。
 * @param samples 当前 ADC 分析帧。
 * @param mean 当前分析帧的直流均值。
 * @param frame_start_sample 当前帧第一个样点的绝对采样点编号。
 * @return 无。
 */
static void track_component_independent(uint32_t channel,
                                        const uint16_t *samples,
                                        float mean,
                                        uint64_t frame_start_sample)
{
  float measured_amplitude;
  uint32_t measured_phase;

  measure_component_with_step(samples, SIGSEP_ANALYSIS_FRAME_LEN, mean,
                              tracking_step_sine[channel],
                              tracking_step_cosine[channel],
                              &measured_amplitude, &measured_phase);
  update_tracked_component(channel, measured_amplitude, measured_phase,
                           frame_start_sample);
}
#endif

#if ((SIGSEP_OPERATION_MODE == SIGSEP_MODE_DUAL_MIXED) && \
     (SIGSEP_COMMON_SOURCE_LOCK == 0U))
/**
 * @brief 在同一混合帧内联合测量两路分量，再分别更新两个独立 PLL。
 * @param samples 当前 ADC 混合信号帧。
 * @param mean 当前帧直流均值。
 * @param frame_start_sample 当前帧第一个样点的绝对采样点编号。
 * @return 无。
 * @note 联合 4x4 最小二乘先消除任意双音之间的非正交串扰，之后两路仍使用与
 *       原方案相同的 PI/NCO 闭环，不共享积分器或频率修正量。
 */
static void track_components_independent_dual(
  const uint16_t *samples, float mean, uint64_t frame_start_sample)
{
  float measured_amplitude[2];
  uint32_t measured_phase[2];
  uint32_t channel;

  (void)measure_components_dual_with_step(
    samples, SIGSEP_ANALYSIS_FRAME_LEN, mean,
    tracking_step_sine, tracking_step_cosine,
    measured_amplitude, measured_phase);
  for (channel = 0U; channel < 2U; channel++)
  {
    update_tracked_component(channel, measured_amplitude[channel],
                             measured_phase[channel], frame_start_sample);
  }
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
         SIGSEP_ADC_DMA_HALF_LEN * sizeof(uint16_t));
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
    if ((active_component[0].low_frequency_path != 0U) &&
        (active_component[0].frequency_hz <= SIGSEP_LOW_LOCK_MAX_HZ))
    {
      track_component_low_frequency(samples, frame_start_sample);
    }
    else
    {
      track_component_independent(0U, samples, mean, frame_start_sample);
    }
#else
#if (SIGSEP_COMMON_SOURCE_LOCK != 0U)
    float measured_amplitude[2];
    uint32_t measured_phase[2];
    uint32_t master_channel =
      (SIGSEP_PHASE_MASTER_CH == 0U) ? 0U : 1U;
    uint32_t follower_channel = master_channel ^ 1U;
    int32_t master_phase_error;

    (void)measure_components_dual_with_step(
      samples, SIGSEP_ANALYSIS_FRAME_LEN, mean,
      tracking_step_sine, tracking_step_cosine,
      measured_amplitude, measured_phase);
    if (measured_amplitude[master_channel] >= SIGSEP_MIN_VALID_ADC_AMP)
    {
      master_phase_error =
        nco_update_lock(master_channel, measured_phase[master_channel],
                        frame_start_sample);
      nco_follow_common_source(follower_channel, master_channel,
                               master_phase_error, frame_start_sample);
      active_component[master_channel].phase_q32 =
        measured_phase[master_channel];
      active_component[follower_channel].phase_q32 =
        nco_phase_at_sample(follower_channel, frame_start_sample);
    }
    active_component[0].amplitude_adc +=
      (measured_amplitude[0] - active_component[0].amplitude_adc) /
      (float)(1UL << SIGSEP_AMP_SMOOTH_SHIFT);
    active_component[1].amplitude_adc +=
      (measured_amplitude[1] - active_component[1].amplitude_adc) /
      (float)(1UL << SIGSEP_AMP_SMOOTH_SHIFT);
    update_dac_amplitude(0U, 1U);
    update_dac_amplitude(1U, 1U);
#else
    track_components_independent_dual(samples, mean, frame_start_sample);
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
#if ((SIGSEP_FREQUENCY_MODE == SIGSEP_FREQ_MODE_CONTINUOUS) || \
     (SIGSEP_FREQUENCY_MODE == SIGSEP_FREQ_MODE_PRECISE_FFT))
    /*
     * 长记录中间若缺少 DMA 半区，样点时间已不连续，不能继续做细频率估计。
     * 仅在尚未识别时丢弃这段记录并重新收集；锁定后的 PLL 仍沿绝对样点时间推进。
     */
    if (separation_identified == 0U)
    {
      signal_separation_restart_identify();
    }
#endif
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
#if ((SIGSEP_FREQUENCY_MODE == SIGSEP_FREQ_MODE_CONTINUOUS) || \
     (SIGSEP_FREQUENCY_MODE == SIGSEP_FREQ_MODE_PRECISE_FFT))
    /*
     * 复制期间 DMA 已翻到下一半区也等价于长记录缺帧。首次识别必须丢弃之前
     * 已收集的数据，防止把不连续的 512 点拼接进 FFT 或相位斜率估计。
     */
    if (separation_identified == 0U)
    {
      signal_separation_restart_identify();
    }
#endif
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
#if (SIGSEP_OPERATION_MODE == SIGSEP_MODE_SINGLE)
  memset(&low_lock_state, 0, sizeof(low_lock_state));
#endif
#if (SIGSEP_FREQUENCY_MODE == SIGSEP_FREQ_MODE_CONTINUOUS)
  identify_sample_count = 0U;
#endif
  frequency_estimator_reset();
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
    status->frequency_millihz[channel] =
      active_component[channel].frequency_millihz;
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
