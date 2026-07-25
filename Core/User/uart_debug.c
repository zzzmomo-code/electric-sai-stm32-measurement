#include "system.h"

/*
 * 模块用途：在不持续破坏实时性的前提下输出最少量串口诊断信息。
 * GPIO 映射：PB14=USART1_TX，PB15=USART1_RX。
 * 外设依赖：USART1，115200 bit/s，8N1，USART1 全局中断。
 * 初始化方法：由 system_init() 调用 uart_debug_init()。
 * 调用方法：由 system_process() 调用 uart_debug_process()。
 */

static uint8_t reported_identified_state;
static char async_message[160];

#if (SIGSEP_FREQUENCY_MODE == SIGSEP_FREQ_MODE_CONTINUOUS)
#define uart_debug_frequency_mode_line \
  "frequency=continuous, coarse=5000Hz, fine=250Hz+PLL\r\n"
#elif (SIGSEP_FREQUENCY_MODE == SIGSEP_FREQ_MODE_PRECISE_FFT)
#if (SIGSEP_OPERATION_MODE == SIGSEP_MODE_SINGLE)
#define uart_debug_frequency_mode_line \
  "frequency=precise_fft, range=40..400000Hz, acquire=419ms\r\n"
#else
#define uart_debug_frequency_mode_line \
  "frequency=precise_fft, N=32768, Hann+peak+phase\r\n"
#endif
#else
#define uart_debug_frequency_mode_line \
  "frequency=grid_5khz\r\n"
#endif

/**
 * @brief 把波形枚举转换为紧凑的串口名称。
 * @param wave 波形类型。
 * @return 指向只读名称字符串的指针。
 */
static const char *uart_debug_wave_name(signal_wave_type_t wave)
{
  if (wave == signal_wave_triangle)
  {
    return "tri";
  }
  if (wave == signal_wave_square)
  {
    return "square";
  }

  return "sin";
}

/**
 * @brief 向 USART1 阻塞发送一个以零结尾的字符串。
 * @param text 待发送字符串。
 * @return 无。
 * @note 仅在高速 TIM2 启动前或首次识别完成时调用。
 */
static void uart_debug_write(const char *text)
{
  size_t length;

  if (text == NULL)
  {
    return;
  }

  length = strlen(text);
  if (length > UINT16_MAX)
  {
    length = UINT16_MAX;
  }

  (void)HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)length, 100U);
}

/**
 * @brief 输出目标芯片、采样率和引脚映射。
 * @param 无。
 * @return 无。
 */
void uart_debug_init(void)
{
  reported_identified_state = 0U;
  /*
   * CubeMX 的 USART1 NVIC 表已启用全局中断；这里把生成时的默认优先级 0
   * 降到 8，保证 ADC/DAC DMA 的 5/6/7 优先级始终更高。
  */
  HAL_NVIC_SetPriority(USART1_IRQn, 8U, 0U);
#if (SIGSEP_OPERATION_MODE == SIGSEP_MODE_SINGLE)
  uart_debug_write("\r\nH743 phase locking port\r\n"
                   "ADC PC0, DAC PA4/PA5, Fs=2500000Hz\r\n"
                   "mode=single, signal=PA4, PA5=midscale\r\n"
                   uart_debug_frequency_mode_line
                   "command: r=restart identify\r\n"
                   "state=search\r\n");
#else
  uart_debug_write("\r\nH743 phase locking port\r\n"
                   "ADC PC0, DAC PA4/PA5, Fs=2500000Hz\r\n"
                   "mode=dual_mixed, low=PA4, high=PA5\r\n"
                   uart_debug_frequency_mode_line
                   "command: r=restart identify\r\n"
                   "state=search\r\n");
#endif
}

/**
 * @brief 首次识别完成时非阻塞输出频率、波形和累计丢帧数。
 * @param 无。
 * @return 无。
 * @note 使用 HAL_UART_Transmit_IT()，不会在主循环等待整条消息发送完成。
 */
void uart_debug_process(void)
{
  signal_separation_status_t status;
  uint8_t command;
  const char *wave0;
#if (SIGSEP_OPERATION_MODE == SIGSEP_MODE_DUAL_MIXED)
  const char *wave1;
#endif
  int length;

  /*
   * 参考工程由串口屏“分离”按键触发重新识别。本移植工程没有该屏，
   * 因此用 USART1 接收 r/R 作为等价入口；轮询超时为 0，不阻塞主循环。
   */
  if ((HAL_UART_Receive(&huart1, &command, 1U, 0U) == HAL_OK) &&
      ((command == (uint8_t)'r') || (command == (uint8_t)'R')))
  {
    signal_separation_restart_identify();
    reported_identified_state = 0U;
    return;
  }

  if (signal_separation_get_status(&status) == 0U)
  {
    return;
  }
  if (status.identified == 0U)
  {
    reported_identified_state = 0U;
    return;
  }
  /*
   * 第一次看到 identified 时先让出一轮主循环。32768 点 FFT 运行期间可能已经
   * 积压 DMA 事件，下一轮 signal_separation_process() 会先把这些事件计入
   * adc_drop/dac_drop，随后再打印，避免串口报告刚锁定时的陈旧统计值。
   */
  if (reported_identified_state == 0U)
  {
    reported_identified_state = 1U;
    return;
  }
  if (reported_identified_state >= 2U)
  {
    return;
  }

  wave0 = uart_debug_wave_name(status.wave[0]);
#if (SIGSEP_OPERATION_MODE == SIGSEP_MODE_SINGLE)
  length = snprintf(async_message, sizeof(async_message),
                    "locked A=%lu.%03luHz/%s adc_drop=%lu dac_drop=%lu\r\n",
                    (unsigned long)(status.frequency_millihz[0] / 1000U),
                    (unsigned long)(status.frequency_millihz[0] % 1000U),
                    wave0,
                    (unsigned long)status.adc_frame_overrun,
                    (unsigned long)status.dac_half_overrun);
#else
  wave1 = uart_debug_wave_name(status.wave[1]);
  length = snprintf(async_message, sizeof(async_message),
                    "locked A=%lu.%03luHz/%s B=%lu.%03luHz/%s "
                    "adc_drop=%lu dac_drop=%lu\r\n",
                    (unsigned long)(status.frequency_millihz[0] / 1000U),
                    (unsigned long)(status.frequency_millihz[0] % 1000U),
                    wave0,
                    (unsigned long)(status.frequency_millihz[1] / 1000U),
                    (unsigned long)(status.frequency_millihz[1] % 1000U),
                    wave1,
                    (unsigned long)status.adc_frame_overrun,
                    (unsigned long)status.dac_half_overrun);
#endif
  if ((length > 0) && ((size_t)length < sizeof(async_message)) &&
      (HAL_UART_Transmit_IT(&huart1, (uint8_t *)async_message,
                            (uint16_t)length) == HAL_OK))
  {
    reported_identified_state = 2U;
  }
}
