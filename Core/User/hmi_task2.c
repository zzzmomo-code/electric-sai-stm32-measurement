/**
 * @file hmi_task2.c
 * @brief 外差式幅频测量训练题的简洁串口屏实现。
 *
 * 模块用途：把当前测量链中的关键结果发送到淘晶驰串口屏，并处理屏幕按键。
 * GPIO 引脚映射：PA9/USART1_TX 接串口屏 RX，PA10/USART1_RX 接串口屏 TX。
 * 依赖的外设和 CubeIDE 配置：USART1，9600 8N1，开启 USART1 全局中断，不使用 DMA。
 * 初始化方法：system_init() 中先调用 hmi_task2_init()，再调用 hmi_task2_bind_uart()。
 * 调用方法：主循环调用 hmi_task2_process()；中断回调只设置接收或错误标志。
 */

#include "system.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define HMI_TASK2_REFRESH_MS      500u
#define HMI_TASK2_TX_TIMEOUT_MS   500u
#define HMI_TASK2_FRAME_SIZE      384u

volatile hmi_task2_diagnostics_t hmi_task2_diagnostics;

static UART_HandleTypeDef *hmi_task2_uart;
static uint8_t hmi_task2_rx_byte;
static volatile uint8_t hmi_task2_rx_flag;
static volatile uint8_t hmi_task2_error_flag;
static uint8_t hmi_task2_vga_error;
static uint32_t hmi_task2_last_refresh_ms;

/**
 * @brief 向帧缓冲区追加一条文本赋值命令。
 * @param frame 帧缓冲区。
 * @param used 当前已使用字节数。
 * @param object_name 串口屏文本控件名。
 * @param value 要显示的 ASCII 文本。
 * @return 成功返回 1，缓冲区不足返回 0。
 */
static uint8_t hmi_task2_append_text(char *frame, uint16_t *used,
                                     const char *object_name,
                                     const char *value)
{
    int length;
    uint16_t remaining = (uint16_t)(HMI_TASK2_FRAME_SIZE - *used);

    length = snprintf(&frame[*used], remaining, "%s.txt=\"%s\"",
                      object_name, value);
    if ((length < 0) || ((uint16_t)length + 3u > remaining))
    {
        return 0u;
    }

    *used = (uint16_t)(*used + (uint16_t)length);
    frame[(*used)++] = (char)0xff;
    frame[(*used)++] = (char)0xff;
    frame[(*used)++] = (char)0xff;
    return 1u;
}

/**
 * @brief 将频率转换成适合屏幕宽度的文本。
 * @param frequency_hz 频率，单位 Hz。
 * @param text 输出缓冲区。
 * @param text_size 输出缓冲区长度。
 * @return 无。
 */
static void hmi_task2_format_frequency(float frequency_hz,
                                       char *text, size_t text_size)
{
    if ((!isfinite(frequency_hz)) || (frequency_hz <= 0.0f))
    {
        (void)snprintf(text, text_size, "--");
    }
    else if (frequency_hz >= 1000000.0f)
    {
        (void)snprintf(text, text_size, "%.3f MHz",
                       (double)(frequency_hz / 1000000.0f));
    }
    else if (frequency_hz >= 1000.0f)
    {
        (void)snprintf(text, text_size, "%.3f kHz",
                       (double)(frequency_hz / 1000.0f));
    }
    else
    {
        (void)snprintf(text, text_size, "%.1f Hz", (double)frequency_hz);
    }
}

/**
 * @brief 将波形枚举转换成页面文本。
 * @param wave 波形枚举。
 * @return 静态 ASCII 文本。
 */
static const char *hmi_task2_wave_text(measurement_wave_type_t wave)
{
    if (wave == MEASUREMENT_WAVE_SINE)
    {
        return "SINE";
    }
    if (wave == MEASUREMENT_WAVE_SQUARE)
    {
        return "SQUARE";
    }
    return "UNKNOWN";
}

/**
 * @brief 处理一个串口屏按键字节。
 * @param command 按键发送的 ASCII 字节。
 * @return 无。
 */
static void hmi_task2_handle_command(uint8_t command)
{
    hmi_task2_diagnostics.last_command = command;
    hmi_task2_diagnostics.rx_count++;

    if ((command >= (uint8_t)'0') && (command <= (uint8_t)'5'))
    {
        if (dac_output_set_level((uint8_t)(command - (uint8_t)'0'))
            == dac_output_status_ok)
        {
            hmi_task2_vga_error = 0u;
            hmi_task2_diagnostics.command_count++;
        }
        else
        {
            hmi_task2_vga_error = 1u;
            hmi_task2_diagnostics.error_count++;
        }
    }
    else if ((command == (uint8_t)'M') || (command == (uint8_t)'m'))
    {
        frequency_measure_request_now();
        hmi_task2_diagnostics.command_count++;
    }
    else
    {
        hmi_task2_diagnostics.error_count++;
    }
}

/**
 * @brief 构建并发送当前页面的全部动态字段。
 * @param 无。
 * @return 无，结果写入诊断计数。
 */
static void hmi_task2_refresh(void)
{
    measurement_result_t result;
    adc_dual_stats_t adc;
    char frame[HMI_TASK2_FRAME_SIZE];
    char timer_text[24];
    char adc_frequency_text[24];
    char adc_amplitude_text[24];
    char real_frequency_text[24];
    char real_amplitude_text[24];
    char dds_text[24];
    char vga_text[12];
    char overflow_text[16];
    const char *status_text = "WAIT";
    measurement_wave_type_t wave = MEASUREMENT_WAVE_UNKNOWN;
    float timer_frequency_hz = frequency_measure_hz;
    float adc_frequency_hz = 0.0f;
    float adc_amplitude_vpp = 0.0f;
    float real_frequency_hz;
    float real_amplitude_vpp;
    uint32_t dds_frequency_hz = dds_control_diagnostics.output_frequency_hz;
    uint8_t vga_level = dac_output_get_level();
    uint16_t used = 0u;

    (void)measurement_result_get_snapshot(&result);
    (void)adc_dual_get_stats(&adc);

    if ((result.valid_mask & MEASUREMENT_VALID_FREQUENCY) != 0u)
    {
        adc_frequency_hz = result.frequency_hz;
    }
    else if (measurement_fft_diagnostics.fft_ready != 0u)
    {
        adc_frequency_hz = measurement_fft_diagnostics.peak_frequency_hz;
    }

    if ((result.valid_mask & MEASUREMENT_VALID_AMPLITUDE) != 0u)
    {
        adc_amplitude_vpp = result.amplitude_vpp;
    }
    else if (measurement_fft_diagnostics.fft_ready != 0u)
    {
        adc_amplitude_vpp = measurement_fft_diagnostics.amplitude_vpp;
    }

    if ((result.valid_mask & MEASUREMENT_VALID_WAVE_TYPE) != 0u)
    {
        wave = result.wave_type;
    }
    else if (measurement_fft_diagnostics.fft_ready != 0u)
    {
        wave = measurement_fft_diagnostics.wave_type;
    }

    real_frequency_hz = measurement_conversion_frequency_hz(
        timer_frequency_hz, (float)dds_frequency_hz, adc_frequency_hz);
    real_amplitude_vpp = measurement_conversion_amplitude_vpp(
        adc_amplitude_vpp, vga_level);

    if ((adc.state == ADC_DUAL_STATE_ERROR)
        || (dds_control_diagnostics.state == dds_control_state_error)
        || (hmi_task2_vga_error != 0u))
    {
        status_text = "ERROR";
    }
    else if (measurement_fft_diagnostics.clipping_mask != 0u)
    {
        status_text = "CLIP";
    }
    else if ((adc_frequency_hz >= 90000.0f)
             && (adc_frequency_hz <= 110000.0f))
    {
        status_text = "LOCK";
    }
    else if ((timer_frequency_hz > 0.0f)
             || (measurement_fft_diagnostics.fft_ready != 0u))
    {
        status_text = "EST";
    }

    hmi_task2_format_frequency(timer_frequency_hz, timer_text,
                               sizeof(timer_text));
    hmi_task2_format_frequency(adc_frequency_hz, adc_frequency_text,
                               sizeof(adc_frequency_text));
    hmi_task2_format_frequency(real_frequency_hz, real_frequency_text,
                               sizeof(real_frequency_text));
    hmi_task2_format_frequency((float)dds_frequency_hz, dds_text,
                               sizeof(dds_text));

    if (adc_amplitude_vpp > 0.0f)
    {
        (void)snprintf(adc_amplitude_text, sizeof(adc_amplitude_text),
                       "%.3f Vpp", (double)adc_amplitude_vpp);
        (void)snprintf(real_amplitude_text, sizeof(real_amplitude_text),
                       "~%.3f Vpp", (double)real_amplitude_vpp);
    }
    else
    {
        (void)snprintf(adc_amplitude_text, sizeof(adc_amplitude_text), "--");
        (void)snprintf(real_amplitude_text, sizeof(real_amplitude_text), "--");
    }

    (void)snprintf(vga_text, sizeof(vga_text), "G%u", (unsigned)vga_level);
    (void)snprintf(overflow_text, sizeof(overflow_text), "%lu",
                   (unsigned long)adc.overflow_count);

    if ((hmi_task2_append_text(frame, &used, "t_timer_freq", timer_text) == 0u)
        || (hmi_task2_append_text(frame, &used, "t_adc_freq", adc_frequency_text) == 0u)
        || (hmi_task2_append_text(frame, &used, "t_adc_amp", adc_amplitude_text) == 0u)
        || (hmi_task2_append_text(frame, &used, "t_real_freq", real_frequency_text) == 0u)
        || (hmi_task2_append_text(frame, &used, "t_real_amp", real_amplitude_text) == 0u)
        || (hmi_task2_append_text(frame, &used, "t_wave", hmi_task2_wave_text(wave)) == 0u)
        || (hmi_task2_append_text(frame, &used, "t_dds_freq", dds_text) == 0u)
        || (hmi_task2_append_text(frame, &used, "t_vga", vga_text) == 0u)
        || (hmi_task2_append_text(frame, &used, "t_status", status_text) == 0u)
        || (hmi_task2_append_text(frame, &used, "t_overflow", overflow_text) == 0u))
    {
        hmi_task2_diagnostics.tx_error_count++;
        return;
    }

    if (HAL_UART_Transmit(hmi_task2_uart, (uint8_t *)frame, used,
                          HMI_TASK2_TX_TIMEOUT_MS) == HAL_OK)
    {
        hmi_task2_diagnostics.tx_count++;
    }
    else
    {
        hmi_task2_diagnostics.tx_error_count++;
    }
}

void hmi_task2_init(void)
{
    memset((void *)&hmi_task2_diagnostics, 0,
           sizeof(hmi_task2_diagnostics));
    hmi_task2_uart = NULL;
    hmi_task2_rx_byte = 0u;
    hmi_task2_rx_flag = 0u;
    hmi_task2_error_flag = 0u;
    hmi_task2_vga_error = 0u;
    hmi_task2_last_refresh_ms = 0u;
}

#if defined(HAL_UART_MODULE_ENABLED)
void hmi_task2_bind_uart(UART_HandleTypeDef *huart)
{
    hmi_task2_uart = huart;
    if ((huart == NULL)
        || (HAL_UART_Receive_IT(huart, &hmi_task2_rx_byte, 1u) != HAL_OK))
    {
        hmi_task2_diagnostics.error_count++;
    }
}
#endif

void hmi_task2_process(void)
{
    uint32_t now;

    if (hmi_task2_uart == NULL)
    {
        return;
    }

    if (hmi_task2_error_flag != 0u)
    {
        hmi_task2_error_flag = 0u;
        hmi_task2_diagnostics.error_count++;
        (void)HAL_UART_AbortReceive(hmi_task2_uart);
        (void)HAL_UART_Receive_IT(hmi_task2_uart, &hmi_task2_rx_byte, 1u);
    }

    if (hmi_task2_rx_flag != 0u)
    {
        hmi_task2_rx_flag = 0u;
        hmi_task2_handle_command(hmi_task2_rx_byte);
        if (HAL_UART_Receive_IT(hmi_task2_uart, &hmi_task2_rx_byte, 1u)
            != HAL_OK)
        {
            hmi_task2_diagnostics.error_count++;
        }
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - hmi_task2_last_refresh_ms) >= HMI_TASK2_REFRESH_MS)
    {
        hmi_task2_last_refresh_ms = now;
        hmi_task2_refresh();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == hmi_task2_uart)
    {
        hmi_task2_rx_flag = 1u;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == hmi_task2_uart)
    {
        hmi_task2_error_flag = 1u;
    }
}
