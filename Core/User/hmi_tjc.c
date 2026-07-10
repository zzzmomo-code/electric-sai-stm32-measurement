/**
 * @file hmi_tjc.c
 * @brief 淘晶驰串口屏显示模块实现。
 *
 * 模块用途：低频读取测量结果快照，构建淘晶驰文本指令并通过 UART 轮询发送。
 * GPIO 引脚映射：无硬编码 GPIO；UART 引脚由 CubeMX 和 hmi_tjc_bind_uart() 决定。
 * 依赖的外设和 CubeIDE 配置：运行发送依赖 UART 9600 8N1；不使用 TX DMA 或 UART 中断。
 * 初始化方法：系统启动调用 hmi_tjc_init()，CubeMX UART 初始化后绑定 UART 句柄。
 * 调用方法：主循环调用 hmi_tjc_process()；发送在主循环中完成。
 */

#include "system.h"

#include <math.h>
#include <stdio.h>

/** 单条淘晶驰命令的三个结束字节。 */
#define HMI_TJC_TERMINATOR_BYTE 0xffu

/** 动态文本字段的最大格式化长度，包含结尾空字符。 */
#define HMI_TJC_TEXT_VALUE_SIZE 24u

/** 动态数值字段的固定显示格式。 */
typedef enum
{
    HMI_TJC_VALUE_FORMAT_AMPLITUDE,
    HMI_TJC_VALUE_FORMAT_FREQUENCY_HZ,
    HMI_TJC_VALUE_FORMAT_FREQUENCY_KHZ,
    HMI_TJC_VALUE_FORMAT_PHASE
} hmi_tjc_value_format_t;

/**
 * @brief 将波形枚举转换为串口屏使用的 ASCII 文本。
 * @param wave_type 算法识别的波形类型。
 * @return 静态 ASCII 波形名称。
 * @note 不返回中文，避免运行时字库与字符编码依赖。
 */
static const char *hmi_tjc_wave_type_text(measurement_wave_type_t wave_type)
{
    switch (wave_type)
    {
        case MEASUREMENT_WAVE_SINE:
            return "SINE";

        case MEASUREMENT_WAVE_SQUARE:
            return "SQUARE";

        case MEASUREMENT_WAVE_TRIANGLE:
            return "TRIANGLE";

        case MEASUREMENT_WAVE_UNKNOWN:
        default:
            return "UNKNOWN";
    }
}

/**
 * @brief 判断测量快照是否可安全显示。
 * @param result 待检查的结果指针。
 * @return 可显示返回 1，否则返回 0。
 * @note 无效、非有限浮点数、负幅度或负频率均显示为 FAULT。
 */
static uint8_t hmi_tjc_result_is_displayable(const measurement_result_t *result)
{
    if ((result == 0) || (result->valid == 0u))
    {
        return 0u;
    }

    if ((!isfinite(result->amplitude_vpp))
        || (!isfinite(result->frequency_hz))
        || (!isfinite(result->phase_deg))
        || (result->amplitude_vpp < 0.0f)
        || (result->frequency_hz < 0.0f))
    {
        return 0u;
    }

    return 1u;
}

/**
 * @brief 格式化一个动态文本字段并确认结果未被截断。
 * @param text 用于接收格式化结果的缓冲区。
 * @param text_capacity 缓冲区容量，单位为字节。
 * @param value_format 固定的 ASCII 显示格式。
 * @param value 待格式化的浮点数。
 * @return 完整格式化成功时返回 1，否则返回 0。
 * @note 结果过长时不保留截断文本，调用方应显示 FAULT。
 */
static uint8_t hmi_tjc_format_value(char *text,
                                    uint16_t text_capacity,
                                    hmi_tjc_value_format_t value_format,
                                    double value)
{
    int text_length;

    if ((text == 0) || (text_capacity == 0u))
    {
        return 0u;
    }

    switch (value_format)
    {
        case HMI_TJC_VALUE_FORMAT_AMPLITUDE:
            text_length = snprintf(text, text_capacity, "%.3f Vpp", value);
            break;

        case HMI_TJC_VALUE_FORMAT_FREQUENCY_HZ:
            text_length = snprintf(text, text_capacity, "%.2f Hz", value);
            break;

        case HMI_TJC_VALUE_FORMAT_FREQUENCY_KHZ:
            text_length = snprintf(text, text_capacity, "%.3f kHz", value);
            break;

        case HMI_TJC_VALUE_FORMAT_PHASE:
            text_length = snprintf(text, text_capacity, "%.1f deg", value);
            break;

        default:
            return 0u;
    }

    if ((text_length < 0) || ((uint32_t)text_length >= text_capacity))
    {
        return 0u;
    }

    return 1u;
}

/**
 * @brief 向 HMI 数据帧追加一条全局文本控件赋值命令。
 * @param frame UART 数据帧缓冲区。
 * @param frame_capacity 缓冲区总容量。
 * @param frame_size 当前帧长度及追加后的帧长度。
 * @param control_name main 页面内的文本控件名称。
 * @param text 待显示的 ASCII 文本。
 * @return 追加结果状态。
 * @note 成功时会追加 main.控件.txt="文本" 及三个 0xff 结束字节。
 */
static hmi_tjc_status_t hmi_tjc_append_text_command(
    uint8_t *frame,
    uint16_t frame_capacity,
    uint16_t *frame_size,
    const char *control_name,
    const char *text)
{
    int command_length;
    uint16_t offset;
    uint16_t remaining;

    if ((frame == 0) || (frame_size == 0)
        || (control_name == 0) || (text == 0))
    {
        return HMI_TJC_STATUS_INVALID_ARGUMENT;
    }

    offset = *frame_size;
    if (offset >= frame_capacity)
    {
        return HMI_TJC_STATUS_BUFFER_TOO_SMALL;
    }

    remaining = (uint16_t)(frame_capacity - offset);
    command_length = snprintf((char *)&frame[offset],
                              remaining,
                              "main.%s.txt=\"%s\"",
                              control_name,
                              text);
    if ((command_length < 0)
        || ((uint32_t)command_length >= (uint32_t)remaining))
    {
        return HMI_TJC_STATUS_BUFFER_TOO_SMALL;
    }

    offset = (uint16_t)(offset + (uint16_t)command_length);
    if ((uint16_t)(frame_capacity - offset) < 3u)
    {
        return HMI_TJC_STATUS_BUFFER_TOO_SMALL;
    }

    frame[offset++] = HMI_TJC_TERMINATOR_BYTE;
    frame[offset++] = HMI_TJC_TERMINATOR_BYTE;
    frame[offset++] = HMI_TJC_TERMINATOR_BYTE;
    *frame_size = offset;
    return HMI_TJC_STATUS_OK;
}

/**
 * @brief 构建一帧包含五条淘晶驰文本指令的 UART 数据。
 * @param result 待显示的测量结果快照。
 * @param frame 用于接收二进制 UART 数据的缓冲区。
 * @param frame_capacity 缓冲区容量，单位为字节。
 * @param frame_size 用于接收实际帧长度的指针。
 * @return 帧构建结果状态。
 * @note 无效快照显示 WAIT；异常数值显示 FAULT。
 */
hmi_tjc_status_t hmi_tjc_build_frame(const measurement_result_t *result,
                                     uint8_t *frame,
                                     uint16_t frame_capacity,
                                     uint16_t *frame_size)
{
    char amplitude_text[HMI_TJC_TEXT_VALUE_SIZE];
    char frequency_text[HMI_TJC_TEXT_VALUE_SIZE];
    char phase_text[HMI_TJC_TEXT_VALUE_SIZE];
    const char *wave_text;
    const char *status_text;
    hmi_tjc_status_t status;
    uint8_t displayable;

    if ((result == 0) || (frame == 0) || (frame_size == 0))
    {
        return HMI_TJC_STATUS_INVALID_ARGUMENT;
    }

    displayable = hmi_tjc_result_is_displayable(result);
    if (result->valid == 0u)
    {
        (void)snprintf(amplitude_text, sizeof(amplitude_text), "--");
        (void)snprintf(frequency_text, sizeof(frequency_text), "--");
        (void)snprintf(phase_text, sizeof(phase_text), "--");
        wave_text = "UNKNOWN";
        status_text = "WAIT";
    }
    else if (displayable == 0u)
    {
        (void)snprintf(amplitude_text, sizeof(amplitude_text), "--");
        (void)snprintf(frequency_text, sizeof(frequency_text), "--");
        (void)snprintf(phase_text, sizeof(phase_text), "--");
        wave_text = "UNKNOWN";
        status_text = "FAULT";
    }
    else
    {
        displayable = hmi_tjc_format_value(amplitude_text,
                                           sizeof(amplitude_text),
                                           HMI_TJC_VALUE_FORMAT_AMPLITUDE,
                                           (double)result->amplitude_vpp);
        if (result->frequency_hz < 1000.0f)
        {
             displayable &= hmi_tjc_format_value(frequency_text,
                                                 sizeof(frequency_text),
                                                 HMI_TJC_VALUE_FORMAT_FREQUENCY_HZ,
                                                 (double)result->frequency_hz);
        }
        else
        {
            displayable &= hmi_tjc_format_value(
                 frequency_text,
                 sizeof(frequency_text),
                HMI_TJC_VALUE_FORMAT_FREQUENCY_KHZ,
                 (double)(result->frequency_hz / 1000.0f));
        }
        displayable &= hmi_tjc_format_value(phase_text,
                                             sizeof(phase_text),
                                             HMI_TJC_VALUE_FORMAT_PHASE,
                                             (double)result->phase_deg);

        if (displayable == 0u)
        {
            (void)snprintf(amplitude_text, sizeof(amplitude_text), "--");
            (void)snprintf(frequency_text, sizeof(frequency_text), "--");
            (void)snprintf(phase_text, sizeof(phase_text), "--");
            wave_text = "UNKNOWN";
            status_text = "FAULT";
        }
        else
        {
            wave_text = hmi_tjc_wave_type_text(result->wave_type);
            status_text = "LIVE";
        }
    }

    *frame_size = 0u;
    status = hmi_tjc_append_text_command(frame,
                                         frame_capacity,
                                         frame_size,
                                         "t_amp",
                                         amplitude_text);
    if (status != HMI_TJC_STATUS_OK)
    {
        return status;
    }

    status = hmi_tjc_append_text_command(frame,
                                         frame_capacity,
                                         frame_size,
                                         "t_freq",
                                         frequency_text);
    if (status != HMI_TJC_STATUS_OK)
    {
        return status;
    }

    status = hmi_tjc_append_text_command(frame,
                                         frame_capacity,
                                         frame_size,
                                         "t_phase",
                                         phase_text);
    if (status != HMI_TJC_STATUS_OK)
    {
        return status;
    }

    status = hmi_tjc_append_text_command(frame,
                                         frame_capacity,
                                         frame_size,
                                         "t_wave",
                                         wave_text);
    if (status != HMI_TJC_STATUS_OK)
    {
        return status;
    }

    return hmi_tjc_append_text_command(frame,
                                       frame_capacity,
                                       frame_size,
                                       "t_status",
                                       status_text);
}

#if defined(HAL_UART_MODULE_ENABLED)

/** 绑定的 CubeMX UART 句柄。 */
static UART_HandleTypeDef *hmi_tjc_uart;

/** UART 轮询发送使用的固定帧缓冲区。 */
static uint8_t hmi_tjc_tx_buffer[HMI_TJC_TX_BUFFER_SIZE];

/** 上电后先发送清理帧的标志。 */
static uint8_t hmi_tjc_flush_pending;

/** 上一次成功或尝试刷新屏幕的 HAL 时基。 */
static uint32_t hmi_tjc_last_refresh_ms;

/**
 * @brief 轮询发送一帧 UART 数据。
 * @param frame_size 待发送的帧长度。
 * @return 发送结果状态。
 * @note 此函数在主循环中阻塞等待 UART 发送完成，不使用 DMA 或 D-Cache 维护。
 */
static hmi_tjc_status_t hmi_tjc_send_frame(uint16_t frame_size)
{
    if (hmi_tjc_uart == 0)
    {
        return HMI_TJC_STATUS_UART_UNAVAILABLE;
    }
    if ((frame_size == 0u) || (frame_size > HMI_TJC_TX_BUFFER_SIZE))
    {
        return HMI_TJC_STATUS_INVALID_ARGUMENT;
    }

    if (HAL_UART_Transmit(hmi_tjc_uart,
                          hmi_tjc_tx_buffer,
                          frame_size,
                          HMI_TJC_TX_TIMEOUT_MS) != HAL_OK)
    {
        return HMI_TJC_STATUS_HAL_ERROR;
    }

    return HMI_TJC_STATUS_OK;
}

/**
 * @brief 绑定 CubeMX 生成的 UART 句柄。
 * @param huart 已配置为 9600 8N1 的 UART 句柄。
 * @return 无。
 * @note 重新绑定后在下一次 process 中发送清理帧，无 DMA 或中断依赖。
 */
void hmi_tjc_bind_uart(UART_HandleTypeDef *huart)
{
    hmi_tjc_uart = huart;
    hmi_tjc_flush_pending = 1u;
    hmi_tjc_last_refresh_ms = HAL_GetTick();
}

#endif

/**
 * @brief 初始化 HMI 模块状态。
 * @param 无。
 * @return 无。
 * @note UART 未启用时只保留帧构建能力，主循环调用不会访问串口硬件。
 */
void hmi_tjc_init(void)
{
#if defined(HAL_UART_MODULE_ENABLED)
    hmi_tjc_uart = 0;
    hmi_tjc_flush_pending = 1u;
    hmi_tjc_last_refresh_ms = HAL_GetTick();
#endif
}

/**
 * @brief 处理 HMI 上电清理和周期刷新。
 * @param 无。
 * @return 无。
 * @note UART 尚未配置时无操作；UART 就绪后每 250 ms 最多轮询发送一帧。
 */
void hmi_tjc_process(void)
{
#if defined(HAL_UART_MODULE_ENABLED)
    measurement_result_t result;
    hmi_tjc_status_t status;
    uint16_t frame_size;
    uint32_t now;

    if (hmi_tjc_uart == 0)
    {
        return;
    }

    if (hmi_tjc_flush_pending != 0u)
    {
        hmi_tjc_tx_buffer[0] = 0x00u;
        hmi_tjc_tx_buffer[1] = HMI_TJC_TERMINATOR_BYTE;
        hmi_tjc_tx_buffer[2] = HMI_TJC_TERMINATOR_BYTE;
        hmi_tjc_tx_buffer[3] = HMI_TJC_TERMINATOR_BYTE;
        status = hmi_tjc_send_frame(4u);
        if (status == HMI_TJC_STATUS_OK)
        {
            hmi_tjc_flush_pending = 0u;
        }
        return;
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - hmi_tjc_last_refresh_ms) < HMI_TJC_REFRESH_MS)
    {
        return;
    }
    hmi_tjc_last_refresh_ms = now;

    (void)measurement_result_get_snapshot(&result);
    status = hmi_tjc_build_frame(&result,
                                 hmi_tjc_tx_buffer,
                                 HMI_TJC_TX_BUFFER_SIZE,
                                 &frame_size);
    if (status != HMI_TJC_STATUS_OK)
    {
        return;
    }

    (void)hmi_tjc_send_frame(frame_size);
#endif
}
