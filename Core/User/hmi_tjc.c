/**
 * @file hmi_tjc.c
 * @brief 淘晶驰串口屏显示模块实现。
 *
 * 模块用途：读取测量结果快照，构建淘晶驰文本和频谱指令，并通过 UART 轮询发送基础页面。
 * GPIO 引脚映射：无硬编码 GPIO；UART 引脚由 CubeMX 和 hmi_tjc_bind_uart() 决定。
 * 依赖的外设和 CubeIDE 配置：运行发送依赖 UART 9600 8N1；不使用 TX DMA 或 UART 中断。
 * 初始化方法：系统启动调用 hmi_tjc_init()，CubeMX UART 初始化后绑定 UART 句柄。
 * 调用方法：主循环调用 hmi_tjc_process()；扩展控件准备完成后可调用详细指标和频谱构帧接口。
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
    HMI_TJC_VALUE_FORMAT_DC,
    HMI_TJC_VALUE_FORMAT_AMPLITUDE,
    HMI_TJC_VALUE_FORMAT_RMS,
    HMI_TJC_VALUE_FORMAT_THD,
    HMI_TJC_VALUE_FORMAT_FREQUENCY_HZ,
    HMI_TJC_VALUE_FORMAT_FREQUENCY_KHZ,
    HMI_TJC_VALUE_FORMAT_PHASE
} hmi_tjc_value_format_t;

/** 单个测量通道送往串口屏的五项 ASCII 文本。 */
typedef struct
{
    char amplitude[HMI_TJC_TEXT_VALUE_SIZE]; /**< 峰峰值文本。 */
    char frequency[HMI_TJC_TEXT_VALUE_SIZE]; /**< 频率文本。 */
    char thd[HMI_TJC_TEXT_VALUE_SIZE];       /**< THD 文本。 */
    const char *wave;                        /**< 波形类型静态文本。 */
    const char *status;                      /**< WAIT、LIVE 或 FAULT。 */
} hmi_tjc_channel_text_t;

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
 * @note 只检查 valid_mask 声明为有效的字段，允许单通道结果把相位显示为 --。
 */
static uint8_t hmi_tjc_result_is_displayable(const measurement_result_t *result)
{
    if ((result == 0) || (result->valid_mask == 0u))
    {
        return 0u;
    }

    if (((result->valid_mask & MEASUREMENT_VALID_DC_VOLTAGE) != 0u)
        && (!isfinite(result->dc_voltage)))
    {
        return 0u;
    }
    if (((result->valid_mask & MEASUREMENT_VALID_AMPLITUDE) != 0u)
        && ((!isfinite(result->amplitude_vpp))
            || (result->amplitude_vpp < 0.0f)))
    {
        return 0u;
    }
    if (((result->valid_mask & MEASUREMENT_VALID_FREQUENCY) != 0u)
        && ((!isfinite(result->frequency_hz))
            || (result->frequency_hz < 0.0f)))
    {
        return 0u;
    }
    if (((result->valid_mask & MEASUREMENT_VALID_PHASE) != 0u)
        && (!isfinite(result->phase_deg)))
    {
        return 0u;
    }
    if (((result->valid_mask & MEASUREMENT_VALID_RMS) != 0u)
        && ((!isfinite(result->rms_voltage))
            || (result->rms_voltage < 0.0f)))
    {
        return 0u;
    }
    if (((result->valid_mask & MEASUREMENT_VALID_THD) != 0u)
        && ((!isfinite(result->thd_percent))
            || (result->thd_percent < 0.0f)))
    {
        return 0u;
    }

    return 1u;
}

static hmi_tjc_status_t hmi_tjc_append_terminator(uint8_t *frame,
                                                   uint16_t frame_capacity,
                                                   uint16_t *frame_size);

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
        case HMI_TJC_VALUE_FORMAT_DC:
            text_length = snprintf(text, text_capacity, "%.3f Vdc", value);
            break;

        case HMI_TJC_VALUE_FORMAT_AMPLITUDE:
            text_length = snprintf(text, text_capacity, "%.3f Vpp", value);
            break;

        case HMI_TJC_VALUE_FORMAT_RMS:
            text_length = snprintf(text, text_capacity, "%.3f Vrms", value);
            break;

        case HMI_TJC_VALUE_FORMAT_THD:
            text_length = snprintf(text, text_capacity, "%.2f %%", value);
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
 * @brief 将一个通道的测量字段转换为五项屏幕文本。
 * @param mode 当前通道测量模式。
 * @param valid_mask 当前通道字段有效位。
 * @param fault 非零表示当前通道本帧异常。
 * @param amplitude_vpp 峰峰值，单位为伏。
 * @param frequency_hz 频率，单位为赫兹。
 * @param thd_percent 总谐波失真，单位为百分比。
 * @param wave_type 波形分类结果。
 * @param text 用于接收五项文本的结构体。
 * @return 文本生成成功返回 1，否则返回 0。
 * @note 当前页面的幅度栏明确标为 VPP，直流模式不借用该栏显示 Vdc。
 */
static uint8_t hmi_tjc_prepare_channel_text(
    measurement_mode_t mode,
    uint16_t valid_mask,
    uint8_t fault,
    float amplitude_vpp,
    float frequency_hz,
    float thd_percent,
    measurement_wave_type_t wave_type,
    hmi_tjc_channel_text_t *text)
{
    uint8_t formatted = 1u;

    if (text == 0)
    {
        return 0u;
    }

    (void)snprintf(text->amplitude, sizeof(text->amplitude), "--");
    (void)snprintf(text->frequency, sizeof(text->frequency), "--");
    (void)snprintf(text->thd, sizeof(text->thd), "--");
    text->wave = "UNKNOWN";
    text->status = "WAIT";

    if (fault != 0u)
    {
        text->status = "FAULT";
        return 1u;
    }
    if (valid_mask == 0u)
    {
        return 1u;
    }
    if (mode == MEASUREMENT_MODE_DC)
    {
        text->wave = "DC";
        text->status = "LIVE";
        return 1u;
    }

    if ((valid_mask & MEASUREMENT_VALID_AMPLITUDE) != 0u)
    {
        if ((!isfinite(amplitude_vpp)) || (amplitude_vpp < 0.0f))
        {
            formatted = 0u;
        }
        else
        {
            formatted &= hmi_tjc_format_value(
                text->amplitude,
                sizeof(text->amplitude),
                HMI_TJC_VALUE_FORMAT_AMPLITUDE,
                (double)amplitude_vpp);
        }
    }
    if ((valid_mask & MEASUREMENT_VALID_FREQUENCY) != 0u)
    {
        if ((!isfinite(frequency_hz)) || (frequency_hz < 0.0f))
        {
            formatted = 0u;
        }
        else if (frequency_hz < 1000.0f)
        {
            formatted &= hmi_tjc_format_value(
                text->frequency,
                sizeof(text->frequency),
                HMI_TJC_VALUE_FORMAT_FREQUENCY_HZ,
                (double)frequency_hz);
        }
        else
        {
            formatted &= hmi_tjc_format_value(
                text->frequency,
                sizeof(text->frequency),
                HMI_TJC_VALUE_FORMAT_FREQUENCY_KHZ,
                (double)(frequency_hz / 1000.0f));
        }
    }
    if ((valid_mask & MEASUREMENT_VALID_THD) != 0u)
    {
        if ((!isfinite(thd_percent)) || (thd_percent < 0.0f))
        {
            formatted = 0u;
        }
        else
        {
            formatted &= hmi_tjc_format_value(text->thd,
                                               sizeof(text->thd),
                                               HMI_TJC_VALUE_FORMAT_THD,
                                               (double)thd_percent);
        }
    }
    if ((valid_mask & MEASUREMENT_VALID_WAVE_TYPE) != 0u)
    {
        text->wave = hmi_tjc_wave_type_text(wave_type);
    }

    if (formatted == 0u)
    {
        (void)snprintf(text->amplitude, sizeof(text->amplitude), "--");
        (void)snprintf(text->frequency, sizeof(text->frequency), "--");
        (void)snprintf(text->thd, sizeof(text->thd), "--");
        text->wave = "UNKNOWN";
        text->status = "FAULT";
        return 0u;
    }

    text->status = "LIVE";
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
 * @note 成功时会追加全局控件名.txt="文本" 及三个 0xff 结束字节。
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
                              "%s.txt=\"%s\"",
                              control_name,
                              text);
    if ((command_length < 0)
        || ((uint32_t)command_length >= (uint32_t)remaining))
    {
        return HMI_TJC_STATUS_BUFFER_TOO_SMALL;
    }

    *frame_size = (uint16_t)(offset + (uint16_t)command_length);
    return hmi_tjc_append_terminator(frame, frame_capacity, frame_size);
}

/**
 * @brief 构建一帧包含双通道十一条淘晶驰文本指令的 UART 数据。
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
    hmi_tjc_channel_text_t channel_text[2];
    char phase_text[HMI_TJC_TEXT_VALUE_SIZE];
    const char *control_names[11] = {
        "t_amp", "t_freq", "t_wave", "t_thd", "t_status",
        "t_amp2", "t_freq2", "t_wave2", "t_thd2", "t_status2",
        "t_phase"
    };
    const char *control_text[11];
    hmi_tjc_status_t status;
    uint8_t index;

    if ((result == 0) || (frame == 0) || (frame_size == 0))
    {
        return HMI_TJC_STATUS_INVALID_ARGUMENT;
    }

    (void)hmi_tjc_prepare_channel_text(
        result->mode,
        result->valid_mask,
        (uint8_t)(result->fault_mask & 0x01u),
        result->amplitude_vpp,
        result->frequency_hz,
        result->thd_percent,
        result->wave_type,
        &channel_text[0]);
    (void)hmi_tjc_prepare_channel_text(
        result->secondary_mode,
        result->secondary_valid_mask,
        (uint8_t)(result->fault_mask & 0x02u),
        result->secondary_amplitude_vpp,
        result->secondary_frequency_hz,
        result->secondary_thd_percent,
        result->secondary_wave_type,
        &channel_text[1]);

    (void)snprintf(phase_text, sizeof(phase_text), "--");
    if (((result->valid_mask & MEASUREMENT_VALID_PHASE) != 0u)
        && isfinite(result->phase_deg))
    {
        (void)hmi_tjc_format_value(phase_text,
                                   sizeof(phase_text),
                                   HMI_TJC_VALUE_FORMAT_PHASE,
                                   (double)result->phase_deg);
    }

    control_text[0] = channel_text[0].amplitude;
    control_text[1] = channel_text[0].frequency;
    control_text[2] = channel_text[0].wave;
    control_text[3] = channel_text[0].thd;
    control_text[4] = channel_text[0].status;
    control_text[5] = channel_text[1].amplitude;
    control_text[6] = channel_text[1].frequency;
    control_text[7] = channel_text[1].wave;
    control_text[8] = channel_text[1].thd;
    control_text[9] = channel_text[1].status;
    control_text[10] = phase_text;

    *frame_size = 0u;
    for (index = 0u; index < 11u; index++)
    {
        status = hmi_tjc_append_text_command(frame,
                                             frame_capacity,
                                             frame_size,
                                             control_names[index],
                                             control_text[index]);
        if (status != HMI_TJC_STATUS_OK)
        {
            return status;
        }
    }

    return HMI_TJC_STATUS_OK;
}

/**
 * @brief 向帧尾追加三个淘晶驰命令结束字节。
 * @param frame UART 数据帧缓冲区。
 * @param frame_capacity 缓冲区总容量。
 * @param frame_size 当前帧长度及追加后的帧长度。
 * @return 追加结果状态。
 * @note 统一由文本、曲线清空和曲线加点命令复用。
 */
static hmi_tjc_status_t hmi_tjc_append_terminator(uint8_t *frame,
                                                   uint16_t frame_capacity,
                                                   uint16_t *frame_size)
{
    uint16_t offset;

    if ((frame == 0) || (frame_size == 0))
    {
        return HMI_TJC_STATUS_INVALID_ARGUMENT;
    }
    offset = *frame_size;
    if ((offset > frame_capacity)
        || ((uint16_t)(frame_capacity - offset) < 3u))
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
 * @brief 构建 DC、RMS 和 THD 三个扩展文本控件的指令帧。
 * @param result 待显示的测量结果快照。
 * @param frame 用于接收 UART 数据的缓冲区。
 * @param frame_capacity 缓冲区容量，单位为字节。
 * @param frame_size 用于接收实际帧长度的指针。
 * @return 帧构建结果状态。
 * @note HMI 页面尚未增加对应控件时只构帧、不在主循环自动发送。
 */
hmi_tjc_status_t hmi_tjc_build_detail_frame(
    const measurement_result_t *result,
    uint8_t *frame,
    uint16_t frame_capacity,
    uint16_t *frame_size)
{
    char dc_text[HMI_TJC_TEXT_VALUE_SIZE] = "--";
    char rms_text[HMI_TJC_TEXT_VALUE_SIZE] = "--";
    char thd_text[HMI_TJC_TEXT_VALUE_SIZE] = "--";
    hmi_tjc_status_t status;

    if ((result == 0) || (frame == 0) || (frame_size == 0))
    {
        return HMI_TJC_STATUS_INVALID_ARGUMENT;
    }
    if ((result->valid_mask != 0u)
        && (hmi_tjc_result_is_displayable(result) == 0u))
    {
        return HMI_TJC_STATUS_INVALID_ARGUMENT;
    }

    if (((result->valid_mask & MEASUREMENT_VALID_DC_VOLTAGE) != 0u)
        && (hmi_tjc_format_value(dc_text,
                                 sizeof(dc_text),
                                 HMI_TJC_VALUE_FORMAT_DC,
                                 (double)result->dc_voltage) == 0u))
    {
        return HMI_TJC_STATUS_INVALID_ARGUMENT;
    }
    if (((result->valid_mask & MEASUREMENT_VALID_RMS) != 0u)
        && (hmi_tjc_format_value(rms_text,
                                 sizeof(rms_text),
                                 HMI_TJC_VALUE_FORMAT_RMS,
                                 (double)result->rms_voltage) == 0u))
    {
        return HMI_TJC_STATUS_INVALID_ARGUMENT;
    }
    if (((result->valid_mask & MEASUREMENT_VALID_THD) != 0u)
        && (hmi_tjc_format_value(thd_text,
                                 sizeof(thd_text),
                                 HMI_TJC_VALUE_FORMAT_THD,
                                 (double)result->thd_percent) == 0u))
    {
        return HMI_TJC_STATUS_INVALID_ARGUMENT;
    }

    *frame_size = 0u;
    status = hmi_tjc_append_text_command(frame,
                                         frame_capacity,
                                         frame_size,
                                         "t_dc",
                                         dc_text);
    if (status != HMI_TJC_STATUS_OK)
    {
        return status;
    }
    status = hmi_tjc_append_text_command(frame,
                                         frame_capacity,
                                         frame_size,
                                         "t_rms",
                                         rms_text);
    if (status != HMI_TJC_STATUS_OK)
    {
        return status;
    }
    return hmi_tjc_append_text_command(frame,
                                       frame_capacity,
                                       frame_size,
                                       "t_thd",
                                       thd_text);
}

/**
 * @brief 向帧中追加一条曲线清空或加点命令。
 * @param frame UART 数据帧缓冲区。
 * @param frame_capacity 缓冲区总容量。
 * @param frame_size 当前帧长度及追加后的帧长度。
 * @param component_id 曲线控件数字 ID。
 * @param channel 曲线通道号。
 * @param value 负数表示清空，0 至 255 表示加入一个曲线点。
 * @return 追加结果状态。
 */
static hmi_tjc_status_t hmi_tjc_append_curve_command(
    uint8_t *frame,
    uint16_t frame_capacity,
    uint16_t *frame_size,
    uint8_t component_id,
    uint8_t channel,
    int16_t value)
{
    int command_length;
    uint16_t remaining;

    if ((frame == 0) || (frame_size == 0))
    {
        return HMI_TJC_STATUS_INVALID_ARGUMENT;
    }
    if (*frame_size >= frame_capacity)
    {
        return HMI_TJC_STATUS_BUFFER_TOO_SMALL;
    }
    if (value > 255)
    {
        return HMI_TJC_STATUS_INVALID_ARGUMENT;
    }

    remaining = (uint16_t)(frame_capacity - *frame_size);
    if (value < 0)
    {
        command_length = snprintf((char *)&frame[*frame_size],
                                  remaining,
                                  "cle %u,%u",
                                  (unsigned int)component_id,
                                  (unsigned int)channel);
    }
    else
    {
        command_length = snprintf((char *)&frame[*frame_size],
                                  remaining,
                                  "add %u,%u,%u",
                                  (unsigned int)component_id,
                                  (unsigned int)channel,
                                  (unsigned int)value);
    }
    if ((command_length < 0)
        || ((uint32_t)command_length >= (uint32_t)remaining))
    {
        return HMI_TJC_STATUS_BUFFER_TOO_SMALL;
    }

    *frame_size = (uint16_t)(*frame_size + (uint16_t)command_length);
    return hmi_tjc_append_terminator(frame, frame_capacity, frame_size);
}

/**
 * @brief 构建清空并重绘 64 点频谱曲线的淘晶驰指令帧。
 * @param spectrum 待显示的 0 至 20 kHz 相对 dB 频谱。
 * @param component_id USART HMI 中曲线控件的实际数字 ID。
 * @param channel 曲线通道号，范围为 0 至 3。
 * @param frame 用于接收 UART 数据的缓冲区。
 * @param frame_capacity 缓冲区容量，单位为字节。
 * @param frame_size 用于接收实际帧长度的指针。
 * @return 帧构建结果状态。
 * @note 普通 add 命令不依赖屏幕回传；当前 9600 波特率下应低频触发。
 */
hmi_tjc_status_t hmi_tjc_build_spectrum_frame(
    const measurement_fft_spectrum_t *spectrum,
    uint8_t component_id,
    uint8_t channel,
    uint8_t *frame,
    uint16_t frame_capacity,
    uint16_t *frame_size)
{
    hmi_tjc_status_t status;
    uint32_t index;

    if ((spectrum == 0) || (frame == 0) || (frame_size == 0)
        || (spectrum->valid == 0u)
        || (component_id == HMI_TJC_SPECTRUM_COMPONENT_DISABLED)
        || (channel > 3u))
    {
        return HMI_TJC_STATUS_INVALID_ARGUMENT;
    }

    *frame_size = 0u;
    status = hmi_tjc_append_curve_command(frame,
                                          frame_capacity,
                                          frame_size,
                                          component_id,
                                          channel,
                                          -1);
    if (status != HMI_TJC_STATUS_OK)
    {
        return status;
    }

    for (index = 0u; index < MEASUREMENT_FFT_SPECTRUM_POINT_COUNT; index++)
    {
        int32_t db_x10 = spectrum->relative_db_x10[index];
        int32_t curve_value;

        if (db_x10 < MEASUREMENT_FFT_SPECTRUM_FLOOR_DB_X10)
        {
            db_x10 = MEASUREMENT_FFT_SPECTRUM_FLOOR_DB_X10;
        }
        else if (db_x10 > 0)
        {
            db_x10 = 0;
        }
        curve_value = ((db_x10 - MEASUREMENT_FFT_SPECTRUM_FLOOR_DB_X10)
                       * 255 + 400) / 800;
        status = hmi_tjc_append_curve_command(frame,
                                              frame_capacity,
                                              frame_size,
                                              component_id,
                                              channel,
                                              (int16_t)curve_value);
        if (status != HMI_TJC_STATUS_OK)
        {
            return status;
        }
    }

    return HMI_TJC_STATUS_OK;
}

/** 串口屏帧构建与 UART 轮询发送累计诊断。 */
static hmi_tjc_diagnostics_t hmi_tjc_diagnostics;

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
        hmi_tjc_diagnostics.last_status = HMI_TJC_STATUS_UART_UNAVAILABLE;
        return HMI_TJC_STATUS_UART_UNAVAILABLE;
    }
    if ((frame_size == 0u) || (frame_size > HMI_TJC_TX_BUFFER_SIZE))
    {
        hmi_tjc_diagnostics.last_status = HMI_TJC_STATUS_INVALID_ARGUMENT;
        return HMI_TJC_STATUS_INVALID_ARGUMENT;
    }

    hmi_tjc_diagnostics.transmit_attempts++;
    hmi_tjc_diagnostics.last_frame_size = frame_size;
    if (HAL_UART_Transmit(hmi_tjc_uart,
                          hmi_tjc_tx_buffer,
                          frame_size,
                          HMI_TJC_TX_TIMEOUT_MS) != HAL_OK)
    {
        hmi_tjc_diagnostics.transmit_failures++;
        hmi_tjc_diagnostics.last_status = HMI_TJC_STATUS_HAL_ERROR;
        return HMI_TJC_STATUS_HAL_ERROR;
    }

    hmi_tjc_diagnostics.transmit_successes++;
    hmi_tjc_diagnostics.last_status = HMI_TJC_STATUS_OK;
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
    hmi_tjc_diagnostics.transmit_attempts = 0u;
    hmi_tjc_diagnostics.transmit_successes = 0u;
    hmi_tjc_diagnostics.transmit_failures = 0u;
    hmi_tjc_diagnostics.build_failures = 0u;
    hmi_tjc_diagnostics.last_frame_size = 0u;
    hmi_tjc_diagnostics.last_status = HMI_TJC_STATUS_UART_UNAVAILABLE;
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
 * @note UART 尚未配置时无操作；UART 就绪后每 500 ms 最多轮询发送一帧。
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
        hmi_tjc_diagnostics.build_failures++;
        hmi_tjc_diagnostics.last_status = status;
        return;
    }

    (void)hmi_tjc_send_frame(frame_size);
#endif
}

/**
 * @brief 读取串口屏模块累计诊断数据。
 * @param diagnostics 用于接收诊断快照的指针。
 * @return 指针有效时返回 1，否则返回 0。
 * @note 只复制主循环维护的状态，不访问 UART 硬件。
 */
uint8_t hmi_tjc_get_diagnostics(hmi_tjc_diagnostics_t *diagnostics)
{
    if (diagnostics == 0)
    {
        return 0u;
    }

    *diagnostics = hmi_tjc_diagnostics;
    return 1u;
}
