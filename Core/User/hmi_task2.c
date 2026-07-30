/**
 * @file hmi_task2.c
 * @brief G 题淘晶驰串口屏后台预装与按键切换实现。
 *
 * 模块用途：把 measurement_conversion 生成的三组 350 点缓存依次写入 s_t1、s_t3、
 *          s_spec，随后更新参数文本；屏幕按键互斥切换时域控件，独立频谱保持可见。
 * GPIO 引脚映射：PA9/USART1_TX 接屏幕 RX，PA10/USART1_RX 接屏幕 TX。
 * 依赖的外设和 CubeIDE 配置：USART1 512000 baud、8N1、TX/RX DMA、USART1 全局中断。
 * 初始化方法：system_init() 调用 hmi_task2_init() 并绑定 huart1。
 * 调用方法：system_process() 每轮调用 hmi_task2_process()。
 */

#include "system.h"

#include <stdio.h>
#include <string.h>

#define HMI_TASK2_RX_BYTES            32u
#define HMI_TASK2_TX_STORAGE_BYTES    9248u
#define HMI_TASK2_TX_TIMEOUT_MS       1000u
#define HMI_TASK2_COMMAND_HEAD        0xa5u
#define HMI_TASK2_COMMAND_TAIL        0x5au

/*
 * 后台预装状态机的核心规则：
 *
 * 1. 上电先隐藏三个曲线控件；
 * 2. 取得一份稳定的 measurement_display_snapshot_t 工作快照；
 * 3. 依次发送当前按键目标曲线、另外两条曲线和参数文本；
 * 4. 每项只有在 TX DMA 完成回调到达后才记为“已装载”；
 * 5. 用户按键只触发 vis 命令，不重新采样、不重新换算整条曲线；
 * 6. 一份工作快照未装完前不换新快照，避免 FPGA 持续产帧导致旧帧永远装不完。
 *
 * 中断回调只写事件标志，所有解析、构帧和状态迁移均在主循环执行。
 */

/** 内部发送动作；动作完成后才更新对应的已装载序号。 */
typedef enum
{
    HMI_TX_ACTION_NONE = 0,
    HMI_TX_ACTION_INITIALIZE,
    HMI_TX_ACTION_ONE_CYCLE,
    HMI_TX_ACTION_THREE_CYCLE,
    HMI_TX_ACTION_SPECTRUM,
    HMI_TX_ACTION_TEXT,
    HMI_TX_ACTION_VISIBILITY
} hmi_tx_action_t;

volatile uint16_t hmi_uart_rx_event_size;
volatile uint8_t hmi_uart_tx_complete_flag;
volatile uint8_t hmi_uart_error_flag;
volatile hmi_task2_diagnostics_t hmi_task2_diagnostics;

/** USART1 句柄，仅由初始化写入。 */
static UART_HandleTypeDef *hmi_task2_uart;

/** RX DMA 缓冲区，32 字节对齐以兼容 H743 D-Cache 维护。 */
static uint8_t hmi_task2_rx_buffer[HMI_TASK2_RX_BYTES]
    __attribute__((aligned(32)));

/** TX DMA 缓冲区，实际可构建区为 9216 字节，尾部用于缓存行取整。 */
static uint8_t hmi_task2_tx_buffer[HMI_TASK2_TX_STORAGE_BYTES]
    __attribute__((aligned(32)));

/** 当前正在预装的一份稳定显示快照。 */
static measurement_display_snapshot_t hmi_task2_work_snapshot;

/** 每个控件已经装载的快照序号，索引 1~3 对应显示模式。 */
static uint32_t hmi_task2_loaded_sequence[4];

/** 参数文本已经装载的快照序号。 */
static uint32_t hmi_task2_text_sequence;

/** 每个控件是否至少成功装载过一次，避免首帧序号为零时误判。 */
static uint8_t hmi_task2_loaded_valid[4];

/** 参数文本是否至少成功装载过一次。 */
static uint8_t hmi_task2_text_valid;

/** 当前 DMA 发送动作及其数据源序号。 */
static hmi_tx_action_t hmi_task2_tx_action;
static uint32_t hmi_task2_tx_source_sequence;
static uint32_t hmi_task2_tx_started_ms;

/** 接收 A5 CMD 5A 的三状态解析器状态。 */
static uint8_t hmi_task2_command_state;
static uint8_t hmi_task2_command_candidate;

/** 软件状态标志。 */
static uint8_t hmi_task2_rx_active;
static uint8_t hmi_task2_tx_active;
static uint8_t hmi_task2_initialize_done;
static uint8_t hmi_task2_work_valid;
static uint8_t hmi_task2_visibility_pending;
/** 收到至少一次有效按键命令后置位；上电预装阶段保持三图隐藏。 */
static uint8_t hmi_task2_display_requested;
static uint8_t hmi_task2_self_test_enabled;

/**
 * @brief 对 DMA 接收缓冲区执行接收前缓存维护。
 * @param buffer 缓冲区首地址。
 * @param length 字节数。
 * @return 无。
 */
static void hmi_task2_cache_prepare_rx(void *buffer, uint32_t length)
{
#if (__DCACHE_PRESENT == 1U)
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0u)
    {
        uint32_t address = (uint32_t)(uintptr_t)buffer;
        uint32_t start = address & ~31u;
        uint32_t end = (address + length + 31u) & ~31u;
        SCB_CleanInvalidateDCache_by_Addr((uint32_t *)start,
                                         (int32_t)(end - start));
    }
#else
    (void)buffer;
    (void)length;
#endif
}

/**
 * @brief 使 CPU 丢弃 RX DMA 已覆盖区域的旧缓存行。
 * @param buffer 缓冲区首地址。
 * @param length DMA 实际写入字节数。
 * @return 无。
 */
static void hmi_task2_cache_finish_rx(void *buffer, uint32_t length)
{
#if (__DCACHE_PRESENT == 1U)
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0u)
    {
        uint32_t address = (uint32_t)(uintptr_t)buffer;
        uint32_t start = address & ~31u;
        uint32_t end = (address + length + 31u) & ~31u;
        SCB_InvalidateDCache_by_Addr((uint32_t *)start,
                                    (int32_t)(end - start));
    }
#else
    (void)buffer;
    (void)length;
#endif
}

/**
 * @brief 在 TX DMA 读取前把 CPU 修改内容写回内存。
 * @param buffer 缓冲区首地址。
 * @param length 待发送字节数。
 * @return 无。
 */
static void hmi_task2_cache_prepare_tx(void *buffer, uint32_t length)
{
#if (__DCACHE_PRESENT == 1U)
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0u)
    {
        uint32_t address = (uint32_t)(uintptr_t)buffer;
        uint32_t start = address & ~31u;
        uint32_t end = (address + length + 31u) & ~31u;
        SCB_CleanDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
    }
#else
    (void)buffer;
    (void)length;
#endif
}

/**
 * @brief 启动下一次屏幕 RX Receive-to-IDLE DMA。
 * @param 无。
 * @return 成功返回 1，失败返回 0。
 */
static uint8_t hmi_task2_start_rx(void)
{
    HAL_StatusTypeDef status;

    if (hmi_task2_uart == NULL)
    {
        return 0u;
    }

    hmi_task2_cache_prepare_rx(hmi_task2_rx_buffer,
                               sizeof(hmi_task2_rx_buffer));
    status = HAL_UARTEx_ReceiveToIdle_DMA(hmi_task2_uart,
                                         hmi_task2_rx_buffer,
                                         sizeof(hmi_task2_rx_buffer));
    if (status != HAL_OK)
    {
        hmi_task2_diagnostics.tx_error_count++;
        hmi_task2_diagnostics.state = HMI_TASK2_STATE_ERROR;
        return 0u;
    }

    if (hmi_task2_uart->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(hmi_task2_uart->hdmarx, DMA_IT_HT);
    }
    hmi_task2_rx_active = 1u;
    return 1u;
}

/**
 * @brief 向构帧缓冲区追加一条带 FF FF FF 结束符的淘晶驰指令。
 * @param buffer 输出缓冲区。
 * @param capacity 缓冲区总容量。
 * @param offset 当前写入偏移，成功后自动推进。
 * @param command 不含结束符的 ASCII 指令。
 * @return 成功返回 1，空间不足返回 0。
 */
static uint8_t hmi_task2_append_command(uint8_t *buffer,
                                        uint16_t capacity,
                                        uint16_t *offset,
                                        const char *command)
{
    size_t length;

    if ((buffer == NULL) || (offset == NULL) || (command == NULL))
    {
        return 0u;
    }
    length = strlen(command);
    if (((uint32_t)(*offset) + length + 3u) > capacity)
    {
        return 0u;
    }

    memcpy(&buffer[*offset], command, length);
    *offset = (uint16_t)(*offset + length);
    buffer[(*offset)++] = 0xffu;
    buffer[(*offset)++] = 0xffu;
    buffer[(*offset)++] = 0xffu;
    return 1u;
}

/**
 * @brief 把微伏整数格式化为便于比赛现场读取的电压文本。
 * @param value_uv 电压值，单位微伏。
 * @param text 输出文本。
 * @param text_size 输出容量。
 * @return 无。
 */
static void hmi_task2_format_voltage(uint32_t value_uv,
                                     char *text,
                                     size_t text_size)
{
    if (value_uv >= 1000000u)
    {
        (void)snprintf(text, text_size, "%lu.%03lu V",
                       (unsigned long)(value_uv / 1000000u),
                       (unsigned long)((value_uv % 1000000u) / 1000u));
    }
    else if (value_uv >= 1000u)
    {
        (void)snprintf(text, text_size, "%lu.%03lu mV",
                       (unsigned long)(value_uv / 1000u),
                       (unsigned long)(value_uv % 1000u));
    }
    else
    {
        (void)snprintf(text, text_size, "%lu uV",
                       (unsigned long)value_uv);
    }
}

/**
 * @brief 把毫赫兹整数格式化为 Hz/kHz 文本。
 * @param value_mhz 频率，单位 0.001 Hz。
 * @param text 输出文本。
 * @param text_size 输出容量。
 * @return 无。
 */
static void hmi_task2_format_frequency(uint32_t value_mhz,
                                       char *text,
                                       size_t text_size)
{
    if (value_mhz >= 1000000u)
    {
        (void)snprintf(text, text_size, "%lu.%03lu kHz",
                       (unsigned long)(value_mhz / 1000000u),
                       (unsigned long)((value_mhz % 1000000u) / 1000u));
    }
    else
    {
        (void)snprintf(text, text_size, "%lu.%03lu Hz",
                       (unsigned long)(value_mhz / 1000u),
                       (unsigned long)(value_mhz % 1000u));
    }
}

/**
 * @brief 生成上电隐藏三条曲线并显示等待状态的命令。
 * @param frame_size 输出实际字节数。
 * @return 成功返回 1，失败返回 0。
 */
static uint8_t hmi_task2_build_initialize(uint16_t *frame_size)
{
    uint16_t size = 0u;

    if (hmi_chart_build_hide_all(hmi_task2_tx_buffer,
                                 HMI_CHART_FRAME_MAX_BYTES,
                                 &size) != HMI_CHART_STATUS_OK)
    {
        return 0u;
    }
    if (hmi_task2_append_command(hmi_task2_tx_buffer,
                                 HMI_CHART_FRAME_MAX_BYTES,
                                 &size,
                                 "t_status.txt=\"WAIT FPGA\"") == 0u)
    {
        return 0u;
    }
    *frame_size = size;
    return 1u;
}

/**
 * @brief 生成当前快照的全部参数文本。
 * @param frame_size 输出实际字节数。
 * @return 成功返回 1，失败返回 0。
 */
static uint8_t hmi_task2_build_text(uint16_t *frame_size)
{
    char command[96];
    char voltage[28];
    char frequency[28];
    char amplitude[28];
    uint16_t size = 0u;
    uint8_t index;

    hmi_task2_format_voltage(hmi_task2_work_snapshot.vpp_uv,
                             voltage, sizeof(voltage));
    (void)snprintf(command, sizeof(command), "t_vpp.txt=\"%s\"", voltage);
    if (hmi_task2_append_command(hmi_task2_tx_buffer,
                                 HMI_CHART_FRAME_MAX_BYTES,
                                 &size, command) == 0u)
    {
        return 0u;
    }

    hmi_task2_format_voltage(hmi_task2_work_snapshot.vrms_uv,
                             voltage, sizeof(voltage));
    (void)snprintf(command, sizeof(command), "t_vrms.txt=\"%s\"", voltage);
    if (hmi_task2_append_command(hmi_task2_tx_buffer,
                                 HMI_CHART_FRAME_MAX_BYTES,
                                 &size, command) == 0u)
    {
        return 0u;
    }

    hmi_task2_format_frequency(hmi_task2_work_snapshot.fundamental_mhz,
                               frequency, sizeof(frequency));
    (void)snprintf(command, sizeof(command), "t_freq.txt=\"%s\"", frequency);
    if (hmi_task2_append_command(hmi_task2_tx_buffer,
                                 HMI_CHART_FRAME_MAX_BYTES,
                                 &size, command) == 0u)
    {
        return 0u;
    }

    for (index = 0u; index < FPGA_PROTOCOL_COMPONENT_MAX; index++)
    {
        const fpga_protocol_component_t *component =
            &hmi_task2_work_snapshot.component[index];

        if ((index < hmi_task2_work_snapshot.component_count)
            && ((component->flags
                 & FPGA_PROTOCOL_COMPONENT_VALID) != 0u))
        {
            hmi_task2_format_frequency(component->frequency_mhz,
                                       frequency, sizeof(frequency));
            hmi_task2_format_voltage(component->amplitude_peak_uv,
                                     amplitude, sizeof(amplitude));
            (void)snprintf(command, sizeof(command),
                           "t_comp%u.txt=\"H%u %s %s\"",
                           (unsigned int)(index + 1u),
                           (unsigned int)component->harmonic_order,
                           frequency, amplitude);
        }
        else
        {
            (void)snprintf(command, sizeof(command),
                           "t_comp%u.txt=\"--\"",
                           (unsigned int)(index + 1u));
        }

        if (hmi_task2_append_command(hmi_task2_tx_buffer,
                                     HMI_CHART_FRAME_MAX_BYTES,
                                     &size, command) == 0u)
        {
            return 0u;
        }
    }

    if (hmi_task2_work_snapshot.dropped_frames == 0u)
    {
        (void)snprintf(command, sizeof(command), "t_status.txt=\"READY #%lu\"",
                       (unsigned long)hmi_task2_work_snapshot.frame_sequence);
    }
    else
    {
        (void)snprintf(command, sizeof(command),
                       "t_status.txt=\"READY #%lu DROP %lu\"",
                       (unsigned long)hmi_task2_work_snapshot.frame_sequence,
                       (unsigned long)hmi_task2_work_snapshot.dropped_frames);
    }
    if (hmi_task2_append_command(hmi_task2_tx_buffer,
                                 HMI_CHART_FRAME_MAX_BYTES,
                                 &size, command) == 0u)
    {
        return 0u;
    }

    *frame_size = size;
    return 1u;
}

/**
 * @brief 生成横轴恰好包含指定周期数的三角波自检点。
 * @param index 当前显示点下标，范围为 0 至显示点数减一。
 * @param cycle_count 整个横轴需要显示的完整周期数。
 * @return 映射到串口屏控件安全纵轴范围内的值。
 *
 * @note 首尾点均位于波谷，因此横轴从第一个点到最后一个点正好覆盖整数周期。
 */
static uint8_t hmi_task2_generate_triangle_point(
    uint16_t index,
    uint8_t cycle_count)
{
    const uint32_t value_min = MEASUREMENT_DISPLAY_Y_MIN;
    const uint32_t value_span =
        MEASUREMENT_DISPLAY_Y_MAX - MEASUREMENT_DISPLAY_Y_MIN;
    const uint32_t full_phase = value_span * 2u;
    uint32_t phase;

    phase = ((uint32_t)index * (uint32_t)cycle_count * full_phase)
        / (MEASUREMENT_DISPLAY_POINT_COUNT - 1u);
    phase %= full_phase;
    if (phase > value_span)
    {
        phase = full_phase - phase;
    }

    return (uint8_t)(value_min + phase);
}

/**
 * @brief 生成一份与正式路径尺寸完全一致的三图自检快照。
 * @param snapshot 输出显示快照。
 * @return 无。
 */
static void hmi_task2_generate_self_test(
    measurement_display_snapshot_t *snapshot)
{
    const uint16_t spectrum_center =
        (uint16_t)((MEASUREMENT_DISPLAY_POINT_COUNT - 1u) / 2u);
    uint16_t index;

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->frame_sequence = 0xffffffffu;
    snapshot->vpp_uv = 3300000u;
    snapshot->vrms_uv = 1166700u;
    snapshot->fundamental_mhz = 12345000u;
    snapshot->component_count = 1u;
    snapshot->component[0].frequency_mhz = snapshot->fundamental_mhz;
    snapshot->component[0].amplitude_peak_uv = 1650000u;
    snapshot->component[0].harmonic_order = 1u;
    snapshot->component[0].flags = 0x03u;
    snapshot->valid = 1u;

    for (index = 0u; index < MEASUREMENT_DISPLAY_POINT_COUNT; index++)
    {
        uint16_t distance = (index > spectrum_center)
            ? (uint16_t)(index - spectrum_center)
            : (uint16_t)(spectrum_center - index);

        snapshot->waveform_1cycle[index] =
            hmi_task2_generate_triangle_point(index, 1u);
        snapshot->waveform_3cycle[index] =
            hmi_task2_generate_triangle_point(index, 3u);
        if (distance < 18u)
        {
            snapshot->spectrum_display[index] = (uint8_t)(
                MEASUREMENT_DISPLAY_Y_MAX
                - (((uint32_t)distance
                    * (MEASUREMENT_DISPLAY_Y_MAX
                       - MEASUREMENT_DISPLAY_Y_MIN))
                   / 17u));
        }
        else
        {
            snapshot->spectrum_display[index] =
                MEASUREMENT_DISPLAY_Y_MIN;
        }
    }
}

/**
 * @brief 判断当前工作快照是否已全部预装。
 * @param 无。
 * @return 三条曲线和文本均完成返回 1，否则返回 0。
 */
static uint8_t hmi_task2_preload_complete(void)
{
    uint32_t sequence = hmi_task2_work_snapshot.frame_sequence;

    return (uint8_t)((hmi_task2_work_valid != 0u)
        && (hmi_task2_loaded_valid[HMI_CHART_MODE_ONE_CYCLE] != 0u)
        && (hmi_task2_loaded_valid[HMI_CHART_MODE_THREE_CYCLE] != 0u)
        && (hmi_task2_loaded_valid[HMI_CHART_MODE_SPECTRUM] != 0u)
        && (hmi_task2_text_valid != 0u)
        && (hmi_task2_loaded_sequence[HMI_CHART_MODE_ONE_CYCLE] == sequence)
        && (hmi_task2_loaded_sequence[HMI_CHART_MODE_THREE_CYCLE] == sequence)
        && (hmi_task2_loaded_sequence[HMI_CHART_MODE_SPECTRUM] == sequence)
        && (hmi_task2_text_sequence == sequence));
}

/**
 * @brief 在上一份预装结束后获取最新显示快照，避免连续新帧造成预装饥饿。
 * @param 无。
 * @return 无。
 */
static void hmi_task2_refresh_work_snapshot(void)
{
    const measurement_display_snapshot_t *latest;

    if (hmi_task2_self_test_enabled != 0u)
    {
        if (hmi_task2_work_valid == 0u)
        {
            hmi_task2_generate_self_test(&hmi_task2_work_snapshot);
            hmi_task2_work_valid = 1u;
        }
        return;
    }

    if ((hmi_task2_work_valid != 0u)
        && (hmi_task2_preload_complete() == 0u))
    {
        return;
    }
    if (measurement_conversion_get_snapshot(&latest) == 0u)
    {
        return;
    }
    if ((hmi_task2_work_valid != 0u)
        && (latest->frame_sequence == hmi_task2_work_snapshot.frame_sequence))
    {
        return;
    }

    memcpy(&hmi_task2_work_snapshot, latest, sizeof(hmi_task2_work_snapshot));
    hmi_task2_work_valid = 1u;
    hmi_task2_diagnostics.last_source_sequence =
        hmi_task2_work_snapshot.frame_sequence;
    hmi_task2_diagnostics.state = HMI_TASK2_STATE_PRELOADING;
}

/**
 * @brief 解析屏幕返回的 A5 CMD 5A 字节流。
 * @param data 本次 DMA 接收到的字节。
 * @param length 字节数。
 * @return 无。
 */
static void hmi_task2_parse_commands(const uint8_t *data, uint16_t length)
{
    uint16_t index;

    for (index = 0u; index < length; index++)
    {
        uint8_t byte = data[index];
        hmi_task2_diagnostics.rx_byte_count++;

        if (hmi_task2_command_state == 0u)
        {
            if (byte == HMI_TASK2_COMMAND_HEAD)
            {
                hmi_task2_command_state = 1u;
            }
        }
        else if (hmi_task2_command_state == 1u)
        {
            if ((byte >= (uint8_t)HMI_CHART_MODE_ONE_CYCLE)
                && (byte <= (uint8_t)HMI_CHART_MODE_SPECTRUM))
            {
                hmi_task2_command_candidate = byte;
                hmi_task2_command_state = 2u;
            }
            else
            {
                hmi_task2_diagnostics.invalid_command_count++;
                hmi_task2_command_state =
                    (byte == HMI_TASK2_COMMAND_HEAD) ? 1u : 0u;
            }
        }
        else
        {
            if (byte == HMI_TASK2_COMMAND_TAIL)
            {
                hmi_task2_diagnostics.last_command =
                    hmi_task2_command_candidate;
                hmi_task2_diagnostics.requested_mode =
                    hmi_task2_command_candidate;
                hmi_task2_diagnostics.command_count++;
                hmi_task2_display_requested = 1u;
                hmi_task2_visibility_pending = 1u;
            }
            else
            {
                hmi_task2_diagnostics.invalid_command_count++;
            }
            hmi_task2_command_state =
                (byte == HMI_TASK2_COMMAND_HEAD) ? 1u : 0u;
        }
    }
}

/**
 * @brief 根据动作构建下一帧 UART 数据。
 * @param action 目标动作。
 * @param frame_size 输出实际字节数。
 * @return 成功返回 1，失败返回 0。
 */
static uint8_t hmi_task2_build_action(hmi_tx_action_t action,
                                      uint16_t *frame_size)
{
    const uint8_t *points = NULL;
    const char *object_name = NULL;

    switch (action)
    {
        case HMI_TX_ACTION_INITIALIZE:
            return hmi_task2_build_initialize(frame_size);

        case HMI_TX_ACTION_ONE_CYCLE:
            points = hmi_task2_work_snapshot.waveform_1cycle;
            object_name = HMI_CHART_T1_OBJECT;
            break;

        case HMI_TX_ACTION_THREE_CYCLE:
            points = hmi_task2_work_snapshot.waveform_3cycle;
            object_name = HMI_CHART_T3_OBJECT;
            break;

        case HMI_TX_ACTION_SPECTRUM:
            points = hmi_task2_work_snapshot.spectrum_display;
            object_name = HMI_CHART_SPECTRUM_OBJECT;
            break;

        case HMI_TX_ACTION_TEXT:
            return hmi_task2_build_text(frame_size);

        case HMI_TX_ACTION_VISIBILITY:
            return (uint8_t)(hmi_chart_build_visibility(
                (hmi_chart_mode_t)hmi_task2_diagnostics.requested_mode,
                hmi_task2_tx_buffer,
                HMI_CHART_FRAME_MAX_BYTES,
                frame_size) == HMI_CHART_STATUS_OK);

        default:
            return 0u;
    }

    return (uint8_t)(hmi_chart_build_waveform(
        object_name, points, MEASUREMENT_DISPLAY_POINT_COUNT,
        hmi_task2_tx_buffer, HMI_CHART_FRAME_MAX_BYTES,
        frame_size) == HMI_CHART_STATUS_OK);
}

/**
 * @brief 选择当前最重要的动作；按键目标优先于后台其余控件。
 * @param 无。
 * @return 待执行动作，无任务返回 HMI_TX_ACTION_NONE。
 */
static hmi_tx_action_t hmi_task2_select_action(void)
{
    uint32_t sequence;
    uint8_t requested_mode = hmi_task2_diagnostics.requested_mode;

    if (hmi_task2_initialize_done == 0u)
    {
        return HMI_TX_ACTION_INITIALIZE;
    }
    if (hmi_task2_work_valid == 0u)
    {
        return HMI_TX_ACTION_NONE;
    }

    sequence = hmi_task2_work_snapshot.frame_sequence;
    if ((hmi_task2_display_requested != 0u)
        && (hmi_task2_visibility_pending != 0u)
        && (hmi_task2_loaded_valid[requested_mode] != 0u)
        && (hmi_task2_loaded_sequence[requested_mode] == sequence))
    {
        return HMI_TX_ACTION_VISIBILITY;
    }
    if ((hmi_task2_loaded_valid[requested_mode] == 0u)
        || (hmi_task2_loaded_sequence[requested_mode] != sequence))
    {
        return (hmi_tx_action_t)(HMI_TX_ACTION_ONE_CYCLE
                                + requested_mode - 1u);
    }
    if ((hmi_task2_loaded_valid[HMI_CHART_MODE_ONE_CYCLE] == 0u)
        || (hmi_task2_loaded_sequence[HMI_CHART_MODE_ONE_CYCLE] != sequence))
    {
        return HMI_TX_ACTION_ONE_CYCLE;
    }
    if ((hmi_task2_loaded_valid[HMI_CHART_MODE_THREE_CYCLE] == 0u)
        || (hmi_task2_loaded_sequence[HMI_CHART_MODE_THREE_CYCLE] != sequence))
    {
        return HMI_TX_ACTION_THREE_CYCLE;
    }
    if ((hmi_task2_loaded_valid[HMI_CHART_MODE_SPECTRUM] == 0u)
        || (hmi_task2_loaded_sequence[HMI_CHART_MODE_SPECTRUM] != sequence))
    {
        return HMI_TX_ACTION_SPECTRUM;
    }
    if ((hmi_task2_text_valid == 0u)
        || (hmi_task2_text_sequence != sequence))
    {
        return HMI_TX_ACTION_TEXT;
    }
    if ((hmi_task2_display_requested != 0u)
        && ((hmi_task2_diagnostics.visible_mode != requested_mode)
            || (hmi_task2_diagnostics.last_visible_sequence != sequence)))
    {
        return HMI_TX_ACTION_VISIBILITY;
    }
    return HMI_TX_ACTION_NONE;
}

/**
 * @brief 成功完成一次 TX DMA 后提交对应软件状态。
 * @param 无。
 * @return 无。
 */
static void hmi_task2_complete_action(void)
{
    uint8_t mode;

    switch (hmi_task2_tx_action)
    {
        case HMI_TX_ACTION_INITIALIZE:
            hmi_task2_initialize_done = 1u;
            break;

        case HMI_TX_ACTION_ONE_CYCLE:
        case HMI_TX_ACTION_THREE_CYCLE:
        case HMI_TX_ACTION_SPECTRUM:
            mode = (uint8_t)(hmi_task2_tx_action
                             - HMI_TX_ACTION_ONE_CYCLE + 1u);
            hmi_task2_loaded_sequence[mode] =
                hmi_task2_tx_source_sequence;
            hmi_task2_loaded_valid[mode] = 1u;
            break;

        case HMI_TX_ACTION_TEXT:
            hmi_task2_text_sequence = hmi_task2_tx_source_sequence;
            hmi_task2_text_valid = 1u;
            break;

        case HMI_TX_ACTION_VISIBILITY:
            hmi_task2_diagnostics.visible_mode =
                hmi_task2_diagnostics.requested_mode;
            hmi_task2_diagnostics.last_visible_sequence =
                hmi_task2_tx_source_sequence;
            hmi_task2_visibility_pending = 0u;
            break;

        default:
            break;
    }

    if (hmi_task2_preload_complete() != 0u)
    {
        if (hmi_task2_diagnostics.state != HMI_TASK2_STATE_READY)
        {
            hmi_task2_diagnostics.preload_complete_count++;
        }
        hmi_task2_diagnostics.state = HMI_TASK2_STATE_READY;
    }
    else if (hmi_task2_work_valid != 0u)
    {
        hmi_task2_diagnostics.state = HMI_TASK2_STATE_PRELOADING;
    }

    hmi_task2_tx_action = HMI_TX_ACTION_NONE;
}

/**
 * @brief 构建并启动一项 USART1 TX DMA。
 * @param action 发送动作。
 * @return 成功启动返回 1，否则返回 0。
 */
static uint8_t hmi_task2_start_tx(hmi_tx_action_t action)
{
    uint16_t frame_size = 0u;

    if (hmi_task2_build_action(action, &frame_size) == 0u)
    {
        hmi_task2_diagnostics.tx_error_count++;
        hmi_task2_diagnostics.state = HMI_TASK2_STATE_ERROR;
        return 0u;
    }

    hmi_task2_cache_prepare_tx(hmi_task2_tx_buffer, frame_size);
    if (HAL_UART_Transmit_DMA(hmi_task2_uart,
                              hmi_task2_tx_buffer,
                              frame_size) != HAL_OK)
    {
        hmi_task2_diagnostics.tx_error_count++;
        hmi_task2_diagnostics.state = HMI_TASK2_STATE_ERROR;
        return 0u;
    }

    hmi_task2_tx_action = action;
    hmi_task2_tx_source_sequence =
        (hmi_task2_work_valid != 0u)
        ? hmi_task2_work_snapshot.frame_sequence : 0u;
    hmi_task2_tx_started_ms = HAL_GetTick();
    hmi_task2_tx_active = 1u;
    hmi_task2_diagnostics.last_tx_bytes = frame_size;
    hmi_task2_diagnostics.tx_start_count++;
    return 1u;
}

void hmi_task2_init(void)
{
    memset((void *)&hmi_task2_diagnostics, 0,
           sizeof(hmi_task2_diagnostics));
    memset(hmi_task2_rx_buffer, 0, sizeof(hmi_task2_rx_buffer));
    memset(hmi_task2_tx_buffer, 0, sizeof(hmi_task2_tx_buffer));
    memset(&hmi_task2_work_snapshot, 0,
           sizeof(hmi_task2_work_snapshot));
    memset(hmi_task2_loaded_sequence, 0,
           sizeof(hmi_task2_loaded_sequence));
    memset(hmi_task2_loaded_valid, 0,
           sizeof(hmi_task2_loaded_valid));

    hmi_task2_uart = NULL;
    hmi_uart_rx_event_size = 0u;
    hmi_uart_tx_complete_flag = 0u;
    hmi_uart_error_flag = 0u;
    hmi_task2_text_sequence = 0u;
    hmi_task2_text_valid = 0u;
    hmi_task2_tx_action = HMI_TX_ACTION_NONE;
    hmi_task2_tx_source_sequence = 0u;
    hmi_task2_tx_started_ms = 0u;
    hmi_task2_command_state = 0u;
    hmi_task2_command_candidate = 0u;
    hmi_task2_rx_active = 0u;
    hmi_task2_tx_active = 0u;
    hmi_task2_initialize_done = 0u;
    hmi_task2_work_valid = 0u;
    hmi_task2_visibility_pending = 0u;
    hmi_task2_display_requested = 0u;
    hmi_task2_self_test_enabled = 0u;
    hmi_task2_diagnostics.requested_mode =
        (uint8_t)HMI_CHART_MODE_ONE_CYCLE;
    hmi_task2_diagnostics.state = HMI_TASK2_STATE_WAIT_DATA;
}

/**
 * @brief 打开或关闭脱离 FPGA 的串口屏三图自检模式。
 * @param enable 非零时使用内部测试曲线，并主动显示；零时恢复正式数据链路。
 * @return 无。
 *
 * @note 正式比赛代码应保持关闭。该接口仅用于排除 FPGA/SPI 之前的屏幕链路问题。
 */
void hmi_task2_set_chart_self_test(uint8_t enable)
{
    hmi_task2_self_test_enabled = (enable != 0u) ? 1u : 0u;
    hmi_task2_work_valid = 0u;
    memset(hmi_task2_loaded_sequence, 0,
           sizeof(hmi_task2_loaded_sequence));
    memset(hmi_task2_loaded_valid, 0,
           sizeof(hmi_task2_loaded_valid));
    hmi_task2_text_sequence = 0u;
    hmi_task2_text_valid = 0u;
    hmi_task2_display_requested =
        (hmi_task2_self_test_enabled != 0u) ? 1u : 0u;
    hmi_task2_visibility_pending = hmi_task2_display_requested;
}

#if defined(HAL_UART_MODULE_ENABLED)
/**
 * @brief 绑定串口屏 UART 并启动 Receive-to-IDLE DMA。
 * @param huart CubeMX 生成的 USART1 HAL 句柄。
 * @return 无。
 *
 * @note 绑定后 RX DMA 会循环重启，用于接收屏幕按键发出的 A5 CMD 5A。
 */
void hmi_task2_bind_uart(UART_HandleTypeDef *huart)
{
    hmi_task2_uart = huart;
    hmi_task2_rx_active = 0u;
    if (hmi_task2_start_rx() == 0u)
    {
        hmi_task2_diagnostics.state = HMI_TASK2_STATE_ERROR;
    }
}
#endif

/**
 * @brief 在主循环中推进串口屏接收、发送、预装和显示切换状态机。
 * @param 无。
 * @return 无。
 *
 * @note 本函数不阻塞等待 UART。DMA 完成、错误和空闲接收事件先由回调置标志，
 *       下一轮主循环再统一处理，避免在中断中解析协议或构建大帧。
 */
void hmi_task2_process(void)
{
    uint16_t rx_size;
    uint32_t primask;
    uint8_t tx_complete;
    uint8_t uart_error;
    hmi_tx_action_t action;

    if (hmi_task2_uart == NULL)
    {
        return;
    }

    /*
     * 用极短临界区一次性“取走”中断事件。这样主循环处理期间，
     * 新到的事件仍可由回调写入，留给下一轮处理，不会长时间关闭中断。
     */
    primask = __get_PRIMASK();
    __disable_irq();
    rx_size = hmi_uart_rx_event_size;
    hmi_uart_rx_event_size = 0u;
    tx_complete = hmi_uart_tx_complete_flag;
    hmi_uart_tx_complete_flag = 0u;
    uart_error = hmi_uart_error_flag;
    hmi_uart_error_flag = 0u;
    if (primask == 0u)
    {
        __enable_irq();
    }

    if (uart_error != 0u)
    {
        hmi_task2_diagnostics.tx_error_count++;
        hmi_task2_diagnostics.state = HMI_TASK2_STATE_ERROR;
        (void)HAL_UART_Abort(hmi_task2_uart);
        hmi_task2_rx_active = 0u;
        hmi_task2_tx_active = 0u;
        hmi_task2_tx_action = HMI_TX_ACTION_NONE;
    }

    if (tx_complete != 0u)
    {
        if (hmi_task2_tx_active != 0u)
        {
            hmi_task2_tx_active = 0u;
            hmi_task2_diagnostics.tx_complete_count++;
            hmi_task2_complete_action();
        }
    }
    else if ((hmi_task2_tx_active != 0u)
             && ((uint32_t)(HAL_GetTick() - hmi_task2_tx_started_ms)
                 > HMI_TASK2_TX_TIMEOUT_MS))
    {
        (void)HAL_UART_AbortTransmit(hmi_task2_uart);
        hmi_task2_tx_active = 0u;
        hmi_task2_tx_action = HMI_TX_ACTION_NONE;
        hmi_task2_diagnostics.tx_error_count++;
        hmi_task2_diagnostics.state = HMI_TASK2_STATE_ERROR;
    }

    if (rx_size != 0u)
    {
        if (rx_size > sizeof(hmi_task2_rx_buffer))
        {
            rx_size = sizeof(hmi_task2_rx_buffer);
        }
        hmi_task2_rx_active = 0u;
        hmi_task2_cache_finish_rx(hmi_task2_rx_buffer, rx_size);
        hmi_task2_diagnostics.rx_event_count++;
        hmi_task2_parse_commands(hmi_task2_rx_buffer, rx_size);
    }

    if (hmi_task2_rx_active == 0u)
    {
        (void)hmi_task2_start_rx();
    }

    /*
     * 先固定本轮需要预装的显示快照，再选择一个 DMA 动作。
     * 每轮最多启动一项发送，主循环不会被约 7.8 KB 的曲线命令阻塞。
     */
    hmi_task2_refresh_work_snapshot();
    if (hmi_task2_tx_active == 0u)
    {
        action = hmi_task2_select_action();
        if (action != HMI_TX_ACTION_NONE)
        {
            (void)hmi_task2_start_tx(action);
        }
    }
}

/**
 * @brief USART Receive-to-IDLE 回调，只记录本次收到的字节数。
 * @param huart 产生事件的 UART 句柄。
 * @param size DMA 已收到的字节数。
 * @return 无。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart == hmi_task2_uart)
    {
        hmi_uart_rx_event_size = size;
    }
}

/**
 * @brief USART TX DMA 完成回调，只置发送完成标志。
 * @param huart 产生事件的 UART 句柄。
 * @return 无。
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == hmi_task2_uart)
    {
        hmi_uart_tx_complete_flag = 1u;
    }
}

/**
 * @brief USART 错误回调，只置错误标志。
 * @param huart 产生错误的 UART 句柄。
 * @return 无。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == hmi_task2_uart)
    {
        hmi_uart_error_flag = 1u;
    }
}
