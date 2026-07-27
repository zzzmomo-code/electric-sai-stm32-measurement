/**
 * @file hmi_task2.c
 * @brief FPGA 功率与 Bode 图串口屏实现。
 *
 * 模块用途：周期刷新 t_power，并用 cle + add 分步刷新 s0 幅频曲线和
 *          s1 相频曲线。页面不依赖其他 Text 控件。
 * GPIO 引脚映射：PA9/USART1_TX 接串口屏 RX，PA10/USART1_RX 接串口屏 TX。
 * 依赖的外设和 CubeIDE 配置：USART1 9600 8N1、全局中断、不使用 DMA。
 * 初始化方法：system_init() 调用 init 后绑定 huart1。
 * 调用方法：system_process() 每轮调用 hmi_task2_process()。
 */

#include "system.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define HMI_TASK2_POWER_REFRESH_MS       500u
#define HMI_TASK2_BODE_REFRESH_MS       2000u
#define HMI_TASK2_POWER_FRAME_SIZE       48u
#define HMI_TASK2_POWER_TX_TIMEOUT_MS    100u
#define HMI_TASK2_COMMAND_TX_TIMEOUT_MS  50u

volatile hmi_task2_diagnostics_t hmi_task2_diagnostics;

/** USART1 句柄。 */
static UART_HandleTypeDef *hmi_task2_uart;

/** USART1 单字节接收状态。 */
static uint8_t hmi_task2_rx_byte;
static volatile uint8_t hmi_task2_rx_flag;
static volatile uint8_t hmi_task2_error_flag;

/** 周期调度状态。 */
static uint32_t hmi_task2_last_power_ms;
static uint32_t hmi_task2_last_bode_start_ms;

/**
 * Bode 构帧缓冲区与源快照均放在静态区，避免占用当前仅 1 KB 的主栈。
 */
static uint8_t hmi_task2_bode_frame[HMI_CHART_FRAME_SIZE_TOTAL];
static fpga_link_bode_t hmi_task2_bode_snapshot;
static uint16_t hmi_task2_bode_frame_size;
static uint16_t hmi_task2_bode_frame_offset;
static uint32_t hmi_task2_bode_source_frame;

/**
 * @brief 追加一条 t_power 文本赋值命令。
 * @param frame 输出缓冲区。
 * @param value 要显示的 ASCII 文本。
 * @return 实际命令长度；空间不足返回 0。
 */
static uint16_t hmi_task2_build_power_command(uint8_t *frame,
                                               const char *value)
{
    int length = snprintf((char *)frame, HMI_TASK2_POWER_FRAME_SIZE,
                          "t_power.txt=\"%s\"", value);

    if ((length < 0)
        || (((uint16_t)length + 3u) > HMI_TASK2_POWER_FRAME_SIZE))
    {
        return 0u;
    }

    frame[length++] = 0xffu;
    frame[length++] = 0xffu;
    frame[length++] = 0xffu;
    return (uint16_t)length;
}

/**
 * @brief 发送当前功率值到 t_power。
 * @param 无。
 * @return 无；结果写入诊断量。
 */
static void hmi_task2_refresh_power(void)
{
    measurement_result_t result;
    uint8_t frame[HMI_TASK2_POWER_FRAME_SIZE];
    char power_text[24];
    uint16_t frame_size;

    (void)measurement_result_get_snapshot(&result);
    if (((result.valid_mask & MEASUREMENT_VALID_POWER) != 0u)
        && isfinite(result.power_w))
    {
        (void)snprintf(power_text, sizeof(power_text), "%.3f W",
                       (double)result.power_w);
    }
    else
    {
        (void)snprintf(power_text, sizeof(power_text), "--");
    }

    frame_size = hmi_task2_build_power_command(frame, power_text);
    if (frame_size == 0u)
    {
        hmi_task2_diagnostics.tx_error_count++;
        return;
    }

    if (HAL_UART_Transmit(hmi_task2_uart, frame, frame_size,
                          HMI_TASK2_POWER_TX_TIMEOUT_MS) == HAL_OK)
    {
        hmi_task2_diagnostics.tx_count++;
    }
    else
    {
        hmi_task2_diagnostics.tx_error_count++;
    }
}

/**
 * @brief 处理原任务保留的单字节按键协议。
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
            hmi_task2_diagnostics.command_count++;
        }
        else
        {
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
 * @brief 若当前空闲且有新 FPGA 数据，则构建下一张 Bode 图。
 * @param now 当前 HAL 毫秒计数。
 * @return 无。
 */
static void hmi_task2_prepare_bode_frame(uint32_t now)
{
    hmi_chart_status_t status;

    if (hmi_task2_bode_frame_offset < hmi_task2_bode_frame_size)
    {
        return;
    }
    if ((uint32_t)(now - hmi_task2_last_bode_start_ms)
        < HMI_TASK2_BODE_REFRESH_MS)
    {
        return;
    }
    if (fpga_link_get_bode(&hmi_task2_bode_snapshot) == 0u)
    {
        return;
    }
    if (hmi_task2_bode_snapshot.frame_count
        == hmi_task2_diagnostics.last_bode_source_frame)
    {
        return;
    }

    status = hmi_chart_build_bode_frame(
        &hmi_task2_bode_snapshot,
        hmi_task2_bode_frame,
        (uint16_t)sizeof(hmi_task2_bode_frame),
        &hmi_task2_bode_frame_size);
    if (status != HMI_CHART_STATUS_OK)
    {
        hmi_task2_bode_frame_size = 0u;
        hmi_task2_bode_frame_offset = 0u;
        hmi_task2_diagnostics.bode_error_count++;
        return;
    }

    hmi_task2_bode_frame_offset = 0u;
    hmi_task2_bode_source_frame = hmi_task2_bode_snapshot.frame_count;
    hmi_task2_last_bode_start_ms = now;
}

/**
 * @brief 发送 Bode 帧中的下一条完整 cle/add 命令。
 * @param 无。
 * @return 无。
 * @note 只在 FF FF FF 边界切分，不会把 t_power 命令插入半条 add 命令中。
 */
static void hmi_task2_send_next_bode_command(void)
{
    uint16_t end;
    uint16_t command_size;
    uint8_t terminator_found = 0u;

    if (hmi_task2_bode_frame_offset >= hmi_task2_bode_frame_size)
    {
        return;
    }

    end = hmi_task2_bode_frame_offset;
    while ((end + 2u) < hmi_task2_bode_frame_size)
    {
        if ((hmi_task2_bode_frame[end] == 0xffu)
            && (hmi_task2_bode_frame[end + 1u] == 0xffu)
            && (hmi_task2_bode_frame[end + 2u] == 0xffu))
        {
            end = (uint16_t)(end + 3u);
            terminator_found = 1u;
            break;
        }
        end++;
    }

    if ((terminator_found == 0u)
        || (end <= hmi_task2_bode_frame_offset)
        || (end > hmi_task2_bode_frame_size))
    {
        hmi_task2_diagnostics.bode_error_count++;
        hmi_task2_bode_frame_size = 0u;
        hmi_task2_bode_frame_offset = 0u;
        return;
    }

    command_size = (uint16_t)(end - hmi_task2_bode_frame_offset);
    if (HAL_UART_Transmit(
            hmi_task2_uart,
            &hmi_task2_bode_frame[hmi_task2_bode_frame_offset],
            command_size,
            HMI_TASK2_COMMAND_TX_TIMEOUT_MS) != HAL_OK)
    {
        hmi_task2_diagnostics.bode_error_count++;
        hmi_task2_bode_frame_size = 0u;
        hmi_task2_bode_frame_offset = 0u;
        return;
    }

    hmi_task2_bode_frame_offset = end;
    hmi_task2_diagnostics.bode_command_count++;
    if (hmi_task2_bode_frame_offset >= hmi_task2_bode_frame_size)
    {
        hmi_task2_diagnostics.bode_frame_count++;
        hmi_task2_diagnostics.last_bode_source_frame =
            hmi_task2_bode_source_frame;
        hmi_task2_bode_frame_size = 0u;
        hmi_task2_bode_frame_offset = 0u;
    }
}

void hmi_task2_init(void)
{
    memset((void *)&hmi_task2_diagnostics, 0,
           sizeof(hmi_task2_diagnostics));
    memset((void *)hmi_task2_bode_frame, 0,
           sizeof(hmi_task2_bode_frame));
    memset((void *)&hmi_task2_bode_snapshot, 0,
           sizeof(hmi_task2_bode_snapshot));
    hmi_task2_uart = NULL;
    hmi_task2_rx_byte = 0u;
    hmi_task2_rx_flag = 0u;
    hmi_task2_error_flag = 0u;
    hmi_task2_last_power_ms = 0u;
    hmi_task2_last_bode_start_ms = 0u;
    hmi_task2_bode_frame_size = 0u;
    hmi_task2_bode_frame_offset = 0u;
    hmi_task2_bode_source_frame = 0u;
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
        (void)HAL_UART_Receive_IT(
            hmi_task2_uart, &hmi_task2_rx_byte, 1u);
    }

    if (hmi_task2_rx_flag != 0u)
    {
        hmi_task2_rx_flag = 0u;
        hmi_task2_handle_command(hmi_task2_rx_byte);
        if (HAL_UART_Receive_IT(
                hmi_task2_uart, &hmi_task2_rx_byte, 1u) != HAL_OK)
        {
            hmi_task2_diagnostics.error_count++;
        }
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - hmi_task2_last_power_ms)
        >= HMI_TASK2_POWER_REFRESH_MS)
    {
        hmi_task2_last_power_ms = now;
        hmi_task2_refresh_power();
    }

    hmi_task2_prepare_bode_frame(now);
    hmi_task2_send_next_bode_command();
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
    else
    {
        fpga_link_handle_error(huart);
    }
}
