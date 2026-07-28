/**
 * @file fpga_link.c
 * @brief FPGA 高速串口双向通信模块实现。
 *
 * 模块用途：用 USART2 Receive-to-IDLE DMA 接收 FPGA Bode 数据帧，
 *          在主循环验证同步头、点数、帧长和帧尾后发布稳定快照。
 * GPIO 引脚映射：PA2/USART2_TX 接 FPGA T19/RX，
 *          PA3/USART2_RX 接 FPGA J15/TX，双方共地。
 * 依赖的外设和 CubeIDE 配置：USART2 1 Mbaud 8N1、DMA1_Stream1 RX Normal、
 *          USART2 与 DMA1_Stream1 中断。
 * 初始化方法：system_init() 调用 init、bind 和 start。
 * 调用方法：system_process() 调用 fpga_link_process()。
 */

#include "system.h"

#include <string.h>

#define FPGA_LINK_STEP_INCREASE_COMMAND 0x2bu
#define FPGA_LINK_STEP_DECREASE_COMMAND 0x2du
#define FPGA_LINK_COMMAND_TX_TIMEOUT_MS 10u

/** DMA 写入缓冲区；长度和地址均按 32 字节缓存行对齐。 */
static uint8_t fpga_link_dma_buffer[FPGA_LINK_DMA_BUFFER_SIZE]
    __attribute__((aligned(32)));

/** 主循环发布的最新完整 Bode 数据。 */
static fpga_link_bode_t fpga_link_bode;

/** 绑定的 USART2 句柄。 */
static UART_HandleTypeDef *fpga_link_uart;

/**
 * Receive-to-IDLE 回调写入的接收长度；零表示没有待处理事件。
 * 该变量同时承担事件标志作用，回调只修改这一处共享状态。
 */
static volatile uint16_t fpga_link_rx_event_size;

/** USART2 错误回调置位，主循环领取并清除。 */
static volatile uint8_t fpga_link_error_flag;

volatile fpga_link_diagnostics_t fpga_link_diagnostics;

/**
 * @brief 判断 Cortex-M7 D-Cache 是否启用。
 * @param 无。
 * @return 已启用返回 1，否则返回 0。
 */
static uint8_t fpga_link_dcache_is_enabled(void)
{
    return ((SCB->CCR & SCB_CCR_DC_Msk) != 0u) ? 1u : 0u;
}

/**
 * @brief 在 DMA 写入前清理并失效整个接收缓冲区。
 * @param 无。
 * @return 无。
 * @note 缓冲区首地址和长度均为 32 字节整数倍，不会影响相邻变量。
 */
static void fpga_link_prepare_dma_buffer(void)
{
    if (fpga_link_dcache_is_enabled() != 0u)
    {
        SCB_CleanInvalidateDCache_by_Addr(
            (uint32_t *)fpga_link_dma_buffer,
            (int32_t)sizeof(fpga_link_dma_buffer));
    }
}

/**
 * @brief 在 CPU 读取前失效 DMA 已写入区域的缓存行。
 * @param received_size DMA 已写入的有效字节数。
 * @return 无。
 */
static void fpga_link_invalidate_received_data(uint16_t received_size)
{
    uint32_t cache_size;

    if (fpga_link_dcache_is_enabled() == 0u)
    {
        return;
    }

    cache_size = ((uint32_t)received_size + 31u) & ~31u;
    SCB_InvalidateDCache_by_Addr(
        (uint32_t *)fpga_link_dma_buffer,
        (int32_t)cache_size);
}

/**
 * @brief 启动一次 Normal 模式 Receive-to-IDLE DMA。
 * @param 无。
 * @return 成功返回 1，失败返回 0。
 */
static uint8_t fpga_link_start_receive(void)
{
    HAL_StatusTypeDef status;

    if ((fpga_link_uart == NULL) || (fpga_link_uart->hdmarx == NULL))
    {
        fpga_link_diagnostics.dma_start_error_count++;
        return 0u;
    }

    fpga_link_prepare_dma_buffer();
    status = HAL_UARTEx_ReceiveToIdle_DMA(
        fpga_link_uart,
        fpga_link_dma_buffer,
        (uint16_t)sizeof(fpga_link_dma_buffer));

    if (status != HAL_OK)
    {
        fpga_link_diagnostics.dma_start_error_count++;
        return 0u;
    }

    /* 不在半缓冲区回调；只在 IDLE 或缓冲区满时通知主循环。 */
    __HAL_DMA_DISABLE_IT(fpga_link_uart->hdmarx, DMA_IT_HT);
    fpga_link_diagnostics.dma_restart_count++;
    return 1u;
}

void fpga_link_init(void)
{
    memset((void *)&fpga_link_bode, 0, sizeof(fpga_link_bode));
    memset((void *)&fpga_link_diagnostics, 0,
           sizeof(fpga_link_diagnostics));
    memset((void *)fpga_link_dma_buffer, 0, sizeof(fpga_link_dma_buffer));
    fpga_link_uart = NULL;
    fpga_link_rx_event_size = 0u;
    fpga_link_error_flag = 0u;
}

#if defined(HAL_UART_MODULE_ENABLED)
void fpga_link_bind_uart(UART_HandleTypeDef *huart)
{
    fpga_link_uart = huart;
}
#endif

uint8_t fpga_link_start(void)
{
    return fpga_link_start_receive();
}

/**
 * @brief 在接收缓冲区中寻找 AA 55 同步头。
 * @param length 有效接收长度。
 * @return 同步头偏移；未找到返回 0xFFFF。
 */
static uint16_t fpga_link_find_header(uint16_t length)
{
    uint16_t index;

    for (index = 0u; (index + 1u) < length; index++)
    {
        if ((fpga_link_dma_buffer[index] == 0xaau)
            && (fpga_link_dma_buffer[index + 1u] == 0x55u))
        {
            return index;
        }
    }

    return 0xffffu;
}

/**
 * @brief 解析一个 Receive-to-IDLE 缓冲区中的第一帧完整数据。
 * @param received_size 本次接收事件的有效字节数。
 * @return 解析成功返回 1，否则返回 0。
 */
static uint8_t fpga_link_parse_frame(uint16_t received_size)
{
    uint16_t header_offset;
    uint16_t point_count;
    uint32_t expected_size;
    uint32_t tail_offset;
    uint16_t index;

    fpga_link_diagnostics.last_frame_bytes = received_size;
    if (received_size < 10u)
    {
        fpga_link_diagnostics.frame_error_count++;
        return 0u;
    }

    header_offset = fpga_link_find_header(received_size);
    if (header_offset == 0xffffu)
    {
        fpga_link_diagnostics.frame_error_count++;
        return 0u;
    }
    fpga_link_diagnostics.frame_found_count++;

    point_count =
        ((uint16_t)fpga_link_dma_buffer[header_offset + 2u] << 8)
        | (uint16_t)fpga_link_dma_buffer[header_offset + 3u];
    if ((point_count == 0u) || (point_count > FPGA_LINK_MAX_POINTS))
    {
        fpga_link_diagnostics.frame_error_count++;
        return 0u;
    }

    expected_size = 4u + ((uint32_t)point_count * 4u) + 2u;
    if (((uint32_t)header_offset + expected_size) > received_size)
    {
        fpga_link_diagnostics.frame_error_count++;
        return 0u;
    }

    tail_offset = (uint32_t)header_offset + expected_size - 2u;
    if ((fpga_link_dma_buffer[tail_offset] != 0x0du)
        || (fpga_link_dma_buffer[tail_offset + 1u] != 0x0au))
    {
        fpga_link_diagnostics.frame_error_count++;
        return 0u;
    }

    for (index = 0u; index < point_count; index++)
    {
        uint32_t data_offset =
            (uint32_t)header_offset + 4u + ((uint32_t)index * 4u);

        fpga_link_bode.mag2_hi[index] =
            ((uint16_t)fpga_link_dma_buffer[data_offset] << 8)
            | (uint16_t)fpga_link_dma_buffer[data_offset + 1u];
        fpga_link_bode.phase[index] =
            (int16_t)(
                ((uint16_t)fpga_link_dma_buffer[data_offset + 2u] << 8)
                | (uint16_t)fpga_link_dma_buffer[data_offset + 3u]);
    }

    fpga_link_bode.point_count = point_count;
    fpga_link_bode.frame_count++;
    fpga_link_bode.valid = 1u;
    fpga_link_diagnostics.last_point_count = point_count;
    fpga_link_diagnostics.frame_valid_count++;
    return 1u;
}

/**
 * @brief 领取回调事件并清零共享状态。
 * @param rx_size 输出接收事件长度。
 * @param error_flag 输出 UART 错误标志。
 * @return 无。
 */
static void fpga_link_claim_events(uint16_t *rx_size,
                                   uint8_t *error_flag)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    *rx_size = fpga_link_rx_event_size;
    fpga_link_rx_event_size = 0u;
    *error_flag = fpga_link_error_flag;
    fpga_link_error_flag = 0u;
    if (primask == 0u)
    {
        __enable_irq();
    }
}

void fpga_link_process(void)
{
    uint16_t rx_size;
    uint8_t error_flag;

    if (fpga_link_uart == NULL)
    {
        return;
    }

    fpga_link_claim_events(&rx_size, &error_flag);

    if (error_flag != 0u)
    {
        fpga_link_diagnostics.uart_error_count++;
        (void)HAL_UART_AbortReceive(fpga_link_uart);
        (void)fpga_link_start_receive();
        return;
    }

    if (rx_size == 0u)
    {
        return;
    }

    fpga_link_diagnostics.rx_event_count++;
    if (rx_size > sizeof(fpga_link_dma_buffer))
    {
        fpga_link_diagnostics.frame_error_count++;
    }
    else
    {
        fpga_link_invalidate_received_data(rx_size);
        (void)fpga_link_parse_frame(rx_size);
    }

    (void)HAL_UART_AbortReceive(fpga_link_uart);
    (void)fpga_link_start_receive();
}

uint8_t fpga_link_get_bode(fpga_link_bode_t *bode)
{
    if (bode == NULL)
    {
        return 0u;
    }

    *bode = fpga_link_bode;
    return fpga_link_bode.valid;
}

/**
 * @brief 通过 USART2 向 FPGA 发送一个扫频控制命令。
 * @param command 只允许 0x2B 或 0x2D。
 * @return 发送成功返回 1，参数、UART 状态或 HAL 发送异常返回 0。
 */
static uint8_t fpga_link_send_command(uint8_t command)
{
    HAL_StatusTypeDef status;

    if ((command != FPGA_LINK_STEP_INCREASE_COMMAND)
        && (command != FPGA_LINK_STEP_DECREASE_COMMAND))
    {
        fpga_link_diagnostics.command_tx_error_count++;
        return 0u;
    }
    if (fpga_link_uart == NULL)
    {
        fpga_link_diagnostics.command_tx_error_count++;
        return 0u;
    }

    status = HAL_UART_Transmit(
        fpga_link_uart, &command, 1u, FPGA_LINK_COMMAND_TX_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        fpga_link_diagnostics.command_tx_error_count++;
        return 0u;
    }

    fpga_link_diagnostics.command_tx_count++;
    fpga_link_diagnostics.last_tx_command = command;
    return 1u;
}

uint8_t fpga_link_send_step_increase(void)
{
    return fpga_link_send_command(FPGA_LINK_STEP_INCREASE_COMMAND);
}

uint8_t fpga_link_send_step_decrease(void)
{
    return fpga_link_send_command(FPGA_LINK_STEP_DECREASE_COMMAND);
}

#if defined(HAL_UART_MODULE_ENABLED)
/**
 * @brief HAL Receive-to-IDLE 事件回调。
 * @param huart 产生事件的 UART。
 * @param size DMA 已写入的有效字节数。
 * @return 无；回调只更新一个兼作标志的接收长度变量。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if ((huart == fpga_link_uart) && (size != 0u))
    {
        fpga_link_rx_event_size = size;
    }
}

void fpga_link_handle_error(UART_HandleTypeDef *huart)
{
    if (huart == fpga_link_uart)
    {
        fpga_link_error_flag = 1u;
    }
}
#endif
