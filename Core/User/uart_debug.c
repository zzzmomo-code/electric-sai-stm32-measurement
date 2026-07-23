/**
 * @file uart_debug.c
 * @brief USART1 单消息非阻塞调试接口。
 *
 * 模块用途：接收单字节命令，并用一个静态缓冲区非阻塞发送调试文本。
 * GPIO 引脚：PB14=USART1_TX，PB15=USART1_RX。
 * 依赖外设：USART1，115200-8-N-1，USART1_IRQn。
 * 初始化方法：CubeMX 初始化 USART1 后调用 uart_debug_init()。
 * 调用方法：主循环调用 uart_debug_process()；忙时直接放弃新消息。
 */

#include "system.h"

volatile uint8_t uart_tx_complete_flag = 0u;
volatile uint8_t uart_rx_ready_flag = 0u;
volatile uint8_t uart_error_flag = 0u;

static uint8_t uart_tx_buffer[UART_DEBUG_TX_BUFFER_SIZE];
static uint8_t uart_rx_byte = 0u;
static uint8_t uart_command_byte = 0u;
static uint8_t uart_command_ready = 0u;
static uint8_t uart_tx_busy = 0u;

/**
 * @brief 原子读取并清除一个 UART 中断标志。
 * @param flag 待处理的单字节标志。
 * @return 清除前的标志值；短暂屏蔽中断。
 */
static uint8_t uart_take_flag(volatile uint8_t *flag)
{
    uint8_t value;
    const uint32_t interrupt_mask = __get_PRIMASK();

    __disable_irq();
    value = *flag;
    *flag = 0u;
    __set_PRIMASK(interrupt_mask);
    return value;
}

/**
 * @brief 初始化串口软件状态并启动单字节中断接收。
 * @param 无。
 * @return 无。
 * @note 不阻塞等待串口。
 */
void uart_debug_init(void)
{
    uart_tx_complete_flag = 0u;
    uart_rx_ready_flag = 0u;
    uart_error_flag = 0u;
    uart_command_ready = 0u;
    uart_tx_busy = 0u;
    (void)HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1u);
}

/**
 * @brief 处理 UART 中断标志并重新挂起接收。
 * @param 无。
 * @return 无。
 * @note 必须由主循环调用。
 */
void uart_debug_process(void)
{
    if (uart_take_flag(&uart_tx_complete_flag) != 0u)
    {
        uart_tx_busy = 0u;
    }

    if (uart_take_flag(&uart_rx_ready_flag) != 0u)
    {
        uart_command_byte = uart_rx_byte;
        uart_command_ready = 1u;
        (void)HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1u);
    }

    if (uart_take_flag(&uart_error_flag) != 0u)
    {
        (void)HAL_UART_AbortReceive(&huart1);
        (void)HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1u);
    }
}

/**
 * @brief 非阻塞发送一条短文本。
 * @param text 待发送的零结尾 ASCII 字符串。
 * @return 1 表示已启动发送，0 表示串口忙或参数无效。
 * @note 文本复制到静态缓冲区，最大 255 字节。
 */
uint8_t uart_debug_write(const char *text)
{
    size_t length;

    if ((text == NULL) || (uart_tx_busy != 0u))
    {
        return 0u;
    }

    length = strlen(text);
    if (length >= UART_DEBUG_TX_BUFFER_SIZE)
    {
        length = UART_DEBUG_TX_BUFFER_SIZE - 1u;
    }
    memcpy(uart_tx_buffer, text, length);

    if (HAL_UART_Transmit_IT(&huart1, uart_tx_buffer, (uint16_t)length) != HAL_OK)
    {
        return 0u;
    }

    uart_tx_busy = 1u;
    return 1u;
}

/**
 * @brief 读取一个接收完成的命令。
 * @param command 接收命令字节的指针。
 * @return 1 表示读取成功，0 表示没有命令。
 * @note 单字节命令槽未读取时，新命令覆盖旧命令。
 */
uint8_t uart_debug_read_command(uint8_t *command)
{
    if ((command == NULL) || (uart_command_ready == 0u))
    {
        return 0u;
    }

    *command = uart_command_byte;
    uart_command_ready = 0u;
    return 1u;
}

/**
 * @brief USART1 发送完成回调。
 * @param huart UART 句柄。
 * @return 无。
 * @note 中断上下文只设置一个发送完成标志。
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uart_tx_complete_flag = 1u;
    }
}

/**
 * @brief USART1 接收完成回调。
 * @param huart UART 句柄。
 * @return 无。
 * @note 中断上下文只设置一个接收完成标志。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uart_rx_ready_flag = 1u;
    }
}

/**
 * @brief USART1 错误回调。
 * @param huart UART 句柄。
 * @return 无。
 * @note 中断上下文只设置一个错误标志。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uart_error_flag = 1u;
    }
}
