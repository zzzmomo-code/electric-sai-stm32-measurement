/**
 * @file uart_debug.h
 * @brief USART1 非阻塞调试输出和单字节命令接口。
 *
 * 模块用途：使用单个静态缓冲区中断发送状态，并接收模式切换命令。
 * GPIO 引脚：PB14=USART1_TX，PB15=USART1_RX。
 * 依赖外设：USART1，115200-8-N-1，NVIC 优先级 10。
 * 初始化方法：system_init() 中调用 uart_debug_init()。
 * 调用方法：主循环调用 uart_debug_process()，其他模块调用 uart_debug_write()。
 */

#ifndef USER_UART_DEBUG_H
#define USER_UART_DEBUG_H

#include <stdint.h>

extern volatile uint8_t uart_tx_complete_flag;
extern volatile uint8_t uart_rx_ready_flag;
extern volatile uint8_t uart_error_flag;

void uart_debug_init(void);
void uart_debug_process(void);
uint8_t uart_debug_write(const char *text);
uint8_t uart_debug_read_command(uint8_t *command);

#endif /* USER_UART_DEBUG_H */
