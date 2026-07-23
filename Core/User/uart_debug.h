#ifndef UART_DEBUG_H
#define UART_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 模块用途：通过 USART1 输出启动信息和低频率运行状态。
 * GPIO 映射：PB14=USART1_TX，PB15=USART1_RX。
 * 外设依赖：USART1，115200 bit/s，8N1，并启用 USART1 全局中断。
 * 初始化方法：system_init() 调用 uart_debug_init()。
 * 调用方法：system_process() 调用 uart_debug_process()。
 */

/**
 * @brief 在高速采样启动前输出一次工程和引脚信息。
 * @param 无。
 * @return 无。
 */
void uart_debug_init(void);

/**
 * @brief 在首次识别完成时用中断方式输出一次结果。
 * @param 无。
 * @return 无。
 * @note 发送缓冲区为静态存储，发送期间不会被主循环覆盖。
 */
void uart_debug_process(void);

#ifdef __cplusplus
}
#endif

#endif
