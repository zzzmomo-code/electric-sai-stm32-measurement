#include "system.h"

/*
 * 模块用途：组织用户模块的初始化顺序和主循环调度。
 * GPIO 映射：本模块无直接 GPIO 引脚。
 * 外设依赖：由 signal_separation 和 uart_debug 模块间接使用外设。
 * 初始化方法：main.c 调用 system_init()。
 * 调用方法：main.c 的 while(1) 调用 system_process()。
 */

/**
 * @brief 初始化串口提示和高速信号分离链路。
 * @param 无。
 * @return 无。
 * @note 先完成阻塞式启动打印，再启动 2.5 MSPS 实时采样。
 */
void system_init(void)
{
  uart_debug_init();
  signal_separation_start();
}

/**
 * @brief 调度实时信号处理和非周期性串口状态输出。
 * @param 无。
 * @return 无。
 */
void system_process(void)
{
  signal_separation_process();
  uart_debug_process();
}
