#ifndef SYSTEM_H
#define SYSTEM_H

/*
 * 模块用途：所有用户模块的唯一统一头文件入口。
 * GPIO 映射：PC0=ADC 输入，PA4/PA5=DAC 输出，PB14/PB15=USART1。
 * 外设依赖：ADC1、DAC1、TIM2、USART1、DMA1。
 * 初始化方法：main.c 在 CubeMX 外设初始化完成后只调用 system_init()。
 * 调用方法：main.c 的 while(1) 中只调用 system_process()。
 */

#include "main.h"
#include "signal_separation_config.h"
#include "signal_separation.h"
#include "uart_debug.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* CubeMX 在 main.c 中定义的外设句柄。 */
extern ADC_HandleTypeDef hadc1;
extern DAC_HandleTypeDef hdac1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;

/*
 * DMA 回调与主循环之间只用单一目的标志交接；回调中不做计算、打印或
 * 缓冲区处理。标志由 ISR 置 1，由 signal_separation_process() 清除。
 */
extern volatile uint8_t adc_half_ready_flag;
extern volatile uint8_t adc_full_ready_flag;
extern volatile uint8_t dac1_half_ready_flag;
extern volatile uint8_t dac1_full_ready_flag;
extern volatile uint8_t dac2_half_ready_flag;
extern volatile uint8_t dac2_full_ready_flag;

/**
 * @brief 初始化全部用户模块。
 * @param 无。
 * @return 无。
 */
void system_init(void);

/**
 * @brief 执行全部需要在主循环运行的用户功能。
 * @param 无。
 * @return 无。
 */
void system_process(void);

#endif
