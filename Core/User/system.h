/**
 * @file system.h
 * @brief 用户代码统一头文件入口。
 *
 * 模块用途：统一包含用户模块、HAL 类型和跨文件外设句柄声明。
 * GPIO 引脚：PC0=ADC1_INP10，PA4=DAC1_OUT1，PB14/PB15=USART1。
 * 依赖外设：ADC1、DAC1、TIM2、DMA1、USART1。
 * 初始化方法：main.c 只调用 system_init()。
 * 调用方法：main.c 主循环只调用 system_process()。
 */

#ifndef USER_SYSTEM_H
#define USER_SYSTEM_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifndef PHASE_LOCK_HOST_TEST
#include "main.h"
#endif

#include "config.h"
#include "nco.h"
#include "dpll.h"
#include "signal_chain.h"
#include "uart_debug.h"

#ifndef PHASE_LOCK_HOST_TEST
extern ADC_HandleTypeDef hadc1;
extern DAC_HandleTypeDef hdac1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_dac1_ch1;
#endif

void system_init(void);
void system_process(void);

#endif /* USER_SYSTEM_H */
