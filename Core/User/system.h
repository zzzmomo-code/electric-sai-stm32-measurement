#ifndef SYSTEM_H
#define SYSTEM_H

/*
 * 模块用途：所有用户模块的唯一统一头文件入口。
 * GPIO 映射：PC0=ADC 输入，PA4=主 DAC 输出，PA5=双信号模式第二 DAC 输出，
 *            PB14/PB15=USART1。
 * 外设依赖：ADC1、DAC1、TIM2、USART1、DMA1。
 * 初始化方法：main.c 在 CubeMX 外设初始化完成后只调用 system_init()。
 * 调用方法：main.c 的 while(1) 中只调用 system_process()。
 */

#include "main.h"
#include "signal_separation_config.h"
#include "frequency_estimator.h"
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
 * DMA 回调与主循环之间只用单一目的事件标志交接；每次半传输或全传输完成时，
 * 对应 ISR 只递增一个 32 位单调计数。主循环不清零计数，而是比较已处理序号，
 * 从而能够识别重复事件、丢帧和 DMA 绝对采样时间。
 */
extern volatile uint32_t adc_dma_event_flag;
extern volatile uint32_t dac1_dma_event_flag;
extern volatile uint32_t dac2_dma_event_flag;

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
