/**
 * @file system.h
 * @brief 用户代码统一头文件入口。
 *
 * 模块用途：集中包含 HAL 生成头文件与全部用户模块头文件。
 * GPIO 引脚映射：无直接 GPIO 引脚，各模块映射见对应模块说明。
 * 依赖的外设和 CubeIDE 配置：依赖 CubeMX 生成的 main.h、spi.h；启用串口屏时还依赖 usart.h。
 * 初始化方法：HAL 与 MX_* 初始化完成后调用 system_init()。
 * 调用方法：main.c 及其他用户 .c 文件仅包含本头文件。
 */

#ifndef SYSTEM_H
#define SYSTEM_H

#include "main.h"
#include "spi.h"
#include "ads8688.h"
#include "ads8688_storage.h"
#include "measurement_result.h"
#include "hmi_tjc.h"

/* USART1 由 CubeMX 生成后，统一头文件自动纳入其句柄声明。 */
#if defined(__has_include)
#if __has_include("usart.h")
#include "usart.h"
#define SYSTEM_USART1_AVAILABLE 1
#endif
#endif

/** DMA 前半区完成标志，由 SPI2 DMA 中断与主循环共享。 */
extern volatile uint8_t ads8688_dma_half_flag;

/** DMA 后半区完成标志，由 SPI2 DMA 中断与主循环共享。 */
extern volatile uint8_t ads8688_dma_full_flag;

/** SPI2/DMA 错误标志，由 SPI2 中断与主循环共享。 */
extern volatile uint8_t ads8688_error_flag;

/**
 * @brief 初始化全部用户模块。
 * @param 无。
 * @return 无。
 * @note 必须在 CubeMX 生成的外设初始化完成后调用。
 */
void system_init(void);

#endif /* SYSTEM_H */
