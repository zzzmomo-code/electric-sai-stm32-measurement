/**
 * @file system.h
 * @brief 用户代码统一头文件入口。
 *
 * 模块用途：集中包含 HAL 生成头文件与全部用户模块头文件。
 * GPIO 引脚映射：统一入口无直接 GPIO；VGA 模块使用 PA4/DAC1_OUT1，其他映射见对应模块说明。
 * 依赖的外设和 CubeIDE 配置：依赖 CubeMX 生成的 main.h、dac.h、adc.h、tim.h、spi.h，
 * 启用串口屏时还依赖 usart.h。DAC1_OUT1 配置为无触发并开启输出缓冲。
 * 旧 ADS8688 模块已从片上 ADC 工程的活动构建中排除。
 * 初始化方法：HAL 与 MX_* 初始化完成后调用 system_init()。
 * 调用方法：main.c 及其他用户 .c 文件仅包含本头文件。
 */

#ifndef SYSTEM_H
#define SYSTEM_H

#include "main.h"
#include "dac.h"
#include "math.h"
#include "measurement_result.h"
#include "hmi_tjc.h"
#include "measurement_fft.h"
#include "fft_f32_65536.h"
#include "adc_dual.h"
#include "frequency_measure.h"
#include "ad9834.h"
#include "dds_control.h"
#include "vga_control.h"

/* ADC1/ADC2 与 TIM2 由用户完成 CubeMX 配置并生成后自动启用真实采集实现。 */
#if defined(__has_include)
#if __has_include("adc.h") && __has_include("tim.h")
#include "adc.h"
#include "tim.h"
#define SYSTEM_ADC_DUAL_AVAILABLE 1
#endif
#endif

/* SPI2由CubeMX生成后，统一头文件自动纳入其句柄声明。 */
#if defined(__has_include)
#if __has_include("spi.h")
#include "spi.h"
#define SYSTEM_SPI2_AVAILABLE 1
#endif
#endif

/* USART1 由 CubeMX 生成后，统一头文件自动纳入其句柄声明。 */
#if defined(__has_include)
#if __has_include("usart.h")
#include "usart.h"
#define SYSTEM_USART1_AVAILABLE 1
#endif
#endif

/** 双 ADC DMA 前半区完成标志，由 ADC1 DMA 回调与主循环共享。 */
extern volatile uint8_t adc_dual_dma_half_flag;

/** 双 ADC DMA 后半区完成标志，由 ADC1 DMA 回调与主循环共享。 */
extern volatile uint8_t adc_dual_dma_full_flag;

/** 双 ADC 错误标志，由 ADC 错误回调与主循环共享。 */
extern volatile uint8_t adc_dual_error_flag;

/** FFT 运行诊断快照，仅供主循环和调试器读取最近一帧分析结果。 */
extern measurement_fft_diagnostics_t measurement_fft_diagnostics;

/**
 * @brief 初始化全部用户模块。
 * @param 无。
 * @return 无。
 * @note 必须在 CubeMX 生成的外设初始化完成后调用。
 */
void system_init(void);

/**
 * @brief 执行全部主循环用户功能。
 * @param 无。
 * @return 无。
 * @note 依次处理外部频率、双 ADC、FFT 状态和串口屏，main.c 不放置业务逻辑。
 */
void system_process(void);

#endif /* SYSTEM_H */
