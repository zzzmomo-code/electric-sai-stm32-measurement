/**
 * @file system.h
 * @brief STM32 用户代码唯一统一头文件入口。
 *
 * 模块用途：集中包含 CubeMX 外设句柄和全部用户模块头文件，供 main.c 与 Core/User
 *          下的实现文件统一使用。
 * GPIO 引脚映射：本模块无直接 GPIO；FPGA SPI3 和串口屏 USART1 映射见对应模块。
 * 依赖的外设和 CubeIDE 配置：SPI3、USART1、DMA、EXTI1，以及 CubeMX 仍保留的旧外设。
 * 初始化方法：HAL 与全部 MX_*_Init() 完成后调用 system_init()。
 * 调用方法：main.c 的 while(1) 仅持续调用 system_process()。
 */

#ifndef SYSTEM_H
#define SYSTEM_H

#include "main.h"
#include "math.h"

/* 保留旧模块头文件，使尚未从工程排除的历史源文件仍可独立编译。 */
#include "measurement_result.h"
#include "measurement_fft.h"
#include "fft_f32_65536.h"
#include "adc_dual.h"
#include "frequency_measure.h"
#include "ad9834.h"
#include "dds_control.h"
#include "dac_output.h"

/* G 题正式链路：FPGA SPI 协议、测量换算和串口屏后台预装。 */
#include "fpga_protocol.h"
#include "fpga_link.h"
#include "measurement_conversion.h"
#include "hmi_chart.h"
#include "hmi_task2.h"

/* CubeMX 生成的旧外设声明继续提供给历史模块，正常主流程不主动启动它们。 */
#if defined(__has_include)
#if __has_include("adc.h") && __has_include("tim.h")
#include "adc.h"
#include "tim.h"
#define SYSTEM_ADC_DUAL_AVAILABLE 1
#endif
#endif

#if defined(__has_include)
#if __has_include("dac.h")
#include "dac.h"
#endif
#if __has_include("opamp.h")
#include "opamp.h"
#endif
#endif

/* SPI3 是 FPGA 正式通信外设；SPI2 仅为旧器件驱动保留。 */
#if defined(__has_include)
#if __has_include("spi.h")
#include "spi.h"
#define SYSTEM_SPI2_AVAILABLE 1
#define SYSTEM_SPI3_AVAILABLE 1
#endif
#endif

/* USART1 是淘晶驰串口屏正式通信外设。 */
#if defined(__has_include)
#if __has_include("usart.h")
#include "usart.h"
#define SYSTEM_USART1_AVAILABLE 1
#endif
#endif

/** 双 ADC DMA 前半区完成标志，仅供仍参与编译的历史模块使用。 */
extern volatile uint8_t adc_dual_dma_half_flag;

/** 双 ADC DMA 后半区完成标志，仅供仍参与编译的历史模块使用。 */
extern volatile uint8_t adc_dual_dma_full_flag;

/** 双 ADC 错误标志，仅供仍参与编译的历史模块使用。 */
extern volatile uint8_t adc_dual_error_flag;

/** FFT 诊断量，仅供仍参与编译的历史模块使用。 */
extern measurement_fft_diagnostics_t measurement_fft_diagnostics;

/**
 * @brief 初始化 FPGA SPI、测量转换和串口屏三个正式用户模块。
 * @param 无。
 * @return 无。
 * @note 必须在 CubeMX 生成的 SPI3、USART1、DMA、GPIO 初始化后调用。
 */
void system_init(void);

/**
 * @brief 执行 FPGA 取帧、显示坐标转换和串口屏后台预装。
 * @param 无。
 * @return 无。
 */
void system_process(void);

#endif /* SYSTEM_H */
