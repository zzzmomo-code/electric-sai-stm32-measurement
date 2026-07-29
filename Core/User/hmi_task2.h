/**
 * @file hmi_task2.h
 * @brief G 题淘晶驰串口屏后台预装与按键切换接口。
 *
 * 模块用途：经 USART1 DMA 把一周期、三周期和频谱三组 350 点数据预装到屏幕控件，
 *          并解析屏幕按键，在不重新测量和计算的情况下切换时域控件并保持频谱可见。
 * GPIO 引脚映射：PA9/USART1_TX 接屏幕 RX，PA10/USART1_RX 接屏幕 TX。
 * 依赖的外设和 CubeIDE 配置：USART1 512000 baud、8N1、TX/RX DMA、USART1 全局中断。
 * 初始化方法：system_init() 调用 hmi_task2_init()，再绑定 huart1。
 * 调用方法：system_process() 持续调用 hmi_task2_process()。
 */

#ifndef HMI_TASK2_H
#define HMI_TASK2_H

#include <stdint.h>

#include "hmi_chart.h"

#if defined(HAL_UART_MODULE_ENABLED)
#include "stm32h7xx_hal_uart.h"
#endif

/** 串口屏后台预装状态。 */
typedef enum
{
    HMI_TASK2_STATE_WAIT_DATA = 0,
    HMI_TASK2_STATE_PRELOADING,
    HMI_TASK2_STATE_READY,
    HMI_TASK2_STATE_ERROR
} hmi_task2_state_t;

/** 串口屏诊断量，可直接加入 STM32CubeIDE Expressions。 */
typedef struct
{
    uint32_t rx_event_count;       /**< Receive-to-IDLE 事件次数。 */
    uint32_t rx_byte_count;        /**< 已处理的屏幕返回字节总数。 */
    uint32_t command_count;        /**< 已识别的 A5 CMD 5A 命令数。 */
    uint32_t invalid_command_count;/**< 非法或不完整命令计数。 */
    uint32_t tx_start_count;       /**< 成功启动 TX DMA 的次数。 */
    uint32_t tx_complete_count;    /**< TX DMA 完成次数。 */
    uint32_t tx_error_count;       /**< TX 启动、超时或 UART 错误次数。 */
    uint32_t preload_complete_count; /**< 三条曲线和参数全部预装完成次数。 */
    uint32_t last_source_sequence; /**< 当前后台工作快照序号。 */
    uint32_t last_visible_sequence;/**< 当前可见曲线对应的快照序号。 */
    uint16_t last_tx_bytes;        /**< 最近一次 DMA 发送字节数。 */
    uint8_t last_command;          /**< 最近一次有效模式命令，范围 1~3。 */
    uint8_t requested_mode;        /**< 用户要求显示的模式。 */
    uint8_t visible_mode;          /**< 屏幕当前已切换的模式。 */
    hmi_task2_state_t state;       /**< 当前预装状态。 */
} hmi_task2_diagnostics_t;

/** USART1 RX DMA 事件与主循环共享的接收长度，零表示无待处理事件。 */
extern volatile uint16_t hmi_uart_rx_event_size;

/** USART1 TX DMA 完成回调与主循环共享的完成标志。 */
extern volatile uint8_t hmi_uart_tx_complete_flag;

/** USART1 错误回调与主循环共享的错误标志。 */
extern volatile uint8_t hmi_uart_error_flag;

/** 串口屏公开诊断量。 */
extern volatile hmi_task2_diagnostics_t hmi_task2_diagnostics;

/**
 * @brief 初始化串口屏任务的软件状态。
 * @param 无。
 * @return 无。
 */
void hmi_task2_init(void);

/**
 * @brief 启用或关闭不依赖 FPGA 的 350 点三图自检数据。
 * @param enable 非零启用，零恢复使用 FPGA 测量快照。
 * @return 无。
 * @note 正常发布版本保持为零，仅用于串口屏工程联调。
 */
void hmi_task2_set_chart_self_test(uint8_t enable);

#if defined(HAL_UART_MODULE_ENABLED)
/**
 * @brief 绑定 USART1 并启动 Receive-to-IDLE DMA。
 * @param huart CubeMX 已初始化的 USART1 句柄。
 * @return 无；启动失败记入诊断量并由主循环重试。
 */
void hmi_task2_bind_uart(UART_HandleTypeDef *huart);
#endif

/**
 * @brief 处理按键、最新快照、后台预装和可见性切换状态机。
 * @param 无。
 * @return 无。
 * @note 函数不等待 DMA 完成；任一时刻只发送一项，过期快照不会排队。
 */
void hmi_task2_process(void);

#endif /* HMI_TASK2_H */
