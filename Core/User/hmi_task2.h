/**
 * @file hmi_task2.h
 * @brief G 题淘晶驰串口屏稳定锁存显示与按键切换接口。
 *
 * 模块用途：经 USART1 DMA 把连续三帧稳定的参数和频谱锁存显示一次；波形上电隐藏，
 *          开始键显示缓存，周期键选择并重画严格的一周期或三周期波形。
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

/** 串口屏稳定锁存显示状态。 */
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
    uint32_t preload_complete_count; /**< 当前波形、频谱和参数完整刷新次数。 */
    uint32_t stable_accept_count;  /**< 连续三帧稳定并锁存为屏幕结果的次数。 */
    uint32_t stable_reject_count;  /**< 与当前锁存结果相近、无需重画的帧数。 */
    uint32_t calibration_toggle_count; /**< 已校准/未校准按键切换次数。 */
    uint32_t last_source_sequence; /**< 当前刷新工作快照序号。 */
    uint32_t last_visible_sequence;/**< 当前可见曲线对应的快照序号。 */
    uint16_t last_tx_bytes;        /**< 最近一次 DMA 发送字节数。 */
    uint8_t last_command;          /**< 最近一次有效命令：1/2/3/4为图形/开始，0x10保留，0x20切换校准。 */
    uint8_t requested_mode;        /**< 用户要求显示的模式。 */
    uint8_t visible_mode;          /**< 屏幕当前已切换的模式。 */
    uint8_t stable_candidate_count;/**< 当前候选结果已连续稳定的帧数，范围0~3。 */
    uint8_t calibration_enabled;   /**< 当前数字显示模式：1=已校准，0=未校准。 */
    hmi_task2_state_t state;       /**< 当前刷新状态。 */
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
 * @brief 处理按键、最新快照、实时刷新和可见性切换状态机。
 * @param 无。
 * @return 无。
 * @note 函数不等待 DMA 完成；任一时刻只发送一项，过期快照不会排队。
 */
void hmi_task2_process(void);

#endif /* HMI_TASK2_H */
