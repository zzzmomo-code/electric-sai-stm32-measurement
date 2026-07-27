/**
 * @file hmi_task2.h
 * @brief FPGA 功率与 Bode 图串口屏接口。
 *
 * 模块用途：向 t_power 文本控件显示功率，并向 s0/s1 Waveform 控件发送
 *          幅频和相频曲线；保留原有单字节按键接收兼容接口。
 * GPIO 引脚映射：PA9/USART1_TX 接串口屏 RX，PA10/USART1_RX 接串口屏 TX。
 * 依赖的外设和 CubeIDE 配置：USART1 9600 8N1、全局中断、不使用 DMA。
 * 初始化方法：system_init() 调用 hmi_task2_init() 后绑定 huart1。
 * 调用方法：system_process() 持续调用 hmi_task2_process()。
 */

#ifndef HMI_TASK2_H
#define HMI_TASK2_H

#include <stdint.h>

#if defined(HAL_UART_MODULE_ENABLED)
#include "stm32h7xx_hal_uart.h"
#endif

/** 串口屏诊断量，供 STM32CubeIDE Expressions 查看。 */
typedef struct
{
    uint32_t tx_count;              /**< t_power 成功刷新次数。 */
    uint32_t tx_error_count;        /**< t_power 发送失败次数。 */
    uint32_t bode_frame_count;      /**< 完整 Bode 图发送完成次数。 */
    uint32_t bode_command_count;    /**< 成功发送的 cle/add 命令数。 */
    uint32_t bode_error_count;      /**< Bode 构帧或发送失败次数。 */
    uint32_t last_bode_source_frame; /**< 最近画完的 FPGA 帧号。 */
    uint32_t rx_count;              /**< 收到的按键字节数。 */
    uint32_t command_count;         /**< 成功执行的兼容按键命令数。 */
    uint32_t error_count;           /**< 无效命令、VGA 或 UART 错误数。 */
    uint8_t last_command;           /**< 最近收到的按键字节。 */
} hmi_task2_diagnostics_t;

extern volatile hmi_task2_diagnostics_t hmi_task2_diagnostics;

/**
 * @brief 初始化串口屏任务状态。
 * @param 无。
 * @return 无。
 */
void hmi_task2_init(void);

/**
 * @brief 启用或关闭串口屏双曲线自检。
 * @param enable 非零时生成一帧内部测试曲线，零时恢复使用 FPGA 数据。
 * @return 无。
 * @note 自检只生成一次 s0/s1 测试帧，不依赖 USART2 或 FPGA。
 */
void hmi_task2_set_chart_self_test(uint8_t enable);

#if defined(HAL_UART_MODULE_ENABLED)
/**
 * @brief 绑定 USART1 并启动单字节中断接收。
 * @param huart CubeMX 已初始化的 USART1 句柄。
 * @return 无；失败记录到诊断量。
 */
void hmi_task2_bind_uart(UART_HandleTypeDef *huart);
#endif

/**
 * @brief 处理按键、刷新 t_power，并分步发送 Bode 图。
 * @param 无。
 * @return 无。
 * @note 每次最多阻塞发送一条 cle/add 命令，避免 9600 波特率整帧阻塞约 2 秒。
 */
void hmi_task2_process(void);

#endif /* HMI_TASK2_H */
