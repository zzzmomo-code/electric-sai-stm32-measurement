/**
 * @file hmi_task2.h
 * @brief 外差式幅频测量训练题的串口屏接口。
 *
 * 模块用途：显示 Timer、ADC、DDS、VGA 和换算结果，并接收六档 VGA 与立即测量按键。
 * GPIO 引脚映射：PA9/USART1_TX 接串口屏 RX，PA10/USART1_RX 接串口屏 TX。
 * 依赖的外设和 CubeIDE 配置：USART1，9600 8N1，开启 USART1 全局中断，不使用 DMA。
 * 初始化方法：system_init() 中调用 hmi_task2_init()，随后绑定 huart1。
 * 调用方法：主循环持续调用 hmi_task2_process()。
 */

#ifndef HMI_TASK2_H
#define HMI_TASK2_H

#include <stdint.h>

#if defined(HAL_UART_MODULE_ENABLED)
#include "stm32h7xx_hal_uart.h"
#endif

/** 串口屏收发诊断，供调试器查看。 */
typedef struct
{
    uint32_t tx_count;       /**< 成功发送页面刷新帧的次数。 */
    uint32_t tx_error_count; /**< 页面刷新发送失败次数。 */
    uint32_t rx_count;       /**< 收到的按键字节数。 */
    uint32_t command_count;  /**< 成功执行的按键命令数。 */
    uint32_t error_count;    /**< 无效命令、VGA 或 UART 错误数。 */
    uint8_t last_command;    /**< 最近收到的按键字节。 */
} hmi_task2_diagnostics_t;

extern volatile hmi_task2_diagnostics_t hmi_task2_diagnostics;

/**
 * @brief 清零本题串口屏状态。
 * @param 无。
 * @return 无。
 */
void hmi_task2_init(void);

#if defined(HAL_UART_MODULE_ENABLED)
/**
 * @brief 绑定并启动 USART 单字节中断接收。
 * @param huart CubeMX 已初始化的 USART1 句柄。
 * @return 无，启动失败记录到 hmi_task2_diagnostics.error_count。
 */
void hmi_task2_bind_uart(UART_HandleTypeDef *huart);
#endif

/**
 * @brief 处理按键并按固定周期刷新本题页面。
 * @param 无。
 * @return 无。
 * @note 按键字符 '0'~'5' 选择 VGA 档位，'M' 请求立即执行一次 TIM5 测频。
 */
void hmi_task2_process(void);

#endif /* HMI_TASK2_H */
