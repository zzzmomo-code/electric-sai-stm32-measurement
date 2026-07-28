/**
 * @file fpga_link.h
 * @brief FPGA 高速串口双向通信模块公共接口。
 *
 * 模块用途：通过 USART2 以 1 Mbaud 接收 FPGA 发送的幅频和相频数据，
 *          使用 DMA + IDLE 事件确定一帧结束，并在主循环解析 AA55 协议。
 * GPIO 引脚映射：PA2/USART2_TX 接 FPGA T19/RX，
 *          PA3/USART2_RX 接 FPGA J15/TX，双方共地。
 * 依赖的外设和 CubeIDE 配置：USART2 1000000 8N1；DMA1_Stream1 RX、
 *          Peripheral-to-Memory、Normal、Byte/Byte、Memory Increment；
 *          开启 USART2 与 DMA1_Stream1 中断。
 * 初始化方法：system_init() 依次调用 fpga_link_init()、
 *          fpga_link_bind_uart(&huart2) 和 fpga_link_start()。
 * 调用方法：system_process() 持续调用 fpga_link_process()；
 *          HMI 模块调用 fpga_link_get_bode() 读取最新完整帧；
 *          主循环可调用步进增减接口向 FPGA 发送单字节命令。
 * FPGA 协议：AA 55 [N高 N低] [N组 mag2_hi高/低 phase高/低] 0D 0A。
 */

#ifndef FPGA_LINK_H
#define FPGA_LINK_H

#include <stdint.h>

#if defined(HAL_UART_MODULE_ENABLED)
#include "stm32h7xx_hal_uart.h"
#endif

/** FPGA 单帧最大点数。 */
#define FPGA_LINK_MAX_POINTS 1024u

/** 最大有效帧长度：4 字节头和点数 + 1024×4 字节数据 + 2 字节帧尾。 */
#define FPGA_LINK_FRAME_MAX_BYTES (4u + (FPGA_LINK_MAX_POINTS * 4u) + 2u)

/** DMA 缓冲区按 Cortex-M7 D-Cache 行大小向上对齐。 */
#define FPGA_LINK_DMA_BUFFER_SIZE \
    ((FPGA_LINK_FRAME_MAX_BYTES + 31u) & ~31u)

/** 解析后的 Bode 数据快照。 */
typedef struct
{
    uint16_t point_count;                   /**< 本帧有效点数。 */
    uint16_t mag2_hi[FPGA_LINK_MAX_POINTS]; /**< (I²+Q²)[63:48]，幅度平方高16位。 */
    int16_t phase[FPGA_LINK_MAX_POINTS];    /**< 相位，除以 32768 后乘 π。 */
    uint32_t frame_count;                   /**< 已接收的有效帧累计数。 */
    uint8_t valid;                          /**< 非零表示已有有效帧。 */
} fpga_link_bode_t;

/** FPGA 串口链路诊断量，供 STM32CubeIDE Expressions 查看。 */
typedef struct
{
    uint32_t rx_event_count;     /**< HAL Receive-to-IDLE 接收事件数。 */
    uint32_t frame_found_count;  /**< 找到 AA 55 同步头的次数。 */
    uint32_t frame_valid_count;  /**< 完整帧解析成功次数。 */
    uint32_t frame_error_count;  /**< 帧长度、点数或帧尾错误次数。 */
    uint32_t uart_error_count;   /**< USART2/DMA 错误回调次数。 */
    uint32_t dma_restart_count;  /**< DMA 接收成功启动次数。 */
    uint32_t dma_start_error_count; /**< DMA 接收启动失败次数。 */
    uint32_t command_tx_count;   /**< 成功发送给 FPGA 的单字节命令数。 */
    uint32_t command_tx_error_count; /**< FPGA 命令发送失败次数。 */
    uint16_t last_frame_bytes;   /**< 最近一次接收事件的字节数。 */
    uint16_t last_point_count;   /**< 最近一帧的有效点数。 */
    uint8_t last_tx_command;     /**< 最近一次成功发送的 FPGA 命令。 */
} fpga_link_diagnostics_t;

/** 公开诊断快照，仅由主循环更新。 */
extern volatile fpga_link_diagnostics_t fpga_link_diagnostics;

/**
 * @brief 初始化 FPGA 串口链路状态。
 * @param 无。
 * @return 无。
 */
void fpga_link_init(void);

#if defined(HAL_UART_MODULE_ENABLED)
/**
 * @brief 绑定 CubeMX 生成的 USART2 句柄。
 * @param huart 已完成初始化的 USART2 句柄。
 * @return 无。
 */
void fpga_link_bind_uart(UART_HandleTypeDef *huart);
#endif

/**
 * @brief 启动第一轮 Receive-to-IDLE DMA 接收。
 * @param 无。
 * @return 启动成功返回 1，未绑定 UART 或 HAL 启动失败返回 0。
 */
uint8_t fpga_link_start(void);

/**
 * @brief 在主循环处理接收事件、解析完整帧并重启 DMA。
 * @param 无。
 * @return 无。
 */
void fpga_link_process(void);

/**
 * @brief 复制最新 Bode 数据快照。
 * @param bode 输出快照。
 * @return 已有有效帧返回 1；空指针或尚无有效帧返回 0。
 */
uint8_t fpga_link_get_bode(fpga_link_bode_t *bode);

/**
 * @brief 通知 FPGA 将扫频步进 f_step 增加 1。
 * @param 无。
 * @return 成功发送 0x2B 返回 1；UART 未绑定或发送失败返回 0。
 */
uint8_t fpga_link_send_step_increase(void);

/**
 * @brief 通知 FPGA 将扫频步进 f_step 减少 1。
 * @param 无。
 * @return 成功发送 0x2D 返回 1；UART 未绑定或发送失败返回 0。
 */
uint8_t fpga_link_send_step_decrease(void);

#if defined(HAL_UART_MODULE_ENABLED)
/**
 * @brief USART2 错误回调分流入口。
 * @param huart 发生错误的 UART 句柄。
 * @return 无；中断上下文只置一个错误标志。
 */
void fpga_link_handle_error(UART_HandleTypeDef *huart);
#endif

#endif /* FPGA_LINK_H */
