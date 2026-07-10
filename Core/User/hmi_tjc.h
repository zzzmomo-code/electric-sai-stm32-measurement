/**
 * @file hmi_tjc.h
 * @brief 淘晶驰串口屏显示模块公共接口。
 *
 * 模块用途：将测量结果格式化为淘晶驰文本指令，并在 UART 可用时低频轮询发送。
 * GPIO 引脚映射：由 CubeMX 为后续选定 UART 分配，不在本模块硬编码引脚。
 * 依赖的外设和 CubeIDE 配置：运行发送依赖一个异步 UART；不使用 UART DMA 或 UART 全局中断。
 * 初始化方法：系统启动时调用 hmi_tjc_init()；CubeMX 启用 UART 后再绑定 UART 句柄。
 * 调用方法：主循环持续调用 hmi_tjc_process()。
 */

#ifndef HMI_TJC_H
#define HMI_TJC_H

#include <stdint.h>

#include "measurement_result.h"

/** HMI 刷新间隔，单位为毫秒。 */
#define HMI_TJC_REFRESH_MS 250u

/** UART 轮询发送超时，单位为毫秒。9600 波特率下可完整发送最大帧。 */
#define HMI_TJC_TX_TIMEOUT_MS 250u

/** UART 轮询发送帧缓冲区长度。 */
#define HMI_TJC_TX_BUFFER_SIZE 192u

/** HMI 模块接口返回状态。 */
typedef enum
{
    HMI_TJC_STATUS_OK = 0,
    HMI_TJC_STATUS_INVALID_ARGUMENT,
    HMI_TJC_STATUS_BUFFER_TOO_SMALL,
    HMI_TJC_STATUS_UART_UNAVAILABLE,
    HMI_TJC_STATUS_HAL_ERROR
} hmi_tjc_status_t;

/**
 * @brief 初始化 HMI 模块状态。
 * @param 无。
 * @return 无。
 * @note UART 尚未通过 CubeMX 启用时，本模块保持空闲且不会访问串口硬件。
 */
void hmi_tjc_init(void);

/**
 * @brief 构建一帧包含五条淘晶驰文本指令的 UART 数据。
 * @param result 待显示的测量结果快照。
 * @param frame 用于接收二进制 UART 数据的缓冲区。
 * @param frame_capacity 缓冲区容量，单位为字节。
 * @param frame_size 用于接收实际帧长度的指针。
 * @return 帧构建结果状态。
 * @note 每条文本命令均以三个 0xff 结束，调用者无需再追加结束符。
 */
hmi_tjc_status_t hmi_tjc_build_frame(const measurement_result_t *result,
                                     uint8_t *frame,
                                     uint16_t frame_capacity,
                                     uint16_t *frame_size);

/**
 * @brief 处理 HMI 上电清理和周期刷新。
 * @param 无。
 * @return 无。
 * @note 必须由主循环调用；轮询发送会短暂阻塞，刷新率固定为约 4 Hz。
 */
void hmi_tjc_process(void);

#if defined(HAL_UART_MODULE_ENABLED)
#include "stm32h7xx_hal_uart.h"

/**
 * @brief 绑定 CubeMX 生成的 UART 句柄。
 * @param huart 已配置为 9600 8N1 的 UART 句柄。
 * @return 无。
 * @note 只能在 UART 初始化完成后调用；不要求 DMA 或 UART 全局中断，重新绑定会重新发送串口屏清理帧。
 */
void hmi_tjc_bind_uart(UART_HandleTypeDef *huart);
#endif

#endif /* HMI_TJC_H */
