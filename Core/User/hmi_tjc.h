/**
 * @file hmi_tjc.h
 * @brief 淘晶驰串口屏显示模块公共接口。
 *
 * 模块用途：将测量结果格式化为淘晶驰文本和频谱指令，并在 UART 可用时低频轮询发送基础页面。
 * GPIO 引脚映射：由 CubeMX 为后续选定 UART 分配，不在本模块硬编码引脚。
 * 依赖的外设和 CubeIDE 配置：运行发送依赖一个异步 UART；不使用 UART DMA 或 UART 全局中断。
 * 初始化方法：系统启动时调用 hmi_tjc_init()；CubeMX 启用 UART 后再绑定 UART 句柄。
 * 调用方法：主循环持续调用 hmi_tjc_process()；扩展页面通过详细指标和频谱构帧接口接入。
 */

#ifndef HMI_TJC_H
#define HMI_TJC_H

#include <stdint.h>

#include "measurement_fft.h"

/** HMI 刷新间隔，单位为毫秒。 */
#define HMI_TJC_REFRESH_MS 500u

/** UART 轮询发送超时，单位为毫秒。9600 波特率下可完整发送最大帧。 */
#define HMI_TJC_TX_TIMEOUT_MS 500u

/** UART 轮询发送帧缓冲区长度。 */
#define HMI_TJC_TX_BUFFER_SIZE 384u

/** 64 点频谱逐点 add 指令帧的最小建议缓冲区长度。 */
#define HMI_TJC_SPECTRUM_FRAME_SIZE 1152u

/** 未确认 USART HMI 曲线控件 ID 时使用的禁用值。 */
#define HMI_TJC_SPECTRUM_COMPONENT_DISABLED 255u

/** HMI 模块接口返回状态。 */
typedef enum
{
    HMI_TJC_STATUS_OK = 0,
    HMI_TJC_STATUS_INVALID_ARGUMENT,
    HMI_TJC_STATUS_BUFFER_TOO_SMALL,
    HMI_TJC_STATUS_UART_UNAVAILABLE,
    HMI_TJC_STATUS_HAL_ERROR
} hmi_tjc_status_t;

/** 串口屏轮询发送与帧构建诊断数据。 */
typedef struct
{
    uint32_t transmit_attempts;  /**< 已实际调用 HAL_UART_Transmit() 的次数。 */
    uint32_t transmit_successes; /**< UART 轮询发送成功次数。 */
    uint32_t transmit_failures;  /**< UART 轮询发送失败次数。 */
    uint32_t build_failures;     /**< 双通道十一控件命令帧构建失败次数。 */
    uint16_t last_frame_size;    /**< 最近一次尝试发送的帧长度。 */
    hmi_tjc_status_t last_status; /**< 最近一次帧构建或发送状态。 */
} hmi_tjc_diagnostics_t;

/**
 * @brief 初始化 HMI 模块状态。
 * @param 无。
 * @return 无。
 * @note UART 尚未通过 CubeMX 启用时，本模块保持空闲且不会访问串口硬件。
 */
void hmi_tjc_init(void);

/**
 * @brief 构建一帧包含双通道十一条淘晶驰文本指令的 UART 数据。
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
 * @brief 构建 DC、RMS 和 THD 三个扩展文本控件的指令帧。
 * @param result 待显示的测量结果快照。
 * @param frame 用于接收 UART 数据的缓冲区。
 * @param frame_capacity 缓冲区容量，单位为字节。
 * @param frame_size 用于接收实际帧长度的指针。
 * @return 帧构建结果状态。
 * @note HMI 页面增加 t_dc、t_rms、t_thd 后再由主循环发送此帧。
 */
hmi_tjc_status_t hmi_tjc_build_detail_frame(
    const measurement_result_t *result,
    uint8_t *frame,
    uint16_t frame_capacity,
    uint16_t *frame_size);

/**
 * @brief 构建清空并重绘 64 点频谱曲线的淘晶驰指令帧。
 * @param spectrum 待显示的 0 至 20 kHz 相对 dB 频谱。
 * @param component_id USART HMI 中曲线控件的实际数字 ID。
 * @param channel 曲线通道号，范围为 0 至 3。
 * @param frame 用于接收 UART 数据的缓冲区。
 * @param frame_capacity 缓冲区容量，建议至少 HMI_TJC_SPECTRUM_FRAME_SIZE。
 * @param frame_size 用于接收实际帧长度的指针。
 * @return 帧构建结果状态。
 * @note 每点由 -80 至 0 dB 映射为 0 至 255；未确认控件 ID 时不要发送。
 */
hmi_tjc_status_t hmi_tjc_build_spectrum_frame(
    const measurement_fft_spectrum_t *spectrum,
    uint8_t component_id,
    uint8_t channel,
    uint8_t *frame,
    uint16_t frame_capacity,
    uint16_t *frame_size);

/**
 * @brief 处理 HMI 上电清理和周期刷新。
 * @param 无。
 * @return 无。
 * @note 必须由主循环调用；轮询发送会短暂阻塞，刷新率固定为约 2 Hz。
 */
void hmi_tjc_process(void);

/**
 * @brief 读取串口屏模块累计诊断数据。
 * @param diagnostics 用于接收诊断快照的指针。
 * @return 指针有效时返回 1，否则返回 0。
 * @note 只复制主循环维护的状态，不访问 UART 硬件。
 */
uint8_t hmi_tjc_get_diagnostics(hmi_tjc_diagnostics_t *diagnostics);

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
