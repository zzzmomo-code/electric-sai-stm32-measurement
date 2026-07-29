/**
 * @file fpga_link.h
 * @brief FPGA SPI3 测量数据链路公共接口。
 *
 * 模块用途：按 FPGA—STM32 SPI V1.0 协议执行 GET_STATUS、READ_FRAME、
 *          ACK_FRAME，校验完整帧并发布双缓冲测量快照。
 * GPIO 引脚映射：PC10/SPI3_SCK、PC11/SPI3_MISO、PC12/SPI3_MOSI、
 *          PA15/FPGA_CS_N、PD1/FPGA_DATA_READY。
 * 依赖的外设和 CubeIDE 配置：SPI3 Master Mode 0、20 MHz、8 bit；
 *          SPI3 RX/TX DMA Normal；PD1 EXTI1 上升沿。
 * 初始化方法：system_init() 调用 fpga_link_init() 并绑定 hspi3。
 * 调用方法：system_process() 持续调用 fpga_link_process()。
 */

#ifndef FPGA_LINK_H
#define FPGA_LINK_H

#include <stdint.h>

#include "fpga_protocol.h"

#if defined(HAL_SPI_MODULE_ENABLED)
#include "stm32h7xx_hal_spi.h"
#endif

/** SPI 链路主循环状态。 */
typedef enum
{
    FPGA_LINK_STATE_IDLE = 0,
    FPGA_LINK_STATE_WAIT_FRAME_DMA
} fpga_link_state_t;

/** 一份已经通过格式和 CRC 校验的完整测量快照。 */
typedef struct
{
    fpga_protocol_frame_header_t header; /**< 已解析的帧头参数。 */
    int16_t time_samples[FPGA_PROTOCOL_MAX_TIME_SAMPLES]; /**< 三周期时域。 */
    uint16_t spectrum[FPGA_PROTOCOL_SPECTRUM_COUNT]; /**< 0~1311 频谱。 */
    uint8_t valid; /**< 非零表示本快照可供换算和显示。 */
} fpga_measurement_snapshot_t;

/** SPI 链路诊断量，可直接加入 STM32CubeIDE Expressions。 */
typedef struct
{
    uint32_t data_ready_irq_count;
    uint32_t status_read_count;
    uint32_t status_valid_count;
    uint32_t status_error_count;
    uint32_t status_crc_error_count;
    uint32_t frame_read_count;
    uint32_t frame_dma_start_count;
    uint32_t frame_dma_complete_count;
    uint32_t frame_dma_error_count;
    uint32_t frame_dma_timeout_count;
    uint32_t frame_valid_count;
    uint32_t frame_format_error_count;
    uint32_t frame_crc_error_count;
    uint32_t frame_retry_count;
    uint32_t duplicate_frame_count;
    uint32_t result_invalid_count;
    uint32_t adc_overrange_count;
    uint32_t fpga_dropped_report_count;
    uint32_t ack_count;
    uint32_t ack_error_count;
    uint32_t published_snapshot_count;
    uint32_t last_frame_sequence;
    uint32_t last_frame_length;
    uint32_t last_hal_error;
    fpga_protocol_result_t last_protocol_result;
    fpga_link_state_t state;
} fpga_link_diagnostics_t;

/** PD1 EXTI 回调与主循环共享的数据就绪标志。 */
extern volatile uint8_t fpga_data_ready_flag;

/** SPI3 DMA 完成回调与主循环共享的完成标志。 */
extern volatile uint8_t fpga_spi_dma_complete_flag;

/** SPI3 错误回调与主循环共享的错误标志。 */
extern volatile uint8_t fpga_spi_dma_error_flag;

/** 公开诊断量，仅中断标志以外的字段由主循环修改。 */
extern volatile fpga_link_diagnostics_t fpga_link_diagnostics;

void fpga_link_init(void);

#if defined(HAL_SPI_MODULE_ENABLED)
/**
 * @brief 绑定 CubeMX 已初始化的 SPI3 句柄。
 * @param hspi SPI3 句柄。
 * @return 无。
 */
void fpga_link_bind_spi(SPI_HandleTypeDef *hspi);
#endif

/**
 * @brief 执行 DATA_READY 电平检查和 SPI 协议状态机。
 * @param 无。
 * @return 无。
 */
void fpga_link_process(void);

/**
 * @brief 获取当前活动测量快照的只读指针。
 * @param snapshot 输出指针地址。
 * @return 已有有效快照返回 1，否则返回 0。
 * @note 返回指针只保证到下一次 fpga_link_process() 发布新帧前稳定；
 *       调用者应立即完成读取或转换。
 */
uint8_t fpga_link_get_snapshot(
    const fpga_measurement_snapshot_t **snapshot);

#endif /* FPGA_LINK_H */
