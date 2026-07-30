/**
 * @file fpga_link.c
 * @brief FPGA SPI3 测量数据链路实现。
 *
 * 模块用途：在同一个 CS 事务中发送命令并立即读取响应，使用 SPI3
 *          双向 DMA 接收最大 10254 字节测量帧，校验后发布双缓冲快照。
 * GPIO 引脚映射：PC10/SCK、PC11/MISO、PC12/MOSI、PA15/CS_N、PD1/DATA_READY。
 * 依赖的外设和 CubeIDE 配置：SPI3 20 MHz Mode 0；RX DMA Memory Increment
 *          Enable；TX DMA Memory Increment Disable；EXTI1 Rising。
 * 初始化方法：system_init() 调用 fpga_link_init() 和 fpga_link_bind_spi()。
 * 调用方法：主循环持续调用 fpga_link_process()。
 */

#include "system.h"

#include <string.h>

#define FPGA_LINK_SPI_TIMEOUT_MS        20u
#define FPGA_LINK_DMA_TIMEOUT_MS        20u
#define FPGA_LINK_RETRY_DELAY_MS        2u
#define FPGA_LINK_MAX_READ_RETRIES      3u
#define FPGA_LINK_ACK_READY_LOW_TIMEOUT_MS 10u
#define FPGA_LINK_ACK_READY_LOW_FAST_POLLS 512u
#define FPGA_LINK_DMA_BUFFER_BYTES \
    ((FPGA_PROTOCOL_MAX_FRAME_BYTES + 31u) & ~31u)

/** SPI3 DMA 接收区，32 字节对齐且长度为缓存行整数倍。 */
static uint8_t fpga_link_frame_buffer[FPGA_LINK_DMA_BUFFER_BYTES]
    __attribute__((aligned(32)));

/** TX DMA 关闭地址递增后重复读取的单个 dummy 字节所在缓存行。 */
static uint8_t fpga_link_dummy_tx_cache_line[32]
    __attribute__((aligned(32)));

/** 两份快照交替写入，校验成功后只切换活动索引。 */
static fpga_measurement_snapshot_t fpga_link_snapshots[2];
static uint8_t fpga_link_active_snapshot_index;

/** GET_STATUS 使用的固定工作区，仅在主循环访问。 */
static uint8_t fpga_link_status_dummy[FPGA_PROTOCOL_STATUS_BYTES];
static uint8_t fpga_link_status_response[FPGA_PROTOCOL_STATUS_BYTES];

/** 当前稳定 FPGA 状态和正在读取的帧信息。 */
static fpga_protocol_status_t fpga_link_current_status;
static SPI_HandleTypeDef *fpga_link_spi;
static uint32_t fpga_link_dma_deadline_ms;
static uint32_t fpga_link_ack_deadline_ms;
static uint32_t fpga_link_next_attempt_ms;
static uint8_t fpga_link_read_retry;
static uint8_t fpga_link_cs_active;

volatile uint8_t fpga_data_ready_flag;
volatile uint8_t fpga_spi_dma_complete_flag;
volatile uint8_t fpga_spi_dma_error_flag;
volatile fpga_link_diagnostics_t fpga_link_diagnostics;

/*
 * 主循环状态机只有两个长期状态：
 *
 * IDLE
 *   ├─ PD1 为高或收到上升沿 -> 同一 CS 下读取 GET_STATUS
 *   ├─ 状态有效且 FRAME_READY=1 -> 启动 READ_FRAME DMA
 *   └─ 状态无效 -> 延时 2 ms 后重试状态
 *
 * WAIT_FRAME_DMA
 *   ├─ DMA 完成 -> 拉高 CS、校验完整帧、发布快照、发送 ACK
 *   ├─ 帧格式/CRC 错 -> 不发 ACK，同一稳定 FPGA 帧最多重读 3 次
 *   └─ DMA 错误/超时 -> 中止 SPI、拉高 CS、回到 IDLE
 *
 * WAIT_DATA_READY_LOW
 *   ├─ 实际读到 PD1 为低 -> 确认 ACK 已被 FPGA 接受，回到 IDLE
 *   └─ 10 ms 内仍为高 -> 记录超时并回到 IDLE，下一轮重新 GET_STATUS
 *
 * 中断回调只产生 ready/complete/error 三个事件；全部 HAL 调用、CRC 和数据复制
 * 都在 fpga_link_process() 中完成，因此不会在中断里阻塞约 4 ms 的大帧传输。
 */

/**
 * @brief 判断 D-Cache 是否启用。
 * @param 无。
 * @return 启用返回 1，否则返回 0。
 */
static uint8_t fpga_link_dcache_enabled(void)
{
    return ((SCB->CCR & SCB_CCR_DC_Msk) != 0u) ? 1u : 0u;
}

/**
 * @brief DMA 前清理并失效接收区，同时清理 dummy TX 缓存行。
 * @param 无。
 * @return 无。
 */
static void fpga_link_prepare_dma_cache(void)
{
    if (fpga_link_dcache_enabled() == 0u)
    {
        return;
    }

    SCB_CleanInvalidateDCache_by_Addr(
        (uint32_t *)fpga_link_frame_buffer,
        (int32_t)sizeof(fpga_link_frame_buffer));
    SCB_CleanDCache_by_Addr(
        (uint32_t *)fpga_link_dummy_tx_cache_line,
        (int32_t)sizeof(fpga_link_dummy_tx_cache_line));
}

/**
 * @brief DMA 完成后失效 CPU 将读取的缓存行。
 * @param length DMA 有效接收长度。
 * @return 无。
 */
static void fpga_link_invalidate_frame_cache(uint32_t length)
{
    uint32_t rounded_length;

    if (fpga_link_dcache_enabled() == 0u)
    {
        return;
    }

    rounded_length = (length + 31u) & ~31u;
    SCB_InvalidateDCache_by_Addr(
        (uint32_t *)fpga_link_frame_buffer,
        (int32_t)rounded_length);
}

/**
 * @brief 读取 FPGA DATA_READY 当前实际电平。
 * @param 无。
 * @return 高电平返回 1，低电平返回 0。
 */
static uint8_t fpga_link_data_ready_is_high(void)
{
    return (HAL_GPIO_ReadPin(
                FPGA_DATA_READY_GPIO_Port,
                FPGA_DATA_READY_Pin) == GPIO_PIN_SET) ? 1u : 0u;
}

/**
 * @brief ACK 发送完成后短时间连续采样 DATA_READY，捕获至少 1 us 的低脉冲。
 * @param 无。
 * @return 观察到低电平返回 1；快速采样窗口内仍为高返回 0。
 * @note STM32-R2 要求依据实际 GPIO 电平确认 ACK，不能假定固定两个 FPGA 时钟后已拉低。
 */
static uint8_t fpga_link_observe_ready_low_fast(void)
{
    uint32_t poll;

    for (poll = 0u;
         poll < FPGA_LINK_ACK_READY_LOW_FAST_POLLS;
         poll++)
    {
        if (fpga_link_data_ready_is_high() == 0u)
        {
            return 1u;
        }
    }

    return 0u;
}

/**
 * @brief 原子拉低软件 CS。
 * @param 无。
 * @return 无。
 */
static void fpga_link_cs_low(void)
{
    HAL_GPIO_WritePin(
        FPGA_CS_N_GPIO_Port, FPGA_CS_N_Pin, GPIO_PIN_RESET);
    fpga_link_cs_active = 1u;
}

/**
 * @brief 拉高软件 CS 并结束当前事务。
 * @param 无。
 * @return 无。
 */
static void fpga_link_cs_high(void)
{
    HAL_GPIO_WritePin(
        FPGA_CS_N_GPIO_Port, FPGA_CS_N_Pin, GPIO_PIN_SET);
    fpga_link_cs_active = 0u;
}

/**
 * @brief 清零 DMA 回调标志。
 * @param 无。
 * @return 无。
 */
static void fpga_link_clear_dma_flags(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    fpga_spi_dma_complete_flag = 0u;
    fpga_spi_dma_error_flag = 0u;
    if (primask == 0u)
    {
        __enable_irq();
    }
}

/**
 * @brief 从中断共享变量领取事件。
 * @param ready_flag 输出 DATA_READY 上升沿标志。
 * @param complete_flag 输出 DMA 完成标志。
 * @param error_flag 输出 SPI/DMA 错误标志。
 * @return 无。
 */
static void fpga_link_claim_events(
    uint8_t *ready_flag,
    uint8_t *complete_flag,
    uint8_t *error_flag)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    *ready_flag = fpga_data_ready_flag;
    fpga_data_ready_flag = 0u;
    *complete_flag = fpga_spi_dma_complete_flag;
    fpga_spi_dma_complete_flag = 0u;
    *error_flag = fpga_spi_dma_error_flag;
    fpga_spi_dma_error_flag = 0u;
    if (primask == 0u)
    {
        __enable_irq();
    }
}

/**
 * @brief 在一次 CS 事务中读取 GET_STATUS 的 16 字节响应。
 * @param status 输出已校验状态。
 * @return 协议解析结果或 HAL 对应的状态错误。
 */
static fpga_protocol_result_t fpga_link_read_status(
    fpga_protocol_status_t *status)
{
    uint8_t command[2] = {
        FPGA_PROTOCOL_COMMAND_PREFIX,
        FPGA_PROTOCOL_COMMAND_GET_STATUS
    };
    HAL_StatusTypeDef hal_status;

    fpga_link_diagnostics.status_read_count++;
    fpga_link_cs_low();
    hal_status = HAL_SPI_Transmit(
        fpga_link_spi, command, sizeof(command),
        FPGA_LINK_SPI_TIMEOUT_MS);
    if (hal_status == HAL_OK)
    {
        hal_status = HAL_SPI_TransmitReceive(
            fpga_link_spi,
            fpga_link_status_dummy,
            fpga_link_status_response,
            FPGA_PROTOCOL_STATUS_BYTES,
            FPGA_LINK_SPI_TIMEOUT_MS);
    }
    fpga_link_cs_high();

    if (hal_status != HAL_OK)
    {
        fpga_link_diagnostics.last_hal_error =
            HAL_SPI_GetError(fpga_link_spi);
        return FPGA_PROTOCOL_ERROR_STATE;
    }

    return fpga_protocol_parse_status(
        fpga_link_status_response,
        FPGA_PROTOCOL_STATUS_BYTES,
        status);
}

/**
 * @brief 启动同一 CS 内的 READ_FRAME DMA 响应读取。
 * @param now 当前毫秒计数。
 * @return 启动成功返回 1，否则返回 0。
 */
static uint8_t fpga_link_start_frame_dma(uint32_t now)
{
    uint8_t command[2] = {
        FPGA_PROTOCOL_COMMAND_PREFIX,
        FPGA_PROTOCOL_COMMAND_READ_FRAME
    };
    HAL_StatusTypeDef hal_status;

    if ((fpga_link_current_status.frame_length == 0u)
        || (fpga_link_current_status.frame_length
            > FPGA_PROTOCOL_MAX_FRAME_BYTES))
    {
        return 0u;
    }

    fpga_link_clear_dma_flags();
    fpga_link_prepare_dma_cache();
    fpga_link_cs_low();
    hal_status = HAL_SPI_Transmit(
        fpga_link_spi, command, sizeof(command),
        FPGA_LINK_SPI_TIMEOUT_MS);
    if (hal_status == HAL_OK)
    {
        hal_status = HAL_SPI_TransmitReceive_DMA(
            fpga_link_spi,
            fpga_link_dummy_tx_cache_line,
            fpga_link_frame_buffer,
            (uint16_t)fpga_link_current_status.frame_length);
    }

    if (hal_status != HAL_OK)
    {
        fpga_link_cs_high();
        fpga_link_diagnostics.frame_dma_error_count++;
        fpga_link_diagnostics.last_hal_error =
            HAL_SPI_GetError(fpga_link_spi);
        return 0u;
    }

    fpga_link_diagnostics.frame_read_count++;
    fpga_link_diagnostics.frame_dma_start_count++;
    fpga_link_diagnostics.state = FPGA_LINK_STATE_WAIT_FRAME_DMA;
    fpga_link_dma_deadline_ms = now + FPGA_LINK_DMA_TIMEOUT_MS;
    return 1u;
}

/**
 * @brief 发送 ACK_FRAME，CRC 在线上低字节先传。
 * @param frame_sequence 已校验的帧序号。
 * @return HAL 发送成功返回 1，否则返回 0。
 */
static uint8_t fpga_link_send_ack(uint32_t frame_sequence)
{
    uint8_t frame[8];
    uint16_t crc;
    HAL_StatusTypeDef hal_status;

    frame[0] = FPGA_PROTOCOL_COMMAND_PREFIX;
    frame[1] = FPGA_PROTOCOL_COMMAND_ACK_FRAME;
    fpga_protocol_write_u32_le(&frame[2], frame_sequence);
    crc = fpga_protocol_crc16(frame, 6u);
    frame[6] = (uint8_t)crc;
    frame[7] = (uint8_t)(crc >> 8);

    fpga_link_cs_low();
    hal_status = HAL_SPI_Transmit(
        fpga_link_spi, frame, sizeof(frame),
        FPGA_LINK_SPI_TIMEOUT_MS);
    fpga_link_cs_high();

    if (hal_status != HAL_OK)
    {
        fpga_link_diagnostics.ack_error_count++;
        fpga_link_diagnostics.last_hal_error =
            HAL_SPI_GetError(fpga_link_spi);
        return 0u;
    }

    fpga_link_diagnostics.ack_count++;
    return 1u;
}

/**
 * @brief 将已校验的原始载荷复制到非活动快照并原子发布。
 * @param header 已解析帧头。
 * @return 无。
 */
static void fpga_link_publish_snapshot(
    const fpga_protocol_frame_header_t *header)
{
    fpga_measurement_snapshot_t *snapshot;
    uint32_t time_offset = FPGA_PROTOCOL_HEADER_BYTES;
    uint32_t spectrum_offset =
        time_offset + ((uint32_t)header->time_count * 2u);
    uint16_t index;
    uint8_t target_index =
        (uint8_t)(fpga_link_active_snapshot_index ^ 1u);

    /*
     * 永远写“非活动”快照：活动快照继续供 measurement_conversion 读取。
     * 原始帧已通过 CRC，所以这里可以按协议偏移把载荷安全复制出来。
     */
    snapshot = &fpga_link_snapshots[target_index];
    memset((void *)snapshot, 0, sizeof(*snapshot));
    snapshot->header = *header;

    for (index = 0u; index < header->time_count; index++)
    {
        snapshot->time_samples[index] = fpga_protocol_read_i16_le(
            &fpga_link_frame_buffer[
                time_offset + ((uint32_t)index * 2u)]);
    }
    for (index = 0u; index < header->spectrum_count; index++)
    {
        snapshot->spectrum[index] = fpga_protocol_read_u16_le(
            &fpga_link_frame_buffer[
                spectrum_offset + ((uint32_t)index * 2u)]);
    }

    snapshot->valid = 1u;
    /*
     * DMB 保证上面的帧头、时域和频谱写入先对 CPU 可见，再切换活动索引。
     * 读取者因此只可能看到完整的旧快照或完整的新快照，不会看到半帧。
     */
    __DMB();
    fpga_link_active_snapshot_index = target_index;
    fpga_link_diagnostics.published_snapshot_count++;
}

/**
 * @brief 统计具体协议错误类型。
 * @param result 协议解析结果。
 * @return 无。
 */
static void fpga_link_record_frame_error(
    fpga_protocol_result_t result)
{
    fpga_link_diagnostics.last_protocol_result = result;
    if (result == FPGA_PROTOCOL_ERROR_CRC)
    {
        fpga_link_diagnostics.frame_crc_error_count++;
    }
    else
    {
        fpga_link_diagnostics.frame_format_error_count++;
    }
}

/**
 * @brief 校验 DMA 帧、按有效位发布并发送 ACK。
 * @param now 当前毫秒计数。
 * @return 无。
 */
static void fpga_link_finish_frame(uint32_t now)
{
    fpga_protocol_frame_header_t header;
    fpga_protocol_result_t result;
    uint8_t is_duplicate;

    fpga_link_invalidate_frame_cache(
        fpga_link_current_status.frame_length);
    result = fpga_protocol_parse_frame(
        fpga_link_frame_buffer,
        fpga_link_current_status.frame_length,
        fpga_link_current_status.frame_seq,
        &header);
    fpga_link_diagnostics.last_protocol_result = result;

    if (result != FPGA_PROTOCOL_OK)
    {
        fpga_link_record_frame_error(result);
        if (fpga_link_read_retry < FPGA_LINK_MAX_READ_RETRIES)
        {
            fpga_link_read_retry++;
            fpga_link_diagnostics.frame_retry_count++;
            if (fpga_link_start_frame_dma(now) != 0u)
            {
                return;
            }
        }

        fpga_link_diagnostics.state = FPGA_LINK_STATE_IDLE;
        fpga_link_next_attempt_ms = now + FPGA_LINK_RETRY_DELAY_MS;
        return;
    }

    fpga_link_diagnostics.frame_valid_count++;
    fpga_link_diagnostics.last_frame_sequence = header.frame_seq;
    fpga_link_diagnostics.last_frame_length = header.frame_length;
    if ((fpga_link_current_status.state
         & FPGA_PROTOCOL_STATUS_ADC_OTR) != 0u)
    {
        fpga_link_diagnostics.adc_overrange_count++;
    }
    if ((fpga_link_current_status.state
         & FPGA_PROTOCOL_STATUS_FRAME_DROPPED) != 0u)
    {
        fpga_link_diagnostics.fpga_dropped_report_count++;
    }

    is_duplicate =
        (fpga_link_snapshots[fpga_link_active_snapshot_index].valid != 0u)
        && (fpga_link_snapshots[
                fpga_link_active_snapshot_index].header.frame_seq
            == header.frame_seq);
    if (((fpga_link_current_status.state
          & FPGA_PROTOCOL_STATUS_RESULT_INVALID) != 0u)
        || ((header.flags
             & FPGA_PROTOCOL_HEADER_MEASUREMENT_VALID) == 0u))
    {
        fpga_link_diagnostics.result_invalid_count++;
    }
    else if (is_duplicate != 0u)
    {
        fpga_link_diagnostics.duplicate_frame_count++;
    }
    else
    {
        fpga_link_publish_snapshot(&header);
    }

    if (fpga_link_send_ack(header.frame_seq) == 0u)
    {
        fpga_link_diagnostics.state = FPGA_LINK_STATE_IDLE;
        fpga_link_next_attempt_ms = now + FPGA_LINK_RETRY_DELAY_MS;
        return;
    }

    /*
     * 正确 ACK 后不立即读取下一帧。先连续采样一次，以可靠捕获 FPGA 保证
     * 至少 1 us 的 DATA_READY 低电平；若此窗口仍未观察到低电平，则转入
     * 非阻塞等待状态，超时后重新 GET_STATUS，而不是盲目认定 ACK 成功。
     */
    if (fpga_link_observe_ready_low_fast() != 0u)
    {
        fpga_link_diagnostics.ack_ready_low_count++;
        fpga_link_diagnostics.state = FPGA_LINK_STATE_IDLE;
        fpga_link_next_attempt_ms = now;
        return;
    }

    fpga_link_diagnostics.state =
        FPGA_LINK_STATE_WAIT_DATA_READY_LOW;
    fpga_link_ack_deadline_ms =
        now + FPGA_LINK_ACK_READY_LOW_TIMEOUT_MS;
}

/**
 * @brief 初始化软件缓冲区、诊断量和链路状态。
 * @param 无。
 * @return 无。
 * @note 必须在 MX_GPIO_Init() 之后调用，因为函数会把 PA15/CS 拉高。
 */
void fpga_link_init(void)
{
    memset((void *)fpga_link_frame_buffer, 0,
           sizeof(fpga_link_frame_buffer));
    memset((void *)fpga_link_dummy_tx_cache_line, 0,
           sizeof(fpga_link_dummy_tx_cache_line));
    memset((void *)fpga_link_status_dummy, 0,
           sizeof(fpga_link_status_dummy));
    memset((void *)fpga_link_status_response, 0,
           sizeof(fpga_link_status_response));
    memset((void *)fpga_link_snapshots, 0,
           sizeof(fpga_link_snapshots));
    memset((void *)&fpga_link_current_status, 0,
           sizeof(fpga_link_current_status));
    memset((void *)&fpga_link_diagnostics, 0,
           sizeof(fpga_link_diagnostics));

    fpga_link_spi = NULL;
    fpga_link_active_snapshot_index = 0u;
    fpga_link_dma_deadline_ms = 0u;
    fpga_link_ack_deadline_ms = 0u;
    fpga_link_next_attempt_ms = 0u;
    fpga_link_read_retry = 0u;
    fpga_link_cs_active = 0u;
    fpga_data_ready_flag = 0u;
    fpga_spi_dma_complete_flag = 0u;
    fpga_spi_dma_error_flag = 0u;
    fpga_link_diagnostics.state = FPGA_LINK_STATE_IDLE;
    fpga_link_cs_high();
}

#if defined(HAL_SPI_MODULE_ENABLED)
/**
 * @brief 绑定 CubeMX 已初始化的 SPI3 句柄。
 * @param hspi SPI3 句柄地址。
 * @return 无。
 */
void fpga_link_bind_spi(SPI_HandleTypeDef *hspi)
{
    fpga_link_spi = hspi;
}
#endif

/**
 * @brief 推进一次 FPGA SPI 非阻塞状态机。
 * @param 无。
 * @return 无。
 * @note 函数需要在 while(1) 中高频调用；只有 2 字节命令和 16 字节状态使用
 *       带超时的短轮询，最长测量帧使用 DMA。
 */
void fpga_link_process(void)
{
    fpga_protocol_result_t status_result;
    uint32_t now;
    uint8_t ready_irq;
    uint8_t dma_complete;
    uint8_t dma_error;
    uint8_t data_ready_level;

    if (fpga_link_spi == NULL)
    {
        return;
    }

    now = HAL_GetTick();
    fpga_link_claim_events(
        &ready_irq, &dma_complete, &dma_error);
    if (ready_irq != 0u)
    {
        fpga_link_diagnostics.data_ready_irq_count++;
    }

    if (fpga_link_diagnostics.state
        == FPGA_LINK_STATE_WAIT_FRAME_DMA)
    {
        if (dma_error != 0u)
        {
            (void)HAL_SPI_Abort(fpga_link_spi);
            if (fpga_link_cs_active != 0u)
            {
                fpga_link_cs_high();
            }
            fpga_link_diagnostics.frame_dma_error_count++;
            fpga_link_diagnostics.last_hal_error =
                HAL_SPI_GetError(fpga_link_spi);
            fpga_link_diagnostics.state = FPGA_LINK_STATE_IDLE;
            fpga_link_next_attempt_ms =
                now + FPGA_LINK_RETRY_DELAY_MS;
            return;
        }
        if (dma_complete != 0u)
        {
            if (fpga_link_cs_active != 0u)
            {
                fpga_link_cs_high();
            }
            fpga_link_diagnostics.frame_dma_complete_count++;
            fpga_link_finish_frame(now);
            return;
        }
        if ((int32_t)(now - fpga_link_dma_deadline_ms) >= 0)
        {
            (void)HAL_SPI_Abort(fpga_link_spi);
            if (fpga_link_cs_active != 0u)
            {
                fpga_link_cs_high();
            }
            fpga_link_diagnostics.frame_dma_timeout_count++;
            fpga_link_diagnostics.state = FPGA_LINK_STATE_IDLE;
            fpga_link_next_attempt_ms =
                now + FPGA_LINK_RETRY_DELAY_MS;
        }
        return;
    }

    if (fpga_link_diagnostics.state
        == FPGA_LINK_STATE_WAIT_DATA_READY_LOW)
    {
        if (fpga_link_data_ready_is_high() == 0u)
        {
            fpga_link_diagnostics.ack_ready_low_count++;
            fpga_link_diagnostics.state = FPGA_LINK_STATE_IDLE;
            fpga_link_next_attempt_ms = now;
            return;
        }
        if ((int32_t)(now - fpga_link_ack_deadline_ms) >= 0)
        {
            fpga_link_diagnostics.ack_ready_low_timeout_count++;
            fpga_link_diagnostics.state = FPGA_LINK_STATE_IDLE;
            fpga_link_next_attempt_ms = now;
        }
        return;
    }

    data_ready_level = fpga_link_data_ready_is_high();
    if ((ready_irq == 0u) && (data_ready_level == 0u))
    {
        return;
    }
    if ((int32_t)(now - fpga_link_next_attempt_ms) < 0)
    {
        return;
    }

    status_result = fpga_link_read_status(&fpga_link_current_status);
    fpga_link_diagnostics.last_protocol_result = status_result;
    if (status_result != FPGA_PROTOCOL_OK)
    {
        fpga_link_diagnostics.status_error_count++;
        if (status_result == FPGA_PROTOCOL_ERROR_CRC)
        {
            fpga_link_diagnostics.status_crc_error_count++;
        }
        fpga_link_next_attempt_ms = now + FPGA_LINK_RETRY_DELAY_MS;
        return;
    }
    fpga_link_diagnostics.status_valid_count++;
    if ((fpga_link_current_status.state
         & FPGA_PROTOCOL_STATUS_FRAME_READY) == 0u)
    {
        fpga_link_diagnostics.status_not_ready_count++;
        fpga_link_next_attempt_ms = now + FPGA_LINK_RETRY_DELAY_MS;
        return;
    }

    fpga_link_read_retry = 0u;
    if (fpga_link_start_frame_dma(now) == 0u)
    {
        fpga_link_diagnostics.state = FPGA_LINK_STATE_IDLE;
        fpga_link_next_attempt_ms = now + FPGA_LINK_RETRY_DELAY_MS;
    }
}

/**
 * @brief 取得当前已经完整发布的测量快照。
 * @param snapshot 输出当前活动快照的只读地址。
 * @return 快照有效返回 1，尚未收到有效帧或参数为空返回 0。
 */
uint8_t fpga_link_get_snapshot(
    const fpga_measurement_snapshot_t **snapshot)
{
    const fpga_measurement_snapshot_t *active;

    if (snapshot == NULL)
    {
        return 0u;
    }

    active = &fpga_link_snapshots[fpga_link_active_snapshot_index];
    *snapshot = active;
    return active->valid;
}

/**
 * @brief HAL GPIO 外部中断回调；只记录 DATA_READY 上升沿事件。
 * @param gpio_pin 产生中断的 GPIO 引脚掩码。
 * @return 无。
 */
void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
    if (gpio_pin == FPGA_DATA_READY_Pin)
    {
        fpga_data_ready_flag = 1u;
    }
}

/**
 * @brief HAL SPI 双向 DMA 完成回调；只设置完成标志。
 * @param hspi 触发回调的 SPI 句柄。
 * @return 无。
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == fpga_link_spi)
    {
        fpga_spi_dma_complete_flag = 1u;
    }
}

/**
 * @brief HAL SPI 错误回调；只设置错误标志。
 * @param hspi 触发回调的 SPI 句柄。
 * @return 无。
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == fpga_link_spi)
    {
        fpga_spi_dma_error_flag = 1u;
    }
}
