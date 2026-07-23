/**
 * @file ads8688.c
 * @brief ADS8688 寄存器传输与器件初始化实现。
 *
 * 模块用途：通过 SPI3 配置 ADS8688，并使用循环 DMA 连续采集、切换模式及故障恢复。
 * GPIO 引脚映射：PA15/FSYNC、PC10/SCLK、PC11/SDO、PC12/SDI、PD0/DAISY、PD1/RST。
 * 依赖的外设和 CubeIDE 配置：SPI3 使用 32 位数据帧、第二边沿采样、硬件低有效 NSS、
 * 1 周期数据间空闲；RX/TX DMA 均为循环模式，RX 地址递增、TX 地址不递增。
 * 初始化方法：CubeMX 完成 GPIO 与 SPI3 初始化后调用 ads8688_init()，该函数不启动 DMA。
 * 调用方法：选择 ADS 后调用 ads8688_start()，主循环持续调用 ads8688_process()。
 */

#include "system.h"

#define ADS8688_COMMAND_NO_OP             0x0000u
#define ADS8688_COMMAND_AUTO_RST          0xa000u
#define ADS8688_REGISTER_AUTO_SEQUENCE    0x01u
#define ADS8688_REGISTER_FEATURE_SELECT   0x03u
#define ADS8688_REGISTER_RANGE_CH0        0x05u
#define ADS8688_REGISTER_RANGE_CH7        0x0cu
/* 默认仅扫描 AIN0 与 AIN1，为双通道频率和相位测量保留采样率。 */
#define ADS8688_DEFAULT_CHANNEL_MASK      0x03u
#define ADS8688_INITIALIZATION_RETRIES    3u
#define ADS8688_SPI_TIMEOUT_MS            10u
#define ADS8688_DMA_WORD_COUNT            1024u
#define ADS8688_DMA_HALF_WORD_COUNT       (ADS8688_DMA_WORD_COUNT / 2u)
#define ADS8688_SPI_FRAME_CYCLES           33.0f

/** 复位低电平期间执行的有界空操作次数，保证持续时间安全超过 400 ns。 */
#define ADS8688_RESET_HOLD_NOP_COUNT      256u

/** 模块是否已经完成器件配置及读回校验。 */
static uint8_t ads8688_initialized;
/** 非零表示 SPI3 循环 DMA 当前正在采样。 */
static uint8_t ads8688_running;

/** 当前采集模式，初始化成功后默认为自动扫描。 */
static ads8688_mode_t ads8688_mode;

/** 自动扫描通道掩码，初始化成功后默认启用 AIN0 与 AIN1。 */
static uint8_t ads8688_channel_mask;

/** 当前待处理通道号，初始化成功后从通道零开始。 */
static uint8_t ads8688_current_channel;

/** 八个输入通道当前采用的量程编码。 */
static ads8688_range_t ads8688_channel_ranges[ADS8688_CHANNEL_COUNT];

/** 后续采集使用的累计采样序号。 */
static uint32_t ads8688_sample_index;

/** ADS8688 初始化和运行故障的累计诊断信息。 */
static ads8688_diagnostics_t ads8688_diagnostics;

/** DMA 接收缓冲区，32 字节对齐以满足 STM32H7 数据缓存行边界要求。 */
static uint32_t ads8688_dma_rx[ADS8688_DMA_WORD_COUNT]
    __attribute__((section(".ads8688_dma"), aligned(32)));

/** DMA 重复发送的 32 位 NO_OP 帧，TX DMA 禁止存储器地址递增。 */
static uint32_t ads8688_dma_tx_word
    __attribute__((section(".ads8688_dma"), aligned(32)));

/** DMA 半缓冲区完成标志，由中断置位并由主循环处理及清除。 */
volatile uint8_t ads8688_dma_half_flag;

/** DMA 全缓冲区完成标志，由中断置位并由主循环处理及清除。 */
volatile uint8_t ads8688_dma_full_flag;

/** SPI/DMA 错误标志，由中断置位并由主循环处理及清除。 */
volatile uint8_t ads8688_error_flag;

/** 期望处理的 DMA 半区，0 表示前半区，1 表示后半区。 */
static uint8_t ads8688_expected_half;

/** 恢复待处理标志，恢复失败后保留并由后续主循环再次尝试。 */
static uint8_t ads8688_recovery_pending;
/** 配置回滚成功后是否需要恢复 DMA 运行。 */
static uint8_t ads8688_rollback_should_restart;

/**
 * @brief 构造 ADS8688 程序寄存器写帧。
 * @param address 程序寄存器地址，最高位会被屏蔽。
 * @param value 待写入的八位寄存器值。
 * @return 左对齐到 32 位 SPI 数据单元高半字的写命令。
 * @note 无副作用。
 */
static uint32_t ads8688_make_register_write(uint8_t address, uint8_t value)
{
    uint16_t command = ((uint16_t)(address & 0x7fu) << 9)
                     | (1u << 8)
                     | value;
    return (uint32_t)command << 16;
}

/**
 * @brief 构造 ADS8688 程序寄存器读帧。
 * @param address 程序寄存器地址，最高位会被屏蔽。
 * @return 左对齐到 32 位 SPI 数据单元高半字的读命令。
 * @note 无副作用。
 */
static uint32_t ads8688_make_register_read(uint8_t address)
{
    return (uint32_t)((uint16_t)(address & 0x7fu) << 9) << 16;
}

/**
 * @brief 构造 ADS8688 独立命令帧。
 * @param command 十六位 ADS8688 命令。
 * @return 左对齐到 32 位 SPI 数据单元高半字的命令帧。
 * @note 无副作用。
 */
static uint32_t ads8688_make_command(uint16_t command)
{
    return (uint32_t)command << 16;
}

/**
 * @brief 使用 SPI2 交换一个完整的 32 位 ADS8688 数据单元。
 * @param tx_word 待发送的 32 位帧。
 * @param received_word 用于接收 32 位返回帧的指针。
 * @return 传输成功返回 ADS8688_STATUS_OK，否则返回 ADS8688_STATUS_HAL_ERROR。
 * @note 阻塞占用 SPI2，最长等待 ADS8688_SPI_TIMEOUT_MS，不启动 DMA。
 */
static ads8688_status_t ads8688_transfer_word(uint32_t tx_word,
                                               uint32_t *received_word)
{
    HAL_StatusTypeDef hal_status;
    uint32_t rx_word = 0u;

    hal_status = HAL_SPI_TransmitReceive(&hspi3,
                                         (uint8_t *)&tx_word,
                                         (uint8_t *)&rx_word,
                                         1u,
                                         ADS8688_SPI_TIMEOUT_MS);
    if (hal_status != HAL_OK)
    {
        return ADS8688_STATUS_HAL_ERROR;
    }

    *received_word = rx_word;
    return ADS8688_STATUS_OK;
}

/**
 * @brief 发送一个无需解析返回数据的完整 ADS8688 命令帧。
 * @param command 十六位 ADS8688 命令。
 * @return 命令发送成功返回 ADS8688_STATUS_OK，否则返回 ADS8688_STATUS_HAL_ERROR。
 * @note 阻塞占用 SPI2 一次，接收数据会被丢弃。
 */
static ads8688_status_t ads8688_send_command(uint16_t command)
{
    uint32_t rx_word = 0u;
    uint32_t tx_word = ads8688_make_command(command);

    return ads8688_transfer_word(tx_word, &rx_word);
}

/**
 * @brief 读取一个 ADS8688 程序寄存器。
 * @param address 程序寄存器地址。
 * @param register_value 用于接收八位寄存器值的指针。
 * @return 读取成功返回 ADS8688_STATUS_OK，否则返回 ADS8688_STATUS_HAL_ERROR。
 * @note 阻塞占用 SPI2 一次，并按返回帧 bits[15:8] 提取寄存器数据。
 */
static ads8688_status_t ads8688_read_register(uint8_t address,
                                               uint8_t *register_value)
{
    ads8688_status_t status;
    uint32_t rx_word = 0u;
    uint32_t tx_word = ads8688_make_register_read(address);

    status = ads8688_transfer_word(tx_word, &rx_word);
    if (status != ADS8688_STATUS_OK)
    {
        return status;
    }

    *register_value = (uint8_t)((rx_word >> 8) & 0xffu);
    return ADS8688_STATUS_OK;
}

/**
 * @brief 写入并读回校验一个 ADS8688 程序寄存器。
 * @param address 程序寄存器地址。
 * @param value 待写入并校验的八位值。
 * @return 成功返回 ADS8688_STATUS_OK；HAL 失败或读回不符时返回对应错误状态。
 * @note 阻塞占用 SPI2 两次，写帧与读回帧相互独立。
 */
static ads8688_status_t ads8688_write_and_verify_register(uint8_t address,
                                                           uint8_t value)
{
    ads8688_status_t status;
    uint8_t register_value;
    uint32_t rx_word = 0u;
    uint32_t tx_word = ads8688_make_register_write(address, value);

    status = ads8688_transfer_word(tx_word, &rx_word);
    if (status != ADS8688_STATUS_OK)
    {
        return status;
    }

    status = ads8688_read_register(address, &register_value);
    if (status != ADS8688_STATUS_OK)
    {
        return status;
    }
    if (register_value != value)
    {
        return ADS8688_STATUS_VERIFY_ERROR;
    }

    return ADS8688_STATUS_OK;
}

/**
 * @brief 执行一次 ADS8688 硬件复位、寄存器配置和命令启动尝试。
 * @param 无。
 * @return 全部配置及校验成功返回 ADS8688_STATUS_OK，否则返回最终错误状态。
 * @note 改变 PD8、PD9 电平，延时至少 15 ms，并阻塞使用 SPI2；不启动 DMA。
 */
static ads8688_status_t ads8688_initialize_attempt(void)
{
    ads8688_status_t status;
    uint8_t address;
    volatile uint32_t nop_index;

    HAL_GPIO_WritePin(ADS8688_DAISY_GPIO_Port,
                      ADS8688_DAISY_Pin,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ADS8688_RST_GPIO_Port,
                      ADS8688_RST_Pin,
                      GPIO_PIN_RESET);
    for (nop_index = 0u;
         nop_index < ADS8688_RESET_HOLD_NOP_COUNT;
         nop_index++)
    {
        __NOP();
    }
    HAL_GPIO_WritePin(ADS8688_RST_GPIO_Port,
                      ADS8688_RST_Pin,
                      GPIO_PIN_SET);
    HAL_Delay(15u);

    status = ads8688_write_and_verify_register(
        ADS8688_REGISTER_AUTO_SEQUENCE,
        ADS8688_DEFAULT_CHANNEL_MASK);
    if (status != ADS8688_STATUS_OK)
    {
        return status;
    }

    status = ads8688_write_and_verify_register(
        ADS8688_REGISTER_FEATURE_SELECT,
        0x00u);
    if (status != ADS8688_STATUS_OK)
    {
        return status;
    }

    for (address = ADS8688_REGISTER_RANGE_CH0;
         address <= ADS8688_REGISTER_RANGE_CH7;
         address++)
    {
        status = ads8688_write_and_verify_register(
            address,
            (uint8_t)ADS8688_RANGE_BIPOLAR_5V12);
        if (status != ADS8688_STATUS_OK)
        {
            return status;
        }
    }

    return ads8688_send_command(ADS8688_COMMAND_AUTO_RST);
}

/**
 * @brief 判断量程枚举值是否为 ADS8688 支持的五种编码之一。
 * @param range 待检查的量程编码。
 * @return 有效返回 1，无效返回 0。
 * @note 无副作用。
 */
static uint8_t ads8688_range_is_valid(ads8688_range_t range)
{
    switch (range)
    {
        case ADS8688_RANGE_BIPOLAR_10V24:
        case ADS8688_RANGE_BIPOLAR_5V12:
        case ADS8688_RANGE_BIPOLAR_2V56:
        case ADS8688_RANGE_UNIPOLAR_10V24:
        case ADS8688_RANGE_UNIPOLAR_5V12:
            return 1u;

        default:
            return 0u;
    }
}

/**
 * @brief 获取自动扫描掩码中编号最低的已使能通道。
 * @param channel_mask 自动扫描通道掩码，调用者保证非零。
 * @return 编号最低的已使能通道。
 * @note 无副作用。
 */
static uint8_t ads8688_find_first_channel(uint8_t channel_mask)
{
    uint8_t channel;

    for (channel = 0u; channel < ADS8688_CHANNEL_COUNT; channel++)
    {
        if ((channel_mask & (uint8_t)(1u << channel)) != 0u)
        {
            return channel;
        }
    }

    return 0u;
}

/**
 * @brief 根据当前采集模式推进软件通道跟踪。
 * @param 无。
 * @return 无。
 * @note 自动模式跳过禁用通道并回绕；手动模式保持固定通道。
 */
static void ads8688_advance_channel(void)
{
    uint8_t offset;
    uint8_t candidate;

    if (ads8688_mode == ADS8688_MODE_MANUAL)
    {
        return;
    }

    for (offset = 1u; offset <= ADS8688_CHANNEL_COUNT; offset++)
    {
        candidate = (uint8_t)(
            (ads8688_current_channel + offset) % ADS8688_CHANNEL_COUNT);
        if ((ads8688_channel_mask & (uint8_t)(1u << candidate)) != 0u)
        {
            ads8688_current_channel = candidate;
            return;
        }
    }
}

/**
 * @brief 清除事件状态并启动 SPI2 循环 DMA 采集。
 * @param 无。
 * @return 启动成功返回 ADS8688_STATUS_OK，否则返回 ADS8688_STATUS_HAL_ERROR。
 * @note 重置 DMA 半区顺序，TX 端重复发送一个 32 位 NO_OP 帧。
 */
static ads8688_status_t ads8688_start_dma(void)
{
    if (ads8688_running != 0u)
    {
        return ADS8688_STATUS_OK;
    }
    ads8688_dma_half_flag = 0u;
    ads8688_dma_full_flag = 0u;
    ads8688_error_flag = 0u;
    ads8688_expected_half = 0u;
    ads8688_dma_tx_word = ADS8688_COMMAND_NO_OP;
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0u)
    {
        SCB_CleanInvalidateDCache_by_Addr(
            ads8688_dma_rx,
            (int32_t)sizeof(ads8688_dma_rx));
        SCB_CleanDCache_by_Addr(
            &ads8688_dma_tx_word,
            32);
    }

    if (HAL_SPI_TransmitReceive_DMA(&hspi3,
                                    (uint8_t *)&ads8688_dma_tx_word,
                                    (uint8_t *)ads8688_dma_rx,
                                    ADS8688_DMA_WORD_COUNT) != HAL_OK)
    {
        return ADS8688_STATUS_HAL_ERROR;
    }

    ads8688_running = 1u;
    return ADS8688_STATUS_OK;
}

/**
 * @brief 使用阻塞 Abort 停止 SPI2 DMA 采集。
 * @param 无。
 * @return 停止成功返回 ADS8688_STATUS_OK，否则返回 ADS8688_STATUS_HAL_ERROR。
 * @note 仅在返回成功后，调用者才可执行阻塞 SPI 传输或硬件复位。
 */
static ads8688_status_t ads8688_stop_dma(void)
{
    if (ads8688_running == 0u)
    {
        return ADS8688_STATUS_OK;
    }
    if (HAL_SPI_Abort(&hspi3) != HAL_OK)
    {
        return ADS8688_STATUS_HAL_ERROR;
    }

    ads8688_running = 0u;
    ads8688_dma_half_flag = 0u;
    ads8688_dma_full_flag = 0u;
    ads8688_error_flag = 0u;
    return ADS8688_STATUS_OK;
}

/**
 * @brief 在短临界区领取并清除一个中断共享标志。
 * @param flag 待领取的 volatile 标志指针。
 * @return 标志原先置位返回 1，否则返回 0。
 * @note 恢复进入函数前的中断屏蔽状态，耗时处理在临界区外执行。
 */
static uint8_t ads8688_claim_flag(volatile uint8_t *flag)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t claimed = 0u;

    __disable_irq();
    if (*flag != 0u)
    {
        *flag = 0u;
        claimed = 1u;
    }
    if (primask == 0u)
    {
        __enable_irq();
    }

    return claimed;
}

/**
 * @brief 在短临界区快照 DMA 两个半区完成标志。
 * @param 无。
 * @return bit0 表示前半区待处理，bit1 表示后半区待处理。
 * @note 不清除任何标志，用于区分调用开始时已经同时到达的两个事件。
 */
static uint8_t ads8688_snapshot_dma_flags(void)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t snapshot;

    __disable_irq();
    snapshot = (ads8688_dma_half_flag != 0u) ? 1u : 0u;
    if (ads8688_dma_full_flag != 0u)
    {
        snapshot |= 2u;
    }
    if (primask == 0u)
    {
        __enable_irq();
    }

    return snapshot;
}

/**
 * @brief 发送当前保存模式对应的 ADS8688 采集命令。
 * @param 无。
 * @return 命令发送结果。
 * @note 自动模式发送 AUTO_RST，手动模式发送固定通道命令。
 */
static ads8688_status_t ads8688_send_saved_mode_command(void)
{
    uint16_t command;

    if (ads8688_mode == ADS8688_MODE_AUTO)
    {
        command = ADS8688_COMMAND_AUTO_RST;
    }
    else
    {
        command = (uint16_t)(
            0xc000u + ((uint16_t)ads8688_current_channel << 10));
    }

    return ads8688_send_command(command);
}

/**
 * @brief 将软件保存的自动掩码、八通道量程及采集模式重新写入器件。
 * @param 无。
 * @return 全部配置成功返回 ADS8688_STATUS_OK，否则返回首个失败状态。
 * @note 使用阻塞 SPI，不启动 DMA，也不修改历史数据或采样序号。
 */
static ads8688_status_t ads8688_reapply_saved_configuration(void)
{
    ads8688_status_t status;
    uint8_t channel;

    status = ads8688_write_and_verify_register(
        ADS8688_REGISTER_AUTO_SEQUENCE,
        ads8688_channel_mask);
    if (status != ADS8688_STATUS_OK)
    {
        return status;
    }

    for (channel = 0u; channel < ADS8688_CHANNEL_COUNT; channel++)
    {
        status = ads8688_write_and_verify_register(
            (uint8_t)(ADS8688_REGISTER_RANGE_CH0 + channel),
            (uint8_t)ads8688_channel_ranges[channel]);
        if (status != ADS8688_STATUS_OK)
        {
            return status;
        }
    }

    return ads8688_send_saved_mode_command();
}

/**
 * @brief 尝试恢复已保存配置并重新启动 DMA。
 * @param 无。
 * @return 恢复及 DMA 启动结果。
 * @note 用于配置命令失败后的回退，不修改软件保存状态和诊断计数。
 */
static ads8688_status_t ads8688_restore_active_acquisition(void)
{
    ads8688_status_t status;
    uint8_t previous_channel = ads8688_current_channel;

    status = ads8688_reapply_saved_configuration();
    if (status != ADS8688_STATUS_OK)
    {
        return status;
    }

    ads8688_current_channel =
        (ads8688_mode == ADS8688_MODE_AUTO)
            ? ads8688_find_first_channel(ads8688_channel_mask)
            : ads8688_current_channel;
    if (ads8688_rollback_should_restart == 0u)
    {
        return ADS8688_STATUS_OK;
    }
    status = ads8688_start_dma();
    if (status != ADS8688_STATUS_OK)
    {
        ads8688_current_channel = previous_channel;
    }

    return status;
}

/**
 * @brief 配置事务失败后恢复旧硬件配置和旧 DMA 采集。
 * @param original_status 新配置事务最初产生的失败状态。
 * @return 始终返回 original_status。
 * @note 软件配置尚未提交；回滚失败时标记未初始化并保留恢复待处理状态。
 */
static ads8688_status_t ads8688_rollback_after_failure(
    ads8688_status_t original_status)
{
    if (ads8688_restore_active_acquisition() != ADS8688_STATUS_OK)
    {
        ads8688_initialized = 0u;
        ads8688_recovery_pending = 1u;
    }

    return original_status;
}

/**
 * @brief 在主循环上下文复位器件、恢复保存配置并重启 DMA。
 * @param 无。
 * @return 恢复成功返回 ADS8688_STATUS_OK，否则返回失败状态。
 * @note 保留模式、量程、采样序号、最新值和历史记录；成功时累计恢复次数。
 */
static ads8688_status_t ads8688_recover(void)
{
    ads8688_status_t status;

    status = ads8688_stop_dma();
    if (status != ADS8688_STATUS_OK)
    {
        ads8688_initialized = 0u;
        ads8688_recovery_pending = 1u;
        return status;
    }

    status = ads8688_initialize_attempt();
    if (status != ADS8688_STATUS_OK)
    {
        ads8688_initialized = 0u;
        ads8688_recovery_pending = 1u;
        return status;
    }

    ads8688_rollback_should_restart = 1u;
    status = ads8688_restore_active_acquisition();
    if (status != ADS8688_STATUS_OK)
    {
        ads8688_initialized = 0u;
        ads8688_recovery_pending = 1u;
        return status;
    }

    ads8688_initialized = 1u;
    ads8688_recovery_pending = 0u;
    ads8688_diagnostics.recoveries++;
    return ADS8688_STATUS_OK;
}

/**
 * @brief 按地址递增顺序处理一个已完成的 DMA 半缓冲区。
 * @param half_index 半区编号，0 对应前 512 个字，1 对应后 512 个字。
 * @return 无。
 * @note 每帧写入最新值和历史记录，并推进采样序号及自动通道跟踪。
 */
static void ads8688_process_dma_half(uint8_t half_index)
{
    uint32_t index;
    uint32_t start_index =
        (uint32_t)half_index * ADS8688_DMA_HALF_WORD_COUNT;
    uint32_t end_index = start_index + ADS8688_DMA_HALF_WORD_COUNT;

    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0u)
    {
        SCB_InvalidateDCache_by_Addr(
            &ads8688_dma_rx[start_index],
            (int32_t)(ADS8688_DMA_HALF_WORD_COUNT
                      * sizeof(ads8688_dma_rx[0])));
    }
    for (index = start_index; index < end_index; index++)
    {
        ads8688_storage_push(
            ads8688_current_channel,
            (uint16_t)(ads8688_dma_rx[index] & 0xffffu),
            ads8688_channel_ranges[ads8688_current_channel],
            ads8688_sample_index);
        if (ads8688_mode == ADS8688_MODE_MANUAL)
        {
            (void)measurement_fft_ingest_single(
                (uint16_t)(ads8688_dma_rx[index] & 0xffffu));
        }
        else
        {
            measurement_fft_ingest_sample(
                ads8688_current_channel,
                (uint16_t)(ads8688_dma_rx[index] & 0xffffu));
        }
        ads8688_sample_index++;
        ads8688_advance_channel();
    }

    ads8688_diagnostics.history_overwrites =
        ads8688_storage_get_overwrite_count();
}

/**
 * @brief 初始化 ADS8688 用户模块。
 * @param 无。
 * @return 初始化成功返回 ADS8688_STATUS_OK；三次尝试均失败时返回最后一次错误状态。
 * @note 重复初始化时先阻塞终止 DMA；最多配置三次，成功会清空采样存储和本地采集状态。
 */
ads8688_status_t ads8688_init(void)
{
    ads8688_status_t status = ADS8688_STATUS_HAL_ERROR;
    uint32_t attempt;
    uint32_t channel;

    if ((ads8688_initialized != 0u)
        || (ads8688_recovery_pending != 0u))
    {
        status = ads8688_stop_dma();
        if (status != ADS8688_STATUS_OK)
        {
            ads8688_initialized = 0u;
            ads8688_recovery_pending = 1u;
            return status;
        }
    }

    ads8688_initialized = 0u;
    ads8688_running = 0u;
    ads8688_recovery_pending = 0u;
    ads8688_rollback_should_restart = 0u;

    for (attempt = 0u; attempt < ADS8688_INITIALIZATION_RETRIES; attempt++)
    {
        status = ads8688_initialize_attempt();
        if (status == ADS8688_STATUS_OK)
        {
            ads8688_storage_init();
            ads8688_mode = ADS8688_MODE_AUTO;
            ads8688_channel_mask = ADS8688_DEFAULT_CHANNEL_MASK;
            ads8688_current_channel = 0u;
            for (channel = 0u; channel < ADS8688_CHANNEL_COUNT; channel++)
            {
                ads8688_channel_ranges[channel] =
                    ADS8688_RANGE_BIPOLAR_5V12;
            }
            ads8688_sample_index = 0u;
            ads8688_diagnostics.history_overwrites = 0u;

            ads8688_initialized = 1u;
            ads8688_recovery_pending = 0u;
            return ADS8688_STATUS_OK;
        }

        ads8688_diagnostics.initialization_failures++;
    }

    ads8688_initialized = 0u;
    return status;
}

/**
 * @brief 启动 ADS8688 SPI3 循环 DMA。
 * @param 无。
 * @return 启动状态。
 * @note 使用当前已保存的单双通道和量程配置。
 */
ads8688_status_t ads8688_start(void)
{
    if (ads8688_initialized == 0u)
    {
        return ADS8688_STATUS_NOT_INITIALIZED;
    }
    return ads8688_start_dma();
}

/**
 * @brief 停止 ADS8688 SPI3 循环 DMA。
 * @param 无。
 * @return 停止状态。
 * @note 重复停止安全，配置和历史数据保持不变。
 */
ads8688_status_t ads8688_stop(void)
{
    if (ads8688_initialized == 0u)
    {
        return ADS8688_STATUS_NOT_INITIALIZED;
    }
    return ads8688_stop_dma();
}

/**
 * @brief 处理 DMA 完成标志、采样顺序异常和 SPI/DMA 错误。
 * @param 无。
 * @return 无。
 * @note 由主循环调用；负责清除已处理标志、保存采样并执行阻塞恢复。
 */
void ads8688_process(void)
{
    ads8688_status_t status;
    uint8_t pending_flags;

    if (ads8688_recovery_pending != 0u)
    {
        status = ads8688_recover();
        if (status != ADS8688_STATUS_OK)
        {
            return;
        }
        return;
    }

    if (ads8688_initialized == 0u)
    {
        return;
    }

    if (ads8688_claim_flag(&ads8688_error_flag) != 0u)
    {
        ads8688_diagnostics.spi_dma_errors++;
        ads8688_recovery_pending = 1u;
        status = ads8688_recover();
        if (status != ADS8688_STATUS_OK)
        {
            return;
        }
        return;
    }

    pending_flags = ads8688_snapshot_dma_flags();
    if (ads8688_expected_half == 0u)
    {
        if ((pending_flags & 1u) != 0u)
        {
            if (ads8688_claim_flag(&ads8688_dma_half_flag) != 0u)
            {
                ads8688_expected_half = 1u;
                ads8688_process_dma_half(0u);
            }

            if (((pending_flags & 2u) != 0u)
                && (ads8688_claim_flag(&ads8688_dma_full_flag) != 0u))
            {
                ads8688_expected_half = 0u;
                ads8688_process_dma_half(1u);
            }
        }
        else if ((pending_flags & 2u) != 0u)
        {
            (void)ads8688_claim_flag(&ads8688_dma_full_flag);
            ads8688_diagnostics.lost_samples +=
                ADS8688_DMA_HALF_WORD_COUNT;
            ads8688_recovery_pending = 1u;
            status = ads8688_recover();
            if (status != ADS8688_STATUS_OK)
            {
                return;
            }
        }
    }
    else
    {
        if ((pending_flags & 2u) != 0u)
        {
            if (ads8688_claim_flag(&ads8688_dma_full_flag) != 0u)
            {
                ads8688_expected_half = 0u;
                ads8688_process_dma_half(1u);
            }

            if (((pending_flags & 1u) != 0u)
                && (ads8688_claim_flag(&ads8688_dma_half_flag) != 0u))
            {
                ads8688_expected_half = 1u;
                ads8688_process_dma_half(0u);
            }
        }
        else if ((pending_flags & 1u) != 0u)
        {
            (void)ads8688_claim_flag(&ads8688_dma_half_flag);
            ads8688_diagnostics.lost_samples +=
                ADS8688_DMA_HALF_WORD_COUNT;
            ads8688_recovery_pending = 1u;
            status = ads8688_recover();
            if (status != ADS8688_STATUS_OK)
            {
                return;
            }
        }
    }
}

/**
 * @brief 切换到指定通道掩码的自动扫描模式。
 * @param channel_mask 自动扫描通道位掩码，至少启用一个通道。
 * @return 配置、命令或 DMA 启动结果。
 * @note 停止并重启 DMA，保留历史记录和采样序号；配置失败时尝试恢复旧状态。
 */
ads8688_status_t ads8688_set_auto_mode(uint8_t channel_mask)
{
    ads8688_status_t status;
    uint8_t was_running;

    if (ads8688_initialized == 0u)
    {
        return ADS8688_STATUS_NOT_INITIALIZED;
    }
    if (channel_mask == 0u)
    {
        return ADS8688_STATUS_INVALID_ARGUMENT;
    }

    was_running = ads8688_running;
    ads8688_rollback_should_restart = was_running;
    status = ads8688_stop_dma();
    if (status != ADS8688_STATUS_OK)
    {
        ads8688_initialized = 0u;
        ads8688_recovery_pending = 1u;
        ads8688_diagnostics.spi_dma_errors++;
        return status;
    }

    status = ads8688_write_and_verify_register(
        ADS8688_REGISTER_AUTO_SEQUENCE,
        channel_mask);
    if (status != ADS8688_STATUS_OK)
    {
        return ads8688_rollback_after_failure(status);
    }

    status = ads8688_send_command(ADS8688_COMMAND_AUTO_RST);
    if (status != ADS8688_STATUS_OK)
    {
        return ads8688_rollback_after_failure(status);
    }

    ads8688_mode = ADS8688_MODE_AUTO;
    ads8688_channel_mask = channel_mask;
    ads8688_current_channel = ads8688_find_first_channel(channel_mask);
    if (was_running != 0u)
    {
        status = ads8688_start_dma();
        if (status != ADS8688_STATUS_OK)
        {
            return ads8688_rollback_after_failure(status);
        }
    }
    return ADS8688_STATUS_OK;
}

/**
 * @brief 切换到指定单通道的手动采集模式。
 * @param channel 手动采集通道号，有效范围为 0 至 7。
 * @return 命令或 DMA 启动结果。
 * @note 停止并重启 DMA，保留历史记录和采样序号；命令失败时尝试恢复旧状态。
 */
ads8688_status_t ads8688_set_manual_mode(uint8_t channel)
{
    ads8688_status_t status;
    uint16_t command;
    uint8_t was_running;

    if (ads8688_initialized == 0u)
    {
        return ADS8688_STATUS_NOT_INITIALIZED;
    }
    if (channel >= ADS8688_CHANNEL_COUNT)
    {
        return ADS8688_STATUS_INVALID_ARGUMENT;
    }

    was_running = ads8688_running;
    ads8688_rollback_should_restart = was_running;
    status = ads8688_stop_dma();
    if (status != ADS8688_STATUS_OK)
    {
        ads8688_initialized = 0u;
        ads8688_recovery_pending = 1u;
        ads8688_diagnostics.spi_dma_errors++;
        return status;
    }

    command = (uint16_t)(0xc000u + ((uint16_t)channel << 10));
    status = ads8688_send_command(command);
    if (status != ADS8688_STATUS_OK)
    {
        return ads8688_rollback_after_failure(status);
    }

    ads8688_mode = ADS8688_MODE_MANUAL;
    ads8688_current_channel = channel;
    if (was_running != 0u)
    {
        status = ads8688_start_dma();
        if (status != ADS8688_STATUS_OK)
        {
            return ads8688_rollback_after_failure(status);
        }
    }
    return ADS8688_STATUS_OK;
}

/**
 * @brief 将 ADS8688 配置为指定单通道手动采样。
 * @param channel 物理输入通道。
 * @return 配置状态。
 * @note 复用手动模式实现。
 */
ads8688_status_t ads8688_set_single_channel(uint8_t channel)
{
    return ads8688_set_manual_mode(channel);
}

/**
 * @brief 将 ADS8688 配置为 AIN0/AIN1 自动轮询。
 * @param 无。
 * @return 配置状态。
 * @note 复用自动序列模式实现。
 */
ads8688_status_t ads8688_set_dual_channel(void)
{
    return ads8688_set_auto_mode(ADS8688_DEFAULT_CHANNEL_MASK);
}

/**
 * @brief 根据 SPI3 内核时钟、预分频和通道模式计算每通道采样率。
 * @param 无。
 * @return 当前每个有效通道的标称采样率，计算失败返回 0。
 * @note 每个转换帧按 32 位数据和 1 个帧间时钟计算。
 */
float ads8688_get_effective_sample_rate_hz(void)
{
    uint32_t kernel_clock_hz;
    uint32_t prescaler;
    float frame_rate_hz;

    kernel_clock_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI3);
    switch (hspi3.Init.BaudRatePrescaler)
    {
        case SPI_BAUDRATEPRESCALER_2:
            prescaler = 2u;
            break;
        case SPI_BAUDRATEPRESCALER_4:
            prescaler = 4u;
            break;
        case SPI_BAUDRATEPRESCALER_8:
            prescaler = 8u;
            break;
        case SPI_BAUDRATEPRESCALER_16:
            prescaler = 16u;
            break;
        case SPI_BAUDRATEPRESCALER_32:
            prescaler = 32u;
            break;
        case SPI_BAUDRATEPRESCALER_64:
            prescaler = 64u;
            break;
        case SPI_BAUDRATEPRESCALER_128:
            prescaler = 128u;
            break;
        case SPI_BAUDRATEPRESCALER_256:
            prescaler = 256u;
            break;
        default:
            return 0.0f;
    }

    frame_rate_hz = ((float)kernel_clock_hz / (float)prescaler)
                    / ADS8688_SPI_FRAME_CYCLES;
    return (ads8688_mode == ADS8688_MODE_AUTO)
               ? frame_rate_hz * 0.5f
               : frame_rate_hz;
}

/**
 * @brief 设置指定输入通道的量程并恢复当前采集模式。
 * @param channel 待配置通道号，有效范围为 0 至 7。
 * @param range 五种有效 ADS8688 量程编码之一。
 * @return 参数、寄存器校验、命令或 DMA 启动结果。
 * @note 停止并重启 DMA，保留历史记录和采样序号；配置失败时尝试恢复旧状态。
 */
ads8688_status_t ads8688_set_channel_range(uint8_t channel,
                                            ads8688_range_t range)
{
    ads8688_status_t status;
    uint8_t was_running;

    if (ads8688_initialized == 0u)
    {
        return ADS8688_STATUS_NOT_INITIALIZED;
    }
    if ((channel >= ADS8688_CHANNEL_COUNT)
        || (ads8688_range_is_valid(range) == 0u))
    {
        return ADS8688_STATUS_INVALID_ARGUMENT;
    }

    was_running = ads8688_running;
    ads8688_rollback_should_restart = was_running;
    status = ads8688_stop_dma();
    if (status != ADS8688_STATUS_OK)
    {
        ads8688_initialized = 0u;
        ads8688_recovery_pending = 1u;
        ads8688_diagnostics.spi_dma_errors++;
        return status;
    }

    status = ads8688_write_and_verify_register(
        (uint8_t)(ADS8688_REGISTER_RANGE_CH0 + channel),
        (uint8_t)range);
    if (status != ADS8688_STATUS_OK)
    {
        return ads8688_rollback_after_failure(status);
    }

    status = ads8688_send_saved_mode_command();
    if (status != ADS8688_STATUS_OK)
    {
        return ads8688_rollback_after_failure(status);
    }

    ads8688_channel_ranges[channel] = range;
    if (ads8688_mode == ADS8688_MODE_AUTO)
    {
        ads8688_current_channel =
            ads8688_find_first_channel(ads8688_channel_mask);
    }
    if (was_running != 0u)
    {
        status = ads8688_start_dma();
        if (status != ADS8688_STATUS_OK)
        {
            return ads8688_rollback_after_failure(status);
        }
    }
    return ADS8688_STATUS_OK;
}

/**
 * @brief 读取指定通道当前使用的输入量程。
 * @param channel 待读取通道号。
 * @param range 用于接收当前量程的指针。
 * @return 成功返回 ADS8688_STATUS_OK；参数无效或模块未初始化时返回对应错误状态。
 * @note 只读取软件保存的已生效量程，不发起 SPI 传输。
 */
ads8688_status_t ads8688_get_channel_range(uint8_t channel,
                                            ads8688_range_t *range)
{
    if ((channel >= ADS8688_CHANNEL_COUNT) || (range == 0))
    {
        return ADS8688_STATUS_INVALID_ARGUMENT;
    }
    if (ads8688_initialized == 0u)
    {
        return ADS8688_STATUS_NOT_INITIALIZED;
    }

    *range = ads8688_channel_ranges[channel];
    return ADS8688_STATUS_OK;
}

/**
 * @brief 读取指定通道的最新采样值。
 * @param channel 待读取通道号。
 * @param latest 用于接收结果的结构体指针。
 * @return 存储模块返回的参数检查及读取状态。
 * @note 不清除最新值或历史记录。
 */
ads8688_status_t ads8688_get_latest(uint8_t channel,
                                    ads8688_latest_t *latest)
{
    return ads8688_storage_get_latest(channel, latest);
}

/**
 * @brief 按时间顺序读取并移除历史采样记录。
 * @param samples 用于接收记录的数组。
 * @param max_count 数组最大记录数。
 * @return 实际读取的记录数。
 * @note 同步历史覆盖诊断计数，不影响各通道最新值。
 */
uint32_t ads8688_read_history(ads8688_sample_t *samples,
                              uint32_t max_count)
{
    uint32_t count = ads8688_storage_read(samples, max_count);

    ads8688_diagnostics.history_overwrites =
        ads8688_storage_get_overwrite_count();
    return count;
}

/**
 * @brief 清空历史采样记录。
 * @param 无。
 * @return 无。
 * @note 保留各通道最新值并将历史覆盖诊断计数同步为零。
 */
void ads8688_clear_history(void)
{
    ads8688_storage_clear();
    ads8688_diagnostics.history_overwrites =
        ads8688_storage_get_overwrite_count();
}

/**
 * @brief 读取 ADS8688 累计诊断信息。
 * @param diagnostics 用于接收诊断信息的结构体指针。
 * @return 成功返回 ADS8688_STATUS_OK，空指针返回 ADS8688_STATUS_INVALID_ARGUMENT。
 * @note 复制前同步历史缓冲区覆盖计数。
 */
ads8688_status_t ads8688_get_diagnostics(
    ads8688_diagnostics_t *diagnostics)
{
    if (diagnostics == 0)
    {
        return ADS8688_STATUS_INVALID_ARGUMENT;
    }

    ads8688_diagnostics.history_overwrites =
        ads8688_storage_get_overwrite_count();
    *diagnostics = ads8688_diagnostics;
    return ADS8688_STATUS_OK;
}

/**
 * @brief SPI2 DMA 前半区传输完成回调。
 * @param hspi 触发回调的 SPI 句柄，本回调不在中断中筛选实例。
 * @return 无。
 * @note 中断上下文仅置位前半区完成标志。
 */
void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi3)
    {
        ads8688_dma_half_flag = 1u;
    }
}

/**
 * @brief SPI2 DMA 全缓冲区传输完成回调。
 * @param hspi 触发回调的 SPI 句柄，本回调不在中断中筛选实例。
 * @return 无。
 * @note 中断上下文仅置位全缓冲区完成标志。
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi3)
    {
        ads8688_dma_full_flag = 1u;
    }
}

/**
 * @brief SPI 错误回调。
 * @param hspi 触发回调的 SPI 句柄，本回调不在中断中筛选实例。
 * @return 无。
 * @note 中断上下文仅置位错误标志，恢复在主循环执行。
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi3)
    {
        ads8688_error_flag = 1u;
    }
}
