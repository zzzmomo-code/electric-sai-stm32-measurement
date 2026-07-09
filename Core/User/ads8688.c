/**
 * @file ads8688.c
 * @brief ADS8688 寄存器传输与器件初始化实现。
 *
 * 模块用途：通过 SPI2 配置并校验 ADS8688 程序寄存器，复位器件并建立采集初始状态。
 * GPIO 引脚映射：PD8 连接 ADS8688 RST/PD，PD9 连接 ADS8688 DAISY。
 * 依赖的外设和 CubeIDE 配置：依赖 SPI2 的 32 位数据帧、第二边沿采样、硬件 NSS
 * 和 9 周期数据间空闲配置；依赖 PD8、PD9 推挽输出；本模块不修改 CubeIDE 配置。
 * 初始化方法：CubeMX 完成 GPIO 与 SPI2 初始化后调用 ads8688_init()。
 * 调用方法：初始化成功后由后续采集模块启动 DMA 并调用公共采集接口。
 */

#include "system.h"

#define ADS8688_COMMAND_NO_OP             0x0000u
#define ADS8688_COMMAND_AUTO_RST          0xa000u
#define ADS8688_REGISTER_AUTO_SEQUENCE    0x01u
#define ADS8688_REGISTER_FEATURE_SELECT   0x03u
#define ADS8688_REGISTER_RANGE_CH0        0x05u
#define ADS8688_REGISTER_RANGE_CH7        0x0cu
#define ADS8688_DEFAULT_CHANNEL_MASK      0xffu
#define ADS8688_INITIALIZATION_RETRIES    3u
#define ADS8688_SPI_TIMEOUT_MS            10u

/** PD8 上的 ADS8688 复位及掉电控制信号。 */
#define ADS8688_RESET_PIN                 GPIO_PIN_8

/** PD9 上的 ADS8688 菊花链模式选择信号。 */
#define ADS8688_DAISY_PIN                 GPIO_PIN_9

/** 复位低电平期间执行的有界空操作次数，保证持续时间安全超过 400 ns。 */
#define ADS8688_RESET_HOLD_NOP_COUNT      256u

/** 模块是否已经完成器件配置及读回校验。 */
static uint8_t ads8688_initialized;

/** 当前采集模式，初始化成功后默认为自动扫描。 */
static ads8688_mode_t ads8688_mode;

/** 自动扫描通道掩码，初始化成功后启用全部八个通道。 */
static uint8_t ads8688_channel_mask;

/** 当前待处理通道号，初始化成功后从通道零开始。 */
static uint8_t ads8688_current_channel;

/** 八个输入通道当前采用的量程编码。 */
static ads8688_range_t ads8688_channel_ranges[ADS8688_CHANNEL_COUNT];

/** 后续采集使用的累计采样序号。 */
static uint32_t ads8688_sample_index;

/** ADS8688 初始化和运行故障的累计诊断信息。 */
static ads8688_diagnostics_t ads8688_diagnostics;

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

    hal_status = HAL_SPI_TransmitReceive(&hspi2,
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

    HAL_GPIO_WritePin(GPIOD, ADS8688_DAISY_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, ADS8688_RESET_PIN, GPIO_PIN_RESET);
    for (nop_index = 0u;
         nop_index < ADS8688_RESET_HOLD_NOP_COUNT;
         nop_index++)
    {
        __NOP();
    }
    HAL_GPIO_WritePin(GPIOD, ADS8688_RESET_PIN, GPIO_PIN_SET);
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
            (uint8_t)ADS8688_RANGE_BIPOLAR_10V24);
        if (status != ADS8688_STATUS_OK)
        {
            return status;
        }
    }

    return ads8688_send_command(ADS8688_COMMAND_AUTO_RST);
}

/**
 * @brief 初始化 ADS8688 用户模块。
 * @param 无。
 * @return 初始化成功返回 ADS8688_STATUS_OK；三次尝试均失败时返回最后一次错误状态。
 * @note 最多复位并配置器件三次；失败会累计诊断计数，成功会清空采样存储和本地采集状态。
 */
ads8688_status_t ads8688_init(void)
{
    ads8688_status_t status = ADS8688_STATUS_HAL_ERROR;
    uint32_t attempt;
    uint32_t channel;

    ads8688_initialized = 0u;

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
                    ADS8688_RANGE_BIPOLAR_10V24;
            }
            ads8688_sample_index = 0u;
            ads8688_initialized = 1u;
            return ADS8688_STATUS_OK;
        }

        ads8688_diagnostics.initialization_failures++;
    }

    ads8688_initialized = 0u;
    return status;
}
