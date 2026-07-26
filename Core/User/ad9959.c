/**
 * @file ad9959.c
 * @brief AD9959 DDS底层驱动实现。
 *
 * 模块用途：使用SPI4和手动CS/RESET/IO_UPDATE完成AD9959双通道的频率、相位、
 * 幅度寄存器写入与读回。两路通道可独立配置，便于外差式测量与数字锁相环闭环。
 * GPIO引脚映射：PE2/SPI4_SCK，PE5/SPI4_MISO，PE6/SPI4_MOSI，
 * PD4/IO_UPDATE，PD5/CS，PB4/RESET。
 * 依赖的外设和CubeIDE配置：SPI4主机Full-Duplex、8bit、MSB优先、
 * CPOL=Low、CPHA=1 Edge、15 Mbit/s；CS低有效，RESET高有效复位，
 * IO_UPDATE上升沿刷新寄存器；25MHz外部晶振经片内PLL 20倍频得到500MHz系统时钟。
 * 初始化方法：由system_init()调用ad9959_init()，默认输出1MHz正弦波。
 * 调用方法：主循环中独立设置两个通道的频率/相位/幅度；禁止在中断中调用。
 */

#include "system.h"

/* AD9959 寄存器地址 */
#define AD9959_REG_CSR    0x00u  /**< Channel Select Register, 1 字节 */
#define AD9959_REG_FR1    0x01u  /**< Function Register 1, 3 字节 */
#define AD9959_REG_FR2    0x02u  /**< Function Register 2, 2 字节 */
#define AD9959_REG_CFR    0x03u  /**< Channel Function Register, 3 字节 */
#define AD9959_REG_CFTW0  0x04u  /**< Channel Frequency Tuning Word 0, 4 字节 */
#define AD9959_REG_CPOW0  0x05u  /**< Channel Phase Offset Word 0, 2 字节 */
#define AD9959_REG_ACR    0x06u  /**< Amplitude Control Register, 3 字节 */

/** SPI 指令字节 bit7=1 表示读操作 */
#define AD9959_READ_BIT 0x80u

/** SPI 阻塞超时，单位 ms */
#define AD9959_SPI_TIMEOUT_MS 10u

/** ACR 寄存器中 bit12 启用手动幅度控制 */
#define AD9959_ACR_AMPLITUDE_ENABLE 0x10u

/** CSR 通道使能位，bit4=CH0，bit5=CH1，bit6=CH2，bit7=CH3 */
static const uint8_t ad9959_csr_channel_enable[2] = { 0x10u, 0x20u };

/** FR1 默认值：PLL 20 倍频（25MHz×20=500MHz）、VCO 高范围、charge pump=75uA 默认。
 *  字节顺序：byte0=MSB（含 VCO gain、PLL enable、PLL ratio 高 5 位），byte2=LSB。
 *  对应商家康威科技 KV-AD9959 模块验证值：0xD0 = 1101_0000，
 *  bit 23=VCO gain、bit 22-18=10100=20 倍频、bit 17-15=000=charge pump 75uA 默认。
 */
static const uint8_t ad9959_fr1_default[3] = { 0xD0u, 0x00u, 0x00u };

/** FR2 默认值：保持默认 */
static const uint8_t ad9959_fr2_default[2] = { 0x00u, 0x00u };

/** CFR 默认值：单音模式（Single Tone），与 veis-lzf 例程一致 */
static const uint8_t ad9959_cfr_default[3] = { 0x00u, 0x03u, 0x02u };

/** AD9959 运行诊断快照，仅由驱动调用上下文修改。 */
volatile ad9959_diagnostics_t ad9959_diagnostics;

/**
 * @brief 产生 IO_UPDATE 上升沿，将影子寄存器内容加载到实际工作寄存器。
 * @param 无。
 * @return 无。
 * @note SYNC_CLK = SYSCLK/4 = 125MHz，对应周期 8ns；
 *       IO_UPDATE 高电平至少 1 个 SYNC_CLK 周期，这里用 16 个 __NOP() 保留充足余量
 *       （约 33ns @ 480MHz CPU），并考虑 PD4 引脚上升/下降沿时间。
 */
static void ad9959_io_update(void)
{
    HAL_GPIO_WritePin(update9959_GPIO_Port, update9959_Pin, GPIO_PIN_RESET);
    __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP();
    HAL_GPIO_WritePin(update9959_GPIO_Port, update9959_Pin, GPIO_PIN_SET);
    __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP();
    HAL_GPIO_WritePin(update9959_GPIO_Port, update9959_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 对 AD9959 执行硬件复位。
 * @param 无。
 * @return 无。
 * @note RESET 高电平至少 13ns 即可触发复位，这里用 HAL_Delay 1ms 保险；
 *       复位后等待 5ms 让内部状态稳定。
 */
static void ad9959_hardware_reset(void)
{
    HAL_GPIO_WritePin(AD9959_RST_GPIO_Port, AD9959_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(AD9959_RST_GPIO_Port, AD9959_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(5);
}

/**
 * @brief 通过 SPI4 向 AD9959 写入一个寄存器。
 * @param address 寄存器地址（0x00-0x06）。
 * @param data 待写入数据缓冲区指针，按 MSB first 排列。
 * @param length 数据字节数。
 * @return 驱动状态。
 * @note 顺序：CS 拉低 -> 发指令字节（bit7=0 表示写）-> 发数据 -> CS 拉高 -> IO_UPDATE。
 *       无论 SPI 成功或失败，退出前都会恢复 CS 为高。
 */
static ad9959_status_t ad9959_write_register(uint8_t address,
                                             const uint8_t *data,
                                             uint8_t length)
{
    HAL_StatusTypeDef hal_status;
    uint8_t header = (uint8_t)(address & 0x7Fu);

    HAL_GPIO_WritePin(AD9959_CS_GPIO_Port, AD9959_CS_Pin, GPIO_PIN_RESET);
    hal_status = HAL_SPI_Transmit(&hspi4, &header, 1u, AD9959_SPI_TIMEOUT_MS);
    if (hal_status == HAL_OK)
    {
        hal_status = HAL_SPI_Transmit(&hspi4, data, length, AD9959_SPI_TIMEOUT_MS);
    }
    HAL_GPIO_WritePin(AD9959_CS_GPIO_Port, AD9959_CS_Pin, GPIO_PIN_SET);

    ad9959_diagnostics.last_hal_status = (int32_t)hal_status;
    if (hal_status != HAL_OK)
    {
        ad9959_diagnostics.error_count++;
        return ad9959_status_spi_error;
    }

    ad9959_diagnostics.write_count++;
    ad9959_io_update();
    return ad9959_status_ok;
}

/**
 * @brief 通过 SPI4 读回 AD9959 一个寄存器的值。
 * @param address 寄存器地址（0x00-0x06）。
 * @param data 存放读回数据的缓冲区。
 * @param length 要读的字节数。
 * @return 驱动状态。
 * @note 顺序：CS 拉低 -> 发指令字节（bit7=1 表示读）-> 发 dummy 字节同时读 MISO ->
 *       CS 拉高。读操作不产生 IO_UPDATE。
 */
static ad9959_status_t ad9959_read_register_raw(uint8_t address,
                                               uint8_t *data,
                                               uint8_t length)
{
    HAL_StatusTypeDef hal_status;
    uint8_t header = (uint8_t)(address | AD9959_READ_BIT);
    static uint8_t dummy[AD9959_READ_MAX_BYTES] = {0};

    HAL_GPIO_WritePin(AD9959_CS_GPIO_Port, AD9959_CS_Pin, GPIO_PIN_RESET);
    hal_status = HAL_SPI_Transmit(&hspi4, &header, 1u, AD9959_SPI_TIMEOUT_MS);
    if (hal_status == HAL_OK)
    {
        hal_status = HAL_SPI_TransmitReceive(&hspi4, dummy, data, length,
                                            AD9959_SPI_TIMEOUT_MS);
    }
    HAL_GPIO_WritePin(AD9959_CS_GPIO_Port, AD9959_CS_Pin, GPIO_PIN_SET);

    ad9959_diagnostics.last_hal_status = (int32_t)hal_status;
    if (hal_status != HAL_OK)
    {
        ad9959_diagnostics.error_count++;
        return ad9959_status_spi_error;
    }

    return ad9959_status_ok;
}

/**
 * @brief 通过写 CSR 寄存器选择目标通道。
 * @param channel 要选择的通道（ad9959_channel_0 或 ad9959_channel_1）。
 * @return 驱动状态。
 * @note 非法通道直接返回错误，不发起 SPI 操作。
 */
static ad9959_status_t ad9959_select_channel(ad9959_channel_t channel)
{
    uint8_t csr;

    if ((channel != ad9959_channel_0) && (channel != ad9959_channel_1))
    {
        return ad9959_status_invalid_channel;
    }

    csr = ad9959_csr_channel_enable[channel];
    return ad9959_write_register(AD9959_REG_CSR, &csr, 1u);
}

/**
 * @brief 计算 AD9959 的 32 位频率控制字。
 * @param frequency_hz 目标输出频率，单位Hz。
 * @return 按 500MHz MCLK 四舍五入后的 32 位频率控制字。
 */
uint32_t ad9959_calculate_tuning_word(uint32_t frequency_hz)
{
    return (uint32_t)((((uint64_t)frequency_hz) << 32u)
                       + (AD9959_MCLK_HZ / 2u)) / AD9959_MCLK_HZ;
}

/**
 * @brief 初始化 AD9959 并输出默认 1MHz 正弦波。
 * @return 驱动状态。
 * @note 步骤：硬件复位 -> 写 FR1（PLL 20倍频）-> 等待 PLL 锁定 -> 写 FR2 ->
 *       两个通道分别写 CFR、默认频率、相位、幅度。任一 SPI 失败立即返回错误，
 *       退出前保证 CS 为高；initialized 标志只在全部步骤成功后置 1。
 */
ad9959_status_t ad9959_init(void)
{
    ad9959_status_t status;

    ad9959_diagnostics.write_count = 0u;
    ad9959_diagnostics.error_count = 0u;
    ad9959_diagnostics.last_hal_status = (int32_t)HAL_OK;
    ad9959_diagnostics.initialized = 0u;
    ad9959_diagnostics.last_channel = 0u;
    ad9959_diagnostics.frequency_hz[0] = 0u;
    ad9959_diagnostics.frequency_hz[1] = 0u;
    ad9959_diagnostics.frequency_tuning_word[0] = 0u;
    ad9959_diagnostics.frequency_tuning_word[1] = 0u;
    ad9959_diagnostics.phase_degrees[0] = 0u;
    ad9959_diagnostics.phase_degrees[1] = 0u;
    ad9959_diagnostics.phase_word[0] = 0u;
    ad9959_diagnostics.phase_word[1] = 0u;
    ad9959_diagnostics.amplitude[0] = 0u;
    ad9959_diagnostics.amplitude[1] = 0u;

    /* 上电等待电源稳定：商家建议 500ms，让模块 5V LDO 和 25MHz 晶振充分稳定 */
    HAL_Delay(500);

    ad9959_hardware_reset();

    status = ad9959_write_register(AD9959_REG_FR1, ad9959_fr1_default, 3u);
    if (status != ad9959_status_ok)
    {
        return status;
    }
    HAL_Delay(5);

    status = ad9959_write_register(AD9959_REG_FR2, ad9959_fr2_default, 2u);
    if (status != ad9959_status_ok)
    {
        return status;
    }

    status = ad9959_select_channel(ad9959_channel_0);
    if (status != ad9959_status_ok)
    {
        return status;
    }
    status = ad9959_write_register(AD9959_REG_CFR, ad9959_cfr_default, 3u);
    if (status != ad9959_status_ok)
    {
        return status;
    }

    status = ad9959_select_channel(ad9959_channel_1);
    if (status != ad9959_status_ok)
    {
        return status;
    }
    status = ad9959_write_register(AD9959_REG_CFR, ad9959_cfr_default, 3u);
    if (status != ad9959_status_ok)
    {
        return status;
    }

    status = ad9959_set_frequency(ad9959_channel_0, 1000000u);
    if (status != ad9959_status_ok)
    {
        return status;
    }
    status = ad9959_set_frequency(ad9959_channel_1, 1000000u);
    if (status != ad9959_status_ok)
    {
        return status;
    }

    status = ad9959_set_phase(ad9959_channel_0, 0u);
    if (status != ad9959_status_ok)
    {
        return status;
    }
    status = ad9959_set_phase(ad9959_channel_1, 0u);
    if (status != ad9959_status_ok)
    {
        return status;
    }

    status = ad9959_set_amplitude(ad9959_channel_0, AD9959_AMPLITUDE_MAX);
    if (status != ad9959_status_ok)
    {
        return status;
    }
    status = ad9959_set_amplitude(ad9959_channel_1, AD9959_AMPLITUDE_MAX);
    if (status != ad9959_status_ok)
    {
        return status;
    }

    ad9959_diagnostics.initialized = 1u;
    return ad9959_status_ok;
}

/**
 * @brief 设置指定通道的输出频率。
 * @param channel 目标通道。
 * @param frequency_hz 目标输出频率，单位Hz。
 * @return 驱动状态。
 * @note 先写 CSR 选择通道，再写 4 字节 CFTW0（MSB first），最后产生 IO_UPDATE。
 *       非法频率或通道不发起 SPI 操作；SPI 失败时保持当前输出不变。
 */
ad9959_status_t ad9959_set_frequency(ad9959_channel_t channel,
                                     uint32_t frequency_hz)
{
    ad9959_status_t status;
    uint32_t ftw;
    uint8_t data[4];

    if ((channel != ad9959_channel_0) && (channel != ad9959_channel_1))
    {
        return ad9959_status_invalid_channel;
    }
    if ((frequency_hz == 0u) || (frequency_hz > AD9959_MAX_OUTPUT_HZ))
    {
        return ad9959_status_invalid_frequency;
    }

    status = ad9959_select_channel(channel);
    if (status != ad9959_status_ok)
    {
        return status;
    }

    ftw = ad9959_calculate_tuning_word(frequency_hz);
    /* 32 位频率字，MSB first 发送 */
    data[0] = (uint8_t)(ftw >> 24u);
    data[1] = (uint8_t)(ftw >> 16u);
    data[2] = (uint8_t)(ftw >> 8u);
    data[3] = (uint8_t)(ftw);

    status = ad9959_write_register(AD9959_REG_CFTW0, data, 4u);
    if (status != ad9959_status_ok)
    {
        return status;
    }

    ad9959_diagnostics.frequency_hz[channel] = frequency_hz;
    ad9959_diagnostics.frequency_tuning_word[channel] = ftw;
    ad9959_diagnostics.last_channel = (uint8_t)channel;
    return ad9959_status_ok;
}

/**
 * @brief 设置指定通道的输出相位。
 * @param channel 目标通道。
 * @param phase_degrees 目标相位，范围 0 至 359 度。
 * @return 驱动状态。
 * @note 先写 CSR 选择通道，再写 2 字节 CPOW0（MSB first），最后产生 IO_UPDATE。
 *       非法相位或通道不发起 SPI 操作；SPI 失败时保持当前输出不变。
 */
ad9959_status_t ad9959_set_phase(ad9959_channel_t channel,
                                 uint16_t phase_degrees)
{
    ad9959_status_t status;
    uint16_t pow;
    uint8_t data[2];

    if ((channel != ad9959_channel_0) && (channel != ad9959_channel_1))
    {
        return ad9959_status_invalid_channel;
    }
    if (phase_degrees > 359u)
    {
        return ad9959_status_invalid_phase;
    }

    status = ad9959_select_channel(channel);
    if (status != ad9959_status_ok)
    {
        return status;
    }

    /* 14 位相位字：POW = phase × 2^14 / 360，四舍五入 */
    pow = (uint16_t)((((uint32_t)phase_degrees) * 16384u + 180u) / 360u);
    /* MSB first 发送 */
    data[0] = (uint8_t)(pow >> 8u);
    data[1] = (uint8_t)(pow);

    status = ad9959_write_register(AD9959_REG_CPOW0, data, 2u);
    if (status != ad9959_status_ok)
    {
        return status;
    }

    ad9959_diagnostics.phase_degrees[channel] = phase_degrees;
    ad9959_diagnostics.phase_word[channel] = pow;
    ad9959_diagnostics.last_channel = (uint8_t)channel;
    return ad9959_status_ok;
}

/**
 * @brief 设置指定通道的输出幅度。
 * @param channel 目标通道。
 * @param amplitude 目标幅度，范围 0 至 1023（0=零输出，1023=满量程）。
 * @return 驱动状态。
 * @note 先写 CSR 选择通道，再写 3 字节 ACR（bit12 置 1 启用手动幅度控制），
 *       最后产生 IO_UPDATE。非法幅度或通道不发起 SPI 操作；
 *       SPI 失败时保持当前输出不变。
 */
ad9959_status_t ad9959_set_amplitude(ad9959_channel_t channel,
                                     uint16_t amplitude)
{
    ad9959_status_t status;
    uint8_t data[3];

    if ((channel != ad9959_channel_0) && (channel != ad9959_channel_1))
    {
        return ad9959_status_invalid_channel;
    }
    if (amplitude > AD9959_AMPLITUDE_MAX)
    {
        return ad9959_status_invalid_amplitude;
    }

    status = ad9959_select_channel(channel);
    if (status != ad9959_status_ok)
    {
        return status;
    }

    /* ACR 3 字节，MSB first：
     *   byte0 (MSB) = 0x00
     *   byte1       = 0x10 | (amplitude >> 8)   -- bit12 启用手动幅度控制 + 幅度高 2 位
     *   byte2 (LSB) = amplitude & 0xFF          -- 幅度低 8 位
     */
    data[0] = 0x00u;
    data[1] = (uint8_t)(AD9959_ACR_AMPLITUDE_ENABLE
                        | (uint8_t)(amplitude >> 8u));
    data[2] = (uint8_t)(amplitude & 0xFFu);

    status = ad9959_write_register(AD9959_REG_ACR, data, 3u);
    if (status != ad9959_status_ok)
    {
        return status;
    }

    ad9959_diagnostics.amplitude[channel] = amplitude;
    ad9959_diagnostics.last_channel = (uint8_t)channel;
    return ad9959_status_ok;
}

/**
 * @brief 通过 SPI 读回 AD9959 寄存器值。
 * @param address 寄存器地址（0x00-0x06）。
 * @param data 存放读回数据的缓冲区，调用者保证容量不少于 length 字节。
 * @param length 要读的字节数，范围 1 至 AD9959_READ_MAX_BYTES。
 * @return 驱动状态。
 * @note 用于数字锁相环闭环验证。指令字节 bit7=1 表示读，
 *       随后通过 MISO 读回数据；读操作不产生 IO_UPDATE。
 */
ad9959_status_t ad9959_read_register(uint8_t address, uint8_t *data,
                                     uint8_t length)
{
    if (data == (uint8_t *)0)
    {
        return ad9959_status_invalid_length;
    }
    if ((length == 0u) || (length > AD9959_READ_MAX_BYTES))
    {
        return ad9959_status_invalid_length;
    }
    if (address > AD9959_REG_ACR)
    {
        return ad9959_status_invalid_length;
    }

    return ad9959_read_register_raw(address, data, length);
}
