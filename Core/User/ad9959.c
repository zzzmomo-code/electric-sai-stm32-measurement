/**
 * @file ad9959.c
 * @brief AD9959 双通道 DDS 驱动实现。
 *
 * 模块用途：使用一次完整 HAL SPI 事务发送“指令+数据”，完成 AD9959 寄存器
 * 写入、三线模式回读和初始化自检；仅启用 CH0、CH1。
 * GPIO 引脚映射：PE2/SPI4_SCK，PE5/SPI4_MISO 接 SDIO_2，
 * PE6/SPI4_MOSI 接 SDIO_0，PD5/CS，PD4/IO_UPDATE，PB4/RESET。
 * 模块固定电平：PDC、SDIO_3/SYNC_I/O、P0 至 P3 必须在模块端接 GND，
 * SDIO_1 不连接；模块使用独立 5 V 电源并与 STM32 共地。
 * 依赖的外设和 CubeIDE 配置：SPI4 全双工主机、Mode 0、8 位、MSB、
 * 软件 NSS、15 MHz、Master Keep IO State；无 DMA、无 SPI 中断。
 * 初始化方法：由 system_init() 在 MX_SPI4_Init() 之后调用 ad9959_init()。
 * 调用方法：在主循环或普通任务中设置频率、相位、幅度；禁止在中断中调用。
 */

#include "system.h"

#define AD9959_SPI_TIMEOUT_MS 20u
#define AD9959_MAX_REGISTER_BYTES 4u
#define AD9959_MAX_FRAME_BYTES (AD9959_MAX_REGISTER_BYTES + 1u)
#define AD9959_READ_INSTRUCTION 0x80u
#define AD9959_CSR_THREE_WIRE_MODE 0x02u
#define AD9959_PHASE_WORD_MAX 0x3FFFu
#define AD9959_CHANNEL_COUNT 2u
#define AD9959_PHYSICAL_CHANNEL_COUNT 4u

/** CSR 通道位和三线串行模式位，任何 CSR 写入都必须保留 bit1。 */
static const uint8_t ad9959_csr_channel[AD9959_PHYSICAL_CHANNEL_COUNT] = {
    0x12u, 0x22u, 0x42u, 0x82u
};

/** 选择全部通道并切换为单比特三线模式。 */
static const uint8_t ad9959_csr_all_channels = 0xF2u;

/** 25 MHz 参考时钟、PLL 20 倍频、VCO 高范围，得到 500 MHz 系统时钟。 */
static const uint8_t ad9959_fr1_expected[3] = {0xD0u, 0x00u, 0x00u};

/** FR2 保持数据手册默认功能。 */
static const uint8_t ad9959_fr2_expected[2] = {0x00u, 0x00u};

/** CH0、CH1：单音模式，DAC 和数字核正常工作。 */
static const uint8_t ad9959_cfr_output_enabled[3] = {0x00u, 0x03u, 0x02u};

/** CH2、CH3：CFR bit7/bit6 置位，关闭数字核和 DAC。 */
static const uint8_t ad9959_cfr_output_disabled[3] = {0x00u, 0x03u, 0xC2u};

/** AD9959 运行诊断快照，仅由驱动调用上下文修改。 */
volatile ad9959_diagnostics_t ad9959_diagnostics;

/**
 * @brief 使用 Cortex-M7 周期计数器产生微秒级忙等待。
 * @param microseconds 延时时间，单位微秒。
 * @return 无。
 * @note 只用于 SPI 帧间隔和控制脉冲，不在中断中调用。
 */
static void ad9959_delay_us(uint32_t microseconds)
{
    uint32_t cycles_per_us;
    uint32_t required_cycles;
    uint32_t start_cycles;

    if (microseconds == 0u)
    {
        return;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0u)
    {
        DWT->CYCCNT = 0u;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    cycles_per_us = SystemCoreClock / 1000000u;
    if (cycles_per_us == 0u)
    {
        cycles_per_us = 1u;
    }
    required_cycles = cycles_per_us * microseconds;
    start_cycles = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start_cycles) < required_cycles)
    {
        __NOP();
    }
}

/**
 * @brief 记录一次对外接口的最终状态。
 * @param status 即将返回给调用者的状态。
 * @return 原样返回 status。
 * @note 非成功状态只在对外接口边界累计一次错误。
 */
static ad9959_status_t ad9959_finish(ad9959_status_t status)
{
    ad9959_diagnostics.last_status = status;
    if (status != ad9959_status_ok)
    {
        ad9959_diagnostics.error_count++;
    }
    return status;
}

/**
 * @brief 返回寄存器规定的数据长度。
 * @param register_address 寄存器地址。
 * @return 合法寄存器的长度；0 表示当前最小驱动不支持。
 */
static uint8_t ad9959_register_length(ad9959_register_t register_address)
{
    static const uint8_t register_lengths[] = {1u, 3u, 2u, 3u, 4u, 2u, 3u};
    uint8_t address = (uint8_t)register_address;

    if (address >= (uint8_t)(sizeof(register_lengths) / sizeof(register_lengths[0])))
    {
        return 0u;
    }
    return register_lengths[address];
}

/**
 * @brief 比较两个寄存器字节序列。
 * @param actual 实际回读值。
 * @param expected 期望值。
 * @param length 字节数。
 * @return 完全相同返回 1，否则返回 0。
 */
static uint8_t ad9959_bytes_equal(const uint8_t *actual,
                                 const uint8_t *expected,
                                 uint8_t length)
{
    uint8_t index;

    for (index = 0u; index < length; index++)
    {
        if (actual[index] != expected[index])
        {
            return 0u;
        }
    }
    return 1u;
}

/**
 * @brief 在一次 CS 低电平窗口内发送完整 SPI 写帧。
 * @param frame 指令字节和数据字节组成的帧。
 * @param length 帧总长度。
 * @return 驱动状态。
 * @note 帧后等待时间同时满足 AD9959 CS 保持时间和 H743 SPI 连续启动勘误规避。
 */
static ad9959_status_t ad9959_spi_write_frame(const uint8_t *frame,
                                             uint8_t length)
{
    HAL_StatusTypeDef hal_status;

    ad9959_delay_us(1u);
    HAL_GPIO_WritePin(AD9959_CS_GPIO_Port, AD9959_CS_Pin, GPIO_PIN_RESET);
    ad9959_delay_us(1u);
    hal_status = HAL_SPI_Transmit(&hspi4, frame, length, AD9959_SPI_TIMEOUT_MS);
    ad9959_delay_us(1u);
    HAL_GPIO_WritePin(AD9959_CS_GPIO_Port, AD9959_CS_Pin, GPIO_PIN_SET);
    ad9959_delay_us(1u);

    ad9959_diagnostics.last_hal_status = (int32_t)hal_status;
    if (hal_status != HAL_OK)
    {
        return ad9959_status_spi_error;
    }
    return ad9959_status_ok;
}

/**
 * @brief 在一次 CS 低电平窗口内完成全双工 SPI 读帧。
 * @param transmit_data 指令和占位字节。
 * @param receive_data 接收缓冲区。
 * @param length 帧总长度。
 * @return 驱动状态。
 * @note 指令和数据使用同一次 HAL 调用，避免 CS 低期间产生两个 EOT/CSTART。
 */
static ad9959_status_t ad9959_spi_read_frame(const uint8_t *transmit_data,
                                            uint8_t *receive_data,
                                            uint8_t length)
{
    HAL_StatusTypeDef hal_status;

    ad9959_delay_us(1u);
    HAL_GPIO_WritePin(AD9959_CS_GPIO_Port, AD9959_CS_Pin, GPIO_PIN_RESET);
    ad9959_delay_us(1u);
    hal_status = HAL_SPI_TransmitReceive(&hspi4,
                                        transmit_data,
                                        receive_data,
                                        length,
                                        AD9959_SPI_TIMEOUT_MS);
    ad9959_delay_us(1u);
    HAL_GPIO_WritePin(AD9959_CS_GPIO_Port, AD9959_CS_Pin, GPIO_PIN_SET);
    ad9959_delay_us(1u);

    ad9959_diagnostics.last_hal_status = (int32_t)hal_status;
    if (hal_status != HAL_OK)
    {
        return ad9959_status_spi_error;
    }
    return ad9959_status_ok;
}

/**
 * @brief 写一个已知长度的 AD9959 寄存器。
 * @param register_address 寄存器地址。
 * @param data 待写数据。
 * @param length 数据字节数。
 * @return 驱动状态。
 */
static ad9959_status_t ad9959_write_register_raw(
    ad9959_register_t register_address,
    const uint8_t *data,
    uint8_t length)
{
    uint8_t frame[AD9959_MAX_FRAME_BYTES];
    uint8_t index;
    ad9959_status_t status;

    if ((data == NULL)
        || (length == 0u)
        || (length > AD9959_MAX_REGISTER_BYTES)
        || (ad9959_register_length(register_address) != length))
    {
        return ad9959_status_invalid_register;
    }

    frame[0] = (uint8_t)register_address;
    for (index = 0u; index < length; index++)
    {
        frame[index + 1u] = data[index];
    }

    status = ad9959_spi_write_frame(frame, (uint8_t)(length + 1u));
    if (status == ad9959_status_ok)
    {
        ad9959_diagnostics.write_count++;
    }
    return status;
}

/**
 * @brief 读取一个已知长度的 AD9959 寄存器。
 * @param register_address 寄存器地址。
 * @param data 接收数据。
 * @param length 数据字节数。
 * @return 驱动状态。
 */
static ad9959_status_t ad9959_read_register_raw(
    ad9959_register_t register_address,
    uint8_t *data,
    uint8_t length)
{
    uint8_t transmit_data[AD9959_MAX_FRAME_BYTES] = {0u};
    uint8_t receive_data[AD9959_MAX_FRAME_BYTES] = {0u};
    uint8_t index;
    ad9959_status_t status;

    if ((data == NULL)
        || (length == 0u)
        || (length > AD9959_MAX_REGISTER_BYTES)
        || (ad9959_register_length(register_address) != length))
    {
        return ad9959_status_invalid_register;
    }

    transmit_data[0] = (uint8_t)((uint8_t)register_address
                                 | AD9959_READ_INSTRUCTION);
    status = ad9959_spi_read_frame(transmit_data,
                                   receive_data,
                                   (uint8_t)(length + 1u));
    if (status != ad9959_status_ok)
    {
        return status;
    }

    for (index = 0u; index < length; index++)
    {
        data[index] = receive_data[index + 1u];
    }
    ad9959_diagnostics.read_count++;
    return ad9959_status_ok;
}

/**
 * @brief 选择一个物理通道并保持三线串行模式。
 * @param channel_index 物理通道编号 0 至 3。
 * @return 驱动状态。
 */
static ad9959_status_t ad9959_select_physical_channel(uint8_t channel_index)
{
    ad9959_status_t status;

    if (channel_index >= AD9959_PHYSICAL_CHANNEL_COUNT)
    {
        return ad9959_status_invalid_channel;
    }

    status = ad9959_write_register_raw(ad9959_register_csr,
                                       &ad9959_csr_channel[channel_index],
                                       1u);
    if (status == ad9959_status_ok)
    {
        ad9959_diagnostics.selected_channel = channel_index;
    }
    return status;
}

/**
 * @brief 产生满足首次 25 MHz 时钟条件的 IO_UPDATE 脉冲。
 * @param 无。
 * @return 无。
 */
static void ad9959_io_update(void)
{
    HAL_GPIO_WritePin(AD9959_IO_UPDATE_GPIO_Port,
                      AD9959_IO_UPDATE_Pin,
                      GPIO_PIN_SET);
    ad9959_delay_us(2u);
    HAL_GPIO_WritePin(AD9959_IO_UPDATE_GPIO_Port,
                      AD9959_IO_UPDATE_Pin,
                      GPIO_PIN_RESET);
    ad9959_delay_us(2u);
    ad9959_diagnostics.update_count++;
}

/**
 * @brief 产生 AD9959 高有效硬件复位脉冲。
 * @param 无。
 * @return 无。
 */
static void ad9959_hardware_reset(void)
{
    HAL_GPIO_WritePin(AD9959_CS_GPIO_Port, AD9959_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AD9959_IO_UPDATE_GPIO_Port,
                      AD9959_IO_UPDATE_Pin,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AD9959_RESET_GPIO_Port,
                      AD9959_RESET_Pin,
                      GPIO_PIN_RESET);
    HAL_Delay(10u);
    HAL_GPIO_WritePin(AD9959_RESET_GPIO_Port,
                      AD9959_RESET_Pin,
                      GPIO_PIN_SET);
    ad9959_delay_us(10u);
    HAL_GPIO_WritePin(AD9959_RESET_GPIO_Port,
                      AD9959_RESET_Pin,
                      GPIO_PIN_RESET);
    HAL_Delay(1u);
}

/**
 * @brief 将 32 位数值编码为大端字节序。
 * @param value 待编码数值。
 * @param data 四字节输出缓冲区。
 * @return 无。
 */
static void ad9959_encode_u32(uint32_t value, uint8_t data[4])
{
    data[0] = (uint8_t)(value >> 24);
    data[1] = (uint8_t)(value >> 16);
    data[2] = (uint8_t)(value >> 8);
    data[3] = (uint8_t)value;
}

/**
 * @brief 将 14 位相位字编码为大端字节序。
 * @param phase_word 14 位相位字。
 * @param data 两字节输出缓冲区。
 * @return 无。
 */
static void ad9959_encode_phase(uint16_t phase_word, uint8_t data[2])
{
    data[0] = (uint8_t)(phase_word >> 8);
    data[1] = (uint8_t)phase_word;
}

/**
 * @brief 将 10 位幅度值编码到 ACR 并使能手动幅度控制。
 * @param amplitude_scale 10 位幅度值。
 * @param data 三字节输出缓冲区。
 * @return 无。
 */
static void ad9959_encode_amplitude(uint16_t amplitude_scale, uint8_t data[3])
{
    data[0] = 0x00u;
    data[1] = (uint8_t)(0x10u | ((amplitude_scale >> 8) & 0x03u));
    data[2] = (uint8_t)amplitude_scale;
}

/**
 * @brief 不记录对外错误次数地完成一次完整配置回读。
 * @param 无。
 * @return 驱动状态。
 */
static ad9959_status_t ad9959_verify_configuration_internal(void)
{
    uint8_t channel_index;
    uint8_t ftw_expected[4];
    uint8_t phase_expected[2];
    uint8_t amplitude_expected[3];
    uint16_t mismatch_mask = 0u;
    ad9959_status_t status;

    ad9959_diagnostics.verified = 0u;
    ad9959_diagnostics.readback_mismatch_mask = 0u;

    status = ad9959_select_physical_channel(0u);
    if (status != ad9959_status_ok)
    {
        return status;
    }
    status = ad9959_read_register_raw(ad9959_register_fr1,
                                      (uint8_t *)ad9959_diagnostics.fr1_readback,
                                      3u);
    if (status != ad9959_status_ok)
    {
        return status;
    }
    if (ad9959_bytes_equal((const uint8_t *)ad9959_diagnostics.fr1_readback,
                           ad9959_fr1_expected,
                           3u) == 0u)
    {
        mismatch_mask |= AD9959_MISMATCH_FR1;
    }
    status = ad9959_read_register_raw(ad9959_register_fr2,
                                      (uint8_t *)ad9959_diagnostics.fr2_readback,
                                      2u);
    if (status != ad9959_status_ok)
    {
        return status;
    }
    if (ad9959_bytes_equal((const uint8_t *)ad9959_diagnostics.fr2_readback,
                           ad9959_fr2_expected,
                           2u) == 0u)
    {
        mismatch_mask |= AD9959_MISMATCH_FR2;
    }

    for (channel_index = 0u;
         channel_index < AD9959_CHANNEL_COUNT;
         channel_index++)
    {
        status = ad9959_select_physical_channel(channel_index);
        if (status != ad9959_status_ok)
        {
            return status;
        }
        status = ad9959_read_register_raw(
            ad9959_register_cfr,
            (uint8_t *)ad9959_diagnostics.cfr_readback[channel_index],
            3u);
        if (status != ad9959_status_ok)
        {
            return status;
        }
        status = ad9959_read_register_raw(
            ad9959_register_ftw0,
            (uint8_t *)ad9959_diagnostics.ftw_readback[channel_index],
            4u);
        if (status != ad9959_status_ok)
        {
            return status;
        }
        status = ad9959_read_register_raw(
            ad9959_register_cpow0,
            (uint8_t *)ad9959_diagnostics.cpow_readback[channel_index],
            2u);
        if (status != ad9959_status_ok)
        {
            return status;
        }
        status = ad9959_read_register_raw(
            ad9959_register_acr,
            (uint8_t *)ad9959_diagnostics.acr_readback[channel_index],
            3u);
        if (status != ad9959_status_ok)
        {
            return status;
        }

        if (ad9959_bytes_equal(
                (const uint8_t *)ad9959_diagnostics.cfr_readback[channel_index],
                ad9959_cfr_output_enabled,
                3u) == 0u)
        {
            mismatch_mask |= (channel_index == 0u)
                                 ? AD9959_MISMATCH_CH0_CFR
                                 : AD9959_MISMATCH_CH1_CFR;
        }

        ad9959_encode_u32(
            ad9959_diagnostics.frequency_tuning_word[channel_index],
            ftw_expected);
        if (ad9959_bytes_equal(
                (const uint8_t *)ad9959_diagnostics.ftw_readback[channel_index],
                ftw_expected,
                4u) == 0u)
        {
            mismatch_mask |= (channel_index == 0u)
                                 ? AD9959_MISMATCH_CH0_FTW
                                 : AD9959_MISMATCH_CH1_FTW;
        }

        ad9959_encode_phase(ad9959_diagnostics.phase_word[channel_index],
                            phase_expected);
        if (ad9959_bytes_equal(
                (const uint8_t *)ad9959_diagnostics.cpow_readback[channel_index],
                phase_expected,
                2u) == 0u)
        {
            mismatch_mask |= (channel_index == 0u)
                                 ? AD9959_MISMATCH_CH0_CPOW
                                 : AD9959_MISMATCH_CH1_CPOW;
        }

        ad9959_encode_amplitude(
            ad9959_diagnostics.amplitude_scale[channel_index],
            amplitude_expected);
        if (ad9959_bytes_equal(
                (const uint8_t *)ad9959_diagnostics.acr_readback[channel_index],
                amplitude_expected,
                3u) == 0u)
        {
            mismatch_mask |= (channel_index == 0u)
                                 ? AD9959_MISMATCH_CH0_ACR
                                 : AD9959_MISMATCH_CH1_ACR;
        }
    }

    for (channel_index = 2u;
         channel_index < AD9959_PHYSICAL_CHANNEL_COUNT;
         channel_index++)
    {
        status = ad9959_select_physical_channel(channel_index);
        if (status != ad9959_status_ok)
        {
            return status;
        }
        status = ad9959_read_register_raw(
            ad9959_register_cfr,
            (uint8_t *)ad9959_diagnostics.cfr_readback[channel_index],
            3u);
        if (status != ad9959_status_ok)
        {
            return status;
        }
        if (ad9959_bytes_equal(
                (const uint8_t *)ad9959_diagnostics.cfr_readback[channel_index],
                ad9959_cfr_output_disabled,
                3u) == 0u)
        {
            mismatch_mask |= (channel_index == 2u)
                                 ? AD9959_MISMATCH_CH2_CFR
                                 : AD9959_MISMATCH_CH3_CFR;
        }
    }

    ad9959_diagnostics.readback_mismatch_mask = mismatch_mask;
    ad9959_diagnostics.verified = (mismatch_mask == 0u) ? 1u : 0u;
    status = ad9959_select_physical_channel(0u);
    if (status != ad9959_status_ok)
    {
        ad9959_diagnostics.verified = 0u;
        return status;
    }
    return (mismatch_mask == 0u)
               ? ad9959_status_ok
               : ad9959_status_readback_mismatch;
}

uint32_t ad9959_calculate_tuning_word(uint32_t frequency_hz)
{
    uint64_t numerator;

    numerator = ((uint64_t)frequency_hz * UINT64_C(0x100000000))
                + (AD9959_SYSTEM_CLOCK_HZ / 2u);
    return (uint32_t)(numerator / AD9959_SYSTEM_CLOCK_HZ);
}

uint16_t ad9959_calculate_phase_word(uint16_t phase_degrees)
{
    uint32_t numerator;

    numerator = ((uint32_t)phase_degrees * 16384u) + 180u;
    return (uint16_t)(numerator / 360u);
}

ad9959_status_t ad9959_init(void)
{
    uint8_t channel_index;
    uint8_t ftw_data[4];
    uint8_t phase_data[2];
    uint8_t amplitude_data[3];
    uint32_t initial_ftw;
    ad9959_status_t status;

    ad9959_diagnostics = (ad9959_diagnostics_t){0};
    ad9959_diagnostics.last_hal_status = (int32_t)HAL_OK;
    ad9959_diagnostics.last_status = ad9959_status_not_initialized;

    ad9959_hardware_reset();

    status = ad9959_write_register_raw(ad9959_register_csr,
                                       &ad9959_csr_all_channels,
                                       1u);
    if (status != ad9959_status_ok)
    {
        return ad9959_finish(status);
    }
    status = ad9959_write_register_raw(ad9959_register_fr1,
                                       ad9959_fr1_expected,
                                       3u);
    if (status != ad9959_status_ok)
    {
        return ad9959_finish(status);
    }
    ad9959_io_update();
    HAL_Delay(5u);

    status = ad9959_write_register_raw(ad9959_register_fr2,
                                       ad9959_fr2_expected,
                                       2u);
    if (status != ad9959_status_ok)
    {
        return ad9959_finish(status);
    }

    initial_ftw = ad9959_calculate_tuning_word(
        AD9959_INITIAL_FREQUENCY_HZ);
    ad9959_encode_u32(initial_ftw, ftw_data);
    ad9959_encode_phase(0u, phase_data);
    ad9959_encode_amplitude(AD9959_AMPLITUDE_FULL_SCALE, amplitude_data);

    for (channel_index = 0u;
         channel_index < AD9959_CHANNEL_COUNT;
         channel_index++)
    {
        status = ad9959_select_physical_channel(channel_index);
        if (status != ad9959_status_ok)
        {
            return ad9959_finish(status);
        }
        status = ad9959_write_register_raw(ad9959_register_cfr,
                                           ad9959_cfr_output_enabled,
                                           3u);
        if (status != ad9959_status_ok)
        {
            return ad9959_finish(status);
        }
        status = ad9959_write_register_raw(ad9959_register_ftw0,
                                           ftw_data,
                                           4u);
        if (status != ad9959_status_ok)
        {
            return ad9959_finish(status);
        }
        status = ad9959_write_register_raw(ad9959_register_cpow0,
                                           phase_data,
                                           2u);
        if (status != ad9959_status_ok)
        {
            return ad9959_finish(status);
        }
        status = ad9959_write_register_raw(ad9959_register_acr,
                                           amplitude_data,
                                           3u);
        if (status != ad9959_status_ok)
        {
            return ad9959_finish(status);
        }

        ad9959_diagnostics.frequency_hz[channel_index] =
            AD9959_INITIAL_FREQUENCY_HZ;
        ad9959_diagnostics.frequency_tuning_word[channel_index] = initial_ftw;
        ad9959_diagnostics.phase_word[channel_index] = 0u;
        ad9959_diagnostics.amplitude_scale[channel_index] =
            AD9959_AMPLITUDE_FULL_SCALE;
    }

    for (channel_index = 2u;
         channel_index < AD9959_PHYSICAL_CHANNEL_COUNT;
         channel_index++)
    {
        status = ad9959_select_physical_channel(channel_index);
        if (status != ad9959_status_ok)
        {
            return ad9959_finish(status);
        }
        status = ad9959_write_register_raw(ad9959_register_cfr,
                                           ad9959_cfr_output_disabled,
                                           3u);
        if (status != ad9959_status_ok)
        {
            return ad9959_finish(status);
        }
    }

    ad9959_io_update();
    HAL_Delay(1u);

    status = ad9959_verify_configuration_internal();
    ad9959_diagnostics.initialized =
        (status == ad9959_status_ok) ? 1u : 0u;
    return ad9959_finish(status);
}

ad9959_status_t ad9959_set_frequency(ad9959_channel_t channel,
                                    uint32_t frequency_hz)
{
    uint8_t data[4];
    uint8_t readback[4];
    uint32_t tuning_word;
    ad9959_status_t status;

    if (ad9959_diagnostics.initialized == 0u)
    {
        return ad9959_finish(ad9959_status_not_initialized);
    }
    if ((uint8_t)channel >= AD9959_CHANNEL_COUNT)
    {
        return ad9959_finish(ad9959_status_invalid_channel);
    }
    if ((frequency_hz == 0u)
        || (frequency_hz > AD9959_MAX_OUTPUT_FREQUENCY_HZ))
    {
        return ad9959_finish(ad9959_status_invalid_frequency);
    }

    tuning_word = ad9959_calculate_tuning_word(frequency_hz);
    ad9959_encode_u32(tuning_word, data);
    status = ad9959_select_physical_channel((uint8_t)channel);
    if (status == ad9959_status_ok)
    {
        status = ad9959_write_register_raw(ad9959_register_ftw0, data, 4u);
    }
    if (status == ad9959_status_ok)
    {
        ad9959_io_update();
        status = ad9959_read_register_raw(ad9959_register_ftw0,
                                          readback,
                                          4u);
    }
    if ((status == ad9959_status_ok)
        && (ad9959_bytes_equal(readback, data, 4u) == 0u))
    {
        status = ad9959_status_readback_mismatch;
    }
    if (status == ad9959_status_ok)
    {
        ad9959_diagnostics.frequency_hz[channel] = frequency_hz;
        ad9959_diagnostics.frequency_tuning_word[channel] = tuning_word;
    }
    return ad9959_finish(status);
}

ad9959_status_t ad9959_set_phase_word(ad9959_channel_t channel,
                                     uint16_t phase_word)
{
    uint8_t data[2];
    uint8_t readback[2];
    ad9959_status_t status;

    if (ad9959_diagnostics.initialized == 0u)
    {
        return ad9959_finish(ad9959_status_not_initialized);
    }
    if ((uint8_t)channel >= AD9959_CHANNEL_COUNT)
    {
        return ad9959_finish(ad9959_status_invalid_channel);
    }
    if (phase_word > AD9959_PHASE_WORD_MAX)
    {
        return ad9959_finish(ad9959_status_invalid_phase);
    }

    ad9959_encode_phase(phase_word, data);
    status = ad9959_select_physical_channel((uint8_t)channel);
    if (status == ad9959_status_ok)
    {
        status = ad9959_write_register_raw(ad9959_register_cpow0, data, 2u);
    }
    if (status == ad9959_status_ok)
    {
        ad9959_io_update();
        status = ad9959_read_register_raw(ad9959_register_cpow0,
                                          readback,
                                          2u);
    }
    if ((status == ad9959_status_ok)
        && (ad9959_bytes_equal(readback, data, 2u) == 0u))
    {
        status = ad9959_status_readback_mismatch;
    }
    if (status == ad9959_status_ok)
    {
        ad9959_diagnostics.phase_word[channel] = phase_word;
    }
    return ad9959_finish(status);
}

ad9959_status_t ad9959_set_phase_degrees(ad9959_channel_t channel,
                                        uint16_t phase_degrees)
{
    if (phase_degrees >= 360u)
    {
        return ad9959_finish(ad9959_status_invalid_phase);
    }
    return ad9959_set_phase_word(
        channel,
        ad9959_calculate_phase_word(phase_degrees));
}

ad9959_status_t ad9959_set_amplitude(ad9959_channel_t channel,
                                    uint16_t amplitude_scale)
{
    uint8_t data[3];
    uint8_t readback[3];
    ad9959_status_t status;

    if (ad9959_diagnostics.initialized == 0u)
    {
        return ad9959_finish(ad9959_status_not_initialized);
    }
    if ((uint8_t)channel >= AD9959_CHANNEL_COUNT)
    {
        return ad9959_finish(ad9959_status_invalid_channel);
    }
    if (amplitude_scale > AD9959_AMPLITUDE_FULL_SCALE)
    {
        return ad9959_finish(ad9959_status_invalid_amplitude);
    }

    ad9959_encode_amplitude(amplitude_scale, data);
    status = ad9959_select_physical_channel((uint8_t)channel);
    if (status == ad9959_status_ok)
    {
        status = ad9959_write_register_raw(ad9959_register_acr, data, 3u);
    }
    if (status == ad9959_status_ok)
    {
        ad9959_io_update();
        status = ad9959_read_register_raw(ad9959_register_acr,
                                          readback,
                                          3u);
    }
    if ((status == ad9959_status_ok)
        && (ad9959_bytes_equal(readback, data, 3u) == 0u))
    {
        status = ad9959_status_readback_mismatch;
    }
    if (status == ad9959_status_ok)
    {
        ad9959_diagnostics.amplitude_scale[channel] = amplitude_scale;
    }
    return ad9959_finish(status);
}

ad9959_status_t ad9959_read_register(ad9959_channel_t channel,
                                    ad9959_register_t register_address,
                                    uint8_t *data,
                                    uint8_t length)
{
    ad9959_status_t status;

    if (ad9959_diagnostics.initialized == 0u)
    {
        return ad9959_finish(ad9959_status_not_initialized);
    }
    if ((uint8_t)channel >= AD9959_CHANNEL_COUNT)
    {
        return ad9959_finish(ad9959_status_invalid_channel);
    }
    if (data == NULL)
    {
        return ad9959_finish(ad9959_status_invalid_argument);
    }
    if (ad9959_register_length(register_address) != length)
    {
        return ad9959_finish(ad9959_status_invalid_register);
    }

    status = ad9959_select_physical_channel((uint8_t)channel);
    if (status == ad9959_status_ok)
    {
        status = ad9959_read_register_raw(register_address, data, length);
    }
    return ad9959_finish(status);
}

ad9959_status_t ad9959_verify_configuration(void)
{
    ad9959_status_t status;

    if (ad9959_diagnostics.initialized == 0u)
    {
        return ad9959_finish(ad9959_status_not_initialized);
    }
    status = ad9959_verify_configuration_internal();
    return ad9959_finish(status);
}
