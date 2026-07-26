/**
 * @file ad9959.c
 * @brief AD9959 DDS底层驱动实现。
 *
 * 模块用途：完成AD9959双通道的频率、相位、幅度寄存器写入与读回。当前诊断版本
 * 临时使用GPIO模拟串行时序，隔离STM32H7硬件SPI传输层；两路通道仍可独立配置。
 * GPIO引脚映射：PE2/SCLK，PE5/SDIO_2输入，PE6/SDIO_0输出，
 * PD4/IO_UPDATE，PD5/CS，PB4/RESET。
 * 模块固定电平：PDC、SDIO_3/SYNC_I/O及P0～P3必须从模块端接GND；
 * SDIO_1未使用。SDIO_3在单位串行模式下禁止浮空。
 * 依赖的外设和CubeIDE配置：SPI4仍保留为回退路径；诊断版本在初始化时将
 * PE2/PE5/PE6接管为普通GPIO并按Mode 0、MSB优先发送。CS低有效，RESET高有效复位，
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

/** AD9959可写寄存器的最大数据长度，不含1字节指令。 */
#define AD9959_WRITE_MAX_BYTES 4u

/** GPIO模拟串行每个半周期的保守延时循环数，确保示波器易于观察。 */
#define AD9959_BITBANG_DELAY_LOOPS 100u

/** AD9959串行时钟：原SPI4_SCK引脚PE2。 */
#define AD9959_SCLK_GPIO_Port GPIOE
#define AD9959_SCLK_Pin GPIO_PIN_2

/** AD9959三线模式读数据：SDIO_2连接PE5。 */
#define AD9959_SDIO2_GPIO_Port GPIOE
#define AD9959_SDIO2_Pin GPIO_PIN_5

/** AD9959三线模式写数据：SDIO_0连接PE6。 */
#define AD9959_SDIO0_GPIO_Port GPIOE
#define AD9959_SDIO0_Pin GPIO_PIN_6

/** 临时总线探针状态：等待下一次可观测脉冲。 */
#define AD9959_BUS_PROBE_STATE_WAITING 0u

/** 临时总线探针状态：CS已经拉低，正在保持可观测低电平。 */
#define AD9959_BUS_PROBE_STATE_CS_LOW 1u

/** ACR 寄存器中 bit12 启用手动幅度控制 */
#define AD9959_ACR_AMPLITUDE_ENABLE 0x10u

/** CSR[2:1]=01：单位串行三线模式，SDIO_0输入、SDIO_2输出。 */
#define AD9959_CSR_THREE_WIRE_MODE 0x02u

/** CSR 通道使能位叠加三线模式，bit4=CH0，bit5=CH1。 */
static const uint8_t ad9959_csr_channel_enable[2] = {
    0x10u | AD9959_CSR_THREE_WIRE_MODE,
    0x20u | AD9959_CSR_THREE_WIRE_MODE
};

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

/** 临时总线探针最近一次执行时刻，单位ms，仅由主循环访问。 */
static uint32_t ad9959_bus_probe_last_ms;

/** 临时总线探针最近一次拉低CS的时刻，单位ms，仅由主循环访问。 */
static uint32_t ad9959_bus_probe_cs_low_started_ms;

/** 临时总线探针状态，仅由主循环访问。 */
static uint8_t ad9959_bus_probe_state;

/**
 * @brief 为GPIO模拟串行时序提供与CPU频率无关的保守短延时。
 * @param 无。
 * @return 无。
 * @note volatile循环变量防止-O3删除循环；该诊断版本优先保证边沿清晰，不追求吞吐率。
 */
static void ad9959_bitbang_delay(void)
{
    volatile uint32_t loop;

    for (loop = 0u; loop < AD9959_BITBANG_DELAY_LOOPS; loop++)
    {
        __NOP();
    }
}

/**
 * @brief 保存GPIO模拟串行三根信号线的空闲电平快照。
 * @param 无。
 * @return 无。
 * @note 快照用于调试器确认PE2/PE6已经回到低电平，以及PE5当前实际输入电平。
 */
static void ad9959_update_serial_gpio_snapshot(void)
{
    __DSB();
    ad9959_diagnostics.bitbang_sclk_idle_odr =
        ((AD9959_SCLK_GPIO_Port->ODR & (uint32_t)AD9959_SCLK_Pin) != 0u)
        ? 1u : 0u;
    ad9959_diagnostics.bitbang_sdio0_idle_odr =
        ((AD9959_SDIO0_GPIO_Port->ODR & (uint32_t)AD9959_SDIO0_Pin) != 0u)
        ? 1u : 0u;
    ad9959_diagnostics.bitbang_sdio2_idle_idr =
        ((AD9959_SDIO2_GPIO_Port->IDR & (uint32_t)AD9959_SDIO2_Pin) != 0u)
        ? 1u : 0u;
}

/**
 * @brief 接管PE2/PE5/PE6并配置为AD9959 GPIO模拟串行接口。
 * @param 无。
 * @return 无。
 * @note PE2和PE6为推挽输出且空闲低；PE5为下拉输入。硬件SPI4在诊断期间关闭，
 *       但CubeMX配置和HAL回退代码保留，便于验证后恢复。
 */
static void ad9959_serial_gpio_init(void)
{
#if (AD9959_USE_GPIO_BITBANG != 0u)
    GPIO_InitTypeDef gpio_init = {0};

    __HAL_SPI_DISABLE(&hspi4);
    __HAL_RCC_GPIOE_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOE, AD9959_SCLK_Pin | AD9959_SDIO0_Pin,
                      GPIO_PIN_RESET);

    gpio_init.Pin = AD9959_SCLK_Pin | AD9959_SDIO0_Pin;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_PULLDOWN;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOE, &gpio_init);

    gpio_init.Pin = AD9959_SDIO2_Pin;
    gpio_init.Mode = GPIO_MODE_INPUT;
    gpio_init.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOE, &gpio_init);

    ad9959_diagnostics.bitbang_gpio_ready = 1u;
    ad9959_update_serial_gpio_snapshot();
#else
    ad9959_diagnostics.bitbang_gpio_ready = 0u;
#endif
}

/**
 * @brief 以Mode 0、MSB优先方式向AD9959发送一个字节。
 * @param value 要发送的8位数据。
 * @return 无。
 * @note 每一位严格执行SCLK低电平设置SDIO_0、SCLK上升沿锁存、再回到低电平。
 */
static void ad9959_bitbang_write_byte(uint8_t value)
{
    uint8_t bit_index;

    for (bit_index = 0u; bit_index < 8u; bit_index++)
    {
        HAL_GPIO_WritePin(AD9959_SCLK_GPIO_Port, AD9959_SCLK_Pin,
                          GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AD9959_SDIO0_GPIO_Port, AD9959_SDIO0_Pin,
                          ((value & 0x80u) != 0u)
                          ? GPIO_PIN_SET : GPIO_PIN_RESET);
        ad9959_bitbang_delay();
        HAL_GPIO_WritePin(AD9959_SCLK_GPIO_Port, AD9959_SCLK_Pin,
                          GPIO_PIN_SET);
        ad9959_bitbang_delay();
        value <<= 1u;
    }

    HAL_GPIO_WritePin(AD9959_SCLK_GPIO_Port, AD9959_SCLK_Pin,
                      GPIO_PIN_RESET);
    ad9959_diagnostics.bitbang_clock_edges += 8u;
}

/**
 * @brief 以Mode 0、MSB优先方式从AD9959的SDIO_2读取一个字节。
 * @param 无。
 * @return 读取到的8位数据。
 * @note AD9959在SCLK下降沿后输出读数据。本函数等待tDV后、在SCLK仍为低电平时采样PE5，
 *       再产生下一次上升沿；这样可避免寄存器最后一位在通信周期结束时释放为高阻后才被采样。
 */
static uint8_t ad9959_bitbang_read_byte(void)
{
    uint8_t bit_index;
    uint8_t value = 0u;

    for (bit_index = 0u; bit_index < 8u; bit_index++)
    {
        HAL_GPIO_WritePin(AD9959_SCLK_GPIO_Port, AD9959_SCLK_Pin,
                          GPIO_PIN_RESET);
        ad9959_bitbang_delay();
        value <<= 1u;
        if (HAL_GPIO_ReadPin(AD9959_SDIO2_GPIO_Port,
                            AD9959_SDIO2_Pin) == GPIO_PIN_SET)
        {
            value |= 1u;
        }
        HAL_GPIO_WritePin(AD9959_SCLK_GPIO_Port, AD9959_SCLK_Pin,
                          GPIO_PIN_SET);
        ad9959_bitbang_delay();
    }

    HAL_GPIO_WritePin(AD9959_SCLK_GPIO_Port, AD9959_SCLK_Pin,
                      GPIO_PIN_RESET);
    ad9959_diagnostics.bitbang_clock_edges += 8u;
    return value;
}

/**
 * @brief 产生 IO_UPDATE 上升沿，将影子寄存器内容加载到实际工作寄存器。
 * @param 无。
 * @return 无。
 * @note AD9959复位后PLL尚未启用，SYSCLK=25MHz且SYNC_CLK=6.25MHz，
 *       此时一个SYNC_CLK周期为160ns。数据手册要求IO_UPDATE高脉冲大于
 *       一个SYNC_CLK周期，因此诊断阶段固定保持40ms，使模块排针处的脉冲可直接观察，
 *       并记录PD4的ODR/IDR以区分软件命令、外部钳位和接线断点。
 */
static void ad9959_io_update(void)
{
    HAL_GPIO_WritePin(update9959_GPIO_Port, update9959_Pin, GPIO_PIN_RESET);
    __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP();
    HAL_GPIO_WritePin(update9959_GPIO_Port, update9959_Pin, GPIO_PIN_SET);
    __DSB();
    ad9959_diagnostics.io_update_high_odr =
        ((update9959_GPIO_Port->ODR & (uint32_t)update9959_Pin) != 0u)
        ? 1u : 0u;
    ad9959_diagnostics.io_update_high_idr =
        ((update9959_GPIO_Port->IDR & (uint32_t)update9959_Pin) != 0u)
        ? 1u : 0u;
    HAL_Delay(AD9959_IO_UPDATE_HIGH_MS);
    HAL_GPIO_WritePin(update9959_GPIO_Port, update9959_Pin, GPIO_PIN_RESET);
    __DSB();
    ad9959_diagnostics.io_update_low_odr =
        ((update9959_GPIO_Port->ODR & (uint32_t)update9959_Pin) != 0u)
        ? 1u : 0u;
    ad9959_diagnostics.io_update_low_idr =
        ((update9959_GPIO_Port->IDR & (uint32_t)update9959_Pin) != 0u)
        ? 1u : 0u;
    ad9959_diagnostics.io_update_count++;
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
 * @brief 通过当前串行传输层向 AD9959 写入一个寄存器。
 * @param address 寄存器地址（0x00-0x06）。
 * @param data 待写入数据缓冲区指针，按 MSB first 排列。
 * @param length 数据字节数。
 * @return 驱动状态。
 * @note 顺序：CS 拉低 -> 发指令字节（bit7=0 表示写）-> 发数据 -> CS 拉高 -> IO_UPDATE。
 *       无论传输成功或失败，退出前都会恢复 CS 为高。
 */
static ad9959_status_t ad9959_write_register(uint8_t address,
                                             const uint8_t *data,
                                             uint8_t length)
{
    HAL_StatusTypeDef hal_status;
    uint8_t frame[AD9959_WRITE_MAX_BYTES + 1u];
    uint8_t index;

    if ((data == (const uint8_t *)0)
        || (length == 0u)
        || (length > AD9959_WRITE_MAX_BYTES))
    {
        return ad9959_status_invalid_length;
    }

    frame[0] = (uint8_t)(address & 0x7Fu);
    for (index = 0u; index < length; index++)
    {
        frame[index + 1u] = data[index];
    }

    HAL_GPIO_WritePin(AD9959_CS_GPIO_Port, AD9959_CS_Pin, GPIO_PIN_RESET);
#if (AD9959_USE_GPIO_BITBANG != 0u)
    ad9959_bitbang_delay();
    for (index = 0u; index <= length; index++)
    {
        ad9959_bitbang_write_byte(frame[index]);
    }
    hal_status = HAL_OK;
#else
    /*
     * H7硬件SPI路径把指令和数据放进同一事务。CS低期间不拆分HAL调用，
     * 与已验证工程一致，也规避低SCLK下相邻EOT/CSTART事务的风险。
     */
    hal_status = HAL_SPI_Transmit(&hspi4, frame, (uint16_t)(length + 1u),
                                  AD9959_SPI_TIMEOUT_MS);
#endif
    HAL_GPIO_WritePin(AD9959_SCLK_GPIO_Port, AD9959_SCLK_Pin,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AD9959_SDIO0_GPIO_Port, AD9959_SDIO0_Pin,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AD9959_CS_GPIO_Port, AD9959_CS_Pin, GPIO_PIN_SET);
    ad9959_update_serial_gpio_snapshot();

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
 * @brief 通过当前串行传输层读回 AD9959 一个寄存器的值。
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
    uint8_t index;
#if (AD9959_USE_GPIO_BITBANG == 0u)
    uint8_t tx_frame[AD9959_READ_MAX_BYTES + 1u] = {0};
    uint8_t rx_frame[AD9959_READ_MAX_BYTES + 1u] = {0};
#endif

    HAL_GPIO_WritePin(AD9959_CS_GPIO_Port, AD9959_CS_Pin, GPIO_PIN_RESET);
#if (AD9959_USE_GPIO_BITBANG != 0u)
    ad9959_bitbang_delay();
    ad9959_bitbang_write_byte((uint8_t)(address | AD9959_READ_BIT));
    for (index = 0u; index < length; index++)
    {
        data[index] = ad9959_bitbang_read_byte();
    }
    hal_status = HAL_OK;
#else
    tx_frame[0] = (uint8_t)(address | AD9959_READ_BIT);
    hal_status = HAL_SPI_TransmitReceive(&hspi4, tx_frame, rx_frame,
                                         (uint16_t)(length + 1u),
                                         AD9959_SPI_TIMEOUT_MS);
    if (hal_status == HAL_OK)
    {
        for (index = 0u; index < length; index++)
        {
            data[index] = rx_frame[index + 1u];
        }
    }
#endif
    HAL_GPIO_WritePin(AD9959_SCLK_GPIO_Port, AD9959_SCLK_Pin,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AD9959_SDIO0_GPIO_Port, AD9959_SDIO0_Pin,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AD9959_CS_GPIO_Port, AD9959_CS_Pin, GPIO_PIN_SET);
    ad9959_update_serial_gpio_snapshot();

    ad9959_diagnostics.last_hal_status = (int32_t)hal_status;
    if (hal_status != HAL_OK)
    {
        ad9959_diagnostics.error_count++;
        return ad9959_status_spi_error;
    }

    return ad9959_status_ok;
}

/**
 * @brief 通过写 CSR 寄存器选择目标通道并保持SDIO_0/SDIO_2三线模式。
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

    /* 每次选择通道都必须同时保持CSR[2:1]=01，否则SDIO_2会恢复为高阻态。 */
    csr = ad9959_csr_channel_enable[channel];
    return ad9959_write_register(AD9959_REG_CSR, &csr, 1u);
}

/**
 * @brief 比较两段寄存器字节是否完全一致。
 * @param actual 实际读回数据。
 * @param expected 期望数据。
 * @param length 比较字节数。
 * @return 完全一致返回1，否则返回0。
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
 * @brief 在初始化末尾回读关键寄存器并保存原始快照。
 * @param 无。
 * @return 串行事务状态；读回内容不一致不会伪装成HAL错误，而由mismatch掩码报告。
 * @note 依次验证全局FR1、CH0/CH1的CFR、CFTW0和ACR，用于区分“MCU已发送”与
 *       “AD9959已接收”。该数字回读不能证明模拟输出幅度或波形正确。
 */
static ad9959_status_t ad9959_capture_init_readback(void)
{
    ad9959_status_t status;
    uint8_t fr1[3];
    uint8_t cfr[2][3];
    uint8_t ftw[2][4];
    uint8_t acr[2][3];
    uint8_t expected_ftw[2][4];
    uint8_t expected_acr[2][3];
    uint8_t channel;
    uint8_t index;
    uint8_t mismatch = 0u;

    for (channel = 0u; channel < 2u; channel++)
    {
        uint32_t tuning_word = ad9959_diagnostics.frequency_tuning_word[channel];

        expected_ftw[channel][0] = (uint8_t)(tuning_word >> 24u);
        expected_ftw[channel][1] = (uint8_t)(tuning_word >> 16u);
        expected_ftw[channel][2] = (uint8_t)(tuning_word >> 8u);
        expected_ftw[channel][3] = (uint8_t)tuning_word;
        expected_acr[channel][0] = 0x00u;
        expected_acr[channel][1] =
            (uint8_t)(AD9959_ACR_AMPLITUDE_ENABLE
                      | (uint8_t)(ad9959_diagnostics.amplitude[channel] >> 8u));
        expected_acr[channel][2] =
            (uint8_t)(ad9959_diagnostics.amplitude[channel] & 0xFFu);
    }

    status = ad9959_read_register_raw(AD9959_REG_FR1, fr1, 3u);
    if (status != ad9959_status_ok)
    {
        return status;
    }
    if (ad9959_bytes_equal(fr1, ad9959_fr1_default, 3u) == 0u)
    {
        mismatch |= AD9959_READBACK_MISMATCH_FR1;
    }

    for (channel = 0u; channel < 2u; channel++)
    {
        status = ad9959_select_channel((ad9959_channel_t)channel);
        if (status != ad9959_status_ok)
        {
            return status;
        }
        status = ad9959_read_register_raw(AD9959_REG_ACR, acr[channel], 3u);
        if (status != ad9959_status_ok)
        {
            return status;
        }

        status = ad9959_read_register_raw(AD9959_REG_CFR, cfr[channel], 3u);
        if (status != ad9959_status_ok)
        {
            return status;
        }
        status = ad9959_read_register_raw(AD9959_REG_CFTW0, ftw[channel], 4u);
        if (status != ad9959_status_ok)
        {
            return status;
        }

        if (ad9959_bytes_equal(cfr[channel], ad9959_cfr_default, 3u) == 0u)
        {
            mismatch |= (channel == 0u)
                      ? AD9959_READBACK_MISMATCH_CH0_CFR
                      : AD9959_READBACK_MISMATCH_CH1_CFR;
        }
        if (ad9959_bytes_equal(ftw[channel], expected_ftw[channel], 4u) == 0u)
        {
            mismatch |= (channel == 0u)
                      ? AD9959_READBACK_MISMATCH_CH0_FTW
                      : AD9959_READBACK_MISMATCH_CH1_FTW;
        }
        if (ad9959_bytes_equal(acr[channel], expected_acr[channel], 3u) == 0u)
        {
            mismatch |= (channel == 0u)
                      ? AD9959_READBACK_MISMATCH_CH0_ACR
                      : AD9959_READBACK_MISMATCH_CH1_ACR;
        }
    }

    for (index = 0u; index < 3u; index++)
    {
        ad9959_diagnostics.fr1_readback[index] = fr1[index];
        ad9959_diagnostics.cfr_readback[0][index] = cfr[0][index];
        ad9959_diagnostics.cfr_readback[1][index] = cfr[1][index];
        ad9959_diagnostics.acr_readback[0][index] = acr[0][index];
        ad9959_diagnostics.acr_readback[1][index] = acr[1][index];
    }
    for (index = 0u; index < 4u; index++)
    {
        ad9959_diagnostics.ftw_readback[0][index] = ftw[0][index];
        ad9959_diagnostics.ftw_readback[1][index] = ftw[1][index];
    }

    ad9959_diagnostics.readback_mismatch_mask = mismatch;
    ad9959_diagnostics.readback_complete = 1u;
    return ad9959_status_ok;
}

/**
 * @brief 计算 AD9959 的 32 位频率控制字。
 * @param frequency_hz 目标输出频率，单位Hz。
 * @return 按 500MHz MCLK 四舍五入后的 32 位频率控制字。
 */
uint32_t ad9959_calculate_tuning_word(uint32_t frequency_hz)
{
    /* 必须先完成 64 位除法再转为 32 位，否则分子提前截断会让常用频率字变成 0。 */
    return (uint32_t)(((((uint64_t)frequency_hz) << 32u)
                        + (AD9959_MCLK_HZ / 2u))
                       / AD9959_MCLK_HZ);
}

/**
 * @brief 初始化 AD9959 并输出默认 1MHz 正弦波。
 * @return 驱动状态。
 * @note 步骤：硬件复位 -> 写 FR1（PLL 20倍频）-> 等待 PLL 锁定 -> 写 FR2 ->
 *       两个通道分别写 CFR、默认频率、相位、幅度。任一传输失败立即返回错误，
 *       退出前保证 CS 为高；initialized 标志只在全部步骤成功后置 1。
 */
ad9959_status_t ad9959_init(void)
{
    ad9959_status_t status;
    uint8_t channel;
    uint8_t index;

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
    ad9959_diagnostics.readback_complete = 0u;
    ad9959_diagnostics.readback_mismatch_mask = 0u;
    ad9959_diagnostics.bus_probe_count = 0u;
    ad9959_diagnostics.bus_probe_last_hal_status = (int32_t)HAL_OK;
    ad9959_diagnostics.bus_probe_fr1[0] = 0u;
    ad9959_diagnostics.bus_probe_fr1[1] = 0u;
    ad9959_diagnostics.bus_probe_fr1[2] = 0u;
    ad9959_diagnostics.bus_probe_signature = AD9959_BUS_PROBE_SIGNATURE;
    ad9959_diagnostics.bitbang_clock_edges = 0u;
    ad9959_diagnostics.bitbang_gpio_ready = 0u;
    ad9959_diagnostics.bitbang_sclk_idle_odr = 0u;
    ad9959_diagnostics.bitbang_sdio0_idle_odr = 0u;
    ad9959_diagnostics.bitbang_sdio2_idle_idr = 0u;
    ad9959_diagnostics.bus_probe_cs_low_count = 0u;
    ad9959_diagnostics.bus_probe_cs_low_tick = 0u;
    ad9959_diagnostics.bus_probe_cs_high_tick = 0u;
    ad9959_diagnostics.bus_probe_state = AD9959_BUS_PROBE_STATE_WAITING;
    ad9959_diagnostics.bus_probe_cs_low_odr = 1u;
    ad9959_diagnostics.bus_probe_cs_low_idr = 1u;
    ad9959_diagnostics.bus_probe_cs_high_odr = 1u;
    ad9959_diagnostics.bus_probe_cs_high_idr = 1u;
    ad9959_diagnostics.io_update_count = 0u;
    ad9959_diagnostics.io_update_high_odr = 0u;
    ad9959_diagnostics.io_update_high_idr = 0u;
    ad9959_diagnostics.io_update_low_odr = 0u;
    ad9959_diagnostics.io_update_low_idr = 0u;
    ad9959_bus_probe_state = AD9959_BUS_PROBE_STATE_WAITING;
    ad9959_bus_probe_cs_low_started_ms = 0u;
    /* 让system_process()第一次调用时立即开始40ms低电平，避免上电后再等待一个周期。 */
    ad9959_bus_probe_last_ms = HAL_GetTick() - AD9959_BUS_PROBE_PERIOD_MS;
    for (index = 0u; index < 3u; index++)
    {
        ad9959_diagnostics.fr1_readback[index] = 0u;
        for (channel = 0u; channel < 2u; channel++)
        {
            ad9959_diagnostics.cfr_readback[channel][index] = 0u;
            ad9959_diagnostics.acr_readback[channel][index] = 0u;
        }
    }
    for (index = 0u; index < 4u; index++)
    {
        for (channel = 0u; channel < 2u; channel++)
        {
            ad9959_diagnostics.ftw_readback[channel][index] = 0u;
        }
    }

    /* 上电等待电源稳定：商家建议 500ms，让模块 5V LDO 和 25MHz 晶振充分稳定 */
    HAL_Delay(500);

    ad9959_serial_gpio_init();
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

    status = ad9959_capture_init_readback();
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
 * @brief 通过当前串行传输层读回 AD9959 寄存器值。
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

/**
 * @brief 周期发送固定SPI帧，便于示波器稳定触发并逐线检查总线。
 * @param 无。
 * @return 无，诊断结果写入ad9959_diagnostics。
 * @note 每200ms先把CS保持低电平40ms，再发送00 12（CSR地址+CH0三线模式），
 *       随后在下一个CS低脉冲内发送81并读取3字节FR1。拉低/拉高后分别采样
 *       GPIOD ODR和IDR，用于区分固件、GPIO配置、外部电平冲突与测点问题。
 *       探针写入不计入write_count。
 */
void ad9959_bus_probe_process(void)
{
#if (AD9959_BUS_PROBE_ENABLE != 0u)
    HAL_StatusTypeDef hal_status;
    uint32_t now_ms = HAL_GetTick();
    uint8_t csr_frame[2] = {
        AD9959_REG_CSR,
        (uint8_t)(0x10u | AD9959_CSR_THREE_WIRE_MODE)
    };
    uint8_t read_header = (uint8_t)(AD9959_REG_FR1 | AD9959_READ_BIT);
#if (AD9959_USE_GPIO_BITBANG == 0u)
    uint8_t dummy[3] = { 0u, 0u, 0u };
#endif
    uint8_t readback[3] = { 0u, 0u, 0u };
    uint8_t index;

    if (ad9959_bus_probe_state == AD9959_BUS_PROBE_STATE_CS_LOW)
    {
        /* 在整个保持窗口内持续记录实际寄存器状态，确认PD5没有被其他代码改回高电平。 */
        ad9959_diagnostics.bus_probe_cs_low_odr =
            ((AD9959_CS_GPIO_Port->ODR & (uint32_t)AD9959_CS_Pin) != 0u)
            ? 1u : 0u;
        ad9959_diagnostics.bus_probe_cs_low_idr =
            ((AD9959_CS_GPIO_Port->IDR & (uint32_t)AD9959_CS_Pin) != 0u)
            ? 1u : 0u;

        if ((uint32_t)(now_ms - ad9959_bus_probe_cs_low_started_ms)
            < AD9959_BUS_PROBE_CS_LOW_MS)
        {
            return;
        }

        /* CS已经稳定保持低电平40ms；此时发送固定写帧，再恢复高电平。 */
#if (AD9959_USE_GPIO_BITBANG != 0u)
        ad9959_bitbang_write_byte(csr_frame[0]);
        ad9959_bitbang_write_byte(csr_frame[1]);
        hal_status = HAL_OK;
#else
        hal_status = HAL_SPI_Transmit(&hspi4, csr_frame, 2u,
                                      AD9959_SPI_TIMEOUT_MS);
#endif
        HAL_GPIO_WritePin(AD9959_SCLK_GPIO_Port, AD9959_SCLK_Pin,
                          GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AD9959_SDIO0_GPIO_Port, AD9959_SDIO0_Pin,
                          GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AD9959_CS_GPIO_Port, AD9959_CS_Pin, GPIO_PIN_SET);
        if (hal_status == HAL_OK)
        {
            ad9959_io_update();

            HAL_GPIO_WritePin(AD9959_CS_GPIO_Port, AD9959_CS_Pin,
                              GPIO_PIN_RESET);
#if (AD9959_USE_GPIO_BITBANG != 0u)
            ad9959_bitbang_delay();
            ad9959_bitbang_write_byte(read_header);
            for (index = 0u; index < 3u; index++)
            {
                readback[index] = ad9959_bitbang_read_byte();
            }
            hal_status = HAL_OK;
#else
            hal_status = HAL_SPI_Transmit(&hspi4, &read_header, 1u,
                                          AD9959_SPI_TIMEOUT_MS);
            if (hal_status == HAL_OK)
            {
                hal_status = HAL_SPI_TransmitReceive(&hspi4, dummy, readback,
                                                     3u,
                                                     AD9959_SPI_TIMEOUT_MS);
            }
#endif
            HAL_GPIO_WritePin(AD9959_SCLK_GPIO_Port, AD9959_SCLK_Pin,
                              GPIO_PIN_RESET);
            HAL_GPIO_WritePin(AD9959_SDIO0_GPIO_Port, AD9959_SDIO0_Pin,
                              GPIO_PIN_RESET);
            HAL_GPIO_WritePin(AD9959_CS_GPIO_Port, AD9959_CS_Pin,
                              GPIO_PIN_SET);
        }
        ad9959_update_serial_gpio_snapshot();

        /* 确保所有外设寄存器写入在采样ODR/IDR之前完成。 */
        __DSB();
        ad9959_diagnostics.bus_probe_cs_high_tick = HAL_GetTick();
        ad9959_diagnostics.bus_probe_cs_high_odr =
            ((AD9959_CS_GPIO_Port->ODR & (uint32_t)AD9959_CS_Pin) != 0u)
            ? 1u : 0u;
        ad9959_diagnostics.bus_probe_cs_high_idr =
            ((AD9959_CS_GPIO_Port->IDR & (uint32_t)AD9959_CS_Pin) != 0u)
            ? 1u : 0u;
        ad9959_bus_probe_state = AD9959_BUS_PROBE_STATE_WAITING;
        ad9959_diagnostics.bus_probe_state =
            AD9959_BUS_PROBE_STATE_WAITING;
        ad9959_diagnostics.bus_probe_last_hal_status = (int32_t)hal_status;
        ad9959_diagnostics.bus_probe_fr1[0] = readback[0];
        ad9959_diagnostics.bus_probe_fr1[1] = readback[1];
        ad9959_diagnostics.bus_probe_fr1[2] = readback[2];
        ad9959_diagnostics.bus_probe_count++;
        return;
    }

    if ((uint32_t)(now_ms - ad9959_bus_probe_last_ms)
        < AD9959_BUS_PROBE_PERIOD_MS)
    {
        return;
    }
    ad9959_bus_probe_last_ms = now_ms;

    /*
     * 先只拉低CS并立即返回主循环，下一阶段至少40ms后才发送SPI。
     * 这样PD5不是微秒脉冲，单次下降沿触发和普通万用表都能观察到。
     */
    HAL_GPIO_WritePin(AD9959_CS_GPIO_Port, AD9959_CS_Pin, GPIO_PIN_RESET);
    __DSB();
    ad9959_bus_probe_cs_low_started_ms = now_ms;
    ad9959_bus_probe_state = AD9959_BUS_PROBE_STATE_CS_LOW;
    ad9959_diagnostics.bus_probe_state = AD9959_BUS_PROBE_STATE_CS_LOW;
    ad9959_diagnostics.bus_probe_cs_low_tick = now_ms;
    ad9959_diagnostics.bus_probe_cs_low_odr =
        ((AD9959_CS_GPIO_Port->ODR & (uint32_t)AD9959_CS_Pin) != 0u)
        ? 1u : 0u;
    ad9959_diagnostics.bus_probe_cs_low_idr =
        ((AD9959_CS_GPIO_Port->IDR & (uint32_t)AD9959_CS_Pin) != 0u)
        ? 1u : 0u;
    ad9959_diagnostics.bus_probe_cs_low_count++;
#endif
}
