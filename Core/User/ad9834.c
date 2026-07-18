/**
 * @file ad9834.c
 * @brief AD9834 DDS底层驱动实现。
 *
 * 模块用途：使用SPI2和手动FSYNC完成AD9834寄存器写入。
 * GPIO引脚映射：PB12/FSYNC，PB13/SPI2_SCK，PB15/SPI2_MOSI。
 * 依赖的外设和CubeIDE配置：见ad9834.h。
 * 初始化方法：由dds_control_init()调用ad9834_init()。
 * 调用方法：主循环通过dds_control间接调用本模块。
 */

#include "system.h"

#define AD9834_CONTROL_RESET 0x2100u
#define AD9834_CONTROL_RUN   0x2000u
#define AD9834_FREQ0_ADDRESS 0x4000u
#define AD9834_PHASE0_ZERO   0xC000u
#define AD9834_SPI_TIMEOUT_MS 10u

/** AD9834运行诊断快照。 */
volatile ad9834_diagnostics_t ad9834_diagnostics;

/**
 * @brief 向AD9834发送一个16位字。
 * @param word 要发送的控制字或数据字。
 * @return 驱动状态。
 * @note 无论SPI成功或失败，退出前都会恢复FSYNC高电平。
 */
static ad9834_status_t ad9834_write_word(uint16_t word)
{
    HAL_StatusTypeDef hal_status;

    ad9834_diagnostics.last_word = word;
    HAL_GPIO_WritePin(DDS_FSYNC_GPIO_Port, DDS_FSYNC_Pin, GPIO_PIN_RESET);
    hal_status = HAL_SPI_Transmit(&hspi2, (const uint8_t *)&word, 1u,
                                  AD9834_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(DDS_FSYNC_GPIO_Port, DDS_FSYNC_Pin, GPIO_PIN_SET);

    ad9834_diagnostics.last_hal_status = (int32_t)hal_status;
    if (hal_status != HAL_OK)
    {
        ad9834_diagnostics.error_count++;
        return ad9834_status_spi_error;
    }

    ad9834_diagnostics.write_count++;
    return ad9834_status_ok;
}

/**
 * @brief 计算AD9834的28位频率控制字。
 * @param frequency_hz 目标输出频率，单位Hz。
 * @return 四舍五入后的28位频率控制字。
 */
uint32_t ad9834_calculate_tuning_word(uint32_t frequency_hz)
{
    return (uint32_t)(((((uint64_t)frequency_hz) << 28)
                       + (AD9834_MCLK_HZ / 2u)) / AD9834_MCLK_HZ);
}

/**
 * @brief 更新AD9834的FREQ0输出频率。
 * @param frequency_hz 目标输出频率，单位Hz。
 * @return 驱动状态。
 * @note 成功写入低14位和高14位后才更新诊断中的输出频率。
 */
ad9834_status_t ad9834_set_frequency_hz(uint32_t frequency_hz)
{
    uint32_t tuning_word;
    uint16_t low_word;
    uint16_t high_word;
    ad9834_status_t status;

    if ((frequency_hz == 0u) || (frequency_hz > AD9834_MAX_OUTPUT_HZ))
    {
        return ad9834_status_invalid_frequency;
    }

    tuning_word = ad9834_calculate_tuning_word(frequency_hz);
    low_word = (uint16_t)(AD9834_FREQ0_ADDRESS | (tuning_word & 0x3FFFu));
    high_word = (uint16_t)(AD9834_FREQ0_ADDRESS
                           | ((tuning_word >> 14) & 0x3FFFu));

    status = ad9834_write_word(low_word);
    if (status != ad9834_status_ok)
    {
        return status;
    }
    status = ad9834_write_word(high_word);
    if (status != ad9834_status_ok)
    {
        return status;
    }

    ad9834_diagnostics.output_frequency_hz = frequency_hz;
    ad9834_diagnostics.tuning_word = tuning_word;
    return ad9834_status_ok;
}

/**
 * @brief 初始化AD9834并输出指定频率的正弦波。
 * @param initial_frequency_hz 初始输出频率，单位Hz。
 * @return 驱动状态。
 * @note 写入顺序为软件复位、FREQ0、PHASE0、退出复位。
 */
ad9834_status_t ad9834_init(uint32_t initial_frequency_hz)
{
    ad9834_status_t status;

    ad9834_diagnostics.output_frequency_hz = 0u;
    ad9834_diagnostics.tuning_word = 0u;
    ad9834_diagnostics.write_count = 0u;
    ad9834_diagnostics.error_count = 0u;
    ad9834_diagnostics.last_word = 0u;
    ad9834_diagnostics.last_hal_status = (int32_t)HAL_OK;
    ad9834_diagnostics.initialized = 0u;
    HAL_GPIO_WritePin(DDS_FSYNC_GPIO_Port, DDS_FSYNC_Pin, GPIO_PIN_SET);

    status = ad9834_write_word(AD9834_CONTROL_RESET);
    if (status != ad9834_status_ok)
    {
        return status;
    }
    status = ad9834_set_frequency_hz(initial_frequency_hz);
    if (status != ad9834_status_ok)
    {
        return status;
    }
    status = ad9834_write_word(AD9834_PHASE0_ZERO);
    if (status != ad9834_status_ok)
    {
        return status;
    }
    status = ad9834_write_word(AD9834_CONTROL_RUN);
    if (status != ad9834_status_ok)
    {
        return status;
    }

    ad9834_diagnostics.initialized = 1u;
    return ad9834_status_ok;
}
