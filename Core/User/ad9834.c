/**
 * @file ad9834.c
 * @brief AD9834 DDS底层驱动实现。
 *
 * 模块用途：使用SPI2和手动FSYNC完成AD9834双频率、双相位寄存器写入。
 * GPIO引脚映射：PB12/FSYNC，PB13/SPI2_SCK，PB15/SPI2_MOSI，
 * PB14/FSELECT，PD8/PSELECT。
 * 依赖的外设和CubeIDE配置：见ad9834.h。
 * 初始化方法：由dds_control_init()调用ad9834_init()，随后默认选择FREQ0和PHASE0。
 * 调用方法：主循环可分别写入四个数据寄存器，再通过选择函数切换输出组合。
 */

#include "system.h"

#define AD9834_CONTROL_RESET 0x2300u
#define AD9834_CONTROL_RUN   0x2200u
#define AD9834_FREQ0_ADDRESS  0x4000u
#define AD9834_FREQ1_ADDRESS  0x8000u
#define AD9834_PHASE0_ADDRESS 0xC000u
#define AD9834_PHASE1_ADDRESS 0xE000u
#define AD9834_SPI_TIMEOUT_MS 10u

/** AD9834运行诊断快照。 */
volatile ad9834_diagnostics_t ad9834_diagnostics;

/**
 * @brief 通过PB14/FSELECT选择AD9834频率寄存器。
 * @param frequency_register 要选择的FREQ0或FREQ1。
 * @return 无。
 * @note 低电平选择FREQ0，高电平选择FREQ1；本函数不会写寄存器内容。
 */
void ad9834_select_frequency_register(
    ad9834_frequency_register_t frequency_register)
{
    GPIO_PinState pin_state;

    if ((frequency_register != ad9834_frequency_register_0)
        && (frequency_register != ad9834_frequency_register_1))
    {
        return;
    }

    pin_state = (frequency_register == ad9834_frequency_register_1)
                    ? GPIO_PIN_SET
                    : GPIO_PIN_RESET;

    HAL_GPIO_WritePin(FS_GPIO_Port, FS_Pin, pin_state);
    ad9834_diagnostics.selected_frequency_register =
        (uint8_t)(pin_state == GPIO_PIN_SET);
}

/**
 * @brief 通过PD8/PSELECT选择AD9834相位寄存器。
 * @param phase_register 要选择的PHASE0或PHASE1。
 * @return 无。
 * @note 低电平选择PHASE0，高电平选择PHASE1；本函数不会写寄存器内容。
 */
void ad9834_select_phase_register(ad9834_phase_register_t phase_register)
{
    GPIO_PinState pin_state;

    if ((phase_register != ad9834_phase_register_0)
        && (phase_register != ad9834_phase_register_1))
    {
        return;
    }

    pin_state = (phase_register == ad9834_phase_register_1)
                    ? GPIO_PIN_SET
                    : GPIO_PIN_RESET;

    HAL_GPIO_WritePin(PS_GPIO_Port, PS_Pin, pin_state);
    ad9834_diagnostics.selected_phase_register =
        (uint8_t)(pin_state == GPIO_PIN_SET);
}

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
 * @brief 将频率写入AD9834的指定频率寄存器。
 * @param frequency_register 目标FREQ0或FREQ1。
 * @param frequency_hz 目标频率，单位Hz。
 * @return 驱动状态。
 * @note 成功写入低14位和高14位后才更新对应寄存器诊断，不改变FSELECT。
 */
ad9834_status_t ad9834_set_frequency_register_hz(
    ad9834_frequency_register_t frequency_register,
    uint32_t frequency_hz)
{
    uint32_t tuning_word;
    uint16_t register_address;
    uint16_t low_word;
    uint16_t high_word;
    ad9834_status_t status;

    if ((frequency_register != ad9834_frequency_register_0)
        && (frequency_register != ad9834_frequency_register_1))
    {
        return ad9834_status_invalid_register;
    }
    if ((frequency_hz == 0u) || (frequency_hz > AD9834_MAX_OUTPUT_HZ))
    {
        return ad9834_status_invalid_frequency;
    }

    tuning_word = ad9834_calculate_tuning_word(frequency_hz);
    register_address = (frequency_register == ad9834_frequency_register_0)
                           ? AD9834_FREQ0_ADDRESS
                           : AD9834_FREQ1_ADDRESS;
    low_word = (uint16_t)(register_address | (tuning_word & 0x3FFFu));
    high_word = (uint16_t)(register_address
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
    ad9834_diagnostics.frequency_hz[frequency_register] = frequency_hz;
    ad9834_diagnostics.frequency_tuning_word[frequency_register] = tuning_word;
    ad9834_diagnostics.last_frequency_register = (uint8_t)frequency_register;
    return ad9834_status_ok;
}

/**
 * @brief 将整数角度写入AD9834的指定相位寄存器。
 * @param phase_register 目标PHASE0或PHASE1。
 * @param phase_degrees 目标相位，范围0至359度。
 * @return 驱动状态。
 * @note 成功发送一个相位数据字后才更新对应寄存器诊断，不改变PSELECT。
 */
ad9834_status_t ad9834_set_phase_register_degrees(
    ad9834_phase_register_t phase_register,
    uint16_t phase_degrees)
{
    uint16_t register_address;
    uint16_t phase_word;
    ad9834_status_t status;

    if ((phase_register != ad9834_phase_register_0)
        && (phase_register != ad9834_phase_register_1))
    {
        return ad9834_status_invalid_register;
    }
    if (phase_degrees > 359u)
    {
        return ad9834_status_invalid_phase;
    }

    register_address = (phase_register == ad9834_phase_register_0)
                           ? AD9834_PHASE0_ADDRESS
                           : AD9834_PHASE1_ADDRESS;
    phase_word = (uint16_t)((((uint32_t)phase_degrees * 4096u) + 180u)
                            / 360u);
    status = ad9834_write_word((uint16_t)(register_address | phase_word));
    if (status != ad9834_status_ok)
    {
        return status;
    }

    ad9834_diagnostics.phase_degrees[phase_register] = phase_degrees;
    ad9834_diagnostics.phase_word[phase_register] = phase_word;
    ad9834_diagnostics.last_phase_register = (uint8_t)phase_register;
    return ad9834_status_ok;
}

/**
 * @brief 更新AD9834的FREQ0输出频率。
 * @param frequency_hz 目标输出频率，单位Hz。
 * @return 驱动状态。
 * @note 保留原接口行为，内部固定写入FREQ0且不改变FSELECT。
 */
ad9834_status_t ad9834_set_frequency_hz(uint32_t frequency_hz)
{
    return ad9834_set_frequency_register_hz(
        ad9834_frequency_register_0,
        frequency_hz);
}

/**
 * @brief 初始化AD9834并输出指定频率的正弦波。
 * @param initial_frequency_hz 初始输出频率，单位Hz。
 * @return 驱动状态。
 * @note 复位期间初始化两组频率和相位，退出复位后选择FREQ0和PHASE0。
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
    ad9834_diagnostics.selected_frequency_register = 0u;
    ad9834_diagnostics.selected_phase_register = 0u;
    ad9834_diagnostics.frequency_hz[0] = 0u;
    ad9834_diagnostics.frequency_hz[1] = 0u;
    ad9834_diagnostics.frequency_tuning_word[0] = 0u;
    ad9834_diagnostics.frequency_tuning_word[1] = 0u;
    ad9834_diagnostics.phase_degrees[0] = 0u;
    ad9834_diagnostics.phase_degrees[1] = 0u;
    ad9834_diagnostics.phase_word[0] = 0u;
    ad9834_diagnostics.phase_word[1] = 0u;
    ad9834_diagnostics.last_frequency_register = 0u;
    ad9834_diagnostics.last_phase_register = 0u;
    HAL_GPIO_WritePin(DDS_FSYNC_GPIO_Port, DDS_FSYNC_Pin, GPIO_PIN_SET);
    ad9834_select_frequency_register(ad9834_frequency_register_0);
    ad9834_select_phase_register(ad9834_phase_register_0);

    status = ad9834_write_word(AD9834_CONTROL_RESET);
    if (status != ad9834_status_ok)
    {
        return status;
    }
    status = ad9834_set_frequency_register_hz(
        ad9834_frequency_register_0,
        initial_frequency_hz);
    if (status != ad9834_status_ok)
    {
        return status;
    }
    status = ad9834_set_frequency_register_hz(
        ad9834_frequency_register_1,
        initial_frequency_hz);
    if (status != ad9834_status_ok)
    {
        return status;
    }
    status = ad9834_set_phase_register_degrees(
        ad9834_phase_register_0,
        0u);
    if (status != ad9834_status_ok)
    {
        return status;
    }
    status = ad9834_set_phase_register_degrees(
        ad9834_phase_register_1,
        0u);
    if (status != ad9834_status_ok)
    {
        return status;
    }
    status = ad9834_write_word(AD9834_CONTROL_RUN);
    if (status != ad9834_status_ok)
    {
        return status;
    }

    ad9834_select_frequency_register(ad9834_frequency_register_0);
    ad9834_select_phase_register(ad9834_phase_register_0);
    ad9834_diagnostics.initialized = 1u;
    return ad9834_status_ok;
}


static ad9834_frequency_register_t active_reg =
    ad9834_frequency_register_0;


/**
 * @brief 使用双频率寄存器无中断地更新第一块AD9834输出频率。
 * @param frequency_hz 目标输出频率，单位Hz。
 * @return 驱动状态。
 * @note 先写入当前非活动频率寄存器，完整成功后才切换FSELECT；
 * 参数或SPI写入失败时保持当前输出不变，禁止在中断中调用。
 */
ad9834_status_t dds_set_frequency(uint32_t frequency_hz)
{
    ad9834_frequency_register_t inactive_reg;

    inactive_reg =
        (active_reg == ad9834_frequency_register_0)
        ? ad9834_frequency_register_1
        : ad9834_frequency_register_0;

    ad9834_status_t status =
        ad9834_set_frequency_register_hz(
            inactive_reg,
            frequency_hz);

    if (status != ad9834_status_ok)
    {
        return status;
    }

    ad9834_select_frequency_register(inactive_reg);

    active_reg = inactive_reg;

    return ad9834_status_ok;
}
