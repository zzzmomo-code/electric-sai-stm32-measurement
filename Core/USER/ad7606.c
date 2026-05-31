#include "ad7606.h"

#include <stddef.h>


volatile uint8_t ad7606_complete_flag = 0U;
int16_t ad7606_adc_data[AD7606_CH_NUM] = {0};
float ad7606_voltage[AD7606_CH_NUM] = {0.0f};

volatile uint8_t ad7606_conv_busy_flag = 0U;

/**
 * @brief Short delay for GPIO bit-bang timing.
 *
 * The AD7606 timing requirements for CONVST, CS, and SCLK are in ns.
 * This loop intentionally leaves margin for STM32 GPIO/HAL write latency.
 */
static void AD7606_DelayShort(void)
{
    volatile uint32_t i;

    for (i = 0U; i < 32U; i++) {
        __NOP();
    }
}


/**
 * @brief Drive optional CONVST B high if the board exposes it.
 */
static void AD7606_ConvstHigh(void)
{
    AD7606_CO_A_H;
    AD7606_CO_B_H;
}

/**
 * @brief Drive optional CONVST B low if the board exposes it.
 */
static void AD7606_ConvstLow(void)
{
    AD7606_CO_A_L;
    AD7606_CO_B_L;
}

/**
 * @brief Generate an AD7606 reset pulse.
 */
static void AD7606_ResetPulse(void)
{
    AD7606_RESET_L;
    AD7606_DelayShort();
    AD7606_RESET_H;
    AD7606_DelayShort();
    AD7606_RESET_L;
    AD7606_DelayShort();
}

/**
 * @brief Read one 16-bit channel word from DOUTA in software serial mode.
 *
 * @return Signed 16-bit ADC conversion code, MSB first.
 */
static int16_t AD7606_ReadWordDoutA(void)
{
    uint16_t value = 0U;
    uint8_t bit;

    for (bit = 0U; bit < 16U; bit++) {
        AD7606_SCLK_L;
        AD7606_DelayShort();
        value <<= 1U;
        if (AD7606_DOUTA_READ() == GPIO_PIN_SET) {
            value |= 1U;
        }

        AD7606_SCLK_H;
        AD7606_DelayShort();
    }

    return (int16_t)value;
}

/**
 * @brief Initialize AD7606 GPIO levels for software serial mode.
 *
 * Configuration:
 * - Serial interface: PAR/SER/BYTE SEL high.
 * - Software serial read: bit-banged CS/SCLK/DOUTA.
 * - Single DOUTA readout: DB15 low selects serial mode, DOUTA clocks all data.
 * - Normal operation: STBY high.
 * - Input range target: RANGE low for +/-5 V if AD7606_RANGE_L is defined.
 */
void AD7606_HW_Init(void)
{
    ad7606_complete_flag = 0U;
    ad7606_conv_busy_flag = 0U;

    AD7606_CS_H;
    AD7606_SCLK_H;
    AD7606_ConvstLow();

    AD7606_SER_H;
    AD7606_DB15_L;

    AD7606_OS0_L;
    AD7606_OS1_L;
    AD7606_OS2_L;

    AD7606_STBY_H;
    HAL_Delay(1U);
    AD7606_DB15_L;

    AD7606_ResetPulse();
}

/**
 * @brief Set AD7606 oversampling ratio via OS2/OS1/OS0 pins.
 *
 * @param os_mode Use AD7606_OS_* macros (AD7606_OS_NO to AD7606_OS_64X).
 */
void AD7606_SetOS(uint8_t os_mode)
{
    (os_mode & 0x04U) ? AD7606_OS2_H : AD7606_OS2_L;
    (os_mode & 0x02U) ? AD7606_OS1_H : AD7606_OS1_L;
    (os_mode & 0x01U) ? AD7606_OS0_H : AD7606_OS0_L;
}

/**
 * @brief Compatibility wrapper for project code that calls program init.
 */
void AD7606_Prog_Init(void)
{
    AD7606_HW_Init();
    AD7606_SetOS(AD7606_OS_64X);
}

/**
 * @brief Start one AD7606 conversion in +/-5 V range.
 *
 * This function does not wait for conversion completion. Call
 * AD7606_BusyPoll() in the while(1) loop, then call AD7606_ReadData()
 * when ad7606_complete_flag becomes nonzero.
 */
void AD7606_ConvStart(void)
{
    ad7606_complete_flag = 0U;
    ad7606_conv_busy_flag = 1U;

    AD7606_ConvstLow();
    AD7606_DelayShort();
    AD7606_ConvstHigh();
    AD7606_DelayShort();
    AD7606_ConvstLow();
}

/**
 * @brief Poll BUSY and set the completion flag when conversion has ended.
 *
 * Use this from the main while(1) loop when no RTOS/task scheduling is used.
 */
void AD7606_BusyPoll(void)
{
    if ((ad7606_conv_busy_flag != 0U) &&
        (AD7606_BUSY_READ() == GPIO_PIN_RESET)) {
        ad7606_conv_busy_flag = 0U;
        ad7606_complete_flag = 1U;
    }
}

/**
 * @brief Optional EXTI helper for a BUSY falling-edge interrupt.
 *
 * Keep the real ISR short and call this helper only after confirming that the
 * EXTI source is the AD7606 BUSY pin.
 */
void AD7606_Busy_IRQHandler(void)
{
    if (AD7606_BUSY_READ() == GPIO_PIN_RESET) {
        ad7606_conv_busy_flag = 0U;
        ad7606_complete_flag = 1U;
    }
}

/**
 * @brief Clear the conversion completion flag.
 */
void AD7606_CompleteFlag_Clear(void)
{
    ad7606_complete_flag = 0U;
}

/**
 * @brief Read all AD7606 channel results through the single DOUTA line.
 *
 * @param data Destination array with at least AD7606_CH_NUM elements.
 * @return AD7606_OK on success, AD7606_NOT_READY if conversion is not done,
 *         AD7606_NULL_POINTER if data is NULL.
 */
uint8_t AD7606_ReadData(int16_t data[8])
{
    uint8_t ch;

    if (data == NULL) {
        return AD7606_NULL_POINTER;
    }

    AD7606_BusyPoll();
    if (ad7606_complete_flag == 0U) {
        return AD7606_NOT_READY;
    }

    ad7606_complete_flag = 0U;

    AD7606_SCLK_H;
    AD7606_CS_L;
    AD7606_DelayShort();

    for (ch = 0U; ch < AD7606_CH_NUM; ch++) {
        data[ch] = AD7606_ReadWordDoutA();
        ad7606_adc_data[ch] = data[ch];
    }

    AD7606_CS_H;
    AD7606_DelayShort();

    return AD7606_OK;
}

/**
 * @brief Convert a signed AD7606 code to voltage.
 *
 * @param code Raw signed conversion code.
 * @return Voltage in volts for the selected bipolar range.
 */
float AD7606_CodeToVolt(int16_t code)
{
    return ((float)code * (2.0f * AD7606_V_RANGE)) / 65536.0f;
}

/**
 * @brief Convert multiple raw AD7606 codes to voltages.
 *
 * @param raw Source raw-code array.
 * @param volt Destination voltage array.
 * @param ch_num Number of channels to convert.
 */
void AD7606_CodeToVoltAll(const int16_t *raw, float *volt, uint8_t ch_num)
{
    uint8_t ch;

    if ((raw == NULL) || (volt == NULL)) {
        return;
    }

    for (ch = 0U; (ch < ch_num) && (ch < AD7606_CH_NUM); ch++) {
        volt[ch] = AD7606_CodeToVolt(raw[ch]);
    }
}

/**
 * @brief Nonblocking sample helper for a single-thread while(1) loop.
 *
 * Call this function repeatedly. It starts a conversion when idle and returns
 * AD7606_NOT_READY until BUSY has fallen and data has been read.
 */
uint8_t AD7606_Sample(int16_t raw[8], float volt[8])
{
    uint8_t ret;

    AD7606_BusyPoll();

    if ((ad7606_conv_busy_flag == 0U) && (ad7606_complete_flag == 0U)) {
        AD7606_ConvStart();
        return AD7606_NOT_READY;
    }

    if (ad7606_complete_flag == 0U) {
        return AD7606_NOT_READY;
    }

    ret = AD7606_ReadData(raw);
    if (ret != AD7606_OK) {
        return ret;
    }

    AD7606_CodeToVoltAll(raw, volt, AD7606_CH_NUM);
    return AD7606_OK;
}

/**
 * @brief Leave register/software configuration mode.
 *
 * The AD7606 data sheet used here selects operation by hardware pins. This
 * helper restores the same serial pin state used by AD7606_HW_Init().
 */
void AD7606_ExitRegMode(void)
{
    AD7606_SER_H;
    AD7606_DB15_L;
}
