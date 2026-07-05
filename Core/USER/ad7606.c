#include "ad7606.h"

#include <stddef.h>

int16_t ad7606_adc_data[AD7606_CH_NUM] = {0};
float ad7606_voltage[AD7606_CH_NUM] = {0.0f};

static ad7606_state_ctx_t ad7606_state;

static uint32_t AD7606_Lock(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void AD7606_Unlock(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static void AD7606_DelayShort(void)
{
    volatile uint32_t i;

    for (i = 0U; i < 32U; i++) {
        __NOP();
    }
}

static void AD7606_ConvstHigh(void)
{
    AD7606_CO_A_H();
    AD7606_CO_B_H();
}

static void AD7606_ConvstLow(void)
{
    AD7606_CO_A_L();
    AD7606_CO_B_L();
}

static void AD7606_ResetPulse(void)
{
    AD7606_RESET_L();
    AD7606_DelayShort();
    AD7606_RESET_H();
    AD7606_DelayShort();
    AD7606_RESET_L();
    AD7606_DelayShort();
}

static int16_t AD7606_ReadWordDoutA(void)
{
    uint16_t value = 0U;
    uint8_t bit;

    for (bit = 0U; bit < 16U; bit++) {
        AD7606_SCLK_L();
        AD7606_DelayShort();
        value <<= 1U;
        if (AD7606_DOUTA_READ() == GPIO_PIN_SET) {
            value |= 1U;
        }
        AD7606_SCLK_H();
        AD7606_DelayShort();
    }

    return (int16_t)value;
}

void AD7606_HW_Init(void)
{
    uint32_t primask = AD7606_Lock();

    AD7606_StateInit(&ad7606_state);
    AD7606_Unlock(primask);

    AD7606_CS_H();
    AD7606_SCLK_H();
    AD7606_ConvstLow();
    AD7606_SER_H();
    AD7606_DB15_L();
    AD7606_OS0_L();
    AD7606_OS1_L();
    AD7606_OS2_L();
    AD7606_STBY_H();
    HAL_Delay(1U);
    AD7606_ResetPulse();
}

ad7606_result_t AD7606_SetOS(uint8_t os_mode)
{
    if (os_mode > AD7606_OS_64X) {
        return AD7606_INVALID_ARGUMENT;
    }

    if ((os_mode & 0x04U) != 0U) {
        AD7606_OS2_H();
    } else {
        AD7606_OS2_L();
    }
    if ((os_mode & 0x02U) != 0U) {
        AD7606_OS1_H();
    } else {
        AD7606_OS1_L();
    }
    if ((os_mode & 0x01U) != 0U) {
        AD7606_OS0_H();
    } else {
        AD7606_OS0_L();
    }

    return AD7606_OK;
}

void AD7606_Prog_Init(void)
{
    AD7606_HW_Init();
    (void)AD7606_SetOS(AD7606_OS_64X);
}

ad7606_result_t AD7606_ConvStart(uint32_t now_ms)
{
    uint32_t primask;
    bool started;
    bool busy_is_high;

    primask = AD7606_Lock();
    started = AD7606_StateStart(&ad7606_state, now_ms, false);
    AD7606_Unlock(primask);
    if (!started) {
        return AD7606_BUSY;
    }

    AD7606_ConvstLow();
    AD7606_DelayShort();
    AD7606_ConvstHigh();
    AD7606_DelayShort();
    AD7606_ConvstLow();

    busy_is_high = (AD7606_BUSY_READ() == GPIO_PIN_SET);
    primask = AD7606_Lock();
    AD7606_StateObserveBusy(&ad7606_state,
                            busy_is_high,
                            now_ms,
                            AD7606_BUSY_TIMEOUT_MS);
    AD7606_Unlock(primask);
    return AD7606_OK;
}

void AD7606_Service(uint32_t now_ms)
{
    bool busy_is_high = (AD7606_BUSY_READ() == GPIO_PIN_SET);
    uint32_t primask = AD7606_Lock();

    AD7606_StateObserveBusy(&ad7606_state,
                            busy_is_high,
                            now_ms,
                            AD7606_BUSY_TIMEOUT_MS);
    AD7606_Unlock(primask);
}

void AD7606_Busy_IRQHandler(void)
{
    bool busy_is_high = (AD7606_BUSY_READ() == GPIO_PIN_SET);
    uint32_t primask = AD7606_Lock();

    AD7606_StateOnFallingEdge(&ad7606_state, busy_is_high);
    AD7606_Unlock(primask);
}

bool AD7606_IsIdle(void)
{
    bool is_idle;
    uint32_t primask = AD7606_Lock();

    is_idle = (ad7606_state.state == AD7606_STATE_IDLE);
    AD7606_Unlock(primask);
    return is_idle;
}

bool AD7606_IsDataReady(void)
{
    bool is_ready;
    uint32_t primask = AD7606_Lock();

    is_ready = (ad7606_state.state == AD7606_STATE_DATA_READY);
    AD7606_Unlock(primask);
    return is_ready;
}

bool AD7606_TakeTimeout(void)
{
    bool timed_out;
    uint32_t primask = AD7606_Lock();

    timed_out = AD7606_StateTakeTimeout(&ad7606_state);
    AD7606_Unlock(primask);
    return timed_out;
}

uint32_t AD7606_GetSpuriousEdgeCount(void)
{
    uint32_t count;
    uint32_t primask = AD7606_Lock();

    count = ad7606_state.spurious_edge_count;
    AD7606_Unlock(primask);
    return count;
}

void AD7606_Recover(void)
{
    uint32_t primask;

    AD7606_CS_H();
    AD7606_SCLK_H();
    AD7606_ConvstLow();
    AD7606_ResetPulse();
    (void)AD7606_SetOS(AD7606_OS_64X);

    primask = AD7606_Lock();
    AD7606_StateRecover(&ad7606_state);
    AD7606_Unlock(primask);
}

ad7606_result_t AD7606_ReadData(int16_t data[AD7606_CH_NUM])
{
    uint32_t primask;
    uint8_t ch;
    bool ready;

    if (data == NULL) {
        return AD7606_NULL_POINTER;
    }

    primask = AD7606_Lock();
    ready = AD7606_StateTakeReady(&ad7606_state);
    AD7606_Unlock(primask);
    if (!ready) {
        return AD7606_NOT_READY;
    }

    AD7606_SCLK_H();
    AD7606_CS_L();
    AD7606_DelayShort();
    for (ch = 0U; ch < AD7606_CH_NUM; ch++) {
        data[ch] = AD7606_ReadWordDoutA();
        ad7606_adc_data[ch] = data[ch];
    }
    AD7606_CS_H();
    AD7606_DelayShort();

    primask = AD7606_Lock();
    AD7606_StateFinishRead(&ad7606_state);
    AD7606_Unlock(primask);
    return AD7606_OK;
}

float AD7606_CodeToVolt(int16_t code)
{
    return (float)code * AD7606_LSB_V;
}

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

ad7606_result_t AD7606_Sample(int16_t raw[AD7606_CH_NUM],
                              float volt[AD7606_CH_NUM])
{
    ad7606_result_t result;

    if ((raw == NULL) || (volt == NULL)) {
        return AD7606_NULL_POINTER;
    }

    AD7606_Service(HAL_GetTick());
    if (AD7606_TakeTimeout()) {
        return AD7606_TIMEOUT;
    }
    if (AD7606_IsIdle()) {
        result = AD7606_ConvStart(HAL_GetTick());
        return (result == AD7606_OK) ? AD7606_NOT_READY : result;
    }
    if (!AD7606_IsDataReady()) {
        return AD7606_NOT_READY;
    }

    result = AD7606_ReadData(raw);
    if (result == AD7606_OK) {
        AD7606_CodeToVoltAll(raw, volt, AD7606_CH_NUM);
    }
    return result;
}

void AD7606_ExitRegMode(void)
{
    AD7606_SER_H();
    AD7606_DB15_L();
}
