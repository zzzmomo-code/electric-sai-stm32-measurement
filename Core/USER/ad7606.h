#ifndef AD7606_H
#define AD7606_H

#include "ad7606_state.h"
#include "main.h"

#define AD7606_SER_H()    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_SET)
#define AD7606_SER_L()    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_RESET)

#define AD7606_OS0_H()    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_SET)
#define AD7606_OS0_L()    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_RESET)
#define AD7606_OS1_H()    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_SET)
#define AD7606_OS1_L()    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_RESET)
#define AD7606_OS2_H()    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)
#define AD7606_OS2_L()    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)

#define AD7606_STBY_H()   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET)
#define AD7606_STBY_L()   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET)
#define AD7606_RESET_H()  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET)
#define AD7606_RESET_L()  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET)

#define AD7606_CO_A_H()   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET)
#define AD7606_CO_A_L()   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET)
#define AD7606_CO_B_H()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)
#define AD7606_CO_B_L()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)

#define AD7606_CS_H()     HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_SET)
#define AD7606_CS_L()     HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET)
#define AD7606_SCLK_H()   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET)
#define AD7606_SCLK_L()   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET)
#define AD7606_DB15_H()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET)
#define AD7606_DB15_L()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET)

#define AD7606_BUSY_GPIO_PORT  GPIOF
#define AD7606_BUSY_GPIO_PIN   GPIO_PIN_3
#define AD7606_BUSY_READ()     HAL_GPIO_ReadPin(AD7606_BUSY_GPIO_PORT, AD7606_BUSY_GPIO_PIN)

#define AD7606_DOUTA_GPIO_PORT GPIOF
#define AD7606_DOUTA_GPIO_PIN  GPIO_PIN_4
#define AD7606_DOUTA_READ()    HAL_GPIO_ReadPin(AD7606_DOUTA_GPIO_PORT, AD7606_DOUTA_GPIO_PIN)

#define AD7606_CH_NUM          8U
#define AD7606_BUSY_TIMEOUT_MS 2U

#define AD7606_OS_NO           0x00U
#define AD7606_OS_2X           0x01U
#define AD7606_OS_4X           0x02U
#define AD7606_OS_8X           0x03U
#define AD7606_OS_16X          0x04U
#define AD7606_OS_32X          0x05U
#define AD7606_OS_64X          0x06U

#define AD7606_V_RANGE         5.0f
#define AD7606_LSB_V           ((2.0f * AD7606_V_RANGE) / 65536.0f)

typedef enum {
    AD7606_OK = 0,
    AD7606_NOT_READY,
    AD7606_NULL_POINTER,
    AD7606_BUSY,
    AD7606_TIMEOUT,
    AD7606_INVALID_ARGUMENT
} ad7606_result_t;

ad7606_result_t AD7606_SetOS(uint8_t os_mode);
void AD7606_HW_Init(void);
void AD7606_Prog_Init(void);
ad7606_result_t AD7606_ConvStart(uint32_t now_ms);
void AD7606_Service(uint32_t now_ms);
void AD7606_Busy_IRQHandler(void);
bool AD7606_IsIdle(void);
bool AD7606_IsDataReady(void);
bool AD7606_TakeTimeout(void);
uint32_t AD7606_GetSpuriousEdgeCount(void);
void AD7606_Recover(void);
ad7606_result_t AD7606_ReadData(int16_t data[AD7606_CH_NUM]);
float AD7606_CodeToVolt(int16_t code);
void AD7606_CodeToVoltAll(const int16_t *raw, float *volt, uint8_t ch_num);
ad7606_result_t AD7606_Sample(int16_t raw[AD7606_CH_NUM],
                              float volt[AD7606_CH_NUM]);
void AD7606_ExitRegMode(void);

extern int16_t ad7606_adc_data[AD7606_CH_NUM];
extern float ad7606_voltage[AD7606_CH_NUM];

#endif
