#ifndef __AD7606_H__
#define __AD7606_H__

#include "system.h"

#define AD7606_SER_H      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_SET)
#define AD7606_SER_L      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_RESET)

#define AD7606_OS0_H      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_SET)
#define AD7606_OS0_L      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, GPIO_PIN_RESET)

#define AD7606_OS1_H      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_SET)
#define AD7606_OS1_L      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_RESET)

#define AD7606_OS2_H      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)
#define AD7606_OS2_L      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)

#define AD7606_STBY_H      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET)
#define AD7606_STBY_L      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET)

#define AD7606_RESET_H       HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET)
#define AD7606_RESET_L       HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET)

#define AD7606_CO_A_H   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET)
#define AD7606_CO_A_L   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET)

#define AD7606_CO_B_H   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)
#define AD7606_CO_B_L   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)

#define AD7606_CS_H   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_SET)
#define AD7606_CS_L   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET)

#define AD7606_SCLK_H   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET)
#define AD7606_SCLK_L   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET)

#define AD7606_BUSY_GPIO_PORT   GPIOF
#define AD7606_BUSY_GPIO_PIN    GPIO_PIN_3
#define AD7606_BUSY_READ()      HAL_GPIO_ReadPin(AD7606_BUSY_GPIO_PORT, AD7606_BUSY_GPIO_PIN)

#define AD7606_DOUTA_GPIO_PORT  GPIOF
#define AD7606_DOUTA_GPIO_PIN   GPIO_PIN_4
#define AD7606_DOUTA_READ()     HAL_GPIO_ReadPin(AD7606_DOUTA_GPIO_PORT, AD7606_DOUTA_GPIO_PIN)

/*#define AD7606_DOUTB_H    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)
#define AD7606_DOUTB_L    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET)

#define AD7606_DOUTC_H    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)
#define AD7606_DOUTC_L    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)

#define AD7606_DOUTD_H    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)
#define AD7606_DOUTD_L    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)

#define AD7606_DOUTE_H    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)
#define AD7606_DOUTE_L    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)

#define AD7606_DOUTF_H    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)
#define AD7606_DOUTF_L    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)

#define AD7606_DOUTG_H    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)
#define AD7606_DOUTG_L    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)

#define AD7606_DOUTH_H    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)
#define AD7606_DOUTH_L    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)*/
#define AD7606_DB15_H HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
#define AD7606_DB15_L HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);


#ifndef AD7606_CH_NUM
#define AD7606_CH_NUM 8U
#endif

#define AD7606_OK             0U
#define AD7606_NOT_READY      1U
#define AD7606_NULL_POINTER   2U


/* 过采样模式，OS2/OS1/OS0 引脚组合 */
#define AD7606_OS_NO    0x00U  /* 无过采样 */
#define AD7606_OS_2X    0x01U  /* 2 倍过采样 */
#define AD7606_OS_4X    0x02U  /* 4 倍过采样 */
#define AD7606_OS_8X    0x03U  /* 8 倍过采样 */
#define AD7606_OS_16X   0x04U  /* 16 倍过采样 */
#define AD7606_OS_32X   0x05U  /* 32 倍过采样 */
#define AD7606_OS_64X   0x06U  /* 64 倍过采样 */

void AD7606_SetOS(uint8_t os_mode);
void AD7606_HW_Init(void);
void AD7606_Prog_Init(void);

void AD7606_ConvStart(void);
void AD7606_BusyPoll(void);
void AD7606_Busy_IRQHandler(void);
void AD7606_CompleteFlag_Clear(void);
uint8_t AD7606_ReadData(int16_t data[8]);
float AD7606_CodeToVolt(int16_t code);
void AD7606_CodeToVoltAll(const int16_t *raw, float *volt, uint8_t ch_num);
uint8_t AD7606_Sample(int16_t raw[8], float volt[8]);
void AD7606_ExitRegMode(void);

#define AD7606_CH_NUM     8U
/* 输入量程：±5V 用 5.0f，±10V 改为 10.0f（与硬件 RANGE 脚一致） */
#define AD7606_V_RANGE    5.0f
#define AD7606_LSB_V      (AD7606_V_RANGE / 65536.0f)

extern volatile uint8_t ad7606_complete_flag;
extern volatile uint8_t ad7606_conv_busy_flag;
extern int16_t ad7606_adc_data[AD7606_CH_NUM];
extern float ad7606_voltage[AD7606_CH_NUM];

#endif
