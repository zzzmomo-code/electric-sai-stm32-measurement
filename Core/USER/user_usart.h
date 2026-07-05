#ifndef USER_USART_H
#define USER_USART_H

#include "stm32f4xx_hal.h"

HAL_StatusTypeDef Usart_Send_Computer(UART_HandleTypeDef *huart,
                                      const char *msg);

#endif
