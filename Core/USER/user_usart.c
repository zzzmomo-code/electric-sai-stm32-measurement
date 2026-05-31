#include "system.h"

void Usart_Send_Computer(UART_HandleTypeDef *huart, char* msg)
{
	HAL_UART_Transmit(huart,(uint8_t*)msg,strlen(msg),1000);
}
