#include"system.h"


void System_Init(void)
{
    AD7606_Prog_Init();
    Usart_Send_Computer(&huart3, "init ok\n");
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_RESET);
}
