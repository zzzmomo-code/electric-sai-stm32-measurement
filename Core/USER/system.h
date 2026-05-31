#ifndef __SYSTEM_H__
#define __SYSTEM_H__


//system
#include "main.h"
#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "usart.h"

//user
#include "user_usart.h"
#include  "ad7606.h"


void System_Init(void);
extern volatile uint8_t ad7606_complete_flag;
extern volatile uint8_t ad7606_conv_busy_flag;
#endif
