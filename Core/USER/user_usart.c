#include "user_usart.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

HAL_StatusTypeDef Usart_Send_Computer(UART_HandleTypeDef *huart,
                                      const char *msg)
{
    size_t remaining;

    if ((huart == NULL) || (msg == NULL)) {
        return HAL_ERROR;
    }

    remaining = strlen(msg);
    while (remaining > 0U) {
        uint16_t chunk = (remaining > UINT16_MAX)
                             ? UINT16_MAX
                             : (uint16_t)remaining;
        HAL_StatusTypeDef status = HAL_UART_Transmit(
            huart, (uint8_t *)(void *)msg, chunk, 1000U);

        if (status != HAL_OK) {
            return status;
        }
        msg += chunk;
        remaining -= chunk;
    }

    return HAL_OK;
}
