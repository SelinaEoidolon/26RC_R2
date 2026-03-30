#include "bsp_uart.h"
#include "CRC.h"
#include "usart.h"

extern uint8_t btReceiveData;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

		if (huart->Instance == USART10)
    {
        UART10_Receive(btReceiveData);

        if (HAL_UART_Receive_IT(&huart10, &btReceiveData, 1) != HAL_OK)
        {
            Error_Handler();
        }
    }
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{

		if (huart->Instance == USART10)
    {
        /* 错误后强制重装载中断 */
        HAL_UART_Receive_IT(&huart10, &btReceiveData, 1);
    }
}
