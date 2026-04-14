#include "bsp_uart.h"
#include "imu.h"
#include "CRC.h"
#include "usart.h"

extern uint8_t imu_rx_byte;
extern uint8_t btReceiveData;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART7)
    {
        /* 1. 将接收到的字节传给Wit SDK解析 */
        WitSerialDataIn(imu_rx_byte);

        /* 2. 重新开启接收中断（核心：保证持续接收） */
        if (HAL_UART_Receive_IT(&huart7, &imu_rx_byte, 1) != HAL_OK)
        {
            Error_Handler();
        }
    }
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
    if (huart->Instance == UART7)
    {
        /* 错误后强制重装载中断 */
        HAL_UART_Receive_IT(&huart7, &imu_rx_byte, 1);
    }
		if (huart->Instance == USART10)
    {
        /* 错误后强制重装载中断 */
        HAL_UART_Receive_IT(&huart10, &btReceiveData, 1);
    }
}
