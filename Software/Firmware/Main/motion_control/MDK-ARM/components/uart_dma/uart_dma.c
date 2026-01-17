#include "uart_dma.h"

uint8_t rx_buff[9]; 
void UART_DMA_Init(void)
{
    HAL_UART_Receive_DMA(&huart4, rx_buff, 9);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == UART4)
    {
        if(rx_buff[0] == 0xAA && rx_buff[1] == 0x55 && rx_buff[8] == 0xFF)
        {
            angle[0] = ((rx_buff[2] << 8) | rx_buff[3]) / 10.0f;
            angle[1] = ((rx_buff[4] << 8) | rx_buff[5]) / 10.0f;
            angle[2] = ((rx_buff[6] << 8) | rx_buff[7]) / 10.0f;
        }
        HAL_UART_Receive_DMA(&huart4, rx_buff, 9); 
    }
}