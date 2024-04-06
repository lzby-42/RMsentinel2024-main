#include "usart.h"
#include "string.h"
#include "Vision.h"
#include "newVision.h"
#include "judgeSys.h"
#include "radio.h"
#include "config.h"
extern DMA_HandleTypeDef hdma_uart8_rx;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
#if VISION_VERSION == 0
        memcpy(Vision_Buffer, Com4_Vision_Buffer, VISION_BUFFER_LENGTH);


        if (time_count > 5)
        {
            vision_read();
            last_yaw_angle = Data_yaw;
            last_pit_angle = Data_pit;
        }

        // Re-start DMA receive
        HAL_UART_Receive_DMA(&huart6, Com4_Vision_Buffer, VISION_BUFFER_LENGTH);
#endif
#if VISION_VERSION == 1
        memcpy(newVision_Buffer, Com4_newVision_Buffer, VISION_BUFFER_LEN);
        HAL_UART_Receive_DMA(&huart6, Com4_newVision_Buffer, VISION_BUFFER_LEN);
#endif
        time_count = 0;
        visioning_flag = 1;
    }
    else if (huart->Instance == UART7)
    {
        memcpy(&radio_data, &radio_data_cacre, RADIO_DATA_SIZE);
        HAL_UART_Receive_DMA(&huart7, radio_data_cacre, RADIO_DATA_SIZE);
    }
    else if (huart->Instance == UART8)
    {

        memcpy(judgment_System_handle_buffer, judgment_System_Signal_Buffer, JUDGEMENT_SIZE);


        judgment_Protect = 1;

        memset(judgment_System_Signal_Buffer, 0, JUDGEMENT_SIZE);
        HAL_UART_Receive_DMA(&huart8, judgment_System_Signal_Buffer, JUDGEMENT_SIZE);
    }
    else {
        // Do nothing
    }


}