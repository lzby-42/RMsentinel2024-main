/**
 * @file visionSys_Task.C
 * @author DreamKerman (2829478110@qq.com)
 * @brief 视觉处理线程
 * @version 0.1
 * @date 2023-12-24
 *
 * @copyright 鼎行双创 (c) 2023
 *
 */

#include "visionSys_Task.h"
#include "main.h"

#include "cmsis_os.h"
#include "config.h"
#include "Vision.h"
#include "newVision.h"
#include "usart.h"
#include "newVision.h"
#include "bsp_imu.h"
#include "tool.h"

extern float yaw_angle, pit_angle;
extern float yaw_angle_now, pit_angle_now;

float cacred = 0.0f;
float look_angle = 0.0f;
void visionSys_Task(void const *argument)
{
    // uint16_t i = 0;
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    osDelay(100);

    for (;;)
    {


        switch (VISION_VERSION)
        {
        case 0:
            vinion_old_Version();
            break;
        case 1:

            vision_new_version();
            if (ReceivedPacketVision.tracking == true)
            {
                pit_angle = 6150 - (b_set_angle * 8191 / PI / 2.0f);
                yaw_angle = yaw2+PI;
                // cacred = (SendPacketVision.yaw - ReceivedPacketVision.yaw);
                // if (cacred > 2 * PI)
                // {
                //     cacred -= 2 * PI;
                // }
                // else if (cacred < -2 * PI)
                // {
                //     cacred += 2 * PI;
                // }
                // look_angle = (-imu.yaw) * (float)PI / 180.0f + cacred;
                
                // LimitMax(cacred, 0.15f);
                // yaw_angle = (-imu.yaw + 180) * (float)PI / 180.0f - cacred;

            }else
            {
                pit_angle = 6150;

            }


            HAL_UART_Receive_DMA(&huart6, Com4_newVision_Buffer, VISION_BUFFER_LEN);



            break;

        default:
            break;
        }

        vTaskDelayUntil(&xLastWakeTime, 10);
    }
}

