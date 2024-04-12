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
#include "bsp_imu.h"
#include "tool.h"
#include "judgeSys.h"
#include "math.h"

extern float yaw_angle, pit_angle;
extern float yaw_angle_now, pit_angle_now;
extern volatile uint8_t shoot_flag;

float cacred = 0.0f;
float look_angle = 0.0f;
uint16_t last_HP = 0;
uint8_t send_cacre[100] = { 0 };
uint16_t wating = 0;//暂时丢失计时
float target = 0;
void visionSys_Task(void const *argument)
{

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    osDelay(100);
    SendPacketRobotStatus.header = 0xA5;
    SendPacketAllRobotHP.header = 0xA5;
    SendPacketGameStatus.header = 0xA5;
    last_HP = robot_status.current_HP;
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
                yaw_angle = yaw2 + PI + 0.05f;

                if (visioning_flag == 1)
                {
                    visioning_flag_shoot = 1;
                }
                else
                {
                    visioning_flag_shoot = 0;
                }
                wating = 0;
            }
            else
            {
                if (visioning_flag == 1)
                {
                    wating++;
                    if (wating >= 500)
                    {
                        pit_angle = 6200+;
                        wating = 500;
                        auto_Aim();

                    }


                }
                // auto_Aim();
                visioning_flag_shoot = 0;



            }


            HAL_UART_Receive_DMA(&huart6, Com4_newVision_Buffer, VISION_BUFFER_LEN);
            if (time_count++ >= 10)
            {
                visioning_flag = 0;
                time_count = 10;
            }





            break;

        default:
            break;
        }

        vTaskDelayUntil(&xLastWakeTime, 10);
    }
}

