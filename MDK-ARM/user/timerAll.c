#include "timerAll.h"
#include "canmotor.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "heat.h" 
#include "bsp_imu.h"
#include "judgeSys.h"
#include "tim.h"
#include "pid.h"
#include "tool.h"
extern osThreadId gimbalHandle;
extern osThreadId chassisHandle;
extern osThreadId visionSysHandle;
extern osThreadId Moto_Shoot1Handle;
extern osThreadId Moto_Shoot2Handle;
extern osTimerId saferHandle;

extern pid_type_def yaw_angle_pid;
extern pid_type_def pit_angle_pid;
extern float yaw_angle, pit_angle;
uint16_t timer_count = 0;
void TIM_Self_CallBack(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        // 定时器6中断处理函数
        if (robot_status.power_management_chassis_output == 0)
        {
            if (eTaskGetState(chassisHandle) != eSuspended)
            {
                vTaskSuspend(chassisHandle);
            }
            pid_chassis(50, 0.048f, 0.046f, 0, 0);

        }
        else if (robot_status.power_management_chassis_output == 1 && eTaskGetState(chassisHandle) == eSuspended)
        {
            vTaskResume(chassisHandle);
        }
        if (robot_status.power_management_gimbal_output == 0)
        {
            if (eTaskGetState(gimbalHandle) != eSuspended)
            {
                vTaskSuspend(gimbalHandle);
                PID_Init(&yaw_angle_pid, -6.690547472340761, -0.001550956007973143, -111.792520982351, 15.6f, 15.6f);
                PID_Init(&pit_angle_pid, 60000, 50, 300, 16000.0f, 1000.0f);

            }
            yaw_angle = (-imu.yaw + 180) * (float)Pi / 180.0f;
            pit_angle = 6300 / 8189 * 2 * Pi;
        }
        else if (robot_status.power_management_gimbal_output == 1 && eTaskGetState(gimbalHandle) == eSuspended)
        {
            vTaskResume(gimbalHandle);
        }


    }
    if (htim->Instance == TIM7)
    {
        // 定时器7中断处理函数
        
    }

}