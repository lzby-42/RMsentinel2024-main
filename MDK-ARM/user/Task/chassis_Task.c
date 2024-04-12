/*********************************************************************
 * @file chassis_Task.C
 * @author DreamKerman(2829478110@qq.com)
 * @brief 底盘控制线程
 * @version 0.1
 * @date 2023-12-24
 *
 * @copyright 鼎行双创 (c) 2023
 * *******************************************************************
 */

#include "main.h"
#include "chassis_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "heat.h" 
#include "pid.h"
#include "canmotor.h"
#include "stdio.h"
#include "bsp_imu.h"
#include "tool.h"
#include "judgeSys.h"

extern  float out1, out2, out3, out4;
extern int motorval1, motorval2, motorval3, motorval4, motorvalleft, motorvalright;
extern imu_t   imu;
extern pid_type_def heatpid;
uint8_t textCut = 0;
float power_limit = 0.0f;
#define power_Limit_count 0.0017f 
#define power_Limit_count_2 0.15f
void chassis_Task(void const *argument)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    osDelay(70);


    for (;;)
    {
        if (power_heat_data.buffer_energy > 20)
        {
            power_limit = 0.6f + power_Limit_count * power_heat_data.buffer_energy;
        }
        else if (power_heat_data.buffer_energy <= 20)
        {
            power_limit = power_Limit_count_2 * get_Power_Limit() * 0.03f * power_heat_data.buffer_energy;
        }

        // heat_gyro(1400.0f, 0.0f, 188.4f);

        // printf("%f,%f\r\n", heatpid.out, imu.temp);
        pid_chassis(50, 0.48f, 0.0046f, motorval4 * power_limit, motorval3 * power_limit);


        can_output(out1, out2, out3, out4);

        vTaskDelayUntil(&xLastWakeTime, 10);

    }
}