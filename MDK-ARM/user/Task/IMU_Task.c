#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "heat.h" 
#include "pid.h"
#include "bsp_imu.h"
float yaw_rad = 0;
void IMU_Task(void const *argument)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        mpu_get_data();
        imu_ahrs_update();
        imu_attitude_update();
        heat_gyro(2885.0f, 0.0f, 288.4f);
        vTaskDelayUntil(&xLastWakeTime, 1);
        yaw_rad = -imu.yaw / 180.0f * Pi;
        if (yaw_rad < -Pi)
        {
            yaw_rad += 2 * Pi;
        }
        else if (yaw_rad > Pi)
        {
            yaw_rad -= 2 * Pi;
        }

    }

}