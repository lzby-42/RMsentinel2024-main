#include "saveControl_Task.h"
#include "canmotor.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "chassis_Task.h"
#include "gimbal_Task.h"
#include "Moto_Shoot_Task.h"
#include "heat.h" 
#include "bsp_imu.h"
#include "judgeSys.h"
#include "tool.h"
#include "pid.h"
#include "config.h"

//任务ID
extern osThreadId gimbalHandle;
extern osThreadId chassisHandle;
extern osThreadId visionSysHandle;
extern osThreadId Moto_Shoot1Handle;
extern osThreadId Moto_Shoot2Handle;
extern osTimerId saferHandle;
//遥控器断电定时器
extern uint8_t busTime;
//遥控器输入数据
extern int motorval1, motorval2, motorval3, motorval4, motorvalleft, setmv1;     //遥控器给定目标转速
//云台PID
extern pid_type_def yaw_angle_pid;
extern pid_type_def pit_angle_pid;
extern float yaw_angle, pit_angle;
//底盘PID累计误差
extern float ue1, ue2, ue3, ue4;									//ki乘的误差累计值
void saveControl_Task(void const *argument)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    // osTimerStart(saferHandle, 10);
    uint8_t count = 0;
    busTime = 0;
    for (;;)
    {


        if (count++ > 10)
        {
            //底盘断电判定
            if (robot_status.power_management_chassis_output == 0)
            {
                if (eTaskGetState(chassisHandle) != eSuspended)
                {
                    vTaskSuspend(chassisHandle);
                    memset(&val_2[0], 0, sizeof(moto_info_t) * 4);
                    ue1 = 0;
                    ue2 = 0;
                    ue3 = 0;
                    ue4 = 0;
                }
                pid_chassis(50, 0.048f, 0.046f, 0, 0);

            }
            else if (robot_status.power_management_chassis_output == 1 && eTaskGetState(chassisHandle) == eSuspended)
            {
                vTaskResume(chassisHandle);
            }

            //云台断电判定
            if (robot_status.power_management_gimbal_output == 0)
            {
                if (eTaskGetState(gimbalHandle) != eSuspended)
                {
                    vTaskSuspend(gimbalHandle);

                    yaw_angle_pid.out = 0;
                    pit_angle_pid.out = 0;
                    yaw_angle_pid.Pout = 0;
                    pit_angle_pid.Pout = 0;
                    yaw_angle_pid.Iout = 0;
                    pit_angle_pid.Iout = 0;
                    yaw_angle_pid.Dout = 0;
                    pit_angle_pid.Dout = 0;
                    memset(&yaw_angle_pid.Dbuf, 0, 3);
                    memset(&pit_angle_pid.Dbuf, 0, 3);
                    memset(&yaw_angle_pid.error, 0, 3);
                    memset(&pit_angle_pid.error, 0, 3);
                    can5FIFO_flag = 1;
                }
                yaw_angle = (-imu.yaw + 180) * (float)Pi / 180.0f;
                pit_angle = 6300;
            }
            else if (robot_status.power_management_gimbal_output == 1 && eTaskGetState(gimbalHandle) == eSuspended)
            {
                if (can5FIFO_flag == 1)
                {
                    CANx_SendStdData(&hcan1, 0x206, Data_Enable, 8);
                }
                else
                {
                    vTaskResume(gimbalHandle);

                }

            }
            count = 0;
        }

        //射击限制判定
        if (count == 5)
        {
            if (get_17Heat_1_Now() + 1 * robot_status.shooter_barrel_cooling_value > 400)
            {
                shoot_flag_1 = 0;
            }
            else if (get_17Heat_1_Now() + 4 * robot_status.shooter_barrel_cooling_value < 400)
            {
                shoot_flag_1 = 1;
            }

            if (get_17Heat_2_Now() + 1 * robot_status.shooter_barrel_cooling_value > 400)
            {
                shoot_flag_2 = 0;
            }
            else if (get_17Heat_2_Now() + 4 * robot_status.shooter_barrel_cooling_value < 400)
            {
                shoot_flag_2 = 1;
            }
        }


        //遥控器紧急断电

        if (conturl == 1 && busTime > 50)
        {

            vTaskSuspend(chassisHandle);
            can_output(0, 0, 0, 0);
            vTaskSuspend(gimbalHandle);
            yaw_angle_pid.out = 0;
            pit_angle_pid.out = 0;
            yaw_angle_pid.Pout = 0;
            pit_angle_pid.Pout = 0;
            yaw_angle_pid.Iout = 0;
            pit_angle_pid.Iout = 0;
            yaw_angle_pid.Dout = 0;
            pit_angle_pid.Dout = 0;
            memset(&yaw_angle_pid.Dbuf, 0, 3);
            memset(&pit_angle_pid.Dbuf, 0, 3);
            memset(&yaw_angle_pid.error, 0, 3);
            memset(&pit_angle_pid.error, 0, 3);
            can5FIFO_flag = 1;
            vTaskSuspend(visionSysHandle);
            vTaskSuspend(Moto_Shoot1Handle);
            vTaskSuspend(Moto_Shoot2Handle);
            CANx_SendStdData(&hcan1, 0x206, Data_Failure, 8);
            // PID_Init(&yaw_angle_pid, -6.690547472340761, -0.001550956007973143, -111.792520982351, 15.6f, 15.6f);
            // PID_Init(&pit_angle_pid, 60000, 50, 300, 16000.0f, 1000.0f);
            while (busTime > 50)
            {
                CANx_SendStdData(&hcan1, 0x206, Data_Failure, 8);
                motorval1 = 0;
                motorval2 = 0;
                motorval3 = 0;
                motorval4 = 0;
                motorvalleft = 2;
                setmv1 = 0;
                can5FIFO_flag = 1;
                HAL_Delay(1);
            }
            // CANx_SendStdData(&hcan1, 0x206, Data_Enable, 8);
            // vTaskResume(chassisHandle);
            // vTaskDelay(1);
            // vTaskResume(gimbalHandle);
            // vTaskDelay(1);
            // vTaskResume(visionSysHandle);
            // vTaskDelay(1);
            // vTaskResume(Moto_Shoot1Handle);
            // vTaskDelay(1);
            // vTaskResume(Moto_Shoot2Handle);
            busTime = 50;
        }
        busTime++;




        vTaskDelayUntil(&xLastWakeTime, 10);
    }
}
void saferStart(void const *argument)
{
    // if (busTime > 50)
    // {

    //     vTaskSuspend(chassisHandle);
    //     vTaskSuspend(gimbalHandle);
    //     vTaskSuspend(visionSysHandle);
    //     vTaskSuspend(Moto_Shoot1Handle);
    //     vTaskSuspend(Moto_Shoot2Handle);
    //     while (busTime > 50)
    //     {
    //         CANx_SendStdData(&hcan1, 0x206, Data_Failure, 8);
    //         motorval1 = 0;
    //         motorval2 = 0;
    //         motorval3 = 0;
    //         motorval4 = 0;
    //         motorvalleft = 2;
    //         setmv1 = 0;
    //         mpu_get_data();
    //         imu_ahrs_update();
    //         imu_attitude_update();
    //         heat_gyro(2885.0f, 0.0f, 288.4f);
    //         HAL_Delay(1);
    //     }
    //     CANx_SendStdData(&hcan1, 0x206, Data_Enable, 8);
    //     vTaskResume(chassisHandle);
    //     vTaskDelay(1);
    //     vTaskResume(gimbalHandle);
    //     vTaskDelay(1);
    //     vTaskResume(visionSysHandle);
    //     vTaskDelay(1);
    //     vTaskResume(Moto_Shoot1Handle);
    //     vTaskDelay(1);
    //     vTaskResume(Moto_Shoot2Handle);

    // }
    // busTime++;
    // if (busTime > 100)
    // {
    //     busTime = 100;
    // }

    // osTimerStart(saferHandle, 10);
}

//#include <stdio.h>

