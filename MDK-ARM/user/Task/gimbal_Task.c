/**
 * @file gimbal_Task.C
 * @author DreamKerman(2829478110@qq.com)
 * @brief 云台线程
 * @version 0.1
 * @date 2023-12-24
 *
 * @copyright 鼎行双创 (c) 2023
 *
 */

#include "cmsis_os.h"
#include "main.h"
#include "gimbal_Task.h"

#include "tim.h"
#include "pid.h"
#include "canmotor.h"

#include "Vision.h"
#include "tool.h"
#include "bsp_imu.h"
#include "stdio.h"
#include "usart.h"
#include <math.h>
extern int outyaw, outpit;

extern pid_type_def pid;
extern int motorval1, motorval2, motorval3, motorval4, motorvalleft;     //遥控器给定目标转速
extern uint16_t bolun;
extern imu_t  imu;

extern union MyUnion
{
    uint8_t uartbuff[8];
    double data;
}myunion;

pid_type_def pid_Speed[4];
pid_type_def yaw_angle_pid;
pid_type_def pit_angle_pid;
pid2_pos_t yaw_pid;

char strff[30];
//平均数
float average = 0;

float yaw_angle = 0, pit_angle = 0;
float yaw_angle_now, pit_angle_now;

uint8_t volatile shoot_flag = 0;
// uint8_t Data_Enable[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC };		//达妙电机使能命令
// uint8_t Data_Failure[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD };		//电机失能命令
// uint8_t Data_Save_zero[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE };	    //电机保存零点命令
double count = 0;
void gimbal_Task(void const *argument)
{

    int32_t i = 0;
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET);
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    osDelay(40);

    PID_Init(&pid_Speed[0], 100, 1, 0, 10000, 1000);//拨盘电机Pid定义
    PID_Init(&pid_Speed[1], 100, 1, 0, 10000, 1000);
    PID_Init(&pid_Speed[2], 100, 1, 0, 10000, 1000);
    PID_Init(&pid_Speed[3], 100, 1, 0, 10000, 1000);

    //PID_Init(&yaw_angle_pid, -4.394293717587193, -0.006187283187752414, -80.086070746268, 500.6f, 500.6f);

    // PID_Init(&yaw_angle_pid, -0.846175124586586, -0.000053321143253938, 0.0f, 50.6f, 50.6f);
    // PID_Init(&yaw_angle_pid, 5.965318035648911f, 0.001678159778040436f, 0.0f, 50.6f, 50.6f);
    //4.405531514724122,0.000706429577441695,3.648934042743
    // PID_Init(&yaw_angle_pid, 3.992492024750318f, 0.000583825399352108f, 5.613866249721f, 50.6f, 50.6f);
    PID_Init(&yaw_angle_pid, 4.290100060787979f, 0.000908624602309391, 0.0f, 50.6f, 50.6f);
    PID_Init(&pit_angle_pid, 28.646833452546126f, 0.063105286115019380f, 189.959613004071f, 8000.0f, 8000.0f);
    // PID_Init(&pit_angle_pid, 50.0f, 0.0f, 1600.0f, 15000.0f, 15000.0f);
    //2.175531364736235e+04,2.080365033389496e+03
    // pid2_Pos_init(&yaw_pid, 21.2367694202004, 54.2357552128244, 1.84916014298235, 0.40375923648333, 0.00231116379003304, 1000, 50000);

    // HAL_UART_Receive_DMA(&huart7, myunion.uartbuff, 8);



    // yaw_angle = ((val_2[7].rotor_angle) % 16471) / 2621.4411123;
    memset(strff, 0, sizeof(strff));
    yaw_angle = (-imu.yaw + 180) * (float)Pi / 180.0f;
    pit_angle = 6150;

    /* Infinite loop */
    for (;;)
    {
        if (i > 0)
        {
            count = -0.5;
        }
        if (i <= 0)
        {
            count = 0.5;
        }
        if (i >= 1000)
        {
            i = -1000;
        }
        i++;

        PID_Calc(&pid_Speed[0], val_2[1 + 8].rotor_speed, -7230, 0);//拨盘电机计算
        PID_Calc(&pid_Speed[1], val_2[2 + 8].rotor_speed, 7230, 0);
        PID_Calc(&pid_Speed[2], val_2[3 + 8].rotor_speed, 7230, 0);
        PID_Calc(&pid_Speed[3], val_2[4 + 8].rotor_speed, -7230, 0);

        set_motor_voltage_can2_low(pid_Speed[0].out, pid_Speed[1].out, pid_Speed[2].out, pid_Speed[3].out);//发送数据到can

        yaw_angle_now = ((-imu.yaw + 180) * (float)Pi / 180.0f);
        // yaw_angle_now = imu.yaw + 180;
        // yaw_angle_now = ((val_2[7].rotor_angle) % 16471) / 2621.4411123;//计算yaw_angle
        yaw_angle -= (fabs(motorval1) > 100 ? motorval1 : 0) / 660000.0f;

        if (yaw_angle > 2 * Pi)
        {
            yaw_angle -= 2 * Pi;
        }
        else if (yaw_angle < 0)
        {
            yaw_angle += 2 * Pi;
        }

        // pid2_Pos_clc(&yaw_pid, yaw_angle, yaw_angle_now, 2 * Pi);//计算yaw_pid

        PID_Calc(&yaw_angle_pid, yaw_angle_now, yaw_angle + circule_compensate(yaw_angle, yaw_angle_now, 2 * Pi), 0.0);//yaw_angle_pid计算

        // HAL_UART_Receive_DMA(&huart7, myunion.uartbuff, 8);//接收数据
        // if (isnan(myunion.data))
        // {
        //     myunion.data = 0;
        // }
        //
        // if (i == 1)
        // {
        //     yaw_angle_pid.out = -1;
        // }
        // if (i == -1)
        // {
        //     yaw_angle_pid.out = 1;
        // }


        Speed_CtrlMotor(&hcan1, 0x206, yaw_angle_pid.out);
        // Speed_CtrlMotor(&hcan1, 0x206, count);
        // // printf("%f\r\t",imu.yaw);
        // yaw_angle_now = ((imu.yaw + 180 + count) * (float)Pi / 180.0f);
        // sprintf(strff, "%f,%f,%f\r\n", yaw_angle, yaw_angle_now,Data_yaw);
        // // //
        // // //
        // sprintf(strff, "%f,%f\r\n",yaw_angle,yaw_angle_now);
        // sprintf(strff, "%f,%f\r\n", count, yaw_angle_now);
        // HAL_UART_Transmit_DMA(&huart7, (uint8_t *)&strff, strlen(strff));//发送数据
        // huart7.gState = HAL_UART_STATE_READY;

        pit_angle_now = val_2[5].rotor_angle;
        pit_angle -= (fabs(motorval2) > 50 ? motorval2 : 0) / 5500.0f;//计算pit_angle
        //
        if (pit_angle < 5800)
        {
            pit_angle = 5800;
        }
        else if (pit_angle > 6600)
        {
            pit_angle = 6600;  //限制pit_angle范围，防止超出电机转角范围。
        }
        PID_Calc(&pit_angle_pid, pit_angle_now, pit_angle, 0);//pit_angle_pid计算

        can_output_gimbal(pit_angle_pid.out - 7800, 0, 0, 0);
        //can_output_gimbal(-5500, 0, 0, 0);
        // can_output_gimbal(count- 7800, 0, 0, 0);
        // sprintf(strff, "%f,%f\r\n", count, pit_angle_now);
        // HAL_UART_Transmit_DMA(&huart7, (uint8_t *)&strff, strlen(strff));//发送数据
        // huart7.gState = HAL_UART_STATE_READY;

        if (bolun < 1024)
        {
            shoot_flag = 1;
        }
        else
        {
            shoot_flag = 0;
        }

        // HAL_UART_Receive_DMA(&huart7, WIT_data_orn, WIT_DATA_LENGTH);
        // WIT_read_data();
        // printf("%f,%f,%f\n", heatpid.out,imu.temp,imu.yaw);
        vTaskDelayUntil(&xLastWakeTime, 1);




    }
}