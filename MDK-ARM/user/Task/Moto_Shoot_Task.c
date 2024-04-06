/**
 * @file Moto_Shoot_Task.C
 * @author  DreamKerman (2829478110@qq.com)
 * @brief 拨弹电机子线程
 * @version 0.1
 * @date 2024-01-09
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "Moto_Shoot_Task.h"
#include "gimbal_Task.h"
#include "Vision.h"

#include "canmotor.h"
#include "pid.h"
#include "pid.h"
#include "main.h"
#include "cmsis_os.h"
#include "tool.h"
#include "config.h"



extern int16_t bullet_v1_filter;
extern int16_t bullet_v2_filter;
extern int16_t buffer_v1[3] = { 180,180,180 };
extern int16_t buffer_v2[3] = { 180,180,180 };
extern uint8_t count1;
extern uint8_t count2;

extern uint8_t shoot_flag;

int8_t turn_flag_1 = 1;
int8_t turn_flag_2 = 1;

volatile uint8_t shoot_flag_1 = 0;
volatile uint8_t shoot_flag_2 = 0;

int target_v1 = 380;
int target_v2 = 380;

uint8_t count1 = 0;
uint8_t count2 = 0;
void Moto_Shoot1_Task(void const *argument)
{
    // TickType_t xLastWakeTime;
    // xLastWakeTime = xTaskGetTickCount();
    int16_t bullet_v1_filter;

    osDelay(99);

    uint8_t i = 0;
    while (1)
    {
        if ((val_2[4 + 8].rotor_speed < -3800 &&
            val_2[3 + 8].rotor_speed > 3800 &&
            (shoot_flag == 1 || (shoot_flag_1 ==1 && visioning_flag_shoot == 1))) ||
            text1)
        {
            pid_Shoot1(target_v1, 25.0f, 1.0f, 1.0f);

            set_motor_voltage_can2_hig(turn_flag_1 * bullet_v1, turn_flag_2 * bullet_v2, 0, 0);

            bullet_v1_filter = wave_filter_3(buffer_v1, val_2[5 + 8].rotor_speed);//取平均

            if (bullet_v1_filter <= 50 && bullet_v1_filter >= -50)//堵转检测
            {
                count1++;
            }
            else
            {
                count1 = 0;
            }

            if (count1 >= 50)//时间到
            {
                // turn_flag_1 = -1;
                target_v1 = -250;
                for (i = 0;i <= 100;i++)//开始震荡
                {
                    pid_Shoot1(target_v1, 25.0f, 1.0f, 1.0f);
                    // vTaskDelay(1);
                    set_motor_voltage_can2_hig(turn_flag_1 * bullet_v1, turn_flag_2 * bullet_v2, 0, 0);
                    vTaskDelay(1);


                }
                count1 = 0;
                turn_flag_1 = 1;
                target_v1 = 380;
            }

        }
        else
        {
            bullet_v1=0;
            set_motor_voltage_can2_hig(0, turn_flag_2 * bullet_v2, 0, 0);
        }
        vTaskDelay(1);
    }
}

void Moto_Shoot2_Task(void const *argument)
{
    // TickType_t xLastWakeTime;
    // xLastWakeTime = xTaskGetTickCount();
    int16_t bullet_v2_filter;


    osDelay(99);
    uint8_t i = 0;

    while (1)
    {
        if ((val_2[1 + 8].rotor_speed < -3800 &&
            val_2[2 + 8].rotor_speed > 3800 &&
            (shoot_flag == 1 || (shoot_flag_2 == 1 && visioning_flag_shoot == 1))) ||
            text1)
        {

            pid_Shoot2(target_v2, 25.0f, 1.0f, 1.0f);
            set_motor_voltage_can2_hig(turn_flag_1 * bullet_v1, turn_flag_2 * bullet_v2, 0, 0);

            bullet_v2_filter = wave_filter_3(buffer_v2, val_2[6 + 8].rotor_speed);

            if (bullet_v2_filter == 0)
            {
                count2++;
            }
            else
            {
                count2 = 0;
            }

            if (count2 >= 50)
            {
                // turn_flag_2 = -1;
                target_v2 = -250;
                for (i = 0;i <= 100;i++)//开始震荡
                {
                    pid_Shoot2(target_v2, 25.0f, 1.0f, 1.0f);
                    // vTaskDelay(1);
                    set_motor_voltage_can2_hig(turn_flag_1 * bullet_v1, turn_flag_2 * bullet_v2, 0, 0);
                    vTaskDelay(1);

                }
                count2 = 0;
                turn_flag_2 = 1;
                target_v2 = 350;
            }

        }else
        {
            bullet_v2 = 0;
            set_motor_voltage_can2_hig(turn_flag_1 * bullet_v1, 0, 0, 0);
        }
        vTaskDelay(1);
    }
}

