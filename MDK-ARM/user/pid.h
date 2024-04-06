#include "stm32f4xx_hal.h"
#include "stdint.h"

#ifndef __PID_H__
#define __PID_H__

#define Pi 3.1415926f
//int set_motor_out(void);
typedef struct
{
    float input,now;
    float kp, ki, kd;

    float b;
    float c;

    float Ierror[2];
    float Derror[2];

    float max_out;  //最大输出
    float max_iout; //最大积分输出

    float Pout;
    float Iout;
    float Dout;
    float Output;
}pid2_pos_t;

extern int bullet_v1;
extern int bullet_v2;
void pid_chassis(float kp, float ki, float kd, float setmv4, float setmv3);
int pid_Shoot1(int sdbspeed, float kp, float ki, float kd);
int pid_Shoot2(int sdbspeed, float kp, float ki, float kd);
void zidong_ornot(void);
void youyi(__IO uint32_t ms);
void zuoyi(__IO uint32_t ms);
void zuotingzhi(__IO uint32_t ms);
void youtingzhi(__IO uint32_t ms);
void youqidong(__IO uint32_t ms);
void zuoqidong(__IO uint32_t ms);
void qianyi(__IO uint32_t ms);
void qianqidong(__IO uint32_t ms);
void qiantingzhi(__IO uint32_t ms);
float pid2_Pos_init(pid2_pos_t *p, float kp, float ki, float kd, float c, float b, float max_out, float max_iout);
float pid2_Pos_clc(pid2_pos_t *p, float target, float now, float circule_num);

//typedef struct pid
//{
//	int16_t speed;
//	int16_t lastspeed;
//	int16_t lalastspeed;
//} pid;
#endif
