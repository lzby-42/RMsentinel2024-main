#ifndef __TOOL_H__
#define __TOOL_H__
#include "stdint.h"

#define fp32 float
#define uint8_t unsigned char

//限幅

#define text 1 


#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

typedef struct
{
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;


extern double w_gyro;

void PID_Init(pid_type_def *pid, const fp32 kp, const fp32 ki, const fp32 kd, fp32 max_out, fp32 max_iout);
// fp32 PID_Calc(pid_type_def *pid, fp32 now, fp32 target);
fp32 PID_Calc(pid_type_def *pid, fp32 now, fp32 target,fp32 deadline);
int16_t wave_filter_3(int16_t filter_buf[3], int16_t input);
float circule_compensate(float target, float current, float circule_num);
float low_pass_filter(float input, float last_output, float alpha);

#endif // !__TOOL_H__

