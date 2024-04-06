#include "tool.h"
#include "stdint.h"
#include "string.h"

double w_gyro = 0.0f;
void PID_Init(pid_type_def *pid, const fp32 kp, const fp32 ki, const fp32 kd, fp32 max_out, fp32 max_iout)
{
    memset(pid, 0, sizeof(pid_type_def));
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;

}

// fp32 PID_Calc(pid_type_def *pid, fp32 now, fp32 target)
// {


//     pid->error[2] = pid->error[1];
//     pid->error[1] = pid->error[0];
//     pid->set = target;
//     pid->fdb = now;
//     pid->error[0] = target - now;

//     pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
//     pid->Iout = pid->Ki * pid->error[0];
//     pid->Dbuf[2] = pid->Dbuf[1];
//     pid->Dbuf[1] = pid->Dbuf[0];
//     pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
//     pid->Dout = pid->Kd * pid->Dbuf[0];
//     pid->out += pid->Pout + pid->Iout + pid->Dout;
//     LimitMax(pid->out, pid->max_out);


//     return pid->out;
// }

fp32 PID_Calc(pid_type_def *pid, fp32 now, fp32 target, fp32 deadline)
{


    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = target;
    pid->fdb = now;
    pid->error[0] = target - now;
    if (pid->error[0] >= deadline || pid->error[0] <= -deadline)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else
    {
        pid->out = 0;
    }
    return pid->out;

}
/**
 * @brief 三平均值滤波器
 *
 * @param input
 * @param fillter_buf[3]
 * @return int16_t
 */
int16_t wave_filter_3(int16_t filter_buf[3], int16_t input)
{
    // 3个数进行平均
    int32_t temp = 0;
    static const fp32 fliter_num[3] = { 1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f };
    filter_buf[2] = filter_buf[1];
    filter_buf[1] = filter_buf[0];
    filter_buf[0] = input;


    temp = filter_buf[0] * fliter_num[0] + filter_buf[1] * fliter_num[1] + filter_buf[2] * fliter_num[2];
    return temp;
}


/**
 * @brief  采用浮点数据等比例转换成整数
 * @param  x_int     	要转换的无符号整数
 * @param  x_min      目标浮点数的最小值
 * @param  x_max    	目标浮点数的最大值
 * @param  bits      	无符号整数的位数
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief  将浮点数转换为无符号整数
 * @param  x     			要转换的浮点数
 * @param  x_min      浮点数的最小值
 * @param  x_max    	浮点数的最大值
 * @param  bits      	无符号整数的位数
 */

int float_to_uint(float x, float x_min, float x_max, int bits) {
    /// Converts a float to an unsigned int, given range and number of bits///
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float circule_compensate(float target, float current, float circule_num)
{
    float temp = target - current;
    if (temp > circule_num / 2.0f)
    {
        temp = -circule_num;
    }
    else if (temp < -circule_num / 2.0f)
    {
        temp = circule_num;
    }
    else
    {
        temp = 0;
    }
    return temp;
}

// 低通滤波器
float low_pass_filter(float input, float last_output, float alpha)
{
    return alpha * input + (1 - alpha) * last_output;
}