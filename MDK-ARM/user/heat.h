#include "main.h"
#ifndef   __HEAT_H__
#define  __HEAT_H__

//typedef float fp32;
//typedef struct 
//{
//    uint8_t mode;
//    //PID 三参数
//    fp32 Kp;
//    fp32 Ki;
//    fp32 Kd;

//    fp32 max_out;  //最大输出
//    fp32 max_iout; //最大积分输出

//    fp32 set;
//    fp32 fdb;

//    fp32 out;
//    fp32 Pout;
//    fp32 Iout;
//    fp32 Dout;
//    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
//    fp32 error[3]; //误差项 0最新 1上一次 2上上次

//} pid_type_def;
void heat_gyro(float kp,float ki,float kd);
void pingtou(float kp,float ki,float kd);

#endif
