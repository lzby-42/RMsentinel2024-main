#include "main.h"
#ifndef   __HEAT_H__
#define  __HEAT_H__

//typedef float fp32;
//typedef struct 
//{
//    uint8_t mode;
//    //PID ������
//    fp32 Kp;
//    fp32 Ki;
//    fp32 Kd;

//    fp32 max_out;  //������
//    fp32 max_iout; //���������

//    fp32 set;
//    fp32 fdb;

//    fp32 out;
//    fp32 Pout;
//    fp32 Iout;
//    fp32 Dout;
//    fp32 Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
//    fp32 error[3]; //����� 0���� 1��һ�� 2���ϴ�

//} pid_type_def;
void heat_gyro(float kp,float ki,float kd);
void pingtou(float kp,float ki,float kd);

#endif
