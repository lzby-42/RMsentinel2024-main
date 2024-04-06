/*********************************/
/*
底盘pid
限制kp应该都能想到
限制ki所乘的累加值，若不限制会导致反应减慢
*/
/********************************/


#include "main.h"
#include "pid.h"
#include "remoter.h"
#include "usart.h"
#include "canmotor.h"
#include "math.h"
#include "bsp_imu.h"
#include "tool.h"
#include "FreeRTOS.h"
#include "task.h"

extern rc_info_t rc;
int timea, tb;
uint32_t wait;
// extern int val_1cs1, val_1cs2, val_1cs3, val_1cs4, val_1cs7speed;         //实际转速
extern int motorval1, motorval2, motorval3, motorval4, motorvalleft;     //遥控器给定目标转速
int limit = 6500;                                         //限制电机的输出值（比例限幅）
int limitcyval = 9450;           							//限制的累加值（应为电机转速最大值积分限幅）
//float kpcs,kics;  比例积分的测试值
float s1, s2, s3, s4;										//电机实际转速与遥控器的发送值对应
float le1, le2, le3, le4;									//上次误差
float e1, e2, e3, e4;										//本次误差
float de1, de2, de3, de4;									//kd乘的误差差值
float ue1, ue2, ue3, ue4;									//ki乘的误差累计值
int sval1, sval2, sval3, sval4;							//各个电机的目标输出值
float setmv1;											//左右旋转的目标值
float out1, out2, out3, out4;						//电机输出值
int bullet_v1 = 0;//弹丸射速
int bullet_v2 = 0;
extern int bolun;


extern imu_t   imu;
#define val  (Pi/180)
float sety, setx;
double seita;
float xcos, ysin;
int setbodanspeed, weifenwucha;
int leijiwucha1, leijiwucha2;
int lasterrorwucha1, lasterrorwucha2;
int speedcs;
float errorwucha1;
float errorwucha2;

void pid_chassis(float kp, float ki, float kd, float setmv4, float setmv3)	//float setmv4（y轴值）,float setmv3（x轴值）
{
	static float wSpeed = 0;

	seita = (((val_2[7].rotor_angle) % 16471) / 2621.4411123) + Pi / 2;//计算yaw_angle
	xcos = cos(-seita);
	ysin = sin(-seita);
	setx = (-setmv3 * xcos) - (setmv4 * ysin);
	sety = (-setmv3 * ysin) + (setmv4 * xcos);


	// setmv1 = (abs(motorval1) > 100 ? motorval1 : 0) / 3.30f;
	if (motorvalleft == 1)
	{
		wSpeed += 0.03f;
		LimitMax(wSpeed, 240);
	}
	if (motorvalleft == 2)
	{
		wSpeed -= 0.03f;
		if (wSpeed < 0)
		{
			wSpeed = 0;
		}

	}


	setmv1 = wSpeed;//60->90°/s
	s1 = val_2[1].rotor_speed / 14.28;
	s2 = val_2[2].rotor_speed / 14.28;
	s3 = val_2[3].rotor_speed / 14.28;
	s4 = val_2[4].rotor_speed / 14.28;
	sval1 = (sety + setx + setmv1);					//当加上云台后y'=y*cos-x*sin   x'=y*sin+x*cos
	sval2 = (sety - setx + setmv1);						//云台的y-x对应遥控器的通道获取值4-y 3-x 
	sval3 = -(sety + setx - setmv1);
	sval4 = -(sety - setx - setmv1);     			//目标值（速度解算）setmv4对应y'  setmv3对应x'  setmv1对应w
	e1 = sval1 - s1;
	e2 = sval2 - s2;
	e3 = sval3 - s3;
	e4 = sval4 - s4; 										//误差1	
	de1 = e1 - le1;
	de2 = e2 - le2;
	de3 = e3 - le3;
	de4 = e4 - le4;											//kd乘的误差差值
	ue1 += e1;
	ue2 += e2;
	ue3 += e3;
	ue4 += e4;

	LimitMax(ue1, limitcyval);							//限制Ki误差值
	LimitMax(ue2, limitcyval);
	LimitMax(ue3, limitcyval);
	LimitMax(ue4, limitcyval);
	//ki乘的误差累计值

	out1 = (kp * e1 + ki * ue1 + kd * de1);
	out2 = (kp * e2 + ki * ue2 + kd * de2);
	out3 = (kp * e3 + ki * ue3 + kd * de3);
	out4 = (kp * e4 + ki * 1 * ue4 + kd * de4);					//电机输出值
	le1 = e1;
	le2 = e2;
	le3 = e3;
	le4 = e4;												//上次误差

	LimitMax(out1, limit);
	LimitMax(out2, limit);
	LimitMax(out3, limit);
	LimitMax(out4, limit);									//限制电机输出值


	//限制的累加值（应为电机转速最大值积分限幅）
}

//void pid_out_val(float kp,float ki,float kd,float setmv4,float setmv3)	//float setmv4（y轴值）,float setmv3（x轴值）
//{
//	setmv1=-(bolun-1024)/2;
//	s1=val_1cs1/14.28;
//	s2=val_1cs2/14.28;
//	s3=val_1cs3/14.28;
//	s4=val_1cs4/14.28;              					//实际值=电机实际转速与/14.28（转速最大值比上遥控器最大输出值）
//	sval1=1.02f*(setmv4+setmv3+setmv1);					//当加上云台后y'=y*cos-x*sin   x'=y*sin+x*cos
//	sval2=(setmv4-setmv3+setmv1);						//云台的y-x对应遥控器的通道获取值4-y 3-x 
//	sval3=-(setmv4+setmv3-setmv1);
//	sval4=-1.1f*(setmv4-setmv3-setmv1);     			//目标值（速度解算）setmv4对应y'  setmv3对应x'  setmv1对应w
//	e1=sval1-s1;
//	e2=sval2-s2;
//	e3=sval3-s3;
//	e4=sval4-1.1f*s4; 										//误差1	
//	de1=e1-le1;
//	de2=e2-le2;
//	de3=e3-le3;
//	de4=e4-le4;											//kd乘的误差差值
//	ue1+=e1;
//	ue2+=e2;
//	ue3+=e3;											
//	ue4+=e4;											//ki乘的误差累计值
//	out1=(kp*e1+ki*ue1+kd*de1);
//	out2=(kp*e2+ki*ue2+kd*de2);
//	out3=(kp*e3+ki*ue3+kd*de3);
//	out4=(kp*e4+ki*ue4+kd*de4);					//电机输出值
////	kpcs=kp*e2;
////	kics=ki*ue2;
//	le1=e1;
//	le2=e2;
//	le3=e3;
//	le4=e4;												//上次误差
//	if(out1>=limit)out1=limit;
//	if(out2>=limit)out2=limit;
//	if(out3>=limit)out3=limit;
//	if(out4>=limit)out4=limit;
//	if(out1<=-limit)out1=-limit;
//	if(out2<=-limit)out2=-limit;
//	if(out3<=-limit)out3=-limit;
//	if(out4<=-limit)out4=-limit;						//限制电机的输出值（比例限幅）
//	if(ue1>=limitcyval)ue1=limitcyval;
//	if(ue2>=limitcyval)ue2=limitcyval;
//	if(ue3>=limitcyval)ue3=limitcyval; 
//	if(ue4>=limitcyval)ue4=limitcyval;
//	if(ue1<=-limitcyval)ue1=-limitcyval;
//	if(ue2<=-limitcyval)ue2=-limitcyval;
//	if(ue3<=-limitcyval)ue3=-limitcyval;
//	if(ue4<=-limitcyval)ue4=-limitcyval;				//限制的累加值（应为电机转速最大值积分限幅）
//}

int pid_Shoot1(int sdbspeed, float kp, float ki, float kd)
{
	// int limitwc = 180;

	int limit = 6500;
	int jifenlimit = 1600;
	speedcs = val_2[5 + 8].rotor_speed / 8;


	errorwucha1 = (sdbspeed - speedcs);
	LimitMax(errorwucha1, 50);
	leijiwucha1 += errorwucha1;
	weifenwucha = errorwucha1 - lasterrorwucha1;

	LimitMax(leijiwucha1, jifenlimit);
	bullet_v1 = kp * errorwucha1 + ki * leijiwucha1 + kd * weifenwucha;

	LimitMax(bullet_v1, limit);
	lasterrorwucha1 = errorwucha1;
	return bullet_v1;
}

int pid_Shoot2(int sdbspeed, float kp, float ki, float kd)
{
	// int limitwc = 180;

	int limit = 6500;
	int jifenlimit = 1000;
	speedcs = val_2[6 + 8].rotor_speed / 8;//我也不知道为什么是8，但是4就会振的很厉害，后辈有时间你们可以试试其他数


	errorwucha2 = (sdbspeed - speedcs);

	leijiwucha2 += errorwucha2;
	weifenwucha = errorwucha2 - lasterrorwucha2;

	LimitMax(leijiwucha2, jifenlimit);
	bullet_v2 = kp * errorwucha2 + ki * leijiwucha2 + kd * weifenwucha;

	LimitMax(bullet_v2, limit);
	lasterrorwucha2 = errorwucha2;
	return bullet_v2;
}

float pid2_Pos_init(pid2_pos_t *p, float kp, float ki, float kd, float c, float b, float max_out, float max_iout)
{
	p->kp = kp;
	p->ki = ki;
	p->kd = kd;
	p->c = c;
	p->b = b;
	p->max_out = max_out;
	p->max_iout = max_iout;
	p->Ierror[0] = 0;
	p->Ierror[1] = 0;
	p->Derror[0] = 0;
	p->Derror[1] = 0;
	return 0;
}

float pid2_Pos_clc(pid2_pos_t *p, float target, float now, float circule_num)
{
	p->Ierror[1] = p->Ierror[0];
	p->Derror[1] = p->Derror[0];

	p->input = target + circule_compensate(target, now, circule_num);

	p->Ierror[0] = p->input - now;
	p->Derror[0] = p->c * p->input - now;

	p->Pout = p->kp * p->b * p->input - now;
	p->Iout += p->ki * p->Ierror[0];
	p->Dout = p->kd * (p->Derror[0] - p->Derror[1]);

	LimitMax(p->Iout, p->max_iout);

	p->Output = p->Pout + p->Iout + p->Dout;
	LimitMax(p->Output, p->max_out);
	return p->Output;
}
//void zidong_ornot(void)
//
//
//{
//
//	    youqidong(500);
//	    youyi(2500);
//		youtingzhi(1000);
//	//*****************************
//		qianqidong(300);
//		qianyi(2000);
//		qiantingzhi(1000);
//	//******************************
//		zuoqidong(500);
//		zuoyi(1500);
//		zuotingzhi(1000);
//	
//		HAL_Delay(1);
//
//}
//
//void zuoqidong(__IO uint32_t ms)
//{
//	uint32_t timea=HAL_GetTick();
//	uint32_t wait1=ms;
//	
//	if(wait1<HAL_MAX_DELAY)
//	{
//		wait1+= (uint32_t)(uwTickFreq);
//	}
//	while((HAL_GetTick()-timea)<wait1)
//	{
////		xyz=1;
//		HAL_Delay(1);
//		pid_out_val(50,0.048f,0.046f,0 ,-50);
//		can_output(out1,out2,out3,out4);
////		can_output(-900,1500,1700,-1400);
//	}
//}
//void youyi(__IO uint32_t ms)
//{
//	uint32_t timea=HAL_GetTick();
//	wait=ms;	
//	int i=50;
//	if(wait<HAL_MAX_DELAY)
//	{
//		wait+= (uint32_t)(uwTickFreq);
//	}
//	while((HAL_GetTick()-timea)<wait)
//	{
//		HAL_Delay(1);
//		i++;
//		if(i>=200)i=200;
//		pid_out_val(50,0.048f,0.046f,0 ,i);
//		can_output(out1,out2,out3,out4);
//		
//	}
//}
//void zuoyi(__IO uint32_t ms)
//{
//	uint32_t timea=HAL_GetTick();
//	uint32_t wait1=ms;
//	int i=-50;
//	if(wait1<HAL_MAX_DELAY)
//	{
//		wait1+= (uint32_t)(uwTickFreq);
//	}
//	while((HAL_GetTick()-timea)<wait1)
//	{
//		HAL_Delay(1);
//		i--;
//		if(i<=-200)i=-200;
//		pid_out_val(50,0.048f,0.046f,0 ,i);
//		can_output(out1,out2,out3,out4);
//	}
//}
//void youtingzhi(__IO uint32_t ms)
//{
//	uint32_t timea=HAL_GetTick();
//	uint32_t wait1=ms;
//	int i=200;
//	if(wait1<HAL_MAX_DELAY)
//	{
//		wait1+= (uint32_t)(uwTickFreq);
//	}
//	while((HAL_GetTick()-timea)<wait1)
//	{
//		HAL_Delay(1);
//		i--;
//		if(i<=0)i=0;
//		pid_out_val(50,0.048f,0.046f,0 ,i);
//		can_output(out1,out2,out3,out4);
//	}
//}
//
//void zuotingzhi(__IO uint32_t ms)
//{
//	uint32_t timea=HAL_GetTick();
//	uint32_t wait1=ms;
//	int i=-200;
//	if(wait1<HAL_MAX_DELAY)
//	{
//		wait1+= (uint32_t)(uwTickFreq);
//	}
//	while((HAL_GetTick()-timea)<wait1)
//	{
//		HAL_Delay(1);
//		i++;
//		if(i>=0)i=0;
//		pid_out_val(50,0.048f,0.046f,0 ,i);
//		can_output(out1,out2,out3,out4);
//	}
//}
//void qianyi(__IO uint32_t ms)
//{
//	uint32_t timea=HAL_GetTick();
//	uint32_t wait1=ms;
//	int i=50;
//	if(wait1<HAL_MAX_DELAY)
//	{
//		wait1+= (uint32_t)(uwTickFreq);
//	}
//	while((HAL_GetTick()-timea)<wait1)
//	{
//		HAL_Delay(1);
//		i++;
//		if(i>=200)i=200;
//		pid_out_val(50,0.048f,0.046f,i ,0);
//		can_output(out1,out2,out3,out4);
//	}
//}
//void qianqidong(__IO uint32_t ms)
//{
//	uint32_t timea=HAL_GetTick();
//	uint32_t wait1=ms;
//	if(wait1<HAL_MAX_DELAY)
//	{
//		wait1+= (uint32_t)(uwTickFreq);
//	}
//	while((HAL_GetTick()-timea)<wait1)
//	{
//		HAL_Delay(1);
//		pid_out_val(50,0.048f,0.046f,50 ,0);
//		can_output(out1,out2,out3,out4);
//	}
//}
//
//void qiantingzhi(__IO uint32_t ms)
//{
//	uint32_t timea=HAL_GetTick();
//	uint32_t wait1=ms;
//	int i=200;
//	if(wait1<HAL_MAX_DELAY)
//	{
//		wait1+= (uint32_t)(uwTickFreq);
//	}
//	while((HAL_GetTick()-timea)<wait1)
//	{
//		HAL_Delay(1);
//		i--;
//		if(i<=0)i=0;
//		pid_out_val(50,0.048f,0.046f,0 ,i);
//		can_output(out1,out2,out3,out4);
//	}
//}
//*extern int motorval1,motorval2,motorval3,motorval4;
// int rmval;
// extern int val_1cs2;
//
//int set_motor_out(void)
//{
//	
//	if((motorval2<=110)&&(motorval2>=-110))
//	{
//			
//		rmval=(-val_1cs2)/14;
//		if(rmval>420){rmval=420;}
//		if(rmval<-420){rmval=-420;}
//	}
//	else
//	{
//		rmval=motorval2;
//	}
//	return rmval;
//}
//*/







