#include "main.h"
#include "Vision.h"
#include "tim.h"
#include "pid.h"
#include "canmotor.h"
#include "stdio.h"
#include "tool.h"
#include "string.h"
#include "config.h"
uint8_t  Com4_Vision_Buffer[VISION_BUFFER_LENGTH] = { 0 };
uint8_t  Vision_Buffer[VISION_BUFFER_LENGTH] = { 0 };
uint8_t  Vision_Buffer_Flag = 0;
// union x16Tofloat
// {
// 	float out;
// 	uint8_t in[4];
// }yaw, pit;

float Data_yaw, Data_pit;

uint16_t time_count = 0;
uint8_t visioning_flag = 0;
volatile uint8_t visioning_flag_shoot = 0;
float last_yaw_angle = 0.0f;
float last_pit_angle = 0.0f;

extern float yaw_angle, pit_angle;
extern float yaw_angle_now, pit_angle_now;

void vision_read(void)
{
	int i;
	for (i = 0; i < VISION_BUFFER_LENGTH - 10; i++)
	{
		//bit:  0123456789A
		//inpot:a-1.1 -1.1e
		//inpot:a1.1 1.1e 
		if (Vision_Buffer[i] == 'a' && Vision_Buffer[i + 10] == 'e')
		{

			Data_pit = (float)(Vision_Buffer[i + 2] - '0') + (float)(Vision_Buffer[i + 4] - '0') / 10.0f;
			if (Vision_Buffer[i + 1] == '-')
			{
				Data_pit = -Data_pit;
			}

			Data_yaw = (float)(Vision_Buffer[i + 7] - '0') + (float)(Vision_Buffer[i + 9] - '0') / 10.0f;
			if (Vision_Buffer[i + 6] == '-')
			{
				Data_yaw = -Data_yaw;
			}



		}


	}
	time_count++;
	if (time_count >= 10)
	{
		visioning_flag = 0;
		time_count = 10;
	}
}
void vinion_old_Version(void)
{
	vision_read();

	if ((Data_yaw > 0.1f || Data_yaw < -0.1f) && (Data_yaw < 10.0f && Data_yaw > -10.0f))
	{
		yaw_angle = yaw_angle_now + ((Data_yaw > 7.5f) || (Data_yaw < -7.5f) ? Data_yaw : (Data_yaw - 0.2f) * 0.95f) / 120.0f;//计算yaw_angle

	}
	if ((Data_pit > 0.1f || Data_pit < -0.1f) && (Data_pit < 10.0f && Data_pit > -10.0f))
	{
		pit_angle = pit_angle_now - ((Data_pit > 0.1f) || (Data_pit < -0.1f) ? Data_pit : 0) / 200.0f;//计算pit_angle

	}
	//自动射击判定
	if (__fabs(Data_yaw - 0.2f) < 2.0f && __fabs(Data_pit) < 2.1f)
	{
		visioning_flag_shoot = 1;
	}
	else
	{
		visioning_flag_shoot = 0;
	}

}
uint16_t count_T = 0;
void auto_Aim(void)
{
	fp32 cacred = 0.3f;
	count_T++;
	switch (2)
	{
	case 1:
		if (count_T > 0 && count_T < 1000)
		{
			cacred = 0.0f;
		}
		if (count_T > 1000 && count_T < 2000)
		{
			cacred = 120.0f / 180.0f * Pi;
		}
		if (count_T > 2000 && count_T < 3000)
		{
			cacred = 240.0f / 180.0f * Pi;
		}
		if (count_T > 3000 && count_T < 4000)
		{
			count_T = 0;
		}
		count_T++;
		yaw_angle = cacred;
		break;

	case 2:
		// if (count_T++ > 500)
		// {
		yaw_angle += 0.01745;
		count_T = 0;
		if (yaw_angle > 2 * Pi)
		{
			yaw_angle -= 2*Pi;
		}

		// }
		break;
	case 3:
		// if (yaw_angle < Pi/2)
		// {
		// 	cacred = 0.034906585f;
		// }
		// if (yaw_angle > Pi + Pi/2)
		// {
		// 	cacred = -0.034906585f;
		// }
		// yaw_angle += cacred;
		// if (count_T < 1000)
		// {
		// 	yaw_angle = Pi / 2;
		// }
		// else if (1000 < count_T && count_T < 2000)
		// {
		// 	yaw_angle = -Pi / 2;
		// }else if (2000 < count_T<)
		// {
		// 	/* code */
		// }



		break;

	default:
		break;
	}





}
// int outd;
// void auto_Aim(float kpp, float kip, float kpy, float kiy, float kdp, float kdy)
// {
// 	int limitP = 6590, limitY = 7000, limitOutPitch = 4000, limitOutYaw = 4500;

// 	errpit = setpitval - 9 - pitval;
// 	erryal = setyawval - yawval;
// 	ipit += errpit;
// 	iyaw += erryal;
// 	dpit = errpit - lasterrpit;
// 	dyaw = erryal - lasterryaw;

// 	LimitMax(erryal, 5);

// 	LimitMax(errpit, 5);

// 	LimitMax(ipit, limitP);

// 	LimitMax(iyaw, limitY);

// 	outpit = -(kpp * errpit + kip * ipit + kdp * dpit);
// 	outyaw = (kpy * erryal + kiy * iyaw + kdy * dyaw);
// 	outd = kdy * dyaw;

// 	LimitMax(outpit, limitOutPitch);

// 	LimitMax(outyaw, limitOutYaw);
// 	lasterrpit = errpit;
// 	lasterryaw = erryal;
// }

//	if(Com4_Vision_Buffer[0]==98)
//	{
//		if(Com4_Vision_Buffer[2]==43)
//			pitge=Com4_Vision_Buffer[6]-48;
//		if(Com4_Vision_Buffer[2]==45)
//			pitge=-(Com4_Vision_Buffer[6]-48);
//		if(Com4_Vision_Buffer[8]==43)
//		{
//			yawge=Com4_Vision_Buffer[12]-48;
//			yawshi=Com4_Vision_Buffer[11]-48;
//		}
//			
//		if(Com4_Vision_Buffer[8]==45)
//		{
//			yawge=-(Com4_Vision_Buffer[12]-48);
//			yawshi=-(Com4_Vision_Buffer[11]-48);
//			csa=1;
//		}
//		pitval=Com4_Vision_Buffer[6]-48;
//		yawval=10*(Com4_Vision_Buffer[11]-48)+Com4_Vision_Buffer[12]-48;
//		
//	}		
//		bzw5=Com4_Vision_Buffer[i+5];
//		bzw6=Com4_Vision_Buffer[i+6];
//		bzw0=Com4_Vision_Buffer[i];

