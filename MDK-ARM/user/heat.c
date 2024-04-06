#include "bsp_imu.h"
#include "heat.h"
#include "tool.h"
#include "tim.h"
#define h
float setheat = 40.3f, heatout, heatcs;
pid_type_def heatpid;
void heat_gyro(float kp, float ki, float kd)
{
	float ilimit = 5050;
	if (imu.temp >= 39.9f)
	{
		heatpid.Kp = kp;
		heatpid.Ki = ki;
		heatpid.Kd = kd;
		heatpid.set = setheat;
		heatpid.fdb = imu.temp;
		heatpid.error[1] = heatpid.error[0];
		heatpid.error[0] = (heatpid.set) - (heatpid.fdb);
		heatpid.Pout = heatpid.Kp * heatpid.error[0];
		heatpid.Iout += heatpid.Ki * heatpid.error[0];
		heatpid.Dout = heatpid.Kd * (heatpid.error[0] - heatpid.error[1]);
		LimitMax(heatpid.Iout, ilimit);
		heatpid.out = heatpid.Pout + heatpid.Iout;
		LimitMax(heatpid.out, 2000);
	
		if (heatpid.out <= 0)heatpid.out = 0;
		heatout = heatpid.out;
	}
	else
	{
		heatpid.out = 2000;
	}

	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, heatpid.out);
	
}
pid_type_def ptpid;
int ptytcs, pterr, ptdcs[2], ptiout;
double ptierr;
void pingtou(float kp, float ki, float kd)
{
	float ilimit = 14500;
	ptpid.Kp = kp;
	ptpid.Ki = ki;
	ptpid.Kd = kd;
	ptpid.set = -5;
	ptpid.fdb = imu.pit;
	ptpid.error[0] = (ptpid.set) - (ptpid.fdb);
	pterr = ptpid.error[0];
	ptpid.Pout = ptpid.Kp * ptpid.error[0];
	ptierr += 1.5f * ptpid.error[0];
	if (ptierr >= ilimit)ptierr = ilimit;
	if (ptierr <= -ilimit)ptierr = -ilimit;
	ptpid.Iout = ptpid.Ki * ptierr;
	ptpid.Dbuf[0] = (ptpid.error[0] - ptpid.error[1]);
	ptpid.Dout = ptpid.Kd * ptpid.Dbuf[0];
	ptpid.out = ptpid.Pout + ptpid.Iout + ptpid.Dout;
	ptdcs[0] = ptpid.Dbuf[0];
	ptdcs[1] = ptpid.Dout;
	ptiout = ptpid.Iout;
	LimitMax(ptpid.out, 5000);
	ptytcs = ptpid.out;
	ptpid.error[1] = ptpid.error[0];
}

