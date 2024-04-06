#include "main.h"
#include "can.h"
#ifndef   __CANMOTOR_H__
#define  __CANMOTOR_H__

#define T_MIN -18			//转矩最大值
#define T_MAX 18			//转矩最小值
#define P_MIN -12.5		//位置最小值
#define P_MAX 12.5		//位置最大值
#define V_MIN -45			//速度最小值
#define V_MAX 45			//速度最大值

extern uint8_t can5FIFO_flag;
extern uint8_t canCFIFO_flag;

extern uint8_t Data_Enable[8];		//达妙电机使能命令
extern uint8_t Data_Failure[8];		//电机失能命令
extern uint8_t Data_Save_zero[8];	    //电机保存零点命令

typedef struct
{
    uint16_t can_id;
    int16_t  set_voltage;
    uint16_t rotor_angle;
    int16_t  rotor_speed;
    int16_t  torque_current;
    uint8_t  temp;
}moto_info_t;
extern moto_info_t val_2[16];

void can_output(int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void can_output_yuntaimotor(int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void can_output_gimbal(int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void PosSpeed_CtrlMotor(CAN_HandleTypeDef *hcan, uint16_t id, float _pos, float _vel);
void Speed_CtrlMotor(CAN_HandleTypeDef *hcan, uint16_t ID, float _vel);
void can_filter_init(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void set_motor_voltage_can2_low(int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void set_motor_voltage_can2_hig(int16_t v1, int16_t v2, int16_t v3, int16_t v4);
uint8_t CANx_SendStdData(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *pData, uint16_t Len);
//void can2_user_init(CAN_HandleTypeDef* hcan );
void can2_filter_init(void);
#endif

