#ifndef __Vision_H
#define __Vision_H
#include "main.h"
#include "stm32f4xx.h"

#include <stdbool.h>

/*--------------------------------暂定协议-------------------------------------*/

//起始字节,协议固定为0xA5
#define    VISION_SOF         (0xFF)
#define    VISION_WZ          (0xEE)
//长度根据协议定义,数据段长度为n需要根据帧头第二字节来获取
#define    VISION_LEN_HEADER    1        //帧头长
#define    VISION_LEN_DATA      10        //数据段长度,可自定义
#define    VISION_LEN_PACKED    8      //数据包长度,可自定义
#define 	 RECEIVE_TASK_INIT_TIME 250 //任务开始初期 delay 一段时间
//任务控制间隔 2ms
#define RECEIVE_CONTROL_TIME_MS 2
#define VISION_BUFFER_LEN 96 
typedef __packed struct
{
  /* 头 */
  uint8_t   SOF;			//帧头起始位,暂定0xA5
  uint8_t   CmdID;		//指令	
}
VisionSendHeader_t;

//STM32接收,直接将串口接收到的数据拷贝进结构体
typedef __packed struct
{
  /* 头 */
//	uint8_t   SOF;			//帧头起始位,暂定0xA5		

  /* 数据 */
  float     pitch_angle;
  float     yaw_angle;

  //	uint8_t	  wz;   //目标距离太近,视觉发1，否则发0

}
VisionRecvData_t;

#define AUTO  KEY_PRESSED_OFFSET_C
#define OPEN  KEY_PRESSED_OFFSET_R
#define CLOSE  KEY_PRESSED_OFFSET_G
#define JUDGEMENT_SIZE 140

//STM32发送,直接将打包好的数据一个字节一个字节地发送出去
typedef struct
{

  /* 数据 */
  uint16_t     pitch_angles;
  uint16_t     yaw_angles;


}
VisionSendData_t;

void vision_new_version(void);
//void Vision_Read_Data(uint8_t *ReadFromUsart);
void Vision_Send_Data(uint16_t Byte);
//ask转16位
uint8_t ask_To_0x(uint8_t ch);

extern VisionRecvData_t      VisionRecvData;    //视觉接收结构体

extern VisionSendData_t      VisionSendData;

typedef struct pid_ctrl {
  float		target;
  float		measure;
  float 	err;
  float 	last_err;
  float		kp;
  float 	ki;
  float 	kd;
  float 	pout;
  float 	iout;
  float 	dout;
  float 	out;
  float		integral;
  float 	integral_max;
  float 	integral_bias;
  float 	out_max;
  float 	true_err;
} pid_ctrl_t;

typedef struct {
  pid_ctrl_t	speed;
  pid_ctrl_t	angle;
  float		out;
} gimbal_motor_pid_t;



#define SET_OUTPUT_VISION_HEDER 0x5A
#define APP_TX_DATA_SIZE 28
#define CRC16_INIT 0xFFFF
typedef struct
{
  uint8_t header;
  uint8_t detect_color : 1; // 0-red 1-blue
  bool reset_tracker : 1;
  uint8_t reserved : 6;
  float roll;
  float pitch;
  float yaw;
  float aim_x;
  float aim_y;
  float aim_z;
  uint16_t checksum;
} __attribute__((packed)) SendPacketVision_s;


//typedef struct
//{
//  uint8_t header;
//  uint8_t detect_color : 1; // 0-red 1-blue
//  bool reset_tracker : 1;
//  uint8_t reserved : 6;
//  float roll;
//  float pitch;
//  float yaw;
//  float aim_x;
//  float aim_y;
//  float aim_z;
//} __attribute__((packed)) SendPacketVision_s;


typedef struct
{
  uint8_t header;
  bool tracking : 1;
  uint8_t id : 3;         // 0-outpost 6-guard 7-base
  uint8_t armors_num : 3; // 2-balance 3-outpost 4-normal
  uint8_t reserved : 1;
  float x;
  float y;
  float z;
  float yaw;
  float vx;
  float vy;
  float vz;
  float v_yaw;
  float r1;
  float r2;
  float dz;
  uint16_t checksum;
} __attribute__((packed)) ReceivedPacketVision_s;




typedef struct
{
  uint8_t header;
  uint16_t length;
  uint8_t name_1[10];
  uint8_t type_1;
  float data_1;
  uint8_t name_2[10];
  uint8_t type_2;
  float data_2;
  uint8_t name_3[10];
  uint8_t type_3;
  float data_3;
  uint8_t name_4[10];
  uint8_t type_4;
  float data_4;
  uint8_t name_5[10];
  uint8_t type_5;
  float data_5;
  uint8_t name_6[10];
  uint8_t type_6;
  float data_6;
  uint8_t name_7[10];
  uint8_t type_7;
  float data_7;
  uint8_t name_8[10];
  uint8_t type_8;
  float data_8;
  uint8_t name_9[10];
  uint8_t type_9;
  float data_9;
  uint8_t name_10[10];
  uint8_t type_10;
  float data_10;
  uint16_t checksum;
} __attribute__((packed)) OutputPCData_s;


void usb_send_vision(void);


//void Vision_Send_Data( uint16_t Byte	);
void Usart_SendFloat(USART_TypeDef *USARTx, float data);
void Serial_SendPacket(void);
void Serial_SendArray(uint8_t *Array, uint16_t Length);


#ifndef PI
#define PI 3.1415926535f
#endif
#define GRAVITY 9.78f

enum ARMOR_ID
{
  ARMOR_OUTPOST = 0,
  ARMOR_HERO = 1,
  ARMOR_ENGINEER = 2,
  ARMOR_INFANTRY3 = 3,
  ARMOR_INFANTRY4 = 4,
  ARMOR_INFANTRY5 = 5,
  ARMOR_GUARD = 6,
  ARMOR_BASE = 7
};

enum ARMOR_NUM
{
  ARMOR_NUM_BALANCE = 2,
  ARMOR_NUM_OUTPOST = 3,
  ARMOR_NUM_NORMAL = 4
};

enum BULLET_TYPE
{
  BULLET_17 = 0,
  BULLET_42 = 1
};



struct SolveTrajectoryParams
{
  float k;             //弹道系数

  //自身参数
  enum BULLET_TYPE bullet_type;  //自身机器人类型 0-步兵 1-英雄
  float current_v;      //当前弹速
  float current_pitch;  //当前pitch
  float current_yaw;    //yaw

  //目标参数
  float xw;             //ROS坐标系下的x
  float yw;             //ROS坐标系下的y
  float zw;             //ROS坐标系下的z
  float vxw;            //ROS坐标系下的vx
  float vyw;            //ROS坐标系下的vy
  float vzw;            //ROS坐标系下的vz
  float tar_yaw;        //目标yaw
  float v_yaw;          //目标yaw速度
  float r1;             //目标中心到前后装甲板的距离
  float r2;             //目标中心到左右装甲板的距离
  float dz;             //装甲板的高度差
  int bias_time;        //偏置时间
  float s_bias;         //枪口前推的时间
  float z_bias;         //yaw电机到枪口水平面的垂直距离
  enum ARMOR_ID armor_id;     //装甲板类型  0-outpost 6-guard 7-base
  //1-英雄 2-工程 3-4-5-步兵 
  enum ARMOR_NUM armor_num;   //装甲板数字  2-balance 3-outpost 4-normal
};

//????????????
struct tar_pos
{
  float x;           //装甲板在世界坐标系下的x
  float y;           //装甲板在世界坐标系下的y
  float z;           //装甲板在世界坐标系下的z
  float yaw;         //装甲板在世界坐标系下的yaw角度
};
extern uint8_t Com4_newVision_Buffer[VISION_BUFFER_LEN];
extern uint8_t newVision_Buffer[VISION_BUFFER_LEN];
extern float b_set_angle;
extern float yaw2;
extern ReceivedPacketVision_s ReceivedPacketVision;
extern SendPacketVision_s SendPacketVision;

extern uint8_t vision_check;//

//?????????
extern float monoDirectionalAirResistanceModel(float s, float v, float angle);
//????????
extern float completeAirResistanceModel(float s, float v, float angle);
//pitch????
extern float pitchTrajectoryCompensation(float s, float y, float v);
//?????????????? ??????
extern void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z);


void Serial_SendArray2(uint8_t *Array, uint16_t Length);


void Vision_Send_Data2(uint16_t Byte);


#endif
