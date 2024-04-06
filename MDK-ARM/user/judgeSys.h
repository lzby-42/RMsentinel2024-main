
#ifndef __JUDGE_SYS_H__
#define __JUDGE_SYS_H__
#include "main.h"

#define JUDGEMENT_SIZE 140//裁判系统接收数据最大长度

/****************************开始标志*********************/
#define UI_SOF 0xA5
/****************************CMD_ID数据********************/
#define UI_CMD_Robo_Exchange 0x0301   
/****************************内容ID数据********************/
#define UI_Data_ID_Del 0x100 
#define UI_Data_ID_Draw1 0x101
#define UI_Data_ID_Draw2 0x102
#define UI_Data_ID_Draw5 0x103
#define UI_Data_ID_Draw7 0x104
#define UI_Data_ID_DrawChar 0x110
/****************************红方机器人ID********************/
#define UI_Data_RobotID_RHero 1         
#define UI_Data_RobotID_REngineer 2
#define UI_Data_RobotID_RStandard3 3
#define UI_Data_RobotID_RStandard4 4
#define UI_Data_RobotID_RStandard5 5
#define UI_Data_RobotID_RAerial 6
#define UI_Data_RobotID_RSentry 7
#define UI_Data_RobotID_RRadar 9
/****************************蓝方机器人ID********************/
#define UI_Data_RobotID_BHero 101
#define UI_Data_RobotID_BEngineer 102
#define UI_Data_RobotID_BStandard3 103
#define UI_Data_RobotID_BStandard4 104
#define UI_Data_RobotID_BStandard5 105
#define UI_Data_RobotID_BAerial 106
#define UI_Data_RobotID_BSentry 107
#define UI_Data_RobotID_BRadar 109
/**************************红方操作手ID************************/
#define UI_Data_CilentID_RHero 0x0101
#define UI_Data_CilentID_REngineer 0x0102
#define UI_Data_CilentID_RStandard3 0x0103
#define UI_Data_CilentID_RStandard4 0x0104
#define UI_Data_CilentID_RStandard5 0x0105
#define UI_Data_CilentID_RAerial 0x0106
/***************************蓝方操作手ID***********************/
#define UI_Data_CilentID_BHero 0x0165
#define UI_Data_CilentID_BEngineer 0x0166
#define UI_Data_CilentID_BStandard3 0x0167
#define UI_Data_CilentID_BStandard4 0x0168
#define UI_Data_CilentID_BStandard5 0x0169
#define UI_Data_CilentID_BAerial 0x016A
/***************************删除操作***************************/
#define UI_Data_Del_NoOperate 0
#define UI_Data_Del_Layer 1
#define UI_Data_Del_ALL 2
/***************************图形配置参数__图形操作********************/
#define UI_Graph_ADD 1
#define UI_Graph_Change 2
#define UI_Graph_Del 3
/***************************图形配置参数__图形类型********************/
#define UI_Graph_Line 0         //直线
#define UI_Graph_Rectangle 1    //矩形
#define UI_Graph_Circle 2       //整圆
#define UI_Graph_Ellipse 3      //椭圆
#define UI_Graph_Arc 4          //圆弧
#define UI_Graph_Float 5        //浮点型
#define UI_Graph_Int 6          //整形
#define UI_Graph_Char 7         //字符型
/***************************图形配置参数__图形颜色********************/
#define UI_Color_Main 0         //红蓝主色
#define UI_Color_Yellow 1
#define UI_Color_Green 2
#define UI_Color_Orange 3
#define UI_Color_Purplish_red 4 //紫红色
#define UI_Color_Pink 5
#define UI_Color_Cyan 6         //青色
#define UI_Color_Black 7
#define UI_Color_White 8

extern uint8_t judgment_Protect;
extern uint8_t judgment_System_Signal_Buffer[JUDGEMENT_SIZE];
extern uint8_t judgment_System_handle_buffer[JUDGEMENT_SIZE];

typedef __packed struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
}frame_header_t;

typedef __packed struct
{
  uint8_t robot_id;
  uint8_t robot_level;
  uint16_t current_HP;
  uint16_t maximum_HP;
  uint16_t shooter_barrel_cooling_value;
  uint16_t shooter_barrel_heat_limit;
  uint16_t chassis_power_limit;
  uint8_t power_management_gimbal_output : 1;
  uint8_t power_management_chassis_output : 1;
  uint8_t power_management_shooter_output : 1;
}robot_status_t;

typedef __packed struct
{
  uint16_t chassis_voltage;
  uint16_t chassis_current;
  float chassis_power;
  uint16_t buffer_energy;
  uint16_t shooter_17mm_1_barrel_heat;
  uint16_t shooter_17mm_2_barrel_heat;
  uint16_t shooter_42mm_barrel_heat;
}power_heat_data_t;

typedef __packed struct
{
 uint8_t bullet_type;
 uint8_t shooter_number;
 uint8_t launching_frequency;
 float initial_speed;
}shoot_data_t;

extern robot_status_t robot_status;
extern power_heat_data_t power_heat_data;
extern shoot_data_t shoot_data;

#define CRC_Check_False   0
#define CRC_Check_True    1

//屏幕分辨率1920x1080
#define SCREEN_WIDTH 1080
#define SCREEN_LENGTH 1920


typedef unsigned char u8;



typedef __packed struct
{
  uint8_t graphic_name[3];
  uint32_t operate_tpye : 3;
  uint32_t graphic_tpye : 3;
  uint32_t layer : 4;
  uint32_t color : 4;
  uint32_t start_angle : 9;
  uint32_t end_angle : 9;
  uint32_t width : 10;
  uint32_t start_x : 11;
  uint32_t start_y : 11;
  uint32_t radius : 10;
  uint32_t end_x : 11;
  uint32_t end_y : 11;
} graphic_data_struct_t;

//客户端绘制图形
typedef __packed struct
{
  graphic_data_struct_t grapic_data_struct[1];//绘制图像的数量即图像数据数组的长度，但要看清楚裁判系统给的增加图形数量对应的内容ID
} ext_client_custom_graphic_t;


//交互数据信息
typedef __packed struct
{
  uint16_t data_cmd_id;	//数据段内容ID
  uint16_t sender_ID;	//发送者ID
  uint16_t receiver_ID;	//接受者ID
  ext_client_custom_graphic_t graphic_custom;//自定义图形数据
}ext_student_interactive_header_data_t;



unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);

uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void referee_data_pack_handle(uint8_t sof, uint16_t cmd_id, uint8_t *p_data, uint16_t len);



void communication_Read_Data(uint8_t *data);

uint16_t get_Power_Limit(void);
uint16_t get_17Heat_Limit(void);
uint16_t get_42Heat_Limit(void);

uint16_t get_Power_Now(void);
uint16_t get_17Heat_1_Now(void);
uint16_t get_17Heat_2_Now(void);
uint16_t get_42Heat_Now(void);

#endif