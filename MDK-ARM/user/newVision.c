#include "newVision.h"
#include "usart.h"		 

#include "stdio.h"
#include "cmsis_os.h"
#include "string.h"

#include "bsp_imu.h"
#include "judgeSys.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "pid.h"
#include <math.h>


#define RECORD_VALUE(x)         			\
    {                           			\
        if ((x) >= -10 && (x) <= 10)  \
            l_value = (x); 						\
        else if((x)==0)               \
            l_value=0; 								\
				else													\
						(x) = l_value;						\
    }											
#define RECORD_VALUE2(x)         			\
{                           					\
		if ((x) >= -14 && (x) <= 14)  		\
				l_value2 = (x); 							\
	  else if((x)==0)               		\
            l_value2=0; 							\
				else													\
						(x) = l_value2;					\
}

extern float yaw_angle, pit_angle;
extern float yaw_angle_now, pit_angle_now;
short l_value, l_value2;
SendPacketVision_s SendPacketVision;

extern frame_header_t frame_header_rx;
extern robot_status_t robot_status;
extern power_heat_data_t power_heat_data;


extern float b;								 //存放pitch值
extern float a;
extern int jieshou;
float pitch = 0.0f;
float yaw = 0.0f;
extern uint16_t res;

float h = -0.3f;
float b_set_angle;
float buchang;

union X16ToFl
{
    float out;
    uint8_t in[4];
}Yaw, Pit;


VisionSendHeader_t    VisionSendHeader;  //头
VisionRecvData_t      VisionRecvData;    //视觉接收结构体
VisionSendData_t      VisionSendData;   //视觉接收结构体

ReceivedPacketVision_s ReceivedPacketVision;
struct SolveTrajectoryParams st;

uint8_t Com4_newVision_Buffer[VISION_BUFFER_LEN] = { 0 };
uint8_t newVision_Buffer[VISION_BUFFER_LEN] = { 0 };

uint8_t ask_To_0x(uint8_t ch)
{
    uint8_t res;
    if (ch > '9')
    {
        res = ch - 'A' + 10;
    }
    else if (ch == 0)
    {
        res = 0;
    }
    else
    {
        res = ch - '0';
    }

    return res;
}
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t receive_high_water;
#endif

struct tar_pos tar_position[4]; //装甲板
float t = 0.5f; // 飞行时间
extern float kry;
#define KK 0.092
int gonglv, gonglv2;
float aim_x = 0, aim_y = 0, aim_z = 0; // aim point 落点，传回上位机用于可视化
float pitch2 = 0; //输出控制量 pitch绝对值 弧度
float yaw2 = 0;   //yaw
float fan;
uint8_t vision_check = 0;
float s = 0;

void vision_new_version(void)
{
    for (uint8_t i = 0; i < VISION_BUFFER_LEN - sizeof(ReceivedPacketVision_s) + 1; i++)
    {
        if (newVision_Buffer[i] == 0xA5 && Verify_CRC16_Check_Sum(&newVision_Buffer[i], 48))
        {
            memcpy(&ReceivedPacketVision, &newVision_Buffer[i], sizeof(ReceivedPacketVision_s));
            vision_check = 1;
            break;
        }
    }


    st.k = KK;
    st.bullet_type = BULLET_17;
    st.current_v = 10.0f;
    st.current_pitch = 0;
    st.current_yaw = 0;
    st.xw = ReceivedPacketVision.x;
    st.yw = ReceivedPacketVision.y;
    st.zw = ReceivedPacketVision.z;

    st.vxw = ReceivedPacketVision.vx;
    st.vyw = ReceivedPacketVision.vy;
    st.vzw = ReceivedPacketVision.vz;
    st.v_yaw = ReceivedPacketVision.v_yaw;
    st.tar_yaw = ReceivedPacketVision.yaw;
    st.r1 = ReceivedPacketVision.r1;
    st.r2 = ReceivedPacketVision.r2;
    st.dz = ReceivedPacketVision.dz;
    st.bias_time = 10;
    st.s_bias = 0.085f;
    st.z_bias = 0.265f;
    st.armor_id = ARMOR_INFANTRY3;
    st.armor_num = ARMOR_NUM_NORMAL;


    usb_send_vision();
    autoSolveTrajectory(&pitch2, &yaw2, &aim_x, &aim_y, &aim_z);

    float angle2, a2;
    //弹道解算
    angle2 = acos(ReceivedPacketVision.x / (sqrt(ReceivedPacketVision.x * ReceivedPacketVision.x + h * h)));
    a2 = (h * st.current_v * st.current_v + GRAVITY * ReceivedPacketVision.x * ReceivedPacketVision.x) / (st.current_v * st.current_v);
    fan = a2 / (sqrt(ReceivedPacketVision.x * ReceivedPacketVision.x + h * h));
    if (fan >= 1) fan = 0.9999;
    else	if (fan <= -1) fan = -0.9999;
    else fan = fan;
    b_set_angle = (asin(fan) - angle2) * 0.5;
    yaw2 = atan2f(aim_y, aim_x);
    // s = sqrt((aim_x) * (aim_x) + (aim_y) * (aim_y)) - st.s_bias;
    // t = (float)(s / (st.current_v * acosf(atan2f(st.current_v, s))));
    // t = (float)((exp(KK * s) - 1.0f) / (KK * v * acosf(angle)));
    

    //系统延时


#if INCLUDE_uxTaskGetStackHighWaterMark
    receive_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
}





void Usart_SendFloat(USART_TypeDef *USARTx, float data)
{
    float v_float;
    unsigned char *char_p = NULL;

    v_float = data;
    char_p = (unsigned char *)(&v_float);

    HAL_UART_Transmit_DMA(&huart6, char_p, 4);
    huart6.gState = HAL_UART_STATE_READY;
}



void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    HAL_UART_Transmit_DMA(&huart6, Array, Length);
    huart6.gState = HAL_UART_STATE_READY;
}


void Serial_SendArray2(uint8_t *Array, uint16_t Length)
{
    HAL_UART_Transmit_DMA(&huart6, Array, Length);
    huart6.gState = HAL_UART_STATE_READY;
}

void Vision_Send_Data2(uint16_t Byte)
{
    HAL_UART_Transmit_DMA(&huart6, (uint8_t *)&Byte, 2);
    huart6.gState = HAL_UART_STATE_READY;
}







uint8_t usb_tx_buf[APP_TX_DATA_SIZE];
void usb_send_vision(void)
{
    uint8_t crc_ok = Verify_CRC16_Check_Sum((uint8_t *)(&ReceivedPacketVision), sizeof(ReceivedPacketVision));
    SendPacketVision.header = SET_OUTPUT_VISION_HEDER; // ??? crc_ok
    if (crc_ok)
    {
        SendPacketVision.detect_color = robot_status.robot_id < 99 ? 1 : 0;
        SendPacketVision.reset_tracker = 0;
        SendPacketVision.reserved = 0;
        SendPacketVision.roll = 0;
        SendPacketVision.pitch = (6150 - pit_angle_now) * 2.0f * Pi / 8191.0f;
        SendPacketVision.yaw = (-imu.yaw) * Pi / 180.0f;
        SendPacketVision.aim_x = aim_x;
        SendPacketVision.aim_y = aim_y;
        SendPacketVision.aim_z = aim_z;

    }
    memcpy(usb_tx_buf, &SendPacketVision, sizeof(SendPacketVision_s));
    Append_CRC16_Check_Sum(usb_tx_buf, 28);
    Serial_SendArray(usb_tx_buf, sizeof(usb_tx_buf));
}



/*
@brief 单方向空气阻力模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
float monoDirectionalAirResistanceModel(float s, float v, float angle)
{



    //    float z;
    //    //t为给定v与angle时的飞行时间
    //		//t=(float)(s/(v * AHRS_cosf(angle)));
    //    t = (float)((exp(KK * s) - 1.0f) / (KK * v * AHRS_cosf(angle)));
    //    //z为给定v与angle时的高度
    //    z = (float)(v * AHRS_sinf(angle) * t - GRAVITY * t * t / 2.0f);
    return 0;
}


// /*
// @brief pitch解算
// @param s:m 距离
// @param z:m 高度
// @param v:m/s
// @return angle_pitch:rad
// */
// float angle_pitch;
// float s1, s2;
// float pitchTrajectoryCompensation(float s, float z, float v)
// {



//     float z_temp, z_actual, dz;

//     int i = 0;
//     z_temp = z;
//     // iteration
//     for (i = 0; i < 10; i++)
//     {


//         angle_pitch = AHRS_atan2f(z_temp, s); // rad
//         z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);
//         dz = 0.02f * (z - z_actual);
//         z_temp = z_temp + dz;
//         if (fabs(dz) < 0.00001f)
//         {
//             break;
//         }
//     }
//     return angle_pitch;
// }

/*
@brief 根据最优先决策得出被打击的装甲板 自动弹道解算
@param pitch:rad  传出pitch
@param yaw:rad    传出yaw
@param aim_x:传出aim_x  打击目标的x
@param aim_y:传出aim_y  打击目标的y
@param aim_z:传出aim_z  打击目标的z
*/


void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z)
{





    //线性预测
    float timeDelay = st.bias_time / 1000.0 + t;
    st.tar_yaw += st.v_yaw * timeDelay;

    //      2
    //   3     1
    //      0
    int use_1 = 1;
    int i = 0;
    int idx = 0; // 选择装甲板

    for (i = 0; i < 4; i++)
    {
        float tmp_yaw = st.tar_yaw + i * PI / 2.0f;
        float r = use_1 ? st.r1 : st.r2;
        tar_position[i].x = st.xw - r * cos(tmp_yaw);
        tar_position[i].y = st.yw - r * sin(tmp_yaw);
        tar_position[i].z = use_1 ? st.zw : st.zw + st.dz;
        tar_position[i].yaw = tmp_yaw;
        use_1 = !use_1;
    }


    //计算枪管到目标装甲板yaw最小的那个装甲板
    float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);
    for (i = 1; i < 4; i++)
    {
        float temp_yaw_diff = fabsf(*yaw - tar_position[i].yaw);
        if (temp_yaw_diff < yaw_diff_min)
        {
            yaw_diff_min = temp_yaw_diff;
            idx = i;
        }
    }





    *aim_z = tar_position[idx].z + st.vzw * timeDelay;
    *aim_x = tar_position[idx].x + st.vxw * timeDelay;
    *aim_y = tar_position[idx].y + st.vyw * timeDelay;
    //???????


//    *pitch = -pitchTrajectoryCompensation
//				(
//				sqrt((*aim_x)*(*aim_x)+(*aim_y)*(*aim_y))-st.s_bias,    																								//s
//        *aim_z + st.z_bias, 																		//z
//				st.current_v																						//v
//				);
    *yaw = (float)(atan2f(*aim_y, *aim_x));

}





