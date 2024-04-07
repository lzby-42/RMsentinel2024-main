/**
 * @file judgeSys_Task.C
 * @author DreamKerman (2829478110@qq.com)
 * @brief 裁判系统处理线程
 * @version 0.1
 * @date 2023-12-24
 *
 * @copyright 鼎行双创 (c) 2023
 *
 */

#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "judgeSys_Task.h"

#include "usart.h"
#include "judgeSys.h"

ext_student_interactive_header_data_t custom_grapic_draw;			//自定义图像绘制
ext_client_custom_graphic_t custom_graphic;	//自定义图像

void judgeSys_Task(void const *argument)
{
    uint8_t i = 0;//记数标定十hz

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    osDelay(100);
    // custom_grapic_draw.data_cmd_id = UI_Data_ID_Draw1;//绘制一个图形（内容ID，查询裁判系统手册）

    // custom_grapic_draw.sender_ID = UI_Data_RobotID_BStandard3;//发送者ID，机器人对应ID，此处为蓝方英雄
    // custom_grapic_draw.receiver_ID = UI_Data_CilentID_BStandard3;//接收者ID，操作手客户端ID
    //自定义图像数据

    // custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[0] = '1';
    // custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[1] = '0';
    // custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_name[2] = '0';//图形名
    // //上面三个字节代表的是图形名，用于图形索引，可自行定义
    // custom_grapic_draw.graphic_custom.grapic_data_struct[0].operate_tpye = UI_Graph_ADD;//图形操作，0：空操作；1：增加；2：修改；3：删除；
    // custom_grapic_draw.graphic_custom.grapic_data_struct[0].graphic_tpye = UI_Graph_Line;//图形类型，0为直线，其他的查看用户手册
    // custom_grapic_draw.graphic_custom.grapic_data_struct[0].layer = 1;//图层数
    // custom_grapic_draw.graphic_custom.grapic_data_struct[0].color = UI_Color_Orange;//颜色
    // custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_angle = 0;
    // custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_angle = 0;
    // custom_grapic_draw.graphic_custom.grapic_data_struct[0].width = 13;//线宽
    // custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_x = 980;
    // custom_grapic_draw.graphic_custom.grapic_data_struct[0].start_y = 540;
    // custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_x = 980;
    // custom_grapic_draw.graphic_custom.grapic_data_struct[0].end_y = 240;
    // custom_grapic_draw.graphic_custom.grapic_data_struct[0].radius = 0;
    



    judgment_Protect = 0;
    HAL_UART_Receive_DMA(&huart8, judgment_System_Signal_Buffer, JUDGEMENT_SIZE);
    while (1)
    {

            communication_Read_Data(judgment_System_handle_buffer);
            judgment_Protect = 0;
        

        // if (i >= 10)
        // {
        //     referee_data_pack_handle(UI_SOF, UI_CMD_Robo_Exchange, (uint8_t *)&custom_grapic_draw, sizeof(custom_grapic_draw));
        //     i = 0;

        // }
        // i++;

        vTaskDelayUntil(&xLastWakeTime, 5);
    }
}
