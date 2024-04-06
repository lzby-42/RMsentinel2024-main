/*
can配置可以移植




*/

#include "canmotor.h"
#include "main.h"
#include "can.h"
#include "string.h"
#include "tool.h"

uint8_t Data_Enable[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC };		//达妙电机使能命令
uint8_t Data_Failure[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD };		//电机失能命令
uint8_t Data_Save_zero[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE };	    //电机保存零点命令

moto_info_t val_1[16];
moto_info_t val_2[16];
uint8_t  rx_data[8] = { 0 };

// int  m6020_angel, val_1cs7speed, val_1cs7angle, val_1cs6speed;
extern  float out1, out2, out3, out4;
int motorval1, motorval2, motorval3, motorval4, motorvalleft = 3, motorvalright;
int16_t can_error = 0, can_ok = 0, can_error1 = 0;
uint8_t can5FIFO_flag = 1;
uint8_t canCFIFO_flag = 1;
void can_filter_init(void)									//过滤器配置
{
  CAN_FilterTypeDef can_filter_st;
  can_filter_st.FilterActivation = ENABLE;
  can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter_st.FilterIdHigh = 0x0000;
  can_filter_st.FilterIdLow = 0x0000;
  can_filter_st.FilterMaskIdHigh = 0x0000;
  can_filter_st.FilterMaskIdLow = 0x0000;
  can_filter_st.FilterBank = 0;
  can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
  HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_IRQHandler(&hcan1);
}
void can2_filter_init(void)									//过滤器配置
{
  CAN_FilterTypeDef can_filter_st;
  can_filter_st.FilterActivation = ENABLE;
  can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter_st.FilterIdHigh = 0x0000;
  can_filter_st.FilterIdLow = 0x0000;
  can_filter_st.FilterMaskIdHigh = 0x0000;
  can_filter_st.FilterMaskIdLow = 0x0000;
  can_filter_st.FilterBank = 14;
  can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
  HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_IRQHandler(&hcan2);
}

void can_output(int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  CAN_TxHeaderTypeDef CAN1_TxHander;                      //发送的结构体
  uint8_t TxDate[8] = { 0 };   							    //放数值的数组
  CAN1_TxHander.DLC = 0x08;									//发送的位数
  CAN1_TxHander.IDE = CAN_ID_STD;							//标准位
  CAN1_TxHander.RTR = CAN_RTR_DATA;							//拓展位
  CAN1_TxHander.StdId = 0x200;								//发送的地址
  TxDate[0] = v1 >> 8;
  TxDate[1] = v1;
  TxDate[2] = v2 >> 8;
  TxDate[3] = v2;
  TxDate[4] = v3 >> 8;
  TxDate[5] = v3;
  TxDate[6] = v4 >> 8;
  TxDate[7] = v4;											//1到4的电机的值
  HAL_CAN_AddTxMessage(&hcan1, &CAN1_TxHander, TxDate, 0);	//发送数据函数	
}


void can_output_gimbal(int16_t v1, int16_t v2, int16_t v3, int16_t v4) //3,4m2006
{
  CAN_TxHeaderTypeDef CAN1_TxHander;                      //发送的结构体
  uint8_t TxDate[8] = { 0 };   							    //放数值的数组
  CAN1_TxHander.DLC = 0x08;									//发送的位数
  CAN1_TxHander.IDE = CAN_ID_STD;							//标准位
  CAN1_TxHander.RTR = CAN_RTR_DATA;							//拓展位
  CAN1_TxHander.StdId = 0x1ff;								//发送的地址
  TxDate[0] = v1 >> 8;
  TxDate[1] = v1;
  TxDate[2] = v2 >> 8;
  TxDate[3] = v2;
  TxDate[4] = v3 >> 8;
  TxDate[5] = v3;
  TxDate[6] = v4 >> 8;
  TxDate[7] = v4;											//1到4的电机的值

  if (HAL_CAN_AddTxMessage(&hcan1, &CAN1_TxHander, TxDate, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK) //发送数据函数	
  {
    if (HAL_CAN_AddTxMessage(&hcan1, &CAN1_TxHander, TxDate, (uint32_t *)CAN_TX_MAILBOX1) != HAL_OK)
    {
      HAL_CAN_AddTxMessage(&hcan1, &CAN1_TxHander, TxDate, (uint32_t *)CAN_TX_MAILBOX2);
    }
  }
}



/**
 * @brief  达妙电机位置速度模式控下控制帧
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  _pos   位置给定
 * @param  _vel   速度给定
 */
void PosSpeed_CtrlMotor(CAN_HandleTypeDef *hcan, uint16_t id, float _pos, float _vel)
{
  static CAN_TxHeaderTypeDef Tx_Header;
  uint8_t TxDate[8] = { 0 };   							    //放数值的数组
  uint8_t *pbuf, *vbuf;
  pbuf = (uint8_t *)&_pos;
  vbuf = (uint8_t *)&_vel;

  Tx_Header.StdId = id;
  Tx_Header.IDE = CAN_ID_STD;
  Tx_Header.RTR = CAN_RTR_DATA;
  Tx_Header.DLC = 0x08;

  TxDate[0] = *pbuf;
  TxDate[1] = *(pbuf + 1);
  TxDate[2] = *(pbuf + 2);
  TxDate[3] = *(pbuf + 3);
  TxDate[4] = *vbuf;
  TxDate[5] = *(vbuf + 1);
  TxDate[6] = *(vbuf + 2);
  TxDate[7] = *(vbuf + 3);

  //找到空的发送邮箱，把数据发送出去
  if (HAL_CAN_AddTxMessage(hcan, &Tx_Header, TxDate, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK) //
  {
    if (HAL_CAN_AddTxMessage(hcan, &Tx_Header, TxDate, (uint32_t *)CAN_TX_MAILBOX1) != HAL_OK)
    {
      HAL_CAN_AddTxMessage(hcan, &Tx_Header, TxDate, (uint32_t *)CAN_TX_MAILBOX2);
    }
  }
}

/**
 * @brief  速度模式控下控制帧
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  _vel   速度给定
 */
void Speed_CtrlMotor(CAN_HandleTypeDef *hcan, uint16_t ID, float _vel)
{
  static CAN_TxHeaderTypeDef   Tx_Header;
  uint8_t TxDate[8] = { 0 };   							    //放数值的数组
  uint8_t *vbuf;
  vbuf = (uint8_t *)&_vel;

  Tx_Header.StdId = ID;
  Tx_Header.IDE = CAN_ID_STD;
  Tx_Header.RTR = CAN_RTR_DATA;
  Tx_Header.DLC = 0x04;

  TxDate[0] = *vbuf;
  TxDate[1] = *(vbuf + 1);
  TxDate[2] = *(vbuf + 2);
  TxDate[3] = *(vbuf + 3);

  //找到空的发送邮箱，把数据发送出去
  if (HAL_CAN_AddTxMessage(hcan, &Tx_Header, TxDate, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK) //
  {
    if (HAL_CAN_AddTxMessage(hcan, &Tx_Header, TxDate, (uint32_t *)CAN_TX_MAILBOX1) != HAL_OK)
    {
      HAL_CAN_AddTxMessage(hcan, &Tx_Header, TxDate, (uint32_t *)CAN_TX_MAILBOX2);
    }
  }
}


void set_motor_voltage_can2_low(int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  //	HAL_StatusTypeDef errorsave;
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];

  tx_header.StdId = 0x200;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 0x08;

  tx_data[0] = (v1 >> 8);
  tx_data[1] = (v1);
  tx_data[2] = (v2 >> 8);
  tx_data[3] = (v2);
  tx_data[4] = (v3 >> 8);
  tx_data[5] = (v3);
  tx_data[6] = (v4 >> 8);
  tx_data[7] = (v4);
  /*找到空的发送邮箱，把数据发送出去*/
  if (HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK) //
  {
    if (HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, (uint32_t *)CAN_TX_MAILBOX1) != HAL_OK)
    {
      HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, (uint32_t *)CAN_TX_MAILBOX2);
    }
  }
}


void set_motor_voltage_can2_hig(int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  //	HAL_StatusTypeDef errorsave;
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];

  tx_header.StdId = 0x1FF;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 0x08;

  tx_data[0] = (v1 >> 8);
  tx_data[1] = (v1);
  tx_data[2] = (v2 >> 8);
  tx_data[3] = (v2);
  tx_data[4] = (v3 >> 8);
  tx_data[5] = (v3);
  tx_data[6] = (v4 >> 8);
  tx_data[7] = (v4);
  /*找到空的发送邮箱，把数据发送出去*/
  if (HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK) //
  {
    if (HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, (uint32_t *)CAN_TX_MAILBOX1) != HAL_OK)
    {
      HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, (uint32_t *)CAN_TX_MAILBOX2);
    }
  }
}

uint8_t CANx_SendStdData(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *pData, uint16_t Len)//自定义信息发送
{
  static CAN_TxHeaderTypeDef   Tx_Header;

  Tx_Header.StdId = ID;
  Tx_Header.ExtId = 0;
  Tx_Header.IDE = 0;
  Tx_Header.RTR = 0;
  Tx_Header.DLC = Len;


  /*找到空的发送邮箱，把数据发送出去*/
  if (HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK) //
  {
    if (HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t *)CAN_TX_MAILBOX1) != HAL_OK)
    {
      HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t *)CAN_TX_MAILBOX2);
    }
  }
  return 0;
}



int idcs;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t             rx_data1[8];
  uint8_t index;
  
  if (hcan->Instance == CAN1)
  {

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can data   接收can数据	
    if (rx_header.StdId == 0x58)
    {
      val_1[7].rotor_angle = (rx_data[1] << 8) | rx_data[2];
      val_1[7].rotor_speed = (rx_data[3] << 4) | (rx_data[4] >> 4);
      val_1[7].torque_current = (rx_data[4] << 4) | rx_data[5];
      val_1[7].temp = rx_data[6] > rx_data[7] ? rx_data[6] : rx_data[7];//取高温
      can5FIFO_flag = 0;
    }
    if ((rx_header.StdId >= FEEDBACK_ID_BASE)
      && (rx_header.StdId < FEEDBACK_ID_BASE + MOTOR_MAX_NUM))       // judge the can id   判断can编号
    {

      canCFIFO_flag = 0;
      index = rx_header.StdId - FEEDBACK_ID_BASE;            // get motor index by can_id    通过can_id获取运动指标		
      val_1[index].rotor_angle = ((rx_data[0] << 8) | rx_data[1]);
      val_1[index].rotor_speed = ((rx_data[2] << 8) | rx_data[3]);
      val_1[index].set_voltage = ((rx_data[4] << 8) | rx_data[5]);

    }


    memcpy(&val_2[0], &val_1[0], sizeof(moto_info_t) * 8);



  }
  if (hcan->Instance == CAN2)
  {

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data1); //receive can data   接收can数据
    if ((rx_header.StdId >= FEEDBACK_ID_BASE)
      && (rx_header.StdId < FEEDBACK_ID_BASE + MOTOR_MAX_NUM))       // judge the can id   判断can编号
    {

      index = rx_header.StdId - FEEDBACK_ID_BASE + 8;            // get motor index by can_id    通过can_id获取运动指标
      idcs = index;
      val_1[index].rotor_angle = ((rx_data1[0] << 8) | rx_data1[1]);
      val_1[index].rotor_speed = ((rx_data1[2] << 8) | rx_data1[3]);
      val_1[index].torque_current = ((rx_data1[4] << 8) | rx_data1[5]);

    }

    memcpy(&val_2[9], &val_1[9], sizeof(moto_info_t) * 8);


  }
}

//void can2_user_init(CAN_HandleTypeDef* hcan )
//{
//  CAN_FilterTypeDef  can_filter;

//  can_filter.FilterBank = 14;                       // filter 0
//  can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;  // mask mode
//  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
//  can_filter.FilterIdHigh = 0;
//  can_filter.FilterIdLow  = 0;
//  can_filter.FilterMaskIdHigh = 0;
//  can_filter.FilterMaskIdLow  = 0;                 // set mask 0 to receive all can id
//  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0
//  can_filter.FilterActivation = ENABLE;           // enable can filter
//  can_filter.SlaveStartFilterBank  = 14;          // only meaningful in dual can mode
//  
//  HAL_CAN_ConfigFilter(hcan, &can_filter);        // init can filter
//  HAL_CAN_Start(hcan);                          // start can1
//  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // enable can1 rx interrupt
//	
//	
//}
