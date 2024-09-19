/*
这个文件靠移植代码
还差点东西    HAL_NVIC_SetPriority(USART1_IRQn, 1, 1);
			  HAL_NVIC_EnableIRQ(USART1_IRQn);      这两句加在 MX_USART1_UART_Init 串口初始化里边

			  uart_receive_handler(&huart1);        这句加在usart中断里边

			  dbus_uart_init                        这句加在main里边

*/
#include "string.h"
#include "stdlib.h"
#include "remoter.h"
#include "usart.h"
#include "main.h"
#include "math.h"
uint8_t   dbus_buf[DBUS_BUFLEN];
rc_info_t rc;
extern int motorval1, motorval2, motorval3, motorval4, motorvalleft, motorvalright;
int bolun = 1024;
uint8_t busTime = 0;
static int uart_receive_dma_no_it(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size)
{

	uint32_t tmp1 = 0;

	tmp1 = huart->RxState;

	if (tmp1 == HAL_UART_STATE_READY)
	{
		if ((pData == NULL) || (Size == 0))
		{
			return HAL_ERROR;
		}

		huart->pRxBuffPtr = pData;
		huart->RxXferSize = Size;
		huart->ErrorCode = HAL_UART_ERROR_NONE;

		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);


		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
}

uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{
	return ((uint16_t)(dma_stream->NDTR));
}

void rc_callback_handler(rc_info_t *rc, uint8_t *buff)
{

	rc->ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
	rc->ch1 -= 1024;
	rc->ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
	rc->ch2 -= 1024;
	rc->ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
	rc->ch3 -= 1024;
	rc->ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
	rc->ch4 -= 1024;

	rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
	rc->sw2 = (buff[5] >> 4) & 0x0003;
	rc->ch5 = (buff[16] | buff[17] << 8) & 0x07FF;
	//rc.bolun.b = ((int16_t)buff[16] | ((int16_t)buff[17] << 8)) & 0x07FF;  //拨轮

	motorval1 = abs(rc->ch1) > 10 ? rc->ch1 : 0;
	motorval2 = abs(rc->ch2) > 10 ? rc->ch2 : 0;
	motorval3 = abs(rc->ch3) > 10 ? rc->ch3 : 0;
	motorval4 = abs(rc->ch4) > 10 ? rc->ch4 : 0;
	motorvalleft = rc->sw1;
	motorvalright = rc->sw2;
	bolun = rc->ch5;
	if ((abs(rc->ch1) > 660) || \
		(abs(rc->ch2) > 660) || \
		(abs(rc->ch3) > 660) || \
		(abs(rc->ch4) > 660))
	{
		memset(rc, 0, sizeof(rc_info_t));
	}
	busTime = 0;

}

static void uart_rx_idle_callback(UART_HandleTypeDef *huart)
{
	__HAL_UART_CLEAR_IDLEFLAG(huart);


	if (huart == &DBUS_HUART)
	{
		__HAL_DMA_DISABLE(huart->hdmarx);

		if ((DBUS_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) == DBUS_BUFLEN)
		{
			rc_callback_handler(&rc, dbus_buf);
		}


		__HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_MAX_LEN);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}

void uart_receive_handler(UART_HandleTypeDef *huart)
{
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
		__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		uart_rx_idle_callback(huart);
	}
}


void dbus_uart_init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
	__HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);

	uart_receive_dma_no_it(&DBUS_HUART, dbus_buf, DBUS_MAX_LEN);
}
