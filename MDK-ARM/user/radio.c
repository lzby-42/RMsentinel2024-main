#include "main.h"
#include "radio.h"
#include "string.h"

#include "usart.h"
#define uint8_t unsigned char
uint8_t radio_data_cacre[RADIO_DATA_SIZE] = { 0 };
uint8_t radio_data[RADIO_DATA_SIZE] = { 0 };

radio_massage_send_t radio_massage_send;
radio_massage_receive_t radio_massage_receive;

void radio_read_data(void)
{
    uint8_t i = 0;
        for (i = 0; i < RADIO_DATA_SIZE - sizeof(radio_massage_receive_t); i++)
        {
            if (radio_data[i] == 0x5A)
            {
                memcpy(&radio_massage_receive, &radio_data[i], sizeof(radio_massage_receive_t));
            }
            
        }


}