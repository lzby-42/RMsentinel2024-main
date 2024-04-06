#ifndef RADIO_H
#define RADIO_H
#include "tool.h"


typedef __packed struct
{
    uint8_t  head;
    float aim_x;
    float roll;
    float pitch;
    float yaw;
    uint16_t checksum;
}radio_massage_send_t;

typedef __packed struct
{
    uint8_t  head;
    float linear_x;
    float linear_y;
    float linear_z;
    float angular_x;
    float angular_y;
    float angular_z;
    uint16_t checksum;
}radio_massage_receive_t;
#define RADIO_DATA_SIZE 100 

extern radio_massage_send_t radio_massage_send;
extern radio_massage_receive_t radio_massage_receive;
extern uint8_t radio_data_cacre[RADIO_DATA_SIZE];
extern uint8_t radio_data[RADIO_DATA_SIZE];

#endif // !RADIO_H
