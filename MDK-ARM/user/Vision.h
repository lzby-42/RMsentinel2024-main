#ifndef   __VISION_H__
#define  __VISION_H__

#define  VISION_BUFFER_LENGTH 22

extern uint16_t time_count;
extern uint8_t visioning_flag;
extern volatile uint8_t visioning_flag_shoot;
extern float Data_yaw, Data_pit;
extern float last_yaw_angle, last_pit_angle;

extern uint8_t  Com4_Vision_Buffer[VISION_BUFFER_LENGTH];
extern uint8_t  Vision_Buffer[VISION_BUFFER_LENGTH];
void vision_read(void);
void vinion_old_Version(void);
// void auto_Aim(float kpp, float kip, float kpy, float kiy, float kdp, float kdy);
void auto_Aim(void);
#endif
