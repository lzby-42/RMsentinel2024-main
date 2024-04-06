#ifndef __MOTO_SHOOT_TASK__
#define __MOTO_SHOOT_TASK__

#include "main.h"
extern volatile uint8_t shoot_flag_1;
extern volatile uint8_t shoot_flag_2;


void Moto_Shoot1_Task(void const * argument);
void Moto_Shoot2_Task(void const * argument);
void waiting_Moto_shoot(void);

#endif // !__MOTO_SHOOT_TASK__
