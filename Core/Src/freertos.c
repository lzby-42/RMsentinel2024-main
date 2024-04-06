/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */



#include "pid.h"
#include "canmotor.h"



#include "gpio.h"
#include "usart.h"
#include "tool.h"
#include "Vision.h"
#include "tim.h"
#include "bsp_imu.h"

#include "judgeSys.h"

extern imu_t  imu;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern  float out1, out2, out3, out4;
extern int motorval1, motorval2, motorval3, motorval4, motorvalleft, motorvalright;
extern int shangdan;
extern float out5_1, out5_1_2;


extern int yval, bzw5, bzw6, outyaw, outpit;



//int w=1;
//int v=2;
const uint8_t c[1] = { 12 };
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId chassisHandle;
osThreadId gimbalHandle;
osThreadId judgeSysHandle;
osThreadId visionSysHandle;
osThreadId Moto_Shoot1Handle;
uint32_t Moto_Shoot1Buffer[ 128 ];
osStaticThreadDef_t Moto_Shoot1ControlBlock;
osThreadId Moto_Shoot2Handle;
uint32_t Moto_Shoot2Buffer[ 128 ];
osStaticThreadDef_t Moto_Shoot2ControlBlock;
osThreadId saveControlHandle;
osThreadId IMUHandle;
osTimerId saferHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void chassis_Task(void const * argument);
void gimbal_Task(void const * argument);
void judgeSys_Task(void const * argument);
void visionSys_Task(void const * argument);
void Moto_Shoot1_Task(void const * argument);
void Moto_Shoot2_Task(void const * argument);
void saveControl_Task(void const * argument);
void IMU_Task(void const * argument);
void saferStart(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer, StackType_t** ppxIdleTaskStackBuffer, uint32_t* pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
    /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of safer */
  osTimerDef(safer, saferStart);
  saferHandle = osTimerCreate(osTimer(safer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of chassis */
  osThreadDef(chassis, chassis_Task, osPriorityNormal, 0, 256);
  chassisHandle = osThreadCreate(osThread(chassis), NULL);

  /* definition and creation of gimbal */
  osThreadDef(gimbal, gimbal_Task, osPriorityAboveNormal, 0, 256);
  gimbalHandle = osThreadCreate(osThread(gimbal), NULL);

  /* definition and creation of judgeSys */
  osThreadDef(judgeSys, judgeSys_Task, osPriorityNormal, 0, 128);
  judgeSysHandle = osThreadCreate(osThread(judgeSys), NULL);

  /* definition and creation of visionSys */
  osThreadDef(visionSys, visionSys_Task, osPriorityNormal, 0, 128);
  visionSysHandle = osThreadCreate(osThread(visionSys), NULL);

  /* definition and creation of Moto_Shoot1 */
  osThreadStaticDef(Moto_Shoot1, Moto_Shoot1_Task, osPriorityAboveNormal, 0, 128, Moto_Shoot1Buffer, &Moto_Shoot1ControlBlock);
  Moto_Shoot1Handle = osThreadCreate(osThread(Moto_Shoot1), NULL);

  /* definition and creation of Moto_Shoot2 */
  osThreadStaticDef(Moto_Shoot2, Moto_Shoot2_Task, osPriorityAboveNormal, 0, 128, Moto_Shoot2Buffer, &Moto_Shoot2ControlBlock);
  Moto_Shoot2Handle = osThreadCreate(osThread(Moto_Shoot2), NULL);

  /* definition and creation of saveControl */
  osThreadDef(saveControl, saveControl_Task, osPriorityNormal, 0, 128);
  saveControlHandle = osThreadCreate(osThread(saveControl), NULL);

  /* definition and creation of IMU */
  osThreadDef(IMU, IMU_Task, osPriorityNormal, 0, 128);
  IMUHandle = osThreadCreate(osThread(IMU), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  osDelay(100);
    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_chassis_Task */
/**
* @brief Function implementing the chassis thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chassis_Task */
__weak void chassis_Task(void const * argument)
{
  /* USER CODE BEGIN chassis_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END chassis_Task */
}

/* USER CODE BEGIN Header_gimbal_Task */
/**
* @brief Function implementing the gimbal thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gimbal_Task */
__weak void gimbal_Task(void const * argument)
{
  /* USER CODE BEGIN gimbal_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END gimbal_Task */
}

/* USER CODE BEGIN Header_judgeSys_Task */
/**
* @brief Function implementing the judgeSys thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_judgeSys_Task */
__weak void judgeSys_Task(void const * argument)
{
  /* USER CODE BEGIN judgeSys_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END judgeSys_Task */
}

/* USER CODE BEGIN Header_visionSys_Task */
/**
* @brief Function implementing the visionSys thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_visionSys_Task */
__weak void visionSys_Task(void const * argument)
{
  /* USER CODE BEGIN visionSys_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END visionSys_Task */
}

/* USER CODE BEGIN Header_Moto_Shoot1_Task */
/**
* @brief Function implementing the Moto_Shoot1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Moto_Shoot1_Task */
__weak void Moto_Shoot1_Task(void const * argument)
{
  /* USER CODE BEGIN Moto_Shoot1_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Moto_Shoot1_Task */
}

/* USER CODE BEGIN Header_Moto_Shoot2_Task */
/**
* @brief Function implementing the Moto_Shoot2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Moto_Shoot2_Task */
__weak void Moto_Shoot2_Task(void const * argument)
{
  /* USER CODE BEGIN Moto_Shoot2_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Moto_Shoot2_Task */
}

/* USER CODE BEGIN Header_saveControl_Task */
/**
* @brief Function implementing the saveControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_saveControl_Task */
__weak void saveControl_Task(void const * argument)
{
  /* USER CODE BEGIN saveControl_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END saveControl_Task */
}

/* USER CODE BEGIN Header_IMU_Task */
/**
* @brief Function implementing the IMU thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_Task */
__weak void IMU_Task(void const * argument)
{
  /* USER CODE BEGIN IMU_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END IMU_Task */
}

/* saferStart function */
__weak void saferStart(void const * argument)
{
  /* USER CODE BEGIN saferStart */

  /* USER CODE END saferStart */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
