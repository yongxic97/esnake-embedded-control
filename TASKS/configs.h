#ifndef _CONFIGS_H
#define _CONFIGS_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define START_TASK_PRIO				1
#define START_STK_SIZE 				128  
TaskHandle_t StartTask_Handler;

#define SENSOR_TASK_PRIO			2
#define SENSOR_STK_SIZE 			256
TaskHandle_t SENSORTask_Handler;

#define CONTROL_TASK_PRIO			3
#define CONTROL_STK_SIZE 	  	256 
TaskHandle_t CONTROLTask_Handler;

#define JETSON_TASK_PRIO			3
#define JETSON_STK_SIZE 	  	256 
TaskHandle_t JETSONTask_Handler;

// queue related
#define PRES_LEN_MSG_Q_NUM    1  		// number of pressure queue message
#define END_PRES_MSG_Q_NUM		1
#define IMU_MANI_MSG_Q_NUM  	1   	// number of imu queue message
#define IMU_END_MSG_Q_NUM   	1   	// number of imu queue message

QueueHandle_t Pres_Len_Queue;   		// manipulator's pressure and length queue handler
QueueHandle_t End_Pres_Queue;   		// end-effector's pressure and queue handler
QueueHandle_t IMU_Mani_Queue;				// manipulator's imu's queue handler
QueueHandle_t IMU_End_Queue;				// end-effector's imu's queue handler

void start_task(void *pvParameters);

#endif
