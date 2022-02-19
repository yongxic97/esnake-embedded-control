#ifndef _CONTROL_TASK_H
#define _CONTROL_TASK_H

/******************************************
* FreeRTOS controlling task.
* 
*
*
*******************************************/

#include "uni_variables.h"
#include "valves.h"
#include "pump.h"
/* FreeRTOS general includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
/* end of FreeRTOS general includes */

extern QueueHandle_t Pres_Len_Queue;   		// manipulator's pressure and length queue handler
extern QueueHandle_t End_Pres_Queue;   		// end-effector's pressure and queue handler
extern QueueHandle_t IMU_Mani_Queue;				// manipulator's imu's queue handler
extern QueueHandle_t IMU_End_Queue;				// end-effector's imu's queue handler

/* Three principle directions,
 0, 120, 240 degrees seperately. */
typedef enum{
	DIRECTION_ZERO_DEG = 0,
	DIRECTION_120_DEG,
	DIRECTION_240_DEG
}PRINCIPLE_BENDING_Direction;

void stayStill(void);
/* PPP and NNN */
void actionPPP(long time);
void actionNNN(long time);
/* PPN and NNP */
void actionPPN(long time);
void actionNNP(long time);
/* PNP and NPN */
void actionPNP(long time);
/* NPP and PNN */
void actionNPP(long time);
void actionPNN(long time);

/* End-effector manipulation */
void actionOuterLinearP(long time);

void pressureFeedbackControl(float pressure_inner_1,float pressure_inner_2, float pressure_inner_3);
void control_task(void *pvParameters);

#endif
