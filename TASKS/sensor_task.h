#ifndef _SENSOR_TASK_H
#define _SENSOR_TASK_H

#include "usart.h"
#include "adc.h"
#include "uni_variables.h"

/* FreeRTOS general includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
/* end of FreeRTOS general includes */

/*IMU includes*/
#include "imu1.h"
#include "imu2.h"

extern QueueHandle_t Pres_Len_Queue;   		// manipulator's pressure and length queue handler
extern QueueHandle_t End_Pres_Queue;			// end-effector's pressure and queue handler
extern QueueHandle_t IMU_Mani_Queue;			// manipualtor's imu's queue handler
extern QueueHandle_t IMU_End_Queue;				// end-effector's imu's queue handler

void getAllSensorDataDMA(ManipulatorData * manipulatorPressure, 
												 EndeffectorData * endeffectorData,
											   u16 * dataBuffer);
void getManiSensorDataDMA(ManipulatorData * manipulatorPressure, 
											   u16 * dataBuffer);
void getEndSensorDataDMA(EndeffectorData * endeffectorData,
											   u16 * dataBuffer);
void sensor_task(void *pvParameters);

#endif
