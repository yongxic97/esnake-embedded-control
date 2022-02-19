#ifndef _JETSON_TASK
#define _JETSON_TASK

/* FreeRTOS general includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
/* end of FreeRTOS general includes */
#include "imu1.h"
#include "imu2.h"

void jetson_task(void *pvParameters);


#endif
