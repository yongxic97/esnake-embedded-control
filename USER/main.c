#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "usart2.h"
#include "usart3.h"
#include "led.h"
#include "adc.h"
#include "valves.h"
#include "control_task.h"
#include "sensor_task.h"
#include "jetson_task.h"
#include "configs.h"
#include "key.h"
#include "imu1.h"
#include "imu2.h"
#include "imu_share.h"

#include "dac.h"

int main(void)
{ 
	delay_init(168);		
	uart_init(115200); 
	usart2_init(115200); 
	usart3_init(115200); 
	LED_Init();
	KEY_Init();
	Pump_Init();
	Manipulator_Valves_Init();
	End_Effector_Valves_Init();
	imu1_init();							/* IMU901 IMU module initialize */
	imu2_init();
//	Dac1_Init();
//	Dac1_Set_Vol(100);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	printf("initialisation done...\r\n");

	xTaskCreate((TaskFunction_t )start_task,           
							(const char*    )"start_task",      
							(uint16_t       )START_STK_SIZE,      
							(void*          )NULL,                 
							(UBaseType_t    )START_TASK_PRIO,     
							(TaskHandle_t*  )&StartTask_Handler); 
	
	vTaskStartScheduler();  // begin scheduling of tasks        
}
 
// start task function
void start_task(void *pvParameters)
{
	taskENTER_CRITICAL(); // enter critical zone          
	// Create message queue
	/* variables defined in config.h */
	Pres_Len_Queue = xQueueCreate(PRES_LEN_MSG_Q_NUM,sizeof(ManipulatorData));      
	End_Pres_Queue = xQueueCreate(END_PRES_MSG_Q_NUM,sizeof(EndeffectorData));
	IMU_Mani_Queue = xQueueCreate(IMU_MANI_MSG_Q_NUM,sizeof(Pose)); 
	IMU_End_Queue  = xQueueCreate(IMU_END_MSG_Q_NUM, sizeof(Pose)); 
	
	// create task of sensing  
	xTaskCreate((TaskFunction_t )sensor_task,     	
							(const char*    )"sensor_task",   	
							(uint16_t       )SENSOR_STK_SIZE, 
							(void*          )NULL,				
							(UBaseType_t    )SENSOR_TASK_PRIO,	
							(TaskHandle_t*  )&SENSORTask_Handler);   
							
	// create task of controlling
	xTaskCreate((TaskFunction_t )control_task,     
							(const char*    )"control_task",   
							(uint16_t       )CONTROL_STK_SIZE, 
							(void*          )NULL,
							(UBaseType_t    )CONTROL_TASK_PRIO,
							(TaskHandle_t*  )&CONTROLTask_Handler);     
							
	// create task of communication with Jetson Nano
	xTaskCreate((TaskFunction_t )jetson_task,     
							(const char*    )"jetson_task",   
							(uint16_t       )JETSON_STK_SIZE, 
							(void*          )NULL,
							(UBaseType_t    )JETSON_TASK_PRIO,
							(TaskHandle_t*  )&JETSONTask_Handler);  
							
	vTaskDelete(StartTask_Handler); // delete starting task
							
	taskEXIT_CRITICAL();            // exit critical zone
}

