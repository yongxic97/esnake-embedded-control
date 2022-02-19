#include "sensor_task.h"

#include "dac.h"

#define ALL_SENSORS				0
#define MANIPULATOR_TEST  1
#define ADDA_TEST 				0


u16 dataBuffer[7]; /* six ways of pressure and one way of distance */

//空闲任务
static StackType_t IdleTaskStack[configMINIMAL_STACK_SIZE];
static StaticTask_t IdleTaskTCB;

//定时器任务
static StackType_t TimerTaskStack[configTIMER_TASK_STACK_DEPTH];
static StaticTask_t TimerTaskTCB;

//空闲任务所需内存
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, 
									StackType_t **ppxIdleTaskStackBuffer, 
									uint32_t *pulIdleTaskStackSize )
{
	*ppxIdleTaskTCBBuffer   = &IdleTaskTCB;
	*ppxIdleTaskStackBuffer = IdleTaskStack;
	*pulIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
}

//定时器任务所需内存
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, 
									 StackType_t **ppxTimerTaskStackBuffer, 
									 uint32_t *pulTimerTaskStackSize )
{
	*ppxTimerTaskTCBBuffer   = &TimerTaskTCB;
	*ppxTimerTaskStackBuffer = TimerTaskStack;
	*pulTimerTaskStackSize   = configMINIMAL_STACK_SIZE;
}

void getAllSensorDataDMA(ManipulatorData * manipulatorData, 
											EndeffectorData * endeffectorData,
											u16 * dataBuffer ){
	
	long adc_raw_data = 0;	  
												
	adc_raw_data = *dataBuffer;
	printf("%ld\r\n",adc_raw_data);
	manipulatorData->InnerGroup1     = (double) adc_raw_data / 4096.0 * 3.3;
	
	adc_raw_data = *(dataBuffer+1);
	manipulatorData->InnerGroup2 		 = (double) adc_raw_data / 4096.0 * 3.3;

	adc_raw_data = *(dataBuffer+2);
	manipulatorData->InnerGroup3 		 = (double) adc_raw_data / 4096.0 * 3.3;
	
	adc_raw_data = *(dataBuffer+3);
	endeffectorData->LinearOuterPres = (double) adc_raw_data / 4096.0 * 3.3;
	
	adc_raw_data = *(dataBuffer+4);
	endeffectorData->LinearInnerPres = (double) adc_raw_data / 4096.0 * 3.3;
	
	adc_raw_data = *(dataBuffer+5);
	endeffectorData->RotPres 				 = (double) adc_raw_data / 4096.0 * 3.3;
	
	adc_raw_data = *(dataBuffer+6);
	manipulatorData->CenterLen	     = (double)	adc_raw_data / 4096.0 * 800.0;
	// 125->130
	manipulatorData->InnerGroup1 		 = 200.0 / 2.4 * manipulatorData->InnerGroup1     - 125.0; /* voltage ranges from 0.3V to 2.7V */
	manipulatorData->InnerGroup2 		 = 200.0 / 2.4 * manipulatorData->InnerGroup2     - 125.0;
	manipulatorData->InnerGroup3 		 = 200.0 / 2.4 * manipulatorData->InnerGroup3     - 125.0;
	endeffectorData->LinearOuterPres = 200.0 / 2.4 * endeffectorData->LinearOuterPres - 125.0;
	endeffectorData->LinearInnerPres = 200.0 / 2.4 * endeffectorData->LinearInnerPres - 125.0;
	endeffectorData->RotPres 				 = 200.0 / 2.4 * endeffectorData->RotPres         - 125.0;

}

void sensor_task(void *pvParameters){

	ManipulatorData		manipulatorData;
	EndeffectorData 	endeffectorData;
	Pose 							pose_mani;
	Pose 							pose_end;
	
	/*********Initialize DMA*********/
	Adc_DMA_Init(dataBuffer, 7);
	getAllSensorDataDMA(&manipulatorData, &endeffectorData, dataBuffer);
	xQueueOverwrite(Pres_Len_Queue,&manipulatorData);
	vTaskDelay(1);
	getAllSensorDataDMA(&manipulatorData, &endeffectorData, dataBuffer);
	xQueueOverwrite(Pres_Len_Queue,&manipulatorData);
	vTaskDelay(1);
	/*****End of initialising DMA****/
	
	//printf("Sensor task init done...\r\n");
	
	
	
	while(1){
		
#if ALL_SENSORS
		
		getAllSensorDataDMA(&manipulatorData, &endeffectorData, dataBuffer);
		
		imu1_get_data(&pose_mani);  // imu1 is the one on the manipulator
		vTaskDelay(10);							
		imu2_get_data(&pose_end);            // imu2 is the one on the end-effector
		vTaskDelay(10);

		taskENTER_CRITICAL(); // enter critical zone     

		printf("Mani pres & len: %.2f %.2f %.2f %.2f\r\n"			 , manipulatorData.InnerGroup1, manipulatorData.InnerGroup2,
																												     manipulatorData.InnerGroup3, manipulatorData.CenterLen  );	
		printf("End  pres : %.2f %.2f %.2f\r\n"					 			 , endeffectorData.LinearOuterPres, endeffectorData.LinearInnerPres,
																												     endeffectorData.RotPres);	
		printf("Mani orientation: X: %.2f Y: %.2f Z: %.2f\r\n" , pose_mani.pitch, pose_mani.roll, pose_mani.yaw);	
		printf("End  orientation: X: %.2f Y: %.2f Z: %.2f\r\n" , pose_end.pitch,  pose_end.roll,  pose_end.yaw);
		
		xQueueOverwrite(Pres_Len_Queue, &manipulatorData);
		xQueueOverwrite(End_Pres_Queue, &endeffectorData);
		xQueueOverwrite(IMU_Mani_Queue, &pose_mani);
		xQueueOverwrite(IMU_End_Queue,  &pose_end);
			
		taskEXIT_CRITICAL();            // exit critical zone			
		
		// vTaskDelay(500);

#elif MANIPULATOR_TEST
// in this mode, pressure sensors 1-3, the length sensor, and the imu1 are expected to work.
// Then it prints a whole pack of sensor data to USART1.

	getAllSensorDataDMA(&manipulatorData, &endeffectorData, dataBuffer); // pressures and the length

	imu1_get_data(&pose_mani);  // imu1 is the one on the manipulator
	vTaskDelay(10);							
		
	taskENTER_CRITICAL(); // enter critical zone     

/***********************************************
 * The sequence of sending one data pack is:   *
 * 1. manipulator pressure 1,                  *
 * 2. manipulator pressure 2,                  *
 * 3. manipulator pressure 3,                  *
 * 4. manipulator length,                      *
 * 5. manipulator pose - roll,                 *
 * 6. manipulator pose - pitch,                *
 * 7. manipulator pose - yaw.                  *
 ***********************************************/
	printf("%.2f %.2f %.2f %.2f %.2f %.2f %.2f\r\n", 
				manipulatorData.InnerGroup1, manipulatorData.InnerGroup2, manipulatorData.InnerGroup3, 
				manipulatorData.CenterLen, pose_mani.roll, pose_mani.pitch, pose_mani.yaw);		
			
	taskEXIT_CRITICAL();            // exit critical zone			
  
	vTaskDelay(500);
	

#elif ADDA_TEST

getAllSensorDataDMA(&manipulatorData, &endeffectorData, dataBuffer);
printf("%.2f \r\n",manipulatorData.InnerGroup1);		
		vTaskDelay(500);
#endif
	}
}
