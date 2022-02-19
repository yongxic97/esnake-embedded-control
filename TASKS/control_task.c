#include "control_task.h"
#include "key.h"


/****************************************************
*------------------IMPORTANT NOTE--------------------
*-eSnake_v1.0.0--------------------------------------
* In the first edition of code, FOUR preprocessing
* directives are designed, which are PRELIMINARY_ROUTINE,
* BOTTOM_NAIVE_CTRL, BOTTOM_MORPHS_CTRL, and CONFIG_FEEDBACK_CTRL.
* But temporarily only the former three are scheduled to
* be implemented in code, the fourth one will be left 
* for later. Also, in later versions of the project code,
* CV will be added, and a ralatively new version of controlling
* (but similar to the fourth paradigm above), CV_FUSION_CTRL
* will be done.
* ---------------------------------------------------
*****************************************************/

/******************Control Strategy******************/
// 1. Preliminary rountine control
#define PRELIMINARY_ROUTINE 0
#define CONFIG_FEEDBACK_CTRL 0
#define PRESSURE_FEEDBACK_TEST 0
#define QUEUE_COMMUNICATE_TEST 0
/****************************************************/

void stayStill(){
	GPIO_ResetBits(MANIPULATOR_CONTROL_GROUP_IN,Inner_group_way_1_IN|Inner_group_way_2_IN|Inner_group_way_3_IN);
	GPIO_ResetBits(MANIPULATOR_CONTROL_GROUP_OUT,Inner_group_way_1_OUT|Inner_group_way_2_OUT|Inner_group_way_3_OUT);
  GPIO_ResetBits(END_EFFECTOR_CONTROL_GROUP_IN,Outer_Linear_IN|Inner_Linear_IN|Grip_IN);
	GPIO_ResetBits(END_EFFECTOR_CONTROL_GROUP_OUT,Outer_Linear_OUT|Inner_Lienar_OUT|Grip_OUT);
	// printf("statStill\r\n");
}

/* PPP and NNN */
void actionPPP(long time){
	GPIO_ResetBits(MANIPULATOR_CONTROL_GROUP_OUT,Inner_group_way_1_OUT|Inner_group_way_2_OUT|Inner_group_way_3_OUT);	
	GPIO_SetBits(MANIPULATOR_CONTROL_GROUP_IN,Inner_group_way_1_IN|Inner_group_way_2_IN|Inner_group_way_3_IN);	
  // printf("actionPPP\r\n");
	vTaskDelay(time);
	stayStill();
}

void actionNNN(long time){
	GPIO_ResetBits(MANIPULATOR_CONTROL_GROUP_IN,Inner_group_way_1_IN|Inner_group_way_2_IN|Inner_group_way_3_IN);	
	GPIO_SetBits(MANIPULATOR_CONTROL_GROUP_OUT,Inner_group_way_1_OUT|Inner_group_way_2_OUT|Inner_group_way_3_OUT);	
	//	printf("actionPPP\r\n");
	vTaskDelay(time);	
	stayStill();
}

/* PPN and NNP */
void actionPPN(long time){
	GPIO_ResetBits(MANIPULATOR_CONTROL_GROUP_IN,Inner_group_way_3_IN);	
	GPIO_ResetBits(MANIPULATOR_CONTROL_GROUP_OUT,Inner_group_way_1_OUT|Inner_group_way_2_OUT);	
	GPIO_SetBits(MANIPULATOR_CONTROL_GROUP_IN,Inner_group_way_1_IN|Inner_group_way_2_IN);
	GPIO_SetBits(MANIPULATOR_CONTROL_GROUP_OUT,Inner_group_way_3_OUT);	
	vTaskDelay(time);	
	stayStill();
}

void actionNNP(long time){
	GPIO_ResetBits(MANIPULATOR_CONTROL_GROUP_IN,Inner_group_way_1_IN|Inner_group_way_2_IN);
	GPIO_ResetBits(MANIPULATOR_CONTROL_GROUP_OUT,Inner_group_way_3_OUT);	
	GPIO_SetBits(MANIPULATOR_CONTROL_GROUP_IN,Inner_group_way_3_IN);	
	GPIO_SetBits(MANIPULATOR_CONTROL_GROUP_OUT,Inner_group_way_1_OUT|Inner_group_way_2_OUT);	
	vTaskDelay(time);	
	stayStill();
}

/* PNP and NPN */
void actionPNP(long time){
	GPIO_ResetBits(MANIPULATOR_CONTROL_GROUP_IN,Inner_group_way_2_IN);	
	GPIO_ResetBits(MANIPULATOR_CONTROL_GROUP_OUT,Inner_group_way_1_OUT|Inner_group_way_3_OUT);
	GPIO_SetBits(MANIPULATOR_CONTROL_GROUP_IN,Inner_group_way_1_IN|Inner_group_way_3_IN);	
	GPIO_SetBits(MANIPULATOR_CONTROL_GROUP_OUT,Inner_group_way_2_OUT);	
	vTaskDelay(time);	
	stayStill();
}
void actionNPN(long time){
	GPIO_SetBits(MANIPULATOR_CONTROL_GROUP_IN,Inner_group_way_2_IN);	
	GPIO_SetBits(MANIPULATOR_CONTROL_GROUP_OUT,Inner_group_way_1_OUT|Inner_group_way_3_OUT);
	GPIO_ResetBits(MANIPULATOR_CONTROL_GROUP_IN,Inner_group_way_1_IN|Inner_group_way_3_IN);	
	GPIO_ResetBits(MANIPULATOR_CONTROL_GROUP_OUT,Inner_group_way_2_OUT);	
	vTaskDelay(time);	
	stayStill();
}

/* NPP and PNN */
void actionNPP(long time){
	GPIO_ResetBits(MANIPULATOR_CONTROL_GROUP_IN,Inner_group_way_1_IN);	
	GPIO_ResetBits(MANIPULATOR_CONTROL_GROUP_OUT,Inner_group_way_2_OUT|Inner_group_way_3_OUT);
	GPIO_SetBits(MANIPULATOR_CONTROL_GROUP_IN,Inner_group_way_2_IN|Inner_group_way_3_IN);	
	GPIO_SetBits(MANIPULATOR_CONTROL_GROUP_OUT,Inner_group_way_1_OUT);	
	vTaskDelay(time);	
	stayStill();
}

void actionPNN(long time){
	GPIO_ResetBits(MANIPULATOR_CONTROL_GROUP_IN,Inner_group_way_2_IN|Inner_group_way_3_IN);	
	GPIO_ResetBits(MANIPULATOR_CONTROL_GROUP_OUT,Inner_group_way_1_OUT);	
	GPIO_SetBits(MANIPULATOR_CONTROL_GROUP_IN,Inner_group_way_1_IN);	
	GPIO_SetBits(MANIPULATOR_CONTROL_GROUP_OUT,Inner_group_way_2_OUT|Inner_group_way_3_OUT);
	vTaskDelay(time);	
	stayStill();
}

/* end-effector manipulation */
void actionOuterLinearP(long time){
	GPIO_ResetBits(END_EFFECTOR_CONTROL_GROUP_OUT,Outer_Linear_OUT);	
	GPIO_SetBits(END_EFFECTOR_CONTROL_GROUP_IN,		Outer_Linear_IN);	
	vTaskDelay(time);	
  stayStill();
}






void pressureFeedbackControl(float pressure_inner_1,float pressure_inner_2, float pressure_inner_3){

	ManipulatorData tmpPressureReceive;
	u8 flag1=0,flag2=0,flag3=0,count=0;
	float tmp1=0,tmp2=0,tmp3=0;
	float err = 1;  
	stayStill();
	vTaskDelay(10);

	while(count<5){
		xQueuePeek(Pres_Len_Queue,&tmpPressureReceive,portMAX_DELAY);
		// adjust inner way 1
		if(!flag1){
			if(tmpPressureReceive.InnerGroup1<=pressure_inner_1+err && tmpPressureReceive.InnerGroup1>=pressure_inner_1-err){
				GPIO_ResetBits(MANIPULATOR_CONTROL_GROUP_IN,Inner_group_way_1_IN);
				GPIO_ResetBits(MANIPULATOR_CONTROL_GROUP_OUT,Inner_group_way_1_OUT);	
				flag1 = 1;
			}else if(tmpPressureReceive.InnerGroup1<pressure_inner_1-err){
				GPIO_SetBits(MANIPULATOR_CONTROL_GROUP_IN,Inner_group_way_1_IN);
				GPIO_ResetBits(MANIPULATOR_CONTROL_GROUP_OUT,Inner_group_way_1_OUT);	
			}else{
				GPIO_SetBits(MANIPULATOR_CONTROL_GROUP_OUT,Inner_group_way_1_OUT);
				GPIO_ResetBits(MANIPULATOR_CONTROL_GROUP_IN,Inner_group_way_1_IN);
			}
		}
		// adjust inner way 2
		if(!flag2){
			if(tmpPressureReceive.InnerGroup2<=pressure_inner_2+err && tmpPressureReceive.InnerGroup2>=pressure_inner_2-err){
				GPIO_ResetBits(MANIPULATOR_CONTROL_GROUP_IN,Inner_group_way_2_IN);
				GPIO_ResetBits(MANIPULATOR_CONTROL_GROUP_OUT,Inner_group_way_2_OUT);	
				flag2 = 1;
			}else if(tmpPressureReceive.InnerGroup2<pressure_inner_2-err){
				GPIO_SetBits(MANIPULATOR_CONTROL_GROUP_IN,Inner_group_way_2_IN);
				GPIO_ResetBits(MANIPULATOR_CONTROL_GROUP_OUT,Inner_group_way_2_OUT);	
			}else{
				GPIO_SetBits(MANIPULATOR_CONTROL_GROUP_OUT,Inner_group_way_2_OUT);
				GPIO_ResetBits(MANIPULATOR_CONTROL_GROUP_IN,Inner_group_way_2_IN);
			}
		}
		// adjust inner way 3
		if(!flag3){
			if(tmpPressureReceive.InnerGroup3<=pressure_inner_3+err && tmpPressureReceive.InnerGroup3>=pressure_inner_3-err){
				GPIO_ResetBits(MANIPULATOR_CONTROL_GROUP_IN,Inner_group_way_3_IN);
				GPIO_ResetBits(MANIPULATOR_CONTROL_GROUP_OUT,Inner_group_way_3_OUT);	
				flag3 = 1;
			}else if(tmpPressureReceive.InnerGroup3<pressure_inner_3-err){
				GPIO_SetBits(MANIPULATOR_CONTROL_GROUP_IN,Inner_group_way_3_IN);
				GPIO_ResetBits(MANIPULATOR_CONTROL_GROUP_OUT,Inner_group_way_3_OUT);
			}else{
				GPIO_SetBits(MANIPULATOR_CONTROL_GROUP_OUT,Inner_group_way_3_OUT);
				GPIO_ResetBits(MANIPULATOR_CONTROL_GROUP_IN,Inner_group_way_3_IN);
			}
		}
		vTaskDelay(5);
		if(flag1&&flag2&&flag3){
			flag1=0,flag2=0,flag3=0;
			tmp1+=tmpPressureReceive.InnerGroup1;
			tmp2+=tmpPressureReceive.InnerGroup2;
			tmp3+=tmpPressureReceive.InnerGroup3;
			count++;
		}
		else{
			count = 0;
		}
	} // end of while count
}
	
void control_task(void *pvParameters){
	
	ManipulatorData manipulatorReceive;
	Pose 						maniPoseReceive;
	Pose 						endPoseReceive;
	
#if QUEUE_COMMUNICATE_TEST
	
	/* queue communicatin test */
	while(1){
		
		if(xQueuePeek(Pres_Len_Queue,&manipulatorReceive,portMAX_DELAY)){
				printf("control task received! %.2f %.2f %.2f %.2f\r\n", manipulatorReceive.InnerGroup1,manipulatorReceive.InnerGroup2,
			manipulatorReceive.InnerGroup3,manipulatorReceive.CenterLen);
		}
		
		if(xQueuePeek(IMU_Mani_Queue,&maniPoseReceive,portMAX_DELAY)){
			printf("control task received pose! %.2f %.2f %.2f\r\n", maniPoseReceive.roll, maniPoseReceive.pitch, maniPoseReceive.yaw);
		}
		vTaskDelay(100);	
	}
	/* queue communication test end */

#endif

	u8 key=0;           //保存键值
	
	while(!key){
		key = KEY_Scan(0);		//得到键值
		vTaskDelay(10);	
	}
	
	Pump_Start();
	
	printf("Pumps started. Control task begin in 3 seconds\r\n");
	
	vTaskDelay(3000);	
	
	while(1){

#if PRELIMINARY_ROUTINE		
		//printf("1 sec passed\r\n");
		actionNPP(200);

		vTaskDelay(2000);
		
		actionPNN(200);

		vTaskDelay(2000);
		
		actionPPN(250);
		vTaskDelay(1000);		
		stayStill();
		vTaskDelay(1000);
		
		actionPNP(250);
		vTaskDelay(1000);
		stayStill();
		vTaskDelay(1000);
		
		actionNPP(250);
		vTaskDelay(1000);
		stayStill();
		vTaskDelay(1000);
		
#endif
		
#if PRESSURE_FEEDBACK_TEST
	
	/* reset */
	pressureFeedbackControl(8,4,4);
	printf("reset successfully\r\n");	
	vTaskDelay(1000);
	
	/* bending */
	pressureFeedbackControl(-5,16,16);
	vTaskDelay(1000);

#endif
	}/* end of big while(1) */
	
}
