#include "key.h" 
#include "FreeRTOS.h"
#include "task.h"

//初始化PE3为输出口.并使能这两个口的时钟		    
//KEY IO初始化
void KEY_Init(void)
{    	 
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	//使能GPIOE时钟

	//GPIOE3初始化设置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	//Key 1对应IO口
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			//上拉
	GPIO_Init(GPIOE, &GPIO_InitStructure);					//初始化GPIO
	GPIO_SetBits(GPIOE,GPIO_Pin_3);			//PE3

}

u8 KEY_Scan(u8 mode)
{	 
	static u8 key_up=1;//按键按松开标志
	if(mode)key_up=1;  //支持连按		  
	if(key_up&&(KEY0==0||KEY1==0)){
		vTaskDelay(10);
		key_up=0;
		if(KEY0==0)return 1;
		else if(KEY1==0)return 2;
	}else if(KEY0==1&&KEY1==1)key_up=1; 	    
 	return 0;// 无按键按下
}