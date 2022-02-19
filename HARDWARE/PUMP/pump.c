#include "pump.h"

void Pump_Start(){
	GPIO_SetBits(GPIOF,PUMP_IN | PUMP_OUT);
}

void Pump_Stop(){
	GPIO_ResetBits(PUMP_PORT,PUMP_IN | PUMP_OUT);			
}

void Pump_Init(void){
  	
  GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);	//使能GPIOF时钟
//	GPIO_InitStructure.GPIO_Pin = PUMP_IN;	
	GPIO_InitStructure.GPIO_Pin = PUMP_IN | PUMP_OUT;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;			//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			//上拉
	GPIO_Init(PUMP_PORT, &GPIO_InitStructure);					//初始化GPIO
	GPIO_ResetBits(PUMP_PORT,PUMP_IN | PUMP_OUT);	
	//GPIO_ResetBits(PUMP_PORT,PUMP_IN);				

}
