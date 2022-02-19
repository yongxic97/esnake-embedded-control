#include "valves.h"

// Initialize PF1-PF8 and PF11-PF14 as output ports for valve shut/open
// Enable clocks. Set all valves' initial state as shut.
void Manipulator_Valves_Init(void){
	
 GPIO_InitTypeDef  GPIO_InitStructure;

 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF|RCC_AHB1Periph_GPIOE, ENABLE);	//使能GPIOF时钟

 GPIO_InitStructure.GPIO_Pin = Inner_group_way_1_IN|Inner_group_way_2_IN|Inner_group_way_3_IN;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;			//普通输出模式
 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		//100MHz
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
 GPIO_Init(MANIPULATOR_CONTROL_GROUP_IN ,&GPIO_InitStructure);
	
 GPIO_InitStructure.GPIO_Pin = Inner_group_way_1_OUT|Inner_group_way_2_OUT|Inner_group_way_3_OUT;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;			//普通输出模式
 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		//100MHz
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
 GPIO_Init(MANIPULATOR_CONTROL_GROUP_OUT ,&GPIO_InitStructure);
	
 //GPIO_ResetBits(MANIPULATOR_CONTROL_GROUP,Inner_group_way_1_IN|Inner_group_way_2_IN|Inner_group_way_3_IN|
 //							Inner_group_way_1_OUT|Inner_group_way_2_OUT|Inner_group_way_3_OUT);	
 // GPIO_ResetBits is for outputing low voltage,
 //	i.e. to shut the corresponding valve.
}

// Enable clocks. Set all valves' initial state as shut.
void End_Effector_Valves_Init(void){
	
 GPIO_InitTypeDef  GPIO_InitStructure;

 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF|RCC_AHB1Periph_GPIOE, ENABLE);	//使能GPIOF时钟

 GPIO_InitStructure.GPIO_Pin = Outer_Linear_IN | Inner_Linear_IN | Grip_IN;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;			//普通输出模式
 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		//100MHz
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
 GPIO_Init(END_EFFECTOR_CONTROL_GROUP_IN ,&GPIO_InitStructure);
	
 GPIO_InitStructure.GPIO_Pin = Outer_Linear_OUT | Inner_Lienar_OUT | Grip_OUT;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;			//普通输出模式
 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		//100MHz
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
 GPIO_Init(END_EFFECTOR_CONTROL_GROUP_OUT ,&GPIO_InitStructure);
	
 // GPIO_ResetBits(MANIPULATOR_CONTROL_GROUP,Inner_group_way_1_IN|Inner_group_way_2_IN|Inner_group_way_3_IN|
 //							Inner_group_way_1_OUT|Inner_group_way_2_OUT|Inner_group_way_3_OUT);	
 // GPIO_ResetBits is for outputing low voltage,
 //	i.e. to shut the corresponding valve.
}