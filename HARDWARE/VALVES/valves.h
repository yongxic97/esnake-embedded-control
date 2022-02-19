#ifndef _VALVES_H
#define _VALVES_H

#include "sys.h"

/* For controlling the 6 ways of the manipulator, 
 use GPIO output of PF1-PF6 and PE8-PE13. */
#define MANIPULATOR_CONTROL_GROUP_IN 	 GPIOF
#define MANIPULATOR_CONTROL_GROUP_OUT  GPIOE
#define END_EFFECTOR_CONTROL_GROUP_IN  GPIOF
#define END_EFFECTOR_CONTROL_GROUP_OUT GPIOE


#define Inner_group_way_1_IN  GPIO_Pin_6
#define Inner_group_way_2_IN  GPIO_Pin_5
#define Inner_group_way_3_IN  GPIO_Pin_2

#define Inner_group_way_1_OUT GPIO_Pin_8
#define Inner_group_way_2_OUT GPIO_Pin_13
#define Inner_group_way_3_OUT GPIO_Pin_10 
 
#define Outer_Linear_IN  			GPIO_Pin_1
#define Inner_Linear_IN  			GPIO_Pin_3
#define Grip_IN  							GPIO_Pin_4
// two DoFs missing

#define Outer_Linear_OUT 			GPIO_Pin_9
#define Inner_Lienar_OUT 			GPIO_Pin_11
#define Grip_OUT 							GPIO_Pin_12
// two DoFs missing

void Manipulator_Valves_Init(void);
void End_Effector_Valves_Init(void);

#endif
