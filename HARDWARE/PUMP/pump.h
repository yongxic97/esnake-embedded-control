#ifndef _PUMP_H
#define _PUMP_H

#include "sys.h"

/* Drive for pumps
 * Connected to pins */
#define PUMP_PORT GPIOF
#define PUMP_IN  GPIO_Pin_7
#define PUMP_OUT GPIO_Pin_8

void Pump_Start(void);
void Pump_Stop(void);

void Pump_Init(void);

#endif
