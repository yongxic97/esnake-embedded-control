#ifndef _USART3_H_
#define _USART3_H_

#include "sys.h"

void usart3_init(uint32_t bound);

void usart3_sendData(uint8_t *data);
uint16_t usart3_getRxData(uint8_t *buf, uint16_t len);


#endif /* _USART3_H_ */

