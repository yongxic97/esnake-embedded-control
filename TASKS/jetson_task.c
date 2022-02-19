#include "jetson_task.h"
#include "sys.h"

void jetson_task(void *pvParameters){
	u16 recvFl = 0;
	u8 t;
	u8 len;	
	u16 times=0;  
	printf("jetson communicate test \r\n");
	while(1){
			if(USART_RX_STA&0x8000)
			{					   
				len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
				printf("\r\n\r\n");//插入换行
				
				for(t=0;t<len;t++)
				{
					USART_SendData(USART1, USART_RX_BUF[t]);         //向串口1发送数据
					while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
				}
				printf("\r\n\r\n");//插入换行
				USART_RX_STA=0;
			}
		
		vTaskDelay(1);
	}
}
