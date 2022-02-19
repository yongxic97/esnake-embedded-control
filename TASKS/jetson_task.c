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
				len=USART_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���
				printf("\r\n\r\n");//���뻻��
				
				for(t=0;t<len;t++)
				{
					USART_SendData(USART1, USART_RX_BUF[t]);         //�򴮿�1��������
					while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
				}
				printf("\r\n\r\n");//���뻻��
				USART_RX_STA=0;
			}
		
		vTaskDelay(1);
	}
}
