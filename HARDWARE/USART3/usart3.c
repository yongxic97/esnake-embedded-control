#include "sys.h"
#include "usart3.h"
#include "ringbuffer.h"

#define UART3_RX_BUFFER_SIZE	256
uint8_t uart3RxBuffer[UART3_RX_BUFFER_SIZE];

ringbuffer_t uart3RxFifo;

/**
 * @brief       ����X��ʼ��
 * @param       ��
 * @retval      ��
 */
void usart3_init(uint32_t bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	//ʹ��USART3��GPIOBʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 , ENABLE);
	
	//����3��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOB10����ΪUSART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB11����ΪUSART3
	
	//USART3_TX   GPIOB10 
	//USART3_RX	  GPIOB11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOB10��GPIOB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOD
      

  //Usart3 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
   //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

  USART_Init(USART3, &USART_InitStructure); //��ʼ������3
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART3, ENABLE);                    //ʹ�ܴ���3

   ringbuffer_init(&uart3RxFifo, uart3RxBuffer, UART3_RX_BUFFER_SIZE);
}


/**
  * @brief  ����3����
  * @param  data: ���͵�����
  * @param  len: ���ݳ���
  * @retval uint8_t: 0�ɹ� ������ʧ��
  */
void usart3_sendData(uint8_t *data){

        USART_SendData(USART3,*data);
        while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
}

/**
  * @brief  ��ȡ����3����fifo������
  * @param  buf: ��ŵĻ�����
  * @param  len: ��Ҫ��ȡ�ĳ���
  * @retval uint16_t: ʵ�ʻ�ȡ���ĳ��� 0��ʾû�����ݿɻ�ȡ
  */
uint16_t usart3_getRxData(uint8_t *buf, uint16_t len)
{
    return ringbuffer_out(&uart3RxFifo, buf, len);
}


/**
 * @brief       ����X�жϷ�����
 * @param       ��
 * @retval      ��
 */

void USART3_IRQHandler(void)
{
	if(USART_GetITStatus(USART3,USART_IT_RXNE))/*!< ���շǿ��ж� */
	{
				uint8_t res = USART_ReceiveData(USART3);
        ringbuffer_in_check(&uart3RxFifo, (uint8_t *)&res, 1); /*!< �����յ������ݷ���FIFO */	
	}
	
}


/*******************************END OF FILE************************************/



