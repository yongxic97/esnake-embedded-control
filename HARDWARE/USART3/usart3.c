#include "sys.h"
#include "usart3.h"
#include "ringbuffer.h"

#define UART3_RX_BUFFER_SIZE	256
uint8_t uart3RxBuffer[UART3_RX_BUFFER_SIZE];

ringbuffer_t uart3RxFifo;

/**
 * @brief       串口X初始化
 * @param       无
 * @retval      无
 */
void usart3_init(uint32_t bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	//使能USART3，GPIOB时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 , ENABLE);
	
	//串口3对应引脚复用映射
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOB10复用为USART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB11复用为USART3
	
	//USART3_TX   GPIOB10 
	//USART3_RX	  GPIOB11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOB10与GPIOB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOD
      

  //Usart3 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART3, &USART_InitStructure); //初始化串口3
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART3, ENABLE);                    //使能串口3

   ringbuffer_init(&uart3RxFifo, uart3RxBuffer, UART3_RX_BUFFER_SIZE);
}


/**
  * @brief  串口3发送
  * @param  data: 发送的数据
  * @param  len: 数据长度
  * @retval uint8_t: 0成功 其他：失败
  */
void usart3_sendData(uint8_t *data){

        USART_SendData(USART3,*data);
        while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
}

/**
  * @brief  获取串口3接收fifo的数据
  * @param  buf: 存放的缓冲区
  * @param  len: 需要获取的长度
  * @retval uint16_t: 实际获取到的长度 0表示没有数据可获取
  */
uint16_t usart3_getRxData(uint8_t *buf, uint16_t len)
{
    return ringbuffer_out(&uart3RxFifo, buf, len);
}


/**
 * @brief       串口X中断服务函数
 * @param       无
 * @retval      无
 */

void USART3_IRQHandler(void)
{
	if(USART_GetITStatus(USART3,USART_IT_RXNE))/*!< 接收非空中断 */
	{
				uint8_t res = USART_ReceiveData(USART3);
        ringbuffer_in_check(&uart3RxFifo, (uint8_t *)&res, 1); /*!< 将接收到的数据放入FIFO */	
	}
	
}


/*******************************END OF FILE************************************/



