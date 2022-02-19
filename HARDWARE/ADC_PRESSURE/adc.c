#include "adc.h"
#include "delay.h"		 

// ALL DEVICES REQUIRING ADC ARE CONFIGURED IN THE ADC PART!!!
// e.g. Pressure sensors, distance sensor.

/************************************************
* --------------Note 2021-08-28---------------- *
* Preliminary tests show that with different    *
* channels of ADC initialized and data acquired *
* independently, the max sampling frequency is  *
* about 28Hz on our STM32F407ZGT6 chip.         *
* Hence, the goal is to significantly increase  *
* sampling frequency compared with the existing *
* speed above.                                  *
*                                               *
* --------------Note 2021-10-02---------------- *
* Now that DMA is used well, in later versions  *
* I delete all independent mode related codes.  *
* In this file all functions are intrinsically  *
* using ADC in DMA mode.                        *
*************************************************/

/* initialise ADC1, multi-channel, DMA mode */
void Adc_DMA_Init(u16 * SendBuff, int Send_buff_size){
	
	GPIO_InitTypeDef  			GPIO_InitStructure;
	ADC_CommonInitTypeDef 	ADC_CommonInitStructure;
	ADC_InitTypeDef       	ADC_InitStructure;
	DMA_InitTypeDef					DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_DMA2, ENABLE); 																	// enable clock of group GPIOA
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 																												// enable clock of ADC1

  /* Initialise GPIO PA1-PA7 */
//  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7; 
  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN; 																																// analog input 1-6
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;																														// no pull-up or pull-down
  GPIO_Init(GPIOA, &GPIO_InitStructure);																																			// initialise GPIO group
	
  /* Config DMA Stream */
  DMA_InitStructure.DMA_Channel            = DMA_Channel_0;  																									// choose channel
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &ADC1->DR;																						// DMA base address of peripheral device (here, ADC1)
  DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t) SendBuff;																							// DMA Memory 0 base address
  DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;																			// peripheral to memory
  DMA_InitStructure.DMA_BufferSize         = Send_buff_size;																									// 数据传输量 
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;																				// 外设非增量模式
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;																						// 存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;																	// peripheral data size halfword
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;																			// memory data size halfword
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;																								// circular mode
  DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;																							// medium priority
  DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;         																		// disable FIFO
  DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;																					// full FIFO threshold 
  DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;																					// 存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;																			// 外设突发单次传输
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);																																	// 初始化DMA Stream
	DMA_Cmd(DMA2_Stream0,ENABLE);
	
	/* ADC common init */
	ADC_CommonInitStructure.ADC_Mode             = ADC_Mode_Independent;																				// 独立模式
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;																// 两个采样阶段之间的延迟5个时钟
  ADC_CommonInitStructure.ADC_DMAAccessMode    = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_Prescaler        = ADC_Prescaler_Div4;																					// 预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz 
  ADC_CommonInit(&ADC_CommonInitStructure);																																		// 初始化
	
	/* ADC init */
  ADC_InitStructure.ADC_Resolution           = ADC_Resolution_12b;																						// 12位模式
  ADC_InitStructure.ADC_ScanConvMode         = ENABLE;																												// open scan mode
  ADC_InitStructure.ADC_ContinuousConvMode   = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;																	// 禁止触发检测，使用软件触发
  ADC_InitStructure.ADC_DataAlign            = ADC_DataAlign_Right;																						// 右对齐	
  ADC_InitStructure.ADC_NbrOfConversion      = 7;																															// 7个转换在规则序列 
  ADC_Init(ADC1, &ADC_InitStructure);																																					// ADC初始化
	
	ADC_RegularChannelConfig(ADC1,ADC_Channel_1,1,ADC_SampleTime_3Cycles); 																			// PA1 - manipulator way 1 pressure
	ADC_RegularChannelConfig(ADC1,ADC_Channel_2,2,ADC_SampleTime_3Cycles);																			// PA2 - manipulator way 2 pressure
	ADC_RegularChannelConfig(ADC1,ADC_Channel_3,3,ADC_SampleTime_3Cycles);																			// PA3 - manipulator way 3 pressure
	ADC_RegularChannelConfig(ADC1,ADC_Channel_4,4,ADC_SampleTime_3Cycles);																			// PA4 - end-effector lower linear pressure
	ADC_RegularChannelConfig(ADC1,ADC_Channel_5,5,ADC_SampleTime_3Cycles);																			// PA5 - end-effector upper linear pressure
	ADC_RegularChannelConfig(ADC1,ADC_Channel_6,6,ADC_SampleTime_3Cycles);																			// PA6 - end-effector rotational pressure
	ADC_RegularChannelConfig(ADC1,ADC_Channel_7,7,ADC_SampleTime_3Cycles);																			// PA7 - length sensor
	
	ADC_DMARequestAfterLastTransferCmd(ADC1,ENABLE);
	
	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1,ENABLE);
	
	/* Enable ADC1 */
	ADC_Cmd(ADC1,ENABLE);
	ADC_SoftwareStartConv(ADC1);

}

