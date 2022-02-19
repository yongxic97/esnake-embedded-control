#include "key.h" 
#include "FreeRTOS.h"
#include "task.h"

//��ʼ��PE3Ϊ�����.��ʹ���������ڵ�ʱ��		    
//KEY IO��ʼ��
void KEY_Init(void)
{    	 
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	//ʹ��GPIOEʱ��

	//GPIOE3��ʼ������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	//Key 1��ӦIO��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			//����
	GPIO_Init(GPIOE, &GPIO_InitStructure);					//��ʼ��GPIO
	GPIO_SetBits(GPIOE,GPIO_Pin_3);			//PE3

}

u8 KEY_Scan(u8 mode)
{	 
	static u8 key_up=1;//�������ɿ���־
	if(mode)key_up=1;  //֧������		  
	if(key_up&&(KEY0==0||KEY1==0)){
		vTaskDelay(10);
		key_up=0;
		if(KEY0==0)return 1;
		else if(KEY1==0)return 2;
	}else if(KEY0==1&&KEY1==1)key_up=1; 	    
 	return 0;// �ް�������
}