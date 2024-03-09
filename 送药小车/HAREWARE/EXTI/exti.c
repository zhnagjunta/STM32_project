#include "exti.h"
#include "key.h"

/**************************************************************************
�������ܣ��ⲿ�жϳ�ʼ��
��ڲ�������
����  ֵ���� 
**************************************************************************/
void EXTIX_Init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStruct;
	KEY_Init();	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);  //�ⲿ�жϣ���Ҫʹ��AFIOʱ��
	
	//GPIOA.5 �ж����Լ��жϳ�ʼ������
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource5);
    EXTI_InitStructure.EXTI_Line=EXTI_Line14;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//�½��ش���
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
	
	//����IO�����ж��ߵ�ӳ���ϵ
	NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;         //ʹ�ܰ������ڵ��ⲿ�ж�ͨ��
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;            //ʹ���ⲿ�ж�ͨ��
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;  //��ռ���ȼ�2 
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;         //��Ӧ���ȼ�1
	NVIC_Init(&NVIC_InitStruct);

}

//�ⲿ�ж�9~5�������
void EXTI15_10_IRQHandler(void)
{			
	delay_ms(4);     //����		
	
   if(KEY==0)	   //��/�ص��
	{
		EXTI_ClearITPendingBit(EXTI_Line14);  //���LINE14�ϵ��жϱ�־λ 
		Flag_Next=1;
	}		
}





