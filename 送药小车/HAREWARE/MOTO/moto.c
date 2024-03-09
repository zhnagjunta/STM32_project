#include "moto.h"

void Motor_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //ʹ�ܶ˿�ʱ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;	//�˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
  GPIO_Init(GPIOB, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOB
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //ʹ�ܶ˿�ʱ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_15;	//�˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
  GPIO_Init(GPIOA, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOA
}

void Motor_PWM_Init(u16 arr,u16 psc)        
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
  GPIO_InitTypeDef GPIO_InitStruct;                             
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);   //ʹ�ܶ�ʱ��1ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);  //ʹ��GPIOA��ʱ��
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;          //�����������
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_11;   //PA8 11
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	TIM_TimeBaseInitStruct.TIM_Period = arr;              //�趨�������Զ���װֵ 
	TIM_TimeBaseInitStruct.TIM_Prescaler  = psc;          //�趨Ԥ��Ƶ��
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;//TIM���ϼ���ģʽ
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;         //����ʱ�ӷָ�
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStruct);       //��ʼ����ʱ��
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;             //ѡ��PWM1ģʽ
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStruct.TIM_Pulse = 0;                            //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;     //�����������
	TIM_OC1Init(TIM1,&TIM_OCInitStruct);                       //��ʼ������Ƚϲ���
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;             //ѡ��PWM1ģʽ
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStruct.TIM_Pulse = 0;                            //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;     //�����������
	TIM_OC4Init(TIM1,&TIM_OCInitStruct);                       //��ʼ������Ƚϲ���
	
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);   //CH1ʹ��Ԥװ�ؼĴ���
	TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Enable);   //CH4ʹ��Ԥװ�ؼĴ���
	
	TIM_ARRPreloadConfig(TIM1, ENABLE);                //ʹ��TIM1��ARR�ϵ�Ԥװ�ؼĴ���
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
	
	TIM_Cmd(TIM1,ENABLE);                              //ʹ�ܶ�ʱ��1
}
/*****************   *********************************************************
�������ܣ����PWM�Լ���ʱ�жϳ�ʼ��
��ڲ�������ڲ�����arr���Զ���װֵ  psc��ʱ��Ԥ��Ƶ�� 
����  ֵ����
**************************************************************************/
void Servo_PWM_Init(u16 arr,u16 psc) 
{		 	
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
    GPIO_InitTypeDef GPIO_InitStruct;                             
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);   //ʹ�ܶ�ʱ��8ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);  //ʹ��GPIOC��ʱ��
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;          //�����������
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;                //PC6
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOC,&GPIO_InitStruct);
	
	TIM_TimeBaseInitStruct.TIM_Period = arr;              //�趨�������Զ���װֵ 
	TIM_TimeBaseInitStruct.TIM_Prescaler  = psc;          //�趨Ԥ��Ƶ��
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;//TIM���ϼ���ģʽ
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;         //����ʱ�ӷָ�
	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseInitStruct);       //��ʼ����ʱ��
	
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;             //ѡ��PWM1ģʽ
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStruct.TIM_Pulse = 0;                            //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;     //�����������
	TIM_OC1Init(TIM8,&TIM_OCInitStruct);                       //��ʼ������Ƚϲ���
	
	TIM_OC1PreloadConfig(TIM8,TIM_OCPreload_Enable);   //CH1ʹ��Ԥװ�ؼĴ���
	
	TIM_ARRPreloadConfig(TIM8, ENABLE);                //ʹ��TIM8��ARR�ϵ�Ԥװ�ؼĴ���
	
	TIM_CtrlPWMOutputs(TIM8,ENABLE);  //�߼���ʱ�������Ҫ�������
	
	TIM_Cmd(TIM8,ENABLE);                              //ʹ�ܶ�ʱ��8
	TIM8->CCR1=1500;
} 

