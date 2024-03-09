#include "moto.h"

void Motor_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能端口时钟
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;	//端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
  GPIO_Init(GPIOB, &GPIO_InitStructure);					      //根据设定参数初始化GPIOB
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能端口时钟
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_15;	//端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
  GPIO_Init(GPIOA, &GPIO_InitStructure);					      //根据设定参数初始化GPIOA
}

void Motor_PWM_Init(u16 arr,u16 psc)        
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
  GPIO_InitTypeDef GPIO_InitStruct;                             
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);   //使能定时器1时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);  //使能GPIOA的时钟
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;          //复用推挽输出
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_11;   //PA8 11
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	TIM_TimeBaseInitStruct.TIM_Period = arr;              //设定计数器自动重装值 
	TIM_TimeBaseInitStruct.TIM_Prescaler  = psc;          //设定预分频器
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;//TIM向上计数模式
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;         //设置时钟分割
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStruct);       //初始化定时器
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;             //选择PWM1模式
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStruct.TIM_Pulse = 0;                            //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;     //设置输出极性
	TIM_OC1Init(TIM1,&TIM_OCInitStruct);                       //初始化输出比较参数
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;             //选择PWM1模式
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStruct.TIM_Pulse = 0;                            //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;     //设置输出极性
	TIM_OC4Init(TIM1,&TIM_OCInitStruct);                       //初始化输出比较参数
	
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);   //CH1使能预装载寄存器
	TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Enable);   //CH4使能预装载寄存器
	
	TIM_ARRPreloadConfig(TIM1, ENABLE);                //使能TIM1在ARR上的预装载寄存器
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
	
	TIM_Cmd(TIM1,ENABLE);                              //使能定时器1
}
/*****************   *********************************************************
函数功能：舵机PWM以及定时中断初始化
入口参数：入口参数：arr：自动重装值  psc：时钟预分频数 
返回  值：无
**************************************************************************/
void Servo_PWM_Init(u16 arr,u16 psc) 
{		 	
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
    GPIO_InitTypeDef GPIO_InitStruct;                             
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);   //使能定时器8时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);  //使能GPIOC的时钟
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;          //复用推挽输出
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;                //PC6
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOC,&GPIO_InitStruct);
	
	TIM_TimeBaseInitStruct.TIM_Period = arr;              //设定计数器自动重装值 
	TIM_TimeBaseInitStruct.TIM_Prescaler  = psc;          //设定预分频器
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;//TIM向上计数模式
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;         //设置时钟分割
	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseInitStruct);       //初始化定时器
	
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;             //选择PWM1模式
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStruct.TIM_Pulse = 0;                            //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;     //设置输出极性
	TIM_OC1Init(TIM8,&TIM_OCInitStruct);                       //初始化输出比较参数
	
	TIM_OC1PreloadConfig(TIM8,TIM_OCPreload_Enable);   //CH1使能预装载寄存器
	
	TIM_ARRPreloadConfig(TIM8, ENABLE);                //使能TIM8在ARR上的预装载寄存器
	
	TIM_CtrlPWMOutputs(TIM8,ENABLE);  //高级定时器输出需要设置这句
	
	TIM_Cmd(TIM8,ENABLE);                              //使能定时器8
	TIM8->CCR1=1500;
} 

