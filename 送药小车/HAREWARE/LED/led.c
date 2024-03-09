#include "led.h"

/**************************************************************************
函数功能：LED接口初始化
入口参数：无 
返回  值：无
****************************  *********************************************/
void LED_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);  //使能 PORTB 时钟 
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;        //推挽输出
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_13);                       //PB13 输出高电平
	
}

/**************************************************************************
函数功能：LED闪烁
入口参数：闪烁频率 
返回  值：无
**************************************************************************/
void Led_Flash(u16 time)
{
	  static int temp;
	  if  (0==time) LED=0;                               //低电平点亮
	  else if(++temp==time)	LED=~LED,temp=0;
}

void Load_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}

/********检测是否转载药品*************/
u8 LoadCount,NotLoadCount;
u8 Load_flag =0;  // 0表示还没转载药品，1表示转载完毕，2表示药品送完
void LoadOrNot()
{
			
			if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0) ==0)
			{
				NotLoadCount = 0;
				if(Load_flag==0)
				{
				LoadCount++;
				if(LoadCount > 50) //3次程序执行到这之后再次判断
				{
					Load_flag = 1;
					delay_ms(1000);
					delay_ms(1000);
				}
			}
			}
			else if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==1)
			{
				LoadCount = 0;
				
				if(Load_flag == 1)    //必须先装载过药品的情况下才能判断是否拿走药品
				{
					NotLoadCount++;
					if(NotLoadCount > 50 )  //3次程序执行到这之后再次判断
					{
						Load_flag = 2;
						delay_ms(1000);
					delay_ms(1000);
					}
			  }
			}
}
