#include "led.h"

/**************************************************************************
�������ܣ�LED�ӿڳ�ʼ��
��ڲ������� 
����  ֵ����
****************************  *********************************************/
void LED_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);  //ʹ�� PORTB ʱ�� 
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;        //�������
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_13);                       //PB13 ����ߵ�ƽ
	
}

/**************************************************************************
�������ܣ�LED��˸
��ڲ�������˸Ƶ�� 
����  ֵ����
**************************************************************************/
void Led_Flash(u16 time)
{
	  static int temp;
	  if  (0==time) LED=0;                               //�͵�ƽ����
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

/********����Ƿ�ת��ҩƷ*************/
u8 LoadCount,NotLoadCount;
u8 Load_flag =0;  // 0��ʾ��ûת��ҩƷ��1��ʾת����ϣ�2��ʾҩƷ����
void LoadOrNot()
{
			
			if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0) ==0)
			{
				NotLoadCount = 0;
				if(Load_flag==0)
				{
				LoadCount++;
				if(LoadCount > 50) //3�γ���ִ�е���֮���ٴ��ж�
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
				
				if(Load_flag == 1)    //������װ�ع�ҩƷ������²����ж��Ƿ�����ҩƷ
				{
					NotLoadCount++;
					if(NotLoadCount > 50 )  //3�γ���ִ�е���֮���ٴ��ж�
					{
						Load_flag = 2;
						delay_ms(1000);
					delay_ms(1000);
					}
			  }
			}
}
