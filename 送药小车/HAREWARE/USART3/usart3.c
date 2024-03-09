
#include "usart3.h"
u8 Usart3_Receive;

/**************************************************************************
�������ܣ�����3��ʼ��
��ڲ�����pclk2:PCLK2 ʱ��Ƶ��(Mhz)    bound:������
����  ֵ����
**************************************************************************/
void uart3_init(u32 bound)
{  	
  USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	// ����ʹ��ʱ��
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);
	USART_DeInit(USART3);  //��λ����2 -> ����û��
 
	/* Configure USART3 Rx (PB.11) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;		  //????��?��??�꨺?	   
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* Configure USART3 Tx (PB.10) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;			  //?��
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = bound;						//
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//
	USART_InitStructure.USART_StopBits = USART_StopBits_1;			//
	USART_InitStructure.USART_Parity = USART_Parity_No;				//?TD��?��??
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   //
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//
	/* Configure USART3 */
	USART_Init(USART3, &USART_InitStructure);							//
	/* Enable USART3 Receive and Transmit interrupts */
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);                    //
	//USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);//
	/* Enable the USART3 */
	USART_Cmd(USART3, ENABLE);	 

/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


/*
********************************************************************************************************
�������ƣ�void USART1_IRQHandler(void)
�������ܣ�����3�жϷ�����
Ӳ�����ӣ�PA9----TXD��PA10----RXD
��    ע��
��    ��:  2017-2-8
*********************************************************************************************************
*/
u8 Di[8];//openmv��������1
u8 lukou;
u8 buff[11]={0};
u8 buff_time;
void USART3_IRQHandler(void)
{
	static int i=0;
	if(USART_GetITStatus( USART3, USART_IT_RXNE) != RESET) //���յ�����
	{	  
		buff[i]=USART3->DR;
			if(buff[0]!=0x55)//�����һ������0x55����i=0����Ч����
				i=-1;
			if(i==1&&(buff[1]!=0x56))//������ڶ��������˲���0x56��Ҳ��Ч ������
				i=-1;
			if(i==10)//��ʶ��Ŀ�꣩������11�������� 
				{
					for(i=0;i<8;i++)
					{		
						Di[i]=buff[i+2];
					}
					if(buff[10]==1)
					{
						lukou=1;
					}
					else lukou=0;
					i=-1;
				}
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);		
		i++;	
	} 	
}
