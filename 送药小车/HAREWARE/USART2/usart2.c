#include "usart2.h"
u8 Usart2_Receive;
 
void uart2_init(u32 bound)
{
  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//GPIOAʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE); //ʹ��USART2
	
	//USART1_TX   GPIOB.10
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA2
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA 2
   
  //USART1_RX	  GPIOA.3��ʼ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOB11

  //Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��NVIC�Ĵ���
  
   //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

  USART_Init(USART2, &USART_InitStructure); //��ʼ������2
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ���2
}

int GetOpenmvDataCount  = 0;
uint8_t  Num=0, LoR =0, Finded_flag = 0, FindTask = 0;     //()
u8 LastNum;
u8 NumCount = 0;

u8 sendBuf[4];

uint8_t uart3_rxbuff;

u8 RoomNum, TargetNum;
u8 TASK=1;    //���TASK���Դ����openmv����ֵopenmv�ϵ�FindTask������openmvģ��ƥ��Ĳ�ͬģʽ

char TargetRoom ='0';  //A, B, C, D, E, F, G, H;    //��˸��ַ���Ӧ�ŵ�ͼʵ�ʷ��䣬���������3��8�������ӦC-H

void USART2_Send(unsigned char *tx_buf, int len)
{
		USART_ClearFlag(USART2, USART_FLAG_TC);
		USART_ClearITPendingBit(USART2, USART_FLAG_TXE);
	while(len--)
	{
		USART_SendData(USART2, *tx_buf);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) != 1);
		USART_ClearFlag(USART2, USART_FLAG_TC);
		USART_ClearITPendingBit(USART2, USART_FLAG_TXE);
		tx_buf++;
	}
	
}
/**************************************************************************
�������ܣ�����2�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
void SendDataToOpenmv(void)
{
	u8 i;
	if(LoR==0)
	//���Ϸ��͸�openmv �����ݵĴ��� (֡ͷ�� ģ��ƥ��ģʽѡ���־λ��ģʽ2����Ҫƥ������֣�֡β)   //����Ҫ�ܸߵķ���Ƶ��			
		{
			for(i = 0; i <= 4; i++)   //��TASK��TargetNum���һ���Է��͸�openmv
			{ 
				sprintf((char *)sendBuf, "*%d%d&", TASK, TargetNum);
				USART2_Send(sendBuf,4);
			}
		}
		else
		{
			for(i = 0; i <= 4; i++)   //��TASK��TargetNum���һ���Է��͸�openmv
			{ 
				sprintf((char *)sendBuf, "*%d%d&", 0, TargetNum);
				USART2_Send(sendBuf,4);
			}
		}
}

void Openmv_Receive_Data(uint8_t com_data)
{
		uint8_t i;
		static uint8_t RxCounter1=0;//����
		static uint16_t RxBuffer1[10]={0};
		static uint8_t RxState = 0;	
		if(RxState==0&&com_data==0x2C)  //0x2c֡ͷ
		{			
			RxState=1;
			RxBuffer1[RxCounter1++]=com_data;  
		}
		else if(RxState==1&&com_data==0x12)  //0x12֡ͷ
		{
			RxState=2;
			RxBuffer1[RxCounter1++]=com_data;
		}		
		else if(RxState==2)
		{		 
			RxBuffer1[RxCounter1++]=com_data;
			if(RxCounter1>=10||com_data == 0x5B)       //RxBuffer1��������,�������ݽ���
			{
				RxState=3;
				 //���������,���е���RxCounter1 == 7��  7-5 = 2    openmv���͹�����һ�����ݰ���8��
				Num =          RxBuffer1[RxCounter1-5]; 
				LoR =          RxBuffer1[RxCounter1-4];     //-1���� 1���ң�0��ʾ��û��ʶ���κ�����
				Finded_flag =  RxBuffer1[RxCounter1-3];
				FindTask =      RxBuffer1[RxCounter1-2];		
				//RxCounter1-1��֡β				
				//greenLED_Toggle;    //�������Ƿ�������ݵ�,��ƽ��תһ����ɹ�����һ�����ݣ��������һ����˼
		  	GetOpenmvDataCount++;      
				//������1���ڳɹ�������ٸ����ݰ��� ��Ҫ��1s�ӵ���ʱ�������֡��Խ��Խ׼ȷ����λ���Ļ�ƫ��ʹ���
				//�����һ�½�����룬��openmv�����֡��ֱ�Ӵ�����	
			}
		}
		else if(RxState==3)		//����Ƿ���ܵ�������־
		{
				if(RxBuffer1[RxCounter1-1] == 0x5B)
				{
							RxCounter1 = 0;
							RxState = 0;
						
				}
				else   //���մ���
				{
							RxState = 0;
							RxCounter1=0;
							for(i=0;i<10;i++)
							{
									RxBuffer1[i]=0x00;      //�����������������
							}
				}
		} 
		else   //�����쳣
		{
				RxState = 0;
				RxCounter1=0;
				for(i=0;i<10;i++)
				{
						RxBuffer1[i]=0x00;      //�����������������
				}
		}
}

/*
********************************************************************************************************
�������ƣ�void USART2_IRQHandler(void)
�������ܣ�����1�жϷ�����
Ӳ�����ӣ�PA2----TXD��PA3----RXD
��    ע��
��    ��:  2017-2-8
*********************************************************************************************************
*/


void USART2_IRQHandler(void)
{
	static int i=0;//���ڽ�������
	if(USART_GetITStatus( USART2, USART_IT_RXNE) != RESET) //���յ�����
	{	  
		i=USART2->DR;
		Openmv_Receive_Data(i);
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);		

		
	} 	
}

//�ȴ�openmvʶ��ָ������,������Ŀ�귿�䡣 ֻ���ڸ�λ��
void SetTargetRoom(void)
{
		//��ѰĿ�겡���ŵĻ��������ݣ���ת����ʼ����   //�������һ�δ�openmv���������ּ�ΪĿ�귿��
		/*һ��ʼʶ��Ŀ�귿���*/  
		if(Finded_flag == 1)
		{
			 RoomNum = Num;
		}
		
		
		else if(Finded_flag == 0)
		{
			RoomNum = 0;   
			LastNum = 0;
			NumCount = 0;
		}
	
		
	 if(RoomNum ==  1) 
	 {
		 TargetRoom = 'A';
		 TASK = 2;   
		 Flag_Stop=0;
	 }
	 else if(RoomNum == 2)
	 {
		 TargetRoom = 'B';
		 TASK = 2;  
		 Flag_Stop=0;
			
	 }
	 else if(RoomNum >= 3)  //����else if(3 <= Num <= 8)
	 {
		 TargetRoom = 'G';
		 TASK = 2;  
		 Flag_Stop=0;
	 }
	 
   switch(RoomNum)
		{
			case 1:
				TargetNum = 1;
			break;
			case 2:
				TargetNum = 2;
			break;
			case 3:
				TargetNum = 3;
			break;
			case 4:
				TargetNum = 4;
			break;
			case 5:
				TargetNum = 5;
			break;
			case 6:
				TargetNum = 6;
			break;
			case 7:
				TargetNum = 7;
			break;
			case 8:
				TargetNum = 8;
			break;	 	 		
		}
	 //ʶ�𵽵�������3-8�� Ĭ���ȸ�  TargetRoom = RoomH
	 //����openmvʶ�𵽵�����,����ҩ�ĺ����������Ŀ��ֵ��ʵʱ����
}

