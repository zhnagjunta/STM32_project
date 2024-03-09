#include "usart2.h"
u8 Usart2_Receive;
 
void uart2_init(u32 bound)
{
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//GPIOA时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE); //使能USART2
	
	//USART1_TX   GPIOB.10
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA2
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA 2
   
  //USART1_RX	  GPIOA.3初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOB11

  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化NVIC寄存器
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART2, &USART_InitStructure); //初始化串口2
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART2, ENABLE);                    //使能串口2
}

int GetOpenmvDataCount  = 0;
uint8_t  Num=0, LoR =0, Finded_flag = 0, FindTask = 0;     //()
u8 LastNum;
u8 NumCount = 0;

u8 sendBuf[4];

uint8_t uart3_rxbuff;

u8 RoomNum, TargetNum;
u8 TASK=1;    //这个TASK可以传输给openmv，赋值openmv上的FindTask来控制openmv模板匹配的不同模式

char TargetRoom ='0';  //A, B, C, D, E, F, G, H;    //这八个字符对应着地图实际房间，里面的数字3—8会随机对应C-H

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
函数功能：串口2接收中断
入口参数：无
返回  值：无
**************************************************************************/
void SendDataToOpenmv(void)
{
	u8 i;
	if(LoR==0)
	//加上发送给openmv 的数据的代码 (帧头， 模板匹配模式选择标志位，模式2所需要匹配的数字，帧尾)   //不需要很高的发送频率			
		{
			for(i = 0; i <= 4; i++)   //将TASK和TargetNum打包一次性发送给openmv
			{ 
				sprintf((char *)sendBuf, "*%d%d&", TASK, TargetNum);
				USART2_Send(sendBuf,4);
			}
		}
		else
		{
			for(i = 0; i <= 4; i++)   //将TASK和TargetNum打包一次性发送给openmv
			{ 
				sprintf((char *)sendBuf, "*%d%d&", 0, TargetNum);
				USART2_Send(sendBuf,4);
			}
		}
}

void Openmv_Receive_Data(uint8_t com_data)
{
		uint8_t i;
		static uint8_t RxCounter1=0;//计数
		static uint16_t RxBuffer1[10]={0};
		static uint8_t RxState = 0;	
		if(RxState==0&&com_data==0x2C)  //0x2c帧头
		{			
			RxState=1;
			RxBuffer1[RxCounter1++]=com_data;  
		}
		else if(RxState==1&&com_data==0x12)  //0x12帧头
		{
			RxState=2;
			RxBuffer1[RxCounter1++]=com_data;
		}		
		else if(RxState==2)
		{		 
			RxBuffer1[RxCounter1++]=com_data;
			if(RxCounter1>=10||com_data == 0x5B)       //RxBuffer1接受满了,接收数据结束
			{
				RxState=3;
				 //正常情况下,运行到这RxCounter1 == 7？  7-5 = 2    openmv发送过来的一个数据包有8个
				Num =          RxBuffer1[RxCounter1-5]; 
				LoR =          RxBuffer1[RxCounter1-4];     //-1是左， 1是右，0表示还没有识别到任何数字
				Finded_flag =  RxBuffer1[RxCounter1-3];
				FindTask =      RxBuffer1[RxCounter1-2];		
				//RxCounter1-1是帧尾				
				//greenLED_Toggle;    //用来看是否接收数据的,电平翻转一次则成功接收一个数据，跟下面的一个意思
		  	GetOpenmvDataCount++;      
				//用来看1秒内成功解码多少个数据包的 需要在1s钟的延时中清除，帧率越高越准确，个位数的话偏差就大了
				//不如改一下解码代码，将openmv那里的帧率直接传过来	
			}
		}
		else if(RxState==3)		//检测是否接受到结束标志
		{
				if(RxBuffer1[RxCounter1-1] == 0x5B)
				{
							RxCounter1 = 0;
							RxState = 0;
						
				}
				else   //接收错误
				{
							RxState = 0;
							RxCounter1=0;
							for(i=0;i<10;i++)
							{
									RxBuffer1[i]=0x00;      //将存放数据数组清零
							}
				}
		} 
		else   //接收异常
		{
				RxState = 0;
				RxCounter1=0;
				for(i=0;i<10;i++)
				{
						RxBuffer1[i]=0x00;      //将存放数据数组清零
				}
		}
}

/*
********************************************************************************************************
函数名称：void USART2_IRQHandler(void)
函数功能：串口1中断服务函数
硬件连接：PA2----TXD，PA3----RXD
备    注：
日    期:  2017-2-8
*********************************************************************************************************
*/


void USART2_IRQHandler(void)
{
	static int i=0;//串口接收数组
	if(USART_GetITStatus( USART2, USART_IT_RXNE) != RESET) //接收到数据
	{	  
		i=USART2->DR;
		Openmv_Receive_Data(i);
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);		

		
	} 	
}

//等待openmv识别指定数字,并设置目标房间。 只有在复位后
void SetTargetRoom(void)
{
		//查寻目标病房号的缓冲区数据，跳转任务开始函数   //重启后第一次从openmv传来的数字即为目标房号
		/*一开始识别目标房间号*/  
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
	 else if(RoomNum >= 3)  //不能else if(3 <= Num <= 8)
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
	 //识别到的数字是3-8， 默认先给  TargetRoom = RoomH
	 //根据openmv识别到的数据,在送药的函数里面进行目标值的实时更改
}

