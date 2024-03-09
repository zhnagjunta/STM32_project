#include "sys.h"
typedef enum
{
  left_90,
	right_90,
	back_180,
	straight,
	fianl
}spin_dir_t;

u16 Speed_L,Speed_R;
char Speed_L1[5];
char Speed_R1[5];
u8 Spin_start_flag , Spin_succeed_flag , Stop_Flag;
u8  Turn_flag = 0 ;
u8 stop_count , spin_count;
u8 Do_count = 0, Do2_count = 0,back1_count = 0,back2_count = 0,back3_count = 0;
int WaitTime_ms=500;
u8 lukoutime=0;
#define speed 25
#define stop_TIME 1.4
#define stop 400 
#define open 0 
#define close 1 
u16 SendTime = 0;

u8 Mode=0,BLUETOOTH_Mode=1,ELE_Line_Patrol_Mode=0,CCD_Line_Patrol_Mode=0;      //小车的模式,蓝牙遥控、CCD巡线、ELE巡线
u8 Flag_Way=0,Flag_Show=0,Flag_Stop=1,Flag_Next;       //停止标志位和 显示标志位 默认停止 显示打开
int Encoder_Left,Encoder_Right;                        //左右编码器的脉冲计数
int Flag_Direction;  
float Velocity,Velocity_Set,Angle_Set,Turn;
int Motor_A,Motor_B,Servo,Target_A,Target_B;           //电机舵机控制相关           
int Voltage;                                           //电池电压采样相关的变量
float Show_Data_Mb;                                    //全局显示变量，用于显示需要查看的数据
u8 delay_50,delay_flag; 							   //延时变量
float Velocity_KP=12,Velocity_KI=12;	       //速度控制PID参数
u16 ADV[128]={0};              
u8 Bluetooth_Velocity=30,APP_RX,APP_Flag;                        //蓝牙遥控速度和APP接收的数据
u8 CCD_Zhongzhi,CCD_Yuzhi,PID_Send,Flash_Send;          //线性CCD FLASH相关
int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor;      //电磁巡线相关
u16 PID_Parameter[10],Flash_Parameter[10];              //Flash相关数组 

void error_back1(void);
void error_back2(void);
void error_back3(void);
void Find(void);
void spin_Turn(spin_dir_t zhuanxiang);
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置中断分组为2
	delay_init(72);                 //=====延时初始化
	JTAG_Set(JTAG_SWD_DISABLE);     //=====关闭JTAG接口
	//JTAG_Set(SWD_ENABLE);         //=====打开SWD接口 可以利用主板的SWD接口调试
	LED_Init();                     //=====初始化与 LED 连接的硬件接口
	KEY_Init();
	OLED_Init();                    //=====OLED初始化
	Encoder_Init_TIM2();            //=====初始化编码器（TIM2的编码器接口模式） 
	Encoder_Init_TIM4();            //=====初始化编码器（TIM3的编码器接口模式）
	EXTIX_Init();                   //=====按键初始化(外部中断的形式)	
	Adc_Init();                     //=====电池电压采样adc初始化
	delay_ms(300);                  //=====延时启动
	uart_init(115200);                //=====初始化串口1
	Motor_Init();
	Motor_PWM_Init(7199,0);  	    //=====初始化PWM 10KHZ，用于驱动电机 
//ccd_Init();
	uart2_init(115200); 				//=====串口2初始化 蓝牙
	uart3_init(115200);
	Flash_Read();	                //=====读取PID参数
	Timer5_Init(49,7199);
	Load_Init();
	while(1)
	{     
		if(SendTime >= 4)
		{
			SendTime = 0;
			SendDataToOpenmv();   //不能太快，否则会超过openmv的串口接收数据缓冲区
			LoadOrNot();
			oled_show();          //===显示屏打开
		}
		if(Load_flag == 1&&TASK == 1)   
		{
			SetTargetRoom();
		}
   		if(TASK == 2)    
	 	{
			oled_show();          //===显示屏打开
			if(Load_flag == 1)    //运送药物
			{
				if(TargetRoom == 'A')   //这个位置理解为A房，数字为1
					{
						switch(Do_count)
						{
							case 0: 	
							{								
								Stop_Flag = 0;	
								 while(Stop_Flag==0)Find();     //此时 Stop_Flag ==0  完成后自动置1        //近端病房不预留openmv识别的位置
									Do_count++;	
							}
							break;					 
							case 1:
								 if(Stop_Flag ==1)          //直行、转弯函数交替使用时，可以不手动将标志位清零 遇到路口
								 {
									 
									 spin_Turn(left_90);   //此时Spin_succeed_flag== 0 ,完成后自动置1 左转
									 Do_count++;
								 }
							break;							 
							case 2:
								 if(Spin_succeed_flag == 1)
								 {
									 Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();      								 
									 }
									 Do_count++;
									 spin_Turn(fianl);
								 }
							break;							 
							case	3:
								if(Spin_succeed_flag ==1)//到终点
								{
									Velocity=0;
									Turn=0;
									Flag_Stop=1;
								}
							break;
						}			 
					}										
				else if(TargetRoom == 'B')
					{
						switch(Do_count)
						{
							case 0:	
							{
								 Stop_Flag = 0;	
								 while(Stop_Flag==0)Find();  						
								 Do_count++;							
							}
							break;				 
							case 1:
								 if(Stop_Flag ==1)
								 {
									 spin_Turn(right_90);   //此时Spin_succeed_flag== 0 ,完成后自动置1
									 Do_count++;
								 }
							break;							 
							case 2:
								 if(Spin_succeed_flag == 1)
								 {
									 Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();      								 
									 }
									 spin_Turn(fianl);
									 Do_count++;
								 }
							break;				 
							case	3:
								if(Spin_succeed_flag ==1)
								{
									Velocity=0;
									Turn=0;
									Flag_Stop=1;
								}
							break;
						}			 
					}				
				//中端病房和远端病房需要发送特定标志位					
				else      //TargetRoom == 'C' || TargetRoom == 'D' || TargetRoom == 'E' || TargetRoom == 'F' || TargetRoom == 'G' || TargetRoom == 'H'
					{
						switch(Do_count)//直走到第一个路口
						{
							case 0: 
							{
								Stop_Flag = 0;	
								while(Stop_Flag==0)
									 {
										 Find();
									 }
								Do_count++;
								SendDataToOpenmv();  //打开数字识别
							}
							break;							
							case 1:
								if(Stop_Flag ==1) //遇到第一个路口
								 {
									 
									 spin_Turn(straight);   //此时Spin_succeed_flag== 0 ,完成后自动置1
									 Do_count++;
									 SendDataToOpenmv(); //打开模板匹配数字
								 }
							case 2:
								if(Spin_succeed_flag == 1)//第一个路口往第二个路口直走
								 {
									
									 Stop_Flag = 0;	
									 lukou_straight=1;
									 while(lukou_straight<=stop_TIME*200) Find(); 
									 lukou_straight=0;
									 Velocity=0;
									 Turn=0;
									 delay_ms(stop);
									 delay_ms(stop);	
									 
									 while(Stop_Flag==0)
									 {
										 Find();
									 }
									 Do_count++;
								 }								
							case 3:       //判定是否识别到数字，且该数字的位置偏左还是偏右  第二个路口
								if(Stop_Flag == 1)
								{
											if(RoomNum == TargetNum && LoR == 1)  //识别到了目标数字, 且其位置偏左  
											{
												Do_count  = 8;  //后续直至左转并直行到病房门口的函数
												TargetRoom = 'C';
											}
											else if(RoomNum == TargetNum && LoR == 2 )  //识别到目标数字, 且其位置偏右
											{
												Do_count  = 8;        //后续直至右转并直行到病房门口的函数
												TargetRoom = 'D';
											}	
											if(LoR==0)  //没识别到就直走到第三个路口停止
												{
													spin_Turn(straight);
													lukou_straight=1;
													 while(lukou_straight<=stop_TIME*200) Find(); 
													 lukou_straight=0;
													 Velocity=0;
													 Turn=0;
													 delay_ms(stop);
													 delay_ms(stop);	
													Stop_Flag = 0;	
													while(Stop_Flag==0)
													{
													Find();
													}
													Velocity=0;
													 Turn=0;
													Do_count++;
											}	
								}
							break;		
							case 4://第三个路口准备转弯
								if(Stop_Flag == 1)
								{
									if(RoomNum == TargetNum && LoR == 1)  //识别到了目标数字, 且其位置偏左  
											{
												Do_count++; 
												TargetRoom = 'E';      //先假定是E
												SendDataToOpenmv(); //关闭模板匹配数字
											}
									else if(RoomNum == TargetNum && LoR == 2 )  //识别到目标数字, 且其位置偏右
											{
												Do_count++;
												TargetRoom = 'G';    //先假定是G
												SendDataToOpenmv(); //关闭模板匹配数字	
											}	
									if(LoR==0){
												 error_back1();
											 }
								}
							break;				
							case 5://转弯
								if(Stop_Flag == 1)
								{
									Do_count++;
									if(TargetRoom == 'E')
									{
										spin_Turn(left_90);
										LoR=0;//清空预转弯方向
										SendDataToOpenmv(); //打开模板匹配数字
									}
									else if(TargetRoom == 'G')
									{
										spin_Turn(right_90);
										LoR=0;//清空预转弯方向
										SendDataToOpenmv(); //打开模板匹配数字
									}
								}
							break;		
								case 6: //直走到第四个路口
								if(Spin_succeed_flag == 1)
								{
									Do_count++;
									Stop_Flag = 0;	
									lukou_straight=1;
									 while(lukou_straight<=1.2*200) Find(); 
									 lukou_straight=0;
									 Velocity=0;
									 Turn=0;
									 delay_ms(stop);
									 delay_ms(stop);	
									 delay_ms(stop);
									 delay_ms(stop);
									 delay_ms(stop);
									while(Stop_Flag==0)
									 {
										 Find();
									 }
								}
							break;	
							case 7: //第四个路口准备转弯
								if(Stop_Flag == 1)
								{
									if(TargetRoom == 'E')
										{
												if(RoomNum == TargetNum && LoR == 1)    //识别到RoomNum, 且其位置偏左    
											 {
												 Do_count ++;  
												 TargetRoom = 'E';
												 SendDataToOpenmv(); //关闭模板匹配数字	
											 }										 
											 if(RoomNum == TargetNum && LoR == 2)   //一定时间内识别到RoomNum, 且其位置偏右
											 {
												 Do_count++;  
												 TargetRoom = 'F';
												 SendDataToOpenmv(); //关闭模板匹配数字	
											 }
											 if(LoR==0){
//												 Velocity=0;
//													Turn=0;
//													Flag_Stop=1;
												 error_back2();
											 }
											
										 }			
										else if(TargetRoom == 'G')
										{
												if(RoomNum == TargetNum && LoR == 1)    //一定时间内识别到RoomNum, 且其位置偏左    
											 {
												 Do_count++; 
												 TargetRoom = 'G';
												 SendDataToOpenmv(); //关闭模板匹配数字	
											 }										 											 
											 if(RoomNum == TargetNum && LoR == 2)   //一定时间内识别到RoomNum, 且其位置偏右
											 {
												 Do_count++;  
												 TargetRoom = 'H';
												 SendDataToOpenmv(); //关闭模板匹配数字	
											 }
											 
											 if(LoR==0){
//												 Velocity=0;
//													Turn=0;
//													Flag_Stop=1;
												 error_back3();
											 }
										}	
								}
							break;	
							case 8://最后路口转弯
								if(Stop_Flag == 1)
								{
									Do_count++;
									if(TargetRoom == 'C' || TargetRoom == 'E' || TargetRoom == 'G')
									{
										spin_Turn(left_90);
									}
									else if(TargetRoom == 'D' || TargetRoom == 'F' || TargetRoom == 'H')
									{
										spin_Turn(right_90);
									}
								}
							break;							
							case 9: //直走到终点
								if(Spin_succeed_flag == 1)
								{
									Do_count++;
									Stop_Flag = 0;	
									while(Stop_Flag==0)Find();
									spin_Turn(fianl);
								}
							break;							
							case 10:
								if(Spin_succeed_flag == 1) //送完药
								{		
										Velocity=0;
									Turn=0;
									Flag_Stop=1;
								}
							break;								
						}
					}
			}
			//送药完毕返回 
			else if(Load_flag == 2)       // 全部房间都可返回了，就是远端的三叉路口那可能有时候会出现问题。
			{
				Flag_Stop=0;
					if(TargetRoom == 'A')
					{
						switch(Do2_count)
						{
							case 0:
								Do2_count++;
								spin_Turn(back_180 );
							break;
							case 1:
								if(Spin_succeed_flag == 1) //转弯成功
								{					
									Stop_Flag = 0;	
									while(Stop_Flag==0)Find();
									Do2_count++;	
								}
							break;
							case 2:
								 if(Stop_Flag ==1)
								 {
									 spin_Turn(right_90 );   //此时Spin_succeed_flag== 0 ,完成后自动置1
									 Do2_count++;
								 }
							break;					 
							case 3:
								 if(Spin_succeed_flag == 1)
								 {					 
									 Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();
									 }
									 spin_Turn(fianl);
									 Do2_count++;
								 }
							break;
							case	4:
								if(Spin_succeed_flag ==1)
								{
									Velocity=0;
									Turn=0;
									Flag_Stop=1;
									LED=open;
								}
							break;
						}	               
					}		
					else if(TargetRoom == 'B')
					{
						switch(Do2_count)
						{
							case 0:
								Do2_count++;	 
								spin_Turn(back_180 );
					 		
							break;					
							case 1:
								if(Spin_succeed_flag == 1)
								{ 
									Stop_Flag = 0;	
								  while(Stop_Flag==0)Find();
									Do2_count++;
								}
							break;
							case 2:
								 if(Stop_Flag ==1)
								 {
									 spin_Turn(left_90 ); 	
									Do2_count++;
								 }
							break;						 
							case 3:
								 if(Spin_succeed_flag == 1)
								 {
									 Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();
									 }
									  spin_Turn(fianl);
									 Do2_count++;
								 }
							break;
							case 4:
								if(Spin_succeed_flag ==1)
								{
									Velocity=0;
									Turn=0;
									Flag_Stop=1;
									LED=open;
								}
							break;
						}
					}			
					else if(TargetRoom == 'C')   //中端病房   这两个与近端病房的差别只是在最后直走的时候走多了几十cm而已
					{
						switch(Do2_count)
						{
							case 0:
								{
								spin_Turn(back_180);
								Do2_count++;
							}
							break;
							case 1:
								if(Spin_succeed_flag == 1)
								{		 
									Stop_Flag = 0;	
								  while(Stop_Flag==0)Find();
									Do2_count++;
								}
							break;		 
							case 2:
								 if(Stop_Flag ==1)
								 {
									spin_Turn(right_90 );   //此时Spin_succeed_flag== 0 ,完成后自动置1
									Spin_succeed_flag=1;
									Do2_count++;
								 }
							break;
							case 3:
								 if(Spin_succeed_flag == 1)
								 {
									 Stop_Flag = 0;	
									 while(Stop_Flag==0)Find();
									 spin_Turn(straight);
									 Do2_count++;
									 Stop_Flag = 1;	
								 }
							break;		 
							case	4:
								if(Stop_Flag ==1)
								{
									  Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();
									 }
									 Do2_count++;
								}
							break;
								case	5:
								{
									  Stop_Flag = 0;	
									 while	(Stop_Flag==0)
									 {
										 Find();
									 }
									  spin_Turn(fianl);
									 Stop_Flag = 0;	
									 while	(Stop_Flag==0)
									 {
										 Find();
									 }
									 Do2_count++;
								}
							break;
				       case	6:
								{
									 Velocity=0;
									Turn=0;
									Flag_Stop=1;
									LED=open;
								}
							break;
						}
					}	
					else if(TargetRoom == 'D')
					{
						switch(Do2_count)
						{
							case 0:		 
								spin_Turn(back_180);
							Do2_count++;
							break;
							
							case 1:
								if(Spin_succeed_flag == 1)
								{
							 						 
									Stop_Flag = 0;	
								  while(Stop_Flag==0)Find();    
									Do2_count++;
								}
							break;					 
							case 2:
								 if(Stop_Flag ==1)
								 {
									 
									 spin_Turn(left_90);   //此时Spin_succeed_flag== 0 ,完成后自动置1
									 Do2_count++;
								 }
							break;
							case 3:
								 if(Spin_succeed_flag == 1)
								 {
							 
									 Stop_Flag = 0;	
									 while(Stop_Flag==0)Find();
									 spin_Turn(straight);
									 Do2_count++;
									 Stop_Flag = 1;	
								 }
							break;							 
							case	4:
								if(Stop_Flag ==1)
								{
									Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();
									 }
									 Do2_count++;
								}
							break;
								case	5:
								{
									
									Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();
									 }
									 spin_Turn(fianl);
									 Stop_Flag = 0;	
									 while	(Stop_Flag==0)
									 {
										 Find();
									 }
									 Do2_count++;
								}
							break;
							 case 6:
								{
									 Velocity=0;
									Turn=0;
									Flag_Stop=1;
									LED=open;
								}
								break;	

						}	
					}
else if(TargetRoom == 'E')   //远端病房   四个只是在两个路口的转向不同而已
					{
						switch(Do2_count)
						{
							case 0:
							{
								Do2_count++;
								spin_Turn(back_180);
							}
							break;							
							case 1:
								if(Spin_succeed_flag == 1)
								{
									Do2_count++;								 
									Stop_Flag = 0;	
									 while(Stop_Flag==0)Find();    
								}
							break;								
							case 2:
								 if(Stop_Flag ==1)
								 {
									 Do2_count++;
									 spin_Turn(right_90);   //此时Spin_succeed_flag== 0 ,完成后自动置1
								 }
							break;								 
							case 3:
								if(Spin_succeed_flag == 1)
								{
									Do2_count++;
									Stop_Flag = 0;	
									 while(Stop_Flag==0)Find();
								}
							break;								
							case 4:
								if(Stop_Flag == 1)
								{
									Do2_count++;
									spin_Turn(right_90);
								}
							break;
							case 5:
								 if(Spin_succeed_flag == 1)
								 {
							 
									 Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();
									 }
									 spin_Turn(straight);
									 Do2_count++;
									 Stop_Flag = 1;	
								 }
							break;							 
							case	6:
								if(Stop_Flag ==1)
								{
									Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();
									 }
									 Do2_count++;
								}
							break;
								case	7:
								{
									
									  Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();
									 }
									 spin_Turn(fianl);
									 Do2_count++;
								}
							break;
								case	8:
								{
									  Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();
									 }
									  spin_Turn(fianl);
									 Do2_count++;
								}
							break;
							 case	9:
								{
									 Velocity=0;
									Turn=0;
									Flag_Stop=1;
									LED=open;
								}
								break;	
						}
					}					
					else if(TargetRoom == 'F')
					{
						switch(Do2_count)
						{
							case 0:
							{
								Do2_count++;
								spin_Turn(back_180);
							}
							break;
							
							case 1:
								if(Spin_succeed_flag == 1)
								{
									Do2_count++;								 
									Stop_Flag = 0;	
									 while(Stop_Flag==0)Find();    
								}
							break;
								
							case 2:
								 if(Stop_Flag ==1)
								 {
									 Do2_count++;
									 spin_Turn(left_90);   //此时Spin_succeed_flag== 0 ,完成后自动置1
								 }
							break;
							case 3:
								if(Spin_succeed_flag == 1)
								{
									Do2_count++;
									Stop_Flag = 0;	
									 while(Stop_Flag==0)Find();
								}
							break;
								
							case 4:
								if(Stop_Flag == 1)
								{
									Do2_count++;
									spin_Turn(right_90);
								}
							break;
							case 5:
								 if(Spin_succeed_flag == 1)
								 {
							 
									 Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();
									 }
									 spin_Turn(straight);
									 Do2_count++;
									 Stop_Flag = 1;	
								 }
							break;							 
							case	6:
								if(Stop_Flag ==1)
								{
									Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();
									 }
									 Do2_count++;
								}
							break;
								case	7:
								{
									  Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();
									 }
									 spin_Turn(fianl);
									 Do2_count++;
								}
							break;
								case	8:
								{
									  Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();
									 }
									  spin_Turn(fianl);
									 Do2_count++;
								}
							break;
							 case	9:
								{
									 Velocity=0;
									Turn=0;
									Flag_Stop=1;
									LED=open;
								}
								break;	
							
						}
					}
					else if(TargetRoom == 'G')
					{
						switch(Do2_count)
						{
							case 0:
							{
								Do2_count++;
								spin_Turn(back_180);
							}
							break;		
							case 1:
								if(Spin_succeed_flag == 1)
								{
									Do2_count++;								 
									Stop_Flag = 0;	
									 while(Stop_Flag==0)Find();    
								}
							break;
								
							case 2:
								 if(Stop_Flag ==1)
								 {
									 Do2_count++;
									 spin_Turn(right_90);   //此时Spin_succeed_flag== 0 ,完成后自动置1
								 }
							break;
							case 3:
								if(Spin_succeed_flag == 1)
								{
									Do2_count++;
									Stop_Flag = 0;	
									 while(Stop_Flag==0)Find();
								}
							break;
								
							case 4:
								if(Stop_Flag == 1)
								{
									Do2_count++;
									spin_Turn(left_90);
								}
							break;
							case 5:
								 if(Spin_succeed_flag == 1)
								 {
							 
									 Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();
									 }
									 spin_Turn(straight);
									 Do2_count++;
									 Stop_Flag = 1;	
								 }
							break;							 
							case	6:
								if(Stop_Flag ==1)
								{
									Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();
									 }
									 Do2_count++;
								}
							break;
								case	7:
								{
									  Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();
									 }
									 spin_Turn(fianl);
									 Do2_count++;
								}
							break;
								case	8:
								{
									  Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();
									 }
									  spin_Turn(fianl);
									 Do2_count++;
								}
							break;
							 case	9:
								{
									 Velocity=0;
									Turn=0;
									Flag_Stop=1;
									LED=open;
								}
								break;	
							
						}
					}
					else if(TargetRoom == 'H')
					{
						switch(Do2_count)
						{
							case 0:
							{
								Do2_count++;
								spin_Turn(back_180);
							}
							break;
							
							case 1:
								if(Spin_succeed_flag == 1)
								{
									Do2_count++;								 
									Stop_Flag = 0;	
									 while(Stop_Flag==0)Find();  
								}
							break;
								
							case 2:
								 if(Stop_Flag ==1)
								 {
									 Do2_count++;
									 spin_Turn(left_90);   //此时Spin_succeed_flag== 0 ,完成后自动置1
								 }
							break;
							case 3:
								if(Spin_succeed_flag == 1)
								{
									Do2_count++;
									Stop_Flag = 0;	
									 while(Stop_Flag==0)Find();
								}
							break;
								
							case 4:
								if(Stop_Flag == 1)
								{
									Do2_count++;
									spin_Turn(left_90);
								}
							break;
							case 5:
								 if(Spin_succeed_flag == 1)
								 {
							 
									 Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();
									 }
									 spin_Turn(straight);
									 Do2_count++;
									 Stop_Flag = 1;	
								 }
							break;							 
							case	6:
								if(Stop_Flag ==1)
								{
									Stop_Flag = 0;	
									while(Stop_Flag==0)
									 {
										 Find();
									 }
									 Do2_count++;
								}
							break;
								case	7:
								{
									  Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();
									 }
									 spin_Turn(fianl);
									 Do2_count++;
								}
							break;
								case	8:
								{
									  Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();
									 }
									  spin_Turn(fianl);
									 Do2_count++;
								}
							break;
							 case	9:
								{
									 Velocity=0;
									Turn=0;
									Flag_Stop=1;
									LED=open;
								}
								break;	
						}
					}
			 }				 
	  	}				
	}
}


void Find(void)
{	  	
	Spin_start_flag = 0;
	Spin_succeed_flag = 0;  
		if(Di[0]==1)// 最左边寻到线 45
	{
		Velocity=speed;
			Turn=-4;
	}
	else if(Di[7]==1)// 最右边寻到线 55
	{
		Velocity=speed;
			Turn=4;
	}
	else if(Di[1]==1)// 更左边寻到线
	{
		Velocity=speed;
			Turn=-3;
	}
	else if(Di[6]==1)// 更右边寻到线
	{
		Velocity=speed;
			Turn=3;
	}
	else if(Di[2]==1)// 左边寻到线
	{
		Velocity=speed;
			Turn=-2;
	}
	else if(Di[5]==1)// 右边寻到线
	{
		Velocity=speed;
			Turn=2;
	}
	else if((Di[0] == 0)&&(Di[1] == 0)&&(Di[2] == 0)&&(Di[3] == 1)&&(Di[4] == 0)&&(Di[5] == 0)&&(Di[6] == 0)&&(Di[7] == 0))// 线在中间，前进
	{
		Velocity=speed;
			Turn=-1;
	}
	else if((Di[0] == 0)&&(Di[1] == 0)&&(Di[2] == 0)&&(Di[3] == 0)&&(Di[4] == 1)&&(Di[5] == 0)&&(Di[6] == 0)&&(Di[7] == 0))// 线在中间，前进
	{
		Velocity=speed;
			Turn=1;
	}
	else if((Di[0] == 0)&&(Di[1] == 0)&&(Di[2] == 0)&&(Di[3] == 1)&&(Di[4] == 1)&&(Di[5] == 0)&&(Di[6] == 0)&&(Di[7] == 0))// 线在中间，前进
	{
		Velocity=speed;
			Turn=0;
	}
	if(lukou==1) 
	{
		lukoutime++;
	}
	else lukoutime=0;
	if(lukoutime>5) Stop_Flag=1; //遇到路口停止
}

/******转向完成时  Spin_start_flag == 0 && Spin_succeed_flag == 1  ********/

/*转角有左转90，右转90，和转180三种情况。*/

void spin_Turn(spin_dir_t zhuanxiang)   //传入小车的轮距(mm) 175mm     //不需要很精准，转弯后直接用直走时的巡线函数回正车身就好
{
	  Stop_Flag = 0;   //执行转弯时，将直走完成的标志位清零. 即如果上一次是直行，这次是转弯，则不用在业务代码里手动置位
	  
    Spin_start_flag = 1;   
	  Spin_succeed_flag = 0;  
		
	
	  /**************要是转弯后不能回到线上，可以改这里的转向系数**************/
		if(zhuanxiang == left_90)  //openmv识别到需要往左边病房走
		{
			Velocity=20;
			Turn=0;
			delay_ms(800);//越长时间就前进越多
			Velocity=0;
			Turn=-20;
			delay_ms(470);//越长时间就转越多
			Velocity=0;
			Turn=0;
			delay_ms(200);
		}
		else if(zhuanxiang == right_90)  //openmv识别到需要往右边病房走
		{
			Velocity=20;
			Turn=0;
			delay_ms(800);//越长时间就前进越多
			Velocity=0;
			Turn=20;
			delay_ms(470);//越长时间就转越多
			Velocity=0;
			Turn=0;
			delay_ms(200);
		}
		else if(zhuanxiang == back_180)
		{	
			Velocity=0;
			Turn=20;
			delay_ms(930);
			Velocity=0;
			Turn=0;
			delay_ms(500);
		}
		else if(zhuanxiang == straight)
		{
			Velocity=20;
			Turn=0;
			delay_ms(500);
		}
		else if(zhuanxiang == fianl)
		{
			Velocity=20;
			Turn=0;
			delay_ms(1000);
		}
		Spin_succeed_flag=1;
}

void error_back1(void)
{
	Flag_Stop=0;
	switch(back1_count)
						{
							case 0:
							{	
							back1_count++;								
							spin_Turn(back_180);
							while(Stop_Flag==0)Find();  
							}
							break;
							case 1:
							{
								if(Spin_succeed_flag == 1)
								{
							 					 
									Stop_Flag = 0;	
								  while(Stop_Flag==0)Find();    
									back1_count++;
								}
							}
							break;
														
							case 2:
								{
								 if(Stop_Flag ==1)
								 {
									 Stop_Flag = 0;	
									 while(Stop_Flag==0)Find();
									 spin_Turn(straight);
									 back1_count++;
									 Stop_Flag = 1;	
								 }
							  }
							break;
							
							case 3:
							{
								 if(Stop_Flag ==1)
								{
									Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();
									 }
									 back1_count++;
								}
							}
							break;
												
							case	4:
								if(Stop_Flag ==1)
								{							
									Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();
									 }
									 spin_Turn(straight);
									 Stop_Flag = 0;	
								while	(Stop_Flag==0)
									 {
										 Find();
									 }
									 back1_count++;
								}
							break;
								case	5:
								{
									spin_Turn(back_180);
									back1_count++;
								}
							break;
							 case 6:
								{
									if(Spin_succeed_flag == 1)
								{
								Do_count = 0;
								Do2_count = 0;
								}
								}
								break;	

						}
				back1_count=0;
}

void error_back2(void)
{
	Flag_Stop=0;
	switch(back2_count)
						{
							case 0:	{	 
							spin_Turn(back_180);
							back2_count++;
								while(Stop_Flag==0)Find();
							break;
							}
							case 1:
								if(Spin_succeed_flag == 1)
								{
							 						 
									Stop_Flag = 0;	
								  while(Stop_Flag==0)Find();    
									back2_count++;
								}
							break;					 
							case 2:
								 if(Stop_Flag ==1)
								 {
									back2_count++;
									 spin_Turn(right_90);   //此时Spin_succeed_flag== 0 ,完成后自动置1
								 }
							break;
							case 3:
								 if(Spin_succeed_flag == 1)
								 {
							 
									 Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();
									 }
									 spin_Turn(straight);
									 back2_count++;
									 Stop_Flag = 1;	
								 }
							break;							 
							case	4:
								if(Stop_Flag ==1)
								{
									Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();
									 }
									 back2_count++;
								}
							break;
								case	5:
								{
									
									  Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();
									 }
									 spin_Turn(fianl);
									 back2_count++;
								}
							break;
								case	6:
								{
									  Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();
									 }
									  spin_Turn(fianl);
									back2_count++;
								}
							break;
							 case	7:
								{
									spin_Turn(back_180);
									back2_count++;
								}
								break;
								 case 8:
								{
									if(Spin_succeed_flag == 1)
								{
								Do_count = 0;
								Do2_count = 0;
								}
								}
								break;	
						}
						back2_count=0;
}

void error_back3(void)
{
	Flag_Stop=0;
	switch(back3_count)
						{
							case 0:{		 
							spin_Turn(back_180);
							back3_count++;
								while(Stop_Flag==0)Find();
							break;
							}
							case 1:
								if(Spin_succeed_flag == 1)
								{
							 						 
									Stop_Flag = 0;	
								  while(Stop_Flag==0)Find();    
									back3_count++;
								}
							break;					 
							case 2:
								 if(Stop_Flag ==1)
								 {
									 back3_count++;
									 spin_Turn(left_90);   //此时Spin_succeed_flag== 0 ,完成后自动置1
								 }
							break;
							case 3:
								 if(Spin_succeed_flag == 1)
								 {
							 
									 Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();
									 }
									 spin_Turn(straight);
									 back3_count++;
									 Stop_Flag = 1;	
								 }
							break;							 
							case	4:
								if(Stop_Flag ==1)
								{
									Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();
									 }
									 back3_count++;
								}
							break;
								case	5:
								{
									
									  Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();
									 }
									 spin_Turn(fianl);
									 back3_count++;
								}
							break;
								case	6:
								{
									  Stop_Flag = 0;	
									 while(Stop_Flag==0)
									 {
										 Find();
									 }
									  spin_Turn(fianl);
									 back3_count++;
								}
							break;
							 case	7:
								{
									spin_Turn(back_180);
									back3_count++;
								}
								break;
								 case 8:
								{
									if(Spin_succeed_flag == 1)
								{
								Do_count = 0;
								Do2_count = 0;
								}
								}
								break;	
						}
						back3_count=0;
}

