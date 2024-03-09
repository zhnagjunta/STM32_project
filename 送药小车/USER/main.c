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

u8 Mode=0,BLUETOOTH_Mode=1,ELE_Line_Patrol_Mode=0,CCD_Line_Patrol_Mode=0;      //С����ģʽ,����ң�ء�CCDѲ�ߡ�ELEѲ��
u8 Flag_Way=0,Flag_Show=0,Flag_Stop=1,Flag_Next;       //ֹͣ��־λ�� ��ʾ��־λ Ĭ��ֹͣ ��ʾ��
int Encoder_Left,Encoder_Right;                        //���ұ��������������
int Flag_Direction;  
float Velocity,Velocity_Set,Angle_Set,Turn;
int Motor_A,Motor_B,Servo,Target_A,Target_B;           //�������������           
int Voltage;                                           //��ص�ѹ������صı���
float Show_Data_Mb;                                    //ȫ����ʾ������������ʾ��Ҫ�鿴������
u8 delay_50,delay_flag; 							   //��ʱ����
float Velocity_KP=12,Velocity_KI=12;	       //�ٶȿ���PID����
u16 ADV[128]={0};              
u8 Bluetooth_Velocity=30,APP_RX,APP_Flag;                        //����ң���ٶȺ�APP���յ�����
u8 CCD_Zhongzhi,CCD_Yuzhi,PID_Send,Flash_Send;          //����CCD FLASH���
int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor;      //���Ѳ�����
u16 PID_Parameter[10],Flash_Parameter[10];              //Flash������� 

void error_back1(void);
void error_back2(void);
void error_back3(void);
void Find(void);
void spin_Turn(spin_dir_t zhuanxiang);
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //�����жϷ���Ϊ2
	delay_init(72);                 //=====��ʱ��ʼ��
	JTAG_Set(JTAG_SWD_DISABLE);     //=====�ر�JTAG�ӿ�
	//JTAG_Set(SWD_ENABLE);         //=====��SWD�ӿ� �������������SWD�ӿڵ���
	LED_Init();                     //=====��ʼ���� LED ���ӵ�Ӳ���ӿ�
	KEY_Init();
	OLED_Init();                    //=====OLED��ʼ��
	Encoder_Init_TIM2();            //=====��ʼ����������TIM2�ı������ӿ�ģʽ�� 
	Encoder_Init_TIM4();            //=====��ʼ����������TIM3�ı������ӿ�ģʽ��
	EXTIX_Init();                   //=====������ʼ��(�ⲿ�жϵ���ʽ)	
	Adc_Init();                     //=====��ص�ѹ����adc��ʼ��
	delay_ms(300);                  //=====��ʱ����
	uart_init(115200);                //=====��ʼ������1
	Motor_Init();
	Motor_PWM_Init(7199,0);  	    //=====��ʼ��PWM 10KHZ������������� 
//ccd_Init();
	uart2_init(115200); 				//=====����2��ʼ�� ����
	uart3_init(115200);
	Flash_Read();	                //=====��ȡPID����
	Timer5_Init(49,7199);
	Load_Init();
	while(1)
	{     
		if(SendTime >= 4)
		{
			SendTime = 0;
			SendDataToOpenmv();   //����̫�죬����ᳬ��openmv�Ĵ��ڽ������ݻ�����
			LoadOrNot();
			oled_show();          //===��ʾ����
		}
		if(Load_flag == 1&&TASK == 1)   
		{
			SetTargetRoom();
		}
   		if(TASK == 2)    
	 	{
			oled_show();          //===��ʾ����
			if(Load_flag == 1)    //����ҩ��
			{
				if(TargetRoom == 'A')   //���λ�����ΪA��������Ϊ1
					{
						switch(Do_count)
						{
							case 0: 	
							{								
								Stop_Flag = 0;	
								 while(Stop_Flag==0)Find();     //��ʱ Stop_Flag ==0  ��ɺ��Զ���1        //���˲�����Ԥ��openmvʶ���λ��
									Do_count++;	
							}
							break;					 
							case 1:
								 if(Stop_Flag ==1)          //ֱ�С�ת�亯������ʹ��ʱ�����Բ��ֶ�����־λ���� ����·��
								 {
									 
									 spin_Turn(left_90);   //��ʱSpin_succeed_flag== 0 ,��ɺ��Զ���1 ��ת
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
								if(Spin_succeed_flag ==1)//���յ�
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
									 spin_Turn(right_90);   //��ʱSpin_succeed_flag== 0 ,��ɺ��Զ���1
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
				//�ж˲�����Զ�˲�����Ҫ�����ض���־λ					
				else      //TargetRoom == 'C' || TargetRoom == 'D' || TargetRoom == 'E' || TargetRoom == 'F' || TargetRoom == 'G' || TargetRoom == 'H'
					{
						switch(Do_count)//ֱ�ߵ���һ��·��
						{
							case 0: 
							{
								Stop_Flag = 0;	
								while(Stop_Flag==0)
									 {
										 Find();
									 }
								Do_count++;
								SendDataToOpenmv();  //������ʶ��
							}
							break;							
							case 1:
								if(Stop_Flag ==1) //������һ��·��
								 {
									 
									 spin_Turn(straight);   //��ʱSpin_succeed_flag== 0 ,��ɺ��Զ���1
									 Do_count++;
									 SendDataToOpenmv(); //��ģ��ƥ������
								 }
							case 2:
								if(Spin_succeed_flag == 1)//��һ��·�����ڶ���·��ֱ��
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
							case 3:       //�ж��Ƿ�ʶ�����֣��Ҹ����ֵ�λ��ƫ����ƫ��  �ڶ���·��
								if(Stop_Flag == 1)
								{
											if(RoomNum == TargetNum && LoR == 1)  //ʶ����Ŀ������, ����λ��ƫ��  
											{
												Do_count  = 8;  //����ֱ����ת��ֱ�е������ſڵĺ���
												TargetRoom = 'C';
											}
											else if(RoomNum == TargetNum && LoR == 2 )  //ʶ��Ŀ������, ����λ��ƫ��
											{
												Do_count  = 8;        //����ֱ����ת��ֱ�е������ſڵĺ���
												TargetRoom = 'D';
											}	
											if(LoR==0)  //ûʶ�𵽾�ֱ�ߵ�������·��ֹͣ
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
							case 4://������·��׼��ת��
								if(Stop_Flag == 1)
								{
									if(RoomNum == TargetNum && LoR == 1)  //ʶ����Ŀ������, ����λ��ƫ��  
											{
												Do_count++; 
												TargetRoom = 'E';      //�ȼٶ���E
												SendDataToOpenmv(); //�ر�ģ��ƥ������
											}
									else if(RoomNum == TargetNum && LoR == 2 )  //ʶ��Ŀ������, ����λ��ƫ��
											{
												Do_count++;
												TargetRoom = 'G';    //�ȼٶ���G
												SendDataToOpenmv(); //�ر�ģ��ƥ������	
											}	
									if(LoR==0){
												 error_back1();
											 }
								}
							break;				
							case 5://ת��
								if(Stop_Flag == 1)
								{
									Do_count++;
									if(TargetRoom == 'E')
									{
										spin_Turn(left_90);
										LoR=0;//���Ԥת�䷽��
										SendDataToOpenmv(); //��ģ��ƥ������
									}
									else if(TargetRoom == 'G')
									{
										spin_Turn(right_90);
										LoR=0;//���Ԥת�䷽��
										SendDataToOpenmv(); //��ģ��ƥ������
									}
								}
							break;		
								case 6: //ֱ�ߵ����ĸ�·��
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
							case 7: //���ĸ�·��׼��ת��
								if(Stop_Flag == 1)
								{
									if(TargetRoom == 'E')
										{
												if(RoomNum == TargetNum && LoR == 1)    //ʶ��RoomNum, ����λ��ƫ��    
											 {
												 Do_count ++;  
												 TargetRoom = 'E';
												 SendDataToOpenmv(); //�ر�ģ��ƥ������	
											 }										 
											 if(RoomNum == TargetNum && LoR == 2)   //һ��ʱ����ʶ��RoomNum, ����λ��ƫ��
											 {
												 Do_count++;  
												 TargetRoom = 'F';
												 SendDataToOpenmv(); //�ر�ģ��ƥ������	
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
												if(RoomNum == TargetNum && LoR == 1)    //һ��ʱ����ʶ��RoomNum, ����λ��ƫ��    
											 {
												 Do_count++; 
												 TargetRoom = 'G';
												 SendDataToOpenmv(); //�ر�ģ��ƥ������	
											 }										 											 
											 if(RoomNum == TargetNum && LoR == 2)   //һ��ʱ����ʶ��RoomNum, ����λ��ƫ��
											 {
												 Do_count++;  
												 TargetRoom = 'H';
												 SendDataToOpenmv(); //�ر�ģ��ƥ������	
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
							case 8://���·��ת��
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
							case 9: //ֱ�ߵ��յ�
								if(Spin_succeed_flag == 1)
								{
									Do_count++;
									Stop_Flag = 0;	
									while(Stop_Flag==0)Find();
									spin_Turn(fianl);
								}
							break;							
							case 10:
								if(Spin_succeed_flag == 1) //����ҩ
								{		
										Velocity=0;
									Turn=0;
									Flag_Stop=1;
								}
							break;								
						}
					}
			}
			//��ҩ��Ϸ��� 
			else if(Load_flag == 2)       // ȫ�����䶼�ɷ����ˣ�����Զ�˵�����·���ǿ�����ʱ���������⡣
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
								if(Spin_succeed_flag == 1) //ת��ɹ�
								{					
									Stop_Flag = 0;	
									while(Stop_Flag==0)Find();
									Do2_count++;	
								}
							break;
							case 2:
								 if(Stop_Flag ==1)
								 {
									 spin_Turn(right_90 );   //��ʱSpin_succeed_flag== 0 ,��ɺ��Զ���1
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
					else if(TargetRoom == 'C')   //�ж˲���   ����������˲����Ĳ��ֻ�������ֱ�ߵ�ʱ���߶��˼�ʮcm����
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
									spin_Turn(right_90 );   //��ʱSpin_succeed_flag== 0 ,��ɺ��Զ���1
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
									 
									 spin_Turn(left_90);   //��ʱSpin_succeed_flag== 0 ,��ɺ��Զ���1
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
else if(TargetRoom == 'E')   //Զ�˲���   �ĸ�ֻ��������·�ڵ�ת��ͬ����
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
									 spin_Turn(right_90);   //��ʱSpin_succeed_flag== 0 ,��ɺ��Զ���1
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
									 spin_Turn(left_90);   //��ʱSpin_succeed_flag== 0 ,��ɺ��Զ���1
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
									 spin_Turn(right_90);   //��ʱSpin_succeed_flag== 0 ,��ɺ��Զ���1
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
									 spin_Turn(left_90);   //��ʱSpin_succeed_flag== 0 ,��ɺ��Զ���1
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
		if(Di[0]==1)// �����Ѱ���� 45
	{
		Velocity=speed;
			Turn=-4;
	}
	else if(Di[7]==1)// ���ұ�Ѱ���� 55
	{
		Velocity=speed;
			Turn=4;
	}
	else if(Di[1]==1)// �����Ѱ����
	{
		Velocity=speed;
			Turn=-3;
	}
	else if(Di[6]==1)// ���ұ�Ѱ����
	{
		Velocity=speed;
			Turn=3;
	}
	else if(Di[2]==1)// ���Ѱ����
	{
		Velocity=speed;
			Turn=-2;
	}
	else if(Di[5]==1)// �ұ�Ѱ����
	{
		Velocity=speed;
			Turn=2;
	}
	else if((Di[0] == 0)&&(Di[1] == 0)&&(Di[2] == 0)&&(Di[3] == 1)&&(Di[4] == 0)&&(Di[5] == 0)&&(Di[6] == 0)&&(Di[7] == 0))// �����м䣬ǰ��
	{
		Velocity=speed;
			Turn=-1;
	}
	else if((Di[0] == 0)&&(Di[1] == 0)&&(Di[2] == 0)&&(Di[3] == 0)&&(Di[4] == 1)&&(Di[5] == 0)&&(Di[6] == 0)&&(Di[7] == 0))// �����м䣬ǰ��
	{
		Velocity=speed;
			Turn=1;
	}
	else if((Di[0] == 0)&&(Di[1] == 0)&&(Di[2] == 0)&&(Di[3] == 1)&&(Di[4] == 1)&&(Di[5] == 0)&&(Di[6] == 0)&&(Di[7] == 0))// �����м䣬ǰ��
	{
		Velocity=speed;
			Turn=0;
	}
	if(lukou==1) 
	{
		lukoutime++;
	}
	else lukoutime=0;
	if(lukoutime>5) Stop_Flag=1; //����·��ֹͣ
}

/******ת�����ʱ  Spin_start_flag == 0 && Spin_succeed_flag == 1  ********/

/*ת������ת90����ת90����ת180���������*/

void spin_Turn(spin_dir_t zhuanxiang)   //����С�����־�(mm) 175mm     //����Ҫ�ܾ�׼��ת���ֱ����ֱ��ʱ��Ѳ�ߺ�����������ͺ�
{
	  Stop_Flag = 0;   //ִ��ת��ʱ����ֱ����ɵı�־λ����. �������һ����ֱ�У������ת�䣬������ҵ��������ֶ���λ
	  
    Spin_start_flag = 1;   
	  Spin_succeed_flag = 0;  
		
	
	  /**************Ҫ��ת����ܻص����ϣ����Ը������ת��ϵ��**************/
		if(zhuanxiang == left_90)  //openmvʶ����Ҫ����߲�����
		{
			Velocity=20;
			Turn=0;
			delay_ms(800);//Խ��ʱ���ǰ��Խ��
			Velocity=0;
			Turn=-20;
			delay_ms(470);//Խ��ʱ���תԽ��
			Velocity=0;
			Turn=0;
			delay_ms(200);
		}
		else if(zhuanxiang == right_90)  //openmvʶ����Ҫ���ұ߲�����
		{
			Velocity=20;
			Turn=0;
			delay_ms(800);//Խ��ʱ���ǰ��Խ��
			Velocity=0;
			Turn=20;
			delay_ms(470);//Խ��ʱ���תԽ��
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
									 spin_Turn(right_90);   //��ʱSpin_succeed_flag== 0 ,��ɺ��Զ���1
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
									 spin_Turn(left_90);   //��ʱSpin_succeed_flag== 0 ,��ɺ��Զ���1
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

