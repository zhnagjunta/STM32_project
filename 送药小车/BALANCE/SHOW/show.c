#include "show.h"

unsigned char i;          //计数变量
unsigned char Send_Count; //串口需要发送的数据个数
float Vol;

/**************************************************************************
函数功能：OLED显示
入口参数：无
返回  值：无
**************************************************************************/
void oled_show(void)
{
	OLED_ShowString(00,0,"Target");
	OLED_ShowChar(65,0,TargetRoom,12,1);
	OLED_ShowString(00,10,"LoR");
	OLED_ShowNumber(30,10, LoR,3,12);   
	OLED_ShowNumber(100,0,TASK,3,12);					
	
	OLED_ShowString(00,20,"Velocity");     
	if( Velocity<0)		OLED_ShowString(80,20,"-"),
	OLED_ShowNumber(95,20,-Velocity,5,12);
	else                 	OLED_ShowString(80,20,"+"),
	OLED_ShowNumber(95,20, Velocity,5,12);

	OLED_ShowString(00,30,"Turn");
	if(Turn<0)	  OLED_ShowString(80,30,"-"),
	OLED_ShowNumber(95,30,-Turn,5,12);
	else               		OLED_ShowString(80,30,"+"),
	OLED_ShowNumber(95,30,Turn,5,12);	


	OLED_ShowString(00,40,"VOLTAGE");
	OLED_ShowString(68,40,".");
	OLED_ShowString(90,40,"V");
	OLED_ShowNumber(55,40,Voltage/100,2,12);
	OLED_ShowNumber(78,40,Voltage%100,2,12);
	if(Voltage%100<10) 	OLED_ShowNumber(72,40,0,2,12);

	if(Flag_Stop==0)
	OLED_ShowString(103,40,"O-N");
	if(Flag_Stop==1)
	OLED_ShowString(103,40,"OFF");


	OLED_ShowString(00,50," yao-");
	if(Load_flag == 1)              OLED_ShowString(40,50,"go  ");
	else if(Load_flag==2)          OLED_ShowString(40,50,"back");

	OLED_Refresh_Gram();	
	}
/**************************************************************************
函数功能：向APP发送数据
入口参数：无
返回  值：无
作    者：平衡小车之家
**************************************************************************/
void APP_Show(void)
{    
	  static u8 flag;
	  int app_2,app_3,app_4;
		app_4=(Voltage-740)*2/3;		if(app_4<0)app_4=0;if(app_4>100)app_4=100;   //对电压数据进行处理
		app_3=Encoder_Right*1.1; if(app_3<0)app_3=-app_3;			                   //对编码器数据就行数据处理便于图形化
		app_2=Encoder_Left*1.1;  if(app_2<0)app_2=-app_2;
	  flag=!flag;
		if(PID_Send==1)//发送PID参数
	{
		printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$",(int)Bluetooth_Velocity,(int)Velocity_KP,(int)Velocity_KI,0,0,0,0,0,0);//打印到APP上面	
		PID_Send=0;	
	}	
  else  if(flag==0)// 
   printf("{A%d:%d:%d:%d}$",(u8)app_2,(u8)app_3,app_4,(int)((Servo-SERVO_INIT)*0.16)); //打印到APP上面
	 else
	 printf("{B%d:%d:%d:%d}$",(int)Servo,(int)Voltage,Encoder_Left,Encoder_Right);//打印到APP上面 显示波形
}
/**************************************************************************
函数功能：虚拟示波器往上位机发送数据 关闭显示屏
入口参数：无
返回  值：无
作    者：平衡小车之家
**************************************************************************/
void DataScope(void)
{   
		DataScope_Get_Channel_Data( Servo-SERVO_INIT, 1 );       //显示角度 单位：度（°）
		DataScope_Get_Channel_Data( Encoder_Left, 2 );         //显示超声波测量的距离 单位：CM 
		DataScope_Get_Channel_Data( Encoder_Right, 3 );                 //显示电池电压 单位：V
//		DataScope_Get_Channel_Data( 0 , 4 );   
//		DataScope_Get_Channel_Data( 0, 5 ); //用您要显示的数据替换0就行了
//		DataScope_Get_Channel_Data( 0 , 6 );//用您要显示的数据替换0就行了
//		DataScope_Get_Channel_Data(0, 7 );
//		DataScope_Get_Channel_Data( 0, 8 ); 
//		DataScope_Get_Channel_Data(0, 9 );  
//		DataScope_Get_Channel_Data( 0 , 10);
		Send_Count = DataScope_Data_Generate(3);
		for( i = 0 ; i < Send_Count; i++) 
		{
		while((USART1->SR&0X40)==0);  
		USART1->DR = DataScope_OutPut_Buffer[i]; 
		}
}
void OLED_DrawPoint_Shu(u8 x,u8 y,u8 t)
{ 
	 u8 i=0;
  OLED_DrawPoint(x,y,t);
	OLED_DrawPoint(x,y,t);
	  for(i = 0;i<8; i++)
  {
      OLED_DrawPoint(x,y+i,t);
  }
}

void OLED_Show_CCD(void)
{ 
	 u8 i,t;
	 for(i = 0;i<128; i++)
  {
		if(ADV[i]<CCD_Yuzhi) t=1; else t=0;
		OLED_DrawPoint_Shu(i,0,t);
  }
}

//开机显示一次的内容
void oled_show_once(void)
{
   OLED_ShowString(0,00,"TO Press UserKey");
   OLED_ShowString(0,10,"TO Select Mode");
	 OLED_ShowString(0,20,"Current Mode Is");
	if(BLUETOOTH_Mode)         OLED_ShowString(50,30,"APP");
	if(CCD_Line_Patrol_Mode)				  OLED_ShowString(50,30,"CCD");
	if(ELE_Line_Patrol_Mode)				  OLED_ShowString(50,30,"ELE");
	
	OLED_ShowString(0,40,"D-Press User Key");
  OLED_ShowString(0,50,"TO End Selection");

		OLED_Refresh_Gram();	
	}
