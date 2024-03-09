#include "control.h"		

#define T 0.156f
#define L 0.1445f
#define K 622.8f
u8 Flag_Target;
int Voltage_Temp,Voltage_Count,Voltage_All,sum;

/**************************************************************************
�������ܣ�С���˶���ѧģ��
��ڲ������ٶȺ�ת��
����  ֵ����
**************************************************************************/
void Kinematic_Analysis(float velocity,float turn)
{
	    Target_A=(velocity+turn); 
		Target_B=(velocity-turn);      //���ֲ���
}
/**************************************************************************
�������ܣ����еĿ��ƴ��붼��������
          5ms��ʱ�ж��ɶ�ʱ��5����
          �ϸ�֤���������ݴ����ʱ��ͬ��				 
**************************************************************************/
uint16_t lukou_straight; 

char distance_oled[10]; 
void TIM5_IRQHandler(void)  
{    
	if(TIM_GetFlagStatus(TIM5,TIM_FLAG_Update)==SET)//5ms��ʱ�ж�
	{   
		TIM_ClearITPendingBit(TIM5,TIM_IT_Update);                             //===�����ʱ��1�жϱ�־λ	      
		SendTime++;
		if(delay_flag==1)
		{
			if(++delay_50==10)	 delay_50=0,delay_flag=0;                      //���������ṩ50ms�ľ�׼��ʱ
		}
		if(lukou_straight>0) lukou_straight++;

			Flag_Target=!Flag_Target;
			 if(Flag_Target==0) 
			 { 
			Encoder_Left=-Read_Encoder(2);                                       //===��ȡ��������ֵ							 //Ϊ�˱�֤M�����ٵ�ʱ���׼�����ȶ�ȡ����������
			Encoder_Right=-Read_Encoder(4);                                      //===��ȡ��������ֵ
			Kinematic_Analysis(Velocity,Turn);     															//С���˶�ѧ����   
			if(Turn_Off(Voltage)==0)                              							 //===����������쳣
			{
				Motor_A=Incremental_PI_A(Encoder_Left,Target_A);                   //===�ٶȱջ����Ƽ�����A����PWM
				Motor_B=Incremental_PI_B(Encoder_Right,Target_B);                  //===�ٶȱջ����Ƽ�����B����PWM 
				Xianfu_Pwm();                                                      //===PWM�޷�
				Set_Pwm(Motor_B,Motor_A);                                   	 //===��ֵ��PWM�Ĵ���  
			}
			else
				Set_Pwm(0,0);                                 						 //===��ֵ��PWM�Ĵ���  	
			Voltage_Temp=Get_battery_volt();		                                 //=====��ȡ��ص�ѹ		
			Voltage_Count++;                                                     //=====ƽ��ֵ������
			Voltage_All+=Voltage_Temp;                                           //=====��β����ۻ�
			if(Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//=====��ƽ��ֵ	
			}			
//			sprintf(distance_oled,"%.1f",Get_distance_volt());
//			OLED_ShowString(80,20,"        ");
//			OLED_ShowString(80,20,(u8 *)distance_oled);			 
	}
} 
/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ���������PWM������PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b)
{
	    if(ELE_Line_Patrol_Mode==1||CCD_Line_Patrol_Mode==1)//Ѳ��ģʽ�£�ֻ��������ת
		{
		    if(motor_a>0)motor_a=0;
		    if(motor_b>0)motor_b=0;
		
		 }
    	if(motor_a>0)	AIN2=0,			AIN1=1;
		else 	        AIN2=1,			AIN1=0;
		PWMA=myabs(motor_a);
		if(motor_b>0)	BIN1=0,			BIN2=1;
		else            BIN1=1,			BIN2=0;
		PWMB=myabs(motor_b);		
}

/**************************************************************************
�������ܣ�����PWM��ֵ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Xianfu_Pwm(void)
{	
	int Amplitude=6900;    //===PWM������7200 ������6900
    if(Motor_A<-Amplitude) Motor_A=-Amplitude;	
	if(Motor_A>Amplitude)  Motor_A=Amplitude;	
	if(Motor_B<-Amplitude) Motor_B=-Amplitude;	
	if(Motor_B>Amplitude)  Motor_B=Amplitude;
    #if Akm_Car	
	  if(Servo<(SERVO_INIT-500))     Servo=SERVO_INIT-500;	  //����޷�
	  if(Servo>(SERVO_INIT+500))     Servo=SERVO_INIT+500;		  //����޷�
	#endif
}
/**************************************************************************
�������ܣ������޸�С������״̬ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Key(void)
{	
	u8 tmp,tmp2;
	tmp=click(); 
	if(tmp==1)Flag_Stop=!Flag_Stop;//��������С������ͣ
	//if(tmp==2)Flag_Show=!Flag_Show;//˫������С������ʾ״̬
	tmp2=Long_Press();          
  if(tmp2==1)Flag_Show=!Flag_Show;//����С������ʾ״̬
}

/**************************************************************************
�������ܣ��쳣�رյ��
��ڲ�������ѹ
����  ֵ��1���쳣  0������
**************************************************************************/
u8 Turn_Off( int voltage)
{
	  u8 temp;
	if(voltage<600||Flag_Stop==1)//��ص�ѹ����10.5V�رյ��
	{	                                                
      temp=1;    
      }
	else
      temp=0;
    return temp;			
}
/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
�������ܣ�����PI������
��ڲ���������������ֵ��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //����ʽPI������
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}
int Incremental_PI_B (int Encoder,int Target)
{ 	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //����ʽPI������
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}
/**************************************************************************
�������ܣ�ң��
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_RC(void)
{
	static float Bias,Last_Bias;
	if(BLUETOOTH_Mode==1&&APP_Flag==1)//��������
	{
		#if Akm_Car
		{
	    	if(Flag_Direction==0) Velocity=0,Turn=0;   //ֹͣ
	    	else if(Flag_Direction==0x41) Velocity=Bluetooth_Velocity,Turn=0;  //ǰ��
	    	else if(Flag_Direction==0x42) Velocity=Bluetooth_Velocity,Turn=Pi/5;  //��ǰ
	    	else if(Flag_Direction==0x43) Velocity=0,Turn=0;   //�������
	    	else if(Flag_Direction==0x44) Velocity=-Bluetooth_Velocity,Turn=Pi/5;  // �Һ�
	    	else if(Flag_Direction==0x45) Velocity=-Bluetooth_Velocity,Turn=0;    //����
	    	else if(Flag_Direction==0x46) Velocity=-Bluetooth_Velocity,Turn=-Pi/5;  //���
	    	else if(Flag_Direction==0x47) Velocity=0,Turn=0;                       //�������
	    	else if(Flag_Direction==0x48) Velocity=Bluetooth_Velocity,Turn=-Pi/5;  //��ǰ
		}
		#elif Diff_Car
		{
			if(Flag_Direction==0) Velocity=0,Turn=0;   //ֹͣ
	    	else if(Flag_Direction==0x41) Velocity=Bluetooth_Velocity,Turn=0;  //ǰ��
	    	else if(Flag_Direction==0x42) Velocity=Bluetooth_Velocity,Turn=20;  //��ǰ
	    	else if(Flag_Direction==0x43) Velocity=0,Turn=20;   //�������
	    	else if(Flag_Direction==0x44) Velocity=-Bluetooth_Velocity,Turn=-20;  // �Һ�
	    	else if(Flag_Direction==0x45) Velocity=-Bluetooth_Velocity,Turn=0;    //����
	    	else if(Flag_Direction==0x46) Velocity=-Bluetooth_Velocity,Turn=20;  //���
	    	else if(Flag_Direction==0x47) Velocity=0,Turn=-20;                       //�������
	    	else if(Flag_Direction==0x48) Velocity=Bluetooth_Velocity,Turn=-20;  //��ǰ
		}
		#elif Small_Tank_Car
		{
		  if(Flag_Direction==0) Velocity=0,Turn=0;   //ֹͣ
		else if(Flag_Direction==0x41) Velocity=Bluetooth_Velocity,Turn=0;  //ǰ��
		else if(Flag_Direction==0x42) Velocity=Bluetooth_Velocity,Turn=20;  //��ǰ
		else if(Flag_Direction==0x43) Velocity=0,Turn=20;   //�������
		else if(Flag_Direction==0x44) Velocity=-Bluetooth_Velocity,Turn=-20;  // �Һ�
		else if(Flag_Direction==0x45) Velocity=-Bluetooth_Velocity,Turn=0;    //����
		else if(Flag_Direction==0x46) Velocity=-Bluetooth_Velocity,Turn=20;  //���
		else if(Flag_Direction==0x47) Velocity=0,Turn=-20;                       //�������
		else if(Flag_Direction==0x48) Velocity=Bluetooth_Velocity,Turn=-20;  //��ǰ
		}
		#endif
		
	}
   else if(CCD_Line_Patrol_Mode)//CCDѲ��
   {
	    Velocity=25;	   //CCDѲ��ģʽ���ٶ�
	    Bias=CCD_Zhongzhi-64;   //��ȡƫ��
	    #if Akm_Car
	        Turn=Bias*0.010+(Bias-Last_Bias)*0.085; //PD����
	    #elif Diff_Car
	        Turn=Bias*0.1+(Bias-Last_Bias)*1; //PD����
	    #elif Small_Tank_Car	  
	        Turn=Bias*0.4+(Bias-Last_Bias)*2; //PD����	 
	    #endif
	     Last_Bias=Bias;   //������һ�ε�ƫ��
	
   }
	else if(ELE_Line_Patrol_Mode)//���Ѳ��
  {  
	
	  Velocity=25;	  //���Ѳ��ģʽ�µ��ٶ�
	  Bias=100-Sensor;  //��ȡƫ��
	  #if Akm_Car
	    Turn=myabs(Bias)*Bias*0.00005+Bias*0.0005+(Bias-Last_Bias)*0.005; //	 
	  #elif Diff_Car
		Turn=myabs(Bias)*Bias*0.001+Bias*0.002+(Bias-Last_Bias)*1.0; //
	  #elif Small_Tank_Car
		Turn=myabs(Bias)*Bias*0.002+Bias*0.006+(Bias-Last_Bias)*2;
	  #endif
		  Last_Bias=Bias;   //��һ�ε�ƫ��
	  }
}
/**************************************************************************
�������ܣ�����CCDȡ��ֵ
��ڲ�������
����  ֵ����
**************************************************************************/
void  Find_CCD_Zhongzhi(void)
{ 
	 static u16 i,j,Left,Right;
	 static u16 value1_max,value1_min;
	
	   value1_max=ADV[0];  //��̬��ֵ�㷨����ȡ������Сֵ
     for(i=5;i<123;i++)   //���߸�ȥ��5����
     {
        if(value1_max<=ADV[i])
        value1_max=ADV[i];
     }
	   value1_min=ADV[0];  //��Сֵ
     for(i=5;i<123;i++) 
     {
        if(value1_min>=ADV[i])
        value1_min=ADV[i];
     }
   CCD_Yuzhi=(value1_max+value1_min)/2;	  //���������������ȡ����ֵ
	 for(i = 5;i<118; i++)   //Ѱ�����������
	{
		if(ADV[i]>CCD_Yuzhi&&ADV[i+1]>CCD_Yuzhi&&ADV[i+2]>CCD_Yuzhi&&ADV[i+3]<CCD_Yuzhi&&ADV[i+4]<CCD_Yuzhi&&ADV[i+5]<CCD_Yuzhi)
		{	
			Left=i;
			break;	
		}
	}
	 for(j = 118;j>5; j--)//Ѱ���ұ�������
  {
		if(ADV[j]<CCD_Yuzhi&&ADV[j+1]<CCD_Yuzhi&&ADV[j+2]<CCD_Yuzhi&&ADV[j+3]>CCD_Yuzhi&&ADV[j+4]>CCD_Yuzhi&&ADV[j+5]>CCD_Yuzhi)
		{	
		  Right=j;
		  break;	
		}
  }
	CCD_Zhongzhi=(Right+Left)/2;//��������λ��
//	if(myabs(CCD_Zhongzhi-Last_CCD_Zhongzhi)>90)   //�������ߵ�ƫ����̫��
//	CCD_Zhongzhi=Last_CCD_Zhongzhi;    //��ȡ��һ�ε�ֵ
//	Last_CCD_Zhongzhi=CCD_Zhongzhi;  //������һ�ε�ƫ��
}
