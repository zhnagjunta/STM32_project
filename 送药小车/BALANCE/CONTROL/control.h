#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"

//小车各模式定义
#define Pi 3.14159265

void Kinematic_Analysis(float velocity,float angle);
void TIM2_IRQHandler(void) ;
void Set_Pwm(int motor_a,int motor_b);
void Key(void);
void Xianfu_Pwm(void);
u8 Turn_Off( int voltage);
void Get_Angle(u8 way);
int myabs(int a);
int Incremental_PI_A (int Encoder,int Target);
int Incremental_PI_B (int Encoder,int Target);
void Get_RC(void);
void  Find_CCD_Zhongzhi(void);
extern uint16_t lukou_straight; 
extern u16 SendTime;
#endif
