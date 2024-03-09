#ifndef __MOTO_H
#define __MOTO_H
#include "sys.h" 

#define PWMA   TIM1->CCR4  
#define AIN2   PAout(15)
#define AIN1   PBout(5)
#define BIN1   PBout(4)
#define BIN2   PAout(12)
#define PWMB   TIM1->CCR1

#define SERVO   TIM8->CCR1  //¶æ»úÒý½Å

void Motor_PWM_Init(u16 arr,u16 psc);
void Servo_PWM_Init(u16 arr,u16 psc);
void TIM1_UP_IRQHandler(void) ;
void Motor_Init(void);
#endif
