#ifndef __LED_H
#define __LED_H
#include "sys.h"

//LED �˿ڶ���
#define LED PBout(13) // PB13
void LED_Init(void);  //��ʼ��
void Led_Flash(u16 time);


void LoadOrNot(void);
void Load_Init(void);
extern u8 Load_flag;
#endif
