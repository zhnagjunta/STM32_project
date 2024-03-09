#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"

#define KEY PBin(14)   

void KEY_Init(void);          //按键初始化
u8 click_N_Double (u8 time);  //单击按键扫描和双击按键扫描
u8 click(void);               //单击按键扫描
u8 Long_Press(void);          //长按检测
u8  select(void);             //选择运行的模式
#endif 
