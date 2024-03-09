#ifndef __USRAT2_H
#define __USRAT2_H 
#include "sys.h"	  	

extern  uint8_t  Num, LoR, Finded_flag, FindTask;
extern u8 RoomNum, TargetNum ,TASK;
extern char TargetRoom;
extern int GetOpenmvDataCount;
void  Openmv_Receive_Data(uint8_t data);
void SetTargetRoom(void);
void SendDataToOpenmv(void);

extern u8 Usart2_Receive;
void uart2_init(u32 bound);
void USART2_IRQHandler(void);
#endif

