#ifndef __OLED_H
#define __OLED_H			  	 
#include "sys.h"

//-----------------OLED端口定义---------------- 
#define OLED_RST_Clr() PCout(15)=0   //RST
#define OLED_RST_Set() PCout(15)=1   //RST

#define OLED_RS_Clr() PCout(0)=0    //DC
#define OLED_RS_Set() PCout(0)=1    //DC

#define OLED_SCLK_Clr()  PCout(13)=0  //SCL
#define OLED_SCLK_Set()  PCout(13)=1   //SCL

#define OLED_SDIN_Clr()  PCout(14)=0   //SDA
#define OLED_SDIN_Set()  PCout(14)=1   //SDA

#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据
//OLED控制用函数
void OLED_WR_Byte(u8 dat,u8 cmd);	         //向OLED写入一个字节
void OLED_Display_On(void);                //开启OLED显示
void OLED_Display_Off(void);               //关闭OLED显示
void OLED_Refresh_Gram(void);		   				 //刷新显示		    
void OLED_Init(void);                      //初始化OLED
void OLED_Clear(void);                     //OLED清屏，清空后整个屏幕都是黑色
void OLED_DrawPoint(u8 x,u8 y,u8 t);       //OLED画点
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode); //在指定的位置显示一个字符
void OLED_ShowNumber(u8 x,u8 y,u32 num,u8 len,u8 size); //在指定的位置显示数字
void OLED_ShowString(u8 x,u8 y,const u8 *p);	          //在指定的位置显示字符串
#endif     
	 
