#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"
#define Battery_Ch 9
void Dly_us(void);
 void RD_TSL(void) ;
void Adc_Init(void);
u16 Get_Adc(u8 ch);
int Get_battery_volt(void);  
float Get_distance_volt(void);   
void CCD(void);   
void  ccd_Init(void);
void  ele_Init(void);

#endif 


