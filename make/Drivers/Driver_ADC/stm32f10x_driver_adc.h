#ifndef __STM32F10X_DRIVER_ADC_H__
#define __STM32F10X_DRIVER_ADC_H__

#include "stm32f10x.h"

#define ADC_CH1  		1  			//通道1
#define ADC_CH_TEMP  	16 			//温度传感器通道

void Adc_Init(void); 				//ADC通道初始化
u16  Get_Adc(u8 ch); 				//获得某个通道值
u16 Get_Adc_Average(u8 ch,u8 times);//得到某个通道10次采样的平均值
int Get_Temp(void);

#endif
