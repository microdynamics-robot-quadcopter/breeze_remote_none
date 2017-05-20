#ifndef __STM32F10X_MODULE_COMM_LINK_H__
#define __STM32F10X_MODULE_COMM_LINK_H__

#include "stm32f10x.h"

#define MSP_ARM_IT	          5
#define MSP_DISARM_IT	      6
#define MSP_SET_4CON	      7
#define MSP_HOLD_ALT 	      10
#define MSP_STOP_HOLD_ALT     11
#define MSP_HEAD_FREE         12
#define MSP_STOP_HEAD_FREE	  13
#define MSP_AUTO_LAND_DISARM  9
#define MSP_ACC_CALI	      205

//对外接口函数
void CommUAVUpload(unsigned char cmd);
void CommLink_ReceiveDataFromUSART(u8 byte);
void CommLink_ProcessDataFromUSART(void);

#endif
