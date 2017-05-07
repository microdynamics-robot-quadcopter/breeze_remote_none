#include "stm32f10x_it.h"
#include "stm32f10x_driver_delay.h"
#include "stm32f10x_driver_usart.h"

/**************************实现函数********************************************
*函数原型:		void delay_ms(u16 nms)
*功　　能:		毫秒级延时  延时nms  nms<=1864
*******************************************************************************/
void delay_ms(uint16_t nms)
{
    uint32_t t0=micros();
    while(micros() - t0 < nms * 1000);
}

/**************************实现函数********************************************
*函数原型:		void delay_us(u32 nus)
*功　　能:		微秒级延时  延时nus  nms<=1864
*******************************************************************************/
void delay_us(u32 nus)
{
    uint32_t t0=micros();
    while(micros() - t0 < nus);
}
