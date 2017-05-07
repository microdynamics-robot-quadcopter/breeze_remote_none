#ifndef __STM32F10X_DRIVER_USART_H__
#define __STM32F10X_DRIVER_USART_H__

#include "stm32f10x.h"

// USART Receiver buffer
#define RX_BUFFER_SIZE   128
#define TX_BUFFER_SIZE   128

typedef struct
{
  uint16_t volatile Wd_Indx;
  uint16_t volatile Rd_Indx;
  uint16_t Mask;
  uint8_t *pbuf;
} UartBuf;

extern UartBuf UartTxbuf;
extern UartBuf UartRxbuf;
extern unsigned char rx_buffer[RX_BUFFER_SIZE];
extern unsigned char tx_buffer[TX_BUFFER_SIZE];
extern UartBuf UartTxbuf;//环形发送结构体
extern UartBuf UartRxbuf;//环形接收结构体
extern uint8_t flyLogF;

void UartBufClear(UartBuf *Ringbuf);
void UART1NVIC_Configuration(void);
void UART1_init(u32 pclk2,u32 bound);
void UART1_Put_Char(unsigned char DataToSend);
uint8_t Uart1_Put_Char(unsigned char DataToSend);
extern uint8_t UartBuf_RD(UartBuf *Ringbuf);
extern uint16_t UartBuf_Cnt(UartBuf *Ringbuf);
extern void UartBuf_WD(UartBuf *Ringbuf,uint8_t DataIn);
void UartSendBuffer(uint8_t *dat, uint8_t len);

#endif
