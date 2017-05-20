#include <stdio.h>
#include <math.h>
#include "stm32f10x_it.h"
#include "stm32f10x_driver_usart.h"
#include "stm32f10x_module_comm_link.h"
#include "stm32f10x_algorithm_control.h"

#include <errno.h>
#include <sys/unistd.h>

//uart reicer flag
#define b_uart_head  0x80
#define b_rx_over    0x40

void USART_PutChar(u8 ch)
{
    USART_SendData(USART1, (u8)ch);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

u8 USART_GetChar(void)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
    return (u8)USART_ReceiveData(USART1);
}

int _read(int file, char *ptr, int len)
{
    int i;
    int num = 0;
    char ch;

    switch (file)
    {
        case STDIN_FILENO:
        {
            for (i = 0; i < len; i++)
            {
                ch = USART_GetChar();
                *ptr++ = ch;
                num++;
            }
            break;
        }
        default:
        {
            errno = EBADF;
            return -1;
        }
    }

    return num;
}

int _write(int file, char *ptr, int len)
{
    int i;

    switch (file)
    {
        case STDOUT_FILENO:
        {
            for (i = 0; i < len; i++)
            {
                USART_PutChar(*ptr++ & (u16)0x01FF);
            }
            break;
        }
        case STDERR_FILENO:
        {
            for (i = 0; i < len; i++)
            {
                USART_PutChar(*ptr++ & (u16)0x01FF);
            }
            break;
        }
        default:
        {
            errno = EBADF;
            return -1;
        }
    }

    return len;
}

/**************************实现函数********************************************
*函数原型:		void U1NVIC_Configuration(void)
*功　　能:		串口1中断配置
输入参数：无
输出参数：没有
*******************************************************************************/
void UART1NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    /* Enable the USART1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**************************实现函数********************************************
*函数原型:		void Initial_UART1(u32 baudrate)
*功　　能:		初始化UART1
输入参数：u32 baudrate   设置RS232串口的波特率
输出参数：没有
*******************************************************************************/
void UART1_init(u32 pclk2,u32 bound)
{
    float temp;
    u16 mantissa;
    u16 fraction;
    temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV
    mantissa=temp;				 //得到整数部分
    fraction=(temp-mantissa)*16; //得到小数部分
    mantissa<<=4;
    mantissa+=fraction;
    RCC->APB2ENR|=1<<2;   //使能PORTA口时钟
    RCC->APB2ENR|=1<<14;  //使能串口时钟
    GPIOA->CRH&=0XFFFFF00F;//IO状态设置
    GPIOA->CRH|=0X000008B0;//IO状态设置
    RCC->APB2RSTR|=1<<14;   //复位串口1
    RCC->APB2RSTR&=~(1<<14);//停止复位
    //波特率设置
    USART1->BRR=mantissa; // 波特率设置
    USART1->CR1|=0X200C;  //1位停止,无校验位.
    USART1->CR1|=1<<8;    //PE中断使能
    USART1->CR1|=1<<5;    //接收缓冲区非空中断使能

    UART1NVIC_Configuration();//中断配置

    UartTxbuf.Wd_Indx = 0;
    UartTxbuf.Rd_Indx = 0;
    UartTxbuf.Mask = TX_BUFFER_SIZE - 1;
    UartTxbuf.pbuf = &tx_buffer[0];

    UartRxbuf.Wd_Indx = 0;
    UartRxbuf.Rd_Indx = 0;
    UartRxbuf.Mask = RX_BUFFER_SIZE - 1;
    UartRxbuf.pbuf = &rx_buffer[0];

    // printf("MCU clock frequency:%dMHz \r\n",pclk2);
    // printf("UART 1 baud frequency:%d \r\n",bound);
}

/**************************实现函数********************************************
*函数原型:		void UART1_Put_Char(unsigned char DataToSend)
*功　　能:		RS232发送一个字节
输入参数：
    unsigned char DataToSend   要发送的字节数据
输出参数：没有
*******************************************************************************/
void UART1_Put_Char(unsigned char DataToSend)
{
    UartBuf_WD(&UartTxbuf,DataToSend);//将待发送数据放在环形缓冲数组中
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);  //启动发送中断开始啪啪啪发送缓冲中的数据
}

// uint8_t Uart1_Put_Char(unsigned char DataToSend)
// {
//   UartBuf_WD(&UartTxbuf,DataToSend);//将待发送数据放在环形缓冲数组中
//   USART_ITConfig(USART1, USART_IT_TXE, ENABLE);  //启动发送中断开始啪啪啪发送缓冲中的数据
// 	 return DataToSend;
// }

//环形 数组结构体实例化两个变量
UartBuf UartTxbuf;//环形发送结构体
UartBuf UartRxbuf;//环形接收结构体

unsigned char rx_buffer[RX_BUFFER_SIZE];
unsigned char tx_buffer[TX_BUFFER_SIZE];

//读取环形数据中的一个字节
uint8_t UartBuf_RD(UartBuf *Ringbuf)
{
    uint8_t temp;
    temp = Ringbuf->pbuf[Ringbuf->Rd_Indx & Ringbuf->Mask];//数据长度掩码很重要，这是决定数据环形的关键
    Ringbuf->Rd_Indx++;//读取完成一次，读指针加1，为下一次 读取做 准备
    return temp;
}

//将一个字节写入一个环形结构体中
void UartBuf_WD(UartBuf *Ringbuf,uint8_t DataIn)
{

    Ringbuf->pbuf[Ringbuf->Wd_Indx & Ringbuf->Mask] = DataIn;//数据长度掩码很重要，这是决定数据环形的关键
    Ringbuf->Wd_Indx++;//写完一次，写指针加1，为下一次写入做准备

}

//环形数据区的可用字节长度，当写指针写完一圈，追上了读指针
//那么证明数据写满了，此时应该增加缓冲区长度，或者缩短外围数据处理时间
uint16_t UartBuf_Cnt(UartBuf *Ringbuf)
{
    return (Ringbuf->Wd_Indx - Ringbuf->Rd_Indx) & Ringbuf->Mask;//数据长度掩码很重要，这是决定数据环形的关键
}

void UartBufClear(UartBuf *Ringbuf)
{
    Ringbuf->Rd_Indx=Ringbuf->Wd_Indx;
}

u8 UartSendBuffer(uint8_t *dat, uint8_t len)
{
    uint8_t i;

    for(i=0;i<len;i++)
    {
        UartBuf_WD(&UartTxbuf,*dat);
        dat++;
    }
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);  //启动发送中断开始啪啪啪发送缓冲中的数据

    return i;
}

volatile uint8_t Udatatmp;//串口接收临时数据字节


extern char IMUcalibratflag;
extern char Lockflag;
extern int  Throttle;
extern int  Roll;
extern int  Pitch;
extern int  Yaw;

#define RC_IMU_CALI  'u'
#define RC_ARM_LOCK  'o'
#define RC_THRUST_UP 'w'
#define RC_THRUST_DN 's'
#define RC_YAW_UP    'a'
#define RC_YAW_DN    'd'
#define RC_PITCH_UP  'i'
#define RC_PITCH_DN  'k'
#define RC_ROLL_UP   'j'
#define RC_ROLL_DN   'l'
#define RC_STEP_UP   'q'
#define RC_STEP_DN   'e'
#define RC_STEP       10

// static int rc_step = 10;

void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
    {
        USART_SendData(USART1, UartBuf_RD(&UartTxbuf)); //环形数据缓存发送
        if(UartBuf_Cnt(&UartTxbuf)==0)
        USART_ITConfig(USART1, USART_IT_TXE, DISABLE);//假如缓冲空了，就关闭串口发送中断
    }

    else if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);//清除接收中断标志
        //此种环形缓冲数组串口接收方式，适用于解包各种数据，很方便。对数据的要求是:
        //发送方必须要求有数据包头，以便解决串口数据无地址的问题
        Udatatmp = (uint8_t)USART_ReceiveData(USART1);          //临时数据赋值
        UartBuf_WD(&UartRxbuf, Udatatmp);               //写串口接收缓冲数组
        // if(UartBuf_Cnt(&UartRxbuf)==0) USART_SendData(USART1, '');//串口接收数组长度等于0时，发送接收数组空标志
        // if(UartBuf_Cnt(&UartRxbuf)==UartRxbuf.Mask) USART_SendData(USART1, '');//串口接收数组长度等于掩码时，发送接收缓冲满标志
        CommLink_ReceiveDataFromUSART(Udatatmp);
        // printf("ch:%d ", Udatatmp);
    }
}
