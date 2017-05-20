#include <stdio.h>
#include <stdint.h>
#include "stm32f10x_module_comm_link.h"
#include "stm32f10x_module_led.h"
#include "stm32f10x_module_nrf24l01.h"
#include "stm32f10x_algorithm_control.h"

#define RC_STATE_IDLE     0
#define RC_STATE_HEADER1  1
#define RC_STATE_HEADER2  2
#define RC_STATE_COMMAND  3
#define RC_STATE_DATA     4
#define RC_STATE_CHECKSUM 5

//4rc定时50Hz发送
//其余指令触发发送
uint8_t sendBuf[32];
uint8_t sendCnt=0;
uint8_t checksum=0;

// 通信协议层与通信BUS层的接口，移植需要修改此函数。
#define NRFUpload() NRF24L01_TxPacket(sendBuf)

//对外的接口变量,输入rc数据
enum{THROTTLE,YAW,PITCH,ROLL};
uint16_t rcData[4]={1500,1500,1500,1500};             //1000-2000
//对外接口函数在.h


// #define CONSTRAIN(_x,min,max)  {if(_x<min) _x=min; if(_x > max) _x=max;}
// //
// static float dbScaleLinear(float x, float x_end, float deadband)
// {
// 	if (x > deadband) {
// 		return (x - deadband) / (x_end - deadband);

// 	} else if (x < -deadband) {
// 		return (x + deadband) / (x_end - deadband);

// 	} else {
// 		return 0.0f;
// 	}
// }

static  void uart8chk(uint8_t _x)
{
    sendBuf[sendCnt++]=_x;
    checksum ^= _x;
}
//大端
static void uart16chk(int16_t a)
{
    uart8chk((uint8_t)(a&0xff));
    uart8chk((uint8_t)(a>>8));
}

void CommUAVUpload(uint8_t cmd)
{
//  uint8_t len;

    sendCnt=0;

    uart8chk('$');
    uart8chk('M');
    uart8chk('<');
    checksum = 0;

    switch(cmd)
    {
        case MSP_SET_4CON:
            uart8chk(8);            //data payload len
            uart8chk(cmd);
            uart16chk(Throttle);
            uart16chk(Yaw);
            uart16chk(Pitch);
            uart16chk(Roll);
            break;
        case MSP_ARM_IT:
            uart8chk(0);
            uart8chk(cmd);
            break;
        case MSP_DISARM_IT:
            uart8chk(0);
            uart8chk(cmd);
            break;
        case MSP_HOLD_ALT:
            uart8chk(0);
            uart8chk(cmd);
            break;
        case MSP_STOP_HOLD_ALT:
            uart8chk(0);
            uart8chk(cmd);
            break;
        case MSP_HEAD_FREE:
            uart8chk(0);
            uart8chk(cmd);
            break;
        case MSP_STOP_HEAD_FREE:
            uart8chk(0);
            uart8chk(cmd);
            break;
        case MSP_AUTO_LAND_DISARM:
            uart8chk(0);
            uart8chk(cmd);
            break;
        case MSP_ACC_CALI:
            uart8chk(0);
            uart8chk(cmd);
            break;
    }

    uart8chk(checksum);
    NRFUpload();
}

#define RC_COMMAND_IMU_CALI 0x01
#define RC_COMMAND_ARM_LOCK 0x02
#define RC_COMMAND_CONTROL  0x03

static u8 rc_checksum   = 0;
static u8 rc_command    = 0;
static u8 rc_length     = 0;
static u8 rc_count      = 0;
static u8 rc_state      = RC_STATE_IDLE;
static u8 rc_buffer[16] = {0};

extern char IMUcalibratflag;
extern char Lockflag;
extern int  Throttle;
extern int  Roll;
extern int  Pitch;
extern int  Yaw;

void CommLink_ReceiveDataFromUSART(u8 byte)
{
    switch (rc_state)
    {
        case RC_STATE_IDLE:
        {
            rc_checksum = 0;
            if (byte == 0xAA)
            {
                rc_state = RC_STATE_HEADER1;
            }
            break ;
        }
        case RC_STATE_HEADER1:
        {
            if (byte == 0xAA)
            {
                rc_state = RC_STATE_HEADER2;
            }
            else
            {
                rc_state = RC_STATE_IDLE;
            }
            break ;
        }
        case RC_STATE_HEADER2:
        {
            rc_command = byte;
            rc_state   = RC_STATE_COMMAND;
            break ;
        }
        case RC_STATE_COMMAND:
        {
            rc_length   = byte;
            rc_state    = RC_STATE_DATA;
            rc_checksum = 0xAA + 0xAA + rc_command + rc_length;
            break ;
        }
        case RC_STATE_DATA:
        {
            if (rc_count < rc_length)
            {
                rc_buffer[rc_count++] = byte;
                rc_checksum += byte;
            }
            else if (rc_count == rc_length)
            {
                rc_state = RC_STATE_CHECKSUM;
                CommLink_ProcessDataFromUSART();
            }
            else
            {
                ;
            }
            break ;
        }
        case RC_STATE_CHECKSUM:
        {
            if (rc_checksum == byte)
            {
            }
            rc_count = 0;
            rc_state = RC_STATE_IDLE;
            break ;
        }
    }
}

void CommLink_ProcessDataFromUSART(void)
{
    // int i = 0;
    // for (i = 0; i < 16; i++)
    // {
    //     printf("buf:%d ", rc_buffer[i]);
    // }

    switch (rc_command)
    {
        case 0x01:
        {
            IMUcalibratflag = !IMUcalibratflag;
            break ;
        }
        case 0x02:
        {
            Lockflag = 1;
            break ;
        }
        case 0x03:
        {
            Roll     = rc_buffer[0] + (rc_buffer[1] << 8);
            Pitch    = rc_buffer[2] + (rc_buffer[3] << 8);
            Yaw      = rc_buffer[4] + (rc_buffer[5] << 8);
            Throttle = rc_buffer[6] + (rc_buffer[7] << 8);
            break ;
        }
        default:
        {
            break ;
        }
    }
}
