#ifndef __NIMING_H
#define __NIMING_H

#include "usart.h"

#define WHO_AM_I       0x48
#define SENDBUF_SIZE   128
//FcnWord
#define P_STAT         0x01
#define P_ATTI         0x02
#define P_SENSOR       0x04
#define P_CTRL         0x08
#define P_MOTOR        0x10
//RCmsg
#define RC_RECEIVE     0x80
#define MOTOR_LOCK     0x01
#define ACC_CALI       0x02
#define GYRO_CALI      0x04

extern u8 FcnWord;  //被task.c调用
extern u8 LenWord;  //被task.c调用
extern short RCchannel[4];  //被task.c调用
extern u8 ErrCnt;  //被task.c调用
extern u8 RCmsg;  //被task.c,main.c调用

void Protocol_Init(void);
void XDAA_Send_S16_Data(s16 *data,u8 len,u8 fcn);
void XDAA_Send_U8_Data(u8 *data,u8 len,u8 fcn);
void Total_Send(void);

#endif
