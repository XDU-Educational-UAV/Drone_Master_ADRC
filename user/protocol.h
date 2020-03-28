#ifndef __NIMING_H
#define __NIMING_H

#include "usart.h"
#include "task.h"

#define SENDBUF_SIZE   128
//FcnWord
#define P_STAT         0x01  //状态
#define P_ATTI         0x02  //姿态
#define P_SENSOR       0x04  //传感器
#define P_CTRL         0x08  //遥控
#define P_MOTOR        0x10  //油门
#define P_REQ_CTRL     0xA0  //读参数
#define P_ROL_CTRL     0xA1  //ROL参数
#define P_ROL_STAT     0xA2  //ROL状态
#define P_PIT_CTRL     0xA3  //PIT参数
#define P_PIT_STAT     0xA4  //PIT状态

extern u8 FcnWord;  //被task.c调用
extern u8 LenWord;  //被task.c调用
extern u8 RxTemp[12];  //被task.c调用

void Protocol_Init(void);
void XDAA_Send_S16_Data(s16 *data,u8 len,u8 fcn);
void XDAA_Send_U8_Data(u8 *data,u8 len,u8 fcn);
void Total_Send(void);

#endif
