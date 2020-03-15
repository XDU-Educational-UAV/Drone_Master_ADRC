#ifndef __NIMING_H
#define __NIMING_H

#include "usart.h"

#define XDAA_Send_Data(DataToSend,cnt)  HAL_UART_Transmit_DMA(&huart2,DataToSend,cnt)


#define FLY_CTRL       0x02
#define USER_FRAME     0xF1

extern short CtrlCmd[4];
extern u8 RCcmd;

void Protocol_Init(void);
void XDAA_Send_User_Data(s16 *UserFrame,u8 len);

#endif
