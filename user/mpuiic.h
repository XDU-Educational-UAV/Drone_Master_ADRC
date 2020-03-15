#ifndef __MPUIIC_H
#define __MPUIIC_H

#include "main.h"

#define	SCL_H   	SCL_PORT->BSRR = SCL_PIN    //SCL高电平
#define	SCL_L   	SCL_PORT->BRR  = SCL_PIN    //SCL低电平
#define	SDA_H   	SDA_PORT->BSRR = SDA_PIN    //SDA高电平
#define	SDA_L   	SDA_PORT->BRR  = SDA_PIN    //SDA低电平
#define	SDA_Read	SDA_PORT->IDR  & SDA_PIN    //SDA读数据

#define SCL_PORT     GPIOB
#define SDA_PORT     GPIOB
#define SCL_PIN      GPIO_PIN_8
#define SDA_PIN      GPIO_PIN_7
//IO方向设置
#define SDA_IN       GPIOB->MODER&=0xFFFF3FFF
#define SDA_OUT      GPIOB->MODER&=0xFFFF3FFF;GPIOB->MODER|=1<<14
#define MPU_ADDR 0x68
u8 IIC_Write_Reg(u8 reg,u8 data);  //IIC写一个字节
u8 IIC_Read_Reg(u8 reg, u8 *data);  //IIC读一个字节
u8 IIC_Read_Len(u8 reg, u8 len, u8 *buf);
#endif
