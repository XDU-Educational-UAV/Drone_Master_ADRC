#include "mpuiic.h"

void IIC_Delay(void)
{
	__nop();__nop();__nop();
	__nop();__nop();__nop();
	__nop();__nop();__nop();
}

//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT;
	SDA_H;
	SCL_H;
	IIC_Delay();
 	SDA_L;
	IIC_Delay();
	SCL_L;
}
//产生IIC停止信号
void IIC_Stop(void)
{
	SCL_L;
	SDA_OUT;
	SDA_L;
 	IIC_Delay();
	SCL_H;
	IIC_Delay();
	SDA_H;
}
//等待应答信号到来
//返回值:0,成功
u8 IIC_Wait_Ack(void)
{
	SDA_IN;
	SCL_L;
	IIC_Delay();
	SCL_H;
	IIC_Delay();
	if(SDA_Read)
	{
		SCL_L;
		return 1;
	}
	SCL_L;
	return 0;
}
//产生ACK应答
void IIC_Ack(void)
{
	SCL_L;
	SDA_OUT;
	SDA_L;
	IIC_Delay();
	SCL_H;
	IIC_Delay();
	SCL_L;
}
//不产生ACK应答		    
void IIC_NAck(void)
{
	SCL_L;
	SDA_OUT;
	SDA_H;
	IIC_Delay();
	SCL_H;
	IIC_Delay();
	SCL_L;
}
//IIC发送一个字节
void IIC_Write_Byte(u8 txd)
{
	u8 t=8;
	SCL_L;
	SDA_OUT;
	while(t--)
	{
		if(txd&0x80)
			SDA_H;
		else
			SDA_L;
		txd<<=1;
		SCL_H;
		IIC_Delay();
		SCL_L;
		IIC_Delay();
	}
}
//读一个字节
//ack=1时发送ACK;ack=0时发送nACK
//返回值:读取的一个字节
u8 IIC_Read_Byte(unsigned char ack)
{
	u8 t=8,receive=0;
	SDA_IN;
	SCL_L;
	while(t--)
	{
		IIC_Delay();
		SCL_H;
		receive<<=1;
		if(SDA_Read) receive++;
		IIC_Delay();
		SCL_L;
	}
	if (!ack)
		IIC_NAck();
	else
		IIC_Ack();
	return receive;
}
//IIC写一个字节
//reg:寄存器地址
//data:数据
//返回值:0,成功
u8 IIC_Write_Reg(u8 reg, u8 data)
{
	IIC_Start();
	IIC_Write_Byte((MPU_ADDR << 1) | 0);
	if (IIC_Wait_Ack())
	{
		IIC_Stop();
		return 1;
	}
	IIC_Write_Byte(reg);
	if (IIC_Wait_Ack())
	{
		IIC_Stop();
		return 2;
	}
	IIC_Write_Byte(data);
	if (IIC_Wait_Ack())
	{
		IIC_Stop();
		return 3;
	}
	IIC_Stop();
	return 0;
}
//IIC读一个字节 
//reg:寄存器地址 
//返回值:0,成功
u8 IIC_Read_Reg(u8 reg, u8 *data)
{
	IIC_Start();
	IIC_Write_Byte((MPU_ADDR << 1) | 0);
	if (IIC_Wait_Ack())
	{
		IIC_Stop();
		return 1;
	}
	IIC_Write_Byte(reg);
	if (IIC_Wait_Ack())
	{
		IIC_Stop();
		return 2;
	}
	IIC_Start();
	IIC_Write_Byte((MPU_ADDR << 1) | 1);
	if (IIC_Wait_Ack())
	{
		IIC_Stop();
		return 3;
	}
	*data = IIC_Read_Byte(0);
	IIC_Stop();
	return 0;
}
//IIC连续读
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,成功
u8 IIC_Read_Len(u8 reg, u8 len, u8 *buf)
{
	IIC_Start();
	IIC_Write_Byte((MPU_ADDR << 1) | 0);
	if (IIC_Wait_Ack())
	{
		IIC_Stop();
		return 1;
	}
	IIC_Write_Byte(reg);
	if (IIC_Wait_Ack())
	{
		IIC_Stop();
		return 2;
	}
	IIC_Start();
	IIC_Write_Byte((MPU_ADDR << 1) | 1);
	if (IIC_Wait_Ack())
	{
		IIC_Stop();
		return 3;
	}
	while (len)
	{
		if (len == 1)*buf = IIC_Read_Byte(0);
		else *buf = IIC_Read_Byte(1);
		len--;
		buf++;
	}
	IIC_Stop();
	return 0;
}
