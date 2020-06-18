#include "mpu6050.h"

void Delay_ms(unsigned short time_ms)
{
	unsigned short i, j;
	for (i = 0; i < time_ms; i++)
	{
		for (j = 0; j < 10309; j++);
	}
}

//设置MPU6050的陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,成功
#define MPU_Set_Gyro_Fsr(fsr)  (IIC_Write_Reg(0x1B, fsr << 3))  //Register 27 – Gyroscope Configuration

//设置MPU6050的加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,成功
#define MPU_Set_Accel_Fsr(fsr)  (IIC_Write_Reg(0x1C, fsr << 3))  //Register 28 – Accelerometer Configuration

//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,成功
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data = 0;
	if (lpf >= 188)data = 1;
	else if (lpf >= 98)data = 2;
	else if (lpf >= 42)data = 3;
	else if (lpf >= 20)data = 4;
	else if (lpf >= 10)data = 5;
	else data = 6;
	return IIC_Write_Reg(0x1A, data);  //Register 26 – Configuration
}

//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,成功
u8 MPU_Set_Rate(u16 rate)
{
	u8 data,status=0;
	if (rate > 1000)rate = 1000;
	if (rate < 4)rate = 4;
	data = 1000 / rate - 1;
	status |= IIC_Write_Reg(0x19, data);  //Sample Rate Divider
	status |= MPU_Set_LPF(rate / 2);  //自动设置LPF为采样率的一半
	return status;
}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Gyroscope(short *gx, short *gy, short *gz)
{
	u8 buf[6], res;
	res = IIC_Read_Len(0x43, 6, buf);  //Registers 67 to 72 – Gyroscope Measurements
	if (res == 0)
	{
		*gx = ((u16)buf[0] << 8) | buf[1];
		*gy = ((u16)buf[2] << 8) | buf[3];
		*gz = ((u16)buf[4] << 8) | buf[5];
	}
	return res;;
}
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Accelerometer(short *ax, short *ay, short *az)
{
	u8 buf[6], res;
	res = IIC_Read_Len(0x3B, 6, buf);  //Registers 59 to 64 – Accelerometer Measurements
	if (res == 0)
	{
		*ax = ((u16)buf[0] << 8) | buf[1];
		*ay = ((u16)buf[2] << 8) | buf[3];
		*az = ((u16)buf[4] << 8) | buf[5];
	}
	return res;;
}

//初始化MPU6050
//返回值:0,成功
//其他,错误代码
u8 MPU_Init(void)
{
	u8 res,status=0;
	status |= IIC_Write_Reg(0x6B,0x80);  //复位MPU6050
	Delay_ms(100);
	status |= IIC_Write_Reg(0x6B,0x00);  //唤醒MPU6050 
	status |= IIC_Write_Reg(0x38,0x00);  //关闭所有中断
	status |= IIC_Write_Reg(0x6A,0x00);  //I2C主模式关闭
	status |= IIC_Write_Reg(0x23,0x00);  //关闭FIFO
	status |= IIC_Write_Reg(0x6B,0x01);  //设置CLKSEL,PLL X轴为参考
	status |= IIC_Write_Reg(0x6C,0x00);  //加速度与陀螺仪都工作
	status |= MPU_Set_Gyro_Fsr(2);  //陀螺仪传感器,±1000dps
	status |= MPU_Set_Accel_Fsr(0);  //加速度传感器,±2g
	status |= MPU_Set_Rate(500);  //设置采样率500Hz
	status |= IIC_Read_Reg(0x75,&res);  //读取器件ID
	if (res != 0x68)//器件ID正确
		status |= 1;
	return status;
}
