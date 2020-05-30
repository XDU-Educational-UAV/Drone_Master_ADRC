#ifndef _IMU_H_
#define	_IMU_H_

#include "mymath.h"
typedef unsigned char u8;

typedef struct
{
	float x,y,z;
}Axis;
typedef struct
{
	short x,y,z;
}AxisInt;

void Acc_Correct(AxisInt *acc);
void Gyro_Correct(AxisInt *gyro);
u8 Acc_Calibrate(AxisInt acc);
u8 Gyro_Calibrate(AxisInt gyro);
void IMUupdate(AxisInt acc,AxisInt gyro,float *rol,float *pit,float *yaw);

#endif
