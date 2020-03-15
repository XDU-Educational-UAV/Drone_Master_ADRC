#ifndef __MPU6050_H
#define __MPU6050_H

#include "mpuiic.h"

u8 MPU_Init(void);
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz);
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az);
void Delay_ms(unsigned short time_ms);

#endif
