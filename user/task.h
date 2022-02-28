#ifndef __TASK_H
#define __TASK_H

#include "protocol.h"
#include "imu.h"
#include "mpu6050.h"
#include "adrc.h"
#include "adc.h"

#define MOTOR1        (TIM1->CCR3) 
#define MOTOR2        (TIM1->CCR2)
#define MOTOR3        (TIM1->CCR4)
#define MOTOR4        (TIM1->CCR1)

#define LOWSPEED      50  //怠速
#define NORMALSPEED   400  //平衡时的油门大小
#define ERR_TIME      20   //没能收到正确遥控器信号的次数
#define LOST_TIME     100  //长时间没能收到正确遥控器信号的次数
//LockMode
#define LOCKED        0    //锁定状态且无操作
#define TOUNLOCK      1    //锁定状态且尝试解锁
#define UNLOCKED      2    //解锁状态
#define LOCK_TIME     20   //解锁时间,2秒

extern AxisInt Gyro;
extern float roll,pitch,yaw;
extern short RCdata[];
extern ADRC_Param adrcX,adrcY;
extern float Kyaw,YawOut;

//在task.c中
void IMU_Processing(void);
void RC_Processing(void);
void RC_Monitor(void);
void RC_Data_Send(void);
void RC_Data_Send_10ms(void);
void HighSpeed_Data_Send(void);
//在control.c中
void Para_Init(void);
void Motor_Iner_loop(void);
void Motor_Outer_loop(void);

#endif
