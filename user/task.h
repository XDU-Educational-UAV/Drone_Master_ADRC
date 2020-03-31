#ifndef __TASK_H
#define __TASK_H

#include "protocol.h"
#include "imu.h"
#include "mpu6050.h"
#include "adrc.h"
#include "adc.h"

#define WHO_AM_I       0x48

#define MOTOR1        (TIM1->CCR3) 
#define MOTOR2        (TIM1->CCR2)
#define MOTOR3        (TIM1->CCR4)
#define MOTOR4        (TIM1->CCR1)

#define LOWSPEED      100  //怠速
#define NORMALSPEED   400  //缓慢下降的大致速度,用于失控保护
#define ERR_TIME      20   //没能收到正确遥控器信号的次数
//LockMode
#define LOCKED        0    //锁定状态且无操作
#define TOUNLOCK      1    //锁定状态且尝试解锁
#define UNLOCKED      2    //解锁状态
#define LOCK_TIME     20   //解锁时间,2秒

extern short RCdata[];  //被control.c调用
extern AxisInt gyro;  //被control.c调用
extern ADRC_Param adrR,adrP;  //被control.c调用
extern Quaternion Qpos;  //被control.c调用
extern float gx,gy;

//在task.c中
void IMU_Processing(void);
void RC_Processing(void);
void RC_Monitor(void);
void RC_Data_Send(void);
//在control.c中
void Para_Init(void);
void Motor_Iner_loop(void);
void Motor_Outer_loop(void);

#endif
