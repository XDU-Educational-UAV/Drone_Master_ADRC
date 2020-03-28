#include "task.h"
/**************文件说明**********************
Para_Init();
Motor_Iner_loop();
Motor_Outer_loop();
与task.c共享头文件task.h
--------------飞行器安装与控制---------------
F450四轴，X型，红前白后，从右前方电机编号为1开始逆时针编号
********************************************/
short PwmOut[4]={0,0,0,0};  //把值赋给定时器，输出PWM

/**********************
控制参数初始化
必须在上电后至少一个PWM周期后进行
**********************/
void Para_Init(void)
{
	MOTOR1=PwmOut[0];MOTOR2=PwmOut[1];MOTOR3=PwmOut[2];MOTOR4=PwmOut[3];
	adrcRoll.KpOut=0;adrcRoll.KpIn=0.5;adrcRoll.KdIn=0;
	adrcPitch.KpOut=0;adrcPitch.KpIn=0.5;adrcPitch.KdIn=0;
}

/**********************
电机内环自抗扰控制
**********************/
void Motor_Iner_loop(void)
{
	if(!(GlobalStat & MOTOR_LOCK))
	{
		MOTOR1=0;
		MOTOR2=0;
		MOTOR3=0;
		MOTOR4=0;
		return;
	}
	if(RCdata[2]<=LOWSPEED)
	{
		MOTOR1=LOWSPEED;
		MOTOR2=LOWSPEED;
		MOTOR3=LOWSPEED;
		MOTOR4=LOWSPEED;
		return;
	}
	float gx=GyroToDeg(gyro.x);
	float gy=GyroToDeg(gyro.y);
	ADRC_LESO(adrcRoll.u,gx,1,&adrcRoll.gEst,&adrcRoll.AccEst,&adrcRoll.w);
	ADRC_LESO(adrcPitch.u,gy,1,&adrcPitch.gEst,&adrcPitch.AccEst,&adrcPitch.w);
	adrcRoll.u=adrcRoll.PosOut-adrcRoll.KpIn*gx-adrcRoll.KdIn*adrcRoll.AccEst;
	adrcPitch.u=adrcPitch.PosOut-adrcPitch.KpIn*gy-adrcPitch.KdIn*adrcPitch.AccEst;
	PwmOut[0]=RCdata[2]+DegToPwm(-adrcRoll.u-adrcPitch.u);
	PwmOut[1]=RCdata[2]+DegToPwm(-adrcRoll.u+adrcPitch.u);
	PwmOut[2]=RCdata[2]+DegToPwm(+adrcRoll.u+adrcPitch.u);
	PwmOut[3]=RCdata[2]+DegToPwm(+adrcRoll.u-adrcPitch.u);
	MOTOR1=LIMIT(PwmOut[0],LOWSPEED,1000);
	MOTOR2=LIMIT(PwmOut[1],LOWSPEED,1000);
	MOTOR3=LIMIT(PwmOut[2],LOWSPEED,1000);
	MOTOR4=LIMIT(PwmOut[3],LOWSPEED,1000);
}

#define SPEED  //角速度模式
/**********************
电机外环比例控制
**********************/
void Motor_Outer_loop(void)
{
#ifdef SPEED
	adrcRoll.PosOut=PwmToDegAdd(RCdata[0]);
	adrcPitch.PosOut=PwmToDegAdd(RCdata[1]);
#else
	Quaternion Qexp;
	float Hroll=PwmToRadAdd(RCdata[0])/2;
	float Hpitch=PwmToRadAdd(RCdata[1])/2;
	float Hyaw=PwmToRadAdd(RCdata[3])/2;
	Qexp.q0=Mcos(Hroll)*Mcos(Hpitch)*Mcos(Hyaw)+Msin(Hroll)*Msin(Hpitch)*Msin(Hyaw);
	Qexp.q1=Msin(Hroll)*Mcos(Hpitch)*Mcos(Hyaw)-Mcos(Hroll)*Msin(Hpitch)*Msin(Hyaw);
	Qexp.q2=Mcos(Hroll)*Msin(Hpitch)*Mcos(Hyaw)+Msin(Hroll)*Mcos(Hpitch)*Msin(Hyaw);
	Qexp.q3=Mcos(Hroll)*Mcos(Hpitch)*Msin(Hyaw)-Msin(Hroll)*Msin(Hpitch)*Mcos(Hyaw);
	Quaternion Qerr=Quaternion_Error(Qexp,Qpos);
	adrcRoll.PosOut=adrcRoll.KpOut*RadToDeg(Masin(Qerr.q1));
	adrcPitch.PosOut=adrcPitch.KpOut*RadToDeg(Masin(Qerr.q2));
#endif
}
