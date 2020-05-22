#include "task.h"
/**************文件说明**********************
Para_Init();
Motor_Iner_loop();
Motor_Outer_loop();
与task.c共享头文件task.h
--------------飞行器安装与控制---------------
X型四轴,从左后方电机编号为1开始顺时针编号,1电机螺旋桨为逆时针
********************************************/

/**********************
控制参数初始化
**********************/
void Para_Init(void)
{
	MOTOR1=PwmOut[0];MOTOR2=PwmOut[1];MOTOR3=PwmOut[2];MOTOR4=PwmOut[3];
	adrR.KpOut=2.0f;adrR.KpIn=0.7f;adrR.KdIn=0.02f;adrR.Kw=0;
	adrP.KpOut=0;adrP.KpIn=0;adrP.KdIn=0;adrP.Kw=0;
	Kyaw=0;
	RolBias=0;PitBias=0;YawBias=0;
}

/**********************
电机内环自抗扰控制
**********************/
void Motor_Iner_loop(void)
{
	float gx=GyroToDeg(gyro.x);
	float gy=GyroToDeg(gyro.y);
	ADRC_LESO(&adrR,gx);
	ADRC_LESO(&adrP,gy);
	adrR.u=adrR.KpIn*(adrR.PosOut-gx)-adrR.KdIn*adrR.AccEst-adrR.Kw*adrR.w;
	adrP.u=adrP.KpIn*(adrP.PosOut-gy)-adrP.KdIn*adrP.AccEst-adrP.Kw*adrP.w;
	PwmOut[0]=throttle+DegToPwm(-adrR.u-adrP.u-YawOut);
	PwmOut[1]=throttle+DegToPwm(-adrR.u+adrP.u+YawOut);
	PwmOut[2]=throttle+DegToPwm(+adrR.u+adrP.u-YawOut);
	PwmOut[3]=throttle+DegToPwm(+adrR.u-adrP.u+YawOut);
	if(!(GlobalStat & MOTOR_LOCK))
	{
		MOTOR1=0;
		MOTOR2=0;
		MOTOR3=0;
		MOTOR4=0;
	}
	else if(throttle<=LOWSPEED)
	{
		MOTOR1=LOWSPEED;
		MOTOR2=LOWSPEED;
		MOTOR3=LOWSPEED;
		MOTOR4=LOWSPEED;
	}
	else
	{
		MOTOR1=LIMIT(PwmOut[0],LOWSPEED,1000);
		MOTOR2=LIMIT(PwmOut[1],LOWSPEED,1000);
		MOTOR3=LIMIT(PwmOut[2],LOWSPEED,1000);
		MOTOR4=LIMIT(PwmOut[3],LOWSPEED,1000);
	}
}

/**********************
电机外环比例控制
**********************/
void Motor_Outer_loop(void)
{
	if(GlobalStat & SPEED_MODE)
	{
		adrR.PosOut=PwmToDegAdd(RCdata[0])/adrR.KpIn;
		adrP.PosOut=PwmToDegAdd(RCdata[1])/adrP.KpIn;
	}
	else
	{
		float RolExp=PwmToDegAdd(RCdata[0]);
		float PitExp=PwmToDegAdd(RCdata[1]);
		adrR.PosOut=adrR.KpOut*(RolExp-roll);
		adrP.PosOut=adrP.KpOut*(pitch-PitExp);
	}
	adrR.PosOut+=RolBias;
	adrP.PosOut+=PitBias;
	YawOut=PwmToDegAdd(RCdata[3]+YawBias)-Kyaw*GyroToDeg(gyro.z);
}
