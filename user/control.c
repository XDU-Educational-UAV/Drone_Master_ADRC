#include "task.h"
/**************文件说明**********************
Para_Init();
Motor_Iner_loop();
Motor_Outer_loop();
与task.c共享头文件task.h
--------------飞行器安装与控制---------------
X型四轴,从左后方电机编号为1开始顺时针编号,1电机螺旋桨为逆时针
********************************************/

short PwmOut[4];  //油门输出,把值赋给定时器,输出PWM
/**********************
控制参数初始化
**********************/
void Para_Init(void)
{
	MOTOR1=PwmOut[0];MOTOR2=PwmOut[1];MOTOR3=PwmOut[2];MOTOR4=PwmOut[3];
	adrR.KpOut=2.0f;adrR.KpIn=0.15f;adrR.KiIn=0;adrR.KdIn=0;adrR.Kw=0;
	adrP.KpOut=2.0f;adrP.KpIn=0.15f;adrP.KiIn=0;adrP.KdIn=0;adrP.Kw=0;
	Kyaw=0;
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
	adrR.SpeErr=adrR.AttOut-gx;
	adrP.SpeErr=adrP.AttOut-gy;
	adrR.u = adrR.KpIn * adrR.SpeErr + adrR.KiIn * adrR.SpeInt - adrR.KdIn * adrR.AccEst - adrR.Kw * adrR.w;
	adrP.u = adrP.KpIn * adrP.SpeErr + adrP.KiIn * adrP.SpeInt - adrP.KdIn * adrP.AccEst - adrP.Kw * adrP.w;
	adrR.SpeInt += adrR.SpeErr * T;
	adrP.SpeInt += adrP.SpeErr * T;
	PwmOut[0]=RCdata[2]+DegToPwm(-adrR.u-adrP.u-YawOut);
	PwmOut[1]=RCdata[2]+DegToPwm(-adrR.u+adrP.u+YawOut);
	PwmOut[2]=RCdata[2]+DegToPwm(+adrR.u+adrP.u-YawOut);
	PwmOut[3]=RCdata[2]+DegToPwm(+adrR.u-adrP.u+YawOut);
	if(!(GlobalStat & MOTOR_LOCK))
	{
		MOTOR1=0;
		MOTOR2=0;
		MOTOR3=0;
		MOTOR4=0;
	}
	else if(RCdata[2]<=LOWSPEED)
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
		adrR.AttOut=+PwmToDegAdd(RCdata[0]);
		adrP.AttOut=-PwmToDegAdd(RCdata[1]);
	}
	else
	{
		adrR.AttOut=adrR.KpOut*(PwmToDegAdd(RCdata[0])-roll);
		adrP.AttOut=adrP.KpOut*(-PwmToDegAdd(RCdata[1])-pitch);
	}
	YawOut=Kyaw*(PwmToDegAdd(RCdata[3])-GyroToDeg(gyro.z));
}
