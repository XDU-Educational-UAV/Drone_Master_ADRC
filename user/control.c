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
float Kyaw=1;

/**********************
控制参数初始化
必须在上电后至少一个PWM周期后进行
**********************/
void Para_Init(void)
{
	MOTOR1=PwmOut[0];MOTOR2=PwmOut[1];MOTOR3=PwmOut[2];MOTOR4=PwmOut[3];
	adrR.KpOut=1.0f;adrR.KpIn=0.8f;adrR.KdIn=0.01f;adrR.Kw=0;
	adrP.KpOut=1.0f;adrP.KpIn=0.3f;adrP.KdIn=0.00f;adrP.Kw=0;
}

/**********************
电机内环自抗扰控制
**********************/
void Motor_Iner_loop(void)
{
	ADRC_LESO(&adrR,gyrox);
	ADRC_LESO(&adrP,gyroy);
	adrR.u=adrR.KpIn*(adrR.PosOut-gyrox)-adrR.KdIn*adrR.AccEst-adrR.Kw*adrR.w;
	adrP.u=adrP.KpIn*(adrP.PosOut-gyroy)-adrP.KdIn*adrP.AccEst-adrP.Kw*adrP.w;
	PwmOut[0]=RCdata[2]+DegToPwm(-adrR.u-adrP.u-PwmToDegAdd(RCdata[3]));
	PwmOut[1]=RCdata[2]+DegToPwm(-adrR.u+adrP.u+PwmToDegAdd(RCdata[3]));
	PwmOut[2]=RCdata[2]+DegToPwm(+adrR.u+adrP.u-PwmToDegAdd(RCdata[3]));
	PwmOut[3]=RCdata[2]+DegToPwm(+adrR.u-adrP.u+PwmToDegAdd(RCdata[3]));
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
		adrR.PosOut=PwmToDegAdd(RCdata[0])/adrR.KpIn;
		adrP.PosOut=PwmToDegAdd(RCdata[1])/adrP.KpIn;
	}
	else
	{
		Quaternion Qexp;
		float Hroll=PwmToRadAdd(RCdata[0])/2;
		float Hpitch=PwmToRadAdd(RCdata[1])/2;
		float Hyaw=PwmToRadAdd(RCdata[3])/2;
		Qexp.q0=Mcos(Hroll)*Mcos(Hpitch)*Mcos(Hyaw)+Msin(Hroll)*Msin(Hpitch)*Msin(Hyaw);
		Qexp.q1=Msin(Hroll)*Mcos(Hpitch)*Mcos(Hyaw)-Mcos(Hroll)*Msin(Hpitch)*Msin(Hyaw);
		Qexp.q2=Mcos(Hroll)*Msin(Hpitch)*Mcos(Hyaw)+Msin(Hroll)*Mcos(Hpitch)*Msin(Hyaw);
		Qexp.q3=Mcos(Hroll)*Mcos(Hpitch)*Msin(Hyaw)-Msin(Hroll)*Msin(Hpitch)*Mcos(Hyaw);
		Quaternion Qerr=Quaternion_Error(Qexp,Qpos);
		float norm=Q_rsqrt(Qerr.q1*Qerr.q1+Qerr.q2*Qerr.q2)*Macos(Qerr.q0);
		adrR.PosOut=adrR.KpOut*RadToDeg(Qerr.q1*norm);
		adrP.PosOut=adrP.KpOut*RadToDeg(Qerr.q2*norm);
	}
}
