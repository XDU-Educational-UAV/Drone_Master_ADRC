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
	adrcRoll.KpOut=0;adrcRoll.KpIn=2;adrcRoll.KdIn=0.1,adrcRoll.AccEst=0;
	adrcPitch.KpOut=0;adrcPitch.KpIn=2;adrcPitch.KdIn=0.1,adrcPitch.AccEst=0;
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
	float gx=GyroToDeg(gyro.x);
	float gy=GyroToDeg(gyro.y);
	ADRC_LESO(adrcRoll.PosOut,gx,1,&adrcRoll.gEst,&adrcRoll.AccEst);
	ADRC_LESO(adrcPitch.PosOut,gx,1,&adrcPitch.gEst,&adrcPitch.AccEst);
	adrcRoll.SpeOut=adrcRoll.KpIn*(adrcRoll.PosOut-gx);
	adrcPitch.SpeOut=adrcPitch.KpIn*(adrcPitch.PosOut-gx);
	float RollOut=adrcRoll.KdIn*(adrcRoll.SpeOut-adrcRoll.AccEst);
	float PitchOut=adrcPitch.KdIn*(adrcPitch.SpeOut-adrcPitch.AccEst);
	PwmOut[0]=RCdata[2]+DegToPwm(+RollOut+PitchOut);
	PwmOut[1]=RCdata[2]+DegToPwm(-RollOut+PitchOut);
	PwmOut[2]=RCdata[2]+DegToPwm(-RollOut-PitchOut);
	PwmOut[3]=RCdata[2]+DegToPwm(+RollOut-PitchOut);
	MOTOR1=LIMIT(PwmOut[0],0,1000);
	MOTOR2=LIMIT(PwmOut[1],0,1000);
	MOTOR3=LIMIT(PwmOut[2],0,1000);
	MOTOR4=LIMIT(PwmOut[3],0,1000);
}

/**********************
电机外环比例控制
**********************/
void Motor_Outer_loop(void)
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
	adrcRoll.PosOut=adrcRoll.KpOut*RadToDeg(Masin(Qerr.q1));
	adrcPitch.PosOut=adrcPitch.KpOut*RadToDeg(Masin(Qerr.q2));
}
