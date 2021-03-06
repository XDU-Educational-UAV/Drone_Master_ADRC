#include "task.h"
/**************文件说明**********************
除了飞行器控制函数之外的定时函数，分别为：
Lock_And_Unlock();             锁定，解锁
RC_Prepare();                  对接收机的信号进行预处理
IMU_Processing();              姿态解算更新，MPU6050数据校准
********************************************/

u8 ReqMsg[4]={0};  //上位机指令
u8 ErrCnt=0;  //未收到遥控器信号的次数
AxisInt acc;  //三轴加速度校准后数据
//以下参数为控制器所使用
AxisInt gyro;  //三轴角速度校准后数据
float roll,pitch,yaw;  //飞行器姿态
short RCdata[4];  //遥控器控制数据
ADRC_Param adrR,adrP;  //自抗扰控制器参数
float Kyaw,YawOut;  //yaw轴比例控制与控制器输出
float RolBias,PitBias,YawBias;  //固定偏差,用于抵消扰动
float throttle=0;  //平衡位置处缓和的实际参考油门输出
short PwmOut[4];  //油门输出,把值赋给定时器,输出PWM

/***********************
姿态解算更新,MPU6050数据校准
*@period:2ms
**********************/
void IMU_Processing(void)
{
	static float IIRax[3],IIRay[3],IIRaz[3];
	static float IIRgx[3],IIRgy[3],IIRgz[3];
	AxisInt oacc,ogyro;
	MPU_Get_Accelerometer(&acc.x,&acc.y,&acc.z);
	MPU_Get_Gyroscope(&gyro.x,&gyro.y,&gyro.z);
	oacc=acc;ogyro=gyro;
	acc.x=IIR_LowPassFilter(oacc.x,IIRax);
	acc.y=IIR_LowPassFilter(oacc.y,IIRay);
	acc.z=IIR_LowPassFilter(oacc.z,IIRaz);
	gyro.x=IIR_LowPassFilter(ogyro.x,IIRgx);
	gyro.y=IIR_LowPassFilter(ogyro.y,IIRgy);
	gyro.z=IIR_LowPassFilter(ogyro.z,IIRgz);
	Acc_Correct(&acc);
	Gyro_Correct(&gyro);
	IMUupdate(acc,gyro,&roll,&pitch,&yaw);
	if(GlobalStat & ACC_CALI)
		if(!Acc_Calibrate(acc))
			GlobalStat &=~ ACC_CALI;
	if(GlobalStat &GYRO_CALI)
		if(!Gyro_Calibrate(gyro))
			GlobalStat &=~ GYRO_CALI;
}

/***********************
失控保护.满足以下任意条件时触发
*侧翻超过75度
*超过2秒未收到遥控信号
触发结果:
超过10秒未收到遥控信号则直接锁定,否则:
低于降落油门则直接锁定,
否则进行姿态保持,油门保持为降落油门
**********************/
void Fail_Safe(char state)
{
	if(state==3)
		GlobalStat&=~MOTOR_LOCK;
	if(throttle<NORMALSPEED-110)
		GlobalStat&=~MOTOR_LOCK;
	else
	{
		RCdata[0]=500;
		RCdata[1]=500;
		throttle=NORMALSPEED-100;
		RCdata[3]=500;
	}
}

/***********************
定时检测是否收到遥控器信号与蓝牙信号
*@period:100ms
**********************/
void RC_Monitor(void)
{
	ErrCnt++;
	if(ErrCnt>=ERR_TIME)
	{
		if(ErrCnt<LOST_TIME)
			Fail_Safe(2);
		else
		{
			Fail_Safe(3);
			ErrCnt--;
		}
	}
	if(STAT_PORT & STAT_Pin)
		LED3_PORT |= LED3_Pin;
	else
		LED3_PORT &=~ LED3_Pin;
}

/***********************
对收到的遥控器信号进行处理.收到一帧数据执行一次
*@period:100ms(Not strict)
**********************/
void RC_Processing(void)
{
	switch(FcnWord)
	{
	case P_STAT:
		if((RxTemp[0]&MOTOR_LOCK)&&(throttle<=LOWSPEED))
			GlobalStat|=MOTOR_LOCK;
		else
			GlobalStat&=~MOTOR_LOCK;
		if((RxTemp[0] & (REQ_MODE_SPEED+REQ_MODE_ATTI))==REQ_MODE_SPEED)
			GlobalStat|=SPEED_MODE;
		else if((RxTemp[0] & (REQ_MODE_SPEED+REQ_MODE_ATTI))==REQ_MODE_ATTI)
			GlobalStat&=~SPEED_MODE;
		break;
	case P_CTRL:
		RCdata[0]=(RxTemp[0]<<8) | RxTemp[1];
		RCdata[1]=(RxTemp[2]<<8) | RxTemp[3];
		RCdata[2]=(RxTemp[4]<<8) | RxTemp[5];
		RCdata[3]=(RxTemp[6]<<8) | RxTemp[7];
		throttle=moderate(RCdata[2],NORMALSPEED);
		break;
	case P_REQ1:
		ReqMsg[0]=RxTemp[0];
		break;
	case P_REQ2:
		ReqMsg[1]=RxTemp[0];
		if(ReqMsg[1] & REQ_ACC_CALI)
			GlobalStat|=ACC_CALI;
		if(ReqMsg[1] & REQ_GYRO_CALI)
			GlobalStat|=GYRO_CALI;
		break;
	case P_REQ3:
		ReqMsg[2]=RxTemp[0];
		break;
	case P_REQ4:
		ReqMsg[3]=RxTemp[0];
		break;
	case P_ROL_CTRL:
		adrR.KpIn=(RxTemp[0]*256.0f+RxTemp[1])/1000.0f;
		adrR.KdIn=(RxTemp[2]*256.0f+RxTemp[3])/1000.0f;
		adrR.KpOut=(RxTemp[4]*256.0f+RxTemp[5])/1000.0f;
		adrR.Kw=(short)(RxTemp[6]*256.0f+RxTemp[7])/1000.0f;
		break;
	case P_PIT_CTRL:
		adrP.KpIn=(RxTemp[0]*256.0f+RxTemp[1])/1000.0f;
		adrP.KdIn=(RxTemp[2]*256.0f+RxTemp[3])/1000.0f;
		adrP.KpOut=(RxTemp[4]*256.0f+RxTemp[5])/1000.0f;
		adrP.Kw=(short)(RxTemp[6]*256.0f+RxTemp[7])/100;
	case P_YAW_CTRL:
		Kyaw=(RxTemp[0]*256.0f+RxTemp[1])/1000.0f;
		adrP.A=(RxTemp[2]*256.0f+RxTemp[3])/1000.0f;
		adrP.B=(RxTemp[4]*256.0f+RxTemp[5])/1000.0f;
	default:break;
	}
	if((ABS(roll)>75)||(ABS(pitch)>75))
	Fail_Safe(1);
}

void RC_Data_Send(void)
{
	ErrCnt=0;
	s16 sdata[6];
	//上位机请求1
	if(ReqMsg[0] & REQ_STAT)
	{
		u16 voltage=Get_Battery_Voltage();
		u8 udata[3]={GlobalStat,BYTE1(voltage),BYTE0(voltage)};
		XDAA_Send_U8_Data(udata,3,P_STAT);
		ReqMsg[0] &=~ REQ_STAT;
	}
	if(ReqMsg[0] & REQ_ATTI)
	{
		sdata[0]=(s16)(roll*100);
		sdata[1]=(s16)(pitch*100);
		sdata[2]=(s16)(yaw*100);
		XDAA_Send_S16_Data(sdata,3,P_ATTI);
		ReqMsg[0] &=~ REQ_ATTI;
	}
	if(ReqMsg[0] & REQ_SENSOR)
	{
		sdata[0]=acc.x;sdata[1]=acc.y;sdata[2]=acc.z;
		sdata[3]=gyro.x;sdata[4]=gyro.y;sdata[5]=gyro.z;
		XDAA_Send_S16_Data(sdata,6,P_SENSOR);
		ReqMsg[0] &=~ REQ_SENSOR;
	}
	if(ReqMsg[0] & REQ_RC)
	{
		XDAA_Send_S16_Data(RCdata,4,P_CTRL);
		ReqMsg[0] &=~ REQ_RC;
	}
	if(ReqMsg[0] & REQ_MOTOR)
	{
		sdata[0]=MOTOR1;
		sdata[1]=MOTOR2;
		sdata[2]=MOTOR3;
		sdata[3]=MOTOR4;
		XDAA_Send_S16_Data(sdata,4,P_MOTOR);
		ReqMsg[0] &=~ REQ_MOTOR;
	}
	if(ReqMsg[0] & REQ_QUATERNION)
	{
		float x=DegToRad(roll/2.0f);
		float y=DegToRad(pitch/2.0f);
		float z=DegToRad(yaw/2.0f);
		float q0=Mcos(x)*Mcos(y)*Mcos(z)+Msin(x)*Msin(y)*Msin(z);
		float q1=Msin(x)*Mcos(y)*Mcos(z)-Mcos(x)*Msin(y)*Msin(z);
		float q2=Mcos(x)*Msin(y)*Mcos(z)+Msin(x)*Mcos(y)*Msin(z);
		float q3=Mcos(x)*Mcos(y)*Msin(z)-Msin(x)*Msin(y)*Mcos(z);
		sdata[0]=(s16)(q0*10000);
		sdata[1]=(s16)(q1*10000);
		sdata[2]=(s16)(q2*10000);
		sdata[3]=(s16)(q3*10000);
		XDAA_Send_S16_Data(sdata,4,P_QUATERNION);
		ReqMsg[0] &=~ REQ_QUATERNION;
	}
	//上位机请求2
	if(ReqMsg[1] & REQ_ROL_CTRL)
	{
		sdata[0]=(s16)(adrR.KpIn*1000);
		sdata[1]=(s16)(adrR.KdIn*1000);
		sdata[2]=(s16)(adrR.KpOut*1000);
		sdata[3]=(s16)(adrR.Kw*1000);
		XDAA_Send_S16_Data(sdata,4,P_ROL_CTRL);
		ReqMsg[1] &=~ REQ_ROL_CTRL;
	}
	if(ReqMsg[1] & REQ_PIT_CTRL)
	{
		sdata[0]=(s16)(adrP.KpIn*1000);
		sdata[1]=(s16)(adrP.KdIn*1000);
		sdata[2]=(s16)(adrP.KpOut*1000);
		sdata[3]=(s16)(adrP.Kw*1000);
		XDAA_Send_S16_Data(sdata,4,P_PIT_CTRL);
		ReqMsg[1] &=~ REQ_PIT_CTRL;
	}
	if(ReqMsg[1] & REQ_YAW_CTRL)
	{
		sdata[0]=(s16)(Kyaw*1000);
		sdata[1]=(s16)(adrP.A*1000);
		sdata[2]=(s16)(adrP.B*1000);
		sdata[3]=0;
		XDAA_Send_S16_Data(sdata,4,P_YAW_CTRL);
		ReqMsg[1] &=~ REQ_YAW_CTRL;
	}
	//上位机请求3
	if(ReqMsg[2] & 0x0F)
	{
		sdata[0]=(s16)(adrP.SpeEst*100);
		sdata[1]=(s16)(GyroToDeg(gyro.y)*100);
		sdata[2]=(s16)(adrP.u*100);
		sdata[3]=(s16)(adrP.w*100);
		XDAA_Send_S16_Data(sdata,4,P_CHART1);
		ReqMsg[2] &=~ 0x0F;
	}
	if(ReqMsg[2] & 0xF0)
	{
		sdata[0]=(s16)(adrP.AccEst*100);
		sdata[1]=0;
		sdata[2]=0;
		sdata[3]=0;
		XDAA_Send_S16_Data(sdata,4,P_CHART2);
		ReqMsg[2] &=~ 0xF0;
	}
}

void HighSpeed_Data_Send(void)
{
	if(!ReqMsg[3])return;
	XDAA_Send_HighSpeed_Data(adrR.u,GyroToDeg(gyro.x));
}
