#include "task.h"
/**************文件说明**********************
除了飞行器控制函数之外的定时函数，分别为：
Lock_And_Unlock();             锁定，解锁
RC_Prepare();                  对接收机的信号进行预处理
IMU_Processing();              姿态解算更新，MPU6050数据校准
********************************************/

u8 ReqMsg1=0;  //上位机指令1
u8 ReqMsg2=0;  //上位机指令2
u8 ErrCnt=0;  //未收到遥控器信号的次数
AxisInt oacc;  //三轴加速度计原始数据
AxisInt ogyro;  //三轴陀螺仪原始数据
AxisInt acc;  //三轴加速度校准后数据
AxisInt gyro;  //三轴角速度校准后数据
Quaternion Qpos={1,0,0,0};  //姿态四元数和期望四元数(跨文件全局变量)
short RCdata[4];  //遥控器控制数据(跨文件全局变量)
float gyrox,gyroy;  //角速度,单位度/秒(跨文件全局变量)
//以下为控制器相关参数
ADRC_Param adrR,adrP;  //自抗扰控制器参数(跨文件全局变量)
float Kyaw,YawOut;  //yaw轴比例控制与控制器输出(跨文件全局变量)
float RolBias,PitBias,YawBias;  //固定偏差,用于抵消扰动(跨文件全局变量)

/***********************
姿态解算更新,MPU6050数据校准
*@period:2ms
**********************/
void IMU_Processing(void)
{
	static float IIRax[3],IIRay[3],IIRaz[3];
	static float IIRgx[3],IIRgy[3],IIRgz[3];
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
	gyrox=GyroToDeg(gyro.x);
	gyroy=GyroToDeg(gyro.y);
	IMUupdate(acc,gyro,&Qpos);
	if(GlobalStat & ACC_CALI)
		if(!Acc_Calibrate(acc))
			GlobalStat &=~ ACC_CALI;
	if(GlobalStat &GYRO_CALI)
		if(!Gyro_Calibrate(gyro))
			GlobalStat &=~ GYRO_CALI;
}

/***********************
失控保护.触发条件:条件1必须满足,条件2和3满足其一
1.飞行状态
2.侧翻接近或超过90度
3.两秒未收到遥控器信号
**********************/
void Fail_Safe(void)
{
	if(RCdata[2]<NORMALSPEED)
		GlobalStat&=~MOTOR_LOCK;
	else
	{
		RCdata[0]=500;
		RCdata[1]=500;
		RCdata[2]=NORMALSPEED;
		RCdata[3]=500;
	}
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
		if(RxTemp[1]!=WHO_AM_I)
			break;
		if(RxTemp[0] & MOTOR_LOCK)
		{
			if(RCdata[2]<=LOWSPEED)
				GlobalStat|=MOTOR_LOCK;
		}
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
		break;
	case P_REQ_CTRL:
		ReqMsg1=RxTemp[0];
		ReqMsg2=RxTemp[1];
		if(ReqMsg2 & REQ_ACC_CALI)
			GlobalStat|=ACC_CALI;
		if(ReqMsg2 & REQ_GYRO_CALI)
			GlobalStat|=GYRO_CALI;
		break;
	case P_ROL_CTRL:
		adrR.KpIn=(RxTemp[0]*256.0f+RxTemp[1])/1000.0f;
		adrR.KdIn=(RxTemp[2]*256.0f+RxTemp[3])/1000.0f;
		adrR.KpOut=(RxTemp[4]*256.0f+RxTemp[5])/1000.0f;
		RolBias=(short)(RxTemp[6]*256.0f+RxTemp[7])/100.0f;
		ReqMsg2|=REQ_ROL_CTRL;
		break;
	case P_PIT_CTRL:
		adrP.KpIn=(RxTemp[0]*256.0f+RxTemp[1])/1000.0f;
		adrP.KdIn=(RxTemp[2]*256.0f+RxTemp[3])/1000.0f;
		adrP.KpOut=(RxTemp[4]*256.0f+RxTemp[5])/1000.0f;
		PitBias=(short)(RxTemp[6]*256.0f+RxTemp[7])/100.0f;
		ReqMsg2|=REQ_PIT_CTRL;
	default:break;
	}
	if((Qpos.q1>0.7)||(Qpos.q2>0.7))
	Fail_Safe();
}

/***********************
定时是否收到遥控器信号
*@period:100ms
**********************/
void RC_Monitor(void)
{
	ErrCnt++;
	if(ErrCnt>=ERR_TIME)
	{
		Fail_Safe();
		ErrCnt--;
	}
	if(STAT_PORT & STAT_Pin)
		LED3_PORT |= LED3_Pin;
	else
		LED3_PORT &=~ LED3_Pin;
}

void RC_Data_Send(void)
{
	ErrCnt=0;
	s16 sdata[6];
	//上位机请求1
	if(ReqMsg1 & REQ_STAT)
	{
		u16 voltage=Get_Battery_Voltage();
		u8 udata[3]={GlobalStat,BYTE1(voltage),BYTE0(voltage)};
		XDAA_Send_U8_Data(udata,3,P_STAT);
		ReqMsg1 &=~ REQ_STAT;
	}
	if(ReqMsg1 & REQ_ATTI)
	{
		float roll=Matan2(2*(Qpos.q0*Qpos.q1+Qpos.q2*Qpos.q3),1-2*(Qpos.q1*Qpos.q1+Qpos.q2*Qpos.q2))*57.3f;
		float pitch=Masin(2*(Qpos.q0*Qpos.q2-Qpos.q1*Qpos.q3))*57.3f;
		float yaw=Matan2(2*(Qpos.q1*Qpos.q2+Qpos.q0*Qpos.q3),1-2*(Qpos.q2*Qpos.q2+Qpos.q3*Qpos.q3))*57.3f;
		sdata[0]=(s16)(roll*100);
		sdata[1]=(s16)(pitch*100);
		sdata[2]=(s16)(yaw*100);
		XDAA_Send_S16_Data(sdata,3,P_ATTI);
		ReqMsg1 &=~ REQ_ATTI;
	}
	if(ReqMsg1 & REQ_QUATERNION)
	{
		sdata[0]=(s16)(Qpos.q0*10000);
		sdata[1]=(s16)(Qpos.q1*10000);
		sdata[2]=(s16)(Qpos.q2*10000);
		sdata[3]=(s16)(Qpos.q3*10000);
		XDAA_Send_S16_Data(sdata,4,P_QUATERNION);
		ReqMsg1 &=~ REQ_QUATERNION;
	}
	if(ReqMsg1 & REQ_SENSOR)
	{
		sdata[0]=acc.x;sdata[1]=acc.y;sdata[2]=acc.z;
		sdata[3]=gyro.x;sdata[4]=gyro.y;sdata[5]=gyro.z;
		XDAA_Send_S16_Data(sdata,6,P_SENSOR);
		ReqMsg1 &=~ REQ_SENSOR;
	}
	if(ReqMsg1 & REQ_RC)
	{
		XDAA_Send_S16_Data(RCdata,4,P_CTRL);
		ReqMsg1 &=~ REQ_RC;
	}
	if(ReqMsg1 & REQ_MOTOR)
	{
		sdata[0]=MOTOR1;
		sdata[1]=MOTOR2;
		sdata[2]=MOTOR3;
		sdata[3]=MOTOR4;
		XDAA_Send_S16_Data(sdata,4,P_MOTOR);
		ReqMsg1 &=~ REQ_MOTOR;
	}
	//上位机请求2
	if(ReqMsg2 & REQ_ROL_CTRL)
	{
		sdata[0]=(s16)(adrR.KpIn*1000);
		sdata[1]=(s16)(adrR.KdIn*1000);
		sdata[2]=(s16)(adrR.KpOut*1000);
		sdata[3]=(s16)(RolBias*100);
		XDAA_Send_S16_Data(sdata,4,P_ROL_CTRL);
		ReqMsg2 &=~ REQ_ROL_CTRL;
	}
	if(ReqMsg2 & REQ_PIT_CTRL)
	{
		sdata[0]=(s16)(adrP.KpIn*1000);
		sdata[1]=(s16)(adrP.KdIn*1000);
		sdata[2]=(s16)(adrP.KpOut*1000);
		sdata[3]=(s16)(PitBias*100);
		XDAA_Send_S16_Data(sdata,4,P_PIT_CTRL);
		ReqMsg2 &=~ REQ_PIT_CTRL;
	}
	if(ReqMsg2 & REQ_ROL_STAT)
	{
		sdata[0]=(s16)(adrR.SpeEst*100);
		sdata[1]=(s16)(adrR.u*100);
		sdata[2]=(s16)(adrR.w*100);
		sdata[3]=(s16)(adrR.AccEst*100);
		XDAA_Send_S16_Data(sdata,4,P_ROL_STAT);
		ReqMsg2 &=~ REQ_ROL_STAT;
	}
	if(ReqMsg2 & REQ_PIT_STAT)
	{
		sdata[0]=(s16)(adrP.SpeEst*100);
		sdata[1]=(s16)(adrP.u*100);
		sdata[2]=(s16)(adrP.w*1000);
		sdata[3]=(s16)(adrP.AccEst*100);
		XDAA_Send_S16_Data(sdata,4,P_PIT_STAT);
		ReqMsg2 &=~ REQ_PIT_STAT;
	}
	Total_Send();
}
