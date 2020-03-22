#include "task.h"
/**************文件说明**********************
除了飞行器控制函数之外的定时函数，分别为：
Lock_And_Unlock();             锁定，解锁
RC_Prepare();                  对接收机的信号进行预处理
IMU_Processing();              姿态解算更新，MPU6050数据校准
********************************************/

Quaternion Qpos={1,0,0,0};  //姿态四元数和期望四元数(跨文件全局变量)
AxisInt acc;  //三轴加速度
AxisInt gyro;  //三轴角速度(跨文件全局变量)
AxisInt oacc;  //三轴加速度计原始数据
AxisInt ogyro;  //三轴陀螺仪原始数据
short RCdata[4];  //遥控器控制数据(跨文件全局变量)
ADRC_Param adrcRoll,adrcPitch;  //自抗扰控制器参数(跨文件全局变量)
u8 GlobalStat=0;  //全局状态(跨文件全局变量)
float IIRroll[3],IIRpitch[3],IIRyaw[3];

/***********************
姿态解算更新,MPU6050数据校准
*@period:2ms
**********************/
void IMU_Processing(void)
{
	MPU_Get_Accelerometer(&acc.x,&acc.y,&acc.z);
	MPU_Get_Gyroscope(&gyro.x,&gyro.y,&gyro.z);
	oacc=acc;ogyro=gyro;
//	gyro.x=IIR_LowPassFilter(gyro.x,IIRroll);
//	gyro.y=IIR_LowPassFilter(gyro.y,IIRpitch);
//	gyro.z=IIR_LowPassFilter(gyro.z,IIRyaw);
	Acc_Calibrate(&acc);
//	GYRO_Calibrate(&gyro);
	IMUupdate(acc,&gyro,&Qpos);
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
		return;
	RCdata[0]=500;
	RCdata[1]=500;
	RCdata[2]=NORMALSPEED;
	RCdata[3]=500;
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
		if(RCmsg & MOTOR_LOCK)
			GlobalStat|=MOTOR_LOCK;
		else
			GlobalStat&=~MOTOR_LOCK;
		break;
	case P_CTRL:
		RCdata[0]=RCchannel[0];
		RCdata[1]=RCchannel[1];
		RCdata[2]=RCchannel[2];
		RCdata[3]=RCchannel[3];
		break;
	default:break;
	}
	if((Qpos.q1>0.7)||(Qpos.q2>0.7))
		Fail_Safe();
}

/***********************
定时监测是否收到遥控器信号
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
}

void Send_Data(void)
{
	s16 data[6];
	static u8 count=0;
	u8 udata[2]={GlobalStat,(u8)AdcData};
	data[0]=acc.x;data[1]=acc.y;data[2]=acc.z;
	data[3]=gyro.x;data[4]=gyro.y;data[5]=gyro.z;
	XDAA_Send_S16_Data(data,6,P_SENSOR);
	XDAA_Send_S16_Data(RCdata,4,P_CTRL);	
	float roll=Matan2(2*(Qpos.q0*Qpos.q1+Qpos.q2*Qpos.q3),1-2*(Qpos.q1*Qpos.q1+Qpos.q2*Qpos.q2))*57.3f;
	float pitch=Masin(2*(Qpos.q0*Qpos.q2-Qpos.q1*Qpos.q3))*57.3f;
	float yaw=Matan2(2*(Qpos.q1*Qpos.q2+Qpos.q0*Qpos.q3),1-2*(Qpos.q2*Qpos.q2+Qpos.q3*Qpos.q3))*57.3f;
	data[0]=(s16)(roll*100);
	data[1]=(s16)(pitch*100);
	data[2]=(s16)(yaw*100);
	XDAA_Send_S16_Data(data,3,P_ATTI);
	count++;
	if(count>=10)
	{
		XDAA_Send_U8_Data(udata,2,1);
		count=0;
	}
	Total_Send();
}
