#include "imu.h"
/**************文件说明**********************
传感器数据校准,滤波,互补滤波解算四元数
********************************************/

#define T  0.002    //采样周期,2ms
#define hT 0.001   //采样周期/2
#define Kp 2.0f
#define Ki 0.1f

/*校准参数1
const float accA[3][3]={
	{1,0,0},
	{0,1,0},
	{0,0,1}};
short accB[3]={0,0,0};
short gyroB[3]={0,0,0};*/
//校准参数2
const float accA[3][3]={
	{0.984375f,-0.019751222473827f,0.005168265332243f},
	{-0.0390625f,1,0.046875f},
	{-0.0234375f,0.0390625f,1}};
short accB[3]={-4955,-3893,1866};
short gyroB[3]={447,45,-1};

/***********************
用事先确定的校准参数校正加速度计原始数据
**********************/
void Acc_Correct(AxisInt *acc)
{
	float ax=acc->x,ay=acc->y,az=acc->z;
	acc->x=accA[0][0]*ax+accA[0][1]*ay+accA[0][2]*az+accB[0];
	acc->y=accA[1][0]*ax+accA[1][1]*ay+accA[1][2]*az+accB[1];
	acc->z=accA[2][0]*ax+accA[2][1]*ay+accA[2][2]*az+accB[2];
}
/***********************
用事先确定的校准参数校正陀螺仪原始数据
**********************/
void Gyro_Correct(AxisInt *gyro)
{
	gyro->x+=gyroB[0];
	gyro->y+=gyroB[1];
	gyro->z+=gyroB[2];
}

/***********************
根据加速度计原始数据计算校准参数
**********************/
u8 Acc_Calibrate(AxisInt acc)
{
	static long sumx=0,sumy=0,sumz=0;
	static char cnt=0;
	acc.z-=16384;
	if(cnt<50)
	{
		sumx+=acc.x;
		sumy+=acc.y;
		sumz+=acc.z;
		cnt++;
		return 1;
	}
	else
	{
		accB[0]-=sumx/50;
		accB[1]-=sumy/50;
		accB[2]-=sumz/50;
		sumx=sumy=sumz=0;
		cnt=0;
		return 0;
	}
}
/***********************
根据陀螺仪原始数据计算校准参数
**********************/
u8 Gyro_Calibrate(AxisInt gyro)
{
	static long sumx=0,sumy=0,sumz=0;
	static char cnt=0;
	if(cnt<50)
	{
		sumx+=gyro.x;
		sumy+=gyro.y;
		sumz+=gyro.z;
		cnt++;
		return 1;
	}
	else
	{
		gyroB[0]-=sumx/50;
		gyroB[1]-=sumy/50;
		gyroB[2]-=sumz/50;
		sumx=sumy=sumz=0;
		cnt=0;
		return 0;
	}
}

/***********************
六轴融合互补滤波
**********************/
void IMUupdate(AxisInt acc,AxisInt gyro,float *rol,float *pit,float *yaw)
{
	float ax=acc.x,ay=acc.y,az=acc.z;  //归一化加速度计数据暂存
	static float q0=1,q1=0,q2=0,q3=0;
	static float exInt=0,eyInt=0;
	if(ax==0 && ay==0 && az==0) return;
	if(q0==0 && q1==0 && q2==0 && q3==0) return;
	float q0t=q0,q1t=q1,q2t=q2,q3t=q3;
	//重力加速度归一化
	float norm=Q_rsqrt(ax*ax+ay*ay+az*az);
	ax*=norm;ay*=norm;az*=norm;
	//提取四元数的等效余弦矩阵中的重力分量
	float vx=2*(q1*q3-q0*q2);
	float vy=2*(q0*q1+q2*q3);
	float vz=1-2*(q1*q1+q2*q2);
	//向量叉积得出姿态误差
	float ex=ay*vz-az*vy; 
	float ey=az*vx-ax*vz;
	//对误差进行积分
	exInt+=ex*Ki;
	eyInt+=ey*Ki;
	//姿态误差补偿到角速度上,修正角速度积分漂移
	float gx=GyroToRad(gyro.x)+Kp*ex+exInt;
	float gy=GyroToRad(gyro.y)+Kp*ey+eyInt;
	float gz=GyroToRad(gyro.z);
	//欧拉法数值求解四元数微分方程
	q0=q0t+hT*(-q1*gx-q2*gy-q3*gz);
	q1=q1t+hT*(q0*gx-q3*gy+q2*gz);
	q2=q2t+hT*(q3*gx+q0*gy-q1*gz);
	q3=q3t+hT*(-q2*gx+q1*gy+q0*gz);
	//四元数归一化
	norm=Q_rsqrt(q0*q0+q1*q1+q2*q2+q3*q3);
	q0*=norm;q1*=norm;q2*=norm;q3*=norm;
	//四元数转欧拉角
	*rol=Matan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2))*57.3f;
	*pit=Masin(2*(q0*q2-q1*q3))*57.3f;
	*yaw=Matan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))*57.3f;
}
