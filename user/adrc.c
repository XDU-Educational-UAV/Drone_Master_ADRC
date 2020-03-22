#include "adrc.h"
/**************文件说明**********************
自抗扰控制器相关函数,不一定所有的函数都能用上
附加一个与自抗扰控制无关的误差四元数计算函数
********************************************/
#define adrcR   4
#define adrcH   0.025
#define adrcD   0.0025
#define T       0.02

/**********************
离散系统最速控制综合函数(bug)
*@x1:二阶积分器的输出
*@x2:二阶积分器输出的微分
*@return:控制量输出
**********************/
float ADRC_fhan(float x1, float x2)
{
	float y1 = x1 + adrcH*x2;
	float y2 = x1 + 2 * adrcH*x2;
	if ((y1 >= -adrcD) && (y1 <= adrcD) && (y2 >= -adrcD) && (y2 <= adrcD))
		return -y2 / adrcH / adrcH;
	float u = (1 + Msqrt(1 + 8 * ABS(y1) / adrcD)) / 2 + 0.0001f;
	short k = (short)u;
	if (k > 2)
		u = (1 - k / 2.0f)*adrcR*SIGN(y1) - (x1 + k*adrcH*x2) / ((k - 1)*adrcH*adrcH);
	else if (k == 2)
		u = -adrcR / 2.0f*SIGN(y1) - (x1 + 3 * adrcH*x2) / 2 * adrcH*adrcH;
	else if (k == 1)
		u = -adrcR*SIGN(x2);
	return LIMIT(u, -adrcR, adrcR);
}

/**********************
跟踪微分器(bug)
*@u:输入
*@derivative:原信号的微分输出
*@return:原信号的滤波输出
**********************/
float ADRC_TD(float r,float *derivative)
{
	static float x1=0,x2=0;
	float u=ADRC_fhan(x1-r,x2);
	x1+=x2;
	x2+=u;
	*derivative=x2;
	return x1;
}

/**********************
非线性增益(bug)
**********************/
float ADRC_fal(float x)
{
	float y;
	if(ABS(x)<=0.02)
		y=x*0.1414213562;
	else
		y=Msqrt(ABS(x))*SIGN(x);
	return y;
}

/**********************
扩张状态观测器(bug)
**********************/
float ADRC_ESO(float u,float y,float b)
{
	static float z1=0,z2=0;
	float e=z1-y;
	z1+=b*u+z2-50*e;
	z2-=220*ADRC_fal(e);
	float w=z2/b;
	return w;
}

/**********************
线性扩张状态观测器
x1'=x2;x2'=-x2+bu+w;
*@u:输入信号
*@y:输出角速度
*@b:输入信号增益
*@z1:角速度x1的估计值(输入时应有初始值)
*@z2:角加速度x2的估计值(输入时应有初始值)
*@return:总扰动w
**********************/
float ADRC_LESO(float u,float y,float b,float *z1,float *z2)
{
	static float w=0;
	float e=y-*z1;
	*z1+=(*z2+6*e)*T;
	*z2+=(-1**z2+b*u+w+12*e)*T;
	w+=8*e*T;
	return w;
}

/**********************
求出姿态四元数pos到期望四元数exp之间的误差四元数
**********************/
Quaternion Quaternion_Error(Quaternion E,Quaternion P)
{
	Quaternion ans;
	ans.q0=E.q0*P.q0+E.q1*P.q1+E.q2*P.q2+E.q3*P.q3;
	ans.q1=P.q0*E.q1-E.q0*P.q1+E.q3*P.q2-E.q2*P.q3;
	ans.q2=P.q0*E.q2-E.q0*P.q2+E.q1*P.q3-E.q3*P.q1;
	ans.q3=P.q0*E.q3-E.q0*P.q3+E.q2*P.q1-E.q1*P.q2;
	return ans;
}
