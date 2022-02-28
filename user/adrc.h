#ifndef __ADRC_H
#define __ADRC_H

#include "mymath.h"

#define T       0.002f
//自抗扰控制器的控制参数与运行状态参数
//自抗扰控制器的设计较为灵活,使用的参数可能会频繁改动
typedef struct
{
	//控制参数
	float KpOut;     //外环姿态比例控制
	float KpIn;      //内环角速度比例控制
	float KdIn;      //内环角速度微分控制
	float B;         //扰动补偿增益
	//状态参数
	float AttOut;    //外环输出
	float SpeEst;    //角速度估计
	float AccEst;    //角加速度估计
	float w;         //总扰动
	float u;         //控制器最终输出
	float x1;        //跟踪微分器输出跟踪信号
	float x2;        //跟踪微分器输出微分信号
}ADRC_Param;

void ADRC_LESO(ADRC_Param *adrc,float y);
void ADRC_ParamClear(ADRC_Param *adrc);
void ADRC_TD(ADRC_Param* adrc);

#endif
