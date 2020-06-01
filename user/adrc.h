#ifndef __ADRC_H
#define __ADRC_H

#include "mymath.h"

#define T       0.002f
//自抗扰控制器的控制参数与运行参数
//自抗扰控制器的设计较为灵活,使用的参数可能会频繁改动
typedef struct
{
	//控制参数
	float KpOut;     //外环姿态比例控制
	float KpIn;      //内环角速度比例控制
	float KiIn;      //内环角速度积分控制
	float KdIn;      //内环角速度微分控制
	float Kw;        //扰动补偿增益
	//运行参数
	float AttOut;    //外环输出
	float SpeErr;    //内环角速度期望输入
	float SpeInt;    //内环角速度期望输入积分
	float SpeEst;    //角速度估计
	float AccEst;    //角加速度估计
	float w;         //总扰动
	float u;         //控制器最终输出
	//IIR滤波使用
	float AccDelay;  //角加速度
}ADRC_Param;

void ADRC_LESO(ADRC_Param *adrc,float y);
void ADRC_TD(ADRC_Param* adrc);
float ADRC_fal_5(float x);
float ADRC_fal_25(float x);
float ADRC_fal_1_5(float x);

#endif
