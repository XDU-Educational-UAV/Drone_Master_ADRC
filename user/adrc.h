#ifndef __ADRC_H
#define __ADRC_H

#include "mymath.h"

//自抗扰控制器的控制参数与运行参数
//自抗扰控制器的设计较为灵活,使用的参数可能会频繁改动
typedef struct
{
	//控制参数
	float KpOut;     //外环位置比例控制
	float KpIn;      //内环速度比例控制
	float KdIn;      //内环加速度比例控制
	float Kw;        //扩张状态观测器补偿大小
	//运行参数
	float PosOut;    //位置误差输出
	float SpeEst;    //角速度的状态估计
	float AccEst;    //角加速度的状态估计
	float w;         //总扰动
	float u;         //控制器最终输出
}ADRC_Param;

typedef struct
{
	float depth;
	float theta;
	float delay[3];
}BSFilter;

float ADRC_fhan(float x1,float x2);
float ADRC_TD(float r,float *derivative);
float ADRC_fal(float x);
float ADRC_ESO(float u,float y,float b);
void ADRC_LESO(ADRC_Param *adrc,float y);

#endif
