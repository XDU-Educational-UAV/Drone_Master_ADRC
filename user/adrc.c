#include "adrc.h"
/**************文件说明**********************
自抗扰控制器
********************************************/

/**********************
线性扩张状态观测器
x1'=x2;x2'=b*u+w;
*@y:被跟踪的角速度
**********************/
void ADRC_LESO(ADRC_Param *adrc,float y)
{
	float e = y - adrc->SpeEst;
	adrc->SpeEst += (adrc->AccEst + adrc->L1 * e) * T;
	adrc->AccEst +=(adrc->B * adrc->u + adrc->w + adrc->L2 * e) * T;
	adrc->w += adrc->L3 * e * T;
}

/**********************
控制参数更新
**********************/
void ADRC_ParamUpdate(ADRC_Param *adrc)
{
	adrc->KpIn = adrc->wc * adrc->wc;
	adrc->KdIn = 2 * adrc->wc;
	adrc->L1 = 3 * adrc->wo;
	adrc->L2 = 3 * adrc->wo * adrc->wo;
	adrc->L3 = adrc->wo * adrc->wo * adrc->wo; 
}

/**********************
状态参数清零
**********************/
void ADRC_ParamClear(ADRC_Param *adrc)
{
	adrc->AttOut=0;
	adrc->SpeEst=0;
	adrc->AccEst=0;
	adrc->w=0;
	adrc->u=0;
}
