#include "mymath.h"
/**************文件说明**********************
常用初等函数的快速算法
与库函数冲突的函数加前缀"M"
函数目录(括号内为定义域和值域):
正弦函数      Msin(x)     [-pi/2,pi/2]    [-1,1]
余弦函数      Mcos(x)     [-pi/2,pi/2]    [-1,1]
反正切函数1   Matan(x)    (-∞,∞)          (-pi/2,pi/2)
反正切函数2   Matan2(y,x) (-∞,∞)          (-pi/2,pi/2)
反正弦函数    Masin(x)    [-1,1]          [-pi/2,pi/2]
反余弦函数    Macos(x)    [-1,1]          [0,pi]
平方根函数    Msqrt       [0,∞)           [0,∞)
平方根倒数    Q_rsqrt     (0,∞)           (0,∞)
自然指数函数  Mexp        (-∞,∞)          (0,∞)
自然对数函数  Mln         (0,∞)           (-∞,∞)
绝对值        ABS
符号          SIGN
饱和          LIMIT
较小值        MIN
较大值        MAX
低通滤波器    IIR_LowPassFilter
********************************************/

//反正切函数.定义域(-∞,∞),值域(-pi/2,pi/2)
float Matan(float rad)
{
	if (rad < 0)
		return -Matan(-rad);
	if (rad <= 0.25f)
	{
		float ans = rad*(0.47831170583472860f * rad - 1.1679191357523431f);
		return ans*(0.29922976727874773f * ans - 0.85630501107017942f);
	}
	if (rad <= 0.75f)
		return 0.463647609f + Matan((rad - 0.5f) / (1.0f + rad*0.5f));
	if (rad <= 2.0f)
		return 0.8760580506f + Matan((rad - 1.2f) / (1.0f + rad*1.2f));
	return 1.3258186637f + Matan((rad - 4.0f) / (1.0f + rad * 4.0f));
}

//360°反正切.定义域(-∞,∞),值域(-pi,pi]
float Matan2(float y,float x)
{
	if (x > 0)
		return Matan(y / x);
	if (x < 0)
		return (y >= 0) ? (Matan(y / x) + PI) : (Matan(y / x) - PI);
	return (y > 0) ? (PI / 2) : ((y < 0) ? (-PI / 2) : 0);
}

//快速平方根算法
float Msqrt(float number)
{
	long i;
	float x, y;
	x = number * 0.5f;
	y = number;
	i = *(long *)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (1.5f - (x * y * y));
	y = y * (1.5f - (x * y * y));
	return number * y;
}

//快速平方根倒数算法
float Q_rsqrt(float number)
{
	long i;
	float x2, y;
	x2 = number * 0.5f;
	y  = number;
	i  = * ( long * ) &y;
	i  = 0x5f3759df - ( i >> 1 );
	y  = * ( float * ) &i;
	y  = y * ( 1.5f - ( x2 * y * y ) );
	y  = y * ( 1.5f - ( x2 * y * y ) );
	return y;
}

//反正弦函数.定义域[-1,1],值域[-pi/2,pi/2]
float Masin(float x)
{
	if(x<0)
		return -Masin(-x);
	else if(x<1.0f)
		return Matan(x*Q_rsqrt(1.0f-x*x));
	else
	{
		x=PI/2.0f;
		return x;
	}
}

//正弦函数.定义域[-pi,pi],值域[-1,1]
float Msin(float rad)
{
	float ans;
	ans = rad*(1.27323954f - 0.405284735f * ABS(rad));
	ans = ans*(0.775f + 0.225f*ABS(ans));
	return ans;
}

//指数函数
float Mexp(float x)
{
	long i = (long)(x * 12102203 + 1064872507);
	return *(float*)&i;
}

/***********************
二阶IIR低通滤波，直接II型结构
*@delay:需要暂存3个状态变量的存储空间
*@DataIn:每次新增的数据
输出滤波后的新增数据
**********************/
float IIR_LowPassFilter(float DataIn,float *delay)
{
	delay[0] = DataIn + 1.7f*delay[1] - 0.7325f*delay[2];
	float DataOut = (delay[0] + 2*delay[1] + delay[2]) * 0.008125f;
	delay[2] = delay[1];
	delay[1] = delay[0];
	return DataOut;
}

/***********************
一阶IIR低通滤波
*@delay:需要暂存1个状态变量的存储空间
*@DataIn:每次新增的数据
输出滤波后的新增数据
**********************/
float OneOrder_Filter(float DataIn,float delay)
{
	float DataOut = DataIn + 0.25f * delay;
	delay = DataOut;
	return DataOut * 0.75f;
}
