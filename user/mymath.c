#include "mymath.h"
/**************文件说明**********************
常用初等函数的快速算法
与库函数冲突的函数加前缀"M"
函数目录(括号内为定义域和值域):
Msin(x)     ([-pi/2,pi/2],[-1,1]);
Mcos(x)     ([-pi/2,pi/2],[-1,1]);
Matan(x)    ((-∞,∞),(-pi/2,pi/2));
Matan2(y,x) ((-∞,∞),(-pi/2,pi/2));
Masin(x)    ([-1,1],[-pi/2,pi/2]);
Msqrt,Q_rsqrt,Mpow,ABS,SIGN,MIN,MAX,SAT
********************************************/

//反正切函数.定义域(-∞,∞),值域(-pi/2,pi/2)
float Matan(float rad)
{
	if (rad < 0)
		return -Matan(-rad);
	if (rad <= 0.25)
	{
		float ans = rad*(0.47831170583472860f * rad - 1.1679191357523431f);
		return ans*(0.29922976727874773f * ans - 0.85630501107017942f);
	}
	if (rad <= 0.75)
		return 0.463647609f + Matan((rad - 0.5) / (1 + rad*0.5));
	if (rad <= 2)
		return 0.8760580506f + Matan((rad - 1.2) / (1 + rad*1.2));
	return 1.3258186637f + Matan((rad - 4) / (1 + rad * 4));
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
	x = number * 0.5F;
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
	x2 = number * 0.5F;
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
	else if(x<1)
		return Matan(x*Q_rsqrt(1-x*x));
	else
	{
		x=PI/2;
		return x;
	}
}

//正弦函数.定义域[-pi,pi],值域[-1,1]
float Msin(float rad)
{
	float ans;
	ans = rad*(1.27323954f - 0.405284735f * ABS(rad));
	ans = ans*(0.775 + 0.225*ABS(ans));
	return ans;
}

//指数函数
float Mexp(float x)
{
	long i = (long)(x * 12102203 + 1064872507);
	return *(float*)&i;
}
