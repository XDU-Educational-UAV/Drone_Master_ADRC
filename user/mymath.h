#ifndef __MYMATH_H__
#define __MYMATH_H__

typedef unsigned char u8;
#define PI                       3.14159265358979323846
#define ABS(x)                   ((x)>=0?(x):-(x))
#define SIGN(x)                  ((x)>=0?1:-1)
#define LIMIT(x,min,max)         (((x)<=(min) ? (min) : ((x)>=(max) ? (max) : (x))))
#define Mcos(rad)                Msin(PI/2.0f-ABS(rad))
#define MIN(a,b)                 ((a)<(b)?(a):(b))
#define MAX(a,b)                 ((a)>(b)?(a):(b))
#define Mln(x)                   (6.0f*((x)-1.0f)/((x)+1.0f+4.0f*Msqrt(x)))

typedef struct
{
	float q0,q1,q2,q3;
}Quaternion;

float Matan(float rad);
float Matan2(float y,float x);
float Msqrt(float number);
float Q_rsqrt(float number);
float Masin(float x);
float Msin(float rad);
float Mexp(float x);
float IIR_LowPassFilter(float DataIn,float *delay);

#endif
