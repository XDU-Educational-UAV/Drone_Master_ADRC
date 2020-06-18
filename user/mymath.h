#ifndef __MYMATH_H__
#define __MYMATH_H__

#define PI                       3.14159265f
#define Mcos(rad)                Msin(PI/2.0f-ABS(rad))
#define Macos(rad)               (PI/2+Masin(-(rad)))
#define Mln(x)                   (6.0f*((x)-1.0f)/((x)+1.0f+4.0f*Msqrt(x)))
#define ABS(x)                   ((x)>=0?(x):-(x))
#define SIGN(x)                  ((x)>=0?1:-1)
#define LIMIT(x,min,max)         (((x)<=(min) ? (min) : ((x)>=(max) ? (max) : (x))))
#define MIN(a,b)                 ((a)<(b)?(a):(b))
#define MAX(a,b)                 ((a)>(b)?(a):(b))

//µ¥Î»×ª»»:gyro[-16383,16384];deg[-90,90];rad[-PI/2,PI/2];pwm[0,1000];pwmAdd[-500,500]
#define DegToRad(x)           ((x)*0.0174532925f)  //(x/57.3)
#define DegToGyro(x)          ((short)((x)*32.768f))  //(x*2^15/1000)
#define DegToPwmAdd(x)        ((short)((x)/9.0f*50.0f+500))
#define DegToPwm(x)           ((short)((x)/9.0f*50.0f))
#define GyroToDeg(x)          ((float)(x)*0.0305176f)  //(x*1000/2^15)
#define GyroToRad(x)          ((float)(x)*5.3263222e-4f)
#define PwmToDegAdd(x)        ((((float)(x))-500.0f)*0.18f)
#define PwmToDeg(x)           ((float)(x)*0.18f)
#define PwmToRadAdd(x)        (((float)(x)-500.0f)*0.00314159265f)
#define RadToDeg(x)           ((x)*57.2957795131f)   //(x*57.3)
#define RadToGyro(x)          ((short)((x)*1877.468103f))  //(x*2^15/250*57.3)

float Matan(float rad);
float Matan2(float y,float x);
float Msqrt(float number);
float Q_rsqrt(float number);
float Masin(float x);
float Msin(float rad);
float Mexp(float x);
float IIR_LowPassFilter(float DataIn,float *delay);
float OneOrder_Filter(float DataIn,float delay);

#endif
