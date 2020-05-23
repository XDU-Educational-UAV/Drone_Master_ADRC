#ifndef __MYMATH_H__
#define __MYMATH_H__

typedef unsigned char u8;
#define PI                       3.14159265358979323846
#define Mcos(rad)                Msin(PI/2.0f-ABS(rad))
#define Macos(rad)               (PI/2+Masin(-(rad)))
#define Mln(x)                   (6.0f*((x)-1.0f)/((x)+1.0f+4.0f*Msqrt(x)))
#define ABS(x)                   ((x)>=0?(x):-(x))
#define SIGN(x)                  ((x)>=0?1:-1)
#define LIMIT(x,min,max)         (((x)<=(min) ? (min) : ((x)>=(max) ? (max) : (x))))
#define MIN(a,b)                 ((a)<(b)?(a):(b))
#define MAX(a,b)                 ((a)>(b)?(a):(b))

//µ¥Î»×ª»»:gyro[-16383,16384];deg[-90,90];rad[-PI/2,PI/2];pwm[0,1000];pwmAdd[-500,500]
#define DegToRad(x)           ((x)*0.0174532925)  //(x/57.3)
#define DegToGyro(x)          ((short)((x)*131.072))  //(x*2^15/250)
#define DegToPwmAdd(x)        ((x)/9.0*50.0+500)
#define DegToPwm(x)           ((x)/9.0*50.0)
#define GyroToDeg(x)          ((float)(x)*0.0076293945)  //(x*250/2^15)
#define GyroToRad(x)          ((float)(x)*1.331580545e-4)
#define PwmToDegAdd(x)        ((((float)(x))-500.0)*0.18)
#define PwmToDeg(x)           ((float)(x)*0.18)
#define PwmToRadAdd(x)        (((float)(x)-500.0)*0.00314159265)
#define RadToDeg(x)           ((x)*57.2957795131)   //(x*57.3)
#define RadToGyro(x)          ((short)((x)*7509.8724))  //(x*2^15/250*57.3)

float Matan(float rad);
float Matan2(float y,float x);
float Msqrt(float number);
float Q_rsqrt(float number);
float Masin(float x);
float Msin(float rad);
float Mexp(float x);
short moderate(short x,short T);
float IIR_LowPassFilter(float DataIn,float *delay);

#endif
