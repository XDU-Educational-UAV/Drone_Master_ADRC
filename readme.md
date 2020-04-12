# 小四轴飞控
<<<<<<< HEAD
version V0.11
=======
version V0.12

         HEAD
	  M2  ↑  M3

	   \     /

	    \   /

	     \ /

	     / \

	    /   \

	   /     \

	  M1     M4
	
默认为姿态模式,可通过地面站进行更改。

[!地面站](https://github.com/xd15zhn/GroundStation/blob/master/GroundStation/bin/Release/GroundStation.exe)
>>>>>>> da2d206e00a8ed6c01b93e7eecc931562607cc3d

## 代码说明

### 关于偏航
偏航方向为开环控制,即完全放弃偏航方向的反馈

{task.c, IMU_Processing()}将读取到的偏航角速度置零

{imu.c}偏航角速度校准值为0

如果考虑偏航方向需注意:

1. 是速度反馈还是姿态反馈;

2. 四元数q3的变化.

### 用户代码
尽量不改动自动生成的代码文件，用户添加的代码文件在user文件夹中，与自动生成的代码文件分离。

自动生成的代码文件中加入的用户代码包括:

1. main.c:初始化,死循环,定时器3任务调度

2. main.h:数据类型定义,如u8,u16,u32等;IO端口定义,如LED1_PORT,STAT_PORT等

3. adc.c:测电池电压函数

用户代码文件中存在的底层代码包括:

1. mpuiic.c mpuiic.h:

2. protocol.c:串口发送和接收完成回调函数

### 变量/函数/宏定义命名格式
变量:全部小写或首字母大写 xxx/Xxx/XxxXxx

函数:首字母大写加下划线分隔 Xxx_Xxx()

宏定义:全部大写加下划线分隔 XXX_XXX

### 控制链路
控制协议见 https://github.com/xd15zhn/GroundStation

锁定与解锁均需要上位机发送正确的指令。

起飞后（即油门超过一定值）2秒内未接收到正确的遥控信号或飞机姿态倾角过大，将进入失控保护模式尝试平稳降落，重新收到信号或姿态恢复正常则可随时恢复控制。

## bug与隐患
<<<<<<< HEAD
* 功能基本完整,但参数未调整。
=======
*速度模式参数基本调整完毕,姿态模式由于四元数问题而待调整
>>>>>>> da2d206e00a8ed6c01b93e7eecc931562607cc3d
