# 小四轴飞控

[![Codacy Badge](https://api.codacy.com/project/badge/Grade/9380144ef91447e3a5b0288f9083182a)](https://app.codacy.com/gh/uav-operation-system/Drone_Master_ADRC?utm_source=github.com&utm_medium=referral&utm_content=uav-operation-system/Drone_Master_ADRC&utm_campaign=Badge_Grade_Dashboard)

version V1.02

         HEAD
	 M2    ↑     M3

	   \         /

	     \     /

	       \ /

	       / \

	     /     \

	   /         \

	 M1          M4

电机M1至M4分别为逆顺逆顺。

默认为姿态模式,可通过地面站进行更改。

![chip3d](github.com/xd15zhn/drone/blob/master/chip3d.png)

[地面站](https://github.com/xd15zhn/GroundStation/blob/master/GroundStation/bin/Release/GroundStation.exe)

## 代码说明

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

解锁后2秒内未接收到正确的遥控信号自动锁定。

起飞前建议进行传感器校准。

## bug与隐患
