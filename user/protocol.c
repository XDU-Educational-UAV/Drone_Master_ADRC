#include "protocol.h"
/**************文件说明**********************
地面站和飞控通过串口进行数据通信
********************************************/
//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

/*与串口硬件相关的程序*/
u8 RxData;
u8 RxTemp[14];  //临时保存串口接收到的待用数据
short CtrlCmd[4];
u8 RCcmd=0;
/**
建立DMA接收通道
*/
void Protocol_Init(void)
{
	HAL_UART_Receive_DMA(&huart2,&RxData,1);
}
/**
接收字节处理
*/
u8 XDAA_Data_Receive_Precess(void)
{
	static u8 RxState=0;
	static u8 sum=0,i=2;
	switch(RxState)
	{
	case 0:  //帧头校验
		if(RxData=='<')
		{
			sum=RxData;
			RxState=1;
		}
		break;
	case 1:  //功能字校验与保存
		if(RxData<=4)
		{
			sum+=RxData;
			RxTemp[0]=RxData;
			RxState=2;
		}
		else if(RxData=='<')  //从帧头重新开始
			sum=RxData;
		else
		{
			sum=0;
			RxState=0;
		}
		break;
	case 2:  //数据长度校验与保存
		if(RxState<=12)
		{
			sum+=RxData;
			RxTemp[1]=RxData;
			RxState=3;
		}
		else if(RxData=='<')  //从帧头重新开始
		{
			sum=RxData;
			RxState=1;
		}
		else
		{
			sum=0;
			RxState=0;
		}
		break;
	case 3:  //临时保存待用数据
		sum+=RxData;
		RxTemp[i++]=RxData;
		if(i>=RxTemp[1]+2)
			RxState=4;
		break;
	case 4:  //匹配校验和
		RxState=0;
		i=2;
		if(sum==RxData)
			return 0;  //待用数据有效
		break;
	default:break;
	}
	return 1;  //待用数据无效
}
/**
串口通过DMA方式接收到一个字节
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(XDAA_Data_Receive_Precess())
		return;
	u8 len=RxTemp[1]/2+1;
	switch(RxTemp[0])
	{
	case FLY_CTRL:
		for(u8 i=1;i<len;i++)
			CtrlCmd[i-1]=(RxTemp[2*i]<<8) | RxTemp[2*i+1];
		break;
	default:break;
	}
}

/*协议正式开始*/
u8 DataToSend[16];
/**
飞控->地面站 发送用户自定义帧
*@UserFrame:用户数据数组
*@len:数组长度(30以内)
*@fun:功能码(0xF1~0xFA)
*/
void XDAA_Send_User_Data(s16 *UserFrame,u8 len)
{
	u8 i,cnt=0,checksum=0;
	s16 temp;
	DataToSend[cnt++]='>';
	DataToSend[cnt++]=0xF1;
	DataToSend[cnt++]=len*2;
	for(i=0;i<len;i++)
	{
		temp = UserFrame[i];
		DataToSend[cnt++]=BYTE1(temp);
		DataToSend[cnt++]=BYTE0(temp);
	}
	for(i=0;i<DataToSend[2];i++)
		checksum+=DataToSend[i+2];
	DataToSend[cnt++]=checksum;
	XDAA_Send_Data(DataToSend,cnt);
}
