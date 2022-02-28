#include "protocol.h"
/**************文件说明**********************
地面站和飞控通过串口进行数据通信
********************************************/

/*串口接收部分**********************************/

u8 RxData;  //从串口收到的一个字节
//以下变量供其它文件读取
u8 FcnWord;  //功能字节
u8 LenWord;  //长度字节
u8 RxTemp[12];  //临时保存串口接收到的待用数据
u8 GlobalStat=0;  //全局状态
u8 RcvCnt=0;  //待处理的数据帧个数

/***********************
建立DMA接收通道,从地面站/遥控器接收数据
**********************/
void Protocol_Init(void)
{
	HAL_UART_Receive_DMA(&huart2,&RxData,1);
}
/***********************
接收字节处理
**********************/
u8 XDAA_Data_Receive_Precess(void)
{
	static u8 RxState=0,sum=0,cnt=0;
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
		sum+=RxData;
		FcnWord=RxData;
		RxState=2;
		break;
	case 2:  //数据长度校验与保存
		if(RxData<=12)
		{
			sum+=RxData;
			LenWord=RxData;
			RxState=3;
		}
		else
		{
			sum=0;
			RxState=0;
		}
		break;
	case 3:  //临时保存待用数据
		sum+=RxData;
		RxTemp[cnt++]=RxData;
		if(cnt>=LenWord)
			RxState=4;
		break;
	case 4:  //匹配校验和
		RxState=0;
		cnt=0;
		if(sum==RxData)
			return 0;  //收到了正确的数据帧
		else
			return 2;  //数据帧出错
	default:
		RxState=0;
		cnt=0;
		return 2;  //数据帧出错
	}
	return 1;  //数据帧尚未接收完成
}
/***********************
串口通过DMA方式接收到一个字节
**********************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance!=USART2) return;
	if(XDAA_Data_Receive_Precess()) return;
	RcvCnt++;
}


/*串口发送部分**********************************/

u8 DataToSend[16];  //待发送的数据
u8 SendBuff[SENDBUF_SIZE];  //发送缓冲区
u8 SendBuff2[SENDBUF_SIZE];  //发送缓冲区2
u16 TotalLen=0;  //发送缓冲区待发送数据长度
/***********************
串口通过DMA方式接收到一个字节
**********************/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance!=USART2)	return;
	GlobalStat&=~TX_BUSY;
}
/***********************
将待发送数据存入缓冲
**********************/
void DMA_Stuff(u8 *Data,u8 len)
{
	for(u8 i=0;i<len;i++)
	{
		if(TotalLen+i>=SENDBUF_SIZE)
			return;  //若发送速率不够快导致缓冲区满则放弃新的数据
		SendBuff2[TotalLen+i]=Data[i];
	}
	TotalLen+=len;
}
/***********************
将发送缓冲区的数据填入DMA发送并清空发送缓冲区
**********************/
void Total_Send(void)
{
	if(TotalLen==0)	return;
	if(GlobalStat & TX_BUSY) return;
	for(u8 i=0;i<TotalLen;i++)
		SendBuff[i]=SendBuff2[i];
	HAL_UART_Transmit_DMA(&huart2,SendBuff,TotalLen);
	GlobalStat|=TX_BUSY;
	TotalLen=0;
}
/***********************
*@data:s16型数据
*@len:数据个数
*@fcn:功能字
**********************/
void XDAA_Send_S16_Data(s16 *data,u8 len,u8 fcn)
{
	u8 i,cnt=0,checksum=0;
	DataToSend[cnt++]='>';
	DataToSend[cnt++]=fcn;
	DataToSend[cnt++]=len*2;
	for(i=0;i<len;i++)
	{
		DataToSend[cnt++]=BYTE1(data[i]);
		DataToSend[cnt++]=BYTE0(data[i]);
	}
	for(i=0;i<cnt;i++)
		checksum+=DataToSend[i];
	DataToSend[cnt++]=checksum;
	DMA_Stuff(DataToSend,cnt);
}
/***********************
*@data:u8型数据
*@len:数据个数
*@fcn:功能字
**********************/
void XDAA_Send_U8_Data(u8 *data,u8 len,u8 fcn)
{
	u8 i,cnt=0,checksum=0;
	DataToSend[cnt++]='>';
	DataToSend[cnt++]=fcn;
	DataToSend[cnt++]=len;
	for(i=0;i<len;i++)
		DataToSend[cnt++]=data[i];
	for(i=0;i<cnt;i++)
		checksum+=DataToSend[i];
	DataToSend[cnt++]=checksum;
	DMA_Stuff(DataToSend,cnt);
}
/***********************
*@data:s32型数据
*@len:数据个数
*@fcn:功能字
**********************/
void XDAA_Send_HighSpeed_Data(short x,short y)
{
	u8 i,cnt=0,checksum=0;
	DataToSend[cnt++]='@';
	DataToSend[cnt++]=BYTE1(x);
	DataToSend[cnt++]=BYTE0(x);
	DataToSend[cnt++]=BYTE1(y);
	DataToSend[cnt++]=BYTE0(y);
	for(i=0;i<cnt;i++)
		checksum+=DataToSend[i];
	DataToSend[cnt++]=checksum;
	DMA_Stuff(DataToSend,cnt);
}
