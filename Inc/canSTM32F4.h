/*******************************************************************************
* Filename: canSTM32F4.h 	                                    	     		   *
* Description: 							           *
* Author:                                                                      *
* Date: 															           *
* Revision:															           *																	           *
*******************************************************************************/
#ifndef _CANSTM32F4_H
#define _CANSTM32F4_H

#include		"KSDsys.h"

#define		CAN_MAX_Buffer		64	//CAN缓冲区最大长度	

//定义CAN数据收发缓冲区标识符
#define FULL                    2
#define EMPTY                   3
#define NOT_FULL                4
#define NOT_EMPTY               5

//定义CAN报文结构体/
typedef	 struct _tCANFrame 
{
		unsigned long ulID;				//CAN报文ID
    unsigned char ucXID;			//0 标准帧；1 扩展帧  
    unsigned char ucDataLength;		//数据场长度
    unsigned char ucData[8];    	//报文数据场
}tCANFrame;

//定义CAN 数据桢接收缓冲区
typedef	 struct _tCANFrmBuffer 
{
	signed int ucWrite;	    	//当前写位置
	signed int ucRead;	    	//当前读位置
	tCANFrame   canFrmData[CAN_MAX_Buffer];	//报文数据存储区
}tCANFrmBuffer;

//变量声明
extern tCANFrmBuffer canFrmRxBuffer;
extern tCANFrmBuffer canFrmTxBuffer;

//函数声明
extern void CANInitialize(unsigned char ucBand);	//初始化CAN节点,设置节点波特率
extern void CANAcceptFilterSet (void);	//设置节点接收验收过滤	
extern unsigned char CANFrmSend(tCANFrmBuffer  *pBuffer);	//
extern unsigned char CANReadBuffer (tCANFrmBuffer  *pBuffer, tCANFrame  *pCANFrame);
extern unsigned char CANWriteBuffer (tCANFrmBuffer  *pCanFrmbuffer, tCANFrame  *pCANFrame);
extern void CANIntHandler(void);

#endif //_CANSTM32F4_H
