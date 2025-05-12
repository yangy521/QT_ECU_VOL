/*******************************************************************************
* Filename: Message.c	                                             	 		   *
* Description:											   			 		   												*
* Author:                                                           		   *
* Date:     														 		   														*
* Revision:															 		 														  *
*******************************************************************************/
#include  "Message.h"
//变量声明
DataQueue MsgICOMQueue;
DataQueue MsgICANQueue;
tAlarm sysAlarm[64];	//系统报警列表

//消息处理初始化
void MsgInitialize(void)
{
	unsigned char i;

	QueueCreate(&MsgICOMQueue,Msg_ICOM_MAX);
	QueueCreate(&MsgICANQueue,Msg_ICAN_MAX);
	
	//故障报警初始化
	for(i=0;i<(sizeof(sysAlarm)/sizeof(sysAlarm[0]));i++)
	{
		sysAlarm[i].ucMacID=i;
		sysAlarm[i].ucType=0;
	}	
}

//消息队列清空
void MsgFlush(void)
{
	QueueFlush(&MsgICANQueue);
}

//消息事件处理	
void MsgManage(void)
{
}

//ICAN消息注册
void MsgICANRegister(unsigned char ucMacID,unsigned char ucEvent)
{
	tMessage message;
	message.ucMsgType=Msg_Type_ICAN;
	message.ucMsgMacID=ucMacID;
	message.ucMsgEvent=ucEvent;
	QueueWrite(&MsgICANQueue,message);
}

//ICOM消息注册	
void MsgICOMRegister(unsigned char ucEvent)
{
	tMessage message;
	message.ucMsgType=Msg_Type_ICOM;
	message.ucMsgMacID=0x00;
	message.ucMsgEvent=ucEvent;
	QueueWrite(&MsgICOMQueue,message);
}	

//消息添加
void MsgAdd(unsigned char ucType,tMessage message)
{
	if(ucType==Msg_Type_ICOM)
	{
		QueueWrite(&MsgICOMQueue,message);
	}
	else if(ucType==Msg_Type_ICAN)
	{
		QueueWrite(&MsgICANQueue,message);		
	}	
}

//ICAN报警注册
void AlarmICANRegister(unsigned char ucMacID,unsigned char ucEvent)
{
    if(sysAlarm[ucMacID].ucType==0)	//无报警注册
	{
		sysAlarm[ucMacID].ucType=ucEvent;
	}
	//有报警注册，但当前报警优先级更高
//	else if((sysAlarm[ucMacID].ucType!=0)&&(ucEvent<sysAlarm[ucMacID].ucType))
	else if(sysAlarm[ucMacID].ucType!=0)
	{
		sysAlarm[ucMacID].ucType=ucEvent;
	}
}

//ICOM报警消息注册	
void AlarmICOMRegister(unsigned char ucEvent)
{
	if(sysAlarm[0].ucType==0)
	{
		sysAlarm[0].ucType=ucEvent;
	}
	else if((sysAlarm[0].ucType!=0)&&(ucEvent<sysAlarm[0].ucType))
	{
		sysAlarm[0].ucType=ucEvent;
	}
}

//系统报警注册
void AlarmSysRegister(unsigned char ucEvent)
{
	if(sysAlarm[0].ucType==0)
	{
		sysAlarm[0].ucType=ucEvent;
	}
	else if((sysAlarm[0].ucType!=0)&&(ucEvent<sysAlarm[0].ucType))
	{
		sysAlarm[0].ucType=ucEvent;
	}
}	

//ICOM消息弹出（先入先出）
tBoolean MsgICOMPop(tMessage *pMessage)
{
	if(QueueNData(&MsgICOMQueue)==0)
	{
		return false;
	}
	QueueRead(pMessage,&MsgICOMQueue);
	return true;
}

//ICAN消息弹出（先入先出）
tBoolean MsgICANPop(tMessage *pMessage)
{
	if(QueueNData(&MsgICANQueue)==0)
	{
		return false;
	}
	QueueRead(pMessage,&MsgICANQueue);
	return true;
}
