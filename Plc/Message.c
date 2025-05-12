/*******************************************************************************
* Filename: Message.c	                                             	 		   *
* Description:											   			 		   												*
* Author:                                                           		   *
* Date:     														 		   														*
* Revision:															 		 														  *
*******************************************************************************/
#include  "Message.h"
//��������
DataQueue MsgICOMQueue;
DataQueue MsgICANQueue;
tAlarm sysAlarm[64];	//ϵͳ�����б�

//��Ϣ�����ʼ��
void MsgInitialize(void)
{
	unsigned char i;

	QueueCreate(&MsgICOMQueue,Msg_ICOM_MAX);
	QueueCreate(&MsgICANQueue,Msg_ICAN_MAX);
	
	//���ϱ�����ʼ��
	for(i=0;i<(sizeof(sysAlarm)/sizeof(sysAlarm[0]));i++)
	{
		sysAlarm[i].ucMacID=i;
		sysAlarm[i].ucType=0;
	}	
}

//��Ϣ�������
void MsgFlush(void)
{
	QueueFlush(&MsgICANQueue);
}

//��Ϣ�¼�����	
void MsgManage(void)
{
}

//ICAN��Ϣע��
void MsgICANRegister(unsigned char ucMacID,unsigned char ucEvent)
{
	tMessage message;
	message.ucMsgType=Msg_Type_ICAN;
	message.ucMsgMacID=ucMacID;
	message.ucMsgEvent=ucEvent;
	QueueWrite(&MsgICANQueue,message);
}

//ICOM��Ϣע��	
void MsgICOMRegister(unsigned char ucEvent)
{
	tMessage message;
	message.ucMsgType=Msg_Type_ICOM;
	message.ucMsgMacID=0x00;
	message.ucMsgEvent=ucEvent;
	QueueWrite(&MsgICOMQueue,message);
}	

//��Ϣ���
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

//ICAN����ע��
void AlarmICANRegister(unsigned char ucMacID,unsigned char ucEvent)
{
    if(sysAlarm[ucMacID].ucType==0)	//�ޱ���ע��
	{
		sysAlarm[ucMacID].ucType=ucEvent;
	}
	//�б���ע�ᣬ����ǰ�������ȼ�����
//	else if((sysAlarm[ucMacID].ucType!=0)&&(ucEvent<sysAlarm[ucMacID].ucType))
	else if(sysAlarm[ucMacID].ucType!=0)
	{
		sysAlarm[ucMacID].ucType=ucEvent;
	}
}

//ICOM������Ϣע��	
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

//ϵͳ����ע��
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

//ICOM��Ϣ�����������ȳ���
tBoolean MsgICOMPop(tMessage *pMessage)
{
	if(QueueNData(&MsgICOMQueue)==0)
	{
		return false;
	}
	QueueRead(pMessage,&MsgICOMQueue);
	return true;
}

//ICAN��Ϣ�����������ȳ���
tBoolean MsgICANPop(tMessage *pMessage)
{
	if(QueueNData(&MsgICANQueue)==0)
	{
		return false;
	}
	QueueRead(pMessage,&MsgICANQueue);
	return true;
}
