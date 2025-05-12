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

#define		CAN_MAX_Buffer		64	//CAN��������󳤶�	

//����CAN�����շ���������ʶ��
#define FULL                    2
#define EMPTY                   3
#define NOT_FULL                4
#define NOT_EMPTY               5

//����CAN���Ľṹ��/
typedef	 struct _tCANFrame 
{
		unsigned long ulID;				//CAN����ID
    unsigned char ucXID;			//0 ��׼֡��1 ��չ֡  
    unsigned char ucDataLength;		//���ݳ�����
    unsigned char ucData[8];    	//�������ݳ�
}tCANFrame;

//����CAN ��������ջ�����
typedef	 struct _tCANFrmBuffer 
{
	signed int ucWrite;	    	//��ǰдλ��
	signed int ucRead;	    	//��ǰ��λ��
	tCANFrame   canFrmData[CAN_MAX_Buffer];	//�������ݴ洢��
}tCANFrmBuffer;

//��������
extern tCANFrmBuffer canFrmRxBuffer;
extern tCANFrmBuffer canFrmTxBuffer;

//��������
extern void CANInitialize(unsigned char ucBand);	//��ʼ��CAN�ڵ�,���ýڵ㲨����
extern void CANAcceptFilterSet (void);	//���ýڵ�������չ���	
extern unsigned char CANFrmSend(tCANFrmBuffer  *pBuffer);	//
extern unsigned char CANReadBuffer (tCANFrmBuffer  *pBuffer, tCANFrame  *pCANFrame);
extern unsigned char CANWriteBuffer (tCANFrmBuffer  *pCanFrmbuffer, tCANFrame  *pCANFrame);
extern void CANIntHandler(void);

#endif //_CANSTM32F4_H
