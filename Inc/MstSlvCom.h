/*******************************************************************************
* Filename: MstSlvCom.h	                                             	 	   *
* Description:	Can收发功能										   			   *
* Author:  QExpand, Lilu                                                       *
* Date: 2023/5/12    														   *
* Revision:	V1.00														 	   *
*******************************************************************************/

#ifndef _MSTSLVCOM_H_
#define _MSTSLVCOM_H_

#include "stdint.h"
#include "NetTimer.h"

#define	MSTSLVCOM_PERIOD			TIMER_PLC_PERIOD
#define	MST_REV_TIMEOUT				(200 / MSTSLVCOM_PERIOD)


#define	MST_REV_HEAD				0xA5
#define	MST_REV_LENGTH				0x0A
#define MST_REV_PARA_LENTH		0x08	

#define	MST_SEND_HEAD				0xAA
#define	MST_SEND_LENGTH				0x06


//typedef enum
//{
//	ServoOnNo = 0,
//	PowerLineOnNo,
//	BrakeReqNo,
//	ForwardReqNo,
//	BackwardReqNo,
//	LiftUpReqNo,
//	LiftDownReqNo,
//	MstSlvBitMax,
//}eMstSlvBitNo;

typedef union
{
	uint8_t buf[10];
	struct
	{
		/*Byte0*/
		uint8_t b1SvonState: 1;
		uint8_t b1MainDriver: 1;
		uint8_t b1Ebrake: 1;
		uint8_t b1Driver3State: 1;
		uint8_t b4Reserve1: 4;
		/*Byte1*/
		uint8_t b6Reserve2: 7;
		uint8_t b1ToggleBit: 1;
		/*Byte2~3*/
		uint8_t	u8SpeedFdbLow;
		uint8_t u8SpeedFdbHigh;
		/*Byte4~5*/
		uint8_t u8CurrentLow;
		uint8_t u8CurrentHigh;
		/*Byte6*/
		uint8_t u8ErrCode;
		/*Byte7*/
		uint8_t u8MotorTmp;
		/*Byte8*/
		uint8_t u8BoardTmp;
		/*Byte9*/
		uint8_t u8Reserve3;
	};
}xMstRevPara;


typedef void (*MstRevCallBackt)(xMstRevPara*);

typedef union
{
	uint8_t buf[9];
	struct
	{
		uint8_t u8Head;			/*Head*/
		uint8_t u8Length;		/*Length*/
		/*Byte0*/
		uint8_t b1ServoOn: 1;
		uint8_t b1PowerLineOn: 1;
		uint8_t b1BrakeReq: 1;
		uint8_t b1ForwardReq: 1;
		uint8_t b1BackwardReq: 1;
		uint8_t b1LiftReq: 1;
		uint8_t b1DownReq: 1;
		uint8_t b1EmsReq: 1; 
		/*Byte1*/
#ifdef Differential_SANZHILIU
		uint8_t b1LeftReq: 1;
		uint8_t b1RightReq: 1;
		uint8_t b5Reserve2: 5;
#else
		uint8_t b7Reserve2: 7;
#endif
		uint8_t b1ToggleBit: 1;
		/*Byte2~3*/
		uint8_t	u8TargetLow;
		uint8_t u8TargetHigh;
		/*Byte4*/
		uint8_t u8PumpTarget;
		/*Byte5*/
		uint8_t u8ErrCode;
		
		uint8_t u8CheckSum;		/*CheckSum*/
	};
}xMstSendPara;


typedef void (*MstSendCallBackt)(xMstSendPara*);
typedef void (*RevMcuParaCallBackt)(uint8_t *u8Data, uint16_t u16Length);

typedef struct
{
	xMstRevPara  MstRevData;
	xMstSendPara MstSendData;
	MstRevCallBackt  MstRevCallBack;
	MstSendCallBackt MstSendCallBack;	
	RevMcuParaCallBackt RevMcuParaCallBack;
}xMstSlvProc;



extern void vMstSlvComProc(void);
//extern void vMstSlvSetMotorVal(uint16_t u16Val);
//extern void vMstSlvSetPumpVal(uint8_t u8Val);
extern void vMstRevRegister(MstRevCallBackt CallBack);
extern void vMstSendRegister(MstSendCallBackt CallBack);
extern void vMcuParaRevRegister(RevMcuParaCallBackt CallBack);
extern void vQueryMcuPara(uint8_t *u8Data, uint16_t u16ength);
//extern void vMstSlvSetBit(eMstSlvBitNo MstSlvBitNo, uint8_t u8Val);

#endif
