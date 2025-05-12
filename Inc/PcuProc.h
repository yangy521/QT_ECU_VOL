/*******************************************************************************
* Filename: PcuProc.h	                                             	 	   *
* Description:Pcu通信处理											   		   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/5/12    														   *
* Revision:															 		   *
*******************************************************************************/

#ifndef _PCUPROC_H_
#define _PCUPROC_H_

#include "stdint.h"
#include "Userdef.h"
#include "NetTimer.h"

#define WAIT_HEAD	 			0
#define WAIT_DATA		 		1
#define WAIT_CHECKSUM			2

#define PCU_START				0		/*Send	0xA5*/
#define	PCU_READY				1		/*Rev	0xA501*/
#define	PCU_QUERY				2		/*Send	0x1A*/
#define	PCU_REV					3		/*Rev	Pcu Data FrameHead:0x1A, total 9 Bytes*/
#define	PCU_SEND				4		/*Send	ErrCode total 8 Bytes*/
#define	PCU_WAIT_ONCE			5		/*For 50ms*/

#if (PCU_TYPE_NONE != PCU_TYPE)

#if (PCU_TYPE_LZ == PCU_TYPE)

#define	PCU_STATE_SUCCESS		1
#define	PCU_STATE_FAILED		0		

#define PCU_QUERY_FRAME			0x1A	/*Pcu Query Command*/
#define PCU_START_FRAME			0xA6	/*Pcu Start Command*/
#define PCU_HEAD				0x13	/*Pcu RevFrame Head*/

#define	PCU_PERIOD				TIMER_PLC_PERIOD
#define PCU_TIMEROUT			(50 / PCU_PERIOD)
#define	PCU_TIMERCOM_PRIOD		500
#define	PCU_COM_FAIL			5
#define	PCU_INIT_FAIL			60

#define	PCU_READY_BYTE1			0xA5
#define	PCU_READY_BYTE2			0x01

typedef union
{
	uint8_t buf[8];
	struct
	{
		/*1st Byte*/
		uint8_t u8Head;
		/*2nd Byte*/
		uint8_t b1Mode: 1;
		uint8_t b1Reserve0: 1;
		uint8_t b1SlowSpdSwitch: 1;/*龟速按键*/
		uint8_t b1Reserve1: 1;
		uint8_t b4Const1: 4;
		/*3rd Byte*/
		uint8_t b1SpeakerSwitch: 1;/*喇叭按键*/
		uint8_t b1Reserve2: 1;		
		uint8_t b1LiftingSwitch: 1;	/*起升按键*/
		uint8_t b1TraSwitch: 1;			/*行走按键*/

		uint8_t b4Const2: 4;
		/*4th Byte*/
		uint8_t b4HandleXHigh: 4;
		uint8_t b4Const3: 4;
		/*5th Byte*/
		uint8_t b4HandleXLow: 4;
		uint8_t b4Const4: 4;
		/*6th Byte*/
		uint8_t b1EnableSwitch: 1;		/*使能按键*/
		uint8_t b1TurnRightSwitch: 1;	/*右转*/
		uint8_t b1TurnLeftSwitch: 1;	/*左转*/
		uint8_t b1Reserve3: 1;
		uint8_t b4Const5: 4;
		/*7th Byte*/
		uint8_t b4HandleCtrlHigh: 4;	/*手柄高四位，int类*/
		uint8_t b4Const6: 4;
		/*8th Byte*/
		uint8_t b4HandleCtrlLow: 4;		/*手柄低四位*/
		uint8_t b4Const7: 4;
		/*9th Byte*/
		//uint8_t u8CheckSum;
	}Data;
}xPcuRevPara;


typedef void (*PcuRevCallBackt)(xPcuRevPara*);

typedef union
{
	uint8_t buf[8];
	struct
	{
		/*1st Byte*/
		uint8_t b4ErrCodeSigLHigh4: 4;
		uint8_t b4Const1: 4;
		/*2nd Byte*/
		uint8_t b4ErrCodeSigLLow4: 4;/*组合成左边数码管显示*/
		uint8_t b4Const2: 4;
		/*3rd Byte*/
		uint8_t b4ErrCodeSigRHigh4: 4;/*组合成右边数码管显示*/
		uint8_t b4Const3: 4;
		/*4th Byte*/
		uint8_t b4ErrCodeSigRLow4: 4;
		uint8_t b4Const4: 4;
		/*5th Byte*/
		uint8_t b1Reserve1: 1;
		uint8_t b1LiftLed: 1;		/*起升灯亮*/
		uint8_t b1Reserve2: 1;	/*行走灯亮*/
		uint8_t b1ModeLed: 1;
		uint8_t b4Const5: 4;
		/*6th Byte*/
		uint8_t b3Reserve3: 3;
		uint8_t b1Beep: 1;			/*手柄上喇叭响*/
		uint8_t b4Const6: 4;
		/*7th Byte*/
		uint8_t b1SlowLed: 1;			/*龟速灯亮*/
		uint8_t b3Reserve4: 3;
		uint8_t b4Const7: 4;
		/*8th Byte*/
		uint8_t b4ErrCodeSigLHigh5: 4;
		uint8_t b4Const8: 4;
	}Data;
}xPcuSendPara;

typedef void (*PcuSendCallBackt)(xPcuSendPara*);

typedef enum
{
	PCU_Init = 0,
	PCU_LiftKeyPress,
	PCU_SlowKeyPress,
	PCU_MoveKeyPress,
	PCU_TurnLeftPress,
	PCU_TurnRightPress,
	PCU_EnableKeyPress,
	PCU_ValueNoZero,
	PCU_SpeakerPress,
}ePcuInitErr;

typedef void (*PcuErrCallBackt)(ePcuInitErr);

typedef struct
{
	xPcuRevPara PcuRevData;
	xPcuSendPara PcuSendData;
	PcuRevCallBackt PcuRevCallBack;	
	PcuSendCallBackt PcuSendCallBack;	
	PcuErrCallBackt PcuErrCallBack;
}xPcuProc;

#elif(PCU_TYPE_2 == PCU_TYPE)
#define	PCU_STATE_SUCCESS		1
#define	PCU_STATE_FAILED		0		

#define PCU_QUERY_FRAME			0x1A	/*Pcu Query Command*/
#define PCU_START_FRAME			0xA5	/*Pcu Start Command*/
#define PCU_HEAD				0x15	/*Pcu RevFrame Head*/

#define	PCU_PERIOD				5
#define PCU_TIMEROUT			(50 / PCU_PERIOD)
#define	PCU_COM_FAIL			5
#define	PCU_INIT_FAIL			60

#define	PCU_READY_BYTE1			0xA5
#define	PCU_READY_BYTE2			0x01

typedef union
{
	uint8_t buf[8];
	struct
	{
		/*1st Byte*/
		uint8_t u8Head;
		/*2nd Byte*/
		uint8_t b1Mode: 1;
		uint8_t b1Reserve0: 1;
		uint8_t b1SlowSpdSwitch: 1;
		uint8_t b5Reserve1: 5;
		/*3rd Byte*/
		uint8_t b1SpeakerSwitch: 1;
		uint8_t b1EnableSwitchNeg: 1;
		uint8_t b1TraSwitch: 1;
		uint8_t b1LiftingSwitch: 1;
		uint8_t b4Reserve2: 4;
		/*4th Byte*/
		uint8_t u8Reserve3;
		/*5th Byte*/
		uint8_t u8Reserve4;
		/*6th Byte*/
		uint8_t b1EnableSwitch: 1;
		uint8_t b1TurnRightSwitch: 1;
		uint8_t b1TurnLeftSwitch: 1;
		uint8_t b5Reserve5: 5;
		/*7th Byte*/
		uint8_t u8HandleCtrlHigh;
		/*8th Byte*/
		uint8_t u8HandleCtrlLow;
		/*9th Byte*/
		//uint8_t u8CheckSum;
	}Data;
}xPcuRevParat;


typedef void (*PcuCallBackt)(xPcuRevParat*);

typedef struct
{
	xPcuRevParat PcuRevData;
	PcuCallBackt PcuCallBack;	
}xPcuRevProc;

typedef union
{
	uint8_t buf[8];
	struct
	{
		/*1st Byte*/
		uint8_t b4ErrCodeSigLHigh4: 4;
		uint8_t b4Const1: 4;
		/*2nd Byte*/
		uint8_t b4ErrCodeSigLLow4: 4;
		uint8_t b4Const2: 4;
		/*3rd Byte*/
		uint8_t b4ErrCodeSigRHigh4: 4;
		uint8_t b4Const3: 4;
		/*4th Byte*/
		uint8_t b4ErrCodeSigRLow4: 4;
		uint8_t b4Const4: 4;
		/*5th Byte*/
		uint8_t b1Reserve1: 1;
		uint8_t b1LiftLed: 1;
		uint8_t b1Reserve2: 1;
		uint8_t b1ModeLed: 1;
		uint8_t b4Const5: 4;
		/*6th Byte*/
		uint8_t b3Reserve3: 3;
		uint8_t b1Beep: 1;
		uint8_t b4Const6: 4;
		/*7th Byte*/
		uint8_t b1SlowLed: 1;
		uint8_t b3Reserve4: 3;
		uint8_t b4Const7: 4;
		/*8th Byte*/
		uint8_t b4ErrCodeSigLHigh5: 4;
		uint8_t b4Const8: 4;
	}Data;
}xPcuSendParat;
#elif(PCU_TYPE_3 == PCU_TYPE)
#elif(PCU_TYPE_XUGONG == PCU_TYPE)
#define	PCU_STATE_SUCCESS		1
#define	PCU_STATE_FAILED		0		

#define PCU_QUERY_FRAME			0x1A	/*Pcu Query Command*/
#define PCU_START_FRAME			0xA0	/*Pcu Start Command*/
#define PCU_HEAD				0x15	/*Pcu RevFrame Head*/

#define	PCU_PERIOD				TIMER_PLC_PERIOD
#define PCU_TIMEROUT			(50 / PCU_PERIOD)
#define	PCU_TIMERCOM_PRIOD		500
#define	PCU_COM_FAIL			5
#define	PCU_INIT_FAIL			60

#define	PCU_READY_BYTE1			0xA0
#define	PCU_READY_BYTE2			0x01

typedef union
{
	uint8_t buf[8];
	struct
	{
		/*1st Byte*/
		uint8_t u8Head;
		
		/*2nd Byte*/
		uint8_t b1Mode: 1;
		uint8_t b1Reserve0: 1;
		uint8_t b1SlowSpdSwitch: 1;
		uint8_t b1Reserve1: 1;
		uint8_t b4Const1: 4;
		
		/*3rd Byte*/
		uint8_t b1SpeakerSwitch: 1;
		uint8_t b1Reserve2: 1;
		uint8_t b1LiftingSwitch: 1;	
		uint8_t b1TraSwitch: 1;
		uint8_t b4Const2: 4;
		
		/*4th Byte*/
		uint8_t b4Reserve3:4;
		uint8_t b4Const3: 4;
		
		/*5th Byte*/		
		uint8_t b4Reserve4:4;
		uint8_t b4Const4:4;
		
		/*6th Byte*/
		uint8_t b1EnableSwitch: 1;
		uint8_t b1TurnRightSwitch: 1;
		uint8_t b1TurnLeftSwitch: 1;
		uint8_t b1HandleForward: 1;
		uint8_t b4Const5: 4;	
		/*7th Byte*/
		uint8_t b4HandleCtrlHigh: 4;
		uint8_t b4Const6: 4;
		/*8th Byte*/
		uint8_t b4HandleCtrlLow: 4;
		uint8_t b4Const7: 4;
		/*9th Byte*/
//		uint8_t u8CheckSum;
	}Data;
}xPcuRevPara;


typedef void (*PcuRevCallBackt)(xPcuRevPara*);

typedef union
{
	uint8_t buf[8];
	struct
	{
		/*1st Byte*/
		uint8_t b4ErrCodeSigLHigh4: 4;
		uint8_t b4Const1: 4;
		/*2nd Byte*/
		uint8_t b4ErrCodeSigLLow4: 4;
		uint8_t b4Const2: 4;
		/*3rd Byte*/
		uint8_t b4ErrCodeSigRHigh4: 4;
		uint8_t b4Const3: 4;
		/*4th Byte*/
		uint8_t b4ErrCodeSigRLow4: 4;
		uint8_t b4Const4: 4;
		
		/*5th Byte*/
//		uint8_t b1LiftLed: 1;
//		uint8_t b1LiftLedCtl: 1;
//		uint8_t b1ModeLed: 1;
//		uint8_t b1MoveLedCtr:1;
		uint8_t b4ModeControl:4;
		uint8_t b4Const5: 4;
		/*6th Byte*/
		uint8_t b1ReadState:1;
		uint8_t b1ErrState:1;
		uint8_t b1Reserve6:1;
		uint8_t b1Beep: 1;
		uint8_t b4Const6: 4;
		/*7th Byte*/
		uint8_t b1SlowLed: 1;
		uint8_t b3Reserve4: 3;
		uint8_t b4Const7: 4;
		/*8th Byte*/
		uint8_t b4BatterySoc: 4;
		uint8_t b4Const8: 4;
	}Data;
}xPcuSendPara;

typedef void (*PcuSendCallBackt)(xPcuSendPara*);

typedef enum
{
	PCU_Init = 0,
	PCU_LiftKeyPress,
	PCU_SlowKeyPress,
	PCU_MoveKeyPress,
	PCU_TurnLeftPress,
	PCU_TurnRightPress,
	PCU_EnableKeyPress,
	PCU_ValueNoZero,
	PCU_SpeakerPress,
}ePcuInitErr;

typedef void (*PcuErrCallBackt)(ePcuInitErr);

typedef struct
{
	xPcuRevPara PcuRevData;
	xPcuSendPara PcuSendData;
	PcuRevCallBackt PcuRevCallBack;	
	PcuSendCallBackt PcuSendCallBack;	
	PcuErrCallBackt PcuErrCallBack;
}xPcuProc;
#endif

#define DISPLAY_NORMAL			0b00000
#define LEFT_DOT						0b00010
#define RIGHT_DOT						0b00001
#define DOUBLE_DOT					0b00011
#define MASK_LEFT						0b01000
#define MASK_RIGHT					0b00100
#define MASK_BOTH						0b01100
#define HEX_DISPLAY					0b10000

extern void vPcuErrRegister(PcuErrCallBackt CallBack);
extern void vPcuRevRegister(PcuRevCallBackt CallBack);
extern void vPcuSendRegister(PcuSendCallBackt CallBack);
extern void vPcuProc(void);
extern void vPcuDisplayNumber(xPcuSendPara *SendData,uint8_t u8Number,uint8_t u8Dot);
extern void vPcuDisplayOrigin(xPcuSendPara *SendData,uint8_t u8NumberL,uint8_t u8NumberR);

#endif //

#endif
