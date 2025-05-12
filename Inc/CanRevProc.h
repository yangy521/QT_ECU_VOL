/*******************************************************************************
* Filename: CanRevProc.h	                                             	   *
* Description:	Can接收处理									   			       *
* Author:  QExpand, Lilu                                                       *
* Date: 2023/5/12    														   *
* Revision:	V1.00														 	   *
*******************************************************************************/

#ifndef _CANREVPROC_H_
#define _CANREVPROC_H_

#include "CanCom.h"
#include "NetTimer.h"


#define CANREV_PERIOD		TIMER_PLC_PERIOD

#define	CAN_LOST			1
#define	CAN_NORMAL			0

#define	CANREV_PROC_NUM		10
#define CANID_LOST_NUM		16

typedef void (*CanRevCallBackt)(tCanFrame*);
typedef void (*CanIdLostCallBackt)(uint32_t, uint8_t);


typedef struct
{
	uint32_t u32CanId;
	union 
	{
		uint32_t u32Data;
		struct
		{
			uint8_t u8DataCnt;
			uint8_t u8Data0;
			uint8_t u8Data1;
			uint8_t u8Data2;
			
		};
	};
	CanRevCallBackt CallBack;
	uint32_t u32TimeOut;
	uint32_t u32TimeCnt;
}xRevCallBackProc;

typedef struct
{
	union
	{
		uint32_t u32Data;
		struct
		{
			uint32_t b31CanId: 31;
			uint32_t b1Lost: 1;
		};
	};
	uint32_t u32TimeOut;
	uint32_t u32Cnt;
	CanIdLostCallBackt CallBack;
}xCanIdLostInfo;

extern void vCanRevProc(void);
extern void vCanRevMsgRegister(xRevCallBackProc *RevCallBack);
extern void vCanIdLostReg(uint32_t u32CanId, uint32_t u32TimeOut, CanIdLostCallBackt CanIdLostCallBack);

#endif
