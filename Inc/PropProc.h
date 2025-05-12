/*******************************************************************************
* Filename: PropProc.h	                                             	 	   *
* Description:	PropProc C Head File										   	   *
* Author:  QExpand, Lilu                                                       *
* Date: 2023/6/2    														   *
* Revision:															 		   *
*******************************************************************************/
#ifndef _PROPPROC_H_
#define _PROPPROC_H_

#include "stdint.h"
#include "NetTimer.h"

#define	PROP_PROC_PERIOD		TIMER_PLC_PERIOD

#define	PROP_LOST_CHK_DELAY_TIME 2000



typedef void (*PropErrCallBackt)(uint8_t u8Channel);

extern void vPropErrReg(PropErrCallBackt CallBack);
extern void vPropSetTarget(uint8_t u8Channel, int32_t i32Target);
extern void vPropProcInit(void);
extern int32_t i32GetPropCmd(uint8_t u8Channel);
extern void vPropProc(void);

#endif 

