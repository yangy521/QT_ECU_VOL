/*******************************************************************************
* Filename: BeepProc.h	                                             	 	   *
* Description:BeepProc C Head File 											   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/5/15    														   *
* Revision: V1.00															   *
*******************************************************************************/

#ifndef _ALARMLAMP_H_
#define _ALARMLAMP_H_

#include "stdint.h"
#include "NetTimer.h"


#define	ALARMLAMP_PERIOD				TIMER_PLC_PERIOD

typedef void (*AlarmLampCallBackt)(uint8_t u8lag);

extern void vAlarmLampSetPeriod(uint16_t u16Period);	
extern void vAlarmLampRegister(AlarmLampCallBackt);
extern void vAlarmLampProc(void);
extern void vAlarmLampSetOpenePeriod(uint16_t u16Period);

#endif
