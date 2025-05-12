/*******************************************************************************
* Filename: BeepProc.h	                                             	 	   *
* Description:BeepProc C Head File 											   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/5/15    														   *
* Revision: V1.00															   *
*******************************************************************************/

#ifndef _BEEPPROC_H_
#define _BEEPPROC_H_

#include "stdint.h"
#include "NetTimer.h"


#define	BEEP_PERIOD				TIMER_PLC_PERIOD

typedef void (*BeepCallBackt)(uint8_t u8lag);

extern void vBeepSetPeriod(uint16_t u16Period);	
extern void vBeepRegister(BeepCallBackt);
extern void vBeepProc(void);
extern void vBeepSetOpenePeriod(uint16_t u16Period);

#endif
