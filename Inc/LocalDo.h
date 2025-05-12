/*******************************************************************************
* Filename: LocadlDo.h	                                             	 	   *
* Description:	LocalDo¹¦ÄÜÄ£¿é									   			   *
* Author:  QExpand, Lilu                                                       *
* Date: 2023/5/16    														   *
* Revision:	V1.00														 	   *
*******************************************************************************/

#ifndef _LOCALD0_H_
#define _LOCALD0_H_

#include "stdint.h"
#include "NetTimer.h"

#define	DO_PERIOD			TIMER_PLC_PERIOD


typedef enum{ 
	DO_DRIVEREN = 0,
	DO_ANALOG_1 = 1,
	DO_ANALOG_2 = 2,
	DO_ANALOG_3 = 3,
	DO_ENCODER1 = 4,
	DO_ENCODER2 = 5,
	LocalDoMax,
}eDoNo;


extern int32_t i32LocalDoSet(eDoNo DoNo, uint8_t u8state);
extern void vLocalDoInit(void);
extern void vLocalDoProc(void);

#endif
