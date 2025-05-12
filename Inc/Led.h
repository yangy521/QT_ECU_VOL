/*******************************************************************************
* Filename: Led.h	                                             	 		   *
* Description:Led C Head File											   	   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/5/15    														   *
* Revision: V1.00															   *
*******************************************************************************/

#ifndef _LED_H_
#define _LED_H_

#include "stdint.h"


typedef enum
{
	LedRed = 0,
	LedYellow = 1,
	LedMax,
}eLed;

typedef enum
{	
	LedSet = 0,
	LedReset = 1,
	LedBlink = 2,
	LedStateMax,
}eLedState;


extern void vLedSetState(eLed LedNo, eLedState LedState);
extern void vLedProc(void);



#endif

