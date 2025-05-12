/*******************************************************************************
* Filename: LocadlDi.h	                                             	 	   *
* Description:	LocalDi¹¦ÄÜ										   			   *
* Author:  QExpand, Lilu                                                       *
* Date: 2023/5/12    														   *
* Revision:	V1.00														 	   *
*******************************************************************************/

#ifndef _LOCALDI_H_
#define _LOCALDI_H_

#include "stdint.h"
#include "NetTimer.h"

#define	DI_PERIOD			TIMER_PLC_PERIOD
#define	DI_FILTER_CONSTANT 	(20 / DI_PERIOD)

#if (CTLBOARD_TYPE == _HB4_GD32)
typedef enum{ 
	SWI1_R = 0,
	SWI2_R = 1,
	SWI3_R = 2,
	SWI4_R = 3,
	SWI5_R = 4,
	SWI6_R = 5,
	SWI7_R = 6,
	SWI8_R = 7,
	DRIVER1_R = 8,
	DRIVER2_R = 9,
	DRIVER3_R = 10,
	DRIVER4_R = 11,
	DRIVER5_R = 12,
	DRIVER6_R = 13,
	DRIVER7_R = 14,
	DRIVER8_R = 15,
	DRIVER9_R = 16,
	DRIVER10_R = 17,
	DRIVER11_R = 18,
	DRIVER12_R = 19,	
	LocalDiMax,
}eDiNo;
#endif //#if (CTLBOARD_TYPE == _HB4_GD32)

#if (CTLBOARD_TYPE == _HB6_GD32)
typedef enum{ 
	SWI1_R = 0,
	SWI2_R = 1,
	SWI3_R = 2,
	SWI4_R = 3,
	SWI5_R = 4,
	SWI6_R = 5,
	SWI7_R = 6,
	SWI8_R = 7,
	SWI9_R = 8,
	SWI10_R = 9,
	SWI11_R = 10,
	SWI12_R = 11,
	DRIVER1_R = 12,
	DRIVER2_R = 13,
	DRIVER3_R = 14,
	DRIVER4_R = 15,
	DRIVER5_R = 16,
	DRIVER6_R = 17,
	DRIVER7_R = 18,
	DRIVER8_R = 19,
	DRIVER9_R = 20,
	DRIVER10_R = 21,
	DRIVER11_R = 22,
	DRIVER12_R = 23,	
	LocalDiMax,
}eDiNo;
#endif //#if (CTLBOARD_TYPE == _HB6_GD32)


extern int32_t i32LocalDiGet(eDiNo DiNo);
extern void vLocalDiProc(void);

#endif
