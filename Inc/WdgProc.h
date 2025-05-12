/*******************************************************************************
* Filename: WdgProc.h	                                             	 	   *
* Description:	WdgProc C Head File										   	   *
* Author:  QExpand, Lilu                                                       *
* Date: 2023/5/15    														   *
* Revision:															 		   *
*******************************************************************************/

#ifndef _WDGPROC_H_
#define _WDGPROC_H_

#include "stdint.h"

#define	WDG_CANREV_BIT		0
#define	WDG_LED_BIT			1
#define	WDG_PCU_BIT			2
#define	WDG_MSTSLV_BIT		3
#define	WDG_LOCALDI_BIT		4
#define	WDG_LOCALDO_BIT		5
#define	WDG_DOPWM_BIT		6
#define	WDG_PROP_BIT		7
#define	WDG_BEEP_BIT		8
#define	WDG_LAMP_BIT		9
#define	WDG_ANGLE_BIT		10
#define	WDG_PRESSURE_BIT	11
#define	WDG_AIPROC_BIT		12
#define	WDG_BATTERY_BIT		13
#define	WDG_FUN14_BIT		14
#define	WDG_USER_BIT		15

#define	WDG_FUNS_NO		16

#define	WDG_FUNS_ALL	0xBFFF	/*14 暂时没有动作*/

#define	WDG_PERIOD				100
#define WDG_FUN_NAME_LENGTH		8


typedef struct
{
	uint32_t u32WdgTimeOut;
	uint32_t u32WdgTimeCnt;
	uint32_t u32WdgMonitorBit;
}xWdgParat;


typedef struct
{
	uint8_t u8Bit;
	uint8_t u8FunName[WDG_FUN_NAME_LENGTH];
}xWdgNamet;


extern void vWdgInit(uint32_t u32TimeOut);
extern void vWdgSetFun(uint8_t u8SetBit);
extern void vWdgProc(void);


#endif
