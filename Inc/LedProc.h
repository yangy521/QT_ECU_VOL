/*******************************************************************************
* Filename: LedProc.h	                                             	 	   *
* Description:LedProc C Head File 											   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/5/15    														   *
* Revision: V1.00															   *
*******************************************************************************/

#ifndef _LEDPROC_H_
#define _LEDPROC_H_

#include "stdint.h"

#define	LED_BLINK_PERIOD 		500
#define	LED_PERIOD				5

extern void vLedSendAlmCode(uint8_t u8AlmCode);	
extern void vLedProc(void);

#endif
