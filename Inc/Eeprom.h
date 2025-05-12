/*******************************************************************************
* Filename: Log.h	                                             	 		   *
* Description:											   			 		   *
* Author:                                                           		   *
* Date:     														 		   *
* Revision:															 		   *
*******************************************************************************/

#ifndef _EEPROM_H_
#define _EEPROM_H_

#include "stdint.h"

typedef int32_t (*EeepromCallBackt)(uint16_t, uint16_t );
typedef void (*SaveParaFinishCallBackt)(uint32_t);
//extern uint16_t u16EepromWrite(uint16_t u16Address, uint16_t u16Data, uint8_t u8Mode);
//extern uint16_t u16EepromRead(uint16_t u16Address, uint16_t *u16Data, uint8_t u8Mode);

extern uint16_t u16EepromWrite(uint16_t u16Address, uint16_t u16Data, uint8_t u8Mode);
extern uint16_t u16EepromRead(uint16_t u16Address, uint16_t *u16Data, uint8_t u8Mode);
extern uint8_t u8GetEepromFlag(void);
extern uint8_t u8GetEepromErrFlag(void);
extern void vClrEepromErrFlag(void);
extern uint8_t u8GetEepromWriteReamin(void);
extern void vEepromNoBlockWriteProc(void);

extern void vEepromSetCallBack(EeepromCallBackt CallBack1, uint16_t u16ParaLastAddr, SaveParaFinishCallBackt CallBack2);


#endif
