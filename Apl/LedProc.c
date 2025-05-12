/*******************************************************************************
* Filename: LedProc.c 	                                    	     	       *
* Description: 	Led处理模块						           				       *
* Author:  QExpand, Lilu                                                       *
* Date: 2023/5/15															   *
* Revision:															           *
*******************************************************************************/

#include "Led.h"
#include "LedProc.h"
#include "WdgProc.h"
#include "Eeprom.h"
#include "PARA.h"


typedef struct
{
	/* led */
	uint16_t	u16LedCountHead;
	uint16_t	u16LedCountRep;
	uint16_t	u16LedCount;
	uint16_t	u16LedBlinkPeriod;      //Led blink period ms
}xLedPara;

static xLedPara sgLedPara;	/*Led状态信息的全局变量*/

/*******************************************************************************
* Name: void vLedSendAlmCode(uint8_t u8AlmCode)
* Descriptio: 给Led功能模块设置故障码
* Input: u8AlmrCode：故障代码
* Output: NULL
*******************************************************************************/
//void vLedSendAlmCode(uint8_t u8AlmCode)
//{
//	if(0 == sgLedPara.u16LedCount)
//	{
//		sgLedPara.u16LedCount = u8AlmCode;
//		sgLedPara.u16LedCountRep = 0;
//		sgLedPara.u16LedCountHead = 2;
//		vLedSetState(LedYellow, LedReset);
//		vLedSetState(LedRed, LedReset);
//	}
//}
void vLedSendAlmCode(uint8_t u8AlmCode)
{
		if (0 != u8AlmCode)
		{
			if(0 == sgLedPara.u16LedCount)
			{
				sgLedPara.u16LedCount = u8AlmCode;
				sgLedPara.u16LedCountRep = 0;
				sgLedPara.u16LedCountHead = 2;
				vLedSetState(LedYellow, LedReset);
				vLedSetState(LedRed, LedReset);
				/*lilu 20240412 éè??àúê·1ê??*/
				{
					uint16_t u16Tmp = 0; 
					uint16_t i= 0;
//							u16EepromRead(PARA_ErrCode0, &u16Tmp, 1);
//							
//							if (u16Tmp != u8AlmCode)
//							{
//											for (i=PARA_ErrCodeMax; i>PARA_ErrCode0; i--)
//											{
//															i32SetPara(i, i32GetPara(i-1));
//															u16EepromWrite(i, i32GetPara(i-1), 2);
//											}
//											u16EepromWrite(PARA_ErrCode0, u8AlmCode, 2);
//							}
					u16Tmp = i32GetPara(PARA_ErrCode0);
					if (u16Tmp != u8AlmCode)
					{
						for (i=PARA_ErrCodeMax; i>PARA_ErrCode0; i--)
						{
								i32SetPara(i, i32GetPara(i - 1));
								u16EepromWrite(i, i32GetPara(i - 1), 2);
						}
						i32SetPara(PARA_ErrCode0, u8AlmCode);
						u16EepromWrite(PARA_ErrCode0, u8AlmCode, 2);
					}
				}
			}
	}
}

/*******************************************************************************
* Name: void vLedProc(void)
* Descriptio: led模块处理
* Input: NULL
* Output: NULL 
*******************************************************************************/	
void vLedProc(void)
{
	if(sgLedPara.u16LedBlinkPeriod >= LED_BLINK_PERIOD)
	{
		sgLedPara.u16LedBlinkPeriod = 0;
		if(sgLedPara.u16LedCount > 0)
		{
			if(sgLedPara.u16LedCountHead <= 0)
			{
				if(sgLedPara.u16LedCount >= 10)
				{
					if(0 != sgLedPara.u16LedCountRep)
					{
						sgLedPara.u16LedCountRep = 0;
						sgLedPara.u16LedCount -= 10;
						vLedSetState(LedYellow, LedBlink);
						vLedSetState(LedRed, LedReset);
					}
					else
					{
						sgLedPara.u16LedCountRep = 1;
						vLedSetState(LedYellow, LedBlink);
						vLedSetState(LedRed, LedReset);
					}
				}
				else
				{
					if(0 != sgLedPara.u16LedCountRep)
					{
						sgLedPara.u16LedCountRep = 0;
						sgLedPara.u16LedCount -= 1;
						vLedSetState(LedYellow, LedReset);
						vLedSetState(LedRed, LedBlink);
					}
					else
					{
						sgLedPara.u16LedCountRep = 1;
						vLedSetState(LedYellow, LedReset);
						vLedSetState(LedRed, LedBlink);
					}
				}
			}
			else 
			{
				sgLedPara.u16LedCountHead--;
			}
		}
		else
		{
			sgLedPara.u16LedCount = 0;
			sgLedPara.u16LedCountRep = 0;
			sgLedPara.u16LedCountHead = 2;
			vLedSetState(LedYellow, LedSet);
			vLedSetState(LedRed, LedReset);
		}	
	}
	else
	{
		sgLedPara.u16LedBlinkPeriod += LED_PERIOD;
	}

	vWdgSetFun(WDG_LED_BIT);
}