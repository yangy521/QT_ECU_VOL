/*******************************************************************************
* Filename: Led.c 	                                    	     	           *
* Description:…Ë÷√Led◊¥Ã¨ 							           				   *
* Author:  QExpand, Lilu                                                       *
* Date: 2023/5/15															   *
* Revision:	V1.00														       *
*******************************************************************************/

#include "Device.h"
#include "Led.h"
#include "Log.h"

typedef struct
{
	uint8_t b1Red: 1;
	uint8_t b1Yellow: 1;
	uint8_t b6Reserve: 6;
}xLedBit;

/*******************************************************************************
* Name: void vLedSetState(eLed LedNo, eLedState LedState)
* Descriptio: Set Led State
* Input: LedNo: range in(LedRed, LedYellow)
*        LedState:rang in (LedSet, LedReset, LedBlink)
* Output: NULL 
*******************************************************************************/
void vLedSetState(eLed LedNo, eLedState LedState)
{

	static xLedBit sLedBit;
	
	if(LedRed == LedNo)
	{
		if(LedSet == LedState)
		{
			LED_R_ON();
			sLedBit.b1Red = 1;
		}
		else if(LedReset == LedState)
		{
			LED_R_OFF();
			sLedBit.b1Red = 0;
		}
		else if(LedBlink == LedState)
		{
			if(0 == sLedBit.b1Red)
			{
				LED_R_ON();
				sLedBit.b1Red = 1;
			}
			else
			{
				LED_R_OFF();
				sLedBit.b1Red = 0;
			}
		}
		else
		{
			i32LogWrite(ERR, LOG_LED, "LedState Parameter is wrong, LedState = %d\r\n", LedState);
		}
	}
	else if(LedYellow == LedNo)
	{
		if(LedSet == LedState)
		{
			LED_Y_ON();
			sLedBit.b1Yellow = 1;
		}
		else if(LedReset == LedState)
		{
			LED_Y_OFF();
			sLedBit.b1Yellow = 0;
		}
		else if(LedBlink == LedState)
		{
			if(0 == sLedBit.b1Yellow)
			{
				LED_Y_ON();
				sLedBit.b1Yellow = 1;
			}
			else
			{
				LED_Y_OFF();
				sLedBit.b1Yellow = 0;
			}
		}
		else
		{
			i32LogWrite(ERR, LOG_LED, "LedState Parameter is wrong, LedState = %d\r\n", LedState);
		}
	}
	else
	{
		i32LogWrite(ERR, LOG_LED, "LedNo Parameter is wrong, LedNo = %d\r\n", LedNo);
	}

//	LED_Y_OFF();
//	LED_R_OFF();
}




