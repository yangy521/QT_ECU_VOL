/*******************************************************************************
* Filename: LocadlAi.h	                                             	 	   *
* Description:	LocalAi收发功能										   		   *
* Author:  QExpand, Lilu                                                       *
* Date: 2023/5/16    														   *
* Revision:	V1.00														 	   *
*******************************************************************************/

#ifndef _LOCALAI_H_
#define _LOCALAI_H_

#include "stdint.h"

#define	AI_LOST_CHK_DELAY_TIME 2000

typedef enum{ 
	AI_B_KSI_CHECK = 0,
	AI_B_VBUS_CHECK = 1,
	AI_B_AI1_R = 2,
	AI_B_AI2_R = 3,
	AI_B_AI3_R = 4,
	AI_5V_12V_OUT2_I = 5,
	AI_5V_12V_OUT2_R = 6,
	AI_5V_12V_OUT1_I = 7,
	AI_5V_CHECK = 8,
	AI_15V_CHECK = 9,
	AI_5V_12V_OUT1_R = 10,
	AI_3V3_CHECK = 11,
	AI_B_AI1_CURRENT =12,
	AI_B_AI2_CURRENT =13,
	AI_B_AI3_CURRENT =14,
	LocalAiMax,
}eAiNo;


extern int32_t i32LocalAiGet(eAiNo AiNo);
extern int32_t i32LocalAiGetValue(eAiNo AiNo);

#endif
