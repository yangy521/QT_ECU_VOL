/*******************************************************************************
* Filename: BeepProc.c 	                                    	     	       *
* Description: 	Beep处理模块						           				   *
* Author:  QExpand, Lilu                                                       *
* Date: 2023/5/15															   *
* Revision:															           *
*******************************************************************************/

#include "LocalAi.h"
#include "WdgProc.h"
#include "NetTimer.h"
#include "AiProc.h"
#include "ErrCode.h"
#include "Log.h"
#include "Para.h"

#define	AI_5V_OPEN_VOLTAGE		5500
#define	AI_10V_OPEN_VOLTAGE		11000
#define	ENCODER_OPEN_VOLTAGE	1200


typedef struct
{
	AiErrCallBackt CallBack;
	union
	{
		uint8_t	u8AiCheckFlag;
		struct
		{
			uint8_t b1Ai1CheckFlag: 1;
			uint8_t b1Ai2CheckFlag: 1;
			uint8_t b1Ai3CheckFlag: 1;
			uint8_t b1Out1CheckFlag: 1;
			uint8_t b1Out2CheckFlag: 1;
		};
	};	
}xAiInfo;

static xAiInfo sgAiInfo;

void vAiErrReg(AiErrCallBackt CallBack)
{
	sgAiInfo.CallBack = CallBack;
}

void vAiProcInit(void)
{
	sgAiInfo.b1Ai1CheckFlag = ((uint16_t)i32GetPara(PARA_DriverFlag) >> 10) & 0x01;
	sgAiInfo.b1Ai2CheckFlag = ((uint16_t)i32GetPara(PARA_DriverFlag) >> 11) & 0x01;
	sgAiInfo.b1Ai3CheckFlag = ((uint16_t)i32GetPara(PARA_DriverFlag) >> 12) & 0x01;
	sgAiInfo.b1Out1CheckFlag = ((uint16_t)i32GetPara(PARA_DriverFlag) >> 13) & 0x01;
	sgAiInfo.b1Out2CheckFlag = ((uint16_t)i32GetPara(PARA_DriverFlag) >> 14) & 0x01;
}

static void vAiCheck(void)
{
	/*lilu 20230830 add Ai Monitor*/
	{
		i32SetPara(PARA_Ai1, i32LocalAiGetValue(AI_B_AI1_R) / 100);
		i32SetPara(PARA_Ai2, i32LocalAiGetValue(AI_B_AI2_R) / 100);
		i32SetPara(PARA_Ai3, i32LocalAiGetValue(AI_B_AI3_R) / 100);
		i32SetPara(PARA_Ai4, i32LocalAiGetValue(AI_5V_12V_OUT1_I) / 100);
		i32SetPara(PARA_Ai5, i32LocalAiGetValue(AI_5V_12V_OUT2_I) / 100);
		i32SetPara(PARA_Vbus, i32LocalAiGetValue(AI_B_VBUS_CHECK) / 100);
		i32SetPara(PARA_Ksi, i32LocalAiGetValue(AI_B_KSI_CHECK) / 100);
	}
	
	if (i32LocalAiGetValue(AI_B_AI1_R) >= AI_5V_OPEN_VOLTAGE)
	{
		if(false == u8GetNetTimerStartFlag(TIMER_AI1Check))
		{
			vSetNetTimer(TIMER_AI1Check, AI_LOST_CHK_DELAY_TIME);
		}
		
		if(true == u8GetNetTimerOverFlag(TIMER_AI1Check))
		{
			vKillNetTimer(TIMER_AI1Check);
			if (1 == sgAiInfo.b1Ai1CheckFlag)
			{
				i32ErrCodeSet(ErrCode71);
				if (NULL != sgAiInfo.CallBack)
				{
					sgAiInfo.CallBack(AI_B_AI1_R_ERR);
				}
			}
		}
	}
	else
	{
		if(true == u8GetNetTimerStartFlag(TIMER_AI1Check))
		{
			vKillNetTimer(TIMER_AI1Check);
		}
	}
	
	if (i32LocalAiGetValue(AI_B_AI2_R) >= AI_5V_OPEN_VOLTAGE)
	{
		if(false == u8GetNetTimerStartFlag(TIMER_AI2Check))
		{
			vSetNetTimer(TIMER_AI2Check, AI_LOST_CHK_DELAY_TIME);
		}
		
		if(true == u8GetNetTimerOverFlag(TIMER_AI2Check))
		{
			vKillNetTimer(TIMER_AI2Check);
			if (1 == sgAiInfo.b1Ai2CheckFlag)
			{
				i32ErrCodeSet(ErrCode72);
				if (NULL != sgAiInfo.CallBack)
				{
					sgAiInfo.CallBack(AI_B_AI2_R_ERR);
				}
			}
		}
	}
	else
	{
		if(true == u8GetNetTimerStartFlag(TIMER_AI2Check))
		{
			vKillNetTimer(TIMER_AI2Check);
		}
	}
	
	if (i32LocalAiGetValue(AI_B_AI3_R) >= AI_10V_OPEN_VOLTAGE)
	{
		if(false == u8GetNetTimerStartFlag(TIMER_AI3Check))
		{
			vSetNetTimer(TIMER_AI3Check, AI_LOST_CHK_DELAY_TIME);
		}
		
		if(true == u8GetNetTimerOverFlag(TIMER_AI3Check))
		{
			vKillNetTimer(TIMER_AI3Check);
			if (1 == sgAiInfo.b1Ai3CheckFlag)
			{
				i32ErrCodeSet(ErrCode73);
				if (NULL != sgAiInfo.CallBack)
				{
					sgAiInfo.CallBack(AI_B_AI3_R_ERR);
				}
			}
		}
	}
	else
	{
		if(true == u8GetNetTimerStartFlag(TIMER_AI3Check))
		{
			vKillNetTimer(TIMER_AI3Check);
		}
	}
	
	if (i32LocalAiGetValue(AI_5V_12V_OUT1_I) < ENCODER_OPEN_VOLTAGE)
	{
		if(false == u8GetNetTimerStartFlag(TIMER_Encoder1Check))
		{
			vSetNetTimer(TIMER_Encoder1Check, AI_LOST_CHK_DELAY_TIME);
		}
		
		if(true == u8GetNetTimerOverFlag(TIMER_Encoder1Check))
		{
			vKillNetTimer(TIMER_Encoder1Check);
			if (1 == sgAiInfo.b1Out1CheckFlag)
			{
				i32ErrCodeSet(ErrCode74);
				if (NULL != sgAiInfo.CallBack)
				{
					sgAiInfo.CallBack(AI_5V_12V_OUT1_I_ERR);
				}
			}
		}
	}
	else
	{
		if(true == u8GetNetTimerStartFlag(TIMER_Encoder1Check))
		{
			vKillNetTimer(TIMER_Encoder1Check);
		}
	}
	
	if (i32LocalAiGetValue(AI_5V_12V_OUT2_I) < ENCODER_OPEN_VOLTAGE)
	{
		if(false == u8GetNetTimerStartFlag(TIMER_Encoder2Check))
		{
			vSetNetTimer(TIMER_Encoder2Check, AI_LOST_CHK_DELAY_TIME);
		}
		
		if(true == u8GetNetTimerOverFlag(TIMER_Encoder2Check))
		{
			vKillNetTimer(TIMER_Encoder2Check);
			if (1 == sgAiInfo.b1Out2CheckFlag)
			{
				i32ErrCodeSet(ErrCode75);
				if (NULL != sgAiInfo.CallBack)
				{
					sgAiInfo.CallBack(AI_5V_12V_OUT2_I_ERR);
				}
			}
		}
	}
	else
	{
		if(true == u8GetNetTimerStartFlag(TIMER_Encoder2Check))
		{
			vKillNetTimer(TIMER_Encoder2Check);
		}
	}
}

/*******************************************************************************
* Name: void vAiProc(void)
* Descriptio: Ai模块处理
* Input: NULL
* Output: NULL 
*******************************************************************************/	
void vAiProc(void)
{
	vAiCheck();
	vWdgSetFun(WDG_AIPROC_BIT);
	i32LogWrite(DEBUG, LOG_AI, "AiProc is Running!\r\n");
}