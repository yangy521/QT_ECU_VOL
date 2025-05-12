/*******************************************************************************
* Filename: PressureSensor.h	                                               *
* Description:PressureSensor C Head File									   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/6/26    														   *
* Revision: V1.00															   *
*******************************************************************************/

#ifndef _AI_PROC_H_
#define _AI_PROC_H_

#include "stdint.h"

typedef enum{ 
	AI_B_AI1_R_ERR = 0,
	AI_B_AI2_R_ERR,
	AI_B_AI3_R_ERR,
	AI_5V_12V_OUT1_I_ERR,
	AI_5V_12V_OUT2_I_ERR,
	AIErrMax,
}eAiErrNo;


typedef void (*AiErrCallBackt)(eAiErrNo);

extern void vAiErrReg(AiErrCallBackt CallBack);
extern void vAiProcInit(void);
extern void vAiProc(void);



#endif

