/*******************************************************************************
* Filename: UserStackerTruckProc.h	                                           *
* Description: 逻辑层的头文件，包含对应的回调函数	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/07/22    														   *
* Revision:	 														 		   *
*******************************************************************************/
#ifndef _USER_TEST_H_
#define _USER_TEST_H_

#include "MstSlvCom.h"
#include "DoPwm.h"
#include "PropDriver.h"
#include "LocalDi.h"
#include "NetTimer.h"


#define	DRIVER_CLOSE		0
#define	DRIVER_OPEN			1
/******************************************************************************
*函数定义
******************************************************************************/	



#define	AI_B_AI1_ERR				ErrCode71
#define	AI_B_AI2_ERR				ErrCode72
#define	AI_B_AI3_ERR				ErrCode73
#define	AI_5V_12V_OUT1_ERR			ErrCode74
#define	AI_5V_12V_OUT2_ERR			ErrCode75

extern void vUserEcuInit(void);
extern void vUserEcuProc(void);


//试验台测试


#endif //#ifndef _USER_COMM_H_
