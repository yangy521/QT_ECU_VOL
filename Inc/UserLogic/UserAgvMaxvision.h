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

#define MOTOR_SPEED_RANGE	4096
#define	MOTOR_CMD_RANGE		5000


//#define	DRIVER_CLOSE		0
//#define	DRIVER_OPEN			1
/******************************************************************************
*函数定义
******************************************************************************/	


extern void vUserEcuInit(void);
extern void vUserEcuProc(void);


//试验台测试


#endif //#ifndef _USER_COMM_H_
