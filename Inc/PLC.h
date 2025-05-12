/*******************************************************************************
* Filename: PLC.h 	                                    	     		   *
*                                                                              *
* Description: The header file of Device.c.							           *
* Author:                                                                      *
* Date: 															           *
* Revision:															           *
*																	           *
*******************************************************************************/

#include "KSDsys.h"


/*******************************************************************************
* 1. 指示灯
*******************************************************************************/
//状态指示灯
#define 	LED_YELLOW_SET						(1 << 0)
#define 	LED_YELLOW_RESET					(1 << 1)
#define 	LED_YELLOW_BLINK					(1 << 2)
#define		LED_RED_SET								(1 << 8)
#define		LED_RED_RESET							(1 << 9)
#define		LED_RED_BLINK							(1 << 10)

/*******************************************************************************
* 2. 设备类型 
*******************************************************************************/
//#define		Prd_Move					_CONTROLLER_MOVE	//主牵引控制器
//#define		Prd_Lift					_CONTROLLER_LIFT	//起升控制器
//#define		Prd_Steer					_CONTROLLER_STEER	//转向控制器
//#define		Prd_Move_Second		_CONTROLLER_MOVE_SECOND	//从牵引控制器
//#define		Prd_Logic					_CONTROLLER_LOGIC	//逻辑控制器
//#define		Prd_HMI 					_CONTROLLER_HMI	//仪表
//#define		Prd_PC						_CONTROLLER_PC	//PC机

/******************************************************************************
*函数定义
******************************************************************************/
/* Functions in PLC.c */
extern void	InitPLCCtl(void);
extern void PLCLogic(void);
extern void PLCDataSwap(void);
extern void PLCErrorTips(INT16U ErrorNo);
extern void PLCErrorClr(INT16U ErrorNo);
void LEDInit(void);


