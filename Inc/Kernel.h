/*******************************************************************************
* Filename: Kernel.h 	                                    	     		   *
*                                                                              *
* Description: The header file of Kernel.c.							           *
* Author:                                                                      *
* Date: 															           *
* Revision:															           *
*																	           *
*******************************************************************************/

#include	"KSDsys.h"

/******************************************************************************
*数据类型定义
******************************************************************************/
/* Kernel control */
typedef struct
{
	INT16U			FSMState;			/* FSM状态 */
	INT16U			CurRunMode;			/* 当前控制方式 */
	INT16U			NextRunMode;		/* 下一控制方式 */
	INT16U			ReadyRun;			/* 运行条件就绪 */
	INT16U			GeneralCnt;
	INT16U			PowerRdyCnt;
	INT16U			BrakeCnt;
	INT16U			SaturatedCnt;
	INT16U			AClrCnt;
	INT16U			OverLoadCnt;
	INT16U			CalibAngleInit;
	INT16U			EncoderTypeInit;
	INT16U			MotorMaxHz;		/* 最大转速, Q0, rpm */
	INT32S			MaxCCWTorque;		/* 最大反转转矩, Q16, Nm */
	INT32S			MaxCWTorque;		/* 最大正转转矩, Q16, Nm */
	INT32S			MaxOverLoadCurrent;	/* 最大过载电流, Q24, 标幺值, A */
	INT32S			OverLoadCurrent2M;			/* 2 min 过载电流, Q24, 标幺值, A */
	INT32S			OverLoadCurrentCount2M;	/* 2 min 过载电流, Q24, 标幺值, A */
	INT32S			PumpMaxOverLoadCurrent;	/* 最大过载电流, Q24, 标幺值, A */
	INT32S			PumpOverLoadCurrent2M;			/* 2 min 过载电流, Q24, 标幺值, A */
	INT32S			PumpOverLoadCurrentCount2M;	/* 2 min 过载电流, Q24, 标幺值, A */
	INT32S			EspLowVoltage;			/* 最低急停电压，Q24，标幺值，V*/
	INT32S			EspHighVoltage;			/* 最高急停电压，Q24，标幺值，V*/	
} KERNEL_CTL;

/* variables for kernel control */
extern		KERNEL_CTL		gKernelCtl;

/******************************************************************************
*函数定义
******************************************************************************/
extern void InitKernel(void);
extern void KernelRun(void);
