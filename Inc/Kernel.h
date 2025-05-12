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
*�������Ͷ���
******************************************************************************/
/* Kernel control */
typedef struct
{
	INT16U			FSMState;			/* FSM״̬ */
	INT16U			CurRunMode;			/* ��ǰ���Ʒ�ʽ */
	INT16U			NextRunMode;		/* ��һ���Ʒ�ʽ */
	INT16U			ReadyRun;			/* ������������ */
	INT16U			GeneralCnt;
	INT16U			PowerRdyCnt;
	INT16U			BrakeCnt;
	INT16U			SaturatedCnt;
	INT16U			AClrCnt;
	INT16U			OverLoadCnt;
	INT16U			CalibAngleInit;
	INT16U			EncoderTypeInit;
	INT16U			MotorMaxHz;		/* ���ת��, Q0, rpm */
	INT32S			MaxCCWTorque;		/* ���תת��, Q16, Nm */
	INT32S			MaxCWTorque;		/* �����תת��, Q16, Nm */
	INT32S			MaxOverLoadCurrent;	/* �����ص���, Q24, ����ֵ, A */
	INT32S			OverLoadCurrent2M;			/* 2 min ���ص���, Q24, ����ֵ, A */
	INT32S			OverLoadCurrentCount2M;	/* 2 min ���ص���, Q24, ����ֵ, A */
	INT32S			PumpMaxOverLoadCurrent;	/* �����ص���, Q24, ����ֵ, A */
	INT32S			PumpOverLoadCurrent2M;			/* 2 min ���ص���, Q24, ����ֵ, A */
	INT32S			PumpOverLoadCurrentCount2M;	/* 2 min ���ص���, Q24, ����ֵ, A */
	INT32S			EspLowVoltage;			/* ��ͼ�ͣ��ѹ��Q24������ֵ��V*/
	INT32S			EspHighVoltage;			/* ��߼�ͣ��ѹ��Q24������ֵ��V*/	
} KERNEL_CTL;

/* variables for kernel control */
extern		KERNEL_CTL		gKernelCtl;

/******************************************************************************
*��������
******************************************************************************/
extern void InitKernel(void);
extern void KernelRun(void);
