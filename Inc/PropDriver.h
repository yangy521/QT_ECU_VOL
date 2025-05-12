/*******************************************************************************
* Filename: PropDriver.h 	                                    	     	   *
*                                                                              *
* Description: The header file of PropDriver.c.							       *
* Author:                                                                      *
* Date: 															           *
* Revision:															           *
*																	           *
*******************************************************************************/
#ifndef _PROPDRIVER_H_
#define _PROPDRIVER_H_

#include	"KSDsys.h"
#include	"Device.h"


#define PROP_CURRENT_FACOTR		0.0016							/*0.0016 = (3.3 * 7.5 / 4096 / (7.5 + 68) / 0.05)*/

typedef enum
{
	PropDriverCh0 = 0,
	PropDriverCh1,
//	PropDriverCh2,
//	PropDriverCh3,
	PropDriverMax,
}ePropDriverNo;
/******************************************************************************
*数据类型定义
******************************************************************************/
/* Prop driver current feedback filter */
typedef struct xPropCurrentFilter
{
	_iq 	In;
	_iq 	Out;
	_iq		Display;
	INT64S DisplaySum;
	void 	(*Init)(struct xPropCurrentFilter *);
	void 	(*Calc)(struct xPropCurrentFilter *);
} xPropCurrentFilter;


/* Prop driver current pid regulator */
typedef struct xPropCurrentPid
{
	_iq		Out;			/* Output: PID output */
	_iq		Ref;			/* Input: Reference input */
	_iq		Fdb;			/* Input: Feedback input */
	_iq		Err;			/* Variable: Error */
	_iq  	Kp;				/* Parameter: Proportional gain */
	_iq  	Ki;				/* Parameter: Integral gain */
	_iq  	Kc;				/* Parameter: Integral correction gain */
	_iq  	Up;				/* Variable: Proportional output */
	_iq		KoeRes;   /* Valve resistance Koe*/
	INT64S	Ui;				/* Variable: 64 bit Integral output */
	_iq  	OutMax;			/* Parameter: Maximum output */
	_iq  	OutMin;			/* Parameter: Minimum output */
	_iq  	OutPreSat;		/* Variable: Pre-saturated output */
	_iq  	SatErr;			/* Variable: Saturated difference */
	void 	(*Init)(struct xPropCurrentPid *);
	void 	(*Calc)(struct xPropCurrentPid *);	/* Pointer to calculation function*/
} xPropCurrentPid;

/* current loop */
typedef struct xPropCurrentLoop
{
	xPropCurrentPid 	IPID;
	PWM_GEN 			PwmGen;
	xPropCurrentFilter PropCurrentFilter;
	INT32U		State;  //
	_iq				IRef;
	_iq				IFdb;
	_iq				VfRef;
	_iq				VbusFdb;
	_iq				IRefMin;
	_iq				IRefMax;
	_iq				AccRatio;
	_iq				AccRatioEnd;
	float			fCurveMinPara;
	float			fCurveMaxPara;
	_iq				CurrentCmd;
	_iq				CurrentCmdRaw;
	
	INT16U 			u16DitherPeriod;	
	INT16U 			u16DitherStep;	
	
	INT16U u16DitherCnt;
	INT32S i32DitherValue;
	INT32U Cnt;
	
	ePropDriverNo	PropDriverNo;
	void 			(*Init)(struct xPropCurrentLoop *);
	void 			(*Calc)(struct xPropCurrentLoop *);
	void 			(*Reset)(struct xPropCurrentLoop *);
} xPropCurrentLoop;


extern void vPropCurrentLoopInit(void);
extern xPropCurrentLoop *pPropCurrentLoopGet(ePropDriverNo PropDriverNo);
extern void PropCurrentSmooth(xPropCurrentLoop *pPropCurrent, INT32S i32CurrentCmd);

#endif //_PROPDRIVER_H_
