/*******************************************************************************
* Filename: Kernel.h 	                                    	     		   *
*                                                                              *
* Description: The header file of Kernel.c.							           *
* Author:                                                                      *
* Date: 															           *
* Revision:															           *
*																	           *
*******************************************************************************/
#ifndef _CURRENT_H_
#define _CURRENT_H_

#include	"KSDsys.h"
#include	"Device.h"
//#include  "Ifoc.h"

/******************************************************************************
*数据类型定义
******************************************************************************/
/* current feedback filter */
typedef struct CURRENT_FILTER
{
	_iq 	In;
	_iq 	Out;
	_iq		Display;
	INT64S DisplaySum;
	void 	(*Init)(struct CURRENT_FILTER *);
	void 	(*Calc)(struct CURRENT_FILTER *);
} CURRENT_FILTER;

/* current clarke transform */
typedef struct CLARKE
{
	_iq  	As;			 /* Input: phase-a stator variable */
	_iq  	Bs;			 /* Input: phase-b stator variable */
	_iq  	Alpha;		 /* Output: stationary d-axis stator variable */
	_iq  	Beta;		 /* Output: stationary q-axis stator variable */
	void 	(*Calc)(struct CLARKE *);	 /* Pointer to calculation function */
} CLARKE;

/* current park transform */
typedef struct PARK
{
	_iq  	Alpha;		 /* Input: stationary d-axis stator variable */
	_iq  	Beta;		 /* Input: stationary q-axis stator variable */
	_iq  	Angle;		 /* Input: rotating angle (pu) */
	_iq  	Ds;			 /* Output: rotating d-axis stator variable */
	_iq  	Qs;			 /* Output: rotating q-axis stator variable */
	void 	(*Calc)(struct PARK *);	 /* Pointer to calculation function */
} PARK;

/* current ipark transform */
typedef struct IPARK
{
	_iq  	Alpha;		 /* Output: stationary d-axis stator variable */
	_iq  	Beta;		 /* Output: stationary q-axis stator variable */
	_iq  	Angle;		 /* Input: rotating angle (pu) */
	_iq  	Ds;			 /* Input: rotating d-axis stator variable */
	_iq  	Qs;			 /* Input: rotating q-axis stator variable */
	void	(*CircleLimit)(struct IPARK *);/* Pointer to limit the Ds, Qs inside circle */
	void 	(*Calc)(struct IPARK *);	 /* Pointer to calculation function */
} IPARK;

/* current sv generator */
typedef struct SVGENDQ	
{
	_iq  	Ualpha; 		/* Input: reference alpha-axis phase voltage */
	_iq  	Ubeta;			/* Input: reference beta-axis phase voltage */
	_iq  	Ta;				/* Output: reference phase-a switching function */
	_iq  	Tb;				/* Output: reference phase-b switching function */
	_iq  	Tc;				/* Output: reference phase-c switching function */
	_iq  	TMin;		  /* para: TMax = (1- DeadTime*2/itp) if Tx >= TMax, x==1*/
	_iq  	TMax;		  /* para: TMax = (1- DeadTime*2/itp) if Tx >= TMax, x==1*/
	_iq		DTvalue;		/* Input dead time para 1 */
	INT16S	DTsector;		/* Input dead time para 2 */
	INT16S	HighCountA; /* itp count under A==1*/
	INT16S	HighCountB; /* itp count under B==1*/
	INT16S	HighCountC; /* itp count under C==1*/
	INT16S	HighCountLimit;
	
	void 	(*Calc)(struct SVGENDQ *);	 /* Pointer to calculation function */
} SVGENDQ;

typedef struct DEAD_TIME
{
	_iq		Value;		/* Output */
	INT16S	Sector;		/* Output */
	_iq		idq;			/* Input */
	_iq		iU;			/* Input */
	_iq		iV;			/* Input */
	_iq		iW;			/* Input */
	_iq		idq_old;		/* For filter */
	_iq		k;			/*  */
	_iq		ValueMax;
	_iq		ThresholdMax;
	void	(*Init)(struct DEAD_TIME *);
	void	(*Calc)(struct DEAD_TIME *);
} DEAD_TIME;

/* current pid regulator */
typedef struct CURRENT_PID
{
	_iq		Out;			/* Output: PID output */
	_iq		Ref;			/* Input: Reference input */
	_iq		Fdb;			/* Input: Feedback input */
	_iq		Err;			/* Variable: Error */
	_iq  	Kp;				/* Parameter: Proportional gain */
	_iq  	Ki;				/* Parameter: Integral gain */
	_iq  	Kc;				/* Parameter: Integral correction gain */
	_iq  	Up;				/* Variable: Proportional output */
	INT64S	Ui;				/* Variable: 64 bit Integral output */
	_iq  	OutMax;			/* Parameter: Maximum output */
	_iq  	OutMin;			/* Parameter: Minimum output */
	_iq  	OutPreSat;		/* Variable: Pre-saturated output */
	_iq  	SatErr;			/* Variable: Saturated difference */
	void 	(*Init)(struct CURRENT_PID *);
	void 	(*Calc)(struct CURRENT_PID *);	/* Pointer to calculation function*/
} CURRENT_PID;

/* current loop */
typedef struct CURRENT_LOOP
{
	CLARKE				Clarke;
	PARK 					Park;
	CURRENT_PID 	IqPID;
	CURRENT_PID 	IdPID;
	IPARK 				Ipark;
	SVGENDQ 			SvGen;
	PWM_GEN 			PwmGen;
	DEAD_TIME			DeadTime;
	CURRENT_FILTER CurrentFilter;
	INT32U		State;  //
	_iq				U;
	_iq				W;
	_iq				V;
	_iq 			RotorAngle;
	_iq 			hRotFlx_Theta;
	_iq				IqRef;
	_iq				IqFdb;
	_iq				IdRef;
	_iq				IdFdb;
	_iq				VfRef;
	_iq				IvFdb;
	_iq				VbusFdb;
	_iq				BackELecForce;
	_iq				SpeedFilter;
	_iq				DTvalue;
	INT16U			DTsector;
	void 			(*Init)(struct CURRENT_LOOP *);
	void 			(*Calc)(struct CURRENT_LOOP *);
	void 			(*Reset)(struct CURRENT_LOOP *);
} CURRENT_LOOP;

/* variables for current control */
extern		CURRENT_LOOP	gCurrentLoop;

/******************************************************************************
*函数定义
******************************************************************************/
extern	void 	CurrentLoopInit(CURRENT_LOOP *);
extern	void 	CurrentLoopCalc(CURRENT_LOOP *);
extern	void 	CurrentLoopReset(CURRENT_LOOP *);
extern  void  SvGenDqCalc1(SVGENDQ *v);
extern  void  IPark_Circle_Limitation1(IPARK *v);

#endif //_CURRENT_H_
