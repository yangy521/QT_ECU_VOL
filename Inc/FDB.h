/*******************************************************************************
* Filename: FDB.h 	                                    	     		   *
*                                                                              *
* Description: The header file of FDB.c.							           *
* Author:                                                                      *
* Date: 															           *
* Revision:															           *
*																	           *
*******************************************************************************/

#include	"KSDsys.h"

/******************************************************************************
*数据类型定义
******************************************************************************/
/* encoder */
typedef struct ENCODER 
{
	INT16S 	ElecTheta;		/* Output: Motor Electrical angle (Q15) */
	INT16S 	ElecThetaM;		/* Output: Motor Electrical angle (Q15) */
	INT16S 	ElecThetaT;		/* Output: Motor Electrical angle (Q15) */
	INT16S	SyncFlag;		/* Variable: Z flag */
	INT32S  Angle;			/* Mec angle Q0, pulse unit 0 ~ stdinc-1/rotation*/
	INT32S 	CalibAngle; 	/* Parameter: Raw angular offset between 
								encoder index and phase a (Q0) */
	INT16U 	PolePairs;		/* Parameter: Number of pole pairs (Q0) */
	INT16S 	ErrorTimes;		/* Output: Error times */
	INT32S	PosAbs;			/* Q0, absolute */
	INT32S	PosAbsRaw;		/* Q0, 原始绝对位置 */
	INT32S	PosAbsOffset;	/* Q0, 绝对位置偏移 */
	INT32S	PosInc;			/* Q0, inc */
	INT16S	PosOld;			/* Q0, old, 增量编码器记录上次位置 */
	INT16S	CntOld;			/* Q0, old, 增量编码器记录上次位置 */

	//T法测速
	INT16S  TCntOld;
	INT32S	TimeAbs;
	INT32S	TimeWait;
	INT32S	TimeCapA;
	INT32S	TimeCapB;
	INT32S	WidthMin;
	INT32S  WidthMax;
	INT32S	tWidth;
	
	//No encoder check
	INT32S TimeCount;
	INT32S IncCount;
	
	void 	(*Init)(struct ENCODER *);		/*Pointer to the init function*/
	void 	(*Calc)(struct ENCODER *);		/*Pointer to the calc function*/
//wwq temp	void 	(*Calc_Pre)(struct ENCODER *);	/*Pointer to the calc_pre function*/
} ENCODER;

/* current sample */
typedef struct CURRENT_SAMPLE
{
	INT16S  U;				/* Q11 */
	INT16S  W;				/* Q11 */
	INT16S  P;				/* Q11 */
	INT16S	KsiVBus;
	INT16S	VBus;			/* Q12 */
	INT16S	V5out;
	INT16S	V12out;
	INT16S	RelayOut;
	INT16S	PowTmpHigh;
	INT16S	PowTmpLow;
	INT16S	MotorTmp;		/* Q??*/
	INT16S	ThrotPotHigh;
	INT16S	ThrotPotWip;
	INT16S	SpdPotWip;	
	INT16S	PropOvCur;	//Pump
	INT16S	Swi7;	//1222  
	INT16S	Swi8;	//1222
	INT16S	PhaseU;
	INT16S	PhaseV;
	INT16S	PhaseW;
	INT16S	PhaseP;
	
	INT16S	URaw;				/* A1 */
	INT16S	WRaw;				/* B0 */
	INT16S	PRaw;				/* P0 */
	INT16S	KsiVBusRaw;			/* A0 */
	INT16S	VBusRaw;			/* B5 */
	INT16S	V5outRaw;			/* A6 */
	INT16S	V12outRaw;
	INT16S	RelayOutRaw;	/* A3 */
	INT16S	PowTmpHighRaw;		/* A4 */
	INT16S	MotorTmpRaw;		/* B6 */
	INT16S	ThrotPotHighRaw;
	INT16S	ThrotPotWipRaw;
	INT16S	SpdPotWipRaw;	
	INT16S	Swi7Raw;	//1222
	INT16S	Swi8Raw;	//1222
	INT16S	PhaseURaw;
	INT16S	PhaseVRaw;
	INT16S	PhaseWRaw;
	INT16S	PhasePRaw;
	INT16S	VrefRaw;	
	
	INT32S  USum;				/* Q11 */
	INT32S  WSum;				/* Q11 */
	INT32S  PSum;				/* Q11 */
	INT32S	KsiVBusSum;
	INT32S	VBusSum;			/* Q12 */
	INT32S	V5outSum;
	INT32S	RelayOutSum;
	INT32S	PowTmpHighSum;
	INT32S	ThrotPotWipSum;
	INT32S	SpdPotWipSum;	
	INT32S	PhaseUSum;
	INT32S	PhaseVSum;
	INT32S	PhaseWSum;
	INT32S	PhasePSum;	
	
	INT16S	UZeroDrift;
	INT32S	UZeroDriftSum;
	INT16S	WZeroDrift;
	INT32S	WZeroDriftSum;
	INT16S	VBusZeroDrift;
	INT32S	VBusZeroDriftSum;
	INT16S	PZeroDrift;
	INT32S	PZeroDriftSum;
	INT16S	KsiVBusZeroDrift;
	INT32S	KsiVBusZeroDriftSum;
	INT16S	VCmdZeroDrift;
	INT32S	VCmdZeroDriftSum;

	void 	(*Init)(struct CURRENT_SAMPLE *);	/*Pointer to the init function*/
	void 	(*Calc)(struct CURRENT_SAMPLE *);	/*Pointer to the calc function*/
	void 	(*Calc_ZeroDrift)(struct CURRENT_SAMPLE *);	
	void 	(*Calc_PropZeroDrift)(struct CURRENT_SAMPLE *);	
} CURRENT_SAMPLE;



/* variables for feedback */
extern		ENCODER			gEncoder;
extern		CURRENT_SAMPLE	gCurrentSample;

/******************************************************************************
*函数定义
******************************************************************************/

extern void InitFeedbackDevice(void);
extern void FeedbackProcess(void);

