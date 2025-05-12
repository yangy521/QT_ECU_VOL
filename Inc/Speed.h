/*******************************************************************************
* Filename: Kernel.h 	                                    	     		   *
*                                                                              *
* Description: The header file of Kernel.c.							           *
* Author:                                                                      *
* Date: 															           *
* Revision:															           *
*																	           *
*******************************************************************************/
#ifndef _SPEED_H_
#define _SPEED_H_

#include	"KSDsys.h"
//#include  "Ifoc.h"
//#define FULL_VOLTAGE_SPRRPM   300.0   //NUOLI
//#define FULL_VOLTAGE_SPRRPM   100.0   // LIDE
/******************************************************************************
*数据类型定义
******************************************************************************/
/* speed pid regulator */
typedef struct SPEED_PID
{
	_iq  	Out;			/* Output: PID output */
	_iq  	Ref;			/* Input: Reference input */
	_iq  	Fdb;			/* Input: Feedback input */
	_iq  	Err;			/* Variable: Error */
	_iq  	Kp;				/* Parameter: Proportional gain */
	_iq  	Ki;				/* Parameter: Integral gain */
	_iq  	Kc;				/* Parameter: Integral correction gain */
	_iq  	Up;				/* Variable: Proportional output */
	_iq  	UfMov;		/* Variable: Move forward control act value */
	_iq  	UfAcc;		/* Variable: Acc forward control act value */
	INT64S	Ui;				/* Variable: 64 bit Integral output */
	INT32S	UpSucc;		/* Variable: 64 bit Proportional remain */
	_iq  	OutMax;			/* Parameter: Maximum output */
	_iq  	OutMin;			/* Parameter: Minimum output */
	_iq  	OutPreSat;		/* Variable: Pre-saturated output */
	_iq  	SatErr;			/* Variable: Saturated difference */
	_iq  	KcoeTorque2Iq;
	INT32U		State;  //0-匀速；1-正向加速；2-负向加速。
	void 	(*Init)(struct SPEED_PID *);
	void 	(*Calc)(struct SPEED_PID *);
} SPEED_PID;


/* speed observer */
typedef struct SPEED_OBSERVER
{
	SPEED_PID 		PID;
	INT64S			TaSum;			/* ???????,?????? */
	_iq				SpdRaw;
	_iq				SpdObs;
	_iq				IqFdb;
	void 			(*Init)(struct SPEED_OBSERVER *);
	void 			(*Calc)(struct SPEED_OBSERVER *);
} SPEED_OBSERVER;

/* butterworth filter struct */
typedef struct BUTTER
{
	_iq		y0;
	_iq		y1;
	_iq		y2;
	_iq		x0;
	_iq		x1;
	_iq		x2;
	_iq		a0;
	_iq		a1;
	_iq		a2;
/*	_iq		b0 = 1;		*/
	_iq		b1;
	_iq		b2;
	_iq		w;			/* ?????? */
	_iq		Q;			/* ???? */
	void	(*Design)(struct BUTTER *);
	void	(*Calc)(struct BUTTER *);
} BUTTER;

/* speed feedback filter */
typedef struct SPEED_FILTER
{
	_iq 	In;
	_iq 	Out;			//1ms everage
	_iq 	Out16ms;		//16ms everage
	_iq 	Out64ms;	//64ms everage
	_iq		Display;	//256ms everage
	INT64S OutSum;
	INT64S Out16msSum;
	INT64S Out64msSum;
	INT64S DisplaySum;
	void 	(*Init)(struct SPEED_FILTER *);
	void 	(*Calc)(struct SPEED_FILTER *);
} SPEED_FILTER;

/* torque filter */
typedef struct TORQUE_FILTER
{
	BUTTER	Notch1;
	BUTTER	Notch2;
	BUTTER	LPF1;
	BUTTER	LPF2;
	_iq 	Out;
	_iq 	In;
	void 	(*Init)(struct TORQUE_FILTER *);
	void 	(*Calc)(struct TORQUE_FILTER *);
} TORQUE_FILTER;

/* speed loop */

#define   SPD_CURLIMIT_DIRVE 		(0x0001 << 0)
#define   SPD_CURLIMIT_UPHILL 	(0x0001 << 1)
#define   SPD_CURLIMIT_DWHILL 	(0x0001 << 2)
#define   SPD_CURLIMIT_REGEN 		(0x0001 << 3)
#define   SPD_CURLIMIT_BRAKE 		(0x0001 << 4)
#define   SPD_REVERSE_THROTLE 	(0x0001 << 5)
#define   SPD_CURLIMIT_ALL      (SPD_CURLIMIT_DIRVE | SPD_CURLIMIT_UPHILL | SPD_CURLIMIT_DWHILL | SPD_CURLIMIT_REGEN | SPD_CURLIMIT_BRAKE | SPD_REVERSE_THROTLE)

#define   SPD_FDB_LOWLL 					(0x0001 << 6)

#define   SPD_FEEDFORW_PKVFF 		(0x0001 << 8)
#define   SPD_FEEDFORW_NKVFF 		(0x0001 << 9)
#define   SPD_FEEDFORW_PKAFF 		(0x0001 << 10)
#define   SPD_FEEDFORW_NKAFF 		(0x0001 << 11)
#define   SPD_FEEDFORW_PKBFF 		(0x0001 << 12)
#define   SPD_FEEDFORW_NKBFF 		(0x0001 << 13)
#define   SPD_FEEDFORW_ALL			(SPD_FEEDFORW_PKVFF|SPD_FEEDFORW_NKVFF|SPD_FEEDFORW_PKAFF|SPD_FEEDFORW_NKAFF|SPD_FEEDFORW_PKBFF|SPD_FEEDFORW_NKBFF)

#define   SPD_REFL_FOLLOW			  (0x0001 << 17)
#define   SPD_SOFT_START				(0x0001 << 18)
#define   SPD_RESET				      (0x0001 << 19)
#define   SPD_LOOP_RESET				(0x0001 << 20)


//*********** PLC spd point
#if (MOTOR_SPEED_LEVEL	== _MOTOR_SPEED_LEVEL100)
	#define MIN_SPD_CMD   			8									//最小速度命令 rpm
	#define RDY_STOP_FRQ   			4									//停止转动转速 rpm
	#define RDY_STAT_FRQ   			6									//恢复电机控制的转动转速 rpm
	#define RAMP_FRQ            4//4							//溜坡转速 rpm
	#define SOFT_LIMIT_FRQ			12.0							//速度接近目标值，需要柔和加减速处理的区域
#endif
#if (MOTOR_SPEED_LEVEL	== _MOTOR_SPEED_LEVEL300)
	#define MIN_SPD_CMD   			8									//最小速度命令 rpm
	#define RDY_STOP_FRQ   			4									//停止转动转速 rpm
	#define RDY_STAT_FRQ   			6									//恢复电机控制的转动转速 rpm
	#define RAMP_FRQ            4									//溜坡转速 rpm
	#define SOFT_LIMIT_FRQ			12.0							//速度接近目标值，需要柔和加减速处理的区域
#endif
#if (MOTOR_SPEED_LEVEL	== _MOTOR_SPEED_LEVEL3000)
	#define MIN_SPD_CMD   			100									//最小速度命令 rpm
	#define RDY_STOP_FRQ   			60									//停止转动转速 rpm
	#define RDY_STAT_FRQ   			90									//恢复电机控制的转动转速 rpm
	#define RAMP_FRQ            60									//溜坡转速 rpm
	#define SOFT_LIMIT_FRQ			200.0							//速度接近目标值，需要柔和加减速处理的区域
#endif

#define REF_FOLLOW_LIMIT(X)		    (X>>5)		//停止制动过程中，最小制动力

#define SOFT_NOACT_AREAH     0.3             //从低速段开始减速到0速，加速度屏蔽柔化的区间占起始速度的比例 30%
#define SOFT_NOACT_AREAL     0.2             //从更低速段开始减速到0速，加速度屏蔽柔化的区间占起始速度的比例 20%
#define REF_FOLLOW_MAX			4							//最小制动力控制时，加速度调整范围 0~16倍

#define STOP_MOVE_TIME			2000								//MS
#define STOP_FIRST_TIME			1500								//MS
typedef struct SPEED_LOOP
{
	SPEED_FILTER	Filter;
	SPEED_OBSERVER 	Observer;
	SPEED_PID 		PID;
	TORQUE_FILTER 	TorqueFilter;
//	TORQUE_FILTER 	FluxFilter;
//	SPEED_WEAK_FLUX WeakFlux;
	INT32U		State;  		//
	INT64S 		SlipFreq;   //
	_iq				FluxWeRef;	
	_iq				BaseFlux;					// Base flux current,vary under different state
	_iq				BaseIq;						// Base Iq current,vary under different state
	_iq				FluxRef1;
	_iq				PidOutMax;
	_iq				PidOutMin;
	_iq  			UfMov;								/* Variable: Move forward control act value  */
	_iq  			UfMovDst;							/* Variable: Move forward control destination value  */
	_iq  			UfMovInc;							/* Variable: Move forward control inc step */
	_iq  			UfMovDec;							/* Variable: Move forward control dec step */
	_iq  			UfAcc;								/* Variable: Acc forward control destination value  */
	_iq  			UfAccDst;							/* Variable: Acc forward control destination value  */
	_iq  			UfAccInc;							/* Variable: Acc forward control inc step */
	_iq  			UfAccDec;							/* Variable: Acc forward control dec step */
	_iq       RefFollowFluxKoe;			/* =1/gCRam.SvPa.MotorPara.DriveFlux */
	_iq       RefFollowDecKoe;			/* =1/REF_FOLLOW_LIMIT(gCRam.SvPa.MotorPara.DriveCurrent) */

	_iq  			UfMovPara;			/* Variable: Move forward control max value limit */
	_iq  			UfMovIncPara;		/* Variable: Move forward control inc step */
	_iq  			UfMovDecPara;		/* Variable: Move forward control dec step */
	_iq  			UfAccPara;			/* Variable: Acc forward control max value limit */
	_iq  			UfAccIncPara;		/* Variable: Acc forward control inc step */
	_iq  			UfAccDecPara;		/* Variable: Acc forward control dec step */
	_iq  			UfBkePara;			/* Variable: Bke forward control max value limit */
	_iq  			UfBkeIncPara;		/* Variable: Bke forward control inc step */
	_iq  			UfBkeDecPara;		/* Variable: Bke forward control dec step */
	
	_iq				FlxCmd;			//d current command to current loop
	_iq				CurrentCmd; //q current command to current loop
	_iq				SpdFdb;
	_iq				SpdCmd;				/* 速度指令 */
	_iq				SpdRef;				/* 加减速之后的速度指令 */
	_iq				SpdRefL;				/* 加减速之后的速度指令 */
	_iq				IqFdb;				/* 用于电流限制 */
	_iq				IvFdb;				/* 用于电流限制 */
	_iq				VoltageBus;			//dc bus voltage
	_iq				VoltageBusLimit;//dc bus voltage break limit.
	
	_iq				BrakeCmd;				/* Brake command 100% - _IQ(1.0)*/
	_iq				BrakeCmdSmall;	/* Brake command 12%*/
	_iq				BrakeCmdLow;		/* Brake command 33%*/
	_iq				BrakeCmdFull;		/* Brake command 90%*/
	_iq				BrakeCmdKoeSL;		/* Koe = 1.0/(BrakeCmdLow-BrakeCmdSmall) */
	_iq				BrakeCmdKoeLF;		/* Koe = 1.0/(BrakeCmdFull-BrakeCmdLow) */

	_iq				FullAccRateHs;			/*Acc rate when full throttle is applied at high vehicle speeds*/
	_iq				FullAccRateLs;			/*Acc rate when full throttle is applied at low vehicle speeds*/
	_iq				FullAccRateKoe;  		/*Koe for speed between Ls & Hs  Koe = (RateHs-RateLs)/(Hs-Ls)*/
	_iq				LowAccRate;					/*Acc rate when a small amount throttle is applied */
	_iq				NeutralDecRateHs;   /*Dec rate when the throttle is released to neutral at high vehicle speeds.*/
	_iq				NeutralDecRateLs;		/*Dec rate when the throttle is released to neutral at low vehicle speeds.*/
	_iq				NeutralDecRateSmall;		/*Dec rate when the throttle is released to neutral at small vehicle speeds.*/
	_iq				NeutralDecRateKoeSL;  /*Koe for speed between Ls & Hs  Koe = (RateHs-RateLs)/(Hs-Ls)*/
	_iq				NeutralDecRateKoeLH;  /*Koe for speed between Ls & Hs  Koe = (RateHs-RateLs)/(Hs-Ls)*/
	_iq				FullBrakeDecRateHs; /*Dec rate when full brake or full throttle in opposite direction at high vehicle speeds.*/
	_iq				FullBrakeDecRateLs; /*Dec rate when full brake or full throttle in opposite direction at low vehicle speeds.*/
	_iq				FullBrakeDecRateKoe;/*Koe for speed between Ls & Hs  Koe = (RateHs-RateLs)/(Hs-Ls)*/
	_iq				LowBrakeDecRateHLs;	/*Dec rate when a small amount of brake brake or a small amount of brake throttle in opposite direction at all vehicle speeds.*/
	_iq				PartialDecRateHLs;	/*Dec rate when the throttle is reduced without being released to neutral */
	_iq				EndDecRateApproach;				/*Dec rate near cmd speed , soft act */
	_iq				EndAccRateApproach;				  /*Acc rate near cmd speed , soft act */
	_iq				EndDecRateStop;			/*Dec rate near zero cmd speed , soft act */
	_iq				EndAccRateStop;			/*Acc rate near zero cmd speed , soft act */
	_iq				SoftSpeedThreshold;			/*Speed error below which the soft parameters will be used. */
	_iq				HighSpeed;			/*Speed below which the HS parameters will be used. */
	_iq				LowSpeed;				/*Speed below which the LS parameters will be used. */
	_iq				SmallSpeed;				/*Speed below which the small parameters will be used. */
	_iq				SpdSmallThrottle; /*Para for small amount throttle, Hz, 10%*/
	_iq				SpdFullThrottle;	/*Para for small amount throttle, Hz, 90%*/
	_iq				SpdThrottleKoe;			/* Koe = 1.0/(SpdFullThrottle-SpdSmallThrottle) */
	
	_iq				SpdAtZeroSpdcmdUp;
	_iq				SpdDecLKOld;
	_iq				SpdAccLKOld;
	
	_iq				SpdAccK;     /*speed cmd low filter koe*/
	_iq				SpdDecK;     /*speed cmd low filter koe*/

	_iq  SpdAccRatioFast;
	_iq  SpdDecRatioFast;
	_iq  SpdAccRatioSlow;
	_iq  SpdDecRatioSlow;
	_iq  SpdAccRatioAct;				/*Acc rate ratio by spd mode*/
	_iq  SpdDecRatioAct;				/*Dec rate ratio by spd mode*/
	
	INT32U EncoderDirCount;
	INT16U BatteryLostCountL;
	
	void 			(*CalcFluxRefTorqueLimit)(struct SPEED_LOOP *);
	void 			(*Init)(struct SPEED_LOOP *);
	void 			(*Calc)(struct SPEED_LOOP *);
	void 			(*Reset)(struct SPEED_LOOP *);
} SPEED_LOOP;

/* variables for speed control */
extern		SPEED_LOOP		gSpeedLoop;

/******************************************************************************
*函数定义
******************************************************************************/
extern	void 	SpeedLoopInit(SPEED_LOOP *);
extern	void 	SpeedLoopCalc(SPEED_LOOP *);
extern	void 	SpeedLoopReset(SPEED_LOOP *);
extern  void	Lpf1Design(BUTTER *);
extern  void	Lpf1Calc(BUTTER *);

#endif //#ifndef _SPEED_H_

