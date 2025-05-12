/*******************************************************************************
* Filename: UserStackerTruckProc.c	                                           *
* Description: 逻辑层的源文件，包含对应的回调函数	   						   *
* Author:                                                      *
* Date: 2023/05/30    														   *
* Revision:	 														 		   *
*******************************************************************************/
#include "UserCanComm.h"
#include "USER_HANGUO_DGC.h"
#include "User_CanOpenHangChaApp.h"
#include "ErrCode.h"
#include "Eeprom.h"
#include "IQmathLib.h"
#include "PropProc.h"
#include "CanRevProc.h"
#include "string.h"
#include "Para.h"
#include "WdgProc.h"
#include "math.h"
#include "Log.h"
#include "LocalAi.h"
#include "LedProc.h"
#include "AiProc.h"
#include "LocalDo.h"
#include "HourCount.h"
#include "BatteryMeter.h"

#if (USER_TYPE == USER_HANGUO_JINJIDAO_DGC)

//转弯降速曲线表
const INT8U SteerAngle2Speedrate[6][90]=
{
	// 00,  01,  02,  03,  04,  05, 06,   07,  08,  09,
	{
	/**********************曲线一*********************************/
	   100, 100, 100, 100, 99,  98, 97,   96,  94,  92,		/* 0~9*/
	   90,  88,   86,  84, 82,  80, 79,   78,  77,  76,		/*10~19*/
	   75,  74,   73,  72, 71,  70, 69,   68,  67,  66,		/*20~29*/
	   65,  64,   63,  62, 61,  60, 59,   59,  58,  58,		/*30~39*/
	   57,  57,   56,  56, 56,  55, 55,   55,  54,  54,		/*40~49*/
	   54,  54,   53,  53, 53,  53, 53,   52,  52,  52,		/*50~59*/
	   52,  52,   51,  51, 51,  51, 51,   50,  50,  50,		/*60~69*/
	   50,  49,   49,  48, 48,  47, 47,   46,  46,  45,		/*70~79*/
	   45,  44,   44,  44, 43,  43, 43,   43,  43,  43,		/*80~89*/
	},
	{
	/**********************曲线二*********************************/
	   100, 100, 100, 100, 98,  96, 94,   92,  90,  88,		/* 0~9*/
	   86,  84,   83,  82, 81,  80, 79,   78,  77,  76,		/*10~19*/
	   75,  74,   73,  72, 71,  70, 69,   68,  67,  66,		/*20~29*/
	   65,  64,   63,  62, 61,  60, 59,   59,  58,  58,		/*30~39*/
	   57,  57,   56,  56, 56,  55, 55,   55,  54,  54,		/*40~49*/
	   54,  54,   53,  53, 53,  53, 53,   52,  52,  52,		/*50~59*/
	   52,  52,   51,  51, 51,  51, 51,   50,  50,  50,		/*60~69*/
	   50,  49,   49,  48, 48,  47, 47,   46,  46,  45,		/*70~79*/
	   45,  44,   44,  44, 43,  43, 43,   43,  43,  43,		/*80~89*/
	},
	{
	/**********************曲线三*********************************/
	   100, 100, 100, 99, 97,  95, 93,   90,  88,  86,		/* 0~9*/
	   84,  82,   80,  78, 76,  74, 72,   70,  68,  66,		/*10~19*/
	   64,  62,   60,  58, 56,  54, 52,   50,  49,  48,		/*20~29*/
	   47,  46,   45,  44, 43,  43, 43,   42,  42,  42,		/*30~39*/
	   42,  42,   42,  42, 42,  42, 42,   42,  42,  42,		/*40~49*/
	   42,  42,   42,  42, 42,  42, 42,   42,  42,  42,		/*50~59*/
	   42,  42,   42,  42, 42,  42, 42,   42,  42,  42,		/*60~69*/
	   42,  42,   42,  42, 42,  42, 42,   42,  42,  42,		/*70~79*/
	   42,  42,   42,  42, 42,  42, 42,   42,  42,  42,		/*80~89*/
	},
	{
	/**********************曲线四*********************************/
	   100, 100,  99,  98, 97,  95, 93,   90,  87,  84,		/* 0~9*/
	   81,  78,   75,  73, 71,  69, 67,   66,  65,  64,		/*10~19*/
	   63,  62,   61,  60, 59,  58, 57,   56,  55,  54,		/*20~29*/
	   53,  52,   51,  50, 50,  49, 49,   48,  48,  47,		/*30~39*/
	   47,  46,   46,  45, 45,  43, 43,   43,  43,  42,		/*40~49*/
	   42,  42,   42,  42, 42,  42, 42,   42,  42,  42,		/*50~59*/
	   42,  42,   42,  42, 42,  42, 42,   42,  42,  42,		/*60~69*/
	   42,  42,   42,  42, 42,  42, 42,   42,  42,  42,		/*70~79*/
	   42,  42,   42,  42, 42,  42, 42,   42,  42,  42,		/*80~89*/
	},
	{
	/**********************曲线五*********************************/
	   100, 100, 100,  99, 99,  98, 97,   96,  94,  92,		/* 0~9*/
	   90,  88,   86,  84, 82,  80, 78,   76,  74,  72,		/*10~19*/
	   70,  68,   66,  64, 62,  60, 58,   56,  54,  52,		/*20~29*/
	   50,  49,   48,  47, 47,  46, 46,   45,  45,  44,		/*30~39*/
	   44,  43,   43,  43, 42,  42, 42,   42,  42,  42,		/*40~49*/
	   42,  42,   42,  42, 42,  42, 42,   42,  42,  42,		/*50~59*/
	   42,  42,   42,  42, 42,  42, 42,   42,  42,  42,		/*60~69*/
	   42,  42,   42,  42, 42,  42, 42,   42,  42,  42,		/*70~79*/
	   42,  42,   42,  42, 42,  42, 42,   42,  42,  42,		/*80~89*/
	},
	{
	/**********************曲线六*********************************/
	   100, 100,  99,  99, 98,  98, 97,   96,  94,  92,		/* 0~9*/
	   90,  88,   86,  84, 82,  80, 78,   76,  74,  72,		/*10~19*/
	   70,  68,   66,  64, 62,  60, 58,   56,  54,  52,		/*20~29*/
	   50,  49,   48,  47, 47,  46, 46,   45,  45,  44,		/*30~39*/
	   44,  43,   43,  43, 42,  42, 42,   42,  42,  42,		/*40~49*/
	   42,  42,   42,  42, 42,  42, 42,   42,  42,  42,		/*50~59*/
	   42,  42,   42,  42, 42,  42, 42,   42,  42,  42,		/*60~69*/
	   42,  42,   42,  42, 42,  42, 42,   42,  42,  42,		/*70~79*/
	   42,  42,   42,  42, 42,  42, 42,   42,  42,  42,		/*80~89*/
	},
};

const static xPdoParameter  sgPdoPara = 
{
	.TpdoPara = {
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 20},
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 20},
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 20},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 50, .u16CanId = 0x380},
//		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 100, .u16CanId = 0x390},
	},
	.RpdoPara = {
//		{.b1Flag = 1, .b11CanRevId = 0x323},
		{.b1Flag = 1, .b11CanRevId = 0x2F4},
		{.b1Flag = 1, .b11CanRevId = 0x7F4},
	},
};

#define	Gear1_Spd_Con1		0
#define	Gear1_Spd_Con2		1
#define	Gear1_Spd_Con3		2
#define	Gear1_Spd_Con4		3

#define	Gear2_Spd_Con1		0
#define	Gear2_Spd_Con2		1
#define	Gear2_Spd_Con3		2
#define	Gear2_Spd_Con4		3

#define	Gear3_Spd_Con1		0
#define	Gear3_Spd_Con2		1
#define	Gear3_Spd_Con3		2
#define	Gear3_Spd_Con4		3

#define	Gear4_Spd_Con1		0
#define	Gear4_Spd_Con2		1
#define	Gear4_Spd_Con3		2
#define	Gear4_Spd_Con4		3

#define BmsLost_Limit (1 << 0)
#define HandleLost_Limit (1 << 1)
#define POWERON_CHECK_LIMIT (1 << 2)


typedef union
{
	uint16_t u16data;
	struct
	{
		uint16_t b1ModeSelc: 1;        
		uint16_t b1Ems: 1;                     //急反开关
		uint16_t b1SafeLock: 1;               //互锁
		uint16_t b1Forward: 1;
		uint16_t b1Backward: 1;
		uint16_t b1SlowMode: 1;
		uint16_t b1LiftUp: 1;
		uint16_t b1LiftDown: 1;
		uint16_t b1Brake: 1;
		uint16_t b1PropEn: 1;
		uint16_t b1QianHouQin: 1;
		uint16_t b1QianHouYi: 1;
		uint16_t b7Reserve: 4;
	};
}xSwiInput;

typedef union
{
	uint16_t u16data;
	struct
	{
		uint16_t b1SafeLock: 1;
		uint16_t b1Forward: 1;
		uint16_t b1Backward: 1;
		uint16_t b1SlowMode: 1;
		uint16_t b1Brake: 1;
		uint16_t b1Ems: 1;
		uint16_t b2Reserve: 2;
		
		uint16_t b1LiftUp: 1;
		uint16_t b1LiftDown: 1;
		uint16_t b1QianYi: 1;
		uint16_t b1HouYi: 1;
		uint16_t b1QianQin: 1;
		uint16_t b1HouQin: 1;	
		uint16_t b1ZuoYi: 1;
		uint16_t b1YouYi: 1;		
	};
}xHandleInput;



typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t b1NoActFlag: 1;
		uint8_t b1MoveFlag: 1;
		uint8_t b1LiftUpFlag: 1;
		uint8_t b1Gear1SpdFlag: 1;
		uint8_t b1Gear2SpdFlag: 1;
		uint8_t b1Gear3SpdFlag: 1;
		uint8_t b1Gear4SpdFlag: 1;		/*bms 低电量*/
		uint8_t b1Reserve1: 1;
		
		uint8_t b1ForWardStat: 1;
		uint8_t b1BackWardStat: 1;
		uint8_t b1LiftUpStat: 1;
		uint8_t b1LeanForWardStat: 1;
		uint8_t b1LeanBackWardStat: 1;
		uint8_t b1LiftDownStat: 1;
		uint8_t	b1PumpMotorNoAct: 1;
		uint8_t b1Reserve2: 1;
		
		uint8_t	b4Gear1Spd: 4;
		uint8_t	b4Gear2Spd: 4;
		uint8_t	b4Gear3Spd: 4;
		uint8_t	b4Gear4Spd: 4;
		
		uint8_t u8NoAct;
		
		uint8_t b4NoLiftUp: 4;
		uint8_t b4NoMove: 4;
		
		uint8_t u8Reserve4;
		uint8_t u8Reserve5;
	};
}xValvesInfo;


typedef struct
{
	uint16_t	u16StartAngle;
	uint16_t	u16EndAngle;
	uint16_t	u16StartAngleSpd;
	uint16_t	u16EndAngleSpd;
	float		fAgnleDecSpdFactor;
}xSteerAngleDecSpd;

typedef struct
{
	uint16_t	u16Gear1Spd;
	uint16_t	u16Gear2Spd;
	uint16_t	u16Gear3Spd;
	uint16_t	u16Gear4Spd;
	
	float		fLiftSpdPer5msAccStep;
	float		fDownSpdPer5msAccStep;
	float		fMoveSpdPer5msAccStep;
	
	float		fLiftSpdPer5msDecStep;
	float		fDownSpdPer5msDecStep;
	float		fMoveSpdPer5msDecStep;
	
	float		fPropMinCurrent;
	float		fPropMaxCurrent;
	
	float		fPropMinCurrent1;
	float		fPropMaxCurrent1;
	
	uint16_t	u16ThrottleMin;
	uint16_t	u16ThrottleMax;
	uint16_t	u16ThrottleRange;
	uint16_t	u16ThrottleMid;
	uint16_t	u16ThrottleMidValue;
	
	uint16_t	u16LiftUpMin;
	uint16_t	u16LiftUpMax;
	uint16_t	u16LiftUpRange;
	uint16_t	u16LiftUpMid;
	uint16_t	u16LiftUpMidValue;

	uint16_t	u16LiftDownMin;
	uint16_t 	u16LiftDownMax;
	uint16_t	u16LiftDownRange;
	uint16_t	u16LiftDownMid;
	uint16_t	u16LiftDownMidValue;
	
	uint8_t		u8LeanBackWardValue;
	uint8_t		u8LeanForWardValue;
	
	xSteerAngleDecSpd SteerAngleDecSpd;

	uint8_t		b1LiftMode: 1;
	uint8_t		b1RentalStop: 1;
	uint8_t		b1RentalStart: 1;
	uint8_t		b1RentalMode: 1;
	uint8_t		b1PasswordFunc: 1;
	uint8_t 	b1HourCntClr:1;
	uint8_t		b3Reserve: 2;	
	
	uint8_t		u8BatteryType;
	
	uint8_t		b1HourConutMode: 1;		/*0: 上电计时， 1：互锁计时*/
	uint8_t		b1StartUpLock: 1;		/*0： 检测， 1：不检测*/
	uint8_t		b1LiftLock: 1;			/*0： 需要互锁， 1：不需要*/
	uint8_t		b1LiftPedal: 1;			/*0： 无要求， 1：踏板open =1， close =0 / open =0， close =1 才可以动作*/
	uint8_t		b1MoveLiftMode: 1;		/*0： 同时动作， 1：互斥*/
	uint8_t		b3Reserve1: 3;
	
	uint16_t	u16RatioOfTransmission;
	uint16_t	u16MotorMaxSpd;
	
	uint16_t	u16RentalTime;
	
	uint8_t u8SOCLimit;
	int16_t Temputer;
	uint8_t u8Language;
	uint8_t HourCountMode;
	uint16_t u16BrakePedalType;
	
	uint16_t	u16BrakePedalRangeL;
	uint16_t	u16BrakePedalRangeH;
	
	uint16_t u16QianHouQinSpeed;
	uint16_t u16QianHouYiSpeed;
	
	uint16_t u16ThrottlePedalType;
}xUserInfo;

typedef union
{
	uint8_t u8Data;
	struct
	{
		uint8_t b1ServoOn: 1;
		uint8_t b1PowerLineOn: 1;
		uint8_t b1BrakeReq: 1;
		uint8_t b1ForwardReq: 1;
		uint8_t b1BackwardReq: 1;
		uint8_t b1LiftReq: 1;
		uint8_t b1DownReq: 1;
		uint8_t b1EmsReq: 1; 
	};
}xMstSendStat;

static int16_t	i16MotorSpd = 0;
static uint16_t u16MotorVal = 0;
static uint8_t	u8PumpValue = 0;
static uint8_t	u8PropValue = 0;
static uint16_t u16CanRev62CCnt = 0;
static xValvesInfo sgValvesInfo;
static xUserInfo sgUserInfo;
static xSwiInput sgSwiInput;
static xHandleInput sgHandleInput;
static uint32_t u32HourCount = 0;
static int16_t i16SteerAngle = 0;
static uint16_t DRIVER_FLAG = 0;
static uint8_t check_err = 0;
static uint32_t sgu16LimitFlg = 0;
static uint32_t u32WorkCount = 0;
static uint32_t Workcnt = 0;
static uint8_t u8PowerLowLimitS = 0;
static uint8_t u8PowerLowLimitL = 0;
static uint8_t u8LimitMove = 0;
static uint8_t u8EmsErrFlg = 0;
static uint16_t Rev_Speed;
static uint8_t SpeedRate = 0;
static uint8_t SpeedRate1 = 0;
static uint16_t RevSPEED = 0;
static uint8_t DelayFlg = 0;
static uint8_t EmsOk = 0;
static uint8_t HumanDelay = 0;
static uint8_t EMSRecive = 0;
static uint16_t steerAngleTab = 0;

static uint8_t SwiInputMode = 0;
static uint16_t CanId390SendCout = 0;
static uint32_t CanId323LostChk = 0;

static uint32_t MotorOdm = 0;
static uint32_t MotorOdmClc = 0;
static uint8_t MotorOdmSave = 0;

/*PDO接收待完善*/
xCanRevPdoInfo gCanRevPdoInfo; 
static xCanRev323Info CanRev323InfoLast;
static xCanRev2F4Info CanRev2F4InfoLast;
static xCanRev7F4Info CanRev7F4InfoLast;
/*PDO发送待完善*/
xCanSendPdoInfo gCanSendPdoInfo;
//static xCanSend5C0Info CanSend5C0Last;

static void vMstRevProc(xMstRevPara *RevData)
{	
	uint16_t tmp = 0;
	static uint8_t Delay = 0;
	static uint16_t TmpLast = 0;
	int16_t i16MotorCurrent = 0;
	
	if(0 != RevData->b1MainDriver)
	{

	}
	else
	{

	}
	
	if(0 != RevData->b1Ebrake)
	{
		
	}
	else
	{	
		
	}
	
	if(0 != RevData->b1Driver3State)
	{
		
	}
	else
	{
		
	}
	/*故障码处理*/
	if(0 != RevData->u8ErrCode)
	{
		if (u8ErrCodeGet() < RevData->u8ErrCode)   /*lilu, 20230818, 清除MCU之前更小的故障码*/
		{
			uint8_t i = 0;
			for (i=0; i<RevData->u8ErrCode; i++)
			{
				i32ErrCodeClr(i);
			}
			
		}
		if(40 != RevData->u8ErrCode)
		{
			i32ErrCodeSet(RevData->u8ErrCode - 1);
		}		
	}
	else
	{
		if (u8ErrCodeGet() < 50)				/*lilu, 20230817, 没有MCU的故障的时候，就清除MCU的所有故障码*/
		{
			uint8_t i = 0;
			for (i=0; i<50; i++)
			{
				i32ErrCodeClr(i);
			}
		}
	}
	
	i16MotorSpd = (uint16_t)(RevData->u8SpeedFdbHigh << 8) | RevData->u8SpeedFdbLow ;
	tmp = abs((i16MotorSpd * sgUserInfo.u16RatioOfTransmission)>>15);
	__disable_irq();
	gCanSendPdoInfo.CanSend1A3Info.ActualRPM = i16MotorSpd;
	gCanSendPdoInfo.CanSend380Info.Speed = tmp/10;
	__enable_irq();
	
	if(MotorOdmClc < 2000)
	{
		MotorOdmClc++;
	}
	else
	{
		MotorOdm +=(tmp * 277 / 100);
		TmpLast = tmp;
		MotorOdmClc = 0;
		u16EepromWrite(PARA_AngleValue6, MotorOdm, 1);
	}
	
	if((MotorOdm % 10) == 0)
	{
		gCanSendPdoInfo.CanSend380Info.Odometer1 = (MotorOdm/10) & 0xFF;
		gCanSendPdoInfo.CanSend380Info.Odometer2 = ((MotorOdm/10)>>8) & 0xFF;
		gCanSendPdoInfo.CanSend380Info.Odometer3 = ((MotorOdm/10)>>16) & 0xFF;
	}
	
	tmp = RevData->u8MotorTmp;
	__disable_irq();
	gCanSendPdoInfo.CanSend1A3Info.MotorTemperature = tmp;
	__enable_irq();
	/*Motor Temp*/
	
	tmp = RevData->u8BoardTmp;
	__disable_irq();
	gCanSendPdoInfo.CanSend1A3Info.ControllerTemperature = tmp;
	__enable_irq();
	/**/
	
	i16MotorCurrent = (RevData->u8CurrentHigh << 8) | RevData->u8CurrentLow;
	gCanSendPdoInfo.CanSend1A3Info.MotorCurrent = i16MotorCurrent;
	
	if(0 != RevData->u8Reserve3)
	{
		gCanSendPdoInfo.CanSend2A3Info.PumpCurrent = RevData->u8Reserve3;
	}
}

static void vMoveModeNoChangeProc(int16_t *i16Spd, const xValvesInfo *pValvesInfo)
{
	float fTmp = 0;
	if ((1 == pValvesInfo->b1ForWardStat) || (1 == pValvesInfo->b1BackWardStat))
	{
		if (*i16Spd > u16MotorVal)
		{
			fTmp = sgUserInfo.fMoveSpdPer5msDecStep * (0.01 + 0.99 * *i16Spd / MOTOR_SPEED_RANGE);
			if ((((*i16Spd - u16MotorVal) > fTmp) && (0 != sgUserInfo.fMoveSpdPer5msDecStep)))
			{
				*i16Spd -= fTmp;
			}
			else
			{
				*i16Spd = u16MotorVal;
			}
		}
		else if (*i16Spd < u16MotorVal)
		{
			fTmp = sgUserInfo.fMoveSpdPer5msAccStep * (0.01 + 0.99 * *i16Spd / MOTOR_SPEED_RANGE);
			if (((u16MotorVal - *i16Spd) > fTmp) && (0 != sgUserInfo.fMoveSpdPer5msAccStep))
			{
				*i16Spd += fTmp;
			}
			else
			{
				*i16Spd = u16MotorVal;
			}
		}
	}
	else
	{
		*i16Spd = 0;
	}
}

/*发送给Mst的回调函数*/
static void vMstSendProc(xMstSendPara *SendData) 
{
	uint16_t LiftUpValApp = 0;
	uint16_t LiftDownValApp = 0;
	uint8_t WorkMode = 0;
	static xValvesInfo sgLastValvesInfo;
	static uint8_t u8MoveSwitchFlag = 0;
	static uint32_t u16MainConnectCnt = 0;
	static uint32_t u32HumanDelay = 0;
	float fTmp = 0.0;
	xMstSendStat LastStatus;
	int16_t i16Spd = 0;
	int32_t i32PropValue = 0;
	static uint16_t u16LiftUpCnt = 0; 
	static uint8_t u8EmsDelay = 0;
	static uint16_t u16PropStartDelay = 0;
	static uint16_t u16PropStopDelay = 4096;
	uint16_t PropGUding = 0;
	static uint8_t u8PropTmp = 0;
	static uint16_t EmsCountTime = 0;

	LastStatus.u8Data = SendData->buf[2];
	SendData->buf[2] = 0;

	/*Move Mode*/
		SendData->b1ServoOn = 1;
#ifndef JINJIDAO_DGC_IO_TYPE
		i32DoPwmSet(DIDI_VALVE,DRIVER_CLOSE);
#else
		i32DoPwmSet(BackWard_VALVE,DRIVER_CLOSE);
		i32DoPwmSet(Direction_VALVE,DRIVER_CLOSE);
#endif
		gCanSendPdoInfo.CanSend380Info.FWD = 0;
		gCanSendPdoInfo.CanSend380Info.REV = 0;
		if(sgSwiInput.b1Backward == 1)
		{
			i32DoPwmSet(BackWard_VALVE,DRIVER_OPEN);
			i32DoPwmSet(Direction_VALVE,DRIVER_OPEN);
			gCanSendPdoInfo.CanSend380Info.REV = 1;
			SendData->b1ForwardReq = 1;
		}
		else if(sgSwiInput.b1Forward == 1)
		{
			gCanSendPdoInfo.CanSend380Info.FWD = 1;
			i32DoPwmSet(Direction_VALVE,DRIVER_OPEN);
#ifndef JINJIDAO_DGC_IO_TYPE
			i32DoPwmSet(DIDI_VALVE,DRIVER_OPEN);
#endif
			SendData->b1BackwardReq = 1;
		}
		
		if((0 != sgSwiInput.b1Backward)||(0 != sgSwiInput.b1Forward))
		{
			SendData->u8TargetHigh = u16MotorVal >> 8;
			SendData->u8TargetLow = u16MotorVal;
		}
		else
		{
			SendData->u8TargetHigh = 0;
			SendData->u8TargetLow = 0;
		}

		if(0 != sgUserInfo.b1StartUpLock)
		{
			if ((1 == sgSwiInput.b1Ems) && (1 == SendData->b1ForwardReq))
			{
				SendData->b1EmsReq = 1;
				EmsCountTime = 2000/5;
			}
			else if(1 == sgSwiInput.b1Ems)
			{
				SendData->b1ServoOn = 0;
			}
		}
		else
		{
			if (1 == sgSwiInput.b1Ems)  
			{
				SendData->b1ServoOn = 0;
			}
		}
		
		if(0 != EmsCountTime)
		{
			EmsCountTime--;
			SendData->b1EmsReq = 1;
		}
		
//		if((1 == sgSwiInput.b1Ems)&&(0 == u16MotorVal))
//		{
//			SendData->b1ServoOn = 0;
//		}

//		if((0 == sgSwiInput.b1Ems)&&((0 == sgSwiInput.b1Forward)||(0 == sgSwiInput.b1Backward)))
//		{
//			if(EmsOk == 1)
//			{
//				i32ErrCodeClr(39);
//			}
//		}
		
//		{
//			if((1 == sgSwiInput.b1Ems)&&(0 != u16MotorVal)&&(0 == Rev_Speed)&&(1 == sgSwiInput.b1Backward))
//			{
//				i32ErrCodeSet(39);
//				EMSRecive = 1;
//			}
//			else if((0 == sgSwiInput.b1Ems)&&(0 == u16MotorVal))
//			{
//				if(1 == EMSRecive)
//				{
//					EMSRecive = 0;
//					i32ErrCodeClr(39);
//				}
//			}
//		}
		
		SendData->b1LiftReq = 0;
		SendData->u8PumpTarget = 0;
		vPropSetTarget(LIFTDOWN_VALVE, 0);
#ifndef JINJIDAO_DGC_IO_TYPE
		i32DoPwmSet(LIFTUP_VALVE,DRIVER_CLOSE);
		i32DoPwmSet(QianQin_VALVE,DRIVER_CLOSE);
		i32DoPwmSet(HOUQin_VALVE,DRIVER_CLOSE);
		i32DoPwmSet(FRONT_VALVE,DRIVER_CLOSE);
		i32DoPwmSet(BACK_VALVE,DRIVER_CLOSE);
#endif

	if((0 == sgSwiInput.b1LiftUp)&&(1 == sgSwiInput.b1LiftDown))
	{
		i32PropValue = _IQ((sgUserInfo.fPropMinCurrent + (sgUserInfo.fPropMaxCurrent - sgUserInfo.fPropMinCurrent) * u8PropValue / PUMP_RANGE) / PROPD_STD_CURRENT);
		vPropSetTarget(LIFTDOWN_VALVE, i32PropValue);
#ifndef JINJIDAO_DGC_IO_TYPE
		i32DoPwmSet(LIFTUP_VALVE,DRIVER_CLOSE);
#endif
	}
	else if((1 == sgSwiInput.b1LiftUp)&&(0 == sgSwiInput.b1LiftDown))
	{
		SendData->b1LiftReq = 1;
		SendData->u8PumpTarget = u8PumpValue;
		vPropSetTarget(LIFTDOWN_VALVE, 0);
#ifndef JINJIDAO_DGC_IO_TYPE
		i32DoPwmSet(LIFTUP_VALVE,DRIVER_OPEN);
#endif
	}
	
	if(((0 == sgHandleInput.b1HouQin)&&(1 == sgHandleInput.b1QianQin))||((0 == SwiInputMode)&&(sgSwiInput.b1QianHouQin)))
	{
		SendData->b1LiftReq = 1;
		if(0 == SwiInputMode)
		{
			if(SendData->u8PumpTarget < (255 * sgUserInfo.u16QianHouQinSpeed / 100))
				SendData->u8PumpTarget = 255 * sgUserInfo.u16QianHouQinSpeed / 100;   //
		}
		else
		{
			if(SendData->u8PumpTarget < CanRev323InfoLast.u8QianHouQinThrottle)
				SendData->u8PumpTarget = CanRev323InfoLast.u8QianHouQinThrottle;
		}	
#ifndef JINJIDAO_DGC_IO_TYPE
		i32DoPwmSet(QianQin_VALVE,DRIVER_OPEN);
#endif
	}
	else if((1 == sgHandleInput.b1HouQin)&&(0 == sgHandleInput.b1QianQin))
	{
		SendData->b1LiftReq = 1;
		if(SendData->u8PumpTarget < CanRev323InfoLast.u8QianHouQinThrottle)
			SendData->u8PumpTarget = CanRev323InfoLast.u8QianHouQinThrottle;
#ifndef JINJIDAO_DGC_IO_TYPE
		i32DoPwmSet(HOUQin_VALVE,DRIVER_OPEN);
#endif
	}
	
	if(((0 == sgHandleInput.b1QianYi)&&(1 == sgHandleInput.b1HouYi))||((0 == SwiInputMode)&&(sgSwiInput.b1QianHouYi)))
	{
		SendData->b1LiftReq = 1;
		if(0 == SwiInputMode)
		{
			if(SendData->u8PumpTarget < (255 * sgUserInfo.u16QianHouYiSpeed / 100))
				SendData->u8PumpTarget = 255 * sgUserInfo.u16QianHouYiSpeed / 100;   //
		}
		else
		{
			if(SendData->u8PumpTarget < CanRev323InfoLast.u8QianHouYiThrottle)
				SendData->u8PumpTarget = CanRev323InfoLast.u8QianHouYiThrottle;
		}	
#ifndef JINJIDAO_DGC_IO_TYPE		
		i32DoPwmSet(FRONT_VALVE,DRIVER_OPEN);
#endif
	}
	else if((1 == sgHandleInput.b1QianYi)&&(0 == sgHandleInput.b1HouYi))
	{
		SendData->b1LiftReq = 1;
		if(SendData->u8PumpTarget < CanRev323InfoLast.u8QianHouYiThrottle)
			SendData->u8PumpTarget = CanRev323InfoLast.u8QianHouYiThrottle;
#ifndef JINJIDAO_DGC_IO_TYPE
		i32DoPwmSet(BACK_VALVE,DRIVER_OPEN);
#endif
	}
	
	if((1 == sgHandleInput.b1ZuoYi)||(1 == sgHandleInput.b1YouYi))
	{
		SendData->b1LiftReq = 1;
		if(SendData->u8PumpTarget < CanRev323InfoLast.u8ZuoYouYiThrottle)
			SendData->u8PumpTarget = CanRev323InfoLast.u8ZuoYouYiThrottle;
	}
	
	if(0 != sgSwiInput.b1PropEn)
	{
		SendData->b1LiftReq = 0;
		SendData->u8PumpTarget = 0;
		vPropSetTarget(LIFTDOWN_VALVE, 0);
	}

	if(0 != sgSwiInput.b1Brake)
	{
		SendData->b1ForwardReq = 0;
		SendData->b1BackwardReq = 0;
		SendData->u8TargetHigh = 0;
		SendData->u8TargetLow = 0;
	}
	
	if ((0 == sgSwiInput.b1SafeLock)&&((0 == i32LocalDiGet(TURTLE_SWI))))
	{
//		SendData->b1PowerLineOn = 1;     // 1:断开 0:连接
		SendData->b1ServoOn = 0;
		SendData->b1EmsReq = 0;
		SendData->b1ForwardReq = 0;
		SendData->b1BackwardReq = 0;
	}
	
	if(((0 == sgSwiInput.b1SafeLock)&&((0 == i32LocalDiGet(TURTLE_SWI))))&&((0 != sgSwiInput.b1Forward)||(0 != sgSwiInput.b1Backward)))
	{
		i32ErrCodeSet(SWILOGIC_WORING);
	}
	
	if((1 == sgSwiInput.b1Ems)&&((0 == sgSwiInput.b1Forward)&&(0 == sgSwiInput.b1Backward)))
	{
		i32ErrCodeSet(SWILOGIC_WORING);
	}
	
	if((0 != sgSwiInput.b1Forward)&&(0 != sgSwiInput.b1Backward))
	{
		i32ErrCodeSet(SWILOGIC_WORING);
	}
	
	if((0 == sgSwiInput.b1Forward)&&(0 == sgSwiInput.b1Backward)&&(0 == sgSwiInput.b1Ems))
	{
		i32ErrCodeClr(SWILOGIC_WORING);
	}
	
	if((0 != i32ErrCodeCheck(SWILOGIC_WORING))||(0 != i32ErrCodeCheck(109)))
	{
		SendData->b1ServoOn = 0;
		SendData->b1EmsReq = 0;
		SendData->b1ForwardReq = 0;
		SendData->b1BackwardReq = 0;
		SendData->b1LiftReq = 0;
		SendData->u8PumpTarget = 0;
	}
	
	
	if(0 != sgValvesInfo.u8NoAct)
	{
		SendData->b1ServoOn = 0;
		SendData->b1EmsReq = 0;
		SendData->b1ForwardReq = 0;
		SendData->b1BackwardReq = 0;
		SendData->b1LiftReq = 0;
		SendData->u8PumpTarget = 0;
	}
	
	gCanSendPdoInfo.CanSend1A3Info.KSI = i32LocalAiGetValue(AI_B_KSI_CHECK) / 10;
	gCanSendPdoInfo.CanSend3A3Info.SWITCH = *(uint8_t *)u16pGetParaPoint(PARA_DoSwi);
	gCanSendPdoInfo.CanSend3A3Info.POTWIFER1 = i32LocalAiGetValue(AI_B_AI1_R) / 10;
	gCanSendPdoInfo.CanSend3A3Info.POTWIFER2 = i32LocalAiGetValue(AI_B_AI2_R) / 10;
	gCanSendPdoInfo.CanSend3A3Info.POTWIFER3 = i32LocalAiGetValue(AI_B_AI3_R) / 10;
	
		{			
			/*lilu 20230819 For Test*/
			{
				i32SetPara(PARA_ForwardValveCurrent, u8PropTmp);		/*Send Motor Value*/
				i32SetPara(PARA_BackValveCurrent, LiftDownValApp);		/*Rev Motor Value*/
				i32SetPara(PARA_PropValveCurrent, sgHandleInput.u16data);	/*Prop Current*/
//				i32SetPara(PARA_EcuLockState, Hour2App);	//上电计时	
//				i32SetPara(PARA_TmpLockState, Work2App);	
//				i32SetPara(PARA_SelfHeartQuery, HourCntFlg);	
//				i32SetPara(PARA_PlatfromHeartQuery, WorkCntFlg);
//				i32SetPara(PARA_LiftValveCurrent, u8PumpOrPropValueMove);		/*Send Pump Value*/
				i32SetPara(PARA_OnOffValveCurrent, SendData->buf[2]);			/*Send Status*/
				i32SetPara(PARA_MotorSpd, RevSPEED);				/*Valve NoFlag*/
				i32SetPara(PARA_CalibrationStatus, sgValvesInfo.u8Data[1]);		/*Value State*/
				i32SetPara(PARA_TurnRightValveCurrent, ((uint16_t)sgValvesInfo.u8NoAct << 8) | ((uint16_t)sgValvesInfo.b4NoMove << 4) | ((uint16_t)sgValvesInfo.b4NoLiftUp));
				i32SetPara(PARA_TurnLeftValveCurrent, ((uint16_t)sgValvesInfo.b4Gear1Spd << 12) | ((uint16_t)sgValvesInfo.b4Gear2Spd << 8) | \
					((uint16_t)sgValvesInfo.b4Gear3Spd << 4) | ((uint16_t)sgValvesInfo.b4Gear4Spd));
//				i32SetPara(PARA_AngleValue, (uint16_t)(CanRev1E0InfoLast.u8MotorValueH << 8)|(CanRev1E0InfoLast.u8MotorValueL));		/*AI1*/
//				i32SetPara(PARA_PressureVlaue1, (CanRev1E0InfoLast.u8LiftValueH << 8)|CanRev1E0InfoLast.u8LiftValueL);	/*AI2*/
//				i32SetPara(PARA_PressureVlaue2, CanRev1E0InfoLast.u8QianHouYiH);	/*AI3*/
				i32SetPara(PARA_ExtSignal, sgSwiInput.u16data);						/*Swi*/
				i32SetPara(PARA_PcuKeyInfo,sgHandleInput.u16data);						/*ErrCode*/
//				i32SetPara(PARA_BmsSoc, gCanSendPdoInfo.CanSend258Info.SOC);		/*BMS SOC*/
				i32SetPara(PARA_SteerAngle, abs(i16SteerAngle)*10);
				
			}
		}
}

static void vDoPwmErrCallBack(eDoPwmNo DoPwmNo)
{
	switch((uint8_t)DoPwmNo)
	{

	}
}

static void vPropErrCallBack(uint8_t u8Channel)
{
	switch(u8Channel)
	{

	}
}

static void vAiErrCallBack(eAiErrNo AiErrNo)
{
	switch((uint8_t)AiErrNo)
	{
		case AI_B_AI1_R_ERR:
			i32ErrCodeSet(AI_B_AI1_ERR);
			break;
		case AI_B_AI2_R_ERR:
			i32ErrCodeSet(AI_B_AI2_ERR);
			break;
		case AI_B_AI3_R_ERR:
			i32ErrCodeSet(AI_B_AI3_ERR);
			break;
		case AI_5V_12V_OUT1_I_ERR:
			i32ErrCodeSet(AI_5V_12V_OUT1_ERR);
			break;
		case AI_5V_12V_OUT2_I_ERR:
//			i32ErrCodeSet(AI_5V_12V_OUT2_ERR);
			break;
		default:
			break;
										
	}
}

/*******************************************************************************
* Name: vCanLostProc(uint32_t u32CanID,uint8_t u8State)
* Descriptio: 正常通道ID掉线检测
* Input: NULL
* Output: NULL  
*******************************************************************************/
void vCanLostProc(uint32_t u32CanID,uint8_t u8State)
{
	switch(u32CanID)
	{
//		case 0x323:
//			if(CAN_NORMAL == u8State)
//			{
//				if(0 != SwiInputMode)
//				{
//					sgValvesInfo.u8NoAct &= ~BmsLost_Limit;
//				}
//			}
//			else if(CAN_LOST == u8State)
//			{
//				if(0 != SwiInputMode)
//				{
//					sgValvesInfo.u8NoAct |= BmsLost_Limit;
//				}			
//			}
//			break;
		case 0x2F4:
			if(CAN_NORMAL == u8State)
			{
				if(0 == sgUserInfo.u8BatteryType)
				{
					sgValvesInfo.u8NoAct &= ~HandleLost_Limit;
				}
			}
			else if(CAN_LOST == u8State)
			{
				if(0 == sgUserInfo.u8BatteryType)
				{
					sgValvesInfo.u8NoAct |= HandleLost_Limit;
				}			
			}
			break;
		default:
			break;
		
		if(0 != (sgValvesInfo.u8NoAct & (HandleLost_Limit|BmsLost_Limit)))
		{
			i32ErrCodeSet(109);
		}
		else
		{
			i32ErrCodeClr(109);
		}
	
	}
}

static void vCanId323Proc(tCanFrame * CanFrame)
{
	/***** 转向323 *******/
	{
		memcpy((char*)CanRev323InfoLast.u8Data, (char*)CanFrame->u8Data, sizeof(CanRev323InfoLast));
		/*添加相关操作*/
		sgHandleInput.u16data = CanRev323InfoLast.u8MoveSwiInput | (CanRev323InfoLast.u8LiftSwiInput<<8);
		if(0 != SwiInputMode)
		{
			gCanSendPdoInfo.CanSend2A3Info.ThrottleOutput = CanRev323InfoLast.u8MoveThrottle * 32767 / 255;
			gCanSendPdoInfo.CanSend2A3Info.PumpVoltage = CanRev323InfoLast.u8LiftThrottle;
			u16MotorVal = CanRev323InfoLast.u8MoveThrottle * 4095 / 255;
			u8PumpValue = CanRev323InfoLast.u8LiftThrottle;
			u8PropValue = CanRev323InfoLast.u8DownThrottle;
		}
	}
	CanId323LostChk = 0;
}


/*接收PDO处理函数*/
static void vCanRevPdoProc(void)
{
	
	if(0 == sgUserInfo.u8BatteryType)
	{
		{
			memcpy((char*)CanRev2F4InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo2F4.u8Data, sizeof(CanRev2F4InfoLast));
			/*添加相关操作*/
			gCanSendPdoInfo.CanSend380Info.BatterSoc = CanRev2F4InfoLast.BatterySOC;
			gCanSendPdoInfo.CanSend380Info.Batterytype = 1;
		}
		
		{
			memcpy((char*)CanRev7F4InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo7F4.u8Data, sizeof(CanRev7F4InfoLast));
		}
		
	}
	else
	{
		gCanSendPdoInfo.CanSend380Info.BatterSoc = u8GetBatterySoc();
		gCanSendPdoInfo.CanSend380Info.Batterytype = 0;
	}
}




/*add Turn with Dec Spd*/
static uint16_t	u16SteerAngleDecSpd(uint16_t u16Spd, uint16_t u16Angle, const xSteerAngleDecSpd *Info)
{
	uint16_t u16Res = 0;
	uint16_t u16Tmp = 0;
	
	if (u16Angle <= Info->u16StartAngle)
	{
		u16Res = u16Spd;
	}
	else if (u16Angle >= Info->u16EndAngle)
	{
		if (u16Spd >= Info->u16EndAngleSpd)
		{
			u16Res = Info->u16EndAngleSpd;
		}
		else
		{
			u16Res = u16Spd;
		}
	}
	else
	{
		u16Tmp = Info->u16StartAngleSpd - (u16Angle - Info->u16StartAngle) * Info->fAgnleDecSpdFactor;
		if (u16Spd > u16Tmp)
		{
			u16Res = u16Tmp;
		}
		else
		{
			u16Res = u16Spd;
		}
	}
	
	return u16Res;
}


/*lilu 20230703 模拟量监控*/
static void vAiMonitor(void)
{
	uint8_t u8SpeedRate = 100;
	int32_t i32AdcValue = 0;
	static uint16_t speedRateSteer = 0;
	
/*********** Motor Speed *************/
	if(0 != sgUserInfo.u16ThrottlePedalType)
	{
		i32AdcValue = i32LocalAiGetValue(MOVE_THROTTLE);
		if(i32AdcValue > sgUserInfo.u16ThrottleMax)
		{
			u16MotorVal = 4096;
		}
		else if((i32AdcValue > sgUserInfo.u16ThrottleMin)&&(i32AdcValue <= sgUserInfo.u16ThrottleMax))
		{
			u16MotorVal = (((i32AdcValue - sgUserInfo.u16ThrottleMin)*4096)/(sgUserInfo.u16ThrottleMax-sgUserInfo.u16ThrottleMin));
		}
		else
		{
			u16MotorVal = 0;
		}
	}
	gCanSendPdoInfo.CanSend2A3Info.ThrottleOutput = u16MotorVal * 32767 / 4096;
	
	
	
	/*add spd limit*/
	if (1 == sgValvesInfo.b1Gear1SpdFlag)
	{
		if (u8SpeedRate >= sgUserInfo.u16Gear1Spd)
		{
			u8SpeedRate = sgUserInfo.u16Gear1Spd;
		}
	}
	
	if (1 == sgValvesInfo.b1Gear2SpdFlag)
	{
		if (u8SpeedRate >= sgUserInfo.u16Gear2Spd)
		{
			u8SpeedRate = sgUserInfo.u16Gear2Spd;
		}
	}
	
	if (1 == sgValvesInfo.b1Gear3SpdFlag)
	{
		if (u8SpeedRate >= sgUserInfo.u16Gear3Spd)
		{
			u8SpeedRate = sgUserInfo.u16Gear3Spd;
		}
	}
	
	if (1 == sgValvesInfo.b1Gear4SpdFlag)
	{
		if (u8SpeedRate >= sgUserInfo.u16Gear4Spd)
		{
			u8SpeedRate = sgUserInfo.u16Gear4Spd;
		}
	}
	if(SpeedRate > 0)
	{
		if (u8SpeedRate >= SpeedRate)
		{
			u8SpeedRate = SpeedRate;
		}
	}
	
	if(SpeedRate1 > 0)
	{
		if (u8SpeedRate >= SpeedRate1)
		{
			u8SpeedRate = SpeedRate1;
		}
	}
		
	/*add Turn Dec Spd*/
	//根据角度值，查表得转弯降速比例
	{
		uint16_t steerAngle = 0;
		
		steerAngle = abs(i16SteerAngle)/10;
		if(steerAngle > (sizeof(SteerAngle2Speedrate[0])-1))
			steerAngle = sizeof(SteerAngle2Speedrate[0])-1;
		
		if(steerAngleTab > 5)
			steerAngleTab = 5;
			
		if((steerAngleTab > (sizeof(SteerAngle2Speedrate)/sizeof(SteerAngle2Speedrate[0]))) \
		  || (steerAngleTab <=0)
		  )
		{
			speedRateSteer = SteerAngle2Speedrate[0][steerAngle];
		}
		else
		{
			speedRateSteer = SteerAngle2Speedrate[steerAngleTab-1][steerAngle];
		}
		
		if(speedRateSteer < u8SpeedRate)
			u8SpeedRate = speedRateSteer;
	}
	
	u16MotorVal = (uint16_t)((u16MotorVal * u8SpeedRate)/100);
		
	if(0 != sgUserInfo.u16BrakePedalType)
	{
		i32AdcValue = i32LocalAiGetValue(LIFT_THROTTLE);
		gCanSendPdoInfo.CanSend2A3Info.PumpVoltage = i32AdcValue /10;
		
		if(sgUserInfo.u16ThrottleMax >= sgUserInfo.u16LiftUpMin)	//正斜率
		{
			if(i32AdcValue < sgUserInfo.u16LiftUpMin)
			{
				u8PumpValue = 0;
			}
			else if((i32AdcValue >= sgUserInfo.u16LiftUpMin)&&(i32AdcValue < sgUserInfo.u16LiftUpMid))
			{
				u8PumpValue = ((((i32AdcValue - sgUserInfo.u16LiftUpMin)*sgUserInfo.u16BrakePedalRangeL)/_IQ8(1))*255)/100;
			}
			else if((i32AdcValue >= sgUserInfo.u16LiftUpMid)&&(i32AdcValue < sgUserInfo.u16ThrottleMax))
			{
				u8PumpValue = ((i32GetPara(MOVE_THROTTLE_MID) + ((i32AdcValue - sgUserInfo.u16LiftUpMid)*sgUserInfo.u16BrakePedalRangeH)/_IQ8(1))*255)/100;
			}
			else if(i32AdcValue >= sgUserInfo.u16ThrottleMax)
			{
				u8PumpValue=255;
			}
		}
		else	//负斜率
		{
			if(i32AdcValue>sgUserInfo.u16LiftUpMin)
			{
				u8PumpValue=0;
			}
			else if((i32AdcValue<=sgUserInfo.u16LiftUpMin)&&(i32AdcValue>sgUserInfo.u16LiftUpMid))
			{
				u8PumpValue=((((i32AdcValue-sgUserInfo.u16LiftUpMin)*sgUserInfo.u16BrakePedalRangeL)/_IQ8(1))*255)/100;
			}
			else if((i32AdcValue<=sgUserInfo.u16LiftUpMid)&&(i32AdcValue>sgUserInfo.u16ThrottleMax))
			{
				u8PumpValue=((i32GetPara(MOVE_THROTTLE_MID)+((i32AdcValue-sgUserInfo.u16LiftUpMid)*sgUserInfo.u16BrakePedalRangeH)/_IQ8(1))*255)/100;
			}
			else if(i32AdcValue<=sgUserInfo.u16ThrottleMax)
			{
				u8PumpValue=255;
			}
		}
	}
	else
	{
			u8PumpValue=255;
	}
	
	u8PropValue = u8PumpValue;
//		if(i32AdcValue > sgUserInfo.u16LiftUpMax)
//		{
//			u8PropValue = 0;
//			u8PumpValue = 255;
//		}
//		else if((i32AdcValue >= ((sgUserInfo.u16LiftUpMax+sgUserInfo.u16LiftUpMin)/2 + sgUserInfo.u16LiftUpMid))&&(i32AdcValue <= sgUserInfo.u16LiftUpMax))
//		{
//			u8PropValue = 0;
//			u8PumpValue = (255*(i32AdcValue-((sgUserInfo.u16LiftUpMax+sgUserInfo.u16LiftUpMin)/2 + sgUserInfo.u16LiftUpMid)))/(sgUserInfo.u16LiftUpMax-((sgUserInfo.u16LiftUpMax+sgUserInfo.u16LiftUpMin)/2 + sgUserInfo.u16LiftUpMid));
//		}
//		else if((i32AdcValue <= ((sgUserInfo.u16LiftUpMax+sgUserInfo.u16LiftUpMin)/2 - sgUserInfo.u16LiftUpMid))&&(i32AdcValue > sgUserInfo.u16LiftUpMin))
//		{
//			u8PumpValue = 0;
//			u8PropValue = (255*(((sgUserInfo.u16LiftUpMax+sgUserInfo.u16LiftUpMin)/2 + sgUserInfo.u16LiftUpMid)) - i32AdcValue)/(((sgUserInfo.u16LiftUpMax+sgUserInfo.u16LiftUpMin)/2 + sgUserInfo.u16LiftUpMid) - sgUserInfo.u16LiftUpMin);
//		}
//		else
//		{
//			u8PumpValue = 0;
//			u8PropValue = 0;
//		}
//	}
//	else
//	{
//		u8PumpValue = 255;
//		u8PropValue = 255;
//	}
	
}


/*开关量监控*/
static void vSwiMonitor(void)
{
	xSwiInput SwiInput = {0};
	static uint8_t Snail_Count = 0;
	static uint8_t SlowMode = 0;
	
	SwiInput.b1ModeSelc = SwiInputMode;
	
	if(0 == i32LocalDiGet(BELLYEms_SWI))
	{
		SwiInput.b1Ems = 1;
	}
	
	if(1 == i32LocalDiGet(SAFELOCK_SWI))
	{
		SwiInput.b1SafeLock = 1;
	}
	
	if(1 == i32LocalDiGet(FORWARD_SWI))
	{
		SwiInput.b1Forward = 1;
	}
	
	if(1 == i32LocalDiGet(BACKWARD_SWI))
	{
		SwiInput.b1Backward = 1;
	}
	
	if((1 == i32LocalDiGet(TURTLE_SWI))||(1 == i32LocalDiGet(MODE_SWI)))
	{
		SwiInput.b1SlowMode = 1;
	}
	
	if(1 == i32LocalDiGet(LIFTUP_SWI))
	{
		SwiInput.b1LiftUp = 1;
	}
	
	if(1 == i32LocalDiGet(LIFTDOWN_SWI))
	{
		SwiInput.b1LiftDown = 1;
	}
	
	if(1 == i32LocalDiGet(BRAKE_SWI))
	{
		SwiInput.b1Brake = 1;
	}
	
	if(1 == i32LocalDiGet(PUMPEN_SWI))
	{
		SwiInput.b1PropEn = 1;
	}
#ifdef JINJIDAO_DGC_IO_TYPE
	if(1 == i32LocalDiGet(QIANHOUQIN_SWI))
	{
		SwiInput.b1QianHouQin = 1;
	}
	if(1 == i32LocalDiGet(QIANHOUYI_SWI))
	{
		SwiInput.b1QianHouYi = 1;
	}
#endif	
	
	if(SwiInputMode == 0)
	{
		sgSwiInput.u16data = SwiInput.u16data;
	}
	else
	{
		sgUserInfo.u16ThrottlePedalType = 0;
		sgSwiInput.b1Backward = sgHandleInput.b1Backward;
		sgSwiInput.b1Forward = sgHandleInput.b1Forward;
		sgSwiInput.b1Brake = sgHandleInput.b1Brake;
		sgSwiInput.b1Ems = sgHandleInput.b1Ems;
		sgSwiInput.b1LiftDown = sgHandleInput.b1LiftDown;
		sgSwiInput.b1LiftUp = sgHandleInput.b1LiftUp;
		sgSwiInput.b1SafeLock = sgHandleInput.b1SafeLock;
		sgSwiInput.b1SlowMode = sgHandleInput.b1SlowMode;
	}
	
	sgValvesInfo.b1Gear1SpdFlag = 0;  
	sgValvesInfo.b1Gear2SpdFlag = 0;
	sgValvesInfo.b1Gear3SpdFlag = 0;
	sgValvesInfo.b1Gear4SpdFlag = 0;
	
	if(gCanSendPdoInfo.CanSend380Info.BatterSoc < 20)
	{
		sgValvesInfo.b1Gear1SpdFlag = 0;   // 低速3
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 0;
		sgValvesInfo.b1Gear4SpdFlag = 1;
	}
	
	gCanSendPdoInfo.CanSend380Info.Tuttlespeed = 0;
	if(1 == sgSwiInput.b1SlowMode)
	{
		gCanSendPdoInfo.CanSend380Info.Tuttlespeed = 1;
		sgValvesInfo.b1Gear1SpdFlag = 0;   // 低速3
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 1;
		sgValvesInfo.b1Gear4SpdFlag = 0;
	}
	
	
	
	{
		tCanFrame Can390Send = {.u32ID = 0x390, .u16DataLength = 8, .u8Data = {0x00}, .u8Rtr = 0};
		memcpy((char*)Can390Send.u8Data, (char*)gCanSendPdoInfo.CanSend390Info.u8Data, sizeof(gCanSendPdoInfo.CanSend390Info));
		if(CanId390SendCout < 20)
		{
			CanId390SendCout ++;
		}
		else
		{
			CanId390SendCout = 0;
			i32CanWrite(Can0, &Can390Send);
		}
	}

}

static uint8_t u8SwiInitChcek(void)
{
	uint8_t u8Res = 0;
		
	if(((0 == i32LocalDiGet(BELLYEms_SWI))&&(0 == SwiInputMode))
		||(1 == i32LocalDiGet(FORWARD_SWI))
		||(1 == i32LocalDiGet(BACKWARD_SWI))	
		||(1 == i32LocalDiGet(SAFELOCK_SWI))
		||(1 == i32LocalDiGet(LIFTUP_SWI))
		||(1 == i32LocalDiGet(LIFTDOWN_SWI))
#ifdef	JINJIDAO_DGC_IO_TYPE
		||(1 == i32LocalDiGet(QIANHOUQIN_SWI))
		||(1 == i32LocalDiGet(QIANHOUYI_SWI))
#endif
			) 
	{
		u8Res = 1;
	}
	return u8Res;		
}

/*******************************************************************************
* Name: void vUserEcuInit(void)
* Descriptio: UserEcu初始化函数
* Input: NULL
* Output: NULL  
*******************************************************************************/
void vUserEcuInit(void)
{	
	uint16_t u16Tmp = 0;
	uint16_t WorkCountL = 0;
	uint16_t WorkCountH = 0;
	vMstRevRegister(vMstRevProc);
	vMstSendRegister(vMstSendProc);
	xRevCallBackProc CanId323 = {.u32CanId = 0x323, .u32Data = 0, .CallBack = vCanId323Proc};
	
	vCanRevMsgRegister(&CanId323);
	vDoPwmErrReg(vDoPwmErrCallBack);
	vPropErrReg(vPropErrCallBack);
	vAiErrReg(vAiErrCallBack);

	memset(&sgValvesInfo, 0, sizeof(sgValvesInfo));
	
	sgUserInfo.fPropMinCurrent = i32GetPara(PROP_MIN_CURRENT) / 1000.0;		/*比例阀最小电流*/
	sgUserInfo.fPropMaxCurrent = i32GetPara(PROP_MAX_CURRENT) / 1000.0;		/*比例阀最大电流*/
	
	sgUserInfo.fPropMinCurrent1 = i32GetPara(PROP_MIN_CURRENT1) / 1000.0;		/*比例阀最小电流*/
	sgUserInfo.fPropMaxCurrent1 = i32GetPara(PROP_MAX_CURRENT1) / 1000.0;		/*比例阀最大电流*/
	
	sgUserInfo.u8LeanForWardValue = i32GetPara(LEAN_FORWARD_PARA) * PUMP_RANGE / 100;		/*前倾参数*/
	sgUserInfo.u8LeanBackWardValue = i32GetPara(LEAN_BACKWARD_PARA) * PUMP_RANGE / 100;		/*后倾参数*/
	
	sgUserInfo.u16Gear1Spd = i32GetPara(PARA_Gear1Spd);	/*1档速度*/
	sgUserInfo.u16Gear2Spd = i32GetPara(PARA_Gear2Spd);	/*2档速度*/
	sgUserInfo.u16Gear3Spd = i32GetPara(PARA_Gear3Spd);	/*3档速度*/
	sgUserInfo.u16Gear4Spd = i32GetPara(PARA_Gear4Spd);	/*4档速度*/
	
	sgUserInfo.u16ThrottlePedalType = i32GetPara(PARA_ThrottleType);
	sgUserInfo.u16ThrottleMin = i32GetPara(MOVE_THROTTLE_MIN) * 100;		/*油门死区值, uint 0.1V*/
	sgUserInfo.u16ThrottleMax = i32GetPara(MOVE_THROTTLE_MAX) * 100;		/*油门死区值最大 ,uint 0.1V*/
	sgUserInfo.u16ThrottleRange = (sgUserInfo.u16ThrottleMax - sgUserInfo.u16ThrottleMin) >> 1;
	sgUserInfo.u16ThrottleMid = i32GetPara(MOVE_THROTTLE_MID) * MOTOR_SPEED_RANGE / 100;
	sgUserInfo.u16ThrottleMidValue = (sgUserInfo.u16ThrottleMax + sgUserInfo.u16ThrottleMin) >> 1;
	
	sgUserInfo.u16BrakePedalType = i32GetPara(BRAKE_THROTTLE_TYPE);
	sgUserInfo.u16LiftUpMin = i32GetPara(BRAKE_THROTTLE_MIN) * 100;		/*油门死区值, uint 0.1V*/
	sgUserInfo.u16LiftUpMax = i32GetPara(BRAKE_THROTTLE_MAX) * 100;		/*油门死区值最大 ,uint 0.1V*/
	sgUserInfo.u16LiftUpMid = (sgUserInfo.u16ThrottleMax + sgUserInfo.u16ThrottleMin)/2;
	sgUserInfo.u16BrakePedalRangeL = _IQ8(i32GetPara(BRAKE_THROTTLE_MID))/(sgUserInfo.u16ThrottleMid - sgUserInfo.u16ThrottleMin);
	sgUserInfo.u16BrakePedalRangeH = _IQ8(100 - i32GetPara(BRAKE_THROTTLE_MID))/(sgUserInfo.u16ThrottleMax - sgUserInfo.u16ThrottleMid);
	
	sgUserInfo.u16LiftDownMin = i32GetPara(LIFT_DOWN_THROTTLE_MIN) * 100;
	sgUserInfo.u16LiftDownMax = i32GetPara(LIFT_DOWN_THROTTLE_MAX) * 100;
	sgUserInfo.u16LiftDownRange	= (sgUserInfo.u16LiftDownMin - sgUserInfo.u16LiftDownMax) >> 1;
	sgUserInfo.u16LiftDownMid = i32GetPara(LIFT_DOWN_THROTTLE_MID) * PUMP_RANGE / 100;
	sgUserInfo.u16LiftDownMidValue	= (sgUserInfo.u16LiftDownMax + sgUserInfo.u16LiftDownMin) >> 1;
	
	sgUserInfo.SteerAngleDecSpd.u16StartAngle = i32GetPara(TURN_START_ANGLE) * 10;
	sgUserInfo.SteerAngleDecSpd.u16EndAngle = i32GetPara(TURN_END_ANGLE) * 10;
	sgUserInfo.SteerAngleDecSpd.u16StartAngleSpd = i32GetPara(START_ANGLE_SPD) * MOTOR_SPEED_RANGE / 100;
	sgUserInfo.SteerAngleDecSpd.u16EndAngleSpd = i32GetPara(END_ANGLE_SPD) * MOTOR_SPEED_RANGE / 100;
	sgUserInfo.SteerAngleDecSpd.fAgnleDecSpdFactor = (sgUserInfo.SteerAngleDecSpd.u16StartAngleSpd - sgUserInfo.SteerAngleDecSpd.u16EndAngleSpd) /   \
													 (sgUserInfo.SteerAngleDecSpd.u16EndAngle - sgUserInfo.SteerAngleDecSpd.u16StartAngle);
													 
	sgUserInfo.u16RatioOfTransmission = i32GetPara(PARA_RatioOfTransmission);
//	sgUserInfo.u16MotorMaxSpd = i32GetPara(PARA_MotorMaxSpd);
		sgUserInfo.HourCountMode = i32GetPara(PARA_AngleValue7);
		sgUserInfo.u16BrakePedalType = i32GetPara(BRAKE_THROTTLE_TYPE);
	
	sgUserInfo.u8BatteryType = i32GetPara(PARA_BatteryType);
	steerAngleTab = i32GetPara(PARA_PumpMotorGear1);
	
	sgUserInfo.u8SOCLimit = i32GetPara(PARA_AccAndDecTurn);    //电量故障模拟
	sgUserInfo.Temputer = i32GetPara(PARA_AccAndDecAntiPinch);   //温度故障模拟
	sgUserInfo.u8Language = i32GetPara(PARA_LanguageType);
	
	sgUserInfo.u16QianHouQinSpeed = i32GetPara(PARA_PumpMotorGear1);
	sgUserInfo.u16QianHouYiSpeed = i32GetPara(PARA_PumpMotorGear2);
	
	MotorOdm = i32GetPara(PARA_AngleValue6);
	gCanSendPdoInfo.CanSend380Info.Odometer1 = (MotorOdm/1000) & 0xFF;
	gCanSendPdoInfo.CanSend380Info.Odometer2 = ((MotorOdm/1000)>>8) & 0xFF;
	gCanSendPdoInfo.CanSend380Info.Odometer3 = ((MotorOdm/1000)>>16) & 0xFF;
	/*lilu 20230823 add user mode*/
	{
		u16Tmp = i32GetPara(USERINFO_MODE);
		sgUserInfo.b1HourConutMode = u16Tmp & 0x01;					/*bit0: HourCount Mode*/
		sgUserInfo.b1StartUpLock = (u16Tmp >> 1) & 0x01;			/*bit1: StartUpCheck*/
		sgUserInfo.b1LiftLock = (u16Tmp >> 2) & 0x01;				/*bit2: Lift by Lock */
		sgUserInfo.b1LiftPedal = (u16Tmp >> 3) & 0x01;				/*bit3: Lift by Peadl*/
		sgUserInfo.b1MoveLiftMode = (u16Tmp >> 4) & 0x01;			/*bit4: Move and Lift */
		
	}
	
	u32HourCount = u32HourCountRead();
	if (0x5555 != u16Tmp)
	{
		u16EepromWrite(PARA_HourSetTime, 0x0000, 1);
		u16EepromWrite(PARA_DefaultFlag, 0x5555, 1);	
	}
	else
	{
		u16EepromRead(PARA_HourSetTime, &u16Tmp, 1);	
	}
//	//i32LogWrite(INFO, "******SaveState = 0x%x\r\n*********", sgSaveState.u16Data);
//	__disable_irq();
	gCanSendPdoInfo.CanSend380Info.Houemeter = (u32HourCount/10) & 0xFFFF;
//	__enable_irq();		
	/*Para Initial*/
	
	sgUserInfo.u16RentalTime = i32GetPara(RENTAL_TIME);
	if (0 != i32GetPara(PARA_AccAndDecFastDrive))
	{
		sgUserInfo.fMoveSpdPer5msAccStep = STEP_FACTOR * i32GetPara(PARA_CurveFastDrive) / i32GetPara(PARA_AccAndDecFastDrive);
		sgUserInfo.fMoveSpdPer5msDecStep = STEP_FACTOR * i32GetPara(PARA_BrakeFastDrive) / i32GetPara(PARA_AccAndDecFastDrive);
	}
	else
	{
		//i32LogWrite(ERR, "Fast Spd Period is 0!!!\r\n");
		sgUserInfo.fMoveSpdPer5msAccStep = 0;
		sgUserInfo.fMoveSpdPer5msDecStep = 0;
	}

	if (0 != i32GetPara(PARA_AccAndDecLift))
	{
		sgUserInfo.fLiftSpdPer5msAccStep = STEP_FACTOR * i32GetPara(PARA_CurveLift) / i32GetPara(PARA_AccAndDecLift);
		sgUserInfo.fLiftSpdPer5msDecStep = STEP_FACTOR * i32GetPara(PARA_BrakeLift) / i32GetPara(PARA_AccAndDecLift);
	}
	else
	{
		//i32LogWrite(ERR, "Lift Spd Period is 0!!!\r\n");
		sgUserInfo.fLiftSpdPer5msAccStep = 0;
		sgUserInfo.fLiftSpdPer5msDecStep = 0;
	}
	if (0 != i32GetPara(PARA_AccAndDecLower))
	{
		sgUserInfo.fDownSpdPer5msAccStep = STEP_FACTOR * i32GetPara(PARA_CurveLower) / i32GetPara(PARA_AccAndDecLower);
		sgUserInfo.fDownSpdPer5msDecStep = STEP_FACTOR * i32GetPara(PARA_BrakeLower) / i32GetPara(PARA_AccAndDecLower);
	}
	else
	{
		//i32LogWrite(ERR, "Down Spd Period is 0!!!\r\n");
		sgUserInfo.fDownSpdPer5msAccStep = 0;
		sgUserInfo.fDownSpdPer5msDecStep = 0;
	}
	
	vCanIdLostReg(0x323,1000,vCanLostProc);
	vCanIdLostReg(0x2F4,1000,vCanLostProc);

	vSetPdoPara(sgPdoPara);
	
	i32LocalDoSet(DO_ENCODER2, 1);
	
	vSetNetTimer(TIMER_EcuPowerOn, ECU_POWERON_DELAY_TIME);	/**/
	
	vSetNetTimer(TIMER_HourCount, HOURCOUNT_PERIOD);
	
}

/*******************************************************************************
* Name: void vUserEcuProc(void)
* Descriptio: UserEcu用户处理函数
* Input: NULL
* Output: NULL  
*******************************************************************************/
void vUserEcuProc(void)
{
	static uint8_t u8EcuProcFlag = 0;
	uint8_t u8ErrCode = 0;
	static uint16_t u16SecCnt = 0;
	static uint16_t u16RentalCnt = 0;
	static uint32_t u8MainConnectCnt = 0;
	static uint16_t u16RentalInfo = 0;
	int32_t i32PropValue1 = 0;

	if(true == u8GetNetTimerOverFlag(TIMER_EcuPowerOn))
	{
		u8EcuProcFlag = 1;
		vKillNetTimer(TIMER_EcuPowerOn);
	}
	
	
	if ((false == u8GetNetTimerOverFlag(TIMER_EcuPowerOn)) && (true == u8GetNetTimerStartFlag(TIMER_EcuPowerOn)))
	{
		SwiInputMode = i32LocalDiGet(BRAKE_SWI);
		if (1 == u8SwiInitChcek())
		{
			sgValvesInfo.u8NoAct |= POWERON_CHECK_LIMIT;
			i32ErrCodeSet(POWERON_CHECK_ERR);
		}
	}
	
	if (0 == u8SwiInitChcek())
	{
		sgValvesInfo.u8NoAct &= ~POWERON_CHECK_LIMIT;
		i32ErrCodeClr(POWERON_CHECK_ERR);
	}
	
	if(1 == u8EcuProcFlag)
	{
		vSwiMonitor();				
		vAiMonitor();
		vCanRevPdoProc();
		
		if(0 != SwiInputMode)
		{
			CanId323LostChk++;
			if(CanId323LostChk > 100)
			{
				sgValvesInfo.u8NoAct |= BmsLost_Limit;
			}
			else
			{
				sgValvesInfo.u8NoAct &= ~BmsLost_Limit;
			}
		}
		
		if(0 != (sgValvesInfo.u8NoAct & (HandleLost_Limit|BmsLost_Limit)))
		{
			i32ErrCodeSet(109);
		}
		else
		{
			i32ErrCodeClr(109);
		}
		u8ErrCode = u8ErrCodeGet();
		if (0 != u8ErrCode)
		{
			i32PropValue1 = _IQ((sgUserInfo.fPropMinCurrent1 + (sgUserInfo.fPropMaxCurrent1 - sgUserInfo.fPropMinCurrent1) * 255 / PUMP_RANGE) / PROPD_STD_CURRENT);
			vPropSetTarget(WARING_VALVE, i32PropValue1);
			vLedSendAlmCode(u8ErrCode);
			gCanSendPdoInfo.CanSend390Info.ErrCodeMove = u8ErrCode;
		}
		else
		{
		gCanSendPdoInfo.CanSend390Info.ErrCodeMove = 0;
			vPropSetTarget(WARING_VALVE, 0);
		}
	}

	if ((true == u8GetNetTimerOverFlag(TIMER_HourCount))&&((1 == sgUserInfo.HourCountMode)||(3 == sgUserInfo.HourCountMode)))
	{
		vResetNetTimer(TIMER_HourCount);
		u16SecCnt++;
		if (u16SecCnt >= 360)			/*360*/ 
		{
			u16SecCnt = 0;
			u32HourCount++;
			vHourCountWrite(u32HourCount);
			if(0 == (u32HourCount % 10))
			{
				gCanSendPdoInfo.CanSend380Info.Houemeter = (u32HourCount/10) & 0xFFFF;
			}
		}
	}
	if(0 != sgUserInfo.b1HourConutMode)
	{
			u32HourCount = 0;
			vHourCountWrite(u32HourCount);
			gCanSendPdoInfo.CanSend380Info.Houemeter = (u32HourCount/10) & 0xFFFF;
	}
	if(0 != sgUserInfo.b1HourConutMode)
	{
		sgUserInfo.b1HourConutMode = 0;
	}
	
	vWdgSetFun(WDG_USER_BIT);
}
#endif
