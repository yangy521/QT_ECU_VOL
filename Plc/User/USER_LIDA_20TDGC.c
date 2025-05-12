/*******************************************************************************
* Filename: UserStackerTruckProc.c	                                           *
* Description: 逻辑层的源文件，包含对应的回调函数	   						   *
* Author:                                                      *
* Date: 2023/05/30    														   *
* Revision:	 														 		   *
*******************************************************************************/
#include "UserCanComm.h"
#include "USER_LIDA_20TDGC.h"
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

#if (USER_TYPE == USER_LIDA_20TDGC)

const static uint8_t u8ErrSwitchArray[ErrCodeMax] = 
{	
/*  0    1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18   19*/
    0,   1,   2,   3,   4,   5,   6,   7,   8,   9,   10,  11,  12,  13,  14,  15,  16,  17,  18,  19,
    20,  21,  22,  23,  24,  25,  26,  27,  28,  29,  30,  31,  32,  33,  34,  35,  36,  37,  38,  39,
    40,  41,  42,  43,  44,  45,  46,  47,  48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,
		60,  61,  62,  63,  64,  65,  66,  67,  68,  69,  70,  71,  72,  73,  74,  75,  76,  77,  78,  79, 
		80,  81,  82,  83,  84,  85,  86,  87,  88,  89,  90,  91,  92,  93,  94,  95,  96,  97,  98,  99,
		100, 40, 	42,  41, 40, 105, 106, 	40, 40, 40, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 
		120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 
		140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159,
		160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 
		180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 
		200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 
		220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 
		240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254,
};

//转弯降速曲线表
const uint8_t SteerAngle2Speedrate[6][90]=
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
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 100, .u16CanId = 0x260},
	},
	.RpdoPara = {
		{.b1Flag = 1, .b11CanRevId = 0x360},
		{.b1Flag = 1, .b11CanRevId = 0x444},
		{.b1Flag = 1, .b11CanRevId = 0x244},
		{.b1Flag = 0, .b11CanRevId = 0x000},
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

#define	NoAct_ChargeSwi		0
#define	NoAct_Init			1
#define	NoAct_Lock			2
#define	NoAct_Security		3	

#define NoLiftUp_HeightLimit	0		
#define NoLiftUp_FaultLock		1
#define	NoLiftUp_Bms			2

#define	NoMove_Pedal			0
#define	NoMove_FaultLock		1
#define	NoMove_Ems				2

#define Move_Limit					(0x01 << 0)
#define LiftUp_Limit				(0x01 << 1)
#define LiftDown_Limit			(0x01 << 2)
#define ZuoYouYi_Limit			(0x01 << 3)
#define QianHouYi_Limit			(0x01 << 4)
#define QianHouQin_Limit		(0x01 << 5)
#define Lift_Limit					(0x01 << 6)
#define LiftMoveMode_Limit	(0x01 << 7)

#define LowPowerLift_Limit  (0x01 << 8)
#define BmsLost_Limit				(0x01 << 9)

#define EmsErr_Limit				(0x01 << 10)

#define ERROR_BMSERRORLEVELFLAG		(0x01 << 11)
#define ALARM_BMSERRORLEVELFLAG		(0x01 << 12)

#define EMSFord_LIMIT						(0x01 << 13)

#define MOVESAFETY_Limit				(0x01 << 14)
#define LIFTSAFETY_Limit				(0x01 << 15)


typedef union
{
	uint16_t u16data;
	struct
	{
		uint16_t b1YouYi: 1;         //高度限速
		uint16_t b1Ems: 1;                 //急反开关
		uint16_t b1SafeLock: 1;               //互锁
		uint16_t b1Forward: 1;		      	 //向货叉
		uint16_t b1Backward: 1;		
		uint16_t b1LiftUp: 1; 
		uint16_t b1LiftDown: 1;
		uint16_t b1QianYi: 1;		
 		uint16_t b1HouYi: 1;                           
		uint16_t b1QianQin: 1;
		uint16_t b1HouQin: 1;
		uint16_t b1ZuoYi: 1;
		uint16_t b1LiftUpLimit: 1;
		uint16_t b1HighSpeedLimit: 1;
		uint16_t b7Reserve: 2;
	};
}xSwiInput;

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
	
	float		fPropMinCurrent0;
	float		fPropMaxCurrent0;
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
	uint8_t   b1HourCntClr: 1;
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

typedef union
{
	uint8_t u8data;
	struct
	{
		uint8_t Height_SpeedLimit: 1;
		uint8_t ZhiLi_MOVE: 1;
		uint8_t Reserve: 6;
	};
}xFlagVal;

typedef union
{
	uint16_t u16Data;
	struct
	{
		uint16_t b1Above1M8: 1;
		uint16_t b1HeightSpdLimit: 1;
		uint16_t b1SpdMode: 1;
		uint16_t b1PasswordFunc: 1;
		uint16_t b1Rental: 1;
		uint16_t b11Reserve: 11;
	};
}xSaveStateInfo;

xSaveStateInfo sgSaveState;
const static uint8_t u8SeedArray[16]= "V!BWT%EM6dJ8<nPs";

static int16_t	i16MotorSpd = 0;
static uint16_t u16MotorVal = 0;
static uint8_t	u8PumpOrPropValue = 0;
static xValvesInfo sgValvesInfo;
static xUserInfo sgUserInfo;
static xSwiInput sgSwiInput;
static xSwiInput sgSwiInputLast;
static uint32_t u32HourCount = 0;
static int16_t i16SteerAngle = 0;
static uint16_t sgu16LimitFlg = 0;

static uint16_t steerAngleTab = 0;
static uint8_t SteetSpeedLimit = 0;

static uint16_t PropContineTime1 = 0;
static uint16_t PropContineTime2 = 0;
static uint16_t PropContineTime3 = 0;
static uint8_t HeighSpeedLimit = 0;

static uint8_t check_err = 0;

static xFlagVal sgFlagVal;

/*PDO接收待完善*/
xCanRevPdoInfo gCanRevPdoInfo; 
static xCanRev360Info CanRev360InfoLast;
static xCanRev444Info CanRev444InfoLast;
static xCanRev244Info CanRev244InfoLast;
/*PDO发送待完善*/
xCanSendPdoInfo gCanSendPdoInfo;

static void vMstRevProc(xMstRevPara *RevData)
{	
	uint16_t tmp = 0;
	static uint8_t tmpLast = 0;
	
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
			i32ErrCodeSet(RevData->u8ErrCode - 1);
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
	gCanSendPdoInfo.CanSend260Info.u8Movespeed = tmp;
	__enable_irq();
	
	tmp = RevData->u8MotorTmp;
	/*Motor Temp*/
	
	tmp = RevData->u8BoardTmp;
	__disable_irq();
//	gCanSendPdoInfo.CanSend43CInfo.i16MotorCurrent = (RevData->u8CurrentHigh << 8) | RevData->u8CurrentLow;
	__enable_irq();
}

/*发送给Mst的回调函数*/
static void vMstSendProc(xMstSendPara *SendData) 
{
	static xValvesInfo sgLastValvesInfo;
	static uint16_t PropCloseDelay = 0;
	int32_t i32PropValue = 0;
	xMstSendStat LastStatus;

		
	LastStatus.u8Data = SendData->buf[2];
	SendData->buf[2] = 0;

	if((0 == check_err)&&(0 == (sgu16LimitFlg & LIFTSAFETY_Limit)))
	{
		/*Move Mode*/
		SendData->b1ServoOn = 1;
			if(0 == (sgu16LimitFlg & (LiftMoveMode_Limit|BmsLost_Limit|EmsErr_Limit|EMSFord_LIMIT|MOVESAFETY_Limit)))
			{
				if(sgSwiInput.b1Backward == 1)
				{
					SendData->b1ForwardReq = 1;
				}
				else if(sgSwiInput.b1Forward == 1)
				{
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

				if ((1 == sgSwiInput.b1Ems) && ((1 == SendData->b1ForwardReq) || (1 == LastStatus.b1EmsReq)))  
				{
					SendData->b1EmsReq = 1;
					if(1 == SendData->b1BackwardReq)
					{
						i32ErrCodeSet(100);
					}
				}
			}
			
		/*Lift Mode*/
		
			if(PropCloseDelay > 0)
			{
				PropCloseDelay --;
			}
			else
			{
				i32DoPwmSet(LIFTUP_VALVE,DRIVER_CLOSE);
				i32DoPwmSet(QIANHOUYI_VALVE,DRIVER_CLOSE);
				i32DoPwmSet(QIANHOUQIN_VALVE,DRIVER_CLOSE);
				i32DoPwmSet(ZUOYOUYI_VALVE,DRIVER_CLOSE);
				i32DoPwmSet(KAIGUAN_VALVE,DRIVER_CLOSE);
				vPropSetTarget(DIRECTION_VALVE, 0);
			}
		vPropSetTarget(LIFTDOWN_VALVE, 0);
			
		SendData->u8PumpTarget = 0;
		SendData->b1LiftReq = 0;
		
		if((1 == sgSwiInput.b1LiftDown)
			&&(0 == (sgu16LimitFlg & Lift_Limit))
			)
		{
			PropCloseDelay = (i32GetPara(PARA_AngleValue0)*100/5);
			i32DoPwmSet(LIFTUP_VALVE,DRIVER_OPEN);
			i32PropValue = _IQ((sgUserInfo.fPropMinCurrent1 + (sgUserInfo.fPropMaxCurrent1 - sgUserInfo.fPropMinCurrent1) * u8PumpOrPropValue / PUMP_RANGE) / PROPD_STD_CURRENT);
			vPropSetTarget(LIFTDOWN_VALVE, i32PropValue);
		}
		else if((1 == sgSwiInput.b1LiftUp)
					&&(0 == (sgu16LimitFlg & (Lift_Limit|LowPowerLift_Limit|ERROR_BMSERRORLEVELFLAG)))
					)
		{
			PropCloseDelay = (i32GetPara(PARA_AngleValue0)*100/5);
			vPropSetTarget(LIFTDOWN_VALVE, 0);
			i32DoPwmSet(LIFTUP_VALVE,DRIVER_OPEN);
			SendData->u8PumpTarget = u8PumpOrPropValue;
			SendData->b1LiftReq = 1;
		}

		if((1 == sgSwiInput.b1ZuoYi)
			&&(0 == (sgu16LimitFlg & ZuoYouYi_Limit))
			)
		{
			if(PropContineTime1 > 0)
			{
				PropContineTime1--;
				PropCloseDelay = (i32GetPara(PARA_AngleValue0)*100/5);
				i32DoPwmSet(ZUOYOUYI_VALVE,DRIVER_OPEN);
				i32DoPwmSet(KAIGUAN_VALVE,DRIVER_OPEN);
				SendData->u8PumpTarget = 255*sgUserInfo.u16Gear2Spd/100;
				SendData->b1LiftReq = 1;
			}
		}
		else if((1 == sgSwiInput.b1YouYi)
					&&(0 == (sgu16LimitFlg & ZuoYouYi_Limit))
					)
		{
			if(PropContineTime1 > 0)
			{
				PropContineTime1--;
				PropCloseDelay = (i32GetPara(PARA_AngleValue0)*100/5);
				i32DoPwmSet(ZUOYOUYI_VALVE,DRIVER_OPEN);
				i32DoPwmSet(KAIGUAN_VALVE,DRIVER_OPEN);
				i32PropValue = _IQ((sgUserInfo.fPropMinCurrent0 + (sgUserInfo.fPropMaxCurrent0 - sgUserInfo.fPropMinCurrent0) * 255 / PUMP_RANGE) / PROPD_STD_CURRENT);
				vPropSetTarget(DIRECTION_VALVE, i32PropValue);
				SendData->u8PumpTarget = 255*sgUserInfo.u16Gear2Spd/100;
				SendData->b1LiftReq = 1;
			}
		}
		else
		{
			PropContineTime1 = (i32GetPara(PARA_AngleValue1)*100/5);
		}
		
		if((1 == sgSwiInput.b1QianQin)
			&&(0 == (sgu16LimitFlg & QianHouQin_Limit)))
		{
			if(PropContineTime2 > 0)
			{
				PropContineTime2--;
				PropCloseDelay = (i32GetPara(PARA_AngleValue0)*100/5);
				i32DoPwmSet(QIANHOUQIN_VALVE,DRIVER_OPEN);
				i32DoPwmSet(KAIGUAN_VALVE,DRIVER_OPEN);
				SendData->u8PumpTarget = 255*sgUserInfo.u16Gear3Spd/100;
				SendData->b1LiftReq = 1;
			}
		}
		else if((1 == sgSwiInput.b1HouQin)
			&&(0 == (sgu16LimitFlg & QianHouQin_Limit)))
		{
			if(PropContineTime2 > 0)
			{
				PropContineTime2--;
				PropCloseDelay = (i32GetPara(PARA_AngleValue0)*100/5);
				i32DoPwmSet(QIANHOUQIN_VALVE,DRIVER_OPEN);
				i32DoPwmSet(KAIGUAN_VALVE,DRIVER_OPEN);
				i32PropValue = _IQ((sgUserInfo.fPropMinCurrent0 + (sgUserInfo.fPropMaxCurrent0 - sgUserInfo.fPropMinCurrent0) * 255 / PUMP_RANGE) / PROPD_STD_CURRENT);
				vPropSetTarget(DIRECTION_VALVE, i32PropValue);
				SendData->u8PumpTarget = 255*sgUserInfo.u16Gear3Spd/100;
				SendData->b1LiftReq = 1;
			}
		}
		else
		{
			PropContineTime2 = (i32GetPara(PARA_AngleValue2)*100/5);
		}
		
		if((1 == sgSwiInput.b1QianYi)
			&&(0 == (sgu16LimitFlg & QianHouYi_Limit)))
		{
			if(PropContineTime3 > 0)
			{
				PropContineTime3--;
				PropCloseDelay = (i32GetPara(PARA_AngleValue0)*100/5);
				i32DoPwmSet(QIANHOUYI_VALVE,DRIVER_OPEN);
				SendData->u8PumpTarget = 255*sgUserInfo.u16Gear4Spd/100;
				SendData->b1LiftReq = 1;
			}
		}
		else if((1 == sgSwiInput.b1HouYi)
			&&(0 == (sgu16LimitFlg & QianHouYi_Limit)))
		{
			if(PropContineTime3 > 0)
			{
				PropContineTime3--;
				PropCloseDelay = (i32GetPara(PARA_AngleValue0)*100/5);
				i32DoPwmSet(QIANHOUYI_VALVE,DRIVER_OPEN);
				i32PropValue = _IQ((sgUserInfo.fPropMinCurrent0 + (sgUserInfo.fPropMaxCurrent0 - sgUserInfo.fPropMinCurrent0) * 255 / PUMP_RANGE) / PROPD_STD_CURRENT);
				vPropSetTarget(DIRECTION_VALVE, i32PropValue);
				SendData->u8PumpTarget = 255*sgUserInfo.u16Gear4Spd/100;
				SendData->b1LiftReq = 1;
			}	
		}
		else
		{
			PropContineTime3 = (i32GetPara(PARA_AngleValue3)*100/5);
		}	
	}
	
	if(0 != sgUserInfo.b1LiftLock)
	{
		if(0 == sgSwiInput.b1SafeLock)
		{
			SendData->u8PumpTarget = 0;
			SendData->b1LiftReq = 0;
			vPropSetTarget(LIFTDOWN_VALVE, 0);
		}
	}
	
	if ((sgu16LimitFlg & ERROR_BMSERRORLEVELFLAG) != 0)
	{
		SendData->b1ForwardReq = 0;
		SendData->b1BackwardReq = 0;	
		SendData->u8TargetHigh = 0;
		SendData->u8TargetLow = 0;
	}
	else if((sgu16LimitFlg & ALARM_BMSERRORLEVELFLAG) != 0)
	{
		SendData->b1LiftReq = 0;
		SendData->u8PumpTarget = 0;
		if(u16MotorVal > 2048)
		{
			u16MotorVal = 2048;
			SendData->u8TargetHigh = u16MotorVal >> 8;
			SendData->u8TargetLow = u16MotorVal;
		}
	}
		
	if(0 == sgSwiInput.b1SafeLock)
	{
		SendData->b1ServoOn = 0;
	}
	
		{			
			/*lilu 20230819 For Test*/
			{
				i32SetPara(PARA_ForwardValveCurrent, u16MotorVal);		/*Send Motor Value*/
				i32SetPara(PARA_BackValveCurrent, u8PumpOrPropValue);		/*Rev Motor Value*/
				i32SetPara(PARA_PropValveCurrent, i32PropValue);	/*Prop Current*/
				i32SetPara(PARA_EcuLockState, sgu16LimitFlg);	//上电计时	
//				i32SetPara(PARA_TmpLockState, Work2App);	
//				i32SetPara(PARA_SelfHeartQuery, HourCntFlg);	
//				i32SetPara(PARA_PlatfromHeartQuery, WorkCntFlg);
//				i32SetPara(PARA_LiftValveCurrent, u8PumpOrPropValueMove);		/*Send Pump Value*/
//				i32SetPara(PARA_OnOffValveCurrent, SendData->buf[2]);			/*Send Status*/
				i32SetPara(PARA_LoadRate, sgFlagVal.u8data);				/*Valve NoFlag*/
				i32SetPara(PARA_CalibrationStatus, sgValvesInfo.u8Data[1]);		/*Value State*/
				i32SetPara(PARA_TurnRightValveCurrent, ((uint16_t)sgValvesInfo.u8NoAct << 8) | ((uint16_t)sgValvesInfo.b4NoMove << 4) | ((uint16_t)sgValvesInfo.b4NoLiftUp));
				i32SetPara(PARA_TurnLeftValveCurrent, ((uint16_t)sgValvesInfo.b4Gear1Spd << 12) | ((uint16_t)sgValvesInfo.b4Gear2Spd << 8) | \
					((uint16_t)sgValvesInfo.b4Gear3Spd << 4) | ((uint16_t)sgValvesInfo.b4Gear4Spd));
				i32SetPara(PARA_AngleValue, (uint16_t)i32LocalAiGetValue(AI_B_AI1_R));		/*AI1*/
				i32SetPara(PARA_PressureVlaue1, (uint16_t)i32LocalAiGetValue(AI_B_AI2_R));	/*AI2*/
				i32SetPara(PARA_PressureVlaue2, (uint16_t)i32LocalAiGetValue(AI_B_AI3_R));	/*AI3*/
				i32SetPara(PARA_ExtSignal, sgSwiInput.u16data);						/*Swi*/
//				i32SetPara(PARA_PcuKeyInfo,sgHandleInput.u8data);						/*ErrCode*/
//				i32SetPara(PARA_BmsSoc, gCanSendPdoInfo.CanSend23CInfo.u8Soc);		/*BMS SOC*/
				i32SetPara(PARA_BrakeValveCurrent, abs(i16SteerAngle));
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
		case 0x444:
			if(0 == sgUserInfo.u8BatteryType)
			{
				if(CAN_NORMAL == u8State)
				{
					sgu16LimitFlg &= ~BmsLost_Limit;
					i32ErrCodeClr(ID444LOST_ERR);
				}
				else if(CAN_LOST == u8State)
				{
					sgu16LimitFlg |= BmsLost_Limit;
					i32ErrCodeSet(ID444LOST_ERR);
				}
			}
			break;
			case 0x360:
				if(CAN_NORMAL == u8State)
				{
					sgu16LimitFlg &= ~BmsLost_Limit;
					i32ErrCodeClr(ID360LOST_ERR);
				}
				else if(CAN_LOST == u8State)
				{
					sgu16LimitFlg |= BmsLost_Limit;
					i32ErrCodeSet(ID360LOST_ERR);
				}
			break;
		default:
			break;
	
	}
}


const INT8U ERROR_CODE_BMS[100]=
{
// 00,  01,  02,  03,  04,  05,  06,  07,  08,  09,
    0,   1,   2,   3,   4,   51,   52,   53, 54,  55,	 	 /*0~9*/
    56,   57, 58,  59,  60,  61,  62,  63,  64,  65,  		/*10~19*/
    66,  0,   0,   0,   0,   0,   0,   0,   0,   0, 	    /*20~29*/
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,	    /*30~39*/
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,	  		/*40~49*/
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,		    /*50~59*/
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,		    /*60~69*/
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,		    /*70~79*/
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,		    /*80~89*/
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0		    /*90~99*/
};



/*接收PDO处理函数*/
static void vCanRevPdoProc(void)
{

	uint32_t AlmBms2McuJdcc;
/***** 转向360 *******/
	memcpy((char*)CanRev360InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo360.u8Data, sizeof(CanRev360InfoLast));
		/*添加相关操作*/
	{
			i16SteerAngle = ((CanRev360InfoLast.i16SteerAngleH<<8)|(CanRev360InfoLast.i16SteerAngleL));
			gCanSendPdoInfo.CanSend260Info.u8ErrorSteer = CanRev360InfoLast.u8ErrSteer;
	}
	
/***** 锂电 444 *******/
	memcpy((char*)CanRev444InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo444.u8Data, sizeof(CanRev444InfoLast));
	memcpy((char*)CanRev244InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo244.u8Data, sizeof(CanRev244InfoLast));
	/*添加相关操作*/
	if(0 == sgUserInfo.u8BatteryType)
	{
		gCanSendPdoInfo.CanSend260Info.u8Soc = CanRev444InfoLast.BMS_SOC;
		AlmBms2McuJdcc = (   ((CanRev244InfoLast.u8Data[0] >> 4) & (3 << 0))
												| ((CanRev244InfoLast.u8Data[0] >> 4) & (3 << 2))
												| (CanRev244InfoLast.u8Data[2] << 4) 
												| (CanRev244InfoLast.u8Data[3] << 12)
												 );
		{
			INT32U ErrorBms2Hmi;
			INT32U BmsError2Mcu;
			
			
			if ((BmsError2Mcu = AlmBms2McuJdcc) != 0)
			{
				ErrorBms2Hmi = 0;
				do{
					ErrorBms2Hmi++;
					if ((BmsError2Mcu & 0x1) != 0)
						break;
					BmsError2Mcu = BmsError2Mcu >> 1;
				}while (ErrorBms2Hmi < 32);
			}
			else
			{
				ErrorBms2Hmi = 0;
			}
			gCanSendPdoInfo.CanSend260Info.u8ErrorBMS = ERROR_CODE_BMS[ErrorBms2Hmi];
			
			sgu16LimitFlg &= ~(ERROR_BMSERRORLEVELFLAG | ALARM_BMSERRORLEVELFLAG);
			
			if ((ErrorBms2Hmi % 2 == 0) && (ErrorBms2Hmi != 0))
				sgu16LimitFlg |= ERROR_BMSERRORLEVELFLAG;
			else if (ErrorBms2Hmi % 2 == 1)
				sgu16LimitFlg |= ALARM_BMSERRORLEVELFLAG;
		}
	}
	else
	{
		gCanSendPdoInfo.CanSend260Info.u8Soc = u8GetBatterySoc();
	}
	
/***** 手柄 1E0 *******/

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
	i32AdcValue = i32LocalAiGetValue(MOVE_THROTTLE);
	if(i32AdcValue > sgUserInfo.u16ThrottleMax)
	{
		u16MotorVal = 4096;
	}
	else if((i32AdcValue >= sgUserInfo.u16ThrottleMin)&&(i32AdcValue <= sgUserInfo.u16ThrottleMax))
	{
		u16MotorVal = (((i32AdcValue - sgUserInfo.u16ThrottleMin)*4095)/(sgUserInfo.u16ThrottleMax-sgUserInfo.u16ThrottleMin));
	}
	else
	{
		u16MotorVal = 0;
	}
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
/*add Turn Dec Spd*/
	//根据角度值，查表得转弯降速比例
	{
		uint16_t steerAngle = 0;
		
		steerAngle = abs(i16SteerAngle)/10;
		if(steerAngle > (sizeof(SteerAngle2Speedrate[0])-1))
			steerAngle = sizeof(SteerAngle2Speedrate[0])-1;
		
		steerAngleTab = steerAngleTab>((sizeof(SteerAngle2Speedrate)/sizeof(SteerAngle2Speedrate[0]))-1)?(sizeof(SteerAngle2Speedrate)/sizeof(SteerAngle2Speedrate[0])):steerAngleTab;
		
		if(0 == steerAngleTab)
		{
			u16MotorVal = (uint16_t)((u16MotorVal * u8SpeedRate)/100);
			u16MotorVal = u16SteerAngleDecSpd(u16MotorVal, abs(i16SteerAngle), &sgUserInfo.SteerAngleDecSpd);
		}
		else
		{
			speedRateSteer = SteerAngle2Speedrate[steerAngleTab-1][steerAngle];
			if(speedRateSteer < u8SpeedRate)
				u8SpeedRate = speedRateSteer;
			u16MotorVal = (uint16_t)((u16MotorVal * u8SpeedRate)/100);
		}
	}
}


/*开关量监控*/
static void vSwiMonitor(void)
{
	xSwiInput SwiInput = {0};
	int32_t i32AdcValue = 0;
	uint8_t PowerLow = 0;
	uint8_t PowerLLow = 0;
	
	sgSwiInputLast.u16data = sgSwiInput.u16data;
	
	if(1 == i32LocalDiGet(YouYi_SWI))
	{
		SwiInput.b1YouYi = 1;
	}
	
	if(1 == i32LocalDiGet(EmergencyReverse))
	{
		SwiInput.b1Ems = 1;
	}
	
	if(1 == i32LocalDiGet(SAFELOCK_SWI))
	{
		SwiInput.b1SafeLock = 1;
	}
	
	if(1 == i32LocalDiGet(ForWard_SWI))
	{
		SwiInput.b1Forward = 1;
	}
	
	if(1 == i32LocalDiGet(BackWard_SWI))
	{
		SwiInput.b1Backward = 1;
	}
	
	if(0 == i32LocalDiGet(LiftUpLimit_SWI))
	{
		SwiInput.b1LiftUpLimit = 1;
	}
	
	if(1 == i32LocalDiGet(QianYi_SWI))
	{
		SwiInput.b1QianYi = 1;
	}
	
	if(1 == i32LocalDiGet(HouYi_SWI))
	{
		SwiInput.b1HouYi = 1;
	}
	
	if(1 == i32LocalDiGet(QianQin_SWI))
	{
		SwiInput.b1QianQin = 1;
	}
	
	if(1 == i32LocalDiGet(HouQin_SWI))
	{
		SwiInput.b1HouQin = 1;
	}
	
	if(1 == i32LocalDiGet(ZuoYi_SWI))
	{
		SwiInput.b1ZuoYi = 1;
	}
	
	/*********** Lift Speed *************/
	i32AdcValue = i32LocalAiGetValue(LIFT_THROTTLE);
	if(i32AdcValue > sgUserInfo.u16LiftUpMax)
	{
		SwiInput.b1LiftDown = 1;
		u8PumpOrPropValue = 255;
	}
	else if((i32AdcValue >= (sgUserInfo.u16LiftUpMid+sgUserInfo.u16LiftUpRange))&&(i32AdcValue <= sgUserInfo.u16LiftUpMax))
	{
		SwiInput.b1LiftDown = 1;
		u8PumpOrPropValue = (((i32AdcValue - (sgUserInfo.u16LiftUpMid+sgUserInfo.u16LiftUpRange))*255)/(sgUserInfo.u16LiftUpMax-(sgUserInfo.u16LiftUpMid+sgUserInfo.u16LiftUpRange)));
	}
	else if(((i32AdcValue <= (sgUserInfo.u16LiftUpMid - sgUserInfo.u16LiftUpRange))&&(i32AdcValue >= sgUserInfo.u16LiftUpMin))&&(0 == SwiInput.b1LiftUpLimit))
	{
		SwiInput.b1LiftUp = 1;
		u8PumpOrPropValue = (((sgUserInfo.u16LiftUpMid - sgUserInfo.u16LiftUpRange) - i32AdcValue)*255)/((sgUserInfo.u16LiftUpMid - sgUserInfo.u16LiftUpRange) - sgUserInfo.u16LiftUpMin);
	}
	else
	{
		SwiInput.b1LiftDown = 0;
		SwiInput.b1LiftUp = 0;
		u8PumpOrPropValue = 0;
	}
	
	i32AdcValue = i32LocalAiGetValue(LIFTSPEED_THROTTLE);
	if(i32AdcValue < sgUserInfo.u16LiftDownMin)
	{
		SwiInput.b1HighSpeedLimit = 1;
	}
	
	sgSwiInput.u16data = SwiInput.u16data;
	
	if(((0 == sgSwiInputLast.b1Forward)&&(1 == sgSwiInput.b1Forward))||((0 == sgSwiInputLast.b1Backward)&&(1 == sgSwiInput.b1Backward)))
	{
		if(1 == sgSwiInput.b1Ems)
		{
			i32ErrCodeSet(EmsLogic_ERR);
			sgu16LimitFlg |= EmsErr_Limit;
		}
		if(0 == sgSwiInput.b1SafeLock)
		{
			i32ErrCodeSet(MOVESAFET_ERR);
			sgu16LimitFlg |= MOVESAFETY_Limit;
		}
	}
	
	if(0 != sgUserInfo.b1LiftLock)
	{
		static uint8_t LiftFlgOk = 0;
		if(((1 == SwiInput.b1LiftUp)||(1 == SwiInput.b1LiftDown))&&(0 == sgSwiInput.b1SafeLock))
		{
			LiftFlgOk = 1;
		}
		else if(((1 == SwiInput.b1LiftUp)||(1 == SwiInput.b1LiftDown))&&(1 == sgSwiInput.b1SafeLock)&&(1 == LiftFlgOk))
		{
			i32ErrCodeSet(LIFTSAFET_ERR);
			sgu16LimitFlg |= LIFTSAFETY_Limit;
		}
		else if((0 == SwiInput.b1LiftUp)&&(0 == SwiInput.b1LiftDown)&&(0 == sgSwiInput.b1SafeLock))
		{
			LiftFlgOk = 0;
			i32ErrCodeClr(LIFTSAFET_ERR);
			sgu16LimitFlg &= ~LIFTSAFETY_Limit;
		}
	}
	
	if((0 == sgSwiInput.b1Forward)&&(0 == sgSwiInput.b1Backward)&&(0 == sgSwiInput.b1Ems))
	{
		if(0 != (sgu16LimitFlg & EmsErr_Limit))
		{
			i32ErrCodeClr(EmsLogic_ERR);
			sgu16LimitFlg &= ~EmsErr_Limit;
		}
	}
	
	if((0 == sgSwiInput.b1Forward)&&(0 == sgSwiInput.b1Backward)&&(0 == sgSwiInput.b1SafeLock))
	{
		if(0 != (sgu16LimitFlg & MOVESAFETY_Limit))
		{
			i32ErrCodeClr(MOVESAFET_ERR);
			sgu16LimitFlg &= ~MOVESAFETY_Limit;
		}
	}
	
	if((1 == SwiInput.b1Forward)&&(1 == SwiInput.b1Ems))
	{
		i32ErrCodeSet(EmsFord_ERR);
		sgu16LimitFlg |= EMSFord_LIMIT;
	}
	else
	{
		if((0 == SwiInput.b1SafeLock)&&(0 ==  SwiInput.b1Forward)&&(0 ==  SwiInput.b1Ems))
		{
			i32ErrCodeClr(EmsFord_ERR);
			sgu16LimitFlg &= ~EMSFord_LIMIT;
		}
	}
	
	
	if((1 == sgSwiInput.b1HighSpeedLimit)&&(1 ==sgSwiInput.b1LiftUp))
	{
		sgFlagVal.Height_SpeedLimit = 1;
		if(HeighSpeedLimit != sgFlagVal.Height_SpeedLimit)
		{
			u16EepromWrite(PARA_SaveState, sgSaveState.u16Data, 1);
			HeighSpeedLimit = sgFlagVal.Height_SpeedLimit;
		}
	}
	else if((1 == sgSwiInput.b1HighSpeedLimit)&&(1 == sgSwiInput.b1LiftDown))
	{
		sgFlagVal.Height_SpeedLimit = 0;
		if(HeighSpeedLimit != sgFlagVal.Height_SpeedLimit)
		{
			u16EepromWrite(PARA_SaveState, sgSaveState.u16Data, 1);
			HeighSpeedLimit = sgFlagVal.Height_SpeedLimit;
		}
	}
	sgSaveState.b1HeightSpdLimit = sgFlagVal.Height_SpeedLimit;
	
	sgValvesInfo.b1Gear1SpdFlag = 0;
	sgValvesInfo.b1Gear2SpdFlag = 0;
	sgValvesInfo.b1Gear3SpdFlag = 0;
	sgValvesInfo.b1Gear4SpdFlag = 0;
	
	
	if(0 == sgUserInfo.u8BatteryType)
	{
		PowerLow = BAT_LOW_ERR_VAL-5;
		PowerLLow = BAT_LOW_WARING_VAL-5;
	}
	else
	{
		PowerLow = BAT_LOW_ERR_VAL;
		PowerLLow = BAT_LOW_WARING_VAL;
	}

	if(gCanSendPdoInfo.CanSend260Info.u8Soc < PowerLow)
	{
		i32ErrCodeSet(POWERLOW_ERR);
		sgValvesInfo.b1Gear1SpdFlag = 1;
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 0;
		sgValvesInfo.b1Gear4SpdFlag = 0;
		sgu16LimitFlg |= LowPowerLift_Limit;
	}
	else if(gCanSendPdoInfo.CanSend260Info.u8Soc < PowerLLow)
	{
		i32ErrCodeSet(POWERLLOW_ERR);
	}
	else
	{
		sgu16LimitFlg &= ~LowPowerLift_Limit;
		i32ErrCodeClr(POWERLOW_ERR);
		i32ErrCodeClr(POWERLLOW_ERR);
	}
	
	if(1 == SwiInput.b1HighSpeedLimit)
	{
		sgValvesInfo.b1Gear1SpdFlag = 1;
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 0;
		sgValvesInfo.b1Gear4SpdFlag = 0;
	}
	
	{
		static uint8_t LiftFlg = 0;
		static uint8_t ZuoYouYiFlg = 0;
		static uint8_t QianHouQinFlg = 0;
		static uint8_t QianHouYiFlg = 0;
		static uint8_t MoveFlg = 0;
		/*先有起升 只会起升*/
		if(((1 == SwiInput.b1LiftUp)||(1 == SwiInput.b1LiftDown))&&(0 == ZuoYouYiFlg)&&(0 == QianHouYiFlg)&&(0 == QianHouQinFlg)&&(0 == MoveFlg))
		{
			LiftFlg = 1;
			if(0 != (sgUserInfo.b1MoveLiftMode & 0x01)) 
			{
				if((1 == SwiInput.b1Forward)||(1 == SwiInput.b1Backward))
				{
					sgu16LimitFlg |= LiftMoveMode_Limit;
				}
				else
				{
					sgu16LimitFlg &= ~LiftMoveMode_Limit;
				}
			}
			if((1 == SwiInput.b1ZuoYi)||(1 == SwiInput.b1YouYi))
			{
				sgu16LimitFlg |= ZuoYouYi_Limit;
			}
			else
			{
				sgu16LimitFlg &= ~ZuoYouYi_Limit;
			}
			if((1 == SwiInput.b1QianQin)||(1 == SwiInput.b1HouQin))
			{
				sgu16LimitFlg |= QianHouQin_Limit;
			}
			else
			{
				sgu16LimitFlg &= ~QianHouQin_Limit;
			}
			if((1 == SwiInput.b1QianYi)||(1 == SwiInput.b1HouYi))
			{
				sgu16LimitFlg |= QianHouYi_Limit;
			}
			else
			{
				sgu16LimitFlg &= ~QianHouYi_Limit;
			}
		}
		else
		{
			LiftFlg = 0;
			if(0 != (sgUserInfo.b1MoveLiftMode & 0x01)) 
			{
				if((0 == SwiInput.b1Forward)&&(0 == SwiInput.b1Backward))
				{
					sgu16LimitFlg &= ~LiftMoveMode_Limit;
				}
			}
			if((0 == SwiInput.b1ZuoYi)&&(0 == SwiInput.b1YouYi))
			{
				sgu16LimitFlg &= ~ZuoYouYi_Limit;
			}
			if((0 == SwiInput.b1QianQin)&&(0 == SwiInput.b1HouQin))
			{
				sgu16LimitFlg &= ~QianHouQin_Limit;
			}
			if((0 == SwiInput.b1QianYi)&&(0 == SwiInput.b1HouYi))
			{
				sgu16LimitFlg &= ~QianHouYi_Limit;
			}
		}
		
		/*先有前后移 只会前后移*/
		if(((1 == SwiInput.b1QianYi)||(1 == SwiInput.b1HouYi))&&(0 == ZuoYouYiFlg)&&(0 == LiftFlg)&&(0 == QianHouQinFlg)&&(0 == MoveFlg))
		{
			QianHouYiFlg = 1;
			if(0 != (sgUserInfo.b1MoveLiftMode & 0x01)) 
			{
				if((1 == SwiInput.b1Forward)||(1 == SwiInput.b1Backward))
				{
					sgu16LimitFlg |= LiftMoveMode_Limit;
				}
				else
				{
					sgu16LimitFlg &= ~LiftMoveMode_Limit;
				}
			}
			if((1 == SwiInput.b1ZuoYi)||(1 == SwiInput.b1YouYi))
			{
				sgu16LimitFlg |= ZuoYouYi_Limit;
			}
			else
			{
				sgu16LimitFlg &= ~ZuoYouYi_Limit;
			}
			if((1 == SwiInput.b1QianQin)||(1 == SwiInput.b1HouQin))
			{
				sgu16LimitFlg |= QianHouQin_Limit;
			}
			else
			{
				sgu16LimitFlg &= ~QianHouQin_Limit;
			}
			if((1 == SwiInput.b1LiftUp)||(1 == SwiInput.b1LiftDown))
			{
				sgu16LimitFlg |= Lift_Limit;
			}
			else
			{
				sgu16LimitFlg &= ~Lift_Limit;
			}
		}
		else
		{
			QianHouYiFlg = 0;
			if((0 == SwiInput.b1ZuoYi)&&(0 == SwiInput.b1YouYi))
			{
				sgu16LimitFlg &= ~ZuoYouYi_Limit;
			}
			if((0 == SwiInput.b1QianQin)&&(0 == SwiInput.b1HouQin))
			{
				sgu16LimitFlg &= ~QianHouQin_Limit;
			}
			if((0 == SwiInput.b1LiftUp)&&(0 == SwiInput.b1LiftDown))
			{
				sgu16LimitFlg &= ~Lift_Limit;
			}
		}
		
		/*前后倾*/
		if(((1 == SwiInput.b1QianQin)||(1 == SwiInput.b1HouQin))&&(0 == ZuoYouYiFlg)&&(0 == LiftFlg)&&(0 == QianHouYiFlg)&&(0 == MoveFlg))
		{
			QianHouQinFlg = 1;
			if(0 != (sgUserInfo.b1MoveLiftMode & 0x01)) 
			{
				if((1 == SwiInput.b1Forward)||(1 == SwiInput.b1Backward))
				{
					sgu16LimitFlg |= LiftMoveMode_Limit;
				}
				else
				{
					sgu16LimitFlg &= ~LiftMoveMode_Limit;
				}
			}
			if((1 == SwiInput.b1ZuoYi)||(1 == SwiInput.b1YouYi))
			{
				sgu16LimitFlg |= ZuoYouYi_Limit;
			}
			else
			{
				sgu16LimitFlg &= ~ZuoYouYi_Limit;
			}
			if((1 == SwiInput.b1QianYi)||(1 == SwiInput.b1HouYi))
			{
				sgu16LimitFlg |= QianHouYi_Limit;
			}
			else
			{
				sgu16LimitFlg &= ~QianHouYi_Limit;
			}
			if((1 == SwiInput.b1LiftUp)||(1 == SwiInput.b1LiftDown))
			{
				sgu16LimitFlg |= Lift_Limit;
			}
			else
			{
				sgu16LimitFlg &= ~Lift_Limit;
			}
		}
		else
		{
			QianHouQinFlg = 0;
			if(0 != (sgUserInfo.b1MoveLiftMode & 0x01)) 
			{
				if((0 == SwiInput.b1Forward)&&(0 == SwiInput.b1Backward))
				{
					sgu16LimitFlg &= ~LiftMoveMode_Limit;
				}
			}
			if((0 == SwiInput.b1ZuoYi)&&(0 == SwiInput.b1YouYi))
			{
				sgu16LimitFlg &= ~ZuoYouYi_Limit;
			}
			if((0 == SwiInput.b1QianYi)&&(0 == SwiInput.b1HouYi))
			{
				sgu16LimitFlg &= ~QianHouYi_Limit;
			}
			if((0 == SwiInput.b1LiftUp)&&(0 == SwiInput.b1LiftDown))
			{
				sgu16LimitFlg &= ~Lift_Limit;
			}
		}
		
		/*左右移*/
		if(((1 == SwiInput.b1ZuoYi)||(1 == SwiInput.b1YouYi))&&(0 == QianHouQinFlg)&&(0 == LiftFlg)&&(0 == QianHouYiFlg)&&(0 == MoveFlg))
		{
			ZuoYouYiFlg = 1;
			if(0 != (sgUserInfo.b1MoveLiftMode & 0x01)) 
			{
				if((1 == SwiInput.b1Forward)||(1 == SwiInput.b1Backward))
				{
					sgu16LimitFlg |= LiftMoveMode_Limit;
				}
				else
				{
					sgu16LimitFlg &= ~LiftMoveMode_Limit;
				}
			}
			if((1 == SwiInput.b1QianQin)||(1 == SwiInput.b1HouQin))
			{
				sgu16LimitFlg |= QianHouQin_Limit;
			}
			else
			{
				sgu16LimitFlg &= ~QianHouQin_Limit;
			}
			if((1 == SwiInput.b1QianYi)||(1 == SwiInput.b1HouYi))
			{
				sgu16LimitFlg |= QianHouYi_Limit;
			}
			else
			{
				sgu16LimitFlg &= ~QianHouYi_Limit;
			}
			if((1 == SwiInput.b1LiftUp)||(1 == SwiInput.b1LiftDown))
			{
				sgu16LimitFlg |= Lift_Limit;
			}
			else
			{
				sgu16LimitFlg &= ~Lift_Limit;
			}
		}
		else
		{
			ZuoYouYiFlg = 0;
			if(0 != (sgUserInfo.b1MoveLiftMode & 0x01)) 
			{
				if((0 == SwiInput.b1Forward)&&(0 == SwiInput.b1Backward))
				{
					sgu16LimitFlg &= ~LiftMoveMode_Limit;
				}
			}
			if((0 == SwiInput.b1QianQin)&&(0 == SwiInput.b1HouQin))
			{
				sgu16LimitFlg &= ~QianHouQin_Limit;
			}
			if((0 == SwiInput.b1QianYi)&&(0 == SwiInput.b1HouYi))
			{
				sgu16LimitFlg &= ~QianHouYi_Limit;
			}
			if((0 == SwiInput.b1LiftUp)&&(0 == SwiInput.b1LiftDown))
			{
				sgu16LimitFlg &= ~Lift_Limit;
			}
		}
		
		/*行走时不可以起升 起身时不可以行走*/
		if(0 != (sgUserInfo.b1MoveLiftMode & 0x01)) 
		{
			if(((1 == SwiInput.b1Forward)||(1 == SwiInput.b1Backward))&&(0 == ZuoYouYiFlg)&&(0 == QianHouYiFlg)&&(0 == QianHouQinFlg)&&(0 == LiftFlg))
			{
				MoveFlg = 1;
				if((1 == SwiInput.b1LiftUp)||(1 == SwiInput.b1LiftDown))
				{
					sgu16LimitFlg |= Lift_Limit;
				}
				else
				{
					sgu16LimitFlg &= ~Lift_Limit;
				}
				if((1 == SwiInput.b1ZuoYi)||(1 == SwiInput.b1YouYi))
				{
					sgu16LimitFlg |= ZuoYouYi_Limit;
				}
				else
				{
					sgu16LimitFlg &= ~ZuoYouYi_Limit;
				}
				if((1 == SwiInput.b1QianQin)||(1 == SwiInput.b1HouQin))
				{
					sgu16LimitFlg |= QianHouQin_Limit;
				}
				else
				{
					sgu16LimitFlg &= ~QianHouQin_Limit;
				}
				if((1 == SwiInput.b1QianYi)||(1 == SwiInput.b1HouYi))
				{
					sgu16LimitFlg |= QianHouYi_Limit;
				}
				else
				{
					sgu16LimitFlg &= ~QianHouYi_Limit;
				}
			}
			else
			{
				MoveFlg = 0;
				if(0 != (sgUserInfo.b1MoveLiftMode & 0x01)) 
				{
					if((0 == SwiInput.b1Forward)&&(0 == SwiInput.b1Backward))
					{
						sgu16LimitFlg &= ~LiftMoveMode_Limit;
					}
				}
				if((0 == SwiInput.b1ZuoYi)&&(0 == SwiInput.b1YouYi))
				{
					sgu16LimitFlg &= ~ZuoYouYi_Limit;
				}
				if((0 == SwiInput.b1QianQin)&&(0 == SwiInput.b1HouQin))
				{
					sgu16LimitFlg &= ~QianHouQin_Limit;
				}
				if((0 == SwiInput.b1QianYi)&&(0 == SwiInput.b1HouYi))
				{
					sgu16LimitFlg &= ~QianHouYi_Limit;
				}
				if((0 == SwiInput.b1LiftUp)&&(0 == SwiInput.b1LiftDown))
				{
					sgu16LimitFlg &= ~Lift_Limit;
				}
			}
		}	
	}
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
	vMstRevRegister(vMstRevProc);
	vMstSendRegister(vMstSendProc);
	
	vDoPwmErrReg(vDoPwmErrCallBack);
	vPropErrReg(vPropErrCallBack);
	vAiErrReg(vAiErrCallBack);

	memset(&sgValvesInfo, 0, sizeof(sgValvesInfo));
	
	sgUserInfo.fPropMinCurrent0 = i32GetPara(PROP_MIN_CURRENT0) / 1000.0;		/*比例阀最小电流*/
	sgUserInfo.fPropMaxCurrent0 = i32GetPara(PROP_MAX_CURRENT0) / 1000.0;
	sgUserInfo.fPropMinCurrent1 = i32GetPara(PROP_MIN_CURRENT1) / 1000.0;		/*比例阀最小电流*/
	sgUserInfo.fPropMaxCurrent1 = i32GetPara(PROP_MAX_CURRENT1) / 1000.0;		/*比例阀最大电流*/
	
	sgUserInfo.u8LeanForWardValue = i32GetPara(LEAN_FORWARD_PARA) * PUMP_RANGE / 100;		/*前倾参数*/
	sgUserInfo.u8LeanBackWardValue = i32GetPara(LEAN_BACKWARD_PARA) * PUMP_RANGE / 100;		/*后倾参数*/
	
	sgUserInfo.b1LiftMode = i32GetPara(LIFT_MODE_PARA);						/*起升模式*/
	
	sgUserInfo.u16Gear1Spd = i32GetPara(PARA_Gear1Spd);	/*1档速度*/
	sgUserInfo.u16Gear2Spd = i32GetPara(PARA_Gear2Spd);	/*2档速度*/
	sgUserInfo.u16Gear3Spd = i32GetPara(PARA_Gear3Spd);	/*3档速度*/
	sgUserInfo.u16Gear4Spd = i32GetPara(PARA_Gear4Spd);	/*4档速度*/
	
	sgUserInfo.u16ThrottleMin = i32GetPara(MOVE_THROTTLE_MIN) * 100;		/*油门死区值, uint 0.1V*/
	sgUserInfo.u16ThrottleMax = i32GetPara(MOVE_THROTTLE_MAX) * 100;		/*油门死区值最大 ,uint 0.1V*/
	sgUserInfo.u16ThrottleRange = (sgUserInfo.u16ThrottleMax - sgUserInfo.u16ThrottleMin) >> 1;
	sgUserInfo.u16ThrottleMid = sgUserInfo.u16ThrottleMax + sgUserInfo.u16ThrottleMin;
	
	sgUserInfo.u16LiftUpMin = i32GetPara(LIFT_UP_THROTTLE_MIN) * 100;
	sgUserInfo.u16LiftUpMax = i32GetPara(LIFT_UP_THROTTLE_MAX) * 100;
	sgUserInfo.u16LiftUpRange = i32GetPara(LIFT_UP_THROTTLE_MID) * 100;
	sgUserInfo.u16LiftUpMid = (sgUserInfo.u16LiftUpMax + sgUserInfo.u16LiftUpMin)/2;
	
	sgUserInfo.u16LiftDownMin = i32GetPara(PARA_Analog3DeadZoneMinVal) * 100;
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
	
	sgUserInfo.u8BatteryType = i32GetPara(PARA_BatteryType);
	steerAngleTab = i32GetPara(PARA_PumpMotorGear1);

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
//	if (0x5555 != u16Tmp)
//	{
//		u16EepromWrite(PARA_SaveState, sgSaveState.u16Data, 1);
//		u16EepromWrite(PARA_HourSetTime, 0x0000, 1);
//		u16EepromWrite(PARA_DefaultFlag, 0x5555, 1);	
//	}
//	else
//	{
		u16EepromRead(PARA_SaveState, &sgSaveState.u16Data, 1);
//		sgSwiInput.b1HeightLimit = sgSaveState.b1Above1M8;
		sgFlagVal.Height_SpeedLimit = sgSaveState.b1HeightSpdLimit;        //高度限速保存到eeprom
		HeighSpeedLimit = sgFlagVal.Height_SpeedLimit;
		sgUserInfo.b1PasswordFunc = sgSaveState.b1PasswordFunc;
		u16EepromRead(PARA_HourSetTime, &u16Tmp, 1);	
//	}

	gCanSendPdoInfo.CanSend260Info.u8HourCountL = u32HourCount & 0xFF;
	gCanSendPdoInfo.CanSend260Info.u8HourCountH = (u32HourCount >> 8) & 0xFF;
	
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
//	
	vCanIdLostReg(0x444,3000,vCanLostProc);
	vCanIdLostReg(0x360,1000,vCanLostProc);

	vSetPdoPara(sgPdoPara);
	
	i32LocalDoSet(DO_ENCODER2, 1);
	
	vSetNetTimer(TIMER_EcuPowerOn, ECU_POWERON_DELAY_TIME);	/**/
	
	vSetNetTimer(TIMER_HourCount, HOURCOUNT_PERIOD);
	
	gCanSendPdoInfo.CanSend260Info.u8Soc = 50;
	
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
	uint16_t ZhiZhenAddre;
	void * MotorVal;
	MotorVal = &ZhiZhenAddre;

	if(true == u8GetNetTimerOverFlag(TIMER_EcuPowerOn))
	{
		u8EcuProcFlag = 1;
		vKillNetTimer(TIMER_EcuPowerOn);
	}
	 
		if(u8MainConnectCnt < 100)
		{
			if((1 == i32LocalDiGet(HouYi_SWI))||(1 == i32LocalDiGet(QianYi_SWI))
				||(1 == i32LocalDiGet(EmergencyReverse))||(1 == i32LocalDiGet(SAFELOCK_SWI))
				||(1 == i32LocalDiGet(QianQin_SWI))||(1 == i32LocalDiGet(HouQin_SWI))
				||(1 == i32LocalDiGet(YouYi_SWI))||(1 == i32LocalDiGet(ZuoYi_SWI))
				||((i32LocalAiGetValue(LIFT_THROTTLE) > (sgUserInfo.u16LiftUpMid+sgUserInfo.u16LiftUpRange))||(i32LocalAiGetValue(LIFT_THROTTLE) < (sgUserInfo.u16LiftUpMid-sgUserInfo.u16LiftUpRange)))
			)
			{
				check_err = 1;
				i32ErrCodeSet(POWENON_ERR);
				sgValvesInfo.b1NoActFlag = 1;
			}
			else
			{
				u8MainConnectCnt++;
			}
		}
		else
		{
			if(1 == check_err)
			{
				if((0 == i32LocalDiGet(HouYi_SWI))&&(0 == i32LocalDiGet(QianYi_SWI))
					&&(0 == i32LocalDiGet(EmergencyReverse))&&(0 == i32LocalDiGet(SAFELOCK_SWI))&&(0 == i32LocalDiGet(QianQin_SWI))
					&&(0 == i32LocalDiGet(HouQin_SWI))&&(0 == i32LocalDiGet(YouYi_SWI))&&(0 == i32LocalDiGet(ZuoYi_SWI))
					&&((i32LocalAiGetValue(LIFT_THROTTLE) <= (sgUserInfo.u16LiftUpMid+sgUserInfo.u16LiftUpRange))&&(i32LocalAiGetValue(LIFT_THROTTLE) >= (sgUserInfo.u16LiftUpMid-sgUserInfo.u16LiftUpRange)))
				)
				{
					check_err = 0;
					i32ErrCodeClr(POWENON_ERR);
					sgValvesInfo.b1NoActFlag = 0;
				}
			}
		}
	

	
	
	if(1 == u8EcuProcFlag)
	{
		vSwiMonitor();
		vCanRevPdoProc();
		vAiMonitor();
		u8ErrCode = u8ErrCodeGet();
		if (0 != u8ErrCode)
		{
			vLedSendAlmCode(u8ErrCode);
			gCanSendPdoInfo.CanSend260Info.u8ErrorMove = u8ErrSwitchArray[u8ErrCode];
		}
		else
		{
			gCanSendPdoInfo.CanSend260Info.u8ErrorMove = 0;
		}
	}
	
	if (true == u8GetNetTimerOverFlag(TIMER_HourCount))
	{
		vResetNetTimer(TIMER_HourCount);
		if (1 == sgUserInfo.b1HourConutMode)
		{
			if (1 == sgSwiInput.b1SafeLock)
			{
				u16SecCnt++;
			}
		}
		else
		{
			u16SecCnt++;
		}
		if (u16SecCnt >= 360)			/*360*/
		{
			u16SecCnt = 0;
			u32HourCount++;
			vHourCountWrite(u32HourCount);
		}
		gCanSendPdoInfo.CanSend260Info.u8HourCountL = u32HourCount & 0xFF;
		gCanSendPdoInfo.CanSend260Info.u8HourCountH = (u32HourCount >> 8) & 0xFF;
	}
	
	if(0 != sgUserInfo.b1LiftPedal)
	{
		u32HourCount = 0;
		vHourCountWrite(0);
		gCanSendPdoInfo.CanSend260Info.u8HourCountL = 0;
		gCanSendPdoInfo.CanSend260Info.u8HourCountH = 0;
	}
	
	vWdgSetFun(WDG_USER_BIT);
}
#endif
