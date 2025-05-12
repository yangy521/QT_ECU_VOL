/*******************************************************************************
* Filename: UserStackerTruckProc.c	                                           *
* Description: 逻辑层的源文件，包含对应的回调函数	   						   *
* Author:                                                      *
* Date: 2023/05/30    														   *
* Revision:	 														 		   *
*******************************************************************************/
#include "UserCanComm.h"
#include "USER_HANGCHA_PHZ.h"
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

#if (USER_TYPE == USER_HANGCHA_PHZ)

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

const static uint8_t u8ErrSwitchArray[ErrCodeMax] = 
{	
/*  0    1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18   19*/
    0,   1,   2,   3,   4,   5,   6,   7,   8,   9,   10,  11,  12,  13,  14,  15,  16,  17,  18,  19,
    20,  21,  22,  23,  24,  25,  26,  27,  28,  29,  30,  31,  32,  33,  34,  35,  36,  37,  38,  39,
    40,  41,  42,  43,  44,  45,  46,  47,  48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,
		60,  61,  62,  26,  64,  65,  66,  67,  68,  69,  70,  71,  72,  73,  74,  75,  76,  77,  78,  79, 
		80,  81,  82,  83,  84,  85,  86,  87,  88,  89,  90,  91,  92,  93,  94,  95,  96,  97,  98,  99,
		100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 
		120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 
		140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159,
		160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 
		180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 
		200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 
		220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 
		240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254,
};


const static xPdoParameter  sgPdoPara = 
{
	.TpdoPara = {
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 50, .u16CanId = 0x258},
//		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 100, .u16CanId = 0x260},
	},
	.RpdoPara = {
		{.b1Flag = 1, .b11CanRevId = 0x360},
		{.b1Flag = 1, .b11CanRevId = 0x2F0},
		{.b1Flag = 1, .b11CanRevId = 0x1E0},
		{.b1Flag = 1, .b11CanRevId = 0x2F1},
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
#define PedalMove_Limit			(0x01 << 3)
#define BMSMove_Limit				(0x01 << 4)
#define BMSLiftUp_Limit			(0x01 << 5)
#define EmsLimitMove				(0x01 << 6)
#define NoDelayLimit				(0x01 << 7)
#define EmsErrFlg 					(0x01 << 8)
#define BMSErrStop					(0x01 << 9)
#define BMSErrSpeed_Limit		(0x01 << 10)

#define QianHouYi_Limit			(0x01 << 11)
#define ZuoYouYi_Limit			(0x01 << 12)
#define QianHouQin_Limit		(0x01 << 13)

#define Stree_Err_Limit			(0x01 << 14)
#define BmsLost_Limit				(0x01 << 15)
#define StreeLost_Limit			(0x01 << 16)
#define SHOUBinLost_Limit		(0x01 << 17)

#define APPWRITE_LIMIT			(0x01 << 30)

/*SdoReqCmd*/
#define Read_Cmd						(0x01 << 0)
#define ReadOk_Cmd					(0x01 << 1)
#define Write_Cmd						(0x01 << 2)
#define WriteOk_Cmd					(0x01 << 3)


#define WRITE_SUCESS    0
#define WRITE_FAIL      1
typedef union
{
	uint16_t u16data;
	struct
	{
		uint16_t b1Height500Spdlimit: 1;         //高度限速
		uint16_t b1Ems: 1;                 //急反开关
		uint16_t b1SafeLock: 1;               //互锁
		uint16_t b1Pedal: 1;		
		uint16_t b1StreeLock: 1;		
		uint16_t b1Human: 1; 
		uint16_t b1Height1800Spdlimit: 1;
		uint16_t b1FENCE1: 1;		
 		uint16_t b1FENCE2: 1;                           
		uint16_t b1LiftLimit: 1;
		uint16_t b1Forward: 1;
		uint16_t b1Backward: 1;
		uint16_t b7Reserve: 4;
	};
}xSwiInput;

typedef union
{
	uint32_t u32data;
	struct
	{
		uint32_t b1TmpData0: 1;
		uint32_t b1TmpData1: 1;
		uint32_t b1TmpData2: 1;
		uint32_t b1TmpData3: 1;
		uint32_t b1TmpData4: 1;
		uint32_t b1TmpData5: 1;
		uint32_t b1TmpData6: 1;
		uint32_t b1TmpData7: 1;
		uint32_t b1TmpData8: 1;
		uint32_t b1TmpData9: 1;
		uint32_t b1TmpData10: 1;
		uint32_t b1TmpData11: 1;
		uint32_t b1TmpData12: 1;
		uint32_t b1TmpData13: 1;
		uint32_t b1TmpData14: 1;
		uint32_t b1TmpData15: 1;
		uint32_t b16Reserve: 16;
	};
}DataTmpR;

//typedef union
//{
//	uint16_t RevAddress;
//	uint8_t RevCs;
//	uint8_t RevCmd;
//	uint8_t FuncCode;
//	
//}xRevCodeCmd;

typedef union
{
	uint8_t u8data;
	struct
	{
		uint8_t b1NoAct: 1;
		uint8_t b1Ems: 1;
		uint8_t b1SlowMode: 1;
		uint8_t b1DIDIOn: 1;
		
		uint8_t b1ZuoLiftDown: 1;
		uint8_t b1ZuoLiftUp: 1;
		uint8_t b1YouLiftUp: 1;
		uint8_t b1YouLiftDown: 1;
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
	uint8_t  	 b1HourCntClr: 1;
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
	
	uint16_t MostLiftUpTim;
	uint16_t MostQianQinTim;
	uint16_t MostHouQinTim;
	
	uint16_t VehicleType;

	uint16_t u16LiftShortTim;
	uint16_t u16LiftShortHold;
	
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

typedef union 
{
	uint8_t u8Data;
	struct
	{
		uint8_t b1ServonState:1;
		uint8_t b1Do1State:1;
		uint8_t b1EbrakeState:1;
		uint8_t b5Reserve5:5;
	};	
}_xMcuState;
xSaveStateInfo sgSaveState;
const static uint8_t u8SeedArray[16]= "V!BWT%EM6dJ8<nPs";

static int16_t	i16MotorSpd = 0;
static uint16_t u16MotorVal = 0;
static uint8_t	u8PumpOrPropValue = 0;
static uint8_t u8PumpOrPropValueMax = 0;
static uint16_t u16CanId5ACPeriod = 0;
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
static uint8_t u8PumpOrPropValueMove = 0;
static uint32_t u32QianHouYiContinue = 0;
static uint32_t u32ZuoYouYiContinue = 0;
static uint32_t u32QianHouQinContinue = 0;
static uint8_t LiftDelayFlg = 0;
static uint8_t QianHouYiDelayFlg = 0;
static uint8_t QianHouQinDelayFlg = 0;
static uint8_t ZuoYouYiDelayFlg = 0;
static uint8_t u8PropStopClose = 0;
static uint32_t u32SwitchOverDelay = 0;
static uint8_t LiftWorking = 0;
static uint8_t QianHouYiWorking = 0;
static uint8_t QianHouQinWorking = 0;
static uint8_t ZuoYouYiWorking = 0;
static uint8_t HourCount8L = 0;
static uint16_t HourCount16H = 0;
static uint8_t WorkCount8L = 0;
static uint16_t WorkCount16H = 0;
static uint16_t steerAngleTab = 0;
static uint8_t SteetSpeedLimit = 0;
static uint8_t sdfagsadgagag = 0;
static uint8_t u8Index = 0;
uint32_t u32RecieveAdress = 0;
uint16_t u16SendAdress;
uint16_t u16Factor;
static uint8_t RevIndex = 0;
volatile static DataTmpR tmpData;
static uint8_t u8HumanLostStop = 0;
static uint8_t u8ErrCodeCir[ErrCodeMax]={0};

static _xMcuState sgMcuState;
//static uint32_t McuBitNum[220]={0};
volatile uint8_t RevCmd;
static uint8_t HourCntFlg = 0;
static uint8_t WorkCntFlg = 0;
static uint32_t WorkRest = 0;
static uint32_t HourRest = 0;
static uint8_t WorkRestFlg = 0;
static uint8_t HourRestFlg = 0;
uint16_t WorkCountL = 0;
uint16_t WorkCountH = 0;
static uint32_t Work2App = 0;
static uint32_t Hour2App = 0;
static INT8U Count = PARA_ErrCode0;               //历史故障结束之后 ++
static uint8_t CountLast = PARA_ErrCode0+1;
volatile static uint32_t MstPARADate[256];
volatile static uint16_t ParaCount = 1;
//volatile static uint16_t SendCount = 0;
volatile static uint8_t MstParaIsOk = 0;
volatile static uint16_t RevMstDate = 1;
volatile static uint16_t ReceiveCount = 0;
static uint8_t PowerOnGetPara = true;
volatile static uint8_t SendOk = 0;
static uint8_t ErrCodeNowSend = 0;
static uint16_t RevErrCodeAddr = 300;
static uint8_t RevErrCCount = 0;
static uint8_t SendHistoryErrCode = 0;

static SdoReqCmd BSdoState;

/*PDO接收待完善*/
xCanRevPdoInfo gCanRevPdoInfo; 
static xCanRev360Info CanRev360InfoLast;
static xCanRev2F0Info CanRev2F0InfoLast;
static xCanRev1E0Info CanRev1E0InfoLast;
static xCanRev2F1Info CanRev2F1InfoLast;
static xCanRev670Info CanRev670InfoLast;
static xCanRev640Info CanRev640InfoLast;
/*PDO发送待完善*/
xCanSendPdoInfo gCanSendPdoInfo;
_tHANGCHADGC_PDO HangChaPdo;
static xCanSend5C0Info CanSend5C0Last;

static void vMstRevProc(xMstRevPara *RevData)
{	
	uint16_t tmp = 0;
	static uint8_t tmpLast = 0;
	if(0 != RevData->b1SvonState)
	{
//		sgMcuState.b1ServonState = 1;
	}
	else
	{
//		sgMcuState.b1ServonState = 0;
	}
	if(0 != RevData->b1MainDriver)
	{
		sgMcuState.b1Do1State = 1;
	}
	else
	{
		sgMcuState.b1Do1State = 0;
	}
	
	if(0 != RevData->b1Ebrake)
	{
		sgMcuState.b1EbrakeState = 1;
	}
	else
	{	
		sgMcuState.b1EbrakeState = 0;
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
	gCanSendPdoInfo.CanSend260Info.u8Movespeed = tmp;
	Rev_Speed	= i16MotorSpd;
	RevSPEED = tmp;
	__enable_irq();
	
	tmp = RevData->u8MotorTmp;
	/*Motor Temp*/
	
	tmp = RevData->u8BoardTmp;
	/**/
//	if(0 != i32GetPara(RENTAL_INFO))
//	{
//			if(sgUserInfo.Temputer > 120)          // 19 34
//		{
//			i32ErrCodeSet(18);
//			u8LimitMove = 1;
//		}
//		else if(sgUserInfo.Temputer > 80)
//		{
//			SpeedRate = 50;
//			i32ErrCodeSet(33);
//		}
//		else if(sgUserInfo.Temputer < -30)
//		{
//			i32ErrCodeSet(31);
//			u8LimitMove = 1;
//		}
//	}
	__disable_irq();
//	gCanSendPdoInfo.CanSend43CInfo.i16MotorCurrent = (RevData->u8CurrentHigh << 8) | RevData->u8CurrentLow;
	__enable_irq();
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
	static uint8_t WorkMode = 0;
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
	uint16_t LiftDownValApp = 0;
	static uint16_t LiftUpCont = 0;
	static uint16_t QianQinCont = 0;
	static uint16_t HouQinCont = 0;
	static uint16_t LiftKeyTim = 0;
	static uint32_t LiftUpKeyControlTim = 0;
	static uint8_t u8LiftUpStateFlg = 0;
	static uint16_t ContinueTim = 0;
	
	if(LSC0407DEM != sgUserInfo.VehicleType)
	{
		if(0 == sgHandleInput.b1YouLiftUp)
		{
			LiftUpCont = sgUserInfo.MostLiftUpTim;
		}
		
		if(0 == sgHandleInput.b1ZuoLiftDown)
		{
			HouQinCont = sgUserInfo.MostHouQinTim;
		}
		
		if(0 == sgHandleInput.b1ZuoLiftUp)
		{
			QianQinCont = sgUserInfo.MostQianQinTim;
		}
	}
	else
	{
		if((0 == sgHandleInput.b1YouLiftUp)&&(0 == sgHandleInput.b1ZuoLiftUp))
		{
			LiftUpCont = sgUserInfo.MostLiftUpTim;
		}
	}

		
	LastStatus.u8Data = SendData->buf[2];
	SendData->buf[2] = 0;
	
	if((0 == sgSwiInput.b1SafeLock)&&(0 == u16MotorVal))
	{
		SendData->b1PowerLineOn = 1;     // 1:断开 0:连接
	}
	
	if((0 != (sgu16LimitFlg & BMSErrSpeed_Limit))||(41 == CanRev360InfoLast.u8ErrSteer))
	{
		if(u16MotorVal > (4096*30/100))
		{
			u16MotorVal = 4096*30/100;
		}
	}

	/*Move Mode*/
	if((0 == (sgu16LimitFlg & (BMSErrStop|Move_Limit|PedalMove_Limit|EmsLimitMove|EmsErrFlg|Stree_Err_Limit|BmsLost_Limit)))&&(0 == u8LimitMove)&&(0 == u8EmsErrFlg)&&(33 != u8ErrCodeGet()))
	{
		SendData->b1ServoOn = 1;
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
				EmsOk = 1;
				i32ErrCodeSet(39);
			}
		}
		
		if((1 == sgSwiInput.b1Ems)&&(0 == u16MotorVal))
		{
			SendData->b1ServoOn = 0;
		}
		
		if((0 == sgSwiInput.b1Ems)&&((0 == sgSwiInput.b1Forward)||(0 == sgSwiInput.b1Backward)))
		{
			if(EmsOk == 1)
			{
				i32ErrCodeClr(39);
			}
		}
		
		{
			if((1 == sgSwiInput.b1Ems)&&(0 != u16MotorVal)&&(0 == Rev_Speed)&&(1 == sgSwiInput.b1Backward))
			{
				i32ErrCodeSet(39);
				EMSRecive = 1;
			}
			else if((0 == sgSwiInput.b1Ems)&&(0 == u16MotorVal))
			{
				if(1 == EMSRecive)
				{
					EMSRecive = 0;
					i32ErrCodeClr(39);
				}
			}
		}
	}	
	/*Lift Mode*/
	if(LSC0407DEM != sgUserInfo.VehicleType)
	{	
		WorkMode = 0;
		i32DoPwmSet(PROPDRIVER_VALVE,DRIVER_CLOSE);	
		i32DoPwmSet(LIFTUP_VALVE,DRIVER_CLOSE);
		i32DoPwmSet(HOUQIN_VALVE,DRIVER_CLOSE);
		i32DoPwmSet(QIANQIN_VALVE,DRIVER_CLOSE);
		vPropSetTarget(LIFTDOWN_VALVE, 0);
		
		if((1 == sgHandleInput.b1YouLiftDown)&&(0 == sgHandleInput.b1YouLiftUp)&&(0 == (sgu16LimitFlg & LiftDown_Limit)))
		{
			LiftDownValApp = u8PumpOrPropValueMove;
			i32PropValue = _IQ((sgUserInfo.fPropMinCurrent + (sgUserInfo.fPropMaxCurrent - sgUserInfo.fPropMinCurrent) * u8PumpOrPropValueMove / PUMP_RANGE) / PROPD_STD_CURRENT);
			WorkMode = 1;
			vPropSetTarget(LIFTDOWN_VALVE, i32PropValue);
			i32DoPwmSet(PROPDRIVER_VALVE,DRIVER_CLOSE);	
		}
		else if((1 == sgHandleInput.b1YouLiftUp)&&(0 == sgHandleInput.b1YouLiftDown)
						&&(0 == (sgu16LimitFlg & (LiftUp_Limit|BMSErrStop)))
						&&(0 == sgSwiInput.b1LiftLimit)
						&&(0 == u8PowerLowLimitL))
		{
//			if(LiftUpCont > 0)
			{
				LiftUpCont--;
				WorkMode = 1;
				i32DoPwmSet(PROPDRIVER_VALVE,DRIVER_OPEN);	
				i32DoPwmSet(LIFTUP_VALVE,DRIVER_OPEN);
				vPropSetTarget(LIFTDOWN_VALVE, 0);	
			}
			
		}
		
		if((1 == sgHandleInput.b1ZuoLiftUp)&&(0 == sgHandleInput.b1ZuoLiftDown))
		{
			if(QianQinCont > 0)
			{
				QianQinCont--;
				WorkMode = 1;
				i32DoPwmSet(PROPDRIVER_VALVE,DRIVER_OPEN);	
				i32DoPwmSet(QIANQIN_VALVE,DRIVER_OPEN);
				i32DoPwmSet(HOUQIN_VALVE,DRIVER_CLOSE);
			}
		}
		else if((0 == sgHandleInput.b1ZuoLiftUp)&&(1 == sgHandleInput.b1ZuoLiftDown))
		{
			if(HouQinCont > 0)
			{
				HouQinCont--;
				WorkMode = 1;
				i32DoPwmSet(PROPDRIVER_VALVE,DRIVER_OPEN);	
				i32DoPwmSet(HOUQIN_VALVE,DRIVER_OPEN);
				i32DoPwmSet(QIANQIN_VALVE,DRIVER_CLOSE);  
			}
		}
	}
	else
	{
		WorkMode = 0;
		i32DoPwmSet(PROPDRIVER_VALVE,DRIVER_CLOSE);	
		vPropSetTarget(LIFTDOWN_VALVE, 0);
		
		if((((1 == sgHandleInput.b1YouLiftDown)&&(0 == sgHandleInput.b1ZuoLiftDown))
			||((0 == sgHandleInput.b1YouLiftDown)&&(1 == sgHandleInput.b1ZuoLiftDown)))
			&&(0 == sgHandleInput.b1YouLiftUp)
			&&(0 == sgHandleInput.b1ZuoLiftUp)
			&&(0 == (sgu16LimitFlg & LiftDown_Limit)))
		{
			if(1 == sgHandleInput.b1ZuoLiftDown)
			{
				i32PropValue = _IQ((sgUserInfo.fPropMinCurrent + (sgUserInfo.fPropMaxCurrent - sgUserInfo.fPropMinCurrent) * u8PumpOrPropValue / PUMP_RANGE) / PROPD_STD_CURRENT);
			}
			else
			{
				i32PropValue = _IQ((sgUserInfo.fPropMinCurrent + (sgUserInfo.fPropMaxCurrent - sgUserInfo.fPropMinCurrent) * u8PumpOrPropValueMove / PUMP_RANGE) / PROPD_STD_CURRENT);
			}
			if(0 != u8PumpOrPropValueMove)   //左右哪一个触发下降就用哪个
				LiftDownValApp = u8PumpOrPropValueMove;
			else
				LiftDownValApp = u8PumpOrPropValue;
			
			WorkMode = 1;
			vPropSetTarget(LIFTDOWN_VALVE, i32PropValue);
			i32DoPwmSet(PROPDRIVER_VALVE,DRIVER_CLOSE);	
		}
		else if((((1 == sgHandleInput.b1YouLiftUp))
					||((1 == sgHandleInput.b1ZuoLiftUp)))
					&&(0 == sgHandleInput.b1YouLiftDown)
					&&(0 == sgHandleInput.b1ZuoLiftDown)
					&&(0 == (sgu16LimitFlg & (LiftUp_Limit|BMSErrStop)))
					&&(0 == sgSwiInput.b1LiftLimit)
					&&(0 == u8PowerLowLimitL)
		)
		{
			ContinueTim = 0;
//			if(LiftUpCont > 0)
			{
				u8LiftUpStateFlg |= 0x01;          //long
				LiftKeyTim++;	
				if(LiftKeyTim >= sgUserInfo.u16LiftShortTim)
				{
					u8LiftUpStateFlg |= (0x01<<1);          //long
					LiftUpCont--;
					WorkMode = 1;
					i32DoPwmSet(PROPDRIVER_VALVE,DRIVER_OPEN);	
					vPropSetTarget(LIFTDOWN_VALVE, 0);
				}
			}
		}
		else
		{
			if((0 != LiftKeyTim)&&(LiftKeyTim < sgUserInfo.u16LiftShortTim))
			{
				u8LiftUpStateFlg |= (0x01<<2);          //long
				ContinueTim = sgUserInfo.u16LiftShortHold;
			}
			LiftKeyTim = 0;
			
			if((0 != ContinueTim)
			&&(0 == (sgu16LimitFlg & (LiftUp_Limit|BMSErrStop)))
			&&(0 == sgSwiInput.b1LiftLimit)
			&&(0 == u8PowerLowLimitL))
			{
				u8LiftUpStateFlg |= (0x01<<3);          //long
				ContinueTim--;
				WorkMode = 1;
				i32DoPwmSet(PROPDRIVER_VALVE,DRIVER_OPEN);	
				vPropSetTarget(LIFTDOWN_VALVE, 0);
			}
			else
			{
				u8LiftUpStateFlg |= (0x01<<4);          //long
				i32DoPwmSet(PROPDRIVER_VALVE,DRIVER_CLOSE);	
				ContinueTim = 0;
			}
		}
	}
	

	if ((0 == sgSwiInput.b1SafeLock)||(0 == u8HumanLostStop)||(0 != (sgu16LimitFlg &(BmsLost_Limit|StreeLost_Limit|SHOUBinLost_Limit))))
	{
		SendData->b1ServoOn = 0;
		SendData->b1EmsReq = 0;
		SendData->b1ForwardReq = 0;
		SendData->b1BackwardReq = 0;
	}
	{
		static uint8_t EmsLast1 = 0;
		static uint8_t EmsDUAN1 = 0;
		static uint8_t EmsStopMove = 0;
		if((1 == EmsLast1)&&(0 == sgSwiInput.b1Ems)&&(1 == SendData->b1ForwardReq))
		{
			EmsDUAN1 = 1;
			SendData->b1ServoOn = 0;
		}
		
		if((1 == EmsDUAN1)&&(0 == Rev_Speed))
		{
			EmsDUAN1 = 0;
			i32ErrCodeSet(39);
		}
		EmsLast1 = sgSwiInput.b1Ems;
		
		
		static uint8_t MOVEDIRT1 = 0;
		if((1 == sgSwiInput.b1Backward)&&(0 == MOVEDIRT1))
		{
			if(1 == sgSwiInput.b1Ems)
				EmsStopMove = 1;
		}
		MOVEDIRT1 = sgSwiInput.b1Backward;
		if(0 == sgSwiInput.b1Ems)
		{
			EmsStopMove = 0;
		}
		
		if(1 == EmsStopMove)
		{
			SendData->b1ServoOn = 0;
		}
	}
	
	if(0!= (sgu16LimitFlg & APPWRITE_LIMIT))           // 有写命令是 禁止所有动作。
	{
		SendData->b1ServoOn = 0;
		SendData->b1EmsReq = 0;
		SendData->b1ForwardReq = 0;
		SendData->b1BackwardReq = 0;
		i32DoPwmSet(PROPDRIVER_VALVE,DRIVER_CLOSE);	
		i32DoPwmSet(LIFTUP_VALVE,DRIVER_CLOSE);
		i32DoPwmSet(HOUQIN_VALVE,DRIVER_CLOSE);
		i32DoPwmSet(QIANQIN_VALVE,DRIVER_CLOSE);
		vPropSetTarget(LIFTDOWN_VALVE, 0);
	}
	
	WorkCntFlg = 0;
	
	{
		if(((1 == SendData->b1ServoOn)||(1 == WorkMode))&&((2 == sgUserInfo.HourCountMode)||(3 == sgUserInfo.HourCountMode)))
		{
			WorkCntFlg = 1;
			Workcnt++;
		}
		if (Workcnt >= 360*200)			/*360**/
		{
			Workcnt = 0;
			WorkCount8L++;
			if(WorkCount8L > 99)
			{
				WorkCount8L = 0;
				WorkCount16H++;
			}
			u32WorkCount = (WorkCount16H << 8)|WorkCount8L;
			WorkCountL = u32WorkCount;
			WorkCountH = u32WorkCount >> 16;
			u16EepromWrite(PARA_WorkCountL, WorkCountL, 1);
			u16EepromWrite(PARA_WorkCountH, WorkCountH, 1);
		}
	}
		{			
			/*lilu 20230819 For  Test*/
			{
				
				i32SetPara(PARA_ForwardValveCurrent, u8PropTmp);		/*Send Motor Value*/  // 0x400401 0x400402  起升 APP
				i32SetPara(PARA_BackValveCurrent, LiftDownValApp);		/*Rev Motor Value*/	// 0x400403 0x400404  起升 APP
				i32SetPara(PARA_PropValveCurrent, sgHandleInput.u8data);	/*Prop Current*/
				i32SetPara(PARA_EcuLockState, Hour2App);	//上电计时	
				i32SetPara(PARA_TmpLockState, Work2App);	
				i32SetPara(PARA_SelfHeartQuery, HourCntFlg);	
				i32SetPara(PARA_PlatfromHeartQuery, WorkCntFlg);
				i32SetPara(PARA_LiftValveCurrent, u8PumpOrPropValueMove);		/*Send Pump Value*/
				i32SetPara(PARA_OnOffValveCurrent, SendData->buf[2]);			/*Send Status*/
				i32SetPara(PARA_MotorSpd, RevSPEED);				/*Valve NoFlag*/
				i32SetPara(PARA_CalibrationStatus, sgValvesInfo.u8Data[1]);		/*Value State*/
				i32SetPara(PARA_TurnRightValveCurrent, ((uint16_t)sgValvesInfo.u8NoAct << 8) | ((uint16_t)sgValvesInfo.b4NoMove << 4) | ((uint16_t)sgValvesInfo.b4NoLiftUp));
				i32SetPara(PARA_TurnLeftValveCurrent, ((uint16_t)sgValvesInfo.b4Gear1Spd << 12) | ((uint16_t)sgValvesInfo.b4Gear2Spd << 8) | \
					((uint16_t)sgValvesInfo.b4Gear3Spd << 4) | ((uint16_t)sgValvesInfo.b4Gear4Spd));
				i32SetPara(PARA_AngleValue, (uint16_t)(CanRev1E0InfoLast.u8MotorValueH << 8)|(CanRev1E0InfoLast.u8MotorValueL));		/*AI1*/
				i32SetPara(PARA_PressureVlaue1, (CanRev1E0InfoLast.u8LiftValueH << 8)|CanRev1E0InfoLast.u8LiftValueL);	/*AI2*/
				i32SetPara(PARA_PressureVlaue2, CanRev1E0InfoLast.u8QianHouYiH);	/*AI3*/
				i32SetPara(PARA_ExtSignal, sgSwiInput.u16data);						/*Swi*/
				i32SetPara(PARA_PcuKeyInfo,sgHandleInput.u8data);						/*ErrCode*/
				i32SetPara(PARA_BmsSoc, gCanSendPdoInfo.CanSend258Info.SOC);		/*BMS SOC*/
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
		case 0x1E0:
			if(CAN_NORMAL == u8State)
			{
				sgu16LimitFlg &= ~BmsLost_Limit;
				i32ErrCodeClr(ErrCode171);
			}
			else if(CAN_LOST == u8State)
			{
				sgu16LimitFlg |= BmsLost_Limit;
				i32ErrCodeSet(ErrCode171);
			}
			break;
		case 0x360:
			if(CAN_NORMAL == u8State)
			{
				sgu16LimitFlg &= ~StreeLost_Limit;
				i32ErrCodeClr(ErrCode63);
			}
			else if(CAN_LOST == u8State)
			{
				sgu16LimitFlg |= StreeLost_Limit;
				i32ErrCodeSet(ErrCode63);
			}
			break;
		case 0x2F0:
			if(0 == sgUserInfo.u8BatteryType)
			{
				if(CAN_NORMAL == u8State)
				{
					sgu16LimitFlg &= ~SHOUBinLost_Limit;
					i32ErrCodeClr(ErrCode172);
				}
				else if(CAN_LOST == u8State)
				{
					sgu16LimitFlg |= SHOUBinLost_Limit;   // APPWRITE_LIMIT
					i32ErrCodeSet(ErrCode172);
				}
			}
			break;
		default:
			break;
	
	}
}




/*接收PDO处理函数*/
static void vCanRevPdoProc(void)
{
	static uint16_t u16CanRev1ACCnt;
	
	tCanFrame CanSendFrame;
	tCanFrame CanSend0={0};
	uint8_t i = 0;
	static uint16_t cnt = 0;
	
/***** 转向360 *******/
	{
		memcpy((char*)CanRev360InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo360.u8Data, sizeof(CanRev360InfoLast));
		/*添加相关操作*/
		i16SteerAngle = ((CanRev360InfoLast.i16SteerAngleH<<8)|(CanRev360InfoLast.i16SteerAngleL));
	  gCanSendPdoInfo.CanSend260Info.u8ErrorSteer = CanRev360InfoLast.u8ErrSteer;
	  
		if((0 != CanRev360InfoLast.u8ErrSteer)&&(41 != CanRev360InfoLast.u8ErrSteer))
		{
			sgu16LimitFlg |= Stree_Err_Limit;
		}
		else
		{
			sgu16LimitFlg &= ~Stree_Err_Limit;
		}
	}
	
/***** 锂电 2F0 *******/
	{
		static uint16_t u16StopMove = 0;
		static uint16_t u16CutSpeed = 0;
		static uint16_t u16BMSErrCode1 = 0;
		static uint16_t u16BMSErrCode2 = 0;
		
		memcpy((char*)CanRev2F0InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo2F0.u8Data, sizeof(CanRev2F0InfoLast));
		memcpy((char*)CanRev2F1InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo2F1.u8Data, sizeof(CanRev2F1InfoLast));
		/*添加相关操作*/
		if(0 == sgUserInfo.u8BatteryType)
		{
			u16StopMove = (CanRev2F0InfoLast.u8Data[6] & 0x37)
		            | (CanRev2F1InfoLast.u8Data[2] & 0x9F)
                | (CanRev2F1InfoLast.u8Data[5] & 0x4A);		    //Level1 --- Stop Car
		
			u16CutSpeed = (CanRev2F0InfoLast.u8Data[6] & 0x48)      //Level2 --- Cut down Power
									| (CanRev2F1InfoLast.u8Data[2] & 0x60)
									| (CanRev2F1InfoLast.u8Data[5] & 0xB5);
			
			if(0 != u16StopMove)
			{
				sgu16LimitFlg |= BMSErrStop;
			}
			else if(0 != u16CutSpeed)
			{
				sgu16LimitFlg |= BMSErrSpeed_Limit;
			}
			else
			{
				sgu16LimitFlg &= ~BMSErrStop;
				sgu16LimitFlg &= ~BMSErrSpeed_Limit;
			}
			/*BMS故障等级     	 名称     				报文定义	  			 故障码
				二级				锂电池欠压					 0x2F0(6.3)					102 
			  一级 				锂电池严重欠压			 0X2f0(6.1)					103
			  二级				锂电池过热					 0x2F0(6.6)					104
			  一级        锂电池严重过热			 0x2F0(6.5)       	105
				一级        锂电池严重过压      0x2F0(6.0)					106
			  一级        锂电池过流          0x2F0(6.4)					107
			  一级        锂电池低温					 0x2F1(5.3)      		108	
			*/
		if((0 != u16StopMove)||(0 != (u16CutSpeed)))
		{
			if((CanRev2F0InfoLast.u8Data[6] & 0x08) != 0)
			{
				i32ErrCodeSet(161);
			}
			else if((CanRev2F0InfoLast.u8Data[6] & 0x02) != 0)
			{
				i32ErrCodeSet(162);
			}
			else if((CanRev2F0InfoLast.u8Data[6] & 0x40) != 0)
			{
				i32ErrCodeSet(163);
			}
			else if((CanRev2F0InfoLast.u8Data[6] & 0x20) != 0)
			{
				i32ErrCodeSet(164);
			}
			else if((CanRev2F0InfoLast.u8Data[6] & 0x01) != 0)
			{
				i32ErrCodeSet(165);
			}
			else if((CanRev2F0InfoLast.u8Data[6] & 0x10) != 0)
			{
				i32ErrCodeSet(166);
			}
			else if((CanRev2F1InfoLast.u8Data[5] & 0x08) != 0)
			{
				i32ErrCodeSet(167);
			}
			else
			{
				i32ErrCodeSet(169);
			}
		}
		else
		{
				i32ErrCodeClr(161);
				i32ErrCodeClr(162);
				i32ErrCodeClr(163);
				i32ErrCodeClr(164);
				i32ErrCodeClr(165);
				i32ErrCodeClr(166);
				i32ErrCodeClr(167);
				i32ErrCodeClr(169);
		}
	}
}

/***** 手柄 1E0 *******/
	{
		static int16_t i16ADCValue = 0;
		static int16_t i16ZuoLiftValue = 0;
		static int16_t i16YouLiftValue = 0;
		memcpy((char*)CanRev1E0InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo1E0.u8Data, sizeof(CanRev1E0InfoLast));
		/*添加相关操作*/
		sgHandleInput.u8data =	CanRev1E0InfoLast.u8WorkState1;
		i16ADCValue = (CanRev1E0InfoLast.u8MotorValueH << 8)|(CanRev1E0InfoLast.u8MotorValueL);
		if(i16ADCValue < 0)
		{
			sgSwiInput.b1Backward = 1;
			sgSwiInput.b1Forward = 0;
			u16MotorVal = -i16ADCValue;
		}
		else if(i16ADCValue > 0)
		{
			sgSwiInput.b1Backward = 0;
			sgSwiInput.b1Forward = 1;
			u16MotorVal = i16ADCValue;
		}
		else
		{
			sgSwiInput.b1Backward = 0;
			sgSwiInput.b1Forward = 0;
			u16MotorVal = 0;
		}
		
		i16ZuoLiftValue = (CanRev1E0InfoLast.u8LiftValueH << 8)|CanRev1E0InfoLast.u8LiftValueL;
		if(i16ZuoLiftValue > 0)
		{
			u8PumpOrPropValue = 255*i16ZuoLiftValue/240;
		}
		else if(i16ZuoLiftValue < 0)
		{
			i16ZuoLiftValue = -i16ZuoLiftValue;
			u8PumpOrPropValue = 255*i16ZuoLiftValue/4096;
		}
		else
		{
			u8PumpOrPropValue = 0;
		}
		
		i16YouLiftValue = (CanRev1E0InfoLast.u8QianHouYiH << 8)|CanRev1E0InfoLast.u8QianHouYiL;
		if(i16YouLiftValue > 0)
		{
			u8PumpOrPropValueMove = 255*i16YouLiftValue/240;
		}
		else if(i16YouLiftValue < 0)
		{
			i16YouLiftValue = -i16YouLiftValue;
			u8PumpOrPropValueMove = 255*i16YouLiftValue/4096;
		}
		else
		{
			u8PumpOrPropValueMove = 0;
		}
		
		{
			static uint8_t EmsErrtmp = 0;
			if((1 == sgSwiInput.b1Ems)&&(0 == EmsErrtmp))
			{
				if(1 == sgSwiInput.b1Backward)
				{
					sgu16LimitFlg |= EmsErrFlg;
					i32ErrCodeSet(100);
				}
				else
				{
					if(0 != (sgu16LimitFlg&EmsErrFlg))
					{
						sgu16LimitFlg &= ~EmsErrFlg;
						i32ErrCodeClr(100);
					}
				}
			}
			if(1 == sgSwiInput.b1Backward)
			{
				EmsErrtmp = 1;
			}
			else
			{
				if(1 == EmsErrtmp)
				{
					sgu16LimitFlg &= ~EmsErrFlg;
					i32ErrCodeClr(100);
					EmsErrtmp = 0;
				}
			}
		}
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
	
//	u16MotorVal = u16SteerAngleDecSpd(u16MotorVal, abs(i16SteerAngle), &sgUserInfo.SteerAngleDecSpd);
}


/*开关量监控*/
static void vSwiMonitor(void)
{
	xSwiInput SwiInput = {0};
	static uint8_t Snail_Count = 0;
	static uint16_t PedalCount = 0;
	
	if((1 == i32LocalDiGet(HEIGHT500_SPEEDLIMIT_SWI))&&(0 == (sgUserInfo.u16RentalTime&0x01)))
	{
		SwiInput.b1Height500Spdlimit = 1;
	}
	
	if(1 == i32LocalDiGet(EmergencyReverse))
	{
		SwiInput.b1Ems = 1;
	}
	
	if(1 == i32LocalDiGet(SAFELOCK_SWI))
	{
		SwiInput.b1SafeLock = 1;
	}
	
	if(1 == i32LocalDiGet(Pedal_SWI))
	{
		SwiInput.b1Pedal = 1;
	}
	
	if(1 == i32LocalDiGet(StreeLock_SWI))
	{
		SwiInput.b1StreeLock = 1;
	}
	
	if((0 == i32LocalDiGet(Human_SWI))&&(0 == i32LocalDiGet(Pedal_SWI)))
	{
		u8HumanLostStop = 0;
	}
	else
	{
		u8HumanLostStop = 1;
	}
	
	if((1 == i32LocalDiGet(Human_SWI))||(PedalCount < 200))
	{
		SwiInput.b1Human = 1;
		if(0 == i32LocalDiGet(Human_SWI)&&(0 != u16MotorVal))
		{
			PedalCount++;
		}
		else if(0 == i32LocalDiGet(Human_SWI)&&(0 == u16MotorVal))
		{
			PedalCount = 255;
		}
	}
	else
	{
		sgu16LimitFlg |= PedalMove_Limit;
	}
	if(0 == i32LocalDiGet(Human_SWI)&&(1 == i32LocalDiGet(Pedal_SWI)))
	{
		sgu16LimitFlg &= ~PedalMove_Limit;
	}

		 
	if((0 == sgSwiInput.b1Backward)&&(0 == sgSwiInput.b1Forward))
	{
		sgu16LimitFlg &= ~PedalMove_Limit;
		if(1 == i32LocalDiGet(Human_SWI))
		{
			PedalCount = 0;			
		}
	}
	
	if((1 == i32LocalDiGet(HEIGHT1800_SPEEDLIMIT_SWI))&&(0 == (sgUserInfo.u16RentalTime&0x01)))
	{
		SwiInput.b1Height1800Spdlimit = 1;
	}
	
	if(1 == i32LocalDiGet(FENCE1_SWI))
	{
		SwiInput.b1FENCE1 = 1;
	}
	
	if(1 == i32LocalDiGet(FENCE2_SWI))
	{
		SwiInput.b1FENCE2 = 1;
	}
	
	if(1 == i32LocalDiGet(LIFTLIMIT_SWI))
	{
		SwiInput.b1LiftLimit = 1;
	}
	sgSwiInput.u16data = SwiInput.u16data;
	
	if((0 == SwiInput.b1Human)&&(0 == SwiInput.b1Pedal)&&(0 == Rev_Speed))
	{
		sgu16LimitFlg |=(Move_Limit|LiftUp_Limit|LiftDown_Limit);
	}
	else if((1 == SwiInput.b1Human)&&(0 == SwiInput.b1Pedal)&&(1 == SwiInput.b1FENCE1)&&(1 == SwiInput.b1Height1800Spdlimit)&&(1 == SwiInput.b1Height500Spdlimit))
	{
		sgu16LimitFlg &= ~(Move_Limit|LiftUp_Limit|LiftDown_Limit);
		sgValvesInfo.b1Gear1SpdFlag = 0;   // 高速
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 0;
		sgValvesInfo.b1Gear4SpdFlag = 0;
	}
	else if((1 == SwiInput.b1Human)&&(0 == SwiInput.b1Pedal)&&(1 == SwiInput.b1FENCE1)&&(1 == SwiInput.b1Height1800Spdlimit)&&(0 == SwiInput.b1Height500Spdlimit))
	{
		sgu16LimitFlg &= ~(Move_Limit|LiftUp_Limit|LiftDown_Limit);
		sgValvesInfo.b1Gear1SpdFlag = 0;   // 低速3
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 1;
		sgValvesInfo.b1Gear4SpdFlag = 0;
	}
	else if((1 == SwiInput.b1Human)&&(0 == SwiInput.b1Pedal)&&(1 == SwiInput.b1FENCE1)&&(0 == SwiInput.b1Height1800Spdlimit))
	{
		sgu16LimitFlg |= LiftUp_Limit;
		sgu16LimitFlg &= ~(Move_Limit|LiftDown_Limit);
		sgValvesInfo.b1Gear1SpdFlag = 0;   // 低速4
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 0;
		sgValvesInfo.b1Gear4SpdFlag = 1;
	}
	else if((1 == SwiInput.b1Human)&&(0 == SwiInput.b1Pedal)&&(0 == SwiInput.b1FENCE1))
	{
		if((1 == SwiInput.b1Height1800Spdlimit)&&(1 == SwiInput.b1Height500Spdlimit))
		{
			sgu16LimitFlg &= ~(Move_Limit|LiftUp_Limit|LiftDown_Limit);
			sgValvesInfo.b1Gear1SpdFlag = 1;   // 低速1
			sgValvesInfo.b1Gear2SpdFlag = 0;
			sgValvesInfo.b1Gear3SpdFlag = 0;
			sgValvesInfo.b1Gear4SpdFlag = 0;
		}
		else if((1 == SwiInput.b1Height1800Spdlimit)&&(0 == SwiInput.b1Height500Spdlimit))
		{
			sgu16LimitFlg &= ~(Move_Limit|LiftUp_Limit|LiftDown_Limit);
			sgValvesInfo.b1Gear1SpdFlag = 0;   // 低速3
			sgValvesInfo.b1Gear2SpdFlag = 0;
			sgValvesInfo.b1Gear3SpdFlag = 1;
			sgValvesInfo.b1Gear4SpdFlag = 0;
		}
		else if(0 == SwiInput.b1Height1800Spdlimit)
		{
			sgu16LimitFlg &= ~(Move_Limit|LiftUp_Limit|LiftDown_Limit);
			sgValvesInfo.b1Gear1SpdFlag = 0;   // 低速4
			sgValvesInfo.b1Gear2SpdFlag = 0;
			sgValvesInfo.b1Gear3SpdFlag = 0;
			sgValvesInfo.b1Gear4SpdFlag = 1;
		}
	}
	else if((1 == SwiInput.b1Pedal)&&(1 == SwiInput.b1FENCE1))
	{
		sgu16LimitFlg |=(Move_Limit|LiftUp_Limit);
		sgu16LimitFlg &= ~LiftDown_Limit;
	}
	else if((1 == SwiInput.b1Pedal)&&(0 == SwiInput.b1FENCE1))
	{
		if((1 == SwiInput.b1Height1800Spdlimit)&&(1 == SwiInput.b1Height500Spdlimit))
		{
			sgu16LimitFlg &= ~(Move_Limit|LiftUp_Limit|LiftDown_Limit);
			sgValvesInfo.b1Gear1SpdFlag = 0;   // 低速2
			sgValvesInfo.b1Gear2SpdFlag = 1;
			sgValvesInfo.b1Gear3SpdFlag = 0;
			sgValvesInfo.b1Gear4SpdFlag = 0;
		}
		else if((1 == SwiInput.b1Height1800Spdlimit)&&(0 == SwiInput.b1Height500Spdlimit))
		{
			sgu16LimitFlg &= ~(Move_Limit|LiftUp_Limit|LiftDown_Limit);
			sgValvesInfo.b1Gear1SpdFlag = 0;   // 低速3
			sgValvesInfo.b1Gear2SpdFlag = 0;
			sgValvesInfo.b1Gear3SpdFlag = 1;
			sgValvesInfo.b1Gear4SpdFlag = 0;
		}
		else if(0 == SwiInput.b1Height1800Spdlimit)
		{
			sgu16LimitFlg &= ~(Move_Limit|LiftUp_Limit|LiftDown_Limit);
			sgValvesInfo.b1Gear1SpdFlag = 0;   // 低速4
			sgValvesInfo.b1Gear2SpdFlag = 0;
			sgValvesInfo.b1Gear3SpdFlag = 0;
			sgValvesInfo.b1Gear4SpdFlag = 1;
		}
	}
	
	if((1 == sgHandleInput.b1SlowMode)||(1 == u8PowerLowLimitS))
	{
		sgValvesInfo.b1Gear1SpdFlag = 0;   //低速4
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 0;
		sgValvesInfo.b1Gear4SpdFlag = 1;
	}
	{
		static uint8_t hunmanspeedtmp = 0;
		if((1 == SwiInput.b1Human)&&(0 == SwiInput.b1Pedal)&&(0 == SwiInput.b1Height1800Spdlimit)&&(1 == SwiInput.b1Height500Spdlimit))
		{
			hunmanspeedtmp = 1;
			SpeedRate = 23;   // 1KM/H
			i32ErrCodeSet(109);
		}
		else
		{
			if(hunmanspeedtmp == 1)
			{
				hunmanspeedtmp = 0;
				SpeedRate = 0;
				i32ErrCodeClr(109);
			}
		}
	}
	
	if((1 == SwiInput.b1Ems)&&(0 == u16MotorVal)&&(0 != Rev_Speed))
	{
		i32ErrCodeClr(39);
	}
	{
		static uint8_t LockFlg = 0;
		static uint8_t LockErr = 0;
		if((0 != u16MotorVal)&&(0 == LockFlg)&&(0 == Rev_Speed))
		{
			if(1 == SwiInput.b1SafeLock)
			{
				LockErr = 1;
				i32ErrCodeSet(39);
				sgu16LimitFlg |= EmsLimitMove;
			}
			else
			{
				if(1 == LockErr)
				{
					LockErr = 0;
					sgu16LimitFlg &= ~EmsLimitMove;
					i32ErrCodeClr(39);
				}
			}
		}
		else if(1 == SwiInput.b1SafeLock)
		{
			LockFlg = 1;
			if(1 == LockErr)
			{
				LockErr = 0;
				sgu16LimitFlg &= ~EmsLimitMove;
				i32ErrCodeClr(39);
			}
		}
		else if(0 == SwiInput.b1SafeLock)
		{
			LockFlg = 0;
			if(1 == LockErr)
			{
				LockErr = 0;
				sgu16LimitFlg &= ~EmsLimitMove;
				i32ErrCodeClr(39);
			}
		}
	}
}

static void vCanSendid2F8Tmp(void)
{
	static uint8_t u8Cnt = 0;
	
	tCanFrame Can2F8Send = {.u32ID = 0x2F8, .u16DataLength = 8, .u8Data = {0x03,u32HourCount,u32HourCount>>8,u32HourCount>>16}, .u8Rtr = 0};
	if (u8Cnt++ >= 11)
	{
		u8Cnt = 0;
		i32CanWrite(Can0, &Can2F8Send);
	}
	
}
static void vCanSendid258Tmp(void)
{
	static uint8_t basic = 0x10;
	static uint8_t cntTmp = 0;
	static uint8_t ErrCodeSendCnt = 0;
	switch (basic)
	{
		case 0x10:
			gCanSendPdoInfo.CanSend258Info.Basic = 0x10;
			gCanSendPdoInfo.CanSend258Info.SOC_Choose = 1;
			gCanSendPdoInfo.CanSend258Info.Time_Choose = 1;
			gCanSendPdoInfo.CanSend258Info.PowerOn_Count = HourCntFlg;
			gCanSendPdoInfo.CanSend258Info.Work_Count = WorkCntFlg;
			gCanSendPdoInfo.CanSend258Info.SpeedOn = 1;
			gCanSendPdoInfo.CanSend258Info.SpeedUint = sgUserInfo.u8Language;
			if(0 == sgUserInfo.u8BatteryType)
			{
				gCanSendPdoInfo.CanSend258Info.SOC = (CanRev2F0InfoLast.BMS_SOC)*0.4;
			}
			else
			{
				gCanSendPdoInfo.CanSend258Info.SOC = u8GetBatterySoc();
			}
			
			if(0 != Rev_Speed)
			{
				gCanSendPdoInfo.CanSend258Info.Park = 0;
				if(0 != sgUserInfo.u8Language)
				{
					gCanSendPdoInfo.CanSend258Info.Speed = RevSPEED*0.6213;
				}
				else
				{
					gCanSendPdoInfo.CanSend258Info.Speed = RevSPEED;				
				}
			}
			else
			{
				gCanSendPdoInfo.CanSend258Info.Park = 1;
			}
			if(0 == sgUserInfo.u8BatteryType)
			{
				if(((CanRev2F0InfoLast.BMS_SOC)*4/10) < sgUserInfo.u8SOCLimit)
				{
					u8PowerLowLimitL = 1;
					u8PowerLowLimitS = 1;
					gCanSendPdoInfo.CanSend258Info.SOC_Warn = 1;
				}
				else if(((CanRev2F0InfoLast.BMS_SOC)*4/10) < (5 + sgUserInfo.u8SOCLimit))
				{
					u8PowerLowLimitS = 1;
					gCanSendPdoInfo.CanSend258Info.SOC_Warn = 1;
				}
				else
				{
					u8PowerLowLimitL = 0;
					u8PowerLowLimitS = 0;
					gCanSendPdoInfo.CanSend258Info.SOC_Warn = 0;
				}
			}
			else
			{
				if(u8GetBatterySoc() < sgUserInfo.u8SOCLimit)
				{
					u8PowerLowLimitL = 1;
					u8PowerLowLimitS = 1;
					gCanSendPdoInfo.CanSend258Info.SOC_Warn = 1;
				}
				else if(u8GetBatterySoc() < (sgUserInfo.u8SOCLimit+5))
				{
					u8PowerLowLimitS = 1;
					gCanSendPdoInfo.CanSend258Info.SOC_Warn = 1;
				}
				else
				{
					u8PowerLowLimitL = 0;
					u8PowerLowLimitS = 0;
					gCanSendPdoInfo.CanSend258Info.SOC_Warn = 0;
				}
			}
			if(1 == sgHandleInput.b1SlowMode)
			{
				gCanSendPdoInfo.CanSend258Info.Work_Mode = 2;
			}
			else
			{
				gCanSendPdoInfo.CanSend258Info.Work_Mode = 0;
			}

			gCanSendPdoInfo.CanSend258Info.Contorl = 1;
			gCanSendPdoInfo.CanSend258Info.ErrCode = 0;
			if (0 != u8ErrCodeGet())
			{
				gCanSendPdoInfo.CanSend258Info.Contorl = 1;
				for(;ErrCodeSendCnt <= ErrCode200; ErrCodeSendCnt++)
				{
					if(0 != u8ErrCodeCir[ErrCodeSendCnt])
					{
						gCanSendPdoInfo.CanSend258Info.ErrCode = u8ErrSwitchArray[u8ErrCodeCir[ErrCodeSendCnt]];
						break;
					}
				}
			}
			
			if((0 != CanRev360InfoLast.u8ErrSteer)&&(ErrCodeSendCnt > ErrCode200))
			{      
  			ErrCodeSendCnt = 0;
				gCanSendPdoInfo.CanSend258Info.Contorl = 5;
				gCanSendPdoInfo.CanSend258Info.ErrCode = CanRev360InfoLast.u8ErrSteer;
			}
			
			if((1 == (sgu16LimitFlg & (LiftUp_Limit|BMSLiftUp_Limit))))
			{
				gCanSendPdoInfo.CanSend258Info.LiftUpLimit = 1;
			}
			else
			{
				gCanSendPdoInfo.CanSend258Info.LiftUpLimit = 0;
			}
			
			break; 
			
			case 0x21:
			gCanSendPdoInfo.CanSend258Info.u8Data[0] = 0x21;
			gCanSendPdoInfo.CanSend258Info.u8Data[1] = u32WorkCount & 0xFF;
			gCanSendPdoInfo.CanSend258Info.u8Data[2] = (u32WorkCount >> 8) & 0xFF;
			gCanSendPdoInfo.CanSend258Info.u8Data[3] = (u32WorkCount >> 16) & 0xFF;
			basic = 0x10;
			break;	
	}
	
	if(cntTmp < 10)
	{
		cntTmp++;
		basic = 0x21;
	}
	else if(cntTmp < 20)
	{
		if(ErrCodeSendCnt <= ErrCode200)
			ErrCodeSendCnt++;
		else
			ErrCodeSendCnt = 0;
		cntTmp++;
		basic = 0x10;
	}
	else
	{
		if(ErrCodeSendCnt <= ErrCode200)
			ErrCodeSendCnt++;
		cntTmp = 0;
	}
}

const wchar_t ErrorUnicode[][10] ={ {0x53cd, 0x9988, 0x8d85, 0x901f},  													 /*** 1 反馈超速***/ 
																		{0x5185, 0x6838, 0x8fd0, 0x884c, 0x9519, 0x8bef},  					 /*** 2 内核运行错误***/
																		{0x957f, 0x65f6, 0x95f4, 0x8fc7, 0x8f7d},  											 /*** 3 长时间过载***/
																		{0x0000, 0x0000},  
																		{0x4f4d, 0x7f6e, 0x8d85, 0x9650},  																					/*** 5 位置超限***/
																		{0x901f, 0x5ea6, 0x8d85, 0x9650},   																				/*** 6 速度超限***/
																		{0x8f6c, 0x77e9, 0x8d85, 0x9650},  																					/*** 7 转矩超限***/
																		{0x901f, 0x5ea6, 0x4f20, 0x611f, 0x5668, 0x4e22, 0x5931},   								/*** 8 速度传感器丢失***/													
																		{0x901f, 0x5ea6, 0x4f20, 0x611f, 0x5668, 0x65b9, 0x5411, 0x9519, 0x8bef},   /*** 9 速度传感器方向错误***/
																		{0x0000},
																		{0x4e24, 0x5206, 0x949f, 0x7535, 0x6d41, 0x4fdd ,0x62a4},   								/*** 11 两分钟电流保护***/
																		{0x63a7, 0x5236, 0x5668, 0x8fc7, 0x6d41},   																/*** 12 控制器过流***/
																		{0x6bcd, 0x7ebf, 0x7535, 0x5bb9, 0x5145, 0x7535, 0x6545, 0x969c},   				/*** 13 母线电容充电故障***/
																		{0x4e3b, 0x63a5, 0x89e6, 0x5668, 0x8fde, 0x63a5, 0x6545, 0x969c},   				/*** 14 主接触器连接故障***/
																		{0x7535, 0x78c1, 0x5236, 0x52a8, 0x8fde, 0x63a5, 0x6545, 0x969c},   				/*** 15 电磁制动连接故障***/
																		{0x7535, 0x6c60, 0x7535, 0x538b, 0x4e25, 0x91cd, 0x8fc7, 0x4f4e},   				/*** 16 电池电压严重过低***/
																		{0x7535, 0x6c60, 0x7535, 0x538b, 0x4e25, 0x91cd, 0x8fc7, 0x9ad8},  					/*** 17 电池电压严重过高***/
																		{0x529f, 0x7387, 0x677f, 0x4e25, 0x91cd, 0x8fc7, 0x6e29},  									/*** 18 功率板严重过温***/
																		{0x7535, 0x673a, 0x4e25, 0x91cd, 0x9ad8, 0x6e29},   												/*** 19 电机严重高温***/
																		{0x8e0f, 0x677f, 0x8f93, 0x5165, 0x5f02, 0x5e38},   												/*** 20 踏板输入异常***/
																		{0x4e3b, 0x63a5, 0x89e6, 0x5668, 0x89e6, 0x70b9, 0x7194, 0x63a5},   				/*** 21 主接触器触点熔接***/
																		{0x8f93, 0x51fa, 0x4e94, 0x4f0f, 0x6545, 0x969c},  								  				/*** 22 输出五伏故障***/
																		{0x8282, 0x70b9, 0x68c0, 0x6d4b, 0x5931, 0x8d25},   												/*** 23 节点检测失败***/
																		{0x4e3b, 0x63a5, 0x89e6, 0x5668, 0x9a71, 0x52a8, 0x6545, 0x969c},  					/*** 24 主接触器驱动故障***/
																		{0x529f, 0x7387, 0x6a21, 0x5757, 0x6545, 0x969c},   												/*** 25 功率模块故障***/
																		{0x8282, 0x70b9, 0x4e22, 0x5931, 0x901a, 0x4fe1, 0x6545, 0x969c},  					/*** 26 节点丢失通信故障***/
																		{0x7535, 0x538b, 0x9ad8, 0x4e8e, 0x6700, 0x5927, 0x7535, 0x538b, 0x4e24, 0x4f0f},   /*** 27 电压高于最大电压两伏***/
																		{0x0000},  
																		{0x7535, 0x673a, 0x7535, 0x963b, 0x5f02, 0x5e38},   												/*** 29 电机电阻异常***/
																		{0x56de, 0x4e2d, 0x9519, 0x8bef},  																					/*** 30 回中错误***/
																		{0x7535, 0x6c60, 0x7535, 0x538b, 0x8f7b, 0x5ea6, 0x8fc7, 0x4f4e},   				/*** 31 电池电压轻度过低***/
																		{0x529f, 0x7387, 0x677f, 0x8f7b, 0x5ea6, 0x8fc7, 0x6e29},   								/*** 32 功率板轻度过温***/	
																		{0x529f, 0x7387, 0x677f, 0x4f4e, 0x6e29},   																/*** 33 功率板低温***/	
																		{0x529f, 0x7387, 0x677f, 0x4f4e, 0x6e29},   																/*** 34 电机轻度过温***/	
																		{0x5341, 0x4e8c, 0x4f0f, 0x8f93, 0x51fa, 0x6545, 0x969c},  									/*** 35 十二伏输出故障***/	
																		{0x9a71, 0x52a8, 0x4e09, 0x8fde, 0x63a5, 0x6545, 0x969c},   								/*** 36 驱动三连接故障***/	
																		{0x9a71, 0x52a8, 0x56db, 0x8fde, 0x63a5, 0x6545, 0x969c},   								/*** 37 驱动四连接故障***/	
																		{0x8bfb, 0x5199, 0x53c2, 0x6570, 0x9519, 0x8bef},   												/*** 38 读写参数错误***/	
																		{0x53c2, 0x6570, 0x8d85, 0x9650, 0x9519, 0x8bef},  	 												/*** 39 参数超限错误***/	
																		{0x64cd, 0x4f5c, 0x65f6, 0x5e8f, 0x9519, 0x8bef},  													/*** 40 操作时序错误***/
																		{0x7535, 0x91cf, 0x4e25, 0x91cd, 0x8fc7, 0x4f4e, 0x8b66, 0x544a},  					/*** 41 电量严重过低警告***/
																		{0x7535, 0x91cf, 0x4e25, 0x91cd, 0x8fc7, 0x4f4e},  													/*** 42 电量严重过低***/
																		{0x0000},{0x0000},{0x0000},{0x0000},{0x0000},{0x0000},{0x0000},{0x0000},     /***43-50***/
																		{0x9a71, 0x52a8, 0x4e00, 0x8fde, 0x63a5, 0x6545,0x969c},									 /*51 驱动一连接故障*/
																		{0x9a71, 0x52a8, 0x4e8c, 0x8fde, 0x63a5, 0x6545,0x969c},
																		{0x9a71, 0x52a8, 0x4e09, 0x8fde, 0x63a5, 0x6545,0x969c},
																		{0x9a71, 0x52a8, 0x56db, 0x8fde, 0x63a5, 0x6545,0x969c},
																		{0x9a71, 0x52a8, 0x4e94, 0x8fde, 0x63a5, 0x6545,0x969c},
																		{0x9a71, 0x52a8, 0x516d, 0x8fde, 0x63a5, 0x6545,0x969c},
																		{0x9a71, 0x52a8, 0x4e03, 0x8fde, 0x63a5, 0x6545,0x969c},
																		{0x9a71, 0x52a8, 0x516b, 0x8fde, 0x63a5, 0x6545,0x969c},     
																		{0x9a71, 0x52a8, 0x4e5d, 0x8fde, 0x63a5, 0x6545,0x969c},                     /*59 驱动九连接故障*/
																		{0x0000},{0x0000},{0x0000},
																		{0x8282, 0x70b9, 0x4e22, 0x5931, 0x901a, 0x4fe1, 0x6545, 0x969c},
																		{0x0000},{0x0000},
																		{0x6bd4, 0x4f8b, 0x9600, 0x4e00, 0x6545, 0x969c},	  												/*66 比例阀1故障*/
																		{0x6bd4, 0x4f8b, 0x9600, 0x4e8c, 0x6545, 0x969c},														/*67 比例阀2故障 */
																		{0x0000},{0x0000},{0x0000},
																		{0x6a21, 0x62df, 0x91cf, 0x4e00, 0x6545, 0x969c},                           /*71 模拟量1故障*/
																		{0x6a21, 0x62df, 0x91cf, 0x4e8c, 0x6545, 0x969c},
																		{0x6a21, 0x62df, 0x91cf, 0x4e09, 0x6545, 0x969c},
																		{0x6a21, 0x62df, 0x91cf, 0x56db, 0x6545, 0x969c},		 												/*74 模拟量4故障*/
																		{0x0000},{0x0000},
																		{0x7f16, 0x7801, 0x5668, 0x4e00, 0x6545, 0x969c},
																		{0x7f16, 0x7801, 0x5668, 0x4e8c, 0x6545, 0x969c},		 												/*78 编码器2故障*/
																		{0x0000},{0x0000},											/***75-80***/
																		{0x9ad8, 0x5ea6, 0x9650, 0x901f, 0x5f02, 0x5e38},  													/*** 81 高度限速异常***/																	
																		{0x672a, 0x5b9a, 0x4e49, 0x63a7, 0x5236, 0x5668},  													/*** 95 未定义控制器***/																				
};
const char ErrorAscii[][28] = { {0x46,0x65,0x65,0x64,0x62,0x61,0x63,0x6B,0x20,0x4F,0x76,0x65,0x72,0x73,0x70,0x65,0x65,0x64},  																							 			/*** 1 Feedback Overspeed***/ 
																{0x4B,0x65,0x72,0x6E,0x65,0x6C,0x20,0x6F,0x70,0x65,0x72,0x61,0x74,0x69,0x6F,0x6E,0x20,0x65,0x72,0x72,0x6F,0x72},  													 			/*** 2 Kernel operation error***/ 
																{0x50,0x72,0x6F,0x6C,0x6F,0x6E,0x67,0x65,0x64,0x20,0x6F,0x76,0x65,0x72,0x6C,0x6F,0x61,0x64},  													 													/*** 3 Prolonged overload***/ 
																{0x00},  																									 																																												/*** 4 NONE***/ 
																{0x50,0x6f,0x73,0x69,0x74,0x69,0x6f,0x6e,0x20,0x6f,0x76,0x65,0x72,0x72,0x75,0x6e},  								 																							/*** 5 Position overrun***/ 
																{0x53,0x70,0x65,0x65,0x64,0x20,0x6f,0x76,0x65,0x72,0x72,0x75,0x6e}, 																																							/*** 6 Speed overrun***/ 
																{0x54,0x6f,0x72,0x71,0x75,0x65,0x20,0x6f,0x76,0x65,0x72,0x72,0x75,0x6e},  																																				/*** 7 Torque overrun***/ 
																{0x53,0x70,0x65,0x65,0x64,0x20,0x73,0x65,0x6E,0x73,0x6F,0x72,0x20,0x6C,0x6F,0x73,0x73},  													 																/*** 8 Speed sensor loss***/ 
																{0x53,0x70,0x64,0x20,0x73,0x65,0x6E,0x73,0x6F,0x72,0x20,0x6D,0x69,0x73,0x64,0x69,0x72,0x65,0x63,0x74,0x65,0x64},  														 	  /*** 9 Spd sensor misdirected***/ 
																{0x00},  																																																																					/*** 10 None***/ 
																{0x32,0x6D,0x69,0x6E,0x20,0x63,0x75,0x72,0x72,0x65,0x6E,0x74,0x20,0x70,0x72,0x6F,0x74,0x65,0x63,0x74,0x73},  																			/*** 11 2min current protects***/ 
																{0x43,0x6F,0x6E,0x74,0x72,0x6F,0x6C,0x6C,0x65,0x72,0x20,0x6F,0x76,0x65,0x72,0x63,0x75,0x72,0x72,0x65,0x6E,0x74},  													 			/*** 12 Controller overcurrent***/ 
																{0x42,0x75,0x73,0x20,0x63,0x61,0x70,0x61,0x63,0x69,0x74,0x6F,0x72,0x20,0x63,0x68,0x61,0x72,0x67,0x69,0x6E,0x67,0x20,0x65,0x72,0x72},  						/*** 13 Bus capacitor charging err***/ 
																{0x4D,0x61,0x69,0x6E,0x20,0x63,0x6F,0x6E,0x74,0x61,0x63,0x74,0x6F,0x72,0x20,0x63,0x6F,0x6E,0x6E,0x65,0x63,0x74,0x73,0x20,0x65,0x72,0x72},  				/*** 14 Main contactor connects err***/ 
																{0x42,0x72,0x61,0x6B,0x65,0x20,0x63,0x6F,0x6E,0x6E,0x65,0x63,0x74,0x69,0x6F,0x6E,0x20,0x66,0x61,0x69,0x6C,0x75,0x72,0x65},  										  /*** 15 Brake connection failure***/ 
																{0x53,0x65,0x76,0x65,0x72,0x65,0x20,0x6C,0x6F,0x77,0x20,0x62,0x61,0x74,0x74,0x65,0x72,0x79,0x20,0x76,0x6F,0x6C,0x74,0x61,0x67,0x65},  						/*** 16 Severe low battery voltage***/ 
																{0x48,0x69,0x67,0x68,0x20,0x62,0x61,0x74,0x74,0x65,0x72,0x79,0x20,0x76,0x6F,0x6C,0x74,0x61,0x67,0x65},  													 								/*** 17 High battery voltage***/ 
																{0x50,0x6F,0x77,0x65,0x72,0x42,0x6F,0x61,0x72,0x64,0x20,0x6F,0x76,0x65,0x72,0x74,0x6D,0x70,0x20,0x73,0x65,0x72,0x69,0x6F,0x75,0x73,0x6C,0x79},    /*** 18 PowerBoard overtmp seriously***/ 
																{0x53,0x65,0x76,0x65,0x72,0x65,0x20,0x6D,0x6F,0x74,0x6F,0x72,0x20,0x68,0x65,0x61,0x74},  													 																/*** 19 Severe motor heat***/ 
																{0x50,0x65,0x64,0x61,0x6C,0x20,0x69,0x6E,0x70,0x75,0x74,0x20,0x61,0x62,0x6E,0x6F,0x72,0x6D,0x61,0x6C,0x6C,0x79},  													 			/*** 20 Pedal input abnormally***/ 
																{0x43,0x6F,0x6E,0x74,0x61,0x63,0x74,0x6F,0x72,0x20,0x63,0x6F,0x6E,0x74,0x61,0x63,0x74,0x73,0x20,0x66,0x75,0x73,0x65,0x64},  											/*** 21 Contactor contacts fused***/ 
																{0x35,0x56,0x20,0x6F,0x75,0x74,0x70,0x75,0x74,0x20,0x66,0x61,0x75,0x6C,0x74},  													 																					/*** 22 5V output fault***/ 
																{0x4D,0x41,0x43,0x49,0x44,0x20,0x64,0x65,0x74,0x65,0x63,0x74,0x69,0x6F,0x6E,0x20,0x66,0x61,0x69,0x6C,0x75,0x72,0x65},  													  /*** 23 MACID detection failure***/ 
																{0x4D,0x61,0x69,0x6E,0x20,0x63,0x6F,0x6E,0x74,0x61,0x63,0x74,0x6F,0x72,0x20,0x64,0x72,0x69,0x76,0x65,0x20,0x66,0x61,0x75,0x6C,0x74},  						/*** 24 Main contactor drive fault***/ 
																{0x50,0x6F,0x77,0x65,0x72,0x20,0x4D,0x6F,0x64,0x75,0x6C,0x65,0x20,0x46,0x61,0x69,0x6C,0x75,0x72,0x65},  																					/*** 25 Power Module Failure***/ 
																{0x43,0x41,0x4E,0x20,0x43,0x6F,0x6D,0x6D,0x75,0x6E,0x69,0x63,0x61,0x74,0x69,0x6F,0x6E,0x20,0x65,0x72,0x72,0x6F,0x72,0x73},  											/*** 26 CAN Communication errors***/ 
																{0x56,0x6F,0x6C,0x74,0x61,0x67,0x65,0x20,0x65,0x78,0x63,0x65,0x65,0x64,0x20,0x74,0x68,0x65,0x20,0x4D,0x61,0x78,0x20,0x62,0x79,0x20,0x32,0x56},  	/*** 27 Voltage exceed the Max by 2V***/ 
																{0x00},  																																																																				  /*** 28 NONE***/ 
																{0x4D,0x6F,0x74,0x6F,0x72,0x20,0x54,0x65,0x6D,0x70,0x20,0x53,0x65,0x6E,0x73,0x6F,0x72,0x20,0x66,0x61,0x69,0x6C},  													 			/*** 29 Motor Temp Sensor fail***/ 
																{0x52,0x65,0x74,0x75,0x72,0x6E,0x20,0x45,0x72,0x72,0x6F,0x72},  													 																												/*** 30 Return Error***/ 
																{0x42,0x61,0x74,0x74,0x20,0x76,0x6F,0x6C,0x74,0x61,0x67,0x65,0x20,0x69,0x73,0x20,0x73,0x6C,0x69,0x67,0x68,0x74,0x6C,0x79,0x20,0x6C,0x6F,0x77},  	/*** 31 Batt voltage is slightly low***/ 
																{0x4D,0x69,0x6C,0x64,0x20,0x6F,0x76,0x65,0x72,0x74,0x65,0x6D,0x70,0x20,0x6F,0x66,0x20,0x70,0x6F,0x77,0x65,0x72,0x20,0x62,0x6F,0x61,0x72,0x64},  	/*** 32 Mild overtemp of power board***/ 
																{0x50,0x6F,0x77,0x65,0x72,0x20,0x62,0x6F,0x61,0x72,0x64,0x20,0x63,0x72,0x79,0x6F,0x67,0x65,0x6E,0x69,0x63,0x73},  													 			/*** 33 Power board cryogenics***/ 
																{0x4D,0x6F,0x74,0x6F,0x72,0x20,0x6D,0x69,0x6C,0x64,0x6C,0x79,0x20,0x68,0x6F,0x74},  													 																		/*** 34 Motor mildly hot***/ 
																{0x31,0x32,0x56,0x20,0x6F,0x75,0x74,0x70,0x75,0x74,0x20,0x66,0x61,0x75,0x6C,0x74},  													 																		/*** 35 12V output  fault***/
																{0x44,0x52,0x49,0x56,0x45,0x52,0x33,0x20,0x63,0x6F,0x6E,0x6E,0x65,0x63,0x74,0x73,0x20,0x66,0x61,0x69,0x6C,0x75,0x72,0x65},  											/*** 36 DRIVER3 connects failure***/ 
																{0x44,0x52,0x49,0x56,0x45,0x52,0x34,0x20,0x63,0x6F,0x6E,0x6E,0x65,0x63,0x74,0x73,0x20,0x66,0x61,0x69,0x6C,0x75,0x72,0x65},  											/*** 37 DRIVER4 connects failure***/ 
																{0x52,0x57,0x20,0x70,0x61,0x72,0x61,0x6D,0x65,0x74,0x65,0x72,0x73,0x20,0x61,0x72,0x65,0x20,0x69,0x6E,0x63,0x6F,0x72,0x72,0x65,0x63,0x74},  				/*** 38 RW parameters are incorrect***/
																{0x50,0x61,0x72,0x61,0x6D,0x65,0x74,0x65,0x72,0x20,0x6C,0x69,0x6D,0x69,0x74,0x20,0x65,0x72,0x72,0x6F,0x72},  													 						/*** 39 Parameter limit error***/
																{0x4F,0x70,0x65,0x72,0x61,0x74,0x69,0x6F,0x6E,0x20,0x73,0x65,0x71,0x75,0x65,0x6E,0x63,0x65,0x20,0x65,0x72,0x72,0x6F,0x72},  											/*** 40 Operation sequence error***/
																{0x4C,0x6F,0x77,0x20,0x42,0x61,0x74,0x74,0x65,0x72,0x79,0x20,0x57,0x61,0x72,0x6E,0x69,0x6E,0x67},  																								/*** 41 Low Battery Warning***/
																{0x42,0x61,0x74,0x74,0x65,0x72,0x79,0x20,0x69,0x73,0x20,0x53,0x65,0x76,0x65,0x72,0x65,0x6C,0x79,0x20,0x4C,0x6F,0x77},  														/*** 42 Battery is Severely Low***/
																{0x00},{0x00},{0x00},{0x00},{0x00},{0x00},{0x00},{0x00},     /***43-50***/
																{0x44,0x52,0x49,0x56,0x45,0x52,0x31,0x20,0x63,0x6F,0x6E,0x6E,0x65,0x63,0x74,0x73,0x20,0x66,0x61,0x69,0x6C,0x75,0x72,0x65},									 /*51 DRIVER1 connects failure*/
																{0x44,0x52,0x49,0x56,0x45,0x52,0x32,0x20,0x63,0x6F,0x6E,0x6E,0x65,0x63,0x74,0x73,0x20,0x66,0x61,0x69,0x6C,0x75,0x72,0x65},
																{0x44,0x52,0x49,0x56,0x45,0x52,0x33,0x20,0x63,0x6F,0x6E,0x6E,0x65,0x63,0x74,0x73,0x20,0x66,0x61,0x69,0x6C,0x75,0x72,0x65},
																{0x44,0x52,0x49,0x56,0x45,0x52,0x34,0x20,0x63,0x6F,0x6E,0x6E,0x65,0x63,0x74,0x73,0x20,0x66,0x61,0x69,0x6C,0x75,0x72,0x65},
																{0x44,0x52,0x49,0x56,0x45,0x52,0x35,0x20,0x63,0x6F,0x6E,0x6E,0x65,0x63,0x74,0x73,0x20,0x66,0x61,0x69,0x6C,0x75,0x72,0x65},
																{0x44,0x52,0x49,0x56,0x45,0x52,0x36,0x20,0x63,0x6F,0x6E,0x6E,0x65,0x63,0x74,0x73,0x20,0x66,0x61,0x69,0x6C,0x75,0x72,0x65},
																{0x44,0x52,0x49,0x56,0x45,0x52,0x37,0x20,0x63,0x6F,0x6E,0x6E,0x65,0x63,0x74,0x73,0x20,0x66,0x61,0x69,0x6C,0x75,0x72,0x65},
																{0x44,0x52,0x49,0x56,0x45,0x52,0x38,0x20,0x63,0x6F,0x6E,0x6E,0x65,0x63,0x74,0x73,0x20,0x66,0x61,0x69,0x6C,0x75,0x72,0x65},     
																{0x44,0x52,0x49,0x56,0x45,0x52,0x39,0x20,0x63,0x6F,0x6E,0x6E,0x65,0x63,0x74,0x73,0x20,0x66,0x61,0x69,0x6C,0x75,0x72,0x65},                     /*59 DRIVER1 connects failure*/
																{0x00},{0x00},{0x00},{0x00},{0x00},{0x00},
																{0x50,0x72,0x6f,0x70,0x31,0x20,0x69,0x73,0x20,0x66,0x61,0x75,0x6c,0x74,0x79},	  												/*66 比例阀1故障*/
																{0x50,0x72,0x6f,0x70,0x32,0x20,0x69,0x73,0x20,0x66,0x61,0x75,0x6c,0x74,0x79},														/*67 比例阀2故障 */
																{0x00},{0x00},{0x00},
																{0x41,0x6e,0x61,0x6c,0x6f,0x67,0x31,0x20,0x66,0x61,0x69,0x6c,0x75,0x72,0x65},                           /*71 模拟量1故障*/
																{0x41,0x6e,0x61,0x6c,0x6f,0x67,0x32,0x20,0x66,0x61,0x69,0x6c,0x75,0x72,0x65},
																{0x41,0x6e,0x61,0x6c,0x6f,0x67,0x33,0x20,0x66,0x61,0x69,0x6c,0x75,0x72,0x65},
																{0x41,0x6e,0x61,0x6c,0x6f,0x67,0x34,0x20,0x66,0x61,0x69,0x6c,0x75,0x72,0x65},		 												/*74 模拟量4故障*/
																{0x00},{0x00},
																{0x45,0x6e,0x63,0x6f,0x64,0x65,0x72,0x31,0x20,0x66,0x61,0x75,0x6c,0x74},
																{0x45,0x6e,0x63,0x6f,0x64,0x65,0x72,0x31,0x20,0x66,0x61,0x75,0x6c,0x74},		 												/*78 编码器2故障*/
																{0x00},{0x00},											/***75-80***/
																{0x48,0x4C,0x53,0x20,0x53,0x77,0x69,0x20,0x69,0x73,0x20,0x61,0x62,0x6E,0x6F,0x72,0x6D,0x61,0x6C},  													 											/*** 81 HLS（Height Limit Speed） Swi is abnormal***/ 
																{0x55,0x6E,0x64,0x65,0x66,0x69,0x6E,0x65,0x64,0x20,0x63,0x6F,0x6E,0x74,0x72,0x6F,0x6C,0x6C,0x65,0x72},  													 								/*** 95 Undefined controller***/ 																											
};

const INT16S K_XISHU[210]=
{
	1,	1,	1,	1,	1,	1,	20,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,			//0~20
	1,	1,	1,	1,	32767,	32767,	32767,	32767,	32767,	32767,	32767,	32767,	32767,	32767,	32767,	32767,	32767,	32767,	32767,	32767,	1,			//21~41
	1,	1,	1,	10,	32767,	1,	1,	1,	1,	1,	1,	1,	1,	1,	32767,	32767,	1,	1,	1,	1,	32767,			//42~62
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,			//63~83         //2024,11.06 DSY 72,73,74改为 1
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	10,	32767,	8192,	255,	255,	1,	1,	1,	1,			//84~104
	1,	32767,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,			//105~125
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,																											
};
const INT16S K_CHU_XISHU[210]=
{
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1, 1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,			//0~20
	1,	1,	100,	1,	100,	100,	100,	100,	100,	100,	100,	100,	100,	100,	100,	100,	100,	100,	100,	100,	1,			//21~41
	1,	1,	1,	3,	100,	1,	1,	1,	1,	1,	1,	1,	1,	1,	100,	100,	1,	1,	1,	1,	100,			//42~62
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,			//63~83
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	10,	100,	100,	100,	100,	1,	1,	1,	1,			//84~104
	100,	100,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,			//105~125
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,																											
};
const INT16S B_XISHU[210]=
{
 	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,			//0-20
	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,			//21-41
	40,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,			//42-62
	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,			//62-83
	0,	0,	0,	0,	0,	0,	3000,	30000,	30000,	40,	40,	1600,	0,	0,	0,	0,	0,	0,	0,	0,	0,			//84-104
	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,																											
};
/*******************************************************************************
* FunctionName:void vErrPackSendPro(INT8U length)
* Description:  故障码打包处理，主要针对当前和历史故障做处理
* Input: uint8_t ErrCode,uint16_t length（当前故障码的长度 特指中文unicod和英文ASCII的长度<包含空格>）
* Output:  
*
* Author: 
* Date:
* Revision:V1.0 
*******************************************************************************/
static void vErrPackSendPro(uint8_t ErrCode,uint16_t Length)
{
	uint32_t Hour = 0;
	static INT16U  TxTimeOut1 = 0;
	uint32_t Minute = 0;
	INT8U DaoxuIndex = 0;
	HangChaPdo.ErrNum = ErrCode;
	HangChaPdo.LastErrByte = false;
	tCanFrame Can5C0Send = {.u32ID = 0x5C0, .u16DataLength = 8, .u8Data = {0x00}, .u8Rtr = 0};
	
	if(0 == CanRev670InfoLast.u8Language)
	{
		Length *= 2;		 //比如：控制器过流  五个双字节  字节总数就是5*2 = 10;
	}
	else if(1 == CanRev670InfoLast.u8Language)
	{
		Length *= 1;
	}
	
	if(Length >= 28)
		Length = 28;
	
	Hour = (u32HourCount * 6) / 60;
	Minute = (u32HourCount * 6) % 60;
	
	/*** Send process Processing ***/
	if(Length % 4) 
	{
		DaoxuIndex = (Length / 4) + 1;
	}
	else
		DaoxuIndex = Length / 4;
	
	switch (HangChaPdo.SubFunID)
	{
		case 0xFF:
			HangChaPdo.DgnDataRxTx.SubIndex = 0xFF;
			HangChaPdo.DataStateBit1 = Length;
			if(HangChaPdo.ErrcodeType == 1)
			{
				HangChaPdo.DataStateBit2  = 0x07; 
				HangChaPdo.DgnDataRxTx.DataL16 = ErrCode;
				HangChaPdo.DgnDataRxTx.DataH16 = ((HangChaPdo.DataStateBit1) | HangChaPdo.DataStateBit2 << 8);
			}
			else
			{
				HangChaPdo.DataStateBit2 = 0x06;
				HangChaPdo.DgnDataRxTx.DataL16 = HangChaPdo.ErrNum;
				HangChaPdo.DgnDataRxTx.DataH16 = ((HangChaPdo.DataStateBit1) | HangChaPdo.DataStateBit2 << 8);
			}
			break;
		case 0xFE:
			HangChaPdo.DgnDataRxTx.SubIndex = 0xFE;
			if(HangChaPdo.ErrcodeType == 1)
			{
				HangChaPdo.DgnDataRxTx.DataL16 = 0;
				HangChaPdo.DgnDataRxTx.DataH16 = 0;
			}
			else if(HangChaPdo.ErrcodeType == 2)
			{
				HangChaPdo.DgnDataRxTx.DataL16 = Hour;//TIME_InitStructure.Minute << 3;//TIME_InitStructure.Hour; 		//32-48Bit  
				HangChaPdo.DgnDataRxTx.DataH16 = Minute;		//49-54Bit
			}
			break;
		case 0xFD:
			HangChaPdo.DgnDataRxTx.SubIndex = 0xFD;
			if(HangChaPdo.SelectLanguage == 0)
			{
				if(HangChaPdo.ErrcodeType == 1)
				{
					HangChaPdo.DgnDataRxTx.DataL16 = ErrorUnicode[ErrCode - 1][0];
					HangChaPdo.DgnDataRxTx.DataH16 = ErrorUnicode[ErrCode - 1][1];
				}
				else if(HangChaPdo.ErrcodeType == 2)
				{
					HangChaPdo.DgnDataRxTx.DataL16 = ErrorUnicode[HangChaPdo.ErrNum - 1][0];
					HangChaPdo.DgnDataRxTx.DataH16 = ErrorUnicode[HangChaPdo.ErrNum - 1][1];					
				}
			}
			else
			{
				if(HangChaPdo.ErrcodeType == 1)
				{
					HangChaPdo.DgnDataRxTx.DataL16 = ErrorAscii[ErrCode - 1][0] | (ErrorAscii[ErrCode - 1][1] << 8);
					HangChaPdo.DgnDataRxTx.DataH16 = ErrorAscii[ErrCode - 1][2] | (ErrorAscii[ErrCode - 1][3] << 8);	
				}
				else if(HangChaPdo.ErrcodeType == 2)
				{
					HangChaPdo.DgnDataRxTx.DataL16 = ErrorAscii[HangChaPdo.ErrNum - 1][0] | (ErrorAscii[HangChaPdo.ErrNum - 1][1] << 8);
					HangChaPdo.DgnDataRxTx.DataH16 = ErrorAscii[HangChaPdo.ErrNum - 1][2] | (ErrorAscii[HangChaPdo.ErrNum - 1][3] << 8);						
				}
			}
			break;
		case 0xFC:
			HangChaPdo.DgnDataRxTx.SubIndex = 0xFC;
			if(HangChaPdo.SelectLanguage == 0)
			{
				if(HangChaPdo.ErrcodeType == 1)
				{
					HangChaPdo.DgnDataRxTx.DataL16 = ErrorUnicode[ErrCode - 1][2];
					HangChaPdo.DgnDataRxTx.DataH16 = ErrorUnicode[ErrCode - 1][3];
				}
				else if(HangChaPdo.ErrcodeType == 2)
				{
					HangChaPdo.DgnDataRxTx.DataL16 = ErrorUnicode[HangChaPdo.ErrNum - 1][2];
					HangChaPdo.DgnDataRxTx.DataH16 = ErrorUnicode[HangChaPdo.ErrNum - 1][3];					
				}
			}
			else
			{
				if(HangChaPdo.ErrcodeType == 1)
				{
					HangChaPdo.DgnDataRxTx.DataL16 = ErrorAscii[ErrCode - 1][4] | (ErrorAscii[ErrCode - 1][5] << 8);
					HangChaPdo.DgnDataRxTx.DataH16 = ErrorAscii[ErrCode - 1][6] | (ErrorAscii[ErrCode - 1][7] << 8);
				}
				else if(HangChaPdo.ErrcodeType == 2)
				{
					HangChaPdo.DgnDataRxTx.DataL16 = ErrorAscii[HangChaPdo.ErrNum - 1][4] | (ErrorAscii[HangChaPdo.ErrNum - 1][5] << 8);
					HangChaPdo.DgnDataRxTx.DataH16 = ErrorAscii[HangChaPdo.ErrNum - 1][6] | (ErrorAscii[HangChaPdo.ErrNum - 1][7] << 8);					
				}
			}
			 break;
		case 0xFB:
			HangChaPdo.DgnDataRxTx.SubIndex = 0xFB;
			if(HangChaPdo.SelectLanguage == 0)
			{
				if(HangChaPdo.ErrcodeType == 1)
				{
					HangChaPdo.DgnDataRxTx.DataL16 = ErrorUnicode[ErrCode - 1][4];
					HangChaPdo.DgnDataRxTx.DataH16 = ErrorUnicode[ErrCode - 1][5];
				}
				else if(HangChaPdo.ErrcodeType == 2)
				{
					HangChaPdo.DgnDataRxTx.DataL16 = ErrorUnicode[HangChaPdo.ErrNum - 1][4];
					HangChaPdo.DgnDataRxTx.DataH16 = ErrorUnicode[HangChaPdo.ErrNum - 1][5];					
				}
			}
			else
			{
				if(HangChaPdo.ErrcodeType == 1)
				{
					HangChaPdo.DgnDataRxTx.DataL16 = ErrorAscii[ErrCode - 1][8] | (ErrorAscii[ErrCode - 1][9] << 8);
					HangChaPdo.DgnDataRxTx.DataH16 = ErrorAscii[ErrCode - 1][10] | (ErrorAscii[ErrCode - 1][11] << 8);
				}
				else if(HangChaPdo.ErrcodeType == 2)
				{
					HangChaPdo.DgnDataRxTx.DataL16 = ErrorAscii[HangChaPdo.ErrNum - 1][8] | (ErrorAscii[HangChaPdo.ErrNum - 1][9] << 8);
					HangChaPdo.DgnDataRxTx.DataH16 = ErrorAscii[HangChaPdo.ErrNum - 1][10] | (ErrorAscii[HangChaPdo.ErrNum - 1][11] << 8);					
				}				
			}
			break;
		case 0xFA:
			HangChaPdo.DgnDataRxTx.SubIndex = 0xFA;
			if(HangChaPdo.SelectLanguage == 0)
			{
				if(HangChaPdo.ErrcodeType == 1)
				{
					HangChaPdo.DgnDataRxTx.DataL16 = ErrorUnicode[ErrCode - 1][6];
					HangChaPdo.DgnDataRxTx.DataH16 = ErrorUnicode[ErrCode - 1][7];
				}
				else if(HangChaPdo.ErrcodeType == 2)
				{
					HangChaPdo.DgnDataRxTx.DataL16 = ErrorUnicode[HangChaPdo.ErrNum - 1][6];
					HangChaPdo.DgnDataRxTx.DataH16 = ErrorUnicode[HangChaPdo.ErrNum - 1][7];					
				}
			}
			else
			{
				if(HangChaPdo.ErrcodeType == 1)
				{
					HangChaPdo.DgnDataRxTx.DataL16 = ErrorAscii[ErrCode - 1][12] | (ErrorAscii[ErrCode - 1][13] << 8);
					HangChaPdo.DgnDataRxTx.DataH16 = ErrorAscii[ErrCode - 1][14] | (ErrorAscii[ErrCode - 1][15] << 8);
				}
				else if(HangChaPdo.ErrcodeType == 2)
				{
					HangChaPdo.DgnDataRxTx.DataL16 = ErrorAscii[HangChaPdo.ErrNum - 1][12] | (ErrorAscii[HangChaPdo.ErrNum - 1][13] << 8);
					HangChaPdo.DgnDataRxTx.DataH16 = ErrorAscii[HangChaPdo.ErrNum - 1][14] | (ErrorAscii[HangChaPdo.ErrNum - 1][15] << 8);					
				}
			}
			break;
		case 0xF9:
			HangChaPdo.DgnDataRxTx.SubIndex = 0xF9;
			if(HangChaPdo.SelectLanguage == 0)
			{
				if(HangChaPdo.ErrcodeType == 1)
				{
					HangChaPdo.DgnDataRxTx.DataL16 = ErrorUnicode[ErrCode - 1][8];
					HangChaPdo.DgnDataRxTx.DataH16 = ErrorUnicode[ErrCode - 1][9];
				}
				else if(HangChaPdo.ErrcodeType == 2)
				{
					HangChaPdo.DgnDataRxTx.DataL16 = ErrorUnicode[HangChaPdo.ErrNum - 1][8];
					HangChaPdo.DgnDataRxTx.DataH16 = ErrorUnicode[HangChaPdo.ErrNum - 1][9];					
				}
			}
			else
			{
				if(HangChaPdo.ErrcodeType == 1)
				{
					HangChaPdo.DgnDataRxTx.DataL16 = ErrorAscii[ErrCode - 1][16] | (ErrorAscii[ErrCode - 1][17] << 8);
					HangChaPdo.DgnDataRxTx.DataH16 = ErrorAscii[ErrCode - 1][18] | (ErrorAscii[ErrCode - 1][19] << 8);
				}
				else if(HangChaPdo.ErrcodeType == 2)
				{
					HangChaPdo.DgnDataRxTx.DataL16 = ErrorAscii[HangChaPdo.ErrNum - 1][16] | (ErrorAscii[HangChaPdo.ErrNum - 1][17] << 8);
					HangChaPdo.DgnDataRxTx.DataH16 = ErrorAscii[HangChaPdo.ErrNum - 1][18] | (ErrorAscii[HangChaPdo.ErrNum - 1][19] << 8);					
				}
			}
			break;
		case 0xF8:
			HangChaPdo.DgnDataRxTx.SubIndex = 0xF8;
			if(HangChaPdo.SelectLanguage == 1)
			{
				if(HangChaPdo.ErrcodeType == 1)
				{
					HangChaPdo.DgnDataRxTx.DataL16 = ErrorAscii[ErrCode - 1][20] | (ErrorAscii[ErrCode - 1][21] << 8);
					HangChaPdo.DgnDataRxTx.DataH16 = ErrorAscii[ErrCode - 1][22] | (ErrorAscii[ErrCode - 1][23] << 8);
				}
				else if(HangChaPdo.ErrcodeType == 2)
				{
					HangChaPdo.DgnDataRxTx.DataL16 = ErrorAscii[HangChaPdo.ErrNum - 1][20] | (ErrorAscii[HangChaPdo.ErrNum - 1][21] << 8);
					HangChaPdo.DgnDataRxTx.DataH16 = ErrorAscii[HangChaPdo.ErrNum - 1][22] | (ErrorAscii[HangChaPdo.ErrNum - 1][23] << 8);					
				}
			}
			break;
		case 0xF7:
			HangChaPdo.DgnDataRxTx.SubIndex = 0xF7;
			if(HangChaPdo.SelectLanguage == 1)
			{
				if(HangChaPdo.ErrcodeType == 1)
				{
					HangChaPdo.DgnDataRxTx.DataL16 = ErrorAscii[ErrCode - 1][24] | (ErrorAscii[ErrCode - 1][25] << 8);
					HangChaPdo.DgnDataRxTx.DataH16 = ErrorAscii[ErrCode - 1][26] | (ErrorAscii[ErrCode - 1][27] << 8);
				}
				else if(HangChaPdo.ErrcodeType == 2)
				{
					HangChaPdo.DgnDataRxTx.DataL16 = ErrorAscii[HangChaPdo.ErrNum - 1][24] | (ErrorAscii[HangChaPdo.ErrNum - 1][25] << 8);
					HangChaPdo.DgnDataRxTx.DataH16 = ErrorAscii[HangChaPdo.ErrNum - 1][26] | (ErrorAscii[HangChaPdo.ErrNum - 1][27] << 8);					
				}
			}
			break;
		}
			Can5C0Send.u8Data[0] = 0x43;
			if(1 == HangChaPdo.ErrcodeType)
				Can5C0Send.u8Data[1] = 0x11 & 0xFF;
			else
				Can5C0Send.u8Data[1] = 0x12 & 0xFF;
			Can5C0Send.u8Data[2] = 0xFF & 0xFF;
			Can5C0Send.u8Data[3] = HangChaPdo.DgnDataRxTx.SubIndex;
			Can5C0Send.u8Data[4] = HangChaPdo.DgnDataRxTx.DataL16 & 0xFF;
			Can5C0Send.u8Data[5] = (HangChaPdo.DgnDataRxTx.DataL16 >> 8) & 0xFF;
			Can5C0Send.u8Data[6] = HangChaPdo.DgnDataRxTx.DataH16 & 0xFF;
			Can5C0Send.u8Data[7] = (HangChaPdo.DgnDataRxTx.DataH16 >> 8) & 0xFF;
			if(CAN_SUCCESS == i32CanWrite(Can0, &Can5C0Send))
			{
				if(HangChaPdo.SubFunID <=  0xFD - DaoxuIndex + 1)
				{
					HangChaPdo.LastErrByte = true;
					HangChaPdo.SubFunID = 0xFF;
				}
				else
				{
					HangChaPdo.SubFunID--;
				}
			}

}

static void ErrCodeSend(uint8_t ErrcodeType)        //01 Now 02 History
{
	uint8_t ErrCodeNow = 0;
	uint8_t i;
	static uint16_t ErrorTotalLength = 0;
	static uint8_t ErrCodeSendDelay = 0;
	
	tCanFrame Can5C0Send = {.u32ID = 0x5C0, .u16DataLength = 8, .u8Data = {0x00}, .u8Rtr = 0};
	if(true == HangChaPdo.SendErrFlag)
	{
		Can5C0Send.u8Data[0] = 0x43;
		Can5C0Send.u8Data[1] = BSdoState.Address & 0xFF;
		Can5C0Send.u8Data[2] = (BSdoState.Address >> 8) & 0xFF;
		Can5C0Send.u8Data[3] = HangChaPdo.ErrcodeType & 0xFF;
		Can5C0Send.u8Data[4] = 0;
		Can5C0Send.u8Data[5] = 0;
		Can5C0Send.u8Data[6] = 0xC0 & 0xFF;
		Can5C0Send.u8Data[7] = 0x05 & 0xFF;
		i32CanWrite(Can0, &Can5C0Send);
		HangChaPdo.SendErrFlag = false;
	}
	
	if(1 == HangChaPdo.ErrcodeType)
	{
		ErrCodeNow = u8ErrCodeGet();
	}
	else if(2 == HangChaPdo.ErrcodeType)
	{
		ErrCodeNow = i32GetPara(Count);
	}
	
	if(0 != ErrCodeNow) 
	{
		if(0 == CanRev670InfoLast.u8Language)
		{
			while((ErrorUnicode[ErrCodeNow - 1][ErrorTotalLength] != L'\0') && (ErrorTotalLength < (sizeof(ErrorUnicode[ErrCodeNow]) / sizeof(wchar_t))))
			{
				ErrorTotalLength++; //多少个汉字（多少个双字节）
			}
		}
		else if(1 == CanRev670InfoLast.u8Language)
		{
			while((ErrorAscii[ErrCodeNow - 1][ErrorTotalLength] != '\0') && (ErrorTotalLength < (sizeof(ErrorAscii[ErrCodeNow]) / sizeof(char))))
			{
				ErrorTotalLength++;  //多少个单字节
			}		
		}
	}
	else
	{
		ErrorTotalLength = 0;
	}
	
	if(1 == HangChaPdo.ErrcodeType)              // 当前故障
	{
		if(0 != ErrCodeNow) 
		{
			vErrPackSendPro(ErrCodeNow,ErrorTotalLength);
		}
	}
	else if(2 == HangChaPdo.ErrcodeType)
	{
		if(((i32GetPara(Count) != 0)&&(Count <= PARA_ErrCodeMax))&&(CountLast != Count))
		{
			CountLast = Count;
			vErrPackSendPro(i32GetPara(Count),ErrorTotalLength); 
			if(true == HangChaPdo.LastErrByte)
			{
				if(ErrCodeSendDelay < 100)
					ErrCodeSendDelay++;
				else
				{
					ErrCodeSendDelay = 0;
					Count++;
				}		
			}
				
		}
		else
		{
			HangChaPdo.ErrcodeType = 0;
			SendHistoryErrCode = 1;
			RevErrCodeAddr = 0;
			BSdoState.Address = 0;
		}
	}
	
}

static uint32_t RevDateManage(uint32_t RevDate)
{
	uint32_t DataTmp = 0;
	switch(u32RecieveAdress)
	{
		case 0x202001:
			DataTmp = (RevDate>>3)&0x01;
			break;
		case 0x202003:
			DataTmp = (RevDate>>4)&0x01;
			break;
		case 0x202004:
			DataTmp = (RevDate>>5)&0x01;
			break;
		
		case 0x202201:
			DataTmp = (RevDate>>6)&0x01;
			break;
		case 0x202202:
			DataTmp = (RevDate>>7)&0x01;
			break;
		
		case 0x202801:
			DataTmp = RevDate&0x01;
			break;
		case 0x202802:
			DataTmp = (RevDate>>1)&0x01;
			break;
		
		case 0x200D04:
			DataTmp = (RevDate)&0x01;
			break;
		case 0x200D05:
			DataTmp = (RevDate>>1)&0x01;
			break;
		
		case 0x202B02:
			DataTmp = (RevDate>>4)&0x01;
			break;
		case 0x202B03:
			DataTmp = (RevDate>>6)&0x01;
			break;
		case 0x202B04:
			DataTmp = (RevDate>>7)&0x01;
			break;
		
		case 0x202B19:
				DataTmp = (RevDate)&0x01;
			break;
		case 0x202B05:
				DataTmp = (RevDate>>1)&0x01;
			break;
		case 0x202B06:
				DataTmp = (RevDate>>2)&0x01;
			break;
		case 0x202B07:
				DataTmp = (RevDate>>3)&0x01;
			break;
		case 0x202B08:
				DataTmp = (RevDate>>4)&0x01;
			break;
		case 0x202B0A:
				DataTmp = (RevDate>>7)&0x01;
			break;
		
		case 0x202E03:
			DataTmp = (RevDate)&0x01;
			break;
		case 0x202E04:
			DataTmp = (RevDate>>1)&0x01;
			break;
		case 0x202E05:
			DataTmp = (RevDate>>2)&0x01;
			break;
		case 0x202E06:
			DataTmp = (RevDate>>3)&0x01;
			break;
		case 0x202E07:
			DataTmp = (RevDate>>4)&0x01;
			break;
		case 0x202E08:
			DataTmp = (RevDate>>5)&0x01;
			break;
		case 0x202E09:
			DataTmp = (RevDate>>6)&0x01;
			break;
		
		case 0x202E0A:
			DataTmp = (RevDate)&0x01;
			break;
		case 0x202E0B:
			DataTmp = (RevDate>>1)&0x01;
			break;
		case 0x202E0C:
			DataTmp = (RevDate>>2)&0x01;
			break;
		case 0x202E0D:
			DataTmp = (RevDate>>3)&0x01;
			break;
		case 0x202E0E:
			DataTmp = (RevDate>>4)&0x01;
			break;
		case 0x202E0F:
			DataTmp = (RevDate>>5)&0x01;
			break;
		case 0x202E10:
			DataTmp = (RevDate>>6)&0x01;
			break;
		
		case 0x201901:
			DataTmp = (RevDate>>5)&0x01;
			break;
		case 0x201902:
			DataTmp = (RevDate>>2)&0x01;
			break;
		case 0x201903:
			DataTmp = (RevDate>>5)&0x01;
			break;
		
		case 0x400101:
			DataTmp = (RevDate)&0x01;
			break;
		case 0x400102:
			DataTmp = (RevDate>>1)&0x01;
			break;
		case 0x400103:
			DataTmp = (RevDate>>2)&0x01;
			break;
		case 0x400104:
			DataTmp = (RevDate>>3)&0x01;
			break;
		case 0x400105:
			DataTmp = (RevDate>>4)&0x01;
			break;
		case 0x400106:
			DataTmp = (RevDate>>5)&0x01;
			break;
		case 0x400107:
			DataTmp = (RevDate>>6)&0x01; 
			break;
		case 0x400108:
			DataTmp = (RevDate>>7)&0x01;
			break;
		case 0x400109:
			DataTmp = (RevDate>>8)&0x01;
			break;
		case 0x40010A:
			DataTmp = (RevDate>>9)&0x01;
			break;
		case 0x40010B:
			DataTmp = (RevDate>>10)&0x01;
			break;
		case 0x40010C:
			DataTmp = (RevDate>>11)&0x01;
			break;
		case 0x40010D:
			DataTmp = (RevDate>>12)&0x01;
			break;
		case 0x40010E:
			DataTmp = (RevDate>>13)&0x01;
			break;
		case 0x40010F:
			DataTmp = (RevDate>>14)&0x01;
			break;
		case 0x400110:
			DataTmp = (RevDate>>15)&0x01;
			break;
		
		/*20241107 Dsy 添加外部输入对应SwiInput*/
		case 0x40030B:
			DataTmp = (RevDate>>0)&0x01;              // √
			break;
		case 0x400301:
			DataTmp = (RevDate>>2)&0x01;              // √
			break;
		case 0x40030A:
			DataTmp = (RevDate>>3)&0x01;              // √
			break;
		case 0x400309:
			DataTmp = (RevDate>>5)&0x01;              // √
			break;
		case 0x40030C:
			DataTmp = (RevDate>>6)&0x01;              // √
			break;
		case 0x400308:
		case 0x40030D:															//平衡重式只有一个护栏，所以两个一起显示。
			DataTmp = (RevDate>>7)&0x01;              // √
			break;
		case 0x400405:
			DataTmp = (RevDate>>9)&0x01;              // √
			break;	
		case 0x400302:
			DataTmp = (RevDate>>10)&0x01;              // √
			break;
		case 0x400303:
			DataTmp = (RevDate>>11)&0x01;              // √
			break;

		case 0x400920:
		case 0x40092A:
			DataTmp = sgMcuState.b1Do1State;              // √
			break;
		case 0x400921:
		case 0x40092B:
			DataTmp = sgMcuState.b1EbrakeState;              // √
			break;
		case 0x400922:
		case 0x40092C:
			DataTmp = !((RevDate>>2)&0x01);              // √
			break;
		case 0x400923:
		case 0x400933:
			DataTmp = !((RevDate>>3)&0x01);
			break;
		case 0x400924:
		case 0x400932:
			DataTmp = !((RevDate>>4)&0x01);
			break;
		case 0x400925:
		case 0x400936:
			DataTmp = !((RevDate>>5)&0x01);
			break;
		case 0x400926:
		case 0x400930:
			DataTmp = !((RevDate>>6)&0x01);
			break;
		case 0x400927:
		case 0x400931:
			DataTmp = !((RevDate>>7)&0x01);
			break;
		case 0x400928:
		case 0x400934:
			DataTmp = !((RevDate>>8)&0x01);
			break;
		case 0x400929:
		case 0x400935:
			DataTmp = !((RevDate>>9)&0x01);
			break;																		//输出信号监控
		
		case 0x204106:
			DataTmp = (RevDate>>2)&0x01;
			break;
		
		case 0x201C01:
			DataTmp = (RevDate>>6)&0x01;
			break;
		case 0x201C02:
			DataTmp = (RevDate>>7)&0x01;
			break;
		
		case 0x201C0A:
			DataTmp = (RevDate>>0)&0x01;
			break;
		case 0x201C0B:
			DataTmp = (RevDate>>1)&0x01;
			break;
		
		case 0x201C0C:
			DataTmp = (RevDate>>2)&0x01;
			break;
		case 0x201C0D:
			DataTmp = (RevDate>>3)&0x01;
			break;
		case 0x201C0E:
			DataTmp = (RevDate>>4)&0x01;
			break;
		case 0x201C0F:
			DataTmp = (RevDate>>5)&0x01;
			break;
		case 0x201C13:
			DataTmp = (RevDate>>6)&0x01;
			break;
		case 0x201C14:
			DataTmp = (RevDate>>7)&0x01;
			break;
		case 0x201C15:
			DataTmp = (RevDate>>8)&0x01;
			break;
		case 0x201C16:
			DataTmp = (RevDate>>9)&0x01;
			break;
		
		case 0x201C4C:
		case 0x201C4E:
		case 0x201C50:
		case 0x201C51:
		case 0x201C52:
		case 0x201C53:
		case 0x201C54:
		case 0x201C55:
			DataTmp = (RevDate>>8)&0xFF;
			break;
		case 0x201C4D:
		case 0x201C4F:
		case 0x201C58:
		case 0x201C59:
		case 0x201C5A:
		case 0x201C5B:
		case 0x201C5C:
		case 0x201C5D:
			DataTmp = (RevDate)&0xFF;
			break;
		
		case 0x40092E:
			if(LSC0407DEM != sgUserInfo.VehicleType)
			{
				if((1 == sgHandleInput.b1YouLiftDown)
					&&(0 == sgHandleInput.b1YouLiftUp)
					&&(0 == (sgu16LimitFlg & LiftDown_Limit)))
				{
					DataTmp = 1;
				}
				else
					DataTmp = 0;
			}
			else
			{
				if((((1 == sgHandleInput.b1YouLiftDown)&&(0 == sgHandleInput.b1ZuoLiftDown))
								||((0 == sgHandleInput.b1YouLiftDown)&&(1 == sgHandleInput.b1ZuoLiftDown)))
								&&(0 == sgHandleInput.b1YouLiftUp)
								&&(0 == sgHandleInput.b1ZuoLiftUp)
								&&(0 == (sgu16LimitFlg & LiftDown_Limit)))
				{
					DataTmp = 1;
				}
				else
					DataTmp = 0;
				
			}	
			break;
		
		case 0x400D50:
			DataTmp = 0x1A0318;          
			break;
		
		default:
			DataTmp = RevDate;
		break;
	}
	return DataTmp;
}

static uint32_t TexDateManage(uint32_t RevDate)
{
	switch(u32RecieveAdress)
	{
		case 0x202001:
			tmpData.b1TmpData3 = RevDate;
			break;
		case 0x202003:
			tmpData.b1TmpData4 = RevDate;
			break;
		case 0x202004:
			tmpData.b1TmpData5 = RevDate;
			break;
		case 0x202201:
			tmpData.b1TmpData6 = RevDate;
			break;
		case 0x202202:
			tmpData.b1TmpData7 = RevDate;
			break;
		case 0x202801:
			tmpData.b1TmpData0 = RevDate;
			break;
		case 0x202802:
			tmpData.b1TmpData1 = RevDate;
			break;
		case 0x200D04:
			tmpData.b1TmpData0 = RevDate;
			break;
		case 0x200D05:
			tmpData.b1TmpData1 = RevDate;
			break;
		
		case 0x202B02:
			tmpData.b1TmpData4 = RevDate;
			break;
		case 0x202B03:
			tmpData.b1TmpData6 = RevDate;
			break;
		case 0x202B04:
			tmpData.b1TmpData7 = RevDate;
			break;
		
		case 0x204106:
			tmpData.b1TmpData2 = RevDate;
		break;
		
		case 0x202B19:
				tmpData.b1TmpData0 = RevDate;
			break;
		case 0x202B05:
				tmpData.b1TmpData1 = RevDate;
			break;
		case 0x202B06:
				tmpData.b1TmpData2 = RevDate;
			break;
		case 0x202B07:
				tmpData.b1TmpData3 = RevDate;
			break;
		case 0x202B08:
				tmpData.b1TmpData4 = RevDate;
			break;
		case 0x202B0A:
				tmpData.b1TmpData7 = RevDate;
			break;
		
		case 0x202E03:
				tmpData.b1TmpData0 = RevDate;
			break;
		case 0x202E04:
				tmpData.b1TmpData1 = RevDate;
			break;
		case 0x202E05:
				tmpData.b1TmpData2 = RevDate;
			break;
		case 0x202E06:
				tmpData.b1TmpData3 = RevDate;
			break;
		case 0x202E07:
				tmpData.b1TmpData4 = RevDate;
			break;
		case 0x202E08:
				tmpData.b1TmpData5 = RevDate;
			break;
		case 0x202E09:
				tmpData.b1TmpData6 = RevDate;
			break;
		
		case 0x202E0A:
				tmpData.b1TmpData0 = RevDate;
			break;
		case 0x202E0B:
				tmpData.b1TmpData1 = RevDate;
			break;
		case 0x202E0C:
				tmpData.b1TmpData2 = RevDate;
			break;
		case 0x202E0D:
				tmpData.b1TmpData3 = RevDate;
			break;
		case 0x202E0E:
				tmpData.b1TmpData4 = RevDate;
			break;
		case 0x202E0F:
				tmpData.b1TmpData5 = RevDate;
			break;
		case 0x202E10:
				tmpData.b1TmpData6 = RevDate;
			break;
		
		case 0x201901:
			tmpData.b1TmpData5 = RevDate;
			break;
		case 0x201902:
			tmpData.b1TmpData2 = RevDate;
			break;
		case 0x201903:
			tmpData.b1TmpData5 = RevDate;
			break;
		
		case 0x201C01:
			tmpData.b1TmpData6 = RevDate;
			break;
		case 0x201C02:
			tmpData.b1TmpData7 = RevDate;
			break;
		case 0x201C0A:
			tmpData.b1TmpData0 = RevDate;
			break;
		case 0x201C0B:
			tmpData.b1TmpData1 = RevDate;
			break;
		case 0x201C0C:
			tmpData.b1TmpData2 = RevDate;
			break;
		case 0x201C0D:
			tmpData.b1TmpData3 = RevDate;
			break;
		case 0x201C0E:
			tmpData.b1TmpData4 = RevDate;
			break;
		case 0x201C0F:
			tmpData.b1TmpData5 = RevDate;
			break;
		case 0x201C13:
			tmpData.b1TmpData6 = RevDate;
			break;
		case 0x201C14:
			tmpData.b1TmpData7 = RevDate;
			break;
		case 0x201C15:
			tmpData.b1TmpData8 = RevDate;
			break;
		case 0x201C16:
			tmpData.b1TmpData9 = RevDate;
			break;
		case 0x201C4C:
		case 0x201C4E:
		case 0x201C50:
		case 0x201C51:
		case 0x201C52:
		case 0x201C53:
		case 0x201C54:
		case 0x201C55:
			tmpData.b1TmpData15 = (RevDate>>7)&0x01;
			tmpData.b1TmpData14 = (RevDate>>6)&0x01;
			tmpData.b1TmpData13 = (RevDate>>5)&0x01;
			tmpData.b1TmpData12 = (RevDate>>4)&0x01;
			tmpData.b1TmpData11 = (RevDate>>3)&0x01;
			tmpData.b1TmpData10 = (RevDate>>2)&0x01;
			tmpData.b1TmpData9 = (RevDate>>1)&0x01;
			tmpData.b1TmpData8 = (RevDate>>0)&0x01;
			break;
		case 0x201C4D:
		case 0x201C4F:
		case 0x201C58:
		case 0x201C59:
		case 0x201C5A:
		case 0x201C5B:
		case 0x201C5C:
		case 0x201C5D:
			tmpData.b1TmpData7 = (RevDate>>7)&0x01;
			tmpData.b1TmpData6 = (RevDate>>6)&0x01;
			tmpData.b1TmpData5 = (RevDate>>5)&0x01;
			tmpData.b1TmpData4 = (RevDate>>4)&0x01;
			tmpData.b1TmpData3 = (RevDate>>3)&0x01;
			tmpData.b1TmpData2 = (RevDate>>2)&0x01;
			tmpData.b1TmpData1 = (RevDate>>1)&0x01;
			tmpData.b1TmpData0 = (RevDate>>0)&0x01;
			break;
		
		default:
			tmpData.u32data = RevDate;
		break;
	}
	return tmpData.u32data;
}

void GetParaFromMcu(uint8_t u8CS,uint32_t SendAddress)
{
//	uint8_t u8Vector[8];
//	uint32_t u32DataReq = 0;
//	vIndexToAdress(SendAddress,&u16SendAdress,&u16Factor);
//	
//	u32DataReq = (uint32_t)((CanRev640InfoLast.u8Rev640Date5)|(CanRev640InfoLast.u8Rev640Date6 << 8)|(CanRev640InfoLast.u8Rev640Date7<<16));
//	u32DataReq = (u32DataReq * K_XISHU[u16Factor])/K_CHU_XISHU[u16Factor] + B_XISHU[u16Factor] ;
//	if(u16SendAdress < 255)
//		tmpData.u32data = MstPARADate[u16SendAdress];
//	u32DataReq = TexDateManage(u32DataReq);
//	if(u16SendAdress < 255)
//		MstPARADate[u16SendAdress] = u32DataReq;
//	u8Vector[0] = u8CS;
//	u8Vector[1] = u16SendAdress & 0xFF;
//	u8Vector[2] = (u16SendAdress >> 8) & 0xFF;
//	u8Vector[3] = 0;
//	u8Vector[4] = u32DataReq & 0xFF;
//	u8Vector[5] = (u32DataReq >> 8) & 0xFF;
//	u8Vector[6] = (u32DataReq >> 16) & 0xFF;
//	u8Vector[7] = (u32DataReq >> 24) & 0xFF;
//	
//	vQueryMcuPara(u8Vector,8);
}


void GetParaFromEcu(uint8_t u8CS,uint32_t SendAddress)
{
	switch(u8CS)
	{
		case 0x40:
			BSdoState.SdoFlg |= Read_Cmd;
			vIndexToAdress2ECU(SendAddress,&u16SendAdress,&u16Factor);
			break;
		case 0x2B:
		case 0x23:
			BSdoState.SdoFlg |= Write_Cmd;
			vIndexToAdress2ECU(SendAddress,&u16SendAdress,&u16Factor);
			break;
		default:
			break;
	}

}
static void vGetMcuPareFLocal(uint8_t u8CS,uint32_t SendAddress)
{
		uint8_t u8Vector[8];
		uint32_t u32DataSend1 = 0;
		uint32_t u32DataReq = 0;
		tCanFrame CanSendFrame;
		vIndexToAdress(SendAddress,&u16SendAdress,&u16Factor);
		
		u32DataReq = (uint32_t)((CanRev640InfoLast.u8Rev640Date5)|(CanRev640InfoLast.u8Rev640Date6 << 8)|(CanRev640InfoLast.u8Rev640Date7<<16));
		u32DataReq = (u32DataReq * K_XISHU[u16Factor])/K_CHU_XISHU[u16Factor] + B_XISHU[u16Factor] ;
	
		if((0x40 == u8CS)&&(u16SendAdress < 255))            // 此处仅为配置参数的读
		{
			if((0 == u16SendAdress)&&(0 == u16Factor))
			{
				u32DataSend1 = 0;
			}
			u32DataSend1 = RevDateManage(MstPARADate[u16SendAdress + 1]); //改完后数组整体偏移了一位
			
			CanSendFrame.u32ID = 0x5C0;
			CanSendFrame.u8Rtr = 0;
			CanSendFrame.u16DataLength = 8;

			u32DataSend1 = ((u32DataSend1 + B_XISHU[u16Factor]) * K_CHU_XISHU[u16Factor]) / K_XISHU[u16Factor];
			
			CanSendFrame.u8Data[0] = 0x43;
			CanSendFrame.u8Data[1] = u32RecieveAdress>>8;
			CanSendFrame.u8Data[2] = u32RecieveAdress>>16;
			CanSendFrame.u8Data[3] = u8Index;
			CanSendFrame.u8Data[4] = u32DataSend1 & 0xFF;
			CanSendFrame.u8Data[5] = (u32DataSend1>>8) & 0xFF;
			CanSendFrame.u8Data[6] = (u32DataSend1>>16) & 0xFF;
			CanSendFrame.u8Data[7] = (u32DataSend1>>24) & 0xFF;

			i32CanWrite(Can0, &CanSendFrame);
		}
		else                               // 此处不仅有 MCU的写 还有 MCU 监控的读命令
		{
			if(u16SendAdress < 255)
			{
				tmpData.u32data = MstPARADate[u16SendAdress + 1];
				u32DataReq = TexDateManage(u32DataReq);
				MstPARADate[u16SendAdress + 1] = u32DataReq;
			}
			u8Vector[0] = u8CS;
			u8Vector[1] = u16SendAdress & 0xFF;
			u8Vector[2] = (u16SendAdress >> 8) & 0xFF;
			u8Vector[3] = 0;
			u8Vector[4] = u32DataReq & 0xFF;
			u8Vector[5] = (u32DataReq >> 8) & 0xFF;
			u8Vector[6] = (u32DataReq >> 16) & 0xFF;
			u8Vector[7] = (u32DataReq >> 24) & 0xFF;
			
			vQueryMcuPara(u8Vector,8);
		}
}

static void vMcuParaRevProc(uint8_t *u8Data, uint16_t u16Length)
{
	uint8_t u8DataArray[8];
	uint32_t u32DataSend = 0;
	memcpy(u8DataArray,u8Data,8);
	
	tCanFrame CanSendFrame;
	u32DataSend = u8DataArray[4]|(u8DataArray[5]<<8)|(u8DataArray[6]<<16)|(u8DataArray[7]<<24);
	if(true == PowerOnGetPara)
	{
		if(RevMstDate == u8DataArray[1]) //Idx相同写入
		{
			MstPARADate[RevMstDate] = u32DataSend;
			MstParaIsOk = 1;
			RevMstDate++;
		}					
	}
	else
	{
		
		if((0 == u16SendAdress)&&(0 == u16Factor))
		{
			u32DataSend = 0;
		}
		if(u16SendAdress < 255)
			u32DataSend = RevDateManage(MstPARADate[u16SendAdress + 1]);
		else
			u32DataSend = RevDateManage(u32DataSend);
		
		CanSendFrame.u32ID = 0x5C0;
		CanSendFrame.u8Rtr = 0;
		CanSendFrame.u16DataLength = 8;

		u32DataSend = ((u32DataSend + B_XISHU[u16Factor]) * K_CHU_XISHU[u16Factor]) / K_XISHU[u16Factor];
		
		CanSendFrame.u8Data[0] = u8DataArray[0];
		CanSendFrame.u8Data[1] = u32RecieveAdress>>8;
		CanSendFrame.u8Data[2] = u32RecieveAdress>>16;
		CanSendFrame.u8Data[3] = u8Index;
		CanSendFrame.u8Data[4] = u32DataSend & 0xFF;
		CanSendFrame.u8Data[5] = (u32DataSend>>8) & 0xFF;
		CanSendFrame.u8Data[6] = (u32DataSend>>16) & 0xFF;
		CanSendFrame.u8Data[7] = (u32DataSend>>24) & 0xFF;

		i32CanWrite(Can0, &CanSendFrame);
		
	}
}


static void vCanId670Proc(tCanFrame * CanFrame)
{
	tCanFrame Can5C0Send = {.u32ID = 0x5C0, .u16DataLength = 8, .u8Data = {0x00}, .u8Rtr = 0};
	memcpy((char*)CanRev670InfoLast.u8Data, (char*) CanFrame->u8Data, sizeof(CanRev670InfoLast));
	Can5C0Send.u8Data[0] = 0x01;
	Can5C0Send.u8Data[1] = 0x00;
	Can5C0Send.u8Data[2] = 0x00;
	Can5C0Send.u8Data[3] = 0x00;
	Can5C0Send.u8Data[4] = 0x04;
	i32CanWrite(Can0, &Can5C0Send);
}

static void vCanId640Proc(tCanFrame * CanFrame)
{
	tCanFrame Can5C0Send = {.u32ID = 0x5C0, .u16DataLength = 8, .u8Data = {0}, .u8Rtr = 0};
	char ReplaceChar1[4] = "";
	char ParseKeyFour[4];
	char tempword[20] = "C2425HB4";
	uint16_t RevAddress;
	uint8_t RevCs;
	uint8_t FuncCode;
	uint32_t ChangeHourPassWd = 0;
	static uint8_t PassWdOk = 0;
	
	memcpy((char*)CanRev640InfoLast.u8Data, (char*) CanFrame->u8Data, sizeof(CanRev640InfoLast));
	RevCmd = CanRev640InfoLast.u8Data[0];
	RevAddress = CanRev640InfoLast.u8Data[1]|(CanRev640InfoLast.u8Data[2]<<8);
	BSdoState.Address = RevAddress;
	RevCs = CanRev640InfoLast.u8Data[3];
	u32RecieveAdress = (RevAddress<<8)|RevCs;
	u8Index = RevCs;
	BSdoState.SdoCs = RevCs;
	BSdoState.SaveData = (uint32_t)((CanRev640InfoLast.u8Rev640Date5)|(CanRev640InfoLast.u8Rev640Date6 << 8)|(CanRev640InfoLast.u8Rev640Date7<<16));
	
//	if(0x40 != RevCmd)
//	{
//		sgu16LimitFlg |= APPWRITE_LIMIT;   
//	}
	
	switch(RevAddress)
	{
		case 0x1F52:
			switch(RevCmd)
			{
				case 0x40:
					switch(RevCs)
					{
						case 0x01:
							vGetRandPwd(&HangChaPdo, 4);
							FuncCode = 0x43;
							Can5C0Send.u8Data[0] = FuncCode;
							Can5C0Send.u8Data[1] = RevAddress;
							Can5C0Send.u8Data[2] = RevAddress >> 8;
							Can5C0Send.u8Data[3] = RevCs;
							Can5C0Send.u8Data[4] = HangChaPdo.RandPwdArr[0];
							Can5C0Send.u8Data[5] = HangChaPdo.RandPwdArr[1];
							Can5C0Send.u8Data[6] = HangChaPdo.RandPwdArr[2];
							Can5C0Send.u8Data[7] = HangChaPdo.RandPwdArr[3];
							i32CanWrite(Can0, &Can5C0Send);
							break;
						case 0x04:
						case 0x05:
							FuncCode = 0x43;					
							Can5C0Send.u8Data[0] = FuncCode;
							Can5C0Send.u8Data[1] = RevAddress;
							Can5C0Send.u8Data[2] = RevAddress >> 8;
							Can5C0Send.u8Data[3] = RevCs;
							Can5C0Send.u8Data[4] = 0x00;
							Can5C0Send.u8Data[5] = 0x00;
							Can5C0Send.u8Data[6] = 0x00;
							Can5C0Send.u8Data[7] = 0x00;
							i32CanWrite(Can0, &Can5C0Send);
							break;
						case 0x06:
							FuncCode = 0x43;			
							Can5C0Send.u8Data[0] = FuncCode;
							Can5C0Send.u8Data[1] = RevAddress;
							Can5C0Send.u8Data[2] = RevAddress >> 8;
							Can5C0Send.u8Data[3] = RevCs;
							Can5C0Send.u8Data[4] = 0x01;
							Can5C0Send.u8Data[5] = 0x00;
							Can5C0Send.u8Data[6] = 0x00;
							Can5C0Send.u8Data[7] = 0x01;
							i32CanWrite(Can0, &Can5C0Send);
							break;
						default:
						break;
					}
					break;
				case 0x23:
				case 0x2B:
				case 0x2F:
					switch(RevCs)
					{
						case 0x02:
							HangChaPdo.DgnDataRxTx.DataL16 = CanRev640InfoLast.u8Data[4]|(CanRev640InfoLast.u8Data[5] << 8);
							HangChaPdo.DgnDataRxTx.DataH16 = CanRev640InfoLast.u8Data[6]|(CanRev640InfoLast.u8Data[7] << 8);
							vGetReplaceChar(HangChaPdo.RandPwdArr,ReplaceChar1);  //源字符替换
							vGenerateKeyPro(ReplaceChar1,ParseKeyFour);		//加密过程处理
							if(1 == iRemoteUnlockPWD(&HangChaPdo,ParseKeyFour))  //对比APP解析的密码
							{
								FuncCode = 0x60;		
							}
							else
							{
								FuncCode = 0x80;
							}
							Can5C0Send.u8Data[0] = FuncCode;
							Can5C0Send.u8Data[1] = RevAddress;
							Can5C0Send.u8Data[2] = RevAddress >> 8;
							Can5C0Send.u8Data[3] = RevCs;
							Can5C0Send.u8Data[4] = HangChaPdo.RandPwdArr[0];
							Can5C0Send.u8Data[5] = HangChaPdo.RandPwdArr[1];
							Can5C0Send.u8Data[6] = HangChaPdo.RandPwdArr[2];
							Can5C0Send.u8Data[7] = HangChaPdo.RandPwdArr[3];
							i32CanWrite(Can0, &Can5C0Send);
							break;
						case 0x03:
							FuncCode = 0x60;
							Can5C0Send.u8Data[0] = FuncCode;
							Can5C0Send.u8Data[1] = RevAddress;
							Can5C0Send.u8Data[2] = RevAddress >> 8;
							Can5C0Send.u8Data[3] = RevCs;
							Can5C0Send.u8Data[4] = 0x00;
							Can5C0Send.u8Data[5] = 0x00;
							Can5C0Send.u8Data[6] = 0x00;
							Can5C0Send.u8Data[7] = 0x00;
							i32CanWrite(Can0, &Can5C0Send);
							break;
						case 0x04:
							break;
						default:
							break;
					}
					break;
				default:
					break;
				}
				break;
		case 0x1018:
			switch(RevCmd)
			{
				case 0x40:
					switch(RevCs)
					{
						case 0x00:
							FuncCode = 0x43;
							Can5C0Send.u8Data[0] = FuncCode;
							Can5C0Send.u8Data[1] = RevAddress;
							Can5C0Send.u8Data[2] = RevAddress >> 8;
							Can5C0Send.u8Data[3] = RevCs;
							Can5C0Send.u8Data[4] = tempword[0];
							Can5C0Send.u8Data[5] = tempword[1];
							Can5C0Send.u8Data[6] = tempword[2];
							Can5C0Send.u8Data[7] = tempword[3];
							i32CanWrite(Can0, &Can5C0Send);
							break;
						case 0x01:
						case 0x04:
						case 0x05:
						case 0x06:
						case 0x07:
						case 0x08:
							FuncCode = 0x43;
							Can5C0Send.u8Data[0] = FuncCode;
							Can5C0Send.u8Data[1] = RevAddress;
							Can5C0Send.u8Data[2] = RevAddress >> 8;
							Can5C0Send.u8Data[3] = RevCs;
							Can5C0Send.u8Data[4] = 0x00;
							Can5C0Send.u8Data[5] = 0x00;
							Can5C0Send.u8Data[6] = 0x00;
							Can5C0Send.u8Data[7] = 0x00;
							i32CanWrite(Can0, &Can5C0Send);
							break;
						case 0x02:   
						case 0x03:							//软件版本
							FuncCode = 0x43;
							Can5C0Send.u8Data[0] = FuncCode;
							Can5C0Send.u8Data[1] = RevAddress;
							Can5C0Send.u8Data[2] = RevAddress >> 8;
							Can5C0Send.u8Data[3] = RevCs;
							Can5C0Send.u8Data[4] = 0x01&0xFF;
							Can5C0Send.u8Data[5] = 0x01&0xFF;
							Can5C0Send.u8Data[6] = 0x00;
							Can5C0Send.u8Data[7] = 0x00;
							i32CanWrite(Can0, &Can5C0Send);
							break;
						case 0x09:
						case 0x0A:
						case 0x0B:
							FuncCode = 0x43;
							Can5C0Send.u8Data[0] = FuncCode;
							Can5C0Send.u8Data[1] = RevAddress;
							Can5C0Send.u8Data[2] = RevAddress >> 8;
							Can5C0Send.u8Data[3] = RevCs;
							Can5C0Send.u8Data[4] = 0x00;
							Can5C0Send.u8Data[5] = 0x00;
							Can5C0Send.u8Data[6] = 0x00;
							Can5C0Send.u8Data[7] = 0x00;
							i32CanWrite(Can0, &Can5C0Send);
							break;
						case 0x0C:
							FuncCode = 0x43;
							Can5C0Send.u8Data[0] = FuncCode;
							Can5C0Send.u8Data[1] = RevAddress;
							Can5C0Send.u8Data[2] = RevAddress >> 8;
							Can5C0Send.u8Data[3] = RevCs;
							Can5C0Send.u8Data[4] = tempword[4];
							Can5C0Send.u8Data[5] = tempword[5];
							Can5C0Send.u8Data[6] = tempword[6];
							Can5C0Send.u8Data[7] = tempword[7];
							i32CanWrite(Can0, &Can5C0Send);
							break;
						default:
							break;
					}
					break;
				case 0x23:
				case 0x2B:
				case 0x2F:
					switch(RevCs)
					{
						case 0x00:
							break;
						default:
							break;
					}
					break;
				default:
					break;
				
			}
			break;
		case 0x400D:
			switch(RevCs)
					{
						case 0x04:
							GetParaFromEcu(RevCmd,u32RecieveAdress);
							break;
						default:
							vGetMcuPareFLocal(RevCmd,u32RecieveAdress);
							break;
						}
			break;
		case 0x400C:
		case 0x200B:
		case 0x200C:
		case 0x200D:
		case 0x200F:
		case 0x2019:
		case 0x2020:
		case 0x201F:
		case 0x2021:
		case 0x2022:
		case 0x2028:
		case 0x2029:
		case 0x202E:
		case 0x2040:
		case 0x2041:
		case 0x2042:
		case 0x2043:
				vGetMcuPareFLocal(RevCmd,u32RecieveAdress);
			break;
		case 0x202B:
			switch(RevCs)
					{
						case 0x09:
						case 0x19:
						case 0x3F:
						case 0x40:
								GetParaFromEcu(RevCmd,u32RecieveAdress);
							break;
						default:
								vGetMcuPareFLocal(RevCmd,u32RecieveAdress);
							break;
					}
			
			break;
					
		case 0x2027:
				GetParaFromEcu(RevCmd,u32RecieveAdress);
			break;
					
		case 0x201E:
			switch(RevCs)
					{
						case 0x13:
						case 0x14:
						case 0x17:
						case 0x18:
						case 0x19:
									vGetMcuPareFLocal(RevCmd,u32RecieveAdress);
							break;
						default:
								GetParaFromEcu(RevCmd,u32RecieveAdress);
							break;
					}
			break;
					
		case 0x200A:
			switch(RevCs)
					{
						case 0x53:
						case 0x54:
						case 0x56:
						case 0x57:
						case 0x5B:
								vGetMcuPareFLocal(RevCmd,u32RecieveAdress);
							break;
						case 0x0C:
							GetParaFromEcu(RevCmd,u32RecieveAdress);
							break;
						default:
							break;
					}
			break;
					
		case 0x201C:
			switch(RevCs)
					{
						case 0x01:
						case 0x02:
						case 0x4B:
						case 0x57:
						case 0x4A:
						case 0x56:
								vGetMcuPareFLocal(RevCmd,u32RecieveAdress);
							break;
						default:
							GetParaFromEcu(RevCmd,u32RecieveAdress);
							break;
					}
			break;
					
			case 0x4001:
				switch(RevCs)
						{
							case 0x11:
							case 0x12:
							case 0x13:
								vGetMcuPareFLocal(RevCmd,u32RecieveAdress);
							break;
						case 0x01:
						case 0x02:
						case 0x03:
						case 0x04:
						case 0x05:
						case 0x06:
						case 0x07:
						case 0x08:
						case 0x09:
						case 0x0A:
						case 0x0B:
						case 0x0C:
						case 0x0D:
						case 0x0E:
						case 0x0F:
						case 0x10:
							GetParaFromEcu(RevCmd,u32RecieveAdress);
							break;
						default:
							break;
					}
				break;
					
			case 0x2026:
				switch(RevCs)
						{
							case 0x05:
									vGetMcuPareFLocal(RevCmd,u32RecieveAdress);
							break;
						case 0x01:
						case 0x02:
						case 0x03:
						case 0x04:
							GetParaFromEcu(RevCmd,u32RecieveAdress);
							break;
						default:
							break;
					}
				break;
					
			case 0x4003:
				switch(RevCs)
						{
							case 0x0E:
							case 0x0F:
							case 0x10:
							case 0x11:
							case 0x12:
							case 0x13:
							case 0x14:
								vGetMcuPareFLocal(RevCmd,u32RecieveAdress);
							break;
						default:
							GetParaFromEcu(RevCmd,u32RecieveAdress);
							break;
						}
				break;
					
			case 0x4002:
			case 0x4004:
			case 0x4009:
			case 0x400B:
			case 0x400E:
			case 0x2004:
			case 0x2005:
			case 0x2008:
			case 0x2009:
			case 0x2006:
				GetParaFromEcu(RevCmd,u32RecieveAdress);
				break;
			case 0x202D:
				switch(RevCs)
				{
					case 0x01:
						ChangeHourPassWd = (uint32_t)((CanRev640InfoLast.u8Rev640Date5)|(CanRev640InfoLast.u8Rev640Date6 << 8)|(CanRev640InfoLast.u8Rev640Date7<<16));
						if(56923 == ChangeHourPassWd)
						{
							PassWdOk = 1;
							FuncCode = 0x60;
							Can5C0Send.u8Data[0] = FuncCode;
							Can5C0Send.u8Data[1] = RevAddress;
							Can5C0Send.u8Data[2] = RevAddress >> 8;
							Can5C0Send.u8Data[3] = RevCs;
							Can5C0Send.u8Data[4] = 0x00;
							Can5C0Send.u8Data[5] = 0x00;
							Can5C0Send.u8Data[6] = 0x00;
							Can5C0Send.u8Data[7] = 0x00;
							i32CanWrite(Can0, &Can5C0Send);
						}
						else
						{
							PassWdOk = 0;
							FuncCode = 0x80;
							Can5C0Send.u8Data[0] = FuncCode;
							Can5C0Send.u8Data[1] = RevAddress;
							Can5C0Send.u8Data[2] = RevAddress >> 8;
							Can5C0Send.u8Data[3] = RevCs;
							Can5C0Send.u8Data[4] = 0x00;
							Can5C0Send.u8Data[5] = 0x00;
							Can5C0Send.u8Data[6] = 0x00;
							Can5C0Send.u8Data[7] = 0x00;
							i32CanWrite(Can0, &Can5C0Send);
						}
						break;
					case 0x02:
							if(1 == PassWdOk)
							{
								WorkRestFlg = 1;
								WorkRest = (uint32_t)((CanRev640InfoLast.u8Rev640Date5)|(CanRev640InfoLast.u8Rev640Date6 << 8)|(CanRev640InfoLast.u8Rev640Date7<<16)|(CanRev640InfoLast.u8Rev640Date8<<24));
								Can5C0Send.u8Data[0] = 0x60;
								Can5C0Send.u8Data[1] = RevAddress;
								Can5C0Send.u8Data[2] = RevAddress >> 8;
								Can5C0Send.u8Data[3] = RevCs;
								Can5C0Send.u8Data[4] = 0x00;
								Can5C0Send.u8Data[5] = 0x00;
								Can5C0Send.u8Data[6] = 0x00;
								Can5C0Send.u8Data[7] = 0x00;
								i32CanWrite(Can0, &Can5C0Send);
							}
							else
							{
								FuncCode = 0x80;
								Can5C0Send.u8Data[0] = FuncCode;
								Can5C0Send.u8Data[1] = RevAddress;
								Can5C0Send.u8Data[2] = RevAddress >> 8;
								Can5C0Send.u8Data[3] = RevCs;
								Can5C0Send.u8Data[4] = 0x00;
								Can5C0Send.u8Data[5] = 0x00;
								Can5C0Send.u8Data[6] = 0x00;
								Can5C0Send.u8Data[7] = 0x00;
								i32CanWrite(Can0, &Can5C0Send);
							}
						break;
					case 0x03:
						if(1 == PassWdOk)
						{
							HourRestFlg = 1;
							HourRest = (uint32_t)((CanRev640InfoLast.u8Rev640Date5)|(CanRev640InfoLast.u8Rev640Date6 << 8)|(CanRev640InfoLast.u8Rev640Date7<<16)|(CanRev640InfoLast.u8Rev640Date8<<24));
							Can5C0Send.u8Data[0] = 0x60;
							Can5C0Send.u8Data[1] = RevAddress;
							Can5C0Send.u8Data[2] = RevAddress >> 8;
							Can5C0Send.u8Data[3] = RevCs;
							Can5C0Send.u8Data[4] = 0x00;
							Can5C0Send.u8Data[5] = 0x00;
							Can5C0Send.u8Data[6] = 0x00;
							Can5C0Send.u8Data[7] = 0x00;
							i32CanWrite(Can0, &Can5C0Send);
						}
						else
						{
							FuncCode = 0x80;
							Can5C0Send.u8Data[0] = FuncCode;
							Can5C0Send.u8Data[1] = RevAddress;
							Can5C0Send.u8Data[2] = RevAddress >> 8;
							Can5C0Send.u8Data[3] = RevCs;
							Can5C0Send.u8Data[4] = 0x00;
							Can5C0Send.u8Data[5] = 0x00;
							Can5C0Send.u8Data[6] = 0x00;
							Can5C0Send.u8Data[7] = 0x00;
							i32CanWrite(Can0, &Can5C0Send);
						}
						break;
				}
				break;
		case 0xFF10:
			switch(RevCs)
				{
					case 0x01:
						Count = PARA_ErrCode0;
						RevErrCCount = 0;
						RevErrCodeAddr = 1;
						ErrCodeNowSend = 1;
						HangChaPdo.ErrcodeType = 1;
						HangChaPdo.SubFunID = 0xFF;
						HangChaPdo.SendErrFlag = true;
						break;
					case 0x02:
 						RevErrCCount++;
						if(RevErrCCount == 1)
						{
							RevErrCodeAddr = 1;
							ErrCodeNowSend = 1;
							Count = PARA_ErrCode0;
							HangChaPdo.ErrcodeType = 2;
							HangChaPdo.SubFunID = 0xFF;
							HangChaPdo.SendErrFlag = true;
						}
						else
						{			
							Count = PARA_ErrCodeMax;
							HangChaPdo.ErrcodeType = 0;
							HangChaPdo.SendErrFlag = false;
						}
						break;
				}
			break;
				
			case 0xFF11:
				RevErrCCount = 0;
				Count = PARA_ErrCode0;
				ErrCodeNowSend = 0;
			break;						
		default:
			break;
	}
	memset(CanRev640InfoLast.u8Data,0,sizeof(CanRev640InfoLast));
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
	CanRev2F0InfoLast.BMS_SOC = 100;
	
	vDoPwmErrReg(vDoPwmErrCallBack);
	vPropErrReg(vPropErrCallBack);
	vAiErrReg(vAiErrCallBack);
	vMcuParaRevRegister(vMcuParaRevProc);
	
	
	xRevCallBackProc CanId670 = {.u32CanId = 0x670, .u32Data = 0, .CallBack = vCanId670Proc};
	xRevCallBackProc CanId640 = {.u32CanId = 0x640, .u32Data = 0, .CallBack = vCanId640Proc};
	
	
	vCanRevMsgRegister(&CanId670);
	vCanRevMsgRegister(&CanId640);

	memset(&sgValvesInfo, 0, sizeof(sgValvesInfo));
	
	sgUserInfo.fPropMinCurrent = i32GetPara(PROP_MIN_CURRENT) / 1000.0;		/*比例阀最小电流*/
	sgUserInfo.fPropMaxCurrent = i32GetPara(PROP_MAX_CURRENT) / 1000.0;		/*比例阀最大电流*/
	
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
	sgUserInfo.u16ThrottleMid = i32GetPara(MOVE_THROTTLE_MID) * MOTOR_SPEED_RANGE / 100;
	sgUserInfo.u16ThrottleMidValue = (sgUserInfo.u16ThrottleMax + sgUserInfo.u16ThrottleMin) >> 1;

	sgUserInfo.u16LiftUpMin = i32GetPara(LIFT_UP_THROTTLE_MIN) * 100;
	sgUserInfo.u16LiftUpMax = i32GetPara(LIFT_UP_THROTTLE_MAX) * 100;
	sgUserInfo.u16LiftUpRange = (sgUserInfo.u16LiftUpMax - sgUserInfo.u16LiftUpMin) >> 1;
	sgUserInfo.u16LiftUpMid = i32GetPara(LIFT_UP_THROTTLE_MID) * PUMP_RANGE / 100;
	sgUserInfo.u16LiftUpMidValue = (sgUserInfo.u16LiftUpMax + sgUserInfo.u16LiftUpMin) >> 1;

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
	
	sgUserInfo.u8BatteryType = i32GetPara(PARA_BatteryType);
	steerAngleTab = i32GetPara(PARA_PumpMotorGear1);
	
	sgUserInfo.MostLiftUpTim = i32GetPara(PARA_AngleValue0);
	sgUserInfo.MostQianQinTim = i32GetPara(PARA_AngleValue1);
	sgUserInfo.MostHouQinTim = i32GetPara(PARA_AngleValue2);
	
	sgUserInfo.VehicleType = i32GetPara(PARA_VehicleType);
	
	sgUserInfo.u16LiftShortTim = i32GetPara(PARA_AngleValue3);   // 0.1S   1000
	sgUserInfo.u16LiftShortHold = i32GetPara(PARA_AngleValue4);		// 0.1S    1
	
	sgUserInfo.u8SOCLimit = i32GetPara(PARA_AccAndDecTurn);    //电量故障模拟
	sgUserInfo.Temputer = i32GetPara(PARA_AccAndDecAntiPinch);   //温度故障模拟
	sgUserInfo.u8Language = i32GetPara(PARA_LanguageType);
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
	HourCount8L = u32HourCount;
	HourCount16H = u32HourCount >> 8;
	u16EepromRead(PARA_WorkCountL, &WorkCountL, 1);
	u16EepromRead(PARA_WorkCountH, &WorkCountH, 1);
	u32WorkCount = (WorkCountH<<16)|WorkCountL;
	WorkCount8L = u32WorkCount;
	WorkCount16H = u32WorkCount>> 8;
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
	gCanSendPdoInfo.CanSend260Info.u8HourCountL = u32HourCount & 0xFF;
	gCanSendPdoInfo.CanSend260Info.u8HourCountH = (u32HourCount >> 8) & 0xFF;
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
	
	vCanIdLostReg(0x1E0,1000,vCanLostProc);
	vCanIdLostReg(0x2F0,1000,vCanLostProc);
	vCanIdLostReg(0x360,1000,vCanLostProc);

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
//	static uint32_t shduhfiau = 0;
	uint16_t ZhiZhenAddre;
	void * MotorVal;
	MotorVal = &ZhiZhenAddre;
	static uint16_t ErrCodeChkCnt = 0;
	
	uint8_t u8Vector1[8];

	if(true == u8GetNetTimerOverFlag(TIMER_EcuPowerOn))
	{
		u8EcuProcFlag = 1;
		vKillNetTimer(TIMER_EcuPowerOn);
	}
	if(u8MainConnectCnt < 200)
	{
		if(1 == i32LocalDiGet(EmergencyReverse))
		{
			u8EmsErrFlg = 1;
			check_err = 1;
			i32ErrCodeSet(39);
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
			if(0 == i32LocalDiGet(EmergencyReverse)
			)
			{
				u8EmsErrFlg = 0;
				check_err = 0;
				i32ErrCodeClr(39);
				sgValvesInfo.b1NoActFlag = 0;
			}
		}
	}
	
	if(1 == u8EcuProcFlag)
	{
		vCanSendid2F8Tmp();
		vSwiMonitor();		
		vCanRevPdoProc();
		vAiMonitor();
		if(0 != u8ErrCodeGet())
		{
			for(ErrCodeChkCnt = 0;ErrCodeChkCnt < ErrCode200;ErrCodeChkCnt++)
			{
				if(0 != i32ErrCodeCheck(ErrCodeChkCnt))
				{
					u8ErrCodeCir[ErrCodeChkCnt] = ErrCodeChkCnt+1;;
				}
				else
				{
					u8ErrCodeCir[ErrCodeChkCnt] = 0;
				}
			}
		}
		vCanSendid258Tmp();
		
		if(((BSdoState.Address == 0xFF10)||(1 == ErrCodeNowSend))&&(RevErrCodeAddr == 1))
		{
			ErrCodeSend(BSdoState.SdoCs);       //发送一次当前故障
		}
		
		if(1 == SendHistoryErrCode)
		{
//			if(shduhfiau > 1000)
			{
				SendHistoryErrCode = 0;
				RevErrCCount = 0;
			}
//			else
//			{
//				shduhfiau++;
//			}
		}
		else
		{
//			shduhfiau = 0;
		}
			
		u8ErrCode = u8ErrCodeGet();
		if (0 != u8ErrCode)
		{
			vLedSendAlmCode(u8ErrCode);
		}
	}
	if(ParaCount < 250)
	{

		u8Vector1[0] = 0x40;
		u8Vector1[1] = ParaCount & 0xFF;
		u8Vector1[2] = (ParaCount >> 8) & 0xFF;
		u8Vector1[3] = 0 & 0xFF;
		u8Vector1[4] = 0 & 0xFF;
		u8Vector1[5] = 0 & 0xFF;
		u8Vector1[6] = 0 & 0xFF;
		u8Vector1[7] = 0 & 0xFF;
		
		vQueryMcuPara(u8Vector1,8);
		if(1 == MstParaIsOk)
		{
			MstParaIsOk = 0;
			ParaCount++;
//			SendOk = 1;
//			SendCount = 0;
		}	
	//	SendCount++;
	//		if((SendCount > 2)&&(1 == MstParaIsOk))
	
	}
	else
	{
		PowerOnGetPara = false;
	}
			
	if(0 !=(BSdoState.SdoFlg&Read_Cmd))
	{
		BSdoState.SdoFlg &= ~Read_Cmd;
		BSdoState.SdoFlg |= ReadOk_Cmd;
		if(u16SendAdress > 200)
		{
			MotorVal = u16pGetParaPoint(u16SendAdress);
			BSdoState.LocalData = *(uint16_t *)MotorVal;
			
		}
		else
			u16EepromRead(u16SendAdress,&BSdoState.LocalData,1);
	}
	
	if(0 !=(BSdoState.SdoFlg&Write_Cmd))
	{
		BSdoState.SdoFlg &= ~Write_Cmd;
		BSdoState.SdoFlg |= WriteOk_Cmd;
		u16EepromRead(u16SendAdress,&BSdoState.LocalData,1);
		if((BSdoState.LocalData) != BSdoState.SaveData)
		{
			tmpData.u32data = BSdoState.LocalData;
			BSdoState.SaveData = TexDateManage(BSdoState.SaveData);
			if(0 != sgMcuState.b1ServonState)     //Sevon下不允许修改参数
				BSdoState.WriteReturnFlg = WRITE_FAIL ;	
			else
				BSdoState.WriteReturnFlg = u16EepromWrite(u16SendAdress, BSdoState.SaveData, 1);
		}
	}
	
	if((0 != (BSdoState.SdoFlg&WriteOk_Cmd))||(0 != (BSdoState.SdoFlg&ReadOk_Cmd)))
	{
		if(0 !=(BSdoState.SdoFlg&ReadOk_Cmd))
		{
			BSdoState.LocalData = RevDateManage(BSdoState.LocalData);
			BSdoState.LocalData = ((BSdoState.LocalData + B_XISHU[u16Factor]) * K_CHU_XISHU[u16Factor]) / K_XISHU[u16Factor];
			tCanFrame Can5C0Send = {.u32ID = 0x5C0, .u16DataLength = 8, .u8Data = {0x00}, .u8Rtr = 0};
			Can5C0Send.u8Data[0] = 0x43;
			Can5C0Send.u8Data[1] = BSdoState.Address;
			Can5C0Send.u8Data[2] = BSdoState.Address>>8;
			Can5C0Send.u8Data[3] = BSdoState.SdoCs;
			Can5C0Send.u8Data[4] = BSdoState.LocalData;
			Can5C0Send.u8Data[5] = BSdoState.LocalData>>8;
			i32CanWrite(Can0, &Can5C0Send);
		}
		else if(0 !=(BSdoState.SdoFlg&WriteOk_Cmd))
		{
				tCanFrame Can5C0Send = {.u32ID = 0x5C0, .u16DataLength = 8, .u8Data = {0x00}, .u8Rtr = 0};
				if(0 == BSdoState.WriteReturnFlg)
					Can5C0Send.u8Data[0] = 0x60;
				else
					Can5C0Send.u8Data[0] = 0x80;
				Can5C0Send.u8Data[1] = BSdoState.Address;
				Can5C0Send.u8Data[2] = BSdoState.Address>>8;
				Can5C0Send.u8Data[3] = BSdoState.SdoCs;
				Can5C0Send.u8Data[4] = BSdoState.SaveData;
				Can5C0Send.u8Data[5] = BSdoState.SaveData>>8;
				i32CanWrite(Can0, &Can5C0Send);
		}
		memset(&BSdoState,0,sizeof(BSdoState));
	}

	if ((true == u8GetNetTimerOverFlag(TIMER_HourCount))&&((1 == sgUserInfo.HourCountMode)||(3 == sgUserInfo.HourCountMode)))
	{
		HourCntFlg = 1;
		vResetNetTimer(TIMER_HourCount);
		u16SecCnt++;
		if (u16SecCnt >= 360)			/*360*/ 
		{
			u16SecCnt = 0;
			HourCount8L++;
			if(HourCount8L > 99)
			{
				HourCount8L = 0;
				HourCount16H++;
			}
			u32HourCount = (HourCount16H << 8)|HourCount8L;
			vHourCountWrite(u32HourCount);
		}
	}
	
	if((0 != WorkRestFlg)||(0 != sgUserInfo.b1HourConutMode))
	{
			WorkRest = WorkRest * 10;
	  	WorkCount8L = WorkRest%100;
		  WorkCount16H = WorkRest/100;
			u32WorkCount = (WorkCount16H << 8)|WorkCount8L;
			WorkCountL = u32WorkCount;
			WorkCountH = u32WorkCount >> 16;
			u16EepromWrite(PARA_WorkCountL, WorkCountL,1);
			u16EepromWrite(PARA_WorkCountH, WorkCountH,1);
			WorkRestFlg= 0;
			WorkRest = 0;
	}
	if((0 != HourRestFlg)||(0 != sgUserInfo.b1HourConutMode))
	{
			HourRest = HourRest * 10;
			HourCount8L = HourRest%100;
		  HourCount16H = HourRest/100;
			u32HourCount = (HourCount16H << 8)|HourCount8L;
			vHourCountWrite(u32HourCount);
			HourRestFlg = 0;
			HourRest = 0;
	}
	if(0 != sgUserInfo.b1HourConutMode)
	{
		sgUserInfo.b1HourConutMode = 0;
	}
	i32DoPwmSet(DRIVER3,DRIVER_OPEN);
	Work2App = WorkCount16H*100+WorkCount8L;
	Hour2App = HourCount16H*100+HourCount8L;
	
	vWdgSetFun(WDG_USER_BIT);
}
#endif
