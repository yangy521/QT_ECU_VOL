/*******************************************************************************
* Filename: UserStackerTruckProc.c	                                           *
* Description: 逻辑层的源文件，包含对应的回调函数	   						   *
* Author:                                                      *
* Date: 2023/05/30    														   *
* Revision:	 														 		   *
*******************************************************************************/
#include "UserCanComm.h"
#include "USER_HANGUO_DBC.h"
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

#if (USER_TYPE == USER_HANGUO_JINJIDAO_DBC)

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
		{.u8Flag = 1, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 1, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 1, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 1, .u8Type = 0x00, .u16Period = 50},
	},
	.RpdoPara = {
		{.b1Flag = 1, .b11CanRevId = 0x303},
		{.b1Flag = 1, .b11CanRevId = 0x500},
		{.b1Flag = 1, .b11CanRevId = 0x501},
		{.b1Flag = 0, .b11CanRevId = 0x000},
	},
};

#define BmsLost_Limit 					(1 << 0)
#define HandleLost_Limit 				(1 << 1)
#define POWERON_CHECK_LIMIT 		(1 << 2)

typedef union
{
	uint8_t u8data;
	struct
	{
		uint8_t b1Brake: 1;        
		uint8_t b1SafeLock: 1;               //互锁
		uint8_t b1Throttle: 1;
		uint8_t b7Reserve: 6;
	};
}xSwiInput;

typedef union
{
	uint16_t u16data;
	struct
	{
		uint16_t b1Reserve1: 1;
		uint16_t b1Reserve2: 1;
		uint16_t b1Reserve3: 1;
		uint16_t b1Reserve4: 1;
		uint16_t b1SpeedMode: 1;
		uint16_t b1Forward: 1;
		uint16_t b1Backward: 1;
		uint16_t b1Reserve8: 1;
		
		uint16_t b1Reserve11: 1;
		uint16_t b1Reserve12: 1;
		uint16_t b1Reserve13: 1;
		uint16_t b1Reserve14: 1;
		uint16_t b1Reserve15: 1;
		uint16_t b1Reserve16: 1;
		uint16_t b1EMB: 1;
		uint16_t b1Break: 1;		
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
	
	uint16_t	u16RatioOfTransmission;
	uint16_t	u16MotorMaxSpd;
	
	uint16_t	u16RentalTime;
	
	uint16_t u16BrakePedalType;
	
	uint16_t	u16BrakePedalRangeL;
	uint16_t	u16BrakePedalRangeH;
	
	uint16_t u16QianHouQinSpeed;
	uint16_t u16QianHouYiSpeed;
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
static int16_t i16SteerAngle = 0;
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

static uint32_t MotorOdm = 0;
static uint32_t MotorOdmClc = 0;

/*PDO接收待完善*/
xCanRevPdoInfo gCanRevPdoInfo; 
static xCanRev303Info CanRev303InfoLast;
static xCanRev500Info CanRev500InfoLast;
static xCanRev501Info CanRev501InfoLast;
/*PDO发送待完善*/
xCanSendPdoInfo gCanSendPdoInfo;
static xCanSend101Info CanSend101Last;
static xCanSend102Info CanSend102Last;
static xCanSend103Info CanSend103Last;
static xCanSend104Info CanSend104Last;

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
	CanSend102Last.u8Data[2] = i16MotorSpd & 0xFF;
	CanSend102Last.u8Data[3] = (i16MotorSpd>>8) & 0xFF;
	CanSend101Last.u8Data[5] = tmp/10;
	__enable_irq();
	tmp = RevData->u8MotorTmp;
	__disable_irq();
	CanSend102Last.u8Data[0] = ((tmp - 40)*10) & 0xFF;
	CanSend102Last.u8Data[1] = (((tmp - 40)*10)>>8) & 0xFF;
	__enable_irq();
	/*Motor Temp*/
	
	tmp = RevData->u8BoardTmp;
	__disable_irq();
	CanSend101Last.u8Data[6] = ((tmp - 40)*10) & 0xFF;
	CanSend101Last.u8Data[7] = (((tmp - 40)*10)>>8) & 0xFF;
	__enable_irq();
	/**/
	
	i16MotorCurrent = (RevData->u8CurrentHigh << 8) | RevData->u8CurrentLow;
	
	if(0 != RevData->u8Reserve3)
	{

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
	float fTmp = 0.0;
	xMstSendStat LastStatus;
	int16_t i16Spd = 0;
	int32_t i32PropValue = 0;
	static uint16_t u16LiftUpCnt = 0; 
	static uint8_t u8EmsDelay = 0;
	static uint16_t u16PropStartDelay = 0;
	static uint16_t u16PropStopDelay = 4096;
	static uint8_t u8PropTmp = 0;
	static uint16_t EmsCountTime = 0;

	LastStatus.u8Data = SendData->buf[2];
	SendData->buf[2] = 0;

	/*Move Mode*/
		SendData->b1ServoOn = 1;
		if(sgHandleInput.b1Backward == 1)
		{
			CanSend101Last.b1BackWord = 1;
			CanSend101Last.b1ForWord = 0;
			CanSend101Last.b1NWord = 0;
			SendData->b1ForwardReq = 1;
		}
		else if(sgHandleInput.b1Forward == 1)
		{
			SendData->b1BackwardReq = 1;
			CanSend101Last.b1BackWord = 0;
			CanSend101Last.b1ForWord = 1;
			CanSend101Last.b1NWord = 0;
		}
		else
		{
			CanSend101Last.b1BackWord = 0;
			CanSend101Last.b1ForWord = 0;
			CanSend101Last.b1NWord = 1;
		}
		
		if(((0 != sgHandleInput.b1Backward)||(0 != sgHandleInput.b1Forward))&&(0 != sgSwiInput.b1Throttle))
		{
			SendData->u8TargetHigh = u16MotorVal >> 8;
			SendData->u8TargetLow = u16MotorVal;
		}
		else
		{
			SendData->u8TargetHigh = 0;
			SendData->u8TargetLow = 0;
		}
	
	if (0 == sgSwiInput.b1SafeLock)
	{
		SendData->b1PowerLineOn = 1;     // 1:断开 0:连接
		SendData->b1ServoOn = 0;
		SendData->b1EmsReq = 0;
		SendData->b1ForwardReq = 0;
		SendData->b1BackwardReq = 0;
	}
	
	if((0 != sgValvesInfo.u8NoAct)||(0 != sgHandleInput.b1Break))
	{
		SendData->b1ServoOn = 0;
		SendData->b1EmsReq = 0;
		SendData->b1ForwardReq = 0;
		SendData->b1BackwardReq = 0;
	}
}

static void vDoPwmErrCallBack(eDoPwmNo DoPwmNo)   //Err Limit Act
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
		case 0x500:
			if(CAN_NORMAL == u8State)
			{
					sgValvesInfo.u8NoAct &= ~BmsLost_Limit;
			}
			else if(CAN_LOST == u8State)
			{
					sgValvesInfo.u8NoAct |= BmsLost_Limit;		
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




/*接收PDO处理函数*/
static void vCanRevPdoProc(void)
{
	if(0 == sgUserInfo.u8BatteryType)
	{
		memcpy((char*)CanRev303InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo303.u8Data, sizeof(CanRev303InfoLast));
		/*添加相关操作*/
//		gCanSendPdoInfo.CanSend260Info.u8Soc = CanRev303InfoLast.u8BMSSOCL | (CanRev303InfoLast.u8BMSSOCH<<8);
	}
	else
	{
//		gCanSendPdoInfo.CanSend260Info.u8Soc = u8GetBatterySoc();
	}
	
	
	{
		{
			memcpy((char*)CanRev500InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo500.u8Data, sizeof(CanRev500InfoLast));
			/*添加相关操作*/
			sgHandleInput.u16data = CanRev500InfoLast.u8HandleInput1 | (CanRev500InfoLast.u8HandleInput2 << 8);
		}
		
		{
//			memcpy((char*)CanRev7F4InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo7F4.u8Data, sizeof(CanRev7F4InfoLast));
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
//		gCanSendPdoInfo.CanSend2A3Info.PumpVoltage = i32AdcValue /10;
		
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
	
}


/*开关量监控*/
static void vSwiMonitor(void)
{
	xSwiInput SwiInput = {0};
	
	if(1 == i32LocalDiGet(SAFELOCK_SWI))
	{
		SwiInput.b1SafeLock = 1;
	}
	if(1 == i32LocalDiGet(THROTTLE_SWI))
	{
		SwiInput.b1Throttle = 1;
		CanSend101Last.b1Throttle = 1;
	}
	else
	{
		CanSend101Last.b1Throttle = 0;
	}
	sgSwiInput.u8data = SwiInput.u8data;
	
	sgValvesInfo.b1Gear1SpdFlag = 0;  
	sgValvesInfo.b1Gear2SpdFlag = 0;
	sgValvesInfo.b1Gear3SpdFlag = 0;
	sgValvesInfo.b1Gear4SpdFlag = 0;
	
	if(1 == sgHandleInput.b1SpeedMode)
	{
		sgValvesInfo.b1Gear1SpdFlag = 1;  
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 0;
		sgValvesInfo.b1Gear4SpdFlag = 0;
	}

}

static uint8_t u8SwiInitChcek(void)
{
	uint8_t u8Res = 0;
		
	if(1 == i32LocalDiGet(SAFELOCK_SWI))
	{
//		u8Res = 1;
	}
	return u8Res;
}

static void vCanId303Proc(tCanFrame * CanFrame)
{
	
}

static void vCanSendDataPlc(void)
{
	CanSend101Last.u8Data[2] = ((*(uint8_t *)u16pGetParaPoint(PARA_Ksi))/10) & 0xFF;
	CanSend101Last.u8Data[3] = (((*(uint8_t *)u16pGetParaPoint(PARA_Ksi))/10)>>8) & 0xFF;
	
	CanSend101Last.u8Data[4] = (i32LocalAiGetValue(MOVE_THROTTLE)/100) & 0xFF;
	
	CanSend104Last.u8Data[4] = (i32LocalAiGetValue(MOVE_THROTTLE)/100) & 0xFF;
	CanSend104Last.u8Data[5] = ((i32LocalAiGetValue(MOVE_THROTTLE)/100)>>8) & 0xFF;
}

static void vCanIdPdoProc(void)
{
	tCanFrame canFrame;
	
	static uint8_t TxCnt = 0;
	
	if(TxCnt == 10/USER_ECU_PERIOD)
	{
		canFrame.u32ID = 0x101;
		canFrame.u16DataLength = 8;
		
		canFrame.u8Data[0] = CanSend101Last.u8Data[0];
		canFrame.u8Data[1] = CanSend101Last.u8Data[1];
		canFrame.u8Data[2] = CanSend101Last.u8Data[2];
		canFrame.u8Data[3] = CanSend101Last.u8Data[3];
		canFrame.u8Data[4] = CanSend101Last.u8Data[4];
		canFrame.u8Data[5] = CanSend101Last.u8Data[5];
		canFrame.u8Data[6] = CanSend101Last.u8Data[6];
		canFrame.u8Data[7] = CanSend101Last.u8Data[7];
		i32CanWrite(Can0, &canFrame);
	}
	else if(TxCnt == 20/USER_ECU_PERIOD)
	{
		canFrame.u32ID = 0x102;
		canFrame.u16DataLength = 8;
		
		canFrame.u8Data[0] = CanSend102Last.u8Data[0];
		canFrame.u8Data[1] = CanSend102Last.u8Data[1];
		canFrame.u8Data[2] = CanSend102Last.u8Data[2];
		canFrame.u8Data[3] = CanSend102Last.u8Data[3];
		canFrame.u8Data[4] = CanSend102Last.u8Data[4];
		canFrame.u8Data[5] = CanSend102Last.u8Data[5];
		canFrame.u8Data[6] = CanSend102Last.u8Data[6];
		canFrame.u8Data[7] = CanSend102Last.u8Data[7];
		i32CanWrite(Can0, &canFrame);
	}
	else if(TxCnt == 30/USER_ECU_PERIOD)
	{
		canFrame.u32ID = 0x103;
		canFrame.u16DataLength = 8;
		
		canFrame.u8Data[0] = CanSend103Last.u8Data[0];
		canFrame.u8Data[1] = CanSend103Last.u8Data[1];
		canFrame.u8Data[2] = CanSend103Last.u8Data[2];
		canFrame.u8Data[3] = CanSend103Last.u8Data[3];
		canFrame.u8Data[4] = CanSend103Last.u8Data[4];
		canFrame.u8Data[5] = CanSend103Last.u8Data[5];
		canFrame.u8Data[6] = CanSend103Last.u8Data[6];
		canFrame.u8Data[7] = CanSend103Last.u8Data[7];
		i32CanWrite(Can0, &canFrame);
	}
	else if(TxCnt == 40/USER_ECU_PERIOD)
	{
		canFrame.u32ID = 0x104;
		canFrame.u16DataLength = 8;
		
		canFrame.u8Data[0] = CanSend104Last.u8Data[0];
		canFrame.u8Data[1] = CanSend104Last.u8Data[1];
		canFrame.u8Data[2] = CanSend104Last.u8Data[2];
		canFrame.u8Data[3] = CanSend104Last.u8Data[3];
		canFrame.u8Data[4] = CanSend104Last.u8Data[4];
		canFrame.u8Data[5] = CanSend104Last.u8Data[5];
		canFrame.u8Data[6] = CanSend104Last.u8Data[6];
		canFrame.u8Data[7] = CanSend104Last.u8Data[7];
		i32CanWrite(Can0, &canFrame);
	}
	
	TxCnt +=1;
	if(TxCnt >= 100/USER_ECU_PERIOD)  // 报文周期 100ms
	{
		TxCnt = 0;
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
	//303
	uint16_t u16Tmp = 0;
	uint16_t WorkCountL = 0;
	uint16_t WorkCountH = 0;
	
	xRevCallBackProc CanId303 = {.u32CanId = 0x303, .u32Data = 0, .CallBack = vCanId303Proc};
	
	vCanRevMsgRegister(&CanId303);
	
	vMstRevRegister(vMstRevProc);
	vMstSendRegister(vMstSendProc);
	
	vDoPwmErrReg(vDoPwmErrCallBack);
	vPropErrReg(vPropErrCallBack);
	vAiErrReg(vAiErrCallBack);

	memset(&sgValvesInfo, 0, sizeof(sgValvesInfo));
	
	sgUserInfo.u8LeanForWardValue = i32GetPara(LEAN_FORWARD_PARA) * PUMP_RANGE / 100;		/*前倾参数*/
	sgUserInfo.u8LeanBackWardValue = i32GetPara(LEAN_BACKWARD_PARA) * PUMP_RANGE / 100;		/*后倾参数*/
	
	sgUserInfo.u16Gear1Spd = i32GetPara(PARA_Gear1Spd);	/*1档速度*/
	sgUserInfo.u16Gear2Spd = i32GetPara(PARA_Gear2Spd);	/*2档速度*/
	sgUserInfo.u16Gear3Spd = i32GetPara(PARA_Gear3Spd);	/*3档速度*/
	sgUserInfo.u16Gear4Spd = i32GetPara(PARA_Gear4Spd);	/*4档速度*/
	
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
													 
	sgUserInfo.u16RatioOfTransmission = i32GetPara(PARA_RatioOfTransmission);
//	sgUserInfo.u16MotorMaxSpd = i32GetPara(PARA_MotorMaxSpd);
		sgUserInfo.u16BrakePedalType = i32GetPara(BRAKE_THROTTLE_TYPE);
	
	sgUserInfo.u8BatteryType = i32GetPara(PARA_BatteryType);
	steerAngleTab = i32GetPara(PARA_PumpMotorGear1);
	
	sgUserInfo.u16QianHouQinSpeed = i32GetPara(PARA_PumpMotorGear1);
	sgUserInfo.u16QianHouYiSpeed = i32GetPara(PARA_PumpMotorGear2);
	
	/*Para Initial*/
	
	vCanIdLostReg(0x500,1000,vCanLostProc);

	vSetPdoPara(sgPdoPara);
	
	i32LocalDoSet(DO_ENCODER2, 1);
	
	vSetNetTimer(TIMER_EcuPowerOn, ECU_POWERON_DELAY_TIME);	/**/
	
	vSetNetTimer(TIMER_HourCount, HOURCOUNT_PERIOD);
	
	CanSend101Last.b1Module = 0;
	CanSend101Last.b1PowerOn = 1;
	CanSend102Last.u8Data[4] = SWVIERSION & 0xFF;
	CanSend102Last.u8Data[5] = (SWVIERSION>>8) & 0xFF;
	CanSend102Last.u8Data[6] = HWVIERSION & 0xFF;
	CanSend102Last.u8Data[7] = (HWVIERSION>>8) & 0xFF;
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
		vCanSendDataPlc();
		vCanIdPdoProc();
		u8ErrCode = u8ErrCodeGet();
		if (0 != u8ErrCode)
		{
			vLedSendAlmCode(u8ErrCode);
			CanSend103Last.u8Data[7] = u8ErrCode;
		}
		else
		{
			CanSend103Last.u8Data[7] = 0;
		}
	}
	
	vWdgSetFun(WDG_USER_BIT);
}
#endif
