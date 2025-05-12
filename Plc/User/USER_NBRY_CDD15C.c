/*******************************************************************************
* Filename: UserStackerTruckProc.c	                                           *
* Description: 逻辑层的源文件，包含对应的回调函数	   						   *
* Author:                                                      *
* Date: 2023/05/30    														   *
* Revision:	 														 		   *
*******************************************************************************/
#include "UserCanComm.h"
#include "UserStackerTruckProcRUYICDD15C.h"
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

#if (USER_TYPE == USER_RUYI_CDD15C)

const static xPdoParameter  sgPdoPara = 
{
	.TpdoPara = {
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 100, .u16CanId = 0x260}
	},
	.RpdoPara = {
		{.b1Flag = 1, .b11CanRevId = 0x360},
		{.b1Flag = 1, .b11CanRevId = 0x200},
		{.b1Flag = 1, .b11CanRevId = 0x270},
		{.b1Flag = 1, .b11CanRevId = 0x190},
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

//#define SLOW_SPEED
//#define FAST_SPEED	(0x01 << 7)
//#define ZUOYI_SWI (0x01 << 4)

#define LifeUpLimit			(0x01 << 0)
#define LiftDownLimit		(0x01 << 1)
#define ZhiTuiFLimit		(0x01 << 2)
#define ZhiTuiSLimit		(0x01 << 3)

#define BMS_NOLIFTUP			(0x0001 << 0)
#define SWI_NOLIFTUP			(0x0001 << 1)

typedef union
{
	uint16_t u16data;
	struct
	{
		uint16_t b1HeightSpdlimit: 1;         //高度限速
		uint16_t b1SafeLock: 1;
		uint16_t b1Forward: 1;
		uint16_t b1Backward: 1;
		uint16_t b1LiftUp: 1;
		uint16_t b1LiftDown: 1;
		uint16_t b1ZhituiS: 1;
		uint16_t b1ZhituiF: 1;
		uint16_t b1Streelock: 1;
		uint16_t b1ZhituiSChk: 1;
		uint16_t b1ZhituiFChk: 1;
		uint16_t b6Reserve: 6;
	};
}xSwiInput;

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
	uint8_t		b3Reserve: 3;	
	
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

xSaveStateInfo sgSaveState;
const static uint8_t u8SeedArray[16]= "V!BWT%EM6dJ8<nPs";

static int16_t	i16MotorSpd = 0;
static uint16_t u16MotorVal = 0;
static uint8_t	u8PumpOrPropValue = 0;
static uint16_t u16CanId5ACPeriod = 0;
static uint16_t u16CanRev62CCnt = 0;
static xValvesInfo sgValvesInfo;
static xUserInfo sgUserInfo;
static xSwiInput sgSwiInput;
static xFlagVal sgFlagVal;
static uint32_t u32HourCount = 0;
static int16_t i16SteerAngle = 0;
static uint16_t DRIVER_FLAG = 0;
static int32_t i32PropValue = 0;
static uint8_t check_err = 0;
static uint8_t PorpState = 0;
static uint8_t LiftFlag = 0;
static uint8_t ZhiTuiFlag = 0;
static uint8_t ZhiTuiSFlag = 0;
static uint8_t ZhiTuiFFlag = 0;
static uint8_t SLimit = 0;
static uint8_t FLimit = 0;
static uint8_t PropLimit = 0;
static uint8_t u8BMS_ERRCODE = 0;
static uint8_t u8SafeState = 0;
static uint32_t DelayProp = 0;
static uint8_t PropFlg = 0;
static uint8_t PropFlg2 = 0;
static uint16_t Rev_Speed;
static uint8_t BMSSOC = 100;
static uint8_t MoveLimit = 0;

/*PDO接收待完善*/
xCanRevPdoInfo gCanRevPdoInfo; 
static xCanRev360Info CanRev360InfoLast;	
static xCanRev200Info CanRev200InfoLast;	
static xCanRev270Info CanRev270InfoLast;
static xCanRev190Info CanRev190InfoLast;
/*PDO发送待完善*/
xCanSendPdoInfo gCanSendPdoInfo;


//const static uint8_t u8ErrSwitchArray[ErrCodeMax] = 
//{	
///*  0    1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18   19*/
//	0,   82,  208, 180, 0,   227, 227, 227, 211, 211, 0,   180, 180, 60,  38,  216, 19,  212, 62,  65,
//	78,  37,  53,  227, 75,  17,  248, 0,   0,   227, 0,   19,  62,  62,  65,  227, 242, 242, 210, 208,
//	79,  66,  66,  0,   0,   0,   0,   0,   0,   0,   242, 242, 242, 242, 242, 242, 242, 242, 242, 242,
//	60,  61,  62,  79,  79,  205, 79,  66,  66,  69,  70,  78,  72,  188,  74,  75,  76,  77,  78,  79, 
//	80,  81,  82,  83,  84,  85,  86,  87,  88,  89,  90,  91,  92,  93,  94,  95,  96,  97,  98,  99,
//	100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 
//	120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 
//	140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159,
//	160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 
//	180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 
//	200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 
//	220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 
//	240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254,
//};
static void vMstRevProc(xMstRevPara *RevData)
{	
	uint16_t tmp = 0;
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
			 if(40 != RevData->u8ErrCode)
				i32ErrCodeClr(i);
			}
		}
		if(40 != RevData->u8ErrCode)
		i32ErrCodeSet(RevData->u8ErrCode - 1);	
	}
	else
	{
		if (u8ErrCodeGet() < 50)				/*lilu, 20230817, 没有MCU的故障的时候，就清除MCU的所有故障码*/
		{
			uint8_t i = 0;
			for (i=0; i<50; i++)
			{
			if(39 != u8ErrCodeGet())
				i32ErrCodeClr(i);
			}
		}
	}
	
	i16MotorSpd = (uint16_t)(RevData->u8SpeedFdbHigh << 8) | RevData->u8SpeedFdbLow ;
	__disable_irq();
	tmp = abs(i16MotorSpd / sgUserInfo.u16RatioOfTransmission);
	gCanSendPdoInfo.CanSend260Info.u8Movespeed = tmp;
//	gCanSendPdoInfo.CanSend260Info.u8ErrorMove = RevData->u8ErrCode;
	__enable_irq();
	
	tmp = RevData->u8MotorTmp;
	/*Motor Temp*/
	
	tmp = RevData->u8BoardTmp;
	/**/

	
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
	static xValvesInfo sgLastValvesInfo;
	static uint8_t u8MoveSwitchFlag = 0;
	static uint16_t u8MainConnectCnt = 0;
	float fTmp = 0.0;
	xMstSendStat LastStatus;
	int16_t i16Spd = 0;
	static uint16_t LiftDelay = 0;
	static uint16_t LiftDelay2 = 0;
	static uint16_t LiftDelay3 = 0;
		
	LastStatus.u8Data = SendData->buf[2];
	SendData->buf[2] = 0;
	
	if ((0 == sgSwiInput.b1SafeLock)
		&&(u8MainConnectCnt>1000))		/*add Main Connector*/
	{
		SendData->b1PowerLineOn = 1;     // 1:断开 0:连接
	}
	else if(0 == sgSwiInput.b1SafeLock)
	{
		u8MainConnectCnt ++;
	}
	else
	{
		u8MainConnectCnt = 0;
	}
	i32DoPwmSet(DIDAFLG_VALVE,DRIVER_CLOSE);
	if(sgSwiInput.b1Backward == 1)
	{
		i32DoPwmSet(DIDAFLG_VALVE,DRIVER_OPEN);
	}
	/*Move Mode*/
	if((0 == sgValvesInfo.b1NoActFlag)&&(0 == (u8BMS_ERRCODE & 0x01))&&(0 == MoveLimit)&&(0 == PropLimit))
	{
		SendData->b1ServoOn = 1;
		if(sgSwiInput.b1Backward == 1)
		{
			i32DoPwmSet(DIDAFLG_VALVE,DRIVER_OPEN);
			SendData->b1BackwardReq = 1;
		}
		else if(sgSwiInput.b1Forward == 1)
		{
			SendData->b1ForwardReq = 1;
		}
		
		if((0 != sgSwiInput.b1Backward)||(0 != sgSwiInput.b1Forward))
		{
			SendData->u8TargetHigh = u16MotorVal >> 8;
			SendData->u8TargetLow = u16MotorVal;
		}
		
		if(0 != CanRev360InfoLast.u8ErrSteer)
		{
			SendData->b1ServoOn = 0;
		}
		
		if(DelayProp > 0)
		{
			DelayProp--;
		}
		
		/*Lift Mode*/
		
		if((1 == sgSwiInput.b1LiftUp)
			&&(0 == (sgValvesInfo.b4NoLiftUp & BMS_NOLIFTUP))
			&&((0 == sgFlagVal.Height_SpeedLimit)||(1 == sgSwiInput.b1ZhituiFChk))
			&&(0 == (PorpState & LifeUpLimit))
			&&(0 == (u8BMS_ERRCODE & 0x02))
			&&(0 == DelayProp)
			&&(0 == PropLimit)
		)
	{
		LiftDelay2 = 0;
		PropFlg = 1;
		SendData->b1LiftReq = 1;
		SendData->u8PumpTarget = u8PumpOrPropValue;
		i32DoPwmSet(LIFTUP_VALVE,DRIVER_OPEN);
		vPropSetTarget(LIFTDOWN_VALVE, 0);
	}
	else if((1 == sgSwiInput.b1LiftDown)&&(0 == DelayProp)&&(0 == (PorpState & LifeUpLimit))&&(0 == PropLimit)&&(1 == sgSwiInput.b1SafeLock))
	{
		LiftDelay2 = 0;
		PropFlg = 1;
		i32DoPwmSet(LIFTUP_VALVE,DRIVER_OPEN);
		i32PropValue = _IQ((sgUserInfo.fPropMinCurrent + (sgUserInfo.fPropMaxCurrent - sgUserInfo.fPropMinCurrent) * u8PumpOrPropValue / PUMP_RANGE) / PROPD_STD_CURRENT);
		vPropSetTarget(LIFTDOWN_VALVE, i32PropValue);
	}
	else
	{
		LiftDelay = 0;
		if(1 == PropFlg)
		{
			if((0 == sgSwiInput.b1ZhituiF)&&(0 == sgSwiInput.b1ZhituiS)&&(0 == sgSwiInput.b1LiftDown)&&(0 == sgSwiInput.b1LiftUp))
			{
				PropFlg = 0;
				DelayProp = i32GetPara(PARA_DriveSpeedAfterLift)*100/5;  //2#
			}
		}
		if(LiftDelay2 < 100)
		{
			LiftDelay2++;
		}
		else
		{
			i32DoPwmSet(LIFTUP_VALVE,DRIVER_CLOSE);
		}
		vPropSetTarget(LIFTDOWN_VALVE, 0);
		u8PumpOrPropValue = 0;
		i32PropValue = 0;
	}
		
		if((0 == PropLimit)&&(1 == sgSwiInput.b1ZhituiS)&&(0 == (PorpState &ZhiTuiFLimit))&&(0 == SLimit)&&(DelayProp == 0))
		{
			LiftDelay3 = 0;
			PropFlg2 = 1;
			SendData->b1LiftReq = 1;
			SendData->u8PumpTarget = (255*sgUserInfo.u16Gear2Spd)/100;
			i32DoPwmSet(ZHITUIS_VALVE,DRIVER_OPEN);
			i32DoPwmSet(ZHITUIF_VALVE,DRIVER_CLOSE);
		}
		else if((0 == PropLimit)&&(1 == sgSwiInput.b1ZhituiF)&&(0 == (PorpState &ZhiTuiFLimit))&&(0 == FLimit)&&(DelayProp == 0))
		{
			LiftDelay3 = 0;
			PropFlg2 = 1;
			SendData->b1LiftReq = 1;
			SendData->u8PumpTarget = (255*sgUserInfo.u16Gear2Spd)/100;
			i32DoPwmSet(ZHITUIF_VALVE,DRIVER_OPEN);
			i32DoPwmSet(ZHITUIS_VALVE,DRIVER_CLOSE);
		}
		else
		{
			if(1 == PropFlg2)
			{
				if((0 == sgSwiInput.b1ZhituiF)&&(0 == sgSwiInput.b1ZhituiS)&&(0 == sgSwiInput.b1LiftDown)&&(0 == sgSwiInput.b1LiftUp))
				{
					PropFlg2 = 0;
					DelayProp = i32GetPara(PARA_DriveSpeedAfterLift)*100/5;  //2#
				}
			}
			if(LiftDelay3 < 150)
			{
				LiftDelay3++;
			}
			else
			{
				i32DoPwmSet(ZHITUIS_VALVE,DRIVER_CLOSE);
				i32DoPwmSet(ZHITUIF_VALVE,DRIVER_CLOSE);
			}
		}
	}
	
	if (0 == sgSwiInput.b1SafeLock)
	{
		SendData->b1ServoOn = 0;
		SendData->b1LiftReq = 0;
	}
	
		static uint32_t u32Cnt = 0;
		u32Cnt++;
		{			
			/*lilu 20230819 For Test*/
			{
				i32SetPara(PARA_ForwardValveCurrent, DelayProp);		/*Send Motor Value*/
				i32SetPara(PARA_BackValveCurrent, CanRev270InfoLast.SpeedMode);		/*Rev Motor Value*/
				i32SetPara(PARA_PropValveCurrent, gCanSendPdoInfo.CanSend260Info.u8ErrorMove);	/*Prop Current*/
				i32SetPara(PARA_LiftValveCurrent, ((gCanSendPdoInfo.CanSend260Info.u8HourCountH >> 8)|(gCanSendPdoInfo.CanSend260Info.u8HourCountL)));		/*Send Pump Value*/
				i32SetPara(PARA_OnOffValveCurrent, SendData->buf[2]);			/*Send Status*/
				i32SetPara(PARA_LoadRate, sgValvesInfo.u8Data[0]);				/*Valve NoFlag*/
				i32SetPara(PARA_CalibrationStatus, sgValvesInfo.u8Data[1]);		/*Value State*/
				i32SetPara(PARA_TurnRightValveCurrent, ((uint16_t)sgValvesInfo.u8NoAct << 8) | ((uint16_t)sgValvesInfo.b4NoMove << 4) | ((uint16_t)sgValvesInfo.b4NoLiftUp));
				i32SetPara(PARA_TurnLeftValveCurrent, ((uint16_t)sgValvesInfo.b4Gear1Spd << 12) | ((uint16_t)sgValvesInfo.b4Gear2Spd << 8) | \
					((uint16_t)sgValvesInfo.b4Gear3Spd << 4) | ((uint16_t)sgValvesInfo.b4Gear4Spd));
				i32SetPara(PARA_AngleValue, (uint16_t)i32LocalAiGetValue(AI_B_AI1_R));		/*AI1*/
				i32SetPara(PARA_PressureVlaue1, SendData->u8PumpTarget);	/*AI2*/
				i32SetPara(PARA_PressureVlaue2, (uint16_t)i32LocalAiGetValue(AI_B_AI2_R));	/*AI2*/
				i32SetPara(PARA_ExtSignal, sgSwiInput.u16data);						/*Swi*/
				i32SetPara(PARA_PcuKeyInfo,sgValvesInfo.u8Data[0]);						/*ErrCode*/
//				i32SetPara(PARA_BmsSoc, gCanSendPdoInfo.CanSend23CInfo.u8Soc);		/*BMS SOC*/
				i32SetPara(PARA_BrakeValveCurrent, abs(i16SteerAngle));
				
			}
		}
}

static void vDoPwmErrCallBack(eDoPwmNo DoPwmNo)
{
//	switch((uint8_t)DoPwmNo)
//	{
//		case LIFTUP_VALVE:
//			i32ErrCodeSet(LIFTUP_VALVE_ERR);
//			/*add ErrCode*/
//			break;
//		default:
//			break;
//	}
}

static void vPropErrCallBack(uint8_t u8Channel)
{
//	switch(u8Channel)
//	{
//		case LIFTDOWN_VALVE:
//			i32ErrCodeSet(LIFTDOWN_VALVE_ERR);
//			/*add errcode*/
//			break;
//		default:
//			break;
//	}
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

//void vCanLostProc(uint32_t u32CanID,uint8_t u8State)
//{
//	switch(u32CanID)
//	{
//		case 0x1E0:
//			if(CAN_NORMAL == u8State)
//			{
//				i32ErrCodeClr(CanIDlost1_ERR);
//			}
//			else if(CAN_LOST == u8State)
//			{
//				i32ErrCodeSet(CanIDlost1_ERR);
//			}
//			break;
//		default:
//			break;
//		}
//}

/*******************************************************************************
* Name: vCanLostProc(uint32_t u32CanID,uint8_t u8State)
* Descriptio: 正常通道ID掉线检测
* Input: NULL
* Output: NULL  
*******************************************************************************/
//void vCanLostProc(uint32_t u32CanID,uint8_t u8State)
//{
//	switch(u32CanID)
//	{
//		case 0x200:
//			if(CAN_NORMAL == u8State)
//			{
//				i32ErrCodeClr(41);
//			}
//			else if(CAN_LOST == u8State)
//			{
//				i32ErrCodeSet(41);
//			}
//			break;
//		default:
//			break;
//	
//	}
//}


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
	}
	
		/***** 锂电200 *******/
	if(0 == sgUserInfo.u8BatteryType)
	{
			/***** 锂电200 *******/
		{
			memcpy((char*)CanRev200InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo200.u8Data, sizeof(CanRev200InfoLast));
			/*添加相关操作*/
				gCanSendPdoInfo.CanSend260Info.u8Soc = CanRev200InfoLast.BMS_SOC;		
				BMSSOC =  CanRev200InfoLast.BMS_SOC;
		}
		
//		if(0 == CanRev200InfoLast.BMS_SOC)
//		{
//			i32ErrCodeSet(41);
//			sgValvesInfo.b4NoLiftUp |= BMS_NOLIFTUP;
//			sgValvesInfo.b1Gear1SpdFlag = 0;
//			sgValvesInfo.b1Gear2SpdFlag = 0;
//			sgValvesInfo.b1Gear3SpdFlag = 1;
//			sgValvesInfo.b1Gear4SpdFlag = 0;
//		}
		
		memcpy((char*)CanRev190InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo190.u8Data, sizeof(CanRev190InfoLast));
		/*添加相关操作*/
			{
		const INT8U HmiErrorTab[] = 
			{
			90, 80, 90, 80, 20, 10, 50, 100,
			110, 70, 70, 60, 00, 00, 40, 30,
			00, 00, 00, 00, 00, 00, 00, 00,
			};
		INT8U ErrorBms2Hmi = 0;
		INT16U BmsError2Mcu;
		INT8U ErrorBms2HmiLevel = 0;
			
		INT16U BmsError2Mcu1 =0;
		INT16U BmsError2Mcu2 =0;
		INT16U BmsError2Mcu3 =0;
			
		BmsError2Mcu1 = ( CanRev190InfoLast.u8BMS_ERR1H<<8 ) | ( CanRev190InfoLast.u8BMS_ERR1L );
		BmsError2Mcu2 = ( CanRev190InfoLast.u8BMS_ERR2H<<8 ) | ( CanRev190InfoLast.u8BMS_ERR2L );
		BmsError2Mcu3 = ( CanRev190InfoLast.u8BMS_ERR3H<<8 ) | ( CanRev190InfoLast.u8BMS_ERR3L );
		
		if ((BmsError2Mcu = BmsError2Mcu1) != 0)
		{
			do{
				ErrorBms2Hmi += 1;
				if ((BmsError2Mcu & 0x3) != 0)
				{
					ErrorBms2HmiLevel = BmsError2Mcu & 0x3;
					break;
				}
				BmsError2Mcu >>= 2;
			} while(BmsError2Mcu != 0);
		}
		else if ((BmsError2Mcu = BmsError2Mcu2) != 0)
		{
			ErrorBms2Hmi = 8;
			do{
				ErrorBms2Hmi += 1;
				if ((BmsError2Mcu & 0x3) != 0)
				{
					ErrorBms2HmiLevel = BmsError2Mcu & 0x3;
					break;
				}
				BmsError2Mcu >>= 2;
			} while(BmsError2Mcu != 0);
		}
		else if ((BmsError2Mcu = (BmsError2Mcu3 & 0xff)) != 0)
		{
			ErrorBms2Hmi = 16;
			do{
				ErrorBms2Hmi += 1;
				if ((BmsError2Mcu & 0x3) != 0)
				{
					ErrorBms2HmiLevel = BmsError2Mcu & 0x3;
					break;
				}
				BmsError2Mcu >>= 2;
			} while(BmsError2Mcu != 0);
		}
		if (   (ErrorBms2Hmi != 0)
			&& (HmiErrorTab[ErrorBms2Hmi - 1] != 0)
			)
		{
			gCanSendPdoInfo.CanSend260Info.u8ErrorBMS = HmiErrorTab[ErrorBms2Hmi - 1] + ErrorBms2HmiLevel;
		}
		u8BMS_ERRCODE = CanRev190InfoLast.u8BMS_ERR_Require;
	}
	}
	else
	{
			CanRev200InfoLast.BMS_SOC = u8GetBatterySoc();
			gCanSendPdoInfo.CanSend260Info.u8Soc = u8GetBatterySoc();
	}
	
	{
		memcpy((char*)CanRev270InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo270.u8Data, sizeof(CanRev270InfoLast));
		/*添加相关操作*/
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
	
/*********** Motor Speed *************/
	/*add Turn Dec Spd*/
	
	i32AdcValue = i32LocalAiGetValue(MOVE_THROTTLE);
	if((i32AdcValue > 2550)&&(i32AdcValue < sgUserInfo.u16ThrottleMax))
	{
		u16MotorVal = (((i32AdcValue - 2550)*4096)/(sgUserInfo.u16ThrottleMax-2550));
	}
	else if((i32AdcValue < 2450)&&(i32AdcValue > sgUserInfo.u16ThrottleMin))
	{
		u16MotorVal = (((2450 - i32AdcValue)*4096)/(2450 - sgUserInfo.u16ThrottleMin));
	}
	else
	{
		u16MotorVal = 0;
	}
	u16MotorVal = u16SteerAngleDecSpd(u16MotorVal, abs(i16SteerAngle), &sgUserInfo.SteerAngleDecSpd);
	
	{
		i32AdcValue = i32LocalAiGetValue(LIFT_THROTTLE);
		if((i32AdcValue > 2550)&&(i32AdcValue < sgUserInfo.u16LiftUpMax))
		{
			u8PumpOrPropValue = (255*(i32AdcValue-2550))/(sgUserInfo.u16LiftUpMax-2550);
		}
		else if((i32AdcValue < 2450)&&(i32AdcValue > sgUserInfo.u16LiftUpMin))
		{
			u8PumpOrPropValue = (255*(2450-i32AdcValue))/(2450-sgUserInfo.u16LiftUpMin);
		}
		else
		{
			u8PumpOrPropValue = 0;
		}
	}
	
	/*add spd limit*/
	gCanSendPdoInfo.CanSend260Info.u8MoveState |= (0x01 << 7);
	gCanSendPdoInfo.CanSend260Info.u8MoveState &= ~(0x01 << 6);
	gCanSendPdoInfo.CanSend260Info.u8MoveState &= ~(0x00 << 0);
	if (1 == sgValvesInfo.b1Gear1SpdFlag)
	{
		if (u8SpeedRate >= sgUserInfo.u16Gear1Spd)   //经济
		{
			u8SpeedRate = sgUserInfo.u16Gear1Spd;
			gCanSendPdoInfo.CanSend260Info.u8MoveState &= ~(0x01 << 7);
			gCanSendPdoInfo.CanSend260Info.u8MoveState &= ~(0x01 << 6);
			gCanSendPdoInfo.CanSend260Info.u8MoveState |= (0x00 << 0);
		}
	}
	
	if (1 == sgValvesInfo.b1Gear2SpdFlag)
	{
		if (u8SpeedRate >= sgUserInfo.u16Gear2Spd)   //踢腿
		{
			u8SpeedRate = sgUserInfo.u16Gear2Spd;
		}
	}
	
	if (1 == sgValvesInfo.b1Gear3SpdFlag)   // 低电量
	{
		if (u8SpeedRate >= sgUserInfo.u16Gear3Spd)
		{
			u8SpeedRate = sgUserInfo.u16Gear3Spd;
		}
	}
	
	if (1 == sgValvesInfo.b1Gear4SpdFlag)    //龟速
	{
		if (u8SpeedRate >= sgUserInfo.u16Gear4Spd)
		{
			u8SpeedRate = sgUserInfo.u16Gear4Spd;
			gCanSendPdoInfo.CanSend260Info.u8MoveState &= ~(0x01 << 7);
			gCanSendPdoInfo.CanSend260Info.u8MoveState &= ~(0x00 << 0);
			gCanSendPdoInfo.CanSend260Info.u8MoveState |= (0x01 << 6);
		}
	}
	u16MotorVal = (uint16_t)((u16MotorVal * u8SpeedRate)/100);
	
	if (1 == sgValvesInfo.b1Gear3SpdFlag)   // 低电量
	{
			gCanSendPdoInfo.CanSend260Info.u8MoveState &= ~(0x01 << 7);
			gCanSendPdoInfo.CanSend260Info.u8MoveState &= ~(0x01 << 6);
			gCanSendPdoInfo.CanSend260Info.u8MoveState |= (0x00 << 0);
	}
}


/*开关量监控*/
static void vSwiMonitor(void)
{
	xSwiInput SwiInput = {0};
	static uint8_t Snail_Count = 0;
	static uint16_t SafeLockDelay = 4096;
	static uint8_t BMSDelay = 0;
	
	if(1 == i32LocalDiGet(HEIGHT_SPEEDLIMIT_SWI))
	{
		SwiInput.b1HeightSpdlimit = 1;
	}
	
	if(1 == i32LocalDiGet(SAFELOCK_SWI))
	{
		SwiInput.b1SafeLock = 1;
		SafeLockDelay = 0;
	}
	else if(0 == i32LocalDiGet(SAFELOCK_SWI))
	{
		if(SafeLockDelay < (i32GetPara(PARA_SlowDriveSpeed)*20))   //1# 0.1S
		{
			SafeLockDelay ++;
			SwiInput.b1SafeLock = 1;
		}
	}
	
	if(0 == SwiInput.b1SafeLock)
	{
		gCanSendPdoInfo.CanSend260Info.u8MoveState |= (0x01 << 4);
	}
	else
	{
		gCanSendPdoInfo.CanSend260Info.u8MoveState &= ~(0x01 << 4);
	}
	
	if(1 == i32LocalDiGet(ForWard_SWI))
	{
		SwiInput.b1Forward = 1;
	}
	
	if(1 == i32LocalDiGet(BackWard_SWI))
	{
		SwiInput.b1Backward = 1;
	}
	
	if(1 == i32LocalDiGet(LIFTUP_SWI))
	{
		SwiInput.b1LiftUp = 1;
	}
	
	if(1 == i32LocalDiGet(LIFTDOWN_SWI))
	{
		SwiInput.b1LiftDown = 1;
	}
	
	if(1 == i32LocalDiGet(ZhiTuiS_SWI))
	{
		SwiInput.b1ZhituiS = 1;
	}
	
	if(1 == i32LocalDiGet(ZhiTuiF_SWI))
	{
		SwiInput.b1ZhituiF = 1;
	}
	
	if(1 == i32LocalDiGet(STREESAFELOCK_SWI))
	{
		SwiInput.b1Streelock = 1;
	}
	
	if(1 == i32LocalDiGet(ZhiTuiFChk_SWI))
	{
		SwiInput.b1ZhituiFChk = 1;
	}	
	
	sgSwiInput.u16data = SwiInput.u16data;
	
	if((1 == sgSwiInput.b1HeightSpdlimit)&&(1 ==sgSwiInput.b1LiftUp))
	{
		sgFlagVal.Height_SpeedLimit = 1;
		u16EepromWrite(PARA_SaveState, sgSaveState.u16Data, 1);
	}
	else if((1 == sgSwiInput.b1HeightSpdlimit)&&(1 == sgSwiInput.b1LiftDown))
	{
		sgFlagVal.Height_SpeedLimit = 0;
		u16EepromWrite(PARA_SaveState, sgSaveState.u16Data, 1);
	}
	sgSaveState.b1HeightSpdLimit = sgFlagVal.Height_SpeedLimit;

	if((1 == sgFlagVal.Height_SpeedLimit)||(1 == SwiInput.b1ZhituiFChk)||(0x09 == CanRev270InfoLast.SpeedMode))    //高度限速，限制为四档速度
	{
		sgValvesInfo.b1Gear1SpdFlag = 0;
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 0;
		sgValvesInfo.b1Gear4SpdFlag = 1;
	}
	else if(0x19 == CanRev270InfoLast.SpeedMode)
	{
		sgValvesInfo.b1Gear1SpdFlag = 1;
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 0;
		sgValvesInfo.b1Gear4SpdFlag = 0;
	}
	else
	{
		sgValvesInfo.b1Gear1SpdFlag = 0;
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 0;
		sgValvesInfo.b1Gear4SpdFlag = 0;
	}
	if(BMSDelay < 200)
	{
		BMSDelay++;
	}
	else
	{
		if(0 == sgUserInfo.u8BatteryType)
		{
			if(BMSSOC < 10)
			{
				i32ErrCodeSet(41);
				sgValvesInfo.b4NoLiftUp |= BMS_NOLIFTUP;
				MoveLimit = 1;
			}
			else if((BMSSOC < 15)||(0 !=(u8BMS_ERRCODE & 0x0A)))
			{
				if(BMSSOC < 15)
				{
					 i32ErrCodeSet(40);
				}
				sgValvesInfo.b4NoLiftUp |= BMS_NOLIFTUP;
				sgValvesInfo.b1Gear1SpdFlag = 0;
				sgValvesInfo.b1Gear2SpdFlag = 0;
				sgValvesInfo.b1Gear3SpdFlag = 1;
				sgValvesInfo.b1Gear4SpdFlag = 0;
			}
		}
		else
		{
			if(u8GetBatterySoc() < 10)
			{
				i32ErrCodeSet(41);
				sgValvesInfo.b4NoLiftUp |= BMS_NOLIFTUP;
				MoveLimit = 1;
			}
			else if((u8GetBatterySoc() < 15)||(0 !=(u8BMS_ERRCODE & 0x0A)))
			{
				if(u8GetBatterySoc() < 15)
				{
					 i32ErrCodeSet(40);
				}
				sgValvesInfo.b4NoLiftUp |= BMS_NOLIFTUP;
				sgValvesInfo.b1Gear1SpdFlag = 0;
				sgValvesInfo.b1Gear2SpdFlag = 0;
				sgValvesInfo.b1Gear3SpdFlag = 1;
				sgValvesInfo.b1Gear4SpdFlag = 0;
			}
		}
	}
	if(((1 == SwiInput.b1LiftUp)||(1 == SwiInput.b1LiftDown))&&(0 == ZhiTuiFlag))
	{
		LiftFlag = 1;
		if((1 == SwiInput.b1ZhituiF)||(1 == SwiInput.b1ZhituiS))
		{
			PorpState |= ZhiTuiFLimit;
		}
		else
		{
			PorpState &= ~ZhiTuiFLimit;
		}
	}
	else
	{
		LiftFlag = 0;
		if((0 != (PorpState & ZhiTuiFLimit))&&(0 == check_err))
		{
			i32ErrCodeSet(101);
		}
		if((0 == SwiInput.b1ZhituiF)&&(0 == SwiInput.b1ZhituiS))
		{
			i32ErrCodeClr(101);
			PorpState &= ~ZhiTuiFLimit;
		}
	}
	
	if(((1 == SwiInput.b1ZhituiF)||(1 == SwiInput.b1ZhituiS))&&(0 == LiftFlag))
	{
		ZhiTuiFlag = 1;
		if((1 == SwiInput.b1LiftUp)||(1 == SwiInput.b1LiftDown))
		{
			PorpState |= LifeUpLimit;
		}
		else
		{
			PorpState &= ~LifeUpLimit;
		}
	}
	else
	{
		ZhiTuiFlag = 0;
		if((0 != (PorpState & LifeUpLimit))&&(0 == check_err))
		{
			i32ErrCodeSet(102);
		}
		if((0 == SwiInput.b1LiftUp)&&(0 == SwiInput.b1LiftDown))
		{
			i32ErrCodeClr(102);
			PorpState &= ~LifeUpLimit;
		}
	}
	

	if((1 == SwiInput.b1ZhituiF)&&(0 == ZhiTuiSFlag))
	{
			ZhiTuiFFlag = 1;
			if(1 == SwiInput.b1ZhituiS)
			{
				SLimit = 1;
			}
			else
			{
				SLimit = 0;
			}
	}
	else
	{
		ZhiTuiFFlag = 0;
		if((1 == SLimit)&&(0 == check_err))
		{
			i32ErrCodeSet(103);
		}
		if(0 == SwiInput.b1ZhituiS)
		{
			i32ErrCodeClr(103);
			SLimit = 0;
		}
	}
	if((1 == SwiInput.b1ZhituiS)&&(0 == ZhiTuiFFlag))
	{
			ZhiTuiSFlag = 1;
			if(1 == SwiInput.b1ZhituiF)
			{
				FLimit = 1;
			}
			else
			{
				FLimit = 0;
			}
	}
	else
	{
		ZhiTuiSFlag = 0;
		if((1 == FLimit)&&(0 == check_err))
		{
			i32ErrCodeSet(104);
		}
		if(0 == SwiInput.b1ZhituiF)
		{
			i32ErrCodeClr(104);
			FLimit = 0;
		}
	}
	
	if(((1 == SwiInput.b1LiftDown)||(1 == SwiInput.b1Backward)||(1 == SwiInput.b1Forward)||(1 == SwiInput.b1LiftUp)||(1 == SwiInput.b1ZhituiF)||(1 == SwiInput.b1ZhituiS))
		&&(0 == u8SafeState)&&(0 == check_err))
	{
		PropLimit = 1;
		if(1 == SwiInput.b1SafeLock)
		{
			i32ErrCodeSet(39);      //在没有互锁时 有起升动作
		}
	}
	else if(1 == SwiInput.b1SafeLock)
	{
		u8SafeState = 1;
		if((0 == SwiInput.b1LiftDown)
			&&(0 == SwiInput.b1LiftUp)
			&&(0 == SwiInput.b1Backward)
			&&(0 == SwiInput.b1Forward)
			&&(0 == SwiInput.b1ZhituiF)
			&&(0 == SwiInput.b1ZhituiS))
			{
				if(1 == PropLimit)
				{
					PropLimit = 0;
					i32ErrCodeClr(39);
				}
			}
	}
	else
	{
		u8SafeState = 0;
		if(1 == PropLimit)
		{
			i32ErrCodeSet(39);
		}
		if((0 == SwiInput.b1LiftDown)
			&&(0 == SwiInput.b1LiftUp)
			&&(0 == SwiInput.b1Backward)
			&&(0 == SwiInput.b1Forward)
			&&(0 == SwiInput.b1ZhituiF)
			&&(0 == SwiInput.b1ZhituiS))
			{
				if(1 == PropLimit)
				{
					PropLimit = 0;
					i32ErrCodeClr(39);
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
	sgUserInfo.u16MotorMaxSpd = i32GetPara(PARA_MotorMaxSpd);
	
	sgUserInfo.u8BatteryType = i32GetPara(PARA_BatteryType);
	
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
	
	/*Rental Info*/
	{
		uint16_t u16RentalInfo = 0;
		u16RentalInfo = i32GetPara(RENTAL_INFO);
		sgUserInfo.b1RentalStop = u16RentalInfo & 0x01;
		sgUserInfo.b1RentalStart = (u16RentalInfo >> 1) & 0x01;
		sgUserInfo.b1RentalMode = (u16RentalInfo >> 2) & 0x01;
	}
	
	u16EepromRead(PARA_DefaultFlag, &u16Tmp, 1);
	u16EepromRead(PARA_SaveState, &sgSaveState.u16Data, 1);
//	//i32LogWrite(INFO, "******DefaultFlag = 0x%x\r\n*********", u16Tmp);
	if (0x5555 != u16Tmp)
	{
		u16EepromWrite(PARA_SaveState, sgSaveState.u16Data, 1);
		u16EepromWrite(PARA_HourSetTime, 0x0000, 1);
		u16EepromWrite(PARA_DefaultFlag, 0x5555, 1);	
	}
	else
	{
		u16EepromRead(PARA_SaveState, &sgSaveState.u16Data, 1);
//		sgSwiInput.b1HeightLimit = sgSaveState.b1Above1M8;
		sgFlagVal.Height_SpeedLimit = sgSaveState.b1HeightSpdLimit;        //高度限速保存到eeprom
		sgUserInfo.b1PasswordFunc = sgSaveState.b1PasswordFunc;
		u16EepromRead(PARA_HourSetTime, &u16Tmp, 1);	
	}
	if (1 == sgUserInfo.b1PasswordFunc)
	{
		sgValvesInfo.u8NoAct |= 1 << NoAct_Security;
//		i32ErrCodeSet(DEV_62C_LOST_ERR);
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

//	vCanIdLostReg(0x200,1000,vCanLostProc);
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
	static uint32_t u8MainConnectCnt= 0;

	if(true == u8GetNetTimerOverFlag(TIMER_EcuPowerOn))
	{
		u8EcuProcFlag = 1;
		vKillNetTimer(TIMER_EcuPowerOn);
	}
	
		if(u8MainConnectCnt < 100)
		{
			if((1 == i32LocalDiGet(SAFELOCK_SWI))
				||(1 == i32LocalDiGet(ForWard_SWI))||(1 == i32LocalDiGet(BackWard_SWI))
				||(1 == i32LocalDiGet(LIFTUP_SWI))||(1 == i32LocalDiGet(LIFTDOWN_SWI))
				||(1 == i32LocalDiGet(ZhiTuiS_SWI))||(1 == i32LocalDiGet(ZhiTuiF_SWI))
			)
			{
				check_err = 1;
				sgValvesInfo.b1NoActFlag = 1;
			}
			u8MainConnectCnt++;
		}
		else
		{
			if(1 == check_err)
			{
				if((0 == i32LocalDiGet(SAFELOCK_SWI))
					&&(0 == i32LocalDiGet(ForWard_SWI))&&(0 == i32LocalDiGet(BackWard_SWI))
					&&(0 == i32LocalDiGet(LIFTUP_SWI))&&(0 == i32LocalDiGet(LIFTDOWN_SWI))
					&&(0 == i32LocalDiGet(ZhiTuiS_SWI))&&(0 == i32LocalDiGet(ZhiTuiF_SWI))
				)
				{
					check_err = 0;
					sgValvesInfo.b1NoActFlag = 0;
				}
				else if(1 == i32LocalDiGet(SAFELOCK_SWI))
				{
					i32ErrCodeSet(106);
				}
				else if((1 == i32LocalDiGet(ForWard_SWI)))
				{
					i32ErrCodeSet(107);
				}
				else if((1 == i32LocalDiGet(BackWard_SWI)))
				{
					i32ErrCodeSet(108);
				}
				else if(1 == i32LocalDiGet(LIFTUP_SWI))
				{
					i32ErrCodeSet(109);
				}
				else if(1 == i32LocalDiGet(LIFTDOWN_SWI))
				{
					i32ErrCodeSet(110);
				}
				else if(1 == i32LocalDiGet(ZhiTuiS_SWI))
				{
					i32ErrCodeSet(111);
				}
				else if(1 == i32LocalDiGet(ZhiTuiF_SWI))
				{
					i32ErrCodeSet(112);
				}
				if(0 == i32LocalDiGet(SAFELOCK_SWI))
				{
					i32ErrCodeClr(106);
				}
				if((0 == i32LocalDiGet(ForWard_SWI)))
				{
					i32ErrCodeClr(107);
				}
				if((0 == i32LocalDiGet(BackWard_SWI)))
				{
					i32ErrCodeClr(108);
				}
				if(0 == i32LocalDiGet(LIFTUP_SWI))
				{
					i32ErrCodeClr(109);
				}
				if(0 == i32LocalDiGet(LIFTDOWN_SWI))
				{
					i32ErrCodeClr(110);
				}
				if(0 == i32LocalDiGet(ZhiTuiS_SWI))
				{
					i32ErrCodeClr(111);
				}
				if(0 == i32LocalDiGet(ZhiTuiF_SWI))
				{
					i32ErrCodeClr(112);
				}
			}
		}
	
	if(1 == u8EcuProcFlag)
	{
		u16CanId5ACPeriod++;
		if (u16CanId5ACPeriod < 210)	/*200 = 5Times*/
		{
			if (0 == (u16CanId5ACPeriod % CANID_5AC_SEND_PERIOD))	/*Send 5 times 5AC*/
			{
//				vCanId5ACSend();
			}
		}
		else 
		{
			u16CanId5ACPeriod = 210;
		}
		
		if (1 == sgUserInfo.b1PasswordFunc)
		{
			if (u16CanRev62CCnt++ >= CAN_62C_LOST_NO)
			{
				sgValvesInfo.u8NoAct |= 1 << NoAct_Security;
				/*62C Lost Err*/
//				i32ErrCodeSet(DEV_62C_LOST_ERR);
				u16CanRev62CCnt = 0;
			}
		}
		
		vSwiMonitor();		
		vCanRevPdoProc();
		vAiMonitor();
		u8ErrCode = u8ErrCodeGet();
		if (0 != u8ErrCode)
		{
			gCanSendPdoInfo.CanSend260Info.u8ErrorMove = u8ErrCode;
			vLedSendAlmCode(u8ErrCode);
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
		
		if (u16SecCnt >= 360)			/**/
		{
			u16SecCnt = 0;
			u32HourCount++;
			vHourCountWrite(u32HourCount);
			__disable_irq();
			gCanSendPdoInfo.CanSend260Info.u8HourCountL = u32HourCount & 0xFF;
			gCanSendPdoInfo.CanSend260Info.u8HourCountH = (u32HourCount >> 8) & 0xFF;			__enable_irq();
		}
		
		
		/*Rental Info*/
		{
			uint16_t u16RentalInfo = 0;
			u16RentalInfo = i32GetPara(RENTAL_INFO);
			sgUserInfo.b1RentalStop = u16RentalInfo & 0x01;
			sgUserInfo.b1RentalStart = (u16RentalInfo >> 1) & 0x01;
			sgUserInfo.b1RentalMode = (u16RentalInfo >> 2) & 0x01;
		}
		
		u16RentalCnt++;
		if (u16RentalCnt >= 360)
		{
			u16RentalCnt = 0;
			sgUserInfo.u16RentalTime = i32GetPara(RENTAL_TIME);
			if (sgUserInfo.u16RentalTime > 0)
			{
				if ((0 == sgUserInfo.b1RentalStop) && (1 == sgUserInfo.b1RentalStart) && (1 == sgUserInfo.b1RentalMode))
				{
					sgUserInfo.u16RentalTime--;
					i32SetPara(RENTAL_TIME, sgUserInfo.u16RentalTime);
					u16SaveParaToEeprom(RENTAL_TIME, sgUserInfo.u16RentalTime);
				}
			}
		}
	}
	
	vWdgSetFun(WDG_USER_BIT);
}
#endif
