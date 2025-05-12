/*******************************************************************************
* Filename: UserStackerTruckProc.c	                                           *
* Description: 逻辑层的源文件，包含对应的回调函数	   						   *
* Author:                                                      *
* Date: 2023/05/30    														   *
* Revision:	 														 		   *
*******************************************************************************/
#include "UserCanComm.h"
#include "UserStackerTruckProcSanyi.h"
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

#if (USER_TYPE == USER_SANYI_DGC)

const static xPdoParameter  sgPdoPara = 
{
	.TpdoPara = {
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 100, .u16CanId = 0x260},
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 100, .u16CanId = 0x101}
	},
	.RpdoPara = {
		{.b1Flag = 1, .b11CanRevId = 0x115},
		{.b1Flag = 1, .b11CanRevId = 0x360},
		{.b1Flag = 1, .b11CanRevId = 0x4D1},
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

const INT8U ERROR_CODE_MOVE[100]=
{
// 00,  01,  02,  03,  04,  05,  06,  07,  08,  09,
    0,   77,   6,   66,   0,   12,   13,   14,  7,  9,	 	 /*0~9*/
    0,   2,  1,  15,  16,  18,  10,  29,  65,   70,  		/*10~19*/
    20,  21,  22,   23,  24,  25,  45,   0,  0,  27,	    /*20~29*/
    0,   0,  64,  28,  69,   30,   32,   33,  40,  34,	    /*30~39*/
    73,  49,   59,   79,   0,   0,   0,   0,   0,   0,	  		/*40~49*/
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,		    /*50~59*/
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,		    /*60~69*/
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,		    /*70~79*/
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,		    /*80~89*/
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0		    /*90~99*/
};
const INT8U ERROR_CODE_STEER[100]=
{
// 00,  01,  02,  03,  04,  05,  06,  07,  08,  09,
    0,   77,   6,   66,   19,   0,   0,   0,  35,  0,	 	 /*0~9*/
    0,   2,  1,  15,  0,  0,  0,  29,  65,   0,  		/*10~19*/
    0,  0,  22,   23,  24,  0,  38,   39,  0,  0,	    /*20~29*/
    0,   0,  64,  28,  0,   0,   0,   0,  40,  34,	    /*30~39*/
    73,   0,   0,   0,   0,   0,   0,   0,   0,   0,	  		/*40~49*/
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,		    /*50~59*/
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,		    /*60~69*/
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,		    /*70~79*/
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,		    /*80~89*/
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0		    /*90~99*/
};

typedef union
{
	uint16_t u16data;
	struct
	{
		uint16_t b1HeightSpdlimit: 1;         //300高度限速
		uint16_t b1LIFT1800_LIMT_SWI: 1;      //1800高度限位
		uint16_t b1SafeLock: 1;               //互锁
		uint16_t b1Fence1: 1;                 
		uint16_t b1Fence2: 1;                 //两个护臂
		uint16_t b1LIFT_LIMT_SWI: 1;
		uint16_t b1Pedal: 1;                  //踏板
		uint16_t b1PedalCHK: 1;
		uint16_t b3Reserve:8;
	};
}xSwiInput;

typedef union
{
	uint8_t u8data;
	struct
	{
		uint8_t Height_SpeedLimit: 1;
		uint8_t Heigh_1800_Limit: 1;
		uint8_t BMS_NOLift: 1;
		uint8_t BMS_SpeedLimit: 1;
		uint8_t Reserve: 4;
	};
}xFlagVal;

typedef union
{
	uint8_t u8data;
	struct
	{
		uint8_t b1LiftUpL: 1;
		uint8_t b1LiftDownL: 1;
		uint8_t b1LiftUpR: 1;
		uint8_t b1LiftDownR: 1;
		uint8_t b3Reserve: 4;
	};
}xLiftMode;

typedef union
{
	uint8_t u16data;
	struct
	{
		uint8_t b1Backward: 1;	
		uint8_t b1Forward: 1;
		uint8_t b1SnailRequest: 1;
		uint8_t b1EmergencyReverse: 1;
		uint8_t b1Reserve0: 1;
		uint8_t b1Reserve1: 1;
		uint8_t b1ZhiLi_Move: 1;
		uint8_t b1Reserve2: 1;
	};
}xhandleInput;


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
	
	uint16_t  SWI_En;
	
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
static uint8_t u8PumpOrPropValueL;
static uint8_t u8PumpOrPropValueR;
static uint16_t u16CanId5ACPeriod = 0;
static uint16_t u16CanRev62CCnt = 0;
static xValvesInfo sgValvesInfo;
static xUserInfo sgUserInfo;
static xSwiInput sgSwiInput = {0};
static xhandleInput sgHandleInput;
static xLiftMode sgLiftMode;
static uint32_t u32HourCount = 0;
static int16_t i16SteerAngle = 0;
static int16_t i16SHanderValue = 0;
static int32_t i32PropValueL = 0;
static int32_t i32PropValueR = 0;
static int32_t i32PropValue;
static int32_t abs1;
static int32_t abs2;
static int32_t abs3;
static xFlagVal sgFlagVal = {0};
static uint8_t check_err = 0;
static uint16_t u16RentalInfo = 0;
/*PDO接收待完善*/
xCanRevPdoInfo gCanRevPdoInfo; 
static xCanRev22CInfo CanRev22CInfoLast;
static xCanRev360Info CanRev360InfoLast;
static xCanRev115Info CanRev115InfoLast;
static xCanRev4D1Info CanRev4D1InfoLast;
//{
//	.CanRevInfo1.u8Soc = 100,					/*BmsSoc Default: 100*/
//};				
/*PDO发送待完善*/
xCanSendPdoInfo gCanSendPdoInfo;			


xhandleInput HandleInput = {0};

static void vMstRevProc(xMstRevPara *RevData)
{	
	uint16_t tmp = 0;
	static uint16_t err_mst;
	
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
//	i32LogWrite(DEBUG, LOG_USER, "Motor Rev Spd = %d\r\n", i16MotorSpd);
	__disable_irq();
	tmp = abs(i16MotorSpd / sgUserInfo.u16RatioOfTransmission);
	gCanSendPdoInfo.CanSend260Info.u8Movespeed = tmp;
	
	gCanSendPdoInfo.CanSend101Info.u8LedState = 1;
	/*Motor Speed*/
//	gCanSendPdoInfo.CanSend33CInfo.u8SpdHighByte = tmp >> 8;
//	gCanSendPdoInfo.CanSend33CInfo.u8SpdLowByte = tmp;
	__enable_irq();
	
	tmp = RevData->u8MotorTmp;
	/*Motor Temp*/
//	gCanSendPdoInfo.CanSend43CInfo.i16MotorTemp = RevData->u8MotorTmp;
	
	tmp = RevData->u8BoardTmp;
	/**/
//	gCanSendPdoInfo.CanSend43CInfo.i16CtrlTemp = RevData->u8BoardTmp;
	
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
	static uint8_t u8MainConnectCnt = 0;
	float fTmp = 0.0;
	xMstSendStat LastStatus;
	int16_t i16Spd = 0;
	static uint8_t u8PumpOrPropValueLast = 0;
	uint8_t u8PumpOrPropValueTmp= 0;
	static uint8_t PropDelay = 0;
	
//	uint8_t u8Flag = 0;
		
	LastStatus.u8Data = SendData->buf[2];
	SendData->buf[2] = 0;

	if(1 == CanRev4D1InfoLast.b1NFCWork)
	{
		SendData->b1ServoOn = 1;
		if(0 == sgValvesInfo.b1NoActFlag)
		{
			if(HandleInput.b1Backward == 1)
			{
				SendData->b1BackwardReq = 1;
			}
			else if(HandleInput.b1Forward == 1)
			{
				SendData->b1ForwardReq = 1;
			}
			SendData->u8TargetHigh = u16MotorVal >> 8;   //0-4096
			SendData->u8TargetLow = u16MotorVal;
		}
		
		if ((0 == CanRev22CInfoLast.b1EmergencyReverse) && ((0 != gCanSendPdoInfo.CanSend260Info.u8Movespeed)||(1 == SendData->b1ForwardReq) || (1 == LastStatus.b1EmsReq)))  
		{
			SendData->b1EmsReq = 1;
			if (1 == SendData->b1ForwardReq)
			{
				sgValvesInfo.b4NoMove |= 1 << NoMove_Ems;
				i32ErrCodeSet(MOVE_EMS_ERR);
			}
			SendData->b1BackwardReq = 1;      //b1ForwardReq
		}
		
		/*Lift Mode*/
		if(
			(((1 == sgLiftMode.b1LiftUpL)&&(1 == sgLiftMode.b1LiftUpR))
			||((1 == sgLiftMode.b1LiftUpL)&&(0 == sgLiftMode.b1LiftUpR))
			||((0 == sgLiftMode.b1LiftUpL)&&(1 == sgLiftMode.b1LiftUpR)))
			&&(0 == sgValvesInfo.b1LiftUpFlag)
			&&((0 == sgFlagVal.Heigh_1800_Limit)||(0 == sgSwiInput.b1Fence2))
			&&(1 == sgSwiInput.b1SafeLock)
			&&(0 == sgFlagVal.BMS_NOLift)
			&&(0 == sgSwiInput.b1LIFT_LIMT_SWI)
		)
		{
			SendData->b1LiftReq = 1;
			SendData->b1ServoOn = 1;
		}
		else
		{
			SendData->b1LiftReq = 0;
		}
		
		if(1 == SendData->b1LiftReq)
		{
			u8PumpOrPropValue = u8PumpOrPropValueL > u8PumpOrPropValueR ? u8PumpOrPropValueL : u8PumpOrPropValueR;
		}
		else
		{
			u8PumpOrPropValue = 0;
		}
		
		if(u8PumpOrPropValue > (PropDelay-10))
		{
			PropDelay = u8PumpOrPropValue;
		}
		else
		{
			if(PropDelay > 0)
			{
				SendData->b1ServoOn = 1;
				SendData->b1LiftReq = 1;
				if(PropDelay > 120)
				{
					PropDelay -= 10;
				}
				else
				{
					PropDelay -= 1;
				}
			}
			else
			{
				SendData->b1LiftReq = 0;
			}
		}
		SendData->u8PumpTarget = PropDelay;
		
		if(((1 == sgLiftMode.b1LiftDownL)&&(1 == sgLiftMode.b1LiftUpR))
			||((1 == sgLiftMode.b1LiftUpL)&&(1 == sgLiftMode.b1LiftDownR))
			)
		{
			i32ErrCodeSet(39);
			sgValvesInfo.b1LiftUpFlag = 1;
		}
		else
		{	
			if(0 == check_err)
			{
				if((40 == u8ErrCodeGet())&&(1 == sgValvesInfo.b1LiftUpFlag))
				{
					i32ErrCodeClr(39);
				}
				sgValvesInfo.b1LiftUpFlag = 0;
			}
		}
		
		if ((0 == sgSwiInput.b1SafeLock)
			&&(0 == HandleInput.b1ZhiLi_Move)
				)
		{
			SendData->b1ServoOn = 0;
		}
	}	
	/*Move Mode*/
	

	{
		static uint32_t u32Cnt = 0;
		u32Cnt++;
		{			
			/*lilu 20230819 For Test*/
			{
				i32SetPara(PARA_ForwardValveCurrent, CanRev4D1InfoLast.u8Data[0]);		/*Send Motor Value*/
				i32SetPara(PARA_BackValveCurrent, u8PumpOrPropValue);		/*Rev Motor Value*/
				i32SetPara(PARA_PropValveCurrent, inserted_data[0] * PROP_CURRENT_FACOTR * 1000);	/*Prop Current*/
				i32SetPara(PARA_LiftValveCurrent, sgLiftMode.u8data);		/*Send Pump Value*/
				i32SetPara(PARA_OnOffValveCurrent, SendData->buf[2]);			/*Send Status*/
				i32SetPara(PARA_LoadRate, sgValvesInfo.u8Data[0]);				/*Valve NoFlag*/
				i32SetPara(PARA_CalibrationStatus, sgValvesInfo.u8Data[1]);		/*Value State*/
				i32SetPara(PARA_TurnRightValveCurrent, ((uint16_t)sgValvesInfo.u8NoAct << 8) | ((uint16_t)sgValvesInfo.b4NoMove << 4) | ((uint16_t)sgValvesInfo.b4NoLiftUp));
				i32SetPara(PARA_TurnLeftValveCurrent, ((uint16_t)sgValvesInfo.b4Gear1Spd << 12) | ((uint16_t)sgValvesInfo.b4Gear2Spd << 8) | \
					((uint16_t)sgValvesInfo.b4Gear3Spd << 4) | ((uint16_t)sgValvesInfo.b4Gear4Spd));
				i32SetPara(PARA_AngleValue, sgFlagVal.u8data);		/*AI1*/
				i32SetPara(PARA_PressureVlaue1, gCanRevPdoInfo.CanRevInfo115.BMS_SOC);	/*AI2*/
				i32SetPara(PARA_PressureVlaue2, (uint16_t)i32LocalAiGetValue(AI_B_AI3_R));	/*AI3*/
				i32SetPara(PARA_ExtSignal, sgSwiInput.u16data);						/*Swi*/
				i32SetPara(PARA_PcuKeyInfo,HandleInput.u16data);						/*ErrCode*/
//				i32SetPara(PARA_BmsSoc, gCanSendPdoInfo.CanSend23CInfo.u8Soc);		/*BMS SOC*/
				i32SetPara(PARA_BrakeValveCurrent, abs(i16SteerAngle));
				
			}
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
//		case REVERSEA_VALVE:
//			i32ErrCodeSet(REVERSEA_VALVE_ERR);
//			/*add ErrCode*/
//			break;
//		case REVERSEB_VALVE:
//			i32ErrCodeSet(REVERSEB_VALVE_ERR);
//			/*add ErrCode*/
//			break;
//		case FB_MOVE_VALVE:
//			i32ErrCodeSet(FB_MOVE_VALVE_ERR);
//			/*add ErrCode*/
//			break;
//		case FB_TILT_VALVE:
//			i32ErrCodeSet(FB_TILT_VALVE_ERR);
//			/*add ErrCode*/
//			break;
//		case LR_MOVE_VALVE:
//			i32ErrCodeSet(LR_MOVE_VALVE_ERR);
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
//		case AI_B_AI1_R_ERR:
//			i32ErrCodeSet(AI_B_AI1_ERR);
//			break;
//		case AI_B_AI2_R_ERR:
//			i32ErrCodeSet(AI_B_AI2_ERR);
//			break;
//		case AI_B_AI3_R_ERR:
//			i32ErrCodeSet(AI_B_AI3_ERR);
//			break;
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


/*接收PDO处理函数*/
static void vCanRevPdoProc(void)
{
	static uint16_t u16CanRev1ACCnt;
/************ 手柄报文 **********/	
	{
		memcpy((char*)CanRev22CInfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo22C.u8Data, sizeof(CanRev22CInfoLast));
		u16MotorVal = ((CanRev22CInfoLast.b1TargetHigh << 8)|(CanRev22CInfoLast.b1TargetLow));
		
		HandleInput.u16data = CanRev22CInfoLast.u8Data[0];
		
		i32PropValueL = ((CanRev22CInfoLast.b1LiftUpValueHigh << 8)|(CanRev22CInfoLast.b1LiftUpValueLow ));
		i32PropValueR = ((CanRev22CInfoLast.b1RightUpValueHigh <<8)|(CanRev22CInfoLast.b1RightUpValueLow ));
	  /********** 左起升下降************/
		{
			if((i32PropValueL > 2252) &&(i32PropValueL <= 3685))    //3685
			{
				sgLiftMode.b1LiftDownL = 0;
				sgLiftMode.b1LiftUpL = 1;
				abs1 = 0;
				u8PumpOrPropValueL = ((i32PropValueL - 2251)*255)/(3685 - 2252);
			}
			else if(i32PropValueL > 3685)
			{
				sgLiftMode.b1LiftDownL = 0;
				sgLiftMode.b1LiftUpL = 1;
				abs1 = 0;
				u8PumpOrPropValueL = 255;
			}
			else if((i32PropValueL < 1843)&&(i32PropValueL >= 204))   // 409
			{
				sgLiftMode.b1LiftUpL = 0;
				sgLiftMode.b1LiftDownL = 1;
				u8PumpOrPropValueL = 0;
				abs1 = ((1843 - i32PropValueL)*255)/(1843 - 204);
//				i32PropValue = _IQ((sgUserInfo.fPropMinCurrent + (sgUserInfo.fPropMaxCurrent - sgUserInfo.fPropMinCurrent) * u8PumpOrPropValue / PUMP_RANGE) / PROPD_STD_CURRENT);
			}
//			else if((i32PropValueL < 409)&&(i32PropValueL > 50))      //204
//			{
//				sgLiftMode.b1LiftUpL = 0;
//				sgLiftMode.b1LiftDownL = 1;
//				abs1 = 255;
//				u8PumpOrPropValueL = 0;
//			}
			else
			{
				abs1 = 0;
				sgLiftMode.b1LiftUpL = 0;
				sgLiftMode.b1LiftDownL = 0;
				u8PumpOrPropValueL = 0;
			}
		}
		 /********** 右起升下降************/
		{
			if((i32PropValueR > 2252) &&(i32PropValueR <= 3685))     //3685
			{
				sgLiftMode.b1LiftDownR = 0;
				sgLiftMode.b1LiftUpR = 1;
				abs2 = 0;
				u8PumpOrPropValueR = ((i32PropValueR - 2251)*255)/(3685 - 2252);
			}
			else if(i32PropValueR > 3685)
			{
				sgLiftMode.b1LiftDownR = 0;
				sgLiftMode.b1LiftUpR = 1;
				abs2 = 0;
				u8PumpOrPropValueR = 255;
			}
			else if((i32PropValueR < 1843)&&(i32PropValueR >= 204))
			{
				sgLiftMode.b1LiftUpR = 0;
				sgLiftMode.b1LiftDownR = 1;
				u8PumpOrPropValueR = 0;
				abs2 = ((1843 - i32PropValueR)*255)/(1843 - 204);
			}
//			else if((i32PropValueR < 409)&&(i32PropValueR > 204))      //204
//			{
//				sgLiftMode.b1LiftUpR = 0;
//				sgLiftMode.b1LiftDownR = 1;
//				u8PumpOrPropValueR = 0;
//				abs2 = 255;
//			}
			else
			{
				abs2 = 0;
				sgLiftMode.b1LiftUpR = 0;
				sgLiftMode.b1LiftDownR = 0;
				u8PumpOrPropValueR = 0;
			}
		}
	}
	
	{
		if((((1 == sgLiftMode.b1LiftDownL)&&(0 == sgLiftMode.b1LiftUpR))
			||((0 == sgLiftMode.b1LiftUpL)&&(1 == sgLiftMode.b1LiftDownR))
			||((1 == sgLiftMode.b1LiftDownL)&&(1 == sgLiftMode.b1LiftDownR)))
			&&(0 == sgValvesInfo.b1LiftUpFlag)
			)
		{
			abs3 = abs1 > abs2 ? abs1 : abs2;
			abs3 = _IQ((sgUserInfo.fPropMinCurrent + (sgUserInfo.fPropMaxCurrent - sgUserInfo.fPropMinCurrent) * abs3 / PUMP_RANGE) / PROPD_STD_CURRENT);
			vPropSetTarget(LIFTDOWN_VALVE, abs3);
		}
		else
		{
			vPropSetTarget(LIFTDOWN_VALVE, 0);
		}
	}
/***** 转向360 *******/
	{
		memcpy((char*)CanRev360InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo360.u8Data, sizeof(CanRev360InfoLast));
		/*添加相关操作*/
		i16SteerAngle = ((CanRev360InfoLast.i16SteerAngleH<<8)|(CanRev360InfoLast.i16SteerAngleL));
		gCanSendPdoInfo.CanSend260Info.u8ErrorSteer = ERROR_CODE_STEER[CanRev360InfoLast.u8ErrSteer];
	}
	
	{
		memcpy((char*)CanRev4D1InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo4D1.u8Data, sizeof(CanRev4D1InfoLast));
		/*添加相关操作*/
	
		if(0 != (u16RentalInfo&(0x01 << 4)))
		{
			CanRev4D1InfoLast.b1NFCWork = 1;
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
	
/*********** Motor Speed *************/
	/*add Turn Dec Spd*/
	u16MotorVal = u16SteerAngleDecSpd(u16MotorVal, abs(i16SteerAngle), &sgUserInfo.SteerAngleDecSpd);
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
	u16MotorVal = (uint16_t)((u16MotorVal * u8SpeedRate)/100);
	
}


/*开关量监控*/
static void vSwiMonitor(void)
{
	xSwiInput SwiInput = {0};
	static uint16_t Snail_Count = 0;
	static uint8_t err = 0;
	
	if(1 == i32LocalDiGet(HEIGHT300_SPEEDLIMIT_SWI))
	{
		SwiInput.b1HeightSpdlimit = 1;
	}
	
	if(1 == i32LocalDiGet(HEIGHT1800_LIMT_SWI))
	{
		SwiInput.b1LIFT1800_LIMT_SWI = 1;
	}

	if(1 == i32LocalDiGet(STEER_SAFE_LOCK))
	{
		SwiInput.b1SafeLock = 1;
	}
	
	if(0 == i32LocalDiGet(FENCE1_SWI))
	{
		SwiInput.b1Fence1 = 1;
	}
	
	if(1 == i32LocalDiGet(FENCE2_SWI))
	{
		SwiInput.b1Fence2 = 1;
	}
	
	if(0 == i32LocalDiGet(LIFT_LIMT_SWI))
	{
		SwiInput.b1LIFT_LIMT_SWI = 1;
	}
	
	if(1 == i32LocalDiGet(PEDAL_SWI))
	{
		SwiInput.b1Pedal = 1;
	}
	
	if(0 == i32LocalDiGet(PeDALS_SWI))
	{
		SwiInput.b1PedalCHK = 1;
	}
	
	/******** 上位机67#参数勾选生效*******/
	
	if(0x01 == sgUserInfo.SWI_En)
	{
		SwiInput.b1Fence1 = 1;
		SwiInput.b1Fence2 = 1;
		SwiInput.b1Pedal = 1;
	}
	sgSwiInput.u16data = SwiInput.u16data;

	if((1 == sgSwiInput.b1HeightSpdlimit)&&((1 ==sgLiftMode.b1LiftUpL)||(1 == sgLiftMode.b1LiftUpR )))
	{
		sgFlagVal.Height_SpeedLimit = 1;
		u16EepromWrite(PARA_SaveState, sgSaveState.u16Data, 1);
	}
	else if((1 == sgSwiInput.b1HeightSpdlimit)&&((1 ==sgLiftMode.b1LiftDownL)||(1 == sgLiftMode.b1LiftDownR)))
	{
		sgFlagVal.Height_SpeedLimit = 0;
		u16EepromWrite(PARA_SaveState, sgSaveState.u16Data, 1);
	}
	sgSaveState.b1HeightSpdLimit = sgFlagVal.Height_SpeedLimit;
	
	if((1 == sgSwiInput.b1LIFT1800_LIMT_SWI)&&((1 ==sgLiftMode.b1LiftUpL)||(1 == sgLiftMode.b1LiftUpR )))
	{
		sgFlagVal.Heigh_1800_Limit = 1;
		u16EepromWrite(PARA_SaveState, sgSaveState.u16Data, 1);
	}
	else if((1 == sgSwiInput.b1LIFT1800_LIMT_SWI)&&((1 ==sgLiftMode.b1LiftDownL)||(1 == sgLiftMode.b1LiftDownR)))
	{
		sgFlagVal.Heigh_1800_Limit = 0;
		u16EepromWrite(PARA_SaveState, sgSaveState.u16Data, 1);
	}
	sgSaveState.b1Above1M8 = sgFlagVal.Heigh_1800_Limit;
	
	if((0 == SwiInput.b1Pedal)&&((1 == SwiInput.b1Fence1)||(1 == SwiInput.b1Fence2)))
	{
		i32ErrCodeSet(39);
		sgValvesInfo.b1NoActFlag = 1;
	}
	else if((0 == SwiInput.b1Pedal)&&(0 == SwiInput.b1Fence1)&&(0 == SwiInput.b1Fence2)&&(0 == SwiInput.b1PedalCHK))
	{
		i32ErrCodeSet(39);
		sgValvesInfo.b1NoActFlag = 1;
	}
	else if(1 == (SwiInput.b1Fence1 ^ SwiInput.b1Fence2))
	{
		i32ErrCodeSet(39);
		sgValvesInfo.b1NoActFlag = 1;
	}
	else if((0 == SwiInput.b1SafeLock)&&(0 == CanRev22CInfoLast.b1EmergencyReverse))
	{
		i32ErrCodeSet(39);
		sgValvesInfo.b1NoActFlag = 1;
	}
	else if((0 == SwiInput.b1SafeLock)&&((1 == CanRev22CInfoLast.b1Forward)||(1 == CanRev22CInfoLast.b1Backward))&&(0 == HandleInput.b1ZhiLi_Move))
	{
		i32ErrCodeSet(39);
		sgValvesInfo.b1NoActFlag = 1;
	}
	else
	{
		if((0 == check_err)&&(1 == sgValvesInfo.b1NoActFlag))
		{
			i32ErrCodeClr(39);
			sgValvesInfo.b1NoActFlag = 0;
		}		
	}
	if((0 == CanRev4D1InfoLast.b1NFCWork)
		&&((1 == HandleInput.b1Forward)||(1 == HandleInput.b1Backward)))
	{
		i32ErrCodeSet(42);
	}
	else
	{
		if(42 == u8ErrCodeGet())
			i32ErrCodeClr(42);
	}
	
	
		sgValvesInfo.b1Gear1SpdFlag = 0;
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 0;
		sgValvesInfo.b1Gear4SpdFlag = 0;
	
	if((1 == SwiInput.b1Pedal)&&(0 == SwiInput.b1Fence2)&&(0 == SwiInput.b1Fence1))
	{
		sgValvesInfo.b1Gear1SpdFlag = 1;
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 0;
		sgValvesInfo.b1Gear4SpdFlag = 0;
	}
	
	if((1 == HandleInput.b1SnailRequest)||(1 == sgFlagVal.Height_SpeedLimit))
	{
		sgValvesInfo.b1Gear1SpdFlag = 0;
		sgValvesInfo.b1Gear2SpdFlag = 1;
		sgValvesInfo.b1Gear3SpdFlag = 0;
		sgValvesInfo.b1Gear4SpdFlag = 0;
	}
	
	if(1 == CanRev22CInfoLast.b1SnailRequest)
	{
		sgValvesInfo.b1Gear1SpdFlag = 0;
		sgValvesInfo.b1Gear2SpdFlag = 1;
		sgValvesInfo.b1Gear3SpdFlag = 0;
		sgValvesInfo.b1Gear4SpdFlag = 0;
	}
	
	if((0 == SwiInput.b1Pedal)&&(0 == SwiInput.b1Fence2)&&(0 == SwiInput.b1Fence1))
	{
		sgValvesInfo.b1Gear1SpdFlag = 0;
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 0;
		sgValvesInfo.b1Gear4SpdFlag = 1;
	}
	
	if((1 == HandleInput.b1ZhiLi_Move)||(1 == sgFlagVal.Heigh_1800_Limit))
	{
		sgValvesInfo.b1Gear1SpdFlag = 0;
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 1;
		sgValvesInfo.b1Gear4SpdFlag = 0;
	}

}

static void vCanId115Proc(tCanFrame * CanFrame)
{
	xCanRev115Info CanRev115Info;
	
	memcpy((char*)CanRev115Info.u8Data, (char*)CanFrame->u8Data, sizeof(CanRev115Info));
	gCanSendPdoInfo.CanSend260Info.u8Soc = (CanRev115Info.BMS_SOC*0.4);
		if(((CanRev115Info.BMS_SOC*0.4)<=15)&&((CanRev115Info.BMS_SOC*0.4) > 10))
		{
			i32ErrCodeSet(48);
			gCanSendPdoInfo.CanSend260Info.u8ErrorBMS = ERROR_CODE_MOVE[41];
		}
		else if((CanRev115Info.BMS_SOC*0.4)<=10)
		{
			i32ErrCodeSet(58);
			gCanSendPdoInfo.CanSend260Info.u8ErrorBMS = ERROR_CODE_MOVE[42];
			sgFlagVal.BMS_NOLift = 1;
			sgFlagVal.BMS_SpeedLimit = 1;
		}
		else
		{
			if(48 == u8ErrCodeGet())
			{
				i32ErrCodeClr(48);
			}
			else if(58 == u8ErrCodeGet())
			{
				i32ErrCodeClr(58);
			}
			gCanSendPdoInfo.CanSend260Info.u8ErrorBMS = 0;
			sgFlagVal.BMS_NOLift = 0;
			sgFlagVal.BMS_SpeedLimit = 0;
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
	
	xRevCallBackProc CanId115 = {.u32CanId = 0x115, .u32Data = 0, .CallBack = vCanId115Proc};
	vCanRevMsgRegister(&CanId115);
	
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
	
	sgUserInfo.SWI_En = i32GetPara(PARA_InAndOutFunc);
	
	
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
		u16RentalInfo = i32GetPara(RENTAL_INFO);
		sgUserInfo.b1RentalStop = u16RentalInfo & 0x01;
		sgUserInfo.b1RentalStart = (u16RentalInfo >> 1) & 0x01;
		sgUserInfo.b1RentalMode = (u16RentalInfo >> 2) & 0x01;
	}
	
	u16EepromRead(PARA_DefaultFlag, &u16Tmp, 1);
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
//		sgFlagVal.Height_SpeedLimit = sgSaveState.b1HeightSpdLimit;
//		sgFlagVal.Heigh_1800_Limit = sgSaveState.b1Above1M8;
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
	gCanSendPdoInfo.CanSend260Info.u8HourCountL = (u32HourCount/10) & 0xFF;
	gCanSendPdoInfo.CanSend260Info.u8HourCountH = ((u32HourCount/10) >> 8) & 0xFF;
//	gCanSendPdoInfo.CanSend260Info.u8HourCountH = (u32HourCount >> 16) & 0xFF;
//	gCanSendPdoInfo.CanSend260Info.u8HourCountH = (u32HourCount >> 24) & 0xFF;
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
			if(1 == i32LocalDiGet(STEER_SAFE_LOCK))
			{
				check_err = 1;
				i32ErrCodeSet(39);
				sgValvesInfo.b1NoActFlag = 1;
			}
			else
			{
				u8MainConnectCnt++;
			}
		}

		if((0 == i32LocalDiGet(STEER_SAFE_LOCK))&&(1 == check_err))
		{
			check_err = 0;
			sgValvesInfo.b1NoActFlag = 0;
			i32ErrCodeClr(39);
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
			gCanSendPdoInfo.CanSend260Info.u8ErrorMove = ERROR_CODE_MOVE[u8ErrCode];
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
			if(0 == u32HourCount%10)
			{
				gCanSendPdoInfo.CanSend260Info.u8HourCountL = (u32HourCount/10) & 0xFF;
				gCanSendPdoInfo.CanSend260Info.u8HourCountH = ((u32HourCount/10) >> 8) & 0xFF;
	//			gCanSendPdoInfo.CanSend260Info.u8HourCountH = (u32HourCount >> 16) & 0xFF;
	//			gCanSendPdoInfo.CanSend260Info.u8HourCountH = (u32HourCount >> 24) & 0xFF;
			}
				__enable_irq();
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
