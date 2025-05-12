/*******************************************************************************
* Filename: UserStackerTruckProc.c	                                           *
* Description: 逻辑层的源文件，包含对应的回调函数	   						   *
* Author:                                                      *
* Date: 2023/05/30    														   *
* Revision:	 														 		   *
*******************************************************************************/
#include "UserCanComm.h"
#include "USER_HANGCHA_DGC.h"
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

#if (USER_TYPE == USER_HANGCHA_DGC)

const static xPdoParameter  sgPdoPara = 
{
	.TpdoPara = {
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 50, .u16CanId = 0x258},
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 100, .u16CanId = 0x260}
	},
	.RpdoPara = {
		{.b1Flag = 1, .b11CanRevId = 0x360},
		{.b1Flag = 1, .b11CanRevId = 0x2F0},
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


typedef union
{
	uint16_t u16data;
	struct
	{
		uint16_t b1Height500Spdlimit: 1;         //高度限速
		uint16_t b1Ems: 1;                 //急反开关
		uint16_t b1SafeLock: 1;               //互锁
		uint16_t b1Forward: 1;                  
		uint16_t b1Backward:1;            
		uint16_t b1Slow_Mode: 1;   
		uint16_t b1Pedal: 1;
		uint16_t b1StreeLock: 1;
		uint16_t b1Human: 1;
		uint16_t b1Height1800Spdlimit: 1; 
		uint16_t b1LiftUp: 1;          
		uint16_t b1LiftDown: 1;                          
		uint16_t b1FENCE1:1;       
		uint16_t b1FENCE2:1;  
		uint16_t b1LiftLimit: 1;
		uint16_t b1Reserve: 1;
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
static uint8_t check_err = 0;
static uint16_t sgu16LimitFlg = 0;
static uint32_t u32WorkCount = 0;
static uint32_t Workcnt = 0;
static uint8_t u8PowerLowLimitS = 0;
static uint8_t u8PowerLowLimitL = 0;
static uint8_t u8LimitMove = 0;
static uint8_t u8EmsErrFlg = 0;
static uint16_t Rev_Speed;
static uint8_t SpeedRate = 0;
static uint16_t RevSPEED = 0;
static uint8_t DelayFlg = 0;
static uint8_t EmsOk = 0;
static uint8_t HumanDelay = 0;
static uint8_t EMSRecive = 0;
static uint8_t BMSErrCode = 0;
static int16_t i16MotorCurrent = 0;

/*PDO接收待完善*/
xCanRevPdoInfo gCanRevPdoInfo; 
static xCanRev360Info CanRev360InfoLast;
static xCanRev2F0Info CanRev2F0InfoLast;
static xCanRev2F1Info CanRev2F1InfoLast;
/*PDO发送待完善*/
xCanSendPdoInfo gCanSendPdoInfo;

static void vMstRevProc(xMstRevPara *RevData)
{	
	uint16_t tmp = 0;
	static uint8_t Delay = 0;
	
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
	__disable_irq();
	tmp = abs(i16MotorSpd / sgUserInfo.u16RatioOfTransmission);
	gCanSendPdoInfo.CanSend260Info.u8Movespeed = tmp;
	Rev_Speed	= i16MotorSpd;
	RevSPEED = tmp;
	__enable_irq();
	
	tmp = RevData->u8MotorTmp;
	/*Motor Temp*/
	
	tmp = RevData->u8BoardTmp;
	/**/
	if(0 != i32GetPara(RENTAL_INFO))
	{
			if(sgUserInfo.Temputer > 120)          // 19 34
		{
			i32ErrCodeSet(17);
			u8LimitMove = 1;
		}
		else if(sgUserInfo.Temputer > 80)
		{
			SpeedRate = 50;
			i32ErrCodeSet(32);
		}
		else if(sgUserInfo.Temputer < -30)
		{
			i32ErrCodeSet(31);
			u8LimitMove = 1;
		}
	}
	__disable_irq();
	i16MotorCurrent = (RevData->u8CurrentHigh << 8) | RevData->u8CurrentLow;
	if((i16MotorCurrent > i32GetPara(PARA_LogLevel))&&(Delay > 120))
		SpeedRate = i32GetPara(PARA_LogModel);
	else
	{
		Delay++;
		SpeedRate = 0;
	}
		
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
	static uint32_t u16MainConnectCnt = 0;
	static uint32_t u32HumanDelay = 0;
	float fTmp = 0.0;
	xMstSendStat LastStatus;
	int16_t i16Spd = 0;
	int32_t i32PropValue = 0;
	static uint16_t u16LiftUpCnt = 0; 
	static uint8_t u8EmsDelay = 0;
		
	LastStatus.u8Data = SendData->buf[2];
	SendData->buf[2] = 0;
	
	if((0 == sgSwiInput.b1SafeLock)&&(0 == u16MotorVal))
	{
		SendData->b1PowerLineOn = 1;     // 1:断开 0:连接
	}
	
	if(0 != (sgu16LimitFlg & BMSErrSpeed_Limit))
	{
		if(u16MotorVal > (4096*30/100))
		{
			u16MotorVal = 4096*30/100;
		}
	}


	/*Move Mode*/
	if((0 == (sgu16LimitFlg & (BMSErrStop|Move_Limit|PedalMove_Limit|EmsLimitMove|EmsErrFlg)))&&(0 == u8LimitMove)&&(0 == u8EmsErrFlg))
	{
		SendData->b1ServoOn = 1;
		if(sgSwiInput.b1Backward == 1)
		{
			SendData->b1ForwardReq = 1;
		}
		else if(sgSwiInput.b1Forward == 1)
		{
//			SendData->b1ServoOn = 1;
			SendData->b1BackwardReq = 1;	
		}
		
		if((0 != sgSwiInput.b1Backward)||(0 != sgSwiInput.b1Forward))
		{
			SendData->u8TargetHigh = u16MotorVal >> 8;
			SendData->u8TargetLow = u16MotorVal;
		}

		if ((1 == sgSwiInput.b1Ems) && ((1 == SendData->b1ForwardReq) || (1 == LastStatus.b1EmsReq)))  
		{
			SendData->b1EmsReq = 1;
			if (1 == SendData->b1BackwardReq)
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
			if((1 == sgSwiInput.b1Ems)&&(0 != u16MotorVal)&&(0 == Rev_Speed))
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
	if((1 == sgSwiInput.b1LiftUp)
		&&(0 == (sgu16LimitFlg & (LiftUp_Limit|BMSErrStop)))
		&&(0 == sgSwiInput.b1LiftLimit)
		&&(0 == u8PowerLowLimitL)
		)
	{
		u16LiftUpCnt++;
		if(u16LiftUpCnt < (i32GetPara(PARA_AccAndDecLift)*100/5))
		{
			i32DoPwmSet(LIFTUP_VALVE,DRIVER_OPEN);
		}
		else
		{ 
			i32DoPwmSet(LIFTUP_VALVE,DRIVER_CLOSE);
		}
	}
	else if((1 == sgSwiInput.b1LiftDown)
				&&(0 == (sgu16LimitFlg & LiftDown_Limit))
				&&(0 == sgSwiInput.b1LiftUp)
				)
	{
		u16LiftUpCnt = 0;
		i32DoPwmSet(LIFTUP_VALVE,DRIVER_CLOSE);
		i32PropValue = _IQ((sgUserInfo.fPropMinCurrent + (sgUserInfo.fPropMaxCurrent - sgUserInfo.fPropMinCurrent) * u8PumpOrPropValue / PUMP_RANGE) / PROPD_STD_CURRENT);
		vPropSetTarget(LIFTDOWN_VALVE, i32PropValue);
	}
	else
	{
		u16LiftUpCnt = 0;
		i32DoPwmSet(LIFTUP_VALVE,DRIVER_CLOSE);
		vPropSetTarget(LIFTDOWN_VALVE, 0);
		u8PumpOrPropValue = 0;
	}
	
	if (0 == sgSwiInput.b1SafeLock)
	{
		SendData->b1ServoOn = 0;
		SendData->b1EmsReq = 0;
		SendData->b1ForwardReq = 0;
		SendData->b1BackwardReq = 0;
	}

	
	if((0 == i32LocalDiGet(Human_SWI))&&(0 == i32LocalDiGet(Pedal_SWI)))
	{
		SendData->u8TargetHigh = 0;
		SendData->u8TargetLow = 0;
	}
	
	{
		uint16_t WorkCountL = 0;
		uint16_t WorkCountH = 0;
		if(1 == SendData->b1ServoOn)
		{
			Workcnt++;
		}
		if (Workcnt >= 360*200)			/**/
		{
			Workcnt = 0;
			u32WorkCount++;
			WorkCountL = u32WorkCount;
			WorkCountH = u32WorkCount >> 16;
			u16EepromWrite(PARA_WorkCountL, WorkCountL, 1);
			u16EepromWrite(PARA_WorkCountH, WorkCountH, 1);
		}
	}
		{			
			/*lilu 20230819 For Test*/
			{
				i32SetPara(PARA_ForwardValveCurrent, u16MotorVal);		/*Send Motor Value*/
				i32SetPara(PARA_BackValveCurrent, u8PumpOrPropValue);		/*Rev Motor Value*/
				i32SetPara(PARA_PropValveCurrent, gCanSendPdoInfo.CanSend260Info.u8Data[5]);	/*Prop Current*/
				i32SetPara(PARA_LiftValveCurrent, ((gCanSendPdoInfo.CanSend260Info.u8HourCountH >> 8)|(gCanSendPdoInfo.CanSend260Info.u8HourCountL)));		/*Send Pump Value*/
				i32SetPara(PARA_OnOffValveCurrent, SendData->buf[2]);			/*Send Status*/
				i32SetPara(PARA_LoadRate, sgFlagVal.u8data);				/*Valve NoFlag*/
				i32SetPara(PARA_CalibrationStatus, sgValvesInfo.u8Data[1]);		/*Value State*/
				i32SetPara(PARA_TurnRightValveCurrent, ((uint16_t)sgValvesInfo.u8NoAct << 8) | ((uint16_t)sgValvesInfo.b4NoMove << 4) | ((uint16_t)sgValvesInfo.b4NoLiftUp));
				i32SetPara(PARA_TurnLeftValveCurrent, ((uint16_t)sgValvesInfo.b4Gear1Spd << 12) | ((uint16_t)sgValvesInfo.b4Gear2Spd << 8) | \
					((uint16_t)sgValvesInfo.b4Gear3Spd << 4) | ((uint16_t)sgValvesInfo.b4Gear4Spd));
				i32SetPara(PARA_AngleValue, (uint16_t)i32LocalAiGetValue(AI_B_AI1_R));		/*AI1*/
				i32SetPara(PARA_PressureVlaue1, (uint16_t)i32LocalAiGetValue(AI_B_AI2_R));	/*AI2*/
				i32SetPara(PARA_PressureVlaue2, DRIVER_FLAG);	/*AI3*/
				i32SetPara(PARA_ExtSignal, sgSwiInput.u16data);						/*Swi*/
				i32SetPara(PARA_PcuKeyInfo,sgValvesInfo.u8Data[0]);						/*ErrCode*/
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
	
/***** 锂电 2F0 *******/
	{
		static uint16_t u16StopMove = 0;
		static uint16_t u16CutSpeed = 0;
		memcpy((char*)CanRev2F0InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo2F0.u8Data, sizeof(CanRev2F0InfoLast));
		memcpy((char*)CanRev2F1InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo2F1.u8Data, sizeof(CanRev2F1InfoLast));
		/*添加相关操作*/
		u16StopMove = (CanRev2F0InfoLast.u8Data[6] & 0x37)
		            | (CanRev2F1InfoLast.u8Data[2] & 0x9F)
                | (CanRev2F1InfoLast.u8Data[5] & 0x4A);		//Level1 --- Stop Car
		
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
			if((CanRev2F0InfoLast.u8Data[6] & 0x08) != 0)
			{
				i32ErrCodeSet(101);
			}
			else if((CanRev2F0InfoLast.u8Data[6] & 0x02) != 0)
			{
				i32ErrCodeSet(102);
			}
			else if((CanRev2F0InfoLast.u8Data[6] & 0x40) != 0)
			{
				i32ErrCodeSet(103);
			}
			else if((CanRev2F0InfoLast.u8Data[6] & 0x20) != 0)
			{
				i32ErrCodeSet(104);
			}
			else if((CanRev2F0InfoLast.u8Data[6] & 0x01) != 0)
			{
				i32ErrCodeSet(105);
			}
			else if((CanRev2F0InfoLast.u8Data[6] & 0x10) != 0)
			{
				i32ErrCodeSet(106);
			}
			else if((CanRev2F0InfoLast.u8Data[5] & 0x08) != 0)
			{
				i32ErrCodeSet(107);
			}
			else
			{
				i32ErrCodeClr(101);
				i32ErrCodeClr(102);
				i32ErrCodeClr(103);
				i32ErrCodeClr(104);
				i32ErrCodeClr(105);
				i32ErrCodeClr(106);
				i32ErrCodeClr(107);
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
	
/*********** Motor Speed *************/
	/*add Turn Dec Spd*/
	
	i32AdcValue = i32LocalAiGetValue(MOVE_THROTTLE);
	if(i32AdcValue > sgUserInfo.u16ThrottleMax)
	{
		u16MotorVal = 4095;
	}
	else if((i32AdcValue > sgUserInfo.u16ThrottleMin)&&(i32AdcValue <= sgUserInfo.u16ThrottleMax))
	{
		u16MotorVal = (4095*(i32AdcValue - sgUserInfo.u16ThrottleMin))/(sgUserInfo.u16ThrottleMax - sgUserInfo.u16ThrottleMin);
	}
	else
	{
		u16MotorVal = 0;
	}	
	{
		i32AdcValue = i32LocalAiGetValue(LIFT_THROTTLE);
		if(i32AdcValue > sgUserInfo.u16LiftUpMax)
		{
			u8PumpOrPropValue = 255;
		}
		else if((i32AdcValue <= sgUserInfo.u16LiftUpMax)&&(i32AdcValue > sgUserInfo.u16LiftUpMin))
		{
			u8PumpOrPropValue = (255*(i32AdcValue - sgUserInfo.u16LiftUpMin))/(sgUserInfo.u16LiftUpMax - sgUserInfo.u16LiftUpMin);
		}
		else
		{
			u8PumpOrPropValue = 0;
		}
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
	
	if(SpeedRate > 0)
	{
		if (u8SpeedRate >= SpeedRate)
		{
			u8SpeedRate = SpeedRate;
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
	u16MotorVal = u16SteerAngleDecSpd(u16MotorVal, abs(i16SteerAngle), &sgUserInfo.SteerAngleDecSpd);
}


/*开关量监控*/
static void vSwiMonitor(void)
{
	xSwiInput SwiInput = {0};
	static uint8_t Snail_Count = 0;
	static uint16_t PedalCount = 0;
	
	if(1 == i32LocalDiGet(HEIGHT_500SPEEDLIMIT_SWI))
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
	
	if(1 == i32LocalDiGet(ForWard_SWI))
	{
		SwiInput.b1Forward = 1;
	}
	
	if(1 == i32LocalDiGet(BackWard_SWI))
	{
		SwiInput.b1Backward = 1;
	}
	
	if(1 == i32LocalDiGet(SnailRequest))
	{
		SwiInput.b1Slow_Mode = 1;
	}
	
		DelayFlg = 0;
	if(1 == i32LocalDiGet(Human_SWI))
	{
		DelayFlg = 1;
	}
	
	if((1 == i32LocalDiGet(Human_SWI))||(PedalCount < 400))
	{
		SwiInput.b1Human = 1;
		if(0 == i32LocalDiGet(Human_SWI)&&(0 != u16MotorVal))
		{
			PedalCount++;
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

		
	if((0 == i32LocalDiGet(ForWard_SWI))&&(0 == i32LocalDiGet(BackWard_SWI)))
	{
		sgu16LimitFlg &= ~PedalMove_Limit;
		if(1 == i32LocalDiGet(Human_SWI))
		{
			PedalCount = 0;			
		}
	}
	
	if(1 == i32LocalDiGet(StreeLock_SWI))
	{
		SwiInput.b1StreeLock = 1;
	}
	
	if(1 == i32LocalDiGet(StreeLock_SWI))
	{
		SwiInput.b1StreeLock = 1;
	}
	
	if(1 == i32LocalDiGet(Pedal_SWI))
	{
		SwiInput.b1Pedal = 1;
	}
	
	if(1 == i32LocalDiGet(HEIGHT_1800SPEEDLIMIT_SWI))
	{
		SwiInput.b1Height1800Spdlimit = 1;
	}
	
	if(1 == i32LocalDiGet(LiftUp_SWI))
	{
		SwiInput.b1LiftUp = 1;
	}
	
	if(1 == i32LocalDiGet(LiftDown_SWI))
	{
		SwiInput.b1LiftDown = 1;
	}
	
	if(1 == i32LocalDiGet(FENCE1_SWI))
	{
		SwiInput.b1FENCE1 = 1;
	}
	
	if(1 == i32LocalDiGet(FENCE2_SWI))
	{
		SwiInput.b1FENCE2 = 1;
	}
#ifdef DDC_IOTYPE
	if(0 == i32LocalDiGet(LiftLimit_SWI))
#else
	if(1 == i32LocalDiGet(LiftLimit_SWI))
#endif
	{
		SwiInput.b1LiftLimit = 1;
	}
	sgSwiInput.u16data = SwiInput.u16data;
	
#ifdef DDC_IOTYPE
	if((0 == i32LocalDiGet(Human_SWI))&&(0 == SwiInput.b1Pedal)&&(0 == Rev_Speed))
	{
		sgu16LimitFlg |=(Move_Limit|LiftUp_Limit|LiftDown_Limit);
	}
	else if((1 == SwiInput.b1Human)&&(0 == SwiInput.b1Pedal)&&(1 == SwiInput.b1FENCE1)
				&&(1 == SwiInput.b1FENCE2)&&(1 == SwiInput.b1Height1800Spdlimit)&&(1 == SwiInput.b1Height500Spdlimit))
	{
		sgu16LimitFlg &= ~(Move_Limit|LiftUp_Limit|LiftDown_Limit);
		sgValvesInfo.b1Gear1SpdFlag = 0;   // 高速
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 0;
		sgValvesInfo.b1Gear4SpdFlag = 0;
	}
	else if((1 == SwiInput.b1Human)&&(0 == SwiInput.b1Pedal)&&(1 == SwiInput.b1FENCE1)
				&&(1 == SwiInput.b1FENCE2)&&(1 == SwiInput.b1Height1800Spdlimit)&&(0 == SwiInput.b1Height500Spdlimit))
	{
		sgu16LimitFlg &= ~(Move_Limit|LiftUp_Limit|LiftDown_Limit);
		sgValvesInfo.b1Gear1SpdFlag = 0;   // 低速3
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 1;
		sgValvesInfo.b1Gear4SpdFlag = 0;
	}
	else if((1 == SwiInput.b1Human)&&(0 == SwiInput.b1Pedal)&&(1 == SwiInput.b1FENCE1)
				&&(1 == SwiInput.b1FENCE2)&&(0 == SwiInput.b1Height1800Spdlimit))
	{
		sgu16LimitFlg |= LiftUp_Limit;
		sgu16LimitFlg &= ~(Move_Limit|LiftDown_Limit);
		sgValvesInfo.b1Gear1SpdFlag = 0;   // 低速4
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 0;
		sgValvesInfo.b1Gear4SpdFlag = 1;
	}
	else if((1 == SwiInput.b1Human)&&(0 == SwiInput.b1Pedal)&&(SwiInput.b1FENCE1 != SwiInput.b1FENCE2))
	{
		sgu16LimitFlg |= Move_Limit;
		sgu16LimitFlg &= ~(LiftUp_Limit|LiftDown_Limit);
		if(0 == SwiInput.b1Height1800Spdlimit)
		{
			sgu16LimitFlg |= LiftUp_Limit;
		}
	}
	else if((1 == SwiInput.b1Human)&&(0 == SwiInput.b1Pedal)&&(0 == SwiInput.b1FENCE1)
				&&(0 == SwiInput.b1FENCE2))
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
	else if((1 == SwiInput.b1Pedal)&&(1 == SwiInput.b1FENCE1)&&(1 == SwiInput.b1FENCE2))
	{
		sgu16LimitFlg |=(Move_Limit|LiftUp_Limit);
		sgu16LimitFlg &= ~LiftDown_Limit;
	}
	else if((1 == SwiInput.b1Pedal)&&(SwiInput.b1FENCE1 != SwiInput.b1FENCE2))
	{
		sgu16LimitFlg |=(Move_Limit|LiftUp_Limit);
		sgu16LimitFlg &= ~LiftDown_Limit;
	}
	else if((1 == SwiInput.b1Pedal)&&(0 == SwiInput.b1FENCE1)&&(0 == SwiInput.b1FENCE2))
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
	
	if((1 == SwiInput.b1Slow_Mode)||(1 == u8PowerLowLimitS))
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
#else
	if((0 == SwiInput.b1Pedal)&&(0 == SwiInput.b1Human))
	{
		sgu16LimitFlg |=(Move_Limit|LiftUp_Limit|LiftDown_Limit);
	}
	else if((1 == SwiInput.b1Human)&&(0 == SwiInput.b1Pedal)&&(1 == SwiInput.b1FENCE1)&&(1 == SwiInput.b1FENCE2))
	{
		sgu16LimitFlg &= ~(Move_Limit|LiftUp_Limit|LiftDown_Limit);
		sgValvesInfo.b1Gear1SpdFlag = 0;   // 高速
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 0;
		sgValvesInfo.b1Gear4SpdFlag = 0;
	}
	else if((1 == SwiInput.b1Human)&&(0 == SwiInput.b1Pedal)&&(SwiInput.b1FENCE1 != SwiInput.b1FENCE2))
	{
		sgu16LimitFlg |= Move_Limit;
		sgu16LimitFlg &= ~(LiftUp_Limit|LiftDown_Limit);
	}
	else if((1 == SwiInput.b1Human)&&(0 == SwiInput.b1Pedal)&&(0 == SwiInput.b1FENCE1)&&(0 == SwiInput.b1FENCE2))
	{
		sgu16LimitFlg &= ~(Move_Limit|LiftUp_Limit|LiftDown_Limit);
		sgValvesInfo.b1Gear1SpdFlag = 0;   //低速1
		sgValvesInfo.b1Gear2SpdFlag = 1;
		sgValvesInfo.b1Gear3SpdFlag = 0;
		sgValvesInfo.b1Gear4SpdFlag = 0;
	}
	else if((1 == SwiInput.b1Pedal)&&(1 == SwiInput.b1FENCE1)&&(1 == SwiInput.b1FENCE2))
	{
		sgu16LimitFlg |= (Move_Limit|LiftUp_Limit);
		sgu16LimitFlg &= ~LiftDown_Limit;
	}
	else if((1 == SwiInput.b1Pedal)&&(SwiInput.b1FENCE1 != SwiInput.b1FENCE2))
	{
		sgu16LimitFlg |= (Move_Limit|LiftUp_Limit);
		sgu16LimitFlg &= ~LiftDown_Limit;
	}
	else if((1 == SwiInput.b1Pedal)&&(0 == SwiInput.b1FENCE1)&&(0 == SwiInput.b1FENCE2))
	{
		sgu16LimitFlg &= ~(Move_Limit|LiftUp_Limit|LiftDown_Limit);
		sgValvesInfo.b1Gear1SpdFlag = 0;   //低速2
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 1;
		sgValvesInfo.b1Gear4SpdFlag = 0;
	}
	
	if((1 == SwiInput.b1Slow_Mode)||(1 == u8PowerLowLimitS))
	{
		sgValvesInfo.b1Gear1SpdFlag = 0;   //龟速
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 0;
		sgValvesInfo.b1Gear4SpdFlag = 1;
	}
#endif
	
	{
		static uint8_t EmsErrtmp = 0;
		if((1 == SwiInput.b1Ems)&&(0 == EmsErrtmp))
		{
			if((1 == SwiInput.b1Forward)||(1 == SwiInput.b1Backward))
			{
				sgu16LimitFlg |= EmsErrFlg;
				i32ErrCodeSet(100);
			}
			else
			{
				sgu16LimitFlg &= ~EmsErrFlg;
				i32ErrCodeClr(100);
			}
		}
		else if((1 == SwiInput.b1Forward)||(1 == SwiInput.b1Backward))
		{
			EmsErrtmp = 1;
		}
		else
		{
			sgu16LimitFlg &= ~EmsErrFlg;
			i32ErrCodeClr(100);
			EmsErrtmp = 0;
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
	switch (basic)
	{
		case 0x10:
			gCanSendPdoInfo.CanSend258Info.Basic = 0x10;
			gCanSendPdoInfo.CanSend258Info.SOC_Choose = 1;
			gCanSendPdoInfo.CanSend258Info.Time_Choose = 1;
			gCanSendPdoInfo.CanSend258Info.Work_Count = 1;
			gCanSendPdoInfo.CanSend258Info.SpeedOn = 1;
			gCanSendPdoInfo.CanSend258Info.SpeedUint = sgUserInfo.u8Language;
			gCanSendPdoInfo.CanSend258Info.SOC = (CanRev2F0InfoLast.BMS_SOC)*0.4;
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
			
			if(((CanRev2F0InfoLast.BMS_SOC)*0.4 < 10)
				||(((CanRev2F0InfoLast.BMS_SOC)*0.4 < sgUserInfo.u8SOCLimit)&&(0 != i32GetPara(RENTAL_INFO))))
			{
				u8PowerLowLimitL = 1;
				u8PowerLowLimitS = 1;
				gCanSendPdoInfo.CanSend258Info.SOC_Warn = 1;
			}
			else if(((CanRev2F0InfoLast.BMS_SOC)*0.4 < 15)
				||(((CanRev2F0InfoLast.BMS_SOC)*0.4 < i32GetPara(PARA_AccAndDecAntiPinch))&&(0 != i32GetPara(RENTAL_INFO))))
			{
				gCanSendPdoInfo.CanSend258Info.SOC_Warn = 1;
			}
			else
			{
				u8PowerLowLimitL = 0;
				u8PowerLowLimitS = 0;
				gCanSendPdoInfo.CanSend258Info.SOC_Warn = 0;
			}
			if(1 == sgSwiInput.b1Slow_Mode)
			{
				gCanSendPdoInfo.CanSend258Info.Work_Mode = 2;
			}
			else
			{
				gCanSendPdoInfo.CanSend258Info.Work_Mode = 0;
			}
			if (0 != u8ErrCodeGet())
			{
				gCanSendPdoInfo.CanSend258Info.Contorl = 1;
				gCanSendPdoInfo.CanSend258Info.ErrCode = u8ErrCodeGet();
			}		
			else if(0 != CanRev360InfoLast.u8ErrSteer)
			{
				gCanSendPdoInfo.CanSend258Info.Contorl = 5;
				gCanSendPdoInfo.CanSend258Info.ErrCode = CanRev360InfoLast.u8ErrSteer;
			}
			else
			{
				gCanSendPdoInfo.CanSend258Info.Contorl = 1;
				gCanSendPdoInfo.CanSend258Info.ErrCode = 0;
			}
			
			if((1 == (sgu16LimitFlg & (LiftUp_Limit|BMSLiftUp_Limit)))
				||(1 == sgSwiInput.b1LiftLimit))
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
		cntTmp++;
		basic = 0x10;
	}
	else
	{
		cntTmp = 0;
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
	uint16_t WorkCountL = 0;
	uint16_t WorkCountH = 0;
	vMstRevRegister(vMstRevProc);
	vMstSendRegister(vMstSendProc);
	CanRev2F0InfoLast.BMS_SOC = 100;
	
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
	
	
	sgUserInfo.u8SOCLimit = i32GetPara(PARA_AccAndDecTurn);    //电量故障模拟
	sgUserInfo.Temputer = i32GetPara(PARA_AccAndDecAntiPinch);   //温度故障模拟
	sgUserInfo.u8Language = i32GetPara(PARA_LanguageType);
	
	u32HourCount = u32HourCountRead();
	u16EepromRead(PARA_WorkCountL, &WorkCountL, 1);
	u16EepromRead(PARA_WorkCountH, &WorkCountH, 1);
	u32WorkCount = ((WorkCountH << 16)|WorkCountL);
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

	if(true == u8GetNetTimerOverFlag(TIMER_EcuPowerOn))
	{
		u8EcuProcFlag = 1;
		vKillNetTimer(TIMER_EcuPowerOn);
	}
		if(u8MainConnectCnt < 20)
		{
			if((1 == i32LocalDiGet(ForWard_SWI))||(1 == i32LocalDiGet(BackWard_SWI))
				||(1 == i32LocalDiGet(LiftUp_SWI))||(1 == i32LocalDiGet(LiftDown_SWI))
				||(1 == i32LocalDiGet(EmergencyReverse))
			)
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
				if((0 == i32LocalDiGet(ForWard_SWI))&&(0 == i32LocalDiGet(BackWard_SWI))
					&&(0 == i32LocalDiGet(LiftUp_SWI))&&(0 == i32LocalDiGet(LiftDown_SWI))
					&&(0 == i32LocalDiGet(EmergencyReverse))
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
		vCanSendid258Tmp();
		u8ErrCode = u8ErrCodeGet();
		if (0 != u8ErrCode)
		{
				vLedSendAlmCode(u8ErrCode);
		}
	}
	{
		static uint8_t hourcounttmp = 0;
		if((255 == i32GetPara(RENTAL_INFO))&&(0 == hourcounttmp))
		{
			hourcounttmp = 1;
			vHourCountWrite(0);
			u16EepromWrite(PARA_WorkCountL, 0, 1);
			u16EepromWrite(PARA_WorkCountH, 0, 1);
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
		}
	}
	vWdgSetFun(WDG_USER_BIT);
}
#endif
