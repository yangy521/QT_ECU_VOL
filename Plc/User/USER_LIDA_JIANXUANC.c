/*******************************************************************************
* Filename: UserStackerTruckProc.c	                                           *
* Description: 逻辑层的源文件，包含对应的回调函数	   						   *
* Author:                                                      *
* Date: 2023/05/30    														   *
* Revision:	 														 		   *
*******************************************************************************/
#include "UserCanComm.h"
#include "User_LIDA_JIANXUANC.h"
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

#if (USER_ECU_OR_ET_TYPE == USER_LIDA_JIANXUANC)

const static xPdoParameter  sgPdoPara = 
{
	.TpdoPara = {
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 30, .u16CanId = 0x260},
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 500, .u16CanId = 0x3BB},
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 500, .u16CanId = 0x3BC},
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 500, .u16CanId = 0x3BE},
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 100, .u16CanId = 0x488},
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 100, .u16CanId = 0x489}
	},
	.RpdoPara = {
		{.b1Flag = 1, .b11CanRevId = 0x444},
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

typedef union
{
	uint16_t u16data;
	struct
	{
		uint16_t b1HeightSpdlimit: 1;         //高度限速
		uint16_t b1Ems: 1;                    //紧急反向
		uint16_t b1SafeLock: 1;               //互锁
		uint16_t b1ForWard: 1;
		uint16_t b1BackWard: 1;
		uint16_t b1StreeLimitSpeed: 1;
		uint16_t b1LiftLimit: 1;              //起升限位
		uint16_t b1DianDong: 1;
		uint16_t b1LiftUp: 1;           //起升标志
		uint16_t b1LiftDown: 1;        //下降标志
		uint16_t b1noAct:1;         //禁止行走
		uint16_t b5Reserve:5;
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
	
	uint16_t u16DianDongSpeedTime;
	
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
static xSwiInput sgSwiInput = {0};
static uint32_t u32HourCount = 0;
static int16_t i16SteerAngle = 0;
static int16_t i16SHanderValue = 0;
static uint8_t MOVE_FLAG = 0;
static uint8_t LIFT_FLAG = 0;
static uint8_t HANDLE_FLAG = 0;
static uint8_t LR_FLAG = 0;
static uint16_t DRIVER_FLAG = 0;
static uint8_t SnailRequest_Flag = 0;
static int32_t i32PropValue = 0;
static uint16_t LIMIT_LIFTUP = 0;
static uint16_t ECU_LOGIC_ERR = 0;
/*PDO接收待完善*/
xCanRevPdoInfo gCanRevPdoInfo; 
static xCanRev444Info CanRev444InfoLast;
//{
//	.CanRevInfo1.u8Soc = 100,					/*BmsSoc Default: 100*/
//};				
/*PDO发送待完善*/
xCanSendPdoInfo gCanSendPdoInfo;			

static void vMstRevProc(xMstRevPara *RevData)
{	
	uint16_t tmp = 0;
	
	if(0 != RevData->b1MainDriver)
	{
		gCanSendPdoInfo.CanSend3BBInfo.b1Main_Connect = 1;
		gCanSendPdoInfo.CanSend488Info.b1MainConnect = 1;
	}
	else
	{
		gCanSendPdoInfo.CanSend488Info.b1MainConnect = 0;
		gCanSendPdoInfo.CanSend3BBInfo.b1Main_Connect = 0;
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
	tmp = i16MotorSpd / sgUserInfo.u16RatioOfTransmission;
	gCanSendPdoInfo.CanSend3BBInfo.SpeedfdbH = (tmp >> 8);
	gCanSendPdoInfo.CanSend3BBInfo.SpeedfdbL = tmp;
	gCanSendPdoInfo.CanSend489Info.SpeedH = (tmp >> 8);
	gCanSendPdoInfo.CanSend489Info.SpeedL = tmp;
	/*Motor Speed*/

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
	static uint16_t DianDongLast = 0;
	static int32_t DianDongTime = 0;
//	uint8_t u8Flag = 0;
		
	LastStatus.u8Data = SendData->buf[2];
	SendData->buf[2] = 0;
	
	if((1 == sgSwiInput.b1DianDong)&&(0 == DianDongLast))
	{
		DianDongTime = sgUserInfo.u16DianDongSpeedTime;
	}
	else
	{
		if(DianDongTime > 0)
		{
			DianDongTime --;
			sgSwiInput.b1BackWard = 1;
			u16MotorVal = 4096*sgValvesInfo.b1Gear2SpdFlag;
		}
	}
	DianDongLast = sgSwiInput.b1DianDong;
	
	SendData->b1ServoOn = 1;

	if((sgSwiInput.b1BackWard == 1)&&((sgSwiInput.b1ForWard == 0)))
	{
		gCanSendPdoInfo.CanSend3BBInfo.b2Gear_State = 2;
		SendData->b1ForwardReq = 1;
	}
	else if((sgSwiInput.b1ForWard == 1)&&(sgSwiInput.b1BackWard == 0))
	{
		gCanSendPdoInfo.CanSend3BBInfo.b2Gear_State = 1;
		SendData->b1BackwardReq = 1;
	}
	else
	{
		gCanSendPdoInfo.CanSend3BBInfo.b2Gear_State = 0;
	}
	
	if(0 == (ECU_LOGIC_ERR & (NO_MOVE|Lift_ERR)))
	{
		SendData->u8TargetHigh = u16MotorVal >> 8;
		SendData->u8TargetLow = u16MotorVal;
		if ((1 == sgSwiInput.b1Ems) && ((1 == SendData->b1ForwardReq) || (1 == LastStatus.b1EmsReq)))  
		{
			SendData->b1EmsReq = 1;
			if (1 == SendData->b1ForwardReq)
			{
				sgValvesInfo.b4NoMove |= 1 << NoMove_Ems;
				i32ErrCodeSet(MOVE_EMS_ERR);
			}
	//		SendData->b1ForwardReq = 1;
		}
	}
	if (0 == sgSwiInput.b1SafeLock)
	{
		SendData->b1ServoOn = 0;
	}

	{
		static uint32_t u32Cnt = 0;
		u32Cnt++;
		{			
			/*lilu 20230819 For Test*/
			{
				i32SetPara(PARA_ForwardValveCurrent, i32PropValue);		/*Send Motor Value*/
				i32SetPara(PARA_BackValveCurrent, u8PumpOrPropValue);		/*Rev Motor Value*/
				i32SetPara(PARA_PropValveCurrent, inserted_data[0] * PROP_CURRENT_FACOTR * 1000);	/*Prop Current*/
				i32SetPara(PARA_LiftValveCurrent, u16MotorVal);		/*Send Pump Value*/
				i32SetPara(PARA_OnOffValveCurrent, SendData->buf[2]);			/*Send Status*/
				i32SetPara(PARA_LoadRate, sgValvesInfo.u8Data[0]);				/*Valve NoFlag*/
				i32SetPara(PARA_CalibrationStatus, sgValvesInfo.u8Data[1]);		/*Value State*/
				i32SetPara(PARA_TurnRightValveCurrent, ((uint16_t)sgValvesInfo.u8NoAct << 8) | ((uint16_t)sgValvesInfo.b4NoMove << 4) | ((uint16_t)sgValvesInfo.b4NoLiftUp));
				i32SetPara(PARA_TurnLeftValveCurrent, ((uint16_t)sgValvesInfo.b4Gear1Spd << 12) | ((uint16_t)sgValvesInfo.b4Gear2Spd << 8) | \
					((uint16_t)sgValvesInfo.b4Gear3Spd << 4) | ((uint16_t)sgValvesInfo.b4Gear4Spd));
				i32SetPara(PARA_AngleValue, (uint16_t)i32LocalAiGetValue(AI_B_AI1_R));		/*AI1*/
				i32SetPara(PARA_PressureVlaue1, (uint16_t)i32LocalAiGetValue(AI_B_AI2_R));	/*AI2*/
				i32SetPara(PARA_PressureVlaue2, (uint16_t)i32LocalAiGetValue(AI_B_AI3_R));	/*AI3*/
				i32SetPara(PARA_ExtSignal, sgSwiInput.u16data);						/*Swi*/
//				i32SetPara(PARA_PcuKeyInfo,HandleInput.u16data);						/*ErrCode*/
//				i32SetPara(PARA_BmsSoc, gCanSendPdoInfo.CanSend23CInfo.u8Soc);		/*BMS SOC*/
				i32SetPara(PARA_BrakeValveCurrent, abs(i16SteerAngle));
				
			}
		}
	}
}

static void vDoPwmErrCallBack(eDoPwmNo DoPwmNo)
{
	switch((uint8_t)DoPwmNo)
	{
		case LIFTUP_VALVE:
			i32ErrCodeSet(LIFTUP_VALVE_ERR);
			/*add ErrCode*/
			break;
		case LIFTDOWN_VALVE:
			i32ErrCodeSet(REVERSEA_VALVE_ERR);
			/*add ErrCode*/
			break;
		default:
			break;
	}
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


/*接收PDO处理函数*/
static void vCanRevPdoProc(void)
{
	static uint16_t u16CanRev1ACCnt;
/************  锂电协议*******************/	
	{
		memcpy((char*)CanRev444InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo444.u8Data, sizeof(CanRev444InfoLast));
		gCanSendPdoInfo.CanSend260Info.u8Soc = CanRev444InfoLast.BMS_SOC;
		gCanSendPdoInfo.CanSend488Info.BatterSoc = CanRev444InfoLast.BMS_SOC;
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
	static uint32_t u16Prop = 0;
	
/*********** Motor Speed *************/
	i32AdcValue = i32LocalAiGetValue(MOVE_THROTTLE);
	
	gCanSendPdoInfo.CanSend3BBInfo.MotorValL = i32AdcValue;
	gCanSendPdoInfo.CanSend3BBInfo.MoterValH = (i32AdcValue>>8);

	/************   前进      ******************/
	{
		i32AdcValue = i32LocalAiGetValue(MOVE_THROTTLE);
		if((i32AdcValue > sgUserInfo.u16ThrottleMin)&&(i32AdcValue < sgUserInfo.u16ThrottleMax))
		{
			u16MotorVal = (((i32AdcValue - sgUserInfo.u16ThrottleMin)*4096)/(sgUserInfo.u16ThrottleMax-sgUserInfo.u16ThrottleMin));		
		}
		else
		{
			u16MotorVal = 0;
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
	gCanSendPdoInfo.CanSend3BBInfo.b1Emergency = 0;
	gCanSendPdoInfo.CanSend3BBInfo.b1ZuoYi_State = 0;
	gCanSendPdoInfo.CanSend3BCInfo.b1HightSpeedLimit = 0;
	gCanSendPdoInfo.CanSend3BCInfo.b1StreeSpeetLimit = 0;
	
	if(1 == i32LocalDiGet(HEIGHT_SPEEDLIMIT_SWI))
	{
		gCanSendPdoInfo.CanSend3BCInfo.b1HightSpeedLimit = 1;
		SwiInput.b1HeightSpdlimit = 1;
	}

	if(1 == i32LocalDiGet(Emergency_SWI))
	{
		gCanSendPdoInfo.CanSend3BBInfo.b1Emergency = 1;
		SwiInput.b1Ems = 1;
	}
	
	if(1 == i32LocalDiGet(SAFELOCK_SWI))
	{
		SwiInput.b1SafeLock = 1;
		gCanSendPdoInfo.CanSend3BBInfo.b1ZuoYi_State = 1;
	}
	
	if(1 == i32LocalDiGet(ForWard_SWI))
	{
		SwiInput.b1ForWard = 1;
	}

	if(1 == i32LocalDiGet(BackWard_SWI))
	{
		SwiInput.b1BackWard = 1;
	}
		
	if(1 == i32LocalDiGet(StreeLimitSpeed_SWI))
	{
		gCanSendPdoInfo.CanSend3BCInfo.b1StreeSpeetLimit = 1;
		SwiInput.b1StreeLimitSpeed = 1;
	}
	
	if(0 == i32LocalDiGet(LIFT_LIMT_SWI))   //常闭开关
	{
		SwiInput.b1LiftLimit = 1;
	}

	if(1 == i32LocalDiGet(DianBackWard_SWI))
	{
		SwiInput.b1DianDong = 1;
	}
	
	
	gCanSendPdoInfo.CanSend3BBInfo.b1LiftUp_Mode = 0;
	gCanSendPdoInfo.CanSend3BBInfo.b1LiftDown = 0;
	if(1 == i32LocalDiGet(LiftUp_SWI))
	{
		gCanSendPdoInfo.CanSend3BBInfo.b1LiftUp_Mode = 1;
		SwiInput.b1LiftUp = 1;
	}
	
	if(1 == i32LocalDiGet(LiftDown_SWI))
	{
		gCanSendPdoInfo.CanSend3BBInfo.b1LiftDown = 1;
		SwiInput.b1LiftDown = 1;
	}
	sgSwiInput.u16data = SwiInput.u16data;
	
	/*LIFT Mode*/
	i32DoPwmSet(LIFTUP_VALVE,DRIVER_CLOSE);
	i32DoPwmSet(LIFTDOWN_VALVE,DRIVER_CLOSE);
	if(0 == (ECU_LOGIC_ERR & Lift_ERR))
	{
		if(1 == SwiInput.b1LiftDown)
		{
			i32DoPwmSet(LIFTDOWN_VALVE,DRIVER_OPEN);
		}
		else if((1 == SwiInput.b1LiftUp)&&(0 == SwiInput.b1LiftLimit))
		{
			i32DoPwmSet(LIFTUP_VALVE,DRIVER_OPEN);
		}
		else //if((1 == SwiInput.b1LiftDown)&&(1 == SwiInput.b1LiftUp))
		{
			
		}
	}

	
	/*Speed limit*/

		sgValvesInfo.b1Gear1SpdFlag = 0;
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 0;
		sgValvesInfo.b1Gear4SpdFlag = 0;
	if(1 == SwiInput.b1HeightSpdlimit)
	{
		sgValvesInfo.b1Gear1SpdFlag = 0;
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 1;
		sgValvesInfo.b1Gear4SpdFlag = 0;
	}
	else if(1 == SwiInput.b1StreeLimitSpeed)
	{
		sgValvesInfo.b1Gear1SpdFlag = 0;
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 0;
		sgValvesInfo.b1Gear4SpdFlag = 1;
	}
	
	if(1 == SwiInput.b1Ems)
	{
		ECU_LOGIC_ERR |= Ems_ERR;
	}
	else 
	{
		ECU_LOGIC_ERR &= ~Ems_ERR;
	}
	if(0 != (ECU_LOGIC_ERR & (Safelock_ERR|ForWard_ERR|Ems_ERR)))
	{
		ECU_LOGIC_ERR |= NO_MOVE;
	}
	else
	{
		ECU_LOGIC_ERR |= NO_MOVE;
	}
	if(0 != ECU_LOGIC_ERR)
	{
		i32ErrCodeSet(39);
	}
	else
	{
		i32ErrCodeClr(39);
	}

}


//static void vCanId3BBSend()
//{
//	memset(gCanSendPdoInfo.CanSend3BBInfo.u8Data,0,sizeof(gCanSendPdoInfo.CanSend3BBInfo.u8Data));
//	 if((1 == sgSwiInput.b1BackWard)&&(0 == sgSwiInput.b1ForWard))
//	 {
//		 gCanSendPdoInfo.CanSend3BBInfo.b2Gear_State = 2;
//	 }
//	 else if((0 == sgSwiInput.b1BackWard)&&(1 == sgSwiInput.b1ForWard))
//	 {
//		 gCanSendPdoInfo.CanSend3BBInfo.b2Gear_State = 1;
//	 }
//	 else
//	 {
//		 gCanSendPdoInfo.CanSend3BBInfo.b2Gear_State = 0;
//	 }
//}


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
	sgUserInfo.u16DianDongSpeedTime = i32GetPara(PARA_MinAngle);   //借用112#
	
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
		sgSwiInput.b1HeightSpdlimit = sgSaveState.b1HeightSpdLimit;
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
	gCanSendPdoInfo.CanSend488Info.HourCountL = u32HourCount & 0xFF;
	gCanSendPdoInfo.CanSend488Info.HourCountH = (u32HourCount >> 8) & 0xFF;
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
	static uint16_t u16PoweronCnt = 0;

	if(true == u8GetNetTimerOverFlag(TIMER_EcuPowerOn))
	{
		u8EcuProcFlag = 1;
		vKillNetTimer(TIMER_EcuPowerOn);
	}
	if(u16PoweronCnt < 100)
	{
		u16PoweronCnt++;
		if(1 == i32LocalDiGet(Emergency_SWI))
			ECU_LOGIC_ERR |= Ems_ERR;
		if(1 == i32LocalDiGet(SAFELOCK_SWI))
			ECU_LOGIC_ERR |= Safelock_ERR;
		if((1 == i32LocalDiGet(ForWard_SWI))||(1 == i32LocalDiGet(BackWard_SWI)))
			ECU_LOGIC_ERR |= ForWard_ERR;
		if((1 == i32LocalDiGet(LiftUp_SWI))||(1 == i32LocalDiGet(LiftDown_SWI)))
			ECU_LOGIC_ERR |= Lift_ERR;
	}
	else
	{
		if(0 == i32LocalDiGet(Emergency_SWI))
			ECU_LOGIC_ERR &= ~Ems_ERR;
		if(0 == i32LocalDiGet(SAFELOCK_SWI))
			ECU_LOGIC_ERR &= ~Safelock_ERR;
		if((0 == i32LocalDiGet(ForWard_SWI))&&(0 == i32LocalDiGet(BackWard_SWI)))
			ECU_LOGIC_ERR &= ~ForWard_ERR;
		if((0 == i32LocalDiGet(LiftUp_SWI))&&(0 == i32LocalDiGet(LiftDown_SWI)))
			ECU_LOGIC_ERR &= ~Lift_ERR;
	}
	
	if(1 == u8EcuProcFlag)
	{
		vSwiMonitor();		
		vCanRevPdoProc();
		vAiMonitor();
		u8ErrCode = u8ErrCodeGet();
		if(0 != u8ErrCode)
		{
			gCanSendPdoInfo.CanSend488Info.ErrCode = u8ErrCode;
			gCanSendPdoInfo.CanSend3BBInfo.b1Err_State = 1;
		}
		else
		{
			gCanSendPdoInfo.CanSend488Info.ErrCode = 0;
			gCanSendPdoInfo.CanSend3BBInfo.b1Err_State = 0;
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
			gCanSendPdoInfo.CanSend260Info.u8HourCountL = (u32HourCount >> 8) & 0xFF;
			gCanSendPdoInfo.CanSend488Info.HourCountL = u32HourCount & 0xFF;
			gCanSendPdoInfo.CanSend488Info.HourCountH = (u32HourCount >> 8) & 0xFF;
			__enable_irq();
		}
		
		gCanSendPdoInfo.CanSend260Info.u8Soc = u8GetBatterySoc();
		
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
