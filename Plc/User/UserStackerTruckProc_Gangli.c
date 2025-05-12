/*******************************************************************************
* Filename: UserStackerTruckProc.c	                                           *
* Description: 逻辑层的源文件，包含对应的回调函数	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/05/30    														   *
* Revision:	 														 		   *
*******************************************************************************/
#include "UserCanComm.h"
#include "UserStackerTruckProc_Gangli.h"
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

#if (USER_TYPE == USER_GANGLI_DGC)

//const static xErrCodeInfo sgErrCodeInfo = 
//{
//	
//};

const static xPdoParameter  PdoPara = 
{
	.TpdoPara = {
		{.u8Flag = 1, .u8Type = 0x0, .u16Period = 200},
		{.u8Flag = 1, .u8Type = 0x0, .u16Period = 200},
		{.u8Flag = 1, .u8Type = 0x0, .u16Period = 200},
		{.u8Flag = 1, .u8Type = 0x0, .u16Period = 200},
	},
	.RpdoPara = {
		{.b1Flag = 1, .b11CanRevId = 0x1AC},
		{.b1Flag = 1, .b11CanRevId = 0x360},
		{.b1Flag = 0, .b11CanRevId = 0x000},
		{.b1Flag = 0, .b11CanRevId = 0x000},
		{.b1Flag = 0, .b11CanRevId = 0x000},
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
#define	NoAct_LowBat		3

#define NoLiftUp_HeightLimit	0		
#define NoLiftUp_FaultLock		1
#define	NoLiftUp_LowBat			2

#define	NoMove_Pedal			0
#define	NoMove_FaultLock		1
#define	NoMove_Ems				2

typedef union
{
	uint16_t u16data;
	struct
	{
		uint16_t b1GuardRaild: 1;
		uint16_t b1Ems: 1;
		uint16_t b1Lock: 1;
		uint16_t b1Forward: 1;
		uint16_t b1Backward: 1;
		uint16_t b1LiftDown: 1;
		uint16_t b1LiftUp: 1;
		uint16_t b1PedalOpen: 1;
		uint16_t b1PedalClose: 1;
		uint16_t b1FaultLockOut: 1;
		uint16_t b1Meter1m8: 1;
		uint16_t b1LeanForward: 1;
		uint16_t b1LeanBackward: 1;
		uint16_t b1UpLimit: 1;
		uint16_t b1Above1M8: 1;
		uint16_t b1Reserve: 1;
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
		uint8_t b2Reserve2: 1;
		
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
	uint8_t		b4Reserve: 7;
	
	uint8_t		u8BatteryType;
	
	uint16_t	u16RatioOfTransmission;
	uint16_t	u16MotorMaxSpd;
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
		uint16_t b1HeightLimit: 1;
		uint16_t b1SpdMode: 1;
		uint16_t b1Security: 1;
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
static uint32_t u32HourCount = 0;
xCanRevPdoInfo gCanRevPdoInfo;	/*PDO接收待完善*/
xCanSendPdoInfo gCanSendPdoInfo; /*PDO发送待完善*/
			
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
	i32LogWrite(DEBUG, LOG_USER, "MotorRev Spd = %d, McuErrCode = %d\r\n", i16MotorSpd, RevData->u8ErrCode);
//	__disable_irq();
//	tmp = i16MotorSpd / sgUserInfo.u16RatioOfTransmission;
//	/*Motor Speed*/
//	gCanSendPdoInfo.CanSend33CInfo.u8SpdHighByte = tmp >> 8;
//	gCanSendPdoInfo.CanSend33CInfo.u8SpdLowByte = tmp;
//	__enable_irq();
	
//	tmp = RevData->u8MotorTmp;
	/*Motor Temp*/
//	gCanSendPdoInfo.CanSend43CInfo.i16MotorTemp = RevData->u8MotorTmp;
//	
//	tmp = RevData->u8BoardTmp;
//	/**/
//	gCanSendPdoInfo.CanSend43CInfo.i16CtrlTemp = RevData->u8BoardTmp;
//	
//	__disable_irq();
//	gCanSendPdoInfo.CanSend43CInfo.i16MotorCurrent = (RevData->u8CurrentHigh << 8) | RevData->u8CurrentLow;
//	__enable_irq();
}

static void vMoveModeNoChangeProc(int16_t *i16Spd, const xValvesInfo *pValvesInfo)
{
	float fTmp = 0;
	if ((1 == pValvesInfo->b1ForWardStat) || (1 == pValvesInfo->b1BackWardStat))
	{
		if (*i16Spd > u16MotorVal)
		{
			fTmp = sgUserInfo.fMoveSpdPer5msDecStep * (0.2 + 0.8 * *i16Spd / MOTOR_SPEED_RANGE);
//			if (((abs(*i16Spd - u16MotorVal) > sgUserInfo.fMoveSpdPer5msDecStep) && (0 != sgUserInfo.fMoveSpdPer5msDecStep)))
			if (((*i16Spd - u16MotorVal) > fTmp) && (0 != sgUserInfo.fMoveSpdPer5msDecStep))
			{
//				*fMotorSpd -= sgUserInfo.fMoveSpdPer5msDecStep * (0.2 + 1.0 * i16MotorSpd / MOTOR_MAX_SPEED);
//				*i16Spd = (int16_t)*fMotorSpd;
//				*i16Spd -= sgUserInfo.fMoveSpdPer5msAccStep * (0.2 + 0.8 * *i16Spd / MOTOR_SPEED_RANGE);
				*i16Spd -= fTmp;
			}
			else
			{
				*i16Spd = u16MotorVal;
//				*fMotorSpd = u16MotorVal;
			}
		}
		else if (*i16Spd < u16MotorVal)
		{
			fTmp = sgUserInfo.fMoveSpdPer5msAccStep * (0.2 + 0.8 * *i16Spd / MOTOR_SPEED_RANGE);
//			if (((u16MotorVal - *i16Spd) > sgUserInfo.fMoveSpdPer5msAccStep) && (0 != sgUserInfo.fMoveSpdPer5msAccStep
			if (((u16MotorVal - *i16Spd) > fTmp) && (0 != sgUserInfo.fMoveSpdPer5msAccStep))
			{
//				*fMotorSpd += sgUserInfo.fMoveSpdPer5msAccStep * (0.2 + 1.0 * i16MotorSpd / MOTOR_MAX_SPEED);
//				*i16Spd = (int16_t)*fMotorSpd;
//				*i16Spd += sgUserInfo.fMoveSpdPer5msAccStep * (0.2 + 0.8 * *i16Spd / MOTOR_SPEED_RANGE);
				*i16Spd += fTmp;
			}
			else
			{
				*i16Spd = u16MotorVal;
//				*fMotorSpd = u16MotorVal;
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
	
	float fTmp = 0.0;
	xMstSendStat LastStatus;
	int16_t i16Spd = 0;
	uint8_t u8Flag = 0;
		
	LastStatus.u8Data = SendData->buf[2];
	SendData->buf[2] = 0;
	 
	if (1 == gCanRevPdoInfo.CanRevInfo1.b1MainDriverOpen)		/*add Main Connector*/
	{
		SendData->b1PowerLineOn = 1;
	}
	
	i16Spd = (SendData->u8TargetHigh << 8) | SendData->u8TargetLow;	
	i16Spd = abs(i16Spd);
	
	/*Move Mode*/
	if ((sgLastValvesInfo.b1ForWardStat != sgValvesInfo.b1ForWardStat) || 
		(sgLastValvesInfo.b1BackWardStat != sgValvesInfo.b1BackWardStat))
	{
		if ((0 == i16Spd) ||
		   ((1 == LastStatus.b1BackwardReq) && (1 == sgValvesInfo.b1ForWardStat)) || 
		   ((1 == LastStatus.b1ForwardReq) && (1 == sgValvesInfo.b1BackWardStat)))
		{
			sgLastValvesInfo.b1ForWardStat = sgValvesInfo.b1ForWardStat;
			sgLastValvesInfo.b1BackWardStat = sgValvesInfo.b1BackWardStat;
		}
		else 
		{
			if (((1 == sgLastValvesInfo.b1ForWardStat) && (0 == sgValvesInfo.b1ForWardStat)) ||
				((1 == sgLastValvesInfo.b1BackWardStat) && (0 == sgValvesInfo.b1BackWardStat)))
			{
				if (0 == u8MoveSwitchFlag)
				{
					uint16_t u16Tmp = 0;
					if (0 != sgUserInfo.u16MotorMaxSpd)
					{
						u16Tmp = abs(i16MotorSpd) * MOTOR_SPEED_RANGE / sgUserInfo.u16MotorMaxSpd;	/*CloseLoop */
					}
					else
					{
						u16Tmp = abs(i16MotorSpd) * MOTOR_SPEED_RANGE / MOTOR_MAX_SPEED;	/*CloseLoop */
					}
					if (u16Tmp < i16Spd)
					{
						i16Spd = u16Tmp;
					}
					u8MoveSwitchFlag = 1;
				}
				fTmp = sgUserInfo.fMoveSpdPer5msDecStep * (0.2 + 0.8 * i16Spd / MOTOR_SPEED_RANGE);
				if ((abs(i16MotorSpd) > MOTOR_MIN_SPEED) && (abs(i16Spd) > (uint16_t)(fTmp + 1)))
//				if ((abs(i16MotorSpd) > MOTOR_MIN_SPEED) && (abs(i16Spd) > (uint16_t)(sgUserInfo.fMoveSpdPer5msDecStep + 1)))
				{
//					i16Spd -= sgUserInfo.fMoveSpdPer5msDecStep * (0.2 + 0.8 * i16Spd / MOTOR_SPEED_RANGE);
					i16Spd -= fTmp;
//					fMotorSpd -= sgUserInfo.fMoveSpdPer5msDecStep * (0.2 + 1.0 * i16MotorSpd / MOTOR_MAX_SPEED);
//					i16Spd = (int16_t)fMotorSpd;
				}
				else
				{
					i16Spd = 0;
//					fMotorSpd = 0;
				}
			}
		}
	}
	else
	{
		vMoveModeNoChangeProc(&i16Spd, &sgLastValvesInfo);
		u8MoveSwitchFlag = 0;
	}
	
	if (i16Spd >= MOTOR_MAX_SPEED_VALUE)
	{
		i16Spd = MOTOR_MAX_SPEED_VALUE;
	}
	
	if (0 != i16Spd)
	{
		u8Flag = 1 << 0;
		if (1 == sgLastValvesInfo.b1ForWardStat)
		{
			SendData->b1ForwardReq = 1;
		}
		else if (1 == sgLastValvesInfo.b1BackWardStat)
		{
			SendData->b1BackwardReq = 1;
		}
	}
//	//i32LogWrite(INFO, "SendSpd********** = %d******************\r\n", i16Spd);
	SendData->u8TargetHigh = i16Spd >> 8;
	SendData->u8TargetLow = i16Spd;

//	if ((0 == (sgValvesInfo.b4NoMove & (1 << NoMove_Ems))) && (1 == LastStatus.b1EmsReq))
//	{
//		LastStatus.b1EmsReq = 0;
//	}
	/*20230803 add ems*/
	//if (1 == sgSwiInput.b1Ems)
	if ((1 == sgSwiInput.b1Ems) && ((1 == SendData->b1ForwardReq) || (1 == LastStatus.b1EmsReq)))  
	{
		SendData->b1EmsReq = 1;
		if (1 == SendData->b1ForwardReq)
		{
			sgValvesInfo.b4NoMove |= 1 << NoMove_Ems;
			i32ErrCodeSet(MOVE_EMS_ERR);
		}
		SendData->b1ForwardReq = 1;
	}
	else
	{
		SendData->b1EmsReq = 0;
	}
	
	/*Lift Mode*/
	if ((sgLastValvesInfo.b1LiftUpStat != sgValvesInfo.b1LiftUpStat) ||
		(sgLastValvesInfo.b1LiftDownStat != sgValvesInfo.b1LiftDownStat) ||
		(sgLastValvesInfo.b1LeanForWardStat != sgValvesInfo.b1LeanForWardStat) ||
		(sgLastValvesInfo.b1LeanBackWardStat != sgValvesInfo.b1LeanBackWardStat))	
	{
		if ((inserted_data[0] * PROP_CURRENT_FACOTR) < sgUserInfo.fPropMinCurrent)
		{
			sgLastValvesInfo.b1LiftUpStat = sgValvesInfo.b1LiftUpStat;
			sgLastValvesInfo.b1LiftDownStat = sgValvesInfo.b1LiftDownStat;
			sgLastValvesInfo.b1LeanForWardStat = sgValvesInfo.b1LeanForWardStat;
			sgLastValvesInfo.b1LeanBackWardStat = sgValvesInfo.b1LeanBackWardStat;
			if (1 == sgLastValvesInfo.b1LiftDownStat)
			{
				i32DoPwmSet(LIFTUP_VALVE, DRIVER_CLOSE);
			}
			else if(1 == sgLastValvesInfo.b1LiftUpStat)
			{
				vPropSetTarget(LIFTDOWN_VALVE, 0);
				i32DoPwmSet(LIFTUP_VALVE, DRIVER_OPEN);
			}
			else 
			{
				vPropSetTarget(LIFTDOWN_VALVE, 0);
				i32DoPwmSet(LIFTUP_VALVE, DRIVER_CLOSE);
			}
		}
		else
		{
			vPropSetTarget(LIFTDOWN_VALVE, 0);
		}
	}
	else
	{
		if (1 == sgLastValvesInfo.b1LiftDownStat)
		{
			int32_t i32PropValue = 0;
			i32PropValue = _IQ((sgUserInfo.fPropMinCurrent + (sgUserInfo.fPropMaxCurrent - sgUserInfo.fPropMinCurrent) * u8PumpOrPropValue / PUMP_RANGE) / PROPD_STD_CURRENT);
			vPropSetTarget(LIFTDOWN_VALVE, i32PropValue);
			/*Down, Pump Stop*/
			if (SendData->u8PumpTarget > 0) 
			{
				fTmp = sgUserInfo.fLiftSpdPer5msDecStep * (0.2 + 0.8 * SendData->u8PumpTarget / PUMP_RANGE);
				if ((SendData->u8PumpTarget > fTmp) && (0 != sgUserInfo.fLiftSpdPer5msDecStep))
				{
					SendData->u8PumpTarget -= fTmp;
				}
				else
				{
					SendData->u8PumpTarget = 0; 
				}
//				if ((SendData->u8PumpTarget > sgUserInfo.fLiftSpdPer5msDecStep) && (0 != sgUserInfo.fLiftSpdPer5msDecStep))
//				{
//					fPumpMototSpd -= sgUserInfo.fLiftSpdPer5msDecStep;
//					SendData->u8PumpTarget = fPumpMototSpd;
//				}
//				else
//				{
//					SendData->u8PumpTarget = 0; 
//					fPumpMototSpd = 0.0;
//				}
			}
		}
		else if ((1 == sgLastValvesInfo.b1LiftUpStat) || (1 == sgLastValvesInfo.b1LeanForWardStat) || (1 == sgLastValvesInfo.b1LeanBackWardStat))
		{
			if(0 == sgLastValvesInfo.b1LiftUpStat)	/*对油泵电机添加速度,前倾和后倾的速度*/
			{
				if (1 == sgValvesInfo.b1LeanBackWardStat)/*arrcoding to Parameter*/
				{
					u8PumpOrPropValue = sgUserInfo.u8LeanBackWardValue;
				}
				else if (1 == sgValvesInfo.b1LeanForWardStat)/*arrcoding to Parameter*/
				{
					u8PumpOrPropValue =  sgUserInfo.u8LeanForWardValue;
				}
			}
			
			if (SendData->u8PumpTarget > u8PumpOrPropValue) 
			{
				fTmp = sgUserInfo.fLiftSpdPer5msDecStep * (0.2 + 0.8 * SendData->u8PumpTarget / PUMP_RANGE);
				if (((SendData->u8PumpTarget - u8PumpOrPropValue)> fTmp) && (0 != sgUserInfo.fLiftSpdPer5msDecStep))
				{
					SendData->u8PumpTarget -= fTmp;
				}
				else
				{
					SendData->u8PumpTarget = u8PumpOrPropValue; 
				}
//				if (((SendData->u8PumpTarget - u8PumpOrPropValue) > sgUserInfo.fLiftSpdPer5msDecStep) && (0 != sgUserInfo.fLiftSpdPer5msDecStep))
//				{
//					fPumpMototSpd -= sgUserInfo.fLiftSpdPer5msDecStep;
//					SendData->u8PumpTarget = fPumpMototSpd;
//				}
//				else
//				{
//					SendData->u8PumpTarget = u8PumpOrPropValue;
//					fPumpMototSpd = u8PumpOrPropValue;
//				}
			}
			else if (SendData->u8PumpTarget < u8PumpOrPropValue)
			{
				fTmp = sgUserInfo.fLiftSpdPer5msAccStep * (0.2 + 0.8 * SendData->u8PumpTarget / PUMP_RANGE);
				if (((u8PumpOrPropValue - SendData->u8PumpTarget)> fTmp) && (0 != sgUserInfo.fLiftSpdPer5msAccStep))
				{
					SendData->u8PumpTarget += fTmp;
				}
				else
				{
					SendData->u8PumpTarget = u8PumpOrPropValue;
				}
//				if (((u8PumpOrPropValue - SendData->u8PumpTarget) > sgUserInfo.fLiftSpdPer5msAccStep) && (0 != sgUserInfo.fLiftSpdPer5msAccStep))
//				{
//					fPumpMototSpd += sgUserInfo.fLiftSpdPer5msAccStep;
//					SendData->u8PumpTarget = fPumpMototSpd;
//				}
//				else
//				{
//					SendData->u8PumpTarget = u8PumpOrPropValue;
//					fPumpMototSpd = u8PumpOrPropValue;
//				}
			}
			else
			{
				SendData->u8PumpTarget = u8PumpOrPropValue;
//				fPumpMototSpd = u8PumpOrPropValue;
			}
		}
		else
		{
			SendData->u8PumpTarget = 0;
//			fPumpMototSpd = 0.0;
		}
	}

	if (0 != SendData->u8PumpTarget)
	{
		SendData->b1LiftReq = 1;
		u8Flag = 1<< 1;
	}

//	if (0 != u8Flag)
//	{
//		SendData->b1ServoOn = 1;
//	}
	/*lilu 20230809 add lock*/
	//if (0 == sgSwiInput.b1Lock)
	if (1 == sgValvesInfo.b1NoActFlag)
	{
		SendData->b1ServoOn = 0;				/*紧急制动， 断开使能*/
		SendData->u8PumpTarget = 0;
		SendData->b1LiftReq = 0;
		SendData->u8TargetHigh = 0;
		SendData->u8TargetLow = 0;
		SendData->b1ForwardReq = 0;
		SendData->b1BackwardReq = 0;
		vPropSetTarget(LIFTDOWN_VALVE, 0);
	}
	else
	{
		if (1 == sgSwiInput.b1Lock)
		{
			SendData->b1ServoOn = 1;				/*紧急制动， 断开使能*/
		}
	}
	
	{
		static uint32_t u32Cnt = 0;
		u32Cnt++;
		if (0 == (u32Cnt % 2))
//		//i32LogWrite(INFO, "*************NoAct = 0x%x**********\r\n", sgValvesInfo.u8NoAct);
		i32LogWrite(INFO, LOG_USER, "SendStat = 0x%x, Prop = %d, Angle = %d ,MotorSpd = %d, u16MotorVal = %d, Valve0 = 0x%x, Valve1 = 0x%x, Cnt = %d\r\n",  \
			SendData->buf[2], u8PumpOrPropValue, gCanRevPdoInfo.CanRevInfo2.i16SteerAngle, i16Spd, u16MotorVal, sgValvesInfo.u8Data[0], sgValvesInfo.u8Data[1], u32Cnt);
	}
}

static void vPropErrCallBack(uint8_t u8Channel)
{
	switch(u8Channel)
	{
		case LIFTDOWN_VALVE:
			i32ErrCodeSet(LIFTDOWN_VALVE_ERR);
			/*add errcode*/
			break;
		default:
			break;
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
//			i32ErrCodeSet(AI_B_AI2_ERR);
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
	static xCanRev1ACInfo CanRev1ACInfoLast;
	
	if(0 != memcmp((char*)CanRev1ACInfoLast.u8Data, (char*)gCanRevPdoInfo.u8RevData1, sizeof(CanRev1ACInfoLast)))
	{
		memcpy((char*)CanRev1ACInfoLast.u8Data, (char*)gCanRevPdoInfo.u8RevData1, sizeof(CanRev1ACInfoLast));
		/*添加相关的操作*/
		
//		gCanSendPdoInfo.CanSend23CInfo.u8Soc = CanRev1ACInfoLast.u8Soc;
		
//		if (CanRev1ACInfoLast.u8Soc <= BMS_LOW_BATTERY_VAL)
//		{
//			sgValvesInfo.b4Gear4Spd |= 1 << Gear4_Spd_Con1;
//		}
//		else
//		{
//			sgValvesInfo.b4Gear4Spd &= ~(1 << Gear4_Spd_Con1);
//		}
		
		if (1 == CanRev1ACInfoLast.b1LimitSpd2)
		{
			sgValvesInfo.b4Gear4Spd |= 1 << Gear4_Spd_Con2;
		}
		else
		{
			sgValvesInfo.b4Gear4Spd &= ~(1 << Gear4_Spd_Con2);
		}
		
		if (1 == CanRev1ACInfoLast.b1LimitSpd1)
		{
			sgValvesInfo.b4Gear3Spd |= 1 << Gear3_Spd_Con1;
		}
		else
		{
			sgValvesInfo.b4Gear3Spd &= ~(1 << Gear3_Spd_Con1);
		}
		
		if (1 == CanRev1ACInfoLast.b1MainDriverOpen)
		{
			
		}
		else
		{
			
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
	int32_t i32AdcValue = 0;
	
	i32AdcValue = i32LocalAiGetValue(MOVE_THROTTLE);
	
	if (i32AdcValue > sgUserInfo.u16ThrottleMax)	/*DeadZone Max*/
	{
		u16MotorVal = MOTOR_MAX_SPEED_VALUE;
	}
	else if (i32AdcValue >= (sgUserInfo.u16ThrottleMidValue))		/*超过一半*/
	{
		u16MotorVal = sgUserInfo.u16ThrottleMid + (i32AdcValue - sgUserInfo.u16ThrottleMidValue) * (MOTOR_SPEED_RANGE - sgUserInfo.u16ThrottleMid) / sgUserInfo.u16ThrottleRange;
	}
	else if (i32AdcValue >= sgUserInfo.u16ThrottleMin)	/*DeadZone Min*/ 
	{
//		u16MotorVal = i32AdcValue * MOTOR_MAX_SPEED_VALUE / MOVE_RANGE_VALUE;
//		u16MotorVal = (i32AdcValue - sgUserInfo.u16ThrottleMin)  * MOTOR_SPEED_RANGE / sgUserInfo.u16ThrottleRange;
		u16MotorVal = (i32AdcValue - sgUserInfo.u16ThrottleMin) * sgUserInfo.u16ThrottleMid / sgUserInfo.u16ThrottleRange;
	}
	else
	{
		u16MotorVal = 0;
	}
	
	if (0 != sgValvesInfo.b4Gear1Spd)
	{
		sgValvesInfo.b1Gear1SpdFlag = 1;
	}
	else
	{
		sgValvesInfo.b1Gear1SpdFlag = 0;
	}
	
	if (0 != sgValvesInfo.b4Gear2Spd)
	{
		sgValvesInfo.b1Gear2SpdFlag = 1;
	}
	else
	{
		sgValvesInfo.b1Gear2SpdFlag = 0;
	}
	
	if (0 != sgValvesInfo.b4Gear3Spd)
	{
		sgValvesInfo.b1Gear3SpdFlag = 1;
	}
	else
	{
		sgValvesInfo.b1Gear3SpdFlag = 0;
	}
	
	if (0 != sgValvesInfo.b4Gear4Spd)
	{
		sgValvesInfo.b1Gear4SpdFlag = 1;
	}
	else
	{
		sgValvesInfo.b1Gear4SpdFlag = 0;
	}
	
	/*add Turn Dec Spd*/
	u16MotorVal = u16SteerAngleDecSpd(u16MotorVal, abs(gCanRevPdoInfo.CanRevInfo2.i16SteerAngle), &sgUserInfo.SteerAngleDecSpd);
	
	if (0 != gCanRevPdoInfo.CanRevInfo2.u8ErrSteer)
	{
		
	}
	/*add spd limit*/
	if (1 == sgValvesInfo.b1Gear1SpdFlag)
	{
		if (u16MotorVal >= sgUserInfo.u16Gear1Spd)
		{
			u16MotorVal = sgUserInfo.u16Gear1Spd;
		}
	}
	
	if (1 == sgValvesInfo.b1Gear2SpdFlag)
	{
		if (u16MotorVal >= sgUserInfo.u16Gear2Spd)
		{
			u16MotorVal = sgUserInfo.u16Gear2Spd;
		}
	}
	
	if (1 == sgValvesInfo.b1Gear3SpdFlag)
	{
		if (u16MotorVal >= sgUserInfo.u16Gear3Spd)
		{
			u16MotorVal = sgUserInfo.u16Gear3Spd;
		}
	}
	
	if (1 == sgValvesInfo.b1Gear4SpdFlag)
	{
		if (u16MotorVal >= sgUserInfo.u16Gear4Spd)
		{
			u16MotorVal = sgUserInfo.u16Gear4Spd;
		}
	}
		
	i32AdcValue = i32LocalAiGetValue(LIFT_THROTTLE);
	if (LIFT_MODE_THROTTLE == sgUserInfo.b1LiftMode)
	{
		if (1 == sgSwiInput.b1LiftUp)
		{
			if (i32AdcValue < sgUserInfo.u16LiftDownMax)	/*小于下降最小*/
			{
				u8PumpOrPropValue = PUMP_MAX_VALUE;
			}
			else if (i32AdcValue <= sgUserInfo.u16LiftDownMidValue)	/*小于下降一半*/
			{
				u8PumpOrPropValue = sgUserInfo.u16LiftDownMid + (sgUserInfo.u16LiftDownMidValue - i32AdcValue) * (PUMP_RANGE - sgUserInfo.u16LiftDownMid) / sgUserInfo.u16LiftDownRange;
			}
			else if (i32AdcValue <= sgUserInfo.u16LiftDownMin)	/*小于下降最小*/
			{
				u8PumpOrPropValue = (sgUserInfo.u16LiftDownMin - i32AdcValue) * sgUserInfo.u16LiftDownMid / sgUserInfo.u16LiftDownRange;
			}
			else if (i32AdcValue < sgUserInfo.u16LiftUpMin)			/*大于下降最小以及小于起升最小*/
			{
				u8PumpOrPropValue = 0;
			}
			else if (i32AdcValue <= sgUserInfo.u16LiftUpMidValue)	/*小于起升一半*/
			{
				u8PumpOrPropValue = (i32AdcValue - sgUserInfo.u16LiftUpMin) * sgUserInfo.u16LiftUpMid / sgUserInfo.u16LiftUpRange;
			}			
			else if (i32AdcValue <= sgUserInfo.u16LiftUpMax)		/*小于起升最大*/
			{
				u8PumpOrPropValue = sgUserInfo.u16LiftUpMid + (i32AdcValue - sgUserInfo.u16LiftUpMidValue) * (PUMP_RANGE - sgUserInfo.u16LiftUpMid) / sgUserInfo.u16LiftUpRange;
			}
			else													/*大于起升最大*/
			{
				u8PumpOrPropValue = PUMP_MAX_VALUE;
			}
				
			if (0 != u8PumpOrPropValue)
			{
				if (i32AdcValue <= sgUserInfo.u16LiftDownMin)
				{
					sgValvesInfo.b1LiftDownStat = 1;
					sgValvesInfo.b1LiftUpStat = 0;
				}
				else
				{
					sgValvesInfo.b1LiftDownStat = 0;
					if (0 == sgValvesInfo.b1LiftUpFlag)		/*lilu 20230811 add NoLiftFlag*/
					{
						sgValvesInfo.b1LiftUpStat = 1;
					}
					else
					{
						sgValvesInfo.b1LiftUpStat = 0;
					}
				}
			}
			else
			{
				sgValvesInfo.b1LiftDownStat = 0;
				sgValvesInfo.b1LiftUpStat = 0;
			}
			
		}
		else
		{
			sgValvesInfo.b1LiftUpStat = 0;
			sgValvesInfo.b1LiftDownStat = 0;
		}
	}
	
	i32AdcValue = i32LocalAiGetValue(CHARGE_SWI);
	if (i32AdcValue <= CHARGE_SWI_AD)
	{
		sgValvesInfo.u8NoAct |= 1 << NoAct_ChargeSwi;
	}
	else
	{
		sgValvesInfo.u8NoAct &= ~(1 << NoAct_ChargeSwi);
	}
	
	if (0 != sgValvesInfo.u8NoAct)
	{
		sgValvesInfo.b1NoActFlag = 1;
	}
	else
	{
		sgValvesInfo.b1NoActFlag = 0;
	}
}


/*开关量监控*/
static void vSwiMonitor(void)
{
	xSwiInput SwiInput = {0};
	
	if(1 == i32LocalDiGet(GUARD_RAIL_SWI))
	{
		SwiInput.b1GuardRaild = 1;
	}

	if(1 == i32LocalDiGet(EMS_SWI))
	{
		SwiInput.b1Ems = 1;
	}
	
	if(1 == i32LocalDiGet(LOCK_SWI))
	{
		SwiInput.b1Lock = 1;
	}
	
	if(1 == i32LocalDiGet(FORWARD_SWI))
	{
		SwiInput.b1Forward = 1;
	}
	
	if(1 == i32LocalDiGet(BACKWARD_SWI))
	{
		SwiInput.b1Backward = 1;
	}
	
	if(1 == i32LocalDiGet(LIFT_DOWN_SWI))
	{
		SwiInput.b1LiftDown = 1;
	}
	
	if(1 == i32LocalDiGet(LIFT_UP_OE_ENABLE_SWI))
	{
		SwiInput.b1LiftUp = 1;
	}
	
	if(1 == i32LocalDiGet(PEDAL_OPEN_SWI))
	{
		SwiInput.b1PedalOpen = 1;
	}
	
	if(1 == i32LocalDiGet(PEDAL_CLOSE_SWI))
	{
		SwiInput.b1PedalClose = 1;
	}
	
	if(1 == i32LocalDiGet(FAULT_LOCKOUT_SWI))
	{
		SwiInput.b1FaultLockOut = 1;
	}
	
	if(1 == i32LocalDiGet(METER_1M8_LIMIT_SWI))
	{
		SwiInput.b1Meter1m8 = 1;
	}
	
	if(1 == i32LocalDiGet(LEAN_FORWARD_SWI))
	{
		SwiInput.b1LeanForward = 1;
	}
	
	if(1 == i32LocalDiGet(LEAN_BACKWARD_SWI))
	{
		SwiInput.b1LeanBackward = 1;
	}
	
	if(1 == i32LocalDiGet(LIFT_UP_LIMIT_SWI))
	{
		SwiInput.b1UpLimit= 1;
	}
	/*add lock */
	if ((1 == SwiInput.b1Forward) || (1 == SwiInput.b1Backward) ||
	   (1 == SwiInput.b1LeanForward) || (1 == SwiInput.b1LeanBackward) ||
	   (1 == SwiInput.b1LiftUp) || (1 == SwiInput.b1LiftDown))
	{
		/*禁止起升和前进后退*/
//		if (0 == SwiInput.b1FaultLockOut) 
		if ((0 != gCanRevPdoInfo.CanRevInfo2.u8ErrSteer) || (0 == SwiInput.b1FaultLockOut))
		{
//			sgValvesInfo.b4NoAct |= 1 << NoAct_FaultLock;
			sgValvesInfo.b4NoLiftUp |= 1 << NoLiftUp_FaultLock;
			sgValvesInfo.b4NoMove |= 1 << NoMove_FaultLock;
			i32ErrCodeSet(ACT_FAULT_LOCK_ERR);
		}
		
		if (0 == SwiInput.b1Lock)
		{
			sgValvesInfo.u8NoAct |= 1 << NoAct_Lock;
			i32ErrCodeSet(ACT_LOCK_ERR);
		}
	}
	
	if ((0 == SwiInput.b1Forward) && (0 == SwiInput.b1Backward) && (1 == SwiInput.b1Ems))
	{
		sgValvesInfo.b4NoMove |= 1 << NoMove_Ems;
		i32ErrCodeSet(MOVE_EMS_ERR);
	}
		
	if ((0 == SwiInput.b1Forward) && (0 == SwiInput.b1Backward) && (0 == SwiInput.b1Ems))
	{
		sgValvesInfo.b4NoMove &= ~(1 << NoMove_Ems);
		i32ErrCodeClr(MOVE_EMS_ERR);
	}
	
	if ((0 == SwiInput.b1Forward) && (0 == SwiInput.b1Backward) &&
	    (0 == SwiInput.b1LeanForward) && (0 == SwiInput.b1LeanBackward) &&
	    (0 == SwiInput.b1LiftUp) && (0 == SwiInput.b1LiftDown) && 
	    (0 == SwiInput.b1Ems) && (0 == SwiInput.b1Lock) && (1 == SwiInput.b1FaultLockOut))
	{
		sgValvesInfo.u8NoAct &= ~(1 << NoAct_Lock);
		sgValvesInfo.u8NoAct &= ~(1 << NoAct_Init);
		
		sgValvesInfo.b4NoLiftUp &= ~(1 << NoLiftUp_FaultLock);
		sgValvesInfo.b4NoMove &= ~(1 << NoMove_FaultLock);
		
		i32ErrCodeClr(ACT_INIT_ERR);
		i32ErrCodeClr(ACT_FAULT_LOCK_ERR);
		i32ErrCodeClr(ACT_LOCK_ERR);
	}
	
	if (0 != sgValvesInfo.b4NoMove)
	{
		sgValvesInfo.b1MoveFlag = 1;
	}
	else
	{
		sgValvesInfo.b1MoveFlag = 0;
	}
	
	if (0 != sgValvesInfo.b4NoLiftUp)
	{
		sgValvesInfo.b1LiftUpFlag = 1;
	}
	else
	{
		sgValvesInfo.b1LiftUpFlag = 0;
	}
	
	/*添加逻辑*/
	if (0 == sgValvesInfo.b1NoActFlag)
	{
//		if (sgSwiInput.u16data != SwiInput.u16data)
		{
			if ((0 == sgSwiInput.b1UpLimit) && (1 == SwiInput.b1UpLimit) && (1 == sgValvesInfo.b1LiftUpStat))
			{
				sgValvesInfo.b4Gear2Spd = 1 << Gear2_Spd_Con1;
			}
			else if ((0 == sgSwiInput.b1UpLimit) && (1 == SwiInput.b1UpLimit) && (1 == sgValvesInfo.b1LiftDownStat))
			{
				sgValvesInfo.b4Gear2Spd &=  ~(1 << Gear2_Spd_Con1);
			}
			//if (1 == SwiInput.b1Meter1m8)
			if ((0 == sgSwiInput.b1Meter1m8) && (1 == SwiInput.b1Meter1m8) && (1 == sgValvesInfo.b1LiftUpStat))
			{
//				sgValvesInfo.b4Gear2Spd = 1 << Gear2_Spd_Con2;
				sgSwiInput.b1Above1M8 = 1;
			}
			//else
			else if ((1 == sgSwiInput.b1Meter1m8) && (0 == SwiInput.b1Meter1m8) && (1 == sgValvesInfo.b1LiftDownStat))
			{
//				sgValvesInfo.b4Gear2Spd &=  ~(1 << Gear2_Spd_Con2);
				sgSwiInput.b1Above1M8 = 0;
			}
			
			SwiInput.b1Above1M8 = sgSwiInput.b1Above1M8;			/*lilu 20230801 Above1M8 retain*/
			sgSwiInput.u16data = SwiInput.u16data;   /**/
			
//			gCanSendPdoInfo.CanSend23CInfo.b1ForWardSwi = sgSwiInput.b1Forward;
//			gCanSendPdoInfo.CanSend23CInfo.b1BackWardSwi = sgSwiInput.b1Backward;
//			gCanSendPdoInfo.CanSend23CInfo.b1LiftUpSwi = sgSwiInput.b1LiftUp;
//			gCanSendPdoInfo.CanSend23CInfo.b1LiftDownSwi = sgSwiInput.b1LiftDown;
//			gCanSendPdoInfo.CanSend23CInfo.b1EmsSwi = sgSwiInput.b1Ems;
				
			if ((1 == sgSwiInput.b1GuardRaild) && (0 == sgSwiInput.b1PedalClose) && (1 == sgSwiInput.b1PedalOpen))
			{
				//sgValvesInfo.b1MoveFlag = 0;
				sgValvesInfo.b4NoMove &= ~(1 << NoMove_Pedal);
				sgValvesInfo.b4Gear1Spd &= ~(1 << Gear1_Spd_Con1);
			}
			/*速度变为1档*/
			else if (((0 == sgSwiInput.b1GuardRaild) && (0 == sgSwiInput.b1PedalClose) && (1 == sgSwiInput.b1PedalOpen)) ||  /*SW1=0,DRIVER6-R=0,SW8=0*/
					  ((0 == sgSwiInput.b1GuardRaild) && (1 == sgSwiInput.b1PedalClose) && (0 == sgSwiInput.b1PedalOpen)))	/*SW1=0,DRIVER6-R=1,SW8=0*/
			{
				sgValvesInfo.b4Gear1Spd = 1 << Gear1_Spd_Con1;
				sgValvesInfo.b4NoMove &= ~(1 << NoMove_Pedal);
				//sgValvesInfo.b1MoveFlag = 0;
			}
			/*不能行驶*/
			else
			{
				//sgValvesInfo.b1MoveFlag = 1;
				sgValvesInfo.b4NoMove |= 1 << NoMove_Pedal;
				sgValvesInfo.b4Gear1Spd &= ~(1 << Gear1_Spd_Con1);
			}
				
			/*护栏放下， 速度变为1档*/
			if (1 == sgSwiInput.b1GuardRaild)
			{
				sgValvesInfo.b4Gear1Spd = 1 << Gear1_Spd_Con2;
			}
			else 
			{
				sgValvesInfo.b4Gear1Spd &= ~(1 << Gear1_Spd_Con2);
			}
				
			/*Limit Up*/
			if (1 == sgSwiInput.b1Above1M8) 
			{
				sgValvesInfo.b4Gear2Spd = 1 << Gear2_Spd_Con2;
//				if (0 == sgSwiInput.b1GuardRaild)
//				{
//					sgValvesInfo.b4NoLiftUp |= 1 << NoLiftUp_HeightLimit;
//				}
//				else
//				{
//					sgValvesInfo.b4NoLiftUp &= ~(1 << NoLiftUp_HeightLimit);
//				}
			}
			else
			{
				sgValvesInfo.b4Gear2Spd &=  ~(1 << Gear2_Spd_Con2);
				sgValvesInfo.b4NoLiftUp &= ~(1 << NoLiftUp_HeightLimit);
			}
							
			if (0 == sgValvesInfo.b1MoveFlag)
			{
				/*Move Mode*/
				if ((1 == sgSwiInput.b1Forward) && (0 == sgSwiInput.b1Backward))
				{
					sgValvesInfo.b1ForWardStat = 1;
					sgValvesInfo.b1BackWardStat = 0;
				}
				else if ((0 == sgSwiInput.b1Forward) && (1 == sgSwiInput.b1Backward))
				{
					sgValvesInfo.b1ForWardStat = 0;
					sgValvesInfo.b1BackWardStat = 1;
				}
				else
				{
					sgValvesInfo.b1ForWardStat = 0;
					sgValvesInfo.b1BackWardStat = 0;
				}
			}
			else
			{
				sgValvesInfo.b1ForWardStat = 0;
				sgValvesInfo.b1BackWardStat = 0;
			}
				
			/*Lift Mode*/
			if (LIFT_MODE_SWI == sgUserInfo.b1LiftMode)
			{
				if ((1 == sgSwiInput.b1LiftUp) && (0 == sgSwiInput.b1LiftDown))
				{
					if (0 == sgValvesInfo.b1LiftUpFlag)
					{
						sgValvesInfo.b1LiftUpStat = 1;
					}
					else
					{
						sgValvesInfo.b1LiftUpStat = 0;
					}
					sgValvesInfo.b1LiftDownStat = 0;
					/*add fix value, max value*/
					u8PumpOrPropValue = PUMP_MAX_VALUE;
				}
				else if ((0 == sgSwiInput.b1LiftUp) && (1 == sgSwiInput.b1LiftDown))
				{
					sgValvesInfo.b1LiftUpStat = 0;
					sgValvesInfo.b1LiftDownStat = 1;
					/*add fix value, max value*/
					u8PumpOrPropValue = PUMP_MAX_VALUE;
				}
				else
				{
					sgValvesInfo.b1LiftUpStat = 0;
					sgValvesInfo.b1LiftDownStat = 0;
					/*add fix value*/
					u8PumpOrPropValue = 0;
				}
			}
		}
	}
	else
	{
		/*ForWard BackWard LiftUp LiftDown LeanForWard LeanBackWard all disable*/
		sgValvesInfo.u8Data[1] &= 0xC0;		/*Hight 2 Bit Reserve*/	
	}
}

static uint8_t u8SwiInitChcek(void)
{
	uint8_t u8Res = 0;
	
	if ((1 == i32LocalDiGet(FORWARD_SWI)) || (1 == i32LocalDiGet(BACKWARD_SWI)) ||					/*前进后退*/
//		(1 == i32LocalDiGet(LIFT_UP_OE_ENABLE_SWI)) || (1 == i32LocalDiGet(LIFT_DOWN_SWI)) ||		/*起升下降*/
		(1 == i32LocalDiGet(LEAN_FORWARD_SWI)) || (1 == i32LocalDiGet(LEAN_BACKWARD_SWI)) ||		/*前倾后倾*/
		(1 == i32LocalDiGet(EMS_SWI)) || (1 == i32LocalDiGet(LOCK_SWI)))							/*急反互锁*/
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
	
	vMstRevRegister(vMstRevProc);
	vMstSendRegister(vMstSendProc);
	
//	vDoPwmErrReg(vDoPwmErrCallBack);
	vPropErrReg(vPropErrCallBack);
	vAiErrReg(vAiErrCallBack);
	
	memset(&sgValvesInfo, 0, sizeof(sgValvesInfo));
	
	sgUserInfo.fPropMinCurrent = i32GetPara(PROP_MIN_CURRENT) / 1000.0;		/*比例阀最小电流*/
	sgUserInfo.fPropMaxCurrent = i32GetPara(PROP_MAX_CURRENT) / 1000.0;		/*比例阀最大电流*/
	
	sgUserInfo.u8LeanForWardValue = i32GetPara(LEAN_FORWARD_PARA) * PUMP_RANGE / 100;		/*前倾参数*/
	sgUserInfo.u8LeanBackWardValue = i32GetPara(LEAN_BACKWARD_PARA) * PUMP_RANGE / 100;		/*后倾参数*/
	
	sgUserInfo.b1LiftMode = i32GetPara(LIFT_MODE_PARA);						/*起升模式*/
	
	sgUserInfo.u16Gear1Spd = i32GetPara(PARA_Gear1Spd) * MOTOR_SPEED_RANGE / 100;	/*1档速度*/
	sgUserInfo.u16Gear2Spd = i32GetPara(PARA_Gear2Spd) * MOTOR_SPEED_RANGE / 100;	/*2档速度*/
	sgUserInfo.u16Gear3Spd = i32GetPara(PARA_Gear3Spd) * MOTOR_SPEED_RANGE / 100;	/*3档速度*/
	sgUserInfo.u16Gear4Spd = i32GetPara(PARA_Gear4Spd) * MOTOR_SPEED_RANGE / 100;	/*4档速度*/
	
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
	
	u32HourCount = u32HourCountRead();
	
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
		sgSwiInput.b1Above1M8 = sgSaveState.b1Above1M8;
		u16EepromRead(PARA_HourSetTime, &u16Tmp, 1);	
	}
//	//i32LogWrite(INFO, "******SaveState = 0x%x\r\n*********", sgSaveState.u16Data);
	
//	__disable_irq();
//	gCanSendPdoInfo.CanSend23CInfo.u8WorkTimeLL = u32HourCount & 0xFF;
//	gCanSendPdoInfo.CanSend23CInfo.u8WorkTimeLH = (u32HourCount >> 8) & 0xFF;
//	gCanSendPdoInfo.CanSend23CInfo.u8WorkTimeHL = (u32HourCount >> 16) & 0xFF;
//	gCanSendPdoInfo.CanSend23CInfo.u8WorkTimeHH = (u32HourCount >> 24) & 0xFF;
//	__enable_irq();		
	/*Para Initial*/
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

	vSetPdoPara(PdoPara);
	
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
	static uint8_t u8OneMinute = 0;
	
//	static uint8_t u8ActCnt = 0;

	if(true == u8GetNetTimerOverFlag(TIMER_EcuPowerOn))
	{
		u8EcuProcFlag = 1;
		vKillNetTimer(TIMER_EcuPowerOn);
	}
	
	if ((false == u8GetNetTimerOverFlag(TIMER_EcuPowerOn)) && (true == u8GetNetTimerStartFlag(TIMER_EcuPowerOn)))
	{
		if (1 == u8SwiInitChcek())
		{
			sgValvesInfo.u8NoAct |= 1 << NoAct_Init;
			i32ErrCodeSet(ACT_INIT_ERR);
//			//i32LogWrite(INFO, "Act Init Err!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
		}
	}

	if(1 == u8EcuProcFlag)
	{
//		if (0 == (u16CanId5ACPeriod++ % CANID_5AC_SEND_PERIOD))	/*Send 5 times 5AC*/
//		{
//			vCanId5ACSend();
//		}
		if (LiBattery != sgUserInfo.u8BatteryType)
		{
			if (u8GetBatterySoc() <= BAT_LOW_WARING_VAL)	
			{
				i32ErrCodeSet(BAT_LOW_1_ERR);
			}
			
			if (u8GetBatterySoc() <= BAT_LOW_ERR_VAL)		/*降低速度*//*禁止起升*/
			{
				sgValvesInfo.b4Gear4Spd |= 1 << Gear4_Spd_Con1;
				sgValvesInfo.b4NoLiftUp |= 1 << NoLiftUp_LowBat;
				i32ErrCodeSet(BAT_LOW_2_ERR);
			}
		}
		
		if (sgSwiInput.b1Above1M8 != sgSaveState.b1Above1M8)
		{
			sgSaveState.b1Above1M8 = sgSwiInput.b1Above1M8;
			u16EepromWrite(PARA_SaveState, sgSaveState.u16Data, 1);	
			//i32LogWrite(INFO, "$$$$$$$$$$$Save Default Flag = 0x%x$$$$$$$$$$$$$\r\n", sgSaveState.u16Data);
		}
		
		vAiMonitor();
		vSwiMonitor();		
		vCanRevPdoProc();
		u8ErrCode = u8ErrCodeGet();
		if (0 != u8ErrCode)
		{
//			gCanSendPdoInfo.CanSend23CInfo.u8ErrCode = u8ErrCode;
			vLedSendAlmCode(u8ErrCode);
			static uint8_t u8ErrCodeCnt = 0;
			u8ErrCodeCnt++;
			if (u8ErrCodeCnt >= 200)	
			{	
				u8ErrCodeCnt = 0;				
				i32LogWrite(DEBUG, LOG_USER, "ErrCode is %d!!!\r\n", u8ErrCode);
			}	
		}
	}
	
	if (true == u8GetNetTimerOverFlag(TIMER_HourCount))
	{
		vResetNetTimer(TIMER_HourCount);
		u8OneMinute++;
		if (u8OneMinute >= 6)
		{
			u8OneMinute = 0;
			u32HourCount++;
			//i32LogWrite(INFO, "************HourCount is %d**************\r\n", u32HourCount);
			vHourCountWrite(u32HourCount);
//			__disable_irq();
//			gCanSendPdoInfo.CanSend23CInfo.u8WorkTimeLL = u32HourCount & 0xFF;
//			gCanSendPdoInfo.CanSend23CInfo.u8WorkTimeLH = (u32HourCount >> 8) & 0xFF;
//			gCanSendPdoInfo.CanSend23CInfo.u8WorkTimeHL = (u32HourCount >> 16) & 0xFF;
//			gCanSendPdoInfo.CanSend23CInfo.u8WorkTimeHH = (u32HourCount >> 24) & 0xFF;
//			__enable_irq();
		}
	}
	
	vWdgSetFun(WDG_USER_BIT);
}
#endif
