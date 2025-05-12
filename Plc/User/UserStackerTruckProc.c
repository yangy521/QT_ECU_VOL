/*******************************************************************************
* Filename: UserStackerTruckProc.c	                                           *
* Description: 逻辑层的源文件，包含对应的回调函数	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/05/30    														   *
* Revision:	 														 		   *
*******************************************************************************/
#include "UserCanComm.h"
#include "UserStackerTruckProc.h"
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

#if (USER_TYPE == USER_ZHONGLI_DGC)

const static xPdoParameter  sgPdoPara = 
{
	.TpdoPara = {
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 200},
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 200},
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 200},
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 200},
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
#define	NoAct_Security		3	

#define NoLiftUp_HeightLimit	0		
#define NoLiftUp_FaultLock		1
#define	NoLiftUp_LowBat			2
#define NoLiftUp_RentalTimeOut	3

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
		uint16_t b1HeightSpdLimit: 1;
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
		uint8_t b1PumpMotorNoAct: 1;
		
		uint8_t b1ForWardStat: 1;
		uint8_t b1BackWardStat: 1;
		uint8_t b1LiftUpStat: 1;
		uint8_t b1LeanForWardStat: 1;
		uint8_t b1LeanBackWardStat: 1;
		uint8_t b1LiftDownStat: 1;
//		uint8_t	b1PumpMotorNoAct: 1;
		uint8_t b2Reserve2: 2;
		
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
	uint8_t		b1HourCntClr: 1;
	uint8_t		b2Reserve: 2;	
	
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

static xSaveStateInfo sgSaveState;
const static uint8_t u8SeedArray[16]= "V!BWT%EM6dJ8<nPs";

static int16_t	i16MotorSpd = 0;
static uint16_t u16MotorVal = 0;
static uint8_t	u8PumpOrPropValue = 0;
static uint16_t u16CanId5ACPeriod = 0;
static uint16_t u16CanRev62CCnt = 0;
static xValvesInfo sgValvesInfo;
static xUserInfo sgUserInfo;
static xSwiInput sgSwiInput;
static uint32_t u32HourCount = 0;
volatile static uint32_t MstPARADate[256];
volatile static uint16_t ParaCount = 1;
volatile static uint16_t SendCount = 0;
volatile static uint8_t MstParaIsOk = 1;
volatile static uint16_t RevMstDate = 1;
volatile static uint16_t ReceiveCount = 0;
static uint8_t PowerOnGetPara = true;
volatile static uint8_t SendOk = 0;

xPARA62CInfo CanRev62CInfoLast;
/*PDO接收待完善*/
xCanRevPdoInfo gCanRevPdoInfo; 
//{
//	.CanRevInfo1.u8Soc = 100,					/*BmsSoc Default: 100*/
//};				
/*PDO发送待完善*/
xCanSendPdoInfo gCanSendPdoInfo = 
{
	.CanSend33CInfo.u8ErrCodeList = 0x03,
	.CanSend33CInfo.u8SoftVersionLowByte = 0x00,
	.CanSend33CInfo.u8SoftVersionHighByte = 0x10,
};			

const static uint8_t u8ErrSwitchArray[ErrCodeMax] = 
{	
/*  0    1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18   19*/
	0,   82,  208, 180, 0,   227, 227, 227, 211, 211, 0,   180, 180, 60,  38,  216, 19,  212, 62,  65,
	78,  37,  53,  227, 75,  17,  248, 0,   0,   218, 0,   19,  62,  62,  65,  227, 242, 242, 210, 208,
	79,  66,  66,  0,   0,   0,   0,   0,   0,   0,   0,   242, 242, 242, 242, 242, 242, 242, 242, 242,
	242, 214, 182, 63,  64,  65, 	66, 67, 68,  69,  70,  78,  72,  188,  74,  75,  76,  77,  78,  79, 
	80,  81,  82,  83,  84,  85,  86,  87,  88,  89,  90,  91,  92,  93,  94,  95,  96,  97,  98,  99,
	100, 79, 	79,  205, 79,  105, 66,  66,  108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 
	120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 
	140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159,
	160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 
	180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 
	200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 
	220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 
	240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254,
};
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
	
	i16MotorSpd = (int16_t)(RevData->u8SpeedFdbHigh << 8) | RevData->u8SpeedFdbLow ;
//	i32LogWrite(DEBUG, LOG_USER, "Motor Rev Spd = %d\r\n", i16MotorSpd);
	__disable_irq();
	tmp = abs(i16MotorSpd / sgUserInfo.u16RatioOfTransmission);
	/*Motor Speed*/
	gCanSendPdoInfo.CanSend33CInfo.u8SpdHighByte = tmp >> 8;
	gCanSendPdoInfo.CanSend33CInfo.u8SpdLowByte = tmp;
	__enable_irq();
	
	tmp = RevData->u8MotorTmp;
	/*Motor Temp*/
	gCanSendPdoInfo.CanSend43CInfo.i16MotorTemp = RevData->u8MotorTmp;
	
	tmp = RevData->u8BoardTmp;
	/**/
	gCanSendPdoInfo.CanSend43CInfo.i16CtrlTemp = RevData->u8BoardTmp;
	
	__disable_irq();
	gCanSendPdoInfo.CanSend43CInfo.i16MotorCurrent = (RevData->u8CurrentHigh << 8) | RevData->u8CurrentLow;
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
	
	float fTmp = 0.0;
	xMstSendStat LastStatus;
	int16_t i16Spd = 0;
//	uint8_t u8Flag = 0;
		
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
				fTmp = sgUserInfo.fMoveSpdPer5msDecStep * (0.01 + 0.99 * i16Spd / MOTOR_SPEED_RANGE);
				if ((abs(i16MotorSpd) > MOTOR_MIN_SPEED) && (abs(i16Spd) > (uint16_t)(fTmp + 1)))
				{
					i16Spd -= fTmp;
				}
				else
				{
					i16Spd = 0;
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
//		u8Flag = 1 << 0;
		if (1 == sgLastValvesInfo.b1ForWardStat)
		{
			SendData->b1ForwardReq = 1;
		}
		else if (1 == sgLastValvesInfo.b1BackWardStat)
		{
			SendData->b1BackwardReq = 1;
		}
	}
	SendData->u8TargetHigh = i16Spd >> 8;
	SendData->u8TargetLow = i16Spd;
	
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
//		if ((inserted_data[0] * PROP_CURRENT_FACOTR) < sgUserInfo.fPropMinCurrent)
		if (((inserted_data[0] * PROP_CURRENT_FACOTR) < sgUserInfo.fPropMinCurrent) && (0 == SendData->u8PumpTarget))
		{
			sgLastValvesInfo.b1LiftUpStat = sgValvesInfo.b1LiftUpStat;
			sgLastValvesInfo.b1LiftDownStat = sgValvesInfo.b1LiftDownStat;
			sgLastValvesInfo.b1LeanForWardStat = sgValvesInfo.b1LeanForWardStat;
			sgLastValvesInfo.b1LeanBackWardStat = sgValvesInfo.b1LeanBackWardStat;
			if (1 == sgLastValvesInfo.b1LiftDownStat)
			{
				i32DoPwmSet(LIFTUP_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(LEAN_FORWARD_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(LEAN_BACKWARD_VALVE, DRIVER_CLOSE);
			}
			else if(1 == sgLastValvesInfo.b1LiftUpStat)
			{
				vPropSetTarget(LIFTDOWN_VALVE, 0);
				i32DoPwmSet(LIFTUP_VALVE, DRIVER_OPEN);
				i32DoPwmSet(LEAN_FORWARD_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(LEAN_BACKWARD_VALVE, DRIVER_CLOSE);
			}
			else if (1 == sgLastValvesInfo.b1LeanBackWardStat)
			{
				vPropSetTarget(LIFTDOWN_VALVE, 0);
				i32DoPwmSet(LIFTUP_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(LEAN_FORWARD_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(LEAN_BACKWARD_VALVE, DRIVER_OPEN);
			}
			else if (1 == sgLastValvesInfo.b1LeanForWardStat)
			{
				vPropSetTarget(LIFTDOWN_VALVE, 0);
				i32DoPwmSet(LIFTUP_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(LEAN_FORWARD_VALVE, DRIVER_OPEN);
				i32DoPwmSet(LEAN_BACKWARD_VALVE, DRIVER_CLOSE);
			}
			else 
			{
				vPropSetTarget(LIFTDOWN_VALVE, 0);
				i32DoPwmSet(LIFTUP_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(LEAN_FORWARD_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(LEAN_BACKWARD_VALVE, DRIVER_CLOSE);
			}
		}
		else
		{
			vPropSetTarget(LIFTDOWN_VALVE, 0);
			if (SendData->u8PumpTarget > 0) 
			{
				fTmp = sgUserInfo.fLiftSpdPer5msDecStep * (0.05 + 0.95 * SendData->u8PumpTarget / PUMP_RANGE);
				if ((SendData->u8PumpTarget > fTmp) && (0 != sgUserInfo.fLiftSpdPer5msDecStep))
				{
					SendData->u8PumpTarget -= fTmp;
				}
				else
				{
					SendData->u8PumpTarget = 0; 
				}
			}
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
//			if (SendData->u8PumpTarget > 0) 
//			{
//				fTmp = sgUserInfo.fLiftSpdPer5msDecStep * (0.05 + 0.95 * SendData->u8PumpTarget / PUMP_RANGE);
//				if ((SendData->u8PumpTarget > fTmp) && (0 != sgUserInfo.fLiftSpdPer5msDecStep))
//				{
//					SendData->u8PumpTarget -= fTmp;
//				}
//				else
//				{
//					SendData->u8PumpTarget = 0; 
//				}
//			}
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
					u8PumpOrPropValue = sgUserInfo.u8LeanForWardValue;
				}
				else
				{
					u8PumpOrPropValue = 0;
				}
			}
			
			if (SendData->u8PumpTarget > u8PumpOrPropValue) 
			{
				fTmp = sgUserInfo.fLiftSpdPer5msDecStep * (0.05 + 0.95 * SendData->u8PumpTarget / PUMP_RANGE);
				if (((SendData->u8PumpTarget - u8PumpOrPropValue)> fTmp) && (0 != sgUserInfo.fLiftSpdPer5msDecStep))
				{
					SendData->u8PumpTarget -= fTmp;
				}
				else
				{
					SendData->u8PumpTarget = u8PumpOrPropValue; 
				}
			}
			else if (SendData->u8PumpTarget < u8PumpOrPropValue)
			{
				fTmp = sgUserInfo.fLiftSpdPer5msAccStep * (0.05 + 0.95 * SendData->u8PumpTarget / PUMP_RANGE);
				if (((u8PumpOrPropValue - SendData->u8PumpTarget) > fTmp) && (0 != sgUserInfo.fLiftSpdPer5msAccStep))
				{
					SendData->u8PumpTarget += fTmp;
				}
				else
				{
					SendData->u8PumpTarget = u8PumpOrPropValue;
				}
			}
			else
			{
				SendData->u8PumpTarget = u8PumpOrPropValue;
			}
		}
		else
		{
			SendData->u8PumpTarget = 0;
		}
	}

	if (0 != SendData->u8PumpTarget)
	{
		SendData->b1LiftReq = 1;
	}
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
		//if (0 == (u32Cnt % 5))
		{
			i32LogWrite(INFO, LOG_USER, "SendStat = 0x%x, Pump = %d, Prop = %d, Angle = %d ,MotorSpd = %d, u16MotorVal = %d, Valve0 = 0x%x, Valve1 = 0x%x\r\n",  \
				SendData->buf[2], SendData->u8PumpTarget, u8PumpOrPropValue, gCanRevPdoInfo.CanRevInfo2.i16SteerAngle, i16Spd, u16MotorVal, sgValvesInfo.u8Data[0], sgValvesInfo.u8Data[1]);
//			if (0 != sgValvesInfo.u8Data[0])
//			{
//				i32LogWrite(INFO, LOG_USER, "NoAct = 0x%x, NoMove = 0x%x, NoLiftUp = 0x%x\r\n", sgValvesInfo.u8NoAct, sgValvesInfo.b4NoMove, sgValvesInfo.b4NoLiftUp);
//				i32LogWrite(INFO, LOG_USER, "Gear1 = 0x%x, Gear2 = 0x%x, Gear3 = 0x%x, Gear4 = 0x%x\r\n",   \
//					sgValvesInfo.b4Gear1Spd, sgValvesInfo.b4Gear2Spd, sgValvesInfo.b4Gear3Spd, sgValvesInfo.b4Gear4Spd);
//			}
			
			/*lilu 20230819 For Test*/
			{
				i32SetPara(PARA_ForwardValveCurrent, i16Spd);		/*Send Motor Value*/
				i32SetPara(PARA_BackValveCurrent, i16MotorSpd);		/*Rev Motor Value*/
				i32SetPara(PARA_PropValveCurrent, inserted_data[0] * PROP_CURRENT_FACOTR * 1000);	/*Prop Current*/
				i32SetPara(PARA_LiftValveCurrent, SendData->u8PumpTarget);		/*Send Pump Value*/
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
				i32SetPara(PARA_PcuKeyInfo, u8ErrCodeGet());						/*ErrCode*/
				i32SetPara(PARA_BmsSoc, gCanSendPdoInfo.CanSend23CInfo.u8Soc);		/*BMS SOC*/
				i32SetPara(PARA_BrakeValveCurrent, abs(gCanRevPdoInfo.CanRevInfo2.i16SteerAngle));
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
		case LEAN_FORWARD_VALVE:
			i32ErrCodeSet(LEAN_FORWARD_VALVE_ERR);
			/*add ErrCode*/
			break;
		case LEAN_BACKWARD_VALVE:
			
//			i32ErrCodeSet(LEAN_BACKWARD_VALVE_ERR);
		#ifdef CHANGE_240403_SHIJIN
		i32ErrCodeClr(ErrCode56);
		i32ErrCodeClr(LEAN_BACKWARD_VALVE_ERR);
		#endif 
			/*add ErrCode*/
			break;
		default:
			break;
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
	static uint16_t u16CanRev1ACCnt;
	
	if (LiBattery == sgUserInfo.u8BatteryType)
	{
		if (CanRev1ACInfoLast.b1ToggleBit == gCanRevPdoInfo.CanRevInfo1.b1ToggleBit)
		{
			u16CanRev1ACCnt++;
			if (u16CanRev1ACCnt >= CAN_1AC_LOST_NO)
			{
				/*1AC Lost Err*/
				i32ErrCodeSet(BMS_NOCAN_ERR);
			}
		}
		else
		{
			u16CanRev1ACCnt = 0;
			i32ErrCodeClr(BMS_NOCAN_ERR);
		}
		
		if(0 != memcmp((char*)CanRev1ACInfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo1.u8Data, sizeof(CanRev1ACInfoLast)))
		{
			memcpy((char*)CanRev1ACInfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo1.u8Data, sizeof(CanRev1ACInfoLast));
			/*添加相关的操作*/
		
			gCanSendPdoInfo.CanSend23CInfo.u8Soc = CanRev1ACInfoLast.u8Soc;
			
			if (CanRev1ACInfoLast.u8Soc <= BAT_LOW_ERR_VAL)
			{
				i32ErrCodeSet(BAT_LOW_2_ERR);
			}
			else if (CanRev1ACInfoLast.u8Soc <= BAT_LOW_WARING_VAL)
			{
				i32ErrCodeSet(BAT_LOW_1_ERR);
				i32ErrCodeClr(BAT_LOW_2_ERR);
			}
			else
			{
				i32ErrCodeClr(BAT_LOW_2_ERR);
				i32ErrCodeClr(BAT_LOW_1_ERR);
			}
			
			if (1 == CanRev1ACInfoLast.b1DisableLiftUp)
			{
				sgValvesInfo.b4NoLiftUp |= 1 << NoLiftUp_LowBat;
				i32ErrCodeSet(BMS_LIFTUP_ERR);
			}
			else
			{
				sgValvesInfo.b4NoLiftUp &= ~(1 << NoLiftUp_LowBat);
				i32ErrCodeClr(BMS_LIFTUP_ERR);
			}
			
			if (1 == CanRev1ACInfoLast.b1LimitSpd2)
			{
				sgValvesInfo.b4Gear4Spd |= 1 << Gear4_Spd_Con2;
				i32ErrCodeSet(BMS_SPD_STOP_ERR);
			}
			else
			{
				sgValvesInfo.b4Gear4Spd &= ~(1 << Gear4_Spd_Con2);
				i32ErrCodeClr(BMS_SPD_STOP_ERR);
			}
			
			if (1 == CanRev1ACInfoLast.b1LimitSpd1)
			{
				sgValvesInfo.b4Gear3Spd |= 1 << Gear3_Spd_Con1;
				i32ErrCodeSet(BMS_SPD_LIMIT_ERR);
			}
			else
			{
				sgValvesInfo.b4Gear3Spd &= ~(1 << Gear3_Spd_Con1);
				i32ErrCodeClr(BMS_SPD_LIMIT_ERR);
			}
			

			if (1 == CanRev1ACInfoLast.b1MainDriverOpen)
			{
				i32ErrCodeSet(BMS_MC_OPEN_ERR);
			}
			else
			{
				i32ErrCodeClr(BMS_MC_OPEN_ERR);
			}
		}
	}
	else
	{
		if (CanRev1ACInfoLast.b1ToggleBit != gCanRevPdoInfo.CanRevInfo1.b1ToggleBit)
		{
			/*添加参数报警*/
			i32ErrCodeSet(BAT_PARA_ERR);
		}
	}
}


static void vCanId42CProc(tCanFrame * CanFrame)
{
	uint8_t i = 0;
	xCanRev42CInfo CanRev42CInfo;
	
	xCanSend3ACInfo CanSend3ACInfo = {0};
	tCanFrame CanSendFrame;
	
	//if(0 != memcmp((char*)sgLastCanRev42CInfo.u8Data, (char*)CanFrame->u8Data, sizeof(sgLastCanRev42CInfo)))
	{
		memcpy((char*)CanRev42CInfo.u8Data, (char*)CanFrame->u8Data, sizeof(CanRev42CInfo));
		
		uint8_t u8Crc1 = u8SeedArray[gCanSendPdoInfo.CanSend23CInfo.u8WorkTimeLL & 0x0F] ^ gCanSendPdoInfo.CanSend23CInfo.u8WorkTimeLL;
		uint8_t u8Crc2 = u8SeedArray[(gCanSendPdoInfo.CanSend23CInfo.u8WorkTimeLL >> 4) & 0x0F] ^ gCanSendPdoInfo.CanSend23CInfo.u8WorkTimeLL;
		
		if ((u8Crc2 != CanRev42CInfo.u8Crc1) || (u8Crc1 != CanRev42CInfo.u8Crc2))
		{
			CanSend3ACInfo.u8RepStat = 0x02;	/*CRC err*/
		}
		else if (CanRev42CInfo.b1StartStat == CanRev42CInfo.b1StopStat)
		{
			CanSend3ACInfo.u8RepStat = 0xFF;	/*Cmd  err*/
		}
		else
		{
			CanSend3ACInfo.u8RepStat = 0x01;	/*OK*/
			{
				sgUserInfo.b1RentalStop = CanRev42CInfo.b1StopStat;
				sgUserInfo.b1RentalStart = CanRev42CInfo.b1StartStat;
				sgUserInfo.b1RentalMode = CanRev42CInfo.b1Mode;
				sgUserInfo.u16RentalTime = 	CanRev42CInfo.u16SetTime;			
				i32SetPara(RENTAL_TIME, CanRev42CInfo.u16SetTime);
				i32SetPara(RENTAL_INFO, CanRev42CInfo.u8Data[0]);
				u16SaveParaToEeprom(RENTAL_TIME, CanRev42CInfo.u16SetTime);
				u16SaveParaToEeprom(RENTAL_INFO, CanRev42CInfo.u8Data[0]);
			}
		}
		CanSend3ACInfo.u16SetTime = CanRev42CInfo.u16SetTime;
		CanSend3ACInfo.u8Crc1 = u8Crc2;
		CanSend3ACInfo.u8Crc2 = u8Crc1;
	}
	
	CanSendFrame.u32ID = 0x3AC;
	CanSendFrame.u8Rtr = 0;
	CanSendFrame.u16DataLength = 8;
	for (i=0; i<8; i++)                                 
	{
		CanSendFrame.u8Data[i] = CanSend3ACInfo.u8Data[i];
	}
	
	i32CanWrite(Can0, &CanSendFrame);
	i32LogWrite(DEBUG, LOG_USER, "Process 42C Frame!!!\r\n");
}

static void vCanId5ACSend(void)
{
	static uint8_t u8Cnt = 0;
	
	tCanFrame Can5ACSend = {.u32ID = 0x5AC, .u16DataLength = 8, .u8Data = {0}, .u8Rtr = 0};
	if (u8Cnt++ < CANID_5AC_SEND_NO)
	{
		i32CanWrite(Can0, &Can5ACSend);
		
		i32LogWrite(DEBUG, LOG_USER, "Send 5AC Frame!!!\r\n");
	}	
}
static void vDeviceCanId62CProc(tCanFrame * CanFrame)
{
	xCanRevDevice62CInfo CanRevDevice62CInfo;
	
	memcpy((char*)CanRevDevice62CInfo.u8Data, (char*)CanFrame->u8Data, sizeof(CanRevDevice62CInfo));
	
	if (1 == sgUserInfo.b1PasswordFunc)
	{
		/* add  user logic*/
		if (0x00 == CanRevDevice62CInfo.u8State)
		{
			sgValvesInfo.u8NoAct |= 1 << NoAct_Security;
			i32ErrCodeSet(DEV_62C_LOST_ERR);
		}
		else if (0x5A == CanRevDevice62CInfo.u8State)
		{
			sgValvesInfo.u8NoAct &= ~(1 << NoAct_Security);
			i32ErrCodeClr(DEV_62C_LOST_ERR);
		}
	}

	u16CanRev62CCnt = 0;		/*Rev 62C, Clear 62C Cnt*/
//	i32ErrCodeClr(DEV_62C_LOST_ERR);
	
	i32LogWrite(DEBUG, LOG_USER, "Device 62C Rev!!!\r\n");	
}

static void vPcCanId62CProc(tCanFrame * CanFrame)
{
	static xCanRevPc62CInfo sgLastCanRevPc62CInfo;
//	i32LogWrite(DEBUG, LOG_USER, "sgUserInfo.b1PasswordFunc = %d\r\n", CanFrame->u8Data[4]);
	if(0 != memcmp((char*)sgLastCanRevPc62CInfo.u8Data, (char*)CanFrame->u8Data, sizeof(sgLastCanRevPc62CInfo)))
	{
		memcpy((char*)sgLastCanRevPc62CInfo.u8Data, (char*)CanFrame->u8Data, sizeof(sgLastCanRevPc62CInfo));
		/* add  user logic*/
		{
			sgUserInfo.b1PasswordFunc = sgLastCanRevPc62CInfo.b1PasswordFunc;
			sgSaveState.b1PasswordFunc = sgLastCanRevPc62CInfo.b1PasswordFunc;
			u16EepromWrite(PARA_SaveState, sgSaveState.u16Data, 1);	
			if (1 == sgUserInfo.b1PasswordFunc)
			{
				sgValvesInfo.u8NoAct |= 1 << NoAct_Security;
				i32ErrCodeSet(DEV_62C_LOST_ERR);
			}
			else
			{
				sgValvesInfo.u8NoAct &= ~(1 << NoAct_Security);
				i32ErrCodeClr(DEV_62C_LOST_ERR);
			}
			i32LogWrite(DEBUG, LOG_USER, "sgUserInfo.b1PasswordFunc = %d\r\n", sgUserInfo.b1PasswordFunc);
		}		
	}
	i32LogWrite(DEBUG, LOG_USER, "PC 62C Rev!!!\r\n");	
}

static void vGetMotorFromMcu(uint8_t u8CS,uint16_t u16SendAdress)
{
	uint8_t u8Vector[8];
	u8Vector[0] = u8CS;
	u8Vector[1] = u16SendAdress & 0xFF;
	u8Vector[2] = (u16SendAdress >> 8) & 0xFF;
	u8Vector[3] = 0;
	u8Vector[4] = 0x00 & 0xFF;
	u8Vector[5] = (0x00 >> 8) & 0xFF;
	u8Vector[6] = (0x00 >> 16) & 0xFF;
	u8Vector[7] = (0x00 >> 24) & 0xFF;
	
	vQueryMcuPara(u8Vector,8);
}

static void vSetMotorFromMcu(uint8_t u8CS,uint16_t u16SendAdress,uint16_t SetData)
{
	uint8_t u8Vector[8];
	u8Vector[0] = u8CS;
	u8Vector[1] = u16SendAdress & 0xFF;
	u8Vector[2] = (u16SendAdress >> 8) & 0xFF;
	u8Vector[3] = 0;
	u8Vector[4] = SetData & 0xFF;
	u8Vector[5] = (SetData >> 8) & 0xFF;
	u8Vector[6] = (0x00 >> 16) & 0xFF;
	u8Vector[7] = (0x00 >> 24) & 0xFF;
	
	vQueryMcuPara(u8Vector,8);
}

static void vPARAWrite62CProc(tCanFrame * CanFrame)
{
	uint16_t u16DataWrite = 0;
	uint16_t u16LocalData = 0;
	uint8_t u8WriteReturnFlg = 1;
	tCanFrame Can5ACSend = {.u32ID = 0x5AC, .u16DataLength = 8, .u8Data = {0x00}, .u8Rtr = 0};
	memcpy((char*)CanRev62CInfoLast.u8Data, (char*) CanFrame->u8Data, sizeof(CanRev62CInfoLast));
	u16DataWrite = (CanRev62CInfoLast.u8DataLL | (CanRev62CInfoLast.u8DataLH << 8));

	if(0x22 == CanRev62CInfoLast.SendCmd)          //写MCU
	{
		MstPARADate[CanRev62CInfoLast.Index] = u16DataWrite;
		vSetMotorFromMcu(0x2B,CanRev62CInfoLast.Index,u16DataWrite);
	}
	else if(0x23 == CanRev62CInfoLast.SendCmd)         //写ECU
	{
		u16EepromRead(CanRev62CInfoLast.Index,&u16LocalData,1);
		if(u16LocalData != u16DataWrite)
		{
			u16LocalData = u16DataWrite;
			u8WriteReturnFlg = u16EepromWrite(CanRev62CInfoLast.Index, u16LocalData, 1);
			if(0 == u8WriteReturnFlg)
					Can5ACSend.u8Data[0] = 0x60;
			else
					Can5ACSend.u8Data[0] = 0x80;
			Can5ACSend.u8Data[1] = CanRev62CInfoLast.Index;
			Can5ACSend.u8Data[2] = CanRev62CInfoLast.SendCmd;
			Can5ACSend.u8Data[3] = 0;
			Can5ACSend.u8Data[4] = u16LocalData & 0xFF;
			Can5ACSend.u8Data[5] = (u16LocalData >> 8) & 0xFF;
			Can5ACSend.u8Data[6] = 0x00 & 0xFF;
			Can5ACSend.u8Data[7] = 0x00 & 0xFF;
			i32CanWrite(Can0, &Can5ACSend);
		}
	}
}

static void vPARARead62CProc(tCanFrame * CanFrame)
{
	uint16_t u16DataReq = 0;
	void * MotorVal;
	tCanFrame Can5ACSend = {.u32ID = 0x5AC, .u16DataLength = 8, .u8Data = {0x00}, .u8Rtr = 0};
	memcpy((char*)CanRev62CInfoLast.u8Data, (char*) CanFrame->u8Data, sizeof(CanRev62CInfoLast));
	
	if((0x40 == CanRev62CInfoLast.SendCmd)&&(0x4E == CanRev62CInfoLast.Index))
	{
		Can5ACSend.u8Data[0] = CanFrame->u8Data[0];
		Can5ACSend.u8Data[1] = CanFrame->u8Data[1];
		Can5ACSend.u8Data[2] = CanFrame->u8Data[2];
		Can5ACSend.u8Data[3] = CanFrame->u8Data[3];
		Can5ACSend.u8Data[4] = 0x0B & 0xFF;
		Can5ACSend.u8Data[5] = 0xCD & 0xFF;
		Can5ACSend.u8Data[6] = 0xB0 & 0xFF;
		Can5ACSend.u8Data[7] = 0xC7 & 0xFF;
		i32CanWrite(Can0, &Can5ACSend);
	}
	else if(0x22 == CanRev62CInfoLast.SendCmd)          //读MCU
	{
		u16DataReq = MstPARADate[CanRev62CInfoLast.Index];
		Can5ACSend.u8Data[0] = CanRev62CInfoLast.FuncCode;
		Can5ACSend.u8Data[1] = CanRev62CInfoLast.Index;
		Can5ACSend.u8Data[2] = CanRev62CInfoLast.SendCmd;
		Can5ACSend.u8Data[3] = 0;
		Can5ACSend.u8Data[4] = u16DataReq & 0xFF;
		Can5ACSend.u8Data[5] = (u16DataReq >> 8) & 0xFF;
		Can5ACSend.u8Data[6] = 0x00 & 0xFF;
		Can5ACSend.u8Data[7] = 0x00 & 0xFF;
		i32CanWrite(Can0, &Can5ACSend);
	}
	else if(0x23 == CanRev62CInfoLast.SendCmd)         //读ECU
	{
		u16EepromRead(CanRev62CInfoLast.Index,&u16DataReq,1);
		Can5ACSend.u8Data[0] = CanRev62CInfoLast.FuncCode;
		Can5ACSend.u8Data[1] = CanRev62CInfoLast.Index;
		Can5ACSend.u8Data[2] = CanRev62CInfoLast.SendCmd;
		Can5ACSend.u8Data[3] = 0;
		Can5ACSend.u8Data[4] = u16DataReq & 0xFF;
		Can5ACSend.u8Data[5] = (u16DataReq >> 8) & 0xFF;
		Can5ACSend.u8Data[6] = 0x00 & 0xFF;
		Can5ACSend.u8Data[7] = 0x00 & 0xFF;
		i32CanWrite(Can0, &Can5ACSend);
	}
	else if(0x32 == CanRev62CInfoLast.SendCmd)   //读MCU监控
	{
		vGetMotorFromMcu(0x40,(0x4000+CanRev62CInfoLast.Index));
	}
	else if(0x33 == CanRev62CInfoLast.SendCmd)  //读ECU监控
	{
		MotorVal = u16pGetParaPoint(200+CanRev62CInfoLast.Index);
		u16DataReq = *(uint16_t *)MotorVal;
		Can5ACSend.u8Data[0] = CanRev62CInfoLast.FuncCode;
		Can5ACSend.u8Data[1] = CanRev62CInfoLast.Index;
		Can5ACSend.u8Data[2] = CanRev62CInfoLast.SendCmd;
		Can5ACSend.u8Data[3] = 0;
		Can5ACSend.u8Data[4] = u16DataReq & 0xFF;
		Can5ACSend.u8Data[5] = (u16DataReq >> 8) & 0xFF;
		Can5ACSend.u8Data[6] = 0x00 & 0xFF;
		Can5ACSend.u8Data[7] = 0x00 & 0xFF;
		i32CanWrite(Can0, &Can5ACSend);
	}
}

static void vMcuParaRevProc(uint8_t *u8Data, uint16_t u16Length)
{
	uint8_t u8DataArray[8];
	uint32_t u32DataSend = 0;
	uint8_t ReqCmd = 0;
	memcpy(u8DataArray,u8Data,8);
	
	tCanFrame CanSendFrame;
	u32DataSend = u8DataArray[4]|(u8DataArray[5]<<8)|(u8DataArray[6]<<16)|(u8DataArray[7]<<24);
	if(true == PowerOnGetPara)
	{
		ReceiveCount++;
		if(ReceiveCount > 2)
		{
			if(1 == SendOk)
			{
				SendOk = 0;
				MstParaIsOk = 1;
				MstPARADate[RevMstDate] = u32DataSend;
				RevMstDate++;				
				ReceiveCount = 0;
			}
		}
	}
	else
	{
		CanSendFrame.u32ID = 0x5AC;
		CanSendFrame.u8Rtr = 0;
		CanSendFrame.u16DataLength = 8;
		
		if(0x43 == u8DataArray[0])     //特殊处理 非标准Canopen协议
		{
			ReqCmd = 0x42;
		}
		else
		{
			ReqCmd = u8DataArray[0];
		}

		CanSendFrame.u8Data[0] = ReqCmd;
		CanSendFrame.u8Data[1] = u8DataArray[1];
		CanSendFrame.u8Data[2] = 0x32;//u8DataArray[2];
		CanSendFrame.u8Data[3] = 0x00;
		CanSendFrame.u8Data[4] = u32DataSend & 0xFF;
		CanSendFrame.u8Data[5] = (u32DataSend>>8) & 0xFF;
		CanSendFrame.u8Data[6] = (u32DataSend>>16) & 0xFF;
		CanSendFrame.u8Data[7] = (u32DataSend>>24) & 0xFF;

		i32CanWrite(Can0, &CanSendFrame);
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
	
	/*租赁设备要求*/
	if ((1 == sgUserInfo.b1RentalStop) ||
		((1 == sgUserInfo.b1RentalStart) && (1 == sgUserInfo.b1RentalMode) && (0 == sgUserInfo.u16RentalTime)))
	{
		sgValvesInfo.b4NoLiftUp |= 1 << NoLiftUp_RentalTimeOut;	//禁止起升
		sgValvesInfo.b4Gear4Spd |= 1 << Gear4_Spd_Con3;					//速度限制为20%
		i32DoPwmSet(LIFTUP_VALVE, DRIVER_CLOSE);
//		i32DoPwmSet(LEAN_FORWARD_VALVE, DRIVER_CLOSE);
//		i32DoPwmSet(LEAN_BACKWARD_VALVE, DRIVER_CLOSE);
		
		i32ErrCodeSet(RENTAL_TIMEOUT_ERR);		
	}
	else
	{
		sgValvesInfo.b4NoLiftUp &= ~(1 << NoLiftUp_RentalTimeOut);
		sgValvesInfo.b4Gear4Spd &= ~(1 << Gear4_Spd_Con3);
		i32ErrCodeClr(RENTAL_TIMEOUT_ERR);
	}	
	
	/*add lock */
	if ((1 == SwiInput.b1Forward) || (1 == SwiInput.b1Backward) ||
	   (1 == SwiInput.b1LeanForward) || (1 == SwiInput.b1LeanBackward) ||
	   (1 == SwiInput.b1LiftUp) || (1 == SwiInput.b1LiftDown))
	{
		/*禁止起升和前进后退*/
		if ((0 != gCanRevPdoInfo.CanRevInfo2.u8ErrSteer) || (0 == SwiInput.b1FaultLockOut))
		{
			sgValvesInfo.b4NoLiftUp |= 1 << NoLiftUp_FaultLock;
			sgValvesInfo.b4NoMove |= 1 << NoMove_FaultLock;
			i32ErrCodeSet(ACT_FAULT_LOCK_ERR);
		}
	}
		
	if (0 == SwiInput.b1Lock)
	{
		if ((1 == SwiInput.b1Forward) || (1 == SwiInput.b1Backward) ||
			((0 == sgUserInfo.b1LiftLock) && 
			((1 == SwiInput.b1LeanForward) || (1 == SwiInput.b1LeanBackward) ||
		     (1 == SwiInput.b1LiftUp) || (1 == SwiInput.b1LiftDown))))
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
			if ((1 == SwiInput.b1UpLimit) && (1 == sgValvesInfo.b1LiftUpStat))
			//if ((0 == sgSwiInput.b1UpLimit) && (1 == SwiInput.b1UpLimit) && (1 == sgValvesInfo.b1LiftUpStat))
			{
//				sgValvesInfo.b4Gear2Spd = 1 << Gear2_Spd_Con1;
				sgSwiInput.b1HeightSpdLimit = 1;
				sgSaveState.b1HeightSpdLimit = 1;
				u16EepromWrite(PARA_SaveState, sgSaveState.u16Data, 1);	
			}
			else if ((1 == SwiInput.b1UpLimit) && (1 == sgValvesInfo.b1LiftDownStat))
			//else if ((0 == sgSwiInput.b1UpLimit) && (1 == SwiInput.b1UpLimit) && (1 == sgValvesInfo.b1LiftDownStat))
			{
//				sgValvesInfo.b4Gear2Spd &=  ~(1 << Gear2_Spd_Con1);
				sgSwiInput.b1HeightSpdLimit = 0;
				sgSaveState.b1HeightSpdLimit = 0;
				u16EepromWrite(PARA_SaveState, sgSaveState.u16Data, 1);	
			}
			if ((1 == SwiInput.b1Meter1m8) && (1 == sgValvesInfo.b1LiftUpStat))
			//if ((0 == sgSwiInput.b1Meter1m8) && (1 == SwiInput.b1Meter1m8) && (1 == sgValvesInfo.b1LiftUpStat))
			{
//				sgValvesInfo.b4Gear2Spd = 1 << Gear2_Spd_Con2;
				sgSwiInput.b1Above1M8 = 1;
				sgSaveState.b1Above1M8 = sgSwiInput.b1Above1M8;
				u16EepromWrite(PARA_SaveState, sgSaveState.u16Data, 1);	
			}
			else if ((1 == SwiInput.b1Meter1m8) && (1 == sgValvesInfo.b1LiftDownStat))
			//else if ((0 == sgSwiInput.b1Meter1m8) && (1 == SwiInput.b1Meter1m8) && (1 == sgValvesInfo.b1LiftDownStat))
			{
//				sgValvesInfo.b4Gear2Spd &=  ~(1 << Gear2_Spd_Con2);
				sgSwiInput.b1Above1M8 = 0;
				sgSaveState.b1Above1M8 = sgSwiInput.b1Above1M8;
				u16EepromWrite(PARA_SaveState, sgSaveState.u16Data, 1);
			}
			
			SwiInput.b1Above1M8 = sgSwiInput.b1Above1M8;			/*lilu 20230801 Above1M8 retain*/
			SwiInput.b1HeightSpdLimit = sgSwiInput.b1HeightSpdLimit;
			sgSwiInput.u16data = SwiInput.u16data;   /**/
			
			gCanSendPdoInfo.CanSend23CInfo.b1SeatSwi = sgSwiInput.b1Lock;
			gCanSendPdoInfo.CanSend23CInfo.b1ForWardSwi = sgSwiInput.b1Forward;
			gCanSendPdoInfo.CanSend23CInfo.b1BackWardSwi = sgSwiInput.b1Backward;
			gCanSendPdoInfo.CanSend23CInfo.b1LiftUpSwi = sgSwiInput.b1LiftUp;
			gCanSendPdoInfo.CanSend23CInfo.b1LiftDownSwi = sgSwiInput.b1LiftDown;
			gCanSendPdoInfo.CanSend23CInfo.b1EmsSwi = sgSwiInput.b1Ems;
				
			if ((0 == sgSwiInput.b1GuardRaild) && (0 == sgSwiInput.b1PedalClose) && (1 == sgSwiInput.b1PedalOpen))
			{
				//sgValvesInfo.b1MoveFlag = 0;
				sgValvesInfo.b4NoMove &= ~(1 << NoMove_Pedal);
				sgValvesInfo.b4Gear1Spd &= ~(1 << Gear1_Spd_Con1);
			}
			/*速度变为1档*/
			else if (((1 == sgSwiInput.b1GuardRaild) && (0 == sgSwiInput.b1PedalClose) && (1 == sgSwiInput.b1PedalOpen)) ||  /*SW1=1,DRIVER6-R=0,SW8=1*/
					  ((1 == sgSwiInput.b1GuardRaild) && (1 == sgSwiInput.b1PedalClose) && (0 == sgSwiInput.b1PedalOpen)))	/*SW1=1,DRIVER6-R=1,SW8=0*/
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
			

			if ((1 == sgSwiInput.b1HeightSpdLimit) || (1 == SwiInput.b1UpLimit))
			{
				sgValvesInfo.b4Gear2Spd = 1 << Gear2_Spd_Con1;
			}
			else
			{
				sgValvesInfo.b4Gear2Spd &=  ~(1 << Gear2_Spd_Con1);
			}
			/*Limit Up*/
			if ((1 == sgSwiInput.b1Above1M8) || (1 == SwiInput.b1Meter1m8))
			{
				sgValvesInfo.b4Gear2Spd = 1 << Gear2_Spd_Con2;
				if (0 == sgSwiInput.b1GuardRaild)
				{
					sgValvesInfo.b4NoLiftUp |= 1 << NoLiftUp_HeightLimit;
				}
				else
				{
					sgValvesInfo.b4NoLiftUp &= ~(1 << NoLiftUp_HeightLimit);
				}
			}
			else
			{
				sgValvesInfo.b4Gear2Spd &=  ~(1 << Gear2_Spd_Con2);
				sgValvesInfo.b4NoLiftUp &= ~(1 << NoLiftUp_HeightLimit);
			}
//			if ((1 == sgSwiInput.b1Above1M8) && (0 == sgSwiInput.b1GuardRaild)) 
//			{
//				sgValvesInfo.b4NoLiftUp |= 1 << NoLiftUp_HeightLimit;
//			}
//			else
//			{
//				sgValvesInfo.b4NoLiftUp &= ~(1 << NoLiftUp_HeightLimit);
//			}
							
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
			
			if (1 == sgSwiInput.b1LeanForward)
			{
				sgValvesInfo.b1LeanForWardStat = 1;
			}
			else
			{
				sgValvesInfo.b1LeanForWardStat = 0;
			}
			
			if (1 == sgSwiInput.b1LeanBackward)
			{
				sgValvesInfo.b1LeanBackWardStat = 1;
			}
			else
			{
				sgValvesInfo.b1LeanBackWardStat = 0;
			}
			
			/*lilu 20230823 油泵电机之间的相互动作做到互斥，同时动作时，复位相关动作*/
			{
				uint8_t u8PumpTmp = 0;
				u8PumpTmp = sgValvesInfo.b1LiftUpStat + sgValvesInfo.b1LiftDownStat + 			
							sgValvesInfo.b1LeanForWardStat + sgValvesInfo.b1LeanBackWardStat;
				if ((1 == sgValvesInfo.b1PumpMotorNoAct) || (u8PumpTmp > 1))
				{
					sgValvesInfo.b1LiftUpStat = 0;
					sgValvesInfo.b1LiftDownStat = 0;
					sgValvesInfo.b1LeanForWardStat = 0;
					sgValvesInfo.b1LeanBackWardStat = 0;
					sgValvesInfo.b1PumpMotorNoAct = 1;
					i32ErrCodeSet(MUTL_PUMP_REQ_ERR);
				}
				/*当油泵所有相关都复位，后面的动作再次有效*/
				if ((0 == sgSwiInput.b1LeanForward) && (0 == sgSwiInput.b1LeanBackward) && \
					(0 == sgSwiInput.b1LiftUp) && (0 == sgSwiInput.b1LiftDown) && \
					(0 == u8PumpOrPropValue))
				{
					sgValvesInfo.b1PumpMotorNoAct = 0;
					i32ErrCodeClr(MUTL_PUMP_REQ_ERR);
				}
			}
			
			/*lilu 20230823 起升时踏板必须收起且护栏放下或者踏板放下且站人，否则所有油泵电机关闭操作*/
			{
				if (((1 == sgUserInfo.b1LiftPedal) && (0 == sgSwiInput.b1PedalClose) && (0 == sgSwiInput.b1PedalOpen))
				|| 	((1 == sgUserInfo.b1LiftPedal) && (1 == sgSwiInput.b1PedalClose) && (0 == sgSwiInput.b1GuardRaild))
				)
				{
					sgValvesInfo.b1LiftUpStat = 0;
					sgValvesInfo.b1LiftDownStat = 0;
					sgValvesInfo.b1LeanForWardStat = 0;
					sgValvesInfo.b1LeanBackWardStat = 0;
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
		(1 == i32LocalDiGet(LIFT_UP_OE_ENABLE_SWI)) || (1 == i32LocalDiGet(LIFT_DOWN_SWI)) ||		/*起升下降*/
		(1 == i32LocalDiGet(LEAN_FORWARD_SWI)) || (1 == i32LocalDiGet(LEAN_BACKWARD_SWI)) ||		/*前倾后倾*/
		(1 == i32LocalDiGet(EMS_SWI)) || 															/*急反*/
		((0 == sgUserInfo.b1StartUpLock) && (1 == i32LocalDiGet(LOCK_SWI))))						/*互锁*/
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
	xRevCallBackProc CanId42C = {.u32CanId = 0x42C, .u32Data = 0, .CallBack = vCanId42CProc};
	xRevCallBackProc DeviceCanId62C = {.u32CanId = 0x62C, {.u8DataCnt = 3, .u8Data0 = 0x22, .u8Data1 = 0x08, .u8Data2 = 0x50}, .CallBack = vDeviceCanId62CProc};
	xRevCallBackProc PcCanId62C = {.u32CanId = 0x62C, {.u8DataCnt = 3, .u8Data0 = 0x22, .u8Data1 = 0x02, .u8Data2 = 0x2A}, .CallBack = vPcCanId62CProc};
//	xRevCallBackProc PARAFirstCanId62C = {.u32CanId = 0x62C, {.u8DataCnt = 3, .u8Data0 = 0x42, .u8Data1 = 0x4E, .u8Data2 = 0x40}, .CallBack = vPARAFirstCanId62CProc};
	xRevCallBackProc PARARead62C = {.u32CanId = 0x62C, {.u8DataCnt = 1, .u8Data0 = 0x42}, .CallBack = vPARARead62CProc};
	xRevCallBackProc PARAWrite62C = {.u32CanId = 0x62C, {.u8DataCnt = 1, .u8Data0 = 0x22}, .CallBack = vPARAWrite62CProc};
	
	vMstRevRegister(vMstRevProc);
	vMstSendRegister(vMstSendProc);
	
	vCanRevMsgRegister(&CanId42C);
	vCanRevMsgRegister(&DeviceCanId62C);
	vCanRevMsgRegister(&PcCanId62C);
	vCanRevMsgRegister(&PARARead62C);
	vCanRevMsgRegister(&PARAWrite62C);
//	vCanRevMsgRegister(&PARAFirstCanId62C);
	
	vDoPwmErrReg(vDoPwmErrCallBack);
	vPropErrReg(vPropErrCallBack);
	vAiErrReg(vAiErrCallBack);
	vMcuParaRevRegister(vMcuParaRevProc);
	
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
		sgUserInfo.b1HourCntClr = (u16RentalInfo >> 3) & 0x01;
		if (1 == sgUserInfo.b1HourCntClr) /*lilu 20230823 add clr hourcnt func*/
		{
			u32HourCount = 0;
			sgUserInfo.b1HourCntClr = 0;
			i32SetPara(RENTAL_INFO, (u16RentalInfo & 0x07));
		}
	}
	
	u16EepromRead(PARA_DefaultFlag, &u16Tmp, 1);
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
		sgSwiInput.b1HeightSpdLimit = sgSaveState.b1HeightSpdLimit;
		sgUserInfo.b1PasswordFunc = sgSaveState.b1PasswordFunc;
		u16EepromRead(PARA_HourSetTime, &u16Tmp, 1);	
	}
	if (1 == sgUserInfo.b1PasswordFunc)
	{
		sgValvesInfo.u8NoAct |= 1 << NoAct_Security;
		i32ErrCodeSet(DEV_62C_LOST_ERR);
	}
//	//i32LogWrite(INFO, "******SaveState = 0x%x\r\n*********", sgSaveState.u16Data);
	
//	__disable_irq();
	gCanSendPdoInfo.CanSend23CInfo.u8WorkTimeLL = u32HourCount & 0xFF;
	gCanSendPdoInfo.CanSend23CInfo.u8WorkTimeLH = (u32HourCount >> 8) & 0xFF;
	gCanSendPdoInfo.CanSend23CInfo.u8WorkTimeHL = (u32HourCount >> 16) & 0xFF;
	gCanSendPdoInfo.CanSend23CInfo.u8WorkTimeHH = (u32HourCount >> 24) & 0xFF;
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
	uint8_t u8Vector1[8];

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
		SendCount++;
		if((SendCount > 2)&&(1 == MstParaIsOk))
		{
			MstParaIsOk = 0;
			ParaCount++;
			SendOk = 1;
			SendCount = 0;
		}		
	}
	else
	{
		PowerOnGetPara = false;
	}
	
	if(1 == u8EcuProcFlag)
	{
		if (LiBattery != sgUserInfo.u8BatteryType)
		{
			gCanSendPdoInfo.CanSend23CInfo.u8Soc = u8GetBatterySoc();
			
			sgValvesInfo.b4Gear4Spd &= ~(1 << Gear4_Spd_Con1);
			sgValvesInfo.b4NoLiftUp &= ~(1 << NoLiftUp_LowBat);
			i32ErrCodeClr(BAT_LOW_2_ERR);
			i32ErrCodeClr(BAT_LOW_1_ERR);
			
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
		
		u16CanId5ACPeriod++;
		if (u16CanId5ACPeriod < 210)	/*200 = 5Times*/
		{
			if (0 == (u16CanId5ACPeriod % CANID_5AC_SEND_PERIOD))	/*Send 5 times 5AC*/
			{
				vCanId5ACSend();
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
				i32ErrCodeSet(DEV_62C_LOST_ERR);
				u16CanRev62CCnt = 0;
			}
		}
		
		vAiMonitor();
		vSwiMonitor();		
		vCanRevPdoProc();
		u8ErrCode = u8ErrCodeGet();
		if (0 != u8ErrCode)
		{
			gCanSendPdoInfo.CanSend23CInfo.u8ErrCode = u8ErrSwitchArray[u8ErrCode];
			vLedSendAlmCode(u8ErrCode);
		}
		else
		{
			gCanSendPdoInfo.CanSend23CInfo.u8ErrCode = 0;
		}
	}
	
	if (true == u8GetNetTimerOverFlag(TIMER_HourCount))
	{
		vResetNetTimer(TIMER_HourCount);	
		
		if (1 == sgSwiInput.b1Lock)
		{
			u16SecCnt++;
		}
		
		if (u16SecCnt >= 360)			/**/
		{
			u16SecCnt = 0;
			u32HourCount++;
			vHourCountWrite(u32HourCount);
			__disable_irq();
			gCanSendPdoInfo.CanSend23CInfo.u8WorkTimeLL = u32HourCount & 0xFF;
			gCanSendPdoInfo.CanSend23CInfo.u8WorkTimeLH = (u32HourCount >> 8) & 0xFF;
			gCanSendPdoInfo.CanSend23CInfo.u8WorkTimeHL = (u32HourCount >> 16) & 0xFF;
			gCanSendPdoInfo.CanSend23CInfo.u8WorkTimeHH = (u32HourCount >> 24) & 0xFF;
			__enable_irq();
		}
		
		/*Rental Info*/
		{
			uint16_t u16RentalInfo = 0;
			u16RentalInfo = i32GetPara(RENTAL_INFO);
			sgUserInfo.b1RentalStop = u16RentalInfo & 0x01;
			sgUserInfo.b1RentalStart = (u16RentalInfo >> 1) & 0x01;
			sgUserInfo.b1RentalMode = (u16RentalInfo >> 2) & 0x01;
			sgUserInfo.b1HourCntClr = (u16RentalInfo >> 3) & 0x01;
			if (1 == sgUserInfo.b1HourCntClr) /*lilu 20230823 add clr hourcnt func*/
			{
				u32HourCount = 0;
				sgUserInfo.b1HourCntClr = 0;
				u16SecCnt = 0;
				i32SetPara(RENTAL_INFO, (u16RentalInfo & 0x07));
				vHourCountWrite(u32HourCount);
				__disable_irq();
				gCanSendPdoInfo.CanSend23CInfo.u8WorkTimeLL = u32HourCount & 0xFF;
				gCanSendPdoInfo.CanSend23CInfo.u8WorkTimeLH = (u32HourCount >> 8) & 0xFF;
				gCanSendPdoInfo.CanSend23CInfo.u8WorkTimeHL = (u32HourCount >> 16) & 0xFF;
				gCanSendPdoInfo.CanSend23CInfo.u8WorkTimeHH = (u32HourCount >> 24) & 0xFF;
				__enable_irq();
			}
		}
		if (1 == sgSwiInput.b1Lock)
		{
			u16RentalCnt++;
		}
		
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
