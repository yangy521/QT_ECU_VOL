/*******************************************************************************
* Filename: UserStackerTruckProc.c	                                           *
* Description: 逻辑层的源文件，包含对应的回调函数	   						   *
* Author:                                                      *
* Date: 2023/05/30    														   *
* Revision:	 														 		   *
*******************************************************************************/
#include "UserCanComm.h"
#include "UserStackerTruckProcJiaLi.h"
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

#if (USER_TYPE == USER_JIALI_15TJDC)

#define BMS_NOLIFTUP			(0x0001 << 0)
#define SWI_NOLIFTUP			(0x0001 << 1)

const static xPdoParameter  sgPdoPara = 
{
	.TpdoPara = {
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 100, .u16CanId = 0x260 }
	},
	.RpdoPara = {
		{.b1Flag = 1, .b11CanRevId = 0x360},
		{.b1Flag = 1, .b11CanRevId = 0x444},
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
		uint16_t b1Fence2: 1;                 //两个护臂
		uint16_t b1SafeLock: 1;               //互锁
		uint16_t b1Pedal: 1;                  //踏板
		uint16_t b1HeightLimit:1;             //1.8M 高度限位
		uint16_t b1LiftLimit: 1;              //起升限位
		uint16_t b1SteerSafeLock: 1;          //转向互锁
		uint16_t b1Ems: 1;                    //紧急反向（待定）
		uint16_t b1LiftUp: 1;           //起升标志
		uint16_t b1LiftDown: 1;        //下降标志
		uint16_t b1Walk_Upright:1;        //直立行走标志
		uint16_t b1Slow_Mode:1;          //龟速模式
		uint16_t b1noAct:1;         //禁止行走
		uint16_t b1ChargeLimitAct: 1;
		uint16_t b3Reserve:2;
	};
}xSwiInput;

typedef union
{
	uint8_t u16data;
	struct
	{
		uint8_t b1BackWard: 1;        
		uint8_t b1ForWard: 1;                
		uint8_t b1ForErake: 1;              
		uint8_t b1BackErake: 1;                 
		uint8_t b1Ems:1;            
		uint8_t b1TorSpeed: 1;                                    
		uint8_t b5Reserve: 2;
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
static xhandleInput sgHandleInput;
static uint32_t u32HourCount = 0;
static int16_t i16SteerAngle = 0;
static int16_t i16SHanderValue = 0;
static uint8_t MOVE_FLAG = 0;
static uint8_t LIFT_FLAG = 0;
static uint8_t HANDLE_FLAG = 0;
static uint8_t LR_FLAG = 0;
static uint16_t DRIVER_FLAG = 0;
static uint8_t SnailRequest_Flag = 0;
static uint8_t receivedId112Message = 0;
int32_t i32PropValue = 0;
static _iq i32PropValueRate = 0;
static uint16_t Rev_Speed = 0;
/*PDO接收待完善*/
xCanRevPdoInfo gCanRevPdoInfo; 
static xCanRev22CInfo CanRev22CInfoLast;
static xCanRev360Info CanRev360InfoLast;
static xCanRev444Info CanRev444InfoLast;
//{
//	.CanRevInfo1.u8Soc = 100,					/*BmsSoc Default: 100*/
//};				
/*PDO发送待完善*/
xCanSendPdoInfo gCanSendPdoInfo;			


xhandleInput HandleInput = {0};


const static uint8_t u8ErrSwitchArray[ErrCodeMax] = 
{	
/*  0    1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18   19*/
	0,   82,  208, 180, 0,   227, 227, 227, 211, 211, 0,   180, 180, 60,  38,  216, 19,  212, 62,  65,
	78,  37,  53,  227, 75,  17,  248, 0,   0,   227, 0,   19,  62,  62,  65,  227, 242, 242, 210, 208,
	79,  66,  66,  0,   0,   0,   0,   0,   0,   0,   242, 242, 242, 242, 242, 242, 242, 242, 242, 242,
	60,  61,  62,  79,  79,  205, 79,  66,  66,  69,  70,  78,  72,  188,  74,  75,  76,  77,  78,  79, 
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
	
	i16MotorSpd = (abs)((RevData->u8SpeedFdbHigh << 8) | RevData->u8SpeedFdbLow) ;
//	i32LogWrite(DEBUG, LOG_USER, "Motor Rev Spd = %d\r\n", i16MotorSpd);
	__disable_irq();
	tmp = i16MotorSpd / sgUserInfo.u16RatioOfTransmission;
  gCanSendPdoInfo.CanSend260Info.u8Movespeed = tmp;
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
	static int16_t i32PropValueCmd = 0;
//	uint8_t u8Flag = 0;
		
	LastStatus.u8Data = SendData->buf[2];
	SendData->buf[2] = 0;

	if (1 == sgSwiInput.b1Walk_Upright)		/*add Main Connector*/
	{
		SendData->b1PowerLineOn = 0;
	}

	/*Move Mode*/
	SendData->b1ServoOn = 1;
	if(HandleInput.b1BackWard == 1)
	{
    SendData->b1ForwardReq = 1;
		gCanSendPdoInfo.CanSend260Info.u8MoveState &= ~FORWARD_MOVESTATE2HMI;
		gCanSendPdoInfo.CanSend260Info.u8MoveState &= ~PARK_MOVESTATE2HMI;
		gCanSendPdoInfo.CanSend260Info.u8MoveState |= REVERSE_MOVESTATE2HMI;
	}
	else if(HandleInput.b1ForWard == 1)
	{
		SendData->b1BackwardReq = 1;
		gCanSendPdoInfo.CanSend260Info.u8MoveState &= ~REVERSE_MOVESTATE2HMI;
		gCanSendPdoInfo.CanSend260Info.u8MoveState &= ~PARK_MOVESTATE2HMI;
		gCanSendPdoInfo.CanSend260Info.u8MoveState |= FORWARD_MOVESTATE2HMI;
	}
	else
	{
		gCanSendPdoInfo.CanSend260Info.u8MoveState &= ~REVERSE_MOVESTATE2HMI;
		gCanSendPdoInfo.CanSend260Info.u8MoveState &= ~FORWARD_MOVESTATE2HMI;
		gCanSendPdoInfo.CanSend260Info.u8MoveState |= PARK_MOVESTATE2HMI;
//		Noflag = 0;
	}
	
	SendData->u8TargetHigh = u16MotorVal >> 8;
	SendData->u8TargetLow = u16MotorVal;
	
	if (1 == sgSwiInput.b1Ems)  
	{
		SendData->b1EmsReq = 1;
	}
	/*Lift Mode*/
	if(0 != i32PropValue)
	{
		if(i32PropValueCmd < (i32PropValue - i32PropValueRate))
		{
			i32PropValueCmd +=  i32PropValueRate;
		}
		else
		{
			i32PropValueCmd = i32PropValue;
		}
		vPropSetTarget(LIFTDOWN_VALVE, (i32PropValueCmd + _IQ((sgUserInfo.fPropMinCurrent) / PROPD_STD_CURRENT)));
	}
	else
	{
		i32PropValueCmd = 0;
		vPropSetTarget(LIFTDOWN_VALVE, 0);
	}
	
	if((HANDLE_FLAG == 1)||(MOVE_FLAG == 1)||(LIFT_FLAG == 1)||(sgSwiInput.b1LiftUp == 1))
	{
		SendData->u8PumpTarget = u8PumpOrPropValue;
		SendData->b1LiftReq = 1;
	}
	
	if(0 != sgSwiInput.b1noAct)
	{
		SendData->u8TargetHigh = 0;
		SendData->u8TargetLow = 0;
	}
	
	if((0 == sgSwiInput.b1SafeLock)&&(0 == sgSwiInput.b1Walk_Upright))
	{
		SendData->b1ServoOn = 0;
	}

	if(receivedId112Message == 1)
	{
		SendData->b1ServoOn = 0;
	}
	
	if(0 != sgValvesInfo.u8NoAct)
	{
		 SendData->b1ForwardReq = 0;
		SendData->b1BackwardReq = 0;
		SendData->u8TargetHigh =0;
		SendData->u8TargetLow = 0;
	}
	
	if(1 == sgSwiInput.b1ChargeLimitAct)
	{
		SendData->u8PumpTarget = 0;
		SendData->b1LiftReq = 0;
		SendData->b1ServoOn = 0;
	}
	
	{
		static uint32_t u32Cnt = 0;
		u32Cnt++;
		{			
			/*lilu 20230819 For Test*/
			{
				i32SetPara(PARA_ForwardValveCurrent, DRIVER_FLAG);		/*Send Motor Value*/
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
				i32SetPara(PARA_PcuKeyInfo,HandleInput.u16data);						/*ErrCode*/
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
		case REVERSEA_VALVE:
			i32ErrCodeSet(REVERSEA_VALVE_ERR);
			/*add ErrCode*/
			break;
		case REVERSEB_VALVE:
			i32ErrCodeSet(REVERSEB_VALVE_ERR);
			/*add ErrCode*/
			break;
		case FB_MOVE_VALVE:
			i32ErrCodeSet(FB_MOVE_VALVE_ERR);
			/*add ErrCode*/
			break;
		case FB_TILT_VALVE:
			i32ErrCodeSet(FB_TILT_VALVE_ERR);
			/*add ErrCode*/
			break;
		case LR_MOVE_VALVE:
			i32ErrCodeSet(LR_MOVE_VALVE_ERR);
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
/************ 手柄报文 **********/	
//	if(0 != memcmp((char*)CanRev22CInfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo22C.u8Data, sizeof(CanRev22CInfoLast)))
	{
		memcpy((char*)CanRev22CInfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo22C.u8Data, sizeof(CanRev22CInfoLast));
		u16MotorVal = ((CanRev22CInfoLast.b1TargetHigh << 8)|(CanRev22CInfoLast.b1TargetLow));
		HandleInput.u16data = CanRev22CInfoLast.u8Data[0];
	}
	/************  锂电协议*******************/	
	{
		memcpy((char*)CanRev444InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo444.u8Data, sizeof(CanRev444InfoLast));
		if(CanRev444InfoLast.BMS_SOC != 0)
		{
			gCanSendPdoInfo.CanSend260Info.u8Soc = CanRev444InfoLast.BMS_SOC;
		}
		else
		{
			gCanSendPdoInfo.CanSend260Info.u8Soc = u8GetBatterySoc();
		}
	}
/***** 转向360 *******/
	{
		memcpy((char*)CanRev360InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo360.u8Data, sizeof(CanRev360InfoLast));
		/*添加相关操作*/
		i16SteerAngle = ((CanRev360InfoLast.i16SteerAngleH<<8)|(CanRev360InfoLast.i16SteerAngleL));
		if(0 != CanRev360InfoLast.u8ErrSteer)
		{
			gCanSendPdoInfo.CanSend260Info.u8ErrorSteer = CanRev360InfoLast.u8ErrSteer;
		}
	}
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
	
/************* LIFT DOWN****************/
	i32PropValue = 0;
		if((0 == LIFT_FLAG)&&(0 == HANDLE_FLAG)&&(0 == MOVE_FLAG))
		{
			i32AdcValue = i32LocalAiGetValue(LIFT_THROTTLE);
			if((i32AdcValue < 2200)&&(i32AdcValue > sgUserInfo.u16ThrottleMin))
			{
				LR_FLAG = 1;
				i32DoPwmSet(LIFTUP_VALVE,DRIVER_CLOSE);
				i32PropValue = _IQ((sgUserInfo.fPropMinCurrent + (sgUserInfo.fPropMaxCurrent - sgUserInfo.fPropMinCurrent) * u8PumpOrPropValue / PUMP_RANGE) / PROPD_STD_CURRENT);
				sgSwiInput.b1LiftDown = 1;
				sgSwiInput.b1LiftUp = 0;
				u8PumpOrPropValue = (255*(2200-i32AdcValue))/(2200-sgUserInfo.u16ThrottleMin);
			}
			else if(((i32AdcValue > 2800) && (i32AdcValue < sgUserInfo.u16ThrottleMax))
						&&((0 == i32LocalDiGet(HEIGHT_LIMIT_SWI))||(0 == i32LocalDiGet(FENCE2_SWI)))
						&&(0 == sgSwiInput.b1LiftLimit)
						&&(0 == (sgValvesInfo.b4NoLiftUp & BMS_NOLIFTUP)))
			{
				LR_FLAG = 1;
				i32DoPwmSet(LIFTUP_VALVE,DRIVER_OPEN);
				vPropSetTarget(LIFTDOWN_VALVE, 0);
				sgSwiInput.b1LiftDown = 0;
				sgSwiInput.b1LiftUp = 1;
				u8PumpOrPropValue = (255*(i32AdcValue-2800))/(sgUserInfo.u16ThrottleMax-2800);
			}
			else if((i32AdcValue > sgUserInfo.u16ThrottleMax)
						&&((0 == i32LocalDiGet(HEIGHT_LIMIT_SWI))||(0 == i32LocalDiGet(FENCE2_SWI)))
						&&(0 == sgSwiInput.b1LiftLimit)
						&&(0 == (sgValvesInfo.b4NoLiftUp & BMS_NOLIFTUP)))
			{
				LR_FLAG = 1;
				i32DoPwmSet(LIFTUP_VALVE,DRIVER_OPEN);
				vPropSetTarget(LIFTDOWN_VALVE, 0);
				sgSwiInput.b1LiftDown = 0;
				sgSwiInput.b1LiftUp = 1;
				u8PumpOrPropValue = 255;
			}
			else
			{
				LR_FLAG = 0;
				i32DoPwmSet(LIFTUP_VALVE,DRIVER_CLOSE);
				vPropSetTarget(LIFTDOWN_VALVE, 0);
				sgSwiInput.b1LiftDown = 0;
				sgSwiInput.b1LiftUp = 0;
				u8PumpOrPropValue = 0;
			}
		}
		else
		{
			LR_FLAG = 0;
			i32DoPwmSet(LIFTUP_VALVE,DRIVER_CLOSE);
			vPropSetTarget(LIFTDOWN_VALVE, 0);
			sgSwiInput.b1LiftDown = 0;
			sgSwiInput.b1LiftUp = 0;
			u8PumpOrPropValue = 0;
		}
	
	
/******************* 前后移调速 ****************/
	
	if((0 == LIFT_FLAG)&&(0 == HANDLE_FLAG)&&(0 == LR_FLAG))
	{
		i32AdcValue = i32LocalAiGetValue(MOVE_THROTTLE);
		if((i32AdcValue < sgUserInfo.u16LiftUpMin)&&(i32AdcValue > 100))
		{
			MOVE_FLAG = 1;
			i32DoPwmSet(REVERSEB_VALVE,DRIVER_OPEN);
			i32DoPwmSet(FB_MOVE_VALVE,DRIVER_OPEN);
			i32DoPwmSet(REVERSEA_VALVE,DRIVER_CLOSE);
			u8PumpOrPropValue = 255;
		}
		else if((i32AdcValue < 2200)&&(i32AdcValue > sgUserInfo.u16LiftUpMin))
		{
			MOVE_FLAG = 1;
			i32DoPwmSet(REVERSEB_VALVE,DRIVER_OPEN);
			i32DoPwmSet(FB_MOVE_VALVE,DRIVER_OPEN);
			i32DoPwmSet(REVERSEA_VALVE,DRIVER_CLOSE);
			u8PumpOrPropValue = (255*(2200-i32AdcValue))/(2200-sgUserInfo.u16LiftUpMin);
		}
		else if((i32AdcValue > 2800) && (i32AdcValue < sgUserInfo.u16LiftUpMax)&&(0 == LIFT_FLAG)&&(0 == HANDLE_FLAG))
		{
			MOVE_FLAG = 1;
			i32DoPwmSet(REVERSEA_VALVE,DRIVER_OPEN);
			i32DoPwmSet(FB_MOVE_VALVE,DRIVER_OPEN);
			i32DoPwmSet(REVERSEB_VALVE,DRIVER_CLOSE);
			u8PumpOrPropValue = (255*(i32AdcValue-2800))/(sgUserInfo.u16LiftUpMax-2800);
		}
		else if((i32AdcValue > sgUserInfo.u16LiftUpMax)&&(0 == LIFT_FLAG)&&(0 == HANDLE_FLAG))
		{
			MOVE_FLAG = 1;
			i32DoPwmSet(REVERSEA_VALVE,DRIVER_OPEN);
			i32DoPwmSet(FB_MOVE_VALVE,DRIVER_OPEN);
			i32DoPwmSet(REVERSEB_VALVE,DRIVER_CLOSE);
			u8PumpOrPropValue = 255;
		}
		else
		{
			MOVE_FLAG = 0;
			i32DoPwmSet(FB_MOVE_VALVE,DRIVER_CLOSE);
			i32DoPwmSet(REVERSEA_VALVE,DRIVER_CLOSE);
			i32DoPwmSet(REVERSEB_VALVE,DRIVER_CLOSE);
			u8PumpOrPropValue = 0;
		}
	}
		
/************** LEFT RIGHT MOVE SPEED *************/        //转向发修改
	
	if((0 == MOVE_FLAG)&&(0 == HANDLE_FLAG)&&(0 == LR_FLAG))
	{
		i32AdcValue = i32LocalAiGetValue(LR_MOVE_THROTTLE);
		if((i32AdcValue < sgUserInfo.u16LiftDownMin)&&(i32AdcValue > 100))
		{
			LIFT_FLAG = 1;
			i32DoPwmSet(REVERSEB_VALVE,DRIVER_CLOSE);
			i32DoPwmSet(LR_MOVE_VALVE,DRIVER_OPEN);
			i32DoPwmSet(REVERSEA_VALVE,DRIVER_OPEN);
			u8PumpOrPropValue = 255;
		}
		else if((i32AdcValue < 2200)&&(i32AdcValue > sgUserInfo.u16LiftDownMin))
		{
			LIFT_FLAG = 1;
			i32DoPwmSet(REVERSEB_VALVE,DRIVER_CLOSE);
			i32DoPwmSet(LR_MOVE_VALVE,DRIVER_OPEN);
			i32DoPwmSet(REVERSEA_VALVE,DRIVER_OPEN);
			u8PumpOrPropValue = (255*(2200-i32AdcValue))/(2200-sgUserInfo.u16LiftDownMin);
		}
		else if((i32AdcValue > 2800) && (i32AdcValue < sgUserInfo.u16LiftDownMax)&&(0 == MOVE_FLAG)&&(0 == HANDLE_FLAG))
		{
			LIFT_FLAG = 1;
			i32DoPwmSet(REVERSEA_VALVE,DRIVER_CLOSE);
			i32DoPwmSet(LR_MOVE_VALVE,DRIVER_OPEN);
			i32DoPwmSet(REVERSEB_VALVE,DRIVER_OPEN);
			u8PumpOrPropValue = (255*(i32AdcValue-2800))/(sgUserInfo.u16LiftDownMax-2800);
		}
		else if((i32AdcValue > sgUserInfo.u16LiftDownMax)&&(0 == MOVE_FLAG)&&(0 == HANDLE_FLAG))
		{
			LIFT_FLAG = 1;
			i32DoPwmSet(REVERSEA_VALVE,DRIVER_CLOSE);
			i32DoPwmSet(LR_MOVE_VALVE,DRIVER_OPEN);
			i32DoPwmSet(REVERSEB_VALVE,DRIVER_OPEN);
			u8PumpOrPropValue = 255;
		}
		else
		{
			LIFT_FLAG = 0;
			i32DoPwmSet(LR_MOVE_VALVE,DRIVER_CLOSE);
			i32DoPwmSet(REVERSEA_VALVE,DRIVER_CLOSE);
			i32DoPwmSet(REVERSEB_VALVE,DRIVER_CLOSE);
			u8PumpOrPropValue = 0;
		}
	}
	
/******************手柄报文控制前后倾*******************************/
	
	if((0 == MOVE_FLAG)&&(0 == LIFT_FLAG)&&(0 == LR_FLAG))
	{
		if((1 == gCanRevPdoInfo.CanRevInfo22C.b1Forerake))         //前倾
		{
			HANDLE_FLAG = 1;
			i32DoPwmSet(REVERSEB_VALVE,DRIVER_CLOSE);
			i32DoPwmSet(REVERSEA_VALVE,DRIVER_OPEN);
			i32DoPwmSet(FB_TILT_VALVE,DRIVER_OPEN);
			u8PumpOrPropValue = 125;
		}
		else if((1 == gCanRevPdoInfo.CanRevInfo22C.b1Hypsokinesis)&&(0 == MOVE_FLAG)&&(0 == LIFT_FLAG))      //后倾
		{
			HANDLE_FLAG = 1;
			i32DoPwmSet(REVERSEB_VALVE,DRIVER_OPEN);
			i32DoPwmSet(REVERSEA_VALVE,DRIVER_CLOSE);
			i32DoPwmSet(FB_TILT_VALVE,DRIVER_OPEN);
			u8PumpOrPropValue = 125;
		}
		else
		{
			HANDLE_FLAG = 0;
			i32DoPwmSet(REVERSEB_VALVE,DRIVER_CLOSE);
			i32DoPwmSet(REVERSEA_VALVE,DRIVER_CLOSE);
			i32DoPwmSet(FB_TILT_VALVE,DRIVER_CLOSE);
			u8PumpOrPropValue = 0;
		}
	}
}


/*开关量监控*/
static void vSwiMonitor(void)
{
	xSwiInput SwiInput = {0};
	static uint16_t Snail_Count = 0;
	static uint8_t SpeedMode = 0;
	static uint8_t ReturnFlg = 0;
	static uint8_t KeyDelay = 0;
	
	if(1 == i32LocalDiGet(HEIGHT_SPEEDLIMIT_SWI))
	{
		sgSwiInput.b1HeightSpdlimit = 1;
		SwiInput.b1HeightSpdlimit = 1;
	}
	else
	{
		sgSwiInput.b1HeightSpdlimit = 0;
	}
	
	if(1 == i32LocalDiGet(CHARGE_SWI))
	{
		gCanSendPdoInfo.CanSend260Info.u8MoveState |= (0x01<<7);
		sgSwiInput.b1ChargeLimitAct = 1;
	}
	else
	{
		if(0 == receivedId112Message)
		{
			gCanSendPdoInfo.CanSend260Info.u8MoveState &= ~(0x01<<7);
		}
		sgSwiInput.b1ChargeLimitAct = 0;
	}
	

	if(1 == i32LocalDiGet(FENCE2_SWI))
	{
		sgSwiInput.b1Fence2 = 1;
		SwiInput.b1Fence2 = 1;
	}
	else
	{
		sgSwiInput.b1Fence2 = 0;
	}
	
	if(1 == i32LocalDiGet(SAFELOCK_SWI))
	{
		sgSwiInput.b1SafeLock = 1;
		SwiInput.b1SafeLock = 1;
	}
	else
	{
		sgSwiInput.b1SafeLock = 0;
	}
	
	if(1 == i32LocalDiGet(PEDAL_SWI))
	{
		sgSwiInput.b1Pedal = 1;
		SwiInput.b1Pedal = 1;
	}
	else
	{
		sgSwiInput.b1Pedal = 0;
	}

	if(1 == i32LocalDiGet(HEIGHT_LIMIT_SWI))
	{
		sgSwiInput.b1HeightLimit = 1;
		SwiInput.b1HeightLimit = 1;
	}
	else
	{
		sgSwiInput.b1HeightLimit = 0;
	}
	
	if(0 == i32LocalDiGet(LIFT_LIMT_SWI))   //常闭开关
	{
		sgSwiInput.b1LiftLimit = 1;
		SwiInput.b1LiftLimit = 1;
	}
	else
	{
		sgSwiInput.b1LiftLimit = 0;
	}
	
	if(1 == i32LocalDiGet(STEER_SAFE_LOCK))
	{
		sgSwiInput.b1SteerSafeLock = 1;
		SwiInput.b1SteerSafeLock = 1;
	}
	else
	{
		sgSwiInput.b1SteerSafeLock = 0;
	}
	
	{
		if((0 != Rev_Speed)&&(0 == SpeedMode))
		{
			ReturnFlg = 1;
			if(1 == i32LocalDiGet(KEY_RETURN_SWI))
			{
				sgValvesInfo.u8NoAct |= (1 << 1);
				i32ErrCodeSet(113);
			}
		}
		else if((1 == i32LocalDiGet(KEY_RETURN_SWI))&&(0 == ReturnFlg))
		{
			SpeedMode = 1;
			gCanSendPdoInfo.CanSend260Info.u8MoveState |= (0x01<<3) ;//发送互锁
		}
	}
	
	{
		if((1 == SpeedMode)&&(0 == i32LocalDiGet(KEY_RETURN_SWI))&&(0 != Rev_Speed))
		{
			sgValvesInfo.u8NoAct |= (1 << 1);
			i32ErrCodeSet(113);
		}
		
		if((0 == i32LocalDiGet(KEY_RETURN_SWI))&&(0 == u16MotorVal))
		{
			if(KeyDelay < 200)
				KeyDelay++;
			if(KeyDelay > 10)
			{
				SpeedMode = 0;
				ReturnFlg = 0;
				sgValvesInfo.u8NoAct &= ~(1 << 1);
				i32ErrCodeClr(113);
				gCanSendPdoInfo.CanSend260Info.u8MoveState &= ~(0x01<<3) ;//发送互锁
			}
		}
	}
	
//	if(0x01 == ((HandleInput.u16data >> 4)&0x01))    //急反
	if(1 == HandleInput.b1Ems)
	{
		sgSwiInput.b1Ems = 1;
		SwiInput.b1Ems = 1;
	}
	else
	{
		sgSwiInput.b1Ems = 0;
	}
	
if(1 == gCanRevPdoInfo.CanRevInfo22C.b1SnailRequest)
	{
		SnailRequest_Flag = 1;
	}
	else
	{
		SnailRequest_Flag = 0;
	}
	sgSwiInput.b1Slow_Mode = SnailRequest_Flag;
	
	
	
	if(1 == gCanRevPdoInfo.CanRevInfo22C.b1SnailRequest)
	{
		if((Snail_Count > 200)&&(0 == sgSwiInput.b1SafeLock))
		{
			//Snail_Count = 0;
			sgSwiInput.b1Walk_Upright = 1;
		}
		else
		{
			Snail_Count++;
		}
	}
	else
	{
		Snail_Count = 0;
		sgSwiInput.b1Walk_Upright = 0;
	}
	/*Speed limit*/
	sgSwiInput.b1noAct = 0;
	if((1 == i32LocalDiGet(PEDAL_SWI))&&(1 == i32LocalDiGet(FENCE2_SWI)))
	{
		sgValvesInfo.b1Gear1SpdFlag = 0;
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 0;
		sgValvesInfo.b1Gear4SpdFlag = 0;
	}
	else if((1 == i32LocalDiGet(PEDAL_SWI))&&(0 == i32LocalDiGet(FENCE2_SWI)))
	{
		sgValvesInfo.b1Gear1SpdFlag = 1;
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 0;
		sgValvesInfo.b1Gear4SpdFlag = 0;
	}
	else if((0 == i32LocalDiGet(PEDAL_SWI)&&(1 == i32LocalDiGet(FENCE2_SWI))))
	{
		sgSwiInput.b1noAct = 1;	
	}
	else
	{
		
	}

	if((1 == i32LocalDiGet(HEIGHT_SPEEDLIMIT_SWI))||(0 == i32LocalDiGet(PEDAL_SWI)&&(0 == i32LocalDiGet(FENCE2_SWI))))
	{
		sgValvesInfo.b1Gear1SpdFlag = 0;
		sgValvesInfo.b1Gear2SpdFlag = 1;
		sgValvesInfo.b1Gear3SpdFlag = 0;
		sgValvesInfo.b1Gear4SpdFlag = 0;
	}
	if((1 == sgSwiInput.b1Slow_Mode)||(1 == sgSwiInput.b1Walk_Upright))
	{
		sgValvesInfo.b1Gear1SpdFlag = 0;
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 0;
		sgValvesInfo.b1Gear4SpdFlag = 1;
	}
	
	if(gCanSendPdoInfo.CanSend260Info.u8Soc <= 15)
	{
		sgValvesInfo.b1Gear1SpdFlag = 0;
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 1;
		sgValvesInfo.b1Gear4SpdFlag = 0;
	}
	else if(gCanSendPdoInfo.CanSend260Info.u8Soc <= 10)
	{
		sgValvesInfo.b4NoLiftUp |= BMS_NOLIFTUP;
		sgValvesInfo.b1Gear1SpdFlag = 0;
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 1;
		sgValvesInfo.b1Gear4SpdFlag = 0;
	}
}

static uint8_t u8SwiInitChcek(void)
{
	uint8_t u8Res = 0;
		
	if((1 == gCanRevPdoInfo.CanRevInfo22C.b1EmergencyReverse) 
		|| (1 == i32LocalDiGet(SAFELOCK_SWI))
		||(1 == i32LocalDiGet(KEY_RETURN_SWI)) 
			) 
	{
		u8Res = 1;
	}
	return u8Res;		
}


/*************************充电状态设置*****************************/
static void vCanId112Proc(tCanFrame * CanFrame)
{	
	receivedId112Message = 1;
	gCanSendPdoInfo.CanSend260Info.u8MoveState |= (0x01<<7);

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
	
	xRevCallBackProc CanId112 = {.u32CanId = 0x112, .u32Data = 0, .CallBack = vCanId112Proc};
	vCanRevMsgRegister(&CanId112);
	
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
	
	i32PropValueRate = _IQ((i32GetPara(PARA_PropDMaxCurrent0) - i32GetPara(PARA_PropDMinCurrent0)) / 1000.0 / PROPD_STD_CURRENT)/i32GetPara(PARA_AngleValue0);
	
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
		sgSwiInput.b1HeightLimit = sgSaveState.b1Above1M8;
		sgSwiInput.b1HeightSpdlimit = sgSaveState.b1HeightSpdLimit;
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

	if(true == u8GetNetTimerOverFlag(TIMER_EcuPowerOn))
	{
		u8EcuProcFlag = 1;
		vKillNetTimer(TIMER_EcuPowerOn);
	}
	
	if ((false == u8GetNetTimerOverFlag(TIMER_EcuPowerOn)) && (true == u8GetNetTimerStartFlag(TIMER_EcuPowerOn)))
	{
		if (1 == u8SwiInitChcek())
		{
			sgValvesInfo.u8NoAct |= (1 << 0);
			i32ErrCodeSet(ACT_INIT_ERR);
		}
	}
	
	if(0 == u8SwiInitChcek())
	{
		sgValvesInfo.u8NoAct &= ~(1 << 0);
		i32ErrCodeClr(ACT_INIT_ERR);
	}
	
	if(1 == u8EcuProcFlag)
	{
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
			gCanSendPdoInfo.CanSend260Info.u8HourCountH = (u32HourCount >> 8) & 0xFF;
			__enable_irq();
		}
		
		if(0 != sgUserInfo.b1StartUpLock)
		{
			u32HourCount = 0;
			vHourCountWrite(0);
			__disable_irq();
			gCanSendPdoInfo.CanSend260Info.u8HourCountL = 0;
			gCanSendPdoInfo.CanSend260Info.u8HourCountH = 0;
			__enable_irq();
		}
		
			
		if(sgSwiInput.b1Slow_Mode == 1)
		{
			gCanSendPdoInfo.CanSend260Info.u8MoveState |= (0x01<<2) ;//发送龟速模式
		}
		else if(sgSwiInput.b1Slow_Mode != 1)
		{
			gCanSendPdoInfo.CanSend260Info.u8MoveState &= ~(0x01<<2) ;
		}
	
		if(1 == sgSwiInput.b1Walk_Upright)
		{
			gCanSendPdoInfo.CanSend260Info.u8MoveState |= (0x01<<4) ;//发送直立行走标志
		}
		else if(0 == sgSwiInput.b1Walk_Upright )
		{
			gCanSendPdoInfo.CanSend260Info.u8MoveState &= ~(0x01<<4) ;
		}
		
//		if (sgSwiInput.b1SafeLock==1)
//		{
//				gCanSendPdoInfo.CanSend260Info.u8MoveState |= (0x01<<3) ;//发送互锁
//		}
//		else if(sgSwiInput.b1SafeLock!= 1)
//		{
//			gCanSendPdoInfo.CanSend260Info.u8MoveState &= ~(0x01<<3) ;
//		}
		
		
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
