/*******************************************************************************
* Filename: UserStackerTruckProc.c	                                           *
* Description: 逻辑层的源文件，包含对应的回调函数	   						   *
* Author:                                                      *
* Date: 2023/05/30    														   *
* Revision:	 														 		   *
*******************************************************************************/
#include "UserCanComm.h"
#include "UserStackerTruckProcSZL.h"
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

#if (USER_TYPE == USER_JIALI_SanZhiLiu)

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
		{.b1Flag = 0, .b11CanRevId = 0x360},
		{.b1Flag = 1, .b11CanRevId = 0x3D2},
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
#define	NoAct_Init			  1
#define	NoAct_Lock			  2
#define	NoAct_Security		3	

#define NoLiftUp_HeightLimit	0		
#define NoLiftUp_FaultLock		1
#define	NoLiftUp_Bms			    2

#define	NoMove_Pedal			0
#define	NoMove_FaultLock	1
#define	NoMove_Ems				2

typedef union
{
	uint16_t u16data;
	struct
	{
		uint16_t Hand_SWI: 1;         
		uint16_t HeightLimit_SWI: 1;                 
		uint16_t HandleUp_SWI: 1;
		uint16_t HandleDown_SWI: 1; 		
		uint16_t ForWard_SWI: 1;                  
		uint16_t BackWard_SWI:1;            
		uint16_t SnailRequest: 1;             
		uint16_t LIFTUP_SWI: 1;          
		uint16_t LIFTDOWN_SWI: 1;                          
		uint16_t SpeedLimit1_SWI:1;       
		uint16_t Left_SWI: 1;                          
		uint16_t LIFTDOWNEN_SWI:1;       
		uint16_t DianCiRelease_SWI:1;
		uint16_t b3Reserve:3;
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
		uint8_t b1OverLeanFlag: 1;
		
		uint8_t b1ForWardStat: 1;
		uint8_t b1BackWardStat: 1;
		uint8_t b1LiftUpStat: 1;
		uint8_t b1LeanForWardStat: 1;
		uint8_t b1LeanBackWardStat: 1;
		uint8_t b1LiftDownStat: 1;
		uint8_t	b1PumpMotorNoAct: 1;
		uint8_t b1BrakeFlag: 1;
		
		uint8_t	b4Gear1Spd: 4;
		uint8_t	b4Gear2Spd: 4;
		uint8_t	b4Gear3Spd: 4;
		uint8_t	b4Gear4Spd: 4;
		
		uint8_t u8NoAct;
		
		uint8_t b4NoLiftUp: 4;
		uint8_t b4NoMove: 4;
		
		uint8_t b1RightReq:1;
		uint8_t b1LeftReq:1;
		uint8_t b1Driver3State:1;
		uint8_t u8Reserve:5;
		int16_t i16LeanAngleValue;        //倾斜角
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
	uint16_t	u16ThrottleDeadzoon;
	uint16_t	u16ThrottleMidValue;
	
	uint16_t	u16LiftUpMin;
	uint16_t	u16LiftUpMax;
	uint16_t	u16LiftUpRange;
	uint16_t	u16LiftUpMid;
	uint16_t	u16LiftUpMidValue;

	uint16_t	u16LeanMin;
	uint16_t 	u16LeanMax;
//	uint16_t	u16LeanRange;
//	uint16_t	u16LeanDeadzoon;
	uint16_t	u16LeanMidValue;
	
	uint16_t u16AngleValueMin;
	uint16_t u16AngleValueMax;
	uint16_t u16AngleValueMidValue;
	uint16_t u16AngleValueDeadzoon;
	uint16_t u16AngleValueRange;
	
	uint8_t		u8LeanBackWardValue;
	uint8_t		u8LeanForWardValue;
	uint16_t  u16LeanMaxValue;
	xSteerAngleDecSpd SteerAngleDecSpd;

	uint8_t		b1LiftMode: 1;
	uint8_t		b1RentalStop: 1;
	uint8_t		b1RentalStart: 1;
	uint8_t		b1RentalMode: 1;
	uint8_t		b1PasswordFunc: 1;
	uint8_t   b1LeanSensorEn:1;
	uint8_t		b3Reserve: 2;	
	
	uint8_t		u8BatteryType;
	uint8_t   u8TurnDecSpd;
	uint8_t		b1HourConutMode: 1;		/*0: 上电计时， 1：互锁计时*/
	uint8_t		b1StartUpLock: 1;		/*0： 检测， 1：不检测*/
	uint8_t		b1LiftLock: 1;			/*0： 需要互锁， 1：不需要*/
	uint8_t		b1LiftPedal: 1;			/*0： 无要求， 1：踏板open =1， close =0 / open =0， close =1 才可以动作*/
	uint8_t		b1MoveLiftMode: 1;		/*0： 同时动作， 1：互斥*/
	uint8_t   b1BatteyCount:1;    //记电量开启关闭
	uint8_t		b3Reserve1: 2;
	
	uint8_t   u8LiftUpSpd;
	uint8_t   u8LiftDownSpd;
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
static uint8_t u8MotorVal = 0;
static uint8_t u8Spd = 0;
static uint8_t u8AngleVal = 0;
static uint8_t u8SocVal = 0;
static uint8_t	u8PumpOrPropValue = 0;
static uint16_t u16CanId5ACPeriod = 0;
static uint16_t u16CanRev62CCnt = 0;
static xValvesInfo sgValvesInfo;
static xUserInfo sgUserInfo;
static xSwiInput sgSwiInput;
static uint32_t u32HourCount = 0;
static int16_t i16SteerAngle = 0;
static uint16_t DRIVER_FLAG = 0;
static uint16_t u8LiftUpVal = 0;
static uint16_t Rev_Speed;

/*PDO接收待完善*/
xCanRevPdoInfo gCanRevPdoInfo; 
static xCanRev360Info CanRev360InfoLast;
static xCanRev3D2Info CanRev3D2InfoLast;
/*PDO发送待完善*/
xCanSendPdoInfo gCanSendPdoInfo;


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
	int16_t  u16Spd;
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
	sgValvesInfo.b1BrakeFlag = RevData->b1Ebrake;      //接收MCU端制动信息 0 制动 1释放
	sgValvesInfo.b1Driver3State =RevData->b1Driver3State;// Do2  Do3电磁阀制动 
	__disable_irq();
	tmp = abs(i16MotorSpd / sgUserInfo.u16RatioOfTransmission);
	gCanSendPdoInfo.CanSend260Info.u8Movespeed = tmp;
	gCanSendPdoInfo.CanSend260Info.u8ErrorMove = RevData->u8ErrCode;
	__enable_irq();
	
	tmp = RevData->u8MotorTmp;
	/*Motor Temp*/
	
	tmp = RevData->u8BoardTmp;
	/**/

	
	__disable_irq();
//	gCanSendPdoInfo.CanSend43CInfo.i16MotorCurrent = (RevData->u8CurrentHigh << 8) | RevData->u8CurrentLow;
	__enable_irq();
}

static void vMoveModeNoChangeProc(uint8_t *u8Spd, const xValvesInfo *pValvesInfo)
{
	uint8_t Tmp = 0;
	if ((1 == pValvesInfo->b1ForWardStat) || (1 == pValvesInfo->b1BackWardStat))
	{
		if (*u8Spd > u8MotorVal)
		{
			Tmp = sgUserInfo.fMoveSpdPer5msDecStep * (0.01 + 0.99 * *u8Spd / MOTOR_SPEED_RANGE);
			if ((((*u8Spd - u8MotorVal) > Tmp) && (0 != sgUserInfo.fMoveSpdPer5msDecStep)))
			{
				*u8Spd -= Tmp;
			}
			else
			{
				*u8Spd = u8MotorVal;
			}
		}
		else if (*u8Spd < u8MotorVal)
		{
			Tmp = sgUserInfo.fMoveSpdPer5msAccStep * (0.01 + 0.99 * *u8Spd / MOTOR_SPEED_RANGE) + 1;
			if (((u8MotorVal - *u8Spd) > Tmp) && (0 != sgUserInfo.fMoveSpdPer5msAccStep))
			{
				*u8Spd += Tmp;
			}
			else
			{
				*u8Spd = u8MotorVal;
			}
		}
	}
	else
	{
		*u8Spd = 0;
	}
}

/*发送给Mst的回调函数*/
static void vMstSendProc(xMstSendPara *SendData) 
{
	static xValvesInfo sgLastValvesInfo;
	static uint8_t u8MoveSwitchFlag = 0;
	static uint8_t u8MainConnectCnt = 0;
	uint8_t fTmp;
	static uint8_t BeepCount = 0;
	static uint8_t BeepFlag = 0;
	xMstSendStat LastStatus;
	int32_t pop = 0;
	static uint16_t liftup = 0;
	static uint16_t HandleCount = 0;	
	LastStatus.u8Data = SendData->buf[2];
	SendData->buf[2] = 0;
	SendData->b1ServoOn = 1;
	static uint8_t Lock_Count;
	static uint8_t BrakeFlag = 0 ;
	
	/*Move Mode*/
	if(1 == sgValvesInfo.b1RightReq)
	{
		SendData->b1RightReq = 1;
		SendData->b1LeftReq = 0;
	}
	else if(1 == sgValvesInfo.b1LeftReq)
	{
		SendData->b1RightReq = 0;
		SendData->b1LeftReq = 1;
	}
	else
	{
		SendData->b1RightReq = 0;
		SendData->b1LeftReq = 0;
	}
	if((0 != sgValvesInfo.b1RightReq || 0 != sgValvesInfo.b1LeftReq) && (sgSwiInput.Hand_SWI != 0))
	{
		SendData->u8TargetHigh = (u8AngleVal * sgUserInfo.u8TurnDecSpd)/ MOTOR_SPEED_RANGE ;
	}
	else
	{
		SendData->u8TargetHigh = 0;
	}
	if ((sgLastValvesInfo.b1ForWardStat != sgValvesInfo.b1ForWardStat) || 
		(sgLastValvesInfo.b1BackWardStat != sgValvesInfo.b1BackWardStat))
	{
		if ((0 == u8Spd) ||
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
					uint8_t uTmp = 0;
					if (0 != sgUserInfo.u16MotorMaxSpd)
					{
						uTmp = abs(i16MotorSpd) * MOTOR_SPEED_RANGE / sgUserInfo.u16MotorMaxSpd;	/*CloseLoop */
					}
					else
					{
						uTmp = abs(i16MotorSpd) * MOTOR_SPEED_RANGE / MOTOR_MAX_SPEED;	/*CloseLoop */
					}
					if (uTmp < u8Spd)
					{                                                                                
						u8Spd = uTmp;
					}
					u8MoveSwitchFlag = 1;
				}
				fTmp = sgUserInfo.fMoveSpdPer5msDecStep * (0.01 + 0.99 * u8Spd / MOTOR_SPEED_RANGE) + 1 ;
				if ((abs(i16MotorSpd) > MOTOR_MIN_SPEED) && (u8Spd > (fTmp + 1)))
				{
					u8Spd -= fTmp;
				}
				else
				{
					u8Spd = 0;
				}
			}
		}
	}
	else
	{
		vMoveModeNoChangeProc(&u8Spd, &sgLastValvesInfo);
		u8MoveSwitchFlag = 0;
	}
	if(sgLastValvesInfo.b1ForWardStat == 1 && sgSwiInput.Hand_SWI == 1)
	{
		 SendData->b1ForwardReq = 1;
	}
	else if(sgLastValvesInfo.b1BackWardStat == 1 && sgSwiInput.Hand_SWI == 1)
	{
		SendData->b1BackwardReq = 1;
	}
	else
	{
		SendData->b1ForwardReq = 0;
		SendData->b1BackwardReq = 0 ;
	}
  SendData->u8TargetLow = u8Spd;
	
	
	/*Lift Mode*/
	if(((1 == sgSwiInput.LIFTUP_SWI) && (sgSwiInput.LIFTDOWNEN_SWI == 1 ))||((sgSwiInput.HandleUp_SWI == 1 && sgSwiInput.Hand_SWI != 0)))
	{
		if(u8LiftUpVal == 0)
			liftup = (255 * sgUserInfo.u8LiftUpSpd) / 100;
		else
			liftup = u8LiftUpVal;
		SendData->u8PumpTarget = liftup;          // 起升命令速度
		SendData->b1LiftReq = 1;                    // 起升命令
		vPropSetTarget(LIFTDOWN_VALVE, 0);             // 下降比例
	}
	else if(((sgSwiInput.LIFTDOWN_SWI == 1)&&(sgSwiInput.LIFTDOWNEN_SWI == 1))||((sgSwiInput.HandleDown_SWI == 1) && (sgSwiInput.Hand_SWI != 0)))
	{
		if(u8LiftUpVal == 0)
			liftup = (255 * sgUserInfo.u8LiftDownSpd) / 100;  
		else
			liftup = u8LiftUpVal;
		pop = _IQ((sgUserInfo.fPropMinCurrent + (sgUserInfo.fPropMaxCurrent - sgUserInfo.fPropMinCurrent) * liftup / PUMP_RANGE) / PROPD_STD_CURRENT);
		SendData->b1DownReq = 1;         // 可要可不要
		vPropSetTarget(LIFTDOWN_VALVE, pop);
				
		if(BeepFlag == 0)
		{
			if(BeepCount >160)
			{
				 BeepCount = 0;
			}
			else if(BeepCount > 80)
			{
				i32DoPwmSet(DownBeep_SWI,DRIVER_OPEN);
			}
			else if(BeepCount >0)
			{
				 i32DoPwmSet(DownBeep_SWI,DRIVER_CLOSE);
			}
			BeepCount++;
		}
	}
	else 
	{
		SendData->b1LiftReq = 0;
		SendData->b1DownReq = 0;
		vPropSetTarget(LIFTDOWN_VALVE, 0);
		SendData->u8PumpTarget = 0;
		i32DoPwmSet(DownBeep_SWI,DRIVER_CLOSE);
	}
	
	if(u8SocVal <= 15)
	{
		SendData->b1BackwardReq = 0;
		SendData->b1ForwardReq = 0; 		
		SendData->b1RightReq = 0;
		SendData->b1LiftReq = 0;
		SendData->u8TargetHigh = 0;
		SendData->u8TargetLow = 0;
	}
	if(sgSwiInput.HeightLimit_SWI == 1)  //到达高度限位禁止起升
	{
		SendData->b1LiftReq  = 0;
	}
	if(sgValvesInfo.b1OverLeanFlag == 1) //过倾限制
	{
		SendData->b1BackwardReq = 0;
		SendData->b1ForwardReq = 0;
		SendData->u8PumpTarget = 0;
		SendData->u8TargetHigh = 0;
		SendData->u8TargetLow = 0;
		SendData->b1LeftReq = 0;
		SendData->b1RightReq = 0;
		SendData->b1LiftReq = 0;
		
		BeepFlag|= 1<<0;
		if(BeepCount >160)
		{
			 BeepCount = 0;
		}
		else if(BeepCount > 80)
		{
			i32DoPwmSet(DownBeep_SWI,DRIVER_OPEN);
		}
		else if(BeepCount >0)
		{
			 i32DoPwmSet(DownBeep_SWI,DRIVER_CLOSE);
		}
		BeepCount++;
	}
	else
		BeepFlag &= ~(1<<0);
 //电磁释放
  if(sgSwiInput.DianCiRelease_SWI == 1)
	{ 
		BeepFlag |= 1<<1;
		if(BeepFlag <= 2)
		{
			if(BeepCount >160)
			{
				 BeepCount = 0;
			}
			else if(BeepCount > 80)
			{
				i32DoPwmSet(DownBeep_SWI,DRIVER_OPEN);
			}
			else if(BeepCount >0)
			{
				 i32DoPwmSet(DownBeep_SWI,DRIVER_CLOSE);
			}
			BeepCount++;
		}

		if((BrakeFlag != 2)&&(sgValvesInfo.b1BrakeFlag == 0 && sgValvesInfo.b1Driver3State == 0)||BrakeFlag == 1)
		{ 
			SendData->b1BrakeReq = 1; //请求释放电磁制动
			SendData->b1LiftReq = 0;
			SendData->b1DownReq = 0;
			SendData->b1ForwardReq = 0;
			SendData->b1BackwardReq = 0;
			SendData->b1LeftReq = 0;
			SendData->b1RightReq = 0;
			SendData->u8TargetHigh = 0;
			SendData->u8TargetLow = 0;
			vPropSetTarget(LIFTDOWN_VALVE, 0);
			SendData->u8PumpTarget = 0; 
			BrakeFlag  = 1;
		}
		else     //在电磁制动释放过程中按下此按键会强制进入锁死状态 复位该按键以解除                           
		{
			SendData->b1ServoOn = 0;
			SendData->b1LiftReq = 0;
			SendData->b1DownReq = 0;
			SendData->b1ForwardReq = 0;
			SendData->b1BackwardReq = 0;
			SendData->b1LeftReq = 0;
			SendData->b1RightReq = 0;
			SendData->u8TargetHigh = 0;
			SendData->u8TargetLow = 0;
			vPropSetTarget(LIFTDOWN_VALVE, 0);
			SendData->u8PumpTarget = 0;
			BrakeFlag = 2;
		}
	}
	else
	{
		BeepFlag &= ~(1<<1);
		BrakeFlag = 0;
	}
		static uint32_t u32Cnt = 0;
		u32Cnt++;
		{			
			/*lilu 20230819 For Test*/
			{
				i32SetPara(PARA_ForwardValveCurrent, SendData->u8TargetLow);		/*Send Motor Value*/
				i32SetPara(PARA_BackValveCurrent, u8Spd);		/*Rev Motor Value*/
				i32SetPara(PARA_PropValveCurrent, SendData->b1RightReq);	/*Prop Current*/
			  i32SetPara(PARA_LiftValveCurrent,sgValvesInfo.i16LeanAngleValue);		/*Send Pump Value*/
				i32SetPara(PARA_OnOffValveCurrent, SendData->buf[2]);			/*Send Status*/
				i32SetPara(PARA_LoadRate, sgValvesInfo.u8Data[0]);				/*Valve NoFlag*/
				i32SetPara(PARA_CalibrationStatus, sgValvesInfo.u8Data[1]);		/*Value State*/
				i32SetPara(PARA_TurnRightValveCurrent, sgValvesInfo.b1BrakeFlag);
				i32SetPara(PARA_TurnLeftValveCurrent, sgValvesInfo.b1Driver3State);
				i32SetPara(PARA_AngleValue, (uint16_t)i32LocalAiGetValue(AI_B_AI1_R));		/*AI1*/
				i32SetPara(PARA_PressureVlaue1,(uint16_t)i32LocalAiGetValue(AI_B_AI2_R));	/*AI2*/
				i32SetPara(PARA_PressureVlaue2, (uint16_t)i32LocalAiGetValue(AI_B_AI3_R));	/*AI3*/
				i32SetPara(PARA_ExtSignal, sgSwiInput.u16data);						/*Swi*/
				i32SetPara(PARA_PcuKeyInfo,sgValvesInfo.u8Data[0]);						/*ErrCode*/
				i32SetPara(PARA_BmsSoc, u8LiftUpVal);		/*BMS SOC*/
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
//		case LIFTDOWN_VALVE:
//			i32ErrCodeSet(LIFTDOWN_VALVE_ERR);
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


/*接收PDO处理函数*/
static void vCanRevPdoProc(void)
{
	static uint16_t u16CanRev1ACCnt;
	
	tCanFrame CanSendFrame;
	tCanFrame CanSend0={0};
	uint8_t i = 0;
	static uint16_t cnt = 0;
	int16_t i16RealLeanAngle;
/***** 转向360 *******/
	{
		memcpy((char*)CanRev360InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo360.u8Data, sizeof(CanRev360InfoLast));
		/*添加相关操作*/
		i16SteerAngle = ((CanRev360InfoLast.i16SteerAngleH<<8)|(CanRev360InfoLast.i16SteerAngleL));
	  gCanSendPdoInfo.CanSend260Info.u8ErrorSteer = CanRev360InfoLast.u8ErrSteer;
	}
/********* 倾角传感器3D2  ********/
	{
//		memcpy((char*)CanRev3D2InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo3D2.u8Data, sizeof(CanRev3D2InfoLast));
//	 //添加sgValueInfo.u8LeanAngleValue值
//		 if(CanRev3D2InfoLast.u8FlagParameter == 1)
//		 {
//			sgValvesInfo.i16LeanAngleValue =(uint16_t)((CanRev3D2InfoLast.u8GPA1ValueHigh<<8)+CanRev3D2InfoLast.u8GPA1ValueLow);
//			i16RealLeanAngle = sgValvesInfo.i16LeanAngleValue - LEANSENSOR_MID;
//			if(abs(i16RealLeanAngle) > sgUserInfo.u16LeanMaxValue)
//				sgValvesInfo.b1OverLeanFlag = 1;
//			else
//				sgValvesInfo.b1OverLeanFlag = 0 ;
//		
//		 }
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
	u8SocVal = u8GetBatterySoc(); //监控电源电量，低于20进入三档速度 低于15禁止行走
/*********** Motor Speed *************/
	/*add Turn Dec Spd*/
	
/************   前进      ******************/
	{	
		i32AdcValue = i32LocalAiGetValue(MOVE_THROTTLE);
		if((i32AdcValue > (sgUserInfo.u16ThrottleMidValue + sgUserInfo.u16ThrottleDeadzoon))&&(i32AdcValue <= sgUserInfo.u16ThrottleMax))
		{
			u8MotorVal = (((i32AdcValue - (sgUserInfo.u16ThrottleMidValue + sgUserInfo.u16ThrottleDeadzoon))*250)/sgUserInfo.u16ThrottleRange);
			sgValvesInfo.b1ForWardStat = 0;
			sgValvesInfo.b1BackWardStat = 1;
		}
		else if((i32AdcValue < (sgUserInfo.u16ThrottleMidValue - sgUserInfo.u16ThrottleDeadzoon))&&(i32AdcValue+20 >= sgUserInfo.u16ThrottleMin))
		{
			u8MotorVal = ((((sgUserInfo.u16ThrottleMidValue - sgUserInfo.u16ThrottleDeadzoon) - i32AdcValue )*250)/sgUserInfo.u16ThrottleRange);
			sgValvesInfo.b1ForWardStat = 1;
			sgValvesInfo.b1BackWardStat = 0;
		}
		else
		{
			sgValvesInfo.b1BackWardStat = 0;
			sgValvesInfo.b1ForWardStat = 0;
			u8MotorVal = 0;
		}
	}
	u8MotorVal = u16SteerAngleDecSpd(u8MotorVal, abs(i16SteerAngle), &sgUserInfo.SteerAngleDecSpd);
	/************   转向    ******************/
	{
		i32AdcValue = i32LocalAiGetValue(STREE_THROTTLE);
		if((i32AdcValue > (sgUserInfo.u16AngleValueMidValue + sgUserInfo.u16AngleValueDeadzoon))&&(i32AdcValue <= sgUserInfo.u16AngleValueMax))
		{
			sgValvesInfo.b1LeftReq = 0;
			sgValvesInfo.b1RightReq = 1;
			u8AngleVal = (((i32AdcValue - (sgUserInfo.u16AngleValueMidValue + sgUserInfo.u16AngleValueDeadzoon)) * 250)/sgUserInfo.u16AngleValueRange);
			
		}
		else if((i32AdcValue < (sgUserInfo.u16AngleValueMidValue - sgUserInfo.u16AngleValueDeadzoon))&&(i32AdcValue+20 >= sgUserInfo.u16AngleValueMin))
		{
			sgValvesInfo.b1LeftReq = 1;
			sgValvesInfo.b1RightReq = 0;
			u8AngleVal = ((( (sgUserInfo.u16AngleValueMidValue - sgUserInfo.u16AngleValueDeadzoon)- i32AdcValue ) * 250)/sgUserInfo.u16AngleValueRange);
		}
		else
		{                                             
			sgValvesInfo.b1LeftReq = 0;
			sgValvesInfo.b1RightReq = 0;
			u8AngleVal = 0;
		}
	}	
	/************   倾角    ******************/
//{
		i32AdcValue = i32LocalAiGetValue(TILE_THROTTLE);
	if(sgUserInfo.b1LeanSensorEn == 1)
	{
		sgValvesInfo.i16LeanAngleValue = i32AdcValue - sgUserInfo.u16LeanMidValue;
		if(abs(sgValvesInfo.i16LeanAngleValue) > sgUserInfo.u16LeanMaxValue)
				sgValvesInfo.b1OverLeanFlag = 1;
		else
				sgValvesInfo.b1OverLeanFlag = 0 ;
	}
//	}
	//1.8M高度限速使用四档速度
	if(1 == sgSwiInput.SpeedLimit1_SWI)
	{
		sgValvesInfo.b1Gear1SpdFlag = 0 ;
		sgValvesInfo.b1Gear2SpdFlag = 0 ;
		sgValvesInfo.b1Gear3SpdFlag = 0 ;
    sgValvesInfo.b1Gear4SpdFlag = 1 ;		
	}
	//龟速使用三档速度 
	else if(1 == sgSwiInput.SnailRequest||(u8SocVal <= 20))
	{
	  sgValvesInfo.b1Gear1SpdFlag = 0 ;
		sgValvesInfo.b1Gear2SpdFlag = 0 ;
		sgValvesInfo.b1Gear3SpdFlag = 1 ;
    sgValvesInfo.b1Gear4SpdFlag = 0 ;	
	}
	/*默认使用一档速度    */
	else
	{
		sgValvesInfo.b1Gear1SpdFlag = 1 ;
		sgValvesInfo.b1Gear2SpdFlag = 0 ;
		sgValvesInfo.b1Gear3SpdFlag = 0 ;
    sgValvesInfo.b1Gear4SpdFlag = 0 ;	
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
	u8MotorVal = ((u8MotorVal * u8SpeedRate)/100);
}

/*开关量监控*/
static void vSwiMonitor(void)
{
	xSwiInput SwiInput = {0};
	static uint8_t Snail_Count = 0;
	static uint8_t Lock_Count = 0;
	if(1 == i32LocalDiGet(Hand_SWI))
	{
		SwiInput.Hand_SWI = 1;
	}
	
	if(1 == i32LocalDiGet(HandleUp_SWI))
	{
		SwiInput.HandleUp_SWI = 1;
	}
	if(1 == i32LocalDiGet(HandleDown_SWI))
	{
		SwiInput.HandleDown_SWI = 1;
	}
	if(1 == i32LocalDiGet(HeightLimit_SWI)) //高度限位常开
	{
		SwiInput.HeightLimit_SWI = 1;
	}
	if(0 == i32LocalDiGet(SnailRequest))
	{
		SwiInput.SnailRequest = 1;
	}
	
	if(1 == i32LocalDiGet(LIFTUP_SWI))
	{
		SwiInput.LIFTUP_SWI = 1;
	}
	
	if(1 == i32LocalDiGet(LIFTDOWN_SWI))
	{
		SwiInput.LIFTDOWN_SWI = 1;
	}
	
	if(1 == i32LocalDiGet(SpeedLimit1_SWI))
	{
		SwiInput.SpeedLimit1_SWI = 1;
	}	
	if(1 == i32LocalDiGet(LIFTDOWNEN_SWI))
	{
		SwiInput.LIFTDOWNEN_SWI = 1;
	}
	
	if(1 == i32LocalDiGet(DianCiRelease_SWI))
	{
		SwiInput.DianCiRelease_SWI = 1;
		
	}
	
	
	sgSwiInput.u16data = SwiInput.u16data;

}
#if 0
static uint8_t u8SwiInitChcek(void)
{
	uint8_t u8Res = 0;
		
	if((0 == gCanRevPdoInfo.CanRevInfo22C.b1EmergencyReverse) 
		|| (1 == i32LocalDiGet(SAFELOCK_SWI)) 
			) 
	{
		u8Res = 1;
	}
	return u8Res;		
}
#endif
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
	
	sgUserInfo.u8LiftUpSpd = i32GetPara(PARA_LiftSpeed);
	sgUserInfo.u8LiftDownSpd = i32GetPara(PARA_LowerSpeed);
	
	sgUserInfo.b1LiftMode = i32GetPara(LIFT_MODE_PARA);						/*起升模式*/
	sgUserInfo.b1LeanSensorEn = i32GetPara(PARA_AngleSimulationLimit);     //60号参数开启或关闭倾角传感器功能
	
	sgUserInfo.u16Gear1Spd = i32GetPara(PARA_Gear1Spd);	/*1档速度*/
	sgUserInfo.u16Gear2Spd = i32GetPara(PARA_Gear2Spd);	/*2档速度*/
	sgUserInfo.u16Gear3Spd = i32GetPara(PARA_Gear3Spd);	/*3档速度*/
	sgUserInfo.u16Gear4Spd = i32GetPara(PARA_Gear4Spd);	/*4档速度*/
	
	/*          行走模拟量          */
	sgUserInfo.u16ThrottleMin = i32GetPara(MOVE_THROTTLE_MIN) * 100;		/*油门最小最, uint 0.1V*/
	sgUserInfo.u16ThrottleMax = i32GetPara(MOVE_THROTTLE_MAX) * 100;		/*油门最大值,uint 0.1V*/
	sgUserInfo.u16ThrottleMidValue = (sgUserInfo.u16ThrottleMax + sgUserInfo.u16ThrottleMin) >> 1;
	sgUserInfo.u16ThrottleDeadzoon = (sgUserInfo.u16ThrottleMax -sgUserInfo.u16ThrottleMidValue)*i32GetPara(MOVE_THROTTLE_MID)/100;         //手柄调整死区值
	sgUserInfo.u16ThrottleRange = ((sgUserInfo.u16ThrottleMax - sgUserInfo.u16ThrottleMin) >> 1) - sgUserInfo.u16ThrottleDeadzoon;
	
	/*           转弯模拟量           */
	sgUserInfo.u16AngleValueMin = i32GetPara(LIFT_UP_THROTTLE_MIN) * 100;
	sgUserInfo.u16AngleValueMax = i32GetPara(LIFT_UP_THROTTLE_MAX) * 100;
	sgUserInfo.u16AngleValueMidValue = (sgUserInfo.u16AngleValueMax + sgUserInfo.u16AngleValueMin) >> 1;
	sgUserInfo.u16AngleValueDeadzoon = (sgUserInfo.u16AngleValueMax -sgUserInfo.u16AngleValueMidValue)*i32GetPara(LIFT_UP_THROTTLE_MID)/100;
	sgUserInfo.u16AngleValueRange =  ((sgUserInfo.u16AngleValueMax - sgUserInfo.u16AngleValueMin) >> 1) - sgUserInfo.u16AngleValueDeadzoon ;
	
	sgUserInfo.u8TurnDecSpd = i32GetPara(TURN_DEC_SPD);
	/*           倾角模拟量           */
	sgUserInfo.u16LeanMin = i32GetPara(PARA_BrakeBDeadZoneMinVal) * 100;      //155
	sgUserInfo.u16LeanMax = i32GetPara(PARA_BrakeBDeadZoneMaxVal) * 100;      //156
  sgUserInfo.u16LeanMidValue = (sgUserInfo.u16LeanMax + sgUserInfo.u16LeanMin) >> 1;

	
	sgUserInfo.SteerAngleDecSpd.u16StartAngle = i32GetPara(TURN_START_ANGLE) * 10;
	sgUserInfo.SteerAngleDecSpd.u16EndAngle = i32GetPara(TURN_END_ANGLE) * 10;
	sgUserInfo.SteerAngleDecSpd.u16StartAngleSpd = i32GetPara(START_ANGLE_SPD) * MOTOR_SPEED_RANGE / 100;
	sgUserInfo.SteerAngleDecSpd.u16EndAngleSpd = i32GetPara(END_ANGLE_SPD) * MOTOR_SPEED_RANGE / 100;
	sgUserInfo.SteerAngleDecSpd.fAgnleDecSpdFactor = (sgUserInfo.SteerAngleDecSpd.u16StartAngleSpd - sgUserInfo.SteerAngleDecSpd.u16EndAngleSpd) /   \
													 (sgUserInfo.SteerAngleDecSpd.u16EndAngle - sgUserInfo.SteerAngleDecSpd.u16StartAngle);
	sgUserInfo.u16LeanMaxValue = i32GetPara(MAX_LEAN_ANGLE);												 
	sgUserInfo.u16RatioOfTransmission = i32GetPara(PARA_RatioOfTransmission);
	sgUserInfo.u16MotorMaxSpd = i32GetPara(PARA_MotorMaxSpd);
	
	sgUserInfo.u8BatteryType = i32GetPara(PARA_BatteryType);
	sgUserInfo.b1BatteyCount = i32GetPara(PARA_LowBatAlmFunc);
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
//		sgFlagVal.Height_SpeedLimit = sgSaveState.b1HeightSpdLimit;        //高度限速保存到eeprom
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
		sgUserInfo.fMoveSpdPer5msAccStep = STEP_FACTOR * i32GetPara(PARA_CurveFastDrive) / i32GetPara(PARA_AccAndDecFastDrive);//  #33
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
	u8SocDisableFlag = sgUserInfo.b1BatteyCount;
	if(true == u8GetNetTimerOverFlag(TIMER_EcuPowerOn))
	{
		u8EcuProcFlag = 1;
		vKillNetTimer(TIMER_EcuPowerOn);
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
			gCanSendPdoInfo.CanSend260Info.u8ErrorMove = u8ErrSwitchArray[u8ErrCode];
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
			if (1 == sgSwiInput.Hand_SWI)
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
