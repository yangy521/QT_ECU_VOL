/*******************************************************************************
* Filename: UserStackerTruckProc.c	                                           *
* Description: �߼����Դ�ļ���������Ӧ�Ļص�����	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/05/30    														   *
* Revision:	 														 		   *
*******************************************************************************/
#include "UserCanComm.h"
#include "UserStackerTruckProc_Ruyi.h"
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

#if (USER_TYPE == USER_RUYI_DGC)


const static xPdoParameter  sgPdoPara = 
{
	.TpdoPara = {
		{.u8Flag = 0, .u8Type = 0x0, .u16Period = 20},
		{.u8Flag = 0, .u8Type = 0x0, .u16Period = 200},
		{.u8Flag = 0, .u8Type = 0x0, .u16Period = 200},
		{.u8Flag = 0, .u8Type = 0x0, .u16Period = 200},
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 20, .u16CanId = 0x260 },
		
		
	},
	.RpdoPara = {
		{.b1Flag = 1, .b11CanRevId = 0x190},
		{.b1Flag = 1, .b11CanRevId = 0x200},
		{.b1Flag = 1, .b11CanRevId = 0x360},
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
#define NoAct_McuErr		4

#define NoLiftUp_HeightLimit	0		
#define NoLiftUp_FaultLock		1
#define	NoLiftUp_LowBat			2
#define NoLiftUp_Init				3

#define	NoMove_LowBat			0
#define	NoMove_FaultLock		1
#define	NoMove_Ems				2
#define NoMove_Init				3

typedef union
{
	uint16_t u16data;
	struct
	{
		uint16_t b1UpLimit: 1;
		uint16_t b1Ems: 1;
		uint16_t b1SafeLock: 1;
		uint16_t b1Forward: 1;
		uint16_t b1Backward: 1;
		uint16_t b1LiftUp: 1;
		uint16_t b1LiftDown: 1;
		uint16_t b1LegUp: 1;
		uint16_t b1LegDown: 1;
		uint16_t b1SteerLock: 1;
		uint16_t b1SlowRequest: 1;
		uint16_t b1OutriggerUp: 1;
		uint16_t b1OutriggerDown: 1;
		uint16_t b1HeightSpdLimit:1;
		uint16_t b1EmergencyStop:1;
		uint16_t b3Reserve: 1;
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
		uint8_t b1Gear4SpdFlag: 1;		/*bms �͵���*/
		uint8_t b1Reserve1:1;
		
		uint8_t b1ForWardStat: 1;
		uint8_t b1BackWardStat: 1;
		uint8_t b1LiftUpStat: 1;
		uint8_t b1LiftDownStat: 1;
		uint8_t b1LegUpStat: 1;
		uint8_t b1LegDownStat: 1;
		uint8_t b1OutriggerUpStat:1;
		uint8_t b1OutriggerDownStat: 1;
		
		uint8_t b1ZhiLiForwardStat:1;
		uint8_t b1ZhiliBackwardState:1;	
		uint8_t	b1PumpMotorNoAct: 1;
		uint8_t b4Reserve2 :5;
		
		uint8_t	b4Gear1Spd: 4;
		uint8_t	b4Gear2Spd: 4;
		uint8_t	b4Gear3Spd: 4;
		uint8_t	b4Gear4Spd: 4;
		
		uint8_t u8NoAct;
		
		uint8_t b4NoLiftUp: 4;
		uint8_t b4NoMove: 4;
		
		uint8_t u8Reserve4;

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
	uint8_t 	b1HourCntClr:1;
	uint8_t		b2Reserve: 2;	
	
	uint8_t		u8BatteryType;
	
	uint8_t		b1HourConutMode: 1;		/*0: �ϵ��ʱ�� 1��������ʱ*/
	uint8_t		b1StartUpLock: 1;		/*0�� ��⣬ 1�������*/
	uint8_t		b1LiftLock: 1;			/*0�� ��Ҫ������ 1������Ҫ*/
	uint8_t		b1LiftPedal: 1;			/*0�� ��Ҫ�� 1��̤��open =1�� close =0 / open =0�� close =1 �ſ��Զ���*/
	uint8_t		b1MoveLiftMode: 1;		/*0�� ͬʱ������ 1������*/
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
static uint32_t u32HourCount = 0;
xCanRevPdoInfo gCanRevPdoInfo;	/*PDO���մ�����*/
static xValvesInfo ValvesInfoRecord;
static uint8_t u8OutriggerNoActFlag = 0;
	


/*PDO���ʹ�����*/
xCanSendPdoInfo gCanSendPdoInfo ;


const static uint8_t u8ErrSwitchArray[ErrCodeMax] = 
{	
/*  0    1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18   19*/
    0,   1,   2,   3,   4,   5,   6,   7,   8,   9,   10,  11,  12,  13,  14,  15,  16,  17,  18,  19,
    20,  21,  22,  23,  24,  25,  26,  27,  28,  29,  30,  31,  32,  33,  34,  35,  36,  37,  38,  39,
    40,  41,  42,  43,  44,  45,  46,  47,  48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,
		60,  61,  62,  63,  64,  65,  66,  67,  68,  69,  70,  71,  72,  73,  74,  75,  76,  77,  78,  79, 
		80,  81,  82,  83,  84,  85,  86,  87,  88,  89,  90,  91,  92,  93,  94,  95,  96,  97,  98,  99,
		100, 40, 	42,  41, 104, 105, 106, 	40, 40, 40, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 
		120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 
		140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159,
		160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 
		180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 
		200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 
		220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 
		240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254,
};

/*��ʼ�����*/
static uint8_t u8SwiInitChcek(void)
{
	uint8_t u8Res = 0;
	if ( (1 == i32LocalDiGet(FORWARD_SWI)) 
		|| (1 == i32LocalDiGet(BACKWARD_SWI)) 
		|| (1 == i32LocalDiGet(SLOW_SPEED_SWI))
		|| (1 == i32LocalDiGet(SAFELOCK_SWI))
		|| (1 == i32LocalDiGet(LIFT_UP_SWI)) 
		|| (1 == i32LocalDiGet(LIFT_DOWN_SWI)) 
		|| (1 == i32LocalDiGet(LEG_UP_SWI))
		|| (1 == i32LocalDiGet(LEG_DOWN_SWI))
		|| (1 == i32LocalDiGet(OUTRIGGER_UP_SWI))
		|| (1 == i32LocalDiGet(OUTRIGGER_DOWN_SWI))) 		
	{
		u8Res = 1;
	}
	return u8Res;		
}

/*���������*/
static void vSwiMonitor(void)
{
	xSwiInput SwiInput = {0};
	static uint16_t u16ZhiLicnt = 0;
	static uint8_t u8EmsCnt = 0;
	static uint8_t u8EstopFlag = 0;


	
	if(1 == i32LocalDiGet(HEIGHT_SPEEDLIMIT_SWI))
	{
		SwiInput.b1UpLimit = 1;
	}


	if(1 == i32LocalDiGet(SAFELOCK_SWI))
	{
		SwiInput.b1SafeLock = 1;
	}
	
	if(1 == i32LocalDiGet(FORWARD_SWI))
	{
		SwiInput.b1Forward = 1;
	}
	
	if(1 == i32LocalDiGet(BACKWARD_SWI))
	{
		SwiInput.b1Backward = 1;
	}
	
	if(1 == i32LocalDiGet(LIFT_UP_SWI))
	{
		SwiInput.b1LiftUp = 1;
	}
	
	if(1 == i32LocalDiGet(LIFT_DOWN_SWI))
	{
		SwiInput.b1LiftDown = 1;
	}
	
	if(1 == i32LocalDiGet(LEG_UP_SWI))
	{
		SwiInput.b1LegUp = 1;
	}
	
	if(1 == i32LocalDiGet(LEG_DOWN_SWI))
	{
		SwiInput.b1LegDown = 1;
	}
	
	if(1 == i32LocalDiGet(STEER_SAFELOCK_SWI))
	{
		SwiInput.b1SteerLock = 1;
	}
	
	if(1 == i32LocalDiGet(SLOW_SPEED_SWI))
	{
		SwiInput.b1SlowRequest = 1;
	}
	
	if(1 == i32LocalDiGet(OUTRIGGER_UP_SWI))
	{
		SwiInput.b1OutriggerUp = 1;
	}
	
	if(1 == i32LocalDiGet(OUTRIGGER_DOWN_SWI))
	{
		SwiInput.b1OutriggerDown = 1;
	}
	

	/*add lock */
	
	//����������ѡ
	
//	if ((1 == SwiInput.b1Forward) || (1 == SwiInput.b1Backward) ||
//	   (1 == SwiInput.b1LeanForward) || (1 == SwiInput.b1LeanBackward) ||
//	   (1 == SwiInput.b1LiftUp) || (1 == SwiInput.b1LiftDown))
//	{
//		/*��ֹ������ǰ������*/
//		if ((0 != gCanRevPdoInfo.CanRevInfo2.u8ErrSteer) || (0 == SwiInput.b1FaultLockOut))
//		{
//			sgValvesInfo.b4NoLiftUp |= 1 << NoLiftUp_FaultLock;
//			sgValvesInfo.b4NoMove |= 1 << NoMove_FaultLock;
//			i32ErrCodeSet(ACT_FAULT_LOCK_ERR);
//		}
//	}
	if(1 ==SwiInput.b1SlowRequest)
	{
		uint16_t u16zhili_time = 0 ;
		uint8_t u8nolimitflag= 0;
		u16zhili_time = i32GetPara(PARA_AccAndDecAntiPinch);
		
		if(u16zhili_time<6)
		{
			u16zhili_time = 6000;
		}
		else if(u16zhili_time>20)
		{
			u8nolimitflag = 1;
		}
		else
		{
			u16zhili_time = u16zhili_time * 1000;
		
		}
		sgValvesInfo.b4Gear3Spd |=  1<<Gear3_Spd_Con1 ;
		if( 0 == SwiInput.b1SafeLock)
		{
			if( u16ZhiLicnt < 200/USER_ECU_PERIOD)
			{
				u16ZhiLicnt ++;
			}
			else if((u16ZhiLicnt < u16zhili_time/USER_ECU_PERIOD)||(u8nolimitflag))//����ֱ������״̬
			{
				u16ZhiLicnt ++;
				sgValvesInfo.b4Gear4Spd |= 1<<Gear4_Spd_Con1;
				if(0 == sgValvesInfo.b1MoveFlag)
				{
					if(1 == SwiInput.b1Forward)
					{
						sgValvesInfo.b1ZhiliBackwardState = 1;
					}
					else if(1 == SwiInput.b1Backward)
					{
						sgValvesInfo.b1ZhiLiForwardStat = 1;
					}
				}
			}
			else//�˳�
			{
				sgValvesInfo.b1ZhiliBackwardState = 0;
				sgValvesInfo.b1ZhiLiForwardStat = 0;
				sgValvesInfo.b4Gear4Spd &= ~(1<<Gear4_Spd_Con1);
			}
		}
		else
		{
				sgValvesInfo.b1ZhiliBackwardState = 0;
				sgValvesInfo.b1ZhiLiForwardStat = 0;
				sgValvesInfo.b4Gear4Spd &= ~(1<<Gear4_Spd_Con1);
		}
	}
	else
	{
		sgValvesInfo.b1ZhiliBackwardState = 0;
		sgValvesInfo.b1ZhiLiForwardStat = 0;
		u16ZhiLicnt = 0;
		sgValvesInfo.b4Gear3Spd &= ~ ( 1<<Gear3_Spd_Con1 );
		sgValvesInfo.b4Gear4Spd &= ~(1<<Gear4_Spd_Con1);
	}

	if ((0 == sgSwiInput.b1SafeLock)
			&&(0 == sgValvesInfo.b1ZhiliBackwardState)
			&&(0 == sgValvesInfo.b1ZhiLiForwardStat))
	{//δ���»������Ҳ���ֱ������״̬ 64
		if ((1 == SwiInput.b1Forward) || (1 == SwiInput.b1Backward) ||
				(((1 == SwiInput.b1LegUp) || (1 == SwiInput.b1LegDown) ||
		     (1 == SwiInput.b1LiftUp) || (1 == SwiInput.b1LiftDown)||
		(1 ==SwiInput.b1OutriggerDown)||(1 == SwiInput.b1OutriggerUp))))
		{
			if(1 == SwiInput.b1SafeLock)
			{
				i32ErrCodeSet(ACT_LOCK_ERR);			
			}
			sgValvesInfo.u8NoAct |= 1 << NoAct_Lock;
		}
	}
	if((1 == sgValvesInfo.b1ZhiliBackwardState)&&(1 == SwiInput.b1SafeLock))
	{
		i32ErrCodeSet(ACT_LOCK_ERR);	
		sgValvesInfo.u8NoAct |= 1 << NoAct_Lock;
	}

//	if ((0 == SwiInput.b1Forward) && (0 == SwiInput.b1Backward) && (1 == SwiInput.b1Ems))
//	{//ֻ����������66
//		sgValvesInfo.b4NoMove |= 1 << NoMove_Ems;
////		i32ErrCodeSet(MOVE_EMS_ERR);
//	}
//	else if ((0 == SwiInput.b1Forward) && (0 == SwiInput.b1Backward) && (0 == SwiInput.b1Ems))
//	{
//		sgValvesInfo.b4NoMove &= ~(1 << NoMove_Ems);
////		i32ErrCodeClr(MOVE_EMS_ERR);
//	} 
	if(i32ErrCodeCheck(EMS_LOCK_ERR))
	{
		sgValvesInfo.b4NoMove |= 1 << NoMove_Ems;
	}
	
	if ((0 == SwiInput.b1Forward) && (0 == SwiInput.b1Backward) &&
	    (0 == SwiInput.b1LegUp) && (0 == SwiInput.b1LegDown) &&
	    (0 == SwiInput.b1LiftUp) && (0 == SwiInput.b1LiftDown) && 
	    (0 == SwiInput.b1Ems) && (0 == SwiInput.b1OutriggerUp) && 
			(0 == SwiInput.b1OutriggerDown)&&(0 == SwiInput.b1SlowRequest))
	{
		if(0 == SwiInput.b1SafeLock)
		{
			
			sgValvesInfo.u8NoAct &= ~(1 << NoAct_Init);
			i32ErrCodeClr(ACT_INIT_ERR);
			
		}
		sgValvesInfo.b4NoMove &= ~(1 << NoMove_Ems);
		
		sgValvesInfo.u8NoAct &= ~(1 << NoAct_Lock);
		sgValvesInfo.b4NoLiftUp &= ~(1 << NoLiftUp_FaultLock);
		sgValvesInfo.b4NoMove &= ~(1 << NoMove_FaultLock);
//		i32ErrCodeClr(ACT_FAULT_LOCK_ERR);
		i32ErrCodeClr(ACT_LOCK_ERR);
		i32ErrCodeClr(EMS_LOCK_ERR);
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
	
	/*����߼�*/
	if(1 == SwiInput.b1EmergencyStop)
	{
		sgValvesInfo.b4NoMove |= (1 << NoMove_FaultLock);
	}
	
	if (0 == sgValvesInfo.b1NoActFlag)
	{
//		if (sgSwiInput.u16data != SwiInput.u16data)
//		{
			if ((0 == sgSwiInput.b1UpLimit) && (1 == SwiInput.b1UpLimit) && (1 == sgValvesInfo.b1LiftUpStat))
			{
				sgValvesInfo.b4Gear2Spd = 1 << Gear2_Spd_Con1;
				sgSwiInput.b1HeightSpdLimit = 1;
				sgSaveState.b1HeightSpdLimit = 1;
				u16EepromWrite(PARA_SaveState, sgSaveState.u16Data, 1);	
			}
			else if ((0 == sgSwiInput.b1UpLimit) && (1 == SwiInput.b1UpLimit) && (1 == sgValvesInfo.b1LiftDownStat))
			{
				sgValvesInfo.b4Gear2Spd &=  ~(1 << Gear2_Spd_Con1);
				sgSwiInput.b1HeightSpdLimit = 0;
				sgSaveState.b1HeightSpdLimit = 0;
				u16EepromWrite(PARA_SaveState, sgSaveState.u16Data, 1);	
			}
//			if ((1 == SwiInput.b1Meter1m8) && (1 == sgValvesInfo.b1LiftUpStat))
//			//if ((0 == sgSwiInput.b1Meter1m8) && (1 == SwiInput.b1Meter1m8) && (1 == sgValvesInfo.b1LiftUpStat))
//			{
////				sgValvesInfo.b4Gear2Spd = 1 << Gear2_Spd_Con2;
//				sgSwiInput.b1Above1M8 = 1;
//				sgSaveState.b1Above1M8 = sgSwiInput.b1Above1M8;
//				u16EepromWrite(PARA_SaveState, sgSaveState.u16Data, 1);	
//			}
//			else if ((1 == SwiInput.b1Meter1m8) && (1 == sgValvesInfo.b1LiftDownStat))
//			{
////				sgValvesInfo.b4Gear2Spd &=  ~(1 << Gear2_Spd_Con2);
//				sgSwiInput.b1Above1M8 = 0;
//				sgSaveState.b1Above1M8 = sgSwiInput.b1Above1M8;
//				u16EepromWrite(PARA_SaveState, sgSaveState.u16Data, 1);
//			}
			
//			SwiInput.b1Above1M8 = sgSwiInput.b1Above1M8;			/*lilu 20230801 Above1M8 retain*/
			SwiInput.b1HeightSpdLimit = sgSwiInput.b1HeightSpdLimit;
			sgSwiInput.u16data = SwiInput.u16data;   /**/
			

			if (1 == sgSwiInput.b1HeightSpdLimit)
			{
 				sgValvesInfo.b4Gear2Spd |= 1 << Gear2_Spd_Con1;
			}
			else
			{
				sgValvesInfo.b4Gear2Spd &=  ~(1 << Gear2_Spd_Con1);
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
			

			if ((1 == sgSwiInput.b1LegUp)&&(0 == sgValvesInfo.b1LiftUpFlag))
			{
				sgValvesInfo.b1LegUpStat = 1;
			}
			else
			{
				sgValvesInfo.b1LegUpStat = 0;
			}
			
			if (1 == sgSwiInput.b1LegDown)
			{
				sgValvesInfo.b1LegDownStat = 1;
			}
			else
			{
				sgValvesInfo.b1LegDownStat = 0;
			}
			
			if ((1 == sgSwiInput.b1OutriggerUp)
				&&(1 == SwiInput.b1SafeLock)
			  &&(0 == u8OutriggerNoActFlag))
			{
				sgValvesInfo.b1OutriggerUpStat = 1;
			}
			else
			{
				sgValvesInfo.b1OutriggerUpStat = 0;
			}
		
			if ((1 == sgSwiInput.b1OutriggerDown)
				&&(1 == SwiInput.b1SafeLock)
				&&(0 == u8OutriggerNoActFlag))
			{
				sgValvesInfo.b1OutriggerDownStat = 1;
			}
			else
			{
				sgValvesInfo.b1OutriggerDownStat = 0;
			}

			{
				//0907������
				uint8_t u8PumpTmp = 0;
				uint8_t u8PumpOld = 0;
				static uint8_t u8PumpOut = 0;
				static uint8_t u8PumpRecord = 0;//ʵ������߼�
				static uint8_t u8ErroDelayFlag = 0;
				static uint8_t PumpMotorNoAct= 0 ;
				static uint8_t u8PumpActDelay = 0;
				static uint8_t u8PumpActFlag = 0;

				
				
				u8PumpTmp = (sgValvesInfo.b1LiftUpStat<<5) | (sgValvesInfo.b1LiftDownStat<<4) |			
							(sgValvesInfo.b1LegUpStat << 3 )| (sgValvesInfo.b1LegDownStat <<2)|
							(	sgValvesInfo.b1OutriggerUpStat <<1) | sgValvesInfo.b1OutriggerDownStat;
				
				u8PumpOld = ( ValvesInfoRecord.b1LiftUpStat<<5) | (ValvesInfoRecord.b1LiftDownStat << 4)| 			
							(ValvesInfoRecord.b1LegUpStat <<3)| (ValvesInfoRecord.b1LegDownStat <<2)|
					(ValvesInfoRecord.b1OutriggerUpStat <<1)| ValvesInfoRecord.b1OutriggerDownStat;
				
				
				if ((0 == PumpMotorNoAct)//���Ƶ�һ���
						&&(( 0 !=(u8PumpTmp &(u8PumpTmp - 1)))))
				{
					sgValvesInfo.b1LiftUpStat = ValvesInfoRecord.b1LiftUpStat;
					sgValvesInfo.b1LiftDownStat = ValvesInfoRecord.b1LiftDownStat;
					sgValvesInfo.b1LegUpStat = ValvesInfoRecord.b1LegUpStat;
					sgValvesInfo.b1LegDownStat= ValvesInfoRecord.b1LegDownStat;
					sgValvesInfo.b1OutriggerUpStat = ValvesInfoRecord.b1OutriggerUpStat;
					sgValvesInfo.b1OutriggerDownStat = ValvesInfoRecord.b1OutriggerDownStat;						
					i32SetPara(PARA_OutCtrlInfo,11);
					u8ErroDelayFlag = 1;
				}

					u8PumpOut = (sgValvesInfo.b1LiftUpStat<<5) | (sgValvesInfo.b1LiftDownStat<<4) |			
							(sgValvesInfo.b1LegUpStat << 3 )| (sgValvesInfo.b1LegDownStat <<2)|
							(	sgValvesInfo.b1OutriggerUpStat <<1) | sgValvesInfo.b1OutriggerDownStat;
				
				if((1 == u8ErroDelayFlag)&&(0 !=(u8PumpOut^u8PumpRecord)))//�����������һ�������һ��
				{
					PumpMotorNoAct = 1;
					i32ErrCodeSet(MUTL_PUMP_REQ_ERR);
				}
				u8PumpRecord = u8PumpOut;				
				
				if(1 == PumpMotorNoAct)
				{
					sgValvesInfo.b1LiftUpStat = 0;
					sgValvesInfo.b1LiftDownStat = 0;
					sgValvesInfo.b1LegUpStat = 0;
					sgValvesInfo.b1LegDownStat= 0;
					sgValvesInfo.b1OutriggerUpStat = 0;
					sgValvesInfo.b1OutriggerDownStat = 0;	
				}
				
				{ //11.20
					static uint8_t u8PumpDelayRec = 0;
					static uint16_t u16PumpDelayCnt = 0;
					static uint8_t u8PumpDelayFlag =0;
					uint8_t u8PumpActTmp = 0;
					u8PumpActTmp = (sgValvesInfo.b1LiftUpStat<<5) | (sgValvesInfo.b1LiftDownStat<<4) |			
							(sgValvesInfo.b1LegUpStat << 3 )| (sgValvesInfo.b1LegDownStat <<2)|
							(	sgValvesInfo.b1OutriggerUpStat <<1) | sgValvesInfo.b1OutriggerDownStat;
					if((0 == u8PumpActTmp)&&(0 != u8PumpDelayRec))//����ʷ�������������붯��Ϊ0��
					{
						u8PumpDelayFlag =1;
					}
					if((1 == u8PumpDelayFlag)&&(u16PumpDelayCnt < (1000 / USER_ECU_PERIOD)))//�б�־λ����ʼ��ʱ
					{
						u16PumpDelayCnt++;
					}
					else//��ʱ��������ձ�־λ
					{
						u16PumpDelayCnt = 0;
						u8PumpDelayFlag = 0;
					}
					u8PumpDelayRec = u8PumpActTmp;
					if(1 == u8PumpDelayFlag)
					{
						sgValvesInfo.b1LiftUpStat = 0;
						sgValvesInfo.b1LiftDownStat = 0;
						sgValvesInfo.b1LegUpStat = 0;
						sgValvesInfo.b1LegDownStat= 0;
						sgValvesInfo.b1OutriggerUpStat = 0;
						sgValvesInfo.b1OutriggerDownStat = 0;	
					}
				}
				
				
				//
//				if(0 != ((sgValvesInfo.b1LiftUpStat<<5) | (sgValvesInfo.b1LiftDownStat<<4) |			
//							(sgValvesInfo.b1LegUpStat << 3 )| (sgValvesInfo.b1LegDownStat <<2)|
//							(	sgValvesInfo.b1OutriggerUpStat <<1) | sgValvesInfo.b1OutriggerDownStat))//�����Ϊ0
//				{
//					u8PumpActFlag = 1;
//				}
//				else if(u8PumpActDelay < 100)
//				{
//					u8PumpActDelay++;
//				}
//				else 
//				{
//					u8PumpActDelay = 0;
//					u8PumpActFlag
//				}
				
				//��λȡ������
				if((0 == sgSwiInput.b1LegUp) && (0 == sgSwiInput.b1LegDown) && 
					(0 == sgSwiInput.b1LiftUp) && (0 == sgSwiInput.b1LiftDown) && 
					(0 == sgSwiInput.b1OutriggerDown)&&
					(0 == sgSwiInput.b1OutriggerUp))
				{
					u8ErroDelayFlag = 0;
					PumpMotorNoAct = 0;
					sgValvesInfo.b1PumpMotorNoAct = 0;
					i32ErrCodeClr(MUTL_PUMP_REQ_ERR);
					i32SetPara(PARA_OutCtrlInfo,33);
				}
				i32SetPara(PARA_OnOffValveCurrent, sgValvesInfo.b1PumpMotorNoAct);
			
				if(1 == sgUserInfo.b1MoveLiftMode)
				{
					if((0 !=u8PumpOut)&&(0 !=(sgValvesInfo.u8Data[1] &0x03)))
					{
						sgValvesInfo.u8Data[1] = ValvesInfoRecord.u8Data[1];
					}
				}				
				
			}
	}
	else
	{
		/*ForWard BackWard LiftUp LiftDown LeanForWard LeanBackWard all disable*/
		sgValvesInfo.u8Data[1] &= 0x00;		/*Hight 2 Bit Reserve*/	
		sgValvesInfo.u8Data[2] &= 0xFC;
	}
}

			/*�����-������ת��*/
static void vMstRevProc(xMstRevPara *RevData)
{	
	
	
	uint8_t tmp = 0;
	
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
	/*�����봦��*/
	if(0 != RevData->u8ErrCode)
	{
		if (u8ErrCodeGet() < RevData->u8ErrCode)   /*lilu, 20230818, ���MCU֮ǰ��С�Ĺ�����*/
		{
			uint8_t i = 0;
			for (i=0; i<RevData->u8ErrCode; i++)
			{
				i32ErrCodeClr(i);
			}
		}
		i32ErrCodeSet(RevData->u8ErrCode - 1);	
		sgValvesInfo.u8NoAct |=	1<< NoAct_McuErr;
		if(40 == RevData->u8ErrCode)
		{
			i32ErrCodeSet(EMS_LOCK_ERR);
		}
	}
	else
	{
		if (u8ErrCodeGet() < 50)				/*lilu, 20230817, û��MCU�Ĺ��ϵ�ʱ�򣬾����MCU�����й�����*/
		{
			uint8_t i = 0;
			for (i=0; i<50; i++)
			{
				i32ErrCodeClr(i);
			}
		}
		sgValvesInfo.u8NoAct &=~(1<< NoAct_McuErr);
	}
	
	i16MotorSpd = (uint16_t)(RevData->u8SpeedFdbHigh << 8) | RevData->u8SpeedFdbLow ;
//	i32LogWrite(DEBUG, LOG_USER, "Motor Rev Spd = %d\r\n", i16MotorSpd);
	__disable_irq();
	tmp = i16MotorSpd / sgUserInfo.u16RatioOfTransmission;
	/*Motor Speed*/
	gCanSendPdoInfo.CanSend260Info.u8Movespeed = tmp ;
	__enable_irq();
	
//	tmp = RevData->u8MotorTmp;
//	/*Motor Temp*/
//	gCanSendPdoInfo.CanSend260Info.i16MotorTemp = RevData->u8MotorTmp;
//	
//	tmp = RevData->u8BoardTmp;
//	/**/
//	gCanSendPdoInfo.CanSend260Info.i16CtrlTemp = RevData->u8BoardTmp;
	
//	__disable_irq();
//	gCanSendPdoInfo.CanSend260Info.i16MotorCurrent = (RevData->u8CurrentHigh << 8) | RevData->u8CurrentLow;
//	__enable_irq();
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

/*���͸�Mst�Ļص�����*/
static void vMstSendProc(xMstSendPara *SendData) 
{
	static xValvesInfo sgLastValvesInfo;
	static uint8_t u8MoveSwitchFlag = 0;
	static uint8_t u8MainConnectCnt =0;
	
	float fTmp = 0.0;
	xMstSendStat LastStatus;
	int16_t i16Spd = 0;
	uint8_t u8Flag = 0;
		
	LastStatus.u8Data = SendData->buf[2];
	SendData->buf[2] = 0;
	 
	if ((0 == sgSwiInput.b1SafeLock)
		&&(u8MainConnectCnt>100))		/*add Main Connector*/
	{
		SendData->b1PowerLineOn = 1;
	}
	else if(0 == sgSwiInput.b1SafeLock)
	{
		u8MainConnectCnt ++;
	}
	else
	{
		u8MainConnectCnt =0;
	}
	
	i16Spd = (SendData->u8TargetHigh << 8) | SendData->u8TargetLow;	
	i16Spd = abs(i16Spd);
	
	/*Move Mode*/
	if ((sgLastValvesInfo.b1ForWardStat != sgValvesInfo.b1ForWardStat) || 
		(sgLastValvesInfo.b1BackWardStat != sgValvesInfo.b1BackWardStat))
	{//ǰ������״̬�任
		if ((0 == i16Spd) ||
		   ((1 == LastStatus.b1BackwardReq) && (1 == sgValvesInfo.b1ForWardStat)) || 
		   ((1 == LastStatus.b1ForwardReq) && (1 == sgValvesInfo.b1BackWardStat)))
		{//ǰ�������л���ɲ��
			sgLastValvesInfo.b1ForWardStat = sgValvesInfo.b1ForWardStat;
			sgLastValvesInfo.b1BackWardStat = sgValvesInfo.b1BackWardStat;
		}
		else 
		{
			if (((1 == sgLastValvesInfo.b1ForWardStat) && (0 == sgValvesInfo.b1ForWardStat)) ||
				((1 == sgLastValvesInfo.b1BackWardStat) && (0 == sgValvesInfo.b1BackWardStat)))
			{//ǰ�������˹���
				if (0 == u8MoveSwitchFlag)
				{
					uint16_t u16Tmp = 0;
					//�����ٶ�
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
//			sgValvesInfo.b4NoMove |= 1 << NoMove_Ems;
//			i32ErrCodeSet(MOVE_EMS_ERR);
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
		(sgLastValvesInfo.b1OutriggerDownStat != sgValvesInfo.b1OutriggerDownStat) ||
		(sgLastValvesInfo.b1OutriggerUpStat != sgValvesInfo.b1OutriggerUpStat)||
		(sgLastValvesInfo.b1LegUpStat != sgValvesInfo.b1LegUpStat)||
		(sgLastValvesInfo.b1LegDownStat != sgValvesInfo.b1LegDownStat))	
	{
//		if ((inserted_data[0] * PROP_CURRENT_FACOTR) < sgUserInfo.fPropMinCurrent)
		if (((inserted_data[1] * PROP_CURRENT_FACOTR) < sgUserInfo.fPropMinCurrent) && (0 == SendData->u8PumpTarget))
		{
			sgLastValvesInfo.b1LiftUpStat = sgValvesInfo.b1LiftUpStat;
			sgLastValvesInfo.b1LiftDownStat = sgValvesInfo.b1LiftDownStat;
			sgLastValvesInfo.b1OutriggerDownStat = sgValvesInfo.b1OutriggerDownStat;
			sgLastValvesInfo.b1OutriggerUpStat = sgValvesInfo.b1OutriggerUpStat;
			sgLastValvesInfo.b1LegUpStat = sgValvesInfo.b1LegUpStat;
			sgLastValvesInfo.b1LegDownStat = sgValvesInfo.b1LegDownStat;
			
			if (1 == sgLastValvesInfo.b1LiftDownStat)
			{
				i32DoPwmSet(LIFTUP_VALVE, DRIVER_OPEN);
				i32DoPwmSet(OUTRIGGER_UP_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(OUTRIGGER_DOWN_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(LEG_UP_VALVE,DRIVER_CLOSE);
			}
			else if(1 == sgLastValvesInfo.b1LiftUpStat)
			{
				vPropSetTarget(LIFTDOWN_VALVE, 0);
				i32DoPwmSet(LIFTUP_VALVE, DRIVER_OPEN);
				i32DoPwmSet(OUTRIGGER_UP_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(OUTRIGGER_DOWN_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(LEG_UP_VALVE,DRIVER_CLOSE);
			}
			else if (1 == sgLastValvesInfo.b1OutriggerDownStat)
			{
				vPropSetTarget(LIFTDOWN_VALVE, 0);
				i32DoPwmSet(LIFTUP_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(OUTRIGGER_UP_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(OUTRIGGER_DOWN_VALVE, DRIVER_OPEN);
				i32DoPwmSet(LEG_UP_VALVE,DRIVER_CLOSE);
			}
			else if (1 == sgLastValvesInfo.b1OutriggerUpStat)
			{
				vPropSetTarget(LIFTDOWN_VALVE, 0);
				i32DoPwmSet(LIFTUP_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(OUTRIGGER_UP_VALVE, DRIVER_OPEN);
				i32DoPwmSet(OUTRIGGER_DOWN_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(LEG_UP_VALVE,DRIVER_CLOSE);
				
			}
			else if(1 == sgLastValvesInfo.b1LegDownStat)
			{
				vPropSetTarget(LIFTDOWN_VALVE, 0);
				i32DoPwmSet(LIFTUP_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(OUTRIGGER_UP_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(OUTRIGGER_DOWN_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(LEG_UP_VALVE,DRIVER_OPEN);
			}
			else if(1 == sgLastValvesInfo.b1LegUpStat)
			{
				vPropSetTarget(LIFTDOWN_VALVE, 0);
				i32DoPwmSet(LIFTUP_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(OUTRIGGER_UP_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(OUTRIGGER_DOWN_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(LEG_UP_VALVE,DRIVER_OPEN);
			}
			else
			{
				vPropSetTarget(LIFTDOWN_VALVE, 0);
				i32DoPwmSet(LIFTUP_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(OUTRIGGER_UP_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(OUTRIGGER_DOWN_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(LEG_UP_VALVE,DRIVER_CLOSE);
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
		}
		else if(1 == sgLastValvesInfo.b1LegDownStat)
		{
			int32_t i32PropValue = 0;
			i32PropValue = _IQ((sgUserInfo.fPropMinCurrent + (sgUserInfo.fPropMaxCurrent - sgUserInfo.fPropMinCurrent) * PUMP_MAX_VALUE /2 / PUMP_RANGE) / PROPD_STD_CURRENT);
			vPropSetTarget(LIFTDOWN_VALVE, i32PropValue);
		}
		else if ((1 == sgLastValvesInfo.b1LiftUpStat) ||
							(1 == sgLastValvesInfo.b1LegUpStat) || 
				(1 == sgLastValvesInfo.b1OutriggerDownStat)||
				(1 == sgLastValvesInfo.b1OutriggerUpStat))
		{
			if(0 == sgLastValvesInfo.b1LiftUpStat)	/*���ͱõ������ٶ�,ǰ��ͺ�����ٶ�*/
			{//������״̬���ٶȵ�λ����
				if (1 == sgValvesInfo.b1OutriggerUpStat)/*arrcoding to Parameter*/
				{
					u8PumpOrPropValue = sgUserInfo.u8LeanBackWardValue;
				}
				else if (1 == sgValvesInfo.b1OutriggerDownStat)/*arrcoding to Parameter*/
				{
					u8PumpOrPropValue = sgUserInfo.u8LeanBackWardValue;
				}
				else if(1 == sgLastValvesInfo.b1LegUpStat)
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
		u8Flag = 1<< 1;
		vSetSocFlag(DISABLE_BATTERY);
	}
	else
	{
		vSetSocFlag(ENABLE_BATTERY);
	}
	
	if (0 != u8Flag)
	{
		SendData->b1ServoOn = 1;
	}
		
	
	if (1 == sgValvesInfo.b1NoActFlag)
	{
		SendData->b1ServoOn = 0;				/*�����ƶ��� �Ͽ�ʹ��*/
		SendData->u8PumpTarget = 0;
		SendData->b1LiftReq = 0;
		SendData->u8TargetHigh = 0;
		SendData->u8TargetLow = 0;
		SendData->b1ForwardReq = 0;
		SendData->b1BackwardReq = 0;
		vPropSetTarget(LIFTDOWN_VALVE, 0);
	}
	
	{
		static uint32_t u32Cnt = 0;
		u32Cnt++;
		{
			/*lilu 20230819 For Test*/
			{				
				i32SetPara(PARA_AngleValue, (uint16_t)i32LocalAiGetValue(AI_B_AI1_R));		/*AI1*/
				i32SetPara(PARA_PressureVlaue1, (uint16_t)i32LocalAiGetValue(AI_B_AI2_R));	/*AI2*/
				i32SetPara(PARA_PressureVlaue2, (uint16_t)i32LocalAiGetValue(AI_B_AI3_R));	/*AI3*/
				i32SetPara(PARA_LoadRate, sgValvesInfo.u8Data[0]);				/*Valve NoFlag*/
				i32SetPara(PARA_CalibrationStatus, sgValvesInfo.u8Data[1]);		/*Value State*/
				
				i32SetPara(PARA_ForwardValveCurrent, i16Spd);		/*Send Motor Value*/
				i32SetPara(PARA_BackValveCurrent, i16MotorSpd);		/*Rev Motor Value*/
				i32SetPara(PARA_LiftValveCurrent, SendData->u8PumpTarget);		/*Send Pump Value*/
				i32SetPara(PARA_PropValveCurrent, inserted_data[1] * PROP_CURRENT_FACOTR * 1000);	/*Prop Current*/
				
							/*Send Status*/
				i32SetPara(PARA_TurnRightValveCurrent, ((uint16_t)sgValvesInfo.u8NoAct << 8) | ((uint16_t)sgValvesInfo.b4NoMove << 4) | ((uint16_t)sgValvesInfo.b4NoLiftUp));
				i32SetPara(PARA_TurnLeftValveCurrent, ((uint16_t)sgValvesInfo.b4Gear1Spd << 12) | ((uint16_t)sgValvesInfo.b4Gear2Spd << 8) | \
					((uint16_t)sgValvesInfo.b4Gear3Spd << 4) | ((uint16_t)sgValvesInfo.b4Gear4Spd));
				i32SetPara(PARA_BrakeValveCurrent, u8PumpOrPropValue);
				
				i32SetPara(PARA_BatteryVoltage,(i32LocalAiGetValue(AI_B_KSI_CHECK)));
				i32SetPara(PARA_ExtSignal, sgSwiInput.u16data);						/*Swi*/
				i32SetPara(PARA_PcuKeyInfo, u8ErrCodeGet());						/*ErrCode*/
				
				i32SetPara(PARA_MotorSpd,i16MotorSpd);
				i32SetPara(PARA_HandleAnalog,i16Spd);
				i32SetPara(PARA_BmsSoc, (gCanSendPdoInfo.CanSend260Info.u8Soc *2.5));		/*BMS SOC*/
				i32SetPara(PARA_TemporaryUnlock,u8ErrCodeGet());			
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
//		case LEG_UP_VALVE:
//			i32ErrCodeSet(LEG_UP_VALVE_ERR);
//			/*add ErrCode*/
//			break;
//		case OUTRIGGER_UP_VALVE:
//			i32ErrCodeSet(OUTRIGGER_UP_VALVE_ERR);
//			/*add ErrCode*/
//			break;
//		case OUTRIGGER_DOWN_VALVE:
//			i32ErrCodeSet(OUTRIGGER_DOWN_VALVE_ERR);
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
//	switch((uint8_t)AiErrNo)
//	{
//		case AI_B_AI1_R_ERR:
//			i32ErrCodeSet(AI_B_AI1_ERR);
//			break;
//		case AI_B_AI2_R_ERR:
//			i32ErrCodeSet(AI_B_AI2_ERR);
//			break;
////		case AI_B_AI3_R_ERR:
////			i32ErrCodeSet(AI_B_AI3_ERR);
////			break;
//		case AI_5V_12V_OUT1_I_ERR:
//			i32ErrCodeSet(AI_5V_12V_OUT1_ERR);
//			break;
//		case AI_5V_12V_OUT2_I_ERR:
////			i32ErrCodeSet(AI_5V_12V_OUT2_ERR);
//			break;
//		default:
//			break;
//										
//	}
}

/*����PDO������*/
static void vCanRevPdoProc(void)
{

	/*5msִ��һ��*/
	static xCanRev190Info CanRev190InfoLast;
	static xCanRev200Info CanRev200InfoLast;
	static xCanRev360Info CanRev360InfoLast;
	tCanFrame CanSendFrame;
	tCanFrame CanSend0={0};
	uint8_t i = 0;
	static uint16_t cnt = 0;
	static uint16_t u16CanRev1ACCnt;
	
	if(LiBattery == sgUserInfo.u8BatteryType)
	{
		if(0 != memcmp((char*)CanRev200InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo2.u8Data, sizeof(CanRev200InfoLast)))
		{
			memcpy((char*)CanRev200InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo2.u8Data, sizeof(CanRev200InfoLast));
			/*�����صĲ���*/
			gCanSendPdoInfo.CanSend260Info.u8Soc = CanRev200InfoLast.u8Soc;
		}
		
		if(0 != memcmp((char*)CanRev190InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo1.u8Data, sizeof(CanRev190InfoLast)))
		{
			memcpy((char*)CanRev190InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo1.u8Data, sizeof(CanRev190InfoLast));
			/*�����صĲ���*/
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
				
			}
		}
	}
	else
	{
		gCanSendPdoInfo.CanSend260Info.u8Soc = u8GetBatterySoc();
	}
	/*��������*/
	if ((gCanSendPdoInfo.CanSend260Info.u8Soc <= BAT_LOW_ERR_VAL)
			||(0 != (CanRev190InfoLast.u8BMS_ERR_Require & 0x01)))//���ߣ�������ֹ��ֻ�½������Ƚ�42
	{
		if(gCanSendPdoInfo.CanSend260Info.u8Soc <= BAT_LOW_ERR_VAL)
		{
			i32ErrCodeSet(BAT_LOW_2_ERR);
		}
		sgValvesInfo.b4NoMove |=  (1 << NoMove_LowBat);
		sgValvesInfo.b4NoLiftUp |= (1 << NoLiftUp_LowBat);
		u8OutriggerNoActFlag = 1;
		
	}
	else if ((gCanSendPdoInfo.CanSend260Info.u8Soc <= BAT_LOW_WARING_VAL)
				||(0 !=(CanRev190InfoLast.u8BMS_ERR_Require & 0x0A)))//����ȫ�٣���������,������41
	{
		if(gCanSendPdoInfo.CanSend260Info.u8Soc <= BAT_LOW_WARING_VAL)
		{
			i32ErrCodeSet(BAT_LOW_1_ERR);
		}
		sgValvesInfo.b4Gear1Spd |= (1<<Gear1_Spd_Con1);
		i32ErrCodeClr(BAT_LOW_2_ERR);
		sgValvesInfo.b4NoMove &= ~(1 << NoMove_LowBat);
		sgValvesInfo.b4NoLiftUp |= (1 << NoLiftUp_LowBat);
		u8OutriggerNoActFlag = 0;

	}
	else
	{
		sgValvesInfo.b4Gear1Spd = 0;
		i32ErrCodeClr(BAT_LOW_1_ERR);
		i32ErrCodeClr(BAT_LOW_2_ERR);
		sgValvesInfo.b4NoMove = 0;
		sgValvesInfo.b4NoLiftUp  = 0;
		u8OutriggerNoActFlag = 0;
	}
	
	if(0 == sgSwiInput.b1SteerLock)//ת���Ĵ���
	{
		if(0 != memcmp((char*)CanRev360InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo3.u8Data, sizeof(CanRev360InfoLast)))
		{
			gCanSendPdoInfo.CanSend260Info.u8ErrorSteer = CanRev360InfoLast.u8ErrSteer;
		}
	}

	CanSendFrame.u32ID = 0x260;
	CanSendFrame.u8Rtr = 0;
	CanSendFrame.u16DataLength = 8;
	for (i=0; i<8; i++)                                 
	{
		CanSendFrame.u8Data[i] = gCanSendPdoInfo.CanSend260Info.u8Data[i];
	}
	
	if (cnt < (100/USER_ECU_PERIOD))
	{
		cnt++;
	}
	else
	{
		i32CanWrite(Can0, &CanSendFrame);
		cnt = 0;
	}
}

/*lilu 20230703 ģ�������*/
static void vAiMonitor(void)
{
	int32_t i32AdcValue = 0;
	
	i32AdcValue = i32LocalAiGetValue(MOVE_THROTTLE);
	
	if (i32AdcValue > sgUserInfo.u16ThrottleMax)	/*DeadZone Max*/
	{
		u16MotorVal = MOTOR_MAX_SPEED_VALUE;
	}
	else if (i32AdcValue >= (sgUserInfo.u16ThrottleMidValue))		/*����һ��*/
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
//	u16MotorVal = u16SteerAngleDecSpd(u16MotorVal, abs(gCanRevPdoInfo.CanRevInfo2.i16SteerAngle), &sgUserInfo.SteerAngleDecSpd);
	
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
		sgValvesInfo.b1LiftUpStat = 0;
		sgValvesInfo.b1LiftDownStat = 0;
		if (1 == sgSwiInput.b1LiftUp)
		{
			if (i32AdcValue > sgUserInfo.u16LiftUpMax)
			{
				u8PumpOrPropValue = PUMP_MAX_VALUE;
			}
			else if (i32AdcValue >= sgUserInfo.u16LiftUpMidValue)		/*С���������*/
			{
				u8PumpOrPropValue = sgUserInfo.u16LiftUpMid + (i32AdcValue - sgUserInfo.u16LiftUpMidValue) * (PUMP_RANGE - sgUserInfo.u16LiftUpMid) / sgUserInfo.u16LiftUpRange;
			}
			else if (i32AdcValue >= sgUserInfo.u16LiftUpMin)	/*С������һ��*/
			{
				u8PumpOrPropValue = (i32AdcValue - sgUserInfo.u16LiftUpMin) * sgUserInfo.u16LiftUpMid / sgUserInfo.u16LiftUpRange;
			}			
			else 
			{
				u8PumpOrPropValue = 0;
			}
			if (0 != u8PumpOrPropValue)
			{
				if (0 == sgValvesInfo.b1LiftUpFlag)		/*lilu 20230811 add NoLiftFlag*/
					sgValvesInfo.b1LiftUpStat = 1;
			}
		}
		else if(1 == sgSwiInput.b1LiftDown)
		{
			if (i32AdcValue > sgUserInfo.u16LiftDownMax)	/*DeadZone Max*/
			{
				u8PumpOrPropValue = PUMP_MAX_VALUE;
			}
			else if (i32AdcValue >= (sgUserInfo.u16LiftDownMidValue))		/*����һ��*/
			{
				u8PumpOrPropValue = sgUserInfo.u16LiftDownMid + (i32AdcValue-sgUserInfo.u16LiftDownMidValue ) * (PUMP_RANGE - sgUserInfo.u16LiftDownMid) / sgUserInfo.u16LiftDownRange;
			}
			else if (i32AdcValue >= sgUserInfo.u16LiftDownMin)	/*DeadZone Min*/ 
			{
				u8PumpOrPropValue = (i32AdcValue-sgUserInfo.u16LiftDownMin ) * sgUserInfo.u16LiftDownMid / sgUserInfo.u16LiftDownRange;
			}
			else
			{
				u8PumpOrPropValue = 0;
			}
			if (0 != u8PumpOrPropValue)
			{
				sgValvesInfo.b1LiftDownStat = 1;
			}
		}
		else
		{
			sgValvesInfo.b1LiftUpStat = 0;
			sgValvesInfo.b1LiftDownStat = 0;
		}
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



/*******************************************************************************
* Name: void vUserEcuInit(void)
* Descriptio: UserEcu��ʼ������
* Input: NULL
* Output: NULL  
*******************************************************************************/
void vUserEcuInit(void)
{	
	uint16_t u16Tmp = 0;
//	xRevCallBackProc CanId42C = {.u32CanId = 0x42C, .u32Data = 0, .CallBack = vCanId42CProc};
//	xRevCallBackProc DeviceCanId62C = {.u32CanId = 0x62C, {.u8DataCnt = 3, .u8Data0 = 0x22, .u8Data1 = 0x08, .u8Data2 = 0x50}, .CallBack = vDeviceCanId62CProc};
//	xRevCallBackProc PcCanId62C = {.u32CanId = 0x62C, {.u8DataCnt = 3, .u8Data0 = 0x22, .u8Data1 = 0x02, .u8Data2 = 0x2A}, .CallBack = vPcCanId62CProc};
	
	vMstRevRegister(vMstRevProc);
	vMstSendRegister(vMstSendProc);
	
//	vCanRevMsgRegister(&CanId42C);
//	vCanRevMsgRegister(&DeviceCanId62C);
//	vCanRevMsgRegister(&PcCanId62C);
	
	vDoPwmErrReg(vDoPwmErrCallBack);
	vPropErrReg(vPropErrCallBack);
	vAiErrReg(vAiErrCallBack);
	
	memset(&sgValvesInfo, 0, sizeof(sgValvesInfo));
	
	sgUserInfo.fPropMinCurrent = i32GetPara(PROP_MIN_CURRENT) / 1000.0;		/*��������С����*/
	sgUserInfo.fPropMaxCurrent = i32GetPara(PROP_MAX_CURRENT) / 1000.0;		/*������������*/
	
	sgUserInfo.u8LeanForWardValue = i32GetPara(LEAN_FORWARD_PARA) * PUMP_RANGE / 100;		/*ǰ�����*/
	sgUserInfo.u8LeanBackWardValue = i32GetPara(LEAN_BACKWARD_PARA) * PUMP_RANGE / 100;		/*�������*/
	
	sgUserInfo.b1LiftMode = i32GetPara(LIFT_MODE_PARA);						/*����ģʽ-�ƶ�̤������*/
	
	sgUserInfo.u16Gear1Spd = i32GetPara(PARA_Gear1Spd) * MOTOR_SPEED_RANGE / 100;	/*1���ٶ�*/
	sgUserInfo.u16Gear2Spd = i32GetPara(PARA_Gear2Spd) * MOTOR_SPEED_RANGE / 100;	/*2���ٶ�*/
	sgUserInfo.u16Gear3Spd = i32GetPara(PARA_Gear3Spd) * MOTOR_SPEED_RANGE / 100;	/*3���ٶ�*/
	sgUserInfo.u16Gear4Spd = i32GetPara(PARA_Gear4Spd) * MOTOR_SPEED_RANGE / 100;	/*4���ٶ�*/
	
	sgUserInfo.u16ThrottleMin = i32GetPara(MOVE_THROTTLE_MIN) * 100;		/*��������ֵ, uint 0.1V*/
	sgUserInfo.u16ThrottleMax = i32GetPara(MOVE_THROTTLE_MAX) * 100;		/*��������ֵ��� ,uint 0.1V*/
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
	sgUserInfo.u16LiftDownRange	= ( sgUserInfo.u16LiftDownMax - sgUserInfo.u16LiftDownMin) >> 1;
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
		sgSwiInput.b1HeightSpdLimit = sgSaveState.b1HeightSpdLimit;
		sgUserInfo.b1PasswordFunc = sgSaveState.b1PasswordFunc;
		u16EepromRead(PARA_HourSetTime, &u16Tmp, 1);	
	}

	
	__disable_irq();
	gCanSendPdoInfo.CanSend260Info.u8HourCountL = (u32HourCount/10) & 0xFF;
	gCanSendPdoInfo.CanSend260Info.u8HourCountH = ((u32HourCount/10) >> 8) & 0xFF;

	__enable_irq();		
	/*Para Initial*/
	
	//sgUserInfo.u16RentalTime = i32GetPara(RENTAL_TIME);
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
		sgUserInfo.fDownSpdPer5msAccStep = 0;
		sgUserInfo.fDownSpdPer5msDecStep = 0;
	}

	vSetPdoPara(sgPdoPara);
	
	vSetNetTimer(TIMER_EcuPowerOn, ECU_POWERON_DELAY_TIME);	/**/
	
	vSetNetTimer(TIMER_HourCount, HOURCOUNT_PERIOD);
}

/*******************************************************************************
* Name: void vUserEcuProc(void)
* Descriptio: UserEcu�û�������
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
	
	/*�ϵ���*/
	if ((false == u8GetNetTimerOverFlag(TIMER_EcuPowerOn)) && (true == u8GetNetTimerStartFlag(TIMER_EcuPowerOn)))
	{ 
		if (1 == u8SwiInitChcek())
		{
			sgValvesInfo.u8NoAct |= 1 << NoAct_Init;
			i32ErrCodeSet(ACT_INIT_ERR);
		}
	}

	if(1 == u8EcuProcFlag)
	{
//		u16CanId5ACPeriod++;
//		if (u16CanId5ACPeriod < 210)	/*200 = 5Times*/
//		{
//			if (0 == (u16CanId5ACPeriod % CANID_5AC_SEND_PERIOD))	/*Send 5 times 5AC*/
//			{
//				vCanId5ACSend();
//			}
//		}
//		else 
//		{
//			u16CanId5ACPeriod = 210;
//		}
		
//		if (1 == sgUserInfo.b1PasswordFunc)
//		{
//			if (u16CanRev62CCnt++ >= CAN_62C_LOST_NO)
//			{
//				sgValvesInfo.u8NoAct |= 1 << NoAct_Security;
//				/*62C Lost Err*/
//				i32ErrCodeSet(DEV_62C_LOST_ERR);
//				u16CanRev62CCnt = 0;
//			}
//		}
		ValvesInfoRecord = sgValvesInfo;
		vAiMonitor();
		vSwiMonitor();		
		vCanRevPdoProc();
		u8ErrCode = u8ErrCodeGet();
		if (0 != u8ErrCode)
		{
			gCanSendPdoInfo.CanSend260Info.u8ErrorMove = u8ErrSwitchArray[u8ErrCode];
			vLedSendAlmCode(u8ErrSwitchArray[u8ErrCode]);
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
		}
//			vHourCountWrite(u32HourCount);
			__disable_irq();
			gCanSendPdoInfo.CanSend260Info.u8HourCountL = (u32HourCount/10) & 0xFF;
			gCanSendPdoInfo.CanSend260Info.u8HourCountH = ((u32HourCount/10) >> 8) & 0xFF;
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
				gCanSendPdoInfo.CanSend260Info.u8HourCountL = (u32HourCount/10) & 0xFF;
				gCanSendPdoInfo.CanSend260Info.u8HourCountH = ((u32HourCount/10) >> 8) & 0xFF;
				__enable_irq();
			}
		}
	
	vWdgSetFun(WDG_USER_BIT);
}
#endif
