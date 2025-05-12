/*******************************************************************************
* Filename: UserStackerTruckProc.c	                                           *
* Description: 逻辑层的源文件，包含对应的回调函数	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/05/30    														   *
* Revision:	 														 		   *
*******************************************************************************/
#include "UserCanComm.h"
#include "UserGangLiPHZ15T.h"
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

#if (USER_TYPE == USER_GANGLI_PHZ15T_MOVE)

const static xPdoParameter  sgPdoPara = 
{
	.TpdoPara = {
		{.u8Flag = 1, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 1, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 1, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 1, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 50, .u16CanId = 0x260},
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 100, .u16CanId = 0x270},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 100, .u16CanId = 0x00},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 100, .u16CanId = 0x00},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 100, .u16CanId = 0x00},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 100, .u16CanId = 0x00},
	},
	.RpdoPara = {
		{.b1Flag = 1, .b11CanRevId = 0x17F},
		{.b1Flag = 1, .b11CanRevId = 0x27F},
		{.b1Flag = 1, .b11CanRevId = 0x37F},
		{.b1Flag = 1, .b11CanRevId = 0x47F},
		{.b1Flag = 0, .b11CanRevId = 0x00},
		{.b1Flag = 0, .b11CanRevId = 0x00},	
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
#define	NoAct_Init				1
#define	NoAct_Lock				2
#define	NoAct_Brake				3
#define	NoAct_FaultLock		4
#define NoAct_BMSLost			5
#define NoAct_LiftLost		6
#define NoAct_BMSError		7

typedef union
{
	uint16_t u16data;
	struct
	{
		uint16_t b1Charge: 1;
		uint16_t b1SafeLock: 1;
		uint16_t b1FootBrake: 1;
		uint16_t b1Throttle: 1;
		uint16_t b1HandBrake: 1;
		uint16_t b1Forward: 1;
		uint16_t b1Backward: 1;
		uint16_t b1SafeBelt: 1;
		uint16_t b1SlowMode: 1;
		uint16_t b1SpeedLimit: 1;
		uint16_t b6Reserve0: 6;
	};
}xSwiInput;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t b1NoActFlag: 1;
		uint8_t b1MoveFlag: 1;
		uint8_t b1Gear1SpdFlag: 1;
		uint8_t b1Gear2SpdFlag: 1;
		uint8_t b1Gear3SpdFlag: 1;
		uint8_t b1Gear4SpdFlag: 1;
		uint8_t b1ForWardStat: 1;
		uint8_t b1BackWardStat: 1;
				
		uint8_t	b4Gear1Spd: 4;
		uint8_t	b4Gear2Spd: 4;
		
		uint8_t	b4Gear3Spd: 4;
		uint8_t	b4Gear4Spd: 4;
		
		uint8_t u8NoAct;
		uint8_t u8NoLift;
		
		uint8_t b4MainDriverOpen: 4;
		uint8_t b4Reserve0: 4;
		
		uint8_t u8Reserve1;
		uint8_t u8Reserve2;
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
	
	float		fMoveSpdPer5msAccStep;
	float		fMoveSpdPer5msDecStep;
	
	uint16_t	u16ThrottleMin;
	uint16_t	u16ThrottleMax;
	uint16_t	u16ThrottleRange;
	uint16_t	u16ThrottleMid;
	uint16_t	u16ThrottleMidValue;
	
	uint16_t	u16SteerAnalogMin;
	uint16_t	u16SteerAnalogMax;
	uint16_t	u16SteerAnalogMid;
	uint16_t  u16SteerAngle;
	
	xSteerAngleDecSpd SteerAngleDecSpd;
	
	uint8_t		u8BatteryType;
	
	uint8_t		b1HourConutMode: 1;		/*0: 上电计时， 1：互锁计时*/
	uint8_t		b1LiftCheck: 1;		/*0： 无起升， 1：有起升*/
	uint8_t		b1SteerCheck: 1;			/*0： 无转向， 1：有转向*/
	uint8_t		b1DaocheDOEnable: 1;			/*0:有倒车继电器，1:无倒车继电器*/
	uint8_t		b1FengshanEnable: 1;		/*0：有风扇继电器， 1：无风扇继电器*/
	uint8_t		b7Reserve1: 3;
	
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
		uint16_t b1HeightSpdLimit: 1;
		uint16_t b11Reserve: 15;
	};
}xSaveStateInfo;

static xSaveStateInfo sgSaveState;

static int16_t	i16MotorSpd = 0;
static uint16_t u16MotorVal = 0;
static xValvesInfo sgValvesInfo;
static xUserInfo sgUserInfo;
static xSwiInput sgSwiInput;
static uint32_t u32HourCount = 0;

static int16_t i16SteerAngle = 0;
static uint16_t DRIVER_FLAG = 0;
static uint16_t Batter_SOC = 0;
static uint16_t Rev_Speed;

/*PDO接收待完善*/
xCanRevPdoInfo gCanRevPdoInfo; 
//{
//	.CanRevInfo1.u8Soc = 100,					/*BmsSoc Default: 100*/
//};				
/*PDO发送待完善*/
xCanSendPdoInfo gCanSendPdoInfo = 
{
	//.CanSend260Info.u8ErrorLift = gCanRevPdoInfo.CanRevInfo8.u8ErrorLift,
	//.CanSend260Info.u8ErrorBMS = 0x00,
	//.CanSend33CInfo.u8SoftVersionHighByte = 0x10,
};			

const static uint8_t u8ErrSwitchArray[20][3] = 
{	
/*  0    					1							2						3						*/
		{3, 13, 29}, {56, 57, 31}, {58, 59, 24}, {9, 17, 28},
    {6, 61, 32}, {62, 20, 25}, {7,  15, 26}, {8, 16, 27}, 
		{70,14, 19}, {66, 67, 33}, {10, 72, 73}, {76,34, 35},
		{0,  0,  0}, {0,   0,  0}, {0,   0,  0}, {0,  0,  0},
		{0,  0,  0}, {0,   0,  0}, {0,   0,  0}, {0,  0,  0},
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

	i16MotorSpd = (uint16_t)(RevData->u8SpeedFdbHigh << 8) | RevData->u8SpeedFdbLow ;
//	i32LogWrite(DEBUG, LOG_USER, "Motor Rev Spd = %d\r\n", i16MotorSpd);
	__disable_irq();
	tmp = i16MotorSpd / sgUserInfo.u16RatioOfTransmission;
	/*Motor Speed*/
	gCanSendPdoInfo.CanSend260Info.u8SMoveSpeed = tmp;
	__enable_irq();
//	
//	tmp = RevData->u8MotorTmp;
//	/*Motor Temp*/
//	gCanSendPdoInfo.CanSend43CInfo.i16MotorTemp = RevData->u8MotorTmp;
//	
	tmp = RevData->u8BoardTmp - 40;		
	/**///接收控制器温度并处理
	if (0 == sgUserInfo.b1FengshanEnable)
	{
		if (tmp > 45)
		{
			i32DoPwmSet(FENGSHAN_DO,DRIVER_OPEN);
		}
		else if (tmp < 40)
		{
			i32DoPwmSet(FENGSHAN_DO,DRIVER_CLOSE);
		}
	}
//	gCanSendPdoInfo.CanSend43CInfo.i16CtrlTemp = RevData->u8BoardTmp;
	
//	__disable_irq();
//	gCanSendPdoInfo.CanSend43CInfo.i16MotorCurrent = (RevData->u8CurrentHigh << 8) | RevData->u8CurrentLow;
//	__enable_irq();
}

//static void vMoveModeNoChangeProc(int16_t *i16Spd, const xValvesInfo *pValvesInfo)
//{
//	float fTmp = 0;
//	if ((1 == pValvesInfo->b1ForWardStat) || (1 == pValvesInfo->b1BackWardStat))
//	{
//		if (*i16Spd > u16MotorVal)
//		{
//			fTmp = sgUserInfo.fMoveSpdPer5msDecStep * (0.01 + 0.99 * *i16Spd / MOTOR_SPEED_RANGE);
//			if ((((*i16Spd - u16MotorVal) > fTmp) && (0 != sgUserInfo.fMoveSpdPer5msDecStep)))
//			{
//				*i16Spd -= fTmp;
//			}
//			else
//			{
//				*i16Spd = u16MotorVal;
//			}
//		}
//		else if (*i16Spd < u16MotorVal)
//		{
//			fTmp = sgUserInfo.fMoveSpdPer5msAccStep * (0.01 + 0.99 * *i16Spd / MOTOR_SPEED_RANGE);
//			if (((u16MotorVal - *i16Spd) > fTmp) && (0 != sgUserInfo.fMoveSpdPer5msAccStep))
//			{
//				*i16Spd += fTmp;
//			}
//			else
//			{
//				*i16Spd = u16MotorVal;
//			}
//		}
//	}
//	else
//	{
//		*i16Spd = 0;
//	}
//}

/*发送给Mst的回调函数*/
static void vMstSendProc(xMstSendPara *SendData) 
{
//	static xValvesInfo sgValvesInfo;
	static uint8_t u8MoveSwitchFlag = 0;
	static uint16_t u16MainConnectCnt = 0;
	
	float fTmp = 0.0;
	xMstSendStat LastStatus;
	int16_t i16Spd = 0;
		
	LastStatus.u8Data = SendData->buf[2];
	SendData->buf[2] = 0;
	
	if (sgValvesInfo.b4MainDriverOpen == 1)	/*add Main Connector*/
	{
		if (u16MainConnectCnt < 1000)
		{
			u16MainConnectCnt ++;
			SendData->b1PowerLineOn = 0; 
		}
		else 
		{
			SendData->b1PowerLineOn = 1;     // 1:断开 0:连接
		}
	}
	else
	{
		u16MainConnectCnt = 0;
		SendData->b1PowerLineOn = 0; 
	}

//	i16Spd = (SendData->u8TargetHigh << 8) | SendData->u8TargetLow;
//	i16Spd = abs(i16Spd);
	
	/*Move Mode*/
	if((sgSwiInput.b1SafeLock == 1) && (sgSwiInput.b1SafeBelt == 1))
	{
		SendData->b1ServoOn = 1;
	}
	else
	{
		SendData->b1ServoOn = 0;
	}
	
	if (0 != sgSwiInput.b1Throttle)
	{
		if((sgValvesInfo.b1BackWardStat == 1) && (sgValvesInfo.b1ForWardStat == 0))
		{
			SendData->b1BackwardReq = 1;
		}
		if((sgValvesInfo.b1ForWardStat == 1) && (sgValvesInfo.b1BackWardStat == 0))
		{
			SendData->b1ForwardReq = 1;
		}
	}
	else
	{
		SendData->b1BackwardReq = 0;
		SendData->b1ForwardReq = 0; 
	}
	
	if((1 == sgSwiInput.b1FootBrake) || (1 == sgSwiInput.b1HandBrake))
	{
		SendData->b1BackwardReq = 0;
		SendData->b1ForwardReq = 0; 
		u16MotorVal = 0;
		SendData->b1ServoOn = 0; 
	}
	
	if (0 != sgSwiInput.b1Throttle)
	{
		if((0 != sgValvesInfo.b1BackWardStat)||(0 != sgValvesInfo.b1ForWardStat))
		{
			SendData->u8TargetHigh = u16MotorVal >> 8;
			SendData->u8TargetLow = u16MotorVal;
		}
		else 
		{
			SendData->u8TargetHigh = 0;
			SendData->u8TargetLow = 0;
		}
	}
	else 
	{
		SendData->u8TargetHigh = 0;
		SendData->u8TargetLow = 0;
	}
	
	{
		static uint32_t u32Cnt = 0;
		u32Cnt++;
		{			
			/*lilu 20230819 For Test*/
			{
				i32SetPara(PARA_ForwardValveCurrent, u16MotorVal);		/*Send Motor Value*/
			//	i32SetPara(PARA_BackValveCurrent, Rev_Speed);		/*Rev Motor Value*/
			//	i32SetPara(PARA_PropValveCurrent, sgUserInfo.u16SteerAngle);	/*Prop Current*/
				i32SetPara(PARA_LiftValveCurrent, ((gCanSendPdoInfo.CanSend260Info.u8HourCountH >> 8)|(gCanSendPdoInfo.CanSend260Info.u8HourCountL)));		/*Send Pump Value*/
				i32SetPara(PARA_OnOffValveCurrent, SendData->buf[2]);			/*Send Status*/
				i32SetPara(PARA_LoadRate, sgValvesInfo.u8Data[0]);				/*Valve NoFlag*/
				i32SetPara(PARA_CalibrationStatus, sgValvesInfo.u8Data[1]);		/*Value State*/
				i32SetPara(PARA_TurnRightValveCurrent, ((uint16_t)sgValvesInfo.u8NoAct << 8) | ((uint16_t)sgValvesInfo.u8NoLift));
				i32SetPara(PARA_TurnLeftValveCurrent, ((uint16_t)sgValvesInfo.b4Gear1Spd << 12) | ((uint16_t)sgValvesInfo.b4Gear2Spd << 8) | \
					((uint16_t)sgValvesInfo.b4Gear3Spd << 4) | ((uint16_t)sgValvesInfo.b4Gear4Spd));
				i32SetPara(PARA_AngleValue, (uint16_t)i32LocalAiGetValue(AI_B_AI1_R));		/*AI1*/
				i32SetPara(PARA_PressureVlaue1, (SendData->u8TargetHigh << 8) | (SendData->u8TargetLow));	/*AI2*/
				i32SetPara(PARA_PressureVlaue2, DRIVER_FLAG);	/*AI3*/
				i32SetPara(PARA_ExtSignal, sgSwiInput.u16data);						/*Swi*/
				i32SetPara(PARA_PcuKeyInfo,sgValvesInfo.u8Data[0]);						/*ErrCode*/
				i32SetPara(PARA_BmsSoc, gCanSendPdoInfo.CanSend260Info.u8Soc);		/*BMS SOC*/
				i32SetPara(PARA_BrakeValveCurrent, abs(i16SteerAngle));
				i32SetPara(PARA_SteerAngle,sgUserInfo.u16SteerAngle);
			}
		}
	}
}	
	
static void vAiErrCallBack(eAiErrNo AiErrNo)
{
	switch((uint8_t)AiErrNo)
	{
		case AI_B_AI1_R_ERR:
			u16MotorVal = 0;
			//i32ErrCodeSet(AI_B_AI1_ERR);
			break;
		case AI_B_AI2_R_ERR:
			if (u16MotorVal >= 800)
			u16MotorVal = 800;
			//i32ErrCodeSet(AI_B_AI2_ERR);
			break;
//		case AI_B_AI3_R_ERR:
//			i32ErrCodeSet(AI_B_AI3_ERR);
//			break;
//		case AI_5V_12V_OUT1_I_ERR:
//			i32ErrCodeSet(AI_5V_12V_OUT1_ERR);
//			break;
//		case AI_5V_12V_OUT2_I_ERR:
////			i32ErrCodeSet(AI_5V_12V_OUT2_ERR);
//			break;
		default:
			break;									
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
		case 0x17F:
			if (LiBattery == sgUserInfo.u8BatteryType)
			{
				if(CAN_NORMAL == u8State)
				{
//					i32ErrCodeClr(BMS_NOCAN_ERR);
//					sgValvesInfo.u8NoAct &= ~(1<<NoAct_BMSLost);
//					sgValvesInfo.u8NoLift &= ~(1<<NoAct_BMSLost);
				}
				else if(CAN_LOST == u8State)
				{
					i32ErrCodeSet(BMS_NOCAN_ERR);
					sgValvesInfo.u8NoAct |= (1<<NoAct_BMSLost);
					sgValvesInfo.u8NoLift |= (1<<NoAct_BMSLost);
					gCanSendPdoInfo.CanSend260Info.u8Soc = 0;
				}
			}
			break;
		case 0x47F:
			if(1 == sgUserInfo.b1LiftCheck)
			{
				if(CAN_NORMAL == u8State)
				{
					sgValvesInfo.u8NoAct &= ~(1<<NoAct_LiftLost);
				}
				else if(CAN_LOST == u8State)
				{
					sgValvesInfo.u8NoAct |= (1<<NoAct_LiftLost);		
				}
			}
			else
			{
				sgValvesInfo.u8NoAct &= ~(1<<NoAct_LiftLost);
			}
			break;
		default:
			break;
	
	}
}
/*接收PDO处理函数*/
static void vCanRevPdoProc(void)
{
	static xCanRev17FInfo CanRev17FInfoLast;
	static xCanRev27FInfo CanRev27FInfoLast;
	static xCanRev37FInfo CanRev37FInfoLast;	
	static xCanRev47FInfo CanRev47FInfoLast;
	static uint16_t u16CanRev17FCnt;
	
/***** 锂电池   ********/
	{
		if (LiBattery == sgUserInfo.u8BatteryType)
		{			
			uint32_t BMSErrorList = 0;
					
//			memcpy((char*)CanRev17FInfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo5.u8Data, sizeof(CanRev17FInfoLast));
//			/*添加相关的操作*/
//		
//			gCanSendPdoInfo.CanSend260Info.u8Soc = CanRev17FInfoLast.u8Soc;
//			if (CanRev17FInfoLast.u8Soc <= BAT_LOW_ERR_VAL)
//			{
//				i32ErrCodeSet(BAT_LOW_2_ERR);
//			}
//			else if (CanRev17FInfoLast.u8Soc <= BAT_LOW_WARING_VAL)
//			{
//				i32ErrCodeSet(BAT_LOW_1_ERR);
//				i32ErrCodeClr(BAT_LOW_2_ERR);
//			}
//			else
//			{
//				i32ErrCodeClr(BAT_LOW_1_ERR);
//			}
			
			memcpy((char*)CanRev37FInfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo7.u8Data, sizeof(CanRev37FInfoLast));
			/*添加相关的操作*/
			BMSErrorList = (CanRev37FInfoLast.u8Data[2] << 16 | CanRev37FInfoLast.u8Data[1] << 8 | CanRev37FInfoLast.u8Data[0]);
			{
				INT8U ErrorBms2Hmi = 0;
				INT8U ErrorBms2HmiLevel = 0;
				if (BMSErrorList != 0)
				{
					do{
						ErrorBms2Hmi += 1;
						if ((BMSErrorList & 0x3) != 0)
						{
							ErrorBms2HmiLevel = BMSErrorList & 0x3;
							break;
						}
						BMSErrorList >>= 2;
					} while(BMSErrorList != 0);
				}
				if (ErrorBms2Hmi != 0)
				{
					gCanSendPdoInfo.CanSend260Info.u8ErrorBMS = u8ErrSwitchArray[ErrorBms2Hmi - 1][ErrorBms2HmiLevel - 1];
				}
				
				if (ErrorBms2HmiLevel == 3)
				{
					sgValvesInfo.u8NoAct |= 1 << NoAct_BMSError;
					sgValvesInfo.u8NoLift |= 1 << NoAct_BMSError;
					sgValvesInfo.b4MainDriverOpen = 1;
				}
				else if (ErrorBms2HmiLevel == 2)
				{
					sgValvesInfo.u8NoAct &= ~(1 << NoAct_BMSError);
					sgValvesInfo.b4Gear4Spd |= 1 << Gear4_Spd_Con1;
					sgValvesInfo.u8NoLift |= 1 << NoAct_BMSError;
				}
				else
				{
					sgValvesInfo.u8NoAct &= ~(1 << NoAct_BMSError);
					sgValvesInfo.u8NoLift &= ~(1 << NoAct_BMSError);
					sgValvesInfo.b4Gear4Spd &= ~(1 << Gear4_Spd_Con1);
					sgValvesInfo.b4MainDriverOpen = 0;
				}
					
			}
			
		}
		else
		{
			if (CanRev17FInfoLast.b4HeartBeat != gCanRevPdoInfo.CanRevInfo5.b4HeartBeat)
			{
				/*添加参数报警*/
				i32ErrCodeSet(BAT_PARA_ERR);
			}
			
			gCanSendPdoInfo.CanSend260Info.u8ErrorBMS = 0;
			gCanSendPdoInfo.CanSend260Info.u8Soc = 70;
		}
	}
/***** 起升47F *******/
	{
		if (1 == sgUserInfo.b1LiftCheck)
		{
			memcpy((char*)CanRev47FInfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo8.u8Data, sizeof(CanRev47FInfoLast));
			/*添加相关操作*/
			if (0 != CanRev47FInfoLast.u8ErrorLift)
			{
				gCanSendPdoInfo.CanSend260Info.u8ErrorLift = CanRev47FInfoLast.u8ErrorLift;
				sgValvesInfo.u8NoAct |= 1 << NoAct_FaultLock;
				//i32ErrCodeSet(ACT_FAULT_LOCK_ERR);
			}
			else 
			{
				gCanSendPdoInfo.CanSend260Info.u8ErrorLift = 0;
				sgValvesInfo.u8NoAct &= ~( 1 << NoAct_FaultLock);
				//i32ErrCodeClr(ACT_FAULT_LOCK_ERR);
			}

			if (0 != (sgValvesInfo.u8NoAct & (1 << NoAct_LiftLost)) || 0 != (sgValvesInfo.u8NoAct & (1 << NoAct_FaultLock)))
			{
				i32ErrCodeSet(ACT_FAULT_LOCK_ERR);
			}
			else 
			{
				i32ErrCodeClr(ACT_FAULT_LOCK_ERR);
			}			
		}
		else 
		{
			gCanSendPdoInfo.CanSend260Info.u8ErrorLift = 0;
		}
	}
	
	/////////////////////////测试、、、、、、、、、、、
	
	gCanSendPdoInfo.CanSend270Info.u8NoAct = sgValvesInfo.u8NoAct;
	gCanSendPdoInfo.CanSend270Info.u8NoLift = sgValvesInfo.u8NoLift;
	
		if (0 != sgValvesInfo.u8NoLift)
	{
		gCanSendPdoInfo.CanSend270Info.u8StopLift = sgValvesInfo.u8NoLift;
	}
	else 
	{
		gCanSendPdoInfo.CanSend270Info.u8StopLift = 0;
	}
	
	if (0 == sgSwiInput.b1Charge)
	{
		if ((1 == sgSwiInput.b1Backward) || (1 == sgSwiInput.b1Forward) || (1 == sgSwiInput.b1Throttle) || (1 == sgSwiInput.b1FootBrake))
		{
			gCanSendPdoInfo.CanSend270Info.u8MoveFlag = 1;
		}
		else 
		{
			gCanSendPdoInfo.CanSend270Info.u8MoveFlag = 0;
		}
	}
}

//**接受PDO处理*//BMS
static void vBMSCanId17FProc(tCanFrame * CanFrame)
{
	xCanRev17FInfo CanRev17FInfo;
	if (LiBattery == sgUserInfo.u8BatteryType)
	{		
		memcpy((char*)CanRev17FInfo.u8Data, (char*)CanFrame->u8Data, sizeof(CanRev17FInfo));

		gCanSendPdoInfo.CanSend260Info.u8Soc = CanRev17FInfo.u8Soc;
		if (CanRev17FInfo.u8Soc <= BAT_LOW_ERR_VAL)
		{
			i32ErrCodeSet(BAT_LOW_2_ERR);
			sgValvesInfo.u8NoLift |= 1 << NoAct_Brake;
			sgValvesInfo.b4Gear4Spd |= 1 << Gear4_Spd_Con1;
		}
		else if (CanRev17FInfo.u8Soc <= BAT_LOW_WARING_VAL)
		{
			i32ErrCodeSet(BAT_LOW_1_ERR);
			i32ErrCodeClr(BAT_LOW_2_ERR);
			sgValvesInfo.u8NoLift &= ~(1 << NoAct_Brake);
			sgValvesInfo.b4Gear4Spd &= ~(1 << Gear4_Spd_Con1);
		}
		else
		{
			i32ErrCodeClr(BAT_LOW_1_ERR);
			sgValvesInfo.u8NoLift &= ~(1 << NoAct_Brake);
			sgValvesInfo.b4Gear4Spd &= ~(1 << Gear4_Spd_Con1);
		}
	}
	else 
	{
		gCanSendPdoInfo.CanSend260Info.u8Soc = u8GetBatterySoc();
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
	int32_t i32AdcValueSteer = 0;
	
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
	
	if (0 != sgUserInfo.b1SteerCheck)
	{
		i32AdcValueSteer = i32LocalAiGetValue(ANGLE_SENSOR);
		
		if (i32AdcValueSteer > sgUserInfo.u16SteerAnalogMid)
		{
			sgUserInfo.u16SteerAngle = ((i32AdcValueSteer - sgUserInfo.u16SteerAnalogMid) * 900 / (sgUserInfo.u16SteerAnalogMax - sgUserInfo.u16SteerAnalogMid));
		}
		else if (i32AdcValueSteer < sgUserInfo.u16SteerAnalogMid)
		{
			sgUserInfo.u16SteerAngle = ((sgUserInfo.u16SteerAnalogMid - i32AdcValueSteer) * 900 / (sgUserInfo.u16SteerAnalogMid - sgUserInfo.u16SteerAnalogMin));
		}
		else 
		{
			sgUserInfo.u16SteerAngle = 0;
		}
	}
	else 
	{
		sgUserInfo.u16SteerAngle = 0;
	}

	/*add Turn Dec Spd*/
	u16MotorVal = u16SteerAngleDecSpd(u16MotorVal, sgUserInfo.u16SteerAngle, &sgUserInfo.SteerAngleDecSpd);

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
	
	//i32AdcValue = i32LocalAiGetValue(LIFT_THROTTLE);

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
	
	if(1 == i32LocalDiGet(CHARGE_SWI))
	{
		SwiInput.b1Charge = 1;
	}

	if(1 == i32LocalDiGet(SAFELOCK_SWI))
	{
		SwiInput.b1SafeLock = 1;
	}
	
	if(1 == i32LocalDiGet(FOOTBRAKE_SWI))
	{
		SwiInput.b1FootBrake = 1;
	}
	
	if(1 == i32LocalDiGet(THROTTLE_SWI))
	{
		SwiInput.b1Throttle = 1;
	}
	
	if(1 == i32LocalDiGet(HANDBRAKE_SWI))
	{
		SwiInput.b1HandBrake = 1;
	}
	
	if(1 == i32LocalDiGet(FORWARD_SWI))
	{
		SwiInput.b1Forward = 1;
	}
	
	if(1 == i32LocalDiGet(BACKWARD_SWI))
	{
		SwiInput.b1Backward = 1;
	}
	
	if(1 == i32LocalDiGet(SAFEBELT_SWI))
	{
		SwiInput.b1SafeBelt = 1;
	}
	
	if(1 == i32LocalDiGet(SLOWMODE_SWI))
	{
		SwiInput.b1SlowMode = 1;
	}
	
	if(1 == i32LocalDiGet(SPEEDLIMIT_SWI))
	{
		SwiInput.b1SpeedLimit = 1;
	}
	
//	if(0x01 == sgUserInfo.u8BatteryType)   //锂电池
//	{
//		Batter_SOC = CanRev2F0InfoLast.BMS_SOC;
//	}
//	else if(0x02 == sgUserInfo.u8BatteryType)  //铅酸
//	{
//		Batter_SOC = u8GetBatterySoc();
//	}
//	else   //未定义
//	{
//		Batter_SOC = 50;
//	}
	
	/*add lock */

//		/*添加相关操作*/	
		if (1 == SwiInput.b1SpeedLimit)		//升高限速
		{
			sgValvesInfo.b4Gear2Spd |= 1 << Gear2_Spd_Con1;
		}
		else 
		{
			sgValvesInfo.b4Gear2Spd &= ~(1 << Gear2_Spd_Con1);
		}
		
		if (1 == SwiInput.b1SlowMode)		//龟速
		{
			sgValvesInfo.b4Gear3Spd |= 1 << Gear3_Spd_Con1;
		}
		else
		{
			sgValvesInfo.b4Gear3Spd &= ~(1 << Gear3_Spd_Con1);
		}
		
		if (1 == SwiInput.b1Charge)		//充电检测
		{
			sgValvesInfo.u8NoAct |= 1 << NoAct_ChargeSwi;
			sgValvesInfo.u8NoLift |= 1 << NoAct_ChargeSwi;
		}
		else 
		{
			sgValvesInfo.u8NoAct &= ~(1 << NoAct_ChargeSwi);
			sgValvesInfo.u8NoLift &= ~(1 << NoAct_ChargeSwi);
		}
		
		if ((1 == SwiInput.b1FootBrake) || (1 == SwiInput.b1HandBrake))		//刹车
		{
			sgValvesInfo.u8NoAct |= 1 << NoAct_Brake;
		}
		else 
		{
			sgValvesInfo.u8NoAct &= ~(1 << NoAct_Brake);
		}
		
		if (sgUserInfo.b1DaocheDOEnable == 0)
		{
			if (1 == SwiInput.b1Backward)
			{
				i32DoPwmSet(DAOCHE_DO,DRIVER_OPEN);					//倒车蜂鸣器响
			}
			else 
			{
				i32DoPwmSet(DAOCHE_DO,DRIVER_CLOSE);
			}
		}
	
	if (0 == SwiInput.b1SafeLock)
	{
		if ((1 == SwiInput.b1Forward) || (1 == SwiInput.b1Backward) || (1 == SwiInput.b1Throttle))
		{
			sgValvesInfo.u8NoAct |= 1 << NoAct_Lock;
			i32ErrCodeSet(ACT_LOCK_ERR);
		}
	}
	
	if ((0 == SwiInput.b1Forward) && (0 == SwiInput.b1Backward) && (0 == SwiInput.b1Throttle))
	{
		sgValvesInfo.u8NoAct &= ~(1 << NoAct_Lock);
		sgValvesInfo.u8NoAct &= ~(1 << NoAct_Init);
				
		i32ErrCodeClr(ACT_INIT_ERR);
		i32ErrCodeClr(ACT_LOCK_ERR);
	}

//	if (0 != sgValvesInfo.u8NoAct)
//	{
//		sgValvesInfo.b1MoveFlag = 1;
//	}
//	else
//	{
//		sgValvesInfo.b1MoveFlag = 0;
//	}	
	
	/*添加逻辑*/
	sgSwiInput.u16data = SwiInput.u16data;   /**/
	if (0 == sgValvesInfo.b1NoActFlag)
	{
		/*Move Mode*/
		if ((1 == sgSwiInput.b1Forward) && (0 == sgSwiInput.b1Backward))
		{
			sgValvesInfo.b1ForWardStat = 1;
			sgValvesInfo.b1BackWardStat = 0;
			gCanSendPdoInfo.CanSend260Info.b1ForWardStat = 1;
		}
		else if ((0 == sgSwiInput.b1Forward) && (1 == sgSwiInput.b1Backward))
		{
			sgValvesInfo.b1ForWardStat = 0;
			sgValvesInfo.b1BackWardStat = 1;
			gCanSendPdoInfo.CanSend260Info.b1BackWardStat = 1;
		}
		else
		{
			sgValvesInfo.b1ForWardStat = 0;
			sgValvesInfo.b1BackWardStat = 0;
			gCanSendPdoInfo.CanSend260Info.b1ForWardStat = 0;
			gCanSendPdoInfo.CanSend260Info.b1BackWardStat = 0;
		}
	}
	else
	{
		sgValvesInfo.b1ForWardStat = 0;
		sgValvesInfo.b1BackWardStat = 0;
		gCanSendPdoInfo.CanSend260Info.b1ForWardStat = 0;
		gCanSendPdoInfo.CanSend260Info.b1BackWardStat = 0;
	}
}
	
static uint8_t u8SwiInitChcek(void)
{
	uint8_t u8Res = 0;
		
	if ((1 == i32LocalDiGet(FORWARD_SWI)) || (1 == i32LocalDiGet(BACKWARD_SWI))
		|| (1 == i32LocalDiGet(THROTTLE_SWI))) 	// /*前进 后退 加速使能*/
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
	xRevCallBackProc BMACanId17F = {.u32CanId = 0x17F,  .CallBack = vBMSCanId17FProc};
	//xRevCallBackProc DeviceCanId17F = {.u32CanId = 0x17F, {.u8DataCnt = 3, .u8Data0 = 0x22, .u8Data1 = 0x08, .u8Data2 = 0x50}, .CallBack = vDeviceCanId17FProc};
	vMstRevRegister(vMstRevProc);
	vMstSendRegister(vMstSendProc);

	vCanRevMsgRegister(&BMACanId17F);
//	vCanRevMsgRegister(&DeviceCanId17F);	
//	vDoPwmErrReg(vDoPwmErrCallBack);
//	vPropErrReg(vPropErrCallBack);
	vAiErrReg(vAiErrCallBack);
	
	memset(&sgValvesInfo, 0, sizeof(sgValvesInfo));
	
	sgUserInfo.u16Gear1Spd = i32GetPara(PARA_Gear1Spd);	/*1档速度*/
	sgUserInfo.u16Gear2Spd = i32GetPara(PARA_Gear2Spd);	/*2档速度*/
	sgUserInfo.u16Gear3Spd = i32GetPara(PARA_Gear3Spd);	/*3档速度*/
	sgUserInfo.u16Gear4Spd = i32GetPara(PARA_Gear4Spd);	/*4档速度*/
	
	sgUserInfo.u16ThrottleMin = i32GetPara(MOVE_THROTTLE_MIN) * 100;		/*油门死区值, uint 0.1V*/
	sgUserInfo.u16ThrottleMax = i32GetPara(MOVE_THROTTLE_MAX) * 100;		/*油门死区值最大 ,uint 0.1V*/
	sgUserInfo.u16ThrottleRange = (sgUserInfo.u16ThrottleMax - sgUserInfo.u16ThrottleMin) >> 1;
	sgUserInfo.u16ThrottleMid = i32GetPara(MOVE_THROTTLE_MID) * MOTOR_SPEED_RANGE / 100;
	sgUserInfo.u16ThrottleMidValue = (sgUserInfo.u16ThrottleMax + sgUserInfo.u16ThrottleMin) >> 1;
	
	sgUserInfo.u16SteerAnalogMin = i32GetPara(STEER_ANALOG_MIN) * 100;		//转向角度范围设置
	sgUserInfo.u16SteerAnalogMax = i32GetPara(STEER_ANALOG_MAX) * 100;
	sgUserInfo.u16SteerAnalogMid = i32GetPara(STEER_ANALOG_MID) * 100;

	sgUserInfo.SteerAngleDecSpd.u16StartAngle = i32GetPara(TURN_START_ANGLE) * 10;
	sgUserInfo.SteerAngleDecSpd.u16EndAngle = i32GetPara(TURN_END_ANGLE) * 10;
	sgUserInfo.SteerAngleDecSpd.u16StartAngleSpd = i32GetPara(START_ANGLE_SPD) * MOTOR_SPEED_RANGE / 100;
	sgUserInfo.SteerAngleDecSpd.u16EndAngleSpd = i32GetPara(END_ANGLE_SPD) * MOTOR_SPEED_RANGE / 100;
	sgUserInfo.SteerAngleDecSpd.fAgnleDecSpdFactor = (sgUserInfo.SteerAngleDecSpd.u16StartAngleSpd - sgUserInfo.SteerAngleDecSpd.u16EndAngleSpd) /   \
													 (sgUserInfo.SteerAngleDecSpd.u16EndAngle - sgUserInfo.SteerAngleDecSpd.u16StartAngle);
													 
	sgUserInfo.u16RatioOfTransmission = i32GetPara(PARA_RatioOfTransmission);
	sgUserInfo.u16MotorMaxSpd = i32GetPara(PARA_MotorMaxSpd);
	
	sgUserInfo.u8BatteryType = i32GetPara(BATTERY_TYPE);
	
	/*lilu 20230823 add user mode*/
	{
		u16Tmp = i32GetPara(USERINFO_MODE);
		sgUserInfo.b1HourConutMode = u16Tmp & 0x01;					/*bit0: 小时计计时模式*/
		sgUserInfo.b1LiftCheck = (u16Tmp >> 1) & 0x01;			/*bit1: 起升互锁*/
		sgUserInfo.b1SteerCheck = (u16Tmp >> 2) & 0x01;				/*bit2: 转向角度使能 */
		sgUserInfo.b1DaocheDOEnable = (u16Tmp >> 3) & 0x01;				/*bit3: 倒车蜂鸣器使能*/
		sgUserInfo.b1FengshanEnable = (u16Tmp >> 4) & 0x01;			/*bit4: 风扇使能 */	
	}	
		
	u32HourCount = u32HourCountRead();
		
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
//		sgUserInfo.b1PasswordFunc = sgSaveState.b1PasswordFunc;
		u16EepromRead(PARA_HourSetTime, &u16Tmp, 1);	
	}

//	//i32LogWrite(INFO, "******SaveState = 0x%x\r\n*********", sgSaveState.u16Data);
//	__disable_irq();
	gCanSendPdoInfo.CanSend260Info.u8HourCountL = u32HourCount & 0xFF;
	gCanSendPdoInfo.CanSend260Info.u8HourCountH = (u32HourCount >> 8) & 0xFF;
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
	if (1 == sgUserInfo.b1LiftCheck)
	{
		vCanIdLostReg(0x47F,1000,vCanLostProc);
	}
	if(LiBattery == sgUserInfo.u8BatteryType)
	{
		vCanIdLostReg(0x17F,5000,vCanLostProc);
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
			sgValvesInfo.u8NoAct |= 1 << NoAct_Init;
			i32ErrCodeSet(ACT_INIT_ERR);
		}
	}
	
	if(1 == u8EcuProcFlag)
	{
		vSwiMonitor();		
		vCanRevPdoProc();
		vAiMonitor();
		u8ErrCode = u8ErrCodeGet();
		if (0 != u8ErrCode)
		{
			gCanSendPdoInfo.CanSend260Info.u8ErrorMove = u8ErrCode;	//u8ErrSwitchArray[u8ErrCode];
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
			gCanSendPdoInfo.CanSend260Info.u8HourCountL = (u32HourCount / 10) & 0xFF;
			gCanSendPdoInfo.CanSend260Info.u8HourCountH = (u32HourCount / 10 >> 8) & 0xFF;
			__enable_irq();
		}
	}
	
	vWdgSetFun(WDG_USER_BIT);
}	

#endif

/*******************************************************************************
* Filename: UserStackerTruckProc.c	                                           *
* Description: 逻辑层的源文件，包含对应的回调函数	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/05/30    														   *
* Revision:	 														 		   *
*******************************************************************************/

#if (USER_TYPE == USER_GANGLI_PHZ15T_LIFT)

const static xPdoParameter  sgPdoPara = 
{
	.TpdoPara = {
		{.u8Flag = 1, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 1, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 1, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 1, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 100, .u16CanId = 0x47F},
		{.u8Flag = 1, .u8Type = 0x00, .u16Period = 100, .u16CanId = 0x360},
		{.u8Flag = 1, .u8Type = 0x00, .u16Period = 100, .u16CanId = 0x360},
		{.u8Flag = 1, .u8Type = 0x00, .u16Period = 100, .u16CanId = 0x360},
		{.u8Flag = 1, .u8Type = 0x00, .u16Period = 100, .u16CanId = 0x360},
		{.u8Flag = 1, .u8Type = 0x00, .u16Period = 100, .u16CanId = 0x360},
	},
	.RpdoPara = {
		{.b1Flag = 1, .b11CanRevId = 0x270},
		{.b1Flag = 0, .b11CanRevId = 0x17F},
		{.b1Flag = 0, .b11CanRevId = 0x27F},
		{.b1Flag = 0, .b11CanRevId = 0x37F},
		{.b1Flag = 0, .b11CanRevId = 0x17F},
		{.b1Flag = 0, .b11CanRevId = 0x17F},
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
#define	NoAct_StopLift		3	

typedef union
{
	uint16_t u16data;
	struct
	{
		uint16_t b1SafeLock: 1;
		uint16_t b1Shuju: 1;
		uint16_t b1Qingxie: 1;
		uint16_t b1Lift: 1;
		uint16_t b1Ceyi: 1;
		uint16_t b11Reserve0: 11;
	};
}xSwiInput;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t b1NoActFlag: 1;
		uint8_t b1MoveFlag: 1;
		uint8_t b1Gear1SpdFlag: 1;
		uint8_t b1Gear2SpdFlag: 1;
		uint8_t b1Gear3SpdFlag: 1;
		uint8_t b1Gear4SpdFlag: 1;
		uint8_t b1DaisuStat: 1;
		uint8_t b1BackWardStat: 1;
				
		uint8_t	b4Gear1Spd: 4;
		uint8_t	b4Gear2Spd: 4;
		
		uint8_t	b4Gear3Spd: 4;
		uint8_t	b4Gear4Spd: 4;
		
		uint8_t u8NoAct;
		uint8_t u8NoMove;
		
		uint8_t u8Reserve1;
		uint8_t u8Reserve2;
		uint8_t u8Reserve3;
	};
}xValvesInfo;

typedef struct
{
	uint16_t	u16Gear1Spd;
	uint16_t	u16Gear2Spd;
	uint16_t	u16Gear3Spd;
	uint16_t	u16Gear4Spd;
	
	float		fMoveSpdPer5msAccStep;
	float		fMoveSpdPer5msDecStep;
	
	uint8_t 	u8ThrottleType;
	uint16_t	u16ThrottleMin;
	uint16_t	u16ThrottleMax;
	uint16_t	u16ThrottleRange;
	uint16_t	u16ThrottleMid;
	uint16_t	u16ThrottleMidValue;
	
	uint16_t	u16MotorMaxSpd;
	
	uint8_t   b1DownDOEnable: 1;
	uint8_t   b7Reserve: 7;
		
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
		uint16_t b1HeightSpdLimit: 1;
		uint16_t b11Reserve: 15;
	};
}xSaveStateInfo;

static xSaveStateInfo sgSaveState;

static int16_t	i16MotorSpd = 0;
static uint16_t u16MotorVal = 0;
static uint16_t u16SpeedRate;
static xValvesInfo sgValvesInfo;
static xUserInfo sgUserInfo;
static xSwiInput sgSwiInput;
static uint32_t u32HourCount = 0;

static int16_t i16SteerAngle = 0;
static uint16_t DRIVER_FLAG = 0;
static uint16_t Batter_SOC = 0;
static uint16_t Rev_Speed;

/*PDO接收待完善*/
xCanRevPdoInfo gCanRevPdoInfo; 
//{
//	.CanRevInfo1.u8Soc = 100,					/*BmsSoc Default: 100*/
//};				
/*PDO发送待完善*/
xCanSendPdoInfo gCanSendPdoInfo;
//{
//	.CanSend33CInfo.u8ErrCodeList = 0x03,
//	.CanSend33CInfo.u8SoftVersionLowByte = 0x00,
//	.CanSend33CInfo.u8SoftVersionHighByte = 0x10,
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
	gCanSendPdoInfo.CanSend47FInfo.u8NoAct = sgValvesInfo.u8NoAct;
//	tmp = RevData->u8MotorTmp;
//	/*Motor Temp*/
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

/*发送给Mst的回调函数*/
static void vMstSendProc(xMstSendPara *SendData) 
{
	static uint8_t u8MoveSwitchFlag = 0;
	
	float fTmp = 0.0;
	xMstSendStat LastStatus;
	int16_t i16Spd = 0;
		
	LastStatus.u8Data = SendData->buf[2];
	SendData->buf[2] = 0;
		
	/*Move Mode*/
	if ((sgSwiInput.b1SafeLock == 1) || (sgValvesInfo.b1DaisuStat == 1))
	{
		SendData->b1ServoOn = 1;
	}
	else
	{
		SendData->b1ServoOn = 0;
	}
	
	if ((sgValvesInfo.b1BackWardStat == 1) || (sgValvesInfo.b1DaisuStat == 1))		//增加怠速
	{
			SendData->b1BackwardReq = 1;
			SendData->u8TargetHigh = u16MotorVal >> 8;
			SendData->u8TargetLow = u16MotorVal;
	}
	else 
	{
		SendData->b1BackwardReq = 0;
		SendData->u8TargetHigh = 0;
		SendData->u8TargetLow = 0;
	}
	
	{
		static uint32_t u32Cnt = 0;
		u32Cnt++;
		{			
			/*lilu 20230819 For Test*/
			{
				i32SetPara(PARA_ForwardValveCurrent, u16MotorVal);		/*Send Motor Value*/
				i32SetPara(PARA_BackValveCurrent, Rev_Speed);		/*Rev Motor Value*/
				i32SetPara(PARA_PropValveCurrent, u16SpeedRate);	/*Prop Current*/
//				i32SetPara(PARA_LiftValveCurrent, ((gCanSendPdoInfo.CanSend260Info.u8HourCountH >> 8)|(gCanSendPdoInfo.CanSend260Info.u8HourCountL)));		/*Send Pump Value*/
				i32SetPara(PARA_OnOffValveCurrent, SendData->buf[2]);			/*Send Status*/
				i32SetPara(PARA_LoadRate, sgValvesInfo.u8Data[0]);				/*Valve NoFlag*/
				i32SetPara(PARA_CalibrationStatus, sgValvesInfo.u8Data[1]);		/*Value State*/
				i32SetPara(PARA_TurnRightValveCurrent, ((uint16_t)sgValvesInfo.u8NoAct << 8) | ((uint16_t)sgValvesInfo.u8NoMove));
				i32SetPara(PARA_TurnLeftValveCurrent, ((uint16_t)sgValvesInfo.b4Gear1Spd << 12) | ((uint16_t)sgValvesInfo.b4Gear2Spd << 8) | \
					((uint16_t)sgValvesInfo.b4Gear3Spd << 4) | ((uint16_t)sgValvesInfo.b4Gear4Spd));
				i32SetPara(PARA_AngleValue, (uint16_t)i32LocalAiGetValue(AI_B_AI1_R));		/*AI1*/
				i32SetPara(PARA_PressureVlaue1, SendData->u8PumpTarget);	/*AI2*/
				i32SetPara(PARA_PressureVlaue2, DRIVER_FLAG);	/*AI3*/
				i32SetPara(PARA_ExtSignal, sgSwiInput.u16data);						/*Swi*/
				i32SetPara(PARA_PcuKeyInfo,sgValvesInfo.u8Data[0]);						/*ErrCode*/
//				i32SetPara(PARA_BmsSoc, gCanSendPdoInfo.CanSend23CInfo.u8Soc);		/*BMS SOC*/
				i32SetPara(PARA_BrakeValveCurrent, abs(i16SteerAngle));
				
			}
		}
	}
}	
	
static void vAiErrCallBack(eAiErrNo AiErrNo)
{
	switch((uint8_t)AiErrNo)
	{
		case AI_B_AI1_R_ERR:
			//u16MotorVal = 0;
			//i32ErrCodeSet(AI_B_AI1_ERR);
			break;
//		case AI_B_AI2_R_ERR:
//			u16MotorVal = 800;
//			//i32ErrCodeSet(AI_B_AI2_ERR);
//			break;
//		case AI_B_AI3_R_ERR:
//			i32ErrCodeSet(AI_B_AI3_ERR);
//			break;
//		case AI_5V_12V_OUT1_I_ERR:
//			i32ErrCodeSet(AI_5V_12V_OUT1_ERR);
//			break;
//		case AI_5V_12V_OUT2_I_ERR:
////			i32ErrCodeSet(AI_5V_12V_OUT2_ERR);
//			break;
		default:
			break;									
	}
}

/*接收PDO处理函数*/
static void vCanRevPdoProc(void)
{
	static xCanRev270Info CanRev270InfoLast;
	static uint16_t u16CanRev270Cnt;
	static uint16_t u16DaisuDelay;
/***** 行走   ********/
	{
			memcpy((char*)CanRev270InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo5.u8Data, sizeof(CanRev270InfoLast));
			/*添加相关的操作*/
		if (0 != CanRev270InfoLast.u8MoveFlag)		//怠速
		{
			sgValvesInfo.b1DaisuStat = 1;
			u16DaisuDelay = 0;
		}
		else 
		{
			if (u16DaisuDelay > 5000/USER_ECU_PERIOD)
			{
				sgValvesInfo.b1DaisuStat = 0;
			}
			else 
			{
			//	sgValvesInfo.b1DaisuStat = 1;
				u16DaisuDelay ++;
			}
		}
		if (0 != CanRev270InfoLast.u8StopLift)		//禁止起升
		{
			sgValvesInfo.u8NoAct |= 1 << NoAct_StopLift;
		}
		else 
		{
			sgValvesInfo.u8NoAct &= ~(1 << NoAct_StopLift);
		}
	}
}
	
/*lilu 20230703 模拟量监控*/
static void vAiMonitor(void)
{
	int32_t i32AdcValue = 0;
	u16SpeedRate = 0;
	if ((1 == sgSwiInput.b1Lift) && (0 == sgValvesInfo.b1NoActFlag))
	{
		if(0 != sgUserInfo.u8ThrottleType)
		{
			i32AdcValue = i32LocalAiGetValue(MOVE_THROTTLE);

			if (i32AdcValue > sgUserInfo.u16ThrottleMax)	/*DeadZone Max*/
			{
				u16SpeedRate = MAX_SPEED_RATE;
			}
			else if (i32AdcValue >= (sgUserInfo.u16ThrottleMidValue))		/*超过一半*/
			{
				u16SpeedRate = sgUserInfo.u16ThrottleMid + (i32AdcValue - sgUserInfo.u16ThrottleMidValue) * (MAX_SPEED_RATE - sgUserInfo.u16ThrottleMid) / sgUserInfo.u16ThrottleRange;
			}
			else if (i32AdcValue >= sgUserInfo.u16ThrottleMin)	/*DeadZone Min*/ 
			{
				u16SpeedRate = (i32AdcValue - sgUserInfo.u16ThrottleMin) * sgUserInfo.u16ThrottleMid / sgUserInfo.u16ThrottleRange;
			}
			else
			{
				u16SpeedRate = 0;
			}
		}
//		else 
//		{
//			u16MotorVal = MOTOR_MAX_SPEED_VALUE;
//		}
	}
	
	/*add spd limit*/
	
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
	
	if(1 == i32LocalDiGet(SAFELOCK_SWI))
	{
		SwiInput.b1SafeLock = 1;
	}
	
	if(1 == i32LocalDiGet(SHUJU_SWI))
	{
		SwiInput.b1Shuju = 1;
	}
	
	if(1 == i32LocalDiGet(QINGXIE_SWI))
	{
		SwiInput.b1Qingxie = 1;
	}
	
	if(1 == i32LocalDiGet(LIFT_SWI))
	{
		SwiInput.b1Lift = 1;
	}
	
	if(1 == i32LocalDiGet(CEYI_SWI))
	{
		SwiInput.b1Ceyi = 1;
	}
	
	/*add lock */
//	if ((1 == SwiInput.b1Shuju) || (1 == SwiInput.b1Qingxie) ||
//	   (1 == SwiInput.b1Lift) || (1 == SwiInput.b1Ceyi)
//	   )
//	{
//		/*禁止动作*/
//		if ((0 != gCanRevPdoInfo.CanRevInfo2.u8ErrSteer) || (0 == SwiInput.b1FaultLockOut))
//		{
//			sgValvesInfo.b4NoLiftUp |= 1 << NoLiftUp_FaultLock;
//			sgValvesInfo.b4NoMove |= 1 << NoMove_FaultLock;
//			i32ErrCodeSet(ACT_FAULT_LOCK_ERR);
//		}
//	}	
	
	//*添加逻辑
	sgSwiInput.u16data = SwiInput.u16data;
	
	if (sgSwiInput.b1SafeLock == 0)
	{
		if (sgSwiInput.b1Qingxie == 1 || sgSwiInput.b1Lift == 1 || sgSwiInput.b1Ceyi == 1 || sgSwiInput.b1Shuju == 1)
		{
			sgValvesInfo.u8NoAct |= 1 << NoAct_Lock;
			i32ErrCodeSet(ACT_LOCK_ERR);
		}
	}
	
	if (0 == sgUserInfo.b1DownDOEnable)
	{
		if (sgSwiInput.b1SafeLock == 1)
		{
			i32DoPwmSet(DOWN_DO,DRIVER_OPEN);
		}
		else 
		{
			i32DoPwmSet(DOWN_DO,DRIVER_CLOSE);
		}
	}
	

	if (0 == sgValvesInfo.b1NoActFlag)
	{
		if ((1 == sgSwiInput.b1Lift) && (0 == sgUserInfo.u8ThrottleType))
		{
			if ((u16SpeedRate == 0) || (u16SpeedRate < sgUserInfo.u16Gear1Spd))
			{
				u16SpeedRate = sgUserInfo.u16Gear1Spd;
			}
		}
		if (1 == sgSwiInput.b1Ceyi)
		{
			if ((u16SpeedRate == 0) || (u16SpeedRate < sgUserInfo.u16Gear3Spd))
			{
				u16SpeedRate = sgUserInfo.u16Gear3Spd;
			}
		}
		if (1 == sgSwiInput.b1Shuju)
		{
			if ((u16SpeedRate == 0) || (u16SpeedRate < sgUserInfo.u16Gear4Spd))
			{
				u16SpeedRate = sgUserInfo.u16Gear4Spd;
			}
		}
		if (1 == sgSwiInput.b1Qingxie)
		{
			if ((u16SpeedRate == 0) || (u16SpeedRate < sgUserInfo.u16Gear2Spd))
			{
				u16SpeedRate = sgUserInfo.u16Gear2Spd;
			}
		}		
	}
	
	if (sgValvesInfo.b1DaisuStat == 1)
	{
		if ((u16SpeedRate == 0) || (u16SpeedRate < sgUserInfo.u16Gear4Spd))
		{
			u16SpeedRate = sgUserInfo.u16Gear4Spd;
		}		
	}
	
	u16MotorVal = MOTOR_MAX_SPEED_VALUE * u16SpeedRate / 100.;
	
	if ((0 == sgSwiInput.b1Shuju) && (0 == sgSwiInput.b1Qingxie)&&(0 == sgSwiInput.b1Lift) && (0 == sgSwiInput.b1Ceyi))
	{
		sgValvesInfo.u8NoAct &= ~(1 << NoAct_Lock);
		sgValvesInfo.u8NoAct &= ~(1 << NoAct_Init);
		
		i32ErrCodeClr(ACT_INIT_ERR);
		i32ErrCodeClr(ACT_LOCK_ERR);
	}

	/*添加逻辑*/
	if (0 == sgValvesInfo.b1NoActFlag)			//增加怠速
	{
		if (sgSwiInput.b1SafeLock == 1)
		{
			/*Move Mode*/
			if ((1 == sgSwiInput.b1Lift) || (1 == sgSwiInput.b1Qingxie) || 
				(1 == sgSwiInput.b1Ceyi) || (1 == sgSwiInput.b1Shuju))		//增加怠速
			{
				sgValvesInfo.b1BackWardStat = 1;
			}
			else
			{
				sgValvesInfo.b1BackWardStat = 0;
			}
		}
		else 
		{
			sgValvesInfo.b1BackWardStat = 0;
		}
	}
}
	
static uint8_t u8SwiInitChcek(void)
{
	uint8_t u8Res = 0;
		
	if ((1 == i32LocalDiGet(SHUJU_SWI)) || (1 == i32LocalDiGet(QINGXIE_SWI)) || 
		(1 == i32LocalDiGet(LIFT_SWI)) || (1 == i32LocalDiGet(CEYI_SWI))) 	// 
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
//	vPropErrReg(vPropErrCallBack);
	vAiErrReg(vAiErrCallBack);
	
	memset(&sgValvesInfo, 0, sizeof(sgValvesInfo));
	
	sgUserInfo.u16Gear1Spd = i32GetPara(PARA_Gear1Spd);	/*1档速度*/
	sgUserInfo.u16Gear2Spd = i32GetPara(PARA_Gear2Spd);	/*2档速度*/
	sgUserInfo.u16Gear3Spd = i32GetPara(PARA_Gear3Spd);	/*3档速度*/
	sgUserInfo.u16Gear4Spd = i32GetPara(PARA_Gear4Spd);	/*4档速度*/
	
	sgUserInfo.u8ThrottleType = i32GetPara(MOVE_THROTTLE_TYPE);					/*加速器类型*/
	sgUserInfo.u16ThrottleMin = i32GetPara(MOVE_THROTTLE_MIN) * 100;		/*油门死区值, uint 0.1V*/
	sgUserInfo.u16ThrottleMax = i32GetPara(MOVE_THROTTLE_MAX) * 100;		/*油门死区值最大 ,uint 0.1V*/
	sgUserInfo.u16ThrottleRange = (sgUserInfo.u16ThrottleMax - sgUserInfo.u16ThrottleMin) >> 1;
	sgUserInfo.u16ThrottleMid = i32GetPara(MOVE_THROTTLE_MID) * MAX_SPEED_RATE / 100;
	sgUserInfo.u16ThrottleMidValue = (sgUserInfo.u16ThrottleMax + sgUserInfo.u16ThrottleMin) >> 1;

	sgUserInfo.u16MotorMaxSpd = i32GetPara(PARA_MotorMaxSpd);

	{
		u16Tmp = i32GetPara(USERINFO_MODE);
		sgUserInfo.b1DownDOEnable = u16Tmp & 0x01;					/*bit0: 下降电磁阀使能*/
//		sgUserInfo.b1LiftCheck = (u16Tmp >> 1) & 0x01;			/*bit1: 起升互锁*/
//		sgUserInfo.b1SteerCheck = (u16Tmp >> 2) & 0x01;				/*bit2: 转向角度使能 */
//		sgUserInfo.b1DaocheDOEnable = (u16Tmp >> 3) & 0x01;				/*bit3: 倒车蜂鸣器使能*/
//		sgUserInfo.b1FengshanEnable = (u16Tmp >> 4) & 0x01;			/*bit4: 风扇使能 */	
	}	
//	//i32LogWrite(INFO, "******DefaultFlag = 0x%x\r\n*********", u16Tmp);
//	if (0x5555 != u16Tmp)
//	{
//		u16EepromWrite(PARA_SaveState, sgSaveState.u16Data, 1);
//		u16EepromWrite(PARA_HourSetTime, 0x0000, 1);
//		u16EepromWrite(PARA_DefaultFlag, 0x5555, 1);	
//	}
//	else
//	{
//		u16EepromRead(PARA_SaveState, &sgSaveState.u16Data, 1);
////		sgSwiInput.b1HeightLimit = sgSaveState.b1Above1M8;
////		sgFlagVal.Height_SpeedLimit = sgSaveState.b1HeightSpdLimit;        //高度限速保存到eeprom
////		sgUserInfo.b1PasswordFunc = sgSaveState.b1PasswordFunc;
////		u16EepromRead(PARA_HourSetTime, &u16Tmp, 1);	
//	}
	
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
			sgValvesInfo.u8NoAct |= 1 << NoAct_Init;
			i32ErrCodeSet(ACT_INIT_ERR);
		}
	}
	
	if(1 == u8EcuProcFlag)
	{
		vSwiMonitor();		
		vCanRevPdoProc();
		vAiMonitor();
		u8ErrCode = u8ErrCodeGet();
		if (0 != u8ErrCode)
		{
			gCanSendPdoInfo.CanSend47FInfo.u8ErrorLift = u8ErrCode;	//u8ErrSwitchArray[u8ErrCode];
			vLedSendAlmCode(u8ErrCode);
		}
		else
		{
			gCanSendPdoInfo.CanSend47FInfo.u8ErrorLift = 0;
		}
	}
	
	vWdgSetFun(WDG_USER_BIT);
}	

#endif //(USER_ECU_OR_ET_TYPE == USER_GANGLI_PHZ15T_MOVE)
