/*******************************************************************************
* Filename: UserStackerTruckProc.c	                                           *
* Description: 逻辑层的源文件，包含对应的回调函数	   						   *
* Author:                                                      *
* Date: 2023/05/30    														   *
* Revision:	 														 		   *
*******************************************************************************/
#include "UserCanComm.h"
#include "User_HANGCHA_GAOJI.h"
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


#if (USER_TYPE == USER_HANGCHA_GAOJI)
		

const static xPdoParameter  sgPdoPara = 
{
	.TpdoPara = {
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 10},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
	},
	.RpdoPara = {
		{.b1Flag = 0, .b11CanRevId = 0x000},
		{.b1Flag = 0, .b11CanRevId = 0x000},
		{.b1Flag = 0, .b11CanRevId = 0x000},
	},
};

typedef union
{
	uint8_t u8data;
	struct
	{
		uint8_t b1ZUOYI: 1;
		uint8_t b1JIAOSHA: 1; 
		uint8_t b1THROTTLE: 1;
		uint8_t b1SHOUSHA: 1;
		uint8_t b1FORWARD:1;
		uint8_t b1BACKWARD: 1;
		uint8_t b2Reserve: 2;
	};
}xSwiInput;

typedef struct
{
	uint16_t	u16StartAngle;
	uint16_t	u16EndAngle;
	uint16_t	u16StartAngleSpd;
	uint16_t	u16EndAngleSpd;
	float		fAgnleDecSpdFactor;
}xSteerAngleDecSpd;

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
	uint16_t	u16Gear1Spd;
	uint16_t	u16Gear2Spd;
	uint16_t	u16Gear3Spd;
	uint16_t	u16Gear4Spd;
	
	float		fPropMinCurrent;
	float		fPropMaxCurrent;
	
	uint16_t	u16ThrottleMin;
	uint16_t	u16ThrottleMax;
	uint16_t	u16ThrottleRangeL;
	uint16_t	u16ThrottleRangeH;
	uint16_t	u16ThrottleMid;
	
	uint16_t 	u16BrakePedalType;
	uint16_t	u16BrakePedalMin;
	uint16_t	u16BrakePedalMax;
	uint16_t	u16BrakePedalRangeL;
	uint16_t	u16BrakePedalRangeH;
	uint16_t	u16BrakePedalMid;
	
	xSteerAngleDecSpd SteerAngleDecSpd;

	uint8_t		b1LiftMode: 1;
	uint8_t		b1RentalStop: 1;
	uint8_t		b1RentalStart: 1;
	uint8_t		b1RentalMode: 1;
	uint8_t		b1PasswordFunc: 1;
	uint8_t		b3Reserve: 3;	
	
	uint8_t		u8BatteryType;
	
	uint8_t		b1HourConutMode: 1;		/*0: 上电计时， 1：互锁计时*/
	uint8_t		b1HourConutClr: 1;		/*0： 不清楚, 1：清除*/
	uint8_t		b1LiftLock: 1;			/*0： 需要互锁， 1：不需要*/
	uint8_t		b1LiftPedal: 1;			/*0： 无要求， 1：踏板open =1， close =0 / open =0， close =1 才可以动作*/
	uint8_t		b1MoveLiftMode: 1;		/*0： 同时动作， 1：互斥*/
	uint8_t		b3Reserve1: 3;
	
	uint16_t	u16RatioOfTransmission;
	
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


static int16_t	i16MotorSpd = 0;
static uint16_t u16MotorVal = 0;
static uint16_t u16StreeVal = 0;
static uint8_t	u8PumpOrPropValue = 0;
static xValvesInfo sgValvesInfo;
static xUserInfo sgUserInfo;
static xSwiInput sgSwiInput;
static uint32_t u32HourCount = 0;
static int16_t i16SteerAngle = 0;
static uint8_t SysSoc = 50;
int16_t steerAngle10 = 0;

/*PDO接收待完善*/
xCanRevPdoInfo gCanRevPdoInfo;
/*PDO发送待完善*/
xCanSendPdoInfo gCanSendPdoInfo;


const static uint8_t u8ErrSwitchArray[ErrCodeMax] = 
{	
/*    0    1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18   19*/
		  0,   1,   2,   3,   4,   5,   6,   7,   8,   9,   10,  11,  12,  13,  14,  15,  16,  17,  18,  19,
     20,  21,  22,  23,  24,  25,  26,  27,  28,  29,   30,  31,  32,  33,  34,  35,  36,  37,  38,  39,
     40,  41,  42,  43,  44,  45,  46,  47,  48,  49,   50,  51,  52,  53,  54,  55,  56,  57,  58,  59,
	   60,  61,  62,  63,  64,  65,  66,  67,  68,  69,   70,  71,  72,  73,  74,  75,  76,  77,  78,  79, 
		 80,  81,  82,  83,  84,  85,  86,  87,  88,  89,   90,  91,  92,  93,  94,  95,  96,  97,  98,  99,
		100, 101, 102, 103, 104, 105, 106, 107, 108, 109,  110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 
		120, 121, 122, 123, 124, 125, 126, 127, 128, 129,  130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 
		140, 141, 142, 143, 144, 145, 146, 147, 148, 149,  150, 151, 152, 153, 154, 155, 156, 157, 158, 159,
		160, 161, 162, 163, 164, 165, 166, 167, 168, 169,  170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 
		180, 181, 182, 183, 184, 185, 186, 187, 188, 189,  190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 
		200, 201, 202, 203, 204, 205, 206, 207, 208, 209,  210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 
		220, 221, 222, 223, 224, 225, 226, 227, 228, 229,  230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 
		240, 241, 242, 243, 244, 245, 246, 247, 248, 249,  250, 251, 252, 253, 254,
};

static uint8_t SteerAngle2Speedrate[10]=
	// 00,  01,  02,  03,  04,  05, 06,   07,  08,  09,
{
	 100, 100, 100, 100, 99,  98, 97,   96,  94,	93,	/* 0~9*/
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
	__disable_irq();
	tmp = abs(i16MotorSpd / sgUserInfo.u16RatioOfTransmission);
	gCanSendPdoInfo.CanSend260Info.u8Movespeed = tmp;
	__enable_irq();
	
	tmp = RevData->u8MotorTmp;
	/*Motor Temp*/
	
	tmp = RevData->u8BoardTmp;
	/**/

	
	__disable_irq();
//	gCanSendPdoInfo.CanSend43CInfo.i16MotorCurrent = (RevData->u8CurrentHigh << 8) | RevData->u8CurrentLow;
	__enable_irq();
}

/*发送给Mst的回调函数*/
static void vMstSendProc(xMstSendPara *SendData) 
{
	xMstSendStat LastStatus;
	static uint16_t sysEnableDelay;
	int16_t StreeTmp = 0;
	
		
	LastStatus.u8Data = SendData->buf[2];
	SendData->buf[2] = 0;
	SendData->u8TargetHigh = 0;
	SendData->u8TargetLow = 0;
	
	/*Move Mode*/
	if(sgSwiInput.b1BACKWARD == 1)
	{
		gCanSendPdoInfo.CanSend260Info.u8MoveState &= ~FORWARD_MOVESTATE2HMI;
		gCanSendPdoInfo.CanSend260Info.u8MoveState &= ~PARK_MOVESTATE2HMI;
		gCanSendPdoInfo.CanSend260Info.u8MoveState |= REVERSE_MOVESTATE2HMI;
	}
	else if(sgSwiInput.b1FORWARD == 1)
	{
		gCanSendPdoInfo.CanSend260Info.u8MoveState &= ~REVERSE_MOVESTATE2HMI;
		gCanSendPdoInfo.CanSend260Info.u8MoveState &= ~PARK_MOVESTATE2HMI;
		gCanSendPdoInfo.CanSend260Info.u8MoveState |= FORWARD_MOVESTATE2HMI;
	}
	else
	{
		gCanSendPdoInfo.CanSend260Info.u8MoveState &= ~REVERSE_MOVESTATE2HMI;
		gCanSendPdoInfo.CanSend260Info.u8MoveState &= ~FORWARD_MOVESTATE2HMI;
		gCanSendPdoInfo.CanSend260Info.u8MoveState |= PARK_MOVESTATE2HMI;
	}
	
		if((0 != sgSwiInput.b1BACKWARD)&&(0 == sgSwiInput.b1FORWARD)
			)
		{
			SendData->b1ServoOn = 1;
			SendData->b1BackwardReq = 1;
		}
		else if((0 != sgSwiInput.b1FORWARD)&&(0 == sgSwiInput.b1BACKWARD)
			)
		{
			SendData->b1ServoOn = 1;
			SendData->b1ForwardReq = 1;
		}	

		if(((0 != sgSwiInput.b1BACKWARD)||(0 != sgSwiInput.b1FORWARD)))
		{
			SendData->u8TargetHigh = u16MotorVal >> 8;
			SendData->u8TargetLow = u16MotorVal;
		}
		
		if((0 == sgSwiInput.b1SHOUSHA)&&(0 != sgSwiInput.b1FORWARD))
		{
			SendData->b1ServoOn = 0;
			SendData->b1ForwardReq = 0;
			SendData->u8TargetHigh = 0;
			SendData->u8TargetLow = 0;
		}
		
		if((0 == sgSwiInput.b1JIAOSHA)&&(0 != sgSwiInput.b1BACKWARD))
		{
			SendData->b1ServoOn = 0;
			SendData->b1BackwardReq = 0;
			SendData->u8TargetHigh = 0;
			SendData->u8TargetLow = 0;
		}
		
		if((0 != sgValvesInfo.u8NoAct)
			)
		{
			SendData->b1ServoOn = 0;
			SendData->b1BackwardReq = 0;
			SendData->b1ForwardReq = 0;
			SendData->u8TargetHigh = 0;
			SendData->u8TargetLow = 0;
		}
	
		{			
			/*lilu 20230819 For Test*/
			{
				i32SetPara(PARA_ForwardValveCurrent, gCanSendPdoInfo.CanSend260Info.u8Movespeed);		/*Send Motor Value*/
//				i32SetPara(PARA_BackValveCurrent, Current_Rev);		/*Rev Motor Value*/
				i32SetPara(PARA_PropValveCurrent, gCanSendPdoInfo.CanSend260Info.u8Data[5]);	/*Prop Current*/
				i32SetPara(PARA_LiftValveCurrent, ((gCanSendPdoInfo.CanSend260Info.u8HourCountH >> 8)|(gCanSendPdoInfo.CanSend260Info.u8HourCountL)));		/*Send Pump Value*/
				i32SetPara(PARA_OnOffValveCurrent, SendData->buf[2]);			/*Send Status*/
				i32SetPara(PARA_LoadRate, sgValvesInfo.u8Data[0]);				/*Valve NoFlag*/
				i32SetPara(PARA_CalibrationStatus, sgValvesInfo.u8Data[1]);		/*Value State*/
				i32SetPara(PARA_TurnRightValveCurrent, ((uint16_t)sgValvesInfo.u8NoAct << 8) | ((uint16_t)sgValvesInfo.b4NoMove << 4) | ((uint16_t)sgValvesInfo.b4NoLiftUp));
				i32SetPara(PARA_TurnLeftValveCurrent, ((uint16_t)sgValvesInfo.b4Gear1Spd << 12) | ((uint16_t)sgValvesInfo.b4Gear2Spd << 8) | \
					((uint16_t)sgValvesInfo.b4Gear3Spd << 4) | ((uint16_t)sgValvesInfo.b4Gear4Spd));
				i32SetPara(PARA_AngleValue, (uint16_t)i32LocalAiGetValue(AI_B_AI1_R));		/*AI1*/
				i32SetPara(PARA_PressureVlaue1, SendData->u8PumpTarget);	/*AI2*/
				i32SetPara(PARA_PressureVlaue2, u16MotorVal);	/*AI3*/
				i32SetPara(PARA_ExtSignal, sgSwiInput.u8data);						/*Swi*/
				i32SetPara(PARA_PcuKeyInfo,sgValvesInfo.u8Data[0]);						/*ErrCode*/
//				i32SetPara(PARA_BmsSoc, gCanSendPdoInfo.CanSend23CInfo.u8Soc);		/*BMS SOC*/
//				i32SetPara(PARA_BrakeValveCurrent, abs(gCanSendPdoInfo.CanSend1ACInfo.u8StreeValL|(gCanSendPdoInfo.CanSend1ACInfo.u8StreeValH << 8)));
				
			}
		}
}

static void vDoPwmErrCallBack(eDoPwmNo DoPwmNo)
{
}

static void vPropErrCallBack(uint8_t u8Channel)
{
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

/*******************************************************************************
* Name: vCanLostProc(uint32_t u32CanID,uint8_t u8State)
* Descriptio: 正常通道ID掉线检测
* Input: NULL
* Output: NULL  
*******************************************************************************/
void vCanLostProc(uint32_t u32CanID,uint8_t u8State)
{
}


/*接收PDO处理函数*/
static void vCanRevPdoProc(void)
{
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
		u16Res = u16Spd * Info->u16EndAngleSpd / 100;

	}
	else
	{
		u16Tmp = Info->u16StartAngleSpd - (u16Angle - Info->u16StartAngle) * Info->fAgnleDecSpdFactor;
		u16Res = u16Tmp;
	}
	
	return u16Res;
}


/*lilu 20230703 模拟量监控*/
static void vAiMonitor(void)
{
	uint8_t u8SpeedRate = 100;
	int32_t i32AdcValue = 0;
	uint8_t Tmp = 0;
	uint8_t Tmp1 = 0;
	
/*********** Motor Speed *************/
//	/*add Turn Dec Spd*/
//	i32AdcValue = i32LocalAiGetValue(MOVE_THROTTLE); //踏板滑动端电压值（1mV）
//	if(sgUserInfo.u16ThrottleMax >= sgUserInfo.u16ThrottleMin)	//正斜率
//	{
//		if(i32AdcValue < sgUserInfo.u16ThrottleMin)
//		{
//			u16MotorVal = 0;
//		}
//		else if((i32AdcValue >= sgUserInfo.u16ThrottleMin)&&(i32AdcValue < sgUserInfo.u16ThrottleMid))
//		{
//			u16MotorVal = ((((i32AdcValue-sgUserInfo.u16ThrottleMin)*sgUserInfo.u16ThrottleRangeL)/_IQ8(1))*4095)/100;
//		}
//		else if((i32AdcValue>=sgUserInfo.u16ThrottleMid)&&(i32AdcValue<sgUserInfo.u16ThrottleMax))
//		{
//			u16MotorVal = ((i32GetPara(MOVE_THROTTLE_MID)+((i32AdcValue-sgUserInfo.u16ThrottleMid)*sgUserInfo.u16ThrottleRangeH)/_IQ8(1))*4095)/100;
//		}
//		else if(i32AdcValue>=sgUserInfo.u16ThrottleMax)
//		{
//			u16MotorVal=4095;
//		}
//	}
//	else	//负斜率
//	{
//		if(i32AdcValue>sgUserInfo.u16ThrottleMin)
//		{
//			u16MotorVal=0;
//		}
//		else if((i32AdcValue<=sgUserInfo.u16ThrottleMin)&&(i32AdcValue>sgUserInfo.u16ThrottleMid))
//		{
//			u16MotorVal=((((i32AdcValue-sgUserInfo.u16ThrottleMin)*sgUserInfo.u16ThrottleRangeL)/_IQ8(1))*4095)/100;
//		}
//		else if((i32AdcValue<=sgUserInfo.u16ThrottleMid)&&(i32AdcValue>sgUserInfo.u16ThrottleMax))
//		{
//			u16MotorVal=((i32GetPara(MOVE_THROTTLE_MID)+((i32AdcValue-sgUserInfo.u16ThrottleMid)*sgUserInfo.u16ThrottleRangeH)/_IQ8(1))*4095)/100;
//		}
//		else if(i32AdcValue<=sgUserInfo.u16ThrottleMax)
//		{
//			u16MotorVal=4095;
//		}
//	}

	u16MotorVal=4095;
}


/*开关量监控*/
static void vSwiMonitor(void)
{
	xSwiInput SwiInput = {0};
	static uint8_t Snail_Count = 0;
	static uint8_t BMS_SocDelay = 0;
	
	if(1 == i32LocalDiGet(SHOUSHA_SWI))
	{
		SwiInput.b1SHOUSHA = 1;
	}
	
	if(1 == i32LocalDiGet(FORWARD_SWI))
	{
		SwiInput.b1FORWARD = 1;
	}
	
	if(1 == i32LocalDiGet(BACKWARD_SWI))
	{
		SwiInput.b1BACKWARD = 1;
	}
	
	if(1 == i32LocalDiGet(JIAOSHA_SWI))
	{
		SwiInput.b1JIAOSHA = 1;
	}
	
	sgSwiInput.u8data = SwiInput.u8data;
	
	if((0 == i32LocalDiGet(FORWARD_SWI))
		&&(0 == i32LocalDiGet(BACKWARD_SWI))
			)
	{
		sgValvesInfo.u8NoAct &= ~POWERON_CHECK_LIMIT;
		i32ErrCodeClr(POWERON_CHECK_ERR);
	}	
}


static uint8_t u8SwiInitChcek(void)
{
	uint8_t u8Res = 0;
		
	if((1 == i32LocalDiGet(FORWARD_SWI))
		||(1 == i32LocalDiGet(BACKWARD_SWI))
			) 
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

	vDoPwmErrReg(vDoPwmErrCallBack);
	vPropErrReg(vPropErrCallBack);
	vAiErrReg(vAiErrCallBack);
	
	memset(&sgValvesInfo, 0, sizeof(sgValvesInfo));
	
	sgUserInfo.fPropMinCurrent = i32GetPara(PROP_MIN_CURRENT) / 1000.0;		/*比例阀最小电流*/
	sgUserInfo.fPropMaxCurrent = i32GetPara(PROP_MAX_CURRENT) / 1000.0;		/*比例阀最大电流*/
	
	sgUserInfo.u16Gear1Spd = i32GetPara(PARA_Gear1Spd);	/*1档速度*/
	sgUserInfo.u16Gear2Spd = i32GetPara(PARA_Gear2Spd);	/*2档速度*/
	sgUserInfo.u16Gear3Spd = i32GetPara(PARA_Gear3Spd);	/*3档速度*/
	sgUserInfo.u16Gear4Spd = i32GetPara(PARA_Gear4Spd);	/*4档速度*/
	
	sgUserInfo.u16ThrottleMin = i32GetPara(MOVE_THROTTLE_MIN) * 100;		/*油门死区值, uint 0.1V*/
	sgUserInfo.u16ThrottleMax = i32GetPara(MOVE_THROTTLE_MAX) * 100;		/*油门死区值最大 ,uint 0.1V*/
	sgUserInfo.u16ThrottleMid = (sgUserInfo.u16ThrottleMax + sgUserInfo.u16ThrottleMin)/2;
	sgUserInfo.u16ThrottleRangeL = _IQ8(i32GetPara(MOVE_THROTTLE_MID))/(sgUserInfo.u16ThrottleMid - sgUserInfo.u16ThrottleMin);
	sgUserInfo.u16ThrottleRangeH = _IQ8(100 - i32GetPara(MOVE_THROTTLE_MID))/(sgUserInfo.u16ThrottleMax - sgUserInfo.u16ThrottleMid);
	
	sgUserInfo.u16BrakePedalType = i32GetPara(BRAKE_THROTTLE_TYPE);
	sgUserInfo.u16BrakePedalMin = i32GetPara(BRAKE_THROTTLE_MIN) * 100;		/*油门死区值, uint 0.1V*/
	sgUserInfo.u16BrakePedalMax = i32GetPara(BRAKE_THROTTLE_MAX) * 100;		/*油门死区值最大 ,uint 0.1V*/
	sgUserInfo.u16BrakePedalMid = (sgUserInfo.u16ThrottleMax + sgUserInfo.u16ThrottleMin)/2;
	sgUserInfo.u16BrakePedalRangeL = _IQ8(i32GetPara(BRAKE_THROTTLE_MID))/(sgUserInfo.u16ThrottleMid - sgUserInfo.u16ThrottleMin);
	sgUserInfo.u16BrakePedalRangeH = _IQ8(100 - i32GetPara(BRAKE_THROTTLE_MID))/(sgUserInfo.u16ThrottleMax - sgUserInfo.u16ThrottleMid);
	
	sgUserInfo.SteerAngleDecSpd.u16StartAngle = i32GetPara(TURN_START_ANGLE) * 10;
	sgUserInfo.SteerAngleDecSpd.u16EndAngle = i32GetPara(TURN_END_ANGLE) * 10;
	sgUserInfo.SteerAngleDecSpd.u16StartAngleSpd = i32GetPara(START_ANGLE_SPD);
	sgUserInfo.SteerAngleDecSpd.u16EndAngleSpd = i32GetPara(END_ANGLE_SPD);
													 
	sgUserInfo.u16RatioOfTransmission = i32GetPara(PARA_RatioOfTransmission);
	
	sgUserInfo.u8BatteryType = i32GetPara(PARA_BatteryType);
	
	SteerAngle2Speedrate[0] = i32GetPara(PARA_CurveFastDrive);
	SteerAngle2Speedrate[1] = i32GetPara(PARA_CurveSlowDrive);
	SteerAngle2Speedrate[2] = i32GetPara(PARA_CurveDriveAfterLift);
	SteerAngle2Speedrate[3] = i32GetPara(PARA_CurveLift);
	SteerAngle2Speedrate[4] = i32GetPara(PARA_CurveLower);
	SteerAngle2Speedrate[5] = i32GetPara(PARA_CurveTurn);
	SteerAngle2Speedrate[6] = i32GetPara(PARA_AccAndDecFastDrive);
	SteerAngle2Speedrate[7] = i32GetPara(PARA_AccAndDecSlowDrive);
	SteerAngle2Speedrate[8] = i32GetPara(PARA_AccAndDecAfterLift);
	SteerAngle2Speedrate[9] = i32GetPara(PARA_AccAndDecLift);
	
	/*lilu 20230823 add user mode*/
	{
		u16Tmp = i32GetPara(USERINFO_MODE);
		sgUserInfo.b1HourConutMode = u16Tmp & 0x01;					/*bit0: HourCount Mode*/
		sgUserInfo.b1HourConutClr = (u16Tmp >> 1) & 0x01;			/*bit1: StartUpCheck*/
		sgUserInfo.b1LiftLock = (u16Tmp >> 2) & 0x01;				/*bit2: Lift by Lock */
		sgUserInfo.b1LiftPedal = (u16Tmp >> 3) & 0x01;				/*bit3: Lift by Peadl*/
		sgUserInfo.b1MoveLiftMode = (u16Tmp >> 4) & 0x01;			/*bit4: Move and Lift */
		
	}
	
	u32HourCount = u32HourCountRead();
	
	gCanSendPdoInfo.CanSend260Info.u8HourCountL = u32HourCount & 0xFF;
	gCanSendPdoInfo.CanSend260Info.u8HourCountH = (u32HourCount >> 8) & 0xFF;


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

/*上电前检测*/
	if(true == u8GetNetTimerOverFlag(TIMER_EcuPowerOn))
	{
		u8EcuProcFlag = 1;
		vKillNetTimer(TIMER_EcuPowerOn);
	}
	
	if ((false == u8GetNetTimerOverFlag(TIMER_EcuPowerOn)) && (true == u8GetNetTimerStartFlag(TIMER_EcuPowerOn)))
	{
		if (1 == u8SwiInitChcek())
		{
			sgValvesInfo.u8NoAct |= POWERON_CHECK_LIMIT;
			i32ErrCodeSet(POWERON_CHECK_ERR);
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
			if (1 == sgSwiInput.b1ZUOYI)
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
	}
	
	if(0 != sgUserInfo.b1HourConutClr)  //清空小时计
	{
		sgUserInfo.b1HourConutClr = 0;
		u16SecCnt = 0;
		u32HourCount = 0;
		vHourCountWrite(u32HourCount);
		gCanSendPdoInfo.CanSend260Info.u8HourCountL = u32HourCount & 0xFF;
		gCanSendPdoInfo.CanSend260Info.u8HourCountH = (u32HourCount >> 8) & 0xFF;
	}
	
	vWdgSetFun(WDG_USER_BIT);
}
#endif


