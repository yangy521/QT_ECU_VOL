/*******************************************************************************
* Filename: USER_GANGLI_PZDGC.c	                                           *
* Description: 逻辑层的源文件，包含对应的回调函数	   						   *
* Author:  Young														   *
* Date: 2024/05/15 														   *
* Revision:	 														 		   *
*******************************************************************************/
#include "UserCanComm.h"
#include "USER_GANGLI_PZDGC.h"
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
#include "UserEcuProc.h"
#include "MstSlvCom.h"

#if (USER_TYPE == USER_GANGLI_PZDGC)



#define NoLiftUp_HeightLimit	(1 << 0)		
#define NoLiftUp_FaultLock		(1 << 1)
#define	NoLiftUp_LowBat			(1 << 2)
#define NoLiftUp_Init			(1 << 3)

const static xPdoParameter  PdoPara = 
{
	.TpdoPara = {
		{.u8Flag = 1, .u8Type = 0x0, .u16Period = 200},
		{.u8Flag = 1, .u8Type = 0x0, .u16Period = 200},
		{.u8Flag = 1, .u8Type = 0x0, .u16Period = 200},
		{.u8Flag = 1, .u8Type = 0x0, .u16Period = 200},

		{.u8Flag = 1, .u8Type = 0, .u16Period = 100, .u16CanId = 260},
	},
	.RpdoPara = {
		{.b1Flag = 0, .b11CanRevId = 0x000},
		{.b1Flag = 0, .b11CanRevId = 0x000},
		{.b1Flag = 0, .b11CanRevId = 0x000},
		{.b1Flag = 0, .b11CanRevId = 0x000},
		{.b1Flag = 0, .b11CanRevId = 0x000},
		{.b1Flag = 0, .b11CanRevId = 0x000},
	},
};

xCanRevPdoInfo gCanRevPdoInfo;				/*PDO接收待完善*/
/*PDO发送待完善*/
xCanSendPdoInfo gCanSendPdoInfo = 
{
	.CanSendInfo1.u16SoftVer = 0x1000,
	.CanSendInfo2.u16HardVer = 0x1000,
};


typedef union
{
	uint16_t u16data;
	struct
	{
		uint16_t b1Reserved: 1;         //模拟量1
		uint16_t b1Forward: 1;    	  //前进
		uint16_t b1Reverse: 1;        //后退
		uint16_t b1EmrReverse: 1;     //急反           
		uint16_t b1SafeLock: 1;       //互锁
		uint16_t b1LiftLimit: 1;	  //起升限位
		uint16_t b1SlowMode: 1;           //龟速
		uint16_t b1Lift: 1;			 //起升
		uint16_t b1Down: 1;			//下降
		uint16_t b3Reserve: 7;
	};
}xSwiInput;

typedef struct{
	uint8_t u8BatteryType;	//电池类型： 0：锂电 	!0：铅酸
	
	uint16_t	u16Gear1Spd;
	uint16_t	u16Gear2Spd;
	uint16_t	u16Gear3Spd;
	uint16_t	u16Gear4Spd;
	
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
	
	uint16_t	u16RatioOfTransmission;	//转速公里比
	
	uint16_t	u16logicLock;
}xUserInfo;

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
		uint8_t b1Gear4SpdFlag: 1;		/* 低电量*/
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
		uint8_t b8LowSpdFlag: 8;
		
		uint8_t u8Reserve4;
		uint8_t u8Reserve5;
	};
}xValvesInfo;

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

static xValvesInfo sgValvesInfo;
static xUserInfo sgUserInfo;
static xSwiInput sgSwiInput;
static int16_t i16MotorVal = 0;
static int16_t	i16MotorSpd = 0; //MCU feedback speed
static uint16_t u16MotorVal = 0;
static uint8_t	u8PumpOrPropValue = 0;
static uint8_t u8EmsErrFlg = 0;

static void vMstRevProc(xMstRevPara *RevData)
{	
	uint16_t tmp = 0;
	
//	if(0 != RevData->b1MainDriver)
//	{
//		
//	}
//	else
//	{
//		
//	}
//	
//	if(0 != RevData->b1Ebrake)
//	{
//		
//	}
//	else
//	{	
//		
//	}
//	
//	if(0 != RevData->b1Driver3State)
//	{
//		
//	}
//	else
//	{
//		
//	}
	/*故障码处理*/
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
	//Rev_Speed	= i16MotorSpd;
	//RevSPEED = tmp;
	__enable_irq();
}



/*发送给Mst的回调函数*/
static void vMstSendProc(xMstSendPara *SendData) 
{
	int16_t i16MotorValTmp = i16MotorVal;
	float fTmp = 0.0;
	static uint16_t u16LiftUpCnt = 0; 
	//int32_t i32PropValue = 0;
	uint32_t u32PumpValue = 0;
	uint16_t LiftVoltage = 0;
	xMstSendStat LastStatus;	
	/*Do1 or Ai1 motor stop*/
//	if ((1 == i32ErrCodeCheck(ErrCode71)) || (1 == i32ErrCodeCheck(ErrCode51)))
//	{
//		i16MotorValTmp = 0;
//	}
//	
//	if (i16MotorValTmp > 0)
//	{
//		if (i16MotorValTmp >= 4095)
//		{
//			i16MotorValTmp = 4095;
//		}
//		SendData->u8TargetHigh = (i16MotorValTmp >> 8) & 0xFF;
//		SendData->u8TargetLow = i16MotorValTmp & 0xFF;
//		SendData->b1ForwardReq = 1;
//	}
//	else if (i16MotorValTmp < 0)
//	{
//		i16MotorValTmp = 0 - i16MotorValTmp;
//		if (i16MotorValTmp >= 4095)
//		{
//			i16MotorValTmp = 4095;
//		}
//		SendData->u8TargetHigh = (i16MotorValTmp >> 8) & 0xFF;
//		SendData->u8TargetLow = i16MotorValTmp & 0xFF;
//		SendData->b1BackwardReq = 1;
//	}
//	else
//	{
//		SendData->u8TargetHigh = 0;
//		SendData->u8TargetLow = 0;
//		SendData->b1ForwardReq = 0;
//		SendData->b1BackwardReq = 0;
//	}
	
	if (1 == sgSwiInput.b1SafeLock)
	{
		SendData->b1AgvModeEnable = 1;
	}
	else
	{
		SendData->b1AgvModeEnable = 0;
	}
	
	if (1 == sgSwiInput.b1EmrReverse)
	{
		SendData->b1EmgReverse = 1;
	}
	else
	{
		SendData->b1EmgReverse = 0;
	}
	if((sgUserInfo.u16logicLock & ZHILI_MOVE) != 0)
	{
		SendData->b1CoastModeReq = 1;
	}
	else
	/*Move Mode*/
	//if((0 == (sgu16LimitFlg & (BMSErrStop|Move_Limit|PedalMove_Limit|EmsLimitMove|EmsErrFlg)))&&(0 == u8LimitMove)&&(0 == u8EmsErrFlg))
	if((0 == u8EmsErrFlg) || (sgValvesInfo.b1NoActFlag == 0))
	{
		///SendData->b1AgvModeEnable = 1;
		if(sgSwiInput.b1Forward == 1)
		{
			SendData->b1ForwardReq = 1;
		}
		else if(sgSwiInput.b1Reverse == 1)
		{
//			SendData->b1ServoOn = 1;
			SendData->b1BackwardReq = 1;	
		}
		else
		{
			SendData->b1ForwardReq = 0;
			SendData->b1BackwardReq = 0;
		}
		
		if((0 != sgSwiInput.b1Forward) || (0 != sgSwiInput.b1Reverse))
		{
			SendData->u8TargetHigh = u16MotorVal >> 8;
			SendData->u8TargetLow = u16MotorVal;
		}

		if ((1 == sgSwiInput.b1EmrReverse) && ((1 == SendData->b1ForwardReq) || (1 == LastStatus.b1EmsReq)))  
		{
			SendData->b1EmgReverse = 1;
			if (1 == SendData->b1BackwardReq)
			{
				//EmsOk = 1;
				i32ErrCodeSet(39);
			}
		}
		
//		if((1 == sgSwiInput.b1EmrReverse)&&(0 == u16MotorVal))
//		{
//			SendData->b1AgvModeEnable = 0;
//		}
		
		if((0 == sgSwiInput.b1EmrReverse) && ((0 == sgSwiInput.b1Forward) || (0 == sgSwiInput.b1Reverse)))
		{
			;
//			if(EmsOk == 1)
//			{
//				i32ErrCodeClr(39);
//			}
		}
		{
//			if((1 == sgSwiInput.b1EmrReverse)&&(0 != u16MotorVal)&&(0 == Rev_Speed))
//			{
//				i32ErrCodeSet(39);
//				//EMSRecive = 1;
//			}
//			else if((0 == sgSwiInput.b1Ems)&&(0 == u16MotorVal))
//			{
//				if(1 == EMSRecive)
//				{
//					EMSRecive = 0;
//					i32ErrCodeClr(39);
//				}
//			}
		}
	}	
	
	/*Lift Mode*/
	if(sgValvesInfo.b1NoActFlag == 0)
	{
		if( ((1 == sgSwiInput.b1Lift) && (0 == sgSwiInput.b1Down))
		  && (0 == sgSwiInput.b1LiftLimit)
			//&&((sgValvesInfo.b4NoLiftUp & NoLiftUp_LowBat) == 0)
			)
		{
	//		u16LiftUpCnt++;
	//		if(u16LiftUpCnt < (i32GetPara(PARA_AccAndDecLift)*100/5))
	//		{
				u32PumpValue =  _IQ((0.0 + (i32GetPara(PARA_PropDMaxCurrent0) / 1.0 * (i32LocalAiGetValue(AI_SWI1_CHECK) - 2500) / 2500)) / PUMP_FACTOR);
				//u32PumpValue = _IQ((sgUserInfo.fPropMinCurrent + (sgUserInfo.fPropMaxCurrent - sgUserInfo.fPropMinCurrent) * u8PumpOrPropValue / PUMP_RANGE) / PROPD_STD_CURRENT);
	//				i32SetPara(PARA_Ai2, i32LocalAiGetValue(AI_THRO_WIP_R));
	//				i32SetPara(PARA_Ai3, i16GetPowerTmp());
	//				i32SetPara(PARA_Ai4, i16GetMotorTmp());
				vPropSetTarget(PropDriverCh0, u32PumpValue);
				//vPropSetTarget(PropDriverCh0, 0);
				i32DoPwmSet(DOWN_VALVE1,DRIVER_CLOSE);	
				i32DoPwmSet(DOWN_VALVE2,DRIVER_CLOSE);
	//		}
	//		else
	//		{ 
	//			i32DoPwmSet(LIFTUP_VALVE,DRIVER_CLOSE);
	//		}
		}
		else if( (1 == sgSwiInput.b1Down) && (0 == sgSwiInput.b1Lift))
		{
			vPropSetTarget(PropDriverCh0, 0);
			LiftVoltage = i32LocalAiGetValue(AI_SWI1_CHECK);
			if((LiftVoltage <= 2300) && (LiftVoltage >= 1800))   //2.3~1.8V  Valve 1 opened
			{
				i32DoPwmSet(DOWN_VALVE1,DRIVER_OPEN);	
				i32DoPwmSet(DOWN_VALVE2,DRIVER_CLOSE);
			}
			else if((LiftVoltage <= 1700) && (LiftVoltage >= 1200)) //1.7~1.2V   Valve 2 opened
			{
				i32DoPwmSet(DOWN_VALVE1,DRIVER_CLOSE);	
				i32DoPwmSet(DOWN_VALVE2,DRIVER_OPEN);			
			}
			else  //1.1~0.6  Valve 1 & 2 opened
			{
				i32DoPwmSet(DOWN_VALVE1,DRIVER_OPEN);	
				i32DoPwmSet(DOWN_VALVE2,DRIVER_OPEN);
				
			}	
			u16LiftUpCnt = 0;
		}
		else
		{
			vPropSetTarget(PropDriverCh0, 0);
			u16LiftUpCnt = 0;
			i32DoPwmSet(DOWN_VALVE1,DRIVER_CLOSE);	
			i32DoPwmSet(DOWN_VALVE2,DRIVER_CLOSE);
			u8PumpOrPropValue = 0;
		}
	}
	
	if( (0 == sgSwiInput.b1SafeLock) && (i16MotorSpd != 0))	//行驶过程丢互锁
	{
		SendData->b1AgvModeEnable = 0;	//急刹车减速 
		//SendData->b1EmgReverse = 0;
		SendData->b1ForwardReq = 0;
		SendData->b1BackwardReq = 0;  
	}
	else if( (0 == sgSwiInput.b1SafeLock) && (i16MotorSpd == 0))
	{
		//静止时互锁为0  比如进入直立模式
		if(sgSwiInput.b1SlowMode  != 0)
		{
			;//SendData->b1SlowModeReq = 1;
		}
	}

	//send para  to upper computer 
	i32SetPara(PARA_AngleValue, u16MotorVal);		/*Send Motor Value*/
	i32SetPara(PARA_LoadRate, u32PumpValue);		/*pump Motor Value*/	
	i32SetPara(PARA_PressureVlaue1, SendData->b1ForwardReq);		/*Move state*/
	i32SetPara(PARA_PressureVlaue2, SendData->b1BackwardReq);		/*Move state*/	
	//i32SetPara(PARA_PressureVlaue2, sgMstSlvProc.MstRevData.b1EBrakeRequest);
}
/*******************************************************************************
* FunctionName: vAiMonitor(void)
* Description:  Analog Input monitor
* Input: void
* Output:  void 
*
* Author: Young
* Date: 2024-05-17
* Revision: V2.0
*******************************************************************************/
static void vAiMonitor(void)
{
	uint8_t u8SpeedRate = 100;
	int32_t i32AdcValue = 0;	
	/*********** Motor Speed *************/
	/*add Turn Dec Spd*/
	//u16MotorVal = u16SteerAngleDecSpd(u16MotorVal, abs(i16SteerAngle), &sgUserInfo.SteerAngleDecSpd);
	
	i32AdcValue = i32LocalAiGetValue(AI_THRO_WIP_R);
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
 
	/* 起升 */
	{
		i32AdcValue = i32LocalAiGetValue(AI_SWI1_CHECK);
		if(i32AdcValue > sgUserInfo.u16LiftUpMax)
		{
			u8PumpOrPropValue = 255;
		}
		else if((i32AdcValue <= sgUserInfo.u16LiftUpMax) && (i32AdcValue > sgUserInfo.u16LiftUpMin))
		{
			u8PumpOrPropValue = (255 * (i32AdcValue - sgUserInfo.u16LiftUpMin)) / (sgUserInfo.u16LiftUpMax - sgUserInfo.u16LiftUpMin);
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
	i32SetPara(PARA_LiftValveCurrent, sgSwiInput.b1SafeLock);		/*pump Motor Value*/		
	//i32SetPara(PARA_CalibrationStatus, i32AdcValue);		/*pump throttle value*/		
	//i32SetPara(PARA_CalibrationStatus, i32LocalAiGetValue(AI_SWI1_CHECK));
//	i32SetPara(PARA_ForwardValveCurrent, i32LocalAiGetValue(AI_SWI1_CHECK) / 100);
//	i32SetPara(PARA_BackValveCurrent, i32LocalAiGet(AI_SWI1_CHECK));
	
}

/*开关量监控*/
static void vSwiMonitor(void)
{
	xSwiInput SwiInput = {0};
	static uint16_t Snail_Count = 0;
	static uint8_t err = 0;
	static uint8_t SafeLockLogic = 0;  //互锁相关逻辑标志位
	static uint8_t EmrLogic	= 0;	//急反相关逻辑标志位
	static uint8_t Acculatelogic = 0; //加速器相关逻辑
	static uint16_t ZhiLiDelay = 0;
	static uint8_t SlowModeDelay = 0;
	
	if(1 == i32LocalDiGet(FORWARD_SWI))
	{
		SwiInput.b1Forward = 1;
	}
	
	if(1 == i32LocalDiGet(REVERSE_SWI))
	{
		SwiInput.b1Reverse = 1;
	}

	if(1 == i32LocalDiGet(EMRREVERSE_SWI))
	{
		SwiInput.b1EmrReverse = 1;
	}	
	if(1 == i32LocalDiGet(SAFELOCK_SWI))
	{
		SwiInput.b1SafeLock = 1;
	}
	
	if(1 == i32LocalDiGet(LIFT_LIMIT_SWI))
	{
		SwiInput.b1LiftLimit = 1;
	}
	
	if(1 == i32LocalDiGet(GUISU_SWI))
	{
		SwiInput.b1SlowMode = 1;
	}
	if( (i32LocalAiGetValue(AI_SWI1_CHECK) >= (2500 + LIFT_THROTTLE_DEAD))  //2.5~5起升
	// && (i32LocalAiGetValue(AI_SWI1_CHECK) <= sgUserInfo.u16LiftUpMax) 
	)
	{
		SwiInput.b1Lift = 1;
		SwiInput.b1Down = 0;
	}
	else if( (i32LocalAiGetValue(AI_SWI1_CHECK) <= (2500 - LIFT_THROTTLE_DEAD))
	// && (i32LocalAiGetValue(AI_SWI1_CHECK) >= sgUserInfo.u16LiftUpMin) 
	)
	{
		SwiInput.b1Lift = 0;
		SwiInput.b1Down = 1;
	}
	else
	{
		SwiInput.b1Down = 0;
		SwiInput.b1Lift = 0;
	}

	//logic process
	if(SwiInput.b1SafeLock == 0)
	{
		if(((SwiInput.b1Forward == 1) || (SwiInput.b1Reverse == 1)) && (SwiInput.b1SlowMode == 0))
		{
			sgUserInfo.u16logicLock |= THRORRLE_LOCK;		//先加速器再互锁标志位
		}
		else
		{
			sgUserInfo.u16logicLock &= ~THRORRLE_LOCK;			
		}
//		if((SwiInput.b1Lift == 1) || (SwiInput.b1Down == 1))
//			sgUserInfo.u16logicLock |= LIFT_LOCK;
			if (SwiInput.b1SlowMode == 1)
			{
				sgUserInfo.u16logicLock |= ZHILI_SWI_ACT;
			}
			else
			{
				sgUserInfo.u16logicLock &= ~ZHILI_SWI_ACT;
			}
		if(SwiInput.b1EmrReverse == 1)
		{
			i32ErrCodeSet(39);		
		}
		
	}
	else  //sw5 = 1
	{
		//互锁闭合  先闭合急反  再旋转加速器
		if((SwiInput.b1EmrReverse == 1) && ((sgSwiInput.b1Forward == 0) || (sgSwiInput.b1Reverse == 0)))
		{
			if((SwiInput.b1EmrReverse == 1) && ((SwiInput.b1Forward == 1) || (SwiInput.b1Reverse == 1)))
				sgUserInfo.u16logicLock |= EMR_LOCK;
		}
		else
			sgUserInfo.u16logicLock &= ~EMR_LOCK;
		if((sgSwiInput.b1SafeLock == 0) && ((SwiInput.b1Forward == 1) || (SwiInput.b1Reverse == 1)))
		{
			if((SwiInput.b1SafeLock == 1) && ((SwiInput.b1Forward == 1) || (SwiInput.b1Reverse == 1)))
				i32ErrCodeSet(39);
		}
	}
		if (SwiInput.b1SlowMode == 1)
		{
			//static INT16S logicLockOld;
			if (SlowModeDelay < (500/T_MS_PLC_PERIOD))
			{
				SlowModeDelay++;
			}
			else if (SlowModeDelay == (500/T_MS_PLC_PERIOD))
			{
				sgUserInfo.u16logicLock ^= SLOW_MODE_FLAG;  //prelong press slow button 500ms Enter/exti Slow mode
				SlowModeDelay++;
			}
			else
			{
			}
		}
		else
		{
			SlowModeDelay = 0;
		}	
//	if((1 == i32ErrCodeCheck(ErrCode40)) && (i16MotorSpd == 0))
//	{
//		SwiInput.b1Forward =0;
//		SwiInput.b1Reverse = 0;
//		//u16MotorVal = 0;
//	}
//	if((SwiInput.b1SafeLock == 0) &&(SwiInput.b1Forward == 0) && (SwiInput.b1Reverse == 0) && (SwiInput.b1EmrReverse == 0)
//		&& (SwiInput.b1SlowMode == 0)
//		)
//	{
//		if(1 == i32ErrCodeCheck(ErrCode40))
//		{
//			i32ErrCodeClr(39);
//			SafeLockLogic = 0;
//			Acculatelogic = 0;
//			EmrLogic = 0;
//		}			
//	}
	if ((sgUserInfo.u16logicLock & ZHILI_SWI_ACT) != 0)  //油门释放，按急反0.2S后，进入直立行走
	{
		if ((SwiInput.b1Forward  == 0) && (SwiInput.b1Reverse == 0))
		{
			if (ZhiLiDelay < (200/PLC_PERIOD))
				ZhiLiDelay++;
			else
				sgUserInfo.u16logicLock |= ZHILI_MOVE;
		}
		else
			ZhiLiDelay = 0;
	}
	else
	{
		sgUserInfo.u16logicLock &= ~ZHILI_MOVE;
		ZhiLiDelay = 0;
	}

	if((sgUserInfo.u16logicLock & (THRORRLE_LOCK | SAFE_LOCK | EMR_LOCK)) != 0)
	{
		SwiInput.b1Forward = 0;
		SwiInput.b1Reverse = 0;
		u16MotorVal = 0;
		i32ErrCodeSet(39);		
	}
	sgSwiInput.u16data = SwiInput.u16data;		
	//default High Speed
	sgValvesInfo.b1Gear1SpdFlag = 0;
	sgValvesInfo.b1Gear2SpdFlag = 0;
	sgValvesInfo.b1Gear3SpdFlag = 0;
	sgValvesInfo.b1Gear4SpdFlag = 0;
	
//	if((1 == SwiInput.b1Pedal)&&(0 == SwiInput.b1Fence2)&&(0 == SwiInput.b1Fence1))
//	{
//		sgValvesInfo.b1Gear1SpdFlag = 1;
//		sgValvesInfo.b1Gear2SpdFlag = 0;
//		sgValvesInfo.b1Gear3SpdFlag = 0;
//		sgValvesInfo.b1Gear4SpdFlag = 0;
//	}
//	
//	if((1 == HandleInput.b1SnailRequest)||(1 == sgFlagVal.Height_SpeedLimit))
//	{
//		sgValvesInfo.b1Gear1SpdFlag = 0;
//		sgValvesInfo.b1Gear2SpdFlag = 1;
//		sgValvesInfo.b1Gear3SpdFlag = 0;
//		sgValvesInfo.b1Gear4SpdFlag = 0;
//	}
//	
//	if(1 == CanRev22CInfoLast.b1SnailRequest)
//	{
//		sgValvesInfo.b1Gear1SpdFlag = 0;
//		sgValvesInfo.b1Gear2SpdFlag = 1;
//		sgValvesInfo.b1Gear3SpdFlag = 0;
//		sgValvesInfo.b1Gear4SpdFlag = 0;
//	}
//	
	if((sgUserInfo.u16logicLock & ZHILI_MOVE) != 0)
	{
		sgValvesInfo.b1Gear1SpdFlag = 0;
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 1;
		sgValvesInfo.b1Gear4SpdFlag = 0;
	}
	
	if((sgValvesInfo.b8LowSpdFlag == 1) || ((sgUserInfo.u16logicLock & SLOW_MODE_FLAG) != 0))
	{
		sgValvesInfo.b1Gear1SpdFlag = 0;
		sgValvesInfo.b1Gear2SpdFlag = 0;
		sgValvesInfo.b1Gear3SpdFlag = 0;
		sgValvesInfo.b1Gear4SpdFlag = 1;
	}
	i32SetPara(PARA_ForwardValveCurrent, SwiInput.b1Lift);	
	i32SetPara(PARA_BackValveCurrent, SwiInput.b1Down);
}

static void vBatteryManage(void)
{
	uint8_t u8Soc;
	if(LiBattery == sgUserInfo.u8BatteryType)
	{
		/***BMS Logic Process, reserved ***/
		//u8Soc = gCanRevPdoInfo.BMSRev052.u8BMSSOC;
			
//		if(1 == gCanRevPdoInfo.BMSRev053.b1LowTempSingleVolL2)	
//			i32ErrCodeSet();	
	}
	else
	{
		u8Soc = u8GetBatterySoc();
	}
	
	if(u8Soc < 15)
	{
		;
//		sgValvesInfo.b8LowSpdFlag = 1;	//降速
//		sgValvesInfo.b4NoLiftUp |= NoLiftUp_LowBat;
//		//i32ErrCodeSet(BAT_LOW_CAP2_ERR);
		//i32ErrCodeClr(BATTERY_LOW_CAP1_ERR);
	}
	else if(u8Soc < 20)
	{
		;
//		i32ErrCodeSet(BATTERY_LOW_CAP1_ERR);
//		i32ErrCodeClr(BAT_LOW_CAP2_ERR);
	}
	else
	{
		i32ErrCodeClr(BAT_LOW_CAP2_ERR);
		i32ErrCodeClr(BATTERY_LOW_CAP1_ERR);		
	}
	i32SetPara(PARA_BmsSoc ,u8Soc);
	
	__disable_irq();
	//gCanSendPdoInfo.sgHMISendPdo.SOC = u8Soc;
	__enable_irq();
}
/*******************************************************************************
* Name: void vUserEcuInit(void)
* Descriptio: UserEcu初始化函数
* Input: NULL
* Output: NULL  
*******************************************************************************/
void vUserEcuInit(void)
{	

	vMstRevRegister(vMstRevProc);
	vMstSendRegister(vMstSendProc);
	
	vSetPdoPara(PdoPara);
	
	
	vSetNetTimer(TIMER_EcuPowerOn, 1000);	/**/
	
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
	
	sgUserInfo.u16RatioOfTransmission = i32GetPara(PARA_RatioOfTransmission);
//	{
//		uint32_t u32Sum = 0;
//		uint32_t i = 0;
//		for (i=0; i<(128 * 1024) >> 2; i++)
//		{
//			u32Sum += *(uint32_t*)(0x08000000 + 4 * i);
//		}
//		u32Sum = u32Sum - *(uint32_t*)(0x08020000);
//		
//		if (0 == u32Sum)
//		{
//			
//		}
//		else
//		{
//			i32LogWrite(INFO, LOG_USER, "Sum = 0x%x\r\n", u32Sum);
//		}
//	}
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
	uint8_t i = 0;
	uint32_t u32DiValue = 0;
	uint32_t u32PumpValue = 0;
	static uint32_t u8MainConnectCnt = 0;
	static uint8_t check_err = 0;	
	if(true == u8GetNetTimerOverFlag(TIMER_EcuPowerOn))
	{
		u8EcuProcFlag = 1;
		vKillNetTimer(TIMER_EcuPowerOn);
	}
	
		if(u8MainConnectCnt < 100)
		{
			u8MainConnectCnt++;
			if(u8MainConnectCnt > 20)
			{
				if( (1 == i32LocalDiGet(SAFELOCK_SWI)) || (1 == i32LocalDiGet(FORWARD_SWI))||(1 == i32LocalDiGet(REVERSE_SWI))
					||(i32LocalAiGetValue(AI_SWI1_CHECK) >= (2500 + LIFT_THROTTLE_DEAD)) 
					||(i32LocalAiGetValue(AI_SWI1_CHECK) <= (2500 - LIFT_THROTTLE_DEAD)) 
					||(1 == i32LocalDiGet(EMRREVERSE_SWI))
					||(1 == i32LocalDiGet(GUISU_SWI))
				)
				{
					u8EmsErrFlg = 1;
					check_err = 1;
					i32ErrCodeSet(39);
					sgValvesInfo.b1NoActFlag = 1;
				}
			}
		}
		else
		{
			if(1 == check_err)
			{
				if((0 == i32LocalDiGet(SAFELOCK_SWI)) &&(0 == i32LocalDiGet(FORWARD_SWI))&&(0 == i32LocalDiGet(REVERSE_SWI))
				&& (i32LocalAiGetValue(AI_SWI1_CHECK) <= (2500 + LIFT_THROTTLE_DEAD)) 
				&& (i32LocalAiGetValue(AI_SWI1_CHECK) >= (2500 - LIFT_THROTTLE_DEAD)) 
				&&(0 == i32LocalDiGet(EMRREVERSE_SWI)) && (0 == i32LocalDiGet(GUISU_SWI))
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
		vBatteryManage();
		vSwiMonitor();		
		//vCanRevPdoProc();
		vAiMonitor();
	}
	
	vWdgSetFun(WDG_USER_BIT);
}
#endif
