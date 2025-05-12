/*******************************************************************************
* Filename: UserEcuProc.c	                                             	   *
* Description: 逻辑层的源文件，包含对应的回调函数	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/05/30    														   *
* Revision:	 														 		   *
*******************************************************************************/
#include "UserCanComm.h"
#include "UserEcuProc.h"
#include "ErrCode.h"
#include "IQmathLib.h"
#include "PropProc.h"
#include "CanRevProc.h"
#include "BeepProc.h"
#include "AlarmLamp.h"
#include "AngleSensor.h"
#include "PressureSensor.h"
#include "string.h"
#include "Para.h"
#include "WdgProc.h"
#include "math.h"
#include "Log.h"
#include "LocalAi.h"

#if (USER_TYPE == USER_ECU_2IN1)

#define MOVE_MODE		1		/*行进模式*/
#define	LIFT_MODE		0		/*升降模式*/
#define INITIAL_MODE	0x80	/*Initial State*/

#define SLOW_SPEED		1		/*龟速模式*/
#define	FULL_SPEED		0		/*全速模式*/

#define	ANTI_PINCH_STAT0	0	/*下限位之上*/
#define	ANTI_PINCH_STAT1	1	/*下限位之上到下限位的一个过程*/
#define	ANTI_PINCH_STAT2	2	/*释放手柄*/
#define	ANTI_PINCH_STAT3	3	/*下限位之下，开始执行动作*/


typedef union
{
	uint8_t u8data;
	struct
	{
		uint8_t b1PcuSpeed: 1;
		uint8_t b1SlowKey: 1;
		uint8_t b6Reserve: 6;
	};
}xSlowKey;

typedef union
{
	uint16_t u16data;
	struct
	{
		uint8_t b130PerMin: 1;			/*动作*/
		uint8_t b160PerMin: 1;			/*不可以取消的报警*/
		uint8_t b1180PerMin: 1;			/*报警，可以取消的报警*/
		uint8_t b1240PerMin: 1;			/*下降报警*/
		uint8_t b4Reserve: 4;
		uint8_t u8Cnt;					/*用于计数，来进行两长一短*/
	};
}xSpeakerFun;

typedef union
{
	uint8_t u8Data;
	struct
	{
		uint8_t b1SenSorErr: 1;
		uint8_t	b1CaliReverse: 1;
		uint8_t b1CaliFailure: 1;
		uint8_t b1Per80Err: 1;
		uint8_t b1Per90Err: 1;
		uint8_t b1Per99Err: 1;
		uint8_t b1Per100Err: 1;
		uint8_t b1Reserve: 1;
	};
}xPresureErrInfo;


typedef union
{
	uint8_t u8Data[2];
	struct
	{
		uint8_t b1ForWardFlag: 1;
		uint8_t b1BackWardFlag: 1;
		uint8_t b1LiftUpFlag: 1;
		uint8_t b1TurnLeftFlag: 1;
		uint8_t b1TurnRightFlag: 1;
		uint8_t b1LiftDownFlag: 1;
		uint8_t	b1LiftSpdFlag: 1;
		uint8_t b1DisablePcuFlag: 1;
		
		uint8_t b1ForWardStat: 1;
		uint8_t b1BackWardStat: 1;
		uint8_t b1LiftUpStat: 1;
		uint8_t b1TurnLeftStat: 1;
		uint8_t b1TurnRightStat: 1;
		uint8_t b1LiftDownStat: 1;
		uint8_t b2Reserve2: 2;
		
	};
}xValvesInfo;

typedef struct
{
	uint16_t	u16FastDriveSpeed;
	uint16_t	u16SlowDriveSpeed;
	uint16_t	u16DriveSpeedAfterLift;
	uint16_t	u16LiftSpeed;
	uint16_t	u16MaxTurnSpeed;
	uint16_t	u16TurnPowerLimit;
	uint16_t	u16LowerSpeed;
	uint16_t	u16ValueOpenLoopCurrent;
	
	float		fFastSpdPer5msAccStep;		
	float		fSlowSpdPer5msAccStep;
	float		fPitSpdPer5msAccStep;		/*过了坑洞就是高空速度*/
	float		fLiftSpdPer5msAccStep;
	float		fDownSpdPer5msAccStep;
	float		fTurnSpdPer5msAccStep;
	
	float		fFastSpdPer5msDecStep;
	float		fSlowSpdPer5msDecStep;
	float		fPitSpdPer5msDecStep;		/*过了坑洞就是高空速度*/
	float		fLiftSpdPer5msDecStep;
	float		fDownSpdPer5msDecStep;
	float		fTurnSpdPer5msDecStep;
	
	float		fPropMinCurrent;
	float		fPropMaxCurrent;
	uint16_t	u16PropMinADC;
	uint16_t	u16PropMaxADC;
	
	uint16_t	u16ValveOpenLoopCurrent;

	uint8_t		b1PitProtectFunc: 1;
	uint8_t		b1AntiPinchFunc: 1;
	uint8_t		b1LowBatAlmFunc: 1;
	uint8_t		b1ActAlmFunc: 1;
	
	uint8_t		b4Reserve: 4;
	
	uint8_t		u8BatteryType;
}xUserInfo;

//static uint8_t u8PcuEnableFlag ;
static uint8_t u8PcuMode;
static xSlowKey sgSlowKey;
static int16_t i16sgPcuVal = 0;
static uint16_t	u16MotorSpd = 0;
static xValvesInfo sgValvesInfo;
static xSpeakerFun sgSpeakerFun;

static xUserInfo sgUserInfo;

static uint8_t u8AntiPinchStat = 0;
static uint16_t	u16PropValue = 0;

xCanRevPdoInfo gCanRevPdoInfo;				
/*PDO发送待完善*/
xCanSendPdoInfo gCanSendPdoInfo;
/*******************************************************************************
* Name: void vPcuErrProc(void)
* Descriptio: Pcu Err Proc
* Input: NULL
* Output: NULL 
*******************************************************************************/
void vPcuErrProc(uint8_t u8Type)
{
	switch (u8Type)
	{
		case PCU_Init:
			{
				u8PcuMode = INITIAL_MODE;
				sgSlowKey.b1PcuSpeed = FULL_SPEED;
				
				i32DoPwmSet(FORWARD_PUMP, PUMP_CLOSE_PERCENTAGE);
				i32DoPwmSet(BACKWARD_PUMP, PUMP_CLOSE_PERCENTAGE);
				i32DoPwmSet(TURNLEFT_PUMP, PUMP_CLOSE_PERCENTAGE);
				i32DoPwmSet(TURNRIGHT_PUMP, PUMP_CLOSE_PERCENTAGE);
				i32DoPwmSet(LIFTUP_PUMP, PUMP_CLOSE_PERCENTAGE);
				vPropSetTarget(LIFTDOWN_PUMP, 0);
			}
			break;
		case PCU_LiftKeyPress:
			{
				i32ErrCodeSet(LIFT_BUTTON_ERR);	
			}
			break;
		case PCU_SlowKeyPress:
			{
				i32ErrCodeSet(SLOW_BUTTON_ERR);
			}
			break;
		case PCU_MoveKeyPress:
			{
				i32ErrCodeSet(MOVE_BUTTON_ERR);
			}
			break;
		case PCU_TurnLeftPress:
			{
				i32ErrCodeSet(PLAT_LEFT_BUTTON_ERR);
			}
			break;
		case PCU_TurnRightPress:
			{
				i32ErrCodeSet(PLAT_RIGHT_BUTTON_ERR);
			}
			break;
		case PCU_EnableKeyPress:
			{
				i32ErrCodeSet(ENABLE_BUTTON_ERR);
			}
			break;
		case PCU_ValueNoZero:/*举升后行走*/
			{
				i32ErrCodeSet(HANDLE_NOT_IN_MIDDLE_POSI_ERR);
			}
			break;
		default:
			break;
		
	}

}
/*******************************************************************************
* Name: void PcuRevProc(xPcuRevParat *RevData)
* Descriptio: 接收Pcu数据进行处理
* Input: RevData：Pcu接收的数据格式
* Output: NULL 
*******************************************************************************/
void vPcuRevProc(xPcuRevPara *RevData)
{
#if (PCU_TYPE_LZ == PCU_TYPE)
	if ((0x2 == RevData->Data.b4Const1) && (0x3 == RevData->Data.b4Const2) && \
		(0x4 == RevData->Data.b4Const3) && (0x5 == RevData->Data.b4Const4) && \
		(0x6 == RevData->Data.b4Const5) && (0x7 == RevData->Data.b4Const6) && \
		(0xC == RevData->Data.b4Const7))
	{
		int8_t i8MotorVal = 0;
		int16_t i16MotorVal = 0;
		uint8_t u8KeyInfo = 0;
		uint16_t u16MotorSpdLimit = 0;
		
		if((1 == i32LocalDiGet(PCU_SWICTH)) && (0 == sgValvesInfo.b1DisablePcuFlag))/*添加上下控检测*/
		{
			if(1 == RevData->Data.b1Mode)
			{
				
			}
			else
			{
				
			}
			
			if(1 == RevData->Data.b1SlowSpdSwitch)
			{
				u8KeyInfo |= 1 << SLOW_KEY;
			}

			
			if(1 == RevData->Data.b1SpeakerSwitch)
			{
				u8KeyInfo |= 1 << SPEAKER_KEY;
			}	
			
			if(1 == RevData->Data.b1TraSwitch)
			{
				u8KeyInfo |= 1 << MOVE_KEY;
			}

			if(1 == RevData->Data.b1LiftingSwitch)
			{
				u8KeyInfo |= 1 << LIFT_KEY;
			}

			if(1 == RevData->Data.b1EnableSwitch)
			{
				u8KeyInfo |= 1 << ENABLE_KEY;
			}

			if(1 == RevData->Data.b1TurnRightSwitch)
			{
				u8KeyInfo |= 1 << RIGHT_KEY;
			}
			
			if(1 == RevData->Data.b1TurnLeftSwitch)
			{
				u8KeyInfo |= 1 << LEFT_KEY;
			}
			
			/*Set Key Info*/
			i32SetPara(PARA_PcuKeyInfo, u8KeyInfo);
			gCanSendPdoInfo.CanSendInfo3.u8PcuKeyInfo = u8KeyInfo;
			
			i8MotorVal = RevData->Data.b4HandleCtrlHigh << 4 | RevData->Data.b4HandleCtrlLow;
			
		}
		else	/*lilu 20230706 恢复到默认模式*/
		{
				u8PcuMode = INITIAL_MODE;
				sgSlowKey.b1PcuSpeed = FULL_SPEED;
			
				sgValvesInfo.b1ForWardStat = 0;
				sgValvesInfo.b1BackWardStat = 0;
				sgValvesInfo.b1TurnLeftStat = 0;
				sgValvesInfo.b1TurnRightStat = 0;
				sgValvesInfo.b1LiftUpStat = 0;
				sgValvesInfo.b1LiftDownStat = 0;
		}
		
		i32SetPara(PARA_HandleAnalog, abs(i8MotorVal) * 100 / 127);
		gCanSendPdoInfo.CanSendInfo3.u8HandleAnalog = abs(i8MotorVal) * 100 / 127;
		
		if (abs(i8MotorVal) < i32GetPara(PAPA_DeadZoneAdjust))		/*lilu 20230706 add deadzone*/
		{
			i8MotorVal = 0;
		}
		
		if (INITIAL_MODE == u8PcuMode)
		{
			if (u8KeyInfo & (1 << MOVE_KEY))
			{
				u8PcuMode = MOVE_MODE;
			}
			
			if(u8KeyInfo & (1 << LIFT_KEY))
			{
				u8PcuMode = LIFT_MODE;
			}
		}
		else
		{	
			/*slow key press*/
			if (u8KeyInfo & (1 << SLOW_KEY))
			{
				sgSlowKey.b1SlowKey = 1;
			}
			else 
			{
				/*按键松开时候进行切换*/
				if ((MOVE_MODE == u8PcuMode) && (1 == sgSlowKey.b1SlowKey))	/*只有行进模式才可以改变龟速状态*/
				{
					sgSlowKey.b1SlowKey = 0;
					sgSlowKey.b1PcuSpeed ^= 1;
				}
			}
			/*Speaker Key Press*/
			if (u8KeyInfo & (1 << SPEAKER_KEY))
			{
				i32DoPwmSet(SPEAKER_PUMP, PUMP_OPEN_PERCENTAGE);;
			}
			else
			{
				i32DoPwmSet(SPEAKER_PUMP, PUMP_CLOSE_PERCENTAGE);
			}
			/*Enable Key Press*/
			if (u8KeyInfo & (1 << ENABLE_KEY))
			{
				/*turn left key press*/
				if (u8KeyInfo & (1 << LEFT_KEY ))
				{
					if ((MOVE_MODE == u8PcuMode) && (0 == sgValvesInfo.b1TurnLeftFlag))
					{
						sgValvesInfo.b1TurnLeftStat = 1;
					}
				}
				else
				{
					sgValvesInfo.b1TurnLeftStat = 0;			
				}
				/*turn right key press*/
				if (u8KeyInfo & (1 << RIGHT_KEY ))
				{
					if ((MOVE_MODE == u8PcuMode) && (0 == sgValvesInfo.b1TurnRightFlag))
					{
						sgValvesInfo.b1TurnRightStat = 1;
					}
				}
				else
				{
					sgValvesInfo.b1TurnRightStat = 0;
				}
				
				if (MOVE_MODE == u8PcuMode)
				{
					if (i8MotorVal > 0) 
					{
						sgValvesInfo.b1BackWardStat = 0;
						if (0 == sgValvesInfo.b1ForWardFlag)
						{
							sgSpeakerFun.b130PerMin = 1;
							sgValvesInfo.b1ForWardStat = 1;
						}
					}
					else if(i8MotorVal < 0)
					{
						sgValvesInfo.b1ForWardStat = 0;
						if (0 == sgValvesInfo.b1BackWardFlag)
						{
							sgSpeakerFun.b130PerMin = 1;
							sgValvesInfo.b1BackWardStat = 1;							
						}
					}
					else
					{
						sgSpeakerFun.b130PerMin = 0;
						sgValvesInfo.b1ForWardStat = 0;
						sgValvesInfo.b1BackWardStat = 0;
					}
				}
				else if (LIFT_MODE == u8PcuMode)
				{
					/*lilu, 20230703, 举升反转功能*/
					if (FunctionEnable == i32GetPara(PARA_LiftReverseFunc))
					{
						i8MotorVal = 0 - i8MotorVal;
					}
					
					if ((i8MotorVal > 0) && (1 == i32LocalDiGet(UP_LIMIT_SWITCH)))   /*上限位是常闭状态，可以起升*/
					{
						sgValvesInfo.b1LiftDownStat = 0;
						if (0 == sgValvesInfo.b1LiftUpFlag)
						{
							sgSpeakerFun.b130PerMin = 1;
							sgValvesInfo.b1LiftUpStat = 1;
						}	
					}
					else if(i8MotorVal < 0)
					{
						sgValvesInfo.b1LiftUpStat = 0;						
						if (0 == sgValvesInfo.b1LiftDownFlag)
						{
							sgSpeakerFun.b1240PerMin = 1;	
							sgValvesInfo.b1LiftDownStat = 1;							
						}
					}
					else 
					{
						sgSpeakerFun.b1240PerMin = 0;
						sgSpeakerFun.b130PerMin = 0;
						sgValvesInfo.b1LiftUpStat = 0;
						sgValvesInfo.b1LiftDownStat = 0;
					}
				}	
			}
			else
			{
				sgSpeakerFun.b1240PerMin = 0;
				sgSpeakerFun.b130PerMin = 0;
				/*Close All Pumps*/
				sgValvesInfo.b1ForWardStat = 0;
				sgValvesInfo.b1BackWardStat = 0;
				sgValvesInfo.b1TurnLeftStat = 0;
				sgValvesInfo.b1TurnRightStat = 0;
				sgValvesInfo.b1LiftUpStat = 0;
				sgValvesInfo.b1LiftDownStat = 0;
				i8MotorVal = 0;
				
				vKillNetTimer(TIMER_EcuAntiPinchFunc);
				u8AntiPinchStat = ANTI_PINCH_STAT2;
				
				/*状态切换的时候使能按键松开*/
				if (u8KeyInfo & (1 << MOVE_KEY))
				{
					u8PcuMode = MOVE_MODE;
				}
				if(u8KeyInfo & (1 << LIFT_KEY))
				{
					u8PcuMode = LIFT_MODE;
				}
			}
			
			i16MotorVal = i8MotorVal * 32;		/*Motor Value range -4064 ~ 4064*/
//			if (i16MotorVal < 0)
//			{
//				i16MotorVal = 0 - i16MotorVal;
//			}
			/*龟速模式下，发给电机的状态值会缩减至相应的比例*/
			if(MOVE_MODE == u8PcuMode)
			{
				if ((u8KeyInfo & (1 << RIGHT_KEY )) || (u8KeyInfo & (1 << LEFT_KEY )))
				{
					if (0 == i16MotorVal)
					{
						i16MotorVal = i32GetPara(PARA_MaxTurnSpeed) * MOTOR_MAX_SPEED_VALUE / 100;
					}
					else 
					{
						u16MotorSpdLimit = i32GetPara(PARA_TurnPowerLimit) * MOTOR_MAX_SPEED_VALUE / 100;
						if (i16MotorVal > 0) 
						{
							i16MotorVal += u16MotorSpdLimit;
						}
						if(i16MotorVal < 0)
						{
							i16MotorVal -= u16MotorSpdLimit;
						}
					}
				}
				/*快速行驶速度*/
				u16MotorSpdLimit = i32GetPara(PARA_FastDriveSpeed) * MOTOR_MAX_SPEED_VALUE / 100;
				if (i16MotorVal >= u16MotorSpdLimit) 
				{
					i16MotorVal = u16MotorSpdLimit;
				}
				if(i16MotorVal <= (0 - u16MotorSpdLimit))
				{
					i16MotorVal = 0 - u16MotorSpdLimit;
				}
				
				
				if (SLOW_SPEED == sgSlowKey.b1PcuSpeed)
				{
					u16MotorSpdLimit = i32GetPara(PARA_SlowDriveSpeed) * MOTOR_MAX_SPEED_VALUE / 100;
					/*慢速行驶速度*/
					if (i16MotorVal >= u16MotorSpdLimit) 
					{
						i16MotorVal = u16MotorSpdLimit;
					}
					if(i16MotorVal <= (0 - u16MotorSpdLimit))
					{
						i16MotorVal = 0 - u16MotorSpdLimit;
					}
				}
				
				/*举升后行驶速度*/
				if ((1 == i32LocalDiGet(PIT_SWITCH)) || (1 == sgValvesInfo.b1LiftSpdFlag))
				{
					u16MotorSpdLimit = i32GetPara(PARA_DriveSpeedAfterLift) * MOTOR_MAX_SPEED_VALUE / 100;
					if (i16MotorVal >= u16MotorSpdLimit) 
					{
						i16MotorVal = u16MotorSpdLimit;
					}
					if(i16MotorVal <= (0 - u16MotorSpdLimit))
					{
						i16MotorVal = 0 - u16MotorSpdLimit;
					}
				}
			}

			/*举升速度*/
			if (LIFT_MODE == u8PcuMode)
			{
				/*Up*/
				u16MotorSpdLimit = i32GetPara(PARA_LiftSpeed) * MOTOR_MAX_SPEED_VALUE / 100;
				if (i16MotorVal >= u16MotorSpdLimit) 
				{
					i16MotorVal = u16MotorSpdLimit;
				}
			}		
			
			if (i16MotorVal >= MOTOR_MAX_SPEED_VALUE)
			{
				i16MotorVal = MOTOR_MAX_SPEED_VALUE;
			}
//			//i32LogWrite(INFO, "MotorVal = %d\r\n", i16MotorVal);

			i16sgPcuVal = i16MotorVal;		/*赋值给全局变量*/	
		}	
	}
#elif (PCU_TYPE == PCU_TYPE_2)
	
		uint16_t u16MotorVal = 0;
		int32_t i32PropValue = 0;
	vEcuSetBeepPeriod();		
		if(1 == RevData->Data.b1Mode)
		{
			
		}
		else
		{
			
		}
		
		if(1 == RevData->Data.b1SlowSpdSwitch)
		{
			u8SlowSpeed ^= 1;
		}

		
		if(1 == RevData->Data.b1SpeakerSwitch)
		{
			i32DoPwmSet(SPEAKER_PUMP, 80);
		}
		else
		{
			i32DoPwmSet(SPEAKER_PUMP, 0);
		}
		
		
		if(1 == RevData->Data.b1TraSwitch)
		{
			u8Mode = MOVE_MODE;
		}

		if(1 == RevData->Data.b1LiftingSwitch)
		{
			u8Mode = LIFT_MODE;
		}

		if(1 == RevData->Data.b1EnableSwitch)
		{
			u8EnableFlag = 1;
		}
		else
		{
			u8EnableFlag = 0;
			/*Close All Pumps*/
			i32DoPwmSet(FORWARD_PUMP, 0);
			i32DoPwmSet(BACKWARD_PUMP, 0);
			i32DoPwmSet(TURNLIFT_PUMP, 0);
			i32DoPwmSet(TURNRIGHT_PUMP, 0);
			i32DoPwmSet(LIFTUP_PUMP, 0);
			vPropSetTarget(PropDriverCh0, 0);
		}
		
		if(1 == RevData->Data.b1TurnRightSwitch)
		{
			if((1 == u8EnableFlag) && (MOVE_MODE == u8Mode))
			{
				i32DoPwmSet(TURNRIGHT_PUMP, 80);
			}
			else
			{
				i32DoPwmSet(TURNRIGHT_PUMP, 0);
			}
		}
		else
		{
			i32DoPwmSet(TURNRIGHT_PUMP, 0);
		}
		
		if(1 == RevData->Data.b1TurnLeftSwitch)
		{
			if((1 == u8EnableFlag) && (MOVE_MODE == u8Mode))
			{
				i32DoPwmSet(TURNLIFT_PUMP, 80);
			}
			else
			{
				i32DoPwmSet(TURNLIFT_PUMP, 0);
			}
		}
		else
		{
			i32DoPwmSet(TURNLIFT_PUMP, 0);
		}
		
		
		u26MotorVal = RevData->Data.u8HandleCtrlHigh << 8 | RevData->Data.u8HandleCtrlLow;
		
		if((1 == u8EnableFlag) && (MOVE_MODE == u8Mode))
		{
			if (u16MotorVal > 0)
			if ((u16MotorVal >= 0x70C0) && (u16MotorVal <= 0x77CF))
			{
				i32DoPwmSet(FORWARD_PUMP, 80);
			}
			else if((u16MotorVal >= 0x78C1) && (u16MotorVal <= 0x7FCF))
			{
				i32DoPwmSet(BACKWARD_PUMP, 80);
			}
		}
		else if((1 == u8EnableFlag) && (LIFT_MODE == u8Mode))
		{
			if ((u16MotorVal >= 0x70C0) && (u16MotorVal <= 0x77CF))
			{
				i32DoPwmSet(LIFTUP_PUMP, 80);
			}
			else if((u16MotorVal >= 0x78C1) && (u16MotorVal <= 0x7FCF))
			{
				/*Set Current Value*/
				i32PropValue = 2.5 * (0x7FCF - u16MotorVal) / 1807;
				vPropSetTarget(PropDriverCh0, i32PropValue);
			}
		}
		else
		{
			i32DoPwmSet(FORWARD_PUMP, 0);
			i32DoPwmSet(BACKWARD_PUMP, 0);
			i32DoPwmSet(LIFTUP_PUMP, 0);
			vPropSetTarget(PropDriverCh0, 0);
		}
		
		vMstSlvSetMotorVal(u16MotorVal);
#else
	
#endif
}

/*发送给PCU的回调函数*/
static void vPcuSendProc(xPcuSendPara *SendData)
{
	if ((SLOW_SPEED == sgSlowKey.b1PcuSpeed) || (1 == i32LocalDiGet(PIT_SWITCH)))
	{
		SendData->Data.b1SlowLed = 1;
	}
	else
	{
		SendData->Data.b1SlowLed = 0;
	}
	
	if(MOVE_MODE == u8PcuMode)
	{
		SendData->Data.b1ModeLed = 1;
		SendData->Data.b1LiftLed = 0;
	}
	else
	{
		SendData->Data.b1ModeLed = 0;
		SendData->Data.b1LiftLed = 1;
	}
}
/*******************************************************************************
* void MstRevProc(xMstRevParat *RevData)
* Descriptio: 接收电控MCU数据进行处理
* Input: RevData：电控MCU接收的数据格式
* Output: NULL 
*******************************************************************************/
static void vMstRevProc(xMstRevPara *RevData)
{	
//	//i32LogWrite(INFO, "Rev Mst Success!\r\n");
	uint16_t tmp = 0;
//	static uint8_t u8OldErrCode = 0;
	
	
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
	/**/
	if(0 != RevData->u8ErrCode)
	{
		if (26 == RevData->u8ErrCode)		/*控制器通讯故障报85*/
		{
			i32ErrCodeSet(ErrCode85);
		}
		else								/*其它报81*/
		{
			i32ErrCodeSet(ErrCode81);
		}	
	}
	
	
	u16MotorSpd = (uint16_t)(RevData->u8SpeedFdbHigh << 8) | RevData->u8SpeedFdbLow ; 
	tmp = u16MotorSpd * 100 / MOTOR_MAX_SPEED;
//	{
//		static uint32_t tmpCnt = 0;
//		//i32LogWrite(INFO, "Rev Mst Spd = %d****************, Cnt = %d\r\n", u16MotorSpd, tmpCnt++);
//	}
	i32SetPara(PARA_MotorSpd, tmp);
	gCanSendPdoInfo.CanSendInfo3.u8MotorSpd = tmp;
	
	tmp = RevData->u8MotorTmp;
	
	tmp = RevData->u8BoardTmp;
}
/*下降阀处理*/
static void vLiftDownProc(void)
{
	int32_t i32PropValue = 0;

	i32PropValue = _IQ((sgUserInfo.fPropMinCurrent + (sgUserInfo.fPropMaxCurrent - sgUserInfo.fPropMinCurrent) * i16sgPcuVal / MOTOR_MAX_SPEED_VALUE) / PROPD_STD_CURRENT);
	sgSpeakerFun.b1240PerMin = 1;	
	vPropSetTarget(LIFTDOWN_PUMP, i32PropValue);
//	//i32LogWrite(INFO, "Down Value = %d, u8AntiPinchStat = %d\r\n", i32PropValue, u8AntiPinchStat);
	i32SetPara(PARA_PropValveCurrent, (inserted_data[1] * PROP_CURRENT_FACOTR));
	gCanSendPdoInfo.CanSendInfo2.u8PropValveCurrent = (inserted_data[1] * PROP_CURRENT_FACOTR) * 100;/*Uint 0.1A*/

	/*lilu 20230703 */
	if (0 == i32LocalDiGet(DOWN_LIMIT_SWITCH))
	{
		u8AntiPinchStat = ANTI_PINCH_STAT0;
	}

	if ((1 == i32LocalDiGet(DOWN_LIMIT_SWITCH)) && (FunctionEnable == i32GetPara(PARA_AntiPinchFunc)))/*防夹手功能*/
	{
		//if(0 == u8AntiPinchFlag)
		if (ANTI_PINCH_STAT3 != u8AntiPinchStat)	/*lilu, 20230703, add funcions*/
		{
			if (ANTI_PINCH_STAT0 == u8AntiPinchStat)
			{
				u8AntiPinchStat = ANTI_PINCH_STAT1;
			}
			else if (ANTI_PINCH_STAT1 == u8AntiPinchStat)	/*lilu, 20230703, add funcions*/
			{
				vPropSetTarget(LIFTDOWN_PUMP, 0);
			}
			else
			{
				if(false == u8GetNetTimerStartFlag(TIMER_EcuAntiPinchFunc))
				{
					vSetNetTimer(TIMER_EcuAntiPinchFunc, ECU_ANTI_PINCH_TIME);
				}
				if(true == u8GetNetTimerOverFlag(TIMER_EcuAntiPinchFunc))
				{
					vKillNetTimer(TIMER_EcuAntiPinchFunc);
					u8AntiPinchStat = ANTI_PINCH_STAT3;
				}
				else
				{
					vPropSetTarget(LIFTDOWN_PUMP, 0);
				}
			}
		}
	}
}
/*阀状态发生变化处理*/
static void vPcuStatNoChangeProc(int16_t *i16Spd, float *fMotorSpd, const xValvesInfo *pValvesInfo)
{
	/*举升后行走*/
	if (((1 == pValvesInfo->b1ForWardStat) || (1 == pValvesInfo->b1BackWardStat)) && 
		((1 == i32LocalDiGet(PIT_SWITCH)) || (1 == sgValvesInfo.b1LiftSpdFlag)))
	{
		if (*i16Spd > i16sgPcuVal) 
		{
			if (((*i16Spd - i16sgPcuVal) > sgUserInfo.fPitSpdPer5msDecStep) && (0 != sgUserInfo.fPitSpdPer5msDecStep))
			{
				*fMotorSpd -= sgUserInfo.fPitSpdPer5msDecStep;
				*i16Spd = (int16_t)*fMotorSpd;
			}
			else
			{
				*i16Spd = i16sgPcuVal;
				*fMotorSpd = i16sgPcuVal;
			}
		}
		else if (*i16Spd < i16sgPcuVal)
		{
			if (((i16sgPcuVal - *i16Spd) > sgUserInfo.fPitSpdPer5msAccStep) && (0 != sgUserInfo.fPitSpdPer5msAccStep))
			{
				*fMotorSpd += sgUserInfo.fPitSpdPer5msAccStep;
				*i16Spd = (int16_t)*fMotorSpd;
			}
			else
			{
				*i16Spd = i16sgPcuVal;
				*fMotorSpd = i16sgPcuVal;
			}
		}
	}
	/*慢速行走*/
	else if (((1 == pValvesInfo->b1ForWardStat) || (1 == pValvesInfo->b1BackWardStat)) && (SLOW_SPEED == sgSlowKey.b1PcuSpeed))
	{

		if (*i16Spd > i16sgPcuVal) 
		{
			if (((*i16Spd - i16sgPcuVal) > sgUserInfo.fSlowSpdPer5msDecStep) && (0 != sgUserInfo.fSlowSpdPer5msDecStep))
			{
				*fMotorSpd -= sgUserInfo.fSlowSpdPer5msDecStep;
				*i16Spd = (int16_t)*fMotorSpd;
			}
			else
			{
				*i16Spd = i16sgPcuVal;
				*fMotorSpd = i16sgPcuVal;
			}
		}
		else if (*i16Spd < i16sgPcuVal)
		{
			if (((i16sgPcuVal - *i16Spd) > sgUserInfo.fSlowSpdPer5msAccStep) && (0 != sgUserInfo.fSlowSpdPer5msAccStep))
			{
				*fMotorSpd += sgUserInfo.fSlowSpdPer5msAccStep;
				*i16Spd = (int16_t)*fMotorSpd;
			}
			else
			{
				*i16Spd = i16sgPcuVal;
				*fMotorSpd = i16sgPcuVal;
			}
		}
	}
	/*快速行走*/
	else if ((1 == pValvesInfo->b1ForWardStat) || (1 == pValvesInfo->b1BackWardStat))
	{

		if (*i16Spd > i16sgPcuVal) 
		{
			if (((*i16Spd - i16sgPcuVal) > sgUserInfo.fFastSpdPer5msDecStep) && (0 != sgUserInfo.fFastSpdPer5msDecStep))
			{
				*fMotorSpd -= sgUserInfo.fFastSpdPer5msDecStep;
				*i16Spd = (int16_t)*fMotorSpd;
			}
			else
			{
				*i16Spd = i16sgPcuVal;
				*fMotorSpd = i16sgPcuVal;
			}
		}
		else if (*i16Spd < i16sgPcuVal)
		{
			if (((i16sgPcuVal - *i16Spd) > sgUserInfo.fFastSpdPer5msAccStep) && (sgUserInfo.fFastSpdPer5msAccStep))
			{
				*fMotorSpd += sgUserInfo.fFastSpdPer5msAccStep;
				*i16Spd = (int16_t)*fMotorSpd;
			}
			else
			{
				*i16Spd = i16sgPcuVal;
				*fMotorSpd = i16sgPcuVal;
			}
		} 
	}
	/*举升*/
	else if (1 == pValvesInfo->b1LiftUpStat)
	{
		if (*i16Spd > i16sgPcuVal) 
		{
			if (((*i16Spd - i16sgPcuVal) > sgUserInfo.fLiftSpdPer5msDecStep) && (0 != sgUserInfo.fLiftSpdPer5msDecStep))
			{
				*fMotorSpd -= sgUserInfo.fLiftSpdPer5msDecStep;
				*i16Spd = (int16_t)*fMotorSpd;
			}
			else
			{
				*i16Spd = i16sgPcuVal;
				*fMotorSpd = i16sgPcuVal;
			}
		}
		else if (*i16Spd < i16sgPcuVal)
		{
			if (((i16sgPcuVal - *i16Spd) > sgUserInfo.fLiftSpdPer5msAccStep) && (0 != sgUserInfo.fLiftSpdPer5msAccStep))
			{
				*fMotorSpd += sgUserInfo.fLiftSpdPer5msAccStep;
				*i16Spd = (int16_t)*fMotorSpd;
			}
			else
			{
				*i16Spd = i16sgPcuVal;
				*fMotorSpd = i16sgPcuVal;
			}
		}
	}
	/*下降*/
	else if (1 == pValvesInfo->b1LiftDownStat)	
	{
		vLiftDownProc();
		*i16Spd = 0;
		*fMotorSpd = 0.0;
	}
	else if ((0 == pValvesInfo->b1ForWardStat) && (0 == pValvesInfo->b1BackWardStat) &&
			(0 == pValvesInfo->b1TurnLeftStat) && (0 == pValvesInfo->b1TurnRightStat) && 
			(0 == pValvesInfo->b1LiftUpStat) && (0 == pValvesInfo->b1LiftDownStat))
	{
		*i16Spd = 0;
		*fMotorSpd = 0.0;
		vPropSetTarget(LIFTDOWN_PUMP, 0);
	}
}
/*阀状态发生变化处理*/
static void vCtrlValvesByStat(const xValvesInfo *pValvesInfo, int16_t *i16Spd)
{
	if (1 == pValvesInfo->b1ForWardStat)
	{
		i32DoPwmSet(FORWARD_PUMP, PUMP_OPEN_PERCENTAGE);
		i32SetPara(PARA_ForwardValveCurrent, sgUserInfo.u16ValveOpenLoopCurrent);
		gCanSendPdoInfo.CanSendInfo2.u8ForwardValveCurrent = sgUserInfo.u16ValveOpenLoopCurrent / 10; /*Uint 0.1A*/
	}
	else
	{
		i32DoPwmSet(FORWARD_PUMP, PUMP_CLOSE_PERCENTAGE);
		i32SetPara(PARA_ForwardValveCurrent, 0);
		gCanSendPdoInfo.CanSendInfo2.u8ForwardValveCurrent = 0;
	}
	
	if (1 == pValvesInfo->b1BackWardStat)
	{
		i32DoPwmSet(BACKWARD_PUMP, PUMP_OPEN_PERCENTAGE);
		i32SetPara(PARA_BackValveCurrent, sgUserInfo.u16ValveOpenLoopCurrent);
		gCanSendPdoInfo.CanSendInfo2.u8BackwardValveCurrent = sgUserInfo.u16ValveOpenLoopCurrent / 10;
	}
	else
	{
		i32DoPwmSet(BACKWARD_PUMP, PUMP_CLOSE_PERCENTAGE);
		i32SetPara(PARA_BackValveCurrent, 0);
		gCanSendPdoInfo.CanSendInfo2.u8BackwardValveCurrent = 0;
	}
	
	if (1 == pValvesInfo->b1TurnLeftStat)
	{
		i32DoPwmSet(TURNLEFT_PUMP, PUMP_OPEN_PERCENTAGE);
		i32SetPara(PARA_TurnLeftValveCurrent, sgUserInfo.u16ValveOpenLoopCurrent);
		gCanSendPdoInfo.CanSendInfo2.u8TurnLeftValveCurrent = sgUserInfo.u16ValveOpenLoopCurrent / 10;
		*i16Spd = i16sgPcuVal;
	}
	else
	{
		i32DoPwmSet(TURNLEFT_PUMP, PUMP_CLOSE_PERCENTAGE);
		i32SetPara(PARA_TurnLeftValveCurrent, 0);
		gCanSendPdoInfo.CanSendInfo2.u8TurnLeftValveCurrent = 0;
//		*i16Spd = i16sgPcuVal;
	}
	
	if (1 == pValvesInfo->b1TurnRightStat)
	{
		i32DoPwmSet(TURNRIGHT_PUMP, PUMP_OPEN_PERCENTAGE);
		i32SetPara(PARA_TurnRightValveCurrent, sgUserInfo.u16ValveOpenLoopCurrent);
		gCanSendPdoInfo.CanSendInfo2.u8TurnRightValveCurrent = sgUserInfo.u16ValveOpenLoopCurrent / 10;
		*i16Spd = i16sgPcuVal;
	}
	else
	{
		i32DoPwmSet(TURNRIGHT_PUMP, PUMP_CLOSE_PERCENTAGE);
		i32SetPara(PARA_TurnRightValveCurrent, 0);
		gCanSendPdoInfo.CanSendInfo2.u8TurnRightValveCurrent = 0;
//		*i16Spd = i16sgPcuVal;
	}
	
	if (1 == pValvesInfo->b1LiftUpStat)
	{
		i32DoPwmSet(LIFTUP_PUMP, PUMP_OPEN_PERCENTAGE);
		i32SetPara(PARA_LiftValveCurrent, sgUserInfo.u16ValveOpenLoopCurrent);
		gCanSendPdoInfo.CanSendInfo2.u8LiftValveCurrent = sgUserInfo.u16ValveOpenLoopCurrent / 10;
	}
	else
	{
		i32DoPwmSet(LIFTUP_PUMP, PUMP_CLOSE_PERCENTAGE);
		i32SetPara(PARA_LiftValveCurrent, 0);
		gCanSendPdoInfo.CanSendInfo2.u8LiftValveCurrent = 0;
	}
	
	if (1 == pValvesInfo->b1LiftDownStat)
	{
		vLiftDownProc();
	}
	else
	{
		vPropSetTarget(LIFTDOWN_PUMP, 0);
		i32SetPara(PARA_PropValveCurrent, 0);
		gCanSendPdoInfo.CanSendInfo2.u8PropValveCurrent = 0;
	}
}

/*发送给Mst的回调函数*/
static void vMstSendProc(xMstSendPara *SendData) 
{
	static xValvesInfo sgLastValvesInfo;
	static float fMotorSpd = 0.0;
	
	int16_t i16Spd = 0;
	int16_t i16Tmp = 0;
	
	uint16_t u16SwitchFlag = 0;
	
	/*小于0， 变成正数*/
	if (i16sgPcuVal < 0)
	{
		i16sgPcuVal = 0 - i16sgPcuVal;
	}
	
	SendData->buf[2] = 0;
	i16Spd = (SendData->u8TargetHigh << 8) | SendData->u8TargetLow;
//	//i32LogWrite(INFO, "Send Motor Value = %d\r\n", i16Spd);
	if (0 == memcmp(&sgLastValvesInfo, &sgValvesInfo, sizeof(sgLastValvesInfo)))	/*一样就是油门控制速度*/
	{		
		vPcuStatNoChangeProc(&i16Spd, &fMotorSpd, &sgLastValvesInfo);
	}
	else	/*不一样就是状态切换过程*/
	{
		if ((sgLastValvesInfo.b1ForWardStat != sgValvesInfo.b1ForWardStat) ||
			(sgLastValvesInfo.b1BackWardStat != sgValvesInfo.b1BackWardStat) || 
			(sgLastValvesInfo.b1LiftUpStat != sgValvesInfo.b1LiftUpStat) ||
			(sgLastValvesInfo.b1LiftDownStat != sgValvesInfo.b1LiftDownStat) ||
			(sgLastValvesInfo.b1TurnLeftStat != sgValvesInfo.b1TurnLeftStat) ||
			(sgLastValvesInfo.b1TurnRightStat != sgValvesInfo.b1TurnRightStat))	
		{
			/*lilu 20230713*/
			if ((u16MotorSpd <= MOTOR_MIN_SPEED) && ((inserted_data[1] * PROP_CURRENT_FACOTR) < sgUserInfo.fPropMinCurrent))				/*电机速度为0， 比例阀没有下降动作*/
//			if (u16MotorSpd <= MOTOR_MIN_SPEED)
			{
				u16SwitchFlag |= 1 << 0;
			}
			else if ((sgLastValvesInfo.b1ForWardStat == sgValvesInfo.b1ForWardStat) &&							/*前进后退不变，左或右发生变化*/
					 (sgLastValvesInfo.b1BackWardStat == sgValvesInfo.b1BackWardStat) &&
					 ((sgLastValvesInfo.b1TurnLeftStat != sgValvesInfo.b1TurnLeftStat) ||
					 (sgLastValvesInfo.b1TurnRightStat != sgValvesInfo.b1TurnRightStat)))
			{
				u16SwitchFlag |= 1 << 1;
			}
			else if (((1 == sgLastValvesInfo.b1TurnLeftStat) || (1 == sgLastValvesInfo.b1TurnRightStat)) && 	/*左转或右转打开，然后打开前进或者后退*/
					 (((0 == sgLastValvesInfo.b1ForWardStat) && (1 == sgValvesInfo.b1ForWardStat)) || 
					 ((0 == sgLastValvesInfo.b1BackWardStat) && (1 == sgValvesInfo.b1BackWardStat))))
			{
				if (u16MotorSpd <= (i32GetPara(PARA_MaxTurnSpeed) * MOTOR_MAX_SPEED / 100))
				{
					u16SwitchFlag |= 1 << 2;
				}
			}
			else if (((1 == sgLastValvesInfo.b1TurnLeftStat) || (1 == sgLastValvesInfo.b1TurnRightStat)) && 	/*左转或右转打开，关闭前进或者后退*/
					 (((1 == sgLastValvesInfo.b1ForWardStat) && (0 == sgValvesInfo.b1ForWardStat)) || 
					 ((1 == sgLastValvesInfo.b1BackWardStat) && (0 == sgValvesInfo.b1BackWardStat))))
			{
				if (u16MotorSpd <= (i32GetPara(PARA_MaxTurnSpeed) * MOTOR_MAX_SPEED / 100))
				{
					u16SwitchFlag |= 1 << 3;
				}
			}

			if (0 != u16SwitchFlag)
			{
				memcpy(&sgLastValvesInfo, &sgValvesInfo, sizeof(sgLastValvesInfo));
				vCtrlValvesByStat(&sgLastValvesInfo, &i16Spd);
			}
			else 
			{
				/*举升后行走*/
				if (((1 == sgLastValvesInfo.b1ForWardStat) || (1 == sgLastValvesInfo.b1BackWardStat)) && 
					((1 == i32LocalDiGet(PIT_SWITCH)) || (1 == sgValvesInfo.b1LiftSpdFlag)))
				{
					if ((u16MotorSpd > MOTOR_MIN_SPEED) && (i16Spd >= sgUserInfo.fPitSpdPer5msDecStep))
					{
						fMotorSpd -= sgUserInfo.fPitSpdPer5msDecStep;
						i16Spd = (int16_t)fMotorSpd;
						//i32LogWrite(INFO, "Pit Send Motor Value = %d, Motor Spd = %d\r\n", i16Spd, u16MotorSpd);
					}
					else
					{
						i16Spd = 0;
						fMotorSpd = 0;
					}
				}
				/*慢速行走*/
				else if (((1 == sgLastValvesInfo.b1ForWardStat) || (1 == sgLastValvesInfo.b1BackWardStat)) && (SLOW_SPEED == sgSlowKey.b1PcuSpeed))
				{
					if ((u16MotorSpd > MOTOR_MIN_SPEED) && (i16Spd >= sgUserInfo.fSlowSpdPer5msDecStep))
					{
						fMotorSpd -= sgUserInfo.fSlowSpdPer5msDecStep;
						i16Spd = (int16_t)fMotorSpd;
						//i32LogWrite(INFO, "Slow Send Motor Value = %d, Motor Spd = %d\r\n", i16Spd, u16MotorSpd);
					}
					else
					{
						i16Spd = 0;
						fMotorSpd = 0;
					}
				}
				/*快速行走*/
				else if ((1 == sgLastValvesInfo.b1ForWardStat) || (1 == sgLastValvesInfo.b1BackWardStat))
				{

 					if ((u16MotorSpd > MOTOR_MIN_SPEED) && (i16Spd >= sgUserInfo.fFastSpdPer5msDecStep))
					{
						fMotorSpd -= sgUserInfo.fFastSpdPer5msDecStep;
						i16Spd = (int16_t)fMotorSpd;
						//i32LogWrite(INFO, "Fast Send Motor Value = %d, Motor Spd = %d\r\n", i16Spd, u16MotorSpd);
					}
					else
					{
						i16Spd = 0;
						fMotorSpd = 0;
					}
				}
				/*举升*/
				else if (1 == sgLastValvesInfo.b1LiftUpStat)
				{
 					if ((u16MotorSpd > MOTOR_MIN_SPEED) && (i16Spd >= sgUserInfo.fLiftSpdPer5msDecStep))
					{
						fMotorSpd -= sgUserInfo.fLiftSpdPer5msDecStep;
						i16Spd = (int16_t)fMotorSpd;
						//i32LogWrite(INFO, "Lift Send Motor Value = %d, Motor Spd = %d\r\n", i16Spd, u16MotorSpd);
					}
					else
					{
						i16Spd = 0;
						fMotorSpd = 0;
					}
				}
				else if (1 == sgLastValvesInfo.b1LiftDownStat)
				{
					vPropSetTarget(LIFTDOWN_PUMP, 0);
					i16Spd = 0;
					fMotorSpd = 0;
				}
			}
		}	
	}
	
	if (i16Spd >= MOTOR_MAX_SPEED_VALUE)
	{
		i16Spd = MOTOR_MAX_SPEED_VALUE;
	}
//	//i32LogWrite(INFO, "Send Last Motor Value = %d\r\n", i16Spd);
	SendData->u8TargetHigh = i16Spd >> 8;
	SendData->u8TargetLow = i16Spd;
	
	if (0 != i16Spd)
	{
		SendData->b1ServoOn = 1;
		SendData->b1ForwardReq = 1;
	}
}

static void vDoPwmErrCallBack(eDoPwmNo DoPwmNo)
{
	
}

static void vPropErrCallBack(uint8_t u8Channel)
{
	
}

/*蜂鸣器回调函数*/
static void vBeepCallBack(uint8_t u8Flag)
{
	if(1 == u8Flag)
	{
		i32DoPwmSet(BLINK_BEEP, PUMP_OPEN_PERCENTAGE);
	}
	else
	{
		i32DoPwmSet(BLINK_BEEP, PUMP_CLOSE_PERCENTAGE);
	}
}
/*闪光灯回调函数*/
static void vAlarmLampCallBack(uint8_t u8Flag)
{
	if(1 == u8Flag)
	{
		i32DoPwmSet(BLINK_LED, PUMP_OPEN_PERCENTAGE);
	}
	else
	{
		i32DoPwmSet(BLINK_LED, PUMP_CLOSE_PERCENTAGE);
	}
}
/*接收PDO处理函数*/
static void vCanRevPdoProc(void)
{
	
	static xBmsInfo1 BmsInfo1Last;
	static xBmsInfo2 BmsInfo2Last;
	
	if (LiBattery == i32GetPara(PARA_BatteryType))
	{
		if(0 != memcmp((char*)BmsInfo1Last.u8Data, (char*)gCanRevPdoInfo.CanRevInfo1.u8Data, sizeof(BmsInfo1Last)))
		{
			memcpy(BmsInfo1Last.u8Data, gCanRevPdoInfo.CanRevInfo1.u8Data, sizeof(BmsInfo1Last));
			i32SetPara(PARA_BmsVoltage, BmsInfo1Last.u16BatteryVol);
		}
		
		if(0 != memcmp((char*)BmsInfo2Last.u8Data, (char*)gCanRevPdoInfo.CanRevInfo2.u8Data, sizeof(BmsInfo2Last)))
		{
			memcpy(BmsInfo2Last.u8Data, gCanRevPdoInfo.CanRevInfo2.u8Data, sizeof(BmsInfo2Last));
			
			i32SetPara(PARA_BmsSoc, BmsInfo2Last.u8Soc);
			
			if (1 == BmsInfo2Last.b1BatTemp1Err)
			{
				i32ErrCodeSet(BMS_BATTERY_TEMP_HIGH1_ERR);
			}
			else
			{
				i32ErrCodeClr(BMS_BATTERY_TEMP_HIGH1_ERR);
			}
			
			if(1 == BmsInfo2Last.b1BatDisChargeCurHigh1Err)
			{
				i32ErrCodeSet(BMS_DISCHARGE_CUR_HIGH1_ERR);
			}
			else
			{
				i32ErrCodeClr(BMS_DISCHARGE_CUR_HIGH1_ERR);
			}
			
			if(1 == BmsInfo2Last.b1BatTotalVolLow1Err)
			{
				i32ErrCodeSet(BMS_TOTAL_VOL_LOW1_ERR);
			}
			else
			{
				i32ErrCodeClr(BMS_TOTAL_VOL_LOW1_ERR);
			}
			
			if(1 == BmsInfo2Last.b1BatSingleVolLow1Err)
			{
				i32ErrCodeSet(BMS_SINGLE_VOL_LOW1_ERR);
			}
			else
			{
				i32ErrCodeClr(BMS_SINGLE_VOL_LOW1_ERR);
			}
			
			if(1 == BmsInfo2Last.b1BatSingleVolLow2Err)
			{
				i32ErrCodeSet(BMS_SINGLE_VOL_LOW2_ERR);
			}
			else
			{
				i32ErrCodeClr(BMS_SINGLE_VOL_LOW2_ERR);
			}
			
			if(1 == BmsInfo2Last.b1BatVolDiffErr)
			{
				i32ErrCodeSet(BMS_BATTERY_VOL_DIFF_HIGH_ERR);
			}
			else
			{
				i32ErrCodeClr(BMS_BATTERY_VOL_DIFF_HIGH_ERR);
			}
			
			if(1 == BmsInfo2Last.b1BatTempDiffErr)
			{
				i32ErrCodeSet(BMS_BATTERY_TEMP_DIFF2_ERR);
			}
			else
			{
				i32ErrCodeClr(BMS_BATTERY_TEMP_DIFF2_ERR);
			}
			
			if(1 == BmsInfo2Last.b1BatDisChargeCurHigh2Err)
			{
				i32ErrCodeSet(BMS_DISCHARGE_CUR_HIGH2_ERR);
			}
			else
			{
				i32ErrCodeClr(BMS_DISCHARGE_CUR_HIGH2_ERR);
			}
			
			if(1 == BmsInfo2Last.b1BatDisChargeTempHigh2Err)
			{
				i32ErrCodeSet(BMS_DISCHARGE_TEMP_HIGH2_ERR);
			}
			else
			{
				i32ErrCodeClr(BMS_DISCHARGE_TEMP_HIGH2_ERR);
			}
			
			if(1 == BmsInfo2Last.b1BatTotalVolLow2Err)
			{
				i32ErrCodeSet(BMS_TOTAL_VOL_LOW2_ERR);
			}
			else
			{
				i32ErrCodeClr(BMS_TOTAL_VOL_LOW2_ERR);
			}
		}
	}
}

/*压力传感器回调函数*/
static void vAngleCallBack(uint8_t u8Type)
{
//	static uint8_t u8LastAngleType = 0;
//	if (1 == i32LocalDiGet(PIT_SWITCH))
//	{
//		if (u8LastAngleType != u8Type)
//		{
//			if(1 == u8Type)
//			{
//				i32ErrCodeSet(ANGLE_SENSOR_ERR);
//			}
////			else
////			{
////				i32ErrCodeClr(ANGLE_SENSOR_ERR);
////			}
//			u8LastAngleType = u8Type;
//		}
//	}
}
/*压力传感器回调函数*/
static void vPressureCallBack(ePressureNo PressureNo)
{
	static xPresureErrInfo LastErrFlag = {0};
	xPresureErrInfo ErrFlag = {0};

	switch ((uint8_t)PressureNo)
	{
		case PressureCaliReverse:
			ErrFlag.b1CaliReverse = 1;
			break;
		case PressureCaliFailure:
			ErrFlag.b1CaliFailure = 1;
			break;
		case PressureWithoutSensor:
			if (1 == i32LocalDiGet(PIT_SWITCH))	/*lilu 20230706 增加坑洞判断*/
			{
				ErrFlag.b1SenSorErr = 1;
			}
			break;
		case PressureOverPer80:
			ErrFlag.b1Per80Err = 1;
			break;
		case PressureOverPer90:
			ErrFlag.b1Per90Err = 1;
			break;
		case PressureOverPer99:
			ErrFlag.b1Per99Err = 1;
			break;
		case PressureOverPer100:
			ErrFlag.b1Per100Err = 1;
			break;
		default:
			break;
	}
	
	if (LastErrFlag.u8Data != ErrFlag.u8Data)
	{
		if(1 == ErrFlag.b1SenSorErr)
		{
			i32ErrCodeSet(PRESSURE_SENSOR_ERR);
		}
		else
		{
			i32ErrCodeClr(PRESSURE_SENSOR_ERR);
		}
		
		if(1 == ErrFlag.b1CaliReverse)
		{
			i32ErrCodeSet(WEIGHT_CALI_REVESER_ERR);
		}
		else
		{
			i32ErrCodeClr(WEIGHT_CALI_REVESER_ERR);
		}
	
		if(1 == ErrFlag.b1CaliFailure)
		{
			i32ErrCodeSet(CALIBRATION_FAILURE_ERR);
		}
		else
		{
			i32ErrCodeClr(CALIBRATION_FAILURE_ERR);
		}
		
		if(1 == ErrFlag.b1Per80Err)
		{
			i32ErrCodeSet(OVER_80_PER_LOAD_ERR);
		}
		else
		{
			i32ErrCodeClr(OVER_80_PER_LOAD_ERR);
		}
		
		if(1 == ErrFlag.b1Per90Err)
		{
			i32ErrCodeSet(OVER_90_PER_LOAD_ERR);
		}
		else
		{
			i32ErrCodeClr(OVER_90_PER_LOAD_ERR);
		}
		
		if(1 == ErrFlag.b1Per99Err)
		{
			i32ErrCodeSet(OVER_99_PER_LOAD_ERR);
		}
		else
		{
			i32ErrCodeClr(OVER_99_PER_LOAD_ERR);
		}
		
		if(1 == ErrFlag.b1Per100Err)
		{
			i32ErrCodeSet(PLAT_OVERLOAD_ERR);
			
		}
		else
		{
			i32ErrCodeClr(PLAT_OVERLOAD_ERR);
		}
		
		LastErrFlag.u8Data = ErrFlag.u8Data;
	}
}

static void vDoPwmErrProc(eDoPwmNo DoPwmNo)
{
	switch((uint8_t) DoPwmNo)
	{
		case FORWARD_PUMP:
			i32ErrCodeSet(FORWARD_VALVE_ERR);
			break;
		case BACKWARD_PUMP:
			i32ErrCodeSet(BACKWARD_VALVE_ERR);
			break;
		case TURNLEFT_PUMP:
			i32ErrCodeSet(TURN_LEFT_VALVE_ERR);
			break;
		case TURNRIGHT_PUMP:
			i32ErrCodeSet(TURN_RIGHT_VALVE_ERR);
			break;
		case LIFTUP_PUMP:
			i32ErrCodeSet(LIFT_UP_VALVE_ERR);
			break;
		default:
			break;
	}
}

/*lilu 20230703 模拟量监控*/
static void vAiMonitor(void)
{
	uint16_t u16AdcValue = 0;
	
	u16AdcValue = i32LocalAiGetValue(AI_B_VBUS_CHECK);
	i32SetPara(PARA_BatteryVoltage, u16AdcValue);
	if (u16AdcValue < 18000)		/*lilu 20230703 低于二级报警电压*/
	{
		i32ErrCodeSet(BAT_LOW_CAP2_ERR);
	}
	else
	{
		i32ErrCodeClr(BAT_LOW_CAP2_ERR);
	}
}
/*开关量监控*/
static void vSwiMonitor(void)
{
	uint16_t u16ExtInput = 0;
	

	if(1 == i32LocalDiGet(PCU_SWICTH))
	{
		u16ExtInput |= 1 << SWI1_EXTINPUT;
	}

	if(1 == i32LocalDiGet(TILT_SIWTCH))
	{
		u16ExtInput |= 1 << SWI2_EXTINPUT;
	}
	
	if(1 == i32LocalDiGet(PIT_SWITCH))
	{
		u16ExtInput |= 1 << SWI5_EXTINPUT;
	}
	
	if(1 == i32LocalDiGet(UP_LIMIT_SWITCH))
	{
		u16ExtInput |= 1 << SWI7_EXTINPUT;
	}

		
	if(1 == i32LocalDiGet(DOWN_LIMIT_SWITCH))
	{
		u16ExtInput |= 1 << SWI8_EXTINPUT;
	}
	
	if (0 == (u16ExtInput & (1 << SWI8_EXTINPUT)))
	{
		if (0 == (u16ExtInput & (1 << SWI2_EXTINPUT)))		/*Tilt*/
		{
			sgSpeakerFun.b1180PerMin = 1;
			i32ErrCodeSet(MACHINE_TILT_OVER_SAFETY_ERR);
			if (0 == (u16ExtInput & (1 << SWI5_EXTINPUT)))
			{
				if (FunctionEnable == i32GetPara(PARA_PitProtectFunc))		/*坑洞保护功能*/
				{
					i32ErrCodeSet(PIT_PROCETION_ERR);
				}
			}
			else
			{
				i32ErrCodeClr(PIT_PROCETION_ERR);
			}
		}
		else
		{
			sgSpeakerFun.b1180PerMin = 0;
			i32ErrCodeClr(MACHINE_TILT_OVER_SAFETY_ERR);
			if (0 == (u16ExtInput & (1 << SWI5_EXTINPUT)))
			{
				if (FunctionEnable == i32GetPara(PARA_PitProtectFunc))		/*坑洞保护功能*/
				{
					sgSpeakerFun.b1180PerMin = 1;
					i32ErrCodeSet(PIT_PROCETION_ERR);
				}
			}
			else
			{
				i32ErrCodeClr(PIT_PROCETION_ERR);
			}
		}
	}
	else
	{
		sgSpeakerFun.b1180PerMin = 0;
		i32ErrCodeClr(PIT_PROCETION_ERR);
		i32ErrCodeClr(MACHINE_TILT_OVER_SAFETY_ERR);
	}
	/*Set External Input*/
	i32SetPara(PARA_ExtSignal, u16ExtInput);
}


/*设置蜂鸣器的周期，根据不同的频率，设置不同的蜂鸣器频率*/
static void vEcuSetBeepPeriod(void)
{
	if(sgSpeakerFun.b1240PerMin)		/*下降动作*/
	{
		vBeepSetPeriod(240);		
		sgSpeakerFun.u8Cnt++;
		if(sgSpeakerFun.u8Cnt >= 180)
		{
			sgSpeakerFun.u8Cnt = 0;
		}
		if(sgSpeakerFun.u8Cnt < 144)
		{
			vBeepSetPeriod(240);
		}
		else
		{
			vBeepSetPeriod(0);
		}
	}
	else if(sgSpeakerFun.b1180PerMin)	/*可恢复报警*/
	{
		vBeepSetPeriod(180);
	}
	else if(sgSpeakerFun.b160PerMin)	/*不可恢复报警*/
	{
		vBeepSetPeriod(60);
	}
	else if((sgSpeakerFun.b130PerMin) && (FunctionEnable == i32GetPara(PARA_ActAlmFunc)))		/*动作报警*/
	{
		vBeepSetPeriod(30);
	}
	else
	{
		vBeepSetPeriod(0);
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
	vPcuErrRegister(vPcuErrProc);	
	vPcuRevRegister(vPcuRevProc);
	vPcuSendRegister(vPcuSendProc);
	
	vMstRevRegister(vMstRevProc);
	vMstSendRegister(vMstSendProc);
	
	vDoPwmErrReg(vDoPwmErrCallBack);
	vPropErrReg(vPropErrCallBack);
	
	
	
	vAngleSensorReg(vAngleCallBack);
	vPressureSensorReg(vPressureCallBack);
	
	vBeepRegister(vBeepCallBack);
	vAlarmLampRegister(vAlarmLampCallBack);
	vAlarmLampSetPeriod(30);			/*StartUp 30/min*/
	memset(&sgValvesInfo, 0, sizeof(sgValvesInfo));
	
	sgUserInfo.fPropMinCurrent = i32GetPara(PROP_MIN_CURRENT) / 1000.0;
	sgUserInfo.fPropMaxCurrent = i32GetPara(PROP_MAX_CURRENT) / 1000.0;
	
	sgUserInfo.u16PropMinADC = sgUserInfo.fPropMinCurrent / PROP_CURRENT_FACOTR;
	sgUserInfo.u16PropMaxADC = (sgUserInfo.fPropMaxCurrent + 0.05)/ PROP_CURRENT_FACOTR;		/*最大电流增加50ma阈值*/

	sgUserInfo.u16ValveOpenLoopCurrent = i32GetPara(PARA_ValueOpenLoopCurrent);
	
	/*Para Initial*/
	if (0 != i32GetPara(PARA_AccAndDecFastDrive))
	{
		sgUserInfo.fFastSpdPer5msAccStep = STEP_FACTOR * i32GetPara(PARA_CurveFastDrive) / i32GetPara(PARA_AccAndDecFastDrive);
		sgUserInfo.fFastSpdPer5msDecStep = STEP_FACTOR * i32GetPara(PARA_BrakeFastDrive) / i32GetPara(PARA_AccAndDecFastDrive);
	}
	else
	{
		//i32LogWrite(ERR, "Fast Spd Period is 0!!!\r\n");
		sgUserInfo.fFastSpdPer5msAccStep = 0;
		sgUserInfo.fFastSpdPer5msDecStep = 0;
	}
	if (0 != i32GetPara(PARA_AccAndDecSlowDrive))
	{
		sgUserInfo.fSlowSpdPer5msAccStep = STEP_FACTOR * i32GetPara(PARA_CurveSlowDrive) / i32GetPara(PARA_AccAndDecSlowDrive);
		sgUserInfo.fSlowSpdPer5msDecStep = STEP_FACTOR * i32GetPara(PARA_BrakeSlowDrive) / i32GetPara(PARA_AccAndDecSlowDrive);
	}
	else
	{
		//i32LogWrite(ERR, "Slow Spd Period is 0!!!\r\n");
		sgUserInfo.fSlowSpdPer5msAccStep = 0;
		sgUserInfo.fSlowSpdPer5msDecStep = 0;
	}
	if (0 != i32GetPara(PARA_AccAndDecAfterLift))
	{
		sgUserInfo.fPitSpdPer5msAccStep = STEP_FACTOR * i32GetPara(PARA_CurveDriveAfterLift) / i32GetPara(PARA_AccAndDecAfterLift);
		sgUserInfo.fPitSpdPer5msDecStep = STEP_FACTOR * i32GetPara(PARA_BrakeDriveAfterLift) / i32GetPara(PARA_AccAndDecAfterLift);
	}
	else
	{
		//i32LogWrite(ERR, "Pit Spd Period is 0!!!\r\n");
		sgUserInfo.fPitSpdPer5msAccStep = 0;
		sgUserInfo.fPitSpdPer5msDecStep = 0;
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
	if (0 != i32GetPara(PARA_AccAndDecTurn))
	{
		sgUserInfo.fTurnSpdPer5msAccStep = STEP_FACTOR * i32GetPara(PARA_CurveTurn) / i32GetPara(PARA_AccAndDecTurn);
		sgUserInfo.fTurnSpdPer5msDecStep = STEP_FACTOR * i32GetPara(PARA_BrakeTurn) / i32GetPara(PARA_AccAndDecTurn);
	}
	else
	{
		//i32LogWrite(ERR, "Turn Spd Period is 0!!!\r\n");
		sgUserInfo.fTurnSpdPer5msAccStep = 0;
		sgUserInfo.fTurnSpdPer5msDecStep = 0;
	}
	
	vSetNetTimer(TIMER_EcuPowerOn, ECU_POWERON_DELAY_TIME);	/**/
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

	if(true == u8GetNetTimerOverFlag(TIMER_EcuPowerOn))
	{
		u8EcuProcFlag = 1;
		vKillNetTimer(TIMER_EcuPowerOn);
	}
	
	if(1 == u8EcuProcFlag)
	{
		vSwiMonitor();
		vAiMonitor();
		
		sgValvesInfo.b1DisablePcuFlag = u8ErrCodeGetAbnormal(ABNORMAL_PCU);
		sgValvesInfo.b1LiftSpdFlag = u8ErrCodeGetAbnormal(ABNORMAL_LIFTSPD);
		sgValvesInfo.b1ForWardFlag = u8ErrCodeGetAbnormal(ABNORMAL_FORWARD);
		sgValvesInfo.b1BackWardFlag = u8ErrCodeGetAbnormal(ABNORMAL_BACKWARD);
		sgValvesInfo.b1LiftUpFlag = u8ErrCodeGetAbnormal(ABNORMAL_UP);
		sgValvesInfo.b1LiftDownFlag = u8ErrCodeGetAbnormal(ABNORMAL_DOWN);
		sgValvesInfo.b1TurnLeftFlag = u8ErrCodeGetAbnormal(ABNORMAL_LEFT);
		sgValvesInfo.b1TurnRightFlag = u8ErrCodeGetAbnormal(ABNORMAL_RIGHT);
		
		vEcuSetBeepPeriod();
		vCanRevPdoProc();
		
		u8ErrCode = u8ErrCodeGet();
		if (0 != u8ErrCode)
		{
			sgSpeakerFun.b160PerMin = 1;
		}
		else
		{
			sgSpeakerFun.b160PerMin = 0;
		}	
	}
	
	vWdgSetFun(WDG_USER_BIT);
}
#endif
