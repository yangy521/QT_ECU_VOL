/*******************************************************************************
*通用程序框架* 						   *
* Author: QExpand; Lilu                                                        *
* Date: 2023/05/30    														   *
* Revision:	 														 		   *
*******************************************************************************/
#include "UserCanComm.h"
#include "User_Demo.h"
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

#if (USER_ECU_OR_ET_TYPE == NULL)//修改

/*参数配置部分*/

/*报文配置*/
const static xPdoParameter  sgPdoPara = 
{		/*标准CanOpen，100，200，300，400/+nodeid*/
		/*u8Type为0xFF时周期发送*/
		/*特殊报文请用i32CanWrite周期发送*/
	.TpdoPara = {
		{.u8Flag = 1, .u8Type = 0x0, .u16Period = 20},
		{.u8Flag = 0, .u8Type = 0x0, .u16Period = 200},
		{.u8Flag = 0, .u8Type = 0x0, .u16Period = 200},
		{.u8Flag = 0, .u8Type = 0x0, .u16Period = 200},
	},
	/*Canopen接收*/
	/*设置对应id及使能*/
	.RpdoPara = {
		{.b1Flag = 1, .b11CanRevId = 0x190},
		{.b1Flag = 1, .b11CanRevId = 0x200},
		{.b1Flag = 1, .b11CanRevId = 0x360},
		{.b1Flag = 0, .b11CanRevId = 0x000},
		{.b1Flag = 0, .b11CanRevId = 0x000},
		{.b1Flag = 0, .b11CanRevId = 0x000},
	},
};

/*开关量输入*/
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

/*动作限制设置*/
typedef union
{
	uint16_t u16Data[3];	
	struct
	{
		uint16_t b4SpeedLimt1:4;
		uint16_t b4SpeedLimt2:4;
		uint16_t b4SpeedLimt3:4;
		uint16_t b4SpeedLimt4:4;
		
		uint16_t b8ErrActLimit:8;
		uint16_t b8ErrMoveLimit:8;
		
		uint16_t b8ErrPumpLimit:8;
		uint16_t b2LiftUpLimt:2;
		uint16_t b2LiftDownLimit:2;
		uint16_t b2LegActLimit:2;
		uint16_t b2OutriggerActLimit: 2;
		
	};
}xActLimit;

/*输出动作逻辑*/
typedef union
{
	uint8_t u8Data;	
	struct
	{
		uint8_t b1ForWardAct:1;
		uint8_t b1BackWardAct:1;
		
		uint8_t b1LiftUpAct:1;
		uint8_t b1LiftDownAct:1;
		
		uint8_t b1LegUpAct:1;
		uint8_t b1LegDown: 1;
		
		uint8_t	b1OutriggerUp: 1;
		uint8_t	b1OutriggerDown: 1;
		
	};
}xActLogic;

/*上位机配置参数*/
typedef struct
{
	uint8_t		u8FastDriveSpeed;
	uint8_t		u8SlowDriveSpeed;
	uint8_t		u8DriveSpeedAfterLift;
	uint8_t		u8LiftSpeed;				
	uint8_t		u8MaxTurnSpeed;			
	uint8_t		u8TurnPowerLimit;			
	uint8_t		u8DeadZoneAdjust;			

	uint8_t		u8BrakeFastDrive;			
	uint8_t		u8BrakeSlowDrive;			
	uint8_t		u8BrakeDriveAfterLift;	
	uint8_t		u8BrakeLift;				
	uint8_t		u8BrakeLower;				
	uint8_t		u8BrakeTurn;				
	uint8_t		u8BrakeAntiPinch;

	uint8_t		u8Reserve1;					/*14*/
	uint8_t		u8Reserve2;					/*15*/

	uint8_t		u8LowerSpeed;				/*16*/			
	uint8_t		u8OverLoadStabilityDelay;	
	uint8_t		u8DynamicOverLoadPercent;	
	uint8_t		u8StaticOverLoadPercent;	
	uint8_t		u8MaxDifferencePercent;	
	uint8_t		u8DriveMotorEncoder;		
	uint8_t		u8MotorHighSpeedDeceRate;	
	uint8_t		u8MotorLowSpeedDeceRate;	
	uint8_t		u8VoiceAlarmVolume;		

	uint8_t		u8CurveFastDrive;						
	uint8_t		u8CurveSlowDrive;			
	uint8_t		u8CurveDriveAfterLift;	
	uint8_t		u8CurveLift;				
	uint8_t		u8CurveLower;
	
	uint8_t		u8Reserve3;					/*30*/
	uint8_t		u8Reserve4;					/*31*/	

	uint8_t		u8CurveTurn;					/*32*/

	uint8_t		u8AccAndDecFastDrive;				
	uint8_t		u8AccAndDecSlowDrive;		
	uint8_t		u8AccAndDecAfterLift;		
	uint8_t		u8AccAndDecLift;			
	uint8_t		u8AccAndDecLower;			
	uint8_t		u8AccAndDecTurn;			
	uint8_t		u8AccAndDecAntiPinch;		

	uint8_t		u8PumpMotorEncoder;					

	uint8_t		u8VehicleType;							
	uint8_t		u8VehcileHeight;			
	uint8_t		u8PressureSensorType;		
	uint8_t		u8PitProtectFunc;			
	uint8_t		u8AntiPinchFunc;

	uint8_t		u8Reserve5;					/*46*/
	uint8_t		u8Reserve6;					/*47*/
	
	uint8_t		u8ActAlmFunc;				/*48*/				
	uint8_t		u8WeighFunc;				
	uint8_t		u8ParallelValveReverseFunc;
	uint8_t		u8LowBatAlmFunc;			
	uint8_t		u8LowVolAlmTime;			
	uint8_t		u8LowVolShutDownTime;		
	uint8_t		u8UpperCtlButSleep;		
	uint8_t		u8LanguageType;			
	uint8_t		u8BatteryType;			
	uint8_t		u8SpeakerSync;			
	uint8_t		u8LowerPumpType;			
	uint8_t		u8PressureType;			
	uint8_t		u8AngleSimulationLimit;	
	uint8_t		u8LiftReverseFunc;

	uint8_t		u8Reserve7;				/*62*/
	uint8_t		u8Reserve8;				/*63*/
	
	uint8_t		u8FourPointWeightFunc;	/*64*/	
	uint8_t		u8DriverType;			
	uint8_t		u8PasswordLock;			
	uint8_t		u8InAndOutFunc;			
	uint8_t		u8HeartBeatQueryFunc;		
	uint8_t		u8LowBatteryMode;			
	uint8_t		u8AngleSensorSetting;		
	uint8_t		u8TiltSwitchSetting;		
	uint8_t		u8AngleSensorType;		
	uint8_t		u8AnaLogLimitDetSwitch;	

	uint8_t		u8IsNoLoadCalibration;					
	uint8_t		u8IsOverLoadCalibration;	

	uint16_t		u8SetDescentHeightValue;		
	uint8_t		u8ReleaseBrake;

	uint8_t		u8Reserve9;				/*78*/
	uint8_t		u8Reserve10;				/*79*/
	
	uint16_t		u8SetOutHeight;			/*80*/
	uint16_t		u8AngleSimulationUpLimit;	
	uint16_t		u8AngleSimulationDownLimit;
	
	uint16_t		u8PumpDriveCurrentLimitRatio0;			
	uint16_t		u8PumpSpdAccRatio0;
	uint16_t		u8PropDKp0;						
	uint16_t		u8PropDKi0;						
	uint16_t		u8PropDMaxCurrent0;				
	uint16_t		u8PropDMinCurrent0;
	uint16_t		u8PropDAccPeriod0;
	uint16_t		u8PropDDitherPeriod0;	
	uint16_t		u8PropDDitherRatio0;			
	uint16_t		u8PropValveResistance0;
	
	uint8_t		u8Reserve11;					/*93*/
	uint8_t		u8Reserve12;					/*94*/
	uint8_t		u8Reserve13;					/*95*/
	
	
	uint16_t		u8PumpDriveCurrentLimitRatio1;		/*96*/	
	uint16_t		u8PumpSpdAccRatio1;
	uint16_t		u8PropDKp1;						
	uint16_t		u8PropDKi1;						
	uint16_t		u8PropDMaxCurrent1;				
	uint16_t		u8PropDMinCurrent1;
	uint16_t		u8PropDAccPeriod1;
	uint16_t		u8PropDDitherPeriod1;	
	uint16_t		u8PropDDitherRatio1;			
	uint16_t		u8PropValveResistance1;
	

	uint16_t		u8EmptyPressure;
	uint16_t		u8FullPressure;
	uint16_t		u8DriverFlag;
	
	uint8_t		u8Reserve14;					/*110*/
	uint8_t		u8Reserve15;					/*111*/
	
	uint16_t		u8MinAngle;					/*112*/
	uint16_t		u8MaxAngle;
	uint16_t		u8BatSocPalyBack;
	uint16_t		u8ValveType;
	uint16_t		u8AnticollisionFunc;
	uint16_t		u8ValueOpenLoopCurrent;
	uint16_t		u8ValueOpenPercentage;
	

	
	uint16_t		u8AngleValue0;
	uint16_t		u8AngleValue1;
	uint16_t		u8AngleValue2;
	uint16_t		u8AngleValue3;
	uint16_t		u8AngleValue4;
	uint16_t		u8AngleValue5;
	uint16_t		u8AngleValue6;
	uint16_t		u8AngleValue7;
	

	uint8_t		u8LogLevel;					/*138 log level*/
	uint8_t		u8LogModel;					/*139*/
	uint8_t		u8MotorMaxSpd;				/*140*/
	
	uint8_t		u8Analog3DeadZoneMinVal;		/*141*/
	uint8_t		u8Analog3DeadZoneMaxVal;		/*142*/
	uint8_t		u8Analog3MidVal;				/*143*/
	
	uint8_t		u8ThrottleType;					/*144 加速器类型*/
	uint8_t		u8ThrottleFDeadZoneMinVal;		/*踏板正向死区最小值（0.1V）*/
	uint8_t		u8ThrottleFDeadZoneMaxVal;		/*踏板正向死区最大值（0.1V）*/
	uint8_t		u8ThrottleFMidVal;				/*踏板正向中值对应输出百分比（1%）*/
	uint8_t		u8ThrottleBDeadZoneMinVal;		/*踏板反向死区最小值（0.1V）*/
	uint8_t		u8ThrottleBDeadZoneMaxVal;		/*踏板反向死区最大值（0.1V）*/
	uint8_t		u8ThrottleBMidVal;				/*踏板反向中值对应输出百分比（1%）*/
	
	uint8_t		u8BrakeType;						/*制动踏板输入类型*/
	uint8_t		u8BrakeFDeadZoneMinVal;			/*制动踏板正向死区最小值（0.1V）*/
	uint8_t		u8BrakeFDeadZoneMaxVal;			/*制动踏板正向死区最大值（0.1V）*/
	uint8_t		u8BrakeFMidVal;					/*制动踏板正向中值对应输出百分比（1%）*/
	uint8_t		u8BrakeBDeadZoneMinVal;			/*制动踏板反向死区最小值（0.1V）*/
	uint8_t		u8BrakeBDeadZoneMaxVal;			/*制动踏板反向死区最大值（0.1V）*/
	uint8_t		u8BrakeBMidVal;					/*制动踏板反向中值对应输出百分比（1%）*/
	
	uint8_t		u8Reserve16;						/*158*/
	uint8_t		u8Reserve17;						/*159*/
	
	uint8_t		u8Gear1Spd;						/*160 1档速度(%)*/
	uint8_t		u8Gear2Spd;						/*2档速度(%)*/
	uint8_t		u8Gear3Spd;						/*3档速度(%)*/
	uint8_t		u8Gear4Spd;						/*4档速度(%)*/
	
	uint16_t		u8RatioOfTransmission;			/*转速公里比*/
	uint16_t		u8MaintenancePeriod;				/*维护周期(h)*/
	uint8_t		u8RemotePara;					/*远程终端参数*/
	
	uint8_t		u8PumpMotorGear1;				/*起升电机中间挡位1(%)*/
	uint8_t		u8PumpMotorGear2;				/*起升电机中间挡位2(%)*/
	
	uint8_t		u8TurnWithDecStartAngle;			/*转弯降速起始角度*/
	uint8_t		u8TurnWithDecEndAngle;			/*转弯降速终止角度*/
	
	uint8_t		u8AngleWithStartSpdPer;			/*起始角度速度百分比*/
	uint8_t		u8AngleWithEndSpdPer;			/*终止角度速度百分比*/
	
	uint8_t		u8HourCountPowerOn;

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
	
	uint8_t		u8OutRiggerRate;
	uint8_t		u8LegUpRate;
	

	uint8_t		b1LiftMode: 1;
	uint8_t		b1RentalStop: 1;
	uint8_t		b1RentalStart: 1;
	uint8_t		b1RentalMode: 1;
	uint8_t		b1PasswordFunc: 1;
	uint8_t		b3Reserve: 3;	
	
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

/*发送给MCU指令*/
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

/*存储数据*/
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

/*ECU故障与显示故障转换表*/
const static uint8_t u8ErrSwitchArray[ErrCodeMax] = 
{	
/*  0    1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18   19*/
    0,   1,   2,   3,   4,   5,   6,   7,   8,   9,   10,  11,  12,  13,  14,  15,  16,  17,  18,  19,
    20,  21,  22,  23,  24,  25,  26,  27,  28,  29,  30,  31,  32,  33,  34,  35,  36,  37,  38,  39,
    40,  41,  42,  43,  44,  45,  46,  47,  48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,
		60,  61,  62,  63,  64,  65,  66,  67,  68,  69,  70,  71,  72,  73,  74,  75,  76,  77,  78,  79, 
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

/*故障与动作限制*/


/*变量*/


/**************************
初始化与逻辑限制
*************************/

/*******************************************************************************
* Name: uint8_t u8SwiInitChcek(void)
* Descriptio: 开关量初始化检查
* Input: NULL
* Output: NULL  
*******************************************************************************/
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


/*******************************************************************************
* Name: void vDoPwmErrCallBack(eDoPwmNo DoPwmNo)
* Descriptio: DO故障检测
* Input: NULL
* Output: NULL  
*******************************************************************************/
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
			i32ErrCodeSet(LEAN_BACKWARD_VALVE_ERR);
			/*add ErrCode*/
			break;
		default:
			break;
	}
}
/*******************************************************************************
* Name: void vPropErrCallBack(uint8_t u8Channel)
* Descriptio: 比例阀故障检测
* Input: NULL
* Output: NULL  
*******************************************************************************/

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

/*******************************************************************************
* Name: void vAiErrCallBack(eAiErrNo AiErrNo)
* Descriptio: 模拟量故障检测
* Input: NULL
* Output: NULL  
*******************************************************************************/
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

/****************************************
输入部分
***************************************/
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
			if ((0 == sgSwiInput.b1UpLimit) && (1 == SwiInput.b1UpLimit) && (1 == sgValvesInfo.b1LiftUpStat))
			{
//				sgValvesInfo.b4Gear2Spd = 1 << Gear2_Spd_Con1;
				sgSwiInput.b1HeightSpdLimit = 1;
				sgSaveState.b1HeightSpdLimit = 1;
				u16EepromWrite(PARA_SaveState, sgSaveState.u16Data, 1);	
			}
			else if ((0 == sgSwiInput.b1UpLimit) && (1 == SwiInput.b1UpLimit) && (1 == sgValvesInfo.b1LiftDownStat))
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
			

			if (1 == sgSwiInput.b1HeightSpdLimit)
			{
				sgValvesInfo.b4Gear2Spd = 1 << Gear2_Spd_Con1;
			}
			else
			{
				sgValvesInfo.b4Gear2Spd &=  ~(1 << Gear2_Spd_Con1);
			}
			/*Limit Up*/
			if (1 == sgSwiInput.b1Above1M8) 
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
			
			/*lilu 20230823 起升时踏板必须收起或者放下站人，负责所有油泵电机关闭操作*/
			{
				if ((1 == sgUserInfo.b1LiftPedal) && (0 == sgSwiInput.b1PedalClose) && (0 == sgSwiInput.b1PedalOpen))
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


/*接收PDO处理函数*/
static void vCanRevPdoProc(void)
{
	static xCanRev1ACInfo CanRev1ACInfoLast;
	static uint16_t u16CanRev1ACCnt;
	
	if (CanRev1ACInfoLast.b1ToggleBit == gCanRevPdoInfo.CanRevInfo1.b1ToggleBit)
	{
		u16CanRev1ACCnt++;
		if (u16CanRev1ACCnt >= CAN_1AC_LOST_NO )
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
			i32ErrCodeClr(BAT_LOW_1_ERR);
		}
		
		if (1 == CanRev1ACInfoLast.b1DisableLiftUp)
		{
			sgValvesInfo.b4NoLiftUp |= 1 << NoLiftUp_Bms;
			i32ErrCodeSet(BMS_LIFTUP_ERR);
		}
		else
		{
			sgValvesInfo.b4NoLiftUp &= ~(1 << NoLiftUp_Bms);
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


/*********************************************************
输出部分
***********************************************************/
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
//		u8Flag = 1<< 1;
	}
//	if (0 != u8Flag)
//	{
//		SendData->b1ServoOn = 1;
//	}
	/*租赁设备要求*/
	if ((1 == sgUserInfo.b1RentalStop) ||
		((1 == sgUserInfo.b1RentalStart) && (1 == sgUserInfo.b1RentalMode) && (0 == sgUserInfo.u16RentalTime)))
	{
		SendData->u8PumpTarget = 0;			/*禁止起升*/
		SendData->b1LiftReq = 0;
		
		SendData->u8TargetHigh = 0;
		SendData->u8TargetLow = 0;
		SendData->b1ForwardReq = 0;
		SendData->b1BackwardReq = 0;
		
		i32DoPwmSet(LIFTUP_VALVE, DRIVER_CLOSE);
		i32DoPwmSet(LEAN_FORWARD_VALVE, DRIVER_CLOSE);
		i32DoPwmSet(LEAN_BACKWARD_VALVE, DRIVER_CLOSE);
		
		i32ErrCodeSet(RENTAL_TIMEOUT_ERR);
		
//		if (i16Spd >= 164)					/*速度大于4%， 按照4%处理*/
//		{
//			SendData->u8TargetHigh = 0;
//			SendData->u8TargetLow = 164;
//		}
	}
	else
	{
		i32ErrCodeClr(RENTAL_TIMEOUT_ERR);
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

/*******************************************************************************
* Name: void vUserEcuInit(void)
* Descriptio: UserEcu初始化函数
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
	
	sgUserInfo.fPropMinCurrent = i32GetPara(PROP_MIN_CURRENT) / 1000.0;		/*比例阀最小电流*/
	sgUserInfo.fPropMaxCurrent = i32GetPara(PROP_MAX_CURRENT) / 1000.0;		/*比例阀最大电流*/
	
	sgUserInfo.u8LeanForWardValue = i32GetPara(LEAN_FORWARD_PARA) * PUMP_RANGE / 100;		/*前倾参数*/
	sgUserInfo.u8LeanBackWardValue = i32GetPara(LEAN_BACKWARD_PARA) * PUMP_RANGE / 100;		/*后倾参数*/
	
	sgUserInfo.b1LiftMode = i32GetPara(LIFT_MODE_PARA);						/*起升模式-制动踏板类型*/
	
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
	gCanSendPdoInfo.CanSend260Info.u8HourCountL = u32HourCount & 0xFF;
	gCanSendPdoInfo.CanSend260Info.u8HourCountH = (u32HourCount >> 8) & 0xFF;

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
		
		if (1 == sgUserInfo.b1HourConutMode)
		{
			if (1 == sgSwiInput.b1Lock)
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
