/*******************************************************************************
*通用程序框架* 						   *
* Author: QExpand; Lilu                                                        *
* Date: 2023/05/30    														   *
* Revision:	 														 		   *
*******************************************************************************/
#include "UserCanComm.h"
#include "USER_RUYI_CQDH15A.h"
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

#if (USER_TYPE == USER_STACKERTRUNK_RUYI_CQDH15A)//修改


/*参数配置部分*/

/*报文配置*/
const static xPdoParameter  sgPdoPara = 
{		/*标准CanOpen，100，200，300，400/+nodeid*/
		/*u8Type为0xFF时周期发送*/
		/*特殊报文请用i32CanWrite周期发送*/
	.TpdoPara = {
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0xFF, .u16Period = 20, .u16CanId = 0x260 },
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50, .u16CanId = 0x000 },
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50, .u16CanId = 0x000 },
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50, .u16CanId = 0x000 },
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50, .u16CanId = 0x000 },
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50, .u16CanId = 0x000 },
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
		uint16_t b1HeightSpdlimit: 1;
		uint16_t b1Ems: 1;
		uint16_t b1SafeLock: 1;
		uint16_t b1Forward: 1;
		uint16_t b1Backward: 1;
		uint16_t b1SlowRequest: 1;
		uint16_t b1LiftUp: 1;
		uint16_t b1OrbitMove:1;
		uint16_t b1Pedal: 1;
		uint16_t b1Tilt: 1;
		uint16_t b1Flip: 1;
		uint16_t b1LeftFence: 1;
		uint16_t b1RightFence: 1;
		uint16_t b1SteerLock:1;
		uint16_t b1EmergencyStop:1;
		uint16_t b1HeightLimitState: 1;
	};
}xSwiInput;

/*动作限制设置*/
typedef union
{
	uint8_t u8Data;
	struct
	{
		uint8_t b1Init:1;
		uint8_t b1BMS_Erro:1;
		uint8_t b1SteerErro:1;
		uint8_t b1SafeLimit:1;
		uint8_t b1LogicSequnce:1;
		uint8_t b1CanIDlost1:1;
		uint8_t b1CanIDlost2:1;
		uint8_t b1FirmwareFailure:1;
	};
}xAct;

typedef union
{
	uint8_t u8Data;
	struct
	{
		uint8_t b1Init:1;
		uint8_t b1Act:1;
		uint8_t b1EMS:1;
		uint8_t b5Rserve1:5;
	};
}xMove;

typedef union
{
	uint8_t u8Data;
	struct
	{
		uint8_t b1Init:1;
		uint8_t b1Act:1;
		uint8_t b1BMSWarning:1;
		uint8_t	b1HeightLimit:1;
		uint8_t b4Rserve1:4;
	
	};
}xLift;

typedef union
{
	uint8_t u8Data[4];
	struct
	{
		uint8_t b1Spd1Lift:1;
		uint8_t b1Spd1Slow:1;
		uint8_t b1Rserve1:6;
		
		uint8_t b1Spd2BMS:1;
		uint8_t b7Rserve2:7;
		
		uint8_t b1Spd3Walk:1;
		uint8_t b7Rserve3:7;
		
		uint8_t b1Spd4Zhili:1;
		uint8_t b7Rserve4:7;
	};
}xSpd;

typedef union
{
	uint8_t u8Data;
	struct
	{
		uint8_t b1OrbitRate:1;
		
		uint8_t b1Tilt:1;
		
		uint8_t b6Rserve:6;
	};
}xPumpSpd;

typedef struct
{
	xAct Action;
	xMove Movement;
	xLift Lift;
	xSpd SpeedRate; 
	xPumpSpd PumpRate;
}xActLimit;

/*输出动作逻辑*/
typedef union
{
	uint8_t u8Data[3];	
	struct
	{
		/*行走动作逻辑*/
		uint8_t b1ForwardAct:1;
		uint8_t b1BackwardAct:1;
		uint8_t	b6Reserve1:6;
		/*起升动作逻辑*/
		uint8_t b1LiftUpAct:1;
		uint8_t b1LiftDownAct:1;
		uint8_t b1OrbitMove:1;
		uint8_t b1Tilt: 1;
		uint8_t b4Reserve2:4;
		/*附加状态逻辑*/
		uint8_t	b1ZhiLiState:1;
		uint8_t b1HeightLimitState:1;
		uint8_t b6Reserve3:6;
		
	};
}xActLogic;


/*转弯降速*/
typedef struct
{
	uint16_t	u16StartAngle;
	uint16_t	u16EndAngle;
	uint16_t	u16StartAngleSpd;
	uint16_t	u16EndAngleSpd;
	float		fAgnleDecSpdFactor;
}xSteerAngleDecSpd;

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
	uint8_t 	b1HourCntClr:1;
	uint8_t		b2Reserve: 2;	
	
	uint8_t		b1HourConutMode: 1;		/*0: 上电计时， 1：互锁计时*/
	uint8_t		b1StartUpLock: 1;		/*0： 检测， 1：不检测*/
	uint8_t		b1LiftLock: 1;			/*0： 需要互锁， 1：不需要*/
	uint8_t		b1LiftPedal: 1;			/*0： 无要求， 1：踏板open =1， close =0 / open =0， close =1 才可以动作*/
	uint8_t		b1MoveLiftMode: 1;		/*0： 同时动作， 1：互斥*/
	uint8_t		b1SteerCheck:1;
	uint8_t		b1CarType:1;   
	uint8_t		b1SpeedLimitAfterLift: 1;
	
	uint16_t	u16RatioOfTransmission;
	uint16_t	u16MotorMaxSpd;
	
	uint16_t	u16RentalTime;
	xSteerAngleDecSpd SteerAngleDecSpd;
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
//const static uint8_t u8ErrSwitchArray[ErrCodeMax] = 
//{	
///*    0    1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18   19*/
//		  0,   1,   2,   3,   4,   5,   6,   7,   8,   9,   10,  11,  12,  13,  14,  15,  16,  17,  18,  19,
//     20,  21,  22,  23,  24,  25,  26,  27,  28,  29,   30,  31,  32,  33,  34,  35,  36,  37,  38,  39,
//     40,  41,  42,  43,  44,  45,  46,  47,  48,  49,   50,  51,  52,  53,  54,  55,  56,  57,  58,  59,
//	   60,  61,  62,  63,  64,  65,  66,  67,  68,  69,   70,  71,  72,  73,  74,  75,  76,  77,  78,  79, 
//		 80,  81,  82,  83,  84,  85,  86,  87,  88,  89,   90,  91,  92,  93,  94,  95,  96,  97,  98,  99,
//		100, 101, 102, 103, 104, 105, 106, 107, 108, 109,  110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 
//		120, 121, 122, 123, 124, 125, 126, 127, 128, 129,  130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 
//		140, 141, 142, 143, 144, 145, 146, 147, 148, 149,  150, 151, 152, 153, 154, 155, 156, 157, 158, 159,
//		160, 161, 162, 163, 164, 165, 166, 167, 168, 169,  170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 
//		180, 181, 182, 183, 184, 185, 186, 187, 188, 189,  190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 
//		200, 201, 202, 203, 204, 205, 206, 207, 208, 209,  210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 
//		220, 221, 222, 223, 224, 225, 226, 227, 228, 229,  230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 
//		240, 241, 242, 243, 244, 245, 246, 247, 248, 249,  250, 251, 252, 253, 254,
//};
const static uint8_t u8ErrSwitchArray[ErrCodeMax] = 
{	
/*  0    1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18   19*/
    0,   1,   2,   3,   4,   5,   6,   7,   8,   9,   10,  11,  12,  13,  14,  15,  16,  17,  18,  19,
    20,  21,  22,  23,  24,  25,  26,  27,  28,  29,  30,  31,  32,  33,  34,  35,  36,  37,  38,  39,
    40,  41,  42,  43,  44,  45,  46,  47,  48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,
		60,  61,  62,  63,  64,  65,  66,  67,  68,  69,  70,  71,  72,  73,  74,  75,  76,  77,  78,  79, 
		80,  81,  82,  83,  84,  85,  86,  87,  88,  89,  90,  91,  92,  93,  94,  95,  96,  97,  98,  99,
		100, 40, 	42,  41, 	90,	 91, 92, 	40, 40, 40, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 
		120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 
		140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159,
		160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 
		180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 
		200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 
		220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 
		240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254,
};

/*故障与动作限制*/

/*PDO发送待完善*/
xCanSendPdoInfo gCanSendPdoInfo ;
xCanRevPdoInfo gCanRevPdoInfo;	/*PDO接收待完善*/
/*变量*/
static xActLimit sgLock;
static xActLogic sgActLogic;
static xSwiInput sgSwiInput;
static xSaveStateInfo sgSaveState;

static int16_t  i16MotorSpd = 0;
static uint16_t u16MotorVal = 0;
static uint8_t	u8PumpVal = 0;
static xUserInfo sgUserInfo;
static uint32_t u32HourCount = 0;

static uint16_t	u16ID190Cnt = 0;
static uint16_t u16ID200Cnt = 0;
static uint16_t u16ID360Cnt = 0;
static int16_t i16SteerAngle = 0;
static uint8_t u8BMS_SOC = 0;
static uint8_t u8BMS_ERRCODE = 0;
static uint8_t u8BMS_FLAG = 0;

static	uint8_t u8PropValue = 0;
static  int32_t i32PropValue = 0;

/*************************
通用逻辑
********************/
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
/**************************初始化与逻辑限制*************************/

/*******************************************************************************
* Name: uint8_t u8SwiInitChcek(void)
* Descriptio: 开关量初始化检查
* Input: NULL
* Output: NULL  
*******************************************************************************/
static void vSwiInitChcek(void)
{
	uint8_t u8Res = 0;
	if ( (1 == i32LocalDiGet(EMERGENCY_REVERSE_SWI)) 
		|| (1 == i32LocalDiGet(SAFELOCK_SWI)) 
		|| (1 == i32LocalDiGet(FORWARD_SWI))
		|| (1 == i32LocalDiGet(BACKWARD_SWI))
		|| (1 == i32LocalDiGet(SLOW_REQUEST_SWI)) 
		|| (1 == i32LocalDiGet(LIFT_UP_SWI)) 
		|| (1 == i32LocalDiGet(ORBITAL_MOVE_SWI))
		|| (1 == i32LocalDiGet(TILT_SWI))
		|| (1 == i32LocalDiGet(STEER_LOCK_SWI))) 		
	{
		i32ErrCodeSet(ACT_INIT_ERR);
		sgLock.Action.b1Init = 1 ;  
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
			sgLock.Action.b1FirmwareFailure = 1;
			break;
		case AI_B_AI2_R_ERR:
			sgLock.Action.b1FirmwareFailure = 1;
			break;
		case AI_B_AI3_R_ERR:
			sgLock.Action.b1FirmwareFailure = 1;
			break;
		case AI_5V_12V_OUT1_I_ERR:
			sgLock.Action.b1FirmwareFailure = 1;
			break;
		case AI_5V_12V_OUT2_I_ERR:
			sgLock.Action.b1FirmwareFailure = 1;
			break;
		default:
			break;
	}
}
/*******************************************************************************
* Name: void vDoPwmErrCallBack(eDoPwmNo DoPwmNo)
* Descriptio: 输出故障检测
* Input: NULL
* Output: NULL  
*******************************************************************************/
static void vDoPwmErrCallBack(eDoPwmNo DoPwmNo)
{
	switch((uint8_t)DoPwmNo)
	{
		case DRIVER1:
			sgLock.Action.b1FirmwareFailure = 1;
			break;
		case DRIVER2:
			sgLock.Action.b1FirmwareFailure = 1;
			break;
		case DRIVER3:
			sgLock.Action.b1FirmwareFailure = 1;
			break;
		case DRIVER4:
			sgLock.Action.b1FirmwareFailure = 1;
			break;
		case DRIVER5:
			sgLock.Action.b1FirmwareFailure = 1;
			break;
		case DRIVER6:
			sgLock.Action.b1FirmwareFailure = 1;
			break;
		case DRIVER7:
			sgLock.Action.b1FirmwareFailure = 1;
			break;
		case DRIVER8:
			sgLock.Action.b1FirmwareFailure = 1;
			break;
		case DRIVER9:
			sgLock.Action.b1FirmwareFailure = 1;
			break;
		case DRIVER10:
			sgLock.Action.b1FirmwareFailure = 1;
			break;
		default:
			break;
	}
}
/*******************************************************************************
* Name: void vPropErrCallBack(uint8_t u8Channel)
* Descriptio: 比例阀故障
* Input: NULL
* Output: NULL  
*******************************************************************************/
static void vPropErrCallBack(uint8_t u8Channel)
{
	switch(u8Channel)
	{
		case PropDriverCh0:
			sgLock.Action.b1FirmwareFailure = 1;
			break;
		case PropDriverCh1:
			sgLock.Action.b1FirmwareFailure = 1;
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
	uint8_t u8SpeedRate = 100;
	
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
	/*add Turn Dec Spd*/
	u16MotorVal = u16SteerAngleDecSpd(u16MotorVal, abs(i16SteerAngle), &sgUserInfo.SteerAngleDecSpd);

	
	/*速度挡位限制*/
	if (0 != sgLock.SpeedRate.u8Data[0])
	{
		if (u8SpeedRate >= sgUserInfo.u16Gear1Spd)
		{
			u8SpeedRate = sgUserInfo.u16Gear1Spd;
		}
	}
	
	if (0 != sgLock.SpeedRate.u8Data[1])
	{
		if (u8SpeedRate >= sgUserInfo.u16Gear2Spd)
		{
			u8SpeedRate = sgUserInfo.u16Gear2Spd;
		}
	}
	
	if (0 != sgLock.SpeedRate.u8Data[2])
	{
		if (u8SpeedRate >= sgUserInfo.u16Gear3Spd)
		{
			u8SpeedRate =  sgUserInfo.u16Gear3Spd;
		}
	}
	
	if (0 != sgLock.SpeedRate.u8Data[3])
	{
		if (u8SpeedRate >= sgUserInfo.u16Gear4Spd)
		{
			u8SpeedRate = sgUserInfo.u16Gear4Spd;
		}
	}
	u16MotorVal = u16MotorVal * u8SpeedRate / 100 ;
	
	/*油泵速度设置*/
	
	i32AdcValue = i32LocalAiGetValue(LIFT_THROTTLE);
	
	if (LIFT_MODE_THROTTLE == sgUserInfo.b1LiftMode)
	{
#ifdef KUNLI_IOTYPE
		vPropSetTarget(LIFTDOWN_VALVE, 0);
		if (i32AdcValue > sgUserInfo.u16LiftDownMax)	/*max*/
		{
			u8PumpVal = PUMP_MAX_VALUE;
		}
		else if (i32AdcValue >= (sgUserInfo.u16LiftDownMidValue+200))	/*between mid & max*/
		{
			u8PumpVal = (255*(i32AdcValue-sgUserInfo.u16LiftDownMidValue-200))/(sgUserInfo.u16LiftDownMax - sgUserInfo.u16LiftDownMidValue-200);
		}
		else if ((i32AdcValue <= (sgUserInfo.u16LiftDownMidValue-200))&&(i32AdcValue >= (1000)))	/*between min & max*/
		{
			u8PropValue = (255*(sgUserInfo.u16LiftDownMidValue-200-i32AdcValue))/(sgUserInfo.u16LiftDownMidValue-200-1000);
			i32PropValue = _IQ((sgUserInfo.fPropMinCurrent + (sgUserInfo.fPropMaxCurrent - sgUserInfo.fPropMinCurrent) * u8PropValue / PUMP_RANGE) / PROPD_STD_CURRENT);
			vPropSetTarget(LIFTDOWN_VALVE, i32PropValue);
		}
		else			/*min*/
		{
			i32PropValue = 0;
			u8PropValue = 0;
			u8PumpVal = 0;
		}	
#else
		if (i32AdcValue > sgUserInfo.u16LiftDownMax)	/*max*/
		{
			u8PumpVal = PUMP_MAX_VALUE;
		}
		else if (i32AdcValue >= sgUserInfo.u16LiftDownMidValue)	/*between mid & max*/
		{
			u8PumpVal = sgUserInfo.u16LiftDownMid + ( i32AdcValue-sgUserInfo.u16LiftDownMidValue) * (PUMP_RANGE - sgUserInfo.u16LiftDownMid) / sgUserInfo.u16LiftDownRange;
		}
		else if (i32AdcValue > sgUserInfo.u16LiftDownMin)	/*between min & max*/
		{
			u8PumpVal = (i32AdcValue-sgUserInfo.u16LiftDownMin ) * sgUserInfo.u16LiftDownMid / sgUserInfo.u16LiftDownRange;
		}
		else			/*min*/
		{
			u8PumpVal = 0;
		}
#endif
	}
	else
	{
		u8PumpVal = PUMP_MAX_VALUE;
	}
	u8PumpVal = (uint8_t)((uint16_t)u8PumpVal * sgUserInfo.u8LiftSpeed /100);
	
	/*起升挡位限制*/
	
	if (0 != sgLock.PumpRate.b1OrbitRate)
	{
		if (u8PumpVal >= sgUserInfo.u8PumpMotorGear1)
		{
			u8PumpVal = sgUserInfo.u8PumpMotorGear1;
		}
	}
	
	if (0 != sgLock.PumpRate.b1Tilt)
	{
		if (u8PumpVal >= sgUserInfo.u8PumpMotorGear2)
		{
			u8PumpVal = sgUserInfo.u8PumpMotorGear2;
		}
	}
}

/*开关量监控*/
static void vSwiMonitor(void)
{
	xSwiInput SwiInput = {0};
	
	if(1 == i32LocalDiGet(HEIGHT_SPEEDLIMIT_SWI))
	{
		SwiInput.b1HeightSpdlimit = 1;
	}

	if(1 == i32LocalDiGet(EMERGENCY_REVERSE_SWI))//ECU不处理，MCU勾选4号参数
	{
		SwiInput.b1Ems = 1;
	}
	
	if(1 == i32LocalDiGet(SAFELOCK_SWI))
	{
		SwiInput.b1SafeLock = 1;
		gCanSendPdoInfo.CanSend260Info.b1Maint = 1;
	}
	else
	{
		gCanSendPdoInfo.CanSend260Info.b1Maint = 0;
	}
	
	if(1 == i32LocalDiGet(FORWARD_SWI))
	{
		SwiInput.b1Forward = 1;
	}
	
	if(1 == i32LocalDiGet(BACKWARD_SWI))
	{
		SwiInput.b1Backward = 1;
	}
	
	if(1 == i32LocalDiGet(SLOW_REQUEST_SWI))
	{
		SwiInput.b1SlowRequest = 1;
	}
	
	if(1 == i32LocalDiGet(LIFT_UP_SWI))
	{
		SwiInput.b1LiftUp = 1;
	}
	
	if(1 == i32LocalDiGet(ORBITAL_MOVE_SWI))
	{
		SwiInput.b1OrbitMove = 1;
	}
	
	if(1 == i32LocalDiGet(PEDAL_SWI))
	{
		SwiInput.b1Pedal = 1;
	}
	
	if(1 == i32LocalDiGet(TILT_SWI))
	{
		SwiInput.b1Tilt = 1;
	}
	
	if(1 == i32LocalDiGet(FLIP_SWI))
	{
		SwiInput.b1Flip = 1;
	}
	
	if(1 == i32LocalDiGet(LEFT_FENCE_SWI))
	{
		SwiInput.b1LeftFence = 1;
	}
	
	if(1 == i32LocalDiGet(RIGHT_FENCE_SWI))
	{
		SwiInput.b1RightFence = 1;
	}
	
	if(1 == i32LocalDiGet(STEER_LOCK_SWI))
	{
		SwiInput.b1SteerLock = 1;
	}
	

	/*高度限速开关设置*/
	if ((1 == SwiInput.b1HeightSpdlimit) && (1 == sgActLogic.b1LiftUpAct))
	{
		sgActLogic.b1HeightLimitState = 1;
		sgSwiInput.b1HeightLimitState = 1;
		sgSaveState.b1HeightSpdLimit = 1;
		u16EepromWrite(PARA_SaveState, sgSaveState.u16Data, 1);	
	}
	else if ((1 == SwiInput.b1HeightSpdlimit))
	{
		sgActLogic.b1HeightLimitState = 0;
		sgSwiInput.b1HeightLimitState = 0;
		sgSaveState.b1HeightSpdLimit = 0;
		u16EepromWrite(PARA_SaveState, sgSaveState.u16Data, 1);	
	}
	SwiInput.b1HeightLimitState = sgSwiInput.b1HeightLimitState;//保留的高度限制状态
	
	
	/*add lock */
	/*清空开关相关错误*/
	if((0 == SwiInput.b1Backward)
		&&(0 == SwiInput.b1Forward)
		&&(0 == SwiInput.b1LiftUp)
		&&(0 == SwiInput.b1OrbitMove)
		&&(0 == SwiInput.b1Tilt))
	{
		if(0 ==SwiInput.b1SafeLock)
		{
			i32ErrCodeClr(ACT_INIT_ERR);
			sgLock.Action.b1Init = 0;		
		}
		i32ErrCodeClr(EMS_LOCK_ERR);
		i32ErrCodeClr(SWI_SEQUENCE_ERR);
		sgLock.Movement.b1EMS = 0;
		sgLock.Action.b1LogicSequnce = 0;
	}
	
	/*操作顺序错误*/
	if((0 ==sgSwiInput.b1SafeLock)
		&&(0 == sgActLogic.b1ZhiLiState))//非直立行走状态，历史使能按键为0
	{
		if((1 == SwiInput.b1SafeLock)
			&&((1 == SwiInput.b1Forward)
			||(1 == SwiInput.b1Backward)))//当前按下前进后退后再按使能
		{
			sgLock.Action.b1LogicSequnce = 1;
			i32ErrCodeSet(SWI_SEQUENCE_ERR);
		}	
	}
	if((1 == SwiInput.b1SafeLock)&&(1 == sgActLogic.b1ZhiLiState))
	{
		sgLock.Action.b1LogicSequnce = 1;
		i32ErrCodeSet(SWI_SEQUENCE_ERR);
	}
	if((1 == sgSwiInput.b1SafeLock)
		&&((1 == sgSwiInput.b1Forward)||(1 == sgSwiInput.b1Backward)))
	{
		if((0 == SwiInput.b1SafeLock)
		&&((1 == SwiInput.b1Forward)||(1 == SwiInput.b1Backward)))
		{
			sgLock.Action.b1LogicSequnce = 1;
			i32ErrCodeSet(SWI_SEQUENCE_ERR);
		}
	}
	if(1 == sgUserInfo.b1LiftLock)
	{
		if((0 == sgSwiInput.b1SafeLock)
			&&(1 ==SwiInput.b1SafeLock)
			&&((1 == SwiInput.b1LiftUp)
			||(1 == SwiInput.b1OrbitMove)
			||(1 == SwiInput.b1Tilt)))
		{
			sgLock.Action.b1LogicSequnce = 1;
			i32ErrCodeSet(SWI_SEQUENCE_ERR);			
		}
	}

	if(0 == sgUserInfo.b1CarType)
	{
		if(0 == (SwiInput.b1Flip + SwiInput.b1Pedal))//人站踏板
		{
			if(0 == (SwiInput.b1LeftFence + SwiInput.b1RightFence))
			{
				sgLock.SpeedRate.b1Spd3Walk = 0;
				
				sgLock.Action.b1SafeLimit = 0;
				
				sgLock.Lift.b1HeightLimit = SwiInput.b1HeightLimitState;
			}
			else if(1 == (SwiInput.b1LeftFence + SwiInput.b1RightFence))
			{
				sgLock.SpeedRate.b1Spd3Walk = 1;
				sgLock.Action.b1SafeLimit = 0;
				sgLock.Lift.b1HeightLimit = SwiInput.b1HeightLimitState;
			}
			else if(2 == (SwiInput.b1LeftFence + SwiInput.b1RightFence))
			{
				sgLock.SpeedRate.b1Spd3Walk = 1;
				sgLock.Action.b1SafeLimit = 0;
				sgLock.Lift.b1HeightLimit = 0;
			}
		}
		else if(4 == (SwiInput.b1Flip + SwiInput.b1Pedal 
			+ SwiInput.b1LeftFence + SwiInput.b1RightFence))
		{
			sgLock.SpeedRate.b1Spd3Walk = 1;
				sgLock.Action.b1SafeLimit = 0;
				sgLock.Lift.b1HeightLimit = 0;
		}
		else
		{
			sgLock.Action.b1SafeLimit = 1;
		}
	}
	else
	{
		if(0 != (SwiInput.b1Flip + SwiInput.b1Pedal + SwiInput.b1LeftFence + SwiInput.b1RightFence))
		{
			sgLock.SpeedRate.b1Spd3Walk = 1;
			sgLock.Action.b1SafeLimit = 0;
			sgLock.Lift.b1HeightLimit = 0;
		}
		else
		{
			sgLock.SpeedRate.b1Spd3Walk = 0;
			sgLock.Action.b1SafeLimit = 0;
			sgLock.Lift.b1HeightLimit = 0;
		}
	}
	
	if(1 == sgUserInfo.b1SpeedLimitAfterLift)//起升限速bit7
	{
		sgLock.SpeedRate.b1Spd1Lift = SwiInput.b1HeightLimitState;
	}
	
	
	if(0 != sgLock.Action.u8Data )//有禁止命令时，禁止行走和起升
	{
		sgLock.Lift.b1Act = 1;
		sgLock.Movement.b1Act = 1;
	}
	else
	{
		sgLock.Lift.b1Act = 0;
		sgLock.Movement.b1Act = 0;
	}
	

	/*动作逻辑*/
	memset(&sgActLogic, 0, sizeof(xActLogic));//动作判断初始化
	
	if(0 == sgLock.Movement.u8Data)//没有禁止指令
	{
		static uint16_t u16ZhiLiCnt = 0; 
			
		if(1 == SwiInput.b1SlowRequest)//龟速按键与直立行走逻辑处理
		{
			uint16_t u16ZhiliTime = 0 ;//直立行走时间设置
			uint8_t u8nolimitflag= 0;
			
			u16ZhiliTime = i32GetPara(PARA_AccAndDecAntiPinch);
			sgLock.SpeedRate.b1Spd1Slow = 1;//龟速	
			
			if(u16ZhiliTime<6)
			{
				u16ZhiliTime = 6000;
			}
			else if(u16ZhiliTime>20)
			{
				u8nolimitflag = 1;
			}
			else
			{
				u16ZhiliTime = u16ZhiliTime * 1000;
			}
			
			if( 0 == SwiInput.b1SafeLock)
			{
				if( u16ZhiLiCnt < 200/USER_ECU_PERIOD)
				{
					u16ZhiLiCnt ++;
				}
				else if((u16ZhiLiCnt < u16ZhiliTime/USER_ECU_PERIOD)||(u8nolimitflag))//进入直立行走状态
				{
					u16ZhiLiCnt ++;
					sgLock.SpeedRate.b1Spd4Zhili = 1;
					sgActLogic.b1ZhiLiState = 1;
				}
				else//超时退出退出
				{
					sgActLogic.b1ZhiLiState = 0;
					sgLock.SpeedRate.b1Spd4Zhili = 0;					
				}
			}
			else//使能按下退出
			{
				sgActLogic.b1ZhiLiState = 0;
				sgLock.SpeedRate.b1Spd4Zhili = 0;					
			}
		}
		else//松开龟速，重置
		{
			u16ZhiLiCnt = 0;
			sgActLogic.b1ZhiLiState = 0;
			sgLock.SpeedRate.b1Spd4Zhili = 0;	
			sgLock.SpeedRate.b1Spd1Slow = 0;//龟速				
		}
		gCanSendPdoInfo.CanSend260Info.u8Data[0] = 0 ; 
		if(((1 == SwiInput.b1SafeLock)||(1 == sgActLogic.b1ZhiLiState)))//直立行走或使能行走
		{
			if(1 == SwiInput.b1Forward)
			{
				sgActLogic.b1ForwardAct = 1;
				gCanSendPdoInfo.CanSend260Info.b1ForwardState =1;
			}
			
			if(1 == SwiInput.b1Backward)
			{
				sgActLogic.b1BackwardAct = 1;
				gCanSendPdoInfo.CanSend260Info.b1BackwardState =1;
			}
			if(1 == sgActLogic.b1ZhiLiState)
			{
				gCanSendPdoInfo.CanSend260Info.b1Upright = 1;
			}
		}
	}
	
	sgLock.PumpRate.u8Data = 0;
	if(0 == sgLock.Lift.u8Data)
	{
		if(1 == SwiInput.b1LiftUp)
		{
			if(((1 ==sgSwiInput.b1SafeLock)
			&&(1 == sgUserInfo.b1LiftLock))
			||(0 == sgUserInfo.b1LiftLock))
			sgActLogic.b1LiftUpAct = 1;
		}
	}		
	if(0 == sgLock.Action.u8Data)
	{
		if(1 == SwiInput.b1OrbitMove)
		{
			sgActLogic.b1OrbitMove = 1;
			sgLock.PumpRate.b1OrbitRate = 1;
		}
		if(1 == SwiInput.b1Tilt)
		{
			sgActLogic.b1Tilt = 1;
			sgLock.PumpRate.b1Tilt = 1;
		}
	}
	
	/*多个动作优先级&报警处理*/
	{
		static xActLogic ActionRecord;//实际油泵输出
		
		if(0 !=(sgActLogic.u8Data[1]&(sgActLogic.u8Data[1] -1)))//复数个油泵逻辑请求
		{
			i32ErrCodeSet(MUTL_PUMP_REQ_ERR);
			sgActLogic.u8Data[1] = 0;//清空油泵动作
		}
		if((0 == SwiInput.b1LiftUp)
			&&(0 == SwiInput.b1OrbitMove)
			&&(0 == SwiInput.b1Tilt))//复位清除警报
		{
			i32ErrCodeClr(MUTL_PUMP_REQ_ERR);
		}
		
		if(1 == sgUserInfo.b1MoveLiftMode)//行走起升互锁
		{
			if((0 != sgActLogic.u8Data[0])
			 &&(0 != sgActLogic.u8Data[1]))
			{
				sgActLogic.u8Data[0] =ActionRecord.u8Data[0];
				sgActLogic.u8Data[1] =ActionRecord.u8Data[1];
			}
		}
		
		ActionRecord = sgActLogic;
	}
	
	i32SetPara(PARA_ExtSignal, SwiInput.u16data);
	sgSwiInput.u16data = SwiInput.u16data;

	
	if(1 == i32LocalDiGet(SAFELOCK_SWI))
	{
		SwiInput.b1SafeLock = 1;
		gCanSendPdoInfo.CanSend260Info.b1Maint = 1;
	}
	else
	{
		gCanSendPdoInfo.CanSend260Info.b1Maint = 0;
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
		i32ErrCodeSet(RevData->u8ErrCode - 1);	//??
		if(40 == RevData->u8ErrCode)
		{
			i32ErrCodeSet(EMS_LOCK_ERR);
			sgLock.Movement.b1EMS = 1;
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
	tmp = i16MotorSpd / sgUserInfo.u16RatioOfTransmission;
	/*Motor Speed*/
	gCanSendPdoInfo.CanSend260Info.u8Movespeed = tmp ;
	__enable_irq();
	
//	i32LogWrite(DEBUG, LOG_USER, "Motor Rev Spd = %d\r\n", i16MotorSpd);
	__disable_irq();
	tmp = i16MotorSpd / sgUserInfo.u16RatioOfTransmission;
	/*Motor Speed*/
//	gCanSendPdoInfo.CanSend33CInfo.u8SpdHighByte = tmp >> 8;
//	gCanSendPdoInfo.CanSend33CInfo.u8SpdLowByte = tmp;
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


/*接收PDO处理函数*/
static void vCanRevPdoProc(void)
{
	
	static xCanRev200Info CanRev200InfoLast;

	tCanFrame CanSendFrame;
	tCanFrame CanSend0={0};
	uint8_t i = 0;
	static uint16_t cnt = 0;
	
	if(LiBattery == sgUserInfo.u8BatteryType)
	{
		gCanSendPdoInfo.CanSend260Info.u8Soc = u8BMS_SOC;
	}
	else
	{
		gCanSendPdoInfo.CanSend260Info.u8Soc = u8GetBatterySoc();
	}
	
	/*电量限制*/
	if ((gCanSendPdoInfo.CanSend260Info.u8Soc <= BAT_LOW_ERR_VAL)
			||(0 != (u8BMS_ERRCODE & 0x01)))	//禁止动作
	{
		if(gCanSendPdoInfo.CanSend260Info.u8Soc <= BAT_LOW_ERR_VAL)
		{
			i32ErrCodeSet(BAT_LOW_2_ERR);
		}
		sgLock.Action.b1BMS_Erro = 1; 
	}
	else if ((gCanSendPdoInfo.CanSend260Info.u8Soc <= BAT_LOW_WARING_VAL)
				||(0 !=(u8BMS_ERRCODE & 0x0A)))
	{
		sgLock.Action.b1BMS_Erro = 0; 
		i32ErrCodeClr(BAT_LOW_2_ERR);
		sgLock.Lift.b1BMSWarning = 1;
		sgLock.SpeedRate.b1Spd2BMS = 1;
		if(gCanSendPdoInfo.CanSend260Info.u8Soc <= BAT_LOW_WARING_VAL)
		{
			i32ErrCodeSet(BAT_LOW_1_ERR);
		}
	}
	else
	{
		sgLock.Lift.b1BMSWarning = 0;
		sgLock.Action.b1BMS_Erro = 0; 
		sgLock.SpeedRate.b1Spd2BMS = 0;
		i32ErrCodeClr(BAT_LOW_1_ERR);
		i32ErrCodeClr(BAT_LOW_2_ERR);
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

/*****************************
掉线检测
**********************************/
static void vCanId360Proc(tCanFrame * CanFrame)
{	
	static xCanRev360Info CanRev360InfoLast;

	//if(0 != memcmp((char*)CanRev360InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo3.u8Data, sizeof(CanRev360InfoLast)))
	{
		memcpy((char*)CanRev360InfoLast.u8Data, (char*)CanFrame->u8Data, sizeof(CanRev360InfoLast));
		gCanSendPdoInfo.CanSend260Info.u8ErrorSteer = CanRev360InfoLast.u8ErrSteer;
		if(0 != CanRev360InfoLast.u8ErrSteer)
		{	
			sgLock.Action.b1SteerErro = 1;
		}
		else
		{
			sgLock.Action.b1SteerErro = 0;
		}
		i16SteerAngle = CanRev360InfoLast.i16SteerAngle;
	}
	u16ID360Cnt = 0; 
}

static void vCanId200Proc(tCanFrame * CanFrame)
{
	static xCanRev200Info CanRev200InfoLast;
	memcpy((char*)CanRev200InfoLast.u8Data, (char*)CanFrame->u8Data, sizeof(CanRev200InfoLast));
	u8BMS_SOC = CanRev200InfoLast.u8Soc;
	u16ID200Cnt = 0;
	u8BMS_FLAG |= 1;
}

static void vCanId190Proc(tCanFrame * CanFrame)
{
	static xCanRev190Info CanRev190InfoLast;
	memcpy((char*)CanRev190InfoLast.u8Data, (char*)CanFrame->u8Data, sizeof(CanRev190InfoLast));
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
		u8BMS_ERRCODE = CanRev190InfoLast.u8BMS_ERR_Require;
	}
	u16ID190Cnt = 0;
	u8BMS_FLAG |= (1<<1);
}
/*********************************************************
输出部分
***********************************************************/
/*发送给Mst的回调函数*/
static void vMstSendProc(xMstSendPara *SendData) 
{
	
	static xActLogic sgLastAction;
	static uint8_t u8MainConnectCnt = 0;
	
	float fTmp = 0.0;
	xMstSendStat LastStatus;
	
	int16_t i16Spd = 0;
	uint8_t u8pump = 0;
	
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

		

	/*行走电机*/
	if(0 != (sgActLogic.u8Data[0]^sgLastAction.u8Data[0]))//前后动作状态发生变化
	{
		//速度处理
	}
	
	if(0 != sgActLogic.u8Data[0])//存在行走指令
		i16Spd = u16MotorVal ;
	else
		i16Spd = 0;
	
	if (i16Spd >= MOTOR_MAX_SPEED_VALUE)
	{
		i16Spd = MOTOR_MAX_SPEED_VALUE;
	}
	
	
	
	/*油泵电机*/
	if(0 != sgActLogic.u8Data[1])//存在油泵指令
	{
			u8pump = u8PumpVal;
	}
	else
		u8pump = 0;

	if (u8pump >= PUMP_MAX_VALUE)
	{
		u8pump = PUMP_MAX_VALUE;
	}

	sgLastAction = sgActLogic;//更新历史动作
	
	/*指令发送*/
	if (0 != i16Spd)
	{
		if (1 == sgActLogic.b1ForwardAct)
		{
			SendData->b1ForwardReq = 1;
		}
		else if (1 == sgActLogic.b1BackwardAct)
		{
			SendData->b1BackwardReq = 1;
		}
	}
	
	if (1 == sgSwiInput.b1Ems)
	{
		SendData->b1EmsReq = 1;
	}
	

	if(0 != u8pump)
		SendData->b1LiftReq = 1;
	
	
	SendData->u8TargetHigh = i16Spd >> 8;
	SendData->u8TargetLow = i16Spd;
	
	SendData->u8PumpTarget = u8pump;
	
	if((0 != i16Spd)||(0 != u8pump))
		SendData->b1ServoOn =1;


	{
		static uint32_t u32Cnt = 0;
		u32Cnt++;
		//if (0 == (u32Cnt % 5))
		{
			/*lilu 20230819 For Test*/
			{
				i32SetPara(PARA_ForwardValveCurrent, i16Spd);		/*Send Motor Value*/
				i32SetPara(PARA_BackValveCurrent, u16MotorVal);		/*Rev Motor Value*/
				i32SetPara(PARA_PropValveCurrent, u8PropValue);	/*Prop Current*/
				i32SetPara(PARA_LiftValveCurrent, i32PropValue);		/*Send Pump Value*/
				i32SetPara(PARA_OnOffValveCurrent, SendData->buf[2]);			/*Send Status*/
				i32SetPara(PARA_LoadRate, sgLock.Lift.u8Data);				/*Valve NoFlag*/
				i32SetPara(PARA_CalibrationStatus,gCanRevPdoInfo.CanRevInfo3.i16SteerAngle);		/*Value State*/
//				i32SetPara(PARA_TurnRightValveCurrent, ((uint16_t)sgValvesInfo.u8NoAct << 8) | ((uint16_t)sgValvesInfo.b4NoMove << 4) | ((uint16_t)sgValvesInfo.b4NoLiftUp));
//				i32SetPara(PARA_TurnLeftValveCurrent, ((uint16_t)sgValvesInfo.b4Gear1Spd << 12) | ((uint16_t)sgValvesInfo.b4Gear2Spd << 8) | \
//					((uint16_t)sgValvesInfo.b4Gear3Spd << 4) | ((uint16_t)sgValvesInfo.b4Gear4Spd));
				i32SetPara(PARA_AngleValue, (uint16_t)i32LocalAiGetValue(AI_B_AI1_R));		/*AI1*/
				i32SetPara(PARA_PressureVlaue1, (uint16_t)i32LocalAiGetValue(AI_B_AI2_R));	/*AI2*/
				i32SetPara(PARA_PressureVlaue2, (uint16_t)i32LocalAiGetValue(AI_B_AI3_R));	/*AI3*/
				i32SetPara(PARA_ExtSignal, sgActLogic.u8Data[0]);						/*Swi*/
				i32SetPara(PARA_PcuKeyInfo, sgActLogic.u8Data[1]);									/*ErrCode*/
				i32SetPara(PARA_BmsSoc, (gCanSendPdoInfo.CanSend260Info.u8Soc *2.5));		/*BMS SOC*/
//				i32SetPara(PARA_BrakeValveCurrent, abs(gCanRevPdoInfo.CanRevInfo2.i16SteerAngle));
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

	vMstRevRegister(vMstRevProc);
	vMstSendRegister(vMstSendProc);
	vAiErrReg(vAiErrCallBack);
	
	xRevCallBackProc CanId190 = {.u32CanId = 0x190, .u32Data = 0, .CallBack = vCanId190Proc};
	xRevCallBackProc CanId200 = {.u32CanId = 0x200, .u32Data = 0, .CallBack = vCanId200Proc};
	xRevCallBackProc CanId360 = {.u32CanId = 0x360, .u32Data = 0, .CallBack = vCanId360Proc};
	
	vCanRevMsgRegister(&CanId190);
	vCanRevMsgRegister(&CanId200);
	vCanRevMsgRegister(&CanId360);
	
	memset(&sgActLogic, 0, sizeof(sgActLogic));
	
	sgUserInfo.u8PumpMotorGear1 = i32GetPara(PARA_PumpMotorGear1) * PUMP_RANGE / 100;			/*前后移动*/
	sgUserInfo.u8PumpMotorGear2 = i32GetPara(PARA_PumpMotorGear2) * PUMP_RANGE / 100;			/*倾斜*/
	sgUserInfo.u8LiftSpeed = i32GetPara(PARA_LiftSpeed);									/*起升*/
	
	sgUserInfo.b1LiftMode = i32GetPara(LIFT_MODE_PARA);						/*起升模式-制动踏板类型*/
	
	sgUserInfo.u16Gear1Spd = i32GetPara(PARA_Gear1Spd)  ;	/*1档速度*/
	sgUserInfo.u16Gear2Spd = i32GetPara(PARA_Gear2Spd) 	;	/*2档速度*/
	sgUserInfo.u16Gear3Spd = i32GetPara(PARA_Gear3Spd)  ;	/*3档速度*/
	sgUserInfo.u16Gear4Spd = i32GetPara(PARA_Gear4Spd)  ;	/*4档速度*/
	
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
	sgUserInfo.u16LiftDownRange	= (sgUserInfo.u16LiftDownMax - sgUserInfo.u16LiftDownMin ) >> 1;
	sgUserInfo.u16LiftDownMid = i32GetPara(LIFT_DOWN_THROTTLE_MID) * PUMP_RANGE / 100;
	sgUserInfo.u16LiftDownMidValue	= (sgUserInfo.u16LiftDownMax + sgUserInfo.u16LiftDownMin) >> 1;
	
	sgUserInfo.fPropMinCurrent = i32GetPara(PROP_MIN_CURRENT) / 1000.0;		/*比例阀最小电流*/
	sgUserInfo.fPropMaxCurrent = i32GetPara(PROP_MAX_CURRENT) / 1000.0;		/*比例阀最大电流*/
	
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
		sgUserInfo.b1HourConutMode = u16Tmp & 0x01;						/*bit0: HourCount Mode*/
		sgUserInfo.b1CarType = (u16Tmp>>1) & 0x01; 						/*bit6:pedal&fence*/
		sgUserInfo.b1LiftLock = (u16Tmp >> 2) & 0x01;					/*bit2: Lift by Lock */
		sgUserInfo.b1SpeedLimitAfterLift = (u16Tmp>>3) & 0x01;/*bit7:*/
		sgUserInfo.b1MoveLiftMode = (u16Tmp >> 4) & 0x01;			/*bit4: Move and Lift */
		sgUserInfo.b1SteerCheck = (u16Tmp>>5) & 0x01; 				/*bit5:Steer Controller Online Check*/
		sgUserInfo.b1StartUpLock = (u16Tmp>>6) & 0x01; 				/*bit5:Steer Controller Online Check*/
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
		sgSwiInput.b1HeightLimitState = sgSaveState.b1HeightSpdLimit;
		sgUserInfo.b1PasswordFunc = sgSaveState.b1PasswordFunc;
		u16EepromRead(PARA_HourSetTime, &u16Tmp, 1);	
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
			i32SetPara(RENTAL_INFO, (u16RentalInfo & 0x07));
		}
	}

	
	__disable_irq();
	gCanSendPdoInfo.CanSend260Info.u8HourCountL = (u32HourCount/10) & 0xFF;
	gCanSendPdoInfo.CanSend260Info.u8HourCountH = ((u32HourCount/10) >> 8) & 0xFF;
	gCanSendPdoInfo.CanSend260Info.u8Soc = u8GetBatterySoc();
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
		vSwiInitChcek();
	}
	
	if(1 == u8EcuProcFlag)
	{
		if (LiBattery == sgUserInfo.u8BatteryType)
		{
			if((u16ID190Cnt>CAN_190_LOST_NO)
				||(u16ID200Cnt>CAN_200_LOST_NO))
			{
				i32ErrCodeSet(BMS_NOCAN_ERR);
				sgLock.Action.b1CanIDlost1 = 1;			
			}
			else
			{
				u16ID190Cnt++;
				u16ID200Cnt++;
				i32ErrCodeClr(BMS_NOCAN_ERR);
				sgLock.Action.b1CanIDlost1 = 0;			
			}
		}
		else if(0 != u8BMS_FLAG)//非锂电状态下，检测到报文，报电池类型错误
		{		
			i32ErrCodeSet(BAT_TYPE_ERR);
			sgLock.Action.b1CanIDlost1 = 1;	
		}
		if(1 == sgUserInfo.b1SteerCheck)
		{
			if (u16ID360Cnt < CAN_360_LOST_NO)
			{
				u16ID360Cnt++;
				i32ErrCodeClr(STEER_NOCAN_ERR);
				sgLock.Action.b1CanIDlost2 = 0;
			}
			else
			{
				i32ErrCodeSet(STEER_NOCAN_ERR);
				sgLock.Action.b1CanIDlost2 = 1;			
			}
		}
		
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
			__disable_irq();
			gCanSendPdoInfo.CanSend260Info.u8HourCountL = (u32HourCount/10) & 0xFF;
			gCanSendPdoInfo.CanSend260Info.u8HourCountH = ((u32HourCount/10) >> 8) & 0xFF;
			__enable_irq();

//		/*Rental Info*/
//		{
//			uint16_t u16RentalInfo = 0;
//			u16RentalInfo = i32GetPara(RENTAL_INFO);
//			sgUserInfo.b1RentalStop = u16RentalInfo & 0x01;
//			sgUserInfo.b1RentalStart = (u16RentalInfo >> 1) & 0x01;
//			sgUserInfo.b1RentalMode = (u16RentalInfo >> 2) & 0x01;
//			sgUserInfo.b1HourCntClr = (u16RentalInfo >> 3) & 0x01;
//			if (1 == sgUserInfo.b1HourCntClr) /*lilu 20230823 add clr hourcnt func*/
//			{
//				u32HourCount = 0;
//				sgUserInfo.b1HourCntClr = 0;
//				u16SecCnt = 0;
//				i32SetPara(RENTAL_INFO, (u16RentalInfo & 0x07));
//				vHourCountWrite(u32HourCount);
//				__disable_irq();
//				gCanSendPdoInfo.CanSend260Info.u8HourCountL = (u32HourCount/10) & 0xFF;
//				gCanSendPdoInfo.CanSend260Info.u8HourCountH = ((u32HourCount/10) >> 8) & 0xFF;
//				__enable_irq();
//			}
//		}
	}
	
	{
			if (0x01 == sgUserInfo.b1StartUpLock) /*lilu 20230823 add clr hourcnt func*/
			{
				u32HourCount = 0;
				sgUserInfo.b1LiftPedal = 0;
				u16SecCnt = 0;
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
