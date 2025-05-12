/*******************************************************************************
*通用程序框架* 						   *
* Author: QExpand; ShiJin                                                        *
* Date: 2023/10/32    														   *
* Revision:	 														 		   *
*******************************************************************************/
#include "UserCanComm.h"
#include "UserForkLiftProc_ChuFeng.h"
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
#include "PcuProc.h"
//#include "UserCommon.h"




#if (USER_TYPE == USER_CHUFENG_2IN1_LOGIC)
/*参数配置部分*/

/*报文配置*/
const static xPdoParameter  sgPdoPara = 
{		/*标准CanOpen，100，200，300，400/+nodeid*/
		/*u8Type为0xFF时周期发送*/
		/*特殊报文请用i32CanWrite周期发送*/
	.TpdoPara = {
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 20, .u16CanId = 0X000 },
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50, .u16CanId = 0x000 },
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50, .u16CanId = 0x000 },
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50, .u16CanId = 0x000 },
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50, .u16CanId = 0x000 },
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50, .u16CanId = 0x000 },
	},
	/*Canopen接收*/
	/*设置对应id及使能*/
	.RpdoPara = {
		{.b1Flag = 0, .b11CanRevId = 0x000},
		{.b1Flag = 0, .b11CanRevId = 0x000},
		{.b1Flag = 0, .b11CanRevId = 0x000},
		{.b1Flag = 0, .b11CanRevId = 0x000},
		{.b1Flag = 0, .b11CanRevId = 0x000},
		{.b1Flag = 0, .b11CanRevId = 0x000},
	},
};

/*开关量输入*/
typedef union
{
	uint8_t u8data;
	struct
	{
		uint8_t b1PcuSwi: 1;
		uint8_t b1TiltSwi: 1;
		uint8_t b1PitSwi: 1;
		uint8_t b1UpLimitSwi: 1;
		uint8_t b1DownLimitSwi: 1;
		uint8_t b1LowerCtlUp:1;
		uint8_t b1LowerCtlDown: 1;
	};
}xSwiInput;

/*PCU Input*/
typedef union
{
	uint8_t u8data;
	struct
	{
		uint8_t b1MoveMode:1;
		uint8_t b1LiftMode:1;
		uint8_t b1Slow:1;
		uint8_t b1Speaker:1;
		uint8_t b1TurnLeft:1;
		uint8_t b1TurnRight:1;
		uint8_t b1Enable:1;
		uint8_t b1Reserve:1;
	};
}xPCUInfo;

/*动作限制设置*/
typedef union
{
	uint16_t u8Data;
	struct
	{
		uint8_t b1Init:1;
		uint8_t b1BMS_Erro:1;
		uint8_t b1SteerErro:1;
		uint8_t b1Error:1;
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
		uint8_t b1Warning:1;
		uint8_t b1Error:1;
		uint8_t b7Rserve1:4;
	};
}xMove;

typedef union
{
	uint8_t u8Data;
	struct
	{
		uint8_t b1Init:1;
		uint8_t b1Act:1;
		uint8_t b1Warning:1;
		uint8_t b1Error:1;
		uint8_t b7Rserve1:4;
	};
}xSteer;

typedef union
{
	uint8_t u8Data;
	struct
	{
		uint8_t b1Init:1;
		uint8_t b1Act:1;
		uint8_t b1BMSWarning:1;
		uint8_t b1HeightLimit:1;
		uint8_t b1TiltError:1;
		uint8_t b1Error:1;
		uint8_t b7Rserve1:1;
	
	};
}xLift;

typedef union
{
	uint8_t u8Data;
	struct
	{
		uint8_t b1AfterLift:1;
		uint8_t b1Slow:1;
		uint8_t b1SpdLowBat:1;
		uint8_t b1Steer:1;
		uint8_t b4Rserve:4;
	};
}xSpd;

typedef union
{
	uint8_t u8Data;
	struct
	{
		uint8_t b1Act:1;
		uint8_t b1OrbitRate:1;
		uint8_t b1Tilt:1;
		uint8_t b6Rserve:5;
	};
}xPumpSpd;

typedef union
{
	uint8_t u8Data;
	struct
	{
		uint8_t b1Act:1;
		uint8_t b1OrbitRate:1;
		uint8_t b1Error:1;
		uint8_t b6Rserve:5;
	};
}xDown;

typedef struct
{
	xAct Action;
	xMove Movement;
	xLift Lift;
	xSpd SpeedRate; 
	xPumpSpd PumpRate;
	xSteer SteerLock;
	xDown Down;
}xActLimit;

/*输出动作逻辑*/
typedef union
{
	uint8_t u8Data;	
	struct
	{
		/*行走动作逻辑*/
		uint8_t b1ForwardAct:1;
		uint8_t b1BackwardAct:1;
		uint8_t b1LiftUpAct:1;
		uint8_t b1LiftDownAct:1;
		uint8_t b1TurnLeft:1;
		uint8_t b1TurnRight: 1;
		uint8_t b2Reserve2:2;
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

//	uint8_t		u8Reserve1;					/*14*/
//	uint8_t		u8Reserve2;					/*15*/

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
	
//	uint8_t		u8Reserve3;					/*30*/
//	uint8_t		u8Reserve4;					/*31*/	

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

//	uint8_t		u8Reserve5;					/*46*/
//	uint8_t		u8Reserve6;					/*47*/
	
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

//	uint8_t		u8Reserve7;				/*62*/
//	uint8_t		u8Reserve8;				/*63*/
	
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

	uint16_t		u16SetDescentHeightValue;		
	uint8_t		u8ReleaseBrake;

//	uint8_t		u8Reserve9;				/*78*/
//	uint8_t		u8Reserve10;				/*79*/
	
	uint16_t		u8SetOutHeight;			/*80*/
	uint16_t		u8AngleSimulationUpLimit;	
	uint16_t		u8AngleSimulationDownLimit;
	
//	uint16_t		u8PumpDriveCurrentLimitRatio0;			
//	uint16_t		u8PumpSpdAccRatio0;
//	uint16_t		u8PropDKp0;						
//	uint16_t		u8PropDKi0;						
//	uint16_t		u8PropDMaxCurrent0;				
//	uint16_t		u8PropDMinCurrent0;
//	uint16_t		u8PropDAccPeriod0;
//	uint16_t		u8PropDDitherPeriod0;	
//	uint16_t		u8PropDDitherRatio0;			
//	uint16_t		u8PropValveResistance0;
//	
//	uint8_t		u8Reserve11;					/*93*/
//	uint8_t		u8Reserve12;					/*94*/
//	uint8_t		u8Reserve13;					/*95*/
//	
//	
//	uint16_t		u8PumpDriveCurrentLimitRatio1;		/*96*/	
//	uint16_t		u8PumpSpdAccRatio1;
//	uint16_t		u8PropDKp1;						
//	uint16_t		u8PropDKi1;						
//	uint16_t		u8PropDMaxCurrent1;				
//	uint16_t		u8PropDMinCurrent1;
//	uint16_t		u8PropDAccPeriod1;
//	uint16_t		u8PropDDitherPeriod1;	
//	uint16_t		u8PropDDitherRatio1;			
//	uint16_t		u8PropValveResistance1;
	
	uint16_t		u16CanBaudRate;

	uint16_t		u8EmptyPressure;
	uint16_t		u8FullPressure;
	uint16_t		u8DriverFlag;
	
//	uint8_t		u8Reserve14;					/*110*/
//	uint8_t		u8Reserve15;					/*111*/
	
	uint16_t		u8MinAngle;					/*112*/
	uint16_t		u8MaxAngle;
	uint16_t		u8BatSocPalyBack;
	uint16_t		u8ValveType;
	uint16_t		u8AnticollisionFunc;
	uint16_t		u8ValueOpenLoopCurrent;
	uint16_t		u8ValueOpenPercentage;
	uint16_t		u16CanOpenNodeId;

	
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
	
//	uint8_t		u8Reserve16;						/*158*/
//	uint8_t		u8Reserve17;						/*159*/
	
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
	
	/*user info*/
	
	#if 0
	float		fLiftSpdPer5msAccStep;
	float		fDownSpdPer5msAccStep;
	float		fMoveSpdPer5msAccStep;
	float		f
	
	float		fLiftSpdPer5msDecStep;
	float		fDownSpdPer5msDecStep;
	float		fMoveSpdPer5msDecStep;
	#endif
	
	float		fFastMoveSpdPer5msAccStep;
	float		fSlowMoveSpdPer5msAccStep;
	float		fMoveAfterLiftSpdPer5msAccStep;
	float		fLiftSpdPer5msAccStep;
	float		fTurnSpdPer5msAccStep;
	
	float		fFastMoveSpdPer5msDecStep;
	float		fSlowMoveSpdPer5msDecStep;
	float		fMoveAfterLiftSpdPer5msDecStep;
	float		fLiftSpdPer5msDecStep;
	float		fTurnSpdPer5msDecStep;
	
	float		fPropMinCurrent0;
	float		fPropMaxCurrent0;
	
	float		fPropMinCurrent1;
	float		fPropMaxCurrent1;
	
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
	uint8_t u8Data;
	struct
	{
		uint8_t b1Error:1;
		uint8_t b1Warning:1;
		uint8_t PCUBeep:1;
		uint8_t PCUSilence:1;
	};
}xErrorState;




static xErrCodeInfo sgErrCodeInfo[ErrCodeMax] = 
{
	#if 1
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//1		反馈超速
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//2		内核运行错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//3		控制器长时间过载
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//4		位置超差
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//5		电子齿轮位置指令变化超电机最大速度
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//6		速度模式，速度指令超过电机最大转速
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//7		转矩模式，转矩指令超过最大转矩
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//8		速度传感器错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//9		速度传感器方向错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//10	无
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//11	电机2分钟最大电流
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//12	电机过流
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//13	母线电容充电故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 10,},	//14	主接触器故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//15	制动器故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//16	电池电压低
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//17	电池电压高
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//18	功率板过温
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//19	电机高温
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//20	电位计短路或电压
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//21	主接触器触电熔接
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//22	5v输出故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//23	id监
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//24	主接触器驱动故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//25	功率模块故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//26	Can通讯故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//27	电压高于最大电压2V
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//28	无
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//29	电机电阻异常
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//30	回中错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//31	电压轻度过低
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//32	功率板轻度过温
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//33	功率板低温
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//34	电机轻度高温
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//35	12v故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//36	DO3故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//37	DO4故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//38	eeprom参数读写错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//39	参数编号错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//40	上电IO异常
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//41	电量低于20%
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//42	电量低于10%
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//43	电压达到限速阈值
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//50
		
		
		
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//51	DO1
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 1,},	//52  DO2		
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 54,},//53	DO3起升阀		
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 56,},//54	DO4右转
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 57,},//55	DO5左转		
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 58,},	//56	DO6喇叭
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 59,},//57	DO7高低速		
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 53,},//58	DO8后退阀
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 52,},//59	DO9前进			
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 58,},	//60	DO10蜂鸣器	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//61	DO11
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//62	DO12
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//63		DO13
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//64	DO14
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//65	DO15		
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 27,},//66 下降阀0		
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 55,},//67 下降阀1
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//68预留
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//69预留
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//70
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//71	模拟量1
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//72	模拟量2
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//73	模拟量3
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//74	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//75
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//76
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//77	编码器错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//78
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//79
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//80
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//81	主从MCU故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//82	主从MCU脉冲异常
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//83	看门狗异常
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//84	PCU通讯错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//85	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//86	写参数错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//87	读参数错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//88	参数超限
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//89	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//90
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//91
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//92
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//93
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//94
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//95
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//96
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//97
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//98
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//99
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//100
		
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//101	初始化
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1,	.b8UserErr = 2,},	//102	通信错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 3,},	//103	无效选项
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 4,},	//104	内存数据错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 5,},	//105	锂电掉线
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 6,},	//106	上电时按下起升
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 7,},	//107	上电时按下龟速
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 8,},	//108	上电时按下行走
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 9,},	//109	GPS掉线
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 10,},//110	主接触器连接错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 11,},//111	无
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 12,},//112	启动时下控错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 13,},//113	BMS电池温差大-2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 14,},//114	BMS电池温度高-1
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 15,},//115 BMS-放电温度过高2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 16,},//116	BMS-放电电流过高1
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 17,},//117	BMS-放电电流过高2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 18,},//118	坑洞保护错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 19,},//119	BMS-总电压过低1
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 20,},//120	BMS-总电压过低2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 21,},//121	BMS-单体电压过低1
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 22,},//122	BMS-单体电压过低2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 23,},
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 24,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 25,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 26,},
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 27,},//127	下降阀2错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 28,},
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 29,},
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 30,},//130	BMS-电池压差过大
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 31,},//131	压力传感器错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1,	.b8UserErr = 32,},//132	角度传感器错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 33,},//133	电池类型错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 34,},
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1,	.b8UserErr = 35,},//135	称重标定反
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 36,},//136	电池电量低一级报警
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 37,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1,	.b8UserErr = 38,},//138	未标定完成或标定失败
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1,	.b8UserErr = 39,},//139	通信故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 40,},
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 41,},//141	平台一级锁车
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 42,},//142	启动时，平台向左转向按钮按下错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 43,},//143	启动时，平台向右转向按钮按下错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1,	.b8UserErr = 44,},//144	平台二级锁车
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 45,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1,	.b8UserErr = 46,},//146	启动时，平台手柄使能开关按钮按下错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 47,},//147	启动时，平台手柄不在中位错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 48,},
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 49,},//149	心跳锁车
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 50,},//
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 51,},
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 52,},//152	前进阀
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 53,},//153	后退阀
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 54,},//154	起升阀
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 55,},//155	下降阀
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 56,},//156	右转阀
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 57,},//157	左转阀
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 58,},//158	刹车阀
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 59,},//159	并联阀
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 60,},//160	驱动器故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 61,},//161	电流传感器故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 62,},//162	驱动器硬件故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 63,},//163	泵电机开路
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 64,},//164	左电机开路
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 65,},//165	控制电压5V故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 66,},//166	动作时，检测到左转阀开路或短路
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 67,},//167	控制电压12V故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 68,},//168	电池低电量二级报警
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 69,},//169	高零位电流错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 70,},//170	驱动器母线电压过高故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 71,},//171	预充故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 72,},//172	驱动器母线电压过低故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 73,},//173	驱动器低温故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 74,},//174	驱动器高温一级故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 75,},//175	泵电机温度一级故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 76,},//176	泵电机编码器故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 77,},//177	电机编码错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 78,},//178	泵电机过流类故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 79,},//179	泵电机温度二级故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 80,},//180	超过 80%负载报警
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},//181	驱动器温度二级故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 82,},//182	右刹车故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 83,},//183	左刹车故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 84,},//184	泵电机堵转、失速
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 85,},//185	左牵引电机堵转、失速
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 86,},//186	由牵引电机堵转、失速
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 87,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 88,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 89,},//189	驱动器运行时间过长故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 90,},//190	超过 90%负载报警
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 91,},//191	左电机电流过流故障	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 92,},//192	右电机电流过流故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 93,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 94,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 95,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 96,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 97,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 98,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 99,},//199	超过 99%负载报警
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 100,},//200	平台超载报警
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 101,},//201	机器倾斜超过安全限定错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 88,},//202	标定中
		
	#else
	
	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//停止所有 	//1	系统初始化错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//停止所有 	//2	系统通信错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//停止所有 	//3	无效选项设置错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//停止所有 	//4	闪存数据错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//停止所有 	//5	锂电池通讯丢失
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//停止PCU  	//6	上电时举升按键按下
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//停止PCU  	//7	上电时龟速按键按下
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//停止PCU  	//8	上电时行走按键按下
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//只是报警   	//9	GPS连接错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//只能下降 	//10	主接触器故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//暂无			//11	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//待定：停止所有底盘控制	//12	启动时底盘上升或下降按钮打开错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//只能下降 	//13	BMS-电池温差过大2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//降低功率输出 	//14	BMS-电池温度过高1
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//只能下降 	//15	BMS-放电温度过高2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//降低功率输出 	//16	BMS-放电电流过高1
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//只能下降 	//17	BMS-放电电流过高2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//降低功率输出 	//18	坑洞保护错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//暂无		//19	BMS-总电压过低1
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//只能下降 	//20	BMS-总电压过低2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//降低功率输出 	//21	BMS-单体电压过低1
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//只能下降 	//22	BMS-单体电压过低2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//暂无		//23	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//暂无		//24	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//暂无//		//25	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//暂无//		//26	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//停止所有 	//27	下降阀2错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//暂无//		//28	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//暂无//		//29	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//只能下降 	//30	BMS-电池压差过大
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//停止起升行走、只转向下降	//31	压力传感器错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//停止起升行走、只转向下降	//32	角度传感器错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 0, .b1NoForWard = 0, },	//禁止起升	//33	电池类型错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//暂无//		//34	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//停止起升行走、只转向下降	//35	称重标定反
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//降低功率输出 	//36	电池电量低一级报警
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//暂无//		//37	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//停止起升行走、只转向下降	//38	未标定完成或标定失败
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//停止起升行走、只转向下降	//39	通信故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//暂无//		//40	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 0, .b1NoForWard = 0, },	//禁止起升	//41	平台一级锁车
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//只是报警   	//42	启动时，平台向左转向按钮按下错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//只是报警   	//43	启动时，平台向右转向按钮按下错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//停止起升行走、只转向下降	//44	平台二级锁车
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//暂无//		//45	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//停止PCU	//46	启动时，平台手柄使能开关按钮按下错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//降低功率输出 	//47	启动时，平台手柄不在中位错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//暂无//		//48	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//暂无//		//49	心跳锁车
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//暂无//		//50	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//暂无//		//51	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//只能下降 	//52	前进阀错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//只能下降 	//53	后退阀错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//只能下降 	//54	举升阀错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//停止所有 	//55	下降阀错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//只能下降 	//56	右转阀错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//只能下降 	//57	左转阀错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//暂无//		//58	刹车阀错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//只能下降 	//59	并联阀故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//只是报警   	//60	驱动器故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//停止所有 	//61	驱动器电流传感器故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//只能下降 	//62	驱动器硬件损坏故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//只能下降 	//63	泵电驱开路故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//只能下降 	//64	左电驱开路故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//停止所有 	//65	控制电压5V故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//只能下降 	//66	动作时，检测到左转阀开路或短路
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//停止所有 	//67	控制电压12V故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//只能下降 	//68	电池低电量二级报警
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//暂无//		//69	高零位电流错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//只能下降 	//70	驱动器母线电压过高故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//只能下降 	//71	预充故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//降低功率输出 	//72	驱动器母线电压过低故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//降低功率输出 	//73	驱动器低温故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//降低功率输出 	//74	驱动器高温一级故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//降低功率输出 	//75	泵电机温度一级故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//只能下降 	//76	泵电机编码器故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//只能下降 	//77	电机编码错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//只能下降 	//78	泵电机过流类故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//只能下降 	//79	泵电机温度二级故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//只是报警   	//80	超过 80%负载报警
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//只能下降 	//81	驱动器温度二级故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 1, .b1NoForWard = 1, },	//禁止行走	//82	右刹车故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 1, .b1NoForWard = 1, },	//禁止行走	//83	左刹车故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//停止所有 	//84	泵电机堵转、失速
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//停止所有 	//85	左牵引电机堵转、失速
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//停止所有 	//86	由牵引电机堵转、失速
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//暂无//		//87	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//暂无//		//88	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//只是报警   	//89	驱动器运行时间过长故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//只是报警   	//90	超过 90%负载报警
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 1, .b1NoForWard = 1, },	//禁止行走	//91	左电机电流过流故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 1, .b1NoForWard = 1, },	//禁止行走	//92	右电机电流过流故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//暂无//		//93	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//暂无//		//94	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//暂无//		//95	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//暂无//		//96	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//暂无//		//97	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//暂无//		//98	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	//只是报警   	//99	超过 99%负载报警
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//停止所有 	//100	平台超载报警
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, },	//停止起升行走、只转向下降	//101	机器倾斜超过安全限定错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*102*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*103*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*104*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*105*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*106*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*107*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*108*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*109*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*110*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*111*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*112*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*113*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*114*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*115*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*116*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*117*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*118*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*119*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*120*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*121*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*122*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*123*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*124*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*125*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*126*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*127*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*128*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*129*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*130*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*131*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*132*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*133*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*134*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*135*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*136*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*137*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*138*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*139*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*140*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*141*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*142*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*143*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*144*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*145*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*146*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*147*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*148*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*149*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*150*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*151*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*152*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*153*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*154*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*155*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*156*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*157*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*158*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*159*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*160*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*161*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*162*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*163*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*164*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*165*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*166*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*167*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*168*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*169*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*170*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*171*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*172*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*173*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*174*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*175*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*176*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*177*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*178*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*179*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*180*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*181*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*182*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*183*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*184*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*185*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*186*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*187*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*188*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*189*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*190*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*191*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*192*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*193*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*194*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*195*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*196*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*197*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*198*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*199*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*200*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*201*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*202*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*203*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*204*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*205*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*206*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*207*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*208*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*209*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*210*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*211*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*212*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*213*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*214*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*215*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*216*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*217*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*218*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*219*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*220*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*221*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*222*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*223*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*224*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*225*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*226*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*227*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*228*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*229*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*230*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*231*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*232*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*233*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*234*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*235*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*236*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*237*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*238*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*239*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*240*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*241*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*242*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*243*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*244*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*245*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*246*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*247*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*248*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*249*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*250*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*251*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*252*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*253*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*254*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, },	/*255*/
#endif

};

/*ECU故障与显示故障转换表*/
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

typedef union
{
	uint8_t u8Data[8];
	struct 
		{
			uint8_t u8CS;
			
			uint8_t u8IndexL;
			uint8_t u8IndexH;
			
			uint8_t u8SubIndex;
			uint8_t u8DataLL;
			uint8_t u8DataLH;
			uint8_t u8DataHL;
			uint8_t u8DataHH;
		};
}xCanBlueInfo;

typedef union
{
	uint8_t u8Data;
	struct 
		{
			uint8_t b1NoAct:1;
			uint8_t b1NoPcu:1;
			uint8_t b1SpeedAfterLift:1;
			uint8_t b1NoDown:1;
			uint8_t b1NoTurn:1;
			uint8_t b1NoLift:1;
			uint8_t b1NoMove:1;
			uint8_t b1Slow:1;
		};
}xLimit;

xCanSendPdoInfo gCanSendPdoInfo ;
xCanRevPdoInfo gCanRevPdoInfo;	/*PDO接收待完善*/
/*变量*/
static xLimit sgLimit;
static xActLogic sgActLogic;
static xSwiInput sgSwiInput;
static xSaveStateInfo sgSaveState;
static uint32_t u32HourCount = 0;

static xUserInfo sgUserInfo;
static xPCUInfo sgPcuInfo;
static uint16_t u16MotroVal = 0;
static uint16_t u16MotorFdb = 0;

static xErrorState sgErrorState;

static uint8_t u8PcuMode = 0;
#define MOVE_MODE		1		/*行进模式*/
#define	LIFT_MODE		2		/*升降模式*/
#define INITIAL_MODE	0	/*Initial State*/
static uint8_t u8AntiPinchState = 0;
#define ABOVE_SWI					1
#define WAIT_RELEASE			2
#define	UNDER_SWI_DELAY		4
#define UNDER_SWI_ACT			5

//加减速参数设置
	static uint16_t u16SpeedTarget = 0;//手柄目标速度
	static uint16_t u16SpeedCmd = 0;//发送的指令速度
	static uint16_t u16Time = 0;//获取的加减速时间参数
	
/**************************初始化与逻辑限制**********************************
* 模拟量检测
* 开关检测
* 输出检测
* 比例阀检测
* PCU故障检测
*	报文掉线检测
*******************************************************************************/
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
			break;
		case AI_B_AI2_R_ERR:
			break;
		case AI_B_AI3_R_ERR:
			break;
		case AI_5V_12V_OUT1_I_ERR:
			break;
		case AI_5V_12V_OUT2_I_ERR:
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
			break;
		case DRIVER2:
			break;
		case DRIVER3:
			break;
		case DRIVER4:
			break;
		case DRIVER5:
			break;
		case DRIVER6:
			break;
		case DRIVER7:
			break;
		case DRIVER8:
			break;
		case DRIVER9:
			break;
		case DRIVER10:
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
			break;
		case PropDriverCh1:
		break;
		default:
			break;
	}
}
static void vSwiInitCheck(void)
{
	if((1 == i32LocalDiGet(LOWER_CONTROLL_UP))
		||(1 == i32LocalDiGet(LOWER_CONTROLL_DOWN)))
	{
		i32ErrCodeSet(UPDOWN_BUTTON_ERR);
	}
}
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
		case PCU_ValueNoZero:
			{
				i32ErrCodeSet(HANDLE_NOT_IN_MIDDLE_POSI_ERR);
			}
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
	#if 0
		switch(u32CanID)
	#endif
}

/****************************************输入部分**************************************
* 开关量输入 check
* PCU输入 check
* 模拟量输入	
* 报文输入	
* MCU输入	
*******************************************************************************/

/*******************************************************************************
* Name: vCanRevPdoProc(void)
* Descriptio: 正常通道PDO接收
* Input: NULL
* Output: NULL  
*******************************************************************************/
static void vCanRevPdoProc(void)
{
	
}
/*******************************************************************************
* Name: vCanID602Proc(void)
* Descriptio: 快速sod，注册
* Input: NULL
* Output: NULL  
*******************************************************************************/
static void vCanId602Proc(tCanFrame *Canframe)
{
	
	#if 0
	static xCanBlueInfo CanRev608InfoLast;
	static uint8_t u8Vector[8];
	uint32_t u32Index = 0;
	uint16_t u16ECUParaIndex = 0;
	uint16_t u16MCUParaIndex = 0;
	
	memcpy((char*)CanRev608InfoLast.u8Data, (char*) Canframe->u8Data, sizeof(CanRev608InfoLast));
	
	u32Index = CanRev608InfoLast.u8SubIndex|(CanRev608InfoLast.u8IndexL<< 8 )|(CanRev608InfoLast.u8IndexH<<16);
	
	switch (u32Index)
	{
		case 0x200001:
			u16ECUParaIndex = PARA_VehicleType;
		break;
		case :
			
			
	}
	
	
	#endif
}
/*******************************************************************************
* Name: void vPcuRevProc(xPcuRevPara *RevData)
* Descriptio: PCU接收处理
* Input: NULL
* Output: 处理 sgPcuInfo, sgActLogic
* checked: 输出动作逻辑、防夹手动作逻辑
* to be check:动作限制、喇叭
*******************************************************************************/
void vPcuRevProc(xPcuRevPara *RevData)
{
	static xPCUInfo PCUStateRecord;

	if ((0x2 == RevData->Data.b4Const1) && (0x3 == RevData->Data.b4Const2) && \
		(0x4 == RevData->Data.b4Const3) && (0x5 == RevData->Data.b4Const4) && \
		(0x6 == RevData->Data.b4Const5) && (0x7 == RevData->Data.b4Const6) && \
		(0xC == RevData->Data.b4Const7))
	{
		int8_t i8MotorVal = 0;
		if((1 == i32LocalDiGet(PCU_SWICTH)) 
		//	&&()// to be update 
		)/*添加上下控检测*/
		{
			sgPcuInfo.b1Slow = RevData->Data.b1SlowSpdSwitch;
			sgPcuInfo.b1Speaker = RevData->Data.b1SpeakerSwitch;
			sgPcuInfo.b1MoveMode = RevData->Data.b1TraSwitch;
			sgPcuInfo.b1LiftMode = RevData->Data.b1LiftingSwitch;
			sgPcuInfo.b1Enable = RevData->Data.b1EnableSwitch;
			sgPcuInfo.b1TurnRight = RevData->Data.b1TurnRightSwitch;
			sgPcuInfo.b1TurnLeft = RevData->Data.b1TurnLeftSwitch;
			/*Set Key Info*/
			i32SetPara(PARA_PcuKeyInfo, sgPcuInfo.u8data);
			i8MotorVal = RevData->Data.b4HandleCtrlHigh << 4 | RevData->Data.b4HandleCtrlLow;
			__disable_irq();
			gCanSendPdoInfo.sgHMISendPdo.WorkMode = UP_CONTROL_MODE;
			__enable_irq();
		}
		else //下控
		{
			sgPcuInfo.u8data = 0;
			sgActLogic.u8Data = 0;
		}
		if (abs(i8MotorVal) < sgUserInfo.u8DeadZoneAdjust)
		{
			i8MotorVal = 0;
		}
		i32SetPara(PARA_HandleAnalog, abs(i8MotorVal) * 100 / 127);
		
		/*Actlogic Initialize and Judge*/
		sgActLogic.u8Data = 0;
		


		
		if(INITIAL_MODE == u8PcuMode)
		{
			if(1 == sgPcuInfo.b1LiftMode)
				u8PcuMode = LIFT_MODE;
			else if(1 == sgPcuInfo.b1MoveMode)
				u8PcuMode = MOVE_MODE;
		}
		else
		{
			/*Slow Speed */
			if((MOVE_MODE == u8PcuMode)
			&& (0 == sgPcuInfo.b1Slow)
			&& (1 == PCUStateRecord.b1Slow))
			{
				sgLimit.b1Slow ^= 1;
			}
			/*Mode Change*/
			if(0 == sgPcuInfo.b1Enable)//PCU 显示有问题、有使能时按下模式切换、显示灯光不变、但是切换模式的指令会发出
			{
				if((MOVE_MODE == u8PcuMode)&&(1 == sgPcuInfo.b1LiftMode))
				{
					u8PcuMode = LIFT_MODE;
				}
				else if((LIFT_MODE == u8PcuMode)&&(1 == sgPcuInfo.b1MoveMode))
				{
					u8PcuMode = MOVE_MODE;
				}
			}
			if((MOVE_MODE == u8PcuMode)&&(1 == sgPcuInfo.b1Enable))
			{
				/*Steer Process*/
				if(0 == sgLimit.b1NoTurn)
				{
					if(1 ==sgPcuInfo.b1TurnLeft)
					{
						sgActLogic.b1TurnLeft = 1;
					}
					else if(1 ==sgPcuInfo.b1TurnRight)
					{
						sgActLogic.b1TurnRight = 1;
					}
				}
				/*Forward Backward Process*/
				if(0 == sgLimit.b1NoMove)
				{
					if(i8MotorVal<0)
					{
						sgActLogic.b1BackwardAct = 1;
					}
					else if(i8MotorVal>0)
					{
						sgActLogic.b1ForwardAct = 1;
					}
				}
			}
			else if((LIFT_MODE == u8PcuMode)&&(1 == sgPcuInfo.b1Enable))
			{
				if(1 == sgUserInfo.u8LiftReverseFunc)
				{	
					i8MotorVal = -i8MotorVal;
				}
				if((i8MotorVal > 0)&&(0 == sgLimit.b1NoLift))
				{
					sgActLogic.b1LiftUpAct = 1;
				}
				if((i8MotorVal < 0)&&(0 == sgLimit.b1NoDown))
				{
					sgActLogic.b1LiftDownAct = 1;
				}
			}
			u16MotroVal = abs(i8MotorVal)*32;
		}
		PCUStateRecord.u8data = sgPcuInfo.u8data;
	}
	if(0 == i32LocalDiGet(PCU_SWICTH))
	{
		__disable_irq();
		gCanSendPdoInfo.sgHMISendPdo.WorkMode = LOWER_CONTROL_MODE;	
		__enable_irq();

		sgActLogic.u8Data = 0;
		if((1 == i32LocalDiGet(LOWER_CONTROLL_UP))&&(0 == sgLimit.b1NoLift))
		{
			sgActLogic.b1LiftUpAct = 1;
			u16MotroVal = MOTOR_MAX_SPEED_VALUE;
		}
		else if((1 == i32LocalDiGet(LOWER_CONTROLL_DOWN))&&(0 == sgLimit.b1NoDown))
		{
			sgActLogic.b1LiftDownAct = 1;
			u16MotroVal = MOTOR_MAX_SPEED_VALUE;
		}
		else
		{
			u16MotroVal = 0;
		}
		
	}
		
	
	/*antipinch process*/
	if(0 != sgUserInfo.u8AntiPinchFunc)
	{
		switch(u8AntiPinchState)
		{
			case ABOVE_SWI:
				vKillNetTimer(TIMER_EcuAntiPinchFunc);
				if((1 == sgSwiInput.b1DownLimitSwi)&&(1 == sgActLogic.b1LiftDownAct))
				{
					u8AntiPinchState = WAIT_RELEASE;
				}
				break;
			case WAIT_RELEASE:
				sgActLogic.b1LiftDownAct = 0;
				if(0 == sgSwiInput.b1DownLimitSwi)
				{
					u8AntiPinchState = ABOVE_SWI;
				}
				else if(((1 == i32LocalDiGet(PCU_SWICTH))&&(0 == sgPcuInfo.b1Enable))
							||((0 == i32LocalDiGet(PCU_SWICTH))&&(0 == i32LocalDiGet(LOWER_CONTROLL_DOWN))))
				{
					u8AntiPinchState = UNDER_SWI_DELAY;
				}
				break;
			case UNDER_SWI_DELAY:
				if(0 == sgSwiInput.b1DownLimitSwi)
				{
					u8AntiPinchState = ABOVE_SWI;
				}
				else if((1 == sgActLogic.b1LiftDownAct)&&(false == u8GetNetTimerStartFlag(TIMER_EcuAntiPinchFunc)))
				{
					uint16_t u16tmp;
					u16tmp = (sgUserInfo.u8AccAndDecAntiPinch * 100);
					vSetNetTimer(TIMER_EcuAntiPinchFunc, u16tmp);
				}
				else if(true == u8GetNetTimerOverFlag(TIMER_EcuAntiPinchFunc))
				{
					vKillNetTimer(TIMER_EcuAntiPinchFunc);
					u8AntiPinchState = UNDER_SWI_ACT;
				}
				sgActLogic.b1LiftDownAct = 0;
				break;
			case UNDER_SWI_ACT:
				if(0 == sgSwiInput.b1DownLimitSwi)
				{
					u8AntiPinchState = ABOVE_SWI;
				}
				else if(0 == sgActLogic.b1LiftDownAct)
				{
					u8AntiPinchState = UNDER_SWI_DELAY;
				}
				break;
			default:
				u8AntiPinchState = ABOVE_SWI;
				break;
		}
	}
	/*trumpet process*/
	if(1 == sgPcuInfo.b1Speaker)
	{
		i32DoPwmSet(SPEAKER_PUMP,PUMP_OPEN_PERCENTAGE);
	}
	else
	{
		i32DoPwmSet(SPEAKER_PUMP,PUMP_CLOSE_PERCENTAGE);
	}
//	i32SetPara(PARA_TurnLeftValveCurrent, u8AntiPinchState);

}
/*******************************************************************************
* Name: void vPcuSendProc(xPcuSendPara *SendData)
* Descriptio: 发送给PCU的注册函数
* Input: NULL
* Output: NULL
* checked: 龟速灯、起升后限速度亮灯
*******************************************************************************/
/*发送给PCU的回调函数*/
static void vPcuSendProc(xPcuSendPara *SendData)
{
	static uint16_t u8BeepCnt = 0;
	
	if ((MOVE_MODE == u8PcuMode)
		&&((0 != sgLimit.b1Slow)||(0 != sgLimit.b1SpeedAfterLift)))
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
	
	if((1 == sgErrorState.b1Error)&&(1 == sgErrorState.PCUBeep))
	{
		SendData->Data.b1Beep = 1;
	}
	else
	{
		SendData->Data.b1Beep = 0;
	}
	
//	if(1 == sgErrorState.b1Error)
//	{
//		u8BeepCnt++;
//		if(u8BeepCnt < 10)
//		SendData->Data.b1Beep = 1;
//		else
//		{
//			SendData->Data.b1Beep = 0;
//			if(u8BeepCnt > 20)
//				u8BeepCnt = 0;	
//		}
//	}
//	else
//	{
//		SendData->Data.b1Beep = 0;		
//	}
}
/*******************************************************************************
* Name: void vSwiMonitor(void)
* Descriptio: 开关量检测
* Input: NULL
* Output: NULL
* checked: 
* to be check:上限位、下限位，模拟限位、坑洞保护、防倾翻、生效电平
*******************************************************************************/
static void vSwiMonitor(void)
{
	xSwiInput SwiInput = {0};
	static	uint16_t u16TiltDelay = 0;
	
	if(1 == i32LocalDiGet(PCU_SWICTH))
	{
		SwiInput.b1PcuSwi = 1;
	}
	if(1 == i32LocalDiGet(LOWER_CONTROLL_UP))
	{
		SwiInput.b1LowerCtlUp = 1;
	}
	if(1 == i32LocalDiGet(LOWER_CONTROLL_DOWN))
	{
		SwiInput.b1LowerCtlDown = 1;
	}	
	if(0 == i32LocalDiGet(TILT_SIWTCH))
	{
		if(u16TiltDelay < 200)//1s
			u16TiltDelay++;
	}
	else
	{
		u16TiltDelay = 0;
	}
	if(200 == u16TiltDelay)//防止起升后行走起步时触发倾角开关
	{
		SwiInput.b1TiltSwi = 1;
	}
	if(1 == i32LocalDiGet(PIT_SWITCH))
	{
		SwiInput.b1PitSwi = 1;
	}
		
	if(0 != sgUserInfo.u8AngleSimulationLimit)/*angle */
	{
		int32_t i32Angletmp = 0;
		i32Angletmp = i32LocalAiGet(ANGLE_SENSOR_CHANNEL);
		if(i32Angletmp <= sgUserInfo.u8MinAngle)
		{
			SwiInput.b1DownLimitSwi = 1;
		}
		else if(i32Angletmp >= sgUserInfo.u8MaxAngle)
		{
			SwiInput.b1UpLimitSwi = 1;
		}
	}
	else
	{
		if(0 == i32LocalDiGet(UP_LIMIT_SWITCH))
		{
			SwiInput.b1UpLimitSwi = 1;
		}
		if(1 == i32LocalDiGet(DOWN_LIMIT_SWITCH))
		{
			SwiInput.b1DownLimitSwi = 1;
		}			
	}
	
	
	if(0 == SwiInput.b1DownLimitSwi)//下限位之上、倾斜报警
	{
		if(1 == SwiInput.b1TiltSwi)
		{
			i32ErrCodeSet(MACHINE_TILT_OVER_SAFETY_ERR);
		}
		else
		{
			i32ErrCodeClr(MACHINE_TILT_OVER_SAFETY_ERR);
		}
		
		if((0 !=sgUserInfo.u8PitProtectFunc)&&(0 == SwiInput.b1PitSwi))
		{
			i32ErrCodeSet(PIT_PROCETION_ERR);
		}
		else
		{
			i32ErrCodeClr(PIT_PROCETION_ERR);
		}
	}
	else
	{
		i32ErrCodeClr(PIT_PROCETION_ERR);
		i32ErrCodeClr(MACHINE_TILT_OVER_SAFETY_ERR);
	}
	sgLimit.b1SpeedAfterLift = SwiInput.b1PitSwi;
	sgSwiInput.u8data = SwiInput.u8data;
}
/*******************************************************************************
* Name: void vMstRevProc(xMstRevPara *RevData)
* Descriptio: 接收主MCU发送的信息
* Input: NULL
* Output: NULL
* checked: 
* to be check:
*******************************************************************************/
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
		if (u8ErrCodeGet() < RevData->u8ErrCode)   
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
		if (u8ErrCodeGet() < 50)				
		{
			uint8_t i = 0;
			for (i=0; i<50; i++)
			{
				i32ErrCodeClr(i);
			}
		}
	}
	
	u16MotorFdb = (uint16_t)(RevData->u8SpeedFdbHigh << 8) | RevData->u8SpeedFdbLow ;

	__disable_irq();

	tmp = u16MotorFdb / sgUserInfo.u16RatioOfTransmission;
	/*Motor Speed*/
	__enable_irq();
	
	__disable_irq();
	/*Motor Speed*/
	__enable_irq();
	
	tmp = RevData->u8MotorTmp;
	/*Motor Temp*/

	tmp = RevData->u8BoardTmp;
}
/*******************************************************************************
* Name: void vAlarmLampCallBack(uint8_t u8Flag)
* Descriptio: 闪光灯回调函数
* Input: NULL
* Output: NULL
* checked: 
* to be check:
vAlarmLampSetPeriod设置每分钟闪烁次数，u8Flag周期性 0 1 变化
*******************************************************************************/
static void vAlarmLampCallBack(uint8_t u8Flag)
{
//	if(1 == u8Flag)
//	{
//		i32DoPwmSet(BLINK_LED, PUMP_OPEN_PERCENTAGE);
//	}
//	else
//	{
//		i32DoPwmSet(BLINK_LED, PUMP_CLOSE_PERCENTAGE);
//	}
}
/*******************************************************************************
* Name: void vBeepCallBack(uint8_t u8Flag)
* Descriptio: 蜂鸣器回调函数
* Input: NULL
* Output: NULL
* checked: 
* to be check:
*******************************************************************************/
static void vBeepCallBack(uint8_t u8Flag)
{
	if(1 == u8Flag)
	{
		i32DoPwmSet(BLINK_BEEP, PUMP_OPEN_PERCENTAGE);
		sgErrorState.PCUBeep = 1;
	}
	else
	{
		i32DoPwmSet(BLINK_BEEP, PUMP_CLOSE_PERCENTAGE);
		sgErrorState.PCUBeep = 0;
	}
}
/*******************************************************************************
* Name: void vAngleCallBack(uint8_t u8Type)
* Descriptio: 角度传感器回调函数
* Input: NULL
* Output: NULL
* checked: 
* to be check:
超过设置的最大最小角度时，u8Type为1
*******************************************************************************/
static void vAngleCallBack(uint8_t u8Type)
{
	if(0 != sgUserInfo.u8AngleSensorSetting)
	{
		if(1 == u8Type)
		{
			i32ErrCodeSet(ANGLE_SENSOR_ERR);
		}
		else
		{
			i32ErrCodeClr(ANGLE_SENSOR_ERR);
		}
	}
	else
	{
		i32ErrCodeClr(ANGLE_SENSOR_ERR);
	}
}
/*******************************************************************************
* Name: void vPressureCallBack(ePressureNo PressureNo)
* Descriptio: 压力传感器回调函数
* Input: NULL
* Output: NULL
* checked: 
* to be check:
PressureNo：
23.10.19 压力传感器通道在库函数里固定为通道2，暂未修改
23.10.19 压力传感器使能标志，115号参数bit14
使能后74，75参数标定为1，标定状态设置为192，否则显示32
使能称重功能参数49后，未标定则PressureNo为PressureCaliFailure
底层需要增加动态超载时间设置；动态超载范围
*******************************************************************************/
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
//			if (1 == i32LocalDiGet(PIT_SWITCH))	
//			{
				ErrFlag.b1SenSorErr = 1;
//			}
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
/*******************************************************************************
* Name: void vAiMonitor(void)
* Descriptio: 模拟量监控、目前监控电压
* Input: NULL
* Output: NULL
* checked: 
* to be check:电压采集值、可设、电压参数
*******************************************************************************/
static void vAiMonitor(void)
{
	uint16_t u16AdcValue = 0;
	
	u16AdcValue = i32LocalAiGetValue(AI_B_VBUS_CHECK);
	i32SetPara(PARA_BatteryVoltage, u16AdcValue);
	if (u16AdcValue < 18000)		
	{
		i32ErrCodeSet(BAT_LOW_CAP2_ERR);
	}
	else
	{
		i32ErrCodeClr(BAT_LOW_CAP2_ERR);
	}
}
static void vBatteryManage(void)
{
	uint8_t u8Soc;
	u8Soc = u8GetBatterySoc();
	if(u8Soc < 10 )
	{
		i32ErrCodeSet(BAT_LOW_CAP2_ERR);
		i32ErrCodeClr(BATTERY_LOW_CAP1_ERR);
	}
	else if(u8Soc < 20)
	{
		i32ErrCodeSet(BATTERY_LOW_CAP1_ERR);
		i32ErrCodeClr(BAT_LOW_CAP2_ERR);
	}
	else
	{
		i32ErrCodeClr(BAT_LOW_CAP2_ERR);
		i32ErrCodeClr(BATTERY_LOW_CAP1_ERR);		
	}
	i32SetPara(PARA_BmsSoc ,u8Soc*2.5);
	
	__disable_irq();
	gCanSendPdoInfo.sgHMISendPdo.SOC = u8Soc;
	__enable_irq();
}
/*******************************************************************************
* Name: void vEcuSetBeepPeriod(void)
* Descriptio: 设置蜂鸣器的周期
* Input: NULL
* Output: NULL
* checked: 
* to be check:下降动作蜂鸣器
*******************************************************************************/
static void vEcuSetBeepPeriod(void)
{
	static uint8_t u8Cnt = 0;
	
	if(1 == sgErrorState.b1Error)	/*不可恢复报警*/
	{
		vBeepSetPeriod(60);
	}
	else if(1 == sgErrorState.b1Warning )	/*可恢复报警*/
	{
		vBeepSetPeriod(180);
	}
	else if(((ABOVE_SWI != u8AntiPinchState)&&(UNDER_SWI_DELAY != u8AntiPinchState))||(true == u8GetNetTimerStartFlag(TIMER_EcuAntiPinchFunc)))		/*下降动作*/
	{
		u8Cnt++;
		if(u8Cnt >= 180)
		{
			u8Cnt = 0;
		}
		if(u8Cnt < 144)
		{
			vBeepSetPeriod(240);
		}
		else
		{
			vBeepSetPeriod(0);
		}
	}
	else if((0 != sgActLogic.u8Data) && (FunctionEnable == i32GetPara(PARA_ActAlmFunc)))		/*动作报警*/
	{
		vBeepSetPeriod(30);
	}
	else
	{
		vBeepSetPeriod(0);
	}
}

static void vErrProc(void)
{
	uint8_t u8ErrCode = 0;
	
	u8ErrCode = u8ErrCodeGet();
	if(0 != u8ErrCode)
	{
		sgErrorState.b1Error = 1;
		vLedSendAlmCode(u8ErrCode);
		u8ErrCode = u8ErrCodeGetTrans();
		__disable_irq();
		gCanSendPdoInfo.sgHMISendPdo.ECUErr =  ( u8ErrCode | 0x80);
		__enable_irq();
	}
	else
	{
		sgErrorState.b1Error = 0;
	}
	
	if(0 != (u8ErrCodeGetAbnormal(ABNORMAL_NoAct)))
		sgLimit.b1NoAct = 1;
	else
		sgLimit.b1NoAct = 0;
	
		/*Action Limit */
	if(0 != sgLimit.b1NoAct)
	{
		sgLimit.b1NoDown = 1;
		sgLimit.b1NoLift = 1;
		sgLimit.b1NoMove = 1;
		sgLimit.b1NoPcu = 1;
		sgLimit.b1NoTurn = 1;
	}
	else 
	{
		if(0 != (u8ErrCodeGetAbnormal(ABNORMAL_NoPcu)))
			sgLimit.b1NoPcu = 1;
		else
			sgLimit.b1NoPcu = 0;
		
		if((0 != (u8ErrCodeGetAbnormal(ABNORMAL_NoUp)))||(1 == sgSwiInput.b1UpLimitSwi))
			sgLimit.b1NoLift = 1;
		else
			sgLimit.b1NoLift = 0;
		
		if(0 != (u8ErrCodeGetAbnormal(ABNORMAL_NoDown)))
			sgLimit.b1NoDown = 1;
		else
			sgLimit.b1NoDown = 0;	

		if((0 != (u8ErrCodeGetAbnormal(ABNORMAL_NoTurnLeft)))||(0 != (u8ErrCodeGetAbnormal(ABNORMAL_NoTurnRight))))
			sgLimit.b1NoTurn = 1;
		else
			sgLimit.b1NoTurn = 0;

		if((0 != (u8ErrCodeGetAbnormal(ABNORMAL_NoForWard)))||(0 != (u8ErrCodeGetAbnormal(ABNORMAL_NoBackWard))))
			sgLimit.b1NoMove = 1;
		else
			sgLimit.b1NoMove = 0;

		if((0 != (u8ErrCodeGetAbnormal(ABNORMAL_Gear1Spd)))||(1 == sgSwiInput.b1PitSwi))
			sgLimit.b1SpeedAfterLift = 1;
		else
			sgLimit.b1SpeedAfterLift = 0;		
	}
		i32SetPara(PARA_LiftValveCurrent, sgLimit.u8Data);		/*Send Pump Value*/
	/*action limit and beep sets*/
}

static void vSpeedControl1(void)
{
	static float fStepLen = 0;
	fStepLen = 4064 / u16Time;
	if(u16SpeedCmd < u16SpeedTarget)
	{
		if((u16SpeedTarget - u16SpeedCmd)> fStepLen )
		{
			u16SpeedCmd += fStepLen ;
		}
		else//加速完成
		{
			u16SpeedCmd = u16SpeedTarget ;
		}
	}
	else 
	{
		if((u16SpeedCmd - u16SpeedTarget)>fStepLen)
		{
			u16SpeedCmd -= fStepLen ;
		}
		else//加速完成
		{
			u16SpeedCmd = u16SpeedTarget ;
		}
	}
	if(1 == sgUserInfo.u8BrakeBDeadZoneMaxVal)
		i32SetPara(PARA_BackValveCurrent, fStepLen);
	else
		i32SetPara(PARA_BackValveCurrent, u16MotorFdb/20);
	i32SetPara(PARA_ForwardValveCurrent, u16SpeedCmd/20);
}

//s形状参数组,加减速取最快的参数
static void vMoveAccParaSets2(void)
{
	u16Time = sgUserInfo.u8AccAndDecFastDrive ;
	if((1 == sgLimit.b1SpeedAfterLift)&&(u16Time < sgUserInfo.u8AccAndDecAfterLift))
	{
		u16Time = sgUserInfo.u8AccAndDecAfterLift;
	}
	else if((1 == sgLimit.b1Slow)&&(u16Time < sgUserInfo.u8AccAndDecSlowDrive))
	{
		u16Time = sgUserInfo.u8AccAndDecSlowDrive;
	}
	else if((0 != sgActLogic.b1TurnLeft + sgActLogic.b1TurnRight)&&(u16Time < sgUserInfo.u8AccAndDecTurn))
	{
		u16Time = sgUserInfo.u8AccAndDecTurn;
	}
}


static void vMoveStableParaSets2(void)
{
	u16Time = sgUserInfo.u8CurveFastDrive ;
	if((1 == sgLimit.b1SpeedAfterLift)&&(u16Time < sgUserInfo.u8CurveDriveAfterLift))
	{
		u16Time = sgUserInfo.u8CurveDriveAfterLift;
	}
	else if((1 == sgLimit.b1Slow)&&(u16Time < sgUserInfo.u8CurveSlowDrive))
	{
		u16Time = sgUserInfo.u8CurveSlowDrive;
	}
	else if((0 != sgActLogic.b1TurnLeft + sgActLogic.b1TurnRight)&&(u16Time < sgUserInfo.u8CurveTurn))
	{
		u16Time = sgUserInfo.u8CurveTurn;
	}
}

static void vMoveDecParaSets2(void)
{
	u16Time = sgUserInfo.u8BrakeFastDrive ;
	if((1 ==  sgLimit.b1SpeedAfterLift)&&(u16Time < sgUserInfo.u8BrakeDriveAfterLift))
	{
		u16Time = sgUserInfo.u8BrakeDriveAfterLift;
	}
	else if((1 == sgLimit.b1Slow)&&(u16Time < sgUserInfo.u8BrakeSlowDrive))
	{
		u16Time = sgUserInfo.u8BrakeSlowDrive;
	}
	else if((0 != sgActLogic.b1TurnLeft + sgActLogic.b1TurnRight)&&(u16Time < sgUserInfo.u8BrakeTurn))
	{
		u16Time = sgUserInfo.u8BrakeTurn;
	}
}

//输入速度比率处理前的目标速度，指令速度，调速时间，返回一个步长，步长再乘以对应动作的加减速比率限制
static void vSpeedControl2()
{
	static float fStepFactor = 0;//实际执行的加减速
	static float fFactorA = 0;
	static float fFactorB = 0;
	static float fFactorC = 0;
	static float fStepRate = 0;//线性加减速时，加减速率
	static uint16_t u16SpeedTargetRec = 0;
	
	static uint8_t u8AccFlag = 0;
	static uint16_t u16Timecnt = 0;//用于计算步长的时间参数
	static uint16_t u16TimeRange = 0;
	
	static uint8_t u8IncreaseDecreaseFlag = 0;

	#define STABLE_STATE		0
	#define INCREASE_STATE	1
	#define DECREASE_STATE	2
	
	
	
	#define ACC_STATIC	0		//默认值，指令速度稳定或波动比较小，
	#define ACC_FRESH		1  	//加减速参数更新
	#define ACC_EXCUTE	2		//加减速执行
	#define RVS_FRESH		3		//指令反向时参数计算
	#define	RVS_EXCUTE	4		//指令反向时，加速步长归零过程
	
	switch(u8AccFlag)
	{
		case ACC_STATIC://单位时间内指令值最大最小差值超出阈值
			fStepFactor = 0;
			if(u16SpeedTarget != u16SpeedCmd)
				u8AccFlag = ACC_FRESH;
			break;
		case ACC_FRESH://参数更新,系数值ABC，目标速度，u16SpeedTarget - u16SpeedCmd
			fStepRate = 4064 / u16Time;
			fFactorA = (3 * fStepFactor * u16Time - 6 * abs(u16SpeedCmd - u16SpeedTarget ))/(pow(u16Time,3));
			fFactorB = (6 * abs(u16SpeedCmd - u16SpeedTarget ) - 4 * fStepFactor * u16Time)/(u16Time * u16Time);
			fFactorC = fStepFactor;
			u16SpeedTargetRec = u16SpeedTarget;
			u16TimeRange = u16Time;
			u16Timecnt = 0;
			u8AccFlag = ACC_EXCUTE;
			if(u16SpeedTarget > u16SpeedCmd)
			{
				u8IncreaseDecreaseFlag = INCREASE_STATE;
			}
			else
			{
				u8IncreaseDecreaseFlag = DECREASE_STATE;
			}
			break;
		case ACC_EXCUTE://目标速度变化不大则执行，变化大则重新计算参数,速度逼近则回到初始状态
			fStepFactor = fFactorA * u16Timecnt * u16Timecnt + fFactorB * u16Timecnt + fFactorC;
			if(u16Timecnt<u16TimeRange)
				u16Timecnt ++;

			if(fStepFactor < (fStepRate / 100))//最小斜率不超过状态下加速度的1/10
			{
				fStepFactor = fStepRate / 100;
			}					

			if((u16SpeedTargetRec == u16SpeedTarget))//指令变化小于阈值
			{
				if(u16SpeedCmd < u16SpeedTarget)
				{
					#if 0
					if((u16SpeedTarget - u16SpeedCmd)>fStepFactor)
					{
						u16SpeedCmd += fStepFactor ;
					}
					#else
					if(u16Timecnt<u16TimeRange)
					{
						u16SpeedCmd += fStepFactor ;
					}
					#endif
					else//加速完成
					{
						u16SpeedCmd = u16SpeedTarget ;
						u8AccFlag = ACC_STATIC;
					}
				}
				else 
				{
					if((u16SpeedCmd - u16SpeedTarget)>fStepFactor)
					{
						u16SpeedCmd -= fStepFactor ;
					}
					else//加速完成
					{
						u16SpeedCmd = u16SpeedTarget ;
						u8AccFlag = ACC_STATIC;
					}
				}
			}
			#if 0
			else
			{
				u8AccFlag = ACC_FRESH;
			}
			#else
			else if(((u16SpeedCmd > u16SpeedTarget)&&(INCREASE_STATE == u8IncreaseDecreaseFlag ))
				||((u16SpeedCmd < u16SpeedTarget)&&(DECREASE_STATE == u8IncreaseDecreaseFlag )))//加速时指令降低了，或减速时指令增加了
			{
				u8AccFlag = RVS_FRESH;
			}
			else//指令变化大于阈值，重新计算
			{
				u8AccFlag = ACC_FRESH;
			}	
			break;
		case RVS_FRESH:
			fFactorA = fStepFactor / (u16TimeRange );
			fFactorB = fStepFactor;
			u16Timecnt = 0;
			u8AccFlag = RVS_EXCUTE;
		{
			u8AccFlag = ACC_FRESH;
		}
			break;
		case RVS_EXCUTE:
			fStepFactor =  - fFactorA *u16Timecnt + fFactorB ;
			if(((u16SpeedCmd < u16SpeedTarget)&&(INCREASE_STATE == u8IncreaseDecreaseFlag ))
					||((u16SpeedCmd > u16SpeedTarget)&&(DECREASE_STATE == u8IncreaseDecreaseFlag )))//反向过程中恢复前进
			{
				u8AccFlag = ACC_STATIC;
			}
		
			if(u16SpeedTarget > u16SpeedCmd)
			{
				u8IncreaseDecreaseFlag = INCREASE_STATE;
			}
			else
			{
				u8IncreaseDecreaseFlag = DECREASE_STATE;
			}
			if(u16Timecnt < u16TimeRange )
				u16Timecnt ++;
			
			if(fStepFactor < (fStepRate / 10))//加速步长低于阈值，回归默认状态
			{
				fStepFactor = fStepRate / 10;
				u8AccFlag = ACC_STATIC;
			}
			else
			{
				if(INCREASE_STATE == u8IncreaseDecreaseFlag)
				{
					u16SpeedCmd += fStepFactor ;
				}
				else 
				{
					u16SpeedCmd -= fStepFactor ;
				}
			}
			#endif
			break;
		default :
			u8AccFlag = ACC_STATIC;
			break;
	}
	i32SetPara(PARA_BmsSoc,u8AccFlag);
	if(1 == sgUserInfo.u8BrakeBDeadZoneMaxVal)
		i32SetPara(PARA_BackValveCurrent, fStepFactor);
	else
		i32SetPara(PARA_BackValveCurrent, u16MotorFdb/20);
	
	i32SetPara(PARA_ForwardValveCurrent, u16SpeedCmd/20);
	
}
//旧滤波方案

static void vSpeedControl3()
{
	static int32_t u32SpeedSum = 0;
	static uint16_t u16TimeRecord = 0;
	static int16_t i16SpeedOld = 0;
	
//	if(u16TimeRecord > u16Time)
//	{
//		u32SpeedSum = u32SpeedSum>>(u16TimeRecord - u16Time);
//		u16TimeRecord = u16Time;
//	}
//	else
//	{
//		u32SpeedSum = u32SpeedSum<<(u16Time - u16TimeRecord);
		u16TimeRecord = u16Time;
//	}
	
	u32SpeedSum += u16SpeedTarget - u16SpeedCmd;
	u16SpeedCmd = u32SpeedSum >> (u16TimeRecord);
	
	if(1 == sgUserInfo.u8BrakeBDeadZoneMaxVal)
		i32SetPara(PARA_BackValveCurrent, abs(i16SpeedOld - u16SpeedCmd));
	else
		i32SetPara(PARA_BackValveCurrent, u16MotorFdb/20);
	
	i32SetPara(PARA_ForwardValveCurrent, u16SpeedCmd/20);
	
	i16SpeedOld = u16SpeedCmd;
}


static void vSpeedControl4()
{
	
}

static void vMoveAccParaSets()
{
	vMoveAccParaSets2();
	if((1 == sgUserInfo.u8BrakeBMidVal)||(2 == sgUserInfo.u8BrakeBMidVal))
		u16Time = u16Time * TIME_FACTOR;
	else if(3 == sgUserInfo.u8BrakeBMidVal)
		u16Time = u16Time / FILTER_FACTOR;
}
static void vMoveStableParaSets()
{
		vMoveStableParaSets2();
	if((1 == sgUserInfo.u8BrakeBMidVal)||(2 == sgUserInfo.u8BrakeBMidVal))
		u16Time = u16Time * TIME_FACTOR;
	else if(3 == sgUserInfo.u8BrakeBMidVal)
		u16Time = u16Time / FILTER_FACTOR;
}

static void vMoveDecParaSets()
{
	vMoveDecParaSets2();
	if((1 == sgUserInfo.u8BrakeBMidVal)||(2 == sgUserInfo.u8BrakeBMidVal))
		u16Time = u16Time * TIME_FACTOR;
	else if(3 == sgUserInfo.u8BrakeBMidVal)
		u16Time = u16Time / FILTER_FACTOR;
}



static void vSpeedControl()
{
	if(1 == sgUserInfo.u8BrakeBMidVal)
		vSpeedControl1();
	else if(2 == sgUserInfo.u8BrakeBMidVal)
		vSpeedControl2();
	else if(3 == sgUserInfo.u8BrakeBMidVal)
		vSpeedControl3();
}

static void vMstSendProc(xMstSendPara *SendData) 
{
	static xActLogic ActLogicRecord;
	static uint8_t u8SpeedRate = 0;

	SendData->buf[2] = 0;

/*前进、后退 起升开关阀做延时处理*/
 
	static uint8_t u8MotorFlag;			/*Motor Act Advance or Delay*/
	#define MOTOR_STATIC			0		 	/*无动作状态*/
	#define MOTOR_F_START			1 		/*前进起步*/ 
	#define MOTOR_B_START			2 		/*后退起步*/
	#define MOTOR_F_STOP 			3			/*前进刹车*/
	#define MOTOR_B_STOP			4			/*后退刹车*/
	#define MOTOR_RUN_STABLE 	5			/*稳定运行*/
	#define MOTOR_MODE_CHANGE	6			/*前后切换*/

	/*MOTOR FEEDBACK VALUE*/
	#define	MOTOR_FEEDBACK_MAX_SPEED			sgUserInfo.u16MotorMaxSpd
	#define	MOTOR_FEEDBACK_MIN_SPEED 			(MOTOR_FEEDBACK_MAX_SPEED * sgUserInfo.u8Analog3DeadZoneMinVal / 100)
	#define MOTOR_FEEDBAC_SHUTDOWN_SPEED	(MOTOR_FEEDBACK_MAX_SPEED * sgUserInfo.u8Analog3DeadZoneMaxVal / 100)
	#define MOTOR_FB_OPEN_SPEED							(MOTOR_FEEDBACK_MAX_SPEED * sgUserInfo.u8Analog3MidVal / 100)
	#define MOTOR_FEEDBACK_CHANGE_SPEED 	(MOTOR_FEEDBACK_MAX_SPEED * sgUserInfo.u8ThrottleFMidVal / 100)
	#define MOTOR_FEEDBACK_STEER_CLOSE_SPEED (MOTOR_FEEDBACK_MAX_SPEED * sgUserInfo.u8ThrottleFDeadZoneMinVal /100)
	#define MOTOR_FEEDBACK_UP_CLOSE_SPEED		(MOTOR_FEEDBACK_MAX_SPEED *  sgUserInfo.u8ThrottleFDeadZoneMaxVal /100)
	#define MOTOR_FEEDBACK_UP_OPEN_SPEED		(MOTOR_FEEDBACK_MAX_SPEED *  sgUserInfo.u8ThrottleBDeadZoneMinVal /100)
	#define MOTOR_HORIZON_CLOSE_SPEED	(MOTOR_FEEDBACK_MAX_SPEED * sgUserInfo.u8ThrottleBMidVal /100)

	static uint8_t u8PumpFlag;
	#define PUMP_STATIC			0
	#define PUMP_UP					1
	#define PUMP_UP_STOP		2
	#define PUMP_DOWN				3
	#define PUMP_DOWN_STOP	4
	
	static uint16_t u8PumpCloseDelay = 0;
	static uint8_t u8HLPumpCloseDelay = 0;

	if(((MOVE_MODE == u8PcuMode)||(MOTOR_STATIC != u8MotorFlag))
		&&(PUMP_STATIC == u8PumpFlag))
	{
		i32DoPwmSet(LIFTUP_PUMP,PUMP_CLOSE_PERCENTAGE);
		vPropSetTarget(LIFTDOWN_PUMP, 0);
		switch(u8MotorFlag)
		{
			case MOTOR_STATIC://静止状态，等待指令
				if(1 == sgActLogic.b1ForwardAct)
				{
					u8MotorFlag = MOTOR_F_START;
				}
				else if(1 == sgActLogic.b1BackwardAct)
				{
					u8MotorFlag = MOTOR_B_START;
				}
				else
				{
					u8MotorFlag = MOTOR_STATIC;
				}
				vMoveAccParaSets();
				u16SpeedTarget = 0;
				break;
			case MOTOR_F_START://前进起步
				if((1 == sgSwiInput.b1TiltSwi)&&(u16MotorFdb > MOTOR_FEEDBACK_MIN_SPEED))//触发倾角，提前转
				{
					i32DoPwmSet(FORWARD_PUMP,PUMP_OPEN_PERCENTAGE);
					u8MotorFlag = MOTOR_RUN_STABLE;
				}
				else if(0 == sgSwiInput.b1TiltSwi)//平地，直接开阀
				{
					i32DoPwmSet(FORWARD_PUMP,PUMP_OPEN_PERCENTAGE);
					u8MotorFlag = MOTOR_RUN_STABLE;					
				}
				else if(0 == sgActLogic.b1ForwardAct)
				{
					u8MotorFlag = MOTOR_STATIC;
				}
				else
				{
					u8MotorFlag = MOTOR_F_START;
				}
				vMoveAccParaSets();
				u16SpeedTarget = u16MotroVal;
				break;
			case MOTOR_B_START:
				if((1 == sgSwiInput.b1TiltSwi)&&(u16MotorFdb > MOTOR_FEEDBACK_MIN_SPEED))
				{
					i32DoPwmSet(BACKWARD_PUMP,PUMP_OPEN_PERCENTAGE);
					u8MotorFlag = MOTOR_RUN_STABLE;
				}
				else if(0 == sgSwiInput.b1TiltSwi)//平地，直接开阀
				{
					i32DoPwmSet(BACKWARD_PUMP,PUMP_OPEN_PERCENTAGE);
					u8MotorFlag = MOTOR_RUN_STABLE;					
				}
				else if(0 == sgActLogic.b1BackwardAct)
				{
					u8MotorFlag = MOTOR_STATIC;
				}
				else
				{
					u8MotorFlag = MOTOR_B_START;
				}
				vMoveAccParaSets();
				u16SpeedTarget = u16MotroVal;
				break;
			case MOTOR_F_STOP:
				if(((1 == sgActLogic.b1TurnLeft)||(1 == sgActLogic.b1TurnRight))
					&&(u16MotorFdb < MOTOR_FEEDBACK_STEER_CLOSE_SPEED))//if steerlogic exists ,shut down rapid 
				{
					i32DoPwmSet(FORWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
					u8MotorFlag = MOTOR_STATIC;
				}
				else if((1 == sgActLogic.b1BackwardAct)&&(u16MotorFdb < MOTOR_FB_OPEN_SPEED))//direction reverse
				{
					u8MotorFlag = MOTOR_MODE_CHANGE;
				}
				else if(1 == sgActLogic.b1ForwardAct)
				{
					u8MotorFlag = MOTOR_RUN_STABLE;
				}
				else if(0 == sgSwiInput.b1TiltSwi)//平地，延迟关阀
				{
					if(u16MotorFdb <= MOTOR_HORIZON_CLOSE_SPEED)
					{
						u8PumpCloseDelay = 0;
						i32DoPwmSet(FORWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
						u8MotorFlag = MOTOR_STATIC;
					}
//					else
//						u8PumpCloseDelay++;
				}
				else if((1 == sgSwiInput.b1TiltSwi)&&(u16MotorFdb < MOTOR_FEEDBAC_SHUTDOWN_SPEED))//wait motor slow down
				{
					i32DoPwmSet(FORWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
					u8MotorFlag = MOTOR_STATIC;
				}
				else
				{
					u8MotorFlag = MOTOR_F_STOP;
				}
				vMoveDecParaSets();
				u16SpeedTarget = 0;
				break;
			case MOTOR_B_STOP:
				if(((1 == sgActLogic.b1TurnLeft)||(1 == sgActLogic.b1TurnRight))
					&&(u16MotorFdb < MOTOR_FEEDBACK_STEER_CLOSE_SPEED))//if steerlogic exists ,shut down rapid 
				{
					i32DoPwmSet(BACKWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
					u8MotorFlag = MOTOR_STATIC;
				}
				else if((1 == sgActLogic.b1ForwardAct)&&(u16MotorFdb < MOTOR_FB_OPEN_SPEED))//direction reverse
				{
					u8MotorFlag = MOTOR_MODE_CHANGE;
				}
				else if(1 == sgActLogic.b1BackwardAct)//
				{
					u8MotorFlag = MOTOR_RUN_STABLE;
				}
				else if(0 == sgSwiInput.b1TiltSwi)
				{
					if(u16MotorFdb <= MOTOR_HORIZON_CLOSE_SPEED)
					{
						u8PumpCloseDelay = 0;
						i32DoPwmSet(BACKWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
						u8MotorFlag = MOTOR_STATIC;
					}
//					else
//						u8PumpCloseDelay++;
				}
				else if((1 == sgSwiInput.b1TiltSwi)&&(u16MotorFdb < MOTOR_FEEDBAC_SHUTDOWN_SPEED))//wait motor slow down
				{
					i32DoPwmSet(BACKWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
					u8MotorFlag = MOTOR_STATIC;
				}
				else
				{
					u8MotorFlag = MOTOR_B_STOP;
				}
				vMoveDecParaSets();
				u16SpeedTarget = 0;
				break;
			case MOTOR_RUN_STABLE:
				if((1 == ActLogicRecord.b1ForwardAct)&&(0 == sgActLogic.b1ForwardAct))
				{
					u8MotorFlag = MOTOR_F_STOP;
				}
				else if((1 == ActLogicRecord.b1BackwardAct)&&(0 == sgActLogic.b1BackwardAct))
				{
					u8MotorFlag = MOTOR_B_STOP;
				}
				else 
				{
					u8MotorFlag = MOTOR_RUN_STABLE;
				}
				vMoveStableParaSets();
				u16SpeedTarget = u16MotroVal;
				break;
			case MOTOR_MODE_CHANGE:
				if(u16MotorFdb < MOTOR_FEEDBACK_CHANGE_SPEED)
				{
					if(1 == sgActLogic.b1BackwardAct)
					{
						i32DoPwmSet(BACKWARD_PUMP,PUMP_OPEN_PERCENTAGE);
						i32DoPwmSet(FORWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
						u8MotorFlag = MOTOR_RUN_STABLE;
					}
					else if(1 == sgActLogic.b1ForwardAct)
					{
						i32DoPwmSet(BACKWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
						i32DoPwmSet(FORWARD_PUMP,PUMP_OPEN_PERCENTAGE);
						u8MotorFlag = MOTOR_RUN_STABLE;
					}
					else
					{
						i32DoPwmSet(BACKWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
						i32DoPwmSet(FORWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
						u8MotorFlag = MOTOR_STATIC;
					}
				}
				else
				{
					i32DoPwmSet(BACKWARD_PUMP,PUMP_OPEN_PERCENTAGE);
					i32DoPwmSet(FORWARD_PUMP,PUMP_OPEN_PERCENTAGE);				
				}
				vMoveDecParaSets();
				u16SpeedTarget = 0;
				break;
			default:
				i32DoPwmSet(BACKWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
				i32DoPwmSet(FORWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
				vMoveDecParaSets();
				u16SpeedTarget = 0;
				u8MotorFlag = MOTOR_STATIC;
				break;
		}	
		/*Speed Rate Limit*/

		u8SpeedRate = sgUserInfo.u8FastDriveSpeed;//默认
		if(1 == sgLimit.b1SpeedAfterLift)
		{
			if(u8SpeedRate > sgUserInfo.u8DriveSpeedAfterLift)
			u8SpeedRate = sgUserInfo.u8DriveSpeedAfterLift;
		}
		if(1 == sgLimit.b1Slow)
		{
			if(u8SpeedRate > sgUserInfo.u8SlowDriveSpeed)
			u8SpeedRate = sgUserInfo.u8SlowDriveSpeed;
			
		}		
		
		//高低速阀	
		if(((1 == sgLimit.b1Slow)||(1 == sgLimit.b1SpeedAfterLift))&&(MOTOR_STATIC != u8MotorFlag))
		{
			u8HLPumpCloseDelay = 0;
			i32DoPwmSet(SLOW_SPEED_PUMP,PUMP_OPEN_PERCENTAGE);
		}
		else
		{	
			if(u8HLPumpCloseDelay>200)
				i32DoPwmSet(SLOW_SPEED_PUMP,PUMP_CLOSE_PERCENTAGE);
			else
				u8HLPumpCloseDelay++;
		}

		if((1 == sgActLogic.b1TurnLeft)||(1 == sgActLogic.b1TurnRight))
		{
			if(u8SpeedRate > sgUserInfo.u8MaxTurnSpeed)
				u8SpeedRate = sgUserInfo.u8MaxTurnSpeed;		
		}
		
		u16SpeedTarget = u16SpeedTarget * u8SpeedRate / 100;
		
			/*steer process*/
		if(1 ==sgActLogic.b1TurnLeft)
		{
			i32DoPwmSet(TURNLEFT_PUMP,PUMP_OPEN_PERCENTAGE);
			if(u16SpeedTarget < (sgUserInfo.u8TurnPowerLimit * 4095 / 100))
				u16SpeedTarget = sgUserInfo.u8TurnPowerLimit * 4095 / 100;
		}
		else
		{
			i32DoPwmSet(TURNLEFT_PUMP,PUMP_CLOSE_PERCENTAGE);
		}
		
		if(1 == sgActLogic.b1TurnRight)
		{
			i32DoPwmSet(TURNRIGHT_PUMP,PUMP_OPEN_PERCENTAGE);
			if(u16SpeedTarget < (sgUserInfo.u8TurnPowerLimit * 4095 / 100))
				u16SpeedTarget = sgUserInfo.u8TurnPowerLimit * 4095 / 100;
		}
		else
		{
			i32DoPwmSet(TURNRIGHT_PUMP,PUMP_CLOSE_PERCENTAGE);
		}

		vSpeedControl();
	}
	else if((((LIFT_MODE == u8PcuMode)||(PUMP_STATIC != u8PumpFlag))&&(MOTOR_STATIC == u8MotorFlag))
		||((0 == i32LocalDiGet(PCU_SWICTH))))
	{
		i32DoPwmSet(BACKWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
		i32DoPwmSet(FORWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
		i32DoPwmSet(TURNLEFT_PUMP,PUMP_CLOSE_PERCENTAGE);
		i32DoPwmSet(TURNRIGHT_PUMP,PUMP_CLOSE_PERCENTAGE);
		/*lift process*/
		switch(u8PumpFlag)
		{
			case PUMP_STATIC://静止，等待动作指令
				if(1 == sgActLogic.b1LiftUpAct)
				{
					u8PumpFlag = PUMP_UP;
				}
				else if(1 == sgActLogic.b1LiftDownAct)
				{
					u8PumpFlag = PUMP_DOWN;
				}
				break;
			case PUMP_UP://稳定执行，指令改变时切换
				u16SpeedTarget = u16MotroVal;
				u8SpeedRate = sgUserInfo.u8LiftSpeed;
				if((0 == sgActLogic.b1LiftUpAct)||(1 == sgActLogic.b1LiftDownAct))
				{
					u8PumpFlag = PUMP_UP_STOP;
				}
				else if (u16MotorFdb < MOTOR_FEEDBACK_UP_OPEN_SPEED)//转速小于阈值，先转
				{
					u16Time = sgUserInfo.u8AccAndDecLift;
				}
				else//转速大于阈值，稳定
				{
					i32DoPwmSet(LIFTUP_PUMP,PUMP_OPEN_PERCENTAGE);
					u16Time = sgUserInfo.u8CurveLift;
				}
				break;
			case PUMP_UP_STOP:
				u16SpeedTarget = 0;
				u16Time = sgUserInfo.u8BrakeLift;
				u8SpeedRate = sgUserInfo.u8LiftSpeed;				
				if(u16MotorFdb < MOTOR_FEEDBACK_UP_CLOSE_SPEED)
				{
					i32DoPwmSet(LIFTUP_PUMP,PUMP_CLOSE_PERCENTAGE);
					u8PumpFlag = PUMP_STATIC;
				}
				else if(1 == sgActLogic.b1LiftUpAct)
				{
					u8PumpFlag = PUMP_UP;
				}

				break;
			case PUMP_DOWN:
				u16SpeedTarget = u16MotroVal;
				u16Time = sgUserInfo.u8AccAndDecLower;
				u8SpeedRate = sgUserInfo.u8LowerSpeed;
				if((1 == sgActLogic.b1LiftUpAct)||(0 == sgActLogic.b1LiftDownAct))
				{
					u8PumpFlag = PUMP_DOWN_STOP;
				}
				break;
			case PUMP_DOWN_STOP:
				u16SpeedTarget = 0;
				u16Time = sgUserInfo.u8BrakeLower;
				u8SpeedRate = sgUserInfo.u8LowerSpeed;			
				if((inserted_data[1] * PROP_CURRENT_FACOTR) < (sgUserInfo.fPropMinCurrent1 * 1.5))
				{
					u8PumpFlag = PUMP_STATIC;
				}
				else if(1 == sgActLogic.b1LiftDownAct)
				{
					u8PumpFlag = PUMP_DOWN;
				}
				break;
			default:
				u16SpeedTarget = 0;
				u16SpeedCmd = 0;
				vPropSetTarget(LIFTDOWN_PUMP, 0);
				i32DoPwmSet(LIFTUP_PUMP,PUMP_CLOSE_PERCENTAGE);
				u8PumpFlag = PUMP_STATIC;
				break;
		}
		u16Time = u16Time * TIME_FACTOR;
		u16SpeedTarget = u16SpeedTarget * u8SpeedRate / 100;
		vSpeedControl();
		
		//比例阀动作在速度指令计算之后
		if((PUMP_DOWN == u8PumpFlag)||(PUMP_DOWN_STOP == u8PumpFlag))
		{
			int32_t i32prop = 0;
			i32prop = _IQ((sgUserInfo.fPropMinCurrent1 + (sgUserInfo.fPropMaxCurrent1 - sgUserInfo.fPropMinCurrent1) * u16SpeedCmd / MOTOR_MAX_SPEED_VALUE) / PROPD_STD_CURRENT);
			vPropSetTarget(LIFTDOWN_PUMP, i32prop);
		}
		else
		{
			vPropSetTarget(LIFTDOWN_PUMP, 0);
		}
	}
	else
	{
		u16SpeedTarget = 0;
		i32DoPwmSet(BACKWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
		i32DoPwmSet(FORWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
		i32DoPwmSet(TURNLEFT_PUMP,PUMP_CLOSE_PERCENTAGE);
		i32DoPwmSet(TURNRIGHT_PUMP,PUMP_CLOSE_PERCENTAGE);
		i32DoPwmSet(LIFTUP_PUMP,PUMP_CLOSE_PERCENTAGE);
		vPropSetTarget(LIFTDOWN_PUMP, 0);
	}		

	if((u8PumpFlag != PUMP_DOWN_STOP)&&(u8PumpFlag != PUMP_DOWN)
		&&((u8PumpFlag != PUMP_STATIC)||
		((u8MotorFlag != MOTOR_STATIC)||(0 != sgActLogic.b1TurnLeft)||(0 != sgActLogic.b1TurnRight))))//非下降状态时，发送指令
	{
		SendData->u8TargetHigh = u16SpeedCmd >> 8;
		SendData->u8TargetLow = u16SpeedCmd;
		
		if (0 != u16SpeedCmd)
		{
			SendData->b1ServoOn = 1;
			SendData->b1ForwardReq = 1;
		}		
	}
	else
	{
		SendData->u8TargetHigh = 0;
		SendData->u8TargetLow = 0;
	}
	
	ActLogicRecord.u8Data = sgActLogic.u8Data;

//	i32SetPara(PARA_ForwardValveCurrent, u16SpeedCmd/20);		/*Send Motor Value*/
	
//	i32SetPara(PARA_LiftValveCurrent, u16SpeedTarget);		/*Send Pump Value*/

	i32SetPara(PARA_OnOffValveCurrent, u16SpeedTarget);
	i32SetPara(PARA_TurnLeftValveCurrent, u8PumpFlag);
	i32SetPara(PARA_TurnRightValveCurrent,u8MotorFlag);
	i32SetPara(PARA_BrakeValveCurrent,u8PumpCloseDelay);
	
}

//压力标定
static void vPressureCalibration(void)
{
	static uint8_t u8CalibrationFlag = 0;
	static int8_t i8ActionAccumulate = 0;
	static uint16_t u16TimLimitCnt = 0;
	static uint8_t i = 0;
	static xSwiInput SwitchRecord;
	#define UP_KEY	1
	#define LOW_KEY	2
	uint16_t u16PressureValue = 0;
	
	if((1 == sgSwiInput.b1PcuSwi)//上控模式下操作下控
		&&((1 == sgSwiInput.b1LowerCtlDown)||(1 == sgSwiInput.b1LowerCtlUp)))
	{
		u8CalibrationFlag = 1;
		i32ErrCodeSet(CALISTATE_SAFEPROTECT_ERR);
	}
	
	if(1 == u8CalibrationFlag)//标定标志位
	{
		if((0 == sgSwiInput.b1LowerCtlUp)&&(1 == SwitchRecord.b1LowerCtlUp))//历史记录动作为上升
		{
			u16TimLimitCnt = 0;
			i8ActionAccumulate ++;
		}
		else if((0 == sgSwiInput.b1LowerCtlDown)&&(1 == SwitchRecord.b1LowerCtlDown))//历史记录动作下降
		{
			u16TimLimitCnt = 0;
			i8ActionAccumulate --;
		}
		else if(u16TimLimitCnt < 1000)//等待5秒操作
		{
			u16TimLimitCnt++;
		}
		else//等待超时，清空历史记录动作
		{
			i8ActionAccumulate = 0;
		}
		if(i8ActionAccumulate > 5)//标定满载
		{
			i8ActionAccumulate = 0;
			u16PressureValue = i32LocalAiGet(PRESSURE_SENSOR_CHANNEL);
			u16SaveParaToEeprom(PARA_IsOverLoadCalibration,1);
			u16SaveParaToEeprom(PARA_FullPressure,u16PressureValue);
		}
		else if(i8ActionAccumulate < (-5))
		{
			i8ActionAccumulate = 0;
			u16PressureValue = i32LocalAiGet(PRESSURE_SENSOR_CHANNEL);
			u16SaveParaToEeprom(PARA_IsNoLoadCalibration,1);
			u16SaveParaToEeprom(PARA_EmptyPressure,u16PressureValue);
		}
		SwitchRecord = sgSwiInput;
	}	
//	i32SetPara(PARA_BrakeValveCurrent,i8ActionAccumulate);
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
	vSetPdoPara(sgPdoPara);

	vPcuErrRegister(vPcuErrProc);	
	vPcuRevRegister(vPcuRevProc);
	vPcuSendRegister(vPcuSendProc);
	
	vMstRevRegister(vMstRevProc);
	vMstSendRegister(vMstSendProc);
	vBeepRegister(vBeepCallBack);
	
	vAiErrReg(vAiErrCallBack);
	vDoPwmErrReg(vDoPwmErrCallBack);
	vPropErrReg(vPropErrCallBack);
	
		
	vAngleSensorReg(vAngleCallBack);
	vPressureSensorReg(vPressureCallBack);
	
	vAlarmLampRegister(vAlarmLampCallBack);
	vAlarmLampSetPeriod(30);
	
	vErrCodeInit(sgErrCodeInfo,202);
	
//	vUserParaInit();
	


	memset(&sgActLogic, 0, sizeof(sgActLogic));
	
	//参数获取
	
	/*speed rate*/
	sgUserInfo.u8FastDriveSpeed = i32GetPara(PARA_FastDriveSpeed);
	sgUserInfo.u8SlowDriveSpeed = i32GetPara(PARA_SlowDriveSpeed);
	sgUserInfo.u8DriveSpeedAfterLift = i32GetPara(PARA_DriveSpeedAfterLift);
	sgUserInfo.u8LiftSpeed =  i32GetPara(PARA_LiftSpeed);
	sgUserInfo.u8MaxTurnSpeed = i32GetPara(PARA_MaxTurnSpeed);
	sgUserInfo.u8TurnPowerLimit = i32GetPara(PARA_TurnPowerLimit) ;
	sgUserInfo.u8LowerSpeed = i32GetPara(PARA_LowerSpeed);
	
	sgUserInfo.u16MotorMaxSpd = i32GetPara(PARA_MotorMaxSpd);
	
	/*deadzone*/
	sgUserInfo.u8DeadZoneAdjust = i32GetPara(PAPA_DeadZoneAdjust);
	
	/*accelerate & decelerate*/
	{
		sgUserInfo.u8AccAndDecAfterLift = i32GetPara(PARA_AccAndDecAfterLift) ;
		sgUserInfo.u8AccAndDecFastDrive = i32GetPara(PARA_AccAndDecFastDrive);
		sgUserInfo.u8AccAndDecLift = i32GetPara(PARA_AccAndDecLift);
		sgUserInfo.u8AccAndDecLower = i32GetPara(PARA_AccAndDecLower);
		sgUserInfo.u8AccAndDecSlowDrive = i32GetPara(PARA_AccAndDecSlowDrive);
		sgUserInfo.u8AccAndDecTurn = i32GetPara(PARA_AccAndDecTurn);		

		sgUserInfo.u8CurveDriveAfterLift = i32GetPara(PARA_CurveDriveAfterLift);
		sgUserInfo.u8CurveFastDrive = i32GetPara(PARA_CurveFastDrive);
		sgUserInfo.u8CurveLift = i32GetPara(PARA_CurveLift);
		sgUserInfo.u8CurveLower = i32GetPara(PARA_CurveLower);
		sgUserInfo.u8CurveSlowDrive = i32GetPara(PARA_CurveSlowDrive);
		sgUserInfo.u8CurveTurn = i32GetPara(PARA_CurveTurn);
		
		sgUserInfo.u8BrakeDriveAfterLift = i32GetPara(PARA_BrakeDriveAfterLift);
		sgUserInfo.u8BrakeFastDrive = i32GetPara(PARA_BrakeFastDrive);
		sgUserInfo.u8BrakeLift = i32GetPara(PARA_BrakeLift);
		sgUserInfo.u8BrakeLower = i32GetPara(PARA_BrakeLower);
		sgUserInfo.u8BrakeSlowDrive = i32GetPara(PARA_BrakeSlowDrive);
		sgUserInfo.u8BrakeTurn = i32GetPara(PARA_BrakeTurn);
		
		sgUserInfo.u8BrakeAntiPinch = i32GetPara(PARA_BrakeAntiPinch);

		if (0 != i32GetPara(PARA_AccAndDecFastDrive))
		{
			sgUserInfo.fFastMoveSpdPer5msAccStep = STEP_FACTOR * i32GetPara(PARA_CurveFastDrive) / i32GetPara(PARA_AccAndDecFastDrive);
			sgUserInfo.fFastMoveSpdPer5msDecStep = STEP_FACTOR * i32GetPara(PARA_BrakeFastDrive) / i32GetPara(PARA_AccAndDecFastDrive);
		}
		else
		{
			sgUserInfo.fFastMoveSpdPer5msDecStep = 0;
			sgUserInfo.fFastMoveSpdPer5msAccStep = 0;
		}
		
		if (0 != i32GetPara(PARA_AccAndDecSlowDrive))
		{
			sgUserInfo.fSlowMoveSpdPer5msAccStep = STEP_FACTOR * i32GetPara(PARA_CurveSlowDrive) / i32GetPara(PARA_AccAndDecSlowDrive);
			sgUserInfo.fSlowMoveSpdPer5msDecStep = STEP_FACTOR * i32GetPara(PARA_BrakeSlowDrive) / i32GetPara(PARA_AccAndDecSlowDrive);
		}
		else
		{
			sgUserInfo.fSlowMoveSpdPer5msAccStep = 0;
			sgUserInfo.fSlowMoveSpdPer5msDecStep = 0;
		}
			
		if (0 != i32GetPara(PARA_AccAndDecAfterLift))
		{
			sgUserInfo.fMoveAfterLiftSpdPer5msAccStep = STEP_FACTOR * i32GetPara(PARA_CurveDriveAfterLift) / i32GetPara(PARA_BrakeDriveAfterLift);
			sgUserInfo.fMoveAfterLiftSpdPer5msDecStep = STEP_FACTOR * i32GetPara(PARA_BrakeDriveAfterLift) / i32GetPara(PARA_BrakeDriveAfterLift);
		}
		else
		{
			sgUserInfo.fMoveAfterLiftSpdPer5msAccStep = 0;
			sgUserInfo.fMoveAfterLiftSpdPer5msDecStep = 0;
		}
		
		if (0 != i32GetPara(PARA_AccAndDecLift))
		{
			sgUserInfo.fLiftSpdPer5msAccStep = STEP_FACTOR * i32GetPara(PARA_CurveLift) / i32GetPara(PARA_AccAndDecLift);
			sgUserInfo.fLiftSpdPer5msDecStep = STEP_FACTOR * i32GetPara(PARA_BrakeLift) / i32GetPara(PARA_AccAndDecLift);
		}
		else
		{
			sgUserInfo.fSlowMoveSpdPer5msAccStep = 0;
			sgUserInfo.fSlowMoveSpdPer5msDecStep = 0;
		}

		if (0 != i32GetPara(PARA_AccAndDecTurn))
		{
			sgUserInfo.fTurnSpdPer5msAccStep = STEP_FACTOR * i32GetPara(PARA_CurveTurn) / i32GetPara(PARA_AccAndDecTurn);
			sgUserInfo.fTurnSpdPer5msDecStep = STEP_FACTOR * i32GetPara(PARA_BrakeTurn) / i32GetPara(PARA_AccAndDecTurn);
		}
		else
		{
			sgUserInfo.fTurnSpdPer5msAccStep = 0;
			sgUserInfo.fTurnSpdPer5msDecStep = 0;
		}
			
		sgUserInfo.u8BrakeAntiPinch = i32GetPara(PARA_BrakeAntiPinch);
		sgUserInfo.u8AccAndDecAntiPinch = i32GetPara(PARA_AccAndDecAntiPinch);
		
		sgUserInfo.u8MotorHighSpeedDeceRate = i32GetPara(PARA_MotorHighSpeedDeceRate);
		sgUserInfo.u8MotorLowSpeedDeceRate = i32GetPara(PARA_MotorLowSpeedDeceRate);	
	}
	
	/*pressure sets*/
	sgUserInfo.u8PressureType = i32GetPara(PARA_PressureType);
	sgUserInfo.u8PressureSensorType = i32GetPara(PARA_PressureSensorType);
	sgUserInfo.u8OverLoadStabilityDelay = i32GetPara(PARA_OverLoadStabilityDelay);
	sgUserInfo.u8DynamicOverLoadPercent = i32GetPara(PARA_DynamicOverLoadPercent);
	sgUserInfo.u8StaticOverLoadPercent = i32GetPara(PARA_StaticOverLoadPercent);
	sgUserInfo.u8IsNoLoadCalibration = i32GetPara(PARA_IsNoLoadCalibration);
	sgUserInfo.u8IsOverLoadCalibration = i32GetPara(PARA_IsOverLoadCalibration);
	sgUserInfo.u8EmptyPressure = i32GetPara(PARA_EmptyPressure);
	sgUserInfo.u8FullPressure = i32GetPara(PARA_FullPressure);
		
	/*function option*/
	sgUserInfo.u8PitProtectFunc = i32GetPara(PARA_PitProtectFunc);
	sgUserInfo.u8AntiPinchFunc = i32GetPara(PARA_AntiPinchFunc);
	sgUserInfo.u8ActAlmFunc = i32GetPara(PARA_ActAlmFunc);
	sgUserInfo.u8WeighFunc = i32GetPara(PARA_WeighFunc);
	sgUserInfo.u8ParallelValveReverseFunc = i32GetPara(PARA_ParallelValveReverseFunc);
	sgUserInfo.u8UpperCtlButSleep = i32GetPara(PARA_UpperCtlButSleep);
	sgUserInfo.u8LiftReverseFunc = i32GetPara(PARA_LiftReverseFunc);
	sgUserInfo.u8FourPointWeightFunc = i32GetPara(PARA_FourPointWeightFunc);
	sgUserInfo.u8TiltSwitchSetting = i32GetPara(PARA_TiltSwitchSetting);
	sgUserInfo.u8HeartBeatQueryFunc = i32GetPara(PARA_HeartBeatQueryFunc);
	sgUserInfo.u8AnticollisionFunc = i32GetPara(PARA_AnticollisionFunc);
	
	/*battery sets*/
	sgUserInfo.u8LowBatAlmFunc = i32GetPara(PARA_LowBatAlmFunc);
	sgUserInfo.u8LowVolAlmTime = i32GetPara(PARA_LowVolAlmTime);
	sgUserInfo.u8LowVolShutDownTime = i32GetPara(PARA_LowVolShutDownTime);	
	sgUserInfo.u8BatteryType = i32GetPara(PARA_BatteryType);
	sgUserInfo.u8LowBatteryMode = i32GetPara(PARA_LowBatteryMode);
	sgUserInfo.u8BatSocPalyBack = i32GetPara(PARA_BatSocPalyBack);

	/*angle sensor sets*/
	sgUserInfo.u8AngleSimulationLimit = i32GetPara(PARA_AngleSimulationLimit);
	sgUserInfo.u8AngleSensorSetting = i32GetPara(PARA_AngleSensorSetting);
	sgUserInfo.u8AngleSensorType = i32GetPara(PARA_AngleSensorType);
	sgUserInfo.u8AnaLogLimitDetSwitch = i32GetPara(PARA_AnaLogLimitDetSwitch);
	sgUserInfo.u8AngleSimulationUpLimit = i32GetPara(PARA_AngleSimulationUpLimit);
	sgUserInfo.u8AngleSimulationDownLimit = i32GetPara(PARA_AngleSimulationDownLimit);
	sgUserInfo.u8MinAngle = i32GetPara(PARA_MinAngle);
	sgUserInfo.u8MaxAngle = i32GetPara(PARA_MaxAngle);
	sgUserInfo.u8AngleValue0 = i32GetPara(PARA_AngleValue0);
	sgUserInfo.u8AngleValue1 = i32GetPara(PARA_AngleValue1);
	sgUserInfo.u8AngleValue2 = i32GetPara(PARA_AngleValue2);
	sgUserInfo.u8AngleValue3 = i32GetPara(PARA_AngleValue3);
	sgUserInfo.u8AngleValue4 = i32GetPara(PARA_AngleValue4);
	sgUserInfo.u8AngleValue5 = i32GetPara(PARA_AngleValue5);
	sgUserInfo.u8AngleValue6 = i32GetPara(PARA_AngleValue6);
	sgUserInfo.u8AngleValue7 = i32GetPara(PARA_AngleValue7);
	
	
	/*outdoor sets*/
	sgUserInfo.u8InAndOutFunc = i32GetPara(PARA_InAndOutFunc);
	sgUserInfo.u8SetOutHeight = i32GetPara(PARA_SetOutHeight);
	
	/*prop valve */
	{
		sgUserInfo.fPropMaxCurrent0 = i32GetPara(PARA_PropDMaxCurrent0) / 1000.0;
		sgUserInfo.fPropMinCurrent0 = i32GetPara(PARA_PropDMinCurrent0) / 1000.0;
		sgUserInfo.fPropMaxCurrent1 = i32GetPara(PARA_PropDMaxCurrent1) / 1000.0;
		sgUserInfo.fPropMinCurrent1 = i32GetPara(PARA_PropDMinCurrent1) / 1000.0;		
	}
	
	/*analog3*/
	sgUserInfo.u8Analog3DeadZoneMinVal = i32GetPara(PARA_Analog3DeadZoneMinVal);/*for start pumpspeed*/
	sgUserInfo.u8Analog3DeadZoneMaxVal = i32GetPara(PARA_Analog3DeadZoneMaxVal);/*for stop pump speed*/
	sgUserInfo.u8Analog3MidVal = i32GetPara(PARA_Analog3MidVal);/*for steer pump speed*/
	
	/*throttle forward*/
	sgUserInfo.u8ThrottleType = i32GetPara(PARA_ThrottleType);
	sgUserInfo.u8ThrottleFDeadZoneMinVal = i32GetPara(PARA_ThrottleFDeadZoneMinVal);
	sgUserInfo.u8ThrottleFDeadZoneMaxVal = i32GetPara(PARA_ThrottleFDeadZoneMaxVal);
	sgUserInfo.u8ThrottleFMidVal = i32GetPara(PARA_ThrottleFMidVal);
	
	/*throttle backward*/
	sgUserInfo.u8ThrottleBDeadZoneMinVal = i32GetPara(PARA_ThrottleBDeadZoneMinVal);
	sgUserInfo.u8ThrottleBDeadZoneMaxVal = i32GetPara(PARA_ThrottleBDeadZoneMaxVal);
	sgUserInfo.u8ThrottleBMidVal = i32GetPara(PARA_ThrottleBMidVal);
	
	/*brake forward*/
	sgUserInfo.u8BrakeType = i32GetPara(PARA_BrakeType);
	sgUserInfo.u8BrakeFDeadZoneMinVal = i32GetPara(PARA_BrakeFDeadZoneMinVal);
	sgUserInfo.u8BrakeFDeadZoneMaxVal = i32GetPara(PARA_BrakeFDeadZoneMaxVal);
	sgUserInfo.u8BrakeFMidVal = i32GetPara(PARA_BrakeFMidVal);
	
	/*brake backward*/
	sgUserInfo.u8BrakeBDeadZoneMinVal = i32GetPara(PARA_BrakeBDeadZoneMinVal);
	sgUserInfo.u8BrakeBDeadZoneMaxVal = i32GetPara(PARA_BrakeBDeadZoneMaxVal);
	sgUserInfo.u8BrakeBMidVal = i32GetPara(PARA_BrakeBMidVal);

	/*move speed rate*/
	sgUserInfo.u8Gear1Spd = i32GetPara(PARA_Gear1Spd);
	sgUserInfo.u8Gear2Spd = i32GetPara(PARA_Gear2Spd);
	sgUserInfo.u8Gear3Spd = i32GetPara(PARA_Gear3Spd);
	sgUserInfo.u8Gear4Spd = i32GetPara(PARA_Gear4Spd);
	
	/*display parameter*/
	sgUserInfo.u16RatioOfTransmission = i32GetPara(PARA_RatioOfTransmission);
	sgUserInfo.u16CanBaudRate = i32GetPara(PARA_CanBaudRate);
	sgUserInfo.u16CanOpenNodeId = i32GetPara(PARA_CanOpenNodeId);
	
	/*pump speed rate */
	sgUserInfo.u8PumpMotorGear1 = i32GetPara(PARA_PumpMotorGear1);
	sgUserInfo.u8PumpMotorGear2 = i32GetPara(PARA_PumpMotorGear2);
	
	/*steer control*/
	sgUserInfo.u8TurnWithDecStartAngle = i32GetPara(PARA_TurnWithDecStartAngle);
	sgUserInfo.u8TurnWithDecEndAngle = i32GetPara(PARA_TurnWithDecEndAngle);
	sgUserInfo.u8AngleWithStartSpdPer = i32GetPara(PARA_AngleWithStartSpdPer);
	sgUserInfo.u8AngleWithEndSpdPer = i32GetPara(PARA_AngleWithEndSpdPer);
	sgUserInfo.u8HourCountPowerOn = i32GetPara(PARA_HourCountPowerOn);	
	
	
	/*not operatable for user*/
	sgUserInfo.u8DriverFlag = i32GetPara(PARA_DriverFlag);
	sgUserInfo.u8ValveType = i32GetPara(PARA_ValveType);
	sgUserInfo.u8ValueOpenLoopCurrent = i32GetPara(PARA_ValueOpenLoopCurrent);
	sgUserInfo.u8ValueOpenPercentage = i32GetPara(PARA_ValueOpenPercentage);
	sgUserInfo.u8LogLevel = i32GetPara(PARA_LogLevel);
	sgUserInfo.u8LogModel = i32GetPara(PARA_LogModel);
	sgUserInfo.u8MotorMaxSpd = i32GetPara(PARA_MotorMaxSpd);/*parameter of motor speed*/
	
	/*vacant*/
	sgUserInfo.u8MaxDifferencePercent = i32GetPara(PARA_MaxDifferencePercent);
	sgUserInfo.u8DriveMotorEncoder = i32GetPara(PARA_DriveMotorEncoder);
	sgUserInfo.u8VoiceAlarmVolume = i32GetPara(PARA_VoiceAlarmVolume);
	sgUserInfo.u8PumpMotorEncoder = i32GetPara(PARA_PumpMotorEncoder);
	sgUserInfo.u8VehicleType = i32GetPara(PARA_VehicleType);
	sgUserInfo.u8VehcileHeight = i32GetPara(PARA_VehcileHeight);
	sgUserInfo.u8LanguageType = i32GetPara(PARA_LanguageType);
	sgUserInfo.u8SpeakerSync = i32GetPara(PARA_SpeakerSync);
	sgUserInfo.u8LowerPumpType = i32GetPara(PARA_LowerPumpType);
	sgUserInfo.u8PasswordLock = i32GetPara(PARA_PasswordLock);
	sgUserInfo.u16SetDescentHeightValue = i32GetPara(PARA_SetDescentHeightValue);
	sgUserInfo.u8ReleaseBrake = i32GetPara(PARA_ReleaseBrake);
	sgUserInfo.u8DriverType = i32GetPara(PARA_DriverType);
	sgUserInfo.u8MaintenancePeriod = i32GetPara(PARA_MaintenancePeriod);	
	sgUserInfo.u8RemotePara = i32GetPara(PARA_RemotePara);
	
	
//	sgUserInfo.u8PumpMotorGear1 = i32GetPara(PARA_PumpMotorGear1) * PUMP_RANGE / 100;			/*前后移动*/
//	sgUserInfo.u8PumpMotorGear2 = i32GetPara(PARA_PumpMotorGear2) * PUMP_RANGE / 100;			/*倾斜*/
//	sgUserInfo.u8LiftSpeed = i32GetPara(PARA_LiftSpeed);									/*起升*/
//	
//	sgUserInfo.b1LiftMode = i32GetPara(LIFT_MODE_PARA);						/*起升模式-制动踏板类型*/
//	
//	sgUserInfo.u16Gear1Spd = i32GetPara(PARA_Gear1Spd)  ;	/*1档速度*/
//	sgUserInfo.u16Gear2Spd = i32GetPara(PARA_Gear2Spd) 	;	/*2档速度*/
//	sgUserInfo.u16Gear3Spd = i32GetPara(PARA_Gear3Spd)  ;	/*3档速度*/
//	sgUserInfo.u16Gear4Spd = i32GetPara(PARA_Gear4Spd)  ;	/*4档速度*/
//	
//	sgUserInfo.u16ThrottleMin = i32GetPara(MOVE_THROTTLE_MIN) * 100;		/*油门死区值, uint 0.1V*/
//	sgUserInfo.u16ThrottleMax = i32GetPara(MOVE_THROTTLE_MAX) * 100;		/*油门死区值最大 ,uint 0.1V*/
//	sgUserInfo.u16ThrottleRange = (sgUserInfo.u16ThrottleMax - sgUserInfo.u16ThrottleMin) >> 1;
//	sgUserInfo.u16ThrottleMid = i32GetPara(MOVE_THROTTLE_MID) * MOTOR_SPEED_RANGE / 100;
//	sgUserInfo.u16ThrottleMidValue = (sgUserInfo.u16ThrottleMax + sgUserInfo.u16ThrottleMin) >> 1;
//	
//	sgUserInfo.u16LiftUpMin = i32GetPara(LIFT_UP_THROTTLE_MIN) * 100;
//	sgUserInfo.u16LiftUpMax = i32GetPara(LIFT_UP_THROTTLE_MAX) * 100;
//	sgUserInfo.u16LiftUpRange = (sgUserInfo.u16LiftUpMax - sgUserInfo.u16LiftUpMin) >> 1;
//	sgUserInfo.u16LiftUpMid = i32GetPara(LIFT_UP_THROTTLE_MID) * PUMP_RANGE / 100;
//	sgUserInfo.u16LiftUpMidValue = (sgUserInfo.u16LiftUpMax + sgUserInfo.u16LiftUpMin) >> 1;
//	
//	sgUserInfo.u16LiftDownMin = i32GetPara(LIFT_DOWN_THROTTLE_MIN) * 100;
//	sgUserInfo.u16LiftDownMax = i32GetPara(LIFT_DOWN_THROTTLE_MAX) * 100;
//	sgUserInfo.u16LiftDownRange	= (sgUserInfo.u16LiftDownMin - sgUserInfo.u16LiftDownMax) >> 1;
//	sgUserInfo.u16LiftDownMid = i32GetPara(LIFT_DOWN_THROTTLE_MID) * PUMP_RANGE / 100;
//	sgUserInfo.u16LiftDownMidValue	= (sgUserInfo.u16LiftDownMax + sgUserInfo.u16LiftDownMin) >> 1;
//	
//	sgUserInfo.SteerAngleDecSpd.u16StartAngle = i32GetPara(TURN_START_ANGLE) * 10;
//	sgUserInfo.SteerAngleDecSpd.u16EndAngle = i32GetPara(TURN_END_ANGLE) * 10;
//	sgUserInfo.SteerAngleDecSpd.u16StartAngleSpd = i32GetPara(START_ANGLE_SPD) * MOTOR_SPEED_RANGE / 100;
//	sgUserInfo.SteerAngleDecSpd.u16EndAngleSpd = i32GetPara(END_ANGLE_SPD) * MOTOR_SPEED_RANGE / 100;
//	sgUserInfo.SteerAngleDecSpd.fAgnleDecSpdFactor = (sgUserInfo.SteerAngleDecSpd.u16StartAngleSpd - sgUserInfo.SteerAngleDecSpd.u16EndAngleSpd) /   \
//													 (sgUserInfo.SteerAngleDecSpd.u16EndAngle - sgUserInfo.SteerAngleDecSpd.u16StartAngle);
//													 
	
//	
//	sgUserInfo.u8BatteryType = i32GetPara(PARA_BatteryType);

	/*lilu 20230823 add user mode*/
	{
//		u16Tmp = i32GetPara(USERINFO_MODE);
		sgUserInfo.b1HourConutMode = u16Tmp & 0x01;						/*bit0: HourCount Mode*/
		sgUserInfo.b1StartUpLock = (u16Tmp >> 1) & 0x01;			/*bit1: StartUpCheck*/
		sgUserInfo.b1LiftLock = (u16Tmp >> 2) & 0x01;					/*bit2: Lift by Lock */
		sgUserInfo.b1LiftPedal = (u16Tmp >> 3) & 0x01;				/*bit3: Lift by Peadl*/
		sgUserInfo.b1MoveLiftMode = (u16Tmp >> 4) & 0x01;			/*bit4: Move and Lift */
		sgUserInfo.b1SteerCheck = (u16Tmp>>5) & 0x01; 				/*bit5:Steer Controller Online Check*/
		sgUserInfo.b1CarType = (u16Tmp>>6) & 0x01; 						/*bit6:pedal&fence*/
		sgUserInfo.b1SpeedLimitAfterLift = (u16Tmp>>7) & 0x01;
	}
	
	u32HourCount = u32HourCountRead();
		__disable_irq();
		gCanSendPdoInfo.sgHMISendPdo.Hourcnt = u32HourCount * 360 ;
		__enable_irq();
	
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
//		sgSwiInput.b1HeightLimitState = sgSaveState.b1HeightSpdLimit;
		sgUserInfo.b1PasswordFunc = sgSaveState.b1PasswordFunc;
		u16EepromRead(PARA_HourSetTime, &u16Tmp, 1);	
	}
	
	__disable_irq();
//	gCanSendPdoInfo.CanSend260Info.u8HourCountL = u32HourCount & 0xFF;
//	gCanSendPdoInfo.CanSend260Info.u8HourCountH = (u32HourCount >> 8) & 0xFF;

	__enable_irq();		
	/*Para Initial*/
	
	//sgUserInfo.u16RentalTime = i32GetPara(RENTAL_TIME);
	
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
		vSwiInitCheck();
//		vSwiInitChcek();
	}
	
	if(1 == u8EcuProcFlag)
	{
		vAiMonitor();
		vSwiMonitor();		
		vCanRevPdoProc();
		vEcuSetBeepPeriod();
		vCanRevPdoProc();
		vErrProc();
		vPressureCalibration();
		vBatteryManage();
		
		if (true == u8GetNetTimerOverFlag(TIMER_HourCount))
		{
			vResetNetTimer(TIMER_HourCount);
			u16SecCnt++;
			
			if (u16SecCnt >= 360)			/**/
			{
				u16SecCnt = 0;
				u32HourCount++;
				vHourCountWrite(u32HourCount);
			}
			
		

		}
		__disable_irq();
		gCanSendPdoInfo.sgHMISendPdo.Hourcnt = u32HourCount * 360 + u16SecCnt;
		__enable_irq();
	}
	vWdgSetFun(WDG_USER_BIT);
	

	
}

#endif