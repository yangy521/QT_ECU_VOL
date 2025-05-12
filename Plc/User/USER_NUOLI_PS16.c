/*******************************************************************************
*通用程序框架* 						   *
* Author: QExpand; Lilu                                                        *
* Date: 2023/05/30    														   *
* Revision:	 														 		   *
*******************************************************************************/
#include "UserCanComm.h"
#include "USER_NUOLI_PS16.h"
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
#include "timer_canfestival.h"

#if (USER_TYPE == USER_NUOLI_PS16)//修改


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
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 20, .u16CanId = 0x261 },
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 50, .u16CanId = 0x260 },
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 50, .u16CanId = 0x2F8 },
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 50, .u16CanId = 0x2F9 },
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 50, .u16CanId = 0x2FA },
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50, .u16CanId = 0x000 },
	},
	/*Canopen接收*/
	/*设置对应id及使能*/
	.RpdoPara = {
		{.b1Flag = 1, .b11CanRevId = 0x1E0},
		{.b1Flag = 1, .b11CanRevId = 0x361},
		{.b1Flag = 1, .b11CanRevId = 0x1A1},
		{.b1Flag = 1, .b11CanRevId = 0x2F1},
		{.b1Flag = 1, .b11CanRevId = 0x1B0},
		{.b1Flag = 1, .b11CanRevId = 0x3E0},
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
		uint16_t b1SteerSpeedLimit: 1;
		uint16_t b1Pedal: 1;
		uint16_t b1Fence2: 1;
		uint16_t b1LiftLimit: 1;
		uint16_t b1HeightLimit:1;
		uint16_t b1Fence1: 1;
		uint16_t b1SteerSafeLock: 1;
		uint16_t b4Reserve:4;
		uint16_t b1H180Swi:1;
		uint16_t b1HeightLimitState:1;
	};
}xSwiInput;

/*动作限制设置*/
typedef union
{
	uint16_t u16Data;
	struct
	{
		uint16_t b1Init:1;
		uint16_t b1BMS_Erro:1;
		uint16_t b1SteerErro:1;
		uint16_t b1SafeLimit:1;
		uint16_t b1LogicSequnce:1;
		uint16_t b1CanIDlost1:1;
		uint16_t b1CanIDlost2:1;
		uint16_t b1CanIDlost3:1;
		uint16_t b1CanIDlost4:1;
		uint16_t b1FirmwareFailure:1;
		uint16_t b1Remote:1;
		uint16_t b1MultiPump:1;
		uint16_t b1RemoteOrder:1;
		uint16_t b1Card:1;
		uint16_t b1EmsError:1;
		uint16_t b1PedalLock:1;
	};
}xAct;

typedef union
{
	uint16_t u16Data;
	struct
	{
		uint16_t b1Init:1;
		uint16_t b1Act:1;
		uint16_t b1FenceLimit:1;
		uint16_t b14Rserve1:13;
	};
}xMove;

typedef union
{
	uint16_t u16Data;
	struct
	{
		uint16_t b1Init:1;
		uint16_t b1Act:1;
		uint16_t b1BMSWarning:1;
		uint16_t b1HeightLimit:1;
		uint16_t b1RmoteCommand:1;
		uint16_t b1RemoteIDLost:1;
		uint16_t b1Card:1;
		uint16_t b1H1800:1;
		uint16_t b8Rserve1:8;
	};
}xLift;

typedef union
{
	uint8_t u8Data[5];
	struct
	{
		uint8_t b1Pedal:1;
		uint8_t b7Rserve1:7;
		
		uint8_t b1Spd2Walk:1;	
		uint8_t b7Rserve2:7;		
		
		
		uint8_t b1Spd3Lift:1;		
		uint8_t b1Spd3Slow:1;
		uint8_t b6Rserve3:6;
		
		
				
		uint8_t b1Spd4Zhili:1;
		uint8_t b7Rserve4:7;
		
		uint8_t b1Spd2BMS:1;
		uint8_t b7Rserve5:7;
		
		
	};
}xSpd;

typedef union
{
	uint8_t u8Data;
	struct
	{
		uint8_t b1Horizontal:1;
		uint8_t b7Rserve:7;
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
		uint8_t b1LeftMove:1;
		uint8_t b1RightMove: 1;
		uint8_t b4Reserve2:4;
		/*附加状态逻辑*/
		uint8_t	b1ZhiLiState:1;
		uint8_t b1HeightLimitState:1;
		uint8_t b1EMS:1;
		uint8_t b5Reserve3:5;
		
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
	uint8_t		b3Reserve: 3;	
	
	/*118号参数用于配置设置*/
	uint8_t		b1HourConutMode: 1;		/*0: 上电计时， 1：互锁计时*/
	uint8_t		b1StartUpLock: 1;		/*0： 检测， 1：不检测*/
	uint8_t		b1LiftLock: 1;			/*0： 需要互锁， 1：不需要*/
	uint8_t		b1LiftPedal: 1;			/*0： 无要求， 1：踏板open =1， close =0 / open =0， close =1 才可以动作*/
	uint8_t		b1MoveLiftMode: 1;		/*0： 同时动作， 1：互斥*/
	uint8_t		b1SteerCheck:1;
	uint8_t		b1CarType:1;   
	uint8_t		b1SpeedLimitAfterLift: 1;
	
	uint8_t		b1ForwardBackwardReverse:1;
	uint8_t		b1RemoteControl:1;
	uint8_t		b1CardState1:1;
	uint8_t		b1CardState2:1;
	uint8_t		b4Reserve:4;
	
	uint16_t	u16RatioOfTransmission;
	uint16_t	u16MotorMaxSpd;
	
	uint16_t	u16RentalTime;
	xSteerAngleDecSpd SteerAngleDecSpd;
}xUserInfo;

typedef union
{
	uint8_t u8Data;
	struct
		{
			uint8_t b1ToggleBit:1;
			uint8_t b1SnailMonde:1;
			uint8_t b1PB0LI1BatteryType:1;
			uint8_t b1LiftDown:1;
			uint8_t	b1LeanForward:1;
			uint8_t b1LeanBackward:1;
			uint8_t b1UpRight:1;//直立
			uint8_t b1MainContacter:1;
		};
}x261State;

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

/*Can手柄开关量*/
typedef union
{
	uint16_t u16Data;
	struct
	{
		uint16_t b1NeutralPose:1;
		uint16_t b1EmergencyReverse:1;
		uint16_t b1SnailRequest:1;
		uint16_t b1Horn:1;
		uint16_t b1Lift1:1;
		uint16_t b1LiftDown1:1;
		uint16_t b1Lift2:1;
		uint16_t b1LiftDown2:1;
		uint16_t b1Pick1:1;
		uint16_t b1Pick2:1;
		uint16_t b2Spare:2;
		uint16_t b1ZhiLi:1;
		uint16_t b2Reserve:2;
		uint16_t b1StuffToggle:1;
	};
}xKeyInfo;

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

/*报文处理*/


typedef union
{
	uint8_t u8Data[8];
	struct 
		{
			uint8_t u8Lable;
			uint8_t u8ErrorCode;
			uint8_t u8SpeedL;
			uint8_t u8SpeedH;
			uint8_t u8MotorTemperature;
			uint8_t u8BoardTemperature;
			uint8_t u8MotorCurrentL;
			uint8_t u8MotorCurrentH;
		};
}xCanSend2F8X1Info;//2F8X1

typedef union
{
	uint8_t u8Data[8];
	struct 
		{;
			uint8_t u8Lable;
			uint8_t u8BDI;
			uint8_t u8HourCountKSIL;
			uint8_t u8HourCountKSIM;
			uint8_t u8HourCountKSIH;
			uint8_t u8HourCountKENL;
			uint8_t u8HourCountKENM;
			uint8_t u8HourCountKENH;
		};
}xCanSend2F8X2Info;//2F8X2

typedef union
{
	uint8_t u8Data[8];
	struct 
		{
			uint8_t u8Lable;
			
			uint8_t b1SafeLock:1;
			uint8_t b1ForWard:1;
			uint8_t b1BackWard:1;
			uint8_t b1Belly:1;//急反开关
			uint8_t b1Height300:1;
			uint8_t b1Height1800:1;
			uint8_t b1LiftLimit:1;
			uint8_t b1SlowSwi:1;
			
			uint8_t u8MoveThrottleL;
			uint8_t u8MoveThrottleH;
			
			uint8_t u8BatteryVoltageL;
			uint8_t u8BatteryVoltageH;
			uint8_t u8MotorRotateSpeedL;
			uint8_t u8MotorRotateSpeedH;
		};
}xCanSend2F8X3Info;//2F8X3

typedef union
{
	uint8_t u8Data[8];
	struct 
		{
			uint8_t u8Lable;
			
			uint8_t b1LeftFence1:1;
			uint8_t b1LeftFence2:1;
			uint8_t b1RightFence1:1;
			uint8_t b1RightFence2:1;
			uint8_t b1PedalClose:1;
			uint8_t b1PedalOpen:1;
			uint8_t b1BatteryMonitor:1;
			uint8_t b1Reserve:1;
			
			uint8_t u8Reserve2;
			
			uint8_t u8Reserve3;
			uint8_t u8Reserve4;
			
			uint8_t b5Reserve5:5;
			uint8_t b1MainContactor:1;
			uint8_t b2Reserve5:2;
			
			uint8_t b6Reserve6:6;
			uint8_t b1RightMove:1;
			uint8_t b1GateUp:1;
			
			uint8_t b1GateDown:1;
			uint8_t b1Forward:1;
			uint8_t b1BackWard:1;
			uint8_t b3Reserve7:3;
			uint8_t b1Trumpet:1;
			uint8_t b1InPutReserve5:1;
		};
}xCanSend2F8X4Info;//2F8X4

typedef union
{
	uint8_t u8Data[8];
	struct 
		{
			uint8_t u8Lable;
			
			uint8_t b1InPutReserve1:1;
			uint8_t b1InPutReserve2:1;
			uint8_t b1InPutReserve3:1;
			uint8_t b1InPutReserve4:1;
			uint8_t b1OutPutReserve5:1;
			uint8_t b1OutPutReserve6:1;
			uint8_t b1OutPutReserve7:1;
			uint8_t b1OutPutReserve8:1;
			
			uint8_t u8Reserve2;
			uint8_t u8Reserve3;
			uint8_t u8Reserve4;
			uint8_t u8Reserve5;
			
			uint8_t u8OutPutReserve1;
			uint8_t u8OutPutReserve2;
		};
}xCanSend2F8X5Info;//2F8X5

typedef union
{
	uint8_t u8Data[8];
	struct 
		{
			uint8_t u8Lable;
			
			uint8_t u8SerialNumber12;
			uint8_t u8SerialNumber34;
			uint8_t u8SerialNumber56;
			uint8_t u8SerialNumber78;
			uint8_t u8SerialNumber9A;
			uint8_t u8MaxSpeed;
			uint8_t u8CarType;

		};
}xCanSend2F8X6Info;//2F8X6

typedef union
{
	uint8_t u8Data[8];
	struct 
		{
			uint8_t u8Lable;
			
			uint8_t b1LiftSwi:1;
			uint8_t b1DownSwi:1;
			uint8_t b1LeanSwi:1;
			uint8_t b1SidleSwi:1;
			uint8_t b1AttachmentSwi:1;
			uint8_t b1SpareInput1:1;
			uint8_t b1SpareInput2:1;
			uint8_t b1SpareInput3:1;
			
			uint8_t b1SpareOutPut1:1;
			uint8_t b1SpareOutPut2:1;
			uint8_t b1SpareOutput3:1;
			uint8_t b4Reserve2:4;
			
			uint8_t u8Reserve3;
			uint8_t u8Reserve4;
			uint8_t u8Reserve5;
			
			uint8_t u8Reserve6;
			uint8_t u8Reserve7;
		};
}xCanSend2F9X2Info;//2F9X2

typedef union
{
	uint8_t u8Data[8];
	struct 
		{
			uint8_t u8Lable;
			
			uint8_t u8SteerError;
			
			uint8_t u8SteerBoardTemperature;
			
			uint8_t u8SteerMotorTemperature;
			
			uint8_t u8SteerCurrentL;
			uint8_t u8SteerCurrentH;
			
			uint8_t u8SteerAngleL;
			uint8_t u8SteerAngleH;
		};
}xCanSend2FAX1Info;//2FAX1

typedef union
{
	uint8_t u8Data[8];
	struct 
		{
			uint8_t u8Lable;
			
			uint8_t b1SteerCentralSwi:1;
			uint8_t b7Reserve:7;
			
			uint8_t u8SteerAnalog1;
			uint8_t u8SteerAnalog2;
			uint8_t u8PositionSensor1;
			uint8_t u8PositionSensor2;
			uint8_t u8Reserve6;
			uint8_t u8Reserve7;

		};
}xCanSend2FAX2Info;//2FAX2

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
}xCanBlueInfo;//蓝牙模块


typedef union
{
	uint8_t u8Data;
	struct
	{
		uint8_t b1RemoteEn:1;
		uint8_t b1SpeedLimit1:1;
		uint8_t b1SpeedLimit2:1;
		uint8_t b1Nolift:1;
		uint8_t b1NoAct:1;
		uint8_t b1FaultOut:1;
		uint8_t b1Reserved:1;
		uint8_t b1HourClr;
	};
}xRemoteInfo;//远程参数

typedef union
{
	uint16_t u16Data[3];
	struct
	{
		uint16_t u16SerialNum1234;
		uint16_t u16SerialNum5678;
		uint16_t u16SerialNum9A;
		uint16_t u16MaxSpeedRemote;
	};
}xRemotePara;//远程参数

/*ECU故障与显示故障转换表*/
const static uint8_t u8ErrSwitchArray[ErrCodeMax] = 
{	
/*    0    1    2    3    4    5    6    7    8    9    10   11   12   13   14   15   16   17   18   19*/
	   00,  00,  00,  00,  00,  00,  00,  00,  36,  73,   00,  00,  12,  14,  31,  32,  17,  18,  16,  28,    
     42,  38,  25,  00,  31,  00,  51,  00,  00,  15,	  00,  23,  22,  21,  28,  69,  33,  34,  46,  00,   
		 47,  00,  00,  00,  00,  00,  00,  00,  00, 	00,   50,  51,  52,  53,  54,  55,  56,  57,  58,  59,
	   60,  61,  62,  63,  64,  65,  66,  67,  68,  69,   70,  71,  72,  73,  74,  75,  76,  77,  78,  79, 
		 80,  81,  82,  51,  51,  51,  51,  51,  51,  89,   90,  91,  92,  93,  94,  95,  96,  97,  98,  99,
		100, 101, 102, 103, 104, 105, 106, 107, 108, 109,  110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 
		120, 121, 122, 123, 124, 125, 126, 127, 128, 129,  130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 
		140, 141, 142, 143, 144, 145, 146, 147, 148, 149,  150, 151, 152, 153, 154, 155, 156, 157, 158, 159,
		160, 161, 162, 163, 164, 165, 166, 167, 168, 169,  170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 
		180, 181, 182, 183, 184, 185, 186, 187, 188, 189,  190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 
		200, 201, 202, 203, 204, 205, 206, 207, 208, 209,  210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 
		220, 221, 222, 223, 224, 225, 226, 227, 228, 229,  230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 
		240, 241, 242, 243, 244, 245, 246, 247, 248, 249,  250, 251, 252, 253, 254,
};

/*故障与动作限制*/

/*PDO发送待完善*/
xCanSendPdoInfo gCanSendPdoInfo ;

/*PDO接收待完善*/
xCanRevPdoInfo gCanRevPdoInfo;	

/*变量*/

static xActLimit sgLock;
static xKeyInfo sgKeyInfo;
static xActLogic sgActLogic;
static xSwiInput sgSwiInput;
static xSaveStateInfo sgSaveState;

static int16_t  i16MotorSpd = 0;
static uint16_t u16MotorVal = 0;
static uint8_t	u8PumpVal = 0;
static uint8_t	u8PropVal = 0;
static uint8_t	u8Auxiliary = 0;
static xUserInfo sgUserInfo;
static uint32_t u32HourCount = 0;

static uint16_t	u16ID190Cnt = 0;
static uint16_t u16ID200Cnt = 0;
static uint16_t u16ID360Cnt = 0;
static int16_t i16SteerAngle = 0;

static uint8_t u8BMS_FLAG = 0;

static xCanSend2F8X1Info Info2F8X1;
static xCanSend2F8X2Info Info2F8X2;
static xCanSend2F8X3Info Info2F8X3;
static xCanSend2F8X4Info Info2F8X4;
static xCanSend2F8X5Info Info2F8X5;
static xCanSend2F8X6Info Info2F8X6;
static xCanSend2F9X2Info Info2F9X2;
static xCanSend2FAX1Info Info2FAX1;
static xCanSend2FAX2Info Info2FAX2;
static xRemoteInfo gRemoteInfo1;
static xRemoteInfo gRemoteInfo2;
static uint8_t u8RemoteLockFlag = 0;
static xRemotePara gRemotePara;
static uint8_t u8ID270Lostcnt = 0;
static uint8_t u8ID27ALostcnt = 0;
static xCanBlueInfo CanRev608InfoLast;

static xCanSendCommon Rev588;

/*********************************通用功能部分*********************************/
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


/*********************************初始化*********************************/

/*******************************************************************************
* Name: uint8_t u8SwiInitChcek(void)
* Descriptio: 开关量初始化检查
* Input: NULL
* Output: NULL  
*******************************************************************************/
static void vSwiInitChcek(void)
{
	uint8_t u8Res = 0;
	if((0 == i32LocalDiGet(EMERGENCY_REVERSE_SWI)) 
		|| (1 == i32LocalDiGet(SAFELOCK_SWI)) 
		|| (0 != (sgKeyInfo.u16Data & 0x10F6)
		|| (0 != gCanRevPdoInfo.CanRevInfo1E0.i16AuxiliaryThrottle)
		|| (0 != gCanRevPdoInfo.CanRevInfo1E0.i16LiftThrottle)
		|| (0 != gCanRevPdoInfo.CanRevInfo1E0.i16MoveThrottle))
			) 		
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
		case 0x1E0:
			if(CAN_NORMAL == u8State)
			{
				i32ErrCodeClr(CanIDlost1_ERR);
				sgLock.Action.b1CanIDlost1 = 0;
			}
			else if(CAN_LOST == u8State)
			{
				i32ErrCodeSet(CanIDlost1_ERR);
				sgLock.Action.b1CanIDlost1 = 1;
			}
			break;
		case 0x361:
			if(1 == sgUserInfo.b1SteerCheck)
			{
				if(CAN_NORMAL == u8State)
				{
					i32ErrCodeClr(CanIDlost2_ERR);
					sgLock.Action.b1CanIDlost2 = 0;
				}
				else if(CAN_LOST == u8State)
				{
					i32ErrCodeSet(CanIDlost2_ERR);
					sgLock.Action.b1CanIDlost2 = 1;					
				}
			}
			else
			{
				i32ErrCodeClr(CanIDlost2_ERR);
				sgLock.Action.b1CanIDlost2 = 0;							
			}
			break;
		case 0x2F1:
			if(LiBattery == sgUserInfo.u8BatteryType)
			{
				if(CAN_NORMAL == u8State)
				{
					i32ErrCodeClr(CanIDlost3_ERR);
					sgLock.Action.b1CanIDlost3 = 0;
				}
				else if(CAN_LOST == u8State)
				{
					i32ErrCodeSet(CanIDlost3_ERR);
					sgLock.Action.b1CanIDlost3 = 1;					
				}
			}
			else
			{
				i32ErrCodeClr(CanIDlost3_ERR);
				sgLock.Action.b1CanIDlost3 = 0;	
			}
			break;
		case 0x1A1:
			if(CAN_NORMAL == u8State)
			{
				i32ErrCodeClr(CanIDlost4_ERR);
				sgLock.Action.b1CanIDlost4 = 0;
			}
			else if(CAN_LOST == u8State)
			{
				i32ErrCodeSet(CanIDlost4_ERR);
				sgLock.Action.b1CanIDlost4 = 1;					
			}
			break;
		default:
			break;
	
	}
}

/*********************************输入处理*********************************/
/*接收PDO处理函数*/
static void vCanRevPdoProc(void)
{
	static xCanRev1E0Info CanRev1E0InfoLast;
	static xCanRev361Info CanRev361InfoLast;
	static xCanRev2F1Info CanRev2F1InfoLast;
	static xCanRev1A1Info CanRev1A1InfoLast;
	
	uint8_t BatteryCapacity = 0;
	/*手柄1E0*/
//	if(0 != memcmp((char*)CanRev1E0InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo1E0.u8Data, sizeof(CanRev1E0InfoLast)))
	{
		memcpy((char*)CanRev1E0InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo1E0.u8Data, sizeof(CanRev1E0InfoLast));
		/*添加相关操作*/
		sgKeyInfo.u16Data = (CanRev1E0InfoLast.u8Data[1]<<8)|(CanRev1E0InfoLast.u8Data[0]);
		{
			
			if(CanRev1E0InfoLast.i16MoveThrottle < 0)
				CanRev1E0InfoLast.i16MoveThrottle = - CanRev1E0InfoLast.i16MoveThrottle;
			
			if(CanRev1E0InfoLast.i16LiftThrottle < 0)
				CanRev1E0InfoLast.i16LiftThrottle = - CanRev1E0InfoLast.i16LiftThrottle;
			
			if(CanRev1E0InfoLast.i16AuxiliaryThrottle < 0)
				CanRev1E0InfoLast.i16AuxiliaryThrottle = - CanRev1E0InfoLast.i16AuxiliaryThrottle;
			
			u16MotorVal = (uint16_t)(CanRev1E0InfoLast.i16MoveThrottle);
			u8PumpVal = (uint8_t)((CanRev1E0InfoLast.i16LiftThrottle)* PUMP_MAX_VALUE /4096);
			u8Auxiliary = (uint8_t)((CanRev1E0InfoLast.i16AuxiliaryThrottle)* PUMP_MAX_VALUE /4096);
		}
	}
	/*转向361*/
//	if(0 != memcmp((char*)CanRev361InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo361.u8Data, sizeof(CanRev361InfoLast)))
	if(1 == sgUserInfo.b1SteerCheck)
	{
		memcpy((char*)CanRev361InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo361.u8Data, sizeof(CanRev361InfoLast));
		/*添加相关操作*/
		i16SteerAngle = (CanRev361InfoLast.u8SteerAngleH << 8) | CanRev361InfoLast.u8SteerAngleL;
		if(0 != CanRev361InfoLast.u8ErrSteer)
		{
			gCanSendPdoInfo.CanSend260Info.u8ErrorSteer = CanRev361InfoLast.u8ErrSteer;
			
			sgLock.Action.b1SteerErro = 1;
			if(0 == gCanSendPdoInfo.CanSend261Info.u8ErrorMove)
			{
				gCanSendPdoInfo.CanSend261Info.u8ErrorMove = CanRev361InfoLast.u8ErrSteer + 100;
			}
		}
			
//	CanRev360InfoLast.u8ErrSteer;
	}
	
	/*仪表261*/
	{
		if((0 != sgLock.SpeedRate.u8Data[2])
			||(0 != sgLock.SpeedRate.u8Data[3]))
		{
			gCanSendPdoInfo.CanSend261Info.b1SnailMonde = 1;
		}
		else
		{
			gCanSendPdoInfo.CanSend261Info.b1SnailMonde = 0;
		}
		if(LiBattery == sgUserInfo.u8BatteryType)
		{
			gCanSendPdoInfo.CanSend261Info.b1PB0LI1BatteryType = 1;
		}
		else
		{
			gCanSendPdoInfo.CanSend261Info.b1PB0LI1BatteryType = 0;
		}
		gCanSendPdoInfo.CanSend261Info.u8BatteryTypeL = 0;
		gCanSendPdoInfo.CanSend261Info.u8BatteryTypeH = 0;
	}
		
	/*BMS 2F1*/
	{
		uint8_t BMSWarning = 0;
		uint8_t BMSError = 0;
		
		memcpy((char*)CanRev2F1InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo2F1.u8Data, sizeof(CanRev2F1InfoLast));
		if(0 != CanRev2F1InfoLast.u8BMSError)
		{
			BMSError = 0;
			do{
				BMSError++;
				if ((BMSError & 0x1) != 0)
					break;
				BMSError = BMSError >> 1;
			}while (BMSError < 8);
			gCanSendPdoInfo.CanSend260Info.u8ErrorBMS = BMSError;
		}
		else if(0 != CanRev2F1InfoLast.u8BMSWarning)
		{
			BMSWarning = 8;
			do{
				BMSWarning++;
				if ((BMSWarning & 0x1) != 0)
					break;
				BMSWarning = BMSWarning >> 1;
			}while (BMSWarning < 16);
			gCanSendPdoInfo.CanSend260Info.u8ErrorBMS = BMSWarning;
		}
		else
		{
			BMSError = 0;
			BMSWarning = 0;
			gCanSendPdoInfo.CanSend260Info.u8ErrorBMS = 0;
		}
	}
	/*仪表1A1*/
	#if 1
	{//电量相关功能
		memcpy((char*)CanRev1A1InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo1A1.u8Data, sizeof(CanRev1A1InfoLast));

		BatteryCapacity = CanRev1A1InfoLast.u8SOC;
		if ((BatteryCapacity <= BAT_LOW_ERR_VAL)
		//	||(0 != (u8BMS_ERRCODE & 0x01))
				)	//禁止动作
		{
			if(BatteryCapacity <= BAT_LOW_ERR_VAL)
			{
				i32ErrCodeSet(BAT_LOW_2_ERR);
			}
			sgLock.Action.b1BMS_Erro = 1; 
		}
		else if ((BatteryCapacity <= BAT_LOW_WARING_VAL)
				//	||(0 !=(u8BMS_ERRCODE & 0x0A))
							)
		{
			sgLock.Action.b1BMS_Erro = 0; 
			i32ErrCodeClr(BAT_LOW_2_ERR);
			sgLock.Lift.b1BMSWarning = 1;
			sgLock.SpeedRate.b1Spd2BMS = 1;
			if(BatteryCapacity <= BAT_LOW_WARING_VAL)
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
	}
	#endif
	

	/*蓝牙270，27A*/
	if(0 != sgUserInfo.b1RemoteControl)
	{
		if((0 != gRemoteInfo1.b1NoAct)
			||(0!= gRemoteInfo2.b1NoAct))
		{
			sgLock.Action.b1RemoteOrder = 1;
		}
		else
		{
			sgLock.Action.b1RemoteOrder = 0;
		}
		
		if((0 != gRemoteInfo1.b1Nolift)
			||(0!= gRemoteInfo2.b1Nolift))
		{
			sgLock.Lift.b1RmoteCommand = 1;
		}
		else
		{
			sgLock.Lift.b1RmoteCommand = 0;
		}
		
		if((0 != gRemoteInfo1.b1NoAct)
			||(0 != gRemoteInfo1.b1Nolift)
			||(0 != gRemoteInfo1.b1SpeedLimit1)
			||(0 != gRemoteInfo1.b1SpeedLimit2)
			||(0 != gRemoteInfo2.b1NoAct)
			||(0 != gRemoteInfo2.b1Nolift)
			||(0 != gRemoteInfo2.b1SpeedLimit1)
			||(0 != gRemoteInfo2.b1SpeedLimit2))//存在远程限制的情况下，报警
		{
			i32ErrCodeSet(REMOTE_COMMAND_LIMIT_ERR);
		}
		else
		{
			i32ErrCodeClr(REMOTE_COMMAND_LIMIT_ERR);
		}
	
	}
		
	/*2F8,2F9,2FA报文填充*/
	{
		Info2F8X1.u8Lable = 0x71;
		Info2F8X1.u8ErrorCode = gCanSendPdoInfo.CanSend261Info.u8ErrorMove;
		
		Info2F8X2.u8Lable = 0x72;
		Info2F8X2.u8BDI = BatteryCapacity;
		Info2F8X2.u8HourCountKENL = u32HourCount & 0xFF;
		Info2F8X2.u8HourCountKENM = (u32HourCount>>8) & 0xFF;
		Info2F8X2.u8HourCountKENH = (u32HourCount>>16) & 0xFF;
		Info2F8X2.u8HourCountKSIL = Info2F8X2.u8HourCountKENL;
		Info2F8X2.u8HourCountKSIM = Info2F8X2.u8HourCountKENM;
		Info2F8X2.u8HourCountKSIH = Info2F8X2.u8HourCountKENH;
		
		Info2F8X3.u8Lable = 0x73;
		Info2F8X3.b1BackWard = sgActLogic.b1BackwardAct;
		Info2F8X3.b1Belly = sgSwiInput.b1Ems;
		Info2F8X3.b1ForWard = sgActLogic.b1ForwardAct;
		Info2F8X3.b1Height1800 = sgSwiInput.b1HeightLimit;
		Info2F8X3.b1Height300 = sgSwiInput.b1HeightLimit;
		Info2F8X3.b1LiftLimit = sgSwiInput.b1LiftLimit;
		Info2F8X3.b1SafeLock = sgSwiInput.b1SafeLock;
		Info2F8X3.b1SlowSwi = sgKeyInfo.b1SnailRequest;
		Info2F8X3.u8MoveThrottleL = (u16MotorVal * 32767 / 4064) & 0xFF;
		Info2F8X3.u8MoveThrottleH = ((u16MotorVal * 32767 / 4064)>>8)& 0xFF;
		Info2F8X3.u8BatteryVoltageL = (i32GetPara(PARA_Ksi) / 10)&0xFF;
		Info2F8X3.u8BatteryVoltageH = ((i32GetPara(PARA_Ksi) / 10)>>8)&0xFF;
		
		Info2F8X4.u8Lable = 0x74;
		Info2F8X4.b1LeftFence1 = sgSwiInput.b1Fence1;
		Info2F8X4.b1RightFence1 = sgSwiInput.b1Fence2;
		Info2F8X4.b1PedalClose = sgSwiInput.b1Pedal;
		Info2F8X4.b1RightMove = sgActLogic.b1RightMove;
		Info2F8X4.b1GateUp = sgActLogic.b1LiftUpAct;
		Info2F8X4.b1GateDown = sgActLogic.b1LiftDownAct;
		Info2F8X4.b1Forward =sgActLogic.b1ForwardAct;
		Info2F8X4.b1BackWard = sgActLogic.b1BackwardAct;
		
		Info2F8X5.u8Lable = 0x75;
		
		
		Info2F8X6.u8Lable = 0x76;
		Info2F8X6.u8SerialNumber12 = (gRemotePara.u16SerialNum1234>>8) & 0XFF;
		Info2F8X6.u8SerialNumber34 = (gRemotePara.u16SerialNum1234) & 0XFF;
		Info2F8X6.u8SerialNumber56 = (gRemotePara.u16SerialNum5678>>8) & 0XFF;
		Info2F8X6.u8SerialNumber78 = (gRemotePara.u16SerialNum5678) & 0XFF;
		Info2F8X6.u8SerialNumber9A = (gRemotePara.u16SerialNum9A>>8) & 0XFF;
		Info2F8X6.u8MaxSpeed = gRemotePara.u16MaxSpeedRemote;
		
		Info2F9X2.u8Lable = 0x72;
		Info2F9X2.b1LiftSwi = sgKeyInfo.b1Lift1;
		Info2F9X2.b1DownSwi = sgKeyInfo.b1LiftDown1;
		
		
		Info2FAX1.u8Lable = 0x71;
		Info2FAX1.u8SteerError = gCanRevPdoInfo.CanRevInfo361.u8ErrSteer;
		Info2FAX1.u8SteerBoardTemperature = 55;
		Info2FAX1.u8SteerMotorTemperature = 55;
		Info2FAX1.u8SteerCurrentL = gCanRevPdoInfo.CanRevInfo3E0.u8SteerCurrentL;
		Info2FAX1.u8SteerCurrentH = gCanRevPdoInfo.CanRevInfo3E0.u8SteerCurrentH;
		Info2FAX1.u8SteerAngleL = gCanRevPdoInfo.CanRevInfo361.u8SteerAngleL & 0xFF;
		Info2FAX1.u8SteerAngleH = gCanRevPdoInfo.CanRevInfo361.u8SteerAngleH & 0xFF;
		
		Info2FAX2.u8Lable =0x72;
		Info2FAX2.b1SteerCentralSwi = sgSwiInput.b1SteerSpeedLimit;
		Info2FAX2.u8PositionSensor1 = gCanRevPdoInfo.CanRevInfo361.u8Reserve3;

	}
}



static void vAiMonitor(void)
{
	uint8_t u8SpeedRate = 100;
	/*add Turn Dec Spd*/
	
	if(u16MotorVal > MOTOR_MAX_SPEED_VALUE)
	{
		u16MotorVal = MOTOR_MAX_SPEED_VALUE;
	}
	
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
	
	if (0 != sgLock.SpeedRate.u8Data[4])
	{
		if (u8SpeedRate >= i32GetPara(PARA_SlowDriveSpeed))
		{
			u8SpeedRate = i32GetPara(PARA_SlowDriveSpeed);
		}
	}
	
	if(1 == sgUserInfo.b1RemoteControl)//远程模块速度限制
	{
	if((0 != gRemoteInfo1.b1SpeedLimit1)
			||(0 != gRemoteInfo2.b1SpeedLimit1))
		{
			if (u8SpeedRate >= sgUserInfo.u16Gear1Spd)
			{
				u8SpeedRate = sgUserInfo.u16Gear1Spd;
			}
		}
		
		if((0 != gRemoteInfo1.b1SpeedLimit2)
			||(0 != gRemoteInfo2.b1SpeedLimit2))
		{
			if (u8SpeedRate >= sgUserInfo.u16Gear4Spd)
			{
				u8SpeedRate = sgUserInfo.u16Gear4Spd;
			}
		}
	}
	
	
	u16MotorVal = (uint16_t)((((uint32_t)u16MotorVal) * u8SpeedRate)/ 100 );
	
	/*油泵速度设置*/
	

	if(0 == sgUserInfo.b1CarType)
	{
		if(u8Auxiliary>u8PumpVal)
			u8PumpVal = u8Auxiliary;
	}
	
	if(u8PumpVal>PUMP_MAX_VALUE)
	{
		u8PumpVal = PUMP_MAX_VALUE;
	}
	
	u8PropVal = u8PumpVal;
	
	u8PumpVal = (uint8_t)((uint16_t)u8PumpVal * sgUserInfo.u8LiftSpeed /100);
	
	/*起升挡位限制*/
	
	if (0 != sgLock.PumpRate.b1Horizontal)//侧移调速
	{
		u8PumpVal = (uint8_t)((uint16_t)u8Auxiliary * sgUserInfo.u8MaxTurnSpeed /100);
	}
}

/*开关量监控*/
static void vSwiMonitor(void)
{
	xSwiInput SwiInput = {0};
	static xKeyInfo KeyInfoRecord;
	static uint16_t u16ZhiLiCnt = 0; //直立行走计时
	
	/*开关量接收*/
	if(0 == i32LocalDiGet(HEIGHT_SPEEDLIMIT_SWI))
	{
		SwiInput.b1HeightSpdlimit = 1;
	}

	if(0 == i32LocalDiGet(EMERGENCY_REVERSE_SWI))
	{
		SwiInput.b1Ems = 1;
	}
	
	if(1 == i32LocalDiGet(SAFELOCK_SWI))
	{
		SwiInput.b1SafeLock = 1;
	}
	
	if(1 == i32LocalDiGet(STEER_SPEED_LIMIT))
	{
		SwiInput.b1SteerSpeedLimit = 1;
	}
	
	if(1 == i32LocalDiGet(PEDAL_SWI))
	{
		SwiInput.b1Pedal = 1;
	}
	
	if(1 == i32LocalDiGet(FENCE2_SWI))
	{
		SwiInput.b1Fence2 = 1;
	}
	
	if(0 == i32LocalDiGet(LIFT_LIMT_SWI))
	{
		SwiInput.b1LiftLimit = 1;
	}
	
	if(0 == i32LocalDiGet(HEIGHT_LIMIT_SWI))
	{
		SwiInput.b1HeightLimit = 1;
	}
	
	if(1 == i32LocalDiGet(FENCE1_SWI))
	{
		SwiInput.b1Fence1 = 1;
	}
	
	if(1 == i32LocalDiGet(STEER_SAFE_LOCK))
	{
		SwiInput.b1SteerSafeLock = 1;
	}

	/*高度限速开关设置*/
	if ((0 == sgSwiInput.b1HeightSpdlimit)&&(1 == SwiInput.b1HeightSpdlimit) && (1 == sgActLogic.b1LiftUpAct))//300mm限速
	{
		SwiInput.b1HeightLimitState = 1;
		sgSwiInput.b1HeightLimitState = 1;
		sgSaveState.b1HeightSpdLimit = 1;
		u16EepromWrite(PARA_SaveState, sgSaveState.u16Data, 1);	
	}
	else if (((1 == sgSwiInput.b1HeightSpdlimit)&&(0 == SwiInput.b1HeightSpdlimit))
		&&(inserted_data[0] * PROP_CURRENT_FACOTR) > (sgUserInfo.fPropMinCurrent)
	)
	{
		SwiInput.b1HeightLimitState = 0;
		sgSwiInput.b1HeightLimitState = 0;
		sgSaveState.b1HeightSpdLimit = 0;
		u16EepromWrite(PARA_SaveState, sgSaveState.u16Data, 1);	
	}
	
	if ((0 == sgSwiInput.b1HeightLimit ) &&(1 == SwiInput.b1HeightLimit ) && (1 == sgActLogic.b1LiftUpAct))//1800mm限速
	{
		sgSwiInput.b1H180Swi = 1;
		SwiInput.b1H180Swi = 1;
		sgSaveState.b1Above1M8 = 1;
		u16EepromWrite(PARA_SaveState, sgSaveState.u16Data, 1);	
	}
	else if (((1 == sgSwiInput.b1HeightLimit )&&(0 == SwiInput.b1HeightLimit))
		&&(inserted_data[0] * PROP_CURRENT_FACOTR) > (sgUserInfo.fPropMinCurrent)
	)
	{
		SwiInput.b1H180Swi = 0;
		sgSwiInput.b1H180Swi = 0;
		sgSaveState.b1Above1M8 = 0;
		u16EepromWrite(PARA_SaveState, sgSaveState.u16Data, 1);	
	}
	
	SwiInput.b1HeightLimitState = sgSwiInput.b1HeightLimitState;
	
	/*add lock */
	/*清空开关相关错误*/
	if((0 == SwiInput.b1Ems)
		&&(0 == (sgKeyInfo.u16Data & 0x10F6))
		&&(0 == gCanRevPdoInfo.CanRevInfo1E0.i16AuxiliaryThrottle)
		&&(0 == gCanRevPdoInfo.CanRevInfo1E0.i16LiftThrottle)
		&&(0 == gCanRevPdoInfo.CanRevInfo1E0.i16MoveThrottle))
	{
		if(0 ==SwiInput.b1SafeLock)
		{
			i32ErrCodeClr(ACT_INIT_ERR);
			sgLock.Action.b1Init = 0;		
			i32ErrCodeClr(EMS_ERROR);
			sgLock.Action.b1EmsError = 0;			
		}
		i32ErrCodeClr(SWI_SEQUENCE_ERR1);
		i32ErrCodeClr(SWI_SEQUENCE_ERR2);
		i32ErrCodeClr(SWI_SEQUENCE_ERR3);

		sgLock.Action.b1LogicSequnce = 0;
		sgLock.Action.b1PedalLock = 0;
	}
	
	/*操作顺序错误*/
	//SwiInput为当前开关量状态 sgSwiInput为历史开关量状态
	if((0 == sgSwiInput.b1SafeLock)
		&&(0 == sgActLogic.b1ZhiLiState))//非直立行走状态，历史使能按键为0
	{
		
		if((0 != gCanRevPdoInfo.CanRevInfo1E0.i16MoveThrottle))//当前按下前进后退后再按使能
		{
			sgLock.Action.b1LogicSequnce = 1;
			i32ErrCodeSet(SWI_SEQUENCE_ERR1);
			if(1 == SwiInput.b1SafeLock)
			{
				i32ErrCodeSet(SWI_SEQUENCE_ERR2);
			}
		}
	}
	if((1 == SwiInput.b1SafeLock)&&(1 == sgActLogic.b1ZhiLiState))
	{
		sgLock.Action.b1LogicSequnce = 1;
		i32ErrCodeSet(SWI_SEQUENCE_ERR3);
	}
	if(1 == SwiInput.b1Ems)
	{
		i32ErrCodeSet(EMS_ERROR);
		if(1 == i32ErrCodeCheck(ErrCode40))//检测到MCU的禁止动作
		{
			sgLock.Action.b1EmsError = 1;
		}
	}

	
//	if((1 == sgSwiInput.b1SafeLock)
//		&&((1 == sgSwiInput.b1Forward)||(1 == sgSwiInput.b1Backward)))
//	{
//		if((0 == SwiInput.b1SafeLock)
//		&&((1 == SwiInput.b1Forward)||(1 == SwiInput.b1Backward)))
//		{
//			sgLock.Action.b1LogicSequnce = 1;
//			i32ErrCodeSet(SWI_SEQUENCE_ERR);
//		}
//	}
//	if(1 == sgUserInfo.b1LiftLock)
//	{
//		if((0 == sgSwiInput.b1SafeLock)
//			&&(1 ==SwiInput.b1SafeLock)
//			&&((1 == SwiInput.b1LiftUp)
//			||(1 == SwiInput.b1LeftMove)
//			||(1 == SwiInput.b1Tilt)))
//		{
//			sgLock.Action.b1LogicSequnce = 1;
//			i32ErrCodeSet(SWI_SEQUENCE_ERR);			
//		}
//	}

	/*踏板护栏逻辑*/
	{
		if(1 == (SwiInput.b1Pedal))//人站踏板
		{
			sgLock.SpeedRate.b1Spd2Walk = 0;
			if(2 == (SwiInput.b1Fence1 + SwiInput.b1Fence2))//踏板+双护臂，不限速，起升1.8m
			{
				if((0 == gCanRevPdoInfo.CanRevInfo1E0.i16AuxiliaryThrottle)
					&&(0 == gCanRevPdoInfo.CanRevInfo1E0.i16LiftThrottle)
					&&(0 == gCanRevPdoInfo.CanRevInfo1E0.i16MoveThrottle))//解除限制条件
				{				
					sgLock.Action.b1SafeLimit = 0;
					sgLock.Movement.b1FenceLimit = 0;
					sgLock.Lift.b1HeightLimit = 0 ;
				}
				sgLock.SpeedRate.b1Pedal = 0;
				sgLock.Lift.b1H1800 = sgSaveState.b1Above1M8;
			}
			else if(0 == (SwiInput.b1Fence1 + SwiInput.b1Fence2))//踏板+无护臂，限速1，起升不受限
			{
				if((0 == gCanRevPdoInfo.CanRevInfo1E0.i16AuxiliaryThrottle)
					&&(0 == gCanRevPdoInfo.CanRevInfo1E0.i16LiftThrottle)
					&&(0 == gCanRevPdoInfo.CanRevInfo1E0.i16MoveThrottle))//解除限制条件
				{
					
					sgLock.Action.b1SafeLimit = 0;
					sgLock.Lift.b1HeightLimit = 0;
					sgLock.Movement.b1FenceLimit = 0;
				}
				sgLock.SpeedRate.b1Pedal = 1;
				sgLock.Lift.b1H1800 = 0 ;
			}
			else//踏板+单护臂，不行走，不起升
			{
				sgLock.SpeedRate.b1Spd2Walk = 0;
				sgLock.Action.b1SafeLimit = 1;
				sgLock.Lift.b1HeightLimit = 1;
			}
		}
		else 
		{
			sgLock.SpeedRate.b1Pedal = 0;
			if(2 == (SwiInput.b1Fence1 + SwiInput.b1Fence2  ))
			{//踏板收，护臂开,不行走，起升1.8m
				if((0 == gCanRevPdoInfo.CanRevInfo1E0.i16AuxiliaryThrottle)
					&&(0 == gCanRevPdoInfo.CanRevInfo1E0.i16LiftThrottle)
					&&(0 == gCanRevPdoInfo.CanRevInfo1E0.i16MoveThrottle))//解除限制条件
				{
					sgLock.Action.b1SafeLimit = 0;
					sgLock.Lift.b1HeightLimit = 0;		
				}
				sgLock.Movement.b1FenceLimit = 1;
				sgLock.Lift.b1H1800 = sgSaveState.b1Above1M8;
			}
			else if(0 == (SwiInput.b1Fence1 + SwiInput.b1Fence2  ))
			{//踏板收，护臂收，限速2，起升不受限
				if((0 == gCanRevPdoInfo.CanRevInfo1E0.i16AuxiliaryThrottle)
					&&(0 == gCanRevPdoInfo.CanRevInfo1E0.i16LiftThrottle)
					&&(0 == gCanRevPdoInfo.CanRevInfo1E0.i16MoveThrottle))//解除限制条件
				{
					sgLock.Action.b1SafeLimit = 0;
					sgLock.Lift.b1HeightLimit = 0;
					sgLock.Movement.b1FenceLimit = 0;		
				}					
				sgLock.SpeedRate.b1Spd2Walk = 1;
				sgLock.Lift.b1H1800 = 0;
			}
			else
			{
				sgLock.SpeedRate.b1Spd2Walk = 1;
				sgLock.Action.b1SafeLimit = 1;
				sgLock.Lift.b1HeightLimit = 0;					
			}
		}
	}
	if(SwiInput.b1Pedal != sgSwiInput.b1Pedal)
	{
		sgLock.Action.b1PedalLock = 1;
	}
	
	if(1 == sgUserInfo.b1SpeedLimitAfterLift)//起升限速bit7
	{
		sgLock.SpeedRate.b1Spd3Lift = sgSaveState.b1HeightSpdLimit;
	}
	
	if(1 == SwiInput.b1LiftLimit)//起升限制开关触发，开关暂时有问题
	{
		sgLock.Lift.b1HeightLimit = 1;
	}
	
	if(0 != sgLock.Action.u16Data )//有禁止命令时，禁止行走和起升
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
	
	/*直立行走动作处理*/
	{

		uint16_t u16ZhiliTime = 0 ;//直立行走时间设置
		uint8_t u8nolimitflag= 0;
		
		u16ZhiliTime = i32GetPara(PARA_AccAndDecAntiPinch);
		
		/*直立行走时间设置，上位机参数39-防夹手周期*/
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
		
		if(1 == sgKeyInfo.b1ZhiLi)
		{
			if(( 0 == SwiInput.b1SafeLock))
			{
				if( u16ZhiLiCnt < 200/USER_ECU_PERIOD)
				{
					u16ZhiLiCnt ++;
				}
				else if((u16ZhiLiCnt < u16ZhiliTime/USER_ECU_PERIOD)||(u8nolimitflag))//进入直立行走状态
				{
					if(0 != gCanRevPdoInfo.CanRevInfo1E0.i16MoveThrottle)
						u16ZhiLiCnt ++;
					
					sgLock.SpeedRate.b1Spd4Zhili = 1;
					sgActLogic.b1ZhiLiState = 1;
					gCanSendPdoInfo.CanSend260Info.b1Upright = 1;
				}
				else//超时退出退出
				{
					sgActLogic.b1ZhiLiState = 0;
					sgLock.SpeedRate.b1Spd4Zhili = 0;
					gCanSendPdoInfo.CanSend260Info.b1Upright = 0;					
				}
			}
			else//按下使能后退出
			{
				u16ZhiLiCnt = 0;
				sgActLogic.b1ZhiLiState = 0;
				gCanSendPdoInfo.CanSend260Info.b1Upright = 0;		
				sgLock.SpeedRate.b1Spd4Zhili = 0;				
			}
		}
		else//松开直立开关退出
		{
			u16ZhiLiCnt = 0;
			sgActLogic.b1ZhiLiState = 0;
			gCanSendPdoInfo.CanSend260Info.b1Upright = 0;		
			sgLock.SpeedRate.b1Spd4Zhili = 0;
		}
	}
		
	if(0 == sgLock.Movement.u16Data)//行走
	{
		if((0 == sgKeyInfo.b1NeutralPose)
			&&(0 != gCanRevPdoInfo.CanRevInfo1E0.i16MoveThrottle)
			&&((1 == SwiInput.b1SafeLock)||(1 == sgActLogic.b1ZhiLiState)))
		{
			if(gCanRevPdoInfo.CanRevInfo1E0.i16MoveThrottle>0)
				sgActLogic.b1ForwardAct = 1;
			else
				sgActLogic.b1BackwardAct = 1;			
		}
		
		if(1 == sgKeyInfo.b1SnailRequest)
			sgLock.SpeedRate.b1Spd3Slow =1;
		else
			sgLock.SpeedRate.b1Spd3Slow =0;
		
		if(1 == sgKeyInfo.b1ZhiLi)
			sgLock.SpeedRate.b1Spd4Zhili = 1;
		else
			sgLock.SpeedRate.b1Spd4Zhili = 0;
	}
	
	sgLock.PumpRate.u8Data = 0;//油泵速度只受动作限制
	
	/*油泵相关动作处理*/
	{
		if(1 == SwiInput.b1SafeLock)
		{
			if(0 == sgLock.Lift.u16Data)//起升
			{
				if(1 == sgKeyInfo.b1Lift1)
				{
					sgActLogic.b1LiftUpAct = 1;
				}
			}

			if(0 == sgLock.Action.u16Data)//下降、侧移，收到禁止动作时禁止
			{
				if(1 == sgKeyInfo.b1Lift2)
				{
					sgActLogic.b1LeftMove = 1;
					sgLock.PumpRate.b1Horizontal = 1;
				}
				if(1 == sgKeyInfo.b1LiftDown2)
				{
					sgActLogic.b1RightMove = 1;
					sgLock.PumpRate.b1Horizontal = 1;
				}
				if(1 == sgKeyInfo.b1LiftDown1)
				{
					sgActLogic.b1LiftDownAct = 1;
				}		
			}
		}
		/*车辆选型*/
		/*车辆类型更换开关触发状态*/
		if(0 == sgUserInfo.b1CarType)//
		{
			sgLock.PumpRate.b1Horizontal = 0;

			if(1 == sgActLogic.b1LeftMove)
			{
				if(0 == sgLock.Lift.u16Data)
					sgActLogic.b1LiftUpAct  = 1;
				sgActLogic.b1LeftMove = 0;
			}
			if(1 == sgActLogic.b1RightMove)
			{
				sgActLogic.b1LiftDownAct = 1;
				sgActLogic.b1RightMove = 0;
			}
		}
	}
	
	/*多个动作优先级&报警处理*/
	#if 1
	{
		static xActLogic ActionRecord;//实际油泵输出
		
		if((0 !=(sgActLogic.u8Data[1] & (sgActLogic.u8Data[1] -1)))
			||((sgKeyInfo.b1Lift1 & sgKeyInfo.b1Lift2)//24.3.12同时起升下降
				||(sgKeyInfo.b1LiftDown1 & sgKeyInfo.b1LiftDown2))
			)//复数个油泵逻辑请求
		{
			i32ErrCodeSet(MUTL_PUMP_REQ_ERR);
			sgActLogic.u8Data[1] = 0;//清空油泵动作
			sgLock.Action.b1MultiPump = 1;
		}
		if((0 == sgKeyInfo.b1Lift1)
			&&(0 == sgKeyInfo.b1Lift2)
			&&(0 == sgKeyInfo.b1LiftDown1)
			&&(0 == sgKeyInfo.b1LiftDown2))//复位清除警报
		{
			i32ErrCodeClr(MUTL_PUMP_REQ_ERR);
			sgLock.Action.b1MultiPump = 0;
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
	#endif
	


	sgSwiInput.u16data = SwiInput.u16data;
	KeyInfoRecord.u16Data = sgKeyInfo.u16Data;
	
	gCanSendPdoInfo.CanSend261Info.b1LiftDown = sgActLogic.b1LiftDownAct;
//	gCanSendPdoInfo.CanSend261Info.b1SnailMonde = (0 != sgLock.SpeedRate.u8Data[0]);

}

static void vMstRevProc(xMstRevPara *RevData)
{	
	uint16_t tmp = 0;
	
	if(0 != RevData->b1MainDriver)
	{
		Info2F8X4.b1MainContactor = 1;
		gCanSendPdoInfo.CanSend260Info.b1MainContactor = 1;
	}
	else
	{
		Info2F8X4.b1MainContactor = 0;
		gCanSendPdoInfo.CanSend260Info.b1MainContactor = 1;
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
	Info2F8X3.u8MotorRotateSpeedL = (i16MotorSpd) &0xFF;
	Info2F8X3.u8MotorRotateSpeedH = (i16MotorSpd>>8) &0xFF;	

	tmp = i16MotorSpd / sgUserInfo.u16RatioOfTransmission;
	Info2F8X1.u8SpeedH = (tmp>>8) & 0xFF;
	Info2F8X1.u8SpeedL = tmp & 0xFF;
	/*Motor Speed*/
	__enable_irq();
	
//	i32LogWrite(DEBUG, LOG_USER, "Motor Rev Spd = %d\r\n", i16MotorSpd);
	__disable_irq();
	tmp = i16MotorSpd / sgUserInfo.u16RatioOfTransmission;
	/*Motor Speed*/
//	gCanSendPdoInfo.CanSend33CInfo.u8SpdHighByte = tmp >> 8;
//	gCanSendPdoInfo.CanSend33CInfo.u8SpdLowByte = tmp;
	__enable_irq();
	
	tmp = RevData->u8MotorTmp;
	Info2F8X1.u8MotorTemperature = tmp + 15;//范围-55到200
	/*Motor Temp*/
//	gCanSendPdoInfo.CanSend43CInfo.i16MotorTemp = RevData->u8MotorTmp;
	
	tmp = RevData->u8BoardTmp;
	Info2F8X1.u8BoardTemperature = tmp + 15;
	
	
	tmp = RevData->u8CurrentLow | (uint16_t)(RevData->u8CurrentHigh << 8);
	tmp = tmp * 10;
	Info2F8X1.u8MotorCurrentL = tmp;
	Info2F8X1.u8MotorCurrentH = tmp >> 8;
	/**/
//	gCanSendPdoInfo.CanSend43CInfo.i16CtrlTemp = RevData->u8BoardTmp;
	
	__disable_irq();
//	gCanSendPdoInfo.CanSend43CInfo.i16MotorCurrent = (RevData->u8CurrentHigh << 8) | RevData->u8CurrentLow;
	__enable_irq();
	
}



/*********************************输出部分*********************************/

/*发送给Mst的回调函数*/
static void vMstSendProc(xMstSendPara *SendData) 
{
	
	static xActLogic sgLastAction;
	static uint8_t u8MainConnectCnt = 0;
	
	float fTmp = 0.0;
	xMstSendStat LastStatus;
	
	int16_t i16Spd = 0;
	uint8_t u8pump = 0;
	int32_t i32PropValue = 0;
	
	i32PropValue = _IQ((sgUserInfo.fPropMinCurrent + (sgUserInfo.fPropMaxCurrent - sgUserInfo.fPropMinCurrent) * u8PropVal / PUMP_RANGE) / PROPD_STD_CURRENT);
	
	LastStatus.u8Data = SendData->buf[2];
	SendData->buf[2] = 0;	
	if(1 == sgSwiInput.b1Ems)
		SendData->b1EmsReq = 1;
	
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
	if((1 == sgActLogic.b1LiftUpAct)
		||(1 == sgActLogic.b1LeftMove)
		||(1 == sgActLogic.b1RightMove))//存在油泵指令
	{
			u8pump = u8PumpVal;
	}
	else
		u8pump = 0;

	if (u8pump >= PUMP_MAX_VALUE)
	{
		u8pump = PUMP_MAX_VALUE;
	}
	/*开关阀处理*/
	vPropSetTarget(LIFTDOWN_VALVE, 0);
	i32DoPwmSet(LIFTUP_VALVE,DRIVER_CLOSE);
	i32DoPwmSet(LEFT_MOVE_VALVE,DRIVER_CLOSE);
	i32DoPwmSet(RIGHT_MOVE_VALVE,DRIVER_CLOSE);
	
	if(1 ==sgActLogic.b1LiftUpAct)
	{
		i32DoPwmSet(LIFTUP_VALVE,DRIVER_OPEN);
	}
	if(1 == sgActLogic.b1LeftMove)
	{
		i32DoPwmSet(LEFT_MOVE_VALVE,DRIVER_OPEN);
	}
	if(1 == sgActLogic.b1RightMove)
	{
		i32DoPwmSet(RIGHT_MOVE_VALVE,DRIVER_OPEN);
	}
	if(1 == sgActLogic.b1LiftDownAct)
	{
		vPropSetTarget(LIFTDOWN_VALVE, i32PropValue);
		i32DoPwmSet(LIFTUP_VALVE,DRIVER_OPEN);
	}
	
	sgLastAction = sgActLogic;//更新历史动作
	
	/*指令发送*/
	if (0 != i16Spd)
	{
		if(1 == sgUserInfo.b1ForwardBackwardReverse)//反向
		{
			if (1 == sgActLogic.b1ForwardAct)
			{
				gCanSendPdoInfo.CanSend260Info.b1ForwardState = 1;
				gCanSendPdoInfo.CanSend260Info.b1BackwardState = 0;
				SendData->b1BackwardReq = 1;
			}
			else if (1 == sgActLogic.b1BackwardAct)
			{
				gCanSendPdoInfo.CanSend260Info.b1ForwardState = 0;
				gCanSendPdoInfo.CanSend260Info.b1BackwardState = 1;
				SendData->b1ForwardReq = 1;
			}
		}
		else
		{
			if (1 == sgActLogic.b1ForwardAct)
			{
				gCanSendPdoInfo.CanSend260Info.b1ForwardState = 1;
				gCanSendPdoInfo.CanSend260Info.b1BackwardState = 0;
				SendData->b1ForwardReq = 1;
			}
			else if (1 == sgActLogic.b1BackwardAct)
			{
				gCanSendPdoInfo.CanSend260Info.b1ForwardState = 0;
				gCanSendPdoInfo.CanSend260Info.b1BackwardState = 1;
				SendData->b1BackwardReq = 1;
			}			
		}
	}
	

	if(0 != u8pump)
		SendData->b1LiftReq = 1;
	
	
	SendData->u8TargetHigh = i16Spd >> 8;
	SendData->u8TargetLow = i16Spd;
	
	SendData->u8PumpTarget = u8pump;
	
	if((0 != i16Spd)||(0 != u8pump))
		SendData->b1ServoOn =1;


		
	/*lilu 20230819 For Test*/
	{		
		i32SetPara(PARA_AngleValue, i32PropValue);		/*AI1*/
		i32SetPara(PARA_PressureVlaue1, (uint16_t)i32LocalAiGetValue(AI_B_AI2_R));	/*AI2*/
		i32SetPara(PARA_PressureVlaue2, (uint16_t)i32LocalAiGetValue(AI_B_AI3_R));	/*AI3*/
		
		i32SetPara(PARA_LoadRate, sgLock.Lift.u16Data);				/*Valve NoFlag*/
		i32SetPara(PARA_CalibrationStatus,sgLock.Movement.u16Data);		/*Value State*/
		i32SetPara(PARA_ForwardValveCurrent, sgLock.SpeedRate.u8Data[0]);		/*Send Motor Value*/
		i32SetPara(PARA_BackValveCurrent, sgLock.SpeedRate.u8Data[1]);		/*Rev Motor Value*/
		i32SetPara(PARA_LiftValveCurrent, sgLock.SpeedRate.u8Data[2]);		/*Send Pump Value*/
		i32SetPara(PARA_PropValveCurrent,sgLock.SpeedRate.u8Data[3]);	/*Prop Current*/
		
		i32SetPara(PARA_OnOffValveCurrent, sgLock.SpeedRate.u8Data[4]);			/*Send Status*/
		i32SetPara(PARA_TurnRightValveCurrent,sgActLogic.u8Data[1]);
		i32SetPara(PARA_TurnLeftValveCurrent, sgActLogic.u8Data[0]);
		i32SetPara(PARA_ExtSignal, sgSwiInput.u16data>>8);						/*Swi*/
		i32SetPara(PARA_PcuKeyInfo,sgSaveState.u16Data);									/*ErrCode*/
//		i32SetPara(PARA_BmsSoc, (gCanSendPdoInfo.CanSend261Info.u8Soc *2.5));		/*BMS SOC*/
		i32SetPara(PARA_ErrCode,u8ErrCodeGet());
	}
	
	/*正式程序使用*/
	#if 0
	{
	
	}
	#endif
}
static void vID261SendProc(uint16_t u16ID,uint8_t *ID261State)
{
	static uint8_t u8TestCnt = 0;
	if(u8TestCnt<200)
		u8TestCnt++;
	else
		u8TestCnt = 0;
	if(0x261 == u16ID )
	{
		if(u8TestCnt % 2)
			ID261State[0] = ID261State[0];
		else
			ID261State[0] = ID261State[0]+1;
	}
}

static void vID2F8SendProc(uint16_t u16ID,uint8_t *ID2F8State)
{
	static uint8_t u8cnt = 0;
	
	if(u8cnt<7)
		u8cnt++;
	else
		u8cnt = 0;
	
	switch(u8cnt)//测试，走特殊报文发送
	{
		case 0:
			memcpy((char*)ID2F8State,(char*)Info2F8X1.u8Data, sizeof(Info2F8X1));
			break;
		case 1:
			memcpy((char*)ID2F8State,(char*)Info2F8X2.u8Data, sizeof(Info2F8X2));
			break;
		case 2:
			memcpy((char*)ID2F8State,(char*)Info2F8X3.u8Data, sizeof(Info2F8X3));
			break;
		case 3:
			memcpy((char*)ID2F8State,(char*)Info2F8X4.u8Data, sizeof(Info2F8X4));
			break;
		case 4:
			memcpy((char*)ID2F8State,(char*)Info2F8X5.u8Data, sizeof(Info2F8X5));
			break;
		case 5:
			memcpy((char*)ID2F8State,(char*)Info2F8X6.u8Data, sizeof(Info2F8X6));
			break;
		default:
			break;
	}
}
static void vID2F9SendProc(uint16_t u16ID,uint8_t *ID2F9State)
{
	memcpy((char*)ID2F9State, (char*)Info2F9X2.u8Data, sizeof(Info2F9X2));
}

static void vID2FASendProc(uint16_t u16ID,uint8_t *ID2FAState)
{
	static uint8_t u8cnt = 0;
	
	if(u8cnt<2)
		u8cnt++;
	else
		u8cnt = 0;
	switch(u8cnt)
	{
		case 0:
			memcpy((char*)ID2FAState, (char*)Info2FAX1.u8Data, sizeof(Info2FAX1));
			break;
		case 1:
			memcpy((char*)ID2FAState, (char*)Info2FAX2.u8Data, sizeof(Info2FAX2));
			break;
		default:
			break;
	}
}


#if 1
static void vCanID1B0Proc(tCanFrame *CanFrame)
{
	static xCanRev1B0Info CanRev1B0InfoLast;
	
	memcpy((char*)CanRev1B0InfoLast.u8Data, (char*)CanFrame->u8Data, sizeof(CanRev1B0InfoLast));
	
	if((0 == sgUserInfo.b1CardState1)
		&&(0 != sgUserInfo.b1CardState2))//01,不刷卡可行走，刷卡可起升
	{
		sgLock.Action.b1Card = 0;
		if((0 != CanRev1B0InfoLast.b1CardState)
			&&(0 == CanRev1B0InfoLast.b1ErrorState))
		{
			sgLock.Lift.b1Card = 1;
		}
		else
		{
			sgLock.Lift.b1Card = 0;
		}
	}
	else if((0 != sgUserInfo.b1CardState1)
				&&(0 == sgUserInfo.b1CardState2))//10，不刷卡不能动作，刷卡可动作
	{
		if((0 != CanRev1B0InfoLast.b1CardState)
			&&(0 == CanRev1B0InfoLast.b1ErrorState))
		{
			sgLock.Action.b1Card = 1;
		}
		else
		{
			sgLock.Action.b1Card = 0;
		}
	}
	else//其他勾选参数方式，不起限制作用
	{
		sgLock.Action.b1Card = 0;
		sgLock.Lift.b1Card = 0;
	}
}
static void vCanId270Proc(tCanFrame *CanFrame)
{
	static xCanRev270Info CanRev270InfoLast;
//	if((0 != memcmp((char*)CanRev270InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo270.u8Data, sizeof(CanRev270InfoLast)))
//		||(0 != memcmp((char*)CanRev27AInfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo27A.u8Data, sizeof(CanRev27AInfoLast))))
	{
		memcpy((char*)CanRev270InfoLast.u8Data, (char*)CanFrame->u8Data, sizeof(CanRev270InfoLast));

		if((1 == CanRev270InfoLast.b1RemoteOpen)&&(0 == CanRev270InfoLast.b1RemoteClose))//暂未处理
		{
			gRemoteInfo1.b1RemoteEn = 1;
		}
		else
		{
			gRemoteInfo1.b1RemoteEn = 0;
		}
		if(0 != CanRev270InfoLast.b1Lock)
		{
			gRemoteInfo1.b1NoAct  = 1;
		}
		else
		{
			gRemoteInfo1.b1NoAct  = 0;
		}
		if(0 != CanRev270InfoLast.b1LiftLock)
		{
			gRemoteInfo1.b1Nolift  = 1;
		}
		else
		{
			gRemoteInfo1.b1Nolift  = 0;
		}
		if(0 != CanRev270InfoLast.b1LowSpeed1)
		{
			gRemoteInfo1.b1SpeedLimit1  = 1;
		}
		else
		{
			gRemoteInfo1.b1SpeedLimit1  = 0;
		}
		if(0 != CanRev270InfoLast.b1LowSpeed2)
		{
			gRemoteInfo1.b1SpeedLimit2  = 1;
		}
		else
		{
			gRemoteInfo1.b1SpeedLimit2  = 0;
		}
	}
	u8ID270Lostcnt = 0;
}

static void vCanId27AProc(tCanFrame * CanFrame)
{
	static xCanRev27AInfo CanRev27AInfoLast;
	if(0 != memcmp((char*)CanRev27AInfoLast.u8Data, (char*)CanFrame->u8Data, sizeof(CanRev27AInfoLast)))
	{
		memcpy((char*)CanRev27AInfoLast.u8Data, (char*) CanFrame->u8Data, sizeof(CanRev27AInfoLast));
		if((1 == CanRev27AInfoLast.b1RemoteOpen)&&(0 == CanRev27AInfoLast.b1RemoteClose))
		{
			gRemoteInfo2.b1RemoteEn = 1;
		}
		else
		{
			gRemoteInfo2.b1RemoteEn = 0;
		}
		if(0 != CanRev27AInfoLast.b1Lock)
		{
			gRemoteInfo2.b1NoAct  = 1;
		}
		else
		{
			gRemoteInfo2.b1NoAct  = 0;
		}
		if(0 != CanRev27AInfoLast.b1LiftLock)
		{
			gRemoteInfo2.b1Nolift  = 1;
		}
		else
		{
			gRemoteInfo2.b1Nolift  = 0;
		}
		if(0 != CanRev27AInfoLast.b1LowSpeed1)
		{
			gRemoteInfo2.b1SpeedLimit1  = 1;
		}
		else
		{
			gRemoteInfo2.b1SpeedLimit1  = 0;
		}
		if(0 != CanRev27AInfoLast.b1LowSpeed2)
		{
			gRemoteInfo2.b1SpeedLimit2  = 1;
		}
		else
		{
			gRemoteInfo2.b1SpeedLimit2  = 0;
		}
		if((0!=CanRev27AInfoLast.u8SerialNumber12)
			||(0 != CanRev27AInfoLast.u8SerialNumber34)
			||(0 != CanRev27AInfoLast.u8SerialNumber56)
			||(0 != CanRev27AInfoLast.u8SerialNumber78)
			||(0 != CanRev27AInfoLast.u8SerialNumber9A))
		{
			if(gRemotePara.u16SerialNum1234 != ((CanRev27AInfoLast.u8SerialNumber12<<8)|(CanRev27AInfoLast.u8SerialNumber34)))
			{
				gRemotePara.u16SerialNum1234 = ((CanRev27AInfoLast.u8SerialNumber12<<8)|(CanRev27AInfoLast.u8SerialNumber34));
				u16EepromWrite(RemotePara1,gRemotePara.u16SerialNum1234,1);
			
			}
			if(gRemotePara.u16SerialNum5678 != ((CanRev27AInfoLast.u8SerialNumber56<<8)|(CanRev27AInfoLast.u8SerialNumber78)))
			{
				gRemotePara.u16SerialNum5678 = ((CanRev27AInfoLast.u8SerialNumber56<<8)|(CanRev27AInfoLast.u8SerialNumber78));
				u16EepromWrite(RemotePara2,gRemotePara.u16SerialNum5678,1);
			}
			if(gRemotePara.u16SerialNum9A != ((CanRev27AInfoLast.u8SerialNumber9A<<8)))
			{
				gRemotePara.u16SerialNum9A = (CanRev27AInfoLast.u8SerialNumber9A<<8);
				u16EepromWrite(RemotePara3,gRemotePara.u16SerialNum9A,1);
			}
		}
		gRemotePara.u16MaxSpeedRemote = CanRev27AInfoLast.u8MaxSpeed;
	}
	u8ID27ALostcnt = 0;
}
const INT16S K_XISHU[210]=
{
	1,	100,	100,	100,	100,	100,	100,	1,	100,	100,	232,	232,	232,	232,	-4,	-4,	-4,	-4,	-4,	-4,	5,			//0~20
	5,	32767,	32767,	32767,	3,	-2,	-2,	2,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,			//21~41
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,			//42~62
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,			//63~83
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,			//84~104
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,			//105~125
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,																											
};
const INT16S K_CHU_XISHU[210]=
{
	1,	30,	1,	1,	1,	1,	1,	1,	30,	1, 10000,	10000,	10000,	10000,	5,	5,	5,	5,	5,	5,	1,			//0~20
	1,	1000,	1000,	1000,	1,	5,	5,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,			//21~41
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,			//42~62
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,			//63~83
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,			//84~104
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,			//105~125
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,
	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,																											
};
const INT16S B_XISHU[210]=
{
	0,	0,	0,	0,	0,	0,	0,	0,	0,	0, 0,	0,	0,	0,	8200,	8200,	8200,	8200,	8200,	8200,	0,			//0~20
	0,	0,	0,	0,	0,	4200,	4200,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,			//20~40
	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,			//42~62
	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,			//63~83
	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,			//84~004
	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,			//005~025
	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,
	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,																											
};
static	uint32_t u32RecieveAdress = 0;
static	uint16_t u16SendAdress = 0;
static	uint16_t u16Factor = 0;
static void vIndexToAdress(void)
{
		switch(u32RecieveAdress)
	{
		case 0x202401:
			u16SendAdress = 48;
			u16Factor = 1;
		break;
		case 0x202402:
			u16SendAdress = 63;
			u16Factor = 2;
		break;
		case 0x202403:
			u16SendAdress = 66;
			u16Factor = 3;
		break;
		case 0x202404:
			u16SendAdress = 67;
			u16Factor = 4;
		break;
		case 0x202405:
			u16SendAdress = 68;
			u16Factor = 5;
		break;
		case 0x202406:
			u16SendAdress = 69;
			u16Factor = 6;
		break;
		case 0x202407:
			u16SendAdress = 80;
			u16Factor = 7;
		break;
		case 0x202408:
			u16SendAdress = 49;
			u16Factor = 8;
		break;
		case 0x20240F:
			u16SendAdress = 64;
			u16Factor = 9;
		break;
		case 0x202409:
			u16SendAdress = 93;
			u16Factor = 10;
		break;
		case 0x20240A:
			u16SendAdress = 92;		// k=0.01
			u16Factor = 11;;
		break;
		case 0x20240B:
			u16SendAdress = 97;		// k=0.01
			u16Factor = 12;
		break;
		case 0x20240C:
			u16SendAdress = 96;		// k=0.01
			u16Factor = 13;
		break;
		case 0x600001:
			u16SendAdress = 63;		// k=0.01
			u16Factor = 14;			
		break;
		case 0x600002:
			u16SendAdress = 66;		// k=0.01
			u16Factor = 15;			
		break;
		case 0x600003:
			u16SendAdress = 64;		// k=0.01
			u16Factor = 16;			
		break;
		case 0x600004:
			u16SendAdress = 67;		// k=0.01
			u16Factor = 17;
		break;
		case 0x600009:
			u16SendAdress = 68;
			u16Factor = 18;
		break;
		case 0x60000A:
			u16SendAdress = 69;
			u16Factor = 19;
		break;
		case 0x60000C:
			u16SendAdress = 48;
			u16Factor = 20;
		break;
		case 0x60000D:
			u16SendAdress = 49;
			u16Factor = 21;
		break;
		case 0x600201:
			u16SendAdress = 19;
			u16Factor = 22;
		break;
		case 0x600202:
			u16SendAdress = 22;
			u16Factor = 23;
		break;
		case 0x600203:
			u16SendAdress = 20;
			u16Factor = 24;
		break;

		case 0x600501:
			u16SendAdress = 159;
			u16Factor = 25;
		break;
//		case 0x600502:
//		break;
		case 0x600503:
			u16SendAdress = 161;
			u16Factor = 26;
		break;
		case 0x600504:
			u16SendAdress = 160;
			u16Factor = 27;
		break;
		case 0x600505:
			u16SendAdress = 158;
			u16Factor = 28;
		break;
		default:
			u16SendAdress = 0;
			u16Factor = 0;
		break;
	}
}

static void vCanId608Proc(tCanFrame *Canframe)
{
	static uint8_t u8Vector[8];
	uint32_t u32DataReq = 0;
	
	memcpy((char*)CanRev608InfoLast.u8Data, (char*) Canframe->u8Data, sizeof(CanRev608InfoLast));
	u32RecieveAdress = (uint32_t)( (CanRev608InfoLast.u8SubIndex)| (CanRev608InfoLast.u8IndexL<<8) | (CanRev608InfoLast.u8IndexH << 16));
	u32DataReq = (uint32_t)((CanRev608InfoLast.u8DataLL)|(CanRev608InfoLast.u8DataLH << 8));
	
	vIndexToAdress();
	
	u32DataReq = u32DataReq *K_XISHU[u16Factor] /K_CHU_XISHU[u16Factor] + B_XISHU[u16Factor] ;
	u8Vector[0] = CanRev608InfoLast.u8CS;
	u8Vector[1] = u16SendAdress & 0xFF;
	u8Vector[2] = (u16SendAdress >> 8) & 0xFF;
	u8Vector[3] = 0;
	u8Vector[4] = u32DataReq & 0xFF;
	u8Vector[5] = (u32DataReq >> 8) & 0xFF;
	u8Vector[6] = (u32DataReq >> 16) & 0xFF;
	u8Vector[7] = (u32DataReq >> 24) & 0xFF;
	
	vQueryMcuPara(u8Vector,8);
}

//static void vID588SendProc(uint32_t u32ID,uint8_t *ID2FAState)
//{
//	
//}
//static void vID589SendProc(uint32_t u32ID,uint8_t *ID2FAState)
//{
//	
//}
//static void vID594SendProc(uint32_t u32ID,uint8_t *ID2FAState)
//{
//	
//}

static void vMcuParaRevProc(uint8_t *u8Data, uint16_t u16Length)
{
	static uint8_t u8DataArray[8];
	
	memcpy(u8DataArray,u8Data,8);
	uint32_t u32DataSend = 0;
	
	tCanFrame CanSendFrame;
	
	uint8_t i;
	if((0x40 == CanRev608InfoLast.u8CS)&&(0x60 == CanRev608InfoLast.u8IndexH))
	{
		CanSendFrame.u32ID = 0x589;
	}
	else
	{	
		CanSendFrame.u32ID = 0x588;
	}
	CanSendFrame.u8Rtr = 0;
	CanSendFrame.u16DataLength = 8;
	
	u32DataSend = u8DataArray[4]|(u8DataArray[5]<<8)|(u8DataArray[6]<<16)|(u8DataArray[7]<<24);

	u32DataSend = ((u32DataSend -B_XISHU[u16Factor]) * K_CHU_XISHU[u16Factor]) / K_XISHU[u16Factor];
	
	CanSendFrame.u8Data[0] = u8DataArray[0];
	CanSendFrame.u8Data[1] = CanRev608InfoLast.u8IndexL;
	CanSendFrame.u8Data[2] = CanRev608InfoLast.u8IndexH;
	CanSendFrame.u8Data[3] = CanRev608InfoLast.u8SubIndex;
	CanSendFrame.u8Data[4] = u32DataSend & 0xFF;
	CanSendFrame.u8Data[5] = (u32DataSend>>8) & 0xFF;
	CanSendFrame.u8Data[6] = (u32DataSend>>16) & 0xFF;
	CanSendFrame.u8Data[7] = (u32DataSend>>24) & 0xFF;

	i32CanWrite(Can0, &CanSendFrame);

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
	vAiErrReg(vAiErrCallBack);
	
	
	vMcuParaRevRegister(vMcuParaRevProc);
	
	u8CanOpenSendReg(0x261,vID261SendProc);
	u8CanOpenSendReg(0x2F8,vID2F8SendProc);
	u8CanOpenSendReg(0x2F9,vID2F9SendProc);
	u8CanOpenSendReg(0x2FA,vID2FASendProc);
	
	
//	u8CanOpenSendReg(0x588,vID588SendProc);
//	u8CanOpenSendReg(0x589,vID589SendProc);
//	u8CanOpenSendReg(0x594,vID594SendProc);
	
	xRevCallBackProc CanId27A = {.u32CanId = 0x27A, .u32Data = 0, .CallBack = vCanId27AProc};
	xRevCallBackProc CanId608 = {.u32CanId = 0x608, .u32Data = 0, .CallBack = vCanId608Proc};
	xRevCallBackProc CanId614 = {.u32CanId = 0x614, .u32Data = 0, .CallBack = vCanId608Proc};

	vCanRevMsgRegister(&CanId27A);
	vCanRevMsgRegister(&CanId608);
	vCanRevMsgRegister(&CanId614);
	
	
	memset(&sgActLogic, 0, sizeof(sgActLogic));
		sgUserInfo.fPropMinCurrent = i32GetPara(PROP_MIN_CURRENT) / 1000.0;		/*比例阀最小电流*/
	sgUserInfo.fPropMaxCurrent = i32GetPara(PROP_MAX_CURRENT) / 1000.0;		/*比例阀最大电流*/
	
	sgUserInfo.u8PumpMotorGear1 = i32GetPara(PARA_PumpMotorGear1) * PUMP_RANGE / 100;			/*前后移动*/
	sgUserInfo.u8PumpMotorGear2 = i32GetPara(PARA_PumpMotorGear2) * PUMP_RANGE / 100;			/*倾斜*/
	sgUserInfo.u8LiftSpeed = i32GetPara(PARA_LiftSpeed);									/*起升*/
	sgUserInfo.u8MaxTurnSpeed = i32GetPara(PARA_MaxTurnSpeed);
	
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
		sgUserInfo.b1HourConutMode = u16Tmp & 0x01;						/*bit0: HourCount Mode*/
//		sgUserInfo.b1StartUpLock = (u16Tmp >> 1) & 0x01;			/*bit1: StartUpCheck*/
//		sgUserInfo.b1LiftLock = (u16Tmp >> 2) & 0x01;					/*bit2: Lift by Lock */
		sgUserInfo.b1SpeedLimitAfterLift = (u16Tmp>>1) & 0x01;		/*bit1:*/
		sgUserInfo.b1ForwardBackwardReverse = (u16Tmp>>2)&0x01;		/*bit:2:*/		
		sgUserInfo.b1LiftPedal = (u16Tmp >> 3) & 0x01;				/*bit3: Lift by Peadl*/
		sgUserInfo.b1MoveLiftMode = (u16Tmp >> 4) & 0x01;			/*bit4: Move and Lift */
		sgUserInfo.b1SteerCheck = (u16Tmp>>5) & 0x01; 				/*bit5:Steer Controller Online Check*/
		sgUserInfo.b1CarType = (u16Tmp>>6) & 0x01; 						/*bit6:pedal&fence*/


		
		u16Tmp = i32GetPara(RENTAL_INFO);
		sgUserInfo.b1RemoteControl = u16Tmp & 0x01;
		sgUserInfo.b1CardState1 = (u16Tmp >> 1) & 0x01;
		sgUserInfo.b1CardState2 = (u16Tmp>>2)&0x01;
		
	}
	
	u32HourCount = u32HourCountRead();
	
	/*Rental Info*/
	{
		u16EepromRead(RemotePara1,&gRemotePara.u16SerialNum1234,1);
		u16EepromRead(RemotePara2,&gRemotePara.u16SerialNum5678,1);
		u16EepromRead(RemotePara2,&gRemotePara.u16SerialNum9A,1);
	}
	
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
		sgSwiInput.b1H180Swi = sgSaveState.b1Above1M8;
		sgUserInfo.b1PasswordFunc = sgSaveState.b1PasswordFunc;
		u16EepromRead(PARA_HourSetTime, &u16Tmp, 1);	
	}

	
	__disable_irq();
	gCanSendPdoInfo.CanSend261Info.u8WorkTimeLL = u32HourCount & 0xFF;
	gCanSendPdoInfo.CanSend261Info.u8WorkTimeLH = (u32HourCount >> 8) & 0xFF;
	gCanSendPdoInfo.CanSend261Info.u8WorkTimeHL = (u32HourCount >> 16) & 0xFF;
	gCanSendPdoInfo.CanSend261Info.u8WorkTimeHH = (u32HourCount >> 24) & 0xFF;
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
	
	vCanIdLostReg(0x1E0,1000,vCanLostProc);
	vCanIdLostReg(0x361,1000,vCanLostProc);
	
	vSetPdoPara(sgPdoPara);
	
	i32LocalDoSet(DO_ENCODER2, 1);
	
	vSetNetTimer(TIMER_EcuPowerOn, ECU_POWERON_DELAY_TIME);	/**/
	
	vSetNetTimer(TIMER_HourCount, HOURCOUNT_PERIOD_NUOLI);
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
	static uint16_t u16OneMinCnt;

	if(true == u8GetNetTimerOverFlag(TIMER_EcuPowerOn))
	{
		u8EcuProcFlag = 1;
		vKillNetTimer(TIMER_EcuPowerOn);
	}
	
	if ((false == u8GetNetTimerOverFlag(TIMER_EcuPowerOn)) && (true == u8GetNetTimerStartFlag(TIMER_EcuPowerOn)))
	{
		vCanRevPdoProc();
		vSwiInitChcek();
	}
	
	if(1 == u8EcuProcFlag)
	{
		i32SetPara(PARA_PressureVlaue1, u16MotorVal);	/*AI3*/
		vCanRevPdoProc();
		i32SetPara(PARA_PressureVlaue2, u16MotorVal);	/*AI3*/
		vAiMonitor();
		vSwiMonitor();		
		
		u8ErrCode = u8ErrCodeGet();
		
		if (0 != u8ErrCode)
		{
			gCanSendPdoInfo.CanSend261Info.u8ErrorMove = u8ErrSwitchArray[u8ErrCode];
			vLedSendAlmCode(u8ErrCode);
		}
		else
		{
			gCanSendPdoInfo.CanSend261Info.u8ErrorMove = 0;
		}
		
		if(1 == sgUserInfo.b1RemoteControl)
		{
			static uint8_t u8IDLostFlag = 0;
			if(u8ID270Lostcnt < 100)
			{
				u8ID270Lostcnt ++ ;
				u8IDLostFlag &=~(1<<0);
				i32ErrCodeClr(CanIDlost5_ERR);
			}
			else
			{	
				u8IDLostFlag |= (1<<0);
				i32ErrCodeSet(CanIDlost5_ERR);
			}
			
			if(u8ID27ALostcnt < 100)
			{
				u8ID27ALostcnt ++ ;
				u8IDLostFlag &=~(1<<1);
				i32ErrCodeClr(CanIDlost6_ERR);
			}
			else
			{	
				u8IDLostFlag |= (1<<1);
				i32ErrCodeSet(CanIDlost6_ERR);
			}
			
			if((0 != (u8IDLostFlag & 0x01))
				&&(0 != (u8IDLostFlag & (1<<1))))
			{
				sgLock.Lift.b1RemoteIDLost = 1;
				i32ErrCodeSet(MODULE_OFFLINE_ERR);
			}
			else
			{
				sgLock.Lift.b1RemoteIDLost = 0;
				i32ErrCodeClr(MODULE_OFFLINE_ERR);				
			}
		}
	}
	
	if (true == u8GetNetTimerOverFlag(TIMER_HourCount))/*定时器0.1s*/
	{
		vResetNetTimer(TIMER_HourCount);
		
		if (1 == sgUserInfo.b1HourConutMode)
		{
			if ((0 != sgActLogic.u8Data[0])
				||(0 != sgActLogic.u8Data[1])//24.3.12
				)
			{
				u16OneMinCnt++;
			}
		}
		else
		{
			u16OneMinCnt++;
		}
		if(u16OneMinCnt >= 3600)
		{
			u16OneMinCnt = 0;
			u32HourCount++;
			vHourCountWrite(u32HourCount);
		}
	}
	i32SetPara(PARA_BmsSoc, u32HourCount);		/*BMS SOC*/
	if(0 != i32GetPara(PARA_HourCountPowerOn))
	{
		u32HourCount = 0;
		vHourCountWrite(u32HourCount);
		u16SaveParaToEeprom(PARA_HourCountPowerOn,0);
		i32SetPara(PARA_HourCountPowerOn,0);
	}
	__disable_irq();
	gCanSendPdoInfo.CanSend261Info.u8WorkTimeLL = ((u32HourCount * 3600 + u16OneMinCnt) >> 0) & 0xFF;
	gCanSendPdoInfo.CanSend261Info.u8WorkTimeLH = ((u32HourCount * 3600 + u16OneMinCnt) >> 8) & 0xFF;
	gCanSendPdoInfo.CanSend261Info.u8WorkTimeHL = ((u32HourCount * 3600 + u16OneMinCnt) >> 16) & 0xFF;
	gCanSendPdoInfo.CanSend261Info.u8WorkTimeHH = ((u32HourCount * 3600 + u16OneMinCnt) >> 24) & 0xFF;
	__enable_irq();	
	vWdgSetFun(WDG_USER_BIT);
}
#endif
