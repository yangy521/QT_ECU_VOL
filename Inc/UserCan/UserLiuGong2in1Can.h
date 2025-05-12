#ifndef _USER_CAN_LIUGONG_CAN_ECU_
#define _USER_CAN_LIUGONG_CAN_ECU_
#include "stdint.h"


//接收
typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t		u8BMSVoltageL;
		uint8_t		u8BMSVoltageH;
		uint8_t		u8BMSCurrentL;
		uint8_t		u8BMSCurrentH;
		uint8_t		u8BMSSOC;
		uint8_t		u8BMSSOH;
		uint16_t	u8Current;
	};
}xBMSRev052;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t		u8ErrRate1;
		uint8_t		u8ErrRate2;
		uint8_t		u8ErrRate3;
		uint8_t		u8Reserv1;
		uint8_t		u8Reserv2;
		uint8_t		u8Reserv3;
		uint8_t		u8Reserv4;
		uint8_t		u8Reserv5;
	};
}xBMSRev057;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t		u8Reserve1;
		uint8_t		u8Reserve2;
		uint8_t		u8Reserve3;
		uint8_t		u8Reserve4;
		uint8_t		u8Reserve5;
		
		uint8_t		b1ErrVoltageDiff2:1;
		uint8_t		b1ErrTemperDiff2:1;
		uint8_t		b1OutTempH1:1;
		uint8_t		b1OutTempH2:1;
		uint8_t		b1OutTempL1:1;
		uint8_t		b1OutTempL2:1;
		uint8_t		b1OutCurrentH1:1;
		uint8_t		b1OutCurrentH2:1;
		
		uint8_t		b1NormalTempVolL1:1;
		uint8_t		b1LowTempVolL1:1;
		uint8_t		b1NormalTempVolL2:1;
		uint8_t		b1LowTempVolL2:1;
		uint8_t		b1NormalTempSingleVolL1:1;
		uint8_t		b1LowTempSingleVolL1:1;
		uint8_t		b1NormalTempSingleVolL2:1;
		uint8_t		b1LowTempSingleVolL2:1;
		
		uint8_t		u8Reserv8;
	};
}xBMSRev053;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8Reserve1;
		uint8_t u8Reserve2;
		uint8_t u8Reserve3;
		uint8_t u8Reserve4;
		uint8_t	u8DownTime1;
		uint8_t	u8DownTime2;
		uint8_t	u8DownTime3;
		uint8_t	u8DownTime4;
	};
}xCanSend163;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8YearL;
		uint8_t u8YearH;
		uint8_t u8MonthL;
		uint8_t u8MonthH;
		uint8_t u8DateL;
		uint8_t u8DateH;
		uint8_t u8MiniVersionL;
		uint8_t u8MiniVersionH;
	};
}xCanSend165;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8UpdateCountL;
		uint8_t	u8UpdateCountH;
		uint8_t	u8EquipmentSpecifyCodeL;
		uint8_t	u8EquipmentSpecifyCodeH;
		uint8_t	u8EquipmentRecongnizeCode;
		uint8_t	u8VersionCodeBig;
		uint8_t	u8VersionCodeSmall;
		uint8_t	u8AppVersionReleaseMonth;
	};
}xCanSend167;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t	b1MoveForward:1;
		uint8_t b1MoveBackward:1;
		uint8_t	b1LiftUp:1;
		uint8_t	b1LiftDown:1;
		uint8_t	b1TurnRight:1;
		uint8_t	b1TurnLeft:1;
		uint8_t b1Reserve1:1;
		uint8_t	b1MotorEn:1;
		
		uint8_t	b1Trumpet:1;
		uint8_t b1Reserve2:1;
		uint8_t	b1Led:1;
		uint8_t	b1Beep:1;
		uint8_t	b5Reserve:4;
		
		uint8_t	u8MotorSpeedL;
		uint8_t	u8MotorSpeedH;
		uint8_t	u8AngleL;
		uint8_t	u8AngleH;
		uint8_t	u8PressureL;
		uint8_t	u8PressureH;
	};
}xCanSend168;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t	b2Reserve:2;
		uint8_t	b1LowerControlDown:1;
		uint8_t	b1Reserv2:1;
		uint8_t	b1PlatformHighestSwitch:1;
		uint8_t	b1PlatformSafeHeightSwitch:1;
		uint8_t	b2Reserve3:2;
		
		uint8_t b1PitSwitch:1;
		uint8_t	b1ControlModeSwitch:1;
		uint8_t	b1LowerControlUp:1;
		uint8_t	b4Reserve4:4;
		uint8_t	b1TiltSensor:1;
		
		uint8_t	b1SlowSpeedSwitch:1;
		uint8_t	b1TrumpetSwitch:1;
		uint8_t	b1EnableSwitch:1;
		uint8_t	b1TurnRightSwitch:1;
		uint8_t	b1TurnLeftSwitch:1;
		uint8_t	b1LiftModeSwitch:1;
		uint8_t	b1MoveModeSwitch:1;
		uint8_t b1Reserve5:1;
		
		uint8_t u8Reserve6;
		
		uint8_t	u8HandleAnalogL;
		uint8_t	u8HandleAnalogH;
		
		uint8_t b1InitComplete:1;
		uint8_t	b1CommunicateConnect:1;
		uint8_t	b1StableRun:1;
		uint8_t	b1ReleaseBrake:1;
		uint8_t	b1LockWhileError:1;
		uint8_t	b1CarMaintenance:1;
		uint8_t	b1ECUPowerOn:1;
		uint8_t	b1Reserve7:1;
		
		uint8_t u8Reserve8;
	};
}xCanSend169;


typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t	u8BatteryVoltageL;
		uint8_t	u8BatteryVoltageH;
		uint8_t	u8BatterySocL;
		uint8_t	u8BatterySocH;
		
		uint8_t	b1SystemInitFailed:1;
		uint8_t	b1SystemCommunityError:1;
		uint8_t	b1ControllConfigErr:1;
		uint8_t	b1LowerControlSwitchError:1;
		uint8_t	b1PitProteclError:1;
		uint8_t	b1PressureSensorError:1;
		uint8_t	b1AngleSensorErr:1;
		uint8_t	b1TurnLeftSwitchError:1;
		
		uint8_t	b1TurnRightSwitchError:1;
		uint8_t	b1EnableSwitchError:1;
		uint8_t	b1HandleOffCenter:1;
		uint8_t	b1BmsLowVoltage:1;
		uint8_t	b1OverLoade:1;
		uint8_t	b1OverTilt:1;
		uint8_t	b2Reserve:2;
		
		uint8_t	b1ForwardValveError:1;
		uint8_t	b1BackwardValveError:1;
		uint8_t	b1LiftUpValveError:1;
		uint8_t	b1LiftDownValveError:1;
		uint8_t	b1TurnRightValveError:1;
		uint8_t	b1TurnLeftValveError:1;
		uint8_t	b1BrakeValveError:1;
		uint8_t	b1Reserve7:1;	
		
		uint8_t u8Byte7;
	};
}xCanSend170;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8FirmCode;
		uint8_t u8CarType;
		uint8_t	u8CUFirmL;
		uint8_t u8CUFirmH;
		uint8_t	u8MarketAreaL;
		uint8_t	u8MarketAreaH;
		uint8_t u8SafeAuthorityL;
		uint8_t u8SafeAuthorityH;
	};
}xCanSend171;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8HourCntLL;
		uint8_t u8HourCntLH;
		uint8_t	u8HourCntHL;
		uint8_t	u8HourCntHH;
		uint8_t	u8ChargeTimesL;
		uint8_t	u8ChargeTimesH;
		uint8_t	OverLoadTimesL;
		uint8_t	OverLoadTimesH;
	};
}xCanSend172;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8LiftPressureL;
		uint8_t	u8LiftPressureH;
		uint8_t u8EquipmetAngleL;
		uint8_t	u8EquipmetAngleH;
		
		uint8_t b1KeySwiState:1;
		uint8_t b1IndoorState:1;
		uint8_t b1LiftMode:1;
		uint8_t b1SlowSpeed:1;
		uint8_t	b1MoveMode:1;
		uint8_t	b3Reserve:3;
		
		uint8_t u8SpeedAfterLift;
		uint8_t u8SteerSpeed;
		uint8_t	u8SteerSpeedLimit;
	};
}xCanSend173;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t	u8MachineCodeL;
		uint8_t u8MachineCodeH;
		
		uint8_t	b1WeightMeasure:1;
		uint8_t b1AngleSensor:1;
		uint8_t	b1AntiPinchFunction:1;
		uint8_t	b1LiBattery:1;
		uint8_t	b1OutDoorMode:1;
		uint8_t	b1DownLimitSwi:1;
		uint8_t b1UpLimitSwi:1;
		uint8_t	b1HorizonSensorSwi:1;
		
		uint8_t	b1HeightBanMove:1;
		uint8_t b1FunctionalKey:1;
		uint8_t	b1SingleChannelPressureSensor:1;
		uint8_t b1DoubleChannelPressureSensor:1;
		uint8_t	b1PitProtectFunction:1;
		uint8_t	b1ActionAlarmFunction:1;
		uint8_t	b1HighSpeedPump:1;
		uint8_t	b1DownValveType:1;
		
		uint8_t	u8LiftSpeed;
		uint8_t	u8DownSpeed;
		uint8_t	u8NormalSpeed;
		uint8_t	u8SlowSpeed;
	};
}xCanSend174;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8DriverTypeL;
		uint8_t u8DriverTypeH;
		uint8_t	u8LeftMotorSpeedL;
		uint8_t	u8LeftMotorSpeedH;
		uint8_t	u8RightMotorSpeedL;
		uint8_t	u8RightMotorSpeedH;
		uint8_t	u8LeftMotorTempL;
		uint8_t	u8LeftMotorTempH;
	};
}xCanSend175;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t	u8RightMotorTempL;
		uint8_t	u8RightMotorTempH;
		uint8_t	u8MoveMotorTempL;
		uint8_t	u8MoveMotorTempH;
		uint8_t u8PumpMotorTempL;
		uint8_t u8PumpMotorTempH;
		uint8_t	u8EnvironmentTempL;
		uint8_t	u8EnvironmentTempH;
	};
}xCanSend176;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8PumpCurrentL;
		uint8_t u8PumpCurrentH;
		uint8_t	u8LeftMotorCurrentL;
		uint8_t	u8LeftMotorCurrentH;
		uint8_t	u8RightMotorCurrentL;
		uint8_t	u8RightMotorCurrentH;
	};
}xCanSend177;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8LiftTimes1;
		uint8_t u8LiftTimes2;
		uint8_t u8LiftTimes3;
		uint8_t u8LiftTimes4;
		uint8_t u8SteerTimes1;
		uint8_t u8SteerTimes2;
		uint8_t u8SteerTimes3;
		uint8_t u8SteerTimes4;
	};
}xCanSend178;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8MoveTimes1;
		uint8_t	u8MoveTimes2;
		uint8_t u8MoveTimes3;
		uint8_t	u8MoveTimes4;
		uint8_t u8OverLoadTimes1;
		uint8_t u8OverLoadTImes2;
		uint8_t u8OverLoadTimes3;
		uint8_t u8OverLoadTimes4;
	};
}xCanSend179;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8ECUSerialNumber1;
		uint8_t u8ECUSerialNumber2;
		uint8_t u8ECUSerialNumber3;
		uint8_t u8ECUSerialNumber4;
		uint8_t u8ECUSerialNumber5;
		uint8_t u8ECUSerialNumber6;
		uint8_t u8ECUSerialNumber7;
		uint8_t u8ECUSerialNumber8;
	};
}xCanSend17A;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8ECUSerialNumber9;
		uint8_t u8ECUSerialNumber10;
	};
}xCanSend17B;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8PCUSerialNumber1;
		uint8_t u8PCUSerialNumber2;
		uint8_t u8PCUSerialNumber3;
		uint8_t u8PCUSerialNumber4;
		uint8_t u8PCUSerialNumber5;
		uint8_t u8PCUSerialNumber6;
		uint8_t u8PCUSerialNumber7;
		uint8_t u8PCUSerialNumber8;
	};
}xCanSend17C;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8PCUSerialNumber9;
		uint8_t u8PCUSerialNumber10;
	};
}xCanSend17D;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t reserve1;
		uint8_t	reserve2;
		uint8_t	reserve3;
		uint8_t	reserve4;
		
		uint8_t u8BatteryVoltageL;
		uint8_t u8BatteryVoltageH;
		
	};
}xCanSend17E;


typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8LiftTime1;
		uint8_t u8LiftTime2;
		uint8_t	u8LiftTime3;
		uint8_t	u8LiftTime4;
		uint8_t	u8SteerTime1;
		uint8_t	u8SteerTime2;
		uint8_t	u8SteerTime3;
		uint8_t	u8SteerTime4;
	};
}xCanSend050;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8LowerMoveTime1;
		uint8_t u8LowerMoveTime2;
		uint8_t u8LowerMoveTime3;
		uint8_t u8LowerMoveTime4;
		uint8_t u8UpperMoveTime1;
		uint8_t u8UpperMoveTime2;
		uint8_t u8UpperMoveTime3;
		uint8_t u8UpperMoveTime4;
	};
}xCanSend055;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8FactoryCode;//1
		uint8_t	u8ParaSetState;
		uint8_t u8Byte2;
		uint8_t u8Byte3;
		//5
		uint8_t	b1HandleReverseFunction:1;
		uint8_t	b1FouPointWeightFunction:1;
		uint8_t	b1HeartBeatCheckFunction:1;
		uint8_t	b1BrakeRelease:1;
		uint8_t	b1AngleSensorType:1;
		uint8_t	b1DriverType:1;
		uint8_t b2Reserve:2;

	};
}xCanSend190;



typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8ErrorNum;
		uint8_t u8SPN;
		uint8_t u8Reserve;
		uint8_t u8FMI;
		uint8_t u8ErrorState;
		uint8_t u8ErrorCnt;
		uint8_t u8ErrorLocation;
	};
}xCanSend25D;//故障码上报

typedef union
{
	uint8_t u8Data[8];
	struct
	{
	};
}xCanSend45C;//TBOX发送

typedef union
{
	uint8_t u8Data[8];
	struct
	{
	};
}xCanSend55C;//TBOX发送

typedef union
{
	uint8_t u8Data[8];
	struct
	{
	};
}xCanSend55D;//TBOX发送

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8Soc;
		uint8_t u8Reserve2;
		uint8_t u8Reserve3;
		uint8_t u8Reserve4;
		
		uint8_t b1LockState:1;
		uint8_t b1HeartState:1;
		uint8_t b6Reserve:6;
		
		uint8_t u8ControlState;
		
	};
}xCanSend55E;//TBOX发送


typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t		u8Reserve1;
		
		uint8_t		b1Reserve1:1;
		uint8_t		b1UpControl:1;
		uint8_t		b6Reserve2:6;
		uint8_t		u8Reserve3;
		uint8_t 	u8Reserve4;
		uint8_t		u8Reserve5;
		uint8_t		u8Reserve6;
		uint8_t 	u8Reserve7;
		uint8_t 	u8Reserve8;
	};
}xHMISend169;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t		u8Reserve1;
		
		uint8_t		u8Reserve2;
		
		uint8_t		u8SOC;
		uint8_t 	u8Reserve4;
		uint8_t		u8Reserve5;
		uint8_t		u8Reserve6;
		uint8_t 	u8Reserve7;
		uint8_t 	u8Reserve8;
	};
}xHMISend170;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t		u8HourCntLL;
		
		uint8_t		u8HourCntLH;
		
		uint8_t		u8HourCntHL;
		uint8_t 	u8HourCntHH;
		uint8_t		u8Reserve5;
		uint8_t		u8Reserve6;
		uint8_t 	u8Reserve7;
		uint8_t 	u8Reserve8;
	};
}xHMISend172;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t		u8CarType;   //bit0车辆类型
		uint8_t  u8VersionCodeHH;    //车辆类型
		uint8_t  u8VersionCodeHL;
		uint8_t  u8VersionCodeLH;
		uint8_t  u8VersionCodeLL;
		uint8_t		u8Reserve6;
		uint8_t 	u8ErrMode;
		uint8_t 	u8ErrCode;
	};
}xHMISendEE5521;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t		u8Reserve1;
		uint8_t		u8Reserve2;
		uint8_t		u8Reserve3;
		uint8_t 	u8Reserve4;
		
		uint8_t		b1Reserve0:1;
		uint8_t		b1Reserve1:1;
		uint8_t		b1TileErr:1;
		uint8_t		b1OverLoadErr:1;
		uint8_t		b4Reserve3:4;
		
		uint8_t		u8Reserve6;
		uint8_t 	u8Reserve7;
		uint8_t 	u8Reserve8;
	};
}xHMISendEC5521;


typedef struct
{ 
	/*接收PDO1， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData1[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		
	};
	/*接收PDO2， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData2[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		
	};
	/*接收PDO3， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData3[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		
	};
	/*接收PDO4， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData4[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
	};
	/*接收PDO5， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData5[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		xBMSRev052 BMSRev052;
	};
	/*接收PDO6， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData6[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		xBMSRev053	BMSRev053;
		
	};
	/*接收PDO7， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData7[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		xBMSRev057 BMSRev057;
	};
	/*接收PDO8， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData8[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
	};	
	union
	{
		uint8_t u8RevData9[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
	};
	union
	{
		uint8_t u8RevData10[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
	};
	
}xCanRevPdoInfo;

typedef struct
{
	/*发送PDO1， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8SendData1[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
	};
	/*发送PDO2， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8SendData2[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
	};
	/*发送PDO3， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8SendData3[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
		//xCanSend43CInfo CanSend43CInfo;
	};
	/*发送PDO4， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8SendData4[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
		//xCanSend53CInfo CanSend53CInfo;
	};
	/*发送PDO5， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8SendData5[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
	};
	/*发送PDO6， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8SendData6[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
	};
	/*发送PDO7， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8SendData7[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
	};
	/*发送PDO8， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8SendData8[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
	};
	union
	{
		uint8_t u8SendData9[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
	};
	union
	{
		uint8_t u8SendData10[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
	};
}xCanSendPdoInfo;

#endif