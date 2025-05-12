/*******************************************************************************
* Filename: Para.h 	                                         		   		   *
*                                                                              *
* Description: 																   *
* Author:                                                                      *
* Date: 															           *
* Revision:															           *
*******************************************************************************/
#ifndef _PARA_H_
#define _PARA_H_
#include "stdint.h"

typedef enum
{
	PARA_FastDriveSpeed = 0,
	PARA_SlowDriveSpeed = 1,		
	PARA_DriveSpeedAfterLift = 2,	
	PARA_LiftSpeed,				
	PARA_MaxTurnSpeed,			
	PARA_TurnPowerLimit,			
	PAPA_DeadZoneAdjust,			

	PARA_BrakeFastDrive,			
	PARA_BrakeSlowDrive,			
	PARA_BrakeDriveAfterLift,	
	PARA_BrakeLift,				
	PARA_BrakeLower,				
	PARA_BrakeTurn,				
	PARA_BrakeAntiPinch,

	PARA_Reserve1,					/*14*/
	PARA_Reserve2,					/*15*/

	PARA_LowerSpeed,				/*16*/			
	PARA_OverLoadStabilityDelay,	
	PARA_DynamicOverLoadPercent,	
	PARA_StaticOverLoadPercent,	
	PARA_MaxDifferencePercent,	
	PARA_DriveMotorEncoder,		
	PARA_MotorHighSpeedDeceRate,	
	PARA_MotorLowSpeedDeceRate,	
	PARA_VoiceAlarmVolume,		

	PARA_CurveFastDrive,						
	PARA_CurveSlowDrive,			
	PARA_CurveDriveAfterLift,	
	PARA_CurveLift,				
	PARA_CurveLower,
	
	PARA_Reserve3,					/*30*/
	PARA_Reserve4,					/*31*/	

	PARA_CurveTurn,					/*32*/

	PARA_AccAndDecFastDrive,				
	PARA_AccAndDecSlowDrive,		
	PARA_AccAndDecAfterLift,		
	PARA_AccAndDecLift,			
	PARA_AccAndDecLower,			
	PARA_AccAndDecTurn,			
	PARA_AccAndDecAntiPinch,		

	PARA_PumpMotorEncoder,					

	PARA_VehicleType,							
	PARA_VehcileHeight,			
	PARA_PressureSensorType,		
	PARA_PitProtectFunc,			
	PARA_AntiPinchFunc,

	PARA_Reserve5,					/*46*/
	PARA_Reserve6,					/*47*/
	
	PARA_ActAlmFunc,				/*48*/				
	PARA_WeighFunc,				
	PARA_ParallelValveReverseFunc,
	PARA_LowBatAlmFunc,			
	PARA_LowVolAlmTime,			
	PARA_LowVolShutDownTime,		
	PARA_UpperCtlButSleep,		
	PARA_LanguageType,			
	PARA_BatteryType,			
	PARA_SpeakerSync,			
	PARA_LowerPumpType,			
	PARA_PressureType,			
	PARA_AngleSimulationLimit,	
	PARA_LiftReverseFunc,

	PARA_Reserve7,				/*62*/
	PARA_Reserve8,				/*63*/
	
	PARA_FourPointWeightFunc,	/*64*/	
	PARA_DriverType,			
	PARA_PasswordLock,			
	PARA_InAndOutFunc,			
	PARA_HeartBeatQueryFunc,		
	PARA_LowBatteryMode,			
	PARA_AngleSensorSetting,		
	PARA_TiltSwitchSetting,		
	PARA_AngleSensorType,		
	PARA_AnaLogLimitDetSwitch,	

	PARA_IsNoLoadCalibration,					
	PARA_IsOverLoadCalibration,	

	PARA_SetDescentHeightValue,		
	PARA_ReleaseBrake,

	PARA_Reserve9,				/*78*/
	PARA_Reserve10,				/*79*/
	
	PARA_SetOutHeight,			/*80*/
	PARA_AngleSimulationUpLimit,	
	PARA_AngleSimulationDownLimit,
	
	PARA_PumpDriveCurrentLimitRatio0,			
	PARA_PumpSpdAccRatio0,
	PARA_PropDKp0,						
	PARA_PropDKi0,						
	PARA_PropDMaxCurrent0,				
	PARA_PropDMinCurrent0,
	PARA_PropDAccPeriod0,
	PARA_PropDDitherPeriod0,	
	PARA_PropDDitherRatio0,			
	PARA_PropValveResistance0,
	
	PARA_Reserve11,					/*93*/
	PARA_Reserve12,					/*94*/
	PARA_Reserve13,					/*95*/
	
	
	PARA_PumpDriveCurrentLimitRatio1,		/*96*/	
	PARA_PumpSpdAccRatio1,
	PARA_PropDKp1,						
	PARA_PropDKi1,						
	PARA_PropDMaxCurrent1,				
	PARA_PropDMinCurrent1,
	PARA_PropDAccPeriod1,
	PARA_PropDDitherPeriod1,	
	PARA_PropDDitherRatio1,			
	PARA_PropValveResistance1,
	
	PARA_CanBaudRate,
	PARA_EmptyPressure,
	PARA_FullPressure,
	PARA_DriverFlag,
	
	PARA_Reserve14,					/*110*/
	PARA_Reserve15,					/*111*/
	
	PARA_MinAngle,					/*112*/
	PARA_MaxAngle,
	PARA_BatSocPalyBack,
	PARA_ValveType,
	PARA_AnticollisionFunc,
	PARA_ValueOpenLoopCurrent,
	PARA_ValueOpenPercentage,
	PARA_CanOpenNodeId,
	
	PARA_AngleValue0,
	PARA_AngleValue1,
	PARA_AngleValue2,
	PARA_AngleValue3,
	PARA_AngleValue4,
	PARA_AngleValue5,
	PARA_AngleValue6,
	PARA_AngleValue7,
	
	PARA_Driver1Vol,		/*Driver1 High Byte Pull Vol, Low Byte hold Vol*/
	PARA_Driver2Vol,		/*Driver2 High Byte Pull Vol, Low Byte hold Vol*/
	PARA_Driver3Vol,		/*Driver3 High Byte Pull Vol, Low Byte hold Vol*/
	PARA_Driver4Vol,		/*Driver4 High Byte Pull Vol, Low Byte hold Vol*/
	PARA_Driver5Vol,		/*Driver5 High Byte Pull Vol, Low Byte hold Vol*/
	PARA_Driver6Vol,		/*Driver6 High Byte Pull Vol, Low Byte hold Vol*/
	PARA_Driver7Vol,		/*Driver7 High Byte Pull Vol, Low Byte hold Vol*/
	PARA_Driver8Vol,		/*Driver8 High Byte Pull Vol, Low Byte hold Vol*/
	PARA_Driver9Vol,		/*Driver9 High Byte Pull Vol, Low Byte hold Vol*/
	PARA_Driver10Vol,		/*Driver10 High Byte Pull Vol, Low Byte hold Vol*/
	
	PARA_LogLevel,					/*138 log level*/
	PARA_LogModel,					/*139*/
	PARA_MotorMaxSpd,				/*140*/
	
	PARA_Analog3DeadZoneMinVal,		/*141*/
	PARA_Analog3DeadZoneMaxVal,		/*142*/
	PARA_Analog3MidVal,				/*143*/
	
	PARA_ThrottleType,					/*144 加速器类型*/
	PARA_ThrottleFDeadZoneMinVal,		/*踏板正向死区最小值（0.1V）*/
	PARA_ThrottleFDeadZoneMaxVal,		/*踏板正向死区最大值（0.1V）*/
	PARA_ThrottleFMidVal,				/*踏板正向中值对应输出百分比（1%）*/
	PARA_ThrottleBDeadZoneMinVal,		/*踏板反向死区最小值（0.1V）*/
	PARA_ThrottleBDeadZoneMaxVal,		/*踏板反向死区最大值（0.1V）*/
	PARA_ThrottleBMidVal,				/*踏板反向中值对应输出百分比（1%）*/
	
	PARA_BrakeType,						/*制动踏板输入类型*/
	PARA_BrakeFDeadZoneMinVal,			/*制动踏板正向死区最小值（0.1V）*/
	PARA_BrakeFDeadZoneMaxVal,			/*制动踏板正向死区最大值（0.1V）*/
	PARA_BrakeFMidVal,					/*制动踏板正向中值对应输出百分比（1%）*/
	PARA_BrakeBDeadZoneMinVal,			/*制动踏板反向死区最小值（0.1V）*/
	PARA_BrakeBDeadZoneMaxVal,			/*制动踏板反向死区最大值（0.1V）*/
	PARA_BrakeBMidVal,					/*制动踏板反向中值对应输出百分比（1%）*/
	
	PARA_Reserve16,						/*158*/
	PARA_Reserve17,						/*159*/
	
	PARA_Gear1Spd,						/*160 1档速度(%)*/
	PARA_Gear2Spd,						/*2档速度(%)*/
	PARA_Gear3Spd,						/*3档速度(%)*/
	PARA_Gear4Spd,						/*4档速度(%)*/
	
	PARA_RatioOfTransmission,			/*转速公里比*/
	PARA_MaintenancePeriod,				/*维护周期(h)*/
	PARA_RemotePara,					/*远程终端参数*/
	
	PARA_PumpMotorGear1,				/*起升电机中间挡位1(%)*/
	PARA_PumpMotorGear2,				/*起升电机中间挡位2(%)*/
	
	PARA_TurnWithDecStartAngle,			/*转弯降速起始角度*/
	PARA_TurnWithDecEndAngle,			/*转弯降速终止角度*/
	
	PARA_AngleWithStartSpdPer,			/*起始角度速度百分比*/
	PARA_AngleWithEndSpdPer,			/*终止角度速度百分比*/
	
	PARA_HourCountPowerOn,
	
	PARA_Reserve18,					/*174*/
	PARA_Reserve19,					/*175*/
	PARA_Reserve20,					/*176*/
	PARA_Reserve21,					/*177*/
	PARA_Reserve22,					/*178*/
	PARA_Reserve23,					/*179*/
	PARA_Reserve24,					/*180*/
	PARA_Reserve25,					/*181*/
	PARA_Reserve26,					/*182*/
	PARA_Reserve27,					/*183*/
		
	PARA_ErrCode0,					/*184*/										
	PARA_ErrCode1,					
	PARA_ErrCode2,
	PARA_ErrCode3,
	PARA_ErrCode4,
	PARA_ErrCode5,
	PARA_ErrCode6,
	PARA_ErrCode7,
	PARA_ErrCode8,
	PARA_ErrCode9,
	PARA_ErrCode10,
	PARA_ErrCode11,
	PARA_ErrCode12,
	PARA_ErrCode13,
	PARA_ErrCode14,
	PARA_ErrCodeMax,				/*199*/
	
	PARA_UserType,					/*200*/
	
	PARA_AngleValue,				
	PARA_PressureVlaue1,
	PARA_PressureVlaue2,
	PARA_LoadRate,
	PARA_CalibrationStatus,
	PARA_ForwardValveCurrent,
	PARA_BackValveCurrent,
	PARA_LiftValveCurrent,
	PARA_PropValveCurrent,
	PARA_OnOffValveCurrent,
	PARA_TurnRightValveCurrent,
	PARA_TurnLeftValveCurrent,
	PARA_BrakeValveCurrent,
	PARA_BatteryVoltage,			
	PARA_ExtSignal,				
	PARA_PcuKeyInfo,			
	PARA_OutCtrlInfo,				
	PARA_MotorSpd,				
	PARA_HandleAnalog,			
	PARA_BmsSoc,
	PARA_BmsVoltage,
//	PARA_CtrlSNCode,
//	PARA_EcuHardVersion,
//	PARA_EcuSoftVersion,
//	PARA_SoftCompileDate,
//	PARA_SoftCompileTime,
	PARA_TemporaryUnlock,			
	PARA_PlatfromHeartQuery,		
	PARA_SelfHeartQuery,			
	PARA_EcuLockState,			
	PARA_TmpLockState,
	
	PARA_Swi,
	PARA_DoSwi,
	PARA_Ai1,
	PARA_Ai2,
	PARA_Ai3,
	PARA_Ai4,
	PARA_Ai5,
	PARA_Vbus,
	PARA_Ksi,
	PARA_ErrCode,
	PARA_PropCurrent1,
	PARA_PropCurrent2,
	
	PARA_MotorCmd,
	PARA_PumpCmd,
	PARA_MotorState,
	PARA_SteerAngle,
	PARA_MotorSpeed,
	
	//PARA_UserType = 255,
	
	
//	PARA_NoBatteryFlag1,
//	PARA_NoBatteryCount1,
//	PARA_NoBatteryFlag2,
//	PARA_NoBatteryCount2,

	
	PARA_Max,
}eParaNo;


#define PARA_VOLUME_NUM				PARA_UserType
#define PARA_HARDHEAD_NUM				(234)
#define PARA_HARDEND_NUM				(249)
#define EEPROM_PARA_NUM				256

#define	PRM_ATTR_POWEROFF  	(1 << 0)
#define	PRM_ATTR_SIGNED    	(1 << 1)
#define	PRM_ATTR_SIZE(n) 	(sizeof(n) << 2)
#define	PRM_ATTR_POSOFS(n)  ((n & 0x1f) << 4)
#define	PRM_ATTR_BITNUM(n)  (((n-1) & 0x1f) << 9)
#define	PRM_ATTR_TYPE_BIT_SET(ofs,len)  	(PRM_ATTR_TYPE_BIT | PRM_ATTR_POSOFS(ofs) | PRM_ATTR_BITNUM(len))
#define PRM_ATTR_RDPOWER(n)	((n & 0x7) << 14)
#define PRM_ATTR_WRPOWER(n)	((n & 0x7) << 17)
#define PRM_ARRT_EPROM			(1 << 20)
#define PRM_ATTR_TYPE_BIT		(1 << 21)
#define PRM_ARRT_HARD			(1 << 22)


typedef struct
{
	uint32_t	PowerOff : 1;	// 0--No poweroff required, 1--Poweroff required
	uint32_t	Signed : 1; 	// 0--unsigned, 1--signed,
	uint32_t	Size : 2;			// 0--bit, 1--byte, 2--half word, 3--rsv
	uint32_t	PosOfs : 5;		// This para‘s bit offset in para var. 0-bit0, 31-bit31
	uint32_t	BitNum : 5;   // This para‘s bit number. 0~31 - 1~32 bit
	uint32_t	RdPower : 3;   // bit0--一级读权限
	uint32_t	WrPower : 3;   // bit0--一级写权限
	uint32_t	Eprom: 1;   // 
	uint32_t	Bit: 1;   // 
	uint32_t	Hard: 1;   // 
	uint32_t	rev: 9;   // 
}xPrmAttrStruct;

typedef struct
{
	uint32_t	PrmNo;
	uint32_t	PrmAttr;
	int32_t  	PrmMinVal;
	int32_t  	PrmMaxVal;
	int32_t  	PrmInitVal;
	void*	  	pPrmData;
	const char* m_pStrGroup;
}xPrmDefStruct;

typedef enum
{
	LSC0407DEM = 0,
	LSC0507DEM = 1,
	LSC0608DEM = 2,
	LSC0407DE,
	LSC0607DE,
	LSC0808DE,
	LSC0812DE,
	LSC1012DE,
	LSC1212DE,
	LSC0407DH,
	LSC0607DH,
	LSC0607DH_L,
	LSC0608DH,
	LSC0808DH,
	LSC0812DH,
	LSC1012DH,
	LSC1212DH,
	LSC1414DH_1,
	LSC0607DE_2_L,
	LSC0808DE_2,
	LSC1012DE_2,
	LSC1212DE_2,
	LSC0808DE_3_L,
	LSC1012DE_3_L,
	LSC1212DE_3_L,
	LSC1414DE_3_L,
	LSC0507DEM_1_1,
	LSC0607DE_2_1,
	LSC0808DE_2_1,
	LSC1012DE_2_1,
	LSC0812DE_2_1,
	LSC0808DE_3,
	LSC0812DE_3,
	LSC1012DE_3,
	LSC1212DE_3,
	LSC1414DE_3,
	LSC0607DE_2,
//	LSC0808DE_2,
	LSC0607DH_1,
	LSC1414DH_1_L,
	LSC0607DH_1_L,
	LS0607H,
	LS0808H,
	LS0812H,
	LS1012H,
	LS1212H,
	LS0607E,
	LS0808E,
	LS0812E,
	LS1012E,
	LS1212E,	
}eVehicleType;

typedef enum
{
	SingleCylinder = 0,
	DoubleCylinder = 1,
}eCylinderType;

typedef enum 
{
	NoSensor = 0,
	SingleChannelSensor = 1,
	DoubleChannelSensor = 2,
}ePressureSensorType;

typedef enum
{
	FunctionDisable = 0,
	FunctionEnable = 1,
}eFunctionAble;	//23.11.21 SJ修改选项顺序，与上位机一致

typedef enum
{
	ChineseDis = 0,
	EnglishDis = 1,
}eLauguageType;

typedef enum
{
	LiBattery = 0,
	QiuJian = 1,
	LiShi = 2,
	XuPai = 3,
}eBatteryType;

typedef enum
{
	OnOffValve = 0,
	PropValve = 1,
}eLowerPumpType;	//23.11.21 SJ修改选项顺序，便于PCU修改参数设置

typedef enum
{
	CurrentType = 0,
	VoltageType = 1,
}ePressureType;

typedef enum
{
	BLDC = 0,
	SAC = 1,
	DACSDC = 2,
	TAC = 3,
	TDC = 4,
	CURTISDACSDC = 5,
	QPSAC = 6,
	QPDAC = 7,	
}eDriverType;


typedef enum
{
	NoKey = 0,
	Key1 = 1,
	Key2 = 2,
}eKeyType;

typedef enum
{
	DefaultMode = 0,
	OptionalMode = 1,
}eLowBatteryMode;

typedef union
{
	uint16_t u16buf[256];
	struct
	{
		uint16_t	u16FastDriveSpeed;			/*快速行驶速度*/
		uint16_t	u16SlowDriveSpeed;			/*慢速行驶速度*/
		uint16_t	u16DriveSpeedAfterLift;		/*举升后行驶速度*/
		uint16_t	u16LiftSpeed;				/*举升速度*/
		uint16_t	u16MaxTurnSpeed;			/*最大转弯速度*/
		uint16_t	u16TurnPowerLimit;			/*转弯动力限定值*/
		uint16_t	u16DeadZoneAdjust;			/*死区值调节*/
		/*刹车减速值调节*/
		uint16_t	u16BrakeFastDrive;			/*快速行走*/
		uint16_t	u16BrakeSlowDrive;			/*慢速行走*/
		uint16_t	u16BrakeDriveAfterLift;		/*举升后行走*/
		uint16_t	u16BrakeLift;				/*举升*/
		uint16_t	u16BrakeLower;				/*下降*/
		uint16_t	u16BrakeTurn;				/*转向动作*/
		uint16_t	u16BrakeAntiPinch;			/*防夹手缓冲*/
		
		uint16_t	u16Reserve1;				/*14*/
		uint16_t	u16Reserve2;				/*15*/
		
		uint16_t	u16LowerSpeed;				/*下降速度*/
		uint16_t	u16OverLoadStabilityDelay;	/*超载稳定延时*/
		uint16_t	u16DynamicOverLoadPercent;	/*动态超载百分比*/
		uint16_t	u16StaticOverLoadPercent;	/*静态超载百分比*/
		uint16_t	u16MaxDifferencePercent;	/*最大差值百分比*/
		uint16_t	u16DriveMotorEncoder;		/*行走电机编码*/
		uint16_t	u16MotorHighSpeedDeceRate;	/*电机高速减速率*/
		uint16_t	u16MotorLowSpeedDeceRate;	/*电机低速减速率*/
		uint16_t	u16VoiceAlarmVolume;		/*语音报警音量*/
		/*曲线加速值调节*/
		uint16_t	u16CurveFastDrive;			/*快速行走*/
		uint16_t	u16CurveSlowDrive;			/*慢速行走*/
		uint16_t	u16CurveDriveAfterLift;		/*举升后行走*/
		uint16_t	u16CurveLift;				/*举升*/
		uint16_t	u16CurveLower;				/*下降*/
		
		uint16_t	u16Reserve3;				/*30*/
		uint16_t	u16Reserve4;				/*31*/
		
		uint16_t	u16CurveTurn;				/*转向*/
		/*加减速周期调节*/
		uint16_t	u16AccAndDecFastDrive;		/*快速行走*/
		uint16_t	u16AccAndDecSlowDrive;		/*慢速行走*/
		uint16_t	u16AccAndDecAfterLift;		/*举升后行走*/
		uint16_t	u16AccAndDecLift;			/*举升*/
		uint16_t	u16AccAndDecLower;			/*下降*/
		uint16_t	u16AccAndDecTurn;			/*转向*/
		uint16_t	u16AccAndDecAntiPinch;		/*防夹手*/
		
		uint16_t	u16PumpMotorEncoder;		/*泵电机编码*/
		
		uint16_t	u16VehicleType;				/*车辆类型*/	
		uint16_t	u16VehcileHeight;			/*车辆高度*/
		uint16_t	u16PressureSensorType;		/*压力传感器类型*/
		uint16_t	u16PitProtectFunc;			/*坑洞保护功能*/
		uint16_t	u16AntiPinchFunc;			/*防夹手功能*/
		
		uint16_t	u16Reserve5;				/*46*/
		uint16_t	u16Reserve6;				/*47*/
		
		uint16_t	u16ActAlmFunc;				/*动作报警功能*/
		uint16_t	u16WeighFunc;				/*称重功能*/
		uint16_t	u16ParallelValveReverseFunc;/*并联阀反向功能*/
		uint16_t	u16LowBatAlmFunc;			/*低电量报警功能*/
		uint16_t	u16LowVolAlmTime;			/*低电压报警时间*/
		uint16_t	u16LowVolShutDownTime;		/*低电压关机时间*/
		uint16_t	u16UpperCtlButSleep;		/*上控按键休眠*/
		uint16_t	u16LanguageType;			/*语言类型*/
		uint16_t	u16BatteryType;				/*电池类型*/
		uint16_t	u16SpeakerSync;				/*讯响器同步*/
		uint16_t	u16LowerPumpType;			/*下降阀类型*/
		uint16_t	u16PressureType;			/*压力采集方式*/
		uint16_t	u16AngleSimulationLimit;	/*角度模拟限位*/
		uint16_t	u16LiftReverseFunc;			/*举升反向功能*/
		
		uint16_t	u16Reserve7;				/*62*/
		uint16_t	u16Reserve8;				/*63*/
		
		uint16_t	u16FourPointWeightFunc;		/*四点称重功能*/
		uint16_t	u16DriverType;				/*驱动器类型*/
		uint16_t	u16PasswordLock;			/*密码锁*/
		uint16_t	u16InAndOutFunc;			/*室内外功能*/
		uint16_t	u16HeartBeatQueryFunc;		/*心跳查询功能*/
		uint16_t	u16LowBatteryMode;			/*低电量模式选择*/
		uint16_t	u16AngleSensorSetting;		/*角度传感器设置*/
		uint16_t	u16TiltSwitchSetting;		/*倾角开关设置*/
		uint16_t	u16AngleSensorType;			/*角度传感器类型*/
		uint16_t	u16AnaLogLimitDetSwitch;	/*模拟限位检测开关*/
		
		uint16_t	u16IsNoLoadCalibration;		/*是否进行空载标定*/
		uint16_t	u16IsOverLoadCalibration;	/*是否进行满载标定*/
		
		uint16_t	u16SetDescentHeightValue;	/*设置可下降高度值*/
		uint16_t	u16ReleaseBrake;			/*释放刹车*/
		
		uint16_t	u16Reserve9;				/*78*/
		uint16_t	u16Reserve10;				/*79*/
		
		uint16_t	u16SetOutHeight;			/*设置室外模式高度*/
		uint16_t	u16AngleSimulationUpLimit;	/*角度模拟上限位*/
		uint16_t	u16AngleSimulationDownLimit;/*角度模拟下限位*/
		
		/* Propdriver 0*/
		uint16_t	u16PumpDriveCurrentLimitRatio0;	/* Pump current limit  map nominal ratio 5~100%--1638~32767 */	
		uint16_t	u16PumpSpdAccRatio0;				/* Pump motor acc time */
		uint16_t	u16PropDKp0;						/* Propdriver current loop Kp gain 1%~100%--328~32767 */
		uint16_t	u16PropDKi0;						/* Propdriver current loop Ki gain 1%~100%--82~8192 */
		uint16_t	u16PropDMaxCurrent0;				/* Propdriver max current 0~2.0A -- 0~2000 */
		uint16_t	u16PropDMinCurrent0;				/* Propdriver min current 0~2.0A -- 0~2000 */
		uint16_t	u16PropDAccPeriod0;					/*Propdriver Acc Step Period*/
		uint16_t	u16PropDDitherPeriod0;				/* Propdriver dither period 15ms~120ms--1~8 */
		uint16_t	u16PropDDitherRatio0;				/* Propdriver dither current ratio.(DitherCurrent/PropDMaxCurrent) 0~100% -- 0~32767 */
		uint16_t	u16PropValveResistance0;			/* Prop valve coil resistance. 1~1000 ohm -- 10~10000 */
		
		uint16_t	u16Reserve11;				/*93*/
		uint16_t	u16Reserve12;				/*94*/
		uint16_t	u16Reserve13;				/*95*/
		/* Propdriver 1*/
		uint16_t	u16PumpDriveCurrentLimitRatio1;	/* Pump current limit  map nominal ratio 5~100%--1638~32767 */	
		uint16_t	u16PumpSpdAccRatio1;				/* Pump motor acc time */
		uint16_t	u16PropDKp1;						/* Propdriver current loop Kp gain 1%~100%--328~32767 */
		uint16_t	u16PropDKi1;						/* Propdriver current loop Ki gain 1%~100%--82~8192 */
		uint16_t	u16PropDMaxCurrent1;				/* Propdriver max current 0~2.0A -- 0~2000 */
		uint16_t	u16PropDMinCurrent1;				/* Propdriver min current 0~2.0A -- 0~2000 */
		uint16_t	u16PropDAccPeriod1;					/*Propdriver Acc Step Period*/
		uint16_t	u16PropDDitherPeriod1;			/* Propdriver dither period 15ms~120ms--1~8 */
		uint16_t	u16PropDDitherRatio1;			/* Propdriver dither current ratio.(DitherCurrent/PropDMaxCurrent) 0~100% -- 0~32767 */
		uint16_t	u16PropValveResistance1;			/* Prop valve coil resistance. 1~1000 ohm -- 10~10000 */
		uint16_t	u16CanBaudRate;
		uint16_t	u16PressureEmptyVlaue;			/*空载压力值*/
		uint16_t	u16PressureFullVlaue;			/*满载压力值*/
		uint16_t	u16DriverFlag;					/*Driver*/
		
		uint16_t	u16Reserve14;				/*110*/
		uint16_t	u16Reserve15;				/*111*/
		
		uint16_t	u16MinAngle;					
		uint16_t	u16MaxAngle;
		uint16_t 	u16BatSocPalyBack;
		uint16_t 	u16ValveType;
		uint16_t	u16AnticollisionFunc;
		uint16_t	u16ValueOpenLoopCurrent;
		uint16_t	u16ValueOpenPercentage;			/*阀打开占空比*/
		uint16_t	u16CanOpenNodeId;
		
		uint16_t	u16AngleValue0;
		uint16_t	u16AngleValue1;
		uint16_t	u16AngleValue2;
		uint16_t	u16AngleValue3;
		uint16_t	u16AngleValue4;
		uint16_t	u16AngleValue5;
		uint16_t	u16AngleValue6;
		uint16_t	u16AngleValue7;
		
		uint16_t	u16Driver1Vol;
		uint16_t	u16Driver2Vol;
		uint16_t	u16Driver3Vol;
		uint16_t	u16Driver4Vol;
		uint16_t	u16Driver5Vol;
		uint16_t	u16Driver6Vol;
		uint16_t	u16Driver7Vol;
		uint16_t	u16Driver8Vol;
		uint16_t	u16Driver9Vol;
		uint16_t	u16Driver10Vol;
		
		uint16_t	u16LogLevel;			/*138*/
		uint16_t	u16LogModel;			/*139*/
		uint16_t	u16MotorMaxSpd;			/*140*/
		
		uint16_t 	u16Analog3DeadZoneMinVal;	/*141*/
		uint16_t 	u16Analog3DeadZoneMaxVal;	/*142*/
		uint16_t	u16Analog3MidVal;			/*143*/
		
		uint16_t	u16ThrottleType;
		uint16_t	u16ThrottleFDeadZoneMinVal;
		uint16_t	u16ThrottleFDeadZoneMaxVal;
		uint16_t	u16ThrottleFMidVal;
		uint16_t	u16ThrottleBDeadZoneMinVal;
		uint16_t	u16ThrottleBDeadZoneMaxVal;
		uint16_t	u16ThrottleBMidVal;
		
		uint16_t	u16BrakeType;
		uint16_t	u16BrakeFDeadZoneMinVal;
		uint16_t	u16BrakeFDeadZoneMaxVal;
		uint16_t	u16BrakeFMidVal;
		uint16_t	u16BrakeBDeadZoneMinVal;
		uint16_t	u16BrakeBDeadZoneMaxVal;
		uint16_t	u16BrakeBMidVal;
		
		uint16_t	u16Reserve16;				/*158*/
		uint16_t	u16Reserve17;				/*159*/
		
		uint16_t 	u16Gear1Spd;
		uint16_t 	u16Gear2Spd;
		uint16_t 	u16Gear3Spd;
		uint16_t 	u16Gear4Spd;
		
		uint16_t	u16RatioOfTransmission;
		uint16_t	u16MaintenancePeriod;
		uint16_t	u16RemotePara;
		
		uint16_t	u16PumpMotorGear1;
		uint16_t	u16PumpMotorGear2;
		
		uint16_t	u16TurnWithDecStartAngle;
		uint16_t	u16TurnWithDecEndAngle;
		
		uint16_t	u16AngleWithStartSpdPer;
		uint16_t	u16AngleWithEndSpdPer;
		
		uint16_t	u16HourCountPowerOn;
		
		uint16_t	u16Reserve18;				/*174*/
		uint16_t	u16Reserve19;				/*175*/
		uint16_t	u16Reserve20;				/*176*/
		uint16_t	u16Reserve21;				/*177*/
		uint16_t	u16Reserve22;				/*178*/
		uint16_t	u16Reserve23;				/*179*/
		uint16_t	u16Reserve24;				/*180*/
		uint16_t	u16Reserve25;				/*181*/
		uint16_t	u16Reserve26;				/*182*/
		uint16_t	u16Reserve27;				/*183*/
		
		uint16_t	u16ErrCode[PARA_ErrCodeMax - PARA_ErrCode0 + 1];
		
		uint16_t	u16UserType;				/*200*/
				
		/*monitor state*/
		/*称重参数*/
		int16_t		i16AngleValue;				/*角度值*/
		uint16_t	u16PressureVlaue1;			/*压力值1*/
		uint16_t	u16PressureVlaue2;			/*压力值2*/
		uint16_t	u16LoadRate;				/*负载率*/
		uint16_t	u16CalibrationStatus;		/*标定状态*/
		/*ECU开关输入监控*/
		uint16_t	u16ForwardValveCurrent;		/*前进阀电流值*/
		uint16_t	u16BackValveCurrent;		/*后退阀电流值*/
		uint16_t	u16LiftValveCUrrent;		/*起升阀电流值*/
		uint16_t	u16PropValveCurrent;		/*比例阀电流值*/
		uint16_t	u16OnOffValveCurrent;		/*开关阀电流值*/
		uint16_t	u16TurnRightValveCurrent;	/*右转阀电流值*/
		uint16_t	u16TurnLeftValveCurrent;	/*左转阀电流值*/
		uint16_t	u16BrakeValveCurrent;		/*刹车阀电流值*/
		/*车身数据*/
		uint16_t	u16BatteryVoltage;			/*电池电压*/
		uint16_t	u16ExtSignal;				/*外部信号输入*/
		uint16_t	u16PcuKeyInfo;				/*PCU按键信息*/
		uint16_t	u16OutCtrlInfo;				/*输出控制信息*/
		uint16_t	u16MotorSpd;				/*电机速度*/
		uint16_t	u16HandleAnalog;			/*手柄模拟量*/
		/*BMS数据监控*/
		uint16_t	u16BmsSoc;					/*BMS SOC*/
		uint16_t	u16BmsVoltage;				/*BMS总电压*/
 		
		/*ECU信息*/
		uint8_t		u8CtrlSNcode[16];			/*控制器SN码*/
		uint8_t		u8EcuHardVersion[16];		/*ECU 硬件版本号*/		
		uint8_t		u8EcuSoftVersion[16];		/*ECU 软件版本*/
		uint8_t		u8SoftComplileDate[16];		/*软件编译日期*/
		uint8_t		u8SoftCompileTime[16];		/*软件编译时间*/

		/*柳工GPS*/
		uint16_t	u16TemporaryUnlock;			/*临时解锁*/
		uint16_t	u16PlatfromHeartQuery;		/*平台心跳查询*/
		uint16_t	u16SelfHeartQuery;			/*本地心跳查询*/
		uint16_t	u16EcuLockState;			/*ECU锁车状态*/
		uint16_t	u16TmpLockState;			/*临时解锁状态*/
		
		uint16_t	u16Swi;
		uint16_t	u16DoSwi;
		uint16_t	u16Ai1;
		uint16_t	u16Ai2;
		uint16_t	u16Ai3;
		uint16_t	u16Ai4;
		uint16_t	u16Ai5;
		uint16_t	u16Vbus;
		uint16_t	u16Ksi;
		uint16_t	u16ErrCodeA;
		uint16_t	u16PropCurrent1;
		uint16_t	u16PropCurrent2;
		uint16_t 	u16MotorCmd;
		uint16_t	u16PumpCmd;
		uint16_t 	u16MotorState;
		uint16_t	u16SteerAngle;
		uint16_t	u16MotorSpeed;
//		uint16_t	u16BatteryFlag1;
//		uint16_t	u16BatteryCount1;
//		uint16_t	u16BatteryFlag2;
//		uint16_t	u16BatteryCount2;
	};

}xParameter;

/* ACCESS */
#define			ACCESS_R	0x00	//参数可读
#define			ACCESS_RW	0x01	//参数可写
/* ACCESS_AUTH */
#define			NONE	0x00	//无权限
#define			OEM		0x01	//OEM用户权限
#define			APP		0x02	//应用工程师权限
#define			SYS		0x03	//系统工程师权限
/* EEPROM */
#define 		EEPROM_PARA_AREA_ADDR	0
#define 		EEPROM_BACK_AREA_ADDR	128
#define 		EEPROM_PVD_ADDR    250


#define 		BATTERY_HOUR_VALID		0x5555
#define 		BATTERY_HOUR_INVALID	0xFFFF

#define			PARA_HourCountFlag1		230
#define			PARA_HourCountFlag2		231
#define			PARA_HourSetTime		232
#define			PARA_SaveState			233
#define			PARA_DefaultFlag		234

#define			PARA_NoBatteryFlag1		241
#define			PARA_NoBatteryCount1 	242
#define			PARA_NoBatteryFlag2		243
#define			PARA_NoBatteryCount2	244

#define			PARA_HourCount1Low		245
#define			PARA_HourCount1High		246
#define			PARA_HourCount2Low		247
#define			PARA_HourCount2High		248



extern int32_t i32GetPara(uint16_t u16Index);
extern uint16_t* u16pGetParaPoint(uint16_t u16Index);
extern int32_t i32SetPara(uint16_t u16Index, uint16_t u16Data);
extern void vParaInit(void);
extern uint16_t u16SaveParaToEeprom(uint16_t Index, uint16_t data);
extern uint16_t u16ReadParaFromEeprom(uint16_t Index);//23.12.5 SJ
#endif //#ifndef _PARA_H_
