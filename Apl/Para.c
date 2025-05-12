#include "Userdef.h"
#include "Device.h"
#include "Para.h"
#include "log.h"
#include "stdlib.h"
#include "Eeprom.h"
//#include ""


static xParameter sgPara;
/*快速行驶速度*/
static const xPrmDefStruct cParaFastDriveSpeed[] = 
{
	PARA_FastDriveSpeed,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16FastDriveSpeed) | PRM_ATTR_POWEROFF | PRM_ATTR_TYPE_BIT_SET(0,8),
	0,     		/*min*/
	255,     	/*max*/
	100,     	/*default*/
	(void*)&sgPara.u16FastDriveSpeed,
};
/*慢速行驶速度*/
static const xPrmDefStruct cParaSlowDriceSPeed[] = 
{
	PARA_SlowDriveSpeed,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16SlowDriveSpeed) | PRM_ATTR_POWEROFF | PRM_ATTR_TYPE_BIT_SET(0,8),
	0,     		/*min*/
	255,   		/*max*/
	50,     	/*default*/
	(void*)&sgPara.u16SlowDriveSpeed,

};
/*举升后行驶速度*/
static const xPrmDefStruct cParaDriveSpeedAfterLift[] = 
{
	PARA_DriveSpeedAfterLift,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16DriveSpeedAfterLift) | PRM_ATTR_POWEROFF | PRM_ATTR_TYPE_BIT_SET(0,8),
	0,     		/*min*/
	255,   		/*max*/		//23.12.16 SJ修改最大值
	20,     	/*default*/
	(void*)&sgPara.u16DriveSpeedAfterLift,

};
/*举升速度*/
static const xPrmDefStruct cParaLiftSpeed[] = 
{
	PARA_LiftSpeed,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16LiftSpeed) | PRM_ATTR_POWEROFF | PRM_ATTR_TYPE_BIT_SET(0,8),
	0,     		/*min*/
	255,   		/*max*/
	10,     	/*default*/
	(void*)&sgPara.u16LiftSpeed,

};
/*最大转弯行驶速度*/
static const xPrmDefStruct cParaMaxTurnSpeed[] = 
{
	PARA_MaxTurnSpeed,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16MaxTurnSpeed),
	0,     		/*min*/
	80,     	/*max*/
	40,     	/*default*/
	(void*)&sgPara.u16MaxTurnSpeed,
};
/*转弯动力限定值*/
static const xPrmDefStruct cParaTurnPowerLimit[] = 
{
	PARA_TurnPowerLimit,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16TurnPowerLimit),
	0,     		/*min*/
	80,     	/*max*/			//23.11.21 SJ 修改转向时最大速度比率
	40,     	/*default*/
	(void*)&sgPara.u16TurnPowerLimit,
};
/*死区值调节*/
static const xPrmDefStruct cParaDeadZoneAdjust[] = 
{
	PAPA_DeadZoneAdjust,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16DeadZoneAdjust),
	0,     		/*min*/
	25,     	/*max*/
	20,     	/*default*/
	(void*)&sgPara.u16DeadZoneAdjust,
};
/*刹车减速值调节*/
/*快速行走*/
static const xPrmDefStruct cParaBrakeFastDrive[] = 
{
	PARA_BrakeFastDrive,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16BrakeFastDrive),
	0,     		/*min*/
	255,     	/*max*/
	100,     		/*default*/
	(void*)&sgPara.u16BrakeFastDrive,
};
/*慢速行走*/
static const xPrmDefStruct cParaBrakeSlowDrive[] = 
{
	PARA_BrakeSlowDrive,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16BrakeSlowDrive),
	0,	    	/*min*/
	255,     	/*max*/
	80,    		/*default*/
	(void*)&sgPara.u16BrakeSlowDrive,
};
/*举升后行走*/
static const xPrmDefStruct cParaBrakeDriveAfterLift[] = 
{
	PARA_BrakeDriveAfterLift,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16BrakeDriveAfterLift),
	0, 	    	/*min*/
	255,     	/*max*/
	70,    		/*default*/
	(void*)&sgPara.u16BrakeDriveAfterLift,
};
/*举升*/
static const xPrmDefStruct cParaBrakeLift[] = 
{
	PARA_BrakeLift,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16BrakeLift),
	0, 	    	/*min*/
	255,     	/*max*/
	65,    		/*default*/
	(void*)&sgPara.u16BrakeLift,
};
/*下降*/
static const xPrmDefStruct cParaBrakeLower[] = 
{
	PARA_BrakeLower,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16BrakeLower),
	0, 	    	/*min*/
	255,     	/*max*/
	60,    		/*default*/
	(void*)&sgPara.u16BrakeLower,
};
/*转向动作*/
static const xPrmDefStruct cParaBrakeTurn[] =              // 25度
{
	PARA_BrakeTurn,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16BrakeTurn),
	0, 	    	/*min*/
	255,     	/*max*/
	55,    		/*default*/
	(void*)&sgPara.u16BrakeTurn,
};
/*防夹手缓冲*/
static const xPrmDefStruct cParaBrakeAntiPinch[] = 
{
	PARA_BrakeAntiPinch,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16BrakeAntiPinch),
	0, 	    	/*min*/
	255,     	/*max*/
	50,    		/*default*/
	(void*)&sgPara.u16BrakeAntiPinch,
};
/*Reserv1*/
static const xPrmDefStruct cParaReserve1[] = 
{
	PARA_Reserve1,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Reserve1),
	0, 	    	/*min*/
	65535,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16Reserve1,
};
/*Reserv2*/
static const xPrmDefStruct cParaReserve2[] = 
{
	PARA_Reserve2,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Reserve2),
	0, 	    	/*min*/
	65535,     	/*max*/
	50,    		/*default*/
	(void*)&sgPara.u16Reserve2,
};
/*下降速度*/
static const xPrmDefStruct cParaLowerSpeed[] =           //35度
{
	PARA_LowerSpeed,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16LowerSpeed),
	0,	 	    /*min*/
	100,     	/*max*/
	45,    		/*default*/
	(void*)&sgPara.u16LowerSpeed,
};
/*超载稳定延时*/
static const xPrmDefStruct cParaOverLoadStabilityDelay[] =       // 40度
{
	PARA_OverLoadStabilityDelay,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16OverLoadStabilityDelay),
	0,	 	    /*min*/
	100,     	/*max*/
	40,    		/*default*/
	(void*)&sgPara.u16OverLoadStabilityDelay,
};
/*动态超载百分比*/
static const xPrmDefStruct cParaDynamicOverLoadPercent[] = 
{
	PARA_DynamicOverLoadPercent,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16DynamicOverLoadPercent),
	0,	 	    /*min*/
	100,     	/*max*/
	5,    		/*default*/
	(void*)&sgPara.u16DynamicOverLoadPercent,
};
/*静态超载百分比*/
static const xPrmDefStruct cParaStaticOverLoadPercent[] = 
{
	PARA_StaticOverLoadPercent,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16StaticOverLoadPercent),
	0,	 		/*min*/
	50,     	/*max*/
	30,    		/*default*/
	(void*)&sgPara.u16StaticOverLoadPercent,
};
/*最大差值百分比*/
static const xPrmDefStruct cParaMaxDifferencePercent[] = 
{
	PARA_MaxDifferencePercent,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16MaxDifferencePercent),
	0,	 	    /*min*/
	100,     	/*max*/
	30,    		/*default*/
	(void*)&sgPara.u16MaxDifferencePercent,
};
/*行走电机编码*/
static const xPrmDefStruct cParaDriveMotorEncoder[] = 
{
	PARA_DriveMotorEncoder,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16DriveMotorEncoder),
	0,	 		/*min*/
	255,		/*max*/
	1,			/*default*/
	(void*)&sgPara.u16DriveMotorEncoder,
};
/*电机高速减速率*/
static const xPrmDefStruct cParaMotorHighSpeedDeceRate[] = 
{	
	PARA_MotorHighSpeedDeceRate,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16MotorHighSpeedDeceRate),
#ifdef HANHGCHA_APP_PARA
	0,	 	    /*min*/
	100,     	/*max*/
	100,    		/*default*/
#else
	0,	 	    /*min*/
	20,     	/*max*/
	10,    		/*default*/
#endif
	(void*)&sgPara.u16MotorHighSpeedDeceRate,
};
/*电机低速减速率*/
static const xPrmDefStruct cParaMotorLowSpeedDeceRate[] = 
{
	PARA_MotorLowSpeedDeceRate,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16MotorLowSpeedDeceRate),
#ifdef HANHGCHA_APP_PARA
	0,	 	    /*min*/
	100,     	/*max*/
	100,    		/*default*/
#else
	0,	 	    /*min*/
	20,     	/*max*/
	15,    		/*default*/
#endif
	(void*)&sgPara.u16MotorLowSpeedDeceRate,
};
/*语音报警音量*/
static const xPrmDefStruct cParaVoiceAlarmVolume[] = 
{
	PARA_VoiceAlarmVolume,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16VoiceAlarmVolume),
	0,	 	    /*min*/
	28,     	/*max*/
	28,    		/*default*/
	(void*)&sgPara.u16VoiceAlarmVolume,
};
/*曲线加速值调节*/
/*快速行走*/
static const xPrmDefStruct cParaCurveFastDrive[] =        //45°
{
	PARA_CurveFastDrive,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16CurveFastDrive),
	0,	 	    /*min*/
	255,     	/*max*/
	30,    		/*default*/
	(void*)&sgPara.u16CurveFastDrive,
};
/*慢速行走*/
static const xPrmDefStruct cParaCurveSlowDrive[] = 
{
	PARA_CurveSlowDrive,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16CurveSlowDrive),
	0,	 	    /*min*/
	255,     	/*max*/
	25,    		/*default*/
	(void*)&sgPara.u16CurveSlowDrive,
};
/*举升后行走*/
static const xPrmDefStruct cParaCurveDriveAfterLift[] = 
{
	PARA_CurveDriveAfterLift,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16CurveDriveAfterLift),
	0,	 	    /*min*/
	255,     	/*max*/
	20,    		/*default*/
	(void*)&sgPara.u16CurveDriveAfterLift,
};
/*举升*/
static const xPrmDefStruct cParaCurveLift[] =     //60度
{
	PARA_CurveLift,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16CurveLift),
	0,	 	   	/*min*/
	255,     	/*max*/
	15,    		/*default*/
	(void*)&sgPara.u16CurveLift,
};
/*下降*/
static const xPrmDefStruct cParaCurveLower[] =           // 65度
{
	PARA_CurveLower,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16CurveLower),
	0,	 	    /*min*/
	255,     	/*max*/
	10,    		/*default*/
	(void*)&sgPara.u16CurveLower,
};
/*Reserve3*/
static const xPrmDefStruct cParaReserve3[] = 
{
	PARA_Reserve3,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Reserve3),
	0, 	    	/*min*/
	65535,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16Reserve3,
};
/*Reserve4*/
static const xPrmDefStruct cParaReserve4[] = 
{
	PARA_Reserve4,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Reserve4),
	0, 	    	/*min*/
	65535,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16Reserve4,
};
/*转向*/
static const xPrmDefStruct cParaCurveTurn[] = 
{
	PARA_CurveTurn,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16CurveTurn),
	0,	 	    /*min*/
	255,     	/*max*/
	5,    		/*default*/
	(void*)&sgPara.u16CurveTurn,
};
/*加减速周期调节*/
/*快速行走*/
static const xPrmDefStruct cParaAccAndDecFastDrive[] = 
{
	PARA_AccAndDecFastDrive,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16AccAndDecFastDrive),
	0,	 	    /*min*/
	255,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16AccAndDecFastDrive,
};
/*慢速行走*/
static const xPrmDefStruct cParaAccAndDecSlowDrive[] =                     
{
	PARA_AccAndDecSlowDrive,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16AccAndDecSlowDrive),
	0,	 	    /*min*/
	255,     	/*max*/
	5,    		/*default*/
	(void*)&sgPara.u16AccAndDecSlowDrive,
};
/*举升后行走*/
static const xPrmDefStruct cParaAccAndDecAfterLift[] = 
{
	PARA_AccAndDecAfterLift,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16AccAndDecAfterLift),
	0,	 	    /*min*/
	255,     	/*max*/
	25,    		/*default*/
	(void*)&sgPara.u16AccAndDecAfterLift,
};
/*举升*/
static const xPrmDefStruct cParaAccAndDecLift[] =                      //89度 90度
{
	PARA_AccAndDecLift,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16AccAndDecLift),
	0,	 	    /*min*/
	255,     	/*max*/
	50,    		/*default*/
	(void*)&sgPara.u16AccAndDecLift,
};
/*下降*/
static const xPrmDefStruct cParaAccAndDecLower[] = 
{
	PARA_AccAndDecLower,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16AccAndDecLower),
	0,	 	  	/*min*/
	255,     	/*max*/
	8,    		/*default*/
	(void*)&sgPara.u16AccAndDecLower,
};
/*转向*/
static const xPrmDefStruct cParaAccAndDecTurn[] = 
{
	PARA_AccAndDecTurn,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16AccAndDecTurn),
	0,	 	    /*min*/
	255,     	/*max*/
	15,    		/*default*/
	(void*)&sgPara.u16AccAndDecTurn,
};
/*防夹手*/
static const xPrmDefStruct cParaAccAndDecAntiPinch[] = 
{
	PARA_AccAndDecAntiPinch,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16AccAndDecAntiPinch),
	0,	 	    /*min*/
	255,     	/*max*/
	8,    		/*default*/
	(void*)&sgPara.u16AccAndDecAntiPinch,
};
/*泵电机编码*/
static const xPrmDefStruct cParaPumpMotorEncoder[] = 
{
	PARA_PumpMotorEncoder,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PumpMotorEncoder),
	0,	 	    /*min*/
	40,     	/*max*/
	21,    		/*default*/
	(void*)&sgPara.u16PumpMotorEncoder,
};
/*车辆类型*/
static const xPrmDefStruct cParaVehicleType[] = 
{
	PARA_VehicleType,
	PRM_ARRT_EPROM | PRM_ATTR_SIGNED | PRM_ATTR_SIZE(sgPara.u16VehicleType),
	0,	 	    /*min*/
	100,     		/*max*/  //23.12.11修改、柳工车型代号已经到63
	LS0607H,    		/*default*/
	(void*)&sgPara.u16VehicleType,
};
/*车辆高度*/
static const xPrmDefStruct cParaVehcileHeight[] = 
{
	PARA_VehcileHeight,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16VehcileHeight),
	SingleCylinder,	 	    /*min*/
	DoubleCylinder,     	/*max*/
	SingleCylinder,    		/*default*/
	(void*)&sgPara.u16VehcileHeight,
};
/*压力传感器类型*/
static const xPrmDefStruct cParaPressureSensorType[] = 
{
	PARA_PressureSensorType,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PressureSensorType),
	NoSensor,	 /*min*/  //23.12.14 修改传感器类型
	DoubleChannelSensor,     /*max*/
	DoubleChannelSensor,	/*default*/
	(void*)&sgPara.u16PressureSensorType,
};
/*坑洞保护功能*/
static const xPrmDefStruct cParaPitProtectFunc[] = 
{
	PARA_PitProtectFunc,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PitProtectFunc),
 	FunctionDisable,    /*min*/
	FunctionEnable,	     	/*max*/
	FunctionEnable,    		/*default*/
	(void*)&sgPara.u16PitProtectFunc,
};
/*防夹手功能*/
static const xPrmDefStruct cParaAntiPinchFunc[] = 
{
	PARA_AntiPinchFunc,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16AntiPinchFunc),
	FunctionDisable,	    /*min*/
	FunctionEnable,	     	/*max*/
	FunctionEnable,    		/*default*/
	(void*)&sgPara.u16AntiPinchFunc,
};
/*Reserve5*/
static const xPrmDefStruct cParaReserve5[] = 
{
	PARA_Reserve5,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Reserve5),
	0, 	    	/*min*/
	65535,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16Reserve5,
};
/*Reserve6*/
static const xPrmDefStruct cParaReserve6[] = 
{
	PARA_Reserve6,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Reserve6),
	0, 	    	/*min*/
	65535,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16Reserve6,
};
/*动作报警功能*/
static const xPrmDefStruct cParaActAlmFunc[] = 
{
	PARA_ActAlmFunc,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16ActAlmFunc),
	FunctionDisable,	 	    /*min*/
	FunctionEnable,    	/*max*/
	FunctionEnable,    		/*default*/
	(void*)&sgPara.u16ActAlmFunc,
};
/*称重功能*/
static const xPrmDefStruct cParaWeighFunc[] = 
{
	PARA_WeighFunc,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16WeighFunc),
	FunctionDisable,  	    /*min*/
	FunctionEnable,		    	/*max*/
	FunctionEnable,    	/*default*/
	(void*)&sgPara.u16WeighFunc,
};
/*并联阀反向功能*/
static const xPrmDefStruct cParaParallelValveReverseFunc[] = 
{
	PARA_ParallelValveReverseFunc,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16ParallelValveReverseFunc),
	FunctionDisable,	 	    /*min*/
	FunctionEnable,     	/*max*/
	FunctionDisable,    	/*default*/
	(void*)&sgPara.u16ParallelValveReverseFunc,
};
/*低电量报警功能*/
static const xPrmDefStruct cParaLowBatAlmFunc[] = 
{
	PARA_LowBatAlmFunc,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16LowBatAlmFunc),
	FunctionDisable,	 	    /*min*/
	FunctionEnable,     	/*max*/
	FunctionEnable,    		/*default*/
	(void*)&sgPara.u16LowBatAlmFunc,
};
/*低电压报警时间*/
static const xPrmDefStruct cParaLowVolAlmTime[] = 
{
	PARA_LowVolAlmTime,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16LowVolAlmTime),
	0,	 	    /*min*/
	60,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16LowVolAlmTime,
};
/*低电压关机时间*/
static const xPrmDefStruct cParaLowVolShutDownTime[] = 
{
	PARA_LowVolShutDownTime,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16LowVolShutDownTime),
	0,	 	    /*min*/
	60,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16LowVolShutDownTime,
};
/*上控按键休眠*/
static const xPrmDefStruct cParaUpperCtlButSleep[] = 
{
	PARA_UpperCtlButSleep,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16UpperCtlButSleep),
	0,	 	    /*min*/
	60,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16UpperCtlButSleep,
};
/*语言类型*/
static const xPrmDefStruct cParaLanguageType[] = 
{
	PARA_LanguageType,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16LanguageType),
	LiBattery,	 	/*min*/
	XuPai,     		/*max*/
	LiShi,    		/*default*/
	(void*)&sgPara.u16LanguageType,
};
/*电池类型*/
static const xPrmDefStruct cParaBatteryType[] = 
{
	PARA_BatteryType,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16BatteryType),
	LiBattery,	 	/*min*/
	XuPai,     		/*max*/
	LiShi,    		/*default*/
	(void*)&sgPara.u16BatteryType,
};
/*讯响器同步*/
static const xPrmDefStruct cParaSpeakerSync[] = 
{
	PARA_SpeakerSync,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16SpeakerSync),
	FunctionDisable,	 	    /*min*/
	FunctionEnable,     	/*max*/
	FunctionDisable,    	/*default*/
	(void*)&sgPara.u16SpeakerSync,
};
/*下降阀类型*/
static const xPrmDefStruct cParaLowerPumpType[] = 
{
	PARA_LowerPumpType,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16LowerPumpType),
	OnOffValve,	 	    /*min*/		//修改比例阀和开关阀选项参数
	PropValve,     	/*max*/
	PropValve,    		/*default*/
	(void*)&sgPara.u16LowerPumpType,
};
/*压力采集方式*/
static const xPrmDefStruct cParaPressureType[] = 
{
	PARA_PressureType,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PressureType),
	CurrentType,	 	/*min*/
	VoltageType,     	/*max*/
	VoltageType,    	/*default*/
	(void*)&sgPara.u16PressureType,
};
/*角度模拟限位*/
static const xPrmDefStruct cParaAngleSimulationLimit[] = 
{
	PARA_AngleSimulationLimit,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16AngleSimulationLimit),
	FunctionDisable,			/*min*/
	FunctionEnable,		/*max*/
	FunctionEnable,			/*default*/
	(void*)&sgPara.u16AngleSimulationLimit,
};
/*举升反向功能*/
static const xPrmDefStruct cParaLiftReverseFunc[] = 
{
	PARA_LiftReverseFunc,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16LiftReverseFunc),
	FunctionDisable,			/*min*/
	FunctionEnable,		/*max*/
	FunctionDisable,		/*default*/
	(void*)&sgPara.u16LiftReverseFunc,
};
/*Reserve7*/
static const xPrmDefStruct cParaReserve7[] = 
{
	PARA_Reserve7,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Reserve7),
	0, 	    	/*min*/
	65535,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16Reserve7,
};
/*Reserve8*/
static const xPrmDefStruct cParaReserve8[] = 
{
	PARA_Reserve8,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Reserve8),
	0, 	    	/*min*/
	65535,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16Reserve8,
};
/*四点称重功能*/
static const xPrmDefStruct cParaFourPointWeightFunc[] = 
{
	PARA_FourPointWeightFunc,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16FourPointWeightFunc),
	FunctionDisable,		/*min*/
	FunctionEnable,			/*max*/
	FunctionEnable,		/*default*/
	(void*)&sgPara.u16FourPointWeightFunc,
};
/*驱动器类型*/
static const xPrmDefStruct cParaDriverType[] = 
{
	PARA_DriverType,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16DriverType),
	BLDC,	 	    /*min*/
	QPDAC,     		/*max*/
	QPSAC,    		/*default*/
	(void*)&sgPara.u16DriverType,
};
/*密码锁*/
static const xPrmDefStruct cParaPasswordLock[] = 
{
	PARA_PasswordLock,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PasswordLock),
	NoKey,	 	    /*min*/
	Key2,     		/*max*/
	Key1,    		/*default*/
	(void*)&sgPara.u16PasswordLock,
};
/*室内外功能*/
static const xPrmDefStruct cParaInAndOutFunc[] = 
{
	PARA_InAndOutFunc,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16InAndOutFunc),
	FunctionDisable,			/*min*/
	FunctionEnable,		/*max*/
	FunctionEnable,			/*default*/
	(void*)&sgPara.u16InAndOutFunc,
};
/*心跳查询功能*/
static const xPrmDefStruct cParaHeartBeatQueryFunc[] = 
{
	PARA_HeartBeatQueryFunc,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16HeartBeatQueryFunc),
	0,	 	    	/*min*/
	65535,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16HeartBeatQueryFunc,
};
/*低电量模式选择*/
static const xPrmDefStruct cParaLowBatteryMode[] = 
{
	PARA_LowBatteryMode,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16LowBatteryMode),
	DefaultMode,	 	/*min*/
	OptionalMode,     	/*max*/
	DefaultMode,    	/*default*/
	(void*)&sgPara.u16LowBatteryMode,
};
/*角度传感器设置*/
static const xPrmDefStruct cParaAngleSensorSetting[] = 
{
	PARA_AngleSensorSetting,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16AngleSensorSetting),
	FunctionDisable,			/*min*/
	FunctionEnable,		/*max*/
	FunctionEnable,			/*default*/
	(void*)&sgPara.u16AngleSensorSetting,
};
/*倾角开关设置*/
static const xPrmDefStruct cParaTiltSwitchSetting[] = 
{
	PARA_TiltSwitchSetting,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16TiltSwitchSetting),
	FunctionDisable,			/*min*/
	FunctionEnable,		/*max*/
	FunctionEnable,			/*default*/
	(void*)&sgPara.u16TiltSwitchSetting,
};
/*角度传感器类型*/
static const xPrmDefStruct cParaAngleSensorType[] = 
{
	PARA_AngleSensorType,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16AngleSensorType),
	NoSensor,	/*min*/
	DoubleChannelSensor,    /*max*/
	SingleChannelSensor,    /*default*/
	(void*)&sgPara.u16AngleSensorType,
};
/*模拟限位检测开关*/
static const xPrmDefStruct cParaAnaLogLimitDetSwitch[] = 
{
	PARA_AnaLogLimitDetSwitch,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16AnaLogLimitDetSwitch),
	FunctionDisable,			/*min*/
	FunctionEnable,		/*max*/
	FunctionEnable,			/*default*/
	(void*)&sgPara.u16AnaLogLimitDetSwitch,
};
/*是否进行空载标定*/
static const xPrmDefStruct cParaIsNoLoadCalibration[] = 
{
	PARA_IsNoLoadCalibration,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16IsNoLoadCalibration),
	0,	 	    /*min*/
	1,     		/*max*/
	1,    		/*default*/
	(void*)&sgPara.u16IsNoLoadCalibration,
};
/*是否进行满载标定*/
static const xPrmDefStruct cParaIsOverLoadCalibration[] = 
{
	PARA_IsOverLoadCalibration,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16IsOverLoadCalibration),
	0,	 	    /*min*/
	1,     	/*max*/
	1,    		/*default*/
	(void*)&sgPara.u16IsOverLoadCalibration,
};
/*设置可下降高度值*/
static const xPrmDefStruct cParaSetDescentHeightValue[] = 
{
	PARA_SetDescentHeightValue,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16SetDescentHeightValue),
	0,	 	    	/*min*/
	65535,     		/*max*/
	1,    		  	/*default*/
	(void*)&sgPara.u16SetDescentHeightValue,
};
/*释放刹车*/
static const xPrmDefStruct cParaReleaseBrake[] = 
{
	PARA_ReleaseBrake,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16ReleaseBrake),
	0,	 	    	/*min*/
	65535,     		/*max*/
	1,    		  	/*default*/
	(void*)&sgPara.u16ReleaseBrake,
};
/*Reserve9*/
static const xPrmDefStruct cParaReserve9[] = 
{
	PARA_Reserve9,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Reserve9),
	0, 	    	/*min*/
	65535,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16Reserve9,
};
/*Reserve10*/
static const xPrmDefStruct cParaReserve10[] = 
{
	PARA_Reserve10,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Reserve10),
	0, 	    	/*min*/
	65535,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16Reserve10,
};
/*设置室外模式高度*/
static const xPrmDefStruct cParaSetOutHeight[] = 
{
	PARA_SetOutHeight,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16SetOutHeight),
	0,	 	    	/*min*/
	2340,     		/*max*/
	2,    		  	/*default*/
	(void*)&sgPara.u16SetOutHeight,
};
/*角度模拟上限位*/
static const xPrmDefStruct cParaAngleSimulationUpLimit[] = 
{
	PARA_AngleSimulationUpLimit,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16AngleSimulationUpLimit),
	0,	 	    	/*min*/
	65535,     		/*max*/
	2,    		  	/*default*/
	(void*)&sgPara.u16AngleSimulationUpLimit,
};
/*角度模拟下限位*/
static const xPrmDefStruct cParaAngleSimulationDownLimit[] = 
{
	PARA_AngleSimulationDownLimit,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16AngleSimulationDownLimit),
	0,	 	    	/*min*/
	65535,     		/*max*/
	2,    		  	/*default*/
	(void*)&sgPara.u16AngleSimulationDownLimit,
};
/**/
static const xPrmDefStruct cParaPumpDriveCurrentLimitRatio0[] = 
{
	PARA_PumpDriveCurrentLimitRatio0,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PumpDriveCurrentLimitRatio0),
	0,	 	    	//min
	100,     		//max
	0,    		//default
	(void*)&sgPara.u16PumpDriveCurrentLimitRatio0,
};
static const xPrmDefStruct cParaPumpSpdAccRatio0[] = 
{
	PARA_PumpSpdAccRatio0,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PumpSpdAccRatio0),
	0,	 	    	//min
	4096,     		//max
	0,    		//default
	(void*)&sgPara.u16PumpSpdAccRatio0,
};

	/* Propdriver */
static const xPrmDefStruct cParaPropDKp0[] = 
{
	PARA_PropDKp0,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PropDKp0),
	328,	 	    		//min
	32767,    		//max
	8192,    		  //default
	(void*)&sgPara.u16PropDKp0,
};
static const xPrmDefStruct cParaPropDKi0[] = 
{
	PARA_PropDKi0,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PropDKi0),
	82,	 	    		//min
	8192,    		//max
	4096,    		  //default
	(void*)&sgPara.u16PropDKi0,
};
static const xPrmDefStruct cParaPropDMaxCurrent0[] = 
{
	PARA_PropDMaxCurrent0,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PropDMaxCurrent0),
	0,	 	    		//min
	2000,    		//max
	800,    		  //default
	(void*)&sgPara.u16PropDMaxCurrent0,
};
static const xPrmDefStruct cParaPropDMinCurrent0[] = 
{
	PARA_PropDMinCurrent0,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PropDMinCurrent0),
	0,	 	    		//min
	2000,    		//max
	200,    		  //default
	(void*)&sgPara.u16PropDMinCurrent0,
};
static const xPrmDefStruct cParaPropDAccPeriod0[] = 
{
	PARA_PropDAccPeriod0,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PropDAccPeriod0),
	0,	 	    		//min
	2000,    		//max
	10,    		  //default
	(void*)&sgPara.u16PropDAccPeriod0,
};
static const xPrmDefStruct cParaPropDDitherPeriod0[] = 
{
	PARA_PropDDitherPeriod0,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PropDDitherPeriod0),
	1,	 	    		//min
	100,    		//max
	16,    		  //default
	(void*)&sgPara.u16PropDDitherPeriod0,
};
static const xPrmDefStruct cParaPropDDitherRatio0[] = 
{
	PARA_PropDDitherRatio0,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PropDDitherRatio0),
	0,	 	    		//min
	32767,    		//max
	5,    		  //default
	(void*)&sgPara.u16PropDDitherRatio0,
};
static const xPrmDefStruct cParaPropValveResistance0[] = 
{
	PARA_PropValveResistance0,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PropValveResistance0),
	10,	 	    		//min
	10000,    		//max
	20,    		  //default
	(void*)&sgPara.u16PropValveResistance0,
};
/*Reserve11*/
static const xPrmDefStruct cParaReserve11[] = 
{
	PARA_Reserve11,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Reserve11),
	0, 	    	/*min*/
	65535,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16Reserve11,
};
/*Reserve12*/
static const xPrmDefStruct cParaReserve12[] = 
{
	PARA_Reserve12,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Reserve12),
	0, 	    	/*min*/
	65535,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16Reserve12,
};
/*Reserve13*/
static const xPrmDefStruct cParaReserve13[] = 
{
	PARA_Reserve13,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Reserve13),
	0, 	    	/*min*/
	65535,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16Reserve13,
};
static const xPrmDefStruct cParaPumpDriveCurrentLimitRatio1[] = 
{
	PARA_PumpDriveCurrentLimitRatio1,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PumpDriveCurrentLimitRatio1),
	0,	 	    	//min
	100,     		//max
	0,    		//default
	(void*)&sgPara.u16PumpDriveCurrentLimitRatio1,
};
static const xPrmDefStruct cParaPumpSpdAccRatio1[] = 
{
	PARA_PumpSpdAccRatio1,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PumpSpdAccRatio1),
	0,	 	    	//min
	4096,     		//max
	0,    		//default
	(void*)&sgPara.u16PumpSpdAccRatio1,
};

	/* Propdriver */
static const xPrmDefStruct cParaPropDKp1[] = 
{
	PARA_PropDKp1,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PropDKp1),
	328,	 	    		//min
	32767,    		//max
	8192,    		  //default
	(void*)&sgPara.u16PropDKp1,
};
static const xPrmDefStruct cParaPropDKi1[] = 
{
	PARA_PropDKi1,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PropDKi1),
	82,	 	    		//min
	8192,    		//max
	4096,    		  //default
	(void*)&sgPara.u16PropDKi1,
};
static const xPrmDefStruct cParaPropDMaxCurrent1[] = 
{
	PARA_PropDMaxCurrent1,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PropDMaxCurrent1),
	0,	 	    		//min
	2000,    		//max
	800,    		  //default
	(void*)&sgPara.u16PropDMaxCurrent1,
};
static const xPrmDefStruct cParaPropDMinCurrent1[] = 
{
	PARA_PropDMinCurrent1,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PropDMinCurrent1),
	0,	 	    		//min
	2000,    		//max
	200,    		  //default
	(void*)&sgPara.u16PropDMinCurrent1,
};
static const xPrmDefStruct cParaPropDAccPeriod1[] = 
{
	PARA_PropDAccPeriod1,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PropDAccPeriod1),
	0,	 	    		//min
	2000,    		//max
	10,    		  //default
	(void*)&sgPara.u16PropDAccPeriod1,
};
static const xPrmDefStruct cParaPropDDitherPeriod1[] = 
{
	PARA_PropDDitherPeriod1,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PropDDitherPeriod1),
	1,	 	    		//min
	1000,    		//max
	16,    		  //default
	(void*)&sgPara.u16PropDDitherPeriod1,
};
static const xPrmDefStruct cParaPropDDitherRatio1[] = 
{
	PARA_PropDDitherRatio1,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PropDDitherRatio1),
	0,	 	    		//min
	32767,    		//max
	5,    		  //default
	(void*)&sgPara.u16PropDDitherRatio1,
//	"Propdriver",
//	"Propdriver dither current ratio.(DitherCurrent/PropDMaxCurrent) 0~100% -- 0~32767",
};
static const xPrmDefStruct cParaPropValveResistance1[] = 
{
	PARA_PropValveResistance1,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PropValveResistance1),
	10,	 	    		//min
	10000,    		//max
	20,    		  //default
	(void*)&sgPara.u16PropValveResistance1,
//	"Propdriver",
//	"Prop valve coil resistance. 1~1000 ohm -- 10~10000.",
};

static const xPrmDefStruct cParaCanBaudRate[] = 
{
	PARA_CanBaudRate,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16CanBaudRate),
	125,	 	    		//min
	500,    		//max
	500,    		  //default
	(void*)&sgPara.u16CanBaudRate,
//	"Propdriver",
//	"Prop valve coil resistance. 1~1000 ohm -- 10~10000.",
};

#if 0
static const xPrmDefStruct cParaPumpDriveCurrentLimitRatio2[] = 
{
	PARA_PumpDriveCurrentLimitRatio2,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PumpDriveCurrentLimitRatio2),
	1638,	 	    	//min
	32767,     		//max
	20000,    		//default
	(void*)&sgPara.u16PumpDriveCurrentLimitRatio2,
//	"Propdriver",
//	"Pump current limit  map nominal ratio 5~100%--1638~32767",
};
static const xPrmDefStruct cParaPumpSpdAccRatio2[] = 
{
	PARA_PumpSpdAccRatio2,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PumpSpdAccRatio2),
	5,	 	    	//min
	30000,     		//max
	200,    		//default
	(void*)&sgPara.u16PumpSpdAccRatio2,
//	"Propdriver",
//	"Pump motor acc time",
};

	/* Propdriver */
static const xPrmDefStruct cParaPropDKp2[] = 
{
	PARA_PropDKp2,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PropDKp2),
	328,	 	    		//min
	32767,    		//max
	8192,    		  //default
	(void*)&sgPara.u16PropDKp2,
//	"Propdriver",
//	"Propdriver current loop Kp gain 1%~100%--328~32767",
};
static const xPrmDefStruct cParaPropDKi2[] = 
{
	PARA_PropDKi2,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PropDKi2),
	82,	 	    		//min
	8192,    		//max
	2048,    		  //default
	(void*)&sgPara.u16PropDKi2,
//	"Propdriver",
//	"Propdriver current loop Ki gain 1%~100%--82~8192.",
};
static const xPrmDefStruct cParaPropDMaxCurrent2[] = 
{
	PARA_PropDMaxCurrent2,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PropDMaxCurrent2),
	0,	 	    		//min
	2000,    		//max
	0,    		  //default
	(void*)&sgPara.u16PropDMaxCurrent2,
//	"Propdriver",
//	"Propdriver max current 0~2.0A -- 0~2000.",
};
static const xPrmDefStruct cParaPropDMinCurrent2[] = 
{
	PARA_PropDMinCurrent2,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PropDMinCurrent2),
	0,	 	    		//min
	2000,    		//max
	0,    		  //default
	(void*)&sgPara.u16PropDMinCurrent2,
//	"Propdriver",
//	"Propdriver min current 0~2.0A -- 0~2000.",
};
static const xPrmDefStruct cParaPropDDitherPeriod2[] = 
{
	PARA_PropDDitherPeriod2,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PropDDitherPeriod2),
	1,	 	    		//min
	8,    		//max
	8,    		  //default
	(void*)&sgPara.u16PropDDitherPeriod2,
//	"Propdriver",
//	"Propdriver dither period 15ms~120ms--1~8",
};
static const xPrmDefStruct cParaPropDDitherRatio2[] = 
{
	PARA_PropDDitherRatio2,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PropDDitherRatio2),
	0,	 	    		//min
	32767,    		//max
	0,    		  //default
	(void*)&sgPara.u16PropDDitherRatio2,
//	"Propdriver",
//	"Propdriver dither current ratio.(DitherCurrent/PropDMaxCurrent) 0~100% -- 0~32767",
};
static const xPrmDefStruct cParaPropValveResistance2[] = 
{
	PARA_PropValveResistance2,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PropValveResistance2),
	10,	 	    		//min
	10000,    		//max
	250,    		  //default
	(void*)&sgPara.u16PropValveResistance2,
//	"Propdriver",
//	"Prop valve coil resistance. 1~1000 ohm -- 10~10000.",
};

static const xPrmDefStruct cParaPumpDriveCurrentLimitRatio3[] = 
{
	PARA_PumpDriveCurrentLimitRatio3,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PumpDriveCurrentLimitRatio3),
	1638,	 	    	//min
	32767,     		//max
	20000,    		//default
	(void*)&sgPara.u16PumpDriveCurrentLimitRatio3,
//	"Propdriver",
//	"Pump current limit  map nominal ratio 5~100%--1638~32767",
};
static const xPrmDefStruct cParaPumpSpdAccRatio3[] = 
{
	PARA_PumpSpdAccRatio3,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PumpSpdAccRatio3),
	5,	 	    	//min
	30000,     		//max
	200,    		//default
	(void*)&sgPara.u16PumpSpdAccRatio3,
//	"Propdriver",
//	"Pump motor acc time",
};

	/* Propdriver */
static const xPrmDefStruct cParaPropDKp3[] = 
{
	PARA_PropDKp3,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PropDKp3),
	328,	 	    		//min
	32767,    		//max
	8192,    		  //default
	(void*)&sgPara.u16PropDKp3,
//	"Propdriver",
//	"Propdriver current loop Kp gain 1%~100%--328~32767",
};
static const xPrmDefStruct cParaPropDKi3[] = 
{
	PARA_PropDKi3,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PropDKi3),
	82,	 	    		//min
	8192,    		//max
	2048,    		  //default
	(void*)&sgPara.u16PropDKi3,
//	"Propdriver",
//	"Propdriver current loop Ki gain 1%~100%--82~8192.",
};
static const xPrmDefStruct cParaPropDMaxCurrent3[] = 
{
	PARA_PropDMaxCurrent3,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PropDMaxCurrent3),
	0,	 	    		//min
	2000,    		//max
	0,    		  //default
	(void*)&sgPara.u16PropDMaxCurrent3,
//	"Propdriver",
//	"Propdriver max current 0~2.0A -- 0~2000.",
};
static const xPrmDefStruct cParaPropDMinCurrent3[] = 
{
	PARA_PropDMinCurrent3,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PropDMinCurrent3),
	0,	 	    		//min
	2000,    		//max
	0,    		  //default
	(void*)&sgPara.u16PropDMinCurrent3,
//	"Propdriver",
//	"Propdriver min current 0~2.0A -- 0~2000.",
};
static const xPrmDefStruct cParaPropDDitherPeriod3[] = 
{
	PARA_PropDDitherPeriod3,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PropDDitherPeriod3),
	1,	 	    		//min
	8,    		//max
	8,    		  //default
	(void*)&sgPara.u16PropDDitherPeriod3,
//	"Propdriver",
//	"Propdriver dither period 15ms~120ms--1~8",
};
static const xPrmDefStruct cParaPropDDitherRatio3[] = 
{
	PARA_PropDDitherRatio3,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PropDDitherRatio3),
	0,	 	    		//min
	32767,    		//max
	0,    		  //default
	(void*)&sgPara.u16PropDDitherRatio3,
//	"Propdriver",
//	"Propdriver dither current ratio.(DitherCurrent/PropDMaxCurrent) 0~100% -- 0~32767",
};
static const xPrmDefStruct cParaPropValveResistance3[] = 
{
	PARA_PropValveResistance3,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PropValveResistance3),
	10,	 	    		//min
	10000,    		//max
	250,    		  //default
	(void*)&sgPara.u16PropValveResistance3,
//	"Propdriver",
//	"Prop valve coil resistance. 1~1000 ohm -- 10~10000.",
};
#endif
static const xPrmDefStruct cParaEmptyPressure[] = 
{
	PARA_EmptyPressure,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PressureEmptyVlaue),
	0,	 	    	/*min*/
	4096,     		/*max*/
	100,    		  	/*default*/
	(void*)&sgPara.u16PressureEmptyVlaue,
//	"Speed",
//	"The dec ratio for Fast Mode  12.5%~800% -- 512~32767",
};

static const xPrmDefStruct cParaFullPressure[] = 
{
	PARA_FullPressure,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PressureFullVlaue),
	0,	 	    	/*min*/
	4096,     		/*max*/
	3000,    		  	/*default*/
	(void*)&sgPara.u16PressureFullVlaue,
//	"Speed",
//	"The dec ratio for Fast Mode  12.5%~800% -- 512~32767",
};

static const xPrmDefStruct cParaDriverFlag[] = 
{
	PARA_DriverFlag,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16DriverFlag),
	0,	 	    	/*min*/
	65535,     		/*max*/
	0x3FE,    		 /*default*/
	(void*)&sgPara.u16DriverFlag,
//	"Speed",
//	"The dec ratio for Fast Mode  12.5%~800% -- 512~32767",
};
/*Reserve14*/
static const xPrmDefStruct cParaReserve14[] = 
{
	PARA_Reserve14,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Reserve14),
	0, 	    	/*min*/
	65535,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16Reserve14,
};
/*Reserve15*/
static const xPrmDefStruct cParaReserve15[] = 
{
	PARA_Reserve15,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Reserve15),
	0, 	    	/*min*/
	65535,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16Reserve15,
};
static const xPrmDefStruct cParaMinAngle[] = 
{
	PARA_MinAngle,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16MinAngle),
	0,	 	    	/*min*/
	4096,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16MinAngle,
//	"Speed",
//	"The dec ratio for Fast Mode  12.5%~800% -- 512~32767",
};

static const xPrmDefStruct cParaMaxAngle[] = 
{
	PARA_MaxAngle,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16MaxAngle),
	0,	 	    	/*min*/
	4096,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16MaxAngle,
//	"Speed",
//	"The dec ratio for Fast Mode  12.5%~800% -- 512~32767",
};

static const xPrmDefStruct cParaBatSocPalyBack[] = 
{
	PARA_BatSocPalyBack,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16BatSocPalyBack),
	0,	 	    	/*min*/
	1000,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16BatSocPalyBack,
//	"Speed",
//	"The dec ratio for Fast Mode  12.5%~800% -- 512~32767",
};

static const xPrmDefStruct cParaValveType[] = /*23.10.12 shijin,change max number*/
{
	PARA_ValveType,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16ValveType),
	0,	 	    	/*min*/
	65535,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16ValveType,
//	"Speed",
//	"The dec ratio for Fast Mode  12.5%~800% -- 512~32767",
};
static const xPrmDefStruct cParaAnticollisionFunc[] = 
{
	PARA_AnticollisionFunc,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16AnticollisionFunc),
	0,	 	    	/*min*/
	1,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16AnticollisionFunc,
//	"Speed",
//	"The dec ratio for Fast Mode  12.5%~800% -- 512~32767",
};

static const xPrmDefStruct cParaValueOpenLoopCurrent[] = 
{
	PARA_ValueOpenLoopCurrent,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16ValueOpenLoopCurrent),
	0,	 	    	/*min*/
	10000,     		/*max*/
	800,    		 /*default*/
	(void*)&sgPara.u16ValueOpenLoopCurrent,
//	"Current",
//	"The dec ratio for Fast Mode  12.5%~800% -- 512~32767",
};

static const xPrmDefStruct cParaValueOpenPercentage[] = 
{
	PARA_ValueOpenPercentage,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16ValueOpenPercentage),
	0,	 	    	/*min*/
	100,     		/*max*/
	80,    		 /*default*/
	(void*)&sgPara.u16ValueOpenPercentage,
//	"Current",
//	"The dec ratio for Fast Mode  12.5%~800% -- 512~32767",
};

static const xPrmDefStruct cParaCanOpenNodeId[] = 
{
	PARA_CanOpenNodeId,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16CanOpenNodeId),
	0,	 	    	/*min*/
	128,     		/*max*/
	44,    		 	/*default*/
	(void*)&sgPara.u16CanOpenNodeId,
//	"Current",
//	"The dec ratio for Fast Mode  12.5%~800% -- 512~32767",
};


static const xPrmDefStruct cParaAngleValue0[] = 
{
	PARA_AngleValue0,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16AngleValue0),
	0,	 	    	/*min*/
	65535,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16AngleValue0,
//	"Speed",
//	"The dec ratio for Fast Mode  12.5%~800% -- 512~32767",
};

static const xPrmDefStruct cParaAngleValue1[] = 
{
	PARA_AngleValue1,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16AngleValue1),
	0,	 	    	/*min*/
	65535,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16AngleValue1,
//	"Speed",
//	"The dec ratio for Fast Mode  12.5%~800% -- 512~32767",
};

static const xPrmDefStruct cParaAngleValue2[] = 
{
	PARA_AngleValue2,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16AngleValue2),
	0,	 	    	/*min*/
	65535,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16AngleValue2,
//	"Speed",
//	"The dec ratio for Fast Mode  12.5%~800% -- 512~32767",
};

static const xPrmDefStruct cParaAngleValue3[] = 
{
	PARA_AngleValue3,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16AngleValue3),
	0,	 	    	/*min*/
	65535,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16AngleValue3,
//	"Speed",
//	"The dec ratio for Fast Mode  12.5%~800% -- 512~32767",
};

static const xPrmDefStruct cParaAngleValue4[] = 
{
	PARA_AngleValue4,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16AngleValue4),
	0,	 	    	/*min*/
	65535,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16AngleValue4,
//	"Speed",
//	"The dec ratio for Fast Mode  12.5%~800% -- 512~32767",
};

static const xPrmDefStruct cParaAngleValue5[] = 
{
	PARA_AngleValue5,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16AngleValue5),
	0,	 	    	/*min*/
	65535,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16AngleValue5,
//	"Speed",
//	"The dec ratio for Fast Mode  12.5%~800% -- 512~32767",
};

static const xPrmDefStruct cParaAngleValue6[] = 
{
	PARA_AngleValue6,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16AngleValue6),
	0,	 	    	/*min*/
	65535,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16AngleValue6,
//	"Speed",
//	"The dec ratio for Fast Mode  12.5%~800% -- 512~32767",
};

static const xPrmDefStruct cParaAngleValue7[] = 
{
	PARA_AngleValue7,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16AngleValue7),
	0,	 	    	/*min*/
	65535,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16AngleValue7,
//	"Speed",
//	"The dec ratio for Fast Mode  12.5%~800% -- 512~32767",
};
/*Default Pull:50  Hold:40*/
static const xPrmDefStruct cParaDriver1Vol[] = 
{
	PARA_Driver1Vol,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Driver1Vol),
	0,	 	    	/*min*/
	65535,     		/*max*/
	12840,    		/*default*/
	(void*)&sgPara.u16Driver1Vol,
};

static const xPrmDefStruct cParaDriver2Vol[] = 
{
	PARA_Driver2Vol,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Driver2Vol),
	0,	 	    	/*min*/
	65535,     		/*max*/
	12840,    		  	/*default*/
	(void*)&sgPara.u16Driver2Vol,
};

static const xPrmDefStruct cParaDriver3Vol[] = 
{
	PARA_Driver3Vol,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Driver3Vol),
	0,	 	    	/*min*/
	65535,     		/*max*/
	12840,    		  	/*default*/
	(void*)&sgPara.u16Driver3Vol,
};

static const xPrmDefStruct cParaDriver4Vol[] = 
{
	PARA_Driver4Vol,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Driver4Vol),
	0,	 	    	/*min*/
	65535,     		/*max*/
	12840,    		  	/*default*/
	(void*)&sgPara.u16Driver4Vol,
};

static const xPrmDefStruct cParaDriver5Vol[] = 
{
	PARA_Driver5Vol,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Driver5Vol),
	0,	 	    	/*min*/
	65535,     		/*max*/
	12840,    		  	/*default*/
	(void*)&sgPara.u16Driver5Vol,
};

static const xPrmDefStruct cParaDriver6Vol[] = 
{
	PARA_Driver6Vol,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Driver6Vol),
	0,	 	    	/*min*/
	65535,     		/*max*/
	12840,    		  	/*default*/
	(void*)&sgPara.u16Driver6Vol,
};

static const xPrmDefStruct cParaDriver7Vol[] = 
{
	PARA_Driver7Vol,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Driver7Vol),
	0,	 	    	/*min*/
	65535,     		/*max*/
	12840,    		  	/*default*/
	(void*)&sgPara.u16Driver7Vol,
};

static const xPrmDefStruct cParaDriver8Vol[] = 
{
	PARA_Driver8Vol,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Driver8Vol),
	0,	 	    	/*min*/
	65535,     		/*max*/
	12840,    		  	/*default*/
	(void*)&sgPara.u16Driver8Vol,
};

static const xPrmDefStruct cParaDriver9Vol[] = 
{
	PARA_Driver9Vol,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Driver9Vol),
	0,	 	    	/*min*/
	65535,     		/*max*/
	12840,    		  	/*default*/
	(void*)&sgPara.u16Driver9Vol,
};

static const xPrmDefStruct cParaDriver10Vol[] = 
{
	PARA_Driver10Vol,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Driver10Vol),
	0,	 	    	/*min*/
	65535,     		/*max*/
	12840,    		  	/*default*/
	(void*)&sgPara.u16Driver10Vol,
};
/*LogLevel*/
static const xPrmDefStruct cParaLogLevel[] = 
{
	PARA_LogLevel,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16LogLevel),
	0, 	    	/*min*/
	65535,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16LogLevel,
};
/*LogModel*/
static const xPrmDefStruct cParaLogModel[] = 
{
	PARA_LogModel,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16LogModel),
	0, 	    	/*min*/
	65535,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16LogModel,
};
/*MotorMaxSpd*/
static const xPrmDefStruct cParaMotorMaxSpd[] = 
{
	PARA_MotorMaxSpd,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16MotorMaxSpd),
	0, 	    	/*min*/
	65535,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16MotorMaxSpd,
};
/*Analog3DeadZoneMinVal*/
static const xPrmDefStruct cParaAnalog3DeadZoneMinVal[] = 
{
	PARA_Analog3DeadZoneMinVal,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Analog3DeadZoneMinVal),
	0, 	    	/*min*/
	4096,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16Analog3DeadZoneMinVal,
};
/*Analog3DeadZoneMaxVal*/
static const xPrmDefStruct cParaAnalog3DeadZoneMaxVal[] = 
{
	PARA_Analog3DeadZoneMaxVal,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Analog3DeadZoneMaxVal),
	0, 	    	/*min*/
	4096,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16Analog3DeadZoneMaxVal,
};
/*Analog3MidVal*/
static const xPrmDefStruct cParaAnalog3MidVal[] = 
{
	PARA_Analog3MidVal,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Analog3MidVal),
	0, 	    	/*min*/
	4096,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16Analog3MidVal,
};
static const xPrmDefStruct cParaThrottleType[] = 
{
	PARA_ThrottleType,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16ThrottleType),
	0,	 	    	/*min*/
	4096,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16ThrottleType,
};

static const xPrmDefStruct cParaThrottleFDeadZoneMinVal[] = 
{
	PARA_ThrottleFDeadZoneMinVal,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16ThrottleFDeadZoneMinVal),
	0,	 	    	/*min*/
	4096,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16ThrottleFDeadZoneMinVal,
};

static const xPrmDefStruct cParaThrottleFDeadZoneMaxVal[] = 
{
	PARA_ThrottleFDeadZoneMaxVal,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16ThrottleFDeadZoneMaxVal),
	0,	 	    	/*min*/
	4096,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16ThrottleFDeadZoneMaxVal,
};

static const xPrmDefStruct cParaThrottleFMidVal[] = 
{
	PARA_ThrottleFMidVal,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16ThrottleFMidVal),
	0,	 	    	/*min*/
	4096,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16ThrottleFMidVal,
};

static const xPrmDefStruct cParaThrottleBDeadZoneMinVal[] = 
{
	PARA_ThrottleBDeadZoneMinVal,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16ThrottleBDeadZoneMinVal),
	0,	 	    	/*min*/
	4096,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16ThrottleBDeadZoneMinVal,
};

static const xPrmDefStruct cParaThrottleBDeadZoneMaxVal[] = 
{
	PARA_ThrottleBDeadZoneMaxVal,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16ThrottleBDeadZoneMaxVal),
	0,	 	    	/*min*/
	4096,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16ThrottleBDeadZoneMaxVal,
};

static const xPrmDefStruct cParaThrottleBMidVal[] = 
{
	PARA_ThrottleBMidVal,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16ThrottleBMidVal),
	0,	 	    	/*min*/
	4096,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16ThrottleBMidVal,
};

static const xPrmDefStruct cParaBrakeType[] = 
{
	PARA_BrakeType,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16BrakeType),
	0,	 	    	/*min*/
	4096,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16BrakeType,
};

static const xPrmDefStruct cParaBrakeFDeadZoneMinVal[] = 
{
	PARA_BrakeFDeadZoneMinVal,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16BrakeFDeadZoneMinVal),
	0,	 	    	/*min*/
	4096,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16BrakeFDeadZoneMinVal,
};

static const xPrmDefStruct cParaBrakeFDeadZoneMaxVal[] = 
{
	PARA_BrakeFDeadZoneMaxVal,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16BrakeFDeadZoneMaxVal),
	0,	 	    	/*min*/
	4096,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16BrakeFDeadZoneMaxVal,
};

static const xPrmDefStruct cParaBrakeFMidVal[] = 
{
	PARA_BrakeFMidVal,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16BrakeFMidVal),
	0,	 	    	/*min*/
	4096,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16BrakeFMidVal,
};

static const xPrmDefStruct cParaBrakeBDeadZoneMinVal[] = 
{
	PARA_BrakeBDeadZoneMinVal,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16BrakeBDeadZoneMinVal),
	0,	 	    	/*min*/
	4096,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16BrakeBDeadZoneMinVal,
};

static const xPrmDefStruct cParaBrakeBDeadZoneMaxVal[] = 
{
	PARA_BrakeBDeadZoneMaxVal,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16BrakeBDeadZoneMaxVal),
	0,	 	    	/*min*/
	4096,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16BrakeBDeadZoneMaxVal,
};

static const xPrmDefStruct cParaBrakeBMidVal[] = 
{
	PARA_BrakeBMidVal,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16BrakeBMidVal),
	0,	 	    	/*min*/
	4096,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16BrakeBMidVal,
};
/*Reserve16*/
static const xPrmDefStruct cParaReserve16[] = 
{
	PARA_Reserve16,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Reserve16),
	0, 	    	/*min*/
	65535,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16Reserve16,
};
/*Reserve17*/
static const xPrmDefStruct cParaReserve17[] = 
{
	PARA_Reserve17,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Reserve17),
	0, 	    	/*min*/
	65535,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16Reserve17,
};
static const xPrmDefStruct cParaGear1Spd[] = 
{
	PARA_Gear1Spd,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Gear1Spd),
	0,	 	    	/*min*/
	4096,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16Gear1Spd,
};

static const xPrmDefStruct cParaGear2Spd[] = 
{
	PARA_Gear2Spd,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Gear2Spd),
	0,	 	    	/*min*/
	4096,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16Gear2Spd,
};

static const xPrmDefStruct cParaGear3Spd[] = 
{
	PARA_Gear3Spd,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Gear3Spd),
	0,	 	    	/*min*/
	4096,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16Gear3Spd,
};

static const xPrmDefStruct cParaGear4Spd[] = 
{
	PARA_Gear4Spd,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Gear4Spd),
	0,	 	    	/*min*/
	4096,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16Gear4Spd,
};

static const xPrmDefStruct cParaRatioOfTransmission[] = 
{
	PARA_RatioOfTransmission,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16RatioOfTransmission),
	0,	 	    	/*min*/
	4096,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16RatioOfTransmission,
};

static const xPrmDefStruct cParaMaintenancePeriod[] = 
{
	PARA_MaintenancePeriod,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16MaintenancePeriod),
	0,	 	    	/*min*/
	4096,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16MaintenancePeriod,
};

static const xPrmDefStruct cParaRemotePara[] = 
{
	PARA_RemotePara,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16RemotePara),
	0,	 	    	/*min*/
	4096,     		/*max*/
	0x02,    		/*default: stop = 0, start = 1, Mode = 0*/
	(void*)&sgPara.u16RemotePara,
};

static const xPrmDefStruct cParaPumpMotorGear1[] = 
{
	PARA_PumpMotorGear1,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PumpMotorGear1),
	0,	 	    	/*min*/
	4096,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16PumpMotorGear1,
};

static const xPrmDefStruct cParaPumpMotorGear2[] = 
{
	PARA_PumpMotorGear2,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16PumpMotorGear2),
	0,	 	    	/*min*/
	4096,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16PumpMotorGear2,
};

static const xPrmDefStruct cParaTurnWithDecStartAngle[] = 
{
	PARA_TurnWithDecStartAngle,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16TurnWithDecStartAngle),
	0,	 	    	/*min*/
	4096,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16TurnWithDecStartAngle,
};

static const xPrmDefStruct cParaTurnWithDecEndAngle[] = 
{
	PARA_TurnWithDecEndAngle,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16TurnWithDecEndAngle),
	0,	 	    	/*min*/
	4096,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16TurnWithDecEndAngle,
};

static const xPrmDefStruct cParaAngleWithStartSpdPer[] = 
{
	PARA_AngleWithStartSpdPer,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16AngleWithStartSpdPer),
	0,	 	    	/*min*/
	4096,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16AngleWithStartSpdPer,
};

static const xPrmDefStruct cParaAngleWithEndSpdPer[] = 
{
	PARA_AngleWithEndSpdPer,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16AngleWithEndSpdPer),
	0,	 	    	/*min*/
	4096,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16AngleWithEndSpdPer,
};

static const xPrmDefStruct cParaHourCountPowerOn[] = 
{
	PARA_HourCountPowerOn,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16HourCountPowerOn),
	0,	 	    	/*min*/
	65535,     		/*max*/		//修改小时计相关配置参数
	0,    		  	/*default*/
	(void*)&sgPara.u16HourCountPowerOn,
};
/*Reserve18*/
static const xPrmDefStruct cParaReserve18[] = 
{
	PARA_Reserve18,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Reserve18),
	0, 	    	/*min*/
	65535,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16Reserve18,
};
/*Reserve19*/
static const xPrmDefStruct cParaReserve19[] = 
{
	PARA_Reserve19,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Reserve19),
	0, 	    	/*min*/
	65535,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16Reserve19,
};
/*Reserve20*/
static const xPrmDefStruct cParaReserve20[] = 
{
	PARA_Reserve20,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Reserve20),
	0, 	    	/*min*/
	65535,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16Reserve20,
};
/*Reserve21*/
static const xPrmDefStruct cParaReserve21[] = 
{
	PARA_Reserve21,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Reserve21),
	0, 	    	/*min*/
	65535,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16Reserve21,
};
/*Reserve22*/
static const xPrmDefStruct cParaReserve22[] = 
{
	PARA_Reserve22,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Reserve22),
	0, 	    	/*min*/
	65535,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16Reserve22,
};
/*Reserve23*/
static const xPrmDefStruct cParaReserve23[] = 
{
	PARA_Reserve23,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Reserve23),
	0, 	    	/*min*/
	65535,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16Reserve23,
};
/*Reserve24*/
static const xPrmDefStruct cParaReserve24[] = 
{
	PARA_Reserve24,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Reserve24),
	0, 	    	/*min*/
	65535,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16Reserve24,
};
/*Reserve25*/
static const xPrmDefStruct cParaReserve25[] = 
{
	PARA_Reserve25,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Reserve25),
	0, 	    	/*min*/
	65535,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16Reserve25,
};
/*Reserve26*/
static const xPrmDefStruct cParaReserve26[] = 
{
	PARA_Reserve26,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Reserve26),
	0, 	    	/*min*/
	65535,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16Reserve26,
};
/*Reserve27*/
static const xPrmDefStruct cParaReserve27[] = 
{
	PARA_Reserve27,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16Reserve27),
	0, 	    	/*min*/
	65535,     	/*max*/
	0,    		/*default*/
	(void*)&sgPara.u16Reserve27,
};

static const xPrmDefStruct cParaErrCode0[] = 
{
	PARA_ErrCode0,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16ErrCode[0]),
	0,	 	    	/*min*/
	99,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16ErrCode[0],
//	"Speed",
//	"Sets Ls percentage of the Typical Max Speed 0~100%--0~32767",
};
static const xPrmDefStruct cParaErrCode1[] = 
{
	PARA_ErrCode1,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16ErrCode[1]),
	0,	 	    	/*min*/
	99,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16ErrCode[1],
//	"Speed",
//	"Larger values create a softer reversal from regen braking to drive when near zero speed.",
};
static const xPrmDefStruct cParaErrCode2[] = 
{
	PARA_ErrCode2,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16ErrCode[2]),
	0,	 	    	/*min*/
	99,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16ErrCode[2],
//	"Speed",
//	"Acc time for maxspd vary.  0.1~30.0 sec -- 100~30000",
};
static const xPrmDefStruct cParaErrCode3[] = 
{
	PARA_ErrCode3,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16ErrCode[3]),
	0,	 	    	/*min*/
	99,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16ErrCode[3],
//	"Speed",
//	"Dec time for maxspd vary.  0.1~30.0 sec -- 100~30000",
};
static const xPrmDefStruct cParaErrCode4[] = 
{
	PARA_ErrCode4,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16ErrCode[4]),
	0,	 	    	/*min*/
	99,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16ErrCode[4],
//	"Speed",
//	"Sets gear 1 percentage of the Typical Max Speed 0~100%--0~100",
};
static const xPrmDefStruct cParaErrCode5[] = 
{
	PARA_ErrCode5,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16ErrCode[5]),
	0,	 	    	/*min*/
	99,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16ErrCode[5],
//	"Speed",
//	"Sets gear 2 percentage of the Typical Max Speed 0~100%--0~100",
};
static const xPrmDefStruct cParaErrCode6[] = 
{
	PARA_ErrCode6,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16ErrCode[6]),
	0,	 	    	/*min*/
	99,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16ErrCode[6],
//	"Speed",
//	"Sets gear 3 percentage of the Typical Max Speed 0~100%--0~100",
};
static const xPrmDefStruct cParaErrCode7[] = 
{
	PARA_ErrCode7,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16ErrCode[7]),
	0,	 	    	/*min*/
	99,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16ErrCode[7],
//	"Speed",
//	"Sets gear 4 percentage of the Typical Max Speed 0~100%--0~100",
};
	/* Restraint */
static const xPrmDefStruct cParaErrCode8[] = 
{
	PARA_ErrCode8,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16ErrCode[8]),
	0,	 	    	/*min*/
	99,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16ErrCode[8],
//	"Speed",
//	"Defines the speed below which a much slower decel rate is used.unit 0.01Hz",
};
static const xPrmDefStruct cParaErrCode9[] = 
{
	PARA_ErrCode9,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16ErrCode[9]),
	0,	 	    	/*min*/
	99,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16ErrCode[9],
//	"Speed",
//	"Defines the dec rate when speed below SoftStopSpeed. unit 1%",
};
static const xPrmDefStruct cParaErrCode10[] = 
{
	PARA_ErrCode10,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16ErrCode[10]),
	0,	 	    	/*min*/
	99,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16ErrCode[10],
//	"Speed",
//	"Determines the speed below which the EM brake will be commanded to set.unit rpm",
};
static const xPrmDefStruct cParaErrCode11[] = 
{
	PARA_ErrCode11,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16ErrCode[11]),
	0,	 	    	/*min*/
	99,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16ErrCode[11],
//	"Speed",
//	"Determines how long the position hold function is allowed to operate before the EM brake is set.unit ms",
};
static const xPrmDefStruct cParaErrCode12[] = 
{
	PARA_ErrCode12,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16ErrCode[12]),
	0,	 	    	/*min*/
	99,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16ErrCode[12],
//	"Speed",
//	"Ramp stop mode: 1-stop,2- move~stop~move~;3- stop-move~~",
};
static const xPrmDefStruct cParaErrCode13[] = 
{
	PARA_ErrCode13,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16ErrCode[13]),
	0,	 	    	/*min*/
	99,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16ErrCode[13],
//	"Speed",
//	"Defines the Acc/dec rate when speed error below SoftStopSpeed",
};
static const xPrmDefStruct cParaErrCode14[] = 
{
	PARA_ErrCode14,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16ErrCode[14]),
	0,	 	    	/*min*/
	99,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16ErrCode[14],
//	"Speed",
//	"The acc ratio for Fast Mode  12.5%~800% -- 512~32767",
};
static const xPrmDefStruct cParaErrCodeMax[] = 
{
	PARA_ErrCodeMax,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16ErrCode[PARA_ErrCodeMax - PARA_ErrCode0]),
	0,	 	    	/*min*/
	99,     		/*max*/
	0,    		  	/*default*/
	(void*)&sgPara.u16ErrCode[PARA_ErrCodeMax - PARA_ErrCode0],
//	"Speed",
//	"The dec ratio for Fast Mode  12.5%~800% -- 512~32767",
};
#if 0
static const xPrmDefStruct cParaBatteryFlag1[] = 
{
//	255,
	PARA_NoBatteryFlag1,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16BatteryFlag1),
	0,	 	    	//min 0
	999,    		//max 9999
	0,  //default
	(void*)&sgPara.u16BatteryFlag1,
//	"Version",
//	"User & Vehicle",
};
static const xPrmDefStruct cParaBatteryCount1[] = 
{
//	255,
	PARA_NoBatteryCount1,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16BatteryCount1),
	0,	 	    	//min 0
	999,    		//max 9999
	0,  //default
	(void*)&sgPara.u16BatteryCount1,
//	"Version",
//	"User & Vehicle",
};
static const xPrmDefStruct cParaBatteryFlag2[] = 
{
//	255,
	PARA_NoBatteryFlag2,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16BatteryFlag2),
	0,	 	    	//min 0
	999,    		//max 9999
	0,  //default
	(void*)&sgPara.u16BatteryFlag2,
//	"Version",
//	"User & Vehicle",
};

static const xPrmDefStruct cParaBatteryCount2[] = 
{
//	255,
	PARA_NoBatteryCount2,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16BatteryCount2),
	0,	 	    	//min 0
	999,    		//max 9999
	0,  //default
	(void*)&sgPara.u16BatteryCount2,
//	"Version",
//	"User & Vehicle",
};
#endif
static const xPrmDefStruct cParaUserType[] = 
{
//	255,
	PARA_UserType,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(sgPara.u16UserType),
	0,	 	    	//min 0
	999,    		//max 9999
	0,  //default
	(void*)&sgPara.u16UserType,
};


static const xPrmDefStruct cMonitorParaAngleValue[] = 
{
	PARA_AngleValue,
	PRM_ATTR_SIZE(sgPara.i16AngleValue),
	0,	 	    	//min 0
	999,    		//max 9999
	0,  //default
	(void*)&sgPara.i16AngleValue,
//	"Version",
//	"User & Vehicle",
};

static const xPrmDefStruct cMonitorParaPressureVlaue1[] = 
{
	PARA_PressureVlaue1,
	PRM_ATTR_SIZE(sgPara.u16PressureVlaue1),
	0,	 	    	//min 0
	999,    		//max 9999
	0,  //default
	(void*)&sgPara.u16PressureVlaue1,
//	"Version",
//	"User & Vehicle",
};

static const xPrmDefStruct cMonitorParaPressureVlaue2[] = 
{
	PARA_PressureVlaue2,
	PRM_ATTR_SIZE(sgPara.u16PressureVlaue2),
	0,	 	    	//min 0
	999,    		//max 9999
	0,  //default
	(void*)&sgPara.u16PressureVlaue2,
//	"Version",
//	"User & Vehicle",
};

static const xPrmDefStruct cMonitorParaLoadRate[] = 
{
	PARA_LoadRate,
	PRM_ATTR_SIZE(sgPara.u16LoadRate),
	0,	 	    	//min 0
	999,    		//max 9999
	0,  //default
	(void*)&sgPara.u16LoadRate,
//	"Version",
//	"User & Vehicle",
};

static const xPrmDefStruct cMonitorParaCalibrationStatus[] = 
{
	PARA_CalibrationStatus,
	PRM_ATTR_SIZE(sgPara.u16CalibrationStatus),
	0,	 	    	//min 0
	999,    		//max 9999
	0,  //default
	(void*)&sgPara.u16CalibrationStatus,
//	"Version",
//	"User & Vehicle",
};

static const xPrmDefStruct cMonitorParaForwardValveCurrent[] = 
{
	PARA_ForwardValveCurrent,
	PRM_ATTR_SIZE(sgPara.u16ForwardValveCurrent),
	0,	 	    	//min 0
	999,    		//max 9999
	0,  //default
	(void*)&sgPara.u16ForwardValveCurrent,
//	"Version",
//	"User & Vehicle",
};

static const xPrmDefStruct cMonitorParaBackValveCurrent[] = 
{
	PARA_BackValveCurrent,
	PRM_ATTR_SIZE(sgPara.u16BackValveCurrent),
	0,	 	    	//min 0
	999,    		//max 9999
	0,  //default
	(void*)&sgPara.u16BackValveCurrent,
//	"Version",
//	"User & Vehicle",
};

static const xPrmDefStruct cMonitorParaLiftValveCurrent[] = 
{
	PARA_LiftValveCurrent,
	PRM_ATTR_SIZE(sgPara.u16LiftValveCUrrent),
	0,	 	    	//min 0
	999,    		//max 9999
	0,  //default
	(void*)&sgPara.u16LiftValveCUrrent,
//	"Version",
//	"User & Vehicle",
};

static const xPrmDefStruct cMonitorParaPropValveCurrent[] = 
{
	PARA_PropValveCurrent,
	PRM_ATTR_SIZE(sgPara.u16PropValveCurrent),
	0,	 	    	//min 0
	999,    		//max 9999
	0,  //default
	(void*)&sgPara.u16PropValveCurrent,
//	"Version",
//	"User & Vehicle",
};

static const xPrmDefStruct cMonitorParaOnOffValveCurrent[] = 
{
	PARA_OnOffValveCurrent,
	PRM_ATTR_SIZE(sgPara.u16OnOffValveCurrent),
	0,	 	    	//min 0
	999,    		//max 9999
	0,  //default
	(void*)&sgPara.u16OnOffValveCurrent,
//	"Version",
//	"User & Vehicle",
};

static const xPrmDefStruct cMonitorParaTurnRightValveCurrent[] = 
{
	PARA_TurnRightValveCurrent,
	PRM_ATTR_SIZE(sgPara.u16TurnRightValveCurrent),
	0,	 	    	//min 0
	999,    		//max 9999
	0,  //default
	(void*)&sgPara.u16TurnRightValveCurrent,
//	"Version",
//	"User & Vehicle",
};

static const xPrmDefStruct cMonitorParaTurnLeftValveCurrent[] = 
{
	PARA_TurnLeftValveCurrent,
	PRM_ATTR_SIZE(sgPara.u16TurnLeftValveCurrent),
	0,	 	    	//min 0
	999,    		//max 9999
	0,  //default
	(void*)&sgPara.u16TurnLeftValveCurrent,
//	"Version",
//	"User & Vehicle",
};

static const xPrmDefStruct cMonitorParaBrakeValveCurrent[] = 
{
	PARA_BrakeValveCurrent,
	PRM_ATTR_SIZE(sgPara.u16BrakeValveCurrent),
	0,	 	    	//min 0
	999,    		//max 9999
	0,  //default
	(void*)&sgPara.u16BrakeValveCurrent,
//	"Version",
//	"User & Vehicle",
};

static const xPrmDefStruct cMonitorParaBatteryVoltage[] = 
{
	PARA_BatteryVoltage,
	PRM_ATTR_SIZE(sgPara.u16BatteryVoltage),
	0,	 	    	//min 0
	999,    		//max 9999
	0,  //default
	(void*)&sgPara.u16BatteryVoltage,
//	"Version",
//	"User & Vehicle",
};
static const xPrmDefStruct cMonitorParaExtSignal[] = 
{
	PARA_ExtSignal,
	PRM_ATTR_SIZE(sgPara.u16ExtSignal),
	0,	 	    	//min 0
	999,    		//max 9999
	0,  //default
	(void*)&sgPara.u16ExtSignal,
//	"Version",
//	"User & Vehicle",
};

static const xPrmDefStruct cMonitorParaPcuKeyInfo[] = 
{
	PARA_PcuKeyInfo,
	PRM_ATTR_SIZE(sgPara.u16PcuKeyInfo),
	0,	 	    	//min 0
	999,    		//max 9999
	0,  //default
	(void*)&sgPara.u16PcuKeyInfo,
//	"Version",
//	"User & Vehicle",
};

static const xPrmDefStruct cMonitorParaOutCtrlInfo[] = 
{
	PARA_OutCtrlInfo,
	PRM_ATTR_SIZE(sgPara.u16OutCtrlInfo),
	0,	 	    	//min 0
	999,    		//max 9999
	0,  //default
	(void*)&sgPara.u16OutCtrlInfo,
//	"Version",
//	"User & Vehicle",
};

static const xPrmDefStruct cMonitorParaMotorSpd[] = 
{
	PARA_MotorSpd,
	PRM_ATTR_SIZE(sgPara.u16MotorSpd),
	0,	 	    	//min 0
	999,    		//max 9999
	0,  //default
	(void*)&sgPara.u16MotorSpd,
//	"Version",
//	"User & Vehicle",
};

static const xPrmDefStruct cMonitorParaHandleAnalog[] = 
{
	PARA_HandleAnalog,
	PRM_ATTR_SIZE(sgPara.u16HandleAnalog),
	0,	 	    	//min 0
	999,    		//max 9999
	0,  //default
	(void*)&sgPara.u16HandleAnalog,
//	"Version",
//	"User & Vehicle",
};

static const xPrmDefStruct cMonitorParaBmsSoc[] = 
{
	PARA_BmsSoc,
	PRM_ATTR_SIZE(sgPara.u16BmsSoc),
	0,	 	    	//min 0
	999,    		//max 9999
	0,  //default
	(void*)&sgPara.u16BmsSoc,
//	"Version",
//	"User & Vehicle",
};

static const xPrmDefStruct cMonitorParaBmsVoltage[] = 
{
	PARA_BmsVoltage,
	PRM_ATTR_SIZE(sgPara.u16BmsVoltage),
	0,	 	    	//min 0
	999,    		//max 9999
	0,  //default
	(void*)&sgPara.u16BmsVoltage,
//	"Version",
//	"User & Vehicle",
};

static const xPrmDefStruct cMonitorParaTemporaryUnlock[] = 
{
	PARA_TemporaryUnlock,
	PRM_ATTR_SIZE(sgPara.u16TemporaryUnlock),
	0,	 	    	//min 0
	999,    		//max 9999
	0,  //default
	(void*)&sgPara.u16TemporaryUnlock,
//	"Version",
//	"User & Vehicle",
};

static const xPrmDefStruct cMonitorParaPlatfromHeartQuery[] = 
{
	PARA_PlatfromHeartQuery,
	PRM_ATTR_SIZE(sgPara.u16PlatfromHeartQuery),
	0,	 	    	//min 0
	999,    		//max 9999
	0,  //default
	(void*)&sgPara.u16PlatfromHeartQuery,
//	"Version",
//	"User & Vehicle",
};

static const xPrmDefStruct cMonitorParaSelfHeartQuery[] = 
{
	PARA_SelfHeartQuery,
	PRM_ATTR_SIZE(sgPara.u16SelfHeartQuery),
	0,	 	    	//min 0
	999,    		//max 9999
	0,  //default
	(void*)&sgPara.u16SelfHeartQuery,
//	"Version",
//	"User & Vehicle",
};

static const xPrmDefStruct cMonitorParaEcuLockState[] = 
{
	PARA_EcuLockState,
	PRM_ATTR_SIZE(sgPara.u16EcuLockState),
	0,	 	    	//min 0
	999,    		//max 9999
	0,  //default
	(void*)&sgPara.u16EcuLockState,
//	"Version",
//	"User & Vehicle",
};

static const xPrmDefStruct cMonitorParaTmpLockState[] = 
{
	PARA_TmpLockState,
	PRM_ATTR_SIZE(sgPara.u16TmpLockState),
	0,	 	    	//min 0
	999,    		//max 9999
	0,  //default
	(void*)&sgPara.u16TmpLockState,
//	"Version",
//	"User & Vehicle",
};

static const xPrmDefStruct cMonitorParaSwi[] = 
{
	PARA_Swi,
	PRM_ATTR_SIZE(sgPara.u16Swi),
	0,	 	    	//min 0
	65535,    		//max 9999
	0,  //default
	(void*)&sgPara.u16Swi,
};

static const xPrmDefStruct cMonitorParaDoSwi[] = 
{
	PARA_DoSwi,
	PRM_ATTR_SIZE(sgPara.u16DoSwi),
	0,	 	    	//min 0
	65535,    		//max 9999
	0,  //default
	(void*)&sgPara.u16DoSwi,
};

static const xPrmDefStruct cMonitorParaAi1[] = 
{
	PARA_Ai1,
	PRM_ATTR_SIZE(sgPara.u16Ai1),
	0,	 	    	//min 0
	65535,    		//max 9999
	0,  //default
	(void*)&sgPara.u16Ai1,
};

static const xPrmDefStruct cMonitorParaAi2[] = 
{
	PARA_Ai2,
	PRM_ATTR_SIZE(sgPara.u16Ai2),
	0,	 	    	//min 0
	65535,    		//max 9999
	0,  //default
	(void*)&sgPara.u16Ai2,
};

static const xPrmDefStruct cMonitorParaAi3[] = 
{
	PARA_Ai3,
	PRM_ATTR_SIZE(sgPara.u16Ai3),
	0,	 	    	//min 0
	65535,    		//max 9999
	0,  //default
	(void*)&sgPara.u16Ai3,
};

static const xPrmDefStruct cMonitorParaAi4[] = 
{
	PARA_Ai4,
	PRM_ATTR_SIZE(sgPara.u16Ai4),
	0,	 	    	//min 0
	65535,    		//max 9999
	0,  //default
	(void*)&sgPara.u16Ai4,
};

static const xPrmDefStruct cMonitorParaAi5[] = 
{
	PARA_Ai5,
	PRM_ATTR_SIZE(sgPara.u16Ai5),
	0,	 	    	//min 0
	65535,    		//max 9999
	0,  //default
	(void*)&sgPara.u16Ai5,
};

static const xPrmDefStruct cMonitorParaVbus[] = 
{
	PARA_Vbus,
	PRM_ATTR_SIZE(sgPara.u16Vbus),
	0,	 	    	//min 0
	65535,    		//max 9999
	0,  //default
	(void*)&sgPara.u16Vbus,
};

static const xPrmDefStruct cMonitorParaKsi[] = 
{
	PARA_Ksi,
	PRM_ATTR_SIZE(sgPara.u16Ksi),
	0,	 	    	//min 0
	65535,    		//max 9999
	0,  //default
	(void*)&sgPara.u16Ksi,
};

static const xPrmDefStruct cMonitorParaErrCode[] = 
{
	PARA_ErrCode,
	PRM_ATTR_SIZE(sgPara.u16ErrCodeA),
	0,	 	    	//min 0
	65535,    		//max 9999
	0,  //default
	(void*)&sgPara.u16ErrCodeA,
};

static const xPrmDefStruct cMonitorParaPropCurrent1[] = 
{
	PARA_PropCurrent1,
	PRM_ATTR_SIZE(sgPara.u16PropCurrent1),
	0,	 	    	//min 0
	65535,    		//max 9999
	0,  //default
	(void*)&sgPara.u16PropCurrent1,
};

static const xPrmDefStruct cMonitorParaPropCurrent2[] = 
{
	PARA_PropCurrent2,
	PRM_ATTR_SIZE(sgPara.u16PropCurrent2),
	0,	 	    	//min 0
	65535,    		//max 9999
	0,  //default
	(void*)&sgPara.u16PropCurrent2,
};

static const xPrmDefStruct cMonitorParaMotorCmd[] = 
{
	PARA_MotorCmd,
	PRM_ATTR_SIZE(sgPara.u16MotorCmd),
	0,	 	    	//min 0
	65535,    		//max 9999
	0,  //default
	(void*)&sgPara.u16MotorCmd,
};

static const xPrmDefStruct cMonitorParaPumpCmd[] = 
{
	PARA_PumpCmd,
	PRM_ATTR_SIZE(sgPara.u16PumpCmd),
	0,	 	    	//min 0
	65535,    		//max 9999
	0,  //default
	(void*)&sgPara.u16PumpCmd,
};

static const xPrmDefStruct cMonitorParaMotorState[] = 
{
	PARA_MotorState,
	PRM_ATTR_SIZE(sgPara.u16MotorState),
	0,	 	    	//min 0
	65535,    		//max 9999
	0,  //default
	(void*)&sgPara.u16MotorState,
};

static const xPrmDefStruct cMonitorParaSteerAngle[] = 
{
	PARA_SteerAngle,
	PRM_ATTR_SIZE(sgPara.u16SteerAngle),
	0,	 	    	//min 0
	65535,    		//max 9999
	0,  //default
	(void*)&sgPara.u16SteerAngle,
};

static const xPrmDefStruct cMonitorParaMotorSpeed[] = 
{
	PARA_MotorSpeed,
	PRM_ATTR_SIZE(sgPara.u16MotorSpeed),
	0,	 	    	//min 0
	65535,    		//max 9999
	0,  //default
	(void*)&sgPara.u16MotorSpeed,
};


const xPrmDefStruct* cMonitorPara_Table[] = 
{
	cMonitorParaAngleValue,	
	cMonitorParaPressureVlaue1,
	cMonitorParaPressureVlaue2,
	cMonitorParaLoadRate,
	cMonitorParaCalibrationStatus,
	cMonitorParaForwardValveCurrent,
	cMonitorParaBackValveCurrent,
	cMonitorParaLiftValveCurrent,
	cMonitorParaPropValveCurrent,
	cMonitorParaOnOffValveCurrent,
	cMonitorParaTurnRightValveCurrent,
	cMonitorParaTurnLeftValveCurrent,
	cMonitorParaBrakeValveCurrent,
	cMonitorParaBatteryVoltage,			
	cMonitorParaExtSignal,				
	cMonitorParaPcuKeyInfo,			
	cMonitorParaOutCtrlInfo,				
	cMonitorParaMotorSpd,				
	cMonitorParaHandleAnalog,			
	cMonitorParaBmsSoc,					
	cMonitorParaBmsVoltage,	
	cMonitorParaTemporaryUnlock,			
	cMonitorParaPlatfromHeartQuery,		
	cMonitorParaSelfHeartQuery,			
	cMonitorParaEcuLockState,			
	cMonitorParaTmpLockState,
	cMonitorParaSwi,
	cMonitorParaDoSwi,
	cMonitorParaAi1,
	cMonitorParaAi2,
	cMonitorParaAi3,
	cMonitorParaAi4,
	cMonitorParaAi5,
	cMonitorParaVbus,
	cMonitorParaKsi,
	cMonitorParaErrCode,
	cMonitorParaPropCurrent1,
	cMonitorParaPropCurrent2,
	cMonitorParaMotorCmd,
	cMonitorParaPumpCmd,
	cMonitorParaMotorState,
	cMonitorParaSteerAngle,
	cMonitorParaMotorSpeed
	
};

const xPrmDefStruct* cPara_Table[PARA_VOLUME_NUM + 1] = 
{
	cParaFastDriveSpeed,
	cParaSlowDriceSPeed,		
	cParaDriveSpeedAfterLift,	
	cParaLiftSpeed,				
	cParaMaxTurnSpeed,			
	cParaTurnPowerLimit,			
	cParaDeadZoneAdjust,			

	cParaBrakeFastDrive,			
	cParaBrakeSlowDrive,			
	cParaBrakeDriveAfterLift,	
	cParaBrakeLift,				
	cParaBrakeLower,				
	cParaBrakeTurn,				
	cParaBrakeAntiPinch,	

	cParaReserve1,
	cParaReserve2,	

	cParaLowerSpeed,							
	cParaOverLoadStabilityDelay,	
	cParaDynamicOverLoadPercent,	
	cParaStaticOverLoadPercent,	
	cParaMaxDifferencePercent,	
	cParaDriveMotorEncoder,		
	cParaMotorHighSpeedDeceRate,	
	cParaMotorLowSpeedDeceRate,	
	cParaVoiceAlarmVolume,		

	cParaCurveFastDrive,			/*23*/			
	cParaCurveSlowDrive,			
	cParaCurveDriveAfterLift,	
	cParaCurveLift,				
	cParaCurveLower,

	cParaReserve3,
	cParaReserve4,
	
	cParaCurveTurn,				

	cParaAccAndDecFastDrive,		/*29*/		
	cParaAccAndDecSlowDrive,		
	cParaAccAndDecAfterLift,		
	cParaAccAndDecLift,			
	cParaAccAndDecLower,			
	cParaAccAndDecTurn,			
	cParaAccAndDecAntiPinch,		

	cParaPumpMotorEncoder,			/*36*/		

	cParaVehicleType,							
	cParaVehcileHeight,			
	cParaPressureSensorType,		
	cParaPitProtectFunc,			
	cParaAntiPinchFunc,	

	cParaReserve5,
	cParaReserve6,
	
	cParaActAlmFunc,				
	cParaWeighFunc,				
	cParaParallelValveReverseFunc,
	cParaLowBatAlmFunc,			
	cParaLowVolAlmTime,			
	cParaLowVolShutDownTime,		
	cParaUpperCtlButSleep,		
	cParaLanguageType,			
	cParaBatteryType,			
	cParaSpeakerSync,			
	cParaLowerPumpType,			
	cParaPressureType,			
	cParaAngleSimulationLimit,	
	cParaLiftReverseFunc,	

	cParaReserve7,
	cParaReserve8,
	
	cParaFourPointWeightFunc,	
	cParaDriverType,			
	cParaPasswordLock,			
	cParaInAndOutFunc,			
	cParaHeartBeatQueryFunc,		
	cParaLowBatteryMode,			
	cParaAngleSensorSetting,		
	cParaTiltSwitchSetting,		
	cParaAngleSensorType,		
	cParaAnaLogLimitDetSwitch,	

	cParaIsNoLoadCalibration,		/*66*/			
	cParaIsOverLoadCalibration,	

	cParaSetDescentHeightValue,		
	cParaReleaseBrake,

	cParaReserve9,
	cParaReserve10,
	
	cParaSetOutHeight,			
	cParaAngleSimulationUpLimit,	
	cParaAngleSimulationDownLimit,
	
	cParaPumpDriveCurrentLimitRatio0,		
	cParaPumpSpdAccRatio0,
	cParaPropDKp0,						
	cParaPropDKi0,						
	cParaPropDMaxCurrent0,				
	cParaPropDMinCurrent0,
	cParaPropDAccPeriod0,
	cParaPropDDitherPeriod0,	
	cParaPropDDitherRatio0,			
	cParaPropValveResistance0,	
	
	cParaReserve11,
	cParaReserve12,
	cParaReserve13,

	cParaPumpDriveCurrentLimitRatio1,		/*82*/	
	cParaPumpSpdAccRatio1,
	cParaPropDKp1,						
	cParaPropDKi1,						
	cParaPropDMaxCurrent1,				
	cParaPropDMinCurrent1,
	cParaPropDAccPeriod1,
	cParaPropDDitherPeriod1,	
	cParaPropDDitherRatio1,			
	cParaPropValveResistance1,	
	
	cParaCanBaudRate,
	cParaEmptyPressure,
	cParaFullPressure,
	cParaDriverFlag,
	
	cParaReserve14,
	cParaReserve15,
	
	cParaMinAngle,
	cParaMaxAngle,
	cParaBatSocPalyBack,
	cParaValveType,
	cParaAnticollisionFunc,
	cParaValueOpenLoopCurrent,
	cParaValueOpenPercentage,
	cParaCanOpenNodeId,
	
	cParaAngleValue0,
	cParaAngleValue1,
	cParaAngleValue2,
	cParaAngleValue3,
	cParaAngleValue4,
	cParaAngleValue5,
	cParaAngleValue6,
	cParaAngleValue7,
	
	cParaDriver1Vol,
	cParaDriver2Vol,
	cParaDriver3Vol,
	cParaDriver4Vol,
	cParaDriver5Vol,
	cParaDriver6Vol,
	cParaDriver7Vol,
	cParaDriver8Vol,
	cParaDriver9Vol,
	cParaDriver10Vol,
	
	cParaLogLevel,
	cParaLogModel,
	cParaMotorMaxSpd,
	
	cParaAnalog3DeadZoneMinVal,
	cParaAnalog3DeadZoneMaxVal,
	cParaAnalog3MidVal,
	
	cParaThrottleType,
	cParaThrottleFDeadZoneMinVal,
	cParaThrottleFDeadZoneMaxVal,
	cParaThrottleFMidVal,
	cParaThrottleBDeadZoneMinVal,
	cParaThrottleBDeadZoneMaxVal,
	cParaThrottleBMidVal,
	
	cParaBrakeType,
	cParaBrakeFDeadZoneMinVal,
	cParaBrakeFDeadZoneMaxVal,
	cParaBrakeFMidVal,
	cParaBrakeBDeadZoneMinVal,
	cParaBrakeBDeadZoneMaxVal,
	cParaBrakeBMidVal,
	
	cParaReserve16,
	cParaReserve17,
	
	cParaGear1Spd,
	cParaGear2Spd,
	cParaGear3Spd,
	cParaGear4Spd,
	
	cParaRatioOfTransmission,
	cParaMaintenancePeriod,
	cParaRemotePara,
	
	cParaPumpMotorGear1,
	cParaPumpMotorGear2,
	
	cParaTurnWithDecStartAngle,
	cParaTurnWithDecEndAngle,
	
	cParaAngleWithStartSpdPer,
	cParaAngleWithEndSpdPer,
	
	cParaHourCountPowerOn,
	
	cParaReserve18,
	cParaReserve19,
	cParaReserve20,
	cParaReserve21,
	cParaReserve22,
	cParaReserve23,
	cParaReserve24,
	cParaReserve25,
	cParaReserve26,
	cParaReserve27,
	
	cParaErrCode0,					
	cParaErrCode1,					
	cParaErrCode2,
	cParaErrCode3,
	cParaErrCode4,
	cParaErrCode5,
	cParaErrCode6,
	cParaErrCode7,
	cParaErrCode8,
	cParaErrCode9,
	cParaErrCode10,
	cParaErrCode11,
	cParaErrCode12,
	cParaErrCode13,
	cParaErrCode14,
	cParaErrCodeMax,
	
	cParaUserType,
	
//	cMonitorParaAngleValue,	
//	cMonitorParaPressureVlaue1,
//	cMonitorParaPressureVlaue2,
//	cMonitorParaLoadRate,
//	cMonitorParaCalibrationStatus,
//	cMonitorParaForwardValveCurrent,
//	cMonitorParaBackValveCurrent,
//	cMonitorParaLiftValveCurrent,
//	cMonitorParaPropValveCurrent,
//	cMonitorParaOnOffValveCurrent,
//	cMonitorParaTurnRightValveCurrent,
//	cMonitorParaTurnLeftValveCurrent,
//	cMonitorParaBrakeValveCurrent,
//	cMonitorParaBatteryVoltage,			
//	cMonitorParaExtSignal,				
//	cMonitorParaPcuKeyInfo,			
//	cMonitorParaOutCtrlInfo,				
//	cMonitorParaMotorSpd,				
//	cMonitorParaHandleAnalog,			
//	cMonitorParaBmsSoc,					
//	cMonitorParaBmsVoltage,	
//	cMonitorParaTemporaryUnlock,			
//	cMonitorParaPlatfromHeartQuery,		
//	cMonitorParaSelfHeartQuery,			
//	cMonitorParaEcuLockState,			
//	cMonitorParaTmpLockState,	
	
	
	
//	cParaBatteryFlag1,
//	cParaBatteryCount1,
//	cParaBatteryFlag2,
//	cParaBatteryCount2,
};
/*******************************************************************************
* Name: ReadParaFromEeprom
* Description: Read parameters from eeprom para area or backup area.
* Input: Index
* Output: 0: success. 1: fail
* 
* Author:
* Date:
* Revision:
*******************************************************************************/
/*static*/ uint16_t u16ReadParaFromEeprom(uint16_t Index)//23.12.5 SJ
{
	uint16_t data, address, ret;
	uint32_t PrmOfs;
	xPrmAttrStruct* pPrmAttr;
	xPrmDefStruct* pPrmDef;
	
	if (Index >= (sizeof(cPara_Table)/sizeof(cPara_Table[0])))
		return 1;
	
	pPrmDef = (xPrmDefStruct*)(cPara_Table[Index]);
	pPrmAttr = (xPrmAttrStruct*)(&pPrmDef->PrmAttr);
	//处理非eprom参数
	if (pPrmAttr->Eprom == 0)
	{
		*((uint16_t*)pPrmDef->pPrmData) = pPrmDef->PrmInitVal;
		return 0;
	}
	//addr overflow protect
	PrmOfs = pPrmDef->PrmNo;
	if (PrmOfs >= EEPROM_PARA_NUM)
	{
		*((uint16_t*)pPrmDef->pPrmData) = pPrmDef->PrmInitVal;
		return 1;
	}
	address = EEPROM_PARA_AREA_ADDR + PrmOfs;	//参数地址计算
	ret = 0;
	
	while(1)
	{
		ret = u16EepromRead(address, &data, 1);
		if(ret == 1)
		{
			*((uint16_t*)pPrmDef->pPrmData) = pPrmDef->PrmInitVal;
			break;
		}

		/* if over limit */
		if (pPrmAttr->Signed == 1)
		{
			if(((int16_t)data > pPrmDef->PrmMaxVal) || ((int16_t)data < pPrmDef->PrmMinVal))
			{
				/* the parameter in eeprom is not in proper range */
				ret = 1;
				//SL_SET(PLC_PARA_OV_LIMIT_ERR);
				
				/* if error, then default the parameter */
				*((uint16_t*)pPrmDef->pPrmData) = pPrmDef->PrmInitVal;
				ret = u16EepromWrite(address, pPrmDef->PrmInitVal, 1);
				break;
			}
			else /*No overflow*/
			{
				*((uint16_t*)(pPrmDef->pPrmData)) = data;	/* set parameter in sram */
				break;
			}
		}
		else //unsign
		{
			if((data > (uint16_t)pPrmDef->PrmMaxVal) || (data < (uint16_t)pPrmDef->PrmMinVal))
			{
				/* the parameter in eeprom is not in proper range */
				ret = 1;
				//SL_SET(PLC_PARA_OV_LIMIT_ERR);

				/* if error, then default the parameter */
				*((uint16_t*)pPrmDef->pPrmData) = pPrmDef->PrmInitVal;
				u16EepromWrite(address, pPrmDef->PrmInitVal, 1);
				break;
			}
			else /*No overflow*/
			{
				*((uint16_t*)(pPrmDef->pPrmData)) = data;	/* set parameter in sram */
				break;
			}
		}
	}
	return ret;
}
//#include "Log.h"
/*******************************************************************************
* Name: SaveParaToEeprom
* Description: Save parameters to eeprom para area or backup area.
* Input: .
* Output: 0: success. 1: fail
* 
* Author:
* Date:
* Revision:
*******************************************************************************/
/*static */uint16_t u16SaveParaToEeprom(uint16_t Index, uint16_t data)
{
	uint16_t dataRd, address, ret;
	uint32_t PrmOfs;
	xPrmAttrStruct* pPrmAttr;
	xPrmDefStruct* pPrmDef;
	
	if (Index >= (sizeof(cPara_Table)/sizeof(cPara_Table[0])))
		return 1;
#ifdef __KFFDEBUG
	if ((Index == 64) && (data != 8000))
		Index = 64;
#endif
	pPrmDef = (xPrmDefStruct*)(cPara_Table[Index]);
	pPrmAttr = (xPrmAttrStruct*)(&pPrmDef->PrmAttr);
	//处理非eprom参数
	if (pPrmAttr->Eprom == 0)
	{
		return 0;
	}
	//addr overflow protect
	PrmOfs = pPrmDef->PrmNo;
	if (PrmOfs >= EEPROM_PARA_NUM)
	{
		return 1;
	}
	address = EEPROM_PARA_AREA_ADDR + PrmOfs;	//参数地址计算

	ret = 0;
	while(1)
	{
		/* if over limit */
		if (pPrmAttr->Signed == 1)
		{
			if(((int16_t)data > pPrmDef->PrmMaxVal) || ((int16_t)data < pPrmDef->PrmMinVal))
			{
				/* the parameter in eeprom is not in proper range */
				ret = 1;
				//SL_SET(PLC_PARA_OV_LIMIT_ERR);
				break;
			}
		}
		else
		{
			if((data > (uint16_t)pPrmDef->PrmMaxVal) || (data < (uint16_t)pPrmDef->PrmMinVal))
			{
				/* the parameter in eeprom is not in proper range */
				ret = 1;
				//SL_SET(PLC_PARA_OV_LIMIT_ERR);
				break;
			}
		}
		/* if not over limit, then save to eeprom */
		/* Read current eprom val out */
		if(u16EepromRead(address, &dataRd, 1) == 1)
		{
			ret = 1;
			//SL_SET(PLC_EEPROM_RW_ERR);
			break;
		}
		/* Check new val equal current val*/
		if (data != dataRd)
		{
			/* write eeprom */
			ret = u16EepromWrite(address, data, 1);
//			//i32LogWrite(ERR, "SAVE To EEPROM Index = %d, data = %d, RdData = %d************************\r\n", Index, data, dataRd);
			break;
		}
		else
		{
			ret = 0;
			break;
		}
	}
	return ret;
}

/*******************************************************************************
* Name: ReadParaValByIndex
* Description: 
* Input: Index
* Output: Para val
* 
* Author:
* Date:
* Revision:
*******************************************************************************/
static uint16_t u16ReadParaValByIndex(uint16_t Index)
{
	if (Index < (sizeof(cPara_Table)/sizeof(cPara_Table[0])))
	{
		u16ReadParaFromEeprom(Index);
		return *((uint16_t*)cPara_Table[Index]->pPrmData);
	}
	else
	{
		//SL_SET(PLC_PARA_OV_INDEX_ERR);
		return 0;
	}
}
/*******************************************************************************
* Name: int32_t i32GetPara(uint16_t u16Index)
* Descriptio: Get Parameter value By Index
* Input: u16Index: Index
* Output:  -1: failure;
		   others: success 
*******************************************************************************/
int32_t i32GetPara(uint16_t u16Index)
{
	if(u16Index > PARA_Max)
	{
		//i32LogWrite(ERR, "GetPara Parameter is wrong, PARA_MAX = %d, Index = %d\r\n", PARA_Max, u16Index);
		return -1;
	}
	return sgPara.u16buf[u16Index];
}
/*******************************************************************************
* Name: uint16_t* u16pGetParaPoint(uint16_t u16Index)
* Descriptio: Get Parameter Point By Index
* Input: u16Index: Index
* Output:  NULL: failure;
		   others: parameter point 
*******************************************************************************/
uint16_t* u16pGetParaPoint(uint16_t u16Index)
{
	if(u16Index > PARA_Max)
	{
		//i32LogWrite(ERR, "GetPara Parameter is wrong, PARA_MAX = %d, Index = %d\r\n", PARA_Max, u16Index);
		return NULL;
	}
	return &(sgPara.u16buf[u16Index]);
}
	
/*******************************************************************************
* Name: int32_t i32SetPara(uint16_t u16Index, uint16_t u16Data)
* Descriptio: Set Paramter Value Bt Index
* Input: u16Index: Index
*        u16Data：value want to set
* Output:  -1; failure
		   0: success 
*******************************************************************************/
int32_t i32SetPara(uint16_t u16Index, uint16_t u16Data)
{
	if(u16Index > PARA_Max)
	{
		//i32LogWrite(ERR, "SetPara Parameter is wrong, PARA_MAX = %d, Index = %d\r\n", PARA_Max, u16Index);
		return -1;
	}
	sgPara.u16buf[u16Index] = u16Data;
	return 0;
}

/*******************************************************************************
* Name: void vParaInit(void)
* Descriptio: Parameters initial
* Input: NULL
* Output:  NULL 
*******************************************************************************/
void vParaInit(void)
{
	int index;
	uint16_t sum, flag;
	xPrmAttrStruct* pPrmAttr;
	xPrmDefStruct* pPrmDef;
	extern void vParaSaveStateUpdate(uint32_t Data);
	vEepromSetCallBack(i32SetPara, PARA_ErrCode0 - 1, vParaSaveStateUpdate);
//	InitEEPROM();
	//Read  & check hardware para
#if 0
	sum = 0;
	flag = 0;
	for (index=0; index < (sizeof(cHardPara_Table)/sizeof(cHardPara_Table[0])); index++)
	{
		flag += ReadHardParaFromEeprom(index);
		sum += *((INT16U*)cHardPara_Table[index]->pPrmData);
	}
	if (((flag | sum) & 0xffff) != 0)
	{ //Data error in eprom, set para to default value. 
		for (index=0; index < (sizeof(cHardPara_Table)/sizeof(cHardPara_Table[0])); index++)
		{
			if (SaveHardParaToEeprom(index,cHardPara_Table[index]->PrmInitVal) == 0)
				ReadHardParaFromEeprom(index);
			else
				*((INT16U*)cHardPara_Table[index]->pPrmData) = cHardPara_Table[index]->PrmInitVal;
		}
	}
#endif
	//Read eprom para volume
	if ((u16ReadParaFromEeprom(PARA_VOLUME_NUM) != 0)
//		  || (sgPara.u16UserType != cPara_Table[PARA_VOLUME_NUM]->PrmInitVal))
		|| (USER_TYPE != sgPara.u16UserType))
	{ //para invalid, to default val
		for(index=0; index < (sizeof(cPara_Table)/sizeof(cPara_Table[0])); index++) 
		{
			pPrmDef = (xPrmDefStruct*)cPara_Table[index];
			pPrmAttr = (xPrmAttrStruct*)(&pPrmDef->PrmAttr);
			if (pPrmAttr->Hard == 0)
			{
				if (u16SaveParaToEeprom(index,cPara_Table[index]->PrmInitVal) == 0)
					u16ReadParaFromEeprom(index);
				else
					*((uint16_t*)cPara_Table[index]->pPrmData) = cPara_Table[index]->PrmInitVal;
			}
			else
			{/*Hard para keep value*/}
		}
		u16SaveParaToEeprom(PARA_UserType, USER_TYPE);
	}
	else //para valid, read para from eprom and rom
	{
		for(index=0;index < (sizeof(cPara_Table)/sizeof(cPara_Table[0]));index++) 
		{
			u16ReadParaFromEeprom(index);
		}
	}
	
	
	/*lilu 20231008 add flash can baudrate*/
	#ifdef BAUDRATE_SYCHRON	//23.11.21 SJ 暂时屏蔽波特率同步相关功能
	if (*(uint32_t*)CAN_BAUDRATE_ADDRESS != i32GetPara(PARA_CanBaudRate))
	{
		uint32_t u32CanBaud =  *(uint32_t*)CAN_BAUDRATE_ADDRESS;
		if ((CAN_BAUD_RATE_125K == u32CanBaud) || (CAN_BAUD_RATE_250K == u32CanBaud))
		{
//			u16SaveParaToEeprom(PARA_CanBaudRate, u32CanBaud);
			i32SetPara(PARA_CanBaudRate, u32CanBaud);
		}
		else
		{
//			u16SaveParaToEeprom(PARA_CanBaudRate, u32CanBaud);
			i32SetPara(PARA_CanBaudRate, CAN_BAUD_RATE_500K);
		}
	}
	#endif //BAUDRATE_SYCHRON
	
	for (index=0; index < (sizeof(cMonitorPara_Table)/sizeof(cMonitorPara_Table[0])); index++)
	{
		pPrmDef = (xPrmDefStruct*)(cMonitorPara_Table[index]);
		*((uint16_t*)pPrmDef->pPrmData) = pPrmDef->PrmInitVal;
	}	
}