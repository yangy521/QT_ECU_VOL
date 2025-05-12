//转弯减速
//参数初始化
//液驱剪叉车故障码通用表、
//小时计6min 1min 1s

#include "UserCommon.h"
#include "HourCount.h"
#include "NetTimer.h"
#include "PARA.h"
#include "Eeprom.h"

UserCommonInfo gUserInfo;

void vUserParaInit(void)
{
    gUserInfo.u8FastDriveSpeed = i32GetPara(PARA_FastDriveSpeed);
    gUserInfo.u8SlowDriveSpeed = i32GetPara(PARA_SlowDriveSpeed);
    gUserInfo.u8DriveSpeedAfterLift = i32GetPara(PARA_DriveSpeedAfterLift);
    gUserInfo.u8LiftSpeed = i32GetPara(PARA_LiftSpeed);
    gUserInfo.u8MaxTurnSpeed = i32GetPara(PARA_MaxTurnSpeed);

    gUserInfo.u8TurnPowerLimit = i32GetPara(PARA_TurnPowerLimit);
    gUserInfo.u8DeadZoneAdjust = i32GetPara(PAPA_DeadZoneAdjust);
    gUserInfo.u8BrakeFastDrive = i32GetPara(PARA_BrakeFastDrive);
    gUserInfo.u8BrakeSlowDrive = i32GetPara(PARA_BrakeSlowDrive);
    gUserInfo.u8BrakeDriveAfterLift = i32GetPara(PARA_BrakeDriveAfterLift);
    gUserInfo.u8BrakeLift = i32GetPara(PARA_BrakeLift);
    gUserInfo.u8BrakeLower = i32GetPara(PARA_BrakeLower);
    gUserInfo.u8BrakeTurn = i32GetPara(PARA_BrakeTurn);
    gUserInfo.u8BrakeAntiPinch = i32GetPara(PARA_BrakeAntiPinch);
    gUserInfo.u8LowerSpeed = i32GetPara(PARA_LowerSpeed);

    gUserInfo.u8OverLoadStabilityDelay = i32GetPara(PARA_OverLoadStabilityDelay);
    
    gUserInfo.u8DynamicOverLoadPercent = i32GetPara(PARA_DynamicOverLoadPercent);
    gUserInfo.u8StaticOverLoadPercent = i32GetPara(PARA_StaticOverLoadPercent);
    gUserInfo.u8MaxDifferencePercent = i32GetPara(PARA_MaxDifferencePercent);
    gUserInfo.u8DriveMotorEncoder = i32GetPara(PARA_DriveMotorEncoder);
    gUserInfo.u8MotorHighSpeedDeceRate = i32GetPara(PARA_MotorHighSpeedDeceRate);
    gUserInfo.u8MotorLowSpeedDeceRate = i32GetPara(PARA_MotorLowSpeedDeceRate);

    gUserInfo.u8VoiceAlarmVolume = i32GetPara(PARA_VoiceAlarmVolume);

    gUserInfo.u8CurveFastDrive = i32GetPara(PARA_CurveFastDrive);

    gUserInfo.u8CurveSlowDrive = i32GetPara(PARA_CurveSlowDrive);
    gUserInfo.u8CurveDriveAfterLift = i32GetPara(PARA_CurveDriveAfterLift);
    gUserInfo.u8CurveLift = i32GetPara(PARA_CurveLift);
    gUserInfo.u8CurveLower = i32GetPara(PARA_CurveLower);
    gUserInfo.u8CurveTurn = i32GetPara(PARA_CurveTurn);
    gUserInfo.u8AccAndDecFastDrive = i32GetPara(PARA_AccAndDecFastDrive);
    gUserInfo.u8AccAndDecSlowDrive = i32GetPara(PARA_AccAndDecSlowDrive);
    gUserInfo.u8AccAndDecAfterLift = i32GetPara(PARA_AccAndDecAfterLift);
    gUserInfo.u8AccAndDecLift = i32GetPara(PARA_AccAndDecLift);
    gUserInfo.u8AccAndDecLower = i32GetPara(PARA_AccAndDecLower);
    gUserInfo.u8AccAndDecTurn = i32GetPara(PARA_AccAndDecTurn);
    gUserInfo.u8AccAndDecAntiPinch = i32GetPara(PARA_AccAndDecAntiPinch);


    gUserInfo.u8PumpMotorEncoder = i32GetPara(PARA_PumpMotorEncoder);
    gUserInfo.u8VehicleType = i32GetPara(PARA_VehicleType);
    gUserInfo.u8VehcileHeight = i32GetPara(PARA_VehcileHeight);
    gUserInfo.u8PressureSensorType = i32GetPara(PARA_PressureSensorType);
    gUserInfo.u8PitProtectFunc = i32GetPara(PARA_PitProtectFunc);
    gUserInfo.u8AntiPinchFunc = i32GetPara(PARA_AntiPinchFunc);

    gUserInfo.u8ActAlmFunc = i32GetPara(PARA_ActAlmFunc);
    gUserInfo.u8WeighFunc = i32GetPara(PARA_WeighFunc);
    gUserInfo.u8ParallelValveReverseFunc = i32GetPara(PARA_ParallelValveReverseFunc);
    gUserInfo.u8LowBatAlmFunc = i32GetPara(PARA_LowBatAlmFunc);
    gUserInfo.u8LowVolAlmTime = i32GetPara(PARA_LowVolAlmTime);
    gUserInfo.u8LowVolShutDownTime = i32GetPara(PARA_LowVolShutDownTime);
    gUserInfo.u8UpperCtlButSleep = i32GetPara(PARA_UpperCtlButSleep);
    gUserInfo.u8LanguageType = i32GetPara(PARA_LanguageType);
    gUserInfo.u8BatteryType = i32GetPara(PARA_BatteryType);
    gUserInfo.u8SpeakerSync = i32GetPara(PARA_SpeakerSync);

    gUserInfo.u8LowerPumpType = i32GetPara(PARA_LowerPumpType);
    gUserInfo.u8PressureType = i32GetPara(PARA_PressureType);
    gUserInfo.u8AngleSimulationLimit = i32GetPara(PARA_AngleSimulationLimit);
    gUserInfo.u8LiftReverseFunc = i32GetPara(PARA_LiftReverseFunc);
    
    gUserInfo.u8FourPointWeightFunc = i32GetPara(PARA_FourPointWeightFunc);
    gUserInfo.u8DriverType = i32GetPara(PARA_DriverType);
    gUserInfo.u8PasswordLock = i32GetPara(PARA_PasswordLock);
    gUserInfo.u8InAndOutFunc = i32GetPara(PARA_InAndOutFunc);
    gUserInfo.u8HeartBeatQueryFunc = i32GetPara(PARA_HeartBeatQueryFunc);
    gUserInfo.u8LowBatteryMode = i32GetPara(PARA_LowBatteryMode);
    gUserInfo.u8AngleSensorSetting = i32GetPara(PARA_AngleSensorSetting);
    gUserInfo.u8TiltSwitchSetting = i32GetPara(PARA_TiltSwitchSetting);

    gUserInfo.u8AngleSensorType = i32GetPara(PARA_AngleSensorType);
    gUserInfo.u8AnaLogLimitDetSwitch = i32GetPara(PARA_AnaLogLimitDetSwitch);
    gUserInfo.u8IsNoLoadCalibration = i32GetPara(PARA_IsNoLoadCalibration);
    gUserInfo.u8IsOverLoadCalibration = i32GetPara(PARA_IsOverLoadCalibration);
    gUserInfo.u16SetDescentHeightValue = i32GetPara(PARA_SetDescentHeightValue);
    gUserInfo.u8ReleaseBrake = i32GetPara(PARA_ReleaseBrake);

    gUserInfo.u16SetOutHeight = i32GetPara(PARA_SetOutHeight);
    gUserInfo.u16AngleSimulationUpLimit = i32GetPara(PARA_AngleSimulationUpLimit);
    gUserInfo.u16AngleSimulationDownLimit = i32GetPara(PARA_AngleSimulationDownLimit);
    gUserInfo.u16CanBaudRate = i32GetPara(PARA_CanBaudRate);
    gUserInfo.u16EmptyPressure = i32GetPara(PARA_EmptyPressure);
    gUserInfo.u16FullPressure = i32GetPara(PARA_FullPressure);
    gUserInfo.u16DriverFlag = i32GetPara(PARA_DriverFlag);
    gUserInfo.u16MinAngle = i32GetPara(PARA_MinAngle);
    gUserInfo.u16MaxAngle = i32GetPara(PARA_MaxAngle);
    gUserInfo.u16BatSocPalyBack = i32GetPara(PARA_BatSocPalyBack);
    gUserInfo.u16ValveType = i32GetPara(PARA_ValveType);
    gUserInfo.u16AnticollisionFunc = i32GetPara(PARA_AnticollisionFunc);
    gUserInfo.u16ValueOpenLoopCurrent = i32GetPara(PARA_ValueOpenLoopCurrent);
    gUserInfo.u16ValueOpenPercentage = i32GetPara(PARA_ValueOpenPercentage);
    gUserInfo.u16CanOpenNodeId = i32GetPara(PARA_CanOpenNodeId);

    gUserInfo.u16AngleValue0 = i32GetPara(PARA_AngleValue0);
    gUserInfo.u16AngleValue1 = i32GetPara(PARA_AngleValue1);
    gUserInfo.u16AngleValue2 = i32GetPara(PARA_AngleValue2);
    gUserInfo.u16AngleValue3 = i32GetPara(PARA_AngleValue3);
    gUserInfo.u16AngleValue4 = i32GetPara(PARA_AngleValue4);
    gUserInfo.u16AngleValue5 = i32GetPara(PARA_AngleValue5);
    gUserInfo.u16AngleValue6 = i32GetPara(PARA_AngleValue6);
    gUserInfo.u16AngleValue7 = i32GetPara(PARA_AngleValue7);

    gUserInfo.u16MotorMaxSpd = i32GetPara(PARA_MotorMaxSpd);
    gUserInfo.u16Analog3DeadZoneMinVal = i32GetPara(PARA_Analog3DeadZoneMinVal);
    gUserInfo.u16Analog3DeadZoneMaxVal = i32GetPara(PARA_Analog3DeadZoneMaxVal);
    gUserInfo.u16Analog3MidVal = i32GetPara(PARA_Analog3MidVal);
    gUserInfo.u8ThrottleType = i32GetPara(PARA_ThrottleType);
    gUserInfo.u16ThrottleFDeadZoneMinVal = i32GetPara(PARA_ThrottleFDeadZoneMinVal);
    gUserInfo.u16ThrottleFDeadZoneMaxVal = i32GetPara(PARA_ThrottleFDeadZoneMaxVal);
    gUserInfo.u16ThrottleFMidVal = i32GetPara(PARA_ThrottleFMidVal);
    gUserInfo.u16ThrottleBDeadZoneMinVal = i32GetPara(PARA_ThrottleBDeadZoneMinVal);
    gUserInfo.u16ThrottleBDeadZoneMaxVal = i32GetPara(PARA_ThrottleBDeadZoneMaxVal);
    gUserInfo.u16ThrottleBMidVal = i32GetPara(PARA_ThrottleBMidVal);
    
    gUserInfo.u16BrakeType = i32GetPara(PARA_BrakeType);

    gUserInfo.u16BrakeFDeadZoneMinVal = i32GetPara(PARA_BrakeFDeadZoneMinVal);
    gUserInfo.u16BrakeFDeadZoneMaxVal = i32GetPara(PARA_BrakeFDeadZoneMaxVal);
    gUserInfo.u16BrakeFMidVal = i32GetPara(PARA_BrakeFMidVal);
    gUserInfo.u16BrakeBDeadZoneMinVal = i32GetPara(PARA_BrakeBDeadZoneMinVal);
    gUserInfo.u16BrakeBDeadZoneMaxVal = i32GetPara(PARA_BrakeBDeadZoneMaxVal);
    gUserInfo.u16BrakeBMidVal = i32GetPara(PARA_BrakeBMidVal);
    gUserInfo.u8Gear1Spd = i32GetPara(PARA_Gear1Spd);
    gUserInfo.u8Gear2Spd = i32GetPara(PARA_Gear2Spd);
    gUserInfo.u8Gear3Spd = i32GetPara(PARA_Gear3Spd);
    gUserInfo.u8Gear4Spd = i32GetPara(PARA_Gear4Spd);
    
    gUserInfo.u16RatioOfTransmission = i32GetPara(PARA_RatioOfTransmission);
    gUserInfo.u16MaintenancePeriod = i32GetPara(PARA_MaintenancePeriod);
    gUserInfo.u8RemotePara = i32GetPara(PARA_RemotePara);
    
    gUserInfo.u8PumpMotorGear1 = i32GetPara(PARA_PumpMotorGear1);
    gUserInfo.u8PumpMotorGear2 = i32GetPara(PARA_PumpMotorGear2);

    gUserInfo.u16TurnWithDecStartAngle = i32GetPara(PARA_TurnWithDecStartAngle);
    gUserInfo.u16TurnWithDecEndAngle = i32GetPara(PARA_TurnWithDecEndAngle);
    gUserInfo.u8AngleWithStartSpdPer = i32GetPara(PARA_AngleWithStartSpdPer);
    gUserInfo.u8AngleWithEndSpdPer = i32GetPara(PARA_AngleWithEndSpdPer);
		
		
		
		gUserInfo.fPropMaxCurrent0 = i32GetPara(PARA_PropDMaxCurrent0) / 1000.0;
		gUserInfo.fPropMinCurrent0 = i32GetPara(PARA_PropDMinCurrent0) / 1000.0;
		gUserInfo.fPropMaxCurrent1 = i32GetPara(PARA_PropDMaxCurrent1) / 1000.0;
		gUserInfo.fPropMinCurrent1 = i32GetPara(PARA_PropDMinCurrent1) / 1000.0;		
	
		
    gUserInfo.fAgnleDecSpdFactor = (gUserInfo.u8AngleWithStartSpdPer - gUserInfo.u8AngleWithEndSpdPer) /   
										(gUserInfo.u16TurnWithDecEndAngle - gUserInfo.u16TurnWithDecEndAngle);
										
    gUserInfo.u16HourCountPowerOn = i32GetPara(PARA_HourCountPowerOn);
		
}
/*******************************************************************************
* Name: void vHourCountInit(void)
* Descriptio: 小时计初始化函数
* Input: 
* Output:
参数u16HourCountPowerOn
bit0：使能小时计计时,bit1计时方式、0使能1上电，bit2,3，计时精度，00一秒种，01一分钟，10六分钟，11一小时
bit6:清除小时计
*******************************************************************************/
void vHourCountInit(void)
{
	if(1 == (gUserInfo.u16HourCountPowerOn & 0x01))
	{
		gUserInfo.u32HourCount = u32HourCountRead();
		vSetNetTimer(TIMER_HourCount, HOURCOUNT_PERIOD);
	}
	else
	{
	}
}
/*******************************************************************************
* Name: void uint32_t u32HourCountGet(uint8_t u8Flag,uint8_t u8EnSwitch)
* Descriptio: 小时计运行
* Input: u8EnSwitch 使能开关状态
* Output: 小时计状态
*******************************************************************************/
uint32_t u32HourCountProc(uint8_t u8EnSwitch)
{
	static uint16_t u16OneMinCnt = 0;
	uint32_t u32HourCountValue;
	
	if (true == u8GetNetTimerOverFlag(TIMER_HourCount))
	{
		vResetNetTimer(TIMER_HourCount);
		if (1 == ((gUserInfo.u16HourCountPowerOn >> 1) & 0x01))//使能计时
		{
			if (0 != u8EnSwitch)//使能开关按下
			{
				u16OneMinCnt++;
			}
		}
		else
		{
			u16OneMinCnt++;
		}		
	}
	
	if(u16OneMinCnt >= 360)
	{
		gUserInfo.u32HourCount ++;
		vHourCountWrite(gUserInfo.u32HourCount);
		u16OneMinCnt = 0;
	}
	
	
	if(0 == ((gUserInfo.u16HourCountPowerOn >> 2) & 0x03))//精度一秒钟
	{
		u32HourCountValue = gUserInfo.u32HourCount * 360 + u16OneMinCnt ; 
	}
	else if(1 == ((gUserInfo.u16HourCountPowerOn >> 2) & 0x03))//精度一分钟
	{
		u32HourCountValue = gUserInfo.u32HourCount * 6 + u16OneMinCnt / 60;
	}
	else if(2 == ((gUserInfo.u16HourCountPowerOn >> 2) & 0x03))//精度六分钟
	{
		u32HourCountValue = gUserInfo.u32HourCount;
	}
	else if(3 == ((gUserInfo.u16HourCountPowerOn >> 2) & 0x03))//精度一小时
	{
		u32HourCountValue = gUserInfo.u32HourCount / 10;
	}
	
	//小时计清除
//	if(1 == ((gUserInfo.u16HourCountPowerOn >> 6 & 0x01)))
//	{
//		gUserInfo.u16HourCountPowerOn &= ~(1 << 6);
//		u16SaveParaToEeprom(PARA_HourCountPowerOn,gUserInfo.u16HourCountPowerOn);
//		gUserInfo.u32HourCount = 0;
//		vHourCountWrite(gUserInfo.u32HourCount);
//	}
	return u32HourCountValue;
}


uint16_t u16Read32DataFromEeprom(uint16_t u16Adress1,uint16_t u16Adress2, uint32_t *u32Data,uint8_t u8Mode)
{
	uint16_t res = 0;
	uint16_t u16tmp = 0;
	res += u16EepromRead(u16Adress1,&u16tmp,u8Mode);
	
	if(1 == ((u16tmp>>15) & 0x1))//最高位存储位置顺序，最高位为1，高15位
	{
		*u32Data = (u16tmp & 0x7FFF)<<15;
		res += u16EepromRead(u16Adress2,&u16tmp,u8Mode);
		*u32Data |= (u16tmp & 0x7FFF);
	}
	else if(0 == ((u16tmp>>15) & 0x1))//最高位为0，为低15位
	{
		*u32Data = (u16tmp & 0x7FFF);
		res += u16EepromRead(u16Adress2,&u16tmp,u8Mode);
		*u32Data |= (u16tmp & 0x7FFF)<<15;		
	}
	return res;
}
uint16_t u16Write32DataFromEeprom(uint16_t u16Adress1,uint16_t u16Adress2,uint32_t u32Data,uint8_t u8Mode)
{
	uint16_t res = 0;
	uint16_t u16DataL = 0;
	uint16_t u16DataH = 0;
	
	u16DataL = u32Data & 0x7FF;
	u16DataH = ((u32Data>>15) & 0x7FF)| 0x8000;
	
	
	if(0 ==  ((u32Data / 10000)%2))
	{
		res += u16EepromWrite(u16Adress1,u16DataL,u8Mode);
		res += u16EepromWrite(u16Adress2,u16DataH,u8Mode);
	}
	else
	{
		res += u16EepromWrite(u16Adress1,u16DataH,u8Mode);
		res += u16EepromWrite(u16Adress2,u16DataL,u8Mode);
	}
	return res;
}

#if 0
static uint16_t	u16SpeedAdjWithAngle(uint16_t u16Spd, uint16_t u16Angle, const xSteerAngleDecSpd *Info)
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
#endif
