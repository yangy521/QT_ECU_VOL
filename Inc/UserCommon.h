#ifndef _USER_COMMON_H_
#define _USER_COMMON_H_

#include "stdint.h"

#define HOUR_SEC
#define HOUR_1MIN
#define HOUR_6MIN
#define HOUR_1HOUR

typedef struct
{
	/*para info*/
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

	uint16_t	u16SetDescentHeightValue;		
	uint8_t		u8ReleaseBrake;

//	uint8_t		u8Reserve9;				/*78*/
//	uint8_t		u8Reserve10;				/*79*/
	
	uint16_t		u16SetOutHeight;			/*80*/
	uint16_t		u16AngleSimulationUpLimit;	
	uint16_t		u16AngleSimulationDownLimit;
	
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

	uint16_t		u16EmptyPressure;
	uint16_t		u16FullPressure;
	uint16_t		u16DriverFlag;
	
//	uint8_t		u8Reserve14;					/*110*/
//	uint8_t		u8Reserve15;					/*111*/
	
	uint16_t		u16MinAngle;					/*112*/
	uint16_t		u16MaxAngle;
	uint16_t		u16BatSocPalyBack;
	uint16_t		u16ValveType;
	uint16_t		u16AnticollisionFunc;
	uint16_t		u16ValueOpenLoopCurrent;
	uint16_t		u16ValueOpenPercentage;
	uint16_t		u16CanOpenNodeId;

	
	uint16_t		u16AngleValue0;
	uint16_t		u16AngleValue1;
	uint16_t		u16AngleValue2;
	uint16_t		u16AngleValue3;
	uint16_t		u16AngleValue4;
	uint16_t		u16AngleValue5;
	uint16_t		u16AngleValue6;
	uint16_t		u16AngleValue7;
	

	uint8_t		u8LogLevel;					/*138 log level*/
	uint8_t		u8LogModel;					/*139*/
	uint16_t	u16MotorMaxSpd;				/*140*/
	
	uint16_t		u16Analog3DeadZoneMinVal;		/*141*/
	uint16_t		u16Analog3DeadZoneMaxVal;		/*142*/
	uint16_t		u16Analog3MidVal;				/*143*/
	
	uint8_t		u8ThrottleType;					/*144 ����������*/
	uint16_t		u16ThrottleFDeadZoneMinVal;		/*̤������������Сֵ��0.1V��*/
	uint16_t		u16ThrottleFDeadZoneMaxVal;		/*̤�������������ֵ��0.1V��*/
	uint16_t		u16ThrottleFMidVal;				/*̤��������ֵ��Ӧ����ٷֱȣ�1%��*/
	uint16_t		u16ThrottleBDeadZoneMinVal;		/*̤�巴��������Сֵ��0.1V��*/
	uint16_t		u16ThrottleBDeadZoneMaxVal;		/*̤�巴���������ֵ��0.1V��*/
	uint16_t		u16ThrottleBMidVal;				/*̤�巴����ֵ��Ӧ����ٷֱȣ�1%��*/
	
	uint16_t		u16BrakeType;						/*�ƶ�̤����������*/
	uint16_t		u16BrakeFDeadZoneMinVal;			/*�ƶ�̤������������Сֵ��0.1V��*/
	uint16_t		u16BrakeFDeadZoneMaxVal;			/*�ƶ�̤�������������ֵ��0.1V��*/
	uint16_t		u16BrakeFMidVal;					/*�ƶ�̤��������ֵ��Ӧ����ٷֱȣ�1%��*/
	uint16_t		u16BrakeBDeadZoneMinVal;			/*�ƶ�̤�巴��������Сֵ��0.1V��*/
	uint16_t		u16BrakeBDeadZoneMaxVal;			/*�ƶ�̤�巴���������ֵ��0.1V��*/
	uint16_t		u16BrakeBMidVal;					/*�ƶ�̤�巴����ֵ��Ӧ����ٷֱȣ�1%��*/
	
//	uint8_t		u8Reserve16;						/*158*/
//	uint8_t		u8Reserve17;						/*159*/
	
	uint8_t		u8Gear1Spd;						/*160 1���ٶ�(%)*/
	uint8_t		u8Gear2Spd;						/*2���ٶ�(%)*/
	uint8_t		u8Gear3Spd;						/*3���ٶ�(%)*/
	uint8_t		u8Gear4Spd;						/*4���ٶ�(%)*/
	
	uint16_t	u16RatioOfTransmission;			/*ת�ٹ����*/
	uint16_t	u16MaintenancePeriod;				/*ά������(h)*/
	uint8_t		u8RemotePara;					/*Զ���ն˲���*/
	
	uint8_t		u8PumpMotorGear1;				/*��������м䵲λ1(%)*/
	uint8_t		u8PumpMotorGear2;				/*��������м䵲λ2(%)*/
	
	uint16_t		u16TurnWithDecStartAngle;			/*ת�併����ʼ�Ƕ�*/
	uint16_t		u16TurnWithDecEndAngle;			/*ת�併����ֹ�Ƕ�*/
	
	uint8_t		u8AngleWithStartSpdPer;			/*��ʼ�Ƕ��ٶȰٷֱ�*/
	uint8_t		u8AngleWithEndSpdPer;			/*��ֹ�Ƕ��ٶȰٷֱ�*/
	
	uint16_t		u16HourCountPowerOn;
	
	/*user info*/

//	float		fFastMoveSpdPer5msAccStep;
//	float		fSlowMoveSpdPer5msAccStep;
//	float		fMoveAfterLiftSpdPer5msAccStep;
//	float		fLiftSpdPer5msAccStep;
//	float		fTurnSpdPer5msAccStep;
//	
//	float		fFastMoveSpdPer5msDecStep;
//	float		fSlowMoveSpdPer5msDecStep;
//	float		fMoveAfterLiftSpdPer5msDecStep;
//	float		fLiftSpdPer5msDecStep;
//	float		fTurnSpdPer5msDecStep;
	
	float		fPropMinCurrent0;
	float		fPropMaxCurrent0;
	
	float		fPropMinCurrent1;
	float		fPropMaxCurrent1;
	
	float		fAgnleDecSpdFactor;
	
	uint32_t u32HourCount;
	
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
	
	uint8_t		b1HourConutMode: 1;		
	uint8_t		b1StartUpLock: 1;		
	uint8_t		b1LiftLock: 1;			
	uint8_t		b1LiftPedal: 1;			
	uint8_t		b1MoveLiftMode: 1;		
	uint8_t		b1SteerCheck:1;
	uint8_t		b1CarType:1;   
	uint8_t		b1SpeedLimitAfterLift: 1;
	

	
	uint16_t	u16RentalTime;
}UserCommonInfo;

extern	UserCommonInfo gUserInfo;
extern	void	vUserParaInit(void);
extern	void	vHourCountInit(void);
extern	uint32_t	u32HourCountProc(uint8_t u8EnSwitch);
extern uint16_t u16Read32DataFromEeprom(uint16_t u16Adress1,uint16_t u16Adress2, uint32_t *u32Data,uint8_t u8Mode);
extern uint16_t u16Write32DataFromEeprom(uint16_t u16Adress1,uint16_t u16Adress2,uint32_t u32Data,uint8_t u8Mode);

#endif
