#ifndef _USER_FORKLIFTXUGONG_CAN_ECU_
#define _USER_FORKLIFTXUGONG_CAN_ECU_
#include "stdint.h"


typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8CirculateTimeL;
		uint8_t u8CirculateTimeH;
		uint8_t u8Soc;
		uint8_t u8BatteryTemperature;
		uint8_t u8SingelBatVoltH;
		uint8_t u8SingelBatVoltL;
		
		uint8_t b1ErrorOverHeatRate1:1;
		uint8_t b1DischargeCurrentHigh1:1;
		uint8_t b1TotalVoltLow1:1;
		uint8_t b1SingleVoltLow1:1;
		uint8_t b1SingelVoltLow2:1;
		uint8_t b1BatVoltDiffHigh:1;
		uint8_t b1BatTmpDiffHigh:1;
		uint8_t b1DischargeCurrentHigh2:1;
		
		uint8_t b1DischargeTmpHigh2:1;
		uint8_t b1TotalVoltLow2:1;
		uint8_t b6Reserve:6;
	};
}xRveBMS28A;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8BatVoltL;
		uint8_t u8BatVoltH;
		uint8_t u8Reserve1;
		uint8_t u8Reserve2;
		uint8_t u8Reserve3;
		uint8_t u8Reserve4;
		uint8_t u8Reserve5;
		uint8_t u8Reserve16;
	};
}xRveBMS18A;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t b1ExitKey:1;
		uint8_t b1OptionUpkey:1;
		uint8_t b1OptionDownKey:1;
		uint8_t b1EnterKey:1;
		uint8_t b1EnableKey:1;
		uint8_t b1LiftKey:1;
		uint8_t b1DownKey:1;
		uint8_t b1Reserve2:1;
		
		uint8_t u8ReservedByte2;
		
		uint8_t u8LanguageType;
		
		uint8_t u8ReservedByte4;
		
		uint8_t u8ReservedByte5;
		
		uint8_t u8ReservedByte6;
		
		uint8_t u8ReservedByte7;
	};
}xRevLowerControlPanel;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8AngleSensorVoltL;
		uint8_t u8AngleSensorVoltH;
		uint8_t u8Pressure1CurrentL;
		uint8_t u8Pressure1CurrentH;
		uint8_t u8Pressure2CurrentL;
		uint8_t u8Pressure2CurrentH;
		uint8_t u8LoadRate;
		uint8_t b1ForwardValveState:1;
		uint8_t b1BackValveState:1;
		uint8_t b1TurnLeftValveState:1;
		uint8_t b1TurnRightValveState:1;
		uint8_t b1LiftValveState:1;
		uint8_t b1PropValve1State:1;
		uint8_t b1PropValve2State:1;
		uint8_t b1HighLowSpeedValveState:1;
	};
}xCanSend5A1;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t KsiVoltL;
		uint8_t KsiVoltH;
		uint8_t b1KeySwitch:1;
		uint8_t b1TiltSwitch:1;
		uint8_t b1Reserve1:1;
		uint8_t b1Reserve2:1;
		uint8_t b1PitSwitch:1;
		uint8_t b1Reserve3:1;
		uint8_t b1UplimitSwitch:1;
		uint8_t b1DownLimitSwitch:1;
		
		uint8_t u8PumpMotorSpeed;
		uint8_t u8LeftMotorSpeed;
		uint8_t u8RightMotorSpeed;
		uint8_t u8Reserve4;
		uint8_t u8Reserve5;
	};
}xCanSend5A2;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t b1PCUEnableSwitch:1;
		uint8_t b1PCULeftSwitch:1;
		uint8_t b1PCURightSwitch:1;
		uint8_t b1PCUMoveModeSwitch:1;
		uint8_t b1PCULiftModeSwitch:1;
		uint8_t b1PCUSlowSwitch:1;
		uint8_t b1PCUSpeakerSwitch:1;
		uint8_t b1Reserve1:1;
		
		uint8_t u8PCUControlWord2;//履带车的控制指令，暂不响应
		uint8_t u8PCUHandleValue;
		uint8_t u8Reserve2;
		uint8_t u8Reserve3;
		uint8_t u8Reserve4;
		uint8_t u8Reserve5;
		uint8_t u8Reserve6;
	};
}xCanSend5A3;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8MotorSpeedL;
		uint8_t u8MotorSpeedH;
		uint8_t u8Reserve1;
		uint8_t u8Reserve2;		
		uint8_t u8Reserve3;
		uint8_t u8Reserve4;		
		uint8_t u8PumpMotorEncoding;
		uint8_t u8MainContacterState;
	};
}xCanSend5A4;



typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8VbusCurrentL;
		uint8_t u8VbusCurrentH;
		uint8_t u8PhaseCurrentL;
		uint8_t u8PhaseCurrentH;		
		uint8_t u8BoardTemperatureL;
		uint8_t u8BoardTemperatureH;		
		uint8_t u8MotorTemperatureL;
		uint8_t u8MotorTemperatureH;
	};
}xCanSend5A5;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8Reserve1;
		uint8_t u8Reserve2;
		uint8_t u8Reserve3;
		uint8_t u8Reserve4;		
		uint8_t u8Reserve5;
		uint8_t u8Reserve6;		
		uint8_t u8Reserve7;
		uint8_t u8Reserve8;
	};
}xCanSend5A6;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8Reserve1;
		uint8_t u8Reserve2;
		uint8_t u8Reserve3;
		uint8_t u8Reserve4;		
		uint8_t u8Reserve5;
		uint8_t u8Reserve6;		
		uint8_t u8Reserve7;
		uint8_t u8Reserve8;
	};
}xCanSend5A7;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8Reserve1;
		uint8_t u8Reserve2;
		uint8_t u8Reserve3;
		uint8_t u8Reserve4;		
		uint8_t u8Reserve5;
		uint8_t u8Reserve6;		
		uint8_t u8Reserve7;
		uint8_t u8Reserve8;
	};
}xCanSend5A8;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8Soc;
		uint8_t u8Reserve2;
		uint8_t u8PumpCmdSpeedL;
		uint8_t u8PumpCmdSpeedH;		
		uint8_t u8Reserve5;
		uint8_t u8Reserve6;		
		uint8_t u8Reserve7;
		uint8_t u8Reserve8;
	};
}xCanSend5A9;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8HourCntLL;
		uint8_t u8HourCntLH;
		uint8_t u8HourCntHL;
		uint8_t u8HourCntHH;		
		uint8_t u8Reserve5;
		uint8_t u8Reserve6;		
		uint8_t u8Reserve7;
		uint8_t u8Reserve8;
	};
}xCanSend5AA;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8ErrorNum;
		uint8_t u8Error1;
		uint8_t u8Error2;
		uint8_t u8Error3;		
		uint8_t u8Error4;
		uint8_t u8Error5;		
		uint8_t u8Error6;
		uint8_t u8DriverError;
	};
}xCanSend5AB;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8CS;
		uint8_t u8IndexL;
		uint8_t u8IndexH;
		uint8_t u8SubIndexL;		
		uint8_t u8DataDD;
		uint8_t u8DataCC;		
		uint8_t u8DataBB;
		uint8_t u8DataAA;
	};
}xCanSDO;


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
		xRveBMS28A CanRevBMS28A;
	};
	/*接收PDO6， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData6[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		xRevLowerControlPanel CanLowerControl;
	};
	/*接收PDO7， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData7[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		xRveBMS18A CanRveBMS18A;

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
		//xCanSend33CInfo CanSend33CInfo;
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