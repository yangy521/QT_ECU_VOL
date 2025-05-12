#ifndef _USER_CAN_NUOLI_PSNW_ECU_
#define _USER_CAN_NUOLI_PSNW_ECU_
#include "stdint.h"

/*接收PDO，转向、手柄、BMS*/

typedef union
{
	uint8_t u8Data[8];
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
		
		int16_t i16MoveThrottle;
		int16_t i16LiftThrottle;
		int16_t i16AuxiliaryThrottle;
	};
}xCanRev1E0Info;//手柄

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8ControlByte;
		uint8_t u8Reserve1;
		uint8_t u8Reserve2;
		uint8_t u8Reserve3;
		uint8_t u8Reserve4;
		uint8_t u8SOC;
		uint8_t u8Reserve6;
		uint8_t u8Reserve7;
	};
}xCanRev1A1Info;//仪表

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8SteerCurrentL;
		uint8_t u8SteerCurrentH;
		uint8_t u8SteerPotVoltageL;
		uint8_t u8SteerPotVoltageH;
		uint8_t u8SteerTmpL;
		uint8_t u8SteerTmpH;
		uint8_t u8Reserve6;
		uint8_t u8Reserve7;
	};
}xCanRev3E0Info;//转向

typedef union
{
	uint8_t u8Data[8];
	struct 
		{
			uint8_t b1Lock:1;
			uint8_t b1LowSpeed1:1;
			uint8_t b1LowSpeed2:1;
			uint8_t b1LiftLock:1;
			uint8_t b1DriverState:1;
			uint8_t b1DriverTypeEn:1;
			uint8_t b1RemoteOpen:1;
			uint8_t b1RemoteClose:1;
			
			uint8_t u8MaxSpeed;
			uint8_t u8Reserve1;
			uint8_t u8SerialNumber12;
			uint8_t u8SerialNumber34;
			uint8_t u8SerialNumber56;
			uint8_t u8SerialNumber78;
			uint8_t u8SerialNumber9A;
		};
}xCanRev27AInfo;//蓝牙2

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t	b1Hmoe: 1;
		uint8_t b1LeftLimit: 1;
		uint8_t	b1RightLimit: 1;
		uint8_t b1Reserve1: 1;
		uint8_t	b1UpFlag: 1;
		uint8_t b3Reserve2: 3;
		
		uint8_t u8ErrSteer;
		
		uint8_t u8SteerAngleL;
		uint8_t u8SteerAngleH;
		
		uint8_t u8SteerSpeedL;
		uint8_t u8SteerSpeedH;
		
		uint8_t u8Reserve3;
		uint8_t u8Reserve4;		
	};
}xCanRev361Info;//转向

typedef union
{
	uint8_t u8Data[8];
	struct 
		{
			uint8_t b1HeartBeatState:1;
			uint8_t b1CardState:1;
			uint8_t b1ErrorState:1;
			uint8_t b5Reserve:5;
			
			uint8_t u8Reserve1;
			uint8_t u8Reserve2;
			uint8_t u8Reserve3;
			uint8_t u8Reserve4;
			uint8_t u8Reserve5;
			uint8_t u8Reserve6;
			uint8_t u8Reserve7;
		};
}xCanRev1B0Info;//刷卡器

typedef union
{
	uint8_t u8Data[8];
	struct 
		{
			uint8_t b1Lock:1;
			uint8_t b1LowSpeed1:1;
			uint8_t b1LowSpeed2:1;
			uint8_t b1LiftLock:1;
			uint8_t b1DriverState:1;
			uint8_t b1DriverTypeEn:1;
			uint8_t b1RemoteOpen:1;
			uint8_t b1RemoteClose:1;
			
			uint8_t u8Reserve1;
			uint8_t u8Reserve2;
			uint8_t u8Reserve3;
			uint8_t u8Reserve4;
			uint8_t u8Reserve5;
			uint8_t u8Reserve6;
			uint8_t u8Reserve7;
		};
}xCanRev270Info;//蓝牙1


typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t b1ForwardState:1;
		uint8_t	b1BackwardState:1;
		uint8_t b1MainContactor:1;
		uint8_t b1HourEn:1;
		uint8_t b1Upright:1;
		uint8_t b1Park:1;//制动器
		uint8_t b1SeatBelt:1;
		uint8_t b1Maint:1;
		
		uint8_t u8ErrorMove;	//MCU故障转发
		uint8_t u8ErrorSteer; //转发报文361[1]
		uint8_t u8Movespeed; 	//速度
		uint8_t u8ErrorBMS;		
		uint8_t u8Soc;
		uint8_t u8HourCountL;
		uint8_t u8HourCountH;
	};
}xCanSend260Info;

typedef union
{
	uint8_t u8Data[8];
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

		
		uint8_t u8BatteryTypeL;
		uint8_t u8BatteryTypeH;

		uint8_t u8ErrorMove;	//MCU故障转发
		
		uint8_t	u8WorkTimeLL;
		uint8_t	u8WorkTimeLH;
		uint8_t	u8WorkTimeHL;
		uint8_t	u8WorkTimeHH;
	};
}xCanSend261Info;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8Reserve0;
		uint8_t u8Reserve1;
		uint8_t u8Reserve2;
		uint8_t u8Reserve3;
		uint8_t u8Reserve4;
		uint8_t u8Reserve5;
		uint8_t u8BMSWarning;
		uint8_t u8BMSError;
	};
}xCanRev2F1Info;//锂电

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8Data1;
		uint8_t u8Data2;
		uint8_t u8Data3;
		uint8_t u8Data4;
		uint8_t u8Data5;
		uint8_t u8Data6;
		uint8_t u8Data7;
		uint8_t u8Data8;
	};
}xCanSendCommon;


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
		xCanRev1E0Info CanRevInfo1E0;
	};
	/*接收PDO6， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData6[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		xCanRev361Info CanRevInfo361;
	};
	/*接收PDO7， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData7[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		xCanRev1A1Info CanRevInfo1A1;
	};
	/*接收PDO8， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData8[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		xCanRev2F1Info CanRevInfo2F1;

	};	
	union
	{
		uint8_t u8RevData9[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		xCanRev1B0Info CanRevInfo1B0;
	};
	union
	{
		uint8_t u8RevData10[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		xCanRev3E0Info	CanRevInfo3E0;
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
		xCanSend261Info CanSend261Info;
	};
	/*发送PDO6， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8SendData6[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
		xCanSend260Info CanSend260Info;
	};
	/*发送PDO7， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8SendData7[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
		xCanSendCommon CanSend2F8Info;
	};
	/*发送PDO8， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8SendData8[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
		xCanSendCommon CanSend2F9Info;
	};
	union
	{
		uint8_t u8SendData9[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
		xCanSendCommon CanSend2FAInfo;
	};
	union
	{
		uint8_t u8SendData10[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
	};
}xCanSendPdoInfo;

#endif