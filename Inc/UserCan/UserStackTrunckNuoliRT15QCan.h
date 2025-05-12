#ifndef _USER_CAN_NUOLI_RT15Q_DGC_ECU_
#define _USER_CAN_NUOLI_RT15Q_DGC_ECU_
#include "stdint.h"

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
		uint8_t	b1Hmoe: 1;
		uint8_t b1LeftLimit: 1;
		uint8_t	b1RightLimit: 1;
		uint8_t b1Reserve1: 1;
		uint8_t	b1UpFlag: 1;
		uint8_t b3Reserve2: 3;
		
		uint8_t u8ErrSteer;
		
		int16_t i16SteerAngle;
		
		uint16_t u16SteerSpeed;
		
		uint8_t u8Reserve3;
		uint8_t u8Reserve4;		
	};
}xCanRev360Info;//转向

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

/************ 发送报文 **********/
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

typedef struct
{
	/*接收PDO1， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData1[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		//xCanRev1ACInfo CanRevInfo1;
	};
	/*接收PDO2， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData2[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		//xCanRev360Info CanRevInfo2;
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
		xCanRev1A1Info CanRevInfo1A1;
	};
	/*接收PDO6， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData6[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		xCanRev360Info CanRevInfo360;
	};
	/*接收PDO7， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData7[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		xCanRev2F1Info CanRevInfo2F1;
	};
	/*接收PDO8， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData8[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
	xCanRev1E0Info CanRevInfo1E0;
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
	};
	/*发送PDO4， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8SendData4[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
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
	/*发送PDO9， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8SendData9[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
	};
	/*发送PDO10， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8SendData10[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
	};
}xCanSendPdoInfo;
#endif