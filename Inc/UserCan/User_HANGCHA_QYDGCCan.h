#ifndef _USER_CAN_HANGCHAQYDGC_CAN_ECU_
#define _USER_CAN_HANGCHAQYDGC_CAN_ECU_
#include "stdint.h"


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
		
		uint8_t i16SteerAngleL;
		uint8_t i16SteerAngleH;
		
		uint16_t u16SteerSpeed;
		
		uint8_t u8Reserve3;
		uint8_t u8Reserve4;		
	};
}xCanRev360Info;

typedef union
{
	uint8_t u8Data[8];
	struct
	{		
		uint8_t u8Reserve0;
		uint8_t u8Reserve1;
		uint8_t u8Reserve2;
		uint8_t u8Reserve3;
		uint8_t BMS_SOC;
		uint8_t	u8Reserve5;
		uint8_t	BMS_ErrCode;
		uint8_t	u8Reserve7;

	};
}xCanRev2F0Info;

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
		uint8_t	u8Reserve5;
		uint8_t	u8Reserve6;
		uint8_t	u8Reserve7;

	};
}xCanRev2F1Info;

typedef union
{
	uint8_t u8Data[8];
	struct
	{		
		uint8_t u8WorkState1;
		uint8_t u8WorkState2;
		uint8_t u8MotorValueL;
		uint8_t u8MotorValueH;
		uint8_t u8LiftValueL;
		uint8_t	u8LiftValueH;
		uint8_t	u8QianHouYiL;
		uint8_t	u8QianHouYiH;

	};
}xCanRev1E0Info;

typedef union
{
	uint8_t u8Data[8];
	struct
	{		
		uint8_t u8Rev670Date1;
		uint8_t u8Rev670Date2;
		uint8_t u8Rev670Date3;
		uint8_t u8Rev670Date4;
		uint8_t u8Language;
		uint8_t	u8Rev670Date6;
		uint8_t	u8Rev670Date7;
		uint8_t	u8Rev670Date8;

	};
}xCanRev670Info;

typedef union
{
	uint8_t u8Data[8];
	struct
	{		
		uint8_t u8Rev640Date1;
		uint8_t u8Rev640Date2;
		uint8_t u8Rev640Date3;
		uint8_t u8Rev640Date4;
		uint8_t u8Rev640Date5;
		uint8_t	u8Rev640Date6;
		uint8_t	u8Rev640Date7;
		uint8_t	u8Rev640Date8;
	};
}xCanRev640Info;


typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8MoveState;//前进0，后退1，直立4，刹车5，
		uint8_t u8ErrorMove;	//MCU故障转发
		uint8_t u8ErrorSteer; //转发报文360[1]
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
		uint8_t Basic;
		
		uint8_t SOC_Choose: 2;
		uint8_t Time_Choose: 2;
		uint8_t SOC_Warn: 1;
		uint8_t PowerOn_Count: 1;
		uint8_t Work_Count: 1;
		uint8_t Err_Flg: 1;
		
		uint8_t Work_Mode: 2;	
		uint8_t SpeedOn: 1;
		uint8_t SpeedUint: 1;
		uint8_t u1Reserve1: 1;
		uint8_t LiftUpLimit: 1;
		uint8_t Park: 1;
		uint8_t Charge: 1;
		
		uint8_t Contorl;
		uint8_t ErrCode;
		uint8_t Speed;
		uint8_t SOC;
		uint8_t u8Reserve1;	
	};
}xCanSend258Info;


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
		uint8_t	u8Reserve5;
		uint8_t	u8Reserve6;
		uint8_t	u8Reserve7;

	};
}xCanSend2F8Info;

typedef union
{
	uint8_t u8Data[8];
	struct
	{		
		uint8_t u85C0DateSend1;
		uint8_t u85C0DateSend2;
		uint8_t u85C0DateSend3;
		uint8_t u85C0DateSend4;
		uint8_t u85C0DateSend5;
		uint8_t	u85C0DateSend6;
		uint8_t	u85C0DateSend7;
		uint8_t	u85C0DateSend8;

	};
}xCanSend5C0Info;

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
		xCanRev360Info CanRevInfo360;
	};
	/*接收PDO6， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData6[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		xCanRev2F0Info CanRevInfo2F0;
	};
	/*接收PDO7， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData7[8];
				/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
	xCanRev1E0Info CanRevInfo1E0;
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
		xCanSend258Info CanSend258Info;
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