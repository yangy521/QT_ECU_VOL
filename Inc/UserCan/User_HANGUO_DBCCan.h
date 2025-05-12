#ifndef _USER_CAN_HANGUO_DBC_CAN_ECU_
#define _USER_CAN_HANGUO_DBC_CAN_ECU_
#include "stdint.h"

/*xCan Send*/
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
		uint8_t u8BMSSOCL;
		uint8_t u8BMSSOCH;
		uint8_t u8Reserve1;
		uint8_t u8Reserve2;
		uint8_t u8Reserve3;
		uint8_t u8Reserve4;
		uint8_t u8Reserve5;
		uint8_t u8Reserve6;
	};
}xCanRev303Info;                  //BMS

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8Reserve1;
		uint8_t u8Reserve2;
		uint8_t u8Reserve3;
		uint8_t u8Reserve4;
		uint8_t u8HandleInput1;
		uint8_t u8HandleInput2;
		uint8_t u8ErrCode;
		uint8_t u8WarnCode;
	};
}xCanRev500Info;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t YearL;
		uint8_t YearH;
		uint8_t Month;
		uint8_t Day;
		uint8_t Hour;
		uint8_t Minute;
		uint8_t Sec;
		uint8_t u8Reserve1;
	};
}xCanRev501Info;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t FWD: 1;
		uint8_t REV: 1;
		uint8_t Reseve2: 1;
		uint8_t Batterytype: 1;
		uint8_t Tuttlespeed: 1;
		uint8_t Reseve5: 1;
		uint8_t Reseve6: 1;
		uint8_t Reseve7: 1;
			
		uint8_t BatterSoc;
		uint8_t Speed;
		uint8_t Odometer1;
		uint8_t Odometer2;
		uint8_t Odometer3;
		uint16_t Houemeter;
	};
}xCanSend380Info;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t ErrCodeMove;
		uint8_t ErrCodeProp;
		uint8_t ErrCodeStree;
		uint8_t ErrCodeBatter;
		uint8_t Reseve5;
		uint8_t Reseve6;
		uint8_t Reseve7;
		uint8_t Reseve8;
	};
}xCanSend390Info;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t b1ForWord: 1;
		uint8_t b1NWord: 1;
		uint8_t b1BackWord: 1;
		uint8_t b1SpeedMode: 1;
		uint8_t b1Throttle: 1;
		uint8_t b1Break: 1;
		uint8_t b1EMBreak: 1;
		uint8_t b1Module: 1;
		
		uint8_t b6Reseve: 6;
		uint8_t b1BreakOn: 1;
		uint8_t b1PowerOn: 1;
		
		uint8_t Reseve3;
		uint8_t Reseve4;
		uint8_t Reseve5;
		uint8_t Reseve6;
		uint8_t Reseve7;
		uint8_t Reseve8;
	};
}xCanSend101Info;
typedef union
{
	uint8_t u8Data[8];
}xCanSend102Info;
typedef union
{
	uint8_t u8Data[8];
}xCanSend103Info;
typedef union
{
	uint8_t u8Data[8];
}xCanSend104Info;


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
		xCanRev303Info CanRevInfo303;
	};
	/*接收PDO6， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData6[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		xCanRev500Info CanRevInfo500;
	};
	/*接收PDO7， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData7[8];
				/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		xCanRev501Info CanRevInfo501;
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