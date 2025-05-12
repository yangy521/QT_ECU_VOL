#ifndef _USER_CAN_KELIZJDQY_LIFT_CAN_ECU_
#define _USER_CAN_KELIZJDQY_LIFT_CAN_ECU_
#include "stdint.h"


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
}xCanRev260Info;

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
		xCanRev260Info CanRevInfo260;
	};
	/*接收PDO6， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData6[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/

	};
	/*接收PDO7， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData7[8];
				/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
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