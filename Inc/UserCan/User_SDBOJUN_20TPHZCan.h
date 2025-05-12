#ifndef _USER_CAN_SDBOJUN_PHZ_CAN_ECU_
#define _USER_CAN_SDBOJUN_PHZ_CAN_ECU_
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
		uint8_t u8MoveStatePdo;
		uint8_t u8MoveErrPdo;
		uint8_t u8MotorValL;
		uint8_t u8MotorValH;
		uint8_t u8StreeValL;
		uint8_t u8StreeValH;
		uint8_t u8VehicleState;
		uint8_t u8SOC;
	};
}xCanSend1ACInfo;
/*xCan Rev*/
typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8MoveStatePdo;
		uint8_t u8MoveErrPdo;
		uint8_t u8MotorValL;
		uint8_t u8MotorValH;
		uint8_t u8StreeValL;
		uint8_t u8StreeValH;
		uint8_t u8VehicleState;
		uint8_t u8SOC;
	};
}xCanRev1ACInfo;
typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8MoveStatePdo;
		uint8_t u8MoveErrPdo;
		uint8_t u8MotorValL;
		uint8_t u8MotorValH;
		uint8_t u8StreeValL;
		uint8_t u8StreeValH;
		uint8_t u8VehicleState;
		uint8_t u8SOC;
	};
}xCanRev1ADInfo;
typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8MoveStatePdo;
		uint8_t u8MoveErrPdo;
		uint8_t u8MotorValL;
		uint8_t u8MotorValH;
		uint8_t u8StreeValL;
		uint8_t u8StreeValH;
		uint8_t u8VehicleState;
		uint8_t u8SOC;
	};
}xCanRev1AEInfo;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8REC1;
		uint8_t u8REC2;
		uint8_t u8REC3;
		uint8_t u8REC4;
		uint8_t u8REC5;
		uint8_t u8REC6;
		uint8_t u8SOC;
		uint8_t u8REC7;
	};
}xCanRev001Info;



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
		xCanRev1ACInfo CanRevInfo1AC;
	};
	/*接收PDO6， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData6[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		xCanRev1ADInfo CanRevInfo1AD;
	};
	/*接收PDO7， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData7[8];
				/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		xCanRev1AEInfo CanRevInfo1AE;
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
		xCanSend1ACInfo CanSend1ACInfo;	
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
	xCanSend260Info CanSend260Info;
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