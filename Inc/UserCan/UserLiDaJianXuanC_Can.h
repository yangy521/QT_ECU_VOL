#ifndef _USER_CAN_LIDA_581_CAN_ECU_
#define _USER_CAN_LIDA_581_CAN_ECU_
#include "stdint.h"

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
		uint8_t	BMS_SOC;
		uint8_t	u8Reserve6;
		uint8_t	u8Reserve7;

	};
}xCanRev444Info;

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
		
		uint8_t b1Vehicle_UnLock: 1;
		uint8_t b1Vehicle_PreLock: 1;
		uint8_t b1Vehicle_Lock: 1;
		uint8_t b1Vehicle_PreUnLock: 1;
		uint8_t b4Reserve: 4;
		
		uint8_t b2Gear_State: 2;
		uint8_t b1Main_Connect: 1;
		uint8_t b1Err_State: 1;
		uint8_t b1ZuoYi_State: 1;
		uint8_t b1Handle_State: 1;
		uint8_t b1JiaoSha_State: 1;
		uint8_t b1Charg_State: 1;
	
		uint8_t b2Speed_Mode: 2;
		uint8_t b1LiftUp_Mode: 1;
		uint8_t b1CeYi_Mode: 1;
		uint8_t b1CmdSpeed_Mode: 1;
		uint8_t b1QianXie_Mode: 1;
		uint8_t b2Reserve: 2;
		
		uint8_t SpeedfdbL;
		uint8_t SpeedfdbH;
		
		uint8_t MotorValL;
		uint8_t MoterValH;
		
		uint8_t b1Emergency: 1;
		uint8_t b1LiftDown: 1;
		uint8_t b1QianHouQin: 1;
		uint8_t b1QianHouYi: 1;
		uint8_t b1ZuoYouYi: 1;
		uint8_t b1PeDal: 1;
		uint8_t b1FENCE2: 1;
		uint8_t b1HightLimit: 1;
	};
}xCanSend3BBInfo;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		
		uint8_t LitfValL;
		uint8_t LiftValH;
		
		uint8_t Gantry_HeightL;
		uint8_t Gantry_HeightH;
		
		uint8_t Load_WeightL;
		uint8_t Load_WeightH;
		
		uint8_t b1Lift1800Limit: 1;
		uint8_t b1StreeSpeetLimit: 1;
		uint8_t b1HightSpeedLimit: 1;
		uint8_t b1PorpDriver: 1;
		uint8_t b1LiftDownDriver: 1;
		uint8_t b3Reserve: 3;
		
		uint8_t Reserve8;
		
	};
}xCanSend3BCInfo;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t Reserve1;
		uint8_t Reserve2;
		uint8_t Reserve3;
		uint8_t Reserve4;
		uint8_t Reserve5;
		uint8_t Reserve6;
		uint8_t Reserve7;
		uint8_t Reserve8;
		
	};
}xCanSend3BEInfo;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t HourCountL;
		uint8_t HourCountH;
		uint8_t Reserve3;
		uint8_t Reserve4;
		uint8_t BatterSoc;
		uint8_t ErrCode;
		
		uint8_t b1Reserve1: 1;
		uint8_t b1Reserve2: 1;
		uint8_t b1Reserve3: 1;
		uint8_t b1Reserve4: 1;
		uint8_t b1Reserve5: 1;
		uint8_t b1Reserve6: 1;
		uint8_t b1Tiller: 1;
		uint8_t b1MainConnect: 1;
		
		uint8_t Reserve8;
		
	};
}xCanSend488Info;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t SpeedH;
		uint8_t SpeedL;
		uint8_t Reserve3;
		uint8_t Reserve4;
		uint8_t Reserve5;
		uint8_t Reserve6;
		uint8_t Reserve7;
		uint8_t Reserve8;
		
	};
}xCanSend489Info;


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
		xCanRev444Info CanRevInfo444;
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
	xCanSend260Info CanSend260Info;
};
	/*发送PDO6， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8SendData6[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
	xCanSend3BBInfo CanSend3BBInfo;
	};
	/*发送PDO7， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8SendData7[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
	xCanSend3BCInfo CanSend3BCInfo;
	};
	/*发送PDO8， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8SendData8[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
	xCanSend3BEInfo CanSend3BEInfo;
	};
	/*发送PDO9， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8SendData9[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
	xCanSend488Info CanSend488Info;
	};
	/*发送PDO10， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8SendData10[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
	xCanSend489Info CanSend489Info;
	};
}xCanSendPdoInfo;

#endif