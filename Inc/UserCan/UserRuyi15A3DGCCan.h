#ifndef _USER_CAN_RUYI15A3_DGC_ECU_
#define _USER_CAN_RUYI15A3_DGC_ECU_
#include "stdint.h"

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8BMS_VoltageH;
		uint8_t u8BMS_VoltageL;
		uint8_t u8BMS_CurrentH;
		uint8_t u8BMS_CurrentL;
		uint8_t	u8Soc;
		uint8_t u8Reserve1;
		uint8_t u8Reserve2;
		uint8_t u8BMS_PDO_Count;
	};
}xCanRev200Info;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8BMS_ERR1L;
		uint8_t u8BMS_ERR1H;
		uint8_t u8BMS_ERR2L;
		uint8_t u8BMS_ERR2H;
		uint8_t	u8BMS_ERR3L;
		uint8_t u8BMS_ERR3H;
		uint8_t u8BMS_ERR_Require;
		uint8_t u8Reserve;
	};
}xCanRev190Info;

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
		uint8_t b1ForwardState:1;
		uint8_t	b1BackwardState:1;
		uint8_t b1Ready:1;
		uint8_t b1HourEn:1;
		uint8_t b1Upright:1;
		uint8_t b1Park:1;
		uint8_t b1SeatBelt:1;
		uint8_t b1Maint:1;
		
		uint8_t u8ErrorMove;	//MCU故障转发
		uint8_t u8ErrorSteer; //转发报文360[1]
		uint8_t u8Movespeed; 	//速度
		uint8_t u8ErrorBMS;		
		uint8_t u8Soc;
		uint8_t u8HourCountL;
		uint8_t u8HourCountH;
	};
}xCanSend260Info;

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
		//xCanRev1ACInfo CanRevInfo1;
		xCanRev190Info CanRevInfo1;
	};
	/*接收PDO6， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData6[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		//xCanRev360Info CanRevInfo2;
		xCanRev200Info CanRevInfo2;
	};
	/*接收PDO7， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData7[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		xCanRev360Info CanRevInfo3;
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