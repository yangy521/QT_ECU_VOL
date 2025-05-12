#ifndef _USER_CAN_4IN1_ECU_
#define _USER_CAN_4IN1_ECU_
#include "stdint.h"

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint16_t	u16BatteryVol;
		uint16_t	u16PowBatCur;
		uint16_t	u16PowBatCap;
		uint16_t	u16PowBatRemainCap;
	};
}xBmsInfo1;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint16_t	u16PowBatCnt;
		uint8_t 	u8Soc;
		uint8_t		u8Byte3;
		uint8_t 	u8Byte4;
		uint8_t 	u8Byte5;
		uint8_t 	b1BatTemp1Err: 1;
		uint8_t 	b1BatDisChargeCurHigh1Err: 1;
		uint8_t 	b1BatTotalVolLow1Err: 1;
		uint8_t 	b1BatSingleVolLow1Err: 1;
		uint8_t 	b1BatSingleVolLow2Err: 1;
		uint8_t 	b1BatVolDiffErr: 1;
		uint8_t 	b1BatTempDiffErr: 1;
		uint8_t 	b1BatDisChargeCurHigh2Err: 1;
		uint8_t 	b1BatDisChargeTempHigh2Err: 1;
		uint8_t 	b1BatTotalVolLow2Err: 1;
		uint8_t 	b6Reserve1: 6;
	};
}xBmsInfo2;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		;
	};
}xBmsInfo3;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		;
	};
}xBmsInfo4;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t		 SOC;
		uint8_t		RESERVED;
		uint8_t		ECUErr;
		uint8_t 	WorkMode;
		uint8_t		u8WorkTimeLL;
		uint8_t		u8WorkTimeLH;
		uint8_t		u8WorkTimeHL;
		uint8_t		u8WorkTimeHH;
	};
}xHMISendPdo;

typedef struct
{
	/*接收PDO1， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData1[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
	//	xCanRev1ACInfo CanRevInfo1;
		xBmsInfo1	BmsInfo1;
	};
	/*接收PDO2， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData2[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		xBmsInfo2	BmsInfo2;
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
//		xCanSend33CInfo CanSend33CInfo;
	};
	/*发送PDO3， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8SendData3[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
//		xCanSend43CInfo CanSend43CInfo;
	};
	/*发送PDO4， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8SendData4[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
//		xCanSend53CInfo CanSend53CInfo;
	};
	/*发送PDO5， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8SendData5[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
		xHMISendPdo sgHMISendPdo;
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