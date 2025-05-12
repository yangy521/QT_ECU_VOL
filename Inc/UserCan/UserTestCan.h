#ifndef _USER_CAN_TEST_ECU_
#define _USER_CAN_TEST_ECU_
#include "stdint.h"

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint32_t b1Swi1: 1;
		uint32_t b1Swi2: 1;
		uint32_t b1Swi3: 1;
		uint32_t b1Swi4: 1;
		uint32_t b1Swi5: 1;
		uint32_t b1Swi6: 1;
		uint32_t b1Swi7: 1;
		uint32_t b1Swi8: 1;
		uint32_t b1Driver1R: 1;
		uint32_t b1Driver2R: 1;
		uint32_t b1Driver3R: 1;
		uint32_t b1Driver4R: 1;
		uint32_t b1Driver5R: 1;
		uint32_t b1Driver6R: 1;
		uint32_t b1Driver7R: 1;
		uint32_t b1Driver8R: 1;
		uint32_t b1Driver9R: 1;
		uint32_t b1Driver10R: 1;
		uint32_t b1Driver11R: 1;
		uint32_t b1Driver12R: 1;
		uint32_t b12Reserve1: 12;
		
		uint8_t u8McuErrCode;
		uint8_t u8EcuErrCode;
		
		uint16_t u16SoftVer;
		
	};
}xEcuInfo1;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint16_t u16Ai1Vol;
		uint16_t u16Ai2Vol;
		uint16_t u16Ai3Vol;
		
		uint16_t u16HardVer;
		
	};
}xEcuInfo2;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint16_t u16KsiVol;
		uint16_t u16VbusVol;
		uint16_t u16PropCur1;
		uint16_t u16PropCur2;
	};
}xEcuInfo3;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		;
	};
}xEcuInfo4;

typedef struct
{
	/*接收PDO1， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData1[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		//xBmsInfo1 CanRevInfo1;
	};
	/*接收PDO2， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData2[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		//xBmsInfo2 CanRevInfo2;
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
		xEcuInfo1 CanSendInfo1;
	};
	/*发送PDO2， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8SendData2[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
		xEcuInfo2 CanSendInfo2;
	};
	/*发送PDO3， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8SendData3[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
		xEcuInfo3 CanSendInfo3;
	};
	/*发送PDO4， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8SendData4[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
		xEcuInfo4 CanSendInfo4;
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
