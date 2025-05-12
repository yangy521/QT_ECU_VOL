/*******************************************************************************
* Filename: UserStackerTruckProc.h	                                           *
* Description: 逻辑层的头文件，包含对应的回调函数	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/07/22    														   *
* Revision:	 														 		   *
*******************************************************************************/
#ifndef _USER_GANGLI_PHZ15T_CAN_H_
#define _USER_GANGLI_PHZ15T_CAN_H_

#include "stdint.h"
#include "Userdef.h"

#if (USER_TYPE == USER_GANGLI_PHZ15T_MOVE)

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8Soc;
		uint16_t u16BatteryVoltage;
		uint16_t u16DischargeCurrent;
		uint16_t u16MaxSingleVoltage;
		
		uint8_t b4HeartBeat: 4;
		uint8_t b4Reserve0: 4;
	};
}xCanRev17FInfo;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint16_t u16MinSingleVoltage;
		uint16_t u16MaxSingleTemperature;
		uint16_t u16InsulationLeakageValue;
		uint16_t u16Reserve0;

	};
}xCanRev27FInfo;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t b2SingleDropoutVoltageErr: 2;
		uint8_t b2SingleOverVoltageErr: 2;
		uint8_t b2SingleUnderVoltageErr: 2;
		uint8_t b2SingleDropoutTemperatureErr: 2;
		
		uint8_t b2OverVoltageErr: 2;
		uint8_t b2UnderVoltageErr: 2;
		uint8_t b2OverTemperatureErr: 2;
		uint8_t b2UnderTemperatureErr: 2;
		
		uint8_t b2LowSOCErr: 2;
		uint8_t b2OverCurrentErr: 2;
		uint8_t b2InsulationErr: 2;
		uint8_t b2BMSErr: 2;		
		
		uint8_t u8Reserve0;
		uint8_t u8Reserve1;
		uint8_t u8Reserve2;
		uint8_t u8Reserve3;
		uint8_t u8Reserve4;
	};
}xCanRev37FInfo;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8ErrorLift;
		uint8_t u8Reserve0;
		uint8_t u8Reserve1;
		uint8_t u8Reserve2;
		uint8_t u8Reserve3;	
		uint8_t u8Reserve4;
		uint8_t u8Reserve5;
		uint8_t u8Reserve6;	
	};
}xCanRev47FInfo;

//typedef union
//{
//	uint8_t u8Data[8];
//	struct
//	{
//		uint16_t u16StateWord;
//		int16_t s16SpeedMeasured;
//		int16_t s16SteerAngle;
//		uint16_t u16CurrentMeasured;
//	};
//}xEcuInfo1;

//typedef union
//{
//	uint8_t u8Data[8];
//	struct
//	{
//		uint8_t u8ErrorCode;
//		uint8_t u8MotorTmp;
//		uint8_t u8BoardTmp;
//		uint8_t u8BDIPercent;
//		uint8_t u8Analog1;
//		uint8_t u8Analog2;
//		uint8_t u8Reserved;
//		uint8_t u8SoftwareVer;
//	};
//}xEcuInfo2;


typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t b1ForWardStat: 1;
		uint8_t b1BackWardStat: 1;
		uint8_t b1HourEn: 1;
		uint8_t b1Upright: 1;
		uint8_t b1Park: 1;
		uint8_t b1SeatBelt: 1;
		uint8_t b1Maint: 1;
		uint8_t b1Reserve0: 1;
		
		uint8_t u8ErrorMove;
		uint8_t u8ErrorLift;
		uint8_t u8SMoveSpeed;
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
		uint8_t u8StopLift;
		uint8_t u8MoveFlag;
		
		uint8_t u8NoAct;
		uint8_t u8NoLift;
		//uint16_t u16Reserve1;
		uint16_t u16Reserve2;
		uint16_t u16Reserve3;
	};
}xCanSend270Info;

typedef struct
{
	/*接收PDO1， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData1[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		//xCanRev1E0Info CanRevInfo1;
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
		xCanRev17FInfo CanRevInfo5;
		
	};
	/*接收PDO6， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData6[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		xCanRev27FInfo CanRevInfo6;
	};
	/*接收PDO7， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData7[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		xCanRev37FInfo CanRevInfo7;
	};
	/*接收PDO8， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData8[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		xCanRev47FInfo CanRevInfo8;
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
		xCanSend270Info CanSend270Info;
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

#endif //(USER_ECU_OR_ET_TYPE == USER_GANGLI_PHZ15T_MOVE)

#if((USER_TYPE == USER_GANGLI_PHZ15T_LIFT))
typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8StopLift;
		uint8_t u8MoveFlag;
		
		uint16_t u16Reserve1;
		uint16_t u16Reserve2;
		uint16_t u16Reserve3;
	};
}xCanRev270Info;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8ErrorLift;
		uint8_t u8NoAct;
		uint8_t u8Reserve1;
		uint8_t u8Reserve2;
		uint8_t u8Reserve3;	
		uint8_t u8Reserve4;
		uint8_t u8Reserve5;
		uint8_t u8Reserve6;	
	};
}xCanSend47FInfo;

typedef struct
{
	/*接收PDO1， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData1[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		//xCanRev1E0Info CanRevInfo1;
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
		xCanRev270Info CanRevInfo5;
		
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
		xCanSend47FInfo CanSend47FInfo;
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

#endif //((USER_ECU_OR_ET_TYPE == USER_GANGLI_PHZ15T_MOVE))

#endif	//_USER_GANGLI_PHZ15T_CAN_H_