/*******************************************************************************
* Filename: UserStackerTruckProc.h	                                           *
* Description: 逻辑层的头文件，包含对应的回调函数	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/07/22    														   *
* Revision:	 														 		   *
*******************************************************************************/
#ifndef _USER_CAN_COMM_HC_PODAOCHE_H_
#define _USER_CAN_COMM_HC_PODAOCHE_H_

#include "stdint.h"
#include "Userdef.h"

#if (USER_TYPE == USER_HANGCHA_PODAOCHE)

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t b1Reserve1: 1;
		uint8_t b1EmsReq: 1;      //急反请求
		uint8_t b1SlowModeReq: 1; //龟速模式
		uint8_t b1HornReq: 1;     //喇叭
		uint8_t b1LiftReq: 1;     //起升
		uint8_t b1DownReq: 1;     //下降
		uint8_t b1LeanBackWardReq: 1; //后倾
		uint8_t b1LeanForWardReq: 1;  //前倾
		
				
		uint8_t b1RampModeReq: 1;   //进入坡道模式
		uint8_t b1RampModeExit: 1;  //退出坡道模式
		uint8_t b1Reserve2: 1;
		uint8_t b1Reserve3: 1;
		uint8_t b1Reserve4: 1;
		uint8_t b1Reserve5: 1;
		uint8_t b1Reserve6: 1;
		uint8_t b1Reserve7: 1;
		
		int16_t i16ThrottleValue;    //手柄命令值 -2048->2047
		
		uint8_t  b8Reserve9;
		
		uint8_t i8LiftDownValue;    //起升下降命令值
		
		uint8_t b8Reserve10;
		
		uint8_t  i8LeanValue;       //前倾后倾命令值
	};
}xCanRev1E0Info;
typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t Xsign;
		uint8_t Xhigh;
		uint8_t Xlow1;
		uint8_t Xlow2;
		uint8_t Ysign;
		uint8_t Yhigh;
		uint8_t Ylow1;
		uint8_t Ylow2;
	};
}xCanRev18XInfo;
typedef union
{
	uint8_t u8Data[8];
	struct
	{
	 /*BMS 协议*/
		uint8_t u8VoltageLow;
		uint8_t u8VoltageHigh;
		uint8_t u8CurrentLow;
		uint8_t u8CurrentHigh;
		uint8_t u8Soc;
		uint8_t u8Volume;          //容量
		
		uint8_t b1OverVotageErr:1;  //总体电压过高#159
		uint8_t b1UnderVotageErr:1; //总体严重欠压#148
		uint8_t b1MissComErr:1;     //BMS锂电池故障#148
		uint8_t b1SoloUnderVotageErr:1; //单体欠压 #158电压警告
		uint8_t b1OverCurrentErr:1;   //放电过流#148 锂电池故障
		uint8_t b1OverTmpErr:1;       //严重过温保护 #157
		uint8_t b1TmpProtectErr:1;    //温度保护（一般） #156降功率
		uint8_t b1ChargeFlag:1;       //充电状态
		
		uint8_t Reserve1;
		
	 
	};
}xCanRev2F0Info;
typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint16_t u16StateWord;
		int16_t s16SpeedMeasured;
		int16_t s16SteerAngle;
		uint16_t u16CurrentMeasured;
	};
}xEcuInfo1;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8ErrorCode;
		uint8_t u8MotorTmp;
		uint8_t u8BoardTmp;
		uint8_t u8BDIPercent;
		uint8_t u8Analog1;
		uint8_t u8Analog2;
		uint8_t u8Reserved;
		uint8_t u8SoftwareVer;
	};
}xEcuInfo2;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8ErrCode288;
		uint8_t u8Reserve2;
		uint8_t u8Reserve3;
		uint8_t u8Reserve4;
		uint8_t u8Reserve5;
		uint8_t u8Reserve6;
		uint8_t u8Reserve7;
		uint8_t u8Reserve8;
	};
}xCanRev288Info;
typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8ErrCode289;
		uint8_t u8Reserve2;
		uint8_t u8Reserve3;
		uint8_t u8Reserve4;
		uint8_t u8Reserve5;
		uint8_t u8Reserve6;
		uint8_t u8Reserve7;
		uint8_t u8Reserve8;
	};
}xCanRev289Info;
/*    仪表协议258 358  2F8  */
typedef union
{
	uint8_t u8Data[8];
	struct
	{
		/* byte 0 */
		uint8_t u8Base;//基本功能 默认10
		/* byte 1 */
		uint8_t b1BmsChoice0:1;
		uint8_t b1BmsChoice1:1;
		uint8_t b1TimeChoice0:1;
		uint8_t b1TimeChoice1:1;
		uint8_t b1LowBowerAlm:1;
		uint8_t b1PowerHourCount:1;
		uint8_t b1WorkHourCount:1;
		uint8_t b1FaultSymbol:1;
		/* byte 2 */
		uint8_t b1WorkMode0:1;
		uint8_t b1WorkMode1:1;
		uint8_t b1ShowSpeed:1;
		uint8_t b1SpeedUnit:1;
		uint8_t b1Reserve1:1;
		uint8_t b1NoLiftSymbol:1;
		uint8_t b1BrakeSymbol:1;
		uint8_t b1ChargeSymbol:1;
		/*  byte 3 */
		uint8_t u8ControlType;//控制器类型
		/*  byte 4 */
		uint8_t u8ErrCode;    //错误代码
		/*  byte 5 */
		uint8_t u8Speed;      //车速
		/*  byte 6 */        
		uint8_t u8BMS;        //电量
		/*  byte 7 */
		uint8_t u8Reserve2;
	};
}xCanTx258Info;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		/* byte 0 */
		uint8_t u8Base;//功能码
		/* byte 1 */
		uint8_t u8HourCountL;
		uint8_t u8HourCountM;
		uint8_t u8HourCountH;
		
		uint8_t u8Reserve1;
		uint8_t u8Reserve2;
		uint8_t u8Reserve3;
		uint8_t u8Reserve4;
	};
}xCanTx2F8Info;
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
		xCanRev1E0Info CanRevInfo5;
		
	};
	/*接收PDO6， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData6[8];
		xCanRev18XInfo CanRevInfo6;
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		
	};
	/*接收PDO7， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData7[8];
		xCanRev18XInfo CanRevInfo7;
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
	};
	/*接收PDO8， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData8[8];
		xCanRev2F0Info CanRevInfoBMS;
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
	};	
	union
	{
		uint8_t u8RevData9[8];
		xCanRev288Info CanRevInfo288;
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
	};
	union
	{
		uint8_t u8RevData10[8];
		xCanRev289Info CanRevInfo289;
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
		xCanTx258Info CanHMISendInfo;

		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
	};
	/*发送PDO6， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8SendData6[8];
		xCanTx2F8Info Can2F8SendInfo;
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

#endif
