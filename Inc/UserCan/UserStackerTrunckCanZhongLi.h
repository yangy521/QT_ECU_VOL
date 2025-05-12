#ifndef _USER_CAN_ZHONGLI_DGC_
#define _USER_CAN_ZHONGLI_DGC_
#include "stdint.h"

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t b1StopStat: 1;
		uint8_t b1StartStat: 1;
		uint8_t b1Mode: 1;
		uint8_t b1Reserve0: 1;
		uint8_t b1Reserve1: 1;
		uint8_t b1Reserve2: 1;
		uint8_t b1Reserve3: 1;
		uint8_t b1Reserve4: 1;
		
		uint8_t	u8Reserve5;
		uint8_t	u8Reserve6;
		uint8_t	u8Reserve7;
		
		uint16_t u16SetTime;
		
		uint8_t	 u8Crc1;
		uint8_t  u8Crc2;	
	};
}xCanRev42CInfo;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8RepStat;
		uint8_t u8Reserve0;
		uint8_t u8Reserve1;
		uint8_t u8Reserve2;
		uint16_t u16SetTime;
		uint8_t u8Crc1;
		uint8_t u8Crc2;	
	};
}xCanSend3ACInfo;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8Const1;
		uint8_t u8Const2;
		uint8_t u8Const3;
		uint8_t u8Const4;
		
		uint8_t u8Data1;
		uint8_t u8Data2;
		
		uint8_t u8Const5;
		
		uint8_t u8State;
	};
}xCanRevDevice62CInfo;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8Const1;
		uint8_t u8Const2;
		uint8_t u8Const3;
		uint8_t u8Const4;
		
		uint8_t b1Const5: 1;
		uint8_t b1PasswordFunc: 1;
		uint8_t b1Free0: 1;
		uint8_t b1Free1: 1;
		uint8_t b1Free2: 1;
		uint8_t b1Free3: 1;
		uint8_t b1Free4: 1;
		uint8_t b1Free5: 1;
		
		uint8_t u8Free6;
		uint8_t u8Free7;
		uint8_t u8Free8;	
	};
}xCanRevPc62CInfo;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t FuncCode;
		uint8_t Index;
		uint8_t SendCmd;
		uint8_t SubIndex;
		uint8_t u8DataLL;
		uint8_t u8DataLH;
		uint8_t u8DataHL;
		uint8_t u8DataHH;		
	};
}xPARA62CInfo;


typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t b1Reserve0: 1;
		uint8_t b1DisableLiftUp: 1;
		uint8_t b1LimitSpd1: 1;
		uint8_t b1LimitSpd2: 1;
		uint8_t	b1MainDriverOpen: 1;
		uint8_t b1Reserve1: 1;
		uint8_t b1Reserve2: 1;
		uint8_t b1ToggleBit: 1;
		
		uint8_t u8Soc;
		
		int16_t i16DischargeCurrent;
		
		int16_t i16DownCurrent;
		
		int16_t i16BatteryActualCurrent;
	};
}xCanRev1ACInfo;


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
		
		int16_t i16SteerAngle;
		
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
		uint8_t u8Soc;
		
		uint8_t u8WorkTimeLL;
		uint8_t u8WorkTimeLH;
		uint8_t u8WorkTimeHL;
		uint8_t u8WorkTimeHH;
		
		uint8_t u8ErrCode;
		
		uint8_t b1SeatSwi: 1;
		uint8_t b1ForWardSwi: 1;
		uint8_t b1BackWardSwi: 1;
		uint8_t b1LiftUpSwi: 1;
		uint8_t b1LiftDownSwi: 1;
		uint8_t b1EmsSwi: 1;
		uint8_t b1ElectricBrakeSwi: 1;
		uint8_t b1Reserve0: 1;
		
		uint8_t u8Reserve1;	
	};
}xCanSend23CInfo;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8Reserve0;
		uint8_t u8Reserve1;
				
		uint8_t u8ErrCodeList;
		
		uint8_t u8SpdLowByte;
		uint8_t u8SpdHighByte;
		
		uint8_t u8SoftVersionLowByte;
		uint8_t u8SoftVersionHighByte;
		
		uint8_t u8Reserve2;	
	};
}xCanSend33CInfo;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint16_t u16BatteryVol;
		int16_t i16CtrlTemp;
		int16_t i16MotorTemp;
		int16_t i16MotorCurrent;
	};
}xCanSend43CInfo;


typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8AccPercent;
		uint8_t b1Reserve0: 1;
		uint8_t b1Reserve1: 1;
		uint8_t b1Reserve2: 1;
		uint8_t b1Reserve3: 1;
		uint8_t b1Reserve4: 1;
		uint8_t b1Reserve5: 1;
		uint8_t b1SeatBelt: 1;
		uint8_t b1Reserve6: 1;
		uint8_t u8Reserve7;
		uint8_t u8Reserve8;
		uint8_t u8Reserve9;
		uint8_t u8Reserve10;
		uint8_t u8Reserve11;
		uint8_t u8Reserve12;	
	};
}xCanSend53CInfo;


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
		xCanRev1ACInfo CanRevInfo1;
	};
	/*接收PDO6， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData6[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		xCanRev360Info CanRevInfo2;
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
		xCanSend23CInfo CanSend23CInfo;
	};
	/*发送PDO2， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8SendData2[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
		xCanSend33CInfo CanSend33CInfo;
	};
	/*发送PDO3， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8SendData3[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
		xCanSend43CInfo CanSend43CInfo;
	};
	/*发送PDO4， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8SendData4[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
		xCanSend53CInfo CanSend53CInfo;
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