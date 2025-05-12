#ifndef _USER_CAN_SHXianGongAGV_CAN_ECU_
#define _USER_CAN_SHXianGongAGV_CAN_ECU_
#include "stdint.h"


typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t b1Saftsign: 1;
		uint8_t	b1ForwardSign: 1;
		uint8_t b1BackwardSign: 1;
		uint8_t	b1BrakewardSign: 1;
		uint8_t b4Reserver: 4;
		
		uint8_t u8VCUSpeedL;             //0-2700
		uint8_t u8VCUSpeedH;             //0-2700
		uint8_t u8VCUAccelerateTime;	 //0-255 (0-25.5) 加速时间
		uint8_t u8VCUDecelerateTime;     //0-255 (0-25.5) 减速时间
		uint8_t u8Reserve1;
		uint8_t u8Reserve2;
		uint8_t u8BatterSoc;		
	};
	
}xCanRev203Info;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t b1LiftLockword: 1;
		uint8_t	b1LiftUpword: 1;
		uint8_t	b1LiftDownword: 1;
		uint8_t b1Beepword: 1;
		uint8_t b1Reserve: 4;
		
		uint8_t u8LiftSpeedL;		//0-1000  (0-100%)
		uint8_t u8LiftSpeedH;       
		uint8_t u8DownSpeedL;       //0-1000  (0-100%)
		uint8_t u8DownSpeedH;
		uint8_t u8SteerSpeedrate;   //0-255   (0-100)
		uint8_t u8Reserver1;
		uint8_t u8Reserver2;
	};
} xCanRev303Info;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8RemoteSteer;
		uint8_t u8RemoteDir;
		uint8_t u8RemoteSteerAngle;
		uint8_t b1RemoteEMS: 1;
		uint8_t b7RemoteReserve: 7;
		uint8_t u8LiftDir;
		uint8_t u8leftDir;
		uint8_t u8ForwardDir;
		uint8_t u8QingxieDir;
	} ;
} xCanRev1A3Info;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8MoveSpeedL;	//驱动转速
		uint8_t u8MoveSpeedH;	//驱动转速
		uint8_t u8Reserve1; 	//预留
		uint8_t u8Reserve2; 	//预留
		uint8_t u8ErrorCode;	//故障码	
		uint8_t u8Reserve3;
		uint8_t b1EmsSwiState: 1;
		uint8_t b1HandSwiState: 1;
		uint8_t b1ErrorState: 1;
		uint8_t b1upLimit: 1;
		uint8_t b1Reserve1: 1;
		uint8_t b1Reserve2: 1;
		uint8_t b1MainDriver: 1;
		uint8_t b1EmsState: 1;	
		uint8_t u8PumErrorstate;
	};
}xCanSend183Info;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8ControlTempL;			//控制器温度  -1000 - 3000（-100 - 300）
		uint8_t u8ControlTempH;			//控制器温度  -1000 - 3000（-100 - 300）
		uint8_t u8ControlCurrentL; 		//控制器电流	 0-10000 (0-1000)
		uint8_t u8ControlCurrentH; 		//控制器电流	 0-10000 (0-1000)
		uint8_t u8BatterSOC;					//电池电量

		uint8_t b1ForwardLimit: 1;
		uint8_t b1BackwardLimit: 1;
		uint8_t b1UpLimit: 1;
		uint8_t b1DownLimit: 1;
		uint8_t b1LeftLimit: 1;
		uint8_t b1RightLimit: 1;
		uint8_t b1OpenLimit: 1;
		uint8_t b1CloseLimit: 1;	

		uint8_t u8MotorTempL;	      //电机温度  -1000 - 3000（-100 - 300）
		uint8_t u8MotorTempH;         //电机温度  -1000 - 3000（-100 - 300）
	};
}xCanSend283Info;


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
		xCanRev203Info CanRevInfo203;
	};
	/*接收PDO6， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData6[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		xCanRev303Info CanRevInfo303;
	};
	/*接收PDO7， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8RevData7[8];
		/*此处添加要接收的结构体类型， for example： xCanRev1ACInfo CanRevInfo1*/
		xCanRev1A3Info CanRevInfo1A3;
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
	xCanSend183Info CanSend183Info;
	};  
	/*发送PDO6， 请根据实际情况修改里面的结构体*/
	union
	{
		uint8_t u8SendData6[8];
		/*此处添加要发送的结构体类型， for example： xCanSend23CInfo CanSend23CInfo*/
	xCanSend283Info CanSend283Info;
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