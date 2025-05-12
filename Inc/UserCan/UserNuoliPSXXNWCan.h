#ifndef _USER_CAN_NUOLI_PSNW_ECU_
#define _USER_CAN_NUOLI_PSNW_ECU_
#include "stdint.h"

/*����PDO��ת���ֱ���BMS*/

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint16_t b1NeutralPose:1;
		uint16_t b1EmergencyReverse:1;
		uint16_t b1SnailRequest:1;
		uint16_t b1Horn:1;
		uint16_t b1Lift1:1;
		uint16_t b1LiftDown1:1;
		uint16_t b1Lift2:1;
		uint16_t b1LiftDown2:1;
		uint16_t b1Pick1:1;
		uint16_t b1Pick2:1;
		uint16_t b2Spare:2;
		uint16_t b1ZhiLi:1;
		uint16_t b2Reserve:2;
		uint16_t b1StuffToggle:1;
		
		int16_t i16MoveThrottle;
		int16_t i16LiftThrottle;
		int16_t i16AuxiliaryThrottle;
	};
}xCanRev1E0Info;//�ֱ�

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8ControlByte;
		uint8_t u8Reserve1;
		uint8_t u8Reserve2;
		uint8_t u8Reserve3;
		uint8_t u8Reserve4;
		uint8_t u8SOC;
		uint8_t u8Reserve6;
		uint8_t u8Reserve7;
	};
}xCanRev1A1Info;//�Ǳ�

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8SteerCurrentL;
		uint8_t u8SteerCurrentH;
		uint8_t u8SteerPotVoltageL;
		uint8_t u8SteerPotVoltageH;
		uint8_t u8SteerTmpL;
		uint8_t u8SteerTmpH;
		uint8_t u8Reserve6;
		uint8_t u8Reserve7;
	};
}xCanRev3E0Info;//ת��

typedef union
{
	uint8_t u8Data[8];
	struct 
		{
			uint8_t b1Lock:1;
			uint8_t b1LowSpeed1:1;
			uint8_t b1LowSpeed2:1;
			uint8_t b1LiftLock:1;
			uint8_t b1DriverState:1;
			uint8_t b1DriverTypeEn:1;
			uint8_t b1RemoteOpen:1;
			uint8_t b1RemoteClose:1;
			
			uint8_t u8MaxSpeed;
			uint8_t u8Reserve1;
			uint8_t u8SerialNumber12;
			uint8_t u8SerialNumber34;
			uint8_t u8SerialNumber56;
			uint8_t u8SerialNumber78;
			uint8_t u8SerialNumber9A;
		};
}xCanRev27AInfo;//����2

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
		
		uint8_t u8SteerAngleL;
		uint8_t u8SteerAngleH;
		
		uint8_t u8SteerSpeedL;
		uint8_t u8SteerSpeedH;
		
		uint8_t u8Reserve3;
		uint8_t u8Reserve4;		
	};
}xCanRev361Info;//ת��

typedef union
{
	uint8_t u8Data[8];
	struct 
		{
			uint8_t b1HeartBeatState:1;
			uint8_t b1CardState:1;
			uint8_t b1ErrorState:1;
			uint8_t b5Reserve:5;
			
			uint8_t u8Reserve1;
			uint8_t u8Reserve2;
			uint8_t u8Reserve3;
			uint8_t u8Reserve4;
			uint8_t u8Reserve5;
			uint8_t u8Reserve6;
			uint8_t u8Reserve7;
		};
}xCanRev1B0Info;//ˢ����

typedef union
{
	uint8_t u8Data[8];
	struct 
		{
			uint8_t b1Lock:1;
			uint8_t b1LowSpeed1:1;
			uint8_t b1LowSpeed2:1;
			uint8_t b1LiftLock:1;
			uint8_t b1DriverState:1;
			uint8_t b1DriverTypeEn:1;
			uint8_t b1RemoteOpen:1;
			uint8_t b1RemoteClose:1;
			
			uint8_t u8Reserve1;
			uint8_t u8Reserve2;
			uint8_t u8Reserve3;
			uint8_t u8Reserve4;
			uint8_t u8Reserve5;
			uint8_t u8Reserve6;
			uint8_t u8Reserve7;
		};
}xCanRev270Info;//����1


typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t b1ForwardState:1;
		uint8_t	b1BackwardState:1;
		uint8_t b1MainContactor:1;
		uint8_t b1HourEn:1;
		uint8_t b1Upright:1;
		uint8_t b1Park:1;//�ƶ���
		uint8_t b1SeatBelt:1;
		uint8_t b1Maint:1;
		
		uint8_t u8ErrorMove;	//MCU����ת��
		uint8_t u8ErrorSteer; //ת������361[1]
		uint8_t u8Movespeed; 	//�ٶ�
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
		uint8_t b1ToggleBit:1;
		uint8_t b1SnailMonde:1;
		uint8_t b1PB0LI1BatteryType:1;
		uint8_t b1LiftDown:1;
		uint8_t	b1LeanForward:1;
		uint8_t b1LeanBackward:1;
		uint8_t b1UpRight:1;//ֱ��
		uint8_t b1MainContacter:1;

		
		uint8_t u8BatteryTypeL;
		uint8_t u8BatteryTypeH;

		uint8_t u8ErrorMove;	//MCU����ת��
		
		uint8_t	u8WorkTimeLL;
		uint8_t	u8WorkTimeLH;
		uint8_t	u8WorkTimeHL;
		uint8_t	u8WorkTimeHH;
	};
}xCanSend261Info;

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
		uint8_t u8Reserve5;
		uint8_t u8BMSWarning;
		uint8_t u8BMSError;
	};
}xCanRev2F1Info;//﮵�

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8Data1;
		uint8_t u8Data2;
		uint8_t u8Data3;
		uint8_t u8Data4;
		uint8_t u8Data5;
		uint8_t u8Data6;
		uint8_t u8Data7;
		uint8_t u8Data8;
	};
}xCanSendCommon;


typedef struct
{ 
	/*����PDO1�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8RevData1[8];
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
		
	};
	/*����PDO2�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8RevData2[8];
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
		
	};
	/*����PDO3�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8RevData3[8];
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
		
	};
	/*����PDO4�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8RevData4[8];
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
	};
	/*����PDO5�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8RevData5[8];
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
		xCanRev1E0Info CanRevInfo1E0;
	};
	/*����PDO6�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8RevData6[8];
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
		xCanRev361Info CanRevInfo361;
	};
	/*����PDO7�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8RevData7[8];
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
		xCanRev1A1Info CanRevInfo1A1;
	};
	/*����PDO8�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8RevData8[8];
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
		xCanRev2F1Info CanRevInfo2F1;

	};	
	union
	{
		uint8_t u8RevData9[8];
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
		xCanRev1B0Info CanRevInfo1B0;
	};
	union
	{
		uint8_t u8RevData10[8];
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
		xCanRev3E0Info	CanRevInfo3E0;
	};
	
}xCanRevPdoInfo;

typedef struct
{
	/*����PDO1�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8SendData1[8];
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
	};
	/*����PDO2�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8SendData2[8];
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
		//xCanSend33CInfo CanSend33CInfo;
	};
	/*����PDO3�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8SendData3[8];
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
		//xCanSend43CInfo CanSend43CInfo;
	};
	/*����PDO4�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8SendData4[8];
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
		//xCanSend53CInfo CanSend53CInfo;
	};
	/*����PDO5�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8SendData5[8];
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
		xCanSend261Info CanSend261Info;
	};
	/*����PDO6�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8SendData6[8];
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
		xCanSend260Info CanSend260Info;
	};
	/*����PDO7�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8SendData7[8];
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
		xCanSendCommon CanSend2F8Info;
	};
	/*����PDO8�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8SendData8[8];
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
		xCanSendCommon CanSend2F9Info;
	};
	union
	{
		uint8_t u8SendData9[8];
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
		xCanSendCommon CanSend2FAInfo;
	};
	union
	{
		uint8_t u8SendData10[8];
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
	};
}xCanSendPdoInfo;

#endif