#ifndef _USER_CAN_2IN1_ECU_
#define _USER_CAN_2IN1_ECU_
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
		;
	};
}xTboxInfo1;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		;
	};
}xTboxInfo2;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		;
	};
}xTboxInfo3;


typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint16_t u16Angle;
		uint16_t u16Pressure1;
		uint16_t u16Pressure12;
		uint8_t  u8LoadRate;
		uint8_t  CalibrationStatus;
	};
}xEcuInfo1;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8ForwardValveCurrent;
		uint8_t u8BackwardValveCurrent;
		uint8_t u8LiftValveCurrent;
		uint8_t u8PropValveCurrent;
		uint8_t u8OnOffValveCurrent;
		uint8_t u8TurnRightValveCurrent;
		uint8_t u8TurnLeftValveCurrent;
		uint8_t u8BrakeValveCurrent;
	};
}xEcuInfo2;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint16_t u16BatteryVoltage;
		uint16_t u16ExtSignal;
		uint8_t	 u8PcuKeyInfo;
		uint8_t  u8OutCtrlInfo;
		uint8_t  u8MotorSpd;
		uint8_t  u8HandleAnalog;
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
	/*����PDO1�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8RevData1[8];
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
		xBmsInfo1 CanRevInfo1;
	};
	/*����PDO2�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8RevData2[8];
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
		xBmsInfo2 CanRevInfo2;
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
	};
	/*����PDO6�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8RevData6[8];
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
	};
	/*����PDO7�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8RevData7[8];
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
	};
	/*����PDO8�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8RevData8[8];
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
	};	
	union
	{
		uint8_t u8RevData9[8];
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
	};
	union
	{
		uint8_t u8RevData10[8];
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
	};
	
}xCanRevPdoInfo;

typedef struct
{
	/*����PDO1�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8SendData1[8];
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
		xEcuInfo1 CanSendInfo1;
	};
	/*����PDO2�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8SendData2[8];
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
		xEcuInfo2 CanSendInfo2;
	};
	/*����PDO3�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8SendData3[8];
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
		xEcuInfo3 CanSendInfo3;
	};
	/*����PDO4�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8SendData4[8];
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
		xEcuInfo4 CanSendInfo4;
	};
	/*����PDO5�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8SendData5[8];
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
	};
	/*����PDO6�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8SendData6[8];
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
	};
	/*����PDO7�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8SendData7[8];
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
	};
	/*����PDO8�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8SendData8[8];
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
	};
	/*����PDO9�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8SendData9[8];
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
	};
	/*����PDO10�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8SendData10[8];
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
	};
	
}xCanSendPdoInfo;

#endif
