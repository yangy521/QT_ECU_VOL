#ifndef _USER_CAN_AGV_MAXVISION_ECU_
#define _USER_CAN_AGV_MAXVISION_ECU_
#include "stdint.h"
typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint16_t	b1ServeOn: 1;
		uint16_t	b1PowerLineOn: 1;
		uint16_t	b1BrakeReq: 1;
		uint16_t	b1ForWardReq: 1;
		uint16_t	b1BackWardReq: 1;
		uint16_t	b1Driver2: 1;
		uint16_t	b1Driver3: 1;
		uint16_t	b1Driver4: 1;
		uint16_t	b1OutSaux4: 1;
		uint16_t	b1outHorn: 1;
		uint16_t	b1Free1: 1;
		uint16_t	b1Free2: 1;
		uint16_t	b1SpdModeReq: 1;
		uint16_t	b1PosiModeReq: 1;
		uint16_t	b1Free3: 1;
		uint16_t	b1Stuffing: 1;
		
		int16_t		i16TargetSpd;
		
		int16_t		i16TargetAngle;
		
		uint8_t		u8MaxTorqueMotoring;
		
		uint8_t		u8MaxTorqueBraking;
	};
}xCanRevInfo1;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t	u8PedalBrakeReq;
		uint8_t	u8DcPumpTarget;
		uint8_t	u8TargetEvp;
		uint8_t	u8TargetEvp1;
	};
}xCanRevInfo2;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint16_t b1ServoReady: 1;
		uint16_t b1OutMcStatus: 1;
		uint16_t b1OutEbStatus: 1;
		uint16_t b1OutAgvMode: 1;
		uint16_t b1InSwiD1: 1;
		uint16_t b1InSwiD2: 1;
		uint16_t b1InSwiD3: 1;
		uint16_t b1InSwiD4: 1;
		uint16_t b1InSwiD5: 1;
		uint16_t b1InSwiD6: 1;
		uint16_t b1InSwiD7: 1;
		uint16_t b1InSwiD8: 1;
		uint16_t b1SpdMode: 1;
		uint16_t b1PositionMode: 1;
		uint16_t b1Free: 1;
		uint16_t b1Stuffing: 1;
		
		int16_t i16ActualSpd;
		
		int16_t i16ActualSteerAngle;
		
		uint16_t u16AcCurrent;
		
	};
}xEcuInfo1;

typedef union
{
	uint8_t u8Data[8];
}xEcuInfo2;

typedef union
{
	uint8_t u8Data[8];
}xEcuInfo3;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8ErrCode;
		uint8_t u8MotorTemp;
		uint8_t u8BoardTemp;
		uint8_t	u8Soc;
		uint8_t	u8Ai1;
		uint8_t	u8Ai2;
		uint8_t u8Free;
		uint8_t u8SwVer;
	};
}xEcuInfo4;

typedef struct
{
	/*����PDO1�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8RevData1[8];
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
		xCanRevInfo1 CanRevInfo1;
	};
	/*����PDO2�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8RevData2[8];
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
		xCanRevInfo2 CanRevInfo2;
	};
	/*����PDO3�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8RevData3[8];
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
		//xCanRevInfo3 CanRevInfo3;
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