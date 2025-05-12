/*******************************************************************************
* Filename: UserStackerTruckProc.h	                                           *
* Description: �߼����ͷ�ļ���������Ӧ�Ļص�����	   							   *
* Author: QExpand, HanCong                                                     *
* Date: 2023/12/07   														   *
* Revision:	 														 		   *
*******************************************************************************/

#ifndef _USER_CAN_SDSAIYISIBICHE_ECU_
#define _USER_CAN_SDSAIYISIBICHE_ECU_
#include "stdint.h"

typedef union
{
	uint8_t u8Data[8];
	struct
	{		
		uint8_t b1MotorInterLock: 1;
		uint8_t b6Reserve1: 6;
		uint8_t b1BrakeRequest: 1;
		
		uint8_t u8Reserve2;
		uint8_t u8MotorSpeedL;       //-32768-32768		(-100 - 100)
		uint8_t u8MotorSpeedH;       //-32768-32768		(-100 - 100)
		uint8_t u8MotorAcc;
		uint8_t u8MotorDec;
		uint8_t u8BoradCurrentLimit; //0-100 ��������������200A
		uint8_t u8Reserve3;

	};
}xCanRev22XInfo;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t b1PumpInterLock;
		uint8_t b7Reserve;
		
		uint8_t u8PumpCurrentLimit;
		uint8_t u8PumpSpeedL;         //(0-32767)  0-100
		uint8_t u8PumpSpeedH;         //(0-32767)  0-100
		uint8_t u32Reserve2;
		uint8_t u32Reserve3;
		uint8_t u32Reserve4;
		uint8_t u32Reserve5;
	};
}xCanRev32XInfo;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8Reserve1;
		uint8_t u8Reserve2;
		uint8_t u8Reserve3;
		uint8_t u8Reserve4;
		uint8_t u8Reserve5;
		uint8_t u8Reserve6;
		uint8_t u8Reserve7;
		uint8_t u8Reserve8;
	};
}xCanRev1AXInfo;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8Reserve1;
		uint8_t u8Reserve2;
		uint8_t u8Reserve3;
		uint8_t u8Reserve4;
		uint8_t u8Reserve5;
		uint8_t u8Reserve6;
		uint8_t u8Reserve7;
		uint8_t u8Reserve8;
	};
}xCanRev2AXInfo;
/**************************************
*����
***************************************/

typedef union
{
	uint8_t u8Datap[8];
	struct
	{
		uint8_t u8DiState1;
		uint8_t u8DiState2;
		uint8_t u8AiInput1L;
		uint8_t u8AiInput1H;
		uint8_t u8AiInput2L;
		uint8_t u8AiInput2H;
		uint8_t u8AiInput3;
		uint8_t u8Reserve;		
	};
}xCanSend1AXInfo;

typedef union
{
	uint8_t u8Datap[8];
	struct
	{
		uint8_t u8KsiVoltageL;
		uint8_t u8KsiVoltageH;
		uint8_t u8BatteryCurrentL;
		uint8_t u8BatteryCurrentH;
		uint8_t u8PumpCurrentL;
		uint8_t u8PumpCurrentH;
		uint8_t u8BatterySoc;
		uint8_t u8Errorcode;		
	};
}xCanSend2AXInfo;

typedef union
{
	uint8_t u8Datap[8];
	struct
	{
		uint8_t u8MotorCurrentL;
		uint8_t u8MotorCurrentH;
		uint8_t u8BoardTemperatureL;
		uint8_t u8BoardTemperatureH;
		uint8_t u8MotorSpeedL;
		uint8_t u8MotorSpeedH;
		uint8_t u8MotorTemperatureL;
		uint8_t u8MotorTemperatureH;		
	};
}xCanSend3AXInfo;

 typedef union
{
	uint8_t u8Datap[8];
	struct
	{
		uint8_t u8ErrorFlag;
		uint8_t u8AngleEcuL;
		uint8_t u8AngleEcuH;
		uint8_t u8VerSion;
		uint8_t u8CurrentThrottleL;
		uint8_t u8CurrentThrottleH;
		
		uint8_t u8DoState;
		
		uint8_t u8LockState;		
	};
}xCanSend4AXInfo;


typedef struct
{
	/*����PDO1�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8RevData1[8];
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
		xCanRev22XInfo CanRevInfo22X;
		//xCanRev1ACInfo CanRevInfo1;
	};
	/*����PDO2�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8RevData2[8];
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
		xCanRev32XInfo CanRevInfo32X;
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
		xCanSend1AXInfo   CanSend1AXInfo;
	};
	/*����PDO2�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8SendData2[8];
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
		xCanSend2AXInfo   CanSend2AXInfo;
	};
	/*����PDO3�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8SendData3[8];
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
		xCanSend3AXInfo   CanSend3AXInfo;
	};
	/*����PDO4�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8SendData4[8];
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
		xCanSend4AXInfo   CanSend4AXInfo;
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