#ifndef _USER_CAN_HANGUO_DGC_CAN_ECU_
#define _USER_CAN_HANGUO_DGC_CAN_ECU_
#include "stdint.h"

/*xCan Send*/
typedef union
{
	uint8_t u8Data[8];
	struct
	{
		
		uint8_t u8MoveState;//ǰ��0������1��ֱ��4��ɲ��5��
		uint8_t u8ErrorMove;	//MCU����ת��
		uint8_t u8ErrorSteer; //ת������360[1]
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
		uint8_t u8MoveThrottle;
		uint8_t u8MoveSwiInput;
		uint8_t u8LiftThrottle;
		uint8_t u8DownThrottle;
		uint8_t u8LiftSwiInput;
		uint8_t u8QianHouYiThrottle;
		uint8_t u8QianHouQinThrottle;
		uint8_t u8ZuoYouYiThrottle;
	};
}xCanRev323Info;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t BatteryVoltageL;
		uint8_t BatteryVoltageH;
		uint8_t BatteryCurrentL;
		uint8_t BatteryCurrentH;
		uint8_t BatterySOC;
		uint8_t Reserved;
		uint8_t DischargeTimeL;
		uint8_t DischargeTimeH;
	};
}xCanRev2F4Info;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t BatterErrCode1: 2;
		uint8_t BatterErrCode2: 2;
		uint8_t BatterErrCode3: 2;
		uint8_t BatterErrCode4: 2;
		
		uint8_t BatterErrCode21: 2;
		uint8_t BatterErrCode22: 2;
		uint8_t BatterErrCode23: 2;
		uint8_t BatterErrCode24: 2;
		
		uint8_t BatterErrCode31: 2;
		uint8_t BatterErrCode32: 2;
		uint8_t BatterErrCode33: 2;
		uint8_t BatterErrCode34: 2;
		
		uint8_t BatterErrCode41: 2;
		uint8_t BatterErrCode42: 2;
		uint8_t BatterErrCode43: 2;
		uint8_t BatterErrCode44: 2;
		
		uint8_t Reserved1;
		uint8_t Reserved2;
		uint8_t Reserved3;
		uint8_t Reserved4;
	};
}xCanRev7F4Info;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint16_t ActualRPM;
		uint16_t KSI;
		uint16_t MotorCurrent;
		uint8_t MotorTemperature;
		uint8_t ControllerTemperature;
	};
}xCanSend1A3Info;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint16_t ThrottleOutput;
		uint16_t PumpVoltage;
		uint16_t PumpCurrent;
		uint8_t MotorTemperature;
		uint8_t ControllerTemperature;
	};
}xCanSend2A3Info;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t SWITCH;
		uint8_t Reserved;
		uint16_t POTWIFER1;
		uint16_t POTWIFER2;
		uint16_t POTWIFER3;
	};
}xCanSend3A3Info;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t FWD: 1;
		uint8_t REV: 1;
		uint8_t Reseve2: 1;
		uint8_t Batterytype: 1;
		uint8_t Tuttlespeed: 1;
		uint8_t Reseve5: 1;
		uint8_t Reseve6: 1;
		uint8_t Reseve7: 1;
			
		uint8_t BatterSoc;
		uint8_t Speed;
		uint8_t Odometer1;
		uint8_t Odometer2;
		uint8_t Odometer3;
		uint16_t Houemeter;
	};
}xCanSend380Info;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t ErrCodeMove;
		uint8_t ErrCodeProp;
		uint8_t ErrCodeStree;
		uint8_t ErrCodeBatter;
		uint8_t Reseve5;
		uint8_t Reseve6;
		uint8_t Reseve7;
		uint8_t Reseve8;
	};
}xCanSend390Info;


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
//		xCanRev323Info CanRevInfo323;
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
		xCanRev2F4Info CanRevInfo2F4;
	};
	/*����PDO6�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8RevData6[8];
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
		xCanRev7F4Info CanRevInfo7F4;
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
		xCanSend1A3Info CanSend1A3Info;
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
	};
	/*����PDO2�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8SendData2[8];
		xCanSend2A3Info CanSend2A3Info;
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
	};
	/*����PDO3�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8SendData3[8];
		xCanSend3A3Info CanSend3A3Info;
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
	};
	/*����PDO4�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8SendData4[8];
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
	};
	/*����PDO5�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8SendData5[8];
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
	xCanSend380Info CanSend380Info;
};
	/*����PDO6�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8SendData6[8];
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
		xCanSend390Info CanSend390Info;
	};
	/*����PDO7�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8SendData7[8];
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
		xCanSend260Info CanSend260Info;
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