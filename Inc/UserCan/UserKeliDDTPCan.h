#ifndef _USER_CAN_KELIDDTPC_CAN_ECU_
#define _USER_CAN_KELIDDTPC_CAN_ECU_
#include "stdint.h"


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
		
		uint8_t i16SteerAngleL;
		uint8_t i16SteerAngleH;
		
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
		uint8_t u8Reserve0;
		uint8_t u8Reserve1;
		uint8_t u8Reserve2;
		uint8_t u8Reserve3;
		uint8_t u8Reserve4;
		uint8_t	BMS_SOC;
		uint8_t	u8Reserve6;
		uint8_t	u8Reserve7;

	};
}xCanRev444Info;

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


typedef struct
{
	/*����PDO1�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8RevData1[8];
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
		//xCanRev1ACInfo CanRevInfo1;
	};
	/*����PDO2�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8RevData2[8];
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
		//xCanRev360Info CanRevInfo2;
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
		xCanRev360Info CanRevInfo360;
	};
	/*����PDO6�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8RevData6[8];
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
		xCanRev444Info CanRevInfo444;
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
	};
	/*����PDO2�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8SendData2[8];
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
	};
	/*����PDO3�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8SendData3[8];
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
	xCanSend260Info CanSend260Info;
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