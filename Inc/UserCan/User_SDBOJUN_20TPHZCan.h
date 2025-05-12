#ifndef _USER_CAN_SDBOJUN_PHZ_CAN_ECU_
#define _USER_CAN_SDBOJUN_PHZ_CAN_ECU_
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
		uint8_t u8MoveStatePdo;
		uint8_t u8MoveErrPdo;
		uint8_t u8MotorValL;
		uint8_t u8MotorValH;
		uint8_t u8StreeValL;
		uint8_t u8StreeValH;
		uint8_t u8VehicleState;
		uint8_t u8SOC;
	};
}xCanSend1ACInfo;
/*xCan Rev*/
typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8MoveStatePdo;
		uint8_t u8MoveErrPdo;
		uint8_t u8MotorValL;
		uint8_t u8MotorValH;
		uint8_t u8StreeValL;
		uint8_t u8StreeValH;
		uint8_t u8VehicleState;
		uint8_t u8SOC;
	};
}xCanRev1ACInfo;
typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8MoveStatePdo;
		uint8_t u8MoveErrPdo;
		uint8_t u8MotorValL;
		uint8_t u8MotorValH;
		uint8_t u8StreeValL;
		uint8_t u8StreeValH;
		uint8_t u8VehicleState;
		uint8_t u8SOC;
	};
}xCanRev1ADInfo;
typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8MoveStatePdo;
		uint8_t u8MoveErrPdo;
		uint8_t u8MotorValL;
		uint8_t u8MotorValH;
		uint8_t u8StreeValL;
		uint8_t u8StreeValH;
		uint8_t u8VehicleState;
		uint8_t u8SOC;
	};
}xCanRev1AEInfo;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8REC1;
		uint8_t u8REC2;
		uint8_t u8REC3;
		uint8_t u8REC4;
		uint8_t u8REC5;
		uint8_t u8REC6;
		uint8_t u8SOC;
		uint8_t u8REC7;
	};
}xCanRev001Info;



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
		xCanRev1ACInfo CanRevInfo1AC;
	};
	/*����PDO6�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8RevData6[8];
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
		xCanRev1ADInfo CanRevInfo1AD;
	};
	/*����PDO7�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8RevData7[8];
				/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
		xCanRev1AEInfo CanRevInfo1AE;
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
		xCanSend1ACInfo CanSend1ACInfo;	
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