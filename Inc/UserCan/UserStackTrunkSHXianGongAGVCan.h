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
		uint8_t u8VCUAccelerateTime;	 //0-255 (0-25.5) ����ʱ��
		uint8_t u8VCUDecelerateTime;     //0-255 (0-25.5) ����ʱ��
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
		uint8_t u8MoveSpeedL;	//����ת��
		uint8_t u8MoveSpeedH;	//����ת��
		uint8_t u8Reserve1; 	//Ԥ��
		uint8_t u8Reserve2; 	//Ԥ��
		uint8_t u8ErrorCode;	//������	
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
		uint8_t u8ControlTempL;			//�������¶�  -1000 - 3000��-100 - 300��
		uint8_t u8ControlTempH;			//�������¶�  -1000 - 3000��-100 - 300��
		uint8_t u8ControlCurrentL; 		//����������	 0-10000 (0-1000)
		uint8_t u8ControlCurrentH; 		//����������	 0-10000 (0-1000)
		uint8_t u8BatterSOC;					//��ص���

		uint8_t b1ForwardLimit: 1;
		uint8_t b1BackwardLimit: 1;
		uint8_t b1UpLimit: 1;
		uint8_t b1DownLimit: 1;
		uint8_t b1LeftLimit: 1;
		uint8_t b1RightLimit: 1;
		uint8_t b1OpenLimit: 1;
		uint8_t b1CloseLimit: 1;	

		uint8_t u8MotorTempL;	      //����¶�  -1000 - 3000��-100 - 300��
		uint8_t u8MotorTempH;         //����¶�  -1000 - 3000��-100 - 300��
	};
}xCanSend283Info;


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
		xCanRev203Info CanRevInfo203;
	};
	/*����PDO6�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8RevData6[8];
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
		xCanRev303Info CanRevInfo303;
	};
	/*����PDO7�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8RevData7[8];
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
		xCanRev1A3Info CanRevInfo1A3;
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
	xCanSend183Info CanSend183Info;
	};  
	/*����PDO6�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8SendData6[8];
		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
	xCanSend283Info CanSend283Info;
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