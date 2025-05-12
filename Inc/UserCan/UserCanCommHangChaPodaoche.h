/*******************************************************************************
* Filename: UserStackerTruckProc.h	                                           *
* Description: �߼����ͷ�ļ���������Ӧ�Ļص�����	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/07/22    														   *
* Revision:	 														 		   *
*******************************************************************************/
#ifndef _USER_CAN_COMM_HC_PODAOCHE_H_
#define _USER_CAN_COMM_HC_PODAOCHE_H_

#include "stdint.h"
#include "Userdef.h"

#if (USER_TYPE == USER_HANGCHA_PODAOCHE)

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t b1Reserve1: 1;
		uint8_t b1EmsReq: 1;      //��������
		uint8_t b1SlowModeReq: 1; //����ģʽ
		uint8_t b1HornReq: 1;     //����
		uint8_t b1LiftReq: 1;     //����
		uint8_t b1DownReq: 1;     //�½�
		uint8_t b1LeanBackWardReq: 1; //����
		uint8_t b1LeanForWardReq: 1;  //ǰ��
		
				
		uint8_t b1RampModeReq: 1;   //�����µ�ģʽ
		uint8_t b1RampModeExit: 1;  //�˳��µ�ģʽ
		uint8_t b1Reserve2: 1;
		uint8_t b1Reserve3: 1;
		uint8_t b1Reserve4: 1;
		uint8_t b1Reserve5: 1;
		uint8_t b1Reserve6: 1;
		uint8_t b1Reserve7: 1;
		
		int16_t i16ThrottleValue;    //�ֱ�����ֵ -2048->2047
		
		uint8_t  b8Reserve9;
		
		uint8_t i8LiftDownValue;    //�����½�����ֵ
		
		uint8_t b8Reserve10;
		
		uint8_t  i8LeanValue;       //ǰ���������ֵ
	};
}xCanRev1E0Info;
typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t Xsign;
		uint8_t Xhigh;
		uint8_t Xlow1;
		uint8_t Xlow2;
		uint8_t Ysign;
		uint8_t Yhigh;
		uint8_t Ylow1;
		uint8_t Ylow2;
	};
}xCanRev18XInfo;
typedef union
{
	uint8_t u8Data[8];
	struct
	{
	 /*BMS Э��*/
		uint8_t u8VoltageLow;
		uint8_t u8VoltageHigh;
		uint8_t u8CurrentLow;
		uint8_t u8CurrentHigh;
		uint8_t u8Soc;
		uint8_t u8Volume;          //����
		
		uint8_t b1OverVotageErr:1;  //�����ѹ����#159
		uint8_t b1UnderVotageErr:1; //��������Ƿѹ#148
		uint8_t b1MissComErr:1;     //BMS﮵�ع���#148
		uint8_t b1SoloUnderVotageErr:1; //����Ƿѹ #158��ѹ����
		uint8_t b1OverCurrentErr:1;   //�ŵ����#148 ﮵�ع���
		uint8_t b1OverTmpErr:1;       //���ع��±��� #157
		uint8_t b1TmpProtectErr:1;    //�¶ȱ�����һ�㣩 #156������
		uint8_t b1ChargeFlag:1;       //���״̬
		
		uint8_t Reserve1;
		
	 
	};
}xCanRev2F0Info;
typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint16_t u16StateWord;
		int16_t s16SpeedMeasured;
		int16_t s16SteerAngle;
		uint16_t u16CurrentMeasured;
	};
}xEcuInfo1;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8ErrorCode;
		uint8_t u8MotorTmp;
		uint8_t u8BoardTmp;
		uint8_t u8BDIPercent;
		uint8_t u8Analog1;
		uint8_t u8Analog2;
		uint8_t u8Reserved;
		uint8_t u8SoftwareVer;
	};
}xEcuInfo2;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8ErrCode288;
		uint8_t u8Reserve2;
		uint8_t u8Reserve3;
		uint8_t u8Reserve4;
		uint8_t u8Reserve5;
		uint8_t u8Reserve6;
		uint8_t u8Reserve7;
		uint8_t u8Reserve8;
	};
}xCanRev288Info;
typedef union
{
	uint8_t u8Data[8];
	struct
	{
		uint8_t u8ErrCode289;
		uint8_t u8Reserve2;
		uint8_t u8Reserve3;
		uint8_t u8Reserve4;
		uint8_t u8Reserve5;
		uint8_t u8Reserve6;
		uint8_t u8Reserve7;
		uint8_t u8Reserve8;
	};
}xCanRev289Info;
/*    �Ǳ�Э��258 358  2F8  */
typedef union
{
	uint8_t u8Data[8];
	struct
	{
		/* byte 0 */
		uint8_t u8Base;//�������� Ĭ��10
		/* byte 1 */
		uint8_t b1BmsChoice0:1;
		uint8_t b1BmsChoice1:1;
		uint8_t b1TimeChoice0:1;
		uint8_t b1TimeChoice1:1;
		uint8_t b1LowBowerAlm:1;
		uint8_t b1PowerHourCount:1;
		uint8_t b1WorkHourCount:1;
		uint8_t b1FaultSymbol:1;
		/* byte 2 */
		uint8_t b1WorkMode0:1;
		uint8_t b1WorkMode1:1;
		uint8_t b1ShowSpeed:1;
		uint8_t b1SpeedUnit:1;
		uint8_t b1Reserve1:1;
		uint8_t b1NoLiftSymbol:1;
		uint8_t b1BrakeSymbol:1;
		uint8_t b1ChargeSymbol:1;
		/*  byte 3 */
		uint8_t u8ControlType;//����������
		/*  byte 4 */
		uint8_t u8ErrCode;    //�������
		/*  byte 5 */
		uint8_t u8Speed;      //����
		/*  byte 6 */        
		uint8_t u8BMS;        //����
		/*  byte 7 */
		uint8_t u8Reserve2;
	};
}xCanTx258Info;

typedef union
{
	uint8_t u8Data[8];
	struct
	{
		/* byte 0 */
		uint8_t u8Base;//������
		/* byte 1 */
		uint8_t u8HourCountL;
		uint8_t u8HourCountM;
		uint8_t u8HourCountH;
		
		uint8_t u8Reserve1;
		uint8_t u8Reserve2;
		uint8_t u8Reserve3;
		uint8_t u8Reserve4;
	};
}xCanTx2F8Info;
typedef struct
{
	/*����PDO1�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8RevData1[8];
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
		//xCanRev1E0Info CanRevInfo1;
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
		xCanRev1E0Info CanRevInfo5;
		
	};
	/*����PDO6�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8RevData6[8];
		xCanRev18XInfo CanRevInfo6;
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
		
	};
	/*����PDO7�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8RevData7[8];
		xCanRev18XInfo CanRevInfo7;
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
	};
	/*����PDO8�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8RevData8[8];
		xCanRev2F0Info CanRevInfoBMS;
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
	};	
	union
	{
		uint8_t u8RevData9[8];
		xCanRev288Info CanRevInfo288;
		/*�˴����Ҫ���յĽṹ�����ͣ� for example�� xCanRev1ACInfo CanRevInfo1*/
	};
	union
	{
		uint8_t u8RevData10[8];
		xCanRev289Info CanRevInfo289;
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
		xCanTx258Info CanHMISendInfo;

		/*�˴����Ҫ���͵Ľṹ�����ͣ� for example�� xCanSend23CInfo CanSend23CInfo*/
	};
	/*����PDO6�� �����ʵ������޸�����Ľṹ��*/
	union
	{
		uint8_t u8SendData6[8];
		xCanTx2F8Info Can2F8SendInfo;
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

#endif
