/*******************************************************************************
* Filename: UserStackerTruckProc.c	                                           *
* Description: �߼����Դ�ļ���������Ӧ�Ļص�����	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/05/30    														   *
* Revision:	 														 		   *
*******************************************************************************/
#include "UserCanComm.h"
#include "UserHangChaPodaocheProc.h"
#include "ErrCode.h"
#include "Eeprom.h"
#include "IQmathLib.h"
#include "PropProc.h"
#include "CanRevProc.h"
#include "string.h"
#include "Para.h"
#include "WdgProc.h"
#include "math.h"
#include "Log.h"
#include "LocalAi.h"
#include "LedProc.h"
#include "AiProc.h"
#include "LocalDo.h"
#include "HourCount.h"
#include "BatteryMeter.h"

#if (USER_TYPE == USER_HANGCHA_PODAOCHE)
const static xPdoParameter  sgPdoPara = 
{
	.TpdoPara = {
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 40},
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 100},
		{.u8Flag = 1, .u8Type = 0x00, .u16Period = 200},
		{.u8Flag = 1, .u8Type = 0x00, .u16Period = 200},
		
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 200, .u16CanId = 0x258},
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 200, .u16CanId = 0x2F8},
		{.u8Flag = 0, .u8Type = 0xFF, .u16Period = 200, .u16CanId = 0x358},
		{.u8Flag = 0, .u8Type = 0xFF, .u16Period = 200, .u16CanId = 0x113},
		{.u8Flag = 0, .u8Type = 0xFF, .u16Period = 200, .u16CanId = 0x114},
		{.u8Flag = 0, .u8Type = 0xFF, .u16Period = 200, .u16CanId = 0x115},
      
	},
	.RpdoPara = {
		{.b1Flag = 1, .b11CanRevId = 0x1E0},
		{.b1Flag = 1, .b11CanRevId = 0x185},
		{.b1Flag = 1, .b11CanRevId = 0x186},
		{.b1Flag = 1, .b11CanRevId = 0x2F0},
		{.b1Flag = 1, .b11CanRevId = 0x288},  //���� VCU������  30���� �����ߵ����ֹ����
		{.b1Flag = 1, .b11CanRevId = 0x289},
	},
};

#define	Gear1_Spd_Con1		0
#define	Gear1_Spd_Con2		1
#define	Gear1_Spd_Con3		2
#define	Gear1_Spd_Con4		3

#define	Gear2_Spd_Con1		0
#define	Gear2_Spd_Con2		1
#define	Gear2_Spd_Con3		2
#define	Gear2_Spd_Con4		3

#define	Gear3_Spd_Con1		0
#define	Gear3_Spd_Con2		1
#define	Gear3_Spd_Con3		2
#define	Gear3_Spd_Con4		3

#define	Gear4_Spd_Con1		0
#define	Gear4_Spd_Con2		1
#define	Gear4_Spd_Con3		2
#define	Gear4_Spd_Con4		3

#define	NoAct_ChargeSwi		0
#define	NoAct_Init				1
#define	NoAct_Lock				2
#define NoAct_OverPress   3     //��ӹ��ؽ�ֹ�˶���־λ
#define NoAct_LowPower    4     //�͵���
#define NoAct_SmoveErr    5     //�Ӷ��ֹ���
#define NoAct_BmsErr      6     //��ع���

#define NoMove_Init       0
#define	NoMove_Ems				1
#define NoMove_Lock       2
																	 
#define NoLiftUp_HeightLimit	0		
#define NoLiftUp_FaultLock		1
#define NoliftUp_Logic        2

#define NoLiftDown_LogicErr   1

#define NoLean_Init           0
#define NoLean_Lock           1
#define NoLean_LogicErr       2

#define NoPump_Init           0
#define NoPump_Lock           1

#define LimitSpeed_LowPower   0
#define LimitSpeed_TmpProtect 1

#define HmiStep_Base         0
#define HmiStep_HourWrite    1
#define HmiStep_WorkCount    2
#define HmiStep_Set          3
#define HmiStep_HourCount    4

#define Plain        0   //ͨ�ÿ���������
#define MainTraction 1   //��ǣ������������
#define SubTraction  2   //��ǣ������������


typedef union
{
	uint16_t u16data;
	struct
	{
		uint16_t b1Ems: 1;
		uint16_t b1Lock: 1;
		uint16_t b1Forward: 1;
		uint16_t b1Backward: 1;
		uint16_t b1LiftDown: 1;
		uint16_t b1LiftUp: 1;
		uint16_t b1LeanForward: 1;
		uint16_t b1LeanBackward: 1;
		uint16_t b1LiftLimit: 1;
		uint16_t b1SlowMode: 1;
		uint16_t b1Charge: 1;
		uint16_t b1Horn: 1;
		uint16_t b1HeightLimit: 1;
		uint16_t b1Reserve: 3;
	};
}xSwiInput;

typedef union
{
	uint8_t u8Data[20];
	struct
	{
		uint8_t b1NoActFlag: 1;
		uint8_t b1MoveFlag: 1;
		uint8_t b1LiftUpFlag: 1;
		uint8_t b1Gear1SpdFlag: 1;
		uint8_t b1Gear2SpdFlag: 1;
		uint8_t b1Gear3SpdFlag: 1;
		uint8_t b1Gear4SpdFlag: 1;		/*bms �͵���*/
		uint8_t b1PumpMotorNoAct: 1;
		
		uint8_t b1ForWardStat: 1;
		uint8_t b1BackWardStat: 1;
		uint8_t b1LiftUpStat: 1;
		uint8_t b1LeanForWardStat: 1;
		uint8_t b1LeanBackWardStat: 1;
		uint8_t b1LiftDownStat: 1;
		uint8_t b1SlowModeStat: 1;
		uint8_t b1HornStat: 1;
		
		uint8_t	b4Gear1Spd: 4;
		uint8_t	b4Gear2Spd: 4;
		uint8_t	b4Gear3Spd: 4;
		uint8_t	b4Gear4Spd: 4;
		
		uint8_t u8NoAct;
		
	  uint8_t b4NoPump:4; //��ֹ����Һѹ����
		uint8_t b4NoLean:4; //��ֹ������б����

		uint8_t b4NoLiftUp: 4; //��ֹ����
		uint8_t b4NoMove: 4;   //��ֹ����
		
		uint8_t b4NoLiftDown:4;//��ֹ�½�
		uint8_t b1EnterRampFlag:1;    //��������״̬
		uint8_t b3DirectionSave:3;    //����Ƕȴ���ʱ�ķ��� 1���� 2����
		
		uint8_t b1AutoRampModeFlag: 1;
		uint8_t b1RampModeFlag: 1;
		uint8_t b1CantExitRampMode:1;
		uint8_t b1AutoRampState: 1;
		uint8_t b1ManuRampState: 1;
		uint8_t b1EmsState: 1;
		uint8_t b1LimtSpeedFlag:1;
		uint8_t b1ChargeFlag:1;
		
		uint8_t u8AngleState;
		uint8_t u8Soc;
		uint8_t u8ErrCode288;
		uint8_t u8ErrCode289;
		uint8_t u8ErrCodeMCU;
		uint8_t	u8LimitSpeed;
		
		uint8_t b1HMI_LowPowerFlag:1;//�Ǳ�͵�������
		uint8_t b1HMI_ChargeFlag:1;//�Ǳ������
		uint8_t b1HMI_FaultFlag:1;//�Ǳ���ϱ�־λ
		uint8_t b1HMI_BrakeFlag:1;//�Ǳ��ƶ���־λ
		
		uint8_t b1LeanFlag:1;
		uint8_t b1LiftDownFlag:1;
		uint8_t b1PreLeanFlag:1; //�Ѿ�Ԥ��б�����жϱ�־
		uint8_t b1Reserve2:1;
		
		int16_t s16TargetAngle;
		int16_t s16RampAngle;
		int16_t s16SaveAngle;
	};
}xValvesInfo;

typedef union
{
	uint8_t u8data[2];
	struct
	{
		/*��ֹ�߼�*/
		uint8_t NoActFlag:1;
		uint8_t NoMoveFlag:1;
		uint8_t NoLeanFlag:1;
		uint8_t NoUpFlag:1;
		uint8_t NoDownFlag:1;
		uint8_t NoPumpFlag:1;
		uint8_t OverLean:1;
		uint8_t DirectionErr:1;  //186�������Ƕȴ���
		
		/*�ٶȵ�λ*/
		uint8_t LowPowFlag:1; 	//�͵����ٶ�
		uint8_t UpslopeSpd:1;		//�����ٶ�
		uint8_t DownslopeSpd:1;	//�����ٶ�
		uint8_t TuttleSpd:1;		//����
		uint8_t ReverseSpd:1;  	//������ٶ�
		/**/
		uint8_t NoForWordFlag:1;  //��ֹǰ��
		uint8_t NoBackWordFlag:1; //��ֹ����
		uint8_t ChargeFlag:1;     //�����~

	};
	
}xLimit;
typedef struct
{
	uint8_t	u8Gear1Spd;
	uint8_t	u8Gear2Spd;
	uint8_t	u8Gear3Spd;
	uint8_t	u8Gear4Spd;
	
	float		fLiftSpdPer5msAccStep;
	float		fDownSpdPer5msAccStep;
	float		fMoveSpdPer5msAccStep;
	
	float		fLiftSpdPer5msDecStep;
	float		fDownSpdPer5msDecStep;
	float		fMoveSpdPer5msDecStep; 
	
	float		fPropMinCurrent;
	float		fPropMaxCurrent;

	uint8_t		u8LeanBackWardValue;
	uint8_t		u8LeanForWardValue;
 
	
	uint8_t		b1LiftMode: 1;
	uint8_t		b1HourCntClr: 1;
	uint8_t   b1RampMode: 1;         //�µ�ģʽ��־λ
	uint8_t		b1AngleSensorEn: 1;	   //����ת�Ǵ�����ʹ��
	uint8_t   b1TileSensorEn:1;      //��Ǵ�����
	uint8_t   b1AngleAiLimit:1;       //��б�Ƕ�ģ����λʹ��
	uint8_t   b1InterLockEN:1;        //�Ӷ��������ֻ���ʹ��
	
	uint8_t		u8BatteryType;
	uint8_t   u8HourCount;            //ʱ��ѡ��
	
	uint8_t		b1HourConutMode: 1;		/*0: �ϵ��ʱ�� 1��������ʱ*/
	uint8_t		b1StartUpLock: 1;		/*0�� ��⣬ 1�������*/
	uint8_t		b1LiftLock: 1;	    /*0�� ����Ҫ������ 1���ò���Ҫ����*/
	uint8_t   b1SpeedUnit:1;      /*ת�ٵ�λ*/
	uint8_t   b1Language:1;       /*����ѡ��*/
	uint8_t   b1HmiVersion:1;     /*�Ǳ�汾*/
	uint8_t   b1CalFun:1;         /*ʹ��ѹ��������*/
	uint8_t   b1TurnDecSpdEn:1;   /*ת�併��ʹ��*/
	
	uint8_t  b1TurnDifSpdEn:1; 		//���ٹ��ܿ����ر�
	uint8_t  b7Reserve7:7;
	
	
	uint8_t  u8FastSpd;  //������ʻ�ٶ�
	uint8_t  u8SlowSpd;  //������ʻ�ٶ�
	uint8_t  u8RampLowestAngle;  //�µ���С�Ƕ� ���ж��Ƿ�Ϊ����״̬��
	uint8_t  u8RampHighestAngle; 
	
	
	uint8_t u8LowPow1Step;  //�͵���һ������
	uint8_t u8LowPow2Step;  //�͵�����������
	
	int16_t   s16SteerAngle;
  int16_t   s16ForwardLimitAngle;       //ǰ�����λ
	int16_t   s16BackwardLimitAngle;      //�������λ
	int16_t   s16AutoRampAngle;           //�Զ��µ���
	int16_t   s16SetExitAngle;            //�趨�˳���
  int16_t   s16SetMaxtileAngle;	       //�趨���Զ���ƽ״̬����������Ƕ�
	int16_t   s16MaxLeanAngle;            //���������µ��Ƕ�
	int16_t   s16PreleanAngle;            //�ֶ��µ�ģʽ��һ�ν���Ԥ��б�Ƕ�
	int16_t   s16TargetRampAngle;         //�µ�ģʽ��Ҫ���ڵ���Ŀ��Ƕ�
	
	uint16_t  u16TurnAngleMax;           //ת������ֵ
	uint16_t  u16TurnAngleMin;           //ת�����Сֵ
	uint16_t  u16TurnAngleMiddle;        //ת����м�ֵ
	uint16_t  u16TurnAngleDeadZoon;       //����
	uint16_t	u16RatioOfTransmission;
	uint16_t	u16MotorMaxSpd;
	uint16_t 	u16MaxPresure;            //���ѹ��ֵ
	
 xSteerAngleDecSpd SteerAngleDecSpd;	
}xUserInfo; 

typedef union   //#115 UserSet���ñõ�������߼�
{
	uint8_t u8Data;
	struct
	{
		uint8_t  b1PumpLockEn:1;    
		uint8_t  b1TileLockEn:1;
		uint8_t  b4Reserve:6;
	};	
}xUserSet;

typedef union
{
	uint8_t u8Data;
	struct
	{
		uint8_t b1ServoOn: 1;
		uint8_t b1PowerLineOn: 1;
		uint8_t b1BrakeReq: 1;
		uint8_t b1ForwardReq: 1;
		uint8_t b1BackwardReq: 1;
		uint8_t b1LiftReq: 1;
		uint8_t b1DownReq: 1;
		uint8_t b1EmsReq: 1; 
	};
}xMstSendStat;

typedef union
{
	uint16_t u16Data;
	struct
	{
		uint16_t b1HeightLimit: 1;
		uint16_t b15Reserve: 15;
	};
}xSaveStateInfo;
/*������ת����*/
static uint8_t ErrCodeTransTable[] = 
{
	0x40,0x40,0x40,0x40,0x41,0x42,0x43,0x44,0x45,0x46, //89->99
	0x52,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5A,0x5B, //100->110
	0x5C,0x5D,0x5E,0x5F,0x61,0x62,0x63,0x64,0x65,0x66, //111->120
};
static xUserSet  sgUserSet;
static xSaveStateInfo sgSaveState;
static xLimit sgLimitInfo;
static int16_t	i16MotorSpd = 0;
static uint16_t u16MotorVal = 0;       
static uint8_t	u8PumpOrPropValue = 0;
static xValvesInfo sgValvesInfo;
static xUserInfo sgUserInfo;
static xSwiInput sgSwiInput;
static uint32_t u32HourCount = 0;
static uint32_t u32WorkCount = 0;
static uint32_t Workcnt = 0;
//static uint16_t RealSpeed = 0;
/*PDO���մ�����*/
xCanRevPdoInfo gCanRevPdoInfo; 
				

/*PDO���ʹ�����*/
xCanSendPdoInfo gCanSendPdoInfo;
//{
//	.CanSendInfo1.b1OutAgvMode = 1,
//	.CanSendInfo4.u8SwVer = 0x0,
//};
 
static void vMstRevProc(xMstRevPara *RevData)
{	
	if(0 != RevData->b1MainDriver)
	{
		
	}
	else
	{
		
	}
	
	if(0 != RevData->b1Ebrake)
	{
		
	}
	else
	{	
		
	}
	
	if(0 != RevData->b1Driver3State)
	{
		
	}
	else
	{
		
	}

	/*�����봦��*/
	if(0 != RevData->u8ErrCode)
	{
		if (u8ErrCodeGet() < RevData->u8ErrCode)   /*lilu, 20230818, ���MCU֮ǰ��С�Ĺ�����*/
		{
			uint8_t i = 0;
			for (i=0; i<RevData->u8ErrCode; i++)
			{
				i32ErrCodeClr(i);
			}
			
		}
		i32ErrCodeSet(RevData->u8ErrCode - 1);
		sgValvesInfo.u8ErrCodeMCU = RevData->u8ErrCode; 
	}
	else
	{
		if (u8ErrCodeGet() < 50)				/*lilu, 20230817, û��MCU�Ĺ��ϵ�ʱ�򣬾����MCU�����й�����*/
		{
			uint8_t i = 0;
			for (i=0; i<50; i++)
			{
				i32ErrCodeClr(i);
			}
			sgValvesInfo.u8ErrCodeMCU = 0;
		}
	}

	i16MotorSpd = (uint16_t)(RevData->u8SpeedFdbHigh << 8) | RevData->u8SpeedFdbLow ;
////	i32LogWrite(DEBUG, LOG_USER, "Motor Rev Spd = %d\r\n", i16MotorSpd);
//	__disable_irq();
//	RealSpeed = i16MotorSpd / sgUserInfo.u16RatioOfTransmission;
	gCanSendPdoInfo.CanSendInfo1.s16SpeedMeasured = (RevData->u8SpeedFdbHigh << 8) | RevData->u8SpeedFdbLow;
	//����ת��
																							
	gCanSendPdoInfo.CanSendInfo1.u16CurrentMeasured = (RevData->u8CurrentHigh << 8) | RevData->u8CurrentLow;
//	__enable_irq();

}

/*�����Ǳ�ת����Ĵ�����*/
 static uint8_t u8GetTrasErrCode(void)
 {
	 uint8_t ErrCodeTrans;
	 uint8_t Idx;
	 if(u8ErrCodeGet() >= ErrCode89)
	 {
		Idx = (u8ErrCodeGet() - ErrCode89); //��89�Ŵ���ʼΪ�Զ������;
		ErrCodeTrans = ErrCodeTransTable[Idx];
	 }
	 else
	 {
		 ErrCodeTrans  = u8ErrCodeGet();
	 }
	 return ErrCodeTrans;
 };
/*�Ǳ��ͽṹ���ʼ��*/
static void vHmiSendFun(void)
{
	static uint8_t SendHmiStep = HmiStep_Base;
	static uint8_t SendHmiCount = 0;
	for(int i =0; i<8;i++)
	{
		gCanSendPdoInfo.CanHMISendInfo.u8Data[i] = 0;
	}
	SendHmiCount++;
	if(SendHmiCount % 9 == 0)
		SendHmiStep = HmiStep_Set;
//	else if(SendHmiCount % 8 == 0)
//		SendHmiStep = HmiStep_HourWrite;
	else if(SendHmiCount % 7 == 0)
		SendHmiStep = HmiStep_WorkCount;
	else if(SendHmiCount % 6 == 0)
		SendHmiStep = HmiStep_HourCount;
	else
		SendHmiStep = HmiStep_Base;
	switch(SendHmiStep)
	{
		case HmiStep_Base:
			gCanSendPdoInfo.CanHMISendInfo.u8Base = 0X10;
			gCanSendPdoInfo.CanHMISendInfo.b1PowerHourCount = 1;
			gCanSendPdoInfo.CanHMISendInfo.b1WorkHourCount = 1;
		
			gCanSendPdoInfo.CanHMISendInfo.b1BmsChoice0 = 1;  // 00-���ص���  01-CAN���ߵ���1  10-CAN���ߵ���2  11-Ԥ��
			gCanSendPdoInfo.CanHMISendInfo.b1BmsChoice1 = 0;
			/*��ʾ����*/
			gCanSendPdoInfo.CanHMISendInfo.u8BMS = sgValvesInfo.u8Soc;
			
			gCanSendPdoInfo.CanHMISendInfo.b1TimeChoice0 = 1;// 00-���ؼ�ʱ 01-CAN����ʱ��1 10-CAN����ʱ��2 11-������ʾ
			gCanSendPdoInfo.CanHMISendInfo.b1TimeChoice1 = 0;
		
			gCanSendPdoInfo.CanHMISendInfo.b1SpeedUnit = sgUserInfo.b1SpeedUnit;    //�ٶȵ�λΪKm/h
			if(i16MotorSpd != 0)
			{
				gCanSendPdoInfo.CanHMISendInfo.b1BrakeSymbol = 0;
				gCanSendPdoInfo.CanHMISendInfo.b1ShowSpeed = 1;	
			}
			else
			{
				gCanSendPdoInfo.CanHMISendInfo.b1BrakeSymbol = 1;
				gCanSendPdoInfo.CanHMISendInfo.b1ShowSpeed = 0;
			}
			if(gCanSendPdoInfo.CanHMISendInfo.b1SpeedUnit)
				gCanSendPdoInfo.CanHMISendInfo.u8Speed = (i16MotorSpd / sgUserInfo.u16RatioOfTransmission) * 0.6213;
			else				
				gCanSendPdoInfo.CanHMISendInfo.u8Speed = (i16MotorSpd / sgUserInfo.u16RatioOfTransmission);		
			//�͵�����ʾ
			if(sgValvesInfo.b1HMI_LowPowerFlag) 
				gCanSendPdoInfo.CanHMISendInfo.b1LowBowerAlm = 1;
			else
				gCanSendPdoInfo.CanHMISendInfo.b1LowBowerAlm = 0;
			//��������ʾ
			if(sgValvesInfo.b1HMI_ChargeFlag) 
				gCanSendPdoInfo.CanHMISendInfo.b1ChargeSymbol = 1;
			else
				gCanSendPdoInfo.CanHMISendInfo.b1ChargeSymbol = 0;
			/*����ģʽ*/
			if(sgLimitInfo.TuttleSpd == 1)
			{
				gCanSendPdoInfo.CanHMISendInfo.b1WorkMode0 = 0;
				gCanSendPdoInfo.CanHMISendInfo.b1WorkMode1 = 1;
			}
			else if(sgValvesInfo.b1RampModeFlag)    //�µ�ģʽΪģʽ2
			{
				gCanSendPdoInfo.CanHMISendInfo.b1WorkMode0 = 1;
				gCanSendPdoInfo.CanHMISendInfo.b1WorkMode1 = 0;
			}
			else
			{				
				gCanSendPdoInfo.CanHMISendInfo.b1WorkMode0 = 0;
				gCanSendPdoInfo.CanHMISendInfo.b1WorkMode1 = 0;
			}			
			 /*��ֹ����*/
			if(sgValvesInfo.b4NoLiftUp)
				gCanSendPdoInfo.CanHMISendInfo.b1NoLiftSymbol = 1;
			else 
				gCanSendPdoInfo.CanHMISendInfo.b1NoLiftSymbol = 0;
			/*��ʾ������ż����ϴ���*/
			if((sgValvesInfo.u8ErrCode288 != 0)||(sgValvesInfo.u8ErrCode289 != 0))
			{
				uint8_t tmp = sgValvesInfo.u8ErrCode288;
				if(tmp == 0)
					tmp = sgValvesInfo.u8ErrCode289;
				gCanSendPdoInfo.CanHMISendInfo.u8ControlType = SubTraction;
				gCanSendPdoInfo.CanHMISendInfo.u8ErrCode = tmp;
				gCanSendPdoInfo.CanHMISendInfo.b1FaultSymbol = 1;
			}
			else if(sgValvesInfo.u8ErrCodeMCU != 0)
			{
				gCanSendPdoInfo.CanHMISendInfo.u8ControlType = MainTraction;
				gCanSendPdoInfo.CanHMISendInfo.u8ErrCode = sgValvesInfo.u8ErrCodeMCU;
				gCanSendPdoInfo.CanHMISendInfo.b1FaultSymbol = 1;
			}
			else if(u8ErrCodeGet() > 50)
			{
				gCanSendPdoInfo.CanHMISendInfo.u8ControlType = Plain;
				gCanSendPdoInfo.CanHMISendInfo.u8ErrCode = u8ErrCodeGet();
				gCanSendPdoInfo.CanHMISendInfo.b1FaultSymbol = 1;
			}
			else
			{
				gCanSendPdoInfo.CanHMISendInfo.u8ControlType = Plain;
				gCanSendPdoInfo.CanHMISendInfo.u8ErrCode = 0;
				gCanSendPdoInfo.CanHMISendInfo.b1FaultSymbol = 0;
			}
			break;
				
		case HmiStep_HourWrite:
			gCanSendPdoInfo.CanHMISendInfo.u8Base = 0X20;
			break;
		
		case HmiStep_WorkCount:
			gCanSendPdoInfo.CanHMISendInfo.u8Base = 0x21;
			gCanSendPdoInfo.CanHMISendInfo.u8Data[1] = (u32WorkCount%100) & 0xFF;
			gCanSendPdoInfo.CanHMISendInfo.u8Data[2] = ((u32WorkCount/100) >> 8) & 0xFF;
			gCanSendPdoInfo.CanHMISendInfo.u8Data[3] = ((u32WorkCount/100) >> 16) & 0xFF;
			break;
		
		case HmiStep_Set:
			gCanSendPdoInfo.CanHMISendInfo.u8Base = 0x02;
			gCanSendPdoInfo.CanHMISendInfo.b1BmsChoice0 = 1;
			gCanSendPdoInfo.CanHMISendInfo.b1BmsChoice1 = 0;
			gCanSendPdoInfo.CanHMISendInfo.b1TimeChoice0= 0; 
//			if(sgUserInfo.b1Language)
//			{
//				gCanSendPdoInfo.CanHMISendInfo.b1BmsChoice1 = 1;
//				gCanSendPdoInfo.CanHMISendInfo.b1TimeChoice0= 0; 
//			}
//			else
//			{
//				gCanSendPdoInfo.CanHMISendInfo.b1BmsChoice1 = 0;
//				gCanSendPdoInfo.CanHMISendInfo.b1TimeChoice0= 1; 
//			}
			if(sgUserInfo.b1HmiVersion)
				gCanSendPdoInfo.CanHMISendInfo.u8Data[2] = 0x02;
			else
				gCanSendPdoInfo.CanHMISendInfo.u8Data[2] = 0x01;
			
			gCanSendPdoInfo.CanHMISendInfo.u8Data[3] = 0x01;
			gCanSendPdoInfo.CanHMISendInfo.u8Data[4] = SOFTWARE_VERSION_CODE & 0xFF;
			gCanSendPdoInfo.CanHMISendInfo.u8Data[5] = (SOFTWARE_VERSION_CODE >> 8) & 0xFF;
			gCanSendPdoInfo.CanHMISendInfo.u8Data[6] = 20; //�͵���������ֵ
			gCanSendPdoInfo.CanHMISendInfo.u8Data[7] = 10; //�͵����ж�
			break;
			
		case HmiStep_HourCount:
			gCanSendPdoInfo.Can2F8SendInfo.u8Base = 0x03;	
			gCanSendPdoInfo.Can2F8SendInfo.u8HourCountL = (u32HourCount%100) & (0xFF);
			gCanSendPdoInfo.Can2F8SendInfo.u8HourCountM = ((u32HourCount/100) >> 8) & 0xFF;
			gCanSendPdoInfo.Can2F8SendInfo.u8HourCountH = ((u32HourCount/100) >> 16) & 0xFF;
			break;	
	}
};

/*add Turn with Dec Spd*/
static uint16_t	u16SteerAngleDecSpd(uint16_t u16Spd, uint16_t u16Angle, const xSteerAngleDecSpd *Info)
{
	uint16_t u16Res = 0;
	uint16_t u16Tmp = 0;
	
	if (u16Angle <= Info->u16StartAngle)
	{
		u16Res = u16Spd;
	}
	else if (u16Angle >= Info->u16EndAngle)
	{
		if (u16Spd >= Info->u16EndAngleSpd)
		{
			u16Res = Info->u16EndAngleSpd;
		}
		else
		{
			u16Res = u16Spd;
		}
	}
	else
	{
		u16Tmp = Info->u16StartAngleSpd - (u16Angle - Info->u16StartAngle) * Info->fAgnleDecSpdFactor;
		if (u16Spd > u16Tmp)
		{
			u16Res = u16Tmp;
		}
		else
		{
			u16Res = u16Spd;
		}
	}
	
	return u16Res;
}
static void vMoveModeNoChangeProc(int16_t *i16Spd, const xValvesInfo *pValvesInfo)
{
	float fTmp = 0;
	if ((1 == pValvesInfo->b1ForWardStat) || (1 == pValvesInfo->b1BackWardStat))
	{
		if (*i16Spd > u16MotorVal)
		{
			fTmp = sgUserInfo.fMoveSpdPer5msDecStep * (0.01 + 0.99 * *i16Spd / MOTOR_SPEED_RANGE);
			if ((((*i16Spd - u16MotorVal) > fTmp) && (0 != sgUserInfo.fMoveSpdPer5msDecStep)))
			{
				*i16Spd -= fTmp;
			}
			else
			{
				*i16Spd = u16MotorVal;
			}
		}
		else if (*i16Spd < u16MotorVal)
		{
			fTmp = sgUserInfo.fMoveSpdPer5msAccStep * (0.01 + 0.99 * *i16Spd / MOTOR_SPEED_RANGE);
			if (((u16MotorVal - *i16Spd) > fTmp) && (0 != sgUserInfo.fMoveSpdPer5msAccStep))
			{
				*i16Spd += fTmp;
			}
			else
			{
				*i16Spd = u16MotorVal;
			}
		}
	}
	else
	{
		*i16Spd = 0;
	}
}

/*���͸�Mst�Ļص�����*/
static void vMstSendProc(xMstSendPara *SendData) 
{
	static xValvesInfo sgLastValvesInfo;
	static uint8_t u8MoveSwitchFlag = 0;
	float fTmp = 0.0;
	xMstSendStat LastStatus;
	int16_t i16Spd = 0;
		
	LastStatus.u8Data = SendData->buf[2];
	SendData->buf[2] = 0;

	i16Spd = (SendData->u8TargetHigh << 8) | SendData->u8TargetLow;
	i16Spd = abs(i16Spd);
	
	if(0 != sgUserInfo.b1TurnDecSpdEn)
	{
		i16Spd = u16SteerAngleDecSpd(u16MotorVal,abs(sgUserInfo.s16SteerAngle),&sgUserInfo.SteerAngleDecSpd);//ת�併�ٹ���
	}
	/*Move Mode*/
	if ((sgLastValvesInfo.b1ForWardStat != sgValvesInfo.b1ForWardStat) || 
		(sgLastValvesInfo.b1BackWardStat != sgValvesInfo.b1BackWardStat))
	{
		if ((0 == i16Spd) ||
		   ((1 == LastStatus.b1BackwardReq) && (1 == sgValvesInfo.b1ForWardStat)) || 
		   ((1 == LastStatus.b1ForwardReq) && (1 == sgValvesInfo.b1BackWardStat)))
		{
			sgLastValvesInfo.b1ForWardStat = sgValvesInfo.b1ForWardStat;
			sgLastValvesInfo.b1BackWardStat = sgValvesInfo.b1BackWardStat;
		}
		else 
		{
			if (((1 == sgLastValvesInfo.b1ForWardStat) && (0 == sgValvesInfo.b1ForWardStat)) ||
				((1 == sgLastValvesInfo.b1BackWardStat) && (0 == sgValvesInfo.b1BackWardStat)))
			{
				if (0 == u8MoveSwitchFlag)
				{
					uint16_t u16Tmp = 0;
					if (0 != sgUserInfo.u16MotorMaxSpd)
					{
						u16Tmp = abs(i16MotorSpd) * MOTOR_SPEED_RANGE / sgUserInfo.u16MotorMaxSpd;	/*CloseLoop */
					}
					else
					{
						u16Tmp = abs(i16MotorSpd) * MOTOR_SPEED_RANGE / MOTOR_MAX_SPEED;	/*CloseLoop */
					}
					if (u16Tmp < i16Spd)
					{                                                                                
						i16Spd = u16Tmp;
					}
					u8MoveSwitchFlag = 1;
				}
				fTmp = sgUserInfo.fMoveSpdPer5msDecStep * (0.01 + 0.99 * i16Spd / MOTOR_SPEED_RANGE);
				if ((abs(i16MotorSpd) > MOTOR_MIN_SPEED) && (abs(i16Spd) > (uint16_t)(fTmp + 1)))
				{
					i16Spd -= fTmp;
				}
				else
				{
					i16Spd = 0;
				}
			}
		}
	}
	else
	{
		vMoveModeNoChangeProc(&i16Spd, &sgLastValvesInfo);
		u8MoveSwitchFlag = 0;
	}
	
	SendData->b1ServoOn = 1;

	if(sgValvesInfo.b1BackWardStat == 1)
	{
		if(sgValvesInfo.b1EmsState == 1)
		{
			 i32ErrCodeSet(MOVE_EMS_ERR);
		}
		else
		{
			SendData->b1BackwardReq = 1;
			i16Spd *= -1;
		}
	}
	else if(sgValvesInfo.b1ForWardStat == 1)
	{
		SendData->b1ForwardReq = 1;
	}
	SendData->u8TargetHigh = i16Spd >> 8;   //0-4096
	SendData->u8TargetLow = i16Spd;

 if(sgUserInfo.b1TileSensorEn == 1)
 {
	 /*                         �µ�ģʽ�Զ���ƽ                                */
	if((sgValvesInfo.b1AutoRampModeFlag == 1)||(sgValvesInfo.b1RampModeFlag == 1))
	{
		sgUserInfo.b1RampMode = 1;
		 //�ڵ������ʱ���ܶ����ͱõ��
		if((sgValvesInfo.b1RampModeFlag == 1)&&(0 == sgValvesInfo.b1PreLeanFlag))//������ֶ��µ�ģʽ�ȵ�һ��Ԥ��б���ٵ�ƽ
		{      
			if((sgValvesInfo.s16TargetAngle - sgValvesInfo.s16SaveAngle) < sgUserInfo.s16PreleanAngle)
			{
				sgValvesInfo.b1LeanBackWardStat  = 1;                                          
				u8PumpOrPropValue  = 30;
			}
			else
			{
				sgValvesInfo.b1PreLeanFlag = 1;
			}
		}
		else
		{
			if(gCanSendPdoInfo.CanSendInfo1.s16SpeedMeasured > 400)
			{
				if( sgValvesInfo.s16TargetAngle < sgUserInfo.s16TargetRampAngle)         
				{ 
					sgValvesInfo.b1LeanBackWardStat  = 1;                                          
					u8PumpOrPropValue = (((sgUserInfo.s16TargetRampAngle - sgValvesInfo.s16RampAngle) * (PUMP_MAX_VALUE - 21) ) / (sgUserInfo.s16MaxLeanAngle * 3)) + 21 ;  //�ͱõ��ֵ 21~85 ���P�����㷨			
				}
				else if(sgValvesInfo.s16TargetAngle > sgUserInfo.s16TargetRampAngle)
				{
					sgValvesInfo.b1LeanForWardStat  = 1;                                          
					u8PumpOrPropValue = (((sgUserInfo.s16TargetRampAngle - sgUserInfo.s16PreleanAngle) * (PUMP_MAX_VALUE - 21) ) / (sgUserInfo.s16MaxLeanAngle * 3)) + 21 ;  //�ͱõ��ֵ 21~85 ���P�����㷨	
				}
				else
				{
					sgValvesInfo.b1LeanBackWardStat  = 0;
					sgValvesInfo.b1LeanForWardStat = 0;
					u8PumpOrPropValue = 0;
				}
			}
			else
			{
				sgValvesInfo.b1LeanBackWardStat  = 0;
				sgValvesInfo.b1LeanForWardStat = 0;
				u8PumpOrPropValue = 0;
			} 	
		}
	}		
	 /*                 �˳��µ�ģʽǰ�㵽ǰ�����λ                         */
	if((sgUserInfo.b1RampMode == 1) && ((sgValvesInfo.b1RampModeFlag == 0) && (sgValvesInfo.b1AutoRampModeFlag == 0)))
	{
		if(sgUserInfo.s16ForwardLimitAngle < i32LocalAiGetValue(ANG_SEN_INPUT))
		{
			sgValvesInfo.b1LeanForWardStat  = 1;
			u8PumpOrPropValue = 30 ;
		}
		//�˴�Ϊ�Ƿ��ͱõ��ǰ�㵽��λ�Ƕ��ж�  
		else 
		{
			sgUserInfo.b1RampMode  = 0;
			sgValvesInfo.b1LeanForWardStat  = 0;
			u8PumpOrPropValue = 0;		
		}
		
	}
}
//�µ���Ϊ״̬0������ǰ��   2���������
if(sgUserInfo.b1AngleAiLimit != 0)
{
	if(sgValvesInfo.u8AngleState == 0)
	{
		sgValvesInfo.b1LeanForWardStat = 0;
	}
	else if(sgValvesInfo.u8AngleState == 2)
	{
		sgValvesInfo.b1LeanBackWardStat = 0;	
	}
}
	/*Lift Mode*/

	if((sgLastValvesInfo.b1LiftUpStat != sgValvesInfo.b1LiftUpStat) ||
		(sgLastValvesInfo.b1LiftDownStat != sgValvesInfo.b1LiftDownStat) ||
		(sgLastValvesInfo.b1LeanForWardStat != sgValvesInfo.b1LeanForWardStat) ||	
		(sgLastValvesInfo.b1LeanBackWardStat != sgValvesInfo.b1LeanBackWardStat))
	{
		if (0 == SendData->u8PumpTarget)
		{
			sgLastValvesInfo.b1LiftUpStat = sgValvesInfo.b1LiftUpStat;
			sgLastValvesInfo.b1LiftDownStat = sgValvesInfo.b1LiftDownStat;
			sgLastValvesInfo.b1LeanForWardStat = sgValvesInfo.b1LeanForWardStat;
			sgLastValvesInfo.b1LeanBackWardStat = sgValvesInfo.b1LeanBackWardStat;
			
			if (1 == sgLastValvesInfo.b1LiftDownStat)
			{
				i32DoPwmSet(DOWN_VALVE,DRIVER_OPEN);
				i32DoPwmSet(LIFTUP_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(LEAN_FORWARD_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(LEAN_BACKWARD_VALVE, DRIVER_CLOSE);
			}
			else if (1 == sgLastValvesInfo.b1LeanBackWardStat)
			{
				i32DoPwmSet(LIFTUP_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(DOWN_VALVE,DRIVER_CLOSE);
				i32DoPwmSet(LEAN_FORWARD_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(LEAN_BACKWARD_VALVE, DRIVER_OPEN);
			}
			else if (1 == sgLastValvesInfo.b1LeanForWardStat)
			{
				i32DoPwmSet(LIFTUP_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(DOWN_VALVE,DRIVER_CLOSE);
				i32DoPwmSet(LEAN_FORWARD_VALVE, DRIVER_OPEN);
				i32DoPwmSet(LEAN_BACKWARD_VALVE, DRIVER_CLOSE);
			}
			else if(1 == sgLastValvesInfo.b1LiftUpStat)
			{
				i32DoPwmSet(LIFTUP_VALVE, DRIVER_OPEN);
				i32DoPwmSet(DOWN_VALVE,DRIVER_CLOSE);
				i32DoPwmSet(LEAN_FORWARD_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(LEAN_BACKWARD_VALVE, DRIVER_CLOSE);
			}			
			else 
			{
				i32DoPwmSet(DOWN_VALVE,DRIVER_CLOSE);
				i32DoPwmSet(LIFTUP_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(LEAN_FORWARD_VALVE, DRIVER_CLOSE);
				i32DoPwmSet(LEAN_BACKWARD_VALVE, DRIVER_CLOSE);				
			}
		}
		else
		{
			if (SendData->u8PumpTarget > 0) 
			{
				fTmp = sgUserInfo.fLiftSpdPer5msDecStep * (0.05 + 0.95 * SendData->u8PumpTarget / PUMP_RANGE);
				if ((SendData->u8PumpTarget > fTmp) && (0 != sgUserInfo.fLiftSpdPer5msDecStep))
				{
					SendData->u8PumpTarget -= fTmp;
				}
				else
				{
					SendData->u8PumpTarget = 0; 
				}
			}
		}
	}
	else
	{
			if ((1 == sgLastValvesInfo.b1LiftUpStat) || (1 == sgLastValvesInfo.b1LeanForWardStat)
			|| (1 == sgLastValvesInfo.b1LeanBackWardStat)||(1 == sgLastValvesInfo.b1LiftDownStat))
			{
				if (SendData->u8PumpTarget > u8PumpOrPropValue) 
				{
					fTmp = sgUserInfo.fLiftSpdPer5msDecStep * (0.05 + 0.95 * SendData->u8PumpTarget / PUMP_RANGE);
					if (((SendData->u8PumpTarget - u8PumpOrPropValue)> fTmp) && (0 != sgUserInfo.fLiftSpdPer5msDecStep))
					{
						SendData->u8PumpTarget -= fTmp;
					}
					else
					{
						SendData->u8PumpTarget = u8PumpOrPropValue; 
					}
				}
				else if (SendData->u8PumpTarget < u8PumpOrPropValue)
				{
					fTmp = sgUserInfo.fLiftSpdPer5msAccStep * (0.05 + 0.95 * SendData->u8PumpTarget / PUMP_RANGE);
					if (((u8PumpOrPropValue - SendData->u8PumpTarget) > fTmp) && (0 != sgUserInfo.fLiftSpdPer5msAccStep))
					{
						SendData->u8PumpTarget += fTmp;
					}
					else
					{
						SendData->u8PumpTarget = u8PumpOrPropValue;
					}
				}
				else
				{
					SendData->u8PumpTarget = u8PumpOrPropValue;
				}
			}
			else
			{
				SendData->u8PumpTarget = 0;
			}

	}
	
	if (0 != SendData->u8PumpTarget)
	{
		SendData->b1LiftReq = 1;
	}	
	if ((0 == sgSwiInput.b1Lock)
			)
	{
		SendData->b1ServoOn = 0;
	}
	if(1 == sgValvesInfo.b1EmsState)
	{
		 SendData->b1EmsReq = 1;
	}
	if (1 == sgValvesInfo.b1NoActFlag)
	{
		SendData->b1ServoOn = 0;				/*�����ƶ��� �Ͽ�ʹ��*/
		SendData->u8PumpTarget = 0;
		SendData->b1LiftReq = 0;
		SendData->u8TargetHigh = 0;
		SendData->u8TargetLow = 0;
		SendData->b1ForwardReq = 0;
		SendData->b1BackwardReq = 0;
	}
	else
	{
		if (1 == sgSwiInput.b1Lock)
		{
			SendData->b1ServoOn = 1;
		}
	}
	if(SendData->b1ServoOn)
	{   
		uint16_t WorkCountL = 0;
		uint16_t WorkCountH = 0;
		if(1 == SendData->b1ServoOn)
		{
			Workcnt++;
		}
		if (Workcnt >= 360*200)			/**/
		{
			Workcnt = 0;
			u32WorkCount++;
			WorkCountL = u32WorkCount;
			WorkCountH = u32WorkCount >> 16;
			u16EepromWrite(PARA_WorkCountL, WorkCountL, 1);
			u16EepromWrite(PARA_WorkCountH, WorkCountH, 1);
		}	
  }
	
	{
		static uint32_t u32Cnt = 0;
		u32Cnt++;
		{
			i32LogWrite(INFO, LOG_USER, "SendStat = 0x%x, Pump = %d, Prop = %d, MotorSpd = %d, u16MotorVal = %d, Valve0 = 0x%x, Valve1 = 0x%x\r\n",  \
				SendData->buf[2], SendData->u8PumpTarget, u8PumpOrPropValue, i16Spd, u16MotorVal, sgValvesInfo.u8Data[0], sgValvesInfo.u8Data[1]);
			
			/*lilu 20230819 For Test*/
			{
				i32SetPara(PARA_ForwardValveCurrent,u8PumpOrPropValue);		/*Send Motor Value*/
				i32SetPara(PARA_BackValveCurrent,sgLastValvesInfo.u8Data[1]);	
				i32SetPara(PARA_BackValveCurrent,gCanRevPdoInfo.CanRevInfo5.i16ThrottleValue );				/*Rev Motor Value*/
				i32SetPara(PARA_PropValveCurrent, inserted_data[0] * PROP_CURRENT_FACOTR * 1000);	/*Prop Current*/
				i32SetPara(PARA_LiftValveCurrent, gCanSendPdoInfo.CanSendInfo1.s16SpeedMeasured);		/*Send �ͱõ�� Value*/
								
				//i32SetPara(PARA_OnOffValveCurrent, u16MotorVal);			/*Send Status*/
				
				i32SetPara(PARA_OnOffValveCurrent, sgValvesInfo.u8AngleState);
																															 
				i32SetPara(PARA_LoadRate, sgUserInfo.b1RampMode);				/*Valve NoFlag*/
				i32SetPara(PARA_CalibrationStatus, sgValvesInfo.u8Data[1]);		/*Value State*/
				i32SetPara(PARA_TurnRightValveCurrent, ((uint16_t)sgValvesInfo.u8NoAct << 8) | ((uint16_t)sgValvesInfo.b4NoMove << 4) | ((uint16_t)sgValvesInfo.b4NoLiftUp));
				i32SetPara(PARA_TurnLeftValveCurrent, ((uint16_t)sgValvesInfo.b4Gear1Spd << 12) | ((uint16_t)sgValvesInfo.b4Gear2Spd << 8) | \
					((uint16_t)sgValvesInfo.b4Gear3Spd << 4) | ((uint16_t)sgValvesInfo.b4Gear4Spd));
				i32SetPara(PARA_AngleValue, (uint16_t)i32LocalAiGetValue(AI_B_AI1_R));		/*AI1*/
				i32SetPara(PARA_PressureVlaue1, sgValvesInfo.s16TargetAngle);     	/*AI2  ��ʱ��ʾ�ͱõ������*/
				i32SetPara(PARA_PressureVlaue2, sgValvesInfo.s16RampAngle);	/*AI3  ��ʱ��ʾ185ǰ�������*/
				i32SetPara(PARA_ExtSignal, sgSwiInput.u16data);						/*Swi*/
//				i32SetPara(PARA_PcuKeyInfo, u8ErrCodeGet());						/*ErrCode*/
				i32SetPara(PARA_BmsSoc, u8ErrCodeGet());		/*BMS SOC*/
//				i32SetPara(PARA_BrakeValveCurrent, abs(gCanRevPdoInfo.CanRevInfo2.i16SteerAngle));
				i32SetPara(PARA_SteerAngle,(sgUserInfo.s16SteerAngle));
			}
		}
	}
 
}
static void vDoPwmErrCallBack(eDoPwmNo DoPwmNo)
{
	switch((uint8_t)DoPwmNo)
	{
		case LIFTUP_VALVE:
			i32ErrCodeSet(LIFTUP_VALVE_ERR);
			/*add ErrCode*/
			break;
		case LEAN_FORWARD_VALVE:
			i32ErrCodeSet(LEAN_FORWARD_VALVE_ERR);
			/*add ErrCode*/
			break;
		case LEAN_BACKWARD_VALVE:
			i32ErrCodeSet(LEAN_BACKWARD_VALVE_ERR);
			/*add ErrCode*/
			break;
		default:
			break;
	}
}

static void vPropErrCallBack(uint8_t u8Channel)
{
	switch(u8Channel)
	{
//		case LIFTDOWN_VALVE:
//			i32ErrCodeSet(LIFTDOWN_VALVE_ERR);
			/*add errcode*/
//			break;
		case  LIFTUP_VALVE:
			i32ErrCodeSet(LIFTUP_VALVE_ERR);
			break;
		case LEAN_FORWARD_VALVE:
			i32ErrCodeSet(LEAN_FORWARD_VALVE_ERR);
			break;
		case LEAN_BACKWARD_VALVE:
			 i32ErrCodeSet(LEAN_BACKWARD_VALVE_ERR);
			break;
		default:
			break;
	}
}

static void vAiErrCallBack(eAiErrNo AiErrNo)
{
	switch((uint8_t)AiErrNo)
	{
		case TURN_SEN_INPUT:
			i32ErrCodeSet(AI_B_AI1_ERR);
			break;
		case ANG_SEN_INPUT:
			i32ErrCodeSet(TILESENSOR_ERR);
			break;
		case PRESS_SEN_INPUT:
			i32ErrCodeSet(PRESSENSOR_ERR);
			break;
		default:
			break;							
	}
}
/*����PDO������*/
void vCanLostProc(uint32_t u32Canid, uint8_t u8State)
{
	switch(u32Canid)
	{
		case 0x1E0: 
			if (CAN_NORMAL == u8State)
			{
				/*clr lost error*/
				i32ErrCodeClr(HANDLE_NOCAN_ERR);
			}
			else if (CAN_LOST == u8State)
			{
				/*add lost error*/
				i32ErrCodeSet(HANDLE_NOCAN_ERR);
				memset((char*)gCanRevPdoInfo.CanRevInfo5.u8Data, 0x00, sizeof(gCanRevPdoInfo.CanRevInfo5));
			}
			break;
		case 0x185: 
			if (CAN_NORMAL == u8State)
			{
				i32ErrCodeClr(FRONTRANK_NOCAN_ERR);
			}
			else if (CAN_LOST == u8State)
			{
				i32ErrCodeSet(FRONTRANK_NOCAN_ERR);
				memset((char*)gCanRevPdoInfo.CanRevInfo6.u8Data, 0x00, sizeof(gCanRevPdoInfo.CanRevInfo6));
				memset((char*)gCanRevPdoInfo.CanRevInfo7.u8Data, 0x00, sizeof(gCanRevPdoInfo.CanRevInfo7));
			}
			break;
		case 0x186:
			if (CAN_NORMAL == u8State)
			{
				i32ErrCodeClr(CASTER_NOCAN_ERR);
			}	
			else if (CAN_LOST == u8State)
			{
				i32ErrCodeSet(CASTER_NOCAN_ERR);
				memset((char*)gCanRevPdoInfo.CanRevInfo6.u8Data, 0x00, sizeof(gCanRevPdoInfo.CanRevInfo6));
				memset((char*)gCanRevPdoInfo.CanRevInfo7.u8Data, 0x00, sizeof(gCanRevPdoInfo.CanRevInfo7));
			}
			break;
		case 0x2F0:
			if (CAN_NORMAL == u8State)
			{
				/*clr lost error*/
				i32ErrCodeClr(BMS_NOCAN_ERR);
			}
			else if (CAN_LOST == u8State)
			{
				/*add lost error*/
				i32ErrCodeSet(BMS_NOCAN_ERR);
			}
			break;
		default:
			break;
	}

}

static void vCanRevPdoProc(void)
{
	static xCanRev1E0Info CanRev1E0InfoLast;
	static uint16_t u16CanRev1E0Cnt;
	int16_t i16ThrottleInput;
	int8_t i8LiftDownInput;
	int8_t i8LeanInput;
	static xCanRev18XInfo CanRev185InfoLast;
	static xCanRev18XInfo CanRev186InfoLast;
	static xCanRev2F0Info CanRev2F0InfoLast;
	static xCanRev288Info CanRev288InfoLast;
	static xCanRev289Info CanRev289InfoLast;
	static uint8_t UpRampDelay;
	static uint8_t DownRampDelay;
	static int8_t Delay_Count;
	if(0 != sgUserInfo.b1TileSensorEn)
	{
		//185ǰ��ǻ�ȡ   ��185�������Զ���ƽ��
		if(0 != memcmp((char*)CanRev185InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo6.u8Data, sizeof(CanRev185InfoLast)))
		{
			memcpy((char*)CanRev185InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo6.u8Data, sizeof(CanRev185InfoLast));
			if(CanRev185InfoLast.Xsign == 0)
			{
				sgValvesInfo.s16TargetAngle = ((CanRev185InfoLast.Xhigh/16) *10 + CanRev185InfoLast.Xhigh %16);
			}
			else
			{
				sgValvesInfo.s16TargetAngle = -1*((CanRev185InfoLast.Xhigh/16)*10+ CanRev185InfoLast.Xhigh %16);
			}		
		}
		//186����ǻ�ȡ   ��186�����������µ��Ƕȣ�
		if(0 != memcmp((char*)CanRev186InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo7.u8Data, sizeof(CanRev186InfoLast)))
		{
			memcpy((char*)CanRev186InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo7.u8Data, sizeof(CanRev186InfoLast));
			if(CanRev186InfoLast.Xsign == 0)
			{
				sgValvesInfo.s16RampAngle = (CanRev186InfoLast.Xhigh/16) *10 + CanRev186InfoLast.Xhigh % 16;
			}
			else
			{                                                                    
				sgValvesInfo.s16RampAngle = -1*(CanRev186InfoLast.Xhigh/16) *10 + CanRev186InfoLast.Xhigh % 16;
			}
		}
		//��������Զ��µ��Ǿͽ����Զ��µ�ģʽ  ���㱨#
			if(sgValvesInfo.s16RampAngle >= sgUserInfo.u8RampLowestAngle)
			{
				if(DownRampDelay > 200)
				{
					sgLimitInfo.DirectionErr = 1;//�жϳ���������λ�ô���   1S�˲�
					
					if((0 != sgValvesInfo.b1ForWardStat)&&(sgValvesInfo.b3DirectionSave == 0))  //����������ֹ����ǰ������ʻ
					{
						sgValvesInfo.b3DirectionSave = 1;//����
						sgLimitInfo.NoForWordFlag = 1;
						sgLimitInfo.NoBackWordFlag = 0;
					}
					else if((0 != sgValvesInfo.b1BackWardStat)&&(sgValvesInfo.b3DirectionSave == 0))
					{
						sgValvesInfo.b3DirectionSave = 2;//����
						sgLimitInfo.NoForWordFlag = 0;
						sgLimitInfo.NoBackWordFlag = 1;
					}
					i32ErrCodeSet(DIRICTION_ERR);
				}
				else
				{
					DownRampDelay++;
				}
			}
			else
			{
				 sgValvesInfo.b3DirectionSave = 0;
				 sgLimitInfo.NoForWordFlag = 0;
				 sgLimitInfo.NoBackWordFlag = 0;
				 i32ErrCodeClr(DIRICTION_ERR);
				 DownRampDelay = 0;
				 sgLimitInfo.DirectionErr = 0;
			}
			if(sgValvesInfo.s16RampAngle <= sgUserInfo.u8RampHighestAngle)
			{
				if(UpRampDelay > 200)
				{
						sgValvesInfo.b1EnterRampFlag = 1;//�ж�������
				}
				else
				{
					UpRampDelay++;
				}
			}
			else
			{
				UpRampDelay = 0;
				sgValvesInfo.b1EnterRampFlag = 0;
			}
			if(sgValvesInfo.s16RampAngle >= sgUserInfo.u8RampLowestAngle)
			if(sgValvesInfo.s16RampAngle < sgUserInfo.s16AutoRampAngle)
			{
				sgValvesInfo.b1AutoRampModeFlag = 1;
				sgValvesInfo.b1CantExitRampMode = 1;
			}
			if(sgValvesInfo.s16RampAngle > sgUserInfo.s16SetExitAngle)
			{
				 sgValvesInfo.b1CantExitRampMode = 0;
			}
			if((sgValvesInfo.s16RampAngle <= sgUserInfo.s16SetMaxtileAngle))
			{
				if((0 != sgValvesInfo.b1ForWardStat)&&(sgValvesInfo.b3DirectionSave == 0))  //�������ֹ����ǰ������ʻ
				{
					sgValvesInfo.b3DirectionSave = 1;//����
					sgLimitInfo.NoForWordFlag = 1;
					sgLimitInfo.NoBackWordFlag = 0;
				}
				else if((0 != sgValvesInfo.b1BackWardStat)&&(sgValvesInfo.b3DirectionSave == 0))
				{
					sgValvesInfo.b3DirectionSave = 2;//����
					sgLimitInfo.NoForWordFlag = 0;
					sgLimitInfo.NoBackWordFlag = 1;
					
				}
				sgLimitInfo.OverLean = 1;
				i32ErrCodeSet(OVER_LEAN_ERR);
			}
			else
			{
				sgValvesInfo.b3DirectionSave = 0;//����
				sgLimitInfo.NoForWordFlag = 0;
				sgLimitInfo.NoBackWordFlag = 0;
				i32ErrCodeClr(OVER_LEAN_ERR);
				sgLimitInfo.OverLean = 0;	
			}
	}
		if(0 != memcmp((char*)CanRev1E0InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo5.u8Data, sizeof(CanRev1E0InfoLast)))
		{
			memcpy((char*)CanRev1E0InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo5.u8Data, sizeof(CanRev1E0InfoLast));
			/*�����صĲ���*/
		//	i16ThrottleInput = (gCanRevPdoInfo.CanRevInfo5.i16ThrottleValueL |(gCanRevPdoInfo.CanRevInfo5.i16ThrottleValueH << 8));
			i16ThrottleInput = CanRev1E0InfoLast.i16ThrottleValue;
			i8LiftDownInput = CanRev1E0InfoLast.i8LiftDownValue;
			i8LeanInput = CanRev1E0InfoLast.i8LeanValue;
			
			u16MotorVal = abs(i16ThrottleInput);
			
			if (0 > i16ThrottleInput)
			{
				sgValvesInfo.b1ForWardStat = 1; 
				sgValvesInfo.b1BackWardStat = 0;
				sgLimitInfo.ReverseSpd = 0; 
				if((0 != sgValvesInfo.b1EnterRampFlag) || (sgUserInfo.b1RampMode))  //�ж�������
				{
					 sgLimitInfo.UpslopeSpd = 1;
				}
				else
				{
					 sgLimitInfo.UpslopeSpd = 0;
				}

			}
			else if (0 < i16ThrottleInput)
			{
				sgValvesInfo.b1ForWardStat = 0; 
				sgValvesInfo.b1BackWardStat = 1;
				sgLimitInfo.ReverseSpd = 1;    			 //������
				if((0 != sgValvesInfo.b1EnterRampFlag)||(sgUserInfo.b1RampMode))//�ж�������
				{
					sgLimitInfo.DownslopeSpd = 1;
				}
				else 
				{
					 sgLimitInfo.DownslopeSpd = 1;
				}
			}
			else
			{
				sgLimitInfo.ReverseSpd = 0; 
				sgValvesInfo.b1ForWardStat = 0; 
				sgValvesInfo.b1BackWardStat = 0; 
			}
			if((CanRev1E0InfoLast.b1RampModeReq != 0)
				&& (sgUserInfo.b1RampMode == 0) 
				&& (0 !=sgLimitInfo.DirectionErr))//�Ѿ������µ�ģʽ ��Ƕȴ�������²��ܽ����µ�ģʽ
			{
				sgValvesInfo.b1RampModeFlag = 1;
				sgValvesInfo.s16SaveAngle = sgValvesInfo.s16TargetAngle;    //�����ֶ��µ�ģʽ�µ�һ�ν���ĽǶ�Ȼ����ڸ�Ԥ��б��	
			}
			else if((sgValvesInfo.b1CantExitRampMode != 1) && (CanRev1E0InfoLast.b1RampModeExit == 1))
			{
				sgValvesInfo.b1RampModeFlag = 0;
				sgValvesInfo.b1AutoRampModeFlag = 0;
				sgValvesInfo.b1PreLeanFlag = 0;			//�Ѿ�����Ԥ��б�Ǿ���1
			}
			if(0 != sgUserInfo.b1RampMode)
			{
				if (CanRev1E0InfoLast.b1SlowModeReq == 1)
				{
					sgLimitInfo.TuttleSpd = 1;
				}
				else 
				{
					sgLimitInfo.TuttleSpd = 0;
				}
			}
			else 
			{
				sgLimitInfo.TuttleSpd = 0;
			}
			
			if (CanRev1E0InfoLast.b1HornReq == 1)
			{
				sgValvesInfo.b1HornStat = 1;
			}
			else 
			{
				sgValvesInfo.b1HornStat = 0;
			}
			if(CanRev1E0InfoLast.b1EmsReq == 1)
			{
				 sgValvesInfo.b1EmsState = 1;
			}
			else
			{
				 sgValvesInfo.b1EmsState = 0;
			}
			if(sgUserInfo.b1RampMode == 1) //�µ�ģʽ�²����� �ֶ������ͱõ��
			{
				CanRev1E0InfoLast.b1LeanBackWardReq = 0;
				CanRev1E0InfoLast.b1LeanForWardReq = 0;
			}
			if((CanRev1E0InfoLast.b1LeanBackWardReq == 1)
				||(CanRev1E0InfoLast.b1LeanForWardReq == 1)
				||(CanRev1E0InfoLast.b1DownReq == 1)
				||(CanRev1E0InfoLast.b1LiftReq == 1))
			{
				if(Delay_Count >= 6)//����������ʱ�����ʱ����
				{
//					if((CanRev1E0InfoLast.b1LeanBackWardReq == 1)&&(CanRev1E0InfoLast.b1LeanForWardReq == 1)) //ǰ�����ͬʱ��Чʱ
//					{
//						sgValvesInfo.b4NoLean |= (1<<NoLean_LogicErr);
//					}
//					else if((CanRev1E0InfoLast.b1DownReq == 1)&&(CanRev1E0InfoLast.b1LiftReq == 1))
//					{
//						sgValvesInfo.b4NoLiftUp |= (1<<NoLean_LogicErr);
//						sgValvesInfo.b4NoLiftDown|= (1<<NoLiftDown_LogicErr);
//					}
					if (CanRev1E0InfoLast.b1LeanBackWardReq == 1)
					{
						sgValvesInfo.b1LeanBackWardStat = 1;
						sgValvesInfo.b1LeanForWardStat = 0;
						u8PumpOrPropValue = ((abs(i8LeanInput) * (PUMP_MAX_VALUE-21) / 48) + 21);
					}
					else if (CanRev1E0InfoLast.b1LeanForWardReq == 1)
					{
						sgValvesInfo.b1LeanBackWardStat = 0;
						sgValvesInfo.b1LeanForWardStat = 1;
						u8PumpOrPropValue = (((abs(i8LeanInput)) * (PUMP_MAX_VALUE-21) / 48) + 21);		
					}
					else if (CanRev1E0InfoLast.b1DownReq == 1)
					{
						sgValvesInfo.b1LiftDownStat = 1;
						sgValvesInfo.b1LiftUpStat = 0;
						u8PumpOrPropValue =  ((abs(i8LiftDownInput) * (PUMP_MAX_VALUE-21) / 48) + 21);
					}			
					else if (CanRev1E0InfoLast.b1LiftReq == 1)
					{
						sgValvesInfo.b1LiftUpStat = 1;
						sgValvesInfo.b1LiftDownStat = 0;
						u8PumpOrPropValue =  ((abs(i8LiftDownInput) * (PUMP_MAX_VALUE-21) / 48) + 21);
					}
				 } 
				else
					Delay_Count++;
			}
			else if(sgUserInfo.b1RampMode == 1)
			{
				//�µ�ģʽ�²����κδ���
			}
			else 
			{
				sgValvesInfo.b1LiftDownStat = 0;
				sgValvesInfo.b1LiftUpStat = 0;
				sgValvesInfo.b1LeanBackWardStat = 0;
				sgValvesInfo.b1LeanForWardStat = 0;
				Delay_Count = 0;
				u8PumpOrPropValue = 0;
			}
		}		
 		if(0 != memcmp((char*)CanRev2F0InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfoBMS.u8Data, sizeof(CanRev2F0InfoLast)))
		{
			memcpy((char*)CanRev2F0InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfoBMS.u8Data, sizeof(CanRev2F0InfoLast));
			sgValvesInfo.u8Soc = (CanRev2F0InfoLast.u8Soc * 4)/10;
			i32SetPara(PARA_BmsSoc,sgValvesInfo.u8Soc); 
			gCanSendPdoInfo.CanHMISendInfo.u8BMS = sgValvesInfo.u8Soc;//��ص���SOC
			
			if(CanRev2F0InfoLast.b1ChargeFlag)
			{
				sgLimitInfo.ChargeFlag = 1;
				sgValvesInfo.b1HMI_ChargeFlag = 1;//��ʾ��ط���
			}
			else
			{
				sgLimitInfo.ChargeFlag = 0;
				sgValvesInfo.b1HMI_ChargeFlag = 0;	
			}
			if(CanRev2F0InfoLast.b1SoloUnderVotageErr)
			{
				sgValvesInfo.u8LimitSpeed |= 1<<LimitSpeed_LowPower;
				i32ErrCodeSet(SoloUnderVotage_ERR);
			}
			else
			{
				sgValvesInfo.u8LimitSpeed &= ~(1<<LimitSpeed_LowPower);
				i32ErrCodeClr(SoloUnderVotage_ERR);
			}
			if(CanRev2F0InfoLast.b1TmpProtectErr)
			{
				sgValvesInfo.u8LimitSpeed |= 1<<LimitSpeed_TmpProtect;
				i32ErrCodeSet(TmpProtect_ERR);
			}
			else
			{
				sgValvesInfo.u8LimitSpeed &= ~(1<<LimitSpeed_TmpProtect);
				i32ErrCodeClr(TmpProtect_ERR);
			}
			if(sgValvesInfo.u8Soc <= sgUserInfo.u8LowPow1Step && sgValvesInfo.u8Soc > sgUserInfo.u8LowPow2Step) //�͵���һ������
			{
				sgValvesInfo.u8NoAct &= ~(1 << NoAct_LowPower);
				sgLimitInfo.LowPowFlag = 1;
				i32ErrCodeSet(LOWPOWER_1ERR);
				i32ErrCodeClr(LOWPOWER_2ERR);
				sgValvesInfo.b1HMI_LowPowerFlag = 1;
			}
			else if(sgValvesInfo.u8Soc <= sgUserInfo.u8LowPow2Step)//�͵�����������
			{ 
				sgValvesInfo.b1HMI_LowPowerFlag = 1;
				i32ErrCodeClr(LOWPOWER_1ERR);				
				i32ErrCodeSet(LOWPOWER_2ERR);
				sgValvesInfo.u8NoAct |= 1 << NoAct_LowPower;
			}
			else
			{
				sgValvesInfo.u8NoAct &= ~(1 << NoAct_LowPower);
				sgLimitInfo.LowPowFlag = 0;
				i32ErrCodeClr(LOWPOWER_1ERR);
				i32ErrCodeClr(LOWPOWER_2ERR);
				sgValvesInfo.b1HMI_LowPowerFlag = 0;
			}
			
			if((CanRev2F0InfoLast.b1OverCurrentErr)
				||(CanRev2F0InfoLast.b1OverTmpErr )
				||(CanRev2F0InfoLast.b1UnderVotageErr)
				||(CanRev2F0InfoLast.b1OverVotageErr))
			{
				sgValvesInfo.u8NoAct|= 1<<NoAct_BmsErr;
				if(CanRev2F0InfoLast.b1OverVotageErr)
					i32ErrCodeSet(OverVotage_ERR);
				else if(CanRev2F0InfoLast.b1OverCurrentErr)
					i32ErrCodeSet(OverCurrent_ERR);
				else if(CanRev2F0InfoLast.b1OverTmpErr)
					i32ErrCodeSet(OverTmp_ERR);
				else if(CanRev2F0InfoLast.b1UnderVotageErr)
					i32ErrCodeSet(AllUnderVotage_ERR);
			}
			else if((CanRev2F0InfoLast.b1SoloUnderVotageErr)
						||(CanRev2F0InfoLast.b1TmpProtectErr))
			{
				sgLimitInfo.TuttleSpd = 1;     //��������
				if(CanRev2F0InfoLast.b1SoloUnderVotageErr)
					i32ErrCodeSet(SoloUnderVotage_ERR);
				else if(CanRev2F0InfoLast.b1TmpProtectErr)
					i32ErrCodeSet(TmpProtect_ERR);
			}
			else
			{
				sgValvesInfo.u8NoAct &= ~(1<<NoAct_BmsErr);
				i32ErrCodeClr(OverVotage_ERR);
			} 
		}
		if(sgUserInfo.b1InterLockEN == 1)
		{
			if(0 != memcmp((char*)CanRev288InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo288.u8Data, sizeof(CanRev288InfoLast)))
			{
				memcpy((char*)CanRev288InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo288.u8Data, sizeof(CanRev288InfoLast));
				if(CanRev288InfoLast.u8ErrCode288 <= 30 && CanRev288InfoLast.u8ErrCode288 != 0 )        //�Ӷ���30���ڹ��ϻ���
					sgValvesInfo.u8ErrCode288 = CanRev288InfoLast.u8ErrCode288;	 
				else
					sgValvesInfo.u8ErrCode288 = 0;
			}	                                       
			if(0 != memcmp((char*)CanRev289InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo289.u8Data, sizeof(CanRev289InfoLast)))
			{
				memcpy((char*)CanRev289InfoLast.u8Data, (char*)gCanRevPdoInfo.CanRevInfo289.u8Data, sizeof(CanRev289InfoLast));
				if(CanRev289InfoLast.u8ErrCode289 <= 30 && CanRev288InfoLast.u8ErrCode288 != 0)        //�Ӷ���30���ڹ��ϻ���
					sgValvesInfo.u8ErrCode289 = CanRev289InfoLast.u8ErrCode289;
				else
					sgValvesInfo.u8ErrCode289 = 0;	
			}
			if((sgValvesInfo.u8ErrCode288 != 0 )||(sgValvesInfo.u8ErrCode289 != 0))
			{
				sgValvesInfo.u8NoAct |= (1 << NoAct_SmoveErr);
			  i32ErrCodeSet(SMOVE_ERR);
			}
			else
			{
				sgValvesInfo.u8NoAct &= ~(1 << NoAct_SmoveErr);
				i32ErrCodeClr(SMOVE_ERR);
			}
		}
 }
 
 static void HornFlick(void)  //�εα�������
 {
	 static uint8_t BeepCount;
		if(BeepCount >160)
		{
			 BeepCount = 0;
		}
		else if(BeepCount > 80)
		{
			i32DoPwmSet(HONR_VALVE,DRIVER_OPEN);
		}
		else if(BeepCount >0)
		{
			 i32DoPwmSet(HONR_VALVE,DRIVER_CLOSE);
		}
		BeepCount++;
 }
/*����PDO������*/
static void vCanSendPdoProc(void)
{
	if(1 == i32LocalDiGet(LOCK_SWI))
		gCanSendPdoInfo.CanSendInfo1.u16StateWord |= (1<<6); //SWITCH3_StateWord
	else	
		gCanSendPdoInfo.CanSendInfo1.u16StateWord &= ~(1<<6);
	
	gCanSendPdoInfo.CanSendInfo1.u16StateWord |= (1<<13); //AGVModeEnable_StateWord
	
	if(0 != sgUserInfo.b1TurnDifSpdEn)    //�����رղ��ٹ��ܣ����ӿ���������ת��Ƕ���Ϣ��
		gCanSendPdoInfo.CanSendInfo1.s16SteerAngle = sgUserInfo.s16SteerAngle;
	else 
		gCanSendPdoInfo.CanSendInfo1.s16SteerAngle = 0;
}
	
/*lilu 20230703 ģ�������*/
static void vAiMonitor(void)
{
	int32_t i32AdcValue = 0;
	//����Analog1ȷ��ת��Ƕ�
	if(sgUserInfo.u16TurnAngleMax != 0 || sgUserInfo.u16TurnAngleMin != 0)
	{
		i32AdcValue = i32LocalAiGetValue(TURN_SEN_INPUT);
		if(sgUserInfo.u16TurnAngleMin > i32AdcValue)
		{
			i32AdcValue = sgUserInfo.u16TurnAngleMin;
		}
		if(i32AdcValue > sgUserInfo.u16TurnAngleMax)
		{
			 i32AdcValue = sgUserInfo.u16TurnAngleMax;
		}
		if(i32AdcValue > sgUserInfo.u16TurnAngleMiddle + sgUserInfo.u16TurnAngleDeadZoon)
		{
			sgUserInfo.s16SteerAngle = (i32AdcValue -(sgUserInfo.u16TurnAngleMiddle +sgUserInfo.u16TurnAngleDeadZoon))*900/(sgUserInfo.u16TurnAngleMiddle -sgUserInfo.u16TurnAngleDeadZoon);
		}
		else if(i32AdcValue < sgUserInfo.u16TurnAngleMiddle - sgUserInfo.u16TurnAngleDeadZoon)
		{
			sgUserInfo.s16SteerAngle = ((sgUserInfo.u16TurnAngleMiddle -sgUserInfo.u16TurnAngleDeadZoon)- i32AdcValue)*900/(sgUserInfo.u16TurnAngleMiddle -sgUserInfo.u16TurnAngleDeadZoon);		
		}
		else
		{
			sgUserInfo.s16SteerAngle = 0;
		}
	}
	else
		sgUserInfo.s16SteerAngle = 0 ;
	
	//����Analog2  ȷ����б�Ƕ�״̬
	i32AdcValue = i32LocalAiGetValue(ANG_SEN_INPUT);
	if(i32AdcValue < sgUserInfo.s16ForwardLimitAngle)
	{
		 sgValvesInfo.u8AngleState = 0;	
	}
	else if(i32AdcValue > sgUserInfo.s16BackwardLimitAngle)
	{
		 sgValvesInfo.u8AngleState = 2;
	}
	else 
	{
		sgValvesInfo.u8AngleState = 1;
	}    
	//����Analog3ȷ��ѹ��ֵ
	i32AdcValue = i32LocalAiGetValue(PRESS_SEN_INPUT);
	if (i32AdcValue > sgUserInfo.u16MaxPresure)
	{                                         
		sgValvesInfo.u8NoAct |= 1 << NoAct_OverPress;
		i32ErrCodeSet(OVER_PRESS_ERR);
	}
	else
	{
		sgValvesInfo.u8NoAct &= ~(1 << NoAct_OverPress);
		i32ErrCodeClr(OVER_PRESS_ERR);
	}
}

/*���������*/
static void vSwiMonitor(void)
{
	xSwiInput SwiInput = {0};
	
	if(1 == i32LocalDiGet(LIFT_UP_LIMIT_SWI))
	{
		SwiInput.b1LiftLimit = 1;
	}

	if(1 == i32LocalDiGet(EMS_SWI))
	{
		SwiInput.b1Ems = 1;
	}
	
	if(1 == i32LocalDiGet(LOCK_SWI))
	{
		SwiInput.b1Lock = 1;
	}
	if(1 == i32LocalDiGet(CHARGE_SWI))
	{
		SwiInput.b1Charge = 1;
	}
	
	if (0 == SwiInput.b1Lock)
	{
		if ((1 == sgValvesInfo.b1ForWardStat) || (1 == sgValvesInfo.b1BackWardStat))
		{
			sgValvesInfo.b4NoMove |= 1 << NoMove_Lock;
			i32ErrCodeSet(ACT_LOCK_ERR);
		}
		else if(((0 != sgValvesInfo.b1LeanForWardStat)||(0 != sgValvesInfo.b1LeanBackWardStat)) && (0 !=sgUserSet.b1TileLockEn))
		{
			sgValvesInfo.b4NoLean |= 1 <<NoLean_Lock;
			i32ErrCodeSet(ACT_LOCK_ERR);
		}
		else if(((0 != sgValvesInfo.b1LiftUpFlag)||(0 != sgValvesInfo.b1LiftDownFlag))&&(0 != sgUserSet.b1PumpLockEn))
		{
			sgLimitInfo.NoUpFlag = 1;
			sgLimitInfo.NoDownFlag =1;
			i32ErrCodeSet(ACT_LOCK_ERR);
		}  
		else 
		{
			sgValvesInfo.b4NoMove &= ~(1 <<NoMove_Lock);
			sgValvesInfo.b4NoLean &= ~(1 <<NoLean_Lock);
			sgLimitInfo.NoUpFlag = 0;
			sgLimitInfo.NoDownFlag =0;
			i32ErrCodeClr(ACT_LOCK_ERR);
		}
	}
	

	if ((0 == sgValvesInfo.b1ForWardStat) && (0 == sgValvesInfo.b1BackWardStat) &&
	    (0 == sgValvesInfo.b1LeanForWardStat) && (0 == sgValvesInfo.b1LeanBackWardStat) &&
	    (0 == sgValvesInfo.b1LiftUpStat) && (0 == sgValvesInfo.b1LiftDownStat) && 
	    (0 == SwiInput.b1Ems) && (0 == SwiInput.b1Lock))
	{
		sgValvesInfo.b4NoMove &= ~(1 << NoMove_Init);
		sgValvesInfo.b4NoPump &= ~(1 <<NoPump_Init);
	
		i32ErrCodeClr(ACT_INIT_ERR);
		i32ErrCodeClr(ACT_LOCK_ERR);
	}
	if (0 != sgValvesInfo.u8NoAct)
	{
		sgLimitInfo.NoActFlag = 1;
	}
	else
	{
		sgLimitInfo.NoActFlag = 0;
	}
	if (0 != sgValvesInfo.b4NoMove)
	{
		sgLimitInfo.NoMoveFlag = 1;
	}
	else
	{
		sgLimitInfo.NoMoveFlag = 0;
	}
  if (0 != sgValvesInfo.b4NoPump)
	{
		sgLimitInfo.NoPumpFlag = 1;
	}
	else
	{
		sgLimitInfo.NoPumpFlag= 0;
	}
	if (0 != sgValvesInfo.b4NoLiftUp)
	{
		sgLimitInfo.NoUpFlag = 1;
	}
	else
	{
		sgLimitInfo.NoUpFlag = 0;
	}
	if (0 != sgValvesInfo.b4NoLiftDown)
	{
		sgLimitInfo.NoDownFlag = 0;
	}
	else
	{
		sgLimitInfo.NoDownFlag = 1;
	}
	
	
	if(0 != sgValvesInfo.u8LimitSpeed)
	{
		sgValvesInfo.b1LimtSpeedFlag = 1;	
	}
	else 
	{
		sgValvesInfo.b1LimtSpeedFlag = 0;
	}
	/*����߼�*/
	if (0 == sgLimitInfo.NoActFlag)
	{
//		if (sgSwiInput.u16data != SwiInput.u16data)
		{
			if ((1 == SwiInput.b1LiftLimit) && (1 == sgValvesInfo.b1LiftUpStat))
			//if ((0 == sgSwiInput.b1UpLimit) && (1 == SwiInput.b1UpLimit) && (1 == sgValvesInfo.b1LiftUpStat))
			{
//				sgValvesInfo.b4Gear2Spd = 1 << Gear2_Spd_Con1;
				sgSwiInput.b1HeightLimit = 1;
				sgSaveState.b1HeightLimit = 1;
				u16EepromWrite(PARA_SaveState, sgSaveState.u16Data, 1);	
			}
			else if ((1 == SwiInput.b1LiftLimit) && (1 == sgValvesInfo.b1LiftDownStat))
			//else if ((0 == sgSwiInput.b1UpLimit) && (1 == SwiInput.b1UpLimit) && (1 == sgValvesInfo.b1LiftDownStat))
			{
//				sgValvesInfo.b4Gear2Spd &=  ~(1 << Gear2_Spd_Con1);
				sgSwiInput.b1HeightLimit = 0;
				sgSaveState.b1HeightLimit = 0;
				u16EepromWrite(PARA_SaveState, sgSaveState.u16Data, 1);	
			}
					
			SwiInput.b1HeightLimit = sgSwiInput.b1HeightLimit;			/*lilu 20230801 Above1M8 retain*/
			sgSwiInput.u16data = SwiInput.u16data;   /**/
			

			/*Limit Up*/
			if ((1 == sgSwiInput.b1HeightLimit) || (1 == SwiInput.b1LiftLimit))
			{
				sgValvesInfo.b4NoLiftUp |= 1 << NoLiftUp_HeightLimit;
			}
			else
			{
				sgValvesInfo.b4NoLiftUp &= ~(1 << NoLiftUp_HeightLimit);
			}

		}
	}
	else
	{
		/*ForWard BackWard LiftUp LiftDown LeanForWard LeanBackWard all disable*/
//		sgValvesInfo.u8Data[1] &= 0xC0;		/*Hight 2 Bit Reserve*/	
		sgValvesInfo.u8Data[1] &= 0x80;
	}
}
static uint8_t u8SwiInitChcek(void)
{
	uint8_t u8Res = 0;
		
	if ((1 == sgValvesInfo.b1ForWardStat) || (1 == sgValvesInfo.b1BackWardStat) ||		/*ǰ������*/
			(1 == sgValvesInfo.b1EmsState)||((0 == sgUserInfo.b1StartUpLock)&&(1 == i32LocalDiGet(LOCK_SWI))))		/*����*/
	{
		u8Res = 1;
	}
	else if(1 == sgValvesInfo.b1LiftUpStat || 1 == sgValvesInfo.b1LiftDownStat ||					/*�����½�*/
				1 == sgValvesInfo.b1LeanForWardStat || 1 == sgValvesInfo.b1LeanBackWardStat ||
				((0 == sgUserInfo.b1StartUpLock)&&(1 == i32LocalDiGet(LOCK_SWI)))	/*ǰ�����*/)
	 {
		 u8Res = 2;
	 }
	return u8Res;		
}



/*���Ƽ��*/
static void vLimitProc(void)
{
	 uint8_t SpeedRate = sgUserInfo.u8FastSpd;//Ĭ��Ϊ�����ٶ�
	 if(0 != sgLimitInfo.NoActFlag)
	 {
		 sgValvesInfo.b1ForWardStat = 0;
		 sgValvesInfo.b1BackWardStat = 0;
		 sgValvesInfo.b1LiftUpStat = 0;
		 sgValvesInfo.b1LiftDownStat= 0;
		 sgValvesInfo.b1LeanForWardStat = 0;
		 sgValvesInfo.b1LeanBackWardStat = 0;
	 } 	 
	 if(0 != sgLimitInfo.NoPumpFlag)
	 {
		 sgValvesInfo.b1LiftUpStat = 0;
		 sgValvesInfo.b1LiftDownStat= 0;
		 sgValvesInfo.b1LeanForWardStat = 0;
		 sgValvesInfo.b1LeanBackWardStat = 0;
	 }
	 if((0 != sgLimitInfo.NoMoveFlag)||(0 != sgLimitInfo.ChargeFlag)) //���ʱ��������
	 {
		 sgValvesInfo.b1ForWardStat = 0;
	 	 sgValvesInfo.b1BackWardStat = 0; 
	 }
	 if(0!= sgLimitInfo.NoLeanFlag)
	 {
		 sgValvesInfo.b1LeanForWardStat = 0;
		 sgValvesInfo.b1LeanBackWardStat = 0;
	 }

	 if(0 != sgLimitInfo.NoUpFlag)
	 {
			sgValvesInfo.b1LiftUpStat = 0;
	 }
	 if(0 != sgLimitInfo.NoDownFlag)
	 {
			sgValvesInfo.b1LiftDownStat = 0;
	 }
	 if((0 != sgLimitInfo.OverLean)||(0!= sgLimitInfo.DirectionErr))//�����Ƕȴ���ֻ������ǰ���򷴷����˶�������
	 {
			HornFlick();
	 }
	 if(0 != sgLimitInfo.NoForWordFlag)
	 {
			sgValvesInfo.b1ForWardStat = 0;
	 }
	 if(0 != sgLimitInfo.NoBackWordFlag)
	 {
			sgValvesInfo.b1BackWardStat = 0;
	 }
	 if(0 != sgLimitInfo.LowPowFlag) //�͵��������ٶ�
	 {
			sgValvesInfo.b1LiftUpStat = 0;
			sgValvesInfo.b1LiftDownStat= 0;
			sgValvesInfo.b1LeanForWardStat = 0;
			sgValvesInfo.b1LeanBackWardStat = 0;
			if(SpeedRate > sgUserInfo.u8Gear4Spd)
				SpeedRate = sgUserInfo.u8Gear4Spd;
	 } 
	 
	 if(0 != sgLimitInfo.ReverseSpd)    //����Ϊ�����ٶ�*һ���ٶ�
	 {
			SpeedRate = sgUserInfo.u8FastSpd * sgUserInfo.u8Gear1Spd;
	 }
	 if(0 != sgLimitInfo.UpslopeSpd)
	 {
		 if(SpeedRate > sgUserInfo.u8Gear2Spd)
			 SpeedRate = sgUserInfo.u8Gear2Spd;
	 }
	 if(0 != sgLimitInfo.DownslopeSpd)
	 {
		 if(SpeedRate > sgUserInfo.u8Gear3Spd)
			 SpeedRate = sgUserInfo.u8Gear3Spd;
	 }
		if(0 != sgLimitInfo.TuttleSpd)
	 {
		 if(SpeedRate > sgUserInfo.u8SlowSpd)
			 SpeedRate = sgUserInfo.u8SlowSpd;
	 }
	 u16MotorVal = ((u16MotorVal * SpeedRate)/100);
}
/*******************************************************************************
* Name: void vUserEcuInit(void)
* Descriptio: UserEcu��ʼ������
* Input: NULL
* Output: NULL  
*******************************************************************************/
void vUserEcuInit(void)
{	
	uint16_t u16Tmp = 0;
	
	uint16_t WorkCountL = 0;
	uint16_t WorkCountH = 0;
	vMstRevRegister(vMstRevProc);
	vMstSendRegister(vMstSendProc);
	
//	vCanRevMsgRegister(&CanId42C);
//	vCanRevMsgRegister(&DeviceCanId62C);
//	vCanRevMsgRegister(&PcCanId62C);
	
	vDoPwmErrReg(vDoPwmErrCallBack);
	vPropErrReg(vPropErrCallBack);
	vAiErrReg(vAiErrCallBack);
	
	memset(&sgValvesInfo, 0, sizeof(sgValvesInfo));
	
	sgUserInfo.fPropMinCurrent = i32GetPara(PROP_MIN_CURRENT) / 1000.0;		/*��������С����*/
	sgUserInfo.fPropMaxCurrent = i32GetPara(PROP_MAX_CURRENT) / 1000.0;		/*������������*/
	
	sgUserInfo.u8LeanForWardValue = i32GetPara(LEAN_FORWARD_PARA) * PUMP_RANGE / 100;		/*ǰ�����*/
	sgUserInfo.u8LeanBackWardValue = i32GetPara(LEAN_BACKWARD_PARA) * PUMP_RANGE / 100;		/*�������*/
	
	sgUserInfo.b1LiftMode = i32GetPara(LIFT_MODE_PARA);						/*����ģʽ*/
	
	sgUserInfo.u8FastSpd  = i32GetPara(PARA_FastDriveSpeed) ;	/*�����ٶ�*/
	sgUserInfo.u8SlowSpd  = i32GetPara(PARA_SlowDriveSpeed) ; /*�����ٶ�*/
	
	sgUserInfo.u8Gear1Spd = i32GetPara(PARA_Gear1Spd);	/*1���ٶ�,�����ٶ�*/
	sgUserInfo.u8Gear2Spd = i32GetPara(PARA_Gear2Spd);	/*2���ٶȣ������ٶ�*/
	sgUserInfo.u8Gear3Spd = i32GetPara(PARA_Gear3Spd);	/*3���ٶȣ��͵����ٶ�*/
	sgUserInfo.u8Gear4Spd = i32GetPara(PARA_Gear4Spd);	/*4���ٶ�*/
													 
	sgUserInfo.u16RatioOfTransmission = i32GetPara(PARA_RatioOfTransmission);
	sgUserInfo.u16MaxPresure = i32GetPara(MAX_PRESSURE);  				//�����޸����ѹ��ֵ
	sgUserInfo.b1CalFun = i32GetPara(PARA_WeighFunc);							 //#49  ѹ��������ʹ��
	sgUserInfo.u16MotorMaxSpd = i32GetPara(PARA_MotorMaxSpd);
	sgUserInfo.b1AngleAiLimit = i32GetPara(ANGLEAILIMIT);          // #60  �Ƕ�ģ����λ����
	sgUserInfo.b1TurnDecSpdEn = i32GetPara(TURN_DEC_SPD_ENABLE);   // #70  ת�併�ٹ��ܿ����ر�
	sgUserInfo.b1TileSensorEn = i32GetPara(TILESENSORENABLE);      // #71  ʹ����Ǵ�����
	sgUserInfo.b1TurnDifSpdEn = i32GetPara(TURN_DIF_SPD_ENABLE);    // #116 �����ر�ת�併�ٹ���
	sgUserInfo.b1InterLockEN  = i32GetPara(INTERLOCK_EN);          // #48  �����������ܿ����ر�
	sgUserInfo.u8HourCount = i32GetPara(PARA_HourCountPowerOn);    // #173 ʱ��ѡ��
	sgUserInfo.b1SpeedUnit = i32GetPara(PARA_RemotePara) >> 7 & 1; // #166 bit7ʹ��ΪMPH ��ʹ��ΪKm/h
	sgUserInfo.b1HmiVersion = i32GetPara(PARA_RemotePara)>> 6 & 1; // #166 bit6ʹ�� �Ǳ�汾Ϊ02 ��ʹ��Ĭ��01
	sgUserInfo.b1Language = i32GetPara(PARA_LanguageType);         // #55  ��������
	/* ת��Ƕȴ�����*/
	sgUserInfo.u16TurnAngleMax = i32GetPara(PARA_AngleSimulationUpLimit) ;//#81�Ƕ�ģ������λ
	sgUserInfo.u16TurnAngleMin = i32GetPara(PARA_AngleSimulationDownLimit);//#82�Ƕ�ģ������λ
	
	sgUserInfo.u8LowPow1Step = i32GetPara(LOWPOWER_1STEP);   //#141�����͵���һ������
	sgUserInfo.u8LowPow2Step = i32GetPara(LOWPOWER_2STEP);   //#142�����͵�����������
	
	//һ��Ҫ���ڶ��� ���С�ھͽ���
	uint8_t Tmp;
	if(sgUserInfo.u8LowPow2Step > sgUserInfo.u8LowPow1Step)
	{
		Tmp = sgUserInfo.u8LowPow1Step ;
		sgUserInfo.u8LowPow1Step = sgUserInfo.u8LowPow2Step;
		sgUserInfo.u8LowPow2Step = Tmp;
	}
	
	sgUserInfo.u16TurnAngleMiddle = (sgUserInfo.u16TurnAngleMax + sgUserInfo.u16TurnAngleMin) >> 1;
	sgUserInfo.u16TurnAngleDeadZoon = ((sgUserInfo.u16TurnAngleMax - sgUserInfo.u16TurnAngleMin) * 8)/100;                                //���޺��ʲ������� Ĭ��8%
	
	
	sgUserSet.u8Data = i32GetPara(USER_SET);  						//#115�Ų����޸��ͱõ�������߼�Ŀǰֻ������λ��F->1111��ȫ������
	
	sgUserInfo.u8RampLowestAngle = i32GetPara(PARA_MinAngle);//�ж��������С�Ƕȣ�������Ϊ��ֵ��
	sgUserInfo.u8RampHighestAngle = -1* i32GetPara(PARA_MaxAngle);//�ж����µĽǶȣ�ʹ�������ٶȣ�
	sgUserInfo.u8BatteryType = i32GetPara(PARA_BatteryType);
	// ǰ������������ (Ӧ����λ�����ô����ý����ο�)
	sgUserInfo.s16ForwardLimitAngle = i32GetPara(FORWORD_LIMIT_ANGLE) ; //2340
	sgUserInfo.s16BackwardLimitAngle = i32GetPara(BACKWORD_LIMIT_ANGLE);//2700
	sgUserInfo.s16AutoRampAngle  =-1*i32GetPara(AUTO_RAMP_ANGLE) ;   //   8 
	sgUserInfo.s16SetExitAngle = -1*i32GetPara(SET_EXIT_ANGLE);      //   2
	sgUserInfo.s16SetMaxtileAngle = -1*i32GetPara(SET_MAX_TILE_ANGLE);  //20
	sgUserInfo.s16MaxLeanAngle = i32GetPara(MAX_LEAN_ANGLE);         //20
	sgUserInfo.s16PreleanAngle = i32GetPara(PRELEAN_ANGLE);   //4 Ԥ��б�ǣ��ֶ������µ�ģʽ���Զ���б���������
	sgUserInfo.s16TargetRampAngle = i32GetPara(TARGET_ANGLE);	//4	Ŀ����б�ǣ���Ŀ��Ƕȵ�ƽ��

	/*lilu 20230823 add user mode*/
	{
		u16Tmp = i32GetPara(USERINFO_MODE);
		sgUserInfo.b1HourConutMode = u16Tmp & 0x01;					/*bit0: HourCount Mode*/
		sgUserInfo.b1StartUpLock = (u16Tmp >> 1) & 0x01;			/*bit1: StartUpCheck*/
	}
	
	u32HourCount = u32HourCountRead();
	
	u16EepromRead(PARA_DefaultFlag, &u16Tmp, 1);
	if (0x5555 != u16Tmp)
	{
		u16EepromWrite(PARA_SaveState, sgSaveState.u16Data, 1);
		u16EepromWrite(PARA_HourSetTime, 0x0000, 1);
		u16EepromWrite(PARA_DefaultFlag, 0x5555, 1);	
	}
	else
	{
		u16EepromRead(PARA_SaveState, &sgSaveState.u16Data, 1);
		sgSwiInput.b1HeightLimit = sgSaveState.b1HeightLimit;
		u16EepromRead(PARA_HourSetTime, &u16Tmp, 1);	
	}

//	//i32LogWrite(INFO, "******SaveState = 0x%x\r\n*********", sgSaveState.u16Data);
	
	if (0 != i32GetPara(PARA_AccAndDecFastDrive))
	{
		sgUserInfo.fMoveSpdPer5msAccStep = STEP_FACTOR * i32GetPara(PARA_CurveFastDrive) / i32GetPara(PARA_AccAndDecFastDrive);
		sgUserInfo.fMoveSpdPer5msDecStep = STEP_FACTOR * i32GetPara(PARA_BrakeFastDrive) / i32GetPara(PARA_AccAndDecFastDrive);
	}
	else
	{
		//i32LogWrite(ERR, "Fast Spd Period is 0!!!\r\n");
		sgUserInfo.fMoveSpdPer5msAccStep = 0;
		sgUserInfo.fMoveSpdPer5msDecStep = 0;
	}

	if (0 != i32GetPara(PARA_AccAndDecLift))
	{
		sgUserInfo.fLiftSpdPer5msAccStep = STEP_FACTOR * i32GetPara(PARA_CurveLift) / i32GetPara(PARA_AccAndDecLift);
		sgUserInfo.fLiftSpdPer5msDecStep = STEP_FACTOR * i32GetPara(PARA_BrakeLift) / i32GetPara(PARA_AccAndDecLift);
	}
	else
	{
		//i32LogWrite(ERR, "Lift Spd Period is 0!!!\r\n");
		sgUserInfo.fLiftSpdPer5msAccStep = 0;
		sgUserInfo.fLiftSpdPer5msDecStep = 0;
	}
	
	if (0 != i32GetPara(PARA_AccAndDecLower))
	{
		sgUserInfo.fDownSpdPer5msAccStep = STEP_FACTOR * i32GetPara(PARA_CurveLower) / i32GetPara(PARA_AccAndDecLower);
		sgUserInfo.fDownSpdPer5msDecStep = STEP_FACTOR * i32GetPara(PARA_BrakeLower) / i32GetPara(PARA_AccAndDecLower);
	}
	else
	{
		//i32LogWrite(ERR, "Down Spd Period is 0!!!\r\n");
		sgUserInfo.fDownSpdPer5msAccStep = 0;
		sgUserInfo.fDownSpdPer5msDecStep = 0;
	}
	
	sgUserInfo.SteerAngleDecSpd.u16StartAngle = i32GetPara(TURN_START_ANGLE) * 10;
	sgUserInfo.SteerAngleDecSpd.u16EndAngle = i32GetPara(TURN_END_ANGLE) * 10;
	sgUserInfo.SteerAngleDecSpd.u16StartAngleSpd = i32GetPara(START_ANGLE_SPD) * MOTOR_SPEED_RANGE / 100;
	sgUserInfo.SteerAngleDecSpd.u16EndAngleSpd = i32GetPara(END_ANGLE_SPD) * MOTOR_SPEED_RANGE / 100;
	sgUserInfo.SteerAngleDecSpd.fAgnleDecSpdFactor = (sgUserInfo.SteerAngleDecSpd.u16StartAngleSpd - sgUserInfo.SteerAngleDecSpd.u16EndAngleSpd) /   \
													 (sgUserInfo.SteerAngleDecSpd.u16EndAngle - sgUserInfo.SteerAngleDecSpd.u16StartAngle);
	
	vCanIdLostReg(0x1E0, 1000, vCanLostProc);
	vCanIdLostReg(0x185,1000,vCanLostProc);
	vCanIdLostReg(0x186,1000,vCanLostProc);
	vCanIdLostReg(0x2F0,1000,vCanLostProc);
	
	u16EepromRead(PARA_WorkCountL, &WorkCountL, 1);
	u16EepromRead(PARA_WorkCountH, &WorkCountH, 1);
	u32WorkCount = ((WorkCountH << 16)|WorkCountL);
	
	vSetPdoPara(sgPdoPara);
	
	i32LocalDoSet(DO_ENCODER2, 1);
	
	vSetNetTimer(TIMER_EcuPowerOn, ECU_POWERON_DELAY_TIME);	/**/
	
	vSetNetTimer(TIMER_HourCount, HOURCOUNT_PERIOD);
	u32HourCount = u32HourCountRead();
}

/*******************************************************************************
* Name: void vUserEcuProc(void)
* Descriptio: UserEcu�û�������
* Input: NULL
* Output: NULL  
*******************************************************************************/
void vUserEcuProc(void)
{
	static uint8_t u8EcuProcFlag = 0;
	uint8_t u8ErrCode = 0;
	static uint16_t u16SecCnt = 0;
	static uint16_t u16RentalCnt = 0;

	if(true == u8GetNetTimerOverFlag(TIMER_EcuPowerOn))
	{
		u8EcuProcFlag = 1;
		vKillNetTimer(TIMER_EcuPowerOn);
	}
	//�ϵ�ǰ500ms���
	if ((false == u8GetNetTimerOverFlag(TIMER_EcuPowerOn)) && (true == u8GetNetTimerStartFlag(TIMER_EcuPowerOn)))
	{
		int ret = 0;
		ret = u8SwiInitChcek();
		if (1 == ret)
		{
			sgValvesInfo.b4NoMove |= 1 << NoMove_Init;
			i32ErrCodeSet(ACT_INIT_ERR);
		}
		else if(2 == ret)   //�ϵ�ʱҺѹ�źŴ��ڣ���ֹҺѹ����
		{
			sgValvesInfo.b4NoPump |= (1<<NoPump_Init);
			i32ErrCodeSet(ACT_INIT_ERR);
		}
	}
	i32SetPara(PARA_ErrCode, u8ErrCodeGet());
	if(1 == u8EcuProcFlag)
	{
		vAiMonitor();
		vSwiMonitor();
		vLimitProc();		
		vCanRevPdoProc();
		vHmiSendFun();
		vCanSendPdoProc();
				
		u8ErrCode = u8ErrCodeGet();
		if (0 != u8ErrCode)
		{
			vLedSendAlmCode(u8ErrCode);
		}
		else
		{
//			gCanSendPdoInfo.CanSend23CInfo.u8ErrCode = 0;
		}
	}
	
	if (true == u8GetNetTimerOverFlag(TIMER_HourCount))
	{
		vResetNetTimer(TIMER_HourCount);	
		
		if (1 == sgUserInfo.b1HourConutMode)
		{
			if (1 == sgSwiInput.b1Lock)
			{
				u16SecCnt++;
			}
		}
		else
		{
			u16SecCnt++;
		}
		

		if (u16SecCnt >= 360)			/**/
		{
			u16SecCnt = 0;
			u32HourCount++;
			vHourCountWrite(u32HourCount);
		}

	}
	
	vWdgSetFun(WDG_USER_BIT);
}
#endif 
