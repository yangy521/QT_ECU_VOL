/*******************************************************************************
* Filename: CommonRam.h 	                                    	     		   *
*                                                                              *
* Description: The header file of CommonRam.c.							           *
* Author:                                                                      *
* Date: 															           *
* Revision:															           *
*																	           *
*******************************************************************************/
#ifndef _COMMONRAM_H_
#define _COMMONRAM_H_

#include	"KSDsys.h"
#include "ServoPara.h"
#include "PARA.h"
/******************************************************************************
*�������Ͷ���
******************************************************************************/

/* �����޸Ŀ��ƽṹ */
typedef struct PARA_MODIFY_CTL
{
	INT16S			OldValue;			/* �����ı�ǰ��ֵ */
	INT16S			NewValue;			/* �����ı���ֵ */
	INT16U			ParaNo;				/* ���޸ĵĲ����� */
	INT16U			bVolatile;			/* �Ƿ񱣴���� */
} PARA_MODIFY_CTL;



#define NORMAL_MODE		0
#define SLOW_MODE   	1
#define FAST_MODE			2
#define COAST_MODE			3

/* Common ram */
typedef struct 
{
	/* kernel input data */
	INT32U	PosCmdSrc;				/*pos command source 0-from plc 1-from fxp*/
	INT32S	PosCmd;						/* λ��ָ��: ָ�����λ��(ָ������) */
	INT32S	PosFdb;						/* λ�÷���: ����������λ��(����������) */
	INT32S	SpeedCmd;					/* �ٶ�ָ��: �����ٶȿ���ģʽ(1 / 65536 HZ) */
	INT32U	SpeedMode;				/* 0-Normal; 1-Slow; 2-Fast; Else- Normal*/
	INT32S	BrakeCmd;					/* ɲ��ָ��: Percent 32768 = 100% */
	INT32S	SpdCmdSum;					/* �ٶ�ָ��: �����ٶȿ���ģʽ(1 / 65536 HZ) */
	INT32S	SpeedFdb;					/* �ٶȷ���: �ٶȻ�����ֵ(Q24����ֵ rpm) */
	INT32S	TorqueCmd;				/* ת��ָ��: ����ת�ؿ���ģʽ(1 / 65536 Nm) */
	INT32S	RotorElecAngle;		/* ת�ӵ�Ƕ�: Q24 (2*PI rad) */
	INT32S  CurrentSampleVBus;/*ֱ��ĸ�߲��� Q12����ֵ A*/
	INT32S  TempratureSample;	/*�¶Ȳ��� ��λ 0.1C*/
	INT32S  PropCurrentCmd;   /* Current command for prop driver (1 / 65536 A) */
	INT32S  PropCurrentCmdSum;   /* Current command for prop driver (1 / 65536 A) */
	INT32S  PumpSpeedCmd;			/* Pump cmd speed */
	/* kernel output data */
	INT32S	PosRef;						/* λ�òο�: λ�û�����ֵ(����������) */
	INT32S	PosErr;						/* λ�����: ���������� */
	INT32S  SpeedRef;					/* �ٶȲο�: �ٶȻ�����ֵ(Q24����ֵ rpm) */
	INT32S	SpeedFdbDisp;			/* �ٶȷ�����ʾֵ:  Q24����ֵ rpm (8ms LPF�˲�) */	
	INT32S	CurrentFdbIq;			/* ��������Iqֵ: Q24����ֵ A */
	
	/* ���ϱ������� */
	INT16U	ErrCode;
	INT16U	AlmCode;


	INT8U	AutoCentreDirectionLock;	//Lock the last return direction
	INT8U	AutoCentreDirection;	//0:ֹͣ�Զ����У�1���Զ�������ת��2���Զ�������ת
	INT32S	AutoCentreCmd;		/* ����λ��ָ��: ָ�����λ��(ָ������) */
	INT32S	PosMiddleCmd;		//��λ
	INT32S	PosMiddleFdb;		//��λ
	INT32S	AutoCentreCmdSum;
	INT8U	AutoCentrePhase;	//�Զ����н׶�
	/* semaphore */
	INT16U	SigLamp[SIGNAL_LAMP_NUM];
	
	/* monitor state*/
	INT16S MotorTmp;				//�����ǰ�¶�
	INT16S ControllerTmp;		//��������ǰ�¶�
	INT16U VBusVoltage;			//��ǰĸ�ߵ�ѹ
	INT16U KsiVBusVoltage;	//KSI��ѹ
	INT16S MoveSpeed;				//��ʻ�ٶ�
	INT16S SteerAngle;			//ת��Ƕ�
	
	INT16U TestVoltage;
	/* monitor signal*/
	INT16U ThrottleCmdSpd;	//̤��ָ�%��
	INT16U ThrottleWipVoltage;	//̤�廬���˵�ѹֵ��v��
	INT16U BrakePedalCmdSpd;	//brake ̤��ָ�%��
	INT16U BrakePedalWipVoltage;	//brake ̤�廬���˵�ѹֵ��v��
	INT16U AnologOut;				//ģ���������v��
	INT16U AnalogIn1;				//ģ����1���루v��
	INT16U AnalogIn2;				//ģ����2���루v��
	INT16U AnalogIn3;				//ģ����3���루v��
	INT16U AnalogIn4;				//ģ����4���루v��
	INT16U Switch;					//��������
	INT16U Driver;					//�������
	INT16U KernelState;
	
	/* Debug */
	INT16U	SbdcVoltageCmd;
	INT16U	SbdcRatioRate2;	
} KSD_COMMON_RAM;

/* variables for common ram */
//extern		KSD_COMMON_RAM	gCRam;

/******************************************************************************
*��������
******************************************************************************/
extern void 	InitCommonRam(void);

#endif //_COMMONRAM_H_
