/*******************************************************************************
* Filename: BatteryMeter.h 	                                    	     		   *
*                                                                              *
* Description: The header file of BatteryMeter.c.							           *
* Author:                                                                      *
* Date: 															           *
* Revision:															           *
*																	           *
*******************************************************************************/
#ifndef __BATTERYMETER_H
#define __BATTERYMETER_H

#include "stdint.h"
#include "KSDsys.h"

#ifdef OPENVOLT_SPEC
#define 	OPENVOLT		OPENVOLT_SPEC
#else
#define 	OPENVOLT		_IQ((25.18*STD_VOLTAGE) / (STD_VBUS*24.0))
#endif

#ifdef OPENVOLT_45_SPEC
#define 	OPENVOLT_45		OPENVOLT_45_SPEC
#else
#define 	OPENVOLT_45	_IQ((25.30*STD_VOLTAGE) / (STD_VBUS*24.0))
//#define 	OPENVOLT_45	_IQ((24.50*STD_VOLTAGE) / (STD_VBUS*24.0))
#endif
#ifdef OPENVOLT_80_SPEC
#define OPENVOLT_80	OPENVOLT_80_SPEC
#define VOLTAGE_80 	80
#endif
#ifdef OPENVOLT_60_SPEC
#define OPENVOLT_60	OPENVOLT_60_SPEC
#define VOLTAGE_60 	60
#endif
/******************************************************************************
*�������Ͷ���
******************************************************************************/
/*** Data for BatteryType  ***/
#define YETI_BATTERYTYPE			0
#define JIAOTI_BATTERYTYPE		1

#define ENABLE_BATTERY		0
#define DISABLE_BATTERY		1

#define	BATTERY_PERIOD			500
#define BATTERY_INIT_PERIOD 500

typedef struct _tBATTERY_METER
{
	INT8U       gCurLevel;                  // ��ǰ��λ
	INT8U      gDownLevel;                  // ��ѹ���޵�λ
	INT8U			BatteryMode;
	INT8U			BatteryType;  //0--yeti; 1--jiaoti; 2-- 

	_iq       gCurVoltage;              // ��ǰ��ѹ
	_iq       gDownVoltage;                // ��ѹ����
	INT16U   gDownCount;                  // ��ѹ�ﵽ���޴���

	_iq      gChargeVoltage;                // ����ѹ����
	INT16U      gChargeCount;                  // ����ѹ�ﵽ���޴���
	INT16U      gChargeFlag;                  // ��� flag
	
	INT8U				gLevelFine;								//ÿ5%�м��ϸ�ֵ�λ ��Χ��0-4
	_iq				gDiffVoltage1Per;						//����1%��Ӧ�ĵ�ѹ��ֵ
	_iq       gUpVoltage;                // ��ѹ����
	INT16U   gDownCountFine;              //����1%��ѹ�ﵽ���޴���
	
} _tBATTERY_METER;

//extern _tBATTERY_METER BatteryMeter;
/******************************************************************************
*��������
******************************************************************************/

extern void InitBattery(void);
extern void vBatteryProc(void);
extern uint8_t u8GetBatterySoc(void);
extern uint8_t u8SocDisableFlag;
extern void vSetSocFlag(uint8_t u8Flag);

//tBoolean BatteryDataInManage(void);
//void BatteryManage(void);
//void BatteryDataOutManage(void);

#endif //__BATTERYMETER_H

