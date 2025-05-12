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
*数据类型定义
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
	INT8U       gCurLevel;                  // 当前档位
	INT8U      gDownLevel;                  // 电压下限档位
	INT8U			BatteryMode;
	INT8U			BatteryType;  //0--yeti; 1--jiaoti; 2-- 

	_iq       gCurVoltage;              // 当前电压
	_iq       gDownVoltage;                // 电压下限
	INT16U   gDownCount;                  // 电压达到下限次数

	_iq      gChargeVoltage;                // 充电电压下限
	INT16U      gChargeCount;                  // 充电电压达到下限次数
	INT16U      gChargeFlag;                  // 充电 flag
	
	INT8U				gLevelFine;								//每5%中间的细分档位 范围：0-4
	_iq				gDiffVoltage1Per;						//电量1%对应的电压差值
	_iq       gUpVoltage;                // 电压上限
	INT16U   gDownCountFine;              //电量1%电压达到下限次数
	
} _tBATTERY_METER;

//extern _tBATTERY_METER BatteryMeter;
/******************************************************************************
*函数定义
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

