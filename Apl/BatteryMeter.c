/*******************************************************************************
* Filename: BatteryMeter.c                                                     *
*                                                                    		   *
* Description: 							            				 		   *
* Author:                                                                 	   *
* Date: 20190929														       *
* Revision:															 		   *
*																	 		   *
*******************************************************************************/

#include "KSDsys.h"
#include "Device.h"
#include "Eeprom.h"
#include "Para.h"
#include "LocalAi.h"
#include "WdgProc.h"
#include "NetTimer.h"
#include "BatteryMeter.h"
#include "Log.h"

#if (BATERRYMETER_TYPE == BATERRYMETER_ENABLE)

uint8_t u8SocDisableFlag = 0;	//23.11.21 SJ添加禁止计电量标志位

_tBATTERY_METER BatteryMeter;
	
#define PARANO_BATTERY1_FLAG		PARA_NoBatteryFlag1
#define PARANO_BATTERY1				PARA_NoBatteryCount1
#define PARANO_BATTERY2_FLAG		PARA_NoBatteryFlag2
#define PARANO_BATTERY2				PARA_NoBatteryCount2

//ye ti bat
#ifdef BATERRY_VOLTAGE_LOW6V
const _iq yetiLoadVol24_48_72[20] = 
	{ _IQ(24.805/STD_VBUS), _IQ(24.325/STD_VBUS), _IQ(23.805/STD_VBUS), _IQ(23.005/STD_VBUS), _IQ(22.704/STD_VBUS), 
		_IQ(22.535/STD_VBUS), _IQ(22.355/STD_VBUS), _IQ(22.232/STD_VBUS), _IQ(22.051/STD_VBUS), _IQ(21.794/STD_VBUS), 
	  _IQ(21.606/STD_VBUS), _IQ(21.408/STD_VBUS), _IQ(21.141/STD_VBUS), _IQ(20.925/STD_VBUS), _IQ(20.713/STD_VBUS), 
	  _IQ(20.486/STD_VBUS), _IQ(20.249/STD_VBUS), _IQ(19.904/STD_VBUS), _IQ(19.732/STD_VBUS), _IQ(19.585/STD_VBUS)
	};
#elif defined(BATERRY_VOLTAGE_LOW12V)
	const _iq yetiLoadVol24_48_72[20] = 
		{ _IQ(24.805/STD_VBUS), _IQ(24.325/STD_VBUS), _IQ(23.705/STD_VBUS), _IQ(23.005/STD_VBUS), _IQ(22.704/STD_VBUS), 
			_IQ(22.535/STD_VBUS), _IQ(22.355/STD_VBUS), _IQ(22.232/STD_VBUS), _IQ(22.051/STD_VBUS), _IQ(21.794/STD_VBUS), 
			_IQ(21.606/STD_VBUS), _IQ(21.408/STD_VBUS), _IQ(21.241/STD_VBUS), _IQ(21.025/STD_VBUS), _IQ(20.813/STD_VBUS), 
			_IQ(20.686/STD_VBUS), _IQ(20.486/STD_VBUS), _IQ(20.370/STD_VBUS), _IQ(20.269/STD_VBUS), _IQ(20.200/STD_VBUS),
		};
#elif defined(BATERRY_VOLTAGE_JIAOTI)
	const _iq yetiLoadVol24_48_72[20] = 
		{	_IQ(26.100/STD_VBUS), _IQ(25.900/STD_VBUS), _IQ(25.750/STD_VBUS), _IQ(25.600/STD_VBUS), _IQ(25.450/STD_VBUS), 
			_IQ(25.300/STD_VBUS), _IQ(25.150/STD_VBUS), _IQ(25.000/STD_VBUS), _IQ(24.850/STD_VBUS), _IQ(24.700/STD_VBUS),
			_IQ(24.550/STD_VBUS), _IQ(24.350/STD_VBUS), _IQ(24.150/STD_VBUS), _IQ(24.000/STD_VBUS), _IQ(23.850/STD_VBUS), 
			_IQ(23.700/STD_VBUS), _IQ(23.600/STD_VBUS), _IQ(23.500/STD_VBUS), _IQ(23.400/STD_VBUS), _IQ(23.300/STD_VBUS)
		};
#elif defined(BATERRY_VOLTAGE_JIAOTI_DISCOV)
	const _iq yetiLoadVol24_48_72[20] = 
        { _IQ(25.950/STD_VBUS), _IQ(24.865/STD_VBUS), _IQ(24.375/STD_VBUS), _IQ(24.100/STD_VBUS), _IQ(23.800/STD_VBUS), 
          _IQ(23.450/STD_VBUS), _IQ(23.250/STD_VBUS), _IQ(23.100/STD_VBUS), _IQ(22.950/STD_VBUS), _IQ(22.650/STD_VBUS), 
          _IQ(22.450/STD_VBUS), _IQ(22.250/STD_VBUS), _IQ(21.850/STD_VBUS), _IQ(21.400/STD_VBUS), _IQ(20.900/STD_VBUS), 
          _IQ(20.400/STD_VBUS), _IQ(19.800/STD_VBUS), _IQ(19.400/STD_VBUS), _IQ(19.100/STD_VBUS), _IQ(18.800/STD_VBUS)
        };
#elif defined(BATERRY_VOLTAGE_SANYI_SPA)
const _iq yetiLoadVol24_48_72[20] = 
        { _IQ(25.500/STD_VBUS), _IQ(24.450/STD_VBUS), _IQ(24.400/STD_VBUS), _IQ(24.300/STD_VBUS), _IQ(24.200/STD_VBUS), 
          _IQ(24.050/STD_VBUS), _IQ(23.900/STD_VBUS), _IQ(23.750/STD_VBUS), _IQ(23.600/STD_VBUS), _IQ(23.450/STD_VBUS), 
          _IQ(23.300/STD_VBUS), _IQ(23.100/STD_VBUS), _IQ(22.900/STD_VBUS), _IQ(22.700/STD_VBUS), _IQ(22.500/STD_VBUS), 
          _IQ(22.300/STD_VBUS), _IQ(22.100/STD_VBUS), _IQ(22.000/STD_VBUS), _IQ(21.900/STD_VBUS), _IQ(21.800/STD_VBUS)
        };
#elif defined(BATERRY_VOLTAGE_KUNYU)
	const _iq yetiLoadVol24_48_72[20] = 
		{ _IQ(25.505/STD_VBUS), _IQ(25.417/STD_VBUS), _IQ(25.333/STD_VBUS), _IQ(25.250/STD_VBUS), _IQ(25.167/STD_VBUS), 
			_IQ(25.083/STD_VBUS), _IQ(25.000/STD_VBUS), _IQ(24.875/STD_VBUS), _IQ(24.750/STD_VBUS), _IQ(24.625/STD_VBUS), 
		  _IQ(24.500/STD_VBUS), _IQ(24.375/STD_VBUS), _IQ(24.250/STD_VBUS), _IQ(24.125/STD_VBUS), _IQ(24.000/STD_VBUS), 
		  _IQ(23.833/STD_VBUS), _IQ(23.667/STD_VBUS), _IQ(23.500/STD_VBUS), _IQ(23.250/STD_VBUS), _IQ(23.000/STD_VBUS)
		};
#elif	defined(BATERRY_VOLTAGE_RUYI)
	const _iq yetiLoadVol24_48_72[20] = 
		{ _IQ(24.805/STD_VBUS), _IQ(24.525/STD_VBUS), _IQ(24.361/STD_VBUS), _IQ(24.198/STD_VBUS), _IQ(24.035/STD_VBUS), 
			_IQ(23.872/STD_VBUS), _IQ(23.709/STD_VBUS), _IQ(23.545/STD_VBUS), _IQ(23.381/STD_VBUS), _IQ(23.218/STD_VBUS), 
		  _IQ(23.054/STD_VBUS), _IQ(22.891/STD_VBUS), _IQ(22.728/STD_VBUS), _IQ(22.565/STD_VBUS), _IQ(22.401/STD_VBUS), 
		  _IQ(22.238/STD_VBUS), _IQ(22.275/STD_VBUS), _IQ(21.904/STD_VBUS), _IQ(21.732/STD_VBUS), _IQ(21.585/STD_VBUS)
		};
#elif	defined(BATERRY_VOLTAGE_XUGONG)//80%24.7 65%24 50%23.2 30%22.2 15%20.7
	const _iq yetiLoadVol24_48_72[20] = 
		{ _IQ(25.400/STD_VBUS), _IQ(25.290/STD_VBUS), _IQ(25.110/STD_VBUS), _IQ(24.942/STD_VBUS), _IQ(24.707/STD_VBUS), 
			_IQ(24.471/STD_VBUS), _IQ(24.235/STD_VBUS), _IQ(24.000/STD_VBUS), _IQ(23.733/STD_VBUS), _IQ(23.466/STD_VBUS), 
		  _IQ(23.200/STD_VBUS), _IQ(22.950/STD_VBUS), _IQ(22.700/STD_VBUS), _IQ(22.450/STD_VBUS), _IQ(22.200/STD_VBUS), 
		  _IQ(21.700/STD_VBUS), _IQ(21.200/STD_VBUS), _IQ(20.700/STD_VBUS), _IQ(20.500/STD_VBUS), _IQ(20.300/STD_VBUS)
		};		
#elif	defined(BATERRY_VOLTAGE_LIUGONG)//80%24 65%23.5 50%22.7 30%21.8 15%20.8  10% 19.8v
	const _iq yetiLoadVol24_48_72[20] = 
		{ _IQ(25.400/STD_VBUS), _IQ(25.050/STD_VBUS), _IQ(24.700/STD_VBUS), _IQ(24.350/STD_VBUS), _IQ(24.000/STD_VBUS), 
			_IQ(23.883/STD_VBUS), _IQ(23.667/STD_VBUS), _IQ(23.500/STD_VBUS), _IQ(23.100/STD_VBUS), _IQ(22.722/STD_VBUS), 
		  _IQ(22.475/STD_VBUS), _IQ(22.250/STD_VBUS), _IQ(22.025/STD_VBUS), _IQ(21.800/STD_VBUS), _IQ(21.466/STD_VBUS), 
		  _IQ(21.130/STD_VBUS), _IQ(20.800/STD_VBUS), _IQ(20.400/STD_VBUS), _IQ(19.800/STD_VBUS), _IQ(19.500/STD_VBUS)
		};
#else //standard yeti bat

#ifndef JINJIDAO_BATTERTYPE
	const _iq yetiLoadVol24_48_72[20] = 
		{ _IQ(24.500/STD_VBUS), _IQ(24.248/STD_VBUS), _IQ(23.996/STD_VBUS), _IQ(23.744/STD_VBUS), _IQ(23.490/STD_VBUS), 
			_IQ(23.348/STD_VBUS), _IQ(23.206/STD_VBUS), _IQ(23.064/STD_VBUS), _IQ(22.920/STD_VBUS), _IQ(22.743/STD_VBUS), 
		  _IQ(22.566/STD_VBUS), _IQ(22.389/STD_VBUS), _IQ(22.210/STD_VBUS), _IQ(21.970/STD_VBUS), _IQ(21.730/STD_VBUS), 
		  _IQ(21.490/STD_VBUS), _IQ(21.249/STD_VBUS), _IQ(20.904/STD_VBUS), _IQ(20.732/STD_VBUS), _IQ(20.585/STD_VBUS)
		};
#else
	const _iq yetiLoadVol24_48_72[20] = 
			{ _IQ(25.500/STD_VBUS), _IQ(25.350/STD_VBUS), _IQ(25.200/STD_VBUS), _IQ(25.050/STD_VBUS), _IQ(24.900/STD_VBUS), 
				_IQ(24.750/STD_VBUS), _IQ(24.600/STD_VBUS), _IQ(24.450/STD_VBUS), _IQ(24.300/STD_VBUS), _IQ(24.150/STD_VBUS), 
				_IQ(24.000/STD_VBUS), _IQ(23.850/STD_VBUS), _IQ(23.700/STD_VBUS), _IQ(23.550/STD_VBUS), _IQ(23.400/STD_VBUS), 
				_IQ(23.250/STD_VBUS), _IQ(23.100/STD_VBUS), _IQ(22.950/STD_VBUS), _IQ(22.800/STD_VBUS), _IQ(22.650/STD_VBUS)
			};
#endif
#endif
//jiao ti bat
const _iq jiaotiLoadVol24_48_72[20] = 
	{	_IQ(25.105/STD_VBUS), _IQ(25.025/STD_VBUS), _IQ(24.805/STD_VBUS), _IQ(24.525/STD_VBUS), _IQ(24.205/STD_VBUS), 
		_IQ(23.985/STD_VBUS), _IQ(23.754/STD_VBUS), _IQ(23.465/STD_VBUS), _IQ(23.225/STD_VBUS), _IQ(22.962/STD_VBUS),
		_IQ(22.771/STD_VBUS), _IQ(22.514/STD_VBUS), _IQ(22.326/STD_VBUS), _IQ(22.128/STD_VBUS), _IQ(21.931/STD_VBUS), 
		_IQ(21.613/STD_VBUS), _IQ(21.249/STD_VBUS), _IQ(20.904/STD_VBUS), _IQ(20.732/STD_VBUS), _IQ(20.585/STD_VBUS)
	};

	//Charge for all
	#define CHARGE_TIME_5PENCENT  228
	#define CHARGE_TIME_5PENCENT_S  (228 - 100)
const float stdChargeVol[20] = 
	{	_IQ(28.150/STD_VBUS), _IQ(28.100/STD_VBUS), _IQ(27.900/STD_VBUS), _IQ(27.700/STD_VBUS), _IQ(27.500/STD_VBUS), 
		_IQ(27.300/STD_VBUS), _IQ(27.100/STD_VBUS), _IQ(26.900/STD_VBUS), _IQ(26.700/STD_VBUS), _IQ(26.500/STD_VBUS),
		_IQ(26.300/STD_VBUS), _IQ(26.100/STD_VBUS), _IQ(25.900/STD_VBUS), _IQ(25.700/STD_VBUS), _IQ(25.500/STD_VBUS), 
		_IQ(25.300/STD_VBUS), _IQ(25.100/STD_VBUS), _IQ(24.900/STD_VBUS), _IQ(24.700/STD_VBUS), _IQ(24.500/STD_VBUS)
	};


static uint8_t sgU8BatteryType = 0;
// 读取电池电压
_iq ReadVoltage(void)
{
	return _IQ12toIQ(i32LocalAiGet(AI_B_KSI_CHECK));
}	

// 读取FLASH电量
INT8U ReadLevelFromEeprom(void)
{
	INT16U dataL;
	INT16U	BatteryFlag1, BatteryFlag2;
	INT16U	Battery1, Battery2;
	INT8U		Battery;
	//No1
	//if (EepromQualifiedRead(PARANO_BATTERY1_FLAG, &BatteryFlag1) != 0)
	if (0 != u16EepromRead(PARANO_BATTERY1_FLAG, &BatteryFlag1, 1))
	{
		BatteryFlag1 = BATTERY_HOUR_INVALID;
	}
	if (BatteryFlag1 == BATTERY_HOUR_VALID)
	{
		//if (EepromQualifiedRead(PARANO_BATTERY1, &dataL) == 0)
		if (0 == u16EepromRead(PARANO_BATTERY1, &dataL, 1))
		{
			Battery1 = dataL;
		}
		else
		{
			Battery1 = 100;
		}
	}
	else
	{
		Battery1 = 100;
	}
	//No2
	//if (EepromQualifiedRead(PARANO_BATTERY2_FLAG, &BatteryFlag2) != 0)
	if (0 != u16EepromRead(PARANO_BATTERY2_FLAG, &BatteryFlag2, 1))
	{
		BatteryFlag2 = BATTERY_HOUR_INVALID;
	}
	if (BatteryFlag2 == BATTERY_HOUR_VALID)
	{
		//if (EepromQualifiedRead(PARANO_BATTERY2, &dataL) == 0)
		if (0 == u16EepromRead(PARANO_BATTERY2, &dataL, 1))
		{
			Battery2 = dataL;
		}
		else
		{
			Battery2 = 100;
		}
	}
	else
	{
		Battery2 = 100;
	}
	//Adopt small Battery
	if (Battery1 > Battery2)
	{
		Battery1 = Battery2;
	}

	Battery = (Battery1 * 5)/5;
	if (Battery > 100)
	{
		Battery = 100;
	}
	if (Battery < 5)
	{
		Battery = 5;
	}

	return Battery;
}	

// 写入FLASH电量
void WriteLevelToEeprom(INT8U level)
{
	INT16U dataL;
	INT16U	BatteryFlag1, BatteryFlag2;
	INT16U	Battery1, Battery2;
	INT16U Battery;
	Battery = level;
	//No1
	//if (EepromQualifiedRead(PARANO_BATTERY1_FLAG, &BatteryFlag1) != 0)
	if (0 != u16EepromRead(PARANO_BATTERY1_FLAG, &BatteryFlag1, 1))
	{
		BatteryFlag1 = BATTERY_HOUR_INVALID;
	}
	if (BatteryFlag1 == BATTERY_HOUR_VALID)
	{
		//if (EepromQualifiedRead(PARANO_BATTERY1, &dataL) == 0)
		if (0 == u16EepromRead(PARANO_BATTERY1, &dataL, 1))
		{
			Battery1 = dataL;
		}
		else
		{
			Battery1 = 100;
		}
	}
	else
	{
		Battery1 = 100;
	}
	if ((BatteryFlag1 != BATTERY_HOUR_VALID) ||(Battery != Battery1))
	{

//		u16EepromWrite(PARANO_BATTERY1_FLAG, BATTERY_HOUR_INVALID, 1);
//		u16EepromWrite(PARANO_BATTERY1, Battery, 1);
//		u16EepromWrite(PARANO_BATTERY1_FLAG, BATTERY_HOUR_VALID, 1);
		u16EepromWrite(PARANO_BATTERY1_FLAG, BATTERY_HOUR_INVALID, 2);
		u16EepromWrite(PARANO_BATTERY1, Battery, 2);
		u16EepromWrite(PARANO_BATTERY1_FLAG, BATTERY_HOUR_VALID, 2);
	
//		if (0 == u16EepromWrite(PARANO_BATTERY1_FLAG, BATTERY_HOUR_INVALID, 1))
//		{
//			//if (EepromQualifiedWrite(PARANO_BATTERY1, Battery) == 0)
//			if (0 == u16EepromWrite(PARANO_BATTERY1, Battery, 1))
//			{
//				//EepromQualifiedWrite(PARANO_BATTERY1_FLAG, BATTERY_HOUR_VALID);
//				u16EepromWrite(PARANO_BATTERY1_FLAG, BATTERY_HOUR_VALID, 1);
//				//i32LogWrite(DEBUG, "Write Eeprom Battery1 Soc = %d**************************\r\n", Battery);
//			}
//		}	
	}
	//No2
	//if (EepromQualifiedRead(PARANO_BATTERY2_FLAG, &BatteryFlag2) != 0)
	if (0 != u16EepromRead(PARANO_BATTERY2_FLAG, &BatteryFlag2, 1))
	{
		BatteryFlag2 = BATTERY_HOUR_INVALID;
	}
	if (BatteryFlag2 == BATTERY_HOUR_VALID)
	{
		//if (EepromQualifiedRead(PARANO_BATTERY2, &dataL) == 0)
		if (0 == u16EepromRead(PARANO_BATTERY2, &dataL, 1))
		{
			Battery2 = dataL;
		}
		else
		{
			Battery2 = 100;
		}
	}
	else
	{
		Battery2 = 100;
	}
	if ((BatteryFlag2 != BATTERY_HOUR_VALID) ||(Battery != Battery2))
	{
//		u16EepromWrite(PARANO_BATTERY2_FLAG, BATTERY_HOUR_INVALID, 1);
//		u16EepromWrite(PARANO_BATTERY2, Battery, 1);
//		u16EepromWrite(PARANO_BATTERY2_FLAG, BATTERY_HOUR_VALID, 1);
		u16EepromWrite(PARANO_BATTERY2_FLAG, BATTERY_HOUR_INVALID, 2);
		u16EepromWrite(PARANO_BATTERY2, Battery, 2);
		u16EepromWrite(PARANO_BATTERY2_FLAG, BATTERY_HOUR_VALID, 2);
		
//	if (EepromQualifiedWrite(PARANO_BATTERY2_FLAG, BATTERY_HOUR_INVALID) == 0)
//		if (0 == u16EepromWrite(PARANO_BATTERY2_FLAG, BATTERY_HOUR_INVALID, 1))
//		{
//			//if (EepromQualifiedWrite(PARANO_BATTERY2, Battery) == 0)
//			if (0 == u16EepromWrite(PARANO_BATTERY2, Battery, 1) )
//			{
//				//EepromQualifiedWrite(PARANO_BATTERY2_FLAG, BATTERY_HOUR_VALID);
//				u16EepromWrite(PARANO_BATTERY2_FLAG, BATTERY_HOUR_VALID, 1);
//				//i32LogWrite(DEBUG, "Write Eeprom Battery2 Soc = %d**************************\r\n", Battery);
//			}
//		}
	}
}


// 整数档5 10 15 20 ... 100转换为负载电压
_iq Level2LoadVoltage(INT8U level)
{
	_iq voltage;
  INT8U idx;

	if (level > 100)
		level = 100;
	if (level < 5)
		level = 5;


	if (BatteryMeter.BatteryType == JIAOTI_BATTERYTYPE)
	{
		idx = (sizeof(jiaotiLoadVol24_48_72)/sizeof(jiaotiLoadVol24_48_72[0])) - (level/5);
		voltage = _IQmpy(jiaotiLoadVol24_48_72[idx], _IQ(STD_VOLTAGE/24.0));
	}
	else //Default YETI_BATTERYTYPE
	{
		idx = (sizeof(yetiLoadVol24_48_72)/sizeof(yetiLoadVol24_48_72[0])) - (level/5);
		voltage = _IQmpy(yetiLoadVol24_48_72[idx], _IQ(STD_VOLTAGE/24.0));
	}
	return voltage;
}

// 整数档5 10 15 20 ... 100转换为充电电压
_iq Level2ChargeVoltage(INT8U level)
{
	_iq voltage;
  INT8U idx;

	if (level > 100)
		level = 100;
	if (level < 5)
		level = 5;
	idx = (sizeof(stdChargeVol)/sizeof(stdChargeVol[0])) - (level/5);

	voltage = _IQmpy(stdChargeVol[idx], _IQ(STD_VOLTAGE/24.0));

	return voltage;
}

static void vBatInitDelayms(uint32_t u32Delay)
{
	uint32_t i = 0, j = 0;
	for (i=0; i<u32Delay; i++)
	{
		for (j=0; j<10000; j++)
		{
			__NOP();
		}
	}
}
// 开机电池电量检测初始化
void InitBattery(void)
{
	_iq voltage;
	uint16_t u8BatteryPct100Volt = 0;
	uint16_t u8BatteryPctResetVolt1 = 0;
	
	sgU8BatteryType = i32GetPara(PARA_BatteryType);
	u8BatteryPct100Volt = i32GetPara(PARA_AngleValue5);
	u8BatteryPctResetVolt1 = i32GetPara(PARA_AngleValue6);
	
	if(u8BatteryPct100Volt < 24000)
	{
		u8BatteryPct100Volt = 24000;
		u16EepromWrite(PARA_AngleValue5, u8BatteryPct100Volt, 1);
	}

	if (LiBattery != sgU8BatteryType)
	{
		BatteryMeter.BatteryType = YETI_BATTERYTYPE;
		BatteryMeter.BatteryMode = VOLTAGE_LEVEL;
		BatteryMeter.gCurLevel = ReadLevelFromEeprom();
		
//		vSetNetTimer(TIMER_BatteryInit, BATTERY_INIT_PERIOD);
		vBatInitDelayms(500);
//		vSetNetTimer(TIMER_BatteryInit, 500);
//		while(false == u8GetNetTimerOverFlag(TIMER_BatteryInit))
//		{
//			;
//		}
//		vKillNetTimer(TIMER_BatteryInit);
		//while (gPLCCtl.PlcCount < 100){}; //Delay 0.5s
		// 电量为零, 没有查询到电量信息, 则直接查询开路电压
		if (BatteryMeter.gCurLevel == 0)
		{
			BatteryMeter.gDownLevel = 0;
		}
		else	// 电量不为零, 判断是否充电
		{
			// 读取电压
			//voltage = ReadVoltage();
			voltage = _IQ12toIQ(i32LocalAiGet(AI_B_KSI_CHECK));
			i32LogWrite(ERR, LOG_BAT, "voltage = %d, Openvolt = %d, openvolt_45 = %d\r\n", voltage, OPENVOLT, OPENVOLT_45);
			// 开路电压转换为档位
#ifdef BATERRY_VOLTAGE_CONFIG_IN_PARA
			if(BatteryMeter.gCurLevel < 75)
#endif
			{
#ifdef BATERRY_VOLTAGE_CONFIG_IN_PARA
				if(voltage > _IQ(((u8BatteryPct100Volt+u8BatteryPctResetVolt1)*STD_VOLTAGE) / (STD_VBUS*24000)))
#else
				if(voltage > OPENVOLT)
#endif
				{
					#ifdef	OPENVOLT_80	//23.11.21 SJ添加80%以上复位电压挡位
					if (voltage >= OPENVOLT_80)//大于29v直接复位
					{
						BatteryMeter.gCurLevel = 100;
					}
					#ifdef OPENVOLT_60
					else if (voltage >= OPENVOLT_60)//大于25.7	小于29v，电量小于80复位
					{
						if(BatteryMeter.gCurLevel <= VOLTAGE_80)
						{
							BatteryMeter.gCurLevel = 100;
						}
					}
					else if (voltage >= OPENVOLT_45)//大于25.3	小于25.7v，电量小于60复位
					{
						if(BatteryMeter.gCurLevel <= VOLTAGE_60)
						{
							BatteryMeter.gCurLevel = 100;
						}
					}
					#else
					else if (voltage >= OPENVOLT_45)//大于25.3	小于25.7v，电量小于80复位
					{
						if(BatteryMeter.gCurLevel <= VOLTAGE_80)
						{
							BatteryMeter.gCurLevel = 100;
						}
					}					
					#endif
					#else
#ifdef BATERRY_VOLTAGE_CONFIG_IN_PARA
					if (voltage >= _IQ(((u8BatteryPct100Volt + u8BatteryPctResetVolt1+200)*STD_VOLTAGE) / (STD_VBUS*24000)))
#else
					if (voltage >= OPENVOLT_45)
#endif
					{
						BatteryMeter.gCurLevel = 100;
					}
					#endif
					else//大于25.18 小于25.3，电量25以下复位
					{
						if (BatteryMeter.gCurLevel <= (BATTERY_ALARM_RATIO+10))//25以下
							BatteryMeter.gCurLevel = 100;
					}
				}
				else if (voltage > Level2LoadVoltage(100))
				{
					if (BatteryMeter.gCurLevel <= BATTERY_PROTECT_RATIO)
						BatteryMeter.gCurLevel = (BATTERY_PROTECT_RATIO + 5);	
				}
			}
			BatteryMeter.gDownLevel = BatteryMeter.gCurLevel - 5;
		}
		// 确定下降档位
		BatteryMeter.gDownCount = 0;
		BatteryMeter.gDownVoltage = Level2LoadVoltage(BatteryMeter.gDownLevel);	
		
		BatteryMeter.gChargeCount = CHARGE_TIME_5PENCENT_S;
		BatteryMeter.gChargeVoltage = Level2ChargeVoltage(BatteryMeter.gCurLevel);

		BatteryMeter.gLevelFine = 0;
		BatteryMeter.gDownCountFine = 0;	
		
		WriteLevelToEeprom(BatteryMeter.gCurLevel);
	}
}

// 输入数据更新	
tBoolean BatteryDataInManage(void)
{
	// 读取电压
	BatteryMeter.gCurVoltage = ReadVoltage();
	return TRUE;
}

// 逻辑处理
void BatteryManage(void)
{  
	BatteryMeter.gChargeFlag = 0;                                    // 如果电量大于80, 由之前的计时方式变为计数
	if (BatteryMeter.gCurLevel > 80)
	{
		if((BatteryMeter.gCurVoltage < BatteryMeter.gDownVoltage)
		#ifdef	SOC_IN_1PERCENT
			&&(4 == BatteryMeter.gLevelFine)//23.11.21 SJ 避免1%计电量时，电量优先触发5%降挡，导致电量跳变
		#endif
			)
		{
			BatteryMeter.gDownCount++;
		}
		else if (BatteryMeter.gCurVoltage > BatteryMeter.gChargeVoltage)
		{
			BatteryMeter.gChargeCount++;
			BatteryMeter.gChargeFlag = 1;
		}
		// 降档
		if (BatteryMeter.gDownCount >= 275)
		{
				BatteryMeter.gDownCount = 0;
				BatteryMeter.gChargeCount = CHARGE_TIME_5PENCENT_S;
				BatteryMeter.gCurLevel = BatteryMeter.gCurLevel - 5;
				BatteryMeter.gDownLevel = BatteryMeter.gCurLevel - 5;
				BatteryMeter.gLevelFine = 0;
				BatteryMeter.gDownCountFine = 0;
		}	
		else if (BatteryMeter.gChargeCount >= CHARGE_TIME_5PENCENT)
		{
			BatteryMeter.gChargeCount = 0;
			BatteryMeter.gDownCount = 0;
			
			BatteryMeter.gCurLevel = BatteryMeter.gCurLevel + 5;
			if (BatteryMeter.gCurLevel > 100)
				BatteryMeter.gCurLevel = 100;
			
			BatteryMeter.gDownLevel = BatteryMeter.gCurLevel - 5;
			BatteryMeter.gLevelFine = 0;
		}

		BatteryMeter.gDownVoltage = Level2LoadVoltage(BatteryMeter.gDownLevel);
		BatteryMeter.gChargeVoltage = Level2ChargeVoltage(BatteryMeter.gCurLevel);	
	}
	else if((BatteryMeter.gCurLevel >= 50) && (BatteryMeter.gCurLevel <=80))
	{
		// 计数
		if((BatteryMeter.gCurVoltage < BatteryMeter.gDownVoltage)
		#ifdef	SOC_IN_1PERCENT//23.11.21 SJ 避免1%计电量时，电量优先触发5%降挡，导致电量跳变
			&&(4 == BatteryMeter.gLevelFine)
		#endif	
		)
		{
			BatteryMeter.gDownCount++;
		}
		else if (BatteryMeter.gCurVoltage > BatteryMeter.gChargeVoltage)
		{
			BatteryMeter.gChargeCount++;
			BatteryMeter.gChargeFlag = 1;
		}
		// 降档
		if (BatteryMeter.gDownCount >= 225)
		{
			BatteryMeter.gDownCount = 0;
			BatteryMeter.gChargeCount = CHARGE_TIME_5PENCENT_S;
			BatteryMeter.gCurLevel = BatteryMeter.gCurLevel - 5;
			BatteryMeter.gDownLevel = BatteryMeter.gCurLevel - 5;
			BatteryMeter.gLevelFine = 0;
			BatteryMeter.gDownCountFine = 0;
		}		
		else if (BatteryMeter.gChargeCount >= CHARGE_TIME_5PENCENT)
		{
			BatteryMeter.gChargeCount = 0;
			BatteryMeter.gDownCount = 0;
			BatteryMeter.gCurLevel = BatteryMeter.gCurLevel + 5;
			BatteryMeter.gDownLevel = BatteryMeter.gCurLevel - 5;
			BatteryMeter.gLevelFine = 0;
		}
		BatteryMeter.gDownVoltage = Level2LoadVoltage(BatteryMeter.gDownLevel);
		BatteryMeter.gChargeVoltage = Level2ChargeVoltage(BatteryMeter.gCurLevel);
	}
	else if((BatteryMeter.gCurLevel < 50) && (BatteryMeter.gCurLevel >=30))
	{
		// 计数
		if((BatteryMeter.gCurVoltage < BatteryMeter.gDownVoltage)
		#ifdef	SOC_IN_1PERCENT//23.11.21 SJ 避免1%计电量时，电量优先触发5%降挡，导致电量跳变
			&&(4 == BatteryMeter.gLevelFine)
		#endif
			)
		{
			BatteryMeter.gDownCount++;
		}
		else if (BatteryMeter.gCurVoltage > BatteryMeter.gChargeVoltage)
		{
			BatteryMeter.gChargeCount++;
			BatteryMeter.gChargeFlag = 1;
		}
		// 降档
		if (BatteryMeter.gDownCount >= 160)
		{
			BatteryMeter.gDownCount = 0;
			BatteryMeter.gChargeCount = CHARGE_TIME_5PENCENT_S;
			BatteryMeter.gCurLevel = BatteryMeter.gCurLevel - 5;
			BatteryMeter.gDownLevel = BatteryMeter.gCurLevel - 5;
			BatteryMeter.gLevelFine = 0;
			BatteryMeter.gDownCountFine = 0;
		}		
		else if (BatteryMeter.gChargeCount >= CHARGE_TIME_5PENCENT)
		{
			BatteryMeter.gChargeCount = 0;
			BatteryMeter.gDownCount = 0;
			BatteryMeter.gCurLevel = BatteryMeter.gCurLevel + 5;
			BatteryMeter.gDownLevel = BatteryMeter.gCurLevel - 5;
			BatteryMeter.gLevelFine = 0;
		}
		BatteryMeter.gDownVoltage = Level2LoadVoltage(BatteryMeter.gDownLevel);
		BatteryMeter.gChargeVoltage = Level2ChargeVoltage(BatteryMeter.gCurLevel);
	}
 	else if((BatteryMeter.gCurLevel<30)&& (BatteryMeter.gCurLevel >=5))
	{
		// 计数
		if((BatteryMeter.gCurVoltage < BatteryMeter.gDownVoltage)
		#ifdef	SOC_IN_1PERCENT//23.11.21 SJ 避免1%计电量时，电量优先触发5%降挡，导致电量跳变
			&&(4 == BatteryMeter.gLevelFine)
		#endif
		)
		{
			BatteryMeter.gDownCount++;
		}
		else if (BatteryMeter.gCurVoltage > BatteryMeter.gChargeVoltage)
		{
			BatteryMeter.gChargeCount++;
			BatteryMeter.gChargeFlag = 1;
		}
		// 降档
		if (BatteryMeter.gDownCount == 150)
		{
			BatteryMeter.gDownCount = 0;
			BatteryMeter.gChargeCount = CHARGE_TIME_5PENCENT_S;
			BatteryMeter.gCurLevel = BatteryMeter.gCurLevel - 5;
			if (BatteryMeter.gCurLevel < 5)
				BatteryMeter.gCurLevel = 5;
			else
				BatteryMeter.gLevelFine = 0;
			
			BatteryMeter.gDownLevel = BatteryMeter.gCurLevel - 5;			
			BatteryMeter.gDownCountFine = 0;
		}		
		else if (BatteryMeter.gChargeCount >= CHARGE_TIME_5PENCENT)
		{
			BatteryMeter.gChargeCount = 0;
			BatteryMeter.gDownCount = 0;
			BatteryMeter.gCurLevel = BatteryMeter.gCurLevel + 5;
			BatteryMeter.gDownLevel = BatteryMeter.gCurLevel - 5;
			BatteryMeter.gLevelFine = 0;
		}
		BatteryMeter.gDownVoltage = Level2LoadVoltage(BatteryMeter.gDownLevel);
		BatteryMeter.gChargeVoltage = Level2ChargeVoltage(BatteryMeter.gCurLevel);
	}
	
	#ifdef SOC_IN_1PERCENT	//电量精度1%	
	BatteryMeter.gUpVoltage = Level2LoadVoltage(BatteryMeter.gCurLevel);
	BatteryMeter.gDiffVoltage1Per = (BatteryMeter.gUpVoltage - BatteryMeter.gDownVoltage) / 5;		
	if(BatteryMeter.gCurVoltage < (BatteryMeter.gUpVoltage - BatteryMeter.gDiffVoltage1Per * BatteryMeter.gLevelFine))
	{
		BatteryMeter.gDownCountFine++;	
		// 1%档位变化
		if (BatteryMeter.gDownCountFine >= 40)	//delay 20s
		{
			BatteryMeter.gDownCountFine = 0;
			if(BatteryMeter.gLevelFine < 4)
				BatteryMeter.gLevelFine += 1;		
		}
	}
	#endif	//SOC_IN_1PERCENT
}

// 输出数据更新
void BatteryDataOutManage(void)
{
	INT8U level = 0;
	level = ReadLevelFromEeprom();
	
#ifdef SOC_IN_1PERCENT	
	if(BatteryMeter.gCurLevel < 5)
		BatteryMeter.gCurLevel = 5;
	if ((BatteryMeter.gCurLevel-BatteryMeter.gLevelFine) != level)
	{
		// 保存			
    WriteLevelToEeprom(BatteryMeter.gCurLevel-BatteryMeter.gLevelFine);
	}
#else
	if (BatteryMeter.gCurLevel != level)
	{
		// 保存	
		WriteLevelToEeprom(BatteryMeter.gCurLevel);
		i32LogWrite(WARN, LOG_BAT, "Eerpom Battery Soc is %d\r\n", BatteryMeter.gCurLevel);
	}	
#endif	//SOC_IN_1PERCENT
}

#endif //#if (BATERRYMETER_TYPE == BATERRYMETER_ENABLE)


void vBatteryProc(void)
{
	if (LiBattery != sgU8BatteryType)
	{
		BatteryDataInManage();
		BatteryManage();
		BatteryDataOutManage();
	}
	vWdgSetFun(WDG_BATTERY_BIT);
	i32LogWrite(DEBUG, LOG_BAT, "Battery is Running!\r\n");
}

uint8_t u8GetBatterySoc(void)
{	
	#ifdef SOC_IN_1PERCENT//23.11.21 SJ 
	return (BatteryMeter.gCurLevel-BatteryMeter.gLevelFine);
	#else
	return BatteryMeter.gCurLevel;
	#endif
	
}

void vSetSocFlag(uint8_t u8Flag)//23.11.21 SJ 设置禁止计电量标志
{
	u8SocDisableFlag = u8Flag;
}
