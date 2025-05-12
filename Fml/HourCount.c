/*******************************************************************************
* Filename: HourCount.c                                                  	   *
*                                                                    		   *
* Description: 							            				 		   *
* Author:                                                                 	   *
* Date: 20190711														       *
* Revision:															 		   *
*																	 		   *
*******************************************************************************/

#include "Para.h"
#include "HourCount.h"
#include "Eeprom.h"



#define PARANO_HOUR1_FLAG				PARA_HourCountFlag1
#define PARANO_HOUR1_LOW				PARA_HourCount1High
#define PARANO_HOUR1_HIGH				PARA_HourCount1Low
#define PARANO_HOUR2_FLAG				PARA_HourCountFlag2
#define PARANO_HOUR2_LOW				PARA_HourCount2High
#define PARANO_HOUR2_HIGH				PARA_HourCount2Low

#define PARA_HOURFLAGENABLE				PARA_HourCountPowerOn

//INT32U HourCount1s;
// 从Eeprom读取配置信息
uint32_t u32HourCountRead(void)
{
	INT16U dataL, dataH;
	INT16U	HourFlag1, HourFlag2;
	INT32U	Hour1, Hour2;
	
	uint32_t u32Res;
	
	// 小时计
	//No1
	if(0 != u16EepromRead(PARANO_HOUR1_FLAG, &HourFlag1, 1))
	{
		HourFlag1 = BATTERY_HOUR_INVALID;
	}
	
	if (BATTERY_HOUR_VALID == HourFlag1)
	{
//		if (0 != i32GetPara(PARA_HourCountPowerOn))//
//		{
			if ((u16EepromRead(PARANO_HOUR1_LOW, &dataL,1) == 0) &&(u16EepromRead(PARANO_HOUR1_HIGH, &dataH,1) == 0))
			{
				Hour1 = dataL + ((INT32U)dataH << 16);
			}
			else
			{
				Hour1 = 0;
			}
//		}
//		else
//		{
//			Hour1 = 0;
//		}
	}
	else
	{
		Hour1 = 0;
	}
	//No2

	if(0 != u16EepromRead(PARANO_HOUR2_FLAG, &HourFlag2, 1))
	{
		HourFlag2 = BATTERY_HOUR_INVALID;
	}
	
	if (BATTERY_HOUR_VALID == HourFlag2)
	{
//		if (0 == i32GetPara(PARA_HourCountPowerOn))
//		{
			if ((u16EepromRead(PARANO_HOUR2_LOW, &dataL,1) == 0) &&(u16EepromRead(PARANO_HOUR2_HIGH, &dataH,1) == 0))
			{
				Hour2 = dataL + ((INT32U)dataH << 16);
			}
			else
			{
				Hour2 = 0;
			}
//		}
//		else
//		{
//			Hour2 = 0;
//		}
	}
	else
	{
		Hour2 = 0;
	}
	//Adopt larger Hour
	if (Hour1 > Hour2)
	{
		u32Res = Hour1;
	}
	else
	{
		u32Res = Hour2;
	}
	
	return u32Res;
	
}
// 向I2C写入配置信息
void vHourCountWrite(uint32_t u32HourCount)
{
	INT16U dataL, dataH;
	INT16U	HourFlag1, HourFlag2;
	INT32U	Hour1, Hour2, HourCnt;
	INT16U AdrHourFlag,AdrDataL, AdrDataH;
	
	if(0 != u16EepromRead(PARANO_HOUR1_FLAG, &HourFlag1, 1))
	{
		HourFlag1 = BATTERY_HOUR_INVALID;
	}
	
	if (BATTERY_HOUR_VALID == HourFlag1)
	{
		// 小时计
		//No1
//		if (0 != i32GetPara(PARA_HourCountPowerOn))//
//		{
			if ((u16EepromRead(PARANO_HOUR1_LOW, &dataL,1) == 0) &&(u16EepromRead(PARANO_HOUR1_HIGH, &dataH,1) == 0))
			{
				Hour1 = dataL + ((INT32U)dataH << 16);
			}
			else
			{
				Hour1 = 0;
			}
//		}
//		else
//		{
//			Hour1 = 0;
//		}
	}
	else
	{
		Hour1 = 0;
	}
	
//	if ((0 != i32GetPara(PARA_HourCountPowerOn)) ||(u32HourCount != Hour1))
	if (u32HourCount != Hour1)
	{
//		u16EepromWrite(PARANO_HOUR1_FLAG, BATTERY_HOUR_INVALID, 1);
//		Hour1 = u32HourCount;
//		u16EepromWrite(PARANO_HOUR1_LOW, (Hour1 & 0xFFFF),1);
//		u16EepromWrite(PARANO_HOUR1_HIGH, ((Hour1 >> 16) & 0xFFFF),1);
//		u16EepromWrite(PARANO_HOUR1_FLAG, BATTERY_HOUR_VALID, 1);
		u16EepromWrite(PARANO_HOUR1_FLAG, BATTERY_HOUR_INVALID, 2);
		Hour1 = u32HourCount;
		u16EepromWrite(PARANO_HOUR1_LOW, (Hour1 & 0xFFFF),2);
		u16EepromWrite(PARANO_HOUR1_HIGH, ((Hour1 >> 16) & 0xFFFF),2);
		u16EepromWrite(PARANO_HOUR1_FLAG, BATTERY_HOUR_VALID, 2);
	}
	
	
	if(0 != u16EepromRead(PARANO_HOUR2_FLAG, &HourFlag2, 1))
	{
		HourFlag2 = BATTERY_HOUR_INVALID;
	}
	
	if (BATTERY_HOUR_VALID == HourFlag2)
	{
		//No2
//		if(0 == i32GetPara(PARA_HourCountPowerOn))
//		{
			if ((u16EepromRead(PARANO_HOUR2_LOW, &dataL,1) == 0) &&(u16EepromRead(PARANO_HOUR2_HIGH, &dataH,1) == 0))
			{
				Hour2 = dataL + ((INT32U)dataH << 16);
			}
			else
			{
				Hour2 = 0;
			}
//		}
//		else
//		{
//			Hour2 = 0;
//		}
	}
	else
	{
		Hour2 = 0;
	}
	
//	if (0 == i32GetPara(PARA_HourCountPowerOn))
	if (u32HourCount != Hour2)
	{
//		u16EepromWrite(PARANO_HOUR2_FLAG, BATTERY_HOUR_INVALID, 1);
//		Hour2 = u32HourCount;
//		u16EepromWrite(PARANO_HOUR2_LOW, (u32HourCount & 0xFFFF),1);
//		u16EepromWrite(PARANO_HOUR2_HIGH, ((u32HourCount >> 16) & 0xFFFF),1) ;	
//		u16EepromWrite(PARANO_HOUR2_FLAG, BATTERY_HOUR_VALID, 1);
		u16EepromWrite(PARANO_HOUR2_FLAG, BATTERY_HOUR_INVALID, 2);
		Hour2 = u32HourCount;
		u16EepromWrite(PARANO_HOUR2_LOW, (u32HourCount & 0xFFFF),2);
		u16EepromWrite(PARANO_HOUR2_HIGH, ((u32HourCount >> 16) & 0xFFFF),2) ;	
		u16EepromWrite(PARANO_HOUR2_FLAG, BATTERY_HOUR_VALID, 2);
	}
}
