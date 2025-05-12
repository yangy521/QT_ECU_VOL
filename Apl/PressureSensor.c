/*******************************************************************************
* Filename: PressureSensor.c	                                               *
* Description:PressureSensor C Source File									   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/6/26    														   *
* Revision: V1.00															   *
*******************************************************************************/
#include "string.h"
#include "Para.h"
#include "LocalAi.h"
#include "PressureSensor.h"
#include "WdgProc.h"
#include "Userdef.h"

static xPressure sgPressure;

//void vPressureSensorSetValue(uint8_t u8ChannelNo, uint16_t u16Value)
//{
//	if (0 == u8ChannelNo)
//	{
//		i32SetPara(PARA_EmptyPressure, u16Value);
//	}
//	else if (1 == u8ChannelNo)
//	{
//		i32SetPara(PARA_FullPressure, u16Value);
//	}
//}

/*******************************************************************************
* Name: void vPressureSensorReg(PressureCallBackt CallBack)
* Descriptio: 设置蜂鸣器回调函数
* Input: Callback ： 回调函数
* Output: NULL
*******************************************************************************/
void vPressureSensorReg(PressureCallBackt CallBack)
{
	sgPressure.CallBack = CallBack;
}
	
void vPressureSensorInit(void)
{
	sgPressure.b1EmptyEn = i32GetPara(PARA_IsNoLoadCalibration);
	sgPressure.b1FullEn = i32GetPara(PARA_IsOverLoadCalibration);
	sgPressure.b1PressureEn  = i32GetPara(PARA_WeighFunc);
	sgPressure.b1FourPointWeightFuncEn  = i32GetPara(PARA_FourPointWeightFunc);//23.11.21 SJ修改
	sgPressure.u16EmptyValue = i32GetPara(PARA_EmptyPressure);
	sgPressure.u16FullValue = i32GetPara(PARA_FullPressure);
	sgPressure.u16Per80Value = sgPressure.u16EmptyValue + (sgPressure.u16FullValue - sgPressure.u16EmptyValue) * 0.8;
	sgPressure.u16Per90Value = sgPressure.u16EmptyValue + (sgPressure.u16FullValue - sgPressure.u16EmptyValue) * 0.9;
	sgPressure.u16Per99Value = sgPressure.u16EmptyValue + (sgPressure.u16FullValue - sgPressure.u16EmptyValue) * 0.99;
	sgPressure.b1PressFlag = (i32GetPara(PARA_ValveType) >> 13) & 0x1;//23.11.21 SJ修改压力传感器使能标志位
}

void vPressureSensorProc(void)
{
	if (1 == sgPressure.b1PressFlag)
	{
		uint16_t u16PressValue = 0;
		
		u16PressValue = (uint16_t)i32LocalAiGet(PRESSURE_ADC_CHANNEL);
		/*lilu 20230703 上传压力传感器相关值*/
		i32SetPara(PARA_PressureVlaue1, u16PressValue);
		i32SetPara(PARA_PressureVlaue2, u16PressValue);
		i32SetPara(PARA_LoadRate, u16PressValue * 100 / sgPressure.u16FullValue);
		if ((1 == sgPressure.b1EmptyEn) && (1 == sgPressure.b1FullEn))
		{
			i32SetPara(PARA_CalibrationStatus, 0xC0);
		}
		else 
		{
			i32SetPara(PARA_CalibrationStatus, 0x20);
		}
		
		if (FunctionEnable == sgPressure.b1PressureEn) 
		{
			if	((FunctionEnable == sgPressure.b1FourPointWeightFuncEn) &&
				(1 == sgPressure.b1EmptyEn) && 
				(1 == sgPressure.b1FullEn))
			{
				if(u16PressValue >= sgPressure.u16FullValue)
				{
					if(NULL != sgPressure.CallBack)
					{
						sgPressure.CallBack(PressureOverPer100);
					}
				}
				else if (u16PressValue >= sgPressure.u16Per99Value)
				{
					if(NULL != sgPressure.CallBack)
					{
						sgPressure.CallBack(PressureOverPer99);
					}
				}
				else if (u16PressValue >= sgPressure.u16Per90Value)
				{
					if(NULL != sgPressure.CallBack)
					{
						sgPressure.CallBack(PressureOverPer90);
					}
				}
				else if (u16PressValue >= sgPressure.u16Per80Value)
				{
					if(NULL != sgPressure.CallBack)
					{
						sgPressure.CallBack(PressureOverPer80);
					}
				}
				else if (u16PressValue < sgPressure.u16EmptyValue)
				{
					if(NULL != sgPressure.CallBack)
					{
						sgPressure.CallBack(PressureWithoutSensor);
					}
				}
				else
				{
					if(NULL != sgPressure.CallBack)
					{
						sgPressure.CallBack(PressureNoProblem);
					}
				}
				
				if (sgPressure.u16EmptyValue > sgPressure.u16FullValue)
				{
					sgPressure.CallBack(PressureCaliReverse);
				}
			}
			else
			{
				if(NULL != sgPressure.CallBack)
				{
					sgPressure.CallBack(PressureCaliFailure);
				}
			}
		}
	}	
	vWdgSetFun(WDG_PRESSURE_BIT);
}