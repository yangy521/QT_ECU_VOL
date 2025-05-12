/*******************************************************************************
* Filename: AngleSensor.c	                                               	   *
* Description:AngleSensor C Source File									       *
* Author: QExpand, Lilu                                                        *
* Date: 2023/6/26    														   *
* Revision: V1.00															   *
*******************************************************************************/
#include "string.h"
#include "Para.h"
#include "LocalAi.h"
#include "AngleSensor.h"
#include "WdgProc.h"
#include "Userdef.h"

static xAngle sgAngle;
static uint8_t sgU8AngleFlag = 0;

void vAngleSensorReg(AngleCallBackt CallBack)
{
	sgAngle.CallBack = CallBack;
}
	
void vAngleSensorInit(void)
{
	sgAngle.u16MinValue = i32GetPara(PARA_MinAngle);
	sgAngle.u16MaxValue = i32GetPara(PARA_MaxAngle);
	sgAngle.u16Value[0] = i32GetPara(PARA_AngleValue0);
	sgAngle.u16Value[1] = i32GetPara(PARA_AngleValue1);
	sgAngle.u16Value[2] = i32GetPara(PARA_AngleValue2);
	sgAngle.u16Value[3] = i32GetPara(PARA_AngleValue3);
	sgAngle.u16Value[4] = i32GetPara(PARA_AngleValue4);
	sgAngle.u16Value[5] = i32GetPara(PARA_AngleValue5);
	sgAngle.u16Value[6] = i32GetPara(PARA_AngleValue6);
	sgAngle.u16Value[7] = i32GetPara(PARA_AngleValue7);
	sgU8AngleFlag = (i32GetPara(PARA_ValveType) >> 12) & 0x1;	//23.11.21 修改判断标志位
}

void vAngleSensorProc(void)
{
//#if (CTRL_TYPE == CTRL_ECU)	
	if (0 != sgU8AngleFlag)
	{

		uint16_t u16AngleValue = 0;
		
		u16AngleValue = (uint16_t)i32LocalAiGet(ANGLE_ADC_CHANNEL);
		
		i32SetPara(PARA_AngleValue, u16AngleValue);						/*lilu 20230703,上传角度值*/
		
		if ((u16AngleValue <= sgAngle.u16MinValue) || (u16AngleValue >= sgAngle.u16MaxValue))
		{
			if (NULL != sgAngle.CallBack)
			{
				sgAngle.CallBack(1);
			}
		}
		else
		{
			if (NULL != sgAngle.CallBack)
			{
				sgAngle.CallBack(0);
			}
		}
	}	
//#endif
	vWdgSetFun(WDG_ANGLE_BIT);
}