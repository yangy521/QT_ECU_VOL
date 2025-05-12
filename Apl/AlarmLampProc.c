/*******************************************************************************
* Filename: AlarmLampProc.c 	                               	     	       *
* Description:	 AlarmLamp处理模块												*
* Author:  QExpand, Lilu                                                       *
* Date: 2023/5/15															   *
* Revision:															           *
*******************************************************************************/

#include "AlarmLamp.h"
#include "WdgProc.h"
#include "string.h"

typedef struct
{
	uint16_t	u16Count;
	uint16_t	u16BlinkPeriod;      //Alarm blink period ms
	uint16_t	u16BlinkOpenPeriod;		//23.11.21 SJ set open valve time
	uint8_t 	u8Flag;
	AlarmLampCallBackt CallBack;
}xAlarmLampProc;


static xAlarmLampProc sgAlarmLampProc;	/*Beep状态信息的全局变量*/

/*******************************************************************************
* Name: void vBeepRegister(BeepCallBackt Callback)
* Descriptio: 设置蜂鸣器回调函数
* Input: Callback ： 回调函数
* Output: NULL
*******************************************************************************/
void vAlarmLampRegister(AlarmLampCallBackt Callback)
{
	sgAlarmLampProc.CallBack = Callback;
}
/*******************************************************************************
* Name: void vBeepSetPeriod(uint16_t u16Period)
* Descriptio: 设置蜂鸣器闪烁周期
* Input: u16Period：闪烁的周期，单位 /min； 0：代表关闭蜂鸣器
* Output: NULL
*******************************************************************************/
void vAlarmLampSetPeriod(uint16_t u16Period)
{
	uint16_t tmp = 0;
	tmp = 1000 * 60 / u16Period ;	//23.11.21 SJ测试
//	tmp = 1000 * 60 / u16Period / 2;  /*time = 1000 * (1 / (u16Period / 60)) / 2*/
	sgAlarmLampProc.u16BlinkPeriod = tmp;
}
/*******************************************************************************
* Name: void vBeepSetPeriod(uint16_t u16Period)
* Descriptio: 设置蜂鸣器开启周期
* Input: u16Period：闪烁的周期，单位 /min； 0：代表关闭蜂鸣器
* Output: NULL
*******************************************************************************/
void vAlarmLampSetOpenePeriod(uint16_t u16Period)
{
	uint16_t tmp = 0;
	tmp = 1000 * 60 / u16Period ;
	sgAlarmLampProc.u16BlinkOpenPeriod = tmp;
}

/*******************************************************************************
* Name: void vBeepProc(void)
* Descriptio: Beep模块处理
* Input: NULL
* Output: NULL 
*******************************************************************************/	
void vAlarmLampProc(void)
{
	#if 1 //23.11.21 SJ测试
	
	if(0 != sgAlarmLampProc.u16BlinkPeriod)
	{
		sgAlarmLampProc.u16Count += ALARMLAMP_PERIOD;
		
		if(sgAlarmLampProc.u16Count <= sgAlarmLampProc.u16BlinkOpenPeriod)
		{
			if ( NULL != sgAlarmLampProc.CallBack)
			{
				sgAlarmLampProc.CallBack(1);
			}
		}
		else if(sgAlarmLampProc.u16Count <= sgAlarmLampProc.u16BlinkPeriod)
		{
			if ( NULL != sgAlarmLampProc.CallBack)
			{
				sgAlarmLampProc.CallBack(0);
			}
		}
		else
		{
			sgAlarmLampProc.u16Count = 0;
		}
	}
	else
	{
		sgAlarmLampProc.u8Flag = 0;
		sgAlarmLampProc.u16Count = 0;
	}	
	
	#else
	
	
	if(0 != sgAlarmLampProc.u16BlinkPeriod)
	{
		if(sgAlarmLampProc.u16Count >= sgAlarmLampProc.u16BlinkPeriod)
		{
			sgAlarmLampProc.u8Flag ^= 1;
			if(1 == sgAlarmLampProc.u8Flag)
			{
				if ( NULL != sgAlarmLampProc.CallBack)
				{
					sgAlarmLampProc.CallBack(1);
				}
			}
			else
			{
				if ( NULL != sgAlarmLampProc.CallBack)
				{
					sgAlarmLampProc.CallBack(0);
				}
			}
			sgAlarmLampProc.u16Count = 0;
		}
		else
		{
			sgAlarmLampProc.u16Count += ALARMLAMP_PERIOD;
		}
	}
	else
	{
		sgAlarmLampProc.u8Flag = 0;
		sgAlarmLampProc.u16Count = 0;
	}
	#endif
	vWdgSetFun(WDG_LAMP_BIT);
}