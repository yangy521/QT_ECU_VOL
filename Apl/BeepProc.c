/*******************************************************************************
* Filename: BeepProc.c 	                                    	     	       *
* Description: 	Beep处理模块						           				   *
* Author:  QExpand, Lilu                                                       *
* Date: 2023/5/15															   *
* Revision:															           *
*******************************************************************************/

#include "BeepProc.h"
#include "WdgProc.h"
#include "string.h"

typedef struct
{
	uint16_t	u16Count;
	uint16_t	u16BlinkPeriod;      //Beep blink period ms
	uint16_t	u16BlinkOpenPeriod;
	uint8_t 	u8Flag;
	BeepCallBackt CallBack;
}xBeepProc;


static xBeepProc sgBeepProc;	/*Beep状态信息的全局变量*/

/*******************************************************************************
* Name: void vBeepRegister(BeepCallBackt Callback)
* Descriptio: 设置蜂鸣器回调函数
* Input: Callback ： 回调函数
* Output: NULL
*******************************************************************************/
void vBeepRegister(BeepCallBackt Callback)
{
	sgBeepProc.CallBack = Callback;
}
/*******************************************************************************
* Name: void vBeepSetPeriod(uint16_t u16Period)
* Descriptio: 设置蜂鸣器闪烁周期
* Input: u16Period：闪烁的周期，单位 /min； 0：代表关闭蜂鸣器
* Output: NULL
*******************************************************************************/
void vBeepSetPeriod(uint16_t u16Period)
{
	uint16_t tmp = 0;
	tmp = 1000 * 60 / u16Period ;	//23.11.21 SJ测试
//	tmp = 1000 * 60 / u16Period / 2;  /*time = 1000 * (1 / (u16Period / 60)) / 2*/
	sgBeepProc.u16BlinkPeriod = tmp;
}

/*******************************************************************************
* Name: void vBeepSetPeriod(uint16_t u16Period)
* Descriptio: 设置蜂鸣器开启周期
* Input: u16Period：闪烁的周期，单位 /min； 0：代表关闭蜂鸣器
* Output: NULL
*******************************************************************************/
void vBeepSetOpenePeriod(uint16_t u16Period)
{
	uint16_t tmp = 0;
	tmp = 1000 * 60 / u16Period ;
	sgBeepProc.u16BlinkOpenPeriod = tmp;
}


/*******************************************************************************
* Name: void vBeepProc(void)
* Descriptio: Beep模块处理
* Input: NULL
* Output: NULL 
*******************************************************************************/	
void vBeepProc(void)
{
	#if 1	//23.11.21 SJ测试
	if(0 != sgBeepProc.u16BlinkPeriod)
	{
		sgBeepProc.u16Count += BEEP_PERIOD;
		
		if(sgBeepProc.u16Count <= sgBeepProc.u16BlinkOpenPeriod)
		{
			if ( NULL != sgBeepProc.CallBack)
			{
				sgBeepProc.CallBack(1);
			}
		}
		else if(sgBeepProc.u16Count <= sgBeepProc.u16BlinkPeriod)
		{
			if ( NULL != sgBeepProc.CallBack)
			{
				sgBeepProc.CallBack(0);
			}
		}
		else
		{
			sgBeepProc.u16Count = 0;
		}
	}
	else
	{
		if ( NULL != sgBeepProc.CallBack)
		{
			sgBeepProc.CallBack(0);
		}
		sgBeepProc.u8Flag = 0;
		sgBeepProc.u16Count = 0;
	}
		
	#else
	if(0 != sgBeepProc.u16BlinkPeriod)
	{
		if(sgBeepProc.u16Count >= sgBeepProc.u16BlinkPeriod)
		{
			sgBeepProc.u8Flag ^= 1;
			if(1 == sgBeepProc.u8Flag)//修改：开阀时间缩短
			{
				if ( NULL != sgBeepProc.CallBack)
				{
					sgBeepProc.CallBack(1);
				}
			}
			else
			{
				if ( NULL != sgBeepProc.CallBack)
				{
					sgBeepProc.CallBack(0);
				}
			}
			sgBeepProc.u16Count = 0;
		}
		else
		{
			sgBeepProc.u16Count += BEEP_PERIOD;
		}
	}
	else
	{
		//i32DoPwmSet(DRIVER8, 0);
		sgBeepProc.u8Flag = 0;
		sgBeepProc.u16Count = 0;
		if ( NULL != sgBeepProc.CallBack)
		{
			sgBeepProc.CallBack(0);
		}
	}
	#endif
	vWdgSetFun(WDG_BEEP_BIT);
}