/*******************************************************************************
* Filename: WdgProc.c 	                                    	     	       *
* Description: 	WdgProc C Source File						           		   *
* Author:  QExpand, Lilu                                                       *
* Date: 2023/5/15															   *
* Revision:															           *
*******************************************************************************/
#include <gd32f30x.h>
#include "Log.h"
#include "WatchDog.h"
#include "WdgProc.h"
#include "Device.h"
#include "LocalDo.h"
#include "ErrCode.h"


#define	MST_WDG_ERR_CNT		100
#define	MST_WDG_INIT_CNT	3000

static const xWdgNamet xWdgName[WDG_FUNS_NO] = {
	{0,  "CanRev"},
	{1,  "Led"},
	{2,  "Pcu"},
	{3,  "MstSlv"},
	{4,  "LocalDi"},
	{5,  "LocalDo"},
	{6,  "DoPwm"},
	{7,  "PropProc"},
	{8,  "BeepProc"},
	{9,  "LampProc"},
	{10, "Angle"},
	{11, "Pressure"},
	{12, "AiProc"},
	{13, "Battery"},
	{14, "Fun14"},
	{15, "UserProc"},
};


static xWdgParat sgWdgPara; /*global watchdog varaible*/

static void Timer6Init(uint32_t psr, uint32_t arr)
{
	timer_parameter_struct timer_init_struct;
	rcu_periph_clock_enable(RCU_TIMER6);
	timer_deinit(TIMER6);
	timer_init_struct.prescaler			= psr;	
	timer_init_struct.period			= arr;	
	timer_init_struct.alignedmode		= TIMER_COUNTER_EDGE;	
	timer_init_struct.counterdirection	= TIMER_COUNTER_UP;		
	timer_init_struct.clockdivision		= TIMER_CKDIV_DIV1;		
	timer_init_struct.repetitioncounter = 0;					
	timer_init(TIMER6, &timer_init_struct);
	nvic_irq_enable(TIMER6_IRQn, 1, 1); 
    timer_interrupt_enable(TIMER6, TIMER_INT_UP);
	timer_enable(TIMER6);
}

typedef union
{
	uint8_t u8Data;
	struct
	{
		uint8_t b7Cnt: 7;
		uint8_t b1SStat: 1;
	};
}xWdgOutInfo;

void TIMER6_IRQHandler(void)
{
	static uint8_t u8Stat = 0;
	static uint16_t u16WdgCnt = 0;
	static uint32_t u32WdgCnt = 0;
	
	static xWdgOutInfo  WdgOutInfo = {.u8Data = 0};
	
	uint8_t u8Tmp = 0;
	if(timer_interrupt_flag_get(TIMER6, TIMER_INT_FLAG_UP)!= RESET)
	{
		timer_interrupt_flag_clear(TIMER6, TIMER_INT_FLAG_UP);
		
		if (WdgOutInfo.b7Cnt++ >= 4)
		{
			if (1 == WdgOutInfo.b1SStat)
			{
				DO_MST_WDG_ON();
			}
			else
			{
				DO_MST_WDG_OFF();
			}
			WdgOutInfo.b1SStat = ~WdgOutInfo.b1SStat;
			WdgOutInfo.b7Cnt = 0;
		}
		
		u32WdgCnt++;
	 
		if (u32WdgCnt >= MST_WDG_INIT_CNT)
		{
			u32WdgCnt = MST_WDG_INIT_CNT;
			
			u8Tmp = READ_WDG();
			
			if (u8Stat == u8Tmp)
			{
				u16WdgCnt++;
				if (u16WdgCnt >= MST_WDG_ERR_CNT)
				{
					/*alarm */
					i32LocalDoSet(DO_DRIVEREN, 0);
					i32ErrCodeSet(ErrCode82);				/*lilu 20230905, MCU Plus error，报64号故障*/	//23.11.21 SJ修改故障码
					i32LogWrite(ERR, LOG_WDG, "Long Time No Rev Mst Hard Wdg!!!!!!\r\n");
				}
			}
			else
			{
				u16WdgCnt = 0;
				u8Stat = u8Tmp;
			}
		}
	}
}


/*******************************************************************************
* Name: void vWdgInit(uint32_t u32TimeOut)
* Descriptio: 看门狗初始化函数
* Input: u32TimeOut：看门狗超时时间，单位ms 
* Output: NULL  
*******************************************************************************/
void vWdgInit(uint32_t u32TimeOut)
{
	sgWdgPara.u32WdgTimeOut = u32TimeOut;
	sgWdgPara.u32WdgTimeCnt = 0;
	sgWdgPara.u32WdgMonitorBit = 0;
	
	Timer6Init(120 - 1, 1000 - 1);
}

/*******************************************************************************
* Name: void vWdgSetFun(uint8_t u8SetBit)
* Descriptio: 设置对应功能的看门狗监控位
* Input: u8SetBit： u8SetBit 功能模块占据的位置
* Output: NULL  
*******************************************************************************/
void vWdgSetFun(uint8_t u8SetBit)
{
	sgWdgPara.u32WdgMonitorBit |= (uint32_t)(1 << u8SetBit);
}

/*清除看门狗监控位*/
static void vWdgClr(void)
{
	sgWdgPara.u32WdgMonitorBit = 0;
}
/*******************************************************************************
* Name: void vWdgProc(void)
* Descriptio: 看门狗处理函数
* Input: NULL
* Output: NULL  
*******************************************************************************/
void vWdgProc(void)
{
	if(WDG_FUNS_ALL == (sgWdgPara.u32WdgMonitorBit & WDG_FUNS_ALL))
	{
		/*Perform hardware dog feeding*/
		vFeedDog();
		vWdgClr();
		sgWdgPara.u32WdgTimeCnt = 0;
		i32ErrCodeClr(ErrCode83);
	}
	else
	{
		if(sgWdgPara.u32WdgTimeCnt++ >= (sgWdgPara.u32WdgTimeOut / WDG_PERIOD))
		{
			uint8_t i = 0;
			for (i = 0; i<WDG_FUNS_NO; i++)
			{
				if(!(sgWdgPara.u32WdgMonitorBit & (1 << (xWdgName[i].u8Bit))))
				{
					i32LogWrite(WARN, LOG_WDG, "%s Fcuntions Feed OverTime\r\n", xWdgName[i].u8FunName);
				}
			}
			i32ErrCodeSet(ErrCode83);				/*lilu 20230905, WatchDog，报65号故障*///23.11.21 SJ修改
			sgWdgPara.u32WdgTimeCnt = 0;
		}
	}	
}