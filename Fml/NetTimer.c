/*******************************************************************************
* Filename:NetTimer.c	                                             	 	   *
* Description:											   			 		   *
* Author:                                                           		   *
* Date:     														 		   *
* Revision:															 		   *
*******************************************************************************/

#include		"NetTimer.h"

tNetTimer netTimer[TIMER_NUM];		//定时器

//网络定时器初始化
void vNetTimerInit(void)
{
	unsigned int i;
	for(i = 0; i < sizeof(netTimer) / sizeof(netTimer[0]); i++)
	{
		vKillNetTimer(i);
	}
}

/*********************************************************************************************************
** 函数名称: netTimerUpdate
** 功能描述: 网络事件定时器计时(单位ms)
** 输　入: 定时时常（us）
** 输　出: 
********************************************************************************************************/
void vNetTimerUpdate(void)
{
	unsigned int i;
	for(i = 0; i < sizeof(netTimer) / sizeof(netTimer[0]); i++)
	{
		if(netTimer[i].bIsStart)
		{	
			netTimer[i].u64TimerCount++;
			if(netTimer[i].u64TimerCount >= netTimer[i].u64Deadline)
			{
				netTimer[i].bIsOvertime = true;
				netTimer[i].u64TimerCount = netTimer[i].u64Deadline;
			}
			else
			{
				netTimer[i].bIsOvertime = false;
			}
		}
	}
	
}

//启动网络定时器，单位ms
void vSetNetTimer(uint8_t u8TimerID, uint64_t u64Deadline)
{
	netTimer[u8TimerID].u64TimerCount = 0;
	//netTimer[u8TimerID].u64Deadline = u64Deadline * ( FS / 1000);
	netTimer[u8TimerID].u64Deadline = u64Deadline ;
	netTimer[u8TimerID].bIsStart = true;
	netTimer[u8TimerID].bIsOvertime = false;
}

//复位网络定时器
void vResetNetTimer(uint8_t u8TimerID)
{
	netTimer[u8TimerID].u64TimerCount = 0;
	netTimer[u8TimerID].bIsOvertime = false;
}

/*获取网络定时器是否超时*/
uint8_t u8GetNetTimerOverFlag(uint8_t u8TimerID)
{
	uint8_t res = 0;
	res = netTimer[u8TimerID].bIsOvertime;
	return res;
}

/*获取网络定时器是否超时*/
uint8_t u8GetNetTimerStartFlag(uint8_t u8TimerID)
{
	uint8_t res = 0;
	res = netTimer[u8TimerID].bIsStart;
	return res;
}


//删除网络定时器
void vKillNetTimer(uint8_t u8TimerID)
{
	netTimer[u8TimerID].u64TimerCount = 0;
	netTimer[u8TimerID].u64Deadline = 0;
	netTimer[u8TimerID].bIsStart = false;
	netTimer[u8TimerID].bIsOvertime = false;
}
