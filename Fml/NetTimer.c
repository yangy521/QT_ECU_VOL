/*******************************************************************************
* Filename:NetTimer.c	                                             	 	   *
* Description:											   			 		   *
* Author:                                                           		   *
* Date:     														 		   *
* Revision:															 		   *
*******************************************************************************/

#include		"NetTimer.h"

tNetTimer netTimer[TIMER_NUM];		//��ʱ��

//���綨ʱ����ʼ��
void vNetTimerInit(void)
{
	unsigned int i;
	for(i = 0; i < sizeof(netTimer) / sizeof(netTimer[0]); i++)
	{
		vKillNetTimer(i);
	}
}

/*********************************************************************************************************
** ��������: netTimerUpdate
** ��������: �����¼���ʱ����ʱ(��λms)
** �䡡��: ��ʱʱ����us��
** �䡡��: 
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

//�������綨ʱ������λms
void vSetNetTimer(uint8_t u8TimerID, uint64_t u64Deadline)
{
	netTimer[u8TimerID].u64TimerCount = 0;
	//netTimer[u8TimerID].u64Deadline = u64Deadline * ( FS / 1000);
	netTimer[u8TimerID].u64Deadline = u64Deadline ;
	netTimer[u8TimerID].bIsStart = true;
	netTimer[u8TimerID].bIsOvertime = false;
}

//��λ���綨ʱ��
void vResetNetTimer(uint8_t u8TimerID)
{
	netTimer[u8TimerID].u64TimerCount = 0;
	netTimer[u8TimerID].bIsOvertime = false;
}

/*��ȡ���綨ʱ���Ƿ�ʱ*/
uint8_t u8GetNetTimerOverFlag(uint8_t u8TimerID)
{
	uint8_t res = 0;
	res = netTimer[u8TimerID].bIsOvertime;
	return res;
}

/*��ȡ���綨ʱ���Ƿ�ʱ*/
uint8_t u8GetNetTimerStartFlag(uint8_t u8TimerID)
{
	uint8_t res = 0;
	res = netTimer[u8TimerID].bIsStart;
	return res;
}


//ɾ�����綨ʱ��
void vKillNetTimer(uint8_t u8TimerID)
{
	netTimer[u8TimerID].u64TimerCount = 0;
	netTimer[u8TimerID].u64Deadline = 0;
	netTimer[u8TimerID].bIsStart = false;
	netTimer[u8TimerID].bIsOvertime = false;
}
