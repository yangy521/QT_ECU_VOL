/*******************************************************************************
* Filename: iTimer.c	                                             	 		   *
* Description:											   			 		   *
* Author:                                                           		   *
* Date:     														 		   *
* Revision:															 		   *
*******************************************************************************/

#include		"iTimer.h"

tNetTimer netTimer[TIMER_NUM];		//��ʱ��

//���綨ʱ����ʼ��
void netTimerInit(void)
{
	unsigned int i;
	for(i=0;i<sizeof(netTimer)/sizeof(netTimer[0]);i++)
	{
		KillNetTimer(i);
	}
}

/*********************************************************************************************************
** ��������: netTimerUpdate
** ��������: �����¼���ʱ����ʱ(��λms)
** �䡡��: ��ʱʱ����us��
** �䡡��: 
********************************************************************************************************/
void netTimerUpdate(void)
{
	unsigned int i;
	for(i=0;i<sizeof(netTimer)/sizeof(netTimer[0]);i++)
	{
		if(netTimer[i].bIsStart)
		{	
			netTimer[i].ulTimerCount ++;
			if(netTimer[i].ulTimerCount>=netTimer[i].ulDeadline)
			{
				netTimer[i].bIsOvertime=true;
				netTimer[i].ulTimerCount=netTimer[i].ulDeadline;
			}
			else
			{
				netTimer[i].bIsOvertime=false;
			}
		}
	}
	
}

//�������綨ʱ������λms
void SetNetTimer(unsigned char ucTimerID,unsigned long ulDeadline)
{
	netTimer[ucTimerID].ulTimerCount=0;
	netTimer[ucTimerID].ulDeadline=ulDeadline*(FS/1000);
	netTimer[ucTimerID].bIsStart=true;
	netTimer[ucTimerID].bIsOvertime=false;
}

//��λ���綨ʱ��
void ResetNetTimer(unsigned char ucTimerID)
{
	netTimer[ucTimerID].ulTimerCount=0;
	netTimer[ucTimerID].bIsOvertime=false;
}

//ɾ�����綨ʱ��
void KillNetTimer(unsigned char ucTimerID)
{
	netTimer[ucTimerID].ulTimerCount=0;
	netTimer[ucTimerID].ulDeadline=0;
	netTimer[ucTimerID].bIsStart=false;
	netTimer[ucTimerID].bIsOvertime=false;
}
