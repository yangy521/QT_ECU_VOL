/*******************************************************************************
* Filename: AlarmLampProc.c 	                               	     	       *
* Description:	 AlarmLamp����ģ��												*
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


static xAlarmLampProc sgAlarmLampProc;	/*Beep״̬��Ϣ��ȫ�ֱ���*/

/*******************************************************************************
* Name: void vBeepRegister(BeepCallBackt Callback)
* Descriptio: ���÷������ص�����
* Input: Callback �� �ص�����
* Output: NULL
*******************************************************************************/
void vAlarmLampRegister(AlarmLampCallBackt Callback)
{
	sgAlarmLampProc.CallBack = Callback;
}
/*******************************************************************************
* Name: void vBeepSetPeriod(uint16_t u16Period)
* Descriptio: ���÷�������˸����
* Input: u16Period����˸�����ڣ���λ /min�� 0������رշ�����
* Output: NULL
*******************************************************************************/
void vAlarmLampSetPeriod(uint16_t u16Period)
{
	uint16_t tmp = 0;
	tmp = 1000 * 60 / u16Period ;	//23.11.21 SJ����
//	tmp = 1000 * 60 / u16Period / 2;  /*time = 1000 * (1 / (u16Period / 60)) / 2*/
	sgAlarmLampProc.u16BlinkPeriod = tmp;
}
/*******************************************************************************
* Name: void vBeepSetPeriod(uint16_t u16Period)
* Descriptio: ���÷�������������
* Input: u16Period����˸�����ڣ���λ /min�� 0������رշ�����
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
* Descriptio: Beepģ�鴦��
* Input: NULL
* Output: NULL 
*******************************************************************************/	
void vAlarmLampProc(void)
{
	#if 1 //23.11.21 SJ����
	
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