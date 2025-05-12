/*******************************************************************************
* Filename: LocalDo.c 	                                    	     	       *
* Description: 	LocalDo����					           				           *
* Author:  QExpand, Lilu                                                       *
* Date: 2023/5/16															   *
* Revision:	V1.00														       *
*******************************************************************************/
#include "LocalDo.h"
#include "Log.h"
#include "Device.h"
#include "WdgProc.h"


static uint8_t sgu8Do[LocalDoMax];	/*DO�����ȫ�ֱ���*/
/*******************************************************************************
* Name: int32_t i32LocalDoSet(eDoNo DoNo, uint8_t u8State)
* Descriptio: ����Doͨ������
* Input: DiNo��Doͨ�����
		 u8State����Ӧ�ĸߵ͵�ƽ��0 or 1
* Output:-1�������������
		  0�����óɹ�
*******************************************************************************/
int32_t i32LocalDoSet(eDoNo DoNo, uint8_t u8State)
{
	if(DoNo >= LocalDoMax)
	{
		i32LogWrite(ERR, LOG_DO, "LocalDo Parameter is wrong, LocalDoMax = %d, LocalDo = %d\r\n", LocalDoMax, DoNo);
		return -1;
	}
	sgu8Do[DoNo] = u8State;
	return 0;
}

/*******************************************************************************
* Name: void vLocalDoInit(void)
* Descriptio: LocalDo��ʼ��
* Input: NULL
* Output:NULL
*******************************************************************************/
void vLocalDoInit(void)
{
	/*�����ɰ��ղ�������*/
	i32LocalDoSet(DO_ANALOG_1, 0);		/*����ĿǰҪ������Ϊ�͵�ƽ*/
	i32LocalDoSet(DO_ANALOG_2, 0);		/*����ĿǰҪ������Ϊ�͵�ƽ*/
	i32LocalDoSet(DO_ANALOG_3, 0);		/*����ĿǰҪ������Ϊ�͵�ƽ*/
	i32LocalDoSet(DO_ENCODER1, 0);		/*����ĿǰҪ������Ϊ�͵�ƽ*/
	i32LocalDoSet(DO_ENCODER2, 0);		/*����ĿǰҪ������Ϊ�͵�ƽ*/
}

/*******************************************************************************
* Name: void vLocalDoProc(void)
* Descriptio: LocalDo���й��ܺ���
* Input: NULL
* Output:  NULL
*******************************************************************************/
void vLocalDoProc(void)
{
	if(1 == sgu8Do[DO_DRIVEREN])
	{
		DO_RESET_ON();
	}
	else
	{
		DO_RESET_OFF();
	}
	
	if(1 == sgu8Do[DO_ANALOG_1])
	{
		D0_ANALOG_1_ON();
	}
	else
	{
		D0_ANALOG_1_OFF();
	}

	if(1 == sgu8Do[DO_ANALOG_2])
	{
		D0_ANALOG_2_ON();
	}
	else
	{
		D0_ANALOG_2_OFF();
	}

	if(1 == sgu8Do[DO_ANALOG_3])
	{
		D0_ANALOG_3_ON();
	}
	else
	{
		D0_ANALOG_3_OFF();
	}

	if(1 == sgu8Do[DO_ENCODER1])
	{
		DO_ENCODER1_POWER_12V();
	}
	else
	{
		DO_ENCODER1_POWER_5V();
	}

	if(1 == sgu8Do[DO_ENCODER2])
	{
		DO_ENCODER2_POWER_12V();
	}
	else
	{
		DO_ENCODER2_POWER_5V();
	}

	
	vWdgSetFun(WDG_LOCALDO_BIT);
	
}