/******************** (C) COPYRIGHT  XI`AN QEXPAND *******************************
 * �ļ���  ��Drvwdg.c
 * ����    ��fwdg�����ļ�       
 * ʵ��ƽ̨��ECU
 * ��汾  ��V1.0.0
 * ����    ��
 * �޸�ʱ��: 2023-05-16
**********************************************************************************/	
#include "Device.h"
#include "gd32f30x_fwdgt.h"
void vDrvwdgInit()
{
	rcu_periph_clock_enable(RCU_WWDGT);
	/* confiure FWDGT counter clock: 40KHz(IRC40K) / 64 = 0.625 KHz */
	fwdgt_config(4095,FWDGT_PSC_DIV64);

	/* After 0.6 seconds to generate a reset */
	fwdgt_enable();

	//fwdgt_counter_reload();
}


