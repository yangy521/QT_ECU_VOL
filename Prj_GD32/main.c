/*!
    \file    main.c
    \brief   led spark with systick, USART print and key example

    \version 2017-02-10, V1.0.0, firmware for GD32F30x
    \version 2018-10-10, V1.1.0, firmware for GD32F30x
    \version 2018-12-25, V2.0.0, firmware for GD32F30x
    \version 2020-09-30, V2.1.0, firmware for GD32F30x 
*/

/*
    Copyright (c) 2020, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32f30x.h"
#include "systick.h"
#include <stdio.h>
#include "main.h"
#include "KSDsys.h"
#include "Device.h"
#include "Eeprom.h"
#include "PLC.h"
#include "NetTimer.h"
#include "PARA.h"
#include "Log.h"
#include "PcuProc.h"
#include "LedProc.h"
#include "BeepProc.h"
#include "AlarmLamp.h"
#include "WdgProc.h"
#include "CanRevProc.h"
#include "LocalDi.h"
#include "LocalDo.h"
#include "LocalAi.h"
#include "Para.h"
#include "MstSlvCom.h"
#include "PressureSensor.h"
#include "AngleSensor.h"
#include "UserTest.h"
#include "UserEcuProc.h"
#include "PropProc.h"
#include "BatteryMeter.h"
#include "timer_canfestival.h"
#include "AiProc.h"
#include "CanBaudRateSync.h"

//void	SysInit(void);

/*******************************************************************************
* Name: SysInit
* Description:
* Input: 
* Output: 
*
* Author: 
* Date: 
* Revision:
*******************************************************************************/
void SysInit(void)
{
	uint16_t tmp = 0;
	vBspInit();
	vParaInit();
	vLogInit();
	vNetTimerInit();
	vWdgInit(5000);
	vDoPwmInit();
	vPropProcInit();
	vAiProcInit();
	vAngleSensorInit();
	vPressureSensorInit();

//	InitBattery();		/*电池类型内部判断*/	
	vUserEcuInit();
	
	/*Canoepn Node default value is */
#ifdef CANOPEN_NODEID   //userdef
	Canopen_int(CANOPEN_NODEID);
#else
	tmp = i32GetPara(PARA_CanOpenNodeId);
	if (0x00 != tmp)  //para cfg nodeid
	{
		Canopen_int((uint8_t)tmp);
	}
	else //default nodeid 0x2C
	{
		Canopen_int(0x2C);
	}
#endif

//#if (MULTIPLE_MCU_MST == MCU_TYPE)
//	tmp = i32GetPara(PARA_CanBaudRate);
//	if ((CAN_BAUD_RATE_500K == tmp) || (CAN_BAUD_RATE_250K == tmp) || (CAN_BAUD_RATE_125K == tmp))
//	{
//		vDrvCanInit(tmp);
//		//vDrvCanInit(CAN_BAUD_RATE_500K);
//	}
//#endif
}


/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/

int main(void)
{
	//__disable_irq();		/*未初始化之前，先关闭总中断*/
#ifdef PRJ_RELEASE
{
	uint32_t msp;
	SCB->VTOR = 0x8004000 | (0x00 & (uint32_t)0x1FFFFF80);
	msp = *(__IO uint32_t*)0x8004000;
	__set_MSP(msp);
}
#endif
	/* configure systick */
	
	__disable_irq();		/*未初始化之前，先关闭总中断*/
	systick_config();

	SysInit();	
	
	__enable_irq();			/*初始化完之后，再打开总中断*/

	i32LogWrite(ERR, LOG_MAIN, "%s!\r\n", HARD_VERSION);
	i32LogWrite(ERR, LOG_MAIN, "%s!\r\n", SOFT_VERSION);
	i32LogWrite(ERR, LOG_MAIN, "Compile: Data:%s Time:%s\n", __DATE__, __TIME__);

	vSetNetTimer(TIMER_PLC, TIMER_PLC_PERIOD);
	vSetNetTimer(TIMER_LedProc, LED_PERIOD);
	vSetNetTimer(TIMER_WdgProc, WDG_PERIOD);
	vSetNetTimer(TIMER_BatteryProc, BATTERY_PERIOD);
	vSetNetTimer(TIMER_BatteryInit, BATTERY_INIT_PERIOD);
	vSetNetTimer(TIMER_Test, 500);		/**/
	i32LogWrite(WARN, LOG_MAIN, "Ecu is Ready!\r\n");
	
		
	i32LocalDoSet(DO_DRIVEREN, 1);
//	i32DoPwmSet(DRIVER3, 1);
	
    while (1)
	{
		if(true == u8GetNetTimerOverFlag(TIMER_BatteryInit))
		{
			InitBattery();
			vKillNetTimer(TIMER_BatteryInit);
		}
		if(true == u8GetNetTimerOverFlag(TIMER_PLC))
		{
			vResetNetTimer(TIMER_PLC);
			#if (PCU_TYPE != PCU_TYPE_NONE)
				vPcuProc();
			#else 
				vWdgSetFun(WDG_PCU_BIT);
			#endif
			vLocalDiProc();
			vCanRevProc();
			vLocalDoProc();
			vPropProc();
			vAiProc();
			vDoPwmProc();
			vMstSlvComProc();
			vBeepProc();
			vAlarmLampProc();
			vAngleSensorProc();
			vPressureSensorProc();
			vUserEcuProc();	
		}
		if(true == u8GetNetTimerOverFlag(TIMER_LedProc))
		{
			vLedProc();
			vResetNetTimer(TIMER_LedProc);
		}
		/*lilu 20240410 提高Eeprom的效率*/
		if (true == u8GetNetTimerOverFlag(TIMER_BatteryProc) && (0 == u8GetEepromFlag()))
		{
			vBatteryProc();
			vResetNetTimer(TIMER_BatteryProc);
		}
		
		if(true == u8GetNetTimerOverFlag(TIMER_WdgProc))
		{
			vWdgProc();
			vResetNetTimer(TIMER_WdgProc);
		}
		
		if(true == u8GetNetTimerOverFlag(TIMER_Test))
		{
			uint8_t i = 0;
			uint32_t DiValue = 0;
			for (i=0; i<LocalDiMax; i++)
			{
				if(1 == i32LocalDiGet(i))
				{
					DiValue |= 1 << i;
				}
//				//i32LogWrite(INFO, "Local DI%d = %d\r\n", i, i32LocalDiGet(i));
			}
			
//			vAiCheck();

			
			i32LogWrite(INFO, LOG_MAIN, "PropDriver1Current = %f\r\n", inserted_data[0] * PROP_CURRENT_FACOTR);
			i32LogWrite(INFO, LOG_MAIN, "PropDriver2Current = %f\r\n", inserted_data[1] * PROP_CURRENT_FACOTR);
			i32LogWrite(INFO, LOG_MAIN, "Battery Soc = %d\r\n", i32GetPara(PARA_BmsSoc));
//			//i32LogWrite(INFO, ",ADValue = %d, PropDriver12Current = %f\r\n", inserted_data[1], inserted_data[1] * PROP_CURRENT_FACOTR);
			i32LogWrite(INFO, LOG_MAIN, "Local Di Test = 0x%08x\r\n", DiValue);
			
			i32LogWrite(INFO, LOG_MAIN, "AI_B_KSI_CHECK Ai Value = %d\r\n", i32LocalAiGetValue(AI_B_KSI_CHECK));
			i32LogWrite(INFO, LOG_MAIN, "AI_B_VBUS_CHECK Ai Value = %d, iqVbus = %d\r\n", i32LocalAiGetValue(AI_B_VBUS_CHECK), _IQ12toIQ(i32LocalAiGet(AI_B_VBUS_CHECK)));
			i32LogWrite(INFO, LOG_MAIN, "AI_B_AI1_R Ai Value = %d\r\n", i32LocalAiGetValue(AI_B_AI1_R));
			i32LogWrite(INFO, LOG_MAIN, "AI_B_AI2_R Ai Value = %d\r\n", i32LocalAiGetValue(AI_B_AI2_R));
			i32LogWrite(INFO, LOG_MAIN, "AI_B_AI3_R Ai Value = %d\r\n", i32LocalAiGetValue(AI_B_AI3_R));
//			//i32LogWrite(INFO, "AI_5V_12V_OUT2_I Ai Value = %d\r\n", i32LocalAiGetValue(AI_5V_12V_OUT2_I));
//			//i32LogWrite(INFO, "AI_5V_12V_OUT2_R Ai Value = %d\r\n", i32LocalAiGetValue(AI_5V_12V_OUT2_R));
//			//i32LogWrite(INFO, "AI_5V_12V_OUT1_I Ai Value = %d\r\n", i32LocalAiGetValue(AI_5V_12V_OUT1_I));
//			//i32LogWrite(INFO, "AI_5V_CHECK Ai Value = %d\r\n", i32LocalAiGetValue(AI_5V_CHECK));
//			//i32LogWrite(INFO, "AI_15V_CHECK Ai Value = %d\r\n", i32LocalAiGetValue(AI_15V_CHECK));
//			//i32LogWrite(INFO, "AI_5V_12V_OUT1_R Ai Value = %d\r\n", i32LocalAiGetValue(AI_5V_12V_OUT1_R));
//			//i32LogWrite(INFO, "AI_3V3_CHECK Ai Value = %d\r\n", i32LocalAiGetValue(AI_3V3_CHECK));
			
//			if (1 == i32LocalDiGet(SWI2_R))
//			{
//				i32DoPwmSet(DRIVER1, 100);
//				i32DoPwmSet(DRIVER2, 100);
//				i32DoPwmSet(DRIVER3, 100);
//				i32DoPwmSet(DRIVER4, 100);
//				i32DoPwmSet(DRIVER5, 100);
//				i32DoPwmSet(DRIVER6, 100);
//				i32DoPwmSet(DRIVER7, 100);
//				i32DoPwmSet(DRIVER8, 100); 
//				i32DoPwmSet(DRIVER9, 100);
//				i32DoPwmSet(DRIVER10, 100);
//				int32_t i32PropValue = _IQ(2.0 / PROPD_STD_CURRENT);
//				vPropSetTarget(PropDriverCh0, i32PropValue);
//				vPropSetTarget(PropDriverCh1, i32PropValue);
//			}
//			else
//			{
//				i32DoPwmSet(DRIVER1, 0);
//				i32DoPwmSet(DRIVER2, 0);
//				i32DoPwmSet(DRIVER3, 0);
//				i32DoPwmSet(DRIVER4, 0);
//				i32DoPwmSet(DRIVER5, 0);
//				i32DoPwmSet(DRIVER6, 0);
//				i32DoPwmSet(DRIVER7, 0);
//				i32DoPwmSet(DRIVER8, 0); 
//				i32DoPwmSet(DRIVER9, 0);
//				i32DoPwmSet(DRIVER10, 0);
//				int32_t i32PropValue = _IQ(2.0 / PROPD_STD_CURRENT);
//				vPropSetTarget(PropDriverCh0, 0);
//				vPropSetTarget(PropDriverCh1, 0);
//			}
			vResetNetTimer(TIMER_Test);
		}
		/*lilu 20240104 添加非阻塞EERPOM的读写*/
		vEepromNoBlockWriteProc();
   }
}



