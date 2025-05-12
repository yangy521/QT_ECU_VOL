/*!
    \file    gd32f30x_it.c
    \brief   interrupt service routines

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

#include "gd32f30x_it.h"
#include "main.h"
#include "systick.h"
#include "NetTimer.h"
#include "Device.h"
#include "PropDriver.h"
#include "PropProc.h"
#include "DoPwm.h"
#include "IQmathLib.h"
#include "LocalDo.h"
/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while (1)
	{
		i32LocalDoSet(DO_DRIVEREN, 0);
    }
}

/*!
    \brief      this function handles MemManage exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void MemManage_Handler(void)
{
    /* if Memory Manage exception occurs, go to infinite loop */
    while (1)
	{
		i32LocalDoSet(DO_DRIVEREN, 0);
    }
}

/*!
    \brief      this function handles BusFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BusFault_Handler(void)
{
    /* if Bus Fault exception occurs, go to infinite loop */
    while (1)
	{
		i32LocalDoSet(DO_DRIVEREN, 0);
    }
}

/*!
    \brief      this function handles UsageFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UsageFault_Handler(void)
{
    /* if Usage Fault exception occurs, go to infinite loop */
    while (1)
	{
		i32LocalDoSet(DO_DRIVEREN, 0);
    }
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
}

/*!
    \brief      this function handles DebugMon exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DebugMon_Handler(void)
{
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
}

/*!
    \brief      this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SysTick_Handler(void)
{
	vNetTimerUpdate();	
}

/*!
    \brief      this function handles external lines 10 to 15 interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void EXTI10_15_IRQHandler(void)
{
#if 0
    if (RESET != exti_interrupt_flag_get(EXTI_15)) {
		PLCISR();
	
		#ifdef  WWDG_ENABLE
			fwdgt_counter_reload();
		#endif	//WWDG_ENABLE
        exti_interrupt_flag_clear(EXTI_15);
    }
#endif
}


/*!
    \brief      this function handles ADC0_1 interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/

void ADC0_1_IRQHandler(void)
{
	xPropCurrentLoop *PropTmp;
    /* clear the ADC flag */
    adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOIC);
	
    /* ADC software trigger enable */
	inserted_data[0] = adc_inserted_data_read(ADC0, ADC_INSERTED_CHANNEL_0);
	inserted_data[1] = adc_inserted_data_read(ADC1, ADC_INSERTED_CHANNEL_0);

    adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);
	/*need to make*/
	PropTmp = pPropCurrentLoopGet(PropDriverCh0);
	PropTmp->IFdb = _IQ12toIQ(inserted_data[0]) ;
	PropCurrentSmooth(PropTmp, i32GetPropCmd(PropDriverCh0));
	PropTmp->IRef = PropTmp->CurrentCmd;
	if(0 == PropTmp->IRef)
	{
		if(NULL != PropTmp->Reset)		/*for Prop Init*/
		{
			PropTmp->Reset(PropTmp);
		}
	}
	else
	{
		if(NULL != PropTmp->Calc)		/*for Prop Calc*/
		{
			PropTmp->Calc(PropTmp);
		}
	}
	
	PropTmp = pPropCurrentLoopGet(PropDriverCh1);
	PropTmp->IFdb = _IQ12toIQ(inserted_data[1]);
	PropCurrentSmooth(PropTmp, i32GetPropCmd(PropDriverCh1));
	PropTmp->IRef = PropTmp->CurrentCmd;
	if(0 == PropTmp->IRef)
	{
		if(NULL != PropTmp->Reset)		/*for Prop Init*/
		{
			PropTmp->Reset(PropTmp);
		}
	}
	else
	{
//		if (PropTmp->IRef < PropTmp->IRefMin)
//		{
//			PropTmp->IRef = PropTmp->IRefMin;
//		}
		if(NULL != PropTmp->Calc)		/*for Prop Init*/
		{
			PropTmp->Calc(PropTmp);
		}
	}
#ifdef DoFdbVoltageCal_EN			/*for Prop Init*/
	vDoFdbVoltageCal();
#endif
	
//	ControlISR();	
}

///*!
//    \brief      this function handles A DC0_1 interrupt
//    \param[in]  none
//    \param[out] none
//    \retval     none
//*/
//void ADC2_IRQHandler(void)
//{
//	xPropCurrentLoop *PropTmp;
//	adc_interrupt_flag_clear(ADC2, ADC_INT_FLAG_EOIC);
//    /* ADC software trigger enable */
//	inserted_data[2] = adc_inserted_data_read(ADC2, ADC_INSERTED_CHANNEL_0);
//	inserted_data[3] = adc_inserted_data_read(ADC2, ADC_INSERTED_CHANNEL_1);
//	
//	PropTmp = pPropCurrentLoopGet(PropDriverCh2);
//	PropTmp->IFdb = _IQ12toIQ(inserted_data[2]);
//	PropTmp->Calc(PropTmp);
//	
//	PropTmp = pPropCurrentLoopGet(PropDriverCh3);
//	PropTmp->IFdb = _IQ12toIQ(inserted_data[2]);
//	PropTmp->Calc(PropTmp);

//}	


/*!
    \brief      this function handles TIMER0 interrupt request.
    \param[in]  none
    \param[out] none
    \retval     none
*/
void TIMER0_UP_IRQHandler(void)
{
    if(SET == timer_interrupt_flag_get(TIMER0,TIMER_INT_FLAG_UP))
	{
		/* clear channel 0 interrupt bit */
		timer_interrupt_flag_clear(TIMER0,TIMER_INT_FLAG_UP);
	}
}

/*!
    \brief      this function handles TIMER3 interrupt request.
    \param[in]  none
    \param[out] none
    \retval     none
*/
void TIMER3_IRQHandler(void)
{
#if 1
    if(SET == timer_interrupt_flag_get(TIMER3,TIMER_INT_FLAG_CH0)){
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(TIMER3,TIMER_INT_FLAG_CH0);

        /* read channel 0 capture value */
        timer_channel_capture_value_register_read(TIMER3,TIMER_CH_0);
    }
    else if(SET == timer_interrupt_flag_get(TIMER3,TIMER_INT_FLAG_CH1)){
        /* clear channel 1 interrupt bit */
        timer_interrupt_flag_clear(TIMER3,TIMER_INT_FLAG_CH1);

        /* read channel 1 capture value */
        timer_channel_capture_value_register_read(TIMER3,TIMER_CH_1);
    }
    else if(SET == timer_interrupt_flag_get(TIMER3,TIMER_INT_FLAG_CH2)){
        /* clear channel 2 interrupt bit */
        timer_interrupt_flag_clear(TIMER3,TIMER_INT_FLAG_CH2);

        /* read channel 2 capture value */
        timer_channel_capture_value_register_read(TIMER3,TIMER_CH_2);
    }
#endif
}
