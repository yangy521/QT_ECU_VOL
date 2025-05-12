/******************** (C) COPYRIGHT  XI`AN QEXPAND *******************************
* 文件名  ：DrvCan.c
* 描述    ：Can驱动文件       
* 实验平台：ECU
* 库版本  ：V1.0.0
* 作者    ：
* 修改时间: 2023-05-16
**********************************************************************************/	

#include "Device.h"

void vDrvPwmInit(void)
{
	/**************************************************************************
	*DO PWM: RELAY 200hz  DRIVER1~4 200HZ  
	*管脚40  PE9	TIMER0_CH0 #Driver1;			管脚42  PE11	 	TIMER0_CH1	Driver2; 
	*管脚45  PE13	TIMER0_CH3 #Driver3;			管脚46  PE14	 	TIMER0_CH4	Driver4; 

	*管脚77  PA15	TIMER1_CH0 #Driver5;			管脚89  PB3	 		TIMER1_CH1	Driver6; 
	*管脚47  PB10	TIMER1_CH3 #Driver7;			管脚48  PB11	 	TIMER1_CH4	Driver8; 

	*管脚77  PC6	TIMER2_CH0 #Driver9;			管脚89  PC7	 		TIMER2_CH1	Driver10; 
	*管脚77  PC8	TIMER2_CH3 #Driver9 I EN;		管脚89  PC9	 		TIMER2_CH4	Driver10 I EN; 

	*管脚77  PD12	TIMER3_CH0 #Driver11;			管脚89  PD13	 	TIMER3_CH1	Driver12; 
	*管脚77  PD14	TIMER3_CH3 #Driver11 I EN;		管脚89  PD15	 	TIMER3_CH4	Driver12 I EN; 

	**************************************************************************/

	timer_parameter_struct timer_initpara;
	timer_oc_parameter_struct timer_ocintpara;
	rcu_periph_clock_enable(RCU_TIMER0); 
	rcu_periph_clock_enable(RCU_TIMER1); 
	rcu_periph_clock_enable(RCU_TIMER2); 
	rcu_periph_clock_enable(RCU_TIMER3); 

	/*Configure PA0 PA1 PA2(TIMER1 CH0 CH1 CH2 CH3) as alternate function*/
	gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_15);
	gpio_init(GPIOB,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_3|GPIO_PIN_10|GPIO_PIN_11);
	gpio_init(GPIOC,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9);
	gpio_init(GPIOD,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
	gpio_init(GPIOE,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_9 | GPIO_PIN_11);
	
	gpio_pin_remap_config(GPIO_TIMER0_FULL_REMAP, ENABLE);
	gpio_pin_remap_config(GPIO_TIMER1_FULL_REMAP, ENABLE);	
	gpio_pin_remap_config(GPIO_TIMER2_FULL_REMAP, ENABLE);
	gpio_pin_remap_config(GPIO_TIMER3_REMAP, ENABLE);
	
	gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);	//JTAG-DP disabled and SW-DP enabled	

	timer_deinit(TIMER1);

	/* TIMER0 configuration */ 
	timer_initpara.prescaler         = (1200 - 1);
	timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
	timer_initpara.counterdirection  = TIMER_COUNTER_UP;
	timer_initpara.period            = (1000 - 1);
	timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
	timer_initpara.repetitioncounter = 0;
	// 初始化定时器0
	timer_init(TIMER0,&timer_initpara);		/*5ms Period*/
	
	/* TIMER1/2 configuration */ 
	timer_initpara.prescaler         = (120 - 1);
	timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
	timer_initpara.counterdirection  = TIMER_COUNTER_UP;
	timer_initpara.period            = (1000 - 1);
	timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
	timer_initpara.repetitioncounter = 0;
	// 初始化定时器1
	timer_init(TIMER1, &timer_initpara);	/*5ms Period*/
	//初始化定时器2
	timer_init(TIMER2, &timer_initpara);	/*5ms Period*/

	timer_initpara.prescaler         = PWM_PRSC;
	timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
//	timer_initpara.alignedmode = TIMER_COUNTER_CENTER_BOTH;
	timer_initpara.counterdirection  = TIMER_COUNTER_UP;
	timer_initpara.period            = (PWM_PERIOD - 1);
	timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
	timer_initpara.repetitioncounter = 0;
	// 初始化定时器3
	timer_init(TIMER3, &timer_initpara);	/*125us Period, 8Khz*/
	
	timer_master_output_trigger_source_select(TIMER3, TIMER_TRI_OUT_SRC_UPDATE);

	/* CH0,CH1 and CH2 configuration in PWM mode */
	timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
	timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
	timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_LOW;
	timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_LOW;
	timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_HIGH;
	timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_HIGH;

	timer_channel_output_config(TIMER0,TIMER_CH_0,&timer_ocintpara);
	timer_channel_output_config(TIMER0,TIMER_CH_1,&timer_ocintpara);
//	timer_channel_output_config(TIMER0,TIMER_CH_2,&timer_ocintpara);
//	timer_channel_output_config(TIMER0,TIMER_CH_3,&timer_ocintpara);

	timer_channel_output_config(TIMER1,TIMER_CH_0,&timer_ocintpara);
	timer_channel_output_config(TIMER1,TIMER_CH_1,&timer_ocintpara);
	timer_channel_output_config(TIMER1,TIMER_CH_2,&timer_ocintpara);
	timer_channel_output_config(TIMER1,TIMER_CH_3,&timer_ocintpara);

	timer_channel_output_config(TIMER2,TIMER_CH_0,&timer_ocintpara);
	timer_channel_output_config(TIMER2,TIMER_CH_1,&timer_ocintpara);
	timer_channel_output_config(TIMER2,TIMER_CH_2,&timer_ocintpara);
	timer_channel_output_config(TIMER2,TIMER_CH_3,&timer_ocintpara);

//	timer_channel_output_config(TIMER3,TIMER_CH_0,&timer_ocintpara);
//	timer_channel_output_config(TIMER3,TIMER_CH_1,&timer_ocintpara);
//	timer_channel_output_config(TIMER3,TIMER_CH_2,&timer_ocintpara);
//	timer_channel_output_config(TIMER3,TIMER_CH_3,&timer_ocintpara);

	/* CH0 configuration in PWM mode0,duty cycle 0% */
	timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,0);
	timer_channel_output_mode_config(TIMER0,TIMER_CH_0,TIMER_OC_MODE_PWM0);
	timer_channel_output_shadow_config(TIMER0,TIMER_CH_0,TIMER_OC_SHADOW_DISABLE);

	/* CH1 configuration in PWM mode0,duty cycle 0% */
	timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,0);
	timer_channel_output_mode_config(TIMER0,TIMER_CH_1,TIMER_OC_MODE_PWM0);
	timer_channel_output_shadow_config(TIMER0,TIMER_CH_1,TIMER_OC_SHADOW_DISABLE);

//	/* CH2 configuration in PWM mode0,duty cycle 0% */
//	timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,0);
//	timer_channel_output_mode_config(TIMER0,TIMER_CH_2,TIMER_OC_MODE_PWM0);
//	timer_channel_output_shadow_config(TIMER0,TIMER_CH_2,TIMER_OC_SHADOW_DISABLE);

//	/* CH3 configuration in PWM mode0,duty cycle 0% */
//	timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_3,0);
//	timer_channel_output_mode_config(TIMER0,TIMER_CH_3,TIMER_OC_MODE_PWM0);
//	timer_channel_output_shadow_config(TIMER0,TIMER_CH_3,TIMER_OC_SHADOW_DISABLE);

	/* CH0 configuration in PWM mode0,duty cycle 0% */
	timer_channel_output_pulse_value_config(TIMER1,TIMER_CH_0,0);
	timer_channel_output_mode_config(TIMER1,TIMER_CH_0,TIMER_OC_MODE_PWM0);
	timer_channel_output_shadow_config(TIMER1,TIMER_CH_0,TIMER_OC_SHADOW_DISABLE);

	/* CH1 configuration in PWM mode0,duty cycle 0% */
	timer_channel_output_pulse_value_config(TIMER1,TIMER_CH_1,0);
	timer_channel_output_mode_config(TIMER1,TIMER_CH_1,TIMER_OC_MODE_PWM0);
	timer_channel_output_shadow_config(TIMER1,TIMER_CH_1,TIMER_OC_SHADOW_DISABLE);

	/* CH2 configuration in PWM mode0,duty cycle 0% */
	timer_channel_output_pulse_value_config(TIMER1,TIMER_CH_2,0);
	timer_channel_output_mode_config(TIMER1,TIMER_CH_2,TIMER_OC_MODE_PWM0);
	timer_channel_output_shadow_config(TIMER1,TIMER_CH_2,TIMER_OC_SHADOW_DISABLE);

	/* CH3 configuration in PWM mode0,duty cycle 0% */
	timer_channel_output_pulse_value_config(TIMER1,TIMER_CH_3,0);
	timer_channel_output_mode_config(TIMER1,TIMER_CH_3,TIMER_OC_MODE_PWM0);
	timer_channel_output_shadow_config(TIMER1,TIMER_CH_3,TIMER_OC_SHADOW_DISABLE);

	/* CH0 configuration in PWM mode0,duty cycle 0% */
	timer_channel_output_pulse_value_config(TIMER2,TIMER_CH_0,0);
	timer_channel_output_mode_config(TIMER2,TIMER_CH_0,TIMER_OC_MODE_PWM0);
	timer_channel_output_shadow_config(TIMER2,TIMER_CH_0,TIMER_OC_SHADOW_DISABLE);

	/* CH1 configuration in PWM mode0,duty cycle 0% */
	timer_channel_output_pulse_value_config(TIMER2,TIMER_CH_1,0);
	timer_channel_output_mode_config(TIMER2,TIMER_CH_1,TIMER_OC_MODE_PWM0);
	timer_channel_output_shadow_config(TIMER2,TIMER_CH_1,TIMER_OC_SHADOW_DISABLE);

	/* CH2 configuration in PWM mode0,duty cycle 0% */
	timer_channel_output_pulse_value_config(TIMER2,TIMER_CH_2,0);
	timer_channel_output_mode_config(TIMER2,TIMER_CH_2,TIMER_OC_MODE_PWM0);
	timer_channel_output_shadow_config(TIMER2,TIMER_CH_2,TIMER_OC_SHADOW_DISABLE);
	
	timer_channel_output_pulse_value_config(TIMER2,TIMER_CH_3,0);
	timer_channel_output_mode_config(TIMER2,TIMER_CH_3,TIMER_OC_MODE_PWM0);
	timer_channel_output_shadow_config(TIMER2,TIMER_CH_3,TIMER_OC_SHADOW_DISABLE);

//	/* CH3 configuration in PWM mode0,duty cycle 0% */
//	timer_channel_output_pulse_value_config(TIMER3,TIMER_CH_3,0);
//	timer_channel_output_mode_config(TIMER3,TIMER_CH_3,TIMER_OC_MODE_PWM0);
//	timer_channel_output_shadow_config(TIMER3,TIMER_CH_3,TIMER_OC_SHADOW_DISABLE);

	timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
	timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
	timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
	timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
	timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
	timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
	
	timer_channel_output_config(TIMER3,TIMER_CH_0,&timer_ocintpara);
	timer_channel_output_config(TIMER3,TIMER_CH_1,&timer_ocintpara);
	timer_channel_output_config(TIMER3,TIMER_CH_2,&timer_ocintpara);
	timer_channel_output_config(TIMER3,TIMER_CH_3,&timer_ocintpara);
	/* CH0 configuration in PWM mode0,duty cycle 0% */
	timer_channel_output_pulse_value_config(TIMER3,TIMER_CH_0, PROP_PWM_PERIOD);
	timer_channel_output_mode_config(TIMER3,TIMER_CH_0,TIMER_OC_MODE_PWM0);
	timer_channel_output_shadow_config(TIMER3,TIMER_CH_0,TIMER_OC_SHADOW_DISABLE);

	/* CH1 configuration in PWM mode0,duty cycle 0% */
	timer_channel_output_pulse_value_config(TIMER3,TIMER_CH_1, PROP_PWM_PERIOD);
	timer_channel_output_mode_config(TIMER3,TIMER_CH_1,TIMER_OC_MODE_PWM0);
	timer_channel_output_shadow_config(TIMER3,TIMER_CH_1,TIMER_OC_SHADOW_DISABLE);

	/* CH2 configuration in PWM mode0,duty cycle 0% */
	timer_channel_output_pulse_value_config(TIMER3,TIMER_CH_2, PROP_PWM_PERIOD);
	timer_channel_output_mode_config(TIMER3,TIMER_CH_2,TIMER_OC_MODE_PWM0);
	timer_channel_output_shadow_config(TIMER3,TIMER_CH_2,TIMER_OC_SHADOW_DISABLE);

	/* CH3 configuration in PWM mode0,duty cycle 0% */
	timer_channel_output_pulse_value_config(TIMER3,TIMER_CH_3, PROP_PWM_PERIOD);
	timer_channel_output_mode_config(TIMER3,TIMER_CH_3,TIMER_OC_MODE_PWM0);
	timer_channel_output_shadow_config(TIMER3,TIMER_CH_3,TIMER_OC_SHADOW_DISABLE);

	/* TIMER0 primary output function enable */
	timer_primary_output_config(TIMER0,ENABLE);
	timer_primary_output_config(TIMER1,ENABLE);	
	timer_primary_output_config(TIMER2,ENABLE);
	timer_primary_output_config(TIMER3,ENABLE);
	/* auto-reload preload enable */
	timer_auto_reload_shadow_enable(TIMER0);
	timer_auto_reload_shadow_enable(TIMER1);
	timer_auto_reload_shadow_enable(TIMER2);
	timer_auto_reload_shadow_enable(TIMER3);
	/* auto-reload preload enable */
	timer_enable(TIMER0);	
	timer_enable(TIMER1);
	timer_enable(TIMER2);
	timer_enable(TIMER3);
	
		
}


void PropDriverPwmUpdate(PWM_GEN *p, uint8_t u8Channel)
{
	uint16_t MPeriod;
	int32_t Tmp;

	/* Compute the timer period (Q0) from the period modulation input (Q15) */
	/* MfuncPeriod = 0x7FFF */
	/* Q15 = Q0*Q15 */
	Tmp = (INT32U)p->PeriodMax * (INT32U)p->MfuncPeriod;
	/* Q0 = (Q15->Q0)/2 + (Q0/2) */
	MPeriod = (INT16S)(Tmp >> 16) + (INT16S)(p->PeriodMax >> 1);
	//TIM_SetAutoreload(TIM3,MPeriod);
	
	/* Compute the compare 2 (Q0) from the PWM 3&4 duty cycle ratio (Q15) */
	/* Q15 = Q0*Q15 */
	Tmp = (int32_t)MPeriod * (int32_t)p->MfuncC;
	/* Q0 = (Q15->Q0)/2 + (Q0/2) */
	Tmp = Tmp >> 15;
	
#if 0	
//#ifdef  PROP_DITHER
	p->u16DitherCnt++;
	if (p->u16DitherCnt >= p->u16DitherPeriod)
	{
		p->u16DitherCnt = 0;
	}
	
	if(p->u16DitherCnt < (p->u16DitherPeriod >> 1))
	{
		p->i16DitherValue += p->u16DitherRatio;
	}
	else
	{
		p->i16DitherValue -= p->u16DitherRatio;
	}
	
	if(p->i16DitherValue < 0)
	{
		p->i16DitherValue = 0;
	}
	
	if(0 != Tmp)
	{
		Tmp += p->i16DitherValue;
	}
#endif
	if(Tmp >= PWM_PLUS_MAX_VALUE)
	{
		Tmp = PWM_PLUS_MAX_VALUE;
	}
	switch(u8Channel)
	{
		case 0:
			if (Tmp <= p->PeriodMax)
			{
				TIMER_CH0CV(TIMER3) = p->PeriodMax - (INT16S)(Tmp);
				TIMER_CH2CV(TIMER3) = p->PeriodMax - (INT16S)(Tmp);
//				TIMER_CH2CV(TIMER3) = p->PeriodMax - (INT16S)(Tmp) + 60;
//				if (Tmp < 75)
//				{
//					TIMER_CH2CV(TIMER3) = p->PeriodMax - 75;
//				}
//				else
//				{
//					TIMER_CH2CV(TIMER3) = p->PeriodMax - (INT16S)(Tmp) + 60;
//				}
//				TIMER_CH0CV(TIMER3) = p->PeriodMax - (INT16S)(Tmp);
//				TIMER_CH2CV(TIMER3) = (p->PeriodMax - (INT16S)(Tmp)) * 0.95;
			}
			else
			{
				TIMER_CH0CV(TIMER3) = 0;
				TIMER_CH2CV(TIMER3) = 0;
			}
			break;
		case 1:
			if (Tmp <= p->PeriodMax)
			{
				TIMER_CH1CV(TIMER3) = p->PeriodMax - (INT16S)(Tmp);
				TIMER_CH3CV(TIMER3) = p->PeriodMax - (INT16S)(Tmp);
			}
			else
			{
				TIMER_CH1CV(TIMER3) = 0;
				TIMER_CH3CV(TIMER3) = 0;
				
			}
			break;	
		default:
			break;
	}
}


void vPwmDriver(uint8_t u8Channel, uint16_t u16PluseWidth)
{
	switch(u8Channel)
	{
		case 0:
			TIMER_CH0CV(TIMER2) = u16PluseWidth;	//Drv1
			break;
		case 1:
			TIMER_CH1CV(TIMER2) = u16PluseWidth;	//Drv2
			break;
		case 2:
			TIMER_CH2CV(TIMER2) = u16PluseWidth;	//Drv3
			break;
		case 3:
			TIMER_CH3CV(TIMER2) = u16PluseWidth;	//Drv4
			break;
		case 4:
			TIMER_CH0CV(TIMER1) = u16PluseWidth;	//Drv5
			break;
		case 5:
			TIMER_CH1CV(TIMER1) = u16PluseWidth;	//Drv6
			break;
		case 6:
			TIMER_CH2CV(TIMER1) = u16PluseWidth;	//Drv7
			break;
		case 7:
			TIMER_CH3CV(TIMER1) = u16PluseWidth;	//Drv8
			break;
		case 8:
			TIMER_CH0CV(TIMER0) = u16PluseWidth;	//Drv9
			break;
		case 9:
			TIMER_CH1CV(TIMER0) = u16PluseWidth;	//Drv10
			break;

		default:
			break;
	}
}


