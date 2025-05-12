/*******************************************************************************
* Filename: Device.c                                             	 		   *
*                                                                    		   *
* Description:	Device driver routine.	Platform dependent.			 		   *
* Author:                                                           		   *
* Date:     														 		   *
* Revision:															 		   *
*																	 		   *
*******************************************************************************/

#include "CommonRam.h"
#include "Device.h"

//EXTI_HandleTypeDef hexti_PLC;
INT32U ADCValue[ADC1_NUM];
INT32U ADC2Value[ADC2_NUM];
INT32U ADC3Value[ADC3_NUM];
uint16_t inserted_data[3];

void InitGPIO(void)
{
		/* enable GPIO clock */
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_GPIOC);
	rcu_periph_clock_enable(RCU_GPIOD);
	rcu_periph_clock_enable(RCU_GPIOE);
	rcu_periph_clock_enable(RCU_GPIOF);
	rcu_periph_clock_enable(RCU_AF);
	
	/*******************************************************************************
	* SPI
	*管脚51	PC10 SPI2_SCK	CLK;	管脚52	PC11 SPI2_MISO	DO;
	*管脚53	PC12 SPI2_MOSI DI;	管脚54	PD0 CS;
	*******************************************************************************/
    rcu_periph_clock_enable(RCU_SPI2);
		
	/* SPI0 GPIO config:SCK/PA5, MISO/PA6, MOSI/PA7 */
	gpio_init(GPIOC, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10 | GPIO_PIN_12);
	gpio_init(GPIOC, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_11);
	/* PD0 as NSS */
	gpio_init(GPIOD, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
	
	gpio_pin_remap_config(GPIO_SPI2_REMAP, ENABLE);
	
	spi_parameter_struct spi_init_struct;
	/* SPI0 parameter config */
	spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
	spi_init_struct.device_mode          = SPI_MASTER;
	spi_init_struct.frame_size           = SPI_FRAMESIZE_16BIT;
	spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
	spi_init_struct.nss                  = SPI_NSS_SOFT;
	spi_init_struct.prescale             = SPI_PSC_32;
	spi_init_struct.endian               = SPI_ENDIAN_MSB;
	spi_init(SPI2, &spi_init_struct);
	
	spi_enable(SPI2);	//使能SPI
	gpio_bit_write(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
				
}

void InitSysCtrl(void)
{
	timer_oc_parameter_struct timer_ocintpara;
	timer_parameter_struct timer_initpara;
	timer_break_parameter_struct timer_breakpara;
	timer_ic_parameter_struct timer_icinitpara;
	
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_GPIOE);
    rcu_periph_clock_enable(RCU_GPIOF);
	rcu_periph_clock_enable(RCU_AF);

	/*******************************************************************************
	* DI	
	*管脚26	 PA3		SWI1_R; 	管脚3	PE4		SWI2_R; 	
	*管脚93	 PB7		SWI3_R; 	管脚92	PB6		SWI4_R; 
	*管脚53	 PB14  	SWI5_R;			管脚7	PC13  	SWI6_R;	
	*管脚8	 PC14  	SWI7_R;			管脚9	PC15  	SWI8_R;		
	*管脚52	 PB13  	SWI9_R;			管脚51	PB12  	SWI10_R;		
	*管脚88	 PD7  	SWI11_R;		管脚87	PD6  	SWI12_R;	
	*管脚66	 PC9  	SWI13_R;		管脚65	PC8  	SWI14_R;
	*管脚98  PE1	Driver1_R;  	管脚97  PE0	Driver2_R;
	*管脚5   PE6	Driver3_R;		管脚67  PA8	Driver4_R;
	*管脚4   PE5	Driver5_R;
	*管脚96  PB9	DO1_SHUT;  		管脚95  PB8	DO2_SHUT;
	*管脚85  PD4	DO3_SHUT;	 	管脚84  PD3	DO4_SHUT;
	*管脚83  PD2	DO5_SHUT;
	*******************************************************************************/
	gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_3|GPIO_PIN_8);
	gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14);
	gpio_init(GPIOC, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
	gpio_init(GPIOD, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7);
	gpio_init(GPIOE, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6);		
	/*******************************************************************************
	* DO
	*管脚64	 PC7  OVERI_RESET 
	*管脚1	 PE2	LED_R;    管脚2	 PE3	LED_Y;   	管脚38	PE7	Driver_EN;
	*管脚45  PE14 THROTTLE1;  管脚46   PE15 POT_LOW;
	*管脚55  PD8	S2;   管脚56   PD9 S1;  	管脚57   PD10 S0;
	*管脚81  PD0  SPI_CS;  管脚82  PD1	THROTTLE2;  管脚86  PD5	CHARGE;
	*******************************************************************************/
	gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);
	gpio_init(GPIOD, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10);	
	gpio_init(GPIOE, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_7|GPIO_PIN_14|GPIO_PIN_15);	

	//gpio_bit_write(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
	gpio_bit_write(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET);
	/**************************************************************************
	*DO PWM: RELAY 200hz  DRIVER1~4 200HZ  
	*管脚77  PA15	TIMER1_CH0 Driver1;		管脚89  PB3 TIMER1_CH1	Driver2; 
	*管脚47  PB10	TIMER1_CH2 Driver3;		管脚48  PB11 TIMER1_CH3	Driver4;  	
	*管脚23  PA0	TIMER4_CH0 Driver5_prop1;			管脚24  PA1 TIMER4_CH1	Driver5_prop;  
	**************************************************************************/
    rcu_periph_clock_enable(RCU_TIMER1); 
	
    /*Configure PA0 PA1 PA2(TIMER1 CH0 CH1 CH2 CH3) as alternate function*/
    gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_15);
    gpio_init(GPIOB,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_3|GPIO_PIN_10|GPIO_PIN_11);
		
	gpio_pin_remap_config(GPIO_TIMER1_FULL_REMAP, ENABLE);	//
	gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);	//JTAG-DP disabled and SW-DP enabled	
		
    timer_deinit(TIMER1);

    /* TIMER1 configuration */
    timer_initpara.prescaler         = 300-1;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 1000;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER1,&timer_initpara);

    /* CH0,CH1 and CH2 configuration in PWM mode */
	timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
	timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_LOW;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_LOW;
	timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
	timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
		
    timer_channel_output_config(TIMER1,TIMER_CH_0,&timer_ocintpara);
    timer_channel_output_config(TIMER1,TIMER_CH_1,&timer_ocintpara);
    timer_channel_output_config(TIMER1,TIMER_CH_2,&timer_ocintpara);
    timer_channel_output_config(TIMER1,TIMER_CH_3,&timer_ocintpara);
		
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
		
    /* TIMER0 primary output function enable */
    timer_primary_output_config(TIMER1,ENABLE);
		
		/* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER1);
    /* auto-reload preload enable */
    timer_enable(TIMER1);	
		
#define PROP_DRIVER
#ifdef	PROP_DRIVER
    /*Configure PA0 PA1 PA2(TIMER4 CH0 CH1 ) as alternate function*/
    gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_0|GPIO_PIN_1);
    rcu_periph_clock_enable(RCU_TIMER4);

    timer_deinit(TIMER4);

    /* TIMER2 configuration */
    timer_initpara.prescaler         = 300-1;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 1000;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER4,&timer_initpara);

    /* CH0,CH1 and CH2 configuration in PWM mode */
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_LOW;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_LOW;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(TIMER4,TIMER_CH_0,&timer_ocintpara);
    timer_channel_output_config(TIMER4,TIMER_CH_1,&timer_ocintpara);

    /* CH0 configuration in PWM mode0,duty cycle 0% */
    timer_channel_output_pulse_value_config(TIMER4,TIMER_CH_0,0);
    timer_channel_output_mode_config(TIMER4,TIMER_CH_0,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER4,TIMER_CH_0,TIMER_OC_SHADOW_DISABLE);

    /* CH1 configuration in PWM mode0,duty cycle 0% */
    timer_channel_output_pulse_value_config(TIMER4,TIMER_CH_1,0);
    timer_channel_output_mode_config(TIMER4,TIMER_CH_1,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER4,TIMER_CH_1,TIMER_OC_SHADOW_DISABLE);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER4);
    /* auto-reload preload enable */
    timer_enable(TIMER4);	
#endif	//#ifdef	PROP_DRIVER
		
	/*******************************************************************************
	* Encoder(T法)
	*管脚	PD12 TIMER3_CH0	CODE_A(HALL_U);	管脚	PD13 TIMER3_CH1	CODE_B(HALL_V);	管脚	PD14 TIMER3_CH2	HALL_W;
	*******************************************************************************/
    /*configure PD12,13 (TIMER3 CH0,CH1) as alternate function*/
	rcu_periph_clock_enable(RCU_TIMER3);
		
    gpio_init(GPIOD,GPIO_MODE_IN_FLOATING,GPIO_OSPEED_50MHZ,GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14);
		
	gpio_pin_remap_config(GPIO_TIMER3_REMAP, ENABLE);

	timer_deinit(TIMER3);       

	timer_initpara.period = 119;
	timer_initpara.prescaler = 65535;
	timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
	timer_initpara.counterdirection  = TIMER_COUNTER_UP;
	timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
	timer_initpara.repetitioncounter = 0;    
	timer_init(TIMER3, &timer_initpara);
 
	/* TIMER3 CH0 input capture configuration */
	timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
	timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
	timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
	timer_icinitpara.icfilter    = 0x05;
	timer_input_capture_config(TIMER3,TIMER_CH_0,&timer_icinitpara);               
	timer_input_capture_config(TIMER3,TIMER_CH_1,&timer_icinitpara);
	timer_input_capture_config(TIMER3,TIMER_CH_2,&timer_icinitpara);
		
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER3);	 
    /* clear channel 0~2 interrupt bit */
    timer_interrupt_flag_clear(TIMER3,TIMER_INT_FLAG_CH0);
    timer_interrupt_flag_clear(TIMER3,TIMER_INT_FLAG_CH1);
    timer_interrupt_flag_clear(TIMER3,TIMER_INT_FLAG_CH2);
    /* channel 0~2 interrupt enable */
    timer_interrupt_enable(TIMER3,TIMER_INT_CH0);  
    timer_interrupt_enable(TIMER3,TIMER_INT_CH1); 
    timer_interrupt_enable(TIMER3,TIMER_INT_CH2); 

    //nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(TIMER3_IRQn, 1, 0);
		
	/* auto-reload preload enable */
	//timer_auto_reload_shadow_enable(TIMER3);
	timer_enable(TIMER3);		
		
	/*******************************************************************************
	* Encoder(M法)
	*管脚	PB4 TIMER2_CH0	CODE_A(HALL_U);管脚	PB5 TIMER2_CH1	CODE_B(HALL_V);
	*******************************************************************************/		
	/* TIM2 clock enable */       
	rcu_periph_clock_enable(RCU_TIMER2);
	  
    gpio_init(GPIOB,GPIO_MODE_IN_FLOATING,GPIO_OSPEED_50MHZ,GPIO_PIN_4|GPIO_PIN_5);
		
	gpio_pin_remap_config(GPIO_TIMER2_PARTIAL_REMAP, ENABLE);
		
	timer_deinit(TIMER2);       

	timer_initpara.period = 3;
	timer_initpara.prescaler = 65535;
	timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
	timer_initpara.counterdirection  = TIMER_COUNTER_UP;
	timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
	timer_initpara.repetitioncounter = 0;       
	   
	timer_init(TIMER2, &timer_initpara);
 
	/* TIMER3 CH0 input capture configuration */
	timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
	timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
	timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
	timer_icinitpara.icfilter    = 0x05;
	timer_input_capture_config(TIMER2,TIMER_CH_0,&timer_icinitpara);               
	timer_input_capture_config(TIMER2,TIMER_CH_1,&timer_icinitpara);
 
	timer_quadrature_decoder_mode_config(TIMER2,TIMER_ENCODER_MODE1,TIMER_IC_POLARITY_RISING,TIMER_IC_POLARITY_RISING);
 
	//timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_CH0);
	//timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_CH1);               
	//timer_interrupt_enable(TIMER2, TIMER_INT_UP);
	//timer_interrupt_enable(TIMER2, TIMER_INT_CH0 |TIMER_INT_CH1);       
 
	/* auto-reload preload enable */
	//timer_auto_reload_shadow_enable(TIMER2);
	timer_enable(TIMER2);		
	
	/*******************************************************************************
	* SVPWM
	*管脚40	PE9  TIM0_CH1	U+;  管脚39	PE8 TIM0_CH1N	U-;
	*管脚42	PE11 TIM0_CH2	V+;  管脚41	PE10 TIM0_CH2N	V-;
	*管脚44	PE13 TIM0_CH3	W+;  管脚43	PE12 TIM0_CH3N	W-;
	*******************************************************************************/	
    rcu_periph_clock_enable(RCU_TIMER0);
		
    /*configure PE9 PE11 PE13(TIMER0 CH0 CH1 CH2) as alternate function*/
    gpio_init(GPIOE,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_13);

    /*configure PE8 PE10 PE12(TIMER0 CH0N CH1N CH2N) as alternate function*/
    gpio_init(GPIOE,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_12);
		
	gpio_pin_remap_config(GPIO_TIMER0_FULL_REMAP, ENABLE);

    timer_deinit(TIMER0);

    /* TIMER0 configuration */
    timer_initpara.prescaler         = PWM_PRSC;
    timer_initpara.alignedmode       = TIMER_COUNTER_CENTER_DOWN;
    timer_initpara.counterdirection  = TIMER_COUNTER_DOWN;
    timer_initpara.period            = PWM_PERIOD;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV2;
    timer_initpara.repetitioncounter = REP_RATE;
    timer_init(TIMER0,&timer_initpara);

     /* CH0/CH0N,CH1/CH1N and CH2/CH2N configuration in timing mode */
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_ENABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(TIMER0,TIMER_CH_0,&timer_ocintpara);
    timer_channel_output_config(TIMER0,TIMER_CH_1,&timer_ocintpara);
    timer_channel_output_config(TIMER0,TIMER_CH_2,&timer_ocintpara);
    timer_channel_output_config(TIMER0,TIMER_CH_3,&timer_ocintpara);	
		
	/* select TIMER master mode output trigger source */
	timer_master_output_trigger_source_select(TIMER0, TIMER_TRI_OUT_SRC_O3CPRE);	
	/* configure TIMER master slave mode */
	timer_master_slave_mode_config(TIMER0, TIMER_MASTER_SLAVE_MODE_DISABLE);	
		
		
    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_0,PWM_PERIOD/2);
    timer_channel_output_mode_config(TIMER0,TIMER_CH_0,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0,TIMER_CH_0,TIMER_OC_SHADOW_ENABLE);

    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,PWM_PERIOD/2);
    timer_channel_output_mode_config(TIMER0,TIMER_CH_1,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0,TIMER_CH_1,TIMER_OC_SHADOW_ENABLE);

    timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_2,PWM_PERIOD/2);
    timer_channel_output_mode_config(TIMER0,TIMER_CH_2,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0,TIMER_CH_2,TIMER_OC_SHADOW_ENABLE);
		
	timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_3,(PWM_PERIOD/32 + (PWM_PERIOD*8/62)));
    timer_channel_output_mode_config(TIMER0,TIMER_CH_3,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER0,TIMER_CH_3,TIMER_OC_SHADOW_ENABLE);

    /* automatic output enable, break, dead time and lock configuration*/
    timer_breakpara.runoffstate      = TIMER_ROS_STATE_DISABLE;
    timer_breakpara.ideloffstate     = TIMER_IOS_STATE_DISABLE ;
    timer_breakpara.deadtime         = DEADTIME;
    timer_breakpara.breakpolarity    = TIMER_BREAK_POLARITY_LOW;
    timer_breakpara.outputautostate  = TIMER_OUTAUTO_ENABLE;
    timer_breakpara.protectmode      = TIMER_CCHP_PROT_OFF;
    timer_breakpara.breakstate       = TIMER_BREAK_DISABLE;
    timer_break_config(TIMER0,&timer_breakpara);
    
    /* TIMER0 primary output function enable */
    timer_primary_output_config(TIMER0,ENABLE);

    /* TIMER0 channel control update interrupt enable */
//    timer_interrupt_enable(TIMER0,TIMER_INT_UP);
    /* TIMER0 break interrupt disable */
    timer_interrupt_disable(TIMER0,TIMER_INT_BRK);

    /* TIMER0 counter enable */
//    timer_enable(TIMER0);
		
	nvic_irq_enable(TIMER0_UP_IRQn,0,3);
	timer_flag_clear(TIMER0, TIMER_FLAG_UP);
		
	/*******************************************************************************
	* PUMP
	*管脚63	PC6  TIM7_CH0	P+;  管脚32	PA7 TIM7_CH0N	P-;
	*******************************************************************************/
	rcu_periph_clock_enable(RCU_TIMER7);
		
    /*configure PC6(TIMER7 CH0) as alternate function*/
    gpio_init(GPIOC,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_6);

    /*configure PA7(TIMER7 CH0N) as alternate function*/
    gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_7);

    timer_deinit(TIMER7);

    /* TIMER0 configuration */
    timer_initpara.prescaler         = PWM_PRSC;
    timer_initpara.alignedmode       = TIMER_COUNTER_CENTER_DOWN;
    timer_initpara.counterdirection  = TIMER_COUNTER_DOWN;
    timer_initpara.period            = PWM_PERIOD;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV2;
    timer_initpara.repetitioncounter = REP_RATE;
    timer_init(TIMER7,&timer_initpara);

     /* CH0/CH0N,CH1/CH1N and CH2/CH2N configuration in timing mode */
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_ENABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(TIMER7,TIMER_CH_0,&timer_ocintpara);

    timer_channel_output_pulse_value_config(TIMER7,TIMER_CH_0,PROP_PWM_PERIOD);
    timer_channel_output_mode_config(TIMER7,TIMER_CH_0,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER7,TIMER_CH_0,TIMER_OC_SHADOW_ENABLE);

    /* automatic output enable, break, dead time and lock configuration*/
    timer_breakpara.runoffstate      = TIMER_ROS_STATE_DISABLE;
    timer_breakpara.ideloffstate     = TIMER_IOS_STATE_DISABLE ;
    timer_breakpara.deadtime         = DEADTIME;
    timer_breakpara.breakpolarity    = TIMER_BREAK_POLARITY_LOW;
    timer_breakpara.outputautostate  = TIMER_OUTAUTO_ENABLE;
    timer_breakpara.protectmode      = TIMER_CCHP_PROT_OFF;
    timer_breakpara.breakstate       = TIMER_BREAK_DISABLE;
    timer_break_config(TIMER7,&timer_breakpara);
    
    /* TIMER7 primary output function enable */
    timer_primary_output_config(TIMER7,ENABLE);

    /* TIMER7 channel control update interrupt enable */
    timer_interrupt_enable(TIMER7,TIMER_INT_CMT);
    /* TIMER7 break interrupt disable */
    timer_interrupt_disable(TIMER7,TIMER_INT_BRK);

    /* TIMER7 counter enable */
    timer_enable(TIMER7);	
	
	/*******************************************************************************
	* AD
	*Hareware V01
	管脚14	PA0 ADC012_IN0	IU-R/AD; 	管脚15	PA1 ADC012_IN1	IW-R/AD;  
	管脚16	PA2 ADC012_IN2	IP-R/AD;	管脚17	PA3 ADC012_IN3	SW1-R/AD;
	管脚17	PA4 ADC01_IN4	TMP-R/AD;		管脚21	PA5 ADC01_IN5		MOTOR-TMP-R/AD;
	管脚21	PA7 ADC01_IN6		5V-12V-OUT-R/AD;  	管脚23	PB0 ADC01_IN8		KSI/AD; 
	管脚23	PB1 ADC01_IN9		VBUS/AD;
	管脚8	  PC0 ADC012_IN10 SIN-R/AD; 	管脚9	PC1 ADC012_IN11	COS-R/AD; 
	管脚10	PC2 ADC012_IN12	WIPER1/AD;  管脚11	PC3 ADC012_IN13	WIPER2/AD
	管脚10	PC4 ADC01_IN14	PROP-CHECK/AD;  管脚11	PC5 ADC01_IN15	POT1-HIGH/AD
	*Hareware V02
	管脚25	PA2 ADC012_IN2	IP-R/AD;	管脚26	PA3 ADC012_IN3	SW1-R/AD;
	管脚29	PA4 ADC01_IN4	TMP-R/AD;		管脚30	PA5 ADC01_IN5		MOTOR-TMP-R/AD;
	管脚31	PA6 ADC01_IN6		5V-12V-OUT-R/AD; 管脚33	PC4 ADC01_IN14	PROP-CHECK/AD;
	管脚35	PB0 ADC01_IN8		KSI/AD;		管脚36	PB1 ADC01_IN9		VBUS/AD;
	管脚15	PC0 ADC012_IN10 IU-R/AD; 	管脚16	PC1 ADC012_IN11	IW-R/AD; 
	管脚17	PC2 ADC012_IN12	WIPER1/AD;  管脚18	PC3 ADC012_IN13	WIPER2/AD
	管脚34	PC5 ADC01_IN15	ADC01_IN15/AD
	
	8选1模拟量开关
	CH_0	U-R/AD				CH_1	V-R/AD				CH_2	W-R/AD				CH_3	P-R/AD
	CH_4	5VOUT-R/AD		CH_5	12VOUT-R/AD		CH_6	POT1-HIGH/AD	CH_7	POT2-HIGH/AD	
	*******************************************************************************/
    /* enable ADC clock */
    rcu_periph_clock_enable(RCU_ADC0);
    rcu_periph_clock_enable(RCU_ADC1);
    rcu_periph_clock_enable(RCU_ADC2);
    /* enable DMA0 clock */
    rcu_periph_clock_enable(RCU_DMA0);
    rcu_periph_clock_enable(RCU_DMA1);
    /* config ADC clock */
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV4);	//ADC clk @30M
		
	{		
    /* config the GPIO as analog mode */
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
//    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_3);
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_5);
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
    gpio_init(GPIOB, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
    gpio_init(GPIOB, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
    gpio_init(GPIOC, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
    gpio_init(GPIOC, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
    gpio_init(GPIOC, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
    gpio_init(GPIOC, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_3);
    gpio_init(GPIOC, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
    gpio_init(GPIOC, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_5);
	}
	
	{
    /* ADC_DMA_channel configuration */
    dma_parameter_struct dma_data_parameter;
    
    /* ADC DMA_channel configuration */
    dma_deinit(DMA0, DMA_CH0);
    
    /* initialize DMA single data mode */
    dma_data_parameter.periph_addr  = (uint32_t)(&ADC_RDATA(ADC0));
    dma_data_parameter.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_data_parameter.memory_addr  = (uint32_t)(&ADCValue);
    dma_data_parameter.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_32BIT;
    dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_32BIT;  
    dma_data_parameter.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_data_parameter.number       = ADC1_NUM;
    dma_data_parameter.priority     = DMA_PRIORITY_HIGH;
    dma_init(DMA0, DMA_CH0, &dma_data_parameter);

    dma_circulation_enable(DMA0, DMA_CH0);
  
    /* enable DMA channel */
    dma_channel_enable(DMA0, DMA_CH0);
	}

	{
    /* ADC mode config */
    adc_mode_config(ADC_DAUL_REGULAL_PARALLEL); 
    /* ADC continuous function enable */
    adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, DISABLE);
    /* ADC scan function enable */
    adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
    /* ADC data alignment config */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);

    /* ADC channel length config */
    adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL, ADC1_NUM);

    /* ADC regular channel config */ 
//    adc_regular_channel_config(ADC0, 0, ADC_CHANNEL_3, ADC_SAMPLETIME_71POINT5);
    adc_regular_channel_config(ADC0, 0, ADC_CHANNEL_4, ADC_SAMPLETIME_71POINT5);
    adc_regular_channel_config(ADC0, 1, ADC_CHANNEL_5, ADC_SAMPLETIME_71POINT5);
    adc_regular_channel_config(ADC0, 2, ADC_CHANNEL_6, ADC_SAMPLETIME_71POINT5);
    adc_regular_channel_config(ADC0, 3, ADC_CHANNEL_8, ADC_SAMPLETIME_71POINT5);
    adc_regular_channel_config(ADC0, 4, ADC_CHANNEL_9, ADC_SAMPLETIME_71POINT5);
//    adc_regular_channel_config(ADC0, 6, ADC_CHANNEL_12, ADC_SAMPLETIME_71POINT5);
//    adc_regular_channel_config(ADC0, 7, ADC_CHANNEL_13, ADC_SAMPLETIME_71POINT5);
    adc_regular_channel_config(ADC0, 5, ADC_CHANNEL_14, ADC_SAMPLETIME_71POINT5);
    adc_regular_channel_config(ADC0, 6, ADC_CHANNEL_15, ADC_SAMPLETIME_71POINT5);
    /* ADC internal reference voltage channel config */
    adc_regular_channel_config(ADC0, 7, ADC_CHANNEL_17, ADC_SAMPLETIME_71POINT5);
		 
    /* ADC temperature and Vrefint enable */
    adc_tempsensor_vrefint_enable();
    
    /* ADC trigger config */
    adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE); /* software trigger */
    adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);

	//ADC0 Injected
	{
		/* ADC channel length config */
		adc_channel_length_config(ADC0, ADC_INSERTED_CHANNEL, 1);
		/* ADC inserted channel config */
		adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_10, ADC_SAMPLETIME_13POINT5);

		/* ADC trigger config */
		adc_external_trigger_source_config(ADC0, ADC_INSERTED_CHANNEL, ADC0_1_EXTTRIG_INSERTED_T0_TRGO); 
		/* ADC external trigger enable */
		adc_external_trigger_config(ADC0, ADC_INSERTED_CHANNEL, ENABLE);
		
		/* 8 times sample, 3 bits shift */
		adc_oversample_mode_config(ADC0, ADC_OVERSAMPLING_ALL_CONVERT, ADC_OVERSAMPLING_SHIFT_3B, ADC_OVERSAMPLING_RATIO_MUL8);
		adc_oversample_mode_enable(ADC0);    
		
		/* clear the ADC flag */
		adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOC);
		adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOIC);
		/* enable ADC interrupt */
		adc_interrupt_enable(ADC0, ADC_INT_EOIC);
	}

    /* enable ADC interface */
    adc_enable(ADC0);
    delay_us(100);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC0);

    /* ADC software trigger enable */
    //adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);
		
    /* ADC DMA function enable */
    adc_dma_mode_enable(ADC0);
		
	nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
	nvic_irq_enable(ADC0_1_IRQn, 0, 0);
	}	
	/************************************************/			
	//ADC1 INJECTED		
	/* ADC channel length config */
    adc_channel_length_config(ADC1, ADC_INSERTED_CHANNEL, 1);
    /* ADC inserted channel config */
    adc_inserted_channel_config(ADC1, 0, ADC_CHANNEL_11, ADC_SAMPLETIME_13POINT5);

    /* ADC trigger config */
    adc_external_trigger_source_config(ADC1, ADC_INSERTED_CHANNEL, ADC0_1_EXTTRIG_INSERTED_T0_TRGO); 
    /* ADC external trigger enable */
    adc_external_trigger_config(ADC1, ADC_INSERTED_CHANNEL, ENABLE);
		
    /* 8 times sample, 3 bits shift */
    adc_oversample_mode_config(ADC1, ADC_OVERSAMPLING_ALL_CONVERT, ADC_OVERSAMPLING_SHIFT_3B, ADC_OVERSAMPLING_RATIO_MUL8);
    adc_oversample_mode_enable(ADC1);    
		
    /* clear the ADC flag */
    adc_interrupt_flag_clear(ADC1, ADC_INT_FLAG_EOC);
    adc_interrupt_flag_clear(ADC1, ADC_INT_FLAG_EOIC);
    /* enable ADC interrupt */
    //adc_interrupt_enable(ADC1, ADC_INT_EOIC);

    /* ADC DMA function enable */
    //adc_dma_mode_enable(ADC1);
    /* enable ADC interface */
    adc_enable(ADC1);
    delay_us(100);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC1);	
		
	/************************************************/
	//ADC2 REGULAL
	{
    /* ADC_DMA_channel configuration */
    dma_parameter_struct dma_data_parameter;
    
    /* ADC DMA_channel configuration */
    dma_deinit(DMA1, DMA_CH4);
    
    /* initialize DMA single data mode */
    dma_data_parameter.periph_addr  = (uint32_t)(&ADC_RDATA(ADC2));
    dma_data_parameter.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_data_parameter.memory_addr  = (uint32_t)(&ADC2Value);
    dma_data_parameter.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_32BIT;
    dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_32BIT;  
    dma_data_parameter.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_data_parameter.number       = ADC2_NUM;
    dma_data_parameter.priority     = DMA_PRIORITY_HIGH;
    dma_init(DMA1, DMA_CH4, &dma_data_parameter);

    dma_circulation_enable(DMA1, DMA_CH4);
  
    /* enable DMA channel */
    dma_channel_enable(DMA1, DMA_CH4);
	}

	{
    /* ADC mode config */
    adc_mode_config(ADC_DAUL_REGULAL_PARALLEL); 
    /* ADC continuous function enable */
    adc_special_function_config(ADC2, ADC_CONTINUOUS_MODE, DISABLE);
    /* ADC scan function enable */
    adc_special_function_config(ADC2, ADC_SCAN_MODE, ENABLE);
    /* ADC data alignment config */
    adc_data_alignment_config(ADC2, ADC_DATAALIGN_RIGHT);

    /* ADC channel length config */
    adc_channel_length_config(ADC2, ADC_REGULAR_CHANNEL, ADC2_NUM);

    /* ADC regular channel config */ 
    adc_regular_channel_config(ADC2, 0, ADC_CHANNEL_3, ADC_SAMPLETIME_71POINT5);
    adc_regular_channel_config(ADC2, 1, ADC_CHANNEL_12, ADC_SAMPLETIME_71POINT5);
    adc_regular_channel_config(ADC2, 2, ADC_CHANNEL_13, ADC_SAMPLETIME_71POINT5);

    /* ADC trigger config */
    adc_external_trigger_source_config(ADC2, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE); /* software trigger */
    adc_external_trigger_config(ADC2, ADC_REGULAR_CHANNEL, ENABLE);	
	}
	
	//ADC2 INJECTED
	/* ADC channel length config */
    adc_channel_length_config(ADC2, ADC_INSERTED_CHANNEL, 1);
    /* ADC inserted channel config */
    adc_inserted_channel_config(ADC2, 0, ADC_CHANNEL_2, ADC_SAMPLETIME_13POINT5);

    /* ADC trigger config */
    adc_external_trigger_source_config(ADC2, ADC_INSERTED_CHANNEL, ADC0_1_EXTTRIG_INSERTED_T0_TRGO); 
    /* ADC external trigger enable */
    adc_external_trigger_config(ADC2, ADC_INSERTED_CHANNEL, ENABLE);
		
    /* 8 times sample, 3 bits shift */
    adc_oversample_mode_config(ADC2, ADC_OVERSAMPLING_ALL_CONVERT, ADC_OVERSAMPLING_SHIFT_3B, ADC_OVERSAMPLING_RATIO_MUL8);
    adc_oversample_mode_enable(ADC2);    
		
    /* clear the ADC flag */
    adc_interrupt_flag_clear(ADC2, ADC_INT_FLAG_EOC);
    adc_interrupt_flag_clear(ADC2, ADC_INT_FLAG_EOIC);
    /* enable ADC interrupt */
    //adc_interrupt_enable(ADC1, ADC_INT_EOIC);

    /* ADC DMA function enable */
    //adc_dma_mode_enable(ADC1);
    /* enable ADC interface */
    adc_enable(ADC2);
    delay_us(100);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC2);	
		
    /* ADC DMA function enable */
    adc_dma_mode_enable(ADC2);
		
//	/*******************************************************************************
//	* SPI
//	*管脚51	PC10 SPI2_SCK	CLK;	管脚52	PC11 SPI2_MISO	DO;
//	*管脚53	PC12 SPI2_MOSI DI;	管脚54	PD0 CS;
//	*******************************************************************************/
//    rcu_periph_clock_enable(RCU_SPI2);
//		
//		/* SPI0 GPIO config:SCK/PA5, MISO/PA6, MOSI/PA7 */
//		gpio_init(GPIOC, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10 | GPIO_PIN_12);
//		gpio_init(GPIOC, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_11);
//		/* PD0 as NSS */
//		gpio_init(GPIOD, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
//		
//		gpio_pin_remap_config(GPIO_SPI2_REMAP, ENABLE);
//		
//		spi_parameter_struct spi_init_struct;
//		/* SPI0 parameter config */
//		spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
//		spi_init_struct.device_mode          = SPI_MASTER;
//		spi_init_struct.frame_size           = SPI_FRAMESIZE_16BIT;
//		spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
//		spi_init_struct.nss                  = SPI_NSS_SOFT;
//		spi_init_struct.prescale             = SPI_PSC_32;
//		spi_init_struct.endian               = SPI_ENDIAN_MSB;
//		spi_init(SPI2, &spi_init_struct);
//		
//		spi_enable(SPI2);	//使能SPI
//		gpio_bit_write(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	
	/*******************************************************************************
	* CAN 500K
	*管脚61	PA11 CAN1_RX	;管脚62	PA12 CAN1_TX	;
	*******************************************************************************/ 
    /* enable CAN clock */
    rcu_periph_clock_enable(RCU_CAN0);    
    rcu_periph_clock_enable(RCU_GPIOA);
    
    /* configure CAN0 GPIO */
    gpio_init(GPIOA,GPIO_MODE_IPU,GPIO_OSPEED_50MHZ,GPIO_PIN_11);
    gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_12);

    can_parameter_struct            can_parameter;
    can_filter_parameter_struct     can_filter;
    
    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    can_struct_para_init(CAN_FILTER_STRUCT, &can_filter);
    /* initialize CAN register */
    can_deinit(CAN0);
    
    /* initialize CAN (CAN clocked at 60 MHz) */
    can_parameter.time_triggered = DISABLE;
    can_parameter.auto_bus_off_recovery = ENABLE;
    can_parameter.auto_wake_up = DISABLE;
    can_parameter.no_auto_retrans = DISABLE;
    can_parameter.rec_fifo_overwrite = DISABLE;
    can_parameter.trans_fifo_order = DISABLE;
    can_parameter.working_mode = CAN_NORMAL_MODE;
    can_parameter.resync_jump_width = CAN_BT_SJW_1TQ;
    can_parameter.time_segment_1 = CAN_BT_BS1_11TQ;
    can_parameter.time_segment_2 = CAN_BT_BS2_3TQ;

#ifdef CANBAUD_SET		//setting by switch
	if(READ_SW2())
	{	
		delay_us(10000);	//delay for 10ms	
		if(READ_SW2())
		{
			can_parameter.prescaler = 16;    /* baudrate 250Kbps */
		}
	}
	else 
	{
		can_parameter.prescaler = 32;    /* baudrate 125Kbps */
	}
#else		//setting by user
		#if (CAN_BAND_RATE	==	CAN_500KBPS)
				can_parameter.prescaler = 8;    /* baudrate 500Kbps */
		#elif (CAN_BAND_RATE	==	CAN_250KBPS)
				can_parameter.prescaler = 16;    /* baudrate 250Kbps */
		#elif (CAN_BAND_RATE	==	CAN_125KBPS)
				can_parameter.prescaler = 32;    /* baudrate 125Kbps */
		#else
				can_parameter.prescaler = 8;    /* baudrate 500Kbps */
		#endif
#endif

    can_init(CAN0, &can_parameter);
    /* initialize filter */
    /* CAN0 filter number */
    can_filter.filter_number = 0;
		
    /* initialize filter */    
    can_filter.filter_mode = CAN_FILTERMODE_MASK;
    can_filter.filter_bits = CAN_FILTERBITS_32BIT;
    can_filter.filter_list_high = 0x0000;
    can_filter.filter_list_low = 0x0000;
    can_filter.filter_mask_high = 0x0000;
    can_filter.filter_mask_low = 0x0000;  
    can_filter.filter_fifo_number = CAN_FIFO0;
    can_filter.filter_enable = ENABLE;
    can_filter_init(&can_filter);		
		
    /* configure CAN0 NVIC */
    nvic_irq_enable(USBD_LP_CAN0_RX0_IRQn,0,1);
    nvic_irq_enable(USBD_HP_CAN0_TX_IRQn,0,2);

    /* enable CAN receive FIFO0 not empty interrupt */
    can_interrupt_enable(CAN0, CAN_INTEN_RFNEIE0);
    /* enable CAN Tx empty interrupt */	
    can_interrupt_enable(CAN0, CAN_INTEN_TMEIE);
	
	/*******************************************************************************
	* 串口通讯
	*管脚68	PA9 UART_Tx;	管脚69	PA10 UART_Rx 
	*******************************************************************************/
#ifdef UART_ENABLE	
    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART0);

    /* connect port to USARTx_Tx */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

    /* connect port to USARTx_Rx */
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

    /* USART configure */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);
#endif	//#ifdef UART_ENABLE	
	/*******************************************************************************
	* 软件中断
	*管脚62	PD15, EXTI Line15 
	*******************************************************************************/
	gpio_init(GPIOD, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_15);

	/* enable and set key EXTI interrupt to the lowest priority */
	nvic_irq_enable(EXTI10_15_IRQn, 2U, 0U);
	
	/* connect key EXTI line to key GPIO pin */
	gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOD, GPIO_PIN_SOURCE_15);

	/* configure key EXTI line */
	exti_init(EXTI_15, EXTI_INTERRUPT, EXTI_TRIG_RISING);
	exti_interrupt_flag_clear(EXTI_15);
		
	/*******************************************************************************
	* 独立看门狗
	*******************************************************************************/
#ifdef WWDG_ENABLE	
    /* confiure FWDGT counter clock: 40KHz(IRC40K) / 64 = 0.625 KHz */
    fwdgt_config(2*500,FWDGT_PSC_DIV64);
    
    /* After 0.6 seconds to generate a reset */
    fwdgt_enable();
		
	//fwdgt_counter_reload();
#endif	

	/*******************************************************************************
	* Post Config
	*******************************************************************************/
	TIMER_CHCTL2(TIMER0) &= ~0x0505; 	//关闭U,W上下管PWM输出，电机开路检测	
	TIMER_CHCTL2(TIMER7) &= ~0x0005; 	//关闭P上下管PWM输出，电机开路检测	
	TIMER_CH0CV(TIMER7) = PROP_PWM_PERIOD;	//	
	
	timer_enable(TIMER0);
	timer_interrupt_enable(TIMER0, TIMER_INT_UP);		
}

//延时ulTime个指令周期,ulTime=10对应1us
void Delay(unsigned long ulTime)
{
	while(ulTime!=0)
	{
		ulTime=ulTime-1;
	}
}
void delay_us(INT32U nus)
{
	INT32U temp;
	temp = 30*nus; //120M/4
	while(temp!=0)
	{
		temp=temp-1;
	}
}

INT32U TIM_GetCounter(void)
{
	return TIMER_CNT(TIMER3);//TIM3->CNT;
}

/*******************************************************************************
* 函数名称：void PwmEnable(void)
* 函数功能：PWM
* 输入参数：
* 返回值：0
*******************************************************************************/
void PwmEnablePwOn(void)
{
#if ((CTLBOARD_TYPE ==_1236) || (CTLBOARD_TYPE ==_1246))
	GPIO_SetBits(GPIOF,GPIO_Pin_11);	/*PWM-OE 1-??,0-??*/
#endif  //#if (CTLBOARD_TYPE ==_1236)

#if (CTLBOARD_TYPE ==_1232)
	GPIO_SetBits(GPIOD,GPIO_Pin_0);	/*PWM-EN 0-??,1-??*/
#endif	//#if (CTLBOARD_TYPE ==_1232)

#if ((CTLBOARD_TYPE ==_1222) || (CTLBOARD_TYPE ==_1242))
	GPIO_SetBits(GPIOB,GPIO_Pin_14);	/*PWM-OE 1-有效,0-无效*/	
#endif

#if (CTLBOARD_TYPE ==_1226)
//  TIM_CtrlPWMOutputs(TIM1, ENABLE);;	/* Enable the TIM Main Output */
	TIM1->CCER |= 0x0555; 	//打开6路PWM输出
	HAL_TIMEx_PWMN_Start(&htim20,TIM_CHANNEL_1);//PWM_N接口使能
	HAL_TIM_PWM_Start(&htim20,TIM_CHANNEL_1);	//PWM接口使能
#endif

#if (CTLBOARD_TYPE ==_1226_GD32)
	TIMER_CHCTL2(TIMER0) |= 0x0555; 	//打开6路PWM输出
	TIMER_CHCTL2(TIMER7) |= 0x0005; 	//打开2路PWM输出	
#endif	
}
void PwmEnable(void)
{
}

/*******************************************************************************
* 函数名称：void PwmDisable(void)
* 函数功能：PWM
* 输入参数：
* 返回值：0
*******************************************************************************/
void PwmDisable(void)
{
#if ((CTLBOARD_TYPE ==_1236) || (CTLBOARD_TYPE ==_1246))
	TIM1->CCR1 = (PWM_PERIOD/2);	//UVW 50% 
	TIM1->CCR2 = (PWM_PERIOD/2);	//UVW 50% 
	TIM1->CCR3 = (PWM_PERIOD/2);	//UVW 50% 
#endif  //#if (CTLBOARD_TYPE ==_1236)

#if (CTLBOARD_TYPE ==_1232)
	TIM1->CCR1 = (PWM_PERIOD/2);	//UVW 50% 
	TIM1->CCR2 = (PWM_PERIOD/2);	//UVW 50% 
	TIM1->CCR3 = (PWM_PERIOD/2);	//UVW 50% 
#endif	//#if (CTLBOARD_TYPE ==_1232)

#if ((CTLBOARD_TYPE ==_1222) || (CTLBOARD_TYPE ==_1242))
	TIM1->CCR1 = (PWM_PERIOD/2);	//UVW 50% 
	TIM1->CCR2 = (PWM_PERIOD/2);	//UVW 50% 
	TIM1->CCR3 = (PWM_PERIOD/2);	//UVW 50% 
#endif

#if (CTLBOARD_TYPE ==_1226)
	TIM1->CCR1 = (PWM_PERIOD/2);	//UVW 50% 
	TIM1->CCR2 = (PWM_PERIOD/2);	//UVW 50% 
	TIM1->CCR3 = (PWM_PERIOD/2);	//UVW 50% 
#endif

#if (CTLBOARD_TYPE ==_1226_GD32)
	TIMER_CH0CV(TIMER0) = (PWM_PERIOD/2);	//UVW 50% 
	TIMER_CH1CV(TIMER0) = (PWM_PERIOD/2);	//UVW 50% 
	TIMER_CH2CV(TIMER0) = (PWM_PERIOD/2);	//UVW 50% 
#endif
//	TIM1->CCR1 = (INT16U)(PWM_PERIOD*100/100);	//UVW 97%,max 
//	TIM1->CCR2 = (INT16U)(PWM_PERIOD*100/100);	//UVW 97% 
//	TIM1->CCR3 = (INT16U)(PWM_PERIOD*100/100);	//UVW 97% 
//	TIM1->CCR1 = (INT16U)(PWM_PERIOD*3/100);	//UVW 3%,min
//	TIM1->CCR2 = (INT16U)(PWM_PERIOD*3/100);	//UVW 3% 
//	TIM1->CCR3 = (INT16U)(PWM_PERIOD*3/100);	//UVW 3% 
//	TIM5->CCR3 = 0;
}

void PwmDisableEsp(void)
{
//	TIM1->CCER &= ~0x0555; 	//关闭六路PWM输出
#if ((CTLBOARD_TYPE ==_1236) || (CTLBOARD_TYPE ==_1246))
	GPIO_ResetBits(GPIOF,GPIO_Pin_11);	/*PWM-OE 1-??,0-??*/	
	TIM1->CCR1 = (PWM_PERIOD/2);	//UVW 50% 
	TIM1->CCR2 = (PWM_PERIOD/2);	//UVW 50% 
	TIM1->CCR3 = (PWM_PERIOD/2);	//UVW 50% 
#endif  //#if (CTLBOARD_TYPE ==_1236)
#if (CTLBOARD_TYPE ==_1232)
	GPIO_ResetBits(GPIOD,GPIO_Pin_0);	/*PWM-EN 0-??,1-??*/
	TIM1->CCR1 = (PWM_PERIOD/2);	//UVW 50% 
	TIM1->CCR2 = (PWM_PERIOD/2);	//UVW 50% 
	TIM1->CCR3 = (PWM_PERIOD/2);	//UVW 50% 
#endif //#if (CTLBOARD_TYPE ==_1232)
#if ((CTLBOARD_TYPE ==_1222) || (CTLBOARD_TYPE ==_1242))
	GPIO_ResetBits(GPIOB,GPIO_Pin_14);	/*PWM-OE 1-有效,0-无效*/	
	TIM1->CCR1 = (PWM_PERIOD/2);	//UVW 50% 
	TIM1->CCR2 = (PWM_PERIOD/2);	//UVW 50% 
	TIM1->CCR3 = (PWM_PERIOD/2);	//UVW 50% 
#endif
#if (CTLBOARD_TYPE ==_1226)
	DRIVEREN_ON();	/*PWM-OE 1-有效,0-无效*/		
	TIM1->CCR1 = (PWM_PERIOD/2);	//UVW 50% 
	TIM1->CCR2 = (PWM_PERIOD/2);	//UVW 50% 
	TIM1->CCR3 = (PWM_PERIOD/2);	//UVW 50% 
	TIM20->CCR1 = PROP_PWM_PERIOD;
	TIM20->CCER &= ~0x0055; 	//关闭四路PWM输出
#endif
#if (CTLBOARD_TYPE ==_1226_GD32)
	DRIVEREN_ON();	/*PWM-OE 1-有效,0-无效*/	
	TIMER_CH0CV(TIMER0) = (PWM_PERIOD/2);	//UVW 50% 
	TIMER_CH1CV(TIMER0) = (PWM_PERIOD/2);	//UVW 50% 
	TIMER_CH2CV(TIMER0) = (PWM_PERIOD/2);	//UVW 50% 
	
	TIMER_CHCTL2(TIMER7) &= ~0x0005; 	//打开2路PWM输出	
	TIMER_CH0CV(TIMER7) = PROP_PWM_PERIOD;	//
#endif
}

/*******************************************************************************
*
* void F281XEv1PwmUpdate(struct PWM_GEN *p)
*
*******************************************************************************/
void PwmUpdate(PWM_GEN *p)
{       
	INT16U MPeriod;
	INT32S Tmp;
	/* Compute the timer period (Q0) from the period modulation input (Q15) */
	/* MfuncPeriod = 0x7FFF */
	/* Q15 = Q0*Q15 */
	Tmp = (INT32U)p->PeriodMax * (INT32U)p->MfuncPeriod;
	/* Q0 = (Q15->Q0)/2 + (Q0/2) */
	//MPeriod = (INT16S)(Tmp >> 16) + (INT16S)(p->PeriodMax >> 1);
	MPeriod = (INT32U)p->PeriodMax;// + (INT16S)(p->PeriodMax >> 1);
	//TIM_SetAutoreload(TIM1,MPeriod);
	
	/* Compute the compare 1 (Q0) from the PWM 1&2 duty cycle ratio (Q15) */
	/* Q15 = Q0*Q15 */
	Tmp = (INT32S)MPeriod * (INT32S)p->MfuncC1;
	/* Q0 = (Q15->Q0)/2 + (Q0/2) */
	TIMER_CH0CV(TIMER0) = (INT16S)(Tmp >> 15);// + (INT16S)((MPeriod + 1) >> 1);

	/* Compute the compare 2 (Q0) from the PWM 3&4 duty cycle ratio (Q15) */
	/* Q15 = Q0*Q15 */
	Tmp = (INT32S)MPeriod * (INT32S)p->MfuncC2;
	/* Q0 = (Q15->Q0)/2 + (Q0/2) */
	TIMER_CH1CV(TIMER0) = (INT16S)(Tmp >> 15);// + (INT16S)((MPeriod + 1) >> 1);

	/* Compute the compare 3 (Q0) from the PWM 5&6 duty cycle ratio (Q15) */
	/* Q15 = Q0*Q15 */
	Tmp = (INT32S)MPeriod * (INT32S)p->MfuncC3;
	/* Q0 = (Q15->Q0)/2 + (Q0/2) */
	TIMER_CH2CV(TIMER0) = (INT16S)(Tmp >> 15);// + (INT16S)((MPeriod + 1) >> 1);
}

/*******************************************************************************
* Name: DoPWM
* Description: 4 ms period. Run in ISR.
* Input: 
* Output: 
*
* Author:
* Date:
* Revision:
*******************************************************************************/
	/**************************************************************************
	*DO PWM: RELAY 200hz???? DRIVER1~2 200HZ
	*管脚55  PB3 TIM2_CH2	Relay;   
	*管脚58  PB6 TIM4_CH1	Driver3;  管脚59 TIM4_CH2 PB7	Driver2;
	**************************************************************************/
void PWMDriver(INT16U pulse[])
{
		/* DO Driver*/
#if (CTLBOARD_TYPE ==_1226)	
		TIM4->CCR1 = DO_PWM_TIM_PERIOD-pulse[0];	//Drv1
		TIM4->CCR2 = DO_PWM_TIM_PERIOD-pulse[1];	//Drv2
		TIM4->CCR3 = DO_PWM_TIM_PERIOD-pulse[2];	//Drv3
		TIM4->CCR4 = DO_PWM_TIM_PERIOD-pulse[3];	//Drv4
#endif	//#if (CTLBOARD_TYPE ==_1226)		
#if (CTLBOARD_TYPE ==_1226_GD32)
		TIMER_CH0CV(TIMER1) = pulse[0];	//Drv1
		TIMER_CH1CV(TIMER1) = pulse[1];	//Drv2
		TIMER_CH2CV(TIMER1) = pulse[2];	//Drv3
		TIMER_CH3CV(TIMER1) = pulse[3];	//Drv4
		//TIMER_CH2CV(TIMER4) = pulse[5];	//Drv5
		//TIMER_CH3CV(TIMER4) = pulse[6];	//Drv5
#endif	//#if (CTLBOARD_TYPE ==_1226_GD32)
}


void PropDriverPwmUpdate(PWM_GEN *p)
{       
	INT16U MPeriod;
	INT32S Tmp;

	/* Compute the timer period (Q0) from the period modulation input (Q15) */
	/* MfuncPeriod = 0x7FFF */
	/* Q15 = Q0*Q15 */
	Tmp = (INT32U)p->PeriodMax * (INT32U)p->MfuncPeriod;
	/* Q0 = (Q15->Q0)/2 + (Q0/2) */
	MPeriod = (INT16S)(Tmp >> 16) + (INT16S)(p->PeriodMax >> 1);
	//TIM_SetAutoreload(TIM3,MPeriod);
	
	/* Compute the compare 2 (Q0) from the PWM 3&4 duty cycle ratio (Q15) */
	/* Q15 = Q0*Q15 */
	Tmp = (INT32S)MPeriod * (INT32S)p->MfuncC2;
	/* Q0 = (Q15->Q0)/2 + (Q0/2) */
	Tmp = Tmp >> 15;
	if (Tmp <= p->PeriodMax)
		TIMER_CH0CV(TIMER7) = p->PeriodMax - (INT16S)(Tmp);
	else
		TIMER_CH0CV(TIMER7) = 0;
}


/*******************************************************************************
* Name: EepromWrite
* Description: EEPROM写入数据.
* Input: 
* Output: 0:Success;1:fail
* Author:
* Date:
* Revision:
*******************************************************************************/
INT16U EepromWrite(INT16U address, INT16U data)
{

#if (CTLBOARD_TYPE ==_1231_G4)
	uint8_t ulTxData[4],ulRxDataL[2];
	INT16U ret;
	ret=0;	
	//
	//1、使能写操作
	//
	//触发上升沿
	SPI_CS_0;  //CS reset
	Delay(20);  
	SPI_CS_1;  //CS set
	//发送写使能操作码	
//	ulTxData= 0x04C0;
	ulTxData[0] = 0x04;
	ulTxData[1] = 0xc0;
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&ulTxData,(uint8_t*)&ulRxDataL,2, SPI_TIMEOUT);

	//
	// 2、写操作
	//
	//触发上升沿
	SPI_CS_0;  //CS reset
	Delay(20); 
	SPI_CS_1;  //CS set
	//发送写操作码、地址
	ulTxData[0] = ((0x0500 | address)>>8)&0xFF;
	ulTxData[1] = (0x0500 | address)&0xFF;
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&ulTxData,(uint8_t*)&ulRxDataL,2, SPI_TIMEOUT);
	//发送16位数据
	ulTxData[0] = (data&0xff00)>>8;
	ulTxData[1] = data&0x00ff;
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&ulTxData,(uint8_t*)&ulRxDataL,2, SPI_TIMEOUT);
	//触发上升沿,等待写操作完成(不小于6ms)
	SPI_CS_0;  //CS reset
	Delay(20); 
	SPI_CS_1;  //CS set
	Delay(150000);
	//
	// 3. 写禁止
	//
	//触发上升沿
	SPI_CS_0;  //CS reset
	Delay(20);
	SPI_CS_1;  //CS set
	//发送写禁止操作码
	ulTxData[0] = 0x04;
	ulTxData[1] = 0x00;
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&ulTxData,(uint8_t*)&ulRxDataL,2, SPI_TIMEOUT);
	SPI_CS_0;  //CS reset
#endif	//#if (CTLBOARD_TYPE ==_1231_G4)

#if (CTLBOARD_TYPE ==_1226_GD32)
	INT16U ulTxData,ulRxData;
	INT16U ret;
	ret=0;	
	//
	//1、使能写操作
	//
	//触发上升沿
	SPI_CS_0;  //CS reset
	Delay(20);  
	SPI_CS_1;  //CS set
	//发送写使能操作码	
	ulTxData= 0x04C0;
	spi_i2s_data_transmit(SPI2, ulTxData);
	while(spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE) == RESET);
	ulRxData = spi_i2s_data_receive(SPI2);
	(void)ulRxData;	//防止编译警告
	//
	// 2、写操作
	//
	//触发上升沿
	SPI_CS_0;  //CS reset
	Delay(20); 
	SPI_CS_1;  //CS set
	//发送写操作码、地址
	ulTxData = 0x0500 | address;
	//等待发送操作完成
	while (spi_i2s_flag_get(SPI2, SPI_FLAG_TBE) == RESET);
	spi_i2s_data_transmit(SPI2, ulTxData);
	while (spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE) == RESET);
	ulRxData=spi_i2s_data_receive(SPI2);//接收读取数据	
	//发送16位数据
	ulTxData = data;
	//等待发送操作完成
	while (spi_i2s_flag_get(SPI2, SPI_FLAG_TBE) == RESET);
	spi_i2s_data_transmit(SPI2, ulTxData);
	while (spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE) == RESET);
	ulRxData=spi_i2s_data_receive(SPI2);//接收读取数据
	//触发上升沿,等待写操作完成(不小于6ms)	
	SPI_CS_0;  //CS reset
	Delay(20); 
	SPI_CS_1;  //CS set
	Delay(150000);
	//
	// 3. 写禁止
	//
	//触发上升沿
	SPI_CS_0;  //CS reset
	Delay(20);
	SPI_CS_1;  //CS set
	//发送写禁止操作码
	ulTxData = 0x0400;
	while (spi_i2s_flag_get(SPI2, SPI_FLAG_TBE) == RESET);
	spi_i2s_data_transmit(SPI2, ulTxData);
	while (spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE) == RESET);
	ulRxData=spi_i2s_data_receive(SPI2);//接收读取数据
	SPI_CS_0;  //CS reset
#endif	//#if (CTLBOARD_TYPE ==_1226_GD32)	
	return ret;
}

/*******************************************************************************
* Name: EepromRead
* Description: EEPROM读取数据
* Input: 
* Output: 0:Success;1:fail
* Author:
* Date:
* Revision:
*******************************************************************************/
INT16U EepromRead(INT16U address, INT16U* pdata)
{

#if (CTLBOARD_TYPE ==_1231_G4)
	uint8_t ulTxData[4],ulRxDataH[4],ulRxDataL[4];
	INT16U ret;
	ret=0;
	//触发上升沿
	SPI_CS_0;  //CS reset
	Delay(20);
	SPI_CS_1;  //CS set
	// 发送操作码和地址
//	ulTxData= (0x0600 | address)<<5;
	ulTxData[0] = (((0x0600 | address)<<5)>>8)&0xFF;
	ulTxData[1] = ((0x0600 | address)<<5)&0xFF;;
	ulTxData[2] = 0x00;
	ulTxData[3] = 0x00;
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&ulTxData,(uint8_t*)&ulRxDataH,2, SPI_TIMEOUT);

	// 发送无效数据，产生时钟信号，接收剩余读取数据 
	ulTxData[0] = 0x00;
	ulTxData[1] = 0x00;
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&ulTxData,(uint8_t*)&ulRxDataL,2, SPI_TIMEOUT);
	
	SPI_CS_0;  //CS reset

	*pdata = ((ulRxDataH[1]<<4)|(ulRxDataL[0]>>4))<<8;
	*pdata |= ((ulRxDataL[0]<<4)|(ulRxDataL[1]>>4))&0x00ff;
#endif	//#if (CTLBOARD_TYPE ==_1231_G4)

#if (CTLBOARD_TYPE ==_1226_GD32)
	INT16U ulTxData,ulRxDataL,ulRxDataH;
	INT16U ret;
	ret=0;

	//触发上升沿
	SPI_CS_0;  //CS reset
	Delay(20);
	SPI_CS_1;  //CS set
	// 发送操作码和地址
	ulTxData= (0x0600 | address)<<5;
	//等待发送操作完成
	while (spi_i2s_flag_get(SPI2, SPI_FLAG_TBE) == RESET);
	spi_i2s_data_transmit(SPI2, ulTxData);
	while (spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE) == RESET);
	ulRxDataH=spi_i2s_data_receive(SPI2);//接收读取数据	

	// 发送无效数据，产生时钟信号，接收剩余读取数据 
	ulTxData = 0x00;
	//等待发送操作完成
	while (spi_i2s_flag_get(SPI2, SPI_FLAG_TBE) == RESET);
	spi_i2s_data_transmit(SPI2, ulTxData);
	while (spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE) == RESET);
	ulRxDataL=spi_i2s_data_receive(SPI2);//接收读取数据	
	*pdata =(ulRxDataH<<12)|(ulRxDataL>>4);
	SPI_CS_0;  //CS reset
#endif	//#if (CTLBOARD_TYPE ==_1226_GD32)
	return ret;
}

/*******************************************************************************
* Name: EepromQualifiedRead
* Description: 从EEPROM中指定地址读出数据，含校验
* Input: eeprom read address.
* Output: 0: success. 1: fail. eeprom read data.
* 
* Author:
* Date:
* Revision:
*******************************************************************************/
INT16U EepromQualifiedRead(INT16U address, INT16U* pdata)
{
	INT16U ReadTimes, ReadFailTimes, ReadValue, data, ret;

	ret = 0;
	ReadTimes = 0;
	ReadFailTimes = 0;

	while(1)
	{
		/* read data in */
		ret = EepromRead(address, &data);
		if(ret == 1)
		{
			break;
		}
		ReadTimes++;
		if(ReadTimes == 1)
		{
			ReadValue = data;
		}
		else
		{
			/* if current read data != the first read data */
			if(data != ReadValue)
			{
				/* then try again */
				ReadTimes = 0;
				/* increase fail times */
				ReadFailTimes++;
				if(ReadFailTimes >= 3)
				{
					ret = 1;
					break;
				}
			}
			/* else current read data == the first read data */
			else
			{
				if(ReadTimes >= 3)
				{
					/* read ok */
					break;
				}
			}
		}
	}

	*pdata = data;
	if(ret == 1)
	{
		/* eeprom error */
		//SL_SET(PLC_EEPROM_RW_ERR);
	}

	return ret;	
}

/*******************************************************************************
* Name: EepromQualifiedWrite
* Description: 将数据写入EEPROM中指定地址,含校验
* Input: eeprom write address, write data
* Output: 0: success. 1: fail
* 
* Author:
* Date:
* Revision:
*******************************************************************************/
INT16U EepromQualifiedWrite(INT16U address, INT16U data)
{
	INT16U i16U, WriteFailTimes, ret;

	ret = 0;
	WriteFailTimes = 0;
	while(1)
	{
		ret = EepromWrite(address, data);
		if(ret == 1)
		{
			break;
		}
		ret = EepromRead(address, &i16U);
		if(ret == 1)
		{
			break;
		}
		/* if read == write */
		if(i16U == data)
		{
			/* then write ok */
			break;
		}
		/* else read != write */
		else
		{
			/* then write again until WriteFailTimes exceed 3 */
			WriteFailTimes++;
			if(WriteFailTimes >= 3)
			{
				ret = 1;
				break;
			}
		}
	}

	if(ret == 1)
	{
		/* eeprom error */
		//SL_SET(PLC_EEPROM_RW_ERR);
	}
	return ret;
}

