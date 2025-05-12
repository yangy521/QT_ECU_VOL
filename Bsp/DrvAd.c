/******************** (C) COPYRIGHT  XI`AN QEXPAND *******************************
* 文件名  ：DrvAd.c
* 描述    ：adc驱动文件       
* 实验平台：ECU
* 库版本  ：V1.0.0
* 作者    ：
* 修改时间: 2023-05-16
**********************************************************************************/	
#include "Device.h"

int32_t ADCValue[ADC0_NUM];
uint16_t inserted_data[4];

static void delay_us(INT32U nus)
{
	INT32U temp;
	temp = 30*nus; //120M/4
	while(temp!=0)
	{
		temp=temp-1;
	}
}

void vDrvAdInit(void)
{
/*******************************************************************************
* AD


管脚23 PA0	  ADC012_IN0	B-KSI-CHECK/AD; 		管脚24	PA1	 	ADC012_IN1			B VBUS-CHECK/AD; 
管脚25 PA2	  ADC012_IN2	B-AI1-R/AD; 			管脚26	PA3 	ADC012_IN3			B-AI2-R/AD;

管脚29 PA4	  ADC01_IN4		B-AI3-R/AD; 			管脚30 PA5	  	ADC01_IN5			5V/12V OUT2-I/AD; 	
管脚31 PA6 	  ADC01_IN6		5V/12V OUT2-R/AD; 		管脚32 PA7	  	ADC01_IN7			5V/12V OUT2-I/AD; 	
	
管脚33 PC4 	  ADC01_IN14	5V/12V OUT2-R/AD; 		管脚34 PC5 	    ADC01_IN15			3V3-CHECK/AD;
管脚35 PB0 	  ADC01_IN18	5V-CHECK/AD;			管脚36 PB1 	    ADC01_IN15			15V-CHECK/AD;	
	
管脚15 PC0	  ADC012_IN10	DRIVER-12/AD;  			管脚16 PC1		ADC012_IN11			DRIVER-11/AD; 
*******************************************************************************/
	/* enable ADC clock */
	rcu_periph_clock_enable(RCU_ADC0);
	rcu_periph_clock_enable(RCU_ADC1);
	//rcu_periph_clock_enable(RCU_ADC2);
	/* config ADC clock */
	rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV4);	//ADC clk @30M
	{		
//		gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
//		gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
//		gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
//		gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_3);
//		gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
//		gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_5);
//		gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
//		gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_7);
//		
//		gpio_init(GPIOB, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_0);	
//		gpio_init(GPIOB, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
//		
//		gpio_init(GPIOC, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
//		gpio_init(GPIOC, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
//		gpio_init(GPIOC, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
//		gpio_init(GPIOC, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_5);

		gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
		gpio_init(GPIOB, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_0 | GPIO_PIN_1);
		gpio_init(GPIOC, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5);
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
		dma_data_parameter.number       = ADC0_NUM;
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
		adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_NUM);
		/* ADC regular channel config */ 
		adc_regular_channel_config(ADC0, 0, ADC_CHANNEL_0, ADC_SAMPLETIME_71POINT5);
		adc_regular_channel_config(ADC0, 1, ADC_CHANNEL_1, ADC_SAMPLETIME_71POINT5);
		adc_regular_channel_config(ADC0, 2, ADC_CHANNEL_2, ADC_SAMPLETIME_71POINT5);
		adc_regular_channel_config(ADC0, 3, ADC_CHANNEL_3, ADC_SAMPLETIME_71POINT5);
		adc_regular_channel_config(ADC0, 4, ADC_CHANNEL_4, ADC_SAMPLETIME_71POINT5);
		adc_regular_channel_config(ADC0, 5, ADC_CHANNEL_5, ADC_SAMPLETIME_71POINT5);
		adc_regular_channel_config(ADC0, 6, ADC_CHANNEL_6, ADC_SAMPLETIME_71POINT5);
		adc_regular_channel_config(ADC0, 7, ADC_CHANNEL_7, ADC_SAMPLETIME_71POINT5);
		adc_regular_channel_config(ADC0, 8, ADC_CHANNEL_8, ADC_SAMPLETIME_71POINT5);
		adc_regular_channel_config(ADC0, 9, ADC_CHANNEL_9, ADC_SAMPLETIME_71POINT5);
		adc_regular_channel_config(ADC0, 10, ADC_CHANNEL_14, ADC_SAMPLETIME_71POINT5);
		adc_regular_channel_config(ADC0, 11, ADC_CHANNEL_15, ADC_SAMPLETIME_71POINT5);		 
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
			adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_11, ADC_SAMPLETIME_13POINT5);
		
			/* ADC trigger config */
			adc_external_trigger_source_config(ADC0, ADC_INSERTED_CHANNEL, ADC0_1_EXTTRIG_INSERTED_T3_TRGO); 
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
	adc_inserted_channel_config(ADC1, 0, ADC_CHANNEL_10, ADC_SAMPLETIME_13POINT5);

	/* ADC trigger config */
	adc_external_trigger_source_config(ADC1, ADC_INSERTED_CHANNEL, ADC0_1_EXTTRIG_INSERTED_T3_TRGO); //需要修改
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

#if 0
	/************************************************/	
	//ADC2 INJECTED
	/* ADC channel length config */
	adc_channel_length_config(ADC2, ADC_INSERTED_CHANNEL, 2);
	/* ADC inserted channel config */
	adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_12, ADC_SAMPLETIME_13POINT5);
	adc_inserted_channel_config(ADC2, 0, ADC_CHANNEL_13, ADC_SAMPLETIME_13POINT5);

	/* ADC trigger config */
	adc_external_trigger_source_config(ADC2, ADC_INSERTED_CHANNEL, ADC0_1_EXTTRIG_INSERTED_T0_TRGO); //需要修改
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
	nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
	nvic_irq_enable(ADC2_IRQn, 0, 0);
#endif
}

