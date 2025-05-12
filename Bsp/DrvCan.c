/******************** (C) COPYRIGHT  XI`AN QEXPAND *******************************
* 文件名  ：DrvCan.c
* 描述    ：Can驱动文件       
* 实验平台：ECU
* 库版本  ：V1.0.0
* 作者    ：
* 修改时间: 2023-05-16
**********************************************************************************/	

#include "Device.h"
#include "string.h"
#include "gd32f30x.h"
#include "canSTM32F4.h"

uint8_t u8GetCanSendState(uint32_t can_periph)
{

	uint8_t res = 0;
	uint8_t mailbox_number = CAN_MAILBOX0;

	/* select one empty mailbox */
	if(CAN_TSTAT_TME0 == (CAN_TSTAT(can_periph) & CAN_TSTAT_TME0))
	{
		mailbox_number = CAN_MAILBOX0;
	}
	else if(CAN_TSTAT_TME1 == (CAN_TSTAT(can_periph) & CAN_TSTAT_TME1))
	{
		mailbox_number = CAN_MAILBOX1;
	}
	else if(CAN_TSTAT_TME2 == (CAN_TSTAT(can_periph) & CAN_TSTAT_TME2))
	{
		mailbox_number = CAN_MAILBOX2;
	}
	else
	{
		mailbox_number = CAN_NOMAILBOX;
	}

	if(CAN_NOMAILBOX == mailbox_number)
	{
		res = 0;
	}
	else
	{
		res = 1;
	}

	return res ;
}

//void vDrvCanInit(void)
void vDrvCanInit(uint16_t u16BaudRate)
{
	/*******************************************************************************
	* CAN

	管脚71 PA12 CAN0_TX	B-CAN-TX; 	管脚72	PA11 CAN0_RX	B-CAN-RX;  

	*******************************************************************************/

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

	switch (u16BaudRate)
	{
		case CAN_BAUD_RATE_125K:
			can_parameter.prescaler = 32;
			break;
		case CAN_BAUD_RATE_250K:
			can_parameter.prescaler = 16;
			break;
		case CAN_BAUD_RATE_500K:
			can_parameter.prescaler = 8;
			break;
		default:
			can_parameter.prescaler = 8;
			break;	
	}
//#ifdef CANBAUD_SET		//setting by switch
//	if(READ_SW2())
//	{	
//		delay_us(10000);	//delay for 10ms	
//		if(READ_SW2())
//		{
//			can_parameter.prescaler = 16;    /* baudrate 250Kbps */
//		}
//	}
//	else 
//	{
//		can_parameter.prescaler = 32;    /* baudrate 125Kbps */
//	}
//#else		//setting by user
//	#if (CAN_BAND_RATE	==	CAN_500KBPS)
//		can_parameter.prescaler = 8;    /* baudrate 500Kbps */
//	#elif (CAN_BAND_RATE	==	CAN_250KBPS)
//		can_parameter.prescaler = 16;    /* baudrate 250Kbps */
//	#elif (CAN_BAND_RATE	==	CAN_125KBPS)
//		can_parameter.prescaler = 32;    /* baudrate 125Kbps */
//	#else
//		can_parameter.prescaler = 8;    /* baudrate 500Kbps */
//	#endif
//#endif

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
}




