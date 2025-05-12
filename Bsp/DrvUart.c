/*******************************************************************************
* Filename: DrvUart.c 	                                    	     	       *
* Description: 	´®¿Úµ×²ãÇý¶¯ÅäÖÃ						           				   *
* Author:  QExpand, Lilu                                                       *
* Date: 2023/5/12															   *
* Revision:	V1.00														       *
*******************************************************************************/

#include "Device.h"
#include "string.h"

static uint8_t u8Uart0SendBuf[UART0_SEND_BUF_LEN];  /*Uart0 Send Buf*/
static uint8_t u8Uart0RevBuf[UART0_REV_BUF_LEN];	/*Uart0 Rev Buf*/

static uint8_t u8Uart1SendBuf[UART1_SEND_BUF_LEN];	/*Uart1 Send Buf*/
static uint8_t u8Uart1RevBuf[UART1_REV_BUF_LEN];	/*Uart1 Rev Buf*/

static uint8_t u8Uart2SendBuf[UART2_SEND_BUF_LEN];	/*Uart2 Send Buf*/

static uint8_t u8UartSendState[3] = {USART_IDLE, USART_IDLE, USART_IDLE};	/*Uart SendState*/

static void vUart0Init(uint32_t u32BaudRate)
{
	/* Uart1_DMA_channel configuration */
	dma_parameter_struct dma_data_parameter;

	rcu_periph_clock_enable(RCU_USART0);
	
	gpio_pin_remap_config(GPIO_USART0_REMAP, ENABLE);

	/* connect port to USARTx_Tx */
	gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
	/* connect port to USARTx_Rx */
	gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_7);
	
	gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_5);

	/* USART configure */
	usart_deinit(USART0);
	usart_baudrate_set(USART0, u32BaudRate);
	usart_receive_config(USART0, USART_RECEIVE_ENABLE);
	usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
	usart_flag_clear(USART0, USART_FLAG_RBNE);
	usart_interrupt_enable(USART0, USART_INT_IDLE);
	/* configure Uart0 NVIC */
	nvic_irq_enable(USART0_IRQn, 0, 1);

	/* Uart1 Rx DMA_channel configuration */
	dma_struct_para_init(&dma_data_parameter);
	dma_data_parameter.direction = DMA_PERIPHERAL_TO_MEMORY;
	dma_data_parameter.memory_addr = (uint32_t)u8Uart0RevBuf;
	dma_data_parameter.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
	dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_8BIT;
	dma_data_parameter.number = sizeof(u8Uart0RevBuf);
	dma_data_parameter.periph_addr = (uint32_t)(&USART_DATA(USART0));
	dma_data_parameter.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
	dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
	dma_data_parameter.priority = DMA_PRIORITY_ULTRA_HIGH;
	dma_init(DMA0, DMA_CH4, &dma_data_parameter);
	usart_dma_receive_config(USART0, USART_DENR_ENABLE);
	dma_channel_enable(DMA0, DMA_CH4);

	/* Uart0 Tx DMA_channel configuration */
	dma_struct_para_init(&dma_data_parameter);
	dma_data_parameter.direction = DMA_MEMORY_TO_PERIPHERAL;
	dma_data_parameter.memory_addr = (uint32_t)u8Uart0SendBuf;
	dma_data_parameter.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
	dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_8BIT;
	dma_data_parameter.number = sizeof(u8Uart0SendBuf);
	dma_data_parameter.periph_addr = (uint32_t)(&USART_DATA(USART0));
	dma_data_parameter.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
	dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
	dma_data_parameter.priority = DMA_PRIORITY_ULTRA_HIGH;
	dma_init(DMA0, DMA_CH3, &dma_data_parameter);

	/* enable DMA channel */
	usart_dma_transmit_config(USART0, USART_DENT_ENABLE);
	dma_interrupt_enable(DMA0, DMA_CH3, DMA_INT_FTF);
	dma_channel_disable(DMA0, DMA_CH3);
	/* configure DMA0_CH3 NVIC */
	nvic_irq_enable(DMA0_Channel3_IRQn, 0, 2);

	usart_enable(USART0);
}

static void vUart1Init(uint32_t u32BaudRate)
{
	/* Uart1_DMA_channel configuration */
	dma_parameter_struct dma_data_parameter;

	rcu_periph_clock_enable(RCU_USART1);
	
	gpio_pin_remap_config(GPIO_USART1_REMAP, ENABLE);

	/* connect port to USARTx_Tx */
	gpio_init(GPIOD, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5);
	/* connect port to USARTx_Rx */
	gpio_init(GPIOD, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_6);

	/* USART configure */
	usart_deinit(USART1);
	usart_baudrate_set(USART1, u32BaudRate);
	usart_receive_config(USART1, USART_RECEIVE_ENABLE);
	usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
	usart_flag_clear(USART1, USART_FLAG_RBNE);
	usart_interrupt_enable(USART1, USART_INT_IDLE);
	/* configure Uart0 NVIC */
	nvic_irq_enable(USART1_IRQn, 0, 1);

	/* Uart1 Rx DMA_channel configuration */
	dma_struct_para_init(&dma_data_parameter);
	dma_data_parameter.direction = DMA_PERIPHERAL_TO_MEMORY;
	dma_data_parameter.memory_addr = (uint32_t)u8Uart1RevBuf;
	dma_data_parameter.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
	dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_8BIT;
	dma_data_parameter.number = sizeof(u8Uart1RevBuf);
	dma_data_parameter.periph_addr = (uint32_t)(&USART_DATA(USART1));
	dma_data_parameter.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
	dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
	dma_data_parameter.priority = DMA_PRIORITY_ULTRA_HIGH;
	dma_init(DMA0, DMA_CH5, &dma_data_parameter);
	usart_dma_receive_config(USART1, USART_DENR_ENABLE);
	dma_channel_enable(DMA0, DMA_CH5);

	/* Uart1 Tx DMA_channel configuration */
	dma_struct_para_init(&dma_data_parameter);
	dma_data_parameter.direction = DMA_MEMORY_TO_PERIPHERAL;
	dma_data_parameter.memory_addr = (uint32_t)u8Uart1SendBuf;
	dma_data_parameter.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
	dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_8BIT;
	dma_data_parameter.number = sizeof(u8Uart1SendBuf);
	dma_data_parameter.periph_addr = (uint32_t)(&USART_DATA(USART1));
	dma_data_parameter.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
	dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
	dma_data_parameter.priority = DMA_PRIORITY_ULTRA_HIGH;
	dma_init(DMA0, DMA_CH6, &dma_data_parameter);

	/* enable DMA channel */
	usart_dma_transmit_config(USART1, USART_DENT_ENABLE);
	dma_interrupt_enable(DMA0, DMA_CH6, DMA_INT_FTF);
	dma_channel_disable(DMA0, DMA_CH6);

	/* configure DMA0_CH6 NVIC */
	nvic_irq_enable(DMA0_Channel6_IRQn, 0, 2);
	usart_enable(USART1);
}

static void vUart2Init(uint32_t u32BaudRate)
{
	/* Uart2_DMA_channel configuration */
	dma_parameter_struct dma_data_parameter;

	rcu_periph_clock_enable(RCU_USART2);
	
	gpio_pin_remap_config(GPIO_USART2_FULL_REMAP, ENABLE);

	/* connect port to USARTx_Tx */
	gpio_init(GPIOD, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
	/* connect port to USARTx_Rx */
	gpio_init(GPIOD, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

	/* USART configure */
	usart_deinit(USART2);
	usart_baudrate_set(USART2, u32BaudRate);
	usart_receive_config(USART2, USART_RECEIVE_ENABLE);
	usart_transmit_config(USART2, USART_TRANSMIT_ENABLE);

	/* Uart2 Tx DMA_channel configuration */
	dma_deinit(DMA0, DMA_CH1);
	dma_struct_para_init(&dma_data_parameter);
	dma_data_parameter.direction = DMA_MEMORY_TO_PERIPHERAL;
	dma_data_parameter.memory_addr = (uint32_t)u8Uart2SendBuf;
	dma_data_parameter.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
	dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_8BIT;
	dma_data_parameter.number = sizeof(u8Uart2SendBuf);
	dma_data_parameter.periph_addr = (uint32_t)(&USART_DATA(USART2));
	dma_data_parameter.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
	dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
	dma_data_parameter.priority = DMA_PRIORITY_ULTRA_HIGH;
	dma_init(DMA0, DMA_CH1, &dma_data_parameter);

	/* enable DMA channel */
	dma_channel_disable(DMA0, DMA_CH1);
	usart_dma_transmit_config(USART2, USART_DENT_ENABLE);
	dma_interrupt_enable(DMA0, DMA_CH1, DMA_INT_FTF);

	/* configure DMA0_CH1 NVIC */
	nvic_irq_enable(DMA0_Channel1_IRQn, 0, 2);

	usart_enable(USART2);    
}

/*******************************************************************************
* Name: void vDrvUartInit(void)
* Descriptio: Uart Bsp Initial
* Input: NULL
* Output: NULL 
*******************************************************************************/
void vDrvUartInit(void)
{
	vUart0Init(9600);     	/*ECU  <---> PCU*/
	vUart1Init(115200);     /*Master  <---> Slave*/
//#ifdef LOG_ENABLE
	vUart2Init(115200);     /*ECU Log*/
//#endif
}

/*******************************************************************************
* Name: vSetUartSendState(eUart UartNo, uint8_t u8State)
* Descriptio: Set Uart Send state
* Input: UartNo: range in (Uart0, Uart1, Uart2)
*        u8State: range in (UART_IDLE or UART_BUSY)
* Output: NULL 
*******************************************************************************/
void vSetUartSendState(eUart UartNo, uint8_t u8State)
{
	u8UartSendState[UartNo] = u8State;
}

/*******************************************************************************
* Name: vSetUartSendState(eUart UartNo, uint8_t u8State)
* Descriptio: Get Uart Send State
* Input: UartNo: range in (Uart0, Uart1, Uart2)
* Output: UartSate 
*******************************************************************************/
uint8_t u8GetUartSendState(eUart UartNo)
{
	uint8_t res;
	__set_PRIMASK(1);
	res = u8UartSendState[UartNo];
	__set_PRIMASK(0);
	return res;
}

/*******************************************************************************
* Name: void vDrvUart0Send(uint8_t *u8pSendBuf, uint16_t u16SendLen)
* Descriptio: Uart0 Send Datas
* Input: u8SendBuf: Data buffer
*        u16SendLen: Data buffer size
* Output: NULL 
*******************************************************************************/
void vDrvUart0Send(uint8_t *u8SendBuf, uint16_t u16SendLen)
{
	memcpy(u8Uart0SendBuf, u8SendBuf, u16SendLen);
	dma_transfer_number_config(DMA0, DMA_CH3, u16SendLen);
	vSetUartSendState(Uart0, USART_BUSY);
	dma_channel_enable(DMA0, DMA_CH3);
}
/*******************************************************************************
* Name: void vDrvUart1Send(uint8_t *u8pSendBuf, uint16_t u16SendLen)
* Descriptio: Uart1 Send Datas
* Input: u8SendBuf: Data buffer
*        u16SendLen: Data buffer size
* Output: NULL 
*******************************************************************************/
void vDrvUart1Send(uint8_t *u8SendBuf, uint16_t u16SendLen)
{
	memcpy(u8Uart1SendBuf, u8SendBuf, u16SendLen);
	dma_transfer_number_config(DMA0, DMA_CH6, u16SendLen);
	vSetUartSendState(Uart1, USART_BUSY);
	dma_channel_enable(DMA0, DMA_CH6);
}

/*******************************************************************************
* Name: void vDrvUart2Send(uint8_t *u8pSendBuf, uint16_t u16SendLen)
* Descriptio: Uart2 Send Datas
* Input: u8SendBuf: Data buffer
*        u16SendLen: Data buffer size
* Output: NULL 
*******************************************************************************/
void vDrvUart2Send(uint8_t *u8SendBuf, uint16_t u16SendLen)
{
	memcpy(u8Uart2SendBuf, u8SendBuf, u16SendLen);
	dma_transfer_number_config(DMA0, DMA_CH1, u16SendLen);
	vSetUartSendState(Uart2, USART_BUSY);
	dma_channel_enable(DMA0, DMA_CH1);
}
/*******************************************************************************
* Name: uint8_t * u8pGetRevBuf(eUart UartNo)
* Descriptio: Get DMA's Uart Mempry address
* Input: UartNo: range Uart0 or Uart1
* Output: DMA's Uart Mempry address 
*******************************************************************************/
uint8_t* u8pGetRevBuf(eUart UartNo)
{
	if(Uart0 == UartNo)
	{
		return u8Uart0RevBuf;
	}
	else 
	{
		return u8Uart1RevBuf;
	}
}




