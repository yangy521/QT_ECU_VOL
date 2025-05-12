/*******************************************************************************
* Filename: UartComm.c 	                                    	     	       *
* Description: 	串口收发功能						           				   *
* Author:  QExpand, Lilu                                                       *
* Date: 2023/5/12															   *
* Revision:	V1.00														       *
*******************************************************************************/

#include "Device.h"
#include "UartComm.h"
#include "Log.h"


typedef struct 
{
	uint16_t u16Write;	    	/*当前写位置*/
	uint16_t u16Read;	    	/*当前读位置*/
//	uint16_t u16Cnt;
	uint8_t u8BufferData[UART_BUF_NUM]; 
}tUARTBuffer;

//UART0
static tUARTBuffer uart0RxBuffer;	/*UART0接收缓存*/
static tUARTBuffer uart0TxBuffer;	/*UART0发送缓存*/

//UART1
static tUARTBuffer uart1RxBuffer;	/*UART1接收缓存*/
static tUARTBuffer uart1TxBuffer;	/*UART1发送缓存*/


/*读UART数据缓冲区*/
static uint8_t u8UartReadBuffer (tUARTBuffer  *pBuffer, uint8_t *u8Byte)
{
	if (pBuffer->u16Read != pBuffer->u16Write)
	{
		*u8Byte = pBuffer->u8BufferData[pBuffer->u16Read];
		pBuffer->u16Read++;
		if (pBuffer->u16Read >= sizeof(pBuffer->u8BufferData)/sizeof(pBuffer->u8BufferData[0]))
		{
			pBuffer->u16Read = 0;
		}
//		pBuffer->u16Cnt--;
		return NOT_EMPTY;
	}
	else
	{
	  return EMPTY;
	}
}

/*写UART数据缓冲区*/
static uint8_t u8UartWriteBuffer (tUARTBuffer  *pBuffer, uint8_t u8Byte)
{
	int tempWrite;
	tempWrite = pBuffer->u16Write + 1;
	if (tempWrite >= sizeof(pBuffer->u8BufferData)/sizeof(pBuffer->u8BufferData[0]))
	{
		tempWrite = 0;
	}
	if (tempWrite != pBuffer->u16Read)
	{
		pBuffer->u8BufferData[pBuffer->u16Write] = u8Byte;		/*  向接收缓冲区写数据 */
		pBuffer->u16Write = tempWrite;
//		pBuffer->u16Cnt++;
//		if(pBuffer->u16Cnt >= sizeof(pBuffer->u8BufferData)/sizeof(pBuffer->u8BufferData[0]))
//		{
//			pBuffer->u16Cnt = sizeof(pBuffer->u8BufferData)/sizeof(pBuffer->u8BufferData[0]);
//		}
		return NOT_FULL;
	}
	else
	{
		return FULL;
	}	
}
/*获取串口缓存区剩余长度*/
static int16_t i16UartGetBufLen(tUARTBuffer  *pBuffer)
{
	int16_t len;
	//len = sizeof(pBuffer->u8BufferData)/sizeof(pBuffer->u8BufferData[0]) - pBuffer->u16Cnt;
	if ((len = pBuffer->u16Write - pBuffer->u16Read) < 0)
	{
		len += sizeof(pBuffer->u8BufferData)/sizeof(pBuffer->u8BufferData[0]);
	}
	return len;
}

/*缓存器写len个字节*/
static uint8_t u8UartWriteBufArray(tUARTBuffer *pBuffer, uint8_t *u8ByteArray, uint16_t len)
{
	uint16_t i;
	if(len < (sizeof(pBuffer->u8BufferData)/sizeof(pBuffer->u8BufferData[0]) - i16UartGetBufLen(pBuffer)))
	{
		for(i = 0; i < len; i++)
		{
			u8UartWriteBuffer(pBuffer, u8ByteArray[i]);
		}
		return C_NO_ERR;
	}
	else 
	{
		return C_NOACT;
	}
}

/*缓存器读取len个字节*/
static uint8_t u8UartReadBufArray(tUARTBuffer *pBuffer, uint8_t *u8ByteArray, uint16_t len)
{
	int16_t i;
	if(len <= i16UartGetBufLen(pBuffer))
	{
		for(i = 0; i < len; i++)
		{
			u8UartReadBuffer(pBuffer, u8ByteArray + i);
		}
		return C_NO_ERR;
	}
	else 
	{
		return C_OVERFLOW;
	}
}

/*空数据缓冲区*/
void UARTBufFlush(tUARTBuffer *pBuffer)
{
	pBuffer->u16Read = pBuffer->u16Write;
}

/*******************************************************************************
* Name: int i32UartWrite(eUart UartNo, uint8_t *u8pRevBuf, uint16_t u16RevLen)
* Descriptio: Write Datas To Uart
* Input: UartNo: range Uart0 or Uart1
*        u8pWriteBuf: Data buffer
*        u16WriteLen: Data buffer size
* Output:  >=0: actually write Data length
			-1: parameter error
*******************************************************************************/
int32_t i32UartWrite(eUart UartNo, uint8_t *u8WriteBuf, uint16_t u16WriteLen)
{
	int length = 0;
	if(Uart0 == UartNo)
	{
		if(USART_IDLE == u8GetUartSendState(Uart0))
		{
			vDrvUart0Send(u8WriteBuf, u16WriteLen);
		}
		else
		{
//			length = i16UartGetBufLen(&uart0TxBuffer);
//			if(length >= u16WriteLen)
//			{
//				length = u16WriteLen;		
//			}
			u8UartWriteBufArray(&uart0TxBuffer, u8WriteBuf, u16WriteLen);
		}
	}
	else if(Uart1 == UartNo)
	{
		if(USART_IDLE == u8GetUartSendState(Uart1))
		{
			vDrvUart1Send(u8WriteBuf, u16WriteLen);
		}
		else
		{
//			length = i16UartGetBufLen(&uart1TxBuffer);
//			if(length >= u16WriteLen)
//			{
//				length = u16WriteLen;		
//			}
			u8UartWriteBufArray(&uart1TxBuffer, u8WriteBuf, u16WriteLen);
			
		}
	}
	else
	{
		//i32LogWrite(ERR, "Write Uart Parameter is Error, UartNo == %d\r\n", UartNo);
		return -1;
	}
	return length;
}
/*******************************************************************************
* Name: int i32UartRead(eUart UartNo, uint8_t *u8pRevBuf, uint16_t u16RevLen)
* Descriptio: Read Datas From Uart
* Input: UartNo: range Uart0 or Uart1
*        u8pReadBuf: Data buffer
*        u16ReadLen: Data buffer size
* Output: -1: parameters err
		  others: actually Read Data length 
*******************************************************************************/
int32_t i32UartRead(eUart UartNo, uint8_t *u8ReadBuf, uint16_t u16ReadLen)
{
	int length = 0;
	if(NULL == u8ReadBuf)
	{
		//i32LogWrite(ERR, "Read Uart Parameter is Error, ReadBuf is NULL\r\n");
		return -1;
	}
	if(Uart0 == UartNo)
	{
		length = i16UartGetBufLen(&uart0RxBuffer);
		if(u16ReadLen <= length)
		{	
			length = u16ReadLen;
		}
		u8UartReadBufArray(&uart0RxBuffer, u8ReadBuf, length);
	}
	else if(Uart1 == UartNo)
	{
		length = i16UartGetBufLen(&uart1RxBuffer);
		if(u16ReadLen <= length)
		{	
			length = u16ReadLen;
		}
		u8UartReadBufArray(&uart1RxBuffer, u8ReadBuf, length);
	}
	else 
	{
		i32LogWrite(ERR, LOG_BSP, "Read Uart Parameter is Error, UartNo = %d\r\n", UartNo);
	}
	return length;
}
/*******************************************************************************
* Name: i32UartWriteRxBufFromInt(eUart UartNo, uint8_t *u8WriteBuf, uint16_t u16WriteLen)
* Descriptio: Write Datas From UartRdBuf From Int
* Input: UartNo: range Uart0 or Uart1
*        u8WriteBuf: Data buffer
*        u16WriteLen: Data buffer size
* Output: others: actually Read Data length 
*******************************************************************************/
static int32_t i32UartWriteRxBufFromInt(eUart UartNo, uint8_t *u8WriteBuf, uint16_t u16WriteLen)
{
	int length = 0;
	if(Uart0 == UartNo)
	{
//		length = i16UartGetBufLen(&uart0RxBuffer);
//		if(length > u16WriteLen)
//		{	
//			length = u16WriteLen;
//		}
//		u8UartWriteBufArray(&uart0RxBuffer, u8WriteBuf, length);
		u8UartWriteBufArray(&uart0RxBuffer, u8WriteBuf, u16WriteLen);
	}
	else if(Uart1 == UartNo)
	{
//		length = i16UartGetBufLen(&uart1RxBuffer);
//		if(length > u16WriteLen)
//		{	
//			length = u16WriteLen;
//		}
//		u8UartWriteBufArray(&uart1RxBuffer, u8WriteBuf, length);
		u8UartWriteBufArray(&uart1RxBuffer, u8WriteBuf, u16WriteLen);
	}
	else
	{
		
	}
	return length;
}

/*******************************************************************************
* Name: int i32UartRead(eUart UartNo, uint8_t *u8pRevBuf, uint16_t u16RevLen)
* Descriptio: Read Datas From UartTxBuf From Int
* Input: UartNo: range Uart0 or Uart1
*        u8pReadBuf: Data buffer
*        u16ReadLen: Data buffer size
* Output: -1: parameters err
		  others: actually Read Data length 
*******************************************************************************/
static int32_t i32UartReadTxBufFromInt(eUart UartNo, uint8_t *u8ReadBuf, uint16_t u16ReadLen)
{
	if(NULL == u8ReadBuf)
	{
		return -1;
	}
	int length = 0;
	if(Uart0 == UartNo)
	{
		length = i16UartGetBufLen(&uart0TxBuffer);
		if(length > u16ReadLen)
		{
			length = u16ReadLen;
		}
		u8UartReadBufArray(&uart0TxBuffer, u8ReadBuf, length);
	}
	else if(Uart1 == UartNo)
	{
		length = i16UartGetBufLen(&uart1TxBuffer);
		if(length > u16ReadLen)
		{
			length = u16ReadLen;
		}
		u8UartReadBufArray(&uart1TxBuffer, u8ReadBuf, length);
	}
	else
	{
		
	}
	return length;
}


/*DMA0_CH3 intterrput handle*/
void DMA0_Channel3_IRQHandler(void)
{
	if(RESET != dma_interrupt_flag_get(DMA0, DMA_CH3, DMA_INT_FLAG_FTF))
	{
		uint16_t length = 0;
		uint8_t SendBuf[UART0_SEND_BUF_LEN];
		dma_interrupt_flag_clear(DMA0, DMA_CH3, DMA_INT_FLAG_FTF);
		dma_channel_disable(DMA0, DMA_CH3);
		vSetUartSendState(Uart0, USART_IDLE);
		length = i32UartReadTxBufFromInt(Uart0, SendBuf, UART0_SEND_BUF_LEN);
		if(length > 0)
		{
			vDrvUart0Send(SendBuf, length);
		}
	}
}
/*DMA0_CH6 intterrput handle*/
void DMA0_Channel6_IRQHandler(void)
{
	if(RESET != dma_interrupt_flag_get(DMA0, DMA_CH6, DMA_INT_FLAG_FTF))
	{
		uint16_t length = 0;
		uint8_t SendBuf[UART1_SEND_BUF_LEN];
		dma_interrupt_flag_clear(DMA0, DMA_CH6, DMA_INT_FLAG_FTF);
		dma_channel_disable(DMA0, DMA_CH6);
		vSetUartSendState(Uart1, USART_IDLE);
		length = i32UartReadTxBufFromInt(Uart1, SendBuf, UART1_SEND_BUF_LEN);
		if(length > 0)
		{
			vDrvUart1Send(SendBuf, length);
		}
	}
}
/*Uart0 intterrput handle*/
void USART0_IRQHandler(void)
{
	if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_IDLE))
	{
		/* clear IDLE flag */
		usart_data_receive(USART0);
		/* number of data received */
		uint16_t u16RevLen = UART0_REV_BUF_LEN - (dma_transfer_number_get(DMA0, DMA_CH4));
		/*copy revbuf to revbuf*/
		i32UartWriteRxBufFromInt(Uart0, u8pGetRevBuf(Uart0), u16RevLen);
		/* disable DMA and reconfigure */
		dma_channel_disable(DMA0, DMA_CH4);
		dma_transfer_number_config(DMA0, DMA_CH4, UART0_REV_BUF_LEN);
		dma_channel_enable(DMA0, DMA_CH4);
	}					
}

/*Uart1 intterrput handle*/
void USART1_IRQHandler(void)
{
	if(RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_IDLE))
	{
		/* clear IDLE flag */
		usart_data_receive(USART1);
		/* number of data received */
		uint16_t u16RevLen = UART1_REV_BUF_LEN - (dma_transfer_number_get(DMA0, DMA_CH5));
		/*copy revbuf to revbuf*/
		i32UartWriteRxBufFromInt(Uart1, u8pGetRevBuf(Uart1), u16RevLen);
		/* disable DMA and reconfigure */
		dma_channel_disable(DMA0, DMA_CH5);
		dma_transfer_number_config(DMA0, DMA_CH5, UART1_REV_BUF_LEN);
		dma_channel_enable(DMA0, DMA_CH5);
	}					
}