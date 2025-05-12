/*******************************************************************************
* Filename: Log.c	                                             	 		   *
* Description: 	log接口处理										   			   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/05/15    														   *
* Revision:	 														 		   *
*******************************************************************************/

#include "stdarg.h"
#include "stdio.h"
#include "string.h"
#include "Log.h"
#include "Device.h"
#include "Para.h"

#ifdef LOG_ENABLE
/*定义log结构体*/
typedef struct
{
	uint8_t u8Message[LOG_MESSAGE_CNT][LOG_MESSAGE_LEN];
	uint8_t u8Read;
	uint8_t u8Write;
	uint8_t u8MesgCnt;
	uint16_t u16Model;
	uint16_t u16Level;
}xLogPara;


static xLogPara sgLog;		/*log global varaible*/

typedef struct
{
	uint8_t u8Bit;
	uint8_t u8FunName[8];
}xLogName;
	
/*******************************************************************************
* Name: void vLogInit(void)
* Descriptio: Log Initial
* Input: NULL
* Output: NULL 
*******************************************************************************/
void vLogInit(void)
{
	sgLog.u8MesgCnt = 0;
	sgLog.u8Read = 0;
	sgLog.u8Write = 0;
	sgLog.u16Level = i32GetPara(PARA_LogLevel);
	sgLog.u16Model = i32GetPara(PARA_LogModel);
}

/*******************************************************************************
* Name: int //i32LogWrite(uint8_t u8LogLevel, char *format,...)
* Descriptio: Write Log
* Input: u8LogLevel
* Output: actually write log length  
*******************************************************************************/
//int i32LogWrite(uint8_t u8LogLevel, char *format,...)
//{
//	if(u8LogLevel > LOG_LEVEL)
//	{
//		uint8_t tmp[LOG_MESSAGE_LEN] = {0};
//		int length = 0;
//		va_list vArgList;

//		va_start (vArgList, format);                 
//		length = vsnprintf((char*)tmp, LOG_MESSAGE_LEN, format, vArgList); 
//		va_end(vArgList);		
//		if(USART_IDLE == u8GetUartSendState(Uart2))
//		{
//			vDrvUart2Send(tmp, length);
//		}
//		else
//		{
//			if(length <= LOG_MESSAGE_LEN)
//			{
//				memset(sgLog.u8Message[sgLog.u8Write], 0x00, LOG_MESSAGE_LEN);
//				memcpy(sgLog.u8Message[sgLog.u8Write], tmp, length);
//				sgLog.u8Write++;
//				if(sgLog.u8Write >= LOG_MESSAGE_CNT)
//				{
//					sgLog.u8Write = 0;
//				}
//				sgLog.u8MesgCnt++;
//				if(sgLog.u8MesgCnt >= LOG_MESSAGE_CNT)
//				{
//					sgLog.u8MesgCnt = LOG_MESSAGE_CNT;
//				}
//				
//			}
//			else
//			{
//				return -1;
//			}
//		}

//		return length;
//	}
//	else
//	{
//		return -1;
//	}
//}

int i32LogWrite(uint8_t u8LogLevel, uint8_t u8LogModel, char *format,...)
{
	if ((u8LogLevel > sgLog.u16Level) && (0 != (sgLog.u16Model & (1 << u8LogModel))))
	{
		uint8_t tmp[LOG_MESSAGE_LEN] = {0};
		int length = 0;
		va_list vArgList;

		va_start (vArgList, format);                 
		length = vsnprintf((char*)tmp, LOG_MESSAGE_LEN, format, vArgList); 
		va_end(vArgList);		
		if(USART_IDLE == u8GetUartSendState(Uart2))
		{
			vDrvUart2Send(tmp, length);
		}
		else
		{
			if(length <= LOG_MESSAGE_LEN)
			{
				memset(sgLog.u8Message[sgLog.u8Write], 0x00, LOG_MESSAGE_LEN);
				memcpy(sgLog.u8Message[sgLog.u8Write], tmp, length);
				sgLog.u8Write++;
				if(sgLog.u8Write >= LOG_MESSAGE_CNT)
				{
					sgLog.u8Write = 0;
				}
				sgLog.u8MesgCnt++;
				if(sgLog.u8MesgCnt >= LOG_MESSAGE_CNT)
				{
					sgLog.u8MesgCnt = LOG_MESSAGE_CNT;
				}
				
			}
			else
			{
				return -1;
			}
		}

		return length;
	}
	else
	{
		return -1;
	}
}

/*******************************************************************************
* Name: int i32LogRead(uint8_t *des)
* Descriptio: Read From Log Buffer
* Input: Data Buffer
* Output: actually Log length  
*******************************************************************************/
static int i32LogReadFromInt(uint8_t *des)
{
	int length = 0;

	if(NULL == des)
	{
		return -1;
	}

	if(sgLog.u8Read != sgLog.u8Write)
	{
		length = strlen((char*)sgLog.u8Message[sgLog.u8Read]);
		memcpy(des, sgLog.u8Message[sgLog.u8Read], length);
		sgLog.u8Read++;
		sgLog.u8MesgCnt--;
		if(sgLog.u8Read >= LOG_MESSAGE_CNT)
		{
			sgLog.u8Read = 0;
		}

		//length = LOG_MESSAGE_LEN;
	}
	else
	{
		length = 0;
	}

	return length;
}
/*******************************************************************************
* Name: uint32_t u32GetLogCnt(void)
* Descriptio: Get Remain Log Cnt
* Input: NULL
* Output: Log Cnt  
*******************************************************************************/
uint32_t u32GetLogCnt(void)
{
	return sgLog.u8MesgCnt;
}

void DMA0_Channel1_IRQHandler(void)
{
	if(RESET != dma_interrupt_flag_get(DMA0, DMA_CH1, DMA_INT_FLAG_FTF))
	{
		uint16_t length = 0;
		uint8_t SendBuf[128];
		dma_interrupt_flag_clear(DMA0, DMA_CH1, DMA_INT_FLAG_FTF);
		dma_channel_disable(DMA0, DMA_CH1);
		vSetUartSendState(Uart2, USART_IDLE);
		length = i32LogReadFromInt(SendBuf);
		if(length > 0)
		{
			vDrvUart2Send(SendBuf, length);
		}
	}
}
#else 


void vLogInit(void)
{
	
}
int i32LogWrite(uint8_t u8LogLevel, uint8_t u8LogModel, char *format,...)
{
	return 0;
}
#endif