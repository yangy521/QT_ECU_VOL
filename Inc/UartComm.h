/*******************************************************************************
* Filename: uartComm.h 	                                    	     		   *
* Description: 	串口收发功能						           				   *
* Author:  QExpand, Lilu                                                       *
* Date: 2023/5/12															   *
* Revision:	V1.00														       *													 
*******************************************************************************/
#ifndef _UART_COMM_H_
#define _UART_COMM_H_

#include "Device.h"

#define UART_BUF_NUM					64
#define UART_DMARX_SIZE					64
#define UART_DMATX_SIZE					64

//定义UART数据收发缓冲区标识符
#define FULL                    2
#define EMPTY                   3
#define NOT_FULL                4
#define NOT_EMPTY               5

#define C_NO_ERR      1
#define C_NOACT       2
#define C_OVERFLOW    3


extern int32_t i32UartWrite(eUart UartNo, uint8_t *u8pWriteBuf, uint16_t u16WriteLen);
extern int32_t i32UartRead(eUart UartNo, uint8_t *u8pWriteBuf, uint16_t u16WriteLen);


#endif


