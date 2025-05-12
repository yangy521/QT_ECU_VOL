#ifndef _CAN_BAUD_RATE_SYNC_H_
#define _CAN_BAUD_RATE_SYNC_H_

#include "stdio.h"
#include "stdint.h"
#include "string.h"

#define SINGLE_MCU                  (0)
#define MULTIPLE_MCU_MST            (1)
#define MULTIPLE_MCU_SLV            (2)  

#define	MCU_TYPE					MULTIPLE_MCU_MST

#define QUERY_HEAD					(0xA5)
#define QUERY_LENGTH				(0x05)
#define RESPOND_HEAD				(0xAA)
#define RESPOND_LENGTH				(0x05)

#define	STATE_HEAD					(0x00)
#define	STATE_LENGTH				(0x01)
#define	STATE_DATA					(0x02)
#define	STATE_CHECKSUM				(0x03)

/*lilu 20240221 不同的平台不同的写法，根据平台填写对应的内容*/
#if ((MULTIPLE_MCU_MST == MCU_TYPE) || (SINGLE_MCU == MCU_TYPE))
void vEepromSpiInit(void);
uint16_t u16EepromReadCanBaudRate(uint16_t Addr, uint16_t *u16Data);
#endif

/*lilu 20210221 不同的平台不同的写法，根据平台填写对应的内容, 作为SLV时，发送接收采用阻塞时，作为MST时，发送接收采用非阻塞时（中断优先考虑）*/
#if (SINGLE_MCU != MCU_TYPE)
typedef void (*SendCallBackt)(uint8_t*, uint16_t u16Length);
void vCanBaudRateSyncUartInit(void);
void SendCallBack(uint8_t *u8Data, uint16_t u16Len);
#endif

#if (MULTIPLE_MCU_MST == MCU_TYPE)
extern void vRespondCanBaudRate(uint8_t u8Data, SendCallBackt SendCallBack);
extern uint8_t u8GetSlvQuery(uint8_t u8Data);
#endif

#if (MULTIPLE_MCU_SLV == MCU_TYPE)
extern void vQueryCanBaudRate(SendCallBackt SendCallBack);
extern uint8_t u8GetMstRespond(uint8_t u8Data, uint8_t *u8CanBaud);
#endif

#define EEPROM_CAN_BUAD_ADDR	(106)			/*lilu 20240221 can baudRate eeprom addr*/

#define	CAN_BAUD_RATE_125K		(125)
#define	CAN_BAUD_RATE_250K		(250)
#define	CAN_BAUD_RATE_500K		(500)
#define CAN_BAUD_RRAT_800K		(800)
#define CAN_BAUD_RATE_1M		(1000)

#endif
