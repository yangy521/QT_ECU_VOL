/*******************************************************************************
* Filename: Log.h	                                             	 		   *
* Description: Log Functions C Head File									   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/5/15    														   *
* Revision:	V1.00														 	   *
*******************************************************************************/

#ifndef _LOG_H_
#define _LOG_H_

#include "stdint.h"
	
#define		LOG_BSP			0
#define		LOG_LED			1
#define		LOG_PCU			2
#define		LOG_MST			3
#define		LOG_DI			4
#define		LOG_DO			5
#define		LOG_PWM			6
#define		LOG_PROP		7
#define		LOG_AI			8
#define		LOG_BAT			9
#define		LOG_CAN			10
#define		LOG_WDG			11
#define		LOG_ERR			12
#define		LOG_FUN			13
#define		LOG_MAIN		14
#define		LOG_USER		15

#define LOG_MESSAGE_CNT 64		/*64 frame*/
#define LOG_MESSAGE_LEN 128

/*Log level*/
typedef enum{ 
	ALL    = 0, 
	INFO   = 1, 
	DEBUG  = 2, 
	WARN   = 3, 
	ERR    = 4, 
	CLOSE  = 255 
}loglevel;


//#define LOG_LEVEL     ALL

extern void vLogInit(void);
extern int i32LogWrite(uint8_t u8LogLevel, uint8_t u8LogModel, char *format,...);
//extern int32_t //i32LogWrite(uint8_t u8LogLevel, char *format,...);

#endif
