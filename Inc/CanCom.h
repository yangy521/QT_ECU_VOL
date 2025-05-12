/*******************************************************************************
* Filename: Can.h	                                             	 		   *
* Description:	Can收发功能										   			   *
* Author:  QExpand, Lilu                                                       *
* Date: 2023/5/12    														   *
* Revision:	V1.00														 	   *
*******************************************************************************/

#ifndef _CANCOM_H_
#define _CANCOM_H_

#include "stdint.h"

#define	CAN_FAILURE	0
#define	CAN_SUCCESS	1

#define	CAN_DATA_LENGH		8

#define CAN_MAX_Buffer		64

typedef enum{ 
	Can0 = 0,
	//Can1 = 1,
	CanMax,
}eCanNo;


typedef	 struct  
{
	uint32_t u32ID;							
	uint8_t	 u8Rtr;		
	uint16_t u16DataLength;					
	uint8_t  u8Data[CAN_DATA_LENGH];    	
}tCanFrame;

extern int32_t i32CanWrite(eCanNo, tCanFrame *pCanWriteFrame);
extern int32_t i32CanRead(eCanNo, tCanFrame *pCanReadFrame);


#endif
