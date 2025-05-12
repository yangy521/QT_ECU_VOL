/*******************************************************************************
* Filename: HourCount.h 	                                    	     	   *
*                                                                              *
* Description: The header file of HourCount.c.							       *
* Author:                                                                      *
* Date: 															           *
* Revision:															           *
*																	           *
*******************************************************************************/
#ifndef __HOURCOUNT_H
#define __HOURCOUNT_H

#include "stdint.h"
#include "KSDsys.h"
/******************************************************************************
*数据类型定义
******************************************************************************/
#define HOUR_TYPE_NORMAL      1
#define HOUR_TYPE_POWENON     2
#define HOURCOUNT_PERIOD	  1000

//extern INT32U HourCount1s;

/******************************************************************************
*函数定义
******************************************************************************/
uint32_t u32HourCountRead(void);
void vHourCountWrite(uint32_t u32HourCount);

#endif //__HOURCOUNT_H
