/*******************************************************************************
* Filename: AutoTune.h 	                                    	     		   *
*                                                                              *
* Description: The header file of FDB.c.							           *
* Author:                                                                      *
* Date: 															           *
* Revision:															           *
*																	           *
*******************************************************************************/
#ifndef _AUTOTUNE_H_
#define _AUTOTUNE_H_

#include	"KSDsys.h"
#include "gd32f30x.h"

/******************************************************************************
*数据类型定义
******************************************************************************/
/* encoder */
typedef struct AUTOTUNE 
{
	INT16U  Enable;		//Tune enable 
	INT16U  WiperVolt;		//Wiper Voltage
	INT32U  WiperVoltSum;		//Wiper Voltage
	INT16U  Max;				//Max Voltage 
	INT16U  Min;				//Min Voltage 
	INT16U  Mid;				//Mid Voltage 
	INT16U  HomeSW;		//Home switch state
	INT16U  HomeSWOld;		//Home switch state	
	INT16S  Step;      //tuning steps
	INT16U  TuneCount;      //tuning counter
}AUTOTUNE;

#define TUNETIMEOUT   1000/5  //1000ms  

extern AUTOTUNE AutoTune;

extern void AutoTuneWiper( AUTOTUNE* this);

#endif 
