/*******************************************************************************
* Filename: DoPwm.h	                                             	 	       *
* Description:	DoPwm功能										   			   *
* Author:  QExpand, Lilu                                                       *
* Date: 2023/5/12    														   *
* Revision:	V1.00														 	   *
*******************************************************************************/

#ifndef _DOPWM_H_
#define _DOPWM_H_

#include "stdint.h"

#define	DOPWM_PERIOD			5
#define FULL_VOLTAGE_ACT_TIME 	(400 / DOPWM_PERIOD)   //400MS
#define	DO_PWM_TIM_PERIOD		1000	

#define	DO_PWM_TIM_CLK			40000
#define	DO_PWM_CMP_LIMIT_MIN	(1000 * 5 / 100)
#define	DO_PWM_CMP_LIMIT_MAX	(1000 * 95 / 100)
#define DoFdbVoltageSHIFT       7

#define	DO_LOST_CHK_DELAY_TIME	500

typedef enum{ 
	DRIVER1 = 0,
	DRIVER2 = 1,
	DRIVER3 = 2,
	DRIVER4 = 3,
	DRIVER5 = 4,
	DRIVER6 = 5,
	DRIVER7 = 6,
	DRIVER8 = 7,
	DRIVER9 = 8,
	DRIVER10 = 9,
	DoPwmMax,
}eDoPwmNo;


typedef struct
{
	uint8_t u8DoPwmFlag;		/*Pwm的百分比*/
	uint16_t u16PluseWidth;			/*脉冲宽度*/
	uint16_t u16PluseWidthDelay;	/*脉冲宽度迟延*/
	uint16_t u16DoFdbVoltage;         //0~(1<<DoFdbVoltageSHIFT)
	uint16_t u16DoFdbVoltageSum;         //
}xDoPwmPara;

typedef void (*DriverErrCallBackt)(eDoPwmNo DoPwmNo);

extern void vDoPwmInit(void);
extern int32_t i32DoPwmSet(eDoPwmNo DoPwmNo, uint8_t u8Flag);
extern void vDoPwmErrReg(DriverErrCallBackt Callback);
extern void vDoPwmProc(void);

extern void vDoPwmErrReg(DriverErrCallBackt);

#ifdef DoFdbVoltageCal_EN
extern void vDoFdbVoltageCal(void);
extern int16_t i16GetDoFdbVoltage(eDoPwmNo DoPwmNo);
#endif


#endif

