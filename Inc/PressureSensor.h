/*******************************************************************************
* Filename: PressureSensor.h	                                               *
* Description:PressureSensor C Head File									   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/6/26    														   *
* Revision: V1.00															   *
*******************************************************************************/

#ifndef _PRESSURE_SENSOR_H_
#define _PRESSURE_SENSOR_H_

#include "stdint.h"


#define	PRESSURE_ADC_CHANNEL 		AI_B_AI2_R

typedef enum{ 
	PressureNoProblem = 0,
	PressureCaliReverse,
	PressureCaliFailure,
	PressureWithoutSensor,
	PressureOverPer80,
	PressureOverPer90,
	PressureOverPer99,
	PressureOverPer100,
	PressureMax,
}ePressureNo;

//#define	NO_OVER_LOAD				50
//#define	OVER_80_PERCENT_LOAD		80
//#define	OVER_90_PERCENT_LOAD		90
//#define	OVER_99_PERCENT_LOAD		99
//#define	OVER_100_PERCENT_LOAD		102

typedef void (*PressureCallBackt)(ePressureNo);

typedef struct
{
	uint16_t u16EmptyValue;
	uint16_t u16FullValue;
	uint16_t u16Per80Value;
	uint16_t u16Per90Value;
	uint16_t u16Per99Value;
	union
	{
		uint32_t u32Info;
		struct
		{
			uint32_t b1EmptyEn: 1;
			uint32_t b1FullEn: 1;
			uint32_t b1PressureEn: 1;
			uint32_t b1FourPointWeightFuncEn: 1;
			uint32_t b27Reserve: 27;
			uint32_t b1PressFlag: 1;
		};
	};
	PressureCallBackt CallBack;
}xPressure;


//extern void vPressureSensorSetValue(uint8_t u8ChannelNo, uint16_t u16Value);
extern void vPressureSensorReg(PressureCallBackt CallBack);
extern void vPressureSensorInit(void);
extern void vPressureSensorProc(void);



#endif

