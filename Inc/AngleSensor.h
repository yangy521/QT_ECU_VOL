/*******************************************************************************
* Filename: AngleSensor.h	                                             	   *
* Description:AngleSensor C Head File										   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/5/15    														   *
* Revision: V1.00															   *
*******************************************************************************/

#ifndef _ANGLE_SENSOR_H_
#define _ANGLE_SENSOR_H_

#include "stdint.h"


#define	ANGLE_ADC_CHANNEL 		AI_B_AI1_R

#define	ANGLE_NUM		8

typedef void (*AngleCallBackt)(uint8_t);

typedef struct
{
	uint16_t u16MinValue;
	uint16_t u16MaxValue;
	uint16_t u16Value[ANGLE_NUM];
	AngleCallBackt CallBack;
}xAngle;



extern void vAngleSensorReg(AngleCallBackt CallBack);
extern void vAngleSensorInit(void);
extern void vAngleSensorProc(void);



#endif

