/*******************************************************************************
* Filename: UserStackerTruckProc.h	                                           *
* Description: 逻辑层的头文件，包含对应的回调函数	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/07/22    														   *
* Revision:	 														 		   *
*******************************************************************************/
#ifndef _USER_HANGUO_JINJIDAO_DBC_H_
#define _USER_HANGUO_JINJIDAO_DBC_H_

#include "MstSlvCom.h"
#include "DoPwm.h"
#include "PropDriver.h"
#include "LocalDi.h"
#include "NetTimer.h"

/******************************************************************************
*函数定义
******************************************************************************/

#define	USER_ECU_PERIOD				5

#define SWVIERSION			0x1105
#define HWVIERSION			0x0105
		
/*Move*/

#define BRAKE_SWI									SWI2_R
#define	SAFELOCK_SWI							SWI3_R
#define THROTTLE_SWI							SWI7_R

#define	MOVE_THROTTLE				AI_B_AI1_R
#define LIFT_THROTTLE				AI_B_AI2_R

#define	ECU_POWERON_DELAY_TIME	500
#define	ECU_ANTI_PINCH_TIME 	3000

#define	DRIVER_CLOSE			0
#define	DRIVER_OPEN				1

#define	MOTOR_MAX_SPEED			2000
#define	MOTOR_MAX_SPEED_VALUE	4095
#define	MOTOR_SPEED_RANGE		4096
#define	PUMP_MAX_VALUE			255
#define	PUMP_RANGE				256

#define	PROP_MIN_CURRENT		PARA_PropDMinCurrent0
#define	PROP_MAX_CURRENT		PARA_PropDMaxCurrent0

#define	PROP_MIN_CURRENT1		PARA_PropDMinCurrent1
#define	PROP_MAX_CURRENT1		PARA_PropDMaxCurrent1

#define	LEAN_FORWARD_PARA		PARA_PumpMotorGear1
#define	LEAN_BACKWARD_PARA		PARA_PumpMotorGear2

#define	MOVE_THROTTLE_MIN		PARA_ThrottleFDeadZoneMinVal
#define	MOVE_THROTTLE_MAX		PARA_ThrottleFDeadZoneMaxVal
#define	MOVE_THROTTLE_MID		PARA_ThrottleFMidVal

#define	LIFT_UP_THROTTLE_MIN	PARA_BrakeFDeadZoneMinVal
#define	LIFT_UP_THROTTLE_MAX	PARA_BrakeFDeadZoneMaxVal
#define	LIFT_UP_THROTTLE_MID	PARA_BrakeFMidVal

#define	LIFT_DOWN_THROTTLE_MIN	PARA_BrakeBDeadZoneMinVal
#define	LIFT_DOWN_THROTTLE_MAX	PARA_BrakeBDeadZoneMaxVal
#define	LIFT_DOWN_THROTTLE_MID	PARA_BrakeBMidVal

#define	BRAKE_THROTTLE_TYPE	PARA_BrakeType
#define	BRAKE_THROTTLE_MIN	PARA_BrakeFDeadZoneMinVal
#define	BRAKE_THROTTLE_MAX	PARA_BrakeFDeadZoneMaxVal
#define	BRAKE_THROTTLE_MID	PARA_BrakeFMidVal

#define	TURN_START_ANGLE		PARA_TurnWithDecStartAngle
#define	TURN_END_ANGLE			PARA_TurnWithDecEndAngle
#define	START_ANGLE_SPD			PARA_AngleWithStartSpdPer
#define	END_ANGLE_SPD				PARA_AngleWithEndSpdPer

#define	RENTAL_INFO				PARA_RemotePara
#define	RENTAL_TIME				PARA_MaintenancePeriod

#define	USERINFO_MODE			PARA_ValueOpenPercentage

#define	LIFT_MIDDLE_VALUE		5000

#define	BAT_LOW_WARING_VAL			20
#define	BAT_LOW_ERR_VAL				15	

#define	AI_B_AI1_ERR				ErrCode71
#define	AI_B_AI2_ERR				ErrCode72
#define	AI_B_AI3_ERR				ErrCode73
#define	AI_5V_12V_OUT1_ERR			ErrCode74
#define	AI_5V_12V_OUT2_ERR			ErrCode75

#define POWERON_CHECK_ERR			ErrCode100
#define	LOWPOWER_ERR					ErrCode101
#define LOWPOWER_WORING				ErrCode102
#define SWILOGIC_WORING				ErrCode103

#define	STEP_FACTOR					(5.0 * 4096 / 255)
#define	MOTOR_MAX_SPD				3000
#define	MOTOR_SWITCH_VALVE_SPD		300
#define	MOTOR_MIN_SPD_BY_CTRL		300

#define	LIFTUP_ERROCDE	
extern void vUserEcuInit(void);
extern void vUserEcuProc(void);

//试验台测试


#endif //#ifndef _USER_COMM_H_
