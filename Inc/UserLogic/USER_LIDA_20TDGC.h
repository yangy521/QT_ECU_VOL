/*******************************************************************************
* Filename: UserStackerTruckProc.h	                                           *
* Description: 逻辑层的头文件，包含对应的回调函数	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/07/22    														   *
* Revision:	 														 		   *
*******************************************************************************/
#ifndef _USER_STACKER_TRUCK_PROC_LIDA20TDGC_H_
#define _USER_STACKER_TRUCK_PROC_LIDA20TDGC_H_

#include "MstSlvCom.h"
#include "DoPwm.h"
#include "PropDriver.h"
#include "LocalDi.h"
#include "NetTimer.h"

/******************************************************************************
*函数定义
******************************************************************************/


#define	USER_ECU_PERIOD				5	

#define	LIFTUP_VALVE								DRIVER3
#define QIANHOUYI_VALVE							DRIVER4
#define QIANHOUQIN_VALVE						DRIVER5
#define ZUOYOUYI_VALVE							DRIVER6
#define KAIGUAN_VALVE								DRIVER7

#define	DIRECTION_VALVE		PropDriverCh0
#define	LIFTDOWN_VALVE		PropDriverCh1

#define	YouYi_SWI											SWI1_R
#define	EmergencyReverse							SWI2_R
#define	SAFELOCK_SWI									SWI3_R
#define ForWard_SWI										SWI4_R
#define BackWard_SWI									SWI5_R
#define LiftUpLimit_SWI								SWI6_R
#define	HouYi_SWI											SWI7_R
#define	QianYi_SWI										SWI8_R 
  
#define QianQin_SWI										DRIVER8_R  
#define HouQin_SWI										DRIVER9_R
#define ZuoYi_SWI											DRIVER10_R

#define MOVE_THROTTLE						AI_B_AI1_R
#define LIFT_THROTTLE						AI_B_AI2_R
#define LIFTSPEED_THROTTLE			AI_B_AI3_R

#define	BAT_LOW_WARING_VAL			20
#define	BAT_LOW_ERR_VAL				15

#define	ECU_POWERON_DELAY_TIME	500
#define	ECU_ANTI_PINCH_TIME 	3000
#define	DRIVER_CLOSE			0
#define	DRIVER_OPEN				1
#define	MOTOR_SPEED_RANGE		4096
#define PUMP_RANGE					256


#define	PROP_MIN_CURRENT0		PARA_PropDMinCurrent0
#define	PROP_MAX_CURRENT0		PARA_PropDMaxCurrent0

#define	PROP_MIN_CURRENT1		PARA_PropDMinCurrent1
#define	PROP_MAX_CURRENT1		PARA_PropDMaxCurrent1

#define	LEAN_FORWARD_PARA		PARA_PumpMotorGear1
#define	LEAN_BACKWARD_PARA		PARA_PumpMotorGear2

#define	LIFT_MODE_PARA			PARA_BrakeType

#define	MOVE_THROTTLE_MIN		PARA_ThrottleFDeadZoneMinVal
#define	MOVE_THROTTLE_MAX		PARA_ThrottleFDeadZoneMaxVal
#define	MOVE_THROTTLE_MID		PARA_ThrottleFMidVal

#define	LIFT_UP_THROTTLE_MIN	PARA_BrakeFDeadZoneMinVal
#define	LIFT_UP_THROTTLE_MAX	PARA_BrakeFDeadZoneMaxVal
#define	LIFT_UP_THROTTLE_MID	PARA_BrakeFMidVal

#define	LIFT_DOWN_THROTTLE_MIN	PARA_BrakeBDeadZoneMinVal
#define	LIFT_DOWN_THROTTLE_MAX	PARA_BrakeBDeadZoneMaxVal
#define	LIFT_DOWN_THROTTLE_MID	PARA_BrakeBMidVal

#define	TURN_START_ANGLE		PARA_TurnWithDecStartAngle
#define	TURN_END_ANGLE			PARA_TurnWithDecEndAngle
#define	START_ANGLE_SPD			PARA_AngleWithStartSpdPer
#define	END_ANGLE_SPD			PARA_AngleWithEndSpdPer

#define	RENTAL_INFO				PARA_RemotePara
#define	RENTAL_TIME				PARA_MaintenancePeriod

#define	USERINFO_MODE			PARA_ValueOpenPercentage	


#define	AI_B_AI1_ERR				ErrCode71
#define	AI_B_AI2_ERR				ErrCode72
#define	AI_B_AI3_ERR				ErrCode73
#define	AI_5V_12V_OUT1_ERR			ErrCode74
#define	AI_5V_12V_OUT2_ERR			ErrCode75

#define POWENON_ERR					ErrCode101
#define POWERLOW_ERR				ErrCode102
#define POWERLLOW_ERR				ErrCode103
#define EmsLogic_ERR				ErrCode104

#define ID444LOST_ERR				ErrCode105
#define ID360LOST_ERR				ErrCode106

#define EmsFord_ERR					ErrCode107
#define MOVESAFET_ERR				ErrCode108
#define LIFTSAFET_ERR				ErrCode109



#define	STEP_FACTOR					(5.0 * 4096 / 255)
#define	MOTOR_MAX_SPD				3000
#define	MOTOR_SWITCH_VALVE_SPD		300
#define	MOTOR_MIN_SPD_BY_CTRL		300

#define	LIFTUP_ERROCDE	
extern void vUserEcuInit(void);
extern void vUserEcuProc(void);


//试验台测试


#endif //#ifndef _USER_COMM_H_

 
