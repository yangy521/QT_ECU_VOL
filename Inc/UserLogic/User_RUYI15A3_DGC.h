/*******************************************************************************
* Filename: UserStackerTruckProc.h	                                           *
* Description: 逻辑层的头文件，包含对应的回调函数	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/07/22    														   *
* Revision:	 														 		   *
*******************************************************************************/
#ifndef _USER_STACKER_TRUCK_RUYI15A3_H_
#define _USER_STACKER_TRUCK_RUYI15A3_H_

#include "MstSlvCom.h"
#include "DoPwm.h"
#include "PropDriver.h"
#include "LocalDi.h"
#include "NetTimer.h"

/******************************************************************************
*函数定义
******************************************************************************/


 
#define	LIFTUP_VALVE					DRIVER3
#define	FORERAKE_VALVE				DRIVER5//前倾
#define HYPSOKINE_VALVE				DRIVER6//后倾




#define	LIFTDOWN_VALVE			PropDriverCh1			/*PropPump*/

#define HEIGHT_SPEEDLIMIT_SWI		SWI1_R
#define EMERGENCY_REVERSE_SWI		SWI2_R
#define SAFELOCK_SWI						SWI3_R
#define FORWARD_SWI							SWI4_R
#define	BACKWARD_SWI						SWI5_R
#define	LIFT_UP_SWI							SWI6_R
#define	LIFT_DOWN_SWI						SWI7_R
#define QianHouYi_SWI						SWI8_R
#define ZuoYouYi_SWI						DRIVER7_R
#define FORERAKE_SWI						DRIVER10_R
#define HYPSOKINE_SWI						DRIVER11_R
#define	STEER_SAFELOCK_SWI			DRIVER8_R
#define	SLOW_SPEED_SWI					DRIVER9_R

#define KEY_RETURN_SWI					DRIVER4_R


#define	MOVE_THROTTLE			AI_B_AI1_R
#define	LIFT_THROTTLE				AI_B_AI2_R

#define	CHARGE_HIGH_LEVEL		2000
#define	CHARGE_LOW_LEVEL		1000

#define	CAN_190_LOST_NO			(200 / USER_ECU_PERIOD)
#define	CAN_200_LOST_NO			(200 / USER_ECU_PERIOD)
#define	CAN_360_LOST_NO			(200 / USER_ECU_PERIOD)


#define	ECU_POWERON_DELAY_TIME	500
#define	ECU_ANTI_PINCH_TIME 	3000
#define	DRIVER_CLOSE			0
#define	DRIVER_OPEN				1

#define	LIFT_MODE_SWI			0
#define	LIFT_MODE_THROTTLE		1

#define	MOTOR_MIN_SPEED			30		/*怠速转速*/
#define	MOTOR_MAX_SPEED			2000
#define	MOTOR_MAX_SPEED_VALUE	4095
#define	MOTOR_SPEED_RANGE		4096
#define	PUMP_MAX_VALUE			255
#define	PUMP_RANGE				256

#define	PROP_MIN_CURRENT		PARA_PropDMinCurrent1
#define	PROP_MAX_CURRENT		PARA_PropDMaxCurrent1

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

#define	LIFT_MIDDLE_VALUE		5000
//#define	LIFT_RANGE_VALUE		5000
//#define	MOVE_RANGE_VALUE		5000

#define	CHARGE_SWI_AD			2000				

#define	CANID_5AC_SEND_NO		5
#define	CANID_5AC_SEND_PERIOD	(200 / USER_PERIOD)

#define	BAT_LOW_WARING_VAL		20
#define	BAT_LOW_ERR_VAL				10		


#define LIFTUP_VALVE_ERR	ErrCode53
#define	ACT_INIT_ERR				ErrCode103
#define	ACT_LOCK_ERR				ErrCode104
#define	ACT_FAULT_LOCK_ERR			ErrCode105
#define	MOVE_EMS_ERR				ErrCode106
#define	BAT_LOW_2_ERR				ErrCode107
#define	BAT_LOW_1_ERR				ErrCode108


#define	MUTL_PUMP_REQ_ERR			ErrCode187

#define	STEP_FACTOR					(5.0 * 4096 / 255)
#define	MOTOR_MAX_SPD				3000
#define	MOTOR_SWITCH_VALVE_SPD		300
#define	MOTOR_MIN_SPD_BY_CTRL		300

#define	LIFTUP_ERROCDE	
extern void vUserEcuInit(void);
extern void vUserEcuProc(void);


//试验台测试


#endif //#ifndef _USER_COMM_H_
