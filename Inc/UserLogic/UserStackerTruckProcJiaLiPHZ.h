/*******************************************************************************
* Filename: UserStackerTruckProc.h	                                           *
* Description: 逻辑层的头文件，包含对应的回调函数	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/07/22    														   *
* Revision:	 														 		   *
*******************************************************************************/
#ifndef _USER_STACKER_TRUCK_PROC_PHZ_H_
#define _USER_STACKER_TRUCK_PROC_PHZ_H_

#include "MstSlvCom.h"
#include "DoPwm.h"
#include "PropDriver.h"
#include "LocalDi.h"
#include "NetTimer.h"

/******************************************************************************
*函数定义
******************************************************************************/


#define	USER_ECU_PERIOD				5	

#define LIFTDOWN_VALVE						DRIVER3

#define	SAFELOCK_SWI							SWI3_R
#define ForWard_SWI								SWI4_R
//#define BackWard_SWI				`			SWI5_R
#define QinXie_SWI								SWI6_R
#define LIFTUP_SWI								SWI7_R
#define PARK_SWI									SWI8_R

#define SPEEDUP_SWI								DRIVER7_R
#define LIFTDOWN_SWI							DRIVER9_R

#define FORWARD_MOVESTATE2HMI					(1 << 0)
#define REVERSE_MOVESTATE2HMI					(1 << 1)
#define UPRIGNT_MOVESTATE2HMI					(1 << 4)
#define PARK_MOVESTATE2HMI						(1 << 5)
#define SEATBELT_MOVESTATE2HMI				(1 << 6)
#define MAINT_MOVESTATE2HMI						(1 << 7)


#define	MOVE_THROTTLE			AI_B_AI1_R


#define	CHARGE_HIGH_LEVEL		2000
#define	CHARGE_LOW_LEVEL		1000

#define	CAN_1AC_LOST_NO			(1000 / USER_ECU_PERIOD)
#define	CAN_62C_LOST_NO			(10000 / USER_ECU_PERIOD)


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

#define	PROP_MIN_CURRENT		PARA_PropDMinCurrent0
#define	PROP_MAX_CURRENT		PARA_PropDMaxCurrent0

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
#define	CANID_5AC_SEND_PERIOD	(200 / USER_ECU_PERIOD)

#define	BAT_LOW_WARING_VAL			20
#define	BAT_LOW_ERR_VAL				15	

#define	LIFTUP_VALVE_ERR			ErrCode53
#define	LIFTDOWN_VALVE_ERR			ErrCode54

#define	ACT_INIT_ERR				ErrCode63
#define	ACT_LOCK_ERR				ErrCode64
#define	ACT_FAULT_LOCK_ERR			ErrCode65
#define	MOVE_EMS_ERR				ErrCode66
#define	BAT_LOW_2_ERR				ErrCode67
#define	BAT_LOW_1_ERR				ErrCode68

#define	AI_B_AI1_ERR				ErrCode71
#define	AI_B_AI2_ERR				ErrCode72
#define	AI_B_AI3_ERR				ErrCode73
#define	AI_5V_12V_OUT1_ERR			ErrCode74
#define	AI_5V_12V_OUT2_ERR			ErrCode75


#define	DEV_62C_LOST_ERR			ErrCode143
#define	RENTAL_TIMEOUT_ERR			ErrCode146

#define	BMS_NOCAN_ERR				ErrCode147
#define	BMS_MC_OPEN_ERR				ErrCode148
#define	BMS_SPD_LIMIT_ERR			ErrCode149
#define	BMS_LIFTUP_ERR				ErrCode150
#define	BMS_SPD_STOP_ERR			ErrCode151

#define	MUTL_PUMP_REQ_ERR			ErrCode187

#define	BAT_PARA_ERR				ErrCode251

#define	STEP_FACTOR					(5.0 * 4096 / 255)
#define	MOTOR_MAX_SPD				3000
#define	MOTOR_SWITCH_VALVE_SPD		300
#define	MOTOR_MIN_SPD_BY_CTRL		300

#define	LIFTUP_ERROCDE	
extern void vUserEcuInit(void);
extern void vUserEcuProc(void);


//试验台测试


#endif //#ifndef _USER_COMM_H_
