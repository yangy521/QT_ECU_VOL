/*******************************************************************************
* Filename: UserStackerTruckProc.h	                                           *
* Description: 逻辑层的头文件，包含对应的回调函数	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/07/22    														   *
* Revision:	 														 		   *
*******************************************************************************/
#ifndef _USER_STACKER_TRUCK_PROC_HANGCHAPHZ_H_
#define _USER_STACKER_TRUCK_PROC_HANGCHAPHZ_H_

#include "MstSlvCom.h"
#include "DoPwm.h"
#include "PropDriver.h"
#include "LocalDi.h"
#include "NetTimer.h"

/******************************************************************************
*函数定义
******************************************************************************/


#define	USER_ECU_PERIOD				5	

#define PROPDRIVER_VALVE 	DRIVER3
#define	LIFTUP_VALVE			DRIVER10
#define QIANQIN_VALVE			DRIVER7
#define HOUQIN_VALVE			DRIVER8

#define	LIFTDOWN_VALVE		PropDriverCh1

#define	HEIGHT500_SPEEDLIMIT_SWI			SWI1_R
#define	EmergencyReverse							SWI2_R
#define	SAFELOCK_SWI									SWI3_R
#define	Pedal_SWI											SWI7_R
#define	StreeLock_SWI									SWI8_R 

#define	Human_SWI								  		DRIVER4_R
#define HEIGHT1800_SPEEDLIMIT_SWI		  DRIVER5_R  
#define FENCE1_SWI										DRIVER9_R  
#define FENCE2_SWI										DRIVER10_R
#define LIFTLIMIT_SWI									DRIVER11_R

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
#define	CANID_5AC_SEND_PERIOD	(200 / USER_ECU_PERIOD)

#define	BAT_LOW_WARING_VAL			20
#define	BAT_LOW_ERR_VAL				15	


#define	AI_B_AI1_ERR				ErrCode71
#define	AI_B_AI2_ERR				ErrCode72
#define	AI_B_AI3_ERR				ErrCode73
#define	AI_5V_12V_OUT1_ERR			ErrCode74
#define	AI_5V_12V_OUT2_ERR			ErrCode75

#define WorkHourCountL_ADDR		220
#define WorkHourCountH_ADDR		221

#define PARA_WorkCountL		WorkHourCountL_ADDR
#define PARA_WorkCountH		WorkHourCountH_ADDR


#define	STEP_FACTOR					(5.0 * 4096 / 255)
#define	MOTOR_MAX_SPD				3000
#define	MOTOR_SWITCH_VALVE_SPD		300
#define	MOTOR_MIN_SPD_BY_CTRL		300

#define	LIFTUP_ERROCDE	
extern void vUserEcuInit(void);
extern void vUserEcuProc(void);


//试验台测试


#endif //#ifndef _USER_COMM_H_

 
