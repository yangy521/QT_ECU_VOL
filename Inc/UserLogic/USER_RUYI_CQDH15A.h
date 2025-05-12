/*******************************************************************************
* Filename: UserStackerTruckProc.h	                                           *
* Description: 逻辑层的头文件，包含对应的回调函数	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/07/22    														   *
* Revision:	 														 		   *
*******************************************************************************/
#ifndef _USER_STACKERTRUNK_RUYI_CQDH15A_H_
#define _USER_STACKERTRUNK_RUYI_CQDH15A_H_

#include "MstSlvCom.h"
#include "DoPwm.h"
#include "PropDriver.h"
#include "LocalDi.h"
#include "NetTimer.h"

/******************************************************************************
*函数定义
******************************************************************************/


#define	USER_ECU_PERIOD			5	

/*DI与DO配置*/

#define MAINCONTACTER_VALV 		DRIVER1
#define EBRAKE_VALVE					DRIVER2
#define	LIFTUP_VALVE					DRIVER3
#define	LEG_UP_VALVE					DRIVER4//提腿
#define	OUTRIGGER_UP_VALVE		DRIVER5//支腿升
#define OUTRIGGER_DOWN_VALVE	DRIVER6//支腿降
#ifdef KUNLI_IOTYPE
//#define   									PropDriverCh0		/*29脚*/
#define	LIFTDOWN_VALVE				PropDriverCh0			/*30脚*/
#endif


#define HEIGHT_SPEEDLIMIT_SWI				SWI1_R
#define EMERGENCY_REVERSE_SWI				SWI2_R
#define SAFELOCK_SWI								SWI3_R
#define FORWARD_SWI									SWI4_R
#define	BACKWARD_SWI								SWI5_R
#define	SLOW_REQUEST_SWI						SWI6_R
#define	LIFT_UP_SWI									SWI7_R

#define	ORBITAL_MOVE_SWI						SWI8_R//前后移
#define PEDAL_SWI										DRIVER7_R//踏板
#define	TILT_SWI										DRIVER5_R//倾斜
#define	FLIP_SWI										DRIVER11_R//翻盖开关
#define	LEFT_FENCE_SWI							DRIVER9_R//左护栏
#define	RIGHT_FENCE_SWI							DRIVER10_R//右护栏
#define	STEER_LOCK_SWI							DRIVER8_R//转向互锁 没高电平报40#


	
#define	MOVE_THROTTLE				AI_B_AI1_R	/*16脚*/
#ifdef KUNLI_IOTYPE
#define	LIFT_THROTTLE				AI_B_AI3_R	/*18脚*/
#else
#define	LIFT_THROTTLE				AI_B_AI2_R	/*18脚*/
#endif
//#define										AI_B_AI3_R 	/*21脚*/

#define	CHARGE_HIGH_LEVEL		2000
#define	CHARGE_LOW_LEVEL		1000

#define	CAN_190_LOST_NO			(800 / USER_ECU_PERIOD)
#define	CAN_200_LOST_NO			(800 / USER_ECU_PERIOD)
#define	CAN_360_LOST_NO			(800 / USER_ECU_PERIOD)


#define	ECU_POWERON_DELAY_TIME	500//初始化等待时间


#define	ECU_ANTI_PINCH_TIME 	3000  //防夹手时间
#define	DRIVER_CLOSE			0
#define	DRIVER_OPEN				1

#define	LIFT_MODE_SWI			0
#define	LIFT_MODE_THROTTLE		1

/*起升行走电机相关参数*/
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


#define	CHARGE_SWI_AD			2000				

#define	CANID_5AC_SEND_NO		5
#define	CANID_5AC_SEND_PERIOD	(200 / USER_PERIOD)


/*电量相关*/
#define	BAT_LOW_WARING_VAL		15
#define	BAT_LOW_ERR_VAL				10		

/*故障码配置*/
//


#define	ACT_INIT_ERR				ErrCode101
#define	BAT_LOW_2_ERR				ErrCode102
#define	BAT_LOW_1_ERR				ErrCode103
#define	BMS_NOCAN_ERR				ErrCode104
#define	STEER_NOCAN_ERR			ErrCode105
#define BAT_TYPE_ERR				ErrCode106

#define	MUTL_PUMP_REQ_ERR		ErrCode107
#define	SWI_SEQUENCE_ERR		ErrCode108
#define EMS_LOCK_ERR				ErrCode109



#define	STEP_FACTOR					(5.0 * 4096 / 255)


#define	MOTOR_MAX_SPD				3000
#define	MOTOR_SWITCH_VALVE_SPD		300
#define	MOTOR_MIN_SPD_BY_CTRL		300

#define	LIFTUP_ERROCDE	
extern void vUserEcuInit(void);
extern void vUserEcuProc(void);


//试验台测试


#endif //#ifndef _USER_COMM_H_