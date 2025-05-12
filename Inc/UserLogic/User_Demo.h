/*******************************************************************************
* Filename: UserStackerTruckProc.h	                                           *
* Description: 逻辑层的头文件，包含对应的回调函数	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/07/22    														   *
* Revision:	 														 		   *
*******************************************************************************/
#ifndef _USER_DEMO_H_
#define _USER_DEMO_H_

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

//#define   									PropDriverCh0		/*29脚*/
#define	LIFTDOWN_VALVE			PropDriverCh1			/*30脚*/


#define HEIGHT_SPEEDLIMIT_SWI		SWI1_R
#define EMERGENCY_REVEERSE_SWI	SWI2_R
#define SAFELOCK_SWI				SWI3_R
#define FORWARD_SWI			SWI4_R
#define	BACKWARD_SWI		SWI5_R
#define	LIFT_UP_SWI			SWI6_R
#define	LIFT_DOWN_SWI			SWI7_R
#define LEG_UP_SWI			SWI8_R
#define	LEG_DOWN_SWI			DRIVER7_R
#define	STEER_SAFELOCK_SWI			DRIVER8_R
#define	SLOW_SPEED_SWI			DRIVER9_R
#define	OUTRIGGER_UP_SWI			DRIVER10_R
#define	OUTRIGGER_DOWN_SWI			DRIVER11_R


	
#define	MOVE_THROTTLE				AI_B_AI1_R	/*16脚*/
#define	LIFT_THROTTLE				AI_B_AI2_R	/*18脚*/
//#define										AI_B_AI3_R 	/*21脚*/

#define	CHARGE_HIGH_LEVEL		2000
#define	CHARGE_LOW_LEVEL		1000

#define	CAN_1AC_LOST_NO			(1000 / USER_PERIOD)
#define	CAN_62C_LOST_NO			(10000 / USER_PERIOD)


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


#define	CHARGE_SWI_AD			2000				

#define	CANID_5AC_SEND_NO		5
#define	CANID_5AC_SEND_PERIOD	(200 / USER_PERIOD)


/*电量相关*/
#define	BAT_LOW_WARING_VAL		15
#define	BAT_LOW_ERR_VAL				10		

/*故障码配置*/
//
#define	LIFTUP_VALVE_ERR			ErrCode53
#define	LEG_UP_VALVE_ERR		ErrCode54
#define	OUTRIGGER_UP_VALVE_ERR		ErrCode55
#define OUTRIGGER_DOWN_VALVE_ERR 	ErrCode56

#define	LIFTDOWN_VALVE_ERR			ErrCode61

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

#define	STEP_FACTOR					(5.0 * 4096 / 255)




extern void vUserEcuInit(void);
extern void vUserEcuProc(void);


//试验台测试


#endif //#ifndef _USER_COMM_H_