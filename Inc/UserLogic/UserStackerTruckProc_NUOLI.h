/*******************************************************************************
* Filename: UserStackerTruckProc.h	                                           *
* Description: 逻辑层的头文件，包含对应的回调函数	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/07/22    														   *
* Revision:	 														 		   *
*******************************************************************************/
#ifndef _USER_NUOLI_PS16_H_
#define _USER_NUOLI_PS16_H_

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
/*DO1与DO2一般不操作，driver flag低两位置零*/

#define	LIFTUP_VALVE					DRIVER3//保压阀
#define	LEFT_MOVE_VALVE				DRIVER5//左移
#define RIGHT_MOVE_VALVE			DRIVER6//右移

#define LIFTDOWN_VALVE 				PropDriverCh0		/*29脚*/
//#define	LIFTDOWN_VALVE				PropDriverCh1			/*30脚*/


#define HEIGHT_SPEEDLIMIT_SWI				SWI1_R/*高度限速*/
#define EMERGENCY_REVERSE_SWI				SWI2_R/*急反*/
#define SAFELOCK_SWI								SWI3_R/*互锁*/
#define STEER_SPEED_LIMIT						SWI4_R/*转弯速度*/
#define	PEDAL_SWI										SWI5_R/*踏板*/
#define	FENCE2_SWI									SWI6_R/*护栏2*/
#define	LIFT_LIMT_SWI								SWI7_R/*起升限位*/
#define	HEIGHT_LIMIT_SWI						SWI8_R/*高度限位*/
#define FENCE1_SWI									DRIVER7_R/*护栏1*/
#define	STEER_SAFE_LOCK							DRIVER8_R/*转向互锁*/



	
#define	MOVE_THROTTLE				AI_B_AI1_R	/*16脚*/
#define	LIFT_THROTTLE				AI_B_AI2_R	/*18脚*/
//#define										AI_B_AI3_R 	/*21脚*/

#define	CHARGE_HIGH_LEVEL		2000
#define	CHARGE_LOW_LEVEL		1000

#define	CAN_190_LOST_NO			(200 / USER_ECU_PERIOD)
#define	CAN_200_LOST_NO			(200 / USER_ECU_PERIOD)
#define	CAN_360_LOST_NO			(200 / USER_ECU_PERIOD)


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


/*小时计*/
#define HOURCOUNT_PERIOD_NUOLI 100
/*故障码配置*/
//
#define	LIFTUP_VALVE_ERR			ErrCode53
#define	LEG_UP_VALVE_ERR		ErrCode54
#define	OUTRIGGER_UP_VALVE_ERR		ErrCode55
#define OUTRIGGER_DOWN_VALVE_ERR 	ErrCode56

#define	LIFTDOWN_VALVE_ERR			ErrCode61

#define	ACT_INIT_ERR				ErrCode60

#define	BMS_NOCAN_ERR			ErrCode65
#define	STEER_NOCAN_ERR				ErrCode66
#define	BAT_LOW_2_ERR				ErrCode67
#define	BAT_LOW_1_ERR				ErrCode68
#define BAT_TYPE_ERR				ErrCode69


#define	AI_B_AI1_ERR				ErrCode71
#define	AI_B_AI2_ERR				ErrCode72
#define	AI_B_AI3_ERR				ErrCode73
#define	AI_5V_12V_OUT1_ERR			ErrCode74
#define	AI_5V_12V_OUT2_ERR			ErrCode75

#define CanIDlost1_ERR	ErrCode100
#define CanIDlost2_ERR	ErrCode101
#define CanIDlost3_ERR	ErrCode102
#define CanIDlost4_ERR	ErrCode103

#define	SWI_SEQUENCE_ERR1				ErrCode110
#define	SWI_SEQUENCE_ERR2				ErrCode111
#define	SWI_SEQUENCE_ERR3				ErrCode112

#define	DEV_62C_LOST_ERR			ErrCode143
#define	RENTAL_TIMEOUT_ERR			ErrCode146

//#define	BMS_NOCAN_ERR				ErrCode147
#define	BMS_MC_OPEN_ERR				ErrCode148
#define	BMS_SPD_LIMIT_ERR			ErrCode149
#define	BMS_LIFTUP_ERR				ErrCode150
#define	BMS_SPD_STOP_ERR			ErrCode151
//#define STEER_NOCAN_ERR				ErrCode153

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