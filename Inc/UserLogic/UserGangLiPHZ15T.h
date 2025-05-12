/*******************************************************************************
* Filename: UserStackerTruckProc.h	                                           *
* Description: �߼����ͷ�ļ���������Ӧ�Ļص�����	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/07/22    														   *
* Revision:	 														 		   *
*******************************************************************************/
#ifndef _USER_GANGLLI_PHZ15T_H_
#define _USER_GANGLLI_PHZ15T_H_

#include "MstSlvCom.h"
#include "DoPwm.h"
#include "PropDriver.h"
#include "LocalDi.h"
#include "NetTimer.h"

/******************************************************************************
*��������
******************************************************************************/

#define	USER_ECU_PERIOD				5	

//#define FORWARD_MOVESTATE2HMI					(1 << 0)
//#define REVERSE_MOVESTATE2HMI					(1 << 1)
//#define UPRIGNT_MOVESTATE2HMI					(1 << 4)
//#define PARK_MOVESTATE2HMI						(1 << 5)
//#define SEATBELT_MOVESTATE2HMI				(1 << 6)
//#define MAINT_MOVESTATE2HMI						(1 << 7)

#if (USER_TYPE == USER_GANGLI_PHZ15T_MOVE)
	#define CHARGE_SWI			SWI2_R
	#define SAFELOCK_SWI		SWI3_R
	#define FOOTBRAKE_SWI		SWI4_R
	#define THROTTLE_SWI		SWI5_R
	#define HANDBRAKE_SWI		SWI6_R
	#define FORWARD_SWI			SWI7_R
	#define BACKWARD_SWI		SWI8_R
	#define SAFEBELT_SWI		DRIVER7_R
	#define SLOWMODE_SWI		DRIVER8_R
	#define SPEEDLIMIT_SWI	DRIVER9_R
	
	#define DAOCHE_DO			DRIVER3
	#define FENGSHAN_DO		DRIVER4
	
	#define MOVE_THROTTLE			AI_B_AI1_R
	#define ANGLE_SENSOR			AI_B_AI2_R
	
#elif (USER_TYPE == USER_GANGLI_PHZ15T_LIFT)
	#define SAFELOCK_SWI		SWI3_R
	#define SHUJU_SWI				SWI5_R
	#define QINGXIE_SWI			SWI6_R
	#define LIFT_SWI				SWI7_R
	#define CEYI_SWI				SWI8_R
	
	#define DOWN_DO				DRIVER2
	
	#define MOVE_THROTTLE			AI_B_AI1_R
	
#endif //USER_ECU_OR_ET_TYPE == USER_GANGLI_PHZ15T_MOVE

//#define	CAN_17F_LOST_NO			(5000 / USER_ECU_PERIOD)
//#define	CAN_47F_LOST_NO			(1000 / USER_ECU_PERIOD)

#define	ECU_POWERON_DELAY_TIME	500
#define	ECU_ANTI_PINCH_TIME 	3000
#define	DRIVER_CLOSE			0
#define	DRIVER_OPEN				1

//#define	LIFT_MODE_SWI			0
//#define	LIFT_MODE_THROTTLE		1

#define	MOTOR_MIN_SPEED			30		/*����ת��*/
#define	MOTOR_MAX_SPEED			2000
#define	MOTOR_MAX_SPEED_VALUE	4095
#define	MOTOR_SPEED_RANGE		4096
#define	PUMP_MAX_VALUE			255
#define	PUMP_RANGE				256
#define MAX_SPEED_RATE		100			//����ٶȵ�λ

#define	PROP_MIN_CURRENT		PARA_PropDMinCurrent0
#define	PROP_MAX_CURRENT		PARA_PropDMaxCurrent0

#define MOVE_THROTTLE_TYPE	PARA_ThrottleType
#define	MOVE_THROTTLE_MIN		PARA_ThrottleFDeadZoneMinVal
#define	MOVE_THROTTLE_MAX		PARA_ThrottleFDeadZoneMaxVal
#define	MOVE_THROTTLE_MID		PARA_ThrottleFMidVal

#define	STEER_ANALOG_MIN	PARA_BrakeFDeadZoneMinVal
#define	STEER_ANALOG_MAX	PARA_BrakeFDeadZoneMaxVal
#define	STEER_ANALOG_MID	PARA_BrakeFMidVal

#define	TURN_START_ANGLE		PARA_TurnWithDecStartAngle
#define	TURN_END_ANGLE			PARA_TurnWithDecEndAngle
#define	START_ANGLE_SPD			PARA_AngleWithStartSpdPer
#define	END_ANGLE_SPD				PARA_AngleWithEndSpdPer

#define BATTERY_TYPE 			PARA_BatteryType
#define	USERINFO_MODE			PARA_ValueOpenPercentage

#define	BAT_LOW_WARING_VAL			20
#define	BAT_LOW_ERR_VAL				15	

//#define	LIFTUP_VALVE_ERR			ErrCode53			//����������
//#define	LIFTDOWN_VALVE_ERR			ErrCode54		//�½�������

#define	ACT_INIT_ERR				ErrCode101		//�ϵ������
#define	ACT_LOCK_ERR				ErrCode102		//�����߼�����
#define	ACT_FAULT_LOCK_ERR	ErrCode103		//������������ͨѶ����
#define	BMS_NOCAN_ERR				ErrCode105		//﮵��ͨѶ��ʱ
#define	BAT_PARA_ERR				ErrCode106		//������ʹ���

#define	AI_B_AI1_ERR				ErrCode71			//AI������
#define	AI_B_AI2_ERR				ErrCode72
#define	AI_B_AI3_ERR				ErrCode73
#define	AI_5V_12V_OUT1_ERR			ErrCode74
#define	AI_5V_12V_OUT2_ERR			ErrCode75

#define	BAT_LOW_2_ERR					ErrCode111			//�������ع���
//#define	BMS_MC_OPEN_ERR				ErrCode148			//﮵�����ϽӴ���
//#define	BMS_SPD_LIMIT_ERR			ErrCode149			//﮵�������
//#define	BMS_LIFTUP_ERR				ErrCode150			//﮵������ֹ����	
//#define	BMS_SPD_STOP_ERR			ErrCode151			//﮵������ֹ����
#define	BAT_LOW_1_ERR				ErrCode112				//������ȹ���

//#define	MUTL_PUMP_REQ_ERR			ErrCode187		//����ͱ�����

#define	STEP_FACTOR					(5.0 * 4096 / 255)
#define	MOTOR_MAX_SPD				3000
#define	MOTOR_SWITCH_VALVE_SPD		300
#define	MOTOR_MIN_SPD_BY_CTRL		300

#define	LIFTUP_ERROCDE	
extern void vUserEcuInit(void);
extern void vUserEcuProc(void);

#endif 