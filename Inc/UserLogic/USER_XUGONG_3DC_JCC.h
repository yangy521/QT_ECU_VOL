/*******************************************************************************
* �칤 ��ֱ�����泵 �߼����� * 						   *
* Author: QExpand; Ji Wenshuai                                                        *
* Date: 2023/12/19   														   *
* Revision:	 														 		   *
*******************************************************************************/

#ifndef _USER_FORKLIFT_XUGONG_3DC_JCC_H_
#define _USER_FORKLIFT_XUGONG_3DC_JCC_H_
#if (USER_TYPE == USER_XUGONG_3DC_JCC)
#include "PcuProc.h"
#include "MstSlvCom.h"
#include "DoPwm.h"
#include "PropDriver.h"
#include "LocalDi.h"
#include "NetTimer.h"
#include "Para.h"
#include "stdint.h"

/******************************************************************************
*��������
******************************************************************************/
//#define test 

#define	USER_ECU_PERIOD	TIMER_PLC_PERIOD

			#define H9_INIT				0	
			#define	H9_HIG16CHOOSE		1
			#define H9_LOW16CHOOSE		2
			#define H9_SAVE				3
			#define H9_SAVEFAIL			4
			#define H9_UPLIMIT_SAVE		5
			#define H9_DOWNLIMIT_SAVE	6

/*switch forward and series pump */

#define MAIN_CONTACTOR    		DRIVER1
#define BRAKECOIL_1				DRIVER2
#define BRAKECOIL_2				DRIVER3


#define	TURNRIGHT_PUMP			DRIVER4
#define	TURNLEFT_PUMP			DRIVER5
#define	SPEAKER_PUMP			DRIVER6
#define	BLINK_LED				DRIVER7
#define	LIFTUP_PUMP				DRIVER8
#define HIGH_SLOW_SPEED_PUMP	DRIVER9
#define	BLINK_BEEP				DRIVER10




#define	FORWARD_PUMP_R			DRIVER2_R
#define	LIFTUP_PUMP_R			DRIVER3_R
#define	TURNRIGHT_PUMP_R		DRIVER4_R
#define	TURNLEFT_PUMP_R			DRIVER5_R
#define	SPEAKER_PUMP_R			DRIVER6_R
#define	BLINK_LED_R				DRIVER7_R
#define	BACKWARD_PUMP_R			DRIVER8_R
#define	HIGH_SLOW_SPEED_PUMP_R	DRIVER9_R
#define	BLINK_BEEP_R			DRIVER10_R
#define	LIFTDOWN_PUMP_R			DRIVER12_R





#define	LIFTDOWN_PUMP1	PropDriverCh0			/*PropPump*/
#define LIFTDOWN_PUMP2	PropDriverCh1

/*ģ����*/
#define PRESSURE_SENSOR_CHANNEL1		AI_B_AI1_R  /**/
#define ANGLE_SENSOR_CHANNEL 	AI_B_AI2_R
#define PRESSURE_SENSOR_CHANNEL2	AI_B_AI3_R
//#define AI_B_AI3_R


#define	PCU_SWICTH			SWI1_R
#define	TILT_SIWTCH			SWI2_R
#define LOWER_CONTROLL_DOWN	SWI3_R
#define LOWER_CONTROLL_UP	SWI4_R
#define	PIT_SWITCH			SWI5_R

#define	UP_LIMIT_SWITCH		SWI7_R
#define	DOWN_LIMIT_SWITCH	SWI8_R	






#define	ECU_POWERON_DELAY_TIME		500
#define SWITCH_CHECK_TIME			2500
#define	ECU_ANTI_PINCH_TIME 		3000

#define ALARM_TIME					5000


#define	PUMP_CLOSE_PERCENTAGE		0
#define	PUMP_OPEN_PERCENTAGE		1

#define MOTOR_CLOESE_SPEED		10
#define	MOTOR_MIN_SPEED			200		/*����ת��*/
#define MOTOR_STEER_SPEED		600
#define MOTOR_CHANGE_SPEED  	300		/**/
#define	MOTOR_MAX_SPEED			2000
#define	MOTOR_MAX_SPEED_VALUE	4064

#define  PROP_FDB_CLOSE_VALUE	100
#define  PUMP_FDB_CLOSE_VALUE	6
#define  MOTOR_FDB_CLOSE_VALUE	30

#define UP_CONTROL_MODE	1
#define LOWER_CONTROL_MODE	0


#define	PROP_PIT_VALUE			(_IQ(PROPD_MAX_CURRENT * 0.3 / PROPD_STD_CURRENT))

#define PROP_CURRENT_FACOTR		0.0016							/*0.0016 = (3.3 * 7.5 / 4096 / (7.5 + 68) / 0.05)*/
#define	PROP_MIN_CURRENT		PARA_PropDMinCurrent1
#define	PROP_MAX_CURRENT		PARA_PropDMaxCurrent1


#define	SWI1_EXTINPUT			0
#define	SWI2_EXTINPUT			1
#define	SWI3_EXTINPUT			2
#define	SWI4_EXTINPUT			3
#define	SWI5_EXTINPUT			4
#define	SWI6_EXTINPUT			5
#define	SWI7_EXTINPUT			6
#define	SWI8_EXTINPUT			7


#define	SLOW_KEY				0
#define	SPEAKER_KEY				1
#define	MOVE_KEY				2
#define	LIFT_KEY				3
#define	ENABLE_KEY				4
#define	LEFT_KEY				5
#define	RIGHT_KEY				6

#define	FORWARD_DISABLE_FLAG	(1 << 0)
#define	BACKWARD_DISABLE_FLAG	(1 << 1)
#define	LIFTUP_DISABLE_FLAG		(1 << 2)
#define	LIFTDOWN_DISABLE_FLAG	(1 << 3)
#define	TURNRIGHT_DISABLE_FLAG	(1 << 4)
#define	TURNLEFT_DISABLE_FLAG	(1 << 5)
#define	BARKE_DISABLE_FLAG		(1 << 6)
#define	PARALLE_DISABLE_FLAG	(1 << 7)

#define	PIT_DISABLE_FLAG		(1 << 12)
#define	TILT_DISABLE_FLAG		(1 << 13)
#define	DISABLE_PCU_FALG		(1 << 14)
#define	SPD_AFTER_LIFT_FALG		(1 << 15)		/*except for down*/

/*lilu 20230707 ��ֹǰ�����Ķ���*/
#define	FORWARD_DISABLE_ACT
/*lilu 20230707 ��ֹ���˷��Ķ���*/


#define	STEP_FACTOR					(5.0 * 4064 / 255)
#define TIME_FACTOR					20
#define FILTER_FACTOR				3 
#define	MOTOR_MAX_SPD				3000
#define	MOTOR_SWITCH_VALVE_SPD		300
#define	MOTOR_MIN_SPD_BY_CTRL		300

#define PressureCali_Times			20
#define CALIBRATION_INIT_TIME		3000
#define CALIBRATION_DELAY_TIME		3000
#define CALIBRATION_PROC_TIME		2000
#define MOTOR_SWITCH_

/*������*/
/*����Ƿ��Ӧ�칤����*/
/*��¼һ����ʱû���Ĺ���*/
#define	SYSTEM_INIT_ERR					ErrCode101		/*ϵͳ��ʼ������*/
#define	SYSTEM_COMM_ERR					ErrCode102		/*ϵͳͨ�Ŵ���*/
#define	INVALID_OPT_SET_ERR				ErrCode103		/*��Чѡ�����ô���*/
#define	EERPOM_ERR						ErrCode104		/*�������ݴ���*/
#define	LI_BATTERY_LOSS_ERR				ErrCode105		/*﮵��ͨѶ��ʧ*/
#define	LIFT_BUTTON_ERR					ErrCode106		/*�ϵ�ʱ������������*/
#define	SLOW_BUTTON_ERR					ErrCode107		/*�ϵ�ʱ���ٰ�������*/
#define	MOVE_BUTTON_ERR					ErrCode108		/*�ϵ�ʱ���߰�������*/
#define	GPS_CONNECT_ERR					ErrCode109		/*GPS���Ӵ���*/
#define	MAIN_CONNECT_ERR				ErrCode110		/*���Ӵ�������*/

#define	UPDOWN_BUTTON_ERR				ErrCode112		/*����ʱ�����������½���ť�򿪴���*/
#define	BMS_BATTERY_TEMP_DIFF2_ERR		ErrCode113		/*BMS-����²����2*/
#define	BMS_BATTERY_TEMP_HIGH1_ERR		ErrCode114		/*BMS-����¶ȹ���1*/
#define	BMS_DISCHARGE_TEMP_HIGH2_ERR	ErrCode115		/*BMS-�ŵ��¶ȹ���2*/
#define	BMS_DISCHARGE_CUR_HIGH1_ERR		ErrCode116		/*BMS-�ŵ��������1*/
#define	BMS_DISCHARGE_CUR_HIGH2_ERR		ErrCode117		/*BMS-�ŵ��������2*/
#define	PIT_PROCETION_ERR				ErrCode118		/*�Ӷ���������*/
#define	BMS_TOTAL_VOL_LOW1_ERR			ErrCode119		/*BMS-�ܵ�ѹ����1*/
//#define BMS_OFFLINE						ErrCode120		/*BMSͨѶ����*/					/*��������*/
#define	BMS_TOTAL_VOL_LOW2_ERR		ErrCode120		/*BMS-�ܵ�ѹ����2*/			/*��������*/
#define	BMS_SINGLE_VOL_LOW1_ERR			ErrCode121		/*BMS-�����ѹ����1*/
#define	BMS_SINGLE_VOL_LOW2_ERR			ErrCode122		/*BMS-�����ѹ����2*/

#define	LOW_VALVE2_ERR						ErrCode127	/*�½���2����*/

#define	BMS_BATTERY_VOL_DIFF_HIGH_ERR	ErrCode130		/*BMS-���ѹ�����*/
#define	PRESSURE_SENSOR_ERR				ErrCode131		/*ѹ������������*/
#define ANGLE_SENSOR_ERR				ErrCode132		/*�Ƕȴ���������*/
#define	BATTERY_TYPE_ERR				ErrCode133		/*������ʹ���*/

#define	WEIGHT_CALI_REVESER_ERR			ErrCode135		/*���ر궨��*/
#define	BATTERY_LOW_CAP1_ERR			ErrCode136		/*��ص�����һ������*/

#define	CALIBRATION_FAILURE_ERR			ErrCode138		/*δ�궨��ɻ�궨ʧ��*/
#define	MCU_ECU_COMMUNICATION_ERR		ErrCode139		/*MCUͨ�Ź���*/
#define	HMI_COMMUNICATION_ERR			ErrCode140		/*�Ǳ�ͨѶ����*/

#define	PLATFORM_LEVEL1_LOCK_ERR		ErrCode141		/*ƽ̨һ������*/
#define	PLAT_LEFT_BUTTON_ERR			ErrCode142		/*����ʱ��ƽ̨����ת��ť���´���*/
#define	PLAT_RIGHT_BUTTON_ERR			ErrCode143		/*����ʱ��ƽ̨����ת��ť���´���*/
#define	PLATFORM_LEVEL2_LOCK_ERR		ErrCode144		/*ƽ̨��������*/

#define	ENABLE_BUTTON_ERR				ErrCode146		/*����ʱ��ƽ̨�ֱ�ʹ�ܿ��ذ�ť���´���*/
#define	HANDLE_NOT_IN_MIDDLE_POSI_ERR	ErrCode147		/*����ʱ��ƽ̨�ֱ�������λ����*/

#define	FORWARD_VALVE_ERR				ErrCode152		/*ǰ��������*/
#define	BACKWARD_VALVE_ERR				ErrCode153		/*���˷�����*/
#define LIFT_UP_VALVE_ERR				ErrCode154		/*����������*/	
#define	LIFT_DOWN_VALVE_ERR				ErrCode155		/*�½�������*/
#define	TURN_RIGHT_VALVE_ERR			ErrCode156		/*��ת������*/
#define	TURN_LEFT_VALVE_ERR				ErrCode157		/*��ת������*/
//#define BLINK_LED_VALV_ERR			ErrCode158		/*LED����ƴ���*/
#define BLIKN_BEEP_VALVE_ERR			ErrCode159		/*����������*/


//#define BRAKE_VALVE_ERR				ErrCode158		/*ɲ��������*/
//#define	PARALLEL_VALVE_ERR				ErrCode159		/*����������*/


#define	CONTROLLER_ERR					ErrCode160		/*����������*/
//#define	CTRL_CURRENT_SENSOR_ERR		ErrCode161		/*��������������������*/
#define	CTRL_HARDWARE_ERR				ErrCode162		/*������Ӳ���𻵹���*/
#define	PUMP_DRIVE_OPEN_ERR				ErrCode163		/*�õ�����·����*/
#define	WALKING_DRIVE_OPEN_ERR			ErrCode164		/*���ߵ�����·����*/
#define	CONTROL_VOL_5V_ERR				ErrCode165		/*���Ƶ�ѹ5V����*/
#define	RIGHT_DRIVE_OPEN_ERR			ErrCode166		/*�ҵ�����·����*/
#define	CONTROL_VOL_12V_ERR				ErrCode167		/*���Ƶ�ѹ12V����*/
#define	BAT_LOW_CAP2_ERR				ErrCode168		/*��ص͵�����������*/
//#define	HIGH_ZERO_CURRENT_ERR		 ErrCode169		/*����λ��������*/
#define	CTRL_BUS_VOL_HIGH_ERR			ErrCode170		/*������ĸ�ߵ�ѹ���߹���*/
#define	PRE_CHARGE_FAULT_ERR			ErrCode171		/*Ԥ�����*/
#define	CTRL_BUS_VOL_LOW_ERR			ErrCode172		/*������ĸ�ߵ�ѹ���͹���*/
#define	CTRL_TEMP_LOW_ERR				ErrCode173		/*���������¹���*/
#define	CTRL_TEMP_HIGH1_ERR				ErrCode174		/*����������һ������*/
#define	PUMP_MOTOR_TEMP1_ERR			ErrCode175		/*�õ���¶�һ������*/
#define	PUMP_MOTOR_ENCODER_ERR			ErrCode176		/*�õ������������*/
#define	MOTOR_ENCODE_ERR				ErrCode177		/*����������*/
#define	PUMP_MOTOR_OVER_CURRENT_ERR		ErrCode178		/*�õ�����������*/
#define	PUMP_MOTER_TEMP2_ERR			ErrCode179		/*�õ���¶ȶ�������*/
#define	OVER_80_PER_LOAD_ERR			ErrCode180		/*���� 80%���ر���*/
#define	CTRL_TEMP2_ERR					ErrCode181		/*�������¶ȶ�������*/
#define	RIGHT_BRAKE_ERR					ErrCode182		/*��ɲ������*/
#define	LEFT_BRAKE_ERR					ErrCode183		/*��ɲ������*/
#define	PUMP_MOTOR_STALL_ERR			ErrCode184		/*�õ����ת��ʧ��*/
#define	LEFT_DRIVE_MOTER_STALL_ERR		ErrCode185		/*��ǣ�������ת��ʧ��*/
#define	RIGTH_DRIVE_MOTOR_STALL_ERR		ErrCode186		/*��ǣ�������ת��ʧ��*/

#define	CTRL_DRIVE_LONG_RUN_ERR			ErrCode189		/*����������ʱ���������*/
#define	OVER_90_PER_LOAD_ERR			ErrCode190		/*���� 90%���ر���*/
#define	LEFT_MOTOR_OVER_CURRENT_ERR		ErrCode191		/*����������������*/
#define RIGTH_MOTOOR_OVER_CURRENT_ERR	ErrCode192		/*�ҵ��������������*/

#define	OVER_99_PER_LOAD_ERR			ErrCode199		/*����99%���ر���*/
#define	PLAT_OVERLOAD_ERR				ErrCode200		/*ƽ̨���ر���*/
#define	MACHINE_TILT_OVER_SAFETY_ERR	ErrCode201		/*������б������ȫ�޶�����*/
#define	ANTICOLLISION_PROTECT_ERR		ErrCode202		/*��ײ��������,��ʾ39*/
#define	HANDLE_CONMUNICATION_ERR		ErrCode244		/*�ֱ����߱���*/

#define	TIMER_SwitchCheck	TIMER_USER_1
#define TIMER_Calibration	TIMER_USER_2
#define	TIMER_AlarmDelay	TIMER_USER_3

//tbox���������ַ����ȷ��
#define	LIFTTIMES_ADDR_L16
#define LIFTTIMES_ADDR_H16
#define STEERTIMES_ADDR_L16
#define STEERTIMES_ADDR_H16
#define MOVETIES_ADDR_L16
#define MOVETIES_ADDR_H16
#define OVERLOADTIMES_ADDR_L16
#define OVERLOADTIMES_ADDR_H16
#define LIFTHOUR_ADDR_L16
#define LIFTHOUR_ADDR_H16
#define STEERHOUR_ADDR_L16
#define	STEERHOUR_ADDR_H16
#define	LOWERMOVEHOUR_ADDR_L16
#define LOWERMOVEHOUR_ADDR_H16		
#define UPMOVEHOUR_ADDER_L16
#define UPMOVEHOUR_ADDR_H16
#define DOWNHOUR_ADDR_L16
#define DOWNHOUR_ADDR_H16

//
#define ADDR_INVALID		999

extern void vUserEcuInit(void);
extern void vUserEcuProc(void);


#endif
#endif