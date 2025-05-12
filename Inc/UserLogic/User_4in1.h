/*******************************************************************************
* Filename: UserEcuProc.h	                                             	   *
* Description: �߼����ͷ�ļ���������Ӧ�Ļص�����	   							   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/05/30    														   *
* Revision:	 														 		   *
*******************************************************************************/
#ifndef _USER_ECU_PROC_H_
#define _USER_ECU_PROC_H_

#include "PcuProc.h"
#include "MstSlvCom.h"
#include "DoPwm.h"
#include "PropDriver.h"
#include "LocalDi.h"
#include "NetTimer.h"

/******************************************************************************
*��������
******************************************************************************/


#define	USER_ECU_PERIOD		TIMER_PLC_PERIOD

#define TIMER_PDO 

#define MAIN_CONTACTOR    DRIVER1
#define BRAKECOIL_1				DRIVER2
#define BRAKECOIL_2				DRIVER3


#define	TURNRIGHT_PUMP		DRIVER4
#define	TURNLEFT_PUMP			DRIVER5
#define	SPEAKER_PUMP			DRIVER6
#define	BLINK_LED					DRIVER7
#define	LIFTUP_PUMP				DRIVER8
//#define	SERIES_PARALLE_PUMP		DRIVER9
#define	BLINK_BEEP				DRIVER10






#define	SPEAKER_PUMP_R				DRIVER6_R
#define	BACKWARD_PUMP_R				DRIVER8_R
#define	FORWARD_PUMP_R				DRIVER2_R
#define	TURNLEFT_PUMP_R				DRIVER4_R
#define	TURNRIGHT_PUMP_R			DRIVER5_R
#define	LIFTUP_PUMP_R					DRIVER3_R
#define	LIFTDOWN_PUMP_R				DRIVER12_R
#define	BLINK_LED_R						DRIVER7_R
#define	BLINK_BEEP_R					DRIVER10_R
#define	SERIES_PARALLE_PUMP_R	DRIVER9_R



#define	SPEAKER_PUMP_T				TIMER_Drive6Check
#define	BACKWARD_PUMP_T				TIMER_Drive8Check
#define	FORWARD_PUMP_T				TIMER_Drive2Check
#define	TURNLEFT_PUMP_T				TIMER_Drive4Check
#define	TURNRIGHT_PUMP_T			TIMER_Drive5Check
#define	LIFTUP_PUMP_T					TIMER_Drive3Check
#define	LIFTDOWN_PUMP_T				TIMER_Drive12Check
#define	BLINK_LED_T						TIMER_Drive7Check
#define	BLINK_BEEP_T					TIMER_Drive10Check
#define	SERIES_PARALLE_PUMP_T	TIMER_Drive9Check


#define	LIFTDOWN_PUMP_1				PropDriverCh1			/*PropPump*/
#define	LIFTDOWN_PUMP_2				PropDriverCh0			// to be update 

#define	PCU_SWICTH												SWI1_R
#define	TILT_SIWTCH												SWI2_R
#define	LOWERCONTROL_LIFT_SIWTCH					SWI3_R						
#define	PEDAL_SIWTCH											SWI4_R
#define	PIT_SWITCH												SWI5_R
#define	LOWERCONTROL_DOWN_SWITCH					SWI6_R				
#define	UP_LIMIT_SWITCH										SWI7_R
#define	DOWN_LIMIT_SWITCH									SWI8_R	



#define	ECU_POWERON_DELAY_TIME	500
#define	ECU_ANTI_PINCH_TIME 	2500
#define	PUMP_CLOSE_PERCENTAGE	0
#define	PUMP_OPEN_PERCENTAGE	80

#define	MOTOR_MAX_SPEED			3045
#define	MOTOR_MAX_SPEED_VALUE	4064

/*�����ر�ֵ*/
#define MOTOR_FDB_CLOSE_VALUE 30
#define	PUMP_FDB_CLOSE_VALUE 6
#define PROP_FDB_CLOSE_VALUE 100


#define	PCU_STATE			0
#define	TILT_STATE			1
#define	LIFT_STATE			2
#define	PEDAL_STATE			3
#define	PIT_STATE			4
#define	DOWN_STATE			5
#define	UP_LIMIT_STATE			6
#define	DOWN_LIMIT_STATE			7


#define	SLOW_KEY				0
#define	SPEAKER_KEY				1
#define	MOVE_KEY				2
#define	LIFT_KEY				3
#define	ENABLE_KEY				4
#define	LEFT_KEY				5
#define	RIGHT_KEY				6

#define	FORWARD_DISABLE_FLAG		(1 << 0)
#define	BACKWARD_DISABLE_FLAG		(1 << 1)
#define	LIFTUP_DISABLE_FLAG			(1 << 2)
#define	LIFTDOWN_DISABLE_FLAG		(1 << 3)
#define	TURNRIGHT_DISABLE_FLAG	(1 << 4)
#define	TURNLEFT_DISABLE_FLAG		(1 << 5)
#define PCU_NOACT_FLAG					(1 << 6)
#define NO_ACT_FLAG							(1 << 7)
#define LIFT_SPEED_FLAG					(1 << 8)

/*�½�������־λ*/
#define NO_DOWN_ACT							1
#define NORMAL_DOWN_ACT					2
#define ANTIPINCH_STATE					3
#define WAITNG_STATE						4

/**/
#define STATE_MODE (1<<0)




#define	SYSTEM_INIT_ERR								ErrCode1		/*ϵͳ��ʼ������*/
#define	SYSTEM_COMM_ERR								ErrCode2		/*ϵͳͨ�Ŵ���*/
#define	INVALID_OPT_SET_ERR						ErrCode3		/*��Чѡ�����ô���*/
#define	EERPOM_ERR										ErrCode4		/*�������ݴ���*/
#define	LI_BATTERY_LOSS_ERR						ErrCode5		/*﮵��ͨѶ��ʧ*/
#define	LIFT_BUTTON_ERR								ErrCode6		/*�ϵ�ʱ������������*/
#define	SLOW_BUTTON_ERR								ErrCode7		/*�ϵ�ʱ���ٰ�������*/
#define	MOVE_BUTTON_ERR								ErrCode8		/*�ϵ�ʱ���߰�������*/
#define	GPS_CONNECT_ERR								ErrCode9		/*GPS���Ӵ���*/
#define	MAIN_CONNECT_ERR							ErrCode10		/*���Ӵ�������*/

#define	UPDOWN_BUTTON_ERR							ErrCode12		/*����ʱ�����������½���ť�򿪴���*/
#define	BMS_BATTERY_TEMP_DIFF2_ERR		ErrCode13		/*BMS-����²����2*/
#define	BMS_BATTERY_TEMP_HIGH1_ERR		ErrCode14		/*BMS-����¶ȹ���1*/
#define	BMS_DISCHARGE_TEMP_HIGH2_ERR	ErrCode15		/*BMS-�ŵ��¶ȹ���2*/
#define	BMS_DISCHARGE_CUR_HIGH1_ERR		ErrCode16		/*BMS-�ŵ��������1*/
#define	BMS_DISCHARGE_CUR_HIGH2_ERR		ErrCode17		/*BMS-�ŵ��������2*/
#define	PIT_PROCETION_ERR							ErrCode18		/*�Ӷ���������*/
#define	BMS_TOTAL_VOL_LOW1_ERR				ErrCode19		/*BMS-�ܵ�ѹ����1*/
#define	BMS_TOTAL_VOL_LOW2_ERR				ErrCode20		/*BMS-�ܵ�ѹ����2*/
#define	BMS_SINGLE_VOL_LOW1_ERR				ErrCode21		/*BMS-�����ѹ����1*/
#define	BMS_SINGLE_VOL_LOW2_ERR				ErrCode22		/*BMS-�����ѹ����2*/



#define	LOW_VALVE2_ERR								ErrCode27		/*�½���2����*/


#define	BMS_BATTERY_VOL_DIFF_HIGH_ERR	ErrCode30		/*BMS-���ѹ�����*/
#define	PRESSURE_SENSOR_ERR						ErrCode31		/*ѹ������������*/
#define ANGLE_SENSOR_ERR							ErrCode32		/*�Ƕȴ���������*/
#define	BATTERY_TYPE_ERR							ErrCode33		/*������ʹ���*/

#define	WEIGHT_CALI_REVESER_ERR				ErrCode35		/*���ر궨��*/
#define	BATTERY_LOW_CAP1_ERR					ErrCode36		/*��ص�����һ������*/

#define	CALIBRATION_FAILURE_ERR				ErrCode38		/*δ�궨��ɻ�궨ʧ��*/
#define	COMMUNICATION_ERR							ErrCode39		/*ͨ�Ź���*/

#define	PLATFORM_LEVEL1_LOCK_ERR			ErrCode41		/*ƽ̨һ������*/
#define	PLAT_LEFT_BUTTON_ERR					ErrCode42		/*����ʱ��ƽ̨����ת��ť���´���*/
#define	PLAT_RIGHT_BUTTON_ERR					ErrCode43		/*����ʱ��ƽ̨����ת��ť���´���*/
#define	PLATFORM_LEVEL2_LOCK_ERR			ErrCode44		/*ƽ̨��������*/

#define	ENABLE_BUTTON_ERR							ErrCode46		/*����ʱ��ƽ̨�ֱ�ʹ�ܿ��ذ�ť���´���*/
#define	HANDLE_NOT_IN_MIDDLE_POSI_ERR	ErrCode47		/*����ʱ��ƽ̨�ֱ�������λ����*/

#define	FORWARD_VALVE_ERR							ErrCode52		/*ǰ��������*/
#define	BACKWARD_VALVE_ERR						ErrCode53		/*���˷�����*/
#define LIFT_UP_VALVE_ERR							ErrCode54		/*����������*/	
#define	LIFT_DOWN_VALVE_ERR						ErrCode55		/*�½�������*/
#define	TURN_RIGHT_VALVE_ERR					ErrCode56		/*��ת������*/
#define	TURN_LEFT_VALVE_ERR						ErrCode57		/*��ת������*/
#define BRAKE_VALVE_ERR								ErrCode58		/*ɲ��������*/
#define	PARALLEL_VALVE_ERR						ErrCode59		/*����������*/
#define	CONTROLLER_ERR								ErrCode60		/*����������*/
#define	CTRL_CURRENT_SENSOR_ERR				ErrCode61		/*��������������������*/
#define	CTRL_HARDWARE_ERR							ErrCode62		/*������Ӳ���𻵹���*/
#define	PUMP_DRIVE_OPEN_ERR						ErrCode63		/*�õ�����·����*/
#define	LEFT_DRIVE_OPEN_ERR						ErrCode64		/*�������·����*/
#define	CONTROL_VOL_5V_ERR						ErrCode65		/*���Ƶ�ѹ5V����*/
#define	LEFT_VALVE_OPEN_SHORT_ERR			ErrCode66		/*����ʱ����⵽��ת����·���·*/
#define	CONTROL_VOL_12V_ERR						ErrCode67		/*���Ƶ�ѹ12V����*/
#define	BAT_LOW_CAP2_ERR							ErrCode68		/*��ص͵�����������*/
#define	HIGH_ZERO_CURRENT_ERR					ErrCode69		/*����λ��������*/
#define	CTRL_BUS_VOL_HIGH_ERR					ErrCode70		/*������ĸ�ߵ�ѹ���߹���*/
#define	PRE_CHARGE_FAULT_ERR					ErrCode71		/*Ԥ�����*/
#define	CTRL_BUS_VOL_LOW_ERR					ErrCode72		/*������ĸ�ߵ�ѹ���͹���*/
#define	CTRL_TEMP_LOW_ERR							ErrCode73		/*���������¹���*/
#define	CTRL_TEMP_HIGH1_ERR						ErrCode74		/*����������һ������*/
#define	PUMP_MOTOR_TEMP1_ERR					ErrCode75		/*�õ���¶�һ������*/
#define	PUMP_MOTOR_ENCODER_ERR				ErrCode76		/*�õ������������*/
#define	MOTOR_ENCODE_ERR							ErrCode77		/*����������*/
#define	PUMP_MOTOR_OVER_CURRENT_ERR		ErrCode78		/*�õ�����������*/
#define	PUMP_MOTER_TEMP2_ERR					ErrCode79		/*�õ���¶ȶ�������*/
#define	OVER_80_PER_LOAD_ERR					ErrCode80		/*���� 80%���ر���*/
#define	CTRL_TEMP2_ERR								ErrCode81		/*�������¶ȶ�������*/
#define	RIGHT_BRAKE_ERR								ErrCode82		/*��ɲ������*/
#define	LEFT_BRAKE_ERR								ErrCode83		/*��ɲ������*/
#define	PUMP_MOTOR_STALL_ERR					ErrCode84		/*�õ����ת��ʧ��*/
#define	LEFT_DRIVE_MOTER_STALL_ERR		ErrCode85		/*��ǣ�������ת��ʧ��*/
#define	RIGTH_DRIVE_MOTOR_STALL_ERR		ErrCode86		/*��ǣ�������ת��ʧ��*/

#define	CTRL_DRIVE_LONG_RUN_ERR				ErrCode89		/*����������ʱ���������*/
#define	OVER_90_PER_LOAD_ERR					ErrCode90		/*���� 90%���ر���*/
#define	LEFT_MOTOR_OVER_CURRENT_ERR		ErrCode91		/*����������������*/
#define RIGTH_MOTOOR_OVER_CURRENT_ERR	ErrCode92		/*�ҵ��������������*/



#define	OVER_99_PER_LOAD_ERR					ErrCode99		/*���� 99%���ر���*/
#define	PLAT_OVERLOAD_ERR							ErrCode100		/*ƽ̨���ر���*/
#define	MACHINE_TILT_OVER_SAFETY_ERR	ErrCode101		/*������б������ȫ�޶�����*/

extern void vUserEcuInit(void);
extern void vUserEcuProc(void);


#endif //#ifndef _USER_COMM_H_
