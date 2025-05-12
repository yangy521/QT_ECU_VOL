/*******************************************************************************
* Filename: UserEcuProc.h	                                             	   *
* Description: �߼����ͷ�ļ���������Ӧ�Ļص�����	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/05/30    														   *
* Revision:	 														 		   *
*******************************************************************************/
#ifndef _USER_ECU_PROC_H_
#define _USER_ECU_PROC_H_


#if (USER_TYPE == USER_FORKLIFT_LIUGONG| USER_TYPE == USER_FORKLIFT_LIUGONG_TEST)
#include "PcuProc.h"
#include "MstSlvCom.h"
#include "DoPwm.h"
#include "PropDriver.h"
#include "LocalDi.h"
#include "NetTimer.h"
#include "Para.h"

/******************************************************************************
*��������
******************************************************************************/
//#define test
#define	USER_ECU_PERIOD		TIMER_PLC_PERIOD
#define PCU_DISPLAY_PERIOD 150

#define H9_INIT							0	
#define	H9_HIG16CHOOSE			1
#define H9_LOW16CHOOSE			2
#define H9_SAVE							3
#define H9_SAVEFAIL					4
#define H9_DOWNLIMIT				5
#define H9_UPLIMIT					6
#define H9_SAVE_SUCCESS			7
#define H9_VERIFY						8
#define H9_LOWLIMIT_CALI 		9
#define H9_LOWLIMITMOVE_EN 	10


#define H2_INITIAL 			0
#define H2_DISPLAY_NUM	1
#define H2_DISPLAY_KEY	2
#define H2_HEART_SET		3
#define H2_SAVED				4
#define H2_HEART_CHECK_FAIL 5

/*switch forward and series pump */

#define	LIFTUP_PUMP						DRIVER3
#define	TURNRIGHT_PUMP				DRIVER4
#define	TURNLEFT_PUMP					DRIVER5

#define	BLINK_LED							DRIVER7
#define	BACKWARD_PUMP					DRIVER8
#define HIGH_SPEED_PUMP				DRIVER9
#define	BLINK_BEEP						DRIVER10

#ifdef test
#define	SPEAKER_PUMP					DRIVER2
#define	FORWARD_PUMP					DRIVER6
#else
#define	SPEAKER_PUMP					DRIVER6
#define	FORWARD_PUMP					DRIVER2
#endif

#define	SPEAKER_PUMP_R			DRIVER6_R
#define	BACKWARD_PUMP_R			DRIVER8_R
#define	FORWARD_PUMP_R			DRIVER2_R
#define	TURNLEFT_PUMP_R			DRIVER5_R
#define	TURNRIGHT_PUMP_R		DRIVER4_R
#define	LIFTUP_PUMP_R					DRIVER3_R
#define	LIFTDOWN_PUMP_R				DRIVER12_R
#define	BLINK_LED_R						DRIVER7_R
#define	BLINK_BEEP_R					DRIVER10_R
#define	HIGH_SPEED_PUMP_R			DRIVER9_R


//����0507���ñ�����0���������
#define	LIFTDOWN_PUMP1	PropDriverCh0			/*PropPump*/
#define LIFTDOWN_PUMP2	PropDriverCh1

/*ģ����*/
#define ANGLE_SENSOR_CHANNEL		 AI_B_AI1_R  /**/
#define PRESSURE_SENSOR_CHANNEL  AI_B_AI2_R
#define PRESSURE_SENSOR_CHANNEL2 AI_B_AI3_R
//#define AI_B_AI3_R


#define	PCU_SWICTH			  SWI1_R
#define	TILT_SIWTCH			  SWI2_R
#define	UP_LIMIT_SWITCH		SWI7_R
#define	DOWN_LIMIT_SWITCH	SWI8_R	
#define	PIT_SWITCH			  SWI5_R

//��ȷ��
#define LOWER_CONTROLL_UP	SWI4_R
#define LOWER_CONTROLL_DOWN	SWI3_R

#define ANTICOLLISION_SWITCH	SWI6_R


#define	ECU_POWERON_DELAY_TIME	500
#define SWITCH_CHECK_TIME				2500
#define	ECU_ANTI_PINCH_TIME 		3000

#define ALARM_TIME							5000


#define	PUMP_CLOSE_PERCENTAGE	0
#define	PUMP_OPEN_PERCENTAGE	1

#define MOTOR_CLOESE_SPEED	10
#define	MOTOR_MIN_SPEED			200		/*����ת��*/
#define MOTOR_STEER_SPEED		600
#define MOTOR_CHANGE_SPEED  300		/**/
#define	MOTOR_MAX_SPEED			2000
#define	MOTOR_MAX_SPEED_VALUE	4064

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
#define	SPEAKER_KEY			1
#define	MOVE_KEY				2
#define	LIFT_KEY				3
#define	ENABLE_KEY			4
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
#define TIME_FACTOR					10 
#define FILTER_FACTOR				3 
#define	MOTOR_MAX_SPD				3000
#define	MOTOR_SWITCH_VALVE_SPD		300
#define	MOTOR_MIN_SPD_BY_CTRL		300

#define PressureCali_Times	20
#define CALIBRATION_INIT_TIME		3000
#define CALIBRATION_DELAY_TIME	3000
#define CALIBRATION_PROC_TIME		2000
#define MOTOR_SWITCH_



#define	SYSTEM_INIT_ERR							ErrCode101		/*ϵͳ��ʼ������*/
#define	SYSTEM_COMM_ERR							ErrCode102		/*ϵͳͨ�Ŵ���*/
#define	INVALID_OPT_SET_ERR					ErrCode103		/*��Чѡ�����ô���*/
//#define	EERPOM_ERR							ErrCode104		/*�������ݴ���*/

#define	LIFT_BUTTON_ERR							ErrCode106		/*�ϵ�ʱ������������*/
#define	SLOW_BUTTON_ERR							ErrCode107		/*�ϵ�ʱ���ٰ�������*/
#define	MOVE_BUTTON_ERR							ErrCode108		/*�ϵ�ʱ���߰�������*/

#define PLATE_LOCK_LEVEL1_ALARMING	ErrCode109		/*ƽ̨һ������Ԥ��*/
#define PLATE_LOCK_LEVEL2_ALARMING	ErrCode110		/*ƽ̨��������Ԥ��*/
#define HEART_LOCK_ALARMING					ErrCode111		/*ƽ̨����������Ԥ��*/

//#define	GPS_CONNECT_ERR							ErrCode109		/*GPS���Ӵ���*/
//#define	MAIN_CONNECT_ERR						ErrCode110		/*���Ӵ�������*/

#define	UPDOWN_BUTTON_ERR							ErrCode112		/*����ʱ�����������½���ť�򿪴���*/
#define	PIT_PROCETION_ERR							ErrCode118		/*�Ӷ���������*/

#define BMS_OFFLINE										ErrCode120		/*BMSͨѶ����*/					/*��������*/
#define	BMS_BATTERY_TEMP_HIGH1_ERR		ErrCode121		/*BMS-����¶ȹ���1*/
#define	BMS_DISCHARGE_CUR_HIGH1_ERR		ErrCode122		/*BMS-�ŵ��������1*/
#define	BMS_TOTAL_VOL_LOW1_ERR				ErrCode123		/*BMS-�ܵ�ѹ����1*/
#define	BMS_SINGLE_VOL_LOW1_ERR				ErrCode124		/*BMS-�����ѹ����1*/
#define	BMS_SINGLE_VOL_LOW2_ERR				ErrCode125		/*BMS-�����ѹ����2*/
#define	BMS_BATTERY_VOL_DIFF_HIGH_ERR	ErrCode136		/*BMS-���ѹ�����*/


#define	BMS_BATTERY_TEMP_DIFF2_ERR		ErrCode127		/*BMS-����²����2*/
#define	BMS_DISCHARGE_CUR_HIGH2_ERR		ErrCode128		/*BMS-�ŵ��������2*/
#define	BMS_DISCHARGE_TEMP_HIGH2_ERR	ErrCode129		/*BMS-�ŵ��¶ȹ���2*/




#define	PRESSURE_SENSOR_ERR						ErrCode131		/*ѹ������������*/
#define ANGLE_SENSOR_ERR							ErrCode132		/*�Ƕȴ���������*/

#define	PCU_BUTT_ERR									ErrCode133		/*PCU��������*/	/*�������ϣ�Э�飿*/

#define	WEIGHT_CALI_REVESER_ERR			ErrCode135		/*���ر궨��*/
#define	BATTERY_LOW_CAP1_ERR			ErrCode136		/*��ص�����һ������*/

#define	CALIBRATION_FAILURE_ERR			ErrCode138		/*δ�궨��ɻ�궨ʧ��*/

#define	PLATFORM_LEVEL1_LOCK_ERR		ErrCode141		/*ƽ̨һ������*/

#define	PLAT_LEFT_BUTTON_ERR			ErrCode142		/*����ʱ��ƽ̨����ת��ť���´���*/
#define	PLAT_RIGHT_BUTTON_ERR			ErrCode143		/*����ʱ��ƽ̨����ת��ť���´���*/
#define	PLATFORM_LEVEL2_LOCK_ERR		ErrCode145		/*ƽ̨��������*/

#define	ENABLE_BUTTON_ERR				ErrCode146		/*����ʱ��ƽ̨�ֱ�ʹ�ܿ��ذ�ť���´���*/
#define	HANDLE_NOT_IN_MIDDLE_POSI_ERR	ErrCode147		/*����ʱ��ƽ̨�ֱ�������λ����*/
#define	HEART_BEAT_LOCK_ERR				ErrCode149				/*��������*/

#define	FORWARD_VALVE_ERR					ErrCode152		/*ǰ��������*/
#define	BACKWARD_VALVE_ERR				ErrCode153		/*���˷�����*/
#define LIFT_UP_VALVE_ERR					ErrCode154		/*����������*/	
#define	LIFT_DOWN_VALVE_ERR				ErrCode155		/*�½�������*/
#define	TURN_RIGHT_VALVE_ERR			ErrCode156		/*��ת������*/
#define	TURN_LEFT_VALVE_ERR				ErrCode157		/*��ת������*/
#define	PARALLEL_VALVE_ERR				ErrCode159		/*����������*/


//#define	CONTROLLER_ERR					ErrCode160		/*����������*/
//#define	CTRL_CURRENT_SENSOR_ERR			ErrCode161		/*��������������������*/
//#define	CTRL_HARDWARE_ERR				ErrCode162		/*������Ӳ���𻵹���*/
//#define	PUMP_DRIVE_OPEN_ERR				ErrCode163		/*�õ�����·����*/
//#define	LEFT_DRIVE_OPEN_ERR				ErrCode164		/*�������·����*/
//#define	CONTROL_VOL_5V_ERR				ErrCode165		/*���Ƶ�ѹ5V����*/
//#define	LEFT_VALVE_OPEN_SHORT_ERR		ErrCode166		/*����ʱ����⵽��ת����·���·*/
//#define	CONTROL_VOL_12V_ERR				ErrCode167		/*���Ƶ�ѹ12V����*/
#define	BAT_LOW_CAP2_ERR				ErrCode168		/*��ص͵�����������*/
//#define	HIGH_ZERO_CURRENT_ERR			ErrCode169		/*����λ��������*/
//#define	CTRL_BUS_VOL_HIGH_ERR			ErrCode170		/*������ĸ�ߵ�ѹ���߹���*/
//#define	PRE_CHARGE_FAULT_ERR			ErrCode171		/*Ԥ�����*/
//#define	CTRL_BUS_VOL_LOW_ERR			ErrCode172		/*������ĸ�ߵ�ѹ���͹���*/
//#define	CTRL_TEMP_LOW_ERR				ErrCode173		/*���������¹���*/
//#define	CTRL_TEMP_HIGH1_ERR				ErrCode174		/*����������һ������*/
//#define	PUMP_MOTOR_TEMP1_ERR			ErrCode175		/*�õ���¶�һ������*/
//#define	PUMP_MOTOR_ENCODER_ERR			ErrCode176		/*�õ������������*/
//#define	MOTOR_ENCODE_ERR				ErrCode177		/*����������*/
//#define	PUMP_MOTOR_OVER_CURRENT_ERR		ErrCode178		/*�õ�����������*/
//#define	PUMP_MOTER_TEMP2_ERR			ErrCode179		/*�õ���¶ȶ�������*/
#define	OVER_80_PER_LOAD_ERR			ErrCode180		/*���� 80%���ر���*/

//#define	CTRL_TEMP2_ERR					ErrCode181		/*�������¶ȶ�������*/

//#define	RIGHT_BRAKE_ERR					ErrCode182		/*��ɲ������*/
//#define	LEFT_BRAKE_ERR						ErrCode183		/*��ɲ������*/
//#define	PUMP_MOTOR_STALL_ERR			ErrCode184		/*�õ����ת��ʧ��*/
#define	LEFT_DRIVE_MOTER_STALL_ERR		ErrCode185		/*��ǣ�������ת��ʧ��*/
//#define	RIGTH_DRIVE_MOTOR_STALL_ERR		ErrCode186		/*��ǣ�������ת��ʧ��*/

//#define	CTRL_DRIVE_LONG_RUN_ERR			ErrCode189		/*����������ʱ���������*/
#define	OVER_90_PER_LOAD_ERR			ErrCode190		/*���� 90%���ر���*/
//#define	LEFT_MOTOR_OVER_CURRENT_ERR		ErrCode191		/*����������������*/
//#define RIGTH_MOTOOR_OVER_CURRENT_ERR	ErrCode192		/*�ҵ��������������*/

#define	OVER_99_PER_LOAD_ERR			ErrCode199		/*���� 99%���ر���*/
#define	PLAT_OVERLOAD_ERR				ErrCode200		/*ƽ̨���ر���*/
#define	MACHINE_TILT_OVER_SAFETY_ERR	ErrCode201		/*������б������ȫ�޶�����*/
#define MACHINE_ANTIPUNCH_ERR				ErrCode202		/*��ֹ��ײ����*/
//#define PCU_OUT_ERROR_O1		ErrCode203		/*�ֱ�оһ������ӿڶϿ�*/
//#define PCU_OUT_ERROR_O2		ErrCode204		/*�ֱ�оһ������ӿڶϿ�*/
//#define PCU_OUT_ERROR_LEVEL2		ErrCode205		/*�ֱ�о��������ӿڶϿ�*/
//#define HEARTBEAT_ALARMING			ErrCode206		/*�յ���������*/
//#define PCU_CONFLICT_KEY_ERR			ErrCode207		/*��ͻ��������*/


#define	TIMER_SwitchCheck	TIMER_USER_1
#define TIMER_Calibration	TIMER_USER_2
#define	TIMER_AlarmDelay	TIMER_USER_3
//#define TIMER_PCUSLEEP		TIMER_USER_4
#define Timer_OneSecond			TIMER_USER_4

#define PARA_USERSETS					PARA_ValveType

#define UPDOWNCOUNT_ADDR      PARA_AngleValue0 //���������½�����ֵ����
#define CARCODE_HISTORY				PARA_AngleValue1 //��ʷ��������

#define	PARA_LOWLIMIT_RANGE		PARA_AngleValue2
#define PARA_LOWLIMIT_ANGLE		PARA_AngleValue3
#define PARA_CARCODE					PARA_AngleValue4
#define PARA_HANDLEMAX				PARA_AngleValue5
#define PARA_HANDLEMID				PARA_AngleValue6
#define PARA_HANDLEMIN				PARA_AngleValue7

#define	LIFTTIMES_ADDR_L16				PARA_USER_DATA_01
#define LIFTTIMES_ADDR_H16				PARA_USER_DATA_02
#define STEERTIMES_ADDR_L16				PARA_USER_DATA_03
#define STEERTIMES_ADDR_H16				PARA_USER_DATA_04
#define MOVETIES_ADDR_L16					PARA_USER_DATA_05
#define MOVETIES_ADDR_H16					PARA_USER_DATA_06
#define OVERLOADTIMES_ADDR_L16		PARA_USER_DATA_07
#define OVERLOADTIMES_ADDR_H16		PARA_USER_DATA_08

#define LIFTHOUR_ADDR_L16					PARA_USER_DATA_09
#define LIFTHOUR_ADDR_H16					PARA_USER_DATA_10
#define STEERHOUR_ADDR_L16				PARA_USER_DATA_11
#define	STEERHOUR_ADDR_H16				PARA_USER_DATA_12
#define	LOWERMOVEHOUR_ADDR_L16		PARA_USER_DATA_13
#define LOWERMOVEHOUR_ADDR_H16		PARA_USER_DATA_14
#define UPMOVEHOUR_ADDR_L16				PARA_USER_DATA_15
#define UPMOVEHOUR_ADDR_H16				PARA_USER_DATA_16
#define DOWNHOUR_ADDR_L16					PARA_USER_DATA_17
#define DOWNHOUR_ADDR_H16					PARA_USER_DATA_18
#define TEMP_UNLOCK_HOUR_ADDR_16	PARA_USER_DATA_19
#define UPDATE_COUNT_ADDR_L16			PARA_USER_DATA_20
#define RANDOM_SEED_ADDR					PARA_USER_DATA_21
#define REMOTE_LOCK_STATE					PARA_USER_DATA_22
#define SOFTWARE_ADDR							PARA_USER_DATA_24


#define SOFTWARE_VERSION_STATE 		PARA_DefaultFlag

extern void vUserEcuInit(void);
extern void vUserEcuProc(void);

/*��Ʒ���к�*/
//#define ECU_SERIALNUMBER_L32 "ABCDEFGH24010501"

#define ECU_SERIALNUMBER_L32 "QTECULTS24030601"

#define PCU_SERIALNUMBER_L32 "LSDPCUL24030601"


#define SOFTWARE_VERSION		"LSQTCNAA24052200"//��ʽ��Ϊ00

#define VERSION_CODE_HMI        240522   //�Ǳ���ʾ����汾ʱ��,ͬ������

#define SOFTWARE_VERSION_CODE		0x132  //@@@@@��һ�μ�һ
//0x123 Ϊ�ı�TBOX��������ʱ��Ϊ3min
//0x124 ��ǿ�����Ӷ����ع���
//0x125 �޸��µ��ﳵ��ת�����ǰ��ͬʱ��������
//0x126 ����������һ�������໥���ǡ�
//0x127 �޸�PSģʽ�µ����������ٶȺ���ͷ�Χ�Ĳ����߼���ECU�����Ĳ�������֤
//0x128 ������޸ĳ���ʱ�����޸�MCU������������֤������ɹ���39�ž���
//0x129 �޸��˲���BUG���Ż���H9ģʽ�±��������
//0x130 H9ģʽ�µ�16λ���ֽ�Bit0��ΪѶ����ͬ����5M�����Ƕ�ģ��̻��������޸�Ϊ490
//0x131�޸�����ˢ����e���洢�ĳ���Ĭ��Ϊ0������������ж�Ϊ0ֱ������ΪĬ��5m�ĳ��ͣ�
//0x132 �޸�MCU��Ϊ�ϰ汾MCU����ECU�޸�MCU����1S��û��ͨѶ��ֱ���ж�����ʧ��
#define RELEASE_MONTH  1


typedef union
{
	uint16_t u16Data;
	struct
	{
		uint16_t b1StartFast:1;
		uint16_t b1AntipinchSlow:1;
		uint16_t b1Err18LiftAllow:1;
		uint16_t b1LiftAllowBeforeCali:1;
		uint16_t b1LiftBanMove:1;
		uint16_t b1StartWithMode:1;
		uint16_t b1LowLimitEn:1;
		uint16_t b1UpLimitEn:1;
		uint16_t b1HighSpeedPump:1;
		uint16_t b1SleepEn:1;
		uint16_t b1DoubleRangeHeight:1;
		uint16_t b1LowLimitBanMoveEn:1;
	};
}xUserSets;

xUserSets sgUserSets;

typedef struct
{
	int8_t i8MiddleValue;
	int8_t i8PositiveValue;
	int8_t i8NegativeValue;
}xPcuHandle;

xPcuHandle sgPcuHandle;

static uint8_t u8RemoteParaFlag;
#define REMOTE_INIT							0//�ȴ����
#define REMOTE_WAITHANDE				1//���ɹ����ȴ�����
#define REMOTE_HANDSHAKE				2//���ֳɹ�������������״̬
//#define REMOTE_CHECKFAIL				3//״̬���ʧ��
//#define REMOTE_WAITUSER					4//״̬���ɹ����ȴ�ȷ�ϻ�ȡ��
#define REMOTE_WAITEDATA				5//�ȴ��޸Ĳ���
#define REMOTE_CHANGE_SUCCESS		6//�����޸ĳɹ�
#define	REMOTE_CHANGE_FAIL			7//�����޸�ʧ��
#define REMOTE_OTA_REQ					8//Զ����������
#define REMOTE_OTA_START		9//�ȴ��û�ȷ��

static uint8_t u8RunMode = 0;
#define	NORMAL_MODE								0
#define PARA_SETS_MODE						1
#define	PRESSURE_CALIBRATION_MODE	2
#define NO_ACTION_MODE 						3
#define AUTO_LIFT_MODE            4
	

	
static uint8_t u8ParaSetMode = 0;
#define PARASETS_DISABLE	0
#define COMMON_PARASET		1
#define SPEED_PARASET			2
#define MACHINE_PARASET		3
#define	FUNCTION_PARASET	4
#define H8_MODE						5
#define H2_MODE						6
#define H0_MODE						7
#define H1_MODE						8
#define TBOX_PARA_SETS		9
#define TBOX_UPDATEAPP		10	

#if 1

#define SECOND_LEVEL_BOOT_ADDR                		0x08008000
#define	USER_FLAG_ADDR														0x08005800
#define USER_VALID																0x12345678

#define	UDS_UDDATE_FALGE_ADDR                     0x08006800
#define UDS_UPDATE_FLAGE                          0x1A1A1A1A

//#endif

#else  //�Ķ����boot������boot

#define SECOND_LEVEL_BOOT_ADDR                		0x08004000
#define	USER_FLAG_ADDR														0x0800F000
#define USER_VALID																0x55555555

#define	UDS_UDDATE_FALGE_ADDR                     0x0800F800
#define UDS_UPDATE_FLAGE                          0xCCCCCCCC

#endif 

static uint8_t u8PumpFlag;

#endif //#if (USER_TYPE == USER_FORKLIFT_LIUGONG)
#endif //#ifndef _USER_COMM_H_
 
