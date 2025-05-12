/*******************************************************************************
* Filename: UserEcuProc.h	                                             	   *
* Description: �߼����ͷ�ļ���������Ӧ�Ļص�����	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/05/30    														   *
* Revision:	 														 		   *
*******************************************************************************/
#ifndef _USER_FORKLIFT_XUGONG_H_
#define _USER_FORKLIFT_XUGONG_H_

#include "PcuProc.h"
#include "MstSlvCom.h"
#include "DoPwm.h"
#include "PropDriver.h"
#include "LocalDi.h"
#include "NetTimer.h"
#include "Para.h"

#include "CanBlock.h"

#if(USER_TYPE == USER_FORKLIFT_XUGONG)
/******************************************************************************
*��������
******************************************************************************/
//#define test 

#define	USER_ECU_PERIOD	TIMER_PLC_PERIOD

#define H9_INIT						0	
#define	H9_HIG16CHOOSE		1
#define H9_LOW16CHOOSE		2
#define H9_SAVE						3
#define H9_SAVEFAIL				4
#define H9_UPLIMIT_SAVE		5
#define H9_DOWNLIMIT_SAVE	6

/*switch forward and series pump */

#define	FORWARD_PUMP					DRIVER2
#define	LIFTUP_PUMP						DRIVER3
#define	TURNRIGHT_PUMP				DRIVER4
#define	TURNLEFT_PUMP					DRIVER5
#define	BLINK_LED							DRIVER7
#define	BACKWARD_PUMP					DRIVER8
#define HIGH_SLOW_SPEED_PUMP				DRIVER9
#define	BLINK_BEEP						DRIVER10
//TMP
//#define	BLINK_BEEP						DRIVER6
#define	SPEAKER_PUMP					DRIVER6



#define	FORWARD_PUMP_R			DRIVER2_R
#define	LIFTUP_PUMP_R			DRIVER3_R
#define	TURNRIGHT_PUMP_R		DRIVER4_R
#define	TURNLEFT_PUMP_R			DRIVER5_R
#define	SPEAKER_PUMP_R			DRIVER6_R
#define	BLINK_LED_R				DRIVER7_R
#define	BACKWARD_PUMP_R			DRIVER8_R
#define	HIGH_SLOW_SPEED_PUMP_R	DRIVER9_R
#define	BLINK_BEEP_R			DRIVER10_R





#define	LIFTDOWN_PUMP1	PropDriverCh0			/*PropPump*/
#define LIFTDOWN_PUMP2	PropDriverCh1

/*ģ����*/
#define ANGLE_SENSOR_CHANNEL		AI_B_AI1_R  /**/
#define PRESSURE_SENSOR_CHANNEL1 AI_B_AI2_R
#define PRESSURE_SENSOR_CHANNEL2	AI_B_AI3_R
//#define AI_B_AI3_R


#define	PCU_SWICTH			SWI1_R
#define	TILT_SIWTCH			SWI2_R
#define	PIT_SWITCH			SWI5_R

#define	UP_LIMIT_SWITCH		SWI7_R
#define	DOWN_LIMIT_SWITCH	SWI8_R	

//#define LOWER_CONTROLL_UP	SWI4_R
//#define LOWER_CONTROLL_DOWN	SWI3_R



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

#define PressureCali_Times	20
#define CALIBRATION_INIT_TIME		3000
#define CALIBRATION_DELAY_TIME	3000
#define CALIBRATION_PROC_TIME		2000
#define MOTOR_SWITCH_

/*������*/
/*����Ƿ��Ӧ�칤����*/
/*��¼һ����ʱû���Ĺ���*/
#define	SYSTEM_INIT_ERR					ErrCode101		/*ϵͳ��ʼ������*/
#define	SYSTEM_COMM_ERR					ErrCode102		/*ϵͳͨ�Ŵ���*/
#define	INVALID_OPT_SET_ERR			ErrCode103		/*��Чѡ�����ô���*/
#define	EERPOM_ERR							ErrCode104		/*�������ݴ���*/
#define	LI_BATTERY_LOSS_ERR			ErrCode105		/*﮵��ͨѶ��ʧ*/
#define	LIFT_BUTTON_ERR					ErrCode106		/*�ϵ�ʱ������������*/
#define	SLOW_BUTTON_ERR					ErrCode107		/*�ϵ�ʱ���ٰ�������*/
#define	MOVE_BUTTON_ERR					ErrCode108		/*�ϵ�ʱ���߰�������*/
#define	GPS_CONNECT_ERR					ErrCode109		/*GPS���Ӵ���*/
#define	MAIN_CONNECT_ERR				ErrCode110		/*���Ӵ�������*/

#define	UPDOWN_BUTTON_ERR							ErrCode112		/*����ʱ�����������½���ť�򿪴���*/
#define	BMS_BATTERY_TEMP_DIFF2_ERR		ErrCode113		/*BMS-����²����2*/
#define	BMS_BATTERY_TEMP_HIGH1_ERR		ErrCode114		/*BMS-����¶ȹ���1*/
#define	BMS_DISCHARGE_TEMP_HIGH2_ERR	ErrCode115		/*BMS-�ŵ��¶ȹ���2*/
#define	BMS_DISCHARGE_CUR_HIGH1_ERR		ErrCode116		/*BMS-�ŵ��������1*/
#define	BMS_DISCHARGE_CUR_HIGH2_ERR		ErrCode117		/*BMS-�ŵ��������2*/
#define	PIT_PROCETION_ERR							ErrCode118		/*�Ӷ���������*/
#define	BMS_TOTAL_VOL_LOW1_ERR				ErrCode119		/*BMS-�ܵ�ѹ����1*/
#define BMS_OFFLINE										ErrCode120		/*BMSͨѶ����*/					/*��������*/
//#define	BMS_TOTAL_VOL_LOW2_ERR				ErrCode120		/*BMS-�ܵ�ѹ����2*/			/*��������*/
#define	BMS_SINGLE_VOL_LOW1_ERR				ErrCode121		/*BMS-�����ѹ����1*/
#define	BMS_SINGLE_VOL_LOW2_ERR				ErrCode122		/*BMS-�����ѹ����2*/

#define	LOW_VALVE2_ERR								ErrCode127		/*�½���2����*/

#define	BMS_BATTERY_VOL_DIFF_HIGH_ERR	ErrCode130		/*BMS-���ѹ�����*/
#define	PRESSURE_SENSOR_ERR						ErrCode131		/*ѹ������������*/
#define ANGLE_SENSOR_ERR							ErrCode132		/*�Ƕȴ���������*/
#define	BATTERY_TYPE_ERR							ErrCode133		/*������ʹ���*/

#define	WEIGHT_CALI_REVESER_ERR			ErrCode135		/*���ر궨��*/
#define	BATTERY_LOW_CAP1_ERR			ErrCode136		/*��ص�����һ������*/

#define	CALIBRATION_FAILURE_ERR			ErrCode138		/*δ�궨��ɻ�궨ʧ��*/
#define	COMMUNICATION_ERR				ErrCode139		/*ͨ�Ź���*/

#define	PLATFORM_LEVEL1_LOCK_ERR		ErrCode141		/*ƽ̨һ������*/
#define	PLAT_LEFT_BUTTON_ERR			ErrCode142		/*����ʱ��ƽ̨����ת��ť���´���*/
#define	PLAT_RIGHT_BUTTON_ERR			ErrCode143		/*����ʱ��ƽ̨����ת��ť���´���*/
#define	PLATFORM_LEVEL2_LOCK_ERR		ErrCode144		/*ƽ̨��������*/

#define	ENABLE_BUTTON_ERR						ErrCode146		/*����ʱ��ƽ̨�ֱ�ʹ�ܿ��ذ�ť���´���*/
#define	HANDLE_NOT_IN_MIDDLE_POSI_ERR	ErrCode147		/*����ʱ��ƽ̨�ֱ�������λ����*/

#define	FORWARD_VALVE_ERR					ErrCode152		/*ǰ��������*/
#define	BACKWARD_VALVE_ERR				ErrCode153		/*���˷�����*/
#define LIFT_UP_VALVE_ERR					ErrCode154		/*����������*/	
#define	LIFT_DOWN_VALVE_ERR				ErrCode155		/*�½�������*/
#define	TURN_RIGHT_VALVE_ERR			ErrCode156		/*��ת������*/
#define	TURN_LEFT_VALVE_ERR				ErrCode157		/*��ת������*/
#define BLINK_LED_VALV_ERR				ErrCode158		/*LED����ƴ���*/
#define BLIKN_BEEP_VALVE_ERR			ErrCode159		/*����������*/

#if 0
#define BRAKE_VALVE_ERR					ErrCode158		/*ɲ��������*/
#define	PARALLEL_VALVE_ERR				ErrCode159		/*����������*/
#endif

#define	CONTROLLER_ERR					ErrCode160		/*����������*/
#define	CTRL_CURRENT_SENSOR_ERR			ErrCode161		/*��������������������*/
#define	CTRL_HARDWARE_ERR				ErrCode162		/*������Ӳ���𻵹���*/
#define	PUMP_DRIVE_OPEN_ERR				ErrCode163		/*�õ�����·����*/
#define	LEFT_DRIVE_OPEN_ERR				ErrCode164		/*�������·����*/
#define	CONTROL_VOL_5V_ERR				ErrCode165		/*���Ƶ�ѹ5V����*/
#define	LEFT_VALVE_OPEN_SHORT_ERR		ErrCode166		/*����ʱ����⵽��ת����·���·*/
#define	CONTROL_VOL_12V_ERR				ErrCode167		/*���Ƶ�ѹ12V����*/
#define	BAT_LOW_CAP2_ERR				ErrCode168		/*��ص͵�����������*/
#define	HIGH_ZERO_CURRENT_ERR			ErrCode169		/*����λ��������*/
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
#define	LEFT_BRAKE_ERR						ErrCode183		/*��ɲ������*/
#define	PUMP_MOTOR_STALL_ERR			ErrCode184		/*�õ����ת��ʧ��*/
#define	LEFT_DRIVE_MOTER_STALL_ERR		ErrCode185		/*��ǣ�������ת��ʧ��*/
#define	RIGTH_DRIVE_MOTOR_STALL_ERR		ErrCode186		/*��ǣ�������ת��ʧ��*/

#define	CTRL_DRIVE_LONG_RUN_ERR			ErrCode189		/*����������ʱ���������*/
#define	OVER_90_PER_LOAD_ERR			ErrCode190		/*���� 90%���ر���*/
#define	LEFT_MOTOR_OVER_CURRENT_ERR		ErrCode191		/*����������������*/
#define RIGTH_MOTOOR_OVER_CURRENT_ERR	ErrCode192		/*�ҵ��������������*/

#define	OVER_99_PER_LOAD_ERR					ErrCode199		/*����99%���ر���*/
#define	PLAT_OVERLOAD_ERR							ErrCode200		/*ƽ̨���ر���*/
#define	MACHINE_TILT_OVER_SAFETY_ERR	ErrCode201		/*������б������ȫ�޶�����*/
#define	ANTICOLLISION_PROTECT_ERR			ErrCode202		/*��ײ��������,��ʾ39*/


#define	TIMER_SwitchCheck	TIMER_USER_1
#define TIMER_Calibration	TIMER_USER_2
#define	TIMER_AlarmDelay	TIMER_USER_3


#define RANDOM_SEED_ADDR					PARA_USER_DATA_21
#define TMP_UNLOCK_TIME_ADDR 							PARA_USER_DATA_01
#define UNLOCK_CNT_TIME				1000
#define UNLOCK_TIME				    12*60*60  //��ʱ����ʱ��12h
			

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



#define PARA_USERSETS					PARA_ValveType

typedef union
{
	uint16_t u16Data;
	struct
	{
		uint16_t b1StartFast:1;       	 //�õ����
		uint16_t b1AntipinchSlow:1;   	 //���������ּ����½�
		uint16_t b1Err18LiftAllow:1;  	 //18(�Ӷ�������������)
		uint16_t b1LiftAllowBeforeCali:1;//�궨ǰ��������
		uint16_t b1LiftBanMove:1;        //�������ֹ����
		uint16_t b1StartWithMode:1;      //�ϵ��Ĭ������ģʽ
		uint16_t b1LowLimitEn:1;         //����λʹ��
		uint16_t b1UpLimitEn:1;          //����λʹ��
		uint16_t b1HighSpeedPump:1;      //�ߵ��ٷ�
		uint16_t b1SleepEn:1;            //˯�߹���ʹ��
		uint16_t b1DoubleRangeHeight:1;  //˫�߶�
		uint16_t b1LowLimitBanMoveEn:1;  //�����λ��ֹ����
		};
}xUserSets;

xUserSets sgUserSets;



//
#define ADDR_INVALID		999

#define SECOND_LEVEL_BOOT_PARA_ADDR                0x08000000
#define	UDS_UDDATE_FALGE_ADDR                        0x0800F800
#define UDS_UPDATE_FLAGE                             0xAA11BB22


extern void vUserEcuInit(void);
extern void vUserEcuProc(void);


typedef union
{
	uint8_t u8Data[72];
	struct
	{
		uint8_t u8TerminalEncode[10];	//�ն˱���
		uint8_t u8ECUEncode[10];		//ecu���к�
		uint8_t u8PCUEncode[10];		//�ֱ�����
		uint8_t u8SystemTime[6];		//ϵͳʱ��
		uint8_t u8ManufacturerCode[6];	//���ұ���
		uint8_t u8VehicleTypeEncode[6];	//���ͱ���
		uint8_t u8VehicleStatus;		//����״̬
		uint8_t u8RemainingDaysUnlocked;//����ʣ������
		uint8_t u8MagicCode[2];			//Magiccode
		uint8_t u8SoftwareVersion[2];	//����汾
		uint8_t u8Markers;				//���
		uint8_t u8ReleaseDate[3];		//�汾����
		uint8_t u8author[6];			//����
		uint8_t u8reserve[6];			//����
		uint8_t u8VerificationCode[2];//У����
	};
}xSystemMessage;


typedef union
{
	uint8_t u8Data[55];
	struct
	{
		uint8_t u8TerminalEncode[10];		//�ն˱���
		uint8_t u8ECUEncode[10];			//ECU����
		uint8_t u8SystemTime[8];			//ϵͳʱ��
		uint8_t u8remainingTime[2];			//�˿�������ʣ��ʱ��
		uint8_t u8controllerTag;			//���������
		uint8_t u8PowerVoltage[2];			//���ʵ�ѹ
		uint8_t u16SystemVotage[2];			//ϵͳ��ѹ
		uint8_t u8MachineStatusoutput[2];	//��������״̬���
		uint8_t u8MachineStatusinput[2];	//��������״̬����
		uint8_t u8AngleSensorVolt[2];		//�Ƕȴ�����
		uint8_t u8CylinderPressure[2];		//�͸�ѹ��AD
		uint8_t u8WeightPercentage;			//�����ٷֱ�
		uint8_t u8HeightPercentage;			//�߶Ȱٷֱ�
		uint8_t u8PropOutputPercentage;		//����������ٷֱ�
		uint8_t u8PcuHandleValue;			//�ֱ�ADֵ
		uint8_t u8PCUDI[2];					//PCUDI
		uint8_t reverse1[2];				//Ԥ��1
		uint8_t reverse2[2];				//Ԥ��2
		uint8_t u16VerificationCode[2];	//У����
	};
}xSystemPortData;

typedef union
{
	uint8_t u8Data[80];
	struct
	{
		uint8_t u8TerminalEncode[10];		//�ն˱���
		uint8_t u8ECUEncode[10];			//ecu���к�
		uint8_t u8PCUEncode[10];			//�ֱ�����
		uint8_t u8SystemTime[6];			//ϵͳʱ��
		uint8_t u8MachineStatusoutput[2];	//��������״̬���
		uint8_t u8MachineStatusinput[2];	//��������״̬����
		uint8_t u8BatteryVoltage[8];          //��ص�ѹ
		uint8_t u8operatingConditionCode;	//��������
		uint8_t u8AngleSensorVolt[2];		//�Ƕȴ�����
		uint8_t u8CylinderPressure[2];		//�͸�ѹ��AD
		uint8_t u8WeightPercentage;			//�����ٷֱ�
		uint8_t u8HeightPercentage;			//�߶Ȱٷֱ�
		uint8_t u8PropOutputPercentage;		//����������ٷֱ�
		uint8_t u8calibrationStatus;		//�궨״̬
		uint8_t u32ECUPowerOnTime[4];		//ECU�ϵ�ʱ��
		uint8_t u32LiftUpTime[4];			//����ʱ��
		uint8_t u32LiftDownTime[4];		//�½�ʱ��
		uint8_t u32steeringTime[4];		//ת��ʱ��
		uint8_t u8BatterySOC;				//��ص���
		uint8_t u8machineCode[2];			//��������
		uint8_t u8FastDriveSpeed;			//�����������ٶȰٷֱ�
		uint8_t u8SlowDriveSpeed;			//����������߰ٷֱ�
		uint8_t u8LiftSpeed;				//�������ʱ��
		uint8_t	u8DriveSpeedAfterLift;		//���������ٶ�
		uint8_t u8overloadAlarm;			//���ظ澯
		uint8_t u8alarmCode;				//��������
		uint8_t u16VerificationCode[2];	//У����
	};
}xSystemOperatingData;



typedef union
{
	uint8_t u8Data[90];
	struct
	{
		uint8_t u8TerminalEncode[10];	 		//�ն˱���
		uint8_t u8ECUEncode[10];		 		//ecu���к�
		uint8_t u8PCUEncode[10];				//�ֱ�����
		uint8_t u8SystemTime[8];		 		//ϵͳʱ��
		uint8_t u8PivotPercentage;	 		//ԭ��ת��ٷֱ�
		uint8_t u8ChassLiftSpeed;		 		//���̾����ٶ�
		uint8_t u8PlatformMaxLiftSpeed;		//ƽ̨��������ٶ�
		uint8_t u8MaxSpeedHandleValue; 		//������������ٶ��ֱ�ֵ
		uint8_t u8HighSpeedAccelerationValue;	//�������߼���ֵ
		uint8_t u8HighSpeeddecelerationValue;	//�������߼���ֵ
		uint8_t u8HighSpeedSlopePeriod;		//�������߼Ӽ����¶�ʱ��
		uint8_t u8maxSpeedHandleValueAfterLift;//��������������ٶ��ֱ�ֵ
		uint8_t u8AccSpeedAfterLift;   		//���������߼��ٶ�ֵ
		uint8_t u8decelerationValueAfterLift;	//���������߼���ֵ
		uint8_t u8SlopePeriodAfterLift;		//���������߼Ӽ����¶�ʱ��
		uint8_t u8HandleValueAfterLift;		//��������ٶ��ֱ�ֵ
		uint8_t u8VerificationCode[2];//У����
	};
}xDebugParams;


typedef union
{
	uint8_t u8Data[43];
	struct
	{
		uint8_t u8TerminalEncode[10];	 		//�ն˱���
		uint8_t u8ECU;		 		//ecu���к�
		uint8_t u8CmdType;	
		uint8_t u8CmdLenth;
		uint8_t u8CmdData[30];//���⴦����У����ֱ�ӷ���ָ��֮��
	};
}xEcho;

typedef struct
{
	uint8_t u8RevCmdFlag;
	uint8_t u8RevCmdCode;
	uint8_t u8RevCmdLength;
	uint8_t u8RevCmdData;
	uint8_t u8SendCmdLength;
	uint8_t u8SendCmdData[10];
}xCmdData;


#define BLOCK_TRANSMIT_IDLE			0
#define GPS_DATA_0001						1
#define SYS_ECHO_1001						2
#define SYS_ECHO_1002						3
#define	SYS_INFO_1101						4
#define SYS_INFO_1102						5
#define SYS_DATA_1201						6
#define SYS_DATA_1202						7
#define SYS_PARA_1301						8
#define SYS_PARA_1302						9
#define SYS_PORT_1401						10	
#define SYS_PORT_1402						11
#define CMD_SYS_QUERY_1901			12
#define CMD_SYS_QUERY_1801			13
#define CMD_SYS_QUERY_1904			14
#define CMD_SYS_QUERY_2001			15


#define TOPIC_1001 "/XGXF/SYS/Echo"	
#define TOPIC_1101	"/XGXF/SYS/Info/NVRJCC"
#define TOPIC_1201	"/XGXF/SYS/Data/NVRJCC"
#define TOPIC_1301	"/XGXF/SYS/Params/NVRJCC"
#define TOPIC_1401	"/XGXF/SYS/Port/NVRJCC"
#define TOPIC_1801 	"/XGXF/CMD/SYS/"	
#define TOPIC_1901  "/XGXF/CMD/SYS/"	


#define TERMINAL_ENCODE 		"QTECU03291"
#define ECU_CODE					  "QTECU2425H"
#define PCU_CODE						"PCUA010331"
#define FACTORY_CODE				"XFNVRJ"
#define CARCODE							"114514"
#define CONTROLLER_MARK				1

#define MCU_SOFTVERSION 	0x00000001
#define ECU_SOFTVERSION 	0x00000001
#define VERSION_DATE			0x240331
#define RMT_SOFTVERSION		0x0101

xCmdData sgRemotCmd;

uint8_t u8M370001[10] = {0};//GPS���к�
xMessageInfo sg3700MessageInfos[1] = 
{
	{.u16Index = 0x3700,.u8SubIndex = 0x01 , .u32Length = (sizeof (u8M370001)/sizeof(u8M370001[0])), .Message = u8M370001}, 
};


uint8_t u8M371001[14] = TOPIC_1001;
xEcho u8M371002 = { .u8TerminalEncode = TERMINAL_ENCODE ,.u8ECU = CONTROLLER_MARK,};
//uint8_t u8M371003[4];
//uint8_t u8M371004[100];
//uint8_t u8M371005[20];
//uint8_t u8M371006[10];

xMessageInfo sg3710MessageInfos[2] = 
{
	{.u16Index = 0x3710,.u8SubIndex = 0x01 , .u32Length = (sizeof (u8M371001)/sizeof(u8M371001[0])), .Message = u8M371001}, 
	{.u16Index = 0x3710,.u8SubIndex = 0x02 , .u32Length = (sizeof (u8M371002.u8Data)/sizeof(u8M371002.u8Data[0])), .Message = u8M371002.u8Data},
//	{.u16Index = 0x3710,.u8SubIndex = 0x03 , .u32Length = (sizeof (u8M371003)/sizeof(u8M371003[0])), .Message = u8M371003},
//	{.u16Index = 0x3710,.u8SubIndex = 0x04 , .u32Length = (sizeof (u8M371004)/sizeof(u8M371004[0])), .Message = u8M371004},
//	{.u16Index = 0x3710,.u8SubIndex = 0x05 , .u32Length = (sizeof (u8M371005)/sizeof(u8M371005[0])), .Message = u8M371005},
//	{.u16Index = 0x3710,.u8SubIndex = 0x06 , .u32Length = (sizeof (u8M371006)/sizeof(u8M371006[0])), .Message = u8M371006},
};

uint8_t u8M371101[21] = TOPIC_1101;
xSystemMessage u8M371102 = 
{
	.u8TerminalEncode = TERMINAL_ENCODE,
	.u8ECUEncode = ECU_CODE,
	.u8PCUEncode = PCU_CODE,
	.u8ManufacturerCode = FACTORY_CODE,
	.u8VehicleTypeEncode = CARCODE,
	.u8SoftwareVersion[0] = RMT_SOFTVERSION & 0xFF,
	.u8SoftwareVersion[1] = RMT_SOFTVERSION >> 8 & 0xFF,
	.u8ReleaseDate[0] = (VERSION_DATE & 0xFF),
	.u8ReleaseDate[1] = (VERSION_DATE >> 8 & 0xFF),
	.u8ReleaseDate[2] = (VERSION_DATE >> 16 & 0xFF),
};

xMessageInfo sg3711MessageInfos[2] = 
{
	{.u16Index = 0x3711,.u8SubIndex = 0x01 , .u32Length = (sizeof (u8M371101)/sizeof(u8M371101[0])), .Message = u8M371101}, 
	{.u16Index = 0x3711,.u8SubIndex = 0x02 , .u32Length = (sizeof (u8M371102.u8Data)/sizeof(u8M371102.u8Data[0])), .Message = u8M371102.u8Data},
};

uint8_t u8M371201[21]= TOPIC_1201;

xSystemOperatingData u8M371202 = 
{
	.u8TerminalEncode = TERMINAL_ENCODE,
	.u8ECUEncode = ECU_CODE,
	.u8PCUEncode = PCU_CODE,
	
};

xMessageInfo sg3712MessageInfos[2] = 
{
	{.u16Index = 0x3712,.u8SubIndex = 0x01 , .u32Length = (sizeof (u8M371201)/sizeof(u8M371201[0])), .Message = u8M371201}, 
	{.u16Index = 0x3712,.u8SubIndex = 0x02 , .u32Length = (sizeof (u8M371202.u8Data)/sizeof(u8M371202.u8Data[0])), .Message = u8M371202.u8Data},
};

uint8_t u8M371301[23]= TOPIC_1301;

xDebugParams u8M371302 = 
{
	.u8TerminalEncode = TERMINAL_ENCODE,
	.u8ECUEncode = ECU_CODE,
	.u8PCUEncode = PCU_CODE,
	
};

xMessageInfo sg3713MessageInfos[2] = 
{
	{.u16Index = 0x3713,.u8SubIndex = 0x01 , .u32Length = (sizeof (u8M371301)/sizeof(u8M371301[0])), .Message = u8M371301}, 
	{.u16Index = 0x3713,.u8SubIndex = 0x02 , .u32Length = (sizeof (u8M371302.u8Data)/sizeof(u8M371302.u8Data[0])), .Message = u8M371302.u8Data},
};

uint8_t u8M371401[21]= TOPIC_1401;

xSystemPortData u8M371402 = 
{
	.u8TerminalEncode = TERMINAL_ENCODE,
	.u8ECUEncode = ECU_CODE,
};

xMessageInfo sg3714MessageInfos[2] = 
{
	{.u16Index = 0x3714,.u8SubIndex = 0x01 , .u32Length = (sizeof (u8M371401)/sizeof(u8M371401[0])), .Message = u8M371401}, 
	{.u16Index = 0x3714,.u8SubIndex = 0x02 , .u32Length = (sizeof (u8M371402.u8Data)/sizeof(u8M371402.u8Data[0])), .Message = u8M371402.u8Data},
};


uint8_t u8M371801[14] = TOPIC_1801;
uint8_t u8M371802[1];
uint8_t u8M371803[4];
uint8_t u8M371804[100];
xMessageInfo sg3718MessageInfos[4] = 
{
	{.u16Index = 0x3718,.u8SubIndex = 0x01 , .u32Length = (sizeof (u8M371801)/sizeof(u8M371801[0])), .Message = u8M371801}, 
	{.u16Index = 0x3718,.u8SubIndex = 0x02 , .u32Length = (sizeof (u8M371802)/sizeof(u8M371802[0])), .Message = u8M371802},
	{.u16Index = 0x3718,.u8SubIndex = 0x03 , .u32Length = (sizeof (u8M371803)/sizeof(u8M371803[0])), .Message = u8M371803},
	{.u16Index = 0x3718,.u8SubIndex = 0x04 , .u32Length = (sizeof (u8M371804)/sizeof(u8M371804[0])), .Message = u8M371804},
};

uint8_t u8M371901[24];
uint8_t u8M371902[1];
uint8_t u8M371903[4];
uint8_t u8M371904[512];
static xMessageInfo sg3719MessageInfos[4] = 
{
	{.u16Index = 0x3719,.u8SubIndex = 0x01 , .u32Length = (sizeof (u8M371901)/sizeof(u8M371901[0])), .Message = u8M371901}, 
	{.u16Index = 0x3719,.u8SubIndex = 0x02 , .u32Length = (sizeof (u8M371902)/sizeof(u8M371902[0])), .Message = u8M371902},
	{.u16Index = 0x3719,.u8SubIndex = 0x03 , .u32Length = (sizeof (u8M371903)/sizeof(u8M371903[0])), .Message = u8M371903},
	{.u16Index = 0x3719,.u8SubIndex = 0x04 , .u32Length = (sizeof (u8M371904)/sizeof(u8M371904[0])), .Message = u8M371904},
};

uint8_t u8M372001[12];
xMessageInfo sg3720MessageInfos[1] = 
{
	{.u16Index = 0x3720,.u8SubIndex = 0x01 , .u32Length = (sizeof (u8M372001)/sizeof(u8M372001[0])), .Message = u8M372001},
};


#define PARA_HANDLEMAX				PARA_AngleValue5
#define PARA_HANDLEMID				PARA_AngleValue6
#define PARA_HANDLEMIN				PARA_AngleValue7
typedef struct
{
	int8_t i8MiddleValue;
	int8_t i8PositiveValue;
	int8_t i8NegativeValue;
}xPcuHandle;

static xPcuHandle sgPcuHandle;

#endif 
#endif //#ifndef _USER_COMM_H_
