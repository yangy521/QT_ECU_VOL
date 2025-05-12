/*******************************************************************************
* Filename: UserStackerTruckProc.h	                                           *
* Description: �߼����ͷ�ļ���������Ӧ�Ļص�����	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/07/22    														   *
* Revision:	 														 		   *
*******************************************************************************/
#ifndef _USER_HANGCHA_PODAOCHE_PROC_H_
#define _USER_HANGCHA_PODAOCHE_PROC_H_

#include "MstSlvCom.h"
#include "DoPwm.h"
#include "PropDriver.h"
#include "LocalDi.h"
#include "NetTimer.h"

/******************************************************************************
*��������
******************************************************************************/


#define	USER_PERIOD				5	

#define	LIFTUP_VALVE				DRIVER5
#define DOWN_VALVE          DRIVER7 //�½���Ϊ��ŷ�
#define	LEAN_FORWARD_VALVE	DRIVER3
#define HONR_VALVE          DRIVER9  

#define	LEAN_BACKWARD_VALVE		DRIVER4


#define	LIFT_UP_LIMIT_SWI		SWI1_R
#define	EMS_SWI							SWI2_R
#define	LOCK_SWI						SWI3_R
#define	CHARGE_SWI					SWI4_R

#define	TURN_SEN_INPUT		AI_B_AI1_R
#define	ANG_SEN_INPUT			AI_B_AI2_R
#define PRESS_SEN_INPUT   AI_B_AI3_R

#define	CAN_1E0_LOST_NO			(1000 / USER_PERIOD)

#define	ECU_POWERON_DELAY_TIME	500
#define	ECU_ANTI_PINCH_TIME 		3000
#define	DRIVER_CLOSE			0
#define	DRIVER_OPEN				1
//#define	DRIVER_CLOSE			1
//#define	DRIVER_OPEN				0

#define	LIFT_MODE_SWI					0
#define	LIFT_MODE_THROTTLE		1

#define	MOTOR_MIN_SPEED				30		/*����ת��*/
#define	MOTOR_MAX_SPEED				2000
#define	MOTOR_MAX_SPEED_VALUE	4095
#define	MOTOR_SPEED_RANGE			4096
#define	PUMP_MAX_VALUE				255
#define	PUMP_RANGE						256

#define	PROP_MIN_CURRENT		PARA_PropDMinCurrent1
#define	PROP_MAX_CURRENT		PARA_PropDMaxCurrent1
#define	LEAN_FORWARD_PARA		PARA_PumpMotorGear1
#define	LEAN_BACKWARD_PARA	PARA_PumpMotorGear2

#define	LIFT_MODE_PARA			PARA_BrakeType
#define	USERINFO_MODE				PARA_ValueOpenPercentage


#define FORWORD_LIMIT_ANGLE  PARA_AngleValue0  //ǰ����λ
#define BACKWORD_LIMIT_ANGLE PARA_AngleValue1  //������λ
#define AUTO_RAMP_ANGLE      PARA_AngleValue2  //�Զ��µ���
#define SET_EXIT_ANGLE       PARA_AngleValue3  //�˳��µ���
#define SET_MAX_TILE_ANGLE   PARA_AngleValue4  //�µ�ģʽ�������������
#define MAX_LEAN_ANGLE       PARA_AngleValue5  //���������������
#define PRELEAN_ANGLE        PARA_AngleValue6  //Ԥ��б��
#define TARGET_ANGLE         PARA_AngleValue7  //Ŀ����б��

#define MAX_PRESSURE         PARA_FullPressure    //���ѹ��

#define TURN_DEC_SPD_ENABLE  PARA_AngleSensorSetting   //#70 1ʹ��ת�併�٣�0�ر�
#define TILESENSORENABLE     PARA_TiltSwitchSetting    //#71 1��Ǵ�������Ч��0��Ч
#define TURN_DIF_SPD_ENABLE  PARA_AnticollisionFunc    //����#116����ײ���� 1�������ٹ��ܣ�0�ر�
#define ANGLEAILIMIT         PARA_AngleSimulationLimit   //#60 �Ƕ�ģ����λ����
#define INTERLOCK_EN         PARA_ActAlmFunc            //������������
#define LOWPOWER_1STEP       PARA_Analog3DeadZoneMinVal //����141�����͵���һ������
#define LOWPOWER_2STEP       PARA_Analog3DeadZoneMaxVal //����142�����͵�����������




#define USER_SET           PARA_ValveType
//#define	BAT_LOW_WARING_VAL			20
//#define	BAT_LOW_ERR_VAL				15	


#define	LIFTUP_VALVE_ERR					ErrCode53
#define	LEAN_FORWARD_VALVE_ERR		ErrCode54
#define	LEAN_BACKWARD_VALVE_ERR		ErrCode55
#define	LIFTDOWN_VALVE_ERR				ErrCode61









/*����ͨ�ô�����*/
#define	ACT_INIT_ERR				 ErrCode89
#define	ACT_LOCK_ERR				 ErrCode90
#define	MOVE_EMS_ERR				 ErrCode91
#define CHARGE_ACT_ERR       ErrCode92
#define	AI_B_AI1_ERR				 ErrCode93   //ת��Ƕȴ�����(ģ����1����)


/*********************�µ������������********************************************/
#define	HANDLE_NOCAN_ERR		 ErrCode100  //�ֱ����ĳ�ʱ
#define BMS_NOCAN_ERR        ErrCode101  //﮵�BMS���ĵ���
#define EBRAKE_ADHESION      ErrCode102  //��������ճ��
#define EBRAKE_ERR           ErrCode103  //�����������˳�����
#define REMOTE_LOST_ERR      ErrCode104  //Զ�̹���ģ�鱨�ĳ�ʱ
#define TILESENSOR_ERR       ErrCode105  //��б��������ģ����2������
#define HMI_COM_ERR          ErrCode106  //�Ǳ�����ʧ��
#define HMI_CAN_LOST         ErrCode107  //�Ǳ��ĳ�ʱ
#define DIRICTION_ERR        ErrCode108  //�Ƕȴ�����Ϊ��ֵ�ҳ�����������Ƕ�
#define OVER_LEAN_ERR        ErrCode109  //���ͱ���
#define OVER_PRESS_ERR       ErrCode110  //��ѹ����
#define FRONTRANK_NOCAN_ERR  ErrCode111  //ǰ��Ǵ���������(185)
#define CASTER_NOCAN_ERR     ErrCode112  //����Ǵ���������(186)
#define PRESSENSOR_ERR       ErrCode113  //ѹ����������ģ����3�����߱���
/**********************﮵������**************************************************/
#define SoloUnderVotage_ERR  ErrCode114   //����Ƿѹ
#define AllUnderVotage_ERR   ErrCode115   //��������Ƿѹ
#define TmpProtect_ERR       ErrCode116   //�¶ȱ�����һ�㣩
#define OverTmp_ERR          ErrCode117   //���ع��±���
#define OverVotage_ERR       ErrCode118   //��ѹ����
#define OverCurrent_ERR      ErrCode119   //��������

/*δ���������*/
#define SMOVE_ERR            ErrCode149
#define LOWPOWER_1ERR        ErrCode150//����һ���͵�����ֵ����
#define LOWPOWER_2ERR        ErrCode151//���ڶ����͵�����ֵ����



#define PARA_WorkCountL		PARA_USER_DATA_24
#define PARA_WorkCountH		PARA_USER_DATA_25

#define	STEP_FACTOR					(5.0 * 4096 / 255)

#define	TURN_START_ANGLE		PARA_TurnWithDecStartAngle
#define	TURN_END_ANGLE			PARA_TurnWithDecEndAngle
#define	START_ANGLE_SPD			PARA_AngleWithStartSpdPer
#define	END_ANGLE_SPD				PARA_AngleWithEndSpdPer

#define SOFTWARE_VERSION_CODE  0x0101

typedef struct
{
	uint16_t	u16StartAngle;
	uint16_t	u16EndAngle;
	uint16_t	u16StartAngleSpd;
	uint16_t	u16EndAngleSpd;
	float		fAgnleDecSpdFactor;
}xSteerAngleDecSpd;


extern void vUserEcuInit(void);
extern void vUserEcuProc(void);
void vUserEcuLogic(void);

#endif
