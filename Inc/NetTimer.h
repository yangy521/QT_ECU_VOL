/*******************************************************************************
* Filename: NetTimer.h	                                             	 		   *
* Description:											   			 		   *
* Author:                                                           		   *
* Date:     														 		   *
* Revision:															 		   *
*******************************************************************************/
#ifndef _NET_TIMER_H_
#define _NET_TIMER_H_
#include "KSDsys.h"
#include "stdint.h"

#define TIMER_NUM			48


#define	TIMER_PLC			0
#define	TIMER_LedProc		1
#define	TIMER_WdgProc		2
#define	TIMER_MstSlvCom		3
#define	TIMER_CanRevProc	4
#define	TIMER_LocalDiProc	5
#define	TIMER_LocalDoProc	6
#define	TIMER_DoPwmProc		7

#define	TIMER_PcuComm			10
#define	TIMER_EcuPowerOn		11
#define	TIMER_EcuAntiPinchFunc	12
#define	TIMER_BatteryInit		13
#define	TIMER_HourCount			14
#define	TIMER_BatteryProc		15

#define	TIMER_SelfCheck			21
#define	TIMER_Drive1Check		22	/*Drive1��ʱ��*/
#define	TIMER_Drive2Check		23	/*Drive2��ʱ��*/
#define	TIMER_Drive3Check		24	/*Drive3��ʱ��*/
#define	TIMER_Drive4Check		25	/*Drive4��ʱ��*/
#define	TIMER_Drive5Check		26	/*Drive5��ʱ��*/
#define	TIMER_Drive6Check		27	/*Drive6��ʱ��*/
#define	TIMER_Drive7Check		28	/*Drive7��ʱ��*/
#define	TIMER_Drive8Check		29	/*Drive8��ʱ��*/
#define	TIMER_Drive9Check		30	/*Drive9��ʱ��*/
#define	TIMER_Drive10Check		31	/*Drive10��ʱ��*/
#define	TIMER_PorpDriver0Check	32	/*Drive11��ʱ��*/
#define	TIMER_PorpDriver1Check	33	/*Drive12��ʱ��*/

#define	TIMER_AI1Check			35	/*AI1��ʱ��*/
#define	TIMER_AI2Check			36	/*AI2��ʱ��*/
#define	TIMER_AI3Check			37	/*AI3��ʱ��*/
#define	TIMER_Encoder1Check		38	/*AI4��ʱ��*/
#define	TIMER_Encoder2Check		39	/*AI5��ʱ��*/


#define	TIMER_EEPROM			40	/*EEPROM��ʱ��*/

#define Timer_SpeedHold		41

//23.11.18 sj�û���ʹ�õĶ�ʱ��
//#define	TIMER_SwitchCheck	41	/*���ؼ����ʱ*/
//#define TIMER_Calibration	42	/*ѹ���궨��ʱ*/
//#define	TIMER_AlarmDelay	43	/*������ʱ*/

#define TIMER_USER_1			41	/*�û���ʱ��1*/
#define TIMER_USER_2			42	/*�û���ʱ��2*/
#define TIMER_USER_3			43	/*�û���ʱ��3*/
#define TIMER_USER_4			44	/*�û���ʱ��4*/

#define	TIMER_Test				45


#define	TIMER_PLC_PERIOD		5
#if 0
/* ����ʱ���ʱ�� */
#define Timer_Connect							0	//ICOMͨ�Ŷ�ʱ��
#define Timer_ICANSlave1					1	//ICAN��վ1���Ͷ�ʱ����Timer_Post)
#define Timer_ICANSlave2					2	//ICAN��վ2���Ͷ�ʱ����Timer_Post)
#define Timer_ICANSlave3					3	//ICAN��վ3���Ͷ�ʱ����Timer_Post)
#define Timer_ICANSlave4					4	//ICAN��վ4���Ͷ�ʱ����Timer_Post)
#define Timer_ICANSlave5					5	//ICAN��վ5���Ͷ�ʱ����Timer_Post)
#define Timer_ICANSlave6					6	//ICAN��վ6���Ͷ�ʱ����Timer_Post)
#define Timer_Logic								7	//�߼�������ʱ��
#define	Timer_SelfCheck						8	//�����������Լ�
#define	Timer_Drive1Check					9	//Drive1��ʱ��
#define	Timer_Drive2Check					10	//Drive2��ʱ��
#define	Timer_Drive3Check					11//Drive1��ʱ��
#define	Timer_Drive4Check					12	//Drive2��ʱ��
#define Timer_MacIDCheck        	13  //MAC��ⶨʱ��
#define Timer_KSICheck            14  //KSI��ⶨʱ��
#define Timer_VBusCheck						15	//ĸ�ߵ�ѹ��ⶨʱ��
#define Timer_Charge							16	//��綨ʱ��
#define Timer_SvOff								17	//����ֹͣ��ʱ��
//#define Timer_Led	          	    18	//ָʾ�Ƶƶ�ʱ��
#define Timer_SpeedHold  	   			19	//ת��������ʱ��ʱ��
#define Timer_VoltageEspCheck			20  //
#define Timer_PowerTmpMaxCheck1		21	//���ʰ������ֵ1��ⶨʱ��
#define Timer_PowerTmpMaxCheck2		22	//���ʰ������ֵ2��ⶨʱ��
#define Timer_PowerTmpMinCheck		23	//���ʰ������ֵ��ⶨʱ��
#define Timer_VoltageMinCheck 		24	//��ص�ѹ��ֵ2��ⶨʱ��
#define Timer_VoltageCutCheck		  25	//��ص�ѹ������ⶨʱ��
#define Timer_VoltageMaxCheck			26	//��ظ�ѹ��ֵ��ⶨʱ��
#define Timer_MotorTmpMaxCheck1		27	//���������ֵ1��ⶨʱ��
#define Timer_MotorTmpMaxCheck2		28	//���������ֵ2��ⶨʱ��
#define	Timer_OUT5V_Check					29	//
#define	Timer_OUT12V_Check				30	//
#define Timer_KSIAbnormalCheck    31  //KSI��ѹ�쳣��ⶨʱ��
#define Timer_VBusAbnormalCheck		32	//ĸ�ߵ�ѹ�쳣��ⶨʱ��
#define Timer_TERSet						33	// 
#define	Timer_WeldedCheck					34	//���Ӵ��������۽Ӽ�⣬ĸ�ߵ��ݵ�ѹδ�ͷ�
#define	Timer_SvOffDelay					35	//
#define Timer_TERClr			36	//
#define Timer_ErrorDelay			37	//
//#define 			38	//
//#define 			39	//
#define Timer_ICAN_Delay					40	//ICAN�����ͺ�ϵͳ30ms����
#endif
//�������綨ʱ���ṹ��
typedef struct _tNetTimer
{
	uint64_t  u64TimerCount;		//	ʱ���ʱ������λms��
	uint64_t  u64Deadline;			//	��ʱʱ��
	tBoolean  bIsStart;						//	�Ƿ�����
	tBoolean  bIsOvertime;				//	�Ƿ�ʱ
	
}tNetTimer;

//��������
extern tNetTimer netTimer[TIMER_NUM];		//

extern void vNetTimerInit(void);//���綨ʱ����ʼ��
extern void vNetTimerUpdate(void);//�����¼���ʱ����ʱ
extern void vSetNetTimer(uint8_t ucTimerID,uint64_t u64Deadline);//�������綨ʱ��
extern void vResetNetTimer(uint8_t u8TimerID);//��λ���綨ʱ��
extern void vKillNetTimer(uint8_t u8TimerID); //ɾ�����綨ʱ��
extern uint8_t u8GetNetTimerOverFlag(uint8_t u8TimerID);
extern uint8_t u8GetNetTimerStartFlag(uint8_t u8TimerID);

#endif