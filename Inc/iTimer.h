/*******************************************************************************
* Filename: iTimer.h	                                             	 		   *
* Description:											   			 		   *
* Author:                                                           		   *
* Date:     														 		   *
* Revision:															 		   *
*******************************************************************************/

#include		"KSDsys.h"

#define TIMER_NUM			48
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

//�������綨ʱ���ṹ��
typedef struct _tNetTimer
{
	unsigned long ulTimerCount;		//	ʱ���ʱ������λms��
	unsigned long ulDeadline;			//	��ʱʱ��
	tBoolean  bIsStart;						//	�Ƿ�����
	tBoolean  bIsOvertime;				//	�Ƿ�ʱ
}tNetTimer;

//��������
extern tNetTimer netTimer[TIMER_NUM];		//

extern void netTimerInit(void);//���綨ʱ����ʼ��
extern void netTimerUpdate(void);//�����¼���ʱ����ʱ
extern void SetNetTimer(unsigned char ucTimerID,unsigned long ulDeadline);//�������綨ʱ��
extern void ResetNetTimer(unsigned char ucTimerID);//��λ���綨ʱ��
extern void KillNetTimer(unsigned char ucTimerID); //ɾ�����綨ʱ��
