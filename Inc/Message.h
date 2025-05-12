/*******************************************************************************
* Filename: Message.h	                                             	 		   *
* Description:											   			 		   												*
* Author:                                                           		   *
* Date:     														 		   														*
* Revision:															 		 														  *
*******************************************************************************/

#include "KSDsys.h"
#include "queue.h"

//������
#define Msg_ICOM_MAX	32 	//��С���ܳ���queue.h��LEN_DATAQUEUE�Ĵ�С
#define Msg_ICAN_MAX	32
//��Ϣ���Ͷ���
#define Msg_Type_ICOM		1
#define Msg_Type_ICAN		2
#define Msg_Type_System	3
#define Msg_Type_Error	4
//�¼����ƶ���
//ICOM
#define Msg_ReadConfigData		1
#define Msg_ReadPthData				2
#define Msg_ICOMRetry					3
//ICAN
#define Msg_ReadPrdType				10
#define Msg_ICANRetry					12

#define Msg_MacIDCheck				13
#define Msg_WritePort					14
#define Msg_ReadPort					15
#define Msg_SetConnect				16

//System
#define Msg_SaveConfigData		21
//���ϱ������壨����ԽС�����ȼ�Խ�ߣ�
#define Msg_Err_ICANMacIDCheck		31	// MacID���ʧ��
#define Msg_Err_ICOMConnect				32 	// ICOM����ʧ��
#define Msg_Err_ICANConnect				33	// ICAN����ʧ��
#define Msg_Err_ConfigUnfinished	34	// ϵͳ����δ���
#define Msg_ErrAck								35	// ICAN����δ����
#define Msg_Err_CnvType						36	// �豸�������ô���

#define Msg_Err_ConfigData			38	// ϵͳ���ô���
#define Msg_Err_PthData					39	// ·�����ô���
#define Msg_Err_OverVoltage			40	// ��ѹ����
#define Msg_Err_ShortVoltage		41	// Ƿѹ����
#define Msg_Err_IPM							42	// ������������
#define Msg_Err_OverLoad				43	// ���ر���
#define Msg_Err_Eeprom					44	// �洢���󱨾�
#define Msg_Err_Belt						45	// Ƥ����ƫ����

//���屨��
typedef struct _tAlarm
{
	unsigned char	ucMacID;		//	��ƷID��CANID��
	unsigned char	ucType;			//	��������
}tAlarm;
//��������
extern tAlarm sysAlarm[64];	//ϵͳ�����б�


extern void MsgInitialize(void);//��Ϣ�����ʼ��
extern void MsgFlush(void);//��Ϣ�������
extern void MsgManage(void);//��Ϣ�¼�����	
extern void MsgICANRegister(unsigned char ucMacID,unsigned char ucEvent);//ICAN��Ϣע��
extern void MsgICOMRegister(unsigned char ucEvent);//ICOM��Ϣע��	
extern void MsgAdd(unsigned char ucType,tMessage message);//��Ϣ���	
extern tBoolean MsgICANPop(tMessage *pMessage);//ICAN��Ϣ�����������ȳ���	
extern tBoolean MsgICOMPop(tMessage *pMessage);//ICOM��Ϣ�����������ȳ���
extern void AlarmICANRegister(unsigned char ucMacID,unsigned char ucEvent);//ICAN����ע��	
extern void AlarmICOMRegister(unsigned char ucEvent);//ICOM����ע��
extern void AlarmSysRegister(unsigned char ucEvent);	//ϵͳ����ע��
