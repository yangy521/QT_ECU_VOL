/*******************************************************************************
* �칤 ��ֱ�����泵 �߼�����* 						   *
* Author: QExpand; Ji Wenshuai                                                        *
* Date: 2023/12/19   														   *
* Revision:	 														 		   *
*******************************************************************************/
#include "UserCanComm.h"
#include "USER_XUGONG_3DC_JCC.h"
#include "AlarmLamp.h"
#include "ErrCode.h"
#include "Eeprom.h"
#include "IQmathLib.h"
#include "PropProc.h"
#include "CanRevProc.h"
#include "string.h"
#include "Para.h"
#include "WdgProc.h"
#include "math.h"
#include "Log.h"
#include "LocalAi.h"
#include "LedProc.h"
#include "BeepProc.h"
#include "AiProc.h"
#include "LocalDo.h"
#include "HourCount.h"
#include "BatteryMeter.h"
#include "UserCommon.h"
#include "PressureSensor.h" 
#include "PcuProc.h"

#if (USER_TYPE == USER_XUGONG_3DC_JCC)//�޸�

const static xPdoParameter  sgPdoPara = 
{	
	.TpdoPara = {
		{.u8Flag = 1, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 1, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 1, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 1, .u8Type = 0x00, .u16Period = 50},
		{.u8Flag = 0, .u8Type = 0xFF, .u16Period = 20, .u16CanId = 0x000 },
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50, .u16CanId = 0x000 },
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50, .u16CanId = 0x000 },
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50, .u16CanId = 0x000 },
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50, .u16CanId = 0x000 },
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50, .u16CanId = 0x000 },
	},
	.RpdoPara = {
		{.b1Flag = 1, .b11CanRevId = 0x28A},
		{.b1Flag = 1, .b11CanRevId = 0x530},
		{.b1Flag = 1, .b11CanRevId = 0x18A},
		{.b1Flag = 1, .b11CanRevId = 0x726},
		{.b1Flag = 0, .b11CanRevId = 0x000},
		{.b1Flag = 0, .b11CanRevId = 0x000},
	},
};

static xErrCodeInfo sgErrCodeInfo[ErrCodeMax] = 
{		/*MCU���ϳ���������ͳһ��81*/
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//1		��������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//2		�ں����д���
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//3		��������ʱ�����
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//4		λ�ó���
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//5		���ӳ���λ��ָ��仯���������ٶ�
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//6		�ٶ�ģʽ���ٶ�ָ���������ת��
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//7		ת��ģʽ��ת��ָ������ת��
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//8		�ٶȴ���������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//9		�ٶȴ������������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//10	��
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//11	���2����������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//12	�������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//13	ĸ�ߵ��ݳ�����
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 10,},	//14	���Ӵ�������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//15	�ƶ�������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//16	��ص�ѹ��
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//17	��ص�ѹ��
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//18	���ʰ����
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//19	�������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//20	��λ�ƶ�·���ѹ
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//21	���Ӵ��������۽�
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//22	5v�������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//23	id��
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//24	���Ӵ�����������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//25	����ģ�����
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//26	CanͨѶ����
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//27	��ѹ��������ѹ2V
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//28	��
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//29	��������쳣
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//30	���д���
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//31	��ѹ��ȹ���
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//32	���ʰ���ȹ���
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//33	���ʰ����
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//34	�����ȸ���
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//35	12v����
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//36	DO3����
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//37	DO4����
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//38	eeprom������д����
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//39	������Ŵ���
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//40	�ϵ�IO�쳣
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//41	��������20%
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//42	��������10%
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//43	��ѹ�ﵽ������ֵ
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 60,},	//50
		
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 0,},	//51	DO1
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 0,},	//52	DO2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 0,},	//53	DO3
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 0,},	//54	DO4
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 0,},	//55	DO5
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 0,},	//56	DO6
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 0,},	//57	DO7
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 0,},	//58	DO8
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 0,},	//59	DO9
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 0,},	//60	DO10�������û��㱨��
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//61		DO11
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//62		DO12
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//63		DO13
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//64		DO14
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//65		DO15		
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 27,},//66 		�½���0		
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 55,},//67 		�½���1
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//68		Ԥ��
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//69		Ԥ��
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//70
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//71	ģ����1
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//72	ģ����2
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//73	ģ����3
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//74	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//75
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//76
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//77	����������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//78
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//79
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//80
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//81	����MCU����
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//82	����MCU�����쳣
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//83	���Ź��쳣
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//84	PCUͨѶ����
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//85	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//86	д��������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//87	����������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//88	��������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//89	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//90
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//91
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//92
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//93
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//94
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//95
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//96
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//97
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//98
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//99
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//100
		
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//101	��ʼ��
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1,	.b8UserErr = 2,},	//102	ͨ�Ŵ���
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 3,},	//103	��Чѡ��
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 4,},	//104	�ڴ����ݴ���
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 5,},	//105	﮵����
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 1, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 6,},	//106	�ϵ�ʱ��������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 1, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 7,},	//107	�ϵ�ʱ���¹���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 1, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 8,},	//108	�ϵ�ʱ��������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 9,},	//109	GPS����
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 10,},//110	���Ӵ������Ӵ���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 11,},//111	��
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 12,},//112	����ʱ�¿ش���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 13,},//113	BMS����²��-2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 14,},//114	BMS����¶ȸ�-1
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 15,},//115 BMS-�ŵ��¶ȹ���2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 16,},//116	BMS-�ŵ��������1
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 17,},//117	BMS-�ŵ��������2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 18,},//118	�Ӷ���������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 19,},//119	BMS-�ܵ�ѹ����1
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 20,},//120	BMS-�ܵ�ѹ����2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 21,},//121	BMS-�����ѹ����1
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 22,},//122	BMS-�����ѹ����2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 23,},
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 24,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 25,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 26,},
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 27,},//127	�½���2����
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 28,},
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 29,},
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 30,},//130	BMS-���ѹ�����
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 31,},//131	ѹ������������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1,	.b8UserErr = 32,},//132	�Ƕȴ���������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 33,},//133	������ʹ���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 34,},
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1,	.b8UserErr = 35,},//135	���ر궨��
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 36,},//136	��ص�����һ������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 37,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1,	.b8UserErr = 38,},//138	δ�궨��ɻ�궨ʧ��
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1,	.b8UserErr = 39,},//139	mcuͨ�Ź���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 40,},//140 �Ǳ�ͨ�Ź���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 41,},//141	ƽ̨һ������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1,	.b8UserErr = 42,},//142	����ʱ��ƽ̨����ת��ť���´���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1,	.b8UserErr = 43,},//143	����ʱ��ƽ̨����ת��ť���´���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1,	.b8UserErr = 44,},//144	ƽ̨��������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 45,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 1, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 46,},//146	����ʱ��ƽ̨�ֱ�ʹ�ܿ��ذ�ť���´���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 47,},//147	����ʱ��ƽ̨�ֱ�������λ����
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 48,},
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 49,},//149	��������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 50,},//
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 51,},
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 52,},//152	ǰ����
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 53,},//153	���˷�
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 54,},//154	������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 55,},//155	�½���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 56,},//156	��ת��
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 57,},//157	��ת��
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 58,},//158	�����
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 59,},//159	����������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 60,},//160	����������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 61,},//161	��������������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 62,},//162	������Ӳ������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 63,},//163	�õ����·
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 64,},//164	������·
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 65,},//165	���Ƶ�ѹ5V����
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 66,},//166	����ʱ����⵽��ת����·���·
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 67,},//167	���Ƶ�ѹ12V����
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 68,},//168	��ص͵�����������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 69,},//169	����λ��������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 70,},//170	������ĸ�ߵ�ѹ���߹���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 71,},//171	Ԥ�����
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 72,},//172	������ĸ�ߵ�ѹ���͹���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 73,},//173	���������¹���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 74,},//174	����������һ������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 75,},//175	�õ���¶�һ������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 76,},//176	�õ������������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 77,},//177	����������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 78,},//178	�õ�����������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 79,},//179	�õ���¶ȶ�������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 80,},//180	���� 80%���ر���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},//181	�������¶ȶ�������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 82,},//182	��ɲ������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 83,},//183	��ɲ������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 84,},//184	�õ����ת��ʧ��
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 85,},//185	��ǣ�������ת��ʧ��
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 86,},//186	��ǣ�������ת��ʧ��
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 87,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 88,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 89,},//189	����������ʱ���������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 90,},//190	���� 90%���ر���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 91,},//191	����������������	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 92,},//192	�ҵ��������������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 93,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 94,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 95,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 96,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 97,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 98,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 99,},//199	���� 99%���ر���
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 100,},//200	ƽ̨���ر���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 101,},//201	������б������ȫ�޶�����
	//{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 39,},//202	��ײ��������ʾ39
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1,  .b8UserErr = 144,},//244  �ֱ�����
};

typedef union
{
	uint16_t u16data;
	struct
	{
		uint16_t b1PcuSwi: 1;
		uint16_t b1TiltSwi: 1;
		uint16_t b1PitSwi: 1;
		uint16_t b1UpLimitSwi: 1;
		uint16_t b1DownLimitSwi: 1;
		uint16_t b1LowerCtlUp:1;
		uint16_t b1LowerCtlDown: 1;
		uint16_t b1AntiCollision:1;
		uint16_t b1LowerCtlEn:1;
	};
}xSwiInput;

/*PCU Input*/
typedef union 
{
	uint8_t u8data;
	struct
	{
		uint8_t b1MoveMode:1;
		uint8_t b1LiftMode:1;
		uint8_t b1Slow:1;
		uint8_t b1Speaker:1;
		uint8_t b1TurnLeft:1;
		uint8_t b1TurnRight:1;
		uint8_t b1Enable:1;
		uint8_t b1HandleValue:1;
	};	
}xPcuKeyInfo;


typedef struct
{
	xPcuKeyInfo PcuKeyInfo;
	int16_t i16HandleValue;
}xPCUInfo;

typedef union
{
	uint8_t u8Data;	
	struct
	{
		uint8_t b1ForwardAct:1;
		uint8_t b1BackwardAct:1;
		uint8_t b1LiftUpAct:1;
		uint8_t b1LiftDownAct:1;
		uint8_t b1TurnLeft:1;
		uint8_t b1TurnRight: 1;
		uint8_t b2Reserve2:2;
	};
}xActLogic;

typedef union
{
	uint16_t u16Data;
	struct
	{
		uint16_t b1Above1M8: 1;
		uint16_t b1HeightSpdLimit: 1;
		uint16_t b1SpdMode: 1;
		uint16_t b1PasswordFunc: 1;
		uint16_t b1Rental: 1;
		uint16_t b11Reserve: 11;
	};
}xSaveStateInfo;

typedef union
{
	uint8_t u8Data;
	struct
	{
		uint8_t b1Error:1;
		uint8_t b1Warning:1;
		uint8_t PCUBeep:1;
		uint8_t PCUSilence:1;
		uint8_t b1CaliInit:1;
		uint8_t	b1CaliEnd:1;
		uint8_t	b1CaliErr:1;
	};
}xErrorState;


typedef union
{
	uint8_t u8Data[8];
	struct 
		{
			uint8_t u8CS;
			uint8_t u8IndexL;
			uint8_t u8IndexH;
			uint8_t u8SubIndex;
			uint8_t u8DataLL;
			uint8_t u8DataLH;
			uint8_t u8DataHL;
			uint8_t u8DataHH;
		};
}xCanBlueInfo;

typedef union
{
	uint8_t u8Data;
	struct 
		{
			uint8_t b1NoAct:1;
			uint8_t b1NoPcu:1;
			uint8_t b1SpeedAfterLift:1;
			uint8_t b1NoDown:1;
			uint8_t b1NoTurn:1;
			uint8_t b1NoLift:1;
			uint8_t b1NoMove:1;
			uint8_t b1Slow:1;
		};
}xLimit;

typedef union
{
	uint8_t u8Data;
	struct
	{
		uint8_t b1LiftState:1;
		uint8_t	b1SteerState:1;
		uint8_t b1LowerMove:1;
		uint8_t	b1UpMove:1;
		uint8_t b1LiftDownState:1;
		uint8_t b1OverLoadState:1;
	};
}xActionFlag;

typedef struct
{
	uint32_t u32LiftTimes;
	uint32_t u32SteerTimes;
	uint32_t u32MoveTimes;
	uint32_t u32OverLoadTimes;
}xActionTimes;

typedef struct
{
	uint32_t u32LifHour;
	uint32_t u32SteerHour;
	uint32_t u32LowerMoveHour;
	uint32_t u32UpMoveHour;
	uint32_t u32DownHour;
}xActionHour;

static int32_t i32PropFeedBack = 0;
static uint16_t i16SpeedFeedback = 0;
static uint8_t	u8PumpFeedback = 0;


xCanSendPdoInfo gCanSendPdoInfo ;
xCanRevPdoInfo gCanRevPdoInfo;	



//����
static xSwiInput sgSwiInput;
static xPCUInfo sgPcuInfo;
static xPCUInfo PCUStateRecord;

static uint32_t u32HourCnt = 0;
static xLimit sgLimit;
static xActLogic sgActLogic;

//��ʼ��ģʽ����
static xPcuKeyInfo sgPcuKeyInit;

static uint8_t u8RunMode = 0;
	#define	NORMAL_MODE	0
	#define PARA_SETS_MODE	1
	#define	PRESSURE_CALIBRATION_MODE	2
	
static uint8_t u8ParaSetMode = 0;
#define PARASETS_DISABLE	0
#define COMMON_PARASET		1
#define SPEED_PARASET			2
#define MACHINE_PARASET		3
#define	FUNCTION_PARASET	4
#define H8_MODE						5
#define H2_MODE						6

static xErrorState sgErrorState;


//�м䴫�ݱ���
static uint16_t	u16MotorVal = 0;
static uint16_t u16MotorFdb = 0;


static uint16_t u16SpeedTarget = 0;
static uint16_t u16SpeedCmd = 0;
static uint16_t u16Time = 0;

static	uint8_t u8Soc;

static uint8_t u8PcuMode = 0;
#define INITIAL_MODE		0	
#define MOVE_MODE				1		
#define	LIFT_MODE				2		


static uint8_t u8AntiPinchState = 2;//��ʼ״̬Ϊ����Ϊ֮��
#define ABOVE_SWI					0
#define WAIT_RELEASE			1
#define	UNDER_SWI_DELAY		2
#define UNDER_SWI_ACT			3

/*tbox*/
static xActionFlag sgActionFlag;
static xActionTimes sgActionTimes;
static xActionHour sgActionour;

/*HMI*/
static xCanSDO CanSend582;
static xCanSDO CanRev602;

static xCanSend5A5 CanSend5A5 ;

static uint8_t u8HmiCheckFlag ;
static uint8_t u8LiCheckFlag ;
static uint8_t u8AngleSimulationFlag ;

//ģ��������
//��ʱδʹ����ѹ��ͨ��1��ѹ��ͨ��2
static void vAiErrCallBack(eAiErrNo AiErrNo)
{
	switch((uint8_t)AiErrNo)
	{
		case ANGLE_SENSOR_CHANNEL:
			i32ErrCodeSet(ANGLE_SENSOR_ERR);			
			break;
		case PRESSURE_SENSOR_CHANNEL1:
			i32ErrCodeSet(PRESSURE_SENSOR_ERR);
			break;
		default:
			break;
	}
}

//DDO����
//ȱ�ٸߵ��١����ȵı�����
static void vDoPwmErrCallBack(eDoPwmNo DoPwmNo)
{
	uint8_t u8cnt = 0;
	//����Դ�DO����
	{
		for(u8cnt = 0;u8cnt<10;u8cnt++)
		{
			i32ErrCodeClr(ErrCode51 + u8cnt);
		}
	}
	
	switch((uint8_t)DoPwmNo)
	{
		case LIFTUP_PUMP:
			i32ErrCodeSet(LIFT_UP_VALVE_ERR);
			break;
		case TURNRIGHT_PUMP:
			i32ErrCodeSet(TURN_RIGHT_VALVE_ERR);
			break;
		case TURNLEFT_PUMP:
			i32ErrCodeSet(TURN_LEFT_VALVE_ERR);
			break;
//��ֱ��û��ǰ�������˷�
//		case BACKWARD_PUMP:
//			i32ErrCodeSet(BACKWARD_VALVE_ERR);
//			break;		
//		case FORWARD_PUMP:
//			i32ErrCodeSet(FORWARD_VALVE_ERR);
//			break;		
//		case BLINK_LED:
//			i32ErrCodeSet(BLINK_LED_VALV_ERR);
//			break;
//		case HIGH_SPEED_PUMP:
//			i32ErrCodeSet();
//			break;
		case BLINK_BEEP:
			i32ErrCodeSet(BLIKN_BEEP_VALVE_ERR);
			break;
//		case SPEAKER_PUMP:
//			i32ErrCodeSet();
//			break;
		default:
			break;
	}
}

static void vPropErrCallBack(uint8_t u8Channel)
{
	//����Դ�����
	i32ErrCodeClr(ErrCode66);
	i32ErrCodeClr(ErrCode67);
	switch(u8Channel)
	{
//		case PropDriverCh0:
//			i32ErrCodeSet(LIFT_DOWN_VALVE_ERR);
//			break;
//		case PropDriverCh1:
//			i32ErrCodeSet(LOW_VALVE2_ERR);
			break;
		default:
			break;
	}
}

static void vSwiInitCheck(void)//�¿ط�Ϊ���������canͨ�����롢��ȷ������ѡ��
{
	if((1 == gCanRevPdoInfo.CanLowerControl.b1LiftKey)
		||(1 == gCanRevPdoInfo.CanLowerControl.b1DownKey))
	{
		i32ErrCodeSet(UPDOWN_BUTTON_ERR);
	}
}

void vPcuErrProc(uint8_t u8Type)
{
	switch (u8Type)
	{
		case PCU_Init:
			{
				u8PcuMode = INITIAL_MODE;
				//��ֱ��û��ǰ�������˷�
//				i32DoPwmSet(FORWARD_PUMP, PUMP_CLOSE_PERCENTAGE);
//				i32DoPwmSet(BACKWARD_PUMP, PUMP_CLOSE_PERCENTAGE);
				i32DoPwmSet(TURNLEFT_PUMP, PUMP_CLOSE_PERCENTAGE);
				i32DoPwmSet(TURNRIGHT_PUMP, PUMP_CLOSE_PERCENTAGE);
				i32DoPwmSet(LIFTUP_PUMP, PUMP_CLOSE_PERCENTAGE);
				vPropSetTarget(LIFTDOWN_PUMP1, 0);
				vPropSetTarget(LIFTDOWN_PUMP2, 0);
			}
			break;
		case PCU_LiftKeyPress:
			sgPcuKeyInit.b1LiftMode = 1;
			i32ErrCodeSet(LIFT_BUTTON_ERR);	
			break;
		case PCU_SlowKeyPress:
			sgPcuKeyInit.b1Slow = 1;
			i32ErrCodeSet(SLOW_BUTTON_ERR);
			break;
		case PCU_MoveKeyPress:
			sgPcuKeyInit.b1MoveMode = 1;
			i32ErrCodeSet(MOVE_BUTTON_ERR);
			break;
		case PCU_TurnLeftPress:
			sgPcuKeyInit.b1TurnLeft = 1;
			i32ErrCodeSet(PLAT_LEFT_BUTTON_ERR);
			break;
		case PCU_TurnRightPress:
			sgPcuKeyInit.b1TurnRight = 1;
			i32ErrCodeSet(PLAT_RIGHT_BUTTON_ERR);
			break;
		case PCU_EnableKeyPress:
			sgPcuKeyInit.b1Enable = 1;
			i32ErrCodeSet(ENABLE_BUTTON_ERR);
			break;
		case PCU_ValueNoZero:
			sgPcuKeyInit.b1HandleValue = 1;
//			i32ErrCodeSet(HANDLE_NOT_IN_MIDDLE_POSI_ERR);
			break;
		case PCU_SpeakerPress:
			sgPcuKeyInit.b1Speaker = 1;
			break;
		default:
			break;
	}
}

//INPUT
void vPcuRevProc(xPcuRevPara *RevData)
{
	int8_t	i8HandleValue = 0;
	if ((0x2 == RevData->Data.b4Const1) && (0x3 == RevData->Data.b4Const2) && \
		(0x4 == RevData->Data.b4Const3) && (0x5 == RevData->Data.b4Const4) && \
		(0x6 == RevData->Data.b4Const5) && (0x7 == RevData->Data.b4Const6) && \
		(0xC == RevData->Data.b4Const7))
	{
		sgPcuInfo.PcuKeyInfo.b1Slow = RevData->Data.b1SlowSpdSwitch;
		sgPcuInfo.PcuKeyInfo.b1Speaker = RevData->Data.b1SpeakerSwitch;
		sgPcuInfo.PcuKeyInfo.b1MoveMode = RevData->Data.b1TraSwitch;
		sgPcuInfo.PcuKeyInfo.b1LiftMode = RevData->Data.b1LiftingSwitch;
		sgPcuInfo.PcuKeyInfo.b1Enable = RevData->Data.b1EnableSwitch;
		sgPcuInfo.PcuKeyInfo.b1TurnRight = RevData->Data.b1TurnRightSwitch;
		sgPcuInfo.PcuKeyInfo.b1TurnLeft = RevData->Data.b1TurnLeftSwitch;
		/*Set Key Info*/
		i32SetPara(PARA_PcuKeyInfo, sgPcuInfo.PcuKeyInfo.u8data);
			
		i8HandleValue  = RevData->Data.b4HandleCtrlHigh << 4 | RevData->Data.b4HandleCtrlLow;
		
		
		if (abs(i8HandleValue) < gUserInfo.u8DeadZoneAdjust)
		{
			i8HandleValue = 0;
		}
		i32SetPara(PARA_HandleAnalog, (abs(i8HandleValue) * 100 / 127));
		
		sgPcuInfo.i16HandleValue = i8HandleValue * 32;
		
		if(LIFT_MODE == u8PcuMode)
		{
			if(FunctionEnable == gUserInfo.u8LiftReverseFunc)
			{	
				sgPcuInfo.i16HandleValue = -i8HandleValue * 32;
			}
		}
	}
}
//����������
//
static void vSwiMonitor(void)
{
	xSwiInput SwiInput = {0};
	static	uint16_t u16TiltDelay = 0;
	
	if(1 == i32LocalDiGet(PCU_SWICTH))
	{
		SwiInput.b1PcuSwi = 1;
	}
	if(1 == gCanRevPdoInfo.CanLowerControl.b1LiftKey)
	{
		SwiInput.b1LowerCtlUp = 1;
	}
	if(1 == gCanRevPdoInfo.CanLowerControl.b1DownKey)
	{
		SwiInput.b1LowerCtlDown = 1;
	}
	if(1 == gCanRevPdoInfo.CanLowerControl.b1EnableKey)
	{
		SwiInput.b1LowerCtlEn = 1;
	}
	if(FunctionEnable == gUserInfo.u8TiltSwitchSetting)
	{
		if(0 == i32LocalDiGet(TILT_SIWTCH))
		{
			if(u16TiltDelay < 200)//��ʱ�жϱ�������������ɸ���
				u16TiltDelay++;
			else
			{
				SwiInput.b1TiltSwi = 1;
			}
		}
		else
		{
			u16TiltDelay = 0;
		}		
	}

	if(1 == i32LocalDiGet(PIT_SWITCH))
	{
		SwiInput.b1PitSwi = 1;
	}
		
	if(FunctionEnable == gUserInfo.u8AngleSimulationLimit)//�Ƕ�ģ����λ����
	{
		int32_t i32Angletmp = 0;
		i32Angletmp = i32LocalAiGet(ANGLE_SENSOR_CHANNEL);
		if(i32Angletmp <= gUserInfo.u16AngleSimulationDownLimit)
		{
			SwiInput.b1DownLimitSwi = 1;
		}
		else if(i32Angletmp >= gUserInfo.u16AngleSimulationUpLimit)
		{
			SwiInput.b1UpLimitSwi = 1;
		}
	}
	else
	{
		if(0 == i32LocalDiGet(UP_LIMIT_SWITCH))
		{
			SwiInput.b1UpLimitSwi = 1;
		}
		if(1 == i32LocalDiGet(DOWN_LIMIT_SWITCH))
		{
			SwiInput.b1DownLimitSwi = 1;
		}
	}
	
	//�����Ӷ�����
	if(0 == SwiInput.b1DownLimitSwi)
	{
		if(1 == SwiInput.b1TiltSwi)
		{
			i32ErrCodeSet(MACHINE_TILT_OVER_SAFETY_ERR);
		}
		else
		{
			i32ErrCodeClr(MACHINE_TILT_OVER_SAFETY_ERR);
		}
		
		if((FunctionEnable == gUserInfo.u8PitProtectFunc)&&(0 == SwiInput.b1PitSwi))
		{
			i32ErrCodeSet(PIT_PROCETION_ERR);
		}
		else
		{
			i32ErrCodeClr(PIT_PROCETION_ERR);
		}
	}
	else
	{
		i32ErrCodeClr(PIT_PROCETION_ERR);
		i32ErrCodeClr(MACHINE_TILT_OVER_SAFETY_ERR);
	}
	
	i32SetPara(PARA_BrakeValveCurrent,SwiInput.u16data);
	sgSwiInput.u16data = SwiInput.u16data;
}

//����ģʽ�л�
//�ֱ��Ĳ������Զ��궨���ع��ܡ�
static void vModeChange(void)//�Ѳ���
{
	if((0 != sgPcuKeyInit.u8data)&&(PARASETS_DISABLE == u8ParaSetMode))//�ϵ�ʱ�ֱ���������Ϊ0
	{
		if((1 == sgSwiInput.b1PcuSwi)
			&&(1 == sgPcuKeyInit.b1Speaker)
			&&(1 == sgPcuKeyInit.b1LiftMode)
			&&(1 == sgPcuKeyInit.b1Slow)
			&&(0 == sgPcuInfo.PcuKeyInfo.b1Speaker)
			&&(0 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
			&&(0 == sgPcuInfo.PcuKeyInfo.b1Slow))//ƽ̨λ�ã������ȣ�����������,Ȼ���ɿ�  ͨ�ò���
		{
			u8RunMode = PARA_SETS_MODE;
			u8ParaSetMode = COMMON_PARASET;
			i32ErrCodeClr(LIFT_BUTTON_ERR);
			i32ErrCodeClr(SLOW_BUTTON_ERR);
			i32ErrCodeClr(MOVE_BUTTON_ERR);
		}
		if((1 == sgSwiInput.b1PcuSwi)
			&&(0 == sgPcuKeyInit.b1Slow)
			&&(1 == sgPcuKeyInit.b1Speaker)
			&&(1 == sgPcuKeyInit.b1LiftMode)
			&&(0 == sgPcuInfo.PcuKeyInfo.b1Speaker)
			&&(0 == sgPcuInfo.PcuKeyInfo.b1LiftMode)//���Ⱦ��� �ٶ�����
		)
		{
			u8RunMode = PARA_SETS_MODE;
			u8ParaSetMode = SPEED_PARASET;
			i32ErrCodeClr(LIFT_BUTTON_ERR);
			i32ErrCodeClr(SLOW_BUTTON_ERR);
			i32ErrCodeClr(MOVE_BUTTON_ERR);
		}
		else if((0 == sgSwiInput.b1PcuSwi)
			&&(1 == sgPcuKeyInit.b1Slow)
			&&(1 == sgPcuKeyInit.b1MoveMode)
			&&(0  == sgPcuInfo.PcuKeyInfo.b1MoveMode)
			&&(0 == sgPcuInfo.PcuKeyInfo.b1Slow)//��������H9
			)
		{
			u8RunMode = PARA_SETS_MODE;
			u8ParaSetMode = FUNCTION_PARASET;		
			i32ErrCodeClr(LIFT_BUTTON_ERR);
			i32ErrCodeClr(SLOW_BUTTON_ERR);
			i32ErrCodeClr(MOVE_BUTTON_ERR);			
		}
		else if((0 == sgSwiInput.b1PcuSwi)
				&&(1 == sgPcuKeyInit.b1Slow)
				&&(1 == sgPcuKeyInit.b1Speaker)
				&&(0 == sgPcuInfo.PcuKeyInfo.b1Speaker)
				&&(0 == sgPcuInfo.PcuKeyInfo.b1Slow))//�������� H8ģʽ
		{
			u8RunMode = PARA_SETS_MODE;
			u8ParaSetMode = H8_MODE;		
			i32ErrCodeClr(LIFT_BUTTON_ERR);
			i32ErrCodeClr(SLOW_BUTTON_ERR);
			i32ErrCodeClr(MOVE_BUTTON_ERR);						
		}
	}
	else if((SPEED_PARASET == u8ParaSetMode))//�ٶ�ģʽ���������͹��������
	{
		static uint16_t u16KeyCnt=0;
		if((u16KeyCnt < (3000 / TIMER_PLC_PERIOD))
			&&(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
			&&(1 == sgPcuInfo.PcuKeyInfo.b1Slow))
			u16KeyCnt++;
		else if((u16KeyCnt == (3000 / TIMER_PLC_PERIOD))&&(0 == sgPcuInfo.PcuKeyInfo.u8data))
			u8ParaSetMode = MACHINE_PARASET;
		
	}
	else if((H8_MODE == u8ParaSetMode)&&(1 == sgPcuInfo.PcuKeyInfo.b1Enable))
	{
		u8RunMode = H2_MODE;
	}
	else if((NORMAL_MODE == u8RunMode)&&(1 == sgSwiInput.b1PcuSwi))//����ģʽ����궨״̬
	{
		//�Ѳ���
		static uint32_t	u32KeyRecord = 0;//����Ϊ1������Ϊ0
		static xSwiInput	KeyState ;
		static uint8_t	u8SwitchCnt = 0;
		
		#define EMPTY_CALI_SWITCH		0b11111011111010111
		#define FULL_CALLI_SWITCH		0b11111011111011111
		
		if((1 == sgSwiInput.b1LowerCtlDown)||(1 == sgSwiInput.b1LowerCtlUp))//ÿ��һ�ξ͸�λ
		{
			if(false == u8GetNetTimerStartFlag(TIMER_SwitchCheck))
				vSetNetTimer(TIMER_SwitchCheck,SWITCH_CHECK_TIME);
			else
				vResetNetTimer(TIMER_SwitchCheck);
		}
		
		if((1 == KeyState.b1LowerCtlUp)&&(0 == sgSwiInput.b1LowerCtlUp))//�ϼ��ɿ�
		{
			u32KeyRecord = u32KeyRecord << 1;
			u8SwitchCnt++;
		}
		
		if((1 == KeyState.b1LowerCtlDown)&&(0 == sgSwiInput.b1LowerCtlDown))//�¼��ɿ�
		{
			u32KeyRecord = ( u32KeyRecord << 1) + 1;
			u8SwitchCnt++;
		}
		KeyState.u16data = sgSwiInput.u16data;
		
		//�Ƚ�
		if(((0 != (u32KeyRecord ^ (EMPTY_CALI_SWITCH >>( 17 - u8SwitchCnt))))
			&&(0 != (u32KeyRecord ^ (FULL_CALLI_SWITCH >>( 17 - u8SwitchCnt)))))
			||(true == u8GetNetTimerOverFlag(TIMER_SwitchCheck)))//����������ͬ��ʱ,�������
		{
			u32KeyRecord = 0;
			u8SwitchCnt = 0;
			vKillNetTimer(TIMER_SwitchCheck);
		}

		if(0 == (u32KeyRecord ^ EMPTY_CALI_SWITCH))//�л�ģʽ�������ʱ��
		{
			u8RunMode = PRESSURE_CALIBRATION_MODE;
			vKillNetTimer(TIMER_SwitchCheck);
		}
		else if(0 == (u32KeyRecord ^ FULL_CALLI_SWITCH))
		{
			u8RunMode = PRESSURE_CALIBRATION_MODE;
			vKillNetTimer(TIMER_SwitchCheck);
		}
		i32SetPara(PARA_ForwardValveCurrent,KeyState.u16data);
	}
//	else if((PRESSURE_CALIBRATION_MODE == u8RunMode)&&(true == ))//�궨��ɣ�������һ��
//	{
//		
//	}
//		i32SetPara(PARA_OnOffValveCurrent,u8RunMode);
		i32SetPara(PARA_TurnRightValveCurrent,u8ParaSetMode);
}


//����sgActLogic
static void vActionMonitNormal()
{
	if(1 == sgSwiInput.b1PcuSwi)
	{
		if(INITIAL_MODE == u8PcuMode)//ģʽ�л�
		{
			if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
				u8PcuMode = LIFT_MODE;
			else if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
				u8PcuMode = MOVE_MODE;
		}
		else
		{
			/*Slow Speed */
			if((MOVE_MODE == u8PcuMode)
			&& (0 == sgPcuInfo.PcuKeyInfo.b1Slow)//��ǰ����Ϊ0
			&& (1 == PCUStateRecord.PcuKeyInfo.b1Slow))//��ʷ����Ϊ1
			{
				sgLimit.b1Slow ^= 1;
			}
			/*Mode Change*/
			if(0 == sgPcuInfo.PcuKeyInfo.b1Enable)//PCU
			{
				if((MOVE_MODE == u8PcuMode)&&(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode))
				{
					u8PcuMode = LIFT_MODE;
				}
				else if((LIFT_MODE == u8PcuMode)&&(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode))
				{
					u8PcuMode = MOVE_MODE;
				}
			}
			
			if((MOVE_MODE == u8PcuMode)&&(1 == sgPcuInfo.PcuKeyInfo.b1Enable))
			{
				/*Steer Process*/
				if(0 == sgLimit.b1NoTurn)//��ת������
				{
					if(1 ==sgPcuInfo.PcuKeyInfo.b1TurnLeft)
					{
						sgActLogic.b1TurnLeft = 1;
					}
					else if(1 ==sgPcuInfo.PcuKeyInfo.b1TurnRight)
					{
						sgActLogic.b1TurnRight = 1;
					}
				}
				/*Forward Backward Process*/
				if(0 == sgLimit.b1NoMove)
				{
					if(sgPcuInfo.i16HandleValue < 0)
					{
						sgActLogic.b1BackwardAct = 1;
					}
					else if(sgPcuInfo.i16HandleValue>0)
					{
						sgActLogic.b1ForwardAct = 1;
					}
				}
				u16MotorVal = abs(sgPcuInfo.i16HandleValue);
			}
			else if((LIFT_MODE == u8PcuMode)&&(1 == sgPcuInfo.PcuKeyInfo.b1Enable))
			{

				
				if((sgPcuInfo.i16HandleValue > 0)&&(0 == sgLimit.b1NoLift))
				{
					sgActLogic.b1LiftUpAct = 1;
				}
				if((sgPcuInfo.i16HandleValue < 0)&&(0 == sgLimit.b1NoDown))
				{
					sgActLogic.b1LiftDownAct = 1;
				}
				u16MotorVal = abs(sgPcuInfo.i16HandleValue);
			}
			else
			{
				u16MotorVal = 0;
			}
		}
	}
	else 
	{
		if((1 == sgSwiInput.b1LowerCtlUp)&&(0 == sgLimit.b1NoLift)&&(1 == sgSwiInput.b1LowerCtlEn))
		{
			sgActLogic.b1LiftUpAct = 1;
			u16MotorVal = MOTOR_MAX_SPEED_VALUE;
		}
		else if((1 == sgSwiInput.b1LowerCtlDown)&&(0 == sgLimit.b1NoDown)&&(1 == sgSwiInput.b1LowerCtlEn))
		{
			sgActLogic.b1LiftDownAct = 1;
			u16MotorVal = MOTOR_MAX_SPEED_VALUE;
		}
		else
		{
			u16MotorVal = 0;
		}
	}
		
	
	/*antipinch process*/
	if(FunctionEnable == gUserInfo.u8AntiPinchFunc)
	{
		switch(u8AntiPinchState)
		{
			case ABOVE_SWI:
				vKillNetTimer(TIMER_EcuAntiPinchFunc);
				if((1 == sgSwiInput.b1DownLimitSwi)&&(1 == sgActLogic.b1LiftDownAct))
				{
					u8AntiPinchState = WAIT_RELEASE;
				}
				break;
			case WAIT_RELEASE:
				sgActLogic.b1LiftDownAct = 0;
				if(0 == sgSwiInput.b1DownLimitSwi)
				{
					u8AntiPinchState = ABOVE_SWI;
				}
				else if(((1 == sgSwiInput.b1PcuSwi)&&(0 == sgPcuInfo.PcuKeyInfo.b1Enable))
							||((0 == sgSwiInput.b1PcuSwi)&&(0 == sgSwiInput.b1LowerCtlEn))
				)
				{
					u8AntiPinchState = UNDER_SWI_DELAY;
				}	
				break;
			case UNDER_SWI_DELAY:
				if(0 == sgSwiInput.b1DownLimitSwi)
				{
					u8AntiPinchState = ABOVE_SWI;
				}
				else if((1 == sgActLogic.b1LiftDownAct)&&(false == u8GetNetTimerStartFlag(TIMER_EcuAntiPinchFunc)))
				{
					uint16_t u16tmp;
					u16tmp = (gUserInfo.u8AccAndDecAntiPinch * 100);
					vSetNetTimer(TIMER_EcuAntiPinchFunc, u16tmp);
				}
				else if((0 == sgActLogic.b1LiftDownAct)&&(true == u8GetNetTimerStartFlag(TIMER_EcuAntiPinchFunc)))
				{
					vKillNetTimer(TIMER_EcuAntiPinchFunc);
				}
				else if(true == u8GetNetTimerOverFlag(TIMER_EcuAntiPinchFunc))
				{
					vKillNetTimer(TIMER_EcuAntiPinchFunc);
					u8AntiPinchState = UNDER_SWI_ACT;
				}
				sgActLogic.b1LiftDownAct = 0;
				break;
			case UNDER_SWI_ACT:
				if(0 == sgSwiInput.b1DownLimitSwi)
				{
					u8AntiPinchState = ABOVE_SWI;
				}
				else if(0 == sgActLogic.b1LiftDownAct)
				{
					u8AntiPinchState = UNDER_SWI_DELAY;
				}
				break;
			default:
				u8AntiPinchState = ABOVE_SWI;
				break;
		}
	}
	if(1==sgPcuInfo.PcuKeyInfo.b1Speaker)
		i32DoPwmSet(SPEAKER_PUMP,PUMP_OPEN_PERCENTAGE);
	else
		i32DoPwmSet(SPEAKER_PUMP,PUMP_CLOSE_PERCENTAGE);
}

//ѹ���궨
//�ȴ������
static void vActionMonitPressureCalibration()
{
	static uint8_t u8ActionFlag = 0;
	static uint8_t u8AngleCount = 0;
	uint16_t u16StopAnle = 0;
	
	#define PC_INIT				0
	#define PC_FIRSTLIFT	1
	#define PC_FIRSTDOWN	2
	#define PC_SECONDLIFT	3
	#define PC_SECONDDOWN	4
	#define PC_THIRDLIFT	5
	#define PC_THIRDDOWN	6
	#define PC_END				7
	#define PC_ERR				8
	
	u8PcuMode = LIFT_MODE;
	u16StopAnle = gUserInfo.u16MinAngle + (gUserInfo.u16MaxAngle - gUserInfo.u16MinAngle) * u8AngleCount /PressureCali_Times;
	i32SetPara(PARA_BackValveCurrent,u8ActionFlag);
		i32SetPara(PARA_LiftValveCurrent,u16StopAnle);
	switch(u8ActionFlag)
	{
		case PC_INIT://�Ƕȡ�����λ�����͵����ˮƽ
			if(false == u8GetNetTimerStartFlag(TIMER_AlarmDelay))//����������ȫ���ͨ����������5�Σ���ʼ�궨�����򱨾�
			{
				vSetNetTimer(TIMER_AlarmDelay,ALARM_TIME);
			}
			else if(false == u8GetNetTimerOverFlag(TIMER_AlarmDelay))//��ʼ��ʼ�����
			{
				if(20 >=(u8GetBatterySoc()))
				{
					u8ActionFlag = PC_ERR;
				}
				if(FunctionEnable == gUserInfo.u8AngleSimulationLimit)
				{
					if(gUserInfo.u16MinAngle >= gUserInfo.u16MaxAngle)
						u8ActionFlag = PC_ERR;
				}
				sgErrorState.b1CaliInit =	1;
			}
			else
			{
				sgErrorState.b1CaliInit =	0;
					if(gUserInfo.u16MinAngle + 300 > i32LocalAiGet(ANGLE_SENSOR_CHANNEL))//����С�Ƕ�֮����ֵ��Χ��
					{
						sgActLogic.b1LiftDownAct = 1;
						u16MotorVal = MOTOR_MAX_SPEED_VALUE;
						if(false == u8GetNetTimerStartFlag(TIMER_Calibration))
						{
							vSetNetTimer(TIMER_Calibration,CALIBRATION_INIT_TIME);
						}
						if(true == u8GetNetTimerOverFlag(TIMER_Calibration))
						{
							vKillNetTimer(TIMER_AlarmDelay);
							vKillNetTimer(TIMER_Calibration);
							u8ActionFlag = PC_FIRSTLIFT;
						}
					}
					else
					{
						sgActLogic.b1LiftDownAct = 1;
						u16MotorVal = MOTOR_MAX_SPEED_VALUE;
					}
			}
			break;
		case PC_FIRSTLIFT:
				if(FunctionEnable == gUserInfo.u8AngleSimulationLimit)//�нǶ�ģ����λ���ܣ����Ƕ��Ƿ�����С�Ƕ�
				{
					if(gUserInfo.u16MaxAngle <= i32LocalAiGet(ANGLE_SENSOR_CHANNEL))//���Ƕ�
					{
						if(false == u8GetNetTimerStartFlag(TIMER_Calibration))
						{
							vSetNetTimer(TIMER_Calibration,CALIBRATION_DELAY_TIME);
						}
						if(true == u8GetNetTimerOverFlag(TIMER_Calibration))
						{
							vKillNetTimer(TIMER_Calibration);
							u8ActionFlag = PC_FIRSTDOWN;
						}
						u16MotorVal = 0;
					}
					else//δ�������Ƕ�
					{
						sgActLogic.b1LiftUpAct = 1;
						u16MotorVal = MOTOR_MAX_SPEED_VALUE;
					}
				}
//				else
//				{
//					if(1 == sgSwiInput.b1UpLimitSwi)//��������λ���أ�ֹͣ
//					{
//						u8ActionFlag = PC_FIRSTDOWN;
//						u16MotorVal = 0;
//					}
//					else//����λ����֮��
//					{
//						sgActLogic.b1LiftUpAct = 1;
//						u16MotorVal = MOTOR_MAX_SPEED_VALUE;
//					}
//				}
			break;
		case PC_FIRSTDOWN:
				if(FunctionEnable == gUserInfo.u8AngleSimulationLimit)//�нǶ�ģ����λ���ܣ����Ƕ��Ƿ�����С�Ƕ�
				{
					if(gUserInfo.u16MinAngle + 300 > i32LocalAiGet(ANGLE_SENSOR_CHANNEL))//����С�Ƕ�֮����ֵ��Χ��
					{	
						sgActLogic.b1LiftDownAct = 1;
						u16MotorVal = MOTOR_MAX_SPEED_VALUE;								
						if(false == u8GetNetTimerStartFlag(TIMER_Calibration))
						{
							vSetNetTimer(TIMER_Calibration,CALIBRATION_INIT_TIME);
						}
						else if(true == u8GetNetTimerOverFlag(TIMER_Calibration))
						{
							vKillNetTimer(TIMER_Calibration);
							u8ActionFlag = PC_SECONDLIFT;
						}
					}
					else
					{
						sgActLogic.b1LiftDownAct = 1;
						u16MotorVal = MOTOR_MAX_SPEED_VALUE;
					}
				}
				else
				{
//					if(1 == sgSwiInput.b1DownLimitSwi)//��������λ����,���½�8��
//					{
//						if(false == u8GetNetTimerStartFlag(TIMER_Calibration))
//						{
//							sgActLogic.b1LiftDownAct = 1;
//							u16MotorVal = MOTOR_MAX_SPEED_VALUE;
//							vSetNetTimer(TIMER_Calibration,CALIBRATION_INIT_TIME);
//						}
//						else if(true == u8GetNetTimerOverFlag(TIMER_Calibration))
//						{
//							vKillNetTimer(TIMER_Calibration);
//							u8ActionFlag = PC_SECONDLIFT;
//						}
//					}
//					else//����λ����֮��
//					{
//						sgActLogic.b1LiftDownAct = 1;
//						u16MotorVal = MOTOR_MAX_SPEED_VALUE;
//					}
				}
			break;
		case PC_SECONDLIFT://û�нǶȴ�������ô����
				if(FunctionEnable == gUserInfo.u8AngleSimulationLimit)//�нǶ�ģ����λ���ܣ����Ƕ��Ƿ�����С�Ƕ�
				{
					if(true == u8GetNetTimerOverFlag(TIMER_Calibration))//��ʱ���˺󴥷����������������Ӧ�ǶȺ�ֹͣȻ�����¿�ʼ��ʱ
					{

						if(u16StopAnle <= i32LocalAiGet(ANGLE_SENSOR_CHANNEL))//�����Ӧ�Ƕȣ����¼�ʱ
						{
							u8AngleCount ++;
							vKillNetTimer(TIMER_Calibration);
						}
						else
						{
							sgActLogic.b1LiftUpAct = 1;
							u16MotorVal = MOTOR_MAX_SPEED_VALUE;
						}
						if(gUserInfo.u16MaxAngle <= i32LocalAiGet(ANGLE_SENSOR_CHANNEL))
						{
							u16MotorVal = 0;

							if(false == u8GetNetTimerStartFlag(TIMER_AlarmDelay))
							{
								vSetNetTimer(TIMER_AlarmDelay,CALIBRATION_DELAY_TIME);
							}
							if(true == u8GetNetTimerOverFlag(TIMER_AlarmDelay))
							{
								vKillNetTimer(TIMER_AlarmDelay);
								vKillNetTimer(TIMER_Calibration);
								u8ActionFlag = PC_SECONDDOWN;
							}
						}
					}
					else
					{
						if(false == u8GetNetTimerStartFlag(TIMER_Calibration))
							vSetNetTimer(TIMER_Calibration,CALIBRATION_PROC_TIME);
						u16MotorVal = 0;
					}				
				}
				else
				{
//					if(1 == sgSwiInput.b1DownLimitSwi)//��������λ����,���½�8��
//					{
//						if(false == u8GetNetTimerStartFlag(TIMER_Calibration))
//						{
//							sgActLogic.b1LiftDownAct = 1;
//							u16MotorVal = MOTOR_MAX_SPEED_VALUE;
//							vSetNetTimer(TIMER_Calibration,CALIBRATION_INIT_TIME);
//						}
//						else if(true == u8GetNetTimerOverFlag(TIMER_Calibration))
//						{
//							vKillNetTimer(TIMER_Calibration);
//							u8ActionFlag = PC_SECONDLIFT;
//						}
//					}
//					else//����λ����֮��
//					{
//						sgActLogic.b1LiftDownAct = 1;
//						u16MotorVal = MOTOR_MAX_SPEED_VALUE;
//					}
				}
			break;
		case PC_SECONDDOWN:
				if(true == u8GetNetTimerOverFlag(TIMER_Calibration))//��ʱ���˺󴥷����������������Ӧ�ǶȺ�ֹͣȻ�����¿�ʼ��ʱ
				{
					if(u16StopAnle >=  i32LocalAiGet(ANGLE_SENSOR_CHANNEL))//�����Ӧ�Ƕȣ����¼�ʱ
					{
						u8AngleCount --;
						vKillNetTimer(TIMER_Calibration);
					}
					else
					{
						sgActLogic.b1LiftDownAct = 1;
						u16MotorVal = MOTOR_MAX_SPEED_VALUE;
					}
					if(gUserInfo.u16MinAngle + 10>= i32LocalAiGet(ANGLE_SENSOR_CHANNEL))
					{
						if(false == u8GetNetTimerStartFlag(TIMER_AlarmDelay))
						{
							vSetNetTimer(TIMER_AlarmDelay,CALIBRATION_DELAY_TIME);
						}
						if (false == u8GetNetTimerOverFlag(TIMER_AlarmDelay))
						{
							sgActLogic.b1LiftDownAct = 1;
							u16MotorVal = MOTOR_MAX_SPEED_VALUE;							
						}
						if(true == u8GetNetTimerOverFlag(TIMER_AlarmDelay))
						{
							u16MotorVal = 0;
							vKillNetTimer(TIMER_AlarmDelay);
							vKillNetTimer(TIMER_Calibration);
							u8ActionFlag = PC_END;
						}
					}
				}
				else
				{
					if(false == u8GetNetTimerStartFlag(TIMER_Calibration))
						vSetNetTimer(TIMER_Calibration,CALIBRATION_PROC_TIME);
					u16MotorVal = 0;
				}	
			break;
		case PC_THIRDLIFT:
			break;
		case PC_THIRDDOWN:
			break;
		case PC_END:
			if(false == u8GetNetTimerStartFlag(TIMER_AlarmDelay))
				vSetNetTimer(TIMER_AlarmDelay,ALARM_TIME);
			else if(false == u8GetNetTimerOverFlag(TIMER_AlarmDelay))
			{
				sgErrorState.b1CaliInit =	1;
			}
			else
			{
				sgErrorState.b1CaliInit =	0;
			}
			break;
		case PC_ERR://������������
//			sgErrorState.b1CaliInit = 0;
//			sgErrorState.b1CaliErr = 1;
//			u16MotorVal = 0;
			break;
		default:
			break;
	}
}

static void vActionMonitParaSets()
{
	//��ִ�в���
}

//ִ��ģʽ�л�

static void vPcuPS(xPcuSendPara *SendData)
{
	SendData->Data.b4ErrCodeSigLHigh4 = 0x7;
	SendData->Data.b4ErrCodeSigLLow4 	= 0x3;
	SendData->Data.b4ErrCodeSigRHigh4 =	0x6;
	SendData->Data.b4ErrCodeSigRLow4 	= 0xD;
}
static void vPcuH9(xPcuSendPara *SendData)
{
	SendData->Data.b4ErrCodeSigLHigh4 = 0x7;
	SendData->Data.b4ErrCodeSigLLow4 	= 0x6;
	SendData->Data.b4ErrCodeSigRHigh4 =	0x6;
	SendData->Data.b4ErrCodeSigRLow4 	= 0xF;
}
static void vPcuSC(xPcuSendPara *SendData)
{
	SendData->Data.b4ErrCodeSigLHigh4 = 0x6;
	SendData->Data.b4ErrCodeSigLLow4 	= 0xD;
	SendData->Data.b4ErrCodeSigRHigh4 =	0x3;
	SendData->Data.b4ErrCodeSigRLow4 	= 0x9;
}
static void vPcuNull(xPcuSendPara *SendData)
{
	SendData->Data.b4ErrCodeSigLHigh4 = 0;
	SendData->Data.b4ErrCodeSigLLow4 	= 0;
	SendData->Data.b4ErrCodeSigRHigh4 =	0;
	SendData->Data.b4ErrCodeSigRLow4 	= 0;
}
static void vPcuCP(xPcuSendPara *SendData)
{
	SendData->Data.b4ErrCodeSigLHigh4 = 0x3;
	SendData->Data.b4ErrCodeSigLLow4 	= 0x9;
	SendData->Data.b4ErrCodeSigRHigh4 =	0x7;
	SendData->Data.b4ErrCodeSigRLow4 	= 0x3;
}
	
#define EEPROME_DATA	0
#define COMBINE_DATA	1

static uint16_t u16PcuParaMonit(xPcuSendPara *SendData,uint16_t u16Index,uint8_t u8DataSource)
{
	static uint16_t u16IndexREC = 0;
	static uint16_t u16ParaValue = 0;
	static uint8_t	u8SaveCnt = 0;
	static xPcuKeyInfo PcuKeyRec ;
	
	if(u16Index != u16IndexREC)//��������ת�䣬�洢�ɲ�������ȡ�²���
	{
		if(EEPROME_DATA == u8DataSource)
		{
			if(0 != u16IndexREC)
			{
				u16SaveParaToEeprom(u16IndexREC , u16ParaValue);
			}
			u16IndexREC = u16Index;
			u16ParaValue = i32GetPara(u16IndexREC);
		}
			
		if(COMBINE_DATA == u8DataSource)
			u16ParaValue = u16Index;
	}
	else
	{
		u8SaveCnt ++ ;
		if((0 == sgPcuInfo.PcuKeyInfo.b1TurnLeft)&&(1 == PcuKeyRec.b1TurnLeft))
		{
			u16ParaValue --;
			u8SaveCnt = 0;
		}
		
		if((0 == sgPcuInfo.PcuKeyInfo.b1TurnRight)&&(1 == PcuKeyRec.b1TurnRight))
		{
			u16ParaValue ++;
			u8SaveCnt = 0;
		}
		if(u16ParaValue> 100)
			u16ParaValue = 100;
		
		if(u8SaveCnt >= 20)//ÿ1s��һ�Σ���ֹͻȻ����
		{
			u8SaveCnt = 0;
			if(EEPROME_DATA == u8DataSource)
			{
				if(u16ParaValue != (i32GetPara(u16IndexREC)))
				u16SaveParaToEeprom(u16IndexREC , u16ParaValue);
				i32SetPara(u16IndexREC,u16ParaValue);
			}
		}
	}	
	vPcuDisplayNumber(SendData,u16ParaValue,DISPLAY_NORMAL);
	
	PcuKeyRec.u8data = sgPcuInfo.PcuKeyInfo.u8data;
	return u16ParaValue;
}

//PCU��˸��ʾ
static void	vPcuParaSetProc(xPcuSendPara *SendData)//����Ϊ���ڣ�A->��->B->��->A
{
	static uint8_t u8DisplayCnt = 0;
	static uint8_t u8DisplayPage = 0;
	static uint8_t u8SwitchDelay = 0;
	static uint8_t u8SubPage = 0;
	static uint8_t u8ParaSetModeRec = 0;
	
//	uint8_t u8CombinedData = 0;
	static uint8_t u8CombinedData = 0;
	static uint32_t u32CombinedDataH9 = 0;
	uint8_t u8DisplayMode1 = 0;
	uint8_t u8DisplayMode2 = 0;
	
	uint8_t u8SaveState = 0;
	
	static xPcuKeyInfo PcuKeyRec2;
	
	#define PS_INIT	0
	#define PS_LIFT	1
	#define PS_DOWN	2
	#define PS_ANTI	3
	#define	PS_FAST	4
	#define PS_SLOW	5
	#define PS_AFTER	6
	#define	PS_STEER	7
	#define PS_SAVEPARA	8
	
	#define SC_INIT		0
	#define SC_SETSL	1
	#define SC_SETSR	2

	#define CP_INIT									0
	#define CP_LOWBATSLP						1
	#define CP_LOWBATALM						2
	#define CP_MAX_PRESSUREDIFF			3
	#define CP_DYN_OVERRLOADRATE		4
	#define	CP_DYN_OVERLOADDELAY		5
	#define	CP_STB_OVERLOADRATE			6
	#define CP_ANGLE_UPLIMIT				7
	#define	CP_ANGLE_DOWNLIMIT			8
	#define	CP_UPCONTROL_SLEEPTIME	9
	#define CP_ALARMVOLUE						10
	#define	CP_ANTI_DELAYTIME				11
	#define CP_DOWN_DEC							12
	
	
	if(u8ParaSetModeRec != u8ParaSetMode)//�л�ģʽ��ҳ�棬��ʱ������ʼ��
	{
		u8ParaSetModeRec = u8ParaSetMode;
		u8DisplayPage = 0;
		u8SwitchDelay = 0;
		u8SubPage = 0;
		u8CombinedData = 0;
		u32CombinedDataH9 = 0;
		SendData->Data.b1Beep = 0;
	}

	
	switch(u8ParaSetMode)
	{
		case PARASETS_DISABLE:
		{
			
			if ((MOVE_MODE == u8PcuMode)
				&&((0 != sgLimit.b1Slow)||(0 != sgLimit.b1SpeedAfterLift)))
			{
				SendData->Data.b1SlowLed = 1;
			}
			else
			{
				SendData->Data.b1SlowLed = 0;
			}
			
			if(MOVE_MODE == u8PcuMode)//���ߵƿ��Ʒ����ݲ�֪��
			{
//				SendData->Data.b1ModeLed = 1;
//				SendData->Data.b1LiftLed = 0;
//				SendData->Data.b1MoveLedCtr = 1;
//				SendData->Data.b1LiftLedCtl = 1;
					//12.2 TEST.�칤DD����ģʽ��D7����ģʽ
				SendData->Data.b4ModeControl = 0xD;
				
			}
			else if(LIFT_MODE == u8PcuMode)
			{
				SendData->Data.b4ModeControl = 0x7;
//				SendData->Data.b1ModeLed = 0;
//				SendData->Data.b1LiftLed = 1;
//				SendData->Data.b1MoveLedCtr = 1;
//				SendData->Data.b1LiftLedCtl = 1;
			}
			else
			{
				SendData->Data.b4ModeControl = 0;
			}
//			SendData->Data.b4BatterySoc = (u8GetBatterySoc()/15);

			if((1 == sgErrorState.b1Error)&&(1 == sgErrorState.PCUBeep))
			{
				SendData->Data.b1Beep = 1;
			}
			else
			{
				SendData->Data.b1Beep = 0;
			}

			if(0 == u8ErrCodeGet())
			{
				if(0 == sgSwiInput.b1PcuSwi)//�¿���˸CH.
				{
					u8DisplayCnt++;
					if(u8DisplayCnt < 4)
						vPcuNull(SendData);
					else if(u8DisplayCnt < 10)
						vPcuDisplayOrigin(SendData,0x39,0xF6);
					else
					{
						vPcuDisplayOrigin(SendData,0x39,0xF6);
						u8DisplayCnt = 0;
					}
				}
				if(1 == sgActLogic.b1LiftUpAct)
					vPcuDisplayNumber(SendData,gUserInfo.u8LiftSpeed,DISPLAY_NORMAL);
				
				if(1 == sgActLogic.b1LiftDownAct)//�½���ʾE1
					vPcuDisplayNumber(SendData,0xE1,HEX_DISPLAY);				
			}

		}
			break;
		case COMMON_PARASET://��ʾ�������Ⱦ�������
			switch(u8DisplayPage)
			{
				case CP_INIT:
					u8DisplayCnt++;
					if(u8DisplayCnt < 4)
						vPcuNull(SendData);
					else if(u8DisplayCnt < 10)
						vPcuCP(SendData);
					else
					{
						vPcuCP(SendData);
						u8DisplayCnt = 0;
					}
					
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
						u8DisplayPage = CP_LOWBATSLP;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LOWBATALM;
					break;
				case CP_LOWBATSLP://����
					u16PcuParaMonit(SendData,PARA_LowVolShutDownTime,EEPROME_DATA);
					
					if(1 == sgPcuInfo.PcuKeyInfo.b1Enable)//ʹ��->�����ز�ֵ
						u8DisplayPage = CP_MAX_PRESSUREDIFF;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Speaker)//����->��̬����
						u8DisplayPage = CP_DYN_OVERRLOADRATE;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Slow)//����->�����ȶ���ʱ
						u8DisplayPage = CP_DYN_OVERLOADDELAY;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LOWBATALM;
					break;
				case CP_MAX_PRESSUREDIFF://���� ʹ��
					u16PcuParaMonit(SendData,PARA_MaxDifferencePercent,EEPROME_DATA);
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
						u8DisplayPage = CP_LOWBATSLP;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LOWBATALM;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Speaker)//����
						u8DisplayPage = CP_STB_OVERLOADRATE;
					break;
				case CP_DYN_OVERRLOADRATE://���� ����
					u16PcuParaMonit(SendData,PARA_DynamicOverLoadPercent,EEPROME_DATA);
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
						u8DisplayPage = CP_LOWBATSLP;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LOWBATALM;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Enable)//ʹ��
						u8DisplayPage = CP_STB_OVERLOADRATE;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Slow)//����
						u8DisplayPage = CP_ANGLE_UPLIMIT;
					break;
				case CP_DYN_OVERLOADDELAY://���� ����
					u16PcuParaMonit(SendData,PARA_OverLoadStabilityDelay,EEPROME_DATA);
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
						u8DisplayPage = CP_LOWBATSLP;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LOWBATALM;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Speaker)//����
						u8DisplayPage = CP_ANGLE_UPLIMIT;
					break;
				case CP_STB_OVERLOADRATE://���� ���� ʹ��
					u16PcuParaMonit(SendData,PARA_StaticOverLoadPercent,EEPROME_DATA);
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
						u8DisplayPage = CP_LOWBATSLP;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LOWBATALM;				
					if(1 == sgPcuInfo.PcuKeyInfo.b1Slow)//����
						u8DisplayPage = CP_ANTI_DELAYTIME;	
					break;
				case CP_ANGLE_UPLIMIT://���� ���� ����
					u16PcuParaMonit(SendData,PARA_AngleValue0,EEPROME_DATA);
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
						u8DisplayPage = CP_LOWBATSLP;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LOWBATALM;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Enable)//ʹ��
						u8DisplayPage = CP_ANTI_DELAYTIME;	
					break;
				case CP_ANTI_DELAYTIME:// ���� ���� ���� ʹ��
					u16PcuParaMonit(SendData,PARA_AccAndDecAntiPinch,EEPROME_DATA);
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
						u8DisplayPage = CP_LOWBATSLP;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LOWBATALM;
					break;
				case CP_LOWBATALM://����
					u16PcuParaMonit(SendData,PARA_LowVolAlmTime,EEPROME_DATA);
					if(1 == sgPcuInfo.PcuKeyInfo.b1Speaker)//����->������������
						u8DisplayPage = CP_ALARMVOLUE;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Slow)//����->����
						u8DisplayPage = CP_UPCONTROL_SLEEPTIME;
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)//
						u8DisplayPage = CP_LOWBATSLP;
					break;
				case	CP_ALARMVOLUE://���� ����
					u16PcuParaMonit(SendData,PARA_VoiceAlarmVolume,EEPROME_DATA);
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
						u8DisplayPage = CP_LOWBATSLP;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LOWBATALM;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Slow)
						u8DisplayPage = CP_ANGLE_DOWNLIMIT;
					break;				
				case CP_UPCONTROL_SLEEPTIME://���� ����
					u16PcuParaMonit(SendData,PARA_UpperCtlButSleep,EEPROME_DATA);
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
						u8DisplayPage = CP_LOWBATSLP;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LOWBATALM;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Speaker)//����
						u8DisplayPage = CP_ANGLE_DOWNLIMIT;
					break;
				case CP_ANGLE_DOWNLIMIT://���� ���� ����
					u16PcuParaMonit(SendData,PARA_AngleValue1,EEPROME_DATA);
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
						u8DisplayPage = CP_LOWBATSLP;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LOWBATALM;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Enable)//ʹ��
						u8DisplayPage = CP_DOWN_DEC;
					break;
				case CP_DOWN_DEC:// ���� ���� ���� ʹ��
					u16PcuParaMonit(SendData,PARA_BrakeLower,EEPROME_DATA);
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
						u8DisplayPage = CP_LOWBATSLP;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LOWBATALM;
					break;
				default:
					break;
			}
			break;
		case SPEED_PARASET://��ʾPS�����ø����ٶȣ����Ⱦ���,���ҵ�����ֵ��С
			//�Ѿ�����
			switch(u8DisplayPage)
			{
				case PS_INIT://��˸PS
					u8DisplayCnt++;
					if(u8DisplayCnt < 4)
						vPcuNull(SendData);
					else if(u8DisplayCnt < 10)
						vPcuPS(SendData);
					else
					{
						vPcuPS(SendData);
						u8DisplayCnt = 0;
					}
					
					if((1 == sgPcuInfo.PcuKeyInfo.b1LiftMode))
						u8DisplayPage = PS_LIFT;
					break;
				case PS_LIFT://���������ٶ�
					u16PcuParaMonit(SendData,PARA_LiftSpeed,EEPROME_DATA);
			
					if(1 == sgPcuInfo.PcuKeyInfo.b1Enable)//����->�½�������->�����֣�����->
					{
						if(1 == sgPcuInfo.PcuKeyInfo.b1Slow)
							u8DisplayPage =  PS_ANTI;
						else if(1 == sgPcuInfo.PcuKeyInfo.b1Speaker)
							u8DisplayPage =  PS_DOWN;
					}
					else if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage =  PS_FAST;
					break;
				case PS_DOWN:
					u16PcuParaMonit(SendData,PARA_LowerSpeed,EEPROME_DATA);

					if (1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
					{
						u8DisplayPage = PS_LIFT;
					}
					else if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
					{
						u8DisplayPage =  PS_FAST;	
					}
					break;
				case PS_ANTI:
					u16PcuParaMonit(SendData,PARA_BrakeAntiPinch,EEPROME_DATA);		
					if (1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
					{
						u8DisplayPage = PS_LIFT;
					}
					else if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
					{
						u8DisplayPage =  PS_FAST;	
					}			
					break;
				case PS_FAST:
					u16PcuParaMonit(SendData,PARA_FastDriveSpeed,EEPROME_DATA);
					if (1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
					{
						u8DisplayPage = PS_LIFT;
					}
					else if ((1 == sgPcuInfo.PcuKeyInfo.b1Speaker)||(1 == sgPcuInfo.PcuKeyInfo.b1Slow))
					{
						u8SwitchDelay ++;
						if((1 == sgPcuInfo.PcuKeyInfo.b1Speaker)&&(1 == sgPcuInfo.PcuKeyInfo.b1Slow))
						{
							u8DisplayPage = PS_STEER;
							u8SwitchDelay = 0;
						}
						else if (u8SwitchDelay > 20)
						{
							if(1 == sgPcuInfo.PcuKeyInfo.b1Speaker)
							{
								u8DisplayPage = PS_AFTER;
								u8SwitchDelay = 0;
							}
							else if(1 == sgPcuInfo.PcuKeyInfo.b1Slow)
							{
								u8DisplayPage = PS_SLOW;
								u8SwitchDelay = 0;
							}
						}
					}
					break;
				case PS_SLOW:
					u16PcuParaMonit(SendData,PARA_SlowDriveSpeed,EEPROME_DATA);

					if (1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
					{
						u8DisplayPage = PS_LIFT;
					}
					else if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
					{
						u8DisplayPage =  PS_FAST;	
					}						
					break;
				case PS_AFTER:
					u16PcuParaMonit(SendData,PARA_DriveSpeedAfterLift,EEPROME_DATA);
					if (1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
					{
						u8DisplayPage = PS_LIFT;
					}
					else if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
					{
						u8DisplayPage =  PS_FAST;	
					}					
					break;
				case PS_STEER:
					u16PcuParaMonit(SendData,PARA_MaxTurnSpeed,EEPROME_DATA);
					if (1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
					{
						u8DisplayPage = PS_LIFT;
					}
					else if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
					{
						u8DisplayPage =  PS_FAST;	
					}					
					break;
				default:
					break;	
			}
			break;
		case MACHINE_PARASET://SC���������룬�����ľ��������������ȱ���
			//�Ѳ���
			//������-���ݳ������͸�������
			switch(u8DisplayPage)
			{
				case	SC_INIT://��˸SC��
					u8DisplayCnt++;
					if(u8DisplayCnt < 4)
						vPcuNull(SendData);
					else if(u8DisplayCnt < 20)
						vPcuSC(SendData);
					else
					{
						vPcuSC(SendData);
						u8DisplayCnt = 0;
					}
					
					u8CombinedData = gUserInfo.u8VehicleType;

					if((1 == sgPcuInfo.PcuKeyInfo.b1LiftMode))
						u8DisplayPage = SC_SETSL;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = SC_SETSR;
					break;
				case SC_SETSL://�����˸,ÿ�ε���10
					if((0 == sgPcuInfo.PcuKeyInfo.b1TurnLeft)&&(1 == PcuKeyRec2.b1TurnLeft))
					{
						u8CombinedData -= 10;
					}
					
					if((0 == sgPcuInfo.PcuKeyInfo.b1TurnRight)&&(1 == PcuKeyRec2.b1TurnRight))
					{
						u8CombinedData += 10;
					}					
										
					u8DisplayCnt++;
					if(u8DisplayCnt < 8)
						vPcuDisplayNumber(SendData,u8CombinedData,MASK_LEFT);
					else if(u8DisplayCnt < 20)
						vPcuDisplayNumber(SendData,u8CombinedData,DISPLAY_NORMAL);
					else
					{
						vPcuDisplayNumber(SendData,u8CombinedData,DISPLAY_NORMAL);
						u8DisplayCnt = 0;
					}
					
					if(1 == sgPcuInfo.PcuKeyInfo.b1Speaker)//�������ȱ���
					{
						if(u8CombinedData != i32GetPara(PARA_VehicleType))
						{
							u16SaveParaToEeprom(PARA_VehicleType,u8CombinedData);
							i32SetPara(PARA_VehicleType,u8CombinedData);
						}
					}
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = SC_SETSR;
					break;
				case	SC_SETSR://�Ҳ���˸
					if((0 == sgPcuInfo.PcuKeyInfo.b1TurnLeft)&&(1 == PcuKeyRec2.b1TurnLeft))
					{
						u8CombinedData --;
					}
					
					if((0 == sgPcuInfo.PcuKeyInfo.b1TurnRight)&&(1 == PcuKeyRec2.b1TurnRight))
					{
						u8CombinedData ++;
					}					
										
					u8DisplayCnt++;
					if(u8DisplayCnt < 8)
						vPcuDisplayNumber(SendData,u8CombinedData,MASK_RIGHT);
					else if(u8DisplayCnt < 20)
						vPcuDisplayNumber(SendData,u8CombinedData,DISPLAY_NORMAL);
					else
					{
						vPcuDisplayNumber(SendData,u8CombinedData,DISPLAY_NORMAL);
						u8DisplayCnt = 0;
					}
					
					if(1 == sgPcuInfo.PcuKeyInfo.b1Speaker)//�������ȱ���
					{
						if(u8CombinedData != i32GetPara(PARA_VehicleType))
						{
							u16SaveParaToEeprom(PARA_VehicleType,u8CombinedData);
							i32SetPara(PARA_VehicleType,u8CombinedData);
						}
					}
					if((1 == sgPcuInfo.PcuKeyInfo.b1LiftMode))
						u8DisplayPage = SC_SETSL;
					break;
				default:
					break;
			}
			break;
		case FUNCTION_PARASET://H9�����ù������ü��Ƕ�ģ����λ
			{
				if((1 == sgPcuInfo.PcuKeyInfo.b1Slow)&&(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode))//���¹��ٺ�����
				{
					u8DisplayPage = H9_LOW16CHOOSE;
					vKillNetTimer(TIMER_SwitchCheck);
				}
				if((1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)&&(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode))//�������ߺ�����
				{
					u8DisplayPage = H9_HIG16CHOOSE;
					vKillNetTimer(TIMER_SwitchCheck);
				}
				if(1 == sgPcuInfo.PcuKeyInfo.b1Speaker)
				{
					u8DisplayPage = H9_SAVE;
				}
				if((1 == sgPcuInfo.PcuKeyInfo.b1TurnLeft)&&(1 == sgPcuInfo.PcuKeyInfo.b1Enable))//��ת+ʹ�� ��������λ
				{
					u8DisplayPage = H9_UPLIMIT_SAVE;
				}
				if((1 == sgPcuInfo.PcuKeyInfo.b1TurnRight)&&(1 == sgPcuInfo.PcuKeyInfo.b1Enable))//��ת+ʹ�� ��������λ
				{
					u8DisplayPage = H9_DOWNLIMIT_SAVE;
				}
				
				if(0 == sgPcuInfo.PcuKeyInfo.u8data)
				{
					vKillNetTimer(TIMER_SwitchCheck);
				}
				u8DisplayCnt++;
			}
			switch(u8DisplayPage)
			{
				case H9_INIT:
					if(u8DisplayCnt < 4)
						vPcuNull(SendData);
					else if(u8DisplayCnt < 10)
						vPcuH9(SendData);
					else
					{
						vPcuH9(SendData);
						u8DisplayCnt = 0;
					}
					/*H9ѡ��ɸߵ���
					3231null,30�ϵ���ģʽ��29�Ӷ�������28~25������ͣ�
					24δ�궨������λ�ɶ�����23�ر�ˮƽ��������22null��21�Ƕ�ģ����λ��20~17�������ͣ�
					16����˫�غɹ��ܣ�15�������ֹ���ߣ�14�߶�˫�����ܣ�13 12 null��11�����ּ����½���10 18������������ ��9 Ԥ��
					8Ԥ����7�½������ͣ�6�������٣�5�ߵ��ٷ��õ���٣�4null��3���߷���ײ���ܣ�2��������1�޶���10s��˯�ߣ�
					*/
					/*��δʵ�֣�
					30�ϵ���ģʽ-Ĭ������		��δʵ��		ȱ�ٲ���
					24 δ�궨������λ�ɶ���		��δʵ��		ȱ�ٲ���
					20~17��������						������ͻ		��������ͳ�������
					16����˫�غ�							��δʵ��		δ֪����
					15�������ֹ����					��δʵ��		ȱ�ٲ���
					14�߶�˫������					��δʵ��		δ֪����
					11�����ּ����½�					��δʵ��		��������
					10�Ӷ�������������				��δʵ��		ȱ�ٲ���
					7�½�������							��δʵ��		���в���
					6��������								��δʵ��		ȱ�ٲ���
					5�ߵͷ�									��δʵ��		û�ж�Ӧ�ķ�
					3���߷���ײ����					��δʵ��		û��ģ��
					1�޶���˯��							��δʵ��		��ȷ��ʹ�ò���
					*/
					u32CombinedDataH9 =(((i32GetPara(PARA_PitProtectFunc)&0x1)<<28) |(((i32GetPara(PARA_BatteryType))&0b1111)<<24)
					|((i32GetPara(PARA_TiltSwitchSetting)&0x1)<<22)|((i32GetPara(PARA_AngleSimulationLimit)&0x1)<<20)
					|((i32GetPara(PARA_PitProtectFunc)&0x1)<<16)|((i32GetPara(PARA_LowerPumpType)&0x1)<<6)
					|((i32GetPara(PARA_LiftReverseFunc)&0x1)<<1));
//					u32CombinedDataH9 = 0xFFFFFFFF;
					break;
				case H9_LOW16CHOOSE:
					if((0 ==sgPcuInfo.PcuKeyInfo.b1LiftMode)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1Slow))
					{	
						u8CombinedData = 1;//���ô˲�����������
					}
					if((1 == u8CombinedData)
					&&(1 ==sgPcuInfo.PcuKeyInfo.b1LiftMode)
					&&(1 == sgPcuInfo.PcuKeyInfo.b1Slow))
					{
						u8CombinedData = 0;
						u8SubPage++;
						if(u8SubPage >= 4)
							u8SubPage = 0;
					}
					#if 1
					switch(u8SubPage)
					{
						case 0:
							u8DisplayMode1 = HEX_DISPLAY | MASK_LEFT | DOUBLE_DOT ;
							u8DisplayMode2 = HEX_DISPLAY | DOUBLE_DOT ;
							u8SwitchDelay = (u32CombinedDataH9>>8) & 0xFF;//���õ������α�����16λ��8
							break;
						case 1:
							u8DisplayMode1 = HEX_DISPLAY | MASK_RIGHT | DOUBLE_DOT ;
							u8DisplayMode2 = HEX_DISPLAY | DOUBLE_DOT ;
							u8SwitchDelay = (u32CombinedDataH9>>8) & 0xFF;//���õ������α�����16λ��8
							break;
						case 2:
							u8DisplayMode1 = HEX_DISPLAY | MASK_LEFT ;
							u8DisplayMode2 = HEX_DISPLAY  ;
							u8SwitchDelay = (u32CombinedDataH9) & 0xFF;//���õ������α�����16λ��8
							break;
						case 3:
							u8DisplayMode1 = HEX_DISPLAY | MASK_RIGHT ;
							u8DisplayMode2 = HEX_DISPLAY  ;
							u8SwitchDelay = (u32CombinedDataH9) & 0xFF;//���õ������α�����16λ��8
							break;
						default:
							break;
					}
					#endif
					if(u8DisplayCnt < 4)
						vPcuDisplayNumber(SendData,u8SwitchDelay,u8DisplayMode1);
					else if(u8DisplayCnt < 10)
						vPcuDisplayNumber(SendData,u8SwitchDelay,u8DisplayMode2);
					else
					{
						vPcuDisplayNumber(SendData,u8SwitchDelay,u8DisplayMode2);
						u8DisplayCnt = 0;
					}
					if(0 != (u8SubPage % 2))//������4λ
					{
						if(0xF == (u8SwitchDelay & 0xF))//ΪF��1
						{
							if((0 == sgPcuInfo.PcuKeyInfo.b1TurnRight)&&(1 == PcuKeyRec2.b1TurnRight))
							{
								u8SwitchDelay &= 0xF0;
							}
							else if((0 == sgPcuInfo.PcuKeyInfo.b1TurnLeft)&&(1 == PcuKeyRec2.b1TurnLeft))
							{
								u8SwitchDelay--;
							}
						}
						else if(0 == (u8SwitchDelay & 0xF))//Ϊ0��1
						{
							if((0 == sgPcuInfo.PcuKeyInfo.b1TurnLeft)&&(1 == PcuKeyRec2.b1TurnLeft))
							{
								u8SwitchDelay |= 0x0F;
							}
							else if((0 == sgPcuInfo.PcuKeyInfo.b1TurnRight)&&(1 == PcuKeyRec2.b1TurnRight))
							{
								u8SwitchDelay++;
							}
						}
						else
						{
							if((0 == sgPcuInfo.PcuKeyInfo.b1TurnRight)&&(1 == PcuKeyRec2.b1TurnRight))
							{
								u8SwitchDelay++;
							}
							if((0 == sgPcuInfo.PcuKeyInfo.b1TurnLeft)&&(1 == PcuKeyRec2.b1TurnLeft))
							{
								u8SwitchDelay--;
							}
						}
					}
					else //��������λ
					{
						if(0xF0 == (u8SwitchDelay & 0xF0))//ΪF��1
						{
							if((0 == sgPcuInfo.PcuKeyInfo.b1TurnRight)&&(1 == PcuKeyRec2.b1TurnRight))
							{
								u8SwitchDelay &= 0xF;
							}
							else if((0 == sgPcuInfo.PcuKeyInfo.b1TurnLeft)&&(1 == PcuKeyRec2.b1TurnLeft))
							{
								u8SwitchDelay -=0x10;
							}
						}
						else if(0 == (u8SwitchDelay & 0xF0))//Ϊ0��1
						{
							if((0 == sgPcuInfo.PcuKeyInfo.b1TurnLeft)&&(1 == PcuKeyRec2.b1TurnLeft))
							{
								u8SwitchDelay |= 0xF0;
							}
							else if((0 == sgPcuInfo.PcuKeyInfo.b1TurnRight)&&(1 == PcuKeyRec2.b1TurnRight))
							{
								u8SwitchDelay +=0x10;
							}
						}
						else
						{
							if((0 == sgPcuInfo.PcuKeyInfo.b1TurnRight)&&(1 == PcuKeyRec2.b1TurnRight))
							{
								u8SwitchDelay +=0x10;
							}
							if((0 == sgPcuInfo.PcuKeyInfo.b1TurnLeft)&&(1 == PcuKeyRec2.b1TurnLeft))
							{
								u8SwitchDelay -=0x10;
							}
						}
					}
					//��ֵ����u32
					switch(u8SubPage)
					{
						case 0:
						case 1:
							u32CombinedDataH9 = (u32CombinedDataH9 & 0xFFFF00FF)|(u8SwitchDelay<<8);
							break;
						case 2:
						case 3:
							u32CombinedDataH9 = (u32CombinedDataH9 & 0xFFFFFF00)|(u8SwitchDelay);
							break;
						default:
							break;
					}
					break;
				case H9_HIG16CHOOSE:
					if((0 ==sgPcuInfo.PcuKeyInfo.b1LiftMode)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1MoveMode))
					{	
						u8CombinedData = 1;//���ô˲�����������
					}
					if((1 == u8CombinedData)
					&&(1 ==sgPcuInfo.PcuKeyInfo.b1LiftMode)
					&&(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode))
					{
						u8CombinedData = 0;
						u8SubPage++;
						if(u8SubPage >= 4)
							u8SubPage = 0;
					}
					#if 1
					switch(u8SubPage)
					{
						case 0:
							u8DisplayMode1 = HEX_DISPLAY | MASK_LEFT | LEFT_DOT ;
							u8DisplayMode2 = HEX_DISPLAY | LEFT_DOT ;
							u8SwitchDelay = (u32CombinedDataH9>>24) & 0xFF;//���õ������α�����16λ��8
							break;
						case 1:
							u8DisplayMode1 = HEX_DISPLAY | MASK_RIGHT | LEFT_DOT ;
							u8DisplayMode2 = HEX_DISPLAY | LEFT_DOT ;
							u8SwitchDelay = (u32CombinedDataH9>>24) & 0xFF;//���õ������α�����16λ��8
							break;
						case 2:
							u8DisplayMode1 = HEX_DISPLAY | MASK_LEFT | RIGHT_DOT ;
							u8DisplayMode2 = HEX_DISPLAY | RIGHT_DOT ;
							u8SwitchDelay = (u32CombinedDataH9>>16) & 0xFF;//���õ������α�����16λ��8
							break;
						case 3:
							u8DisplayMode1 = HEX_DISPLAY | MASK_RIGHT | RIGHT_DOT ;
							u8DisplayMode2 = HEX_DISPLAY | RIGHT_DOT ;
							u8SwitchDelay = (u32CombinedDataH9>>16) & 0xFF;//���õ������α�����16λ��8
							break;
						default:
							break;
					}
					#endif
					if(u8DisplayCnt < 4)
						vPcuDisplayNumber(SendData,u8SwitchDelay,u8DisplayMode1);
					else if(u8DisplayCnt < 10)
						vPcuDisplayNumber(SendData,u8SwitchDelay,u8DisplayMode2);
					else
					{
						vPcuDisplayNumber(SendData,u8SwitchDelay,u8DisplayMode2);
						u8DisplayCnt = 0;
					}
					if(0 != (u8SubPage % 2))//������4λ
					{
						if(0xF == (u8SwitchDelay & 0xF))//ΪF��1
						{
							if((0 == sgPcuInfo.PcuKeyInfo.b1TurnRight)&&(1 == PcuKeyRec2.b1TurnRight))
							{
								u8SwitchDelay &= 0xF0;
							}
							else if((0 == sgPcuInfo.PcuKeyInfo.b1TurnLeft)&&(1 == PcuKeyRec2.b1TurnLeft))
							{
								u8SwitchDelay--;
							}
						}
						else if(0 == (u8SwitchDelay & 0xF))//Ϊ0��1
						{
							if((0 == sgPcuInfo.PcuKeyInfo.b1TurnLeft)&&(1 == PcuKeyRec2.b1TurnLeft))
							{
								u8SwitchDelay |= 0x0F;
							}
							else if((0 == sgPcuInfo.PcuKeyInfo.b1TurnRight)&&(1 == PcuKeyRec2.b1TurnRight))
							{
								u8SwitchDelay++;
							}
						}
						else
						{
							if((0 == sgPcuInfo.PcuKeyInfo.b1TurnRight)&&(1 == PcuKeyRec2.b1TurnRight))
							{
								u8SwitchDelay++;
							}
							if((0 == sgPcuInfo.PcuKeyInfo.b1TurnLeft)&&(1 == PcuKeyRec2.b1TurnLeft))
							{
								u8SwitchDelay--;
							}
						}
					}
					else //��������λ
					{
						if(0xF0 == (u8SwitchDelay & 0xF0))//ΪF��1
						{
							if((0 == sgPcuInfo.PcuKeyInfo.b1TurnRight)&&(1 == PcuKeyRec2.b1TurnRight))
							{
								u8SwitchDelay &= 0xF;
							}
							else if((0 == sgPcuInfo.PcuKeyInfo.b1TurnLeft)&&(1 == PcuKeyRec2.b1TurnLeft))
							{
								u8SwitchDelay -=0x10;
							}
						}
						else if(0 == (u8SwitchDelay & 0xF0))//Ϊ0��1
						{
							if((0 == sgPcuInfo.PcuKeyInfo.b1TurnLeft)&&(1 == PcuKeyRec2.b1TurnLeft))
							{
								u8SwitchDelay |= 0xF0;
							}
							else if((0 == sgPcuInfo.PcuKeyInfo.b1TurnRight)&&(1 == PcuKeyRec2.b1TurnRight))
							{
								u8SwitchDelay +=0x10;
							}
						}
						else
						{
							if((0 == sgPcuInfo.PcuKeyInfo.b1TurnRight)&&(1 == PcuKeyRec2.b1TurnRight))
							{
								u8SwitchDelay +=0x10;
							}
							if((0 == sgPcuInfo.PcuKeyInfo.b1TurnLeft)&&(1 == PcuKeyRec2.b1TurnLeft))
							{
								u8SwitchDelay -=0x10;
							}
						}
					}
					//��ֵ����u32
					switch(u8SubPage)
					{
						case 0:
						case 1:
							u32CombinedDataH9 = (u32CombinedDataH9 & 0x00FFFFFF)|(u8SwitchDelay<<24);
							break;
						case 2:
						case 3:
							u32CombinedDataH9 = (u32CombinedDataH9 & 0xFF00FFFF)|(u8SwitchDelay<<16);
							break;
						default:
							break;
					}
					break;
				case	H9_SAVE:
					vPcuNull(SendData);//���޸ģ���Ҫ��α���
					u8SaveState += u16SaveParaToEeprom(PARA_PitProtectFunc,((u32CombinedDataH9>>28)& 0x1));
					u8SaveState += u16SaveParaToEeprom(PARA_BatteryType,((u32CombinedDataH9>>24)& 0b1111));
					u8SaveState += u16SaveParaToEeprom(PARA_TiltSwitchSetting,((u32CombinedDataH9>>22)& 0x1));
					u8SaveState += u16SaveParaToEeprom(PARA_AngleSimulationLimit,((u32CombinedDataH9>>20)& 0x1));
					u8SaveState += u16SaveParaToEeprom(PARA_PitProtectFunc,((u32CombinedDataH9>>16)& 0x1));
					u8SaveState += u16SaveParaToEeprom(PARA_LowerPumpType,((u32CombinedDataH9>>6)& 0x1));
					u8SaveState += u16SaveParaToEeprom(PARA_LiftReverseFunc,((u32CombinedDataH9>>1)& 0x1));
					if(0 != u8SaveState)
					{
						u8DisplayPage = H9_SAVEFAIL;
					}
					else
					{
						u8DisplayPage = H9_INIT;
					}
					break;
				case H9_SAVEFAIL: 
					if(u8DisplayCnt < 4)
						vPcuNull(SendData);
					else if(u8DisplayCnt < 10)
						vPcuDisplayOrigin(SendData,0x79,0x50);
					else
					{
						vPcuDisplayOrigin(SendData,0x79,0x50);
						u8DisplayCnt = 0;
					}
					break;
				case H9_UPLIMIT_SAVE:
					if(0 == u16SaveParaToEeprom(PARA_AngleSimulationUpLimit,i32LocalAiGetValue(ANGLE_SENSOR_CHANNEL)))
						u8DisplayPage = H9_INIT;
					break;
				case H9_DOWNLIMIT_SAVE:
					if(0 == u16SaveParaToEeprom(PARA_AngleSimulationDownLimit,i32LocalAiGetValue(ANGLE_SENSOR_CHANNEL)))
						u8DisplayPage = H9_INIT;					
					break;
				default:
					break;
			}
			break;
		case H8_MODE:
			break;
		case H2_MODE:
			break;
		default:
			break;
	}
	PcuKeyRec2.u8data = sgPcuInfo.PcuKeyInfo.u8data;
}

static void vActionProcess()
{
	sgActLogic.u8Data = 0;

	vModeChange();
	switch(u8RunMode)
	{
		case NORMAL_MODE:
			vActionMonitNormal();
			break;
		case PARA_SETS_MODE:
			vActionMonitParaSets();
			break;
		case PRESSURE_CALIBRATION_MODE:
			vActionMonitPressureCalibration();
			break;
		default:
			break;
	}
}


static void vPcuSendProc(xPcuSendPara *SendData)//���͸�pcu�Ļص�����
{
	vPcuParaSetProc(SendData);
}

//����MCUͨ��
static void vMstRevProc(xMstRevPara *RevData)
{
	uint16_t tmp = 0;
	
	if (0 != i32ErrCodeCheck(ErrCode81))
	{
		i32ErrCodeSet(MCU_ECU_COMMUNICATION_ERR);
	}
	if(0 != RevData->b1MainDriver)//���Ӵ���״̬
	{
		
	}
	else
	{
		
	}
	
	if(0 != RevData->b1Ebrake)//ɲ��״̬
	{
		
	}
	else
	{	
		
	}
	
	if(0 != RevData->b1Driver3State)//DO3״̬
	{
		
	}
	else
	{
		
	}
	
	if(0 != RevData->u8ErrCode)
	{
		if (u8ErrCodeGet() < RevData->u8ErrCode)   
		{
			uint8_t i = 0;
			for (i=0; i<RevData->u8ErrCode; i++)
			{
				i32ErrCodeClr(i);
			}
		}
		i32ErrCodeSet(RevData->u8ErrCode - 1);	
	}
	else
	{
		if (u8ErrCodeGet() < 50)				
		{
			uint8_t i = 0;
			for (i=0; i<50; i++)
			{
				i32ErrCodeClr(i);
			}
		}
	}
	
	u16MotorFdb = (uint16_t)(RevData->u8SpeedFdbHigh << 8) | RevData->u8SpeedFdbLow ;

	__disable_irq();

	tmp = u16MotorFdb / gUserInfo.u16RatioOfTransmission;
	/*Motor Speed*/
	__enable_irq();
	
	__disable_irq();
	/*Motor Speed*/
	__enable_irq();
	
	CanSend5A5.u8PhaseCurrentH = RevData->u8CurrentHigh;
	CanSend5A5.u8PhaseCurrentL = RevData->u8CurrentLow;
	
	tmp = RevData->u8MotorTmp - 40;
	CanSend5A5.u8MotorTemperatureL = tmp;
	/*Motor Temp*/

	tmp = RevData->u8BoardTmp - 40;
	CanSend5A5.u8BoardTemperatureL = tmp;
}

//����ƻص�
static void vAlarmLampCallBack(uint8_t u8Flag)
{
	if(1 == u8Flag)
	{
		i32DoPwmSet(BLINK_LED, PUMP_OPEN_PERCENTAGE);
	}
	else
	{
		i32DoPwmSet(BLINK_LED, PUMP_CLOSE_PERCENTAGE);
	}
}

//���������
//Ѹ����ͬ�����ܴ�ȷ�ϣ� ���ȡ�������������ֱ�����������������
/*������*/
// if��enable == ��57�Ų���
static void vBeepCallBack(uint8_t u8Flag)
{
	if((1 == u8Flag))
	{
		i32DoPwmSet(BLINK_BEEP, PUMP_OPEN_PERCENTAGE);
		sgErrorState.PCUBeep = 1;
	}
	else
	{
		i32DoPwmSet(BLINK_BEEP, PUMP_CLOSE_PERCENTAGE);
		sgErrorState.PCUBeep = 0;
	}
}

static void vPressureCallBack(ePressureNo PressureNo)
{
	#if 0
	static xPresureErrInfo LastErrFlag = {0};
	xPresureErrInfo ErrFlag = {0};

	switch ((uint8_t)PressureNo)
	{
		case PressureCaliReverse:
			ErrFlag.b1CaliReverse = 1;
			break;
		case PressureCaliFailure:
			ErrFlag.b1CaliFailure = 1;
			break;
		case PressureWithoutSensor:
//			if (1 == i32LocalDiGet(PIT_SWITCH))	
//			{
				ErrFlag.b1SenSorErr = 1;
//			}
			break;
		case PressureOverPer80:
			ErrFlag.b1Per80Err = 1;
			break;
		case PressureOverPer90:
			ErrFlag.b1Per90Err = 1;
			break;
		case PressureOverPer99:
			ErrFlag.b1Per99Err = 1;
			break;
		case PressureOverPer100:
			ErrFlag.b1Per100Err = 1;
			break;
		default:
			break;
	}
	
	if (LastErrFlag.u8Data != ErrFlag.u8Data)
	{
		if(1 == ErrFlag.b1SenSorErr)
		{
			i32ErrCodeSet(PRESSURE_SENSOR_ERR);
		}
		else
		{
			i32ErrCodeClr(PRESSURE_SENSOR_ERR);
		}
		
		if(1 == ErrFlag.b1CaliReverse)
		{
			i32ErrCodeSet(WEIGHT_CALI_REVESER_ERR);
		}
		else
		{
			i32ErrCodeClr(WEIGHT_CALI_REVESER_ERR);
		}
	
		if(1 == ErrFlag.b1CaliFailure)
		{
			i32ErrCodeSet(CALIBRATION_FAILURE_ERR);
		}
		else
		{
			i32ErrCodeClr(CALIBRATION_FAILURE_ERR);
		}
		
		if(1 == ErrFlag.b1Per80Err)
		{
			i32ErrCodeSet(OVER_80_PER_LOAD_ERR);
		}
		else
		{
			i32ErrCodeClr(OVER_80_PER_LOAD_ERR);
		}
		
		if(1 == ErrFlag.b1Per90Err)
		{
			i32ErrCodeSet(OVER_90_PER_LOAD_ERR);
		}
		else
		{
			i32ErrCodeClr(OVER_90_PER_LOAD_ERR);
		}
		
		if(1 == ErrFlag.b1Per99Err)
		{
			i32ErrCodeSet(OVER_99_PER_LOAD_ERR);
		}
		else
		{
			i32ErrCodeClr(OVER_99_PER_LOAD_ERR);
		}
		
		if(1 == ErrFlag.b1Per100Err)
		{
			i32ErrCodeSet(PLAT_OVERLOAD_ERR);
		}
		else
		{
			i32ErrCodeClr(PLAT_OVERLOAD_ERR);
		}
		LastErrFlag.u8Data = ErrFlag.u8Data;
	}
	#endif
}

static void vAiMonitor(void)
{
	uint16_t u16AdcValue = 0;
	
	u16AdcValue = i32LocalAiGetValue(AI_B_VBUS_CHECK);
	i32SetPara(PARA_BatteryVoltage, u16AdcValue);
}

static void vBatteryManage(void)
{
	uint16_t u16BatteryVoltage = 0;
	if(LiBattery == gUserInfo.u8BatteryType)
	{
		u8Soc = gCanRevPdoInfo.CanRevBMS28A.u8Soc * 0.4;
		u16BatteryVoltage = (gCanRevPdoInfo.CanRveBMS18A.u8BatVoltH<<8) |gCanRevPdoInfo.CanRveBMS18A.u8BatVoltL;
		u16BatteryVoltage /= 10; 
		i32SetPara(PARA_BmsVoltage,u16BatteryVoltage);
		
		if(1 == gCanRevPdoInfo.CanRevBMS28A.b1ErrorOverHeatRate1)
			i32ErrCodeSet(BMS_BATTERY_TEMP_HIGH1_ERR);
		
		if(1 == gCanRevPdoInfo.CanRevBMS28A.b1SingelVoltLow2)
			i32ErrCodeSet(BMS_TOTAL_VOL_LOW2_ERR);

		if(1 == gCanRevPdoInfo.CanRevBMS28A.b1DischargeCurrentHigh1)
			i32ErrCodeSet(BMS_DISCHARGE_CUR_HIGH1_ERR);
		
		if(1 == gCanRevPdoInfo.CanRevBMS28A.b1TotalVoltLow1)
			i32ErrCodeSet(BMS_TOTAL_VOL_LOW1_ERR);
		
		if(1 == gCanRevPdoInfo.CanRevBMS28A.b1SingleVoltLow1)
			i32ErrCodeSet(BMS_SINGLE_VOL_LOW1_ERR);
		
		if(1 == gCanRevPdoInfo.CanRevBMS28A.b1SingelVoltLow2)
			i32ErrCodeSet(BMS_SINGLE_VOL_LOW2_ERR);
		
		if(1 == gCanRevPdoInfo.CanRevBMS28A.b1BatVoltDiffHigh)
			i32ErrCodeSet(BMS_BATTERY_VOL_DIFF_HIGH_ERR);
		
		if(1 == gCanRevPdoInfo.CanRevBMS28A.b1BatTmpDiffHigh)
			i32ErrCodeSet(BMS_BATTERY_TEMP_DIFF2_ERR);
		
		if(1 == gCanRevPdoInfo.CanRevBMS28A.b1DischargeCurrentHigh2)
			i32ErrCodeSet(BMS_DISCHARGE_CUR_HIGH2_ERR);
		
		if(1 == gCanRevPdoInfo.CanRevBMS28A.b1DischargeTmpHigh2)
			i32ErrCodeSet(BMS_DISCHARGE_TEMP_HIGH2_ERR);
		
		if(1 == gCanRevPdoInfo.CanRevBMS28A.b1TotalVoltLow2)
			i32ErrCodeSet(BMS_DISCHARGE_CUR_HIGH1_ERR);
		if(1 == gCanRevPdoInfo.CanRevBMS28A.b1DischargeCurrentHigh1)
			i32ErrCodeSet(BMS_SINGLE_VOL_LOW2_ERR);
	}
	else
	{
		u8Soc = u8GetBatterySoc();
		
	}
	
	if((u8Soc < 10 )||((i32LocalAiGetValue(AI_B_VBUS_CHECK))<19000))
		{
			i32ErrCodeSet(BAT_LOW_CAP2_ERR);
			i32ErrCodeClr(BATTERY_LOW_CAP1_ERR);
		}
		else if(u8Soc < 20)
		{
			i32ErrCodeSet(BATTERY_LOW_CAP1_ERR);
		}
		else
		{
			i32ErrCodeClr(BAT_LOW_CAP2_ERR);
			i32ErrCodeClr(BATTERY_LOW_CAP1_ERR);		
		}

	i32SetPara(PARA_BmsSoc ,u8Soc * 2.5);
	
}
/*******************************************************************************
* Name: void vEcuSetBeepPeriod(void)
* Descriptio: ���������ƺ���
* Input: NULL
* Output: NULL
* checked: 
* to be check:
*******************************************************************************/
static void vEcuSetBeepPeriod(void)
{
	static uint8_t u8Cnt = 0;
	
	if((1 == sgErrorState.b1Error)||(1 == sgErrorState.b1CaliErr))	/*������ڹ��ϵĻ�*/
	{
		vBeepSetPeriod(60);
		vBeepSetOpenePeriod(120);
	}
	else if((1 == sgErrorState.b1Warning)
			||(1 == sgErrorState.b1CaliInit)
			||(1 == sgErrorState.b1CaliEnd))	/*����*/
	{
		vBeepSetPeriod(180);
		vBeepSetOpenePeriod(240);
	}
	else if(((ABOVE_SWI != u8AntiPinchState)&&(UNDER_SWI_DELAY != u8AntiPinchState))
		||(true == u8GetNetTimerStartFlag(TIMER_EcuAntiPinchFunc)))		/*������*/
	{
		u8Cnt++;
		if(u8Cnt >= 180)
		{
			u8Cnt = 0;
		}
		if(u8Cnt < 144)
		{
			vBeepSetPeriod(240);
			vBeepSetOpenePeriod(480);
		}
		else
		{
			vBeepSetOpenePeriod(0);
			vBeepSetPeriod(0);
		}
	}
	else if((0 != sgActLogic.u8Data) && ( FunctionEnable == gUserInfo.u8ActAlmFunc )&&(u8RunMode == NORMAL_MODE))		/*��������*/
	{
		vBeepSetPeriod(30);
		vBeepSetOpenePeriod(120);
	}
	else
	{
		vBeepSetOpenePeriod(0);
		vBeepSetPeriod(0);
	}
}

static void vErrProc(void)
{
	uint8_t u8ErrCode = 0;
	
	u8ErrCode = u8ErrCodeGet();
	if(0 != u8ErrCode)
	{
		sgErrorState.b1Error = 1;
		
		vLedSendAlmCode(u8ErrCode);
		
		u8ErrCode = u8ErrCodeGetTrans();
		__disable_irq();
		__enable_irq();
	}
	else
	{
		sgErrorState.b1Error = 0;
		__disable_irq();
		__enable_irq();
	}
	
	if(0 != (u8ErrCodeGetAbnormal(ABNORMAL_NoAct)))
		sgLimit.b1NoAct = 1;
	else
		sgLimit.b1NoAct = 0;
	
		/*Action Limit */
	if(0 != sgLimit.b1NoAct)
	{
		sgLimit.b1NoDown = 1;
		sgLimit.b1NoLift = 1;
		sgLimit.b1NoMove = 1;
		sgLimit.b1NoPcu = 1;
		sgLimit.b1NoTurn = 1;
	}
	else 
	{
		if(0 != (u8ErrCodeGetAbnormal(ABNORMAL_NoPcu)))
			sgLimit.b1NoPcu = 1;
		else
			sgLimit.b1NoPcu = 0;
		
		if((0 != (u8ErrCodeGetAbnormal(ABNORMAL_NoUp)))||(1 == sgSwiInput.b1UpLimitSwi))
			sgLimit.b1NoLift = 1;
		else
			sgLimit.b1NoLift = 0;
		
		if(0 != (u8ErrCodeGetAbnormal(ABNORMAL_NoDown)))
			sgLimit.b1NoDown = 1;
		else
			sgLimit.b1NoDown = 0;	

		if((0 != (u8ErrCodeGetAbnormal(ABNORMAL_NoTurnLeft)))||(0 != (u8ErrCodeGetAbnormal(ABNORMAL_NoTurnRight))))
			sgLimit.b1NoTurn = 1;
		else
			sgLimit.b1NoTurn = 0;

		if((0 != (u8ErrCodeGetAbnormal(ABNORMAL_NoForWard)))||(0 != (u8ErrCodeGetAbnormal(ABNORMAL_NoBackWard))))
			sgLimit.b1NoMove = 1;
		else
			sgLimit.b1NoMove = 0;

		if((0 != (u8ErrCodeGetAbnormal(ABNORMAL_Gear1Spd)))||(1 == sgSwiInput.b1PitSwi))
			sgLimit.b1SpeedAfterLift = 1;
		else
			sgLimit.b1SpeedAfterLift = 0;		
	}
//		i32SetPara(PARA_LiftValveCurrent, sgLimit.u8Data);		/*Send Pump Value*/
	/*action limit and beep sets*/
}



static void vMoveAccParaSets2(void)
{
	u16Time = gUserInfo.u8AccAndDecFastDrive ;
	if((1 == sgLimit.b1SpeedAfterLift)&&(u16Time < gUserInfo.u8AccAndDecAfterLift))
	{
		u16Time = gUserInfo.u8AccAndDecAfterLift;
	}
	else if((1 == sgLimit.b1Slow)&&(u16Time < gUserInfo.u8AccAndDecSlowDrive))
	{
		u16Time = gUserInfo.u8AccAndDecSlowDrive;
	}
	else if((0 != sgActLogic.b1TurnLeft + sgActLogic.b1TurnRight)&&(u16Time < gUserInfo.u8AccAndDecTurn))
	{
		u16Time = gUserInfo.u8AccAndDecTurn;
	}
}


static void vMoveStableParaSets2(void)
{
	u16Time = gUserInfo.u8CurveFastDrive ;
	if((1 == sgLimit.b1SpeedAfterLift)&&(u16Time < gUserInfo.u8CurveDriveAfterLift))
	{
		u16Time = gUserInfo.u8CurveDriveAfterLift;
	}
	else if((1 == sgLimit.b1Slow)&&(u16Time < gUserInfo.u8CurveSlowDrive))
	{
		u16Time = gUserInfo.u8CurveSlowDrive;
	}
	else if((0 != sgActLogic.b1TurnLeft + sgActLogic.b1TurnRight)&&(u16Time < gUserInfo.u8CurveTurn))
	{
		u16Time = gUserInfo.u8CurveTurn;
	}
}

static void vMoveDecParaSets2(void)
{
	u16Time = gUserInfo.u8BrakeFastDrive ;
	if((1 ==  sgLimit.b1SpeedAfterLift)&&(u16Time < gUserInfo.u8BrakeDriveAfterLift))
	{
		u16Time = gUserInfo.u8BrakeDriveAfterLift;
	}
	else if((1 == sgLimit.b1Slow)&&(u16Time < gUserInfo.u8BrakeSlowDrive))
	{
		u16Time = gUserInfo.u8BrakeSlowDrive;
	}
	else if((0 != sgActLogic.b1TurnLeft + sgActLogic.b1TurnRight)&&(u16Time < gUserInfo.u8BrakeTurn))
	{
		u16Time = gUserInfo.u8BrakeTurn;
	}
}

//�����ٶȱ��ʴ���ǰ��Ŀ���ٶȣ�ָ���ٶȣ�����ʱ�䣬����һ�������������ٳ��Զ�Ӧ�����ļӼ��ٱ�������
static void vSpeedControl2()
{
	static float fStepFactor = 0;//ʵ��ִ�еļӼ���
	static float fFactorA = 0;
	static float fFactorB = 0;
	static float fFactorC = 0;
	static float fStepRate = 0;//���ԼӼ���ʱ���Ӽ�����
	static uint16_t u16SpeedTargetRec = 0;
	
	static uint8_t u8AccFlag = 0;
	static uint16_t u16Timecnt = 0;//���ڼ��㲽����ʱ�����
	static uint16_t u16TimeRange = 0;
	
	static uint8_t u8IncreaseDecreaseFlag = 0;

	#define STABLE_STATE		0
	#define INCREASE_STATE	1
	#define DECREASE_STATE	2
	
	
	
	#define ACC_STATIC	0		//Ĭ��ֵ��ָ���ٶ��ȶ��򲨶��Ƚ�С��
	#define ACC_FRESH		1  	//�Ӽ��ٲ�������
	#define ACC_EXCUTE	2		//�Ӽ���ִ��
	#define RVS_FRESH		3		//ָ���ʱ��������
	#define	RVS_EXCUTE	4		//ָ���ʱ�����ٲ����������
	
	switch(u8AccFlag)
	{
		case ACC_STATIC://��λʱ����ָ��ֵ�����С��ֵ������ֵ
			fStepFactor = 0;
			if((u16SpeedTarget != u16SpeedCmd)||(u16TimeRange != u16Time))
				u8AccFlag = ACC_FRESH;
			break;
		case ACC_FRESH://��������,ϵ��ֵABC��Ŀ���ٶȣ�u16SpeedTarget - u16SpeedCmd
			fStepRate = 4064 / u16Time;
			fFactorA = (3 * fStepFactor * u16Time - 6 * abs(u16SpeedCmd - u16SpeedTarget ))/(pow(u16Time,3));
			fFactorB = (6 * abs(u16SpeedCmd - u16SpeedTarget ) - 4 * fStepFactor * u16Time)/(u16Time * u16Time);
			fFactorC = fStepFactor;
			u16SpeedTargetRec = u16SpeedTarget;
			u16TimeRange = u16Time;
			u16Timecnt = 0;
			u8AccFlag = ACC_EXCUTE;
			if(u16SpeedTarget > u16SpeedCmd)
			{
				u8IncreaseDecreaseFlag = INCREASE_STATE;
			}
			else
			{
				u8IncreaseDecreaseFlag = DECREASE_STATE;
			}
			break;
		case ACC_EXCUTE://Ŀ���ٶȱ仯������ִ�У��仯�������¼������,�ٶȱƽ���ص���ʼ״̬
			fStepFactor = fFactorA * u16Timecnt * u16Timecnt + fFactorB * u16Timecnt + fFactorC;
			if(u16Timecnt<u16TimeRange)
				u16Timecnt ++;

			if(fStepFactor < (fStepRate / 100))//��Сб�ʲ�����״̬�¼��ٶȵ�1/10
			{
				fStepFactor = fStepRate / 100;
			}					

			if((u16SpeedTargetRec == u16SpeedTarget))//ָ��仯С����ֵ
			{
				if(u16SpeedCmd < u16SpeedTarget)
				{
					if(u16Timecnt<u16TimeRange)
					{
						u16SpeedCmd += fStepFactor ;
					}
					else//�������
					{
						u16SpeedCmd = u16SpeedTarget ;
						u8AccFlag = ACC_STATIC;
					}
				}
				else 
				{
					if((u16SpeedCmd - u16SpeedTarget)>fStepFactor)
					{
						u16SpeedCmd -= fStepFactor ;
					}
					else//�������
					{
						u16SpeedCmd = u16SpeedTarget ;
						u8AccFlag = ACC_STATIC;
					}
				}
			}
			else if(((u16SpeedCmd > u16SpeedTarget)&&(INCREASE_STATE == u8IncreaseDecreaseFlag ))
				||((u16SpeedCmd < u16SpeedTarget)&&(DECREASE_STATE == u8IncreaseDecreaseFlag )))//����ʱָ����ˣ������ʱָ��������
			{
				u8AccFlag = RVS_FRESH;
			}
			else//ָ��仯������ֵ�����¼���
			{
				u8AccFlag = ACC_FRESH;
			}	
			break;
		case RVS_FRESH:
			fFactorA = fStepFactor / (u16TimeRange );
			fFactorB = fStepFactor;
			u16Timecnt = 0;
			u8AccFlag = RVS_EXCUTE;
		{
			u8AccFlag = ACC_FRESH;
		}
			break;
		case RVS_EXCUTE:
			fStepFactor =  - fFactorA *u16Timecnt + fFactorB ;
			if(((u16SpeedCmd < u16SpeedTarget)&&(INCREASE_STATE == u8IncreaseDecreaseFlag ))
					||((u16SpeedCmd > u16SpeedTarget)&&(DECREASE_STATE == u8IncreaseDecreaseFlag )))//��������лָ�ǰ��?
			{
				u8AccFlag = ACC_STATIC;
			}
		
			if(u16SpeedTarget > u16SpeedCmd)
			{
				u8IncreaseDecreaseFlag = INCREASE_STATE;
			}
			else
			{
				u8IncreaseDecreaseFlag = DECREASE_STATE;
			}
			if(u16Timecnt < u16TimeRange )
				u16Timecnt ++;
			
			if(fStepFactor < (fStepRate / 10))//���ٲ���������ֵ���ع�Ĭ��״̬
			{
				fStepFactor = fStepRate / 10;
				u8AccFlag = ACC_STATIC;
			}
			else
			{
				if(INCREASE_STATE == u8IncreaseDecreaseFlag)
				{
					u16SpeedCmd += fStepFactor ;
				}
				else 
				{
					u16SpeedCmd -= fStepFactor ;
				}
			}
			break;
		default :
			u8AccFlag = ACC_STATIC;
			break;
	}	
}

static void vMoveAccParaSets()
{
	vMoveAccParaSets2();
	u16Time = u16Time * TIME_FACTOR;
}
static void vMoveStableParaSets()
{	
	vMoveStableParaSets2();
	u16Time = u16Time * TIME_FACTOR;
}

static void vMoveDecParaSets()
{
	vMoveDecParaSets2();
	u16Time = u16Time * TIME_FACTOR;
}
static void vSpeedControl()
{
	vSpeedControl2();
}

/*�ٶȵ��ں���*/
static int32_t i32SpeedAdjust(int16_t Adj_Speedcmd ,int16_t Adj_Speedmax,
	uint8_t Adj_Steptime,uint8_t Adj_Brakespd, uint8_t Adj_Accspd)
{//�ֱ�ָ��4096���ٶȵ�λ100���Ӽ�������255��ɲ�����ٶ�255�����ټ��ٶ�255
	static int32_t Adj_Speedold = 0;//��¼ֵ�����ڱȽ�
	static uint8_t Adj_Stepcnt = 0;//
	static int32_t Adj_Speedmaxold = 0;
	int32_t Adj_Steplength = 0;
	uint8_t Adj_Stepodd = 0;
	

	Adj_Speedcmd = Adj_Speedcmd * Adj_Speedmax / 100;//�ֱ�ָ��ɶ�Ӧ�ٶȱ���
	Adj_Speedmax = Adj_Speedmax * 4096 / 100;	//����ٶ�ָ��ӳ�䵽4096
	
	if(Adj_Speedmax != Adj_Speedmaxold)//��������
	{
		Adj_Speedmaxold = Adj_Speedmax;
		Adj_Stepcnt = 0;
	}
	
	if((0 == Adj_Speedcmd)&&(0 != Adj_Speedold))//ɲ��
	{
		Adj_Steplength = Adj_Speedmax * Adj_Brakespd / 255 / Adj_Steptime ;
		Adj_Stepodd = (Adj_Speedmax * Adj_Brakespd /255)% Adj_Steptime;
	}
	else if(0 != Adj_Steptime)
	{
		Adj_Steplength = Adj_Speedmax * Adj_Accspd /255/ Adj_Steptime ;
		Adj_Stepodd = (Adj_Speedmax * Adj_Accspd /255)% Adj_Steptime;
	}
	else//���������
	{
		
	}
	
//	i32LogWrite(INFO,"spd = %d \r \n",Adj_Steplength);
	
	if((abs(Adj_Speedcmd - Adj_Speedold)<(Adj_Speedmax/3))
		||((abs(Adj_Speedcmd)<(Adj_Speedmax /4))&&(0 != Adj_Speedcmd)))//�ֶε���,�����ʡ��Ͳ���ʱ��Ӧ����С
	{
		Adj_Steplength = Adj_Steplength / 2;
	}
	
	//i32LogWrite(INFO,"spdadj = %d \r \n",Adj_Steplength);
	
	if(0 != Adj_Steptime)
	{
		if(Adj_Stepcnt < Adj_Stepodd)
		{
			Adj_Stepcnt ++ ;
			if(Adj_Speedcmd < Adj_Speedold)
			{
				if((abs(Adj_Speedcmd - Adj_Speedold))>Adj_Steplength)
				{
					Adj_Speedcmd = Adj_Speedold - Adj_Steplength - 1;
				}
					
			}
			else if(Adj_Speedcmd > Adj_Speedold)
			{
				if((abs(Adj_Speedcmd - Adj_Speedold))>Adj_Steplength)
				{
					Adj_Speedcmd = Adj_Speedold + Adj_Steplength + 1;
				}
			}
		}
		else if(Adj_Stepcnt < Adj_Steplength)
		{
			Adj_Stepcnt ++ ;
			if(Adj_Speedcmd < Adj_Speedold)
			{
				if((abs(Adj_Speedcmd - Adj_Speedold))>Adj_Steplength)
				{
					Adj_Speedcmd = Adj_Speedold - Adj_Steplength ;
				}
					
			}
			else if(Adj_Speedcmd > Adj_Speedold)
			{
				if((abs(Adj_Speedcmd - Adj_Speedold))>Adj_Steplength)
				{
					Adj_Speedcmd = Adj_Speedold + Adj_Steplength ;
				}
			}
		}
		else
		{
			Adj_Stepcnt = 0 ;
			if(Adj_Speedcmd < Adj_Speedold)
			{
				if((abs(Adj_Speedcmd - Adj_Speedold))>Adj_Steplength)
				{
					Adj_Speedcmd = Adj_Speedold - Adj_Steplength ;
				}
			}
			else if(Adj_Speedcmd > Adj_Speedold)
			{
				if((abs(Adj_Speedcmd - Adj_Speedold))>Adj_Steplength)
				{
					Adj_Speedcmd = Adj_Speedold + Adj_Steplength ;
				}
			}
		}
	}

	Adj_Speedold = Adj_Speedcmd;
	
	return Adj_Speedcmd;
	
}

static void vAccNormal(xMstSendPara *SendData)
{
	/*���ٲ���*/
	static int32_t i32Maxspeedrate = 100;//��ʼ��Ϊ100
	static uint8_t u8Steptime = 0;
	static int32_t i32Brakespd = 0;
	static int32_t i32Accspd = 0;
	static int8_t i8StateRecord = 0;//����ת������
	
	int16_t i16Tmp = 0;
	
	int16_t i16Spd = 0;
	uint8_t	u8Pump=0;
	int32_t i32Prop = 0;
	uint8_t u8MotionState = 0;
	
	//
	SendData->buf[2] = 0;
	
	/*
	input;
	sgActLogic;�����жϵĶ����߼���ǰ������	�����½���ת��ת
	u16MotorVal;�����жϵ�ָ���ٶ� ��Χ0��4064
	
	
	output;
	i32DoPwmSet(CHANNEL,OPEN/CLOES);//���ڴ򿪡��رն�Ӧ�Ŀ��ط�����ض���������ǵùر�
	vPropSetTarget(CHANNEL,i32prop);//������ר�ã����������������ǵö�Ӧһ��ͨ����û����ʱ������ֵ��Ϊ0
	
	����������ר��,��Ӧ��ͨ��Ҫ�Ͳ���ƥ���ϡ�ͨ��0�ò���0��ͨ��1�ò���1��
	�����߼�����Сֵ + �����ֵ-��Сֵ��*��ָ���ٶȵİٷֱȣ�*����������ϵ��
	i32prop = _IQ((sgUserInfo.fPropMinCurrent1 + (sgUserInfo.fPropMaxCurrent1 - sgUserInfo.fPropMinCurrent1) * u16SpeedCmd / MOTOR_MAX_SPEED_VALUE) / PROPD_STD_CURRENT);
	
	SendData->u8PumpTarget;�ͱ�ת��ָ���Χ0��255����Ӧ0��100%
	SendData->u8TargetHigh;SendData->u8TargetLow;�������ߵ�����ٶȡ���Χ0��4064����Ӧ0��100%
	SendDataǰ�������ˡ�������ʹ��ָ��ο�һ����ǰ����
	*/
	
	/*
	�����߼���
	����ģʽ��ǰ��������ʹ��ָ�ǰ��ָ������ת�٣�������ǰ��ָ�Ϊ����ָ��
	����ģʽ��������תָ������ͱ�ת�١�����ָ��򿪶�Ӧ����ת����ת����ָ����ʧ֮��ǵùط��������ָ�����
	
	����ģʽ���������������������ͱ�ת�١�����ָ�����������ָ�������ر���������
	����ģʽ���½���ֻ��򿪱�������
	
	��Ҫע��رշ���ʱ������ֱ���ĵ��û�б������������з����ٶȣ�������صĿ죬���ת�ٻ�û��������ʹ�ø��ܻ�Ƚϳ塣
	�Ӽ��ٲ�������λ����1��38��������
	��λ����������gUserInfo�У����һ��������ļӼ��ٷ����ɣ���
	*/
	

	int32_t i32PropMax ;
	i32PropMax = _IQ(PROPD_MAX_CURRENT * i32GetPara(PARA_LowerSpeed)/100 / PROPD_STD_CURRENT);
	
	
	/*ת�򿪹ط�����*/
	if(1 ==  sgActLogic.b1TurnLeft)
	{
		i32DoPwmSet(TURNLEFT_PUMP, PUMP_OPEN_PERCENTAGE);
		i32DoPwmSet(TURNRIGHT_PUMP, PUMP_CLOSE_PERCENTAGE);
		i32SetPara(PARA_TurnLeftValveCurrent, i32GetPara(PARA_ValueOpenLoopCurrent));
		i32SetPara(PARA_TurnRightValveCurrent, 0);
	}
	else if(1 == sgActLogic.b1TurnRight)
	{
		i32DoPwmSet(TURNLEFT_PUMP, PUMP_CLOSE_PERCENTAGE);
		i32DoPwmSet(TURNRIGHT_PUMP, PUMP_OPEN_PERCENTAGE);
		i32SetPara(PARA_TurnRightValveCurrent, i32GetPara(PARA_ValueOpenLoopCurrent));
		i32SetPara(PARA_TurnLeftValveCurrent, 0);
	}
	else
	{
		i32DoPwmSet(TURNLEFT_PUMP, PUMP_CLOSE_PERCENTAGE);
		i32DoPwmSet(TURNRIGHT_PUMP, PUMP_CLOSE_PERCENTAGE);
		i32SetPara(PARA_TurnLeftValveCurrent, 0);
		i32SetPara(PARA_TurnRightValveCurrent, 0);
	}
	
	/*�����ٶȴ���*/
	if((1 == sgActLogic.b1ForwardAct) || (1 == sgActLogic.b1BackwardAct))//����ʹ��
	{
		//�����ٶ�
			i32Maxspeedrate = i32GetPara(PARA_FastDriveSpeed);
			u8Steptime =  i32GetPara(PARA_AccAndDecFastDrive);
			i32Brakespd = i32GetPara(PARA_BrakeFastDrive)  ;
			i32Accspd = i32GetPara(PARA_CurveFastDrive)  ;
			
		if (1 == sgActLogic.b1ForwardAct)
			u8MotionState = 1;
		else if (1 == sgActLogic.b1BackwardAct)
			u8MotionState = 2;
		
		
		
		if(1 == sgLimit.b1SpeedAfterLift)//�������ٶ�
		{
			if(i32Maxspeedrate > i32GetPara(PARA_DriveSpeedAfterLift))
				i32Maxspeedrate = i32GetPara(PARA_DriveSpeedAfterLift);
			u8Steptime =  i32GetPara(PARA_AccAndDecAfterLift);
			i32Brakespd = i32GetPara(PARA_BrakeDriveAfterLift)  ;
			i32Accspd = i32GetPara(PARA_CurveDriveAfterLift)  ;
		}

	 if(1 == sgLimit.b1Slow)//����
		{
			if(i32Maxspeedrate > i32GetPara(PARA_SlowDriveSpeed))
				i32Maxspeedrate = i32GetPara(PARA_SlowDriveSpeed);
			u8Steptime =  i32GetPara(PARA_AccAndDecSlowDrive);
			i32Brakespd = i32GetPara(PARA_BrakeSlowDrive)  ;
			i32Accspd = i32GetPara(PARA_CurveSlowDrive)  ;
		}

		if((1 == sgActLogic.b1TurnLeft)
				||(1 == sgActLogic.b1TurnRight))//ת��
		{
			if(i32Maxspeedrate > i32GetPara(PARA_MaxTurnSpeed))
				i32Maxspeedrate = i32GetPara(PARA_MaxTurnSpeed) ;
			u8Steptime =  i32GetPara(PARA_AccAndDecTurn);
			i32Brakespd = i32GetPara(PARA_BrakeTurn)  ;
			i32Accspd = i32GetPara(PARA_CurveTurn)  ;
		}

		/*���߼Ӽ��ٵ���*/
		i16Spd = i32SpeedAdjust(u16MotorVal,i32Maxspeedrate,
							u8Steptime,i32Brakespd,i32Accspd);
	}
	else if(u16MotorFdb>MOTOR_FDB_CLOSE_VALUE)//ɲ������
	{
		i16Spd = i32SpeedAdjust(0,i32Maxspeedrate,
							u8Steptime,i32Brakespd,i32Accspd);
	}
	else
	{
		i16Spd = 0;
		u8MotionState = 0;
	}
		

	if (i16Spd >= MOTOR_MAX_SPEED_VALUE)
	{
		i16Spd = MOTOR_MAX_SPEED_VALUE;
	}
		/*�ͱ�����ָ��*/
	if(1==sgActLogic.b1LiftUpAct)
	{
		i16Spd = 0;
		i32Maxspeedrate = i32GetPara(PARA_LiftSpeed);
		u8Steptime =  i32GetPara(PARA_AccAndDecLift);
		i32Brakespd = i32GetPara(PARA_BrakeLift) ;
		i32Accspd = i32GetPara(PARA_CurveLift) ;
		if(u16MotorVal>=0)
			u8Pump = (uint8_t)(i32SpeedAdjust(u16MotorVal,i32Maxspeedrate,
				u8Steptime,i32Brakespd,i32Accspd)/16);/*���ŵ�256*/
		u8PumpFeedback = u8Pump ;
	}
	else if(u8PumpFeedback > PUMP_FDB_CLOSE_VALUE)//ɲ������
	{
		u8Pump = (uint8_t)(i32SpeedAdjust(0,i32Maxspeedrate,
			u8Steptime,i32Brakespd,i32Accspd)/16);/*���ŵ�256*/
		u8PumpFeedback = u8Pump ;
	}
	else if((1 == sgActLogic.b1TurnLeft)||(1 == sgActLogic.b1TurnRight))
	{
		u8Pump = i32GetPara(PARA_TurnPowerLimit) * 255 * 2 / 100;
		u8PumpFeedback = 0 ;//ת��ʱ�����뷴��
	}
	else
	{
		u8PumpFeedback = 0 ;
		u8Pump = 0;
	}
	
	
	/*�½�����*/
	if(1 == sgActLogic.b1LiftDownAct)
	{
		
		i32Prop = _IQ(gUserInfo.fPropMaxCurrent1);
//		i32Prop = _IQ((gUserInfo.fPropMinCurrent1 + (gUserInfo.fPropMaxCurrent1 - gUserInfo.fPropMinCurrent1) *
//						u16MotorVal / MOTOR_MAX_SPEED_VALUE) / PROPD_STD_CURRENT);
		//i32LogWrite(INFO, LOG_USER, "Prop= %d, MotorVal= %d \r\n",i32Prop, u16MotorVal);
 
		i32SetPara(PARA_PropValveCurrent, (inserted_data[1] * 3300 * 7.5 / 4096 / (7.5 + 68) / 0.05));
	}
	else if(i32PropFeedBack>PROP_FDB_CLOSE_VALUE)//�ӳٹر�
	{
		i32Prop = 0;
		i32SetPara(PARA_PropValveCurrent, (inserted_data[1] * 3300 * 7.5 / 4096 / (7.5 + 68) / 0.05));		
	}
	else
	{
		i32Prop = 0;
		i32SetPara(PARA_PropValveCurrent, (inserted_data[1] * 3300 * 7.5 / 4096 / (7.5 + 68) / 0.05));
		//i32SetPara(PARA_PropValveCurrent, 0);
//		sgSpeakerFun.b1240PerMin = 0;
	}
	

	if(((1==sgActLogic.b1LiftUpAct)
		||(u8PumpFeedback > PUMP_FDB_CLOSE_VALUE))
				&&((0 == sgActLogic.b1TurnLeft)
					&&(0 == sgActLogic.b1TurnRight)))
	{
		i32DoPwmSet(LIFTUP_PUMP, PUMP_OPEN_PERCENTAGE);
		i32SetPara(PARA_LiftValveCurrent, i32GetPara(PARA_ValueOpenLoopCurrent));
		
	}
	else
	{
		i32DoPwmSet(LIFTUP_PUMP, PUMP_CLOSE_PERCENTAGE);
		i32SetPara(PARA_LiftValveCurrent, 0);
	}
	
	
	/*ָ���*/
	SendData->u8TargetHigh = i16Spd >> 8;
	SendData->u8TargetLow = i16Spd;
	SendData->u8PumpTarget = u8Pump;
	
	if(u8Pump>0)
	{
		SendData->b1LiftReq = 1;
	}
	if (1 == sgActLogic.b1ForwardAct || 1 == u8MotionState)
	{
		SendData->b1ForwardReq = 1;
	}
	else if(1 == sgActLogic.b1BackwardAct || 2 == u8MotionState)
	{
		SendData->b1BackwardReq = 1;
	}
	
	if((abs(i16Spd)>0)||(u8Pump>0))
	//if ((abs(i16Spd)>0) || (1==sgActLogic.b1LiftUpAct) || (1 == sgActLogic.b1TurnLeft)||(1 == sgActLogic.b1TurnRight))
	{
		SendData->b1ServoOn = 1;
	}
	
	vPropSetTarget(LIFTDOWN_PUMP2, i32Prop);
	i32SetPara(PARA_ForwardValveCurrent,i32Prop);
		
	i16SpeedFeedback = i16Spd;
	
	i32PropFeedBack = inserted_data[1] * 3300 * 7.5 / 4096 / (7.5 + 68) / 0.05;
	
}
//���������ƺ�CV����
static void vAccParaSets(xMstSendPara *SendData)
{
	
}
static void vAccPressureCalibration(xMstSendPara *SendData)
{
	if(1 == sgActLogic.b1LiftDownAct)
	{
		int32_t i32prop = 0;
		i32DoPwmSet(LIFTUP_PUMP,PUMP_CLOSE_PERCENTAGE);
		i32prop = _IQ((gUserInfo.fPropMinCurrent1 + (gUserInfo.fPropMaxCurrent1 - gUserInfo.fPropMinCurrent1) * u16MotorVal *gUserInfo.u8LowerSpeed /100 / MOTOR_MAX_SPEED_VALUE) / PROPD_STD_CURRENT);
		vPropSetTarget(LIFTDOWN_PUMP1, i32prop);
		vPropSetTarget(LIFTDOWN_PUMP2, i32prop);
		SendData->b1ForwardReq = 0;
		SendData->b1ServoOn = 0;
		SendData->u8TargetHigh = 0;
		SendData->u8TargetLow = 0;
	}
	else	if(1 == sgActLogic.b1LiftUpAct)
	{
		i32DoPwmSet(LIFTUP_PUMP,PUMP_OPEN_PERCENTAGE);
		vPropSetTarget(LIFTDOWN_PUMP1, 0);
		vPropSetTarget(LIFTDOWN_PUMP2, 0);
		SendData->b1ForwardReq = 1;
		SendData->b1ServoOn = 1;
		SendData->u8TargetHigh = (u16MotorVal * gUserInfo.u8LiftSpeed / 100) >> 8;
		SendData->u8TargetLow =(u16MotorVal * gUserInfo.u8LiftSpeed / 100);
	}
	else
	{
		i32DoPwmSet(LIFTUP_PUMP,PUMP_CLOSE_PERCENTAGE);
		vPropSetTarget(LIFTDOWN_PUMP1, 0);
		vPropSetTarget(LIFTDOWN_PUMP2, 0);
		SendData->b1ForwardReq = 0;
		SendData->b1ServoOn = 0;
		SendData->u8TargetHigh = 0;
		SendData->u8TargetLow = 0;
	}
}
static void vAccProcess(xMstSendPara *SendData)
{
	switch(u8RunMode)
	{
		case NORMAL_MODE:
			vAccNormal(SendData);
			break;
		case PARA_SETS_MODE:
			vAccParaSets(SendData);
			break;
		case PRESSURE_CALIBRATION_MODE:
			vAccPressureCalibration(SendData);
			break;
		default:
			break;
	}
}

static void vMstSendProc(xMstSendPara *SendData) 
{
	SendData->buf[2] = 0;
	vAccProcess(SendData);
}
/*������*/
//﮵硢�¿ص��߱���
void vCanLostProc(uint32_t u32CanID,uint8_t u8State)
{
	switch(u32CanID)
	{
		case 0x28A://﮵籨�ģ�ֻ����﮵�ģʽ�¡����Ķ�ʧ�ű���
			if(u8State == CAN_LOST)
				{
					u8LiCheckFlag = 1;
				}
				if (CAN_NORMAL == u8State)
				{
					 u8LiCheckFlag = 0;
				}
				
			break;
//		case :
//			break;
//		case :
//			break;
		case 0x726:
		{
				if(u8State == CAN_LOST)
				{
					u8HmiCheckFlag = 1;
				}
				if (CAN_NORMAL == u8State)
				{
					 u8HmiCheckFlag = 0;
				}
				
				break;
		}
		default:
			break;
	}
}

//T-BOX

void vTBoxProc(void)
{

}
static void SDOParaSetsError()
{
	CanSend582.u8IndexH = CanRev602.u8IndexH; 
	CanSend582.u8IndexL = CanRev602.u8IndexL;
	CanSend582.u8SubIndexL = CanRev602.u8SubIndexL;	
	CanSend582.u8CS = 0x80;
	CanSend582.u8DataAA = 0;
	CanSend582.u8DataBB = 0;
	CanSend582.u8DataCC = 0;
	CanSend582.u8DataDD = 0;	
}

static void vSDOParaSets(uint16_t u16ParaAdressAA,uint16_t u16ParaAdressBB,
	uint16_t u16ParaAdressCC,uint16_t u16ParaAdressDD)
{
	uint8_t u8CsReturn = 0;
	uint8_t u8Cnt = 3;
	
	uint16_t u16Delaycnt = 0;
	
	CanSend582.u8IndexH = CanRev602.u8IndexH; 
	CanSend582.u8IndexL = CanRev602.u8IndexL;
	CanSend582.u8SubIndexL = CanRev602.u8SubIndexL;	
	
	if(0x40 == CanRev602.u8CS)//��ȡ
	{
		if(ADDR_INVALID != u16ParaAdressAA)
			u8CsReturn += u16ReadParaFromEeprom(u16ParaAdressAA);
		if(ADDR_INVALID != u16ParaAdressBB)
			u8CsReturn += u16ReadParaFromEeprom(u16ParaAdressBB);
		if(ADDR_INVALID != u16ParaAdressCC)
			u8CsReturn += u16ReadParaFromEeprom(u16ParaAdressCC);
		if(ADDR_INVALID != u16ParaAdressDD)
			u8CsReturn += u16ReadParaFromEeprom(u16ParaAdressDD);
		
		if(0 == u8CsReturn)//��ȡ�ɹ�
		{
			CanSend582.u8CS = 0x43;
			if(ADDR_INVALID != u16ParaAdressAA)
				CanSend582.u8DataAA = i32GetPara(u16ParaAdressAA);
			if(ADDR_INVALID != u16ParaAdressBB)
				CanSend582.u8DataBB = i32GetPara(u16ParaAdressBB);
			if(ADDR_INVALID != u16ParaAdressCC)
				CanSend582.u8DataCC = i32GetPara(u16ParaAdressCC);
			if(ADDR_INVALID != u16ParaAdressDD)
				CanSend582.u8DataDD = i32GetPara(u16ParaAdressDD);
		}
		else
		{
			SDOParaSetsError();
		}
	}
	else if(0x23 == CanRev602.u8CS)//д��
	{
		while(u8Cnt)
		{
			u8CsReturn = 0;
			u8Cnt--;
			if((ADDR_INVALID != u16ParaAdressAA)&&(CanRev602.u8DataAA != (uint8_t)(i32GetPara(u16ParaAdressAA))))
				u8CsReturn += u16SaveParaToEeprom(u16ParaAdressAA,CanRev602.u8DataAA);
			if((ADDR_INVALID != u16ParaAdressBB)&&(CanRev602.u8DataBB != (uint8_t)(i32GetPara(u16ParaAdressBB))))
				u8CsReturn += u16SaveParaToEeprom(u16ParaAdressBB,CanRev602.u8DataBB);
			if((ADDR_INVALID != u16ParaAdressCC)&&(CanRev602.u8DataCC != (uint8_t)(i32GetPara(u16ParaAdressCC))))
				u8CsReturn += u16SaveParaToEeprom(u16ParaAdressCC,CanRev602.u8DataCC);
			if((ADDR_INVALID != u16ParaAdressDD)&&(CanRev602.u8DataDD != (uint8_t)(i32GetPara(u16ParaAdressDD))))
				u8CsReturn += u16SaveParaToEeprom(u16ParaAdressDD,CanRev602.u8DataDD);			
			if(0 == u8CsReturn)
				break;
		}

		if(0 == u8CsReturn)
		{
			CanSend582.u8CS = 0x60;
			CanSend582.u8DataAA = CanRev602.u8DataAA;
			CanSend582.u8DataBB = CanRev602.u8DataBB;
			CanSend582.u8DataCC = CanRev602.u8DataCC;
			CanSend582.u8DataDD = CanRev602.u8DataDD;					
		}
		else
		{
			SDOParaSetsError();		
			CanSend582.u8DataAA = CanRev602.u8DataAA;
			CanSend582.u8DataBB = CanRev602.u8DataBB;
			CanSend582.u8DataCC = CanRev602.u8DataCC;
			CanSend582.u8DataDD = u8CsReturn;							
		}
	}
	else
	{
		SDOParaSetsError();		
	}
}
static void vSDOSpeedParaSets(uint16_t u16ParaAdressAA,uint16_t u16ParaAdressBB,
	uint16_t u16ParaAdressCC,uint16_t u16ParaAdressDD)
{
	uint8_t u8CsReturn = 0;
	uint8_t u8Cnt = 3;
	
	CanSend582.u8IndexH = CanRev602.u8IndexH; 
	CanSend582.u8IndexL = CanRev602.u8IndexL;
	CanSend582.u8SubIndexL = CanRev602.u8SubIndexL;	
	
	if(0x40 == CanRev602.u8CS)//��ȡ
	{
		if(ADDR_INVALID != u16ParaAdressAA)
			u8CsReturn += u16ReadParaFromEeprom(u16ParaAdressAA);
		if(ADDR_INVALID != u16ParaAdressBB)
			u8CsReturn += u16ReadParaFromEeprom(u16ParaAdressBB);
		if(ADDR_INVALID != u16ParaAdressCC)
			u8CsReturn += u16ReadParaFromEeprom(u16ParaAdressCC);
		if(ADDR_INVALID != u16ParaAdressDD)
			u8CsReturn += u16ReadParaFromEeprom(u16ParaAdressDD);
		
		if(0 == u8CsReturn)//��ȡ�ɹ�
		{
			CanSend582.u8CS = 0x43;
			if(ADDR_INVALID != u16ParaAdressAA)
				CanSend582.u8DataAA = (4064 / 20 /i32GetPara(u16ParaAdressAA));//����б��
			if(ADDR_INVALID != u16ParaAdressBB)
				CanSend582.u8DataBB = i32GetPara(u16ParaAdressBB);//�ٶ����ֵ
			if(ADDR_INVALID != u16ParaAdressCC)
				CanSend582.u8DataCC = i32GetPara(u16ParaAdressCC);//�ٶ���Сֵ �ݲ���Ӧ
			if(ADDR_INVALID != u16ParaAdressDD)
				CanSend582.u8DataDD = (4064 / 20 /i32GetPara(u16ParaAdressDD));//����б��
		}
		else
		{
			SDOParaSetsError();
		}
	}
	else if(0x23 == CanRev602.u8CS)//д��
	{
		while(u8Cnt)
		{
			u8CsReturn = 0;
			u8Cnt--;
			if((ADDR_INVALID != u16ParaAdressAA)&&( (4064 / 20 / CanRev602.u8DataAA)!= (uint8_t)(i32GetPara(u16ParaAdressAA))))
				u8CsReturn += u16SaveParaToEeprom(u16ParaAdressAA,(4064 / 20 / CanRev602.u8DataAA));
			if((ADDR_INVALID != u16ParaAdressBB)&&(CanRev602.u8DataBB != (uint8_t)(i32GetPara(u16ParaAdressBB))))
				u8CsReturn += u16SaveParaToEeprom(u16ParaAdressBB,CanRev602.u8DataBB);
			if((ADDR_INVALID != u16ParaAdressCC)&&(CanRev602.u8DataCC != (uint8_t)(i32GetPara(u16ParaAdressCC))))
				u8CsReturn += u16SaveParaToEeprom(u16ParaAdressCC,CanRev602.u8DataCC);
			if((ADDR_INVALID != u16ParaAdressDD)&&((4064 / 20 /CanRev602.u8DataDD) != (uint8_t)(i32GetPara(u16ParaAdressDD))))
				u8CsReturn += u16SaveParaToEeprom(u16ParaAdressDD,(4064 / 20 /CanRev602.u8DataDD));
			
			if(0 == u8CsReturn)
				break;
		}
		
		if(0 == u8CsReturn)
		{
			CanSend582.u8CS = 0x60;
			CanSend582.u8DataAA = CanRev602.u8DataAA;
			CanSend582.u8DataBB = CanRev602.u8DataBB;
			CanSend582.u8DataCC = CanRev602.u8DataCC;
			CanSend582.u8DataDD = CanRev602.u8DataDD;					
		}
		else
		{
			SDOParaSetsError();		
			CanSend582.u8DataAA = CanRev602.u8DataAA;
			CanSend582.u8DataBB = CanRev602.u8DataBB;
			CanSend582.u8DataCC = CanRev602.u8DataCC;
			CanSend582.u8DataDD = u8CsReturn;							
		}
	}
	else
	{
		SDOParaSetsError();		
	}
}


/* �����
	2000����λ������λȱ�����������ؿ��½�������
	2001�ٶ����ã���С�ٶȡ��汾��Ϣ��ecu��mcu�汾
	2002�ֱ��궨���Ƕ�ģ��
	2004�������ܡ�GPS
	*/
static void CanId602Rev(tCanFrame * Can602)//ģ��SDO
{
	tCanFrame CanSendData;
	uint8_t i = 0;
	
	memcpy((char*)CanRev602.u8Data, (char*) Can602->u8Data, sizeof(CanRev602));

	memset(&CanSendData,0,sizeof(CanSend582));
	uint8_t u8CS = 0;
	uint16_t u16Index = 0;
	uint8_t u8SubIndex = 0;
	uint32_t u32Data  = 0;
	uint16_t u16AngleSimulation = 0;
	
	u8CS = Can602->u8Data[0];
	u16Index = Can602->u8Data[1] |( Can602->u8Data[2]<<8);
	u8SubIndex = Can602->u8Data[3];

	if(0x2000 == u16Index)
	{
		switch (u8SubIndex)
		{
			case 1://��������
				vSDOParaSets(PARA_VehicleType,ADDR_INVALID,ADDR_INVALID,ADDR_INVALID);
				break;
			case 2://�õ�������ߵ��
				vSDOParaSets(PARA_PumpMotorEncoder,PARA_DriveMotorEncoder,ADDR_INVALID,ADDR_INVALID);
				break;
			case 3://�Ӷ��������֡���������
				vSDOParaSets(PARA_PitProtectFunc,PARA_AntiPinchFunc,PARA_ActAlmFunc,ADDR_INVALID);
				break;
			case 4://����λʹ�ܡ�����λʹ�ܡ���б��⡢��������			
				vSDOParaSets(ADDR_INVALID,ADDR_INVALID,PARA_TiltSwitchSetting,PARA_LiftReverseFunc);
				break;
			case 5://������͡��½�������
				vSDOParaSets(PARA_BatteryType,PARA_LowerPumpType,ADDR_INVALID,ADDR_INVALID);
				break;
			case 6://���������������Ͽذ������ߡ����ؿ��½��߶�
				vSDOParaSets(PARA_VoiceAlarmVolume,PARA_UpperCtlButSleep,ADDR_INVALID,ADDR_INVALID);
				break;
			case 7://���ع���
				vSDOParaSets(PARA_WeighFunc,ADDR_INVALID,ADDR_INVALID,ADDR_INVALID);
				break;
			case 8://������ʱ����̬���ذٷֱȡ���̬���ذٷֱȡ�����ֵ�ٷֱ�
				vSDOParaSets(PARA_OverLoadStabilityDelay,PARA_DynamicOverLoadPercent,PARA_StaticOverLoadPercent,PARA_MaxDifferencePercent);
				break;
			default:
				SDOParaSetsError();
				break;
		}
	}
	else if(0x2001 == u16Index)
	{
		switch (u8SubIndex)
		{
			case 1://����б�ʡ��������ֵ����Сֵ������б��
				vSDOSpeedParaSets(PARA_AccAndDecFastDrive,PARA_FastDriveSpeed,ADDR_INVALID,PARA_BrakeFastDrive);
				break;
			case 2://����
				vSDOSpeedParaSets(PARA_AccAndDecSlowDrive,PARA_SlowDriveSpeed,ADDR_INVALID,PARA_BrakeSlowDrive);
				break;
			case 3://�������ٶ�
				vSDOSpeedParaSets(PARA_AccAndDecAfterLift,PARA_DriveSpeedAfterLift,ADDR_INVALID,PARA_BrakeDriveAfterLift);
				break;
			case 4://�����ٶ�
				vSDOSpeedParaSets(PARA_AccAndDecLift,PARA_LiftSpeed,ADDR_INVALID,PARA_BrakeLift);
				break;
			case 5://ת���ٶ�
				vSDOSpeedParaSets(PARA_AccAndDecTurn,PARA_MaxTurnSpeed,ADDR_INVALID,PARA_BrakeTurn);
				break;
			case 6://ת�������½��ٶ�
				vSDOParaSets(PARA_TurnPowerLimit,PARA_LowerSpeed,ADDR_INVALID,ADDR_INVALID);
				break;
			case 7://ECU�汾
				vSDOParaSets(ADDR_INVALID,ADDR_INVALID,ADDR_INVALID,ADDR_INVALID);
				break;
			case 8://MCU�汾
				vSDOParaSets(ADDR_INVALID,ADDR_INVALID,ADDR_INVALID,ADDR_INVALID);
				break;
			default:
				SDOParaSetsError();
				break;
		}
	}
	else if(0x2002 == u16Index)
	{
		switch (u8SubIndex)
		{
			case 1://�ֱ��궨���϶�
				vSDOParaSets(ADDR_INVALID,ADDR_INVALID,ADDR_INVALID,ADDR_INVALID);
				break;
			case 2://�ֱ��궨����λ
				vSDOParaSets(ADDR_INVALID,ADDR_INVALID,ADDR_INVALID,ADDR_INVALID);
				break;
			case 3://�ֱ��궨���¶�
				vSDOParaSets(ADDR_INVALID,ADDR_INVALID,ADDR_INVALID,ADDR_INVALID);
				break;
			case 4://���ر궨������
				vSDOParaSets(ADDR_INVALID,ADDR_INVALID,ADDR_INVALID,ADDR_INVALID);
				break;
			case 5://���ر궨������
				vSDOParaSets(ADDR_INVALID,ADDR_INVALID,ADDR_INVALID,ADDR_INVALID);
				break;
			case 6://�Ƕ�ģ������λ
				/*������*/
				//�յ���ѯָ��ʱ����ʵʱ�ĽǶ�ֵ
				//�յ�����ָ��ʱ���浱ǰ�Ƕ�ֵ
			
				CanSend582.u8IndexH = CanRev602.u8IndexH; 
				CanSend582.u8IndexL = CanRev602.u8IndexL;
				CanSend582.u8SubIndexL = CanRev602.u8SubIndexL;	

				if(0x40 == CanRev602.u8CS)//��ȡ
				{
					
					u16AngleSimulation = i32LocalAiGetValue(ANGLE_SENSOR_CHANNEL);
					
					CanSend582.u8CS = 0x43;
					CanSend582.u8DataAA = (u16AngleSimulation >> 8) & 0xFF;
					CanSend582.u8DataBB = u16AngleSimulation & 0xFF;
					
				}
		
				else if(0x23 == CanRev602.u8CS)//д��
				{
					u16AngleSimulation = 0;
					uint8_t successToSaveParam = 0;
					if(CanRev602.u8DataAA != (uint8_t)((i32GetPara(PARA_AngleSimulationDownLimit) >> 8) & 0xFF) ||
						(CanRev602.u8DataBB != (uint8_t)(i32GetPara(PARA_AngleSimulationDownLimit) & 0xFF)))
					{
						u16AngleSimulation = (CanRev602.u8DataAA << 8) + CanRev602.u8DataBB;
						successToSaveParam = u16SaveParaToEeprom(PARA_AngleSimulationDownLimit,u16AngleSimulation);
 					}
							

					if(0 == successToSaveParam)
					{
						CanSend582.u8CS = 0x60;
						CanSend582.u8DataAA = CanRev602.u8DataAA;
						CanSend582.u8DataBB = CanRev602.u8DataBB;
						CanSend582.u8DataCC = CanRev602.u8DataCC;
						CanSend582.u8DataDD = CanRev602.u8DataDD;					
					}
					else
					{
						SDOParaSetsError();		
						CanSend582.u8DataAA = CanRev602.u8DataAA;
						CanSend582.u8DataBB = CanRev602.u8DataBB;
						CanSend582.u8DataCC = CanRev602.u8DataCC;
						CanSend582.u8DataDD = u16AngleSimulation;							
					}
				}
				else
				{
					SDOParaSetsError();		
				}
				break;
			case 7://�Ƕ�ģ������λ
				/*������*/
				//�յ���ѯָ��ʱ����ʵʱ�ĽǶ�ֵ
				//�յ�����ָ��ʱ���浱ǰ �Ƕ�ֵ
				CanSend582.u8IndexH = CanRev602.u8IndexH; 
				CanSend582.u8IndexL = CanRev602.u8IndexL;
				CanSend582.u8SubIndexL = CanRev602.u8SubIndexL;	

				if(0x40 == CanRev602.u8CS)//��ȡ
				{
					
					u16AngleSimulation = i32LocalAiGetValue(ANGLE_SENSOR_CHANNEL);
				
					CanSend582.u8CS = 0x43;
					CanSend582.u8DataAA = (u16AngleSimulation >> 8) & 0xFF;
					CanSend582.u8DataBB = u16AngleSimulation & 0xFF;
				}
		
				else if(0x23 == CanRev602.u8CS)//д��
				{
					u16AngleSimulation = 0;
					uint8_t successToSaveParam = 0;
					if(CanRev602.u8DataAA != (uint8_t)((i32GetPara(PARA_AngleSimulationDownLimit) >> 8) & 0xFF) ||
						(CanRev602.u8DataBB != (uint8_t)(i32GetPara(PARA_AngleSimulationDownLimit) & 0xFF)))
					{
						u16AngleSimulation = (CanRev602.u8DataAA << 8) + CanRev602.u8DataBB;
						successToSaveParam = u16SaveParaToEeprom(PARA_AngleSimulationDownLimit,u16AngleSimulation);
 					}
							

					if(0 == successToSaveParam)
					{
						CanSend582.u8CS = 0x60;
						CanSend582.u8DataAA = CanRev602.u8DataAA;
						CanSend582.u8DataBB = CanRev602.u8DataBB;
						CanSend582.u8DataCC = CanRev602.u8DataCC;
						CanSend582.u8DataDD = CanRev602.u8DataDD;					
					}
					else
					{
						SDOParaSetsError();		
						CanSend582.u8DataAA = CanRev602.u8DataAA;
						CanSend582.u8DataBB = CanRev602.u8DataBB;
						CanSend582.u8DataCC = CanRev602.u8DataCC;
						CanSend582.u8DataDD = u16AngleSimulation;							
					}
				}
				else
				{
					SDOParaSetsError();		
				}
			
				break;
			default:
				SDOParaSetsError();
				break;
		}
	}
	else if(0x2003 == u16Index)
	{
		switch (u8SubIndex)
		{
			case 1://��ʷ����1-4
				vSDOParaSets(PARA_ErrCode0,PARA_ErrCode1,PARA_ErrCode2,PARA_ErrCode3);
				break;
			case 2://��ʷ����5-8
				vSDOParaSets(PARA_ErrCode4,PARA_ErrCode5,PARA_ErrCode6,PARA_ErrCode7);
				break;
			case 3://��ʷ����9-10
				vSDOParaSets(PARA_ErrCode8,PARA_ErrCode9,ADDR_INVALID,ADDR_INVALID);
				break;
			default:
				SDOParaSetsError();
				break;
		}
	}
	else if(0x2004 == u16Index)
	{
		switch (u8SubIndex)
		{
			case 1://��������������
				vSDOParaSets(ADDR_INVALID,ADDR_INVALID,ADDR_INVALID,ADDR_INVALID);
				break;
			case 2://���ý���������
				vSDOParaSets(ADDR_INVALID,ADDR_INVALID,ADDR_INVALID,ADDR_INVALID);
				break;
			case 3://����������֤��
				vSDOParaSets(ADDR_INVALID,ADDR_INVALID,ADDR_INVALID,ADDR_INVALID);
				break;
			case 4://���ý�����֤��
				vSDOParaSets(ADDR_INVALID,ADDR_INVALID,ADDR_INVALID,ADDR_INVALID);
				break;
			case 5://GPS״̬
				vSDOParaSets(ADDR_INVALID,ADDR_INVALID,ADDR_INVALID,ADDR_INVALID);
				break;
			case 6://gpsID 1-4
				vSDOParaSets(ADDR_INVALID,ADDR_INVALID,ADDR_INVALID,ADDR_INVALID);
				break;
			case 7://gpsID 5-8
				vSDOParaSets(ADDR_INVALID,ADDR_INVALID,ADDR_INVALID,ADDR_INVALID);
				break;
			case 8://gpsID 9-12
				vSDOParaSets(ADDR_INVALID,ADDR_INVALID,ADDR_INVALID,ADDR_INVALID);
				break;
			default:
				SDOParaSetsError();
				break;
		}
	}
	else
		SDOParaSetsError();
	
	
	for (i=0; i<8; i++)                                 
	{
		CanSendData.u8Data[i] = CanSend582.u8Data[i];
	}

	CanSendData.u16DataLength = 8;
	CanSendData.u32ID = 0x582;
	CanSendData.u8Rtr = 0;
	
	i32CanWrite(Can0, &CanSendData);
}


static void StartCommandSend()
{
	uint8_t start_command_data[2] = {0x01, 0x00};
	tCanFrame CanframeSend;
		
	if (u8HmiCheckFlag)
	{
		CanframeSend.u32ID = 0x000;
		CanframeSend.u16DataLength = 2;
		CanframeSend.u8Rtr = 0;
		for (int i = 0; i < 2; i++)                                 
		{
			CanframeSend.u8Data[i] = start_command_data[i];
		}
		
		i32CanWrite(Can0, &CanframeSend);
	}
}

static void LiCheck()
{
//	if (u8LiCheckFlag)
//		i32ErrCodeSet(LI_BATTERY_LOSS_ERR);
	
}	
static void HMISend()
{
	xCanSend5A1 CanSend5A1 ;
	xCanSend5A2 CanSend5A2 ;
	xCanSend5A3 CanSend5A3 ;
	xCanSend5A4 CanSend5A4 ;

	xCanSend5A6 CanSend5A6 ;
	xCanSend5A7 CanSend5A7 ;
	xCanSend5A8 CanSend5A8 ;
	xCanSend5A9 CanSend5A9 ;
	xCanSend5AA CanSend5AA ;
	xCanSend5AB CanSend5AB ;
	
	memset(&CanSend5A1,0,sizeof(CanSend5A1));
	memset(&CanSend5A2,0,sizeof(CanSend5A2));
	memset(&CanSend5A3,0,sizeof(CanSend5A3));
	memset(&CanSend5A4,0,sizeof(CanSend5A4));
//	memset(&CanSend5A5,0,sizeof(CanSend5A5));
	memset(&CanSend5A6,0,sizeof(CanSend5A6));
	memset(&CanSend5A7,0,sizeof(CanSend5A7));
	memset(&CanSend5A8,0,sizeof(CanSend5A8));
	memset(&CanSend5A9,0,sizeof(CanSend5A9));
	memset(&CanSend5AA,0,sizeof(CanSend5AA));
	memset(&CanSend5AB,0,sizeof(CanSend5AB));
	
	static uint8_t u8SendCnt ;
	tCanFrame CanframeSend;
	uint8_t i = 0;
	
	u8SendCnt++;
	{
		CanSend5A1.b1BackValveState = i32LocalDiGet(BACKWARD_PUMP_R);
		CanSend5A1.b1ForwardValveState = i32LocalDiGet(FORWARD_PUMP_R);
		CanSend5A1.b1HighLowSpeedValveState = i32LocalDiGet(HIGH_SLOW_SPEED_PUMP_R);
		CanSend5A1.b1LiftValveState = i32LocalDiGet(LIFTUP_PUMP_R);
		CanSend5A1.b1TurnLeftValveState = i32LocalDiGet(TURNLEFT_PUMP_R);
		CanSend5A1.b1TurnRightValveState = i32LocalDiGet(TURNRIGHT_PUMP_R);	
		
		CanSend5A1.b1PropValve1State = i32LocalDiGet(LIFTUP_PUMP_R);
		CanSend5A1.b1PropValve2State = i32LocalDiGet(BACKWARD_PUMP_R);
		
		CanSend5A1.u8AngleSensorVoltH = ((i32LocalAiGetValue(ANGLE_SENSOR_CHANNEL)/10)>>8)&0xFF;
		CanSend5A1.u8AngleSensorVoltL = (i32LocalAiGetValue(ANGLE_SENSOR_CHANNEL)/10) & 0xFF;
		
		CanSend5A1.u8LoadRate = i32GetPara(PARA_LoadRate);
		
		CanSend5A1.u8Pressure1CurrentH = ((i32LocalAiGetValue(PRESSURE_SENSOR_CHANNEL1)/10)>>8)&0xFF;
		CanSend5A1.u8Pressure1CurrentL = (i32LocalAiGetValue(PRESSURE_SENSOR_CHANNEL1)/10) & 0xFF;
		CanSend5A1.u8Pressure2CurrentH = ((i32LocalAiGetValue(PRESSURE_SENSOR_CHANNEL2)/10)>>8)&0xFF;
		CanSend5A1.u8Pressure2CurrentL = (i32LocalAiGetValue(PRESSURE_SENSOR_CHANNEL1)/10) & 0xFF;
	}
	{
		CanSend5A2.b1DownLimitSwitch = sgSwiInput.b1DownLimitSwi;
		CanSend5A2.b1KeySwitch = sgSwiInput.b1PcuSwi;
		CanSend5A2.b1PitSwitch		= sgSwiInput.b1PitSwi;
		CanSend5A2.b1TiltSwitch = sgSwiInput.b1TiltSwi;
		CanSend5A2.b1UplimitSwitch = sgSwiInput.b1UpLimitSwi;
		CanSend5A2.KsiVoltH = ((i32GetPara(PARA_Ksi) * 10)>> 8) & 0xFF;
		CanSend5A2.KsiVoltL = (i32GetPara(PARA_Ksi) * 10) & 0xFF;
		CanSend5A2.u8PumpMotorSpeed = (u16MotorFdb/gUserInfo.u16MotorMaxSpd) & 0xFF;
	}
	{
		CanSend5A3.b1PCUEnableSwitch = sgPcuInfo.PcuKeyInfo.b1Enable;
		CanSend5A3.b1PCULeftSwitch = sgPcuInfo.PcuKeyInfo.b1TurnLeft;
		CanSend5A3.b1PCULiftModeSwitch = sgPcuInfo.PcuKeyInfo.b1LiftMode;
		CanSend5A3.b1PCUMoveModeSwitch = sgPcuInfo.PcuKeyInfo.b1MoveMode;
		CanSend5A3.b1PCURightSwitch = sgPcuInfo.PcuKeyInfo.b1TurnRight;
		CanSend5A3.b1PCUSlowSwitch = sgPcuInfo.PcuKeyInfo.b1Slow;
		CanSend5A3.b1PCUSpeakerSwitch = sgPcuInfo.PcuKeyInfo.b1Speaker;
		CanSend5A3.u8PCUHandleValue = i32GetPara(PARA_HandleAnalog);
	}
	{
		CanSend5A4.u8MainContacterState = 1;//��ȷ��
		CanSend5A4.u8MotorSpeedH = (u16MotorFdb >> 8) & 0xFF;
		CanSend5A4.u8MotorSpeedL = u16MotorFdb & 0xFF;
		CanSend5A4.u8PumpMotorEncoding = i32GetPara(PARA_PumpMotorEncoder);
	}
//	{
//		CanSend5A5.
//		
//	}
	{
		CanSend5A9.u8Soc = u8Soc;
		CanSend5A9.u8PumpCmdSpeedH = ((gUserInfo.u16MotorMaxSpd * u16SpeedCmd / MOTOR_MAX_SPEED_VALUE)>>8) & 0xFF;
		CanSend5A9.u8PumpCmdSpeedL = ((gUserInfo.u16MotorMaxSpd * u16SpeedCmd / MOTOR_MAX_SPEED_VALUE)) & 0xFF;
	}
	{
		CanSend5AA.u8HourCntLL = u32HourCnt & 0xFF; 
		CanSend5AA.u8HourCntLH = (u32HourCnt>>8) & 0xFF; 
		CanSend5AA.u8HourCntHL = (u32HourCnt>>16) & 0xFF; 
		CanSend5AA.u8HourCntHH = (u32HourCnt>>24) & 0xFF; 
	}
	{
		CanSend5AB.u8Error1 = i32GetPara(PARA_ErrCode0); 
		CanSend5AB.u8Error2 = i32GetPara(PARA_ErrCode1); 
		CanSend5AB.u8Error3 = i32GetPara(PARA_ErrCode2); 
		CanSend5AB.u8Error4 = i32GetPara(PARA_ErrCode3); 
		CanSend5AB.u8Error5 = i32GetPara(PARA_ErrCode4); 
		CanSend5AB.u8Error6 = i32GetPara(PARA_ErrCode5); 
		if(u8ErrCodeGet()<50)
			CanSend5AB.u8DriverError = u8ErrCodeGet();		
	}
	switch(u8SendCnt)
	{
		case 1:
			CanframeSend.u32ID = 0x5A1;
			CanframeSend.u16DataLength = 8;
			CanframeSend.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				CanframeSend.u8Data[i] = CanSend5A1.u8Data[i];
			}
			break;
		case 2:
			CanframeSend.u32ID = 0x5A2;
			CanframeSend.u16DataLength = 8;
			CanframeSend.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				CanframeSend.u8Data[i] = CanSend5A2.u8Data[i];
			}
			break;
		case 3:
			CanframeSend.u32ID = 0x5A3;
			CanframeSend.u16DataLength = 8;
			CanframeSend.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				CanframeSend.u8Data[i] = CanSend5A3.u8Data[i];
			}
			break;
		case 4:
			CanframeSend.u32ID = 0x5A4;
			CanframeSend.u16DataLength = 8;
			CanframeSend.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				CanframeSend.u8Data[i] = CanSend5A4.u8Data[i];
			}
			break;
		case 5:
			CanframeSend.u32ID = 0x5A5;
			CanframeSend.u16DataLength = 8;
			CanframeSend.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				CanframeSend.u8Data[i] = CanSend5A5.u8Data[i];
			}
			break;
		case 6:
			CanframeSend.u32ID = 0x5A6;
			CanframeSend.u16DataLength = 8;
			CanframeSend.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				CanframeSend.u8Data[i] = CanSend5A6.u8Data[i];
			}
			break;
		case 7:
			CanframeSend.u32ID = 0x5A7;
			CanframeSend.u16DataLength = 8;
			CanframeSend.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				CanframeSend.u8Data[i] = CanSend5A7.u8Data[i];
			}
			break;
		case 8:
			CanframeSend.u32ID = 0x5A8;
			CanframeSend.u16DataLength = 8;
			CanframeSend.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				CanframeSend.u8Data[i] = CanSend5A8.u8Data[i];
			}
			break;
		case 9:
			CanframeSend.u32ID = 0x5A9;
			CanframeSend.u16DataLength = 8;
			CanframeSend.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				CanframeSend.u8Data[i] = CanSend5A9.u8Data[i];
			}
			break;
		case 10:
			CanframeSend.u32ID = 0x5AA;
			CanframeSend.u16DataLength = 8;
			CanframeSend.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				CanframeSend.u8Data[i] = CanSend5AA.u8Data[i];
			}
			break;
		case 11:
			CanframeSend.u32ID = 0x5AB;
			CanframeSend.u16DataLength = 8;
			CanframeSend.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				CanframeSend.u8Data[i] = CanSend5AB.u8Data[i];
			}
			break;
		default:
			CanframeSend.u32ID = 0;
			u8SendCnt = 0;
			break;
	}
	if(0 != CanframeSend.u32ID)
		i32CanWrite(Can0, &CanframeSend);
}




void vUserEcuInit(void)
{
	vSetPdoPara(sgPdoPara);
	vPcuErrRegister(vPcuErrProc);	
	vPcuRevRegister(vPcuRevProc);
	vPcuSendRegister(vPcuSendProc);
	vMstRevRegister(vMstRevProc);
	vMstSendRegister(vMstSendProc);
	vBeepRegister(vBeepCallBack);
	vAiErrReg(vAiErrCallBack);
	vDoPwmErrReg(vDoPwmErrCallBack);
	vPropErrReg(vPropErrCallBack);
	vPressureSensorReg(vPressureCallBack);
	vAlarmLampRegister(vAlarmLampCallBack);
	
	/*��������ڵ���*/
	vAlarmLampSetPeriod(60);
	vAlarmLampSetOpenePeriod(120);//����Ϊ�ϱ����ڵı���
	
	vErrCodeInit(sgErrCodeInfo,(sizeof(sgErrCodeInfo))/(sizeof(sgErrCodeInfo[0])));
	
	memset(&sgActLogic, 0, sizeof(sgActLogic));
	
	vUserParaInit();
	vHourCountInit();
	vCanIdLostReg(0x726,200,vCanLostProc);
	vCanIdLostReg(0x28A,200,vCanLostProc);
	
	xRevCallBackProc CanId602 = {.u32CanId = 0x602, .u32Data = 0, .CallBack = CanId602Rev};
	vCanRevMsgRegister(&CanId602);
			
	__disable_irq();
//	gCanSendPdoInfo.CanSend260Info.u8HourCountL = u32HourCount & 0xFF;
//	gCanSendPdoInfo.CanSend260Info.u8HourCountH = (u32HourCount >> 8) & 0xFF;

	__enable_irq();		

	vSetNetTimer(TIMER_EcuPowerOn, ECU_POWERON_DELAY_TIME);	/**/
	
}
/*******************************************************************************
* Name: void vUserEcuProc(void)
* Descriptio: UserEcu�û���������
* Input: NULL
* Output: NULL  
*******************************************************************************/
void vUserEcuProc(void)
{
	static uint8_t u8EcuProcFlag = 0;
	uint8_t u8ErrCode = 0;
	static uint16_t u16SecCnt = 0;
	static uint32_t u32RentalCnt = 0;
	static uint16_t u16HMISendCnt = 0;
	static uint16_t StartCommandSendCnt = 0;
	static uint16_t liCheckCnt = 0;

	if(true == u8GetNetTimerOverFlag(TIMER_EcuPowerOn))
	{
		u8EcuProcFlag = 1;
		vKillNetTimer(TIMER_EcuPowerOn);
	}
	
	if ((false == u8GetNetTimerOverFlag(TIMER_EcuPowerOn)) && (true == u8GetNetTimerStartFlag(TIMER_EcuPowerOn)))
	{
		vSwiInitCheck();
	}
	
	if(1 == u8EcuProcFlag)
	{
		vActionProcess();
		vAiMonitor();
		vSwiMonitor();		
		vEcuSetBeepPeriod();
		vErrProc();
		vBatteryManage();
		
		
		if(StartCommandSendCnt < 20)//100ms���ڷ���һ��
			StartCommandSendCnt++;
		else
		{
			StartCommandSend();
			StartCommandSendCnt = 0;
		}
		
		
		if(liCheckCnt < 20)//100ms���ڷ���һ��
			liCheckCnt++;
		else
		{
			LiCheck();
			liCheckCnt = 0;
		}
		
		
		if(u16HMISendCnt<2)//10ms���ڷ���һ��
			u16HMISendCnt++;
		else
		{
			HMISend();
			u16HMISendCnt = 0;
		}
		
		{
			u32HourCnt = u32HourCountProc(1);//������Ҫ�жϵļ�ʱ״̬������1
		}
		i32SetPara(PARA_TurnLeftValveCurrent,u32HourCnt);
		__disable_irq();
		__enable_irq();
		PCUStateRecord.PcuKeyInfo.u8data = sgPcuInfo.PcuKeyInfo.u8data;
	}
	vWdgSetFun(WDG_USER_BIT);
}
#endif
