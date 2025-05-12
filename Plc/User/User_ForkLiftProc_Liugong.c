/*******************************************************************************
*ͨ�ó�����* 						   *
* Author: QExpand; ShiJin                                                        *
* Date: 2023/10/32    														   *
* Revision:	 														 		   *
*******************************************************************************/
#include "UserCanComm.h"
#include "USER_LIUGONG_FORKLIFT_2IN1.h"
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
#include "AiProc.h"
#include "LocalDo.h"
#include "HourCount.h"
#include "BatteryMeter.h"
#include "BeepProc.h"
#include "AlarmLamp.h"
#include "AngleSensor.h"
#include "PressureSensor.h"
#include "string.h"
#include "Para.h"
#include "WdgProc.h"
#include "math.h"
#include "Log.h"
#include "LocalAi.h"
#include "PcuProc.h"
#include "UserCommon.h"
#include "time.h"


#if (USER_TYPE == USER_FORKLIFT_LIUGONG|USER_TYPE == USER_FORKLIFT_LIUGONG_TEST)

const static xPdoParameter  sgPdoPara = 
{	
	.TpdoPara = {
		{.u8Flag = 1, .u8Type = 0x00, .u16Period = 4000},
		{.u8Flag = 1, .u8Type = 0x00, .u16Period = 4000},
		{.u8Flag = 1, .u8Type = 0x00, .u16Period = 4000},
		{.u8Flag = 1, .u8Type = 0x00, .u16Period = 4000},
		{.u8Flag = 0, .u8Type = 0xFF, .u16Period = 20, .u16CanId = 0x000 },
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50, .u16CanId = 0x000 },
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50, .u16CanId = 0x000 },
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50, .u16CanId = 0x000 },
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50, .u16CanId = 0x000 },
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 50, .u16CanId = 0x000 },
	},
	.RpdoPara = {
		{.b1Flag = 1, .b11CanRevId = 0x052},
		{.b1Flag = 1, .b11CanRevId = 0x053},
		{.b1Flag = 1, .b11CanRevId = 0x057},
		{.b1Flag = 1, .b11CanRevId = 0x166},
		{.b1Flag = 0, .b11CanRevId = 0x000},
		{.b1Flag = 0, .b11CanRevId = 0x000},
	},
};

static xErrCodeInfo sgErrCodeInfo[ErrCodeMax] = 
{		/*MCU���ϳ���������ͳһ��81*/
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//1		��������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//2		�ں����д���
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//3		��������ʱ�����
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//4		λ�ó���
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//5		���ӳ���λ��ָ��仯���������ٶ�
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//6		�ٶ�ģʽ���ٶ�ָ���������ת��
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//7		ת��ģʽ��ת��ָ������ת��
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//8		�ٶȴ���������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//9		�ٶȴ������������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//10	��
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//11	���2����������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//12	�������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//13	ĸ�ߵ��ݳ�����
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//14	���Ӵ�������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//15	�ƶ�������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//16	��ص�ѹ��
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//17	��ص�ѹ��
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//18	���ʰ����
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//19	�������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//20	��λ�ƶ�·���ѹ
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//21	���Ӵ��������۽�
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//22	5v�������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//23	id��
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//24	���Ӵ�����������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//25	����ģ�����
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//26	CanͨѶ����
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//27	��ѹ��������ѹ2V
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//28	��
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//29	��������쳣
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//30	���д���
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//31	��ѹ��ȹ���
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//32	���ʰ���ȹ���
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//33	���ʰ����
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//34	�����ȸ���
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//35	12v����
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//36	DO3����
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//37	DO4����
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//38	eeprom������д����
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//39	������Ŵ���
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//40	�ϵ�IO�쳣
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//41	��������20%
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//42	��������10%
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//43	��ѹ�ﵽ������ֵ
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//50
		
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
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 0,},	//61		DO11
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 0,},	//62		DO12
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 0,},	//63		DO13
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 0,},	//64		DO14
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 0,},	//65		DO15		
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 55,},//66 		�½���0		
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 55,},//67 		�½���1
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 0,},	//68		Ԥ��
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 0,},	//69		Ԥ��
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 0,},	//70
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 32,},	//71	ģ����1
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 31,},	//72	ģ����2
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 31,},	//73	ģ����3
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 0,},	//74	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 0,},	//75
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 0,},	//76
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 0,},	//77	����������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 0,},	//78
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 0,},	//79
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 0,},	//80
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//81	����MCU����
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//82	����MCU�����쳣
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//83	���Ź��쳣
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 2,},	//84	PCUͨѶ����
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 0,},	//85	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 0,},	//86	д��������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 0,},	//87	����������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 0,},	//88	��������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 0,},	//89	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 0,},	//90
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 0,},	//91
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 0,},	//92
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 0,},	//93
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 0,},	//94
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 0,},	//95
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 0,},	//96
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 0,},	//97
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 0,},	//98
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 0,},	//99
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 0,},	//100
		
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//101	��ʼ��
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1,	.b8UserErr = 2,},	//102	ͨ�Ŵ���
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 3,},	//103	��Чѡ��
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 4,},	//104	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 5,},	//105	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 0, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 6,},	//106	�ϵ�ʱ��������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 0, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 7,},	//107	�ϵ�ʱ���¹���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 0, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 8,},	//108	�ϵ�ʱ��������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 9,},	//109	һ������Ԥ��
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 10,},//110	��������Ԥ��
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 11,},//111	��������Ԥ��
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 12,},//112	����ʱ�¿ش���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 13,},//113	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 14,},//114	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 15,},//115 
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 16,},//116	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 17,},//117	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 18,},//118	�Ӷ�
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 19,},//119
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 20,},//120	﮵����
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 21,},//121	BMS����¶ȹ���1��ֻ����
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 22,},//122	�ŵ��������1��ֻ����
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 23,},//123 �ܵ�ѹ����1����ֹ������ֻ�ܹ���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 24,},//124	�����ѹ����1
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 25,},//125 �����ѹ����2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 26,},//126	BMSѹ�����
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 27,},//127	����²����
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 28,},//128 �ŵ����2��
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 29,},//129 �ŵ��¶ȹ���2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 30,},//130	BMS-���ѹ�����
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 31,},//131	ѹ������������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1,	.b8UserErr = 32,},//132	�Ƕȴ���������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 33,},//133	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 34,},
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1,	.b8UserErr = 35,},//135	���ر궨��
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 36,},//136	��ص�����һ������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 37,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 38,},//138	δ�궨��ɻ�궨ʧ�ܡ���ֹ����
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1,	.b8UserErr = 39,},//139	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 40,},
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 41,},//141	ƽ̨һ������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 42,},//142	����ʱ��ƽ̨����ת��ť���´���
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 43,},//143	����ʱ��ƽ̨����ת��ť���´���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 44,},
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1,	.b8UserErr = 45,},//145	ƽ̨��������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1,	.b8UserErr = 46,},//146	����ʱ��ƽ̨�ֱ�ʹ�ܿ��ذ�ť���´���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1,	.b8UserErr = 47,},//147	����ʱ��ƽ̨�ֱ�������λ����
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 48,},
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1,	.b8UserErr = 49,},//149	��������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 50,},//
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 51,},
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 52,},//152	ǰ���� ��ֻ���½�
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 53,},//153	���˷�
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 54,},//154	������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 55,},//155	�½���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 56,},//156	��ת��
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 57,},//157	��ת��
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 58,},//158	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 59,},//159	������
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
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 82,},//182	��ɲ������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 83,},//183	��ɲ������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 84,},//184	�õ����ת��ʧ��
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 85,},//185	��ǣ�������ת��ʧ��
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 86,},//186	��ǣ�������ת��ʧ��
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 87,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 88,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 89,},//189	����������ʱ���������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 90,},//190	���� 90%���ر���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 91,},//191	����������������	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 92,},//192	�ҵ��������������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 93,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 94,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 95,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 96,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 97,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 98,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 99,},//199	���� 99%���ر���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 100,},//200	ƽ̨���ر���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 101,},//201	������б������ȫ�޶�����
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 39,},//202	��ײ��������ʾ39
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 31,},//203	�ֱ�����1
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 32,},//204	�ֱ�����2
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 33,},//205	�ֱ��������� //ȷ��һ��
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr =  0,},//206	��������Ԥ��,������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 34,},//207	��ͻ������������ֹ����
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 31,},//203	�ֱ�����1
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 32,},//204	�ֱ�����2
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 33,},//205	�ֱ��������� //ȷ��һ��
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr =  0,},//206	��������Ԥ��,������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 34,},//207	��ͻ������������ֹ����
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
		uint16_t b1LowestState:1;
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
	uint16_t u16Data;
	struct
	{
		uint16_t b1Error:1;
		uint16_t b1Warning:1;
		uint16_t PCUBeep:1;
		uint16_t PCUSilence:1;
		uint16_t b1CaliInit:1;
		uint16_t	b1CaliEnd:1;
		uint16_t	b1CaliErr:1;
		uint16_t	b1SaveSuccess:1;
		uint16_t b1Verified:1;
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
	uint16_t u16UpDateTimes;
}xActionTimes;

typedef struct
{
	uint32_t u32LifHour;
	uint32_t u32SteerHour;
	uint32_t u32LowerMoveHour;
	uint32_t u32UpMoveHour;
	uint32_t u32DownHour;
}xActionHour;




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

static xErrorState sgErrorState;
<<<<<<< .mine

||||||| .r31


=======
static uint16_t u16CarCodeOld;//��ʼ����ʷ��������
>>>>>>> .r49
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

static uint8_t u8AntiPinchState = 4;
#define ANTIPINCH_INIT		0
#define ABOVE_SWI					1
#define WAIT_RELEASE			2
#define	UNDER_SWI_DELAY		4
#define UNDER_SWI_ACT			5

static uint8_t u8MotorTmp = 0;
static uint8_t u8BoardTmp = 0;
static uint16_t u16Current = 0;

static	xCanSend169 CanSend169;

/*tbox*/
static xActionFlag sgActionFlag;
static xActionTimes sgActionTimes;
static xActionHour sgActionour;
static uint8_t u8EcuProcFlag = 0;

//��������������Ӧ
static uint8_t u8ResponceFlag;
static tCanFrame ResponceCanFrame;

//������������
typedef struct
{
	uint16_t u16RandomNum;//�����
	uint32_t u32TempUnlockKey;//��ʱ��������
	uint32_t u32PermanUnlockKey;//���ý�������
	uint32_t u32FactoryMode;//����ģʽ
	uint32_t u32ParaSets;//��������
	uint16_t u16UnlockHour;//ÿ����ʱ�����󵹼�ʱ 72H(Сʱ���߹�480��0.1h)��������ʱ,��ֵΪ0xABFFʱ����Ϊ���ý�����
}xRemoteLockKey;

typedef union
{
	uint8_t u8Data;
	struct
	{//������Ҫ����������Ч�������յ�ָ��֮��������Ч
		//ִ�б�־λ
		uint8_t b1ECUHeartQuery:1;
		uint8_t b1HeartBeatQuery:1;//������ѯ����or�ر�
		uint8_t b1HeartBeatLock:1;//��������״̬����������3���ӳ�ʱ���ñ�־λ
		uint8_t b1LockLevel1:1;//һ������
		uint8_t b1LockLevel2:1;//��������
		//����ʵʱ��Ӧ
		uint8_t b1PlateformUnlock:1;//ƽ̨����
		uint8_t b1TempUnlock:1;//��ʱ����
		uint8_t b1PermaneUnlock:1;//���ý���
	};
}xHeartBeatLock;

static xRemoteLockKey sgRemoteLockKey;
static xHeartBeatLock sgHeartBeatLockDynamic;
static xHeartBeatLock LockRec;

#define TEMP_UNLOCK				(uint32_t)(0x230116)
#define PERMAN_UNLOCK			(uint32_t)(0x240117)
#define FACTORY_MODE			(uint32_t)(0x250118)
#define PARA_SETS					(uint32_t)(0x260119)

static uint32_t u32KeyCalculate(uint32_t u32NumABCDEF,uint16_t u16NumGHJN)
{
	uint32_t u32KeyResult = 0;
	int8_t i8DataA = 0;
	int8_t i8DataB = 0;
	int8_t i8DataC = 0;
	int8_t i8DataD = 0;
	int8_t i8DataE = 0;
	int8_t i8DataF = 0;
	int8_t i8DataG = 0;
	int8_t i8DataH = 0;
	int8_t i8DataJ = 0;
	int8_t i8DataN = 0;
	{
		i8DataA =  (((u32NumABCDEF >> 20)& 0xF) % 10);
		i8DataB =  (((u32NumABCDEF >> 16)& 0xF) % 10);
		i8DataC =  (((u32NumABCDEF >> 12)& 0xF) % 10);
		i8DataD =  (((u32NumABCDEF >> 8)& 0xF) % 10);
		i8DataE =  (((u32NumABCDEF >> 4)& 0xF) % 10);
		i8DataF =  (((u32NumABCDEF >> 0)& 0xF) % 10);
		
		i8DataG =  (((u16NumGHJN >> 12)& 0xF) % 10);
		i8DataH =  (((u16NumGHJN >> 8)& 0xF) % 10);
		i8DataJ=  (((u16NumGHJN >> 4)& 0xF) % 10);
		i8DataN =  (((u16NumGHJN >> 0)& 0xF) % 10);
	}
	{
		u32KeyResult |= (((abs(i8DataA + i8DataG + i8DataN * i8DataH - i8DataJ * i8DataJ - i8DataH * i8DataH)) % 10) << 20);//��һλ����
		u32KeyResult |= (((abs(i8DataB + i8DataG * i8DataG + i8DataN * i8DataJ - i8DataJ * i8DataJ + i8DataH * i8DataH)) % 10) << 16);//�ڶ�λ����
		u32KeyResult |= (((abs(i8DataC + i8DataG * i8DataG + i8DataN * i8DataN * i8DataJ - i8DataH + i8DataN * i8DataN)) % 10) << 12 );//����λ����
		u32KeyResult |= (((abs(i8DataD + i8DataG * i8DataG + i8DataN * i8DataN * i8DataJ * i8DataJ - i8DataH * i8DataH + i8DataN)) % 10) << 8);//����λ����
		u32KeyResult |= (((abs(i8DataE + i8DataG * i8DataG + i8DataN * i8DataJ * i8DataH - i8DataH * i8DataH - i8DataN)) % 10) <<4);//����λ����
		u32KeyResult |= (((abs(i8DataF + i8DataG * i8DataG + i8DataN * i8DataJ * i8DataG - i8DataH * i8DataH - i8DataN * i8DataN)) % 10) <<0);//����λ����
	}
	return u32KeyResult;
}

static void vRemoteUnlockKey()//���ɽ����������,��һ�ν����������ʧ��ʱִ��һ��
{
	uint16_t u16RandNum = 0;
	static uint16_t u16Seed = 0;
	
	u16EepromRead(RANDOM_SEED_ADDR,&u16Seed,1);
	
	u16Seed = (uint16_t)(u32HourCnt + i32LocalAiGetValue(AI_B_AI1_R)
					+ i32LocalAiGetValue(AI_B_KSI_CHECK) + i32LocalAiGetValue(AI_B_VBUS_CHECK) + u16Seed);
	
	srand(u16Seed);
	u16RandNum = (uint16_t)rand();
	u16Seed = u16RandNum;
	u16EepromWrite(RANDOM_SEED_ADDR,u16Seed,1);
	
	sgRemoteLockKey.u16RandomNum =//u16RandNum;
	(((u16RandNum >> 12) % 10)<<12)|(((u16RandNum >> 8) % 10)<<8)
																|(((u16RandNum >> 4) % 10)<<4)|(((u16RandNum >> 0) % 10)<<0);
	
	sgRemoteLockKey.u32TempUnlockKey = u32KeyCalculate( TEMP_UNLOCK ,sgRemoteLockKey.u16RandomNum);
	sgRemoteLockKey.u32PermanUnlockKey = u32KeyCalculate( PERMAN_UNLOCK ,sgRemoteLockKey.u16RandomNum);
	sgRemoteLockKey.u32FactoryMode = u32KeyCalculate( FACTORY_MODE ,sgRemoteLockKey.u16RandomNum);
	sgRemoteLockKey.u32ParaSets = u32KeyCalculate( PARA_SETS ,sgRemoteLockKey.u16RandomNum);
}

static void vCanId162Proc(tCanFrame *Canframe)//����ָ��
{
	tCanFrame CanSend;
	
	uint8_t u8CanFrameData[8]= {0};
	
	memcpy((char*)u8CanFrameData, (char*) Canframe->u8Data, sizeof(u8CanFrameData));
	memset(&CanSend.u8Data,0xFF,sizeof (CanSend.u8Data));
	
	CanSend.u16DataLength = 8;
	CanSend.u32ID = 0x263;
	CanSend.u8Rtr = 0;
	
	if(1 == u8EcuProcFlag)
	{
		if((0x6B == u8CanFrameData[0])&&(0x23 == u8CanFrameData[1]))//һ������ָ��
		{
			CanSend.u8Data[0] = 0x23;
			CanSend.u8Data[1] = 0x6B;
			if(0 == sgHeartBeatLockDynamic.b1LockLevel1)
			{
				sgHeartBeatLockDynamic.b1LockLevel1 = 1;           //�յ���ָ�������ָ��
				sgHeartBeatLockDynamic.b1LockLevel2 = 0;            
				u16SaveParaToEeprom(PARA_HeartBeatQueryFunc,sgHeartBeatLockDynamic.u8Data);
			}
		}
		if((0x6A == u8CanFrameData[0])&&(0x21 == u8CanFrameData[1]))//��������ָ��
		{
			CanSend.u8Data[0] = 0x21;
			CanSend.u8Data[1] = 0x6A;
			if(0 == sgHeartBeatLockDynamic.b1LockLevel2)
			{
				sgHeartBeatLockDynamic.b1LockLevel2 = 1;            //�յ���ָ�������ָ��
				sgHeartBeatLockDynamic.b1LockLevel1 = 0;
				u16SaveParaToEeprom(PARA_HeartBeatQueryFunc,sgHeartBeatLockDynamic.u8Data);			
			}
		}
		if((0x5A == u8CanFrameData[0])&&(0x32 == u8CanFrameData[1]))//ƽ̨����ָ��
		{
			CanSend.u8Data[0] = 0x32;
			CanSend.u8Data[1] = 0x5A;
			
			{//24.3.4ƽ̨����������б�־λ
				sgHeartBeatLockDynamic.b1LockLevel1 = 0;
				sgHeartBeatLockDynamic.b1LockLevel2 = 0;
				LockRec.u8Data = sgHeartBeatLockDynamic.u8Data;
				u16SaveParaToEeprom(PARA_HeartBeatQueryFunc,sgHeartBeatLockDynamic.u8Data);
			}
		}
		i32CanWrite(Can0,&CanSend);
	}
}

static void vCanId164Proc(tCanFrame *Canframe)//��������ָ��
{
	tCanFrame CanSend;
	
	uint8_t u8CanFrameData[8]= {0};
	static uint8_t SendDelay;
	
	
	memcpy((char*)u8CanFrameData, (char*) Canframe->u8Data, sizeof(u8CanFrameData));
	memset(&CanSend.u8Data,0xFF,sizeof (CanSend.u8Data));
	
	CanSend.u16DataLength = 8;
	CanSend.u32ID = 0x265;
	CanSend.u8Rtr = 0;
	
	if(1 == u8EcuProcFlag)
	{
		u8ResponceFlag = 1;
		if((0x7A == u8CanFrameData[0])&&(0x45 == u8CanFrameData[1]))  //��������
		{
			CanSend.u8Data[0] = 0x45;
			CanSend.u8Data[1] = 0x7A;
			memcpy(&ResponceCanFrame,&CanSend,sizeof(CanSend));
			if(0 == sgHeartBeatLockDynamic.b1HeartBeatQuery)
			{
				sgHeartBeatLockDynamic.b1HeartBeatQuery = 1;
				u16SaveParaToEeprom(PARA_HeartBeatQueryFunc,sgHeartBeatLockDynamic.u8Data);					
			}
		}
		if((0x8B == u8CanFrameData[0])&&
			((0x45 == u8CanFrameData[1])
			||(0x54 == u8CanFrameData[1])))//�ر�����
		{
			CanSend.u8Data[0] = u8CanFrameData[1];
			CanSend.u8Data[1] = 0x8B;
			memcpy(&ResponceCanFrame,&CanSend,sizeof(CanSend));
			if(1 == sgHeartBeatLockDynamic.b1HeartBeatQuery)
			{
				sgHeartBeatLockDynamic.b1HeartBeatQuery = 0;
				u16SaveParaToEeprom(PARA_HeartBeatQueryFunc,sgHeartBeatLockDynamic.u8Data);			
			}
		}
		if(SendDelay > 50)
		{
			SendDelay = 0;
			i32CanWrite(Can0,&CanSend);
		}
		else 
		{
			SendDelay++;
		}
	}
}
static void vResponceProc()
{
	static uint16_t u16SendCnt;
	if(1 == u8ResponceFlag)
	{
		if(u16SendCnt < ( 1000 / USER_ECU_PERIOD))
			u16SendCnt++;
		else
		{
			i32CanWrite(Can0,&ResponceCanFrame);
		}		
	}

}

static void vRemoteLockInit()
{
	xRevCallBackProc CanId162 = {.u32CanId = 0x162, .u32Data = 0, .CallBack = vCanId162Proc};
	xRevCallBackProc CanId164 = {.u32CanId = 0x164, .u32Data = 0, .CallBack = vCanId164Proc};
	vCanRevMsgRegister(&CanId162);
	vCanRevMsgRegister(&CanId164);
	
	sgHeartBeatLockDynamic.u8Data = gUserInfo.u8HeartBeatQueryFunc;
	LockRec.u8Data = gUserInfo.u8HeartBeatQueryFunc;
	sgRemoteLockKey.u16UnlockHour = i32GetPara(TEMP_UNLOCK_HOUR_ADDR_16);//��ȡ����ʱ��
}

static void vRemoteLockProc()//����lock������dymanic������dynamic�б䶯��ʵʱ�洢
{
	/*��ʱ������ʱ*/
	static uint16_t u16Cnt6Min = 0;
	
	if((0 == LockRec.b1TempUnlock)&&(1 == sgHeartBeatLockDynamic.b1TempUnlock))
	{
		sgRemoteLockKey.u16UnlockHour = 72 * 60 / 6;//72Сʱ��Сʱ��6����һ��
		u16SaveParaToEeprom(TEMP_UNLOCK_HOUR_ADDR_16, sgRemoteLockKey.u16UnlockHour);
	}
	
	if(u16Cnt6Min != (u32HourCnt & 0xFFFF))//Сʱ�Ƶ�16λ
	{
		if(sgRemoteLockKey.u16UnlockHour > 0)//��ʱ����ʱ�����
		{
			sgRemoteLockKey.u16UnlockHour --;
			u16SaveParaToEeprom(TEMP_UNLOCK_HOUR_ADDR_16, sgRemoteLockKey.u16UnlockHour);
		}
		u16Cnt6Min = (u32HourCnt & 0xFFFF);
	}
	
	if((1 == sgHeartBeatLockDynamic.b1TempUnlock)&&(0 == sgRemoteLockKey.u16UnlockHour))//��ʱ������ʱ����
	{
		if(1 == sgHeartBeatLockDynamic.b1TempUnlock)
		{
			sgHeartBeatLockDynamic.b1TempUnlock = 0;
			u16SaveParaToEeprom(PARA_HeartBeatQueryFunc,sgHeartBeatLockDynamic.u8Data);	
		}
	}
	
	if((1 == sgHeartBeatLockDynamic.b1HeartBeatLock)
		&&(1 == sgHeartBeatLockDynamic.b1HeartBeatQuery))//���ȼ� ������������һ��
	{
		i32ErrCodeSet(HEART_BEAT_LOCK_ERR);
	}
	else if(1 == LockRec.b1LockLevel2)
	{
		i32ErrCodeSet(PLATFORM_LEVEL2_LOCK_ERR);
	}	
	else if(1 == LockRec.b1LockLevel1)
	{
		i32ErrCodeSet(PLATFORM_LEVEL1_LOCK_ERR);
	}
	
//	if((0 == LockRec.b1HeartBeatLock)&&(1 == sgHeartBeatLockDynamic.b1HeartBeatLock))
//	{
//		i32ErrCodeSet(HEART_LOCK_ALARMING);
//	}
	if((0 == LockRec.b1LockLevel1)&&(1 == sgHeartBeatLockDynamic.b1LockLevel1))
	{
		i32ErrCodeSet(PLATE_LOCK_LEVEL1_ALARMING);
	}
	if((0 == LockRec.b1LockLevel2)&&(1 == sgHeartBeatLockDynamic.b1LockLevel2))
	{
		i32ErrCodeSet(PLATE_LOCK_LEVEL2_ALARMING);
	}
	
	//��������ʱ�����ֽ��յ�����ָ�
		

	if(0 == sgHeartBeatLockDynamic.b1LockLevel2)
	{
		i32ErrCodeClr(PLATE_LOCK_LEVEL2_ALARMING);
		i32ErrCodeClr(PLATFORM_LEVEL2_LOCK_ERR);
	}	
	
		
	if(0 == sgHeartBeatLockDynamic.b1LockLevel1)
	{
		i32ErrCodeClr(PLATE_LOCK_LEVEL1_ALARMING);
		i32ErrCodeClr(PLATFORM_LEVEL1_LOCK_ERR);
	}
	
//	if((1 == sgHeartBeatLockDynamic.b1PermaneUnlock)//���ڽ���ָ�ʵʱ����
//		||(1 == sgHeartBeatLockDynamic.b1PlateformUnlock)
//		||(1 == sgHeartBeatLockDynamic.b1TempUnlock))
//	{
//		i32ErrCodeClr(HEART_BEAT_LOCK_ERR);
//		i32ErrCodeClr(PLATFORM_LEVEL2_LOCK_ERR);
//		i32ErrCodeClr(PLATFORM_LEVEL1_LOCK_ERR);
//	}
	
	
//	LockRec.u8Data = sgHeartBeatLockDynamic.u8Data;
}

/*  ����260 �޸Ĳ�����ɺ� ���� 190 */
	static uint16_t u16FunctionIndex = 0;
	static uint8_t u8SpeedIndex = 0;
	static uint16_t u16SpeedRate = 0;
	static uint8_t u8Result = 0;
	static uint8_t u8FunctionSets = 0;


typedef void (*func_ptr_t)(void);
typedef  void (*pFunction)(void);
uint32_t JumpAddress;
func_ptr_t JumpToApplication;
uint32_t msp;
static void JumpToBoot2()
{
	JumpAddress = *(__IO uint32_t*) (SECOND_LEVEL_BOOT_ADDR + 4);
	JumpToApplication = (pFunction) JumpAddress;
	/*disable systick interrupt*/
	//SysTick->CTRL = 0;
	/* Initialize user application's Stack Pointer */
	msp = *(__IO uint32_t*)SECOND_LEVEL_BOOT_ADDR;
	
	__set_MSP(msp);
	JumpToApplication();
}
static void vSetFlashUpdateFlag()
{
	fmc_unlock();
	fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);
	fmc_page_erase(UDS_UDDATE_FALGE_ADDR);
	fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);
	fmc_lock();
	
	vDelayms(10);
	
	fmc_unlock();
    /* Clear All pending flags */
    fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);
	fmc_word_program(UDS_UDDATE_FALGE_ADDR, UDS_UPDATE_FLAGE);
	fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);
    fmc_lock();
}


static uint8_t u8260ParaMonit(uint16_t u16Index,uint8_t u8Data)
{
	uint8_t u8res = 0;
	if((0xFF != u8Data)&&(u8Data <= 1))
		u8res = u16SaveParaToEeprom(u16Index,(1 - u8Data));
	return u8res;
}  

static void vTboxDownLoadData()
{
	static uint16_t u16ExitMode;
	
	if(REMOTE_INIT == u8RemoteParaFlag)//���״̬������λ֮�£��Ͽ�ģʽ
	{
		if((1 == sgSwiInput.b1DownLimitSwi)
		&&(1 == sgSwiInput.b1PcuSwi)
		&&(u8Soc > 20)
		&&(0 == u8ErrCodeGet()))
		{
			u8RemoteParaFlag = REMOTE_WAITHANDE;//���ɹ����ȴ�����
		}
		else
		{
			u8RemoteParaFlag = REMOTE_INIT;
		}
	}
	if(REMOTE_HANDSHAKE == u8RemoteParaFlag)//���ֳɹ����ȴ��û�ȷ��
	{
		u16ExitMode++;
		if(1 == sgSwiInput.b1LowerCtlUp)
		{
			u8RemoteParaFlag = REMOTE_WAITEDATA;
		}
		if((u16ExitMode > (60000 / USER_ECU_PERIOD))
			||(1 == sgSwiInput.b1LowerCtlDown))//�˳�ģʽ
		{
			u8RemoteParaFlag = REMOTE_INIT;
			u8RunMode = NORMAL_MODE;
			u8ParaSetMode = PARASETS_DISABLE;
		}
	}
	if(u8RemoteParaFlag == REMOTE_WAITEDATA)
	{
		switch(u16FunctionIndex)
		{
				case 1://����
					u8Result = u8260ParaMonit(PARA_WeighFunc,u8FunctionSets);
					break;
				case 2://�Ƕȴ�����
					u8Result = u8260ParaMonit(PARA_AngleSimulationLimit,u8FunctionSets);
					break;
				case 3://������
					u8Result = u8260ParaMonit(PARA_AntiPinchFunc,u8FunctionSets);
						break;
				case 4://﮵������
					u8Result = u8260ParaMonit(PARA_BatteryType,u8FunctionSets);
						break;
				case 5://����ģʽ
					u8Result = u8260ParaMonit(PARA_InAndOutFunc,u8FunctionSets);
						break;
				case 6://����λ����
					if(1 == u8FunctionSets)//�ر�
					{
						sgUserSets.b1LowLimitEn = 0;
					}
					else if(0 == u8FunctionSets)
					{
						sgUserSets.b1LowLimitEn = 1;
					}
					u8Result = u16SaveParaToEeprom(PARA_USERSETS,sgUserSets.u16Data);
						break;
				case 7://����λ����
					if(1 == u8FunctionSets)//�ر�
					{
						sgUserSets.b1UpLimitEn = 0;
					}
					else if(0 == u8FunctionSets)
					{
						sgUserSets.b1UpLimitEn = 1;
					}
					u8Result = u16SaveParaToEeprom(PARA_USERSETS,sgUserSets.u16Data);
						break;
				case 8://ˮƽ������
					u8Result = u8260ParaMonit(PARA_TiltSwitchSetting,u8FunctionSets);
						break;
				case 9://�߿��������߹���
					if(1 == u8FunctionSets)//�ر�
					{
						sgUserSets.b1LiftBanMove = 0;
					}
					else if(0 == u8FunctionSets)
					{
						sgUserSets.b1LiftBanMove = 1;
					}
					u8Result = u16SaveParaToEeprom(PARA_USERSETS,sgUserSets.u16Data);
						break;
				case 10://���ܰ���
//					u8Result = u8260ParaMonit(PARA_AngleSimulationLimit,u8FunctionSets);
						break;
				case 11://��ѹ��������
				case 12://˫ѹ��������
					if((0 == u8FunctionSets)&&(11 == u16FunctionIndex))
					{
						u8Result = u16SaveParaToEeprom(PARA_AngleSensorType,SingleChannelSensor);
					}
					else if((0 == u8FunctionSets)&&(12 == u16FunctionIndex))
					{
						u8Result = u16SaveParaToEeprom(PARA_AngleSensorType,DoubleChannelSensor);
					}
					else
					{
						u8Result = u16SaveParaToEeprom(PARA_AngleSensorType,NoSensor);
					}
						break;
				case 14://����������
//					u8Result = u16SaveParaToEeprom(PARA_DriverType,u8FunctionSets);
						break;
				case 15://��������
					u8Result = u8260ParaMonit(PARA_CARCODE,u8FunctionSets);
						break;
				case 16://TBOX�������
//					u8Result = u8260ParaMonit(PARA_AngleSimulationLimit,u8FunctionSets);
						break;
				case 17://�Ӷ�����
					u8Result = u8260ParaMonit(PARA_PitProtectFunc,u8FunctionSets);
						break;
				case 18://��������
					u8Result = u8260ParaMonit(PARA_ActAlmFunc,u8FunctionSets);
						break;
				case 19://���ٷ�״̬
					u8Result = u8260ParaMonit(PARA_ParallelValveReverseFunc,u8FunctionSets);
						break;
				case 20://�½�������
					u8Result = u8260ParaMonit(PARA_LowerPumpType,u8FunctionSets);
						break;
				case 21://�ֱ�����
					u8Result = u8260ParaMonit(PARA_LiftReverseFunc,u8FunctionSets);
						break;
				case 22://�ĵ����
					u8Result = u8260ParaMonit(PARA_FourPointWeightFunc,u8FunctionSets);
						break;
				case 23://������ѯ
					
					if(1 == u8FunctionSets)//�ر�
					{
						sgHeartBeatLockDynamic.b1HeartBeatQuery = 0;
					}
					else if(0 == u8FunctionSets)
					{
						sgHeartBeatLockDynamic.b1HeartBeatQuery = 1;
					}
					u8Result = u16SaveParaToEeprom(PARA_HeartBeatQueryFunc,sgHeartBeatLockDynamic.u8Data);
						break;
				case 24://ɲ���ͷ�
//					u8Result = u8260ParaMonit(PARA_AngleSimulationLimit,u8FunctionSets);
						break;
				case 25://�Ƕȴ������ɼ���ʽ
					u8Result = u8260ParaMonit(PARA_AngleSensorType,u8FunctionSets);
						break;
				case 26://����������
//					u8Result = u8260ParaMonit(PARA_AngleSimulationLimit,u8FunctionSets);
						break;
				default:
						break;
		}
		switch(u8SpeedIndex)
		{
			case 1:
				if(0xFFFF != u16SpeedRate)
				{
					u8Result += u16SaveParaToEeprom(PARA_LiftSpeed,u16SpeedRate);
				}
				break;
			case 2:
				if(0xFFFF != u16SpeedRate)
				{
					u8Result += u16SaveParaToEeprom(PARA_LowerSpeed,u16SpeedRate);
				}
				break;
			case 3:
				if(0xFFFF != u16SpeedRate)
				{
					u8Result += u16SaveParaToEeprom(PARA_FastDriveSpeed,u16SpeedRate);
				}
				break;
			case 4:
				if(0xFFFF != u16SpeedRate)
				{
					u8Result += u16SaveParaToEeprom(PARA_SlowDriveSpeed,u16SpeedRate);
				}
				break;
			case 5:
				if(0xFFFF != u16SpeedRate)
				{
					u8Result += u16SaveParaToEeprom(PARA_DriveSpeedAfterLift,u16SpeedRate);
				}
				break;
			case 6:
				if(0xFFFF != u16SpeedRate)
				{
					u8Result += u16SaveParaToEeprom(PARA_MaxTurnSpeed,u16SpeedRate);
				}
				break;
			case 7:
				if(0xFFFF != u16SpeedRate)
				{
					u8Result += u16SaveParaToEeprom(PARA_TurnPowerLimit,u16SpeedRate);
				}
				break;
			default:
				break;
		}
//		{
//			memset(&CanSend,0,sizeof(CanSend));
//			CanSend.u16DataLength = 8;
//			CanSend.u32ID = 0x190;
//			CanSend.u8Rtr = 0;
			if(0 == u8Result)
			{
//				CanSend.u8Data[0] = 2;
				u8RemoteParaFlag = REMOTE_CHANGE_SUCCESS;
			}			
			else
			{
//				CanSend.u8Data[0] = 3;
				u8RemoteParaFlag = REMOTE_CHANGE_FAIL;
			}
//			i32CanWrite(Can0,&CanSend);
//		}
	}
	
	if(REMOTE_OTA_REQ == u8RemoteParaFlag)//�ϲ�6��ȷ�ϣ��²�3�λ�1����δȷ�ϣ��˳�
	{//�ϲ�ΪA���²�Ϊ5
		static uint32_t u32KeyRec = 0xFFFFFFFF;
		static xSwiInput SwiRec;
		static uint16_t u16TimeOut;
		u16TimeOut++;
		
		u8ParaSetMode = TBOX_UPDATEAPP;

		if((0 == sgSwiInput.b1LowerCtlUp)&&(1 == SwiRec.b1LowerCtlUp))
		{
			u32KeyRec = u32KeyRec <<4 | 0xA;
		}
		if((0 == sgSwiInput.b1LowerCtlDown)&&(1 == SwiRec.b1LowerCtlDown))
		{
			u32KeyRec = u32KeyRec <<4 | 0x5;
		}
		
		if(0xAAAAAA == (u32KeyRec & 0xFFFFFF))//�ϲ�6��
		{
			u8RemoteParaFlag = REMOTE_OTA_START;
		}
		else if((0x555 == (u32KeyRec & 0xFFF))
			||(u16TimeOut > (60000 / USER_ECU_PERIOD)))//�²�3��
		{
			tCanFrame CanReject;
			memset(CanReject.u8Data,0xFF,8);
			CanReject.u16DataLength = 8;
			CanReject.u32ID = 0x59;
			CanReject.u8Rtr = 0;
			CanReject.u8Data[0] = 0x53;
			CanReject.u8Data[1] = 0x4A;
			CanReject.u8Data[2] = 0xAF;
			
			u8RemoteParaFlag = REMOTE_INIT;
			u8ParaSetMode = PARASETS_DISABLE;
			i32CanWrite(Can0,&CanReject);
		}
		SwiRec.u16data = sgSwiInput.u16data;
	}
	if(REMOTE_OTA_START == u8RemoteParaFlag)
	{
		#ifdef SECOND_BOOT_EN
		{
			vSetFlashUpdateFlag();
			__disable_irq();
			JumpToBoot2();
		}
		#endif
	}
}


#if 1
static void vClearUpdateFlag()
{
	if(USER_VALID !=  *(__IO uint32_t*)USER_FLAG_ADDR)
	{
		fmc_unlock();
		fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);
		fmc_page_erase(UDS_UDDATE_FALGE_ADDR);
		fmc_page_erase(USER_FLAG_ADDR);
		fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);
		fmc_lock();
		
		vDelayms(10);
		
		fmc_unlock();
			/* Clear All pending flags */
	 
		fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);
		fmc_word_program(USER_FLAG_ADDR, USER_VALID);
		fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);
		fmc_lock();
		
		vDelayms(10);		
	}
}
#endif

static void vCanId058Proc(tCanFrame *Canframe)
{
	uint8_t u8CanFrameData[8];
	uint8_t i;
	tCanFrame CanSend;
	memcpy((char*)u8CanFrameData, (char*) Canframe->u8Data, sizeof(u8CanFrameData));
	if((0x8C == u8CanFrameData[0])&&(0x16 == u8CanFrameData[1]))
	{
		if(REMOTE_WAITHANDE == u8RemoteParaFlag)
		{
			u8RemoteParaFlag = REMOTE_HANDSHAKE;//���ֳɹ�
			u8RunMode = NO_ACTION_MODE;//��ֹ����
			u8ParaSetMode = TBOX_PARA_SETS;//�л���ʾģʽ
			CanSend.u16DataLength = 8;
			CanSend.u32ID = 0x59;
			CanSend.u8Rtr = 0;
			CanSend.u8Data[0] = 0x16;
			CanSend.u8Data[1] = 0x8C;
			for(i=2;i<8;i++)
			{
				CanSend.u8Data[i] = 0xFF;
			}
			i32CanWrite(Can0,&CanSend);			
		}		
	}
	else if((0x53 == u8CanFrameData[0])&&(0x4A == u8CanFrameData[1])&&(0x10 == u8CanFrameData[2]))
	{
		sgHeartBeatLockDynamic.b1HeartBeatQuery = 0;
		if(REMOTE_WAITHANDE == u8RemoteParaFlag)
		{
			u8RemoteParaFlag = REMOTE_OTA_REQ;
		}
	}
}


static void vCanId260Proc(tCanFrame *Canframe)
{
	uint8_t u8CanFrameData[8];
	tCanFrame CanSend;

	
	memcpy((char*)u8CanFrameData, (char*) Canframe->u8Data, sizeof(u8CanFrameData));
	u16FunctionIndex = u8CanFrameData[0] | (u8CanFrameData[1]<<8);
	u8FunctionSets = u8CanFrameData[2];
	u8SpeedIndex = u8CanFrameData[3];
	u16SpeedRate = u8CanFrameData[5] | (u8CanFrameData[6]<<8);	

}

static void vTboxDownLoadDataInit()
{
	xRevCallBackProc CanId058 = {.u32CanId = 0x058, .u32Data = 0, .CallBack = vCanId058Proc};
	xRevCallBackProc CanId260 = {.u32CanId = 0x260, .u32Data = 0, .CallBack = vCanId260Proc};
	vCanRevMsgRegister(&CanId058);
	vCanRevMsgRegister(&CanId260);
}

static void vAiErrCallBack(eAiErrNo AiErrNo)
{
//	i32ErrCodeClr(ErrCode71);
//	i32ErrCodeClr(ErrCode72);
//	i32ErrCodeClr(ErrCode73);
	switch((uint8_t)AiErrNo)
	{
		case ANGLE_SENSOR_CHANNEL:
//			if(FunctionEnable == gUserInfo.u8AngleSimulationLimit)
				i32ErrCodeSet(ANGLE_SENSOR_ERR);			
			break;
		case PRESSURE_SENSOR_CHANNEL:
			i32ErrCodeSet(PRESSURE_SENSOR_ERR);
			break;
		case PRESSURE_SENSOR_CHANNEL2:
			i32ErrCodeSet(PRESSURE_SENSOR_ERR);
			break;		
		default:
			break;
	}
}

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
		case BACKWARD_PUMP:
			i32ErrCodeSet(BACKWARD_VALVE_ERR);
			break;		
		case FORWARD_PUMP:
			i32ErrCodeSet(FORWARD_VALVE_ERR);
			break;		
		case HIGH_SPEED_PUMP:
			i32ErrCodeSet(PARALLEL_VALVE_ERR);
			break;
		default:
			break;
	}
}



static void vPropErrCallBack(uint8_t u8Channel)
{
//	i32ErrCodeClr(ErrCode66);
//	i32ErrCodeClr(ErrCode67);
	switch(u8Channel)
	{
		case PropDriverCh0:
			i32ErrCodeSet(LIFT_DOWN_VALVE_ERR);
			break;
		case PropDriverCh1:
			i32ErrCodeSet(LIFT_DOWN_VALVE_ERR);
			break;
		default:
			break;
	}
}
static void vSwiInitCheck(void)
{
	if((1 == i32LocalDiGet(LOWER_CONTROLL_UP))
		||(1 == i32LocalDiGet(LOWER_CONTROLL_DOWN)))
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
				i32DoPwmSet(FORWARD_PUMP, PUMP_CLOSE_PERCENTAGE);
				i32DoPwmSet(BACKWARD_PUMP, PUMP_CLOSE_PERCENTAGE);
				i32DoPwmSet(TURNLEFT_PUMP, PUMP_CLOSE_PERCENTAGE);
				i32DoPwmSet(TURNRIGHT_PUMP, PUMP_CLOSE_PERCENTAGE);
				i32DoPwmSet(LIFTUP_PUMP, PUMP_CLOSE_PERCENTAGE);
				vPropSetTarget(LIFTDOWN_PUMP1, 0);
				vPropSetTarget(LIFTDOWN_PUMP2, 0);
			}
			break;
		case PCU_LiftKeyPress:
			sgPcuKeyInit.b1LiftMode = 1;	
			break;
		case PCU_SlowKeyPress:
			sgPcuKeyInit.b1Slow = 1;
			break;
		case PCU_MoveKeyPress:
			sgPcuKeyInit.b1MoveMode = 1;
			break;
		case PCU_TurnLeftPress:
			sgPcuKeyInit.b1TurnLeft = 1;
			break;
		case PCU_TurnRightPress:
			sgPcuKeyInit.b1TurnRight = 1;
			break;
		case PCU_EnableKeyPress:
			sgPcuKeyInit.b1Enable = 1;
			break;
		case PCU_ValueNoZero:
			sgPcuKeyInit.b1HandleValue = 1;
			break;
		case PCU_SpeakerPress:
			sgPcuKeyInit.b1Speaker = 1;
			break;
		default:
			break;
	}
}

//INPUT
void vPcuRevProc(xPcuRevPara *RevData)//�ϵ��ִ��
{
	int8_t	i8HandleValue = 0;
	int8_t i8HandleValueChange = 0;
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
		if(i8HandleValue > sgPcuHandle.i8PositiveValue)
		{
			i8HandleValueChange = 127;
		}
		else if(i8HandleValue > (sgPcuHandle.i8MiddleValue + gUserInfo.u8DeadZoneAdjust))//��ֵ�����ֵ
		{
			i8HandleValueChange = (int8_t)((int16_t)(i8HandleValue - sgPcuHandle.i8MiddleValue) * 127 / (sgPcuHandle.i8PositiveValue - sgPcuHandle.i8MiddleValue)) ;
		}
		else if(i8HandleValue > (sgPcuHandle.i8MiddleValue - gUserInfo.u8DeadZoneAdjust))
		{
			i8HandleValueChange = 0;
		}
		else if(i8HandleValue > sgPcuHandle.i8NegativeValue)//��ֵ����Сֵ
		{
			i8HandleValueChange =  (int8_t)((sgPcuHandle.i8MiddleValue - i8HandleValue) * 127 / (sgPcuHandle.i8NegativeValue - sgPcuHandle.i8MiddleValue)) ;
		}
		else
		{
			i8HandleValueChange = - 128;
		}
		
		i32SetPara(PARA_HandleAnalog, (abs(i8HandleValueChange) * 100 / 127));
		i32SetPara(PARA_BrakeValveCurrent, (abs(i8HandleValueChange)));
		
		sgPcuInfo.i16HandleValue = i8HandleValueChange * 32;
		
	 if((LIFT_MODE == u8PcuMode))
		{
			if(FunctionEnable == gUserInfo.u8LiftReverseFunc)
				sgPcuInfo.i16HandleValue = 0 - i8HandleValueChange * 32;				
		}	
	}
}

static void vPcuSleep()
{
	static uint16_t u16Cnt = 0;
	if(1 == sgUserSets.b1SleepEn)
	{
		if(0 == sgPcuInfo.PcuKeyInfo.u8data)
		{
			if(0 != gUserInfo.u8UpperCtlButSleep)
			{
				if(u16Cnt < (gUserInfo.u8UpperCtlButSleep * 1000 / USER_ECU_PERIOD))
				{
					u16Cnt ++;
				}
				else
				{
					u8PcuMode = INITIAL_MODE;
				}
			}
		}
		else
		{
			u16Cnt = 0;
		}		
	}
}

static void vSwiMonitor(void)
{
	xSwiInput SwiInput = {0};
	static	uint16_t u16TiltDelay = 0;
	
	if(1 == i32LocalDiGet(PCU_SWICTH))
	{
		SwiInput.b1PcuSwi = 1;
	}
	if(1 == i32LocalDiGet(LOWER_CONTROLL_UP))
	{
		SwiInput.b1LowerCtlUp = 1;
	}
	if(1 == i32LocalDiGet(LOWER_CONTROLL_DOWN))
	{
		SwiInput.b1LowerCtlDown = 1;
	}
	
	if(0 == i32LocalDiGet(ANTICOLLISION_SWITCH))
	{
		SwiInput.b1AntiCollision = 1;
	}
	if(0 == i32LocalDiGet(TILT_SIWTCH))
	{
		if(FunctionEnable == gUserInfo.u8TiltSwitchSetting)
		{
			if(u16TiltDelay < 200)//��ʱ�жϱ�������������ɸ���
				u16TiltDelay++;
			else
			{
				SwiInput.b1TiltSwi = 1;
			}		
		}

	}
	else
	{
		u16TiltDelay = 0;
	}
	
	if(1 == i32LocalDiGet(PIT_SWITCH))
	{
		SwiInput.b1PitSwi = 1;
	}
		
	if(FunctionEnable == gUserInfo.u8AngleSimulationLimit)
	{
		int32_t i32Angletmp = 0;
		i32Angletmp = i32LocalAiGet(ANGLE_SENSOR_CHANNEL);
		if(i32Angletmp <= gUserInfo.u16AngleSimulationDownLimit)
		{
			SwiInput.b1DownLimitSwi = 1;
		}
		else
		{
			if(1 == sgUserSets.b1DoubleRangeHeight)
			{
				if(((i32Angletmp >= gUserInfo.u16AngleSimulationUpLimit)&&(FunctionEnable == gUserInfo.u8InAndOutFunc))
					||((i32Angletmp >= i32GetPara(PARA_SetOutHeight))&&(FunctionDisable == gUserInfo.u8InAndOutFunc)))
				{
					SwiInput.b1UpLimitSwi = 1;
				}
			}
			else
			{
				if((i32Angletmp >= gUserInfo.u16AngleSimulationUpLimit))
				{
					SwiInput.b1UpLimitSwi = 1;
				}
			}
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
	
	if(0 == sgUserSets.b1LowLimitEn)
	{
<<<<<<< .mine
		SwiInput.b1DownLimitSwi = 0;
	}
	if(0 == sgUserSets.b1UpLimitEn)
	{
		SwiInput.b1UpLimitSwi = 0;
	}
	
	if(1 == sgUserSets.b1LowLimitBanMoveEn)
	{
		if((i32LocalAiGet(ANGLE_SENSOR_CHANNEL)) 
			> (i32GetPara(PARA_LOWLIMIT_ANGLE)+i32GetPara(PARA_LOWLIMIT_RANGE)))
		SwiInput.b1LowestState = 1;
	}
	
	if(FunctionEnable == gUserInfo.u16AnticollisionFunc)
	{
		if((1 == SwiInput.b1AntiCollision)
			&&((LIFT_MODE == u8PcuMode)
				||((MOVE_MODE == u8PcuMode)&&(1 == SwiInput.b1DownLimitSwi || 1 == SwiInput.b1LowestState)))
			)
		{
			i32ErrCodeSet(MACHINE_ANTIPUNCH_ERR);
		}
		else
		{
			i32ErrCodeClr(MACHINE_ANTIPUNCH_ERR);
		}
	}
	
	//�����Ӷ�����  
	if(1 == SwiInput.b1PitSwi)     //2024.0508��@DSY��б������Ӷ����ع���
	{
||||||| .r31
=======
		SwiInput.b1DownLimitSwi = 0;
	}
	if(0 == sgUserSets.b1UpLimitEn)
	{
		SwiInput.b1UpLimitSwi = 0;
	}
	
	if(1 == sgUserSets.b1LowLimitBanMoveEn)
	{
		if((i32LocalAiGet(ANGLE_SENSOR_CHANNEL)) 
			> (i32GetPara(PARA_LOWLIMIT_ANGLE)+i32GetPara(PARA_LOWLIMIT_RANGE)))
		SwiInput.b1LowestState = 1;
		else 
			SwiInput.b1LowestState = 0;	
	}
	
	if(FunctionEnable == gUserInfo.u16AnticollisionFunc)
	{
		if((1 == SwiInput.b1AntiCollision)
			&&((LIFT_MODE == u8PcuMode)
				||((MOVE_MODE == u8PcuMode)&&((1 == SwiInput.b1DownLimitSwi) ||( 1 == SwiInput.b1LowestState))))
			)
		{
			i32ErrCodeSet(MACHINE_ANTIPUNCH_ERR);
		}
		else
		{
			i32ErrCodeClr(MACHINE_ANTIPUNCH_ERR);
		}
	}
	
	//�����Ӷ�����  
	if(1 == SwiInput.b1PitSwi)     //2024.0508��@DSY��б������Ӷ����ع���
	{
>>>>>>> .r49
		if(1 == SwiInput.b1TiltSwi)
		{
			i32ErrCodeSet(MACHINE_TILT_OVER_SAFETY_ERR);
		}
		else
		{
			i32ErrCodeClr(MACHINE_TILT_OVER_SAFETY_ERR);
		}
	}
	else
	{
		i32ErrCodeClr(MACHINE_TILT_OVER_SAFETY_ERR);
	}
	if(0 == SwiInput.b1DownLimitSwi)
	{
		if((FunctionEnable == gUserInfo.u8PitProtectFunc)&&(0 == SwiInput.b1PitSwi))
		{
			if(false == i32ErrCodeCheck(ANGLE_SENSOR_ERR))
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
	}
	sgSwiInput.u16data = SwiInput.u16data;
}

static void vModeChange(void)//ģʽ�л���������Ӧ�Ĳ������ã�Ȼ����Ӧ��������
{
	static uint16_t u16AlarmCnt;
	if(u16AlarmCnt < (500 / USER_ECU_PERIOD))
	{
		u16AlarmCnt++;
	}
	else if(u16AlarmCnt < (2000 / USER_ECU_PERIOD))
	{
		u16AlarmCnt++;
		if(NORMAL_MODE == u8RunMode)
		{
			if((1 == sgPcuKeyInit.b1Enable)&&(1 == sgPcuInfo.PcuKeyInfo.b1Enable))
			{
				i32ErrCodeSet(ENABLE_BUTTON_ERR);
			}
			if((1 == sgPcuKeyInit.b1HandleValue)&&(0 != sgPcuInfo.i16HandleValue))
			{
//				i32ErrCodeSet(HANDLE_NOT_IN_MIDDLE_POSI_ERR);
			}
			if((1 == sgPcuKeyInit.b1LiftMode)&&(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode))
			{
				i32ErrCodeSet(LIFT_BUTTON_ERR);
			}
			if((1 == sgPcuKeyInit.b1MoveMode)&&(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode))
			{
				i32ErrCodeSet(MOVE_BUTTON_ERR);
			}
			if((1 == sgPcuKeyInit.b1Slow)&&(1 == sgPcuInfo.PcuKeyInfo.b1Slow))
			{
				i32ErrCodeSet(SLOW_BUTTON_ERR);
			}
			if((1 == sgPcuKeyInit.b1TurnLeft)&&(1 == sgPcuInfo.PcuKeyInfo.b1TurnLeft))
			{
				i32ErrCodeSet(PLAT_LEFT_BUTTON_ERR);
			}
			if((1 == sgPcuKeyInit.b1TurnRight)&&(1 == sgPcuInfo.PcuKeyInfo.b1TurnRight))
			{
				i32ErrCodeSet(PLAT_RIGHT_BUTTON_ERR);
			}
			if(abs(sgPcuInfo.i16HandleValue) > 0)
			{
				i32ErrCodeSet(HANDLE_NOT_IN_MIDDLE_POSI_ERR);
			}
		}
	}

	if((0 != sgPcuKeyInit.u8data)&&(PARASETS_DISABLE == u8ParaSetMode))//�ϵ�ʱ�ֱ���������Ϊ0
	{
		if((1 == sgSwiInput.b1PcuSwi)
			&&(1 == sgPcuKeyInit.b1Speaker)
			&&(1 == sgPcuKeyInit.b1LiftMode)
			&&(1 == sgPcuKeyInit.b1Slow)
//			&&(0 == sgPcuInfo.PcuKeyInfo.b1Speaker)
//			&&(0 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
//			&&(0 == sgPcuInfo.PcuKeyInfo.b1Slow)
			)//ƽ̨λ�ã������ȣ�����������,Ȼ���ɿ�  ͨ�ò���
		{
			u8RunMode = PARA_SETS_MODE;      //����ģʽ����Ҫ����������
			u8ParaSetMode = COMMON_PARASET;
			i32ErrCodeClr(LIFT_BUTTON_ERR);
			i32ErrCodeClr(SLOW_BUTTON_ERR);
			i32ErrCodeClr(MOVE_BUTTON_ERR);
		}
		if((1 == sgSwiInput.b1PcuSwi)
			&&(0 == sgPcuKeyInit.b1Slow)
			&&(1 == sgPcuKeyInit.b1Speaker)
			&&(1 == sgPcuKeyInit.b1LiftMode)
//			&&(0 == sgPcuInfo.PcuKeyInfo.b1Speaker)
//			&&(0 == sgPcuInfo.PcuKeyInfo.b1LiftMode)//���Ⱦ��� �ٶ�����
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
//			&&(0  == sgPcuInfo.PcuKeyInfo.b1MoveMode)
//			&&(0 == sgPcuInfo.PcuKeyInfo.b1Slow)//��������H9
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
//				&&(0 == sgPcuInfo.PcuKeyInfo.b1Speaker)
//				&&(0 == sgPcuInfo.PcuKeyInfo.b1Slow)
		)//�������� H8ģʽ
		{
			u8RunMode = PARA_SETS_MODE;
			u8ParaSetMode = H8_MODE;		
			i32ErrCodeClr(LIFT_BUTTON_ERR);
			i32ErrCodeClr(SLOW_BUTTON_ERR);
			i32ErrCodeClr(MOVE_BUTTON_ERR);						
		}
		
		//��������
	}
	else if((SPEED_PARASET == u8ParaSetMode))//�ٶ�ģʽ���������͹��������
	{
		static uint16_t u16KeyCnt=0;
		if((u16KeyCnt < (3000 / TIMER_PLC_PERIOD))
			&&(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
			&&(1 == sgPcuInfo.PcuKeyInfo.b1Slow))
			u16KeyCnt++;
		else if((u16KeyCnt == (3000 / TIMER_PLC_PERIOD)))//&&(0 == sgPcuInfo.PcuKeyInfo.u8data))
			u8ParaSetMode = MACHINE_PARASET;
		
//		i32SetPara(PARA_BrakeValveCurrent,u16KeyCnt);
	}
	else if((H8_MODE == u8ParaSetMode)&&(1 == sgPcuInfo.PcuKeyInfo.b1Enable))
	{
		u8RunMode = H2_MODE;
	}
	else if((NORMAL_MODE == u8RunMode)&&(1 == sgSwiInput.b1PcuSwi))//����ģʽ����궨״̬
	{
		//�Ѳ���
		static uint32_t	u32KeyRecord = 0;//����Ϊ1������Ϊ0
		static xSwiInput	KeyState ;  //��¼��һ�ο���״̬
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
	}
	#ifdef LIUGONG_TEST
 else if((NORMAL_MODE == u8RunMode)&&(0 == sgSwiInput.b1PcuSwi))  //�¿�ģʽ�´����Զ���������
 {
	 static uint32_t	u32KeyRecord = 0;//����Ϊ1������Ϊ0
	 static xSwiInput	KeyState ;
	 static uint8_t	u8SwitchCnt = 0;
	
	#define AUTO_LIFT		0b11111

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
	if((0 != (u32KeyRecord ^ (AUTO_LIFT >>( 5 - u8SwitchCnt))))
		||(true == u8GetNetTimerOverFlag(TIMER_SwitchCheck)))//����������ͬ��ʱ,�������
	{
		u32KeyRecord = 0;
		u8SwitchCnt = 0;
		vKillNetTimer(TIMER_SwitchCheck);
	}
	if(0 == (u32KeyRecord ^ AUTO_LIFT))//�л�ģʽ�������ʱ��
	{
		u8RunMode = AUTO_LIFT_MODE;
		vKillNetTimer(TIMER_SwitchCheck);
	}
}	 
	#endif
	}
//	else if((PRESSURE_CALIBRATION_MODE == u8RunMode)&&(true == ))//�궨��ɣ�������һ��
//	{
//		
//	}
//		i32SetPara(PARA_OnOffValveCurrent,u8RunMode);

//	sgPcuKeyInit.b1MoveMode = 1;



//����sgActLogic
static void vActionMonitNormal()
{
	if(1 == sgSwiInput.b1PcuSwi)
	{
		if(INITIAL_MODE == u8PcuMode)
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
			&& (0 == sgPcuInfo.PcuKeyInfo.b1Slow)
			&& (1 == PCUStateRecord.PcuKeyInfo.b1Slow))
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
				if(0 == sgLimit.b1NoTurn)
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
//				if(FunctionEnable == gUserInfo.u8LiftReverseFunc)
//				{	
//					sgPcuInfo.i16HandleValue = -sgPcuInfo.i16HandleValue;
//				}
				
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
		u8PcuMode = LIFT_MODE;
		if((1 == i32LocalDiGet(LOWER_CONTROLL_UP))&&(0 == sgLimit.b1NoLift))
		{
			sgActLogic.b1LiftUpAct = 1;
			u16MotorVal = MOTOR_MAX_SPEED_VALUE;
		}
		else if((1 == i32LocalDiGet(LOWER_CONTROLL_DOWN))&&(0 == sgLimit.b1NoDown))
		{
			sgActLogic.b1LiftDownAct = 1;
			u16MotorVal = MOTOR_MAX_SPEED_VALUE;
		}
		else
		{
			u16MotorVal = 0;
		}
	}
	
	if(((1 == sgSwiInput.b1PitSwi || sgSwiInput.b1LowestState )&&(sgUserSets.b1LiftBanMove))
	//	||(1 == sgUserSets.b1LowLimitBanMoveEn)
	//	&&((i32LocalAiGetValue(ANGLE_SENSOR_CHANNEL)) > (i32GetPara(PARA_LOWLIMIT_ANGLE) +i32GetPara(PARA_LOWLIMIT_RANGE)))
	)
	{
		sgActLogic.b1BackwardAct = 0;
		sgActLogic.b1ForwardAct = 0;
		sgActLogic.b1TurnLeft = 0;
		sgActLogic.b1TurnRight = 0;
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
							||((0 == sgSwiInput.b1PcuSwi)&&(0 == i32LocalDiGet(LOWER_CONTROLL_DOWN))))
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
					u16tmp = (gUserInfo.u8AccAndDecAntiPinch * 1000);
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
	i32SetPara(PARA_OnOffValveCurrent,u8AntiPinchState);
}

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
	sgActLogic.u8Data = 0;
	if(FUNCTION_PARASET == u8ParaSetMode)
	{
		if((1 == i32LocalDiGet(LOWER_CONTROLL_UP))
		//	&&(0 == sgLimit.b1NoLift)
		)
		{
			sgActLogic.b1LiftUpAct = 1;
			u16MotorVal = MOTOR_MAX_SPEED_VALUE;
		}
		else if((1 == i32LocalDiGet(LOWER_CONTROLL_DOWN))
			//&&(0 == sgLimit.b1NoDown)
		)
		{
			sgActLogic.b1LiftDownAct = 1;
			u16MotorVal = MOTOR_MAX_SPEED_VALUE;
		}
		else
		{
			u16MotorVal = 0;
		}
	}
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
	
//#define EEPROME_DATA	0
//#define COMBINE_DATA	1
static uint16_t u16PcuParaMonit(xPcuSendPara *SendData,uint16_t u16Index,
uint8_t u8MinNum,uint8_t u8MaxNum)
{
	static uint16_t u16IndexREC = 0;
	static uint16_t u16ParaValue = 0;
	static uint8_t	u8SaveCnt = 0;
	static xPcuKeyInfo PcuKeyRec ;
	
	static uint8_t u8PressCount;
	static uint8_t u8NumIncreaseCnt;
	
	
	if(u16Index != u16IndexREC)//��������ת�䣬�洢�ɲ�������ȡ�²���
	{
		if(0 != u16IndexREC)
		{
			u16SaveParaToEeprom(u16IndexREC , u16ParaValue);
		}
		u16IndexREC = u16Index;
		u16ParaValue = i32GetPara(u16IndexREC);
	}
	else
	{
		u8SaveCnt ++ ;
		if((1 == PcuKeyRec.b1TurnLeft)||(1 == PcuKeyRec.b1TurnRight))//������������
		{
			if(u8NumIncreaseCnt<15)
			{
				u8NumIncreaseCnt++;
			}
			else
			{
				if(u8PressCount <= 1)
					u8PressCount++;
				else
					u8PressCount = 0;				
			}
		}
		else
		{
			u8NumIncreaseCnt = 0;
			u8PressCount = 0;
		}
		
		if(((0 == sgPcuInfo.PcuKeyInfo.b1TurnLeft)&&(1 == PcuKeyRec.b1TurnLeft))
			||((1 == u8PressCount)&&(1 == PcuKeyRec.b1TurnLeft)))
		{
			u8SaveCnt = 0;
			if(u16ParaValue == u8MinNum)
				u16ParaValue = u8MaxNum;
			else			
				u16ParaValue --;
		}
		
		if(((0 == sgPcuInfo.PcuKeyInfo.b1TurnRight)&&(1 == PcuKeyRec.b1TurnRight))
			||((1 == u8PressCount)&&(1 == PcuKeyRec.b1TurnRight)))
		{
			u16ParaValue ++;
			u8SaveCnt = 0;
			if(u16ParaValue> u8MaxNum)
				u16ParaValue = u8MinNum;			
		}

		if(u8SaveCnt >= 20)//ÿ1s��һ�Σ���ֹͻȻ����
		{
			u8SaveCnt = 0;
			if(u16ParaValue != (i32GetPara(u16IndexREC)))
			u16SaveParaToEeprom(u16IndexREC , u16ParaValue);
			i32SetPara(u16IndexREC,u16ParaValue);
		}
	}	
	vPcuDisplayNumber(SendData,u16ParaValue,DISPLAY_NORMAL);
	
	PcuKeyRec.u8data = sgPcuInfo.PcuKeyInfo.u8data;
	return u16ParaValue;
}

/*MCU�̻�����*/
static uint16_t MCU_Para6m[][2] = {
{1,1},
{2,1},
{3,5},
{4,0},
{5,100},
{6,0},
{7,2},
{8,32},
{9,48},
{10,60},
{11,90},
{12,239},
{13,13000},
{14,9830},
{15,7000},
{16,13000},
{17,7000},
{18,3500},
{19,26800},
{20,32000},
{21,30000},
{22,30000},
{23,26800},
{24,32767},
{25,26800},
{26,22000},
{27,17621},
{28,12922},
{29,32767},
{30,26800},
{31,22000},
{32,17621},
{33,12922},
{34,32767},
{35,32767},
{36,6144},
{37,1},
{38,0},
{39,120},
{40,140},
{41,30000},
{48,12000},
{49,12000},
{50,600},
{51,20},
{52,50},
{53,64},
{54,96},
{55,71},
{56,300},
{57,500},
{58,100},
{59,0},
{60,400},
{61,500},
{62,200},
{63,2000},
{64,2000},
{65,3000},
{66,1000},
{67,1000},
{68,1000},
{69,1000},
{70,2000},
{71,3000},
{72,16000},
{73,10000},
{74,300},
{75,500},
{76,500},
{77,100},
{78,100},
{79,100},
{80,100},
{81,700},
{82,10},
{83,10},
{84,5000},
{85,3},
{86,80},
{87,4096},
{88,4096},
{89,3072},
{90,3072},
{106,1638},
{91,3},
{92,3},
{93,95},
{94,50},
{95,0},
{96,4},
{97,100},
{98,50},
{99,0},
{100,10},
{101,0},
{102,5},
{103,100},
{104,50},
{105,0},
{121,920},
{122,0},
{123,0},
{124,30},
{125,30},
{126,30},
{127,200},
{131,3},
{132,0},
{133,100},
{134,80},
{135,0},
{136,1},
{137,100},
{138,80},
{139,100},
{140,80},
{141,0},
{142,100},
{143,80},
{144,0},
{145,5},
{146,0},
{151,8192},
{152,2048},
{153,0},
{154,0},
{155,5},
{156,0},
{157,250},
{158,0},
{159,5000},
{160,3000},
{161,1000},
{190,0},
{191,100},
{192,15},
{193,1},
{194,1},
{195,384},
{196,384},
{197,384},
{198,0},
{199,0}
};

static uint16_t MCU_Para5m[][2] ={
{1,1},
{2,1},
{3,5},
{4,0},
{5,100},
{6,0},
{7,2},
{8,32},
{9,48},
{10,120},
{11,180},
{12,239},
{13,7500},
{14,9830},
{15,7000},
{16,13000},
{17,7000},
{18,3500},
{19,26800},
{20,32000},
{21,26800},
{22,26800},
{23,26800},
{24,32767},
{25,26800},
{26,22000},
{27,17621},
{28,12922},
{29,32767},
{30,26800},
{31,22000},
{32,17621},
{33,12922},
{34,32767},
{35,32767},
{36,6144},
{37,1},
{38,0},
{39,120},
{40,140},
{41,30000},
{48,12000},
{49,12000},
{50,700},
{51,20},
{52,50},
{53,64},
{54,96},
{55,71},
{56,300},
{57,500},
{58,100},
{59,0},
{60,400},
{61,500},
{62,200},
{63,2000},
{64,2000},
{65,3000},
{66,1000},
{67,1000},
{68,1000},
{69,1000},
{70,2000},
{71,3000},
{72,20000},
{73,10000},
{74,600},
{75,500},
{76,500},
{77,100},
{78,100},
{79,100},
{80,100},
{81,700},
{82,10},
{83,10},
{84,5000},
{85,3},
{86,80},
{87,4096},
{88,4096},
{89,3072},
{90,3072},
{106,1638},
{91,3},
{92,3},
{93,95},
{94,50},
{95,0},
{96,4},
{97,100},
{98,50},
{99,0},
{100,10},
{101,0},
{102,5},
{103,100},
{104,50},
{105,0},
{121,920},
{122,0},
{123,0},
{124,30},
{125,30},
{126,30},
{127,200},
{131,3},
{132,0},
{133,100},
{134,80},
{135,0},
{136,1},
{137,100},
{138,80},
{139,100},
{140,80},
{141,0},
{142,100},
{143,80},
{144,0},
{145,5},
{146,0},
{151,8192},
{152,2048},
{153,0},
{154,0},
{155,5},
{156,0},
{157,250},
{158,0},
{159,5000},
{160,3000},
{161,1000},
{190,0},
{191,100},
{192,15},
{193,1},
{194,1},
{195,384},
{196,384},
{197,384},
{198,0},
{199,0}	
};






const static uint8_t u8CarTypeSwitch[]=
{
	0b1100,0b1101,0b1110,0b1111,//48��49��50��51
	0b1100,0b1101,0b1110,0b1111,//52��53��54��54
	0b1000,0b1001,0b1010,0b1011,//56��57��58��59
	0b1000,0b1001,0b1010,0b1011,//60��61��62��63
	
	
//	0b1000,0b1001,0b1010,0b1011,//64��65��66��67
//	0b1100,0b1101,0b1110,0b1111,//68��69��70��71
//	0b0000,0b0001,0b0010,0b0011,//72��73��74��75
//	0b0100,0b0101,0b0110,0b0111,//76��77��78��79
	0b00001,0b00011,0b00101,0b00111,0b01001,0b01011,0b01101,0b01111,//64~71
	0b10001,0b10011,0b10101,0b10111,0b11001,0b11011,0b11101,0b11111,//72~79
	0b00001,0b00011,0b00101,0b00111,0b01001,0b01011,0b01101,0b01111,//80~87
	0b10001,0b10011,0b10101,0b10111,0b11001,0b11011,0b11101,0b11111,//88~95
};//���ݳ��������ò�ͬ�Ĺ���ѡ��Ӷ��������֡���������������
//24.1.28������޸�Ϊ���ء������֡��߿����С�����ײ
//24.3.12ǰ48��63�����뱱��һ��
//24.3.1864֮��������������
static void vChangeCarType(void)
{
	uint8_t u8CarType = 0;
	u8CarType = i32GetPara(PARA_VehicleType);
	if(gUserInfo.u8VehicleType != u8CarType)
	{
		uint8_t u8tmp = 0;
		u8tmp = u8CarTypeSwitch[(u8CarType - 48)];
		if(64 <= u8CarType)
		{
			if(0 != (u8tmp&0b10000))//����
				u16SaveParaToEeprom(PARA_WeighFunc, FunctionEnable);
			else
				u16SaveParaToEeprom(PARA_WeighFunc, FunctionDisable);
			
			if(0 != (u8tmp&0b01000))//������
				u16SaveParaToEeprom(PARA_AntiPinchFunc, FunctionEnable);
			else
				u16SaveParaToEeprom(PARA_AntiPinchFunc, FunctionDisable);
			
			if(0 != (u8tmp&0b00100))//�߿�������
				sgUserSets.b1LiftBanMove = 1;
			else
				sgUserSets.b1LiftBanMove = 0;
			u16SaveParaToEeprom(PARA_USERSETS,sgUserSets.u16Data);
					
			if(0 != (u8tmp&0b00010))//����ײ
				u16SaveParaToEeprom(PARA_AnticollisionFunc, FunctionEnable);
			else
				u16SaveParaToEeprom(PARA_AnticollisionFunc, FunctionDisable);		
			
			if(0 != (u8tmp&0b00001))//��������
				u16SaveParaToEeprom(PARA_ActAlmFunc, FunctionEnable);
			else
				u16SaveParaToEeprom(PARA_ActAlmFunc, FunctionDisable);					
		}
		else
		{
			if(0 != (u8tmp&0b1000))//�Ӷ�
				u16SaveParaToEeprom(PARA_PitProtectFunc, FunctionEnable);
			else
				u16SaveParaToEeprom(PARA_PitProtectFunc, FunctionDisable);		
			
			if(0 != (u8tmp&0b0100))//������
				u16SaveParaToEeprom(PARA_AntiPinchFunc, FunctionEnable);
			else
				u16SaveParaToEeprom(PARA_AntiPinchFunc, FunctionDisable);
			
			if(0 != (u8tmp&0b0010))//��������
				u16SaveParaToEeprom(PARA_ActAlmFunc, FunctionEnable);
			else
				u16SaveParaToEeprom(PARA_ActAlmFunc, FunctionDisable);		
			
			if(0 != (u8tmp&0b0001))//����
				u16SaveParaToEeprom(PARA_WeighFunc, FunctionEnable);
			else
				u16SaveParaToEeprom(PARA_WeighFunc, FunctionDisable);
		}
	}
}

static void vChangeCarTypePara(uint16_t u16Carcode);
static void vChangeMcuPara();



//PCU��˸��ʾ
static void	vPcuParaSetProc(xPcuSendPara *SendData)//����Ϊ���ڣ�A->��->B->��->A
{
	static uint8_t u8DisplayCnt = 0;
	static uint8_t u8DisplayPage = 0;
	static uint8_t u8SwitchDelay = 0;
	static uint8_t u8SubPage = 0;
	static uint8_t u8ParaSetModeRec = 0;
<<<<<<< .mine
	static uint16_t u16Carcode;
	static uint16_t u16CarCodeOld;
	u16EepromRead(CARCODE_HISTORY,&u16CarCodeOld,1);
||||||| .r31
	
=======
	static uint16_t u16Carcode;

>>>>>>> .r49
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
	#define PS_LOWRANGE	9


	#define SC_INIT					0
	#define SC_SETSL				1
	#define SC_SETSR				2
	#define SC_SAVE_SUCCESS	3	
		
	/*CPģʽ���ֲ����޸�*/
	#define CP_INIT									0
	#define CP_LIFTACC						1
	#define CP_LIFTDEC						2
	#define CP_MAX_PRESSUREDIFF			3
	#define CP_DYN_OVERRLOADRATE		4
	#define	CP_DYN_OVERLOADDELAY		5
	#define	CP_STB_OVERLOADRATE			6
	#define CP_MOVEACC				7
	#define	CP_MOVEDEC			8
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
		vKillNetTimer(TIMER_SwitchCheck);
	}
	
	//ͨ�����
	#if(PCU_TYPE_LZ == PCU_TYPE)
	if((1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)&&(0 == PcuKeyRec2.b1LiftMode))//����������
	{
		SendData->Data.b1ModeLed = 1;
		SendData->Data.b1LiftLed = 0;
	}
	if((1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)&&(0 == PcuKeyRec2.b1LiftMode))//�������ߵ�
	{
		SendData->Data.b1ModeLed = 0;
		SendData->Data.b1LiftLed = 1;
	}
	#elif(PCU_TYPE_XUGONG == PCU_TYPE)
	if((1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)&&(0 == PcuKeyRec2.b1LiftMode))//����������
	{
		SendData->Data.b4ModeControl = 0x7;
	}
	if((1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)&&(0 == PcuKeyRec2.b1MoveMode))//�������ߵ�
	{
		SendData->Data.b4ModeControl = 0xD;
	}
	#endif

	
	switch(u8ParaSetMode)
	{
		case PARASETS_DISABLE://����ģʽ��ʾ
		{
			if(1 == u8EcuProcFlag)
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
				#if(PCU_TYPE_LZ == PCU_TYPE)
				if(MOVE_MODE == u8PcuMode)
				{
					SendData->Data.b1ModeLed = 1;
					SendData->Data.b1LiftLed = 0;
				}
				else if(LIFT_MODE == u8PcuMode)
				{
					SendData->Data.b1ModeLed = 0;
					SendData->Data.b1LiftLed = 1;
				}
				else
				{
					SendData->Data.b1ModeLed = 0;
					SendData->Data.b1LiftLed = 0;				
				}
				#elif(PCU_TYPE_XUGONG == PCU_TYPE)
				if(MOVE_MODE == u8PcuMode)//���ߵƿ��Ʒ����ݲ�֪��
				{
						//12.2 TEST.�칤DD����ģʽ��D7����ģʽ
					SendData->Data.b4ModeControl = 0xD;
				}
				else if(LIFT_MODE == u8PcuMode)
				{
					SendData->Data.b4ModeControl = 0x7;
				}
				else
				{
					SendData->Data.b4ModeControl = 0b0101;//ʵ�ʲ��ԣ�0101ʱ�Ʋ���
				}
				#endif
				if((1 == sgErrorState.b1Error)&&(1 == sgErrorState.PCUBeep))//�ֱ�������������Ʒ�����
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
					#if 0
					if(1 == sgActLogic.b1LiftUpAct)//������ʾ��������
						vPcuDisplayNumber(SendData,gUserInfo.u8VehicleType,DISPLAY_NORMAL);
					
					if(1 == sgActLogic.b1LiftDownAct)//�½���ʾ����汾
						vPcuDisplayNumber(SendData,0xC1,HEX_DISPLAY);		
				#endif					
				}				
			}
			else
			{
				vPcuNull(SendData);
				SendData->Data.b4ModeControl = 0b0101;//ʵ�ʲ��ԣ�0101ʱ�Ʋ���
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
					if((1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)&&(0 == PcuKeyRec2.b1LiftMode))
						u8DisplayPage = CP_LIFTACC;
					if((1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)&&(0 == PcuKeyRec2.b1MoveMode))
						u8DisplayPage = CP_LIFTDEC;
					break;
				case CP_LIFTACC://����
					u16PcuParaMonit(SendData,PARA_AccAndDecLift,1,100);
					
					if(1 == sgPcuInfo.PcuKeyInfo.b1Enable)//ʹ��->�����ز�ֵ
						u8DisplayPage = CP_MAX_PRESSUREDIFF;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Speaker)//����->��̬����
						u8DisplayPage = CP_DYN_OVERRLOADRATE;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Slow)//����->�����ȶ���ʱ
						u8DisplayPage = CP_DYN_OVERLOADDELAY;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LIFTDEC;
					break;
				case CP_MAX_PRESSUREDIFF://���� ʹ��
					u16PcuParaMonit(SendData,PARA_MaxDifferencePercent,0,100);
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
						u8DisplayPage = CP_LIFTACC;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LIFTDEC;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Speaker)//����
						u8DisplayPage = CP_STB_OVERLOADRATE;
					break;
				case CP_DYN_OVERRLOADRATE://���� ����
					u16PcuParaMonit(SendData,PARA_DynamicOverLoadPercent,0,20);
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
						u8DisplayPage = CP_LIFTACC;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LIFTDEC;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Enable)//ʹ��
						u8DisplayPage = CP_STB_OVERLOADRATE;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Slow)//����
						u8DisplayPage = CP_MOVEACC;
					break;
				case CP_DYN_OVERLOADDELAY://���� ����
					u16PcuParaMonit(SendData,PARA_OverLoadStabilityDelay,0,100);
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
						u8DisplayPage = CP_LIFTACC;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LIFTDEC;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Speaker)//����
						u8DisplayPage = CP_MOVEACC;
					break;
				case CP_STB_OVERLOADRATE://���� ���� ʹ��
					u16PcuParaMonit(SendData,PARA_StaticOverLoadPercent,0,20);
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
						u8DisplayPage = CP_LIFTACC;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LIFTDEC;				
					if(1 == sgPcuInfo.PcuKeyInfo.b1Slow)//����
						u8DisplayPage = CP_ANTI_DELAYTIME;	
					break;
				case CP_MOVEACC://���� ���� ����
					u16PcuParaMonit(SendData,PARA_AccAndDecFastDrive,1,100);
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
						u8DisplayPage = CP_LIFTACC;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LIFTDEC;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Enable)//ʹ��
						u8DisplayPage = CP_ANTI_DELAYTIME;	
					break;
				case CP_ANTI_DELAYTIME:// ���� ���� ���� ʹ��
					u16PcuParaMonit(SendData,PARA_AccAndDecAntiPinch,1,20);
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
						u8DisplayPage = CP_LIFTACC;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LIFTDEC;
					break;
				case CP_LIFTDEC://����
					u16PcuParaMonit(SendData,PARA_BrakeLift,1,100);
					if(1 == sgPcuInfo.PcuKeyInfo.b1Speaker)//����->������������
						u8DisplayPage = CP_ALARMVOLUE;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Slow)//����->����
						u8DisplayPage = CP_UPCONTROL_SLEEPTIME;
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)//
						u8DisplayPage = CP_LIFTACC;
					break;
				case	CP_ALARMVOLUE://���� ����
					u16PcuParaMonit(SendData,PARA_VoiceAlarmVolume,0,28);
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
						u8DisplayPage = CP_LIFTACC;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LIFTDEC;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Slow)
						u8DisplayPage = CP_MOVEDEC;
					break;				
				case CP_UPCONTROL_SLEEPTIME://���� ����
					u16PcuParaMonit(SendData,PARA_UpperCtlButSleep,0,60);
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
						u8DisplayPage = CP_LIFTACC;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LIFTDEC;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Speaker)//����
						u8DisplayPage = CP_MOVEDEC;
					break;
				case CP_MOVEDEC://���� ���� ����
					u16PcuParaMonit(SendData,PARA_BrakeFastDrive,1,100);
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
						u8DisplayPage = CP_LIFTACC;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LIFTDEC;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Enable)//ʹ��
						u8DisplayPage = CP_DOWN_DEC;
					break;
				case CP_DOWN_DEC:// ���� ���� ���� ʹ��
					u16PcuParaMonit(SendData,PARA_BrakeLower,0,40);
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
						u8DisplayPage = CP_LIFTACC;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LIFTDEC;
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
					
					if(((1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)&&(0 ==PcuKeyRec2.b1LiftMode)))
						u8DisplayPage = PS_LIFT;
					break;
				case PS_LOWRANGE:
					u16PcuParaMonit(SendData,PARA_LOWLIMIT_RANGE,0,20);
					if (1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
					{
						u8DisplayPage = PS_LIFT;
					}
					else if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
					{
						u8DisplayPage =  PS_FAST;	
					}						
					break;
				case PS_LIFT://���������ٶ�
					u16PcuParaMonit(SendData,PARA_LiftSpeed,0,100);
					if(1 == sgPcuInfo.PcuKeyInfo.b1Enable)//����->�½�������->�����֣�����->
					{	
						static uint8_t u8DelayCnt ;
						if(u8DelayCnt < 5)//��ʱ�ж�
						{
							u8DelayCnt ++;
						}
						else
						{
							u8DelayCnt = 0;
							
							if((0 == sgPcuInfo.PcuKeyInfo.b1Speaker)&&(1 == sgPcuInfo.PcuKeyInfo.b1Slow))
								u8DisplayPage = PS_ANTI;
							else if((1 == sgPcuInfo.PcuKeyInfo.b1Speaker)&&(1 == sgPcuInfo.PcuKeyInfo.b1Slow))
								u8DisplayPage =  PS_LOWRANGE;
							else if(1 == sgPcuInfo.PcuKeyInfo.b1Speaker)
								u8DisplayPage =  PS_DOWN;							
						}
					}
					else if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage =  PS_FAST;
					break;
				case PS_DOWN:
					u16PcuParaMonit(SendData,PARA_LowerSpeed,0,100);
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
					u16PcuParaMonit(SendData,PARA_BrakeAntiPinch,0,100);		
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
					u16PcuParaMonit(SendData,PARA_FastDriveSpeed,0,100);
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
					u16PcuParaMonit(SendData,PARA_SlowDriveSpeed,0,100);
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
					u16PcuParaMonit(SendData,PARA_DriveSpeedAfterLift,0,80);
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
					u16PcuParaMonit(SendData,PARA_TurnPowerLimit,0,80);
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
					else if(u8DisplayCnt < 10)
						vPcuSC(SendData);
					else
					{
						vPcuSC(SendData);
						u8DisplayCnt = 0;
					}
					
					u8CombinedData = gUserInfo.u8VehicleType;

					if((1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)&&(0 == PcuKeyRec2.b1LiftMode))
					{
						u32CombinedDataH9 = 0;
						u8DisplayPage = SC_SETSL;
					}
						
					if((1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)&&(0 == PcuKeyRec2.b1MoveMode))
					{
						u32CombinedDataH9 = 0;
						u8DisplayPage = SC_SETSR;
					}
						
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
					if(u8CombinedData<48)
						u8CombinedData = 48;
					
					if(u8CombinedData>95)
						u8CombinedData = 95;
										
					u8DisplayCnt++;
					if(u8DisplayCnt < 4)
						vPcuDisplayNumber(SendData,u8CombinedData,MASK_LEFT);
					else if(u8DisplayCnt < 10)
						vPcuDisplayNumber(SendData,u8CombinedData,DISPLAY_NORMAL);
					else
					{
						vPcuDisplayNumber(SendData,u8CombinedData,DISPLAY_NORMAL);
						u8DisplayCnt = 0;
					}
					
					if(1 == sgPcuInfo.PcuKeyInfo.b1Speaker)//�������ȱ���
					{
						u32CombinedDataH9++;
						if(u32CombinedDataH9 > 60)
						{
							if((u8CombinedData != i32GetPara(PARA_VehicleType)))
							{
								u16SaveParaToEeprom(PARA_VehicleType,u8CombinedData);
								i32SetPara(PARA_VehicleType,u8CombinedData);
								vChangeCarType();
							}
							sgErrorState.b1SaveSuccess =1;
							u8DisplayPage = SC_SAVE_SUCCESS;						
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
					if(u8CombinedData<48)
						u8CombinedData = 48;
					
					if(u8CombinedData>95)
						u8CombinedData = 95;
										
					u8DisplayCnt++;
					if(u8DisplayCnt < 4)
						vPcuDisplayNumber(SendData,u8CombinedData,MASK_RIGHT);
					else if(u8DisplayCnt < 10)
						vPcuDisplayNumber(SendData,u8CombinedData,DISPLAY_NORMAL);
					else
					{
						vPcuDisplayNumber(SendData,u8CombinedData,DISPLAY_NORMAL);
						u8DisplayCnt = 0;
					}
					
					if(1 == sgPcuInfo.PcuKeyInfo.b1Speaker)//�������ȱ���
					{
						u32CombinedDataH9++;
						if(u32CombinedDataH9 > 60)
						{
							if((u8CombinedData != i32GetPara(PARA_VehicleType)))
							{
								u16SaveParaToEeprom(PARA_VehicleType,u8CombinedData);
								i32SetPara(PARA_VehicleType,u8CombinedData);
								vChangeCarType();
								
							}
							sgErrorState.b1SaveSuccess =1;
							u8DisplayPage = SC_SAVE_SUCCESS;						
						}
					}
					if((1 == sgPcuInfo.PcuKeyInfo.b1LiftMode))
						u8DisplayPage = SC_SETSL;
					break;
				case SC_SAVE_SUCCESS:
					u8DisplayCnt++;
					if(0 == sgPcuInfo.PcuKeyInfo.b1Speaker)
					{
						sgErrorState.b1SaveSuccess = 0;
					}
					if(u8DisplayCnt < 4)
						vPcuNull(SendData);
					else if(u8DisplayCnt < 10)
						vPcuDisplayOrigin(SendData,0x3E,0x39);//��ʾUC
					else
					{
						vPcuDisplayOrigin(SendData,0x3E,0x39);//��ʾUC
						u8DisplayCnt = 0;
					}
					break;
				default:
					break;
			}
			break;
		case FUNCTION_PARASET://H9�����ù������ü��Ƕ�ģ����λ
			{
				SendData->Data.b4ModeControl = 0b0101;
				SendData->Data.b1SlowLed = 0;
				static uint16_t u16VerifyCnt;
				if((u8DisplayPage == H9_INIT)
					&&(1 == sgPcuInfo.PcuKeyInfo.b1Speaker)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1Enable)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1Slow)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1TurnLeft)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1TurnRight))
				{
					u16VerifyCnt++;
				}
				if((u8DisplayPage == H9_INIT)
					&&(u16VerifyCnt >(5000 / PCU_DISPLAY_PERIOD)))
				{
					u8DisplayPage = H9_VERIFY;
					u16VerifyCnt = 0;
				}
				if(u8DisplayPage == H9_VERIFY)
				{
					u16VerifyCnt++;
					if(u16VerifyCnt <= (3000 / PCU_DISPLAY_PERIOD))
					{
						sgErrorState.b1Verified = 1;
					}
					else
					{
						sgErrorState.b1Verified = 0;
					}
				}
				else
				{
					sgErrorState.b1Verified = 0;
				}
				if((1 == sgPcuInfo.PcuKeyInfo.b1Slow)
					&&(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1Enable)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1Speaker)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1TurnLeft)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1TurnRight)
					&&(u8DisplayPage != H9_INIT))//���¹��ٺ�����
				{
					if(H9_LOW16CHOOSE != u8DisplayPage)
						u8SubPage = 3;//���������л���������ʾ��������һ��
					u8DisplayPage = H9_LOW16CHOOSE;
					vKillNetTimer(TIMER_SwitchCheck);
				}
				if((1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
					&&(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1Slow)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1Enable)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1Speaker)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1TurnLeft)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1TurnRight)				
					&&(u8DisplayPage != H9_INIT))//�������ߺ�����
				{
					if(H9_HIG16CHOOSE != u8DisplayPage)
						u8SubPage = 3;//���������л���������ʾ��������һ��
					u8DisplayPage = H9_HIG16CHOOSE;
					vKillNetTimer(TIMER_SwitchCheck);
				}
				if((1 == sgPcuInfo.PcuKeyInfo.b1Speaker)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1Slow)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1Enable)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1TurnLeft)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1TurnRight)		
					&&(u8DisplayPage != H9_INIT)
					&&(u8DisplayPage != H9_VERIFY)
					&&(u8DisplayPage != H9_SAVE_SUCCESS))//�ǳ�ʼ���׶Σ����뱣��
				{
					u8DisplayPage = H9_SAVE;
				}
				if((1 == sgPcuInfo.PcuKeyInfo.b1Enable)
					&&(1 == sgPcuInfo.PcuKeyInfo.b1TurnRight)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1Slow)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1Speaker)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1TurnLeft)		
				)//ʹ��+��ת ����λ
				{
					if(H9_SAVE_SUCCESS != u8DisplayPage)
					{
						if(H9_DOWNLIMIT != u8DisplayPage)
							u8DisplayCnt = 0;//���������л���������ʾ��������һ��
						u8DisplayPage = H9_DOWNLIMIT;			
					}
				}
				if((1 == sgPcuInfo.PcuKeyInfo.b1Enable)
					&&(1 == sgPcuInfo.PcuKeyInfo.b1TurnLeft)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1Slow)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1Speaker)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1TurnRight)		
				)//ʹ��+��ת ����λ
				{
					if(H9_SAVE_SUCCESS != u8DisplayPage)
					{
					if(H9_UPLIMIT != u8DisplayPage)
						u8DisplayCnt = 0;//���������л���������ʾ��������һ��
					u8DisplayPage = H9_UPLIMIT;
					}
				}
				//��ס����ʹ�����ȣ��궨��͵�
				if( (1 == sgPcuInfo.PcuKeyInfo.b1Slow)
					&&(1 == sgPcuInfo.PcuKeyInfo.b1Enable)
					&&(1 == sgPcuInfo.PcuKeyInfo.b1Speaker)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1TurnLeft)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1TurnRight)		
				)
				{
					static uint8_t u8LCnt;
					if(H9_INIT == u8DisplayPage)
					{
						u8LCnt++;
						if(u8LCnt > 30)
						{
							u8DisplayPage = H9_LOWLIMIT_CALI;
							u8LCnt = 0;
						}
					}
				}
				if((1 == sgPcuInfo.PcuKeyInfo.b1Slow)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1Enable)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1Speaker)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1TurnLeft)
					&&(0 == sgPcuInfo.PcuKeyInfo.b1TurnRight)		
				)
				{
					static uint8_t u8Cnt;
					if(H9_INIT == u8DisplayPage)
					{
						u8Cnt++;
						if(u8Cnt > 30)
						{
							u8DisplayPage = H9_LOWLIMITMOVE_EN;
							u8Cnt = 0;
						}	
					}
				}
				if(0 == sgPcuInfo.PcuKeyInfo.u8data)
				{
					vKillNetTimer(TIMER_SwitchCheck);
				}
				u8DisplayCnt++;
			}
			switch(u8DisplayPage)
			{
				case H9_LOWLIMIT_CALI:
				{
					uint16_t u16AngleTmp;
					u16AngleTmp = i32LocalAiGet(ANGLE_SENSOR_CHANNEL);
					if(0 == u16SaveParaToEeprom(PARA_LOWLIMIT_ANGLE,u16AngleTmp))
						u8DisplayPage = H9_SAVE_SUCCESS;
				}
					break;
				case H9_LOWLIMITMOVE_EN:
					if((1 == sgPcuInfo.PcuKeyInfo.b1TurnRight)&&(0 == PcuKeyRec2.b1TurnRight)
						||(1 == sgPcuInfo.PcuKeyInfo.b1TurnLeft)&&(0 == PcuKeyRec2.b1TurnLeft))
						sgUserSets.b1LowLimitBanMoveEn ^= 1;
						vPcuDisplayNumber(SendData,sgUserSets.b1LowLimitBanMoveEn,DOUBLE_DOT);
					break;
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
					30�ϵ���ģʽ-Ĭ������			
					24 δ�궨������λ�ɶ���		��δʵ��		ȱ�ٲ���
					20~17��������						
					16����˫�غ�							��δʵ��		δ֪����
					15�������ֹ����					
					14�߶�˫������					��δʵ��		δ֪����
					11�����ּ����½�					
					10�Ӷ�������������				��δʵ��		ȱ�ٲ���
					7�½�������								
					6��������									
					5�ߵͷ�										��δʵ��		û�ж�Ӧ�ķ�
					3���߷���ײ����						��δʵ��		û��ģ��
					1�޶���˯��								��δʵ��		��ȷ��ʹ�ò���
					*/
<<<<<<< .mine
					if(LiBattery == i32GetPara(PARA_BatteryType))
						u32CombinedDataH9 |=(((1)&0xF)<<24);

					u32CombinedDataH9 |=((i32GetPara(PARA_CARCODE)&0xFF)<<16);//16��������
||||||| .r31
=======
					if(LiBattery == i32GetPara(PARA_BatteryType))
						u32CombinedDataH9 |=(((1)&0xF)<<24); 

					u32CombinedDataH9 |=((i32GetPara(PARA_CARCODE)&0xFF)<<16);//16��������
>>>>>>> .r49
					
<<<<<<< .mine
					u32CombinedDataH9 |=(sgUserSets.b1StartWithMode << 15);//15�ϵ���ģʽ
					u32CombinedDataH9 |=((i32GetPara(PARA_PitProtectFunc)&0x1)<<14);//14�Ӷ�����
					if((i32GetPara(PARA_DriverFlag)&(0x01<<8)) == 0)
						u32CombinedDataH9 |=(0x00000001 << 13);//0���� 1 �ر� 13���޴�������
					u32CombinedDataH9 |=((i32GetPara(PARA_TiltSwitchSetting)&0x1)<<12);//12ˮƽ������
||||||| .r31
					u32CombinedDataH9 |= ((i32GetPara(PARA_PitProtectFunc)&0x1)<<28);//�Ӷ�����
=======
					u32CombinedDataH9 |=(sgUserSets.b1StartWithMode << 15);//15�ϵ���ģʽ
					u32CombinedDataH9 |=((i32GetPara(PARA_PitProtectFunc)&0x1)<<14);//14�Ӷ�����
>>>>>>> .r49
					
<<<<<<< .mine
||||||| .r31
					u32CombinedDataH9 |=(((i32GetPara(PARA_BatteryType))&0b1111)<<24);//�������
=======
					if((i32GetPara(PARA_DriverFlag)&(0x01<<8)) == 0)
					{
						u32CombinedDataH9 |=(1 << 13);//0���� 1 �ر� 13���޴�������
					}	
					u32CombinedDataH9 |=((i32GetPara(PARA_TiltSwitchSetting)&0x1)<<12);//12ˮƽ������
					u32CombinedDataH9 |=((i32GetPara(PARA_AngleSimulationLimit)&0x1)<<11);//11�Ƕ�ģ����λ
					if(i32GetPara(PARA_DriverFlag)&(0x01<<10))
					{
						u32CombinedDataH9 |=(1 << 10);//0���� 1 �ر� 13���޽Ƕ�ģ����λ
					}
					u32CombinedDataH9 |=((sgUserSets.b1LiftBanMove)<<9);//9��������
					u32CombinedDataH9 |=((i32GetPara(PARA_SpeakerSync)&0x1)<<8);//8Ѹ����ͬ��

					u32CombinedDataH9 |=(sgUserSets.b1AntipinchSlow << 7);//7�����ּ���
					u32CombinedDataH9 |=(sgUserSets.b1Err18LiftAllow << 6);// 6 18������������
					u32CombinedDataH9 |=((i32GetPara(PARA_LowerPumpType)&0x1)<<5);//5 �½�������
					u32CombinedDataH9 |=(sgUserSets.b1StartFast << 4);//4 ��������
>>>>>>> .r49
					
<<<<<<< .mine
					u32CombinedDataH9 |=((i32GetPara(PARA_AngleSimulationLimit)&0x1)<<11);//11�Ƕ�ģ����λ
					u32CombinedDataH9 |=((i32GetPara(PARA_DriverFlag)&(0x1<<10))<<10);//10�Ƕȴ�����ʹ��
					u32CombinedDataH9 |=((sgUserSets.b1LiftBanMove)<<9);//9��������
					u32CombinedDataH9 |=((sgUserSets.b1DoubleRangeHeight)<<8);//8�߶�˫����

					u32CombinedDataH9 |=(sgUserSets.b1AntipinchSlow << 7);//7�����ּ���
					u32CombinedDataH9 |=(sgUserSets.b1Err18LiftAllow << 6);// 6 18������������
					u32CombinedDataH9 |=((i32GetPara(PARA_LowerPumpType)&0x1)<<5);//5 �½�������
					u32CombinedDataH9 |=(sgUserSets.b1StartFast << 4);//4 ��������
||||||| .r31
					u32CombinedDataH9 |=((i32GetPara(PARA_TiltSwitchSetting)&0x1)<<22);//ˮƽ������
=======
					u32CombinedDataH9 |=((i32GetPara(PARA_ParallelValveReverseFunc)&0x1) << 3);//3 �õ����
					u32CombinedDataH9 |=((i32GetPara(PARA_AnticollisionFunc)&0x1) << 2);//2 ����ײ����
					u32CombinedDataH9 |=((i32GetPara(PARA_LiftReverseFunc)&0x1)<<1);//1 ��������
					u32CombinedDataH9 |=(sgUserSets.b1SleepEn);//0 ��������
>>>>>>> .r49
					
<<<<<<< .mine
					u32CombinedDataH9 |=((i32GetPara(PARA_ParallelValveReverseFunc)&0x1) << 3);//3 �õ����
					u32CombinedDataH9 |=((i32GetPara(PARA_AnticollisionFunc)&0x1) << 2);//2 ����ײ����
					u32CombinedDataH9 |=((i32GetPara(PARA_LiftReverseFunc)&0x1)<<1);//1 ��������
					u32CombinedDataH9 |=(sgUserSets.b1SleepEn);//0 ��������
					
||||||| .r31
					u32CombinedDataH9 |=((i32GetPara(PARA_AngleSimulationLimit)&0x1)<<20);//
					
					u32CombinedDataH9 |=((i32GetPara(PARA_PitProtectFunc)&0x1)<<16);//�Ӷ�����
					u32CombinedDataH9 |=((i32GetPara(PARA_LowerPumpType)&0x1)<<6);
					u32CombinedDataH9 |=((i32GetPara(PARA_LiftReverseFunc)&0x1)<<1);
=======
>>>>>>> .r49
					break;
				case H9_VERIFY:
					vPcuH9(SendData);
					break;
				case H9_LOW16CHOOSE:
					SendData->Data.b1SlowLed = 1;
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

						if(0 == u8SubPage)
							u8SubPage = 3;
						else
							u8SubPage--;							
					}
					
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
					SendData->Data.b4ModeControl = 0xD;
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
						if(0 == u8SubPage)
							u8SubPage = 3;
						else
							u8SubPage--;		
					}

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
					if(McuParaIdx == 0)//ECU�޸�ֻ����һ��
					if(McuParaIdx == 0)//ECU�޸�ֻ����һ��
					{
<<<<<<< .mine
						vChangeCarTypePara(u16Carcode);//�������޸�ECU����
						uint16_t u16Dftmp;
						sgUserSets.b1StartWithMode = (u32CombinedDataH9>>15)& 0x1;
						//sgUserSets.b1LiftAllowBeforeCali = (u32CombinedDataH9>>13)& 0x1;
						sgUserSets.b1LiftBanMove = (u32CombinedDataH9>>9)& 0x1;
						sgUserSets.b1DoubleRangeHeight = (u32CombinedDataH9>>8)& 0x1;
						sgUserSets.b1AntipinchSlow = (u32CombinedDataH9>>7)& 0x1;
						sgUserSets.b1Err18LiftAllow = (u32CombinedDataH9>>6)& 0x1;
						sgUserSets.b1StartFast = (u32CombinedDataH9>>4)& 0x1;
						sgUserSets.b1SleepEn = (u32CombinedDataH9>>0)& 0x1;
						
						i32SetPara(PARA_CARCODE,((u32CombinedDataH9>>16)& 0xFF));
						i32SetPara(PARA_PitProtectFunc,((u32CombinedDataH9>>14)& 0x1));
						u16Dftmp = i32GetPara(PARA_DriverFlag);
						if((u32CombinedDataH9>>13)& 0x1)            //���ݸò����޸�DO9�������
							u16Dftmp &=  ~(1 << 8);                   
						else
							u16Dftmp |= (1 << 8);
						i32SetPara(PARA_TiltSwitchSetting,((u32CombinedDataH9>>12)& 0x1));
						i32SetPara(PARA_AngleSimulationLimit,((u32CombinedDataH9>>11)& 0x1));
						if((u32CombinedDataH9>>10)& 0x1) //�رսǶȴ�������
							u16Dftmp |= (1<<10);
						else
							u16Dftmp &= ~(1<<10);
	
						i32SetPara(PARA_LowerPumpType,((u32CombinedDataH9>>5)& 0x1));
						i32SetPara(PARA_ParallelValveReverseFunc,((u32CombinedDataH9>>3)& 0x1));
						i32SetPara(PARA_AnticollisionFunc,((u32CombinedDataH9>>2)& 0x1));
						i32SetPara(PARA_LiftReverseFunc,((u32CombinedDataH9>>1)& 0x1));
						i32SetPara(PARA_DriverFlag,u16Dftmp);
||||||| .r31
=======
						uint16_t u16Dftmp;
						sgUserSets.b1StartWithMode = (u32CombinedDataH9>>15)& 0x1;
						sgUserSets.b1LiftBanMove = (u32CombinedDataH9>>9)& 0x1;
						sgUserSets.b1AntipinchSlow = (u32CombinedDataH9>>7)& 0x1;
						sgUserSets.b1Err18LiftAllow = (u32CombinedDataH9>>6)& 0x1;
						sgUserSets.b1StartFast = (u32CombinedDataH9>>4)& 0x1;
						sgUserSets.b1SleepEn = (u32CombinedDataH9>>0)& 0x1;
						
						i32SetPara(PARA_CARCODE,((u32CombinedDataH9>>16)& 0xFF));
						u16Carcode = ((u32CombinedDataH9>>16)& 0xFF);
						i32SetPara(PARA_PitProtectFunc,((u32CombinedDataH9>>14)& 0x1));
						u16Dftmp = i32GetPara(PARA_DriverFlag);
						if((u32CombinedDataH9>>13)& 0x1)            //���ݸò����޸�DO9�������
							u16Dftmp &=  ~(1 << 8);                   
						else
							u16Dftmp |= (1 << 8);
						i32SetPara(PARA_TiltSwitchSetting,((u32CombinedDataH9>>12)& 0x1));
						i32SetPara(PARA_AngleSimulationLimit,((u32CombinedDataH9>>11)& 0x1));
						if((u32CombinedDataH9>>10)& 0x1) //�رսǶȴ�������
							u16Dftmp |= (1<<10);
						else
							u16Dftmp &= ~(1<<10);
	
						i32SetPara(PARA_LowerPumpType,((u32CombinedDataH9>>5)& 0x1));
						i32SetPara(PARA_ParallelValveReverseFunc,((u32CombinedDataH9>>3)& 0x1));
						i32SetPara(PARA_AnticollisionFunc,((u32CombinedDataH9>>2)& 0x1));
						i32SetPara(PARA_LiftReverseFunc,((u32CombinedDataH9>>1)& 0x1));
						i32SetPara(PARA_DriverFlag,u16Dftmp);
>>>>>>> .r49
						uint8_t u8Cnt = 5;
						while(u8Cnt)
						{
							u8Cnt--;
							u8SaveState = 0;
							u8SaveState += u16SaveParaToEeprom(PARA_CARCODE,((u32CombinedDataH9>>16)& 0xFF));
							u8SaveState += u16SaveParaToEeprom(PARA_PitProtectFunc,((u32CombinedDataH9>>14)& 0x1));
							u8SaveState += u16SaveParaToEeprom(PARA_TiltSwitchSetting,((u32CombinedDataH9>>12)& 0x1));
							u8SaveState += u16SaveParaToEeprom(PARA_AngleSimulationLimit,((u32CombinedDataH9>>11)& 0x1));
							u8SaveState += u16SaveParaToEeprom(PARA_LowerPumpType,((u32CombinedDataH9>>5)& 0x1));
							u8SaveState += u16SaveParaToEeprom(PARA_ParallelValveReverseFunc,((u32CombinedDataH9>>3)& 0x1));
							u8SaveState += u16SaveParaToEeprom(PARA_AnticollisionFunc,((u32CombinedDataH9>>2)& 0x1));
							u8SaveState += u16SaveParaToEeprom(PARA_LiftReverseFunc,((u32CombinedDataH9>>1)& 0x1));
<<<<<<< .mine
							u8SaveState += u16SaveParaToEeprom(PARA_DriverFlag,u16Dftmp);
							if(1 == ((u32CombinedDataH9>>24)& 0xF))
								u8SaveState += u16SaveParaToEeprom(PARA_BatteryType,LiBattery);
							else
								u8SaveState += u16SaveParaToEeprom(PARA_BatteryType,LiShi);
							
							u8SaveState += u16SaveParaToEeprom(PARA_USERSETS,sgUserSets.u16Data);
||||||| .r31
=======
							u8SaveState += u16SaveParaToEeprom(PARA_SpeakerSync,((u32CombinedDataH9>>8)& 0x1));
							u8SaveState += u16SaveParaToEeprom(PARA_DriverFlag,u16Dftmp);
							if(1 == ((u32CombinedDataH9>>24)& 0xF))
								u8SaveState += u16SaveParaToEeprom(PARA_BatteryType,LiBattery);
							else
								u8SaveState += u16SaveParaToEeprom(PARA_BatteryType,LiShi);
							
							u8SaveState += u16SaveParaToEeprom(PARA_USERSETS,sgUserSets.u16Data);
>>>>>>> .r49
							if(0 == u8SaveState)
								break;
<<<<<<< .mine
						}
						u16Carcode = i32GetPara(PARA_CARCODE);		
					}
					if(u16Carcode != u16CarCodeOld)
						vChangeMcuPara();//MCU�������봮�ڽ�����ν���
					
||||||| .r31
						}
					}				
=======
						}						
						if(u16Carcode != u16CarCodeOld)
							vChangeCarTypePara(u16Carcode);//���Ͳ�ͬʱ�޸�ECU����						
					}
					if(u16Carcode != u16CarCodeOld)
						vChangeMcuPara();//MCU�������봮�ڽ�����ν���
					
>>>>>>> .r49
					vPcuNull(SendData);//���޸ģ���Ҫ��α���
<<<<<<< .mine
||||||| .r31

=======
					
					if(false == u8GetNetTimerStartFlag(TIMER_AlarmDelay))
						vSetNetTimer(TIMER_AlarmDelay,1000);//1S�ڽ��ղ���ECU����Ϣ���жϱ���ʧ��
>>>>>>> .r49
					if(0 != u8SaveState)
					{
						u8DisplayPage = H9_SAVEFAIL;
					}
					else if(McuParaIdx == (sizeof(MCU_Para5m)/sizeof(MCU_Para5m[0]))||(u16Carcode == u16CarCodeOld))
					{
						u8DisplayPage = H9_SAVE_SUCCESS;
						u16SaveParaToEeprom(CARCODE_HISTORY,u16Carcode);
						u8DisplayCnt = 0;
					}
					else if(true == u8GetNetTimerOverFlag(TIMER_AlarmDelay))
					{
						u8DisplayPage = H9_SAVEFAIL;
						u16SaveParaToEeprom(CARCODE_HISTORY,u16Carcode);//MCU��������ʧ��ECU��������ɹ�
						u8DisplayCnt = 0;
						vKillNetTimer(TIMER_AlarmDelay);
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
				case H9_DOWNLIMIT:
					{
						uint16_t u16tmp = 0;
						u16tmp = i32LocalAiGet(ANGLE_SENSOR_CHANNEL);
						u8DisplayCnt++;
						if(u8DisplayCnt > (5000 / PCU_DISPLAY_PERIOD))
						{
							if(0 == u16SaveParaToEeprom(PARA_AngleSimulationDownLimit,u16tmp))
								u8DisplayPage = H9_SAVE_SUCCESS;
						}
						else
							vPcuDisplayOrigin(SendData,0x5E,0x3F);//��ʾdO
					}
					break;
				case H9_UPLIMIT:
					{
						uint16_t u16tmp = 0;
						u16tmp = i32LocalAiGet(ANGLE_SENSOR_CHANNEL);
						u8DisplayCnt++;
						if(u8DisplayCnt > (5000 / PCU_DISPLAY_PERIOD))
						{
							if(0 == u16SaveParaToEeprom(PARA_AngleSimulationUpLimit,u16tmp))
							{
								u8DisplayPage = H9_SAVE_SUCCESS;
							}
						}
						else
							vPcuDisplayOrigin(SendData,0x3E,0x73);//��ʾUP
					}
					break;
				case H9_SAVE_SUCCESS :
					{
						vPcuDisplayOrigin(SendData,0x3E,0x39);//��ʾUC
						McuParaIdx = 0;//����ɹ���MCU����������
						
						if(u8DisplayCnt<4)
						{
							u8DisplayCnt++;
						}
						else
						{
							if(0 != sgPcuInfo.PcuKeyInfo.u8data)
								sgErrorState.b1SaveSuccess =1;
							else
							{
								sgErrorState.b1SaveSuccess =0;
								u8DisplayPage = H9_INIT;
							}
						}
					}
					break;
				default:
					break;
			}
			break;
		case H8_MODE:
			/*
			1������״̬ H8��˸4s��Ϩ�𡢽���״̬һֱ��˸
			2������ʹ�ܺ����H2ģʽ
			3�����¹��ٺ󣬽���H0��ʱ����
			4���������ߣ�����H1���ý���
			*/
				{
					if(false == u8GetNetTimerStartFlag(TIMER_SwitchCheck))
					{
						vSetNetTimer(TIMER_SwitchCheck,4000);
					}
					else if(false == u8GetNetTimerOverFlag(TIMER_SwitchCheck))
					{
						u8DisplayCnt++;
						if(u8DisplayCnt < 4)
							vPcuNull(SendData);
						else if(u8DisplayCnt < 10)
							vPcuDisplayOrigin(SendData,0x76,0x7F);//��ʾH8 
						else
						{
							vPcuDisplayOrigin(SendData,0x76,0x7F);//��ʾH8 
							u8DisplayCnt = 0;
						}						
					}
					else
					{
						if((i32ErrCodeCheck(PLATFORM_LEVEL1_LOCK_ERR))
							||(i32ErrCodeCheck(PLATFORM_LEVEL2_LOCK_ERR))
							||(i32ErrCodeCheck(HEART_BEAT_LOCK_ERR))
							)//δ����״̬��������˸
						{
							u8DisplayCnt++;
							if(u8DisplayCnt < 4)
								vPcuNull(SendData);
							else if(u8DisplayCnt < 10)
								vPcuDisplayOrigin(SendData,0x76,0x7F);//��ʾH8 
							else
							{
								vPcuDisplayOrigin(SendData,0x76,0x7F);//��ʾH8 
								u8DisplayCnt = 0;
							}						
						}
						else
						{
							vPcuDisplayOrigin(SendData,0x76,0x7F);//��ʾH8 
						}
						
						if((0 == PcuKeyRec2.b1Enable)&&(1 == sgPcuInfo.PcuKeyInfo.b1Enable))//��ʹ�ܽ�H2
						{
							u8ParaSetMode = H2_MODE;
						}
						if((0 == PcuKeyRec2.b1Slow)&&(1 == sgPcuInfo.PcuKeyInfo.b1Slow))//�����٣���H0
						{
							u8ParaSetMode = H0_MODE;
						}
						if((0 == PcuKeyRec2.b1MoveMode)&&(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode))//�����ߣ���H1
						{
							u8ParaSetMode = H1_MODE;
						}
					}
				}
			break;
		case H2_MODE://�����Ƿ���������ѯ
			/*
			1�����¾�����ʾ�����ǰ��λ������λ
			2���������Ƚ���У�����������
			3��������水��������λ��
			4��
			*/
			{
				switch(u8DisplayPage)
				{
					case H2_INITIAL:
						{/*��ʾ*/
							u8DisplayCnt++;
							if(u8DisplayCnt < 4)
								vPcuNull(SendData);
							else if(u8DisplayCnt < 10)
								vPcuDisplayOrigin(SendData,0x76,0x5B);//��ʾH2
							else
							{
								vPcuDisplayOrigin(SendData,0x76,0x5B);//��ʾH2 
								u8DisplayCnt = 0;
							}
							/*�л�*/
							if((0 == PcuKeyRec2.b1LiftMode)&&(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode))//����������ʾ�����
							{	
								u8DisplayPage = H2_DISPLAY_NUM;		
								u8DisplayCnt = 0;
							}
						}
						break;
					case H2_DISPLAY_NUM:
						{
							//���������л������
							if((0 == PcuKeyRec2.b1LiftMode)&&(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode))
							{
								u8DisplayCnt ^= 1;
							}							
							//�л��ߵ�8λ��ʾ
							if(0 == u8DisplayCnt)
							{
								vPcuDisplayNumber(SendData,((sgRemoteLockKey.u16RandomNum) & 0xFF),(DISPLAY_NORMAL | HEX_DISPLAY));//��λ��ʮλ
							}
							else 
							{
								vPcuDisplayNumber(SendData,(((sgRemoteLockKey.u16RandomNum) >> 8) & 0xFF),(DOUBLE_DOT | HEX_DISPLAY));//ǧλ�Ͱ�λ
							}
							
							//�����ȣ������������
							if((0 == PcuKeyRec2.b1Speaker)&&(1 == sgPcuInfo.PcuKeyInfo.b1Speaker))
							{	
								u8DisplayPage = H2_DISPLAY_KEY;		
								u8DisplayCnt = 0;
								u32CombinedDataH9 = 0;
								u8CombinedData = 0;
								u8SubPage = 0;
							}
						}
						break;
					case H2_DISPLAY_KEY:
						{
							if((0 == PcuKeyRec2.b1LiftMode)&&(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode))//���������л�ҳ��
							{
								{
									u32CombinedDataH9 &= ~(0xF << ((5 - u8SubPage) * 4));//���
									u32CombinedDataH9 |= (u8CombinedData << ((5 - u8SubPage) * 4));//��ǰλ�����¸�ֵ	
								}									
								u8SubPage ++;
								if((6 <= u8SubPage))
								{
									u8SubPage = 0;
								}									
								{
									u8CombinedData = ((u32CombinedDataH9 >> ((5 - u8SubPage) * 4)) & 0xF);
								}
							}
							if((0 == PcuKeyRec2.b1TurnLeft)&&(1 == sgPcuInfo.PcuKeyInfo.b1TurnLeft))//��ת������
							{
								if(u8CombinedData == 0)
								{
									u8CombinedData = 9;
								}
								else
								{
									u8CombinedData--;
								}
							}
							if((0 == PcuKeyRec2.b1TurnRight)&&(1 == sgPcuInfo.PcuKeyInfo.b1TurnRight))//��ת����
							{
								if(u8CombinedData == 9)
								{
									u8CombinedData = 0;
								}
								else
								{
									u8CombinedData ++;
								}
							}
							u8DisplayCnt++;
							if(u8DisplayCnt < 4)
								vPcuDisplayNumber(SendData,((u8SubPage + 1 ) * 10 + u8CombinedData),MASK_RIGHT);
							else if(u8DisplayCnt < 10)
								vPcuDisplayNumber(SendData,((u8SubPage + 1 ) * 10 + u8CombinedData),DISPLAY_NORMAL);
							else
							{
								vPcuDisplayNumber(SendData,((u8SubPage + 1 ) * 10 + u8CombinedData),DISPLAY_NORMAL);
								u8DisplayCnt = 0;
							}
							if((0 == PcuKeyRec2.b1Speaker)&&(1 == sgPcuInfo.PcuKeyInfo.b1Speaker))//������ȷ��
							{
								u32CombinedDataH9 &= ~(0xF << ((5 - u8SubPage) * 4));//���
								u32CombinedDataH9 |= (u8CombinedData << ((5 - u8SubPage) * 4));//��ǰλ�����¸�ֵ	
								if((u32CombinedDataH9 == sgRemoteLockKey.u32ParaSets)
									||(u32CombinedDataH9 == sgRemoteLockKey.u32PermanUnlockKey))
								{
									u8DisplayPage = H2_HEART_SET;
								}
								else
								{
									u8DisplayPage = H2_HEART_CHECK_FAIL;
								}
							}
						}
						break;
					case H2_HEART_SET:
						{
							if(((0 == PcuKeyRec2.b1TurnLeft)&&(1 == sgPcuInfo.PcuKeyInfo.b1TurnLeft))
								||((0 == PcuKeyRec2.b1TurnRight)&&(1 == sgPcuInfo.PcuKeyInfo.b1TurnRight)))//��ת������
							{
								sgHeartBeatLockDynamic.b1HeartBeatQuery ^= 1;
							}
							vPcuDisplayNumber(SendData,sgHeartBeatLockDynamic.b1HeartBeatQuery,MASK_LEFT);
							if((0 == PcuKeyRec2.b1Speaker)&&(1 == sgPcuInfo.PcuKeyInfo.b1Speaker))//������ȷ��
							{
								u8DisplayPage = H2_SAVED;
								u8CombinedData = 3;//���ڱ������
							}
						}
						break;
					case H2_SAVED:
						{
							while(u8CombinedData)
							{								
								if(0 == u8SaveState)
									break;
								u8CombinedData--;
							}
							
							if(0 != u8SaveState)
							{
								u8DisplayPage = H2_HEART_CHECK_FAIL;
							}
							else
							{
								u8DisplayPage = H2_INITIAL;
							}
						}
						break;
					case H2_HEART_CHECK_FAIL://��ʾEr���ٰ������л���H8
						{
							u8DisplayCnt++;
							if(u8DisplayCnt < 4)
								vPcuNull(SendData);
							else if(u8DisplayCnt < 10)
								vPcuDisplayOrigin(SendData,0x79,0x50);
							else
							{
								vPcuDisplayOrigin(SendData,0x79,0x50);
								u8DisplayCnt = 0;
							}
							if((0 == PcuKeyRec2.b1LiftMode)&&(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode))
							{
								vRemoteUnlockKey();
								u8ParaSetMode = H8_MODE;
							}
						}							
						break;
					default :
						break;
				}
			}
			break;
		case H0_MODE:
			{
				switch(u8DisplayPage)
				{
					case H2_INITIAL:
						{/*��ʾ*/
							u8DisplayCnt++;
							if(u8DisplayCnt < 4)
								vPcuNull(SendData);
							else if(u8DisplayCnt < 10)
								vPcuDisplayOrigin(SendData,0x76,0x3F);//��ʾH0
							else
							{
								vPcuDisplayOrigin(SendData,0x76,0x3F);//��ʾH0 
								u8DisplayCnt = 0;
							}
							/*�л�*/
							if((0 == PcuKeyRec2.b1LiftMode)&&(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode))//����������ʾ�����
							{	
								u8DisplayPage = H2_DISPLAY_NUM;		
								u8DisplayCnt = 0;
							}
						}
						break;
					case H2_DISPLAY_NUM:
						{
							//���������л������
							if((0 == PcuKeyRec2.b1LiftMode)&&(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode))
							{
								u8DisplayCnt ^= 1;
							}							
							//�л��ߵ�8λ��ʾ
							if(0 == u8DisplayCnt)
							{
								vPcuDisplayNumber(SendData,((sgRemoteLockKey.u16RandomNum) & 0xFF),(DISPLAY_NORMAL | HEX_DISPLAY));//��λ��ʮλ
							}
							else 
							{
								vPcuDisplayNumber(SendData,(((sgRemoteLockKey.u16RandomNum) >> 8) & 0xFF),(DOUBLE_DOT | HEX_DISPLAY));//ǧλ�Ͱ�λ
							}
							//�����ȣ������������
							if((0 == PcuKeyRec2.b1Speaker)&&(1 == sgPcuInfo.PcuKeyInfo.b1Speaker))
							{	
								u8DisplayPage = H2_DISPLAY_KEY;		
								u8DisplayCnt = 0;
								u32CombinedDataH9 = 0;
								u8CombinedData = 0;
								u8SubPage = 0;
							}
						}
						break;
					case H2_DISPLAY_KEY:
						{
							if((0 == PcuKeyRec2.b1LiftMode)&&(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode))//���������л�ҳ��
							{
								{
									u32CombinedDataH9 &= ~(0xF << ((5 - u8SubPage) * 4));//���
									u32CombinedDataH9 |= (u8CombinedData << ((5 - u8SubPage) * 4));//��ǰλ�����¸�ֵ	
								}									
								u8SubPage ++;
								if((6 <= u8SubPage))
								{
									u8SubPage = 0;
								}									
								{
									u8CombinedData = ((u32CombinedDataH9 >> ((5 - u8SubPage) * 4)) & 0xF);
								}
							}
							if((0 == PcuKeyRec2.b1TurnLeft)&&(1 == sgPcuInfo.PcuKeyInfo.b1TurnLeft))//��ת������
							{
								if(u8CombinedData == 0)
								{
									u8CombinedData = 9;
								}
								else
								{
									u8CombinedData--;
								}
							}
							if((0 == PcuKeyRec2.b1TurnRight)&&(1 == sgPcuInfo.PcuKeyInfo.b1TurnRight))//��ת����
							{
								if(u8CombinedData == 9)
								{
									u8CombinedData = 0;
								}
								else
								{
									u8CombinedData ++;
								}
							}
							u8DisplayCnt++;
							if(u8DisplayCnt < 4)
								vPcuDisplayNumber(SendData,((u8SubPage + 1 ) * 10 + u8CombinedData),MASK_RIGHT);
							else if(u8DisplayCnt < 10)
								vPcuDisplayNumber(SendData,((u8SubPage + 1 ) * 10 + u8CombinedData),DISPLAY_NORMAL);
							else
							{
								vPcuDisplayNumber(SendData,((u8SubPage + 1 ) * 10 + u8CombinedData),DISPLAY_NORMAL);
								u8DisplayCnt = 0;
							}							
							if((0 == PcuKeyRec2.b1Speaker)&&(1 == sgPcuInfo.PcuKeyInfo.b1Speaker))//������ȷ��
							{
								u32CombinedDataH9 &= ~(0xF << ((5 - u8SubPage) * 4));//���
								u32CombinedDataH9 |= (u8CombinedData << ((5 - u8SubPage) * 4));//��ǰλ�����¸�ֵ	
								if(u32CombinedDataH9 == sgRemoteLockKey.u32TempUnlockKey)
								{
									u8DisplayPage = H2_HEART_SET;
								}
								else
								{
									u8DisplayPage = H2_HEART_CHECK_FAIL;
								}
							}
						}
						break;
					case H2_HEART_SET:
						{
							sgHeartBeatLockDynamic.b1TempUnlock = 1;
							u8DisplayPage = H2_SAVED;
							u8CombinedData = 3;//���ڱ������
						}
						break;
					case H2_SAVED:
						{
							while(u8CombinedData)
							{
								u8SaveState = u16SaveParaToEeprom(PARA_HeartBeatQueryFunc,sgHeartBeatLockDynamic.u8Data);
								
								if(0 == u8SaveState)
									break;
								u8CombinedData--;
							}
							if(0 != u8SaveState)
							{
								u8DisplayPage = H2_HEART_CHECK_FAIL;
							}
							else
							{
								u8DisplayPage = H2_INITIAL;
							}
						}
						break;
					case H2_HEART_CHECK_FAIL://��ʾEr���ٰ������л���H8
						{
							u8DisplayCnt++;
							if(u8DisplayCnt < 4)
								vPcuNull(SendData);
							else if(u8DisplayCnt < 10)
								vPcuDisplayOrigin(SendData,0x79,0x50);
							else
							{
								vPcuDisplayOrigin(SendData,0x79,0x50);
								u8DisplayCnt = 0;
							}
							if((0 == PcuKeyRec2.b1LiftMode)&&(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode))
							{
								vRemoteUnlockKey();
								u8ParaSetMode = H8_MODE;
							}
						}							
						break;
					default :
						break;
				}
			}			
			break;
		case H1_MODE :
			{
				switch(u8DisplayPage)
				{
					case H2_INITIAL:
						{/*��ʾ*/
							u8DisplayCnt++;
							if(u8DisplayCnt < 4)
								vPcuNull(SendData);
							else if(u8DisplayCnt < 10)
								vPcuDisplayOrigin(SendData,0x76,0x06);//��ʾH1
							else
							{
								vPcuDisplayOrigin(SendData,0x76,0x06);//��ʾH1
								u8DisplayCnt = 0;
							}
							/*�л�*/
							if((0 == PcuKeyRec2.b1LiftMode)&&(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode))//����������ʾ�����
							{	
								u8DisplayPage = H2_DISPLAY_NUM;		
								u8DisplayCnt = 0;
							}
						}
						break;
					case H2_DISPLAY_NUM:
						{
							//���������л������
							if((0 == PcuKeyRec2.b1LiftMode)&&(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode))
							{
								u8DisplayCnt ^= 1;
							}							
							//�л��ߵ�8λ��ʾ
							if(0 == u8DisplayCnt)
							{
								vPcuDisplayNumber(SendData,((sgRemoteLockKey.u16RandomNum) & 0xFF),(DISPLAY_NORMAL | HEX_DISPLAY));//��λ��ʮλ
							}
							else 
							{
								vPcuDisplayNumber(SendData,(((sgRemoteLockKey.u16RandomNum) >> 8) & 0xFF),(DOUBLE_DOT | HEX_DISPLAY));//ǧλ�Ͱ�λ
							}
							//�����ȣ������������
							if((0 == PcuKeyRec2.b1Speaker)&&(1 == sgPcuInfo.PcuKeyInfo.b1Speaker))
							{	
								u8DisplayPage = H2_DISPLAY_KEY;		
								u8DisplayCnt = 0;
								u32CombinedDataH9 = 0;
								u8CombinedData = 0;
								u8SubPage = 0;
							}
						}
						break;
					case H2_DISPLAY_KEY:
						{
							if((0 == PcuKeyRec2.b1LiftMode)&&(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode))//���������л�ҳ��
							{
								{
									u32CombinedDataH9 &= ~(0xF << ((5 - u8SubPage) * 4));//���
									u32CombinedDataH9 |= (u8CombinedData << ((5 - u8SubPage) * 4));//��ǰλ�����¸�ֵ	
								}									
								u8SubPage ++;
								if((6 <= u8SubPage))
								{
									u8SubPage = 0;
								}									
								{
									u8CombinedData = ((u32CombinedDataH9 >> ((5 - u8SubPage) * 4)) & 0xF);
								}
							}
							if((0 == PcuKeyRec2.b1TurnLeft)&&(1 == sgPcuInfo.PcuKeyInfo.b1TurnLeft))//��ת������
							{
								if(u8CombinedData == 0)
								{
									u8CombinedData = 9;
								}
								else
								{
									u8CombinedData--;
								}
							}
							if((0 == PcuKeyRec2.b1TurnRight)&&(1 == sgPcuInfo.PcuKeyInfo.b1TurnRight))//��ת����
							{
								if(u8CombinedData == 9)
								{
									u8CombinedData = 0;
								}
								else
								{
									u8CombinedData ++;
								}
							}
							u8DisplayCnt++;
							if(u8DisplayCnt < 4)
								vPcuDisplayNumber(SendData,((u8SubPage + 1 ) * 10 + u8CombinedData),MASK_RIGHT);
							else if(u8DisplayCnt < 10)
								vPcuDisplayNumber(SendData,((u8SubPage + 1 ) * 10 + u8CombinedData),DISPLAY_NORMAL);
							else
							{
								vPcuDisplayNumber(SendData,((u8SubPage + 1 ) * 10 + u8CombinedData),DISPLAY_NORMAL);
								u8DisplayCnt = 0;
							}							
							if((0 == PcuKeyRec2.b1Speaker)&&(1 == sgPcuInfo.PcuKeyInfo.b1Speaker))//������ȷ��
							{
								u32CombinedDataH9 &= ~(0xF << ((5 - u8SubPage) * 4));//���
								u32CombinedDataH9 |= (u8CombinedData << ((5 - u8SubPage) * 4));//��ǰλ�����¸�ֵ	
								if(u32CombinedDataH9 == sgRemoteLockKey.u32PermanUnlockKey)
								{
									u8DisplayPage = H2_HEART_SET;
								}
								else
								{
									u8DisplayPage = H2_HEART_CHECK_FAIL;
								}
							}
						}
						break;
					case H2_HEART_SET:
						{
							sgHeartBeatLockDynamic.b1LockLevel1 = 0;
							sgHeartBeatLockDynamic.b1LockLevel2 = 0;
							u8DisplayPage = H2_SAVED;
							u8CombinedData = 3;//���ڱ������
						}
						break;
					case H2_SAVED:
						{
							while(u8CombinedData)
							{
								u8SaveState = u16SaveParaToEeprom(PARA_HeartBeatQueryFunc,sgHeartBeatLockDynamic.u8Data);
								
								if(0 == u8SaveState)
									break;
								u8CombinedData--;
							}
							if(0 != u8SaveState)
							{
								u8DisplayPage = H2_HEART_CHECK_FAIL;
							}
							else
							{
								u8DisplayPage = H2_INITIAL;
							}
						}
						break;
					case H2_HEART_CHECK_FAIL://��ʾEr���ٰ������л���H8
						{
							u8DisplayCnt++;
							if(u8DisplayCnt < 4)
								vPcuNull(SendData);
							else if(u8DisplayCnt < 10)
								vPcuDisplayOrigin(SendData,0x79,0x50);
							else
							{
								vPcuDisplayOrigin(SendData,0x79,0x50);
								u8DisplayCnt = 0;
							}
							if((0 == PcuKeyRec2.b1LiftMode)&&(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode))
							{
								vRemoteUnlockKey();
								u8ParaSetMode = H8_MODE;
							}
						}							
						break;
					default :
						break;
				}
			}			
			break;
		case TBOX_PARA_SETS:
			{
				switch(u8RemoteParaFlag)
				{
					case REMOTE_HANDSHAKE:
							u8DisplayCnt++;
							if(u8DisplayCnt < 4)
								vPcuNull(SendData);
							else if(u8DisplayCnt < 10)
								vPcuDisplayOrigin(SendData,0x73,0x5E);//��ʾPD
							else
							{
								vPcuDisplayOrigin(SendData,0x73,0x5E);//��ʾPD
								u8DisplayCnt = 0;
							}
						break;
					case REMOTE_CHANGE_SUCCESS:
							u8DisplayCnt++;
							if(u8DisplayCnt < 4)
								vPcuNull(SendData);
							else if(u8DisplayCnt < 10)
								vPcuDisplayOrigin(SendData,0x3E,0x39);//��ʾUC
							else
							{
								vPcuDisplayOrigin(SendData,0x3E,0x39);//��ʾUC
								u8DisplayCnt = 0;
							}
						break;						
						break;
					case REMOTE_CHANGE_FAIL:
							u8DisplayCnt++;
							if(u8DisplayCnt < 4)
								vPcuNull(SendData);
							else if(u8DisplayCnt < 10)
								vPcuDisplayOrigin(SendData,0x79,0x50);//��ʾER
							else
							{
								vPcuDisplayOrigin(SendData,0x79,0x50);//��ʾER
								u8DisplayCnt = 0;
							}
						break ;
				}
			}
			break;							
		case TBOX_UPDATEAPP:
		{			
			u8DisplayCnt++;
			if(u8DisplayCnt < 4)
				vPcuNull(SendData);
			else if(u8DisplayCnt < 10)
				vPcuDisplayOrigin(SendData,0x3E,0x5E);//��ʾPD
			else
			{
				vPcuDisplayOrigin(SendData,0x3E,0x5E);//��ʾPD
				u8DisplayCnt = 0;
			}
		}
			break;					
		default:
			break;
	}
	PcuKeyRec2.u8data = sgPcuInfo.PcuKeyInfo.u8data;
<<<<<<< .mine
} 

#ifdef LIUGONG_TEST  	
	static uint8_t TestStep = 0;
  #define UpProc    0
	#define DownProc  1
	#define DownButt  2  	
	uint16_t u16UpDownCnt;
static void vTestUpDownCount(void)
{
	static tCanFrame Canframe;

	static uint16_t AngleValueSave1;
	static uint16_t AngleValueSave2;
	static uint8_t Delay;
	u16EepromRead(UPDOWNCOUNT_ADDR,&u16UpDownCnt,1);
	if(FunctionEnable == i32GetPara(PARA_AntiPinchFunc))//�رշ����ֹ���
	{
		u16SaveParaToEeprom(PARA_AntiPinchFunc, FunctionDisable);
	}
	switch(TestStep)
	{
		case UpProc:
			i32SetPara(PARA_CalibrationStatus,UpProc);//��������½�״̬���궨״̬��0
			if(0 == sgSwiInput.b1UpLimitSwi )
			{
				sgActLogic.b1LiftUpAct = 1;
				sgActLogic.b1LiftDownAct = 0;
				u16MotorVal = MOTOR_MAX_SPEED_VALUE;
			}
			else
			{
				sgActLogic.b1LiftUpAct = 0;
				sgActLogic.b1LiftDownAct = 0;
				u16MotorVal = 0;
				AngleValueSave1 = 0x00; //��ֹ�����½�δ�����ж�
				AngleValueSave2 = 0xff;
				TestStep = DownProc;
			}
			break;
		case DownProc:
			i32SetPara(PARA_CalibrationStatus,DownProc);//��������½�״̬���궨״̬��1
			if(1 == sgSwiInput.b1DownLimitSwi)//
			{ 
				if(u8GetNetTimerStartFlag(TIMER_Calibration) == false)
				{
					vSetNetTimer(TIMER_Calibration,gUserInfo.u8Gear1Spd*100);  //��������λ�����½�Gear1spd*100 MS
				}
				if((u8GetNetTimerOverFlag(TIMER_Calibration) == true))
				{
					sgActLogic.b1LiftUpAct = 0;            
					sgActLogic.b1LiftDownAct = 0;
					u16MotorVal = 0;
					TestStep = DownButt; 	
					vKillNetTimer(TIMER_Calibration);
				}
				else
				{
					sgActLogic.b1LiftDownAct = 1;
					sgActLogic.b1LiftUpAct = 0;
					u16MotorVal = MOTOR_MAX_SPEED_VALUE;
				}
			}
			else
			{
				sgActLogic.b1LiftDownAct = 1;
				sgActLogic.b1LiftUpAct = 0;
				u16MotorVal = MOTOR_MAX_SPEED_VALUE;
			}
			break;
		case DownButt:
			i32SetPara(PARA_CalibrationStatus,DownButt);//��������½�״̬���궨״̬��2 			
			u16UpDownCnt++;
			TestStep = UpProc; 	
			u16EepromWrite(UPDOWNCOUNT_ADDR,u16UpDownCnt,1);	
			break;
	}

||||||| .r31
=======
} 

#ifdef LIUGONG_TEST  	
	static uint8_t TestStep = 0;
  #define UpProc    0
	#define DownProc  1
	#define DownButt  2  	
	uint16_t u16UpDownCnt;
static void vTestUpDownCount(void)
{
	static tCanFrame Canframe;

	static uint16_t AngleValueSave1;
	static uint16_t AngleValueSave2;
	static uint8_t Delay;
	u16EepromRead(UPDOWNCOUNT_ADDR,&u16UpDownCnt,1);
	if(FunctionEnable == i32GetPara(PARA_AntiPinchFunc))//�رշ����ֹ���
	{
		u16SaveParaToEeprom(PARA_AntiPinchFunc, FunctionDisable);
	}
	switch(TestStep)
	{
		case UpProc:
			i32SetPara(PARA_CalibrationStatus,UpProc);//��������½�״̬���궨״̬��0
			if(0 == sgSwiInput.b1UpLimitSwi )
			{
				sgActLogic.b1LiftUpAct = 1;
				sgActLogic.b1LiftDownAct = 0;
				u16MotorVal = MOTOR_MAX_SPEED_VALUE;
			}
			else
			{
				if(u8GetNetTimerStartFlag(TIMER_Calibration) == false)
					{
						vSetNetTimer(TIMER_Calibration,2000);  //��������λ2S��ʱ
					}
					if((u8GetNetTimerOverFlag(TIMER_Calibration) == true))
					{
						sgActLogic.b1LiftUpAct = 0;            
						sgActLogic.b1LiftDownAct = 0;
						u16MotorVal = 0;
						TestStep = DownProc; 	
						vKillNetTimer(TIMER_Calibration);
					}
			}
			break;
		case DownProc:
			i32SetPara(PARA_CalibrationStatus,DownProc);//��������½�״̬���궨״̬��1
			if(1 == sgSwiInput.b1DownLimitSwi)//
			{ 
				if(u8GetNetTimerStartFlag(TIMER_Calibration) == false)
				{
					vSetNetTimer(TIMER_Calibration,gUserInfo.u8Gear1Spd*100);  //��������λ�����½�Gear1spd*100 MS
				}
				if((u8GetNetTimerOverFlag(TIMER_Calibration) == true))
				{
					sgActLogic.b1LiftUpAct = 0;            
					sgActLogic.b1LiftDownAct = 0;
					u16MotorVal = 0;
					TestStep = DownButt; 	
					vKillNetTimer(TIMER_Calibration);
				}
				else
				{
					sgActLogic.b1LiftDownAct = 1;
					sgActLogic.b1LiftUpAct = 0;
					u16MotorVal = MOTOR_MAX_SPEED_VALUE;
				}
			}
			else
			{
				sgActLogic.b1LiftDownAct = 1;
				sgActLogic.b1LiftUpAct = 0;
				u16MotorVal = MOTOR_MAX_SPEED_VALUE;
			}
			break;
		case DownButt:
			i32SetPara(PARA_CalibrationStatus,DownButt);//��������½�״̬���궨״̬��2 			
			u16UpDownCnt++;
			TestStep = UpProc; 	
			u16EepromWrite(UPDOWNCOUNT_ADDR,u16UpDownCnt,1);	
			break;
	}

>>>>>>> .r49
}
#endif
static void vActionProcess()
{
	sgActLogic.u8Data = 0;

//	i32SetPara(PARA_LoadRate,u8RunMode); //������
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
		#ifdef LIUGONG_TEST
		case AUTO_LIFT_MODE:
			vTestUpDownCount();
			break;
		#endif
		default:
			break;
	}
}

static void vPcuSendProc(xPcuSendPara *SendData)
{
	vPcuParaSetProc(SendData);

		
//	if(1 == sgErrorState.b1Error)
//	{
//		u8BeepCnt++;
//		if(u8BeepCnt < 10)
//		SendData->Data.b1Beep = 1;
//		else
//		{
//			SendData->Data.b1Beep = 0;
//			if(u8BeepCnt > 20)
//				u8BeepCnt = 0;	
//		}
//	}
//	else
//	{
//		SendData->Data.b1Beep = 0;		
//	}
//	SendData->Data.
}


static void vMstRevProc(xMstRevPara *RevData)
{
	uint16_t tmp = 0;
	
	if(0 != RevData->b1MainDriver)
	{
		CanSend169.b1CarMaintenance = 1;
	}
	else
	{
		CanSend169.b1CarMaintenance = 0;
	}
	
	if(0 != RevData->b1Ebrake)
	{
		
	}
	else
	{	
		
	}
	
	if(0 != RevData->b1Driver3State)
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
	if(u16MotorFdb > 32767)    //�������ͽ�����ת������
		u16MotorFdb = 0;
	i32SetPara(PARA_MotorSpd,u16MotorFdb);
	u16Current = (RevData->u8CurrentHigh << 8)| RevData->u8CurrentLow;

	__disable_irq();

	tmp = u16MotorFdb / gUserInfo.u16RatioOfTransmission;
	/*Motor Speed*/
	__enable_irq();
	
	__disable_irq();
	/*Motor Speed*/
	__enable_irq();
	
	tmp = RevData->u8MotorTmp ;
	u8MotorTmp = tmp;
	/*Motor Temp*/

	tmp = RevData->u8BoardTmp ;
	u8BoardTmp = tmp;
}


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


static void vBeepCallBack(uint8_t u8Flag)
{
	if((1 == u8Flag)
		||((1==sgPcuInfo.PcuKeyInfo.b1Speaker)
			&&(PARASETS_DISABLE == u8ParaSetMode)
			&&(1 == gUserInfo.u8SpeakerSync))
//		||()//����ɹ�����
		)
	{
		sgErrorState.PCUBeep = 1;
	}
	else
	{
		sgErrorState.PCUBeep = 0;
	}
}

static void vBlinkBeepProc()
{
	if((1 == sgErrorState.PCUBeep))
//		||((1 == sgPcuInfo.PcuKeyInfo.b1Speaker)&&(1 == gUserInfo.u8SpeakerSync)))//Ѹ����ͬ��
	{
		i32DoPwmSet(BLINK_BEEP, PUMP_OPEN_PERCENTAGE);
	}
	else
	{
		i32DoPwmSet(BLINK_BEEP, PUMP_CLOSE_PERCENTAGE);
	}
	if((1 == sgPcuInfo.PcuKeyInfo.b1Speaker)&&(PARASETS_DISABLE == u8ParaSetMode))
	{
		i32DoPwmSet(SPEAKER_PUMP, PUMP_OPEN_PERCENTAGE);
	}
	else
	{
		i32DoPwmSet(SPEAKER_PUMP, PUMP_CLOSE_PERCENTAGE);
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


static void vLowSocAlarm()
{
	tCanFrame LowBatAlarmFrame ={.u16DataLength = 8,.u32ID = 0x18E15521,.u8Rtr = 0,
	.u8Data[0] = 0x01,.u8Data[1] = 0x51,.u8Data[2] = 0x0E,.u8Data[3] = 0x0E,
	.u8Data[4] = 0x00,.u8Data[5] = 0x00,.u8Data[6] = 0x50,.u8Data[7] = 0x02,};
	static uint16_t u16AlarmCnt;
	if((i32ErrCodeCheck(BAT_LOW_CAP2_ERR))||(i32ErrCodeCheck(BATTERY_LOW_CAP1_ERR)))
	{
		u16AlarmCnt++;
		if(u16AlarmCnt > (60000 / USER_ECU_PERIOD))
		{
			u16AlarmCnt = 0;
			i32CanWrite(Can0,&LowBatAlarmFrame);
		}
	}
}

typedef struct 
{
	uint16_t Voltage;
	uint8_t Soc;
}BatteryTable;
	
<<<<<<< .mine
const static BatteryTable DynamicSoc[6]=
{
	{.Voltage = 24600,.Soc = 100},
	{.Voltage = 23500,.Soc = 80},
	{.Voltage = 22700,.Soc = 65},
	{.Voltage = 21800,.Soc = 50},
	{.Voltage = 20800,.Soc = 30},
	{.Voltage = 19800,.Soc = 0},
};
||||||| .r31
	u16AdcValue = i32LocalAiGetValue(AI_B_VBUS_CHECK);
	i32SetPara(PARA_BatteryVoltage, u16AdcValue);
=======
const static BatteryTable DynamicSoc[6]=
{
	{.Voltage = 24600,.Soc = 100},
	{.Voltage = 23500,.Soc = 80},
	{.Voltage = 22700,.Soc = 65},
	{.Voltage = 21800,.Soc = 50},
	{.Voltage = 20800,.Soc = 25},
	{.Voltage = 19800,.Soc = 0},
};
>>>>>>> .r49

<<<<<<< .mine
static BatteryTable StaticSoc[6]=
{
	{.Voltage = 26000,.Soc = 100},//��
	{.Voltage = 25100,.Soc = 80 },//����80 5��
	{.Voltage = 24700,.Soc = 65 },//
	{.Voltage = 24300,.Soc = 50 },
	{.Voltage = 24000,.Soc = 30 },
	{.Voltage = 23700,.Soc = 0  },
};

static uint8_t u8BatteryMeter(const BatteryTable *BatTable,uint8_t u8TableSize)
{
	static uint16_t u16Voltage = 25000;
	static uint32_t u32VoltageSum = 25000 * 20;
	static uint8_t u8TmpSoc;
	
	u32VoltageSum += i32LocalAiGetValue(AI_B_KSI_CHECK) - u16Voltage;
	u16Voltage = u32VoltageSum / 20;
	
	if(u16Voltage >= BatTable[0].Voltage)
	{
		return BatTable[0].Soc;
	}
	else if(u16Voltage <= BatTable[u8TableSize - 1].Voltage)
	{
		return BatTable[u8TableSize - 1].Soc;
	}
	
	for(uint8_t i = 0; i < u8TableSize - 1; i++)
	{
		if((u16Voltage >= BatTable[i+1].Voltage)
			&&(u16Voltage < BatTable[i].Voltage))
		{
			u8TmpSoc = BatTable[i+1].Soc
			+  (BatTable[i].Soc - BatTable[i+1].Soc)
				*(u16Voltage - BatTable[i+1].Voltage)
				/(BatTable[i].Voltage - BatTable[i+1].Voltage);
			break;
		}
	}
	return u8TmpSoc;
||||||| .r31
//	else
//	{
//		i32ErrCodeClr(BAT_LOW_CAP2_ERR);
//	}
=======
static BatteryTable StaticSoc[6]=
{
	{.Voltage = 25600,.Soc = 100},//��
	{.Voltage = 25100,.Soc = 80 },//����80 5��
	{.Voltage = 24700,.Soc = 65 },//
	{.Voltage = 24300,.Soc = 50 },
	{.Voltage = 24000,.Soc = 25 },
	{.Voltage = 23700,.Soc = 0 },
};

static uint8_t u8BatteryMeter(const BatteryTable *BatTable,uint8_t u8TableSize)
{
	static uint16_t u16Voltage = 25000;
	static uint32_t u32VoltageSum = 25000 * 20;
	static uint8_t u8TmpSoc;
	
	u32VoltageSum += i32LocalAiGetValue(AI_B_KSI_CHECK) - u16Voltage;
	u16Voltage = u32VoltageSum / 20;
	
	if(u16Voltage >= BatTable[0].Voltage)
	{
		return BatTable[0].Soc;
	}
	else if(u16Voltage <= BatTable[u8TableSize - 1].Voltage)
	{
		return BatTable[u8TableSize - 1].Soc;
	}
	
	for(uint8_t i = 0; i < u8TableSize - 1; i++)
	{
		if((u16Voltage >= BatTable[i+1].Voltage)
			&&(u16Voltage < BatTable[i].Voltage))
		{
			u8TmpSoc = BatTable[i+1].Soc
			+  (BatTable[i].Soc - BatTable[i+1].Soc)
				*(u16Voltage - BatTable[i+1].Voltage)
				/(BatTable[i].Voltage - BatTable[i+1].Voltage);
			break;
		}
	}
	return u8TmpSoc;
>>>>>>> .r49
}
static void vBatteryManage(void)
{
	static uint16_t u16LowBatCnt ;  //һ������������ʱ
	static uint16_t u16LowBatCnt2;  //��������������ʱ
	static uint16_t u16StableSocCnt = 0; //��̬�ǵ�����ʱ

	static uint16_t u16DownCnt = 0;
	static uint16_t u16LowBatAlarming = 0;
	static uint8_t 	u8SocStatic = 0;
	static uint16_t u16AlarmCnt = 0;
	uint16_t u16AdcValue = 0;
	
	u16AdcValue = i32LocalAiGetValue(AI_B_KSI_CHECK);
	
	i32SetPara(PARA_BatteryVoltage, u16AdcValue);	
	
	if(LiBattery == gUserInfo.u8BatteryType)
	{
		u8Soc = gCanRevPdoInfo.BMSRev052.u8BMSSOC;
		
		if(1 == gCanRevPdoInfo.BMSRev053.b1ErrVoltageDiff2)
			i32ErrCodeSet(BMS_BATTERY_VOL_DIFF_HIGH_ERR);
		
		if(1 == gCanRevPdoInfo.BMSRev053.b1ErrTemperDiff2)
			i32ErrCodeSet(BMS_BATTERY_TEMP_DIFF2_ERR);
		
		if(1 == gCanRevPdoInfo.BMSRev053.b1OutTempH1)
			i32ErrCodeSet(BMS_DISCHARGE_TEMP_HIGH2_ERR);
		
		if(1 == gCanRevPdoInfo.BMSRev053.b1OutTempH2)
			i32ErrCodeSet(BMS_DISCHARGE_TEMP_HIGH2_ERR);
		
//		if(1 == gCanRevPdoInfo.BMSRev053.b1OutTempL1)
//			i32ErrCodeSet();
//		
//		if(1 == gCanRevPdoInfo.BMSRev053.b1OutTempL2)
//			i32ErrCodeSet();
//		
//		if(1 == gCanRevPdoInfo.BMSRev053.b1OutTempH1)
//			i32ErrCodeSet();
		
		if(1 == gCanRevPdoInfo.BMSRev053.b1OutCurrentH1)
			i32ErrCodeSet(BMS_DISCHARGE_CUR_HIGH1_ERR);
		
		if(1 == gCanRevPdoInfo.BMSRev053.b1OutCurrentH2)
			i32ErrCodeSet(BMS_DISCHARGE_CUR_HIGH2_ERR);
		
		if(1 == gCanRevPdoInfo.BMSRev053.b1NormalTempVolL1)
			i32ErrCodeSet(BMS_TOTAL_VOL_LOW1_ERR);
		
//		if(1 == gCanRevPdoInfo.BMSRev053.b1LowTempVolL1)
//			i32ErrCodeSet();
		
		if(1 == gCanRevPdoInfo.BMSRev053.b1NormalTempVolL2)
//			i32ErrCodeSet(BMS_TOTAL_VOL_LOW2_ERR);
			
//		if(1 == gCanRevPdoInfo.BMSRev053.b1LowTempVolL2)
//			i32ErrCodeSet();
		
		if(1 == gCanRevPdoInfo.BMSRev053.b1NormalTempSingleVolL1)
			i32ErrCodeSet(BMS_SINGLE_VOL_LOW1_ERR);
			
//		if(1 == gCanRevPdoInfo.BMSRev053.b1LowTempSingleVolL1)
//			i32ErrCodeSet();
			
		if(1 == gCanRevPdoInfo.BMSRev053.b1NormalTempSingleVolL2)
			i32ErrCodeSet(BMS_SINGLE_VOL_LOW2_ERR);
			
//		if(1 == gCanRevPdoInfo.BMSRev053.b1LowTempSingleVolL2)	
//			i32ErrCodeSet();

	}
	else
	{
		
		if(0 != sgActLogic.u8Data)//�ж���ʱ��ռ���
		{
			u8Soc = u8BatteryMeter(DynamicSoc,6);//��̬�Ƶ���
			u16StableSocCnt = 0;
		}	
		if(u16StableSocCnt < (10000/USER_ECU_PERIOD))//��̬�Ƶ���
		{
			u8BatteryMeter(StaticSoc,6);//��̬�Ƶ���
			u16StableSocCnt++;
		}
		
		
		if(u16StableSocCnt > (6000/USER_ECU_PERIOD))
		{
			u8Soc = u8BatteryMeter(StaticSoc,6);//��̬�Ƶ���  6S����һ�ε���
		}
	}

	if((((u8Soc < 10 )||(u16AdcValue < 19800))
		&&(u16StableSocCnt < (2000/USER_ECU_PERIOD)))
		||((u16AdcValue < 23700)&&(u16StableSocCnt >(2000/USER_ECU_PERIOD))))        //��̬�µ�ѹС��23.7v��̬С��19.8v
	{/*��̬��socС��10�����ѹ����19.8 ��̬��ѹ����23.7*/
//		if(i32ErrCodeCheck(BATTERY_LOW_CAP1_ERR))//���һ���͵���
//		{
//			u16LowBatCnt = 0;
//			i32ErrCodeClr(BATTERY_LOW_CAP1_ERR);
//		}
		if((u16LowBatCnt2 < (5000 / USER_ECU_PERIOD))
			&&(0 == i32ErrCodeCheck(BAT_LOW_CAP2_ERR)))
		{
			u16LowBatCnt2 ++;
		}
		else
		{
			if(i32ErrCodeCheck(BATTERY_LOW_CAP1_ERR))//���һ���͵���
			{
				i32ErrCodeClr(BATTERY_LOW_CAP1_ERR);
			}	
			u16LowBatCnt2 = 0;
			i32ErrCodeSet(BAT_LOW_CAP2_ERR);
		}
	}
 

	if((((u8Soc < 20)||(u16AdcValue < 20800))
		&&(u16StableSocCnt < (2000/USER_ECU_PERIOD)))
		||((u16AdcValue < 24000)&&(u16StableSocCnt >(2000/USER_ECU_PERIOD))))     //2S������
	{/*36#һ���͵���,��ʻʱ��ѹ����24v 30s��
		����ʱ����20.8v��20s~30s*/
		if((u16LowBatCnt < (25000 / USER_ECU_PERIOD))
			&&(0 == i32ErrCodeCheck(BATTERY_LOW_CAP1_ERR)))
		{
			u16LowBatCnt ++;
		}
		else
		{
			u16LowBatCnt = 0;
			i32ErrCodeSet(BATTERY_LOW_CAP1_ERR);
		}
	}
//	}
//	else
//	{
//		u16LowBatCnt = 0;
//		i32ErrCodeClr(BAT_LOW_CAP2_ERR);
//		i32ErrCodeClr(BATTERY_LOW_CAP1_ERR);		
//	}
	vLowSocAlarm();
	i32SetPara(PARA_BmsSoc ,u8Soc*2.5);
	
}

/*******************************************************************************
* Name: void vEcuSetBeepPeriod(void)
* Descriptio: ���÷�����������
* Input: NULL
* Output: NULL
* checked: 
* to be check:�½�����������
*******************************************************************************/
static void vEcuSetBeepPeriod(void)
{
	static uint8_t u8Cnt = 0;
	
	if((1 == sgErrorState.b1Error)||(1 == sgErrorState.b1CaliErr)
		||(1 == sgErrorState.b1SaveSuccess))	/*���ɻָ�����*/
	{
		vBeepSetPeriod(60);
		vBeepSetOpenePeriod(120);
	}
	else if((1 == sgErrorState.b1Warning )
			||(1 == sgErrorState.b1CaliInit)
			||(1 == sgErrorState.b1CaliEnd)
			||(1 == sgErrorState.b1Verified))	/*�ɻָ�����*/
	{
		vBeepSetPeriod(180);
		vBeepSetOpenePeriod(240);
	}
	else if(((ABOVE_SWI != u8AntiPinchState)&&(UNDER_SWI_DELAY != u8AntiPinchState)&&(ANTIPINCH_INIT != u8AntiPinchState))
		||(true == u8GetNetTimerStartFlag(TIMER_EcuAntiPinchFunc)))		/*�½�����*/
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
	}
	else
	{
		sgErrorState.b1Error = 0;
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
		{
			sgLimit.b1NoLift = 1;
<<<<<<< .mine
//			if((1 == sgUserSets.b1Err18LiftAllow)
//				&&((118 == u8ErrCodeGet()))//���ô���18����
//			{
//				sgLimit.b1NoLift = 0;
//			}
		}
||||||| .r31
=======
		}
>>>>>>> .r49
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

<<<<<<< .mine
		if((0 != (u8ErrCodeGetAbnormal(ABNORMAL_Gear1Spd)))||(1 == sgSwiInput.b1PitSwi || sgSwiInput.b1LowestState ))
||||||| .r31
		if((0 != (u8ErrCodeGetAbnormal(ABNORMAL_Gear1Spd)))||(1 == sgSwiInput.b1PitSwi))
=======
		if((0 != (u8ErrCodeGetAbnormal(ABNORMAL_Gear1Spd)))||((1 == sgSwiInput.b1PitSwi) || (sgSwiInput.b1LowestState )))
>>>>>>> .r49
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

static void vSpeedContorl1()
{
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
	static uint16_t u16Timecnt = 0;//���ڼ��㲽����ʱ�����?
	static uint16_t u16TimeRange = 0;
	
	static uint8_t u8IncreaseDecreaseFlag = 0;

	#define STABLE_STATE		0
	#define INCREASE_STATE	1
	#define DECREASE_STATE	2
	
	
	
	#define ACC_STATIC	0		//Ĭ��ֵ��ָ���ٶ��ȶ��򲨶��Ƚ�С��
	#define ACC_FRESH		1  	//�Ӽ��ٲ�������
	#define ACC_EXCUTE	2		//�Ӽ���ִ��
	#define RVS_FRESH		3		//ָ���ʱ��������
	#define	RVS_EXCUTE	4		//ָ���ʱ�����ٲ����������?
	
	switch(u8AccFlag)
	{
		case ACC_STATIC://��λʱ����ָ��ֵ�����С��ֵ�������?
			fStepFactor = 0;
			if(u16SpeedTarget != u16SpeedCmd)
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
		case ACC_EXCUTE://Ŀ���ٶȱ仯������ִ�У��仯�������¼������?,�ٶȱƽ���ص���ʼ״�?
			fStepFactor = fFactorA * u16Timecnt * u16Timecnt + fFactorB * u16Timecnt + fFactorC;
			if(u16Timecnt<u16TimeRange)
				u16Timecnt ++;

			if(fStepFactor < (fStepRate / 10))//��Сб�ʲ�����״̬�¼��ٶȵ�1/10
			{
				fStepFactor = fStepRate / 10;
			}					

			if((u16SpeedTargetRec == u16SpeedTarget))//ָ���?С����ֵ
			{
				if(u16SpeedCmd < u16SpeedTarget)
				{
					if(u16Timecnt<u16TimeRange)
					{
						u16SpeedCmd += fStepFactor ;
					}
					else//�������?
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
					else//�������?
					{
						u16SpeedCmd = u16SpeedTarget ;
						u8AccFlag = ACC_STATIC;
					}
				}
			}
			else if(((u16SpeedCmd > u16SpeedTarget)&&(INCREASE_STATE == u8IncreaseDecreaseFlag ))
				||((u16SpeedCmd < u16SpeedTarget)&&(DECREASE_STATE == u8IncreaseDecreaseFlag )))//����ʱָ����ˣ������ʱָ��������?
			{
				u8AccFlag = RVS_FRESH;
			}
			else//ָ���?������ֵ�����¼���
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

static void vAccNormal(xMstSendPara *SendData)
{
	static xActLogic ActLogicRecord;
	static uint8_t u8SpeedRate = 0;
	static uint8_t u8MotorFlag;			/*Motor Act Advance or Delay*/
	
	static uint16_t u8DelayClose = 0;
	#define DELAY_CNT			(gUserInfo.u16ThrottleBDeadZoneMaxVal * 20)
		
	#define MOTOR_STATIC			0		 	/*�޶���״̬*/
	#define MOTOR_F_START			1 		/*ǰ����*/ 
	#define MOTOR_B_START			2 		/*������*/
	#define MOTOR_F_STOP 			3			/*ǰ��ɲ��*/
	#define MOTOR_B_STOP			4			/*����ɲ��*/
	#define MOTOR_RUN_STABLE 	5			/*�ȶ�����*/
	#define MOTOR_MODE_CHANGE	6			/*ǰ���л�*/

	/*MOTOR FEEDBACK VALUE*/
	#define	MOTOR_FEEDBACK_MAX_SPEED			gUserInfo.u16MotorMaxSpd
	#define	MOTOR_FEEDBACK_MIN_SPEED 			(MOTOR_FEEDBACK_MAX_SPEED * gUserInfo.u16Analog3DeadZoneMinVal / 100)//
	#define MOTOR_FEEDBAC_SHUTDOWN_SPEED	(MOTOR_FEEDBACK_MAX_SPEED * gUserInfo.u16Analog3DeadZoneMaxVal / 100)//�µ��ط�
	#define MOTOR_FB_OPEN_SPEED							(MOTOR_FEEDBACK_MAX_SPEED * gUserInfo.u16Analog3MidVal / 100)//0
	#define MOTOR_FEEDBACK_CHANGE_SPEED 			(MOTOR_FEEDBACK_MAX_SPEED * gUserInfo.u16ThrottleFMidVal / 100)
	#define MOTOR_FEEDBACK_STEER_CLOSE_SPEED (MOTOR_FEEDBACK_MAX_SPEED * gUserInfo.u16ThrottleFDeadZoneMinVal /100)
	#define MOTOR_FEEDBACK_UP_CLOSE_SPEED		(MOTOR_FEEDBACK_MAX_SPEED *  gUserInfo.u16ThrottleFDeadZoneMaxVal /100)
	#define MOTOR_FEEDBACK_UP_OPEN_SPEED		(MOTOR_FEEDBACK_MAX_SPEED *  gUserInfo.u16ThrottleBDeadZoneMinVal /100)
	#define MOTOR_HORIZON_CLOSE_SPEED				(MOTOR_FEEDBACK_MAX_SPEED * gUserInfo.u16ThrottleBMidVal /100)

	
	#define PUMP_STATIC			0
	#define PUMP_UP					1
	#define PUMP_UP_STOP		2
	#define PUMP_DOWN				3
	#define PUMP_DOWN_STOP	4

	if(((MOVE_MODE == u8PcuMode)||(MOTOR_STATIC != u8MotorFlag))
		&&(PUMP_STATIC == u8PumpFlag))
	{
		i32DoPwmSet(LIFTUP_PUMP,PUMP_CLOSE_PERCENTAGE);
		vPropSetTarget(LIFTDOWN_PUMP1, 0);
		vPropSetTarget(LIFTDOWN_PUMP2, 0);
		switch(u8MotorFlag)
		{
			case MOTOR_STATIC://��ֹ״̬���ȴ�ָ��
				if(1 == sgActLogic.b1ForwardAct)
				{
					u8MotorFlag = MOTOR_F_START;
				}
				else if(1 == sgActLogic.b1BackwardAct)
				{
					u8MotorFlag = MOTOR_B_START;
				}
				else
				{
					u8MotorFlag = MOTOR_STATIC;
				}
				u8DelayClose = 0;
				vMoveAccParaSets();
				u16SpeedTarget = 0;
				{
						i32DoPwmSet(HIGH_SPEED_PUMP,PUMP_CLOSE_PERCENTAGE);
				}
				break;
			case MOTOR_F_START://ǰ����
				if((1 == sgSwiInput.b1TiltSwi)&&(u16MotorFdb >= MOTOR_FEEDBACK_MIN_SPEED))//�µ��ϵ���һ��ת���ٿ�����ֹ����
				{
					i32DoPwmSet(FORWARD_PUMP,PUMP_OPEN_PERCENTAGE);
					u8MotorFlag = MOTOR_RUN_STABLE;
				}
				else if(0 == sgSwiInput.b1TiltSwi)//ƽ��ֱ�ӿ���
				{
					i32DoPwmSet(FORWARD_PUMP,PUMP_OPEN_PERCENTAGE);
					u8MotorFlag = MOTOR_RUN_STABLE;
				}
				else if(0 == sgActLogic.b1ForwardAct)
				{
					u8MotorFlag = MOTOR_STATIC;
				}
				else
				{
					u8MotorFlag = MOTOR_F_START;
				}
				vMoveAccParaSets();
				u16SpeedTarget = u16MotorVal;
				
				{
					if(FunctionEnable == gUserInfo.u8ParallelValveReverseFunc)
					{
						if(((1 == sgLimit.b1Slow)||(1 == sgLimit.b1SpeedAfterLift)))
						{
							i32DoPwmSet(HIGH_SPEED_PUMP,PUMP_CLOSE_PERCENTAGE);
						}
						else
						{
							i32DoPwmSet(HIGH_SPEED_PUMP,PUMP_OPEN_PERCENTAGE);
						}			
					}
					else
					{
						if(((1 == sgLimit.b1Slow)||(1 == sgLimit.b1SpeedAfterLift)))
						{
							i32DoPwmSet(HIGH_SPEED_PUMP,PUMP_OPEN_PERCENTAGE);
						}
						else
						{
							i32DoPwmSet(HIGH_SPEED_PUMP,PUMP_CLOSE_PERCENTAGE);
						}		
					}
				}
				break;
			case MOTOR_B_START:
				if((1 == sgSwiInput.b1TiltSwi)&&(u16MotorFdb >= MOTOR_FEEDBACK_MIN_SPEED))
				{
					i32DoPwmSet(BACKWARD_PUMP,PUMP_OPEN_PERCENTAGE);
					u8MotorFlag = MOTOR_RUN_STABLE;
				}
				else if(0 == sgSwiInput.b1TiltSwi)//ƽ�أ�ֱ�ӿ���
				{
					i32DoPwmSet(BACKWARD_PUMP,PUMP_OPEN_PERCENTAGE);
					u8MotorFlag = MOTOR_RUN_STABLE;
				}
				else if(0 == sgActLogic.b1BackwardAct)
				{
					u8MotorFlag = MOTOR_STATIC;
				}
				else
				{
					u8MotorFlag = MOTOR_B_START;
				}
				vMoveAccParaSets();
				u16SpeedTarget = u16MotorVal;
				
				{
					if(FunctionEnable == gUserInfo.u8ParallelValveReverseFunc)
					{
						if(((1 == sgLimit.b1Slow)||(1 == sgLimit.b1SpeedAfterLift)))
						{
							i32DoPwmSet(HIGH_SPEED_PUMP,PUMP_CLOSE_PERCENTAGE);
						}
						else
						{
							i32DoPwmSet(HIGH_SPEED_PUMP,PUMP_OPEN_PERCENTAGE);
						}			
					}
					else
					{
						if(((1 == sgLimit.b1Slow)||(1 == sgLimit.b1SpeedAfterLift)))
						{
							i32DoPwmSet(HIGH_SPEED_PUMP,PUMP_OPEN_PERCENTAGE);
						}
						else
						{
							i32DoPwmSet(HIGH_SPEED_PUMP,PUMP_CLOSE_PERCENTAGE);
						}		
					}
				}
				break;
			case MOTOR_F_STOP:
				if(((1 == sgActLogic.b1TurnLeft)||(1 == sgActLogic.b1TurnRight))
					&&(u16MotorFdb <= MOTOR_FEEDBACK_STEER_CLOSE_SPEED))//if steerlogic exists ,shut down rapid 
				{
					i32DoPwmSet(FORWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
					u8MotorFlag = MOTOR_STATIC;
				}
				else if((1 == sgActLogic.b1BackwardAct)&&(u16MotorFdb <= MOTOR_FB_OPEN_SPEED))//direction reverse
				{
					u8MotorFlag = MOTOR_MODE_CHANGE;
				}                              
				else if(1 == sgActLogic.b1ForwardAct)
				{
					u8MotorFlag = MOTOR_RUN_STABLE;
				}
				else if((1 == sgActLogic.b1BackwardAct)
					&&((1 == sgActLogic.b1TurnLeft)||(1 == sgActLogic.b1TurnRight))) //ת�������ֱ�ӽ���ģʽ�л�
				{
					u8MotorFlag = MOTOR_MODE_CHANGE;
				}
				else if(0 == sgSwiInput.b1TiltSwi)//ƽ��С�ڹط���ֱֵ�ӹط�
				{
					if(u16MotorFdb <= MOTOR_HORIZON_CLOSE_SPEED)
					{
						i32DoPwmSet(FORWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
						u8MotorFlag = MOTOR_STATIC;
					}
				}
				else if((1 == sgSwiInput.b1TiltSwi)&&(u16MotorFdb <= MOTOR_FEEDBAC_SHUTDOWN_SPEED))//wait motor slow down
				{
					u8DelayClose++;
					if(u8DelayClose > DELAY_CNT)
					{						
						i32DoPwmSet(FORWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
						u8MotorFlag = MOTOR_STATIC;
					}
				}
				else
				{
					u8MotorFlag = MOTOR_F_STOP;
				}
				if(MOTOR_F_STOP != u8MotorFlag)
				{					
					if(FunctionEnable == gUserInfo.u8ParallelValveReverseFunc)
					{
						if(((1 == sgLimit.b1Slow)||(1 == sgLimit.b1SpeedAfterLift)))
						{
							i32DoPwmSet(HIGH_SPEED_PUMP,PUMP_OPEN_PERCENTAGE);
						}
						else
						{
							i32DoPwmSet(HIGH_SPEED_PUMP,PUMP_CLOSE_PERCENTAGE);
						}			
					}
					else
					{
						if(((1 == sgLimit.b1Slow)||(1 == sgLimit.b1SpeedAfterLift)))
						{
							i32DoPwmSet(HIGH_SPEED_PUMP,PUMP_CLOSE_PERCENTAGE);
						}
						else
						{
							i32DoPwmSet(HIGH_SPEED_PUMP,PUMP_OPEN_PERCENTAGE);
						}		
					}
				}
				vMoveDecParaSets();
				u16SpeedTarget = 0;
				break;
			case MOTOR_B_STOP:
				if(((1 == sgActLogic.b1TurnLeft)||(1 == sgActLogic.b1TurnRight))
					&&(u16MotorFdb <= MOTOR_FEEDBACK_STEER_CLOSE_SPEED))//if steerlogic exists ,shut down rapid 
				{
					i32DoPwmSet(BACKWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
					u8MotorFlag = MOTOR_STATIC;
				}
				else if((1 == sgActLogic.b1ForwardAct)&&(u16MotorFdb <= MOTOR_FB_OPEN_SPEED))//direction reverse
				{
					u8MotorFlag = MOTOR_MODE_CHANGE;       //��סת��ʱ�ٶȲ������벻�˵��״̬ת��
				}
				else if(1 == sgActLogic.b1BackwardAct)//
				{
					u8MotorFlag = MOTOR_RUN_STABLE;
				}
				else if((1 == sgActLogic.b1ForwardAct)
					&&((1 == sgActLogic.b1TurnLeft)||(1 == sgActLogic.b1TurnRight))) //ת�������ֱ�ӽ���ģʽ�л�
				{
					u8MotorFlag = MOTOR_MODE_CHANGE;
				}
				else if(0 == sgSwiInput.b1TiltSwi)
				{
					if(u16MotorFdb <= MOTOR_HORIZON_CLOSE_SPEED)
					{

						i32DoPwmSet(BACKWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
						u8MotorFlag = MOTOR_STATIC;
					}
//					else
//						u8PumpCloseDelay++;
				}
				else if((1 == sgSwiInput.b1TiltSwi)&&(u16MotorFdb <= MOTOR_FEEDBAC_SHUTDOWN_SPEED))//wait motor slow down
				{
					u8DelayClose++;
					if(u8DelayClose > DELAY_CNT)
					{					
						{
						i32DoPwmSet(HIGH_SPEED_PUMP,PUMP_CLOSE_PERCENTAGE);
						}
					i32DoPwmSet(BACKWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
					u8MotorFlag = MOTOR_STATIC;					
					}
				}
				else
				{
					u8MotorFlag = MOTOR_B_STOP;
				}
								
				if(MOTOR_B_STOP != u8MotorFlag)
				{					
					if(FunctionEnable == gUserInfo.u8ParallelValveReverseFunc)
					{
						if(((1 == sgLimit.b1Slow)||(1 == sgLimit.b1SpeedAfterLift)))
						{
							i32DoPwmSet(HIGH_SPEED_PUMP,PUMP_OPEN_PERCENTAGE);
						}
						else
						{
							i32DoPwmSet(HIGH_SPEED_PUMP,PUMP_CLOSE_PERCENTAGE);
						}			
					}
					else
					{
						if(((1 == sgLimit.b1Slow)||(1 == sgLimit.b1SpeedAfterLift)))
						{
							i32DoPwmSet(HIGH_SPEED_PUMP,PUMP_CLOSE_PERCENTAGE);
						}
						else
						{
							i32DoPwmSet(HIGH_SPEED_PUMP,PUMP_OPEN_PERCENTAGE);
						}		
					}
				}
				vMoveDecParaSets();
				u16SpeedTarget = 0;
				break;
			case MOTOR_RUN_STABLE:
				if((1 == ActLogicRecord.b1ForwardAct)&&(0 == sgActLogic.b1ForwardAct))
				{
					u8MotorFlag = MOTOR_F_STOP;
				}
				else if((1 == ActLogicRecord.b1BackwardAct)&&(0 == sgActLogic.b1BackwardAct))
				{
					u8MotorFlag = MOTOR_B_STOP;
				}
				else 
				{
					u8MotorFlag = MOTOR_RUN_STABLE;
				}
				if(FunctionEnable == gUserInfo.u8ParallelValveReverseFunc)
				{
					if(((1 == sgLimit.b1Slow)||(1 == sgLimit.b1SpeedAfterLift)))
					{
						i32DoPwmSet(HIGH_SPEED_PUMP,PUMP_CLOSE_PERCENTAGE);
					}
					else
					{
						i32DoPwmSet(HIGH_SPEED_PUMP,PUMP_OPEN_PERCENTAGE);
					}			
				}
				else
				{
					if(((1 == sgLimit.b1Slow)||(1 == sgLimit.b1SpeedAfterLift)))
					{
						i32DoPwmSet(HIGH_SPEED_PUMP,PUMP_OPEN_PERCENTAGE);
					}
					else
					{
						i32DoPwmSet(HIGH_SPEED_PUMP,PUMP_CLOSE_PERCENTAGE);
					}		
				}
				vMoveStableParaSets();
				u16SpeedTarget = u16MotorVal;
				break;
			case MOTOR_MODE_CHANGE:
				if(u16MotorFdb <= MOTOR_FEEDBACK_CHANGE_SPEED
					||(1 == sgActLogic.b1TurnLeft)||(1 == sgActLogic.b1TurnRight)) //С����С�����ٶȻ���ת��ָ�������
				{
					if(1 == sgActLogic.b1BackwardAct)
					{
						i32DoPwmSet(BACKWARD_PUMP,PUMP_OPEN_PERCENTAGE);
						i32DoPwmSet(FORWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
						u8MotorFlag = MOTOR_RUN_STABLE;
					}
					else if(1 == sgActLogic.b1ForwardAct)
					{
						i32DoPwmSet(BACKWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
						i32DoPwmSet(FORWARD_PUMP,PUMP_OPEN_PERCENTAGE);
						u8MotorFlag = MOTOR_RUN_STABLE;
					}
					else
					{
						i32DoPwmSet(BACKWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
						i32DoPwmSet(FORWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
						u8MotorFlag = MOTOR_STATIC;
					}
				}
				else
				{
					i32DoPwmSet(BACKWARD_PUMP,PUMP_OPEN_PERCENTAGE);
					i32DoPwmSet(FORWARD_PUMP,PUMP_OPEN_PERCENTAGE);				
				}
				vMoveDecParaSets();
				u16SpeedTarget = 0;
				break;
			default:
				i32DoPwmSet(BACKWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
				i32DoPwmSet(FORWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
				vMoveDecParaSets();
				u16SpeedTarget = 0;
				u8MotorFlag = MOTOR_STATIC;
				break;
		}	
		/*Speed Rate Limit*/

		u8SpeedRate = gUserInfo.u8FastDriveSpeed;//Ĭ��
		if(1 == sgLimit.b1SpeedAfterLift)
		{
			if(u8SpeedRate > gUserInfo.u8DriveSpeedAfterLift)
			u8SpeedRate = gUserInfo.u8DriveSpeedAfterLift;
		}
		if(1 == sgLimit.b1Slow)
		{
			if(u8SpeedRate > gUserInfo.u8SlowDriveSpeed)
			u8SpeedRate = gUserInfo.u8SlowDriveSpeed;
		}
//		if(FunctionEnable == gUserInfo.u8ParallelValveReverseFunc)
//		{
//			if(((1 == sgLimit.b1Slow)||(1 == sgLimit.b1SpeedAfterLift)))
//			{
//				i32DoPwmSet(HIGH_SPEED_PUMP,PUMP_CLOSE_PERCENTAGE);
//			}
//			else
//			{
//				i32DoPwmSet(HIGH_SPEED_PUMP,PUMP_OPEN_PERCENTAGE);
//			}			
//		}
//		else
//		{
//			if(((1 == sgLimit.b1Slow)||(1 == sgLimit.b1SpeedAfterLift)))
//			{
//				i32DoPwmSet(HIGH_SPEED_PUMP,PUMP_OPEN_PERCENTAGE);
//			}
//			else
//			{
//				i32DoPwmSet(HIGH_SPEED_PUMP,PUMP_CLOSE_PERCENTAGE);
//			}
//		}

		
		if((1 == sgActLogic.b1TurnLeft)||(1 == sgActLogic.b1TurnRight))
		{
			if(u8SpeedRate > gUserInfo.u8MaxTurnSpeed)
				u8SpeedRate = gUserInfo.u8MaxTurnSpeed;		
		}
		
		u16SpeedTarget = u16SpeedTarget * u8SpeedRate / 100;
		
			/*steer process*/
		if(1 ==sgActLogic.b1TurnLeft)
		{
			
			i32DoPwmSet(TURNLEFT_PUMP,PUMP_OPEN_PERCENTAGE);
			if(u16SpeedTarget < (gUserInfo.u8TurnPowerLimit * 4095 / 100))
				u16SpeedTarget = gUserInfo.u8TurnPowerLimit * 4095 / 100;
		}
		else
		{
			i32DoPwmSet(TURNLEFT_PUMP,PUMP_CLOSE_PERCENTAGE);
		}
		
		if(1 == sgActLogic.b1TurnRight)
		{
			
			i32DoPwmSet(TURNRIGHT_PUMP,PUMP_OPEN_PERCENTAGE);
			if(u16SpeedTarget < (gUserInfo.u8TurnPowerLimit * 4095 / 100))
				u16SpeedTarget = gUserInfo.u8TurnPowerLimit * 4095 / 100;
		}
		else
		{
			i32DoPwmSet(TURNRIGHT_PUMP,PUMP_CLOSE_PERCENTAGE);
		}

		vSpeedControl();
	}
	else if((((LIFT_MODE == u8PcuMode)||(PUMP_STATIC != u8PumpFlag))&&(MOTOR_STATIC == u8MotorFlag))
		||((0 == i32LocalDiGet(PCU_SWICTH))))
	{
		i32DoPwmSet(BACKWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
		i32DoPwmSet(FORWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
		i32DoPwmSet(TURNLEFT_PUMP,PUMP_CLOSE_PERCENTAGE);
		if(FunctionEnable == gUserInfo.u8ParallelValveReverseFunc)
		{
			{
				i32DoPwmSet(HIGH_SPEED_PUMP,PUMP_OPEN_PERCENTAGE);
			}			
		}
		else
		{
			{
				i32DoPwmSet(HIGH_SPEED_PUMP,PUMP_CLOSE_PERCENTAGE);
			}
		}
		i32DoPwmSet(HIGH_SPEED_PUMP,PUMP_CLOSE_PERCENTAGE);
		/*lift process*/
		switch(u8PumpFlag)
		{
			case PUMP_STATIC://��ֹ���ȴ�����ָ��
				if(1 == sgActLogic.b1LiftUpAct)
				{
					u8PumpFlag = PUMP_UP;
				}
				else if(1 == sgActLogic.b1LiftDownAct)
				{
					u8PumpFlag = PUMP_DOWN;
				}
				break;
			case PUMP_UP://�ȶ�ִ�У�ָ��ı�ʱ�л�?
				u16SpeedTarget = u16MotorVal;
				u8SpeedRate = gUserInfo.u8LiftSpeed;
				if((0 == sgActLogic.b1LiftUpAct)||(1 == sgActLogic.b1LiftDownAct))
				{
					u8PumpFlag = PUMP_UP_STOP;
				}
				else if (u16MotorFdb <= MOTOR_FEEDBACK_UP_OPEN_SPEED)//ת��С����ֵ����ת
				{
					u16Time = gUserInfo.u8AccAndDecLift * TIME_FACTOR;
				}
				else//ת�ٴ�����ֵ���ȶ�
				{
					i32DoPwmSet(LIFTUP_PUMP,PUMP_OPEN_PERCENTAGE);
					u16Time = gUserInfo.u8CurveLift * TIME_FACTOR;
				}
				break;
			case PUMP_UP_STOP:
				u16SpeedTarget = 0;
				u16Time = gUserInfo.u8BrakeLift * TIME_FACTOR;
				u8SpeedRate = gUserInfo.u8LiftSpeed;				
				if(u16MotorFdb <= MOTOR_FEEDBACK_UP_CLOSE_SPEED)
				{
					i32DoPwmSet(LIFTUP_PUMP,PUMP_CLOSE_PERCENTAGE);
					u8PumpFlag = PUMP_STATIC;
				}
				else if(1 == sgActLogic.b1LiftUpAct)
				{
					u8PumpFlag = PUMP_UP;
				}

				break;
			case PUMP_DOWN:
				u16SpeedTarget = u16MotorVal;
				u16Time = gUserInfo.u8AccAndDecLower * TIME_FACTOR;
				u8SpeedRate = gUserInfo.u8LowerSpeed;
				if((1 == sgUserSets.b1AntipinchSlow)&&(u8AntiPinchState == UNDER_SWI_ACT))
				{
					u8SpeedRate = gUserInfo.u8BrakeAntiPinch & 0x7F;
				}
				if((1 == sgActLogic.b1LiftUpAct)||(0 == sgActLogic.b1LiftDownAct))
				{
					u8PumpFlag = PUMP_DOWN_STOP;
				}
				break;
			case PUMP_DOWN_STOP:
				u16SpeedTarget = 0;
				u16Time = gUserInfo.u8BrakeLower * TIME_FACTOR;
				u8SpeedRate = gUserInfo.u8LowerSpeed;			
				if((1 == sgUserSets.b1AntipinchSlow)&&(u8AntiPinchState == UNDER_SWI_ACT))
				{
					u8SpeedRate = gUserInfo.u8BrakeAntiPinch & 0x7F;
				}
				if((inserted_data[1] * PROP_CURRENT_FACOTR) < (gUserInfo.fPropMinCurrent1 * 1.5))
				{
					u8PumpFlag = PUMP_STATIC;
				}
				else if(1 == sgActLogic.b1LiftDownAct)
				{
					u8PumpFlag = PUMP_DOWN;
				}
				break;
			default:
				u16SpeedTarget = 0;
				u16SpeedCmd = 0;
				vPropSetTarget(LIFTDOWN_PUMP1, 0);
				vPropSetTarget(LIFTDOWN_PUMP2, 0);
				i32DoPwmSet(LIFTUP_PUMP,PUMP_CLOSE_PERCENTAGE);
				u8PumpFlag = PUMP_STATIC;
				break;
		}
		i32SetPara(PARA_ForwardValveCurrent,u8PumpFlag);
		i32SetPara(PARA_BackValveCurrent,u8ParaSetMode);
		
		u16SpeedTarget = u16SpeedTarget * u8SpeedRate / 100;
		vSpeedControl();

		if((PUMP_DOWN == u8PumpFlag)||(PUMP_DOWN_STOP == u8PumpFlag))
		{
			int32_t i32prop = 0;
			i32prop = _IQ((gUserInfo.fPropMinCurrent1 + (gUserInfo.fPropMaxCurrent1 - gUserInfo.fPropMinCurrent1) * u16SpeedCmd / MOTOR_MAX_SPEED_VALUE) / PROPD_STD_CURRENT);
			vPropSetTarget(LIFTDOWN_PUMP1, i32prop);
			vPropSetTarget(LIFTDOWN_PUMP2, i32prop);
		}
		else
		{
			vPropSetTarget(LIFTDOWN_PUMP1, 0);
			vPropSetTarget(LIFTDOWN_PUMP2, 0);
		}
	}
	else
	{
		u16SpeedTarget = 0;
		i32DoPwmSet(BACKWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
		i32DoPwmSet(FORWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
		i32DoPwmSet(TURNLEFT_PUMP,PUMP_CLOSE_PERCENTAGE);
		i32DoPwmSet(TURNRIGHT_PUMP,PUMP_CLOSE_PERCENTAGE);
		i32DoPwmSet(LIFTUP_PUMP,PUMP_CLOSE_PERCENTAGE);
		vPropSetTarget(LIFTDOWN_PUMP1, 0);
		vPropSetTarget(LIFTDOWN_PUMP2, 0);
	}		

	
	
	if((u8PumpFlag != PUMP_DOWN_STOP)&&(u8PumpFlag != PUMP_DOWN)
		&&((u8PumpFlag != PUMP_STATIC)||
		((u8MotorFlag != MOTOR_STATIC)||(0 != sgActLogic.b1TurnLeft)||(0 != sgActLogic.b1TurnRight))))//���½�״̬ʱ������ָ��
	{
		SendData->u8TargetHigh = u16SpeedCmd >> 8;
		SendData->u8TargetLow = u16SpeedCmd;
		
		if (0 != u16SpeedCmd)
		{
			SendData->b1ServoOn = 1;
			SendData->b1ForwardReq = 1;
		}		
	}
	else
	{
		SendData->u8TargetHigh = 0;
		SendData->u8TargetLow = 0;
	}
//	if((u8PumpFlag == PUMP_DOWN_STOP)
//		||(MOTOR_F_STOP == u8MotorFlag)
//		||(MOTOR_B_STOP == u8MotorFlag))
//	{ɲ�������޷���ems��brake���٣������������˶�ʱ�л��ᵼ�¿��١�
//		if(0 != u16MotorFdb)
//		SendData->b1EmsReq = 1;
//	}
	
	ActLogicRecord.u8Data = sgActLogic.u8Data;
	i32SetPara(PARA_TurnRightValveCurrent,u16SpeedCmd);
}
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
	else if(1 == sgActLogic.b1LiftUpAct)
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

static void vAccTestProc(xMstSendPara *SendData)
{
	if((1 == sgActLogic.b1LiftDownAct )&& (0 == sgLimit.b1NoDown))
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
	else	if((1 == sgActLogic.b1LiftUpAct) && (0 == sgLimit.b1NoLift) && (0 == i32ErrCodeCheck(BATTERY_LOW_CAP1_ERR)) && (0 == i32ErrCodeCheck(BAT_LOW_CAP2_ERR)))
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
			vAccPressureCalibration(SendData);
			//vAccParaSets(SendData);
			break;
		case PRESSURE_CALIBRATION_MODE:
			vAccPressureCalibration(SendData);
			break;
<<<<<<< .mine
		#ifdef LIUGONG_TEST
		case AUTO_LIFT_MODE:
			vAccPressureCalibration(SendData);
			break; 
		#endif
||||||| .r31
			break;
=======
		#ifdef LIUGONG_TEST
		case AUTO_LIFT_MODE:
			vAccTestProc(SendData);
			break; 
		#endif
>>>>>>> .r49
		default:
			break;
	}
}
																								
static void vMstSendProc(xMstSendPara *SendData) 
{


	SendData->buf[2] = 0;
	vAccProcess(SendData);

}

void vCanLostProc(uint32_t u32CanID,uint8_t u8State) 
{
	switch(u32CanID)
	{
		case 0x51:
			if(CAN_NORMAL == u8State)
			{
				i32ErrCodeClr(BMS_OFFLINE);
			}
			else if(CAN_LOST == u8State)
			{
				if(LiBattery == gUserInfo.u8BatteryType)
					i32ErrCodeSet(BMS_OFFLINE);
			}
			break;
		case 0x166:
			if((1 == sgHeartBeatLockDynamic.b1HeartBeatQuery)
				//&&(1 == sgHeartBeatLockDynamic.b1ECUHeartQuery)
				)
			{
				if(CAN_NORMAL == u8State)
				{
					i32ErrCodeClr(HEART_LOCK_ALARMING);
					i32ErrCodeClr(HEART_BEAT_LOCK_ERR);
					if((1 == sgHeartBeatLockDynamic.b1HeartBeatLock)&&(1 == u8EcuProcFlag))
					{
						sgHeartBeatLockDynamic.b1HeartBeatLock = 0;
						u16SaveParaToEeprom(PARA_HeartBeatQueryFunc,sgHeartBeatLockDynamic.u8Data);
					}					
				}
				else if(CAN_LOST == u8State)  //Can��ʧ
				{
					if(0 == i32ErrCodeCheck(HEART_BEAT_LOCK_ERR))
						i32ErrCodeSet(HEART_LOCK_ALARMING);
					if(0 == sgHeartBeatLockDynamic.b1HeartBeatLock)
					{
						sgHeartBeatLockDynamic.b1HeartBeatLock = 1;
						u16SaveParaToEeprom(PARA_HeartBeatQueryFunc,sgHeartBeatLockDynamic.u8Data);
					}
				}				
			}

		default:
			break;
	}
}

static void vTboxParaInit(void)
{
	uint16_t u16UpDateCnt = 0;
	
	u16Read32DataFromEeprom(LIFTTIMES_ADDR_L16,LIFTTIMES_ADDR_H16,&sgActionTimes.u32LiftTimes,1);
	u16Read32DataFromEeprom(STEERTIMES_ADDR_L16,STEERTIMES_ADDR_H16,&sgActionTimes.u32SteerTimes,1);
	u16Read32DataFromEeprom(MOVETIES_ADDR_L16,MOVETIES_ADDR_H16,&sgActionTimes.u32MoveTimes,1);
	u16Read32DataFromEeprom(OVERLOADTIMES_ADDR_L16,OVERLOADTIMES_ADDR_H16,&sgActionTimes.u32OverLoadTimes,1);
	
	u16EepromRead(UPDATE_COUNT_ADDR_L16,&u16UpDateCnt,1);
	if((0xFFFF == u16UpDateCnt))//��һ�ζ�ȡ
	{
		u16EepromWrite(UPDATE_COUNT_ADDR_L16,0,1);
		sgActionTimes.u16UpDateTimes = 0;
	}
	else
	{
		sgActionTimes.u16UpDateTimes = u16UpDateCnt;
	}
	
	u16Read32DataFromEeprom(LIFTHOUR_ADDR_L16,LIFTHOUR_ADDR_H16,&sgActionour.u32LifHour,1);
	u16Read32DataFromEeprom(STEERHOUR_ADDR_L16,STEERHOUR_ADDR_H16,&sgActionour.u32SteerHour,1);
	u16Read32DataFromEeprom(LOWERMOVEHOUR_ADDR_L16,LOWERMOVEHOUR_ADDR_H16,&sgActionour.u32LowerMoveHour,1);
	u16Read32DataFromEeprom(UPMOVEHOUR_ADDR_L16,UPMOVEHOUR_ADDR_H16,&sgActionour.u32UpMoveHour,1);
	u16Read32DataFromEeprom(DOWNHOUR_ADDR_L16,DOWNHOUR_ADDR_H16,&sgActionour.u32DownHour,1);
}


//T-BOX
void vTBoxProc(void)
{
	static	xActLogic ActRecord ;
	static xActionFlag ActionFlagRec;
	static uint8_t u8SendCnt = 0;
	static uint16_t u16Cnt5Min;
	static uint16_t u16SecondCnt;
	
	tCanFrame Canframe;
	uint8_t i = 0;
	
	static xCanSend163 CanSend163;
	static xCanSend165 CanSend165;
	static xCanSend167	CanSend167;
	static xCanSend168 CanSend168;

	static xCanSend170 CanSend170;
	static xCanSend171	CanSend171;
	static xCanSend172	CanSend172;
	static xCanSend173	CanSend173;
	static xCanSend174	CanSend174;
	static xCanSend175	CanSend175;
	static xCanSend176	CanSend176;
	static xCanSend177	CanSend177;
	static xCanSend178	CanSend178;
	static xCanSend179	CanSend179;
	static xCanSend17A	CanSend17A;
	static xCanSend17B	CanSend17B;
	static xCanSend17C	CanSend17C;
	static xCanSend17D	CanSend17D;
	static xCanSend17E	CanSend17E;
	static xCanSend050 CanSend050;
	static xCanSend055 CanSend055;
	static xCanSend190 CanSend190;
	static xCanSend25D CanSend25D;
	static xCanSend55E CanSend55E;
	
	uint16_t u16AngleTmp = 0;
	u16AngleTmp = (i32LocalAiGetValue(ANGLE_SENSOR_CHANNEL) * 90 /100 / 5  + 600);
	
	if(false == u8GetNetTimerStartFlag(Timer_OneSecond))
	{
		//vSetNetTimer(ONE_MINUT_CNT, 60000);//
		vSetNetTimer(Timer_OneSecond, 1000);
	}
	if(true == u8GetNetTimerOverFlag(Timer_OneSecond))//ÿ5���ӷ���һ��
	{
		if(u16SecondCnt < 3000)
			u16SecondCnt++;
		else
			u16SecondCnt = 0;
		
		if(u16Cnt5Min<(5 ))
		{
			u16Cnt5Min++;
		}		
		else
		{
			u16Cnt5Min = 0;
			u8SendCnt = 0;
			sgActionTimes.u16UpDateTimes++;
			u16EepromWrite(UPDATE_COUNT_ADDR_L16,sgActionTimes.u16UpDateTimes,1);
		}
		vResetNetTimer(Timer_OneSecond);
	}
	
	{
		memset(&CanSend163,0,sizeof(CanSend163));
		memset(&CanSend165,0,sizeof(CanSend165));
		memset(&CanSend167,0,sizeof(CanSend167));
		memset(&CanSend168,0,sizeof(CanSend168));
		memset(&CanSend170,0,sizeof(CanSend170));
		memset(&CanSend171,0,sizeof(CanSend171));
		memset(&CanSend172,0,sizeof(CanSend172));
		memset(&CanSend173,0,sizeof(CanSend173));
		memset(&CanSend174,0,sizeof(CanSend174));
		memset(&CanSend175,0,sizeof(CanSend175));
		memset(&CanSend176,0,sizeof(CanSend176));
		memset(&CanSend177,0,sizeof(CanSend177));
		memset(&CanSend178,0,sizeof(CanSend178));
		memset(&CanSend179,0,sizeof(CanSend179));
		memset(&CanSend17A,0,sizeof(CanSend17A));
		memset(&CanSend17B,0,sizeof(CanSend17B));
		memset(&CanSend17C,0,sizeof(CanSend17C));
		memset(&CanSend17D,0,sizeof(CanSend17D));
		memset(&CanSend17E,0,sizeof(CanSend17E));
		memset(&CanSend050,0,sizeof(CanSend050));
		memset(&CanSend055,0,sizeof(CanSend055));
		memset(&CanSend190,0,sizeof(CanSend190));
		memset(&CanSend25D,0,sizeof(CanSend25D));
		memset(&CanSend55E,0,sizeof(CanSend55E));
		memset(&Canframe.u8Data,0,sizeof(Canframe.u8Data));
	}
	
	/*action times & hours */
	{ static uint8_t u8SteerCnt ;
		static uint8_t u8OverLoadCnt ;
		static uint8_t u8LiftCnt ;
		static uint8_t u8DownCnt;
		static uint8_t u8MoveLow;
		static uint8_t u8MoveUp;

		if((1 == sgActLogic.b1TurnLeft)
			 ||(1 == sgActLogic.b1TurnRight))//ת���־λ
		{
			if((0 == sgActionFlag.b1SteerState)&&(u8SteerCnt < 200))
				u8SteerCnt++;
			else
				sgActionFlag.b1SteerState = 1;
		}
		else if((0 == sgActLogic.b1TurnLeft)
			 &&(0 == sgActLogic.b1TurnRight))
		{
			u8SteerCnt = 0;
			sgActionFlag.b1SteerState = 0;			
		}
		
		if(1 == sgActLogic.b1LiftUpAct)//������־λ
		{
			if((0 == sgActionFlag.b1LiftState)&&(u8LiftCnt < 200))
				u8LiftCnt++;
			else
				sgActionFlag.b1LiftState = 1;
		}
		else if(0 == sgActLogic.b1LiftUpAct)
		{
			u8LiftCnt = 0;
			sgActionFlag.b1LiftState = 0;			
		}
		
		
		if(1 == sgActLogic.b1LiftDownAct)//�½���־λ
		{
			if((0 == sgActionFlag.b1LiftDownState)&&(u8DownCnt < 200))
				u8DownCnt++;
			else
				sgActionFlag.b1LiftDownState = 1;
		}
		else if(0 == sgActLogic.b1LiftDownAct)
		{
			u8DownCnt = 0;
			sgActionFlag.b1LiftDownState = 0;			
		}
		
		/*move low*/
		if(((1 == sgActLogic.b1BackwardAct)||(1 == sgActLogic.b1ForwardAct))
			&&(1 == sgSwiInput.b1DownLimitSwi))//��λ����
		{
			if((0 == sgActionFlag.b1LowerMove)&&(u8MoveLow < 200))
				u8MoveLow++;
			else
				sgActionFlag.b1LowerMove = 1;
		}
		else if((0 == sgActLogic.b1BackwardAct)&&(0 == sgActLogic.b1ForwardAct))
		{
			u8MoveLow = 0;
			sgActionFlag.b1LowerMove = 0;			
		}
		
		if(((1 == sgActLogic.b1BackwardAct)||(1 == sgActLogic.b1ForwardAct))
			&&(0 == sgSwiInput.b1DownLimitSwi))//��λ����
		{
			if((0 == sgActionFlag.b1UpMove)&&(u8MoveUp < 200))
				u8MoveUp++;
			else
				sgActionFlag.b1UpMove = 1;
		}
		else if((0 == sgActLogic.b1BackwardAct)&&(0 == sgActLogic.b1ForwardAct))
		{
			u8MoveUp = 0;
			sgActionFlag.b1UpMove = 0;			
		}
		
				
		if(0 != i32ErrCodeCheck(PLAT_OVERLOAD_ERR))
		{
			if((0 == sgActionFlag.b1OverLoadState)&&(u8OverLoadCnt < 200))
				u8OverLoadCnt++;
			else
				sgActionFlag.b1OverLoadState = 1;
		}
		else
		{
			u8OverLoadCnt = 0;
			sgActionFlag.b1OverLoadState = 0;			
		}
	}
	{
		static uint16_t u16DownTimeCnt;
		static uint16_t u16LiftTimeCnt;
		static uint16_t u16SteerTimeCnt;
		static uint16_t u16LowerMoveTimeCnt;
		static uint16_t u16UpMoveTimeCnt;
		
		
		static xActionHour ActionHourRec;//��¼��ǰ����ʱ�䣬����ֹͣ�󱣴浽eeprome
		static uint16_t u16SecondCntRec;
		uint16_t u16DiffTime = 0;
		
		//�½�ʱ��
		if((0 == ActionFlagRec.b1LiftDownState)&&(1 == sgActionFlag.b1LiftDownState))
		{
//			ActionHourRec.u32DownHour = 0;
		}
		
		//����ʱ��
		if((0 == ActionFlagRec.b1LiftState)&&(1 == sgActionFlag.b1LiftState))
		{
			sgActionTimes.u32LiftTimes ++;
			u16Write32DataToEeprom(LIFTTIMES_ADDR_L16,LIFTTIMES_ADDR_H16,sgActionTimes.u32LiftTimes,1);
		}		
				//��λ����
		if((0 == ActionFlagRec.b1LowerMove)&&(1 == sgActionFlag.b1LowerMove))
		{
			sgActionTimes.u32MoveTimes ++;
			u16Write32DataToEeprom(MOVETIES_ADDR_L16,MOVETIES_ADDR_H16,sgActionTimes.u32MoveTimes,1);
		}
		//����ʱ��
		if((0 == ActionFlagRec.b1OverLoadState)&&(1 == sgActionFlag.b1OverLoadState))
		{
			sgActionTimes.u32OverLoadTimes ++;
			u16Write32DataToEeprom(OVERLOADTIMES_ADDR_L16,OVERLOADTIMES_ADDR_H16,sgActionTimes.u32OverLoadTimes,1);			
		}
		//ת��ʱ��
		if((0 == ActionFlagRec.b1SteerState)&&(1 == sgActionFlag.b1SteerState))
		{
			sgActionTimes.u32SteerTimes ++;
			u16Write32DataToEeprom(STEERTIMES_ADDR_L16,STEERTIMES_ADDR_H16,sgActionTimes.u32SteerTimes,1);			
		}
		//��λ����
		if((0 == ActionFlagRec.b1UpMove)&&(1 == sgActionFlag.b1UpMove))
		{
			sgActionTimes.u32MoveTimes ++;
			u16Write32DataToEeprom(MOVETIES_ADDR_L16,MOVETIES_ADDR_H16,sgActionTimes.u32MoveTimes,1);
		}
		
		if((1 == sgActionFlag.b1LiftState)&&(u16SecondCntRec != u16SecondCnt))
		{
			ActionHourRec.u32LifHour ++;
			if(0 == (ActionHourRec.u32LifHour % 10))
			{
				sgActionour.u32LifHour ++;
				u16Write32DataToEeprom(LIFTHOUR_ADDR_L16,LIFTHOUR_ADDR_H16,sgActionour.u32LifHour,1);
			}
		}
		if((1 == sgActionFlag.b1LiftDownState)&&(u16SecondCntRec != u16SecondCnt))
		{
			ActionHourRec.u32DownHour ++;
			if(0 == (ActionHourRec.u32DownHour % 10))
			{
				sgActionour.u32DownHour ++;
				u16Write32DataToEeprom(DOWNHOUR_ADDR_L16,DOWNHOUR_ADDR_H16,sgActionour.u32DownHour,1);
			}
		}
		if((1 == sgActionFlag.b1LowerMove)&&(u16SecondCntRec != u16SecondCnt))
		{
			ActionHourRec.u32LowerMoveHour ++;
			if(0 == (ActionHourRec.u32LowerMoveHour % 10))
			{
				sgActionour.u32LowerMoveHour ++;
				u16Write32DataToEeprom(LOWERMOVEHOUR_ADDR_L16,LOWERMOVEHOUR_ADDR_H16,sgActionour.u32LowerMoveHour,1);
			}
		}
		if((1 == sgActionFlag.b1SteerState)&&(u16SecondCntRec != u16SecondCnt))
		{
			ActionHourRec.u32SteerHour ++;
			if(0 == (ActionHourRec.u32SteerHour % 10))
			{
				sgActionour.u32SteerHour ++;
				u16Write32DataToEeprom(STEERHOUR_ADDR_L16,STEERHOUR_ADDR_H16,sgActionour.u32SteerHour,1);
			}
		}
		if((1 == sgActionFlag.b1UpMove)&&(u16SecondCntRec != u16SecondCnt))
		{
			ActionHourRec.u32UpMoveHour ++;
			if(0 == (ActionHourRec.u32UpMoveHour % 10))
			{
				sgActionour.u32UpMoveHour ++;
				u16Write32DataToEeprom(UPMOVEHOUR_ADDR_L16,UPMOVEHOUR_ADDR_H16,sgActionour.u32UpMoveHour,1);
			}
		}
			
		i32SetPara(PARA_LiftValveCurrent,ActionHourRec.u32DownHour);
		
		u16SecondCntRec = u16SecondCnt;
	}
	ActionFlagRec.u8Data = sgActionFlag.u8Data;
	{
		CanSend163.u8DownTime1 = ((sgActionour.u32DownHour / 36) & 0xFF);
		CanSend163.u8DownTime2 = (((sgActionour.u32DownHour / 36)>>8) & 0xFF);
		CanSend163.u8DownTime3 = (((sgActionour.u32DownHour / 36)>>16) & 0xFF);
		CanSend163.u8DownTime4 = (((sgActionour.u32DownHour / 36)>>24) & 0xFF);
	}
	/*165*/
	{
		memcpy(CanSend165.u8Data,(uint8_t*)(SOFTWARE_VERSION)+ 8,8);
	}
		/*171*/
	{
//		CanSend171.u8FirmCode = ;
		memcpy(CanSend171.u8Data,(uint8_t*)(SOFTWARE_VERSION),8);
	}
	/*167*/
	{
		CanSend167.u8UpdateCountH = (sgActionTimes.u16UpDateTimes >> 8) & 0xFF;
		CanSend167.u8UpdateCountL = (sgActionTimes.u16UpDateTimes >> 0) & 0xFF;
		CanSend167.u8EquipmentSpecifyCodeL = 0xFF;
		CanSend167.u8EquipmentSpecifyCodeH = 0xFF;
		CanSend167.u8EquipmentRecongnizeCode = 0xFF;
		CanSend167.u8VersionCodeBig = (SOFTWARE_VERSION_CODE >> 8) & 0xFF;
		CanSend167.u8VersionCodeSmall = (SOFTWARE_VERSION_CODE >> 0) & 0xFF;
		CanSend167.u8AppVersionReleaseMonth = RELEASE_MONTH;
	}

	/*168*/
	{
		CanSend168.b1MoveForward = sgActLogic.b1ForwardAct;
		CanSend168.b1MoveBackward = sgActLogic.b1BackwardAct;
		CanSend168.b1LiftUp = sgActLogic.b1LiftUpAct;
		CanSend168.b1LiftDown = sgActLogic.b1LiftDownAct;
		CanSend168.b1TurnRight = sgActLogic.b1TurnRight;
		CanSend168.b1TurnLeft = sgActLogic.b1TurnLeft;
		
		if(0 != u16MotorFdb)
			CanSend168.b1MotorEn = 1;
		
		CanSend168.b1Trumpet = ~i32LocalDiGet(SPEAKER_PUMP_R);
		CanSend168.b1Led = ~i32LocalDiGet(BLINK_LED_R);
		CanSend168.b1Beep = ~i32LocalDiGet(BLINK_BEEP_R);
		
		
		CanSend168.u8MotorSpeedL = (u16MotorFdb & 0xFF);
		CanSend168.u8MotorSpeedH = ((u16MotorFdb >> 8) & 0xFF);
		
		if(NoSensor != gUserInfo.u8AngleSensorType)
		{
			CanSend168.u8AngleL = (u16AngleTmp & 0xFF);
			CanSend168.u8AngleH = ((u16AngleTmp>>8) & 0xFF);
		}
		else
		{
			CanSend168.u8AngleL = 0xFF;
			CanSend168.u8AngleH = 0xFF;
		}
		if(NoSensor != gUserInfo.u8PressureSensorType)
		{
			CanSend168.u8PressureL = (i32LocalAiGetValue(PRESSURE_SENSOR_CHANNEL) & 0xFF);
			CanSend168.u8PressureH = ((i32LocalAiGetValue(PRESSURE_SENSOR_CHANNEL)>>8) & 0xFF);
		}
		else
		{
			CanSend168.u8PressureL = 0xFF;
			CanSend168.u8PressureH = 0xFF;			
		}
	}
	/*ID169*/
	{
		CanSend169.b1LowerControlDown = sgSwiInput.b1LowerCtlDown;
		#if 0
			CanSend169.b1PlatformHighestSwitch = ;
			CanSend169.b1PlatformSafeHeightSwitch = ;
		#endif
		CanSend169.b1PitSwitch = sgSwiInput.b1PitSwi;
		CanSend169.b1ControlModeSwitch = sgSwiInput.b1PcuSwi;
		CanSend169.b1LowerControlUp = sgSwiInput.b1LowerCtlUp;
		if(0 == sgSwiInput.b1TiltSwi)
			CanSend169.b1TiltSensor = 1;
		CanSend169.b1SlowSpeedSwitch = sgPcuInfo.PcuKeyInfo.b1Slow;
		CanSend169.b1TrumpetSwitch = sgPcuInfo.PcuKeyInfo.b1Speaker;
		CanSend169.b1EnableSwitch = sgPcuInfo.PcuKeyInfo.b1Enable;
		CanSend169.b1TurnLeftSwitch = sgPcuInfo.PcuKeyInfo.b1TurnLeft;
		CanSend169.b1TurnRightSwitch = sgPcuInfo.PcuKeyInfo.b1TurnRight;
		CanSend169.b1LiftModeSwitch = sgPcuInfo.PcuKeyInfo.b1LiftMode;
		CanSend169.b1MoveModeSwitch = sgPcuInfo.PcuKeyInfo.b1MoveMode;
		CanSend169.u8HandleAnalogL = ((sgPcuInfo.i16HandleValue/32) & 0xFF);
		CanSend169.u8HandleAnalogH = (((sgPcuInfo.i16HandleValue/32) >> 8) & 0xFF);
		CanSend169.b1InitComplete = 1;
		CanSend169.b1CommunicateConnect = 1;
		if(0 == u8ErrCodeGet())
			CanSend169.b1StableRun = 1;
		else
			CanSend169.b1StableRun = 0;
		if((1 == sgActLogic.b1ForwardAct)
		||(1 == sgActLogic.b1BackwardAct))
			CanSend169.b1ReleaseBrake = 1;
		if(1 == sgLimit.b1NoAct)
			CanSend169.b1LockWhileError = 1;
		else
			CanSend169.b1LockWhileError = 0;
		
		CanSend169.b1ECUPowerOn = 1;
	}
	
	{
		CanSend170.u8BatteryVoltageH = (uint8_t)(((i32LocalAiGetValue(AI_B_KSI_CHECK) /100)>>8)&0xFF);
		CanSend170.u8BatteryVoltageL = (uint8_t)(((i32LocalAiGetValue(AI_B_KSI_CHECK) /100))&0xFF);
		CanSend170.u8BatterySocH = 0;
		CanSend170.u8BatterySocL = u8Soc;
		
		{
			CanSend170.b1SystemInitFailed = 0;
			CanSend170.b1SystemCommunityError = 0;
			CanSend170.b1ControllConfigErr = 0;
		}
		
		CanSend170.b1LowerControlSwitchError = i32ErrCodeCheck(UPDOWN_BUTTON_ERR);
		CanSend170.b1PitProteclError = i32ErrCodeCheck(PIT_PROCETION_ERR);
		CanSend170.b1PressureSensorError = i32ErrCodeCheck(PRESSURE_SENSOR_ERR);
		CanSend170.b1AngleSensorErr = i32ErrCodeCheck(ANGLE_SENSOR_ERR);
		CanSend170.b1TurnLeftSwitchError = i32ErrCodeCheck(PLAT_LEFT_BUTTON_ERR);
		CanSend170.b1TurnRightSwitchError = i32ErrCodeCheck(PLAT_RIGHT_BUTTON_ERR);
		CanSend170.b1EnableSwitchError = i32ErrCodeCheck(ENABLE_BUTTON_ERR);
		CanSend170.b1HandleOffCenter = i32ErrCodeCheck(HANDLE_NOT_IN_MIDDLE_POSI_ERR);
		CanSend170.b1BmsLowVoltage = i32ErrCodeCheck(BMS_TOTAL_VOL_LOW1_ERR);
		CanSend170.b1OverLoade = i32ErrCodeCheck(PLAT_OVERLOAD_ERR);
		CanSend170.b1OverTilt = i32ErrCodeCheck(MACHINE_TILT_OVER_SAFETY_ERR);
		CanSend170.b1ForwardValveError = i32ErrCodeCheck(FORWARD_VALVE_ERR);
		CanSend170.b1BackwardValveError = i32ErrCodeCheck(BACKWARD_VALVE_ERR);
		CanSend170.b1LiftUpValveError = i32ErrCodeCheck(LIFT_UP_VALVE_ERR);
		CanSend170.b1LiftDownValveError = i32ErrCodeCheck(LIFT_DOWN_VALVE_ERR);
		CanSend170.b1TurnRightValveError = i32ErrCodeCheck(TURN_RIGHT_VALVE_ERR);
		CanSend170.b1TurnLeftValveError = i32ErrCodeCheck(TURN_LEFT_VALVE_ERR);
	}
	/*172*/
	{
		CanSend172.u8HourCntLL = ((u32HourCnt>>0) & 0xFF);
		CanSend172.u8HourCntLH = ((u32HourCnt>>8) & 0xFF);
		CanSend172.u8HourCntHL = ((u32HourCnt>>16) & 0xFF);
		CanSend172.u8HourCntHH = ((u32HourCnt>>24) & 0xFF);
		CanSend172.OverLoadTimesL = (sgActionTimes.u32OverLoadTimes & 0xFF);
		CanSend172.OverLoadTimesH = ((sgActionTimes.u32OverLoadTimes>>8) & 0xFF);
	}
	{
		#if 1
		{
			CanSend173.u8LiftPressureL= 0xFF;
			CanSend173.u8LiftPressureH= 0xFF;
		}
		#endif


		if(FunctionDisable == gUserInfo.u8AngleSimulationLimit)
		{
			CanSend173.u8EquipmetAngleL = 0xFF;
			CanSend173.u8EquipmetAngleH = 0xFF;	
		}
		else
		{
			CanSend173.u8EquipmetAngleL = ((u16AngleTmp >> 0)& 0xFF);
			CanSend173.u8EquipmetAngleH = ((u16AngleTmp >> 8)& 0xFF);			
		}

		CanSend173.b1KeySwiState = sgSwiInput.b1PcuSwi;
		CanSend173.b1IndoorState = gUserInfo.u8InAndOutFunc;
		if(LIFT_MODE == u8PcuMode)
			CanSend173.b1LiftMode = 1;
		if(MOVE_MODE == u8PcuMode)
			CanSend173.b1MoveMode =1;
		if((MOVE_MODE == u8PcuMode)
		&&((0 != sgLimit.b1Slow)||(0 != sgLimit.b1SpeedAfterLift)))
			CanSend173.b1SlowSpeed = 1;
		CanSend173.u8SpeedAfterLift = gUserInfo.u8DriveSpeedAfterLift;
		CanSend173.u8SteerSpeed = gUserInfo.u8MaxTurnSpeed;
		CanSend173.u8SteerSpeedLimit = gUserInfo.u8TurnPowerLimit;
	}
	{
		CanSend174.u8MachineCodeL = i32GetPara(PARA_CARCODE) & 0xFF;

		CanSend174.u8MachineCodeH = (i32GetPara(PARA_CARCODE) >> 8) & 0xFF;

		if(FunctionDisable == gUserInfo.u8WeighFunc)
			CanSend174.b1WeightMeasure = 1;
		if(NoSensor == gUserInfo.u8AngleSensorSetting)
			CanSend174.b1AngleSensor = 1;
		if(FunctionDisable == gUserInfo.u8AntiPinchFunc)
			CanSend174.b1AntiPinchFunction = 1;
		if(LiBattery != gUserInfo.u8BatteryType)
			CanSend174.b1LiBattery = 1;
		CanSend174.b1DownLimitSwi = ~sgSwiInput.b1DownLimitSwi;
		CanSend174.b1UpLimitSwi = ~sgSwiInput.b1UpLimitSwi;
		CanSend174.b1HorizonSensorSwi = sgSwiInput.b1TiltSwi;
		if(SingleChannelSensor != gUserInfo.u8PressureSensorType)
			CanSend174.b1SingleChannelPressureSensor = 1;
		if(DoubleChannelSensor != gUserInfo.u8PressureSensorType)
			CanSend174.b1DoubleChannelPressureSensor = 1;

			CanSend174.b1HeightBanMove =  ~sgUserSets.b1LiftBanMove;
			CanSend174.b1FunctionalKey = 1;


		if(SingleChannelSensor == gUserInfo.u8PressureSensorType)
		{
			CanSend174.b1SingleChannelPressureSensor = 1;
		}
		else if(DoubleChannelSensor == gUserInfo.u8PressureSensorType)
		{
			CanSend174.b1DoubleChannelPressureSensor = 1;
		}		
		CanSend174.b1PitProtectFunction = gUserInfo.u8PitProtectFunc;
		CanSend174.b1ActionAlarmFunction = gUserInfo.u8ActAlmFunc;
		if(1 == i32LocalDiGet(HIGH_SPEED_PUMP_R))
			CanSend174.b1HighSpeedPump = 1;
		if(OnOffValve == gUserInfo.u8LowerPumpType)
			CanSend174.b1DownValveType =1;
		CanSend174.u8LiftSpeed = gUserInfo.u8LiftSpeed;
		CanSend174.u8DownSpeed = gUserInfo.u8LowerSpeed;
		CanSend174.u8NormalSpeed = gUserInfo.u8FastDriveSpeed;
		CanSend174.u8SlowSpeed = gUserInfo.u8SlowDriveSpeed;
	}
	{
		CanSend175.u8DriverTypeL = 1;
		CanSend175.u8LeftMotorSpeedL = 0xFF;
		CanSend175.u8LeftMotorSpeedH = 0xFF;
		CanSend175.u8RightMotorSpeedL = 0xFF;
		CanSend175.u8RightMotorSpeedH = 0xFF;
		CanSend175.u8LeftMotorTempL = 0xFF;
		CanSend175.u8LeftMotorTempH = 0xFF;
	}
	{
		CanSend176.u8RightMotorTempL = 0xFF;
		CanSend176.u8RightMotorTempH = 0xFF;
		CanSend176.u8MoveMotorTempL = 0xFF;
		CanSend176.u8MoveMotorTempH = 0xFF;
		CanSend176.u8PumpMotorTempL = u8MotorTmp + 20;
		CanSend176.u8PumpMotorTempH = (((u8MotorTmp + 20) >> 8) & 0xFF);
		CanSend176.u8EnvironmentTempL = 0xFF;
		CanSend176.u8EnvironmentTempH = 0xFF;
	}
	{
		CanSend177.u8PumpCurrentL = (u16Current & 0xFF);
		CanSend177.u8PumpCurrentH = ((u16Current >> 8) & 0xFF);
		CanSend177.u8LeftMotorCurrentL = 0xFF;
		CanSend177.u8LeftMotorCurrentH = 0xFF;
		CanSend177.u8RightMotorCurrentL = 0xFF;
		CanSend177.u8RightMotorCurrentH = 0xFF;
	}
	{
		CanSend178.u8LiftTimes1 = (sgActionTimes.u32LiftTimes & 0xFF);
		CanSend178.u8LiftTimes2 = ((sgActionTimes.u32LiftTimes >> 8) & 0xFF);
		CanSend178.u8LiftTimes3 = ((sgActionTimes.u32LiftTimes >> 16) & 0xFF);
		CanSend178.u8LiftTimes4 = ((sgActionTimes.u32LiftTimes >> 24) & 0xFF);
		CanSend178.u8SteerTimes1 = ((sgActionTimes.u32SteerTimes >> 0) & 0xFF);
		CanSend178.u8SteerTimes2 = ((sgActionTimes.u32SteerTimes >> 8) & 0xFF);
		CanSend178.u8SteerTimes3 = ((sgActionTimes.u32SteerTimes >> 16) & 0xFF);
		CanSend178.u8SteerTimes4 = ((sgActionTimes.u32SteerTimes >> 24) & 0xFF);
	}
	{
		CanSend179.u8MoveTimes1 = ((sgActionTimes.u32MoveTimes >> 0) & 0xFF);
		CanSend179.u8MoveTimes2 = ((sgActionTimes.u32MoveTimes >> 8) & 0xFF);
		CanSend179.u8MoveTimes3 = ((sgActionTimes.u32MoveTimes >> 16) & 0xFF);
		CanSend179.u8MoveTimes4 = ((sgActionTimes.u32MoveTimes >> 24) & 0xFF);
		CanSend179.u8OverLoadTimes1 = ((sgActionTimes.u32OverLoadTimes >> 0) & 0xFF);
		CanSend179.u8OverLoadTImes2 = ((sgActionTimes.u32OverLoadTimes >> 8) & 0xFF);
		CanSend179.u8OverLoadTimes3 = ((sgActionTimes.u32OverLoadTimes >> 16) & 0xFF);
		CanSend179.u8OverLoadTimes4 = ((sgActionTimes.u32OverLoadTimes >> 24) & 0xFF);
	}
	{
		memcpy(CanSend17A.u8Data,(uint8_t*)(ECU_SERIALNUMBER_L32),8);
	}
	{
		memcpy(CanSend17B.u8Data,(uint8_t*)(ECU_SERIALNUMBER_L32)+ 8,2);
	}
	{
		memcpy(CanSend17C.u8Data,(uint8_t*)(PCU_SERIALNUMBER_L32),8);
	}
	{
		memcpy(CanSend17D.u8Data,(uint8_t*)(PCU_SERIALNUMBER_L32 ) + 8,2);
	}
	{
		CanSend17E.u8BatteryVoltageL = (((((i32LocalAiGetValue(AI_B_KSI_CHECK))/100) + 200) >> 0) & 0xFF);
		CanSend17E.u8BatteryVoltageH = (((((i32LocalAiGetValue(AI_B_KSI_CHECK))/100) + 200) >> 8) & 0xFF);
	}
	{
		CanSend050.u8LiftTime1 = (((sgActionour.u32LifHour / 36 )>> 0 )& 0xFF) ;
		CanSend050.u8LiftTime2 = (((sgActionour.u32LifHour / 36 )>> 8 )& 0xFF) ;
		CanSend050.u8LiftTime3 = (((sgActionour.u32LifHour / 36 )>> 16 )& 0xFF) ;
		CanSend050.u8LiftTime4 = (((sgActionour.u32LifHour / 36 )>> 24 )& 0xFF) ;
		CanSend050.u8SteerTime1 = (((sgActionour.u32SteerHour / 36) >> 0 )& 0xFF) ;
		CanSend050.u8SteerTime2 = (((sgActionour.u32SteerHour / 36) >> 8 )& 0xFF) ;
		CanSend050.u8SteerTime3 = (((sgActionour.u32SteerHour / 36) >> 16 )& 0xFF) ;
		CanSend050.u8SteerTime4 = (((sgActionour.u32SteerHour / 36) >> 24 )& 0xFF) ;
	}
	{
		CanSend055.u8LowerMoveTime1 = (((sgActionour.u32LowerMoveHour / 36) >> 0 )& 0xFF) ;
		CanSend055.u8LowerMoveTime2 = (((sgActionour.u32LowerMoveHour / 36) >> 8 )& 0xFF) ;
		CanSend055.u8LowerMoveTime3 = (((sgActionour.u32LowerMoveHour / 36) >> 16 )& 0xFF) ;
		CanSend055.u8LowerMoveTime4 = (((sgActionour.u32LowerMoveHour / 36) >> 24 )& 0xFF) ;
		CanSend055.u8UpperMoveTime1 = (((sgActionour.u32UpMoveHour / 36) >> 0 )& 0xFF) ;
		CanSend055.u8UpperMoveTime2 = (((sgActionour.u32UpMoveHour / 36) >> 8 )& 0xFF) ;
		CanSend055.u8UpperMoveTime3 = (((sgActionour.u32UpMoveHour / 36) >> 16 )& 0xFF) ;
		CanSend055.u8UpperMoveTime4 = (((sgActionour.u32UpMoveHour / 36) >> 24 )& 0xFF) ;
	}
	{
		CanSend190.u8FactoryCode = 4;
		//CanSend190.u8ParaSetState = ;
		if(FunctionDisable == gUserInfo.u8LiftReverseFunc)
			CanSend190.b1HandleReverseFunction = 1;
		
		if(FunctionDisable == gUserInfo.u8FourPointWeightFunc)
			CanSend190.b1FouPointWeightFunction = 1;
		
		if(0 == sgHeartBeatLockDynamic.b1HeartBeatQuery)
			CanSend190.b1HeartBeatCheckFunction = 1;
		
		if((1 == sgActLogic.b1ForwardAct)
		||(1 == sgActLogic.b1BackwardAct))
			CanSend190.b1BrakeRelease = 1;
		
			CanSend190.b1AngleSensorType = 0;
		
			CanSend190.b1DriverType = 1;
	}
	{
		if(0 != u8ErrCodeGet())
		{
			CanSend25D.u8ErrorNum = 1;
			CanSend25D.u8ErrorState = 1;
			if(50 >= u8ErrCodeGet())
			CanSend25D.u8ErrorLocation = 0x1E;	
		}
	}
	{
		CanSend55E.u8Soc = u8Soc;
		if(0 == sgHeartBeatLockDynamic.b1HeartBeatQuery)
			CanSend55E.b1HeartState = 1;
		if((0 == i32ErrCodeCheck(PLATFORM_LEVEL1_LOCK_ERR))
			&&(0 == i32ErrCodeCheck(PLATFORM_LEVEL2_LOCK_ERR))
			&&(0 == i32ErrCodeCheck(HEART_BEAT_LOCK_ERR))
			)
			CanSend55E.b1LockState = 1;
		if(1 == sgHeartBeatLockDynamic.b1TempUnlock)
		{
			CanSend55E.u8ControlState = 93;
		}
		else if(1 == i32ErrCodeCheck(HEART_LOCK_ALARMING))
		{
			CanSend55E.u8ControlState = 91;
		}
		else if(1 == i32ErrCodeCheck(PLATE_LOCK_LEVEL1_ALARMING))
		{
			CanSend55E.u8ControlState = 2;
		}
		else if(1 == i32ErrCodeCheck(PLATE_LOCK_LEVEL2_ALARMING))
		{
			CanSend55E.u8ControlState = 4;
		}
		else if(1 == i32ErrCodeCheck(HEART_BEAT_LOCK_ERR))
		{
			CanSend55E.u8ControlState = 90;
		}
		else if(0 == sgHeartBeatLockDynamic.b1HeartBeatQuery)
		{
			CanSend55E.u8ControlState = 92;
		}
		else if(1 == i32ErrCodeCheck(PLATFORM_LEVEL1_LOCK_ERR))
		{
			CanSend55E.u8ControlState = 1;
		}
		else if(1 == i32ErrCodeCheck(PLATFORM_LEVEL2_LOCK_ERR))
		{
			CanSend55E.u8ControlState = 3;
		}
		else if(1 == sgHeartBeatLockDynamic.b1HeartBeatQuery)
		{
			CanSend55E.u8ControlState = 0;
		}
		else
		{
			CanSend55E.u8ControlState = 0xFF;
		}
	}
	ActRecord.u8Data = sgActLogic.u8Data;
	if(u8SendCnt < 100)
		u8SendCnt++;
	
	switch(u8SendCnt)
	{
		case 22:
			Canframe.u32ID = 0x165;
			Canframe.u16DataLength = 8;
			Canframe.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				Canframe.u8Data[i] = CanSend165.u8Data[i];
			}
			break;			
		case 23:
			Canframe.u32ID = 0x17F;
			Canframe.u16DataLength = 8;
			Canframe.u8Rtr = 0;

			Canframe.u8Data[0] = 0;
			Canframe.u8Data[1] = 0xE8;
			break;				
		case 24:
			Canframe.u32ID = 0x190;
			Canframe.u16DataLength = 8;
			Canframe.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				Canframe.u8Data[i] = CanSend190.u8Data[i];
			}
			break;		
		case 25:
			Canframe.u32ID = 0x55E;
			Canframe.u16DataLength = 8;
			Canframe.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				Canframe.u8Data[i] = CanSend55E.u8Data[i];
			}
			break;	
		case 1:
			Canframe.u32ID = 0x163;
			Canframe.u16DataLength = 8;
			Canframe.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				Canframe.u8Data[i] = CanSend163.u8Data[i];
			}
			break;
		case 2:
			Canframe.u32ID = 0x167;
			Canframe.u16DataLength = 8;
			Canframe.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				Canframe.u8Data[i] = CanSend167.u8Data[i];
			}
			break;
		case 3:
			Canframe.u32ID = 0x168;
			Canframe.u16DataLength = 8;
			Canframe.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				Canframe.u8Data[i] = CanSend168.u8Data[i];
			}
			break;
		case 4:
			Canframe.u32ID = 0x169;
			Canframe.u16DataLength = 8;
			Canframe.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				Canframe.u8Data[i] = CanSend169.u8Data[i];
			}
			break;
		case 5:
			Canframe.u32ID = 0x170;
			Canframe.u16DataLength = 8;
			Canframe.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				Canframe.u8Data[i] = CanSend170.u8Data[i];
			}
			break;
		case 6:
		
			Canframe.u32ID = 0x171;
			Canframe.u16DataLength = 8;
			Canframe.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				Canframe.u8Data[i] = CanSend171.u8Data[i];
			}
			
			break;
		case 7: 	
			Canframe.u32ID = 0x172;
			Canframe.u16DataLength = 8;
			Canframe.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				Canframe.u8Data[i] = CanSend172.u8Data[i];
			}
			break;
		case 8:
			Canframe.u32ID = 0x173;
			Canframe.u16DataLength = 8;
			Canframe.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				Canframe.u8Data[i] = CanSend173.u8Data[i];
			}
			break;
		case 9:
			Canframe.u32ID = 0x174;
			Canframe.u16DataLength = 8;
			Canframe.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				Canframe.u8Data[i] = CanSend174.u8Data[i];
			}
			break;
		case 10:
			Canframe.u32ID = 0x175;
			Canframe.u16DataLength = 8;
			Canframe.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				Canframe.u8Data[i] = CanSend175.u8Data[i];
			}
			break;
		case 11:
			Canframe.u32ID = 0x176;
			Canframe.u16DataLength = 8;
			Canframe.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				Canframe.u8Data[i] = CanSend176.u8Data[i];
			}
			break;
		case 12:
			Canframe.u32ID = 0x177;
			Canframe.u16DataLength = 8;
			Canframe.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				Canframe.u8Data[i] = CanSend177.u8Data[i];
			}
			break;
		case 13:
			Canframe.u32ID = 0x178;
			Canframe.u16DataLength = 8;
			Canframe.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				Canframe.u8Data[i] = CanSend178.u8Data[i];
			}
			break;
		case 14:
			Canframe.u32ID = 0x179;
			Canframe.u16DataLength = 8;
			Canframe.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				Canframe.u8Data[i] = CanSend179.u8Data[i];
			}
			break;
		case 15:
			Canframe.u32ID = 0x17A;
			Canframe.u16DataLength = 8;
			Canframe.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				Canframe.u8Data[i] = CanSend17A.u8Data[i];
			}
			break;
		case 16:
			Canframe.u32ID = 0x17B;
			Canframe.u16DataLength = 8;
			Canframe.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				Canframe.u8Data[i] = CanSend17B.u8Data[i];
			}
			break;
		case 17:
			Canframe.u32ID = 0x17C;
			Canframe.u16DataLength = 8;
			Canframe.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				Canframe.u8Data[i] = CanSend17C.u8Data[i];
			}
			break;
		case 18:
			Canframe.u32ID = 0x17D;
			Canframe.u16DataLength = 8;
			Canframe.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				Canframe.u8Data[i] = CanSend17D.u8Data[i];
			}
			break;
		case 19:
			Canframe.u32ID = 0x17E;
			Canframe.u16DataLength = 8;
			Canframe.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				Canframe.u8Data[i] = CanSend17E.u8Data[i];
			}
			break;
		case 20:
			Canframe.u32ID = 0x050;
			Canframe.u16DataLength = 8;
			Canframe.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				Canframe.u8Data[i] = CanSend050.u8Data[i];
			}
			break;
		case 21:
			Canframe.u32ID = 0x055;
			Canframe.u16DataLength = 8;
			Canframe.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				Canframe.u8Data[i] = CanSend055.u8Data[i];
			}
			break;
		default:
			Canframe.u32ID = 0;
			break;
	}
	if(0 != Canframe.u32ID)
		i32CanWrite(Can0, &Canframe);

}

static void HMISend()
{
	tCanFrame Canframe;
	xHMISend169 HMI169;
	xHMISend170 HMI170;
	xHMISend172 HMI172;
	xHMISendEC5521 HMIEC5521;
	xHMISendEE5521 HMIEE5521;
	uint8_t u8ErrCode = 0;
	uint8_t i = 0;
	uint16_t u16Carcode;
	
	static uint8_t u8HMICnt = 0;
	u8HMICnt++;
	memset(&HMI169,0,sizeof (HMI169));
	memset(&HMI170,0,sizeof (HMI170));
	memset(&HMI172,0,sizeof (HMI172));
	memset(&HMIEC5521,0,sizeof (HMIEC5521));
	memset(&HMIEE5521,0,sizeof (HMIEE5521));
	
	HMI169.b1UpControl = sgSwiInput.b1PcuSwi;
	HMI170.u8SOC = u8Soc;
	HMI172.u8HourCntHH = (u32HourCnt>> 24)&0xFF;
	HMI172.u8HourCntHL = (u32HourCnt>> 16)&0xFF;
	HMI172.u8HourCntLH = (u32HourCnt>> 8)&0xFF;
	HMI172.u8HourCntLL = (u32HourCnt)&0xFF;
	u8ErrCode= u8ErrCodeGet();
	if((0 != u8ErrCode)&&(u8ErrCode<= 50))//�������ϣ�С��50
	{
		HMIEE5521.u8ErrMode = 0xE1;
		HMIEE5521.u8ErrCode = u8ErrCode;
	}
	else if((0 != u8ErrCode)&&(u8ErrCode > 50))//����50��ECU����
	{
		HMIEE5521.u8ErrCode = u8ErrCodeGetTrans();
	}                 
	u16EepromRead(PARA_CARCODE,&u16Carcode,1);
	HMIEE5521.u8VersionCodeHH = (VERSION_CODE_HMI >> 24)&0xff;   //���Ͱ汾��
	HMIEE5521.u8VersionCodeHL = (VERSION_CODE_HMI >> 16)&0xff;
	HMIEE5521.u8VersionCodeLH = (VERSION_CODE_HMI >> 8)&0xff;
	HMIEE5521.u8VersionCodeLL = (VERSION_CODE_HMI)&0xff;
	
	HMIEE5521.u8CarType = u16Carcode;
	HMIEC5521.b1OverLoadErr = i32ErrCodeCheck(PLAT_OVERLOAD_ERR);
	HMIEC5521.b1TileErr = i32ErrCodeCheck(MACHINE_TILT_OVER_SAFETY_ERR);
	switch (u8HMICnt)
	{
		case 1:
			Canframe.u32ID = 0x169;
			Canframe.u16DataLength = 8;
			Canframe.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				Canframe.u8Data[i] = HMI169.u8Data[i];
			}
			break;
		case 2:
			Canframe.u32ID = 0x170;
			Canframe.u16DataLength = 8;
			Canframe.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				Canframe.u8Data[i] = HMI170.u8Data[i];
			}
			break;
		case 3:
			Canframe.u32ID = 0x172;
			Canframe.u16DataLength = 8;
			Canframe.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				Canframe.u8Data[i] = HMI172.u8Data[i];
			}
			break;
		case 4:
			Canframe.u32ID = 0x18EE5521;
			Canframe.u16DataLength = 8;
			Canframe.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				Canframe.u8Data[i] = HMIEE5521.u8Data[i];
			}
			break;
		case 5:
			Canframe.u32ID = 0x18EC5521;
			Canframe.u16DataLength = 8;
			Canframe.u8Rtr = 0;
			for (i=0; i<8; i++)                                 
			{
				Canframe.u8Data[i] = HMIEC5521.u8Data[i];
			}
			break;
		case 6:
			Canframe.u32ID = 0;
			u8HMICnt = 0;
			break;
		default:
			Canframe.u32ID = 0;
			u8HMICnt = 0;		
			break;
	}
	if(0 != Canframe.u32ID)
		i32CanWrite(Can0, &Canframe);
}

<<<<<<< .mine
/*******************
���ݳ��͸Ĳ���
**********************/
static	uint16_t Para0507[][2] = {
	{PARA_FastDriveSpeed , 63},//0
	{PARA_SlowDriveSpeed , 99},//1
	{PARA_DriveSpeedAfterLift , 36},//2
	{PARA_LiftSpeed , 40},//3
	{PARA_MaxTurnSpeed , 40},//4
	{PARA_TurnPowerLimit , 30},//5
	{PAPA_DeadZoneAdjust , 10},//6
	{PARA_BrakeFastDrive , 5},//7
	{PARA_BrakeSlowDrive , 2},//8
	{PARA_BrakeDriveAfterLift , 5},//9
	{PARA_BrakeLift , 7},//10
	{PARA_BrakeLower , 5},//11
	{PARA_BrakeTurn , 2},//12
	{PARA_BrakeAntiPinch , 82},//13
	{PARA_LowerSpeed , 99},//16
	{PARA_OverLoadStabilityDelay , 1},//17
	{PARA_DynamicOverLoadPercent , 13},//18
	{PARA_StaticOverLoadPercent , 31},//19
	{PARA_MaxDifferencePercent , 35},//20
	{PARA_DriveMotorEncoder , 1},//21			��δʹ�ò���
	{PARA_MotorHighSpeedDeceRate , 5},//22		  �� δʹ�ò���
	{PARA_MotorLowSpeedDeceRate , 15},//23			��δʹ�ò���
	{PARA_VoiceAlarmVolume , 1},//24
	{PARA_CurveFastDrive , 15},//25
	{PARA_CurveSlowDrive , 2},//26
	{PARA_CurveDriveAfterLift , 2},//27
	{PARA_CurveLift , 5},//28
	{PARA_CurveLower , 2},//29
	{PARA_CurveTurn , 2},//32
	{PARA_AccAndDecFastDrive , 2},//33
	{PARA_AccAndDecSlowDrive , 2},//34
	{PARA_AccAndDecAfterLift , 2},//35
	{PARA_AccAndDecLift , 2},//36
	{PARA_AccAndDecLower , 8},//37
	{PARA_AccAndDecTurn , 2},//38
	{PARA_AccAndDecAntiPinch , 2},//39
	{PARA_PumpMotorEncoder , 21},//40
	{PARA_VehicleType , 50},//41
	{PARA_VehcileHeight , 0},//42
	{PARA_PressureSensorType , 1},//43
	{PARA_PitProtectFunc , 1},//44
	{PARA_AntiPinchFunc , 1},//45
	{PARA_ActAlmFunc , 1},//48
	{PARA_WeighFunc , 0},//49
	{PARA_ParallelValveReverseFunc , 1},//50
	{PARA_LowBatAlmFunc , 1},//51
	{PARA_LowVolAlmTime , 12},//52
	{PARA_LowVolShutDownTime , 30},//53
	{PARA_UpperCtlButSleep , 10},//54
	{PARA_LanguageType , 2},//55
	{PARA_BatteryType , 2},//56
	{PARA_SpeakerSync , 1},//57
	{PARA_LowerPumpType , 0},//58
	{PARA_PressureType , 1},//59
	{PARA_AngleSimulationLimit , 1},//60
	{PARA_LiftReverseFunc , 0},//61
	{PARA_FourPointWeightFunc , 0},//64
	{PARA_DriverType , 6},//65
	{PARA_PasswordLock , 1},//66
	{PARA_InAndOutFunc , 0},//67
	{PARA_HeartBeatQueryFunc , 1},//68
	{PARA_LowBatteryMode , 0},//69
	{PARA_AngleSensorSetting , 1},//70
	{PARA_TiltSwitchSetting , 1},//71
	{PARA_AngleSensorType , 0},//72
	{PARA_AnaLogLimitDetSwitch , 0},//73
	{PARA_IsNoLoadCalibration , 1},//74
	{PARA_IsOverLoadCalibration , 1},//75
	{PARA_SetDescentHeightValue , 1},//76
	{PARA_ReleaseBrake , 1},//77
	{PARA_SetOutHeight , 2},//80
	{PARA_AngleSimulationUpLimit , 2043},//81 �궨��
	{PARA_AngleSimulationDownLimit , 882},//82  �궨��
	{PARA_PumpDriveCurrentLimitRatio0 , 0},//83
	{PARA_PumpSpdAccRatio0 , 0},//84
	{PARA_PropDKp0 , 8192},//85
	{PARA_PropDKi0 , 4096},//86
	{PARA_PropDMaxCurrent0 , 5},//87
	{PARA_PropDMinCurrent0 , 5},//88
	{PARA_PropDAccPeriod0 , 10},//89
	{PARA_PropDDitherPeriod0 , 16},//90
	{PARA_PropDDitherRatio0 , 5},//91
	{PARA_PropValveResistance0 , 20},//92
	{PARA_PumpDriveCurrentLimitRatio1 , 0},//96
	{PARA_PumpSpdAccRatio1 , 0},//97
	{PARA_PropDKp1 , 8192},//98
	{PARA_PropDKi1 , 4096},//99
	{PARA_PropDMaxCurrent1 , 1000},//100
	{PARA_PropDMinCurrent1 , 460},//101
	{PARA_PropDAccPeriod1 , 20},//102
	{PARA_PropDDitherPeriod1 , 40},//103
	{PARA_PropDDitherRatio1 , 5},//104
	{PARA_PropValveResistance1 , 20},//105
	{PARA_CanBaudRate , 500},//106
	{PARA_EmptyPressure , 3000},//107
	{PARA_FullPressure , 4000},//108
	{PARA_DriverFlag , 2046},//109
	{PARA_MinAngle , 300},//112
	{PARA_MaxAngle , 2010},//113
	{PARA_BatSocPalyBack , 0},//114
	{PARA_ValveType , 451},//115
	{PARA_AnticollisionFunc , 1},//116
	{PARA_ValueOpenLoopCurrent , 800},//117
	{PARA_ValueOpenPercentage , 80},//118
	{PARA_CanOpenNodeId ,44},//119
	{PARA_AngleValue0 , 0},//120
	{PARA_AngleValue1 , 0},//121
	{PARA_AngleValue2 , 0},//122
	{PARA_AngleValue3 , 0},//123
//	{PARA_AngleValue4 , },//124 0507��������
	{PARA_AngleValue5 , 127},//125 �ֱ�ǰ
	{PARA_AngleValue6 , 0},//126
	{PARA_AngleValue7 , 129},//127 �ֱ���
	{PARA_Driver1Vol , 25680},//128
	{PARA_Driver2Vol , 25680},//129
	{PARA_Driver3Vol , 25680},//130
	{PARA_Driver4Vol , 25680},//131
	{PARA_Driver5Vol , 25680},//132
	{PARA_Driver6Vol , 25680},//133
	{PARA_Driver7Vol , 25680},//134
	{PARA_Driver8Vol , 25680},//135
	{PARA_Driver9Vol , 25680},//136
	{PARA_Driver10Vol, 25680},//137
	{PARA_LogLevel, 0},//138
	{PARA_LogModel, 0},//139
	{PARA_MotorMaxSpd, 3000},//140 �����жϵķ����ٶ���ֵ
	{PARA_Analog3DeadZoneMinVal, 2},//141
	{PARA_Analog3DeadZoneMaxVal, 30},//142
	{PARA_Analog3MidVal, 3},//143
	{PARA_ThrottleType, 0},//144
	{PARA_ThrottleFDeadZoneMinVal, 10},//145 ����ת��ʱ�ط���ֵ
	{PARA_ThrottleFDeadZoneMaxVal, 3},//146 �����ط���ֵ
	{PARA_ThrottleFMidVal, 20},//147 ǰ�������л�
	{PARA_ThrottleBDeadZoneMinVal, 0},//148 ��������
	{PARA_ThrottleBDeadZoneMaxVal, 1},//149 �µ���ʱ
	{PARA_ThrottleBMidVal, 15},//150  ˮƽ�ط�
	{PARA_BrakeType, 0},//151
	{PARA_BrakeFDeadZoneMinVal, 0},//152
	{PARA_BrakeFDeadZoneMaxVal, 0},//153
	{PARA_BrakeFMidVal, 0},//154
	{PARA_BrakeBDeadZoneMinVal, 0},//155
	{PARA_BrakeBDeadZoneMaxVal, 1},//156
	{PARA_BrakeBMidVal, 2},//157
//	{PARA_Gear1Spd, 0},//160
//	{PARA_Gear2Spd, 0},//161
//	{PARA_Gear3Spd, 0},//162
//	{PARA_Gear4Spd, 0},//163
	{PARA_RatioOfTransmission, 0},//164
	{PARA_MaintenancePeriod, 0},//165
//	{PARA_RemotePara, 2},//166
//	{PARA_PumpMotorGear1, 0},//167
//	{PARA_PumpMotorGear2, 0},//168
//	{PARA_TurnWithDecStartAngle, 0},//169
//	{PARA_TurnWithDecEndAngle, 0},//170
//	{PARA_AngleWithStartSpdPer, 0},//171
//	{PARA_AngleWithEndSpdPer, 0},//172
	{PARA_HourCountPowerOn, 0b1011},//173 	ǰ��bit 00�룬01�֣�10 6�֣�11Сʱ��bit1��������ʱ1���ϵ��ʱ0��bit0������Сʱ�ƣ�
};
#if 0
static uint16_t Para0507Li[][2] = {
	{PAPA_DeadZoneAdjust , 10},//6
	{PARA_BrakeSlowDrive , 2},//8
	{PARA_BrakeDriveAfterLift , 5},//9
	{PARA_BrakeTurn , 2},//12
	{PARA_BrakeAntiPinch , 75},//13
	{PARA_VoiceAlarmVolume , 1},//24
	{PARA_CurveFastDrive , 15},//25
	{PARA_CurveSlowDrive , 2},//26
	{PARA_CurveDriveAfterLift , 2},//27
	{PARA_CurveLift , 5},//28
	{PARA_CurveLower , 2},//29
	{PARA_CurveTurn , 2},//32
	{PARA_AccAndDecSlowDrive , 2},//34
	{PARA_AccAndDecAfterLift , 2},//35
	{PARA_AccAndDecLower , 8},//37
	{PARA_AccAndDecTurn , 2},//38
	{PARA_VehicleType , 50},//41
	{PARA_PressureSensorType , 1},//43
	{PARA_PitProtectFunc , 1},//44
	{PARA_ActAlmFunc , 1},//48
	{PARA_WeighFunc , 0},//49
	{PARA_ParallelValveReverseFunc , 1},//50
	{PARA_BatteryType , 0},//56
	{PARA_SpeakerSync , 1},//57
	{PARA_LowerPumpType , 1},//58
	{PARA_PressureType , 0},//59
	{PARA_AngleSimulationLimit , 1},//60
	{PARA_LiftReverseFunc , 0},//61
	{PARA_FourPointWeightFunc , 0},//64
	{PARA_InAndOutFunc , 0},//67
	{PARA_AngleSensorSetting , 1},//70
	{PARA_TiltSwitchSetting , 1},//71
	{PARA_AngleSensorType , 0},//72
	{PARA_SetOutHeight , 0},//80
	{PARA_PumpDriveCurrentLimitRatio0 , 0},//83
	{PARA_PumpSpdAccRatio0 , 0},//84
	{PARA_PropDKp0 , 8192},//85
	{PARA_PropDKi0 , 4096},//86
	{PARA_PropDMaxCurrent0 , 5},//87
	{PARA_PropDMinCurrent0 , 5},//88
	{PARA_PropDAccPeriod0 , 10},//89
	{PARA_PropDDitherPeriod0 , 40},//90
	{PARA_PropDDitherRatio0 , 5},//91
	{PARA_PropValveResistance0 , 20},//92
	{PARA_PumpDriveCurrentLimitRatio1 , 0},//96
	{PARA_PumpSpdAccRatio1 , 0},//97
	{PARA_PropDKp1 , 8192},//98
	{PARA_PropDKi1 , 4096},//99
	{PARA_PropDMaxCurrent1 , 1000},//100
	{PARA_PropDMinCurrent1 , 400},//101
	{PARA_PropDAccPeriod1 , 10},//102
	{PARA_PropDDitherPeriod1 , 40},//103
	{PARA_PropDDitherRatio1 , 5},//104
	{PARA_PropValveResistance1 , 20},//105
	{PARA_CanBaudRate , 250},//106
	{PARA_DriverFlag , 2046},//109
	{PARA_AnticollisionFunc , 1},//116
	{PARA_AngleValue5 , 127},//125 �ֱ�ǰ
	{PARA_AngleValue6 , 0},//126
	{PARA_AngleValue7 , 129},//127 �ֱ���
	{PARA_Driver1Vol , 25680},//128
	{PARA_Driver2Vol , 25680},//129
	{PARA_Driver3Vol , 25680},//130
	{PARA_Driver4Vol , 25680},//131
	{PARA_Driver5Vol , 25680},//132
	{PARA_Driver6Vol , 25680},//133
	{PARA_Driver7Vol , 25680},//134
	{PARA_Driver8Vol , 25680},//135
	{PARA_Driver9Vol , 25680},//136
	{PARA_Driver10Vol , 25680},//137
	{PARA_MotorMaxSpd, 2700},//140 �����жϵķ����ٶ���ֵ
	{PARA_Analog3DeadZoneMinVal, 5},//141
	{PARA_Analog3DeadZoneMaxVal, 100},//142
	{PARA_Analog3MidVal, 0},//143
	{PARA_ThrottleType, 0},//144
	{PARA_ThrottleFDeadZoneMinVal, 30},//145 ����ת��ʱ�ط���ֵ
	{PARA_ThrottleFDeadZoneMaxVal, 10},//146 �����ط���ֵ
	{PARA_ThrottleFMidVal, 20},//147 ǰ�������л�
	{PARA_ThrottleBDeadZoneMinVal, 0},//148 ��������
	{PARA_ThrottleBDeadZoneMaxVal, 1},//149 �µ���ʱ
	{PARA_ThrottleBMidVal, 50},//150  ˮƽ�ط�
	{PARA_BrakeFDeadZoneMinVal, 0},//152
	{PARA_BrakeFDeadZoneMaxVal, 0},//153
	{PARA_BrakeFMidVal, 0},//154
	{PARA_BrakeBDeadZoneMinVal, 0},//155
	{PARA_BrakeBDeadZoneMaxVal, 1},//156
	{PARA_BrakeBMidVal, 2},//157
	{PARA_RatioOfTransmission, 0},//164
	{PARA_MaintenancePeriod, 0},//165
	{PARA_HourCountPowerOn, 0b1011},//173 	ǰ��bit 00�룬01�֣�10 6�֣�11Сʱ��bit1��������ʱ1���ϵ��ʱ0��bit0������Сʱ�ƣ�
};
#endif
static uint16_t Para1212Dif[][2] = {
	{PARA_FastDriveSpeed , 90},//0
	{PARA_SlowDriveSpeed , 90},//1
	{PARA_DriveSpeedAfterLift , 30},//2
	{PARA_LiftSpeed , 80},//3
	{PARA_MaxTurnSpeed , 60},//4
	{PARA_TurnPowerLimit , 30},//5
	{PAPA_DeadZoneAdjust , 10},//6
	{PARA_BrakeFastDrive , 1},//7
	{PARA_BrakeSlowDrive , 1},//8
	{PARA_BrakeDriveAfterLift , 5},//9
	{PARA_BrakeLift , 12},//10
	{PARA_BrakeLower , 15},//11
	{PARA_BrakeTurn , 2},//12
	{PARA_BrakeAntiPinch , 85},//13
	{PARA_LowerSpeed , 90},//16
	{PARA_OverLoadStabilityDelay , 1},//17
	{PARA_DynamicOverLoadPercent , 13},//18
	{PARA_StaticOverLoadPercent , 31},//19
	{PARA_MaxDifferencePercent , 35},//20
	{PARA_DriveMotorEncoder , 1},//21			��δʹ�ò���
	{PARA_MotorHighSpeedDeceRate , 2},//22		  �� δʹ�ò���
	{PARA_MotorLowSpeedDeceRate , 2},//23			��δʹ�ò���
	{PARA_VoiceAlarmVolume , 1},//24
	{PARA_CurveFastDrive , 10},//25
	{PARA_CurveSlowDrive , 2},//26
	{PARA_CurveDriveAfterLift , 2},//27
	{PARA_CurveLift , 5},//28
	{PARA_CurveLower , 5},//29
	{PARA_CurveTurn , 2},//32
	{PARA_AccAndDecFastDrive , 2},//33
	{PARA_AccAndDecSlowDrive , 2},//34
	{PARA_AccAndDecAfterLift , 2},//35
	{PARA_AccAndDecLift , 4},//36
	{PARA_AccAndDecLower , 4},//37
	{PARA_AccAndDecTurn , 2},//38
	{PARA_AccAndDecAntiPinch , 2},//39
	{PARA_PumpMotorEncoder , 21},//40
	{PARA_VehicleType , 50},//41
	{PARA_VehcileHeight , 0},//42
	{PARA_PressureSensorType , 1},//43
	{PARA_PitProtectFunc , 1},//44
	{PARA_AntiPinchFunc , 1},//45
	{PARA_ActAlmFunc , 1},//48
	{PARA_WeighFunc , 0},//49
	{PARA_ParallelValveReverseFunc , 1},//50
	{PARA_LowBatAlmFunc , 1},//51
	{PARA_LowVolAlmTime , 12},//52
	{PARA_LowVolShutDownTime , 30},//53
	{PARA_UpperCtlButSleep , 10},//54
	{PARA_LanguageType , 2},//55
	{PARA_BatteryType , 2},//56
	{PARA_SpeakerSync , 1},//57
	{PARA_LowerPumpType , 0},//58
	{PARA_PressureType , 0},//59
	{PARA_AngleSimulationLimit , 0},//60
	{PARA_LiftReverseFunc , 0},//61
	{PARA_FourPointWeightFunc , 0},//64
	{PARA_DriverType , 6},//65
	{PARA_PasswordLock , 1},//66
	{PARA_InAndOutFunc , 0},//67
	{PARA_HeartBeatQueryFunc , 3},//68
	{PARA_LowBatteryMode , 0},//69
	{PARA_AngleSensorSetting , 0},//70
	{PARA_TiltSwitchSetting , 1},//71
	{PARA_AngleSensorType , 0},//72
	{PARA_AnaLogLimitDetSwitch , 0},//73
	{PARA_IsNoLoadCalibration , 1},//74
	{PARA_IsOverLoadCalibration , 1},//75
	{PARA_SetDescentHeightValue , 1},//76
	{PARA_ReleaseBrake , 1},//77
	{PARA_SetOutHeight , 2},//80
	{PARA_AngleSimulationUpLimit , 2043},//81 �궨��
	{PARA_AngleSimulationDownLimit , 882},//82  �궨��
	{PARA_PumpDriveCurrentLimitRatio0 , 0},//83
	{PARA_PumpSpdAccRatio0 , 0},//84
	{PARA_PropDKp0 , 8192},//85
	{PARA_PropDKi0 , 4096},//86
	{PARA_PropDMaxCurrent0 , 1050},//87
	{PARA_PropDMinCurrent0 , 180},//88
	{PARA_PropDAccPeriod0 , 10},//89
	{PARA_PropDDitherPeriod0 , 40},//90
	{PARA_PropDDitherRatio0 , 5},//91
	{PARA_PropValveResistance0 , 20},//92
	{PARA_PumpDriveCurrentLimitRatio1 , 0},//96
	{PARA_PumpSpdAccRatio1 , 0},//97
	{PARA_PropDKp1 , 8192},//98
	{PARA_PropDKi1 , 4096},//99
	{PARA_PropDMaxCurrent1 , 1050},//100
	{PARA_PropDMinCurrent1 , 180},//101
	{PARA_PropDAccPeriod1 , 10},//102
	{PARA_PropDDitherPeriod1 , 40},//103
	{PARA_PropDDitherRatio1 , 5},//104
	{PARA_PropValveResistance1 , 20},//105
	{PARA_CanBaudRate , 250},//106
	{PARA_EmptyPressure , 3000},//107
	{PARA_FullPressure , 4000},//108
	{PARA_DriverFlag , 1022},//109
	{PARA_MinAngle , 300},//112
	{PARA_MaxAngle , 2010},//113
	{PARA_BatSocPalyBack , 0},//114
	{PARA_ValveType , 451},//115
	{PARA_AnticollisionFunc , 1},//116
	{PARA_ValueOpenLoopCurrent , 800},//117
	{PARA_ValueOpenPercentage , 80},//118
	{PARA_CanOpenNodeId ,44},//119
	{PARA_AngleValue0 , 0},//120
	{PARA_AngleValue1 , 0},//121
	{PARA_AngleValue2 , 0},//122
	{PARA_AngleValue3 , 0},//123
//	{PARA_AngleValue4 , },//124 0507��������
	{PARA_AngleValue5 , 127},//125 �ֱ�ǰ
	{PARA_AngleValue6 , 0},//126
	{PARA_AngleValue7 , 129},//127 �ֱ���
	{PARA_Driver1Vol , 25680},//128
	{PARA_Driver2Vol , 25680},//129
	{PARA_Driver3Vol , 25680},//130
	{PARA_Driver4Vol , 25680},//131
	{PARA_Driver5Vol , 25680},//132
	{PARA_Driver6Vol , 25680},//133
	{PARA_Driver7Vol , 25680},//134
	{PARA_Driver8Vol , 25680},//135
	{PARA_Driver9Vol , 25680},//136
	{PARA_Driver10Vol, 25680},//137
	{PARA_LogLevel, 0},//138
	{PARA_LogModel, 0},//139
	{PARA_MotorMaxSpd, 3000},//140 �����жϵķ����ٶ���ֵ
	{PARA_Analog3DeadZoneMinVal, 5},//141
	{PARA_Analog3DeadZoneMaxVal, 30},//142
	{PARA_Analog3MidVal, 50},//143
	{PARA_ThrottleType, 0},//144
	{PARA_ThrottleFDeadZoneMinVal, 30},//145 ����ת��ʱ�ط���ֵ
	{PARA_ThrottleFDeadZoneMaxVal, 0},//146 �����ط���ֵ
	{PARA_ThrottleFMidVal, 4},//147 ǰ�������л�
	{PARA_ThrottleBDeadZoneMinVal, 0},//148 ��������
	{PARA_ThrottleBDeadZoneMaxVal, 1},//149 �µ���ʱ
	{PARA_ThrottleBMidVal, 15},//150  ˮƽ�ط�
	{PARA_BrakeType, 0},//151
	{PARA_BrakeFDeadZoneMinVal, 0},//152
	{PARA_BrakeFDeadZoneMaxVal, 0},//153
	{PARA_BrakeFMidVal, 0},//154
	{PARA_BrakeBDeadZoneMinVal, 0},//155
	{PARA_BrakeBDeadZoneMaxVal, 1},//156
	{PARA_BrakeBMidVal, 2},//157
//	{PARA_Gear1Spd, 0},//160
//	{PARA_Gear2Spd, 0},//161
//	{PARA_Gear3Spd, 0},//162
//	{PARA_Gear4Spd, 0},//163
	{PARA_RatioOfTransmission, 0},//164
	{PARA_MaintenancePeriod, 0},//165
//	{PARA_RemotePara, 2},//166
//	{PARA_PumpMotorGear1, 0},//167
//	{PARA_PumpMotorGear2, 0},//168
//	{PARA_TurnWithDecStartAngle, 0},//169
//	{PARA_TurnWithDecEndAngle, 0},//170
//	{PARA_AngleWithStartSpdPer, 0},//171
//	{PARA_AngleWithEndSpdPer, 0},//172
	{PARA_HourCountPowerOn, 0b1011},
};

static uint16_t Para0608C[][2] = {
	{PARA_FastDriveSpeed , 80},//0
	{PARA_SlowDriveSpeed , 50},//1
	{PARA_DriveSpeedAfterLift , 30},//2
	{PARA_LiftSpeed , 50},//3
	{PARA_MaxTurnSpeed , 60},//4
	{PARA_TurnPowerLimit , 30},//5
	{PAPA_DeadZoneAdjust , 10},//6
	{PARA_BrakeFastDrive , 10},//7
	{PARA_BrakeSlowDrive , 3},//8
	{PARA_BrakeDriveAfterLift , 5},//9
	{PARA_BrakeLift , 15},//10
	{PARA_BrakeLower , 22},//11
	{PARA_BrakeTurn , 2},//12
	{PARA_BrakeAntiPinch , 90},//13
	{PARA_LowerSpeed , 80},//16
	{PARA_OverLoadStabilityDelay , 1},//17
	{PARA_DynamicOverLoadPercent , 13},//18
	{PARA_StaticOverLoadPercent , 31},//19
	{PARA_MaxDifferencePercent , 35},//20
	{PARA_DriveMotorEncoder , 1},//21			��δʹ�ò���
	{PARA_MotorHighSpeedDeceRate , 10},//22		  �� δʹ�ò���
	{PARA_MotorLowSpeedDeceRate , 15},//23			��δʹ�ò���
	{PARA_VoiceAlarmVolume , 1},//24
	{PARA_CurveFastDrive , 10},//25
	{PARA_CurveSlowDrive , 2},//26
	{PARA_CurveDriveAfterLift , 2},//27
	{PARA_CurveLift , 5},//28
	{PARA_CurveLower , 2},//29
	{PARA_CurveTurn , 2},//32
	{PARA_AccAndDecFastDrive , 2},//33
	{PARA_AccAndDecSlowDrive , 2},//34
	{PARA_AccAndDecAfterLift , 2},//35
	{PARA_AccAndDecLift , 6},//36
	{PARA_AccAndDecLower , 8},//37
	{PARA_AccAndDecTurn , 2},//38
	{PARA_AccAndDecAntiPinch , 2},//39
	{PARA_PumpMotorEncoder , 21},//40
	{PARA_VehicleType , 50},//41
	{PARA_VehcileHeight , 0},//42
	{PARA_PressureSensorType , 1},//43
	{PARA_PitProtectFunc , 1},//44
	{PARA_AntiPinchFunc , 1},//45
	{PARA_ActAlmFunc , 1},//48
	{PARA_WeighFunc , 0},//49
	{PARA_ParallelValveReverseFunc , 1},//50
	{PARA_LowBatAlmFunc , 1},//51
	{PARA_LowVolAlmTime , 12},//52
	{PARA_LowVolShutDownTime , 30},//53
	{PARA_UpperCtlButSleep , 10},//54
	{PARA_LanguageType , 2},//55
	{PARA_BatteryType , 2},//56
	{PARA_SpeakerSync , 1},//57
	{PARA_LowerPumpType , 0},//58
	{PARA_PressureType , 0},//59
	{PARA_AngleSimulationLimit , 0},//60
	{PARA_LiftReverseFunc , 0},//61
	{PARA_FourPointWeightFunc , 0},//64
	{PARA_DriverType , 6},//65
	{PARA_PasswordLock , 1},//66
	{PARA_InAndOutFunc , 0},//67
	{PARA_HeartBeatQueryFunc , 3},//68
	{PARA_LowBatteryMode , 0},//69
	{PARA_AngleSensorSetting , 0},//70
	{PARA_TiltSwitchSetting , 1},//71
	{PARA_AngleSensorType , 0},//72
	{PARA_AnaLogLimitDetSwitch , 0},//73
	{PARA_IsNoLoadCalibration , 1},//74
	{PARA_IsOverLoadCalibration , 1},//75
	{PARA_SetDescentHeightValue , 1},//76
	{PARA_ReleaseBrake , 1},//77
	{PARA_SetOutHeight , 2},//80
	{PARA_AngleSimulationUpLimit , 2043},//81 �궨��
	{PARA_AngleSimulationDownLimit , 882},//82  �궨��
	{PARA_PumpDriveCurrentLimitRatio0 , 0},//83
	{PARA_PumpSpdAccRatio0 , 0},//84
	{PARA_PropDKp0 , 8192},//85
	{PARA_PropDKi0 , 4096},//86
	{PARA_PropDMaxCurrent0 , 5},//87
	{PARA_PropDMinCurrent0 , 5},//88
	{PARA_PropDAccPeriod0 , 10},//89
	{PARA_PropDDitherPeriod0 , 40},//90
	{PARA_PropDDitherRatio0 , 5},//91
	{PARA_PropValveResistance0 , 20},//92
	{PARA_PumpDriveCurrentLimitRatio1 , 0},//96
	{PARA_PumpSpdAccRatio1 , 0},//97
	{PARA_PropDKp1 , 8192},//98
	{PARA_PropDKi1 , 4096},//99
	{PARA_PropDMaxCurrent1 , 1050},//100
	{PARA_PropDMinCurrent1 , 180},//101
	{PARA_PropDAccPeriod1 , 10},//102
	{PARA_PropDDitherPeriod1 , 40},//103
	{PARA_PropDDitherRatio1 , 5},//104
	{PARA_PropValveResistance1 , 20},//105
	{PARA_CanBaudRate , 250},//106
	{PARA_EmptyPressure , 3000},//107
	{PARA_FullPressure , 4000},//108
	{PARA_DriverFlag , 766},//109
	{PARA_MinAngle , 300},//112
	{PARA_MaxAngle , 2010},//113
	{PARA_BatSocPalyBack , 0},//114
	{PARA_ValveType , 451},//115
	{PARA_AnticollisionFunc , 1},//116
	{PARA_ValueOpenLoopCurrent , 800},//117
	{PARA_ValueOpenPercentage , 80},//118
	{PARA_CanOpenNodeId ,44},//119
	{PARA_AngleValue0 , 0},//120
	{PARA_AngleValue1 , 0},//121
	{PARA_AngleValue2 , 0},//122
	{PARA_AngleValue3 , 0},//123
//	{PARA_AngleValue4 , },//124 0507��������
	{PARA_AngleValue5 , 127},//125 �ֱ�ǰ
	{PARA_AngleValue6 , 0},//126
	{PARA_AngleValue7 , 129},//127 �ֱ���
	{PARA_Driver1Vol , 25680},//128
	{PARA_Driver2Vol , 25680},//129
	{PARA_Driver3Vol , 25680},//130
	{PARA_Driver4Vol , 25680},//131
	{PARA_Driver5Vol , 25680},//132
	{PARA_Driver6Vol , 25680},//133
	{PARA_Driver7Vol , 25680},//134
	{PARA_Driver8Vol , 25680},//135
	{PARA_Driver9Vol , 25680},//136
	{PARA_Driver10Vol, 25680},//137
	{PARA_LogLevel, 0},//138
	{PARA_LogModel, 0},//139
	{PARA_MotorMaxSpd, 3000},//140 �����жϵķ����ٶ���ֵ
	{PARA_Analog3DeadZoneMinVal, 5},//141
	{PARA_Analog3DeadZoneMaxVal, 30},//142
	{PARA_Analog3MidVal, 50},//143
	{PARA_ThrottleType, 0},//144
	{PARA_ThrottleFDeadZoneMinVal, 30},//145 ����ת��ʱ�ط���ֵ
	{PARA_ThrottleFDeadZoneMaxVal, 0},//146 �����ط���ֵ
	{PARA_ThrottleFMidVal, 4},//147 ǰ�������л�
	{PARA_ThrottleBDeadZoneMinVal, 0},//148 ��������
	{PARA_ThrottleBDeadZoneMaxVal, 1},//149 �µ���ʱ
	{PARA_ThrottleBMidVal, 15},//150  ˮƽ�ط�
	{PARA_BrakeType, 0},//151
	{PARA_BrakeFDeadZoneMinVal, 0},//152
	{PARA_BrakeFDeadZoneMaxVal, 0},//153
	{PARA_BrakeFMidVal, 0},//154
	{PARA_BrakeBDeadZoneMinVal, 0},//155
	{PARA_BrakeBDeadZoneMaxVal, 1},//156
	{PARA_BrakeBMidVal, 2},//157
//	{PARA_Gear1Spd, 0},//160
//	{PARA_Gear2Spd, 0},//161
//	{PARA_Gear3Spd, 0},//162
//	{PARA_Gear4Spd, 0},//163
	{PARA_RatioOfTransmission, 0},//164
	{PARA_MaintenancePeriod, 0},//165
//	{PARA_RemotePara, 2},//166
//	{PARA_PumpMotorGear1, 0},//167
//	{PARA_PumpMotorGear2, 0},//168
//	{PARA_TurnWithDecStartAngle, 0},//169
//	{PARA_TurnWithDecEndAngle, 0},//170
//	{PARA_AngleWithStartSpdPer, 0},//171
//	{PARA_AngleWithEndSpdPer, 0},//172
	{PARA_HourCountPowerOn, 0b1011},//173 	ǰ��bit 00�룬01�֣�10 6�֣�11Сʱ��bit1��������ʱ1���ϵ��ʱ0��bit0������Сʱ�ƣ�
};
static uint16_t Para0808[][2] = {
	{PARA_FastDriveSpeed , 90},//0
	{PARA_SlowDriveSpeed , 90},//1
	{PARA_DriveSpeedAfterLift , 30},//2
	{PARA_LiftSpeed , 60},//3
	{PARA_MaxTurnSpeed , 60},//4
	{PARA_TurnPowerLimit , 30},//5
	{PAPA_DeadZoneAdjust , 10},//6
	{PARA_BrakeFastDrive , 1},//7
	{PARA_BrakeSlowDrive , 1},//8
	{PARA_BrakeDriveAfterLift , 5},//9
	{PARA_BrakeLift , 20},//10
	{PARA_BrakeLower , 20},//11
	{PARA_BrakeTurn , 2},//12
	{PARA_BrakeAntiPinch , 85},//13
	{PARA_LowerSpeed , 90},//16
	{PARA_OverLoadStabilityDelay , 1},//17
	{PARA_DynamicOverLoadPercent , 13},//18
	{PARA_StaticOverLoadPercent , 31},//19
	{PARA_MaxDifferencePercent , 35},//20
	{PARA_DriveMotorEncoder , 1},//21			��δʹ�ò���
	{PARA_MotorHighSpeedDeceRate , 2},//22		  �� δʹ�ò���
	{PARA_MotorLowSpeedDeceRate , 2},//23			��δʹ�ò���
	{PARA_VoiceAlarmVolume , 1},//24
	{PARA_CurveFastDrive , 10},//25
	{PARA_CurveSlowDrive , 2},//26
	{PARA_CurveDriveAfterLift , 2},//27
	{PARA_CurveLift , 5},//28
	{PARA_CurveLower , 5},//29
	{PARA_CurveTurn , 2},//32
	{PARA_AccAndDecFastDrive , 2},//33
	{PARA_AccAndDecSlowDrive , 2},//34
	{PARA_AccAndDecAfterLift , 2},//35
	{PARA_AccAndDecLift , 4},//36
	{PARA_AccAndDecLower , 4},//37
	{PARA_AccAndDecTurn , 2},//38
	{PARA_AccAndDecAntiPinch , 2},//39
	{PARA_PumpMotorEncoder , 21},//40
	{PARA_VehicleType , 50},//41
	{PARA_VehcileHeight , 0},//42
	{PARA_PressureSensorType , 1},//43
	{PARA_PitProtectFunc , 1},//44
	{PARA_AntiPinchFunc , 1},//45
	{PARA_ActAlmFunc , 1},//48
	{PARA_WeighFunc , 0},//49
	{PARA_ParallelValveReverseFunc , 1},//50
	{PARA_LowBatAlmFunc , 1},//51
	{PARA_LowVolAlmTime , 12},//52
	{PARA_LowVolShutDownTime , 30},//53
	{PARA_UpperCtlButSleep , 10},//54
	{PARA_LanguageType , 2},//55
	{PARA_BatteryType , 2},//56
	{PARA_SpeakerSync , 1},//57
	{PARA_LowerPumpType , 0},//58
	{PARA_PressureType , 0},//59
	{PARA_AngleSimulationLimit , 0},//60
	{PARA_LiftReverseFunc , 0},//61
	{PARA_FourPointWeightFunc , 0},//64
	{PARA_DriverType , 6},//65
	{PARA_PasswordLock , 1},//66
	{PARA_InAndOutFunc , 0},//67
	{PARA_HeartBeatQueryFunc , 3},//68
	{PARA_LowBatteryMode , 0},//69
	{PARA_AngleSensorSetting , 0},//70
	{PARA_TiltSwitchSetting , 1},//71
	{PARA_AngleSensorType , 0},//72
	{PARA_AnaLogLimitDetSwitch , 0},//73
	{PARA_IsNoLoadCalibration , 1},//74
	{PARA_IsOverLoadCalibration , 1},//75
	{PARA_SetDescentHeightValue , 1},//76
	{PARA_ReleaseBrake , 1},//77
	{PARA_SetOutHeight , 2},//80
	{PARA_AngleSimulationUpLimit , 2043},//81 �궨��
	{PARA_AngleSimulationDownLimit , 882},//82  �궨��
	{PARA_PumpDriveCurrentLimitRatio0 , 0},//83
	{PARA_PumpSpdAccRatio0 , 0},//84
	{PARA_PropDKp0 , 8192},//85
	{PARA_PropDKi0 , 4096},//86
	{PARA_PropDMaxCurrent0 , 5},//87
	{PARA_PropDMinCurrent0 , 5},//88
	{PARA_PropDAccPeriod0 , 10},//89
	{PARA_PropDDitherPeriod0 , 40},//90
	{PARA_PropDDitherRatio0 , 5},//91
	{PARA_PropValveResistance0 , 20},//92
	{PARA_PumpDriveCurrentLimitRatio1 , 0},//96
	{PARA_PumpSpdAccRatio1 , 0},//97
	{PARA_PropDKp1 , 8192},//98
	{PARA_PropDKi1 , 4096},//99
	{PARA_PropDMaxCurrent1 , 1020},//100
	{PARA_PropDMinCurrent1 , 180},//101
	{PARA_PropDAccPeriod1 , 10},//102
	{PARA_PropDDitherPeriod1 , 40},//103
	{PARA_PropDDitherRatio1 , 5},//104
	{PARA_PropValveResistance1 , 20},//105
	{PARA_CanBaudRate , 250},//106
	{PARA_EmptyPressure , 3000},//107
	{PARA_FullPressure , 4000},//108
	{PARA_DriverFlag , 1022},//109
	{PARA_MinAngle , 300},//112
	{PARA_MaxAngle , 2010},//113
	{PARA_BatSocPalyBack , 0},//114
	{PARA_ValveType , 451},//115
	{PARA_AnticollisionFunc , 1},//116
	{PARA_ValueOpenLoopCurrent , 800},//117
	{PARA_ValueOpenPercentage , 80},//118
	{PARA_CanOpenNodeId ,44},//119
	{PARA_AngleValue0 , 0},//120
	{PARA_AngleValue1 , 0},//121
	{PARA_AngleValue2 , 0},//122
	{PARA_AngleValue3 , 0},//123
//	{PARA_AngleValue4 , },//124 0507��������
	{PARA_AngleValue5 , 127},//125 �ֱ�ǰ
	{PARA_AngleValue6 , 0},//126
	{PARA_AngleValue7 , 129},//127 �ֱ���
	{PARA_Driver1Vol , 25680},//128
	{PARA_Driver2Vol , 25680},//129
	{PARA_Driver3Vol , 25680},//130
	{PARA_Driver4Vol , 25680},//131
	{PARA_Driver5Vol , 25680},//132
	{PARA_Driver6Vol , 25680},//133
	{PARA_Driver7Vol , 25680},//134
	{PARA_Driver8Vol , 25680},//135
	{PARA_Driver9Vol , 25680},//136
	{PARA_Driver10Vol, 25680},//137
	{PARA_LogLevel, 0},//138
	{PARA_LogModel, 0},//139
	{PARA_MotorMaxSpd, 3000},//140 �����жϵķ����ٶ���ֵ
	{PARA_Analog3DeadZoneMinVal, 5},//141
	{PARA_Analog3DeadZoneMaxVal, 30},//142
	{PARA_Analog3MidVal, 50},//143
	{PARA_ThrottleType, 0},//144
	{PARA_ThrottleFDeadZoneMinVal, 30},//145 ����ת��ʱ�ط���ֵ
	{PARA_ThrottleFDeadZoneMaxVal, 0},//146 �����ط���ֵ
	{PARA_ThrottleFMidVal, 4},//147 ǰ�������л�
	{PARA_ThrottleBDeadZoneMinVal, 0},//148 ��������
	{PARA_ThrottleBDeadZoneMaxVal, 1},//149 �µ���ʱ
	{PARA_ThrottleBMidVal, 15},//150  ˮƽ�ط�
	{PARA_BrakeType, 0},//151
	{PARA_BrakeFDeadZoneMinVal, 0},//152
	{PARA_BrakeFDeadZoneMaxVal, 0},//153
	{PARA_BrakeFMidVal, 0},//154
	{PARA_BrakeBDeadZoneMinVal, 0},//155
	{PARA_BrakeBDeadZoneMaxVal, 1},//156
	{PARA_BrakeBMidVal, 2},//157
//	{PARA_Gear1Spd, 0},//160
//	{PARA_Gear2Spd, 0},//161
//	{PARA_Gear3Spd, 0},//162
//	{PARA_Gear4Spd, 0},//163
	{PARA_RatioOfTransmission, 0},//164
	{PARA_MaintenancePeriod, 0},//165
//	{PARA_RemotePara, 2},//166
//	{PARA_PumpMotorGear1, 0},//167
//	{PARA_PumpMotorGear2, 0},//168
//	{PARA_TurnWithDecStartAngle, 0},//169
//	{PARA_TurnWithDecEndAngle, 0},//170
//	{PARA_AngleWithStartSpdPer, 0},//171
//	{PARA_AngleWithEndSpdPer, 0},//172
	{PARA_HourCountPowerOn, 0b1011},
	
};



static void vMultiParaChange(uint8_t u8Length,uint16_t (*u16Array)[2])
{
	static uint8_t u8SaveState;
	static uint8_t i;
	i32SetPara(PARA_PressureVlaue1,1);
	while(1)
	{		
		u8SaveState = 0;
		for(i = 0;i < u8Length; i ++)
		{
				if(u16SaveParaToEeprom(u16Array[i][0],u16Array[i][1]))
				{
					u8SaveState++;					
				}
		}
		if(0 == u8SaveState)
			break;
	}
}

static void vMcuParaRevProc(uint8_t *u8Data, uint16_t u16Length)
{
	uint8_t u8DataArray[8];
	
	memcpy(u8DataArray,u8Data,8);
	int16_t i16DataSend = 0;
	if(u8DataArray[0] == 0x60)
		McuParaIdx++;
}
static void vMCUParaWrite(uint8_t u8Length,uint16_t (*u16Array)[2])
{
	static uint8_t u8Vector[8];
		i32SetPara(PARA_PressureVlaue1,2);
		memset(u8Vector,0,sizeof(u8Vector));
		u8Vector[0] = 0x23;
		u8Vector[1] = u16Array[McuParaIdx][0] & 0xFF;
		u8Vector[2] = 0;
		u8Vector[3] = 0;
		u8Vector[4] = u16Array[McuParaIdx][1] & 0xFF;
		u8Vector[5] = (u16Array[McuParaIdx][1] >> 8) & 0xFF;
		u8Vector[6] = 0;
		u8Vector[7] = 0;
		
		vQueryMcuPara(u8Vector,8);	
}
	

static void vDriverBanParallePump()
{
	uint16_t u16TmpPara;
	u16TmpPara = i32GetPara(PARA_DriverFlag);
	if((u16TmpPara >> 8) & 0x01)
	{
		u16TmpPara &= ~ (1 << 8);
		u16SaveParaToEeprom(PARA_DriverFlag,u16TmpPara);
	}
}	
static void vChangeMcuPara()
{
	static uint16_t McuParaIdxOld;
	if((McuParaIdx == 0)||(McuParaIdxOld != McuParaIdx))
	{
		McuParaIdxOld = McuParaIdx;
		if	((12 == i32GetPara(PARA_CARCODE))  //-LS0407H
			|| (40 == i32GetPara(PARA_CARCODE))  //-LS0507HM
			|| (41 == i32GetPara(PARA_CARCODE))) //LS0507HM-Li
			vMCUParaWrite((sizeof(MCU_Para5m)/sizeof(MCU_Para5m[0])),MCU_Para5m);
		else
			vMCUParaWrite((sizeof(MCU_Para6m)/sizeof(MCU_Para6m[0])),MCU_Para6m);
	}
}
static void vChangeCarTypePara(uint16_t u16Carcode)
{

	{
		switch (u16Carcode) 
		{
			case 12:  //0407H
				vMultiParaChange((sizeof(Para0507)/sizeof(Para0507[0])), Para0507); 
				break;
			case 13:  //0607H
				vMultiParaChange((sizeof(Para0608C)/sizeof(Para0608C[0])), Para0608C);
				break;  
			case 14:  //0608H .
				vMultiParaChange((sizeof(Para0608C)/sizeof(Para0608C[0])), Para0608C); 
				break;
			case 15:  // 0808				
				vMultiParaChange((sizeof(Para0808)/sizeof(Para0808[0])), Para0808); 
				break;
			case 16:  //0812H
				vMultiParaChange((sizeof(Para0808)/sizeof(Para0808[0])), Para0808); 
				break;
			case 17: //1012H	
				vMultiParaChange((sizeof(Para1212Dif)/sizeof(Para1212Dif[0])), Para1212Dif); 
				break;
			case 18:  // 1212
				vMultiParaChange((sizeof(Para1212Dif)/sizeof(Para1212Dif[0])), Para1212Dif);
				break;
			case 40:  // 0507hm
				vMultiParaChange((sizeof(Para0507)/sizeof(Para0507[0])), Para0507); 
				break;
			case 41:	//0507hmli
				vMultiParaChange((sizeof(Para0507)/sizeof(Para0507[0])), Para0507);
				u16SaveParaToEeprom(PARA_BatteryType,0);//�޸ĵ������Ϊ﮵��
				break;
			case 42:	//0608 C				
				vMultiParaChange((sizeof(Para0608C)/sizeof(Para0608C[0])), Para0608C); 
				break;
			case 43:	//0808 C
				vMultiParaChange((sizeof(Para0808)/sizeof(Para0808[0])), Para0808);
				break;                                                             
			case 44://1012 C
				vMultiParaChange((sizeof(Para1212Dif)/sizeof(Para1212Dif[0])), Para1212Dif);
				break;	
			case 45://1212 C	
				vMultiParaChange((sizeof(Para1212Dif)/sizeof(Para1212Dif[0])), Para1212Dif);
				break;		
			default:
				break;
		}
	}
}


static void vUserSetsInit()
{
	sgUserSets.u16Data = i32GetPara(PARA_USERSETS);
	if(0 == sgUserSets.b1StartFast)//��������
		sgLimit.b1Slow = 1;
	if(1 == sgUserSets.b1StartWithMode)
		 u8PcuMode	= MOVE_MODE;
	
	if(1 == sgUserSets.b1Err18LiftAllow)
	{
		sgErrCodeInfo[117].b1NoUp = 0;
	}
	if(11 != i32GetPara(PARA_HourCountPowerOn))//24.3.12�̶�Сʱ��Ϊ������ʱ
	{
		u16SaveParaToEeprom(PARA_HourCountPowerOn,11);
	}
	
	//24.3.18������������
	{
		uint16_t u16tmp;
		u16tmp = i32GetPara(PARA_DriverFlag);
		if(FunctionDisable != gUserInfo.u8AngleSimulationLimit)
		{
			if(0 == (u16tmp & (1<<10)))
			{
				u16tmp |= (1<<10);
				u16SaveParaToEeprom(PARA_DriverFlag,u16tmp);
			}
		}
//		else//���ر�  //24.5.08 ������������������ر�ģ�������
//		{
//			if(0 != (u16tmp & (1<<10)))
//			{
//				u16tmp &=~ (1<<10);
//				u16SaveParaToEeprom(PARA_DriverFlag,u16tmp);
//			}			
//		}
		if((FunctionDisable != gUserInfo.u8WeighFunc)
			||(FunctionDisable != gUserInfo.u8FourPointWeightFunc))	//���������ʹ��	
		{
			if(0 == (u16tmp & (1<<11)))
			{
				u16tmp |= (1<<11);
				u16SaveParaToEeprom(PARA_DriverFlag,u16tmp);
			}
		}
		else
		{
			if(0 != (u16tmp & (1<<11)))
			{
				u16tmp &=~ (1<<11);
				u16SaveParaToEeprom(PARA_DriverFlag,u16tmp);
			}
		}
	}
	{
		uint16_t u16SftCode;
		u16EepromRead(SOFTWARE_VERSION_STATE,&u16SftCode,1);
		//i32SetPara(PARA_LoadRate,u16SftCode);
//		i32SetPara(PARA_CalibrationStatus,SOFTWARE_VERSION_CODE);
		if(SOFTWARE_VERSION_CODE != u16SftCode)//��������汾ʱˢ�²�����
		{
			if(0 == u16SftCode || 0xFFFF == u16SftCode)
			{
					//117�汾֮ǰ��û�洢�汾��Ϣ
					//118�汾��Ҫ���������λ�Ƕȣ���ֵ���رճ��ع��ܼ��ĵ���ع���
				u16SaveParaToEeprom(PARA_LOWLIMIT_RANGE,5);
				u16SaveParaToEeprom(PARA_LOWLIMIT_ANGLE,490);
				u16SaveParaToEeprom(PARA_FourPointWeightFunc,FunctionDisable);
				u16SaveParaToEeprom(PARA_WeighFunc,FunctionDisable);		
			}
			else
			{ /*�Բ�ͬ�汾�������д���*/
				/*121�汾���£������λ����Ϊ490*/
				if(u16SftCode < 0x0121 )
				{
					u16SaveParaToEeprom(PARA_LOWLIMIT_ANGLE,490);
				}
			}
			/*��������ز���֮�󣬸��±���İ汾��*/
			u16EepromWrite(SOFTWARE_VERSION_STATE,SOFTWARE_VERSION_CODE,1);			
		}
	}
}

||||||| .r31
=======
/*******************
���ݳ��͸Ĳ���
**********************/
static	uint16_t Para0507[][2] = {
	{PARA_FastDriveSpeed , 63},//0
	{PARA_SlowDriveSpeed , 99},//1
	{PARA_DriveSpeedAfterLift , 36},//2
	{PARA_LiftSpeed , 40},//3
	{PARA_MaxTurnSpeed , 40},//4
	{PARA_TurnPowerLimit , 30},//5
	{PAPA_DeadZoneAdjust , 10},//6
	{PARA_BrakeFastDrive , 5},//7
	{PARA_BrakeSlowDrive , 2},//8
	{PARA_BrakeDriveAfterLift , 5},//9
	{PARA_BrakeLift , 7},//10
	{PARA_BrakeLower , 5},//11
	{PARA_BrakeTurn , 2},//12
	{PARA_BrakeAntiPinch , 82},//13
	{PARA_LowerSpeed , 99},//16
	{PARA_OverLoadStabilityDelay , 1},//17
	{PARA_DynamicOverLoadPercent , 13},//18
	{PARA_StaticOverLoadPercent , 31},//19
	{PARA_MaxDifferencePercent , 35},//20
	{PARA_DriveMotorEncoder , 1},//21			��δʹ�ò���
	{PARA_MotorHighSpeedDeceRate , 5},//22		  �� δʹ�ò���
	{PARA_MotorLowSpeedDeceRate , 15},//23			��δʹ�ò���
	{PARA_VoiceAlarmVolume , 1},//24
	{PARA_CurveFastDrive , 15},//25
	{PARA_CurveSlowDrive , 2},//26
	{PARA_CurveDriveAfterLift , 2},//27
	{PARA_CurveLift , 5},//28
	{PARA_CurveLower , 2},//29
	{PARA_CurveTurn , 2},//32
	{PARA_AccAndDecFastDrive , 2},//33
	{PARA_AccAndDecSlowDrive , 2},//34
	{PARA_AccAndDecAfterLift , 2},//35
	{PARA_AccAndDecLift , 2},//36
	{PARA_AccAndDecLower , 8},//37
	{PARA_AccAndDecTurn , 2},//38
	{PARA_AccAndDecAntiPinch , 2},//39
	{PARA_PumpMotorEncoder , 21},//40
	{PARA_VehicleType , 69},//41
	{PARA_VehcileHeight , 0},//42
	{PARA_PressureSensorType , 1},//43
	{PARA_PitProtectFunc , 1},//44
	{PARA_AntiPinchFunc , 1},//45
	{PARA_ActAlmFunc , 1},//48
	{PARA_WeighFunc , 0},//49
	{PARA_ParallelValveReverseFunc , 1},//50
	{PARA_LowBatAlmFunc , 1},//51
	{PARA_LowVolAlmTime , 12},//52
	{PARA_LowVolShutDownTime , 30},//53
	{PARA_UpperCtlButSleep , 10},//54
	{PARA_LanguageType , 2},//55
	{PARA_BatteryType , 2},//56
	{PARA_SpeakerSync , 1},//57
	{PARA_LowerPumpType , 1},//58
	{PARA_PressureType , 1},//59
	{PARA_AngleSimulationLimit , 1},//60
	{PARA_LiftReverseFunc , 0},//61
	{PARA_FourPointWeightFunc , 0},//64
	{PARA_DriverType , 6},//65
	{PARA_PasswordLock , 1},//66
	{PARA_InAndOutFunc , 0},//67
	{PARA_HeartBeatQueryFunc , 1},//68
	{PARA_LowBatteryMode , 0},//69
	{PARA_AngleSensorSetting , 1},//70
	{PARA_TiltSwitchSetting , 1},//71
	{PARA_AngleSensorType , 0},//72
	{PARA_AnaLogLimitDetSwitch , 0},//73
	{PARA_IsNoLoadCalibration , 1},//74
	{PARA_IsOverLoadCalibration , 1},//75
	{PARA_SetDescentHeightValue , 1},//76
	{PARA_ReleaseBrake , 1},//77
	{PARA_SetOutHeight , 2},//80
	{PARA_AngleSimulationUpLimit , 2043},//81 �궨��
	{PARA_AngleSimulationDownLimit , 882},//82  �궨��
	{PARA_PumpDriveCurrentLimitRatio0 , 0},//83
	{PARA_PumpSpdAccRatio0 , 0},//84
	{PARA_PropDKp0 , 8192},//85
	{PARA_PropDKi0 , 4096},//86
	{PARA_PropDMaxCurrent0 , 5},//87
	{PARA_PropDMinCurrent0 , 5},//88
	{PARA_PropDAccPeriod0 , 10},//89
	{PARA_PropDDitherPeriod0 , 16},//90
	{PARA_PropDDitherRatio0 , 5},//91
	{PARA_PropValveResistance0 , 20},//92
	{PARA_PumpDriveCurrentLimitRatio1 , 0},//96
	{PARA_PumpSpdAccRatio1 , 0},//97
	{PARA_PropDKp1 , 8192},//98
	{PARA_PropDKi1 , 4096},//99
	{PARA_PropDMaxCurrent1 , 1000},//100
	{PARA_PropDMinCurrent1 , 460},//101
	{PARA_PropDAccPeriod1 , 20},//102
	{PARA_PropDDitherPeriod1 , 40},//103
	{PARA_PropDDitherRatio1 , 5},//104
	{PARA_PropValveResistance1 , 20},//105
	{PARA_CanBaudRate , 500},//106
	{PARA_EmptyPressure , 3000},//107
	{PARA_FullPressure , 4000},//108
	{PARA_DriverFlag , 2046},//109
	{PARA_MinAngle , 300},//112
	{PARA_MaxAngle , 2010},//113
	{PARA_BatSocPalyBack , 0},//114
	{PARA_ValveType , 449},//115
	{PARA_AnticollisionFunc , 1},//116
	{PARA_ValueOpenLoopCurrent , 800},//117
	{PARA_ValueOpenPercentage , 80},//118
	{PARA_CanOpenNodeId ,44},//119
	{PARA_AngleValue0 , 0},//120
//{PARA_AngleValue1 , 0},//121
	{PARA_AngleValue2 , 5},//122
	{PARA_AngleValue3 , 490},//123
//	{PARA_AngleValue4 , },//124 0507��������
	{PARA_AngleValue5 , 127},//125 �ֱ�ǰ
	{PARA_AngleValue6 , 0},//126
	{PARA_AngleValue7 , 129},//127 �ֱ���
	{PARA_Driver1Vol , 25680},//128
	{PARA_Driver2Vol , 25680},//129
	{PARA_Driver3Vol , 25680},//130
	{PARA_Driver4Vol , 25680},//131
	{PARA_Driver5Vol , 25680},//132
	{PARA_Driver6Vol , 25680},//133
	{PARA_Driver7Vol , 25680},//134
	{PARA_Driver8Vol , 25680},//135
	{PARA_Driver9Vol , 25680},//136
	{PARA_Driver10Vol, 25680},//137
	{PARA_LogLevel, 0},//138
	{PARA_LogModel, 0},//139
	{PARA_MotorMaxSpd, 3000},//140 �����жϵķ����ٶ���ֵ
	{PARA_Analog3DeadZoneMinVal, 2},//141
	{PARA_Analog3DeadZoneMaxVal, 30},//142
	{PARA_Analog3MidVal, 3},//143
	{PARA_ThrottleType, 0},//144
	{PARA_ThrottleFDeadZoneMinVal, 10},//145 ����ת��ʱ�ط���ֵ
	{PARA_ThrottleFDeadZoneMaxVal, 3},//146 �����ط���ֵ
	{PARA_ThrottleFMidVal, 20},//147 ǰ�������л�
	{PARA_ThrottleBDeadZoneMinVal, 0},//148 ��������
	{PARA_ThrottleBDeadZoneMaxVal, 1},//149 �µ���ʱ
	{PARA_ThrottleBMidVal, 15},//150  ˮƽ�ط�
	{PARA_BrakeType, 0},//151
	{PARA_BrakeFDeadZoneMinVal, 0},//152
	{PARA_BrakeFDeadZoneMaxVal, 0},//153
	{PARA_BrakeFMidVal, 0},//154
	{PARA_BrakeBDeadZoneMinVal, 0},//155
	{PARA_BrakeBDeadZoneMaxVal, 1},//156
	{PARA_BrakeBMidVal, 2},//157
//	{PARA_Gear1Spd, 0},//160
//	{PARA_Gear2Spd, 0},//161
//	{PARA_Gear3Spd, 0},//162
//	{PARA_Gear4Spd, 0},//163
	{PARA_RatioOfTransmission, 0},//164
	{PARA_MaintenancePeriod, 0},//165
//	{PARA_RemotePara, 2},//166
//	{PARA_PumpMotorGear1, 0},//167
//	{PARA_PumpMotorGear2, 0},//168
//	{PARA_TurnWithDecStartAngle, 0},//169
//	{PARA_TurnWithDecEndAngle, 0},//170
//	{PARA_AngleWithStartSpdPer, 0},//171
//	{PARA_AngleWithEndSpdPer, 0},//172
	{PARA_HourCountPowerOn, 0b1011},//173 	ǰ��bit 00�룬01�֣�10 6�֣�11Сʱ��bit1��������ʱ1���ϵ��ʱ0��bit0������Сʱ�ƣ�
};
#if 0
static uint16_t Para0507Li[][2] = {
	{PAPA_DeadZoneAdjust , 10},//6
	{PARA_BrakeSlowDrive , 2},//8
	{PARA_BrakeDriveAfterLift , 5},//9
	{PARA_BrakeTurn , 2},//12
	{PARA_BrakeAntiPinch , 75},//13
	{PARA_VoiceAlarmVolume , 1},//24
	{PARA_CurveFastDrive , 15},//25
	{PARA_CurveSlowDrive , 2},//26
	{PARA_CurveDriveAfterLift , 2},//27
	{PARA_CurveLift , 5},//28
	{PARA_CurveLower , 2},//29
	{PARA_CurveTurn , 2},//32
	{PARA_AccAndDecSlowDrive , 2},//34
	{PARA_AccAndDecAfterLift , 2},//35
	{PARA_AccAndDecLower , 8},//37
	{PARA_AccAndDecTurn , 2},//38
	{PARA_VehicleType , 50},//41
	{PARA_PressureSensorType , 1},//43
	{PARA_PitProtectFunc , 1},//44
	{PARA_ActAlmFunc , 1},//48
	{PARA_WeighFunc , 0},//49
	{PARA_ParallelValveReverseFunc , 1},//50
	{PARA_BatteryType , 0},//56
	{PARA_SpeakerSync , 1},//57
	{PARA_LowerPumpType , 1},//58
	{PARA_PressureType , 0},//59
	{PARA_AngleSimulationLimit , 1},//60
	{PARA_LiftReverseFunc , 0},//61
	{PARA_FourPointWeightFunc , 0},//64
	{PARA_InAndOutFunc , 0},//67
	{PARA_AngleSensorSetting , 1},//70
	{PARA_TiltSwitchSetting , 1},//71
	{PARA_AngleSensorType , 0},//72
	{PARA_SetOutHeight , 0},//80
	{PARA_PumpDriveCurrentLimitRatio0 , 0},//83
	{PARA_PumpSpdAccRatio0 , 0},//84
	{PARA_PropDKp0 , 8192},//85
	{PARA_PropDKi0 , 4096},//86
	{PARA_PropDMaxCurrent0 , 5},//87
	{PARA_PropDMinCurrent0 , 5},//88
	{PARA_PropDAccPeriod0 , 10},//89
	{PARA_PropDDitherPeriod0 , 40},//90
	{PARA_PropDDitherRatio0 , 5},//91
	{PARA_PropValveResistance0 , 20},//92
	{PARA_PumpDriveCurrentLimitRatio1 , 0},//96
	{PARA_PumpSpdAccRatio1 , 0},//97
	{PARA_PropDKp1 , 8192},//98
	{PARA_PropDKi1 , 4096},//99
	{PARA_PropDMaxCurrent1 , 1000},//100
	{PARA_PropDMinCurrent1 , 400},//101
	{PARA_PropDAccPeriod1 , 10},//102
	{PARA_PropDDitherPeriod1 , 40},//103
	{PARA_PropDDitherRatio1 , 5},//104
	{PARA_PropValveResistance1 , 20},//105
	{PARA_CanBaudRate , 250},//106
	{PARA_DriverFlag , 2046},//109
	{PARA_AnticollisionFunc , 1},//116
	{PARA_AngleValue5 , 127},//125 �ֱ�ǰ
	{PARA_AngleValue6 , 0},//126
	{PARA_AngleValue7 , 129},//127 �ֱ���
	{PARA_Driver1Vol , 25680},//128
	{PARA_Driver2Vol , 25680},//129
	{PARA_Driver3Vol , 25680},//130
	{PARA_Driver4Vol , 25680},//131
	{PARA_Driver5Vol , 25680},//132
	{PARA_Driver6Vol , 25680},//133
	{PARA_Driver7Vol , 25680},//134
	{PARA_Driver8Vol , 25680},//135
	{PARA_Driver9Vol , 25680},//136
	{PARA_Driver10Vol , 25680},//137
	{PARA_MotorMaxSpd, 2700},//140 �����жϵķ����ٶ���ֵ
	{PARA_Analog3DeadZoneMinVal, 5},//141
	{PARA_Analog3DeadZoneMaxVal, 100},//142
	{PARA_Analog3MidVal, 0},//143
	{PARA_ThrottleType, 0},//144
	{PARA_ThrottleFDeadZoneMinVal, 30},//145 ����ת��ʱ�ط���ֵ
	{PARA_ThrottleFDeadZoneMaxVal, 10},//146 �����ط���ֵ
	{PARA_ThrottleFMidVal, 20},//147 ǰ�������л�
	{PARA_ThrottleBDeadZoneMinVal, 0},//148 ��������
	{PARA_ThrottleBDeadZoneMaxVal, 1},//149 �µ���ʱ
	{PARA_ThrottleBMidVal, 50},//150  ˮƽ�ط�
	{PARA_BrakeFDeadZoneMinVal, 0},//152
	{PARA_BrakeFDeadZoneMaxVal, 0},//153
	{PARA_BrakeFMidVal, 0},//154
	{PARA_BrakeBDeadZoneMinVal, 0},//155
	{PARA_BrakeBDeadZoneMaxVal, 1},//156
	{PARA_BrakeBMidVal, 2},//157
	{PARA_RatioOfTransmission, 0},//164
	{PARA_MaintenancePeriod, 0},//165
	{PARA_HourCountPowerOn, 0b1011},//173 	ǰ��bit 00�룬01�֣�10 6�֣�11Сʱ��bit1��������ʱ1���ϵ��ʱ0��bit0������Сʱ�ƣ�
};
#endif
static uint16_t Para1212Dif[][2] = {
	{PARA_FastDriveSpeed , 90},//0
	{PARA_SlowDriveSpeed , 90},//1
	{PARA_DriveSpeedAfterLift , 30},//2
	{PARA_LiftSpeed , 80},//3
	{PARA_MaxTurnSpeed , 60},//4
	{PARA_TurnPowerLimit , 30},//5
	{PAPA_DeadZoneAdjust , 10},//6
	{PARA_BrakeFastDrive , 1},//7
	{PARA_BrakeSlowDrive , 1},//8
	{PARA_BrakeDriveAfterLift , 5},//9
	{PARA_BrakeLift , 12},//10
	{PARA_BrakeLower , 15},//11
	{PARA_BrakeTurn , 2},//12
	{PARA_BrakeAntiPinch , 85},//13
	{PARA_LowerSpeed , 90},//16
	{PARA_OverLoadStabilityDelay , 1},//17
	{PARA_DynamicOverLoadPercent , 13},//18
	{PARA_StaticOverLoadPercent , 31},//19
	{PARA_MaxDifferencePercent , 35},//20
	{PARA_DriveMotorEncoder , 1},//21			��δʹ�ò���
	{PARA_MotorHighSpeedDeceRate , 2},//22		  �� δʹ�ò���
	{PARA_MotorLowSpeedDeceRate , 2},//23			��δʹ�ò���
	{PARA_VoiceAlarmVolume , 1},//24
	{PARA_CurveFastDrive , 10},//25
	{PARA_CurveSlowDrive , 2},//26
	{PARA_CurveDriveAfterLift , 2},//27
	{PARA_CurveLift , 5},//28
	{PARA_CurveLower , 5},//29
	{PARA_CurveTurn , 2},//32
	{PARA_AccAndDecFastDrive , 2},//33
	{PARA_AccAndDecSlowDrive , 2},//34
	{PARA_AccAndDecAfterLift , 2},//35
	{PARA_AccAndDecLift , 4},//36
	{PARA_AccAndDecLower , 4},//37
	{PARA_AccAndDecTurn , 2},//38
	{PARA_AccAndDecAntiPinch , 2},//39
	{PARA_PumpMotorEncoder , 21},//40
	{PARA_VehicleType , 69},//41
	{PARA_VehcileHeight , 0},//42
	{PARA_PressureSensorType , 1},//43
	{PARA_PitProtectFunc , 1},//44
	{PARA_AntiPinchFunc , 1},//45
	{PARA_ActAlmFunc , 1},//48
	{PARA_WeighFunc , 0},//49
	{PARA_ParallelValveReverseFunc , 1},//50
	{PARA_LowBatAlmFunc , 1},//51
	{PARA_LowVolAlmTime , 12},//52
	{PARA_LowVolShutDownTime , 30},//53
	{PARA_UpperCtlButSleep , 10},//54
	{PARA_LanguageType , 2},//55
	{PARA_BatteryType , 2},//56
	{PARA_SpeakerSync , 1},//57
	{PARA_LowerPumpType , 1},//58
	{PARA_PressureType , 0},//59
	{PARA_AngleSimulationLimit , 0},//60
	{PARA_LiftReverseFunc , 0},//61
	{PARA_FourPointWeightFunc , 0},//64
	{PARA_DriverType , 6},//65
	{PARA_PasswordLock , 1},//66
	{PARA_InAndOutFunc , 0},//67
	{PARA_HeartBeatQueryFunc , 3},//68
	{PARA_LowBatteryMode , 0},//69
	{PARA_AngleSensorSetting , 0},//70
	{PARA_TiltSwitchSetting , 1},//71
	{PARA_AngleSensorType , 0},//72
	{PARA_AnaLogLimitDetSwitch , 0},//73
	{PARA_IsNoLoadCalibration , 1},//74
	{PARA_IsOverLoadCalibration , 1},//75
	{PARA_SetDescentHeightValue , 1},//76
	{PARA_ReleaseBrake , 1},//77
	{PARA_SetOutHeight , 2},//80
	{PARA_AngleSimulationUpLimit , 2043},//81 �궨��
	{PARA_AngleSimulationDownLimit , 882},//82  �궨��
	{PARA_PumpDriveCurrentLimitRatio0 , 0},//83
	{PARA_PumpSpdAccRatio0 , 0},//84
	{PARA_PropDKp0 , 8192},//85
	{PARA_PropDKi0 , 4096},//86
	{PARA_PropDMaxCurrent0 , 1050},//87
	{PARA_PropDMinCurrent0 , 180},//88
	{PARA_PropDAccPeriod0 , 10},//89
	{PARA_PropDDitherPeriod0 , 40},//90
	{PARA_PropDDitherRatio0 , 5},//91
	{PARA_PropValveResistance0 , 20},//92
	{PARA_PumpDriveCurrentLimitRatio1 , 0},//96
	{PARA_PumpSpdAccRatio1 , 0},//97
	{PARA_PropDKp1 , 8192},//98
	{PARA_PropDKi1 , 4096},//99
	{PARA_PropDMaxCurrent1 , 1050},//100
	{PARA_PropDMinCurrent1 , 180},//101
	{PARA_PropDAccPeriod1 , 10},//102
	{PARA_PropDDitherPeriod1 , 40},//103
	{PARA_PropDDitherRatio1 , 5},//104
	{PARA_PropValveResistance1 , 20},//105
	{PARA_CanBaudRate , 250},//106
	{PARA_EmptyPressure , 3000},//107
	{PARA_FullPressure , 4000},//108
	{PARA_DriverFlag , 1022},//109
	{PARA_MinAngle , 300},//112
	{PARA_MaxAngle , 2010},//113
	{PARA_BatSocPalyBack , 0},//114
	{PARA_ValveType , 449},//115
	{PARA_AnticollisionFunc , 1},//116
	{PARA_ValueOpenLoopCurrent , 800},//117
	{PARA_ValueOpenPercentage , 80},//118
	{PARA_CanOpenNodeId ,44},//119
	{PARA_AngleValue0 , 0},//120
	//{PARA_AngleValue1 , 0},//121
//	{PARA_AngleValue2 , 0},//122
//	{PARA_AngleValue3 , 0},//123
//	{PARA_AngleValue4 , },//124 0507��������
	{PARA_AngleValue5 , 127},//125 �ֱ�ǰ
	{PARA_AngleValue6 , 0},//126
	{PARA_AngleValue7 , 129},//127 �ֱ���
	{PARA_Driver1Vol , 25680},//128
	{PARA_Driver2Vol , 25680},//129
	{PARA_Driver3Vol , 25680},//130
	{PARA_Driver4Vol , 25680},//131
	{PARA_Driver5Vol , 25680},//132
	{PARA_Driver6Vol , 25680},//133
	{PARA_Driver7Vol , 25680},//134
	{PARA_Driver8Vol , 25680},//135
	{PARA_Driver9Vol , 25680},//136
	{PARA_Driver10Vol, 25680},//137
	{PARA_LogLevel, 0},//138
	{PARA_LogModel, 0},//139
	{PARA_MotorMaxSpd, 3000},//140 �����жϵķ����ٶ���ֵ
	{PARA_Analog3DeadZoneMinVal, 5},//141
	{PARA_Analog3DeadZoneMaxVal, 30},//142
	{PARA_Analog3MidVal, 50},//143
	{PARA_ThrottleType, 0},//144
	{PARA_ThrottleFDeadZoneMinVal, 30},//145 ����ת��ʱ�ط���ֵ
	{PARA_ThrottleFDeadZoneMaxVal, 0},//146 �����ط���ֵ
	{PARA_ThrottleFMidVal, 4},//147 ǰ�������л�
	{PARA_ThrottleBDeadZoneMinVal, 0},//148 ��������
	{PARA_ThrottleBDeadZoneMaxVal, 1},//149 �µ���ʱ
	{PARA_ThrottleBMidVal, 15},//150  ˮƽ�ط�
	{PARA_BrakeType, 0},//151
	{PARA_BrakeFDeadZoneMinVal, 0},//152
	{PARA_BrakeFDeadZoneMaxVal, 0},//153
	{PARA_BrakeFMidVal, 0},//154
	{PARA_BrakeBDeadZoneMinVal, 0},//155
	{PARA_BrakeBDeadZoneMaxVal, 1},//156
	{PARA_BrakeBMidVal, 2},//157
//	{PARA_Gear1Spd, 0},//160
//	{PARA_Gear2Spd, 0},//161
//	{PARA_Gear3Spd, 0},//162
//	{PARA_Gear4Spd, 0},//163
	{PARA_RatioOfTransmission, 0},//164
	{PARA_MaintenancePeriod, 0},//165
//	{PARA_RemotePara, 2},//166
//	{PARA_PumpMotorGear1, 0},//167
//	{PARA_PumpMotorGear2, 0},//168
//	{PARA_TurnWithDecStartAngle, 0},//169
//	{PARA_TurnWithDecEndAngle, 0},//170
//	{PARA_AngleWithStartSpdPer, 0},//171
//	{PARA_AngleWithEndSpdPer, 0},//172
	{PARA_HourCountPowerOn, 0b1011},
};

static uint16_t Para0608C[][2] = {
	{PARA_FastDriveSpeed , 80},//0
	{PARA_SlowDriveSpeed , 50},//1
	{PARA_DriveSpeedAfterLift , 30},//2
	{PARA_LiftSpeed , 50},//3
	{PARA_MaxTurnSpeed , 60},//4
	{PARA_TurnPowerLimit , 30},//5
	{PAPA_DeadZoneAdjust , 10},//6
	{PARA_BrakeFastDrive , 10},//7
	{PARA_BrakeSlowDrive , 3},//8
	{PARA_BrakeDriveAfterLift , 5},//9
	{PARA_BrakeLift , 15},//10
	{PARA_BrakeLower , 22},//11
	{PARA_BrakeTurn , 2},//12
	{PARA_BrakeAntiPinch , 90},//13
	{PARA_LowerSpeed , 80},//16
	{PARA_OverLoadStabilityDelay , 1},//17
	{PARA_DynamicOverLoadPercent , 13},//18
	{PARA_StaticOverLoadPercent , 31},//19
	{PARA_MaxDifferencePercent , 35},//20
	{PARA_DriveMotorEncoder , 1},//21			��δʹ�ò���
	{PARA_MotorHighSpeedDeceRate , 10},//22		  �� δʹ�ò���
	{PARA_MotorLowSpeedDeceRate , 15},//23			��δʹ�ò���
	{PARA_VoiceAlarmVolume , 1},//24
	{PARA_CurveFastDrive , 10},//25
	{PARA_CurveSlowDrive , 2},//26
	{PARA_CurveDriveAfterLift , 2},//27
	{PARA_CurveLift , 5},//28
	{PARA_CurveLower , 2},//29
	{PARA_CurveTurn , 2},//32
	{PARA_AccAndDecFastDrive , 2},//33
	{PARA_AccAndDecSlowDrive , 2},//34
	{PARA_AccAndDecAfterLift , 2},//35
	{PARA_AccAndDecLift , 6},//36
	{PARA_AccAndDecLower , 8},//37
	{PARA_AccAndDecTurn , 2},//38
	{PARA_AccAndDecAntiPinch , 2},//39
	{PARA_PumpMotorEncoder , 21},//40
	{PARA_VehicleType , 69},//41
	{PARA_VehcileHeight , 0},//42
	{PARA_PressureSensorType , 1},//43
	{PARA_PitProtectFunc , 1},//44
	{PARA_AntiPinchFunc , 1},//45
	{PARA_ActAlmFunc , 1},//48
	{PARA_WeighFunc , 0},//49
	{PARA_ParallelValveReverseFunc , 1},//50
	{PARA_LowBatAlmFunc , 1},//51
	{PARA_LowVolAlmTime , 12},//52
	{PARA_LowVolShutDownTime , 30},//53
	{PARA_UpperCtlButSleep , 10},//54
	{PARA_LanguageType , 2},//55
	{PARA_BatteryType , 2},//56
	{PARA_SpeakerSync , 1},//57
	{PARA_LowerPumpType , 1},//58
	{PARA_PressureType , 0},//59
	{PARA_AngleSimulationLimit , 0},//60
	{PARA_LiftReverseFunc , 0},//61
	{PARA_FourPointWeightFunc , 0},//64
	{PARA_DriverType , 6},//65
	{PARA_PasswordLock , 1},//66
	{PARA_InAndOutFunc , 0},//67
	{PARA_HeartBeatQueryFunc , 3},//68
	{PARA_LowBatteryMode , 0},//69
	{PARA_AngleSensorSetting , 0},//70
	{PARA_TiltSwitchSetting , 1},//71
	{PARA_AngleSensorType , 0},//72
	{PARA_AnaLogLimitDetSwitch , 0},//73
	{PARA_IsNoLoadCalibration , 1},//74
	{PARA_IsOverLoadCalibration , 1},//75
	{PARA_SetDescentHeightValue , 1},//76
	{PARA_ReleaseBrake , 1},//77
	{PARA_SetOutHeight , 2},//80
	{PARA_AngleSimulationUpLimit , 2043},//81 �궨��
	{PARA_AngleSimulationDownLimit , 882},//82  �궨��
	{PARA_PumpDriveCurrentLimitRatio0 , 0},//83
	{PARA_PumpSpdAccRatio0 , 0},//84
	{PARA_PropDKp0 , 8192},//85
	{PARA_PropDKi0 , 4096},//86
	{PARA_PropDMaxCurrent0 , 5},//87
	{PARA_PropDMinCurrent0 , 5},//88
	{PARA_PropDAccPeriod0 , 10},//89
	{PARA_PropDDitherPeriod0 , 40},//90
	{PARA_PropDDitherRatio0 , 5},//91
	{PARA_PropValveResistance0 , 20},//92
	{PARA_PumpDriveCurrentLimitRatio1 , 0},//96
	{PARA_PumpSpdAccRatio1 , 0},//97
	{PARA_PropDKp1 , 8192},//98
	{PARA_PropDKi1 , 4096},//99
	{PARA_PropDMaxCurrent1 , 1050},//100
	{PARA_PropDMinCurrent1 , 180},//101
	{PARA_PropDAccPeriod1 , 10},//102
	{PARA_PropDDitherPeriod1 , 40},//103
	{PARA_PropDDitherRatio1 , 5},//104
	{PARA_PropValveResistance1 , 20},//105
	{PARA_CanBaudRate , 250},//106
	{PARA_EmptyPressure , 3000},//107
	{PARA_FullPressure , 4000},//108
	{PARA_DriverFlag , 766},//109
	{PARA_MinAngle , 300},//112
	{PARA_MaxAngle , 2010},//113
	{PARA_BatSocPalyBack , 0},//114
	{PARA_ValveType , 449},//115
	{PARA_AnticollisionFunc , 1},//116
	{PARA_ValueOpenLoopCurrent , 800},//117
	{PARA_ValueOpenPercentage , 80},//118
	{PARA_CanOpenNodeId ,44},//119
	{PARA_AngleValue0 , 0},//120
	//{PARA_AngleValue1 , 0},//121
	//{PARA_AngleValue2 , 0},//122
	//{PARA_AngleValue3 , 0},//123
//	{PARA_AngleValue4 , },//124 0507��������
	{PARA_AngleValue5 , 127},//125 �ֱ�ǰ
	{PARA_AngleValue6 , 0},//126
	{PARA_AngleValue7 , 129},//127 �ֱ���
	{PARA_Driver1Vol , 25680},//128
	{PARA_Driver2Vol , 25680},//129
	{PARA_Driver3Vol , 25680},//130
	{PARA_Driver4Vol , 25680},//131
	{PARA_Driver5Vol , 25680},//132
	{PARA_Driver6Vol , 25680},//133
	{PARA_Driver7Vol , 25680},//134
	{PARA_Driver8Vol , 25680},//135
	{PARA_Driver9Vol , 25680},//136
	{PARA_Driver10Vol, 25680},//137
	{PARA_LogLevel, 0},//138
	{PARA_LogModel, 0},//139
	{PARA_MotorMaxSpd, 3000},//140 �����жϵķ����ٶ���ֵ
	{PARA_Analog3DeadZoneMinVal, 5},//141
	{PARA_Analog3DeadZoneMaxVal, 30},//142
	{PARA_Analog3MidVal, 50},//143
	{PARA_ThrottleType, 0},//144
	{PARA_ThrottleFDeadZoneMinVal, 30},//145 ����ת��ʱ�ط���ֵ
	{PARA_ThrottleFDeadZoneMaxVal, 0},//146 �����ط���ֵ
	{PARA_ThrottleFMidVal, 4},//147 ǰ�������л�
	{PARA_ThrottleBDeadZoneMinVal, 0},//148 ��������
	{PARA_ThrottleBDeadZoneMaxVal, 1},//149 �µ���ʱ
	{PARA_ThrottleBMidVal, 15},//150  ˮƽ�ط�
	{PARA_BrakeType, 0},//151
	{PARA_BrakeFDeadZoneMinVal, 0},//152
	{PARA_BrakeFDeadZoneMaxVal, 0},//153
	{PARA_BrakeFMidVal, 0},//154
	{PARA_BrakeBDeadZoneMinVal, 0},//155
	{PARA_BrakeBDeadZoneMaxVal, 1},//156
	{PARA_BrakeBMidVal, 2},//157
//	{PARA_Gear1Spd, 0},//160
//	{PARA_Gear2Spd, 0},//161
//	{PARA_Gear3Spd, 0},//162
//	{PARA_Gear4Spd, 0},//163
	{PARA_RatioOfTransmission, 0},//164
	{PARA_MaintenancePeriod, 0},//165
//	{PARA_RemotePara, 2},//166
//	{PARA_PumpMotorGear1, 0},//167
//	{PARA_PumpMotorGear2, 0},//168
//	{PARA_TurnWithDecStartAngle, 0},//169
//	{PARA_TurnWithDecEndAngle, 0},//170
//	{PARA_AngleWithStartSpdPer, 0},//171
//	{PARA_AngleWithEndSpdPer, 0},//172
	{PARA_HourCountPowerOn, 0b1011},//173 	ǰ��bit 00�룬01�֣�10 6�֣�11Сʱ��bit1��������ʱ1���ϵ��ʱ0��bit0������Сʱ�ƣ�
};
static uint16_t Para0808[][2] = {
	{PARA_FastDriveSpeed , 90},//0
	{PARA_SlowDriveSpeed , 90},//1
	{PARA_DriveSpeedAfterLift , 30},//2
	{PARA_LiftSpeed , 60},//3
	{PARA_MaxTurnSpeed , 60},//4
	{PARA_TurnPowerLimit , 30},//5
	{PAPA_DeadZoneAdjust , 10},//6
	{PARA_BrakeFastDrive , 1},//7
	{PARA_BrakeSlowDrive , 1},//8
	{PARA_BrakeDriveAfterLift , 5},//9
	{PARA_BrakeLift , 20},//10
	{PARA_BrakeLower , 20},//11
	{PARA_BrakeTurn , 2},//12
	{PARA_BrakeAntiPinch , 85},//13
	{PARA_LowerSpeed , 90},//16
	{PARA_OverLoadStabilityDelay , 1},//17
	{PARA_DynamicOverLoadPercent , 13},//18
	{PARA_StaticOverLoadPercent , 31},//19
	{PARA_MaxDifferencePercent , 35},//20
	{PARA_DriveMotorEncoder , 1},//21			��δʹ�ò���
	{PARA_MotorHighSpeedDeceRate , 2},//22		  �� δʹ�ò���
	{PARA_MotorLowSpeedDeceRate , 2},//23			��δʹ�ò���
	{PARA_VoiceAlarmVolume , 1},//24
	{PARA_CurveFastDrive , 10},//25
	{PARA_CurveSlowDrive , 2},//26
	{PARA_CurveDriveAfterLift , 2},//27
	{PARA_CurveLift , 5},//28
	{PARA_CurveLower , 5},//29
	{PARA_CurveTurn , 2},//32
	{PARA_AccAndDecFastDrive , 2},//33
	{PARA_AccAndDecSlowDrive , 2},//34
	{PARA_AccAndDecAfterLift , 2},//35
	{PARA_AccAndDecLift , 4},//36
	{PARA_AccAndDecLower , 4},//37
	{PARA_AccAndDecTurn , 2},//38
	{PARA_AccAndDecAntiPinch , 2},//39
	{PARA_PumpMotorEncoder , 21},//40
	{PARA_VehicleType , 69},//41
	{PARA_VehcileHeight , 0},//42
	{PARA_PressureSensorType , 1},//43
	{PARA_PitProtectFunc , 1},//44
	{PARA_AntiPinchFunc , 1},//45
	{PARA_ActAlmFunc , 1},//48
	{PARA_WeighFunc , 0},//49
	{PARA_ParallelValveReverseFunc , 1},//50
	{PARA_LowBatAlmFunc , 1},//51
	{PARA_LowVolAlmTime , 12},//52
	{PARA_LowVolShutDownTime , 30},//53
	{PARA_UpperCtlButSleep , 10},//54
	{PARA_LanguageType , 2},//55
	{PARA_BatteryType , 2},//56
	{PARA_SpeakerSync , 1},//57
	{PARA_LowerPumpType , 1},//58
	{PARA_PressureType , 0},//59
	{PARA_AngleSimulationLimit , 0},//60
	{PARA_LiftReverseFunc , 0},//61
	{PARA_FourPointWeightFunc , 0},//64
	{PARA_DriverType , 6},//65
	{PARA_PasswordLock , 1},//66
	{PARA_InAndOutFunc , 0},//67
	{PARA_HeartBeatQueryFunc , 3},//68
	{PARA_LowBatteryMode , 0},//69
	{PARA_AngleSensorSetting , 0},//70
	{PARA_TiltSwitchSetting , 1},//71
	{PARA_AngleSensorType , 0},//72
	{PARA_AnaLogLimitDetSwitch , 0},//73
	{PARA_IsNoLoadCalibration , 1},//74
	{PARA_IsOverLoadCalibration , 1},//75
	{PARA_SetDescentHeightValue , 1},//76
	{PARA_ReleaseBrake , 1},//77
	{PARA_SetOutHeight , 2},//80
	{PARA_AngleSimulationUpLimit , 2043},//81 �궨��
	{PARA_AngleSimulationDownLimit , 882},//82  �궨��
	{PARA_PumpDriveCurrentLimitRatio0 , 0},//83
	{PARA_PumpSpdAccRatio0 , 0},//84
	{PARA_PropDKp0 , 8192},//85
	{PARA_PropDKi0 , 4096},//86
	{PARA_PropDMaxCurrent0 , 5},//87
	{PARA_PropDMinCurrent0 , 5},//88
	{PARA_PropDAccPeriod0 , 10},//89
	{PARA_PropDDitherPeriod0 , 40},//90
	{PARA_PropDDitherRatio0 , 5},//91
	{PARA_PropValveResistance0 , 20},//92
	{PARA_PumpDriveCurrentLimitRatio1 , 0},//96
	{PARA_PumpSpdAccRatio1 , 0},//97
	{PARA_PropDKp1 , 8192},//98
	{PARA_PropDKi1 , 4096},//99
	{PARA_PropDMaxCurrent1 , 1020},//100
	{PARA_PropDMinCurrent1 , 180},//101
	{PARA_PropDAccPeriod1 , 10},//102
	{PARA_PropDDitherPeriod1 , 40},//103
	{PARA_PropDDitherRatio1 , 5},//104
	{PARA_PropValveResistance1 , 20},//105
	{PARA_CanBaudRate , 250},//106
	{PARA_EmptyPressure , 3000},//107
	{PARA_FullPressure , 4000},//108
	{PARA_DriverFlag , 1022},//109
	{PARA_MinAngle , 300},//112
	{PARA_MaxAngle , 2010},//113
	{PARA_BatSocPalyBack , 0},//114
	{PARA_ValveType , 449},//115
	{PARA_AnticollisionFunc , 1},//116
	{PARA_ValueOpenLoopCurrent , 800},//117
	{PARA_ValueOpenPercentage , 80},//118
	{PARA_CanOpenNodeId ,44},//119
	{PARA_AngleValue0 , 0},//120
//	{PARA_AngleValue1 , 0},//121
//	{PARA_AngleValue2 , 0},//122
//	{PARA_AngleValue3 , 0},//123
//	{PARA_AngleValue4 , 0},//124 0507��������
	{PARA_AngleValue5 , 127},//125 �ֱ�ǰ
	{PARA_AngleValue6 , 0},//126
	{PARA_AngleValue7 , 129},//127 �ֱ���
	{PARA_Driver1Vol , 25680},//128
	{PARA_Driver2Vol , 25680},//129
	{PARA_Driver3Vol , 25680},//130
	{PARA_Driver4Vol , 25680},//131
	{PARA_Driver5Vol , 25680},//132
	{PARA_Driver6Vol , 25680},//133
	{PARA_Driver7Vol , 25680},//134
	{PARA_Driver8Vol , 25680},//135
	{PARA_Driver9Vol , 25680},//136
	{PARA_Driver10Vol, 25680},//137
	{PARA_LogLevel, 0},//138
	{PARA_LogModel, 0},//139
	{PARA_MotorMaxSpd, 3000},//140 �����жϵķ����ٶ���ֵ
	{PARA_Analog3DeadZoneMinVal, 5},//141
	{PARA_Analog3DeadZoneMaxVal, 30},//142
	{PARA_Analog3MidVal, 50},//143
	{PARA_ThrottleType, 0},//144
	{PARA_ThrottleFDeadZoneMinVal, 30},//145 ����ת��ʱ�ط���ֵ
	{PARA_ThrottleFDeadZoneMaxVal, 0},//146 �����ط���ֵ
	{PARA_ThrottleFMidVal, 4},//147 ǰ�������л�
	{PARA_ThrottleBDeadZoneMinVal, 0},//148 ��������
	{PARA_ThrottleBDeadZoneMaxVal, 1},//149 �µ���ʱ
	{PARA_ThrottleBMidVal, 15},//150  ˮƽ�ط�
	{PARA_BrakeType, 0},//151
	{PARA_BrakeFDeadZoneMinVal, 0},//152
	{PARA_BrakeFDeadZoneMaxVal, 0},//153
	{PARA_BrakeFMidVal, 0},//154
	{PARA_BrakeBDeadZoneMinVal, 0},//155
	{PARA_BrakeBDeadZoneMaxVal, 1},//156
	{PARA_BrakeBMidVal, 2},//157
//	{PARA_Gear1Spd, 0},//160
//	{PARA_Gear2Spd, 0},//161
//	{PARA_Gear3Spd, 0},//162
//	{PARA_Gear4Spd, 0},//163
	{PARA_RatioOfTransmission, 0},//164
	{PARA_MaintenancePeriod, 0},//165
//	{PARA_RemotePara, 2},//166
//	{PARA_PumpMotorGear1, 0},//167
//	{PARA_PumpMotorGear2, 0},//168
//	{PARA_TurnWithDecStartAngle, 0},//169
//	{PARA_TurnWithDecEndAngle, 0},//170
//	{PARA_AngleWithStartSpdPer, 0},//171
//	{PARA_AngleWithEndSpdPer, 0},//172
	{PARA_HourCountPowerOn, 0b1011},
	
};



static void vMultiParaChange(uint8_t u8Length,uint16_t (*u16Array)[2])
{
	static uint8_t u8SaveState;
	static uint8_t i;
	i32SetPara(PARA_PressureVlaue1,1);
	while(1)
	{		
		u8SaveState = 0;
		for(i = 0;i < u8Length; i ++)
		{
				if(u16SaveParaToEeprom(u16Array[i][0],u16Array[i][1]))
				{
					u8SaveState++;					
				}
		}
		if(0 == u8SaveState)
			break;
	}
}

static void vMcuParaRevProc(uint8_t *u8Data, uint16_t u16Length)
{
	uint8_t u8DataArray[8];
	
	memcpy(u8DataArray,u8Data,8);
	int16_t i16DataSend = 0;
	if(u8DataArray[0] == 0x60)
		McuParaIdx++;
	vResetNetTimer(TIMER_AlarmDelay);
}
static void vMCUParaWrite(uint8_t u8Length,uint16_t (*u16Array)[2])
{
	static uint8_t u8Vector[8];
		i32SetPara(PARA_PressureVlaue1,2);
		memset(u8Vector,0,sizeof(u8Vector));
		u8Vector[0] = 0x23;
		u8Vector[1] = u16Array[McuParaIdx][0] & 0xFF;
		u8Vector[2] = 0;
		u8Vector[3] = 0;
		u8Vector[4] = u16Array[McuParaIdx][1] & 0xFF;
		u8Vector[5] = (u16Array[McuParaIdx][1] >> 8) & 0xFF;
		u8Vector[6] = 0;
		u8Vector[7] = 0;
		
		vQueryMcuPara(u8Vector,8);	
}
	

static void vDriverBanParallePump()
{
	uint16_t u16TmpPara;
	u16TmpPara = i32GetPara(PARA_DriverFlag);
	if((u16TmpPara >> 8) & 0x01)
	{
		u16TmpPara &= ~ (1 << 8);
		u16SaveParaToEeprom(PARA_DriverFlag,u16TmpPara);
	}
}	
static void vChangeMcuPara()
{
	static uint16_t McuParaIdxOld;
	if((McuParaIdx == 0)||(McuParaIdxOld != McuParaIdx))
	{
		McuParaIdxOld = McuParaIdx;
		if	((12 == i32GetPara(PARA_CARCODE))  //-LS0407H
			|| (40 == i32GetPara(PARA_CARCODE))  //-LS0507HM
			|| (41 == i32GetPara(PARA_CARCODE))) //LS0507HM-Li
			vMCUParaWrite((sizeof(MCU_Para5m)/sizeof(MCU_Para5m[0])),MCU_Para5m);
		else
			vMCUParaWrite((sizeof(MCU_Para6m)/sizeof(MCU_Para6m[0])),MCU_Para6m);
	}
}
static void vChangeCarTypePara(uint16_t u16Carcode)
{

	{
		switch (u16Carcode) 
		{
			case 12:  //0507H
				vMultiParaChange((sizeof(Para0507)/sizeof(Para0507[0])), Para0507); 
				break;
			case 13:  //0607H
				vMultiParaChange((sizeof(Para0608C)/sizeof(Para0608C[0])), Para0608C);
				break;  
			case 14:  //0608H .
				vMultiParaChange((sizeof(Para0608C)/sizeof(Para0608C[0])), Para0608C); 
				break;
			case 15:  // 0808				
				vMultiParaChange((sizeof(Para0808)/sizeof(Para0808[0])), Para0808); 
				break;
			case 16:  //0812H
				vMultiParaChange((sizeof(Para0808)/sizeof(Para0808[0])), Para0808); 
				break;
			case 17: //1012H	
				vMultiParaChange((sizeof(Para1212Dif)/sizeof(Para1212Dif[0])), Para1212Dif); 
				break;
			case 18:  // 1212
				vMultiParaChange((sizeof(Para1212Dif)/sizeof(Para1212Dif[0])), Para1212Dif);
				break;
			case 40:  // 0507hm
				vMultiParaChange((sizeof(Para0507)/sizeof(Para0507[0])), Para0507); 
				break;
			case 41:	//0507hmli
				vMultiParaChange((sizeof(Para0507)/sizeof(Para0507[0])), Para0507);
				u16SaveParaToEeprom(PARA_BatteryType,0);//�޸ĵ������Ϊ﮵��
				break;
			case 42:	//0608 C				
				vMultiParaChange((sizeof(Para0608C)/sizeof(Para0608C[0])), Para0608C);
				break;
			case 43:	//0808 C
				vMultiParaChange((sizeof(Para0808)/sizeof(Para0808[0])), Para0808);
				break;                                                             
			case 44://1012 C
				vMultiParaChange((sizeof(Para1212Dif)/sizeof(Para1212Dif[0])), Para1212Dif);
				break;	
			case 45://1212 C	
				vMultiParaChange((sizeof(Para1212Dif)/sizeof(Para1212Dif[0])), Para1212Dif);
				break;		
			default:
				break;
		}
	}
}


static void vUserSetsInit()
{
	sgUserSets.u16Data = i32GetPara(PARA_USERSETS);
	if(0 == sgUserSets.b1StartFast)//��������
		sgLimit.b1Slow = 1;
	if(1 == sgUserSets.b1StartWithMode)
		 u8PcuMode	= MOVE_MODE;
	
	if(1 == sgUserSets.b1Err18LiftAllow)
	{
		sgErrCodeInfo[117].b1NoUp = 0;
	}
	if(11 != i32GetPara(PARA_HourCountPowerOn))//24.3.12�̶�Сʱ��Ϊ������ʱ
	{
		u16SaveParaToEeprom(PARA_HourCountPowerOn,11);
	}
	
	//24.3.18������������
	{
		uint16_t u16tmp;
		u16tmp = i32GetPara(PARA_DriverFlag);
		if(FunctionDisable != gUserInfo.u8AngleSimulationLimit)
		{
			if(0 == (u16tmp & (1<<10)))
			{
				u16tmp |= (1<<10);
				u16SaveParaToEeprom(PARA_DriverFlag,u16tmp);
			}
		}
//		else//���ر�  //24.5.08 ������������������ر�ģ�������
//		{
//			if(0 != (u16tmp & (1<<10)))
//			{
//				u16tmp &=~ (1<<10);
//				u16SaveParaToEeprom(PARA_DriverFlag,u16tmp);
//			}			
//		}
		if((FunctionDisable != gUserInfo.u8WeighFunc)
			||(FunctionDisable != gUserInfo.u8FourPointWeightFunc))	//���������ʹ��	
		{
			if(0 == (u16tmp & (1<<11)))
			{
				u16tmp |= (1<<11);
				u16SaveParaToEeprom(PARA_DriverFlag,u16tmp);
			}
		}
		else
		{
			if(0 != (u16tmp & (1<<11)))
			{
				u16tmp &=~ (1<<11);
				u16SaveParaToEeprom(PARA_DriverFlag,u16tmp);
			}
		}
	}
	{
		uint16_t u16SftCode;
		u16EepromRead(SOFTWARE_VERSION_STATE,&u16SftCode,1);
		//i32SetPara(PARA_LoadRate,u16SftCode);
//		i32SetPara(PARA_CalibrationStatus,SOFTWARE_VERSION_CODE);
		if(SOFTWARE_VERSION_CODE != u16SftCode)//��������汾ʱˢ�²�����
		{
			if(0 == u16SftCode || 0xFFFF == u16SftCode)
			{
					//117�汾֮ǰ��û�洢�汾��Ϣ
					//118�汾��Ҫ���������λ�Ƕȣ���ֵ���رճ��ع��ܼ��ĵ���ع���
				u16SaveParaToEeprom(PARA_LOWLIMIT_RANGE,5);
				u16SaveParaToEeprom(PARA_LOWLIMIT_ANGLE,490);
				u16SaveParaToEeprom(PARA_FourPointWeightFunc,FunctionDisable);
				u16SaveParaToEeprom(PARA_WeighFunc,FunctionDisable);		
			}
			else
			{ /*�Բ�ͬ�汾�������д���*/
				/*121�汾���£������λ����Ϊ490*/
				if(u16SftCode < 0x0121 )
				{
					u16SaveParaToEeprom(PARA_LOWLIMIT_ANGLE,490);
				}
			}
			/*��������ز���֮�󣬸��±���İ汾��*/
			u16EepromWrite(SOFTWARE_VERSION_STATE,SOFTWARE_VERSION_CODE,1);			
		}
	}
}

>>>>>>> .r49
void vUserEcuInit(void)
{	
	vClearUpdateFlag();
	vUserSetsInit();
	vUserParaInit();
	vSetPdoPara(sgPdoPara);
	vPcuErrRegister(vPcuErrProc);	
	vPcuRevRegister(vPcuRevProc);
	vPcuSendRegister(vPcuSendProc);
	
	vMcuParaRevRegister(vMcuParaRevProc);
	vMstRevRegister(vMstRevProc);
	vMstSendRegister(vMstSendProc);
	vBeepRegister(vBeepCallBack);
	
	vAiErrReg(vAiErrCallBack);
	vDoPwmErrReg(vDoPwmErrCallBack);
	vPropErrReg(vPropErrCallBack);
	
	vPressureSensorReg(vPressureCallBack);
	
	vAlarmLampRegister(vAlarmLampCallBack);
	vAlarmLampSetPeriod(60);
	vAlarmLampSetOpenePeriod(120);//����Ϊ�ϱ����ڵı���
<<<<<<< .mine
	u32HourCnt = u32HourCountRead();
	vRemoteLockInit();
	vRemoteUnlockKey();
	vTboxParaInit();
	
||||||| .r31
//	vTboxParaInit();
	
=======
	u32HourCnt = u32HourCountRead();
	vRemoteLockInit();
	vRemoteUnlockKey();
	vTboxParaInit();

	{
		sgPcuHandle.i8PositiveValue = i32GetPara(PARA_HANDLEMAX);
		sgPcuHandle.i8MiddleValue = i32GetPara(PARA_HANDLEMID);
		sgPcuHandle.i8NegativeValue = i32GetPara(PARA_HANDLEMIN);
	}
	#ifdef LIUGONG_TEST
	u16UpDownCnt = 0;//���������½�����
	#endif
>>>>>>> .r49
	
	{
		sgPcuHandle.i8PositiveValue = i32GetPara(PARA_HANDLEMAX);
		sgPcuHandle.i8MiddleValue = i32GetPara(PARA_HANDLEMID);
		sgPcuHandle.i8NegativeValue = i32GetPara(PARA_HANDLEMIN);
	}
	#ifdef LIUGONG_TEST
	u16UpDownCnt = 0;//���������½�����
	#endif
	vErrCodeInit(sgErrCodeInfo,(sizeof(sgErrCodeInfo))/(sizeof(sgErrCodeInfo[0])));
	vTboxDownLoadDataInit();
	
	vCanIdLostReg(0x51,60000,vCanLostProc);
	vCanIdLostReg(0x166,180000,vCanLostProc);  //���ݼ���TBOX�ն�3minδ�յ��������ı�������������
	

	memset(&sgActLogic, 0, sizeof(sgActLogic));
<<<<<<< .mine
	McuParaIdx = 0;//MCU��������
||||||| .r31
=======
	McuParaIdx = 0;//MCU�������� 
>>>>>>> .r49
	
<<<<<<< .mine
||||||| .r31
	vUserParaInit();
=======
	u16EepromRead(CARCODE_HISTORY,&u16CarCodeOld,1);
	
	
	uint16_t u16Carcode = i32GetPara(PARA_CARCODE);					
	if(u16Carcode == 0)//�ó���Ĭ��Ϊ5m�ĳ���
	{
		vChangeCarTypePara(u16Carcode);
		u16SaveParaToEeprom(PARA_CARCODE,40);//Ĭ��5m
		i32SetPara(PARA_CARCODE,40);
	}
	
>>>>>>> .r49
	vHourCountInit();


//	__disable_irq();
////	gCanSendPdoInfo.CanSend260Info.u8HourCountL = u32HourCount & 0xFF;
////	gCanSendPdoInfo.CanSend260Info.u8HourCountH = (u32HourCount >> 8) & 0xFF;

//	__enable_irq();		

	
	vSetNetTimer(TIMER_EcuPowerOn, ECU_POWERON_DELAY_TIME);	/**/
	

}
/*******************************************************************************
* Name: void vUserEcuProc(void)
* Descriptio: UserEcu�û���������
* Input: NULL
* Output: NULL  
*******************************************************************************/
/*���Ա���*/

static void vRelayCanSendFram233()
{
	static tCanFrame CanID233 = 
	{.u16DataLength = 8,.u32ID = 0x233,.u8Rtr = 0};
	static uint8_t u8SendCnt;
	if(u8SendCnt <= (50 / 5))
	{
		u8SendCnt ++;
	}
	else
	{
		u8SendCnt = 0;
		if(((sgActLogic.b1LiftUpAct||sgActLogic.b1LiftDownAct) && sgSwiInput.b1PcuSwi == 0)
			||(sgActLogic.b1ForwardAct||sgActLogic.b1BackwardAct||sgActLogic.b1TurnLeft||sgActLogic.b1TurnRight)
		  ||(1 == sgSwiInput.b1PitSwi || sgSwiInput.b1LowestState))
			CanID233.u8Data[0] = 1;
		else
			CanID233.u8Data[0] = 0;

		#ifdef LIUGONG_TEST
		CanID233.u8Data[6] = (u16UpDownCnt>>8)&0xff; //����ֵ��8λ
		CanID233.u8Data[7] = (u16UpDownCnt)&0xff;    //����ֵ��8λ
		#endif
		i32CanWrite(Can0,&CanID233);
	}	
}


void vUserEcuProc(void)
{
	uint8_t u8ErrCode = 0;
	static uint16_t u16SecCnt = 0;
	static uint32_t u32RentalCnt = 0;
	static uint16_t u16HMISendCnt = 0;

	if(true == u8GetNetTimerOverFlag(TIMER_EcuPowerOn))
	{
		u8EcuProcFlag = 1;
		vKillNetTimer(TIMER_EcuPowerOn);
	}
	
	if ((false == u8GetNetTimerOverFlag(TIMER_EcuPowerOn)) && (true == u8GetNetTimerStartFlag(TIMER_EcuPowerOn)))
	{
		vSwiInitCheck();
		vUserSetsInit();
	}
	
	if(1 == u8EcuProcFlag)
	{
		vSwiMonitor();		
		vEcuSetBeepPeriod();
		vModeChange();
		vBatteryManage();
		vTBoxProc();
		vPcuSleep();
		vRemoteLockProc();
		vBlinkBeepProc();
		vActionProcess();
		vTboxDownLoadData();
		vErrProc();
		vRelayCanSendFram233();
		
		vResponceProc();
		
		if(u16HMISendCnt<20)
			u16HMISendCnt++;
		else
		{
			HMISend();
			u16HMISendCnt = 0;
		}
		
		
		{
			u32HourCnt = u32HourCountProc((sgActLogic.u8Data));//��0ʱ��ʼ��ʱ
		}
//		i32SetPara(PARA_TurnLeftValveCurrent,u32HourCnt);
		__disable_irq();
		__enable_irq();
		PCUStateRecord.PcuKeyInfo.u8data = sgPcuInfo.PcuKeyInfo.u8data;
	}

	vWdgSetFun(WDG_USER_BIT);
}

#endif