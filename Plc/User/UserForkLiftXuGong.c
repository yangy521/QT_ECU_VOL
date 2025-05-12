/*******************************************************************************
*通锟矫筹拷锟斤拷锟斤拷* 						   *
* Author: QExpand; ShiJin                                                        *
* Date: 2023/10/32    														   *
* Revision:	 														 		   *
*******************************************************************************/
#include "UserCanComm.h"
#include "UserForkLiftXuGong.h"
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
#include "CanBlock.h"


#if (USER_TYPE == USER_FORKLIFT_XUGONG)

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
{		/*MCU故障除个别，其余统一报81*/
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//1		反馈超速
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//2		内核运行错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//3		控制器长时间过载
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//4		位置超差
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//5		电子齿轮位置指令变化超电机最大速度
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//6		速度模式，速度指令超过电机最大转速
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//7		转矩模式，转矩指令超过最大转矩
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//8		速度传感器错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//9		速度传感器方向错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//10	无
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//11	电机2分钟最大电流
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//12	电机过流
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//13	母线电容充电故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 10,},	//14	主接触器故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//15	制动器故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//16	电池电压低
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//17	电池电压高
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//18	功率板过温
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//19	电机高温
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//20	电位计短路或电压
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//21	主接触器触电熔接
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//22	5v输出故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//23	id监
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//24	主接触器驱动故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//25	功率模块故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//26	Can通讯故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//27	电压高于最大电压2V
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//28	无
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//29	电机电阻异常
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//30	回中错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//31	电压轻度过低
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//32	功率板轻度过温
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//33	功率板低温
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//34	电机轻度高温
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//35	12v故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//36	DO3故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//37	DO4故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//38	eeprom参数读写错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//39	参数编号错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//40	上电IO异常
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//41	电量低于20%
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//42	电量低于10%
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},	//43	电压达到限速阈值
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
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 0,},	//60	DO10报警在用户层报警
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//61		DO11
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//62		DO12
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//63		DO13
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//64		DO14
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//65		DO15		
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 27,},//66 		下降阀0		
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 55,},//67 		下降阀1
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//68		预留
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//69		预留
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//70
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 32,},	//71	模拟量1
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 31,},	//72	模拟量2
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//73	模拟量3
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//74	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//75
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//76
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//77	编码器错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//78
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//79
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//80
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//81	主从MCU故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//82	主从MCU脉冲异常
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//83	看门狗异常
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//84	PCU通讯错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//85	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//86	写参数错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//87	读参数错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//88	参数超限
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
		
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 1,},	//101	初始化
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1,	.b8UserErr = 2,},	//102	通信错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 3,},	//103	无效选项
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 4,},	//104	内存数据错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 5,},	//105	锂电掉线
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 6,},	//106	上电时按下起升
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 7,},	//107	上电时按下龟速
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 8,},	//108	上电时按下行走
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 9,},	//109	GPS掉线
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 10,},//110	主接触器连接错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 11,},//111	无
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 12,},//112	启动时下控错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 13,},//113	BMS电池温差大-2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 14,},//114	BMS电池温度高-1
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 15,},//115 BMS-放电温度过高2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 16,},//116	BMS-放电电流过高1
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 17,},//117	BMS-放电电流过高2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 18,},//118	坑洞保护错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 19,},//119	BMS-总电压过低1
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 20,},//120	BMS-总电压过低2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 21,},//121	BMS-单体电压过低1
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 22,},//122	BMS-单体电压过低2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 23,},
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 24,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 25,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 26,},
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 27,},//127	下降阀2错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 28,},
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 29,},
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 30,},//130	BMS-电池压差过大
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 31,},//131	压力传感器错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1,	.b8UserErr = 32,},//132	角度传感器错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 33,},//133	电池类型错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 34,},
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1,	.b8UserErr = 35,},//135	称重标定反
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 36,},//136	电池电量低一级报警
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 37,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1,	.b8UserErr = 38,},//138	未标定完成或标定失败
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1,	.b8UserErr = 39,},//139	通信故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 40,},
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 41,},//141	平台一级锁车
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 42,},//142	启动时，平台向左转向按钮按下错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 43,},//143	启动时，平台向右转向按钮按下错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1,	.b8UserErr = 44,},//144	平台二级锁车
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 45,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1,	.b8UserErr = 46,},//146	启动时，平台手柄使能开关按钮按下错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 47,},//147	启动时，平台手柄不在中位错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 48,},
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 49,},//149	心跳锁车
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0,	.b8UserErr = 50,},//
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 51,},
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 52,},//152	前进阀
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 53,},//153	后退阀
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 54,},//154	起升阀
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 55,},//155	下降阀
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 56,},//156	右转阀
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 57,},//157	左转阀
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 58,},//158	闪光灯
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 59,},//159	蜂鸣器
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 60,},//160	驱动器故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 61,},//161	电流传感器故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 62,},//162	驱动器硬件故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 63,},//163	泵电机开路
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 64,},//164	左电机开路
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 65,},//165	控制电压5V故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 66,},//166	动作时，检测到左转阀开路或短路
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 67,},//167	控制电压12V故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 68,},//168	电池低电量二级报警
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 69,},//169	高零位电流错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 70,},//170	驱动器母线电压过高故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 71,},//171	预充故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 72,},//172	驱动器母线电压过低故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 73,},//173	驱动器低温故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 74,},//174	驱动器高温一级故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 1, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 75,},//175	泵电机温度一级故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 76,},//176	泵电机编码器故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 77,},//177	电机编码错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 78,},//178	泵电机过流类故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 79,},//179	泵电机温度二级故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 80,},//180	超过 80%负载报警
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 81,},//181	驱动器温度二级故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 82,},//182	右刹车故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 83,},//183	左刹车故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 84,},//184	泵电机堵转、失速
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 85,},//185	左牵引电机堵转、失速
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 86,},//186	由牵引电机堵转、失速
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 87,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 88,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 89,},//189	驱动器运行时间过长故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 90,},//190	超过 90%负载报警
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 91,},//191	左电机电流过流故障	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 92,},//192	右电机电流过流故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 93,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 94,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 95,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 96,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 97,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 98,},	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 0, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 99,},//199	超过 99%负载报警
	{.b1ErrCode = 0, .b1NoAct = 1, .b1NoPcu = 1, .b1Gear1Spd = 1, .b1NoDwon = 1, .b1NoTurnLeft = 1, .b1NoTurnRight = 1, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 100,},//200	平台超载报警
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 1, .b1NoForWard = 1, .b8UserErr = 101,},//201	机器倾斜超过安全限定错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1NoPcu = 0, .b1Gear1Spd = 0, .b1NoDwon = 0, .b1NoTurnLeft = 0, .b1NoTurnRight = 0, .b1NoUp = 1, .b1NoBackWard = 0, .b1NoForWard = 0, .b8UserErr = 39,},//202	碰撞保护，显示39
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
	int8_t i8HandleRaw;
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




xCanSendPdoInfo gCanSendPdoInfo ;
xCanRevPdoInfo gCanRevPdoInfo;	



//输入
static xSwiInput sgSwiInput;
static xPCUInfo sgPcuInfo;
static xPCUInfo PCUStateRecord;

static uint32_t u32HourCnt = 0;
static xLimit sgLimit;
static xActLogic sgActLogic;

//初始化模式处理
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


//中间传递变量
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

/*tbox*/
static xActionFlag sgActionFlag;
static xActionTimes sgActionTimes;
static xActionHour sgActionour;

/*HMI*/
static xCanSDO CanSend582;
static xCanSDO CanRev602;

static xCanSend5A5 CanSend5A5 ;

static uint8_t u8HmiCheckFlag ;

/*
锁车功能试做
*/

//密码锁车功能
typedef struct
{
	uint32_t u32GPSRandomNum;//GPS随机数
	uint32_t u32TMPRandomNum;//临时解锁随机数
	uint32_t u32TempUnlockKey;//临时解锁密码
	uint32_t u32GPSUnlockKey;//GPS解锁密码
	uint16_t u16UnlockHour;//每次临时解锁后倒计时 72H(小时计走过480个0.1h)，动作计时,数值为0xABFF时设置为永久解锁。
}xRemoteLockKey;

typedef union
{
	uint8_t u8Data;
	struct
	{
		//执行标志位
		uint8_t b1HeartBeatQuery:1;//心跳查询开启or关闭
		uint8_t b1HeartBeatLock:1;//心跳锁车状态
		uint8_t b1LockLevel1:1;//一级锁车
		uint8_t b1LockLevel2:1;//二级锁车
		//锁车实时响应
		uint8_t b1PlateformUnlock:1;//平台解锁
		uint8_t b1TempUnlock:1;//临时解锁
		uint8_t b1PermaneUnlock:1;//永久解锁
	};
}xHeartBeatLock;

static xRemoteLockKey sgRemoteLockKey;
static xHeartBeatLock sgLockState;

//模拟量报警
//暂时未使用上压力通道1、压力通道2
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

//DDO报警
//缺少高低速、喇叭的报警码
static void vDoPwmErrCallBack(eDoPwmNo DoPwmNo)
{
	uint8_t u8cnt = 0;
	//清除自带DO报警
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
		case BLINK_LED:
			i32ErrCodeSet(BLINK_LED_VALV_ERR);
			break;
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
	//清除自带报警
	i32ErrCodeClr(ErrCode66);
	i32ErrCodeClr(ErrCode67);
	switch(u8Channel)
	{
		case PropDriverCh0:
			i32ErrCodeSet(LIFT_DOWN_VALVE_ERR);
			break;
		case PropDriverCh1:
			i32ErrCodeSet(LOW_VALVE2_ERR);
			break;
		default:
			break;
	}
}

static void vSwiInitCheck(void)//下控分为开关输入和can通信输入、待确定参数选项
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
//	int8_t	i8HandleValue = 0;
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
			
		sgPcuInfo.i8HandleRaw  = RevData->Data.b4HandleCtrlHigh << 4 | RevData->Data.b4HandleCtrlLow;
		
				
		if(sgPcuInfo.i8HandleRaw > sgPcuHandle.i8PositiveValue)
		{
			i8HandleValueChange = 127;
		}
		else if(sgPcuInfo.i8HandleRaw > (sgPcuHandle.i8MiddleValue + gUserInfo.u8DeadZoneAdjust))//中值到最大值
		{
			i8HandleValueChange = (int8_t)((int16_t)(sgPcuInfo.i8HandleRaw - sgPcuHandle.i8MiddleValue) * 127 / (sgPcuHandle.i8PositiveValue - sgPcuHandle.i8MiddleValue)) ;
		}
		else if(sgPcuInfo.i8HandleRaw > (sgPcuHandle.i8MiddleValue - gUserInfo.u8DeadZoneAdjust))
		{
			i8HandleValueChange = 0;
		}
		else if(sgPcuInfo.i8HandleRaw > sgPcuHandle.i8NegativeValue)//中值到最小值
		{
			i8HandleValueChange =  (int8_t)((sgPcuHandle.i8MiddleValue - sgPcuInfo.i8HandleRaw) * 127 / (sgPcuHandle.i8NegativeValue - sgPcuHandle.i8MiddleValue)) ;
		}
		else
		{
			i8HandleValueChange = - 128;
		}
		
	
		sgPcuInfo.i16HandleValue = i8HandleValueChange * 32;
		
		i32SetPara(PARA_HandleAnalog, (abs(sgPcuInfo.i8HandleRaw) * 100 / 127));
		
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


//开关量处理
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
			if(u16TiltDelay < 200)//延时判断避免行走启动造成干扰
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
		
	if(FunctionEnable == gUserInfo.u8AngleSimulationLimit)//角度模拟限位功能
	{
		int32_t i32Angletmp = 0;
		i32Angletmp = i32LocalAiGetValue(ANGLE_SENSOR_CHANNEL);
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
		
	if(0 == sgUserSets.b1LowLimitEn)
	{
		SwiInput.b1DownLimitSwi = 0;
	}
	if(0 == sgUserSets.b1UpLimitEn)
	{
		SwiInput.b1UpLimitSwi = 0;
	}
	
	//倾角与坑洞保护
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
		
		if((FunctionEnable == gUserInfo.u8PitProtectFunc)&&(0 == SwiInput.b1PitSwi)
			&&(0 != sgUserSets.b1LowLimitEn))
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

//运行模式切换
//手柄改参数、自动标定称重功能。
static void vModeChange(void)//已测试
{
	if((0 != sgPcuKeyInit.u8data)&&(PARASETS_DISABLE == u8ParaSetMode))//上电时手柄开关量不为0
	{
		if((1 == sgSwiInput.b1PcuSwi)
			&&(1 == sgPcuKeyInit.b1Speaker)
			&&(1 == sgPcuKeyInit.b1LiftMode)
			&&(1 == sgPcuKeyInit.b1Slow)
			&&(0 == sgPcuInfo.PcuKeyInfo.b1Speaker)
			&&(0 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
			&&(0 == sgPcuInfo.PcuKeyInfo.b1Slow))//平台位置，先喇叭，举升，慢速,然后松开  通用参数
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
			&&(0 == sgPcuInfo.PcuKeyInfo.b1LiftMode)//喇叭举升 速度设置
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
			&&(0 == sgPcuInfo.PcuKeyInfo.b1Slow)//龟速行走H9
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
				&&(0 == sgPcuInfo.PcuKeyInfo.b1Slow))//龟速喇叭 H8模式
		{
			u8RunMode = PARA_SETS_MODE;
			u8ParaSetMode = H8_MODE;		
			i32ErrCodeClr(LIFT_BUTTON_ERR);
			i32ErrCodeClr(SLOW_BUTTON_ERR);
			i32ErrCodeClr(MOVE_BUTTON_ERR);						
		}
	}
	else if((SPEED_PARASET == u8ParaSetMode))//速度模式，按举升和龟速三秒后
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
	else if((NORMAL_MODE == u8RunMode)&&(1 == sgSwiInput.b1PcuSwi))//正常模式进入标定状态
	{
		//已测试
		static uint32_t	u32KeyRecord = 0;//向下为1，向上为0
		static xSwiInput	KeyState ;
		static uint8_t	u8SwitchCnt = 0;
		
		#define EMPTY_CALI_SWITCH		0b11111011111010111
		#define FULL_CALLI_SWITCH		0b11111011111011111
		
		if((1 == sgSwiInput.b1LowerCtlDown)||(1 == sgSwiInput.b1LowerCtlUp))//每按一次就复位
		{
			if(false == u8GetNetTimerStartFlag(TIMER_SwitchCheck))
				vSetNetTimer(TIMER_SwitchCheck,SWITCH_CHECK_TIME);
			else
				vResetNetTimer(TIMER_SwitchCheck);
		}
		
		if((1 == KeyState.b1LowerCtlUp)&&(0 == sgSwiInput.b1LowerCtlUp))//上键松开
		{
			u32KeyRecord = u32KeyRecord << 1;
			u8SwitchCnt++;
		}
		
		if((1 == KeyState.b1LowerCtlDown)&&(0 == sgSwiInput.b1LowerCtlDown))//下键松开
		{
			u32KeyRecord = ( u32KeyRecord << 1) + 1;
			u8SwitchCnt++;
		}
		KeyState.u16data = sgSwiInput.u16data;
		
		//比较
		if(((0 != (u32KeyRecord ^ (EMPTY_CALI_SWITCH >>( 17 - u8SwitchCnt))))
			&&(0 != (u32KeyRecord ^ (FULL_CALLI_SWITCH >>( 17 - u8SwitchCnt)))))
			||(true == u8GetNetTimerOverFlag(TIMER_SwitchCheck)))//与两个都不同或超时,清空所有
		{
			u32KeyRecord = 0;
			u8SwitchCnt = 0;
			vKillNetTimer(TIMER_SwitchCheck);
		}

		if(0 == (u32KeyRecord ^ EMPTY_CALI_SWITCH))//切换模式，清除计时器
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
//	else if((PRESSURE_CALIBRATION_MODE == u8RunMode)&&(true == ))//标定完成，软重启一次
//	{
//		
//	}
//		i32SetPara(PARA_OnOffValveCurrent,u8RunMode);
		//i32SetPara(PARA_TurnRightValveCurrent,u8ParaSetMode);
}


//操作sgActLogic
static void vActionMonitNormal()
{
	if(1 == sgSwiInput.b1PcuSwi)
	{
		if(INITIAL_MODE == u8PcuMode)//模式切换
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
			&& (0 == sgPcuInfo.PcuKeyInfo.b1Slow)//当前按键为0
			&& (1 == PCUStateRecord.PcuKeyInfo.b1Slow))//历史按键为1
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
				if(0 == sgLimit.b1NoTurn)//无转向限制
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
				if(FunctionEnable == gUserInfo.u8LiftReverseFunc)
				{	
					sgPcuInfo.i16HandleValue = -sgPcuInfo.i16HandleValue;
				}
				
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
	
	PCUStateRecord.PcuKeyInfo.u8data = sgPcuInfo.PcuKeyInfo.u8data;
}

//压力标定
//等矗峰测试
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
		case PC_INIT://角度、下限位、降低到最低水平
			if(false == u8GetNetTimerStartFlag(TIMER_AlarmDelay))//检测电量，安全检测通过后，喇叭响5次，开始标定，否则报警
			{
				vSetNetTimer(TIMER_AlarmDelay,ALARM_TIME);
			}
			else if(false == u8GetNetTimerOverFlag(TIMER_AlarmDelay))//开始初始化检查
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
					if(gUserInfo.u16MinAngle + 300 > i32LocalAiGet(ANGLE_SENSOR_CHANNEL))//在最小角度之上阈值范围内
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
				if(FunctionEnable == gUserInfo.u8AngleSimulationLimit)//有角度模拟限位功能，检测角度是否在最小角度
				{
					if(gUserInfo.u16MaxAngle <= i32LocalAiGet(ANGLE_SENSOR_CHANNEL))//最大角度
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
					else//未触及最大角度
					{
						sgActLogic.b1LiftUpAct = 1;
						u16MotorVal = MOTOR_MAX_SPEED_VALUE;
					}
				}
//				else
//				{
//					if(1 == sgSwiInput.b1UpLimitSwi)//触发上限位开关，停止
//					{
//						u8ActionFlag = PC_FIRSTDOWN;
//						u16MotorVal = 0;
//					}
//					else//下限位开关之上
//					{
//						sgActLogic.b1LiftUpAct = 1;
//						u16MotorVal = MOTOR_MAX_SPEED_VALUE;
//					}
//				}
			break;
		case PC_FIRSTDOWN:
				if(FunctionEnable == gUserInfo.u8AngleSimulationLimit)//有角度模拟限位功能，检测角度是否在最小角度
				{
					if(gUserInfo.u16MinAngle + 300 > i32LocalAiGet(ANGLE_SENSOR_CHANNEL))//在最小角度之上阈值范围内
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
//					if(1 == sgSwiInput.b1DownLimitSwi)//触发下限位开关,再下降8秒
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
//					else//下限位开关之上
//					{
//						sgActLogic.b1LiftDownAct = 1;
//						u16MotorVal = MOTOR_MAX_SPEED_VALUE;
//					}
				}
			break;
		case PC_SECONDLIFT://没有角度传感器怎么做？
				if(FunctionEnable == gUserInfo.u8AngleSimulationLimit)//有角度模拟限位功能，检测角度是否在最小角度
				{
					if(true == u8GetNetTimerOverFlag(TIMER_Calibration))//定时到了后触发起升，起升到达对应角度后停止然后重新开始计时
					{

						if(u16StopAnle <= i32LocalAiGet(ANGLE_SENSOR_CHANNEL))//到达对应角度，重新计时
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
//					if(1 == sgSwiInput.b1DownLimitSwi)//触发下限位开关,再下降8秒
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
//					else//下限位开关之上
//					{
//						sgActLogic.b1LiftDownAct = 1;
//						u16MotorVal = MOTOR_MAX_SPEED_VALUE;
//					}
				}
			break;
		case PC_SECONDDOWN:
				if(true == u8GetNetTimerOverFlag(TIMER_Calibration))//定时到了后触发起升，起升到达对应角度后停止然后重新开始计时
				{
					if(u16StopAnle >=  i32LocalAiGet(ANGLE_SENSOR_CHANNEL))//到达对应角度，重新计时
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
		case PC_ERR://报警，不动作
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
	//不执行操作
}

//执行模式切换

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
	
	if(u16Index != u16IndexREC)//操作参数转变，存储旧参数，获取新参数
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
		
		if(u8SaveCnt >= 20)//每1s存一次，防止突然掉电
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

//PCU闪烁显示
static void	vPcuParaSetProc(xPcuSendPara *SendData)//两秒为周期，A->灭->B->灭->A
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
	
	
	if(u8ParaSetModeRec != u8ParaSetMode)//切换模式，页面，临时参数初始化
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
			
			if(MOVE_MODE == u8PcuMode)//行走灯控制方法暂不知道
			{
//				SendData->Data.b1ModeLed = 1;
//				SendData->Data.b1LiftLed = 0;
//				SendData->Data.b1MoveLedCtr = 1;
//				SendData->Data.b1LiftLedCtl = 1;
					//12.2 TEST.徐工DD行走模式，D7起升模式
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
				if(0 == sgSwiInput.b1PcuSwi)//下控闪烁CH.
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
				
				if(1 == sgActLogic.b1LiftDownAct)//下降显示E1
					vPcuDisplayNumber(SendData,0xE1,HEX_DISPLAY);				
			}

		}
			break;
		case COMMON_PARASET://显示，，喇叭举升龟速
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
				case CP_LOWBATSLP://起升
					u16PcuParaMonit(SendData,PARA_LowVolShutDownTime,EEPROME_DATA);
					
					if(1 == sgPcuInfo.PcuKeyInfo.b1Enable)//使能->空满载差值
						u8DisplayPage = CP_MAX_PRESSUREDIFF;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Speaker)//喇叭->动态超载
						u8DisplayPage = CP_DYN_OVERRLOADRATE;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Slow)//龟速->超载稳定延时
						u8DisplayPage = CP_DYN_OVERLOADDELAY;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LOWBATALM;
					break;
				case CP_MAX_PRESSUREDIFF://起升 使能
					u16PcuParaMonit(SendData,PARA_MaxDifferencePercent,EEPROME_DATA);
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
						u8DisplayPage = CP_LOWBATSLP;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LOWBATALM;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Speaker)//喇叭
						u8DisplayPage = CP_STB_OVERLOADRATE;
					break;
				case CP_DYN_OVERRLOADRATE://起升 喇叭
					u16PcuParaMonit(SendData,PARA_DynamicOverLoadPercent,EEPROME_DATA);
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
						u8DisplayPage = CP_LOWBATSLP;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LOWBATALM;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Enable)//使能
						u8DisplayPage = CP_STB_OVERLOADRATE;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Slow)//龟速
						u8DisplayPage = CP_ANGLE_UPLIMIT;
					break;
				case CP_DYN_OVERLOADDELAY://起升 龟速
					u16PcuParaMonit(SendData,PARA_OverLoadStabilityDelay,EEPROME_DATA);
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
						u8DisplayPage = CP_LOWBATSLP;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LOWBATALM;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Speaker)//喇叭
						u8DisplayPage = CP_ANGLE_UPLIMIT;
					break;
				case CP_STB_OVERLOADRATE://起升 喇叭 使能
					u16PcuParaMonit(SendData,PARA_StaticOverLoadPercent,EEPROME_DATA);
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
						u8DisplayPage = CP_LOWBATSLP;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LOWBATALM;				
					if(1 == sgPcuInfo.PcuKeyInfo.b1Slow)//龟速
						u8DisplayPage = CP_ANTI_DELAYTIME;	
					break;
				case CP_ANGLE_UPLIMIT://起升 喇叭 龟速
					u16PcuParaMonit(SendData,PARA_AngleValue0,EEPROME_DATA);
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
						u8DisplayPage = CP_LOWBATSLP;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LOWBATALM;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Enable)//使能
						u8DisplayPage = CP_ANTI_DELAYTIME;	
					break;
				case CP_ANTI_DELAYTIME:// 起升 喇叭 龟速 使能
					u16PcuParaMonit(SendData,PARA_AccAndDecAntiPinch,EEPROME_DATA);
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
						u8DisplayPage = CP_LOWBATSLP;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LOWBATALM;
					break;
				case CP_LOWBATALM://行走
					u16PcuParaMonit(SendData,PARA_LowVolAlmTime,EEPROME_DATA);
					if(1 == sgPcuInfo.PcuKeyInfo.b1Speaker)//喇叭->语音报警电量
						u8DisplayPage = CP_ALARMVOLUE;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Slow)//龟速->休眠
						u8DisplayPage = CP_UPCONTROL_SLEEPTIME;
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)//
						u8DisplayPage = CP_LOWBATSLP;
					break;
				case	CP_ALARMVOLUE://行走 喇叭
					u16PcuParaMonit(SendData,PARA_VoiceAlarmVolume,EEPROME_DATA);
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
						u8DisplayPage = CP_LOWBATSLP;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LOWBATALM;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Slow)
						u8DisplayPage = CP_ANGLE_DOWNLIMIT;
					break;				
				case CP_UPCONTROL_SLEEPTIME://行走 龟速
					u16PcuParaMonit(SendData,PARA_UpperCtlButSleep,EEPROME_DATA);
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
						u8DisplayPage = CP_LOWBATSLP;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LOWBATALM;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Speaker)//喇叭
						u8DisplayPage = CP_ANGLE_DOWNLIMIT;
					break;
				case CP_ANGLE_DOWNLIMIT://行走 龟速 喇叭
					u16PcuParaMonit(SendData,PARA_AngleValue1,EEPROME_DATA);
					if(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode)
						u8DisplayPage = CP_LOWBATSLP;
					if(1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)
						u8DisplayPage = CP_LOWBATALM;
					if(1 == sgPcuInfo.PcuKeyInfo.b1Enable)//使能
						u8DisplayPage = CP_DOWN_DEC;
					break;
				case CP_DOWN_DEC:// 行走 喇叭 龟速 使能
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
		case SPEED_PARASET://显示PS，设置各个速度，喇叭举升,左右调节数值大小
			//已经测试
			switch(u8DisplayPage)
			{
				case PS_INIT://闪烁PS
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
				case PS_LIFT://设置起升速度
					u16PcuParaMonit(SendData,PARA_LiftSpeed,EEPROME_DATA);
			
					if(1 == sgPcuInfo.PcuKeyInfo.b1Enable)//举升->下降，举升->防夹手，举升->
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
		case MACHINE_PARASET://SC，机器代码，电量耗尽报警，长按喇叭保存
			//已测试
			//待完善-根据车辆类型更改设置
			switch(u8DisplayPage)
			{
				case	SC_INIT://闪烁SC，
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
				case SC_SETSL://左侧闪烁,每次调节10
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
					
					if(1 == sgPcuInfo.PcuKeyInfo.b1Speaker)//按下喇叭保存
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
				case	SC_SETSR://右侧闪烁
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
					
					if(1 == sgPcuInfo.PcuKeyInfo.b1Speaker)//按下喇叭保存
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
		case FUNCTION_PARASET://H9，设置功能配置及角度模拟限位
			{
				if((1 == sgPcuInfo.PcuKeyInfo.b1Slow)&&(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode))//按下龟速和起升
				{
					u8DisplayPage = H9_LOW16CHOOSE;
					vKillNetTimer(TIMER_SwitchCheck);
				}
				if((1 == sgPcuInfo.PcuKeyInfo.b1MoveMode)&&(1 == sgPcuInfo.PcuKeyInfo.b1LiftMode))//按下行走和起升
				{
					u8DisplayPage = H9_HIG16CHOOSE;
					vKillNetTimer(TIMER_SwitchCheck);
				}
				if(1 == sgPcuInfo.PcuKeyInfo.b1Speaker)
				{
					u8DisplayPage = H9_SAVE;
				}
				if((1 == sgPcuInfo.PcuKeyInfo.b1TurnLeft)&&(1 == sgPcuInfo.PcuKeyInfo.b1Enable))//左转+使能 保存上限位
				{
					u8DisplayPage = H9_UPLIMIT_SAVE;
				}
				if((1 == sgPcuInfo.PcuKeyInfo.b1TurnRight)&&(1 == sgPcuInfo.PcuKeyInfo.b1Enable))//右转+使能 保存下限位
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
					/*H9选项，由高到低
					3231null,30上电有模式，29坑洞保护，28~25电池类型，
					24未标定上下限位可动作，23关闭水平传感器，22null，21角度模拟限位，20~17车辆类型，
					16重量双载荷功能，15举升后禁止行走，14高度双区域功能，13 12 null，11防夹手减速下降，10 18报警允许起升 ，9 预留
					8预留，7下降阀类型，6启动高速，5高低速阀得电高速，4null，3无线防碰撞功能，2举升反向，1无动作10s后睡眠，
					*/
					/*暂未实现：
					30上电有模式-默认行走		暂未实现		缺少参数
					24 未标定上下限位可动作		暂未实现		缺少参数
					20~17车辆类型						参数冲突		机器代码和车辆类型
					16重量双载荷							暂未实现		未知功能
					15举升后禁止行走					暂未实现		缺少参数
					14高度双区域功能					暂未实现		未知功能
					11防夹手减速下降					暂未实现		参数待定
					10坑洞报警允许行走				暂未实现		缺少参数
					7下降阀类型							暂未实现		已有参数
					6启动高速								暂未实现		缺少参数
					5高低阀									暂未实现		没有对应的阀
					3无线防碰撞功能					暂未实现		没有模块
					1无动作睡眠							暂未实现		待确定使用参数
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
						u8CombinedData = 1;//借用此参数处理开关量
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
							u8SwitchDelay = (u32CombinedDataH9>>8) & 0xFF;//借用当作传参变量高16位高8
							break;
						case 1:
							u8DisplayMode1 = HEX_DISPLAY | MASK_RIGHT | DOUBLE_DOT ;
							u8DisplayMode2 = HEX_DISPLAY | DOUBLE_DOT ;
							u8SwitchDelay = (u32CombinedDataH9>>8) & 0xFF;//借用当作传参变量高16位高8
							break;
						case 2:
							u8DisplayMode1 = HEX_DISPLAY | MASK_LEFT ;
							u8DisplayMode2 = HEX_DISPLAY  ;
							u8SwitchDelay = (u32CombinedDataH9) & 0xFF;//借用当作传参变量高16位低8
							break;
						case 3:
							u8DisplayMode1 = HEX_DISPLAY | MASK_RIGHT ;
							u8DisplayMode2 = HEX_DISPLAY  ;
							u8SwitchDelay = (u32CombinedDataH9) & 0xFF;//借用当作传参变量高16位低8
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
					if(0 != (u8SubPage % 2))//操作低4位
					{
						if(0xF == (u8SwitchDelay & 0xF))//为F加1
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
						else if(0 == (u8SwitchDelay & 0xF))//为0减1
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
					else //操作高四位
					{
						if(0xF0 == (u8SwitchDelay & 0xF0))//为F加1
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
						else if(0 == (u8SwitchDelay & 0xF0))//为0减1
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
					//将值传给u32
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
						u8CombinedData = 1;//借用此参数处理开关量
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
							u8SwitchDelay = (u32CombinedDataH9>>24) & 0xFF;//借用当作传参变量高16位高8
							break;
						case 1:
							u8DisplayMode1 = HEX_DISPLAY | MASK_RIGHT | LEFT_DOT ;
							u8DisplayMode2 = HEX_DISPLAY | LEFT_DOT ;
							u8SwitchDelay = (u32CombinedDataH9>>24) & 0xFF;//借用当作传参变量高16位高8
							break;
						case 2:
							u8DisplayMode1 = HEX_DISPLAY | MASK_LEFT | RIGHT_DOT ;
							u8DisplayMode2 = HEX_DISPLAY | RIGHT_DOT ;
							u8SwitchDelay = (u32CombinedDataH9>>16) & 0xFF;//借用当作传参变量高16位低8
							break;
						case 3:
							u8DisplayMode1 = HEX_DISPLAY | MASK_RIGHT | RIGHT_DOT ;
							u8DisplayMode2 = HEX_DISPLAY | RIGHT_DOT ;
							u8SwitchDelay = (u32CombinedDataH9>>16) & 0xFF;//借用当作传参变量高16位低8
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
					if(0 != (u8SubPage % 2))//操作低4位
					{
						if(0xF == (u8SwitchDelay & 0xF))//为F加1
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
						else if(0 == (u8SwitchDelay & 0xF))//为0减1
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
					else //操作高四位
					{
						if(0xF0 == (u8SwitchDelay & 0xF0))//为F加1
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
						else if(0 == (u8SwitchDelay & 0xF0))//为0减1
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
					//将值传给u32
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
					vPcuNull(SendData);//待修改，需要多次保存
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


static void vPcuSendProc(xPcuSendPara *SendData)//发送给pcu的回调函数
{
	vPcuParaSetProc(SendData);
}

//接收MCU通信
static void vMstRevProc(xMstRevPara *RevData)
{
	uint16_t tmp = 0;
	
	if(0 != RevData->b1MainDriver)//主接触器状态
	{
		
	}
	else
	{
		
	}
	
	if(0 != RevData->b1Ebrake)//刹车状态
	{
		
	}
	else
	{	
		
	}
	
	if(0 != RevData->b1Driver3State)//DO3状态
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
	
	CanSend5A5.u8VbusCurrentH = RevData->u8CurrentHigh;
	CanSend5A5.u8PhaseCurrentL = RevData->u8CurrentLow;
	
	tmp = RevData->u8MotorTmp - 40;
	CanSend5A5.u8MotorTemperatureL = tmp * 10;
	CanSend5A5.u8MotorTemperatureH = (tmp * 10 )>> 8;
	/*Motor Temp*/

	tmp = RevData->u8BoardTmp - 40;
	CanSend5A5.u8BoardTemperatureL = tmp * 10;
	CanSend5A5.u8BoardTemperatureH = (tmp * 10 )>> 8;
}

//闪光灯回调
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

//闪光蜂鸣器
//迅响器同步功能待确认： 喇叭、闪光蜂鸣器、手柄蜂鸣器，具体内容
/*待完善*/
// if（enable == ）57号参数
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


/*
电量曲线重做
*/
//const static uint16_t u16BatteryTable[6][2]=
//{
//	{25000, 100},
//	{24700, 80 },
//	{24000, 64 },
//	{23200, 30 },
//	{22200, 15 },
//	{20700, 0 },
//};

static xBatteryTable u16BatteryTable[7]=
{
	{.Voltage = 25000,.Soc = 100},
	{.Voltage = 24700,.Soc = 80 },
	{.Voltage = 24000,.Soc = 65 },
	{.Voltage = 23200,.Soc = 50 },
	{.Voltage = 22200,.Soc = 30 },
	{.Voltage = 20700,.Soc =15 },
	{.Voltage = 20000,.Soc =0 },
};

static void vBatteryDisplaySets()
{
	static xBatToCode BatToCodeTable[]=
	{
		{.u8SocLevel = 80,.u8Display = 0,.u8FlashState = 0},
		{.u8SocLevel = 65,.u8Display = 1,.u8FlashState = 0},
		{.u8SocLevel = 50,.u8Display = 2,.u8FlashState = 0},
		{.u8SocLevel = 30,.u8Display = 3,.u8FlashState = 0},
		{.u8SocLevel = 15,.u8Display = 4,.u8FlashState = 0},
		{.u8SocLevel =  0,.u8Display = 5,.u8FlashState = 0},
	};
	xBatDisplay BatDisplaySets = {.u8BatSetsNum = (sizeof(BatToCodeTable)/sizeof(BatToCodeTable[0])),.u8FlashDisplayCnt = 10,.u8FlashMaskCnt = 10,.BatToCode = BatToCodeTable};

	vDisplayBatSet(&BatDisplaySets);
}

static void vBatterySocInit()
{	
	xBatterySets StaticSoc = {.u16CheckTime = (360 / USER_ECU_PERIOD), .u16FreshTime = (3*1000),.u8FilterPoint = 40,.u8TableLenth = (sizeof(u16BatteryTable)/sizeof(u16BatteryTable[0])),.BatteryTable = u16BatteryTable};
	vBatterySocSet(&StaticSoc,BatterySoc_Static);
	vBatteryDisplaySets();
}
	
static void vBatteryManage(void)
{
	uint16_t u16BatteryVoltage = 0;
	static uint16_t u16WarningCnt;
		
	static uint16_t u16AlarmCnt;
	static uint16_t u16DelayCheck = 3000;//初始先执行一次

	if(LiBattery == gUserInfo.u8BatteryType)
	{
		u8Soc = gCanRevPdoInfo.CanRevBMS28A.u8Soc * 0.4;
		u16BatteryVoltage = (gCanRevPdoInfo.CanRveBMS18A.u8BatVoltH<<8) |gCanRevPdoInfo.CanRveBMS18A.u8BatVoltL;
		u16BatteryVoltage /= 10; 
		i32SetPara(PARA_BmsVoltage,u16BatteryVoltage);
		
		if(1 == gCanRevPdoInfo.CanRevBMS28A.b1ErrorOverHeatRate1)
			i32ErrCodeSet(BMS_BATTERY_TEMP_HIGH1_ERR);

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
		if(0 == sgActLogic.u8Data)
		{
			if(u16DelayCheck < (3000 / USER_ECU_PERIOD))
			{
				u16DelayCheck++ ;
			}
			else
			{
				i32SetPara(PARA_BackValveCurrent,u8ForkLiftSocProc(BatterySoc_Static));
				u8Soc = u8ForkLiftSocProc(BatterySoc_Static);
			}
		}
		else
		{
			u16DelayCheck = 0;
		}
	}
	
	if(u16WarningCnt > 1000)
	{
	 if(u8Soc < 5)
		{
			i32ErrCodeSet(BAT_LOW_CAP2_ERR);
			i32ErrCodeClr(BATTERY_LOW_CAP1_ERR );
		}	
		else if(u8Soc < 10)
		{
			if(0 == i32ErrCodeCheck(BAT_LOW_CAP2_ERR))
			{
				if(u16AlarmCnt < (30*1000 / USER_ECU_PERIOD))//延时30秒报警
				{
					u16AlarmCnt++;
				}
				else
				{
					i32ErrCodeSet(BATTERY_LOW_CAP1_ERR );
				}			
			}
		}
		else
		{
			u16AlarmCnt = 0;
			i32ErrCodeClr(BAT_LOW_CAP2_ERR);
			i32ErrCodeClr(BATTERY_LOW_CAP1_ERR);		
		}		
	}
	else
	{
		u16WarningCnt ++;
	}

	i32SetPara(PARA_BmsSoc ,u8Soc);
	
}
/*******************************************************************************
* Name: void vEcuSetBeepPeriod(void)
* Descriptio: 蜂鸣器控制函数
* Input: NULL
* Output: NULL
* checked: 
* to be check:
*******************************************************************************/
static void vEcuSetBeepPeriod(void)
{
	static uint8_t u8Cnt = 0;
	
	if((1 == sgErrorState.b1Error)||(1 == sgErrorState.b1CaliErr))	/*如果存在故障的话*/
	{
		vBeepSetPeriod(60);
		vBeepSetOpenePeriod(120);
	}
	else if((1 == sgErrorState.b1Warning)
			||(1 == sgErrorState.b1CaliInit)
			||(1 == sgErrorState.b1CaliEnd))	/*警告*/
	{
		vBeepSetPeriod(180);
		vBeepSetOpenePeriod(240);
	}
	else if(((ABOVE_SWI != u8AntiPinchState)&&(UNDER_SWI_DELAY != u8AntiPinchState))
		||(true == u8GetNetTimerStartFlag(TIMER_EcuAntiPinchFunc)))		/*防夹手*/
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
	else if((0 != sgActLogic.u8Data) && ( FunctionEnable == gUserInfo.u8ActAlmFunc )&&(u8RunMode == NORMAL_MODE))		/*动作报警*/
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

static void vSpeedControl2()
{
	static float fStepFactor = 0;//实锟斤拷执锟叫的加硷拷锟斤拷
	static float fFactorA = 0;
	static float fFactorB = 0;
	static float fFactorC = 0;
	static float fStepRate = 0;//锟斤拷锟皆加硷拷锟斤拷时锟斤拷锟接硷拷锟斤拷锟斤拷
	static uint16_t u16SpeedTargetRec = 0;
	
	static uint8_t u8AccFlag = 0;
	static uint16_t u16Timecnt = 0;//锟斤拷锟节硷拷锟姐步锟斤拷锟斤拷时锟斤拷锟斤拷锟?
	static uint16_t u16TimeRange = 0;
	
	static uint8_t u8IncreaseDecreaseFlag = 0;

	#define STABLE_STATE		0
	#define INCREASE_STATE	1
	#define DECREASE_STATE	2
	
	
	
	#define ACC_STATIC	0		//默锟斤拷值锟斤拷指锟斤拷锟劫讹拷锟饺讹拷锟津波讹拷锟饺斤拷小锟斤拷
	#define ACC_FRESH		1  	//锟接硷拷锟劫诧拷锟斤拷锟斤拷锟斤拷
	#define ACC_EXCUTE	2		//锟接硷拷锟斤拷执锟斤拷
	#define RVS_FRESH		3		//指锟筋反锟斤拷时锟斤拷锟斤拷锟斤拷锟斤拷
	#define	RVS_EXCUTE	4		//指锟筋反锟斤拷时锟斤拷锟斤拷锟劫诧拷锟斤拷锟斤拷锟斤拷锟斤拷锟?
	
	switch(u8AccFlag)
	{
		case ACC_STATIC://锟斤拷位时锟斤拷锟斤拷指锟斤拷值锟斤拷锟斤拷锟叫★拷锟街碉拷锟斤拷锟斤拷锟街?
			fStepFactor = 0;
			if((u16SpeedTarget != u16SpeedCmd)||(u16TimeRange != u16Time))
				u8AccFlag = ACC_FRESH;
			break;
		case ACC_FRESH://锟斤拷锟斤拷锟斤拷锟斤拷,系锟斤拷值ABC锟斤拷目锟斤拷锟劫度ｏ拷u16SpeedTarget - u16SpeedCmd
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
		case ACC_EXCUTE://目锟斤拷锟劫度变化锟斤拷锟斤拷锟斤拷执锟叫ｏ拷锟戒化锟斤拷锟斤拷锟斤拷锟铰硷拷锟斤拷锟斤拷锟?,锟劫度逼斤拷锟斤拷氐锟斤拷锟绞甲刺?
			fStepFactor = fFactorA * u16Timecnt * u16Timecnt + fFactorB * u16Timecnt + fFactorC;
			if(u16Timecnt<u16TimeRange)
				u16Timecnt ++;

			if(fStepFactor < (fStepRate / 100))//锟斤拷小斜锟绞诧拷锟斤拷锟斤拷状态锟铰硷拷锟劫度碉拷1/10
			{
				fStepFactor = fStepRate / 100;
			}					

			if((u16SpeedTargetRec == u16SpeedTarget))//指锟斤拷浠?小锟斤拷锟斤拷值
			{
				if(u16SpeedCmd < u16SpeedTarget)
				{
					if(u16Timecnt<u16TimeRange)
					{
						u16SpeedCmd += fStepFactor ;
					}
					else//锟斤拷锟斤拷锟斤拷锟?
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
					else//锟斤拷锟斤拷锟斤拷锟?
					{
						u16SpeedCmd = u16SpeedTarget ;
						u8AccFlag = ACC_STATIC;
					}
				}
			}
			else if(((u16SpeedCmd > u16SpeedTarget)&&(INCREASE_STATE == u8IncreaseDecreaseFlag ))
				||((u16SpeedCmd < u16SpeedTarget)&&(DECREASE_STATE == u8IncreaseDecreaseFlag )))//锟斤拷锟斤拷时指锟筋降锟斤拷锟剿ｏ拷锟斤拷锟斤拷锟绞敝革拷锟斤拷锟斤拷锟斤拷锟?
			{
				u8AccFlag = RVS_FRESH;
			}
			else//指锟斤拷浠?锟斤拷锟斤拷锟斤拷值锟斤拷锟斤拷锟铰硷拷锟斤拷
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
					||((u16SpeedCmd > u16SpeedTarget)&&(DECREASE_STATE == u8IncreaseDecreaseFlag )))//锟斤拷锟斤拷锟斤拷锟斤拷谢指锟角帮拷锟?
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
			
			if(fStepFactor < (fStepRate / 10))//锟斤拷锟劫诧拷锟斤拷锟斤拷锟斤拷锟斤拷值锟斤拷锟截癸拷默锟斤拷状态
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
	static uint8_t u8MotorFlag = 0;			/*Motor Act Advance or Delay*/
	#define MOTOR_STATIC			0		 	/*静止状态*/
	#define MOTOR_F_START			1 		/*前进起步*/ 
	#define MOTOR_B_START			2 		/*后退起步*/
	#define MOTOR_F_STOP 			3			/*前进刹车*/
	#define MOTOR_B_STOP			4			/*后退刹车*/
	#define MOTOR_RUN_STABLE 	5			/*稳定运行*/


	/*MOTOR FEEDBACK VALUE*///反馈速度状态判断
	#define	MOTOR_FEEDBACK_MAX_SPEED						gUserInfo.u16MotorMaxSpd//最大转速
	#define	MOTOR_SLOP_START_SPEED 							(MOTOR_FEEDBACK_MAX_SPEED * gUserInfo.u16Analog3DeadZoneMinVal / 100)//
	#define MOTOR_SLOP_SHUTDOWN_SPEED						(MOTOR_FEEDBACK_MAX_SPEED * gUserInfo.u16Analog3DeadZoneMaxVal / 100)
	#define MOTOR_FEEDBACK_STEER_CLOSE_SPEED		(MOTOR_FEEDBACK_MAX_SPEED * gUserInfo.u16Analog3MidVal / 100)
	#define MOTOR_HORIZON_CLOSE_SPEED				 		(MOTOR_FEEDBACK_MAX_SPEED * gUserInfo.u16ThrottleFMidVal / 100)
	#define MOTOR_FEEDBACK_UP_CLOSE_SPEED 			(MOTOR_FEEDBACK_MAX_SPEED * gUserInfo.u16ThrottleFDeadZoneMinVal /100)
	#define MOTOR_FEEDBACK_UP_OPEN_SPEED				(MOTOR_FEEDBACK_MAX_SPEED *  gUserInfo.u16ThrottleFDeadZoneMaxVal /100)

	static uint8_t u8PumpFlag = 0;
	#define PUMP_STATIC			0
	#define PUMP_UP_START		1
	#define PUMP_UP					2
	#define PUMP_UP_STOP		3
	#define PUMP_DOWN_START	4
	#define PUMP_DOWN				5
	#define PUMP_DOWN_STOP	6

	if(((MOVE_MODE == u8PcuMode)||(MOTOR_STATIC != u8MotorFlag))
		&&(PUMP_STATIC == u8PumpFlag))
	{
		i32DoPwmSet(LIFTUP_PUMP,PUMP_CLOSE_PERCENTAGE);
		vPropSetTarget(LIFTDOWN_PUMP1, 0);
		vPropSetTarget(LIFTDOWN_PUMP2, 0);
		switch(u8MotorFlag)
		{
			case MOTOR_STATIC://静止状态
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
				vMoveAccParaSets();
				u16SpeedTarget = 0;
				break;
			case MOTOR_F_START://前进起步
				if((1 == sgSwiInput.b1TiltSwi)&&(u16MotorFdb > MOTOR_SLOP_START_SPEED))//斜坡起步
				{
					i32DoPwmSet(FORWARD_PUMP,PUMP_OPEN_PERCENTAGE);
					u8MotorFlag = MOTOR_RUN_STABLE;
				}
				else if(0 == sgSwiInput.b1TiltSwi)//平地起步
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
				break;
			case MOTOR_B_START:
				if((1 == sgSwiInput.b1TiltSwi)&&(u16MotorFdb > MOTOR_SLOP_START_SPEED))
				{
					i32DoPwmSet(BACKWARD_PUMP,PUMP_OPEN_PERCENTAGE);
					u8MotorFlag = MOTOR_RUN_STABLE;
				}
				else if(0 == sgSwiInput.b1TiltSwi)
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
				break;
			case MOTOR_F_STOP:
				if(((1 == sgActLogic.b1TurnLeft)||(1 == sgActLogic.b1TurnRight))
					&&(u16MotorFdb < MOTOR_FEEDBACK_STEER_CLOSE_SPEED))//边转向边行走时，停止行走
				{
					i32DoPwmSet(FORWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
					u8MotorFlag = MOTOR_STATIC;
				}
				else if(1 == sgActLogic.b1ForwardAct)
				{
					u8MotorFlag = MOTOR_RUN_STABLE;
				}
				else if((0 == sgSwiInput.b1TiltSwi)&&(u16MotorFdb <= MOTOR_HORIZON_CLOSE_SPEED))//平地关阀阈值
				{
					i32DoPwmSet(FORWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
					u8MotorFlag = MOTOR_STATIC;
				}
				else if((1 == sgSwiInput.b1TiltSwi)&&(u16MotorFdb < MOTOR_SLOP_SHUTDOWN_SPEED))//wait motor slow down
				{
					i32DoPwmSet(FORWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
					u8MotorFlag = MOTOR_STATIC;
				}
				else
				{
					u8MotorFlag = MOTOR_F_STOP;
				}
				vMoveDecParaSets();
				u16SpeedTarget = 0;
				break;
			case MOTOR_B_STOP:
				if(((1 == sgActLogic.b1TurnLeft)||(1 == sgActLogic.b1TurnRight))
					&&(u16MotorFdb < MOTOR_FEEDBACK_STEER_CLOSE_SPEED))//if steerlogic exists ,shut down rapid 
				{
					i32DoPwmSet(BACKWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
					u8MotorFlag = MOTOR_STATIC;
				}
				else if(1 == sgActLogic.b1BackwardAct)//
				{
					u8MotorFlag = MOTOR_RUN_STABLE;
				}
				else if((0 == sgSwiInput.b1TiltSwi)&&(u16MotorFdb <= MOTOR_HORIZON_CLOSE_SPEED))
				{
					i32DoPwmSet(BACKWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
					u8MotorFlag = MOTOR_STATIC;
				}
				else if((1 == sgSwiInput.b1TiltSwi)&&(u16MotorFdb < MOTOR_SLOP_SHUTDOWN_SPEED))//wait motor slow down
				{
					i32DoPwmSet(BACKWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
					u8MotorFlag = MOTOR_STATIC;
				}
				else
				{
					u8MotorFlag = MOTOR_B_STOP;
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
				vMoveStableParaSets();
				u16SpeedTarget = u16MotorVal;
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

		u8SpeedRate = gUserInfo.u8FastDriveSpeed;//默锟斤拷
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
		if((1 == sgActLogic.b1TurnLeft)||(1 == sgActLogic.b1TurnRight))
		{
			if(u8SpeedRate > gUserInfo.u8MaxTurnSpeed)
				u8SpeedRate = gUserInfo.u8MaxTurnSpeed;		
		}
		
		if(1 == gUserInfo.u8ParallelValveReverseFunc)
		{
			if((1 == sgLimit.b1SpeedAfterLift)||(1 == sgLimit.b1Slow))//高低速阀可选
			{
				i32DoPwmSet(HIGH_SLOW_SPEED_PUMP,PUMP_OPEN_PERCENTAGE);
			}
			else
			{
				i32DoPwmSet(HIGH_SLOW_SPEED_PUMP,PUMP_CLOSE_PERCENTAGE);
			}				
		}
		else
		{
			if((1 == sgLimit.b1SpeedAfterLift)||(1 == sgLimit.b1Slow))//高低速阀可选
			{
				i32DoPwmSet(HIGH_SLOW_SPEED_PUMP,PUMP_CLOSE_PERCENTAGE);
			}
			else
			{
				i32DoPwmSet(HIGH_SLOW_SPEED_PUMP,PUMP_OPEN_PERCENTAGE);
			}			
		}

		
		u16SpeedTarget = u16SpeedTarget * u8SpeedRate / 100;
		
			/*steer process*/
		if(1 ==sgActLogic.b1TurnLeft)
		{
			i32DoPwmSet(TURNLEFT_PUMP,PUMP_OPEN_PERCENTAGE);
			u16SpeedTarget += gUserInfo.u8TurnPowerLimit * 4095 / 100;
		}
		else
		{
			i32DoPwmSet(TURNLEFT_PUMP,PUMP_CLOSE_PERCENTAGE);
		}
		
		if(1 == sgActLogic.b1TurnRight)
		{
			i32DoPwmSet(TURNRIGHT_PUMP,PUMP_OPEN_PERCENTAGE);
			u16SpeedTarget += gUserInfo.u8TurnPowerLimit * 4095 / 100;
		}
		else
		{
			i32DoPwmSet(TURNRIGHT_PUMP,PUMP_CLOSE_PERCENTAGE);
		}
		vSpeedControl();
	}
	else if((((LIFT_MODE == u8PcuMode)||(PUMP_STATIC != u8PumpFlag))&&(MOTOR_STATIC == u8MotorFlag))
		||((0 == i32LocalDiGet(PCU_SWICTH))))//行走、
	{
		i32DoPwmSet(BACKWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
		i32DoPwmSet(FORWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
		i32DoPwmSet(TURNLEFT_PUMP,PUMP_CLOSE_PERCENTAGE);
		i32DoPwmSet(TURNRIGHT_PUMP,PUMP_CLOSE_PERCENTAGE);
		i32DoPwmSet(HIGH_SLOW_SPEED_PUMP,PUMP_CLOSE_PERCENTAGE);
		/*lift process*/
		switch(u8PumpFlag)
		{
			case PUMP_STATIC:
				u16SpeedTarget = 0;
				vPropSetTarget(LIFTDOWN_PUMP1, 0);
				vPropSetTarget(LIFTDOWN_PUMP2, 0);
				i32DoPwmSet(LIFTUP_PUMP,PUMP_CLOSE_PERCENTAGE);
			
				if(1 == sgActLogic.b1LiftUpAct)
					u8PumpFlag = PUMP_UP_START;
				else if(1 == sgActLogic.b1LiftDownAct)
					u8PumpFlag = PUMP_DOWN_START;
				break;
			case PUMP_UP_START://起升开始
				u16SpeedTarget = u16MotorVal;
				u8SpeedRate = gUserInfo.u8LiftSpeed;
				u16Time = gUserInfo.u8AccAndDecLift;
				i32DoPwmSet(LIFTUP_PUMP,PUMP_OPEN_PERCENTAGE);
				if(u16MotorFdb > MOTOR_FEEDBACK_UP_OPEN_SPEED)//大于速度阈值切换到稳定运行
					u8PumpFlag = PUMP_UP;
				else if((0 == sgActLogic.b1LiftUpAct)||(1 == sgActLogic.b1LiftDownAct))
					u8PumpFlag = PUMP_UP_STOP;
				break;
			case PUMP_UP://起升稳定过程
				u16SpeedTarget = u16MotorVal;
				u8SpeedRate = gUserInfo.u8LiftSpeed;
				u16Time = gUserInfo.u8CurveLift;
				if(((0 == sgActLogic.b1LiftUpAct)&&(MOTOR_FEEDBACK_UP_CLOSE_SPEED < u16MotorFdb))||(1 == sgActLogic.b1LiftDownAct))//起升停止
					u8PumpFlag = PUMP_UP_STOP;
				break;
			case PUMP_UP_STOP://起升停止过程
				u16SpeedTarget = 0;
				u8SpeedRate = gUserInfo.u8LiftSpeed;				
				u16Time = gUserInfo.u8BrakeLift;
				if((0 == u16MotorFdb)||(1 == sgActLogic.b1LiftDownAct))//反馈转速为0
				{
					i32DoPwmSet(LIFTUP_PUMP,PUMP_CLOSE_PERCENTAGE);
					u8PumpFlag = PUMP_STATIC;
				}
				else if(1 == sgActLogic.b1LiftUpAct)
					u8PumpFlag = PUMP_UP;
				break;
			case PUMP_DOWN_START:
				u16SpeedTarget = u16MotorVal;
				u16Time = gUserInfo.u8AccAndDecLower;
				u8SpeedRate = gUserInfo.u8LowerSpeed;			
				if((((inserted_data[1] * PROP_CURRENT_FACOTR) > (gUserInfo.fPropMinCurrent1 + (gUserInfo.fPropMaxCurrent1 -  gUserInfo.fPropMinCurrent1) * gUserInfo.u16ThrottleBDeadZoneMinVal / 100))
					&&(PropValve == gUserInfo.u8LowerPumpType))
					||(OnOffValve == gUserInfo.u8LowerPumpType))
				{
					u8PumpFlag = PUMP_DOWN;
				}		
				break;
			case PUMP_DOWN:
				u16SpeedTarget = u16MotorVal;
				u16Time = gUserInfo.u8CurveLower;
				u8SpeedRate = gUserInfo.u8LowerSpeed;
				if((FunctionEnable == gUserInfo.u8AntiPinchFunc)&&(ABOVE_SWI != u8AntiPinchState))
					u8SpeedRate = gUserInfo.u8BrakeAntiPinch;
				if((1 == sgActLogic.b1LiftUpAct)||(0 == sgActLogic.b1LiftDownAct))
				{
					u16SpeedTarget = 0;
					if(OnOffValve == gUserInfo.u8LowerPumpType)
					{
						u8PumpFlag = PUMP_DOWN_STOP;
					}
					if((((inserted_data[1] * PROP_CURRENT_FACOTR) < (gUserInfo.fPropMinCurrent1 + (gUserInfo.fPropMaxCurrent1 -  gUserInfo.fPropMinCurrent1)* gUserInfo.u16ThrottleBDeadZoneMaxVal / 100))
						||(1 == sgActLogic.b1LiftUpAct))&&(PropValve == gUserInfo.u8LowerPumpType))
						u8PumpFlag = PUMP_DOWN_STOP;					
				}	
				break;
			case PUMP_DOWN_STOP:
				u16SpeedTarget = 0;
				u16Time = gUserInfo.u8BrakeLower;
				u8SpeedRate = gUserInfo.u8LowerSpeed;			
				if((FunctionEnable == gUserInfo.u8AntiPinchFunc)&&(ABOVE_SWI != u8AntiPinchState))
					u8SpeedRate = gUserInfo.u8BrakeAntiPinch;
				if(((inserted_data[1] * PROP_CURRENT_FACOTR) < (gUserInfo.fPropMinCurrent1 + (gUserInfo.fPropMaxCurrent1 -  gUserInfo.fPropMinCurrent1)* gUserInfo.u16ThrottleBMidVal / 100))
					||(1 == sgActLogic.b1LiftUpAct))
				{
					u8PumpFlag = PUMP_STATIC;
				}
				else if(1 == sgActLogic.b1LiftDownAct)
				{
					u8PumpFlag = PUMP_DOWN;
				}
				if(OnOffValve == gUserInfo.u8LowerPumpType)
				{
					u8PumpFlag = PUMP_STATIC;
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
		u16Time = u16Time * TIME_FACTOR;
		u16SpeedTarget = u16SpeedTarget * u8SpeedRate / 100;
		vSpeedControl();
		//23.12.04下降比例阀&开关阀区分
		/*SJ待完善*/
		//开关阀和比例阀的控制区分？
		if(PropValve == gUserInfo.u8LowerPumpType)
		{
			if((PUMP_DOWN == u8PumpFlag)||(PUMP_DOWN_STOP == u8PumpFlag)||(PUMP_DOWN_START == u8PumpFlag))
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
		else//开关阀/*SJ待完善*/
		{
			if((PUMP_DOWN == u8PumpFlag)||(PUMP_DOWN_STOP == u8PumpFlag)||(PUMP_DOWN_START == u8PumpFlag))
			{
				int32_t i32prop = 0;
				i32prop = _IQ((gUserInfo.fPropMinCurrent1 + (gUserInfo.fPropMaxCurrent1 - gUserInfo.fPropMinCurrent1) * MOTOR_MAX_SPEED_VALUE / MOTOR_MAX_SPEED_VALUE) / PROPD_STD_CURRENT);
				if(0 == sgSwiInput.b1DownLimitSwi)//下限位之上，全速下降
				{
					if( u16SpeedCmd > (MOTOR_MAX_SPEED_VALUE/2))
					{
						vPropSetTarget(LIFTDOWN_PUMP1, i32prop);
						vPropSetTarget(LIFTDOWN_PUMP2, i32prop);					
					}
					else
					{
						vPropSetTarget(LIFTDOWN_PUMP1, 0);
						vPropSetTarget(LIFTDOWN_PUMP2, i32prop);
					}
				}
				else//触发下限位,开两个阀
				{
					vPropSetTarget(LIFTDOWN_PUMP1, i32prop);
					vPropSetTarget(LIFTDOWN_PUMP2, i32prop);					
				}
			}
			else
			{
				vPropSetTarget(LIFTDOWN_PUMP1, 0);
				vPropSetTarget(LIFTDOWN_PUMP2, 0);
			}						
		}
	}
	else
	{
		u16SpeedTarget = 0;
		i32DoPwmSet(HIGH_SLOW_SPEED_PUMP,PUMP_CLOSE_PERCENTAGE);
		i32DoPwmSet(BACKWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
		i32DoPwmSet(FORWARD_PUMP,PUMP_CLOSE_PERCENTAGE);
		i32DoPwmSet(TURNLEFT_PUMP,PUMP_CLOSE_PERCENTAGE);
		i32DoPwmSet(TURNRIGHT_PUMP,PUMP_CLOSE_PERCENTAGE);
		i32DoPwmSet(LIFTUP_PUMP,PUMP_CLOSE_PERCENTAGE);
		vPropSetTarget(LIFTDOWN_PUMP1, 0);
		vPropSetTarget(LIFTDOWN_PUMP2, 0);
	}		
	if((u8PumpFlag != PUMP_DOWN_STOP)&&(u8PumpFlag != PUMP_DOWN)&&(u8PumpFlag != PUMP_DOWN_START)
		&&((u8PumpFlag != PUMP_STATIC)||
		((u8MotorFlag != MOTOR_STATIC)||(0 != sgActLogic.b1TurnLeft)||(0 != sgActLogic.b1TurnRight))))//锟斤拷锟铰斤拷状态时锟斤拷锟斤拷锟斤拷指锟斤拷
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
	ActLogicRecord.u8Data = sgActLogic.u8Data;
	
//	i32SetPara(PARA_OnOffValveCurrent,u8PumpFlag);
}
//等柳工完善后CV工程
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
/*待完善*/
//锂电、下控掉线报警
void vCanLostProc(uint32_t u32CanID,uint8_t u8State)
{
	switch(u32CanID)
	{
		case 0x28A://锂电报文，只有在锂电模式下、报文丢失才报警
			if(LiBattery == gUserInfo.u8BatteryType)
			{
				if(u8State == CAN_LOST)
				{
					i32ErrCodeSet(LI_BATTERY_LOSS_ERR);
				}
				if (CAN_NORMAL == u8State)
				{
					 i32ErrCodeClr(LI_BATTERY_LOSS_ERR);
				}					
			}
			break;
		case 0x726:
			if(u8State == CAN_LOST)
			{
				u8HmiCheckFlag = 1;
			}
			if (CAN_NORMAL == u8State)
			{
				 u8HmiCheckFlag = 0;
			}
			break;
		case 0x3D5:
			if(u8State == CAN_LOST)
			{
			 sgLockState.b1HeartBeatLock = 1;
			}
			if (CAN_NORMAL == u8State)
			{
			 sgLockState.b1HeartBeatLock = 0;
			}
			break;
		default:
			break;
	}
}

//T-BOX




static void SDOParaSetsError()
{
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
	
	if(0x40 == CanRev602.u8CS)//读取
	{
		if(ADDR_INVALID != u16ParaAdressAA)
			u8CsReturn += u16ReadParaFromEeprom(u16ParaAdressAA);
		if(ADDR_INVALID != u16ParaAdressBB)
			u8CsReturn += u16ReadParaFromEeprom(u16ParaAdressBB);
		if(ADDR_INVALID != u16ParaAdressCC)
			u8CsReturn += u16ReadParaFromEeprom(u16ParaAdressCC);
		if(ADDR_INVALID != u16ParaAdressDD)
			u8CsReturn += u16ReadParaFromEeprom(u16ParaAdressDD);
		
		if(0 == u8CsReturn)//读取成功
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
	else if(0x23 == CanRev602.u8CS)//写入
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

#if 1
static void vChangeBit(uint32_t *u32data,uint8_t u8Loc1,uint8_t u8Loc2)
{
	static uint32_t u32Mask1 ;
	static uint32_t u32Mask2 ;
	static uint32_t u32Bits1 ;
	static uint32_t u32Bits2 ;
	
	u32Mask1 = 0xF << u8Loc1;
	u32Mask2 = 0xF << u8Loc2;
	u32Bits1 = (*u32data & u32Mask1) >> u8Loc1;
	u32Bits2 = (*u32data & u32Mask2) >> u8Loc2;
	
	*u32data &= ~(u32Mask1);
	*u32data &= ~(u32Mask2);
	*u32data |= (u32Bits1 << u8Loc2);
	*u32data |= (u32Bits2 << u8Loc1);
}

static void vKeyCal(uint32_t *u32Seed,uint32_t *u32Key)
{
	uint32_t u32TempKey;
	
	u32TempKey = *u32Seed ^ 0x936DA8C1;
	
	vChangeBit(&u32TempKey,0,12);
	vChangeBit(&u32TempKey,4,24);
	vChangeBit(&u32TempKey,8,16);
	
	u32TempKey ^= 0x67AA537E;
	
	vChangeBit(&u32TempKey,0,16);
	vChangeBit(&u32TempKey,4,12);
	vChangeBit(&u32TempKey,8,24);
	
	u32TempKey = u32TempKey % 100000000;
	
	*u32Key = u32TempKey;
}

static void vRemoteUnlockKey()//生成解锁密码组合,第一次进入或者输入失败时执行一次
{
	uint32_t u32RandNum = 0;
	static uint16_t u16Seed = 0;
	
	u16EepromRead(RANDOM_SEED_ADDR,&u16Seed,1);
	
	u16Seed = (uint16_t)(u32HourCnt + i32LocalAiGetValue(AI_B_AI1_R)
					+ i32LocalAiGetValue(AI_B_KSI_CHECK) + i32LocalAiGetValue(AI_B_VBUS_CHECK) + u16Seed);
	
	srand(u16Seed);
	u32RandNum = (uint32_t)rand();
	u16Seed = u32RandNum;
	u16EepromWrite(RANDOM_SEED_ADDR,u16Seed,1);
	
	sgRemoteLockKey.u32GPSRandomNum = u32RandNum % 99999999;
	
//	(((u32RandNum >> 28) % 10)<<28)|(((u32RandNum >> 24) % 10)<<24)|
//	(((u32RandNum >> 20) % 10)<<20)|(((u32RandNum >> 16) % 10)<<16)|
//	(((u32RandNum >> 12) % 10)<<12)|(((u32RandNum >> 8) % 10)<<8)|
//	(((u32RandNum >> 4) % 10)<<4)|(((u32RandNum >> 0) % 10)<<0);
	
	srand(u32RandNum);
	u32RandNum = (uint32_t)rand();
	
	sgRemoteLockKey.u32TMPRandomNum = u32RandNum % 99999999;
//	(((u32RandNum >> 28) % 10)<<28)|(((u32RandNum >> 24) % 10)<<24)|
//	(((u32RandNum >> 20) % 10)<<20)|(((u32RandNum >> 16) % 10)<<16)|
//	(((u32RandNum >> 12) % 10)<<12)|(((u32RandNum >> 8) % 10)<<8)|
//	(((u32RandNum >> 4) % 10)<<4)|(((u32RandNum >> 0) % 10)<<0);
	
	
	vKeyCal(&sgRemoteLockKey.u32GPSRandomNum ,&sgRemoteLockKey.u32GPSUnlockKey);
	vKeyCal(&sgRemoteLockKey.u32TMPRandomNum ,&sgRemoteLockKey.u32TempUnlockKey);	
}

static void vLockProc()
{
	static xHeartBeatLock LockRec;
	
	static uint16_t u16TmpUnlockCnt = 0xFFFF;
	
	
	
//	if(FunctionDisable != i32GetPara(PARA_HeartBeatQueryFunc))
//		sgLockState.b1HeartBeatQuery = 1;
//	else
//		sgLockState.b1HeartBeatQuery = 0;
	
//	if(FunctionDisable != i32GetPara(PARA_PasswordLock))
//		sgLockState.b1TempUnlock = 1;
//	else
//		sgLockState.b1TempUnlock = 0;
//	

	
	if(1 == sgLockState.b1TempUnlock)
	{
		if(0xFFFF == u16TmpUnlockCnt)
		{
			u16EepromRead(TMP_UNLOCK_TIME_ADDR,&sgRemoteLockKey.u16UnlockHour,1);
			u16TmpUnlockCnt = 0;
			
		}
		i32SetPara(PARA_PropValveCurrent,sgRemoteLockKey.u16UnlockHour);
		 
		if(u16TmpUnlockCnt > (UNLOCK_CNT_TIME / USER_ECU_PERIOD))//
		{
			u16TmpUnlockCnt = 0;
			
			if(sgRemoteLockKey.u16UnlockHour)
				sgRemoteLockKey.u16UnlockHour--;
			else
				sgLockState.b1TempUnlock = 0;
					
			u16EepromWrite(TMP_UNLOCK_TIME_ADDR,sgRemoteLockKey.u16UnlockHour,1);
		}
		else if (0 != sgActLogic.u8Data)//临时解锁，动作计时
		{
			u16TmpUnlockCnt++ ;
		}
	}
	
	if((1 == sgLockState.b1HeartBeatLock)
		&&(1 == sgLockState.b1HeartBeatQuery))
		i32ErrCodeSet(GPS_CONNECT_ERR);
	else
		i32ErrCodeClr(GPS_CONNECT_ERR);
	
	if((1 == sgLockState.b1LockLevel1)
		&&(0 == sgLockState.b1TempUnlock))
		i32ErrCodeSet(PLATFORM_LEVEL1_LOCK_ERR);
	else
		i32ErrCodeClr(PLATFORM_LEVEL1_LOCK_ERR);
	
	if((1 == sgLockState.b1LockLevel2)
		&&(0 == sgLockState.b1TempUnlock))
		i32ErrCodeSet(PLATFORM_LEVEL2_LOCK_ERR);
	else
		i32ErrCodeClr(PLATFORM_LEVEL2_LOCK_ERR);
	
		
	if(LockRec.u8Data != sgLockState.u8Data)
	{
		LockRec.u8Data = sgLockState.u8Data;
		u16SaveParaToEeprom(PARA_RemotePara,LockRec.u8Data);
	}
	
}

#endif


static void vUserSetsInit();
/* 待完成
	2000上限位、下限位缺乏参数、超载可下降参数、
	2001速度设置，最小速度、版本信息，ecu，mcu版本
	2002手柄标定、角度模拟
	2004解锁功能、GPS
	*/
static void CanId602Rev(tCanFrame * Can602)//模拟SDO
{
	tCanFrame CanSendData;
	uint8_t i = 0;
	
	memcpy((char*)CanRev602.u8Data, (char*) Can602->u8Data, sizeof(CanRev602));

	memset(CanSend582.u8Data,0,sizeof(CanSend582));
	
	CanSend582.u8IndexH = CanRev602.u8IndexH; 
	CanSend582.u8IndexL = CanRev602.u8IndexL;
	CanSend582.u8SubIndexL = CanRev602.u8SubIndexL;
	
	

	uint16_t u16Index = 0;
	uint8_t u8SubIndex = 0;
	uint32_t u32Data  = 0;

	u16Index = Can602->u8Data[1] |( Can602->u8Data[2]<<8);
	u8SubIndex = Can602->u8Data[3];

	if(0x2000 == u16Index)
	{
		switch (u8SubIndex)
		{
			case 1://车辆类型
				vSDOParaSets(PARA_VehicleType,ADDR_INVALID,ADDR_INVALID,ADDR_INVALID);
				break;
			case 2://泵电机、行走电机
				vSDOParaSets(PARA_PumpMotorEncoder,PARA_DriveMotorEncoder,ADDR_INVALID,ADDR_INVALID);
				break;
			case 3://坑洞、防夹手、动作报警
				vSDOParaSets(PARA_PitProtectFunc,PARA_AntiPinchFunc,PARA_ActAlmFunc,PARA_AnticollisionFunc);
				break;
			case 4://上限位使能、下限位使能、倾斜检测、举升反向			
				vSDOParaSets(ADDR_INVALID,ADDR_INVALID,PARA_TiltSwitchSetting,PARA_LiftReverseFunc);			
				if(0x40 == CanRev602.u8CS)//读取
				{
					
					CanSend582.u8DataAA = sgUserSets.b1UpLimitEn;
					CanSend582.u8DataBB = sgUserSets.b1LowLimitEn;
				}
				else if(0x23 == CanRev602.u8CS)//写入
				{
					sgUserSets.b1UpLimitEn = CanSend582.u8DataAA;
					sgUserSets.b1LowLimitEn = CanSend582.u8DataBB;
					u16SaveParaToEeprom(PARA_USERSETS,sgUserSets.u16Data);
				}
				break;
			case 5://电池类型、下降阀类型、GPS防拆、临时解锁
				vSDOParaSets(ADDR_INVALID,PARA_LowerPumpType,PARA_HeartBeatQueryFunc,PARA_PasswordLock);
				/*电池类型特殊处理*/
				if(0x40 == CanRev602.u8CS)//读取
				{
					if(LiBattery ==  i32GetPara(PARA_BatteryType))
						CanSend582.u8DataAA = 1;
					else
						CanSend582.u8DataAA = 0;
					
					if(1 ==  sgLockState.b1HeartBeatQuery)//CC
						CanSend582.u8DataCC = 1;
					else
						CanSend582.u8DataCC = 0;
					
					if(1 == sgLockState.b1TempUnlock)     //DD
						CanSend582.u8DataDD = 1;
					else
						CanSend582.u8DataDD = 0;
				}
				else if(0x23 == CanRev602.u8CS)//写入
				{
					if (1 == CanSend582.u8DataAA)
						u16SaveParaToEeprom(PARA_BatteryType,LiBattery);
					else
						u16SaveParaToEeprom(PARA_BatteryType,QiuJian);
					if(1 == CanSend582.u8DataCC)
					{
						sgLockState.b1HeartBeatQuery = 1;
						u16SaveParaToEeprom(PARA_RemotePara,sgLockState.u8Data);
					}
					else
					{
						sgLockState.b1HeartBeatQuery = 0;
						u16SaveParaToEeprom(PARA_RemotePara,sgLockState.u8Data);						
					}
					
					if(1 == CanSend582.u8DataDD)
					{
						sgLockState.b1TempUnlock = 1;
						u16SaveParaToEeprom(PARA_RemotePara,sgLockState.u8Data);
					}
					else
					{
						sgLockState.b1TempUnlock = 0;
						u16SaveParaToEeprom(PARA_RemotePara,sgLockState.u8Data);						
					}
				}
				if(sgLockState.b1TempUnlock == 1)//重置计时
				{
					sgRemoteLockKey.u16UnlockHour = UNLOCK_TIME;
					u16EepromWrite(TMP_UNLOCK_TIME_ADDR,sgRemoteLockKey.u16UnlockHour,1);
				}
				break;
			case 6://语音报警电量、上控按键休眠、超载可下降高度
				vSDOParaSets(PARA_VoiceAlarmVolume,PARA_UpperCtlButSleep,PARA_SetDescentHeightValue,ADDR_INVALID);
				break;
			case 7://称重功能
				vSDOParaSets(PARA_WeighFunc,ADDR_INVALID,ADDR_INVALID,ADDR_INVALID);
				break;
			case 8://超载延时、动态超载百分比、静态超载百分比、最大差值百分比
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
			case 1://高速斜率、高速最大值、最小值、减速斜率
				vSDOParaSets(PARA_AccAndDecFastDrive,PARA_FastDriveSpeed,ADDR_INVALID,PARA_BrakeFastDrive);
				break;
			case 2://低速
				vSDOParaSets(PARA_AccAndDecSlowDrive,PARA_SlowDriveSpeed,ADDR_INVALID,PARA_BrakeSlowDrive);
				break;
			case 3://起升后速度
				vSDOParaSets(PARA_AccAndDecAfterLift,PARA_DriveSpeedAfterLift,ADDR_INVALID,PARA_BrakeDriveAfterLift);
				break;
			case 4://起升速度
				vSDOParaSets(PARA_AccAndDecLift,PARA_LiftSpeed,ADDR_INVALID,PARA_BrakeLift);
				break;
			case 5://转向速度
				vSDOParaSets(PARA_AccAndDecTurn,PARA_MaxTurnSpeed,ADDR_INVALID,PARA_BrakeTurn);
				break;
			case 6://转向动力、下降速度
				vSDOParaSets(PARA_TurnPowerLimit,PARA_LowerSpeed,ADDR_INVALID,ADDR_INVALID);
				break;
			case 7://ECU版本
				vSDOParaSets(ADDR_INVALID,ADDR_INVALID,ADDR_INVALID,ADDR_INVALID);
				CanSend582.u8DataAA = ECU_SOFTVERSION >> 24;
				CanSend582.u8DataBB = ECU_SOFTVERSION >> 16;
				CanSend582.u8DataCC = ECU_SOFTVERSION >> 8;
				CanSend582.u8DataDD = ECU_SOFTVERSION >> 0;
				break;
			case 8://MCU版本
				vSDOParaSets(ADDR_INVALID,ADDR_INVALID,ADDR_INVALID,ADDR_INVALID);
				CanSend582.u8DataAA = MCU_SOFTVERSION >> 24;
				CanSend582.u8DataBB = MCU_SOFTVERSION >> 16;
				CanSend582.u8DataCC = MCU_SOFTVERSION >> 8;
				CanSend582.u8DataDD = MCU_SOFTVERSION >> 0;
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
			case 1://手柄标定，上端				
				vSDOParaSets(ADDR_INVALID,ADDR_INVALID,ADDR_INVALID,ADDR_INVALID);
				if(0x40 == CanRev602.u8CS)//读取
				{
					CanSend582.u8DataAA = sgPcuHandle.i8PositiveValue;
					CanSend582.u8DataCC = sgPcuInfo.i8HandleRaw;
				}
				else if(0x23 == CanRev602.u8CS)//写入
				{
					sgPcuHandle.i8PositiveValue = sgPcuInfo.i8HandleRaw;
					u16SaveParaToEeprom(PARA_HANDLEMAX,sgPcuHandle.i8PositiveValue);
				}
				break;
			case 2://手柄标定，中位
				vSDOParaSets(ADDR_INVALID,ADDR_INVALID,ADDR_INVALID,ADDR_INVALID);
				if(0x40 == CanRev602.u8CS)//读取
				{
					CanSend582.u8DataAA = sgPcuHandle.i8MiddleValue;
					CanSend582.u8DataCC = sgPcuInfo.i8HandleRaw;
				}
				else if(0x23 == CanRev602.u8CS)//写入
				{
					sgPcuHandle.i8MiddleValue = sgPcuInfo.i8HandleRaw;
					u16SaveParaToEeprom(PARA_HANDLEMID,sgPcuHandle.i8MiddleValue);
				}
				break;
			case 3://手柄标定，下端
				vSDOParaSets(ADDR_INVALID,ADDR_INVALID,ADDR_INVALID,ADDR_INVALID);
				if(0x40 == CanRev602.u8CS)//读取
				{
					CanSend582.u8DataAA = sgPcuHandle.i8NegativeValue;
					CanSend582.u8DataCC = sgPcuInfo.i8HandleRaw;
				}
				else if(0x23 == CanRev602.u8CS)//写入
				{
					sgPcuHandle.i8NegativeValue = sgPcuInfo.i8HandleRaw;
					u16SaveParaToEeprom(PARA_HANDLEMIN,sgPcuHandle.i8NegativeValue);
				}
				break;
			case 4://称重标定、空载
				vSDOParaSets(ADDR_INVALID,ADDR_INVALID,ADDR_INVALID,ADDR_INVALID);
				break;
			case 5://称重标定、满载
				vSDOParaSets(ADDR_INVALID,ADDR_INVALID,ADDR_INVALID,ADDR_INVALID);
				break;
			case 6://角度模拟上限位
			if(0x40 == CanRev602.u8CS)//读取
			{
				CanSend582.u8CS = 0x43;
				CanSend582.u8DataAA = gUserInfo.u16AngleSimulationUpLimit >> 8;
				CanSend582.u8DataBB = gUserInfo.u16AngleSimulationUpLimit >> 0;
			}
			else if(0x23 == CanRev602.u8CS)//写入
			{
				gUserInfo.u16AngleSimulationUpLimit = i32LocalAiGetValue(ANGLE_SENSOR_CHANNEL);
				u16SaveParaToEeprom(PARA_AngleSimulationUpLimit,gUserInfo.u16AngleSimulationUpLimit);
				CanSend582.u8CS = 0x60;
				CanSend582.u8DataAA = gUserInfo.u16AngleSimulationUpLimit >> 8;
				CanSend582.u8DataBB = gUserInfo.u16AngleSimulationUpLimit >> 0;
			}
				break;
			case 7://角度模拟下限位
			if(0x40 == CanRev602.u8CS)//读取
			{	
				CanSend582.u8CS = 0x43;
				CanSend582.u8DataAA = gUserInfo.u16AngleSimulationDownLimit >> 8;
				CanSend582.u8DataBB = gUserInfo.u16AngleSimulationDownLimit >> 0;
			}
			else if(0x23 == CanRev602.u8CS)//写入
			{
				gUserInfo.u16AngleSimulationDownLimit = i32LocalAiGetValue(ANGLE_SENSOR_CHANNEL);
				u16SaveParaToEeprom(PARA_AngleSimulationDownLimit,gUserInfo.u16AngleSimulationDownLimit);
				CanSend582.u8CS = 0x60;
				CanSend582.u8DataAA = gUserInfo.u16AngleSimulationDownLimit >> 8;
				CanSend582.u8DataBB = gUserInfo.u16AngleSimulationDownLimit >> 0;
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
			case 1://历史故障1-4
				vSDOParaSets(PARA_ErrCode0,PARA_ErrCode1,PARA_ErrCode2,PARA_ErrCode3);
				break;
			case 2://历史故障5-8
				vSDOParaSets(PARA_ErrCode4,PARA_ErrCode5,PARA_ErrCode6,PARA_ErrCode7);
				break;
			case 3://历史故障9-10
				vSDOParaSets(PARA_ErrCode8,PARA_ErrCode9,ADDR_INVALID,ADDR_INVALID);
				break;
			default:
				SDOParaSetsError();
				break;
		}
	}
	else if(0x2004 == u16Index)
	{
		if(0x40 == CanRev602.u8CS)CanSend582.u8CS = 0x43;
		if(0x23 == CanRev602.u8CS)CanSend582.u8CS = 0x60;
		switch (u8SubIndex)
		{
			case 1://紧急解锁申请码//暂无需求
				vSDOParaSets(ADDR_INVALID,ADDR_INVALID,ADDR_INVALID,ADDR_INVALID);
				break;
			case 2://永久解锁申请码
				vSDOParaSets(ADDR_INVALID,ADDR_INVALID,ADDR_INVALID,ADDR_INVALID);
				break;
			case 3://紧急解锁验证码
				vSDOParaSets(ADDR_INVALID,ADDR_INVALID,ADDR_INVALID,ADDR_INVALID);
				break;
			case 4://永久解锁验证码
				vSDOParaSets(ADDR_INVALID,ADDR_INVALID,ADDR_INVALID,ADDR_INVALID);
				break;
			case 5://GPS状态//未知状态
				CanSend582.u8DataAA = 0;
				break;
			case 6://gpsID 1-4
				CanSend582.u8DataAA = sg3700MessageInfos[0].Message[0];
				CanSend582.u8DataBB = sg3700MessageInfos[0].Message[1];
				CanSend582.u8DataCC = sg3700MessageInfos[0].Message[2];
				CanSend582.u8DataDD = sg3700MessageInfos[0].Message[3];			
				break;
			case 7://gpsID 5-8
				CanSend582.u8DataAA = sg3700MessageInfos[0].Message[4];
				CanSend582.u8DataBB = sg3700MessageInfos[0].Message[5];
				CanSend582.u8DataCC = sg3700MessageInfos[0].Message[6];
				CanSend582.u8DataDD = sg3700MessageInfos[0].Message[7];
				break;
			case 8://gpsID 9-12
				CanSend582.u8DataAA = sg3700MessageInfos[0].Message[8];
				CanSend582.u8DataBB = sg3700MessageInfos[0].Message[9];
				break;
			default:
				SDOParaSetsError();
				break;
		}
	}
	else if(0x2005 == u16Index)
	{
		if(0x40 == CanRev602.u8CS)CanSend582.u8CS = 0x43;
		if(0x23 == CanRev602.u8CS)CanSend582.u8CS = 0x60;
		switch (u8SubIndex)
		{
			case 1://GPS随机码
				vRemoteUnlockKey();
				CanSend582.u8DataAA = sgRemoteLockKey.u32GPSRandomNum >> 24;
				CanSend582.u8DataBB = sgRemoteLockKey.u32GPSRandomNum >> 16;
				CanSend582.u8DataCC = sgRemoteLockKey.u32GPSRandomNum >> 8;
				CanSend582.u8DataDD = sgRemoteLockKey.u32GPSRandomNum >> 0;
				break;
			case 2://GPS密码
					u32Data = (uint32_t)(CanRev602.u8DataAA << 24 )|(uint32_t)(CanRev602.u8DataBB << 16 )|
											(uint32_t)(CanRev602.u8DataCC << 8 )|(uint32_t)(CanRev602.u8DataDD << 0 );
				if(sgRemoteLockKey.u32GPSUnlockKey == u32Data)
				{
					CanSend582.u8DataAA = 01;
				}
				else
				{
					CanSend582.u8DataAA = 00;
				}
				break;
			case 3://临时解锁随机码
				vRemoteUnlockKey();
				CanSend582.u8DataAA = sgRemoteLockKey.u32TMPRandomNum >> 24;
				CanSend582.u8DataBB = sgRemoteLockKey.u32TMPRandomNum >> 16;
				CanSend582.u8DataCC = sgRemoteLockKey.u32TMPRandomNum >>  8;
				CanSend582.u8DataDD = sgRemoteLockKey.u32TMPRandomNum >> 0;
				break;
			case 4://临时解锁密码
					u32Data = (uint32_t)(CanRev602.u8DataAA << 24 )|(uint32_t)(CanRev602.u8DataBB << 16 )|
											(uint32_t)(CanRev602.u8DataCC << 8 )|(uint32_t)(CanRev602.u8DataDD << 0 );
				if(sgRemoteLockKey.u32TempUnlockKey == u32Data)
				{
					CanSend582.u8DataAA = 01;
				}
				else
				{
					CanSend582.u8DataAA = 00;
				}
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
	
	vUserSetsInit();
	#if 0
	{
		tCanFrame Can666;
		memset(Can666.u8Data,0,sizeof(Can666.u8Data));
		Can666.u32ID = 0x666;
		Can666.u16DataLength = 8;
		Can666.u8Rtr = 0;
		memcpy(Can666.u8Data,&sgRemoteLockKey.u32GPSRandomNum,sizeof(sgRemoteLockKey.u32GPSRandomNum));
		Can666.u8Data[7] = 1;
		i32CanWrite(Can0, &Can666);
				
		memcpy(Can666.u8Data,&sgRemoteLockKey.u32TMPRandomNum,sizeof(sgRemoteLockKey.u32GPSRandomNum));
		Can666.u8Data[7] = 2;
		i32CanWrite(Can0, &Can666);
				
		memcpy(Can666.u8Data,&sgRemoteLockKey.u32GPSUnlockKey,sizeof(sgRemoteLockKey.u32GPSRandomNum));
		Can666.u8Data[7] = 3;
		i32CanWrite(Can0, &Can666);
				
		memcpy(Can666.u8Data,&sgRemoteLockKey.u32TempUnlockKey,sizeof(sgRemoteLockKey.u32GPSRandomNum));
		Can666.u8Data[7] = 4;
		i32CanWrite(Can0, &Can666);
	}
	#endif
}


static void StartCommandSend()
{
	static uint8_t u8StartCnt = 0;
	uint8_t start_command_data[2] = {0x01, 0x00};
	tCanFrame CanframeSend;
		
	if(u8StartCnt > 100)
	{
		if ((u8HmiCheckFlag))
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
		u8StartCnt = 0;
	}
	else
		u8StartCnt++;

}

static void HMISend()
{
	static xCanSend5A1 CanSend5A1 ;
	static xCanSend5A2 CanSend5A2 ;
	static xCanSend5A3 CanSend5A3 ;
	static xCanSend5A4 CanSend5A4 ;
	static xCanSend5A6 CanSend5A6 ;
	static xCanSend5A7 CanSend5A7 ;
	static xCanSend5A8 CanSend5A8 ;
	static xCanSend5A9 CanSend5A9 ;
	static xCanSend5AA CanSend5AA ;
	static xCanSend5AB CanSend5AB ;

	
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
		CanSend5A1.b1BackValveState = ~i32LocalDiGet(BACKWARD_PUMP_R);
		CanSend5A1.b1ForwardValveState = ~i32LocalDiGet(FORWARD_PUMP_R);
		CanSend5A1.b1HighLowSpeedValveState = ~i32LocalDiGet(HIGH_SLOW_SPEED_PUMP_R);
		CanSend5A1.b1LiftValveState = ~i32LocalDiGet(LIFTUP_PUMP_R);
		CanSend5A1.b1TurnLeftValveState = ~i32LocalDiGet(TURNLEFT_PUMP_R);
		CanSend5A1.b1TurnRightValveState = ~i32LocalDiGet(TURNRIGHT_PUMP_R);	
		
		CanSend5A1.b1PropValve1State = ~i32LocalDiGet(LIFTUP_PUMP_R);
		CanSend5A1.b1PropValve2State = ~i32LocalDiGet(BACKWARD_PUMP_R);
		
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
		CanSend5A2.u8PumpMotorSpeed = (u16MotorFdb * 100 / gUserInfo.u16MotorMaxSpd) & 0xFF;
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
		CanSend5A4.u8MainContacterState = 1;//待确定
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
		if(0 != u8ErrCodeGet())
			CanSend5AB.u8ErrorNum = 1;
		CanSend5AB.u8Error1 = u8ErrCodeGetTrans(); 
//		CanSend5AB.u8Error2 = i32GetPara(PARA_ErrCode1); 
//		CanSend5AB.u8Error3 = i32GetPara(PARA_ErrCode2); 
//		CanSend5AB.u8Error4 = i32GetPara(PARA_ErrCode3); 
//		CanSend5AB.u8Error5 = i32GetPara(PARA_ErrCode4); 
//		CanSend5AB.u8Error6 = i32GetPara(PARA_ErrCode5); 
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
		case 20:
			u8SendCnt = 0;
		default:
			CanframeSend.u32ID = 0;
			break;
	}
	if(0 != CanframeSend.u32ID)
		i32CanWrite(Can0, &CanframeSend);
}

void BlockTransInit()
{
	xUserParaSets BlockParaSet;
	BlockParaSet.b1ClinetCrcEn = 1;
	BlockParaSet.b1ClinetDataSizeEn = 1;
	BlockParaSet.u16RevId = 0x5D5;
	BlockParaSet.u16SendId = 0x655;
	BlockParaSet.u8BlockSize = 0x7F;
	
	vBlockTransParaSet(&BlockParaSet);
}

static uint8_t u8PlatCmd;

#if 1  //旧的协议方案

static void CanId3D5Rev(tCanFrame *CanData)
{
	static uint8_t u8CmdCnt;
	if(u8CmdCnt != CanData->u8Data[4])
	{
//		u8PlatCmd = 1;
		u8CmdCnt = CanData->u8Data[4];
	}
}

static uint16_t u16CrcModBus(uint8_t * u8Data,uint16_t u16Length)
{
	uint16_t CrcValue = 0;
	uint8_t i ,j =0;
	CrcValue = 0xFFFF;
	for( i = 0 ; i < u16Length; i++)
	{
		CrcValue ^= *(u8Data + 1);
		for(j = 0; j < 8; j++)
		{
			if(CrcValue & 0x0001)
				CrcValue = (CrcValue >> 1) ^ 0xA001;
			else
				CrcValue = (CrcValue >> 1);
		}
	}
	CrcValue = ((CrcValue >> 8)) + (CrcValue << 8);
	return CrcValue;
}	



static void vDataFresh()
{  
	uint16_t u16GPSID = 0;
	/*保存GPSID到eeprom*/
	{
		u16GPSID = i32GetPara(PARA_USER_DATA_25);
		if(u8M370001[9]!= u16GPSID) //上次GPSID不为0 或收到的GPSID与上次不同，将ID写入到EEPROM，从EEPROM里读取ID
		{
			uint16_t CheckSum = 0;
			for(int i=0;i<10;i++)
			{
				CheckSum += u8M370001[i];
			}
			if(CheckSum != 0)
			{
				u16SaveParaToEeprom(PARA_USER_DATA_25,u8M370001[9]);
				u16GPSID = u8M370001[9];
			}
		}
	}
	
	{
//		u8M371002.u8CmdType = sgRemotCmd.u8RevCmdCode;
//		u8M371002.u8CmdLenth = sgRemotCmd.u8SendCmdLength;
//		memcpy(u8M371002.u8CmdData,&sgRemotCmd.u8SendCmdData,sgRemotCmd.u8SendCmdLength);
		
		sg3710MessageInfos[1].Message[10] = 1;
		sg3710MessageInfos[1].Message[11] = sg3719MessageInfos[3].Message[11];//命令代码
		sg3710MessageInfos[1].Message[12] = 3;//命令代码
		sg3710MessageInfos[1].Message[13] = sg3719MessageInfos[3].Message[13];
		sg3710MessageInfos[1].Message[14] = sg3719MessageInfos[3].Message[14];
		sg3710MessageInfos[1].Message[15] = 1;
		sg3710MessageInfos[1].Message[16] = u16CrcModBus(sg3710MessageInfos[1].Message,17);
		sg3710MessageInfos[1].u32Length = u8M371002.u8CmdLenth + 13;
	}
	
	
	{
		/*需要变动的参数：系统时间，解锁剩余天数，解锁私钥，车辆状态*/
		if( sgLockState.b1LockLevel1 || sgLockState.b1LockLevel2)
		{
			u8M371102.u8VehicleStatus = 0x11 ;
			if(sgLockState.b1LockLevel2)
				u8M371102.u8VehicleStatus = 0x12 ;
			if(sgLockState.b1TempUnlock)
				u8M371102.u8VehicleStatus += 0x10;
		}
		else
			u8M371102.u8VehicleStatus = 0;
	}
	
	/*工况数据待填充*/
	{
	//	u8M371202.
	}
	/*调试参数待填充*/
	{
		//u8M371302.
	}
	/*端口数据待填充*/
	{
		//u8M371402.
	}
	/**/
	{
	}

	/*18，目前只有接收的数据*/
	{
	}
	/*19*/
	{
		memcpy(sg3719MessageInfos[0].Message,TOPIC_1901,14);
		memcpy(sg3719MessageInfos[0].Message + 14,sg3700MessageInfos[0].Message,10);//获取的GPS编号
		sg3719MessageInfos[0].u32Length = 24;
	}
	/*20 协议内没有，截取的报文里有*/
	/*GPSid传输完成，才进入*/
	//if(TRANSMIT_SUCCESS == sg3700MessageInfos[0].TransmitState)
	{
		uint16_t u16SumCheck = 0;
		uint8_t i;
		for(i = 0; i < 11 ;i++)//0到11的和
		{
			u16SumCheck += u8M372001[i];
		}
		u16SumCheck = (u16SumCheck + u16GPSID);//加上GPS序列号最后一个
		i32SetPara(PARA_TurnLeftValveCurrent,u8M372001[11]);
		i32SetPara(PARA_TurnRightValveCurrent,u16SumCheck);
		if((u8M372001[11] == (u16SumCheck & 0xFF)) && (u16SumCheck != 0))//最后一位用于校验，通过校验处理数据
		{
			if(u8M372001[0] & 0b0001)//一级锁车
			{
				if(0 == sgLockState.b1LockLevel1)
				{
					sgLockState.b1LockLevel1 = 1;
					u16SaveParaToEeprom(PARA_RemotePara,sgLockState.u8Data);
				}
			}
			if(u8M372001[0] & 0b0010)//二级锁车
			{
				if(0 == sgLockState.b1LockLevel2)
				{
					sgLockState.b1LockLevel2 = 1;
					u16SaveParaToEeprom(PARA_RemotePara,sgLockState.u8Data);
				}				
			}
			if(u8M372001[0] & 0b0100)//临时解锁
			{
				/*存疑 计时是否接收到新的临时解锁指令后更新？
				暂无此需求 临时解锁仅用下控操作，下控再次输入临时解锁后刷新时间
				*/
				sgRemoteLockKey.u16UnlockHour = UNLOCK_TIME;
				if(0 == sgLockState.b1TempUnlock)
				{
					sgLockState.b1TempUnlock = 1;
					u16SaveParaToEeprom(PARA_RemotePara,sgLockState.u8Data);
					u16EepromWrite(TMP_UNLOCK_TIME_ADDR,sgRemoteLockKey.u16UnlockHour,1);
				}
			}
			if(0 == u8M372001[0])//平台解锁
			{
				if(0 != sgLockState.b1TempUnlock 
					+ sgLockState.b1LockLevel1 
					+ sgLockState.b1LockLevel2)
				{
					sgLockState.b1TempUnlock = 0;
					sgLockState.b1LockLevel1 = 0;
					sgLockState.b1LockLevel2 = 0;
					u16SaveParaToEeprom(PARA_RemotePara,sgLockState.u8Data);
				}
			}
		}
		i32SetPara(PARA_OnOffValveCurrent,sgLockState.u8Data);
	}
}


static  void vTBoxProc(void)
{
	static uint8_t u8SendState;
	static uint16_t u16LockCmd;
	static uint16_t u16SendCmdCnt;
	
	if((0 == u16SendCmdCnt))//初始时请求一次GPSID 
	{
		u8SendState = GPS_DATA_0001;
	}
	
	
	if(u16SendCmdCnt < 60000 / USER_ECU_PERIOD)
	{
		u16SendCmdCnt++;
	}
	else
	{
		u16SendCmdCnt = 1;
	}
	
	if(0 == u16SendCmdCnt % (5100 / USER_ECU_PERIOD))//5m查询一次
	{
		if((TRANSMIT_IDLE == BlockStateCheck())
			&&(BLOCK_TRANSMIT_IDLE == u8SendState))
		u8SendState = CMD_SYS_QUERY_2001;
	}
	else if(0 == u16SendCmdCnt % (30000 / USER_ECU_PERIOD))//
	{
		if((TRANSMIT_IDLE == BlockStateCheck())
			&&(BLOCK_TRANSMIT_IDLE == u8SendState))
		u8SendState = SYS_DATA_1201;
	}
	
	/**/
//	if(TRANSMIT_SUCCESS != sg3700MessageInfos[0].TransmitState)    //gps数据无效，再次查询
//	{
//		 u8SendState  = GPS_DATA_0001;
//	}
//	
	#if 0
	else if(0 == u16SendCmdCnt % (45000 / USER_ECU_PERIOD))
	{
		if((TRANSMIT_IDLE == BlockStateCheck())
			&&(BLOCK_TRANSMIT_IDLE == u8SendState))
		u8SendState = CMD_SYS_QUERY_1904;
	}
	#endif
		

	if(TRANSMIT_IDLE == BlockStateCheck())
	{
		/*处理接收的数据以及更新发送的数据*/
		vDataFresh();

		#if 0
		if(1 == u8PlatCmd)
		{
			u8SendState = GPS_DATA_0001;
			u8PlatCmd = 2;
		}
		if(3 == u8PlatCmd)//处理接收到的信息
		{
			if(5 == sg3719MessageInfos[3].Message[11]//锁车指令
				&& 2 == sg3719MessageInfos[3].Message[12])//指令长度
			{
				u16LockCmd =  (uint16_t)sg3719MessageInfos[3].Message[13] << 8 | sg3719MessageInfos[3].Message[14];
				switch(u16LockCmd)
				{	
					case 0x0002://永久解锁
						sgLockState.b1LockLevel1 = 0;
						sgLockState.b1LockLevel2 = 0;
						sgLockState.b1PermaneUnlock = 1;
						u16SaveParaToEeprom(PARA_RemotePara,sgLockState.u8Data);
						break;
					case 0x0201://二级锁车
						sgLockState.b1LockLevel2 = 1;
						sgLockState.b1PermaneUnlock = 0;
						u16SaveParaToEeprom(PARA_RemotePara,sgLockState.u8Data);
						break;
					case 0x0101://一级锁车
						sgLockState.b1LockLevel1 = 1;
						sgLockState.b1PermaneUnlock = 0;
						u16SaveParaToEeprom(PARA_RemotePara,sgLockState.u8Data);						
						break;
					default:
						break;
				}
			}	
			u8PlatCmd = 0;
			u8SendState = SYS_ECHO_1001;
		}
		#endif 
		
		switch(u8SendState)
		{
			case BLOCK_TRANSMIT_IDLE:
				break;
			case GPS_DATA_0001://上电查询一次
				BlockSend(&sg3700MessageInfos[0]);
//				if(2 == u8PlatCmd)
//					u8SendState = CMD_SYS_QUERY_1801;
//				else
			u8SendState = SYS_INFO_1101;
				break;
			case SYS_INFO_1101://查询完GPS后发送
				BlockRev(&sg3711MessageInfos[0]);
				u8SendState = SYS_INFO_1102;				
				break;
			case SYS_INFO_1102://查询完GPS后发送
				BlockRev(&sg3711MessageInfos[1]);
				u8SendState = BLOCK_TRANSMIT_IDLE;					
				break;
			case SYS_DATA_1201://周期触发-数据标题
				BlockRev(&sg3712MessageInfos[0]);
				u8SendState = SYS_DATA_1202;						
				break;
			case SYS_DATA_1202://周期触发-数据内容
				BlockRev(&sg3712MessageInfos[1]);
				u8SendState = BLOCK_TRANSMIT_IDLE;						
				break;
			case SYS_PARA_1301://被动发送参数-触发机制位置
				BlockRev(&sg3713MessageInfos[0]);
				u8SendState = SYS_PARA_1302;							
				break;
			case SYS_PARA_1302://参数-内容
				BlockRev(&sg3713MessageInfos[1]);
				u8SendState = BLOCK_TRANSMIT_IDLE;	
				break;
			case SYS_PORT_1401://端口-被动-触发条件未知
				BlockRev(&sg3714MessageInfos[0]);
				u8SendState = SYS_PORT_1402;		
				break;
			case SYS_PORT_1402://端口-内容
				BlockRev(&sg3714MessageInfos[1]);
				u8SendState = BLOCK_TRANSMIT_IDLE;	
				break;
			case CMD_SYS_QUERY_1901://订阅主题
				BlockRev(&sg3719MessageInfos[0]);
				u8SendState = CMD_SYS_QUERY_1904;
				break;
			case CMD_SYS_QUERY_1801://订阅主题
				BlockRev(&sg3718MessageInfos[0]);
				u8SendState = CMD_SYS_QUERY_1901;
				break;
			case CMD_SYS_QUERY_1904://查询
				BlockSend(&sg3719MessageInfos[3]);
				u8SendState = BLOCK_TRANSMIT_IDLE;
//				u8PlatCmd = 3;
				break;
			case SYS_ECHO_1001://反馈-标题
				BlockRev(&sg3710MessageInfos[0]);
				u8SendState = SYS_ECHO_1002;
				break;
			case SYS_ECHO_1002://反馈-内容
				BlockRev(&sg3710MessageInfos[1]);
				u8SendState = BLOCK_TRANSMIT_IDLE;
				break;
			case CMD_SYS_QUERY_2001://请求锁车内容
				BlockSend(&sg3720MessageInfos[0]);
				u8SendState = BLOCK_TRANSMIT_IDLE;
				break;
			default:
				break;
		}
	}
	
}



#endif 


static void vClearBootFlag()
{
	vFlashInfoErase(UDS_UDDATE_FALGE_ADDR,4);
}


typedef void (*func_ptr_t)(void);
typedef  void (*pFunction)(void);
uint32_t JumpAddress;
func_ptr_t JumpToApplication;
uint32_t msp;
static void JumpToBoot2()
{
	JumpAddress = *(__IO uint32_t*) (SECOND_LEVEL_BOOT_PARA_ADDR + 4);
	JumpToApplication = (pFunction) JumpAddress;
	/*disable systick interrupt*/
	//SysTick->CTRL = 0;
	/* Initialize user application's Stack Pointer */
	msp = *(__IO uint32_t*)SECOND_LEVEL_BOOT_PARA_ADDR;
	
	__set_MSP(msp);
	JumpToApplication();
}

static void vSoftReset(void)
{
	__set_FAULTMASK(1);
	NVIC_SystemReset();
}

static void vUpdateFlashFlag()
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

static void CanIdCD4FDFDRev(tCanFrame *CanRev)//模拟SDO
{
	if((0x02== CanRev->u8Data[0])&&(0x10== CanRev->u8Data[1])&&(0x03 == CanRev->u8Data[2]))
	{
//		vSetBootUpdateFlag();
		vUpdateFlashFlag();
		__disable_irq();
		vSoftReset();
//		JumpToBoot2();
	}	
}


static void vUserSetsInit()
{
	
	sgUserSets.u16Data = i32GetPara(PARA_USERSETS);
	if(0 == sgUserSets.b1StartFast)//启动低速
		sgLimit.b1Slow = 1;
	if(1 == sgUserSets.b1StartWithMode)
		 u8PcuMode	= MOVE_MODE;
	
	if(1 == sgUserSets.b1Err18LiftAllow)
	{
		sgErrCodeInfo[117].b1NoUp = 0;
	}
	if(11 != i32GetPara(PARA_HourCountPowerOn))//24.3.12固定小时计为动作计时
	{
		u16SaveParaToEeprom(PARA_HourCountPowerOn,11);
	}
	
	//24.3.18关联参数设置
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
		else//检测关闭
		{
			if(0 != (u16tmp & (1<<10)))
			{
				u16tmp &=~ (1<<10);
				u16SaveParaToEeprom(PARA_DriverFlag,u16tmp);
			}			
		}
		if((FunctionDisable != gUserInfo.u8WeighFunc)
			||(FunctionDisable != gUserInfo.u8FourPointWeightFunc))	//开启，检测使能	
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
}
static void vGPSInit(void)  //防止上次传输掉线卡在某个状态
{
	tCanFrame GPSFrame;
	memset(&GPSFrame,0,sizeof(GPSFrame));
	GPSFrame.u32ID =0x655;
	GPSFrame.u16DataLength = 8;
	GPSFrame.u8Data[0] = 0x80;
	i32CanWrite(Can0,&GPSFrame);
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
	
	/*闪光灯周期调节*/
	vAlarmLampSetPeriod(60);
	vAlarmLampSetOpenePeriod(120);//设置为上边周期的倍数
	
	vErrCodeInit(sgErrCodeInfo,(sizeof(sgErrCodeInfo))/(sizeof(sgErrCodeInfo[0])));
	
	memset(&sgActLogic, 0, sizeof(sgActLogic));
	vBatterySocInit();
	
	vUserParaInit();
	vHourCountInit();
	vGPSInit();
	vCanIdLostReg(0x28A,1000,vCanLostProc);
	vCanIdLostReg(0x726,200,vCanLostProc);
	vCanIdLostReg(0x3D5,5000,vCanLostProc);// 5s中收不到3D5报文则判断为GPS掉线
//	vClearBootFlag();
#ifdef SECOND_BOOT_EN
	xRevCallBackProc CanIdCD4FDFD = {.u32CanId = 0XCD4FDFD, .u32Data = 0, .CallBack = CanIdCD4FDFDRev};
	vCanRevMsgRegister(&CanIdCD4FDFD);
#endif	
	xRevCallBackProc CanId602 = {.u32CanId = 0x602, .u32Data = 0, .CallBack = CanId602Rev};
	vCanRevMsgRegister(&CanId602);
	xRevCallBackProc CanId3D5 =	{.u32CanId = 0x3D5, .u32Data = 0, .CallBack = CanId3D5Rev};
	vCanRevMsgRegister(&CanId3D5);
	
	vRemoteUnlockKey();
	BlockTransInit();
	vUserSetsInit();
	
	sgLockState.u8Data = i32GetPara(PARA_RemotePara);
	
			
	sgPcuHandle.i8PositiveValue = i32GetPara(PARA_HANDLEMAX);
		
	sgPcuHandle.i8MiddleValue = i32GetPara(PARA_HANDLEMID);
		
	sgPcuHandle.i8NegativeValue = i32GetPara(PARA_HANDLEMIN);
			
	__disable_irq();
//	gCanSendPdoInfo.CanSend260Info.u8HourCountL = u32HourCount & 0xFF;
//	gCanSendPdoInfo.CanSend260Info.u8HourCountH = (u32HourCount >> 8) & 0xFF;

	__enable_irq();		

	vSetNetTimer(TIMER_EcuPowerOn, ECU_POWERON_DELAY_TIME);	/**/
	
	
}
/*******************************************************************************
* Name: void vUserEcuProc(void)
* Descriptio: UserEcu锟矫伙拷锟斤拷锟斤拷锟斤拷锟斤拷
* Input: NULL
* Output: NULL  
*******************************************************************************/
void vUserEcuProc(void)
{
	static uint8_t u8EcuProcFlag = 0;
	uint8_t u8ErrCode = 0;
	static uint16_t u16SecCnt = 0;

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
		
		vTBoxProc();
		
		StartCommandSend();
		
		HMISend();
		
		vLockProc();//锁车解锁功能

		u32HourCnt = u32HourCountProc(sgActLogic.u8Data);//置入需要判断的计时状态，或者1
		
//		i32SetPara(PARA_TurnLeftValveCurrent,u32HourCnt);
		__disable_irq();
		__enable_irq();
		
	}
	vWdgSetFun(WDG_USER_BIT);
}
#endif