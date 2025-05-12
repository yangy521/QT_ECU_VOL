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
#define	TIMER_Drive1Check		22	/*Drive1定时器*/
#define	TIMER_Drive2Check		23	/*Drive2定时器*/
#define	TIMER_Drive3Check		24	/*Drive3定时器*/
#define	TIMER_Drive4Check		25	/*Drive4定时器*/
#define	TIMER_Drive5Check		26	/*Drive5定时器*/
#define	TIMER_Drive6Check		27	/*Drive6定时器*/
#define	TIMER_Drive7Check		28	/*Drive7定时器*/
#define	TIMER_Drive8Check		29	/*Drive8定时器*/
#define	TIMER_Drive9Check		30	/*Drive9定时器*/
#define	TIMER_Drive10Check		31	/*Drive10定时器*/
#define	TIMER_PorpDriver0Check	32	/*Drive11定时器*/
#define	TIMER_PorpDriver1Check	33	/*Drive12定时器*/

#define	TIMER_AI1Check			35	/*AI1定时器*/
#define	TIMER_AI2Check			36	/*AI2定时器*/
#define	TIMER_AI3Check			37	/*AI3定时器*/
#define	TIMER_Encoder1Check		38	/*AI4定时器*/
#define	TIMER_Encoder2Check		39	/*AI5定时器*/


#define	TIMER_EEPROM			40	/*EEPROM定时器*/

#define Timer_SpeedHold		41

//23.11.18 sj用户层使用的定时器
//#define	TIMER_SwitchCheck	41	/*开关检测延时*/
//#define TIMER_Calibration	42	/*压力标定计时*/
//#define	TIMER_AlarmDelay	43	/*报警延时*/

#define TIMER_USER_1			41	/*用户定时器1*/
#define TIMER_USER_2			42	/*用户定时器2*/
#define TIMER_USER_3			43	/*用户定时器3*/
#define TIMER_USER_4			44	/*用户定时器4*/

#define	TIMER_Test				45


#define	TIMER_PLC_PERIOD		5
#if 0
/* 定义时间计时器 */
#define Timer_Connect							0	//ICOM通信定时器
#define Timer_ICANSlave1					1	//ICAN从站1发送定时器（Timer_Post)
#define Timer_ICANSlave2					2	//ICAN从站2发送定时器（Timer_Post)
#define Timer_ICANSlave3					3	//ICAN从站3发送定时器（Timer_Post)
#define Timer_ICANSlave4					4	//ICAN从站4发送定时器（Timer_Post)
#define Timer_ICANSlave5					5	//ICAN从站5发送定时器（Timer_Post)
#define Timer_ICANSlave6					6	//ICAN从站6发送定时器（Timer_Post)
#define Timer_Logic								7	//逻辑程序处理定时器
#define	Timer_SelfCheck						8	//控制器开机自检
#define	Timer_Drive1Check					9	//Drive1定时器
#define	Timer_Drive2Check					10	//Drive2定时器
#define	Timer_Drive3Check					11//Drive1定时器
#define	Timer_Drive4Check					12	//Drive2定时器
#define Timer_MacIDCheck        	13  //MAC检测定时器
#define Timer_KSICheck            14  //KSI检测定时器
#define Timer_VBusCheck						15	//母线电压检测定时器
#define Timer_Charge							16	//充电定时器
#define Timer_SvOff								17	//运行停止定时器
//#define Timer_Led	          	    18	//指示灯灯定时器
#define Timer_SpeedHold  	   			19	//转向助力延时定时器
#define Timer_VoltageEspCheck			20  //
#define Timer_PowerTmpMaxCheck1		21	//功率板高温阈值1检测定时器
#define Timer_PowerTmpMaxCheck2		22	//功率板高温阈值2检测定时器
#define Timer_PowerTmpMinCheck		23	//功率板低温阈值检测定时器
#define Timer_VoltageMinCheck 		24	//电池低压阈值2检测定时器
#define Timer_VoltageCutCheck		  25	//电池低压削减检测定时器
#define Timer_VoltageMaxCheck			26	//电池高压阈值检测定时器
#define Timer_MotorTmpMaxCheck1		27	//电机高温阈值1检测定时器
#define Timer_MotorTmpMaxCheck2		28	//电机高温阈值2检测定时器
#define	Timer_OUT5V_Check					29	//
#define	Timer_OUT12V_Check				30	//
#define Timer_KSIAbnormalCheck    31  //KSI电压异常检测定时器
#define Timer_VBusAbnormalCheck		32	//母线电压异常检测定时器
#define Timer_TERSet						33	// 
#define	Timer_WeldedCheck					34	//主接触器触点熔接检测，母线电容电压未释放
#define	Timer_SvOffDelay					35	//
#define Timer_TERClr			36	//
#define Timer_ErrorDelay			37	//
//#define 			38	//
//#define 			39	//
#define Timer_ICAN_Delay					40	//ICAN网络滞后系统30ms启动
#endif
//定义网络定时器结构体
typedef struct _tNetTimer
{
	uint64_t  u64TimerCount;		//	时间计时器（单位ms）
	uint64_t  u64Deadline;			//	超时时限
	tBoolean  bIsStart;						//	是否启动
	tBoolean  bIsOvertime;				//	是否超时
	
}tNetTimer;

//变量声明
extern tNetTimer netTimer[TIMER_NUM];		//

extern void vNetTimerInit(void);//网络定时器初始化
extern void vNetTimerUpdate(void);//网络事件定时器计时
extern void vSetNetTimer(uint8_t ucTimerID,uint64_t u64Deadline);//启动网络定时器
extern void vResetNetTimer(uint8_t u8TimerID);//复位网络定时器
extern void vKillNetTimer(uint8_t u8TimerID); //删除网络定时器
extern uint8_t u8GetNetTimerOverFlag(uint8_t u8TimerID);
extern uint8_t u8GetNetTimerStartFlag(uint8_t u8TimerID);

#endif