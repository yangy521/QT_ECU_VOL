/*******************************************************************************
* Filename: iTimer.h	                                             	 		   *
* Description:											   			 		   *
* Author:                                                           		   *
* Date:     														 		   *
* Revision:															 		   *
*******************************************************************************/

#include		"KSDsys.h"

#define TIMER_NUM			48
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

//定义网络定时器结构体
typedef struct _tNetTimer
{
	unsigned long ulTimerCount;		//	时间计时器（单位ms）
	unsigned long ulDeadline;			//	超时时限
	tBoolean  bIsStart;						//	是否启动
	tBoolean  bIsOvertime;				//	是否超时
}tNetTimer;

//变量声明
extern tNetTimer netTimer[TIMER_NUM];		//

extern void netTimerInit(void);//网络定时器初始化
extern void netTimerUpdate(void);//网络事件定时器计时
extern void SetNetTimer(unsigned char ucTimerID,unsigned long ulDeadline);//启动网络定时器
extern void ResetNetTimer(unsigned char ucTimerID);//复位网络定时器
extern void KillNetTimer(unsigned char ucTimerID); //删除网络定时器
