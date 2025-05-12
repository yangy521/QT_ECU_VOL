/*******************************************************************************
* Filename: PLCHardware.h	                                             	 		   *
* Description:											   			 		   *
* Author:                                                           		   *
* Date:     														 		   *
* Revision:															 		   *
*******************************************************************************/
#ifndef _PLCHARDWARE_H_
#define _PLCHARDWARE_H_

#include "KSDsys.h"
#include "iCANPlc.h"

//****************** Error & Alarm No define ******************/
extern void PLCErrorClr(INT16U ErrorNo);

#define ERNO_POWER_MAX_TMP          18
//#define ERNO_          19
#define ERNO_MOTOR_MAX_TMP          19

#define ALNO_POWER_CUT_TMP          32
#define ALNO_POWER_MIN_TMP          33
#define ALNO_MOTOR_CUT_TMP          34


//*********** Time delay constant
#define PLC_PERIOD  5  //5ms
#define FULL_VOLTAGE_ACT_TIME (400/PLC_PERIOD)   //400MS
#define DO_LOST_CHK_DELAY_TIME    1000						//1000ms
/*******************************************************************************
* 1. DI
*******************************************************************************/
//#if (CTLBOARD_TYPE ==_1226)
#define		SWI1_R					0
#define		SWI2_R					1
#define		SWI3_R					2
#define		SWI4_R					3
#define		SWI5_R					4
#define		SWI6_R					5
#define		SWI7_R					6
#define		SWI8_R					7
#define		SWI9_R					8
#define		SWI10_R					9
#define		SWI11_R					10
#define		SWI12_R					11
#define		DRIVER1_R				10
#define		DRIVER2_R			  11
#define		DRIVER3_R				12
#define		DRIVER4_R			  13
#define		DRIVER5_R				14
#define		DO1_SHUT				15
#define		DO2_SHUT			  16
#define		DO3_SHUT				17
#define		DO4_SHUT			  18
#define		DO5_SHUT				19
#define		LOCK_IN					SWI10_R
#define		SWI13_R					20
#define		SWI14_R					21
//#endif //#if (CTLBOARD_TYPE ==_1226)

/*******************************************************************************
* 2. DO
*******************************************************************************/

//#if (CTLBOARD_TYPE ==_1226)
#define		DRIVER1						0
#define		RELAY						DRIVER1
#define		DRIVER2					1
#define		DRIVER3					2
#define		DRIVER4					3
#define		LED_Y						4
#define		LED_R						5
#define		LOCK_OUT				6
#define		DRIVER_EN				7
#define		LED_OUT					8
#define		CHARGE					9
#define		BRAKE					10
//#endif //#if (CTLBOARD_TYPE ==_1226)
/*******************************************************************************
* 3. AI
*******************************************************************************/

//#if (CTLBOARD_TYPE ==_1226)
	#define			AD_KsiVBus					0
	#define			AD_VBus							1
	#define			AD_V5out						2
	#define			AD_PowTmpHigh				3
	#define			AD_MotorTmp 				4
	#define			AD_ThrotPotHigh			5
	#define			AD_ThrotPotWip			6
	#define			AD_SpdPotHigh				7
	#define			AD_SpdPotWip				8
	#define			AD_PotLow						9
	#define			AD_RelayR						10
	#define			AD_V12out						11
//#endif  //#if (CTLBOARD_TYPE ==_1226)
/******************************************************************************
*数据类型定义
******************************************************************************/
/* terminal control */
typedef struct
{
	INT16U	InAOld;
	INT16U	InATime;
	INT16U	InBOld;
	INT16U	InBTime;

	INT16U	RdyDelayTime;
	INT16U	PwmChargeTime;
} TERMINAL_CTL;

#define DI_FILTER_CONSTANT	4//8		//5*8=40ms

typedef struct DI_FILTER{
	INT8U ucIn[32];	   		//DI 32路
	INT8U ucInNew[32];	 	//DI 32路
	INT8U ucInOld[32];	 	//DI 32路
	INT8U ucTimer[32];	 	//DI 32路
}DI_FILTER;

#define LED_BLINK_PERIOD 		500
#define VOL_BAK_100					516    //51.6v,100% Battery

/* logicRemote bit define */
#define LOCK_logicRemote             				(1 << 0)
#define LOWSPD1_logicRemote									(1 << 1)
#define LOWSPD2_logicRemote									(1 << 2)
#define NOLIFT_logicRemote									(1 << 3)

//#define BATTERY_PROTECT_RATIO	20
//#define DIFF_BETWEEN_LIBAT_PBBAT	5

#define BRAKE_EBRAKE_DELAY  100
#define TIME_LIMIT_STOP  100
#define ZEROTHROTTLE_EBRAKE_DELAY 100


//*** Bit define for SwiLogic

#define STEERFAULT_LOCK_LOGIC			(1 << 0)
#define SAFE_LOCK_LOGIC						(1 << 1)
#define GUISU_LOGIC								(1 << 2)
#define FORWARD_LOGIC       			(1 << 3)
#define REVERSE_LOGIC       			(1 << 4)
#define EMREVERSE_LOGIC     			(1 << 5)
#define LIFT_LOGIC          			(1 << 6)
#define DOWN_LOGIC          			(1 << 7)
#define HULAN1_LOGIC   						(1 << 8)
#define HULAN2_LOGIC   						(1 << 9)
#define TABAN_LOGIC   						(1 << 10)
#define GAODUXIANSU1_LOGIC   			(1 << 11)
#define GAODUXIANSU2_LOGIC   			(1 << 12)
#define LIFT_LIMIT_LOGIC   				(1 << 13)
#define ZHILI_LOGIC   						(1 << 14)
#define STEERANGLE_LOGIC   				(1 << 15)
#define SHUJU_LOGIC   						(1 << 16)
#define THROTTLE_LOGIC   					(1 << 17)
#define HANDBRAKE_LOGIC   				(1 << 18)
#define LIFT_THRO_LOGIC          	(1 << 19)
#define DOWN_THRO_LOGIC          	(1 << 20)
#define ZUOYI_LOGIC								(1 << 21)
#define TABAN2_LOGIC   						(1 << 22)
#define DOWN2_LOGIC          			(1 << 23)
#define FOOTBRAKE_LOGIC          	(1 << 24)
#define QIANHOU_LOGIC          		(1 << 25)
#define QINGXIE_LOGIC          		(1 << 26)
#define CEYI_LOGIC          			(1 << 27)
#define CHARGE_LOGIC          		(1 << 28)
#define MOVEDIR_LOGIC          		(1 << 29)

//#define RSV_LOGIC   						bit28~31
#define STEP_FORWAR_LOGIC    			HULAN1_LOGIC //牵引车点动，借用护栏
#define STEP_REVERSE_LOGIC    		HULAN2_LOGIC //牵引车点动，借用护栏

//*** Bit define for gPLCCtl.logicLock
#define THROTTLE_LOCK					(0x0001 << 0)
#define BRAKECMD_ACT					(0x0001 << 1)
#define LIFT_LOCK							(0x0001 << 2)
#define SA_LOCK							  (0x0001 << 3)
#define RVS_ACT_STOP				  (0x0001 << 4)
#define RVS_ACT_AWAY				  (0x0001 << 5)
#define RVS_LOCK		 				  (0x0001 << 6)
#define DIR_RVS_BRAKE		 			(0x0001 << 7)
#define DIR_RVS_ACT		 				(0x0001 << 8)
#define FWD_RVS_ACT		 				(0x0001 << 9)
#define REMOTE_WHEEL_LOCK		 	(0x0001 << 10)

#define ZHILI_MOVE						(0x0001 << 10)
#define SLOW_MODE_FLAG			  (0x0001 << 11)

#define HULAN_TABAN_PROTECT	  (0x0001 << 12)
#define ZHILI_MOVE_ACT				(0x0001 << 13)

#define HULAN_LOCK				    (0x0001 << 14)
#define STEP_MODE							(0x0001 << 15)

#define THROTTLE_AD_LOST					(0x0001 << 16)
#define THROTTLE_DIR_LOCK					(0x0001 << 17)
#define THROTTLE_ENSWI_LOCK					(0x0001 << 18)
#define THROTTLE_DIR_ERROR					(0x0001 << 19)

#define LIFT_SWI_LOCK							(0x0001 << 20)
#define QINGXIE_SWI_LOCK					(0x0001 << 21)
#define CEYI_SWI_LOCK							(0x0001 << 22)
#define QIANYI_SWI_LOCK						(0x0001 << 23)
#define SHUJU_SWI_LOCK						(0x0001 << 24)

#define N_MAINDRIVER_LOCK					(0x0001 << 25)

//*** Bit define for gPLCCtl.LogicState
#define GAODUXIANSU_LOGICSTATE							(0x01 << 0)		//bit0- 
#define GAODUXIANSU1_LOGICSTATE							GAODUXIANSU_LOGICSTATE		//bit0- 				
#define GAODUXIANSU2_LOGICSTATE							(0x01 << 1)		//bit1- 				
#define MOTOR_OVTAL_LOGICSTATE							(0x01 << 2)		//bit2- 
#define MOTOR_OVTER_LOGICSTATE							(0x01 << 3)		//bit3- 
#define POWER_OVTAL_LOGICSTATE							(0x01 << 4)		//bit4- 
#define POWER_OVTER_LOGICSTATE							(0x01 << 5)		//bit5- 

#define CURRENTRATIO_STOP_LOGICSTATE				(0xFF << 8)		//bit15-8  -127 ~ 127 				

//*** Bit define for gPLCCtl.RemoteParaRdWrFlag
#define RDREQ_RemoteParaRdWrFlag				    (0x0001 << 0)
#define RDRDY_RemoteParaRdWrFlag				    (0x0001 << 1)
#define WRREQ_RemoteParaRdWrFlag				    (0x0001 << 2)
#define WRRDY_RemoteParaRdWrFlag				    (0x0001 << 3)
/* PLC control */
typedef struct
{
	INT32U PlcCount;
	/* DI、DO、AI AO*/
	DI_FILTER		diDataIn;
	INT16U			doDataOut[10];
#if ((CTLBOARD_TYPE ==_1236) || (CTLBOARD_TYPE ==_1246))
	INT16U aiDataIn[16];
	INT16U			aoDataOut;
#endif //#if (CTLBOARD_TYPE ==_1236)
	
#if (CTLBOARD_TYPE ==_1232)
	INT16U aiDataIn[14];
#endif  //#if (CTLBOARD_TYPE ==_1232)

#if ((CTLBOARD_TYPE ==_1222) || (CTLBOARD_TYPE ==_1242))
	INT16U aiDataIn[12];
#endif  //#if (CTLBOARD_TYPE ==_1222)

#if ((CTLBOARD_TYPE ==_1226)||(CTLBOARD_TYPE ==_1226_GD32))
	INT16U aiDataIn[12];
#endif  //#if (CTLBOARD_TYPE ==_1226)

	INT16U			doConnect[4];
	INT16U			PulseWidth[4];
	INT16S			PulseWidthDelay[4];
	INT16U			PulsePeroid[4];
	
	INT8U			  SigLampSvOff;
	INT8U			  SigLampSvOffOld;
	INT8U       diLockInAct;

	unsigned char  ucBatteryAct;			  //	电池电量 
	unsigned char  ucBatteryBms;			  //	Bms电池电量
	INT16S			ThrottleWipVoltageBus;  //总线发来的油门电压 -4096 ~ 4096
	
	INT8U				CmdDirection;	//指令方向：0:中位（停止）；1：反向2：正向
	INT16U			CurSpeedRate;
	INT16S      rateLow,rateHigh; //百分比/0.1V, _IQ8
	INT16U			deadBandMin,deadBandMax,throttleMid;
	INT16S      rateLowRvs,rateHighRvs; //百分比/0.1V, _IQ8
	INT16U			deadBandMinRvs,deadBandMaxRvs,throttleMidRvs;
	INT16U			throttleOutput;	//单位百分比
	INT16U			speedRate;			//速度档位（百分比）
	INT16U			throttleWip;		//踏板滑动端电压值（0.1V）
	INT16U			AcMotorMaxSpdFAct; //当前使用的最大速度
	INT32U			AcMotorMaxTorqFAct; //当前使用的最大转矩
	INT32S			TorqueCmdRaw;		//
	INT32S			TorqueCmdSum;		//
	INT32S			TorqueCmdAvg;		//
	INT32S			TorqueCmdForward;		//
	INT8U				CutDowmFlag;	//
	
	INT8U				BrakePedalCmdDirection;	//指令方向：0:中位（停止）；1：反向2：正向
	INT16U			BrakePedalCurSpeedRate;
	INT16S      BrakePedalrateLow,BrakePedalrateHigh; //百分比/0.1V, _IQ8
	INT16U			BrakePedaldeadBandMin,BrakePedaldeadBandMax,BrakePedalMid;
	INT16S      BrakePedalrateLowRvs,BrakePedalrateHighRvs; //百分比/0.1V, _IQ8
	INT16U			BrakePedaldeadBandMinRvs,BrakePedaldeadBandMaxRvs,BrakePedalMidRvs,BrakePedalMapRvs;
	INT16U			BrakePedalOutput;	//单位百分比
	INT16U			BrakePedalspeedRate;			//速度档位（百分比）
	INT16U			BrakePedalWip;		//踏板滑动端电压值（0.1V）
	INT16U			BrakePedalAcMotorMaxSpdFAct; //当前使用的最大速度

	INT16U			LiftPedalOutput;	//单位百分比
	INT16U			DownPedalOutput;	//单位百分比

	INT8U				SidleCmdDirection;	//指令方向：0:中位（停止）；1：反向2：正向
	INT16U			SidleWip;		//踏板滑动端电压值（0.1V）
	INT16U			SidleOutput;	//单位百分比
	INT8U				PitchCmdDirection;	//指令方向：0:中位（停止）；1：反向2：正向
	INT16U			PitchWip;		//踏板滑动端电压值（0.1V）
	INT16U			PitchOutput;	//单位百分比

	INT16U			ActOutput;	//单位百分比

	INT8U			  LocalEnable;	//本地使能，对应于PIN9
	INT8U				SignalHome;		//中位信号，对应于PIN10
	tBoolean		bAutoCentreStart;	//是否启动自动回中 
	tBoolean		bIsAutoCentreOn;	//是否正在进行自动回中
	
	INT16S			TmpMotor;	//电机温度
	INT16S			TmpPower;	//功率板温度
	INT16S			TmpDrive;	//驱动板温度
	INT16U			TmpFlag;  //loop ctl flag
	INT16S			TmpCount; //loop ctl flag
	INT32S 			TmpPowerLSum;
	INT32S 			TmpPowerHSum;
	INT32S 			TmpPowerL;
	INT32S 			TmpPowerH;
	INT32S 			TmpMotorAdSum;
	INT32S 			TmpMotorAd;
	INT32S 			TmpDriveAdSum;
	INT32S 			TmpDriveAd;
	_iq					OutMaxRatioOvTmp;
	INT16U			KsiVBus;
	INT16U			VBus;
	
	INT16U   RemoteParaRdWrFlag;
	INT16U   RemoteParaRdWrIndex;
	INT16U   RemoteParaRdWrData;
	INT16U			LogicState;
	INT32U  HourCnt;  		//0.1hour
	INT32U  HourCntNew;  		//0.1hour
	INT32U  HourCntPowerOn;  		//0.1hour
	INT32U  HourCntPowerOnNew;  		//0.1hour
	INT32U  HourCount6min;
	_iq	iqKsiVBus;
	_iq iqVBus;
	_iq iqDcRateVoltage;		  //电瓶电压等级
	_iq iqDcMaxVoltage;				/* 最高电压，超过该电压系统不启动 */
	_iq iqDcCutVoltage;				/* 削减电压，低于该电压系统降耗运行，削减输出扭矩 */
	_iq iqDcMinVoltage;				/* 最低电压，低于该电压系统不启动，停止运行 */
	_iq iqDcBroVoltage;				/* 启动电压，低于该电压，系统复位，关闭一切输出 */
	_iq iqDcEspLowVoltage;
	_iq iqCtlRate1HCurrent;		//控制器1小时电流等级
	_iq iqCtlRate2MCurrent;		//控制器2分钟电流等级
	_iq iqOut5V;
	_iq iqOut12V;
	_iq iqOut5VMaxVoltage;
	_iq iqOut5VMinVoltage;
	_iq iqOut12VMaxVoltage;
	_iq iqOut12VMinVoltage;
	
	/* led */
	INT16S		LedCountHead;
	INT16S		LedCountRep;
	INT16S		LedCount;
	INT16S    LedBlinkPeriod;      //Led blink period ms

	INT16S ucBatteryDelay;
	
	INT16S delaySvoff;
	INT16S delaySvoffMove;
	_iq delaySvoffSpd;
	
	INT32U logicLock;
	INT16S logicLockDelay;
	INT16U logicRemote;
	INT32U  SwiLogic;


	INT16S sysEnableDelay;
	INT16U CanIdPowerOnDelay;
	INT16U CanIdLostDelay;
	/* ICAN资源节点 */
	tICAN_LIFT icanLift;
	tICAN_STEER icanSteer;
	tICAN_SMOVE icanSmove;
	tICAN_LOGIC icanLogic;
	tICAN_HMI icanHMI;
	tICAN_PC icanPc;
	
#if (USER_TYPE == USER_NUOLI_DDTBC_MOVE)
	_tCANOPEN_REMAHANDLE canopenRemaHandle;	
#endif
} PLC_CTL;

/* variables for PLC control */
extern		PLC_CTL			gPLCCtl;

/******************************************************************************
*函数定义
******************************************************************************/
/* Functions in PLC.c */
extern void	InitPLCLogic(void);
extern void PLCStateMonitor(void);
extern void PLCHardwareCheck(void);
extern void LocalDi(void);
extern void LocalDo(void);
extern void DoPWM(void);
extern void LEDInit(void);
extern tBoolean UserPoweronIoCheck(void);
#endif //_PLCHARDWARE_H_
