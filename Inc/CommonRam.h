/*******************************************************************************
* Filename: CommonRam.h 	                                    	     		   *
*                                                                              *
* Description: The header file of CommonRam.c.							           *
* Author:                                                                      *
* Date: 															           *
* Revision:															           *
*																	           *
*******************************************************************************/
#ifndef _COMMONRAM_H_
#define _COMMONRAM_H_

#include	"KSDsys.h"
#include "ServoPara.h"
#include "PARA.h"
/******************************************************************************
*数据类型定义
******************************************************************************/

/* 参数修改控制结构 */
typedef struct PARA_MODIFY_CTL
{
	INT16S			OldValue;			/* 参数改变前的值 */
	INT16S			NewValue;			/* 参数改变后的值 */
	INT16U			ParaNo;				/* 正修改的参数号 */
	INT16U			bVolatile;			/* 是否保存参数 */
} PARA_MODIFY_CTL;



#define NORMAL_MODE		0
#define SLOW_MODE   	1
#define FAST_MODE			2
#define COAST_MODE			3

/* Common ram */
typedef struct 
{
	/* kernel input data */
	INT32U	PosCmdSrc;				/*pos command source 0-from plc 1-from fxp*/
	INT32S	PosCmd;						/* 位置指令: 指令绝对位置(指令脉冲) */
	INT32S	PosFdb;						/* 位置反馈: 编码器绝对位置(编码器脉冲) */
	INT32S	SpeedCmd;					/* 速度指令: 用于速度控制模式(1 / 65536 HZ) */
	INT32U	SpeedMode;				/* 0-Normal; 1-Slow; 2-Fast; Else- Normal*/
	INT32S	BrakeCmd;					/* 刹车指令: Percent 32768 = 100% */
	INT32S	SpdCmdSum;					/* 速度指令: 用于速度控制模式(1 / 65536 HZ) */
	INT32S	SpeedFdb;					/* 速度反馈: 速度环输入值(Q24标幺值 rpm) */
	INT32S	TorqueCmd;				/* 转矩指令: 用于转矩控制模式(1 / 65536 Nm) */
	INT32S	RotorElecAngle;		/* 转子电角度: Q24 (2*PI rad) */
	INT32S  CurrentSampleVBus;/*直流母线采样 Q12标幺值 A*/
	INT32S  TempratureSample;	/*温度采样 单位 0.1C*/
	INT32S  PropCurrentCmd;   /* Current command for prop driver (1 / 65536 A) */
	INT32S  PropCurrentCmdSum;   /* Current command for prop driver (1 / 65536 A) */
	INT32S  PumpSpeedCmd;			/* Pump cmd speed */
	/* kernel output data */
	INT32S	PosRef;						/* 位置参考: 位置环输入值(编码器脉冲) */
	INT32S	PosErr;						/* 位置误差: 编码器脉冲 */
	INT32S  SpeedRef;					/* 速度参考: 速度环输入值(Q24标幺值 rpm) */
	INT32S	SpeedFdbDisp;			/* 速度反馈显示值:  Q24标幺值 rpm (8ms LPF滤波) */	
	INT32S	CurrentFdbIq;			/* 电流反馈Iq值: Q24标幺值 A */
	
	/* 故障报警代码 */
	INT16U	ErrCode;
	INT16U	AlmCode;


	INT8U	AutoCentreDirectionLock;	//Lock the last return direction
	INT8U	AutoCentreDirection;	//0:停止自动回中；1：自动回中左转；2：自动回中右转
	INT32S	AutoCentreCmd;		/* 回中位置指令: 指令绝对位置(指令脉冲) */
	INT32S	PosMiddleCmd;		//中位
	INT32S	PosMiddleFdb;		//中位
	INT32S	AutoCentreCmdSum;
	INT8U	AutoCentrePhase;	//自动回中阶段
	/* semaphore */
	INT16U	SigLamp[SIGNAL_LAMP_NUM];
	
	/* monitor state*/
	INT16S MotorTmp;				//电机当前温度
	INT16S ControllerTmp;		//控制器当前温度
	INT16U VBusVoltage;			//当前母线电压
	INT16U KsiVBusVoltage;	//KSI电压
	INT16S MoveSpeed;				//行驶速度
	INT16S SteerAngle;			//转向角度
	
	INT16U TestVoltage;
	/* monitor signal*/
	INT16U ThrottleCmdSpd;	//踏板指令（%）
	INT16U ThrottleWipVoltage;	//踏板滑动端电压值（v）
	INT16U BrakePedalCmdSpd;	//brake 踏板指令（%）
	INT16U BrakePedalWipVoltage;	//brake 踏板滑动端电压值（v）
	INT16U AnologOut;				//模拟量输出（v）
	INT16U AnalogIn1;				//模拟量1输入（v）
	INT16U AnalogIn2;				//模拟量2输入（v）
	INT16U AnalogIn3;				//模拟量3输入（v）
	INT16U AnalogIn4;				//模拟量4输入（v）
	INT16U Switch;					//开关输入
	INT16U Driver;					//驱动输出
	INT16U KernelState;
	
	/* Debug */
	INT16U	SbdcVoltageCmd;
	INT16U	SbdcRatioRate2;	
} KSD_COMMON_RAM;

/* variables for common ram */
//extern		KSD_COMMON_RAM	gCRam;

/******************************************************************************
*函数定义
******************************************************************************/
extern void 	InitCommonRam(void);

#endif //_COMMONRAM_H_
