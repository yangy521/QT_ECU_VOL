#define PLC_PERIOD					5
#define LED_BLINK_PERIOD 		2000
#include "KSDsys.h"
#include "CommonRam.h"
#include "Device.h"

/* PLC control */
// typedef struct
// {
// 	/* DI、DO、AI AO*/
// 	DI_FILTER		diDataIn;
// 	INT16U			doDataOut[4];
// #if (CTLBOARD_TYPE ==_1236)
// 	INT16U aiDataIn[16];
// 	INT16U			aoDataOut;
// #endif //#if (CTLBOARD_TYPE ==_1236)
// 	
// #if (CTLBOARD_TYPE ==_1232)
// 	INT16U aiDataIn[14];
// #endif  //#if (CTLBOARD_TYPE ==_1232)

// 	#if (CTLBOARD_TYPE ==_1222)
// 	INT16U aiDataIn[12];
// #endif  //#if (CTLBOARD_TYPE ==_1222)

// 	INT16U			doConnect[4];
// 	INT16U			PulseWidth[4];
// 	INT16S			PulseWidthDelay[4];
// 	
// 	INT8U			  SigLampSvOff;
// 	INT8U			  SigLampSvOffOld;
// 	
// 	INT8U				CmdDirection;	//指令方向：0:中位（停止）；1：反向2：正向
// 	INT16U			CurSpeedRate;
// 	INT16S      rateLow,rateHigh; //百分比/0.1V, _IQ8
// 	INT16U			deadBandMin,deadBandMax,throttleMid;
// 	INT16U			throttleOutput;	//单位百分比
// 	INT16U			speedRate;			//速度档位（百分比）
// 	INT16U			throttleWip;		//踏板滑动端电压值（0.1V）
// 	INT16U			AcMotorMaxSpdFAct; //当前使用的最大速度

// 	INT8U				BrakePedalCmdDirection;	//指令方向：0:中位（停止）；1：反向2：正向
// 	INT16U			BrakePedalCurSpeedRate;
// 	INT16S      BrakePedalrateLow,BrakePedalrateHigh; //百分比/0.1V, _IQ8
// 	INT16U			BrakePedaldeadBandMin,BrakePedaldeadBandMax,BrakePedalMid;
// 	INT16U			BrakePedalOutput;	//单位百分比
// 	INT16U			BrakePedalspeedRate;			//速度档位（百分比）
// 	INT16U			BrakePedalWip;		//踏板滑动端电压值（0.1V）
// 	INT16U			BrakePedalAcMotorMaxSpdFAct; //当前使用的最大速度

// 	INT8U				SidleCmdDirection;	//指令方向：0:中位（停止）；1：反向2：正向
// 	INT16U			SidleWip;		//踏板滑动端电压值（0.1V）
// 	INT16U			SidleOutput;	//单位百分比
// 	INT8U				PitchCmdDirection;	//指令方向：0:中位（停止）；1：反向2：正向
// 	INT16U			PitchWip;		//踏板滑动端电压值（0.1V）
// 	INT16U			PitchOutput;	//单位百分比

// 	INT16U			ActOutput;	//单位百分比

// 	INT16S			TmpMotor;	//电机温度
// 	INT16S			TmpPower;	//功率板温度
// 	INT16S			TmpDrive;	//驱动板温度
// 	INT16U			TmpFlag;  //loop ctl flag
// 	INT16S			TmpCount; //loop ctl flag
// 	INT32S 			TmpPowerLSum;
// 	INT32S 			TmpPowerHSum;
// 	INT32S 			TmpPowerL;
// 	INT32S 			TmpPowerH;
// 	INT32S 			TmpMotorAdSum;
// 	INT32S 			TmpMotorAd;
// 	INT32S 			TmpDriveAdSum;
// 	INT32S 			TmpDriveAd;

// 	INT16U			KsiVBus;
// 	INT16U			VBus;
// 	
// 	_iq	iqKsiVBus;
// 	_iq iqVBus;
// 	_iq iqDiffBus;						//KsiBus与VBus电压差
// 	_iq iqMaxDiffBus;					
// 	_iq iqDcRateVoltage;		  //电瓶电压等级
// 	_iq iqDcMaxVoltage;				/* 最高电压，超过该电压系统不启动 */
// 	_iq iqDcCutVoltage;				/* 削减电压，低于该电压系统降耗运行，削减输出扭矩 */
// 	_iq iqDcMinVoltage;				/* 最低电压，低于该电压系统不启动，停止运行 */
// 	_iq iqDcBroVoltage;				/* 启动电压，低于该电压，系统复位，关闭一切输出 */
// 	_iq iqDcEspLowVoltage;
// 	_iq iqCtlRate1HCurrent;		//控制器1小时电流等级
// 	_iq iqCtlRate2MCurrent;		//控制器2分钟电流等级
// 	_iq iqOut5V;
// 	_iq iqOut12V;
// 	_iq iqOut5VMaxVoltage;
// 	_iq iqOut5VMinVoltage;
// 	_iq iqOut12VMaxVoltage;
// 	_iq iqOut12VMinVoltage;
// 	
// 	/* led */
// 	INT16S		LedCountHead;
// 	INT16S		LedCountRep;
// 	INT16S		LedCount;
// 	INT16S    LedBlinkPeriod;      //Led blink period ms
// 	
// 	/* ICAN资源节点 */
// 	tICAN_Lift icanLift;
// 	tICAN_Steer icanSteer;
// 	tICAN_HMI icanHMI;
// 	
// } PLC_CTL;

/*******************************************************************************
* 1. 指示灯
*******************************************************************************/
//状态指示灯
#define 	LED_YELLOW_SET						(1 << 0)
#define 	LED_YELLOW_RESET					(1 << 1)
#define 	LED_YELLOW_BLINK					(1 << 2)
#define		LED_RED_SET								(1 << 8)
#define		LED_RED_RESET							(1 << 9)
#define		LED_RED_BLINK							(1 << 10)

void LedStateShow(INT16U ucState)
{
	//yellow
	if (ucState & LED_YELLOW_SET)
	{
		gPLCCtl.doDataOut[LED_Y] = 1;
	}
	else if (ucState & LED_YELLOW_RESET)
	{
		gPLCCtl.doDataOut[LED_Y] = 0;
	}
	else if (ucState & LED_YELLOW_BLINK)
	{
		gPLCCtl.doDataOut[LED_Y] ^= 1;
	}

	//red
	if (ucState & LED_RED_SET)
	{
		gPLCCtl.doDataOut[LED_R] = 1;
	}
	else if (ucState & LED_RED_RESET)
	{
		gPLCCtl.doDataOut[LED_R] = 0;
	}
	else if (ucState & LED_RED_BLINK)
	{
		gPLCCtl.doDataOut[LED_R] ^= 1;
	}
}
/*******************************************************************************
* Name: LEDInit
* Description: 指示灯显示初始化
* Input: 
* Output: 
*
* Author:
* Date:
* Revision:
*******************************************************************************/
void LEDInit(void)
{
	gPLCCtl.LedCountHead = 0;
	gPLCCtl.LedCountRep = 0;
	gPLCCtl.LedCount = 0;
	gPLCCtl.LedBlinkPeriod = 0;	
	LedStateShow(LED_YELLOW_RESET | LED_RED_RESET);
}

/*******************************************************************************
* Name: LEDProcess
* Description: 指示灯显示
* Input: 
* Output: 
*
* Author:
* Date:
* Revision:
*******************************************************************************/
void LEDProcess(void)
{
	//gCRam.ErrCode=99; //23,1,9,10,11,15,19,20,99 
	if (gPLCCtl.LedBlinkPeriod >= LED_BLINK_PERIOD)
	{
		gPLCCtl.LedBlinkPeriod = 0;
		if (gPLCCtl.LedCount > 0)  //First, finish current process
		{
			if (gPLCCtl.LedCountHead <= 0) //Head over
			{
				if (gPLCCtl.LedCount >= 10) //digit 10
				{
					if (gPLCCtl.LedCountRep != 0)
					{
						LedStateShow(LED_YELLOW_BLINK | LED_RED_RESET);
						gPLCCtl.LedCountRep = 0;
						gPLCCtl.LedCount -= 10;
					}
					else
					{
						gPLCCtl.LedCountRep = 1;
						LedStateShow(LED_YELLOW_BLINK | LED_RED_RESET);
					}
				}
				else  //digit 1
				{
					if (gPLCCtl.LedCountRep != 0)
					{
						LedStateShow(LED_YELLOW_RESET | LED_RED_BLINK);
						gPLCCtl.LedCountRep = 0;
						gPLCCtl.LedCount -= 1;
					}
					else
					{
						gPLCCtl.LedCountRep = 1;
						LedStateShow(LED_YELLOW_RESET | LED_RED_BLINK);
					}
				}
			}
			else //Head delay
			{
				gPLCCtl.LedCountHead--;
			}
		}
		else if (gPLCCtl.ErrCode != 0) //Second,Err
		{
			gPLCCtl.LedCount = gPLCCtl.ErrCode;
			gPLCCtl.LedCountRep = 0;
			gPLCCtl.LedCountHead = 2;
			LedStateShow(LED_YELLOW_RESET | LED_RED_RESET);
		}
		else if (gPLCCtl.AlmCode != 0) //Third, Alm 
		{
			gPLCCtl.LedCount = gPLCCtl.AlmCode;
			gPLCCtl.LedCountRep = 0;
			gPLCCtl.LedCountHead = 2;
			LedStateShow(LED_YELLOW_RESET | LED_RED_RESET);
		}
		else //No Err & No Alm
		{
			gPLCCtl.LedCount = 0;
			gPLCCtl.LedCountRep = 0;
			gPLCCtl.LedCountHead = 2;
			LedStateShow(LED_YELLOW_SET | LED_RED_RESET);
		}
	}
	else
	{
		gPLCCtl.LedBlinkPeriod += PLC_PERIOD;
	}
}

