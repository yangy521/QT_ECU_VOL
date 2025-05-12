/*******************************************************************************
* Filename: Kernel.c                                             	 		   	   *
*                                                                    		   *
* Description:											   			 		   *
* Author:                                                                   *
* Date: 														           *
* Revision:															 		   *
*																	 		   *
*******************************************************************************/

#include	"Kernel.h"
#include	"Commonram.h"
#include	"Current.h"
#include	"PropDriver.h"
#include	"Speed.h"
#include	"ServoPara.h"
#include	"Fdb.h"
#include	"PLC.h"
#include	"iTimer.h"
#include	"PARA.h"
#include  "PLCHardware.h"

KERNEL_CTL		gKernelCtl;

/*******************************************************************************
* Name: InitServoCtlLoop
* Description:
* Input: 
* Output: 
*
* Author: 
* Date: 
* Revision:
*******************************************************************************/
void InitServoCtlLoop(void)
{
	/* current loop */
	gCurrentLoop.Init = CurrentLoopInit;
	gCurrentLoop.Calc = CurrentLoopCalc;
	gCurrentLoop.Reset = CurrentLoopReset;
	gCurrentLoop.Init(&gCurrentLoop);	

	/* speed loop */
	gSpeedLoop.Init = SpeedLoopInit;
	gSpeedLoop.Calc = SpeedLoopCalc;
	gSpeedLoop.Reset = SpeedLoopReset;
	gSpeedLoop.Init(&gSpeedLoop);

	/*prop driver current loop */
#if ((CTLBOARD_TYPE ==_1236) || (CTLBOARD_TYPE ==_1246) || (CTLBOARD_TYPE ==_1226) || (CTLBOARD_TYPE ==_1226_GD32))
	gPropCurrentLoop.Init = PropCurrentLoopInit;
	gPropCurrentLoop.Calc = PropCurrentLoopCalc;
	gPropCurrentLoop.Reset = PropCurrentLoopReset;
	gPropCurrentLoop.Init(&gPropCurrentLoop);	
#endif
	/* calculate servo loop parameters */
	CalcServoLoopPara();
}

/*******************************************************************************
* Name: AlarmErrorMonitor
* Description: 4ms period. Run in ISR. or called by NcCommMeLinkIICalc.
* Input: 
* Output: 
* 
* Author:
* Date:
* Revision:
*******************************************************************************/
void AlarmErrorMonitor(void)
{
	INT32S t32S;
	
	/******************************* 故障监测**********************************/
	/* if no error */
	if(gCRam.ErrCode == 0)
	{
		/* No.1: 反馈超速 */
		t32S = _IQrmpy(gCRam.SpeedFdbDisp, STD_SPEEDRPM);
		t32S = (t32S >= 0 ? t32S : -t32S);
		if(t32S > ((INT32S)_IQrmpy(gKernelCtl.MotorMaxHz, _IQ(1.1))))
		{
			PLCErrorTips(1);
		}
		
		/* No.2: 内核运行错误 */
		if(SL_CHK(SL_KERNEL_ERR))
		{
			PLCErrorTips(2);
			SL_CLR(SL_KERNEL_ERR);
		}

		/* No.3: 控制器及电机长时间过载 */
		if(SL_CHK(SL_OVER_LOAD_ERR))
		{
			PLCErrorTips(3);
			SL_CLR(SL_OVER_LOAD_ERR);
		}		
		
		/* No.4: 位置超差 */


		/* No.5: 电子齿轮比之后的位置指令变化超过电机最大速度 */
		if(SL_CHK(SL_POSCMD_OV))
		{
			PLCErrorTips(5);
			SL_CLR(SL_POSCMD_OV);
		}		
		
		/* No.6: 速度模式时，速度指令超过电机最大转速 */
		if(SL_CHK(SL_SPDCMD_OV))
		{
			PLCErrorTips(6);
			SL_CLR(SL_SPDCMD_OV);
		}
		
		/* No.7: 转矩模式时，转矩指令超过电机最大转矩 */
		if(SL_CHK(SL_TORCMD_OV))
		{
			PLCErrorTips(7);
			SL_CLR(SL_TORCMD_OV);
		}
		
		/* No.8: 速度传感器错误 */ 
		/* 速度环长时间饱和，而速度反馈不存在*/
		if(SL_CHK(SL_ENCODER_UVW_ERR))
		{
			PLCErrorTips(8);
			SL_CLR(SL_ENCODER_UVW_ERR);
		}
		/* No.9: 速度传感器方向错误 */ 
		/* 速度环长时间饱和，而速度反馈反向*/
		if(SL_CHK(SL_ENCODER_DIR_ERR))
		{
			PLCErrorTips(9);
			SL_CLR(SL_ENCODER_DIR_ERR);
		}
		
		/* No.11: 电机2分钟最大电流保护 */
		if(SL_CHK(SL_2MOVER_CURRENT_ERR))
		{
			PLCErrorTips(11);
		}
		
		/* No.12: 电机过电流 */
		if(SL_CHK(SL_OVER_CURRENT_ERR))
		{
			PLCErrorTips(12);
			SL_CLR(SL_OVER_CURRENT_ERR);
		}
	}
}

/*******************************************************************************
* Name: OverlLoadCheck
* Description: 1s period. Run in background.
* Input: 
* Output: 
* 
* Author:
* Date:
* Revision:
*******************************************************************************/
#ifndef _RELEASE
//#define DEBUG_CURRENT_MASK_OVCHK
#endif
void OverlLoadCheck(void)
{
	INT32S IvCurrent;

	/* No.12: 电机电流瞬时超过最大过载值 */
	//IvCurrent = gCurrentLoop.CurrentFilter.Display;
	IvCurrent = gCurrentLoop.CurrentFilter.Out;
	if(IvCurrent > gKernelCtl.MaxOverLoadCurrent)
	{
//#ifndef DEBUG_CURRENT_MASK_OVCHK
		PwmDisableEsp();
		SL_SET(SL_OVER_CURRENT_ERR);
//#endif
	}
	IvCurrent = gCurrentLoop.CurrentFilter.Display;
	if (IvCurrent > gKernelCtl.OverLoadCurrent2M)
	{
		if (gKernelCtl.OverLoadCurrentCount2M >= (FS * 120))  //120s
		{
			SL_SET(SL_2MOVER_CURRENT_ERR);
			gKernelCtl.OverLoadCurrentCount2M = 0;
		}
		else
			gKernelCtl.OverLoadCurrentCount2M += 1;
	}
	else
	{
		gKernelCtl.OverLoadCurrentCount2M = 0;
	}
	
	//泵控2分钟过流检测
	IvCurrent = gPropCurrentLoop.IFdb;
	if (IvCurrent > gKernelCtl.PumpOverLoadCurrent2M)
	{
		if (gKernelCtl.PumpOverLoadCurrentCount2M >= (FS * 120))  //120s
		{
			SL_SET(SL_2MOVER_CURRENT_ERR);
			gKernelCtl.PumpOverLoadCurrentCount2M = 0;
		}
		else
			gKernelCtl.PumpOverLoadCurrentCount2M += 1;
	}
	else
	{
		gKernelCtl.PumpOverLoadCurrentCount2M = 0;
	}

	//母线电压低于最小值
	if (gCurrentLoop.VbusFdb < gKernelCtl.EspLowVoltage)
	{
		//kff
		//SL_SET(SL_ESP_VOLTAGE_ERR);
		//PwmDisableEsp();
	}
	
	//母线电压高于最大值
	if (gCurrentLoop.VbusFdb > gKernelCtl.EspHighVoltage)
	{
		SL_SET(SL_ESP_VOLTAGE_ERR);
		PwmDisableEsp();
		gPLCCtl.doDataOut[DRIVER_EN] = 1; //关闭DriverEN
	}

}

/*******************************************************************************
* Name: GetCurrentLoopRefFdb
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void GetCurrentLoopRefFdb(void)
{
	_iq t32S;

	/* get current loop ref */
	if(gKernelCtl.CurRunMode == TORQUE_RUN_MODE)
	{
		gCurrentLoop.IdRef = 0;
		/* get external input torque command */
		t32S = gCRam.TorqueCmd;
		//Reg limit
		if (_IQ12toIQ(gCRam.CurrentSampleVBus) > (gCRam.SvPa.DcBrkLimitVoltage * _IQ(1.0/(10.0 * STD_VBUS))))
			t32S = 0;
		/* 转矩偏置 */
		/* gCRam.SvPa.TorqueOffset为0.01Nm单位，
		gCRam.TorqueCmd为Q16格式Nm单位 */		
		t32S += (INT32S)gCRam.SvPa.TorqueOffset * 655;
		/* 检查转矩指令是否超过电机过载能力 */
		if((t32S > gKernelCtl.MaxCCWTorque) || (t32S < -gKernelCtl.MaxCWTorque))
		{
			gCurrentLoop.IqRef = 0;
			SL_SET(SL_TORCMD_OV);
		}
		else
		{
			t32S = _IQmpy(t32S << 8, _IQ(1.0/STD_TORQUE_K/STD_CURRENT));
			gCurrentLoop.IqRef = _IQdiv(t32S, gCRam.SvPa.MotorPara.KTorque);
			/* 转矩方向 */
			if(BIT_PARA_MOTOR_ROT_DIR)
			{
				gCurrentLoop.IqRef = -gCurrentLoop.IqRef;
			}
		}
	}
	else
	{
		/* get torque from speed loop */
		//gCurrentLoop.IqRef = gSpeedLoop.CurrentCmd;
		//Reg limit
		if (gSpeedLoop.VoltageBus > gSpeedLoop.VoltageBusLimit)
			gCurrentLoop.IqRef = -gSpeedLoop.CurrentCmd;
		else
			gCurrentLoop.IqRef = gSpeedLoop.CurrentCmd;			

		gCurrentLoop.IdRef = 0;
		gCurrentLoop.State = gSpeedLoop.State;
	}
	if (SL_CHK(SL_2MOVER_CURRENT_ERR))
	{
		gCurrentLoop.IqRef = 0;
		gCurrentLoop.IdRef = 0;
	}

	/* get current loop fdb */
	gCurrentLoop.U = _IQ11toIQ(gCRam.CurrentSampleW);
	gCurrentLoop.W = _IQ11toIQ(gCRam.CurrentSampleU);
	gCurrentLoop.RotorAngle = _IQ15toIQ(gCRam.RotorElecAngle);

	/* get motor speed to calc BackELecForce  */
	gCurrentLoop.SpeedFilter = gSpeedLoop.Filter.Out;
	/* get Vbus */
	gCurrentLoop.VbusFdb = _IQ12toIQ(gCRam.CurrentSampleVBus);
}

/*******************************************************************************
* Name: PumpSpeedSmooth
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void PumpSpeedSmooth(void)
{
	PROP_CURRENT_LOOP *pSpeedLoop = &gPropCurrentLoop;	
	_iq SpeedErr;
	static INT16U Cnt=0;
	
	if(Cnt++>9)  //test add 
	{
		Cnt=0;
		SpeedErr=gCRam.PumpSpeedCmd-pSpeedLoop->IRef;

		if(SpeedErr>0)
			pSpeedLoop->IRef += _IQ(0.002);//pSpeedLoop->SpdAccRatioAct; 
		else if(SpeedErr<0)
		{
			if(SL_CHK(SL_TER_PROP_CURRENT_CUT))
				pSpeedLoop->IRef += _IQ(0.002);//pSpeedLoop->SpdAccRatioAct;
			else	
				pSpeedLoop->IRef -= _IQ(0.002);//pSpeedLoop->SpdAccRatioAct; 
			SpeedErr = -SpeedErr;
		}
//		if(SpeedErr<pSpeedLoop->SpdAccRatioAct) 	
		if(SpeedErr<_IQ(0.005)) 				
			pSpeedLoop->IRef = gCRam.PumpSpeedCmd;	
	}
}

/*******************************************************************************
* Name: GetPropDriverRefFdb
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
#if ((CTLBOARD_TYPE ==_1236) || (CTLBOARD_TYPE ==_1246) || (CTLBOARD_TYPE ==_1226) || (CTLBOARD_TYPE ==_1226_GD32))
void GetPropDriverRefFdb(void)
{
	/* get prop driver current loop fdb */
	//gPropCurrentLoop.IFdb = _IQ12toIQ(gCurrentSample.PropOvCur);
	gPropCurrentLoop.IFdb = _IQ11toIQ(-gCurrentSample.P);
	
	/* get prop driver current loop ref */
//	gPropCurrentLoop.IRef = gCRam.PropCurrentCmd*_IQ(PROPD_MAX_CURRENT / PROPD_STD_CURRENT / 65536.0);	
//	if (gPropCurrentLoop.IRef  > gPropCurrentLoop.IRefMax)
//		gPropCurrentLoop.IRef = gPropCurrentLoop.IRefMax;
		PumpSpeedSmooth();
//		if(SL_CHK(PLC_TER_CURRENT_CUT))
//			gPropCurrentLoop.IRefMax = 0;	//消减功率禁止起升
}
#endif
/*******************************************************************************
* Name: SpeedSmoothPos
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void SpeedSmoothPos(void)
{
	SPEED_LOOP *pSpeedLoop = &gSpeedLoop;
	pSpeedLoop->SpdRef = pSpeedLoop->SpdCmd;
}

/*******************************************************************************
* Name: SpeedSmooth
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void SpeedSmooth(void)
{
	SPEED_LOOP *pSpeedLoop = &gSpeedLoop;	
	_iq SpeedErr;
	static INT16U Cnt=0;
	
	if(SL_CHK(PLC_CURRENT_LIMIT))
	{	
		pSpeedLoop->SpdRef = pSpeedLoop->SpdRef + gCurrentLoop.IqPID.Out;
	}
	else
	{	
		if(Cnt++>9)  //test add 
		{
			Cnt=0;
			SpeedErr=pSpeedLoop->SpdCmd-pSpeedLoop->SpdRef;

			if(SpeedErr>0)
				pSpeedLoop->SpdRef += pSpeedLoop->SpdAccRatioAct; 
			else if(SpeedErr<0)
			{
				pSpeedLoop->SpdRef -= pSpeedLoop->SpdAccRatioAct; 
				SpeedErr = -SpeedErr;
			}
			if(SpeedErr<pSpeedLoop->SpdAccRatioAct) 		
				pSpeedLoop->SpdRef = pSpeedLoop->SpdCmd;	
		}
			
	}
}

/*******************************************************************************
* Name: GetSpeedLoopRefFdb
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void GetSpeedLoopRefFdb(void)
{
	/* get speed loop ref */
	/* if position run mode */
	if(gKernelCtl.CurRunMode == POS_RUN_MODE)
	{
		//gSpeedLoop.SpdCmd = gPositionLoop.SpeedCmd;
		SpeedSmoothPos();
	}
	/* else if external input speed run mode */
	else if(gKernelCtl.CurRunMode == SPEED_RUN_MODE)
	{
		/* get external input speed command */
		INT32S ExtSpeedCmd;
		ExtSpeedCmd = gCRam.SpeedCmd;
		if((ExtSpeedCmd >> 16) > (INT32S)gKernelCtl.MotorMaxHz){
			ExtSpeedCmd = (INT32S)gKernelCtl.MotorMaxHz << 16;
		}else if ((ExtSpeedCmd >> 16) < -(INT32S)gKernelCtl.MotorMaxHz){
			ExtSpeedCmd = -(INT32S)gKernelCtl.MotorMaxHz << 16;
		}
		gSpeedLoop.SpdCmd = ExtSpeedCmd * _IQ(1.0 / STD_SPEEDRPM / 65536.0);
		gSpeedLoop.BrakeCmd = gCRam.BrakeCmd * _IQ(1.0 / STD_PERCENT);
		/* rotation direction */
		if(BIT_PARA_MOTOR_ROT_DIR)
		{
			gSpeedLoop.SpdCmd = -gSpeedLoop.SpdCmd;
		}
		//Normal,Slow,Fast mode set
		if (gCRam.SpeedMode == SLOW_MODE)
		{
			gSpeedLoop.SpdAccRatioAct = gSpeedLoop.SpdAccRatioSlow>>6;	//gSpeedLoop.SpdAccRatioSlow
			gSpeedLoop.SpdDecRatioAct = gSpeedLoop.SpdDecRatioSlow>>6;	//gSpeedLoop.SpdDecRatioSlow
		}
		else if (gCRam.SpeedMode == FAST_MODE)
		{
			gSpeedLoop.SpdAccRatioAct = gSpeedLoop.SpdAccRatioFast>>6;
			gSpeedLoop.SpdDecRatioAct = gSpeedLoop.SpdDecRatioFast>>6;
		}
		else
		{
			gSpeedLoop.SpdAccRatioAct = gSpeedLoop.SpdAccRatioFast>>6;	//gSpeedLoop.SpdAccRatioAct = _IQ(1.0);
			gSpeedLoop.SpdDecRatioAct = gSpeedLoop.SpdDecRatioFast>>6;	//gSpeedLoop.SpdDecRatioAct = _IQ(1.0);
		}
		SpeedSmooth();
	}
	else
	{
		/* blank */
	}

	/* get speed loop fdb */
	gSpeedLoop.SpdFdb = gCRam.SpeedFdb;
	gSpeedLoop.IqFdb = gCurrentLoop.IqFdb;	/* for Speed Observer */
	gSpeedLoop.IvFdb = gCurrentLoop.IvFdb;	/* for current limit */
	gSpeedLoop.VoltageBus = _IQ12toIQ(gCRam.CurrentSampleVBus);
}

/*******************************************************************************
* Name: UpdateKerOutputVar
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void UpdateKerOutputVar(void)
{
	/* set common ram area */
	gCRam.SpeedRef = gSpeedLoop.SpdRef;
	gCRam.SpeedFdbDisp = gSpeedLoop.Filter.Display;
	gCRam.CurrentFdbIq = gCurrentLoop.IqFdb;
	/* set monitor data */
	gPara.AcMotorSpeed = gCRam.SpeedFdbDisp;//_IQrmpy(gCRam.SpeedFdbDisp, STD_SPEEDRPM);		//电机当前转速
	gPara.AcMotorSpeedF = _IQrmpy(gCRam.SpeedRef, STD_SPEEDRPM);    //转速命令
	//gPara.AcPhaseCurrent = _IQrmpy(gCurrentLoop.CurrentFilter.Display,STD_CURRENT/SQRT2/PH3_TO_PH2);	//电机当前相电流
	gPara.AcPhaseCurrent = _IQrmpy(gCurrentLoop.CurrentFilter.Out,STD_CURRENT);	//电机当前相电流(直流)
	//gPara.MotorEncoder = gEncoder.PosAbs;		//电机编码器输入
  /*Dir*/
	if (BIT_PARA_MOTOR_ROT_DIR)
	{
		gPara.AcMotorSpeed = -gPara.AcMotorSpeed;
		gPara.AcMotorSpeedF = -gPara.AcMotorSpeedF;
		gPara.MotorEncoder = -gPara.MotorEncoder;
	}
}

/*******************************************************************************
* Name: FsmInactive
* Description: Call by KernelControl
* Input: 
* Output: 
*
* Author:
* Date:
* Revision:
*******************************************************************************/
void FsmInactive(void)
{
//	if(SL_CHK(SL_POWER_RDY) && SL_CHK(SL_TER_DALM_N) && SL_CHK(SL_TER_SRVON))  //debug del
	if(SL_CHK(SL_POWER_RDY) && SL_CHK(SL_TER_DALM_N))  
	{
		gKernelCtl.ReadyRun = 1;
		/* 记录当前运行模式 */
		gKernelCtl.CurRunMode = SPEED_RUN_MODE;//gCRam.SvPa.RunMode;  //debug del Chow
		/* 设置APR, ASR, ACR */
		if(gKernelCtl.CurRunMode == POS_RUN_MODE)
		{
			SL_SET(SL_APR_RUN);
			SL_SET(SL_ASR_RUN);
			SL_SET(SL_ACR_RUN);
		}
		else if(gKernelCtl.CurRunMode == SPEED_RUN_MODE) 
		{
			SL_CLR(SL_APR_RUN);
			SL_SET(SL_ASR_RUN);
			SL_SET(SL_ACR_RUN);
		}
		else if(gKernelCtl.CurRunMode == TORQUE_RUN_MODE)
		{
			SL_CLR(SL_APR_RUN);
			SL_CLR(SL_ASR_RUN);
			SL_SET(SL_ACR_RUN);
		}
		else
		{
			SL_CLR(SL_APR_RUN);
			SL_CLR(SL_ASR_RUN);
			SL_CLR(SL_ACR_RUN);
			SL_SET(SL_KERNEL_ERR);
		}
		/* 转TO_BRKOFF状态 */
		gKernelCtl.GeneralCnt = 0;
		gKernelCtl.FSMState = FSMSTATE_TO_BRKOFF;
	}
	else
	{
		SL_CLR(SL_APR_RUN);
		SL_CLR(SL_ASR_RUN);
		SL_CLR(SL_ACR_RUN);
		SL_CLR(SL_TER_PROP);
		gKernelCtl.ReadyRun = 0;
		/* 理论上不需清除SL_TER_EN_PWM, 此处为保险起见 */
		SL_CLR(SL_TER_EN_PWM);
		/* 保持INACTIVE状态 */
		gKernelCtl.FSMState = FSMSTATE_INACTIVE;
	}
}

/*******************************************************************************
* Name: FsmToBrkOff
* Description: Call by KernelControl
* Input: 
* Output: 
*
* Author:
* Date:
* Revision:
*******************************************************************************/
void FsmToBrkOff(void)
{
	/* enable PWM output */
	SL_SET(SL_TER_EN_PWM);
	/* wait about 200ms, then set SL_TER_BRK_N to host */
	if ((gKernelCtl.GeneralCnt < 20) && ((gCRam.SvPa.DriverEn1 & BrakeEnable) != 0))
	{
		gKernelCtl.GeneralCnt++;
		SL_CLR(SL_TER_BRK_N);
		/* 保持TO_BRKOFF状态 */
		gKernelCtl.FSMState = FSMSTATE_TO_BRKOFF;
	}
	else
	{
		SL_SET(SL_TER_BRK_N);
		gKernelCtl.GeneralCnt = 0;
		/* 转ACTIVE状态 */
		gKernelCtl.FSMState = FSMSTATE_ACTIVE;
	}
}

/*******************************************************************************
* Name: FsmActive
* Description: Call by KernelControl
* Input: 
* Output: 
*
* Author:
* Date:
* Revision:
*******************************************************************************/
void FsmActive(void)
{
	if(SL_CHK(SL_POWER_RDY) && SL_CHK(SL_TER_DALM_N) && SL_CHK(SL_TER_SRVON))
	{
		gKernelCtl.ReadyRun = 1;
		gKernelCtl.NextRunMode = gCRam.SvPa.RunMode;
		/* 若控制模式未改变, 则保持ACTIVE状态 */
		if(gKernelCtl.NextRunMode == gKernelCtl.CurRunMode)
		{
			gKernelCtl.FSMState = FSMSTATE_ACTIVE;
		}
		/* 否则, 转PREPARE状态 */
		else
		{
			gKernelCtl.FSMState = FSMSTATE_PREPARE;
		}
	}
	else
	{
		gKernelCtl.ReadyRun = 0;
		/* 转PREPARE状态 */
		gKernelCtl.FSMState = FSMSTATE_PREPARE;
	}
}

/*******************************************************************************
* Name: FsmPrepare
* Description: Call by KernelControl
* Input: 
* Output: 
*
* Author:
* Date:
* Revision:
*******************************************************************************/
void FsmPrepare(void)
{
	if(gKernelCtl.ReadyRun == 1)
	{
		/* 当前设计为: 任何运行模式间的切换都需反馈速度低于特定值 */
		/* 反馈速度低于0.5hZ */
		if( (gCRam.SpeedFdb < _IQ( 5 / STD_SPEEDRPM)) && 
			(gCRam.SpeedFdb > _IQ(-5 / STD_SPEEDRPM)) )
		{
			gKernelCtl.CurRunMode = gKernelCtl.NextRunMode;
			if(gKernelCtl.CurRunMode == POS_RUN_MODE)
			{
				SL_SET(SL_APR_RUN);
				SL_SET(SL_ASR_RUN);
				SL_SET(SL_ACR_RUN);
			}
			else if(gKernelCtl.CurRunMode == SPEED_RUN_MODE) 
			{
				SL_CLR(SL_APR_RUN);
				SL_SET(SL_ASR_RUN);
				SL_SET(SL_ACR_RUN);
			}
			else if(gKernelCtl.CurRunMode == TORQUE_RUN_MODE)
			{
				SL_CLR(SL_APR_RUN);
				SL_CLR(SL_ASR_RUN);
				SL_SET(SL_ACR_RUN);
			}
			else
			{
				SL_CLR(SL_APR_RUN);
				SL_CLR(SL_ASR_RUN);
				SL_CLR(SL_ACR_RUN);
				SL_SET(SL_KERNEL_ERR);
			}
			/* 转ACTIVE状态 */
			gKernelCtl.FSMState = FSMSTATE_ACTIVE;
		}
		else
		{
			/* 检测运行条件, 以避免在PREPARE状态无限循环 */
			if(!(SL_CHK(SL_POWER_RDY) && SL_CHK(SL_TER_DALM_N) && 
				SL_CHK(SL_TER_SRVON)))
			{
				gKernelCtl.ReadyRun = 0;
				/* 保持PREPARE状态 */
				gKernelCtl.FSMState = FSMSTATE_PREPARE;
			}
			else
			{
				/* 监测参数选择的当前控制模式 */
				gKernelCtl.NextRunMode = gCRam.SvPa.RunMode;
				/* 若控制模式变回原模式, 则转ACTIVE状态 */
				if(gKernelCtl.NextRunMode == gKernelCtl.CurRunMode)
				{
					gKernelCtl.FSMState = FSMSTATE_ACTIVE;
				}
				else
				{
					/* 保持PREPARE状态 */
					gKernelCtl.FSMState = FSMSTATE_PREPARE;
				}
			}
		}
	}
	else
	{
		/* 当前设计为: 无论反馈速度多大, 无论是否能够继续控制电机,
		均立即使能BRK */
		/* 转TO_PWMOFF状态 */
		gKernelCtl.GeneralCnt = 0;
		gKernelCtl.FSMState = FSMSTATE_TO_PWMOFF;
	}
}

/*******************************************************************************
* Name: FsmToPwmOff
* Description: Call by KernelControl
* Input: 
* Output: 
*
* Author:
* Date:
* Revision:
*******************************************************************************/
void FsmToPwmOff(void)
{
	/* appeal to host to enable the external brake immediately */
	SL_CLR(SL_TER_BRK_N);

	/* 若主电源失效, 或DALM_N有效 */
	if(!SL_CHK(SL_POWER_RDY) || !SL_CHK(SL_TER_DALM_N))
	{
		/* 立即关闭PWM, 转INACTIVE状态 */
		SL_CLR(SL_TER_EN_PWM);
		gKernelCtl.GeneralCnt = 0;
		/* 转INACTIVE状态 */
		gKernelCtl.FSMState = FSMSTATE_INACTIVE;
	}
	/* 否则, 是正常SVON OFF时序 */
	else
	{
		/* after command brake to host, wait for milliseconds, then
		disable the control to motor */
		gKernelCtl.GeneralCnt ++;
		if(  ((gCRam.SvPa.DriverEn1 & BrakeEnable) != 0)
			&& (gKernelCtl.GeneralCnt < (gCRam.SvPa.ExtBrakeHoldTime >> 1)))
		{
			SL_SET(SL_TER_EN_PWM);
			/* 保持TO_PWMOFF状态 */
			gKernelCtl.FSMState = FSMSTATE_TO_PWMOFF;
		}
		else
		{
			SL_CLR(SL_TER_EN_PWM);
			gKernelCtl.GeneralCnt = 0;
			/* 转INACTIVE状态 */
			gKernelCtl.FSMState = FSMSTATE_INACTIVE;
		}
	}
}

/*******************************************************************************
* Name: KernelControl
* Description: 4ms period. Run in ISR.
* Input: 
* Output: 
*
* Author:
* Date:
* Revision:
*******************************************************************************/
void KernelControl(void)
{
//	INT32S t32S;

	/* 1. 设置输出: SL_TER_COIN_N, SL_TER_DALM_N, SL_TER_SRDY */
	/* (1) update COIN_N */
//	if(gPositionLoop.PID.Err < 0)
//	{
//		t32S = -gPositionLoop.PID.Err;
//	}
//	else
//	{
//		t32S = gPositionLoop.PID.Err;
//	}
//	if(gCRam.SvPa.EncoderType == INC_ENCODER_2500)
//	{
//		t32S = _IQmpy(t32S,	_IQ(1000.0 / STD_INC2500));
//	}
//	else
//	{
//		t32S = 0;
//	}
//	/* PosReachThreshold is 1/1000 r unit */
//	if(t32S > gCRam.SvPa.PosReachThreshold)
//	{
//		/* not position on status */
//		SL_SET(SL_TER_COIN_N);
//	}
//	else
//	{
//		/* position on status */
//		SL_CLR(SL_TER_COIN_N);
//	}

	/* (2) set DALM_N when error code not 0*/
	if(gCRam.ErrCode != 0)
	{
		SL_CLR(SL_TER_DALM_N);
	}
	else
	{
		SL_SET(SL_TER_DALM_N);
	}

	/* (3) 设置SL_POWER_RDY: SL_TER_RDY有效40ms后有效 */
	if(SL_CHK(SL_TER_RDY))
	{
//		SL_SET(SL_TER_EN_BRAKE);
		if(SL_CHK(SL_POWER_RDY) == 0)
		{
			gKernelCtl.PowerRdyCnt ++;
			/* if 40ms time up */
			if(gKernelCtl.PowerRdyCnt > 8)
			{
				gKernelCtl.PowerRdyCnt = 0;
				SL_SET(SL_POWER_RDY);
			}
		}
	}
	else
	{
		gKernelCtl.PowerRdyCnt = 0;
		SL_CLR(SL_POWER_RDY);
	}

	/* (4) 设置SRDY: 有报警时立即无效, 无报警时, 等同于SL_POWER_RDY */
	if(SL_CHK(SL_TER_DALM_N) == 0)
	{
		SL_CLR(SL_TER_SRDY);
	}
	else
	{
		if(SL_CHK(SL_POWER_RDY))
		{
			SL_SET(SL_TER_SRDY);
		}
		else
		{
			SL_CLR(SL_TER_SRDY);
		}
	}

	/* 2. FSM */
	switch(gKernelCtl.FSMState)
	{
	case FSMSTATE_INACTIVE:  FsmInactive(); break;
	case FSMSTATE_TO_BRKOFF: FsmToBrkOff(); break;
	case FSMSTATE_ACTIVE:    FsmActive();   break;
	case FSMSTATE_PREPARE:   FsmPrepare();  break;
	case FSMSTATE_TO_PWMOFF: FsmToPwmOff(); break;
	default: gKernelCtl.FSMState = FSMSTATE_INACTIVE; 
					 SL_SET(SL_KERNEL_ERR);
	}
}

/*******************************************************************************
* Name: KernelRun
* Description: 125us period. Run in ISR.
* Input: 
* Output: 
*
* Author:
* Date:
* Revision:
*******************************************************************************/
void KernelRun(void)
{
	/* current control */
	if(SL_CHK(SL_ACR_RUN))
	{
		GetCurrentLoopRefFdb();
		gCurrentLoop.Calc(&gCurrentLoop);
		OverlLoadCheck();
	}
	else
	{
		/* reset current loop */
		gCurrentLoop.Reset(&gCurrentLoop);
	}
#if ((CTLBOARD_TYPE ==_1236) || (CTLBOARD_TYPE ==_1246) || (CTLBOARD_TYPE ==_1226) || (CTLBOARD_TYPE ==_1226_GD32))
	if(SL_CHK(SL_TER_PROP))
	{
		GetPropDriverRefFdb();
		gPropCurrentLoop.Calc(&gPropCurrentLoop);
	}
	else
	{
		/* reset current loop */
		gPropCurrentLoop.Reset(&gPropCurrentLoop);		
	}
#endif
	/* speed control */
	if(SL_CHK(SL_ASR_RUN))
	{
		GetSpeedLoopRefFdb();
		gSpeedLoop.Calc(&gSpeedLoop);
	}
	else
	{
		/* reset speed loop */
		gSpeedLoop.Reset(&gSpeedLoop);
	}

	/* 计算电流采样电路零漂 */
	{
		static INT16S svOffDelay = 0;
		if(gKernelCtl.FSMState == FSMSTATE_INACTIVE)
		{
			/* calculate current sample zero drift */
			if (svOffDelay > 0)
			{
				svOffDelay--;
			}
			else
			{
				gCurrentSample.Calc_ZeroDrift(&gCurrentSample);
			}
		}
		else
		{
			svOffDelay = FS;
		}

		static INT16S PumpOffDelay = 0;		
		if(gCRam.PumpSpeedCmd==0)//Pump closed
		{		
			/* calculate current sample zero drift */
			if (PumpOffDelay > 0)
			{
				PumpOffDelay--;
			}
			else
			{
				gCurrentSample.Calc_PropZeroDrift(&gCurrentSample);
			}
		}
		else
		{
			PumpOffDelay = FS;
		}		
	}

	/* speed feedback filter */
	gSpeedLoop.Filter.In = gCRam.SpeedFdb;
	gSpeedLoop.Filter.Calc(&gSpeedLoop.Filter);

	/* DIO, run control state machine, alarm error monitor and updatapara */
	if(netTimer[Timer_Logic].bIsOvertime) //5ms
	{
		if(SL_CHK(SL_LOGIC_BUSY))
		{
			gKernelCtl.OverLoadCnt++;
		}
		else
		{
			PLCDataSwap();
			exti_software_interrupt_enable(EXTI_15);//EXTI_GenerateSWInterrupt(EXTI_Line0);
		}
		KernelControl();/* 内核控制 */
		AlarmErrorMonitor();/* 警告及故障监测 */
		ResetNetTimer(Timer_Logic);
	}
	/* update kernel output variables in common ram area */
	UpdateKerOutputVar();
}

/*******************************************************************************
* Name: InitKernel
* Description: 
* Input: 
* Output: 
*
* Author:
* Date:
* Revision:
*******************************************************************************/
void InitKernel(void)
{
	gKernelCtl.FSMState = FSMSTATE_INACTIVE;
	gKernelCtl.ReadyRun = 0;
	gKernelCtl.CurRunMode = gCRam.SvPa.RunMode;
	gKernelCtl.NextRunMode = gCRam.SvPa.RunMode;
	gKernelCtl.GeneralCnt = 0;
	gKernelCtl.PowerRdyCnt = 0;
	gKernelCtl.BrakeCnt = 0;
	gKernelCtl.SaturatedCnt = 0;
	gKernelCtl.AClrCnt = 0;
	gKernelCtl.OverLoadCnt = 0;

	gKernelCtl.CalibAngleInit = 0;
	gKernelCtl.EncoderTypeInit = 0;
	gKernelCtl.MotorMaxHz = 0;
	gKernelCtl.MaxCCWTorque = 0;
	gKernelCtl.MaxCWTorque = 0;
	gKernelCtl.MaxOverLoadCurrent = 0;
	gKernelCtl.OverLoadCurrent2M = 0;
	gKernelCtl.OverLoadCurrentCount2M = 0;
	gKernelCtl.PumpMaxOverLoadCurrent = 0;
	gKernelCtl.PumpOverLoadCurrent2M = 0;
	gKernelCtl.PumpOverLoadCurrentCount2M = 0;
	gKernelCtl.EspLowVoltage = 0;
	gKernelCtl.EspHighVoltage = 0;

	SL_CLR(SL_APR_RUN);
	SL_CLR(SL_ASR_RUN);
	SL_CLR(SL_ACR_RUN);
	SL_CLR(SL_TER_EN_PWM);

//	SL_CLR(SL_TER_EN_BRAKE);
	InitServoCtlLoop();

	/* set error No 0 into error trace for separate the last poweron error No */
	PLCErrorTips(0);
}

