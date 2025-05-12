/*******************************************************************************
* Filename: CommonRam.c                                             	 		   	   *
*                                                                    		   *
* Description:											   			 		   *
* Author:                                                                   *
* Date: 														           *
* Revision:															 		   *
*																	 		   *
*******************************************************************************/

#include        "CommonRam.h"
#include				"Temprature.h"
#include				"PARA.h"
#include				"ServoPara.h"

KSD_COMMON_RAM	gCRam;
/*******************************************************************************
* Name: InitCommonRam
* Description:
* Input: 
* Output: 
*
* Author: 
* Date: 
* Revision:
*******************************************************************************/
void SpecialParaForTest(void)
{



#if IS_USER_TEST_DRV
#endif

#if (USER_TYPE == USER_WXHF_GWJX_MOVE)
	gCRam.SvPa.StopMode=1; //Ramp stop mode: 1-stop,2- move~stop~move~;3- stop-move~~
	gPara.SpdPerKmRatio = 993;
#endif	

#if (USER_TYPE == USER_WXHF_GWJX_LIFT)
	gPara.PropDKp = 8192;					/* Propdriver current loop Kp gain 1%~100%--328~32767 */
	gPara.PropDKi = 2048;					/* Propdriver current loop Ki gain 1%~100%--82~8192 */
	gPara.PropDMaxCurrent = 750;	/* Propdriver max current 0~2.0A -- 0~2000 */
	gPara.PropDMinCurrent = 150;	/* Propdriver min current 0~2.0A -- 0~2000 */
	gPara.PropDDitherPeriod = 1;	/* Propdriver dither period 15ms~120ms--1~8 */
	gPara.PropDDitherRatio = 8192;	/* Propdriver dither current ratio.(DitherCurrent/PropDMaxCurrent) 0~100% -- 0~32767 */
	gPara.PropValveResistance = 250;/* Prop valve coil resistance. 1~1000 ohm -- 10~10000 */
#endif	

#if (USER_TYPE == USER_BJHL_PHZC04T_MOVE)
#endif
#if (USER_TYPE == USER_BJHL_PHZC04T_LIFT)
#endif

#if (USER_TYPE == USER_NBRY_SXCC_MOVE)
#endif
#if (USER_TYPE == USER_NBRY_SXCC_LIFT)
#endif

#if (USER_TYPE == USER_NBRY_QYCC15T_MOVE)
#endif	

#if (USER_TYPE == USER_NBRY_QYCC15T_LIFT)
	gPara.BrakePedalType = gPara.ThrottleType;	        //脚踏板速度信号输入类型：0：电压输入；1：2线电阻输入；2；3线电阻输入；3：总线输入
	gPara.BrakePedalMinVoltage = gPara.ThrottleMinVoltage;		//踏板死区最小值
	gPara.BrakePedalMaxVoltage = gPara.ThrottleMaxVoltage;		//踏板死区最大值
	gPara.BrakePedalMap = gPara.ThrottleMap;						//踏板斜率曲线（%）

	gPara.PropDKp = 8192;					/* Propdriver current loop Kp gain 1%~100%--328~32767 */
	gPara.PropDKi = 2048;					/* Propdriver current loop Ki gain 1%~100%--82~8192 */
	gPara.PropDMaxCurrent = 620;	/* Propdriver max current 0~2.0A -- 0~2000 */
	gPara.PropDMinCurrent = 150;	/* Propdriver min current 0~2.0A -- 0~2000 */
	gPara.PropDDitherPeriod = 1;	/* Propdriver dither period 15ms~120ms--1~8 */
	gPara.PropDDitherRatio = 8192;	/* Propdriver dither current ratio.(DitherCurrent/PropDMaxCurrent) 0~100% -- 0~32767 */
	gPara.PropValveResistance = 191;/* Prop valve coil resistance. 1~1000 ohm -- 10~10000 */
#endif

#if (USER_TYPE == USER_NBRY_DDTBC_MOVE)
#endif
#if (USER_TYPE == USER_NUOLI_DDTBC_MOVE)
#endif

#if (  (USER_TYPE == USER_XJTU_DDJCQS_MOVE) \
		|| (USER_TYPE == USER_XJTU_DDJCLD_MOVE) )
#endif

}


/*******************************************************************************
* Name: InitCommonRam
* Description:
* Input: 
* Output: 
*
* Author: 
* Date: 
* Revision:
*******************************************************************************/
void InitCommonRam(void)
{
	INT16U i;
	SpecialParaForTest();
	/* gCRam, input to kernel */
	gCRam.PosCmdSrc = 0;
	gCRam.PosCmd = 0;
	gCRam.PosFdb = 0;
	gCRam.SpeedCmd = 0;
	gCRam.SpeedMode = 0;
	gCRam.BrakeCmd = 0;
	gCRam.SpdCmdSum = 0;
	gCRam.SpeedFdb = 0;
	gCRam.TorqueCmd = 0;
	gCRam.CurrentSampleU = 0;
	gCRam.CurrentSampleW = 0;
	gCRam.RotorElecAngle = 0;
	gCRam.CurrentSampleVBus = 0;
	gCRam.TempratureSample = 0;
	gCRam.PropCurrentCmd = 0;
	gCRam.PropCurrentCmdSum = 0;
	gCRam.PumpSpeedCmd = 0;
	/* gCRam, output from kernel */
	gCRam.PosRef = 0;
	gCRam.PosErr = 0;
	gCRam.SpeedRef = 0;
	gCRam.SpeedFdbDisp = 0;
	gCRam.CurrentFdbIq = 0;
	
	/* gCRam, ErrCode, AlmCode */
	gCRam.ErrCode = 0;
	gCRam.AlmCode = 0;
	
	/* gCRam.SigLamp */
	for(i = 0; i < SIGNAL_LAMP_NUM; i++)
	{
		gCRam.SigLamp[i] = 0;
	}
	
	/* monitor state*/
	gCRam.AcMotorSpeed=0;		  //电机当前转速
	gCRam.AcMotorSpeedF=0;		//电机转速命令
	gCRam.AcPhaseCurrent=0;	  //电机当前相电流
	gCRam.MotorTmp=TMP_DEFAULT;				//电机当前温度
	gCRam.ControllerTmp=TMP_DEFAULT;		//控制器当前温度
	gCRam.VBusVoltage=0;			//当前母线电压
	gCRam.KsiVBusVoltage=0;	  //KSI电压
	gCRam.MoveSpeed=0;				//行驶速度
	gCRam.SteerAngle=0;			  //转向角度
	
	gCRam.TestVoltage=0;
	/* monitor signal*/
	gCRam.ThrottleCmdSpd=0;	  //踏板指令（%）
	gCRam.ThrottleWipVoltage=0;	//踏板滑动端电压值（v）
	gCRam.BrakePedalCmdSpd=0;	//brake 踏板指令（%）
	gCRam.BrakePedalWipVoltage=0;	//brake 踏板滑动端电压值（v）
	
	gCRam.AnologOut=0;				//模拟量输出（v）
	gCRam.AnalogIn1=0;				//模拟量1输入（v）
	gCRam.AnalogIn2=0;				//模拟量2输入（v）
	gCRam.AnalogIn3=0;				//模拟量3输入（v）
	gCRam.AnalogIn4=0;				//模拟量4输入（v）
	gCRam.Switch=0;					  //开关输入
	gCRam.Driver=0;					  //驱动输出
	gCRam.MotorEncoder=0;		  //电机编码器输入
	gCRam.DirectionEncoder1=0;		//方向盘编码器输入1
	gCRam.DirectionEncoder2=0;		//方向盘编码器输入2
	gCRam.KernelState = 0;
	
	gCRam.AutoCentreDirectionLock=0;
	gCRam.AutoCentreDirection=0;
	gCRam.AutoCentreCmd=0;
	gCRam.PosMiddleCmd=0;
	gCRam.PosMiddleFdb=0;
	gCRam.AutoCentrePhase=0;
	gCRam.SteerAngle=0;
	gCRam.AutoCentreCmdSum = 0;


/**********************************************************************/
	gCRam.SvPa.CanEn = gPara.CanEn;
	gCRam.SvPa.CanLockEn = gPara.CanLockEn;
	gCRam.SvPa.ConBit1 = gPara.ConBit1;
	gCRam.SvPa.ConBit2 = gPara.ConBit2;
	gCRam.SvPa.CanHeartBeat = gPara.CanHeartBeat;

	/* motor */
	gCRam.SvPa.MotorType=gPara.MotorType;
	/* motor_constant */
	gCRam.SvPa.AcMotorPloes = gPara.AcMotorPloes;
	gCRam.SvPa.EncoderLineNum = gPara.EncoderLineNum;
	gCRam.SvPa.Inertia = gPara.Inertia;
	gCRam.SvPa.StatorResist = gPara.StatorResist;
	gCRam.SvPa.StatorTimeConstant = gPara.StatorTimeConstant;
	gCRam.SvPa.TorqueCoe1 = gPara.TorqueCoe1;
	gCRam.SvPa.RotorTimeConstant = gPara.RotorTimeConstant;
	gCRam.SvPa.AcMotorFluxRatio = gPara.AcMotorFluxRatio;
	gCRam.SvPa.AcMotorWkBaseSpdF = gPara.AcMotorWkBaseSpdF;
	gCRam.SvPa.AcMotorTypicalSpdF = gPara.AcMotorTypicalSpdF;
	gCRam.SvPa.AcMotorRateSpdF = gPara.AcMotorRateSpdF;
	gCRam.SvPa.AcMotorDeltaSpdF = gPara.AcMotorDeltaSpdF;
	gCRam.SvPa.AcMotorDriveCurrentLimitRatio = gPara.AcMotorDriveCurrentLimitRatio;
	gCRam.SvPa.AcMotorUphillCurrentLimitRatio = gPara.AcMotorUphillCurrentLimitRatio;
	gCRam.SvPa.AcMotorRegenCurrentLimitRatio = gPara.AcMotorRegenCurrentLimitRatio;
	gCRam.SvPa.AcMotorBrakeCurrentLimitRatio = gPara.AcMotorBrakeCurrentLimitRatio;
	gCRam.SvPa.AcMotorEmrCurrentLimitRatio = gPara.AcMotorEmrCurrentLimitRatio;
	gCRam.SvPa.AcMotorDriveMapNominalRatio = gPara.AcMotorDriveMapNominalRatio;
	gCRam.SvPa.AcMotorDriveMapDelta1Ratio = gPara.AcMotorDriveMapDelta1Ratio;
	gCRam.SvPa.AcMotorDriveMapDelta2Ratio = gPara.AcMotorDriveMapDelta2Ratio;
	gCRam.SvPa.AcMotorDriveMapDelta4Ratio = gPara.AcMotorDriveMapDelta4Ratio;
	gCRam.SvPa.AcMotorDriveMapDelta8Ratio = gPara.AcMotorDriveMapDelta8Ratio;
	gCRam.SvPa.AcMotorRegenMapNominalRatio = gPara.AcMotorRegenMapNominalRatio;
	gCRam.SvPa.AcMotorRegenMapDelta1Ratio = gPara.AcMotorRegenMapDelta1Ratio;
	gCRam.SvPa.AcMotorRegenMapDelta2Ratio = gPara.AcMotorRegenMapDelta2Ratio;
	gCRam.SvPa.AcMotorRegenMapDelta4Ratio = gPara.AcMotorRegenMapDelta4Ratio;
	gCRam.SvPa.AcMotorRegenMapDelta8Ratio = gPara.AcMotorRegenMapDelta8Ratio;
	gCRam.SvPa.AcMotorWkPowerRatio = gPara.AcMotorWkPowerRatio;
	gCRam.SvPa.AcMotorWkRate = gPara.AcMotorWkRate;
	gCRam.SvPa.WkAdjRatio = gPara.WkAdjRatio;
	gCRam.SvPa.MotorTmpSensorType=gPara.MotorTmpSensorType;
	gCRam.SvPa.MotorTmpOfs = gPara.MotorTmpOfs;
	gCRam.SvPa.MotorCutTmp=gPara.MotorCutTmp;
	gCRam.SvPa.MotorMaxTmp=gPara.MotorMaxTmp;
	gCRam.SvPa.MotorTmpLosSpd = gPara.MotorTmpLosSpd;

	/* Speed */
	gCRam.SvPa.FwdMaxSpd=gPara.FwdMaxSpd;
	gCRam.SvPa.RvsMaxSpd=gPara.RvsMaxSpd;
	gCRam.SvPa.IntertiaRatio=gPara.IntertiaRatio;
	gCRam.SvPa.SpeedWc=gPara.SpeedWc;
	gCRam.SvPa.SpeedWit=gPara.SpeedWit;
	gCRam.SvPa.TorFilt1W=gPara.TorFilt1W;
	gCRam.SvPa.TorFilt2W=gPara.TorFilt2W;
	gCRam.SvPa.TorFilt2Q=gPara.TorFilt2Q;

	gCRam.SvPa.KvffCurrent = gPara.KvffCurrent;
	gCRam.SvPa.KvffBuildTime = gPara.KvffBuildTime;
	gCRam.SvPa.KvffReleaseTime = gPara.KvffReleaseTime;
	
	gCRam.SvPa.KaccCurrent = gPara.KaccCurrent;
	gCRam.SvPa.KbkeCurrent = gPara.KbkeCurrent;
	gCRam.SvPa.KaccBuildTime = gPara.KaccBuildTime;
	gCRam.SvPa.KaccReleaseTime = gPara.KaccReleaseTime;
	
	gCRam.SvPa.FullAccTimeHs = gPara.FullAccTimeHs;
	gCRam.SvPa.FullAccTimeLs = gPara.FullAccTimeLs;
	gCRam.SvPa.LowAccTime = gPara.LowAccTime;
	gCRam.SvPa.NeutralDecTimeHs = gPara.NeutralDecTimeHs;
	gCRam.SvPa.NeutralDecTimeLs = gPara.NeutralDecTimeLs;
	gCRam.SvPa.FullBrakeDecTimeHs = gPara.FullBrakeDecTimeHs;
	gCRam.SvPa.FullBrakeDecTimeLs = gPara.FullBrakeDecTimeLs;
	gCRam.SvPa.LowBrakeDecTimeHLs = gPara.LowBrakeDecTimeHLs;
	gCRam.SvPa.PartialDecTimeHLs = gPara.PartialDecTimeHLs;
	gCRam.SvPa.HighSpeedRatio = gPara.HighSpeedRatio;
	gCRam.SvPa.LowSpeedRatio = gPara.LowSpeedRatio;
	gCRam.SvPa.SmallSpeedRatio = gPara.SmallSpeedRatio;
	gCRam.SvPa.ReversalSoften = gPara.ReversalSoften;
	gCRam.SvPa.MaxSpdAccTime = gPara.MaxSpdAccTime;
	gCRam.SvPa.MaxSpdDecTime = gPara.MaxSpdDecTime;

	gCRam.SvPa.SpeedRate1=gPara.SpeedRate1;
	gCRam.SvPa.SpeedRate2=gPara.SpeedRate2;
	gCRam.SvPa.SpeedRate3=gPara.SpeedRate3;
	gCRam.SvPa.SpeedRate4=gPara.SpeedRate4;
	/* Restraint */
	//Position Hold Enable  ConBit1:bit3
	gCRam.SvPa.SoftStopSpeed = gPara.SoftStopSpeed;
	gCRam.SvPa.EntryRateStop = gPara.EntryRateStop;
	gCRam.SvPa.SetSpeedThreshold = gPara.SetSpeedThreshold;
	gCRam.SvPa.SetSpeedSettleTime = gPara.SetSpeedSettleTime;
	gCRam.SvPa.StopMode = gPara.StopMode;
	gCRam.SvPa.EntryRateApproach = gPara.EntryRateApproach;

	gCRam.SvPa.SpdAccRatioFast = gPara.SpdAccRatioFast;
	gCRam.SvPa.SpdDecRatioFast = gPara.SpdDecRatioFast;
	gCRam.SvPa.SpdAccRatioSlow = gPara.SpdAccRatioSlow;
	gCRam.SvPa.SpdDecRatioSlow = gPara.SpdDecRatioSlow;
	
	/* Throttle */
	gCRam.SvPa.ThrottleType = gPara.ThrottleType;
	gCRam.SvPa.ThrottleMinVoltage = gPara.ThrottleMinVoltage;
	gCRam.SvPa.ThrottleMaxVoltage = gPara.ThrottleMaxVoltage;
	gCRam.SvPa.ThrottleMap = gPara.ThrottleMap;
	gCRam.SvPa.ThrottleOfs = gPara.ThrottleOfs;

	gCRam.SvPa.ThrottleMinVoltageRvs = gPara.ThrottleMinVoltageRvs;
	gCRam.SvPa.ThrottleMaxVoltageRvs = gPara.ThrottleMaxVoltageRvs;
	gCRam.SvPa.ThrottleOfsRvs = gPara.ThrottleOfsRvs;
	gCRam.SvPa.ThrottleMapRvs = gPara.ThrottleMapRvs;
	gCRam.SvPa.SequenceDelay = gPara.SequenceDelay;
  /* Brake */
	gCRam.SvPa.BrakePedalType=gPara.BrakePedalType;	        //脚踏板速度信号输入类型：0：电压输入；1：2线电阻输入；2；3线电阻输入；3：总线输入
	gCRam.SvPa.BrakePedalMinVoltage=gPara.BrakePedalMinVoltage;		//踏板死区最小值
	gCRam.SvPa.BrakePedalMaxVoltage=gPara.BrakePedalMaxVoltage;		//踏板死区最大值
	gCRam.SvPa.BrakePedalMap=gPara.BrakePedalMap;						//踏板斜率曲线（%）
	gCRam.SvPa.BrakePedalOfs=gPara.BrakePedalOfs;					//initial ofs to increase act

	/* Second Move */

	/* Vehicle */
	//Metric Units ConBit1:bit7
	gCRam.SvPa.SpdPerKmRatio=gPara.SpdPerKmRatio;
	gCRam.SvPa.CapSpd1=gPara.CapSpd1;
	gCRam.SvPa.CapSpd2=gPara.CapSpd2;
	gCRam.SvPa.CapDist1=gPara.CapDist1;
	gCRam.SvPa.CapDist2=gPara.CapDist2;
	gCRam.SvPa.CapDist3=gPara.CapDist3;
	gCRam.SvPa.WeihuPeriod=gPara.WeihuPeriod;
	
	
	/* Driver */
	gCRam.SvPa.DriverEn1=gPara.DriverEn1;
	gCRam.SvPa.DriverEn2=gPara.DriverEn2;
	/* Main Contactor */
	gCRam.SvPa.MainPullVoltage=gPara.MainPullVoltage;
	gCRam.SvPa.MainHoldVoltage=gPara.MainHoldVoltage;
	gCRam.SvPa.MainOpenDelay=gPara.MainOpenDelay;
	gCRam.SvPa.MainDncThreshold=gPara.MainDncThreshold;

	/* Drive3 */
	gCRam.SvPa.Drive3PullVoltage = gPara.Drive3PullVoltage;
	gCRam.SvPa.Drive3HoldVoltage = gPara.Drive3HoldVoltage;
	/* Drive4 */
	gCRam.SvPa.Drive4PullVoltage = gPara.Drive4PullVoltage;
	gCRam.SvPa.Drive4HoldVoltage = gPara.Drive4HoldVoltage;
	
	/* EmBrake */
	gCRam.SvPa.EmBrkType = gPara.EmBrkType;
	gCRam.SvPa.EmBrkPullVoltage = gPara.EmBrkPullVoltage;
	gCRam.SvPa.EmBrkHoldVoltage = gPara.EmBrkHoldVoltage;
	gCRam.SvPa.TrqPreloadDelay = gPara.TrqPreloadDelay;
	gCRam.SvPa.EmBrkReleaseDelay = gPara.EmBrkReleaseDelay;
	gCRam.SvPa.TrqPreloadCancelDelay = gPara.TrqPreloadCancelDelay;
	
	/* Propdriver */
	gCRam.SvPa.PumpDriveCurrentLimitRatio = gPara.PumpDriveCurrentLimitRatio;
	gCRam.SvPa.PumpSpdAccRatio = gPara.PumpSpdAccRatio;
	gCRam.SvPa.PropDKp = gPara.PropDKp;
	gCRam.SvPa.PropDKi = gPara.PropDKi;
	gCRam.SvPa.PropDMaxCurrent = gPara.PropDMaxCurrent;
	gCRam.SvPa.PropDMinCurrent = gPara.PropDMinCurrent;
	gCRam.SvPa.PropDDitherPeriod = gPara.PropDDitherPeriod;
	gCRam.SvPa.PropDDitherRatio = gPara.PropDDitherRatio;
	gCRam.SvPa.PropValveResistance = gPara.PropValveResistance;
	
	/* Emreverse */
	gCRam.SvPa.EmrTimeLimit = gPara.EmrTimeLimit;
	gCRam.SvPa.EmrSpeed = gPara.EmrSpeed;
	gCRam.SvPa.EmrAccTime = gPara.EmrAccTime;
	gCRam.SvPa.EmrDecTime = gPara.EmrDecTime;
	/* Steer */
	gCRam.SvPa.SteerBit = gPara.SteerBit;
	gCRam.SvPa.PosRefFiltT=gPara.PosRefFiltT;
	gCRam.SvPa.PosKp=gPara.PosKp;
	gCRam.SvPa.PosGearNum = gPara.PosGearNum;
	gCRam.SvPa.PosGearDen = gPara.PosGearDen;
	gCRam.SvPa.PosCmdFor90Degree = gPara.PosCmdFor90Degree;
	gCRam.SvPa.PosCmdLimitCw = gPara.PosCmdLimitCw;
	gCRam.SvPa.PosCmdLimitCcw = gPara.PosCmdLimitCcw;
	gCRam.SvPa.MiddleOfs = gPara.MiddleOfs;
	gCRam.SvPa.MiddleDirErr = gPara.MiddleDirErr;

	//Var, Default 90 degree for fxp pulse para  kff remove to posloop
	gCRam.SvPa.KoePosFdb2Angle = (INT64S)((INT64S)_IQ(1.0)*gCRam.SvPa.PosGearDen*90)/(gCRam.SvPa.PosGearNum*gCRam.SvPa.PosCmdFor90Degree);

	/*Hardware*/
	gCRam.SvPa.CurrentULin = gPara.CurrentULin;
	gCRam.SvPa.CurrentVLin = gPara.CurrentVLin;
	gCRam.SvPa.KsiVotageLin = gPara.KsiVotageLin;
	gCRam.SvPa.KsiVotageOfs = gPara.KsiVotageOfs;
	gCRam.SvPa.BusVotageLin = gPara.BusVotageLin;
	gCRam.SvPa.BusVotageOfs = gPara.BusVotageOfs;
	gCRam.SvPa.HardwareRsv = gPara.HardwareRsv;
	gCRam.SvPa.HardwareSum = gPara.HardwareSum;
	/*PassWord*/
	gCRam.SvPa.AppPassWord = gPara.AppPassWord;
	gCRam.SvPa.OemPassWord = gPara.OemPassWord;
	/* Product */
	gCRam.SvPa.UserType = gPara.UserType;

/********************* Read only fix para *********************/
	/* CAN */
	gCRam.SvPa.StationAddr = gPara.StationAddr;
	/* motor */
	gCRam.SvPa.RotorTCoe = gPara.RotorTCoe;
	gCRam.SvPa.CalibAngle = gPara.CalibAngle;
	/* Speed */
	gCRam.SvPa.RunMode=gPara.RunMode;
	gCRam.SvPa.SpdAccT = gPara.SpdAccT;
	gCRam.SvPa.SpdDecT = gPara.SpdDecT;
	gCRam.SvPa.NotchFilt1W=gPara.NotchFilt1W;
	gCRam.SvPa.NotchFilt1Q=gPara.NotchFilt1Q;
	gCRam.SvPa.NotchFilt2W=gPara.NotchFilt2W;
	gCRam.SvPa.NotchFilt2Q=gPara.NotchFilt2Q;
	/* Vehicle */
	gCRam.SvPa.TimetoSpd1 = gPara.TimetoSpd1;
	gCRam.SvPa.TimetoSpd2 = gPara.TimetoSpd2;
	gCRam.SvPa.TimetoDist1 = gPara.TimetoDist1;
	gCRam.SvPa.TimetoDist2 = gPara.TimetoDist2;
	gCRam.SvPa.TimetoDist3 = gPara.TimetoDist3;
	gCRam.SvPa.DcRateVoltage = gPara.DcRateVoltage;
	gCRam.SvPa.BatAlarmRatio = gPara.BatAlarmRatio;
	gCRam.SvPa.BatProtectRatio = gPara.BatProtectRatio;

	/* 内部设定值 */
	gCRam.SvPa.DcMaxVoltage = gPara.DcMaxVoltage;
	gCRam.SvPa.DcBrkLimitVoltage = gPara.DcBrkLimitVoltage;
	gCRam.SvPa.DcCutVoltage = gPara.DcCutVoltage;
	gCRam.SvPa.DcMinVoltage = gPara.DcMinVoltage;
	gCRam.SvPa.DcBroVoltage = gPara.DcBroVoltage;
	gCRam.SvPa.DcEspLowVoltage = gPara.DcEspLowVoltage;
	gCRam.SvPa.CtlRate1HCurrent = gPara.CtlRate1HCurrent;
	gCRam.SvPa.CtlRate2MCurrent = gPara.CtlRate2MCurrent;
	gCRam.SvPa.Out5VMaxVoltage = gPara.Out5VMaxVoltage;
	gCRam.SvPa.Out5VMinVoltage = gPara.Out5VMinVoltage;
	gCRam.SvPa.Out12VMaxVoltage = gPara.Out12VMaxVoltage;
	gCRam.SvPa.Out12VMinVoltage = gPara.Out12VMinVoltage;
	
	gCRam.SvPa.PowerCutTmp = gPara.PowerCutTmp;
	gCRam.SvPa.PowerMaxTmp = gPara.PowerMaxTmp;
	gCRam.SvPa.PowerMinTmp = gPara.PowerMinTmp;

	/* Tune */
	gCRam.SvPa.TuneSpdMax = gPara.TuneSpdMax;
	gCRam.SvPa.TuneR = gPara.TuneR;
	gCRam.SvPa.RefRigid = gPara.RefRigid;

	/* other */
	gCRam.SvPa.ExtBrakeHoldTime = gPara.ExtBrakeHoldTime;
	gCRam.SvPa.TorqueOffset = gPara.TorqueOffset;
	gCRam.SvPa.ObserverEnable= gPara.ObserverEnable;
	gCRam.SvPa.ObserverK= gPara.ObserverK;
	for(i=0;i<10;i++)
	{
		gCRam.SvPa.ErrorTrace[i] = gPara.ErrorTrace[i];
	}	
/**********Ican readable******************************************************/
	/* Product */
	gCRam.SvPa.ProductType = gPara.ProductType;
	gCRam.SvPa.ControllerType = gPara.ControllerType;
	gCRam.SvPa.SoftVersion = gPara.SoftVersion;
	gCRam.SvPa.SoftVerYear = gPara.SoftVerYear;
	gCRam.SvPa.SoftVerMonthDay = gPara.SoftVerMonthDay;
	gCRam.SvPa.HardVersion1 = gPara.HardVersion1;
	gCRam.SvPa.HardVersion2 = gPara.HardVersion2;
	gCRam.SvPa.FrameVersion = gPara.FrameVersion;

	gCRam.SvPa.NullPara = gPara.NullPara;
/*********************************************************************/
	/* motor */
	/* 电机类型 0:电机参数上位机设定; */
	/* 1~n：电机参数程序预先内置 cMotorParaRom[n] */
	if (gCRam.SvPa.MotorType > (sizeof(cMotorParaRom)/sizeof(cMotorParaRom[0])))
	{
		SL_SET(PLC_PARA_OV_LIMIT_ERR);
		gCRam.SvPa.MotorType = 0;
	}
	if (gCRam.SvPa.MotorType != 0) 
	{
		MOTOR_PARA *pcMotorPara = &cMotorParaRom[gCRam.SvPa.MotorType - 1];
		gCRam.SvPa.EncoderLineNum = pcMotorPara->EncoderLineNumM4 >> 2;
		gCRam.SvPa.CalibAngle = pcMotorPara->CalibAngle;
		gCRam.SvPa.AcMotorPloes = pcMotorPara->PolePair;
		gCRam.SvPa.AcMotorTypicalSpdF = _IQrmpy(pcMotorPara->TypicalHZ,STD_SPEEDRPM);
		gCRam.SvPa.AcMotorWkBaseSpdF =_IQmpy(pcMotorPara->WkBaseHZ,STD_SPEEDRPM);
		gCRam.SvPa.AcMotorRateSpdF = _IQmpy(pcMotorPara->RatedHZ,STD_SPEEDRPM);
		gCRam.SvPa.AcMotorDeltaSpdF = _IQmpy(pcMotorPara->DeltaHZ,STD_SPEEDRPM);
		gCRam.SvPa.AcMotorDriveCurrentLimitRatio = _IQmpy(_IQdiv(pcMotorPara->DriveCurrent,_IQ(RATE_2M_CURRENT /STD_CURRENT)),STD_PERCENT);
		gCRam.SvPa.AcMotorUphillCurrentLimitRatio = _IQmpy(_IQdiv(pcMotorPara->UphillCurrent,_IQ(RATE_2M_CURRENT	/STD_CURRENT)),STD_PERCENT);
		gCRam.SvPa.AcMotorRegenCurrentLimitRatio = _IQmpy(_IQdiv(pcMotorPara->RegenCurrent,_IQ(RATE_2M_CURRENT /STD_CURRENT)),STD_PERCENT);
		gCRam.SvPa.AcMotorBrakeCurrentLimitRatio = _IQmpy(_IQdiv(pcMotorPara->BrakeCurrent,_IQ(RATE_2M_CURRENT /STD_CURRENT)),STD_PERCENT);
		gCRam.SvPa.AcMotorFluxRatio = _IQmpy(_IQdiv(pcMotorPara->DriveFlux,pcMotorPara->DriveCurrent),STD_PERCENT);
		gCRam.SvPa.AcMotorDriveMapNominalRatio = _IQmpy(pcMotorPara->DriveMapNominalRatio, STD_PERCENT);	
		gCRam.SvPa.AcMotorDriveMapDelta1Ratio = _IQmpy(pcMotorPara->DriveMapDelta1Ratio, STD_PERCENT);	
		gCRam.SvPa.AcMotorDriveMapDelta2Ratio = _IQmpy(pcMotorPara->DriveMapDelta2Ratio, STD_PERCENT);	
		gCRam.SvPa.AcMotorDriveMapDelta4Ratio = _IQmpy(pcMotorPara->DriveMapDelta4Ratio, STD_PERCENT);	
		gCRam.SvPa.AcMotorDriveMapDelta8Ratio = _IQmpy(pcMotorPara->DriveMapDelta8Ratio, STD_PERCENT);	
		gCRam.SvPa.AcMotorRegenMapNominalRatio = _IQmpy(pcMotorPara->RegenMapNominalRatio, STD_PERCENT);	
		gCRam.SvPa.AcMotorRegenMapDelta1Ratio = _IQmpy(pcMotorPara->RegenMapDelta1Ratio, STD_PERCENT);	
		gCRam.SvPa.AcMotorRegenMapDelta2Ratio = _IQmpy(pcMotorPara->RegenMapDelta2Ratio, STD_PERCENT);	
		gCRam.SvPa.AcMotorRegenMapDelta4Ratio = _IQmpy(pcMotorPara->RegenMapDelta4Ratio, STD_PERCENT);	
		gCRam.SvPa.AcMotorRegenMapDelta8Ratio = _IQmpy(pcMotorPara->RegenMapDelta8Ratio, STD_PERCENT);	
		gCRam.SvPa.AcMotorWkPowerRatio = _IQmpy(pcMotorPara->WkPowerRatio, STD_PERCENT);	
		gCRam.SvPa.AcMotorWkRate = _IQmpy(pcMotorPara->WkRate, STD_PERCENT);	
		gCRam.SvPa.WkAdjRatio = _IQmpy(pcMotorPara->WkAdjRatio, STD_PERCENT);
		gCRam.SvPa.RotorTCoe = _IQmpy(pcMotorPara->RotorTCoe, 100000);
		gCRam.SvPa.TorqueCoe1 =_IQmpy(pcMotorPara->KTorque,1000.0*STD_TORQUE_K);
		gCRam.SvPa.Inertia = _IQmpy(pcMotorPara->Inertia,10000.0*STD_INERTIA);
		gCRam.SvPa.StatorResist = _IQmpy(pcMotorPara->Resist,10000.0*STD_RESIST);
		gCRam.SvPa.StatorTimeConstant = _IQmpy(pcMotorPara->Tstator,10000.0*STD_T);
		gCRam.SvPa.RotorTimeConstant = _IQmpy(pcMotorPara->Trotor,10000.0*STD_T);
		gCRam.SvPa.MotorPara = *pcMotorPara;
		
		gPara.EncoderLineNum = gCRam.SvPa.EncoderLineNum;
		gPara.AcMotorPloes = gCRam.SvPa.AcMotorPloes;
		gPara.AcMotorTypicalSpdF = gCRam.SvPa.AcMotorTypicalSpdF;
		gPara.AcMotorWkBaseSpdF = gCRam.SvPa.AcMotorWkBaseSpdF;
		gPara.AcMotorRateSpdF = gCRam.SvPa.AcMotorRateSpdF;
		gPara.AcMotorDeltaSpdF = gCRam.SvPa.AcMotorDeltaSpdF;
		gPara.AcMotorDriveCurrentLimitRatio = gCRam.SvPa.AcMotorDriveCurrentLimitRatio;
		gPara.AcMotorUphillCurrentLimitRatio = gCRam.SvPa.AcMotorUphillCurrentLimitRatio;
		gPara.AcMotorRegenCurrentLimitRatio = gCRam.SvPa.AcMotorRegenCurrentLimitRatio;
		gPara.AcMotorBrakeCurrentLimitRatio = gCRam.SvPa.AcMotorBrakeCurrentLimitRatio;
		gPara.AcMotorFluxRatio = gCRam.SvPa.AcMotorFluxRatio;
		gPara.AcMotorDriveMapNominalRatio = gCRam.SvPa.AcMotorDriveMapNominalRatio;
		gPara.AcMotorDriveMapDelta1Ratio = gCRam.SvPa.AcMotorDriveMapDelta1Ratio;
		gPara.AcMotorDriveMapDelta2Ratio = gCRam.SvPa.AcMotorDriveMapDelta2Ratio;
		gPara.AcMotorDriveMapDelta4Ratio = gCRam.SvPa.AcMotorDriveMapDelta4Ratio;
		gPara.AcMotorDriveMapDelta8Ratio = gCRam.SvPa.AcMotorDriveMapDelta8Ratio;
		gPara.AcMotorRegenMapNominalRatio = gCRam.SvPa.AcMotorRegenMapNominalRatio;
		gPara.AcMotorRegenMapDelta1Ratio = gCRam.SvPa.AcMotorRegenMapDelta1Ratio;
		gPara.AcMotorRegenMapDelta2Ratio = gCRam.SvPa.AcMotorRegenMapDelta2Ratio;
		gPara.AcMotorRegenMapDelta4Ratio = gCRam.SvPa.AcMotorRegenMapDelta4Ratio;
		gPara.AcMotorRegenMapDelta8Ratio = gCRam.SvPa.AcMotorRegenMapDelta8Ratio;
		gPara.AcMotorWkPowerRatio = gCRam.SvPa.AcMotorWkPowerRatio;
		gPara.AcMotorWkRate = gCRam.SvPa.AcMotorWkRate;
		gPara.WkAdjRatio = gCRam.SvPa.WkAdjRatio;
		gPara.RotorTCoe = gCRam.SvPa.RotorTCoe;
		gPara.TorqueCoe1 = gCRam.SvPa.TorqueCoe1;
		gPara.Inertia = gCRam.SvPa.Inertia;
		gPara.StatorResist = gCRam.SvPa.StatorResist;
		gPara.StatorTimeConstant = gCRam.SvPa.StatorTimeConstant;
		gPara.RotorTimeConstant = gCRam.SvPa.RotorTimeConstant;
	}
	else //gCRam.SvPa.MotorType==0
	{
		MOTOR_PARA *pMotorPara = &gCRam.SvPa.MotorPara;

		pMotorPara->Type = gCRam.SvPa.MotorType;
		pMotorPara->EncoderLineNumM4 = gCRam.SvPa.EncoderLineNum*4;
		pMotorPara->CalibAngle = gCRam.SvPa.CalibAngle;
		pMotorPara->PolePair = gCRam.SvPa.AcMotorPloes;
		pMotorPara->TypicalHZ = (_iq)gCRam.SvPa.AcMotorTypicalSpdF * _IQ(1.0/STD_SPEEDRPM);
		pMotorPara->RatedHZ = (_iq)gCRam.SvPa.AcMotorRateSpdF *  _IQ(1.0/STD_SPEEDRPM);
		pMotorPara->WkBaseHZ = (_iq)gCRam.SvPa.AcMotorWkBaseSpdF *  _IQ(1.0/STD_SPEEDRPM);
		pMotorPara->DeltaHZ =  (_iq)gCRam.SvPa.AcMotorDeltaSpdF *  _IQ(1.0/STD_SPEEDRPM);
		//pMotorPara->DriveCurrent = (_iq)gCRam.SvPa.AcMotorDriveCurrentLimitRatio * _IQ(RATE_2M_CURRENT*SQRT2*PH3_TO_PH2 / (STD_CURRENT*STD_PERCENT));
		pMotorPara->DriveCurrent = (_iq)gCRam.SvPa.AcMotorDriveCurrentLimitRatio * _IQ(RATE_2M_CURRENT / (STD_CURRENT*STD_PERCENT));//(for DC motor)
		pMotorPara->RatedCurrent = (_iq)gCRam.SvPa.AcMotorDriveCurrentLimitRatio * _IQ(RATE_2M_CURRENT / (STD_CURRENT*STD_PERCENT));//(for DC motor)
		//pMotorPara->UphillCurrent = (_iq)gCRam.SvPa.AcMotorUphillCurrentLimitRatio * _IQ(RATE_2M_CURRENT*SQRT2*PH3_TO_PH2 / (STD_CURRENT*STD_PERCENT));
		pMotorPara->UphillCurrent = (_iq)gCRam.SvPa.AcMotorUphillCurrentLimitRatio * _IQ(RATE_2M_CURRENT / (STD_CURRENT*STD_PERCENT));//(for DC motor)
		pMotorPara->RegenCurrent = (_iq)gCRam.SvPa.AcMotorRegenCurrentLimitRatio * _IQ(RATE_2M_CURRENT*SQRT2*PH3_TO_PH2 / (STD_CURRENT*STD_PERCENT));
		pMotorPara->BrakeCurrent = (_iq)gCRam.SvPa.AcMotorBrakeCurrentLimitRatio * _IQ(RATE_2M_CURRENT*SQRT2*PH3_TO_PH2 / (STD_CURRENT*STD_PERCENT));
		pMotorPara->DriveFlux = _IQmpy(pMotorPara->DriveCurrent, (_iq)gCRam.SvPa.AcMotorFluxRatio * _IQ(1.0/STD_PERCENT));
		pMotorPara->UphillFlux = _IQmpy(pMotorPara->UphillCurrent, (_iq)gCRam.SvPa.AcMotorFluxRatio * _IQ(1.0/STD_PERCENT));
		pMotorPara->RegenFlux = _IQmpy(pMotorPara->RegenCurrent, (_iq)gCRam.SvPa.AcMotorFluxRatio * _IQ(1.0/STD_PERCENT));
		pMotorPara->BrakeFlux = _IQmpy(pMotorPara->BrakeCurrent, (_iq)gCRam.SvPa.AcMotorFluxRatio * _IQ(1.0/STD_PERCENT));
		pMotorPara->DriveMapNominalRatio = (_iq)gCRam.SvPa.AcMotorDriveMapNominalRatio * _IQ(1.0/STD_PERCENT);
		pMotorPara->DriveMapDelta1Ratio =  (_iq)gCRam.SvPa.AcMotorDriveMapDelta1Ratio * _IQ(1.0/STD_PERCENT);
		pMotorPara->DriveMapDelta2Ratio = (_iq)gCRam.SvPa.AcMotorDriveMapDelta2Ratio * _IQ(1.0/STD_PERCENT);
		pMotorPara->DriveMapDelta4Ratio = (_iq)gCRam.SvPa.AcMotorDriveMapDelta4Ratio * _IQ(1.0/STD_PERCENT);
		pMotorPara->DriveMapDelta8Ratio = (_iq)gCRam.SvPa.AcMotorDriveMapDelta8Ratio * _IQ(1.0/STD_PERCENT);
		pMotorPara->RegenMapNominalRatio = (_iq)gCRam.SvPa.AcMotorRegenMapNominalRatio * _IQ(1.0/STD_PERCENT);
		pMotorPara->RegenMapDelta1Ratio = (_iq)gCRam.SvPa.AcMotorRegenMapDelta1Ratio * _IQ(1.0/STD_PERCENT);
		pMotorPara->RegenMapDelta2Ratio = (_iq)gCRam.SvPa.AcMotorRegenMapDelta2Ratio * _IQ(1.0/STD_PERCENT);
		pMotorPara->RegenMapDelta4Ratio = (_iq)gCRam.SvPa.AcMotorRegenMapDelta4Ratio * _IQ(1.0/STD_PERCENT);
		pMotorPara->RegenMapDelta8Ratio = (_iq)gCRam.SvPa.AcMotorRegenMapDelta8Ratio * _IQ(1.0/STD_PERCENT);
		pMotorPara->WkPowerRatio = (_iq)gCRam.SvPa.AcMotorWkPowerRatio * _IQ(1.0/STD_PERCENT);
		pMotorPara->WkRate = (_iq)gCRam.SvPa.AcMotorWkRate * _IQ(1.0/STD_PERCENT);
		pMotorPara->WkAdjRatio = (_iq)gCRam.SvPa.WkAdjRatio * _IQ(1.0/STD_PERCENT);
		pMotorPara->RotorTCoe = (_iq)gCRam.SvPa.RotorTCoe * _IQ(1.0/100000);
		pMotorPara->KTorque = (_iq)gCRam.SvPa.TorqueCoe1 * _IQ(1.0/(1000.0*SQRT2*PH3_TO_PH2*STD_TORQUE_K));
		pMotorPara->Inertia = (_iq)gCRam.SvPa.Inertia * _IQ(1.0/(10000.0*STD_INERTIA));
		pMotorPara->Resist = (_iq)gCRam.SvPa.StatorResist * _IQ(1.0/(10000.0*STD_RESIST));
		pMotorPara->Tstator = (_iq)gCRam.SvPa.StatorTimeConstant * _IQ(1.0/(10000.0*STD_T));
		pMotorPara->Trotor = (_iq)gCRam.SvPa.RotorTimeConstant * _IQ(1.0/(10000.0*STD_T));
		pMotorPara->MinSpeed = _IQ16(gCRam.SvPa.SetSpeedThreshold);
	}
	{
		MOTOR_PARA *pMotorPara = &gCRam.SvPa.MotorPara;
		
		pMotorPara->RatedHZ1 = pMotorPara->RatedHZ + pMotorPara->DeltaHZ;
		pMotorPara->RatedHZ2 = pMotorPara->RatedHZ + (pMotorPara->DeltaHZ * 2);
		pMotorPara->RatedHZ4 = pMotorPara->RatedHZ + (pMotorPara->DeltaHZ * 4);
		pMotorPara->RatedHZ8 = pMotorPara->RatedHZ + (pMotorPara->DeltaHZ * 8);
		pMotorPara->DrvKf2iqRatio1 = _IQdiv((pMotorPara->DriveMapDelta1Ratio - pMotorPara->DriveMapNominalRatio), (pMotorPara->RatedHZ1 - pMotorPara->RatedHZ));
		pMotorPara->DrvKf2iqRatio2 = _IQdiv((pMotorPara->DriveMapDelta2Ratio - pMotorPara->DriveMapDelta1Ratio), (pMotorPara->RatedHZ2 - pMotorPara->RatedHZ1));
		pMotorPara->DrvKf2iqRatio4 = _IQdiv((pMotorPara->DriveMapDelta4Ratio - pMotorPara->DriveMapDelta2Ratio), (pMotorPara->RatedHZ4 - pMotorPara->RatedHZ2));
		pMotorPara->DrvKf2iqRatio8 = _IQdiv((pMotorPara->DriveMapDelta8Ratio - pMotorPara->DriveMapDelta4Ratio), (pMotorPara->RatedHZ8 - pMotorPara->RatedHZ4));
		
		pMotorPara->RegenKf2iqRatio1 = _IQdiv((pMotorPara->RegenMapDelta1Ratio - pMotorPara->RegenMapNominalRatio), (pMotorPara->RatedHZ1 - pMotorPara->RatedHZ));
		pMotorPara->RegenKf2iqRatio2 = _IQdiv((pMotorPara->RegenMapDelta2Ratio - pMotorPara->RegenMapDelta1Ratio), (pMotorPara->RatedHZ2 - pMotorPara->RatedHZ1));
		pMotorPara->RegenKf2iqRatio4 = _IQdiv((pMotorPara->RegenMapDelta4Ratio - pMotorPara->RegenMapDelta2Ratio), (pMotorPara->RatedHZ4 - pMotorPara->RatedHZ2));
		pMotorPara->RegenKf2iqRatio8 = _IQdiv((pMotorPara->RegenMapDelta8Ratio - pMotorPara->RegenMapDelta4Ratio), (pMotorPara->RatedHZ8 - pMotorPara->RatedHZ4));
		pMotorPara->UphillRatioKoe = _IQdiv((_IQdiv(pMotorPara->UphillCurrent, pMotorPara->DriveCurrent) - _IQ(1.0)), (pMotorPara->RatedHZ>>2));
		pMotorPara->WkAdjRatioKoe = _IQdiv((pMotorPara->WkAdjRatio - _IQ(1.0)), (pMotorPara->TypicalHZ - pMotorPara->RatedHZ));
		pMotorPara->RotorTCoeK = _IQ(1.0) + pMotorPara->RotorTCoe * (20 - 20);
	}
}
