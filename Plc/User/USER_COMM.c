/*******************************************************************************
* Filename: USER_COMM.c	                                             	 		   *
* Description: 用户通用程序										   			 		   *
* Author:                                                           		       *
* Date:     														 		   													       *
* Revision:															 		   
*******************************************************************************/

#include  "USER_COMM.h"
#include	"PLCHardware.h"
#include	"CommonRam.h"
#include	"Speed.h"
#include	"FDB.h"
//#include	"iCAN.h"
#include	"iTimer.h"
#include	"Queue.h"
#include	"PARA.h"
#include	"PropDriver.h"
#include	"Current.h"
//
//ICAN通用监控参数设置
//
#if 0
#define LOW_POT_FLAG								(1<<0)
#define THROTTLE_HIGH_POT_FLAG			(1<<1)
#define BRAKE_HIGH_POT_FLAG					(1<<2)
void ICANParaConfig(void)
{
	static INT16U errorDelay = 0;
	static INT16U PotDelay = 0;
	static INT16U ThrottleDelay = 0;
	static INT16U BrakeDelay = 0;
	INT16U PotFlag;
	INT16U ThrottleMax, BrakeMax;
	INT16S CutoffPercent;
	_iq DriveCurrentCutoff;
		
	CanOpenUpdatePlcDataFromRxbuf();	
	CanOpenRxFrameMonitor();
	
	//
	//设定监控数据
	//
	gPara.MotorTmp=gPLCCtl.TmpMotor;				//电机当前温度
	gPara.ControllerTmp=gPLCCtl.TmpPower;		//控制器当前温度
	gPara.VBusVoltage=gPLCCtl.VBus;			  //当前母线电压
	gPara.KsiVBusVoltage=gPLCCtl.KsiVBus;	//KSI电压
	gPara.KernelState= gCRam.KernelState; //控制环状态监测
	
	if (gCRam.ErrCode != 0)
		gPara.ErrCode = gCRam.ErrCode;
	else 
		gPara.ErrCode = gCRam.AlmCode;
//Test	
	if(gPLCCtl.VBus>gCRam.TestVoltage)
	{
		gCRam.TestVoltage=gPLCCtl.VBus;
	}
	if (errorDelay < 800)
	{
		gPara.VBusVoltage=gPLCCtl.VBus;
		errorDelay++;
	}
	else if (errorDelay < 1200)
	{
		gPara.VBusVoltage=gCRam.TestVoltage;
		errorDelay++;
	}
	else
	{
		errorDelay = 0;
	}
//Test
	gPara.ThrottleCmdSpd=gPLCCtl.throttleOutput;	//踏板指令（%）	
	gPara.BrakePedalCmdSpd=gPLCCtl.SidleOutput;		//油泵指令（%）

	PotFlag = 0;
	gPara.PotLowVoltage=((INT32S)gPLCCtl.aiDataIn[AD_PotLow]*66)/4096;	//电位计低端电压值（0.1v）
	//Throttle踏板
	gPara.ThrottleHighVoltage=((INT32S)gPLCCtl.aiDataIn[AD_ThrotPotHigh]*66)/4096;	//电位计高端电压值（0.1v）
	if(gCRam.SvPa.ThrottleType==THROTTLE_VOLTAGE5V) //电压输入
	{
#if ((CTLBOARD_TYPE ==_1226)||(CTLBOARD_TYPE ==_1226_GD32))
		gPara.ThrottleWipVoltage=((INT32S)gPLCCtl.aiDataIn[AD_ThrotPotWip]*66)/4096;	//踏板滑动端电压值（0.1v）
#endif //#if (CTLBOARD_TYPE ==_1226)
		ThrottleMax = THROTTLE_5V_VIN_MAX * 10;
	}
	else if(gCRam.SvPa.ThrottleType==THROTTLE_RESISTANCE_2WIRE) //2线电阻输入
	{
		gPara.ThrottleWipVoltage=((INT32S)gPLCCtl.aiDataIn[AD_ThrotPotWip]*66)/4096;	//踏板滑动端电压值（0.1v）
		ThrottleMax = THROTTLE_POT2_5K_VIN_MAX * 10;
		PotFlag |= LOW_POT_FLAG;
	}
	else if(gCRam.SvPa.ThrottleType==THROTTLE_RESISTANCE_3WIRE) //3线电阻输入
	{
		gPara.ThrottleWipVoltage=((INT32S)gPLCCtl.aiDataIn[AD_ThrotPotWip]*66)/4096;	//踏板滑动端电压值（0.1v）
		ThrottleMax = THROTTLE_POT3_VIN_MAX * 10;
		PotFlag |= LOW_POT_FLAG | THROTTLE_HIGH_POT_FLAG;
	}
	else if(gCRam.SvPa.ThrottleType==THROTTLE_BUS)
	{
		INT16S ad12abs;
		ad12abs = 0;
#if (USER_TYPE == USER_NUOLI_DDTBC_MOVE)
		if ((ad12abs = gPLCCtl.canopenRemaHandle.MoveThrottleAd13S) < 0)
			ad12abs = -ad12abs;
#endif	
		ad12abs = gPLCCtl.ThrottleWipVoltageBus;		
		gPara.ThrottleWipVoltage = (ad12abs*100)/4096;
		ThrottleMax = THROTTLE_BUS_VIN_MAX * 10;
	}
	else if (gCRam.SvPa.ThrottleType==THROTTLE_VOLTAGE10V)
	{
		gPara.ThrottleWipVoltage=gPara.Analog1;	//踏板滑动端电压值（0.1v）
		ThrottleMax = THROTTLE_10V_VIN_MAX * 10;
	}
	else
	{
		gPara.ThrottleWipVoltage=0;
		ThrottleMax = THROTTLE_5V_VIN_MAX * 10;
	}
	//Brake pedal 踏板
#if ((HARDVERSION1 & 0xff) <= 3)
	gPara.BrakePedalHighVoltage=((INT32S)gPLCCtl.aiDataIn[AD_SpdPotHigh]*66)/4096;	//电位计高端电压值（0.1v）;
	if(gCRam.SvPa.BrakePedalType==THROTTLE_VOLTAGE5V) //电压输入
	{
		gPara.BrakePedalWipVoltage=((INT32S)gPLCCtl.aiDataIn[AD_SpdPotWip]*66)/4096;	//踏板滑动端电压值（0.1v）
		BrakeMax = THROTTLE_5V_VIN_MAX * 10;
	}
	else if(gCRam.SvPa.BrakePedalType==THROTTLE_RESISTANCE_2WIRE) //2线电阻输入
	{
		gPara.BrakePedalWipVoltage=((INT32S)gPLCCtl.aiDataIn[AD_SpdPotWip]*66)/4096;	//踏板滑动端电压值（0.1v）
		BrakeMax = THROTTLE_POT2_5K_VIN_MAX * 10;
		PotFlag |= LOW_POT_FLAG;
	}
	else if(gCRam.SvPa.BrakePedalType==THROTTLE_RESISTANCE_3WIRE) //3线电阻输入
	{
		gPara.BrakePedalWipVoltage=((INT32S)gPLCCtl.aiDataIn[AD_SpdPotWip]*66)/4096;	//踏板滑动端电压值（0.1v）
		BrakeMax = THROTTLE_POT3_VIN_MAX * 10;
		PotFlag |= LOW_POT_FLAG | BRAKE_HIGH_POT_FLAG;
	}
	else if(gCRam.SvPa.BrakePedalType==THROTTLE_BUS)
	{
		gPara.BrakePedalWipVoltage = 0;
		BrakeMax = THROTTLE_BUS_VIN_MAX * 10;
	}
	else if (gCRam.SvPa.BrakePedalType==THROTTLE_VOLTAGE10V)
	{
		gPara.BrakePedalWipVoltage=gPara.Analog1;	//踏板滑动端电压值（0.1v）
		BrakeMax = THROTTLE_10V_VIN_MAX * 10;
	}
	else
	{
		gPara.BrakePedalWipVoltage=((INT32S)gPLCCtl.aiDataIn[AD_SpdPotWip]*66)/4096;	//踏板滑动端电压值（0.1v）
		BrakeMax = gPara.BrakePedalWipVoltage + 5;
	}
#endif //#if ((HARDVERSION1 & 0xff) <= 2)
	
	//POT check
	if ((PotFlag & (LOW_POT_FLAG | THROTTLE_HIGH_POT_FLAG | BRAKE_HIGH_POT_FLAG)) != 0)
	{
		if ((PotFlag & LOW_POT_FLAG) != 0)
		{
			if (gPara.PotLowVoltage >= (POT_LOW_MAX*10))
			{
				if (PotDelay >= 200)
				{
					SL_SET(PLC_POT_SHORTCIRCUITS_ERR);
				}
				else
					PotDelay++;
			}
		}
		
		if ((PotFlag & THROTTLE_HIGH_POT_FLAG) != 0)
		{
			if (gPara.ThrottleHighVoltage <= (POT_HIGH_MIN*10))
			{
				if (PotDelay >= 200)
				{
					SL_SET(PLC_POT_SHORTCIRCUITS_ERR);
				}
				else
					PotDelay++;
			}
		}
#if ((HARDVERSION1 & 0xff) <= 2)
		if ((PotFlag & BRAKE_HIGH_POT_FLAG) != 0)
		{
			if (gPara.BrakePedalHighVoltage <= (POT_HIGH_MIN*10))
			{
				if (PotDelay >= 200)
				{
					SL_SET(PLC_POT_SHORTCIRCUITS_ERR);
				}
				else
					PotDelay++;
			}
		}		
#endif //((HARDVERSION1 & 0xff) <= 2)
	}
	else
	{
		if (PotDelay > 0)
			PotDelay--;
	}
	//Throttle Voltage check
	if (gCRam.SvPa.ThrottleType != THROTTLE_NONE)
	{
		if (gPara.ThrottleWipVoltage >= ThrottleMax)
		{
			if (ThrottleDelay >= 200)
			{
				SL_SET(PLC_THROTTLE_OVER_ERR);
			}
			else
			{
				ThrottleDelay++;
			}
		}
		else
		{
			if (ThrottleDelay > 0)
				ThrottleDelay--;
		}
	}
	//Brake Voltage check 
#if ((HARDVERSION1 & 0xff) <= 3)
	if (gCRam.SvPa.BrakePedalType != THROTTLE_NONE)
	{
		if (gPara.BrakePedalWipVoltage >= BrakeMax)
		{
			if (BrakeDelay > 200)
			{
				SL_SET(PLC_BRAKE_OVER_ERR);
			}
			else
			{
				BrakeDelay++;
			}
		}
		else
		{
			if (BrakeDelay > 0)
				BrakeDelay--;
		}
	}
#endif //#if ((HARDVERSION1 & 0xff) <= 2)	
		if(gCurrentLoop.W<0)
			gPara.Analog1 = _IQrmpy(-gCurrentLoop.W,STD_CURRENT);
		else
			gPara.Analog1 = _IQrmpy(gCurrentLoop.W,STD_CURRENT);
		
		if(gCurrentLoop.U<0)
			gPara.Analog2 = _IQrmpy(-gCurrentLoop.U,STD_CURRENT);
		else			
			gPara.Analog2 = _IQrmpy(gCurrentLoop.U,STD_CURRENT);
		
		gPara.Analog3 = _IQrmpy(gPropCurrentLoop.IFdb,PROPD_STD_CURRENT);//0;
		gPara.Analog4 = 0;
		
		if(gSpeedLoop.Filter.Out64ms < gCRam.SvPa.AcMotorRateSpdF)
		{	//小于额定转速，恒扭矩输出
			if((gCRam.SvPa.MotorPara.RatedCurrent - gCRam.SvPa.MotorPara.UphillCurrent) > _IQ(0.005))
				gCRam.SvPa.MotorPara.RatedCurrent -= _IQ(0.005);
			else if((gCRam.SvPa.MotorPara.UphillCurrent - gCRam.SvPa.MotorPara.RatedCurrent) > _IQ(0.005))
				gCRam.SvPa.MotorPara.RatedCurrent += _IQ(0.005);
			else
				gCRam.SvPa.MotorPara.RatedCurrent = gCRam.SvPa.MotorPara.UphillCurrent;
		}
		else	//大于额定转速，恒功率输出
		{		
			if(gCRam.SvPa.MotorPara.RatedCurrent>gCRam.SvPa.MotorPara.DriveCurrent)
			{
				gCRam.SvPa.MotorPara.RatedCurrent -= _IQ(0.005);
			}
			else
			{			
				CutoffPercent = _IQ8div((gCRam.SvPa.AcMotorTypicalSpdF + gCRam.SvPa.AcMotorRateSpdF - gSpeedLoop.Filter.Out64ms), \
																				gCRam.SvPa.AcMotorTypicalSpdF);
	//			DriveCurrentCutoff = _IQmpy(_IQtoIQ16(gCRam.SvPa.MotorPara.DriveCurrent), CutoffPercent);
				DriveCurrentCutoff = (gCRam.SvPa.MotorPara.DriveCurrent*CutoffPercent)>>8;
				if((gCRam.SvPa.MotorPara.RatedCurrent - DriveCurrentCutoff) > _IQ(0.005))
					gCRam.SvPa.MotorPara.RatedCurrent -= _IQ(0.005);
				else if((DriveCurrentCutoff - gCRam.SvPa.MotorPara.RatedCurrent) > _IQ(0.005))
					gCRam.SvPa.MotorPara.RatedCurrent += _IQ(0.005);
				else
					gCRam.SvPa.MotorPara.RatedCurrent = DriveCurrentCutoff;
			}			
		}
		
#if ((CTLBOARD_TYPE ==_1226)||(CTLBOARD_TYPE ==_1226_GD32))
	gPara.Switch=( (gPLCCtl.diDataIn.ucIn[SWI1_R]<<0)
										|(gPLCCtl.diDataIn.ucIn[SWI2_R]<<1)
										|(gPLCCtl.diDataIn.ucIn[SWI3_R]<<2)
										|(gPLCCtl.diDataIn.ucIn[SWI4_R]<<3)
										|(gPLCCtl.diDataIn.ucIn[SWI5_R]<<4)
										|(gPLCCtl.diDataIn.ucIn[SWI6_R]<<5)
										|(gPLCCtl.diDataIn.ucIn[SWI7_R]<<6)
										|(gPLCCtl.diDataIn.ucIn[SWI8_R]<<7)
										|(gPLCCtl.diDataIn.ucIn[SWI9_R]<<8)
										|(gPLCCtl.diDataIn.ucIn[SWI10_R]<<9));
										
	gPara.Driver = ((gPLCCtl.doConnect[0] << 0)
								 |(gPLCCtl.doConnect[1] << 1) 
								 |(gPLCCtl.doConnect[2] << 2)
								 |(gPLCCtl.doConnect[3] << 3)
								 |((gPLCCtl.doDataOut[DRIVER1]^1) << 4)
								 |((gPLCCtl.doDataOut[DRIVER2]^1) << 5)
								 |((gPLCCtl.doDataOut[DRIVER3]^1) << 6)
								 |((gPLCCtl.doDataOut[DRIVER4]^1) << 7));
#endif //#if (CTLBOARD_TYPE ==_1226)

	CanOpenUpdateTxbufFromPlcData();
	CanOpenTxFrame();
	
}
/* The table of sin(x)*1024       */
/* x= 0,1,2,... 90   unit: ° degree */
//static const INT16U SIN0_90_1024_TAB[] = 
//{
//0,18,36,54,71,89,107,125,143,160,178,195,213,230,248,265,282,299,316,333,350,367,384,400,416,433,449,465,481,496,512,527,543,558,573,587,602,616,630,644,658,672,685,698,711,724,737,749,761,773,784,796,807,818,828,839,849,859,868,878,887,896,904,912,920,928,935,943,949,956,962,968,974,979,984,989,994,998,1002,1005,1008,1011,1014,1016,1018,1020,1022,1023,1023,1024,1024
//};
/* The table of (1/sin(x)) * 1024       */
/* x= 0,1,2,... 90   unit: ° degree */
static const INT16U SIN0_90_RECIPROCAL_1024_TAB[] = 
{
65535,58674,29341,19566,14680,11749,9796,8402,7358,6546,5897,5367,4925,4552,4233,3956,3715,3502,3314,3145,2994,2857,2734,2621,2518,2423,2336,2256,2181,2112,2048,1988,1932,1880,1831,1785,1742,1702,1663,1627,1593,1561,1530,1501,1474,1448,1424,1400,1378,1357,1337,1318,1299,1282,1266,1250,1235,1221,1207,1195,1182,1171,1160,1149,1139,1130,1121,1112,1104,1097,1090,1083,1077,1071,1065,1060,1055,1051,1047,1043,1040,1037,1034,1032,1030,1028,1027,1025,1025,1024,1024
};
/*Function: Upon the value of steer angle ,limit the rotation speed below 30% */
/*Input:                                                                  */
/*    SteerAngle: Angle from steer. Unit: Degree. Range: 0~180            */
/*    ThrottleRatio: Current throttle ratio. Unit: 1%. Range: 0~100       */
/*Output: Throttle ration under limition                                  */
//90 degree -- 30%
#define RATIOLIMIT_FOR90DEGREE   30 
INT16U SpeedLimitBySteerAngle(INT32U SteerAngle, INT32U ThrottleRatio)
{
	INT32U ThrottleRatioLimit;
	
	if (SteerAngle <= 180)
	{
		if (SteerAngle > 90)
			SteerAngle = 180 - SteerAngle;
		ThrottleRatioLimit = (RATIOLIMIT_FOR90DEGREE * SIN0_90_RECIPROCAL_1024_TAB[SteerAngle])/1024;
		if (ThrottleRatio > ThrottleRatioLimit)
			ThrottleRatio = ThrottleRatioLimit;
	}
	else //Angle illegal
	{
		//No limit act.
		//ThrottleRatio = ThrottleRatio;
	}
	return ThrottleRatio;
}

/*Function: Battery protect for Mov logic module        */
/*Input:                                                */
/*Output:                                               */
#if (LOGIC_TYPE	== _CONTROLLER_MOVE)
void BatteryProtecMove(void)
{
	gPLCCtl.ucBatteryAct = gPLCCtl.icanHMI.ucBattery;
	//Battery protect
	if (gPLCCtl.ucBatteryDelay > 1000)
	{
		static INT16S delayBatProtect=0;
		static INT16S delayBatLow=0;
		
		//
		gPLCCtl.icanHMI.ucBatteryAct = gPLCCtl.ucBatteryAct;
		if (gPLCCtl.ucBatteryAct <= gCRam.SvPa.BatProtectRatio)
		{//15%
			if (delayBatProtect > 1000)
			{
				SL_SET(PLC_BAT_PROTECTALM);
				SL_CLR(PLC_BAT_LOWALM);
			}
			else
				delayBatProtect++;
		}
		else
		{
			if (delayBatProtect > 0)
				delayBatProtect--;
			
			if (gPLCCtl.ucBatteryAct <= gCRam.SvPa.BatAlarmRatio)
			{//20%
				if (delayBatLow >= 1000)
				{
					SL_SET(PLC_BAT_LOWALM);
				}
				else
				{
						delayBatLow++;
				}
			}
			else
			{
				if (delayBatLow > 0)
					delayBatLow--;
			}
		}
	}
	else
	{
		gPLCCtl.ucBatteryDelay++;
		gPLCCtl.icanHMI.ucBatteryAct = 0;
	}
	gPLCCtl.icanLift.ucBatteryAct = gPLCCtl.ucBatteryAct;

}
#endif //#if (LOGIC_TYPE	== _CONTROLLER_MOVE)

/*Function: Battery protect for Lift logic module        */
/*Input:                                                */
/*Output:                                               */
#if (LOGIC_TYPE	== _CONTROLLER_LIFT)
void BatteryProtecLift(void)
{
		//Battery protect
	if (gPLCCtl.ucBatteryDelay > 1000)
	{
		static INT16S delayBatProtect=0;
		static INT16S delayBatLow=0;
		
		if (gPLCCtl.icanLift.ucBatteryAct <= gCRam.SvPa.BatProtectRatio)
		{//15% No lift
			if (delayBatProtect > 1000)
			{
				SL_SET(PLC_BAT_PROTECTALM);
				SL_CLR(PLC_BAT_LOWALM);
			}
			else
				delayBatProtect++;
		}
		else
		{
			if (delayBatProtect > 0)
				delayBatProtect--;
			
			if (gPLCCtl.icanLift.ucBatteryAct <= gCRam.SvPa.BatAlarmRatio)
			{//20%   
				if (delayBatLow >= 1000)
				{
					SL_SET(PLC_BAT_LOWALM);
				}
				else
				{
						delayBatLow++;
				}
			}
			else
			{
				if (delayBatLow > 0)
					delayBatLow--;
			}
		}
	}
	else
	{
		gPLCCtl.ucBatteryDelay++;
	}

}
#endif //#if (LOGIC_TYPE	== _CONTROLLER_LIFT)
#endif
