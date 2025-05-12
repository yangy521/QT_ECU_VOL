/*******************************************************************************
* Filename: Temprature.h                                             	 		   	   *
*                                                                    		   *
* Description:											   			 		   *
* Author:                                                           		   *
* Date:     														 		   *
* Revision:															 		   *
*																	 		   *
*******************************************************************************/

#include				"PARA.h"
#include  			"PLCHardware.h"
#include 				"CommonRam.h"
#include        "Temprature.h"
#include 				"gd32f30x.h"

//NCPWB473 47.3K 功率板热敏电阻
//1236 功率板热敏电阻（47.3K）温度查找表
//-40~125 度，每度对应的AD采集值 
//AD > TmpTable_Power[0]   -->>   T <= -40
//AD < TmpTable_Power[end] -->>   T >= 125
//此表数据假定低端电压为0
static const INT32S TmpTable_Power[] = 
{
	4049,4046,4043,4039,4035,4031,4027,4022,4017,4012,
	4006,4001,3995,3988,3982,3974,3967,3959,3951,3942,
	3933,3924,3914,3903,3892,3880,3869,3856,3843,3829,
	3815,3800,3785,3768,3752,3734,3716,3697,3678,3657,
	3636,3614,3592,3569,3545,3520,3495,3469,3441,3413,
	3385,3356,3326,3295,3263,3231,3198,3165,3131,3095,
	3059,3024,2987,2950,2912,2873,2835,2795,2756,2715,
	2674,2634,2593,2552,2510,2468,2427,2385,2343,2301,
	2258,2217,2175,2134,2092,2050,2009,1969,1928,1887,
	1847,1808,1769,1730,1691,1653,1616,1579,1542,1506,
	1471,1436,1402,1368,1335,1302,1271,1239,1209,1178,
	1149,1120,1091,1063,1036,1009,984,958,933,909,
	885,862,840,818,796,775,755,735,716,697,
	678,660,643,626,609,593,577,562,547,533,
	519,505,492,479,466,454,442,430,419,408,
	397,387,377,367,357,348,339,330,322,314,
	306,298,290,283,276,269
};

//NCPWB473 47.3K 功率板热敏电阻
//1236 功率板热敏电阻（47.3K）温度查找表
//-40~125 度，每度对应的电阻值（1欧） 
//Res > TmpTable_Power[0]   -->>   T <= -40
//Res < TmpTable_Power[end] -->>   T >= 125
static const INT32S TmpTable_PowerRes[] = 
{
	1747920,1634977,1528257,1427758,1333482,1245428,1167698,1094139,1024751,959532,
	898485,844281,792911,744375,698671,655802,617532,581213,546843,514424,
	483954,456617,430638,406017,382754,360850,341105,322318,304487,287614,
	271697,257291,243565,230518,218151,206463,195829,185688,176038,166880,
	158214,150309,142761,135570,128736,122259,116329,110662,105255,100110,
	95227,90741,86448,82348,78442,74730,71309,68032,64899,61910,
	59065,56435,53914,51501,49196,47000,44964,43011,41140,39350,
	37643,36055,34531,33069,31670,30334,29089,27892,26744,25643,
	24591,23608,22662,21753,20882,20048,19267,18515,17792,17098,
	16433,15809,15207,14628,14072,13539,13037,12553,12087,11639,
	11209,10804,10414,10038,9676,9328,8999,8682,8376,8081,
	7798,7528,7269,7018,6776,6544,6324,6111,5906,5708,
	5518,5337,5163,4994,4831,4674,4524,4379,4238,4102,
	3972,3847,3726,3609,3496,3388,3284,3184,3087,2992,
	2902,2815,2731,2649,2570,2494,2420,2349,2281,2214,
	2150,2088,2028,1970,1914,1860,1808,1757,1708,1661,
	1615,1571,1528,1486,1445,1406
};
//NCPWB473 47.3K 驱动板热敏电阻
//1236 驱动板热敏电阻（47.3K）温度查找表
//-40~125 度，每度对应的AD采集值 
//AD > TmpTable_Drive[0]   -->>   T <= -40
//AD < TmpTable_Drive[end] -->>   T >= 125
#define TmpTable_Drive TmpTable_Power

//KTY84-130 603欧/25度 电机热敏电阻
//-40~165 度，每度对应的AD采集值 
//AD < TmpTable_Drive[0]   -->>   T <= -40
//AD > TmpTable_Drive[end] -->>   T >= 165

static const INT32S TmpTable_Motor[] = 
{
	445,449,453,457,461,465,469,473,477,481,
	485,489,493,497,501,505,509,513,517,522,
	526,530,535,539,544,548,553,557,562,566,
	570,575,580,585,589,594,599,603,608,613,
	618,623,628,633,637,642,647,652,657,662,
	667,673,678,684,689,695,700,705,710,715,
	721,726,732,738,743,749,755,760,766,771,
	776,782,788,794,799,805,811,816,822,828,
	834,840,846,852,858,865,871,877,883,889,
	896,902,909,915,921,928,934,940,947,953,
	959,961,962,964,966,968,979,990,1002,1013,
	1025,1032,1039,1046,1053,1059,1066,1073,1080,1087,
	1094,1101,1109,1116,1123,1130,1137,1145,1152,1159,
	1166,1174,1181,1189,1196,1203,1211,1218,1226,1233,
	1241,1248,1256,1264,1271,1279,1287,1295,1302,1310,
	1318,1326,1334,1342,1350,1359,1367,1375,1382,1390,
	1398,1407,1415,1424,1432,1441,1449,1457,1465,1473,
	1482,1490,1498,1507,1515,1524,1532,1541,1549,1557,
	1566,1575,1584,1593,1602,1611,1620,1628,1637,1646,
	1655,1664,1674,1683,1692,1701,1710,1719,1728,1737,
	1746,1755,1764,1773,1783,1792
};
	
/*******************************************************************************
* Name: TmpPowerCalRes
* Description: 功率板温度检测
* Input
* tempHighAdL0-- 低端通态下AD采集值；
* tempHighAdL33-- 低端断态下AD采集值；
* Output
* 功率板温度，单位：度
* Author: 
* Date: 
* Revision:
*******************************************************************************/
INT16S TmpPowerCalRes(INT16U tempHighAdL0, INT16U tempHighAdL33)
{
	INT16U index;
	INT16S temp,start,end;
	INT32U ThRes;
	
	if (tempHighAdL33 <= tempHighAdL0)
	{
		//SL_SET(PLC_POWER_THRES_ERR);//alarm for thermal res
		ThRes = 20000;
	}
	else
	{
		ThRes = ((INT32U)20000*tempHighAdL0)/(tempHighAdL33 - tempHighAdL0);
	}
	end = (sizeof(TmpTable_PowerRes)/sizeof(TmpTable_PowerRes[0])) - 1;
	start = 0;

	while (start < end)
	{
		index = start + (end - start)/2;
		if(ThRes < TmpTable_PowerRes[index])
		{
			start = index + 1;
		} 
		else 
		{
			end = index ;
		}
	}
	temp = TMP_INDEX0 + (end-1) * TMP_POWER_STEP;

	return temp;
}
/*******************************************************************************
* Name: TmpDriveCal
* Description:驱动板温度检测
* Author: 
* Date: 
* Revision:
*******************************************************************************/
INT16S TmpDriveCal(INT16U tmpHighAd)
{
	INT16U index;
	INT16S temp,start,end;

	end = (sizeof(TmpTable_Drive)/sizeof(TmpTable_Drive[0])) - 1;
	start = 0;

	while (start < end)
	{
		index = start + (end - start)/2;
		if(tmpHighAd < TmpTable_Drive[index])
		{
			start = index + 1;
		} 
		else 
		{
			end = index ;
		}
	}
	temp = TMP_INDEX0 + (end-1) * TMP_DRIVE_STEP;

	return temp;
}

/*******************************************************************************
* Name: TmpMotorCal
* Description:电机温度检测
* Author: 
* Date: 
* Revision:
*******************************************************************************/
INT16S TmpMotorCal(INT16U tmpMotor)
{
	INT16U index;
	INT16S temp,start,end;

	end = (sizeof(TmpTable_Motor)/sizeof(TmpTable_Motor[0])) - 1;
	start = 0;

	while (start < end)
	{
		index = start + (end - start)/2;
		if(tmpMotor > TmpTable_Motor[index])
		{
			start = index + 1;
		} 
		else 
		{
			end = index ;
		}
	}
	temp = TMP_INDEX0 + (end-1) * TMP_MOTOR_STEP;

	return temp;
}
/*******************************************************************************
* Name: TmpProcess
* Description: 控制器温度检测
* Author: 
* Date: 
* Revision:
*******************************************************************************/
#define		TMPPW_LH			(1 << 0)
#define		TMPPW_LL			(1 << 1)
#define		TMPPW_NORMAL	(1 << 2)
#define   TMP_PWON_DELAY  256
	//set AD to lowside low
#if ((CTLBOARD_TYPE ==_1222) || (CTLBOARD_TYPE ==_1242))
#define RESET_AD_LOWSIDE		GPIO_ResetBits(GPIOC,GPIO_Pin_3)
#endif
#if (CTLBOARD_TYPE ==_1232)
#define RESET_AD_LOWSIDE		GPIO_ResetBits(GPIOC,GPIO_Pin_1)
#endif
#if ((CTLBOARD_TYPE ==_1236) || (CTLBOARD_TYPE ==_1246))
#define RESET_AD_LOWSIDE		GPIO_ResetBits(GPIOC,GPIO_Pin_2)
#endif
#if ((CTLBOARD_TYPE ==_1220_V103) || (CTLBOARD_TYPE ==_1220))
#define RESET_AD_LOWSIDE		GPIO_ResetBits(GPIOC,GPIO_Pin_4)
#endif
#if ((CTLBOARD_TYPE ==_1226))
#define RESET_AD_LOWSIDE		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET)
#endif
#if ((CTLBOARD_TYPE ==_1226_GD32))
#define RESET_AD_LOWSIDE		gpio_bit_write(GPIOE,GPIO_PIN_15,RESET)
#endif

//set AD to lowside high
#if ((CTLBOARD_TYPE ==_1222) || (CTLBOARD_TYPE ==_1242))
#define SET_AD_LOWSIDE		GPIO_SetBits(GPIOC,GPIO_Pin_3)
#endif
#if (CTLBOARD_TYPE ==_1232)
#define SET_AD_LOWSIDE		GPIO_SetBits(GPIOC,GPIO_Pin_1)
#endif
#if ((CTLBOARD_TYPE ==_1236) || (CTLBOARD_TYPE ==_1246))
#define SET_AD_LOWSIDE		GPIO_SetBits(GPIOC,GPIO_Pin_2)
#endif
#if ((CTLBOARD_TYPE ==_1220_V103) || (CTLBOARD_TYPE ==_1220))
#define SET_AD_LOWSIDE		GPIO_SetBits(GPIOC,GPIO_Pin_4)
#endif
#if ((CTLBOARD_TYPE ==_1226))
#define SET_AD_LOWSIDE		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET)
#endif
#if ((CTLBOARD_TYPE ==_1226_GD32))
#define SET_AD_LOWSIDE		gpio_bit_write(GPIOE,GPIO_PIN_15,SET)
#endif

void TmpInit(void)
{
	gPLCCtl.TmpFlag = 0;
	gPLCCtl.TmpPowerLSum = 0;
	gPLCCtl.TmpPowerHSum = 0;
	gPLCCtl.TmpPowerL = 0;
	gPLCCtl.TmpPowerH = 0;
	gPLCCtl.TmpCount = 0;
	gPLCCtl.TmpMotorAdSum = 0;
	gPLCCtl.TmpDriveAdSum = 0;
	gPLCCtl.TmpMotorAd = 0;
	gPLCCtl.TmpDriveAd = 0;
	
	//set AD to lowside low
	RESET_AD_LOWSIDE;
}

void TmpProcess(void)
{
	if ((gPLCCtl.TmpFlag & TMPPW_NORMAL) != 0)
	{
		//alter ad for tmp power
		if ((gPLCCtl.TmpCount & 1) == 0)
		{
			gPLCCtl.TmpPowerLSum += gPLCCtl.aiDataIn[AD_PowTmpHigh] - gPLCCtl.TmpPowerL;
			gPLCCtl.TmpPowerL = gPLCCtl.TmpPowerLSum >> 3;
			//set AD to lowside high
			SET_AD_LOWSIDE;
		}
		else
		{
			gPLCCtl.TmpPowerHSum += gPLCCtl.aiDataIn[AD_PowTmpHigh] - gPLCCtl.TmpPowerH;
			gPLCCtl.TmpPowerH = gPLCCtl.TmpPowerHSum >> 3;
			//set AD to lowside low
			RESET_AD_LOWSIDE;
		}
		gPLCCtl.TmpPower = TmpPowerCalRes(gPLCCtl.TmpPowerL,gPLCCtl.TmpPowerH);//功率板
		gPLCCtl.TmpCount++;
		//cal drive board tmp
	#if ((CTLBOARD_TYPE ==_1236) || (CTLBOARD_TYPE ==_1246))
		gPLCCtl.TmpDriveAdSum += gPLCCtl.aiDataIn[AD_DriveTmp] - gPLCCtl.TmpDriveAd;
		gPLCCtl.TmpDriveAd = gPLCCtl.TmpDriveAdSum >> 3;
		gPLCCtl.TmpDrive=TmpDriveCal(gPLCCtl.TmpDriveAd);//驱动板
	#endif
		//MOTOR TMP
		if((gCRam.SvPa.ConBit1 & MotorTmpChkEnable) != 0)
		{
			gPLCCtl.TmpMotorAdSum += gPLCCtl.aiDataIn[AD_MotorTmp] - gPLCCtl.TmpMotorAd;
			gPLCCtl.TmpMotorAd = gPLCCtl.TmpMotorAdSum >> 3;
			gPLCCtl.TmpMotor=TmpMotorCal(gPLCCtl.TmpMotorAd);//电机
		}
		else
		{
			gPLCCtl.TmpMotor=TMP_DEFAULT;
		}
	}
	else //POWER ON Process
	{
		if (gPLCCtl.TmpCount < TMP_PWON_DELAY)
		{
			//alter ad, prepare filter
			if ((gPLCCtl.TmpCount & 1) == 0)
			{
				gPLCCtl.TmpPowerLSum += gPLCCtl.aiDataIn[AD_PowTmpHigh] - gPLCCtl.TmpPowerL;
				gPLCCtl.TmpPowerL = gPLCCtl.TmpPowerLSum >> 3;
				//set AD to lowside high
				SET_AD_LOWSIDE;
			}
			else
			{
				gPLCCtl.TmpPowerHSum += gPLCCtl.aiDataIn[AD_PowTmpHigh] - gPLCCtl.TmpPowerH;
				gPLCCtl.TmpPowerH = gPLCCtl.TmpPowerHSum >> 3;
				//set AD to lowside low
				RESET_AD_LOWSIDE;
			}
			gPLCCtl.TmpCount++;
			//TmpDrive init
		#if ((CTLBOARD_TYPE ==_1236) || (CTLBOARD_TYPE ==_1246))
			gPLCCtl.TmpDriveAdSum += gPLCCtl.aiDataIn[AD_DriveTmp] - gPLCCtl.TmpDriveAd;
			gPLCCtl.TmpDriveAd = gPLCCtl.TmpDriveAdSum >> 3;
		#endif
			//TmpMotor init
			if((gCRam.SvPa.ConBit1 & MotorTmpChkEnable) != 0)
			{
				gPLCCtl.TmpMotorAdSum += gPLCCtl.aiDataIn[AD_MotorTmp] - gPLCCtl.TmpMotorAd;
				gPLCCtl.TmpMotorAd = gPLCCtl.TmpMotorAdSum >> 3;
			}
		}
		else // Power on Process end
		{
			gPLCCtl.TmpFlag = TMPPW_NORMAL;
		}
	}
}
