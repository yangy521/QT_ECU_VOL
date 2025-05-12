/*******************************************************************************
* Filename: DoPwm.c 	                                    	     	       *
* Description: 	DoPwm功能					           				           *
* Author:  QExpand, Lilu                                                       *
* Date: 2023/5/16															   *
* Revision:	V1.00														       *
*******************************************************************************/
#include "DoPwm.h"
#include "Log.h"
#include "Device.h"
#include "WdgProc.h"
#include "Para.h"
#include "LocalAi.h"
#include "NetTimer.h"
#include "LocalDi.h"
#include "ErrCode.h"


typedef union
{
	uint16_t u16DriverEn;
	struct
	{
		uint16_t b1Driver1En: 1;
		uint16_t b1Driver2En: 1;
		uint16_t b1Driver3En: 1;
		uint16_t b1Driver4En: 1;
		uint16_t b1Driver5En: 1;
		uint16_t b1Driver6En: 1;
		uint16_t b1Driver7En: 1;
		uint16_t b1Driver8En: 1;
		uint16_t b1Driver9En: 1;
		uint16_t b1Driver10En: 1;
		uint16_t b5Reserve: 5;
		uint16_t b1VolMode: 1;
	};
}xDriverEn;

typedef union
{
	uint16_t u16Data;
	struct
	{
		uint8_t u8HoldVol;
		uint8_t	u8PullVol;
	};
}xDriverVol;

typedef struct
{
	xDriverEn DriverEn;
	xDriverVol DriverVol[DoPwmMax];
	xDoPwmPara DoPwmPara[DoPwmMax];	
	DriverErrCallBackt CallBack;
	
}xDriverInfo;

static xDriverInfo sgDriverInfo;	/*DoPwm全局变量*/
 

static void vDoPwmCheck(void);

/*******************************************************************************
* Name: void vDoPwmInit(void)
* Descriptio: DoPwm初始化函数
* Input: NULL
* Output: NULL
*******************************************************************************/
void vDoPwmInit(void)
{	
//	sgDriverInfo.DriverEn.u16DriverEn = 0x03FE;
	sgDriverInfo.DriverEn.u16DriverEn = (uint16_t)i32GetPara(PARA_DriverFlag);
	sgDriverInfo.DriverVol[DRIVER1].u16Data = (uint16_t)i32GetPara(PARA_Driver1Vol);
	sgDriverInfo.DriverVol[DRIVER2].u16Data = (uint16_t)i32GetPara(PARA_Driver2Vol);
	sgDriverInfo.DriverVol[DRIVER3].u16Data = (uint16_t)i32GetPara(PARA_Driver3Vol);
	sgDriverInfo.DriverVol[DRIVER4].u16Data = (uint16_t)i32GetPara(PARA_Driver4Vol);
	sgDriverInfo.DriverVol[DRIVER5].u16Data = (uint16_t)i32GetPara(PARA_Driver5Vol);
	sgDriverInfo.DriverVol[DRIVER6].u16Data = (uint16_t)i32GetPara(PARA_Driver6Vol);
	sgDriverInfo.DriverVol[DRIVER7].u16Data = (uint16_t)i32GetPara(PARA_Driver7Vol);
	sgDriverInfo.DriverVol[DRIVER8].u16Data = (uint16_t)i32GetPara(PARA_Driver8Vol);
	sgDriverInfo.DriverVol[DRIVER9].u16Data = (uint16_t)i32GetPara(PARA_Driver9Vol);
	sgDriverInfo.DriverVol[DRIVER10].u16Data = (uint16_t)i32GetPara(PARA_Driver10Vol);
	sgDriverInfo.DriverEn.b1VolMode = (i32GetPara(PARA_ValveType) >> 15) & 0x1;					/*lilu 20230923 Add VolMode*/
}


/*******************************************************************************
* Name: void vDoPwmErrReg(DriverErrCallBackt Callback)
* Descriptio: 设置DoPwmc错误回调函数
* Input: Callback ： 回调函数
* Output: NULL
*******************************************************************************/
void vDoPwmErrReg(DriverErrCallBackt Callback)
{
	sgDriverInfo.CallBack = Callback;
}

/*******************************************************************************
* Name: int32_t i32DoPwmSet(eDoPwmNo DoPwmNo, uint8_t u8Flag)
* Descriptio: 设置DoPwm通道数据
* Input: DoPwmNo：DoPwm通道编号
  u8Flag：0: Open; 1: Close
* Output:-1：输入参数错误
		  0：设置成功
*******************************************************************************/
int32_t i32DoPwmSet(eDoPwmNo DoPwmNo, uint8_t u8Flag)
{
	if(DoPwmNo >= DoPwmMax)
	{
		i32LogWrite(ERR, LOG_PWM, "DoPwm Parameter is wrong, DoPwmMax = %d, DoPwm = %d\r\n", DoPwmMax, DoPwmNo);
		return -1;
	}
	if (0 == i32ErrCodeCheck(ErrCode51 + DoPwmNo))
	{
		sgDriverInfo.DoPwmPara[DoPwmNo].u8DoPwmFlag = u8Flag;
	}
	return 0;
}
	
/*单个DoPwm运行功能*/
static void vOneDoPwmPrco(eDoPwmNo DoPwmNo)
{
	_iq VcmpRatio;
	uint8_t u8EnFlag = 0;
	
	if (0 != (sgDriverInfo.DriverEn.u16DriverEn & (1 << DoPwmNo)))
	{
		u8EnFlag = 1;
	}
	//	if ((1 == u8EnFlag) && (0 == i32ErrCodeCheck(ErrCode51 + DoPwmNo)))
	if (1 == u8EnFlag)
	{
		if (0 == i32ErrCodeCheck(ErrCode51 + DoPwmNo))
		{
			if (1 == sgDriverInfo.DriverEn.b1VolMode)
			{
				VcmpRatio = _IQdiv(_IQ(48.0/STD_VBUS), _IQ12toIQ(i32LocalAiGet(AI_B_KSI_CHECK)));	/*lilu, 20230704*/
			}
			else
			{
				VcmpRatio = _IQdiv(_IQ(24.0/STD_VBUS), _IQ12toIQ(i32LocalAiGet(AI_B_KSI_CHECK)));	/*lilu, 20230704*/
			}
	//		VcmpRatio = _IQdiv(_IQ(STD_VOLTAGE/STD_VBUS), _IQ12toIQ(i32LocalAiGet(AI_B_KSI_CHECK)));	/*lilu, 20230704*/
			
			if(0 == sgDriverInfo.DoPwmPara[DoPwmNo].u8DoPwmFlag)
			{
				sgDriverInfo.DoPwmPara[DoPwmNo].u16PluseWidth = 0;
				sgDriverInfo.DoPwmPara[DoPwmNo].u16PluseWidthDelay = FULL_VOLTAGE_ACT_TIME;
			}
			else
			{
				if(sgDriverInfo.DoPwmPara[DoPwmNo].u16PluseWidthDelay > 0)
				{
					sgDriverInfo.DoPwmPara[DoPwmNo].u16PluseWidth = sgDriverInfo.DriverVol[DoPwmNo].u8PullVol * DO_PWM_TIM_PERIOD / 100;
					sgDriverInfo.DoPwmPara[DoPwmNo].u16PluseWidthDelay--;
				}
				else
				{
					sgDriverInfo.DoPwmPara[DoPwmNo].u16PluseWidth = _IQmpy(sgDriverInfo.DriverVol[DoPwmNo].u8HoldVol / 100.0 * DO_PWM_TIM_PERIOD, VcmpRatio);	/*lilu, 20230704*/
					if(sgDriverInfo.DoPwmPara[DoPwmNo].u16PluseWidth > DO_PWM_CMP_LIMIT_MAX)
					{
						sgDriverInfo.DoPwmPara[DoPwmNo].u16PluseWidth = DO_PWM_CMP_LIMIT_MAX;
					}
					else if(sgDriverInfo.DoPwmPara[DoPwmNo].u16PluseWidth < DO_PWM_CMP_LIMIT_MIN)
					{
						sgDriverInfo.DoPwmPara[DoPwmNo].u16PluseWidth = DO_PWM_CMP_LIMIT_MIN;
					}
				}
			}
		}
		else
		{
			sgDriverInfo.DoPwmPara[DoPwmNo].u16PluseWidth = 0;
		}
	}
	else
	{
		if (DRIVER1 == DoPwmNo)
		{
			sgDriverInfo.DoPwmPara[DRIVER1].u16PluseWidth = DO_PWM_TIM_PERIOD;
		}
		else if (DRIVER2 == DoPwmNo)
		{
			sgDriverInfo.DoPwmPara[DRIVER2].u16PluseWidth = DO_PWM_TIM_PERIOD;
		}
		else if (DRIVER3 == DoPwmNo)
		{
			sgDriverInfo.DoPwmPara[DRIVER3].u16PluseWidth = DO_PWM_TIM_PERIOD;
		}
		else
		{
			sgDriverInfo.DoPwmPara[DoPwmNo].u16PluseWidth = 0;
		}
	}
	/*设置对应通道的PWM输出*/
	vPwmDriver(DoPwmNo, sgDriverInfo.DoPwmPara[DoPwmNo].u16PluseWidth);

}

/*******************************************************************************
* Name: void vDoPwmProc(void)
* Descriptio: DoPwm运行功能函数
* Input: NULL
* Output:  NULL
*******************************************************************************/
void vDoPwmProc(void)
{
	uint8_t i = 0;
	for(i = 0; i < DoPwmMax; i++)
	{
		vOneDoPwmPrco(i);
	}
	
	vDoPwmCheck();
	vWdgSetFun(WDG_DOPWM_BIT);
}

/*PWM的短路和掉线检测*/
static void vDoPwmCheck(void)
{
	//xDoPwmPara *DoPwmTmp;
	
	uint8_t i = 0;
	
	for (i = 0; i < DoPwmMax; i++)
	{
		uint8_t u8EnFlag = 0;
		uint8_t u8TmpFlag = 0;
		if (0 != (sgDriverInfo.DriverEn.u16DriverEn & (1 << i)))
		{
			u8EnFlag = 1;
		}
		
		/*Driver6 No Check*/
		if(DRIVER6 == i)
		{
			continue;
		}
#if (USER_TYPE == USER_RUYI_CDD15C)
		if(DRIVER4 == i)
		{
			continue;
		}
#endif
		
		if (1 == u8EnFlag)
		{
#ifdef DoPowerOnCaL_EN
			if (((DO_PWM_TIM_PERIOD == sgDriverInfo.DoPwmPara[i].u16PluseWidth) && (0 != i32LocalDiGet(DRIVER1_R + i)))
				|| ((0 == sgDriverInfo.DoPwmPara[i].u16PluseWidth) && (0 == i32LocalDiGet(DRIVER1_R + i))))
			{
				u8TmpFlag |= 1 << 0;
			}
#endif
#ifdef DoFdbVoltageCal_EN
			if (((sgDriverInfo.DoPwmPara[i].u16PluseWidth <= DO_PWM_CMP_LIMIT_MAX) && (sgDriverInfo.DoPwmPara[i].u16PluseWidth >= DO_PWM_CMP_LIMIT_MIN)) 
				&& ((0 == sgDriverInfo.DoPwmPara[i].u16DoFdbVoltage) || ((1 << DoFdbVoltageSHIFT) == sgDriverInfo.DoPwmPara[i].u16DoFdbVoltage)))
#endif
			{
				u8TmpFlag |= 1 << 1;
			}
			
			if(0 != u8TmpFlag)
			{
				if(false == u8GetNetTimerStartFlag(TIMER_Drive1Check + i))
				{
					vSetNetTimer(TIMER_Drive1Check + i, DO_LOST_CHK_DELAY_TIME);
				}
				if(true == u8GetNetTimerOverFlag(TIMER_Drive1Check + i))
				{
					i32ErrCodeSet(ErrCode51 + i);
					if (NULL != sgDriverInfo.CallBack)
					{
						sgDriverInfo.CallBack(i);
					}
					vKillNetTimer(TIMER_Drive1Check + i);
				}
			}
			else
			{
				if(true == u8GetNetTimerStartFlag(TIMER_Drive1Check + i))
				{
					vKillNetTimer(TIMER_Drive1Check + i);
				}
			}
		}
	}
}


#ifdef DoFdbVoltageCal_EN
/*******************************************************************************
* Name: void vDoFdbVoltageCal(void)
* Descriptio: DoPwm短路和掉线运行功能函数
* Input: NULL
* Output:  NULL
*******************************************************************************/
void vDoFdbVoltageCal(void)
{
	if(0 == READ_DRV1())
	{
		sgDriverInfo.DoPwmPara[DRIVER1].u16DoFdbVoltage = sgDriverInfo.DoPwmPara[DRIVER1].u16DoFdbVoltageSum >> DoFdbVoltageSHIFT;
		if (0 != sgDriverInfo.DoPwmPara[DRIVER1].u16DoFdbVoltageSum)
		{
			if (sgDriverInfo.DoPwmPara[DRIVER1].u16DoFdbVoltage == 0)
			{
				sgDriverInfo.DoPwmPara[DRIVER1].u16DoFdbVoltage = 1;
			}

			sgDriverInfo.DoPwmPara[DRIVER1].u16DoFdbVoltageSum -= sgDriverInfo.DoPwmPara[DRIVER1].u16DoFdbVoltage;
		}
	}
	else
	{
		sgDriverInfo.DoPwmPara[DRIVER1].u16DoFdbVoltageSum += (1 << DoFdbVoltageSHIFT);
		sgDriverInfo.DoPwmPara[DRIVER1].u16DoFdbVoltage = sgDriverInfo.DoPwmPara[DRIVER1].u16DoFdbVoltageSum >> DoFdbVoltageSHIFT;
		if (0 == sgDriverInfo.DoPwmPara[DRIVER1].u16DoFdbVoltage)
		{
			sgDriverInfo.DoPwmPara[DRIVER1].u16DoFdbVoltage = 1;
		}
		sgDriverInfo.DoPwmPara[DRIVER1].u16DoFdbVoltageSum -= sgDriverInfo.DoPwmPara[DRIVER1].u16DoFdbVoltage;
	}
	
	if(0 == READ_DRV2())
	{
		sgDriverInfo.DoPwmPara[DRIVER2].u16DoFdbVoltage = sgDriverInfo.DoPwmPara[DRIVER2].u16DoFdbVoltageSum >> DoFdbVoltageSHIFT;
		if (0 != sgDriverInfo.DoPwmPara[DRIVER2].u16DoFdbVoltageSum)
		{
			if (sgDriverInfo.DoPwmPara[DRIVER2].u16DoFdbVoltage == 0)
			{
				sgDriverInfo.DoPwmPara[DRIVER2].u16DoFdbVoltage = 1;
			}

			sgDriverInfo.DoPwmPara[DRIVER2].u16DoFdbVoltageSum -= sgDriverInfo.DoPwmPara[DRIVER2].u16DoFdbVoltage;
		}
	}
	else
	{
		sgDriverInfo.DoPwmPara[DRIVER2].u16DoFdbVoltageSum += (1 << DoFdbVoltageSHIFT);
		sgDriverInfo.DoPwmPara[DRIVER2].u16DoFdbVoltage = sgDriverInfo.DoPwmPara[DRIVER2].u16DoFdbVoltageSum >> DoFdbVoltageSHIFT;
		if (0 == sgDriverInfo.DoPwmPara[DRIVER2].u16DoFdbVoltage)
		{
			sgDriverInfo.DoPwmPara[DRIVER2].u16DoFdbVoltage = 1;
		}
		sgDriverInfo.DoPwmPara[DRIVER2].u16DoFdbVoltageSum -= sgDriverInfo.DoPwmPara[DRIVER2].u16DoFdbVoltage;
	}
	
	
	if(0 == READ_DRV3())
	{
		sgDriverInfo.DoPwmPara[DRIVER3].u16DoFdbVoltage = sgDriverInfo.DoPwmPara[DRIVER3].u16DoFdbVoltageSum >> DoFdbVoltageSHIFT;
		if (0 != sgDriverInfo.DoPwmPara[DRIVER3].u16DoFdbVoltageSum)
		{
			if (sgDriverInfo.DoPwmPara[DRIVER3].u16DoFdbVoltage == 0)
			{
				sgDriverInfo.DoPwmPara[DRIVER3].u16DoFdbVoltage = 1;
			}

			sgDriverInfo.DoPwmPara[DRIVER3].u16DoFdbVoltageSum -= sgDriverInfo.DoPwmPara[DRIVER3].u16DoFdbVoltage;
		}
	}
	else
	{
		sgDriverInfo.DoPwmPara[DRIVER3].u16DoFdbVoltageSum += (1 << DoFdbVoltageSHIFT);
		sgDriverInfo.DoPwmPara[DRIVER3].u16DoFdbVoltage = sgDriverInfo.DoPwmPara[DRIVER3].u16DoFdbVoltageSum >> DoFdbVoltageSHIFT;
		if (0 == sgDriverInfo.DoPwmPara[DRIVER3].u16DoFdbVoltage)
		{
			sgDriverInfo.DoPwmPara[DRIVER3].u16DoFdbVoltage = 1;
		}
		sgDriverInfo.DoPwmPara[DRIVER3].u16DoFdbVoltageSum -= sgDriverInfo.DoPwmPara[DRIVER3].u16DoFdbVoltage;
	}
	
	if(0 == READ_DRV4())
	{
		sgDriverInfo.DoPwmPara[DRIVER4].u16DoFdbVoltage = sgDriverInfo.DoPwmPara[DRIVER4].u16DoFdbVoltageSum >> DoFdbVoltageSHIFT;
		if (0 != sgDriverInfo.DoPwmPara[DRIVER4].u16DoFdbVoltageSum)
		{
			if (sgDriverInfo.DoPwmPara[DRIVER4].u16DoFdbVoltage == 0)
			{
				sgDriverInfo.DoPwmPara[DRIVER4].u16DoFdbVoltage = 1;
			}

			sgDriverInfo.DoPwmPara[DRIVER4].u16DoFdbVoltageSum -= sgDriverInfo.DoPwmPara[DRIVER4].u16DoFdbVoltage;
		}
	}
	else
	{
		sgDriverInfo.DoPwmPara[DRIVER4].u16DoFdbVoltageSum += (1 << DoFdbVoltageSHIFT);
		sgDriverInfo.DoPwmPara[DRIVER4].u16DoFdbVoltage = sgDriverInfo.DoPwmPara[DRIVER4].u16DoFdbVoltageSum >> DoFdbVoltageSHIFT;
		if (0 == sgDriverInfo.DoPwmPara[DRIVER4].u16DoFdbVoltage)
		{
			sgDriverInfo.DoPwmPara[DRIVER4].u16DoFdbVoltage = 1;
		}
		sgDriverInfo.DoPwmPara[DRIVER4].u16DoFdbVoltageSum -= sgDriverInfo.DoPwmPara[DRIVER4].u16DoFdbVoltage;
	}
	
	if(0 == READ_DRV5())
	{
		sgDriverInfo.DoPwmPara[DRIVER5].u16DoFdbVoltage = sgDriverInfo.DoPwmPara[DRIVER5].u16DoFdbVoltageSum >> DoFdbVoltageSHIFT;
		if (0 != sgDriverInfo.DoPwmPara[DRIVER5].u16DoFdbVoltageSum)
		{
			if (sgDriverInfo.DoPwmPara[DRIVER5].u16DoFdbVoltage == 0)
			{
				sgDriverInfo.DoPwmPara[DRIVER5].u16DoFdbVoltage = 1;
			}

			sgDriverInfo.DoPwmPara[DRIVER5].u16DoFdbVoltageSum -= sgDriverInfo.DoPwmPara[DRIVER5].u16DoFdbVoltage;
		}
	}
	else
	{
		sgDriverInfo.DoPwmPara[DRIVER5].u16DoFdbVoltageSum += (1 << DoFdbVoltageSHIFT);
		sgDriverInfo.DoPwmPara[DRIVER5].u16DoFdbVoltage = sgDriverInfo.DoPwmPara[DRIVER5].u16DoFdbVoltageSum >> DoFdbVoltageSHIFT;
		if (0 == sgDriverInfo.DoPwmPara[DRIVER5].u16DoFdbVoltage)
		{
			sgDriverInfo.DoPwmPara[DRIVER5].u16DoFdbVoltage = 1;
		}
		sgDriverInfo.DoPwmPara[DRIVER5].u16DoFdbVoltageSum -= sgDriverInfo.DoPwmPara[DRIVER5].u16DoFdbVoltage;
	}
	
	if(0 == READ_DRV6())
	{
		sgDriverInfo.DoPwmPara[DRIVER6].u16DoFdbVoltage = sgDriverInfo.DoPwmPara[DRIVER6].u16DoFdbVoltageSum >> DoFdbVoltageSHIFT;
		if (0 != sgDriverInfo.DoPwmPara[DRIVER6].u16DoFdbVoltageSum)
		{
			if (sgDriverInfo.DoPwmPara[DRIVER6].u16DoFdbVoltage == 0)
			{
				sgDriverInfo.DoPwmPara[DRIVER6].u16DoFdbVoltage = 1;
			}

			sgDriverInfo.DoPwmPara[DRIVER6].u16DoFdbVoltageSum -= sgDriverInfo.DoPwmPara[DRIVER6].u16DoFdbVoltage;
		}
	}
	else
	{
		sgDriverInfo.DoPwmPara[DRIVER6].u16DoFdbVoltageSum += (1 << DoFdbVoltageSHIFT);
		sgDriverInfo.DoPwmPara[DRIVER6].u16DoFdbVoltage = sgDriverInfo.DoPwmPara[DRIVER6].u16DoFdbVoltageSum >> DoFdbVoltageSHIFT;
		if (0 == sgDriverInfo.DoPwmPara[DRIVER6].u16DoFdbVoltage)
		{
			sgDriverInfo.DoPwmPara[DRIVER6].u16DoFdbVoltage = 1;
		}
		sgDriverInfo.DoPwmPara[DRIVER6].u16DoFdbVoltageSum -= sgDriverInfo.DoPwmPara[DRIVER6].u16DoFdbVoltage;
	}
	
	if(0 == READ_DRV7())
	{
		sgDriverInfo.DoPwmPara[DRIVER7].u16DoFdbVoltage = sgDriverInfo.DoPwmPara[DRIVER7].u16DoFdbVoltageSum >> DoFdbVoltageSHIFT;
		if (0 != sgDriverInfo.DoPwmPara[DRIVER7].u16DoFdbVoltageSum)
		{
			if (sgDriverInfo.DoPwmPara[DRIVER7].u16DoFdbVoltage == 0)
			{
				sgDriverInfo.DoPwmPara[DRIVER7].u16DoFdbVoltage = 1;
			}

			sgDriverInfo.DoPwmPara[DRIVER7].u16DoFdbVoltageSum -= sgDriverInfo.DoPwmPara[DRIVER7].u16DoFdbVoltage;
		}
	}
	else
	{
		sgDriverInfo.DoPwmPara[DRIVER7].u16DoFdbVoltageSum += (1 << DoFdbVoltageSHIFT);
		sgDriverInfo.DoPwmPara[DRIVER7].u16DoFdbVoltage = sgDriverInfo.DoPwmPara[DRIVER7].u16DoFdbVoltageSum >> DoFdbVoltageSHIFT;
		if (0 == sgDriverInfo.DoPwmPara[DRIVER7].u16DoFdbVoltage)
		{
			sgDriverInfo.DoPwmPara[DRIVER7].u16DoFdbVoltage = 1;
		}
		sgDriverInfo.DoPwmPara[DRIVER7].u16DoFdbVoltageSum -= sgDriverInfo.DoPwmPara[DRIVER7].u16DoFdbVoltage;
	}
	
	if(0 == READ_DRV8())
	{
		sgDriverInfo.DoPwmPara[DRIVER8].u16DoFdbVoltage = sgDriverInfo.DoPwmPara[DRIVER8].u16DoFdbVoltageSum >> DoFdbVoltageSHIFT;
		if (0 != sgDriverInfo.DoPwmPara[DRIVER8].u16DoFdbVoltageSum)
		{
			if (sgDriverInfo.DoPwmPara[DRIVER8].u16DoFdbVoltage == 0)
			{
				sgDriverInfo.DoPwmPara[DRIVER8].u16DoFdbVoltage = 1;
			}

			sgDriverInfo.DoPwmPara[DRIVER8].u16DoFdbVoltageSum -= sgDriverInfo.DoPwmPara[DRIVER8].u16DoFdbVoltage;
		}
	}
	else
	{
		sgDriverInfo.DoPwmPara[DRIVER8].u16DoFdbVoltageSum += (1 << DoFdbVoltageSHIFT);
		sgDriverInfo.DoPwmPara[DRIVER8].u16DoFdbVoltage = sgDriverInfo.DoPwmPara[DRIVER8].u16DoFdbVoltageSum >> DoFdbVoltageSHIFT;
		if (0 == sgDriverInfo.DoPwmPara[DRIVER8].u16DoFdbVoltage)
		{
			sgDriverInfo.DoPwmPara[DRIVER8].u16DoFdbVoltage = 1;
		}
		sgDriverInfo.DoPwmPara[DRIVER8].u16DoFdbVoltageSum -= sgDriverInfo.DoPwmPara[DRIVER8].u16DoFdbVoltage;
	}
	
	if(0 == READ_DRV9())
	{
		sgDriverInfo.DoPwmPara[DRIVER9].u16DoFdbVoltage = sgDriverInfo.DoPwmPara[DRIVER9].u16DoFdbVoltageSum >> DoFdbVoltageSHIFT;
		if (0 != sgDriverInfo.DoPwmPara[DRIVER9].u16DoFdbVoltageSum)
		{
			if (sgDriverInfo.DoPwmPara[DRIVER9].u16DoFdbVoltage == 0)
			{
				sgDriverInfo.DoPwmPara[DRIVER9].u16DoFdbVoltage = 1;
			}

			sgDriverInfo.DoPwmPara[DRIVER9].u16DoFdbVoltageSum -= sgDriverInfo.DoPwmPara[DRIVER9].u16DoFdbVoltage;
		}
	}
	else
	{
		sgDriverInfo.DoPwmPara[DRIVER9].u16DoFdbVoltageSum += (1 << DoFdbVoltageSHIFT);
		sgDriverInfo.DoPwmPara[DRIVER9].u16DoFdbVoltage = sgDriverInfo.DoPwmPara[DRIVER9].u16DoFdbVoltageSum >> DoFdbVoltageSHIFT;
		if (0 == sgDriverInfo.DoPwmPara[DRIVER9].u16DoFdbVoltage)
		{
			sgDriverInfo.DoPwmPara[DRIVER9].u16DoFdbVoltage = 1;
		}
		sgDriverInfo.DoPwmPara[DRIVER9].u16DoFdbVoltageSum -= sgDriverInfo.DoPwmPara[DRIVER9].u16DoFdbVoltage;
	}
	
	if(0 == READ_DRV10())
	{
		sgDriverInfo.DoPwmPara[DRIVER10].u16DoFdbVoltage = sgDriverInfo.DoPwmPara[DRIVER10].u16DoFdbVoltageSum >> DoFdbVoltageSHIFT;
		if (0 != sgDriverInfo.DoPwmPara[DRIVER10].u16DoFdbVoltageSum)
		{
			if (sgDriverInfo.DoPwmPara[DRIVER10].u16DoFdbVoltage == 0)
			{
				sgDriverInfo.DoPwmPara[DRIVER10].u16DoFdbVoltage = 1;
			}

			sgDriverInfo.DoPwmPara[DRIVER10].u16DoFdbVoltageSum -= sgDriverInfo.DoPwmPara[DRIVER10].u16DoFdbVoltage;
		}
	}
	else
	{
		sgDriverInfo.DoPwmPara[DRIVER10].u16DoFdbVoltageSum += (1 << DoFdbVoltageSHIFT);
		sgDriverInfo.DoPwmPara[DRIVER10].u16DoFdbVoltage = sgDriverInfo.DoPwmPara[DRIVER10].u16DoFdbVoltageSum >> DoFdbVoltageSHIFT;
		if (0 == sgDriverInfo.DoPwmPara[DRIVER10].u16DoFdbVoltage)
		{
			sgDriverInfo.DoPwmPara[DRIVER10].u16DoFdbVoltage = 1;
		}
		sgDriverInfo.DoPwmPara[DRIVER10].u16DoFdbVoltageSum -= sgDriverInfo.DoPwmPara[DRIVER10].u16DoFdbVoltage;
	}
	
//	if(0 == READ_DRV11())
//	{
//		sgDriverInfo.DoPwmPara[DRIVER11].u16DoFdbVoltage = sgDriverInfo.DoPwmPara[DRIVER11].u16DoFdbVoltageSum >> DoFdbVoltageSHIFT;
//		if (0 != sgDriverInfo.DoPwmPara[DRIVER11].u16DoFdbVoltageSum)
//		{
//			if (sgDriverInfo.DoPwmPara[DRIVER11].u16DoFdbVoltage == 0)
//			{
//				sgDriverInfo.DoPwmPara[DRIVER11].u16DoFdbVoltage = 1;
//			}

//			sgDriverInfo.DoPwmPara[DRIVER11].u16DoFdbVoltageSum -= sgDriverInfo.DoPwmPara[DRIVER11].u16DoFdbVoltage;
//		}
//	}
//	else
//	{
//		sgDriverInfo.DoPwmPara[DRIVER11].u16DoFdbVoltageSum += (1 << DoFdbVoltageSHIFT);
//		sgDriverInfo.DoPwmPara[DRIVER11].u16DoFdbVoltage = sgDriverInfo.DoPwmPara[DRIVER11].u16DoFdbVoltageSum >> DoFdbVoltageSHIFT;
//		if (0 == sgDriverInfo.DoPwmPara[DRIVER11].u16DoFdbVoltage)
//		{
//			sgDriverInfo.DoPwmPara[DRIVER11].u16DoFdbVoltage = 1;
//		}
//		sgDriverInfo.DoPwmPara[DRIVER11].u16DoFdbVoltageSum -= sgDriverInfo.DoPwmPara[DRIVER11].u16DoFdbVoltage;
//	}
//	
//	if(0 == READ_DRV12())
//	{
//		sgDriverInfo.DoPwmPara[DRIVER12].u16DoFdbVoltage = sgDriverInfo.DoPwmPara[DRIVER12].u16DoFdbVoltageSum >> DoFdbVoltageSHIFT;
//		if (0 != sgDriverInfo.DoPwmPara[DRIVER12].u16DoFdbVoltageSum)
//		{
//			if (sgDriverInfo.DoPwmPara[DRIVER12].u16DoFdbVoltage == 0)
//			{
//				sgDriverInfo.DoPwmPara[DRIVER12].u16DoFdbVoltage = 1;
//			}

//			sgDriverInfo.DoPwmPara[DRIVER12].u16DoFdbVoltageSum -= sgDriverInfo.DoPwmPara[DRIVER12].u16DoFdbVoltage;
//		}
//	}
//	else
//	{
//		sgDriverInfo.DoPwmPara[DRIVER12].u16DoFdbVoltageSum += (1 << DoFdbVoltageSHIFT);
//		sgDriverInfo.DoPwmPara[DRIVER12].u16DoFdbVoltage = sgDriverInfo.DoPwmPara[DRIVER12].u16DoFdbVoltageSum >> DoFdbVoltageSHIFT;
//		if (0 == sgDriverInfo.DoPwmPara[DRIVER12].u16DoFdbVoltage)
//		{
//			sgDriverInfo.DoPwmPara[DRIVER12].u16DoFdbVoltage = 1;
//		}
//		sgDriverInfo.DoPwmPara[DRIVER12].u16DoFdbVoltageSum -= sgDriverInfo.DoPwmPara[DRIVER12].u16DoFdbVoltage;
//	}
}

/*******************************************************************************
* Name: int16_t i16GetDoFdbVoltage(eDoPwmNo DoPwmNo)
* Descriptio: 获取DoPwm掉线短路状态
* Input: DoPwmNo：对应的通道号
* Output: -1 ：参数失败
* 		  >=0 ： 对应的电压值
*******************************************************************************/
int16_t i16GetDoFdbVoltage(eDoPwmNo DoPwmNo)
{
	if(DoPwmNo >= DoPwmMax)
	{
		i32LogWrite(ERR, LOG_PWM, "GetDoFdbVoltage Parameter is wrong, DoPwmMax = %d, DoPwm = %d\r\n", DoPwmMax, DoPwmNo);
		return -1;
	}
	return sgDriverInfo.DoPwmPara[DoPwmNo].u16DoFdbVoltage;
}
#endif