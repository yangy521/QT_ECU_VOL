
/*******************************************************************************
* Filename: PropProc.c 	                                    	     	       *
* Description: 	PropProc C Source File						           		   *
* Author:  QExpand, Lilu                                                       *
* Date: 2023/6/2															   *
* Revision:															           *
*******************************************************************************/

#include "PropDriver.h"
#include "PropProc.h"
#include "Log.h"
#include "string.h"
#include "Para.h"
#include "WdgProc.h"
#include "NetTimer.h"
#include "ErrCode.h"
#include "LocalDi.h"

#define		PROP_IQ_30MA_VAL		(_IQ12toIQ(0.03 / PROP_CURRENT_FACOTR))		/*lilu 30mA CurrentVal */
#define		PROP_IQ_100MA_VAL		(_IQ12toIQ(0.1 / PROP_CURRENT_FACOTR))		/*lilu 100mA CurrentVal */
#define		PROP_ERR_CNT				20

typedef struct
{
	int32_t i32PropSumCmd;
	int32_t i32PropCmd;
	int32_t i32PropCurrentCmd;
	uint16_t u16PropDelay;
	uint16_t u16PropDDitherPeriod;
	uint8_t u8PropRunFlag;
	uint8_t u8ErrCnt;
	uint16_t u16PropMinADC;
	uint16_t u16PropMaxADC;
}xPropProcCmd;

typedef struct
{
	xPropProcCmd PropProcCmd[PropDriverMax];
	PropErrCallBackt CallBack;
}xPropInfo;

static xPropInfo sgPropInfo;



/*******************************************************************************
* Name: void vPropErrReg(PropErrCallBackt CallBack)
* Descriptio: 设置比例阀错误回调函数
* Input: CallBack：要设置的通道
* Output: NULL  
*******************************************************************************/
void vPropErrReg(PropErrCallBackt CallBack)
{
	sgPropInfo.CallBack = CallBack;
}

/*******************************************************************************
* Name: vSetPropSetTarget(uint8_t u8Channel, int32_t i32Target)
* Descriptio: 设置比例阀对应的目标值
* Input: u8Channel：要设置的通道
*        i32Target：要设置的目标值
* Output: NULL  
*******************************************************************************/
void vPropSetTarget(uint8_t u8Channel, int32_t i32Target)
{
	if(u8Channel >= PropDriverMax)
	{
		i32LogWrite(ERR, LOG_PROP, "SetPropSetTarget Parameter is wrong, PropMax = %d, PropCur = %d\r\n", PropDriverMax, u8Channel);
	}
	else
	{
		if (1 == i32ErrCodeCheck(ErrCode61 + u8Channel))
		{
			sgPropInfo.PropProcCmd[u8Channel].i32PropCurrentCmd = 0;
		}
		else
		{
			sgPropInfo.PropProcCmd[u8Channel].i32PropCurrentCmd = i32Target;
		}
	}
	
//	if (1 == i32ErrCodeCheck(ErrCode61 + u8Channel))
//	{
//		sgPropInfo.PropProcCmd[u8Channel].i32PropCurrentCmd = 0;
//	}
}

/*******************************************************************************
* Name: void vPropProcInit(void)
* Descriptio: 比例阀初始化函数
* Input: NULL
* Output: NULL  
*******************************************************************************/
void vPropProcInit(void)
{
	vPropCurrentLoopInit();
	
	memset(sgPropInfo.PropProcCmd, 0x00, sizeof(sgPropInfo.PropProcCmd));
	
	sgPropInfo.PropProcCmd[PropDriverCh0].u16PropDDitherPeriod = i32GetPara(PARA_PropDDitherPeriod0);
	sgPropInfo.PropProcCmd[PropDriverCh1].u16PropDDitherPeriod = i32GetPara(PARA_PropDDitherPeriod1);
	
	if ((i32GetPara(PARA_PropDMinCurrent0) < 10) && (i32GetPara(PARA_PropDMaxCurrent0) < 10))
	{
		sgPropInfo.PropProcCmd[PropDriverCh0].u16PropMinADC = i32GetPara(PARA_PropDMinCurrent0) / 1000.0 / PROP_CURRENT_FACOTR;
		sgPropInfo.PropProcCmd[PropDriverCh0].u16PropMaxADC = (i32GetPara(PARA_PropDMaxCurrent0) / 1000.0)/ PROP_CURRENT_FACOTR;		/*最大电流增加50ma阈值*/
	}
	else
	{
		sgPropInfo.PropProcCmd[PropDriverCh0].u16PropMinADC = i32GetPara(PARA_PropDMinCurrent0) / 1000.0 / PROP_CURRENT_FACOTR;
		sgPropInfo.PropProcCmd[PropDriverCh0].u16PropMaxADC = (i32GetPara(PARA_PropDMaxCurrent0) / 1000.0 + 0.05)/ PROP_CURRENT_FACOTR;		/*最大电流增加50ma阈值*/
	}
	
	if ((i32GetPara(PARA_PropDMinCurrent1) < 10) && (i32GetPara(PARA_PropDMaxCurrent1) < 10))
	{
		sgPropInfo.PropProcCmd[PropDriverCh1].u16PropMinADC = i32GetPara(PARA_PropDMinCurrent1) / 1000.0 / PROP_CURRENT_FACOTR;
		sgPropInfo.PropProcCmd[PropDriverCh1].u16PropMaxADC = (i32GetPara(PARA_PropDMaxCurrent1) / 1000.0)/ PROP_CURRENT_FACOTR;		/*最大电流增加50ma阈值*/
	}
	else
	{
		sgPropInfo.PropProcCmd[PropDriverCh1].u16PropMinADC = i32GetPara(PARA_PropDMinCurrent1) / 1000.0 / PROP_CURRENT_FACOTR;
		sgPropInfo.PropProcCmd[PropDriverCh1].u16PropMaxADC = (i32GetPara(PARA_PropDMaxCurrent1) / 1000.0 + 0.05)/ PROP_CURRENT_FACOTR;		/*最大电流增加50ma阈值*/
	}
}

///*******************************************************************************
//* Name: uint8_t u8PropGetRunFalg(uint8_t u8Channel)
//* Descriptio: 获取Prop阀运行标志状态
//* Input: u8Channel：通道号
//* Output: NULL  
//*******************************************************************************/
//uint8_t u8PropGetRunFlag(uint8_t u8Channel)
//{
//	return sgPropInfo.PropProcCmd[u8Channel].u8PropRunFlag;
//}
/*******************************************************************************
* Name: void vPropProc(void)
* Descriptio: 比例阀运行函数
* Input: NULL 
* Output: NULL  
*******************************************************************************/
void vPropProc(void)
{
	uint8_t i = 0;
	xPropCurrentLoop *PropTmp;
	/*lilu 20230830 add PropCurrent monitor*/
	{
		i32SetPara(PARA_PropCurrent1, inserted_data[0] * PROP_CURRENT_FACOTR * 1000);
		i32SetPara(PARA_PropCurrent2, inserted_data[1] * PROP_CURRENT_FACOTR * 1000);
		i32SetPara(PARA_ErrCode, u8ErrCodeGet());										/*add errcode*/
	}
	
	for (i=0; i<PropDriverMax; i++)
	{
		uint8_t u8TmpFlag = 0;
#ifdef DoPowerOnCaL_EN
		if ((0 == sgPropInfo.PropProcCmd[i].i32PropCurrentCmd) && (0 == i32LocalDiGet(DRIVER11_R + i)))
		{
			u8TmpFlag |= 1 << 0;
		}
#endif
		
#ifdef DoFdbVoltageCal_EN
			if ((0 != sgPropInfo.PropProcCmd[i].i32PropCurrentCmd) && (abs(sgPropInfo.PropProcCmd[i].i32PropCmd - sgPropInfo.PropProcCmd[i].i32PropCurrentCmd)) <= PROP_IQ_30MA_VAL)
			{
				if ((abs(_IQ12toIQ(inserted_data[i]) - sgPropInfo.PropProcCmd[i].i32PropCmd) > PROP_IQ_100MA_VAL) ||
						(inserted_data[i] < sgPropInfo.PropProcCmd[i].u16PropMinADC) || 
						(inserted_data[i] > sgPropInfo.PropProcCmd[i].u16PropMaxADC))
						{
							sgPropInfo.PropProcCmd[i].u8ErrCnt = PROP_ERR_CNT;
						}
							
					if (sgPropInfo.PropProcCmd[i].u8ErrCnt > 0)
					{
						sgPropInfo.PropProcCmd[i].u8ErrCnt--;
						u8TmpFlag |= 1 << 1;
					}
			}
			else
			{
							sgPropInfo.PropProcCmd[i].i32PropCmd = sgPropInfo.PropProcCmd[i].i32PropCurrentCmd;
			}
#endif
		
//		if ((0 != sgPropInfo.PropProcCmd[i].i32PropCurrentCmd) && ((inserted_data[i] < sgPropInfo.PropProcCmd[i].u16PropMinADC) || (inserted_data[i] > sgPropInfo.PropProcCmd[i].u16PropMaxADC))) 
		if (0 != u8TmpFlag)
		{
			if(false == u8GetNetTimerStartFlag(TIMER_PorpDriver0Check + i))
			{
				vSetNetTimer(TIMER_PorpDriver0Check + i, PROP_LOST_CHK_DELAY_TIME);
			}
			if(true == u8GetNetTimerOverFlag(TIMER_PorpDriver0Check + i))
			{
				if ((sgPropInfo.PropProcCmd[i].u16PropMinADC > 5) || (sgPropInfo.PropProcCmd[i].u16PropMaxADC > 5))
				{
					i32ErrCodeSet(ErrCode66 + i);
					if (NULL != sgPropInfo.CallBack)
					{
						sgPropInfo.CallBack(i);
					}
				}
				
				vKillNetTimer(TIMER_PorpDriver0Check + i);
			}
		}
		else
		{
			if(true == u8GetNetTimerStartFlag(TIMER_PorpDriver0Check + i))
			{
				vKillNetTimer(TIMER_PorpDriver0Check + i);
			}
		}
		
		if (1 == i32ErrCodeCheck(ErrCode61 + i))
		{
			sgPropInfo.PropProcCmd[i].i32PropCurrentCmd = 0;
		}
	}
	vWdgSetFun(WDG_PROP_BIT);
	i32LogWrite(DEBUG, LOG_PROP, "Prop Proc is Running!\r\n");
}

int32_t i32GetPropCmd(uint8_t u8Channel)
{
	return sgPropInfo.PropProcCmd[u8Channel].i32PropCurrentCmd;
}
