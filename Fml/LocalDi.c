/*******************************************************************************
* Filename: LocalDi.c 	                                    	     	       *
* Description: 	LocalDi功能					           				           *
* Author:  QExpand, Lilu                                                       *
* Date: 2023/5/16															   *
* Revision:	V1.00														       *
*******************************************************************************/
#include "LocalDi.h"
#include "Para.h"
#include "Log.h"
#include "Device.h"
#include "WdgProc.h"


typedef struct DI_FILTER{
	uint8_t u8In[LocalDiMax];	   		/*DI输入值*/
	uint8_t u8InNew[LocalDiMax];	 	/*Di最新的变化值*/
	uint8_t u8InOld[LocalDiMax];	 	/*DI上一次的值*/
	uint8_t u8Timer[LocalDiMax];	 	/*DI滤波消抖时间参数*/
}xDiFilter;


static xDiFilter sgDiFilter;		/*DI全局变量*/
//xDiFilter sgDiFilter;		/*DI全局变量*/

/*******************************************************************************
* Name: int32_t i32LocalDiGet(eDiNo DiNo)
* Descriptio: 获取Di通道数据
* Input: DiNo：Di通道编号
* Output:  -1:    输入参数错误
		   其它值：Di通道对应的电平值
*******************************************************************************/
int32_t i32LocalDiGet(eDiNo DiNo)
{
	if(DiNo >= LocalDiMax)
	{
		i32LogWrite(ERR, LOG_DI, "LocalDi Parameter is wrong, LocalDiMax = %d, LocalDi = %d\r\n", LocalDiMax, DiNo);
		return -1;
	}
	return sgDiFilter.u8In[DiNo];
}

/*******************************************************************************
* Name: void vLocalDiProc(void)
* Descriptio: LocalDI运行功能函数
* Input: NULL
* Output:  NULL
*******************************************************************************/
void vLocalDiProc(void)
{
	uint8_t index = 0;
	uint16_t DiValue = 0;
	
	if(0 != READ_SW1())
	{
		sgDiFilter.u8InNew[SWI1_R] = 1;
	}
	else
	{
		sgDiFilter.u8InNew[SWI1_R] = 0;
	}
	
	if(0 != READ_SW2())
	{
		sgDiFilter.u8InNew[SWI2_R] = 1;
	}
	else
	{
		sgDiFilter.u8InNew[SWI2_R] = 0;
	}
	
	if(0 != READ_SW3())
	{
		sgDiFilter.u8InNew[SWI3_R] = 1;
	}
	else
	{
		sgDiFilter.u8InNew[SWI3_R] = 0;
	}
	
	if(0 != READ_SW4())
	{
		sgDiFilter.u8InNew[SWI4_R] = 1;
	}
	else
	{
		sgDiFilter.u8InNew[SWI4_R] = 0;
	}
	
	if(0 != READ_SW5())
	{
		sgDiFilter.u8InNew[SWI5_R] = 1;
	}
	else
	{
		sgDiFilter.u8InNew[SWI5_R] = 0;
	}
	
	if(0 != READ_SW6())
	{
		sgDiFilter.u8InNew[SWI6_R] = 1;
	}
	else
	{
		sgDiFilter.u8InNew[SWI6_R] = 0;
	}
	
	if(0 != READ_SW7())
	{
		sgDiFilter.u8InNew[SWI7_R] = 1;
	}
	else
	{
		sgDiFilter.u8InNew[SWI7_R] = 0;
	}
	
	if(0 != READ_SW8())
	{
		sgDiFilter.u8InNew[SWI8_R] = 1;
	}
	else
	{
		sgDiFilter.u8InNew[SWI8_R] = 0;
	}
	
#if (CTLBOARD_TYPE == _HB6_GD32)
	if(0 != READ_SW9())
	{
		sgDiFilter.u8InNew[SWI9_R] = 1;
	}
	else
	{
		sgDiFilter.u8InNew[SWI9_R] = 0;
	}
	
	if(0 != READ_SW10())
	{
		sgDiFilter.u8InNew[SWI10_R] = 1;
	}
	else
	{
		sgDiFilter.u8InNew[SWI10_R] = 0;
	}
	
	if(0 != READ_SW11())
	{
		sgDiFilter.u8InNew[SWI11_R] = 1;
	}
	else
	{
		sgDiFilter.u8InNew[SWI11_R] = 0;
	}
	
	if(0 != READ_SW12())
	{
		sgDiFilter.u8InNew[SWI12_R] = 1;
	}
	else
	{
		sgDiFilter.u8InNew[SWI12_R] = 0;
	}
#endif //#if (CTLBOARD_TYPE == _HB6_GD32)
	
	if(0 != READ_DRV1())
	{
		sgDiFilter.u8InNew[DRIVER1_R] = 1;
	}
	else
	{
		sgDiFilter.u8InNew[DRIVER1_R] = 0;
	}
	
	if(0 != READ_DRV2())
	{
		sgDiFilter.u8InNew[DRIVER2_R] = 1;
	}
	else
	{
		sgDiFilter.u8InNew[DRIVER2_R] = 0;
	}
	
	if(0 != READ_DRV3())
	{
		sgDiFilter.u8InNew[DRIVER3_R] = 1;
	}
	else
	{
		sgDiFilter.u8InNew[DRIVER3_R] = 0;
	}

	#ifdef DO_HIGH_SIDE_DRIVE_R
	if(0 == READ_DRV4())
	#else		
	if(0 != READ_DRV4())
	#endif	
	{
		sgDiFilter.u8InNew[DRIVER4_R] = 1;
	}
	else
	{
		sgDiFilter.u8InNew[DRIVER4_R] = 0;
	}

	#ifdef DO_HIGH_SIDE_DRIVE_R
	if(0 == READ_DRV5())
	#else		
	if(0 != READ_DRV5())
	#endif		
	{
		sgDiFilter.u8InNew[DRIVER5_R] = 1;
	}
	else
	{
		sgDiFilter.u8InNew[DRIVER5_R] = 0;
	}
	
	#ifdef DO_HIGH_SIDE_DRIVE_R
	if(0 == READ_DRV6())
	#else		
	if(0 != READ_DRV6())
	#endif	
	{
		sgDiFilter.u8InNew[DRIVER6_R] = 1;
	}
	else
	{
		sgDiFilter.u8InNew[DRIVER6_R] = 0;
	}
	
	#ifdef DO_HIGH_SIDE_DRIVE_R
	if(0 == READ_DRV7())
	#else		
	if(0 != READ_DRV7())
	#endif	
	{
		sgDiFilter.u8InNew[DRIVER7_R] = 1;
	}
	else
	{
		sgDiFilter.u8InNew[DRIVER7_R] = 0;
	}
	
	#ifdef DO_HIGH_SIDE_DRIVE_R
	if(0 == READ_DRV8())
	#else		
	if(0 != READ_DRV8())
	#endif	
	{
		sgDiFilter.u8InNew[DRIVER8_R] = 1;
	}
	else
	{
		sgDiFilter.u8InNew[DRIVER8_R] = 0;
	}
	
	#ifdef DO_HIGH_SIDE_DRIVE_R
	if(0 == READ_DRV9())
	#else		
	if(0 != READ_DRV9())
	#endif	
	{
		sgDiFilter.u8InNew[DRIVER9_R] = 1;
	}
	else
	{
		sgDiFilter.u8InNew[DRIVER9_R] = 0;
	}
	
	if(0 != READ_DRV10())
	{
		sgDiFilter.u8InNew[DRIVER10_R] = 1;
	}
	else
	{
		sgDiFilter.u8InNew[DRIVER10_R] = 0;
	}
	
	if(0 != READ_DRV11())
	{
		sgDiFilter.u8InNew[DRIVER11_R] = 1;
	}
	else
	{
		sgDiFilter.u8InNew[DRIVER11_R] = 0;
	}
	
	if(0 != READ_DRV12())
	{
		sgDiFilter.u8InNew[DRIVER12_R] = 1;
	}
	else
	{
		sgDiFilter.u8InNew[DRIVER12_R] = 0;
	}
	
	for(index = 0; index < LocalDiMax; index++)
	{
		if (sgDiFilter.u8InNew[index] != sgDiFilter.u8In[index])
		{
			if (sgDiFilter.u8InNew[index] == sgDiFilter.u8InOld[index])
			{
				if (sgDiFilter.u8Timer[index] == 0)
				{
					sgDiFilter.u8In[index] = sgDiFilter.u8InNew[index];
				} 
				else 
				{
					sgDiFilter.u8Timer[index]--;
				}
			} 
			else 
			{
				sgDiFilter.u8InOld[index] = sgDiFilter.u8InNew[index];
				sgDiFilter.u8Timer[index] = DI_FILTER_CONSTANT;
			}
		} 
		else 
		{
			sgDiFilter.u8InOld[index] = sgDiFilter.u8In[index];
		}
	}
	/*lilu 20238030 add Swi Monitor*/
	DiValue = 0;
	for (index = 0; index < DRIVER1_R; index++)
	{
		if(1 == sgDiFilter.u8In[index])
		{
			DiValue |= 1 << index;
		}
	}
	i32SetPara(PARA_Swi, DiValue);
	
	DiValue = 0;
	for (index = DRIVER1_R; index < LocalDiMax; index++)
	{
		if(1 == sgDiFilter.u8In[index])
		{
			DiValue |= 1 << (index - DRIVER1_R);
		}
	}
	i32SetPara(PARA_DoSwi, DiValue);
	
	
	vWdgSetFun(WDG_LOCALDI_BIT);
}