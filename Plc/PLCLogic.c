#include "Device.h"
#include "CommonRam.h"
#include "math.h"
#include  "iCAN.h"
#include "FDB.h"
#include  "iTimer.h"
#include  "AutoTune.h"
#include "CanOpen_QexpandAgvV04.h"
#include "PLCHardware.h"

void LocalDO(void)
{
	if(gPLCCtl.doDataOut[LED_Y])	
		LED_Y_ON();
	else 
		LED_Y_OFF();
		
	if(gPLCCtl.doDataOut[LED_R])	
		LED_R_ON();
	else 
		LED_R_OFF();

	gPara.Driver = ((gPLCCtl.doDataOut[RELAY] << 0)
								 |(gPLCCtl.doDataOut[LOCK_OUT] << 1) 
								 |(gPLCCtl.doDataOut[SOUT1] << 2)
								 |(gPLCCtl.doDataOut[SOUT2] << 3));
}

void LocalDI(void)
{
	INT8U index;
	
	if(READ_SW1()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[SWI1_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[SWI1_R]=0;
	}
	
	if(READ_SW2()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[SWI2_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[SWI2_R]=0;
	}
	
	if(READ_SW3()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[SWI3_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[SWI3_R]=0;
	}
	
	if(READ_SW4()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[SWI4_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[SWI4_R]=0;
	}
	
	if(READ_SW5()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[SWI5_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[SWI5_R]=0;
	}
	
	if(READ_SW6()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[SWI6_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[SWI6_R]=0;
	}
	
	if(READ_SW7()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[SWI7_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[SWI7_R]=0;
	}
	
	if(READ_SW8()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[SWI8_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[SWI8_R]=0;
	}
	if(READ_SW7()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[SWI7_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[SWI7_R]=0;
	}
	
	if(READ_SW8()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[SWI8_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[SWI8_R]=0;
	}	
	
	if(READ_SW9()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[SWI9_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[SWI9_R]=0;
	}
	
	if(READ_SW10()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[SWI10_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[SWI10_R]=0;
	}	
	
	if(READ_SW11()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[SWI11_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[SWI11_R]=0;
	}	
	
	if(READ_SW12()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[SWI12_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[SWI12_R]=0;
	}	
	
	if(READ_DRV1()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[DRIVER1_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[DRIVER1_R]=0;
	}	
	
	if(READ_DRV2()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[DRIVER2_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[DRIVER2_R]=0;
	}	
	
	if(READ_DRV3()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[DRIVER3_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[DRIVER3_R]=0;
	}	
	
	if(READ_DRV4()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[DRIVER4_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[DRIVER4_R]=0;
	}	
	
	if(READ_DRV5()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[DRIVER5_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[DRIVER5_R]=0;
	}	

	for (index=0; index < (sizeof(gPLCCtl.diDataIn.ucIn)/sizeof(gPLCCtl.diDataIn.ucIn[0])); index++)
	{
		if (gPLCCtl.diDataIn.ucInNew[index] != gPLCCtl.diDataIn.ucIn[index])
		{
			if (gPLCCtl.diDataIn.ucInNew[index] == gPLCCtl.diDataIn.ucInOld[index])
			{
				if (gPLCCtl.diDataIn.ucTimer[index] == 0)
				{
					gPLCCtl.diDataIn.ucIn[index] = gPLCCtl.diDataIn.ucInNew[index];
				} 
				else 
				{
					gPLCCtl.diDataIn.ucTimer[index]--;
				}
			} 
			else 
			{
				gPLCCtl.diDataIn.ucInOld[index] = gPLCCtl.diDataIn.ucInNew[index];
				gPLCCtl.diDataIn.ucTimer[index] = DI_FILTER_CONSTANT;
			}
		} 
		else 
		{
			gPLCCtl.diDataIn.ucInOld[index] = gPLCCtl.diDataIn.ucIn[index];
		}
	}	

}
