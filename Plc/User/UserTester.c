/*******************************************************************************
* Filename: UserStackerTruckProc.c	                                           *
* Description: 逻辑层的源文件，包含对应的回调函数	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/05/30    														   *
* Revision:	 														 		   *
*******************************************************************************/
#include "UserCanComm.h"
#include "UserTest.h"
#include "ErrCode.h"
#include "IQmathLib.h"
#include "PropProc.h"
#include "CanRevProc.h"
#include "string.h"
#include "Para.h"
#include "WdgProc.h"
#include "math.h"
#include "Log.h"
#include "LocalAi.h"
#include "LedProc.h"
#include "AiProc.h"
#include "LocalDo.h"

#if (USER_TYPE == USER_TEST)
	
const static xPdoParameter  PdoPara = 
{
	.TpdoPara = {
		{.u8Flag = 1, .u8Type = 0x0, .u16Period = 200},
		{.u8Flag = 1, .u8Type = 0x0, .u16Period = 200},
		{.u8Flag = 1, .u8Type = 0x0, .u16Period = 200},
		{.u8Flag = 1, .u8Type = 0x0, .u16Period = 200},

		{.u8Flag = 1, .u8Type = 0, .u16Period = 200, .u16CanId = 0x123},
		{.u8Flag = 1, .u8Type = 0, .u16Period = 200, .u16CanId = 0x124},
		{.u8Flag = 1, .u8Type = 0, .u16Period = 200, .u16CanId = 0x125},
		{.u8Flag = 1, .u8Type = 0, .u16Period = 200, .u16CanId = 0x126},
		{.u8Flag = 1, .u8Type = 0, .u16Period = 200, .u16CanId = 0x127},
		{.u8Flag = 1, .u8Type = 0, .u16Period = 200, .u16CanId = 0x128},
	},
	.RpdoPara = {
		{.b1Flag = 0, .b11CanRevId = 0x000},
		{.b1Flag = 0, .b11CanRevId = 0x000},
		{.b1Flag = 0, .b11CanRevId = 0x000},
		{.b1Flag = 0, .b11CanRevId = 0x000},
		{.b1Flag = 0, .b11CanRevId = 0x000},
		{.b1Flag = 0, .b11CanRevId = 0x000},
	},
};

xCanRevPdoInfo gCanRevPdoInfo;				/*PDO接收待完善*/
/*PDO发送待完善*/
xCanSendPdoInfo gCanSendPdoInfo = 
{
	.CanSendInfo1.u16SoftVer = 0x1000,
	.CanSendInfo2.u16HardVer = 0x1000,
};


static uint16_t u16MotorVal = 0;
static uint8_t	u8PumpOrPropValue = 0;

static void vDoPwmErrCallBack(eDoPwmNo DoPwmNo)
{
	switch((uint8_t)DoPwmNo)
	{
		case DRIVER1:
//			i32ErrCodeSet(ErrCode51);
			break;
		case DRIVER2:
//			i32ErrCodeSet(ErrCode52);
			break;
		case DRIVER3:
//			i32ErrCodeSet(ErrCode53);
			break;
		case DRIVER4:
//			i32ErrCodeSet(ErrCode54);
			break;
		case DRIVER5:
//			i32ErrCodeSet(ErrCode55);
			break;
		case DRIVER6:
//			i32ErrCodeSet(ErrCode56);
			break;
		case DRIVER7:
//			i32ErrCodeSet(ErrCode57);
			break;
		case DRIVER8:
//			i32ErrCodeSet(ErrCode58);
			break;
		case DRIVER9:
//			i32ErrCodeSet(ErrCode59);
			break;
		case DRIVER10:
//			i32ErrCodeSet(ErrCode60);
			break;
		default:
			break;
		
	}
}

static void vPropErrCallBack(uint8_t u8Channel)
{
	switch(u8Channel)
	{
		case 0:
//			i32ErrCodeSet(ErrCode61);
			break;
		case 1:
//			i32ErrCodeSet(ErrCode62);
			break;
		
	}
}

static void vAiErrCallBack(eAiErrNo AiErrNo)
{
	switch((uint8_t)AiErrNo)
	{
		case AI_B_AI1_R_ERR:
//			i32ErrCodeSet(ErrCode71);
			break;
		case AI_B_AI2_R_ERR:
//			i32ErrCodeSet(ErrCode72);
			break;
		case AI_B_AI3_R_ERR:
//			i32ErrCodeSet(ErrCode73);
			break;
		case AI_5V_12V_OUT1_I_ERR:
//			i32ErrCodeSet(ErrCode74);
			break;
		case AI_5V_12V_OUT2_I_ERR:
//			i32ErrCodeSet(ErrCode75);
			break;
		default:
			break;
										
	}
}



static void vMstRevProc(xMstRevPara *RevData)
{	
	uint16_t tmp = 0;
	
	if(0 != RevData->b1MainDriver)
	{
		
	}
	else
	{
		
	}
	
	if(0 != RevData->b1Ebrake)
	{
		
	}
	else
	{	
		
	}
	
	if(0 != RevData->b1Driver3State)
	{
		
	}
	else
	{
		
	}
	/*故障码处理*/
	/*故障码处理*/
	if(0 != RevData->u8ErrCode)
	{
		if (u8ErrCodeGet() < RevData->u8ErrCode)   /*lilu, 20230818, 清除MCU之前更小的故障码*/
		{
			uint8_t i = 0;
			for (i=0; i<RevData->u8ErrCode; i++)
			{
				i32ErrCodeClr(i);
			}
			
		}
		i32ErrCodeSet(RevData->u8ErrCode - 1);	
	}
	else
	{
		if (u8ErrCodeGet() < 50)				/*lilu, 20230817, 没有MCU的故障的时候，就清除MCU的所有故障码*/
		{
			uint8_t i = 0;
			for (i=0; i<50; i++)
			{
				i32ErrCodeClr(i);
			}
		}
	}
	
	//i16MotorSpd = (uint16_t)(RevData->u8SpeedFdbHigh << 8) | RevData->u8SpeedFdbLow ;

	
	tmp = RevData->u8MotorTmp;

	
	tmp = RevData->u8BoardTmp;
	/**/
}


/*发送给Mst的回调函数*/
static void vMstSendProc(xMstSendPara *SendData) 
{
	SendData->buf[2] = 0;
	
	if (u16MotorVal > 0)
	{
		if (u16MotorVal >= 4095)
		{
			u16MotorVal = 4095;
		}
		SendData->u8TargetHigh = (u16MotorVal >> 8) & 0xFF;
		SendData->u8TargetLow = u16MotorVal & 0xFF;
		SendData->b1ForwardReq = 1;
	}
	else
	{
		SendData->u8TargetHigh = 0;
		SendData->u8TargetLow = 0;
		SendData->b1ForwardReq = 0;
	}
	
	if (u8PumpOrPropValue > 0)
	{
		if (u8PumpOrPropValue >= 255)
		{
			u8PumpOrPropValue = 255;
		}
		
		SendData->u8PumpTarget = u8PumpOrPropValue;
		SendData->b1LiftReq = 1;
	}
	else
	{
		SendData->u8PumpTarget = 0;
		SendData->b1LiftReq  = 0;
	}
	
	if (1 == SendData->b1LiftReq || 1 == SendData->b1ForwardReq) 
	{
		SendData->b1ServoOn = 1;
	}
}


/*******************************************************************************
* Name: void vUserEcuInit(void)
* Descriptio: UserEcu初始化函数
* Input: NULL
* Output: NULL  
*******************************************************************************/
void vUserEcuInit(void)
{	

	vMstRevRegister(vMstRevProc);
	vMstSendRegister(vMstSendProc);
	
	vDoPwmErrReg(vDoPwmErrCallBack);
	vPropErrReg(vPropErrCallBack);
	vAiErrReg(vAiErrCallBack);
	
	
	vSetPdoPara(PdoPara);
	
	i32LocalDoSet(DO_ENCODER2, 1);
	
	vSetNetTimer(TIMER_EcuPowerOn, 1000);	/**/
	
//	{
//		uint32_t u32Sum = 0;
//		uint32_t i = 0;
//		for (i=0; i<(128 * 1024) >> 2; i++)
//		{
//			u32Sum += *(uint32_t*)(0x08000000 + 4 * i);
//		}
//		u32Sum = u32Sum - *(uint32_t*)(0x08020000);
//		
//		if (0 == u32Sum)
//		{
//			
//		}
//		else
//		{
//			i32LogWrite(INFO, LOG_USER, "Sum = 0x%x\r\n", u32Sum);
//		}
//	}
}

/*******************************************************************************
* Name: void vUserEcuProc(void)
* Descriptio: UserEcu用户处理函数
* Input: NULL
* Output: NULL  
*******************************************************************************/
void vUserEcuProc(void)
{
	static uint8_t u8EcuProcFlag = 0;
	uint8_t u8ErrCode = 0;
	uint8_t i = 0;
	uint32_t u32DiValue = 0;

	if(true == u8GetNetTimerOverFlag(TIMER_EcuPowerOn))
	{
		u8EcuProcFlag = 1;
		vKillNetTimer(TIMER_EcuPowerOn);
	}
	
	if(1 == u8EcuProcFlag)
	{
		for (i=0; i<LocalDiMax; i++)
		{
			if(1 == i32LocalDiGet(i))
			{
				u32DiValue |= 1 << i;
			}
		}
		memcpy(gCanSendPdoInfo.CanSendInfo1.u8Data, &u32DiValue, 4);
		
		if (1 == i32LocalDiGet(SWI7_R) && 1 == i32LocalDiGet(SWI8_R))
		{
			int32_t i32AdcValue = 0;
			i32AdcValue = i32LocalAiGetValue(AI_B_AI1_R);
			if (i32AdcValue <= 500)
			{
				u16MotorVal = 0;
			}
			else if(i32AdcValue < 4500)
			{
				u16MotorVal = (i32AdcValue - 500) * 4096 / 4000;
			}
			else
			{
				u16MotorVal = 4095;
			}
			
			i32AdcValue = i32LocalAiGetValue(AI_B_AI2_R);
			if (i32AdcValue <= 500)
			{
				u8PumpOrPropValue = 0;
			}
			else if(i32AdcValue < 4500)
			{
				u8PumpOrPropValue = (i32AdcValue - 500) * 256 / 4000;
			}
			else
			{
				u8PumpOrPropValue = 255;
			}
			vPropSetTarget(PropDriverCh0, 0);
			vPropSetTarget(PropDriverCh1, 0);
			i32DoPwmSet(DRIVER10, 0);
		}
		else
		{
			u16MotorVal = 0;
			u8PumpOrPropValue = 0;
			for(i=0; i<7; i++) 
			{
				if (1 == i32LocalDiGet(SWI1_R + i))
				{
					i32DoPwmSet(DRIVER4 + i, 1);
					if (i == 0)
					{
						i32DoPwmSet(DRIVER3, 1);
					}
				}
				else
				{
					i32DoPwmSet(DRIVER4 + i, 0);
					if (i == 0)
					{
						i32DoPwmSet(DRIVER3, 0);
					}
				}
			}
			
			if (1 == i32LocalDiGet(SWI8_R))
			{
				uint32_t i32PropValue = 0;
				if (i32LocalAiGet(AI_B_AI1_R) >= 20)
				{
					i32PropValue = _IQ((0.7 * i32LocalAiGet(AI_B_AI1_R) / 4096) / PROPD_STD_CURRENT);
				}
				vPropSetTarget(PropDriverCh0, i32PropValue);
				
				i32PropValue = 0;
				if (i32LocalAiGet(AI_B_AI2_R) >= 20)
				{
					i32PropValue = _IQ((0.7 * i32LocalAiGet(AI_B_AI2_R) / 4096) / PROPD_STD_CURRENT);
				}
				vPropSetTarget(PropDriverCh1, i32PropValue);	
			}
			else
			{
				vPropSetTarget(PropDriverCh0, 0);
				vPropSetTarget(PropDriverCh1, 0);
			}
		}

		{
			__disable_irq();		
			gCanSendPdoInfo.CanSendInfo2.u16Ai1Vol = i32LocalAiGetValue(AI_B_AI1_R);
			gCanSendPdoInfo.CanSendInfo2.u16Ai2Vol = i32LocalAiGetValue(AI_B_AI2_R);
			gCanSendPdoInfo.CanSendInfo2.u16Ai3Vol = i32LocalAiGetValue(AI_B_AI3_R);
			__enable_irq();
			
			__disable_irq();		
			gCanSendPdoInfo.CanSendInfo3.u16KsiVol = i32LocalAiGetValue(AI_B_KSI_CHECK);
			gCanSendPdoInfo.CanSendInfo3.u16VbusVol = i32LocalAiGetValue(AI_B_VBUS_CHECK);
			gCanSendPdoInfo.CanSendInfo3.u16PropCur1 = inserted_data[0] * PROP_CURRENT_FACOTR * 1000;
			gCanSendPdoInfo.CanSendInfo3.u16PropCur2 = inserted_data[1] * PROP_CURRENT_FACOTR * 1000;
			__enable_irq();
			
			
		}		
		//vCanRevPdoProc();
		u8ErrCode = u8ErrCodeGet();
		if (0 != u8ErrCode)
		{
			//gCanSendPdoInfo.CanSend23CInfo.u8ErrCode = u8ErrCode;
			vLedSendAlmCode(u8ErrCode);
			if (u8ErrCode <= 50)
			{
				gCanSendPdoInfo.CanSendInfo1.u8McuErrCode = u8ErrCode;
				gCanSendPdoInfo.CanSendInfo1.u8EcuErrCode = 0;
			}
			else
			{
				gCanSendPdoInfo.CanSendInfo1.u8McuErrCode = 0;
				gCanSendPdoInfo.CanSendInfo1.u8EcuErrCode = u8ErrCode;
			}
		}
		else
		{
				gCanSendPdoInfo.CanSendInfo1.u8McuErrCode = 0;
				gCanSendPdoInfo.CanSendInfo1.u8EcuErrCode = 0;
		}
	}
	
	vWdgSetFun(WDG_USER_BIT);
}
#endif
