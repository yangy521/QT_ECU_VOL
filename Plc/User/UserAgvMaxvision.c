/*******************************************************************************
* Filename: UserStackerTruckProc.c	                                           *
* Description: 逻辑层的源文件，包含对应的回调函数	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/05/30    														   *
* Revision:	 														 		   *
*******************************************************************************/
#include "UserCanComm.h"
#include "UserAgvMaxvision.h"
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

#if (USER_TYPE == USER_MAXVISION_AGV)


const static xPdoParameter  PdoPara = 
{
	.TpdoPara = {
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 16},
		{.u8Flag = 1, .u8Type = 0x00, .u16Period = 200},
		{.u8Flag = 1, .u8Type = 0x00, .u16Period = 200},
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 160},

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
	.CanSendInfo1.b1OutAgvMode = 1,
	.CanSendInfo4.u8SwVer = 0x03,
};



static void vCanRevPdoProc(void)
{
	
	static xCanRevInfo1 sgCanRevInfo1;
	static xCanRevInfo2 sgCanRevInfo2;
	
	if (0 != memcmp(sgCanRevInfo1.u8Data, gCanRevPdoInfo.CanRevInfo1.u8Data, sizeof(sgCanRevInfo1)))
	{
		memcpy(sgCanRevInfo1.u8Data, gCanRevPdoInfo.u8RevData1, sizeof(sgCanRevInfo1));
	}
	
	if (0 != memcmp(sgCanRevInfo2.u8Data, gCanRevPdoInfo.CanRevInfo2.u8Data, sizeof(sgCanRevInfo2)))
	{
		memcpy(sgCanRevInfo2.u8Data, gCanRevPdoInfo.u8RevData2, sizeof(sgCanRevInfo2));
	}	
}


//static void vDoPwmErrCallBack(eDoPwmNo DoPwmNo)
//{
//	switch((uint8_t)DoPwmNo)
//	{
//		case DRIVER4:
//			i32ErrCodeSet(ErrCode54);
//			break;
//		case DRIVER5:
//			i32ErrCodeSet(ErrCode55);
//			break;
//		case DRIVER6:
//			i32ErrCodeSet(ErrCode56);
//			break;
//		case DRIVER7:
//			i32ErrCodeSet(ErrCode57);
//			break;
//		case DRIVER8:
//			i32ErrCodeSet(ErrCode58);
//			break;
//		case DRIVER9:
//			i32ErrCodeSet(ErrCode59);
//			break;
//		case DRIVER10:
//			i32ErrCodeSet(ErrCode60);
//			break;
//		default:
//			break;
//		
//	}
//}

//static void vPropErrCallBack(uint8_t u8Channel)
//{
//	switch(u8Channel)
//	{
//		case 0:
//			i32ErrCodeSet(ErrCode61);
//			break;
//		case 1:
//			i32ErrCodeSet(ErrCode62);
//			break;	
//	}
//}

static void vAiErrCallBack(eAiErrNo AiErrNo)
{
	switch((uint8_t)AiErrNo)
	{
		case AI_B_AI1_R_ERR:
			i32ErrCodeSet(ErrCode71);
			break;
		case AI_B_AI2_R_ERR:
			i32ErrCodeSet(ErrCode72);
			break;
		case AI_B_AI3_R_ERR:
			i32ErrCodeSet(ErrCode73);
			break;
		case AI_5V_12V_OUT1_I_ERR:
			i32ErrCodeSet(ErrCode74);
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
	if(0 != RevData->u8ErrCode)
	{
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
	
	gCanSendPdoInfo.CanSendInfo1.b1ServoReady = RevData->b1SvonState;
	gCanSendPdoInfo.CanSendInfo1.b1OutMcStatus = RevData->b1MainDriver;
	gCanSendPdoInfo.CanSendInfo1.b1OutEbStatus = RevData->b1Ebrake;
	gCanSendPdoInfo.CanSendInfo1.b1SpdMode = 1;
	
	tmp = (uint16_t)(RevData->u8SpeedFdbHigh << 8) | RevData->u8SpeedFdbLow ;
	gCanSendPdoInfo.CanSendInfo1.i16ActualSpd = tmp;
	
	tmp = RevData->u8MotorTmp;
	gCanSendPdoInfo.CanSendInfo4.u8MotorTemp = tmp;
	
	tmp = RevData->u8BoardTmp;
	gCanSendPdoInfo.CanSendInfo4.u8BoardTemp = tmp;
	
	tmp = (uint16_t)(RevData->u8CurrentHigh << 8) | RevData->u8CurrentLow ;
	gCanSendPdoInfo.CanSendInfo1.u16AcCurrent = tmp;
	
	/**/
}


/*发送给Mst的回调函数*/
static void vMstSendProc(xMstSendPara *SendData) 
{
	uint16_t tmp = 0;
	if (1 == gCanRevPdoInfo.CanRevInfo1.b1SpdModeReq)
	{
		SendData->b1ServoOn = gCanRevPdoInfo.CanRevInfo1.b1ServeOn;
		SendData->b1ForwardReq = gCanRevPdoInfo.CanRevInfo1.b1ForWardReq;
		SendData->b1BackwardReq = gCanRevPdoInfo.CanRevInfo1.b1BackWardReq;

		tmp = gCanRevPdoInfo.CanRevInfo1.i16TargetSpd * MOTOR_SPEED_RANGE / MOTOR_CMD_RANGE;
		
		if (tmp >= (MOTOR_SPEED_RANGE - 1))
		{
			tmp = (MOTOR_SPEED_RANGE - 1);
		}
//		SendData->u8TargetHigh = gCanRevPdoInfo.CanRevInfo1.i16TargetSpd >> 8;
//		SendData->u8TargetLow = gCanRevPdoInfo.CanRevInfo1.i16TargetSpd & 0xFF;
		SendData->u8TargetHigh = (tmp >> 8) & 0xFF;
		SendData->u8TargetLow = tmp & 0xFF;
	}
	else
	{
		SendData->b1ServoOn = 0;
		SendData->b1ForwardReq = 0;
		SendData->b1BackwardReq = 0;
		SendData->u8TargetHigh = 0;
		SendData->u8TargetLow = 0;
	}
//	//i32LogWrite(INFO, "ServoOn = %d, For = %d, Back = %d, Speed = %d\r\n", \
//	SendData->b1ServoOn, SendData->b1ForwardReq, SendData->b1BackwardReq, (SendData->u8TargetHigh << 8) | SendData->u8TargetLow);
}

/*lilu 20231222 按照客户需求，增加CanID 208 can报文丢失处理*/
static void vCanLostProc(uint32_t u32CanID,uint8_t u8State)
{
	if (0x208 == u32CanID)
	{
		if (CAN_LOST == u8State)
		{
			memset(gCanRevPdoInfo.u8RevData1, 0x00, sizeof(xCanRevInfo1));
		}
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
	/*lilu 20231222 按照客户需求，增加CanID 208 can报文丢失处理*/
	vCanIdLostReg(0x208, 500, vCanLostProc);
	
	
//	vDoPwmErrReg(vDoPwmErrCallBack);
//	vPropErrReg(vPropErrCallBack);
	vAiErrReg(vAiErrCallBack);
	
	vSetPdoPara(PdoPara);
	
//	i32LocalDoSet(DO_ENCODER2, 1);
	
	vSetNetTimer(TIMER_EcuPowerOn, 1000);	/**/
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
		vCanRevPdoProc();
		
		gCanSendPdoInfo.CanSendInfo1.b1InSwiD1 = i32LocalDiGet(SWI1_R);
		gCanSendPdoInfo.CanSendInfo1.b1InSwiD2 = i32LocalDiGet(SWI2_R);
		gCanSendPdoInfo.CanSendInfo1.b1InSwiD3 = i32LocalDiGet(SWI3_R);
		gCanSendPdoInfo.CanSendInfo1.b1InSwiD4 = i32LocalDiGet(SWI4_R);
		gCanSendPdoInfo.CanSendInfo1.b1InSwiD5 = i32LocalDiGet(SWI5_R);
		gCanSendPdoInfo.CanSendInfo1.b1InSwiD6 = i32LocalDiGet(SWI6_R);
		gCanSendPdoInfo.CanSendInfo1.b1InSwiD7 = i32LocalDiGet(SWI7_R);
		gCanSendPdoInfo.CanSendInfo1.b1InSwiD8 = i32LocalDiGet(SWI8_R);
		
		u8ErrCode = u8ErrCodeGet();
		if (0 != u8ErrCode)
		{
			gCanSendPdoInfo.CanSendInfo4.u8ErrCode = u8ErrCode;
			vLedSendAlmCode(u8ErrCode);
//			//i32LogWrite(INFO, "ErrCode is %d!!!\r\n", u8ErrCode);
		}
		gCanSendPdoInfo.CanSendInfo4.u8Ai1 = i32LocalAiGetValue(AI_B_AI1_R) / 100;
		gCanSendPdoInfo.CanSendInfo4.u8Ai2 = i32LocalAiGetValue(AI_B_AI2_R) / 100;
		
	}
	
	vWdgSetFun(WDG_USER_BIT);
}
#endif
