/*******************************************************************************
* Filename: PcuProc.c 	                                    	     	       *
* Description: 	PCU通信处理						           				       *
* Author:  QExpand, Lilu                                                       *
* Date: 2023/5/12															   *
* Revision:															           *
*******************************************************************************/

#include "UartComm.h"
#include "string.h"
#include "PcuProc.h"
#include "WdgProc.h"
#include "ErrCode.h"
#include "LocalDi.h"
#include "Para.h"
#include "Log.h"
#include "NetTimer.h"
#include "LocalDo.h"

#if (PCU_TYPE_NONE != PCU_TYPE)

#define		BMS_SOC_PER_80		200
#define		BMS_SOC_PER_65		162
#define		BMS_SOC_PER_50		125
#define		BMS_SOC_PER_30		75	
#define		BMS_SOC_PER_15		37

//static xPcuRevParat sgPcuRevPara;/*接收PCU的全局变量*/
static const uint8_t u8SigList[2][16] = 
{
	/*0,   1,   2,   3,   4,   5,   6,   7,   8,   9,		A, 	B,		C,		D,	E,	F,*/
	{0x3, 0x0, 0x5, 0x4, 0x6, 0x6, 0x7, 0x0, 0x7, 0x6, 0x7, 0x7, 0x3, 0x5, 0x7, 0x7},
	{0xF, 0x6, 0xB, 0xF, 0x6, 0xD, 0xD, 0x7, 0xF, 0xF, 0x7, 0xC, 0x9, 0xE, 0x9,	0x1},
};

//static xPcuRevParat sgPcuRevPara;/*接收PCU的全局变量*/
static const uint8_t u8BatteryList[6][4] = 
{
	{0x4, 0x9, 0x4, 0x9},	/*≡≡  6*/
	{0x4, 0x8, 0x4, 0x9},	/*5*/
	{0x0, 0x8, 0x4, 0x9},	/*4*/
	{0x0, 0x0, 0x4, 0x9},	/*3*/
	{0x0, 0x0, 0x4, 0x8},	/*2*/
	{0x0, 0x0, 0x0, 0x8},	/*1*/
};

static xPcuProc sgPcuProc;

static uint8_t sgU8PcuFlag = 0;

#if (PCU_TYPE_LZ == PCU_TYPE)
/*接收PCU发送的校验和函数*/
static uint8_t u8PcuGetCheckSum(uint8_t *u8Src, uint16_t u16Len)
{
	uint8_t res = 0;
	uint16_t i = 0;
	uint16_t x = 0;
	for (i=1; i<u16Len; i++)
	{
		x ^= u8Src[i];
		x <<= 1;
		if(x > 0x7F)
		{
			x ^= 0x135;
		}
	}
	
	res = x ^ u8Src[0];
	
	return res;
}
#elif (PCU_TYPE_2 == PCU_TYPE)
/*接收PCU发送的校验和函数*/
static uint8_t u8PcuGetCheckSum(uint8_t *u8Src, uint16_t u16Len)
{
	uint8_t res = 0;
	uint16_t i = 0;
	for(i = 0; i < u16Len; i++)
	{
		res += u8Src[i];
	}
	return res;
}
#elif	(PCU_TYPE_XUGONG == PCU_TYPE)
/*接收PCU发送的校验和函数*/
static uint8_t u8PcuGetCheckSum(uint8_t *u8Src, uint16_t u16Len)
{
	uint16_t u16CRCtmp = 0;
	uint16_t ix;

	for(ix = 0; ix < (u16Len -1) ;ix ++)
	{
		u16CRCtmp ^= (uint16_t)u8Src[ix];
		u16CRCtmp <<= 1;
		if( u16CRCtmp > 0xFF)
		{
			u16CRCtmp ^= 0x119;
		}
	}
	u16CRCtmp ^= u8Src[ix];
	return ((uint8_t)u16CRCtmp);
}
#endif

/*给PCU发送0xA5，查看PCU在线*/
static void vPcuStart(void)
{
	uint8_t u8SendData = PCU_START_FRAME;
	i32UartWrite(Uart0, &u8SendData, 1);
	if(NULL != sgPcuProc.PcuErrCallBack)
	{
		sgPcuProc.PcuErrCallBack(PCU_Init);
	}
}


/*接收PCU发送的信息*/
static uint8_t u8PcuReadyProc(void)
{
	int length = 0;
	int i = 0;
	uint8_t u8tmp[8];
	uint8_t res = PCU_STATE_FAILED;
	
	length = i32UartRead(Uart0, u8tmp, 8);
	if(length >=2 )
	{
		for(i = 0; i < length - 1; i++)
		{
			if((PCU_READY_BYTE1 == u8tmp[i]) && (PCU_READY_BYTE2 == u8tmp[i + 1]))
			{
				res = PCU_STATE_SUCCESS;
				i32LogWrite(INFO, LOG_PCU, "PCU_STATE_SUCCESS\r\n");
				if(false == u8GetNetTimerStartFlag(TIMER_PcuComm))
				{
					vSetNetTimer(TIMER_PcuComm, PCU_TIMERCOM_PRIOD);
				}
				break;
			}				
		}
	}
	return res;
}

/*给PCU发送0x1A，查看PCU的状态*/
static void vPcuQuery(void)
{
	uint8_t u8SendData = PCU_QUERY_FRAME;
	i32UartWrite(Uart0, &u8SendData, 1);

}

/*接收PCU发送的信息*/
static uint8_t u8PcuRevProc(void)
{
	int length = 0;
	uint8_t u8tmp[32];
	uint8_t *pTmp = u8tmp;
	uint8_t u8CheckSum = 0;
	uint8_t res = PCU_STATE_FAILED;
	
	static uint8_t u8RevCnt = 0;
	static uint8_t u8PcuRevBuf[8];
	static uint8_t u8RxState = 0;
	
	length = i32UartRead(Uart0, u8tmp, 32);
	if(length > 0)
	{
		while(length)
		{
			
			switch (u8RxState)
			{
				case WAIT_HEAD:
					if(PCU_HEAD == *pTmp++)
					{
						u8PcuRevBuf[u8RevCnt++] = PCU_HEAD;
						u8RxState = WAIT_DATA;
					}
					else
					{
						u8RxState = WAIT_HEAD;
						u8RevCnt = 0;
					}
					break;
				case WAIT_DATA:
					if(u8RevCnt < 7)
					{
						u8PcuRevBuf[u8RevCnt++] = *pTmp++;
						u8RxState = WAIT_DATA;
					}
					else
					{
						u8PcuRevBuf[u8RevCnt] = *pTmp++;
						u8RxState = WAIT_CHECKSUM;
					}
					break;
				case WAIT_CHECKSUM:
					u8CheckSum = u8PcuGetCheckSum(u8PcuRevBuf + 1, 7);
					if(*pTmp++ == u8CheckSum)
					{
						//memcpy(&sgPcuRevPara, u8PcuRevBuf, 8);
						memcpy(&sgPcuProc.PcuRevData, u8PcuRevBuf, 8);
						res = PCU_STATE_SUCCESS;
						/*add user resolve pcu data*/
						if(NULL != sgPcuProc.PcuRevCallBack)
						{
							sgPcuProc.PcuRevCallBack(&sgPcuProc.PcuRevData);
						}
						u8RxState = WAIT_HEAD;
						u8RevCnt = 0;
					}
					else	
					{
						u8RxState = WAIT_HEAD;
						u8RevCnt = 0;
					}
					break;
				default:
					u8RxState = WAIT_HEAD;
					break;	
			}
			length--;
		}
	}
	return res;
}

/*给PCU发送ECU的信息*/
static void vPcuSend(xPcuSendPara *SendData)
{
	SendData->Data.b4Const1 = 0xD;
	SendData->Data.b4Const2 = 0xD;
	SendData->Data.b4Const3 = 0xD;
	SendData->Data.b4Const4 = 0xD;
	SendData->Data.b4Const5 = 0xD;
	SendData->Data.b4Const6 = 0x8;
	SendData->Data.b4Const7 = 0x9;
	SendData->Data.b4Const8 = 0xB;
	

	
	/*Get ErrCode*/
	uint8_t u8ErrCode = u8ErrCodeGetTrans();//23.11.21 SJ PCU 发送转换后的故障码

	if(0 == u8ErrCode)
	{
		/*Get Battery Capacity*/
		uint16_t u16Battery;
		u16Battery = i32GetPara(PARA_BmsSoc);
		if (u16Battery >= BMS_SOC_PER_80)
		{
			SendData->Data.b4ErrCodeSigLHigh4 = u8BatteryList[0][0];
			SendData->Data.b4ErrCodeSigLLow4 = u8BatteryList[0][1];
			SendData->Data.b4ErrCodeSigRHigh4 = u8BatteryList[0][2];
			SendData->Data.b4ErrCodeSigRLow4 = u8BatteryList[0][3];
		}
		else if (u16Battery >= BMS_SOC_PER_65)
		{
			SendData->Data.b4ErrCodeSigLHigh4 = u8BatteryList[1][0];
			SendData->Data.b4ErrCodeSigLLow4 = u8BatteryList[1][1];
			SendData->Data.b4ErrCodeSigRHigh4 = u8BatteryList[1][2];
			SendData->Data.b4ErrCodeSigRLow4 = u8BatteryList[1][3];
		}
		else if (u16Battery >= BMS_SOC_PER_50)
		{
			SendData->Data.b4ErrCodeSigLHigh4 = u8BatteryList[2][0];
			SendData->Data.b4ErrCodeSigLLow4 = u8BatteryList[2][1];
			SendData->Data.b4ErrCodeSigRHigh4 = u8BatteryList[2][2];
			SendData->Data.b4ErrCodeSigRLow4 = u8BatteryList[2][3];
		}
		else if (u16Battery >= BMS_SOC_PER_30)
		{
			SendData->Data.b4ErrCodeSigLHigh4 = u8BatteryList[3][0];
			SendData->Data.b4ErrCodeSigLLow4 = u8BatteryList[3][1];
			SendData->Data.b4ErrCodeSigRHigh4 = u8BatteryList[3][2];
			SendData->Data.b4ErrCodeSigRLow4 = u8BatteryList[3][3];
		}
		else if (u16Battery >= BMS_SOC_PER_15)
		{
			SendData->Data.b4ErrCodeSigLHigh4 = u8BatteryList[4][0];
			SendData->Data.b4ErrCodeSigLLow4 = u8BatteryList[4][1];
			SendData->Data.b4ErrCodeSigRHigh4 = u8BatteryList[4][2];
			SendData->Data.b4ErrCodeSigRLow4 = u8BatteryList[4][3];
		}
		else
		{
			SendData->Data.b4ErrCodeSigLHigh4 = u8BatteryList[5][0];
			SendData->Data.b4ErrCodeSigLLow4 = u8BatteryList[5][1];
			SendData->Data.b4ErrCodeSigRHigh4 = u8BatteryList[5][2];
			SendData->Data.b4ErrCodeSigRLow4 = u8BatteryList[5][3];
		}
		/*≡≡*/
//		SendData->Data.b4ErrCodeSigLHigh4 = 0x4;
//		SendData->Data.b4ErrCodeSigLLow4 = 0x9;
//		SendData->Data.b4ErrCodeSigRHigh4 = 0x4;
//		SendData->Data.b4ErrCodeSigRLow4 = 0x9;
	}
	else if(u8ErrCode < 100)
	{
		/*0~9*/
		SendData->Data.b4ErrCodeSigLHigh4 = u8SigList[0][(u8ErrCode / 10)];
		SendData->Data.b4ErrCodeSigLLow4 = u8SigList[1][(u8ErrCode / 10)];
		SendData->Data.b4ErrCodeSigRHigh4 = u8SigList[0][(u8ErrCode % 10)];
		SendData->Data.b4ErrCodeSigRLow4 = u8SigList[1][(u8ErrCode % 10)];
	}
	else if(100 == u8ErrCode)
	{
		/*OL*/
		SendData->Data.b4ErrCodeSigLHigh4 = u8SigList[0][0];
		SendData->Data.b4ErrCodeSigLLow4 = u8SigList[1][0];
		SendData->Data.b4ErrCodeSigRHigh4 = 0x3;
		SendData->Data.b4ErrCodeSigRLow4 = 0x8;
	}
	else
	{
		/*LL*/
		SendData->Data.b4ErrCodeSigLHigh4 = 0x3;
		SendData->Data.b4ErrCodeSigLLow4 = 0x8;
		SendData->Data.b4ErrCodeSigRHigh4 = 0x3;
		SendData->Data.b4ErrCodeSigRLow4 = 0x8;
	}
	if (NULL != sgPcuProc.PcuSendCallBack)
	{
		sgPcuProc.PcuSendCallBack(SendData);
	}
	/*建立通讯后的半秒钟内，如果0110，说明ECU给PCU发的模式是举升模式，否则为行走模式*/
	if(false == u8GetNetTimerOverFlag(TIMER_PcuComm) && (true == u8GetNetTimerStartFlag(TIMER_PcuComm)))
	{
		SendData->Data.b1SlowLed = 0;
		{//此部分为原来对乐瞻手柄协议处理
			#if(PCU_TYPE == PCU_TYPE_LZ)
			//SendData->Data.b3Reserve4 = 3;
			#else
			SendData->Data.b3Reserve4 = 0;
			#endif
		//23.12.29取消上电后模式通讯。
			SendData->Data.b3Reserve4 = 0;
		}
		if(1 == sgPcuProc.PcuRevData.Data.b1LiftingSwitch) 
		{
			if(NULL != sgPcuProc.PcuErrCallBack)
			{
				sgPcuProc.PcuErrCallBack(PCU_LiftKeyPress);
			}
		}
		
		if(1 == sgPcuProc.PcuRevData.Data.b1SlowSpdSwitch)
		{
			if(NULL != sgPcuProc.PcuErrCallBack)
			{
				sgPcuProc.PcuErrCallBack(PCU_SlowKeyPress);
			}
		}
		
		if(1 == sgPcuProc.PcuRevData.Data.b1TraSwitch)
		{
			if(NULL != sgPcuProc.PcuErrCallBack)
			{
				sgPcuProc.PcuErrCallBack(PCU_MoveKeyPress);
			}
		}
		
		if(1 == sgPcuProc.PcuRevData.Data.b1TurnLeftSwitch)
		{
			if(NULL != sgPcuProc.PcuErrCallBack)
			{
				sgPcuProc.PcuErrCallBack(PCU_TurnLeftPress);
			}
		}
		
		if(1 == sgPcuProc.PcuRevData.Data.b1TurnRightSwitch)
		{
			if(NULL != sgPcuProc.PcuErrCallBack)
			{
				sgPcuProc.PcuErrCallBack(PCU_TurnRightPress);
			}
		}
		
		if(1 == sgPcuProc.PcuRevData.Data.b1EnableSwitch)
		{
			if(NULL != sgPcuProc.PcuErrCallBack)
			{
				sgPcuProc.PcuErrCallBack(PCU_EnableKeyPress);
			}
		}
		
		if(1 == sgPcuProc.PcuRevData.Data.b1SpeakerSwitch)//23.11.21 SJ 添加上电后喇叭按键按下反馈
		{
			if(NULL != sgPcuProc.PcuErrCallBack)
			{
				sgPcuProc.PcuErrCallBack(PCU_SpeakerPress);
			}
		}
		
		if((0 != sgPcuProc.PcuRevData.Data.b4HandleCtrlLow) || ( 0 != sgPcuProc.PcuRevData.Data.b4HandleCtrlHigh))
		{
			if(NULL != sgPcuProc.PcuErrCallBack)
			{
				sgPcuProc.PcuErrCallBack(PCU_ValueNoZero);
			}
		}
		
//				/*≡≡*/
//		SendData->Data.b4ErrCodeSigLHigh4 = 0x4;
//		SendData->Data.b4ErrCodeSigLLow4 = 0x9;
//		SendData->Data.b4ErrCodeSigRHigh4 = 0x4;
//		SendData->Data.b4ErrCodeSigRLow4 = 0x9;
	}
	else
	{
		SendData->Data.b3Reserve4 = 0;
		if(true == u8GetNetTimerStartFlag(TIMER_PcuComm))
		{
			vKillNetTimer(TIMER_PcuComm);
		}
	}
	//xSend->Data.b1SlowLed = i32LocalDiGet(DRIVER1_R) | i32LocalDiGet(DRIVER2_R);
	//xSend->Data.b1ModeLed = i32LocalDiGet(DRIVER2_R);
	//xSend->Data.b1LiftLed = i32LocalDiGet(DRIVER1_R) | i32LocalDiGet(DRIVER2_R);
	//xSend->Data.b1Beep = i32LocalDiGet();
	
	i32UartWrite(Uart0, (uint8_t*)SendData, sizeof(xPcuSendPara));	
}


/*******************************************************************************
* Name: vPcuErrRegister(PcuCallBackt CallBack)
* Descriptio: Pcu Err callBack
* Input: CallBack：回调函数
* Output: NULL  
*******************************************************************************/
void vPcuErrRegister(PcuErrCallBackt CallBack)
{
	sgPcuProc.PcuErrCallBack = CallBack;
}

/*******************************************************************************
* Name: vPcuRevRegister(PcuCallBackt CallBack)
* Descriptio: Pcu接收处理注册回调函数
* Input: CallBack：回调函数
* Output: NULL  
*******************************************************************************/
void vPcuRevRegister(PcuRevCallBackt CallBack)
{
	sgPcuProc.PcuRevCallBack = CallBack;
	sgU8PcuFlag = (i32GetPara(PARA_ValveType) >> 14) & 0x1;			/*lilu 20230925 add Mode*/
}

/*******************************************************************************
* Name: vPcuSendRegister(PcuCallBackt CallBack)
* Descriptio: Pcu接收处理注册回调函数
* Input: CallBack：回调函数
* Output: NULL  
*******************************************************************************/
void vPcuSendRegister(PcuSendCallBackt CallBack)
{
	sgPcuProc.PcuSendCallBack = CallBack;
}

/*******************************************************************************
* Name: void vPcuProc(void)
* Descriptio: Pcu通信处理
* Input: NULL
* Output: NULL  
*******************************************************************************/
void vPcuProc(void)
{
//#ifdef PCU_TYPE
	if (0 != sgU8PcuFlag)	
	{
		static uint8_t u8PcuState = 0;			/*state machine */
		static uint8_t u8PcuTimerCnt = 0;		/*for 50ms cnt*/
		static uint8_t u8PcuQueryCnt = 0;		/*For Send 3 times 0x1A, No response, switch to PCU_Start*/
		static uint8_t u8PcuInitCnt = 0;
		
		switch (u8PcuState)
		{
			case PCU_START:
				if(++u8PcuInitCnt >= PCU_INIT_FAIL)//24.01.02 SJ 
				{
					/*pcu poweron timeout warn*/
					i32ErrCodeSet(ErrCode84);	//23.11.21 SJ 修改故障码
				}
				vPcuStart();
				u8PcuState = PCU_READY;
				u8PcuTimerCnt = 0;
				break;
			case PCU_READY:
				if(++u8PcuTimerCnt >= PCU_TIMEROUT)
				{
					u8PcuState = PCU_START;
					u8PcuTimerCnt = 0;
				}
				else
				{
					if(PCU_STATE_SUCCESS == u8PcuReadyProc())
					{
						u8PcuState = PCU_QUERY;
						u8PcuTimerCnt = 0;
						
					}
				}
				break;
			case PCU_QUERY:
				if(u8PcuQueryCnt++ >= PCU_COM_FAIL)
				{
					u8PcuState = PCU_START;
					u8PcuTimerCnt = 0;
					u8PcuQueryCnt = 0;
					/*enter safety mode*/
					{
//						i32LocalDoSet(DO_DRIVEREN, 0);		/*lilu 20230704 DRIVEREN disable*/
						//23.12.26 SJ 
						i32LogWrite(ERR, LOG_PCU, "PcuProc TimeOut!\r\n");
					}
				}
				else
				{
					vPcuQuery();
					u8PcuState = PCU_REV;
	//				u8PcuQueryCnt = 0;
					u8PcuTimerCnt = 0;	
				}
				break;
			case PCU_REV:
				if(++u8PcuTimerCnt >= PCU_TIMEROUT)
				{
					u8PcuState = PCU_QUERY;
				}
				else
				{
					if(PCU_STATE_SUCCESS == u8PcuRevProc())
					{
						u8PcuState = PCU_SEND;
						if(ErrCode84 == u8ErrCodeGet())
						{
							i32ErrCodeClr(ErrCode84);//23.11.21 SJ 修改故障码
							u8PcuInitCnt = 0;  //24.01.03 SJ 通讯成功后
						}
						u8PcuQueryCnt = 0;
					}
				}
				break;
			case PCU_SEND:
				if(++u8PcuTimerCnt >= PCU_TIMEROUT)
				{
					u8PcuState = PCU_QUERY;
				}
				else
				{			
					vPcuSend(&sgPcuProc.PcuSendData);
					u8PcuState = PCU_WAIT_ONCE;
				}
				
				break;
			case PCU_WAIT_ONCE:
				if(++u8PcuTimerCnt >= PCU_TIMEROUT)
				{
					u8PcuState = PCU_QUERY;
				}
				break;
			default:
				u8PcuState = PCU_START;
				u8PcuTimerCnt = 0;
				u8PcuQueryCnt = 0;
				break;
		}
	}
//#endif	
	vWdgSetFun(WDG_PCU_BIT);
	i32LogWrite(DEBUG, LOG_PCU, "PcuProc is Running!\r\n");
}

void vPcuDisplayNumber(xPcuSendPara *SendData,uint8_t u8Number,uint8_t u8Dot)//23.11.21 SJ增加单独控制PCU显示数字功能
{
	uint8_t	u8Left;
	uint8_t	u8Right;
	if(0 != (u8Dot & HEX_DISPLAY))
	{
		u8Left = (u8SigList[0][((u8Number & 0xF0) / 16)]<<4) | u8SigList[1][((u8Number & 0xF0) / 16)];
		u8Right = (u8SigList[0][(u8Number & 0xF)]<<4) | u8SigList[1][((u8Number & 0xF) % 16)];			
	}
	else
	{
		if(u8Number >= 100)//100以上显示9.9
		{
			u8Number = 99;
			u8Dot |= LEFT_DOT;
		}
		
		u8Left = (u8SigList[0][(u8Number / 10)]<<4) | u8SigList[1][(u8Number / 10)];
		u8Right = (u8SigList[0][(u8Number % 10)]<<4) | u8SigList[1][(u8Number % 10)];	
	}

	if(0 != (u8Dot & MASK_LEFT))//左侧不显示
	{
		u8Left = 0;
	}
	if(0 != (u8Dot & MASK_RIGHT))//右侧不显示
	{
		u8Right = 0;
	}	
	if(0 != (u8Dot & RIGHT_DOT))//右侧小数点
	{
		u8Right += 128;
	}
	
	if(0 != (u8Dot & LEFT_DOT))//左侧小数点
	{
		u8Left += 128;
	}

		SendData->Data.b4ErrCodeSigLHigh4 = (u8Left>>4) & 0b1111;
		SendData->Data.b4ErrCodeSigLLow4 = u8Left & 0b1111;
		SendData->Data.b4ErrCodeSigRHigh4 = (u8Right>>4) & 0b1111;
		SendData->Data.b4ErrCodeSigRLow4 = u8Right & 0b1111;
}
void vPcuDisplayOrigin(xPcuSendPara *SendData,uint8_t u8NumberL,uint8_t u8NumberR)//23.11.21 SJ修改发送给PCU的原始数据，八位数显管模式
{
	SendData->Data.b4ErrCodeSigLHigh4 = (u8NumberL>>4) & 0b1111;
	SendData->Data.b4ErrCodeSigLLow4 = u8NumberL & 0b1111;
	SendData->Data.b4ErrCodeSigRHigh4 = (u8NumberR>>4) & 0b1111;
	SendData->Data.b4ErrCodeSigRLow4 = u8NumberR & 0b1111;
}

#endif
