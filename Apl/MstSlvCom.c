#include "MstSlvCom.h"
#include "WdgProc.h"
#include "UartComm.h"
#include "string.h"
#include "Led.h"
#include "LocalDo.h"
#include "LocalDi.h"
#include "Log.h"
#include "ErrCode.h"
#include "Para.h"

#define	MST_REV_STATE_HEAD		0
#define	MST_REV_STATE_LEGNTH	1
#define	MST_REV_STATE_DATE		2
#define	MST_REV_STATE_CHECKSUM	3

#define	MST_STATE_FAILED		0
#define	MST_STATE_SUCCESS		1

#define MCU_PARA_LENGTH			8


static xMstSlvProc sgMstSlvProc;
//static xPcuRevProct sgMstRevProc;
//static xMstSendPara sgMstSend;

/*MST And Slave data checksum*/
static uint8_t u8GetXorCheckSum(uint8_t *u8Src, uint16_t u16Len)
{
	uint8_t res = 0;
	uint16_t i = 0;
	for(i = 0; i < u16Len; i++)
	{
//		res ^= u8Src[i];
		res += u8Src[i];
	}
	return res;
}


static void vDelayms(uint32_t u32Delay)
{
	uint32_t i = 0, j = 0;
	for (i=0; i<u32Delay; i++)
	{
		for (j=0; j<10000; j++)
		{
			__NOP();
		}
	}
}

#ifdef BAUDRATE_SYCHRON	//23.11.21 SJ 暂时屏蔽波特率相关功能，避免自动设置为500k
static void vUpdateCanBaud(uint32_t u32CanBaud)
{
	fmc_unlock();
	fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);
	fmc_page_erase(CAN_BAUDRATE_ADDRESS);
	fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);
	fmc_lock();
	
	vDelayms(10);
	
	fmc_unlock();
    /* Clear All pending flags */
    fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);
	fmc_word_program(CAN_BAUDRATE_ADDRESS, u32CanBaud);
	fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);
    fmc_lock();
	
	u16SaveParaToEeprom(PARA_CanBaudRate, (uint16_t)(u32CanBaud & 0xFFFF));
	
}

static void vSoftReset(void)
{
	__disable_irq();
	__set_FAULTMASK(1);
	NVIC_SystemReset();
}

#endif

/*接收MST发送的信息*/
static uint8_t u8MstRevProc(void)
{	
	int length = 0;
	uint8_t u8tmp[16];
	uint8_t *pTmp = u8tmp;
	uint8_t u8CheckSum = 0;
	uint8_t res = MST_STATE_FAILED;
	
	static uint8_t u8RevCnt = 0;
	static uint8_t u8PcuRevBuf[MST_REV_LENGTH + 3];
	static uint8_t u8RxState = 0;
	static uint8_t u8Length = 0;
	
	static uint8_t u8CanFlag = 0;		/*lilu 20231008 CanFlag*/
	static uint8_t u8CanCount = 0; 	/*查询20次*/
	
	length = i32UartRead(Uart1, u8tmp, 16);
	if(length > 0)
	{
		while(length)
		{
			
			switch (u8RxState)
			{
				case MST_REV_STATE_HEAD:
					if(MST_REV_HEAD == *pTmp++)
					{
						u8PcuRevBuf[u8RevCnt++] = MST_REV_HEAD;
						u8RxState = MST_REV_STATE_LEGNTH;
					}
					else
					{
						u8RxState = MST_REV_STATE_HEAD;
						u8RevCnt = 0;
					}
					u8Length = 0;
					break;
				case MST_REV_STATE_LEGNTH:
					if ((MST_REV_LENGTH == *pTmp) || (MCU_PARA_LENGTH == *pTmp))
					{
						u8Length = *pTmp;
						u8PcuRevBuf[u8RevCnt++] = *pTmp++;
						u8RxState = MST_REV_STATE_DATE;
					}
					else
					{
						u8RxState = MST_REV_STATE_HEAD;
						u8RevCnt = 0;
						u8Length = 0;
					}
//					u8Length = *pTmp++;
//					u8RxState = MST_REV_STATE_DATE;
//					if(MST_REV_LENGTH == *pTmp++)
//					{
//						u8PcuRevBuf[u8RevCnt++] = MST_REV_LENGTH;
//						u8RxState = MST_REV_STATE_DATE;
//					}
//					else
//					{
//						u8RxState = MST_REV_STATE_HEAD;
//						u8RevCnt = 0;
//					}
					break;
				case MST_REV_STATE_DATE:
					if (u8RevCnt < u8Length + 1)
					//if(u8RevCnt < MST_REV_LENGTH + 1)
					{
						u8PcuRevBuf[u8RevCnt++] = *pTmp++;
						u8RxState = MST_REV_STATE_DATE;
					}
					else
					{
						u8PcuRevBuf[u8RevCnt++] = *pTmp++;
						u8RxState = MST_REV_STATE_CHECKSUM;
					}
					break;
				case MST_REV_STATE_CHECKSUM:
					//u8CheckSum = u8GetXorCheckSum(u8PcuRevBuf + 1, MST_REV_LENGTH + 1);
					u8CheckSum = u8GetXorCheckSum(u8PcuRevBuf + 1, u8Length + 1);
					if(*pTmp++ == u8CheckSum)
					{
						#ifdef BAUDRATE_SYCHRON//(MCU_PARA_LENGTH == u8Length) //23.11.21 SJ 暂时屏蔽波特率相关功能，避免自动设置为500k
						{
							/*lilu 20231008 add canbaud para*/
							if ((0x81 == u8PcuRevBuf[3]) && (0x43 == u8PcuRevBuf[2]))
							{
								u8CanFlag = 1;
								uint32_t u32CanBaud = ((uint32_t)u8PcuRevBuf[9] << 24) | ((uint32_t)u8PcuRevBuf[8] << 16) | ((uint32_t)u8PcuRevBuf[7] << 8) | ((uint32_t)u8PcuRevBuf[6]);
								if (u32CanBaud != i32GetPara(PARA_CanBaudRate))
								{
									/*Update CanBaud*/
									vUpdateCanBaud(u32CanBaud);
									vDelayms(10);
									vSoftReset();
								}
							}
							/*add Para Communication*/
							if (NULL != sgMstSlvProc.RevMcuParaCallBack)
							{
								sgMstSlvProc.RevMcuParaCallBack(u8PcuRevBuf + 2, MCU_PARA_LENGTH);
							}
							u8RevCnt = 0;	
						}
						else 
						#endif	//BAUDRATE_SYCHRON
						if(MST_REV_LENGTH == u8Length)
						{
#if(0)
							/*lilu 20231008  add canbaud para*/
							if ((0 == u8CanFlag)&&( 20 >= u8CanCount))
							{
								u8CanCount++;
								uint8_t u8QueryCanBaud[8] = {0x40, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,};
								vQueryMcuPara(u8QueryCanBaud, 8);
							}
#endif
							if(0 == (sgMstSlvProc.MstRevData.b1ToggleBit & (u8PcuRevBuf[2] >> 7)))
							{
								memcpy(&sgMstSlvProc.MstRevData, u8PcuRevBuf + 2, MST_REV_LENGTH);
								res = MST_STATE_SUCCESS;
								/*add user resolve mst data*/
								i32SetPara(PARA_MotorSpeed, (uint16_t)(sgMstSlvProc.MstRevData.u8SpeedFdbHigh << 8) | sgMstSlvProc.MstRevData.u8SpeedFdbLow);		/*lilu 20230920 add MotorSpd*/
								if(NULL != sgMstSlvProc.MstRevCallBack)
								{
									sgMstSlvProc.MstRevCallBack(&sgMstSlvProc.MstRevData);
								}
								u8RxState = MST_REV_STATE_HEAD;
								u8RevCnt = 0;
							}
							else
							{
								u8RxState = MST_REV_STATE_HEAD;
								u8RevCnt = 0;
							}
						}
						else if(MST_REV_PARA_LENTH == u8Length)//24.3.18 SJ查改参数
						{
							res = MST_STATE_SUCCESS;
							if (NULL != sgMstSlvProc.RevMcuParaCallBack)
							{
								sgMstSlvProc.RevMcuParaCallBack(u8PcuRevBuf + 2, MCU_PARA_LENGTH);
							}											
							u8RxState = MST_REV_STATE_HEAD;
							u8RevCnt = 0;
						}
						else
						{
							/**/
							u8RevCnt = 0;
						}
					}
					else	
					{
						u8RxState = MST_REV_STATE_HEAD;
						u8RevCnt = 0;
					}
					break;
				default:
					u8RxState = MST_REV_STATE_HEAD;
					break;	
			}
			length--;
		}
	}
	return res;
}

/*给MST发送的信息*/
static void vMstSlvSend(void)
{
	sgMstSlvProc.MstSendData.u8Head = MST_SEND_HEAD;
	sgMstSlvProc.MstSendData.u8Length = MST_SEND_LENGTH;
	if (NULL != sgMstSlvProc.MstSendCallBack)
	{
		sgMstSlvProc.MstSendCallBack(&sgMstSlvProc.MstSendData);
	}
	sgMstSlvProc.MstSendData.b1ToggleBit = !sgMstSlvProc.MstSendData.b1ToggleBit;
	sgMstSlvProc.MstSendData.u8ErrCode =  u8ErrCodeGet();
	sgMstSlvProc.MstSendData.u8CheckSum = u8GetXorCheckSum(&sgMstSlvProc.MstSendData.u8Length, MST_SEND_LENGTH + 1);
	i32UartWrite(Uart1, (uint8_t*)&sgMstSlvProc.MstSendData, sizeof(sgMstSlvProc.MstSendData));
	/*lilu 20230920 add monitor para*/
	{
		i32SetPara(PARA_MotorCmd, (uint16_t)(sgMstSlvProc.MstSendData.u8TargetHigh << 8) | sgMstSlvProc.MstSendData.u8TargetLow);			/**/
		i32SetPara(PARA_PumpCmd, sgMstSlvProc.MstSendData.u8PumpTarget);			/**/
		i32SetPara(PARA_MotorState, sgMstSlvProc.MstSendData.buf[2]);				/**/
	}
}

/*给MST发送的信息*/
void vQueryMcuPara(uint8_t *u8Data, uint16_t u16ength)
{
	uint8_t u8SendData[MCU_PARA_LENGTH + 3] = {0};
	u8SendData[0] = MST_SEND_HEAD;
	u8SendData[1] = MCU_PARA_LENGTH;
	if (u16ength <= MCU_PARA_LENGTH)
	{
		memcpy(u8SendData + 2, u8Data, u16ength);
	}
	u8SendData[MCU_PARA_LENGTH + 2] = u8GetXorCheckSum(u8SendData + 1, MCU_PARA_LENGTH + 1);
	i32UartWrite(Uart1, u8SendData, MCU_PARA_LENGTH + 3);
}


/////*******************************************************************************
////* Name: void vMstSlvSetMotorVal(uint16_t u16Val)
////* Descriptio: Set Motor Destion Value
////* Input: u16Val: Motor Target Value
////* Output: NULL  
////*******************************************************************************/
//void vMstSlvSetMotorVal(uint16_t u16Val)
//{	
//	sgMstSlvProc.MstSendData.u8TargetHigh = (uint8_t)((u16Val >> 8) & 0xFF);
//	sgMstSlvProc.MstSendData.u8TargetLow = (uint8_t)(u16Val & 0xFF);
////	//i32LogWrite(INFO, "High = %d, Low = %d\r\n", sgMstSlvProc.MstSendData.u8TargetHigh, \
////						sgMstSlvProc.MstSendData.u8TargetLow);
//}
/////*******************************************************************************
////* Name: vMstSlvSetPumpVal(uint8_t u8Val)
////* Descriptio: Set PmupVal
////* Input: u8Val: Pump Target Value
////* Output: NULL  
////*******************************************************************************/
//void vMstSlvSetPumpVal(uint8_t u8Val)
//{
//	sgMstSlvProc.MstSendData.u8PumpTarget = u8Val;
//}

/*******************************************************************************
* Name: void vMstSlvSetBit(eMstSlvBitNo MstSlvBitNo, uint8_t u8Val)
* Descriptio: Set MstSlv Send Bit
* Input: MstSlvBitNo:channel range in[ServoOnNo ~ MstSlvBitMax)
*		 u8Val: 0 or 1
* Output: NULL  
*******************************************************************************/
//void vMstSlvSetBit(eMstSlvBitNo MstSlvBitNo, uint8_t u8Val)
//{
//	if(MstSlvBitNo >= MstSlvBitMax)
//	{
//		//i32LogWrite(ERR, "MstSlvSetBit Para is Wrong, MstSlvBitMax = %d, MstSlvBitNo = %d\r\n", MstSlvBitMax, MstSlvBitNo);
//	}
//	
//	switch(MstSlvBitNo)
//	{
//		case ServoOnNo:
//			sgMstSend.b1ServoOn = u8Val;
//			break;
//		case PowerLineOnNo:
//			sgMstSend.b1PowerLineOn = u8Val;
//			break;
//		case BrakeReqNo:
//			sgMstSend.b1BrakeReq = u8Val;
//			break;
//		case ForwardReqNo:
//			sgMstSend.b1ForwardReq = u8Val;
//			break;
//		case BackwardReqNo:
//			sgMstSend.b1BackwardReq = u8Val;
//			break;
//		case LiftUpReqNo:
//			sgMstSend.b1LiftReq = u8Val;
//			break;
//		case LiftDownReqNo:
//			sgMstSend.b1DownReq = u8Val;
//			break;
//		default:
//			break;
//	}
//}
/*******************************************************************************
* Name: vMstRevRegister(MstCallBackt CallBack)
* Descriptio: MstSlv接收注册回调函数
* Input: CallBack：回调函数
* Output: NULL  
*******************************************************************************/
void vMstRevRegister(MstRevCallBackt CallBack)
{
	sgMstSlvProc.MstRevCallBack = CallBack;
}

/* Name: vMstSendRegister(MstCallBackt CallBack)
* Descriptio: MstSlvSend注册回调函数
* Input: CallBack：回调函数
* Output: NULL  
*******************************************************************************/
void vMstSendRegister(MstSendCallBackt CallBack)
{
	sgMstSlvProc.MstSendCallBack = CallBack;
}

/* Name: vMstSendRegister(MstCallBackt CallBack)
* Descriptio: MstSlvSend注册回调函数
* Input: CallBack：回调函数
* Output: NULL  
*******************************************************************************/
void vMcuParaRevRegister(RevMcuParaCallBackt CallBack)
{
	sgMstSlvProc.RevMcuParaCallBack = CallBack;
}

/*******************************************************************************
* Name: void vMstSlvComProc(void)
* Descriptio: Pcu通信处理
* Input: NULL
* Output: NULL  
*******************************************************************************/
void vMstSlvComProc(void)
{
	static uint16_t u16MstRevCnt = 0;
	static uint8_t u8InitStat = 0;
	
	/*lilu 20231008 add CanBaud*/
	#ifdef BAUDRATE_SYCHRON	//23.11.21 SJ 暂时屏蔽波特率相关功能，避免自动设置为500k
	if (*(uint32_t*)CAN_BAUDRATE_ADDRESS != i32GetPara(PARA_CanBaudRate))
	{
		vUpdateCanBaud(i32GetPara(PARA_CanBaudRate));
	}
	#endif //BAUDRATE_SYCHRON
	
	if(MST_STATE_SUCCESS == u8MstRevProc())
	{
		u16MstRevCnt = 0;
		u8InitStat = 1;
	}
	else
	{
		
		if(u16MstRevCnt++ >= MST_REV_TIMEOUT) 
		{
			u16MstRevCnt = 0;
			/*close driver enable*/
			if (1 == u8InitStat)
			{	
				i32LocalDoSet(DO_DRIVEREN, 0);
				i32ErrCodeSet(ErrCode81);				/*lilu 20230701, 主从MCU通信异常，报63号故障*///23.11.21 SJ 修改报警码
				i32LogWrite(ERR, LOG_MST, "Long Time No Rev Mst Uart Data!!!!!!\r\n");
			}
		}
	}

	vMstSlvSend();	
	vWdgSetFun(WDG_MSTSLV_BIT);
	i32LogWrite(DEBUG, LOG_MST, "MstProc is Running\r\n");
}