#include "CanCom.h"
#include "WdgProc.h"
#include "Log.h"
#include "CanRevProc.h"
#include "canfestival.h"
#include "TestSlave.h"

#define	CANID_LOST_INVALID	0x7FFFFFFF

static xRevCallBackProc sgRevCallBack[CANREV_PROC_NUM];
/*lilu 20230902 check canid lost*/
static xCanIdLostInfo sgCanIdLost[CANID_LOST_NUM] = {
	{.u32Data = CANID_LOST_INVALID},
	{.u32Data = CANID_LOST_INVALID},
	{.u32Data = CANID_LOST_INVALID},
	{.u32Data = CANID_LOST_INVALID},
	{.u32Data = CANID_LOST_INVALID},
	{.u32Data = CANID_LOST_INVALID},
	{.u32Data = CANID_LOST_INVALID},
	{.u32Data = CANID_LOST_INVALID},
	{.u32Data = CANID_LOST_INVALID},
	{.u32Data = CANID_LOST_INVALID},
	{.u32Data = CANID_LOST_INVALID},
	{.u32Data = CANID_LOST_INVALID},
	{.u32Data = CANID_LOST_INVALID},
	{.u32Data = CANID_LOST_INVALID},
	{.u32Data = CANID_LOST_INVALID},
	{.u32Data = CANID_LOST_INVALID},
};			

void vCanRevMsgRegister(xRevCallBackProc *RevCallBack)
{
	uint8_t i = 0;
	
	for (i=0; i<CANREV_PROC_NUM; i++)
	{
		if ((RevCallBack->u32CanId == sgRevCallBack[i].u32CanId) && (RevCallBack->u32Data == sgRevCallBack[i].u32Data))
		{
			sgRevCallBack[i].CallBack = RevCallBack->CallBack;
			return;
		}
		else if(0 == sgRevCallBack[i].u32CanId)
		{
			break;
		}
	}
	
	if (i < CANREV_PROC_NUM)
	{
		sgRevCallBack[i].u32CanId = RevCallBack->u32CanId;
		sgRevCallBack[i].CallBack = RevCallBack->CallBack;
		sgRevCallBack[i].u32Data = RevCallBack->u32Data;
	}
	else
	{
		i32LogWrite(ERR, LOG_CAN, "sgRevCallBack is full\r\n");
	}
}

void vCanIdLostReg(uint32_t u32CanId, uint32_t u32TimeOut, CanIdLostCallBackt CanIdLostCallBack)
{
	uint8_t i = 0;
	for (i=0; i<CANID_LOST_NUM; i++)
	{
		if (u32CanId == sgCanIdLost[i].b31CanId)
		{
			sgCanIdLost[i].u32Cnt = 0;
			sgCanIdLost[i].u32TimeOut = u32TimeOut / CANREV_PERIOD;
			sgCanIdLost[i].CallBack = CanIdLostCallBack;
			return;
		}
		else if(CANID_LOST_INVALID == sgCanIdLost[i].b31CanId)
		{
			break;
		}
	}
	
	if (i < CANID_LOST_NUM)
	{
		sgCanIdLost[i].b31CanId = u32CanId;
		sgCanIdLost[i].u32TimeOut = u32TimeOut / CANREV_PERIOD;
		sgCanIdLost[i].CallBack = CanIdLostCallBack;
	}
	else
	{
		i32LogWrite(ERR, LOG_CAN, "sgCanIdCallBack is full\r\n");
	}
}

static uint8_t u8CanRevProcess(tCanFrame *CanFrameTmp)
{
	uint8_t res = 0;
	uint8_t i = 0;
	
	for(i=0; i<CANREV_PROC_NUM; i++)
	{
		if (CanFrameTmp->u32ID == sgRevCallBack[i].u32CanId)
		{
			if (0 == sgRevCallBack[i].u8DataCnt)
			{
				if(NULL != sgRevCallBack[i].CallBack)
				{
					sgRevCallBack[i].CallBack(CanFrameTmp);
				}
				res = 1;
				break;
			}
			else
			{
				if (((1 == sgRevCallBack[i].u8DataCnt) && (CanFrameTmp->u8Data[0] == sgRevCallBack[i].u8Data0)) || 	/*1 Head Byte*/
				    ((2 == sgRevCallBack[i].u8DataCnt) && (CanFrameTmp->u8Data[0] == sgRevCallBack[i].u8Data0) && (CanFrameTmp->u8Data[1] == sgRevCallBack[i].u8Data1)) || /*2 Byte Head*/
					((3 == sgRevCallBack[i].u8DataCnt) && (CanFrameTmp->u8Data[0] == sgRevCallBack[i].u8Data0) && 	/*3 Byte Head*/
					 (CanFrameTmp->u8Data[1] == sgRevCallBack[i].u8Data1) && (CanFrameTmp->u8Data[2] == sgRevCallBack[i].u8Data2)))
				{
					if(NULL != sgRevCallBack[i].CallBack)
					{
						sgRevCallBack[i].CallBack(CanFrameTmp);
					}
					res = 1;
					break;
				}
			}
		}
	}
	return res;
}


/*******************************************************************************
* Name: void vCanRevProc(void)
* Descriptio: Can接收处理
* Input: NULL
* Output: NULL 
*******************************************************************************/
void vCanRevProc(void)
{
	tCanFrame CanFrameReadTmp;
	Message MessageTmp;
	
	uint8_t i = 0;
	for (i=0; i<CANID_LOST_NUM; i++)
	{
		if(CANID_LOST_INVALID != sgCanIdLost[i].b31CanId)
		{
			sgCanIdLost[i].u32Cnt++;
			if (sgCanIdLost[i].u32Cnt >= sgCanIdLost[i].u32TimeOut) 
			{
				if (NULL != sgCanIdLost[i].CallBack)
				{
					sgCanIdLost[i].b1Lost = CAN_LOST;
					sgCanIdLost[i].CallBack(sgCanIdLost[i].b31CanId, CAN_LOST);
				}
			}
		}
	}
	
	while(CAN_SUCCESS == i32CanRead(Can0, &CanFrameReadTmp))
	{
		for (i=0; i<CANID_LOST_NUM; i++)
		{
			if (CanFrameReadTmp.u32ID == sgCanIdLost[i].b31CanId)
			{
				sgCanIdLost[i].u32Cnt = 0;
				if ((NULL != sgCanIdLost[i].CallBack) && (CAN_NORMAL != sgCanIdLost[i].b1Lost))
				{
					sgCanIdLost[i].b1Lost = CAN_NORMAL;
					sgCanIdLost[i].CallBack(sgCanIdLost[i].b31CanId, CAN_NORMAL);
				}
				break;
			}
		}
		if(0 == u8CanRevProcess(&CanFrameReadTmp))
		{
			/*add Process Rev Can Frame*/
			MessageTmp.cob_id = (uint16_t)CanFrameReadTmp.u32ID;
			MessageTmp.rtr = CanFrameReadTmp.u8Rtr;
			MessageTmp.len = (uint8_t)CanFrameReadTmp.u16DataLength;
			MessageTmp.data[0] =  CanFrameReadTmp.u8Data[0];
			MessageTmp.data[1] =  CanFrameReadTmp.u8Data[1];
			MessageTmp.data[2] =  CanFrameReadTmp.u8Data[2];
			MessageTmp.data[3] =  CanFrameReadTmp.u8Data[3];
			MessageTmp.data[4] =  CanFrameReadTmp.u8Data[4];
			MessageTmp.data[5] =  CanFrameReadTmp.u8Data[5];
			MessageTmp.data[6] =  CanFrameReadTmp.u8Data[6];
			MessageTmp.data[7] =  CanFrameReadTmp.u8Data[7];
			canDispatch(&TestSlave_Data, &MessageTmp);	
		}
			
	}
	vWdgSetFun(WDG_CANREV_BIT);	
	i32LogWrite(DEBUG, LOG_CAN, "can Proc is Running!\r\n");
}