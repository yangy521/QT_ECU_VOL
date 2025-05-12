/*******************************************************************************
* Filename: CanCom.c 	                                    	     	           *
* Description: 	Can收发功能						           				       *
* Author:  QExpand, Lilu                                                       *
* Date: 2023/5/12															   *
* Revision:	V1.00														       *
*******************************************************************************/

#include "CanCom.h"
#include "Log.h"
#include "Device.h"
#include "string.h"


//定义CAN 数据桢接收缓冲区
typedef	 struct 
{
	uint16_t u16Write;	    	//当前写位置
	uint16_t u16Read;	    	//当前读位置
	tCanFrame   canFrmData[CAN_MAX_Buffer];	//报文数据存储区
}tCanFrmBuffer;

static tCanFrmBuffer Can0WrFrmBuffer = {.u16Write = 0, .u16Read = 0};
static tCanFrmBuffer Can0RdFrmBuffer = {.u16Write = 0, .u16Read = 0};


/*canFrame转换成CanMessage */
static int32_t i32CanFrameToCanMessage(tCanFrame *src, can_trasnmit_message_struct *dest)
{
	if(NULL == dest)
	{
		return -1;
	}
	
	dest->tx_ft = src->u8Rtr;
//	if (0 != dest->tx_ft)
//	{
//		dest->tx_ff = CAN_FF_EXTENDED;
//		dest->tx_efid = src->u32ID;
//	}
//	else
//	{
//		dest->tx_ff = CAN_FF_STANDARD;
//		dest->tx_sfid = src->u32ID;
//	}
		
	if (src->u32ID > 0x7FF)
	{
		dest->tx_ff = CAN_FF_EXTENDED;
		dest->tx_efid = src->u32ID;
	}
	else
	{
		dest->tx_ff = CAN_FF_STANDARD;
		dest->tx_sfid = src->u32ID;
	}
	dest->tx_dlen = src->u16DataLength;	
	dest->tx_data[0]= src->u8Data[0];
	dest->tx_data[1]= src->u8Data[1];
	dest->tx_data[2]= src->u8Data[2];
	dest->tx_data[3]= src->u8Data[3];
	dest->tx_data[4]= src->u8Data[4];
	dest->tx_data[5]= src->u8Data[5];
	dest->tx_data[6]= src->u8Data[6];
	dest->tx_data[7]= src->u8Data[7];

	return 0;	
}


/*******************************************************************************
* Name: int32_t i32CanWrite(eCanNo CanNo, tCanFrame *pCanWriteFrame)
* Descriptio: Write CanFrame to Can
* Input: CanNo: range Can0
*        pCanWriteFrame: write CanFrame Point
* Output:  CAN_FAILURE: failure;
		   CAN_SUCCESS: success
*******************************************************************************/
int32_t i32CanWrite(eCanNo CanNo, tCanFrame *pCanWriteFrame)
{
	uint8_t res = CAN_FAILURE;
	uint16_t u16Tmp = 0;
	
	if(CanNo < CanMax)
	{
		if(Can0 == CanNo)
		{
			uint8_t canState = u8GetCanSendState(CAN0);
			if(1 == canState)	/*Cano have empty mailbox*/
			{
				can_trasnmit_message_struct xTransmitMmessageTmp;
				i32CanFrameToCanMessage(pCanWriteFrame, &xTransmitMmessageTmp);
				can_message_transmit(CAN0, &xTransmitMmessageTmp);
				/**/
				res = CAN_SUCCESS;
			}
			else
			{
				u16Tmp = Can0WrFrmBuffer.u16Write + 1;
				
				if (u16Tmp >= CAN_MAX_Buffer)
				{
					u16Tmp = 0;
				}
				
				if (u16Tmp != Can0WrFrmBuffer.u16Read)
				{
					memcpy(&Can0WrFrmBuffer.canFrmData[Can0WrFrmBuffer.u16Write], pCanWriteFrame, sizeof(tCanFrame));
					Can0WrFrmBuffer.u16Write = u16Tmp;
//					Can0WrFrmBuffer.u16Write++;
//					if(Can0WrFrmBuffer.u16Write >= CAN_MAX_Buffer)
//					{
//						Can0WrFrmBuffer.u16Write = 0;
//					}
					res = CAN_SUCCESS;
				}
			}
		}
	}
	else
	{
		i32LogWrite(ERR, LOG_CAN, "Can Write Para is Wrong, CanNo =  %d\r\n", CanNo);
	}
	
	return res;
}
/*******************************************************************************
* Name: i32CanRead(eCanNo CanNo, tCanFrame *pCanReadFrame)
* Descriptio: can read frame function
* Input: CanNo: Can0
*        pCanReadFrame： can readFrame pointe
* Output:  CAN_FAILURE: failure;
		   CAN_SUCCESS: success 
*******************************************************************************/
int32_t i32CanRead(eCanNo CanNo, tCanFrame *pCanReadFrame)
{
	uint8_t res = CAN_FAILURE;
	
	if(NULL == pCanReadFrame)
	{
		return -1;
	}
	
	if(CanNo < CanMax)
	{
		if(Can0 == CanNo)
		{
			if(Can0RdFrmBuffer.u16Read != Can0RdFrmBuffer.u16Write)
			{
				memcpy(pCanReadFrame, &Can0RdFrmBuffer.canFrmData[Can0RdFrmBuffer.u16Read], sizeof(tCanFrame));
				Can0RdFrmBuffer.u16Read++;
				if(Can0RdFrmBuffer.u16Read >= CAN_MAX_Buffer)
				{
					Can0RdFrmBuffer.u16Read = 0;
				}
				res = CAN_SUCCESS;
			}
		}
	}
	else
	{
		i32LogWrite(ERR, LOG_CAN, "Can Read Para is Wrong, CanNo =  %d\r\n", CanNo);
	}
	
	return res;
}

/*在中断中对CAN读缓存写入canFrame*/
static int32_t i32CanWriteRxBufFromInt(eCanNo CanNo, tCanFrame *pCanWriteFrame)
{
	uint8_t res = CAN_FAILURE;
	uint16_t u16Tmp = 0;
	
	
	if(CanNo < CanMax)
	{
		if(Can0 == CanNo)
		{
			u16Tmp = Can0RdFrmBuffer.u16Write + 1;	
			if (u16Tmp >= CAN_MAX_Buffer)
			{
				u16Tmp = 0;
			}
			
			if (u16Tmp != Can0RdFrmBuffer.u16Read)
			{				
				memcpy(&Can0RdFrmBuffer.canFrmData[Can0RdFrmBuffer.u16Write], pCanWriteFrame, sizeof(tCanFrame));
				Can0RdFrmBuffer.u16Write = u16Tmp;
//				Can0RdFrmBuffer.u16Write++;
//				if(Can0RdFrmBuffer.u16Write >= CAN_MAX_Buffer)
//				{
//					Can0RdFrmBuffer.u16Write = 0;
//				}
				res = CAN_SUCCESS;
			}
		}
	}
	
	return res;
}
/*在中断中从CAN写缓存读取canFrame*/
static int32_t i32CanReadTxBufFromInt(eCanNo CanNo, tCanFrame *pCanReadFrame)
{
	if(NULL == pCanReadFrame)
	{
		return -1;
	}
	
	uint8_t res = CAN_FAILURE;
	
	if(CanNo < CanMax)
	{
		if(Can0 == CanNo)
		{
			if(Can0WrFrmBuffer.u16Read != Can0WrFrmBuffer.u16Write)
			{
				memcpy(pCanReadFrame, &Can0WrFrmBuffer.canFrmData[Can0WrFrmBuffer.u16Read], sizeof(tCanFrame));
				Can0WrFrmBuffer.u16Read++;
				if(Can0WrFrmBuffer.u16Read >= CAN_MAX_Buffer)
				{
					Can0WrFrmBuffer.u16Read = 0;
				}
				res = CAN_SUCCESS;
			}
		}
	}
	return res;
}



void USBD_HP_CAN0_TX_IRQHandler(void)
{
	tCanFrame CanFrameTmp;
	can_trasnmit_message_struct xTransmitMmessageTmp;
	
	if(can_flag_get(CAN0, CAN_FLAG_TME0))
	{
		if(CAN_SUCCESS == i32CanReadTxBufFromInt(Can0, &CanFrameTmp))
		{
			//i32CanReadTxBufFromInt(Can0, &CanFrameTmp);
			i32CanFrameToCanMessage(&CanFrameTmp, &xTransmitMmessageTmp);
			can_message_transmit(CAN0, &xTransmitMmessageTmp);
		}
		can_interrupt_flag_clear(CAN0, CAN_INT_FLAG_MTF0);
	}
	
	if(can_flag_get(CAN0, CAN_FLAG_TME1))
	{
		if(CAN_SUCCESS == i32CanReadTxBufFromInt(Can0, &CanFrameTmp))
		{
			//i32CanReadTxBufFromInt(Can0, &CanFrameTmp);
			i32CanFrameToCanMessage(&CanFrameTmp, &xTransmitMmessageTmp);
			can_message_transmit(CAN0, &xTransmitMmessageTmp);
		}
		can_interrupt_flag_clear(CAN0, CAN_INT_FLAG_MTF1);
	}
	
	if(can_flag_get(CAN0, CAN_FLAG_TME2))
	{
		if(CAN_SUCCESS == i32CanReadTxBufFromInt(Can0, &CanFrameTmp))
		{
			//i32CanReadTxBufFromInt(Can0, &CanFrameTmp);
			i32CanFrameToCanMessage(&CanFrameTmp, &xTransmitMmessageTmp);
			can_message_transmit(CAN0, &xTransmitMmessageTmp);
		}
		can_interrupt_flag_clear(CAN0, CAN_INT_FLAG_MTF2);
	}

}



void USBD_LP_CAN0_RX0_IRQHandler(void)
{
	tCanFrame canFrameTmp;
	can_receive_message_struct rxMessage;
	/* check the receive message */
	can_message_receive(CAN0, CAN_FIFO0, &rxMessage);

	if (rxMessage.rx_ff == CAN_FF_STANDARD)
	{
		canFrameTmp.u32ID = rxMessage.rx_sfid;  
	}
	else 
	{
		canFrameTmp.u32ID = rxMessage.rx_efid; 
	}
	
    canFrameTmp.u8Rtr = rxMessage.rx_ft; 
	canFrameTmp.u16DataLength=rxMessage.rx_dlen;	//	有效数据长度
	canFrameTmp.u8Data[0]=rxMessage.rx_data[0];
	canFrameTmp.u8Data[1]=rxMessage.rx_data[1];
	canFrameTmp.u8Data[2]=rxMessage.rx_data[2];
	canFrameTmp.u8Data[3]=rxMessage.rx_data[3];
	canFrameTmp.u8Data[4]=rxMessage.rx_data[4];
	canFrameTmp.u8Data[5]=rxMessage.rx_data[5];
	canFrameTmp.u8Data[6]=rxMessage.rx_data[6];
	canFrameTmp.u8Data[7]=rxMessage.rx_data[7];

	i32CanWriteRxBufFromInt(Can0, &canFrameTmp);	//把接收的数据写入CAN数据接收缓存区
}

