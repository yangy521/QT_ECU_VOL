#include "CanBaudRateSync.h"


#if (SINGLE_MCU != MCU_TYPE)
static uint8_t u8GetSumCheckSum(uint8_t *u8Src, uint16_t u16Len)
{
	uint8_t res = 0;
	uint16_t i = 0;
	for(i = 0; i < u16Len; i++)
	{
		res += u8Src[i];
	}
	return res;
}
#endif



#if (MULTIPLE_MCU_SLV == MCU_TYPE)
void vQueryCanBaudRate(SendCallBackt SendCallBack)
{
	uint8_t u8Buf[8] = {0};
	u8Buf[0] = QUERY_HEAD;
	u8Buf[1] = QUERY_LENGTH;
	u8Buf[2] = 'C';
	u8Buf[3] = 'A';
	u8Buf[4] = 'N';
	u8Buf[5] = '=';
	u8Buf[6] = '?';
	u8Buf[7] = u8GetSumCheckSum(u8Buf + 1, QUERY_LENGTH + 1);
	
	/*lilu 20240202 write to Uart*/
	{
		if (NULL != SendCallBack)
		{
			SendCallBack(u8Buf, sizeof(u8Buf));
		}
	}	
}

uint8_t u8GetMstRespond(uint8_t u8Data, uint8_t *u8CanBaud)
{
	uint8_t u8Res = 0;
	
	static uint8_t u8RevBuf[8] = {0};
	static uint8_t u8RxState = 0;
	static uint8_t u8Cnt = 0;
	
	switch (u8RxState)
	{
		case STATE_HEAD:
			if (RESPOND_HEAD == u8Data)
			{
				u8RxState++;
				u8Cnt = 0;
			}
			else
			{
				u8RxState = STATE_HEAD;
			}
			break;

		case STATE_LENGTH:
			if (RESPOND_LENGTH == u8Data)
			{
				u8RxState++;
				u8RevBuf[u8Cnt++] = u8Data;
			}
			else
			{
				u8RxState = STATE_HEAD;
			}
			break;
		case STATE_DATA:
			if (u8Cnt < RESPOND_LENGTH)
			{
				u8RevBuf[u8Cnt++] = u8Data;
			}
			else
			{
				u8RevBuf[u8Cnt++] = u8Data;
				u8RxState++;
			}
			break;
		case STATE_CHECKSUM:
			if (u8Data == u8GetSumCheckSum(u8RevBuf,  RESPOND_LENGTH + 1))
			{
				if ('C' == u8RevBuf[1] && 'A' == u8RevBuf[2] && 'N' == u8RevBuf[3] && '=' == u8RevBuf[4])
				{
					u8Res = 1;
					if (NULL != u8CanBaud)
					{
						*u8CanBaud = u8RevBuf[5] - '0';
					}
				}
			}
			u8RxState = STATE_HEAD;
			break;
		default:
			u8RxState = STATE_HEAD;
			break;
	}
	return u8Res;
}
#elif (MULTIPLE_MCU_MST == MCU_TYPE)

void vRespondCanBaudRate(uint8_t u8Data, SendCallBackt SendCallBack)
{
	uint8_t u8Buf[8] = {0};
	u8Buf[0] = RESPOND_HEAD;
	u8Buf[1] = RESPOND_LENGTH;
	u8Buf[2] = 'C';
	u8Buf[3] = 'A';
	u8Buf[4] = 'N';
	u8Buf[5] = '=';
	u8Buf[6] = u8Data + '0';
	u8Buf[7] = u8GetSumCheckSum(u8Buf + 1, RESPOND_LENGTH + 1);
	
	/*lilu 20240202 write to Uart*/
	{
		if (NULL != SendCallBack)
		{
			SendCallBack(u8Buf, sizeof(u8Buf));
		}
	}	
}

uint8_t u8GetSlvQuery(uint8_t u8Data)
{
	uint8_t u8Res = 0;
	
	static uint8_t u8RevBuf[8] = {0};
	static uint8_t u8RxState = 0;
	static uint8_t u8Cnt = 0;
	
	switch (u8RxState)
	{
		case STATE_HEAD:
			if (QUERY_HEAD == u8Data)
			{
				u8RxState++;
				u8Cnt = 0;
			}
			else
			{
				u8RxState = STATE_HEAD;
			}
			break;

		case STATE_LENGTH:
			if (QUERY_LENGTH == u8Data)
			{
				u8RxState++;
				u8RevBuf[u8Cnt++] = u8Data;
			}
			else
			{
				u8RxState = STATE_HEAD;
			}
			break;
		case STATE_DATA:
			if (u8Cnt < QUERY_LENGTH)
			{
				u8RevBuf[u8Cnt++] = u8Data;
			}
			else
			{
				u8RevBuf[u8Cnt++] = u8Data;
				u8RxState++;
			}
			break;
		case STATE_CHECKSUM:
			if (u8Data == u8GetSumCheckSum(u8RevBuf,  QUERY_LENGTH + 1))
			{
				if ('C' == u8RevBuf[1] && 'A' == u8RevBuf[2] && 'N' == u8RevBuf[3] && '=' == u8RevBuf[4] && '?' == u8RevBuf[5])
				{
					u8Res = 1;
				}
			}
			u8RxState = STATE_HEAD;
			break;
		default:
			u8RxState = STATE_HEAD;
			break;
	}
	return u8Res;
}

#endif
