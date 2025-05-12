/*******************************************************************************
* Filename: ErrCode.c	                                             	 	   *
* Description:ErrCode C Source File											   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/5/31    														   *
* Revision: V1.00															   *
*******************************************************************************/
#include "ErrCode.h"
#include "Log.h"
#include "Para.h"
static uint32_t  u32ErrCode[(ErrCodeMax + 1) / 32] = {0};

static xErrCodeInfo *sgPErrCodeInfo = NULL;
static uint16_t	u16ErrCodeNo = ErrCodeMax;

void vErrCodeInit(xErrCodeInfo *ErrCodeArray, uint16_t u16ErrCodeMax)
{
	sgPErrCodeInfo = ErrCodeArray;
	u16ErrCodeNo = u16ErrCodeMax;
	if (u16ErrCodeNo >= ErrCodeMax)
	{
		u16ErrCodeNo = ErrCodeMax;
	}
	/*sync*/
	{
		uint8_t i = 0;
		for (i=0; i<u16ErrCodeNo; i++)
		{
			sgPErrCodeInfo[i].b1ErrCode = (u32ErrCode[i/32] >> (i % 32)) & 0x1;
		}
	}
}


/*******************************************************************************
* Name: int32_t i32ErrCodeSet(eErrCodeCh ErrCodeNo)
* Descriptio: Set ErrCode 
* Input: ErrCodeNo: ??ó|μ?í¨μà
* Output: -1:failure
*		   0:success
*******************************************************************************/
int32_t i32ErrCodeSet(eErrCodeCh ErrCodeNo)
{
	if (NULL == sgPErrCodeInfo)
	{
		if(ErrCodeNo >= ErrCodeMax)
		{
			i32LogWrite(ERR, LOG_ERR, "ErrCodeSet Parameter is wrong, ErrCodeMax = %d, ErrCode = %d\r\n", ErrCodeMax, ErrCodeNo);
			return -1;
		}
		u32ErrCode[ErrCodeNo / 32] |= 1 << (ErrCodeNo % 32);
	}
	else//对应bit位置一
	{
		if(ErrCodeNo >= u16ErrCodeNo)
		{
			i32LogWrite(ERR, LOG_ERR, "ErrCodeSet Parameter is wrong, ErrCodeMax = %d, ErrCode = %d\r\n", ErrCodeMax, ErrCodeNo);
			return -1;
		}
		sgPErrCodeInfo[ErrCodeNo].b1ErrCode = 1;
	}
	if (ErrCodeNo != i32GetPara(PARA_ErrCode0))//上位机故障码顺延
	{
		uint8_t i = 0;
		for (i=PARA_ErrCode0; i<PARA_ErrCodeMax; i++)
		{
			i32SetPara(i + 1, i32GetPara(i));
		}
		i32SetPara(PARA_ErrCode0, ErrCodeNo + 1);
	}
	
	return 0;
}


static int getLowestNonZeroBit(uint32_t num) 
{
    int bitPosition = 0;

    while ((num & 1) == 0 && bitPosition < 32) 
	{
        num >>= 1;
        bitPosition++;
    }

    return bitPosition;
}
/*******************************************************************************
* Name: uint8_t u8ErrCodeGet(void)
* Descriptio: ??è?×???ó??è??1ê???? 
* Input: NULL
* Output: 返回最小故障码
*******************************************************************************/
uint8_t u8ErrCodeGet(void)
{
	uint16_t i = 0;
	uint8_t res = 0;
	
	if (NULL == sgPErrCodeInfo)
	{
		if(0 != u32ErrCode[0])
		{
			res = 1 + getLowestNonZeroBit(u32ErrCode[0]);
		}
		else if(0 != u32ErrCode[1])
		{
			res = 33 + getLowestNonZeroBit(u32ErrCode[1]);
		}
		else if(0 != u32ErrCode[2])
		{
			res = 65 + getLowestNonZeroBit(u32ErrCode[2]);
		}
		else if(0 != u32ErrCode[3])
		{
			res = 97 + getLowestNonZeroBit(u32ErrCode[3]);
		}
		else if (0 != u32ErrCode[4])
		{
			res = 129 + getLowestNonZeroBit(u32ErrCode[4]);
		}
		else if (0 != u32ErrCode[5])
		{
			res = 161 + getLowestNonZeroBit(u32ErrCode[5]);
		}
		else if (0 != u32ErrCode[6])
		{
			res = 193 + getLowestNonZeroBit(u32ErrCode[6]);
		}
		else if (0 != u32ErrCode[7])
		{
			res = 225 + getLowestNonZeroBit(u32ErrCode[7]);
		}
	}
	else
	{
		for (i=0; i<u16ErrCodeNo; i++)
		{
			if (1 == sgPErrCodeInfo[i].b1ErrCode)
			{
				break;
			}
		}
		if (i < u16ErrCodeNo)
		{
			res = i + 1;
		}
	}
	return res;
}

/*******************************************************************************
* Name: uint8_t i32ErrCodeCheck(void)
* Descriptio: ??è?×???ó??è??1ê???? 
* Input: NULL
* Output: 有故障返回1，无故障返回0，超限返回-1
*******************************************************************************/
int32_t i32ErrCodeCheck(eErrCodeCh ErrCodeNo)
{
	uint8_t u8Res = 0;
	
	if (NULL == sgPErrCodeInfo)
	{
		if(ErrCodeNo >= ErrCodeMax)
		{
			i32LogWrite(ERR, LOG_ERR, "ErrCodeSet Parameter is wrong, ErrCodeMax = %d, ErrCode = %d\r\n", ErrCodeMax, ErrCodeNo);
			return -1;
		}
		u8Res = (u32ErrCode[(ErrCodeNo) / 32] >> ((ErrCodeNo) % 32)) & 0x1;
	}
	else
	{
		if(ErrCodeNo >= u16ErrCodeNo)
		{
			i32LogWrite(ERR, LOG_ERR, "ErrCodeClr Parameter is wrong, ErrCodeMax = %d, ErrCode = %d\r\n", ErrCodeMax, ErrCodeNo);
			return -1;
		}
		u8Res = sgPErrCodeInfo[ErrCodeNo].b1ErrCode;
	}
	return u8Res;
}


/*******************************************************************************
* Name: int32_t i32ErrCodeClr(eErrCodeCh ErrCodeNo)
* Descriptio: Clear ErrCode 
* Input: ErrCodeNo: ??ó|μ?í¨μà
* Output: -1:failure
*		   0:success
*******************************************************************************/
int32_t i32ErrCodeClr(eErrCodeCh ErrCodeNo )
{
	if (NULL == sgPErrCodeInfo)
	{
		if(ErrCodeNo >= ErrCodeMax)
		{
			i32LogWrite(ERR, LOG_ERR, "ErrCodeSet Parameter is wrong, ErrCodeMax = %d, ErrCode = %d\r\n", ErrCodeMax, ErrCodeNo);
			return -1;
		}
		u32ErrCode[(ErrCodeNo ) / 32] &= ~(1 << ((ErrCodeNo ) % 32));
	}
	else
	{
		if(ErrCodeNo >= u16ErrCodeNo)
		{
			i32LogWrite(ERR, LOG_ERR, "ErrCodeClr Parameter is wrong, ErrCodeMax = %d, ErrCode = %d\r\n", ErrCodeMax, ErrCodeNo);
			return -1;
		}
		sgPErrCodeInfo[ErrCodeNo].b1ErrCode = 0;
	}
	return 0;
}


uint8_t u8ErrCodeGetAbnormal(eAbnormalType eType)
{
	uint8_t res = 0;
	if (NULL != sgPErrCodeInfo)
	{
		if (eType >= ABNORMAL_NoAct && eType < ABNORMAL_Max)
		{
			uint8_t i = 0;
			for (i=0; i<u16ErrCodeNo; i++)
			{
				if ((1 == sgPErrCodeInfo[i].b1ErrCode) && (1 == ((sgPErrCodeInfo[i].u32Data >> (1 + eType))& 0x1)))
				{
					break;
				}
			}
			
			if (i < u16ErrCodeNo)
			{
				res = 1;
			}
		}
	}
	return res;
}

uint8_t u8ErrCodeGetTrans(void)
{
	uint8_t res = 0;
	uint16_t i = 0;
	if (NULL != sgPErrCodeInfo)
	{
		uint8_t u8UserErr = u16ErrCodeNo;
		
		for (i=0; i<u16ErrCodeNo; i++)
		{
			if (1 == sgPErrCodeInfo[i].b1ErrCode)
			{
				if((u8UserErr > sgPErrCodeInfo[i].b8UserErr)&&(0 != sgPErrCodeInfo[i].b8UserErr))
					u8UserErr = sgPErrCodeInfo[i].b8UserErr;
			}
		}
		if(u16ErrCodeNo  == u8UserErr)
		{
			res = 0;
		}
		else
		{
			res = u8UserErr;
		}
	}
	else
	{
		res = u8ErrCodeGet();
	}
	return res;
}


//23、12、1待完善、获取当前故障数量及对应的故障码
uint8_t u8GetQuangtityOfError(void)
{
	uint8_t res = 0;
	uint16_t i = 0;
	if (NULL != sgPErrCodeInfo)
	{
		for (i=0; i<u16ErrCodeNo; i++)
		{
			if (1 == sgPErrCodeInfo[i].b1ErrCode)
			{
				res++;
			}
		}
	}
	else
	{
		res = u8ErrCodeGet();
	}
	return res;
}



#if 0
//static uint32_t  u32ErrCode[4];	/*global varible*/
	
static xErrCodeInfo sgErrCodeInfo[ErrCodeMax] = 
{
	{.b1ErrCode = 0, .b1NoAct = 1, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//停止所有 	//1	系统初始化错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//停止所有 	//2	系统通信错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//停止所有 	//3	无效选项设置错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//停止所有 	//4	闪存数据错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//停止所有 	//5	锂电池通讯丢失
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//停止PCU  	//6	上电时举升按键按下
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//停止PCU  	//7	上电时龟速按键按下
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//停止PCU  	//8	上电时行走按键按下
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//只是报警   	//9	GPS连接错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//只能下降 	//10	主接触器故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//暂无		//11	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//待定：停止所有底盘控制	//12	启动时底盘上升或下降按钮打开错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//只能下降 	//13	BMS-电池温差过大2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 1, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//降低功率输出 	//14	BMS-电池温度过高1
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//只能下降 	//15	BMS-放电温度过高2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 1, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//降低功率输出 	//16	BMS-放电电流过高1
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//只能下降 	//17	BMS-放电电流过高2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 1, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//降低功率输出 	//18	坑洞保护错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//暂无		//19	BMS-总电压过低1
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//只能下降 	//20	BMS-总电压过低2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 1, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//降低功率输出 	//21	BMS-单体电压过低1
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//只能下降 	//22	BMS-单体电压过低2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//暂无		//23	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//暂无		//24	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//暂无//		//25	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//暂无//		//26	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//停止所有 	//27	下降阀2错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//暂无//		//28	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//暂无//		//29	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//只能下降 	//30	BMS-电池压差过大
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//停止起升行走、只转向下降	//31	压力传感器错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//停止起升行走、只转向下降	//32	角度传感器错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 1, .b1BackWard = 0, .b1ForWard = 0, },	//禁止起升	//33	电池类型错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//暂无//		//34	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//停止起升行走、只转向下降	//35	称重标定反
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 1, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//降低功率输出 	//36	电池电量低一级报警
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//暂无//		//37	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//停止起升行走、只转向下降	//38	未标定完成或标定失败
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//停止起升行走、只转向下降	//39	通信故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//暂无//		//40	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 1, .b1BackWard = 0, .b1ForWard = 0, },	//禁止起升	//41	平台一级锁车
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//只是报警   	//42	启动时，平台向左转向按钮按下错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//只是报警   	//43	启动时，平台向右转向按钮按下错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//停止起升行走、只转向下降	//44	平台二级锁车
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//暂无//		//45	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//停止PCU	//46	启动时，平台手柄使能开关按钮按下错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 1, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//降低功率输出 	//47	启动时，平台手柄不在中位错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//暂无//		//48	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//暂无//		//49	心跳锁车
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//暂无//		//50	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//暂无//		//51	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//只能下降 	//52	前进阀错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//只能下降 	//53	后退阀错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//只能下降 	//54	举升阀错误
	{.b1ErrCode = 0, .b1NoAct = 1, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//停止所有 	//55	下降阀错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//只能下降 	//56	右转阀错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//只能下降 	//57	左转阀错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//暂无//		//58	刹车阀错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//只能下降 	//59	并联阀故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//只是报警   	//60	驱动器故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//停止所有 	//61	驱动器电流传感器故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//只能下降 	//62	驱动器硬件损坏故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//只能下降 	//63	泵电驱开路故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//只能下降 	//64	左电驱开路故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//停止所有 	//65	控制电压5V故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//只能下降 	//66	动作时，检测到左转阀开路或短路
	{.b1ErrCode = 0, .b1NoAct = 1, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//停止所有 	//67	控制电压12V故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//只能下降 	//68	电池低电量二级报警
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//暂无//		//69	高零位电流错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//只能下降 	//70	驱动器母线电压过高故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//只能下降 	//71	预充故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 1, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//降低功率输出 	//72	驱动器母线电压过低故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 1, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//降低功率输出 	//73	驱动器低温故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 1, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//降低功率输出 	//74	驱动器高温一级故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 1, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//降低功率输出 	//75	泵电机温度一级故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//只能下降 	//76	泵电机编码器故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//只能下降 	//77	电机编码错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//只能下降 	//78	泵电机过流类故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//只能下降 	//79	泵电机温度二级故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//只是报警   	//80	超过 80%负载报警
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//只能下降 	//81	驱动器温度二级故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 1, .b1ForWard = 1, },	//禁止行走	//82	右刹车故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 1, .b1ForWard = 1, },	//禁止行走	//83	左刹车故障
	{.b1ErrCode = 0, .b1NoAct = 1, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//停止所有 	//84	泵电机堵转、失速
	{.b1ErrCode = 0, .b1NoAct = 1, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//停止所有 	//85	左牵引电机堵转、失速
	{.b1ErrCode = 0, .b1NoAct = 1, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//停止所有 	//86	由牵引电机堵转、失速
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//暂无//		//87	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//暂无//		//88	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//只是报警   	//89	驱动器运行时间过长故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//只是报警   	//90	超过 90%负载报警
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 1, .b1ForWard = 1, },	//禁止行走	//91	左电机电流过流故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 1, .b1ForWard = 1, },	//禁止行走	//92	右电机电流过流故障
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//暂无//		//93	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//暂无//		//94	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//暂无//		//95	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//暂无//		//96	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//暂无//		//97	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//暂无//		//98	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//只是报警   	//99	超过 99%负载报警
	{.b1ErrCode = 0, .b1NoAct = 1, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//停止所有 	//100	平台超载报警
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//停止起升行走、只转向下降	//101	机器倾斜超过安全限定错误
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*102*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*103*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*104*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*105*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*106*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*107*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*108*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*109*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*110*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*111*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*112*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*113*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*114*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*115*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*116*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*117*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*118*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*119*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*120*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*121*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*122*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*123*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*124*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*125*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*126*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*127*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*128*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*129*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*130*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*131*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*132*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*133*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*134*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*135*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*136*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*137*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*138*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*139*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*140*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*141*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*142*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*143*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*144*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*145*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*146*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*147*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*148*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*149*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*150*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*151*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*152*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*153*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*154*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*155*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*156*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*157*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*158*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*159*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*160*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*161*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*162*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*163*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*164*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*165*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*166*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*167*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*168*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*169*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*170*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*171*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*172*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*173*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*174*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*175*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*176*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*177*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*178*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*179*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*180*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*181*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*182*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*183*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*184*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*185*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*186*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*187*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*188*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*189*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*190*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*191*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*192*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*193*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*194*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*195*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*196*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*197*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*198*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*199*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*200*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*201*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*202*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*203*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*204*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*205*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*206*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*207*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*208*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*209*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*210*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*211*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*212*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*213*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*214*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*215*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*216*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*217*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*218*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*219*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*220*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*221*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*222*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*223*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*224*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*225*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*226*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*227*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*228*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*229*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*230*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*231*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*232*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*233*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*234*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*235*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*236*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*237*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*238*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*239*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*240*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*241*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*242*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*243*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*244*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*245*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*246*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*247*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*248*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*249*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*250*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*251*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*252*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*253*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*254*/
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	/*255*/
};

/*******************************************************************************
* Name: int32_t i32ErrCodeSet(eErrCodeCh ErrCodeNo)
* Descriptio: Set ErrCode 
* Input: ErrCodeNo: 对应的通道
* Output: -1:failure
*		   0:success
*******************************************************************************/
int32_t i32ErrCodeSet(eErrCodeCh ErrCodeNo)
{
//	sgErrCodeInfo[]
	if(ErrCodeNo >= ErrCodeMax)
	{
		i32LogWrite(ERR, LOG_ERR, "ErrCodeSet Parameter is wrong, ErrCodeMax = %d, ErrCode = %d\r\n", ErrCodeMax, ErrCodeNo);
		return -1;
	}
	sgErrCodeInfo[ErrCodeNo].b1ErrCode = 1;
//	u32ErrCode[ErrCodeNo / 32] |= 1 << (ErrCodeNo % 32);
//	/*UpDate History ErrCode*/
//	if ((0 != ErrCodeNo) && (ErrCodeNo != i32GetPara(PARA_ErrCode0)))
	if (ErrCodeNo != i32GetPara(PARA_ErrCode0))
	{
		uint8_t i = 0;
		for (i=PARA_ErrCode0; i<PARA_ErrCodeMax; i++)
		{
			i32SetPara(i + 1, i32GetPara(i));
		}
		i32SetPara(PARA_ErrCode0, ErrCodeNo + 1);
	}
	
	return 0;
}
/*******************************************************************************
* Name: int32_t i32ErrCodeClr(eErrCodeCh ErrCodeNo)
* Descriptio: Clear ErrCode 
* Input: ErrCodeNo: 对应的通道
* Output: -1:failure
*		   0:success
*******************************************************************************/
int32_t i32ErrCodeClr(eErrCodeCh ErrCodeNo )
{
	if(ErrCodeNo >= ErrCodeMax)
	{
		i32LogWrite(ERR, LOG_ERR, "ErrCodeClr Parameter is wrong, ErrCodeMax = %d, ErrCode = %d\r\n", ErrCodeMax, ErrCodeNo);
		return -1;
	}
	
//	u32ErrCode[ErrCodeNo / 32] &= ~(1 << (ErrCodeNo % 32));
	sgErrCodeInfo[ErrCodeNo].b1ErrCode = 0;
	return 0;
}


static int getLowestNonZeroBit(uint32_t num) 
{
    int bitPosition = 0;

    while ((num & 1) == 0 && bitPosition < 32) 
	{
        num >>= 1;
        bitPosition++;
    }

    return bitPosition;
}
/*******************************************************************************
* Name: uint8_t u8ErrCodeGet(void)
* Descriptio: 获取最高优先级故障码 
* Input: NULL
* Output: 最高优先级的故障码
*******************************************************************************/
uint8_t u8ErrCodeGet(void)
{
	uint16_t i = 0;
	uint8_t res = 0;
	
	for (i=0; i<ErrCodeMax; i++)
	{
		if (1 == sgErrCodeInfo[i].b1ErrCode)
		{
			break;
		}
	}
	if (i < ErrCodeMax)
	{
		res = i + 1;
	}
//	uint8_t res = 0;
//	
//	if(0 != u32ErrCode[0])
//	{
//		res = 1 + getLowestNonZeroBit(u32ErrCode[0]);
//	}
//	else if(0 != u32ErrCode[1])
//	{
//		res = 33 + getLowestNonZeroBit(u32ErrCode[1]);
//	}
//	else if(0 != u32ErrCode[2])
//	{
//		res = 65 + getLowestNonZeroBit(u32ErrCode[2]);
//	}
//	else if(0 != u32ErrCode[3])
//	{
//		res = 97 + getLowestNonZeroBit(u32ErrCode[3]);
//	}
//	if(res)
//	{
////		//i32LogWrite(INFO, "Current ErrCode is %d\r\n", res);
//	}
	return res;
}


/*******************************************************************************
* Name: uint8_t u8ErrCodeGet(void)
* Descriptio: 获取最高优先级故障码 
* Input: NULL
* Output: 最高优先级的故障码
*******************************************************************************/
int32_t i32ErrCodeCheck(eErrCodeCh ErrCodeNo)
{
	uint8_t u8Res = 0;
	if(ErrCodeNo >= ErrCodeMax)
	{
		i32LogWrite(ERR, LOG_ERR, "ErrCodeClr Parameter is wrong, ErrCodeMax = %d, ErrCode = %d\r\n", ErrCodeMax, ErrCodeNo);
		return -1;
	}
	
	u8Res = sgErrCodeInfo[ErrCodeNo].b1ErrCode;
	
	return u8Res;
}


static uint8_t u8ErrCodeAbnormalNoAct(void)
{
	uint8_t res = 0;
	uint8_t i = 0;
	
	for (i=0; i<ErrCodeMax; i++)
	{
		if ((1 == sgErrCodeInfo[i].b1ErrCode) && (1 == sgErrCodeInfo[i].b1NoAct))
		{
			break;
		}
	}
	
	if (i < ErrCodeMax)
	{
		res = 1;
	}
	
	return res;
}

static uint8_t u8ErrCodeAbnormalPcu(void)
{
	uint8_t res = 0;
	uint8_t i = 0;
	
	for (i=0; i<ErrCodeMax; i++)
	{
		if ((1 == sgErrCodeInfo[i].b1ErrCode) && (1 == sgErrCodeInfo[i].b1Pcu))
		{
			break;
		}
	}
	
	if (i < ErrCodeMax)
	{
		res = 1;
	}
	
	return res;
}

static uint8_t u8ErrCodeAbnormalLiftSpd(void)
{
	uint8_t res = 0;
	uint8_t i = 0;
	
	for (i=0; i<ErrCodeMax; i++)
	{
		if ((1 == sgErrCodeInfo[i].b1ErrCode) && (1 == sgErrCodeInfo[i].b1LiftSpd))
		{
			break;
		}
	}
	
	if (i < ErrCodeMax)
	{
		res = 1;
	}
	
	return res;
}


static uint8_t u8ErrCodeAbnormalDown(void)
{
	uint8_t res = 0;
	uint8_t i = 0;
	
	for (i=0; i<ErrCodeMax; i++)
	{
		if ((1 == sgErrCodeInfo[i].b1ErrCode) && (1 == sgErrCodeInfo[i].b1Down))
		{
			break;
		}
	}
	
	if (i < ErrCodeMax)
	{
		res = 1;
	}
	
	return res;
}

static uint8_t u8ErrCodeAbnormalLeft(void)
{
	uint8_t res = 0;
	uint8_t i = 0;
	
	for (i=0; i<ErrCodeMax; i++)
	{
		if ((1 == sgErrCodeInfo[i].b1ErrCode) && (1 == sgErrCodeInfo[i].b1Left))
		{
			break;
		}
	}
	
	if (i < ErrCodeMax)
	{
		res = 1;
	}
	
	return res;
}

static uint8_t u8ErrCodeAbnormalRight(void)
{
	uint8_t res = 0;
	uint8_t i = 0;
	
	for (i=0; i<ErrCodeMax; i++)
	{
		if ((1 == sgErrCodeInfo[i].b1ErrCode) && (1 == sgErrCodeInfo[i].b1Right))
		{
			break;
		}
	}
	
	if (i < ErrCodeMax)
	{
		res = 1;
	}
	
	return res;
}

static uint8_t u8ErrCodeAbnormalUp(void)
{
	uint8_t res = 0;
	uint8_t i = 0;
	
	for (i=0; i<ErrCodeMax; i++)
	{
		if ((1 == sgErrCodeInfo[i].b1ErrCode) && (1 == sgErrCodeInfo[i].b1Up))
		{
			break;
		}
	}
	
	if (i < ErrCodeMax)
	{
		res = 1;
	}
	
	return res;
}

static uint8_t u8ErrCodeAbnormalBackWard(void)
{
	uint8_t res = 0;
	uint8_t i = 0;
	
	for (i=0; i<ErrCodeMax; i++)
	{
		if ((1 == sgErrCodeInfo[i].b1ErrCode) && (1 == sgErrCodeInfo[i].b1BackWard))
		{
			break;
		}
	}
	
	if (i < ErrCodeMax)
	{
		res = 1;
	}
	
	return res;
}

static uint8_t u8ErrCodeAbnormalForWard(void)
{
	uint8_t res = 0;
	uint8_t i = 0;
	
	for (i=0; i<ErrCodeMax; i++)
	{
		if ((1 == sgErrCodeInfo[i].b1ErrCode) && (1 == sgErrCodeInfo[i].b1ForWard))
		{
			break;
		}
	}
	
	if (i < ErrCodeMax)
	{
		res = 1;
	}
	
	return res;
}


uint8_t u8ErrCodeGetAbnormal(eAbnormalType eType)
{
	uint8_t res = 0;
	
	switch((uint8_t)eType)
	{
		case ABNORMAL_NOACT:
			res = u8ErrCodeAbnormalNoAct();
			break;
		case ABNORMAL_PCU:
			res = u8ErrCodeAbnormalPcu();
			break;
		case ABNORMAL_LIFTSPD:
			res = u8ErrCodeAbnormalLiftSpd();
			break;
		case ABNORMAL_DOWN:
			res = u8ErrCodeAbnormalDown();
			break;
		case ABNORMAL_LEFT:
			res = u8ErrCodeAbnormalLeft();
			break;
		case ABNORMAL_RIGHT:
			res = u8ErrCodeAbnormalRight();
			break;
		case ABNORMAL_UP:
			res = u8ErrCodeAbnormalUp();
			break;
		case ABNORMAL_BACKWARD:
			res = u8ErrCodeAbnormalBackWard();
			break;
		case ABNORMAL_FORWARD:
			res = u8ErrCodeAbnormalForWard();
			break;
		default:
			break;
	}
		
	
	return res;
}
#endif