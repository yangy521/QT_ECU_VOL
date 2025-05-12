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
* Input: ErrCodeNo: ??��|��?�����̨�
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
	else//��Ӧbitλ��һ
	{
		if(ErrCodeNo >= u16ErrCodeNo)
		{
			i32LogWrite(ERR, LOG_ERR, "ErrCodeSet Parameter is wrong, ErrCodeMax = %d, ErrCode = %d\r\n", ErrCodeMax, ErrCodeNo);
			return -1;
		}
		sgPErrCodeInfo[ErrCodeNo].b1ErrCode = 1;
	}
	if (ErrCodeNo != i32GetPara(PARA_ErrCode0))//��λ��������˳��
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
* Descriptio: ??��?��???��??��??1��???? 
* Input: NULL
* Output: ������С������
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
* Descriptio: ??��?��???��??��??1��???? 
* Input: NULL
* Output: �й��Ϸ���1���޹��Ϸ���0�����޷���-1
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
* Input: ErrCodeNo: ??��|��?�����̨�
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


//23��12��1�����ơ���ȡ��ǰ������������Ӧ�Ĺ�����
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
	{.b1ErrCode = 0, .b1NoAct = 1, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֹͣ���� 	//1	ϵͳ��ʼ������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֹͣ���� 	//2	ϵͳͨ�Ŵ���
	{.b1ErrCode = 0, .b1NoAct = 1, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֹͣ���� 	//3	��Чѡ�����ô���
	{.b1ErrCode = 0, .b1NoAct = 1, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֹͣ���� 	//4	�������ݴ���
	{.b1ErrCode = 0, .b1NoAct = 1, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֹͣ���� 	//5	﮵��ͨѶ��ʧ
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֹͣPCU  	//6	�ϵ�ʱ������������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֹͣPCU  	//7	�ϵ�ʱ���ٰ�������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֹͣPCU  	//8	�ϵ�ʱ���߰�������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//ֻ�Ǳ���   	//9	GPS���Ӵ���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֻ���½� 	//10	���Ӵ�������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//����		//11	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//������ֹͣ���е��̿���	//12	����ʱ�����������½���ť�򿪴���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֻ���½� 	//13	BMS-����²����2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 1, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//���͹������ 	//14	BMS-����¶ȹ���1
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֻ���½� 	//15	BMS-�ŵ��¶ȹ���2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 1, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//���͹������ 	//16	BMS-�ŵ��������1
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֻ���½� 	//17	BMS-�ŵ��������2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 1, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//���͹������ 	//18	�Ӷ���������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//����		//19	BMS-�ܵ�ѹ����1
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֻ���½� 	//20	BMS-�ܵ�ѹ����2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 1, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//���͹������ 	//21	BMS-�����ѹ����1
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֻ���½� 	//22	BMS-�����ѹ����2
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//����		//23	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//����		//24	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//����//		//25	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//����//		//26	
	{.b1ErrCode = 0, .b1NoAct = 1, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֹͣ���� 	//27	�½���2����
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//����//		//28	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//����//		//29	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֻ���½� 	//30	BMS-���ѹ�����
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֹͣ�������ߡ�ֻת���½�	//31	ѹ������������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֹͣ�������ߡ�ֻת���½�	//32	�Ƕȴ���������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 1, .b1BackWard = 0, .b1ForWard = 0, },	//��ֹ����	//33	������ʹ���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//����//		//34	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֹͣ�������ߡ�ֻת���½�	//35	���ر궨��
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 1, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//���͹������ 	//36	��ص�����һ������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//����//		//37	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֹͣ�������ߡ�ֻת���½�	//38	δ�궨��ɻ�궨ʧ��
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֹͣ�������ߡ�ֻת���½�	//39	ͨ�Ź���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//����//		//40	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 1, .b1BackWard = 0, .b1ForWard = 0, },	//��ֹ����	//41	ƽ̨һ������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//ֻ�Ǳ���   	//42	����ʱ��ƽ̨����ת��ť���´���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//ֻ�Ǳ���   	//43	����ʱ��ƽ̨����ת��ť���´���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֹͣ�������ߡ�ֻת���½�	//44	ƽ̨��������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//����//		//45	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֹͣPCU	//46	����ʱ��ƽ̨�ֱ�ʹ�ܿ��ذ�ť���´���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 1, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//���͹������ 	//47	����ʱ��ƽ̨�ֱ�������λ����
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//����//		//48	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//����//		//49	��������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//����//		//50	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//����//		//51	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֻ���½� 	//52	ǰ��������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֻ���½� 	//53	���˷�����
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֻ���½� 	//54	����������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֹͣ���� 	//55	�½�������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֻ���½� 	//56	��ת������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֻ���½� 	//57	��ת������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//����//		//58	ɲ��������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֻ���½� 	//59	����������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//ֻ�Ǳ���   	//60	����������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֹͣ���� 	//61	��������������������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֻ���½� 	//62	������Ӳ���𻵹���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֻ���½� 	//63	�õ�����·����
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֻ���½� 	//64	�������·����
	{.b1ErrCode = 0, .b1NoAct = 1, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֹͣ���� 	//65	���Ƶ�ѹ5V����
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֻ���½� 	//66	����ʱ����⵽��ת����·���·
	{.b1ErrCode = 0, .b1NoAct = 1, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֹͣ���� 	//67	���Ƶ�ѹ12V����
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֻ���½� 	//68	��ص͵�����������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//����//		//69	����λ��������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֻ���½� 	//70	������ĸ�ߵ�ѹ���߹���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֻ���½� 	//71	Ԥ�����
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 1, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//���͹������ 	//72	������ĸ�ߵ�ѹ���͹���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 1, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//���͹������ 	//73	���������¹���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 1, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//���͹������ 	//74	����������һ������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 1, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//���͹������ 	//75	�õ���¶�һ������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֻ���½� 	//76	�õ������������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֻ���½� 	//77	����������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֻ���½� 	//78	�õ�����������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֻ���½� 	//79	�õ���¶ȶ�������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//ֻ�Ǳ���   	//80	���� 80%���ر���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֻ���½� 	//81	�������¶ȶ�������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 1, .b1ForWard = 1, },	//��ֹ����	//82	��ɲ������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 1, .b1ForWard = 1, },	//��ֹ����	//83	��ɲ������
	{.b1ErrCode = 0, .b1NoAct = 1, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֹͣ���� 	//84	�õ����ת��ʧ��
	{.b1ErrCode = 0, .b1NoAct = 1, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֹͣ���� 	//85	��ǣ�������ת��ʧ��
	{.b1ErrCode = 0, .b1NoAct = 1, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֹͣ���� 	//86	��ǣ�������ת��ʧ��
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//����//		//87	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//����//		//88	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//ֻ�Ǳ���   	//89	����������ʱ���������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//ֻ�Ǳ���   	//90	���� 90%���ر���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 1, .b1ForWard = 1, },	//��ֹ����	//91	����������������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 1, .b1ForWard = 1, },	//��ֹ����	//92	�ҵ��������������
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//����//		//93	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//����//		//94	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//����//		//95	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//����//		//96	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//����//		//97	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//����//		//98	
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 0, .b1BackWard = 0, .b1ForWard = 0, },	//ֻ�Ǳ���   	//99	���� 99%���ر���
	{.b1ErrCode = 0, .b1NoAct = 1, .b1Pcu = 1, .b1LiftSpd = 1, .b1Down = 1, .b1Left = 1, .b1Right = 1, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֹͣ���� 	//100	ƽ̨���ر���
	{.b1ErrCode = 0, .b1NoAct = 0, .b1Pcu = 0, .b1LiftSpd = 0, .b1Down = 0, .b1Left = 0, .b1Right = 0, .b1Up = 1, .b1BackWard = 1, .b1ForWard = 1, },	//ֹͣ�������ߡ�ֻת���½�	//101	������б������ȫ�޶�����
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
* Input: ErrCodeNo: ��Ӧ��ͨ��
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
* Input: ErrCodeNo: ��Ӧ��ͨ��
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
* Descriptio: ��ȡ������ȼ������� 
* Input: NULL
* Output: ������ȼ��Ĺ�����
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
* Descriptio: ��ȡ������ȼ������� 
* Input: NULL
* Output: ������ȼ��Ĺ�����
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