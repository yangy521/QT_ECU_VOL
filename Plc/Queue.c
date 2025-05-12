/*******************************************************************************
* Filename: Queue.c	                                             	 		   *
* Description:											   			 		   												*
* Author:                                                           		   *
* Date:     														 		   														*
* Revision:															 		 														  *
*******************************************************************************/
#include  "Queue.h"
/*********************************************************************************************************
** ��������: QueueCreate
** ��������: ��ʼ�����ݶ���
** �䡡��: Buf      ��Ϊ���з���Ĵ洢�ռ��ַ
**         SizeOfBuf��Ϊ������ȣ�Ԫ�ظ���
** �䡡��: NOT_OK:��������;         QUEUE_OK:�ɹ�
********************************************************************************************************/
 unsigned char QueueCreate(void *Buf, unsigned long SizeOfBuf)       
{
    DataQueue *Queue;
    
    if (Buf != NULL)        /* �жϲ����Ƿ���Ч */
    {
        Queue = (DataQueue *)Buf;
                                                                /* ��ʼ���ṹ������ */
        Queue->MaxData = SizeOfBuf;               /* ������п��Դ洢��������Ŀ */
        Queue->End = Queue->Buffer + Queue->MaxData;      /* �������ݻ���Ľ�����ַ */	
        Queue->Out = Queue->Buffer;
        Queue->In = Queue->Buffer;
        Queue->NData = 0;

        return QUEUE_OK;
    }
    else
    {
        return NOT_OK;
    }
}

/*********************************************************************************************************
** ��������: QueueRead
** ��������: ��ȡ�����е�����
** �䡡��: Ret:�洢���ص���Ϣ�ĵ�ַ;         Buf:ָ����е�ָ��
** �䡡��: NOT_OK     ����������;         QUEUE_OK   ���յ���Ϣ;         QUEUE_EMPTY������Ϣ
********************************************************************************************************/
unsigned char QueueRead(QUEUE_DATA_TYPE *Ret, void *Buf)
{
    unsigned char err;
    DataQueue *Queue;

    err = NOT_OK;
    if (Buf != NULL)                                            /* �����Ƿ���Ч */
    {                                                           /* ��Ч */
        Queue = (DataQueue *)Buf;

        if ((unsigned long)Queue->In !=  (unsigned long)Queue->Out)                                   /* �����Ƿ�Ϊ�� */
        {                                                       /* ����         */
            *Ret = Queue->Out[0];                               /* ���ݳ���     */
            Queue->Out++;                                       /* ��������ָ�� */
            if (Queue->Out >= Queue->End)
            {
                Queue->Out = Queue->Buffer;
            }
            Queue->NData--;                                     /* ���ݼ���      */
            err = QUEUE_OK;
        }
        else
        {                                                       /* ��              */
            err = QUEUE_EMPTY;
        }

    }
    return err;
}

/*********************************************************************************************************
** ��������: QueueWrite
** ��������: FIFO��ʽ��������
** �䡡��: Buf :ָ����е�ָ��;         Data:��Ϣ����
** �䡡��: NOT_OK   :��������;         QUEUE_FULL:������;         QUEUE_OK  :���ͳɹ�
********************************************************************************************************/
unsigned char QueueWrite(void *Buf, QUEUE_DATA_TYPE Data)
{
 unsigned char err;
    DataQueue *Queue;
	QUEUE_DATA_TYPE* pInNext; 

    err = NOT_OK;
    if (Buf != NULL)                                                    /* �����Ƿ���Ч */
    {
        Queue = (DataQueue *)Buf;
        pInNext = Queue->In + 1;
		if ((unsigned long)pInNext >= (unsigned long)Queue->End)
			pInNext = Queue->Buffer;
        if ((unsigned long)pInNext != (unsigned long)Queue->Out)                              /* �����Ƿ���  */
        {                                                               /* ����        */
            Queue->In[0] = Data;                                        /* �������    */
            Queue->In = pInNext;                                                /* �������ָ��*/
            Queue->NData++;                                             /* ��������    */
            err = QUEUE_OK;
        }
        else
        {                                                               /* ��           */
            err = QUEUE_FULL;
            
        }
    }
    return err;
}

/*********************************************************************************************************
** ��������: QueueWriteFront
** ��������: LIFO��ʽ��������
** �䡡��: Buf:ָ����е�ָ��;         Data:��Ϣ����
** �䡡��: QUEUE_FULL:������;         QUEUE_OK:���ͳɹ�
********************************************************************************************************/
unsigned char QueueWriteFront(void *Buf, QUEUE_DATA_TYPE Data)
{
    unsigned char err;
    DataQueue *Queue;

    err = NOT_OK;
    if (Buf != NULL)                                                    /* �����Ƿ���Ч */
    {
        Queue = (DataQueue *)Buf;

        if (Queue->NData < Queue->MaxData)                              /* �����Ƿ���  */
        {                                                               /* ���� */
            Queue->Out--;                                               /* ��������ָ�� */
            if (Queue->Out < Queue->Buffer)
            {
                Queue->Out = Queue->End - 1;
            }
            Queue->Out[0] = Data;                                       /* �������     */
            Queue->NData++;                                             /* ������Ŀ���� */
            err = QUEUE_OK;
        }
        else
        {                                                               /* ��           */
            err = QUEUE_FULL;
            
        }

    }
    return err;
}

/*********************************************************************************************************
** ��������: QueueNData
** ��������: ȡ�ö�����������
** �䡡��: Buf:ָ����е�ָ��
** �䡡��: ��Ϣ��
********************************************************************************************************/
unsigned short QueueNData(void *Buf)
{
    unsigned short temp;
    
    temp = 0;                                                   /* ������Ч����0 */
    if (Buf != NULL)
    {
        temp = ((DataQueue *)Buf)->NData;
    }
    return temp;
}

/*********************************************************************************************************
** ��������: QueueSize
** ��������: ȡ�ö���������
** �䡡��: Buf:ָ����е�ָ��
** �䡡��: ����������
********************************************************************************************************/
unsigned short QueueSize(void *Buf)
{
    unsigned short temp;
    
    temp = 0;                                                   /* ������Ч����0 */
    if (Buf != NULL)
    {
        temp = ((DataQueue *)Buf)->MaxData;
    }
    return temp;
}

/*********************************************************************************************************
** ��������: OSQFlush
** ��������: ��ն���
** �䡡��: Buf:ָ����е�ָ��
** �䡡��: ��
********************************************************************************************************/
void QueueFlush(void *Buf)
{
    DataQueue *Queue;
    
    if (Buf != NULL)                                                /* �����Ƿ���Ч */
    {                                                               /* ��Ч         */
        Queue = (DataQueue *)Buf;
        Queue->Out = Queue->Buffer;
        Queue->In = Queue->Buffer;
        Queue->NData = 0;                                           /* ������ĿΪ0 */
    }
}

/*********************************************************************************************************
** ��������: GetDataByte
** ��������: �����������ݲ�ֳɵ��ֽ�
** �䡡��: ulData:���������ݣ�ucNumber:�ֽ�λ��
** �䡡��: ���ֽ�
********************************************************************************************************/
unsigned char GetDataByte(unsigned long ulData,unsigned char ucNumber)
{
	unsigned char ucByte;
	ucByte=0;
	if(ucNumber==1)
	{
		ulData=ulData&0x000000FF;
		ucByte=ulData;	
	}
	if(ucNumber==2)
	{
		ulData=ulData>>8;
		ucByte=ulData&0x000000FF;	
	}
	if(ucNumber==3)
	{
		ulData=ulData>>16;
		ucByte=ulData&0x000000FF;	
	}
	if(ucNumber==4)
	{
		ulData=ulData>>24;
		ucByte=ulData&0x000000FF;	
	}
	return ucByte;
}
