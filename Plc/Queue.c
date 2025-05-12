/*******************************************************************************
* Filename: Queue.c	                                             	 		   *
* Description:											   			 		   												*
* Author:                                                           		   *
* Date:     														 		   														*
* Revision:															 		 														  *
*******************************************************************************/
#include  "Queue.h"
/*********************************************************************************************************
** 函数名称: QueueCreate
** 功能描述: 初始化数据队列
** 输　入: Buf      ：为队列分配的存储空间地址
**         SizeOfBuf：为队列深度，元素个数
** 输　出: NOT_OK:参数错误;         QUEUE_OK:成功
********************************************************************************************************/
 unsigned char QueueCreate(void *Buf, unsigned long SizeOfBuf)       
{
    DataQueue *Queue;
    
    if (Buf != NULL)        /* 判断参数是否有效 */
    {
        Queue = (DataQueue *)Buf;
                                                                /* 初始化结构体数据 */
        Queue->MaxData = SizeOfBuf;               /* 计算队列可以存储的数据数目 */
        Queue->End = Queue->Buffer + Queue->MaxData;      /* 计算数据缓冲的结束地址 */	
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
** 函数名称: QueueRead
** 功能描述: 获取队列中的数据
** 输　入: Ret:存储返回的消息的地址;         Buf:指向队列的指针
** 输　出: NOT_OK     ：参数错误;         QUEUE_OK   ：收到消息;         QUEUE_EMPTY：无消息
********************************************************************************************************/
unsigned char QueueRead(QUEUE_DATA_TYPE *Ret, void *Buf)
{
    unsigned char err;
    DataQueue *Queue;

    err = NOT_OK;
    if (Buf != NULL)                                            /* 队列是否有效 */
    {                                                           /* 有效 */
        Queue = (DataQueue *)Buf;

        if ((unsigned long)Queue->In !=  (unsigned long)Queue->Out)                                   /* 队列是否为空 */
        {                                                       /* 不空         */
            *Ret = Queue->Out[0];                               /* 数据出队     */
            Queue->Out++;                                       /* 调整出队指针 */
            if (Queue->Out >= Queue->End)
            {
                Queue->Out = Queue->Buffer;
            }
            Queue->NData--;                                     /* 数据减少      */
            err = QUEUE_OK;
        }
        else
        {                                                       /* 空              */
            err = QUEUE_EMPTY;
        }

    }
    return err;
}

/*********************************************************************************************************
** 函数名称: QueueWrite
** 功能描述: FIFO方式发送数据
** 输　入: Buf :指向队列的指针;         Data:消息数据
** 输　出: NOT_OK   :参数错误;         QUEUE_FULL:队列满;         QUEUE_OK  :发送成功
********************************************************************************************************/
unsigned char QueueWrite(void *Buf, QUEUE_DATA_TYPE Data)
{
 unsigned char err;
    DataQueue *Queue;
	QUEUE_DATA_TYPE* pInNext; 

    err = NOT_OK;
    if (Buf != NULL)                                                    /* 队列是否有效 */
    {
        Queue = (DataQueue *)Buf;
        pInNext = Queue->In + 1;
		if ((unsigned long)pInNext >= (unsigned long)Queue->End)
			pInNext = Queue->Buffer;
        if ((unsigned long)pInNext != (unsigned long)Queue->Out)                              /* 队列是否满  */
        {                                                               /* 不满        */
            Queue->In[0] = Data;                                        /* 数据入队    */
            Queue->In = pInNext;                                                /* 调整入队指针*/
            Queue->NData++;                                             /* 数据增加    */
            err = QUEUE_OK;
        }
        else
        {                                                               /* 满           */
            err = QUEUE_FULL;
            
        }
    }
    return err;
}

/*********************************************************************************************************
** 函数名称: QueueWriteFront
** 功能描述: LIFO方式发送数据
** 输　入: Buf:指向队列的指针;         Data:消息数据
** 输　出: QUEUE_FULL:队列满;         QUEUE_OK:发送成功
********************************************************************************************************/
unsigned char QueueWriteFront(void *Buf, QUEUE_DATA_TYPE Data)
{
    unsigned char err;
    DataQueue *Queue;

    err = NOT_OK;
    if (Buf != NULL)                                                    /* 队列是否有效 */
    {
        Queue = (DataQueue *)Buf;

        if (Queue->NData < Queue->MaxData)                              /* 队列是否满  */
        {                                                               /* 不满 */
            Queue->Out--;                                               /* 调整出队指针 */
            if (Queue->Out < Queue->Buffer)
            {
                Queue->Out = Queue->End - 1;
            }
            Queue->Out[0] = Data;                                       /* 数据入队     */
            Queue->NData++;                                             /* 数据数目增加 */
            err = QUEUE_OK;
        }
        else
        {                                                               /* 满           */
            err = QUEUE_FULL;
            
        }

    }
    return err;
}

/*********************************************************************************************************
** 函数名称: QueueNData
** 功能描述: 取得队列中数据数
** 输　入: Buf:指向队列的指针
** 输　出: 消息数
********************************************************************************************************/
unsigned short QueueNData(void *Buf)
{
    unsigned short temp;
    
    temp = 0;                                                   /* 队列无效返回0 */
    if (Buf != NULL)
    {
        temp = ((DataQueue *)Buf)->NData;
    }
    return temp;
}

/*********************************************************************************************************
** 函数名称: QueueSize
** 功能描述: 取得队列总容量
** 输　入: Buf:指向队列的指针
** 输　出: 队列总容量
********************************************************************************************************/
unsigned short QueueSize(void *Buf)
{
    unsigned short temp;
    
    temp = 0;                                                   /* 队列无效返回0 */
    if (Buf != NULL)
    {
        temp = ((DataQueue *)Buf)->MaxData;
    }
    return temp;
}

/*********************************************************************************************************
** 函数名称: OSQFlush
** 功能描述: 清空队列
** 输　入: Buf:指向队列的指针
** 输　出: 无
********************************************************************************************************/
void QueueFlush(void *Buf)
{
    DataQueue *Queue;
    
    if (Buf != NULL)                                                /* 队列是否有效 */
    {                                                               /* 有效         */
        Queue = (DataQueue *)Buf;
        Queue->Out = Queue->Buffer;
        Queue->In = Queue->Buffer;
        Queue->NData = 0;                                           /* 数据数目为0 */
    }
}

/*********************************************************************************************************
** 函数名称: GetDataByte
** 功能描述: 将长整形数据拆分成单字节
** 输　入: ulData:长整形数据；ucNumber:字节位数
** 输　出: 单字节
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
