#include <gd32f30x.h>
#include <stdbool.h>
#include "canfestival.h"
#include "timer_canfestival.h"
#include "CanCom.h"
#include "UserCanComm.h"

#define	CANOPEN_SEND_NUM		10

extern void vTestSlaveSetObjict(void);

/************************** Modul variables **********************************/
// Store the last timer value to calculate the elapsed time
//unsigned int TimeCNT=0;//
//unsigned int NextTime=0;// 
//unsigned int TIMER_MAX_COUNT=70000;//
unsigned int TimeCNT=0;//
unsigned int NextTime=0;
unsigned int TIMER_MAX_COUNT=70000;
static TIMEVAL last_time_set = TIMEVAL_MAX;//

typedef struct
{
	uint32_t u32CanId;
	CanOpenSendCbt CallBack;
}xCanOpenSendCb;

TIMEVAL last_counter_val = 0;
TIMEVAL elapsed_time = 0;

static xCanOpenSendCb sgCanOpenSendCb[CANOPEN_SEND_NUM] = 
{
	{.u32CanId = 0xFFFFFFFF,},
	{.u32CanId = 0xFFFFFFFF,},
	{.u32CanId = 0xFFFFFFFF,},
	{.u32CanId = 0xFFFFFFFF,},
	{.u32CanId = 0xFFFFFFFF,},
	{.u32CanId = 0xFFFFFFFF,},
	{.u32CanId = 0xFFFFFFFF,},
	{.u32CanId = 0xFFFFFFFF,},
	{.u32CanId = 0xFFFFFFFF,},
	{.u32CanId = 0xFFFFFFFF,},
};

UNS8 u8CanOpenSendReg(UNS32 u32CanId, CanOpenSendCbt CallBack)
{
	uint8_t i = 0;
	uint8_t res = 0;
	
	for (i=0; i<CANOPEN_SEND_NUM; i++)
	{
		if (sgCanOpenSendCb[i].u32CanId == u32CanId)
		{
			sgCanOpenSendCb[i].CallBack = CallBack;
			res = 1;
		}
		else if(0xFFFFFFFF == sgCanOpenSendCb[i].u32CanId)
		{
			break;
		}
	}
	
	if (i < CANOPEN_SEND_NUM)
	{
		sgCanOpenSendCb[i].u32CanId = u32CanId;
		sgCanOpenSendCb[i].CallBack = CallBack;
		res = 1;
	}
	return res;
}


void setTimer(TIMEVAL value)
{
	uint32_t timer = timer_counter_read(TIMER5);        // Copy the value of the running timer
	elapsed_time += timer - last_counter_val;
	last_counter_val = 65535 - value;
	timer_counter_value_config(TIMER5, 65535 - value);
	timer_enable(TIMER5);
}
TIMEVAL getElapsedTime(void)
{
	uint32_t timer = timer_counter_read(TIMER5);        // Copy the value of the running timer
	if(timer < last_counter_val)
	{
		timer += 65535;
	}
	TIMEVAL elapsed = timer - last_counter_val + elapsed_time;
	return elapsed;
}
void timerForCan(void)
{
	TimeCNT++;
	if (TimeCNT>=TIMER_MAX_COUNT)
	{
		TimeCNT=0;
	}
	if (TimeCNT==NextTime)
	{
		TimeDispatch();
	}
}
unsigned char canSend(CAN_PORT notused, Message *m)
{ 
	unsigned char ret;
	unsigned char i;
	tCanFrame tmp;
	
	tmp.u32ID = (uint32_t)(m->cob_id);
	tmp.u8Rtr = m->rtr;
	tmp.u16DataLength = m->len;
	for(i = 0; i < m->len; i++)                                 
	{
		tmp.u8Data[i] = m->data[i];
	}
	//can_message_transmit(CAN0, &transmit_message);
	for(i = 0; i <CANOPEN_SEND_NUM; i++) 
	{
		if (tmp.u32ID == sgCanOpenSendCb[i].u32CanId)
		{
			if (NULL != sgCanOpenSendCb[i].CallBack)
			{
				sgCanOpenSendCb[i].CallBack(tmp.u32ID, tmp.u8Data);
			}
		}
	}
	i32CanWrite(Can0, &tmp);
	return 0;
}

void timer5_init(uint32_t psr, uint32_t arr)
{

	timer_parameter_struct timer_init_struct;
	rcu_periph_clock_enable(RCU_TIMER5);
	timer_deinit(TIMER5);
	timer_init_struct.prescaler			= psr;	/* ????? */
	timer_init_struct.period			= arr;	/* ?????? */
	timer_init_struct.alignedmode		= TIMER_COUNTER_EDGE;	/* ???????,????(???5??)*/
	timer_init_struct.counterdirection	= TIMER_COUNTER_UP;		/* ???????,??(???5??)*/
	timer_init_struct.clockdivision		= TIMER_CKDIV_DIV1;		/* DTS?????(???5??) */
	timer_init_struct.repetitioncounter = 0;					/* ???????(???5??)*/
	timer_init(TIMER5, &timer_init_struct);

	nvic_irq_enable(TIMER5_IRQn, 1, 1); 

    timer_interrupt_enable(TIMER5, TIMER_INT_UP);

	timer_enable(TIMER5);
}

void TIMER5_IRQHandler(void)
{

	if(timer_interrupt_flag_get(TIMER5, TIMER_INT_FLAG_UP)!= RESET)
	{
		timer_interrupt_flag_clear(TIMER5, TIMER_INT_FLAG_UP);
		last_counter_val = 0;
		elapsed_time = 0;
		TimeDispatch();
	}
}
void Canopen_int(uint8_t nodeId)
{
	UNS32 i;
//	unsigned char nodeID=0x21;
	extern CO_Data TestSlave_Data;
	
	vTestSlaveSetObjict();
	//timer5_init(1200-1, 100-1);
	timer5_init(120 - 1,  65536 - 1);
	//timer5_init(120 - 1,  50000 - 1);
	setNodeId(&TestSlave_Data, nodeId);
	 /* ** Initialize the transmit PDO communication parameters. Only for 0x1800 to 0x1803 */
#if (USER_TYPE == USER_ZHONGLI_DGC)
	{
		CO_Data* d = &TestSlave_Data;
		UNS8 i = 0;
		UNS16 offset = d->firstIndex->PDO_TRS;
		UNS16 lastIndex = d->lastIndex->PDO_TRS;
		UNS32 cobID[] = {0x210, 0x310, 0x410, 0x510};
		i = 0;
		if( offset ) while ((offset <= lastIndex) && (i < 4)) 
		{
//			if((*(UNS32*)d->objdict[offset].pSubindex[1].pObject == cobID[i] + *d->bDeviceNodeId)||(*d->bDeviceNodeId==0xFF))
			*(UNS32*)d->objdict[offset].pSubindex[1].pObject = cobID[i] + nodeId;
			i ++;
			offset ++;
		}
	}
#endif
	setState(&TestSlave_Data, Initialisation); // Init the state  
	setState(&TestSlave_Data, Operational); // Init the state  
	
}
