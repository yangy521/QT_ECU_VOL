#include "Device.h"
#include "Userdef.h"
#include "PARA.h"

//static const uint32_t  u32CanBuad __attribute__ ((at(CAN_BAUDRATE_ADDRESS))) = 500;
//const uint32_t  u32CanBuad __attribute__ ((section(".ARM.__at_0x08006800"))) = 500;

//const uint8_t version[6] __attribute__((section(".ARM.__AT_0x08003800")))= {0x11,0x12,0x13,0x14,0x15,0x16};
//uint8_t version1[12] __attribute__((at(0x081F8010)))= {0x11,0x12,0x13,0x14,0x15,0x16};

/*lilu 20231110 调用时记得修改延时*/
static uint16_t u16GetBaudRate(void)
{
	
    uint32_t u32Temp;    
    uint32_t i = 0;
	uint32_t u32LevelCnt = 0;
	uint8_t u8Flag = 0;
	
	#ifdef BAUDRATE_SYCHRON
	u32Temp = *(uint32_t *)CAN_BAUDRATE_ADDRESS;   

/*单MCU*/
#if (SINGLE_MCU == MCU_TYPE)
    {
        
    }
/*双MCU主*/
#elif (MULTIPLE_MCU_MST == MCU_TYPE)
    {	
		/*lilu 20231110 初始化以及设置波特率对应的IO电平*/
		CANBAUD_SYNC_Init();
		switch (u32Temp)
		{
			case CAN_BAUD_RATE_500K:
				CANBAUD_SYNC_SW1(1);
				CANBAUD_SYNC_SW2(0);
				break;
			case CAN_BAUD_RATE_250K:
				CANBAUD_SYNC_SW1(0);
				CANBAUD_SYNC_SW2(1);
				break;
			case CAN_BAUD_RATE_125K:
				CANBAUD_SYNC_SW1(0);
				CANBAUD_SYNC_SW2(0);
				break;
			default:
				CANBAUD_SYNC_SW1(1);
				CANBAUD_SYNC_SW2(0);
				break;
		}
		CANBAUD_SYNC_MST(0);
		/*lilu 20231110 等待从机设置对应的电平*/
		i = 0;
		u32LevelCnt = 0;
		while(i++ < 0xFFFFFFFF)
		{
			if (0 == (i % 1000))
			{
				if (1 == CANBAUD_SYNC_SLV_Read())
				{
					u32LevelCnt = 0;
				}
				else
				{
					u32LevelCnt++;
					if (u32LevelCnt >= 10)		/*lilu 20231110 TimerOut 10ms*/
					{
						break;
					}
				}
			}
		}
		/*lilu 20231110 出现异常状况*/
		if (0xFFFFFFFF == i)
		{
			u8Flag |= 1 << 0;
		}
		/*lilu 20231110 等待从机设置对应的电平*/
		i = 0;
		u32LevelCnt = 0;
		while(i++ < 0xFFFFFFFF)
		{
			if (0 == (i % 1000))
			{
				if (0 == CANBAUD_SYNC_SLV_Read())
				{
					u32LevelCnt = 0;
				}
				else
				{
					u32LevelCnt++;
					if (u32LevelCnt >= 10)		/*lilu 20231110 TimerOut 10ms*/
					{
						break;
					}
				}
			}
		}
		/*lilu 20231110 出现异常状况*/
		if (0xFFFFFFFF == i)
		{
			u8Flag |= 1 << 1;
		}
		/*lilu 20231110 异常状况走默认波特率500K*/
		if (0 != u8Flag)
		{
			u32Temp = CAN_BAUD_RATE_500K;
		}
		CANBAUD_SYNC_MST(1);
		CANBAUD_SYNC_SW1(1);
		CANBAUD_SYNC_SW2(1);
    }
/*双MCU从*/
#else
    /*从MCU*/
    {
		/*lilu 20231110 初始化*/
		CANBAUD_SYNC_Init();
		/*lilu 2023110 等待电平设置完成*/
		i = 0;
		u32LevelCnt = 0;
		while(i++ < 0xFFFFFFFF)
		{
			if(0 == (i % 1000))
			{
				if (1 == CANBAUD_SYNC_MST_Read())
				{
					u32LevelCnt = 0;
				}
				else
				{
					u32LevelCnt++;
					if (u32LevelCnt >= 15)		/*lilu 20231110 TimerOut 10ms*/
					{
						break;
					}
				}
			}
		}
		/*lilu 2023110 出现异常状况*/
		if (0xFFFFFFFF == i)
		{
			u8Flag |= 1 << 0;
		}
		/*lilu 20231110 电平依然有效获取波特率*/
		if (0 == CANBAUD_SYNC_MST_Read())
		{
			if (0 == CANBAUD_SYNC_SW1_Read() && 0 == CANBAUD_SYNC_SW2_Read())
			{
				u32Temp = CAN_BAUD_RATE_125K;
			}
			else if (0 == CANBAUD_SYNC_SW1_Read() && 1 == CANBAUD_SYNC_SW2_Read())
			{
				u32Temp = CAN_BAUD_RATE_250K;
			}
			else if (1 == CANBAUD_SYNC_SW1_Read() && 0 == CANBAUD_SYNC_SW2_Read())
			{
				u32Temp = CAN_BAUD_RATE_500K;
			}
			else
			{
				u32Temp = CAN_BAUD_RATE_500K;
			}
		}
		else
		{
			u32Temp = CAN_BAUD_RATE_500K;
		}
		/*lilu 20231110 告知主机我已获取到波特率*/
		CANBAUD_SYNC_SLV(0);
		i = 0;
		while(i++ < 50000)	/*lilu 20231110 Need Exceed 10ms*/
		{
			;
		}
		/*lilu 20231110 告知主机可以往下走了*/
		CANBAUD_SYNC_SLV(1);
		i = 0;
		while(i++ < 50000)	/*lilu 20231110 Need Exceed 10ms*/
		{
			;
		}
		if (0 != u8Flag)
		{
			u32Temp = CAN_BAUD_RATE_500K;
		}
	}
#endif
#endif	//BAUDRATE_SYCHRON
    return (uint16_t)u32Temp;
}



void vBspInit(void)
{
	uint32_t u32CanBaud = u16GetBaudRate(); 
	uint8_t u8Tmp ;
	/*IO Clock Enable*/
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_GPIOC);
	rcu_periph_clock_enable(RCU_GPIOD);
	rcu_periph_clock_enable(RCU_GPIOE);
	rcu_periph_clock_enable(RCU_GPIOF);	
	rcu_periph_clock_enable(RCU_AF);
	rcu_periph_clock_enable(RCU_DMA0);
	rcu_periph_clock_enable(RCU_DMA1);

	/*DO Initial*/
	vDrvDoInit();
	/*Di Initial*/
	vDrvDiInit();
	/*Ad Initial*/
	vDrvAdInit();
	/*Spi Initial*/
	vDrvSpiInit();
	/*Pwm Initial*/
	vDrvPwmInit();
	/*Uart Initial*/
	vDrvUartInit();
	/*Can Initial*/
	#if 0		//23.11.21 SJ暂时屏蔽波特率同步功能
	{
		uint32_t u32Temp = u32CanBaud;
		switch (u32Temp)
		{
			case CAN_BAUD_RATE_500K:
				vDrvCanInit(CAN_BAUD_RATE_500K);
				break;
			case CAN_BAUD_RATE_250K:
				vDrvCanInit(CAN_BAUD_RATE_250K);
				break;
			case CAN_BAUD_RATE_125K:
				vDrvCanInit(CAN_BAUD_RATE_125K);
				break;
			default:
				vDrvCanInit(CAN_BAUD_RATE_500K);
				break;
		}
	}
	#endif
//	vDrvCanInit(CAN_BAUD_RATE_500K);

#ifdef CAN_Baudrate  //userdef
	vDrvCanInit(CAN_Baudrate);
	/*lilu 20231227 bsp阶段不能获取参数，参数还未进行初始化*/
//#else
//	tmp = i32GetPara(PARA_CanBaudRate);	
//	if ((CAN_BAUD_RATE_500K == tmp) || (CAN_BAUD_RATE_250K == tmp) || (CAN_BAUD_RATE_125K == tmp))
//	{
//		vDrvCanInit(tmp);
//	}
//	else
//	{
//		vDrvCanInit(CAN_BAUD_RATE_250K);
//	}
#endif

#ifdef PRJ_RELEASE
//	vDrvwdgInit();
#endif
}

