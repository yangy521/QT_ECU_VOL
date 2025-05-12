#if 0
#include "Device.h"
#include "Eeprom.h"
#include "ErrCode.h"
#include "Log.h"
#include "NetTimer.h"


#define EEPROM_LENGTH	256
typedef struct
{
	uint16_t u16Addr;
	uint16_t u16Data;
}xEepromInfo;

typedef struct
{
	uint16_t	u16EepromWr;
	uint16_t	u16EepromRd;
	xEepromInfo EepromData[EEPROM_LENGTH];
	union
	{
		uint32_t u32Data;
		struct
		{
			uint32_t b1Flag: 1;
			uint32_t b7RemainCnt: 7;
			uint32_t b1ErrFlag: 1;
			uint32_t b23Reserve: 23;
		};
	};
}xEepromArray;

xEepromArray sgEepromArray = {.u16EepromWr = 0, .u16EepromRd = 0, .u32Data = 0};

/*延时ulTime个指令周期,ulTime=10对应1us*/
static void vDelay(uint64_t u64Time)
{
	uint64_t i = 0;
	for (i=0; i<u64Time; i++)
	{
		__NOP();
	}
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
/*******************************************************************************
* Name: EepromWrite
* Description: EEPROM写入数据.
* Input: 
* Output: 0:Success;1:fail
* Author:
* Date:
* Revision:
*******************************************************************************/
static INT16U EepromWrite(INT16U address, INT16U data)
{

#if (CTLBOARD_TYPE ==_1231_G4)
	uint8_t ulTxData[4],ulRxDataL[2];
	INT16U ret;
	ret=0;	
	//
	//1、使能写操作
	//
	//触发上升沿
	SPI_CS_0;  //CS reset
	Delay(20);  
	SPI_CS_1;  //CS set
	//发送写使能操作码	
//	ulTxData= 0x04C0;
	ulTxData[0] = 0x04;
	ulTxData[1] = 0xc0;
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&ulTxData,(uint8_t*)&ulRxDataL,2, SPI_TIMEOUT);

	//
	// 2、写操作
	//
	//触发上升沿
	SPI_CS_0;  //CS reset
	Delay(20); 
	SPI_CS_1;  //CS set
	//发送写操作码、地址
	ulTxData[0] = ((0x0500 | address)>>8)&0xFF;
	ulTxData[1] = (0x0500 | address)&0xFF;
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&ulTxData,(uint8_t*)&ulRxDataL,2, SPI_TIMEOUT);
	//发送16位数据
	ulTxData[0] = (data&0xff00)>>8;
	ulTxData[1] = data&0x00ff;
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&ulTxData,(uint8_t*)&ulRxDataL,2, SPI_TIMEOUT);
	//触发上升沿,等待写操作完成(不小于6ms)
	SPI_CS_0;  //CS reset
	Delay(20); 
	SPI_CS_1;  //CS set
	Delay(150000);
	//
	// 3. 写禁止
	//
	//触发上升沿
	SPI_CS_0;  //CS reset
	Delay(20);
	SPI_CS_1;  //CS set
	//发送写禁止操作码
	ulTxData[0] = 0x04;
	ulTxData[1] = 0x00;
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&ulTxData,(uint8_t*)&ulRxDataL,2, SPI_TIMEOUT);
	SPI_CS_0;  //CS reset
#endif	//#if (CTLBOARD_TYPE ==_1231_G4)

#if ((CTLBOARD_TYPE ==_HB4_GD32) || (CTLBOARD_TYPE ==_HB6_GD32))
	INT16U ulTxData,ulRxData;
	INT16U ret;
	ret=0;	
	//
	//1、使能写操作
	//
	//触发上升沿
	SPI_CS_0;  //CS reset
	vDelay(20);  
	SPI_CS_1;  //CS set
	//发送写使能操作码	
	ulTxData= 0x04C0;
	spi_i2s_data_transmit(SPI2, ulTxData);
	while(spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE) == RESET);
	ulRxData = spi_i2s_data_receive(SPI2);
	(void)ulRxData;	//防止编译警告
	//
	// 2、写操作
	//
	//触发上升沿
	SPI_CS_0;  //CS reset
	vDelay(20); 
	SPI_CS_1;  //CS set
	//发送写操作码、地址
	ulTxData = 0x0500 | address;
	//等待发送操作完成
	while (spi_i2s_flag_get(SPI2, SPI_FLAG_TBE) == RESET);
	spi_i2s_data_transmit(SPI2, ulTxData);
	while (spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE) == RESET);
	ulRxData=spi_i2s_data_receive(SPI2);//接收读取数据	
	//发送16位数据
	ulTxData = data;
	//等待发送操作完成
	while (spi_i2s_flag_get(SPI2, SPI_FLAG_TBE) == RESET);
	spi_i2s_data_transmit(SPI2, ulTxData);
	while (spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE) == RESET);
	ulRxData=spi_i2s_data_receive(SPI2);//接收读取数据
	//触发上升沿,等待写操作完成(不小于6ms)	
	SPI_CS_0;  //CS reset
	vDelay(20); 
	SPI_CS_1;  //CS set
//	vDelay(150000);
	vDelay(80000);
	//
	// 3. 写禁止
	//
	//触发上升沿
	SPI_CS_0;  //CS reset
	vDelay(20);
	SPI_CS_1;  //CS set
	//发送写禁止操作码
	ulTxData = 0x0400;
	while (spi_i2s_flag_get(SPI2, SPI_FLAG_TBE) == RESET);
	spi_i2s_data_transmit(SPI2, ulTxData);
	while (spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE) == RESET);
	ulRxData=spi_i2s_data_receive(SPI2);//接收读取数据
	SPI_CS_0;  //CS reset
#endif	//#if (CTLBOARD_TYPE ==_HB4_GD32)	
	return ret;
}

/*******************************************************************************
* Name: EepromRead
* Description: EEPROM读取数据
* Input: 
* Output: 0:Success;1:fail
* Author:
* Date:
* Revision:
*******************************************************************************/
static INT16U EepromRead(INT16U address, INT16U* pdata)
{

#if (CTLBOARD_TYPE ==_1231_G4)
	uint8_t ulTxData[4],ulRxDataH[4],ulRxDataL[4];
	INT16U ret;
	ret=0;
	//触发上升沿
	SPI_CS_0;  //CS reset
	Delay(20);
	SPI_CS_1;  //CS set
	// 发送操作码和地址
//	ulTxData= (0x0600 | address)<<5;
	ulTxData[0] = (((0x0600 | address)<<5)>>8)&0xFF;
	ulTxData[1] = ((0x0600 | address)<<5)&0xFF;;
	ulTxData[2] = 0x00;
	ulTxData[3] = 0x00;
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&ulTxData,(uint8_t*)&ulRxDataH,2, SPI_TIMEOUT);

	// 发送无效数据，产生时钟信号，接收剩余读取数据 
	ulTxData[0] = 0x00;
	ulTxData[1] = 0x00;
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&ulTxData,(uint8_t*)&ulRxDataL,2, SPI_TIMEOUT);
	
	SPI_CS_0;  //CS reset

	*pdata = ((ulRxDataH[1]<<4)|(ulRxDataL[0]>>4))<<8;
	*pdata |= ((ulRxDataL[0]<<4)|(ulRxDataL[1]>>4))&0x00ff;
#endif	//#if (CTLBOARD_TYPE ==_1231_G4)

#if ((CTLBOARD_TYPE ==_HB4_GD32) || (CTLBOARD_TYPE ==_HB6_GD32))
	INT16U ulTxData,ulRxDataL,ulRxDataH;
	INT16U ret;
	ret=0;

	//触发上升沿
	SPI_CS_0;  //CS reset
	vDelay(20);
	SPI_CS_1;  //CS set
	// 发送操作码和地址
	ulTxData= (0x0600 | address)<<5;
	//等待发送操作完成
	while (spi_i2s_flag_get(SPI2, SPI_FLAG_TBE) == RESET);
	spi_i2s_data_transmit(SPI2, ulTxData);
	while (spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE) == RESET);
	ulRxDataH=spi_i2s_data_receive(SPI2);//接收读取数据	

	// 发送无效数据，产生时钟信号，接收剩余读取数据 
	ulTxData = 0x00;
	//等待发送操作完成
	while (spi_i2s_flag_get(SPI2, SPI_FLAG_TBE) == RESET);
	spi_i2s_data_transmit(SPI2, ulTxData);
	while (spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE) == RESET);
	ulRxDataL=spi_i2s_data_receive(SPI2);//接收读取数据	
	*pdata =(ulRxDataH<<12)|(ulRxDataL>>4);
	SPI_CS_0;  //CS reset
#endif	//#if (CTLBOARD_TYPE ==_HB4_GD32)
	return ret;
}

/*******************************************************************************
* Name: EepromQualifiedRead
* Description: 从EEPROM中指定地址读出数据，含校验
* Input: eeprom read address.
* Output: 0: success. 1: fail. eeprom read data.
* 
* Author:
* Date:
* Revision:
*******************************************************************************/
static INT16U EepromQualifiedRead(INT16U address, INT16U* pdata)
{
	INT16U ReadTimes, ReadFailTimes, ReadValue, data, ret;

	ret = 0;
	ReadTimes = 0;
	ReadFailTimes = 0;

	while(1)
	{
		/* read data in */
		ret = EepromRead(address, &data);
		if(ret == 1)
		{
			break;
		}
		ReadTimes++;
		if(ReadTimes == 1)
		{
			ReadValue = data;
		}
		else
		{
			/* if current read data != the first read data */
			if(data != ReadValue)
			{
				/* then try again */
				ReadTimes = 0;
				/* increase fail times */
				ReadFailTimes++;
				if(ReadFailTimes >= 3)
				{
					ret = 1;
					break;
				}
			}
			/* else current read data == the first read data */
			else
			{
				if(ReadTimes >= 3)
				{
					/* read ok */
					break;
				}
			}
		}
	}

	*pdata = data;
	if(ret == 1)
	{
		/* eeprom error */
		//SL_SET(PLC_EEPROM_RW_ERR);
	}

	return ret;	
}

/*******************************************************************************
* Name: EepromQualifiedWrite
* Description: 将数据写入EEPROM中指定地址,含校验
* Input: eeprom write address, write data
* Output: 0: success. 1: fail
* 
* Author:
* Date:
* Revision:
*******************************************************************************/
static INT16U EepromQualifiedWrite(INT16U address, INT16U data)
{
	INT16U i16U, WriteFailTimes, ret;

	ret = 0;
	WriteFailTimes = 0;
	while(1)
	{
		ret = EepromWrite(address, data);
		if(ret == 1)
		{
			break;
		}
		ret = EepromRead(address, &i16U);
		if(ret == 1)
		{
			break;
		}
		/* if read == write */
		if(i16U == data)
		{
			/* then write ok */
			break;
		}
		/* else read != write */
		else
		{
			/* then write again until WriteFailTimes exceed 3 */
			WriteFailTimes++;
			if(WriteFailTimes >= 3)
			{
				ret = 1;
				break;
			}
		}
	}

	if(ret == 1)
	{
		/* eeprom error */
		//SL_SET(PLC_EEPROM_RW_ERR);
	}
	
	return ret;
}

/*******************************************************************************
* Name: uint16_t u16EepromWrite(uint16_t u16Address, uint16_t u16Data, uint8_t u8Mode)
* Descriptio: Eeprom write Data
* Input: u16Address: eeprom address
*        u16Data: wtite data
*		 u8Mode : 0: 不校验写入; 1:校验写入
* Output:  0: success. 1: fail
*******************************************************************************/
uint16_t u16EepromWrite(uint16_t u16Address, uint16_t u16Data, uint8_t u8Mode)
{
	uint16_t res = 0;
	
	if(1 == u8Mode)
	{
		res = EepromQualifiedWrite(u16Address, u16Data);
	}
	else
	{
		res = EepromWrite(u16Address, u16Data);
	}
	
//	if (0 != res)
//	{
//		i32ErrCodeSet(ErrCode69);		/*Eeprom is wrong*/
//		i32LogWrite(ERR, LOG_BSP, "Write Eeprom is wrong, Address = 0x%x, Data = 0x%x, Mode = %d\r\n", u16Address, u16Data, u8Mode);
//	}
	
//	vDelayms(10);

	return res;
}

/*******************************************************************************
* Name: uint16_t u16EepromRead(uint16_t u16Address, uint16_t u16Data, uint8_t u8Mode)
* Descriptio: Eeprom write Data
* Input: u16Address: eeprom address
*        u16Data: read data point
*		 u8Mode : 0: 不校验读取; 1:校验读取
* Output:  0: success. 1: fail
*******************************************************************************/
uint16_t u16EepromRead(uint16_t u16Address, uint16_t *u16Data, uint8_t u8Mode)
{
	uint16_t res;
	
	if(1 == u8Mode)
	{
		res = EepromQualifiedRead(u16Address, u16Data);
	}
	else
	{
		res = EepromRead(u16Address, u16Data);
	}
	
	return res;
}

uint8_t u8GetEepromFlag(void)
{
	return sgEepromArray.b1Flag;
}


uint8_t u8GetEepromErrFlag(void)
{
	return sgEepromArray.b1ErrFlag;
}

void vClrEepromErrFlag(void)
{
	sgEepromArray.b1ErrFlag = 0;
}


uint8_t u8GetEepromWriteReamin(void)
{
	return sgEepromArray.b7RemainCnt;
}

void vEepromNoBlockWriteProc(void)
{
	if (sgEepromArray.u16EepromRd != sgEepromArray.u16EepromWr)
	{
		if (0 == EepromWriteNoBlock(sgEepromArray.EepromData[sgEepromArray.u16EepromRd].u16Addr, sgEepromArray.EepromData[sgEepromArray.u16EepromRd].u16Data))
		{
			sgEepromArray.u16EepromRd++;
			if (sgEepromArray.u16EepromRd >= EEPROM_LENGTH)
			{
				sgEepromArray.u16EepromRd = 0;
			}
			if (sgEepromArray.b7RemainCnt > 0)
			{
				sgEepromArray.b7RemainCnt--;
			}
		}
	}
	
	
//	if (1 == sgEepromArray.b1ErrFlag)
//	{
//		if (0 == sgEepromArray.b1Flag)
//		{
//					for(index=0;index < (sizeof(cPara_Table)/sizeof(cPara_Table[0]));index++) 
//		{
//			u16ReadParaFromEeprom(index);
//		}
//			sgEepromArray.b1ErrFlag = 0;
//		}
//	}

}
#endif


#include "Device.h"
#include "Eeprom.h"
#include "ErrCode.h"
#include "Log.h"
#include "NetTimer.h"


#define EEPROM_LENGTH	256
typedef struct
{
	uint16_t u16Addr;
	uint16_t u16Data;
}xEepromInfo;

typedef struct
{
	uint16_t	u16EepromWr;
	uint16_t	u16EepromRd;
	xEepromInfo EepromData[EEPROM_LENGTH];
	union
	{
		uint32_t u32Data;
		struct
		{
			uint32_t b1Flag: 1;
			uint32_t b8RemainCnt: 8;
			uint32_t b1ErrFlag: 1;
			uint32_t b22Reserve: 22;
		};
	};
}xEepromArray;

xEepromArray sgEepromArray = {.u16EepromWr = 0, .u16EepromRd = 0, .u32Data = 0};
static EeepromCallBackt sgEepromCallBack = NULL;
static SaveParaFinishCallBackt sgSaveParaFinishCallBack = NULL;
static uint16_t sgU16ParaLastAddr = 0;

#if (CTLBOARD_TYPE ==_CPD_S_GD32) 
typedef enum {
    I2C_START = 0,
    I2C_SEND_ADDRESS,
    I2C_CLEAR_ADDRESS_FLAG,
    I2C_TRANSMIT_DATA,
    I2C_STOP,
} i2c_process_enum;

#define I2C_TIME_OUT           (uint16_t)(5000)
#define EEP_FIRST_PAGE         0x00

#define I2C_OK                 1
#define I2C_FAIL               0
#define I2C_END                1

#endif

/*延时ulTime个指令周期,ulTime=10对应1us*/
static void vDelay(uint64_t u64Time)
{
	uint64_t i = 0;
	for (i=0; i<u64Time; i++)
	{
		__NOP();
	}
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
/*******************************************************************************
* Name: EepromWrite
* Description: EEPROM写入数据.
* Input: 
* Output: 0:Success;1:fail
* Author:
* Date:
* Revision:
*******************************************************************************/
static INT16U EepromWrite(INT16U address, INT16U data)
{

#if (CTLBOARD_TYPE ==_1231_G4)
	
	uint8_t ulTxData[4],ulRxDataL[2];
	INT16U ret;
	ret=0;	
	if (0 == sgEepromArray.b1Flag)
	{
		sgEepromArray.b1Flag = 1;
		//
		//1、使能写操作
		//
		//触发上升沿
		SPI_CS_0;  //CS reset
		Delay(20);  
		SPI_CS_1;  //CS set
		//发送写使能操作码	
	//	ulTxData= 0x04C0;
		ulTxData[0] = 0x04;
		ulTxData[1] = 0xc0;
		HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&ulTxData,(uint8_t*)&ulRxDataL,2, SPI_TIMEOUT);

		//
		// 2、写操作
		//
		//触发上升沿
		SPI_CS_0;  //CS reset
		Delay(20); 
		SPI_CS_1;  //CS set
		//发送写操作码、地址
		ulTxData[0] = ((0x0500 | address)>>8)&0xFF;
		ulTxData[1] = (0x0500 | address)&0xFF;
		HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&ulTxData,(uint8_t*)&ulRxDataL,2, SPI_TIMEOUT);
		//发送16位数据
		ulTxData[0] = (data&0xff00)>>8;
		ulTxData[1] = data&0x00ff;
		HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&ulTxData,(uint8_t*)&ulRxDataL,2, SPI_TIMEOUT);
		//触发上升沿,等待写操作完成(不小于6ms)
		SPI_CS_0;  //CS reset
		Delay(20); 
		SPI_CS_1;  //CS set
		Delay(150000);
		//
		// 3. 写禁止
		//
		//触发上升沿
		SPI_CS_0;  //CS reset
		Delay(20);
		SPI_CS_1;  //CS set
		//发送写禁止操作码
		ulTxData[0] = 0x04;
		ulTxData[1] = 0x00;
		HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&ulTxData,(uint8_t*)&ulRxDataL,2, SPI_TIMEOUT);
		SPI_CS_0;  //CS reset
		sgEepromArray.b1Flag = 0;
	}	
#endif	//#if (CTLBOARD_TYPE ==_1231_G4)

#if ((CTLBOARD_TYPE == _HB4_GD32) || (CTLBOARD_TYPE == _HB6_GD32) || (CTLBOARD_TYPE == _EPS_S_GD32))
	INT16U ulTxData,ulRxData;
	INT16U ret;
	ret=0;	
	if (0 == sgEepromArray.b1Flag)
	{
		sgEepromArray.b1Flag = 1;
		//
		//1、使能写操作
		//
		//触发上升沿
		SPI_CS_0;  //CS reset
		vDelay(20);  
		SPI_CS_1;  //CS set
		//发送写使能操作码	
		ulTxData= 0x04C0;
		spi_i2s_data_transmit(SPI2, ulTxData);
		while(spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE) == RESET);
		ulRxData = spi_i2s_data_receive(SPI2);
		(void)ulRxData;	//防止编译警告
		//
		// 2、写操作
		//
		//触发上升沿
		SPI_CS_0;  //CS reset
		vDelay(20); 
		SPI_CS_1;  //CS set
		//发送写操作码、地址
		ulTxData = 0x0500 | address;
		//等待发送操作完成
		while (spi_i2s_flag_get(SPI2, SPI_FLAG_TBE) == RESET);
		spi_i2s_data_transmit(SPI2, ulTxData);
		while (spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE) == RESET);
		ulRxData=spi_i2s_data_receive(SPI2);//接收读取数据	
		//发送16位数据
		ulTxData = data;
		//等待发送操作完成
		while (spi_i2s_flag_get(SPI2, SPI_FLAG_TBE) == RESET);
		spi_i2s_data_transmit(SPI2, ulTxData);
		while (spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE) == RESET);
		ulRxData=spi_i2s_data_receive(SPI2);//接收读取数据
		//触发上升沿,等待写操作完成(不小于6ms)	
		SPI_CS_0;  //CS reset
		vDelay(20); 
		SPI_CS_1;  //CS set
	//	vDelay(150000);
		vDelay(80000);
		//
		// 3. 写禁止
		//
		//触发上升沿
		SPI_CS_0;  //CS reset
		vDelay(20);
		SPI_CS_1;  //CS set
		//发送写禁止操作码
		ulTxData = 0x0400;
		while (spi_i2s_flag_get(SPI2, SPI_FLAG_TBE) == RESET);
		spi_i2s_data_transmit(SPI2, ulTxData);
		while (spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE) == RESET);
		ulRxData=spi_i2s_data_receive(SPI2);//接收读取数据
		SPI_CS_0;  //CS reset

		sgEepromArray.b1Flag = 0;
	}
#endif	//#if (CTLBOARD_TYPE ==_HB4_GD32)

#if(CTLBOARD_TYPE ==_CPD_S_GD32)

	uint16_t ret = 0;
	if (0 == sgEepromArray.b1Flag)
	{
		sgEepromArray.b1Flag = 1;
		uint8_t   state = I2C_START;
		uint16_t  timeout = 0;
		uint8_t   i2c_timeout_flag = 0;
		uint8_t eeprom_address = 0;
		uint8_t write_address = 0;
		uint8_t number_of_byte = 2;
		uint8_t buffer[2] = {0};
		uint8_t *p_buffer = buffer;
		/*lilu 20240313 大于等于256 B8 是1， else 是0*/
		if (address >= 0x80)
		{
			eeprom_address = 0xA2;
		}
		else
		{
			eeprom_address = 0xA0;
		}
		
		write_address = (uint8_t)(address << 1); 
		buffer[0] = (uint8_t)(data >> 8);
		buffer[1] = (uint8_t)(data & 0xFF);
		

		/* enable acknowledge */
		i2c_ack_config(I2C1, I2C_ACK_ENABLE);

		while(!(i2c_timeout_flag) && (0 == ret)) 
		{
			switch(state) 
			{
				case I2C_START:
					/* i2c master sends start signal only when the bus is idle */
					while(i2c_flag_get(I2C1, I2C_FLAG_I2CBSY) && (timeout < I2C_TIME_OUT)) 
					{
						timeout++;
					}
					if(timeout < I2C_TIME_OUT) 
					{
						i2c_start_on_bus(I2C1);
						timeout = 0;
						state = I2C_SEND_ADDRESS;
					} 
					else 
					{
						ret= 1;
						timeout = 0;
						state   = I2C_START;
						printf("i2c bus is busy in WRITE BYTE!\n");
					}
					break;
				case I2C_SEND_ADDRESS:
					/* i2c master sends START signal successfully */
					while((! i2c_flag_get(I2C1, I2C_FLAG_SBSEND)) && (timeout < I2C_TIME_OUT)) 
					{
						timeout++;
					}
					if(timeout < I2C_TIME_OUT) 
					{
						i2c_master_addressing(I2C1, eeprom_address, I2C_TRANSMITTER);
						timeout = 0;
						state = I2C_CLEAR_ADDRESS_FLAG;
					} 
					else 
					{
						ret = 1;
						timeout = 0;
						state = I2C_START;
						printf("i2c master sends start signal timeout in WRITE BYTE!\n");
					}
					break;
				case I2C_CLEAR_ADDRESS_FLAG:
					/* address flag set means i2c slave sends ACK */
					while((! i2c_flag_get(I2C1, I2C_FLAG_ADDSEND)) && (timeout < I2C_TIME_OUT)) 
					{
						timeout++;
					}
					if(timeout < I2C_TIME_OUT) 
					{
						i2c_flag_clear(I2C1, I2C_FLAG_ADDSEND);
						timeout = 0;
						state = I2C_TRANSMIT_DATA;
					} 
					else 
					{
						ret = 1;
						timeout = 0;
						state = I2C_START;
						printf("i2c master clears address flag timeout in WRITE BYTE!\n");
					}
					break;
				case I2C_TRANSMIT_DATA:
					/* wait until the transmit data buffer is empty */
					while((! i2c_flag_get(I2C1, I2C_FLAG_TBE)) && (timeout < I2C_TIME_OUT)) 
					{
						timeout++;
					}
					if(timeout < I2C_TIME_OUT) 
					{
						/* send the EEPROM's internal address to write to : only one byte address */
						i2c_data_transmit(I2C1, write_address);
						timeout = 0;
					} 
					else 
					{
						ret = 1;
						timeout = 0;
						state = I2C_START;
						printf("i2c master sends data timeout in WRITE BYTE!\n");
					}

					/* wait until BTC bit is set */
					while((!i2c_flag_get(I2C1, I2C_FLAG_BTC)) && (timeout < I2C_TIME_OUT)) 
					{
						timeout++;
					}
								
					if(timeout < I2C_TIME_OUT) 
					{
						timeout = 0;
					} 
					else 
					{
						ret = 1;
						timeout = 0;
						state = I2C_START;
						printf("i2c master sends data timeout in WRITE BYTE!\n");
					}
					
					while(number_of_byte--) 
					{
						i2c_data_transmit(I2C1, *p_buffer);
						/* point to the next byte to be written */
						p_buffer++;
						/* wait until BTC bit is set */
						while((!i2c_flag_get(I2C1, I2C_FLAG_BTC)) && (timeout < I2C_TIME_OUT)) 
						{
							timeout++;
						}
						if(timeout < I2C_TIME_OUT) 
						{
							timeout = 0;
						} 
						else 
						{
							ret = 1;
							timeout = 0;
							state = I2C_START;
							printf("i2c master sends data timeout in WRITE!\n");
						}
					}
					timeout = 0;
					state = I2C_STOP;
					break;
				case I2C_STOP:
					/* send a stop condition to I2C bus */
					i2c_stop_on_bus(I2C1);
					/* i2c master sends STOP signal successfully */
					while((I2C_CTL0(I2C1) & I2C_CTL0_STOP) && (timeout < I2C_TIME_OUT)) 
					{
						timeout++;
					}
					if(timeout < I2C_TIME_OUT) 
					{
						timeout = 0;
						state = I2C_END;
						i2c_timeout_flag = I2C_OK;
					} 
					else 
					{
						ret = 1;
						timeout = 0;
						state = I2C_START;
						printf("i2c master sends stop signal timeout in WRITE BYTE!\n");
					}
					break;
				default:
					state = I2C_START;
					i2c_timeout_flag = I2C_OK;
					timeout = 0;
					printf("i2c master sends start signal in WRITE BYTE.\n");
					break;
			}
		
		}
		vDelay(50000);
		sgEepromArray.b1Flag = 1;
	}
#endif
	
	return ret;
}

/*******************************************************************************
* Name: EepromRead
* Description: EEPROM读取数据
* Input: 
* Output: 0:Success;1:fail
* Author:
* Date:
* Revision:
*******************************************************************************/
static INT16U EepromRead(INT16U address, INT16U* pdata)
{
#if (CTLBOARD_TYPE ==_1231_G4)
	uint8_t ulTxData[4],ulRxDataH[4],ulRxDataL[4];
	INT16U ret;
	ret=0;
	//触发上升沿
	if (0 == sgEepromArray.b1Flag)
	{
		sgEepromArray.b1Flag = 1;
		SPI_CS_0;  //CS reset
		Delay(20);
		SPI_CS_1;  //CS set
		// 发送操作码和地址
	//	ulTxData= (0x0600 | address)<<5;
		ulTxData[0] = (((0x0600 | address)<<5)>>8)&0xFF;
		ulTxData[1] = ((0x0600 | address)<<5)&0xFF;;
		ulTxData[2] = 0x00;
		ulTxData[3] = 0x00;
		HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&ulTxData,(uint8_t*)&ulRxDataH,2, SPI_TIMEOUT);

		// 发送无效数据，产生时钟信号，接收剩余读取数据 
		ulTxData[0] = 0x00;
		ulTxData[1] = 0x00;
		HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&ulTxData,(uint8_t*)&ulRxDataL,2, SPI_TIMEOUT);
		
		SPI_CS_0;  //CS reset
		

		*pdata = ((ulRxDataH[1]<<4)|(ulRxDataL[0]>>4))<<8;
		*pdata |= ((ulRxDataL[0]<<4)|(ulRxDataL[1]>>4))&0x00ff;
		sgEepromArray.b1Flag = 0;
	}
#endif	//#if (CTLBOARD_TYPE ==_1231_G4)

#if ((CTLBOARD_TYPE == _HB4_GD32) || (CTLBOARD_TYPE == _HB6_GD32) || (CTLBOARD_TYPE == _EPS_S_GD32))
	INT16U ulTxData,ulRxDataL,ulRxDataH;
	INT16U ret;
	ret=0;
	if (0 == sgEepromArray.b1Flag)
	{
		sgEepromArray.b1Flag = 1;
		//触发上升沿
		SPI_CS_0;  //CS reset
		vDelay(20);
		SPI_CS_1;  //CS set
		// 发送操作码和地址
		ulTxData= (0x0600 | address)<<5;
		//等待发送操作完成
		while (spi_i2s_flag_get(SPI2, SPI_FLAG_TBE) == RESET);
		spi_i2s_data_transmit(SPI2, ulTxData);
		while (spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE) == RESET);
		ulRxDataH=spi_i2s_data_receive(SPI2);//接收读取数据	

		// 发送无效数据，产生时钟信号，接收剩余读取数据 
		ulTxData = 0x00;
		//等待发送操作完成
		while (spi_i2s_flag_get(SPI2, SPI_FLAG_TBE) == RESET);
		spi_i2s_data_transmit(SPI2, ulTxData);
		while (spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE) == RESET);
		ulRxDataL=spi_i2s_data_receive(SPI2);//接收读取数据	
		*pdata =(ulRxDataH<<12)|(ulRxDataL>>4);
		SPI_CS_0;  //CS reset
		sgEepromArray.b1Flag = 0;
	}
	else
	{
		ret = 1;
	}
#endif	//#if (CTLBOARD_TYPE ==_HB4_GD32)

#if (CTLBOARD_TYPE ==_CPD_S_GD32)
	uint16_t ret = 0; 
	if (0 == sgEepromArray.b1Flag)
	{
		sgEepromArray.b1Flag = 1;
		
		uint8_t   state = I2C_START;
		uint8_t   read_cycle = 0;
		uint16_t  timeout = 0;
		uint8_t   i2c_timeout_flag = 0;

		uint8_t eeprom_address = 0;
		uint8_t read_address = 0;
		uint8_t number_of_byte = 2;
		uint8_t buffer[2] = {0};
		uint8_t *p_buffer = buffer;
		/*lilu 20240313 大于等于256 B8 是1， else 是0*/
		if (address >= 0x80)
		{
			eeprom_address = 0xA2;
		}
		else
		{
			eeprom_address = 0xA0;
		}
		
		read_address = (uint8_t)(address << 1); 
		
		/* enable acknowledge */
		i2c_ack_config(I2C1, I2C_ACK_ENABLE);

		while(!(i2c_timeout_flag) && (0 == ret)) 
		{
			switch(state) 
			{
				case I2C_START:
					if(RESET == read_cycle) 
					{
						/* i2c master sends start signal only when the bus is idle */
						while(i2c_flag_get(I2C1, I2C_FLAG_I2CBSY) && (timeout < I2C_TIME_OUT)) 
						{
							timeout++;
						}
						if(timeout < I2C_TIME_OUT) 
						{
							/* whether to send ACK or not for the next byte */
							if(2 == number_of_byte) 
							{
								i2c_ackpos_config(I2C1, I2C_ACKPOS_NEXT);
							}
						} 
						else 
						{
							//i2c_bus_reset();
							ret = 1;
							timeout = 0;
							state = I2C_START;
							printf("i2c bus is busy in READ!\n");
						}
					}
					/* send the start signal */
					i2c_start_on_bus(I2C1);
					timeout = 0;
					state = I2C_SEND_ADDRESS;
					break;
				case I2C_SEND_ADDRESS:
					/* i2c master sends START signal successfully */
					while((! i2c_flag_get(I2C1, I2C_FLAG_SBSEND)) && (timeout < I2C_TIME_OUT)) 
					{
						timeout++;
					}
					if(timeout < I2C_TIME_OUT) 
					{
						if(RESET == read_cycle) 
						{
							i2c_master_addressing(I2C1, eeprom_address, I2C_TRANSMITTER);
							state = I2C_CLEAR_ADDRESS_FLAG;
						} 
						else 
						{
							i2c_master_addressing(I2C1, eeprom_address, I2C_RECEIVER);
							if(number_of_byte < 3) 
							{
								/* disable acknowledge */
								i2c_ack_config(I2C1, I2C_ACK_DISABLE);
							}
							state = I2C_CLEAR_ADDRESS_FLAG;
						}
						timeout = 0;
					} 
					else 
					{
						ret = 1;
						timeout = 0;
						state = I2C_START;
						read_cycle = 0;
						printf("i2c master sends start signal timeout in READ!\n");
					}
					break;
				case I2C_CLEAR_ADDRESS_FLAG:
					/* address flag set means i2c slave sends ACK */
					while((!i2c_flag_get(I2C1, I2C_FLAG_ADDSEND)) && (timeout < I2C_TIME_OUT)) 
					{
						timeout++;
					}
					if(timeout < I2C_TIME_OUT) 
					{
						i2c_flag_clear(I2C1, I2C_FLAG_ADDSEND);
						if((SET == read_cycle) && (1 == number_of_byte)) 
						{
							/* send a stop condition to I2C bus */
							i2c_stop_on_bus(I2C1);
						}
						timeout = 0;
						state   = I2C_TRANSMIT_DATA;
					} 
					else 
					{
						ret = 1;
						timeout = 0;
						state   = I2C_START;
						read_cycle = 0;
						printf("i2c master clears address flag timeout in READ!\n");
					}
					break;
				case I2C_TRANSMIT_DATA:
					if(RESET == read_cycle) 
					{
						/* wait until the transmit data buffer is empty */
						while((! i2c_flag_get(I2C1, I2C_FLAG_TBE)) && (timeout < I2C_TIME_OUT)) 
						{
							timeout++;
						}
						if(timeout < I2C_TIME_OUT) 
						{
							/* send the EEPROM's internal address to write to : only one byte address */
							i2c_data_transmit(I2C1, read_address);
							timeout = 0;
						} 
						else 
						{
							ret = 1;
							timeout = 0;
							state = I2C_START;
							read_cycle = 0;
							printf("i2c master wait data buffer is empty timeout in READ!\n");
						}
						/* wait until BTC bit is set */
						while((!i2c_flag_get(I2C1, I2C_FLAG_BTC)) && (timeout < I2C_TIME_OUT)) 
						{
							timeout++;
						}
						if(timeout < I2C_TIME_OUT) 
						{
							timeout = 0;
							state = I2C_START;
							read_cycle++;
						} 
						else 
						{
							ret = 1;
							timeout = 0;
							state = I2C_START;
							read_cycle = 0;
							printf("i2c master sends EEPROM's internal address timeout in READ!\n");
						}
					} 
					else 
					{
						while(number_of_byte) 
						{
							timeout++;
							if(3 == number_of_byte) 
							{
								/* wait until BTC bit is set */
								while(!i2c_flag_get(I2C1, I2C_FLAG_BTC));
								/* disable acknowledge */
								i2c_ack_config(I2C1, I2C_ACK_DISABLE);
							}
							if(2 == number_of_byte) 
							{
								/* wait until BTC bit is set */
								while(!i2c_flag_get(I2C1, I2C_FLAG_BTC));
								/* send a stop condition to I2C bus */
								i2c_stop_on_bus(I2C1);
							}
							/* wait until RBNE bit is set */
							if(i2c_flag_get(I2C1, I2C_FLAG_RBNE)) 
							{
								/* read a byte from the EEPROM */
								*p_buffer = i2c_data_receive(I2C1);

								/* point to the next location where the byte read will be saved */
								p_buffer++;

								/* decrement the read bytes counter */
								number_of_byte--;
								timeout = 0;
							}
							if(timeout > I2C_TIME_OUT) 
							{
								ret = 1;
								timeout = 0;
								state = I2C_START;
								read_cycle = 0;
								printf("i2c master sends data timeout in READ!\n");
							}
						}
						timeout = 0;
						state = I2C_STOP;
					}
					break;
				case I2C_STOP:
					/* i2c master sends STOP signal successfully */
					while((I2C_CTL0(I2C1) & I2C_CTL0_STOP) && (timeout < I2C_TIME_OUT)) 
					{
						timeout++;
					}
					if(timeout < I2C_TIME_OUT) 
					{
						timeout = 0;
						state = I2C_END;
						i2c_timeout_flag = I2C_OK;
					} 
					else 
					{
						ret = 1;
						timeout = 0;
						state = I2C_START;
						read_cycle = 0;
						printf("i2c master sends stop signal timeout in READ!\n");
					}
					break;
				default:
					state = I2C_START;
					read_cycle = 0;
					i2c_timeout_flag = I2C_OK;
					timeout = 0;
					printf("i2c master sends start signal in READ.\n");
					break;
			}
		}
		*pdata = (uint16_t)(buffer[0] << 8) | buffer[1];
		sgEepromArray.b1Flag = 0;
	}
	else
	{
		ret = 1;
	}
#endif
	return ret;
}


static INT16U EepromWriteNoBlock(INT16U address, INT16U data)
{
#if ((CTLBOARD_TYPE == _HB4_GD32) || (CTLBOARD_TYPE == _HB6_GD32) || (CTLBOARD_TYPE == _EPS_S_GD32))
	INT16U ulTxData = 0, ulRxData = 0;
	INT16U ret;
	ret = 1;
	
	static uint8_t u8WrCnt = 0;
	static uint8_t u8State = 0;
	
	switch (u8State)
	{
		case 0:
			EepromRead(address, &ulRxData);
			if (ulRxData == data)
			{
				u8State = 0;
				ret = 0;
				/*lilu 20240524 add Para Save Finish Func*/
				if (sgU16ParaLastAddr == address)
				{
					if (1 == sgEepromArray.b1ErrFlag)
					{
						if (NULL != sgSaveParaFinishCallBack)
						{
							sgSaveParaFinishCallBack(0xFF);
						}
					}
					else
					{
						if (NULL != sgSaveParaFinishCallBack)
						{
							sgSaveParaFinishCallBack(0x01);
						}
					}
				}
			}
			else
			{
				u8State = 1;
			}
			break;
		case 1:
		{
			sgEepromArray.b1Flag = 1;
			SPI_CS_0;  //CS reset
			vDelay(20);  
			SPI_CS_1;  //CS set
			//发送写使能操作码	
			ulTxData= 0x04C0;
			spi_i2s_data_transmit(SPI2, ulTxData);
			while(spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE) == RESET);
			ulRxData = spi_i2s_data_receive(SPI2);
			(void)ulRxData;	//防止编译警告
			//
			// 2、写操作
			//
			//触发上升沿
			SPI_CS_0;  //CS reset
			vDelay(20); 
			SPI_CS_1;  //CS set
			//发送写操作码、地址
			ulTxData = 0x0500 | address;
			//等待发送操作完成
			while (spi_i2s_flag_get(SPI2, SPI_FLAG_TBE) == RESET);
			spi_i2s_data_transmit(SPI2, ulTxData);
			while (spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE) == RESET);
			ulRxData=spi_i2s_data_receive(SPI2);//接收读取数据	
			//发送16位数据
			ulTxData = data;
			//等待发送操作完成
			while (spi_i2s_flag_get(SPI2, SPI_FLAG_TBE) == RESET);
			spi_i2s_data_transmit(SPI2, ulTxData);
			while (spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE) == RESET);
			ulRxData=spi_i2s_data_receive(SPI2);//接收读取数据
			//触发上升沿,等待写操作完成(不小于6ms)	
			SPI_CS_0;  //CS reset
			vDelay(20); 
			SPI_CS_1;  //CS set
			/*lilu 20240104 Delay 6ms*/
			if (false == u8GetNetTimerStartFlag(TIMER_EEPROM))
			{
				vSetNetTimer(TIMER_EEPROM, 6);	/**/
			}
			
			u8State = 2;
			break;
		}
		case 2:
			if (true == u8GetNetTimerOverFlag((TIMER_EEPROM)))
			{
				SPI_CS_0;  //CS reset
				vDelay(20);
				SPI_CS_1;  //CS set
				//发送写禁止操作码
				ulTxData = 0x0400;
				while (spi_i2s_flag_get(SPI2, SPI_FLAG_TBE) == RESET);
				spi_i2s_data_transmit(SPI2, ulTxData);
				while (spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE) == RESET);
				ulRxData=spi_i2s_data_receive(SPI2);//接收读取数据
				SPI_CS_0;  //CS reset
				
				u8State = 3;
				sgEepromArray.b1Flag = 0;
				vKillNetTimer(TIMER_EEPROM);
			}
			break;
		case 3:
			EepromRead(address, &ulRxData);
			/*lilu 20240409 Update Para Ram*/
			if ((NULL != sgEepromCallBack) && (address < 200))
			{
				sgEepromCallBack(address, ulRxData);
				//i32SetPara(address, ulRxData);
			}
			if (ulRxData == data)
			{
				u8WrCnt = 0;
				ret = 0;
				u8State = 0;
				/*lilu 20240524 add Para Save Finish Func*/
				if (sgU16ParaLastAddr == address)
				{
					if (1 == sgEepromArray.b1ErrFlag)
					{
						if (NULL != sgSaveParaFinishCallBack)
						{
							sgSaveParaFinishCallBack(0xFF);
						}
					}
					else
					{
						if (NULL != sgSaveParaFinishCallBack)
						{
							sgSaveParaFinishCallBack(0x01);
						}
					}
				}
			}
			else
			{
				u8WrCnt++;
				u8State = 1;
				if (u8WrCnt >= 3)
				{
					ret = 0;
					u8WrCnt = 0;
					u8State = 0;
					/*lilu 20240114 记录一次写Eeprom失败*/
					i32ErrCodeSet(ErrCode39);
					/*lilu 20240116*/
					if (address <= sgU16ParaLastAddr)
					{
						sgEepromArray.b1ErrFlag = 1;
					}
					
					/*lilu 20240524 add Para Save Finish Func*/
					if (sgU16ParaLastAddr == address)
					{
						if (1 == sgEepromArray.b1ErrFlag)
						{
							if (NULL != sgSaveParaFinishCallBack)
							{
								sgSaveParaFinishCallBack(0xFF);
							}
						}
						else
						{
							if (NULL != sgSaveParaFinishCallBack)
							{
								sgSaveParaFinishCallBack(0x01);
							}
						}
					}
				}
			}
				
			break;
		default:
			u8State = 0;
			break;
	}
#endif
#if (CTLBOARD_TYPE == _CPD_S_GD32)
	uint16_t ret = 1;
	uint16_t ulTxData = 0, ulRxData = 0;
	
	static uint8_t u8WrCnt = 0;
	static uint8_t u8State = 0;
	
	switch(u8State)
	{
		case 0:/*Read*/
			EepromRead(address, &ulRxData);
			if (ulRxData == data)
			{
				u8State = 0;
				ret = 0;
			}
			else
			{
				u8State = 1;
			}
			break;
		case 1:/*Write*/
			sgEepromArray.b1Flag = 1;
			uint8_t   state = I2C_START;
			uint8_t u8IICRet = 0;
			uint16_t  timeout = 0;
			uint8_t   i2c_timeout_flag = 0;
			uint8_t eeprom_address = 0;
			uint8_t write_address = 0;
			uint8_t number_of_byte = 2;
			uint8_t buffer[2] = {0};
			uint8_t *p_buffer = buffer;
			/*lilu 20240313 大于等于256 B8 是1， else 是0*/
			if (address >= 0x80)
			{
				eeprom_address = 0xA2;
			}
			else
			{
				eeprom_address = 0xA0;
			}
		
			write_address = (uint8_t)(address << 1); 
			buffer[0] = (uint8_t)(data >> 8);
			buffer[1] = (uint8_t)(data & 0xFF);
		

			/* enable acknowledge */
			i2c_ack_config(I2C1, I2C_ACK_ENABLE);

			while(!(i2c_timeout_flag) && (0 == u8IICRet)) 
			{
				switch(state) 
				{
					case I2C_START:
						/* i2c master sends start signal only when the bus is idle */
						while(i2c_flag_get(I2C1, I2C_FLAG_I2CBSY) && (timeout < I2C_TIME_OUT)) 
						{
							timeout++;
						}
						if(timeout < I2C_TIME_OUT) 
						{
							i2c_start_on_bus(I2C1);
							timeout = 0;
							state = I2C_SEND_ADDRESS;
						} 
						else 
						{
							u8IICRet = 1;
							timeout = 0;
							state   = I2C_START;
							printf("i2c bus is busy in WRITE BYTE!\n");
						}
						break;
					case I2C_SEND_ADDRESS:
						/* i2c master sends START signal successfully */
						while((! i2c_flag_get(I2C1, I2C_FLAG_SBSEND)) && (timeout < I2C_TIME_OUT)) 
						{
							timeout++;
						}
						if(timeout < I2C_TIME_OUT) 
						{
							i2c_master_addressing(I2C1, eeprom_address, I2C_TRANSMITTER);
							timeout = 0;
							state = I2C_CLEAR_ADDRESS_FLAG;
						} 
						else 
						{
							u8IICRet = 1;
							timeout = 0;
							state = I2C_START;
							printf("i2c master sends start signal timeout in WRITE BYTE!\n");
						}
						break;
					case I2C_CLEAR_ADDRESS_FLAG:
						/* address flag set means i2c slave sends ACK */
						while((! i2c_flag_get(I2C1, I2C_FLAG_ADDSEND)) && (timeout < I2C_TIME_OUT)) 
						{
							timeout++;
						}
						if(timeout < I2C_TIME_OUT) 
						{
							i2c_flag_clear(I2C1, I2C_FLAG_ADDSEND);
							timeout = 0;
							state = I2C_TRANSMIT_DATA;
						} 
						else 
						{
							u8IICRet = 1;
							timeout = 0;
							state = I2C_START;
							printf("i2c master clears address flag timeout in WRITE BYTE!\n");
						}
						break;
					case I2C_TRANSMIT_DATA:
						/* wait until the transmit data buffer is empty */
						while((! i2c_flag_get(I2C1, I2C_FLAG_TBE)) && (timeout < I2C_TIME_OUT)) 
						{
							timeout++;
						}
						if(timeout < I2C_TIME_OUT) 
						{
							/* send the EEPROM's internal address to write to : only one byte address */
							i2c_data_transmit(I2C1, write_address);
							timeout = 0;
						} 
						else 
						{
							u8IICRet = 1;
							timeout = 0;
							state = I2C_START;
							printf("i2c master sends data timeout in WRITE BYTE!\n");
						}

						/* wait until BTC bit is set */
						while((!i2c_flag_get(I2C1, I2C_FLAG_BTC)) && (timeout < I2C_TIME_OUT)) 
						{
							timeout++;
						}
									
						if(timeout < I2C_TIME_OUT) 
						{
							timeout = 0;
						} 
						else 
						{
							u8IICRet = 1;
							timeout = 0;
							state = I2C_START;
							printf("i2c master sends data timeout in WRITE BYTE!\n");
						}
						
						while(number_of_byte--) 
						{
							i2c_data_transmit(I2C1, *p_buffer);
							/* point to the next byte to be written */
							p_buffer++;
							/* wait until BTC bit is set */
							while((!i2c_flag_get(I2C1, I2C_FLAG_BTC)) && (timeout < I2C_TIME_OUT)) 
							{
								timeout++;
							}
							if(timeout < I2C_TIME_OUT) 
							{
								timeout = 0;
							} 
							else 
							{
								u8IICRet = 1;
								timeout = 0;
								state = I2C_START;
								printf("i2c master sends data timeout in WRITE!\n");
							}
						}
						timeout = 0;
						state = I2C_STOP;
						break;
					case I2C_STOP:
						/* send a stop condition to I2C bus */
						i2c_stop_on_bus(I2C1);
						/* i2c master sends STOP signal successfully */
						while((I2C_CTL0(I2C1) & I2C_CTL0_STOP) && (timeout < I2C_TIME_OUT)) 
						{
							timeout++;
						}
						if(timeout < I2C_TIME_OUT) 
						{
							timeout = 0;
							state = I2C_END;
							i2c_timeout_flag = I2C_OK;
						} 
						else 
						{
							u8IICRet = 1;
							timeout = 0;
							state = I2C_START;
							printf("i2c master sends stop signal timeout in WRITE BYTE!\n");
						}
						break;
					default:
						state = I2C_START;
						i2c_timeout_flag = I2C_OK;
						timeout = 0;
						printf("i2c master sends start signal in WRITE BYTE.\n");
						break;
				}
		
			}
			if (false == u8GetNetTimerStartFlag(TIMER_EEPROM))
			{
				vSetNetTimer(TIMER_EEPROM, 3);	/**/
			}
			u8State = 2;
			break;
		case 2:/*Delay*/
			if (true == u8GetNetTimerOverFlag((TIMER_EEPROM)))
			{
				u8State = 3;
				sgEepromArray.b1Flag = 0;
				vKillNetTimer(TIMER_EEPROM);
			}
			else
			{
				u8State = 2;
			}
			break;
		case 3:/*Compare*/
			EepromRead(address, &ulRxData);
			if (ulRxData == data)
			{
				u8WrCnt = 0;
				ret = 0;
				u8State = 0;
			}
			else
			{
				u8WrCnt++;
				u8State = 1;
				if (u8WrCnt >= 3)
				{
					ret = 0;
					u8WrCnt = 0;
					u8State = 0;
					/*lilu 20240114 记录一次写Eeprom失败*/
					i32ErrCodeSet(ErrCode39);
					/*lilu 20240116*/
					if (address < 200)
					{
						sgEepromArray.b1ErrFlag = 1;
					}
				}
			}
			break;
		default:
			break;
	}
	
	//EepromWrite(address, data);
	
	//ret = 1;
#endif
	return ret;
}

/*******************************************************************************
* Name: EepromQualifiedRead
* Description: 从EEPROM中指定地址读出数据，含校验
* Input: eeprom read address.
* Output: 0: success. 1: fail. eeprom read data.
* 
* Author:
* Date:
* Revision:
*******************************************************************************/
static INT16U EepromQualifiedRead(INT16U address, INT16U* pdata)
{
	INT16U ReadTimes, ReadFailTimes, ReadValue, data, ret;

	ret = 0;
	ReadTimes = 0;
	ReadFailTimes = 0;

	while(1)
	{
		/* read data in */
		ret = EepromRead(address, &data);
		if(ret == 1)
		{
			break;
		}
		ReadTimes++;
		if(ReadTimes == 1)
		{
			ReadValue = data;
		}
		else
		{
			/* if current read data != the first read data */
			if(data != ReadValue)
			{
				/* then try again */
				ReadTimes = 0;
				/* increase fail times */
				ReadFailTimes++;
				if(ReadFailTimes >= 3)
				{
					ret = 1;
					break;
				}
			}
			/* else current read data == the first read data */
			else
			{
				if(ReadTimes >= 3)
				{
					/* read ok */
					break;
				}
			}
		}
	}

	*pdata = data;
	if(ret == 1)
	{
		/* eeprom error */
		//SL_SET(PLC_EEPROM_RW_ERR);
	}

	return ret;	
}

/*******************************************************************************
* Name: EepromQualifiedWrite
* Description: 将数据写入EEPROM中指定地址,含校验
* Input: eeprom write address, write data
* Output: 0: success. 1: fail
* 
* Author:
* Date:
* Revision:
*******************************************************************************/
static INT16U EepromQualifiedWrite(INT16U address, INT16U data)
{
	INT16U i16U, WriteFailTimes, ret;

	ret = 0;
	WriteFailTimes = 0;
	while(1)
	{
		ret = EepromWrite(address, data);
		if(ret == 1)
		{
			break;
		}
		ret = EepromRead(address, &i16U);
		if(ret == 1)
		{
			break;
		}
		/* if read == write */
		if(i16U == data)
		{
			/* then write ok */
			break;
		}
		/* else read != write */
		else
		{
			/* then write again until WriteFailTimes exceed 3 */
			WriteFailTimes++;
			if(WriteFailTimes >= 3)
			{
				ret = 1;
				break;
			}
		}
	}

	if(ret == 1)
	{
		/* eeprom error */
		//SL_SET(PLC_EEPROM_RW_ERR);
	}
	
	return ret;
}

/*******************************************************************************
* Name: uint16_t u16EepromWrite(uint16_t u16Address, uint16_t u16Data, uint8_t u8Mode)
* Descriptio: Eeprom write Data
* Input: u16Address: eeprom address
*        u16Data: wtite data
*		 u8Mode : 0: 不校验写入; 1:校验写入
* Output:  0: success. 1: fail
*******************************************************************************/
uint16_t u16EepromWrite(uint16_t u16Address, uint16_t u16Data, uint8_t u8Mode)
{
	uint16_t res = 0;
	
	if (2 == u8Mode)
	{
		sgEepromArray.EepromData[sgEepromArray.u16EepromWr].u16Addr = u16Address;
		sgEepromArray.EepromData[sgEepromArray.u16EepromWr].u16Data = u16Data;
		sgEepromArray.u16EepromWr++;
		if (sgEepromArray.u16EepromWr >= EEPROM_LENGTH)
		{
			sgEepromArray.u16EepromWr = 0;
		}
		//sgEepromArray.b7RemainCnt++;
	}
	
	else if(1 == u8Mode)
	{
		res = EepromQualifiedWrite(u16Address, u16Data);
	}
	else
	{
		res = EepromWrite(u16Address, u16Data);
	}
	
//	if (0 != res)
//	{
//		i32ErrCodeSet(ErrCode69);		/*Eeprom is wrong*/
//		i32LogWrite(ERR, LOG_BSP, "Write Eeprom is wrong, Address = 0x%x, Data = 0x%x, Mode = %d\r\n", u16Address, u16Data, u8Mode);
//	}
	
//	vDelayms(10);

	return res;
}

/*******************************************************************************
* Name: uint16_t u16EepromRead(uint16_t u16Address, uint16_t u16Data, uint8_t u8Mode)
* Descriptio: Eeprom write Data
* Input: u16Address: eeprom address
*        u16Data: read data point
*		 u8Mode : 0: 不校验读取; 1:校验读取
* Output:  0: success. 1: fail
*******************************************************************************/
uint16_t u16EepromRead(uint16_t u16Address, uint16_t *u16Data, uint8_t u8Mode)
{
	uint16_t res;
	
	if(1 == u8Mode)
	{
		res = EepromQualifiedRead(u16Address, u16Data);
	}
	else
	{
		res = EepromRead(u16Address, u16Data);
	}
	
	return res;
}


uint8_t u8GetEepromFlag(void)
{
	return sgEepromArray.b1Flag;
}

uint8_t u8GetEepromErrFlag(void)
{
	return sgEepromArray.b1ErrFlag;
}

void vClrEepromErrFlag(void)
{
	sgEepromArray.b1ErrFlag = 0;
}



uint8_t u8GetEepromWriteReamin(void)
{
	/*lilu, 20240410 不考虑超出一圈的情况*/
	uint8_t Cnt = 0;
	if (sgEepromArray.u16EepromWr >= sgEepromArray.u16EepromRd)
	{
		Cnt = sgEepromArray.u16EepromWr - sgEepromArray.u16EepromRd;
	}
	else
	{
		Cnt = sgEepromArray.u16EepromWr + EEPROM_LENGTH - sgEepromArray.u16EepromRd;
	}
	return Cnt;
}

void vEepromNoBlockWriteProc(void)
{
	if (sgEepromArray.u16EepromRd != sgEepromArray.u16EepromWr)
	{
		if (0 == EepromWriteNoBlock(sgEepromArray.EepromData[sgEepromArray.u16EepromRd].u16Addr, sgEepromArray.EepromData[sgEepromArray.u16EepromRd].u16Data))
		{
			sgEepromArray.u16EepromRd++;
			if (sgEepromArray.u16EepromRd >= EEPROM_LENGTH)
			{
				sgEepromArray.u16EepromRd = 0;
			}
			if (sgEepromArray.b8RemainCnt > 0)
			{
				sgEepromArray.b8RemainCnt--;
			}
		}
	}
	
	
//	if (1 == sgEepromArray.b1ErrFlag)
//	{
//		if (0 == sgEepromArray.b1Flag)
//		{
//					for(index=0;index < (sizeof(cPara_Table)/sizeof(cPara_Table[0]));index++) 
//		{
//			u16ReadParaFromEeprom(index);
//		}
//			sgEepromArray.b1ErrFlag = 0;
//		}
//	}

}

void vEepromSetCallBack(EeepromCallBackt CallBack1, uint16_t u16ParaLastAddr, SaveParaFinishCallBackt CallBack2)
{
	sgEepromCallBack = CallBack1;
	sgU16ParaLastAddr = u16ParaLastAddr;
	sgSaveParaFinishCallBack = CallBack2;	
}
