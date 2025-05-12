/*******************************************************************************
* Filename: UserStackerTruckProc.h	                                           *
* Description: 逻辑层的头文件，包含对应的回调函数	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/07/22    														   *
* Revision:	 														 		   *
*******************************************************************************/
#ifndef _UDS_OTA_H_
#define _UDS_OTA_H_
#include "stdint.h"

#define	 PAGE_SIZE			2048

#define	FUNCTION_CANID		(0x7DF)
#define	PHYSICA_CANID		(0x721)
#define	RESPOND_CANID		(0x729)

#define	FLASH_INVALID		0xAA
#define	FLASH_VALID			0x55

#define	UDS_FRAME_LENGTH    8

typedef void (*FlashWrite)(uint32_t u32Addr, uint32_t u32Size, const uint32_t *u32WrData);
typedef void (*UdsRev)(uint32_t *u32CanId, uint32_t *u32DataLenth, uint8_t *u8RevData);
typedef void (*UdsSend)(uint32_t u32CanId, uint32_t u32DataLenth, const uint8_t *u8SendData);
typedef void (*SoftReset)(void);
typedef void (*FlashEraseInfo)(uint32_t u32Addr, uint32_t u32Size);
typedef void (*FlashEraseApp)(uint32_t u32Addr, uint32_t u32Size);
typedef void (*FlashEraseBackUp)(uint32_t u32Addr, uint32_t u32Size);
typedef void (*JumpToApp)(uint32_t);
typedef uint32_t (*AppCalc)(uint8_t *u8Src, uint32_t u32Size);
typedef uint8_t	(*SeedToKey)(uint8_t *u8Seed, uint32_t u32SeedLength, uint8_t *u8Key, uint32_t *u32KeyLength);
typedef void (*StopWdg)(void);
typedef void (*Delayms)(uint32_t);
typedef uint32_t (*PackCalc)(uint8_t *u8Src, uint32_t u32Size);

typedef struct
{
	uint32_t u32NodeId;			/*0xFFFF 代表首次上电，写入默认值*/
	union
	{
		uint32_t u32UpdateInfo;
		struct
		{
			uint8_t u8UpdateFlag;
			uint8_t u8BackUpFlag;	/**/
			uint8_t	u8AppEraseFlag;
			uint8_t u8AppCheckFlag;
		};
	};
	uint32_t u32DefaultValue;
	uint32_t u32CheckSum;
	
}xFlashInfo;

typedef enum
{
	UpdateFlag = 0,
	BackUpFlag, 
	AppEraseFlag,
	AppCheckFlag,
}eFlagNo;

typedef struct
{
	uint32_t u32UserAddr;						/*USER_ADDR*/
	uint32_t u32InfoAddr;						/*Info 地址*/
	uint32_t u32InfoBackUpAddr;					/*Info BackUp 地址*/
	uint32_t u32AppAddr;						/*App 地址*/
	uint32_t u32BackUpAddr;						/*备份地址*/
	uint32_t u32UserSize;						//
	uint32_t u32InfoSize;						//
	uint32_t u32AppSize;
	uint32_t u32BackUpSize;						
	uint32_t u32PhysicaCanId;					/*物理寻址canid*/			
	uint32_t u32FunctionCanId;					/*功能寻址canid*/
	uint32_t u32RespondCanId;					/*应答canid*/
	uint32_t u32FrameLength;					/*帧数据长度*/
	uint8_t u8SendFlag;							/*Can报文发送标志  1：不发送can报文*/
	uint8_t u8ErrCodeFlag;						/*故障码记录标志*  1：不记录故障码*/
	uint8_t u8Session;							/*会话模式*/
	uint8_t u8ArrayHw[5];						/*硬件版本号*/
	uint8_t u8ArraySw[5];						/*软件版本号*/
	uint16_t u16PackSize;						/*每包传输的长度*/
	uint8_t u8PackCheckSize;					/*包校验的长度，最大4个字节*/
	
	FlashEraseInfo InfoEraseCallBack;			/*参数擦除Flash*/
	FlashEraseApp AppEraseCallBack;				/*App擦除Flash*/
	FlashEraseBackUp BackUpEraseCallBack;		/*备份擦除Flash*/
	FlashWrite WrCallBack;						/*写Flash回调函数*/
	UdsSend UdsSendCallBack;					/*UdsSend回调函数*/	
	UdsRev UdsRevCallBack;						/*UdsRev CallBack*/
	SoftReset SResetCallBack;					/*软件复位回调函数*/
	JumpToApp JumpToAppCallBack;				/*跳转至APP回调函数*/
	AppCalc AppCalcCallBack;					/*文件校验算法*/
	SeedToKey SeedToKeyCallBack;				/*27服务*/
	StopWdg	StopWdgCallBack;					/*停止看门狗回调函数*/
	Delayms DelaymsCallBack;					/*ms callback*/
	PackCalc PackCalcCallBack;					/*包校验回调函数*/
}xUdsInfo;


/******************************************************************************
*函数定义
******************************************************************************/	

extern void vUdsInit(xUdsInfo *UdsInfo, xFlashInfo *FlashInfo);
extern void vUdsAppProc(uint32_t u32CanId, uint32_t u32Size, uint8_t *u8Data);
extern void vUpdateCheck(void);
//extern void vUpdateCanBaud(uint32_t u32CanBaud);
extern void vUpdateNodeId(uint32_t u32NodeId);
extern void vUpdateFlag(eFlagNo FlagNo, uint8_t u8Value);


#endif //#ifndef _USER_COMM_H_
