/*******************************************************************************
* Filename: UserStackerTruckProc.h	                                           *
* Description: �߼����ͷ�ļ���������Ӧ�Ļص�����	   						   *
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
	uint32_t u32NodeId;			/*0xFFFF �����״��ϵ磬д��Ĭ��ֵ*/
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
	uint32_t u32InfoAddr;						/*Info ��ַ*/
	uint32_t u32InfoBackUpAddr;					/*Info BackUp ��ַ*/
	uint32_t u32AppAddr;						/*App ��ַ*/
	uint32_t u32BackUpAddr;						/*���ݵ�ַ*/
	uint32_t u32UserSize;						//
	uint32_t u32InfoSize;						//
	uint32_t u32AppSize;
	uint32_t u32BackUpSize;						
	uint32_t u32PhysicaCanId;					/*����Ѱַcanid*/			
	uint32_t u32FunctionCanId;					/*����Ѱַcanid*/
	uint32_t u32RespondCanId;					/*Ӧ��canid*/
	uint32_t u32FrameLength;					/*֡���ݳ���*/
	uint8_t u8SendFlag;							/*Can���ķ��ͱ�־  1��������can����*/
	uint8_t u8ErrCodeFlag;						/*�������¼��־*  1������¼������*/
	uint8_t u8Session;							/*�Ựģʽ*/
	uint8_t u8ArrayHw[5];						/*Ӳ���汾��*/
	uint8_t u8ArraySw[5];						/*����汾��*/
	uint16_t u16PackSize;						/*ÿ������ĳ���*/
	uint8_t u8PackCheckSize;					/*��У��ĳ��ȣ����4���ֽ�*/
	
	FlashEraseInfo InfoEraseCallBack;			/*��������Flash*/
	FlashEraseApp AppEraseCallBack;				/*App����Flash*/
	FlashEraseBackUp BackUpEraseCallBack;		/*���ݲ���Flash*/
	FlashWrite WrCallBack;						/*дFlash�ص�����*/
	UdsSend UdsSendCallBack;					/*UdsSend�ص�����*/	
	UdsRev UdsRevCallBack;						/*UdsRev CallBack*/
	SoftReset SResetCallBack;					/*�����λ�ص�����*/
	JumpToApp JumpToAppCallBack;				/*��ת��APP�ص�����*/
	AppCalc AppCalcCallBack;					/*�ļ�У���㷨*/
	SeedToKey SeedToKeyCallBack;				/*27����*/
	StopWdg	StopWdgCallBack;					/*ֹͣ���Ź��ص�����*/
	Delayms DelaymsCallBack;					/*ms callback*/
	PackCalc PackCalcCallBack;					/*��У��ص�����*/
}xUdsInfo;


/******************************************************************************
*��������
******************************************************************************/	

extern void vUdsInit(xUdsInfo *UdsInfo, xFlashInfo *FlashInfo);
extern void vUdsAppProc(uint32_t u32CanId, uint32_t u32Size, uint8_t *u8Data);
extern void vUpdateCheck(void);
//extern void vUpdateCanBaud(uint32_t u32CanBaud);
extern void vUpdateNodeId(uint32_t u32NodeId);
extern void vUpdateFlag(eFlagNo FlagNo, uint8_t u8Value);


#endif //#ifndef _USER_COMM_H_
