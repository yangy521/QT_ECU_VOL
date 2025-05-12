/*******************************************************************************
* Filename: UserStackerTruckProc.c	                                           *
* Description: 逻辑层的源文件，包含对应的回调函数	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/05/30    														   *
* Revision:	 														 		   *
*******************************************************************************/
#include "UserCanComm.h"
#include "UserTest.h"
#include "ErrCode.h"
#include "IQmathLib.h"
#include "PropProc.h"
#include "CanRevProc.h"
#include "string.h"
#include "Para.h"
#include "WdgProc.h"
#include "math.h"
#include "Log.h"
#include "LocalAi.h"
#include "LedProc.h"
#include "AiProc.h"
#include "LocalDo.h"
#include "stdlib.h"
#include "Uds_Ota.h"



#define	BAUDRATE_ADDR		(0x08005000)
#define	BAUDRATE_SIZE		(2048)

#define	UPDATE_ADDR			(0x08005800)
#define	UPDATE_SIZE			(2048)

#define	USER_ADDR			(0x08006000)
#define USER_SIZE			(2048)				/*2KB*/

#define INFO_ADDR			(0x08006800)
#define INFO_SIZE			(2048)				/*2KB*/

#define	INFO_BACKUP_ADDR	(0x08007000)		/*Info backUp  2KB*/

#define APP_ADDR			(0x0800C000)
#define APP_SIZE			(102400)			/*100KB*/			

#define BACKUP_ADDR			(0x08025000)
#define BACKUP_SIZE			(102400)			/*100KB*/

#if (0)
	
const static xPdoParameter  PdoPara = 
{
	.TpdoPara = {
		{.u8Flag = 1, .u8Type = 0x0, .u16Period = 200},
		{.u8Flag = 1, .u8Type = 0x0, .u16Period = 200},
		{.u8Flag = 1, .u8Type = 0x0, .u16Period = 200},
		{.u8Flag = 1, .u8Type = 0x0, .u16Period = 200},

		{.u8Flag = 0, .u8Type = 0xFF, .u16Period = 200, .u16CanId = 0x110},
		{.u8Flag = 0, .u8Type = 0xFF, .u16Period = 200, .u16CanId = 0x111},
		{.u8Flag = 0, .u8Type = 0xFF, .u16Period = 200, .u16CanId = 0x112},
		{.u8Flag = 0, .u8Type = 0xFF, .u16Period = 200, .u16CanId = 0x113},
		{.u8Flag = 0, .u8Type = 0xFF, .u16Period = 200, .u16CanId = 0x114},
		{.u8Flag = 0, .u8Type = 0xFF, .u16Period = 200, .u16CanId = 0x115},
	},
	.RpdoPara = {
		{.b1Flag = 1, .b11CanRevId = 0x331},
		{.b1Flag = 1, .b11CanRevId = 0x332},
		{.b1Flag = 1, .b11CanRevId = 0x333},
		{.b1Flag = 1, .b11CanRevId = 0x334},
		{.b1Flag = 1, .b11CanRevId = 0x335},
		{.b1Flag = 1, .b11CanRevId = 0x336},
	},
};

xCanRevPdoInfo gCanRevPdoInfo;				/*PDO接收待完善*/
/*PDO发送待完善*/
xCanSendPdoInfo gCanSendPdoInfo = 
{
	.CanSendInfo1.u16SoftVer = 0x1000,
	.CanSendInfo2.u16HardVer = 0x1000,
};

#define	DEFAULT_SESSION		0
#define	PROGRAME_SESSION	1
#define	EXTEND_SESSION		2


static uint8_t	u8Session = DEFAULT_SESSION;
static xFlashInfo sgFlashInfo;
static xUdsInfo sgUdsInfo;
//xUdsInfo *pUdsInfo = NULL;
 



//void vReadFlashInfo(xFlashInfo *FlashInfo)
//{
//	memcpy(FlashInfo, (uint32_t*)INFO_ADDR, sizeof(xFlashInfo));
//	
//	if (FLASH_INVALID == FlashInfo->u8AppCheckFlag)
//	{
//		vUpdateFlag(AppCheckFlag, FLASH_VALID);
////		if (NULL != pUdsInfo->InfoEraseCallBack)
////		{
////			pUdsInfo->InfoEraseCallBack(pUdsInfo->u32InfoAddr, pUdsInfo->u32InfoSize);
////		}
////		FlashInfo->u8AppCheckFlag = FLASH_VALID;
////		if (NULL != pUdsInfo->WrCallBack)
////		{
////			pUdsInfo->WrCallBack(pUdsInfo->u32InfoAddr, sizeof(FlashInfo), (const uint32_t *)FlashInfo);
////		}
//		
////		uint8_t u8SendData[UDS_FRAME_LENGTH] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
////		u8SendData[0] = 0x02;
////		u8SendData[1] = 0x51;
////		u8SendData[2] = 0x01;
////		if ((NULL != pUdsInfo->UdsSendCallBack) && (NULL != pUdsInfo) && (0xAA != u8SendData[0]))
////		{
////			pUdsInfo->UdsSendCallBack(pUdsInfo->u32RespondCanId, UDS_FRAME_LENGTH, u8SendData);
////		}
//	}
//}


//void vUdsInit(xUdsInfo *UdsInfo)
//{
//	pUdsInfo = UdsInfo;
//}


#if 0
static void vUdsProc(uint8_t *u8Data)
{
	static uint8_t u8Flag = 0;	/*0x11: Write F184; 0x21: DownFlashDriver; 0x22: FlashDriverTrans; 0x23: FlashDirverCheck; 
								  0x30: EraseApp; 0x31: DownApp; 0x32: AppTrans; 0x33: AppCheck;*/
	static uint32_t u32AppLength = 0;
	static uint32_t u32AppCnt = 0;
	static uint16_t u16ByteNum = 0;
	static uint8_t u8FrameCnt = 0;
	static uint16_t u16ByteCnt = 0;
	static uint8_t u8SeedArray[5] = {'Q', 'T'};
	static uint32_t u32CheckRes = 0;
	uint8_t u8Key[4];
	uint32_t u32KeyLen = 0;
	uint32_t u32Seed = 0;
	uint8_t u8SendData[UDS_FRAME_LENGTH] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
	if (u8FrameCnt == u8Data[0])					/*MultiFrame */
	{
		if (0x32 == u8Flag)							/*AppTrans*/
		{
			if ((u16ByteCnt + 7) < u16ByteNum)
			{
				memcpy(u8FlashArray + u16ByteCnt, u8SendData + 1, 7);
				u16ByteCnt += 7;
				u8FrameCnt = (u8FrameCnt + 1) & 0x2F;
			}
			else
			{
				memcpy(u8FlashArray + u16ByteCnt, u8SendData + 1, u16ByteNum - u16ByteCnt);
				/*Write To Flash*/
				{
//					u32AppLength = u32AppLength - u16ByteNum;
					pUdsInfo->WrCallBack(pUdsInfo->u32AppAddr + u32AppCnt, u16ByteNum, (uint32_t*)u8FlashArray);		/*每次按照2K处理*/
					u32AppCnt = u32AppCnt + u16ByteNum;
				}
				u8SendData[0] = 0x01;
				u8SendData[1] = 0x76;
				u16ByteCnt = 0;
				u8FrameCnt = 0;
			}
		}
		else if (0x22 == u8Flag)					/*FlashDriverTrans*/
		{
			if ((u16ByteCnt + 7) < u16ByteNum)
			{
				u16ByteCnt += 7;
				u8FrameCnt = (u8FrameCnt + 1) & 0x2F;
			}
			else
			{
				u8SendData[0] = 0x01;
				u8SendData[1] = 0x76;
				u16ByteCnt = 0;
				u8FrameCnt = 0;
			}
		}
		else if (0x11 == u8Flag)					/*0xF184 Trans*/
		{
			if ((u16ByteCnt + 7) < u16ByteNum)
			{
				memcpy(u8FingerprintData + u16ByteCnt, u8Data + 1, 7);
				u16ByteCnt += 7;
				u8FrameCnt = (u8FrameCnt + 1) & 0x2F;
			}
			else
			{
				memcpy(u8FingerprintData + u16ByteCnt, u8Data + 1, u16ByteNum - u16ByteCnt);
				u16ByteCnt = 0;
				u8FrameCnt = 0;
				u8SendData[0] = 0x04;
				u8SendData[1] = 0x6E;
				u8SendData[2] = 0xF1;
				u8SendData[2] = 0x84;
			}
		}
		else if (0x21 == u8Flag)			/*Flash Request*/
		{
			if ((u16ByteCnt + 7) < u16ByteNum)
			{
				u16ByteCnt += 7;
				u8FrameCnt = (u8FrameCnt + 1) & 0x2F;
			}
			else
			{
				u8SendData[0] = 0x06;
				u8SendData[1] = 0x74;
				u8SendData[2] = 0x20;
				u8SendData[3] = 0x08;
				u8SendData[4] = 0x00;
				u16ByteCnt = 0;
				u8FrameCnt = 0;
			}
		}
		else if (0x31 == u8Flag)			/*App Request*/
		{
			if ((u16ByteCnt + 7) < u16ByteNum)
			{
				u16ByteCnt += 7;
				u8FrameCnt = (u8FrameCnt + 1) & 0x2F;
			}
			else
			{
				u8SendData[0] = 0x06;
				u8SendData[1] = 0x74;
				u8SendData[2] = 0x20;
				u8SendData[3] = 0x08;
				u8SendData[4] = 0x00;
				u32AppLength = (uint32_t)(u8Data[2] << 24) | (uint32_t)(u8Data[3] << 16) | (uint32_t)(u8Data[4] << 8) | (uint32_t)(u8Data[5] << 0);
				u32AppCnt = 0;
				u16ByteCnt = 0;
				u8FrameCnt = 0;
			}
		}
		
		else if (0x23 == u8Flag)						/*Flash Driver Check*/
		{
			if ((u16ByteCnt + 7) < u16ByteNum)
			{
				u16ByteCnt += 7;
				u8FrameCnt = (u8FrameCnt + 1) & 0x2F;
			}
			else
			{
				u8SendData[0] = 0x05;
				u8SendData[1] = 0x71;
				u8SendData[2] = 0x01;
				u8SendData[3] = 0xF0;		
				u8SendData[4] = 0x01;
				u8SendData[5] = 0x00;
				u16ByteCnt = 0;
				u8FrameCnt = 0;
			}
		}
		else if (0x33 == u8Flag)						/*App Check*/
		{
			if ((u16ByteCnt + 7) < u16ByteNum)
			{
				u16ByteCnt += 7;
				u8FrameCnt = (u8FrameCnt + 1) & 0x2F;
			}
			else
			{
				u32CheckRes |= (uint32_t)(u8Data[1] << 8) | (uint32_t)(u8Data[2] << 0);
				u8SendData[0] = 0x05;
				u8SendData[1] = 0x71;
				u8SendData[2] = 0x01;
				u8SendData[3] = 0xF0;		
				u8SendData[4] = 0x01;
				u8SendData[5] = 0x00;
				u16ByteCnt = 0;
				u8FrameCnt = 0;
			}
		}
		
		else if (0x30 == u8Flag)					/*App Erase requset*/		
		{
			if ((u16ByteCnt + 7) < u16ByteNum)
			{
				u16ByteCnt += 7;
				u8FrameCnt = (u8FrameCnt + 1) & 0x2F;
			}
			else
			{
				u8SendData[0] = 0x05;
				u8SendData[1] = 0x71;
				u8SendData[2] = 0x01;
				u8SendData[3] = 0xFF;		
				u8SendData[4] = 0x00;
				u8SendData[5] = 0x00;
				//u32AppLength = (uint32_t)(u8Data[3] << 24) | (uint32_t)(u8Data[4] << 16) | (uint32_t)(u8Data[5] << 8) | (uint32_t)(u8Data[6] << 0);
				/*Erase BackUp, Memcpy App To BackUp*/
				{
					sgFlashInfo.u8BackUpFlag = FLASH_INVALID;
					/*1st Write */
					if (NULL != pUdsInfo->InfoEraseCallBack)
					{
						pUdsInfo->InfoEraseCallBack(pUdsInfo->u32InfoAddr, pUdsInfo->u32InfoSize);
					}
					if (NULL != pUdsInfo->WrCallBack)
					{
						pUdsInfo->WrCallBack(pUdsInfo->u32InfoAddr, sizeof(xFlashInfo), (const uint32_t *)&sgFlashInfo);
					}
					if (NULL != pUdsInfo->BackUpEraseCallBack)
					{
						pUdsInfo->BackUpEraseCallBack(pUdsInfo->u32BackUpAddr, pUdsInfo->u32BackUpSize);		/*262144 = 256 * 1024, 256KB*/
					}
					
					{
						uint32_t i = 0;
						for (i=0; i<(pUdsInfo->u32AppSize >> 2); i++)		/*49152 = 192 * 1024 / 4*/
						{
							*(uint32_t*)(pUdsInfo->u32BackUpAddr + 4 * i) = *(uint32_t*)(pUdsInfo->u32AppAddr + 4 * i);
						}
						sgFlashInfo.u8BackUpFlag = FLASH_VALID;
						sgFlashInfo.u8AppEraseFlag = FLASH_INVALID;
						if (NULL != pUdsInfo->InfoEraseCallBack)
						{
							pUdsInfo->InfoEraseCallBack(pUdsInfo->u32InfoAddr, pUdsInfo->u32InfoSize);
						}
						if (NULL != pUdsInfo->WrCallBack)
						{
							pUdsInfo->WrCallBack(pUdsInfo->u32InfoAddr, sizeof(xFlashInfo), (const uint32_t *)&sgFlashInfo);
						}
						
						if (NULL != pUdsInfo->AppEraseCallBack)
						{
							pUdsInfo->AppEraseCallBack(pUdsInfo->u32AppAddr, pUdsInfo->u32AppSize);		/*196608 = 192 * 1024, 192KB*/
						}
					}
				}
				u16ByteCnt = 0;
				u8FrameCnt = 0;
			}
		}
	}
	else if ((0x02 == u8Data[0]) && (0x27 == u8Data[1]) && (0x07== u8Data[2]))		/*Request Seed*/
	{
		u32Seed = rand();
		u8SeedArray[2] = (uint8_t)(u32Seed >> 16);
		u8SeedArray[3] = (uint8_t)(u32Seed >> 8);
		u8SeedArray[4] = (uint8_t)(u32Seed >> 0);
		u8SendData[0] = 0x07;
		u8SendData[1] = 0x67;
		u8SendData[2] = 0x07;
		u8SendData[3] = u8SeedArray[0];
		u8SendData[4] = u8SeedArray[1];
		u8SendData[5] = u8SeedArray[2];
		u8SendData[6] = u8SeedArray[3];
		u8SendData[7] = u8SeedArray[4];
	}
	else if ((0x06 == u8Data[0]) && (0x27 == u8Data[1]) && (0x08 == u8Data[2]))	/*Check Key*/
	{
		u8SeedToKey(u8SeedArray, 5, u8Key, &u32KeyLen);
		if ((u8Key[0] == u8Data[3]) && (u8Key[1] == u8Data[4]) && (u8Key[2] == u8Data[6]) && (u8Key[0] == u8Data[7]))
		{
			u8SendData[0] = 0x02;
			u8SendData[1] = 0x67;
			u8SendData[2] = 0x08;
		}
	}
	else if ((0x00 == u8Flag) && (0x2E == u8Data[2]) && (0xF1 == u8Data[3]) && (0x84 == u8Data[4]))	/*Read F184*/
	{
		u8SendData[0] = 0x30;
		u8SendData[1] = 0x00;
		u8SendData[2] = 0x00;
		u8FingerprintData[0]= u8Data[5];
		u8FingerprintData[1]= u8Data[6];
		u8FingerprintData[2]= u8Data[7];
		u16ByteNum = (uint16_t)(u8Data[0] & 0x0F) << 8 | (uint16_t)u8Data[1];
		u16ByteCnt += 3;
		u8FrameCnt = 0x21;		/**/
		u8Flag = 0x11;
	}
	else if ((0x11 == u8Flag) && 
		     (((0x34 == u8Data[1]) && (u8Data[0] <= 7)) || ((0x34 == u8Data[2]) && (u8Data[0] > 7))))		/*Flash Driver Request*/
	{
		if (u8Data[0] <= 7)
		{
			u8SendData[0] = 0x06;
			u8SendData[1] = 0x74;
			u8SendData[2] = 0x20;
			u8SendData[3] = 0x08;
			u8SendData[4] = 0x00;
			u8Flag = 0x21;
		}
		else
		{
			u8SendData[0] = 0x30;
			u8SendData[1] = 0x00;
			u8SendData[2] = 0x00;
			u16ByteNum = (uint16_t)(u8Data[0] & 0x0F) << 8 | (uint16_t)u8Data[1];
			u16ByteCnt += 5;
			u8FrameCnt = 0x21;		/**/
			u8Flag = 0x21;
		}
	}
	else if ((0x21 == u8Flag) &&
			 (((0x36 == u8Data[1]) && (u8Data[0] <= 7)) || ((0x36 == u8Data[2]) && (u8Data[0] > 7))))		/*Flash Driver Trans*/
	{
		if (u8Data[0] <= 7)
		{
			u8SendData[0] = 0x01;
			u8SendData[1] = 0x76;
			u8Flag = 0x22;
		}
		else
		{
			u8SendData[0] = 0x30;
			u8SendData[1] = 0x00;
			u8SendData[2] = 0x00;
			u16ByteNum = (uint16_t)(u8Data[0] & 0x0F) << 8 | (uint16_t)u8Data[1];
			u16ByteCnt += 5;
			u8FrameCnt = 0x21;		/**/
			u8Flag = 0x22;
		}
	}
	else if ((0x01 == u8Data[0]) && (0x37 == u8Data[1]))											/*Trans Quit*/
	{
		u8SendData[0] = 0x01;
		u8SendData[1] = 0x77;
	}
	else if ((0x22 == u8Flag) && (0x31 == u8Data[2]) && (0x01 == u8Data[3]) && (0xF0 == u8Data[4]) && (0x01 == u8Data[5]))	/*Flash Driver Check*/
	{
		u8SendData[0] = 0x30;
		u8SendData[1] = 0x00;
		u8SendData[2] = 0x00;
		u32CheckRes = (uint32_t)(u8Data[6] << 24) | (uint32_t)(u8Data[7] << 16);
		u16ByteNum = (uint16_t)(u8Data[0] & 0x0F) << 8 | (uint16_t)u8Data[1];
		u16ByteCnt += 2;
		u8FrameCnt = 0x21;		/**/
		u8Flag = 0x23;
	}
	else if ((0x31 == u8Data[2]) && (0x01 == u8Data[3]) && (0xFF == u8Data[4]) && (0x00 == u8Data[5]))		/*Request Erase App*/
	{
		u8SendData[0] = 0x30;
		u8SendData[1] = 0x00;
		u8SendData[2] = 0x00;
		u16ByteNum = (uint16_t)(u8Data[0] & 0x0F) << 8 | (uint16_t)u8Data[1];
		u16ByteCnt += 2;
		u8FrameCnt = 0x21;		/**/
		u8Flag = 0x30;
	}
	else if ((0x30 == u8Flag) && 																			/*APP Request*/
		     (((0x34 == u8Data[1]) && (u8Data[0] <= 7)) || ((0x34 == u8Data[2]) && (u8Data[0] > 7))))
	{
		if (u8Data[0] <= 7)
		{
			u8SendData[0] = 0x04;
			u8SendData[1] = 0x74;
			u8SendData[2] = 0x20;
			u8SendData[3] = 0x08;
			u8SendData[4] = 0x00;
			u8Flag = 0x31;
		}
		else
		{
			u8SendData[0] = 0x30;
			u8SendData[1] = 0x00;
			u8SendData[2] = 0x00;
			u16ByteNum = (uint16_t)(u8Data[0] & 0x0F) << 8 | (uint16_t)u8Data[1];
			u16ByteCnt += 5;
			u8FrameCnt = 0x21;		/**/
			u8Flag = 0x31;
		}
	}
	else if ((0x31 == u8Flag) &&																			/*App Trans*/
			 (((0x36 == u8Data[1]) && (u8Data[0] <= 7)) || ((0x36 == u8Data[2]) && (u8Data[0] > 7))))
	{
		if (u8Data[0] <= 7)
		{
			u8SendData[0] = 0x01;
			u8SendData[1] = 0x76;
			u8Flag = 0x32;
		}
		else
		{
			u8SendData[0] = 0x30;
			u8SendData[1] = 0x00;
			u8SendData[2] = 0x00;
			u16ByteNum = (uint16_t)(u8Data[0] & 0x0F) << 8 | (uint16_t)u8Data[1];
			u16ByteCnt += 5;
			memcpy(u8FlashArray, u8SendData + 3, 5);
			u8FrameCnt = 0x21;		/**/
			u8Flag = 0x32;
		}
	}
	else if ((0x32 == u8Flag) && (0x31 == u8Data[2]) && (0x01 == u8Data[3]) && (0xF0 == u8Data[4]) && (0x01 == u8Data[5]))	/*App Check*/
	{
		u8SendData[0] = 0x30;
		u8SendData[1] = 0x00;
		u8SendData[2] = 0x00;
		u32CheckRes = (uint32_t)(u8Data[6] << 24) | (uint32_t)(u8Data[7] << 16);
		u16ByteNum = (uint16_t)(u8Data[0] & 0x0F) << 8 | (uint16_t)u8Data[1];
		u16ByteCnt += 2;
		u8FrameCnt = 0x21;		/**/
		u8Flag = 0x33;
	}
	
	else if ((0x04 == u8Data[0]) && (0x31 == u8Data[1]) && (0x01 == u8Data[2]) && (0xFF == u8Data[3]) && (0x01 == u8Data[4]))/*App Check*/
	{
		u8SendData[0] = 0x05;
		u8SendData[1] = 0x71;
		u8SendData[2] = 0x01;
		u8SendData[3] = 0xFF;
		u8SendData[4] = 0x01;
		u8SendData[5] = 0x00;
	}
	else if ((0x02 == u8Data[0]) && (0x11 == u8Data[1]) && (0x01 == u8Data[2]))/*Soft Reset*/
	{
		u8SendData[0] = 0x02;
		u8SendData[1] = 0x51;
		u8SendData[2] = 0x01;
	}
	{
		if ((NULL != pUdsInfo->UdsSendCallBack) && (NULL != pUdsInfo) && (0xAA != u8SendData[0]))
		{
			pUdsInfo->UdsSendCallBack(pUdsInfo->u32RespondCanId, UDS_FRAME_LENGTH, u8SendData);
		}
	}	
}


void vBootUdsProc(uint32_t u32CanId, uint32_t u32Size, uint8_t *u8Data)
{
	if ((u32CanId == pUdsInfo->u32PhysicaCanId) && (u32Size == pUdsInfo->u32FrameLength))
	{
		vUdsProc(u8Data);
	}
}
#endif
//void vUpdateCheck(void)
//{
//	xFlashInfo FlashInfoTmp;
//	
//	vReadFlashInfo(&sgFlashInfo);
//	
//	/*没有升级，主份程序OK，直接跳转至主份*/
//	if ((FLASH_INVALID == sgFlashInfo.u8UpdateFlag) && (FLASH_VALID == sgFlashInfo.u8AppCheckFlag))
//	{
//		
//	}
//	/*升级完成之后的重启操作， 将升级标记清除，AppCheck设置成无效，跳转至主份APP*/
//	else if (0x11 == sgFlashInfo.u8UpdateFlag)
//	{
//		sgFlashInfo.u8UpdateFlag = FLASH_INVALID;
//		sgFlashInfo.u8AppCheckFlag = FLASH_INVALID;
//		vWriteFlashInfo(sgFlashInfo);
//	}
//	/*有更新APP请求，等待升级*/
//	else if ((FLASH_VALID == sgFlashInfo.u8UpdateFlag) && (FLASH_VALID == sgFlashInfo.u8BackUpFlag) && (FLASH_VALID == sgFlashInfo.u8AppEraseFlag) && (FLASH_VALID == sgFlashInfo.u8AppCheckFlag))
//	{
//		
//	}
//	/*备份被擦除，主份还在，将主份赋值给备份*/
//	else if ((FLASH_VALID == sgFlashInfo.u8UpdateFlag) && (FLASH_INVALID == sgFlashInfo.u8BackUpFlag) && (FLASH_VALID == sgFlashInfo.u8AppEraseFlag) && (FLASH_VALID == sgFlashInfo.u8AppCheckFlag))
//	{
//		/*Erase BackUp Sector*/
//		uint32_t i = 0;
//		for (i = FLASH_APP_ADD; i < FLASH_BACKUP_ADD; i = i + 4)
//		{
//			*(uint32_t*)(FLASH_BACKUP_ADD + i) = *(uint32_t*)(FLASH_APP_ADD + i);
//		}
//		/*Jump to App*/
//		
//	}
//	/*备份OK，主份被擦除，将备份赋值给主份*/
//	else if ((FLASH_VALID == sgFlashInfo.u8UpdateFlag) && (FLASH_VALID == sgFlashInfo.u8BackUpFlag) && (FLASH_INVALID == sgFlashInfo.u8AppEraseFlag) && (FLASH_VALID == sgFlashInfo.u8AppCheckFlag))
//	{
//		/*Erase App Sector*/
//		uint32_t i = 0;
//		for (i = FLASH_APP_ADD; i < FLASH_BACKUP_ADD; i = i + 4)
//		{
//			*(uint32_t*)(FLASH_APP_ADD + i) = *(uint32_t*)(FLASH_BACKUP_ADD + i);
//		}
//		/*Jump to App*/
//	}
//	/*升级结束,主份程序存在异常，将备份赋值给主份*/
//	else if ((FLASH_INVALID == sgFlashInfo.u8UpdateFlag) && (FLASH_VALID == sgFlashInfo.u8BackUpFlag) && (FLASH_VALID == sgFlashInfo.u8AppEraseFlag) && (FLASH_INVALID == sgFlashInfo.u8AppCheckFlag))
//	{
//		/*Erase App Sector*/
//		
//		uint32_t i = 0;
//		for (i = FLASH_APP_ADD; i < FLASH_BACKUP_ADD; i = i + 4)
//		{
//			*(uint32_t*)(FLASH_APP_ADD + i) = *(uint32_t*)(FLASH_BACKUP_ADD + i);
//		}
//		/*Jump to App*/
//	}
//}

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
#
static void vCanIdUdsPhysicallyProc(tCanFrame * CanFrame)
//static void vCanIdUdsPhysicallyProc(uint8_t *u8Data)
{
//	static uint8_t u8Flag = 0;
//	uint8_t u8SendData[UDS_FRAME_LENGTH] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
//	
//	uint8_t u8Data[8];
//	memcpy(u8Data, CanFrame->u8Data, 8);
	vUdsAppProc(CanFrame->u32ID, CanFrame->u16DataLength, CanFrame->u8Data);
		
//    if ((0x04 == u8Data[0]) && (0x31 == u8Data[1]) && (0x01 == u8Data[2]) && (0xF0 == u8Data[3]) && (0x02 == u8Data[4]))
//    {
//		u8SendData[0] = 0x05;
//		u8SendData[1] = 0x71;
//		u8SendData[2] = 0x01;
//		u8SendData[3] = 0xF0;
//		u8SendData[4] = 0x02;
//		u8SendData[5] = 0x00;
//		/*添加停止电机相关动作等*/
//    }
//	else if ((0x04 == u8Data[0]) && (0x14 == u8Data[1]) && (0xFF== u8Data[2]) && (0xFF == u8Data[3]) && (0xFF == u8Data[4]))
//	{
//		u8SendData[0] = 0x01;
//		u8SendData[1] = 0x54;
//	}
//    else if ((0x03 == u8Data[0]) && (0x22 == u8Data[1]) && (0xF1== u8Data[2]) && (0x91 == u8Data[3]))
//    {
//		u8SendData[0] = 0x10;
//		u8SendData[1] = 0x08;
//		u8SendData[2] = 0x62;
//		u8SendData[3] = 0xF1;
//		u8SendData[4] = 0x91;
//		memcpy(u8SendData + 5, pUdsInfo->u8ArrayHw, 3);
//		u8Flag = 0x01;
//    }
//    else if ((0x03 == u8Data[0]) && (0x22 == u8Data[1]) && (0xF1== u8Data[2]) && (0xA0 == u8Data[3]))
//    {
//		u8SendData[0] = 0x10;
//		u8SendData[1] = 0x08;
//		u8SendData[2] = 0x62;
//		u8SendData[3] = 0xF1;
//		u8SendData[4] = 0xA0;
//		memcpy(u8SendData + 5, pUdsInfo->u8ArraySw, 3);
//		u8Flag = 0x02;
//    }
//    else if ((0x02 == u8Data[0]) && (0x10 == u8Data[1]) && (0x02 == u8Data[2]))
//    {
//		if (EXTEND_SESSION == u8Session)
//		{
//			u8SendData[0] = 0x03;
//			u8SendData[1] = 0x7F;
//			u8SendData[2] = 0x10;
//			u8SendData[3] = 0x78;
//			/*Send 0x78 right Now*/
//			if ((NULL != pUdsInfo->UdsSendCallBack) && (NULL != pUdsInfo) && (0xAA != u8SendData[0]))
//			{
//				pUdsInfo->UdsSendCallBack(pUdsInfo->u32RespondCanId, UDS_FRAME_LENGTH, u8SendData);
//			}
//			//fwdgt_write_disable();
//			/*stop motor*/
//			{
//				
//			}
//			
//			/*write flag to flash*/
//			if (NULL != pUdsInfo)
//			{
//				memcpy(&sgFlashInfo, (uint32_t*)(pUdsInfo->u32InfoAddr), sizeof(xFlashInfo));
//				if (NULL != pUdsInfo->InfoEraseCallBack)
//				{
//					pUdsInfo->InfoEraseCallBack(pUdsInfo->u32InfoAddr, pUdsInfo->u32InfoSize);
//				}
//				
//				sgFlashInfo.u8UpdateFlag = FLASH_VALID;
//				sgFlashInfo.u8AppCheckFlag = FLASH_VALID;
//				sgFlashInfo.u8AppEraseFlag = FLASH_VALID;
//				sgFlashInfo.u8BackUpFlag = FLASH_VALID;
//				if (NULL != pUdsInfo->WrCallBack)
//				{
//					pUdsInfo->WrCallBack(pUdsInfo->u32InfoAddr, sizeof(sgFlashInfo), (const uint32_t *)&sgFlashInfo);
//				}
//			}
//			rcu_periph_clock_disable(RCU_WWDGT);  /*enter bootloader close Wdg */
//			/*softreset*/
//			if ((NULL != pUdsInfo) && (NULL != pUdsInfo->SResetCallBack))
//			{
//				vDelayms(5); 
//				pUdsInfo->SResetCallBack();
//			}
//			
//		}
//	}
//	else if ((0x30 == u8Data[0]) && (0x00 == u8Data[1]) && (0x00 == u8Data[2]))
//	{
//		if (0x01 == u8Flag)
//		{
//			u8SendData[0] = 0x21;
//			memcpy(u8SendData + 1, pUdsInfo->u8ArrayHw + 3, 2);
//			u8Flag = 0;
//		}
//		else if (0x02 == u8Flag)
//		{
//			u8SendData[0] = 0x21;
//			memcpy(u8SendData + 1, pUdsInfo->u8ArraySw + 3, 2);
//			u8Flag = 0;
//		}
//	}
//		
//	{
//		if ((NULL != pUdsInfo->UdsSendCallBack) && (NULL != pUdsInfo) && (0xAA != u8SendData[0]))
//		{
//			pUdsInfo->UdsSendCallBack(pUdsInfo->u32RespondCanId, UDS_FRAME_LENGTH, u8SendData);
//		}
//	}
	i32LogWrite(DEBUG, LOG_USER, "Process UDS Frame!!!\r\n");
}

static void vCanIdUdsFunctionallyProc(tCanFrame * CanFrame)
//static void vCanIdUdsFunctionallyProc(uint8_t *u8Data)
{
	vUdsAppProc(CanFrame->u32ID, CanFrame->u16DataLength, CanFrame->u8Data);
	
//	uint8_t u8Data[8];
//	memcpy(u8Data, CanFrame->u8Data, 8);
//	
//	if ((0x02 == u8Data[0]) && (0x10 == u8Data[1]) && (0x03 == u8Data[2]))
//    {
//		u8SendData[0] = 0x06;
//		u8SendData[1] = 0x50;
//		u8SendData[2] = 0x03;
//		u8SendData[3] = 0x00;
//		u8SendData[4] = 0x32;
//		u8SendData[5] = 0x00;
//		u8SendData[6] = 0xC8;
//		if ((NULL != pUdsInfo->UdsSendCallBack) && (NULL != pUdsInfo))
//		{
//			pUdsInfo->UdsSendCallBack(pUdsInfo->u32RespondCanId, UDS_FRAME_LENGTH, u8SendData);
//		}
//	
//		u8Session = EXTEND_SESSION;
//    }
//	
//	else if ((0x02 == u8Data[0]) && (0x10 == u8Data[1]) && (0x81 == u8Data[2]))
//	{
//		/*default session*/
//		u8Session = DEFAULT_SESSION;
//	}
//	else if ((0x02 == u8Data[0]) && (0x85 == u8Data[1]) && (0x82 == u8Data[2]))
//    {
//		/*close DTC*/
//		if (NULL != pUdsInfo)
//		{
//			pUdsInfo->u8ErrCodeFlag = 1;
//		}
//    }
//	else if ((0x02 == u8Data[0]) && (0x85 == u8Data[1]) && (0x81 == u8Data[2]))
//	{
//		/*open DTC*/
//		if (NULL != pUdsInfo)
//		{
//			pUdsInfo->u8ErrCodeFlag = 0;
//		}
//	}
//    else if ((0x03 == u8Data[0]) && (0x28 == u8Data[1]) && (0x83== u8Data[2]) && (0x01 == u8Data[3]))
//    {
//		/*stop communication*/
//		if (NULL != pUdsInfo)
//		{
//			pUdsInfo->u8SendFlag = 1;
//		}
//    }
//	else if ((0x03 == u8Data[0]) && (0x28 == u8Data[1]) && (0x80== u8Data[2]) && (0x01 == u8Data[3]))
//    {
//		/*start communication*/
//		if (NULL != pUdsInfo)
//		{
//			pUdsInfo->u8SendFlag = 0;
//		}
//    }
}


//void UdsProc(uint32_t u32CanId, uint32_t u32Size, uint8_t *u8Data)
//{
//	if ((u32CanId == pUdsInfo->u32PhysicaCanId) && (u32Size == pUdsInfo->u32FrameLength))
//	{
//		vCanIdUdsPhysicallyProc(u8Data);
//	}
//	else if ((u32CanId == pUdsInfo->u32FunctionCanId) && (u32Size == pUdsInfo->u32FrameLength))
//	{
//		vCanIdUdsFunctionallyProc(u8Data);
//	}
//	else
//	{
//		
//	}
//}

static void vSoftReset(void)
{
	__set_FAULTMASK(1);
	NVIC_SystemReset();
}

static void vUdsSend(uint32_t u32CanId, uint32_t u32Length, const uint8_t *u8Data)
{
	tCanFrame CanSendFrame;
	
	CanSendFrame.u32ID = u32CanId;
	CanSendFrame.u8Rtr = 0;
	CanSendFrame.u16DataLength = u32Length;
	memcpy(CanSendFrame.u8Data, u8Data, u32Length);
	
	i32CanWrite(Can0, &CanSendFrame);
}

static void vInfoErase(uint32_t u32Addr, uint32_t u32Length)
{
	fmc_unlock();
	fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);
	fmc_page_erase(u32Addr);
	fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);
	fmc_lock();
}

static void vFlashWrite(uint32_t u32Addr, uint32_t u32Size, const uint32_t *u32WrData)
{
    uint32_t i;
    fmc_unlock();
    
    /* Clear All pending flags */
    fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);
    
    i = 0;
    for (i=0; i<(u32Size >> 2); i++)
    {
        fmc_word_program(u32Addr + 4 * i, u32WrData[i]);
        fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);
    }
    
    fmc_lock();
    
//    return TransferStatus;
}

static void vStopWdg(void)
{
	rcu_periph_clock_disable(RCU_WWDGT); 
	i32LocalDoSet(DO_DRIVEREN, 0);
}

//#define	CAN1_LOST_ID		0x231
//#define	CAN2_LOST_ID		0x232
//#define	CAN3_LOST_ID		0x233
//#define	CAN4_LOST_ID		0x234
//#define	CAN5_LOST_ID		0x235
//#define	CAN6_LOST_ID		0x236
//#define	CAN7_LOST_ID		0x237
//#define	CAN8_LOST_ID		0x238

//uint8_t u8CanLost = 0;
//static void vCanLost(uint32_t u32CanId, uint8_t u8Func)
//{
//	switch(u32CanId)
//	{
//		case CAN1_LOST_ID:
//			if (CAN_NORMAL == u8Func)
//			{
//				i32LogWrite(DEBUG, LOG_USER, "CAN1_ID Normal!!!\r\n");
//				u8CanLost &= ~(1 << 0);
//			}
//			else
//			{
//				i32LogWrite(DEBUG, LOG_USER, "CAN1_ID Lost!!!\r\n");
//				u8CanLost |= 1 << 0;
//			}
//			break;
//		case CAN2_LOST_ID:
//			if (CAN_NORMAL == u8Func)
//			{
//				i32LogWrite(DEBUG, LOG_USER, "CAN2_ID Normal!!!\r\n");
//				u8CanLost &= ~(1 << 1);
//			}
//			else
//			{
//				i32LogWrite(DEBUG, LOG_USER, "CAN2_ID Lost!!!\r\n");
//				u8CanLost |= 1 << 1;
//			}
//			break;
//		case CAN3_LOST_ID:
//			if (CAN_NORMAL == u8Func)
//			{
//				i32LogWrite(DEBUG, LOG_USER, "CAN3_ID Normal!!!\r\n");
//				u8CanLost &= ~(1 << 2);
//			}
//			else
//			{
//				i32LogWrite(DEBUG, LOG_USER, "CAN3_ID Lost!!!\r\n");
//				u8CanLost |= 1 << 2;
//			}
//			break;
//		case CAN4_LOST_ID:
//			if (CAN_NORMAL == u8Func)
//			{
//				i32LogWrite(DEBUG, LOG_USER, "CAN4_ID Normal!!!\r\n");
//				u8CanLost &= ~(1 << 3);
//			}
//			else
//			{
//				i32LogWrite(DEBUG, LOG_USER, "CAN4_ID Lost!!!\r\n");
//				u8CanLost |= 1 << 3;
//			}
//			break;
//		case CAN5_LOST_ID:
//			if (CAN_NORMAL == u8Func)
//			{
//				i32LogWrite(DEBUG, LOG_USER, "CAN5_ID Normal!!!\r\n");
//				u8CanLost &= ~(1 << 4);
//			}
//			else
//			{
//				i32LogWrite(DEBUG, LOG_USER, "CAN5_ID Lost!!!\r\n");
//				u8CanLost |= 1 << 4;
//			}
//			break;
//		case CAN6_LOST_ID:
//			if (CAN_NORMAL == u8Func)
//			{
//				i32LogWrite(DEBUG, LOG_USER, "CAN6_ID Normal!!!\r\n");
//				u8CanLost &= ~(1 << 5);
//			}
//			else
//			{
//				i32LogWrite(DEBUG, LOG_USER, "CAN6_ID Lost!!!\r\n");
//				u8CanLost |= 1 << 5;
//			}
//			break;
//		case CAN7_LOST_ID:
//			if (CAN_NORMAL == u8Func)
//			{
//				i32LogWrite(DEBUG, LOG_USER, "CAN7_ID Normal!!!\r\n");
//				u8CanLost &= ~(1 << 6);
//			}
//			else
//			{
//				i32LogWrite(DEBUG, LOG_USER, "CAN7_ID Lost!!!\r\n");
//				u8CanLost |= 1 << 6;
//			}
//			break;
//		case CAN8_LOST_ID:
//			if (CAN_NORMAL == u8Func)
//			{
//				i32LogWrite(DEBUG, LOG_USER, "CAN8_ID Normal!!!\r\n");
//				u8CanLost &= ~(1 << 7);
//			}
//			else
//			{
//				i32LogWrite(DEBUG, LOG_USER, "CAN8_ID Lost!!!\r\n");
//				u8CanLost |= 1 << 7;
//			}
//			break;
//		default:
//			break;
//		
//	}
//}

static void vCanBuadSwicthProc(tCanFrame * CanFrame)
{
//	if (0x51 0x54 0x42 0x80 0x83 0x00 0x7D 0x67)
	uint8_t u8Data[8];
	uint16_t u16CanBaud = 0;
	uint16_t u16NodeId = 0;
	uint8_t u8Sum = 0, i = 0;
	
	
	memcpy(u8Data, CanFrame->u8Data, 8);
	
	if (0x51 == u8Data[0] && 0x54 == u8Data[1] && 0x42 == u8Data[2] && 0x80 == u8Data[3] && 0x83 == u8Data[4])
	{
		for (i=0; i<7; i++)
		{
			u8Sum += u8Data[i];
		}
		if (u8Sum == u8Data[7])
		{
			u16CanBaud = (uint16_t)(u8Data[5] << 8) | u8Data[6];
//			vUpdateCanBaud(u16CanBaud);
		}
	}
	else if (0x4E == u8Data[0] && 0x6F == u8Data[1] && 0x64 == u8Data[2] && 
			 0x65 == u8Data[3] && 0x49 == u8Data[4] && 0x64 == u8Data[5])
	{
		u16NodeId = (uint16_t)(u8Data[6] << 8) | u8Data[7];
		vUpdateNodeId(u16NodeId);
	}
	else
	{
		
	}
//	vUdsAppProc(CanFrame->u32ID, CanFrame->u16DataLength, CanFrame->u8Data);
}

static void vCanUpdateProc(tCanFrame * CanFrame)
{
	fmc_unlock();
	fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);
	fmc_page_erase(UPDATE_ADDR);
	fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);
	fmc_lock();
	
	uint32_t u32Flag = 0xAAAAAAAA;
	
	vFlashWrite(UPDATE_ADDR, sizeof(uint32_t), &u32Flag);
	vDelayms(10);
	rcu_periph_clock_disable(RCU_WWDGT); 
	i32LocalDoSet(DO_DRIVEREN, 0);
	
	vDelayms(10);
	
	vSoftReset();
}
/*******************************************************************************
* Name: void vUserEcuInit(void)
* Descriptio: UserEcu初始化函数
* Input: NULL
* Output: NULL  
*******************************************************************************/
void vUserEcuInit(void)
{	
	uint32_t u32PhyiscalId = 0;
	{
		u32PhyiscalId = *(uint32_t*)(INFO_ADDR);
		
		if (0xFFFFFFFF == u32PhyiscalId)
		{
			u32PhyiscalId = PHYSICA_CANID;
		}
	}
	xRevCallBackProc CanIdFunctionallyUds = {.u32CanId = FUNCTION_CANID, .u32Data = 0, .CallBack = vCanIdUdsFunctionallyProc};
	xRevCallBackProc CanIdPhysicallyUds = {.u32CanId =u32PhyiscalId/*PHYSICA_CANID*/, .u32Data = 0, .CallBack = vCanIdUdsPhysicallyProc};
	xRevCallBackProc CanBaud = {.u32CanId = 0x0142414E, .u32Data = 0, .CallBack = vCanBuadSwicthProc};
	xRevCallBackProc CanUpDate = {.u32CanId = 0x02C281E2, .u32Data = 0, .CallBack = vCanUpdateProc};
	
//	CanIdPhysicallyUds.u32CanId = u32PhyiscalId;
	
	vCanRevMsgRegister(&CanIdFunctionallyUds);
	vCanRevMsgRegister(&CanIdPhysicallyUds);
	vCanRevMsgRegister(&CanBaud);
	vCanRevMsgRegister(&CanUpDate);
	/*Uds Init*/
	sgUdsInfo.u32InfoAddr = INFO_ADDR;
	sgUdsInfo.u32InfoSize = INFO_SIZE;
	sgUdsInfo.u32InfoBackUpAddr = INFO_BACKUP_ADDR;
	
	sgUdsInfo.u32AppAddr = APP_ADDR;
	sgUdsInfo.u32AppSize = APP_SIZE;
	
	sgUdsInfo.u32BackUpAddr = BACKUP_ADDR;
	sgUdsInfo.u32BackUpSize = BACKUP_SIZE;
	
	sgUdsInfo.u32FrameLength = UDS_FRAME_LENGTH ;
	/*HW09.06*/
	sgUdsInfo.u8ArrayHw[0] = 0x48;
	sgUdsInfo.u8ArrayHw[1] = 0x56;
	sgUdsInfo.u8ArrayHw[2] = 0x09;
	sgUdsInfo.u8ArrayHw[3] = 0x2E;
	sgUdsInfo.u8ArrayHw[4] = 0x13;
	/*SW09.06*/
	sgUdsInfo.u8ArraySw[0] = 0x53;
	sgUdsInfo.u8ArraySw[1] = 0x56;
	sgUdsInfo.u8ArraySw[2] = 0x09;
	sgUdsInfo.u8ArraySw[3] = 0x2E;
	sgUdsInfo.u8ArraySw[4] = 0x22;
	
	sgUdsInfo.u32FunctionCanId = FUNCTION_CANID;
	sgUdsInfo.u32PhysicaCanId = PHYSICA_CANID;
	sgUdsInfo.u32RespondCanId = RESPOND_CANID;
	
	sgUdsInfo.SResetCallBack = vSoftReset;
	sgUdsInfo.DelaymsCallBack = vDelayms;
	sgUdsInfo.UdsSendCallBack = vUdsSend;
	sgUdsInfo.StopWdgCallBack = vStopWdg;
	sgUdsInfo.WrCallBack = vFlashWrite;
	sgUdsInfo.InfoEraseCallBack = vInfoErase;
//	sgUdsInfo.AppEraseCallBack = vAppErase;
//	sgUdsInfo.BackUpEraseCallBack = vBackUpErase;
//	sgUdsInfo.JumpToAppCallBack = IAP_JumpToApp;
//	sgUdsInfo.SeedToKeyCallBack = u8SeedToKey;
//	sgUdsInfo.AppCalcCallBack = u32AppCalc;
	sgUdsInfo.AppCalcCallBack = NULL;
	sgUdsInfo.u16PackSize = PAGE_SIZE;
	sgUdsInfo.u8PackCheckSize = 0;
	sgUdsInfo.PackCalcCallBack = NULL;
	
	//vUdsInit
	vUdsInit(&sgUdsInfo, &sgFlashInfo);

	//pUdsInfo = &sgUdsInfo;
	if (FLASH_INVALID == sgFlashInfo.u8AppCheckFlag)
	{
		vUpdateFlag(AppCheckFlag, FLASH_VALID);
		uint8_t u8SendData[UDS_FRAME_LENGTH] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
		u8SendData[0] = 0x02;
		u8SendData[1] = 0x51;
		u8SendData[2] = 0x01;
		if ((NULL != sgUdsInfo.UdsSendCallBack) && (0xAA != u8SendData[0]))
		{
			sgUdsInfo.UdsSendCallBack(sgUdsInfo.u32RespondCanId, UDS_FRAME_LENGTH, u8SendData);
		}
	}
	
	
	vSetPdoPara(PdoPara);
	
//	vCanIdLostReg(CAN1_LOST_ID, 5000, vCanLost);
//	vCanIdLostReg(CAN2_LOST_ID, 5000, vCanLost);
//	vCanIdLostReg(CAN3_LOST_ID, 5000, vCanLost);
//	vCanIdLostReg(CAN4_LOST_ID, 5000, vCanLost);
//	vCanIdLostReg(CAN5_LOST_ID, 5000, vCanLost);
//	vCanIdLostReg(CAN6_LOST_ID, 5000, vCanLost);
//	vCanIdLostReg(CAN7_LOST_ID, 5000, vCanLost);
//	vCanIdLostReg(CAN8_LOST_ID, 5000, vCanLost);
	
	vSetNetTimer(TIMER_EcuPowerOn, 1000);	/**/
}

/*******************************************************************************
* Name: void vUserEcuProc(void)
* Descriptio: UserEcu用户处理函数
* Input: NULL
* Output: NULL  
*******************************************************************************/
void vUserEcuProc(void)
{
	static uint8_t u8EcuProcFlag = 0;
	uint8_t u8ErrCode = 0;
	uint8_t i = 0;
	uint32_t u32DiValue = 0;
	
	if(true == u8GetNetTimerOverFlag(TIMER_EcuPowerOn))
	{
		u8EcuProcFlag = 1;
		vKillNetTimer(TIMER_EcuPowerOn);
	}
	
	if(1 == u8EcuProcFlag)
	{
		for (i=0; i<LocalDiMax; i++)
		{
			if(1 == i32LocalDiGet(i))
			{
				u32DiValue |= 1 << i;
			}
		}
		memcpy(gCanSendPdoInfo.CanSendInfo1.u8Data, &u32DiValue, 4);
		for(i=0; i<7; i++) 
		{
			if (1 == i32LocalDiGet(SWI1_R + i))
			{
				i32DoPwmSet(DRIVER4 + i, 1);
			}
			else
			{
				i32DoPwmSet(DRIVER4 + i, 0);
			}
		}
		
		if (1 == i32LocalDiGet(SWI8_R))
		{
			i32DoPwmSet(DRIVER3, 1);
			uint32_t i32PropValue = 0;
			if (i32LocalAiGet(AI_B_AI1_R) >= 20)
			{
				i32PropValue = _IQ((1.0 * i32LocalAiGet(AI_B_AI1_R) / 4096) / PROPD_STD_CURRENT);
			}
			vPropSetTarget(PropDriverCh0, i32PropValue);
			
			i32PropValue = 0;
			if (i32LocalAiGet(AI_B_AI2_R) >= 20)
			{
				i32PropValue = _IQ((1.0 * i32LocalAiGet(AI_B_AI2_R) / 4096) / PROPD_STD_CURRENT);
			}
			vPropSetTarget(PropDriverCh1, i32PropValue);	
		}
		else
		{
			i32DoPwmSet(DRIVER3, 0);
			vPropSetTarget(PropDriverCh0, 0);
			vPropSetTarget(PropDriverCh1, 0);
		}

		{
			__disable_irq();		
			gCanSendPdoInfo.CanSendInfo2.u16Ai1Vol = i32LocalAiGetValue(AI_B_AI1_R);
			gCanSendPdoInfo.CanSendInfo2.u16Ai2Vol = i32LocalAiGetValue(AI_B_AI2_R);
			gCanSendPdoInfo.CanSendInfo2.u16Ai3Vol = i32LocalAiGetValue(AI_B_AI3_R);
			__enable_irq();
			
			__disable_irq();		
			gCanSendPdoInfo.CanSendInfo3.u16KsiVol = i32LocalAiGetValue(AI_B_KSI_CHECK);
			gCanSendPdoInfo.CanSendInfo3.u16VbusVol = i32LocalAiGetValue(AI_B_VBUS_CHECK);
			gCanSendPdoInfo.CanSendInfo3.u16PropCur1 = inserted_data[0] * PROP_CURRENT_FACOTR * 1000;
			gCanSendPdoInfo.CanSendInfo3.u16PropCur2 = inserted_data[1] * PROP_CURRENT_FACOTR * 1000;
			__enable_irq();
			
			
		}		
		//vCanRevPdoProc();
		u8ErrCode = u8ErrCodeGet();
		if (0 != u8ErrCode)
		{
			//gCanSendPdoInfo.CanSend23CInfo.u8ErrCode = u8ErrCode;
			vLedSendAlmCode(u8ErrCode);
			if (u8ErrCode <= 50)
			{
				gCanSendPdoInfo.CanSendInfo1.u8McuErrCode = u8ErrCode;
				gCanSendPdoInfo.CanSendInfo1.u8EcuErrCode = 0;
			}
			else
			{
				gCanSendPdoInfo.CanSendInfo1.u8McuErrCode = 0;
				gCanSendPdoInfo.CanSendInfo1.u8EcuErrCode = u8ErrCode;
			}
		}
		else
		{
				gCanSendPdoInfo.CanSendInfo1.u8McuErrCode = 0;
				gCanSendPdoInfo.CanSendInfo1.u8EcuErrCode = 0;
		}
	}
	
	vWdgSetFun(WDG_USER_BIT);
}
#endif
