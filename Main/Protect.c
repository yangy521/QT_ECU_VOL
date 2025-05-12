/*******************************************************************************
* Filename: Protect.c                                                  	 	   *
*                                                                    		   *
* Description: 							            				 		   *
* Author:                                                                 	   *
* Date: 20161128														           *
* Revision:															 		   *
*																	 		   												*
*******************************************************************************/

#include "KSDsys.h"
#include "Protect.h"


const APPPROG_VER_TypeDef  AppProgramVerConst =
{
	{0xffffffff,0xffffffff,0xffffffff},
	LOGIC_TYPE,
	(LOGIC_TYPE + BOOT_INC),
	DRIVER_TYPE,
	CTLBOARD_TYPE,
	SOFTVERSION,
	USER_TYPE,
	__DATE__,
	{
		0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,
		0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,
		0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,
		0xFFFF,0xFFFF,0xFFFF,0xFFFF,
	}
};
void SetId(const U_ID_TypeDef* pId)
{
//		FLASH_Unlock();
//		FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
//										FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
//		/*  */
//		FLASH_ProgramWord((uint32_t)&pId->UniqueDeviceID[0],pId->UniqueDeviceID[0] & U_ID->UniqueDeviceID[0]);
//		FLASH_ProgramWord((uint32_t)&pId->UniqueDeviceID[1],pId->UniqueDeviceID[1] & U_ID->UniqueDeviceID[1]);
//		FLASH_ProgramWord((uint32_t)&pId->UniqueDeviceID[2],pId->UniqueDeviceID[2] & U_ID->UniqueDeviceID[2]);
//		FLASH_Lock();
}

uint32_t CheckId(const U_ID_TypeDef* pId)
{
	return (  (pId->UniqueDeviceID[0] == U_ID->UniqueDeviceID[0])
					&&(pId->UniqueDeviceID[1] == U_ID->UniqueDeviceID[1])
					&&(pId->UniqueDeviceID[2] == U_ID->UniqueDeviceID[2]));
}
uint32_t CheckIdEmpty(const U_ID_TypeDef* pId)
{
	return ((pId->UniqueDeviceID[0] == 0xFFFFFFFF)
					|| (pId->UniqueDeviceID[1] == 0xFFFFFFFF)
					|| (pId->UniqueDeviceID[2] == 0xFFFFFFFF));
}
