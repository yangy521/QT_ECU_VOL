/*******************************************************************************
* Filename: Protect.h 	                                    	     		   *
*                                                                              *
* Description: The header file of Protect.c.							           *
* Author:                                                                      *
* Date: 															           *
* Revision:															           *
*																	           *
*******************************************************************************/
#ifndef __PROTECT_H
#define __PROTECT_H

#include "KSDsys.h"
#include "gd32f30x.h"
/******************************************************************************
*数据类型定义
******************************************************************************/
typedef struct
{
  uint32_t UniqueDeviceID[3];    /*!< unique device identifier,96bit,                  Address offset: Unique_Device_Identifier_Base base address + 0x0 */
} U_ID_TypeDef;
typedef struct APPPROG_VER_TypeDef
{
	U_ID_TypeDef		AppId;
	uint16_t	AppCanID;
	uint16_t	BootCanID;
	uint16_t	DriverType;
	uint16_t	ContorBoardType;
	uint16_t	SoftVer;
	uint16_t	User;
	char      date[12];
	uint16_t  rev[28];
} APPPROG_VER_TypeDef;

extern const APPPROG_VER_TypeDef  AppProgramVerConst;
#define Unique_Device_Identifier_Base  0x1FFF7A10
#define U_ID               ((U_ID_TypeDef *) Unique_Device_Identifier_Base)
/******************************************************************************
*函数定义
******************************************************************************/

void SetId(const U_ID_TypeDef* pId);
uint32_t CheckId(const U_ID_TypeDef* pId);
uint32_t CheckIdEmpty(const U_ID_TypeDef* pId);

#endif //__PROTECT_H

