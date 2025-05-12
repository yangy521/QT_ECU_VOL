/*******************************************************************************
* Filename: NCComm.c                                             	 	       *
*                                                                    		   *
* Description:											   			 		   *
* Author:                                                           		   *
* Date:     														 		   *
* Revision:															 		   *
*																	 		   *
*******************************************************************************/
#include 		"stm32f4xx.h"
#include 		"CommonRam.h"
#include 		"Position.h"
#include    "Kernel.h"
#include    "NCComm.h"

NC_COMM			gNcComm;

/*******************************************************************************
* Name: NcCommPulseDirInit
* Description:
* Input: 
* Output: 
*
* Author:
* Date:
* Revision:
*******************************************************************************/
void NcCommPulseDirInit(NC_COMM *p)
{
	p->PosCmd = 0;
	p->PosFdb = 0;
	p->PosRef = 0;
	p->PosErr = 0;
	p->PosOld1 = 0;
	p->Increase1 = 0;
	p->PosOld2 = 0;
	p->Increase2 = 0;

}

/*******************************************************************************
* Name: NcCommPulseDirCalc
* Description:
* Input: 
* Output: 
*
* Author:
* Date:
* Revision:
*******************************************************************************/
void NcCommPulseDirCalc(NC_COMM *p)
{
}

/*******************************************************************************
* Name: InitNcComm
* Description:
* Input: 
* Output: 
*
* Author:
* Date:
* Revision:
*******************************************************************************/
void InitNcComm(void)
{
	/* pulse dir interface */
	gNcComm.Init = NcCommPulseDirInit;
	gNcComm.Calc = NcCommPulseDirCalc;
	gNcComm.Init(&gNcComm);
}

