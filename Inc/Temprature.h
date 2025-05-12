/*******************************************************************************
* Filename: Temprature.h	                                             	 		   *
* Description:											   			 		   *
* Author:                                                           		   *
* Date:     														 		   *
* Revision:															 		   *
*******************************************************************************/

#include		"KSDsys.h"


#define TMP_INDEX0		  -40
#define TMP_POWER_STEP		1
#define TMP_DRIVE_STEP		1
#define TMP_MOTOR_STEP		1

#define TMP_DEFAULT			20
#define TMP_POWERCUT		85
#define TMP_POWERMAX		95
#define TMP_POWERMIN		-30

/*******************************************************************************
* local function declaration 
*******************************************************************************/
//±äÁ¿ÉùÃ÷
void TmpInit(void);
void TmpProcess(void);

