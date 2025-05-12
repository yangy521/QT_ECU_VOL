/*******************************************************************************
* Filename: WatchDog.c 	                                    	     	       *
* Description: 	看门狗处理						           				       *
* Author:  QExpand, Lilu                                                       *
* Date: 2023/5/15															   *
* Revision:	V1.00														       *
*******************************************************************************/
#include "gd32f30x.h"

/*******************************************************************************
* Name: void vFeedDog(void)
* Descriptio: feed watch dog
* Input:  NULL
* Output: NULL 
*******************************************************************************/
void vFeedDog(void)
{
	fwdgt_counter_reload();
}