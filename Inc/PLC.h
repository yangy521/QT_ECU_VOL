/*******************************************************************************
* Filename: PLC.h 	                                    	     		   *
*                                                                              *
* Description: The header file of Device.c.							           *
* Author:                                                                      *
* Date: 															           *
* Revision:															           *
*																	           *
*******************************************************************************/

#include "KSDsys.h"


/*******************************************************************************
* 1. ָʾ��
*******************************************************************************/
//״ָ̬ʾ��
#define 	LED_YELLOW_SET						(1 << 0)
#define 	LED_YELLOW_RESET					(1 << 1)
#define 	LED_YELLOW_BLINK					(1 << 2)
#define		LED_RED_SET								(1 << 8)
#define		LED_RED_RESET							(1 << 9)
#define		LED_RED_BLINK							(1 << 10)

/*******************************************************************************
* 2. �豸���� 
*******************************************************************************/
//#define		Prd_Move					_CONTROLLER_MOVE	//��ǣ��������
//#define		Prd_Lift					_CONTROLLER_LIFT	//����������
//#define		Prd_Steer					_CONTROLLER_STEER	//ת�������
//#define		Prd_Move_Second		_CONTROLLER_MOVE_SECOND	//��ǣ��������
//#define		Prd_Logic					_CONTROLLER_LOGIC	//�߼�������
//#define		Prd_HMI 					_CONTROLLER_HMI	//�Ǳ�
//#define		Prd_PC						_CONTROLLER_PC	//PC��

/******************************************************************************
*��������
******************************************************************************/
/* Functions in PLC.c */
extern void	InitPLCCtl(void);
extern void PLCLogic(void);
extern void PLCDataSwap(void);
extern void PLCErrorTips(INT16U ErrorNo);
extern void PLCErrorClr(INT16U ErrorNo);
void LEDInit(void);


