/******************** (C) COPYRIGHT  XI`AN QEXPAND *******************************
* �ļ���  ��DrvDo.c
* ����    ��DO�����ļ�       
* ʵ��ƽ̨��ECU
* ��汾  ��V1.0.0
* ����    ��
* �޸�ʱ��: 2023-05-16
**********************************************************************************/	
#include "Device.h"

void vDrvDoInit(void)
{
	/*******************************************************************************
	* DO
	*�ܽ�1	 PE2	LED_R;   			 	�ܽ�2	 PE3	LED_Y;   	
	
	�ܽ�38	 PE15	Driver_EN;       		�ܽ�41    PE10    5V/12V OUT1-EN
	
	�ܽ�43	 PE12	5V/12V OUT2-EN			 �ܽ�51    PB12  AI1 EN

	�ܽ�52	 PB13	AI2 EN					�ܽ� 54  PB14	AI3 EN
	
	*******************************************************************************/
	gpio_init(LED_R_GPIO_Port, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LED_R_Pin);
    gpio_init(LED_Y_GPIO_Port, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LED_Y_Pin);
    gpio_init(DIVER_EN, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, DIVER_EN_Pin);
	gpio_init(ENCODER_POWER_1, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, ENCODER_POWER_1_Pin);	/*lilu 20230713 add IO initial*/
	gpio_init(ENCODER_POWER_2, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, ENCODER_POWER_2_Pin);
    gpio_init(SPI_CS_GPIO_Port, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, SPI_CS_Pin);
    gpio_init(ANALOG_1_EN, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, ANALOG_1_EN_Pin);
    gpio_init(ANALOG_2_EN, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, ANALOG_2_EN_Pin);
    gpio_init(ANALOG_3_EN, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, ANALOG_3_EN_Pin);
	
	gpio_init(WATCH_DOG, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, WATCH_DOG_Pin);	/*lilu 20230725 add WatchDog*/
}

