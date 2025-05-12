/*******************************************************************************
* Filename: Device.h 	                                    	     		   *
*                                                                              *
* Description: The header file of Device.c.							           *
* Author:                                                                      *
* Date: 															           *
* Revision:															           *
*																	           *
*******************************************************************************/
#ifndef _DEVICE_H_
#define _DEVICE_H_

#include "KSDsys.h"
#include "main.h"
#include "gd32f30x.h"
#include "CanBaudRateSync.h"

#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET

#define USART_IDLE		0
#define USART_BUSY		1

#define UART0_SEND_BUF_LEN		64
#define UART0_REV_BUF_LEN		64

#define UART1_SEND_BUF_LEN		16
#define UART1_REV_BUF_LEN		16

#define	UART2_SEND_BUF_LEN		256

typedef enum
{
	Uart0 = 0,
	Uart1 = 1,
	Uart2 = 2
}eUart;



/*lilu 20231108 CanBaudSync*/
//#define	CANBAUD_SYNC_MST_Pin	GPIO_PIN_10
//#define CANBAUD_SYNC_MST_Port 	GPIOD
//#define CANBAUD_SYNC_SW1_Pin	GPIO_PIN_5
//#define CANBAUD_SYNC_SW1_Port 	GPIOD
//#define CANBAUD_SYNC_SW2_Pin	GPIO_PIN_6
//#define CANBAUD_SYNC_SW2_Port 	GPIOD
//#define	CANBAUD_SYNC_SLV_Pin	GPIO_PIN_11
//#define CANBAUD_SYNC_SLV_Port 	GPIOD

//#define CANBAUD_SYNC_MST(Level)	gpio_bit_write(CANBAUD_SYNC_MST_Port, CANBAUD_SYNC_MST_Pin, 0 != Level ? GPIO_PIN_SET : GPIO_PIN_RESET)
//#define CANBAUD_SYNC_SW1(Level)	gpio_bit_write(CANBAUD_SYNC_SW1_Port, CANBAUD_SYNC_SW1_Pin, 0 != Level ? GPIO_PIN_SET : GPIO_PIN_RESET)
//#define CANBAUD_SYNC_SW2(Level)	gpio_bit_write(CANBAUD_SYNC_SW2_Port, CANBAUD_SYNC_SW2_Pin, 0 != Level ? GPIO_PIN_SET : GPIO_PIN_RESET)
//#define	CANBAUD_SYNC_SLV_Read()	gpio_input_bit_get(CANBAUD_SYNC_SLV_Port, CANBAUD_SYNC_SLV_Pin)

//#define	CANBAUD_SYNC_Init()																				\
//	do{																									\
//		rcu_periph_clock_enable(RCU_GPIOD); 															\
//		gpio_init(CANBAUD_SYNC_MST_Port, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, CANBAUD_SYNC_MST_Pin);	\
//		gpio_init(CANBAUD_SYNC_SW1_Port, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, CANBAUD_SYNC_SW1_Pin);	\
//		gpio_init(CANBAUD_SYNC_SW2_Port, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, CANBAUD_SYNC_SW2_Pin);	\
//		gpio_init(CANBAUD_SYNC_SLV_Port, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, CANBAUD_SYNC_SLV_Pin);	\
//		CANBAUD_SYNC_MST(1);																			\
//		CANBAUD_SYNC_SW1(1);																			\
//		CANBAUD_SYNC_SW2(1);																			\
//	}while(0)


#if (CTLBOARD_TYPE == _HB4_GD32)
/************ DO define *****************/

#define LED_R_GPIO_Port 			GPIOE 
#define LED_R_Pin 					GPIO_PIN_2
#define LED_Y_GPIO_Port 			GPIOE 
#define LED_Y_Pin 					GPIO_PIN_3
#define SPI_CS_GPIO_Port			GPIOD
#define SPI_CS_Pin 					GPIO_PIN_0
#define ANALOG_1_EN 				GPIOB
#define ANALOG_1_EN_Pin 			GPIO_PIN_12
#define ANALOG_2_EN					GPIOB
#define ANALOG_2_EN_Pin 			GPIO_PIN_13
#define ANALOG_3_EN					GPIOB
#define ANALOG_3_EN_Pin 			GPIO_PIN_14
#define DIVER_EN					GPIOE
#define DIVER_EN_Pin 				GPIO_PIN_15
#define ENCODER_POWER_1				GPIOE
#define ENCODER_POWER_1_Pin			GPIO_PIN_10
#define ENCODER_POWER_2				GPIOE
#define ENCODER_POWER_2_Pin			GPIO_PIN_12
#define WATCH_DOG					GPIOD
#define WATCH_DOG_Pin				GPIO_PIN_10
/************ DI define *****************/ 
#define SWI1_GPIO_Port				GPIOE
#define SWI1_Pin					GPIO_PIN_4
#define SWI2_GPIO_Port				GPIOE
#define SWI2_Pin					GPIO_PIN_5
#define SWI3_GPIO_Port				GPIOE
#define SWI3_Pin					GPIO_PIN_6
#define SWI4_GPIO_Port				GPIOC
#define SWI4_Pin					GPIO_PIN_13
#define SWI5_GPIO_Port				GPIOC
#define SWI5_Pin					GPIO_PIN_14
#define SWI6_GPIO_Port				GPIOC
#define SWI6_Pin					GPIO_PIN_15
#define SWI7_GPIO_Port				GPIOE
#define SWI7_Pin					GPIO_PIN_7
#define SWI8_GPIO_Port				GPIOE
#define SWI8_Pin					GPIO_PIN_8
#define DRIVER1_R_GPIO_Port			GPIOE
#define DRIVER1_R_Pin				GPIO_PIN_1
#define DRIVER2_R_GPIO_Port			GPIOE
#define DRIVER2_R_Pin				GPIO_PIN_0
#define DRIVER3_R_GPIO_Port			GPIOB
#define DRIVER3_R_Pin				GPIO_PIN_8
#define DRIVER4_R_GPIO_Port			GPIOD
#define DRIVER4_R_Pin				GPIO_PIN_1
#define DRIVER5_R_GPIO_Port			GPIOA
#define DRIVER5_R_Pin				GPIO_PIN_10
#define DRIVER6_R_GPIO_Port			GPIOB
#define DRIVER6_R_Pin				GPIO_PIN_4
#define DRIVER7_R_GPIO_Port			GPIOA
#define DRIVER7_R_Pin				GPIO_PIN_8
#define DRIVER8_R_GPIO_Port			GPIOB
#define DRIVER8_R_Pin				GPIO_PIN_9
#define DRIVER9_R_GPIO_Port			GPIOD
#define DRIVER9_R_Pin				GPIO_PIN_4
#define DRIVER10_R_GPIO_Port		GPIOA
#define DRIVER10_R_Pin				GPIO_PIN_9
#define DRIVER11_R_GPIO_Port		GPIOD
#define DRIVER11_R_Pin				GPIO_PIN_3
#define DRIVER12_R_GPIO_Port		GPIOD
#define DRIVER12_R_Pin				GPIO_PIN_2
/*lilu 20230725*/
#define WATCH_DOG_R_GPIO_Port		GPIOD
#define WATCH_DOG_R_Pin				GPIO_PIN_11
#endif //#if (CTLBOARD_TYPE == _HB4_GD32)


#if (CTLBOARD_TYPE == _HB6_GD32)
/************ DO define *****************/

#define LED_R_GPIO_Port 			GPIOE 
#define LED_R_Pin 					GPIO_PIN_2
#define LED_Y_GPIO_Port 			GPIOE 
#define LED_Y_Pin 					GPIO_PIN_3
#define SPI_CS_GPIO_Port			GPIOD
#define SPI_CS_Pin 					GPIO_PIN_0
#define ANALOG_1_EN 				GPIOB
#define ANALOG_1_EN_Pin 			GPIO_PIN_12
#define ANALOG_2_EN					GPIOB
#define ANALOG_2_EN_Pin 			GPIO_PIN_13
#define ANALOG_3_EN					GPIOB
#define ANALOG_3_EN_Pin 			GPIO_PIN_14
#define DIVER_EN					GPIOE
#define DIVER_EN_Pin 				GPIO_PIN_15
#define ENCODER_POWER_1				GPIOE
#define ENCODER_POWER_1_Pin			GPIO_PIN_10
#define ENCODER_POWER_2				0//GPIOE
#define ENCODER_POWER_2_Pin			0//GPIO_PIN_12
#define WATCH_DOG					GPIOD
#define WATCH_DOG_Pin				GPIO_PIN_10
/************ DI define *****************/ 
#define SWI1_GPIO_Port				GPIOE
#define SWI1_Pin					GPIO_PIN_4
#define SWI2_GPIO_Port				GPIOE
#define SWI2_Pin					GPIO_PIN_5
#define SWI3_GPIO_Port				GPIOE
#define SWI3_Pin					GPIO_PIN_6
#define SWI4_GPIO_Port				GPIOC
#define SWI4_Pin					GPIO_PIN_13
#define SWI5_GPIO_Port				GPIOC
#define SWI5_Pin					GPIO_PIN_14
#define SWI6_GPIO_Port				GPIOC
#define SWI6_Pin					GPIO_PIN_15
#define SWI7_GPIO_Port				GPIOE
#define SWI7_Pin					GPIO_PIN_7
#define SWI8_GPIO_Port				GPIOE
#define SWI8_Pin					GPIO_PIN_8
#define SWI9_GPIO_Port				GPIOC
#define SWI9_Pin					GPIO_PIN_2
#define SWI10_GPIO_Port				GPIOC
#define SWI10_Pin					GPIO_PIN_3
#define SWI11_GPIO_Port				GPIOE
#define SWI11_Pin					GPIO_PIN_13
#define SWI12_GPIO_Port				GPIOE
#define SWI12_Pin					GPIO_PIN_14
#define DRIVER1_R_GPIO_Port			GPIOE
#define DRIVER1_R_Pin				GPIO_PIN_1
#define DRIVER2_R_GPIO_Port			GPIOE
#define DRIVER2_R_Pin				GPIO_PIN_0
#define DRIVER3_R_GPIO_Port			GPIOB
#define DRIVER3_R_Pin				GPIO_PIN_8
#define DRIVER4_R_GPIO_Port			GPIOD
#define DRIVER4_R_Pin				GPIO_PIN_1
#define DRIVER5_R_GPIO_Port			GPIOA
#define DRIVER5_R_Pin				GPIO_PIN_10
#define DRIVER6_R_GPIO_Port			GPIOB
#define DRIVER6_R_Pin				GPIO_PIN_4
#define DRIVER7_R_GPIO_Port			GPIOA
#define DRIVER7_R_Pin				GPIO_PIN_8
#define DRIVER8_R_GPIO_Port			GPIOB
#define DRIVER8_R_Pin				GPIO_PIN_9
#define DRIVER9_R_GPIO_Port			GPIOD
#define DRIVER9_R_Pin				GPIO_PIN_4
#define DRIVER10_R_GPIO_Port		GPIOA
#define DRIVER10_R_Pin				GPIO_PIN_9
#define DRIVER11_R_GPIO_Port		GPIOD
#define DRIVER11_R_Pin				GPIO_PIN_3
#define DRIVER12_R_GPIO_Port		GPIOD
#define DRIVER12_R_Pin				GPIO_PIN_2
/*lilu 20230725*/
#define WATCH_DOG_R_GPIO_Port		GPIOD
#define WATCH_DOG_R_Pin				GPIO_PIN_11
#endif //#if (CTLBOARD_TYPE == _HB6_GD32)

/************ DO *****************/

#define LED_Y_ON()     	 		gpio_bit_write(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET)
#define LED_Y_OFF()				gpio_bit_write(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_RESET)
#define LED_R_ON()     	 		gpio_bit_write(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET)
#define LED_R_OFF()       		gpio_bit_write(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET)
#define SPI_CS_0       			gpio_bit_write(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET)
#define SPI_CS_1     	 		gpio_bit_write(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET)
#define DO_RESET_ON()     	 	gpio_bit_write(DIVER_EN, DIVER_EN_Pin, GPIO_PIN_SET)
#define DO_RESET_OFF()    	 	gpio_bit_write(DIVER_EN, DIVER_EN_Pin, GPIO_PIN_RESET)

#define D0_ANALOG_1_ON()			gpio_bit_write(ANALOG_1_EN, ANALOG_1_EN_Pin, GPIO_PIN_SET)
#define D0_ANALOG_1_OFF()			gpio_bit_write(ANALOG_1_EN, ANALOG_1_EN_Pin, GPIO_PIN_RESET)
#define D0_ANALOG_2_ON()			gpio_bit_write(ANALOG_2_EN, ANALOG_2_EN_Pin, GPIO_PIN_SET)
#define D0_ANALOG_2_OFF()			gpio_bit_write(ANALOG_2_EN, ANALOG_2_EN_Pin, GPIO_PIN_RESET)
#define D0_ANALOG_3_ON()			gpio_bit_write(ANALOG_3_EN, ANALOG_3_EN_Pin, GPIO_PIN_SET)
#define D0_ANALOG_3_OFF()			gpio_bit_write(ANALOG_3_EN, ANALOG_3_EN_Pin, GPIO_PIN_RESET)
#define DO_ENCODER1_POWER_5V()		gpio_bit_write(ENCODER_POWER_1, ENCODER_POWER_1_Pin, GPIO_PIN_RESET)
#define DO_ENCODER1_POWER_12V()		gpio_bit_write(ENCODER_POWER_1, ENCODER_POWER_1_Pin, GPIO_PIN_SET)
#define DO_ENCODER2_POWER_5V()		gpio_bit_write(ENCODER_POWER_2, ENCODER_POWER_2_Pin, GPIO_PIN_RESET)
#define DO_ENCODER2_POWER_12V()		gpio_bit_write(ENCODER_POWER_2, ENCODER_POWER_2_Pin, GPIO_PIN_SET)

#define	DO_MST_WDG_ON()				gpio_bit_write(WATCH_DOG, WATCH_DOG_Pin, GPIO_PIN_SET)
#define	DO_MST_WDG_OFF()			gpio_bit_write(WATCH_DOG, WATCH_DOG_Pin, GPIO_PIN_RESET)

///************ DI *****************/ 
#define    READ_SW1() 	gpio_input_bit_get(SWI1_GPIO_Port, SWI1_Pin)
#define    READ_SW2() 	gpio_input_bit_get(SWI2_GPIO_Port, SWI2_Pin)
#define    READ_SW3() 	gpio_input_bit_get(SWI3_GPIO_Port, SWI3_Pin)
#define    READ_SW4() 	gpio_input_bit_get(SWI4_GPIO_Port, SWI4_Pin)
#define    READ_SW5() 	gpio_input_bit_get(SWI5_GPIO_Port, SWI5_Pin)
#define    READ_SW6() 	gpio_input_bit_get(SWI6_GPIO_Port, SWI6_Pin)
#define    READ_SW7() 	gpio_input_bit_get(SWI7_GPIO_Port, SWI7_Pin)
#define    READ_SW8() 	gpio_input_bit_get(SWI8_GPIO_Port, SWI8_Pin)
#define    READ_SW9() 	gpio_input_bit_get(SWI9_GPIO_Port, SWI9_Pin)
#define    READ_SW10() 	gpio_input_bit_get(SWI10_GPIO_Port, SWI10_Pin)
#define    READ_SW11() 	gpio_input_bit_get(SWI11_GPIO_Port, SWI11_Pin)
#define    READ_SW12() 	gpio_input_bit_get(SWI12_GPIO_Port, SWI12_Pin)

#define    READ_DRV1() 	gpio_input_bit_get(DRIVER1_R_GPIO_Port, DRIVER1_R_Pin)
#define    READ_DRV2() 	gpio_input_bit_get(DRIVER2_R_GPIO_Port, DRIVER2_R_Pin)
#define    READ_DRV3() 	gpio_input_bit_get(DRIVER3_R_GPIO_Port, DRIVER3_R_Pin)
#define    READ_DRV4() 	gpio_input_bit_get(DRIVER4_R_GPIO_Port, DRIVER4_R_Pin)
#define    READ_DRV5() 	gpio_input_bit_get(DRIVER5_R_GPIO_Port, DRIVER5_R_Pin)
#define    READ_DRV6() 	gpio_input_bit_get(DRIVER6_R_GPIO_Port, DRIVER6_R_Pin)
#define    READ_DRV7() 	gpio_input_bit_get(DRIVER7_R_GPIO_Port, DRIVER7_R_Pin)
#define    READ_DRV8() 	gpio_input_bit_get(DRIVER8_R_GPIO_Port, DRIVER8_R_Pin)
#define    READ_DRV9() 	gpio_input_bit_get(DRIVER9_R_GPIO_Port, DRIVER9_R_Pin)
#define    READ_DRV10() gpio_input_bit_get(DRIVER10_R_GPIO_Port, DRIVER10_R_Pin)
#define    READ_DRV11() gpio_input_bit_get(DRIVER11_R_GPIO_Port, DRIVER11_R_Pin)
#define    READ_DRV12() gpio_input_bit_get(DRIVER12_R_GPIO_Port, DRIVER12_R_Pin)
#define    READ_WDG() 	gpio_input_bit_get(WATCH_DOG_R_GPIO_Port, WATCH_DOG_R_Pin)

///************ AI *****************/ 
#define   ADC0_NUM		12  //ADC0 12规则通道
/********************** POWER BOARD PROTECTIONS THRESHOLDS ********************/

#define NTC_THRESHOLD_C    (INT16U)80 //on heatsink of MB459 board
#define NTC_HYSTERIS_C     (INT16U)5   // Temperature hysteresis

#define OVERVOLTAGE_THRESHOLD_V   28  //Volt on DC Bus 
#define UNDERVOLTAGE_THRESHOLD_V  16  //Volt on DC Bus
#define VOLTAGELIMIT_THRESHOLD_V  32  //Bus Volt limit 
#define RELAYON_THRESHOLD_V  			18  //Volt on DC Bus 
#define BUS_ADC_CONV_RATIO 			  0.045 /* 6.8K/(143K+6.8K)*/
#define POWER5V_THRESHOLD_V				4
#define POWER5V_CONV_RATIO 			  0.5 /*23.7/(41.2+23.7)*/
#define POWER12V_CONV_RATIO 			0.167 /*20/(20+100)*/
#define ANALOG_ADC_CONV_RATIO 		0.547 //0.547 /* 49.9/(49.9+41.2)*/


//#ifdef BAUDRATE_SYCHRON
//#define	CAN_BAUDRATE_ADDRESS		((uint32_t)0x08003800U)
//#endif
//#define	CAN_BAUD_RATE_125K			CAN_BAUD_RATE_125K
//#define	CAN_BAUD_RATE_250K			250
//#define	CAN_BAUD_RATE_500K			500

/******************************************************************************
*�������Ͷ���
******************************************************************************/
#define PWM_PRSC ((INT8U)0)
#define CKTIM	((INT32U)120000000uL)
#define PWM_FREQ ((INT16U) 16000)//((u16) 12500) //Power devices switching frequency in Hz
//#define PWM_FREQ ((INT16U) 8000)//((u16) 12500) //Power devices switching frequency in Hz
#define DEADTIME_NS	((INT16U)800)  //Deadtime Valuein nsec; range is [0...3500] 
#define PWM_PERIOD ((INT16U) (CKTIM / (INT32U)( PWM_FREQ *(PWM_PRSC+1)))) /* Resolution: 1Hz */    //4800         

#define	PWM_PLUS_MAX_VALUE	(PWM_PERIOD * 99 / 100)
#define DEADTIME  (INT16U)((unsigned long long)CKTIM/2 \
          *(unsigned long long)DEADTIME_NS/1000000000uL) // Deadtime Value 
//�ò������Ե�����������ˢ��Ƶ�� SAMPLING_FREQ
/****	ADC IRQ-HANDLER frequency, related to PWM  ****/
#define REP_RATE (1)  // (N.b): Internal current loop is performed every 
                      //             (REP_RATE + 1)/(2*PWM_FREQ) seconds.
                      // REP_RATE has to be an odd number in case of three-shunt
                      // current reading; this limitation doesn't apply to ICS

#define PROP_PWM_PERIOD  PWM_PERIOD//(PWM_PERIOD/2)
		  
#define	PROP_DITHER_PERIOD_FACOTR		16
#define	PROP_DITHER_RATIO_FACTOR		328
                      
/* Define the structure of the PWM generator Object */
typedef struct PWM_GEN
{
 	INT16U PeriodMax;	/* Parameter: PWM Half-Period in CPU clock cycles(Q0)*/
	INT16U MfuncPeriod;	/* Input: Period scaler (Q15) */
	INT16S MfuncC;		/* Input: PWM 3&4 Duty cycle ratio (Q15) */
	uint16_t u16DitherPeriod;
	uint16_t u16DitherCnt;
	uint16_t u16DitherRatio;
	int16_t i16DitherValue;
	void (*Update)(struct PWM_GEN *, uint8_t u8Channel);	/* Pointer to the update function */
} PWM_GEN;

//extern EXTI_HandleTypeDef hexti_PLC;
extern int32_t ADCValue[ADC0_NUM];
extern uint16_t inserted_data[4];
/******************************************************************************
*��������
******************************************************************************/
uint8_t u8GetCanSendState(uint32_t can_periph);


extern void vBspInit(void);
extern void vDrvDoInit(void);
extern void vDrvDiInit(void);
extern void vDrvAdInit(void);
extern void vDrvSpiInit(void);
extern void vDrvPwmInit(void);
extern void vDrvUartInit(void);
extern void vDrvCanInit(uint16_t u16BaudRate);
extern void vDrvwdgInit();

extern uint8_t u8GetCanSendState(uint32_t can_periph);

extern void vSetUartSendState(eUart UartNo, uint8_t u8State);
extern uint8_t u8GetUartSendState(eUart UartNo);
extern void vDrvUart0Send(uint8_t *u8SendBuf, uint16_t u16SendLen);
extern void vDrvUart1Send(uint8_t *u8SendBuf, uint16_t u16SendLen);
extern void vDrvUart2Send(uint8_t *u8SendBuf, uint16_t u16SendLen);
extern uint8_t* u8pGetRevBuf(eUart UartNo);



extern void InitGPIO(void);
extern void InitSysCtrl(void);
extern void InitDone(void);
extern void CPULoadStat(void);
extern void PwmEnable(void);
extern void PwmEnablePwOn(void);
extern void PwmDisable(void);
extern void PwmDisableEsp(void);
extern uint8_t* u8pGetRevBuf(eUart UartNo);

void PwmUpdate(PWM_GEN *p);
void PWMDriver(INT16U pulse[]);
//extern void STM32F4PwmUpdate(PWM_GEN *p);
extern void vPwmDriver(uint8_t u8Channel, uint16_t u16PluseWidth);
extern void PropDriverPwmUpdate(PWM_GEN *p, uint8_t u8Channel);
//extern INT16U EepromWrite(INT16U address, INT16U data);
//extern INT16U EepromRead(INT16U address, INT16U* pdata);
extern void Delay(unsigned long ulTime);
//extern void delay_us(INT32U nus);
extern void CAN_Config(void);
//INT32U TIM_GetCounter(void);

//extern 	tBoolean EepromBufRead(unsigned short address, unsigned short NByte,unsigned short *pRcvBuffer);
//extern 	tBoolean EepromBufWrite(unsigned short address,unsigned short NByte,unsigned short *pSndBuffer);
#endif //_DEVICE_H_
