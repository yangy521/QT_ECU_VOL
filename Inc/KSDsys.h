/*******************************************************************************
* Filename: KSDsys.h 	                                       		               *
* Description: ϵͳ����		           				                                   *
* Author:                                                                      *
* Date:     														                                       *
* Revision:															                                       *
*******************************************************************************/
#ifndef _KSDSYS_H_
#define _KSDSYS_H_

#include  "KTypedef.h"
#include	"IQmathLib.h"
#include "Userdef.h"


#if(DRIVER_TYPE	==	_2425_HB4_GD32)
	#define CTLBOARD_TYPE		_HB4_GD32
	#define VOLTAGE_LEVEL		_VOLTAGE_24V
	#define RATE_1H_CURRENT				120				/* 1Сʱ�����Ƶ���  unit A */
	#define RATE_2M_CURRENT				250				/* 2���ӹ����Ƶ���   unit A */
	#define PRODUCT_TYPE          2425
	#define ESP_OVER_CUREENT			(RATE_2M_CURRENT*1.5)		/* ���������뼱ͣ�����ĵ��� , A*/									
	#define STD_CURRENT						1260.0//1668.0				/* 1236 ������· 1= 1668A */
	#define SOFTVERSION						((SOFTVERSION_KENERL << 8) | SOFTVERSION_LOGIC)
	#define HARDVERSION1					0x0203			/* ���ֽڿ��ư塢���ֽڹ��ʰ� */
	#define HARDVERSION2					0x0103			/* ���ֽ������塢���ֽڵ��ݰ�/������*/
	#define FRAMEVERSION					0x0111			/* BIT0~3��ǡ�BIT4~7�װ塢BIT8~11�ṹ��*/
#endif

#if(DRIVER_TYPE	==	_4825_HB4_GD32)
	#define CTLBOARD_TYPE		_HB4_GD32
	#define VOLTAGE_LEVEL		_VOLTAGE_48V
	#define RATE_1H_CURRENT				120				/* 1Сʱ�����Ƶ���  unit A */
	#define RATE_2M_CURRENT				250				/* 2���ӹ����Ƶ���   unit A */
	#define PRODUCT_TYPE          2425
	#define ESP_OVER_CUREENT			(RATE_2M_CURRENT*1.5)		/* ���������뼱ͣ�����ĵ��� , A*/									
	#define STD_CURRENT						1260.0//1668.0				/* 1236 ������· 1= 1668A */
	#define SOFTVERSION						((SOFTVERSION_KENERL << 8) | SOFTVERSION_LOGIC)
	#define HARDVERSION1					0x0203			/* ���ֽڿ��ư塢���ֽڹ��ʰ� */
	#define HARDVERSION2					0x0103			/* ���ֽ������塢���ֽڵ��ݰ�/������*/
	#define FRAMEVERSION					0x0111			/* BIT0~3��ǡ�BIT4~7�װ塢BIT8~11�ṹ��*/
#endif

#if(DRIVER_TYPE	==	_2425_HD4_GD32)
	#define CTLBOARD_TYPE		_HD4_GD32 
	#define VOLTAGE_LEVEL		_VOLTAGE_24V
	#define RATE_1H_CURRENT				120				/* 1Сʱ�����Ƶ���  unit A */
	#define RATE_2M_CURRENT				250				/* 2���ӹ����Ƶ���   unit A */
	#define PRODUCT_TYPE          2425
	#define ESP_OVER_CUREENT			(RATE_2M_CURRENT*1.5)		/* ���������뼱ͣ�����ĵ��� , A*/									
	#define STD_CURRENT						1260.0//1668.0				/* 1236 ������· 1= 1668A */
	#define SOFTVERSION						((SOFTVERSION_KENERL << 8) | SOFTVERSION_LOGIC)
	#define HARDVERSION1					0x0203			/* ���ֽڿ��ư塢���ֽڹ��ʰ� */
	#define HARDVERSION2					0x0103			/* ���ֽ������塢���ֽڵ��ݰ�/������*/
	#define FRAMEVERSION					0x0111			/* BIT0~3��ǡ�BIT4~7�װ塢BIT8~11�ṹ��*/
#endif

#if(DRIVER_TYPE	==	_4825_HD4_GD32)
	#define CTLBOARD_TYPE		_HD4_GD32
	#define VOLTAGE_LEVEL		_VOLTAGE_48V
	#define RATE_1H_CURRENT				120				/* 1Сʱ�����Ƶ���  unit A */
	#define RATE_2M_CURRENT				250				/* 2���ӹ����Ƶ���   unit A */
	#define PRODUCT_TYPE          4825
	#define ESP_OVER_CUREENT			(RATE_2M_CURRENT*1.5)		/* ���������뼱ͣ�����ĵ��� , A*/									
	#define STD_CURRENT						1260.0//1668.0				/* 1236 ������· 1= 1668A */
	#define SOFTVERSION						((SOFTVERSION_KENERL << 8) | SOFTVERSION_LOGIC)
	#define HARDVERSION1					0x0203			/* ���ֽڿ��ư塢���ֽڹ��ʰ� */
	#define HARDVERSION2					0x0103			/* ���ֽ������塢���ֽڵ��ݰ�/������*/
	#define FRAMEVERSION					0x0111			/* BIT0~3��ǡ�BIT4~7�װ塢BIT8~11�ṹ��*/
#endif

#if(DRIVER_TYPE	==	_4825_HB6_GD32)
	#define CTLBOARD_TYPE		_HB6_GD32
	#define VOLTAGE_LEVEL		_VOLTAGE_24V
	#define RATE_1H_CURRENT				120				/* 1Сʱ�����Ƶ���  unit A */
	#define RATE_2M_CURRENT				250				/* 2���ӹ����Ƶ���   unit A */
	#define PRODUCT_TYPE          2412
	#define ESP_OVER_CUREENT			(RATE_2M_CURRENT*1.5)		/* ���������뼱ͣ�����ĵ��� , A*/									
	#define STD_CURRENT						1260.0//1668.0				/* 1236 ������· 1= 1668A */
	#define SOFTVERSION						((SOFTVERSION_KENERL << 8) | SOFTVERSION_LOGIC)
	#define HARDVERSION1					0x0203			/* ���ֽڿ��ư塢���ֽڹ��ʰ� */
	#define HARDVERSION2					0x0103			/* ���ֽ������塢���ֽڵ��ݰ�/������*/
	#define FRAMEVERSION					0x0111			/* BIT0~3��ǡ�BIT4~7�װ塢BIT8~11�ṹ��*/
#endif

#if (VOLTAGE_LEVEL == _VOLTAGE_24V)
	#define RATE_VOLTAGE					240				/* ��ƿ��׼��ѹ unit 0.1 */
	#define MAX_VOLTAGE						350				/* ��ߵ�ѹ�������õ�ѹϵͳ������ unit 0.1 */
	#define CUT_VOLTAGE						208				/* ������ѹ�����ڸõ�ѹϵͳ�������У��������Ť�� unit 0.1  */
	#define MIN_VOLTAGE						168				/* ��͵�ѹ�����ڸõ�ѹϵͳ��������ֹͣ���� unit 0.1  */
	#define BRO_VOLTAGE						150				/* ������ѹ�����ڸõ�ѹ��ϵͳ��λ���ر�һ����� unit 0.1  */
	#define ESP_LOW_VOLTAGE				120				/* ��ͣ��ѹ�����ڸõ�ѹ����Ϊ��·���ر�PWM�����űۣ�����ϵ�ָ� unit 0.1  */
	#define STD_VOLTAGE						24.0				/* Bus voltage V */
#endif

#if (VOLTAGE_LEVEL == _VOLTAGE_48V)
	#define RATE_VOLTAGE					480				/* ��ƿ��׼��ѹ  unit 0.1 */
	#define MAX_VOLTAGE						630				/* ��ߵ�ѹ�������õ�ѹϵͳ������  unit 0.1 */
	#define CUT_VOLTAGE						418				/* ����Ƿѹ�����ڸõ�ѹϵͳ�������У��������Ť��  unit 0.1 */
	#define MIN_VOLTAGE						336				/* ��͵�ѹ�����ڸõ�ѹϵͳ��������ֹͣ����  unit 0.1 */
	#define BRO_VOLTAGE						300				/* ������ѹ�����ڸõ�ѹ��ϵͳ��λ���ر�һ�����  unit 0.1 */
	#define ESP_LOW_VOLTAGE				240				/* ��ͣ��ѹ�����ڸõ�ѹ����Ϊ��·���ر�PWM�����űۣ�����ϵ�ָ� unit 0.1  */
	#define STD_VOLTAGE						48.0				/* Bus voltage V */
#endif

#if (VOLTAGE_LEVEL == _VOLTAGE_72V)
	#define RATE_VOLTAGE					720				/* ��ƿ��׼��ѹ  unit 0.1 */
	#define MAX_VOLTAGE						920				/* ��ߵ�ѹ�������õ�ѹϵͳ������  unit 0.1 */
	#define CUT_VOLTAGE						623				/* ����Ƿѹ�����ڸõ�ѹϵͳ�������У��������Ť��  unit 0.1 */
	#define MIN_VOLTAGE						504				/* ��͵�ѹ�����ڸõ�ѹϵͳ��������ֹͣ����  unit 0.1 */
	#define BRO_VOLTAGE						450				/* ������ѹ�����ڸõ�ѹ��ϵͳ��λ���ر�һ�����  unit 0.1 */
	#define ESP_LOW_VOLTAGE				360				/* ��ͣ��ѹ�����ڸõ�ѹ����Ϊ��·���ر�PWM�����űۣ�����ϵ�ָ� unit 0.1  */
	#define STD_VOLTAGE						72.0			/* Bus voltage V */
#endif

#if (VOLTAGE_LEVEL == _VOLTAGE_80V)
	#define RATE_VOLTAGE					800				/* ��ƿ��׼��ѹ  unit 0.1 */
	#define MAX_VOLTAGE						100				/* ��ߵ�ѹ�������õ�ѹϵͳ������  unit 0.1 */
	#define CUT_VOLTAGE						692				/* ����Ƿѹ�����ڸõ�ѹϵͳ�������У��������Ť��  unit 0.1 */
	#define MIN_VOLTAGE						560				/* ��͵�ѹ�����ڸõ�ѹϵͳ��������ֹͣ����  unit 0.1 */
	#define BRO_VOLTAGE						500				/* ������ѹ�����ڸõ�ѹ��ϵͳ��λ���ر�һ�����  unit 0.1 */
	#define ESP_LOW_VOLTAGE				400				/* ��ͣ��ѹ�����ڸõ�ѹ����Ϊ��·���ر�PWM�����űۣ�����ϵ�ָ� unit 0.1  */
	#define STD_VOLTAGE						80.0			/* Bus voltage V */
#endif

#if (VOLTAGE_LEVEL == _VOLTAGE_88V)
	#define RATE_VOLTAGE					880				/* ��ƿ��׼��ѹ  unit 0.1 */
	#define MAX_VOLTAGE						1020				/* ��ߵ�ѹ�������õ�ѹϵͳ������  unit 0.1 */
	#define CUT_VOLTAGE						762				/* ����Ƿѹ�����ڸõ�ѹϵͳ�������У��������Ť��  unit 0.1 */
	#define MIN_VOLTAGE						616				/* ��͵�ѹ�����ڸõ�ѹϵͳ��������ֹͣ����  unit 0.1 */
	#define BRO_VOLTAGE						550				/* ������ѹ�����ڸõ�ѹ��ϵͳ��λ���ر�һ�����  unit 0.1 */
	#define ESP_LOW_VOLTAGE				440				/* ��ͣ��ѹ�����ڸõ�ѹ����Ϊ��·���ر�PWM�����űۣ�����ϵ�ָ� unit 0.1  */
	#define STD_VOLTAGE						88.0			/* Bus voltage V */
#endif

#define BRKLIMIT_VOLTAGE			(MAX_VOLTAGE - 10) /* ������ѹ���ޣ�unit 0.1 */

#define OVER_TEMP_CUT_RATIO  (0.8)    /* ������ȵ���������80% */

/*******************************************************************************
* ��������
*******************************************************************************/
#define	RIGID_NUM							13
#define	SIGNAL_LAMP_NUM				10
#define SYS_TIMER_NUM					6
/* ���� */
#define PI									3.141592653589793		/* PI */
#define SEC_MIN							60.0								/* 60s = 1min */
#define SQRT2								1.4142135623731			/* sqrt(2) */
#define SQRT3								1.7320508075689			/* sqrt(3) */
#define SYSTEM_FREQUENCY		120									/* 120M */
#define FS									8000.0//16000.0							/* 8K����Ƶ�� */
#define TS									(1.0/FS)							/* 8K�������� */
#define C_1_MS							(FS/1000)							/* 1ms 	  =	125us x	8	 */
#define	MULT_256S						1										//��Χ��1~127.����1�������ʱ,�����ʱ��,1 ���� 256��
#define PH3_TO_PH2   				1.5

/* ����ֵ */
#define STD_WC							256.0								/* ��ֹƵ��1 = 256 Hz */
#define STD_SPEEDRPM				256.0								/* ת��1 = 256 RPM */
#define STD_SPEED						(2*PI*STD_SPEEDRPM/SEC_MIN)	
																								/* ת��1 = 26.8083 rad/s */
#define STD_FRQ						  (4.0)								/* 1=4Hz*/

#define STD_SPEEDACC				4096.0			/* ���ٶ�1 = 4096 rad/s^2 */


#define STD_INERTIA					(1.0/256.0)			/* ����1 = 3.9*10^-3 Kgm^2 */
#define STD_TORQUE_K				1.0				/* ת��ϵ��1 = 1 Nm/A */
#define STD_RESIST					1.0				/* DQ�����1 = 1 Ohm */
#define STD_T								1.0				/* time, 1 = 1 s */
#define STD_ABS17						131072.0
#define STD_INC2500					10000.0
#define STD_INC30						120.00
#define STD_INC32						128.00
#define STD_INC48						192.00
#define STD_INC64						256.00
#define STD_INC96						384.00


#if (CTLBOARD_TYPE ==_HB4_GD32)
	#define STD_VBUS            72.7	      /* ��ѹ������ֵ��3.3V*10*(6.8K+143K)/6.8K = 727 */
	#define PROPD_STD_CURRENT		6.556 				/*3.3*7.5/(7.5+68)/0.05*/
	#define PROPD_MAX_CURRENT		0.65
	#define POT_HIGH_MIN							2.5					/* V, <= THEN ALARM SHORT circuit*/
	#define POT_LOW_MAX								2.5					/* V, >= THEN ALARM SHORT circuit*/
	#define THROTTLE_5V_VIN_MAX				5.6					/* V, >= THEN ALARM NO SENSOR*/
	#define THROTTLE_POT2_5K_VIN_MAX	5.6					/* V, >= THEN ALARM NO SENSOR */
	#define THROTTLE_POT3_VIN_MAX			5.6					/* V, >= THEN ALARM NO SENSOR*/
	#define THROTTLE_10V_VIN_MAX			10.5				/* V, >= THEN ALARM*/
	#define THROTTLE_BUS_VIN_MAX			10.5				/* V, >= THEN ALARM*/
	
	#define	DoPowerOnCaL_EN					/* ʹ��DO�ϵ��⹦�� */
	#define	DoFdbVoltageCal_EN				/* ʹ��DO������⹦�� */
	#define	PROP_DITHER						/* ʹ�ܱ��������� */

#endif

#if (CTLBOARD_TYPE ==_HD4_GD32)
	#undef CTLBOARD_TYPE
	#define CTLBOARD_TYPE	_HB4_GD32  		/* ���ư�������HB4��ͬ */
	
	#define STD_VBUS            72.7	      /* ��ѹ������ֵ��3.3V*10*(6.8K+143K)/6.8K = 727 */
	#define PROPD_STD_CURRENT		6.556 				/*3.3*7.5/(7.5+68)/0.05*/
	#define PROPD_MAX_CURRENT		0.65
	#define POT_HIGH_MIN							2.5					/* V, <= THEN ALARM SHORT circuit*/
	#define POT_LOW_MAX								2.5					/* V, >= THEN ALARM SHORT circuit*/
	#define THROTTLE_5V_VIN_MAX				5.6					/* V, >= THEN ALARM NO SENSOR*/
	#define THROTTLE_POT2_5K_VIN_MAX	5.6					/* V, >= THEN ALARM NO SENSOR */
	#define THROTTLE_POT3_VIN_MAX			5.6					/* V, >= THEN ALARM NO SENSOR*/
	#define THROTTLE_10V_VIN_MAX			10.5				/* V, >= THEN ALARM*/
	#define THROTTLE_BUS_VIN_MAX			10.5				/* V, >= THEN ALARM*/
	
	#define	DoPowerOnCaL_EN					/* ʹ��DO�ϵ��⹦�� */
	#define	DoFdbVoltageCal_EN				/* ʹ��DO������⹦�� */
	#define	PROP_DITHER						/* ʹ�ܱ��������� */

#endif

#if (CTLBOARD_TYPE ==_HB6_GD32)
	#define STD_VBUS            72.7	      /* ��ѹ������ֵ��3.3V*10*(6.8K+143K)/6.8K = 727 */
	#define PROPD_STD_CURRENT		6.556 				/*3.3*7.5/(7.5+68)/0.05*/
	#define PROPD_MAX_CURRENT		0.65
	#define POT_HIGH_MIN							2.5					/* V, <= THEN ALARM SHORT circuit*/
	#define POT_LOW_MAX								2.5					/* V, >= THEN ALARM SHORT circuit*/
	#define THROTTLE_5V_VIN_MAX				5.6					/* V, >= THEN ALARM NO SENSOR*/
	#define THROTTLE_POT2_5K_VIN_MAX	5.6					/* V, >= THEN ALARM NO SENSOR */
	#define THROTTLE_POT3_VIN_MAX			5.6					/* V, >= THEN ALARM NO SENSOR*/
	#define THROTTLE_10V_VIN_MAX			10.5				/* V, >= THEN ALARM*/
	#define THROTTLE_BUS_VIN_MAX			10.5				/* V, >= THEN ALARM*/
		
	#define	DoPowerOnCaL_EN					/* ʹ��DO�ϵ��⹦�� */
	#define	DoFdbVoltageCal_EN				/* ʹ��DO������⹦�� */
	#define	PROP_DITHER						/* ʹ�ܱ��������� */
	
	#define DO_HIGH_SIDE_DRIVE_R			/* DO4~DO9 �߱�����������ƽ���� */
#endif


#define STD_5VOUT           66.0       /* 5V�����ѹ������ֵ��3.3V*10*2 = 6.6 */
#define STD_12VOUT          198.0      /* 12V���������ֵ��3.3V*10*6 = 19.8 */

#define STD_PERCENT					32768      /*100%(32768) �ٷֱȱ���ֵ*/
#define STD_ELECTHETA				360				/* ��Ƕȱ���ֵ 360��*/
//#define	DO_PWM_TIM_CLK			200000
#define	DO_PWM_TIM_PERIOD		1000

/* ̤���������� */
#define		THROTTLE_NONE					    0	//��̤������
#define		THROTTLE_RESISTANCE_2WIRE			1	//2�ߵ����� 0~5K, 5K~0
#define   	THROTTLE_RESISTANCE_3WIRE			2 	//3�ߵ����� 1K~10K 
#define		THROTTLE_BUS					    3	//������
#define		THROTTLE_VOLTAGE10V					4	//��ѹ��0~10V
#define		THROTTLE_VOLTAGE5V					5	//��ѹ��0~5V

/* �������ź��������� */
#define DIRECTION_ENCODER					0  //����������
#define DIRECTION_VOLTAGE         			1  //��ѹ������

/* ����ģʽ */
#define POS_RUN_MODE								0
#define SPEED_RUN_MODE							1
#define TORQUE_RUN_MODE							2

#if (LOGIC_TYPE == _CONTROLLER_STEER)
	#define DEFAULT_RUNMODE  POS_RUN_MODE
#else
	#define DEFAULT_RUNMODE  SPEED_RUN_MODE
#endif

/* FSM ״̬ */
#define	FSMSTATE_INACTIVE						0
#define	FSMSTATE_TO_BRKOFF					1
#define	FSMSTATE_ACTIVE							2
#define	FSMSTATE_PREPARE						3
#define	FSMSTATE_TO_PWMOFF					4

/* Tune FSM state definitions */
#define TUNE_IDLE								0x00
#define TUNE_INERTIA						0x01
#define TUNE_CALIBZ							0x02

/* Tune inertia FSM state */
#define TUNE_INERTIA_IDLE					0x10
#define TUNE_INERTIA_START				0x11
#define TUNE_INERTIA_Z						0x12
#define TUNE_INERTIA_Z_DONE					0x13
#define TUNE_INERTIA_STOP1					0x14
#define TUNE_INERTIA_POSACC1				0x15
#define TUNE_INERTIA_POSDEC1				0x16
#define TUNE_INERTIA_NEGACC1				0x17
#define TUNE_INERTIA_NEGDEC1				0x18
#define TUNE_INERTIA_STOP2					0x19
#define TUNE_INERTIA_POSACC2				0x1A
#define TUNE_INERTIA_POSDEC2				0x1B
#define TUNE_INERTIA_NEGACC2				0x1C
#define TUNE_INERTIA_NEGDEC2				0x1D
#define TUNE_INERTIA_DONE						0x1E

/* Tune calib z FSM state */
#define TUNE_CALIBZ_IDLE					0x20
#define TUNE_CALIBZ_START					0x21

/* FFT */
#define FFT_N											1024
#define FFT_SAMPLE_INTERVAL					2
#define FFT_SAMPLE_FREQ					(FS/FFT_SAMPLE_INTERVAL)
#define FFT_RESOLVING						(FFT_SAMPLE_FREQ/FFT_N)

/* Vibration */
#define VIBRATION_DETECT_IDLE				0x00
#define VIBRATION_DETECT_SAMPLE			0x01
#define VIBRATION_DETECT_CALC				0x02
#define VIBRATION_DETECT_DONE				0x03

/* background loop timer period */
#define C_1_MS_TIMER_PERIOD					(1*C_1_MS)		/* 1ms 	  =	125us x	8	 */
#define C_4_MS_TIMER_PERIOD					(4*C_1_MS)		/* 4ms 	  =	125us x	32	 */
#define C_10_MS_TIMER_PERIOD				(10*C_1_MS)		/* 10ms	  =	125us x	80	 */
#define C_120_MS_TIMER_PERIOD				(120*C_1_MS)	/* 120ms  =	125us x	960	 */
#define C_150_MS_TIMER_PERIOD				(150*C_1_MS)	/* 150ms  =	125us x	1200 */
#define C_1000_MS_TIMER_PERIOD			(1000*C_1_MS)	/* 1000ms =	125us x	8000 */
#define C_500_MS_TIMER_PERIOD				(500*C_1_MS)	/* 500ms =	125us x	4000 */

#define T_MS_PLC_PERIOD							5             /* 5ms */
/*******************************************************************************
* 1. KERNEL(PLC<->KERNEL)
*******************************************************************************/
/* KERNEL <- PLC */ 
#define 		SL_TER_RDY							0x0000U	//ĸ�ߵ�ѹ����
#define 		SL_TER_SRVON						0x0001U //����������ָ��
#define 		SL_TER_PROP						0x0002U //������ʹ��ָ��
#define 		SL_TER_CURRENT_CUT			0x0003U //��������������ָ��
#define 		SL_TER_BAT_ERROR						0x0004U //��ص�����Ƿ
#define 		SL_TER_BAT_PROTECTALM			0x0005U //��ص�������10%
#define 		SL_TER_BAT_LOWALM					0x0006U //��ص�������20%
#define 		SL_TER_THROTTLE_ZERO				0x0007U //̤��0����
#define 		SL_TER_PROP_CURRENT_CUT				0x0008U //�ÿط����������
										/*	0x0009U */
										/*	0x000AU */
										/*	0x000BU	*/
										/*	0x000CU */	
										/*	0x000DU */	
										/*	0x000EU */	
										/*	0x000FU */

/* KERNEL -> PLC */
#define 		SL_TER_EN_PWM						0x0100U
										/*	0x0101U */	
										/*	0x0102U */	
										/*	0x0103U */	
										/*	0x0104U */	
										/*	0x0105U */	
										/*	0x0106U */	
#define			SL_ESP_VOLTAGE_ERR			  0x0107U
#define 		SL_OVER_CURRENT_ERR				0x0108U 
#define 		SL_2MOVER_CURRENT_ERR			0x0109U 
#define 		SL_FXP_ERROR							0x010AU
										/*	0x010BU */
										/*	0x010CU */
										/*	0x010DU */
										/*	0x010EU */
										/*	0x010FU */
										
#define 		SL_TER_CWLI							0x0101U //��SL_TER_CWLI����Ϊ0ʱ����ֹ��ת
#define			SL_TER_CCWLI						0x0102U //��SL_TER_CCWLI����Ϊ0ʱ����ֹ��ת
//#define 		SL_TER_ACLR							0x0105U

/*******************************************************************************
* 2. PLC(PLC<->KERNEL)
*******************************************************************************/
/* PLC <- KERNEL */
#define 		PLC_TER_EN_PWM						0x0200U
										/*	0x0201U */
										/*	0x0202U */
										/*	0x0203U */
										/*	0x0204U */
										/*	0x0205U */
										/*	0x0206U */
										/*	0x0207U */
#define 		PLC_OVER_CURRENT_ERR			0x0208U 
										/*	0x0209U */
#define 		PLC_FXP_ERROR							0x020AU
										/*	0x020BU */
										/*	0x020CU */
										/*	0x020DU */
										/*	0x020EU */
										/*	0x020FU */
//#define 		SL_TER_EN_BRAKE					0x0207U
#define 		PLC_TER_SBDC				0x020FU			// 

/* PLC -> KERNEL */
#define 		PLC_TER_RDY							0x0300U	//���ʰ��Դ����
#define 		PLC_TER_SRVON						0x0301U //����������ָ��
#define 		PLC_TER_PROP						0x0302U //������ʹ��ָ��
#define 		PLC_TER_CURRENT_CUT			0x0303U //��������������ָ��
#define 		PLC_BAT_ERROR						0x0304U //��ص�����Ƿ
#define 		PLC_BAT_PROTECTALM			0x0305U //��ص�������10%
#define 		PLC_BAT_LOWALM					0x0306U //��ص�������20%
#define 		PLC_THROTTLE_ZERO				0x0307U //̤��0����
#define 		PLC_THROTTLE_OVER_ERR		0x0308U //Throttle over voltage
#define 		PLC_BRAKE_OVER_ERR			0x0309U //Brake over voltage
#define 		PLC_TER_ERVS						0x030AU //E Rvs Act
#define 		PLC_MAIN_DRIVER_OFF			0x030CU //�Ͽ����Ӵ���
#define 		PLC_ESP_BRAKE						0x030DU
#define 		PLC_TER_CHKPHASE				0x030EU //ȱ����	
#define 		PLC_CURRENT_LIMIT				0x030FU //over current limit
#define 		PLC_MOTOR_OPEN_ERR			0x030BU //Motor open

/*******************************************************************************
* 2. PLC 
*******************************************************************************/
#define			PLC_KSI_RDY								0x0400U /*	*/
#define			PLC_VBUS_RDY							0x0401U /*	*/
#define			PLC_C8051F_RDY						0x0402U /*	*/
#define			PLC_DRIVE1_CONNECT				0x0403U 
#define			PLC_DRIVE2_CONNECT				0x0404U /*	*/
#define			PLC_DRIVE3_CONNECT				0x0405U /*	*/
#define			PLC_DRIVE4_CONNECT				0x0406U /*	*/
#define			PLC_SYS_ENABLE						0x0407U /*ϵͳʹ��*/
#define			PLC_STOP_MOVE							0x0408U /**/
#define			PLC_MIDDLE_RDY						0x0409U	/*��λ���趨*/
#define     PLC_POWER_THRES_ERR	    0x040AU  /* ���ʰ���µ�·�쳣����*/
#define 		PLC_CUT_VOLTAGE_ERR	  		0x040BU	/* ��ص�ѹ��ȹ��ͣ�����������*/
#define 		PLC_MIN_VOLTAGE_ERR			  0x040CU	/* ��ص�ѹ���ع��ͣ�ͣ����*/
#define 		PLC_MAX_VOLTAGE_ERR			  0x040DU	/* ��ص�ѹ����*/
#define 		PLC_POWER_CUT_TMP_ERR		  0x040EU	/* ���ʰ���ȹ��£�85������������*/
#define 		PLC_POWER_MAX_TMP_ERR		  0x040FU	/* ���ʰ����ع��£�95��ͣ����*/

#define			PLC_POWER_MIN_TMP_ERR			0x0500U	/* ���ʰ���£�-25�ȣ�*/
#define			PLC_MOTOR_CUT_TMP_ERR		  0x0501U /* �����ȸ���*/
#define			PLC_MOTOR_MAX_TMP_ERR		  0x0502U /* ������ظ���*/
#define			PLC_CAP_CHARGE_ERR				0x0503U /* ĸ�ߵ��ݳ��*/
#define			PLC_OUT_5V_ERR						0x0504U /* ���5V����*/
#define			PLC_OUT_12V_ERR						0x0505U /* ���12V����*/
#define			PLC_MAIN_CONNECT_ERR	  	0x0506U /* ���Ӵ������ӹ���*/
#define			PLC_DRIVE2_CONNECT_ERR		0x0507U	/* ����ƶ����ӹ���*/
#define			PLC_DRIVE3_CONNECT_ERR		0x0508U /* ����3���ӹ���*/
#define			PLC_DRIVE4_CONNECT_ERR		0x0509U /* ����4���ӹ���*/
#define			PLC_MAIN_WELDED_ERR				0x050AU /* ���Ӵ��������۽�*/
#define			PLC_MAIN_DRIVE_ERR				0x050BU /* ���Ӵ�����������*/
#define     ICAN_MACIDCHECK_ERR				0x050CU	/* MACID���ʧ��*/
#define     ICAN_CONNECT_ERR					0x050DU	/* ICAN����ʧ��*/										
#define     PLC_IOLOGIC_ERR					  0x050EU	/* io POWER ON LOGIC ERRROR*/										
#define     PLC_POT_SHORTCIRCUITS_ERR	0x050FU  /*��λ�ƶ�·*/

/*******************************************************************************
* 3. Kernel
*******************************************************************************/
#define 		SL_POSCMD_OV 						0x0600U	/* λ�ÿ���ģʽʱָ����� */
#define 		SL_SPDCMD_OV						0x0601U	/* �ٶȿ���ģʽʱָ����� */
#define 		SL_TORCMD_OV	 					0x0602U	/* ת�ؿ���ģʽʱָ����� */
#define 		SL_ASR_SATURATED 				0x0603U /* �ٶȻ����ֱ��� */
#define 		SL_POWER_RDY 						0x0604U	/* ��Դ��ready */
#define 		SL_KERNEL_ERR 					0x0605U /* Kernel���д��� */
#define 		SL_RE_POWER_ON 					0x0606U	/* �����¿��� */
#define 		SL_TER_SRDY							0x0607U	
#define 		SL_TER_DALM_N						0x0608U	
#define 		SL_TER_COIN_N						0x0609U
#define 		SL_TER_BRK_N						0x060AU
										/*	0x060BU */
										/*	0x060CU */
										/*	0x060DU */
										/*	0x060EU */
										/*	0x060FU */
#define 		SL_APR_RUN							0x0700U
#define 		SL_ASR_RUN							0x0701U
#define 		SL_ACR_RUN							0x0702U
#define			SL_TU1_ALM							0x0703U 
#define 		SL_ERR_TIP_LOG					0x0704U
#define 		SL_ERR_TIP_VISIB				0x0705U
#define 		SL_ALM_TIP_VISIB				0x0706U
#define 		SL_PARA_MODIFIED				0x0707U	/* �����ѱ��޸� */
#define 		SL_OVER_LOAD_ERR				0x0708U	
#define 		SL_ENCODER_UVW_ERR			0x0709U
#define 		SL_ENCODER_DIR_ERR			0x070AU
#define 		SL_ENCODER_COMM_ERR			0x070BU
#define 		PLC_EEPROM_RW_ERR				0x070CU	/* EEPROM��д��������*/
#define 		SL_EEPROM_BACK_ERR			0x070DU
#define 		PLC_PARA_OV_INDEX_ERR		0x070EU /* ������Ŵ���*/
#define 		PLC_PARA_OV_LIMIT_ERR		0x070FU /* �������޴���*/
/*******************************************************************************
* 4. HMI
*******************************************************************************/
										/*	0x0800U */
										/*	0x0801U */
										/*	0x0802U */
										/*	0x0803U */
										/*	0x0804U */
										/*	0x0805U */
										/*	0x0806U */
#define 		SL_SAVE_MODIFIED_PARA				0x0807U	/* �����޸ĵĲ��� */
#define 		SL_SAVE_RIGID_RELATED_PARA	0x0808U	/* ���Ա����������仯 */
#define 		SL_SAVE_MOTOR_RELATED_PARA	0x0809U	/* ��������������仯 */
										/*	0x080AU */
										/*	0x080BU */
										/*	0x080CU */
										/*	0x080DU */
										/*	0x080EU */
										/*	0x080FU */
/*******************************************************************************
* 5. other
*******************************************************************************/
#define			SL_BATTERY_LOST				0x0900U 
#define			SL_MASK_BATERRY_COUNT		0x0901U
#define			PLC_TER_STEERLIMIT_SPD			0x0902U
#define			SL_PHASELOST_ERR			0x0903U
#define			PLC_TUNE_SVON				0x0904U 
#define			PLC_TUNE_BRAKEOPEN			0x0905U 
#define			SL_LOGIC_BUSY					0x0906U 	//PLC�߼�����æ
										/*	0x0907U */		
										/*	0x0908U */
										/*	0x0909U */
										/*	0x090AU */
										/*	0x090BU */
										/*	0x090CU */
										/*	0x090DU */
#define			SL_INP_TUNE_INERTIA			0x090EU
#define			SL_INP_TUNE_CALIBZ			0x090FU

/*******************************************************************************
* Signal lamp operation macro
*******************************************************************************/

#define SL_SET(sl)		(gCRam.SigLamp[sl >> 8] |= (  1U << (sl & 0x00FFU) ))
#define SL_CLR(sl)		(gCRam.SigLamp[sl >> 8] &= (~(1U << (sl & 0x00FFU))))
#define SL_CHK(sl)		(gCRam.SigLamp[sl >> 8] &  (  1U << (sl & 0x00FFU) ))

#define KERNEL_IN_FROM_PLC		0
#define KERNEL_OUT_TO_PLC			1
#define PLC_IN_FROM_KERNEL		2
#define PLC_OUT_TO_KERNEL			3

#define MAX_ERR_NO						30
#define MIN_ALM_NO						31
extern		INT16U			gCpuLoad;
/*******************************************************************************
* ��������
*******************************************************************************/
extern void ControlISR(void);//
extern void PLCISR(void);

#endif //#ifndef _KSDSYS_H_

