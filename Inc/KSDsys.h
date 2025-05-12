/*******************************************************************************
* Filename: KSDsys.h 	                                       		               *
* Description: 系统定义		           				                                   *
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
	#define RATE_1H_CURRENT				120				/* 1小时工作制电流  unit A */
	#define RATE_2M_CURRENT				250				/* 2分钟工作制电流   unit A */
	#define PRODUCT_TYPE          2425
	#define ESP_OVER_CUREENT			(RATE_2M_CURRENT*1.5)		/* 驱动器必须急停保护的电流 , A*/									
	#define STD_CURRENT						1260.0//1668.0				/* 1236 电流回路 1= 1668A */
	#define SOFTVERSION						((SOFTVERSION_KENERL << 8) | SOFTVERSION_LOGIC)
	#define HARDVERSION1					0x0203			/* 低字节控制板、高字节功率板 */
	#define HARDVERSION2					0x0103			/* 低字节驱动板、高字节电容板/电流板*/
	#define FRAMEVERSION					0x0111			/* BIT0~3外壳、BIT4~7底板、BIT8~11结构件*/
#endif

#if(DRIVER_TYPE	==	_4825_HB4_GD32)
	#define CTLBOARD_TYPE		_HB4_GD32
	#define VOLTAGE_LEVEL		_VOLTAGE_48V
	#define RATE_1H_CURRENT				120				/* 1小时工作制电流  unit A */
	#define RATE_2M_CURRENT				250				/* 2分钟工作制电流   unit A */
	#define PRODUCT_TYPE          2425
	#define ESP_OVER_CUREENT			(RATE_2M_CURRENT*1.5)		/* 驱动器必须急停保护的电流 , A*/									
	#define STD_CURRENT						1260.0//1668.0				/* 1236 电流回路 1= 1668A */
	#define SOFTVERSION						((SOFTVERSION_KENERL << 8) | SOFTVERSION_LOGIC)
	#define HARDVERSION1					0x0203			/* 低字节控制板、高字节功率板 */
	#define HARDVERSION2					0x0103			/* 低字节驱动板、高字节电容板/电流板*/
	#define FRAMEVERSION					0x0111			/* BIT0~3外壳、BIT4~7底板、BIT8~11结构件*/
#endif

#if(DRIVER_TYPE	==	_2425_HD4_GD32)
	#define CTLBOARD_TYPE		_HD4_GD32 
	#define VOLTAGE_LEVEL		_VOLTAGE_24V
	#define RATE_1H_CURRENT				120				/* 1小时工作制电流  unit A */
	#define RATE_2M_CURRENT				250				/* 2分钟工作制电流   unit A */
	#define PRODUCT_TYPE          2425
	#define ESP_OVER_CUREENT			(RATE_2M_CURRENT*1.5)		/* 驱动器必须急停保护的电流 , A*/									
	#define STD_CURRENT						1260.0//1668.0				/* 1236 电流回路 1= 1668A */
	#define SOFTVERSION						((SOFTVERSION_KENERL << 8) | SOFTVERSION_LOGIC)
	#define HARDVERSION1					0x0203			/* 低字节控制板、高字节功率板 */
	#define HARDVERSION2					0x0103			/* 低字节驱动板、高字节电容板/电流板*/
	#define FRAMEVERSION					0x0111			/* BIT0~3外壳、BIT4~7底板、BIT8~11结构件*/
#endif

#if(DRIVER_TYPE	==	_4825_HD4_GD32)
	#define CTLBOARD_TYPE		_HD4_GD32
	#define VOLTAGE_LEVEL		_VOLTAGE_48V
	#define RATE_1H_CURRENT				120				/* 1小时工作制电流  unit A */
	#define RATE_2M_CURRENT				250				/* 2分钟工作制电流   unit A */
	#define PRODUCT_TYPE          4825
	#define ESP_OVER_CUREENT			(RATE_2M_CURRENT*1.5)		/* 驱动器必须急停保护的电流 , A*/									
	#define STD_CURRENT						1260.0//1668.0				/* 1236 电流回路 1= 1668A */
	#define SOFTVERSION						((SOFTVERSION_KENERL << 8) | SOFTVERSION_LOGIC)
	#define HARDVERSION1					0x0203			/* 低字节控制板、高字节功率板 */
	#define HARDVERSION2					0x0103			/* 低字节驱动板、高字节电容板/电流板*/
	#define FRAMEVERSION					0x0111			/* BIT0~3外壳、BIT4~7底板、BIT8~11结构件*/
#endif

#if(DRIVER_TYPE	==	_4825_HB6_GD32)
	#define CTLBOARD_TYPE		_HB6_GD32
	#define VOLTAGE_LEVEL		_VOLTAGE_24V
	#define RATE_1H_CURRENT				120				/* 1小时工作制电流  unit A */
	#define RATE_2M_CURRENT				250				/* 2分钟工作制电流   unit A */
	#define PRODUCT_TYPE          2412
	#define ESP_OVER_CUREENT			(RATE_2M_CURRENT*1.5)		/* 驱动器必须急停保护的电流 , A*/									
	#define STD_CURRENT						1260.0//1668.0				/* 1236 电流回路 1= 1668A */
	#define SOFTVERSION						((SOFTVERSION_KENERL << 8) | SOFTVERSION_LOGIC)
	#define HARDVERSION1					0x0203			/* 低字节控制板、高字节功率板 */
	#define HARDVERSION2					0x0103			/* 低字节驱动板、高字节电容板/电流板*/
	#define FRAMEVERSION					0x0111			/* BIT0~3外壳、BIT4~7底板、BIT8~11结构件*/
#endif

#if (VOLTAGE_LEVEL == _VOLTAGE_24V)
	#define RATE_VOLTAGE					240				/* 电瓶标准电压 unit 0.1 */
	#define MAX_VOLTAGE						350				/* 最高电压，超过该电压系统不启动 unit 0.1 */
	#define CUT_VOLTAGE						208				/* 削减电压，低于该电压系统降耗运行，削减输出扭矩 unit 0.1  */
	#define MIN_VOLTAGE						168				/* 最低电压，低于该电压系统不启动，停止运行 unit 0.1  */
	#define BRO_VOLTAGE						150				/* 启动电压，低于该电压，系统复位，关闭一切输出 unit 0.1  */
	#define ESP_LOW_VOLTAGE				120				/* 急停电压，低于该电压，认为短路，关闭PWM上下桥臂，必须断电恢复 unit 0.1  */
	#define STD_VOLTAGE						24.0				/* Bus voltage V */
#endif

#if (VOLTAGE_LEVEL == _VOLTAGE_48V)
	#define RATE_VOLTAGE					480				/* 电瓶标准电压  unit 0.1 */
	#define MAX_VOLTAGE						630				/* 最高电压，超过该电压系统不启动  unit 0.1 */
	#define CUT_VOLTAGE						418				/* 工作欠压，低于该电压系统降耗运行，削减输出扭矩  unit 0.1 */
	#define MIN_VOLTAGE						336				/* 最低电压，低于该电压系统不启动，停止运行  unit 0.1 */
	#define BRO_VOLTAGE						300				/* 启动电压，低于该电压，系统复位，关闭一切输出  unit 0.1 */
	#define ESP_LOW_VOLTAGE				240				/* 急停电压，低于该电压，认为短路，关闭PWM上下桥臂，必须断电恢复 unit 0.1  */
	#define STD_VOLTAGE						48.0				/* Bus voltage V */
#endif

#if (VOLTAGE_LEVEL == _VOLTAGE_72V)
	#define RATE_VOLTAGE					720				/* 电瓶标准电压  unit 0.1 */
	#define MAX_VOLTAGE						920				/* 最高电压，超过该电压系统不启动  unit 0.1 */
	#define CUT_VOLTAGE						623				/* 工作欠压，低于该电压系统降耗运行，削减输出扭矩  unit 0.1 */
	#define MIN_VOLTAGE						504				/* 最低电压，低于该电压系统不启动，停止运行  unit 0.1 */
	#define BRO_VOLTAGE						450				/* 启动电压，低于该电压，系统复位，关闭一切输出  unit 0.1 */
	#define ESP_LOW_VOLTAGE				360				/* 急停电压，低于该电压，认为短路，关闭PWM上下桥臂，必须断电恢复 unit 0.1  */
	#define STD_VOLTAGE						72.0			/* Bus voltage V */
#endif

#if (VOLTAGE_LEVEL == _VOLTAGE_80V)
	#define RATE_VOLTAGE					800				/* 电瓶标准电压  unit 0.1 */
	#define MAX_VOLTAGE						100				/* 最高电压，超过该电压系统不启动  unit 0.1 */
	#define CUT_VOLTAGE						692				/* 工作欠压，低于该电压系统降耗运行，削减输出扭矩  unit 0.1 */
	#define MIN_VOLTAGE						560				/* 最低电压，低于该电压系统不启动，停止运行  unit 0.1 */
	#define BRO_VOLTAGE						500				/* 启动电压，低于该电压，系统复位，关闭一切输出  unit 0.1 */
	#define ESP_LOW_VOLTAGE				400				/* 急停电压，低于该电压，认为短路，关闭PWM上下桥臂，必须断电恢复 unit 0.1  */
	#define STD_VOLTAGE						80.0			/* Bus voltage V */
#endif

#if (VOLTAGE_LEVEL == _VOLTAGE_88V)
	#define RATE_VOLTAGE					880				/* 电瓶标准电压  unit 0.1 */
	#define MAX_VOLTAGE						1020				/* 最高电压，超过该电压系统不启动  unit 0.1 */
	#define CUT_VOLTAGE						762				/* 工作欠压，低于该电压系统降耗运行，削减输出扭矩  unit 0.1 */
	#define MIN_VOLTAGE						616				/* 最低电压，低于该电压系统不启动，停止运行  unit 0.1 */
	#define BRO_VOLTAGE						550				/* 启动电压，低于该电压，系统复位，关闭一切输出  unit 0.1 */
	#define ESP_LOW_VOLTAGE				440				/* 急停电压，低于该电压，认为短路，关闭PWM上下桥臂，必须断电恢复 unit 0.1  */
	#define STD_VOLTAGE						88.0			/* Bus voltage V */
#endif

#define BRKLIMIT_VOLTAGE			(MAX_VOLTAGE - 10) /* 泵生电压上限，unit 0.1 */

#define OVER_TEMP_CUT_RATIO  (0.8)    /* 电机过热电流削减到80% */

/*******************************************************************************
* 常量定义
*******************************************************************************/
#define	RIGID_NUM							13
#define	SIGNAL_LAMP_NUM				10
#define SYS_TIMER_NUM					6
/* 常量 */
#define PI									3.141592653589793		/* PI */
#define SEC_MIN							60.0								/* 60s = 1min */
#define SQRT2								1.4142135623731			/* sqrt(2) */
#define SQRT3								1.7320508075689			/* sqrt(3) */
#define SYSTEM_FREQUENCY		120									/* 120M */
#define FS									8000.0//16000.0							/* 8K采样频率 */
#define TS									(1.0/FS)							/* 8K采样周期 */
#define C_1_MS							(FS/1000)							/* 1ms 	  =	125us x	8	 */
#define	MULT_256S						1										//范围：1~127.过流1倍额定电流时,允许的时间,1 代表 256秒
#define PH3_TO_PH2   				1.5

/* 标幺值 */
#define STD_WC							256.0								/* 截止频率1 = 256 Hz */
#define STD_SPEEDRPM				256.0								/* 转速1 = 256 RPM */
#define STD_SPEED						(2*PI*STD_SPEEDRPM/SEC_MIN)	
																								/* 转速1 = 26.8083 rad/s */
#define STD_FRQ						  (4.0)								/* 1=4Hz*/

#define STD_SPEEDACC				4096.0			/* 加速度1 = 4096 rad/s^2 */


#define STD_INERTIA					(1.0/256.0)			/* 惯量1 = 3.9*10^-3 Kgm^2 */
#define STD_TORQUE_K				1.0				/* 转矩系数1 = 1 Nm/A */
#define STD_RESIST					1.0				/* DQ相电阻1 = 1 Ohm */
#define STD_T								1.0				/* time, 1 = 1 s */
#define STD_ABS17						131072.0
#define STD_INC2500					10000.0
#define STD_INC30						120.00
#define STD_INC32						128.00
#define STD_INC48						192.00
#define STD_INC64						256.00
#define STD_INC96						384.00


#if (CTLBOARD_TYPE ==_HB4_GD32)
	#define STD_VBUS            72.7	      /* 电压检测标幺值：3.3V*10*(6.8K+143K)/6.8K = 727 */
	#define PROPD_STD_CURRENT		6.556 				/*3.3*7.5/(7.5+68)/0.05*/
	#define PROPD_MAX_CURRENT		0.65
	#define POT_HIGH_MIN							2.5					/* V, <= THEN ALARM SHORT circuit*/
	#define POT_LOW_MAX								2.5					/* V, >= THEN ALARM SHORT circuit*/
	#define THROTTLE_5V_VIN_MAX				5.6					/* V, >= THEN ALARM NO SENSOR*/
	#define THROTTLE_POT2_5K_VIN_MAX	5.6					/* V, >= THEN ALARM NO SENSOR */
	#define THROTTLE_POT3_VIN_MAX			5.6					/* V, >= THEN ALARM NO SENSOR*/
	#define THROTTLE_10V_VIN_MAX			10.5				/* V, >= THEN ALARM*/
	#define THROTTLE_BUS_VIN_MAX			10.5				/* V, >= THEN ALARM*/
	
	#define	DoPowerOnCaL_EN					/* 使能DO上电检测功能 */
	#define	DoFdbVoltageCal_EN				/* 使能DO反馈检测功能 */
	#define	PROP_DITHER						/* 使能比例阀抖动 */

#endif

#if (CTLBOARD_TYPE ==_HD4_GD32)
	#undef CTLBOARD_TYPE
	#define CTLBOARD_TYPE	_HB4_GD32  		/* 控制板类型与HB4相同 */
	
	#define STD_VBUS            72.7	      /* 电压检测标幺值：3.3V*10*(6.8K+143K)/6.8K = 727 */
	#define PROPD_STD_CURRENT		6.556 				/*3.3*7.5/(7.5+68)/0.05*/
	#define PROPD_MAX_CURRENT		0.65
	#define POT_HIGH_MIN							2.5					/* V, <= THEN ALARM SHORT circuit*/
	#define POT_LOW_MAX								2.5					/* V, >= THEN ALARM SHORT circuit*/
	#define THROTTLE_5V_VIN_MAX				5.6					/* V, >= THEN ALARM NO SENSOR*/
	#define THROTTLE_POT2_5K_VIN_MAX	5.6					/* V, >= THEN ALARM NO SENSOR */
	#define THROTTLE_POT3_VIN_MAX			5.6					/* V, >= THEN ALARM NO SENSOR*/
	#define THROTTLE_10V_VIN_MAX			10.5				/* V, >= THEN ALARM*/
	#define THROTTLE_BUS_VIN_MAX			10.5				/* V, >= THEN ALARM*/
	
	#define	DoPowerOnCaL_EN					/* 使能DO上电检测功能 */
	#define	DoFdbVoltageCal_EN				/* 使能DO反馈检测功能 */
	#define	PROP_DITHER						/* 使能比例阀抖动 */

#endif

#if (CTLBOARD_TYPE ==_HB6_GD32)
	#define STD_VBUS            72.7	      /* 电压检测标幺值：3.3V*10*(6.8K+143K)/6.8K = 727 */
	#define PROPD_STD_CURRENT		6.556 				/*3.3*7.5/(7.5+68)/0.05*/
	#define PROPD_MAX_CURRENT		0.65
	#define POT_HIGH_MIN							2.5					/* V, <= THEN ALARM SHORT circuit*/
	#define POT_LOW_MAX								2.5					/* V, >= THEN ALARM SHORT circuit*/
	#define THROTTLE_5V_VIN_MAX				5.6					/* V, >= THEN ALARM NO SENSOR*/
	#define THROTTLE_POT2_5K_VIN_MAX	5.6					/* V, >= THEN ALARM NO SENSOR */
	#define THROTTLE_POT3_VIN_MAX			5.6					/* V, >= THEN ALARM NO SENSOR*/
	#define THROTTLE_10V_VIN_MAX			10.5				/* V, >= THEN ALARM*/
	#define THROTTLE_BUS_VIN_MAX			10.5				/* V, >= THEN ALARM*/
		
	#define	DoPowerOnCaL_EN					/* 使能DO上电检测功能 */
	#define	DoFdbVoltageCal_EN				/* 使能DO反馈检测功能 */
	#define	PROP_DITHER						/* 使能比例阀抖动 */
	
	#define DO_HIGH_SIDE_DRIVE_R			/* DO4~DO9 高边驱动反馈电平反相 */
#endif


#define STD_5VOUT           66.0       /* 5V输出电压检测标幺值：3.3V*10*2 = 6.6 */
#define STD_12VOUT          198.0      /* 12V输出检测标幺值：3.3V*10*6 = 19.8 */

#define STD_PERCENT					32768      /*100%(32768) 百分比标幺值*/
#define STD_ELECTHETA				360				/* 电角度标幺值 360度*/
//#define	DO_PWM_TIM_CLK			200000
#define	DO_PWM_TIM_PERIOD		1000

/* 踏板输入类型 */
#define		THROTTLE_NONE					    0	//无踏板输入
#define		THROTTLE_RESISTANCE_2WIRE			1	//2线电阻型 0~5K, 5K~0
#define   	THROTTLE_RESISTANCE_3WIRE			2 	//3线电阻型 1K~10K 
#define		THROTTLE_BUS					    3	//总线型
#define		THROTTLE_VOLTAGE10V					4	//电压型0~10V
#define		THROTTLE_VOLTAGE5V					5	//电压型0~5V

/* 方向盘信号输入类型 */
#define DIRECTION_ENCODER					0  //编码器输入
#define DIRECTION_VOLTAGE         			1  //电压型输入

/* 运行模式 */
#define POS_RUN_MODE								0
#define SPEED_RUN_MODE							1
#define TORQUE_RUN_MODE							2

#if (LOGIC_TYPE == _CONTROLLER_STEER)
	#define DEFAULT_RUNMODE  POS_RUN_MODE
#else
	#define DEFAULT_RUNMODE  SPEED_RUN_MODE
#endif

/* FSM 状态 */
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
#define 		SL_TER_RDY							0x0000U	//母线电压正常
#define 		SL_TER_SRVON						0x0001U //控制器启动指令
#define 		SL_TER_PROP						0x0002U //比例阀使能指令
#define 		SL_TER_CURRENT_CUT			0x0003U //控制器电流消减指令
#define 		SL_TER_BAT_ERROR						0x0004U //电池电量亏欠
#define 		SL_TER_BAT_PROTECTALM			0x0005U //电池电量低于10%
#define 		SL_TER_BAT_LOWALM					0x0006U //电池电量低于20%
#define 		SL_TER_THROTTLE_ZERO				0x0007U //踏板0输入
#define 		SL_TER_PROP_CURRENT_CUT				0x0008U //泵控反向电流消减
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
										
#define 		SL_TER_CWLI							0x0101U //当SL_TER_CWLI输入为0时，禁止正转
#define			SL_TER_CCWLI						0x0102U //当SL_TER_CCWLI输入为0时，禁止反转
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
#define 		PLC_TER_RDY							0x0300U	//功率板电源正常
#define 		PLC_TER_SRVON						0x0301U //控制器启动指令
#define 		PLC_TER_PROP						0x0302U //比例阀使能指令
#define 		PLC_TER_CURRENT_CUT			0x0303U //控制器电流消减指令
#define 		PLC_BAT_ERROR						0x0304U //电池电量亏欠
#define 		PLC_BAT_PROTECTALM			0x0305U //电池电量低于10%
#define 		PLC_BAT_LOWALM					0x0306U //电池电量低于20%
#define 		PLC_THROTTLE_ZERO				0x0307U //踏板0输入
#define 		PLC_THROTTLE_OVER_ERR		0x0308U //Throttle over voltage
#define 		PLC_BRAKE_OVER_ERR			0x0309U //Brake over voltage
#define 		PLC_TER_ERVS						0x030AU //E Rvs Act
#define 		PLC_MAIN_DRIVER_OFF			0x030CU //断开主接触器
#define 		PLC_ESP_BRAKE						0x030DU
#define 		PLC_TER_CHKPHASE				0x030EU //缺相检查	
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
#define			PLC_SYS_ENABLE						0x0407U /*系统使能*/
#define			PLC_STOP_MOVE							0x0408U /**/
#define			PLC_MIDDLE_RDY						0x0409U	/*中位已设定*/
#define     PLC_POWER_THRES_ERR	    0x040AU  /* 功率板测温电路异常电阻*/
#define 		PLC_CUT_VOLTAGE_ERR	  		0x040BU	/* 电池电压轻度过低（性能消减）*/
#define 		PLC_MIN_VOLTAGE_ERR			  0x040CU	/* 电池电压严重过低（停机）*/
#define 		PLC_MAX_VOLTAGE_ERR			  0x040DU	/* 电池电压过高*/
#define 		PLC_POWER_CUT_TMP_ERR		  0x040EU	/* 功率板轻度过温（85度性能消减）*/
#define 		PLC_POWER_MAX_TMP_ERR		  0x040FU	/* 功率板严重过温（95度停机）*/

#define			PLC_POWER_MIN_TMP_ERR			0x0500U	/* 功率板低温（-25度）*/
#define			PLC_MOTOR_CUT_TMP_ERR		  0x0501U /* 电机轻度高温*/
#define			PLC_MOTOR_MAX_TMP_ERR		  0x0502U /* 电机严重高温*/
#define			PLC_CAP_CHARGE_ERR				0x0503U /* 母线电容充电*/
#define			PLC_OUT_5V_ERR						0x0504U /* 输出5V故障*/
#define			PLC_OUT_12V_ERR						0x0505U /* 输出12V故障*/
#define			PLC_MAIN_CONNECT_ERR	  	0x0506U /* 主接触器连接故障*/
#define			PLC_DRIVE2_CONNECT_ERR		0x0507U	/* 电磁制动连接故障*/
#define			PLC_DRIVE3_CONNECT_ERR		0x0508U /* 驱动3连接故障*/
#define			PLC_DRIVE4_CONNECT_ERR		0x0509U /* 驱动4连接故障*/
#define			PLC_MAIN_WELDED_ERR				0x050AU /* 主接触器触点熔接*/
#define			PLC_MAIN_DRIVE_ERR				0x050BU /* 主接触器驱动故障*/
#define     ICAN_MACIDCHECK_ERR				0x050CU	/* MACID检测失败*/
#define     ICAN_CONNECT_ERR					0x050DU	/* ICAN连接失败*/										
#define     PLC_IOLOGIC_ERR					  0x050EU	/* io POWER ON LOGIC ERRROR*/										
#define     PLC_POT_SHORTCIRCUITS_ERR	0x050FU  /*电位计短路*/

/*******************************************************************************
* 3. Kernel
*******************************************************************************/
#define 		SL_POSCMD_OV 						0x0600U	/* 位置控制模式时指令溢出 */
#define 		SL_SPDCMD_OV						0x0601U	/* 速度控制模式时指令溢出 */
#define 		SL_TORCMD_OV	 					0x0602U	/* 转矩控制模式时指令溢出 */
#define 		SL_ASR_SATURATED 				0x0603U /* 速度环积分饱和 */
#define 		SL_POWER_RDY 						0x0604U	/* 电源板ready */
#define 		SL_KERNEL_ERR 					0x0605U /* Kernel运行错误 */
#define 		SL_RE_POWER_ON 					0x0606U	/* 须重新开机 */
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
#define 		SL_PARA_MODIFIED				0x0707U	/* 参数已被修改 */
#define 		SL_OVER_LOAD_ERR				0x0708U	
#define 		SL_ENCODER_UVW_ERR			0x0709U
#define 		SL_ENCODER_DIR_ERR			0x070AU
#define 		SL_ENCODER_COMM_ERR			0x070BU
#define 		PLC_EEPROM_RW_ERR				0x070CU	/* EEPROM读写参数错误*/
#define 		SL_EEPROM_BACK_ERR			0x070DU
#define 		PLC_PARA_OV_INDEX_ERR		0x070EU /* 参数编号错误*/
#define 		PLC_PARA_OV_LIMIT_ERR		0x070FU /* 参数超限错误*/
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
#define 		SL_SAVE_MODIFIED_PARA				0x0807U	/* 保存修改的参数 */
#define 		SL_SAVE_RIGID_RELATED_PARA	0x0808U	/* 刚性变更引起参数变化 */
#define 		SL_SAVE_MOTOR_RELATED_PARA	0x0809U	/* 电机变更引起参数变化 */
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
#define			SL_LOGIC_BUSY					0x0906U 	//PLC逻辑处理忙
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
* 函数定义
*******************************************************************************/
extern void ControlISR(void);//
extern void PLCISR(void);

#endif //#ifndef _KSDSYS_H_

