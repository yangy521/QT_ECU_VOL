/*******************************************************************************
* Filename: UserEcuProc.h	                                             	   *
* Description: 逻辑层的头文件，包含对应的回调函数	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/05/30    														   *
* Revision:	 														 		   *
*******************************************************************************/
#ifndef _USER_FORKLIFT_LIUGONG_PROC_H_
#define _USER_FORKLIFT_LIUGONG_PROC_H_

#include "PcuProc.h"
#include "MstSlvCom.h"
#include "DoPwm.h"
#include "PropDriver.h"
#include "LocalDi.h"
#include "NetTimer.h"
#include "Para.h"

/******************************************************************************
*函数定义
******************************************************************************/
#define test 

#define	USER_ECU_PERIOD	
#ifdef test
/*switch forward and series pump */
#define	LIFTUP_PUMP						DRIVER3
#define	TURNRIGHT_PUMP				DRIVER4
#define	TURNLEFT_PUMP					DRIVER5
#define	SPEAKER_PUMP					DRIVER6
#define	BLINK_LED							DRIVER7
#define	BACKWARD_PUMP					DRIVER8
#define	FORWARD_PUMP					DRIVER9
#define	BLINK_BEEP						DRIVER10




#define	SERIES_PARALLE_PUMP		DRIVER2

#else

#define	SPEAKER_PUMP			DRIVER6
#define	BACKWARD_PUMP			DRIVER8
#define	FORWARD_PUMP			DRIVER2
#define	TURNLEFT_PUMP			DRIVER5
#define	TURNRIGHT_PUMP			DRIVER4
#define	LIFTUP_PUMP				DRIVER3
#define	BLINK_LED				DRIVER7
#define	BLINK_BEEP				DRIVER10
#define	SERIES_PARALLE_PUMP		DRIVER9

#endif

#define	SPEAKER_PUMP_R			DRIVER6_R
#define	BACKWARD_PUMP_R			DRIVER8_R
#define	FORWARD_PUMP_R			DRIVER2_R
#define	TURNLEFT_PUMP_R			DRIVER5_R
#define	TURNRIGHT_PUMP_R		DRIVER4_R
#define	LIFTUP_PUMP_R			DRIVER3_R
#define	LIFTDOWN_PUMP_R			DRIVER12_R
#define	BLINK_LED_R				DRIVER7_R
#define	BLINK_BEEP_R			DRIVER10_R
#define	SERIES_PARALLE_PUMP_R	DRIVER9_R



#define	SPEAKER_PUMP_T			TIMER_Drive6Check
#define	BACKWARD_PUMP_T			TIMER_Drive8Check
#define	FORWARD_PUMP_T			TIMER_Drive2Check
#define	TURNLEFT_PUMP_T			TIMER_Drive5Check
#define	TURNRIGHT_PUMP_T		TIMER_Drive4Check
#define	LIFTUP_PUMP_T			TIMER_Drive3Check
#define	LIFTDOWN_PUMP_T			TIMER_Drive12Check
#define	BLINK_LED_T				TIMER_Drive7Check
#define	BLINK_BEEP_T			TIMER_Drive10Check
#define	SERIES_PARALLE_PUMP_T	TIMER_Drive9Check


#define	LIFTDOWN_PUMP	PropDriverCh1			/*PropPump*/

/*模拟量*/
#define ANGLE_SENSOR_CHANNEL		AI_B_AI1_R  /**/
#define PRESSURE_SENSOR_CHANNEL AI_B_AI2_R
//#define AI_B_AI3_R


#define	PCU_SWICTH			SWI1_R
#define	TILT_SIWTCH			SWI2_R
#define	PIT_SWITCH			SWI7_R
#define	UP_LIMIT_SWITCH		SWI5_R
#define	DOWN_LIMIT_SWITCH	SWI4_R	


#define	ECU_POWERON_DELAY_TIME	500
#define	ECU_ANTI_PINCH_TIME 	3000
#define	PUMP_CLOSE_PERCENTAGE	0
#define	PUMP_OPEN_PERCENTAGE	1

#define MOTOR_CLOESE_SPEED	10
#define	MOTOR_MIN_SPEED			200		/*怠速转速*/
#define MOTOR_STEER_SPEED		600
#define MOTOR_CHANGE_SPEED  300		/**/
#define	MOTOR_MAX_SPEED			2000
#define	MOTOR_MAX_SPEED_VALUE	4064


#define	PROP_PIT_VALUE			(_IQ(PROPD_MAX_CURRENT * 0.3 / PROPD_STD_CURRENT))

#define PROP_CURRENT_FACOTR		0.0016							/*0.0016 = (3.3 * 7.5 / 4096 / (7.5 + 68) / 0.05)*/
#define	PROP_MIN_CURRENT		PARA_PropDMinCurrent1
#define	PROP_MAX_CURRENT		PARA_PropDMaxCurrent1


#define	SWI1_EXTINPUT			0
#define	SWI2_EXTINPUT			1
#define	SWI3_EXTINPUT			2
#define	SWI4_EXTINPUT			3
#define	SWI5_EXTINPUT			4
#define	SWI6_EXTINPUT			5
#define	SWI7_EXTINPUT			6
#define	SWI8_EXTINPUT			7


#define	SLOW_KEY				0
#define	SPEAKER_KEY				1
#define	MOVE_KEY				2
#define	LIFT_KEY				3
#define	ENABLE_KEY				4
#define	LEFT_KEY				5
#define	RIGHT_KEY				6

#define	FORWARD_DISABLE_FLAG	(1 << 0)
#define	BACKWARD_DISABLE_FLAG	(1 << 1)
#define	LIFTUP_DISABLE_FLAG		(1 << 2)
#define	LIFTDOWN_DISABLE_FLAG	(1 << 3)
#define	TURNRIGHT_DISABLE_FLAG	(1 << 4)
#define	TURNLEFT_DISABLE_FLAG	(1 << 5)
#define	BARKE_DISABLE_FLAG		(1 << 6)
#define	PARALLE_DISABLE_FLAG	(1 << 7)

#define	PIT_DISABLE_FLAG		(1 << 12)
#define	TILT_DISABLE_FLAG		(1 << 13)
#define	DISABLE_PCU_FALG		(1 << 14)
#define	SPD_AFTER_LIFT_FALG		(1 << 15)		/*except for down*/

/*lilu 20230707 禁止前进阀的动作*/
#define	FORWARD_DISABLE_ACT
/*lilu 20230707 禁止后退阀的动作*/


#define	STEP_FACTOR					(5.0 * 4064 / 255)
#define TIME_FACTOR					20
#define FILTER_FACTOR				3 
#define	MOTOR_MAX_SPD				3000
#define	MOTOR_SWITCH_VALVE_SPD		300
#define	MOTOR_MIN_SPD_BY_CTRL		300


#if 0

#define SYSTEM_INIT_ERR										 	ErrCode81 /*系统初始化错误*/
#define SYSTEM_COMM_ERR										 	ErrCode82 /*系统通信错误*/
#define INVALID_OPT_SET_ERR		 							ErrCode83 /*无效选项设置错误*/
#define EERPOM_ERR												 	ErrCode84 /*闪存数据错误*/
#define LI_BATTERY_LOSS_ERR									ErrCode85 /*锂电池通讯丢失*/
#define LIFT_BUTTON_ERR 										ErrCode86 /*上电时举升按键按下*/
#define SLOW_BUTTON_ERR											ErrCode87 /*上电时龟速按键按下*/
#define MOVE_BUTTON_ERR 										ErrCode88 /*上电时行走按键按下*/
#define GPS_CONNECT_ERR 										ErrCode89 /*GPS连接错误*/
#define MAIN_CONNECT_ERR 										ErrCode90 /*主接触器故障*/


#define UPDOWN_BUTTON_ERR 									ErrCode92 /*启动时底盘上升或下降按钮打开错误*/
#define BMS_BATTERY_TEMP_DIFF2_ERR					ErrCode93 /*BMS-电池温差过大2*/
#define BMS_BATTERY_TEMP_HIGH1_ERR					ErrCode94 /*BMS-电池温度过高1*/
#define BMS_DISCHARGE_TEMP_HIGH2_ERR				ErrCode95 /*BMS-放电温度过高2*/
#define BMS_DISCHARGE_CUR_HIGH1_ERR					ErrCode96 /*BMS-放电电流过高1*/
#define BMS_DISCHARGE_CUR_HIGH2_ERR					ErrCode97 /*BMS-放电电流过高2*/
#define PIT_PROCETION_ERR										ErrCode98 /*坑洞保护错误*/
#define BMS_TOTAL_VOL_LOW1_ERR							ErrCode99 /*BMS-总电压过低1*/
#define BMS_TOTAL_VOL_LOW2_ERR							ErrCode100 /*BMS-总电压过低2*/
#define BMS_SINGLE_VOL_LOW1_ERR							ErrCode101 /*BMS-单体电压过低1*/
#define BMS_SINGLE_VOL_LOW2_ERR							ErrCode102 /*BMS-单体电压过低2*/



#define LOW_VALVE2_ERR											ErrCode107 /*下降阀2错误*/

#define BMS_BATTERY_VOL_DIFF_HIGH_ERR				ErrCode110 /*BMS-电池压差过大*/
#define PRESSURE_SENSOR_ERR									ErrCode111 /*压力传感器错误*/
#define ANGLE_SENSOR_ERR										ErrCode112 /*角度传感器错误*/
#define BATTERY_TYPE_ERR										ErrCode113 /*电池类型错误*/

#define WEIGHT_CALI_REVESER_ERR							ErrCode115 /*称重标定反*/
#define BATTERY_LOW_CAP1_ERR								ErrCode116 /*电池电量低一级报警*/

#define CALIBRATION_FAILURE_ERR							ErrCode118 /*未标定完成或标定失败*/
#define COMMUNICATION_ERR										ErrCode119 /*通信故障*/

#define PLATFORM_LEVEL1_LOCK_ERR						ErrCode121 /*平台一级锁车*/
#define PLAT_LEFT_BUTTON_ERR								ErrCode122 /*启动时，平台向左转向按钮按下错误*/
#define PLAT_RIGHT_BUTTON_ERR								ErrCode123 /*启动时，平台向右转向按钮按下错误*/
#define PLATFORM_LEVEL2_LOCK_ERR						ErrCode124 /*平台二级锁车*/

#define ENABLE_BUTTON_ERR										ErrCode126 /*启动时，平台手柄使能开关按钮按下错误*/
#define HANDLE_NOT_IN_MIDDLE_POSI_ERR				ErrCode127 /*启动时，平台手柄不在中位错误*/

#define FORWARD_VALVE_ERR										ErrCode132 /*前进阀错误*/
#define BACKWARD_VALVE_ERR									ErrCode133 /*后退阀错误*/
#define LIFT_UP_VALVE_ERR										ErrCode134 /*举升阀错误*/
#define LIFT_DOWN_VALVE_ERR									ErrCode135 /*下降阀错误*/
#define TURN_RIGHT_VALVE_ERR								ErrCode136 /*右转阀错误*/
#define TURN_LEFT_VALVE_ERR									ErrCode137 /*左转阀错误*/
#define BRAKE_VALVE_ERR											ErrCode138 /*刹车阀错误*/
#define PARALLEL_VALVE_ERR									ErrCode139 /*并联阀故障*/
#define CONTROLLER_ERR											ErrCode140 /*驱动器故障*/
#define CTRL_CURRENT_SENSOR_ERR							ErrCode141 /*驱动器电流传感器故障*/
#define CTRL_HARDWARE_ERR										ErrCode142 /*驱动器硬件损坏故障*/
#define PUMP_DRIVE_OPEN_ERR									ErrCode143 /*泵电驱开路故障*/
#define LEFT_DRIVE_OPEN_ERR									ErrCode144 /*左电驱开路故障*/
#define CONTROL_VOL_5V_ERR									ErrCode145 /*控制电压5V故障*/
#define LEFT_VALVE_OPEN_SHORT_ERR						ErrCode146 /*动作时，检测到左转阀开路或短路*/
#define CONTROL_VOL_12V_ERR									ErrCode147 /*控制电压12V故障*/
#define BAT_LOW_CAP2_ERR										ErrCode148 /*电池低电量二级报警*/
#define HIGH_ZERO_CURRENT_ERR								ErrCode149 /*高零位电流错误*/
#define CTRL_BUS_VOL_HIGH_ERR								ErrCode150 /*驱动器母线电压过高故障*/
#define PRE_CHARGE_FAULT_ERR								ErrCode151 /*预充故障*/
#define CTRL_BUS_VOL_LOW_ERR								ErrCode152 /*驱动器母线电压过低故障*/
#define CTRL_TEMP_LOW_ERR										ErrCode153 /*驱动器低温故障*/
#define CTRL_TEMP_HIGH1_ERR									ErrCode154 /*驱动器高温一级故障*/
#define PUMP_MOTOR_TEMP1_ERR								ErrCode155 /*泵电机温度一级故障*/
#define PUMP_MOTOR_ENCODER_ERR							ErrCode156 /*泵电机编码器故障*/
#define MOTOR_ENCODE_ERR										ErrCode157 /*电机编码错误*/
#define PUMP_MOTOR_OVER_CURRENT_ERR					ErrCode158 /*泵电机过流类故障*/
#define PUMP_MOTER_TEMP2_ERR								ErrCode159 /*泵电机温度二级故障*/
#define OVER_80_PER_LOAD_ERR								ErrCode160 /*超过 80%负载报警*/
#define CTRL_TEMP2_ERR											ErrCode161 /*驱动器温度二级故障*/
#define RIGHT_BRAKE_ERR											ErrCode162 /*右刹车故障*/
#define LEFT_BRAKE_ERR											ErrCode163 /*左刹车故障*/
#define PUMP_MOTOR_STALL_ERR								ErrCode164 /*泵电机堵转、失速*/
#define LEFT_DRIVE_MOTER_STALL_ERR					ErrCode165 /*左牵引电机堵转、失速*/
#define RIGTH_DRIVE_MOTOR_STALL_ERR					ErrCode166 /*右牵引电机堵转、失速*/


#define CTRL_DRIVE_LONG_RUN_ERR							ErrCode169 /*驱动器运行时间过长故障*/
#define OVER_90_PER_LOAD_ERR								ErrCode170 /*超过 90%负载报警*/
#define LEFT_MOTOR_OVER_CURRENT_ERR					ErrCode171/*左电机电流过流故障*/
#define RIGTH_MOTOOR_OVER_CURRENT_ERR				ErrCode172 /*右电机电流过流故障*/

#define OVER_99_PER_LOAD_ERR								ErrCode179 /*超过 99%负载报警*/
#define PLAT_OVERLOAD_ERR										ErrCode180 /*平台超载报警*/
#define MACHINE_TILT_OVER_SAFETY_ERR				ErrCode181 /*机器倾斜超过安全限定错误*/
#else


#define	SYSTEM_INIT_ERR					ErrCode101		/*系统初始化错误*/
#define	SYSTEM_COMM_ERR					ErrCode102		/*系统通信错误*/
#define	INVALID_OPT_SET_ERR				ErrCode103		/*无效选项设置错误*/
#define	EERPOM_ERR						ErrCode104		/*闪存数据错误*/
#define	LI_BATTERY_LOSS_ERR				ErrCode105		/*锂电池通讯丢失*/
#define	LIFT_BUTTON_ERR					ErrCode106		/*上电时举升按键按下*/
#define	SLOW_BUTTON_ERR					ErrCode107		/*上电时龟速按键按下*/
#define	MOVE_BUTTON_ERR					ErrCode108		/*上电时行走按键按下*/
#define	GPS_CONNECT_ERR					ErrCode109		/*GPS连接错误*/
#define	MAIN_CONNECT_ERR				ErrCode110		/*主接触器故障*/

#define	UPDOWN_BUTTON_ERR				ErrCode112		/*启动时底盘上升或下降按钮打开错误*/
#define	BMS_BATTERY_TEMP_DIFF2_ERR		ErrCode113		/*BMS-电池温差过大2*/
#define	BMS_BATTERY_TEMP_HIGH1_ERR		ErrCode114		/*BMS-电池温度过高1*/
#define	BMS_DISCHARGE_TEMP_HIGH2_ERR	ErrCode115		/*BMS-放电温度过高2*/
#define	BMS_DISCHARGE_CUR_HIGH1_ERR		ErrCode116		/*BMS-放电电流过高1*/
#define	BMS_DISCHARGE_CUR_HIGH2_ERR		ErrCode117		/*BMS-放电电流过高2*/
#define	PIT_PROCETION_ERR				ErrCode118		/*坑洞保护错误*/
#define	BMS_TOTAL_VOL_LOW1_ERR			ErrCode119		/*BMS-总电压过低1*/
#define	BMS_TOTAL_VOL_LOW2_ERR			ErrCode120		/*BMS-总电压过低2*/
#define	BMS_SINGLE_VOL_LOW1_ERR			ErrCode121		/*BMS-单体电压过低1*/
#define	BMS_SINGLE_VOL_LOW2_ERR			ErrCode122		/*BMS-单体电压过低2*/

#define	LOW_VALVE2_ERR					ErrCode127		/*下降阀2错误*/

#define	BMS_BATTERY_VOL_DIFF_HIGH_ERR	ErrCode130		/*BMS-电池压差过大*/
#define	PRESSURE_SENSOR_ERR				ErrCode131		/*压力传感器错误*/
#define ANGLE_SENSOR_ERR				ErrCode132		/*角度传感器错误*/
#define	BATTERY_TYPE_ERR				ErrCode133		/*电池类型错误*/

#define	WEIGHT_CALI_REVESER_ERR			ErrCode135		/*称重标定反*/
#define	BATTERY_LOW_CAP1_ERR			ErrCode136		/*电池电量低一级报警*/

#define	CALIBRATION_FAILURE_ERR			ErrCode138		/*未标定完成或标定失败*/
#define	COMMUNICATION_ERR				ErrCode139		/*通信故障*/

#define	PLATFORM_LEVEL1_LOCK_ERR		ErrCode141		/*平台一级锁车*/
#define	PLAT_LEFT_BUTTON_ERR			ErrCode142		/*启动时，平台向左转向按钮按下错误*/
#define	PLAT_RIGHT_BUTTON_ERR			ErrCode143		/*启动时，平台向右转向按钮按下错误*/
#define	PLATFORM_LEVEL2_LOCK_ERR		ErrCode144		/*平台二级锁车*/

#define	ENABLE_BUTTON_ERR				ErrCode146		/*启动时，平台手柄使能开关按钮按下错误*/
#define	HANDLE_NOT_IN_MIDDLE_POSI_ERR	ErrCode147		/*启动时，平台手柄不在中位错误*/

#define	FORWARD_VALVE_ERR				ErrCode152		/*前进阀错误*/
#define	BACKWARD_VALVE_ERR				ErrCode153		/*后退阀错误*/
#define LIFT_UP_VALVE_ERR				ErrCode154		/*举升阀错误*/	
#define	LIFT_DOWN_VALVE_ERR				ErrCode155		/*下降阀错误*/
#define	TURN_RIGHT_VALVE_ERR			ErrCode156		/*右转阀错误*/
#define	TURN_LEFT_VALVE_ERR				ErrCode157		/*左转阀错误*/
#define BRAKE_VALVE_ERR					ErrCode158		/*刹车阀错误*/
#define	PARALLEL_VALVE_ERR				ErrCode159		/*并联阀故障*/
#define	CONTROLLER_ERR					ErrCode160		/*驱动器故障*/
#define	CTRL_CURRENT_SENSOR_ERR			ErrCode161		/*驱动器电流传感器故障*/
#define	CTRL_HARDWARE_ERR				ErrCode162		/*驱动器硬件损坏故障*/
#define	PUMP_DRIVE_OPEN_ERR				ErrCode163		/*泵电驱开路故障*/
#define	LEFT_DRIVE_OPEN_ERR				ErrCode164		/*左电驱开路故障*/
#define	CONTROL_VOL_5V_ERR				ErrCode165		/*控制电压5V故障*/
#define	LEFT_VALVE_OPEN_SHORT_ERR		ErrCode166		/*动作时，检测到左转阀开路或短路*/
#define	CONTROL_VOL_12V_ERR				ErrCode167		/*控制电压12V故障*/
#define	BAT_LOW_CAP2_ERR				ErrCode168		/*电池低电量二级报警*/
#define	HIGH_ZERO_CURRENT_ERR			ErrCode169		/*高零位电流错误*/
#define	CTRL_BUS_VOL_HIGH_ERR			ErrCode170		/*驱动器母线电压过高故障*/
#define	PRE_CHARGE_FAULT_ERR			ErrCode171		/*预充故障*/
#define	CTRL_BUS_VOL_LOW_ERR			ErrCode172		/*驱动器母线电压过低故障*/
#define	CTRL_TEMP_LOW_ERR				ErrCode173		/*驱动器低温故障*/
#define	CTRL_TEMP_HIGH1_ERR				ErrCode174		/*驱动器高温一级故障*/
#define	PUMP_MOTOR_TEMP1_ERR			ErrCode175		/*泵电机温度一级故障*/
#define	PUMP_MOTOR_ENCODER_ERR			ErrCode176		/*泵电机编码器故障*/
#define	MOTOR_ENCODE_ERR				ErrCode177		/*电机编码错误*/
#define	PUMP_MOTOR_OVER_CURRENT_ERR		ErrCode178		/*泵电机过流类故障*/
#define	PUMP_MOTER_TEMP2_ERR			ErrCode179		/*泵电机温度二级故障*/
#define	OVER_80_PER_LOAD_ERR			ErrCode180		/*超过 80%负载报警*/
#define	CTRL_TEMP2_ERR					ErrCode181		/*驱动器温度二级故障*/
#define	RIGHT_BRAKE_ERR					ErrCode182		/*右刹车故障*/
#define	LEFT_BRAKE_ERR					ErrCode183		/*左刹车故障*/
#define	PUMP_MOTOR_STALL_ERR			ErrCode184		/*泵电机堵转、失速*/
#define	LEFT_DRIVE_MOTER_STALL_ERR		ErrCode185		/*左牵引电机堵转、失速*/
#define	RIGTH_DRIVE_MOTOR_STALL_ERR		ErrCode186		/*右牵引电机堵转、失速*/

#define	CTRL_DRIVE_LONG_RUN_ERR			ErrCode189		/*驱动器运行时间过长故障*/
#define	OVER_90_PER_LOAD_ERR			ErrCode190		/*超过 90%负载报警*/
#define	LEFT_MOTOR_OVER_CURRENT_ERR		ErrCode191		/*左电机电流过流故障*/
#define RIGTH_MOTOOR_OVER_CURRENT_ERR	ErrCode192		/*右电机电流过流故障*/

#define	OVER_99_PER_LOAD_ERR			ErrCode199		/*超过 99%负载报警*/
#define	PLAT_OVERLOAD_ERR				ErrCode200		/*平台超载报警*/
#define	MACHINE_TILT_OVER_SAFETY_ERR	ErrCode201		/*机器倾斜超过安全限定错误*/

#endif

extern void vUserEcuInit(void);
extern void vUserEcuProc(void);


//试验台测试


#endif //#ifndef _USER_COMM_H_
