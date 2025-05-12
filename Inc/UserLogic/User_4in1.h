/*******************************************************************************
* Filename: UserEcuProc.h	                                             	   *
* Description: 逻辑层的头文件，包含对应的回调函数	   							   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/05/30    														   *
* Revision:	 														 		   *
*******************************************************************************/
#ifndef _USER_ECU_PROC_H_
#define _USER_ECU_PROC_H_

#include "PcuProc.h"
#include "MstSlvCom.h"
#include "DoPwm.h"
#include "PropDriver.h"
#include "LocalDi.h"
#include "NetTimer.h"

/******************************************************************************
*函数定义
******************************************************************************/


#define	USER_ECU_PERIOD		TIMER_PLC_PERIOD

#define TIMER_PDO 

#define MAIN_CONTACTOR    DRIVER1
#define BRAKECOIL_1				DRIVER2
#define BRAKECOIL_2				DRIVER3


#define	TURNRIGHT_PUMP		DRIVER4
#define	TURNLEFT_PUMP			DRIVER5
#define	SPEAKER_PUMP			DRIVER6
#define	BLINK_LED					DRIVER7
#define	LIFTUP_PUMP				DRIVER8
//#define	SERIES_PARALLE_PUMP		DRIVER9
#define	BLINK_BEEP				DRIVER10






#define	SPEAKER_PUMP_R				DRIVER6_R
#define	BACKWARD_PUMP_R				DRIVER8_R
#define	FORWARD_PUMP_R				DRIVER2_R
#define	TURNLEFT_PUMP_R				DRIVER4_R
#define	TURNRIGHT_PUMP_R			DRIVER5_R
#define	LIFTUP_PUMP_R					DRIVER3_R
#define	LIFTDOWN_PUMP_R				DRIVER12_R
#define	BLINK_LED_R						DRIVER7_R
#define	BLINK_BEEP_R					DRIVER10_R
#define	SERIES_PARALLE_PUMP_R	DRIVER9_R



#define	SPEAKER_PUMP_T				TIMER_Drive6Check
#define	BACKWARD_PUMP_T				TIMER_Drive8Check
#define	FORWARD_PUMP_T				TIMER_Drive2Check
#define	TURNLEFT_PUMP_T				TIMER_Drive4Check
#define	TURNRIGHT_PUMP_T			TIMER_Drive5Check
#define	LIFTUP_PUMP_T					TIMER_Drive3Check
#define	LIFTDOWN_PUMP_T				TIMER_Drive12Check
#define	BLINK_LED_T						TIMER_Drive7Check
#define	BLINK_BEEP_T					TIMER_Drive10Check
#define	SERIES_PARALLE_PUMP_T	TIMER_Drive9Check


#define	LIFTDOWN_PUMP_1				PropDriverCh1			/*PropPump*/
#define	LIFTDOWN_PUMP_2				PropDriverCh0			// to be update 

#define	PCU_SWICTH												SWI1_R
#define	TILT_SIWTCH												SWI2_R
#define	LOWERCONTROL_LIFT_SIWTCH					SWI3_R						
#define	PEDAL_SIWTCH											SWI4_R
#define	PIT_SWITCH												SWI5_R
#define	LOWERCONTROL_DOWN_SWITCH					SWI6_R				
#define	UP_LIMIT_SWITCH										SWI7_R
#define	DOWN_LIMIT_SWITCH									SWI8_R	



#define	ECU_POWERON_DELAY_TIME	500
#define	ECU_ANTI_PINCH_TIME 	2500
#define	PUMP_CLOSE_PERCENTAGE	0
#define	PUMP_OPEN_PERCENTAGE	80

#define	MOTOR_MAX_SPEED			3045
#define	MOTOR_MAX_SPEED_VALUE	4064

/*反馈关闭值*/
#define MOTOR_FDB_CLOSE_VALUE 30
#define	PUMP_FDB_CLOSE_VALUE 6
#define PROP_FDB_CLOSE_VALUE 100


#define	PCU_STATE			0
#define	TILT_STATE			1
#define	LIFT_STATE			2
#define	PEDAL_STATE			3
#define	PIT_STATE			4
#define	DOWN_STATE			5
#define	UP_LIMIT_STATE			6
#define	DOWN_LIMIT_STATE			7


#define	SLOW_KEY				0
#define	SPEAKER_KEY				1
#define	MOVE_KEY				2
#define	LIFT_KEY				3
#define	ENABLE_KEY				4
#define	LEFT_KEY				5
#define	RIGHT_KEY				6

#define	FORWARD_DISABLE_FLAG		(1 << 0)
#define	BACKWARD_DISABLE_FLAG		(1 << 1)
#define	LIFTUP_DISABLE_FLAG			(1 << 2)
#define	LIFTDOWN_DISABLE_FLAG		(1 << 3)
#define	TURNRIGHT_DISABLE_FLAG	(1 << 4)
#define	TURNLEFT_DISABLE_FLAG		(1 << 5)
#define PCU_NOACT_FLAG					(1 << 6)
#define NO_ACT_FLAG							(1 << 7)
#define LIFT_SPEED_FLAG					(1 << 8)

/*下降动作标志位*/
#define NO_DOWN_ACT							1
#define NORMAL_DOWN_ACT					2
#define ANTIPINCH_STATE					3
#define WAITNG_STATE						4

/**/
#define STATE_MODE (1<<0)




#define	SYSTEM_INIT_ERR								ErrCode1		/*系统初始化错误*/
#define	SYSTEM_COMM_ERR								ErrCode2		/*系统通信错误*/
#define	INVALID_OPT_SET_ERR						ErrCode3		/*无效选项设置错误*/
#define	EERPOM_ERR										ErrCode4		/*闪存数据错误*/
#define	LI_BATTERY_LOSS_ERR						ErrCode5		/*锂电池通讯丢失*/
#define	LIFT_BUTTON_ERR								ErrCode6		/*上电时举升按键按下*/
#define	SLOW_BUTTON_ERR								ErrCode7		/*上电时龟速按键按下*/
#define	MOVE_BUTTON_ERR								ErrCode8		/*上电时行走按键按下*/
#define	GPS_CONNECT_ERR								ErrCode9		/*GPS连接错误*/
#define	MAIN_CONNECT_ERR							ErrCode10		/*主接触器故障*/

#define	UPDOWN_BUTTON_ERR							ErrCode12		/*启动时底盘上升或下降按钮打开错误*/
#define	BMS_BATTERY_TEMP_DIFF2_ERR		ErrCode13		/*BMS-电池温差过大2*/
#define	BMS_BATTERY_TEMP_HIGH1_ERR		ErrCode14		/*BMS-电池温度过高1*/
#define	BMS_DISCHARGE_TEMP_HIGH2_ERR	ErrCode15		/*BMS-放电温度过高2*/
#define	BMS_DISCHARGE_CUR_HIGH1_ERR		ErrCode16		/*BMS-放电电流过高1*/
#define	BMS_DISCHARGE_CUR_HIGH2_ERR		ErrCode17		/*BMS-放电电流过高2*/
#define	PIT_PROCETION_ERR							ErrCode18		/*坑洞保护错误*/
#define	BMS_TOTAL_VOL_LOW1_ERR				ErrCode19		/*BMS-总电压过低1*/
#define	BMS_TOTAL_VOL_LOW2_ERR				ErrCode20		/*BMS-总电压过低2*/
#define	BMS_SINGLE_VOL_LOW1_ERR				ErrCode21		/*BMS-单体电压过低1*/
#define	BMS_SINGLE_VOL_LOW2_ERR				ErrCode22		/*BMS-单体电压过低2*/



#define	LOW_VALVE2_ERR								ErrCode27		/*下降阀2错误*/


#define	BMS_BATTERY_VOL_DIFF_HIGH_ERR	ErrCode30		/*BMS-电池压差过大*/
#define	PRESSURE_SENSOR_ERR						ErrCode31		/*压力传感器错误*/
#define ANGLE_SENSOR_ERR							ErrCode32		/*角度传感器错误*/
#define	BATTERY_TYPE_ERR							ErrCode33		/*电池类型错误*/

#define	WEIGHT_CALI_REVESER_ERR				ErrCode35		/*称重标定反*/
#define	BATTERY_LOW_CAP1_ERR					ErrCode36		/*电池电量低一级报警*/

#define	CALIBRATION_FAILURE_ERR				ErrCode38		/*未标定完成或标定失败*/
#define	COMMUNICATION_ERR							ErrCode39		/*通信故障*/

#define	PLATFORM_LEVEL1_LOCK_ERR			ErrCode41		/*平台一级锁车*/
#define	PLAT_LEFT_BUTTON_ERR					ErrCode42		/*启动时，平台向左转向按钮按下错误*/
#define	PLAT_RIGHT_BUTTON_ERR					ErrCode43		/*启动时，平台向右转向按钮按下错误*/
#define	PLATFORM_LEVEL2_LOCK_ERR			ErrCode44		/*平台二级锁车*/

#define	ENABLE_BUTTON_ERR							ErrCode46		/*启动时，平台手柄使能开关按钮按下错误*/
#define	HANDLE_NOT_IN_MIDDLE_POSI_ERR	ErrCode47		/*启动时，平台手柄不在中位错误*/

#define	FORWARD_VALVE_ERR							ErrCode52		/*前进阀错误*/
#define	BACKWARD_VALVE_ERR						ErrCode53		/*后退阀错误*/
#define LIFT_UP_VALVE_ERR							ErrCode54		/*举升阀错误*/	
#define	LIFT_DOWN_VALVE_ERR						ErrCode55		/*下降阀错误*/
#define	TURN_RIGHT_VALVE_ERR					ErrCode56		/*右转阀错误*/
#define	TURN_LEFT_VALVE_ERR						ErrCode57		/*左转阀错误*/
#define BRAKE_VALVE_ERR								ErrCode58		/*刹车阀错误*/
#define	PARALLEL_VALVE_ERR						ErrCode59		/*并联阀故障*/
#define	CONTROLLER_ERR								ErrCode60		/*驱动器故障*/
#define	CTRL_CURRENT_SENSOR_ERR				ErrCode61		/*驱动器电流传感器故障*/
#define	CTRL_HARDWARE_ERR							ErrCode62		/*驱动器硬件损坏故障*/
#define	PUMP_DRIVE_OPEN_ERR						ErrCode63		/*泵电驱开路故障*/
#define	LEFT_DRIVE_OPEN_ERR						ErrCode64		/*左电驱开路故障*/
#define	CONTROL_VOL_5V_ERR						ErrCode65		/*控制电压5V故障*/
#define	LEFT_VALVE_OPEN_SHORT_ERR			ErrCode66		/*动作时，检测到左转阀开路或短路*/
#define	CONTROL_VOL_12V_ERR						ErrCode67		/*控制电压12V故障*/
#define	BAT_LOW_CAP2_ERR							ErrCode68		/*电池低电量二级报警*/
#define	HIGH_ZERO_CURRENT_ERR					ErrCode69		/*高零位电流错误*/
#define	CTRL_BUS_VOL_HIGH_ERR					ErrCode70		/*驱动器母线电压过高故障*/
#define	PRE_CHARGE_FAULT_ERR					ErrCode71		/*预充故障*/
#define	CTRL_BUS_VOL_LOW_ERR					ErrCode72		/*驱动器母线电压过低故障*/
#define	CTRL_TEMP_LOW_ERR							ErrCode73		/*驱动器低温故障*/
#define	CTRL_TEMP_HIGH1_ERR						ErrCode74		/*驱动器高温一级故障*/
#define	PUMP_MOTOR_TEMP1_ERR					ErrCode75		/*泵电机温度一级故障*/
#define	PUMP_MOTOR_ENCODER_ERR				ErrCode76		/*泵电机编码器故障*/
#define	MOTOR_ENCODE_ERR							ErrCode77		/*电机编码错误*/
#define	PUMP_MOTOR_OVER_CURRENT_ERR		ErrCode78		/*泵电机过流类故障*/
#define	PUMP_MOTER_TEMP2_ERR					ErrCode79		/*泵电机温度二级故障*/
#define	OVER_80_PER_LOAD_ERR					ErrCode80		/*超过 80%负载报警*/
#define	CTRL_TEMP2_ERR								ErrCode81		/*驱动器温度二级故障*/
#define	RIGHT_BRAKE_ERR								ErrCode82		/*右刹车故障*/
#define	LEFT_BRAKE_ERR								ErrCode83		/*左刹车故障*/
#define	PUMP_MOTOR_STALL_ERR					ErrCode84		/*泵电机堵转、失速*/
#define	LEFT_DRIVE_MOTER_STALL_ERR		ErrCode85		/*左牵引电机堵转、失速*/
#define	RIGTH_DRIVE_MOTOR_STALL_ERR		ErrCode86		/*右牵引电机堵转、失速*/

#define	CTRL_DRIVE_LONG_RUN_ERR				ErrCode89		/*驱动器运行时间过长故障*/
#define	OVER_90_PER_LOAD_ERR					ErrCode90		/*超过 90%负载报警*/
#define	LEFT_MOTOR_OVER_CURRENT_ERR		ErrCode91		/*左电机电流过流故障*/
#define RIGTH_MOTOOR_OVER_CURRENT_ERR	ErrCode92		/*右电机电流过流故障*/



#define	OVER_99_PER_LOAD_ERR					ErrCode99		/*超过 99%负载报警*/
#define	PLAT_OVERLOAD_ERR							ErrCode100		/*平台超载报警*/
#define	MACHINE_TILT_OVER_SAFETY_ERR	ErrCode101		/*机器倾斜超过安全限定错误*/

extern void vUserEcuInit(void);
extern void vUserEcuProc(void);


#endif //#ifndef _USER_COMM_H_
