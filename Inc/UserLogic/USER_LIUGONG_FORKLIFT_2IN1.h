/*******************************************************************************
* Filename: UserEcuProc.h	                                             	   *
* Description: 逻辑层的头文件，包含对应的回调函数	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/05/30    														   *
* Revision:	 														 		   *
*******************************************************************************/
#ifndef _USER_ECU_PROC_H_
#define _USER_ECU_PROC_H_


#if (USER_TYPE == USER_FORKLIFT_LIUGONG| USER_TYPE == USER_FORKLIFT_LIUGONG_TEST)
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
//#define test
#define	USER_ECU_PERIOD		TIMER_PLC_PERIOD
#define PCU_DISPLAY_PERIOD 150

#define H9_INIT							0	
#define	H9_HIG16CHOOSE			1
#define H9_LOW16CHOOSE			2
#define H9_SAVE							3
#define H9_SAVEFAIL					4
#define H9_DOWNLIMIT				5
#define H9_UPLIMIT					6
#define H9_SAVE_SUCCESS			7
#define H9_VERIFY						8
#define H9_LOWLIMIT_CALI 		9
#define H9_LOWLIMITMOVE_EN 	10


#define H2_INITIAL 			0
#define H2_DISPLAY_NUM	1
#define H2_DISPLAY_KEY	2
#define H2_HEART_SET		3
#define H2_SAVED				4
#define H2_HEART_CHECK_FAIL 5

/*switch forward and series pump */

#define	LIFTUP_PUMP						DRIVER3
#define	TURNRIGHT_PUMP				DRIVER4
#define	TURNLEFT_PUMP					DRIVER5

#define	BLINK_LED							DRIVER7
#define	BACKWARD_PUMP					DRIVER8
#define HIGH_SPEED_PUMP				DRIVER9
#define	BLINK_BEEP						DRIVER10

#ifdef test
#define	SPEAKER_PUMP					DRIVER2
#define	FORWARD_PUMP					DRIVER6
#else
#define	SPEAKER_PUMP					DRIVER6
#define	FORWARD_PUMP					DRIVER2
#endif

#define	SPEAKER_PUMP_R			DRIVER6_R
#define	BACKWARD_PUMP_R			DRIVER8_R
#define	FORWARD_PUMP_R			DRIVER2_R
#define	TURNLEFT_PUMP_R			DRIVER5_R
#define	TURNRIGHT_PUMP_R		DRIVER4_R
#define	LIFTUP_PUMP_R					DRIVER3_R
#define	LIFTDOWN_PUMP_R				DRIVER12_R
#define	BLINK_LED_R						DRIVER7_R
#define	BLINK_BEEP_R					DRIVER10_R
#define	HIGH_SPEED_PUMP_R			DRIVER9_R


//柳工0507，用比例阀0做驱动输出
#define	LIFTDOWN_PUMP1	PropDriverCh0			/*PropPump*/
#define LIFTDOWN_PUMP2	PropDriverCh1

/*模拟量*/
#define ANGLE_SENSOR_CHANNEL		 AI_B_AI1_R  /**/
#define PRESSURE_SENSOR_CHANNEL  AI_B_AI2_R
#define PRESSURE_SENSOR_CHANNEL2 AI_B_AI3_R
//#define AI_B_AI3_R


#define	PCU_SWICTH			  SWI1_R
#define	TILT_SIWTCH			  SWI2_R
#define	UP_LIMIT_SWITCH		SWI7_R
#define	DOWN_LIMIT_SWITCH	SWI8_R	
#define	PIT_SWITCH			  SWI5_R

//待确认
#define LOWER_CONTROLL_UP	SWI4_R
#define LOWER_CONTROLL_DOWN	SWI3_R

#define ANTICOLLISION_SWITCH	SWI6_R


#define	ECU_POWERON_DELAY_TIME	500
#define SWITCH_CHECK_TIME				2500
#define	ECU_ANTI_PINCH_TIME 		3000

#define ALARM_TIME							5000


#define	PUMP_CLOSE_PERCENTAGE	0
#define	PUMP_OPEN_PERCENTAGE	1

#define MOTOR_CLOESE_SPEED	10
#define	MOTOR_MIN_SPEED			200		/*怠速转速*/
#define MOTOR_STEER_SPEED		600
#define MOTOR_CHANGE_SPEED  300		/**/
#define	MOTOR_MAX_SPEED			2000
#define	MOTOR_MAX_SPEED_VALUE	4064

#define UP_CONTROL_MODE	1
#define LOWER_CONTROL_MODE	0


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
#define	SPEAKER_KEY			1
#define	MOVE_KEY				2
#define	LIFT_KEY				3
#define	ENABLE_KEY			4
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
#define TIME_FACTOR					10 
#define FILTER_FACTOR				3 
#define	MOTOR_MAX_SPD				3000
#define	MOTOR_SWITCH_VALVE_SPD		300
#define	MOTOR_MIN_SPD_BY_CTRL		300

#define PressureCali_Times	20
#define CALIBRATION_INIT_TIME		3000
#define CALIBRATION_DELAY_TIME	3000
#define CALIBRATION_PROC_TIME		2000
#define MOTOR_SWITCH_



#define	SYSTEM_INIT_ERR							ErrCode101		/*系统初始化错误*/
#define	SYSTEM_COMM_ERR							ErrCode102		/*系统通信错误*/
#define	INVALID_OPT_SET_ERR					ErrCode103		/*无效选项设置错误*/
//#define	EERPOM_ERR							ErrCode104		/*闪存数据错误*/

#define	LIFT_BUTTON_ERR							ErrCode106		/*上电时举升按键按下*/
#define	SLOW_BUTTON_ERR							ErrCode107		/*上电时龟速按键按下*/
#define	MOVE_BUTTON_ERR							ErrCode108		/*上电时行走按键按下*/

#define PLATE_LOCK_LEVEL1_ALARMING	ErrCode109		/*平台一级锁车预警*/
#define PLATE_LOCK_LEVEL2_ALARMING	ErrCode110		/*平台二级锁车预警*/
#define HEART_LOCK_ALARMING					ErrCode111		/*平台心跳级锁车预警*/

//#define	GPS_CONNECT_ERR							ErrCode109		/*GPS连接错误*/
//#define	MAIN_CONNECT_ERR						ErrCode110		/*主接触器故障*/

#define	UPDOWN_BUTTON_ERR							ErrCode112		/*启动时底盘上升或下降按钮打开错误*/
#define	PIT_PROCETION_ERR							ErrCode118		/*坑洞保护错误*/

#define BMS_OFFLINE										ErrCode120		/*BMS通讯故障*/					/*柳工故障*/
#define	BMS_BATTERY_TEMP_HIGH1_ERR		ErrCode121		/*BMS-电池温度过高1*/
#define	BMS_DISCHARGE_CUR_HIGH1_ERR		ErrCode122		/*BMS-放电电流过高1*/
#define	BMS_TOTAL_VOL_LOW1_ERR				ErrCode123		/*BMS-总电压过低1*/
#define	BMS_SINGLE_VOL_LOW1_ERR				ErrCode124		/*BMS-单体电压过低1*/
#define	BMS_SINGLE_VOL_LOW2_ERR				ErrCode125		/*BMS-单体电压过低2*/
#define	BMS_BATTERY_VOL_DIFF_HIGH_ERR	ErrCode136		/*BMS-电池压差过大*/


#define	BMS_BATTERY_TEMP_DIFF2_ERR		ErrCode127		/*BMS-电池温差过大2*/
#define	BMS_DISCHARGE_CUR_HIGH2_ERR		ErrCode128		/*BMS-放电电流过高2*/
#define	BMS_DISCHARGE_TEMP_HIGH2_ERR	ErrCode129		/*BMS-放电温度过高2*/




#define	PRESSURE_SENSOR_ERR						ErrCode131		/*压力传感器错误*/
#define ANGLE_SENSOR_ERR							ErrCode132		/*角度传感器错误*/

#define	PCU_BUTT_ERR									ErrCode133		/*PCU按键故障*/	/*威卡故障，协议？*/

#define	WEIGHT_CALI_REVESER_ERR			ErrCode135		/*称重标定反*/
#define	BATTERY_LOW_CAP1_ERR			ErrCode136		/*电池电量低一级报警*/

#define	CALIBRATION_FAILURE_ERR			ErrCode138		/*未标定完成或标定失败*/

#define	PLATFORM_LEVEL1_LOCK_ERR		ErrCode141		/*平台一级锁车*/

#define	PLAT_LEFT_BUTTON_ERR			ErrCode142		/*启动时，平台向左转向按钮按下错误*/
#define	PLAT_RIGHT_BUTTON_ERR			ErrCode143		/*启动时，平台向右转向按钮按下错误*/
#define	PLATFORM_LEVEL2_LOCK_ERR		ErrCode145		/*平台二级锁车*/

#define	ENABLE_BUTTON_ERR				ErrCode146		/*启动时，平台手柄使能开关按钮按下错误*/
#define	HANDLE_NOT_IN_MIDDLE_POSI_ERR	ErrCode147		/*启动时，平台手柄不在中位错误*/
#define	HEART_BEAT_LOCK_ERR				ErrCode149				/*心跳锁车*/

#define	FORWARD_VALVE_ERR					ErrCode152		/*前进阀错误*/
#define	BACKWARD_VALVE_ERR				ErrCode153		/*后退阀错误*/
#define LIFT_UP_VALVE_ERR					ErrCode154		/*举升阀错误*/	
#define	LIFT_DOWN_VALVE_ERR				ErrCode155		/*下降阀错误*/
#define	TURN_RIGHT_VALVE_ERR			ErrCode156		/*右转阀错误*/
#define	TURN_LEFT_VALVE_ERR				ErrCode157		/*左转阀错误*/
#define	PARALLEL_VALVE_ERR				ErrCode159		/*并联阀故障*/


//#define	CONTROLLER_ERR					ErrCode160		/*驱动器故障*/
//#define	CTRL_CURRENT_SENSOR_ERR			ErrCode161		/*驱动器电流传感器故障*/
//#define	CTRL_HARDWARE_ERR				ErrCode162		/*驱动器硬件损坏故障*/
//#define	PUMP_DRIVE_OPEN_ERR				ErrCode163		/*泵电驱开路故障*/
//#define	LEFT_DRIVE_OPEN_ERR				ErrCode164		/*左电驱开路故障*/
//#define	CONTROL_VOL_5V_ERR				ErrCode165		/*控制电压5V故障*/
//#define	LEFT_VALVE_OPEN_SHORT_ERR		ErrCode166		/*动作时，检测到左转阀开路或短路*/
//#define	CONTROL_VOL_12V_ERR				ErrCode167		/*控制电压12V故障*/
#define	BAT_LOW_CAP2_ERR				ErrCode168		/*电池低电量二级报警*/
//#define	HIGH_ZERO_CURRENT_ERR			ErrCode169		/*高零位电流错误*/
//#define	CTRL_BUS_VOL_HIGH_ERR			ErrCode170		/*驱动器母线电压过高故障*/
//#define	PRE_CHARGE_FAULT_ERR			ErrCode171		/*预充故障*/
//#define	CTRL_BUS_VOL_LOW_ERR			ErrCode172		/*驱动器母线电压过低故障*/
//#define	CTRL_TEMP_LOW_ERR				ErrCode173		/*驱动器低温故障*/
//#define	CTRL_TEMP_HIGH1_ERR				ErrCode174		/*驱动器高温一级故障*/
//#define	PUMP_MOTOR_TEMP1_ERR			ErrCode175		/*泵电机温度一级故障*/
//#define	PUMP_MOTOR_ENCODER_ERR			ErrCode176		/*泵电机编码器故障*/
//#define	MOTOR_ENCODE_ERR				ErrCode177		/*电机编码错误*/
//#define	PUMP_MOTOR_OVER_CURRENT_ERR		ErrCode178		/*泵电机过流类故障*/
//#define	PUMP_MOTER_TEMP2_ERR			ErrCode179		/*泵电机温度二级故障*/
#define	OVER_80_PER_LOAD_ERR			ErrCode180		/*超过 80%负载报警*/

//#define	CTRL_TEMP2_ERR					ErrCode181		/*驱动器温度二级故障*/

//#define	RIGHT_BRAKE_ERR					ErrCode182		/*右刹车故障*/
//#define	LEFT_BRAKE_ERR						ErrCode183		/*左刹车故障*/
//#define	PUMP_MOTOR_STALL_ERR			ErrCode184		/*泵电机堵转、失速*/
#define	LEFT_DRIVE_MOTER_STALL_ERR		ErrCode185		/*左牵引电机堵转、失速*/
//#define	RIGTH_DRIVE_MOTOR_STALL_ERR		ErrCode186		/*右牵引电机堵转、失速*/

//#define	CTRL_DRIVE_LONG_RUN_ERR			ErrCode189		/*驱动器运行时间过长故障*/
#define	OVER_90_PER_LOAD_ERR			ErrCode190		/*超过 90%负载报警*/
//#define	LEFT_MOTOR_OVER_CURRENT_ERR		ErrCode191		/*左电机电流过流故障*/
//#define RIGTH_MOTOOR_OVER_CURRENT_ERR	ErrCode192		/*右电机电流过流故障*/

#define	OVER_99_PER_LOAD_ERR			ErrCode199		/*超过 99%负载报警*/
#define	PLAT_OVERLOAD_ERR				ErrCode200		/*平台超载报警*/
#define	MACHINE_TILT_OVER_SAFETY_ERR	ErrCode201		/*机器倾斜超过安全限定错误*/
#define MACHINE_ANTIPUNCH_ERR				ErrCode202		/*防止碰撞报警*/
//#define PCU_OUT_ERROR_O1		ErrCode203		/*手柄芯一个输出接口断开*/
//#define PCU_OUT_ERROR_O2		ErrCode204		/*手柄芯一个输出接口断开*/
//#define PCU_OUT_ERROR_LEVEL2		ErrCode205		/*手柄芯两个输出接口断开*/
//#define HEARTBEAT_ALARMING			ErrCode206		/*收到锁车报警*/
//#define PCU_CONFLICT_KEY_ERR			ErrCode207		/*冲突按键报警*/


#define	TIMER_SwitchCheck	TIMER_USER_1
#define TIMER_Calibration	TIMER_USER_2
#define	TIMER_AlarmDelay	TIMER_USER_3
//#define TIMER_PCUSLEEP		TIMER_USER_4
#define Timer_OneSecond			TIMER_USER_4

#define PARA_USERSETS					PARA_ValveType

#define UPDOWNCOUNT_ADDR      PARA_AngleValue0 //测试上升下降计数值代码
#define CARCODE_HISTORY				PARA_AngleValue1 //历史车辆代码

#define	PARA_LOWLIMIT_RANGE		PARA_AngleValue2
#define PARA_LOWLIMIT_ANGLE		PARA_AngleValue3
#define PARA_CARCODE					PARA_AngleValue4
#define PARA_HANDLEMAX				PARA_AngleValue5
#define PARA_HANDLEMID				PARA_AngleValue6
#define PARA_HANDLEMIN				PARA_AngleValue7

#define	LIFTTIMES_ADDR_L16				PARA_USER_DATA_01
#define LIFTTIMES_ADDR_H16				PARA_USER_DATA_02
#define STEERTIMES_ADDR_L16				PARA_USER_DATA_03
#define STEERTIMES_ADDR_H16				PARA_USER_DATA_04
#define MOVETIES_ADDR_L16					PARA_USER_DATA_05
#define MOVETIES_ADDR_H16					PARA_USER_DATA_06
#define OVERLOADTIMES_ADDR_L16		PARA_USER_DATA_07
#define OVERLOADTIMES_ADDR_H16		PARA_USER_DATA_08

#define LIFTHOUR_ADDR_L16					PARA_USER_DATA_09
#define LIFTHOUR_ADDR_H16					PARA_USER_DATA_10
#define STEERHOUR_ADDR_L16				PARA_USER_DATA_11
#define	STEERHOUR_ADDR_H16				PARA_USER_DATA_12
#define	LOWERMOVEHOUR_ADDR_L16		PARA_USER_DATA_13
#define LOWERMOVEHOUR_ADDR_H16		PARA_USER_DATA_14
#define UPMOVEHOUR_ADDR_L16				PARA_USER_DATA_15
#define UPMOVEHOUR_ADDR_H16				PARA_USER_DATA_16
#define DOWNHOUR_ADDR_L16					PARA_USER_DATA_17
#define DOWNHOUR_ADDR_H16					PARA_USER_DATA_18
#define TEMP_UNLOCK_HOUR_ADDR_16	PARA_USER_DATA_19
#define UPDATE_COUNT_ADDR_L16			PARA_USER_DATA_20
#define RANDOM_SEED_ADDR					PARA_USER_DATA_21
#define REMOTE_LOCK_STATE					PARA_USER_DATA_22
#define SOFTWARE_ADDR							PARA_USER_DATA_24


#define SOFTWARE_VERSION_STATE 		PARA_DefaultFlag

extern void vUserEcuInit(void);
extern void vUserEcuProc(void);

/*产品序列号*/
//#define ECU_SERIALNUMBER_L32 "ABCDEFGH24010501"

#define ECU_SERIALNUMBER_L32 "QTECULTS24030601"

#define PCU_SERIALNUMBER_L32 "LSDPCUL24030601"


#define SOFTWARE_VERSION		"LSQTCNAA24052200"//正式版为00

#define VERSION_CODE_HMI        240522   //仪表显示软件版本时间,同步更新

#define SOFTWARE_VERSION_CODE		0x132  //@@@@@出一次加一
//0x123 为改变TBOX心跳报警时间为3min
//0x124 倾角开关与坑洞开关关联
//0x125 修复坡道溜车与转向会与前进同时动作问题
//0x126 二级锁车与一级锁车相互覆盖。
//0x127 修改PS模式下调整防夹手速度和最低范围的操作逻辑。ECU批量改参数已验证
//0x128 添加了修改车型时批量修改MCU参数。（已验证）保存成功报39号警。
//0x129 修复了部分BUG，优化了H9模式下保存参数。
//0x130 H9模式下低16位高字节Bit0改为讯响器同步，5M参数角度模拟固化参数下限改为490
//0x131修改了重刷程序e方存储的车型默认为0的情况（程序判断为0直接设置为默认5m的车型）
//0x132 修改MCU若为老版本MCU程序，ECU修改MCU参数1S内没有通讯，直接判定保存失败
#define RELEASE_MONTH  1


typedef union
{
	uint16_t u16Data;
	struct
	{
		uint16_t b1StartFast:1;
		uint16_t b1AntipinchSlow:1;
		uint16_t b1Err18LiftAllow:1;
		uint16_t b1LiftAllowBeforeCali:1;
		uint16_t b1LiftBanMove:1;
		uint16_t b1StartWithMode:1;
		uint16_t b1LowLimitEn:1;
		uint16_t b1UpLimitEn:1;
		uint16_t b1HighSpeedPump:1;
		uint16_t b1SleepEn:1;
		uint16_t b1DoubleRangeHeight:1;
		uint16_t b1LowLimitBanMoveEn:1;
	};
}xUserSets;

xUserSets sgUserSets;

typedef struct
{
	int8_t i8MiddleValue;
	int8_t i8PositiveValue;
	int8_t i8NegativeValue;
}xPcuHandle;

xPcuHandle sgPcuHandle;

static uint8_t u8RemoteParaFlag;
#define REMOTE_INIT							0//等待检查
#define REMOTE_WAITHANDE				1//检测成功，等待握手
#define REMOTE_HANDSHAKE				2//握手成功，检测参数升级状态
//#define REMOTE_CHECKFAIL				3//状态检测失败
//#define REMOTE_WAITUSER					4//状态检测成功、等待确认或取消
#define REMOTE_WAITEDATA				5//等待修改参数
#define REMOTE_CHANGE_SUCCESS		6//参数修改成功
#define	REMOTE_CHANGE_FAIL			7//参数修改失败
#define REMOTE_OTA_REQ					8//远程升级请求
#define REMOTE_OTA_START		9//等待用户确认

static uint8_t u8RunMode = 0;
#define	NORMAL_MODE								0
#define PARA_SETS_MODE						1
#define	PRESSURE_CALIBRATION_MODE	2
#define NO_ACTION_MODE 						3
#define AUTO_LIFT_MODE            4
	

	
static uint8_t u8ParaSetMode = 0;
#define PARASETS_DISABLE	0
#define COMMON_PARASET		1
#define SPEED_PARASET			2
#define MACHINE_PARASET		3
#define	FUNCTION_PARASET	4
#define H8_MODE						5
#define H2_MODE						6
#define H0_MODE						7
#define H1_MODE						8
#define TBOX_PARA_SETS		9
#define TBOX_UPDATEAPP		10	

#if 1

#define SECOND_LEVEL_BOOT_ADDR                		0x08008000
#define	USER_FLAG_ADDR														0x08005800
#define USER_VALID																0x12345678

#define	UDS_UDDATE_FALGE_ADDR                     0x08006800
#define UDS_UPDATE_FLAGE                          0x1A1A1A1A

//#endif

#else  //改动后的boot及二级boot

#define SECOND_LEVEL_BOOT_ADDR                		0x08004000
#define	USER_FLAG_ADDR														0x0800F000
#define USER_VALID																0x55555555

#define	UDS_UDDATE_FALGE_ADDR                     0x0800F800
#define UDS_UPDATE_FLAGE                          0xCCCCCCCC

#endif 

static uint8_t u8PumpFlag;

#endif //#if (USER_TYPE == USER_FORKLIFT_LIUGONG)
#endif //#ifndef _USER_COMM_H_
 
