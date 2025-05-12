/*******************************************************************************
* Filename: UserStackerTruckProc.h	                                           *
* Description: 逻辑层的头文件，包含对应的回调函数	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/07/22    														   *
* Revision:	 														 		   *
*******************************************************************************/
#ifndef _USER_HANGCHA_PODAOCHE_PROC_H_
#define _USER_HANGCHA_PODAOCHE_PROC_H_

#include "MstSlvCom.h"
#include "DoPwm.h"
#include "PropDriver.h"
#include "LocalDi.h"
#include "NetTimer.h"

/******************************************************************************
*函数定义
******************************************************************************/


#define	USER_PERIOD				5	

#define	LIFTUP_VALVE				DRIVER5
#define DOWN_VALVE          DRIVER7 //下降阀为电磁阀
#define	LEAN_FORWARD_VALVE	DRIVER3
#define HONR_VALVE          DRIVER9  

#define	LEAN_BACKWARD_VALVE		DRIVER4


#define	LIFT_UP_LIMIT_SWI		SWI1_R
#define	EMS_SWI							SWI2_R
#define	LOCK_SWI						SWI3_R
#define	CHARGE_SWI					SWI4_R

#define	TURN_SEN_INPUT		AI_B_AI1_R
#define	ANG_SEN_INPUT			AI_B_AI2_R
#define PRESS_SEN_INPUT   AI_B_AI3_R

#define	CAN_1E0_LOST_NO			(1000 / USER_PERIOD)

#define	ECU_POWERON_DELAY_TIME	500
#define	ECU_ANTI_PINCH_TIME 		3000
#define	DRIVER_CLOSE			0
#define	DRIVER_OPEN				1
//#define	DRIVER_CLOSE			1
//#define	DRIVER_OPEN				0

#define	LIFT_MODE_SWI					0
#define	LIFT_MODE_THROTTLE		1

#define	MOTOR_MIN_SPEED				30		/*怠速转速*/
#define	MOTOR_MAX_SPEED				2000
#define	MOTOR_MAX_SPEED_VALUE	4095
#define	MOTOR_SPEED_RANGE			4096
#define	PUMP_MAX_VALUE				255
#define	PUMP_RANGE						256

#define	PROP_MIN_CURRENT		PARA_PropDMinCurrent1
#define	PROP_MAX_CURRENT		PARA_PropDMaxCurrent1
#define	LEAN_FORWARD_PARA		PARA_PumpMotorGear1
#define	LEAN_BACKWARD_PARA	PARA_PumpMotorGear2

#define	LIFT_MODE_PARA			PARA_BrakeType
#define	USERINFO_MODE				PARA_ValueOpenPercentage


#define FORWORD_LIMIT_ANGLE  PARA_AngleValue0  //前倾限位
#define BACKWORD_LIMIT_ANGLE PARA_AngleValue1  //后倾限位
#define AUTO_RAMP_ANGLE      PARA_AngleValue2  //自动坡道角
#define SET_EXIT_ANGLE       PARA_AngleValue3  //退出坡道角
#define SET_MAX_TILE_ANGLE   PARA_AngleValue4  //坡道模式下允许的最大倾角
#define MAX_LEAN_ANGLE       PARA_AngleValue5  //整体允许的最大倾角
#define PRELEAN_ANGLE        PARA_AngleValue6  //预倾斜角
#define TARGET_ANGLE         PARA_AngleValue7  //目标倾斜角

#define MAX_PRESSURE         PARA_FullPressure    //最大压力

#define TURN_DEC_SPD_ENABLE  PARA_AngleSensorSetting   //#70 1使能转弯降速，0关闭
#define TILESENSORENABLE     PARA_TiltSwitchSetting    //#71 1倾角传感器有效，0无效
#define TURN_DIF_SPD_ENABLE  PARA_AnticollisionFunc    //借用#116防碰撞功能 1开启差速功能，0关闭
#define ANGLEAILIMIT         PARA_AngleSimulationLimit   //#60 角度模拟限位开关
#define INTERLOCK_EN         PARA_ActAlmFunc            //动作互锁开关
#define LOWPOWER_1STEP       PARA_Analog3DeadZoneMinVal //借用141用作低电量一级报警
#define LOWPOWER_2STEP       PARA_Analog3DeadZoneMaxVal //借用142用作低电量二级报警




#define USER_SET           PARA_ValveType
//#define	BAT_LOW_WARING_VAL			20
//#define	BAT_LOW_ERR_VAL				15	


#define	LIFTUP_VALVE_ERR					ErrCode53
#define	LEAN_FORWARD_VALVE_ERR		ErrCode54
#define	LEAN_BACKWARD_VALVE_ERR		ErrCode55
#define	LIFTDOWN_VALVE_ERR				ErrCode61









/*杭叉通用错误码*/
#define	ACT_INIT_ERR				 ErrCode89
#define	ACT_LOCK_ERR				 ErrCode90
#define	MOVE_EMS_ERR				 ErrCode91
#define CHARGE_ACT_ERR       ErrCode92
#define	AI_B_AI1_ERR				 ErrCode93   //转弯角度传感器(模拟量1故障)


/*********************坡道车特殊错误码********************************************/
#define	HANDLE_NOCAN_ERR		 ErrCode100  //手柄报文超时
#define BMS_NOCAN_ERR        ErrCode101  //锂电BMS报文掉线
#define EBRAKE_ADHESION      ErrCode102  //紧急反向粘连
#define EBRAKE_ERR           ErrCode103  //紧急反向操作顺序故障
#define REMOTE_LOST_ERR      ErrCode104  //远程管理模块报文超时
#define TILESENSOR_ERR       ErrCode105  //倾斜传感器（模拟量2）故障
#define HMI_COM_ERR          ErrCode106  //仪表握手失败
#define HMI_CAN_LOST         ErrCode107  //仪表报文超时
#define DIRICTION_ERR        ErrCode108  //角度传感器为正值且超过允许的最大角度
#define OVER_LEAN_ERR        ErrCode109  //陡峭报警
#define OVER_PRESS_ERR       ErrCode110  //过压报警
#define FRONTRANK_NOCAN_ERR  ErrCode111  //前倾角传感器掉线(185)
#define CASTER_NOCAN_ERR     ErrCode112  //后倾角传感器掉线(186)
#define PRESSENSOR_ERR       ErrCode113  //压力传感器（模拟量3）掉线报警
/**********************锂电错误码**************************************************/
#define SoloUnderVotage_ERR  ErrCode114   //单体欠压
#define AllUnderVotage_ERR   ErrCode115   //总体严重欠压
#define TmpProtect_ERR       ErrCode116   //温度保护（一般）
#define OverTmp_ERR          ErrCode117   //严重过温保护
#define OverVotage_ERR       ErrCode118   //过压错误
#define OverCurrent_ERR      ErrCode119   //过流错误

/*未定义错误码*/
#define SMOVE_ERR            ErrCode149
#define LOWPOWER_1ERR        ErrCode150//低于一级低电量阈值报警
#define LOWPOWER_2ERR        ErrCode151//低于二级低电量阈值报警



#define PARA_WorkCountL		PARA_USER_DATA_24
#define PARA_WorkCountH		PARA_USER_DATA_25

#define	STEP_FACTOR					(5.0 * 4096 / 255)

#define	TURN_START_ANGLE		PARA_TurnWithDecStartAngle
#define	TURN_END_ANGLE			PARA_TurnWithDecEndAngle
#define	START_ANGLE_SPD			PARA_AngleWithStartSpdPer
#define	END_ANGLE_SPD				PARA_AngleWithEndSpdPer

#define SOFTWARE_VERSION_CODE  0x0101

typedef struct
{
	uint16_t	u16StartAngle;
	uint16_t	u16EndAngle;
	uint16_t	u16StartAngleSpd;
	uint16_t	u16EndAngleSpd;
	float		fAgnleDecSpdFactor;
}xSteerAngleDecSpd;


extern void vUserEcuInit(void);
extern void vUserEcuProc(void);
void vUserEcuLogic(void);

#endif
