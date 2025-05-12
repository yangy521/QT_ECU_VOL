/*******************************************************************************
* Filename: iCANPlc.h	                                             	 		   *
* Description:											   			 		   *
* Author:                                                           		   *
* Date:     														 		   *
* Revision:															 		   *
*******************************************************************************/
#ifndef __ICANPLC_H
#define __ICANPLC_H

#include "KSDsys.h"

/*******************************************************************************
* 4. 结构体定义
*******************************************************************************/
#define FOOT_BRAKE_ucIoFromMove   (1 << 0)

/*Bit define of ucPrdStateS & ucPrdStateM */
#define  START_PRDSTATE				(1 << 0)
#define  READY_PRDSTATE				(1 << 1)
#define  CANBEAT_PRDSTATE			(1 << 2)

/*Bit define of ucStateSettingS & ucStateSettingM */
#define  PRDID_STATESETTING(n)				(n & 0x7)

#define  MODE_STATESETTING(n)					((n >> 3) & 0x7)
#define  PC_IDLE_MODE_STATESETTING				0
#define  PC_MONITOR_MODE_STATESETTING			1
#define  PC_CONFIG_MODE_STATESETTING			2
#define  PC_TUNE_MODE_STATESETTING				3
#define  HMI_IDLE_MODE_STATESETTING				4
#define  HMI_MONITOR_MODE_STATESETTING		5
#define  HMI_CONFIG_MODE_STATESETTING			6
#define  HMI_TUNE_MODE_STATESETTING				7

#define  INSTRUCT_STATESETTING(n)	    ((n >> 6) & 0x3)
#define  IDLE_INSTRUCT_STATESETTING   0
#define  RD_INSTRUCT_STATESETTING   	1
#define  WR_INSTRUCT_STATESETTING     2

//起升控制器
typedef struct _tICAN_LIFT
{
	//资源节点信息
	unsigned char  ucStateSettingS;	//  设备工作状态设定	SoruceID:0x00
	unsigned char  ucErrorCode;			//	设备故障代码			SoruceID:0x01
	unsigned char  ucPrdStateS;			//	设备运行状态		 	SoruceID:0x0D
	
	unsigned char  ucStateSettingM;	//  设备工作状态设定	SoruceID:0x20
#if ((USER_TYPE == USER_NBRY_QYCC15T_MOVE)\
	  || (USER_TYPE == USER_NBRY_QYCC15T_LIFT))
	unsigned char  ucPitchCmd;  		//倾斜命令	SoruceID:0x21
	unsigned char  ucSidleCmd;			//侧移命令	SoruceID:0x22
#endif
#if ((USER_TYPE == USER_AHHL_PHZC20T_MOVE) \
	|| (USER_TYPE == USER_AHHL_PHZC20T_LIFT) \
	|| (USER_TYPE == USER_AHHL_PHZC25T_MOVE) \
	|| (USER_TYPE == USER_AHHL_PHZC25T_LIFT) \
	)
	unsigned char  ucIoFromMove;  		//IO	SoruceID:0x21
#endif
	
	unsigned char  ucBatteryAct;		//  修正后的电池电量 	SoruceID:0x29
	unsigned char  ucPrdStateM;			//	设备运行状态		 	SoruceID:0x2D
	
}tICAN_LIFT;

//转向控制器
typedef struct _tICAN_STEER
{
	//资源节点信息
	unsigned char  ucStateSettingS;	//  设备工作状态设定	SoruceID:0x00
	unsigned char  ucErrorCode;			//	设备故障代码		 SoruceID:0x01
	unsigned char  ucSteerAngleH;		//	转向角度高字节	 SoruceID:0x02
	unsigned char  ucSteerAngleL;		//	转向角度低字节	 SoruceID:0x03
	unsigned char  ucSteerSpeedH;		//  转向速度高字节  	SoruceID:0x06
	unsigned char  ucSteerSpeedL;		//  转向速度高字节  	SoruceID:0x07
	unsigned char  ucPrdStateS;			//	设备运行状态     SoruceID:0x0D
	
	unsigned char  ucStateSettingM;	//  设备工作状态设定	SoruceID:0x20
	unsigned char  ucBatteryAct;		//  修正后的电池电量 	SoruceID:0x29
	unsigned char  ucPrdStateM;			//	设备运行状态     SoruceID:0x2D

}tICAN_STEER;

//副行走控制器
typedef struct _tICAN_SMOVE
{
	//资源节点信息
	unsigned char  ucStateSettingS;	//  设备工作状态设定	SoruceID:0x00
	unsigned char  ucErrorCode;			//	设备故障代码			SoruceID:0x01
	unsigned char  ucPrdStateS;			//	设备运行状态		 	SoruceID:0x0D
	
	unsigned char  ucStateSettingM;	//  设备工作状态设定	SoruceID:0x20
	unsigned char  ucBatteryAct;		//  修正后的电池电量 	SoruceID:0x29
	unsigned char  ucPrdStateM;			//	设备运行状态		 	SoruceID:0x2D
	
}tICAN_SMOVE;

//逻辑控制器
typedef struct _tICAN_LOGIC
{
	//资源节点信息
	unsigned char  ucStateSettingS;	//  设备工作状态设定	SoruceID:0x00
	unsigned char  ucErrorCode;			//	设备故障代码			SoruceID:0x01
	unsigned char  ucPrdStateS;			//	设备运行状态		 	SoruceID:0x0D
	
	unsigned char  ucStateSettingM;	//  设备工作状态设定	SoruceID:0x20
	unsigned char  ucBatteryAct;		//  修正后的电池电量 	SoruceID:0x29
	unsigned char  ucPrdStateM;			//	设备运行状态		 	SoruceID:0x2D
	
}tICAN_LOGIC;

//仪表
//Bit define of ucVehicleState
#define  START_VEHICLESTATE				(1<<0)
#define  FORWARD_VEHICLESTATE			(1<<1)
#define  REVERSE_VEHICLESTATE			(1<<2)
#define  MIDDLE_VEHICLESTATE			(1<<3)
#define  ZUOYI_VEHICLESTATE				(1<<4)
#define  HANDBRAKE_VEHICLESTATE		(1<<5)
#define  GET_MODE__VEHICLESTATE(VehicleState)  ((VehicleState & (3<<6)) >> 6)
#define  SET_MODE__VEHICLESTATE(mode,VehicleState)  VehicleState &= ~(3<<6);\
																		                VehicleState |= mode << 6;
typedef struct _tICAN__tHMI
{
	//资源节点信息
	unsigned char  ucStateSettingS;	//  设备工作状态设定	SoruceID:0x00
	unsigned char  ucErrorCode;			//	设备故障代码			SoruceID:0x01
	unsigned char  ucBattery;			  //	电池电量          SoruceID:0x02
	unsigned char  ucPrdStateS;			//	设备运行状态		 	SoruceID:0x0D
	
	unsigned char  ucStateSettingM;	//  设备工作状态设定	SoruceID:0x20
	unsigned char  ucNetCfg;				//  子站连接状态位	  SoruceID:0x21
	unsigned char  ucErrMove;				//  牵引控制器故障  	SoruceID:0x22
	unsigned char  ucErrLift;			  //  起升控制器故障		SoruceID:0x23
	unsigned char  ucErrSteer;			//  转向控制器故障		SoruceID:0x24
	unsigned char  ucMoveSpeedH;		//  行驶速度高字节  	SoruceID:0x25
	unsigned char  ucMoveSpeedL;		//  行驶速度低字节  	SoruceID:0x26
	unsigned char  ucSteerAngleH;		//  行驶速度高字节  	SoruceID:0x27
	unsigned char  ucSteerAngleL;		//  行驶速度低字节  	SoruceID:0x28
	unsigned char  ucBatteryAct;		//  修正后的电池电量 	SoruceID:0x29
	unsigned char  ucVehicleState;	//  车辆状态位 				SoruceID:0x2A
	unsigned char  ucPrdStateM;			//	设备运行状态		 	SoruceID:0x2D
	
}tICAN_HMI;
//逻辑控制器
typedef struct _tICAN_PC
{
	//资源节点信息
	unsigned char  ucStateSettingS;	//  设备工作状态设定	SoruceID:0x00
	unsigned char  ucErrorCode;			//	设备故障代码			SoruceID:0x01
	unsigned char  ucPrdStateS;			//	设备运行状态		 	SoruceID:0x0D
	
	unsigned char  ucStateSettingM;	//  设备工作状态设定	SoruceID:0x20
	unsigned char  ucNetCfg;				//  子站连接状态位	  SoruceID:0x21
	unsigned char  ucBatteryAct;		//  修正后的电池电量 	SoruceID:0x29
	unsigned char  ucPrdStateM;			//	设备运行状态		 	SoruceID:0x2D
}tICAN_PC;

//Rema 手柄
#define NEUTRAL_POS_DI				(0x0001 << 0)
#define EMERGENCY_REV_DI			(0x0001 << 1)
#define SNAIL_TOGGLE_DI				(0x0001 << 2)
#define HORN_PUSH_DI				  (0x0001 << 3)
#define LIFT1_DIGIT_DI				(0x0001 << 4)
#define LOW1_DIGIT_DI				  (0x0001 << 5)
#define LIFT2_DIGIT_DI				(0x0001 << 6)
#define LOW2_DIGIT_DI				  (0x0001 << 7)
#define PICK1_GIGIT_DI				(0x0001 << 8)
#define PICK2_GIGIT_DI				(0x0001 << 9)
#define SPARE1_GIGIT_DI				(0x0001 << 10)
#define SPARE2_GIGIT_DI				(0x0001 << 11)
//#define REV_DI				(0x0001 << 12)
//#define REV_DI				(0x0001 << 13)
//#define REV_DI				(0x0001 << 14)
#define STUFF_TOGGLE_DI				(0x0001 << 15)
#define LIFT_LOW_ALL		(LIFT1_DIGIT_DI | LOW1_DIGIT_DI | LIFT2_DIGIT_DI | LOW2_DIGIT_DI)

typedef struct _tCANOPEN_REMAHANDLE
{
	//资源节点信息
	INT16U		Di;
	INT16S		nCmdSpeed;
	INT16S		nCmdAngle;
	INT16S		nReserved;
	INT16S		MoveThrottleAd13S;
}_tCANOPEN_REMAHANDLE;

typedef union _tPDO_STEER
{
	//资源节点信息
	INT8U ucData[8];
	
	struct{
	INT8U		ucSteerState;
	INT8U		ucErrSteer;
	INT8U		ucSteerAngleL;
	INT8U		ucSteerAngleH;
	INT8U	  ucSteerSpeedL;
	INT8U		ucSteerSpeedH;
	INT8U		ucReserve1;	
	INT8U		ucReserve2;		
	}DataStruct;
}tPDO_STEER;
/******************************************************************************
*函数定义
******************************************************************************/
extern void ICANLogicIn_Master(void);
extern void ICANLogicOut_Master(void);
extern void ICANLogicIn_Slave(void);
extern void ICANLogicOut_Slave(void);
extern void ICANInitialize(void);

#endif //__ICANPLC_H
