/*******************************************************************************
* Filename: Message.h	                                             	 		   *
* Description:											   			 		   												*
* Author:                                                           		   *
* Date:     														 		   														*
* Revision:															 		 														  *
*******************************************************************************/

#include "KSDsys.h"
#include "queue.h"

//宏声明
#define Msg_ICOM_MAX	32 	//大小不能超过queue.h中LEN_DATAQUEUE的大小
#define Msg_ICAN_MAX	32
//消息类型定义
#define Msg_Type_ICOM		1
#define Msg_Type_ICAN		2
#define Msg_Type_System	3
#define Msg_Type_Error	4
//事件名称定义
//ICOM
#define Msg_ReadConfigData		1
#define Msg_ReadPthData				2
#define Msg_ICOMRetry					3
//ICAN
#define Msg_ReadPrdType				10
#define Msg_ICANRetry					12

#define Msg_MacIDCheck				13
#define Msg_WritePort					14
#define Msg_ReadPort					15
#define Msg_SetConnect				16

//System
#define Msg_SaveConfigData		21
//故障报警定义（数字越小，优先级越高）
#define Msg_Err_ICANMacIDCheck		31	// MacID检测失败
#define Msg_Err_ICOMConnect				32 	// ICOM连接失败
#define Msg_Err_ICANConnect				33	// ICAN连接失败
#define Msg_Err_ConfigUnfinished	34	// 系统配置未完成
#define Msg_ErrAck								35	// ICAN连接未建立
#define Msg_Err_CnvType						36	// 设备类型配置错误

#define Msg_Err_ConfigData			38	// 系统配置错误
#define Msg_Err_PthData					39	// 路径配置错误
#define Msg_Err_OverVoltage			40	// 过压报警
#define Msg_Err_ShortVoltage		41	// 欠压报警
#define Msg_Err_IPM							42	// 功率驱动报警
#define Msg_Err_OverLoad				43	// 过载报警
#define Msg_Err_Eeprom					44	// 存储错误报警
#define Msg_Err_Belt						45	// 皮带跑偏报警

//定义报警
typedef struct _tAlarm
{
	unsigned char	ucMacID;		//	产品ID（CANID）
	unsigned char	ucType;			//	报警类型
}tAlarm;
//变量声明
extern tAlarm sysAlarm[64];	//系统报警列表


extern void MsgInitialize(void);//消息处理初始化
extern void MsgFlush(void);//消息队列清空
extern void MsgManage(void);//消息事件处理	
extern void MsgICANRegister(unsigned char ucMacID,unsigned char ucEvent);//ICAN消息注册
extern void MsgICOMRegister(unsigned char ucEvent);//ICOM消息注册	
extern void MsgAdd(unsigned char ucType,tMessage message);//消息添加	
extern tBoolean MsgICANPop(tMessage *pMessage);//ICAN消息弹出（先入先出）	
extern tBoolean MsgICOMPop(tMessage *pMessage);//ICOM消息弹出（先入先出）
extern void AlarmICANRegister(unsigned char ucMacID,unsigned char ucEvent);//ICAN报警注册	
extern void AlarmICOMRegister(unsigned char ucEvent);//ICOM报警注册
extern void AlarmSysRegister(unsigned char ucEvent);	//系统报警注册
