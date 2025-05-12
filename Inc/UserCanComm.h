/*******************************************************************************
* Filename: UserStackerTruckProc.h	                                           *
* Description: 逻辑层的头文件，包含对应的回调函数	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/07/22    														   *
* Revision:	 														 		   *
*******************************************************************************/
#ifndef _USER_CAN_COMM_H_
#define _USER_CAN_COMM_H_

#include "stdint.h"
#include "Userdef.h"

#if (USER_TYPE == USER_ZHONGLI_DGC)
#include "UserStackerTrunckCanZhongLi.h"
#elif (USER_TYPE == USER_GANGLI_DGC)
#include "UserGangliDGCCan.h"
#elif (USER_TYPE == USER_ECU_2IN1)
#include "UserEcuCan.h"
#elif(USER_TYPE ==USER_RUYI_DGC)
#include "UserRuyiDGCCan.h"
#elif (USER_TYPE == USER_STACKERTRUNK_RUYI_CQDH15A)
#include "UserRuyiDGCCan.h"
#elif (USER_TYPE == USER_TEST)
#include "UserTestCan.h"
#elif(USER_TYPE == USER_SHANDONGKUNTYU_LOGIC)
#include "User4in1Can.h"
#elif (USER_TYPE == USER_MAXVISION_AGV)
#include "UserAgvMaxvisionCan.h"
#elif (USER_TYPE == USER_NUOLI_PS16)
#include "UserNuoliPSXXNWCan.h"
#elif (USER_TYPE == USER_LIUGONG_2IN1_LOGIC)
#include "UserLiuGong2in1Can.h"
#elif  (USER_TYPE == USER_NUOLI_RT15Q)
#include "UserStackTrunckNuoliRT15QCan.h"
#elif (USER_TYPE == USER_JIALI_15TJDC)
#include "UserStackerTruckCanJiaLi.h"
#elif (USER_TYPE == USER_HANGCHA_PODAOCHE)
#include "UserCanCommHangChaPodaoche.h"
#elif (USER_TYPE == USER_CHUFENG_2IN1_LOGIC)
#include "UserForkLift_ChuFen_Can.h"
#elif (USER_TYPE == USER_FORKLIFT_LIUGONG)
#include "UserLiuGong2in1Can.h"
#elif	(USER_TYPE == USER_QIANGSHENG_DGC)
#include "UserStackTrunkQiangShengCan.h"
#elif(USER_TYPE == USER_FORKLIFT_XUGONG)
#include	"UserForkLiftXuGongCan.h"
#elif(USER_TYPE == USER_SHXIANGONG_AGV)
#include "UserStackTrunkSHXianGongAGVCan.h"
#elif((USER_TYPE == USER_SDSAIYISI_LDJCC_MOVE) ||(USER_TYPE == USER_SDSAIYISI_LDJCC_SMOVE))
#include "USER_SDSaiYiSi_LDJCCCan.h"
#elif((USER_TYPE == USER_SDSAIYISI_BICHE_ZUO_MOVE) ||(USER_TYPE == USER_SDSAIYISI_BICHE_YOU_SMOVE))
#include "USER_SDSaiYiSi_2QBICHECan.h"
#elif ((USER_TYPE == USER_GANGLI_PHZ15T_MOVE) || (USER_TYPE == USER_GANGLI_PHZ15T_LIFT))
#include "UserGangLiPHZ15TCan.h"
#elif	((USER_TYPE == USER_KELI_DDTPC)||(USER_TYPE == USER_JIALI_SZDPHZ))
#include "UserKeliDDTPCan.h"
#elif(USER_TYPE == USER_KELI_ZJDQY_MOVE)
#include "UserKeliZJDQYMoveCan.h"
#elif(USER_TYPE == USER_KELI_ZJDQY_LIFT)
#include "UserKeliZJDQYLiftCan.h"
#elif(USER_TYPE == USER_SANYI_DGC)
#include "UserSanyiDGCCan.h"
#elif(USER_TYPE == USER_JIALI_Q20GA)
#include "UserJialiQ20ACan.h"
#elif(USER_TYPE == USER_JIALI_SanZhiLiu)
#include "UserJialiSanzhiliuCan.h"
#elif(USER_TYPE == USER_XUGONG_3DC_JCC)
#include	"USER_XUGONG_3DC_JCC_Can.h"
#elif(USER_TYPE == USER_RUYI_CDD15C)
#include "UserRUYICDD15CCan.h"
#elif(USER_TYPE == USER_LIDA_JIANXUANC)
#include "UserLiDaJianXuanC_Can.h"
#elif(USER_TYPE == USER_HANGCHA_DGC)
#include "User_HANGCHA_DGCCan.h"
#elif((USER_TYPE == USER_HANGCHA_QYDGC)||(USER_TYPE == USER_HANGCHA_PHZ))
#include "User_HANGCHA_QYDGCCan.h"
#elif(USER_TYPE == USER_MAXIER_DGC)
#include	"UserStackTruckMaXiErCan.h"
#elif(USER_TYPE == USER_RUYI15A3_DGC)
#include "UserRuyi15A3DGCCan.h"
#elif(USER_TYPE == USER_HANGLI_20TDGC)
#include "User_GANGLI_20TDGCCan.h"
#elif(USER_TYPE == USER_LIDA_20TDGC)
#include "User_LIDA_20TDGCCan.h"
#elif((USER_TYPE == USER_SDBOJUN_20TPHZ_MOVE)||(USER_TYPE == USER_SDBOJUN_20TPHZ_SMOVE)||(USER_TYPE == USER_SDBOJUN_20TPHZ_LIFT))
#include "User_SDBOJUN_20TPHZCan.h"
#elif(USER_TYPE == USER_HANGUO_JINJIDAO_DGC)
#include "User_HANGUO_DGCCan.h"
#elif(USER_TYPE == USER_HANGCHA_GAOJI)
#include "User_HANGUO_DGCCan.h"
#elif(USER_TYPE == USER_HANGUO_JINJIDAO_DBC)
#include "User_HANGUO_DBCCan.h"

#endif

typedef struct
{
	uint8_t   u8Flag;				/*0, No Make; 1: Make*/
	uint8_t	  u8Type;				/*0xFF, Period Send, other: No Period Send*/
	uint16_t  u16Period;			/*Send Period*/
	uint16_t  u16CanId;				/*Send Canid, 前四个不起左右*/
}xTpdoParameter;

typedef union
{
	uint16_t u6Data;
	struct
	{
		uint16_t	b1Flag: 1;		/*0: No Make; 1: Make*/
		uint16_t	b11CanRevId: 11;/*CanId*/
		uint16_t	b4Reserve: 4;
	};
}xRpdoParameter;

typedef struct
{
	xTpdoParameter TpdoPara[10];	/*4th is: 0x180 + NodeId, 0x280 + NodeId, 0x380 + NodeId, 0x480 + NodeId,*/
	xRpdoParameter RpdoPara[6];	/*4th is: 0x200 + NodeId, 0x300 + NodeId, 0x400 + NodeId, 0x500 + NodeId,*/
}xPdoParameter;

extern void vSetPdoPara(const xPdoParameter  PdoPara);

extern xCanRevPdoInfo gCanRevPdoInfo;
extern xCanSendPdoInfo gCanSendPdoInfo;

#endif //#ifndef _USER_COMM_H_
