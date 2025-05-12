/*******************************************************************************
* Filename: ErrCode.h	                                             	 	   *
* Description:ErrCode C Head File											   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/5/31    														   *
* Revision: V1.00															   *
*******************************************************************************/

#ifndef _ERRCODE_H_
#define _ERRCODE_H_

#include "stdint.h"
#include "string.h"

typedef enum
{
	ErrCode1 = 0,
	ErrCode2,
	ErrCode3,
	ErrCode4,
	ErrCode5,
	ErrCode6,
	ErrCode7,
	ErrCode8,
	ErrCode9,
	ErrCode10,
	ErrCode11,
	ErrCode12,
	ErrCode13,
	ErrCode14,
	ErrCode15,
	ErrCode16,
	ErrCode17,
	ErrCode18,
	ErrCode19,
	ErrCode20,
	ErrCode21,
	ErrCode22,
	ErrCode23,
	ErrCode24,
	ErrCode25,
	ErrCode26,
	ErrCode27,
	ErrCode28,
	ErrCode29,
	ErrCode30,
	ErrCode31,
	ErrCode32,
	ErrCode33,
	ErrCode34,
	ErrCode35,
	ErrCode36,
	ErrCode37,
	ErrCode38,
	ErrCode39,
	ErrCode40,
	ErrCode41,
	ErrCode42,
	ErrCode43,
	ErrCode44,
	ErrCode45,
	ErrCode46,
	ErrCode47,
	ErrCode48,
	ErrCode49,
	ErrCode50,
	ErrCode51,
	ErrCode52,
	ErrCode53,
	ErrCode54,
	ErrCode55,
	ErrCode56,
	ErrCode57,
	ErrCode58,
	ErrCode59,
	ErrCode60,
	ErrCode61,
	ErrCode62,
	ErrCode63,
	ErrCode64,
	ErrCode65,
	ErrCode66,
	ErrCode67,
	ErrCode68,
	ErrCode69,
	ErrCode70,
	ErrCode71,
	ErrCode72,
	ErrCode73,
	ErrCode74,
	ErrCode75,
	ErrCode76,
	ErrCode77,
	ErrCode78,
	ErrCode79,
	ErrCode80,
	ErrCode81,
	ErrCode82,
	ErrCode83,
	ErrCode84,
	ErrCode85,
	ErrCode86,
	ErrCode87,
	ErrCode88,
	ErrCode89,
	ErrCode90,
	ErrCode91,
	ErrCode92,
	ErrCode93,
	ErrCode94,
	ErrCode95,
	ErrCode96,
	ErrCode97,
	ErrCode98,
	ErrCode99,
	ErrCode100,
	ErrCode101,
	ErrCode102,
	ErrCode103,
	ErrCode104,
	ErrCode105,
	ErrCode106,
	ErrCode107,
	ErrCode108,
	ErrCode109,
	ErrCode110,
	ErrCode111,
	ErrCode112,
	ErrCode113,
	ErrCode114,
	ErrCode115,
	ErrCode116,
	ErrCode117,
	ErrCode118,
	ErrCode119,
	ErrCode120,
	ErrCode121,
	ErrCode122,
	ErrCode123,
	ErrCode124,
	ErrCode125,
	ErrCode126,
	ErrCode127,
	ErrCode128,
	ErrCode129,
	ErrCode130,
	ErrCode131,
	ErrCode132,
	ErrCode133,
	ErrCode134,
	ErrCode135,
	ErrCode136,
	ErrCode137,
	ErrCode138,
	ErrCode139,
	ErrCode140,
	ErrCode141,
	ErrCode142,
	ErrCode143,
	ErrCode144,
	ErrCode145,
	ErrCode146,
	ErrCode147,
	ErrCode148,
	ErrCode149,
	ErrCode150,
	ErrCode151,
	ErrCode152,
	ErrCode153,
	ErrCode154,
	ErrCode155,
	ErrCode156,
	ErrCode157,
	ErrCode158,
	ErrCode159,
	ErrCode160,
	ErrCode161,
	ErrCode162,
	ErrCode163,
	ErrCode164,
	ErrCode165,
	ErrCode166,
	ErrCode167,
	ErrCode168,
	ErrCode169,
	ErrCode170,
	ErrCode171,
	ErrCode172,
	ErrCode173,
	ErrCode174,
	ErrCode175,
	ErrCode176,
	ErrCode177,
	ErrCode178,
	ErrCode179,
	ErrCode180,
	ErrCode181,
	ErrCode182,
	ErrCode183,
	ErrCode184,
	ErrCode185,
	ErrCode186,
	ErrCode187,
	ErrCode188,
	ErrCode189,
	ErrCode190,
	ErrCode191,
	ErrCode192,
	ErrCode193,
	ErrCode194,
	ErrCode195,
	ErrCode196,
	ErrCode197,
	ErrCode198,
	ErrCode199,
	ErrCode200,
	ErrCode201,
	ErrCode202,
	ErrCode203,
	ErrCode204,
	ErrCode205,
	ErrCode206,
	ErrCode207,
	ErrCode208,
	ErrCode209,
	ErrCode210,
	ErrCode211,
	ErrCode212,
	ErrCode213,
	ErrCode214,
	ErrCode215,
	ErrCode216,
	ErrCode217,
	ErrCode218,
	ErrCode219,
	ErrCode220,
	ErrCode221,
	ErrCode222,
	ErrCode223,
	ErrCode224,
	ErrCode225,
	ErrCode226,
	ErrCode227,
	ErrCode228,
	ErrCode229,
	ErrCode230,
	ErrCode231,
	ErrCode232,
	ErrCode233,
	ErrCode234,
	ErrCode235,
	ErrCode236,
	ErrCode237,
	ErrCode238,
	ErrCode239,
	ErrCode240,
	ErrCode241,
	ErrCode242,
	ErrCode243,
	ErrCode244,
	ErrCode245,
	ErrCode246,
	ErrCode247,
	ErrCode248,
	ErrCode249,
	ErrCode250,
	ErrCode251,
	ErrCode252,
	ErrCode253,
	ErrCode254,
	ErrCode255,
	ErrCodeMax,
}eErrCodeCh;




//typedef union
//{
//	uint16_t	u16Data;
//	struct
//	{
//		uint16_t	b1ErrCode: 1;
//		uint16_t	b1NoAct: 1;
//		uint16_t	b1Pcu: 1;
//		uint16_t	b1LiftSpd: 1;
//		uint16_t	b1Down: 1;
//		uint16_t	b1Left: 1;
//		uint16_t	b1Right: 1;
//		uint16_t	b1Up: 1;
//		uint16_t	b1BackWard: 1;
//		uint16_t	b1ForWard: 1;
//		uint16_t	b6Reserve: 6;
//	};
//}xErrCodeInfo;


typedef enum
{	
	ABNORMAL_NoAct = 0,
	ABNORMAL_NoMove,
	ABNORMAL_NoPump,
	ABNORMAL_NoPcu,
	ABNORMAL_NoUp,
	ABNORMAL_NoDown,
	ABNORMAL_MainDriver,
	ABNORMAL_NoTurnLeft,
	ABNORMAL_NoTurnRight,
	ABNORMAL_NoForWard,
	ABNORMAL_NoBackWard,
	ABNORMAL_Gear1Spd,
	ABNORMAL_Gear2Spd,
	ABNORMAL_Gear3Spd,
	ABNORMAL_Gear4Spd,
	ABNORMAL_Gear5Spd,
	ABNORMAL_Gear6Spd,
	ABNORMAL_NoLeanForWard,
	ABNORMAL_NoLeanBackWard,
	ABNORMAL_NoForMove,
	ABNORMAL_NoBackMove,
	ABNORMAL_NoLeftMove,
	ABNORMAL_NoRightMove,
	ABNORMAL_Max,
}eAbnormalType;

typedef union
{
	uint32_t	u32Data;
	struct
	{
		uint32_t	b1ErrCode: 1;				/*故障码是否发生*/
		uint32_t	b1NoAct: 1;					/*禁止动作*/
		uint32_t	b1NoMove: 1;				/*禁止行走*/
		uint32_t	b1NoPump: 1;				/*禁止油泵动作*/
		uint32_t	b1NoPcu: 1;					/*禁止PCU*/
		uint32_t	b1NoUp: 1;					/*禁止上升*/
		uint32_t	b1NoDwon: 1;				/*禁止下降*/
		uint32_t	b1MainDriver: 1;			/*断开主接触器*/
		
		uint32_t	b1NoTurnLeft: 1;			/*禁止左转*/
		uint32_t	b1NoTurnRight: 1;			/*禁止右转*/
		uint32_t	b1NoForWard: 1;				/*禁止前进*/
		uint32_t	b1NoBackWard: 1;			/*禁止后退*/
		uint32_t	b1Gear1Spd: 1;				/*1档速度*/
		uint32_t	b1Gear2Spd: 1;				/*2档速度*/
		uint32_t	b1Gear3Spd: 1;				/*3档速度*/
		uint32_t	b1Gear4Spd: 1;				/*4档速度*/
		
	
		uint32_t	b1Gear5Spd: 1;				/*5档速度*/
		uint32_t	b1Gear6Spd: 1;				/*6档速度*/	
		/*下面6个动作是针对货叉而言*/
		uint32_t	b1NoLeanForWard: 1;			/*禁止前倾*/
		uint32_t	b1NoLeanBackWard: 1;		/*禁止后倾*/
		uint32_t	b1NoForMove: 1;				/*禁止前移*/
		uint32_t	b1NoBackMove: 1;			/*禁止后移*/
		uint32_t	b1NoLeftMove: 1;			/*禁止左移*/
		uint32_t	b1NoRightMove: 1;			/*禁止后移*/
		
		uint32_t	b8UserErr: 8;				/*转换的故障码*/			

	};
}xErrCodeInfo;


extern int32_t i32ErrCodeSet(eErrCodeCh ErrCodeNo);
extern int32_t i32ErrCodeClr(eErrCodeCh ErrCodeNo);
extern uint8_t u8ErrCodeGet(void);
extern int32_t i32ErrCodeCheck(eErrCodeCh ErrCodeNo);
extern uint8_t u8ErrCodeGetAbnormal(eAbnormalType eType);
extern void vErrCodeInit(xErrCodeInfo *ErrCodeArray, uint16_t u16ErrCodeMax);
extern uint8_t u8ErrCodeGetTrans();


#endif