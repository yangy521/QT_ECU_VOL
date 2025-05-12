/*******************************************************************************
* Filename: ServoPara.h 	                                    	     		   *
*                                                                              *
* Description: The header file of ServoPara.c.							           *
* Author:                                                                      *
* Date: 															           *
* Revision:															           *
*																	           *
*******************************************************************************/
#ifndef _SERVOPARA_H_
#define _SERVOPARA_H_

#include	"KSDsys.h"
//Voltage circle
#define VOLTAGE_MODULATE_MAX		  (_IQ(1.0) - 1)
#define VOLTAGE_MODULATE_MIN  		(-_IQ(1.0) + 1)
//Voltage hexagon
//#define VOLTAGE_MODULATE_MAX		  (_IQ(2.0/SQRT3) - 1)
//#define VOLTAGE_MODULATE_MIN  		(-_IQ(2.0/SQRT3) + 1)



/* motor parameters */
typedef struct MOTOR_PARA
{
	INT16U	Type;					/* 电机号 0:电机参数上位机设定；1~n：电机参数程序预先内置 cMotorParaRom[n]*/
	INT16U	EncoderLineNumM4;	/* Encoder M4 line number */
	INT16U	CalibAngle;		/* Q0, 编码器零位偏差 */
	INT16U	PolePair;			/* Q0, motor pole pairs*/
	_iq		TypicalHZ;			    /* Q24 = STD_FRQ Typical frq, as standard for acc/dec rate */
	_iq		WkBaseHZ;				/* Q24 = STD_FRQ Base frq for weak flux */
	_iq		RatedHZ;			  /* Q24 = STD_FRQ Base frq for current limit */
	_iq		DeltaHZ;			  /* Q24 = STD_FRQ Delta inc */
	_iq		RatedCurrent;		/* Q24 = STD_CURRENT  */
	_iq		DriveCurrent;		/* Q24 = STD_CURRENT  */
	_iq		DriveFlux;				/* Q24 = STD_CURRENT  */
	_iq		UphillCurrent;			/* Q24 = STD_CURRENT  */
	_iq		UphillFlux;			/* Q24 = STD_CURRENT  */
	_iq		RegenCurrent;			/* Q24 = STD_CURRENT  */
	_iq		RegenFlux;			/* Q24 = STD_CURRENT  */
	_iq		BrakeCurrent;			/* Q24 = STD_CURRENT  */
	_iq		BrakeFlux;			/* Q24 = STD_CURRENT  */
	_iq		DriveMapNominalRatio;		//Drive current limit  map nominal ratio 0~100%--IQ(0)~IQ(1.0)
	_iq		DriveMapDelta1Ratio;		//Drive current limit  map Delta1 ratio 0~100%--IQ(0)~IQ(1.0)
	_iq		DriveMapDelta2Ratio;		//Drive current limit  map Delta2 ratio 0~100%--IQ(0)~IQ(1.0)
	_iq		DriveMapDelta4Ratio;		//Drive current limit  map Delta4 ratio 0~100%--IQ(0)~IQ(1.0)
	_iq		DriveMapDelta8Ratio;		//Drive current limit  map Delta8 ratio 0~100%--IQ(0)~IQ(1.0)
	_iq		RegenMapNominalRatio;		//Regenerate current limit  map nominal ratio 0~100%--IQ(0)~IQ(1.0)
	_iq		RegenMapDelta1Ratio;		//Regenerate current limit  map Delta1 ratio 0~100%--IQ(0)~IQ(1.0)
	_iq		RegenMapDelta2Ratio;		//Regenerate current limit  map Delta2 ratio 0~100%--IQ(0)~IQ(1.0)
	_iq		RegenMapDelta4Ratio;		//Regenerate current limit  map Delta4 ratio 0~100%--IQ(0)~IQ(1.0)
	_iq		RegenMapDelta8Ratio;		//Regenerate current limit  map Delta8 ratio 0~100%--IQ(0)~IQ(1.0)
	_iq		WkPowerRatio;						//The amount of high speed power the controller will allow 0~100%--IQ(0)~IQ(1.0)
	_iq		WkRate;									//The control loop gains for field weakening.
	_iq		WkAdjRatio;							//The weak flux Ratio adjustment amount from WkBaseHZ to TypicalHZ 0~200%--IQ(0)~IQ(2.0)
	_iq		RotorTCoe;							//The motor rotor thermal coe. Cuprum--_IQ(0.00393),Aluminum--_IQ(0.00429)
	_iq		KTorque;				/* Q24 = STD_TORQUE_K */
	_iq		Inertia;				/* Q24 = STD_INERTIA */
	_iq		Resist;					/* Q24 = STD_RESIST */
	_iq		Tstator;				/* Q24 = STD_T */
	_iq		Trotor;					/* Q24 = STD_T */
	//var
	_iq		RatedHZ1;			  /* Q24 = STD_FRQ  RatedHZ + DeltaHZ * 1 */
	_iq		RatedHZ2;			  /* Q24 = STD_FRQ  RatedHZ + DeltaHZ * 2 */
	_iq		RatedHZ4;			  /* Q24 = STD_FRQ  RatedHZ + DeltaHZ * 4 */
	_iq		RatedHZ8;			  /* Q24 = STD_FRQ  RatedHZ + DeltaHZ * 8 */
	_iq		DrvKf2iqRatio1;		/* Drive Kcoe for RatedHZ~RatedHZ1  */
	_iq		DrvKf2iqRatio2;		/* Drive Kcoe for RatedHZ1~RatedHZ2 */
	_iq		DrvKf2iqRatio4;		/* Drive Kcoe for RatedHZ2~RatedHZ4 */
	_iq		DrvKf2iqRatio8;		/* Drive Kcoe for RatedHZ4~RatedHZ8 */
	_iq		RegenKf2iqRatio1;		/* Regenerate Kcoe for RatedHZ~RatedHZ1  */
	_iq		RegenKf2iqRatio2;		/* Regenerate Kcoe for RatedHZ1~RatedHZ2 */
	_iq		RegenKf2iqRatio4;		/* Regenerate Kcoe for RatedHZ2~RatedHZ4 */
	_iq		RegenKf2iqRatio8;		/* Regenerate Kcoe for RatedHZ4~RatedHZ8 */
	_iq		UphillRatioKoe;			/* UphillRatioKoe = (UphillCurrent/DriveCurrent-1.0)/(WkBaseHZ>>2)*/
	_iq		WkAdjRatioKoe;			/* WkAdjRatioKoe = (WkAdjRatio-1.0)/(TypicalHZ - RatedHZ)*/
	_iq		RotorTCoeK;					//_IQ(1.0)+RotorTCoe*(Tmotor-20)
	_iq		MinSpeed;			/* Min Speed */
} MOTOR_PARA;

/* rigid parameters */
typedef struct RIGID_PARA
{
	INT16U	Rigid;				/* ???? */
	INT16U	PosKp;				/* ????? */
	INT16U	SpdKp;				/* ????? */
	INT16U	SpdKit;				/* ???????*/
	INT16U	TorFilterW1;		/* ?????1???? */
	INT16U	TorFilterW2;		/* ?????2???? */
} RIGID_PARA;

#if (VOLTAGE_LEVEL == _VOLTAGE_24V)  //For 24V driver
extern struct MOTOR_PARA cMotorParaRom[2];
#endif
#if (VOLTAGE_LEVEL == _VOLTAGE_48V)  //For 48V driver
extern struct MOTOR_PARA cMotorParaRom[1];
#endif
//For 80V driver
#if (   (VOLTAGE_LEVEL == _VOLTAGE_72V) \
			||(VOLTAGE_LEVEL == _VOLTAGE_80V) \
			||(VOLTAGE_LEVEL == _VOLTAGE_88V)  )
extern struct MOTOR_PARA cMotorParaRom[1];
#endif

extern void CalcServoLoopPara(void);
extern void UpdatePara(void);
#endif //_SERVOPARA_H_
