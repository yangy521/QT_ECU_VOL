/*******************************************************************************
* Filename: ServoPara.c                                           	 		   *
*                                                                    		   *
* Description:											   			 		   *
* Author:                                                           		   *
* Date:     														 		   *
* Revision:															 		   *
*																	 		   *
*******************************************************************************/

#include  "ServoPara.h"
#include	"CommonRam.h"
#include	"Current.h"
#include	"PropDriver.h"
#include	"Speed.h"
#include	"Kernel.h"
//#include	"Tune.h"

/*******************************************************************************
* ���������
*
* ע��1�����ֵΪ��Ч���������²���ֵΪIQ��ֵ����
*******************************************************************************/
#if (VOLTAGE_LEVEL == _VOLTAGE_24V)  //For 24V driver
struct MOTOR_PARA cMotorParaRom[2] = 
{
	/****** MotorType 1 ******/
	/*TYPE:  */
	/*PART NO:  */
	/*POWER: 450W, 45A; 20 POLE PARES  Hall 6*20 = 120pls / r */
	/*USER:  ˶����� 450W ���裺0.13  ��������120A ��ų���B��0.0058 ���ת�٣�4000rpm*/
	{
		1,																	/* ����� 0:���������λ���趨��1~n�������������Ԥ������ cMotorParaRom[n]*/
		STD_INC30,													/* Encoder M4 line number */
		2360,																/* Calib angle */
		20,																	/* Q0, motor pole pairs*/
		_IQ(4000.0				/STD_SPEEDRPM),			    /* Q24 = STD_SPEEDRPM  Typical speedrpm, as standard for acc/dec rate */
		_IQ(60.0				/STD_SPEEDRPM),			    /* Q24 = STD_SPEEDRPM Base speedrpm for weak flux */
		_IQ(2000.0				/STD_SPEEDRPM),			    /* Q24 = STD_SPEEDRPM Base speedrpm for current limit */
		_IQ(10.0				/STD_SPEEDRPM),			    /* Q24 = STD_SPEEDRPM Delta inc */
		_IQ(90.0/ STD_CURRENT), /* Q24 = STD_CURRENT RateCurrent  */
		_IQ(90.0/ STD_CURRENT), /* Q24 = STD_CURRENT DriveCurrent  */
		_IQ(90.0 / STD_CURRENT), /* Q24 = STD_CURRENT  DriveFlux*/
		_IQ(110.0 / STD_CURRENT), /* Q24 = STD_CURRENT UphillCurrent */
		_IQ(110.0 / STD_CURRENT), /* Q24 = STD_CURRENT UphillFlux */
		_IQ(45.0 / STD_CURRENT), /* Q24 = STD_CURRENT RegenCurrent  */
		_IQ(45.0 / STD_CURRENT), /* Q24 = STD_CURRENT RegenFlux */
		_IQ(45.0 / STD_CURRENT), /* Q24 = STD_CURRENT BrakeCurrent */
		_IQ(45.0 / STD_CURRENT), /* Q24 = STD_CURRENT  BrakeFlux*/
		_IQ(0.5)-1, 									//Drive current limit  map nominal ratio 0~100%--IQ(0)~IQ(1.0)
		_IQ(0.1)-1, 							//Drive current limit  map Delta1 ratio 0~100%--IQ(0)~IQ(1.0)
		_IQ(1.0)-1, 					//Drive current limit  map Delta2 ratio 0~100%--IQ(0)~IQ(1.0)
		_IQ(1.0)-1, 			//Drive current limit  map Delta4 ratio 0~100%--IQ(0)~IQ(1.0)
		_IQ(1.0)-1,		//Drive current limit  map Delta8 ratio 0~100%--IQ(0)~IQ(1.0)
		_IQ(1.0)-1, 									//Regenerate current limit  map nominal ratio 0~100%--IQ(0)~IQ(1.0)
		_IQ(1.0)-1, 							//Regenerate current limit  map Delta1 ratio 0~100%--IQ(0)~IQ(1.0)
		_IQ(1.0)-1, 					//Regenerate current limit  map Delta2 ratio 0~100%--IQ(0)~IQ(1.0)
		_IQ(1.0)-1, 			//Regenerate current limit  map Delta4 ratio 0~100%--IQ(0)~IQ(1.0)
		_IQ(1.0)-1,		//Regenerate current limit  map Delta8 ratio 0~100%--IQ(0)~IQ(1.0)
		_IQ(1.0)-1, 									//The amount of high speed power the controller will allow 0~100%--IQ(0)~IQ(1.0)
		_IQ(1.0)-1, 									//The control loop gains for field weakening.
		_IQ(0.03125),											//The Ratio increment from WkBaseHZ to TypicalHZ
		_IQ(393.0/100000),						//The motor rotor thermal coe. Cuprum--_IQ(0.00393),Aluminum--_IQ(0.00429)
		_IQ(58.0 /(1000*STD_TORQUE_K)),		/* Torque Coefficient */
		_IQ(0.0244		/STD_INERTIA),		/* Inertia */
		_IQ(0.1300		/STD_RESIST),		/* R */
		_IQ(0.0300				/STD_T),			/* t0=L/R,stator time constant */
		_IQ(1.75				/STD_T),			  /* t0=L/R,rotor time constant */
	//var
		_IQ((60.0+10.0*1)				/STD_SPEEDRPM),  /* Q24 = STD_FRQ  RatedHZ + DeltaHZ * 1 */
		_IQ((60.0+10.0*2)				/STD_SPEEDRPM),  /* Q24 = STD_FRQ  RatedHZ + DeltaHZ * 2 */
		_IQ((60.0+10.0*4)				/STD_SPEEDRPM),  /* Q24 = STD_FRQ  RatedHZ + DeltaHZ * 4 */
		_IQ((60.0+10.0*8)				/STD_SPEEDRPM),  /* Q24 = STD_FRQ  RatedHZ + DeltaHZ * 8 */
		_IQ(0*STD_SPEEDRPM/(10.0*1)),				/* Drive Kcoe for RatedHZ~RatedHZ1  */
		_IQ(0*STD_SPEEDRPM/(10.0*2)),				/* Drive Kcoe for RatedHZ1~RatedHZ2 */
		_IQ(0*STD_SPEEDRPM/(10.0*4)),				/* Drive Kcoe for RatedHZ2~RatedHZ4 */
		_IQ(0*STD_SPEEDRPM/(10.0*8)),				/* Drive Kcoe for RatedHZ4~RatedHZ8 */
		_IQ(0*STD_SPEEDRPM/(10.0*1)),				/* Regen Kcoe for RatedHZ~RatedHZ1  */
		_IQ(0*STD_SPEEDRPM/(10.0*2)),				/* Regen Kcoe for RatedHZ1~RatedHZ2 */
		_IQ(0*STD_SPEEDRPM/(10.0*4)),				/* Regen Kcoe for RatedHZ2~RatedHZ4 */
		_IQ(0*STD_SPEEDRPM/(10.0*8)),				/* Regen Kcoe for RatedHZ4~RatedHZ8 */
		_IQ(0),														/* UphillRatioKoe = (UphillCurrent/DriveCurrent-1.0)/(WkBaseHZ - HFLUX_START_FRQ)*/
		_IQ(0),														/* WkAdjRatioKoe = WkAdjRatio/(TypicalHZ - WkBaseHZ)*/
		_IQ(1.0),													//RotorTCoeK = _IQ(1.0)+RotorTCoe*(Tmotor-20)
		_IQ16(8.0),									/* Min Speed */
	},
	/****** MotorType 2 ******/
	/*TYPE:  */
	/*PART NO:  */
	/*POWER: 550W, 45A; 20 POLE PARES  Hall 6*20 = 120pls / r */
	/*USER:  ˶����� 550W ���裺0.06  ��������120A ��ų���B��0.0058 ���ת�٣�4000rpm*/
	{
		2,																	/* ����� 0:���������λ���趨��1~n�������������Ԥ������ cMotorParaRom[n]*/
		STD_INC30,													/* Encoder M4 line number */
		2360,																/* Calib angle */
		20,																	/* Q0, motor pole pairs*/
		_IQ(4000.0				/STD_SPEEDRPM),			    /* Q24 = STD_SPEEDRPM  Typical speedrpm, as standard for acc/dec rate */
		_IQ(60.0				/STD_SPEEDRPM),			    /* Q24 = STD_SPEEDRPM Base speedrpm for weak flux */
		_IQ(2000.0				/STD_SPEEDRPM),			    /* Q24 = STD_SPEEDRPM Base speedrpm for current limit */
		_IQ(10.0				/STD_SPEEDRPM),			    /* Q24 = STD_SPEEDRPM Delta inc */
		_IQ(90.0/ STD_CURRENT), /* Q24 = STD_CURRENT RateCurrent  */
		_IQ(90.0/ STD_CURRENT), /* Q24 = STD_CURRENT DriveCurrent  */
		_IQ(90.0 / STD_CURRENT), /* Q24 = STD_CURRENT  DriveFlux*/
		_IQ(110.0 / STD_CURRENT), /* Q24 = STD_CURRENT UphillCurrent */
		_IQ(110.0 / STD_CURRENT), /* Q24 = STD_CURRENT UphillFlux */
		_IQ(45.0 / STD_CURRENT), /* Q24 = STD_CURRENT RegenCurrent  */
		_IQ(45.0 / STD_CURRENT), /* Q24 = STD_CURRENT RegenFlux */
		_IQ(45.0 / STD_CURRENT), /* Q24 = STD_CURRENT BrakeCurrent */
		_IQ(45.0 / STD_CURRENT), /* Q24 = STD_CURRENT  BrakeFlux*/
		_IQ(0.5)-1, 									//Drive current limit  map nominal ratio 0~100%--IQ(0)~IQ(1.0)
		_IQ(0.1)-1, 							//Drive current limit  map Delta1 ratio 0~100%--IQ(0)~IQ(1.0)
		_IQ(1.0)-1, 					//Drive current limit  map Delta2 ratio 0~100%--IQ(0)~IQ(1.0)
		_IQ(1.0)-1, 			//Drive current limit  map Delta4 ratio 0~100%--IQ(0)~IQ(1.0)
		_IQ(1.0)-1,		//Drive current limit  map Delta8 ratio 0~100%--IQ(0)~IQ(1.0)
		_IQ(1.0)-1, 									//Regenerate current limit  map nominal ratio 0~100%--IQ(0)~IQ(1.0)
		_IQ(1.0)-1, 							//Regenerate current limit  map Delta1 ratio 0~100%--IQ(0)~IQ(1.0)
		_IQ(1.0)-1, 					//Regenerate current limit  map Delta2 ratio 0~100%--IQ(0)~IQ(1.0)
		_IQ(1.0)-1, 			//Regenerate current limit  map Delta4 ratio 0~100%--IQ(0)~IQ(1.0)
		_IQ(1.0)-1,		//Regenerate current limit  map Delta8 ratio 0~100%--IQ(0)~IQ(1.0)
		_IQ(1.0)-1, 									//The amount of high speed power the controller will allow 0~100%--IQ(0)~IQ(1.0)
		_IQ(1.0)-1, 									//The control loop gains for field weakening.
		_IQ(0.03125),											//The Ratio increment from WkBaseHZ to TypicalHZ
		_IQ(393.0/100000),						//The motor rotor thermal coe. Cuprum--_IQ(0.00393),Aluminum--_IQ(0.00429)
		_IQ(58.0 /(1000*STD_TORQUE_K)),		/* Torque Coefficient */
		_IQ(0.0244		/STD_INERTIA),		/* Inertia */
		_IQ(0.0600		/STD_RESIST),		/* R */
		_IQ(0.0300				/STD_T),			/* t0=L/R,stator time constant */
		_IQ(1.75				/STD_T),			  /* t0=L/R,rotor time constant */
	//var
		_IQ((60.0+10.0*1)				/STD_SPEEDRPM),  /* Q24 = STD_FRQ  RatedHZ + DeltaHZ * 1 */
		_IQ((60.0+10.0*2)				/STD_SPEEDRPM),  /* Q24 = STD_FRQ  RatedHZ + DeltaHZ * 2 */
		_IQ((60.0+10.0*4)				/STD_SPEEDRPM),  /* Q24 = STD_FRQ  RatedHZ + DeltaHZ * 4 */
		_IQ((60.0+10.0*8)				/STD_SPEEDRPM),  /* Q24 = STD_FRQ  RatedHZ + DeltaHZ * 8 */
		_IQ(0*STD_SPEEDRPM/(10.0*1)),				/* Drive Kcoe for RatedHZ~RatedHZ1  */
		_IQ(0*STD_SPEEDRPM/(10.0*2)),				/* Drive Kcoe for RatedHZ1~RatedHZ2 */
		_IQ(0*STD_SPEEDRPM/(10.0*4)),				/* Drive Kcoe for RatedHZ2~RatedHZ4 */
		_IQ(0*STD_SPEEDRPM/(10.0*8)),				/* Drive Kcoe for RatedHZ4~RatedHZ8 */
		_IQ(0*STD_SPEEDRPM/(10.0*1)),				/* Regen Kcoe for RatedHZ~RatedHZ1  */
		_IQ(0*STD_SPEEDRPM/(10.0*2)),				/* Regen Kcoe for RatedHZ1~RatedHZ2 */
		_IQ(0*STD_SPEEDRPM/(10.0*4)),				/* Regen Kcoe for RatedHZ2~RatedHZ4 */
		_IQ(0*STD_SPEEDRPM/(10.0*8)),				/* Regen Kcoe for RatedHZ4~RatedHZ8 */
		_IQ(0),														/* UphillRatioKoe = (UphillCurrent/DriveCurrent-1.0)/(WkBaseHZ - HFLUX_START_FRQ)*/
		_IQ(0),														/* WkAdjRatioKoe = WkAdjRatio/(TypicalHZ - WkBaseHZ)*/
		_IQ(1.0),													//RotorTCoeK = _IQ(1.0)+RotorTCoe*(Tmotor-20)
		_IQ16(8.0),									/* Min Speed */
	},
};
#endif

	
#if (VOLTAGE_LEVEL == _VOLTAGE_48V)  //For 48V driver
struct MOTOR_PARA cMotorParaRom[1] = 
{
	/****** MotorType 1 ******/
	/*TYPE: NUOLI xjg BLDC  MOTOR */
	/*PART NO:  */
	/*POWER: 600W, 45A; 20 POLE PARES  Hall 6*20 = 120pls / r */
	/*USER:  ŵ��������챵�� */
	{
		1,																	/* ����� 0:���������λ���趨��1~n�������������Ԥ������ cMotorParaRom[n]*/
		STD_INC30,													/* Encoder M4 line number */
		2360,																/* Calib angle */
		20,																	/* Q0, motor pole pairs*/
		_IQ(120.0				/STD_SPEEDRPM),			    /* Q24 = STD_SPEEDRPM  Typical speedrpm, as standard for acc/dec rate */
		_IQ(60.0				/STD_SPEEDRPM),			    /* Q24 = STD_SPEEDRPM Base speedrpm for weak flux */
		_IQ(60.0				/STD_SPEEDRPM),			    /* Q24 = STD_SPEEDRPM Base speedrpm for current limit */
		_IQ(10.0				/STD_SPEEDRPM),			    /* Q24 = STD_SPEEDRPM Delta inc */
		_IQ(15.0/ STD_CURRENT), /* Q24 = STD_CURRENT DriveCurrent  */
		_IQ(15.0 / STD_CURRENT), /* Q24 = STD_CURRENT  DriveFlux*/
		_IQ(45.0 / STD_CURRENT), /* Q24 = STD_CURRENT UphillCurrent */
		_IQ(45.0 / STD_CURRENT), /* Q24 = STD_CURRENT UphillFlux */
		_IQ(45.0 / STD_CURRENT), /* Q24 = STD_CURRENT RegenCurrent  */
		_IQ(45.0 / STD_CURRENT), /* Q24 = STD_CURRENT RegenFlux */
		_IQ(45.0 / STD_CURRENT), /* Q24 = STD_CURRENT BrakeCurrent */
		_IQ(45.0 / STD_CURRENT), /* Q24 = STD_CURRENT  BrakeFlux*/
		_IQ(1.0)-1, 									//Drive current limit  map nominal ratio 0~100%--IQ(0)~IQ(1.0)
		_IQ(1.0)-1, 							//Drive current limit  map Delta1 ratio 0~100%--IQ(0)~IQ(1.0)
		_IQ(1.0)-1, 					//Drive current limit  map Delta2 ratio 0~100%--IQ(0)~IQ(1.0)
		_IQ(1.0)-1, 			//Drive current limit  map Delta4 ratio 0~100%--IQ(0)~IQ(1.0)
		_IQ(1.0)-1,		//Drive current limit  map Delta8 ratio 0~100%--IQ(0)~IQ(1.0)
		_IQ(1.0)-1, 									//Regenerate current limit  map nominal ratio 0~100%--IQ(0)~IQ(1.0)
		_IQ(1.0)-1, 							//Regenerate current limit  map Delta1 ratio 0~100%--IQ(0)~IQ(1.0)
		_IQ(1.0)-1, 					//Regenerate current limit  map Delta2 ratio 0~100%--IQ(0)~IQ(1.0)
		_IQ(1.0)-1, 			//Regenerate current limit  map Delta4 ratio 0~100%--IQ(0)~IQ(1.0)
		_IQ(1.0)-1,		//Regenerate current limit  map Delta8 ratio 0~100%--IQ(0)~IQ(1.0)
		_IQ(1.0)-1, 									//The amount of high speed power the controller will allow 0~100%--IQ(0)~IQ(1.0)
		_IQ(1.0)-1, 									//The control loop gains for field weakening.
	  _IQ(1.0),											//The Ratio increment from WkBaseHZ to TypicalHZ
		_IQ(393.0/100000),						//The motor rotor thermal coe. Cuprum--_IQ(0.00393),Aluminum--_IQ(0.00429)
		_IQ(50.0/12.0 /STD_TORQUE_K),		/* Torque Coefficient */
		_IQ(0.0244		/STD_INERTIA),		/* Inertia */
		_IQ(0.2000				/STD_RESIST),		/* R */
		_IQ(0.0300				/STD_T),			/* t0=L/R,stator time constant */
		_IQ(1.75				/STD_T),			  /* t0=L/R,rotor time constant */
	//var
		_IQ((60.0+10.0*1)				/STD_SPEEDRPM),  /* Q24 = STD_FRQ  RatedHZ + DeltaHZ * 1 */
		_IQ((60.0+10.0*2)				/STD_SPEEDRPM),  /* Q24 = STD_FRQ  RatedHZ + DeltaHZ * 2 */
		_IQ((60.0+10.0*4)				/STD_SPEEDRPM),  /* Q24 = STD_FRQ  RatedHZ + DeltaHZ * 4 */
		_IQ((60.0+10.0*8)				/STD_SPEEDRPM),  /* Q24 = STD_FRQ  RatedHZ + DeltaHZ * 8 */
		_IQ(0*STD_SPEEDRPM/(10.0*1)),				/* Drive Kcoe for RatedHZ~RatedHZ1  */
		_IQ(0*STD_SPEEDRPM/(10.0*2)),				/* Drive Kcoe for RatedHZ1~RatedHZ2 */
		_IQ(0*STD_SPEEDRPM/(10.0*4)),				/* Drive Kcoe for RatedHZ2~RatedHZ4 */
		_IQ(0*STD_SPEEDRPM/(10.0*8)),				/* Drive Kcoe for RatedHZ4~RatedHZ8 */
		_IQ(0*STD_SPEEDRPM/(10.0*1)),				/* Regen Kcoe for RatedHZ~RatedHZ1  */
		_IQ(0*STD_SPEEDRPM/(10.0*2)),				/* Regen Kcoe for RatedHZ1~RatedHZ2 */
		_IQ(0*STD_SPEEDRPM/(10.0*4)),				/* Regen Kcoe for RatedHZ2~RatedHZ4 */
		_IQ(0*STD_SPEEDRPM/(10.0*8)),				/* Regen Kcoe for RatedHZ4~RatedHZ8 */
		_IQ(0),														/* UphillRatioKoe = (UphillCurrent/DriveCurrent-1.0)/(WkBaseHZ - HFLUX_START_FRQ)*/
		_IQ(0),														/* WkAdjRatioKoe = WkAdjRatio/(TypicalHZ - WkBaseHZ)*/
		_IQ(1.0),													//RotorTCoeK = _IQ(1.0)+RotorTCoe*(Tmotor-20)
	},
};
#endif

//For 80V driver
#if (   (VOLTAGE_LEVEL == _VOLTAGE_72V) \
			||(VOLTAGE_LEVEL == _VOLTAGE_80V) \
			||(VOLTAGE_LEVEL == _VOLTAGE_88V)  )
struct MOTOR_PARA cMotorParaRom[1] = 
{
	/****** MotorType 1 ******/
	/*TYPE: 3 PHASE AC MOTOR */
	/*PART NO:  */
	/*POWER: */
	/*USER:  ������80Vʵ��̨ */
};
#endif
/*******************************************************************************
*
* �նȲ����趨��
*
* �˸��Բ�������SigmaII�����趨
*
*******************************************************************************/
static const struct RIGID_PARA gRigidParaRom[13] = 
{
	/* �ն�,λ������,�ٶ�����,����ʱ��,ת���˲���1��ֹƵ��,ת���˲���2��ֹƵ��*/
	{	 0,		 	10,			10,	    100,	 	64,		 	96		},
	{	 1,		 	15,	 		15,			70,		  70,	  	105		},
	{	 2,		 	20,	 		20,			50,		  80,	 		120		},
	{	 3,		 	30,	 		30,			40,		 	120,	 	180		},
	{	 4,		 	40,	 		40,			25,		 	160,	 	240		},
	{	 5,			50,			50,		 	22,			200,		300		},
	{	 6,		 	60,	 		60,			20,		 	230,	 	345		},
	{	 7,		 	80,	 		80,			18,		 	300,	 	450		},
	{	 8,			100,		100,		15,		 	400,	 	600		},
	{	 9,			120,		120,		10,		 	520,	 	780		},
	{	10,			140,		140,	 	9,		 	670,		1000	},
	{	11,			160,		160,	 	8,		 	800,		1200	},
	{	12,			200,		200,	 	6,			1060,		1600	}
};

/*******************************************************************************
* Name: ParaToCurrentPID
* Description: 
* Input: 
* Output:
*
* Author:
* Date:
* Revision:
*******************************************************************************/
static void ParaToCurrentPID(void)
{
	_iq kp;
	_iq ki;
	_iq t0;
	_iq resist;
	_iq td;

	/* g_servo_para.motor_para.resistΪ������� */
	/* resistΪID��IQ���� */
	resist = gCRam.SvPa.MotorPara.Resist;
	t0 = gCRam.SvPa.MotorPara.Tstator;
//	td = _IQ(TS/STD_T);
	td = _IQ(TS/STD_T);
	/* delay ~= ??us, ��ֵ��PWM���ں͵��������˲���·���� */

	/* Kp = L / (2 * Td) = t0 * R / (2 * Td) */
	kp = _IQdiv(_IQmpy(_IQmpy(t0, resist), _IQ(STD_CURRENT/STD_VOLTAGE/2)), td);
	/* Ki = T / t0 */
	ki = _IQdiv(_IQ(TS), t0);
	gCurrentLoop.IdPID.Kp = (kp >> 6);
	gCurrentLoop.IdPID.Ki = (ki >> 6);
	gCurrentLoop.IdPID.Kc = (ki >> 6);
	gCurrentLoop.IqPID.Kp = (kp >> 6);
	gCurrentLoop.IqPID.Ki = (ki >> 6);
	gCurrentLoop.IqPID.Kc = (ki >> 6);
}
/*******************************************************************************
* Name: ParaToSpeedPID
* Description: 
* Input: 
* Output:
*
* Author:
* Date:
* Revision:
*******************************************************************************/
static void ParaToSpeedPID(void)
{
	_iq wc;
	_iq inertia;
	_iq kinertia;
	_iq kobserver;
	_iq ktorque;
	_iq kit;
	_iq kp;
	_iq ki;
	_iq torlimit;

	/* ��ֹƵ��ת��ΪQ��ʽ����ֵ��λ */
	wc = (INT32S)gCRam.SvPa.SpeedWc * _IQ(1.0/STD_WC);
	/* Բ��Ƶ��ת��Ϊ��Ƶ�� */
	wc = _IQmpy(wc, _IQ(2 * PI));
	/* ����Ļ���ʱ�䳣����msΪ��λ����ת��Ϊ���ʵ�λ(s)Q��ʽ����ֵ��λ�� */
	kit = (INT32S)gCRam.SvPa.SpeedWit * _IQ(1.0/(1000.0 * STD_T));

	/* �������(Q��ʽ����ֵ��λ) */
	inertia = gCRam.SvPa.MotorPara.Inertia;
	/* ת��ϵ��(Q��ʽ����ֵ��λ) */
	ktorque = gCRam.SvPa.MotorPara.KTorque;

	/* ������(Q��ʽ) */
	kinertia = (INT32S)gCRam.SvPa.IntertiaRatio * _IQ(1.0/100.0);
	/* ������� + ���ع��� (Q��ʽ����ֵ��λ) */
	inertia = _IQmpy(inertia, kinertia);

	/* kp = wc * J / k */
	kp = _IQmpy(_IQdiv(_IQmpy(wc, inertia), ktorque), 
		_IQ(STD_SPEED / STD_CURRENT * STD_WC * STD_INERTIA / STD_TORQUE_K));
	/* ki = ��������/����ʱ�䳣�� (Q��ʽ����ֵ��λ) */
	ki = _IQdiv(_IQ(TS), kit);

	/* �����ٶȻ�PID���� */
	gSpeedLoop.PID.Kp = kp;
	gSpeedLoop.PID.Ki = ki;
	gSpeedLoop.PID.Kc = ki;
	/* ���ݹ������趨�ٶȻ��������ֵ */
	
	/* CCW */
	torlimit = gCRam.SvPa.MotorPara.DriveCurrent;
	if (torlimit > _IQ(0.8))
	{
		torlimit = _IQ(0.8);
	}
	gSpeedLoop.PID.OutMax = torlimit;
	gSpeedLoop.PidOutMax = torlimit;
	/* �������תת��, Q16, Nm */
	gKernelCtl.MaxCCWTorque =_IQmpy(torlimit,gCRam.SvPa.MotorPara.KTorque);
	gKernelCtl.MaxCCWTorque =_IQmpy(gKernelCtl.MaxCCWTorque,_IQ16(STD_TORQUE_K * STD_CURRENT));

	/* CW */
	torlimit = gCRam.SvPa.MotorPara.DriveCurrent;
	if (torlimit > _IQ(0.8))
	{
		torlimit = _IQ(0.8);
	}
	gSpeedLoop.PID.OutMin = -torlimit;
	gSpeedLoop.PidOutMin = -torlimit;
	/* ���������תת��, Q16, Nm */
	gKernelCtl.MaxCWTorque =_IQmpy(torlimit,gCRam.SvPa.MotorPara.KTorque);
	gKernelCtl.MaxCWTorque =_IQmpy(gKernelCtl.MaxCWTorque,_IQ16(STD_TORQUE_K * STD_CURRENT));

	/* �����ٶȹ۲�����PI�������Ĳ��� */
	gSpeedLoop.Observer.PID = gSpeedLoop.PID;
	/* �ٶȹ۲���ϵ��(�ٷֱ�)ת��ΪQ��ʽ */
	kobserver = (INT32S)gCRam.SvPa.ObserverK * _IQ(1.0/100.0);
	gSpeedLoop.Observer.PID.Kp = _IQmpy(kp, kobserver);
	gSpeedLoop.Observer.PID.Ki = _IQmpy(ki, kobserver);
}
/*******************************************************************************
* Name: ParaToPositionPID
* Description: 
* Input: 
* Output:
*
* Author:
* Date:
* Revision:
*******************************************************************************/
static void ParaToPositionPID(void)
{
//	_iq kp;

	/* �����ٶȻ�PID���� */
//	gPositionLoop.PID.Kp = gCRam.SvPa.PosKp;

	/* �������ֵ */
	//Limit the max pos
//	gCRam.SvPa.PosCmdLimitCw = Encoder2CmdUnit(gCRam.SvPa.EncoderLineNum);		
	
}
/*******************************************************************************
* Name: ParaToTorqueFilter
* Description: 
* Input: 
* Output:
*
* Author:
* Date:
* Revision:
*******************************************************************************/
static void ParaToTorqueFilter(void)
{
	TORQUE_FILTER *pTorque = &gSpeedLoop.TorqueFilter;
//	TORQUE_FILTER *pFlux = &gSpeedLoop.FluxFilter;

	/* LPF1: 1 order butterworth */
	/* ת���˲���1��һ����ֹƵ��ת��ΪQ��ʽ */
	pTorque->LPF1.w = (INT32S)gCRam.SvPa.TorFilt1W * _IQ(1.0/FS);
	pTorque->LPF1.Design(&pTorque->LPF1);

	/* LPF2: 2 order butterworth */
	/* ת���˲���2��һ����ֹƵ��ת��ΪQ��ʽ */
	pTorque->LPF2.w = (INT32S)gCRam.SvPa.TorFilt2W * _IQ(1.0/FS);
	/* ת���˲���2Ʒ������(�ٷֱ�)ת��ΪQ��ʽ */
	pTorque->LPF2.Q = (INT32S)gCRam.SvPa.TorFilt2Q * _IQ(1.0/100.0);
	pTorque->LPF2.Design(&pTorque->LPF2);

	/* Notch1 */
	/* �ݲ��˲���1��һ���������Ƶ��ת��ΪQ��ʽ */
	pTorque->Notch1.w = (INT32S)gCRam.SvPa.NotchFilt1W * _IQ(1.0/FS);
	/* �ݲ��˲���1Ʒ������(�ٷֱ�)ת��ΪQ��ʽ */
	pTorque->Notch1.Q = (INT32S)gCRam.SvPa.NotchFilt1Q * _IQ(1.0/100.0);
	pTorque->Notch1.Design(&pTorque->Notch1);

	/* Notch2 */
	/* �ݲ��˲���2��һ���������Ƶ��ת��ΪQ��ʽ */
	pTorque->Notch2.w = (INT32S)gCRam.SvPa.NotchFilt2W * _IQ(1.0/FS);
	/* �ݲ��˲���2Ʒ������(�ٷֱ�)ת��ΪQ��ʽ */
	pTorque->Notch2.Q = (INT32S)gCRam.SvPa.NotchFilt2Q * _IQ(1.0/100.0);
	pTorque->Notch2.Design(&pTorque->Notch2);

	/* LPF1: 1 order butterworth */
	/* ת���˲���1��һ����ֹƵ��ת��ΪQ��ʽ */
//	pFlux->LPF1.w = (INT32S)gCRam.SvPa.TorFilt1W * _IQ(1.0/FS);
//	pFlux->LPF1.Design(&pFlux->LPF1);

	/* LPF2: 2 order butterworth */
	/* ת���˲���2��һ����ֹƵ��ת��ΪQ��ʽ */
//	pFlux->LPF2.w = (INT32S)gCRam.SvPa.TorFilt2W * _IQ(1.0/FS);
	/* ת���˲���2Ʒ������(�ٷֱ�)ת��ΪQ��ʽ */
//	pFlux->LPF2.Q = (INT32S)gCRam.SvPa.TorFilt2Q * _IQ(1.0/100.0);
//	pFlux->LPF2.Design(&pFlux->LPF2);

	/* Notch1 */
	/* �ݲ��˲���1��һ���������Ƶ��ת��ΪQ��ʽ */
//	pFlux->Notch1.w = (INT32S)gCRam.SvPa.NotchFilt1W * _IQ(1.0/FS);
	/* �ݲ��˲���1Ʒ������(�ٷֱ�)ת��ΪQ��ʽ */
//	pFlux->Notch1.Q = (INT32S)gCRam.SvPa.NotchFilt1Q * _IQ(1.0/100.0);
//	pFlux->Notch1.Design(&pFlux->Notch1);

	/* Notch2 */
	/* �ݲ��˲���2��һ���������Ƶ��ת��ΪQ��ʽ */
//	pFlux->Notch2.w = (INT32S)gCRam.SvPa.NotchFilt2W * _IQ(1.0/FS);
	/* �ݲ��˲���2Ʒ������(�ٷֱ�)ת��ΪQ��ʽ */
//	pFlux->Notch2.Q = (INT32S)gCRam.SvPa.NotchFilt2Q * _IQ(1.0/100.0);
//	pFlux->Notch2.Design(&pFlux->Notch2);
}

/*******************************************************************************
* Name: ParaToPanSpdT
* Description: 
* Input: 
* Output:
*
* Author:
* Date:
* Revision:
*******************************************************************************/
void ParaToPanSpdT(void)
{
	_iq k;
	SPEED_LOOP* pSpeedLoop = &gSpeedLoop;

	k = gCRam.SvPa.MotorPara.TypicalHZ;
	/****** Hs & Ls ******/
	/*Speed upon which the HS parameters will be used. */
	pSpeedLoop->HighSpeed = _IQmpy(k, (_iq)gCRam.SvPa.HighSpeedRatio * _IQ(1.0/STD_PERCENT));
	/*Speed below which the LS parameters will be used. */
	pSpeedLoop->LowSpeed = _IQmpy(k, (_iq)gCRam.SvPa.LowSpeedRatio * _IQ(1.0/STD_PERCENT));
	/*Para protect */
	if ((pSpeedLoop->HighSpeed - pSpeedLoop->LowSpeed) < _IQ(10.0/STD_SPEEDRPM))
	{
		pSpeedLoop->HighSpeed = pSpeedLoop->LowSpeed + _IQ(10.0/STD_SPEEDRPM);
	}
	pSpeedLoop->SmallSpeed = _IQmpy(k, (_iq)gCRam.SvPa.SmallSpeedRatio * _IQ(1.0/STD_PERCENT));
	if ((pSpeedLoop->LowSpeed - pSpeedLoop->SmallSpeed) < _IQ(10.0/STD_SPEEDRPM))
	{
		pSpeedLoop->SmallSpeed = pSpeedLoop->LowSpeed - _IQ(10.0/STD_SPEEDRPM);
	}
	/****** Throttle small & full & koe ******/
	/*Para for small amount throttle */
	pSpeedLoop->SpdSmallThrottle = pSpeedLoop->SmallSpeed;
	/*Para for full amount throttle, Hz, 90%*/
	pSpeedLoop->SpdFullThrottle = _IQmpy(k, _IQ(0.9));
	/* Koe = 1.0/(SpdFullThrottle-SpdSmallThrottle) */
	pSpeedLoop->SpdThrottleKoe = _IQdiv(_IQ(1.0), (pSpeedLoop->SpdFullThrottle - pSpeedLoop->SpdSmallThrottle));

	
	/****** Brake Limit ******/
	pSpeedLoop->BrakeCmdSmall = _IQ(0.10);
	pSpeedLoop->BrakeCmdLow = _IQ(0.25);
	pSpeedLoop->BrakeCmdFull = _IQ(0.90);
	pSpeedLoop->BrakeCmdKoeSL = _IQdiv(_IQ(1.0), (pSpeedLoop->BrakeCmdLow - pSpeedLoop->BrakeCmdSmall));
	pSpeedLoop->BrakeCmdKoeLF = _IQdiv(_IQ(1.0), (pSpeedLoop->BrakeCmdFull - pSpeedLoop->BrakeCmdLow));
	
	/************************* ACC & DEC Rate **************************/
	/*Acc rate when full throttle is applied at high vehicle speeds*/
	if (gCRam.SvPa.FullAccTimeHs == 0)
	{
		pSpeedLoop->FullAccRateHs = k/(FS/1000);
	}
	else
	{
		pSpeedLoop->FullAccRateHs = k/((INT32U)gCRam.SvPa.FullAccTimeHs*(FS/1000));
	}
	if(pSpeedLoop->FullAccRateHs == 0)
		pSpeedLoop->FullAccRateHs = 1;

	/*Acc rate when full throttle is applied at low vehicle speeds*/	
	if (gCRam.SvPa.FullAccTimeLs == 0)
	{
		pSpeedLoop->FullAccRateLs = k/(FS/1000);
	}
	else
	{
		pSpeedLoop->FullAccRateLs = k/((INT32U)gCRam.SvPa.FullAccTimeLs*(FS/1000));
	}
	if(pSpeedLoop->FullAccRateLs == 0)
		pSpeedLoop->FullAccRateLs = 1;
	/*Koe for speed between Ls & Hs  Koe = (RateHs-RateLs)/(Hs-Ls)*/
	pSpeedLoop->FullAccRateKoe = _IQdiv((pSpeedLoop->FullAccRateHs - pSpeedLoop->FullAccRateLs), (pSpeedLoop->HighSpeed - pSpeedLoop->LowSpeed));

	/*Acc rate when a small amount throttle is applied */
	if (gCRam.SvPa.LowAccTime == 0)
	{
		pSpeedLoop->LowAccRate = k/(FS/1000);
	}
	else
	{
		pSpeedLoop->LowAccRate = k/((INT32U)gCRam.SvPa.LowAccTime*(FS/1000));
	}
	if(pSpeedLoop->LowAccRate == 0)
		pSpeedLoop->LowAccRate = 1;

	/*Dec rate when the throttle is released to neutral at high vehicle speeds.*/
	if (gCRam.SvPa.NeutralDecTimeHs == 0)
	{
		pSpeedLoop->NeutralDecRateHs = k/(FS/1000);
	}
	else
	{
		pSpeedLoop->NeutralDecRateHs = k/((INT32U)gCRam.SvPa.NeutralDecTimeHs*(FS/1000));
	}
	if(pSpeedLoop->NeutralDecRateHs == 0)
		pSpeedLoop->NeutralDecRateHs = 1;

	/*Dec rate when the throttle is released to neutral at low vehicle speeds.*/
	if (gCRam.SvPa.NeutralDecTimeLs == 0)
	{
		pSpeedLoop->NeutralDecRateLs = k/(FS/1000);
	}
	else
	{
		pSpeedLoop->NeutralDecRateLs = k/((INT32U)gCRam.SvPa.NeutralDecTimeLs*(FS/1000));
	}
	if(pSpeedLoop->NeutralDecRateLs == 0)
		pSpeedLoop->NeutralDecRateLs = 1;
  /*Koe for speed between Ls & Hs  Koe = (RateHs-RateLs)/(Hs-Ls)*/
	pSpeedLoop->NeutralDecRateKoeLH = _IQdiv((pSpeedLoop->NeutralDecRateHs - pSpeedLoop->NeutralDecRateLs), (pSpeedLoop->HighSpeed - pSpeedLoop->LowSpeed));

	/*Dec rate when full brake or full throttle in opposite direction at high vehicle speeds.*/
	if (gCRam.SvPa.FullBrakeDecTimeHs == 0)
	{
		pSpeedLoop->FullBrakeDecRateHs = k/(FS/1000);
	}
	else
	{
		pSpeedLoop->FullBrakeDecRateHs = k/((INT32U)gCRam.SvPa.FullBrakeDecTimeHs*(FS/1000));
	}
	if(pSpeedLoop->FullBrakeDecRateHs == 0)
		pSpeedLoop->FullBrakeDecRateHs = 1;

	/*Dec rate when full brake or full throttle in opposite direction at low vehicle speeds.*/
	if (gCRam.SvPa.FullBrakeDecTimeLs == 0)
	{
		pSpeedLoop->FullBrakeDecRateLs = k/(FS/1000);
	}
	else
	{
		pSpeedLoop->FullBrakeDecRateLs = k/((INT32U)gCRam.SvPa.FullBrakeDecTimeLs*(FS/1000));
	}
	if(pSpeedLoop->FullBrakeDecRateLs == 0)
		pSpeedLoop->FullBrakeDecRateLs = 1;
	/*Koe for speed between Ls & Hs  Koe = (RateHs-RateLs)/(Hs-Ls)*/
	pSpeedLoop->FullBrakeDecRateKoe = _IQdiv((pSpeedLoop->FullBrakeDecRateHs - pSpeedLoop->FullBrakeDecRateLs), (pSpeedLoop->HighSpeed - pSpeedLoop->LowSpeed));;;
	
	/*Dec rate when a small amount of brake brake or a small amount of brake throttle in opposite direction at all vehicle speeds.*/
	if (gCRam.SvPa.LowBrakeDecTimeHLs == 0)
	{
		pSpeedLoop->LowBrakeDecRateHLs = k/(FS/1000);
	}
	else
	{
		pSpeedLoop->LowBrakeDecRateHLs = k/((INT32U)gCRam.SvPa.LowBrakeDecTimeHLs*(FS/1000));
	}
	if(pSpeedLoop->LowBrakeDecRateHLs == 0)
		pSpeedLoop->LowBrakeDecRateHLs = 1;
	
	pSpeedLoop->NeutralDecRateSmall = pSpeedLoop->LowBrakeDecRateHLs;
  /*Koe for speed between Small & Ls  Koe = (RateLs-RateSmall)/(Ls-Small)*/
	pSpeedLoop->NeutralDecRateKoeSL = _IQdiv((pSpeedLoop->NeutralDecRateLs - pSpeedLoop->NeutralDecRateSmall), (pSpeedLoop->LowSpeed - pSpeedLoop->SmallSpeed));
	
	/*Dec rate when the throttle is reduced without being released to neutral */
	if (gCRam.SvPa.PartialDecTimeHLs == 0)
	{
		pSpeedLoop->PartialDecRateHLs = k/(FS/1000);
	}
	else
	{
		pSpeedLoop->PartialDecRateHLs = k/((INT32U)gCRam.SvPa.PartialDecTimeHLs*(FS/1000));
	}
	if(pSpeedLoop->PartialDecRateHLs == 0)
		pSpeedLoop->PartialDecRateHLs = 1;


	if ((pSpeedLoop->EndAccRateApproach = _IQmpy(pSpeedLoop->LowAccRate,  gCRam.SvPa.EntryRateApproach*_IQ(1.0/100))) == 0)
		pSpeedLoop->EndAccRateApproach = 1;
	if ((pSpeedLoop->EndDecRateApproach = _IQmpy(pSpeedLoop->NeutralDecRateSmall,gCRam.SvPa.EntryRateApproach*_IQ(1.0/100))) == 0)
		pSpeedLoop->EndDecRateApproach = 1;
	if ((pSpeedLoop->EndAccRateStop = _IQmpy(pSpeedLoop->LowAccRate,  gCRam.SvPa.EntryRateStop*_IQ(1.0/100))) == 0)
		pSpeedLoop->EndAccRateStop = 1;
	if ((pSpeedLoop->EndDecRateStop = _IQmpy(pSpeedLoop->NeutralDecRateSmall,gCRam.SvPa.EntryRateStop*_IQ(1.0/100))) == 0)
		pSpeedLoop->EndAccRateStop = 1;

	pSpeedLoop->SoftSpeedThreshold = gCRam.SvPa.SoftStopSpeed * _IQ(1.0/(STD_SPEEDRPM));

	/************ ACC & DEC forward control max value limit */
	gSpeedLoop.UfMovPara = (_iq)gCRam.SvPa.KvffCurrent * _IQ((SQRT2*PH3_TO_PH2)/(STD_CURRENT*10));
	/* Variable: Move forward control inc step */
	if (gCRam.SvPa.KvffBuildTime == 0)
	{
		gSpeedLoop.UfMovIncPara = gSpeedLoop.UfMovPara/(FS/1000);
	}
	else
	{
		gSpeedLoop.UfMovIncPara = gSpeedLoop.UfMovPara/((INT32U)gCRam.SvPa.KvffBuildTime*(FS/1000));
	}
	if(gSpeedLoop.UfMovIncPara == 0)
		gSpeedLoop.UfMovIncPara = 1;
	
	/* Variable: Move forward control dec step */
	if (gCRam.SvPa.KvffBuildTime == 0)
	{
		gSpeedLoop.UfMovDecPara = gSpeedLoop.UfMovPara/(FS/1000);
	}
	else
	{
		gSpeedLoop.UfMovDecPara = gSpeedLoop.UfMovPara/((INT32U)gCRam.SvPa.KvffReleaseTime*(FS/1000));
	}
	if(gSpeedLoop.UfMovDecPara == 0)
		gSpeedLoop.UfMovDecPara = 1;

	/* Variable: Acc forward control max value limit */
	gSpeedLoop.UfAccPara = (_iq)gCRam.SvPa.KaccCurrent * _IQ((SQRT2*PH3_TO_PH2)/(STD_CURRENT*10));
	/* Variable: Acc forward control inc step */
	if (gCRam.SvPa.KaccBuildTime == 0)
	{
		gSpeedLoop.UfAccIncPara = gSpeedLoop.UfAccPara/(FS/1000);
	}
	else
	{
		gSpeedLoop.UfAccIncPara = gSpeedLoop.UfAccPara/((INT32U)gCRam.SvPa.KaccBuildTime*(FS/1000));
	}
	if(gSpeedLoop.UfAccIncPara == 0)
		gSpeedLoop.UfAccIncPara = 1;
	/* Variable: Acc forward control dec step */
	if (gCRam.SvPa.KaccReleaseTime == 0)
	{
		gSpeedLoop.UfAccDecPara = gSpeedLoop.UfAccPara/(FS/1000);
	}
	else
	{
		gSpeedLoop.UfAccDecPara = gSpeedLoop.UfAccPara/((INT32U)gCRam.SvPa.KaccReleaseTime*(FS/1000));
	}
	if(gSpeedLoop.UfAccDecPara == 0)
		gSpeedLoop.UfAccDecPara = 1;
	/* Variable: Bke forward control max value limit */
	gSpeedLoop.UfBkePara = (_iq)gCRam.SvPa.KbkeCurrent * _IQ((SQRT2*PH3_TO_PH2)/(STD_CURRENT*10));
	/* Variable: Bke forward control inc step */
	if (gCRam.SvPa.KaccBuildTime == 0)
	{
		gSpeedLoop.UfBkeIncPara = gSpeedLoop.UfBkePara/(FS/1000);
	}
	else
	{
		gSpeedLoop.UfBkeIncPara = gSpeedLoop.UfBkePara/((INT32U)gCRam.SvPa.KaccBuildTime*(FS/1000));
	}
	if(gSpeedLoop.UfBkeIncPara == 0)
		gSpeedLoop.UfBkeIncPara = 1;
	/* Variable: Bke forward control dec step */
	if (gCRam.SvPa.KaccReleaseTime == 0)
	{
		gSpeedLoop.UfBkeDecPara = gSpeedLoop.UfBkePara/(FS/1000);
	}
	else
	{
		gSpeedLoop.UfBkeDecPara = gSpeedLoop.UfBkePara/((INT32U)gCRam.SvPa.KaccReleaseTime*(FS/1000));
	}
	if(gSpeedLoop.UfBkeDecPara == 0)
		gSpeedLoop.UfBkeDecPara = 1;

	if (gSpeedLoop.UfBkeDecPara < gSpeedLoop.UfAccDecPara)
		gSpeedLoop.UfBkeDecPara = gSpeedLoop.UfAccDecPara;
	else
		gSpeedLoop.UfAccDecPara = gSpeedLoop.UfBkeDecPara;

	//exp acc dec para
	if (gCRam.SvPa.SpdAccT == 0)
	{
		gSpeedLoop.SpdAccK = _IQ(1.0);
	}
	else
	{
		k = _IQdiv(_IQ16(TS) * 1000, (INT32S)gCRam.SvPa.SpdAccT << 16);
		gSpeedLoop.SpdAccK = k - _IQrmpy(k, k)/2;
	}

	if (gCRam.SvPa.SpdDecT == 0)
	{
		gSpeedLoop.SpdDecK = _IQ(1.0);
	}
	else
	{
		k = _IQdiv(_IQ16(TS) * 1000, (INT32S)gCRam.SvPa.SpdDecT << 16);
		gSpeedLoop.SpdDecK = k - _IQrmpy(k, k)/2;
	}
	pSpeedLoop->SpdAccRatioFast = (_iq)gCRam.SvPa.SpdAccRatioFast * _IQ(1.0/4096);
	pSpeedLoop->SpdDecRatioFast = (_iq)gCRam.SvPa.SpdDecRatioFast * _IQ(1.0/4096);
	pSpeedLoop->SpdAccRatioSlow = (_iq)gCRam.SvPa.SpdAccRatioSlow * _IQ(1.0/4096);
	pSpeedLoop->SpdDecRatioSlow = (_iq)gCRam.SvPa.SpdDecRatioSlow * _IQ(1.0/4096);
	pSpeedLoop->SpdAccRatioAct = _IQ(1.0);
	pSpeedLoop->SpdDecRatioAct = _IQ(1.0);
	
	gSpeedLoop.RefFollowFluxKoe = _IQdiv(_IQ(1.0/STD_CURRENT), gCRam.SvPa.MotorPara.DriveFlux);
	gSpeedLoop.RefFollowDecKoe = _IQdiv(_IQ(1.0/STD_CURRENT), REF_FOLLOW_LIMIT(gCRam.SvPa.MotorPara.DriveCurrent));
}

/*******************************************************************************
* Name: ParaToTune
* Description: 
* Input: 
* Output:
*
* Author:
* Date:
* Revision:
*******************************************************************************/
void ParaToTune(void)
{
//	INT32S i32S;

//	/* Tune����ٶ�, Q��ʽ, ����ֵ */
//	gTune.SpeedMax = (INT32S)gCRam.SvPa.TuneSpdMax * _IQ(1.0/STD_SPEEDRPM);
//	/* �м�1/3��Ϊת�ز������� */
//	gTune.SpeedMeetMin = gTune.SpeedMax / 3;
//	gTune.SpeedMeetMax = gTune.SpeedMeetMin * 2;
//	/* ������ٶ� a = max^2 / R */
//	i32S = (INT32S)gCRam.SvPa.TuneSpdMax * (INT32S)gCRam.SvPa.TuneSpdMax;
//	i32S = i32S / gCRam.SvPa.TuneR + 1;
//	i32S = _IQmpy(i32S, _IQ(2 * PI * 2 * PI / SEC_MIN / SEC_MIN / 2 / PI));
//	/* ���ٶ� a, Q��ʽ, ����ֵ��λ */
//	gTune.SpeedAcc = i32S * _IQ(1.0/STD_SPEEDACC);
}

/*******************************************************************************
* Name: ParaToRigid
* Description: 
* Input: 
* Output:
*
* Author:
* Date:
* Revision:
*******************************************************************************/
void ParaToRigid(void)
{
	if(gCRam.SvPa.RefRigid < 13)
	{
		/* �ٶȻ���ֹƵ�� */
		gCRam.SvPa.SpeedWc = gRigidParaRom[gCRam.SvPa.RefRigid].SpdKp;
		/* �ٶȻ�����ʱ�䳣�� */
		gCRam.SvPa.SpeedWit = gRigidParaRom[gCRam.SvPa.RefRigid].SpdKit;
		/* ת���˲���1���� */
		gCRam.SvPa.TorFilt1W = gRigidParaRom[gCRam.SvPa.RefRigid].TorFilterW1;
		/* ת���˲���2���� */
		gCRam.SvPa.TorFilt2W = gRigidParaRom[gCRam.SvPa.RefRigid].TorFilterW2;

		if(gCRam.SvPa.ModifyCtl.bVolatile == 0)
		{
			/* ֪ͨ��������Բ���(PA55)���������Ĳ��� */
			SL_SET(SL_SAVE_RIGID_RELATED_PARA);
		}
		ParaToSpeedPID();
		ParaToTorqueFilter();
	}	
}

/*******************************************************************************
* Name: UpdatePara
* Description: �����ж��е���
* Input: 
* Output:
*
* Author:
* Date:
* Revision:
*******************************************************************************/
void UpdatePara(void)
{
	if(SL_CHK(SL_PARA_MODIFIED))
	{
		switch(gCRam.SvPa.ModifyCtl.ParaNo)
		{
		case  1: 
		case  5: 
		case  6: ParaToSpeedPID();			break;
		case  7: 
		case  8: ParaToTorqueFilter();		break;
		case  9: 
		case 10:
		case 11: 
		case 19: 
		case 34:
		case 35: ParaToSpeedPID();			break;
		case 40:
		case 41: ParaToPanSpdT();			break;
		case 55: ParaToRigid(); 			break;
		case 56: ParaToSpeedPID(); 			break;
		case 80:
		case 81:
		case 83:
		case 84:
		case 86: ParaToTorqueFilter();		break;
		case 90:
		case 91: ParaToTune();		    	break;
		default:;
		}
		if(gCRam.SvPa.ModifyCtl.bVolatile == 0)
		{
			SL_SET(SL_SAVE_MODIFIED_PARA);
		}
		SL_CLR(SL_PARA_MODIFIED);
	}
}

/*******************************************************************************
* Name: GetMotorParaRom
* Description: 
* Input: 
* Output:
*
* Author:
* Date:
* Revision:
*******************************************************************************/
static void GetMotorParaRom(void)
{
	_iq MaxHZ;

	//set AcMotorPloes
	
	/* calculate max rpm(Q0) */
	MaxHZ = _IQrmpy((INT32S)gCRam.SvPa.AcMotorTypicalSpdF, _IQ(1.2));
	gKernelCtl.MotorMaxHz = MaxHZ;
//		gKernelCtl.MaxOverLoadCurrent = _IQ(ESP_OVER_CUREENT*SQRT2*PH3_TO_PH2/STD_CURRENT);
//		gKernelCtl.OverLoadCurrent2M = _IQ(RATE_2M_CURRENT*SQRT2*PH3_TO_PH2/STD_CURRENT);
		gKernelCtl.MaxOverLoadCurrent = _IQ(ESP_OVER_CUREENT/STD_CURRENT);	//for DC motor
		gKernelCtl.OverLoadCurrent2M = _IQ(RATE_2M_CURRENT*0.95/STD_CURRENT);	//for DC motor
		gKernelCtl.PumpMaxOverLoadCurrent = gKernelCtl.MaxOverLoadCurrent*1.5;	//for DC motor	
		gKernelCtl.PumpOverLoadCurrent2M = _IQ(PROPD_MAX_CURRENT*0.95/PROPD_STD_CURRENT);	//for DC motor
	
	/* clamp to IPM max current */
#ifndef __IQFDB_INVALID
	if (gKernelCtl.MaxOverLoadCurrent > _IQ(0.999*PH3_TO_PH2))
	{
		gKernelCtl.MaxOverLoadCurrent = _IQ(0.999*PH3_TO_PH2);
	}
#endif
	
	/* calculate esp low voltage */
	gKernelCtl.EspLowVoltage = gCRam.SvPa.DcEspLowVoltage * _IQ(1.0/(10.0*STD_VBUS));
	gKernelCtl.EspHighVoltage = (gCRam.SvPa.DcMaxVoltage+20) * _IQ(1.0/(10.0*STD_VBUS));
}

/*******************************************************************************
* Name: CalcServoLoopPara
* Description: 
* Input: 
* Output:
*
* Author:
* Date:
* Revision:
*******************************************************************************/
void CalcServoLoopPara(void)
{
	GetMotorParaRom();
	ParaToCurrentPID();
	ParaToSpeedPID();
	ParaToPositionPID();
	ParaToTorqueFilter();
	ParaToPanSpdT();
	ParaToTune();
}

