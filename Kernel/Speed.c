/*******************************************************************************
* Filename: Speed.c                                             	 		   *
*                                                                    		   *
* Description:											   			 		   *
* Author: kff                                                                  *
* Date: 100623														           *
* Revision:															 		   *
*																	 		   *
*******************************************************************************/

#include  "Speed.h"
#include	"CommonRam.h"
#include	"Fdb.h"
#include	"ServoPara.h"

SPEED_LOOP		gSpeedLoop;
/*******************************************************************************
* Name: SpeedPidInit
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void SpeedPidInit(SPEED_PID *p)
{
	p->Out = 0;
	p->Ref = 0;
	p->Fdb = 0;
	p->Err = 0;
	p->Kp = 0;
	p->Ki = 0;
	p->Kc = 0;
	p->Up = 0;
	p->Ui = 0;
	p->UfMov = 0;
	p->UfAcc = 0;
	
//	p->KcoeTorque2Iq = _IQ(1.0);
//	p->KcoeIq2Torque = _IQ(1.0);
	p->OutMax = _IQ(+0.999);
	p->OutMin = _IQ(-0.999);
	p->OutPreSat = 0;
	p->SatErr = 0;
	p->State = 0;
	p->UpSucc = 0;
}

/*******************************************************************************
* Name: SpeedPidCalc
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void SpeedPidCalc(SPEED_PID *v)
{
	_iq TorquePi;
	_iq OutMax, OutMin;
	INT64S 	UpMult;		/* Variable: 64 bit Proportional mult */
	INT64S 	CurrentPi;		/* Variable: 64 bit Proportional mult */
	/* Compute the error */
	v->Err = v->Ref - v->Fdb;

	/* Compute the proportional output，比例项损失的精度不计 */
	//v->Up = _IQmpy(v->Kp, v->Err);
	UpMult = (INT64S)v->Kp * (INT64S)v->Err + (INT64S)v->UpSucc;
	v->Up = UpMult >> GLOBAL_Q;
	if ((UpMult & (INT64S)(1 << (GLOBAL_Q-1))) != 0)
	{
		v->Up++;
	}
	v->UpSucc = UpMult;
	v->UpSucc = (v->UpSucc << (32 - GLOBAL_Q)) >> (32 - GLOBAL_Q);
	/* Compute the integral output */
	v->Ui += (INT64S)v->Ki * (INT64S)v->Up + (INT64S)v->Kc * (INT64S)v->SatErr;
	TorquePi = v->Up + (_iq)(v->Ui >> GLOBAL_Q);
	CurrentPi = (INT64S)v->KcoeTorque2Iq * (INT64S)TorquePi;

	/* Compute the pre-saturated output */
	v->OutPreSat = (_iq)(CurrentPi >> GLOBAL_Q) + v->UfMov + v->UfAcc;

	/* Saturate the output */
	OutMax = v->OutMax;
	OutMin = v->OutMin;
	if (v->OutPreSat > OutMax)
	{
		SL_SET(SL_ASR_SATURATED);
		v->Out =  OutMax;
	}
	else if (v->OutPreSat < OutMin)
	{
		SL_SET(SL_ASR_SATURATED);
		v->Out = OutMin;
	}
	else
	{
		SL_CLR(SL_ASR_SATURATED);
		v->Out = v->OutPreSat;
	}

	/* Compute the saturate difference */
	//v->SatErr = _IQmpy(v->Out - v->OutPreSat,v->KcoeIq2Torque);
}

/*******************************************************************************
* Name: Lpf1Design
* Description: 1 order Butterworth
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void Lpf1Design(BUTTER *f)
{
	_iq a0, b1;
	_iq vsin, vcos;
	_iq w;					/* 归一化角频率(0, 0.5] = (0, pi] */

	/* 输入参数检查，w在(0, 0.5) */
	w = f->w;
	if ((w <= _IQ(0.0)) || (w >= _IQ(0.5)))
	{
		f->w = _IQ(0.5);
	}
	else
	{
		vsin = _IQsinPU(w / 2);
		vcos = _IQcosPU(w / 2);
		a0 = _IQdiv(vsin       , vsin + vcos);
		b1 = _IQdiv(vsin - vcos, vsin + vcos);
		f->a0 = f->a1 = a0;
		f->b1 = b1;
	}
}

/*******************************************************************************
* Name: Lpf1Calc
* Description: 1 order Butterworth
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void Lpf1Calc(BUTTER *f)
{
	/* if exceed nyquist frequency */
	if (f->w >= _IQ(0.5))
	{
		/* do not filter */
		f->y1 = f->y0 = f->x1 = f->x0;
	}
	else
	{
		/* filter calculate */
		f->y0 = _IQrmpy(f->a0, f->x0) + _IQrmpy(f->a1, f->x1);
		f->y0 -= _IQrmpy(f->b1, f->y1);
		f->x1 = f->x0;
		f->y1 = f->y0;
	}
}

/*******************************************************************************
* Name: Lpf2Design
* Description: 2 order Butterworth
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void Lpf2Design(BUTTER *f)
{
	_iq a0, a1, b1, b2;
	_iq vsin, vcos;
	_iq w;
	_iq q;
	_iq v2sin, v2cos, vq;

	/* 输入参数检查，w在(0, 0.5) */
	w = f->w;
	q = f->Q;
	if ((w <= _IQ(0.0)) || (w >= _IQ(0.5)))
	{
		f->w = _IQ(0.5);
	}
	else
	{
		vsin = _IQsinPU(w / 2);
		vcos = _IQcosPU(w / 2);
		v2sin = _IQrmpy(vsin, vsin);
		v2cos = _IQrmpy(vcos, vcos);
		vq = _IQdiv(_IQrmpy(vsin, vcos), q);

		a1 = _IQdiv(2 * v2sin          , _IQ(1.0) + vq);
		b1 = _IQdiv(2 * (v2sin - v2cos), _IQ(1.0) + vq);
		b2 = _IQdiv(_IQ(1.0) - vq      , _IQ(1.0) + vq);
		a0 = a1 / 2;

		f->a0 = f->a2 = a0;
		f->a1 = a1;
		f->b1 = b1;
		f->b2 = b2;
	}
}

/*******************************************************************************
* Name: Lpf2Calc
* Description: 2 order Butterworth
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void Lpf2Calc(BUTTER *f)
{
	/* if exceed nyquist frequency */
	if (f->w >= _IQ(0.5))
	{
		/* do not filter */
		f->y2 = f->y1 = f->y0 = f->x2 = f->x1 = f->x0;
	}
	else
	{
		/* filter calculate */
		f->y0 = _IQrmpy(f->a0, f->x0) + _IQrmpy(f->a1, f->x1)
			+ _IQrmpy(f->a2, f->x2);
		f->y0 -= _IQrmpy(f->b1, f->y1) + _IQrmpy(f->b2, f->y2);
		f->x2 = f->x1;
		f->x1 = f->x0;
		f->y2 = f->y1;
		f->y1 = f->y0;
	}
}

/*******************************************************************************
* Name: NotchDesign
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void NotchDesign(BUTTER *f)
{
	_iq a0, a1, b1, b2;
	_iq vsin, vcos;
	_iq w;
	_iq q;                 
	_iq v2sin, v2cos, vsincos;

	/* 输入参数检查，w在(0, 0.5) */
	w = f->w;
	q = f->Q;
	if ((w <= _IQ(0.0)) || (w >= _IQ(0.5)))
	{
		f->w = _IQ(0.5);
	}
	else
	{
		vsin = _IQsinPU(w / 2);
		vcos = _IQcosPU(w / 2);
		v2sin = _IQrmpy(vsin, vsin);
		v2cos = _IQrmpy(vcos, vcos);
		vsincos = _IQrmpy(vsin, vcos);

		a0 = _IQdiv(q                            , q + vsincos);
		a1 = _IQdiv(2 * _IQrmpy(q, v2sin - v2cos), q + vsincos);
		b1 = a1;
		b2 = _IQdiv(q - vsincos                  , q + vsincos);

		f->a0 = f->a2 = a0;
		f->a1 = a1;
		f->b1 = b1;
		f->b2 = b2;
	}
}

/*******************************************************************************
* Name: NotchCalc
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void NotchCalc(BUTTER *f)
{
	/* if exceed nyquist frequency */
	if (f->w >= _IQ(0.5))
	{
		/* do not filter */
		f->y2 = f->y1 = f->y0 = f->x2 = f->x1 = f->x0;
	}
	else
	{
		/* filter calculate */
		f->y0 = _IQrmpy(f->a0, f->x0) + _IQrmpy(f->a1, f->x1) 
			+ _IQrmpy(f->a2, f->x2);
		f->y0 -= _IQrmpy(f->b1, f->y1) + _IQrmpy(f->b2, f->y2);
		f->x2 = f->x1;
		f->x1 = f->x0;
		f->y2 = f->y1;
		f->y1 = f->y0;
	}
}

/*******************************************************************************
* Name: TorqueFilterInit
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void TorqueFilterInit(TORQUE_FILTER *p)
{
	p->In = 0;
	p->Out = 0;

	/* LPF1 */
	p->LPF1.Design = Lpf1Design;
	p->LPF1.Calc = Lpf1Calc;
	p->LPF1.a0 = p->LPF1.a1 = p->LPF1.a2 = 0;
	p->LPF1.b1 = p->LPF1.b2 = 0;
	p->LPF1.x0 = p->LPF1.x1 = p->LPF1.x2 = 0;
	p->LPF1.y0 = p->LPF1.y1 = p->LPF1.y2 = 0;

	/* LPF2 */
	p->LPF2.Design = Lpf2Design;
	p->LPF2.Calc = Lpf2Calc;
	p->LPF2.a0 = p->LPF2.a1 = p->LPF2.a2 = 0;
	p->LPF2.b1 = p->LPF2.b2 = 0;
	p->LPF2.x0 = p->LPF2.x1 = p->LPF2.x2 = 0;
	p->LPF2.y0 = p->LPF2.y1 = p->LPF2.y2 = 0;

	/* Notch1 */
	p->Notch1.Design = NotchDesign;
	p->Notch1.Calc = NotchCalc;
	p->Notch1.a0 = p->Notch1.a1 = p->Notch1.a2 = 0;
	p->Notch1.b1 = p->Notch1.b2 = 0;
	p->Notch1.x0 = p->Notch1.x1 = p->Notch1.x2 = 0;
	p->Notch1.y0 = p->Notch1.y1 = p->Notch1.y2 = 0;

	/* Notch2 */
	p->Notch2.Design = NotchDesign;
	p->Notch2.Calc = NotchCalc;
	p->Notch2.a0 = p->Notch2.a1 = p->Notch2.a2 = 0;
	p->Notch2.b1 = p->Notch2.b2 = 0;
	p->Notch2.x0 = p->Notch2.x1 = p->Notch2.x2 = 0;
	p->Notch2.y0 = p->Notch2.y1 = p->Notch2.y2 = 0;
}

/*******************************************************************************
* Name: TorqueFilterCalc
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void TorqueFilterCalc(TORQUE_FILTER *p)
{
	/* Notch1 */
//	p->Notch1.x0 = p->In;
//	p->Notch1.Calc(&p->Notch1);

	/* Notch2 */
//	p->Notch2.x0 = p->Notch1.y0;
//	p->Notch2.Calc(&p->Notch2);

	/* LPF2 */
//	p->LPF2.x0 = p->Notch2.y0;
	p->LPF2.x0 = p->In;
	p->LPF2.Calc(&p->LPF2);

	/* LPF1 */
	p->LPF1.x0 = p->LPF2.y0;
	p->LPF1.Calc(&p->LPF1);

	/* filter output */
	p->Out = p->LPF1.y0;
}

/*******************************************************************************
* Name: SpeedFilterInit
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void SpeedFilterInit(SPEED_FILTER *p)
{
	p->In = 0;
	p->Out = 0;
	p->OutSum = 0;
	p->Out16ms = 0;
	p->Out16msSum = 0;
	p->Out64ms = 0;
	p->Out64msSum = 0;
	p->Display = 0;
	p->DisplaySum = 0;
}

/*******************************************************************************
* Name: SpeedFilterCalc
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void SpeedFilterCalc(SPEED_FILTER *p)
{
	/* t = 4ms */
	p->OutSum += p->In - p->Out;
	p->Out = p->OutSum >> 5;
	/* t = 16ms */
	p->Out16msSum += p->In - p->Out16ms;
	p->Out16ms = p->Out16msSum >> 7;
	/* t = 64ms */
	p->Out64msSum += p->In - p->Out64ms;
	p->Out64ms = p->Out64msSum >> 9;
	/* t = 256ms */
	p->DisplaySum += p->In - p->Display;
	p->Display = p->DisplaySum >> 11;
}

/*******************************************************************************
* Name: SpeedObserverInit
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void SpeedObserverInit(SPEED_OBSERVER *p)
{
	/* Initialize variables */
	p->SpdRaw = 0;
	p->IqFdb = 0;
	p->SpdObs = 0;
	p->TaSum = 0;

	/* Initialize PID */
	p->PID.Out = 0;
	p->PID.Ref = 0;
	p->PID.Fdb = 0;
	p->PID.Err = 0;
	p->PID.Kp = 0;
	p->PID.Ki = 0;
	p->PID.Kc = 0;
	p->PID.Up = 0;
	p->PID.Ui = 0;
	p->PID.OutMax = _IQ(+0.999);
	p->PID.OutMin = _IQ(-0.999);
	p->PID.OutPreSat = 0;
	p->PID.SatErr = 0;
}

/*******************************************************************************
* Name: SpeedObserverCalc
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void SpeedObserverCalc(SPEED_OBSERVER *p)
{
	_iq ta;			/* Ta = Te - Tload */
	_iq inertia;
	_iq kinertia;
	_iq ktorque;
	_iq coff;

	p->PID.Ref = p->SpdRaw;
	p->PID.Fdb = p->SpdObs;
	p->PID.Calc(&p->PID);

	ta = p->IqFdb + p->PID.Out;
	/* ta = 加速度转矩 */
	/* a = Ta * Coeff / J */
	inertia = gCRam.SvPa.MotorPara.Inertia;
	ktorque = gCRam.SvPa.MotorPara.KTorque;

	kinertia = (INT32S)gCRam.SvPa.IntertiaRatio * _IQ(1.0/100.0);
	inertia = _IQmpy(inertia, kinertia);

	coff = _IQrmpy(_IQdiv(ktorque, inertia), 
		_IQ(STD_CURRENT * STD_TORQUE_K / STD_SPEED  / STD_INERTIA * TS));
	/* 1/s */
	p->TaSum += ta;
	p->SpdObs = (INT64S)(p->TaSum * coff >> 24);
}
/*******************************************************************************
* Name: SpeedPidPreset
* Description: 
*             Set speed pid max,min output limit upon the speed state and weakflux
*             Set forward control value Vff, Accff value.
* Input: state,spdfdb,slip,
* Output: fluxRatio, torque current ratio
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void SpeedPidPreset(SPEED_LOOP *p)
{
}

/*******************************************************************************
* Name: SpeedLoopInit
* Description: speed loop initialization
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void SpeedLoopInit(SPEED_LOOP *p)
{
	/* Initialize speed feedback Filter */
	p->Filter.Init = SpeedFilterInit;
	p->Filter.Calc = SpeedFilterCalc;
	p->Filter.Init(&p->Filter);

	/* Initialize PID */
	p->PID.Init = SpeedPidInit;
	p->PID.Calc = SpeedPidCalc;
	p->PID.Init(&p->PID);

	/* Initialize Observer */
	p->Observer.Init = SpeedObserverInit;
	p->Observer.Calc = SpeedObserverCalc;
	p->Observer.Init(&p->Observer);

	/* Initialize TorqueFilter */
	p->TorqueFilter.Init = TorqueFilterInit;
	p->TorqueFilter.Calc = TorqueFilterCalc;
	p->TorqueFilter.Init(&p->TorqueFilter);
	/* Initialize TorqueFilter */
//	p->FluxFilter.Init = TorqueFilterInit;
//	p->FluxFilter.Calc = TorqueFilterCalc;
//	p->FluxFilter.Init(&p->FluxFilter);

//	p->WeakFlux.Init = WeakFluxInit;
//	p->WeakFlux.Init(&p->WeakFlux);
	/* Initialize variables */

	p->State = SPD_RESET;
	p->SlipFreq = 0;
	p->FluxWeRef = 0;
	p->BaseFlux = gCRam.SvPa.MotorPara.DriveFlux;
	p->BaseIq = gCRam.SvPa.MotorPara.DriveCurrent;
	p->EncoderDirCount = 0;

	//p->FluxRef1 = Need not init;
//	p->KcoeTorque2Iq = _IQ(1.0);
//	p->KcoeIq2Torque = _IQ(1.0);
	p->PidOutMax = p->BaseIq;
	p->PidOutMin = -p->PidOutMax;
	p->UfMov = 0;
	p->UfMovDst = 0;
	p->UfMovInc = 0;
	p->UfMovDec = 0;
	p->UfAcc = 0;
	p->UfAccDst = 0;
	p->UfAccInc = 0;
	p->UfAccDec = 0;

	p->UfMovPara = 0;
	p->UfMovIncPara = 0;
	p->UfMovDecPara = 0;
	p->UfAccPara = 0;
	p->UfAccIncPara = 0;
	p->UfAccDecPara = 0;
	p->UfBkePara = 0;
	p->UfBkeIncPara = 0;
	p->UfBkeDecPara = 0;

	p->FlxCmd = p->BaseFlux;
	p->CurrentCmd = 0;
	p->SpdFdb = 0;
	p->SpdCmd = 0;
	p->SpdRef = 0;
	p->SpdRefL = 0;

	p->IqFdb = 0;
	p->VoltageBus = 0;
	p->VoltageBusLimit = 	(gCRam.SvPa.DcBrkLimitVoltage * _IQ(1.0/(10.0 * STD_VBUS)));
	p->BrakeCmd = 0;
	p->SpdAtZeroSpdcmdUp = 0;
	p->SpdDecLKOld = p->EndDecRateStop;
	p->SpdAccLKOld = p->EndAccRateStop;
}

/*******************************************************************************
* Name: SpeedLoopCalc
* Description: speed loop calculation
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
#ifndef _RELEASE
//#define DEBUG_SPEED
//#define DEBUG_LOG
#endif
#define SPEED_OPEN_LOOP	//速度开环控制
void SpeedLoopCalc(SPEED_LOOP *p)
{
#ifdef SPEED_OPEN_LOOP
	p->PID.Out = p->SpdRef;
#else
	p->PID.Fdb=p->SpdFdb;	
	p->PID.Ref=p->SpdRef; 
	
	p->PID.Calc(&p->PID);	
#endif //SPEED_OPEN_LOOP
	
	p->CurrentCmd=p->PID.Out;
}


/*******************************************************************************
* Name: SpeedLoopReset
* Description: speed loop reset
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void SpeedLoopReset(SPEED_LOOP *p)
{
}

