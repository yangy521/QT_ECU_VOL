/*******************************************************************************
* Filename: PropDriver.c                                             	 		   *
*                                                                    		   *
* Description: Prop driver control implementation file of KSD.   			 		   *
* Author:                                                                   *
* Date: 100622														           *
* Revision:															 		   *
*																	 		   *
*******************************************************************************/

#include	"PropDriver.h"
#include  "ServoPara.h"
#include	"CommonRam.h"

PROP_CURRENT_LOOP	gPropCurrentLoop;
/*******************************************************************************
* Name: PropCurrentFilterInit
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void PropCurrentFilterInit(PROP_CURRENT_FILTER *p)
{
	p->In = 0;
	p->Out = 0;
	p->Display = 0;
	p->DisplaySum = 0;
}

/*******************************************************************************
* Name: PropCurrentFilterCalc
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void PropCurrentFilterCalc(PROP_CURRENT_FILTER *p)
{
	/* t = 1ms */
	p->Out += (p->In - p->Out + 4) / 8;
	/* t = 8ms */
	p->DisplaySum += p->In - p->Display;
	p->Display = p->DisplaySum >> 10;
}
/*******************************************************************************
* Name: ParaToPropCurrentPID
* Description: 
* Input: 
* Output:
*
* Author:
* Date:
* Revision:
*******************************************************************************/
static void ParaToPropCurrentPID(PROP_CURRENT_PID *p)
{
	_iq kp;
	_iq ki;
	_iq t0;
	_iq resist;
	_iq td;

	/* g_servo_para.motor_para.resist为单相电阻 */
	/* resist为ID，IQ电阻 */
	resist = _IQ(0.1				/STD_RESIST);		/* R */
	t0	= _IQ(0.00160				/STD_T);				/* t0=L/R,stator time constant */
//	td = _IQ(TS/STD_T);
	td = _IQ(TS/STD_T) / 2;
	/* delay ~= ??us, 此值由PWM周期和电流采样滤波电路决定 */

	/* Kp = L / (2 * Td) = t0 * R / (2 * Td) */
	kp = _IQdiv(_IQmpy(_IQmpy(t0, resist), _IQ(PROPD_STD_CURRENT/STD_VOLTAGE/2)), td);
	kp = _IQmpy(kp, _IQ15toIQ(gCRam.SvPa.PropDKp));
	/* Ki = T / t0 */
	ki = _IQdiv(_IQ(TS), t0);
	ki = _IQmpy(ki, _IQ13toIQ(gCRam.SvPa.PropDKi));
	p->Kp = kp>>3;
	p->Ki = ki>>1;
	p->Kc = ki>>1;
	p->KoeRes = gCRam.SvPa.PropValveResistance * _IQ(PROPD_STD_CURRENT/(STD_VOLTAGE*10));
}

/*******************************************************************************
* Name: PropCurrentPidInit
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void PropCurrentPidInit(PROP_CURRENT_PID *p)
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
	p->OutMax = _IQ(1.0) - 1; //
	p->OutMin = 0;
	p->OutPreSat = 0;
	p->SatErr = 0;
	ParaToPropCurrentPID(p);
}

/*******************************************************************************
* Name: PropCurrentPidCalc
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void PropCurrentPidCalc(PROP_CURRENT_PID *v)
{
	_iq UiLimit;
	SL_CLR(SL_TER_PROP_CURRENT_CUT);
//	if (v->Ref > _IQ(1.0/PROPD_STD_CURRENT))
	if (v->Fdb > gPropCurrentLoop.IRefMax)
	{//close loop
		v->Ref = gPropCurrentLoop.IRefMax;	//add Chow
		/* Compute the error */
		v->Err = v->Ref - v->Fdb;

		/* Compute the proportional output，比例项损失的精度不计 */
		v->Up = _IQmpy(v->Kp, v->Err);

		/* Compute the integral output */
		v->Ui += (INT64S)v->Ki * (INT64S)v->Up + (INT64S)v->Kc * (INT64S)v->SatErr;
		/* Limit Ui*/
		UiLimit = (_iq)((v->Ui + (INT64S)(1 << (GLOBAL_Q-1))) >> GLOBAL_Q);
		if (UiLimit > (v->OutMax>>2))
		{
			v->Ui = ((INT64S)(v->OutMax>>2)) << GLOBAL_Q;
		}
		else if (UiLimit < (v->OutMin>>2))
		{
			v->Ui = ((INT64S)(v->OutMin>>2)) << GLOBAL_Q;
		}

		/* Compute the pre-saturated output */
		v->OutPreSat = v->Up + (_iq)((v->Ui + (INT64S)(1 << (GLOBAL_Q-1))) >> GLOBAL_Q);
	}
	else if (v->Fdb < -_IQ(10.0/PROPD_STD_CURRENT)) //回馈电流大于10A
	{
		SL_SET(SL_TER_PROP_CURRENT_CUT);
	}		
	else
	{//open loop 
		//v->OutPreSat = _IQmpy(v->Ref,  v->KoeRes);
		if(v->OutPreSat < v->Ref)
			v->OutPreSat += _IQ(0.001);
		else
			v->OutPreSat = v->Ref;
		v->Ui = ((INT64S)v->OutPreSat) << GLOBAL_Q;
	}
	/* Saturate the output */
	if (v->OutPreSat > v->OutMax)
	{
		v->Out =  v->OutMax;
	}
	else if (v->OutPreSat < v->OutMin)
	{
		v->Out =  v->OutMin;
	}
	else
	{
		v->Out = v->OutPreSat;
	}

	/* Compute the saturate difference */
	v->SatErr = v->Out - v->OutPreSat;
}

/*******************************************************************************
* Name: PropCurrentLoopInit
* Description: Prop driver current loop initialization
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void PropCurrentLoopInit(PROP_CURRENT_LOOP *p)
{
	/* initialize Id PID struct varibale */
	p->IPID.Init = PropCurrentPidInit;
	p->IPID.Calc = PropCurrentPidCalc;
	p->IPID.Init(&p->IPID);

	/* initialize PWMGen struct varibale */
	p->PwmGen.PeriodMax = (INT16U)(PROP_PWM_PERIOD);
	p->PwmGen.MfuncPeriod = 0x8000;
	p->PwmGen.MfuncC1 = 0;
	p->PwmGen.MfuncC2 = 0;
	p->PwmGen.MfuncC3 = 0;
	p->PwmGen.Update = PropDriverPwmUpdate;
	
	/* current fdb filter */
	p->PropCurrentFilter.Init = PropCurrentFilterInit;
	p->PropCurrentFilter.Calc = PropCurrentFilterCalc;
	p->PropCurrentFilter.Init(&p->PropCurrentFilter);
	/* initialize other variables */
	p->State = 0;
	p->IRef = 0;
	p->IFdb = 0;


	p->VfRef = 0;
	p->VbusFdb = 0;
//	p->IRefMax = _IQ(PROPD_MAX_CURRENT/PROPD_STD_CURRENT);
	p->IRefMax = (_iq)gCRam.SvPa.PumpDriveCurrentLimitRatio * _IQ(PROPD_MAX_CURRENT / (PROPD_STD_CURRENT*STD_PERCENT));
}

/*******************************************************************************
* Name: PropCurrentLoopCalc
* Description: Prop driver current loop calculation
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
//#define DEBUG_PROP_CURRENT_CLOSELOOP
//#define DEBUG_PROP_CURRENT_OPENLOOP
#define IQ15_DEADBAND 0x100
void PropCurrentLoopCalc(PROP_CURRENT_LOOP *p)
{

	/*  PID */
	/* set I PID input */
	p->IPID.Ref = p->IRef;
  
#ifdef DEBUG_PROP_CURRENT_CLOSELOOP
	p->IPID.Ref = _IQ(1.0/PROPD_STD_CURRENT);
#endif

	p->IPID.Fdb = p->IFdb;
	/* execute PID */
	p->IPID.Calc(&p->IPID);
	//p->IPID.Out 母线电压补偿
#ifdef DEBUG_PROP_CURRENT_OPENLOOP
	p->IPID.Out =  _IQ(2.4/STD_VOLTAGE);
#endif //#ifdef DEBUG_CURRENT_OPENLOOP


	/* 5. PWMGEN */
	/* set PWMGEN input */
	p->PwmGen.MfuncC1 = 0;
	p->PwmGen.MfuncC2 = (INT16S)(_IQtoIQ15(p->IPID.Out));
	p->PwmGen.MfuncC3 = 0;
	//Dead band cmp
	if (p->PwmGen.MfuncC2 != 0)
	{
		p->PwmGen.MfuncC2 += IQ15_DEADBAND;
		if ((p->PwmGen.MfuncC2 & _IQ15(1.0)) != 0)
		{
			p->PwmGen.MfuncC2 = _IQ15(1.0) - 1;
		}
	}
	else
	{
		p->PwmGen.MfuncC2 = IQ15_DEADBAND;
	}
	/* execute PWMGEN */
	p->PwmGen.Update(&p->PwmGen);
}

/*******************************************************************************
* Name: CurrentLoopReset
* Description: current loop reset
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void PropCurrentLoopReset(PROP_CURRENT_LOOP *p)
{
	/* reset Id PID struct varibale */
	p->IPID.Out = 0;
	p->IPID.Ref = 0;
	p->IPID.Fdb = 0;
	p->IPID.Err = 0;
	p->IPID.Up = 0;
	p->IPID.Ui = 0;
	p->IPID.OutPreSat = 0;
	p->IPID.SatErr = 0;

	/* reset other variables */
	p->State = 0;
	p->IRef = 0;
	p->IFdb = 0;
	p->VbusFdb = 0;

	/* current fdb filter*/
	p->PropCurrentFilter.Init(&p->PropCurrentFilter);
		/* execute PWMGEN */
	p->PwmGen.MfuncC1 = 0;
	p->PwmGen.MfuncC2 = 0;
	p->PwmGen.MfuncC3 = 0;
	p->PwmGen.Update(&p->PwmGen);

}

