/*******************************************************************************
* Filename: PropDriver.c                                             	 		   *
*                                                                    		   *
* Description: Prop driver control implementation file of KSD.   			 		   *
* Author:                                                                   *
* Date: 100622														           *
* Revision:															 		   *
*																	 		   *
*******************************************************************************/
#include "PropDriver.h"
//#include "CommonRam.h"
#include "Log.h"
#include "Para.h"

static xPropCurrentLoop	sgPropCurrentLoop[PropDriverMax];

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
static void vPropCurrentFilterInit(xPropCurrentFilter *p)
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
static void vPropCurrentFilterCalc(xPropCurrentFilter *p)
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
static void vParaToPropCurrentPID(xPropCurrentPid *p)
{
	_iq kp;
	_iq ki;
	_iq t0;
	_iq resist;
	_iq td;

	/* g_servo_para.motor_para.resist????? */
	/* resist?ID,IQ?? */
//	resist = _IQ(20				/STD_RESIST);		/* R */
	switch(((xPropCurrentLoop*)p)->PropDriverNo)
	{
		case PropDriverCh0:
			resist = _IQ(i32GetPara(PARA_PropValveResistance0) / STD_RESIST );
			break;
		case PropDriverCh1:
			resist = _IQ(i32GetPara(PARA_PropValveResistance1) / STD_RESIST );
//			resist = _IQ(_IQ15toIQ(i32GetPara(PARA_PropValveResistance1)) / (STD_RESIST * 10));
			break;
//		case PropDriverCh2:
//			kp = _IQmpy(kp, _IQ15toIQ(i32GetPara(PARA_PropDKp2)));
//			break;
//		case PropDriverCh3:
//			kp = _IQmpy(kp, _IQ15toIQ(i32GetPara(PARA_PropDKp3)));
//			break;
		default:
			break;
	}
	t0	= _IQ(0.00160				/STD_T);				/* t0=L/R,stator time constant */
//	td = _IQ(TS/STD_T);
	td = _IQ(TS/STD_T) / 2;
	/* delay ~= ??us, ???PWM????????????? */

	/* Kp = L / (2 * Td) = t0 * R / (2 * Td) */
	kp = _IQdiv(_IQmpy(_IQmpy(t0, resist), _IQ(PROPD_STD_CURRENT/STD_VOLTAGE/2)), td);
	//kp = _IQmpy(kp, _IQ15toIQ(gCRam.SvPa.PropDKp));
	//kp = _IQmpy(kp, _IQ15toIQ(i32GetPara(PARA_PropDKp)));
	switch(((xPropCurrentLoop*)p)->PropDriverNo)
	{
		case PropDriverCh0:
			kp = _IQmpy(kp, _IQ15toIQ(i32GetPara(PARA_PropDKp0)));
			break;
		case PropDriverCh1:
			kp = _IQmpy(kp, _IQ15toIQ(i32GetPara(PARA_PropDKp1)));
			break;
//		case PropDriverCh2:
//			kp = _IQmpy(kp, _IQ15toIQ(i32GetPara(PARA_PropDKp2)));
//			break;
//		case PropDriverCh3:
//			kp = _IQmpy(kp, _IQ15toIQ(i32GetPara(PARA_PropDKp3)));
//			break;
		default:
			break;
	}
	/* Ki = T / t0 */
	ki = _IQdiv(_IQ(TS), t0);
	//ki = _IQmpy(ki, _IQ13toIQ(gCRam.SvPa.PropDKi));
	//ki = _IQmpy(ki, _IQ13toIQ(i32GetPara(PARA_PropDKi)));
	switch(((xPropCurrentLoop*)p)->PropDriverNo)
	{
		case PropDriverCh0:
			ki = _IQmpy(ki, _IQ13toIQ(i32GetPara(PARA_PropDKi0)));
			break;
		case PropDriverCh1:
			ki = _IQmpy(ki, _IQ13toIQ(i32GetPara(PARA_PropDKi1)));
			break;
//		case PropDriverCh2:
//			ki = _IQmpy(ki, _IQ13toIQ(i32GetPara(PARA_PropDKi2)));
//			break;
//		case PropDriverCh3:
//			ki = _IQmpy(ki, _IQ13toIQ(i32GetPara(PARA_PropDKi3)));
			break;
		default:
			break;
	}
	p->Kp = kp>>3;
	p->Ki = ki>>1;
	p->Kc = ki>>1;
	//p->KoeRes = gCRam.SvPa.PropValveResistance * _IQ(PROPD_STD_CURRENT/(STD_VOLTAGE*10));
	//p->KoeRes = i32GetPara(PARA_PropValveResistance) * _IQ(PROPD_STD_CURRENT/(STD_VOLTAGE*10));
	switch(((xPropCurrentLoop*)p)->PropDriverNo)
	{
		case PropDriverCh0:
			p->KoeRes = i32GetPara(PARA_PropValveResistance0) * _IQ(PROPD_STD_CURRENT/(STD_VOLTAGE * 10));
			break;
		case PropDriverCh1:
			p->KoeRes = i32GetPara(PARA_PropValveResistance1) * _IQ(PROPD_STD_CURRENT/(STD_VOLTAGE * 10));
			break;
//		case PropDriverCh2:
//			p->KoeRes = i32GetPara(PARA_PropValveResistance2) * _IQ(PROPD_STD_CURRENT/(STD_VOLTAGE*10));
//			break;
//		case PropDriverCh3:
//			p->KoeRes = i32GetPara(PARA_PropValveResistance3) * _IQ(PROPD_STD_CURRENT/(STD_VOLTAGE*10));
//			break;
		default:
			break;
	}	
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
static void vPropCurrentPidInit(xPropCurrentPid *p)
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
	vParaToPropCurrentPID(p);
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
static void vPropCurrentPidCalc(xPropCurrentPid *v)
{
	_iq UiLimit;
//	SL_CLR(SL_TER_PROP_CURRENT_CUT);
//	if (v->Ref > _IQ(1.0/PROPD_STD_CURRENT))
	if (1)	
	{//close loop
//		v->Ref = ((xPropCurrentLoop*)v)->IRefMax;	//add Chow
		/* Compute the error */
		v->Err = v->Ref - v->Fdb;

		/* Compute the proportional output,?????????? */
		v->Up = _IQmpy(v->Kp, v->Err);

		/* Compute the integral output */
		v->Ui += (INT64S)v->Ki * (INT64S)v->Up + (INT64S)v->Kc * (INT64S)v->SatErr;
		/* Limit Ui*/
		UiLimit = (_iq)((v->Ui + (INT64S)(1 << (GLOBAL_Q-1))) >> GLOBAL_Q);
		if (UiLimit > (v->OutMax))
		{
			v->Ui = ((INT64S)(v->OutMax)) << GLOBAL_Q;
		}
		else if (UiLimit < (v->OutMin))
		{
			v->Ui = ((INT64S)(v->OutMin)) << GLOBAL_Q;
		}

		/* Compute the pre-saturated output */
		v->OutPreSat = v->Up + (_iq)((v->Ui + (INT64S)(1 << (GLOBAL_Q-1))) >> GLOBAL_Q);
	}
	else if (v->Fdb < -_IQ(10.0/PROPD_STD_CURRENT)) //??????10A
	{
//		SL_SET(SL_TER_PROP_CURRENT_CUT);
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
		v->Out = v->OutMax;
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
static void vPropCurrentLoopAInit(xPropCurrentLoop *p)
{
	/* initialize Id PID struct varibale */
	p->IPID.Init = vPropCurrentPidInit;
	p->IPID.Calc = vPropCurrentPidCalc;
	p->IPID.Init(&p->IPID);

	/* initialize PWMGen struct varibale */
	p->PwmGen.PeriodMax = (INT16U)(PROP_PWM_PERIOD);
	p->PwmGen.MfuncPeriod = 0x8000;
	p->PwmGen.MfuncC = 0;
	p->PwmGen.Update = PropDriverPwmUpdate;
	switch(p->PropDriverNo)
	{
		case PropDriverCh0:
			p->fCurveMinPara = i32GetPara(PARA_PumpDriveCurrentLimitRatio0) / 10.0;
			p->fCurveMaxPara = i32GetPara(PARA_PumpSpdAccRatio0) / 10.0;
			p->u16DitherPeriod = i32GetPara(PARA_PropDDitherPeriod0);
			p->u16DitherStep =  _IQ(i32GetPara(PARA_PropDMaxCurrent0) / 1000.0 / PROPD_STD_CURRENT) * i32GetPara(PARA_PropDDitherRatio0) / 100 / (p->u16DitherPeriod >> 1);
		p->AccRatio = _IQ((i32GetPara(PARA_PropDMaxCurrent0) - i32GetPara(PARA_PropDMinCurrent0)) / 1000.0 / PROPD_STD_CURRENT) / i32GetPara(PARA_PropDAccPeriod0);
			break;
		case PropDriverCh1:
			p->fCurveMinPara = i32GetPara(PARA_PumpDriveCurrentLimitRatio1) / 10.0;
			p->fCurveMaxPara = i32GetPara(PARA_PumpSpdAccRatio1) / 10.0;
			p->u16DitherPeriod = i32GetPara(PARA_PropDDitherPeriod1);
			p->u16DitherStep = _IQ(i32GetPara(PARA_PropDMaxCurrent1) / 1000.0 / PROPD_STD_CURRENT) * i32GetPara(PARA_PropDDitherRatio1) / 100 / (p->u16DitherPeriod >> 1);
//			p->AccRatio = _IQ16(i32GetPara(PARA_PropDMaxCurrent1)) / 1000 / i32GetPara(PARA_PropDAccPeriod1);
			p->AccRatio = _IQ((i32GetPara(PARA_PropDMaxCurrent1) - i32GetPara(PARA_PropDMinCurrent1)) / 1000.0 / PROPD_STD_CURRENT) / i32GetPara(PARA_PropDAccPeriod1);		
			break;
		default:
			break;
	}
	
	/* current fdb filter */
	p->PropCurrentFilter.Init = vPropCurrentFilterInit;
	p->PropCurrentFilter.Calc = vPropCurrentFilterCalc;
	p->PropCurrentFilter.Init(&p->PropCurrentFilter);
	/* initialize other variables */
	p->State = 0;
	p->IRef = 0;
	p->IFdb = 0;


	p->VfRef = 0;
	p->VbusFdb = 0;

	switch(p->PropDriverNo)
	{
		case PropDriverCh0:
			p->IRefMin = _IQ(i32GetPara(PARA_PropDMinCurrent0) / PROPD_STD_CURRENT / 1000);
			p->IRefMax = _IQ(i32GetPara(PARA_PropDMaxCurrent0) / PROPD_STD_CURRENT / 1000);
			break;
		case PropDriverCh1:
			p->IRefMin = _IQ(i32GetPara(PARA_PropDMinCurrent1) / PROPD_STD_CURRENT / 1000);
			p->IRefMax = _IQ(i32GetPara(PARA_PropDMaxCurrent1) / PROPD_STD_CURRENT / 1000);
			break;
//		case PropDriverCh2:
//			p->IRefMax = (_iq)i32GetPara(PARA_PumpDriveCurrentLimitRatio2) * _IQ(PROPD_MAX_CURRENT / (PROPD_STD_CURRENT*STD_PERCENT));
//			break;
//		case PropDriverCh3:
//			p->IRefMax = (_iq)i32GetPara(PARA_PumpDriveCurrentLimitRatio3) * _IQ(PROPD_MAX_CURRENT / (PROPD_STD_CURRENT*STD_PERCENT));
//			break;
		default:
			break;
	}
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
static void vPropCurrentLoopCalc(xPropCurrentLoop *p)
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
	//p->IPID.Out ??????
#ifdef DEBUG_PROP_CURRENT_OPENLOOP
	p->IPID.Out =  _IQ(2.4/STD_VOLTAGE);
#endif //#ifdef DEBUG_CURRENT_OPENLOOP


	/* 5. PWMGEN */
	/* set PWMGEN input */
	p->PwmGen.MfuncC = (INT16S)(_IQtoIQ15(p->IPID.Out));
	//p->PwmGen.MfuncC = _IQ15(0.5);
	//Dead band cmp
	if (p->PwmGen.MfuncC != 0)
	{
		p->PwmGen.MfuncC += IQ15_DEADBAND;
		if ((p->PwmGen.MfuncC & _IQ15(1)) != 0)
		{
			p->PwmGen.MfuncC = _IQ15(1) - 1;
		}
	}
	else
	{
		p->PwmGen.MfuncC = IQ15_DEADBAND;
	}
	/* execute PWMGEN */
	p->PwmGen.Update(&p->PwmGen, p->PropDriverNo);
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
static void vPropCurrentLoopReset(xPropCurrentLoop *p)
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
	p->CurrentCmd = 0;
	p->CurrentCmdRaw = 0;
	
	p->u16DitherCnt = 0;
	p->i32DitherValue = 0;
//	p->Cnt = 0;

	/* current fdb filter*/
	p->PropCurrentFilter.Init(&p->PropCurrentFilter);
		/* execute PWMGEN */
	p->PwmGen.MfuncC = 0;
	p->PwmGen.Update(&p->PwmGen, p->PropDriverNo);
}

/*******************************************************************************
* Name: void vPropCurrentLoopInit(void)
* Descriptio: PropCurrent Initial
* Input: NULL
* Output: NULL 
*******************************************************************************/
void vPropCurrentLoopInit(void)
{
	uint8_t i = 0;
	for (i=0; i<PropDriverMax; i++)
	{
		sgPropCurrentLoop[i].PropDriverNo = i;
		sgPropCurrentLoop[i].Init = vPropCurrentLoopAInit;
		sgPropCurrentLoop[i].Calc = vPropCurrentLoopCalc;
		sgPropCurrentLoop[i].Reset = vPropCurrentLoopReset;
		sgPropCurrentLoop[i].Init(&sgPropCurrentLoop[i]);
	}	
}

/*******************************************************************************
* Name: xPropCurrentLoop *pPropCurrentLoopGet(ePropDriverNo PropDriverNo)
* Descriptio: Get PropCurrent Point
* Input: PropDriverNo rangin[PropDriverCh0 ~ PropDriverMax)
* Output: PropDriver's Point 
*******************************************************************************/
xPropCurrentLoop *pPropCurrentLoopGet(ePropDriverNo PropDriverNo)
{
	if(PropDriverNo >= PropDriverMax)
	{
		i32LogWrite(ERR, LOG_PROP, "PropDriver Parameters is Wrong, PropDriveMax = %d, PropDriverNo\r\n", PropDriverMax, PropDriverNo);
		return NULL;
	}
	return &sgPropCurrentLoop[PropDriverNo];
}


/*******************************************************************************
* Name: PropCurrentSmooth
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void PropCurrentSmooth(xPropCurrentLoop *pPropCurrent, INT32S i32CurrentCmd)
{
	_iq CurrentErr;
	_iq Tmp;

	pPropCurrent->Cnt++;
	if((pPropCurrent->Cnt % 16) == 0) //1ms period
	{
		CurrentErr = i32CurrentCmd - pPropCurrent->CurrentCmdRaw;
		if ((0 != pPropCurrent->fCurveMinPara) && (0 != pPropCurrent->fCurveMaxPara))
		{
			if ((abs(CurrentErr) >= pPropCurrent->IRefMin) && 
				(i32CurrentCmd > pPropCurrent->IRefMin) && 
				(pPropCurrent->CurrentCmdRaw < pPropCurrent->IRefMin))
			{
				pPropCurrent->CurrentCmdRaw = pPropCurrent->IRefMin;
			}
			/*lilu 20230717 min current to 0*/
			else if ((abs(CurrentErr) >= pPropCurrent->IRefMin) && 
				(i32CurrentCmd < pPropCurrent->IRefMin) &&		
				((pPropCurrent->CurrentCmdRaw - pPropCurrent->IRefMin) < pPropCurrent->AccRatio))
			{
				pPropCurrent->CurrentCmdRaw = 0;
			}
			else
			{
				Tmp = pPropCurrent->AccRatio * (pPropCurrent->fCurveMinPara + pPropCurrent->fCurveMaxPara * (pPropCurrent->CurrentCmdRaw - pPropCurrent->IRefMin) / (pPropCurrent->IRefMax - pPropCurrent->IRefMin));
				if ((CurrentErr > 0) && (CurrentErr > Tmp))
				{
					pPropCurrent->CurrentCmdRaw += Tmp;
				}			
				else if ((CurrentErr < 0) && (abs(CurrentErr) > Tmp))
				{
					pPropCurrent->CurrentCmdRaw -= Tmp; 
					CurrentErr = -CurrentErr;
				}
				else
				{
					pPropCurrent->CurrentCmdRaw = i32CurrentCmd;
				}
			}
		}
		else
		{
			/*lilu 20230717 0 to min current*/
			if ((abs(CurrentErr) >= pPropCurrent->IRefMin) && 
				(i32CurrentCmd > pPropCurrent->IRefMin) && 
				(pPropCurrent->CurrentCmdRaw < pPropCurrent->IRefMin))
			{
				pPropCurrent->CurrentCmdRaw = pPropCurrent->IRefMin;
			}
			/*lilu 20230717 min current to 0*/
			else if ((abs(CurrentErr) >= pPropCurrent->IRefMin) && 
				(i32CurrentCmd < pPropCurrent->IRefMin) &&		
				(((pPropCurrent->CurrentCmdRaw - pPropCurrent->IRefMin) < pPropCurrent->AccRatio)))
			{
				pPropCurrent->CurrentCmdRaw = 0;
			}
			else
			{
				if(CurrentErr > 0)
				{
					pPropCurrent->CurrentCmdRaw += pPropCurrent->AccRatio;
				}
				else if(CurrentErr < 0)
				{
					pPropCurrent->CurrentCmdRaw -= pPropCurrent->AccRatio; 
					CurrentErr = -CurrentErr;
				}
			}
			if((CurrentErr < pPropCurrent->AccRatio))
			{			
				pPropCurrent->CurrentCmdRaw = i32CurrentCmd;	
			}			
			/* Prop driver cmd current update */
		}
		pPropCurrent->CurrentCmd = pPropCurrent->CurrentCmdRaw;
//		
		#if(1) //#ifdef PROP_DRIVER_DITHER
		/* Prop driver cmd current dither */
		{
			pPropCurrent->u16DitherCnt++;
			if (pPropCurrent->u16DitherCnt >= (pPropCurrent->u16DitherPeriod))	//1 Dither Period
			{
				pPropCurrent->u16DitherCnt = 0;
				pPropCurrent->i32DitherValue = 0;
			}
			
			if(pPropCurrent->u16DitherCnt < (pPropCurrent->u16DitherPeriod >> 1)) // 1/2 period Acc
			{
				pPropCurrent->i32DitherValue += pPropCurrent->u16DitherStep;
			}
			else // 1/2 period Dec
			{
				pPropCurrent->i32DitherValue -= pPropCurrent->u16DitherStep;
			}	
		}
		
		if (0 != pPropCurrent->CurrentCmd)
		{
			pPropCurrent->CurrentCmd += pPropCurrent->i32DitherValue;
		}
		#endif //#ifdef PROP_DRIVER_DITHER
		if (0 == (pPropCurrent->Cnt % 80) && (PropDriverCh0 == pPropCurrent->PropDriverNo))
		{
			static uint32_t u32PropCnt = 0;
			i32LogWrite(INFO, LOG_PROP, "CurrentCmd = %d, Target = %d, Iref = %d, Cnt = %d\r\n", pPropCurrent->CurrentCmd, i32CurrentCmd, pPropCurrent->IRef, u32PropCnt);
			u32PropCnt = u32PropCnt + 5;
//			//i32LogWrite(ERR, "Step = %d, AccRatio = %d, DitherValue = %d, Period = %d\r\n", \
//			pPropCurrent->u16DitherStep, pPropCurrent->AccRatio, pPropCurrent->i16DitherValue, pPropCurrent->u16DitherPeriod);			
		}

//		if (0 == (pPropCurrent->Cnt % 800) && (PropDriverCh1 == pPropCurrent->PropDriverNo))
//		{
//			//i32LogWrite(ERR, "CurrentCmd = %d, Target = %d, Iref = %d\r\n", pPropCurrent->CurrentCmd, i32CurrentCmd, pPropCurrent->IRef);
////			//i32LogWrite(ERR, "Step = %d, AccRatio = %d, DitherValue = %d, Period = %d\r\n", \
////			pPropCurrent->u16DitherStep, pPropCurrent->AccRatio, pPropCurrent->i16DitherValue, pPropCurrent->u16DitherPeriod);			
//		}		
	}			
}