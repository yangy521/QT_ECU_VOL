/*******************************************************************************
* Filename: Current.c                                             	 		   *
*                                                                    		   *
* Description: current loop implementation file of KSD.   			 		   *
* Author:                                                                   *
* Date: 100622														           *
* Revision:															 		   *
*																	 		   *
*******************************************************************************/

#include	"Current.h"
#include  "Speed.h"
//#include  "Bldc.h"
#include  "ServoPara.h"
#include	"CommonRam.h"
CURRENT_LOOP	gCurrentLoop;
/*******************************************************************************
* Name: CurrentFilterInit
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void CurrentFilterInit(CURRENT_FILTER *p)
{
	p->In = 0;
	p->Out = 0;
	p->Display = 0;
	p->DisplaySum = 0;
}

/*******************************************************************************
* Name: CurrentFilterCalc
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void CurrentFilterCalc(CURRENT_FILTER *p)
{
	/* t = 1ms */
	p->Out += (p->In - p->Out + 4) / 8;
	/* t = 8ms */
	p->DisplaySum += p->In - p->Display;
	p->Display = p->DisplaySum >> 10;
}

/*******************************************************************************
* Name: CurrentPidInit
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void CurrentPidInit(CURRENT_PID *p)
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
	p->OutMax = 0;
	p->OutMin = 0;
	p->OutPreSat = 0;
	p->SatErr = 0;
}

/*******************************************************************************
* Name: CurrentPidCalc
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void CurrentPidCalc(CURRENT_PID *v)
{
	_iq UiLimit;
	/* Compute the error */
	v->Err = v->Ref - v->Fdb;

	/* Compute the proportional output，比例项损失的精度不计 */
 	v->Up = _IQmpy(v->Kp, v->Err);

	/* Compute the integral output */
	v->Ui += (INT64S)v->Ki * (INT64S)v->Up + (INT64S)v->Kc * (INT64S)v->SatErr;
	/* Limit Ui*/
	UiLimit = (_iq)((v->Ui + (INT64S)(1 << (GLOBAL_Q-1))) >> GLOBAL_Q);
	if (UiLimit > v->OutMax)
	{
		v->Ui = ((INT64S)v->OutMax) << GLOBAL_Q;
	}
	else if (UiLimit < v->OutMin)
	{
		v->Ui = ((INT64S)v->OutMin) << GLOBAL_Q;
	}

	/* Compute the pre-saturated output */
	v->OutPreSat = v->Up + (_iq)((v->Ui + (INT64S)(1 << (GLOBAL_Q-1))) >> GLOBAL_Q);

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
* Name: ClarkCalc
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void ClarkCalc(CLARKE *v)
{
	v->Alpha = v->As;
 	v->Beta = _IQmpy((v->As + _IQmpy(_IQ(2.0), v->Bs)), _IQ(1.0 / SQRT3));
}

/*******************************************************************************
* Name: ParkCalc
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void ParkCalc(PARK *v)
{
	_iq Cosine, Sine;

	/* Using look-up IQ sine table */
 	Sine = _IQsinPU(v->Angle);
 	Cosine = _IQcosPU(v->Angle);

 	v->Ds = _IQmpy(v->Alpha, Cosine) + _IQmpy(v->Beta,  Sine);
 	v->Qs = _IQmpy(v->Beta,  Cosine) - _IQmpy(v->Alpha, Sine);
}
/*******************************************************************************
* Name: IParkCalc
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void IParkCalc(IPARK *v)
{
	_iq Cosine,Sine;
	v->CircleLimit(v);	//voltage circle limit 
	/* Using look-up IQ sine table */
 	Sine = _IQsinPU(v->Angle);
 	Cosine = _IQcosPU(v->Angle);

 	v->Alpha = _IQmpy(v->Ds, Cosine) - _IQmpy(v->Qs, Sine);
 	v->Beta = _IQmpy(v->Qs, Cosine) + _IQmpy(v->Ds, Sine);
}
/*******************************************************************************
* Name: DeadTimeInit
* Description:
* Input: 
* Output: 
*
* Author:
* Date:
* Revision:
*******************************************************************************/
void DeadTimeInit(DEAD_TIME *p)
{
	p->Value = 0;
	p->Sector = 0;
	p->idq = 0;	
	p->idq_old = 0;
	p->ValueMax = _IQ(0.0064);					/*0.0064 <= 1.6us/62.5us/2 */
	p->ThresholdMax = _IQ(1.0/STD_CURRENT);		/* 1.5A */
 	p->k = _IQdiv(p->ValueMax, p->ThresholdMax);
}

/*******************************************************************************
* Name: DeadTimeCalc
* Description:
* Input: 
* Output: 
*
* Author:
* Date:
* Revision:
*******************************************************************************/
void DeadTimeCalc(DEAD_TIME *p)
{
}

/*******************************************************************************
* Name: CurrentLoopInit
* Description: current loop initialization
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void CurrentLoopInit(CURRENT_LOOP *p)
{
	/* initialize Clarke */
	p->Clarke.Alpha = 0;
	p->Clarke.Beta = 0;
	p->Clarke.As = 0;
	p->Clarke.Bs = 0;
	p->Clarke.Calc = ClarkCalc;

	/* initialize Park */
	p->Park.Alpha = 0;
	p->Park.Beta = 0;
	p->Park.Angle = 0;
	p->Park.Ds = 0;
	p->Park.Qs = 0;
	p->Park.Calc = ParkCalc;

	/* initialize Id PID struct varibale */
	p->IdPID.Init = CurrentPidInit;
	p->IdPID.Calc = CurrentPidCalc;
	p->IdPID.Init(&p->IdPID);
	p->IdPID.OutMax = VOLTAGE_MODULATE_MAX; //
	p->IdPID.OutMin = VOLTAGE_MODULATE_MIN;
//	p->IdPID.OutMax = _IQ(1.0) - 1; //
//	p->IdPID.OutMin = -_IQ(1.0) + 1;

	/* initialize Iq PID struct varibale */
	p->IqPID.Init = CurrentPidInit;
	p->IqPID.Calc = CurrentPidCalc;
	p->IqPID.Init(&p->IqPID);
	p->IqPID.OutMax = VOLTAGE_MODULATE_MAX;
	p->IqPID.OutMin = VOLTAGE_MODULATE_MIN;
//	p->IqPID.OutMax = _IQ(1.0) - 1;
//	p->IqPID.OutMin = -_IQ(1.0) + 1;
	/* D轴和Q轴的饱和值需要重新设定 */

	/* initialize IPark */
	p->Ipark.Alpha = 0;
	p->Ipark.Beta = 0;
	p->Ipark.Angle = 0;
	p->Ipark.Ds = 0;
	p->Ipark.Qs = 0;
	p->Ipark.Calc = IParkCalc;
	//p->Ipark.CircleLimit = IPark_Circle_Limitation1;

	/* initialize SvGen */
	p->SvGen.Ta = _IQ(0);
	p->SvGen.Tb = _IQ(0);
	p->SvGen.Tc = _IQ(0);
	p->SvGen.Ualpha = 0;
	p->SvGen.Ubeta = 0;
	p->SvGen.DTsector = 0;
	p->SvGen.DTvalue = 0;
	p->SvGen.HighCountA = 0;
	p->SvGen.HighCountB = 0;
	p->SvGen.HighCountC = 0;
//	p->SvGen.TMin = _IQ((DEADTIME_US*4)/62.5);
//	p->SvGen.TMax = _IQ(1.0-(DEADTIME_US*4)/62.5);
//	p->SvGen.TMin = _IQ(1.0-(DEADTIME_US*4)/31.25);
	p->SvGen.TMax = _IQ(1.0)-1;
//	p->SvGen.Calc = SvGenDqCalc1;

	/* initialize PWMGen struct varibale */
	p->PwmGen.PeriodMax = (INT16U)(PWM_PERIOD);
	p->PwmGen.MfuncPeriod = 0x8000;
	p->PwmGen.MfuncC1 = 0x4000;
	p->PwmGen.MfuncC2 = 0x4000;
	p->PwmGen.MfuncC3 = 0x4000;
	p->PwmGen.Update = PwmUpdate;

	/* dead time */
	p->DeadTime.Init = DeadTimeInit;
	p->DeadTime.Calc = DeadTimeCalc;
	p->DeadTime.Init(&p->DeadTime);
	
	/* current fdb filter */
	p->CurrentFilter.Init = CurrentFilterInit;
	p->CurrentFilter.Calc = CurrentFilterCalc;
	p->CurrentFilter.Init(&p->CurrentFilter);
	/* initialize other variables */
	p->State = 0;
	p->U = 0;
	p->W = 0;
	p->V = 0;
	p->RotorAngle = 0;
	p->hRotFlx_Theta = 0;
	p->IqRef = 0;
	p->IqFdb = 0;

	p->IdRef = 0;
	p->IdFdb = 0;
	p->BackELecForce = 0;
	p->SpeedFilter = 0;
	p->DTvalue = 0;
	p->DTsector = 0;

	p->IvFdb = 0;
	p->VfRef = 0;
	p->VbusFdb = 0;
}

/*******************************************************************************
* Name: CurrentLoopCalc
* Description: current loop calculation
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
#ifndef _RELEASE
//#define DEBUG_CURRENT_OPENLOOP
//#define DEBUG_CURRENT_CLOSELOOP
//#define DEBUG_ANGLE
#endif
#ifdef DEBUG_ANGLE
INT64S  HallAngle = 0;
INT64S  DugAngle = 0;
extern INT32S gTheta;
#endif
#define  CURRENT_OPEN_LIMIT
void CurrentLoopCalc(CURRENT_LOOP *p)
{
	INT32S CurrentFdbU,CurrentFdbW;
	_iq		DriveCurrentLimit;		/* Q24 = STD_CURRENT  */
	
	if(SL_CHK(PLC_TER_CURRENT_CUT))
		DriveCurrentLimit = gCRam.SvPa.MotorPara.RatedCurrent*OVER_TEMP_CUT_RATIO;
	else
		DriveCurrentLimit = gCRam.SvPa.MotorPara.RatedCurrent;
	
	if(p->U < 0)  CurrentFdbU = -p->U;
	else					CurrentFdbU = p->U;
	if(p->W < 0)  CurrentFdbW = -p->W;
	else					CurrentFdbW = p->W;
	/* Get Max feedback Current  */
	if(CurrentFdbU>CurrentFdbW)
	{
		p->IqPID.Fdb = -p->U;
	}
	else
	{
		p->IqPID.Fdb = p->W;
	}
	/* filter fdb */
	p->CurrentFilter.In = (CurrentFdbU>CurrentFdbW) ? CurrentFdbU:CurrentFdbW;
	p->CurrentFilter.Calc(&p->CurrentFilter);
	
	#ifdef CURRENT_OPEN_LIMIT
	if(p->IqPID.Fdb > DriveCurrentLimit) 
	{
		p->IqPID.Ref = DriveCurrentLimit;	
		
		SL_SET(PLC_CURRENT_LIMIT);		//过流标志
	}
	else if(p->IqPID.Fdb < -DriveCurrentLimit) 
	{	
		p->IqPID.Ref = -DriveCurrentLimit;
		
		SL_SET(PLC_CURRENT_LIMIT);		//过流标志		
	}
	else
	{
		p->IqPID.Ref = p->IqPID.Fdb;
		
		/* reset current loop */
		p->IqPID.Ui = 0;	//clear PID integal
		p->IqPID.Up = 0;
		p->IqPID.Out = 0;
		
		SL_CLR(PLC_CURRENT_LIMIT);		//清过流标志	
	}
	
	/* PID Regulator Calc	*/
	p->IqPID.Calc(&p->IqPID); 

	p->SvGen.DTvalue = p->IqRef + p->IqPID.Out;
	
	#else
	p->IqPID.Fdb = -p->U;	
	p->IqPID.Ref = p->IqRef;
	
	/* PID Regulator Calc	*/
	p->IqPID.Calc(&p->IqPID); 	
	#endif	// CURRENT_OPEN_LIMIT	
	

	//电流均衡
//#define CURRENT_BLANCE
#ifdef  CURRENT_BLANCE
	static INT32S BlanceCutU =0;	//U相均衡消减
	static INT32S BlanceCutW =0;	//W相均衡消减
	
	if((p->CurrentFilter.In > _IQ(40/STD_CURRENT))	//>40A
	&&(gCRam.SpeedCmd != 0))
	{	
		if((CurrentFdbW-CurrentFdbU)> _IQ(20/STD_CURRENT))	//differ 20A
		{
			BlanceCutU=0;	
			if(p->SvGen.DTvalue>0)
			{
				if(BlanceCutW < (p->SvGen.DTvalue>>1))
					BlanceCutW += 256;	
				else
					BlanceCutW = (p->SvGen.DTvalue>>1);
			}
			else
			{
				if(BlanceCutW > (p->SvGen.DTvalue>>1))
					BlanceCutW -= 256;
				else
					BlanceCutW = (p->SvGen.DTvalue>>1);				
			}
		}			
		else if((CurrentFdbU-CurrentFdbW)> _IQ(20/STD_CURRENT))	//differ 20A		
		{
			BlanceCutW=0;	
			if(p->SvGen.DTvalue>0)
			{
				if(BlanceCutU < (p->SvGen.DTvalue>>1))
					BlanceCutU += 256;	
				else
					BlanceCutU = (p->SvGen.DTvalue>>1);
			}
			else
			{
				if(BlanceCutU > (p->SvGen.DTvalue>>1))
					BlanceCutU -= 256;
				else
					BlanceCutU = (p->SvGen.DTvalue>>1);				
			}
		}	
		else
		{
			if(BlanceCutU > 256L)
				BlanceCutU -= 256;
			else if(BlanceCutU < -256L)
				BlanceCutU += 256;
			else
				BlanceCutU = 0;

			if(BlanceCutW > 256L)
				BlanceCutW -= 256;
			else if(BlanceCutW < -256L)
				BlanceCutW += 256;
			else
				BlanceCutW = 0;	
		}
	}	
	else
	{
		if(BlanceCutU > 256L)
			BlanceCutU -= 256;
		else if(BlanceCutU < -256L)
			BlanceCutU += 256;
		else
			BlanceCutU = 0;

		if(BlanceCutW > 256L)
			BlanceCutW -= 256;
		else if(BlanceCutW < -256L)
			BlanceCutW += 256;
		else
			BlanceCutW = 0;	
	}

	/* 4. SVGEN */
	/* set SVGEN input */
	if (p->SvGen.DTvalue > 0)
	{
		p->SvGen.Ta = p->SvGen.DTvalue-BlanceCutW;//p->IqPID.Out;
		p->SvGen.Tb = 0;
		p->SvGen.Tc = p->SvGen.DTvalue-BlanceCutU;//p->IqPID.Out;
	}
	else if (p->SvGen.DTvalue < 0)
	{
		p->SvGen.Ta = p->SvGen.TMax+p->SvGen.DTvalue-BlanceCutW;
		p->SvGen.Tb = _IQ(0.98);//p->SvGen.TMax;//
		p->SvGen.Tc = p->SvGen.TMax+p->SvGen.DTvalue-BlanceCutU;
	}		
	else //zero
	{
		p->SvGen.Ta = p->SvGen.TMax>>1;
		p->SvGen.Tb = p->SvGen.TMax>>1;
		p->SvGen.Tc = p->SvGen.TMax>>1;
	}
#else

	/* 4. SVGEN */
	/* set SVGEN input */
	if (p->SvGen.DTvalue > 0)
	{
		p->SvGen.Ta = p->SvGen.DTvalue;//p->IqPID.Out;
		p->SvGen.Tb = 0;
		p->SvGen.Tc = p->SvGen.DTvalue;//p->IqPID.Out;
	}
	else if (p->SvGen.DTvalue < 0)
	{
		p->SvGen.Ta = p->SvGen.TMax+p->SvGen.DTvalue;
		p->SvGen.Tb = _IQ(0.98);//p->SvGen.TMax;//
		p->SvGen.Tc = p->SvGen.TMax+p->SvGen.DTvalue;
	}		
	else //zero
	{
		p->SvGen.Ta = p->SvGen.TMax>>1;
		p->SvGen.Tb = p->SvGen.TMax>>1;
		p->SvGen.Tc = p->SvGen.TMax>>1;
	}
#endif	// #ifdef  CURRENT_BLANCE
		
#ifdef DEBUG_CURRENT_OPENLOOP
		p->SvGen.Ta = 0;
		p->SvGen.Tb = 0;
		p->SvGen.Tc = 0;
#endif //#ifdef DEBUG_CURRENT_OPENLOOP
	/* execute SVGEN */
	//p->SvGen.Calc(&p->SvGen);

	/* 5. PWMGEN */
	/* set PWMGEN input */
	p->PwmGen.MfuncC1 = (INT16S)(_IQtoIQ15(p->SvGen.Ta));
	p->PwmGen.MfuncC2 = (INT16S)(_IQtoIQ15(p->SvGen.Tb));
	p->PwmGen.MfuncC3 = (INT16S)(_IQtoIQ15(p->SvGen.Tc));
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
void CurrentLoopReset(CURRENT_LOOP *p)
{
	/* reset Clarke */
	p->Clarke.Alpha = 0;
	p->Clarke.Beta = 0;
	p->Clarke.As = 0;
	p->Clarke.Bs = 0;

	/* reset Park */
	p->Park.Alpha = 0;
	p->Park.Beta = 0;
	p->Park.Angle = 0;
	p->Park.Ds = 0;
	p->Park.Qs = 0;

	/* reset Id PID struct varibale */
	p->IdPID.Out = 0;
	p->IdPID.Ref = 0;
	p->IdPID.Fdb = 0;
	p->IdPID.Err = 0;
	p->IdPID.Up = 0;
	p->IdPID.Ui = 0;
	p->IdPID.OutPreSat = 0;
	p->IdPID.SatErr = 0;

	/* reset Iq PID struct varibale */
	p->IqPID.Out = 0;
	p->IqPID.Ref = 0;
	p->IqPID.Fdb = 0;
	p->IqPID.Err = 0;
	p->IqPID.Up = 0;
	p->IqPID.Ui = 0;
	p->IqPID.OutPreSat = 0;
	p->IqPID.SatErr = 0;

	/* reset IPark */
	p->Ipark.Alpha = 0;
	p->Ipark.Beta = 0;
	p->Ipark.Angle = 0;
	p->Ipark.Ds = 0;
	p->Ipark.Qs = 0;

	/* reset SvGen */
	p->SvGen.Ta = 0;
	p->SvGen.Tb = 0;
	p->SvGen.Tc = 0;
	p->SvGen.Ualpha = 0;
	p->SvGen.Ubeta = 0;
	p->SvGen.DTsector = 0;
	p->SvGen.DTvalue = 0;
	p->SvGen.HighCountA = 0;
	p->SvGen.HighCountB = 0;
	p->SvGen.HighCountC = 0;

	/* dead time */
	p->DeadTime.Value = 0;
	p->DeadTime.Sector = 0;
	p->DeadTime.idq = 0;
	p->DeadTime.idq_old = 0;

	/* reset other variables */
	p->State = 0;
	p->U = 0;
	p->W = 0;
	p->V = 0;
	p->RotorAngle = 0;
	p->hRotFlx_Theta = 0;
	p->IqRef = 0;
	p->IqFdb = 0;
	p->IdRef = 0;
	p->IdFdb = 0;
	p->BackELecForce = 0;
	p->SpeedFilter = 0;
	p->DTvalue = 0;
	p->DTsector = 0;

	p->IvFdb = 0;
	p->VbusFdb = 0;

	/* current fdb filter*/
	p->CurrentFilter.Init(&p->CurrentFilter);
}

