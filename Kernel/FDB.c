/*******************************************************************************
* Filename: FDB.c                                             	 		   	   *
*                                                                    		   *
* Description:											   			 		   *
* Author:                                                                   *
* Date: 														           *
* Revision:															 		   *
*																	 		   *
*******************************************************************************/

#include  "FDB.h"
#include "Kernel.h"
#include	"CommonRam.h"
#include "Device.h"
#include 	"PARA.h"
#include "Current.h"
#include "gd32f30x.h"

CURRENT_SAMPLE	gCurrentSample;
//ENCODER			gEncoder;
//HALL_FB			gHall;

/*******************************************************************************
* Name: EncoderIncInit
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
#define T_TIMER_CLOCK  (SystemCoreClock/2)     //60 000 000  60M

void EncoderIncInit(ENCODER *p)
{
}


/*******************************************************************************
* Name: CurrentSampleInit
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
#define CURRENT_ZERO_DRIFT_DEFALT		0x740
#define CURRENT_ZERO_DRIFT_DIV			8
#define ANOLOG_SAMPLE_DIV  					3
#define CMDVAL_SAMPLE_DIV  					5

void CurrentSampleInit(CURRENT_SAMPLE *p)
{
	p->U = 0;
	p->W = 0;
	p->URaw = 0;
	p->WRaw = 0;
	p->ThrotPotHighRaw = 0;
	p->ThrotPotWipRaw = 0;
	p->UZeroDrift = CURRENT_ZERO_DRIFT_DEFALT;
	p->WZeroDrift = CURRENT_ZERO_DRIFT_DEFALT;
	p->UZeroDriftSum = CURRENT_ZERO_DRIFT_DEFALT << CURRENT_ZERO_DRIFT_DIV;
	p->WZeroDriftSum = CURRENT_ZERO_DRIFT_DEFALT << CURRENT_ZERO_DRIFT_DIV;
	p->PZeroDrift = CURRENT_ZERO_DRIFT_DEFALT;
	p->PZeroDriftSum = CURRENT_ZERO_DRIFT_DEFALT << CURRENT_ZERO_DRIFT_DIV;
	
	p->VBus = 0;
	p->VBusRaw = 0;
	p->VBusZeroDrift = 0;
	p->VBusZeroDriftSum = 0;

	p->KsiVBus = 0;
	p->KsiVBusRaw = 0;
	p->KsiVBusZeroDrift = 0;;
	p->KsiVBusZeroDriftSum = 0;

	p->V5out = 0;
	p->V12out = 0;
	p->RelayOut = 0;
	p->PowTmpHigh = 0;
	p->PowTmpLow = 0;
	p->MotorTmp = 0;
	p->Swi7 = 0;
	p->Swi8 = 0;
	p->PhaseP = 0;
	p->SpdPotWip = 0;

	p->VBusSum = 0;
	p->V5outSum = 0;
	p->RelayOutSum = 0;
	p->PowTmpHighSum = 0;	
	p->PhasePSum = 0;
	p->SpdPotWipSum = 0;

	p->V5outRaw = 0;
	p->V12outRaw = 0;
	p->RelayOutRaw = 0;
	p->PowTmpHighRaw = 0;
	p->MotorTmpRaw = 0;
	p->Swi7Raw = 0;
	p->Swi8Raw = 0;
	p->PhasePRaw = 0;
	p->SpdPotWipRaw = 0;
	
}

/*******************************************************************************
* Name: CurrentSampleCalc
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void CurrentSampleCalc(CURRENT_SAMPLE *p)
{
	/*
	//ADC1
	Rank1		PA0 ADC1_IN1	THRO_WIP2;			Rank2		PA1 ADC1_IN2	POT2_High		
	Rank3		PA2 ADC1_IN3	PhaseU-R;				Rank4		PC0 ADC1_IN6	VBus		
	Rank5		PC1 ADC1_IN7	KSI;						Rank6		PC2 ADC1_IN8	PROP_CHECK	
	Rank7		PB12 ADC1_IN11	PhaseW-R;			Rank8		PB1 ADC1_IN12	POT_High	
	Rank9		PB0 ADC1_IN15	THRO_WIP	
	
	//ADC2
	Rank1		PA6 ADC2_IN3	TMP;						Rank2		PA7 ADC2_IN4	V5out		
	Rank3		PC4 ADC2_IN5	SW1_R;					Rank4		PC5 ADC2_IN11	V12out		
	Rank5		PB2 ADC2_IN12	PhaseV-R;			  Rank6		PA5 ADC2_IN13	MOTOR_TMP	
	
	//ADC1&ADC2 Injected 	
	ADC1 INJECTED_RANK_1	PA3 ADC1_IN4   IW
	ADC2 INJECTED_RANK_1	PA4 ADC2_IN17  IU
	*/
#ifdef CPU_STM32G473
	p->WRaw = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_1);  //电流检测
	p->URaw = HAL_ADCEx_InjectedGetValue(&hadc2,ADC_INJECTED_RANK_1);
	p->PRaw = HAL_ADCEx_InjectedGetValue(&hadc3,ADC_INJECTED_RANK_1);
#endif
	p->URaw = inserted_data[0];//adc_inserted_data_read(ADC0, ADC_INSERTED_CHANNEL_0);  //电流检测
	p->WRaw = inserted_data[1];//adc_inserted_data_read(ADC1, ADC_INSERTED_CHANNEL_0);
	p->PRaw = inserted_data[2];//adc_inserted_data_read(ADC2, ADC_INSERTED_CHANNEL_0);
	
//	ADC_SoftwareStartConv(ADC1);	//启动转换
	//p->Swi1= (INT16U)ADC2Value[0];	  //SW1-R/AD
	p->PowTmpHighRaw= (INT16U)ADCValue[0];	  //TMP
	p->MotorTmpRaw= (INT16U)ADCValue[1];		//Motor TMP
	//p->V5_12outRaw= (INT16U)ADCValue[2];	  //5V-12V-OUT	
	p->KsiVBusRaw = (INT16U)ADCValue[3];	  //KSI
	p->VBusRaw = (INT16U)ADCValue[4];	  //VBus
	p->ThrotPotWipRaw = (INT16U)ADC2Value[1];	  //THROWIP1
	p->SpdPotWipRaw = (INT16U)ADC2Value[2];	  //THROWIP2
	p->PropOvCur = (INT16U)ADCValue[5];	  //PROP-CHECK
	//(INT16U)ADCValue[6];	  //ADC01_IN15
	p->VrefRaw = (INT16U)ADCValue[7];	  //Vref
	
#ifndef	ANALOG_SWITCH
	static INT16U ADC3_Idx=0;	
	static INT16U cnt=0;	
	if(((cnt++)&0x01) == 0)		//2*125us
	{
		//Get ADC value
		ADC3Value[ADC3_Idx] =	(INT16U)ADCValue[6];
					
		p->PhaseURaw = ADC3Value[0];
		p->PhaseVRaw = ADC3Value[1];
		p->PhaseWRaw = ADC3Value[2];
		p->PhasePRaw = ADC3Value[3];
		p->V5outRaw = ADC3Value[4];
		p->V12outRaw = ADC3Value[5];
		p->ThrotPotHighRaw = ADC3Value[6];
				
		if(ADC3_Idx<7)	
			ADC3_Idx++;	//Change ADC channel	
		else
			ADC3_Idx=0;	
		
		if(ADC3_Idx&0x01)
			ANALOG_SW0(GPIO_PIN_SET);
		else
			ANALOG_SW0(GPIO_PIN_RESET);		
		
		if(ADC3_Idx&0x02)
			ANALOG_SW1(GPIO_PIN_SET);
		else
			ANALOG_SW1(GPIO_PIN_RESET);	
		
		if(ADC3_Idx&0x04)
			ANALOG_SW2(GPIO_PIN_SET);
		else
			ANALOG_SW2(GPIO_PIN_RESET);		
	}
#endif	//#ifdef	ANALOG_SWITCH
		
//	p->USum += ((p->URaw - p->UZeroDrift) -	p->U); 
//	p->U = (INT16S)(p->USum >> ANOLOG_SAMPLE_DIV);
	p->U = (INT16S)(p->URaw - p->UZeroDrift); 	
	
//	p->WSum += ((p->WRaw - p->WZeroDrift) -	p->W); 
//	p->W = (INT16S)(p->WSum >> ANOLOG_SAMPLE_DIV);	
	p->W = (INT16S)(p->WRaw - p->WZeroDrift); 
	
//	p->PSum += ((p->PRaw - p->PZeroDrift) -	p->P); 
//	p->P = (INT16S)(p->PSum >> ANOLOG_SAMPLE_DIV);
	p->P = (INT16S)(p->PRaw - p->PZeroDrift); 	
	
//	p->VBus = p->VBusRaw;
	p->VBusSum += (p->VBusRaw - p->VBus);
	p->VBus = (INT16S)(p->VBusSum >> ANOLOG_SAMPLE_DIV);	
//	p->KsiVBus = p->KsiVBusRaw;	
	p->KsiVBusSum += (p->KsiVBusRaw - p->KsiVBus);
	p->KsiVBus = (INT16S)(p->KsiVBusSum >> ANOLOG_SAMPLE_DIV);	
	p->V5outSum += (p->V5outRaw - p->V5out);
	p->V5out = (INT16S)(p->V5outSum >> ANOLOG_SAMPLE_DIV);	
	p->V12out = p->V12outRaw;
	p->PowTmpHigh = p->PowTmpHighRaw;		
	p->ThrotPotHigh = p->ThrotPotHighRaw;
	p->ThrotPotWipSum += p->ThrotPotWipRaw - p->ThrotPotWip;
	p->ThrotPotWip = p->ThrotPotWipSum >> ANOLOG_SAMPLE_DIV;
	p->SpdPotWipSum += p->SpdPotWipRaw - p->SpdPotWip;
	p->SpdPotWip = p->SpdPotWipSum >> ANOLOG_SAMPLE_DIV;
	p->MotorTmp = p->MotorTmpRaw;
//	p->Swi7 = p->Swi7Raw;
//	p->Swi8 = p->Swi8Raw;
	p->PhaseUSum += (p->PhaseURaw - p->PhaseU);
	p->PhaseU = (INT16S)(p->PhaseUSum >> ANOLOG_SAMPLE_DIV);
	p->PhaseVSum += (p->PhaseVRaw - p->PhaseV);
	p->PhaseV = (INT16S)(p->PhaseVSum >> ANOLOG_SAMPLE_DIV);
	p->PhaseWSum += (p->PhaseWRaw - p->PhaseW);
	p->PhaseW = (INT16S)(p->PhaseWSum >> ANOLOG_SAMPLE_DIV);
	p->PhasePSum += (p->PhasePRaw - p->PhaseP);
	p->PhaseP = (INT16S)(p->PhasePSum >> ANOLOG_SAMPLE_DIV);	
}

/*******************************************************************************
* Name: CurrentSampleCalcZeroDrift
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void CurrentSampleCalcZeroDrift(CURRENT_SAMPLE *p)
{
	/* calculate current zero drift using LPF */
	p->UZeroDriftSum += (p->URaw - p->UZeroDrift);
	p->UZeroDrift = (INT16S)(p->UZeroDriftSum >> CURRENT_ZERO_DRIFT_DIV);
	p->WZeroDriftSum += (p->WRaw - p->WZeroDrift);
	p->WZeroDrift = (INT16S)(p->WZeroDriftSum >> CURRENT_ZERO_DRIFT_DIV);
}

/*******************************************************************************
* Name: CurrentSampleCalcZeroDrift
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void CurrentSamplePropCalcZeroDrift(CURRENT_SAMPLE *p)
{
	/* calculate current zero drift using LPF */
	p->PZeroDriftSum += (p->PRaw - p->PZeroDrift);
	p->PZeroDrift = (INT16S)(p->PZeroDriftSum >> CURRENT_ZERO_DRIFT_DIV);
}

/*******************************************************************************
* Name: GetIncEncoderUVW
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
//INT16U GetIncEncoderUVW(void)
//{
//	return (INT16U)(1 & 0x0007);
//}
/*******************************************************************************
* Name: EncoderIncCalc
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
//static void EncoderIncCalc(ENCODER *p)
//{

//	//use T spdraw
//	p->SpdRaw = 0;
//	p->ElecTheta = 0;

//	//use M spdraw
//	//p->SpdRaw = p->MSpdRaw;
//	//p->ElecTheta = p->ElecThetaM;
//	

//}
/*******************************************************************************
* Name: InitFeedbackDevice
* Description:
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void InitFeedbackDevice(void)
{
	/* 1. encoder feedback initialize */
//	gEncoder.Init = EncoderIncInit;
//	gEncoder.Calc = EncoderIncCalc;
//	/* execute init function */
//	gEncoder.Init(&gEncoder);
//	/* 第一次数据读取 */
//	gEncoder.Calc(&gEncoder);

	/* 2. Current Sample feedback initialize */
	gCurrentSample.Init = CurrentSampleInit;
	gCurrentSample.Calc = CurrentSampleCalc;
	gCurrentSample.Calc_ZeroDrift = CurrentSampleCalcZeroDrift;
	gCurrentSample.Init(&gCurrentSample);
#if ((CTLBOARD_TYPE ==_1226)||(CTLBOARD_TYPE ==_1226_GD32))
	gCurrentSample.Calc_PropZeroDrift = CurrentSamplePropCalcZeroDrift;
#endif //#if (CTLBOARD_TYPE ==_1226)

	/* 3. line scale feedback initialize */
}
/*******************************************************************************
* Name: FeedbackOvLoadCheck
* Description: 1s period. Run in background.
* Input: 
* Output: 
* 
* Author:
* Date:
* Revision:
*******************************************************************************/
 #ifdef MAX_PHASECUR_MONITOR
//INT32S gUVWX[9];
#endif
void FeedbackOvLoadCheck(void)
{
	INT32S UQ24,VQ24,WQ24,PQ24;

	/* 获取最大电流值  */
	if ((UQ24 = _IQ11toIQ(gCRam.CurrentSampleU)) < 0)
			UQ24 = -UQ24;
//	if ((VQ24 = _IQ11toIQ(gCRam.CurrentSampleU-gCRam.CurrentSampleW)) < 0)
//			VQ24 = -VQ24;
	VQ24 = 0;
	if ((WQ24 = _IQ11toIQ(gCRam.CurrentSampleW)) < 0)
			WQ24 = -WQ24;

	if (UQ24 < VQ24)
			UQ24 = VQ24;
	if (UQ24 < WQ24)
			UQ24 = WQ24;

	#if (SBDCDRIVER_TYPE == SBDCDRIVER_ENABLE)
	if ((PQ24 = _IQ11toIQ(gCurrentSample.P)) < 0)
			PQ24 = -PQ24;

	#endif
	/* No.12: 电机电流瞬时超过最大过载值 */
	if (UQ24 >= gKernelCtl.MaxOverLoadCurrent)
	{
			PwmDisableEsp();
			SL_SET(SL_OVER_CURRENT_ERR);
	}
	if (PQ24 >= gKernelCtl.PumpMaxOverLoadCurrent)
	{
			PwmDisableEsp();
			SL_SET(SL_OVER_CURRENT_ERR);
	}
	#ifdef MAX_PHASECUR_MONITOR
	if (gCRam.CurrentSampleMax < UQ24)
	{
			gCRam.CurrentSampleMax = UQ24;
	}
	#endif //MAX_PHASECUR_MONITOR
	
//	if (SL_CHK(SL_KSICHK_EN))
	{
		if ((_IQ12toIQ(gCRam.CurrentSampleVBus) > (gKernelCtl.EspHighVoltage))  
//		 || (_IQ12toIQ(gCRam.CurrentSampleKsiVBus) < (gKernelCtl.EspLowVoltage - _IQ(2.0/STD_VBUS)))
		   )
		{
				PwmDisableEsp();
				SL_SET(SL_ESP_VOLTAGE_ERR);
		}
	}
}
/*******************************************************************************
* Name: FeedbackProcess
* Description: Run in ISR.
* Input: 
* Output: 
*
* Author:
* Date: 
* Revision:
*******************************************************************************/
void FeedbackProcess(void)
{
	/* process */
	/* 1. current sample */
	gCurrentSample.Calc(&gCurrentSample);
	/* set common ram area */
	gCRam.CurrentSampleU = gCurrentSample.U;
	gCRam.CurrentSampleW = gCurrentSample.W;
	gCRam.CurrentSampleVBus = gCurrentSample.VBus;
	FeedbackOvLoadCheck();
//#if (FEEDBACK_TYPE == ENCODER_FEEDBACK)
//	/* 2. encoder */
//	gEncoder.Calc(&gEncoder);
//	
//	/* set common ram area */
//	gCRam.PosFdb = gEncoder.PosAbs;
//	gCRam.RotorElecAngle = (INT32S)gEncoder.ElecTheta;
//	gCRam.SpeedFdb = gEncoder.SpdRaw;	
//	
//#elif (FEEDBACK_TYPE == HALL_FEEDBACK)
//	/* 3. Hall feedback process */	
//	gHall.Calc(&gHall);
//	
//	/* set common ram area */
//	gCRam.PosFdb = 0;
//	gCRam.SpeedFdb = (INT32S)gHall.SpdFdbAvg;//<<16;

//	E=U-Ir 根据反电动势计算速度
		_iq iqPWMduty = gCurrentLoop.SvGen.DTvalue;
		if(iqPWMduty<0) iqPWMduty = -iqPWMduty;
		gCRam.BemfVoltage = _IQmpy(iqPWMduty, gPara.VBusVoltage) - _IQmpy((_iq)gPara.StatorResist * _IQ(1.0/1000), gPara.AcPhaseCurrent);
		if(gCRam.BemfVoltage<1)	
			gCRam.BemfVoltage = 1;
		gCRam.SpeedFdb = _IQdiv(gCRam.BemfVoltage, (_iq)gPara.TorqueCoe1 * _IQ(1.0/1000));
		//gCRam.SpeedCmd = _IQrmpy(p->SvGen.DTvalue, gCRam.SvPa.AcMotorTypicalSpdF);
//#endif
}

