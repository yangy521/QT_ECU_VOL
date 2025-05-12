#define PLC_PERIOD					5
#define LED_BLINK_PERIOD 		2000
#include "KSDsys.h"
#include "CommonRam.h"
#include "Device.h"

/* PLC control */
// typedef struct
// {
// 	/* DI��DO��AI AO*/
// 	DI_FILTER		diDataIn;
// 	INT16U			doDataOut[4];
// #if (CTLBOARD_TYPE ==_1236)
// 	INT16U aiDataIn[16];
// 	INT16U			aoDataOut;
// #endif //#if (CTLBOARD_TYPE ==_1236)
// 	
// #if (CTLBOARD_TYPE ==_1232)
// 	INT16U aiDataIn[14];
// #endif  //#if (CTLBOARD_TYPE ==_1232)

// 	#if (CTLBOARD_TYPE ==_1222)
// 	INT16U aiDataIn[12];
// #endif  //#if (CTLBOARD_TYPE ==_1222)

// 	INT16U			doConnect[4];
// 	INT16U			PulseWidth[4];
// 	INT16S			PulseWidthDelay[4];
// 	
// 	INT8U			  SigLampSvOff;
// 	INT8U			  SigLampSvOffOld;
// 	
// 	INT8U				CmdDirection;	//ָ���0:��λ��ֹͣ����1������2������
// 	INT16U			CurSpeedRate;
// 	INT16S      rateLow,rateHigh; //�ٷֱ�/0.1V, _IQ8
// 	INT16U			deadBandMin,deadBandMax,throttleMid;
// 	INT16U			throttleOutput;	//��λ�ٷֱ�
// 	INT16U			speedRate;			//�ٶȵ�λ���ٷֱȣ�
// 	INT16U			throttleWip;		//̤�廬���˵�ѹֵ��0.1V��
// 	INT16U			AcMotorMaxSpdFAct; //��ǰʹ�õ�����ٶ�

// 	INT8U				BrakePedalCmdDirection;	//ָ���0:��λ��ֹͣ����1������2������
// 	INT16U			BrakePedalCurSpeedRate;
// 	INT16S      BrakePedalrateLow,BrakePedalrateHigh; //�ٷֱ�/0.1V, _IQ8
// 	INT16U			BrakePedaldeadBandMin,BrakePedaldeadBandMax,BrakePedalMid;
// 	INT16U			BrakePedalOutput;	//��λ�ٷֱ�
// 	INT16U			BrakePedalspeedRate;			//�ٶȵ�λ���ٷֱȣ�
// 	INT16U			BrakePedalWip;		//̤�廬���˵�ѹֵ��0.1V��
// 	INT16U			BrakePedalAcMotorMaxSpdFAct; //��ǰʹ�õ�����ٶ�

// 	INT8U				SidleCmdDirection;	//ָ���0:��λ��ֹͣ����1������2������
// 	INT16U			SidleWip;		//̤�廬���˵�ѹֵ��0.1V��
// 	INT16U			SidleOutput;	//��λ�ٷֱ�
// 	INT8U				PitchCmdDirection;	//ָ���0:��λ��ֹͣ����1������2������
// 	INT16U			PitchWip;		//̤�廬���˵�ѹֵ��0.1V��
// 	INT16U			PitchOutput;	//��λ�ٷֱ�

// 	INT16U			ActOutput;	//��λ�ٷֱ�

// 	INT16S			TmpMotor;	//����¶�
// 	INT16S			TmpPower;	//���ʰ��¶�
// 	INT16S			TmpDrive;	//�������¶�
// 	INT16U			TmpFlag;  //loop ctl flag
// 	INT16S			TmpCount; //loop ctl flag
// 	INT32S 			TmpPowerLSum;
// 	INT32S 			TmpPowerHSum;
// 	INT32S 			TmpPowerL;
// 	INT32S 			TmpPowerH;
// 	INT32S 			TmpMotorAdSum;
// 	INT32S 			TmpMotorAd;
// 	INT32S 			TmpDriveAdSum;
// 	INT32S 			TmpDriveAd;

// 	INT16U			KsiVBus;
// 	INT16U			VBus;
// 	
// 	_iq	iqKsiVBus;
// 	_iq iqVBus;
// 	_iq iqDiffBus;						//KsiBus��VBus��ѹ��
// 	_iq iqMaxDiffBus;					
// 	_iq iqDcRateVoltage;		  //��ƿ��ѹ�ȼ�
// 	_iq iqDcMaxVoltage;				/* ��ߵ�ѹ�������õ�ѹϵͳ������ */
// 	_iq iqDcCutVoltage;				/* ������ѹ�����ڸõ�ѹϵͳ�������У��������Ť�� */
// 	_iq iqDcMinVoltage;				/* ��͵�ѹ�����ڸõ�ѹϵͳ��������ֹͣ���� */
// 	_iq iqDcBroVoltage;				/* ������ѹ�����ڸõ�ѹ��ϵͳ��λ���ر�һ����� */
// 	_iq iqDcEspLowVoltage;
// 	_iq iqCtlRate1HCurrent;		//������1Сʱ�����ȼ�
// 	_iq iqCtlRate2MCurrent;		//������2���ӵ����ȼ�
// 	_iq iqOut5V;
// 	_iq iqOut12V;
// 	_iq iqOut5VMaxVoltage;
// 	_iq iqOut5VMinVoltage;
// 	_iq iqOut12VMaxVoltage;
// 	_iq iqOut12VMinVoltage;
// 	
// 	/* led */
// 	INT16S		LedCountHead;
// 	INT16S		LedCountRep;
// 	INT16S		LedCount;
// 	INT16S    LedBlinkPeriod;      //Led blink period ms
// 	
// 	/* ICAN��Դ�ڵ� */
// 	tICAN_Lift icanLift;
// 	tICAN_Steer icanSteer;
// 	tICAN_HMI icanHMI;
// 	
// } PLC_CTL;

/*******************************************************************************
* 1. ָʾ��
*******************************************************************************/
//״ָ̬ʾ��
#define 	LED_YELLOW_SET						(1 << 0)
#define 	LED_YELLOW_RESET					(1 << 1)
#define 	LED_YELLOW_BLINK					(1 << 2)
#define		LED_RED_SET								(1 << 8)
#define		LED_RED_RESET							(1 << 9)
#define		LED_RED_BLINK							(1 << 10)

void LedStateShow(INT16U ucState)
{
	//yellow
	if (ucState & LED_YELLOW_SET)
	{
		gPLCCtl.doDataOut[LED_Y] = 1;
	}
	else if (ucState & LED_YELLOW_RESET)
	{
		gPLCCtl.doDataOut[LED_Y] = 0;
	}
	else if (ucState & LED_YELLOW_BLINK)
	{
		gPLCCtl.doDataOut[LED_Y] ^= 1;
	}

	//red
	if (ucState & LED_RED_SET)
	{
		gPLCCtl.doDataOut[LED_R] = 1;
	}
	else if (ucState & LED_RED_RESET)
	{
		gPLCCtl.doDataOut[LED_R] = 0;
	}
	else if (ucState & LED_RED_BLINK)
	{
		gPLCCtl.doDataOut[LED_R] ^= 1;
	}
}
/*******************************************************************************
* Name: LEDInit
* Description: ָʾ����ʾ��ʼ��
* Input: 
* Output: 
*
* Author:
* Date:
* Revision:
*******************************************************************************/
void LEDInit(void)
{
	gPLCCtl.LedCountHead = 0;
	gPLCCtl.LedCountRep = 0;
	gPLCCtl.LedCount = 0;
	gPLCCtl.LedBlinkPeriod = 0;	
	LedStateShow(LED_YELLOW_RESET | LED_RED_RESET);
}

/*******************************************************************************
* Name: LEDProcess
* Description: ָʾ����ʾ
* Input: 
* Output: 
*
* Author:
* Date:
* Revision:
*******************************************************************************/
void LEDProcess(void)
{
	//gCRam.ErrCode=99; //23,1,9,10,11,15,19,20,99 
	if (gPLCCtl.LedBlinkPeriod >= LED_BLINK_PERIOD)
	{
		gPLCCtl.LedBlinkPeriod = 0;
		if (gPLCCtl.LedCount > 0)  //First, finish current process
		{
			if (gPLCCtl.LedCountHead <= 0) //Head over
			{
				if (gPLCCtl.LedCount >= 10) //digit 10
				{
					if (gPLCCtl.LedCountRep != 0)
					{
						LedStateShow(LED_YELLOW_BLINK | LED_RED_RESET);
						gPLCCtl.LedCountRep = 0;
						gPLCCtl.LedCount -= 10;
					}
					else
					{
						gPLCCtl.LedCountRep = 1;
						LedStateShow(LED_YELLOW_BLINK | LED_RED_RESET);
					}
				}
				else  //digit 1
				{
					if (gPLCCtl.LedCountRep != 0)
					{
						LedStateShow(LED_YELLOW_RESET | LED_RED_BLINK);
						gPLCCtl.LedCountRep = 0;
						gPLCCtl.LedCount -= 1;
					}
					else
					{
						gPLCCtl.LedCountRep = 1;
						LedStateShow(LED_YELLOW_RESET | LED_RED_BLINK);
					}
				}
			}
			else //Head delay
			{
				gPLCCtl.LedCountHead--;
			}
		}
		else if (gPLCCtl.ErrCode != 0) //Second,Err
		{
			gPLCCtl.LedCount = gPLCCtl.ErrCode;
			gPLCCtl.LedCountRep = 0;
			gPLCCtl.LedCountHead = 2;
			LedStateShow(LED_YELLOW_RESET | LED_RED_RESET);
		}
		else if (gPLCCtl.AlmCode != 0) //Third, Alm 
		{
			gPLCCtl.LedCount = gPLCCtl.AlmCode;
			gPLCCtl.LedCountRep = 0;
			gPLCCtl.LedCountHead = 2;
			LedStateShow(LED_YELLOW_RESET | LED_RED_RESET);
		}
		else //No Err & No Alm
		{
			gPLCCtl.LedCount = 0;
			gPLCCtl.LedCountRep = 0;
			gPLCCtl.LedCountHead = 2;
			LedStateShow(LED_YELLOW_SET | LED_RED_RESET);
		}
	}
	else
	{
		gPLCCtl.LedBlinkPeriod += PLC_PERIOD;
	}
}

