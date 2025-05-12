/*******************************************************************************
* Filename: PLCLogic.c                                             	 		   	   *
* Description:											   			 		   												 *
* Author:                                                           		   *
* Date:     														 		   													   *
* Revision:															 		   
*******************************************************************************/

#include  "PLCHardware.h"
#include	"CommonRam.h"
#include	"Device.h"
//#include	"Speed.h"
#include	"FDB.h"
#include	"iCAN.h"
#include	"iTimer.h"
#include	"Queue.h"
#include	"Temprature.h"
#include	"PARA.h"

PLC_CTL			gPLCCtl;

//PLC����״̬���
void PLCStateMonitor(void)
{
	//
	//�����Լ�
	//
	
	//
	//���Ӵ��������۽Ӽ�⣬ĸ�ߵ��ݵ�ѹδ�ͷ�
	//
	if((!SL_CHK(PLC_VBUS_RDY))&&(SL_CHK(PLC_KSI_RDY)))
	{
		if(gPLCCtl.VBus>=gPLCCtl.KsiVBus) //
		{
			if(netTimer[Timer_WeldedCheck].bIsStart==false)
			{
				SetNetTimer(Timer_WeldedCheck,40);	//40ms
			}
			if(netTimer[Timer_WeldedCheck].bIsOvertime==true)
			{
//				SL_SET(PLC_MAIN_WELDED_ERR);
			}
		}
		else
		{
			if(netTimer[Timer_WeldedCheck].bIsStart==true)
			{
				KillNetTimer(Timer_WeldedCheck);
			}
		}
	}
	
		
	//
	//���Ӽ��
	//
	if(netTimer[Timer_SelfCheck].bIsOvertime==true)
	{
		if(gPLCCtl.diDataIn.ucIn[DRIVER1_R]==1)//Drive1���Ӽ��
		{
			SL_SET(PLC_DRIVE1_CONNECT);
			gPLCCtl.doConnect[0]=1;
		}
		else
		{
			if((gCRam.SvPa.DriverEn1 & MainEnable) != 0)
			{
				SL_SET(PLC_MAIN_CONNECT_ERR);
			}
		}
			
		if(gPLCCtl.diDataIn.ucIn[DRIVER2_R]==1)//Drive2���Ӽ��
		{
			SL_SET(PLC_DRIVE2_CONNECT);
			gPLCCtl.doConnect[1]=1;
		}
		else
		{
			if ((gCRam.SvPa.DriverEn1 & BrakeEnable) != 0)
			{
				SL_SET(PLC_DRIVE2_CONNECT_ERR);
			}
		}
			
		if(gPLCCtl.diDataIn.ucIn[DRIVER3_R]==1)//Drive3���Ӽ��
		{
			SL_SET(PLC_DRIVE3_CONNECT);
			gPLCCtl.doConnect[2]=1;
		}
		else
		{
			if ((gCRam.SvPa.DriverEn2 & Drive3Enable) != 0)
			{
				SL_SET(PLC_DRIVE3_CONNECT_ERR);
			}
		}
#if (((HARDVERSION1 & 0xff) >= 2) && ((HARDVERSION1 & 0xff) <= 3))
		if(gPLCCtl.diDataIn.ucIn[DRIVER4_R]==1)//Drive4���Ӽ��
		{
			SL_SET(PLC_DRIVE4_CONNECT);
			gPLCCtl.doConnect[3]=1;
		}
		else
		{
			if ((gCRam.SvPa.DriverEn2 & Drive4Enable) != 0)
			{
				SL_SET(PLC_DRIVE4_CONNECT_ERR);
			}
		}
#endif	
		
	#define PHASE_6VAD		315	//6.0V*(3K/(3K+68K)*4096/3.3V)		
	//
	//�õ����·���
	//		
		if(gCurrentSample.PhaseP > (PHASE_6VAD*2))//Pump���Ӽ��
		{
			SL_SET(PLC_DRIVE4_CONNECT);
		}
		else
		{
			if ((gCRam.SvPa.ConBit2 & PumpCheck) != 0)
			{
				SL_SET(PLC_MOTOR_OPEN_ERR);
			}
		}	
		
	//
	//���������·���
	//
		if((gCurrentSample.PhaseU < PHASE_6VAD) || (gCurrentSample.PhaseW < PHASE_6VAD))	//phase 6V Adc
		{
			if ((gCRam.SvPa.ConBit2 & MotorCheck) == 0)
			{
				SL_SET(PLC_MOTOR_OPEN_ERR);
			}		
		}		
		PwmEnablePwOn();
		
		KillNetTimer(Timer_SelfCheck);
	}

	//
	//��ص�ѹ��ȹ��ͼ�⣨Ť��������
	//
	if((gPLCCtl.iqKsiVBus<=gPLCCtl.iqDcCutVoltage)&&(gPLCCtl.iqKsiVBus>gPLCCtl.iqDcMinVoltage))
	{
		if(netTimer[Timer_VoltageCutCheck].bIsStart==false)
		{
			SetNetTimer(Timer_VoltageCutCheck,10000);	//10s
		}
		if(netTimer[Timer_VoltageCutCheck].bIsOvertime==true)
		{
			//if ((gCRam.SvPa.ConBit1 & MOVE_BATTERY_EN) != 0)
			//	SL_SET(PLC_CUT_VOLTAGE_ERR);
		}
	}
	else
	{
			KillNetTimer(Timer_VoltageCutCheck);
	}
	
	//
	//��ص�ѹ���ع��ͼ�⣨ͣ����
	//
	if(gPLCCtl.iqKsiVBus<=gPLCCtl.iqDcMinVoltage)
	{
		if(netTimer[Timer_VoltageMinCheck].bIsStart==false)
		{
			SetNetTimer(Timer_VoltageMinCheck,5000);	//5s
		}
		if(netTimer[Timer_VoltageMinCheck].bIsOvertime==true)
		{
			SL_SET(PLC_MIN_VOLTAGE_ERR);
			SL_CLR(PLC_KSI_RDY);
			SL_CLR(PLC_VBUS_RDY);
		}
	}
	else
	{
		KillNetTimer(Timer_VoltageMinCheck);
	}

	if(gPLCCtl.iqKsiVBus<=gPLCCtl.iqDcEspLowVoltage)
	{
		if(netTimer[Timer_VoltageEspCheck].bIsStart==false)
		{
			SetNetTimer(Timer_VoltageEspCheck,200);	//0.2s
		}
		if(netTimer[Timer_VoltageEspCheck].bIsOvertime==true)
		{
			SL_SET(SL_ESP_VOLTAGE_ERR);
			SL_CLR(PLC_KSI_RDY);
			SL_CLR(PLC_VBUS_RDY);
			SL_CLR(PLC_TER_RDY);
			gPLCCtl.doDataOut[DRIVER1]=1;	//�Ͽ����Ӵ���
		}
	}
	else
	{
		KillNetTimer(Timer_VoltageEspCheck);
	}
	
	//
	//��ص�ѹ���߼��
	//
	if(gPLCCtl.iqKsiVBus>=gPLCCtl.iqDcMaxVoltage)
	{
		if(netTimer[Timer_VoltageMaxCheck].bIsStart==false)
		{
			SetNetTimer(Timer_VoltageMaxCheck,100);	//100ms
		}
		if(netTimer[Timer_VoltageMaxCheck].bIsOvertime==true)
		{
			SL_SET(PLC_MAX_VOLTAGE_ERR);
		}
	}
	else
	{
		KillNetTimer(Timer_VoltageMaxCheck);
	}

	//���ʰ���ȹ��¼�⣨85������������
	if((gPLCCtl.TmpPower>=gCRam.SvPa.PowerCutTmp)&&(gPLCCtl.TmpPower<gCRam.SvPa.PowerMaxTmp))
	{
		if(netTimer[Timer_PowerTmpMaxCheck1].bIsStart==false)
		{
			SetNetTimer(Timer_PowerTmpMaxCheck1,5000);	//5s
		}
		if(netTimer[Timer_PowerTmpMaxCheck1].bIsOvertime==true)
		{
			SL_SET(PLC_POWER_CUT_TMP_ERR);
		}
	}
	else
	{
			KillNetTimer(Timer_PowerTmpMaxCheck1);
	}
	
	//���ʰ����ع��¼�⣨95��ͣ����
	if(gPLCCtl.TmpPower>=gCRam.SvPa.PowerMaxTmp)
	{
		if(netTimer[Timer_PowerTmpMaxCheck2].bIsStart==false)
		{
			SetNetTimer(Timer_PowerTmpMaxCheck2,3000);	//3s
		}
		if(netTimer[Timer_PowerTmpMaxCheck2].bIsOvertime==true)
		{
			SL_SET(PLC_POWER_MAX_TMP_ERR);
		}
	}
	else
	{
			KillNetTimer(Timer_PowerTmpMaxCheck2);
	}
	
	//���ʰ���¼�⣨-25������������
	if(gPLCCtl.TmpPower<gCRam.SvPa.PowerMinTmp)
	{
		if(netTimer[Timer_PowerTmpMinCheck].bIsStart==false)
		{
			SetNetTimer(Timer_PowerTmpMinCheck,3000);	//3s
		}
		if(netTimer[Timer_PowerTmpMinCheck].bIsOvertime==true)
		{
			SL_SET(PLC_POWER_MIN_TMP_ERR);
		}
	}
	else
	{
		KillNetTimer(Timer_PowerTmpMinCheck);
		SL_CLR(PLC_POWER_MIN_TMP_ERR);
	}
	
	if((gCRam.SvPa.ConBit1 & MotorTmpChkEnable) != 0)
	{
		//
		//�����ȸ��¼�⣨120������������
		//
		if((gPLCCtl.TmpMotor>=gCRam.SvPa.MotorCutTmp)&&(gPLCCtl.TmpMotor<gCRam.SvPa.MotorMaxTmp))
		{
			if(netTimer[Timer_MotorTmpMaxCheck1].bIsStart==false)
			{
				SetNetTimer(Timer_MotorTmpMaxCheck1,5000);	//5s
			}
			if(netTimer[Timer_MotorTmpMaxCheck1].bIsOvertime==true)
			{
				SL_SET(PLC_MOTOR_CUT_TMP_ERR);
			}
		}
		else
		{
				KillNetTimer(Timer_MotorTmpMaxCheck1);
		}
		
		//
		//������ظ��¼�⣨140��ͣ����
		//
		if(gPLCCtl.TmpMotor>=gCRam.SvPa.MotorMaxTmp)
		{
			if(netTimer[Timer_MotorTmpMaxCheck2].bIsStart==false)
			{
				SetNetTimer(Timer_MotorTmpMaxCheck2,3000);	//3s
			}
			if(netTimer[Timer_MotorTmpMaxCheck2].bIsOvertime==true)
			{
				SL_SET(PLC_MOTOR_MAX_TMP_ERR);
			}
		}
		else
		{
			KillNetTimer(Timer_MotorTmpMaxCheck2);
		}
	}
	
	//
	//���5V���
	//
	//min 4V; max 6V
	if((gPLCCtl.iqOut5V<gPLCCtl.iqOut5VMinVoltage)||(gPLCCtl.iqOut5V>gPLCCtl.iqOut5VMaxVoltage))
	{
		if(netTimer[Timer_OUT5V_Check].bIsStart==false)
		{
			SetNetTimer(Timer_OUT5V_Check,2000);	//2s
		}
		if(netTimer[Timer_OUT5V_Check].bIsOvertime==true)
		{
			SL_SET(PLC_OUT_5V_ERR);
		}
	}
	else
	{
		SL_CLR(PLC_OUT_5V_ERR);
		KillNetTimer(Timer_OUT5V_Check);
	}

	//
	//���12V���
	//
	//min 10V; max 15V
	//
	//����1��⣨���Ӵ�����
	//
	if((SL_CHK(PLC_DRIVE1_CONNECT))&&(gPLCCtl.doDataOut[DRIVER1]==0))	//�������
	{
		if(!SL_CHK(PLC_TER_RDY))
		{
			if(netTimer[Timer_Drive1Check].bIsStart==false)
			{
				SetNetTimer(Timer_Drive1Check,1000);	//1s
			}
			if(netTimer[Timer_Drive1Check].bIsOvertime==true)
			{
				SL_SET(PLC_MAIN_DRIVE_ERR);
				gPLCCtl.doDataOut[DRIVER1]=1;	//ֹͣ���
			}
		}
		else
		{
			KillNetTimer(Timer_Drive1Check);
			SL_CLR(PLC_MAIN_DRIVE_ERR);
		}
	}
	
	//
	//����1��⣨���Ӵ�����
	//
	if (SL_CHK(PLC_DRIVE1_CONNECT))
	{
		if (((gPLCCtl.PulseWidth[0] == DO_PWM_TIM_PERIOD) && (gPLCCtl.diDataIn.ucIn[DRIVER1_R] != 0))
			|| ((gPLCCtl.PulseWidth[0] == 0) && (gPLCCtl.diDataIn.ucIn[DRIVER1_R] == 0)))
		{
			if(netTimer[Timer_Drive1Check].bIsStart==false)
			{
				SetNetTimer(Timer_Drive1Check,DO_LOST_CHK_DELAY_TIME);
			}
			if(netTimer[Timer_Drive1Check].bIsOvertime==true)
			{
				if((gCRam.SvPa.DriverEn1 & MainEnable) != 0)
				{
					SL_SET(PLC_MAIN_CONNECT_ERR);
				}
				KillNetTimer(Timer_Drive1Check);
			}
		}
		else
		{
			if(netTimer[Timer_Drive1Check].bIsStart==true)
			{
				KillNetTimer(Timer_Drive1Check);
			}
		}
	}
	
	//
	//����2��⣨����ƶ���
	//
	if((SL_CHK(PLC_DRIVE2_CONNECT)) && (gPLCCtl.doDataOut[DRIVER_EN]==0))
	{
		if (((gPLCCtl.PulseWidth[1] == DO_PWM_TIM_PERIOD) && (gPLCCtl.diDataIn.ucIn[DRIVER2_R] != 0))
			|| ((gPLCCtl.PulseWidth[1] == 0) && (gPLCCtl.diDataIn.ucIn[DRIVER2_R] == 0)))
		{
			if(netTimer[Timer_Drive2Check].bIsStart==false)
			{
				SetNetTimer(Timer_Drive2Check,DO_LOST_CHK_DELAY_TIME);
			}
			if(netTimer[Timer_Drive2Check].bIsOvertime==true)
			{
				if ((gCRam.SvPa.DriverEn1 & BrakeEnable) != 0)
				{
					SL_SET(PLC_DRIVE2_CONNECT_ERR);
				}

				KillNetTimer(Timer_Drive2Check);
			}
		}
		else
		{
			if(netTimer[Timer_Drive2Check].bIsStart==true)
			{
				KillNetTimer(Timer_Drive2Check);
			}
		}
	}	
	
	//
	//����3���
	//
	if((SL_CHK(PLC_DRIVE3_CONNECT)) && (gPLCCtl.doDataOut[DRIVER_EN]==0))
	{
		if (((gPLCCtl.PulseWidth[2] == DO_PWM_TIM_PERIOD) && (gPLCCtl.diDataIn.ucIn[DRIVER3_R] != 0))
			|| ((gPLCCtl.PulseWidth[2] == 0) && (gPLCCtl.diDataIn.ucIn[DRIVER3_R] == 0)))
		{
			if(netTimer[Timer_Drive3Check].bIsStart==false)
			{
				SetNetTimer(Timer_Drive3Check,DO_LOST_CHK_DELAY_TIME);
			}
			if(netTimer[Timer_Drive3Check].bIsOvertime==true)
			{
				if ((gCRam.SvPa.DriverEn2 & Drive3Enable) != 0)
				{
					SL_SET(PLC_DRIVE3_CONNECT_ERR);
				}
				KillNetTimer(Timer_Drive3Check);
			}
		}
		else
		{
			if(netTimer[Timer_Drive3Check].bIsStart==true)
			{
				KillNetTimer(Timer_Drive3Check);
			}
		}
	}
	
#if(DRIVER_TYPE	==	_4825)
	//
	//����4���
	//
#if (((HARDVERSION1 & 0xff) >= 2) && ((HARDVERSION1 & 0xff) <= 3))
	if((SL_CHK(PLC_DRIVE4_CONNECT)) && (gPLCCtl.doDataOut[DRIVER_EN]==0))
	{
		if (((gPLCCtl.PulseWidth[3] == DO_PWM_TIM_PERIOD) && (gPLCCtl.diDataIn.ucIn[DRIVER4_R] != 0))
			|| ((gPLCCtl.PulseWidth[3] == 0) && (gPLCCtl.diDataIn.ucIn[DRIVER4_R] == 0)))
		{
			if(netTimer[Timer_Drive4Check].bIsStart==false)
			{
				SetNetTimer(Timer_Drive4Check,DO_LOST_CHK_DELAY_TIME);
			}
			if(netTimer[Timer_Drive4Check].bIsOvertime==true)
			{
				if ((gCRam.SvPa.DriverEn2 & Drive4Enable) != 0)
				{
					SL_SET(PLC_DRIVE4_CONNECT_ERR);
				}
				KillNetTimer(Timer_Drive4Check);
			}
		}
		else
		{
			if(netTimer[Timer_Drive4Check].bIsStart==true)
			{
				KillNetTimer(Timer_Drive4Check);
			}
		}
	}
#endif
#endif //#if(DRIVER_TYPE	==	_BLDC4860)
	
	//
	//�������������־
	//
	if (SL_CHK(PLC_POWER_CUT_TMP_ERR)
	  ||SL_CHK(PLC_POWER_MIN_TMP_ERR)
	  ||SL_CHK(PLC_MOTOR_CUT_TMP_ERR))
	{
		SL_SET(PLC_TER_CURRENT_CUT);
	}
	else
	{
		SL_CLR(PLC_TER_CURRENT_CUT);
	}
			
	//
	//���������
	//
	
	//
	//�������ȱ�ࡢ���߷������
	//
	
	//
	//���������ߴ���
	//
	
	//
	//�����������
	//
	{
		static INT16S Driver1ShutCnt = 0;
		static INT16S Driver2ShutCnt = 0;
		static INT16S Driver3ShutCnt = 0;
		static INT16S Driver4ShutCnt = 0;

		if(  (gPLCCtl.diDataIn.ucInNew[DO1_SHUT] != 0)
			|| (gPLCCtl.diDataIn.ucInNew[DO2_SHUT] != 0)
			|| (gPLCCtl.diDataIn.ucInNew[DO3_SHUT] != 0)
			|| (gPLCCtl.diDataIn.ucInNew[DO4_SHUT] != 0)
			)
			{
				DO_RESET_OFF();
			}
					
			if(gPLCCtl.diDataIn.ucInNew[DO1_SHUT] != 0)
			{
				Driver1ShutCnt++;
				if (Driver1ShutCnt >= (20/PLC_PERIOD))
				{
					if (((gCRam.SvPa.DriverEn1 & MainEnable) != 0)
							|| ((gCRam.SvPa.DriverEn2 & Drive1Enable) != 0)
							)
						{
							SL_SET(PLC_MAIN_CONNECT_ERR);
						}
				}
			}
			else
			{
				if (Driver1ShutCnt > 0)
					Driver1ShutCnt--;
			}
					
			if(gPLCCtl.diDataIn.ucInNew[DO2_SHUT] != 0)
			{
				Driver2ShutCnt++;
				if (Driver2ShutCnt >= (20/PLC_PERIOD))
				{
					if (((gCRam.SvPa.DriverEn1 & BrakeEnable) != 0)
					|| ((gCRam.SvPa.DriverEn2 & Drive2Enable) != 0)
						 )
						{
							SL_SET(PLC_DRIVE2_CONNECT_ERR);
						}
				}
			}
			else
			{
				if (Driver2ShutCnt > 0)
					Driver2ShutCnt--;
			}
			
			if(gPLCCtl.diDataIn.ucInNew[DO3_SHUT] != 0)
			{
					Driver3ShutCnt++;
					if (Driver3ShutCnt >= (20/PLC_PERIOD))
					{
						if ((gCRam.SvPa.DriverEn2 & Drive3Enable) != 0)
						{
							SL_SET(PLC_DRIVE3_CONNECT_ERR);
						}
					}
			}
			else
			{
				if (Driver3ShutCnt > 0)
					Driver3ShutCnt--;
			}
			
			if(gPLCCtl.diDataIn.ucInNew[DO4_SHUT] != 0)
			{
				Driver4ShutCnt++;
				if (Driver4ShutCnt >= (20/PLC_PERIOD))
				{
					if((gCRam.SvPa.DriverEn2 & Drive4Enable) != 0)
					{
						SL_SET(PLC_DRIVE4_CONNECT_ERR);
					}
				}
			}
			else
			{
				if (Driver4ShutCnt > 0)
					Driver4ShutCnt--;
			}
			
			DO_RESET_ON();                
	}

}



//PLCӲ��״̬���
void PLCHardwareCheck(void)
{	
	//
	//״̬ADֵ���
	//
	
	//��ѹ����ֵ����
	gPLCCtl.iqKsiVBus=_IQ12toIQ(gPLCCtl.aiDataIn[AD_KsiVBus]);
	gPLCCtl.iqVBus=_IQ12toIQ(gPLCCtl.aiDataIn[AD_VBus]);
	gPLCCtl.VBus=_IQrmpy(gPLCCtl.iqVBus,(10.0*STD_VBUS));			//��ǰĸ�ߵ�ѹ
	gPLCCtl.KsiVBus=_IQrmpy(gPLCCtl.iqKsiVBus,(10.0*STD_VBUS));	//KSI��ѹ
	gPLCCtl.iqOut5V=_IQ12toIQ(gPLCCtl.aiDataIn[AD_V5out]);
	//�¶Ȳ���ֵ����
	TmpProcess();
	
	//
	//KSI��ѹ���
	//
	if(!SL_CHK(PLC_KSI_RDY))
	{
		if((gPLCCtl.iqKsiVBus>=gPLCCtl.iqDcMinVoltage)&&(gPLCCtl.iqKsiVBus<=gPLCCtl.iqDcMaxVoltage))
		{
			if(netTimer[Timer_KSICheck].bIsStart==false)
			{
				SetNetTimer(Timer_KSICheck,40);
			}
			if(netTimer[Timer_KSICheck].bIsOvertime==true)
			{
				SL_SET(PLC_KSI_RDY);
				KillNetTimer(Timer_KSICheck);
			}
		}
		else
		{
			KillNetTimer(Timer_KSICheck);
		}
	}
	
	//
	//KSI��ѹ�쳣���
	//
	if(SL_CHK(PLC_KSI_RDY))
	{
		if(gPLCCtl.iqKsiVBus<gPLCCtl.iqDcMinVoltage)
		{
			if(netTimer[Timer_KSIAbnormalCheck].bIsStart==false)
			{
				SetNetTimer(Timer_KSIAbnormalCheck,5000);
			}
			if(netTimer[Timer_KSIAbnormalCheck].bIsOvertime==true)
			{
				KillNetTimer(Timer_KSIAbnormalCheck);
				
				SL_CLR(PLC_KSI_RDY);
				SL_CLR(PLC_VBUS_RDY);
			}
		}
		else
		{
			KillNetTimer(Timer_KSIAbnormalCheck);
		}
	}
	
	//
	//��Ƭ�������źŴ���
	//
#if ((CTLBOARD_TYPE ==_1236) || (CTLBOARD_TYPE ==_1246))
	GPIO_ToggleBits(GPIOD,GPIO_Pin_3);	//��Ƭ��������ת�ź�(WATCHDOG)
	//���յ�Ƭ��Ready�ź�
	if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_9)!=0)
	{
		SL_CLR(PLC_C8051F_RDY);
	}
	else
	{
		SL_SET(PLC_C8051F_RDY);
	}
#endif  //#if (CTLBOARD_TYPE ==_1236)

#if (CTLBOARD_TYPE ==_1232)
	GPIO_ToggleBits(GPIOG,GPIO_Pin_9);	//��Ƭ��������ת�ź�(WATCHDOG)
	//���յ�Ƭ��Ready�ź�
	if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6)!=0)
	{
		SL_CLR(PLC_C8051F_RDY);
	}
	else
	{
		SL_SET(PLC_C8051F_RDY);
	}
#endif	//#if (CTLBOARD_TYPE ==_1232)

#if (CTLBOARD_TYPE ==_1226)
//	GPIO_ToggleBits(GPIOD,GPIO_Pin_3);	//��Ƭ��������ת�ź�(WATCHDOG)
//	//���յ�Ƭ��Ready�ź�
//	if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_9)!=0)
//	{
//		SL_CLR(PLC_C8051F_RDY);
//	}
//	else
//	{
//		SL_SET(PLC_C8051F_RDY);
//	}
#endif //#if (CTLBOARD_TYPE ==_1226)
	
	//
	//ĸ�ߵ��ݵ�ѹ���
	//
	if(SL_CHK(PLC_KSI_RDY))
	{
		if(!SL_CHK(PLC_VBUS_RDY))//&&(SL_CHK(PLC_C8051F_RDY)))
		{
			if(gPLCCtl.iqVBus>=gPLCCtl.iqDcMinVoltage)
			{
				if(netTimer[Timer_VBusCheck].bIsStart==false)
				{
					SetNetTimer(Timer_VBusCheck,500);
				}
				if(netTimer[Timer_VBusCheck].bIsOvertime==true)
				{
					SL_SET(PLC_VBUS_RDY);
					KillNetTimer(Timer_VBusCheck);
				}
			}
			else
			{
				KillNetTimer(Timer_VBusCheck);
			}
		}
	}
	
	//
	//ĸ�ߵ��ݵ�ѹ�쳣���
	//
	if(SL_CHK(PLC_VBUS_RDY))
	{
		if(gPLCCtl.iqVBus<gPLCCtl.iqDcMinVoltage)
		{
			if(netTimer[Timer_VBusAbnormalCheck].bIsStart==false)
			{
				SetNetTimer(Timer_VBusAbnormalCheck,5000);
			}
			if(netTimer[Timer_VBusAbnormalCheck].bIsOvertime==true)
			{
				KillNetTimer(Timer_VBusAbnormalCheck);
				
				SL_CLR(PLC_VBUS_RDY);
			}		
		}	
		else
		{
			KillNetTimer(Timer_VBusAbnormalCheck);
		}		
	}
	
	//
	//���Ӵ�����������
	//
	if(SL_CHK(PLC_KSI_RDY))
	{
		if ( //(!SL_CHK(PLC_MAIN_DRIVER_OFF))
					 (!SL_CHK(SL_OVER_CURRENT_ERR))
			  && (!SL_CHK(SL_ESP_VOLTAGE_ERR))
		   )
		{
			if(netTimer[Timer_Charge].bIsStart==false)
			{
				SetNetTimer(Timer_Charge,3000);	//������3s

				gPLCCtl.doDataOut[CHARGE]=1;	//��ʼ���
			}
			if(SL_CHK(PLC_VBUS_RDY))	//ĸ�ߵ�ѹ����
			{
				INT32S ChargeTimerCount;
				ChargeTimerCount = netTimer[Timer_Charge].ulTimerCount;

				if(ChargeTimerCount >= (FS/1000)*1200)//��С���ʱ��1.2s
				{
					if(SL_CHK(PLC_DRIVE1_CONNECT))
					{
						gPLCCtl.doDataOut[DRIVER1]=0;	//�������Ӵ���
					}
					if(netTimer[Timer_Charge].bIsOvertime==true)
					{
						gPLCCtl.doDataOut[CHARGE]=0;	//ֹͣ���
					}
				}
			}
			else 	//ĸ�ߵ�ѹ����
			{
				INT32S ChargeTimerCount;
				ChargeTimerCount = netTimer[Timer_Charge].ulTimerCount;

				if(netTimer[Timer_Charge].bIsOvertime == true)
				{ //���3s��δ�ﵽԤ����ѹ��Ԥ���ʧ��
					gPLCCtl.doDataOut[CHARGE]=0;	//ֹͣ���
					SL_SET(PLC_CAP_CHARGE_ERR);
				}else	if(ChargeTimerCount >= (FS/1000)*1200/2)
				{ //��·����
					if(gPLCCtl.iqVBus <= (gPLCCtl.iqDcMinVoltage >> 2))
					{
						gPLCCtl.doDataOut[CHARGE]=0;	//ֹͣ���
						SL_SET(PLC_CAP_CHARGE_ERR);
					}
				}
			}
		}
		else
		{
			KillNetTimer(Timer_Charge);
			gPLCCtl.doDataOut[DRIVER1]=1;	//�Ͽ����Ӵ���
			gPLCCtl.doDataOut[CHARGE]=0;	//ֹͣ���
		}
	}

	//
	//���ʰ��Դ�����ж�
	//
	if(!SL_CHK(PLC_TER_RDY))
	{
		if(SL_CHK(PLC_KSI_RDY)
			&& SL_CHK(PLC_VBUS_RDY) 
		  && (!SL_CHK(PLC_POWER_MAX_TMP_ERR))
		  && (gPLCCtl.doDataOut[DRIVER1]==0))
		{
			if(netTimer[Timer_TERSet].bIsStart==false)
			{
				SetNetTimer(Timer_TERSet,40);//40ms for relay act
			}
			if(netTimer[Timer_TERSet].bIsOvertime==true)
			{
				SL_SET(PLC_TER_RDY);
				KillNetTimer(Timer_TERSet);
			}
		}
		else
		{
			KillNetTimer(Timer_TERSet);
		}
	}
	if(SL_CHK(PLC_TER_RDY))
	{
		if(SL_CHK(PLC_KSI_RDY) 
			&& SL_CHK(PLC_VBUS_RDY) 
		  && (!SL_CHK(PLC_POWER_MAX_TMP_ERR))
		  && (gPLCCtl.doDataOut[DRIVER1]==0))
		{
				KillNetTimer(Timer_TERClr);
		}
		else
		{
			if(netTimer[Timer_TERClr].bIsStart==false)
			{
				SetNetTimer(Timer_TERClr,5000);
				SL_SET(PLC_STOP_MOVE);
			}
			if(netTimer[Timer_TERClr].bIsOvertime==true)
			{
				SL_CLR(PLC_TER_RDY);
				SL_CLR(PLC_STOP_MOVE);
				KillNetTimer(Timer_TERClr);
			}
		}
	}
	
//	//
//	//���ƶ����ж�
//	//	
//	if(SL_CHK(PLC_TER_RDY))
//	{
//		gPLCCtl.doDataOut[DRIVER2]=0;	//���ƶ���
//	}
//	else
//	{
//		gPLCCtl.doDataOut[DRIVER2]=1;	//�ر��ƶ���
//	}
}
void memzero(INT8U *pData, INT32S sizeOfByte)
{
	while (((INT32U)pData & 0x3) != 0)
	{
		if (sizeOfByte > 0)
		{
			*((INT8U*)pData) = 0;
			pData++;
			sizeOfByte--;
		}
		else
			break;
	}
	while (sizeOfByte >= 4)
	{
		*((INT32U*)pData) = 0;
		pData += 4;
		sizeOfByte -= 4;
	}
	while (sizeOfByte > 0)
	{
		*((INT8U*)pData) = 0;
		pData++;
		sizeOfByte--;
	}
}
/*******************************************************************************
* Name: InitPLCCtl
* Description: PLC��ʼ��
* Input: 
* Output: 
*
* Author: 
* Date: 
* Revision:
*******************************************************************************/
void InitPLCLogic(void)
{
	INT32S index;
	INT32U PotLowFlag;
	//NetTimer
	netTimerInit();
	gPLCCtl.PlcCount = 0;
	//DI
	memzero((INT8U*)&gPLCCtl.diDataIn,sizeof(gPLCCtl.diDataIn));
	//AI
	memzero((INT8U*)&gPLCCtl.aiDataIn,sizeof(gPLCCtl.aiDataIn));
	//DO
	gPLCCtl.doDataOut[DRIVER1]=1;	//����Ч
	gPLCCtl.doDataOut[DRIVER2]=1;	//����Ч
	gPLCCtl.doDataOut[DRIVER3]=1;	//����Ч
	gPLCCtl.doDataOut[DRIVER4]=1;	//����Ч
	gPLCCtl.doDataOut[LED_Y]=0;	//
	gPLCCtl.doDataOut[LED_R]=0;	//
	gPLCCtl.doDataOut[LOCK_OUT]=0;	//
	gPLCCtl.doDataOut[DRIVER_EN]=0;	//

	//
	LEDInit();
	TmpInit();
	//Connect��PulseWidth
	for(index=0;index<(sizeof(gPLCCtl.doConnect)/sizeof(gPLCCtl.doConnect[0]));index++)
	{
		gPLCCtl.doConnect[index]=0;
		gPLCCtl.PulseWidth[index]=0;
		gPLCCtl.PulseWidthDelay[index]=FULL_VOLTAGE_ACT_TIME;
	}

	PotLowFlag = 0;	
	//����̤������
	if(gCRam.SvPa.ThrottleType==THROTTLE_VOLTAGE5V) //��ѹ����
	{
#if ((CTLBOARD_TYPE ==_1226)||(CTLBOARD_TYPE ==_1226_GD32))
		THROTTLE1_ON();	//#THROTTLE: voltage input mode or three line res
#endif
	}
	else if(gCRam.SvPa.ThrottleType==THROTTLE_RESISTANCE_2WIRE) //2�ߵ�������
	{
#if ((CTLBOARD_TYPE ==_1226)||(CTLBOARD_TYPE ==_1226_GD32))
		THROTTLE1_ON();		//#THROTTLE:  1-two wire res mode
#endif
		PotLowFlag = 1;
	}
	else if(gCRam.SvPa.ThrottleType==THROTTLE_RESISTANCE_3WIRE) //3�ߵ�������
	{
#if ((CTLBOARD_TYPE ==_1226)||(CTLBOARD_TYPE ==_1226_GD32))
		THROTTLE1_ON();	//#THROTTLE: voltage input mode or three line res
#endif

		PotLowFlag = 1;
	}
	else if(gCRam.SvPa.ThrottleType==THROTTLE_BUS)
	{
#if ((CTLBOARD_TYPE ==_1226)||(CTLBOARD_TYPE ==_1226_GD32))
		THROTTLE1_OFF();	//#THROTTLE: voltage input mode or three line res
#endif
	}
	else if(gCRam.SvPa.ThrottleType==THROTTLE_VOLTAGE10V) //0~10��ѹ����
	{
#if ((CTLBOARD_TYPE ==_1226)||(CTLBOARD_TYPE ==_1226_GD32))
		THROTTLE1_OFF();	//#THROTTLE: voltage input mode or three line res
#endif
	}
	else
	{
#if ((CTLBOARD_TYPE ==_1226)||(CTLBOARD_TYPE ==_1226_GD32))
		THROTTLE1_OFF();	//#THROTTLE: voltage input mode or three line res
#endif
	}

	//ɲ��̤������
#if ((HARDVERSION1 & 0xff) <= 3)
	if(gCRam.SvPa.BrakePedalType==THROTTLE_VOLTAGE5V) //��ѹ����
	{
#if (CTLBOARD_TYPE ==_1226)
		THROTTLE2_OFF();	//#SPEED: voltage input mode or three line res
#endif
	}
	else if(gCRam.SvPa.BrakePedalType==THROTTLE_RESISTANCE_2WIRE) //2�ߵ�������
	{
#if (CTLBOARD_TYPE ==_1226)
		THROTTLE2_ON();		//#BRAKE:  1-two wire res mode
#endif
		
		PotLowFlag = 1;
	}
	else if(gCRam.SvPa.BrakePedalType==THROTTLE_RESISTANCE_3WIRE) //3�ߵ�������
	{
#if (CTLBOARD_TYPE ==_1226)
		THROTTLE2_OFF();	//#SPEED: voltage input mode or three line res
#endif
		
		PotLowFlag = 1;
	}
	else if(gCRam.SvPa.BrakePedalType==THROTTLE_BUS)
	{
#if (CTLBOARD_TYPE ==_1226)
		THROTTLE2_OFF();	//#SPEED: voltage input mode or three line res
#endif
	}
	else if(gCRam.SvPa.BrakePedalType==THROTTLE_VOLTAGE10V) //0~10��ѹ����
	{
#if (CTLBOARD_TYPE ==_1226)
		THROTTLE2_OFF();	//#SPEED: voltage input mode or three line res
#endif
	}
	else
	{
#if (CTLBOARD_TYPE ==_1226)
		THROTTLE2_OFF();	//#SPEED: voltage input mode or three line res
#endif
	}
#endif //((HARDVERSION1 & 0xff) <= 2)
	
	
	if (PotLowFlag != 0)
	{
#if (CTLBOARD_TYPE ==_1226)
	  //GPIO_SetBits(GPIOC,GPIO_Pin_13);	//#PotLow: 0-diable; 1-enable
#endif
	}
	else
	{
#if (CTLBOARD_TYPE ==_1226)
	  //GPIO_ResetBits(GPIOC,GPIO_Pin_13);	//#PotLow: 0-diable; 1-enable
#endif
	}

#if ((CTLBOARD_TYPE ==_1236) || (CTLBOARD_TYPE ==_1246))
	gPLCCtl.aoDataOut = 0;
#endif
	gPLCCtl.CmdDirection=0;	//ָ���0:��λ��ֹͣ����1������2������
	gPLCCtl.ucBatteryAct = 50;
	gPLCCtl.CurSpeedRate=0;	//��ǰ�ٶ�
	gPLCCtl.deadBandMin=gCRam.SvPa.ThrottleMinVoltage;
	gPLCCtl.deadBandMax=gCRam.SvPa.ThrottleMaxVoltage;
	gPLCCtl.throttleMid=(gPLCCtl.deadBandMin+gPLCCtl.deadBandMax)/2;
	gPLCCtl.rateLow=_IQ8(gCRam.SvPa.ThrottleMap)/(gPLCCtl.throttleMid-gPLCCtl.deadBandMin);
	gPLCCtl.rateHigh=_IQ8(100-gCRam.SvPa.ThrottleMap)/(gPLCCtl.deadBandMax-gPLCCtl.throttleMid);
	gPLCCtl.deadBandMinRvs=gCRam.SvPa.ThrottleMinVoltageRvs;
	gPLCCtl.deadBandMaxRvs=gCRam.SvPa.ThrottleMaxVoltageRvs;
	gPLCCtl.throttleMidRvs=(gPLCCtl.deadBandMinRvs+gPLCCtl.deadBandMaxRvs)/2;
	gPLCCtl.rateLowRvs=_IQ8(gCRam.SvPa.ThrottleMapRvs)/(gPLCCtl.throttleMidRvs-gPLCCtl.deadBandMinRvs);
	gPLCCtl.rateHighRvs=_IQ8(100-gCRam.SvPa.ThrottleMapRvs)/(gPLCCtl.deadBandMaxRvs-gPLCCtl.throttleMidRvs);
	gPLCCtl.throttleOutput=0;
	gPLCCtl.speedRate=gCRam.SvPa.SpeedRate1;
	gPLCCtl.throttleWip=0;
	gPLCCtl.AcMotorMaxSpdFAct = 0;
	gPLCCtl.TorqueCmdRaw = 0;		//
	gPLCCtl.TorqueCmdSum = 0;		//
	gPLCCtl.TorqueCmdAvg = 0;		//

	gPLCCtl.BrakePedalCmdDirection=0;	//ָ���0:��λ��ֹͣ����1������2������
	gPLCCtl.BrakePedalCurSpeedRate=0;	//��ǰ�ٶ�
	gPLCCtl.BrakePedaldeadBandMin=gCRam.SvPa.BrakePedalMinVoltage;
	gPLCCtl.BrakePedaldeadBandMax=gCRam.SvPa.BrakePedalMaxVoltage;
	gPLCCtl.BrakePedalMid=(gPLCCtl.BrakePedaldeadBandMin+gPLCCtl.BrakePedaldeadBandMax)/2;
	gPLCCtl.BrakePedalrateLow=_IQ8(gCRam.SvPa.BrakePedalMap)/(gPLCCtl.BrakePedalMid-gPLCCtl.BrakePedaldeadBandMin);
	gPLCCtl.BrakePedalrateHigh=_IQ8(100-gCRam.SvPa.BrakePedalMap)/(gPLCCtl.BrakePedaldeadBandMax-gPLCCtl.BrakePedalMid);
	gPLCCtl.BrakePedalOutput=0;
	gPLCCtl.BrakePedalspeedRate=100;
	gPLCCtl.BrakePedalWip=0;
	gPLCCtl.BrakePedalAcMotorMaxSpdFAct = 0;

	gPLCCtl.TmpMotor=TMP_DEFAULT;	//����¶�20��
	gPLCCtl.TmpPower=TMP_DEFAULT;	//���ʰ��¶�20��
	gPLCCtl.TmpDrive=TMP_DEFAULT;	//�������¶�20��
	gPLCCtl.VBus=0;			//��ǰĸ�ߵ�ѹ
	gPLCCtl.KsiVBus=0;	//KSI��ѹ	
	gPLCCtl.SigLampSvOff=1;
	gPLCCtl.SigLampSvOffOld=1;
	
	gPLCCtl.iqKsiVBus=0;
	gPLCCtl.iqVBus=0;
	gPLCCtl.iqDcRateVoltage=gCRam.SvPa.DcRateVoltage * _IQ(1.0/(10.0*STD_VBUS));		  //��ƿ��ѹ�ȼ�
	gPLCCtl.iqDcMaxVoltage=gCRam.SvPa.DcMaxVoltage * _IQ(1.0/(10.0*STD_VBUS));				/* ��ߵ�ѹ�������õ�ѹϵͳ������ */
	gPLCCtl.iqDcCutVoltage=gCRam.SvPa.DcCutVoltage * _IQ(1.0/(10.0*STD_VBUS));				/* ������ѹ�����ڸõ�ѹϵͳ�������У��������Ť�� */
	gPLCCtl.iqDcMinVoltage=gCRam.SvPa.DcMinVoltage * _IQ(1.0/(10.0*STD_VBUS));				/* ��͵�ѹ�����ڸõ�ѹϵͳ��������ֹͣ���� */
	gPLCCtl.iqDcBroVoltage=gCRam.SvPa.DcBroVoltage * _IQ(1.0/(10.0*STD_VBUS));				/* ������ѹ�����ڸõ�ѹ��ϵͳ��λ���ر�һ����� */
	gPLCCtl.iqDcEspLowVoltage = gCRam.SvPa.DcEspLowVoltage * _IQ(1.0/(10.0*STD_VBUS));
//	gPLCCtl.iqCtlRate1HCurrent=_IQ(gCRam.SvPa.CtlRate1HCurrent/10/STD_VBUS);		//������1Сʱ�����ȼ�
//	gPLCCtl.iqCtlRate2MCurrent=_IQ(gCRam.SvPa.CtlRate2MCurrent/10/STD_VBUS);		//������2���ӵ����ȼ�
	gPLCCtl.iqOut5V=0;
	gPLCCtl.iqOut5VMaxVoltage=_IQ(gCRam.SvPa.Out5VMaxVoltage/STD_5VOUT);
	gPLCCtl.iqOut5VMinVoltage=_IQ(gCRam.SvPa.Out5VMinVoltage/STD_5VOUT);
	gPLCCtl.iqOut12VMaxVoltage=_IQ(gCRam.SvPa.Out12VMaxVoltage/STD_12VOUT);
	gPLCCtl.iqOut12VMinVoltage=_IQ(gCRam.SvPa.Out12VMinVoltage/STD_12VOUT);

	gPLCCtl.ucBatteryDelay = 0;
	gPLCCtl.delaySvoff = 0;
	gPLCCtl.delaySvoffMove = 0;
	gPLCCtl.delaySvoffSpd = 0;
	gPLCCtl.logicLock = 0;
	gPLCCtl.logicLockDelay = 0;
	gPLCCtl.sysEnableDelay = 0;
	gPLCCtl.CanIdPowerOnDelay = 0;
	gPLCCtl.CanIdLostDelay = 0;
	memzero((INT8U*)&gPLCCtl.icanLift,sizeof(gPLCCtl.icanLift));
	memzero((INT8U*)&gPLCCtl.icanSteer,sizeof(gPLCCtl.icanSteer));
	memzero((INT8U*)&gPLCCtl.icanSmove,sizeof(gPLCCtl.icanSmove));
	memzero((INT8U*)&gPLCCtl.icanLogic,sizeof(gPLCCtl.icanLogic));
	memzero((INT8U*)&gPLCCtl.icanHMI,sizeof(gPLCCtl.icanHMI));
	gPLCCtl.icanHMI.ucBattery=50;
	memzero((INT8U*)&gPLCCtl.icanPc,sizeof(gPLCCtl.icanPc));
	
	//���������Լ춨ʱ��
	if(netTimer[Timer_SelfCheck].bIsStart==false)
	{
		SetNetTimer(Timer_SelfCheck,300);	//300ms
	}
	
	if(netTimer[Timer_ICAN_Delay].bIsStart==false)
	{
		SetNetTimer(Timer_ICAN_Delay,500);
	}
}

/*******************************************************************************
* Name: LocalDi
* Description: 
* Input: 
* Output: 
*
* Author: 
* Date: 
* Revision:
*******************************************************************************/
void LocalDi(void)
{
		INT8U index;
	
	if(READ_SW1()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[SWI1_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[SWI1_R]=0;
	}

	if(READ_SW2()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[SWI2_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[SWI2_R]=0;
	}

	if(READ_SW3()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[SWI3_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[SWI3_R]=0;
	}

	if(READ_SW4()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[SWI4_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[SWI4_R]=0;
	}

	if(READ_SW5()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[SWI5_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[SWI5_R]=0;
	}

	if(READ_SW6()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[SWI6_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[SWI6_R]=0;
	}

	if(READ_SW7()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[SWI7_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[SWI7_R]=0;
	}

	if(READ_SW8()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[SWI8_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[SWI8_R]=0;
	}
	if(READ_SW7()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[SWI7_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[SWI7_R]=0;
	}

	if(READ_SW8()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[SWI8_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[SWI8_R]=0;
	}	

	if(READ_SW9()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[SWI9_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[SWI9_R]=0;
	}

	if(READ_SW10()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[SWI10_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[SWI10_R]=0;
	}	

//	if(READ_SW11()!=0)
//	{
//		gPLCCtl.diDataIn.ucInNew[SWI11_R]=1;
//	}
//	else
//	{
//		gPLCCtl.diDataIn.ucInNew[SWI11_R]=0;
//	}	

//	if(READ_SW12()!=0)
//	{
//		gPLCCtl.diDataIn.ucInNew[SWI12_R]=1;
//	}
//	else
//	{
//		gPLCCtl.diDataIn.ucInNew[SWI12_R]=0;
//	}	

	if(READ_DRV1()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[DRIVER1_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[DRIVER1_R]=0;
	}	

	if(READ_DRV2()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[DRIVER2_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[DRIVER2_R]=0;
	}	

	if(READ_DRV3()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[DRIVER3_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[DRIVER3_R]=0;
	}	

	if(READ_DRV4()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[DRIVER4_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[DRIVER4_R]=0;
	}	

	if(READ_DRV5()!=0)
	{
		gPLCCtl.diDataIn.ucInNew[DRIVER5_R]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[DRIVER5_R]=0;
	}	

	if(DO1_SHUT_Read()==0)
	{
		gPLCCtl.diDataIn.ucInNew[DO1_SHUT]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[DO1_SHUT]=0;
	}	

	if(DO2_SHUT_Read()==0)
	{
		gPLCCtl.diDataIn.ucInNew[DO2_SHUT]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[DO2_SHUT]=0;
	}	

	if(DO3_SHUT_Read()==0)
	{
		gPLCCtl.diDataIn.ucInNew[DO3_SHUT]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[DO3_SHUT]=0;
	}	

	if(DO4_SHUT_Read()==0)
	{
		gPLCCtl.diDataIn.ucInNew[DO4_SHUT]=1;
	}
	else
	{
		gPLCCtl.diDataIn.ucInNew[DO4_SHUT]=0;
	}	

	for (index=0; index < (sizeof(gPLCCtl.diDataIn.ucIn)/sizeof(gPLCCtl.diDataIn.ucIn[0])); index++)
	{
		if (gPLCCtl.diDataIn.ucInNew[index] != gPLCCtl.diDataIn.ucIn[index])
		{
			if (gPLCCtl.diDataIn.ucInNew[index] == gPLCCtl.diDataIn.ucInOld[index])
			{
				if (gPLCCtl.diDataIn.ucTimer[index] == 0)
				{
					gPLCCtl.diDataIn.ucIn[index] = gPLCCtl.diDataIn.ucInNew[index];
				} 
				else 
				{
					gPLCCtl.diDataIn.ucTimer[index]--;
				}
			} 
			else 
			{
				gPLCCtl.diDataIn.ucInOld[index] = gPLCCtl.diDataIn.ucInNew[index];
				gPLCCtl.diDataIn.ucTimer[index] = DI_FILTER_CONSTANT;
			}
		} 
		else 
		{
			gPLCCtl.diDataIn.ucInOld[index] = gPLCCtl.diDataIn.ucIn[index];
		}
	}
	
	//
	//AI
	//
	
	//
	//PWM���ʹ��
	//
	if(SL_CHK(PLC_TER_EN_PWM))
	{
		PwmEnable();
	}
	else
	{
		PwmDisable();
	}
}

/*******************************************************************************
* Name: LocalDO
* Description: 
* Input: 
* Output: 
*
* Author: 
* Date: 
* Revision:
*******************************************************************************/
void LocalDo(void)
{
	static INT8U Toggle=0;
#if ((CTLBOARD_TYPE ==_1226)||(CTLBOARD_TYPE ==_1226_GD32))
	/*******************************************************************************
	* DO
	*Hareware V01
	*�ܽ�22  PA6  TMP_LOW  
	*�ܽ�44	 PA11	LED2;    �ܽ�45	 PA12	LED1;   	�ܽ�50	PA15	Driver_EN;
	*�ܽ�55  PB3	Relay;   �ܽ�58  PB6	Driver3;  �ܽ�59  PB7	Driver2;
  *�ܽ�4   PC15	Speed;   �ܽ�3   PC14 THROTTLE  �ܽ�2   PC13 POT_LOW
	*�ܽ�54  PD2  SPI_CS

	*Hareware V02
	*�ܽ�22  PB12  TMP_LOW  
	*�ܽ�44	 PA11	LED2;    �ܽ�45	 PA12	LED1;   	�ܽ�50	PA15	Driver_EN;
	*�ܽ�55  PB3	Relay;   �ܽ�58  PB6	Driver3;  �ܽ�59  PB7	Driver2;
  *�ܽ�4   PC15	Speed;   �ܽ�3   PC14 THROTTLE  �ܽ�2   PC13 POT_LOW
	*�ܽ�54  PD2  SPI_CS;  �ܽ�29  PB10	Driver4;
	*Hareware V03
	*�ܽ�22  PB12  TMP_LOW  
	*�ܽ�44	 PA11	LED2;    �ܽ�45	 PA12	LED1;   	�ܽ�50	PA15	Driver_EN;
	*�ܽ�55  PB3	Relay;   �ܽ�58  PB6	Driver3;  �ܽ�59  PB7	Driver2;
  *�ܽ�4   PC15	LED_OUT;   �ܽ�3   PC14 THROTTLE  �ܽ�2   PC13 POT_LOW
	*�ܽ�54  PD2  SPI_CS;  �ܽ�29  PB10	Driver4;
	*******************************************************************************/
		
	if(gPLCCtl.doDataOut[LED_Y])	
		LED_Y_ON();
	else 
		LED_Y_OFF();
		
	if(gPLCCtl.doDataOut[LED_R])	
		LED_R_ON();
	else 
		LED_R_OFF();
	
	if (gPLCCtl.doDataOut[DRIVER_EN] == 1)
	{
		DRIVEREN_ON();
	}
	else
	{
		DRIVEREN_OFF();
	}
	
	if (gPLCCtl.doDataOut[CHARGE] == 1)
	{
		Toggle ^= 0x01;
		if(Toggle)
			CHARGE_OFF();
		else
			CHARGE_ON();
	}
	else
	{
		CHARGE_ON();
	}
#if ((HARDVERSION1 & 0xff) == 3)
	if (gPLCCtl.doDataOut[LED_OUT] == 1)
	{
//		if ((gPLCCtl.PlcCount & 1) == 0)
//			GPIO_SetBits(GPIOC,GPIO_Pin_15);
//		else
//			GPIO_ResetBits(GPIOC,GPIO_Pin_15);
	}
	else
	{
		//GPIO_ResetBits(GPIOC,GPIO_Pin_15);
	}
#endif //#if ((HARDVERSION1 & 0xff) == 3)

//	if(gPLCCtl.doDataOut[LOCK_OUT]==1)
//	{
//	}
//	else
//	{
//	}
	
#endif //#if (CTLBOARD_TYPE ==_1226)

}

/*******************************************************************************
* Name: DoPWM
* Description: 
* Input: 
* Output: 
*
* Author: 
* Date: 
* Revision:
*******************************************************************************/
void DoPWM(void)
{
	_iq VcmpRatio;
	static INT16S DriverEnDelay = 0;
	
	VcmpRatio = _IQdiv(_IQ(STD_VOLTAGE/STD_VBUS),gPLCCtl.iqKsiVBus);
	//
	//Dirver1
	//
	if (gPLCCtl.doDataOut[DRIVER1]==0)	//�����Ч
	{
		gPLCCtl.doDataOut[LOCK_OUT] = 1;
	}
	else
	{
		gPLCCtl.doDataOut[LOCK_OUT] = 0;
	}
	if ((gPLCCtl.doDataOut[DRIVER1]==0) && (gPLCCtl.diDataIn.ucIn[DRIVER1_R]==0))	//�����Ч
	{ //����ɹ�
		if (DriverEnDelay > ((FULL_VOLTAGE_ACT_TIME-80)/T_MS_PLC_PERIOD)) //xxxms delay
			gPLCCtl.doDataOut[DRIVER_EN] = 0;
		else
			DriverEnDelay++;
	}
	else
	{
		DriverEnDelay = 0;
	}
	
	if((gPLCCtl.doDataOut[DRIVER1]==0)	//�����Ч
	   && ((gCRam.SvPa.DriverEn1 & MainEnable)!= 0))
	{
		if(gPLCCtl.PulseWidthDelay[0] >= 0)
		{				
			gPLCCtl.PulseWidth[0]=DO_PWM_TIM_PERIOD;	//
			gPLCCtl.PulseWidthDelay[0]--;
		}
		else
		{
			gPLCCtl.PulseWidth[0]=_IQmpy(gCRam.SvPa.MainHoldVoltage*(DO_PWM_TIM_PERIOD/100),VcmpRatio);
		}
	}
	else	//�����Ч
	{
		gPLCCtl.PulseWidth[0]=0;
		gPLCCtl.PulseWidthDelay[0] = FULL_VOLTAGE_ACT_TIME;
	}
	
	//
	//Dirver2
	//
	if(gPLCCtl.doDataOut[DRIVER2]==0)	//�����Ч
	{
		if(gPLCCtl.PulseWidthDelay[1] >= 0)
		{				
			gPLCCtl.PulseWidth[1]=DO_PWM_TIM_PERIOD;	//
			gPLCCtl.PulseWidthDelay[1]--;
		}
		else
		{
			gPLCCtl.PulseWidth[1]=_IQmpy(gCRam.SvPa.EmBrkHoldVoltage*(DO_PWM_TIM_PERIOD/100),VcmpRatio);
		}
	}
	else	//�����Ч
	{
		gPLCCtl.PulseWidth[1]=0;
		gPLCCtl.PulseWidthDelay[1] = FULL_VOLTAGE_ACT_TIME;
	}
	//
	//Dirver3
	//
	if(gPLCCtl.doDataOut[DRIVER3]==0)	//�����Ч
	{
		if(gPLCCtl.PulseWidthDelay[2] >= 0)
		{				
			gPLCCtl.PulseWidth[2]=DO_PWM_TIM_PERIOD;	//
			gPLCCtl.PulseWidthDelay[2]--;
		}
		else
		{
			gPLCCtl.PulseWidth[2]=_IQmpy(gCRam.SvPa.Drive3HoldVoltage*(DO_PWM_TIM_PERIOD/100),VcmpRatio);
		}
	}
	else	//�����Ч
	{
		gPLCCtl.PulseWidth[2]=0;
		gPLCCtl.PulseWidthDelay[2] = FULL_VOLTAGE_ACT_TIME;
	}
#if (((HARDVERSION1 & 0xff) >= 2) && ((HARDVERSION1 & 0xff) <= 3))
	//
	//Dirver4
	//
	if(gPLCCtl.doDataOut[DRIVER4]==0)	//�����Ч
	{
		if(gPLCCtl.PulseWidthDelay[3] >= 0)
		{				
			gPLCCtl.PulseWidth[3]=DO_PWM_TIM_PERIOD;	//
			gPLCCtl.PulseWidthDelay[3]--;
		}
		else
		{
			gPLCCtl.PulseWidth[3]=_IQmpy(gCRam.SvPa.Drive4HoldVoltage*(DO_PWM_TIM_PERIOD/100),VcmpRatio);
		}
	}
	else	//�����Ч
	{
		gPLCCtl.PulseWidth[3]=0;
		gPLCCtl.PulseWidthDelay[3] = FULL_VOLTAGE_ACT_TIME;
	}
#endif
	PWMDriver(gPLCCtl.PulseWidth);
}

