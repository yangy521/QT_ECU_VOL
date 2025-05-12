/*******************************************************************************
* Filename: PLC.c                                             	 		   	   *
* Description:											   			 		   												 *
* Author:                                                           		   *
* Date:     														 		   													   *
* Revision:															 		   
*******************************************************************************/

#include  "PLC.h"
#include  "PLCHardware.h"
#include	"CommonRam.h"
//#include	"Speed.h"
#include	"FDB.h"
#include	"iCAN.h"
#include "iCANPlc.h"
#include "iCANLogic.h"
#include	"iTimer.h"
#include	"Queue.h"
#include	"PARA.h"
#include	"canSTM32F4.h"
#include  "Current.h"

#include	"USER_SDLG_TDM.h"
#include	"USER_XUGONG_TDM.h"
#include	"USER_JIANGHE_TDM.h"
#include	"USER_XINGBANG_TDM.h"
#include	"USER_LIUGONG_TDM.h"
//#include  	"USER_NBRY_DDTBC.h"
#include  	"USER_NBRY_DJG2T.h"
#include  	"USER_EP_DDTBC.h"
#include  "USER_NUOLI_DDTBC.h"

//函数指针
void (*PlcLogicFunc)(void);

void PLCErrorClr(INT16U ErrorNo)
{
	if ( gCRam.ErrCode == ErrorNo)
		gCRam.ErrCode = 0;
	if ( gCRam.AlmCode == ErrorNo)
		gCRam.AlmCode = 0;
}
void PLCErrorTips(INT16U ErrorNo)
{
	INT16U i16U;

	/* push into error trace fifo */
	for(i16U = 9; i16U > 0; i16U --)
	{
		gCRam.SvPa.ErrorTrace[i16U] = gCRam.SvPa.ErrorTrace[i16U - 1];
	}
	gCRam.SvPa.ErrorTrace[0] = ErrorNo;

	if(ErrorNo <30)
	{
		gCRam.ErrCode = ErrorNo;
	}
	
	if(ErrorNo >30)
	{
		gCRam.AlmCode = ErrorNo;
	}	
//	gCRam.ErrCode = 0;
}

void PLCErrorMonitor(void)
{
	INT32S ErrorNO, AlarmNo;
	ErrorNO = 0;
	/******************************* 故障监测**********************************/
	/* error */
	/* No.1~12: 系统内核错误 */
	
		/* No.13: 母线电容充电 */
	if(SL_CHK(PLC_CAP_CHARGE_ERR))
	{
		if (ErrorNO == 0)
			ErrorNO = 13;
//		SL_CLR(PLC_CAP_CHARGE_ERR);
	}
	
	/* No.14: 主接触器连接故障 */
	if(SL_CHK(PLC_MAIN_CONNECT_ERR))
	{
		if (ErrorNO == 0)
			ErrorNO = 14;
//		SL_CLR(PLC_DRIVE1_ERR);
	}
	
	/* No.15: 电磁制动连接故障 */
	if(SL_CHK(PLC_DRIVE2_CONNECT_ERR))
	{
		if (ErrorNO == 0)
			ErrorNO = 15;
//		SL_CLR(PLC_DRIVE2_ERR);
	}
	
	/* No.16: 电池电压严重过低（停机） */
	if(SL_CHK(PLC_MIN_VOLTAGE_ERR))
	{
		if (ErrorNO == 0)
			ErrorNO = 16;
//		SL_CLR(PLC_MIN_VOLTAGE_ERR);
	}
	
	/* No.17: 电池电压过高 */
	if(SL_CHK(PLC_MAX_VOLTAGE_ERR))
	{
		if (ErrorNO == 0)
			ErrorNO = 17;
//		SL_CLR(PLC_MAX_VOLTAGE_ERR);
	}
	
	/* No.18: 功率板严重过温（95度停机） */
	if(SL_CHK(PLC_POWER_MAX_TMP_ERR))
	{
		if (ErrorNO == 0)
			ErrorNO = 18;
//		SL_CLR(PLC_POWER_MAX_TMP_ERR);
	}
	
	/* No.19: 电机严重高温 */
	if(SL_CHK(PLC_MOTOR_MAX_TMP_ERR))
	{
		if (ErrorNO == 0)
			ErrorNO = 19;
//		SL_CLR(PLC_MOTOR_MAX_TMP_ERR);
	}
	
	/* No.20: 电位计短路 或 过压*/
	if((SL_CHK(PLC_POT_SHORTCIRCUITS_ERR))
		||(SL_CHK(PLC_THROTTLE_OVER_ERR))
		||(SL_CHK(PLC_BRAKE_OVER_ERR)))
	{
		if (ErrorNO == 0)
			ErrorNO = 20;
//		SL_CLR(PLC_MOTOR_MAX_TMP_ERR);
	}
	
	/* No.21: 主接触器触点熔接 */
	if(SL_CHK(PLC_MAIN_WELDED_ERR))
	{
		if (ErrorNO == 0)
			ErrorNO = 21;
//		SL_CLR(PLC_MAIN_WELDED_ERR);
	}
	
	/* No.22: 输出5V故障 */
	if(SL_CHK(PLC_OUT_5V_ERR))
	{
		if (ErrorNO == 0)
			ErrorNO = 22;
//		SL_CLR(PLC_OUT_5V_ERR);
	}
	
	/* No.23: MACID检测失败 */
	if(SL_CHK(ICAN_MACIDCHECK_ERR))
	{
		if (ErrorNO == 0)
			ErrorNO = 23;
//		SL_CLR(ICAN_MACIDCHECK_ERR);
	}
	
	/* No.24: 主接触器驱动故障 */
	if(SL_CHK(PLC_MAIN_DRIVE_ERR))
	{
		if (ErrorNO == 0)
			ErrorNO = 24;
//		SL_CLR(PLC_MAIN_DRIVE_ERR);
	}
	/* No.25: 功率模块故障 */
	if(SL_CHK(SL_ESP_VOLTAGE_ERR))
	{
		if (ErrorNO == 0)
			ErrorNO = 25;
//		SL_CLR(SL_ESP_VOLTAGE_ERR);
	}
	/* No.26: Can 通信错误（节点丢失，连接超时，断线） */
	if(SL_CHK(ICAN_CONNECT_ERR))
	{
		if (ErrorNO == 0)
			ErrorNO = 26;
//		SL_CLR(SL_ESP_VOLTAGE_ERR);
	}
	/* No.28: 电机开路 */
	if(SL_CHK(PLC_MOTOR_OPEN_ERR))
	{
		if (ErrorNO == 0)
			ErrorNO = 28;
//		SL_CLR(PLC_MOTOR_OPEN_ERR);
	}	
	/* No.29: 功率板电阻异常 */
	if(SL_CHK(PLC_POWER_THRES_ERR))
	{
		if (ErrorNO == 0)
			ErrorNO = 29;
//		SL_CLR(PLC_POWER_THRES_ERR);
	}
		//For Plc error, stop the motor first, then error act.
	if (ErrorNO != 0)
	{		
		if(netTimer[Timer_ErrorDelay].bIsStart==false)
		{
			SetNetTimer(Timer_ErrorDelay,5000);
			SL_SET(PLC_STOP_MOVE);
		}
		if(netTimer[Timer_ErrorDelay].bIsOvertime==true)
		{
			PLCErrorTips(ErrorNO);
		}
	}
	else
	{
		KillNetTimer(Timer_ErrorDelay);
	}
	/* alarm */
	AlarmNo = 0;
	/* No.31: 电池电压轻度过低（性能消减）*/
	if(SL_CHK(PLC_CUT_VOLTAGE_ERR))
	{
		if (AlarmNo == 0)
			AlarmNo = 31;
//		SL_CLR(PLC_CUT_VOLTAGE_ERR);
	}
	
	/* No.32: 功率板轻度过温（85度性能消减） */
	if(SL_CHK(PLC_POWER_CUT_TMP_ERR))
	{
		if (AlarmNo == 0)
			AlarmNo = 32;
//		SL_CLR(PLC_POWER_CUT_TMP_ERR);
	}
	
	/* No.33: 功率板低温（-25度） */
	if(SL_CHK(PLC_POWER_MIN_TMP_ERR))
	{
		if (AlarmNo == 0)
			AlarmNo = 33;
//		SL_CLR(PLC_POWER_MIN_TMP_ERR);
	}
	
	/* No.34: 电机轻度高温 */
	if(SL_CHK(PLC_MOTOR_CUT_TMP_ERR))
	{
		if (AlarmNo == 0)
			AlarmNo = 34;
//		SL_CLR(PLC_MOTOR_CUT_TMP_ERR);
	}
	
	/* No.35: 输出12V故障 */
	if(SL_CHK(PLC_OUT_12V_ERR))
	{
		if (AlarmNo == 0)
			AlarmNo = 35;
//		SL_CLR(PLC_OUT_12V_ERR);
	}
	
	/* No.36: 驱动3 */
	if(SL_CHK(PLC_DRIVE3_CONNECT_ERR))
	{
		if (AlarmNo == 0)
			AlarmNo = 36;
//		SL_CLR(PLC_DRIVE2_ERR);
	}
	
	/* No.37: 驱动4 */
	if(SL_CHK(PLC_DRIVE4_CONNECT_ERR))
	{
		if (AlarmNo == 0)
			AlarmNo = 37;
//		SL_CLR(PLC_DRIVE2_ERR);
	}
	
	/* No.38:  EEPROM读写参数错误 */
	if(SL_CHK(PLC_EEPROM_RW_ERR))
	{
		if (AlarmNo == 0)
			AlarmNo = 38;
//		SL_CLR(PLC_EEPROM_RW_ERR);
	}
	
	/* No.39: 参数编号错误 */
	if(SL_CHK(PLC_PARA_OV_INDEX_ERR))
	{
		PLCErrorTips(39);
//		SL_CLR(PLC_PARA_OV_INDEX_ERR);
	}
	
	/* No.39: 参数超限错误 */
	if(SL_CHK(PLC_PARA_OV_LIMIT_ERR))
	{
		if (AlarmNo == 0)
			AlarmNo = 39;
//		SL_CLR(PLC_PARA_OV_LIMIT_ERR);
	}
	/* No.40: 上电io异常 */
	if(SL_CHK(PLC_IOLOGIC_ERR))
	{
		if (AlarmNo == 0)
			AlarmNo = 40;
//		SL_CLR(PLC_IOLOGIC_ERR);
	}
	else
	{
		PLCErrorClr(40);
	}
	/* No.41: 电量低于20% */
	if(SL_CHK(PLC_BAT_LOWALM))
	{
		if (AlarmNo == 0)
			AlarmNo = 41;
	}
	/* No.42: 电量低于10% */
	if(SL_CHK(PLC_BAT_PROTECTALM))
	{
		if (AlarmNo == 0)
			AlarmNo = 42;
	}
	if (AlarmNo != 0)
		PLCErrorTips(AlarmNo);
}

/*******************************************************************************
* Name: InitPLCCtl
* Description: PLC初始化
* Input: 
* Output: 
*
* Author: 
* Date: 
* Revision:
*******************************************************************************/
void User_Null(void)
{
	//error
}
void InitPLCCtl(void)
{
	//PLC硬件初始化
	InitPLCLogic();	

	//设定PLC运行逻辑函数指针
	PlcLogicFunc = NULL;

#if(USER_TYPE == USER_WXHF_GWJX_MOVE)
	PlcLogicFunc = WXHF_GWJX_Move;
#endif		
#if(USER_TYPE == USER_WXHF_GWJX_LIFT)
	PlcLogicFunc = WXHF_GWJX_Lift;
#endif

#if(USER_TYPE == USER_BJHL_PHZC04T_MOVE)
	PlcLogicFunc = BJHL_PHZC04T_Move;
#endif		
#if(USER_TYPE == USER_BJHL_PHZC04T_LIFT)
	PlcLogicFunc = BJHL_PHZC04T_Lift;
#endif

#if(USER_TYPE == USER_NBRY_SXCC_MOVE)
	PlcLogicFunc = NBRY_SXCC_Move;
#endif		
#if(USER_TYPE == USER_NBRY_SXCC_LIFT)
	PlcLogicFunc = NBRY_SXCC_Lift;
#endif

#if(USER_TYPE == USER_NBRY_QYCC15T_MOVE)
	PlcLogicFunc = NBRY_QYCC15T_Move;
#endif		
#if(USER_TYPE == USER_NBRY_QYCC15T_LIFT)
	PlcLogicFunc = NBRY_QYCC15T_Lift;
#endif

#if(USER_TYPE == USER_NBRY_DDTBC_MOVE)
	PlcLogicFunc = NBRY_DDTBC_Move;
#endif

#if ((USER_TYPE == USER_AHHL_PHZC20T_MOVE) || (USER_TYPE == USER_AHHL_PHZC25T_MOVE))
	PlcLogicFunc = AHHL_PHZC20T_Move;
#endif		
#if ((USER_TYPE == USER_AHHL_PHZC20T_LIFT) || (USER_TYPE == USER_AHHL_PHZC25T_LIFT))
	PlcLogicFunc = AHHL_PHZC20T_Lift;
#endif

#if(USER_TYPE == USER_NUOLI_DDTBC_MOVE)
	PlcLogicFunc = NUOLI_DDTBC_Move;
#endif
#if(USER_TYPE == USER_NUOLI_XJGLH10T_MOVE)
	PlcLogicFunc = NUOLI_XJGLH10T_Move;
#endif
#if(USER_TYPE == USER_NBRY_XJGXL12T_MOVE)
	PlcLogicFunc = NBRY_XJGXL12T_Move;
#endif
#if(USER_TYPE == USER_NBRY_DJG2T_MOVE)
	PlcLogicFunc = NBRY_DJG2T_Move;
#endif
#if(USER_TYPE == USER_HANGCHA_XJG12T_MOVE)
	PlcLogicFunc = HANGCHA_XJG12T_Move;
#endif
#if(USER_TYPE == USER_TONGLUSLD_XJG12T_MOVE)
	PlcLogicFunc = TONGLUSLD_XJG12T_Move;
#endif
#if(USER_TYPE == USER_SDLG_TDM)
	PlcLogicFunc = SDLG_TDM_Move;
#endif
#if(USER_TYPE == USER_XUGONG_TDM)
	PlcLogicFunc = SDLG_XUGONG_Move;
#endif
#if(USER_TYPE == USER_JIANGHE_TDM)
	PlcLogicFunc = TDM_JIANGHE_Move;
#endif
#if(USER_TYPE == USER_XINGBANG_TDM)
	PlcLogicFunc = TDM_XINGBANG_Move;
#endif
#if(USER_TYPE == USER_LIUGONG_TDM)
	PlcLogicFunc = TDM_LIUGONG_Move;
#endif

#if(USER_TYPE == USER_JSQIANLI_DDTBC_MOVE)
	PlcLogicFunc = JSQIANLI_DDTBC_Move;
#endif
#if(USER_TYPE == USER_WXWS_PHZC12T_MOVE)
	PlcLogicFunc = WXWS_PHZC12T_Move;
#endif
#if(USER_TYPE == USER_EP_PHZCC12T_MOVE)
	PlcLogicFunc = UserPlcLogicFunc;
#endif

#if (  (USER_TYPE == USER_XJTU_DDJCQS_MOVE) \
		|| (USER_TYPE == USER_XJTU_DDJCLD_MOVE) )
	PlcLogicFunc = XJTU_DDJC_Move;
#endif
//48V电转向
#if(USER_TYPE == USER_DVM4805_STEER)
	PlcLogicFunc = DVM4805_Steer;
#endif
//


//测试老化程序
#if(USER_TYPE == USER_TEST_PVM4806)
	PlcLogicFunc = TEST_PVM4806;
#endif

	//PLC config protect
	if (PlcLogicFunc == NULL)
		PlcLogicFunc = User_Null;
	
	//ICAN网络初始化
#if IS_LOGIC_MASTER(LOGIC_TYPE)
	ICANInitialize_Master();
#endif //IS_LOGIC_MASTER
		
#if IS_LOGIC_SLAVE(LOGIC_TYPE)
	ICANInitialize_Slave();
#endif //IS_LOGIC_SLAVE
}

/*******************************************************************************
* Name: PLCDi
* Description: 4 ms period. Run in ISR.
* Input: 
* Output: 
*
* Author:
* Date:
* Revision:
*******************************************************************************/
void PLCDi(void)
{
	//
	//ICAN输入
	//
#if IS_LOGIC_MASTER(LOGIC_TYPE)
	ICANLogicIn_Master();
#endif //IS_LOGIC_MASTER
		
#if IS_LOGIC_SLAVE(LOGIC_TYPE)
	ICANLogicIn_Slave();
#endif //IS_LOGIC_SLAVE
	//
	//本地IO输入数据
	//
	LocalDi();
}

/*******************************************************************************
* Name: PLCDo
* Description: 4 ms period. Run in ISR.
* Input: 
* Output: 
*
* Author:
* Date:
* Revision:
*******************************************************************************/
void PLCDo(void)
{	
//ICAN输出
#if IS_LOGIC_MASTER(LOGIC_TYPE)
	ICANLogicOut_Master();
#endif //IS_LOGIC_MASTER
		
#if IS_LOGIC_SLAVE(LOGIC_TYPE)
	ICANLogicOut_Slave();
#endif //IS_LOGIC_SLAVE

	LocalDo();//本地IO输出
	DoPWM();	//Do PWM输出
}

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
* Description: 指示灯显示初始化
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
* Description: 指示灯显示
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
		else if (gCRam.ErrCode != 0) //Second,Err
		{
			gPLCCtl.LedCount = gCRam.ErrCode;
			gPLCCtl.LedCountRep = 0;
			gPLCCtl.LedCountHead = 2;
			LedStateShow(LED_YELLOW_RESET | LED_RED_RESET);
		}
		else if (gCRam.AlmCode != 0) //Third, Alm 
		{
			gPLCCtl.LedCount = gCRam.AlmCode;
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

/*******************************************************************************
* Name: SetKernelState
* Description: set 
* bit 15       14       13       12       11       10       9        8 
*                      SPDFDB  SPDREF   IQFDB     IQREF  IDFDB    IDREF
* bit 7        6        5        4        3        2        1        0      
*            8051RDY  VBUSRDY  KSIRDY    SYSEN    EN_PWM   SVON     RDY
* Input: 
* Output: 
* 
* Author:
* Date:
* Revision:
*******************************************************************************/
//#define								(1 << 15)
//#define								(1 << 14)
#define			KERNEL_STATE_SPDFDB					(1 << 13)
#define			KERNEL_STATE_SPDREF					(1 << 12)
#define			KERNEL_STATE_IQFDB					(1 << 11)
#define			KERNEL_STATE_IQREF					(1 << 10)
#define			KERNEL_STATE_IDFDB					(1 << 9)
#define			KERNEL_STATE_IDREF					(1 << 8)
//#define								(1 << 7)
#define			KERNEL_STATE_C8051F					(1 << 6)
#define			KERNEL_STATE_VBUS						(1 << 5)
#define			KERNEL_STATE_KSI						(1 << 4)
#define			KERNEL_STATE_SYS_ENABLE			(1 << 3)
#define			KERNEL_STATE_EN_PWM					(1 << 2)
#define			KERNEL_STATE_SVON						(1 << 1)
#define			KERNEL_STATE_RDY						(1 << 0)
void SetKernelState(void)
{
	//Bit0
	if(SL_CHK(SL_TER_RDY))
		gCRam.KernelState |= KERNEL_STATE_RDY;
	else
		gCRam.KernelState &= ~KERNEL_STATE_RDY;
	//Bit1
	if(SL_CHK(SL_TER_SRVON))
		gCRam.KernelState |= KERNEL_STATE_SVON;
	else
		gCRam.KernelState &= ~KERNEL_STATE_SVON;
	//Bit2
	if(SL_CHK(SL_TER_EN_PWM))
		gCRam.KernelState |= KERNEL_STATE_EN_PWM;
	else
		gCRam.KernelState &= ~KERNEL_STATE_EN_PWM;
	//Bit3
	if(SL_CHK(PLC_SYS_ENABLE))
		gCRam.KernelState |= KERNEL_STATE_SYS_ENABLE;
	else
		gCRam.KernelState &= ~KERNEL_STATE_SYS_ENABLE;
	//Bit4
	if(SL_CHK(PLC_KSI_RDY))
		gCRam.KernelState |= KERNEL_STATE_KSI;
	else
		gCRam.KernelState &= ~KERNEL_STATE_KSI;
	//Bit5
	if(SL_CHK(PLC_VBUS_RDY))
		gCRam.KernelState |= KERNEL_STATE_VBUS;
	else
		gCRam.KernelState &= ~KERNEL_STATE_VBUS;
	//Bit6
	if(SL_CHK(PLC_C8051F_RDY))
		gCRam.KernelState |= KERNEL_STATE_C8051F;
	else
		gCRam.KernelState &= ~KERNEL_STATE_C8051F;
	//Bit7
	//Bit8
	if(gCurrentLoop.IdRef != 0)
		gCRam.KernelState |= KERNEL_STATE_IDREF;
	else
		gCRam.KernelState &= ~KERNEL_STATE_IDREF;
	//Bit9
	if(gCurrentLoop.IdFdb != 0)
		gCRam.KernelState |= KERNEL_STATE_IDFDB;
	else
		gCRam.KernelState &= ~KERNEL_STATE_IDFDB;
	//Bit10
	if(gCurrentLoop.IqRef != 0)
		gCRam.KernelState |= KERNEL_STATE_IQREF;
	else
		gCRam.KernelState &= ~KERNEL_STATE_IQREF;
	//Bit11
	if(gCurrentLoop.IqFdb != 0)
		gCRam.KernelState |= KERNEL_STATE_IQFDB;
	else
		gCRam.KernelState &= ~KERNEL_STATE_IQFDB;
	//Bit12
//	if(gSpeedLoop.SpdRef != 0)
//		gCRam.KernelState |= KERNEL_STATE_SPDREF;
//	else
//		gCRam.KernelState &= ~KERNEL_STATE_SPDREF;
//	//Bit13
//	if(gSpeedLoop.SpdFdb != 0)
//		gCRam.KernelState |= KERNEL_STATE_SPDFDB;
//	else
//		gCRam.KernelState &= ~KERNEL_STATE_SPDFDB;
	//Bit14
	//Bit15
}
/*******************************************************************************
* Name: PLCLogic
* Description: 4 ms period. Run in ISR.
* Input: 
* Output: 
*
* Author:
* Date:
* Revision:
*******************************************************************************/
void PLCLogic(void)
{
	unsigned char  PcProcType;		//PC process type
	unsigned char  HmiProcType;	//HMI process type
	
	gPLCCtl.PlcCount++;
	//主站CAN网络滞后启动500ms
#if IS_LOGIC_MASTER(LOGIC_TYPE)
	{	
		if(netTimer[Timer_ICAN_Delay].bIsOvertime==true)
		{
			if ((gPLCCtl.PlcCount & 3) == 0)
			{
				ICANDataInManage_Master();	//
				ICANNetManage_Master();
			}
			
			if (gPLCCtl.CanIdPowerOnDelay > 500)
			{
				if ((icanMaster.ucNetCfg & gCRam.SvPa.CanEn) != gCRam.SvPa.CanEn)
				{//Lost can id
					if ((icanMaster.ucNetCfg & gCRam.SvPa.CanLockEn) != gCRam.SvPa.CanLockEn)
					{//The lost id enable lock.
						if (gPLCCtl.CanIdLostDelay > 100)	
							SL_SET(ICAN_CONNECT_ERR);
						else
							gPLCCtl.CanIdLostDelay++;
					}
					else
						gPLCCtl.CanIdLostDelay = 0;
				}
			}
			else
			{
				gPLCCtl.CanIdPowerOnDelay++;
			}
		}
	}
#endif //IS_LOGIC_MASTER
#if IS_LOGIC_SLAVE(LOGIC_TYPE)
	{
		if ((gPLCCtl.PlcCount & 3) == 0)
		{
			ICANDataInManage_Slave();	//
			ICANNetManage_Slave();
		}
	}
#endif //IS_LOGIC_SLAVE
	PLCDi();
	PLCHardwareCheck();
	PLCStateMonitor();
  GetSysProcedure(&PcProcType,&HmiProcType);	//获取当前系统进程ID

	//仪表监控及配置
	switch(HmiProcType)
	{
	#if IS_LOGIC_MASTER(LOGIC_TYPE)
		case HMI_IDLE_MODE_STATESETTING:
			HMIIdle_Master();
			break;
		case HMI_MONITOR_MODE_STATESETTING:
			HMIMonitor_Master();
			break;
		case HMI_CONFIG_MODE_STATESETTING:
			HMIConfig_Master();
			SL_SET(PLC_STOP_MOVE);
			break;
		case HMI_TUNE_MODE_STATESETTING:
			ICANSysMatch();
			break;
		default :
			HMIPcIdle_Master();
			break;
	#endif //IS_LOGIC_MASTER
		
	#if IS_LOGIC_SLAVE(LOGIC_TYPE)
		case HMI_IDLE_MODE_STATESETTING:
			HMIIdle_Slave();
			break;
		case HMI_MONITOR_MODE_STATESETTING:
			HMIMonitor_Slave();
			break;
		case HMI_CONFIG_MODE_STATESETTING:
			HMIConfig_Slave();
			SL_SET(PLC_STOP_MOVE);
			break;
		case HMI_TUNE_MODE_STATESETTING:
			ICANSysMatch();
			break;
		default :
			HMIPcIdle_Slave();
			break;
	#endif //IS_LOGIC_SLAVE
	}
	
	//PC监控及配置
	switch(PcProcType)
	{
	#if IS_LOGIC_MASTER(LOGIC_TYPE)
		case PC_IDLE_MODE_STATESETTING:
			PCIdle_Master();
			break;
		case PC_MONITOR_MODE_STATESETTING:
			PCMonitor_Master();
			break;
		case PC_CONFIG_MODE_STATESETTING:
			PCConfig_Master();
			SL_SET(PLC_STOP_MOVE);
			break;
		case PC_TUNE_MODE_STATESETTING:
			ICANSysMatch();
			break;
		default :
			PCIdle_Master();
			break;
	#endif //IS_LOGIC_MASTER
		
	#if IS_LOGIC_SLAVE(LOGIC_TYPE)
		case PC_IDLE_MODE_STATESETTING:
			PCIdle_Slave();
			break;
		case PC_MONITOR_MODE_STATESETTING:
			PCMonitor_Slave();
			break;
		case PC_CONFIG_MODE_STATESETTING:
			PCConfig_Slave();
			SL_SET(PLC_STOP_MOVE);
			break;
		case PC_TUNE_MODE_STATESETTING:
			ICANSysMatch();
			break;
		default :
			PCIdle_Slave();
			break;
	#endif //IS_LOGIC_SLAVE
	}
	PlcLogicFunc();
	PLCErrorMonitor();
	LEDProcess();
	PLCDo();
	SetKernelState();
	
	#if IS_LOGIC_MASTER(LOGIC_TYPE)
	{	
		if(netTimer[Timer_ICAN_Delay].bIsOvertime==true)
		{
			if ((gPLCCtl.PlcCount & 3) == 0)
				ICANDataOutManage_Master();	//
		}
	}
	#endif //IS_LOGIC_MASTER
	#if IS_LOGIC_SLAVE(LOGIC_TYPE)
	{
		if ((gPLCCtl.PlcCount & 3) == 0)
			ICANDataOutManage_Slave();	//
	}
	#endif //IS_LOGIC_SLAVE
	
	CANFrmSend(&canFrmTxBuffer);	
}

/*******************************************************************************
* Name: PLCDataSwap
* Description: 
* Input: 
* Output: 
*
* Author:
* Date:
* Revision:
*******************************************************************************/
void PLCDataSwap(void)
{
	//
	// KERNEL -> PLC
	//
	
	//AD
#if ((CTLBOARD_TYPE ==_1226)||(CTLBOARD_TYPE ==_1226_GD32))
	//AD
	gPLCCtl.aiDataIn[AD_KsiVBus]=gCurrentSample.KsiVBus;						
	gPLCCtl.aiDataIn[AD_VBus]=gCurrentSample.VBus;							
	gPLCCtl.aiDataIn[AD_V5out]=gCurrentSample.V5out;		
	gPLCCtl.aiDataIn[AD_PowTmpHigh]=gCurrentSample.PowTmpHigh;				
	gPLCCtl.aiDataIn[AD_MotorTmp]=gCurrentSample.MotorTmp;
	gPLCCtl.aiDataIn[AD_ThrotPotHigh]=gCurrentSample.ThrotPotHigh;			
	gPLCCtl.aiDataIn[AD_ThrotPotWip]=gCurrentSample.ThrotPotWip;			
	
//	gPLCCtl.aiDataIn[AD_SpdPotHigh]=gCurrentSample.SpdPotHigh;				
	gPLCCtl.aiDataIn[AD_SpdPotWip]=gCurrentSample.SpdPotWip;					
//	gPLCCtl.aiDataIn[AD_PotLow]=gCurrentSample.PotLow;					
//	gPLCCtl.aiDataIn[AD_RelayR]=gCurrentSample.RelayR;

#endif //#if (CTLBOARD_TYPE ==_1226)

	//SigLamp
	gCRam.SigLamp[PLC_IN_FROM_KERNEL]=gCRam.SigLamp[KERNEL_OUT_TO_PLC];
	
	//	
	// PLC ->KERNEL
	//	
	
	//SigLamp
	gCRam.SigLamp[KERNEL_IN_FROM_PLC]=gCRam.SigLamp[PLC_OUT_TO_KERNEL];
	
	/* Set thermal koe*/
	if((gCRam.SvPa.ConBit1 & MotorTmpChkEnable) != 0)
		gCRam.SvPa.MotorPara.RotorTCoeK = _IQ(1.0) + gCRam.SvPa.MotorPara.RotorTCoe * (gPLCCtl.TmpMotor - 20);
	else
		gCRam.SvPa.MotorPara.RotorTCoeK = _IQ(1.0);

}

