/*******************************************************************************
* Filename: UserEcuProc.c	                                             	   *
* Description: 逻辑层的源文件，包含对应的回调函数	   						   *
* Author: QExpand, Lilu                                                        *
* Date: 2023/05/30    														   *
* Revision:	 														 		   *
*******************************************************************************/
#include "User_4in1.h"
#include "ErrCode.h"
#include "IQmathLib.h"
#include "PropProc.h"
#include "CanRevProc.h"
#include "BeepProc.h"
#include "AlarmLamp.h"
#include "PressureSensor.h"
#include "string.h"
#include "Para.h"
#include "WdgProc.h"
#include "math.h"
#include "Log.h"
#include "AngleSensor.h"
#include "LocalAi.h"
#include "BatteryMeter.h"
#include "CanCom.h"
#include "HourCount.h"
#include "UserCanComm.h"
#include "LedProc.h"


#if (USER_TYPE == USER_SHANDONGKUNTYU_LOGIC)

#define TRAVEL_MODE		1					/*行进模式*/
#define	LIFT_MODE			0					/*升降模式*/

#define INITIAL_MODE	0x80	/*Initial State*/

#define SLOW_SPEED		1		/*龟速模式*/
#define	FULL_SPEED		0		/*全速模式*/

#define 	BATERRYMETER_TYPE BATERRYMETER_ENABLE

/* 客户类型 */
#define SHANDONG_KUNYU 1
#define CUSTOMER_TYPE SHANDONG_KUNYU

#if(CUSTOMER_TYPE == SHANDONG_KUNYU)

#define LIFTDOWN_WITH_UPPERPUMPOPEN//下降时打开起身阀

//下降阀只用J29 下降阀1

#endif

typedef union
{
	uint8_t u8data;
	struct
	{
		uint8_t b1PcuSpeed: 1;
		uint8_t b1SlowKey: 1;
		uint8_t b6Reserve: 6;
	};
}xSlowKey;

typedef union
{
	uint16_t u16data;
	struct
	{
		uint8_t b130PerMin: 1;			/*动作*/
		uint8_t b160PerMin: 1;			/*故障，不可取消报警*/
		uint8_t b1180PerMin: 1;			/*报警，可以取消的报警*/
		uint8_t b1240PerMin: 1;			/*下降*/
		uint8_t b4Reserve: 4;
		uint8_t u8Cnt;					/*用于计数，来进行两长一短*/
	};
}xSpeakerFun;

typedef union
{
	uint8_t u8Data;
	struct
	{
		uint8_t b1SenSorErr: 1;
		uint8_t	b1CaliReverse: 1;
		uint8_t b1CaliFailure: 1;
		uint8_t b1Per80Err: 1;
		uint8_t b1Per90Err: 1;
		uint8_t b1Per99Err: 1;
		uint8_t b1Per100Err: 1;
		uint8_t b1Reserve: 1;
	};
}xPresureErrInfo;

typedef union
{
	uint8_t u8data;
	struct
	{		
		uint8_t b1MoveTurnLeft:	1;//左转
		uint8_t b1MoveTurnRight:1;//右转
		uint8_t b1NormalMove: 	1;//行走使能
		uint8_t b1SlowMove: 		1;//慢速模式
		uint8_t	b1MoveAfterLift:1;/*举升后行走%*/
		
		uint8_t b1NormalLift: 	1;	//正常起升
		
		uint8_t b1NormalDown:		1;	//正常下降
		uint8_t b1Reserve:1;


	};
}xSpeedRate;

typedef union
{
	uint8_t u8data;
	struct
	{
		uint8_t b1MoveDisable:	1;//禁止行走
		uint8_t b1LiftDisabel:	1;//禁止起升
		uint8_t b1DownDisable:	1;//禁止下降
		uint8_t b1TurnDisable:	1;//禁止转向
		uint8_t	b1PCUDisable:		1;//禁止PCU
		uint8_t b1SpeedLimit:		1;//行走速度限制
		uint8_t b2Reserve:			2;//保留
	};

}xErrDisable;

typedef union
{
	uint8_t u8data;
	struct
	{
		uint8_t b1MoveDisable:	1;//禁止行走
		uint8_t b1LiftDisabel:	1;//禁止起升
		uint8_t b1DownDelay:		1;//防夹手下降
		uint8_t b1TurnDisable:	1;//禁止转向
		uint8_t	b1PCUDisable:		1;//禁止PCU
		uint8_t b1SpeedLimit:		1;//行走速度限制
		uint8_t b1DownDisable: 	1;//禁止下降
	
		uint8_t b1Reserve:			1;//保留
	};
}xSWILimit;

typedef union
{
	uint8_t u8data;
	struct
	{
		uint8_t b1Normal:1;		//正常状态
		uint8_t b1Arrive:1;		//第一次到达
		uint8_t	b1Relase:1;		//第一次到达后松开使能
		uint8_t	b1UnderLimit:1;			//下限位之下
		uint8_t b4Reserve:4;
	};
}xAntiPinchFlag;

const static xPdoParameter  PdoPara = 
{
	.TpdoPara = {
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 200},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 200},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 200},
		{.u8Flag = 0, .u8Type = 0x00, .u16Period = 200},
		{.u8Flag = 1, .u8Type = 0xFF, .u16Period = 200},
	},
	.RpdoPara = {
		{.b1Flag = 0, .b11CanRevId = 0x000},
		{.b1Flag = 0, .b11CanRevId = 0x000},
		{.b1Flag = 0, .b11CanRevId = 0x000},
		{.b1Flag = 0, .b11CanRevId = 0x000},
		{.b1Flag = 0, .b11CanRevId = 0x000},
		{.b1Flag = 0, .b11CanRevId = 0x000},
	},
};

/*PCU*/
static uint8_t u8PcuMode;  //初始、行走、起升三种模式
static xSlowKey sgSlowKey;				//龟速反馈

/*常规参数*/
static xSpeakerFun sgSpeakerFun;//蜂鸣器

/*动作限制参数*/
static xErrDisable sgErrDisable;//故障禁止动作
static xSWILimit sgSWILimit;//开关量反馈动作限制

/*处理速度、开关阀门*/
static xSpeedRate sgSpeedRate;//工作模式、速度比率
static int16_t i16HandleVal = 0;//手柄指令

/*防夹手功能*/
static xAntiPinchFlag sgAntiPinchFlag;

static uint32_t u32HourCount = 0;
static uint8_t u8Batterysoc = 0;

static uint16_t u16DrivrDisableFlag = 0;//故障禁止动作
static int16_t i16Speed_cmd;
static uint8_t u8Pump_cmd;
static int32_t i32Prop_cmd;

static int32_t i32PropFeedBack = 0;
static uint16_t i16SpeedFeedback = 0;
static uint8_t	u8PumpFeedback = 0;
static uint16_t u16AngleValue = 0;


xCanRevPdoInfo gCanRevPdoInfo;			/*PDO接收待完善*/
xCanSendPdoInfo gCanSendPdoInfo;                        /*PDO发送待完善*/
/*******************************************************************************
* Name: void vPcuErrProc(void)
* Descriptio: Pcu Err Proc,打开制动器，禁止起升下降左右行动
* Input: NULL
* Output: NULL 
*******************************************************************************/
void vPcuErrProc(uint8_t u8Type)
{
	switch (u8Type)
	{
		case PCU_Init:
			{
				u8PcuMode = INITIAL_MODE;
				sgSlowKey.b1PcuSpeed = FULL_SPEED;
				i32DoPwmSet(TURNLEFT_PUMP, PUMP_CLOSE_PERCENTAGE);
				i32DoPwmSet(TURNRIGHT_PUMP, PUMP_CLOSE_PERCENTAGE);
				i32DoPwmSet(LIFTUP_PUMP, PUMP_CLOSE_PERCENTAGE);
				vPropSetTarget(LIFTDOWN_PUMP_1, 0);
//				vPropSetTarget(LIFTDOWN_PUMP_2, 0);
			}
			break;
		case PCU_LiftKeyPress:
			{
				i32ErrCodeSet(LIFT_BUTTON_ERR);	
			}
			break;
		case PCU_SlowKeyPress:
			{
				i32ErrCodeSet(SLOW_BUTTON_ERR);
			}
			break;
		case PCU_MoveKeyPress:
			{
				i32ErrCodeSet(MOVE_BUTTON_ERR);
			}
			break;
		case PCU_TurnLeftPress:
			{
				i32ErrCodeSet(PLAT_LEFT_BUTTON_ERR);
			}
			break;
		case PCU_TurnRightPress:
			{
				i32ErrCodeSet(PLAT_RIGHT_BUTTON_ERR);
			}
			break;
		case PCU_EnableKeyPress:
			{
				i32ErrCodeSet(ENABLE_BUTTON_ERR);
			}
			break;
		case PCU_ValueNoZero:/*举升后行走*/
			{
				i32ErrCodeSet(HANDLE_NOT_IN_MIDDLE_POSI_ERR);
			}
			break;
		default:
			break;
	}
	//关停电机动作
}
/*******************************************************************************
* Name: void PcuRevProc(xPcuRevParat *RevData)
* Descriptio: 接收Pcu数据进行处理+开关量处理
* Input: RevData：Pcu接收的数据格式
* Output: NULL 
*******************************************************************************/
void vPcuRevProc(xPcuRevPara *RevData)//PCU数据处理,参数传递，动作判断
{
#if (PCU_TYPE_LZ == PCU_TYPE)
	if ((0x2 == RevData->Data.b4Const1) && (0x3 == RevData->Data.b4Const2) && \
			(0x4 == RevData->Data.b4Const3) && (0x5 == RevData->Data.b4Const4) && \
			(0x6 == RevData->Data.b4Const5) && (0x7 == RevData->Data.b4Const6) && \
			(0xC == RevData->Data.b4Const7))
	{
		uint8_t  u8KeyInfo = 0;
		int8_t i8MotorVal=0;

		/*PCU按键处理*/
		if((1 == i32LocalDiGet(PCU_SWICTH))
			&&(0 == (u16DrivrDisableFlag & PCU_NOACT_FLAG)))//上控、PCU正常
		{
			if(1 == RevData->Data.b1Mode)
			{
				//模式切换
			}
			else
			{
				
			}
			if(1 == RevData->Data.b1SlowSpdSwitch)
			{
				u8KeyInfo |= 1 << SLOW_KEY;
			}
			if(1 == RevData->Data.b1SpeakerSwitch)
			{
				u8KeyInfo |= 1 << SPEAKER_KEY;
			}
			if(1 == RevData->Data.b1TraSwitch)
			{
				u8KeyInfo |= 1 << MOVE_KEY;
			}

			if(1 == RevData->Data.b1LiftingSwitch)
			{
				u8KeyInfo |= 1 << LIFT_KEY;
			}

			if(1 == RevData->Data.b1EnableSwitch)
			{
				u8KeyInfo |= 1 << ENABLE_KEY;
			}
			if(1 == RevData->Data.b1TurnRightSwitch)
			{
				u8KeyInfo |= 1 << RIGHT_KEY;
			}
			if(1 == RevData->Data.b1TurnLeftSwitch)
			{
				u8KeyInfo |= 1 << LEFT_KEY;
			}
			/*Set Key Info*/
			
			i32SetPara(PARA_PcuKeyInfo, u8KeyInfo);
			
			i8MotorVal = RevData->Data.b4HandleCtrlHigh << 4 | RevData->Data.b4HandleCtrlLow;
						

			if(abs(i8MotorVal) <= i32GetPara(PAPA_DeadZoneAdjust))//死区调节
			{
				i8MotorVal = 0;
			}
			i32SetPara(PARA_HandleAnalog, abs(i8MotorVal) * 100 / 127);
			
			i16HandleVal = i8MotorVal *32;/*4096*/
//			i32LogWrite(INFO,"HD=%d\r\n",i16HandleVal);

		}
		else if(0 == i32LocalDiGet(PCU_SWICTH))//下控模式
		{
			u8KeyInfo = 0;
			if(1 == i32LocalDiGet(LOWERCONTROL_LIFT_SIWTCH))
			{
				sgSpeedRate.b1NormalDown = 0;
				sgSpeedRate.b1NormalLift = 1;
				i16HandleVal = 4066;
			}
			else if(1 == i32LocalDiGet(LOWERCONTROL_DOWN_SWITCH))
			{
				sgSpeedRate.b1NormalLift = 0;
				sgSpeedRate.b1NormalDown = 1;
				i16HandleVal = 4066;
			}
			else 
			{
				sgSpeedRate.u8data = 0;
			}
		}
		else//PCU异常、初始模式			/*lilu 20230706 恢复到默认模式*/
		{
			u8PcuMode = INITIAL_MODE;
			sgSlowKey.b1PcuSpeed = FULL_SPEED;
			sgErrDisable.u8data = 0;
		}
		
		
		/*PCU*/
		if (INITIAL_MODE == u8PcuMode) //初始模式，不能直接动作
		{
			if (u8KeyInfo & (1 << MOVE_KEY))
			{
				u8PcuMode = TRAVEL_MODE;
			}
			
			if(u8KeyInfo & (1 << LIFT_KEY))
			{
				u8PcuMode = LIFT_MODE;
			}
		}
		else//按下行走或起升后
		{
			/*slow key press*/
			if (u8KeyInfo & (1 << SLOW_KEY))
			{
				sgSlowKey.b1SlowKey = 1;
			}
			else 
			{
				/*按键松开时候进行切换*/
				if ((TRAVEL_MODE == u8PcuMode) && (1 == sgSlowKey.b1SlowKey))	/*只有行进模式才可以改变龟速状态*/
				{
					sgSlowKey.b1SlowKey = 0;
					sgSlowKey.b1PcuSpeed ^= 1; 
				}
			}
			/*Speaker Key Press*/
			if (u8KeyInfo & (1 << SPEAKER_KEY))
			{
				i32DoPwmSet(SPEAKER_PUMP, PUMP_OPEN_PERCENTAGE);
			}
			else
			{
				i32DoPwmSet(SPEAKER_PUMP, PUMP_CLOSE_PERCENTAGE);
			}
			
			
			/*Enable Key Press*/
			if (u8KeyInfo & (1 << ENABLE_KEY))
			{
				sgSpeedRate.u8data = 0;//先清空再进判断
				
				if (TRAVEL_MODE == u8PcuMode)
				{
				/*turn left key press*/
					if (u8KeyInfo & (1 << LEFT_KEY ))// 左转
					{
						if ((TRAVEL_MODE == u8PcuMode) 
							&& (0 == sgErrDisable.b1TurnDisable))//转向不检查开关量
						{
							sgSpeedRate.b1MoveTurnLeft = 1;
						}
					}
					else if (u8KeyInfo & (1 << RIGHT_KEY ))//右转
					{
						if ((TRAVEL_MODE == u8PcuMode) 
								&& (0 == sgErrDisable.b1TurnDisable))
						{
							sgSpeedRate.b1MoveTurnRight = 1;
						}
					}
					else //前后行走
					{
						sgSpeedRate.b1MoveTurnLeft = 0;						
						sgSpeedRate.b1MoveTurnRight = 0;
					}
					if (0 != i16HandleVal ) //行走
					{
						if(0 == sgErrDisable.b1MoveDisable)//故障检测
						{
							sgSpeedRate.b1NormalMove = 1;	//正常行走使能
							
							if (SLOW_SPEED == sgSlowKey.b1PcuSpeed)
							{
								sgSpeedRate.b1SlowMove = 1;
							}
							
							if((1 == sgErrDisable.b1SpeedLimit)//故障速度限制
								||(1 == sgSWILimit.b1SpeedLimit))//开关量速度限制
							{
								sgSpeedRate.b1MoveAfterLift = 1;
							}
							
						}
					}
				}
				else if (LIFT_MODE == u8PcuMode)
				{
					
					if (FunctionEnable == i32GetPara(PARA_LiftReverseFunc))//起升反向
					{
						i16HandleVal = 0 - i16HandleVal;
					}
					
					if ((i16HandleVal > 0) 
						&& (0 == sgErrDisable.b1LiftDisabel)
						&&(0 == sgSWILimit.b1LiftDisabel))   //起升
					{
							sgSpeedRate.b1NormalLift = 1;
						
					}
					else if((i16HandleVal < 0)
									&& (0 == sgErrDisable.b1DownDisable)
									&& (0 == sgSWILimit.b1DownDisable))  
					{

						if(0 == sgSWILimit.b1DownDelay)//防夹手未触发
						{
							sgSpeedRate.b1NormalDown = 1;
							sgAntiPinchFlag.u8data = 0;
						}
						else//下限位开关触发
						{
							if(0 == sgAntiPinchFlag.b1Arrive)
							{
								sgAntiPinchFlag.b1Arrive =1;
								sgSpeedRate.b1NormalDown = 0;
							}
							else if(0 == sgAntiPinchFlag.b1UnderLimit)//第一次下降到位
							{
								//不执行任何操作
							}
							else if(1 == sgAntiPinchFlag.b1UnderLimit)//下限位之下，延时打开
							{
								if(false == u8GetNetTimerStartFlag(TIMER_EcuAntiPinchFunc))
								{
									vSetNetTimer(TIMER_EcuAntiPinchFunc, ECU_ANTI_PINCH_TIME);//松开使能后关掉
								}
								if(true == u8GetNetTimerOverFlag(TIMER_EcuAntiPinchFunc))
								{
									sgSpeedRate.b1NormalDown = 1;
								}
								
							}					
						}
					}
					else //停止
					{
						sgSpeedRate.u8data = 0;
						vKillNetTimer(TIMER_EcuAntiPinchFunc);
					}
				}
			}
			else//没按使能键后操作
			{
				sgSpeedRate.u8data = 0;
				/*Close All Pumps*/
				//电机+制动器
				vKillNetTimer(TIMER_EcuAntiPinchFunc);
				
				if(1 == sgAntiPinchFlag.b1Arrive)
					sgAntiPinchFlag.b1UnderLimit = 1;
				else
					sgAntiPinchFlag.u8data = 0;
				
				if (u8KeyInfo & (1 << MOVE_KEY))
				{
					u8PcuMode = TRAVEL_MODE;
				}
				if(u8KeyInfo & (1 << LIFT_KEY))
				{
					u8PcuMode = LIFT_MODE;
				}
			}
		}
	}

#endif
}

/*发送给PCU的回调函数*/
static void vPcuSendProc(xPcuSendPara *SendData)
{
	if ((SLOW_SPEED == sgSlowKey.b1PcuSpeed)
		||(1 == sgErrDisable.b1SpeedLimit)//故障速度限制
		||(1 == sgSWILimit.b1SpeedLimit))
	{
		SendData->Data.b1SlowLed = 1;
	}
	else
	{
		SendData->Data.b1SlowLed = 0;
	}
	
	if(TRAVEL_MODE == u8PcuMode)
	{
		SendData->Data.b1ModeLed = 1;
		SendData->Data.b1LiftLed = 0;
	}
	else
	{
		SendData->Data.b1ModeLed = 0;
		SendData->Data.b1LiftLed = 1;
	}
}
/*速度调节函数*/
static int32_t i32SpeedAdjust(int16_t Adj_Speedcmd ,int16_t Adj_Speedmax,
	uint8_t Adj_Steptime,uint8_t Adj_Brakespd, uint8_t Adj_Accspd)
{//手柄指令4096，速度档位100，加减速周期255，刹车加速度255，调速加速度255
	static int32_t Adj_Speedold = 0;//记录值，用于比较
	static uint8_t Adj_Stepcnt = 0;//
	static int32_t Adj_Speedmaxold = 0;
	int32_t Adj_Steplength = 0;
	uint8_t Adj_Stepodd = 0;
	

	Adj_Speedcmd = Adj_Speedcmd * Adj_Speedmax / 100;//手柄指令换成对应速度比例
	Adj_Speedmax = Adj_Speedmax * 4096 / 100;	//最大速度指令映射到4096
	
	if(Adj_Speedmax != Adj_Speedmaxold)//换挡重置
	{
		Adj_Speedmaxold = Adj_Speedmax;
		Adj_Stepcnt = 0;
	}
	
	if((0 == Adj_Speedcmd)&&(0 != Adj_Speedold))//刹车
	{
		Adj_Steplength = Adj_Speedmax * Adj_Brakespd / 255 / Adj_Steptime ;
		Adj_Stepodd = (Adj_Speedmax * Adj_Brakespd /255)% Adj_Steptime;
	}
	else if(0 != Adj_Steptime)
	{
		Adj_Steplength = Adj_Speedmax * Adj_Accspd /255/ Adj_Steptime ;
		Adj_Stepodd = (Adj_Speedmax * Adj_Accspd /255)% Adj_Steptime;
	}
	else//不参与调速
	{
		
	}
	
//	i32LogWrite(INFO,"spd = %d \r \n",Adj_Steplength);
	
	if((abs(Adj_Speedcmd - Adj_Speedold)<(Adj_Speedmax/3))
		||((abs(Adj_Speedcmd)<(Adj_Speedmax /4))&&(0 != Adj_Speedcmd)))//分段调速,低速率、低差异时响应幅度小
	{
		Adj_Steplength = Adj_Steplength / 2;
	}
	
	//i32LogWrite(INFO,"spdadj = %d \r \n",Adj_Steplength);
	
	if(0 != Adj_Steptime)
	{
		if(Adj_Stepcnt < Adj_Stepodd)
		{
			Adj_Stepcnt ++ ;
			if(Adj_Speedcmd < Adj_Speedold)
			{
				if((abs(Adj_Speedcmd - Adj_Speedold))>Adj_Steplength)
				{
					Adj_Speedcmd = Adj_Speedold - Adj_Steplength - 1;
				}
					
			}
			else if(Adj_Speedcmd > Adj_Speedold)
			{
				if((abs(Adj_Speedcmd - Adj_Speedold))>Adj_Steplength)
				{
					Adj_Speedcmd = Adj_Speedold + Adj_Steplength + 1;
				}
			}
		}
		else if(Adj_Stepcnt < Adj_Steplength)
		{
			Adj_Stepcnt ++ ;
			if(Adj_Speedcmd < Adj_Speedold)
			{
				if((abs(Adj_Speedcmd - Adj_Speedold))>Adj_Steplength)
				{
					Adj_Speedcmd = Adj_Speedold - Adj_Steplength ;
				}
					
			}
			else if(Adj_Speedcmd > Adj_Speedold)
			{
				if((abs(Adj_Speedcmd - Adj_Speedold))>Adj_Steplength)
				{
					Adj_Speedcmd = Adj_Speedold + Adj_Steplength ;
				}
			}
		}
		else
		{
			Adj_Stepcnt = 0 ;
			if(Adj_Speedcmd < Adj_Speedold)
			{
				if((abs(Adj_Speedcmd - Adj_Speedold))>Adj_Steplength)
				{
					Adj_Speedcmd = Adj_Speedold - Adj_Steplength ;
				}
			}
			else if(Adj_Speedcmd > Adj_Speedold)
			{
				if((abs(Adj_Speedcmd - Adj_Speedold))>Adj_Steplength)
				{
					Adj_Speedcmd = Adj_Speedold + Adj_Steplength ;
				}
			}
		}
	}

	Adj_Speedold = Adj_Speedcmd;
	
	return Adj_Speedcmd;
	
}
/*******************************************************************************
* void MstRevProc(xMstRevParat *RevData)
* Descriptio: 接收电控MCU数据进行处理
* Input: RevData：电控MCU接收的数据格式
* Output: NULL 
*******************************************************************************/
static void vMstRevProc(xMstRevPara *RevData)//电机状态报警
{	
//	i32LogWrite(INFO, "Rev Mst Success!\r\n");
	uint16_t tmp = 0;
//	static uint8_t u8OldErrCode = 0;
	
	if(0 != RevData->b1MainDriver)
	{
		
	}
	else
	{
		
	}
	
	if(0 != RevData->b1Ebrake)
	{
		
	}
	else
	{	
		
	}
	
	if(0 != RevData->b1Driver3State)
	{
		
	}
	else
	{
		
	}
	/**/
	gCanSendPdoInfo.sgHMISendPdo.ECUErr = 0;
	if(0 != u8ErrCodeGet() )
	{
	gCanSendPdoInfo.sgHMISendPdo.ECUErr = u8ErrCodeGet();
	gCanSendPdoInfo.sgHMISendPdo.ECUErr =  (gCanSendPdoInfo.sgHMISendPdo.ECUErr | 0x80);
	}
	
	if(0 != RevData->u8ErrCode)
	{
		if (26 == RevData->u8ErrCode)		/*控制器通讯故障报85*/
		{
			i32ErrCodeSet(ErrCode85);
		}
		else								/*其它报81*/
		{
			i32ErrCodeSet(ErrCode81);
		}
		vLedSendAlmCode(RevData->u8ErrCode);
		gCanSendPdoInfo.sgHMISendPdo.ECUErr =  RevData->u8ErrCode;
	}
	
	tmp = (uint16_t)(RevData->u8SpeedFdbHigh << 8) | RevData->u8SpeedFdbLow ; 
	tmp = tmp * 100 / MOTOR_MAX_SPEED_VALUE;
	i32SetPara(PARA_MotorSpd, tmp);

	//温度设置//待定
	tmp = RevData->u8MotorTmp;//电机温度
//	i32LogWrite(INFO,"motortemp = %d",tmp);
//	if(tmp < 10)//驱动器低温
//	{
//		i32ErrCodeSet(PUMP_MOTOR_TEMP1_ERR);
//	}
//	else if(80 < tmp)//高温一级
//	{
//		i32ErrCodeSet(PUMP_MOTER_TEMP2_ERR);
//	}
//	else
//	{
//		i32ErrCodeClr(PUMP_MOTOR_TEMP1_ERR);
//		i32ErrCodeClr(PUMP_MOTER_TEMP2_ERR);
//	}
//	
	tmp = RevData->u8BoardTmp;//主板温度
//	i32LogWrite(INFO,"boardtemp = %d",tmp);
//	if(tmp < 10)//驱动器低温
//	{
//		i32ErrCodeSet(CTRL_TEMP_LOW_ERR);
//	}
//	else if(80 < tmp)//高温一级
//	{
//		i32ErrCodeSet(CTRL_TEMP_HIGH1_ERR);
//	}
//	else if(90<tmp)
//	{
//		i32ErrCodeSet(CTRL_TEMP2_ERR);
//	}
//	else
//	{
//		i32ErrCodeClr(CTRL_TEMP_LOW_ERR);
//		i32ErrCodeClr(CTRL_TEMP_HIGH1_ERR);
//		i32ErrCodeClr(CTRL_TEMP2_ERR);
//	}
	
}


static void vMstSendProc(xMstSendPara *SendData) //电机发送数据
{ 
	/*调速参数*/
	static int32_t i32Maxspeedrate = 100;//初始化为100
	static uint8_t u8Steptime = 0;
	static int32_t i32Brakespd = 0;
	static int32_t i32Accspd = 0;
	static int8_t i8StateRecord = 0;//左右转，起升
	
	int16_t i16Tmp = 0;
	
	int16_t i16Spd = 0;
	uint8_t	u8Pump=0;
	int32_t i32Prop = 0;
	
	//
	SendData->buf[2] = 0;
	

	int32_t i32PropMax ;
	i32PropMax = _IQ(PROPD_MAX_CURRENT * i32GetPara(PARA_LowerSpeed)/100 / PROPD_STD_CURRENT);
	
	
	/*转向开关阀处理*/
	if(1 == sgSpeedRate.b1MoveTurnLeft)
	{
		i32DoPwmSet(TURNLEFT_PUMP, PUMP_OPEN_PERCENTAGE);
		i32DoPwmSet(TURNRIGHT_PUMP, PUMP_CLOSE_PERCENTAGE);
		i32SetPara(PARA_TurnLeftValveCurrent, i32GetPara(PARA_ValueOpenLoopCurrent));
		i32SetPara(PARA_TurnRightValveCurrent, 0);
	}
	else if(1 == sgSpeedRate.b1MoveTurnRight)
	{
		i32DoPwmSet(TURNLEFT_PUMP, PUMP_CLOSE_PERCENTAGE);
		i32DoPwmSet(TURNRIGHT_PUMP, PUMP_OPEN_PERCENTAGE);
		i32SetPara(PARA_TurnLeftValveCurrent, 0);
		i32SetPara(PARA_TurnRightValveCurrent, i32GetPara(PARA_ValueOpenLoopCurrent));
	}
	else
	{
		i32DoPwmSet(TURNLEFT_PUMP, PUMP_CLOSE_PERCENTAGE);
		i32DoPwmSet(TURNRIGHT_PUMP, PUMP_CLOSE_PERCENTAGE);
		i32SetPara(PARA_TurnLeftValveCurrent, 0);
		i32SetPara(PARA_TurnRightValveCurrent, 0);
	}
	
	/*行走速度处理*/
	if(1 == sgSpeedRate.b1NormalMove)//行走使能
	{
		//正常速度
			i32Maxspeedrate = i32GetPara(PARA_FastDriveSpeed);
			u8Steptime =  i32GetPara(PARA_AccAndDecFastDrive);
			i32Brakespd = i32GetPara(PARA_BrakeFastDrive)  ;
			i32Accspd = i32GetPara(PARA_CurveFastDrive)  ;
		
		if(1 == sgSpeedRate.b1MoveAfterLift)//起升后速度
		{
			if(i32Maxspeedrate > i32GetPara(PARA_DriveSpeedAfterLift))
				i32Maxspeedrate = i32GetPara(PARA_DriveSpeedAfterLift);
			u8Steptime =  i32GetPara(PARA_AccAndDecAfterLift);
			i32Brakespd = i32GetPara(PARA_BrakeDriveAfterLift)  ;
			i32Accspd = i32GetPara(PARA_CurveDriveAfterLift)  ;
		}

	 if(1 == sgSpeedRate.b1SlowMove)//慢速
		{
			if(i32Maxspeedrate > i32GetPara(PARA_SlowDriveSpeed))
				i32Maxspeedrate = i32GetPara(PARA_SlowDriveSpeed);
			u8Steptime =  i32GetPara(PARA_AccAndDecSlowDrive);
			i32Brakespd = i32GetPara(PARA_BrakeSlowDrive)  ;
			i32Accspd = i32GetPara(PARA_CurveSlowDrive)  ;
		}

		if((1 == sgSpeedRate.b1MoveTurnRight)
				||(1 == sgSpeedRate.b1MoveTurnLeft))//转向
		{
			if(i32Maxspeedrate > i32GetPara(PARA_MaxTurnSpeed))
				i32Maxspeedrate = i32GetPara(PARA_MaxTurnSpeed) ;
			u8Steptime =  i32GetPara(PARA_AccAndDecTurn);
			i32Brakespd = i32GetPara(PARA_BrakeTurn)  ;
			i32Accspd = i32GetPara(PARA_CurveTurn)  ;
		}

		/*行走加减速调节*/
		i16Spd = i32SpeedAdjust(i16HandleVal,i32Maxspeedrate,
							u8Steptime,i32Brakespd,i32Accspd);
	}
	else if(i16SpeedFeedback>MOTOR_FDB_CLOSE_VALUE)//刹车过程
	{
		i16Spd = i32SpeedAdjust(0,i32Maxspeedrate,
							u8Steptime,i32Brakespd,i32Accspd);
	}
	else
	{
		i16Spd = 0;
	}
		

	if (i16Spd >= MOTOR_MAX_SPEED_VALUE)
	{
		i16Spd = MOTOR_MAX_SPEED_VALUE;
	}
		/*油泵命令指令*/
	if(1==sgSpeedRate.b1NormalLift)
	{
		i16Spd = 0;
		i32Maxspeedrate = i32GetPara(PARA_LiftSpeed);
		u8Steptime =  i32GetPara(PARA_AccAndDecLift);
		i32Brakespd = i32GetPara(PARA_BrakeLift) ;
		i32Accspd = i32GetPara(PARA_CurveLift) ;
		if(i16HandleVal>=0)
			u8Pump = (uint8_t)(i32SpeedAdjust(i16HandleVal,i32Maxspeedrate,
				u8Steptime,i32Brakespd,i32Accspd)/16);/*缩放到256*/
		u8PumpFeedback = u8Pump ;
	}
	else if(u8PumpFeedback > PUMP_FDB_CLOSE_VALUE)//刹车减速
	{
		u8Pump = (uint8_t)(i32SpeedAdjust(0,i32Maxspeedrate,
			u8Steptime,i32Brakespd,i32Accspd)/16);/*缩放到256*/
		u8PumpFeedback = u8Pump ;
	}
	else if((1 == sgSpeedRate.b1MoveTurnLeft)||(1 == sgSpeedRate.b1MoveTurnRight))
	{
		u8Pump = i32GetPara(PARA_TurnPowerLimit) * 255 * 2 / 100;
		u8PumpFeedback = 0 ;//转向时不参与反馈
	}
	else
	{
		u8PumpFeedback = 0 ;
		u8Pump = 0;
	}
	
	/*下降阀门*/
	if(1 == sgSpeedRate.b1NormalDown)
	{
		i32Prop = _IQ((i32GetPara(PARA_PropDMinCurrent0)+
							((i32GetPara(PARA_PropDMaxCurrent0)-i32GetPara(PARA_PropDMinCurrent0)) 
							* (0 - i16HandleVal) / MOTOR_MAX_SPEED_VALUE)) / PROPD_STD_CURRENT/1000); /*0.2A Test*/

		i32SetPara(PARA_PropValveCurrent, (inserted_data[1] * 3300 * 7.5 / 4096 / (7.5 + 68) / 0.05));
		sgSpeakerFun.b1240PerMin = 1;
	}
	else if(i32PropFeedBack>PROP_FDB_CLOSE_VALUE)//延迟关闭
	{
		i32Prop = 0;
		i32SetPara(PARA_PropValveCurrent, (inserted_data[1] * 3300 * 7.5 / 4096 / (7.5 + 68) / 0.05));		
		sgSpeakerFun.b1240PerMin = 1;
	}
	else
	{
		i32Prop = 0;
		i32SetPara(PARA_PropValveCurrent, (inserted_data[1] * 3300 * 7.5 / 4096 / (7.5 + 68) / 0.05));
//		i32SetPara(PARA_PropValveCurrent, 0);
		sgSpeakerFun.b1240PerMin = 0;
	}
	

	if(((1==sgSpeedRate.b1NormalLift)
		||(u8PumpFeedback > PUMP_FDB_CLOSE_VALUE)
	#ifdef LIFTDOWN_WITH_UPPERPUMPOPEN//山东坤宇下降时打开起身阀
		||(1 == sgSpeedRate.b1NormalDown)
		||(i32PropFeedBack>PROP_FDB_CLOSE_VALUE)
	#endif		//#ifdef LIFTDOWN_WITH_UPPERPUMPOPEN
				)
				&&((0 == sgSpeedRate.b1MoveTurnLeft)
					&&(0 == sgSpeedRate.b1MoveTurnRight)))
	{
		i32DoPwmSet(LIFTUP_PUMP, PUMP_OPEN_PERCENTAGE);
		i32SetPara(PARA_LiftValveCurrent, i32GetPara(PARA_ValueOpenLoopCurrent));
		
	}
	else
	{
		i32DoPwmSet(LIFTUP_PUMP, PUMP_CLOSE_PERCENTAGE);
		i32SetPara(PARA_LiftValveCurrent, 0);
	}
	
	
	/*指令发送*/
	SendData->u8TargetHigh = i16Spd >> 8;
	SendData->u8TargetLow = i16Spd;
	SendData->u8PumpTarget=u8Pump;
	
	if(0 != u8Pump)
	{
		SendData->b1LiftReq = 1;
	}
	if (i16Spd > 0)
	{
		SendData->b1ForwardReq = 1;
	}
	else if(i16Spd < 0)
	{
		SendData->b1BackwardReq = 1;
	}
	
	if((abs(i16Spd)>0)||(u8Pump>0))
	{
		SendData->b1ServoOn = 1;
	}
	
	vPropSetTarget(LIFTDOWN_PUMP_1, i32Prop);
		
	i16SpeedFeedback = i16Spd;
	
	i32PropFeedBack = inserted_data[1] * 3300 * 7.5 / 4096 / (7.5 + 68) / 0.05;
	
}

static void vDoPwmErrCallBack(eDoPwmNo DoPwmNo)
{
	switch((uint8_t) DoPwmNo)
	{
		case TURNLEFT_PUMP:
			i32ErrCodeSet(TURN_LEFT_VALVE_ERR);
			break;
		case TURNRIGHT_PUMP:
			i32ErrCodeSet(TURN_RIGHT_VALVE_ERR);
			break;
		case LIFTUP_PUMP:
			i32ErrCodeSet(LIFT_UP_VALVE_ERR);
			break;
		default:
			break;
	}
}

static void vPropErrCallBack(uint8_t u8Channel)
{
	switch(u8Channel)
	{
		case LIFTDOWN_PUMP_2:
			i32ErrCodeSet(LIFT_DOWN_VALVE_ERR);
			break;
		default:
			break;
	}
}

/*蜂鸣器回调函数*/
static void vBeepCallBack(uint8_t u8Flag)//end
{
	if(1 == u8Flag)
	{
		i32DoPwmSet(BLINK_BEEP, PUMP_OPEN_PERCENTAGE);
	}
	else
	{
		i32DoPwmSet(BLINK_BEEP, PUMP_CLOSE_PERCENTAGE);
	}
}
/*闪光灯回调函数*/
static void vAlarmLampCallBack(uint8_t u8Flag)//end
{
	if(1 == u8Flag)
	{
		i32DoPwmSet(BLINK_LED, PUMP_OPEN_PERCENTAGE);
	}
	else
	{
		i32DoPwmSet(BLINK_LED, PUMP_CLOSE_PERCENTAGE);
	}
}
/*接收PDO处理函数*/
static void vCanRevPdoProc(void)
{
	u8Batterysoc = u8GetBatterySoc();
	if((u8Batterysoc < i32GetPara(PARA_VoiceAlarmVolume))
		&&(FunctionEnable == i32GetPara(PARA_LowBatAlmFunc)))//电量档位报警
	{
		i32ErrCodeSet(BATTERY_LOW_CAP1_ERR);
	}
	else
	{
		i32ErrCodeClr(BATTERY_LOW_CAP1_ERR);
	}
}
/*发送PDO处理函数*/
static void vCanSendPdoProc(void)
{
	gCanSendPdoInfo.sgHMISendPdo.SOC = u8Batterysoc;
	i32SetPara(PARA_BmsSoc, u8Batterysoc * 2.5);
	
	if(1 == i32LocalDiGet(PCU_SWICTH))
	{
		gCanSendPdoInfo.sgHMISendPdo.WorkMode |= STATE_MODE;
	}
	else
	{
		gCanSendPdoInfo.sgHMISendPdo.WorkMode &= ~STATE_MODE;
	}
	
}
/*角度传感器回调函数*/
static void vAngleCallBack(uint8_t u8Type)
{
	static uint8_t u8LastAngleType = 0;
	if (1 == i32LocalDiGet(PIT_SWITCH))
	{
		if (u8LastAngleType != u8Type)
		{
			if(1 == u8Type)
			{
				i32ErrCodeSet(ANGLE_SENSOR_ERR);
			}
			else
			{
				i32ErrCodeClr(ANGLE_SENSOR_ERR);
			}
			u8LastAngleType = u8Type;
		}
	}
}
/*压力传感器回调函数*/
static void vPressureCallBack(ePressureNo PressureNo)
{
	static xPresureErrInfo LastErrFlag = {0};
	xPresureErrInfo ErrFlag = {0};

	switch ((uint8_t)PressureNo)
	{
		case PressureCaliReverse:
			ErrFlag.b1CaliReverse = 1;
			break;
		case PressureCaliFailure:
			ErrFlag.b1CaliFailure = 1;
			break;
		case PressureWithoutSensor:
			if (1 == i32LocalDiGet(PIT_SWITCH))	/*lilu 20230706 增加坑洞判断*/
			{
				ErrFlag.b1SenSorErr = 1;
			}
			break;
		case PressureOverPer80:
			ErrFlag.b1Per80Err = 1;
			break;
		case PressureOverPer90:
			ErrFlag.b1Per90Err = 1;
			break;
		case PressureOverPer99:
			ErrFlag.b1Per99Err = 1;
			break;
		case PressureOverPer100:
			ErrFlag.b1Per100Err = 1;
			break;
		default:
			break;
	}
	
	if (LastErrFlag.u8Data != ErrFlag.u8Data)
	{
		if(1 == ErrFlag.b1SenSorErr)
		{
			i32ErrCodeSet(PRESSURE_SENSOR_ERR);
		}
		else
		{
			i32ErrCodeClr(PRESSURE_SENSOR_ERR);
		}
		
		if(1 == ErrFlag.b1CaliReverse)
		{
			i32ErrCodeSet(WEIGHT_CALI_REVESER_ERR);
		}
		else
		{
			i32ErrCodeClr(WEIGHT_CALI_REVESER_ERR);
		}
	
		if(1 == ErrFlag.b1CaliFailure)
		{
			i32ErrCodeSet(CALIBRATION_FAILURE_ERR);
		}
		else
		{
			i32ErrCodeClr(CALIBRATION_FAILURE_ERR);
		}
		
		if(1 == ErrFlag.b1Per80Err)
		{
			i32ErrCodeSet(OVER_80_PER_LOAD_ERR);
		}
		else
		{
			i32ErrCodeClr(OVER_80_PER_LOAD_ERR);
		}
		
		if(1 == ErrFlag.b1Per90Err)
		{
			i32ErrCodeSet(OVER_90_PER_LOAD_ERR);
		}
		else
		{
			i32ErrCodeClr(OVER_90_PER_LOAD_ERR);
		}
		
		if(1 == ErrFlag.b1Per99Err)
		{
			i32ErrCodeSet(OVER_99_PER_LOAD_ERR);
		}
		else
		{
			i32ErrCodeClr(OVER_99_PER_LOAD_ERR);
		}
		
		if(1 == ErrFlag.b1Per100Err)
		{
			i32ErrCodeSet(PLAT_OVERLOAD_ERR);
			
		}
		else
		{
			i32ErrCodeClr(PLAT_OVERLOAD_ERR);
		}
		
		LastErrFlag.u8Data = ErrFlag.u8Data;
	}
}

static void vDoPwmErrProc(eDoPwmNo DoPwmNo)
{
	switch((uint8_t) DoPwmNo)
	{
		case TURNLEFT_PUMP:
			i32ErrCodeSet(TURN_LEFT_VALVE_ERR);
			break;
		case TURNRIGHT_PUMP:
			i32ErrCodeSet(TURN_RIGHT_VALVE_ERR);
			break;
		case LIFTUP_PUMP:
			i32ErrCodeSet(LIFT_UP_VALVE_ERR);
			break;
		default:
			break;
	}
}
/*******************************************************************************
* Name: void vUserEcuInit(void)
* Descriptio: UserEcu初始化函数
* Input: NULL
* Output: NULL  
*******************************************************************************/
void vUserEcuInit(void)
{
	vPcuErrRegister(vPcuErrProc);	
	vPcuRevRegister(vPcuRevProc);
	vPcuSendRegister(vPcuSendProc);
	
	vMstRevRegister(vMstRevProc);
	vMstSendRegister(vMstSendProc);
	
	vDoPwmErrReg(vDoPwmErrCallBack);
	vPropErrReg(vPropErrCallBack);
	
	
	
	vAngleSensorReg(vAngleCallBack);
	vPressureSensorReg(vPressureCallBack);
	
	vBeepRegister(vBeepCallBack);
	vAlarmLampRegister(vAlarmLampCallBack);
	vAlarmLampSetPeriod(30);			/*StartUp 30/min*/
	vSetPdoPara(PdoPara);
	
	vSetNetTimer(TIMER_EcuPowerOn, ECU_POWERON_DELAY_TIME);	/**/
	
	vSetNetTimer(TIMER_HourCount, HOURCOUNT_PERIOD);
	
	u32HourCount = u32HourCountRead();

}


static void vAiMonitor(void)
{
	uint16_t u16AdcValue = 0;
	
	u16AdcValue = i32LocalAiGetValue(AI_B_VBUS_CHECK);
	i32SetPara(PARA_BatteryVoltage, u16AdcValue);
	if (u16AdcValue < 18000)		/*lilu 20230703 低于二级报警电压*/
	{
		i32ErrCodeSet(BAT_LOW_CAP2_ERR);
	}
	else
	{
		i32ErrCodeClr(BAT_LOW_CAP2_ERR);
	}
}
static void vSwiMonitor(void)
{
	uint16_t u16ExtInput = 0;
	
	/*开关量获取*/
	if(1 == i32LocalDiGet(PCU_SWICTH))//上下控切换
	{
		u16ExtInput |= 1 << PCU_STATE;
	}
	else
	{
		sgErrDisable.b1PCUDisable = 1;
	}
	
	if(1 == i32LocalDiGet(TILT_SIWTCH))//倾角开关，闭合只能下降
	{
		u16ExtInput |= 1 << TILT_STATE;
	}
	
	if(1 == i32LocalDiGet(LOWERCONTROL_LIFT_SIWTCH))//上升（下控）
	{
		u16ExtInput |= 1 << LIFT_STATE;
	}
	
	if(1 == i32LocalDiGet(PEDAL_SIWTCH))//踏板
	{
		u16ExtInput |= 1 << PEDAL_STATE;
	}
	
	if(1 == i32LocalDiGet(PIT_SWITCH))//坑洞开关24v展开,速度为起升后速度，0v收起
	{
		u16ExtInput |= 1 << PIT_STATE;
	}
	
	if(1 == i32LocalDiGet(LOWERCONTROL_DOWN_SWITCH))
	{
		u16ExtInput |= 1 << DOWN_STATE;
	}
	
	if(1 == i32LocalDiGet(UP_LIMIT_SWITCH))
	{
		u16ExtInput |= 1 << UP_LIMIT_STATE;
	}
	
	if(1 == i32LocalDiGet(DOWN_LIMIT_SWITCH))
	{
		u16ExtInput |= 1 << DOWN_LIMIT_STATE;
	}
		
	/*Set External Input*/
	i32SetPara(PARA_ExtSignal, u16ExtInput);
	
	
	/*上下限位功能*/
	if(FunctionEnable == i32GetPara(PARA_AngleSimulationLimit))
	{
		if(i32LocalAiGet(ANGLE_ADC_CHANNEL)<i32GetPara(PARA_SetDescentHeightValue))//可下降高度值
		{
			sgSWILimit.b1DownDisable =1;
			sgSWILimit.b1DownDelay = 1;
			sgSWILimit.b1LiftDisabel = 0;
		}
		else if(i32LocalAiGet(ANGLE_ADC_CHANNEL)<i32GetPara(PARA_AngleSimulationDownLimit))//角度模拟下限位
		{
			sgSWILimit.b1DownDisable = 0;
			sgSWILimit.b1DownDelay = 1;
			sgSWILimit.b1LiftDisabel = 0;
		}
		else if(i32LocalAiGet(ANGLE_ADC_CHANNEL)<i32GetPara(PARA_AngleSimulationUpLimit))//不限制动作区间
		{
			sgSWILimit.b1DownDisable = 0;
			sgSWILimit.b1DownDelay = 0;
			sgSWILimit.b1LiftDisabel = 0;
		}
		else//大于角度模拟上限位
		{
			sgSWILimit.b1DownDisable = 0;
			sgSWILimit.b1DownDelay = 0;
			sgSWILimit.b1LiftDisabel = 1;
		}
	}
	else //开关量限位
	{
		if(0 == i32LocalDiGet(UP_LIMIT_SWITCH))
			sgSWILimit.b1LiftDisabel = 1;
		else
			sgSWILimit.b1LiftDisabel = 0;
		
		if(1 == i32LocalDiGet(DOWN_LIMIT_SWITCH))
			sgSWILimit.b1DownDelay = 1;
		else
			sgSWILimit.b1DownDelay = 0;
	}
	
	/*坑洞和倾斜*/
	if(0 == sgSWILimit.b1DownDelay)//下限位之上
	{
		if (0 == (u16ExtInput & (1 << TILT_STATE)))
			i32ErrCodeSet(MACHINE_TILT_OVER_SAFETY_ERR);
		else
			i32ErrCodeClr(MACHINE_TILT_OVER_SAFETY_ERR);
		
		if(1 == i32LocalDiGet(PIT_SWITCH))
			i32ErrCodeSet(PIT_PROCETION_ERR);
		else
		{
			i32ErrCodeClr(PIT_PROCETION_ERR);
			sgSWILimit.b1SpeedLimit = 1;
		}
	}
	else
	{
		sgSWILimit.b1SpeedLimit = 0;
		i32ErrCodeClr(PIT_PROCETION_ERR);
		i32ErrCodeClr(MACHINE_TILT_OVER_SAFETY_ERR);
	}
}

/*设置蜂鸣器的周期，根据不同的频率，设置不同的蜂鸣器频率*/
static void vEcuSetBeepPeriod(void)
{
	if(sgSpeakerFun.b1240PerMin)		/*下降动作*/
	{
		vBeepSetPeriod(240);		
		sgSpeakerFun.u8Cnt++;
		if(sgSpeakerFun.u8Cnt >= 180)
		{
			sgSpeakerFun.u8Cnt = 0;
		}
		if(sgSpeakerFun.u8Cnt < 144)
		{
			vBeepSetPeriod(240);
		}
		else
		{
			vBeepSetPeriod(0);
		}
	}
	else if(sgSpeakerFun.b1180PerMin)	/*可恢复报警*/
	{
		vBeepSetPeriod(180);
	}
	else if(sgSpeakerFun.b160PerMin)	/*不可恢复报警*/
	{
		vBeepSetPeriod(60);
	}
	else if((sgSpeakerFun.b130PerMin) && (FunctionEnable == i32GetPara(PARA_ActAlmFunc)))		/*动作报警*/
	{
		vBeepSetPeriod(30);
	}
	else
	{
		vBeepSetPeriod(0);
	}
}
/*******************************************************************************
* Name: void vUserEcuProc(void)
* Descriptio: UserEcu用户处理函数
* Input: NULL
* Output: NULL  
*******************************************************************************/
void vUserEcuProc(void)
{
	static uint8_t u8EcuProcFlag = 0;
	uint8_t u8ErrCode = 0;
	static uint8_t u8OneMinute = 0;

	if(true == u8GetNetTimerOverFlag(TIMER_EcuPowerOn))
	{
		u8EcuProcFlag = 1;
		vKillNetTimer(TIMER_EcuPowerOn);
	}
	
	if(1 == u8EcuProcFlag)
	{
		vSwiMonitor();
		vAiMonitor();
		sgErrDisable.b1PCUDisable = u8ErrCodeGetAbnormal(ABNORMAL_PCU);
		
		sgErrDisable.b1SpeedLimit = u8ErrCodeGetAbnormal(ABNORMAL_LIFTSPD);
		
		sgErrDisable.b1DownDisable = u8ErrCodeGetAbnormal(ABNORMAL_NOACT) 
																| u8ErrCodeGetAbnormal(ABNORMAL_DOWN);
																
		sgErrDisable.b1LiftDisabel = u8ErrCodeGetAbnormal(ABNORMAL_NOACT)
																|	u8ErrCodeGetAbnormal(ABNORMAL_UP) ;
																
		sgErrDisable.b1MoveDisable = u8ErrCodeGetAbnormal(ABNORMAL_NOACT) 
																| u8ErrCodeGetAbnormal(ABNORMAL_BACKWARD) 
																|	u8ErrCodeGetAbnormal(ABNORMAL_FORWARD);
																
		sgErrDisable.b1TurnDisable = u8ErrCodeGetAbnormal(ABNORMAL_LEFT) 
																| u8ErrCodeGetAbnormal(ABNORMAL_RIGHT) 
																| u8ErrCodeGetAbnormal(ABNORMAL_NOACT);
		
	if (true == u8GetNetTimerOverFlag(TIMER_HourCount))
	{
		vResetNetTimer(TIMER_HourCount);
		u32HourCount++;
		
			__disable_irq();
			gCanSendPdoInfo.sgHMISendPdo.u8WorkTimeLL = u32HourCount & 0xFF;
			gCanSendPdoInfo.sgHMISendPdo.u8WorkTimeLH = (u32HourCount >> 8) & 0xFF;
			gCanSendPdoInfo.sgHMISendPdo.u8WorkTimeHL = (u32HourCount >> 16) & 0xFF;
			gCanSendPdoInfo.sgHMISendPdo.u8WorkTimeHH = (u32HourCount >> 24) & 0xFF;
			__enable_irq();
		if(0 == (u32HourCount %360))//6分钟写一次
			vHourCountWrite(u32HourCount);
	}

		vEcuSetBeepPeriod();
		vCanRevPdoProc();
		vCanSendPdoProc();

		/*蜂鸣器设置*/
		u8ErrCode = u8ErrCodeGet();
		if (0 != u8ErrCode)
		{
			sgSpeakerFun.b160PerMin = 1;
		}
		else
		{
			sgSpeakerFun.b160PerMin = 0;
		}
		
		if(0 != sgSpeedRate.u8data)
			sgSpeakerFun.b130PerMin = 1;
		else
			sgSpeakerFun.b130PerMin = 0;
	}

	vWdgSetFun(WDG_USER_BIT);
}

#endif

