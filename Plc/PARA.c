/*******************************************************************************
* Filename: PARA.c	                                             	 		   *
*                                                                    		   *
* Description:											   			 		   *
* Author:                                                           		   *
* Date:     														 		   *
* Revision:															 		   *
*																	 		   *
*******************************************************************************/
#include  "PARA.h"
#include	"Device.h"
#include	"Temprature.h"
#include	"CommonRam.h"
#include	"gd32f30x_spi.h"
/******************************************************************************/
PARAMETER_STRUCT gPara;

static const PRM_DEF_STRUCT cPara0000[] = 
{
	0,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,     		//min
	0,     	//max
	0,     		//default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0001[] = 
{
	1,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.CanEn) | PRM_ATTR_POWEROFF | PRM_ATTR_TYPE_BIT_SET(0,8),
	1,     		//min
	0x7F,     	//max
	0x41,     		//default
	(void*)&gPara.CanEn,
//	"System"
//	"Bit0: 主牵引控制器",
//	"Bit1: 泵升控制器",
//	"Bit2: 转向控制器",
//	"Bit3: 从牵引控制器",
//	"Bit4: 逻辑控制器",
//	"Bit5: HMI仪表",
//	"Bit6: PC/手机",
//	"Bit7: Rev1 module act"
};
static const PRM_DEF_STRUCT cPara0002[] = 
{
	2,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.CanLockEn) | PRM_ATTR_POWEROFF | PRM_ATTR_TYPE_BIT_SET(0,8),
	1,     		//min
	0x3F,   		//max
	1,     		//default
	(void*)&gPara.CanLockEn,
//	"System"
//	"Bit0: 主牵引控制器",
//	"Bit1: 泵升控制器",
//	"Bit2: 转向控制器",
//	"Bit3: 从牵引控制器",
//	"Bit4: 逻辑控制器",
//	"Bit5: HMI仪表",
//	"Bit6: PC/手机",
//	"Bit7: Rev1 module act"
};
static const PRM_DEF_STRUCT cPara0003[] = 
{
	3,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.ConBit1) | PRM_ATTR_POWEROFF | PRM_ATTR_TYPE_BIT_SET(0,8),
	0,     		//min
	255,   		//max
	0,     		//default
	(void*)&gPara.ConBit1,
//	"System",
//	"Bit0: 编码器换向",
//	"Bit1: 电机反向",
//	"Bit2: 电机温度使能",
//	"Bit3: 位置保持使能",
//	"Bit4: Rsv",
//	"Bit5: 油门踏板使能",
//	"Bit6: 制动踏板使能",
//	"Bit7: 英制单位",
};
static const PRM_DEF_STRUCT cPara0004[] = 
{
	4,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.ConBit2) | PRM_ATTR_POWEROFF | PRM_ATTR_TYPE_BIT_SET(0,8),
	0,     		//min
	255,   		//max
	0,     		//default
	(void*)&gPara.ConBit2,
//	"System",
//	"Bit0: Rsv",
//	"Bit1: Rsv",
//	"Bit2: Rsv",
//	"Bit3: Rsv",
//	"Bit4: Throttle hpd check",
//	"Bit5: Throttle sro check",
//	"Bit6: Rsv",
//	"Bit7: Rsv",
};
static const PRM_DEF_STRUCT cPara0005[] = 
{
	5,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.CanHeartBeat),
	20,     		//min
	400,     	//max
	100,     	//default
	(void*)&gPara.CanHeartBeat,
//	"System",
//	"控制器Can通信超时时间(5ms)",
};
static const PRM_DEF_STRUCT cPara0006[] = 
{
	6,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.MotorType),
	0,     		//min
	255,     	//max
	0,     		//default
	(void*)&gPara.MotorType,
//	"Motor",
//	"Motor type. 0- user para; 1~255- Rom fix para",
};
static const PRM_DEF_STRUCT cPara0007[] = 
{
	7,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.AcMotorPloes),
	1,     		//min
	32,     	//max
	2,     		//default
	(void*)&gPara.AcMotorPloes,
//	"Motor",
//	"Motor poles num.",
};
static const PRM_DEF_STRUCT cPara0008[] = 
{
	8,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.EncoderLineNum),
	16,     		//min
	4096,     	//max
	64,     		//default
	(void*)&gPara.EncoderLineNum,
//	"Motor",
//	"Motor encoder single phase pulse number per rotation.",
};
static const PRM_DEF_STRUCT cPara0009[] = 
{
	9,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.Inertia),
	10,	    		//min
	65535,     	//max
	3666,    		//default
	(void*)&gPara.Inertia,
//	"Motor",
//	"Motor Inertia,unit 0.0001 Kgm^2",
};
static const PRM_DEF_STRUCT cPara0010[] = 
{
	10,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.StatorResist),
	1, 	    		//min
	65535,     	//max
	400,    		//default
	(void*)&gPara.StatorResist,
//	"Motor",
//	"Motor Stator Resist,unit 0.00001 ohm",
};
static const PRM_DEF_STRUCT cPara0011[] = 
{
	11,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.StatorTimeConstant),
	1, 	    		//min
	65535,     	//max
	986,    		//default
	(void*)&gPara.StatorTimeConstant,
//	"Motor",
//	"Motor Stator Time Constant,unit 0.0001S",
};
static const PRM_DEF_STRUCT cPara0012[] = 
{
	12,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.TorqueCoe1),
	1, 	    		//min
	65535,     	//max
	50,    			//default
	(void*)&gPara.TorqueCoe1,
//	"Motor",
//	"Motor torque coe,unit 0.001 A/NM",
};
static const PRM_DEF_STRUCT cPara0013[] = 
{
	13,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.RotorTimeConstant),
	1000, 	    		//min
	65535,     	//max
	10000,    			//default
	(void*)&gPara.RotorTimeConstant,
//	"Motor",
//	"Motor Rotor Time Constant,unit 0.0001S",
};
static const PRM_DEF_STRUCT cPara0014[] = 
{
	14,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.AcMotorFluxRatio),
	4096, 	    //min
	32767,     	//max
	11469,    	//default
	(void*)&gPara.AcMotorFluxRatio,
//	"Motor",
//	"Motor flux current ratio. 32767 ~ 100%",
};
static const PRM_DEF_STRUCT cPara0015[] = 
{
	15,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.AcMotorWkBaseSpdF),
	600,	 	    //min
	20000,     	//max
	4000,    		//default
	(void*)&gPara.AcMotorWkBaseSpdF,
//	"Motor",
//	"Motor Base frq for weak flux. unit 0.01Hz",
};
static const PRM_DEF_STRUCT cPara0016[] = 
{
	16,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.AcMotorTypicalSpdF),
	600,	 	    //min
	30000,     	//max
	4000,    		//default
	(void*)&gPara.AcMotorTypicalSpdF,
//	"Motor",
//	"Typical frq, as standard for acc/dec rate. uint 0.01hz",
};
static const PRM_DEF_STRUCT cPara0017[] = 
{
	17,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.AcMotorRateSpdF),
	300,	 	    //min
	15000,     	//max
	4000,    		//default
	(void*)&gPara.AcMotorRateSpdF,
//	"Motor",
//	"Base frq for current limit. uint 0.01hz",
};
static const PRM_DEF_STRUCT cPara0018[] = 
{
	18,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.AcMotorDeltaSpdF),
	100,	 	    //min
	3500,     	//max
	3500,    		//default
	(void*)&gPara.AcMotorDeltaSpdF,
//	"Motor",
//	"Delta frq uint. 0.01hz",
};
static const PRM_DEF_STRUCT cPara0019[] = 
{
	19,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.AcMotorDriveCurrentLimitRatio),
	1638,	 	    //min
	19660,     	//max
	16384,    		//default
	(void*)&gPara.AcMotorDriveCurrentLimitRatio,
//	"Motor",
//	"Drive current limit ratio of max driver current  5~100%--1638~32767",
};
static const PRM_DEF_STRUCT cPara0020[] = 
{
	20,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.AcMotorUphillCurrentLimitRatio),
	1638,	 	    //min
	19660,     	//max
	11469,    		//default
	(void*)&gPara.AcMotorUphillCurrentLimitRatio,
//	"Motor",
//	"Uphill Drive current limit ratio of max driver current  5~100%--1638~32767",
};
static const PRM_DEF_STRUCT cPara0021[] = 
{
	21,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.AcMotorRegenCurrentLimitRatio),
	1638,	 	    //min
	32767,     	//max
	11469,    		//default
	(void*)&gPara.AcMotorRegenCurrentLimitRatio,
//	"Motor",
//	"Regenerate current limit ratio of max driver current  5~100%--1638~32767",
};
static const PRM_DEF_STRUCT cPara0022[] = 
{
	22,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.AcMotorBrakeCurrentLimitRatio),
	1638,	 	    //min
	32767,     	//max
	11469,    		//default
	(void*)&gPara.AcMotorBrakeCurrentLimitRatio,
//	"Motor",
//	"Brake current limit ratio of max driver current  5~100%--1638~32767",
};
static const PRM_DEF_STRUCT cPara0023[] = 
{
	23,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.AcMotorEmrCurrentLimitRatio),
	1638,	 	    //min
	32767,     	//max
	11469,    		//default
	(void*)&gPara.AcMotorEmrCurrentLimitRatio,
//	"Motor",
//	"Emr Brake & Drive current limit ratio of max driver current  5~100%--1638~32767",
};
static const PRM_DEF_STRUCT cPara0024[] = 
{
	24,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.AcMotorDriveMapNominalRatio),
	1638,	 	    	//min
	32767,     		//max
	32767,    		//default
	(void*)&gPara.AcMotorDriveMapNominalRatio,
//	"Motor",
//	"Drive current limit  map nominal ratio 5~100%--1638~32767",
};
static const PRM_DEF_STRUCT cPara0025[] = 
{
	25,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.AcMotorDriveMapDelta1Ratio),
	1638,	 	    	//min
	32767,     		//max
	32767,    		//default
	(void*)&gPara.AcMotorDriveMapDelta1Ratio,
//	"Motor",
//	"Drive current limit  map delta1 ratio 5~100%--1638~32767",
};
static const PRM_DEF_STRUCT cPara0026[] = 
{
	26,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.AcMotorDriveMapDelta2Ratio),
	1638,	 	    	//min
	32767,     		//max
	32767,    		//default
	(void*)&gPara.AcMotorDriveMapDelta2Ratio,
//	"Motor",
//	"Drive current limit  map delta2 ratio 5~100%--1638~32767",
};
static const PRM_DEF_STRUCT cPara0027[] = 
{
	27,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.AcMotorDriveMapDelta4Ratio),
	1638,	 	    	//min
	32767,     		//max
	32767,    		//default
	(void*)&gPara.AcMotorDriveMapDelta4Ratio,
//	"Motor",
//	"Drive current limit  map delta4 ratio 5~100%--1638~32767",
};
static const PRM_DEF_STRUCT cPara0028[] = 
{
	28,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.AcMotorDriveMapDelta8Ratio),
	1638,	 	    	//min
	32767,     		//max
	32767,    		//default
	(void*)&gPara.AcMotorDriveMapDelta8Ratio,
//	"Motor",
//	"Drive current limit  map delta8 ratio 5~100%--1638~32767",
};
static const PRM_DEF_STRUCT cPara0029[] = 
{
	29,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.AcMotorRegenMapNominalRatio),
	1638,	 	    	//min
	32767,     		//max
	32767,    		//default
	(void*)&gPara.AcMotorRegenMapNominalRatio,
//	"Motor",
//	"Regenerat current limit  map nominal ratio 5~100%--1638~32767",
};
static const PRM_DEF_STRUCT cPara0030[] = 
{
	30,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.AcMotorRegenMapDelta1Ratio),
	1638,	 	    	//min
	32767,     		//max
	32767,    		//default
	(void*)&gPara.AcMotorRegenMapDelta1Ratio,
//	"Motor",
//	"Regenerat current limit  map delta1 ratio 5~100%--1638~32767",
};
static const PRM_DEF_STRUCT cPara0031[] = 
{
	31,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.AcMotorRegenMapDelta2Ratio),
	1638,	 	    	//min
	32767,     		//max
	32767,    		//default
	(void*)&gPara.AcMotorRegenMapDelta2Ratio,
//	"Motor",
//	"Regenerat current limit  map delta2 ratio 5~100%--1638~32767",
};
static const PRM_DEF_STRUCT cPara0032[] = 
{
	32,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.AcMotorRegenMapDelta4Ratio),
	1638,	 	    	//min
	32767,     		//max
	32767,    		//default
	(void*)&gPara.AcMotorRegenMapDelta4Ratio,
//	"Motor",
//	"Regenerat current limit  map delta4 ratio 5~100%--1638~32767",
};
static const PRM_DEF_STRUCT cPara0033[] = 
{
	33,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.AcMotorRegenMapDelta8Ratio),
	1638,	 	    	//min
	32767,     		//max
	32767,    		//default
	(void*)&gPara.AcMotorRegenMapDelta8Ratio,
//	"Motor",
//	"Regenerat current limit  map delta8 ratio 5~100%--1638~32767",
};
static const PRM_DEF_STRUCT cPara0034[] = 
{
	34,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.AcMotorWkPowerRatio),
	1638,	 	    	//min
	32767,     		//max
	32767,    		//default
	(void*)&gPara.AcMotorWkPowerRatio,
//	"Motor",
//	"The amount of high speed power the controller will allow 5~100%--1638~32767",
};
static const PRM_DEF_STRUCT cPara0035[] = 
{
	35,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.AcMotorWkRate),
	1638,	 	    	//min
	32767,     		//max
	32767,    		//default
	(void*)&gPara.AcMotorWkRate,
//	"Motor",
//	"The control loop gains for field weakening. 5~100%--1638~32767",
};
static const PRM_DEF_STRUCT cPara0036[] = 
{
	36,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.WkAdjRatio),
	512,	 	    	//min
	32767,     		//max
	4096,    		//default
	(void*)&gPara.WkAdjRatio,
//	"Motor",
//	"The weak flux Ratio adjustment amount from WkBaseHZ to TypicalHZ 12.5%~800%--512~32767",
};
static const PRM_DEF_STRUCT cPara0037[] = 
{
	37,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.MotorTmpSensorType),
	0,	 	    	//min
	1,     			//max
	1,    		  //default
	(void*)&gPara.MotorTmpSensorType,
//	"Motor",
//	"Sensor types predefined, 0- no sensor;1~n Valid",
};
static const PRM_DEF_STRUCT cPara0038[] = 
{
	38,
	PRM_ARRT_EPROM | PRM_ATTR_SIGNED | PRM_ATTR_SIZE(gPara.MotorTmpOfs),
	-20,	 	    	//min
	20,     			//max
	0,    		  //default
	(void*)&gPara.MotorTmpOfs,
//	"Motor",
//	"Sensor Temp Offset,-20~20,unit oC",
};
static const PRM_DEF_STRUCT cPara0039[] = 
{
	39,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.MotorCutTmp),
	25,	 	    		//min
	250,     			//max
	120,    		  //default
	(void*)&gPara.MotorCutTmp,
//	"Motor",
//	"Temperature at which drive current cutback begins.unit oC",
};
static const PRM_DEF_STRUCT cPara0040[] = 
{
	40,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.MotorMaxTmp),
	25,	 	    		//min
	250,     			//max
	140,    		  //default
	(void*)&gPara.MotorMaxTmp,
//	"Motor",
//	"Temperature at which drive current is cut back to zero.unit oC",
};
static const PRM_DEF_STRUCT cPara0041[] = 
{
	41,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.MotorTmpLosSpd),
	300,	 	    		//min
	30000,     			//max
	30000,    		  //default
	(void*)&gPara.MotorTmpLosSpd,
//	"Motor",
//	"Max speed when a Motor Temp Sensor Fault.unit 0.01Hz",
};
static const PRM_DEF_STRUCT cPara0042[] = 
{
	42,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,     			//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0043[] = 
{
	43,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,     			//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0044[] = 
{
	44,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,     			//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0045[] = 
{
	45,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,     			//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0046[] = 
{
	46,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,     			//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0047[] = 
{
	47,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,     			//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
/* Speed */
static const PRM_DEF_STRUCT cPara0048[] = 
{
	48,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.FwdMaxSpd),
	300,	 	    		//min
	30000,     			//max
	10000,    		  //default
	(void*)&gPara.FwdMaxSpd,
//	"Speed",
//	"Max forward speed.unit 0.01Hz",
};
static const PRM_DEF_STRUCT cPara0049[] = 
{
	49,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.RvsMaxSpd),
	300,	 	    		//min
	30000,     			//max
	10000,    		  //default
	(void*)&gPara.RvsMaxSpd,
//	"Speed",
//	"Max reverse speed at full throttle.unit 0.01Hz",
};


static const PRM_DEF_STRUCT cPara0050[] = 
{
	50,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.IntertiaRatio),
	100,	 	    		//min
	1200,     			//max
	100,    		  //default
	(void*)&gPara.IntertiaRatio,
//	"Speed",
//	"Vehicle Intertia Ratio. unit 1%",
};
static const PRM_DEF_STRUCT cPara0051[] = 
{
	51,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.SpeedWc),
	5,	 	    		//min
	200,     			//max
	60,    		  //default
	(void*)&gPara.SpeedWc,
//	"Speed",
//	"速度环截止频率(Hz)",
};
static const PRM_DEF_STRUCT cPara0052[] = 
{
	52,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.SpeedWit),
	5,	 	    		//min
	200,     			//max
	30,    		  //default
	(void*)&gPara.SpeedWit,
//	"Speed",
//	"速度环积分时间(ms)",
};
static const PRM_DEF_STRUCT cPara0053[] = 
{
	53,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.TorFilt1W),
	10,	 	    		//min
	1600,     			//max
	75,    		  //default
	(void*)&gPara.TorFilt1W,
//	"Speed",
//	"一阶转矩滤波器截止频率(Hz)",
};
static const PRM_DEF_STRUCT cPara0054[] = 
{
	54,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.TorFilt2W),
	10,	 	    		//min
	1600,     			//max
	75,    		  //default
	(void*)&gPara.TorFilt2W,
//	"Speed",
//	"二阶转矩滤波器截止频率(Hz)",
};
static const PRM_DEF_STRUCT cPara0055[] = 
{
	55,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.TorFilt2Q),
	0,	 	    		//min
	100,     			//max
	71,    		  //default
	(void*)&gPara.TorFilt2Q,
//	"Speed",
//	"二阶转矩滤波器品质因数(%)",
};
static const PRM_DEF_STRUCT cPara0056[] = 
{
	56,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.KvffCurrent),
	0,	 	    		//min
	5000,     			//max
	25,    		  //default
	(void*)&gPara.KvffCurrent,
//	"Speed",
//	"V forward control 0~500A -- 0~5000",
};
static const PRM_DEF_STRUCT cPara0057[] = 
{
	57,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.KvffBuildTime),
	100,	 	    		//min
	5000,     			//max
	600,    		  //default
	(void*)&gPara.KvffBuildTime,
//	"Speed",
//	"Kvff build up time. unit ms",
};
static const PRM_DEF_STRUCT cPara0058[] = 
{
	58,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.KvffReleaseTime),
	100,	 	    		//min
	2000,     			//max
	500,    		  //default
	(void*)&gPara.KvffReleaseTime,
//	"Speed",
//	"Kvff release time. unit ms",
};
static const PRM_DEF_STRUCT cPara0059[] = 
{
	59,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.KaccCurrent),
	0,	 	    		//min
	5000,     			//max
	0,    		  //default
	(void*)&gPara.KaccCurrent,
//	"Speed",
//	"Acc forward control 0~500A -- 0~5000",
};
static const PRM_DEF_STRUCT cPara0060[] = 
{
	60,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.KbkeCurrent),
	0,	 	    		//min
	5000,     			//max
	0,    		  //default
	(void*)&gPara.KbkeCurrent,
//	"Speed",
//	"Brake forward control 0~500A -- 0~5000",
};
static const PRM_DEF_STRUCT cPara0061[] = 
{
	61,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.KaccBuildTime),
	100,	 	    		//min
	5000,     			//max
	1000,    		  //default
	(void*)&gPara.KaccBuildTime,
//	"Speed",
//	"Kacc build up time 0.1~5.0 sec -- 100~5000",
};
static const PRM_DEF_STRUCT cPara0062[] = 
{
	62,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.KaccReleaseTime),
	100,	 	    		//min
	2000,     			//max
	500,    		  //default
	(void*)&gPara.KaccReleaseTime,
//	"Speed",
//	"Kacc release time  0.1~2.0 sec -- 100~2000",
};
static const PRM_DEF_STRUCT cPara0063[] = 
{
	63,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.FullAccTimeHs),
	100,	 	    		//min
	30000,     			//max
	200,    		  //default
	(void*)&gPara.FullAccTimeHs,
//	"Speed",
//	"Acc time for full throttle at high speed  0.1~30.0 sec -- 100~30000",
};
static const PRM_DEF_STRUCT cPara0064[] = 
{
	64,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.FullAccTimeLs),
	100,	 	    		//min
	30000,     			//max
	8000,    		  //default
	(void*)&gPara.FullAccTimeLs,
//	"Speed",
//	"Acc time for full throttle at low speed  0.1~30.0 sec -- 100~30000",
};
static const PRM_DEF_STRUCT cPara0065[] = 
{
	65,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.LowAccTime),
	100,	 	    		//min
	30000,     			//max
	12000,    		  //default
	(void*)&gPara.LowAccTime,
//	"Speed",
//	"Acc time for small amount throttle  0.1~30.0 sec -- 100~30000",
};
static const PRM_DEF_STRUCT cPara0066[] = 
{
	66,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.NeutralDecTimeHs),
	100,	 	    		//min
	30000,     			//max
	6000,    		  //default
	(void*)&gPara.NeutralDecTimeHs,
//	"Speed",
//	"Dec time for neutral throttle at high speed  0.1~30.0 sec -- 100~30000",
};
static const PRM_DEF_STRUCT cPara0067[] = 
{
	67,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.NeutralDecTimeLs),
	100,	 	    		//min
	30000,     			//max
	8000,    		  //default
	(void*)&gPara.NeutralDecTimeLs,
//	"Speed",
//	"Dec time for neutral throttle at low speed  0.1~30.0 sec -- 100~30000",
};
static const PRM_DEF_STRUCT cPara0068[] = 
{
	68,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.FullBrakeDecTimeHs),
	100,	 	    		//min
	30000,     			//max
	3000,    		  //default
	(void*)&gPara.FullBrakeDecTimeHs,
//	"Speed",
//	"Dec time for full brake or full neg throttle at high speed  0.1~30.0 sec -- 100~30000",
};
static const PRM_DEF_STRUCT cPara0069[] = 
{
	69,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.FullBrakeDecTimeLs),
	100,	 	    		//min
	30000,     			//max
	4000,    		  //default
	(void*)&gPara.FullBrakeDecTimeLs,
//	"Speed",
//	"Dec time for full brake or full neg throttle at low speed  0.1~30.0 sec -- 100~30000",
};
static const PRM_DEF_STRUCT cPara0070[] = 
{
	70,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.LowBrakeDecTimeHLs),
	100,	 	    		//min
	30000,     			//max
	10000,    		  //default
	(void*)&gPara.LowBrakeDecTimeHLs,
//	"Speed",
//	"Dec time for small amount Brake or small amount neg throttle 0.1~30.0 sec -- 100~30000",
};
static const PRM_DEF_STRUCT cPara0071[] = 
{
	71,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.PartialDecTimeHLs),
	100,	 	    		//min
	30000,     			//max
	12000,    		  //default
	(void*)&gPara.PartialDecTimeHLs,
//	"Speed",
//	"Dec time for throttle up  neutral 0.1~30.0 sec -- 100~30000",
};
static const PRM_DEF_STRUCT cPara0072[] = 
{
	72,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.HighSpeedRatio),
	0,	 	    		//min
	32767,     			//max
	22938,    		  //default
	(void*)&gPara.HighSpeedRatio,
//	"Speed",
//	"Sets Hs percentage of the Typical Max Speed 0~100%--0~32767",
};
static const PRM_DEF_STRUCT cPara0073[] = 
{
	73,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.LowSpeedRatio),
	0,	 	    		//min
	32767,     			//max
	9830,    		  //default
	(void*)&gPara.LowSpeedRatio,
//	"Speed",
//	"Sets Ls percentage of the Typical Max Speed 0~100%--0~32767",
};
static const PRM_DEF_STRUCT cPara0074[] = 
{
	74,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.ReversalSoften),
	0,	 	    		//min
	32767,     			//max
	32767,    		  //default
	(void*)&gPara.ReversalSoften,
//	"Speed",
//	"Larger values create a softer reversal from regen braking to drive when near zero speed.",
};
static const PRM_DEF_STRUCT cPara0075[] = 
{
	75,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.MaxSpdAccTime),
	100,	 	    		//min
	30000,     			//max
	500,    		  //default
	(void*)&gPara.MaxSpdAccTime,
//	"Speed",
//	"Acc time for maxspd vary.  0.1~30.0 sec -- 100~30000",
};
static const PRM_DEF_STRUCT cPara0076[] = 
{
	76,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.MaxSpdDecTime),
	100,	 	    		//min
	30000,     			//max
	500,    		  //default
	(void*)&gPara.MaxSpdDecTime,
//	"Speed",
//	"Dec time for maxspd vary.  0.1~30.0 sec -- 100~30000",
};
static const PRM_DEF_STRUCT cPara0077[] = 
{
	77,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.SpeedRate1),
	0,	 	    		//min
	100,     			//max
	80,    		  //default
	(void*)&gPara.SpeedRate1,
//	"Speed",
//	"Sets gear 1 percentage of the Typical Max Speed 0~100%--0~100",
};
static const PRM_DEF_STRUCT cPara0078[] = 
{
	78,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.SpeedRate2),
	0,	 	    		//min
	100,     			//max
	100,    		  //default
	(void*)&gPara.SpeedRate2,
//	"Speed",
//	"Sets gear 2 percentage of the Typical Max Speed 0~100%--0~100",
};
static const PRM_DEF_STRUCT cPara0079[] = 
{
	79,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.SpeedRate3),
	0,	 	    		//min
	100,     			//max
	25,    		  //default
	(void*)&gPara.SpeedRate3,
//	"Speed",
//	"Sets gear 3 percentage of the Typical Max Speed 0~100%--0~100",
};
static const PRM_DEF_STRUCT cPara0080[] = 
{
	80,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.SpeedRate4),
	0,	 	    		//min
	100,     			//max
	60,    		  //default
	(void*)&gPara.SpeedRate4,
//	"Speed",
//	"Sets gear 4 percentage of the Typical Max Speed 0~100%--0~100",
};
	/* Restraint */
static const PRM_DEF_STRUCT cPara0081[] = 
{
	81,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.SoftStopSpeed),
	0,	 	    		//min
	2000,     			//max
	700,    		  //default
	(void*)&gPara.SoftStopSpeed,
//	"Speed",
//	"Defines the speed below which a much slower decel rate is used.unit 0.01Hz",
};
static const PRM_DEF_STRUCT cPara0082[] = 
{
	82,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.EntryRateStop),
	5,	 	    		//min
	100,     			//max
	25,    		  //default
	(void*)&gPara.EntryRateStop,
//	"Speed",
//	"Defines the dec rate when speed below SoftStopSpeed. unit 1%",
};
static const PRM_DEF_STRUCT cPara0083[] = 
{
	83,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.SetSpeedThreshold),
	5,	 	    		//min
	100,     			//max
	8,    		  //default
	(void*)&gPara.SetSpeedThreshold,
//	"Speed",
//	"Determines the speed below which the EM brake will be commanded to set.unit rpm",
};
static const PRM_DEF_STRUCT cPara0084[] = 
{
	84,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.SetSpeedSettleTime),
	0,	 	    		//min
	5000,     			//max
	5000,    		  //default
	(void*)&gPara.SetSpeedSettleTime,
//	"Speed",
//	"Determines how long the position hold function is allowed to operate before the EM brake is set.unit ms",
};
static const PRM_DEF_STRUCT cPara0085[] = 
{
	85,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.StopMode),
	1,	 	    		//min
	3,     			//max
	3,    		  //default
	(void*)&gPara.StopMode,
//	"Speed",
//	"Ramp stop mode: 1-stop,2- move~stop~move~;3- stop-move~~",
};
static const PRM_DEF_STRUCT cPara0086[] = 
{
	86,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.EntryRateApproach),
	5,	 	    		//min
	100,     			//max
	25,    		  //default
	(void*)&gPara.EntryRateApproach,
//	"Speed",
//	"Defines the Acc/dec rate when speed error below SoftStopSpeed",
};
static const PRM_DEF_STRUCT cPara0087[] = 
{
	87,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.SpdAccRatioFast),
	512,	 	    		//min
	32767,     			//max
	4096,    		  //default
	(void*)&gPara.SpdAccRatioFast,
//	"Speed",
//	"The acc ratio for Fast Mode  12.5%~800% -- 512~32767",
};
static const PRM_DEF_STRUCT cPara0088[] = 
{
	88,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.SpdDecRatioFast),
	512,	 	    		//min
	32767,     			//max
	4096,    		  //default
	(void*)&gPara.SpdDecRatioFast,
//	"Speed",
//	"The dec ratio for Fast Mode  12.5%~800% -- 512~32767",
};
static const PRM_DEF_STRUCT cPara0089[] = 
{
	89,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.SpdAccRatioSlow),
	512,	 	    		//min
	32767,     			//max
	4096,    		  //default
	(void*)&gPara.SpdAccRatioSlow,
//	"Speed",
//	"The acc ratio for Slow Mode  12.5%~800% -- 512~32767",
};
static const PRM_DEF_STRUCT cPara0090[] = 
{
	90,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.SpdDecRatioSlow),
	512,	 	    		//min
	32767,     			//max
	4096,    		  //default
	(void*)&gPara.SpdDecRatioSlow,
//	"Speed",
//	"The dec ratio for Slow Mode  12.5%~800% -- 512~32767",
};

	/* Throttle */
static const PRM_DEF_STRUCT cPara0091[] = 
{
	91,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.ThrottleType),
	0,	 	    		//min
	4,     			//max
	0,    		  //default
	(void*)&gPara.ThrottleType,
//	"Throttle",
//	"油门踏板速度信号输入类型：0: 电压输入; 1: 2线电阻输入; 2: 3线电阻输入; 3: 逻辑程序输入; 4: 0~10V输入;",
};
static const PRM_DEF_STRUCT cPara0092[] = 
{
	92,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.ThrottleMinVoltage),
	0,	 	    		//min
	500,     			//max
	5,    		  //default
	(void*)&gPara.ThrottleMinVoltage,
//	"Throttle",
//	"油门踏板死区最小值. unit 0.1v",
};
static const PRM_DEF_STRUCT cPara0093[] = 
{
	93,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.ThrottleMaxVoltage),
	0,	 	    		//min
	500,     			//max
	45,    		  //default
	(void*)&gPara.ThrottleMaxVoltage,
//	"Throttle",
//	"油门踏板死区最大值. unit 0.1v",
};
static const PRM_DEF_STRUCT cPara0094[] = 
{
	94,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.ThrottleMap),
	0,	 	    		//min
	500,     			//max
	50,    		  //default
	(void*)&gPara.ThrottleMap,
//	"Throttle",
//	"The percentage of spd output at half throttle position.unit 1%",
};
static const PRM_DEF_STRUCT cPara0095[] = 
{
	95,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.ThrottleOfs),
	0,	 	    		//min
	32767,     			//max
	0,    		  //default
	(void*)&gPara.ThrottleOfs,
//	"Throttle",
//	"Initial offset to increase start act.unit 32767~100%",
};

static const PRM_DEF_STRUCT cPara0096[] = 
{
	96,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.ThrottleMinVoltageRvs),
	0,	 	    		//min
	500,     			//max
	5,    		  //default
	(void*)&gPara.ThrottleMinVoltageRvs,
//	"Throttle",
//	"油门踏板反向死区最小值. unit 0.1v",
};
static const PRM_DEF_STRUCT cPara0097[] = 
{
	97,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.ThrottleMaxVoltageRvs),
	0,	 	    		//min
	500,     			//max
	45,    		  //default
	(void*)&gPara.ThrottleMaxVoltageRvs,
//	"Throttle",
//	"油门踏板反向死区最大值. unit 0.1v",
};
static const PRM_DEF_STRUCT cPara0098[] = 
{
	98,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.ThrottleMapRvs),
	0,	 	    		//min
	500,     			//max
	50,    		  //default
	(void*)&gPara.ThrottleMapRvs,
//	"Throttle",
//	"The percentage of spd output at half throttle position.1%",
};
static const PRM_DEF_STRUCT cPara0099[] = 
{
	99,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.ThrottleOfsRvs),
	0,	 	    		//min
	32767,     			//max
	0,    		  //default
	(void*)&gPara.ThrottleOfsRvs,
//	"Throttle",
//	"Initial offset to increase start act.unit 32767~100%",
};
static const PRM_DEF_STRUCT cPara0100[] = 
{
	100,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.SequenceDelay),
	0,	 	    		//min
	250,     			//max
	10,    		  //default
	(void*)&gPara.SequenceDelay,
//	"Throttle",
//	"Interlock switch delay.0~250 -- 0~5s ",
};
	/* BrakePedal */
static const PRM_DEF_STRUCT cPara0101[] = 
{
	101,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.BrakePedalType),
	0,	 	    		//min
	4,     			//max
	0,    		  //default
	(void*)&gPara.BrakePedalType,
//	"BrakePedal",
//	"脚踏板信号输入类型：0：电压输入；1：2线电阻输入；2；3线电阻输入；3：逻辑程序输入",
};
static const PRM_DEF_STRUCT cPara0102[] = 
{
	102,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.BrakePedalMinVoltage),
	0,	 	    		//min
	150,     			//max
	5,    		  //default
	(void*)&gPara.BrakePedalMinVoltage,
//	"BrakePedal",
//	"踏板死区最小值. unit 0.1v",
};
static const PRM_DEF_STRUCT cPara0103[] = 
{
	103,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.BrakePedalMaxVoltage),
	0,	 	    		//min
	150,     			//max
	100,    		  //default
	(void*)&gPara.BrakePedalMaxVoltage,
//	"BrakePedal",
//	"踏板死区最大值. unit 0.1v",
};
static const PRM_DEF_STRUCT cPara0104[] = 
{
	104,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.BrakePedalMap),
	0,	 	    		//min
	100,     			//max
	50,    		  //default
	(void*)&gPara.BrakePedalMap,
//	"BrakePedal",
//	"The percentage of brake output at half pedal position.1%",
};
static const PRM_DEF_STRUCT cPara0105[] = 
{
	105,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.BrakePedalOfs),
	0,	 	    		//min
	32767,     			//max
	0,    		  //default
	(void*)&gPara.BrakePedalOfs,
//	"BrakePedal",
//	"Initial offset to increase brake start act.unit 32767~100%",
};
	/* Speed */
static const PRM_DEF_STRUCT cPara0106[] = 
{
	106,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.SmallSpeedRatio),
	1638,	 	    		//min
	32767,     			//max
	4260,    		  //default 13%
	(void*)&gPara.SmallSpeedRatio,
//	"Speed",
//	"Sets Small percentage of the Typical Max Speed 5~100%--1638~32767",
};
	/* SecondMove */
static const PRM_DEF_STRUCT cPara0107[] = 
{
	107,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,     			//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"SecondMove",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0108[] = 
{
	108,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,     			//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"SecondMove",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0109[] = 
{
	109,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,     			//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"SecondMove",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0110[] = 
{
	110,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,     			//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"SecondMove",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0111[] = 
{
	111,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,     			//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"SecondMove",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0112[] = 
{
	112,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,     			//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"SecondMove",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0113[] = 
{
	113,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,     			//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"SecondMove",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0114[] = 
{
	114,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,     			//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"SecondMove",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0115[] = 
{
	115,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,     			//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"SecondMove",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0116[] = 
{
	116,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,     			//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"SecondMove",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0117[] = 
{
	117,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,     			//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"SecondMove",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0118[] = 
{
	118,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,     			//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"SecondMove",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0119[] = 
{
	119,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,     			//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"SecondMove",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0120[] = 
{
	120,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,     			//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"SecondMove",
//	"Rsv",
};

	/* Vehicle */
	//Metric Units ConBit1:bit7
static const PRM_DEF_STRUCT cPara0121[] = 
{
	121,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.SpdPerKmRatio),
	100,	 	    		//min
	32767,     			//max
	100,    		  //default
	(void*)&gPara.SpdPerKmRatio,
//	"Vehicle",
//	"Conversion factor that scales motor speed to vehicle speed.",
};
static const PRM_DEF_STRUCT cPara0122[] = 
{
	122,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.CapSpd1),
	0,	 	    		//min
	30000,     			//max
	0,    		  //default
	(void*)&gPara.CapSpd1,
//	"Vehicle",
//	"The motor speed to capture TimetoSpd1.",
};
static const PRM_DEF_STRUCT cPara0123[] = 
{
	123,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.CapSpd2),
	0,	 	    		//min
	30000,     			//max
	0,    		  //default
	(void*)&gPara.CapSpd2,
//	"Vehicle",
//	"The motor speed to capture TimetoSpd2.",
};
static const PRM_DEF_STRUCT cPara0124[] = 
{
	124,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.CapDist1),
	1,	 	    		//min
	3281,    			//max
	30,    		  //default
	(void*)&gPara.CapDist1,
//	"Vehicle",
//	"The Distance to capture TimetoDist1. unit meter/feet",
};
static const PRM_DEF_STRUCT cPara0125[] = 
{
	125,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.CapDist2),
	1,	 	    		//min
	3281,    			//max
	30,    		  //default
	(void*)&gPara.CapDist2,
//	"Vehicle",
//	"The Distance to capture TimetoDist2. unit meter/feet",
};
static const PRM_DEF_STRUCT cPara0126[] = 
{
	126,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.CapDist3),
	1,	 	    		//min
	3281,    			//max
	30,    		  //default
	(void*)&gPara.CapDist3,
//	"Vehicle",
//	"The Distance to capture TimetoDist3. unit meter/feet",
};
static const PRM_DEF_STRUCT cPara0127[] = 
{
	127,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.WeihuPeriod),
	40,	 	    		//min
	65535,    			//max
	200,    		  //default
	(void*)&gPara.WeihuPeriod,
//	"Vehicle",
//	"Wei hu period. Unit 1Hour",
};
static const PRM_DEF_STRUCT cPara0128[] = 
{
	128,
	PRM_ATTR_SIZE(gPara.RemoteBit),
	0,	 	    		//min
	65535,    			//max
	0,    		  //default
	(void*)&gPara.RemoteBit,
//	"Vehicle",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0129[] = 
{
	129,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    			//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Vehicle",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0130[] = 
{
	130,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    			//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Vehicle",
//	"Rsv",
};	
	/* Driver */
static const PRM_DEF_STRUCT cPara0131[] = 
{
	131,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.DriverEn1) | PRM_ATTR_POWEROFF | PRM_ATTR_TYPE_BIT_SET(0,8),
	0,     		//min
	255,   		//max
	1,     		//default
	(void*)&gPara.DriverEn1,
//	"Driver"
//	"Bit0: MainEnable",
//	"Bit1: MainCheckEnable",
//	"Bit2: InterlockEnable",
//	"Bit3: BrakeEnable",
//	"Bit4: BrakeTrqPreload",
//	"Bit5: BrakeSetOnFault",
//	"Bit6: PdEnable",
//	"Bit7: PdHydLowerEnable",
};
static const PRM_DEF_STRUCT cPara0132[] = 
{
	132,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.DriverEn2) | PRM_ATTR_POWEROFF | PRM_ATTR_TYPE_BIT_SET(0,8),
	0,     		//min
	255,   		//max
	0,     		//default
	(void*)&gPara.DriverEn2,
//	"Driver"
//	"Bit0: Drive1Enable",
//	"Bit1: Drive2Enable",
//	"Bit2: Drive3Enable",
//	"Bit3: Drive4Enable",
//	"Bit4: PdCheckEnable",
//	"Bit5: SWI1Type SWI1输入类型设置：0：数字输入；1：模拟输入",
//	"Bit6: EmrEnable",
//	"Bit7: EmrDirLock",
};
	/* Main Contactor */
static const PRM_DEF_STRUCT cPara0133[] = 
{
	133,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.MainPullVoltage),
	0,	 	    		//min
	100,    			//max
	100,    		  //default
	(void*)&gPara.MainPullVoltage,
//	"Driver",
//	"Initial voltage when the contactor coil first turns on,1~100-- 1~100%",
};
static const PRM_DEF_STRUCT cPara0134[] = 
{
	134,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.MainHoldVoltage),
	0,	 	    		//min
	100,    			//max
	80,    		  //default
	(void*)&gPara.MainHoldVoltage,
//	"Driver",
//	"Reduced average voltage to be applied to the contactor coil once it has closed.1~100-- 1~100%",
};
static const PRM_DEF_STRUCT cPara0135[] = 
{
	135,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.MainOpenDelay),
	0,	 	    		//min
	5000,    			//max
	0,    		  //default
	(void*)&gPara.MainOpenDelay,
//	"Driver",
//	"Delay time after the interlock lost.0~50s",
};
static const PRM_DEF_STRUCT cPara0136[] = 
{
	136,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.MainDncThreshold),
	1,	 	    		//min
	8000,    			//max
	1,    		  //default
	(void*)&gPara.MainDncThreshold,
//	"Driver",
//	"Voltage for detecting the main drive do not closed.unit 0.1V",
};
	/* Drive3 */
static const PRM_DEF_STRUCT cPara0137[] = 
{
	137,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.Drive3PullVoltage),
	0,	 	    		//min
	100,    			//max
	100,    		  //default
	(void*)&gPara.Drive3PullVoltage,
//	"Driver",
//	"Initial voltage when the contactor coil first turns on,1~100-- 1~100%",
};
static const PRM_DEF_STRUCT cPara0138[] = 
{
	138,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.Drive3HoldVoltage),
	0,	 	    		//min
	100,    			//max
	80,    		  //default
	(void*)&gPara.Drive3HoldVoltage,
//	"Driver",
//	"Reduced average voltage to be applied to the contactor coil once it has closed.1~100-- 1~100%",
};
	/* Drive4 */
static const PRM_DEF_STRUCT cPara0139[] = 
{
	139,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.Drive4PullVoltage),
	0,	 	    		//min
	100,    			//max
	100,    		  //default
	(void*)&gPara.Drive4PullVoltage,
//	"Driver",
//	"Initial voltage when the contactor coil first turns on,1~100-- 1~100%",
};
static const PRM_DEF_STRUCT cPara0140[] = 
{
	140,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.Drive4HoldVoltage),
	0,	 	    		//min
	100,    			//max
	80,    		  //default
	(void*)&gPara.Drive4HoldVoltage,
//	"Driver",
//	"Reduced average voltage to be applied to the contactor coil once it has closed.1~100-- 1~100%",
};

	/* EmBrake */
static const PRM_DEF_STRUCT cPara0141[] = 
{
	141,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.EmBrkType),
	0,	 	    		//min
	2,    			//max
	0,    		  //default
	(void*)&gPara.EmBrkType,
//	"EmBrake",
//	"EM brake type",
};
static const PRM_DEF_STRUCT cPara0142[] = 
{
	142,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.EmBrkPullVoltage),
	0,	 	    		//min
	100,    			//max
	100,    		  //default
	(void*)&gPara.EmBrkPullVoltage,
//	"EmBrake",
//	"Initial voltage when the contactor coil first turns on,1~100-- 1~100%",
};
static const PRM_DEF_STRUCT cPara0143[] = 
{
	143,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.EmBrkHoldVoltage),
	0,	 	    		//min
	100,    			//max
	80,    		  //default
	(void*)&gPara.EmBrkHoldVoltage,
//	"EmBrake",
//	"Reduced average voltage to be applied to the contactor coil once it has closed.1~100-- 1~100%",
};
static const PRM_DEF_STRUCT cPara0144[] = 
{
	144,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.TrqPreloadDelay),
	0,	 	    		//min
	100,    			//max
	0,    		  //default
	(void*)&gPara.TrqPreloadDelay,
//	"EmBrake",
//	"Torque Preload Delay.unit 8ms",
};
static const PRM_DEF_STRUCT cPara0145[] = 
{
	145,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.EmBrkReleaseDelay),
	5,	 	    		//min
	250,    			//max
	5,    		  //default
	(void*)&gPara.EmBrkReleaseDelay,
//	"EmBrake",
//	"Time for the EM brake to physically release after the pull-in voltage is applied.unit 8ms",
};
static const PRM_DEF_STRUCT cPara0146[] = 
{
	146,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.TrqPreloadCancelDelay),
	0,	 	    		//min
	15000,    		//max
	0,    		  //default
	(void*)&gPara.TrqPreloadCancelDelay,
//	"EmBrake",
//	"Torque Preload Cancel Delay.unit 8ms",
};
static const PRM_DEF_STRUCT cPara0147[] = 
{
	147,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.PumpDriveCurrentLimitRatio),
	1638,	 	    	//min
	32767,     		//max
	20000,    		//default
	(void*)&gPara.PumpDriveCurrentLimitRatio,
//	"Propdriver",
//	"Pump current limit  map nominal ratio 5~100%--1638~32767",
};
static const PRM_DEF_STRUCT cPara0148[] = 
{
	148,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0149[] = 
{
	149,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0150[] = 
{
	150,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.PumpSpdAccRatio),
	5,	 	    	//min
	30000,     		//max
	200,    		//default
	(void*)&gPara.PumpSpdAccRatio,
//	"Propdriver",
//	"Pump motor acc time",
};

	/* Propdriver */
static const PRM_DEF_STRUCT cPara0151[] = 
{
	151,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.PropDKp),
	328,	 	    		//min
	32767,    		//max
	8192,    		  //default
	(void*)&gPara.PropDKp,
//	"Propdriver",
//	"Propdriver current loop Kp gain 1%~100%--328~32767",
};
static const PRM_DEF_STRUCT cPara0152[] = 
{
	152,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.PropDKi),
	82,	 	    		//min
	8192,    		//max
	2048,    		  //default
	(void*)&gPara.PropDKi,
//	"Propdriver",
//	"Propdriver current loop Ki gain 1%~100%--82~8192.",
};
static const PRM_DEF_STRUCT cPara0153[] = 
{
	153,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.PropDMaxCurrent),
	0,	 	    		//min
	2000,    		//max
	0,    		  //default
	(void*)&gPara.PropDMaxCurrent,
//	"Propdriver",
//	"Propdriver max current 0~2.0A -- 0~2000.",
};
static const PRM_DEF_STRUCT cPara0154[] = 
{
	154,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.PropDMinCurrent),
	0,	 	    		//min
	2000,    		//max
	0,    		  //default
	(void*)&gPara.PropDMinCurrent,
//	"Propdriver",
//	"Propdriver min current 0~2.0A -- 0~2000.",
};
static const PRM_DEF_STRUCT cPara0155[] = 
{
	155,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.PropDDitherPeriod),
	1,	 	    		//min
	8,    		//max
	8,    		  //default
	(void*)&gPara.PropDDitherPeriod,
//	"Propdriver",
//	"Propdriver dither period 15ms~120ms--1~8",
};
static const PRM_DEF_STRUCT cPara0156[] = 
{
	156,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.PropDDitherRatio),
	0,	 	    		//min
	32767,    		//max
	0,    		  //default
	(void*)&gPara.PropDDitherRatio,
//	"Propdriver",
//	"Propdriver dither current ratio.(DitherCurrent/PropDMaxCurrent) 0~100% -- 0~32767",
};
static const PRM_DEF_STRUCT cPara0157[] = 
{
	157,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.PropValveResistance),
	10,	 	    		//min
	10000,    		//max
	250,    		  //default
	(void*)&gPara.PropValveResistance,
//	"Propdriver",
//	"Prop valve coil resistance. 1~1000 ohm -- 10~10000.",
};
	/* Emreverse */
static const PRM_DEF_STRUCT cPara0158[] = 
{
	158,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.EmrTimeLimit),
	0,	 	    		//min
	30001,    		//max
	0,    		  //default
	(void*)&gPara.EmrTimeLimit,
//	"Emreverse",
//	"Time limit after the vehicle is moving in the reverse direction.0~30000--0~30s",
};
static const PRM_DEF_STRUCT cPara0159[] = 
{
	159,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.EmrSpeed),
	600,	 	    		//min
	20000,    		//max
	5000,    		  //default
	(void*)&gPara.EmrSpeed,
//	"Emreverse",
//	"The maximum reverse speed when emergency reverse is active. 0.01Hz",
};
static const PRM_DEF_STRUCT cPara0160[] = 
{
	160,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.EmrAccTime),
	100,	 	    		//min
	3000,    		//max
	3000,    		  //default
	(void*)&gPara.EmrAccTime,
//	"Emreverse",
//	"Rate for reverse acc 100~3000 -- 0.1~3 s",
};
static const PRM_DEF_STRUCT cPara0161[] = 
{
	161,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.EmrDecTime),
	100,	 	    		//min
	3000,    				//max
	1000,    		  	//default
	(void*)&gPara.EmrDecTime,
//	"Emreverse",
//	"Rate for dec forward move to stop 100~3000 -- 0.1~3 s",
};
static const PRM_DEF_STRUCT cPara0162[] = 
{
	162,
	PRM_ATTR_SIZE(gPara.Buadrate),
	0,	 	    		//min
	3,    		//max
	0,    		  //default
	(void*)&gPara.Buadrate,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0163[] = 
{
	163,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0164[] = 
{
	164,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0165[] = 
{
	165,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0166[] = 
{
	166,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0167[] = 
{
	167,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0168[] = 
{
	168,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0169[] = 
{
	169,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0170[] = 
{
	170,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0171[] = 
{
	171,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0172[] = 
{
	172,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0173[] = 
{
	173,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0174[] = 
{
	174,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0175[] = 
{
	175,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0176[] = 
{
	176,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0177[] = 
{
	177,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0178[] = 
{
	178,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0179[] = 
{
	179,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0180[] = 
{
	180,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0181[] = 
{
	181,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0182[] = 
{
	182,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0183[] = 
{
	183,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0184[] = 
{
	184,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0185[] = 
{
	185,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0186[] = 
{
	186,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0187[] = 
{
	187,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0188[] = 
{
	188,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0189[] = 
{
	189,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
/* Steer */
static const PRM_DEF_STRUCT cPara0190[] = 
{
	190,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.SteerBit) | PRM_ATTR_TYPE_BIT_SET(0,8),
	0,	 	    		//min
	255,    		//max
	0x04,    		  //default
	(void*)&gPara.SteerBit,
//	"Steer",
//	"Bit0: FXP rotation direction",
//	"Bit1: Middle pos return direction",
//	"Bit2: Middle return action each time SVON",
//	"Bit3: Rsv",
//	"Bit4: Rsv",
//	"Bit5: Rsv",
//	"Bit6: Rsv",
//	"Bit7: Rsv"
};
static const PRM_DEF_STRUCT cPara0191[] = 
{
	191,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.PosRefFiltT),
	0,	 	    		//min
	1000,    		//max
	200,    		  //default
	(void*)&gPara.PosRefFiltT,
//	"Steer",
//	"位置环指令平滑时间ms",
};
static const PRM_DEF_STRUCT cPara0192[] = 
{
	192,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.PosKp),
	1,	 	    		//min
	200,    		//max
	40,    		  //default
	(void*)&gPara.PosKp,
//	"Steer",
//	"位置环增益(rad/s)",
};
static const PRM_DEF_STRUCT cPara0193[] = 
{
	193,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.PosGearNum),
	1,	 	    		//min
	32767,    		//max
	220,    		  //default
	(void*)&gPara.PosGearNum,
//	"Steer",
//	"齿轮比分子",
};
static const PRM_DEF_STRUCT cPara0194[] = 
{
	194,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.PosGearDen),
	1,	 	    		//min
	32767,    		//max
	1,    		  //default
	(void*)&gPara.PosGearDen,
//	"Steer",
//	"齿轮比分母",
};
static const PRM_DEF_STRUCT cPara0195[] = 
{
	195,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.PosCmdFor90Degree),
	4,	 	    	//min
	8192,    	//max
	(32*4*3),    		  //default
	(void*)&gPara.PosCmdFor90Degree,
//	"Steer",
//	"Cmd pulse number(M4) required for steer wheel rotate 90 degree.",
};
static const PRM_DEF_STRUCT cPara0196[] = 
{
	196,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.PosCmdLimitCw),
	0,	 	    	//min
	(8192*2),    	//max
	(32*4*3),   //default
	(void*)&gPara.PosCmdLimitCw,
//	"Steer",
//	"Cw 转向范围限制",
};
static const PRM_DEF_STRUCT cPara0197[] = 
{
	197,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.PosCmdLimitCcw),
	0,	 	    	//min
	(8192*2),    	//max
	(32*4*3),    		  //default
	(void*)&gPara.PosCmdLimitCcw,
//	"Steer",
//	"Ccw 转向范围限制",
};
static const PRM_DEF_STRUCT cPara0198[] = 
{
	198,
	PRM_ARRT_EPROM | PRM_ATTR_SIGNED | PRM_ATTR_SIZE(gPara.MiddleOfs),
	-(8192*2),	 	    	//min
	(8192*2),    	//max
	0,    		  //default
	(void*)&gPara.MiddleOfs,
//	"Steer",
//	"中点位置(0度)",
};
static const PRM_DEF_STRUCT cPara0199[] = 
{
	199,
	PRM_ARRT_EPROM | PRM_ATTR_SIGNED | PRM_ATTR_SIZE(gPara.MiddleDirErr),
	-(8192*2),	 	    	//min
	(8192*2),    	//max
	0,    		  //default
	(void*)&gPara.MiddleDirErr,
//	"Steer",
//	"反向间隙",
};
static const PRM_DEF_STRUCT cPara0200[] = 
{
	200,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0201[] = 
{
	201,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0202[] = 
{
	202,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0203[] = 
{
	203,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0204[] = 
{
	204,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0205[] = 
{
	205,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0206[] = 
{
	206,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0207[] = 
{
	207,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0208[] = 
{
	208,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0209[] = 
{
	209,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0210[] = 
{
	210,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0211[] = 
{
	211,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0212[] = 
{
	212,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0213[] = 
{
	213,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0214[] = 
{
	214,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0215[] = 
{
	215,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0216[] = 
{
	216,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0217[] = 
{
	217,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0218[] = 
{
	218,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0219[] = 
{
	219,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0220[] = 
{
	220,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0221[] = 
{
	221,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0222[] = 
{
	222,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0223[] = 
{
	223,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
/*Battery*/
static const PRM_DEF_STRUCT cPara0224[] = 
{
	224,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.BatFlag1),
	0,	 	    		//min
	0xffff,    		//max
	BATTERY_HOUR_VALID,    		  //default
	(void*)&gPara.BatFlag1,
//	"BatteryHour",
//	"电量1有效标志",
};
static const PRM_DEF_STRUCT cPara0225[] = 
{
	225,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.BatCount1),
	5,	 	    		//min
	100,    		//max
	100,    		  //default
	(void*)&gPara.BatCount1,
//	"BatteryHour",
//	"电量1",
};
static const PRM_DEF_STRUCT cPara0226[] = 
{
	226,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.BatFlag2),
	0,	 	    		//min
	0xffff,    		//max
	BATTERY_HOUR_VALID,    		  //default
	(void*)&gPara.BatFlag2,
//	"BatteryHour",
//	"电量2有效标志",
};
static const PRM_DEF_STRUCT cPara0227[] = 
{
	227,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.BatCount2),
	5,	 	    		//min
	100,    		//max
	100,    		  //default
	(void*)&gPara.BatCount2,
//	"BatteryHour",
//	"电量2",
};
static const PRM_DEF_STRUCT cPara0228[] = 
{
	228,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.HourFlag1),
	0,	 	    		//min
	0xffff,    		//max
	BATTERY_HOUR_VALID,    		  //default
	(void*)&gPara.HourFlag1,
//	"BatteryHour",
//	"小时计1有效标志",
};
static const PRM_DEF_STRUCT cPara0229[] = 
{
	229,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.HourCount1L),
	0,	 	    		//min
	0xffff,    		//max
	0,    		  //default
	(void*)&gPara.HourCount1L,
//	"BatteryHour",
//	"小时计1低16位",
};
static const PRM_DEF_STRUCT cPara0230[] = 
{
	230,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.HourCount1H),
	0,	 	    		//min
	0xffff,    		//max
	0,    		  //default
	(void*)&gPara.HourCount1H,
//	"BatteryHour",
//	"小时计1高16位",
};
static const PRM_DEF_STRUCT cPara0231[] = 
{
	231,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.HourFlag2),
	0,	 	    		//min
	0xffff,    		//max
	BATTERY_HOUR_VALID,    		  //default
	(void*)&gPara.HourFlag2,
//	"BatteryHour",
//	"小时计2有效标志",
};
static const PRM_DEF_STRUCT cPara0232[] = 
{
	232,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.HourCount2L),
	0,	 	    		//min
	0xffff,    		//max
	0,    		  //default
	(void*)&gPara.HourCount2L,
//	"BatteryHour",
//	"小时计2低16位",
};
static const PRM_DEF_STRUCT cPara0233[] = 
{
	233,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.HourCount2H),
	0,	 	    		//min
	0xffff,    		//max
	0,    		  //default
	(void*)&gPara.HourCount2H,
//	"BatteryHour",
//	"小时计2高16位",
};

/*Hardware 234~249*/
static const PRM_DEF_STRUCT cPara0234[] = 
{
	234,
	PRM_ARRT_HARD | PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.CurrentULin),
	(4096-512),	 	    		//min
	(4096+512),    		//max
	4096,    		  //default
	(void*)&gPara.CurrentULin,
//	"Hardware",
//	"U phase currrent sensor nonlinear para.",
};
static const PRM_DEF_STRUCT cPara0235[] = 
{
	235,
	PRM_ARRT_HARD | PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.CurrentVLin),
	(4096-512),	 	    		//min
	(4096+512),    		//max
	4096,    		  //default
	(void*)&gPara.CurrentVLin,
//	"Hardware",
//	"V phase currrent sensor nonlinear para.",
};
static const PRM_DEF_STRUCT cPara0236[] = 
{
	236,
	PRM_ARRT_HARD | PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.KsiVotageLin),
	(4096-512),	 	    		//min
	(4096+512),    		//max
	4096,    		  //default
	(void*)&gPara.KsiVotageLin,
//	"Hardware",
//	"KSI voltage sensor nonlinear para.",
};
static const PRM_DEF_STRUCT cPara0237[] = 
{
	237,
	PRM_ARRT_HARD | PRM_ARRT_EPROM | PRM_ATTR_SIGNED | PRM_ATTR_SIZE(gPara.KsiVotageOfs),
	(0-512),	 	    		//min
	(0+512),    		//max
	0,    		  //default
	(void*)&gPara.KsiVotageOfs,
//	"Hardware",
//	"KSI voltage sensor zero offset.",
};
static const PRM_DEF_STRUCT cPara0238[] = 
{
	238,
	PRM_ARRT_HARD | PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.BusVotageLin),
	(4096-512),	 	    		//min
	(4096+512),    		//max
	4096,    		  //default
	(void*)&gPara.BusVotageLin,
//	"Hardware",
//	"Bus voltage sensor nonlinear para.",
};
static const PRM_DEF_STRUCT cPara0239[] = 
{
	239,
	PRM_ARRT_HARD | PRM_ARRT_EPROM | PRM_ATTR_SIGNED | PRM_ATTR_SIZE(gPara.BusVotageOfs),
	(0-512),	 	    		//min
	(0+512),    		//max
	0,    		  //default
	(void*)&gPara.BusVotageOfs,
//	"Hardware",
//	"BUS voltage sensor zero offset.",
};
static const PRM_DEF_STRUCT cPara0240[] = 
{
	240,
	PRM_ARRT_HARD | PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.HardwareRsv),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.HardwareRsv,
//	"Hardware",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0241[] = 
{
	241,
	PRM_ARRT_HARD | PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.HardwareRsv),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.HardwareRsv,
//	"Hardware",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0242[] = 
{
	242,
	PRM_ARRT_HARD | PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.HardwareRsv),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.HardwareRsv,
//	"Hardware",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0243[] = 
{
	243,
	PRM_ARRT_HARD | PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.HardwareRsv),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.HardwareRsv,
//	"Hardware",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0244[] = 
{
	244,
	PRM_ARRT_HARD | PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.HardwareRsv),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.HardwareRsv,
//	"Hardware",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0245[] = 
{
	245,
	PRM_ARRT_HARD | PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.HardwareRsv),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.HardwareRsv,
//	"Hardware",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0246[] = 
{
	246,
	PRM_ARRT_HARD | PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.HardwareRsv),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.HardwareRsv,
//	"Hardware",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0247[] = 
{
	247,
	PRM_ARRT_HARD | PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.HardwareRsv),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.HardwareRsv,
//	"Hardware",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0248[] = 
{
	248,
	PRM_ARRT_HARD | PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.HardwareRsv),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.HardwareRsv,
//	"Hardware",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0249[] = 
{
	249,
	PRM_ARRT_HARD | PRM_ARRT_EPROM | PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.HardwareSum),
	0,	 	    		//min
	65535,    		//max
	((~(16384)+1) & 0xffff),    		  //default
	(void*)&gPara.HardwareSum,
//	"Hardware",
//	"Sum of Hardware para 001~015.",
};

/*PassWord*/
static const PRM_DEF_STRUCT cPara0250[] = 
{
	250,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.AppPassWord),
	0,	 	    		//min
	32767,    		//max
	11111,    		  //default
	(void*)&gPara.AppPassWord,
//	"PassWord",
//	"App user pass word",
};
static const PRM_DEF_STRUCT cPara0251[] = 
{
	251,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.OemPassWord),
	0,	 	    		//min
	32767,    		//max
	12345,    		  //default
	(void*)&gPara.OemPassWord,
//	"PassWord",
//	"Oem pass word",
};
static const PRM_DEF_STRUCT cPara0252[] = 
{
	252,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.SysPassWord),
	0,	 	    		//min
	32767,    		//max
	12345,    		  //default
	(void*)&gPara.SysPassWord,
//	"PassWord",
//	"Oem pass word",
};
static const PRM_DEF_STRUCT cPara0253[] = 
{
	253,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.SysDevPassWord),
	0,	 	    		//min
	32767,    		//max
	12345,    		  //default
	(void*)&gPara.SysDevPassWord,
//	"PassWord",
//	"Oem pass word",
};
static const PRM_DEF_STRUCT cPara0254[] = 
{
	254,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    		//min
	0,    		//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cPara0255[] = 
{
	255,
	PRM_ARRT_EPROM | PRM_ATTR_SIZE(gPara.UserType),
	USER_TYPE,	 	    	//min 0
	USER_TYPE,    		//max 9999
	USER_TYPE,  //default
	(void*)&gPara.UserType,
//	"Version",
//	"User & Vehicle",
};

/********************* Read only fix para *********************/
	/* CAN */
static const PRM_DEF_STRUCT cPara0256[] = 
{
	256,
	PRM_ATTR_SIZE(gPara.StationAddr),
	0,	 	    	//min
	31,    			//max
	LOGIC_TYPE,    		  //default
	(void*)&gPara.StationAddr,
//	"Can",
//	"Can id addr",
};
	/* motor */
static const PRM_DEF_STRUCT cPara0257[] = 
{
	257,
	PRM_ATTR_SIZE(gPara.RotorTCoe),
	256,	 	    	//min
	512,    			//max
	400,    		  //default
	(void*)&gPara.RotorTCoe,
//	"Motor",
//	"The thermal Coe for motor rotor.",
};
static const PRM_DEF_STRUCT cPara0258[] = 
{
	258,
	PRM_ATTR_SIZE(gPara.CalibAngle),
	0,	 	    	//min
	32767,    			//max
	0,    		  //default
	(void*)&gPara.CalibAngle,
//	"Motor",
//	"The thermal Coe for motor rotor.",
};

	/* Sys */
static const PRM_DEF_STRUCT cPara0259[] = 
{
	259,
	PRM_ATTR_SIZE(gPara.RunMode),
	0,	 	    	//min
	2,    			//max
	1,    		  //default
	(void*)&gPara.RunMode,
//	"Sys",
//	"Sys mode 0-Pos, 1-Speed, 2-Torque.",
};
	/* Speed */
static const PRM_DEF_STRUCT cPara0260[] = 
{
	260,
	PRM_ATTR_SIZE(gPara.SpdAccT),
	0,	 	    	//min
	1000,    			//max
	30,    		  //default
	(void*)&gPara.SpdAccT,
//	"Speed",
//	"Acc acceleration  low filter time 0~1000 ms.",
};
static const PRM_DEF_STRUCT cPara0261[] = 
{
	261,
	PRM_ATTR_SIZE(gPara.SpdDecT),
	0,	 	    	//min
	1000,    			//max
	30,    		  //default
	(void*)&gPara.SpdDecT,
//	"Speed",
//	"Dec acceleration  low filter time 0~1000 ms.",
};
static const PRM_DEF_STRUCT cPara0262[] = 
{
	262,
	PRM_ATTR_SIZE(gPara.NotchFilt1W),
	10,	 	    	//min
	5000,    			//max
	5000,    		  //default
	(void*)&gPara.NotchFilt1W,
//	"Speed",
//	"第一陷波滤波器陷波频率(Hz).",
};
static const PRM_DEF_STRUCT cPara0263[] = 
{
	263,
	PRM_ATTR_SIZE(gPara.NotchFilt1Q),
	0,	 	    	//min
	100,    			//max
	71,    		  //default
	(void*)&gPara.NotchFilt1Q,
//	"Speed",
//	"第一陷波滤波器品质因数(%).",
};
static const PRM_DEF_STRUCT cPara0264[] = 
{
	264,
	PRM_ATTR_SIZE(gPara.NotchFilt2W),
	10,	 	    	//min
	5000,    			//max
	5000,    		  //default
	(void*)&gPara.NotchFilt2W,
//	"Speed",
//	"第二陷波滤波器陷波频率(Hz).",
};
static const PRM_DEF_STRUCT cPara0265[] = 
{
	265,
	PRM_ATTR_SIZE(gPara.NotchFilt2Q),
	0,	 	    	//min
	100,    			//max
	71,    		  //default
	(void*)&gPara.NotchFilt2Q,
//	"Speed",
//	"第二陷波滤波器品质因数(%).",
};
	/* Vehicle */
static const PRM_DEF_STRUCT cPara0266[] = 
{
	266,
	PRM_ATTR_SIZE(gPara.DcRateVoltage),
	240,	 	    	//min
	9000,    	//max
	RATE_VOLTAGE,    		  //default
	(void*)&gPara.DcRateVoltage,
//	"Vehicle",
//	"电瓶电压等级,unit 0.1V",
};
static const PRM_DEF_STRUCT cPara0267[] = 
{
	267,
	PRM_ATTR_SIZE(gPara.BatAlarmRatio),
	15,	 	    	//min
	30,    	//max
	20,    		  //default
	(void*)&gPara.BatAlarmRatio,
//	"Vehicle",
//	"电瓶电量低报警点(%)",
};
static const PRM_DEF_STRUCT cPara0268[] = 
{
	268,
	PRM_ATTR_SIZE(gPara.BatProtectRatio),
	10,	 	    	//min
	25,    	//max
	15,    		  //default
	(void*)&gPara.BatProtectRatio,
//	"Vehicle",
//	"电瓶电量低保护点(%)",
};
	/*Hardware*/
static const PRM_DEF_STRUCT cPara0269[] = 
{
	269,
	PRM_ATTR_SIZE(gPara.DcMaxVoltage),
	120,	 	    	//min
	1200,    			//max
	MAX_VOLTAGE,  //default
	(void*)&gPara.DcMaxVoltage,
//	"Hardware",
//	"最高电压，超过该电压系统不启动 ,unit 0.1V",
};

static const PRM_DEF_STRUCT cPara0270[] = 
{
	270,
	PRM_ATTR_SIZE(gPara.DcBrkLimitVoltage),
	120,	 	    	//min
	1200,    			//max
	BRKLIMIT_VOLTAGE,    		  //default
	(void*)&gPara.DcBrkLimitVoltage,
//	"Hardware",
//	"最高电压，超过该电压限制刹车力,unit 0.1V",
};
static const PRM_DEF_STRUCT cPara0271[] = 
{
	271,
	PRM_ATTR_SIZE(gPara.DcCutVoltage),
	120,	 	    	//min
	1200,    			//max
	CUT_VOLTAGE,  //default
	(void*)&gPara.DcCutVoltage,
//	"Hardware",
//	" 削减电压，低于该电压系统降耗运行，削减输出扭矩 ,unit 0.1V",
};
static const PRM_DEF_STRUCT cPara0272[] = 
{
	272,
	PRM_ATTR_SIZE(gPara.DcMinVoltage),
	120,	 	    	//min
	1200,    			//max
	MIN_VOLTAGE,    		  //default
	(void*)&gPara.DcMinVoltage,
//	"Hardware",
//	"最低电压，低于该电压系统不启动，停止运行 ,unit 0.1V",
};
static const PRM_DEF_STRUCT cPara0273[] = 
{
	273,
	PRM_ATTR_SIZE(gPara.DcBroVoltage),
	120,	 	    	//min
	1200,    			//max
	BRO_VOLTAGE,    		  //default
	(void*)&gPara.DcBroVoltage,
//	"Hardware",
//	"启动电压，低于该电压，系统复位，关闭一切输出 ,unit 0.1V",
};
static const PRM_DEF_STRUCT cPara0274[] = 
{
	274,
	PRM_ATTR_SIZE(gPara.DcEspLowVoltage),
	120,	 	    	//min
	1200,    			//max
	ESP_LOW_VOLTAGE,    		  //default
	(void*)&gPara.DcEspLowVoltage,
//	"Hardware",
//	"急停电压，低于该电压，认为短路，关闭PWM上下桥臂，必须断电恢复 unit 0.1",
};
static const PRM_DEF_STRUCT cPara0275[] = 
{
	275,
	PRM_ATTR_SIZE(gPara.CtlRate1HCurrent),
	1,	 	    				//min
	800,    					//max
	RATE_1H_CURRENT,  //default
	(void*)&gPara.CtlRate1HCurrent,
//	"Hardware",
//	"1小时工作制电流, unit A",
};
static const PRM_DEF_STRUCT cPara0276[] = 
{
	276,
	PRM_ATTR_SIZE(gPara.CtlRate2MCurrent),
	1,	 	    				//min
	800,    					//max
	RATE_2M_CURRENT,  //default
	(void*)&gPara.CtlRate2MCurrent,
//	"Hardware",
//	"2分钟工作制电流, unit A",
};
static const PRM_DEF_STRUCT cPara0277[] = 
{
	277,
	PRM_ATTR_SIZE(gPara.Out5VMaxVoltage),
	0,	 	    	//min
	1200,    		//max
	55,    		  //default
	(void*)&gPara.Out5VMaxVoltage,
//	"Hardware",
//	"Max voltage of 5V out. Unit 0.1V",
};
static const PRM_DEF_STRUCT cPara0278[] = 
{
	278,
	PRM_ATTR_SIZE(gPara.Out5VMinVoltage),
	0,	 	    	//min
	1200,    		//max
	45,    		  //default
	(void*)&gPara.Out5VMinVoltage,
//	"Hardware",
//	"Min voltage of 5V out. Unit 0.1V",
};
static const PRM_DEF_STRUCT cPara0279[] = 
{
	279,
	PRM_ATTR_SIZE(gPara.Out12VMaxVoltage),
	0,	 	    	//min
	1200,    		//max
	145,    		  //default
	(void*)&gPara.Out12VMaxVoltage,
//	"Hardware",
//	"Max voltage of 12V out. Unit 0.1V",
};
static const PRM_DEF_STRUCT cPara0280[] = 
{
	280,
	PRM_ATTR_SIZE(gPara.Out12VMinVoltage),
	0,	 	    	//min
	1200,    		//max
	115,    		  //default
	(void*)&gPara.Out12VMinVoltage,
//	"Hardware",
//	"Min voltage of 12V out. Unit 0.1V",
};
static const PRM_DEF_STRUCT cPara0281[] = 
{
	281,
	PRM_ATTR_SIZE(gPara.PowerCutTmp),
	0,	 	    	//min
	100,    	//max
	TMP_POWERCUT,    		  //default
	(void*)&gPara.PowerCutTmp,
//	"Hardware",
//	"功率板过载温度1，需降低功率板输出电流",
};
static const PRM_DEF_STRUCT cPara0282[] = 
{
	282,
	PRM_ATTR_SIZE(gPara.PowerMaxTmp),
	0,	 	    			//min
	100,    				//max
	TMP_POWERMAX,   //default
	(void*)&gPara.PowerMaxTmp,
//	"Hardware",
//	"功率板过载温度2，需关闭功率板输出电流",
};
static const PRM_DEF_STRUCT cPara0283[] = 
{
	283,
	PRM_ATTR_SIZE(gPara.PowerMinTmp),
	-40,	 	    		//min
	100,    				//max
	TMP_POWERMIN,   //default
	(void*)&gPara.PowerMinTmp,
//	"Hardware",
//	"功率板低温，需降低功率板输出电流",
};

	/* Tune */
static const PRM_DEF_STRUCT cPara0284[] = 
{
	284,
	PRM_ATTR_SIZE(gPara.TuneSpdMax),
	1000,	 	    	//min
	20000,    	//max
	3500,    		  //default
	(void*)&gPara.TuneSpdMax,
//	"Tune",
//	"参数整定 最大速度 Unit 0.01Hz",
};
static const PRM_DEF_STRUCT cPara0285[] = 
{
	285,
	PRM_ATTR_SIZE(gPara.TuneR),
	1,	 	    	//min
	30000,    	//max
	4,    		  //default
	(void*)&gPara.TuneR,
//	"Tune",
//	"参数整定 最大圈数 ",
};
static const PRM_DEF_STRUCT cPara0286[] = 
{
	286,
	PRM_ATTR_SIZE(gPara.RefRigid),
	0,	 	    	//min
	10,    	//max
	1,    		  //default
	(void*)&gPara.RefRigid,
//	"Tune",
//	"参考刚性 ",
};

	/* other */
static const PRM_DEF_STRUCT cPara0287[] = 
{
	287,
	PRM_ATTR_SIZE(gPara.ExtBrakeHoldTime),
	0,	 	    	//min
	200,    	//max
	50,    		  //default
	(void*)&gPara.ExtBrakeHoldTime,
//	"Other",
//	" 电机抱闸保持时间(10 ms)",
};
static const PRM_DEF_STRUCT cPara0288[] = 
{
	288,
	PRM_ATTR_SIZE(gPara.ErrorTrace[0]),
	0,	 	    	//min
	1000,    	//max
	0,    		  //default
	(void*)&gPara.ErrorTrace[0],
//	"Other",
//	"Error trace",
};
static const PRM_DEF_STRUCT cPara0289[] = 
{
	289,
	PRM_ATTR_SIZE(gPara.ErrorTrace[1]),
	0,	 	    	//min
	1000,    	//max
	0,    		  //default
	(void*)&gPara.ErrorTrace[1],
//	"Other",
//	"Error trace",
};
static const PRM_DEF_STRUCT cPara0290[] = 
{
	290,
	PRM_ATTR_SIZE(gPara.ErrorTrace[2]),
	0,	 	    	//min
	1000,    	//max
	0,    		  //default
	(void*)&gPara.ErrorTrace[2],
//	"Other",
//	"Error trace",
};
static const PRM_DEF_STRUCT cPara0291[] = 
{
	291,
	PRM_ATTR_SIZE(gPara.ErrorTrace[3]),
	0,	 	    	//min
	1000,    	//max
	0,    		  //default
	(void*)&gPara.ErrorTrace[3],
//	"Other",
//	"Error trace",
};
static const PRM_DEF_STRUCT cPara0292[] = 
{
	292,
	PRM_ATTR_SIZE(gPara.ErrorTrace[4]),
	0,	 	    	//min
	1000,    	//max
	0,    		  //default
	(void*)&gPara.ErrorTrace[4],
//	"Other",
//	"Error trace",
};
static const PRM_DEF_STRUCT cPara0293[] = 
{
	293,
	PRM_ATTR_SIZE(gPara.ErrorTrace[5]),
	0,	 	    	//min
	1000,    	//max
	0,    		  //default
	(void*)&gPara.ErrorTrace[5],
//	"Other",
//	"Error trace",
};
static const PRM_DEF_STRUCT cPara0294[] = 
{
	294,
	PRM_ATTR_SIZE(gPara.ErrorTrace[6]),
	0,	 	    	//min
	1000,    	//max
	0,    		  //default
	(void*)&gPara.ErrorTrace[6],
//	"Other",
//	"Error trace",
};
static const PRM_DEF_STRUCT cPara0295[] = 
{
	295,
	PRM_ATTR_SIZE(gPara.ErrorTrace[7]),
	0,	 	    	//min
	1000,    	//max
	0,    		  //default
	(void*)&gPara.ErrorTrace[7],
//	"Other",
//	"Error trace",
};
static const PRM_DEF_STRUCT cPara0296[] = 
{
	296,
	PRM_ATTR_SIZE(gPara.ErrorTrace[8]),
	0,	 	    	//min
	1000,    	//max
	0,    		  //default
	(void*)&gPara.ErrorTrace[8],
//	"Other",
//	"Error trace",
};
static const PRM_DEF_STRUCT cPara0297[] = 
{
	297,
	PRM_ATTR_SIZE(gPara.ErrorTrace[9]),
	0,	 	    	//min
	1000,    	//max
	0,    		  //default
	(void*)&gPara.ErrorTrace[9],
//	"Other",
//	"Error trace",
};
static const PRM_DEF_STRUCT cPara0298[] = 
{
	298,
	PRM_ATTR_SIZE(gPara.TorqueOffset),
	0,	 	    	//min
	0,    	//max
	0,    		  //default
	(void*)&gPara.TorqueOffset,
//	"Torque",
//	"转矩运行模式时的转矩偏置(0.01Nm)",
};
static const PRM_DEF_STRUCT cPara0299[] = 
{
	299,
	PRM_ATTR_SIZE(gPara.ObserverEnable),
	0,	 	    	//min
	1,    	//max
	0,    		  //default
	(void*)&gPara.ObserverEnable,
//	"Speed",
//	"速度观测器使能 ",
};
static const PRM_DEF_STRUCT cPara0300[] = 
{
	300,
	PRM_ATTR_SIZE(gPara.ObserverK),
	0,	 	    	//min
	0,    	//max
	0,    		  //default
	(void*)&gPara.ObserverK,
//	"Speed",
//	"速度观测器系数：负载带宽是速度带宽的K%倍 ",
};
static const PRM_DEF_STRUCT cPara0301[] = 
{
	301,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    	//min
	0,    	//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"State",
//	"null",
};
static const PRM_DEF_STRUCT cPara0302[] = 
{
	302,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    	//min
	0,    	//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"State",
//	"null",
};
/* parameters records data */
//static 
const PRM_DEF_STRUCT* cPara_Table[] = 
{
	cPara0000, cPara0001, cPara0002, cPara0003, cPara0004, cPara0005, cPara0006, cPara0007, cPara0008, cPara0009, 
	cPara0010, cPara0011, cPara0012, cPara0013, cPara0014, cPara0015, cPara0016, cPara0017, cPara0018, cPara0019, 
	cPara0020, cPara0021, cPara0022, cPara0023, cPara0024, cPara0025, cPara0026, cPara0027, cPara0028, cPara0029,
	cPara0030, cPara0031, cPara0032, cPara0033, cPara0034, cPara0035, cPara0036, cPara0037, cPara0038, cPara0039,
	cPara0040, cPara0041, cPara0042, cPara0043, cPara0044, cPara0045, cPara0046, cPara0047, cPara0048, cPara0049,
	cPara0050, cPara0051, cPara0052, cPara0053, cPara0054, cPara0055, cPara0056, cPara0057, cPara0058, cPara0059,
	cPara0060, cPara0061, cPara0062, cPara0063, cPara0064, cPara0065, cPara0066, cPara0067, cPara0068, cPara0069,
	cPara0070, cPara0071, cPara0072, cPara0073, cPara0074, cPara0075, cPara0076, cPara0077, cPara0078, cPara0079,
	cPara0080, cPara0081, cPara0082, cPara0083, cPara0084, cPara0085, cPara0086, cPara0087, cPara0088, cPara0089,
	cPara0090, cPara0091, cPara0092, cPara0093, cPara0094, cPara0095, cPara0096, cPara0097, cPara0098, cPara0099,
	cPara0100, cPara0101, cPara0102, cPara0103, cPara0104, cPara0105, cPara0106, cPara0107, cPara0108, cPara0109,
	cPara0110, cPara0111, cPara0112, cPara0113, cPara0114, cPara0115, cPara0116, cPara0117, cPara0118, cPara0119,
	cPara0120, cPara0121, cPara0122, cPara0123, cPara0124, cPara0125, cPara0126, cPara0127, cPara0128, cPara0129,
	cPara0130, cPara0131, cPara0132, cPara0133, cPara0134, cPara0135, cPara0136, cPara0137, cPara0138, cPara0139,
	cPara0140, cPara0141, cPara0142, cPara0143, cPara0144, cPara0145, cPara0146, cPara0147, cPara0148, cPara0149,
	cPara0150, cPara0151, cPara0152, cPara0153, cPara0154, cPara0155, cPara0156, cPara0157, cPara0158, cPara0159,
	cPara0160, cPara0161, cPara0162, cPara0163, cPara0164, cPara0165, cPara0166, cPara0167, cPara0168, cPara0169,
	cPara0170, cPara0171, cPara0172, cPara0173, cPara0174, cPara0175, cPara0176, cPara0177, cPara0178, cPara0179,
	cPara0180, cPara0181, cPara0182, cPara0183, cPara0184, cPara0185, cPara0186, cPara0187, cPara0188, cPara0189,
	cPara0190, cPara0191, cPara0192, cPara0193, cPara0194, cPara0195, cPara0196, cPara0197, cPara0198, cPara0199,
	cPara0200, cPara0201, cPara0202, cPara0203, cPara0204, cPara0205, cPara0206, cPara0207, cPara0208, cPara0209,
	cPara0210, cPara0211, cPara0212, cPara0213, cPara0214, cPara0215, cPara0216, cPara0217, cPara0218, cPara0219,
	cPara0220, cPara0221, cPara0222, cPara0223, cPara0224, cPara0225, cPara0226, cPara0227, cPara0228, cPara0229,
	cPara0230, cPara0231, cPara0232, cPara0233, cPara0234, cPara0235, cPara0236, cPara0237, cPara0238, cPara0239,
	cPara0240, cPara0241, cPara0242, cPara0243, cPara0244, cPara0245, cPara0246, cPara0247, cPara0248, cPara0249,
	cPara0250, cPara0251, cPara0252, cPara0253, cPara0254, cPara0255,	cPara0256, cPara0257, cPara0258, cPara0259,
	cPara0260, cPara0261, cPara0262, cPara0263, cPara0264, cPara0265, cPara0266, cPara0267, cPara0268, cPara0269,
	cPara0270, cPara0271, cPara0272, cPara0273, cPara0274, cPara0275, cPara0276, cPara0277, cPara0278, cPara0279,
	cPara0280, cPara0281, cPara0282, cPara0283, cPara0284, cPara0285, cPara0286, cPara0287, cPara0288, cPara0289,
	cPara0290, cPara0291, cPara0292, cPara0293, cPara0294, cPara0295, cPara0296, cPara0297, cPara0298, cPara0299,
	cPara0300, cPara0301, cPara0302,
};
/* Hardware parameters records data */
static const PRM_DEF_STRUCT* cHardPara_Table[] = 
{
	cPara0234, cPara0235, cPara0236, cPara0237, cPara0238, cPara0239,	cPara0240, cPara0241,
	cPara0242, cPara0243, cPara0244, cPara0245, cPara0246, cPara0247, cPara0248, cPara0249
};

/*******************************************************************************
* Name: ReadHardParaFromEeprom
* Description: Read hardware parameters from eeprom para area.
* Input: Index
* Output: 0: success. 1: fail
* 
* Author:
* Date:
* Revision:
*******************************************************************************/
INT16U ReadHardParaFromEeprom(INT16U Index)
{
	INT16U data, address, ret;
	INT32U PrmOfs;
	PRM_ATTR_STRUCT* pPrmAttr;
	PRM_DEF_STRUCT* pPrmDef;
	
	if (Index >= (sizeof(cHardPara_Table)/sizeof(cHardPara_Table[0])))
		return 1;
	
	pPrmDef = (PRM_DEF_STRUCT*)(cHardPara_Table[Index]);
	pPrmAttr = (PRM_ATTR_STRUCT*)(&pPrmDef->PrmAttr);
	//处理非eprom参数
	if (pPrmAttr->Eprom == 0)
	{
		*((INT16U*)pPrmDef->pPrmData) = pPrmDef->PrmInitVal;
		return 0;
	}
	//addr overflow protect
	PrmOfs = pPrmDef->PrmNo;
	if (PrmOfs >= EEPROM_PARA_NUM)
	{
		*((INT16U*)pPrmDef->pPrmData) = pPrmDef->PrmInitVal;
		return 1;
	}
	address = EEPROM_PARA_AREA_ADDR+PrmOfs;	//参数地址计算
	ret = 0;
	
	while(1)
	{
		ret = EepromQualifiedRead(address, &data);
		if(ret == 1)
		{
			*((INT16U*)pPrmDef->pPrmData) = pPrmDef->PrmInitVal;
			break;
		}
		/* if over limit */
		if (pPrmAttr->Signed == 1)
		{
			if(((INT16S)data > pPrmDef->PrmMaxVal) || ((INT16S)data < pPrmDef->PrmMinVal))
			{
				/* the parameter in eeprom is not in proper range */
				ret = 1;
				SL_SET(PLC_PARA_OV_LIMIT_ERR);
				
				/* if error, then default the parameter */
				*((INT16U*)pPrmDef->pPrmData) = pPrmDef->PrmInitVal;
				EepromQualifiedWrite(address, pPrmDef->PrmInitVal);
				break;
			}
			else /*No overflow*/
			{
				*((INT16U*)(pPrmDef->pPrmData)) = data;	/* set parameter in sram */
				break;
			}
		}
		else //unsign
		{
			if((data > (INT16U)pPrmDef->PrmMaxVal) || (data < (INT16U)pPrmDef->PrmMinVal))
			{
				/* the value in eeprom is overflow */
				ret = 1;
				SL_SET(PLC_PARA_OV_LIMIT_ERR);
				/* if error, then default the parameter */
				*((INT16U*)pPrmDef->pPrmData) = pPrmDef->PrmInitVal;
				EepromQualifiedWrite(address, pPrmDef->PrmInitVal);
				break;
			}
			else	/*No overflow */
			{
				*((INT16U*)(pPrmDef->pPrmData)) = data;	/* set parameter in sram */
				break;
			}
		}
	}
	return ret;
}
/*******************************************************************************
* Name: SaveHardParaToEeprom
* Description: Save parameters to eeprom para area or backup area.
* Input: .
* Output: 0: success. 1: fail
* 
* Author:
* Date:
* Revision:
*******************************************************************************/
INT16U SaveHardParaToEeprom(INT16U Index,INT16U data)
{
	INT16U dataRd, address, ret;
	INT32U PrmOfs;
	PRM_ATTR_STRUCT* pPrmAttr;
	PRM_DEF_STRUCT* pPrmDef;
	
	if (Index >= (sizeof(cHardPara_Table)/sizeof(cHardPara_Table[0])))
		return 1;
#ifdef __KFFDEBUG
	if ((Index == 64) && (data != 8000))
		Index = 64;
#endif
	pPrmDef = (PRM_DEF_STRUCT*)(cHardPara_Table[Index]);
	pPrmAttr = (PRM_ATTR_STRUCT*)(&pPrmDef->PrmAttr);
	//处理非eprom参数
	if (pPrmAttr->Eprom == 0)
	{
		return 0;
	}
	//addr overflow protect
	PrmOfs = pPrmDef->PrmNo;
	if (PrmOfs >= EEPROM_PARA_NUM)
	{
		return 1;
	}
	address = EEPROM_PARA_AREA_ADDR + PrmOfs;	//参数地址计算

	ret = 0;
	while(1)
	{
		/* if over limit */
		if (pPrmAttr->Signed == 1)
		{
			if(((INT16S)data > pPrmDef->PrmMaxVal) || ((INT16S)data < pPrmDef->PrmMinVal))
			{
				/* the parameter in eeprom is not in proper range */
				ret = 1;
				SL_SET(PLC_PARA_OV_LIMIT_ERR);
				break;
			}
		}
		else
		{
			if((data > (INT16U)pPrmDef->PrmMaxVal) || (data < (INT16U)pPrmDef->PrmMinVal))
			{
				/* the parameter in eeprom is not in proper range */
				ret = 1;
				SL_SET(PLC_PARA_OV_LIMIT_ERR);
				break;
			}
		}
		/* if not over limit, then save to eeprom */
		/* Read current eprom val out */
		if(EepromQualifiedRead(address, &dataRd) == 1)
		{
			ret = 1;
			SL_SET(PLC_EEPROM_RW_ERR);
			break;
		}
		/* Check new val equal current val*/
		if (data != dataRd)
		{
			/* write eeprom */
			ret = EepromQualifiedWrite(address, data);
			break;
		}
		else
		{
			ret = 0;
			break;
		}
	}
	return ret;
}


///*******************************************************************************
//* Name: EepromQualifiedRead
//* Description: 从EEPROM中指定地址读出数据，含校验
//* Input: eeprom read address.
//* Output: 0: success. 1: fail. eeprom read data.
//* 
//* Author:
//* Date:
//* Revision:
//*******************************************************************************/
//INT16U EepromQualifiedRead(INT16U address, INT16U* pdata)
//{
//	INT16U ReadTimes, ReadFailTimes, ReadValue, data, ret;

//	ret = 0;
//	ReadTimes = 0;
//	ReadFailTimes = 0;

//	while(1)
//	{
//		/* read data in */
//		ret = EepromRead(address, &data);
//		if(ret == 1)
//		{
//			break;
//		}
//		ReadTimes++;
//		if(ReadTimes == 1)
//		{
//			ReadValue = data;
//		}
//		else
//		{
//			/* if current read data != the first read data */
//			if(data != ReadValue)
//			{
//				/* then try again */
//				ReadTimes = 0;
//				/* increase fail times */
//				ReadFailTimes++;
//				if(ReadFailTimes >= 3)
//				{
//					ret = 1;
//					break;
//				}
//			}
//			/* else current read data == the first read data */
//			else
//			{
//				if(ReadTimes >= 3)
//				{
//					/* read ok */
//					break;
//				}
//			}
//		}
//	}

//	*pdata = data;
//	if(ret == 1)
//	{
//		/* eeprom error */
////		SL_SET(PLC_EEPROM_RW_ERR);
//		MCL_SetFault(EEPROM_RW_ERR);
//	}

//	return ret;	
//}

///*******************************************************************************
//* Name: EepromQualifiedWrite
//* Description: 将数据写入EEPROM中指定地址,含校验
//* Input: eeprom write address, write data
//* Output: 0: success. 1: fail
//* 
//* Author:
//* Date:
//* Revision:
//*******************************************************************************/
//INT16U EepromQualifiedWrite(INT16U address, INT16U data)
//{
//	INT16U i16U, WriteFailTimes, ret;

//	ret = 0;
//	WriteFailTimes = 0;
//	while(1)
//	{
//		ret = EepromWrite(address, data);
//		if(ret == 1)
//		{
//			break;
//		}
//		ret = EepromRead(address, &i16U);
//		if(ret == 1)
//		{
//			break;
//		}
//		/* if read == write */
//		if(i16U == data)
//		{
//			/* then write ok */
//			break;
//		}
//		/* else read != write */
//		else
//		{
//			/* then write again until WriteFailTimes exceed 3 */
//			WriteFailTimes++;
//			if(WriteFailTimes >= 3)
//			{
//				ret = 1;
//				break;
//			}
//		}
//	}

//	if(ret == 1)
//	{
//		/* eeprom error */
////		SL_SET(PLC_EEPROM_RW_ERR);
//		MCL_SetFault(EEPROM_RW_ERR);
//	}
//	return ret;
//}

/*******************************************************************************
* Name: ReadParaFromEeprom
* Description: Read parameters from eeprom para area or backup area.
* Input: Index
* Output: 0: success. 1: fail
* 
* Author:
* Date:
* Revision:
*******************************************************************************/
INT16U ReadParaFromEeprom(INT16U Index)
{
	INT16U data, address, ret;
	INT32U PrmOfs;
	PRM_ATTR_STRUCT* pPrmAttr;
	PRM_DEF_STRUCT* pPrmDef;
	
	if (Index >= (sizeof(cPara_Table)/sizeof(cPara_Table[0])))
		return 1;
	
	pPrmDef = (PRM_DEF_STRUCT*)(cPara_Table[Index]);
	pPrmAttr = (PRM_ATTR_STRUCT*)(&pPrmDef->PrmAttr);
	//处理非eprom参数
	if (pPrmAttr->Eprom == 0)
	{
		*((INT16U*)pPrmDef->pPrmData) = pPrmDef->PrmInitVal;
		return 0;
	}
	//addr overflow protect
	PrmOfs = pPrmDef->PrmNo;
	if (PrmOfs >= EEPROM_PARA_NUM)
	{
		*((INT16U*)pPrmDef->pPrmData) = pPrmDef->PrmInitVal;
		return 1;
	}
	address = EEPROM_PARA_AREA_ADDR + PrmOfs;	//参数地址计算
	ret = 0;
	
	while(1)
	{
		ret = EepromQualifiedRead(address, &data);
		if(ret == 1)
		{
			*((INT16U*)pPrmDef->pPrmData) = pPrmDef->PrmInitVal;
			break;
		}

		/* if over limit */
		if (pPrmAttr->Signed == 1)
		{
			if(((INT16S)data > pPrmDef->PrmMaxVal) || ((INT16S)data < pPrmDef->PrmMinVal))
			{
				/* the parameter in eeprom is not in proper range */
				ret = 1;
				SL_SET(PLC_PARA_OV_LIMIT_ERR);
				
				/* if error, then default the parameter */
				*((INT16U*)pPrmDef->pPrmData) = pPrmDef->PrmInitVal;
				ret = EepromQualifiedWrite(address, pPrmDef->PrmInitVal);
				break;
			}
			else /*No overflow*/
			{
				*((INT16U*)(pPrmDef->pPrmData)) = data;	/* set parameter in sram */
				break;
			}
		}
		else //unsign
		{
			if((data > (INT16U)pPrmDef->PrmMaxVal) || (data < (INT16U)pPrmDef->PrmMinVal))
			{
				/* the parameter in eeprom is not in proper range */
				ret = 1;
				SL_SET(PLC_PARA_OV_LIMIT_ERR);

				/* if error, then default the parameter */
				*((INT16U*)pPrmDef->pPrmData) = pPrmDef->PrmInitVal;
				EepromQualifiedWrite(address, pPrmDef->PrmInitVal);
				break;
			}
			else /*No overflow*/
			{
				*((INT16U*)(pPrmDef->pPrmData)) = data;	/* set parameter in sram */
				break;
			}
		}
	}
	return ret;
}

/*******************************************************************************
* Name: SaveParaToEeprom
* Description: Save parameters to eeprom para area or backup area.
* Input: .
* Output: 0: success. 1: fail
* 
* Author:
* Date:
* Revision:
*******************************************************************************/
INT16U SaveParaToEeprom(INT16U Index,INT16U data)
{
	INT16U dataRd, address, ret;
	INT32U PrmOfs;
	PRM_ATTR_STRUCT* pPrmAttr;
	PRM_DEF_STRUCT* pPrmDef;
	
	if (Index >= (sizeof(cPara_Table)/sizeof(cPara_Table[0])))
		return 1;
#ifdef __KFFDEBUG
	if ((Index == 64) && (data != 8000))
		Index = 64;
#endif
	pPrmDef = (PRM_DEF_STRUCT*)(cPara_Table[Index]);
	pPrmAttr = (PRM_ATTR_STRUCT*)(&pPrmDef->PrmAttr);
	//处理非eprom参数
	if (pPrmAttr->Eprom == 0)
	{
		return 0;
	}
	//addr overflow protect
	PrmOfs = pPrmDef->PrmNo;
	if (PrmOfs >= EEPROM_PARA_NUM)
	{
		return 1;
	}
	address = EEPROM_PARA_AREA_ADDR + PrmOfs;	//参数地址计算

	ret = 0;
	while(1)
	{
		/* if over limit */
		if (pPrmAttr->Signed == 1)
		{
			if(((INT16S)data > pPrmDef->PrmMaxVal) || ((INT16S)data < pPrmDef->PrmMinVal))
			{
				/* the parameter in eeprom is not in proper range */
				ret = 1;
				SL_SET(PLC_PARA_OV_LIMIT_ERR);
				break;
			}
		}
		else
		{
			if((data > (INT16U)pPrmDef->PrmMaxVal) || (data < (INT16U)pPrmDef->PrmMinVal))
			{
				/* the parameter in eeprom is not in proper range */
				ret = 1;
				SL_SET(PLC_PARA_OV_LIMIT_ERR);
				break;
			}
		}
		/* if not over limit, then save to eeprom */
		/* Read current eprom val out */
		if(EepromQualifiedRead(address, &dataRd) == 1)
		{
			ret = 1;
			SL_SET(PLC_EEPROM_RW_ERR);
			break;
		}
		/* Check new val equal current val*/
		if (data != dataRd)
		{
			/* write eeprom */
			ret = EepromQualifiedWrite(address, data);
			break;
		}
		else
		{
			ret = 0;
			break;
		}
	}
	return ret;
}

/*******************************************************************************
* Name: ReadParaValByIndex
* Description: 
* Input: Index
* Output: Para val
* 
* Author:
* Date:
* Revision:
*******************************************************************************/
INT16U ReadParaValByIndex(INT16U Index)
{
	if (Index < (sizeof(cPara_Table)/sizeof(cPara_Table[0])))
	{
		ReadParaFromEeprom(Index);
		return *((INT16U*)cPara_Table[Index]->pPrmData);
	}
	else
	{
		SL_SET(PLC_PARA_OV_INDEX_ERR);
		return 0;
	}
}

	/* Version */
static const PRM_DEF_STRUCT cMonitorPara0210[] = 
{
	210,
	PRM_ATTR_SIZE(gPara.UserType),
	USER_TYPE,	 	    	//min 0
	USER_TYPE,    		//max 9999
	USER_TYPE,  //default
	(void*)&gPara.UserType,
//	"Version",
//	"User & Vehicle",
};
static const PRM_DEF_STRUCT cMonitorPara0211[] = 
{
	211,
	PRM_ATTR_SIZE(gPara.ProductType),
	PRODUCT_TYPE,	 	    		//min 0
	PRODUCT_TYPE,    				//max 9999
	PRODUCT_TYPE,   //default
	(void*)&gPara.ProductType,
//	"Version",
//	"驱动器类型 4845/4835/2425/2405/4805",
};
static const PRM_DEF_STRUCT cMonitorPara0212[] = 
{
	212,
	PRM_ATTR_SIZE(gPara.ControllerType),
	LOGIC_TYPE,	 	    			//min
	LOGIC_TYPE,    					//max
	LOGIC_TYPE,    //default
	(void*)&gPara.ControllerType,
//	"Version",
//	" 控制器逻辑类型 Move/lift/steer",
};
static const PRM_DEF_STRUCT cMonitorPara0213[] = 
{
	213,
	PRM_ATTR_SIZE(gPara.SoftVersion),
	0,	 	    				//min
	65535,    				//max
	SOFTVERSION,    	//default
	(void*)&gPara.SoftVersion,
//	"Version",
//	" 软件版本号: 低字节内核程序、高字节逻辑程序",
};
static const PRM_DEF_STRUCT cMonitorPara0214[] = 
{
	214,
	PRM_ATTR_SIZE(gPara.SoftVerYear),
	2016,	 	  //min
	2100,    	//max
	SOFT_YEAR,    		  //default
	(void*)&gPara.SoftVerYear,
//	"Version",
//	"Release date Year",
};
static const PRM_DEF_STRUCT cMonitorPara0215[] = 
{
	215,
	PRM_ATTR_SIZE(gPara.SoftVerMonthDay),
	0101,	 	    	//min
	1231,    	//max
	SOFT_MONTHDAY,    		  //default
	(void*)&gPara.SoftVerMonthDay,
//	"Version",
//	"Release date Month & Day",
};
static const PRM_DEF_STRUCT cMonitorPara0216[] = 
{
	216,
	PRM_ATTR_SIZE(gPara.HardVersion1),
	0,	 	    			//min
	65535,    			//max
	HARDVERSION1,   //default
	(void*)&gPara.HardVersion1,
//	"Version",
//	" 硬件版本号1",
};
static const PRM_DEF_STRUCT cMonitorPara0217[] = 
{
	217,
	PRM_ATTR_SIZE(gPara.HardVersion2),
	0,	 	    				//min
	65535,    				//max
	HARDVERSION2,    	//default
	(void*)&gPara.HardVersion2,
//	"Version",
//	" 硬件版本号2",
};
static const PRM_DEF_STRUCT cMonitorPara0218[] = 
{
	218,
	PRM_ATTR_SIZE(gPara.FrameVersion),
	0,	 	    	//min
	65535,    	//max
	0,    		  //default
	(void*)&gPara.FrameVersion,
//	"Version",
//	"结构版本",
};
static const PRM_DEF_STRUCT cMonitorPara0219[] = 
{
	219,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    	//min
	0,    	//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
	/* monitor state*/
static const PRM_DEF_STRUCT cMonitorPara0220[] = 
{
	220,
	PRM_ATTR_SIGNED | PRM_ATTR_SIZE(gPara.AcMotorSpeed),
	-30000,	 	    	//min
	30000,    	//max
	0,    		  //default
	(void*)&gPara.AcMotorSpeed,
//	"State",
//	"电机转速(rpm)",
};
static const PRM_DEF_STRUCT cMonitorPara0221[] = 
{
	221,
	PRM_ATTR_SIGNED | PRM_ATTR_SIZE(gPara.AcMotorSpeedF),
	-30000,	 	    	//min
	30000,    	//max
	0,    		  //default
	(void*)&gPara.AcMotorSpeedF,
//	"State",
//	"电机转速命令(rpm)",
};
static const PRM_DEF_STRUCT cMonitorPara0222[] = 
{
	222,
	PRM_ATTR_SIZE(gPara.AcPhaseCurrent),
	0,	 	    	//min
	1000,    	//max
	0,    		  //default
	(void*)&gPara.AcPhaseCurrent,
//	"State",
//	"电机相电流(A)",
};
static const PRM_DEF_STRUCT cMonitorPara0223[] = 
{
	223,
	PRM_ATTR_SIGNED | PRM_ATTR_SIZE(gPara.MotorTmp),
	-40,	 	    	//min
	165,    	//max
	0,    		  //default
	(void*)&gPara.MotorTmp,
//	"State",
//	"电机温度",
};
static const PRM_DEF_STRUCT cMonitorPara0224[] = 
{
	224,
	PRM_ATTR_SIGNED | PRM_ATTR_SIZE(gPara.ControllerTmp),
	-40,	 	    	//min
	125,    	//max
	0,    		  //default
	(void*)&gPara.ControllerTmp,
//	"State",
//	"控制器温度",
};
static const PRM_DEF_STRUCT cMonitorPara0225[] = 
{
	225,
	PRM_ATTR_SIZE(gPara.VBusVoltage),
	0,	 	    	//min
	2000,    	//max
	0,    		  //default
	(void*)&gPara.VBusVoltage,
//	"State",
//	"母线电压(0.1V)",
};
static const PRM_DEF_STRUCT cMonitorPara0226[] = 
{
	226,
	PRM_ATTR_SIZE(gPara.KsiVBusVoltage),
	0,	 	    	//min
	2000,    	//max
	0,    		  //default
	(void*)&gPara.KsiVBusVoltage,
//	"State",
//	"KSI电压(0.1V)",
};
static const PRM_DEF_STRUCT cMonitorPara0227[] = 
{
	227,
	PRM_ATTR_SIGNED | PRM_ATTR_SIZE(gPara.MoveSpeed),
	-1600,	 	    	//min
	1600,    	//max
	0,    		  //default
	(void*)&gPara.MoveSpeed,
//	"State",
//	"行驶速度(0.1Km/h)",
};
static const PRM_DEF_STRUCT cMonitorPara0228[] = 
{
	228,
	PRM_ATTR_SIZE(gPara.SteerAngle),
	0,	 	    	//min
	65535,    	//max
	0,    		  //default
	(void*)&gPara.SteerAngle,
//	"State",
//	"转向角度",
};
static const PRM_DEF_STRUCT cMonitorPara0229[] = 
{
	229,
	PRM_ATTR_SIZE(gPara.ErrCode),
	0,	 	    	//min
	65535,    	//max
	0,    		  //default
	(void*)&gPara.ErrCode,
//	"State",
//	"错误号",
};
static const PRM_DEF_STRUCT cMonitorPara0230[] = 
{
	230,
	PRM_ATTR_SIZE(gPara.ThrottleCmdSpd),
	0,	 	    	//min
	65535,    	//max
	0,    		  //default
	(void*)&gPara.ThrottleCmdSpd,
//	"State",
//	"油门踏板指令(%)",
};
static const PRM_DEF_STRUCT cMonitorPara0231[] = 
{
	231,
	PRM_ATTR_SIZE(gPara.ThrottleWipVoltage),
	0,	 	    	//min
	2000,    	//max
	0,    		  //default
	(void*)&gPara.ThrottleWipVoltage,
//	"State",
//	"油门踏板指令电压(0.1v)",
};
static const PRM_DEF_STRUCT cMonitorPara0232[] = 
{
	232,
	PRM_ATTR_SIZE(gPara.ThrottleHighVoltage),
	0,	 	    	//min
	2000,    	//max
	0,    		  //default
	(void*)&gPara.ThrottleHighVoltage,
//	"State",
//	"油门电位计高端电压(0.1V)",
};
static const PRM_DEF_STRUCT cMonitorPara0233[] = 
{
	233,
	PRM_ATTR_SIZE(gPara.PotLowVoltage),
	0,	 	    	//min
	2000,    	//max
	0,    		  //default
	(void*)&gPara.PotLowVoltage,
//	"State",
//	"电位计低端电压(0.1V)",
};
static const PRM_DEF_STRUCT cMonitorPara0234[] = 
{
	234,
	PRM_ATTR_SIZE(gPara.Analog1),
	0,	 	    	//min
	2000,    	//max
	0,    		  //default
	(void*)&gPara.Analog1,
//	"State",
//	"模拟量1输入(0.1V)",
};
static const PRM_DEF_STRUCT cMonitorPara0235[] = 
{
	235,
	PRM_ATTR_SIZE(gPara.Analog2),
	0,	 	    	//min
	2000,    	//max
	0,    		  //default
	(void*)&gPara.Analog2,
//	"State",
//	"模拟量2输入(0.1V)",
};
static const PRM_DEF_STRUCT cMonitorPara0236[] = 
{
	236,
	PRM_ATTR_SIZE(gPara.Analog3),
	0,	 	    	//min
	2000,    	//max
	0,    		  //default
	(void*)&gPara.Analog3,
//	"State",
//	"模拟量3输入(0.1V)",
};
static const PRM_DEF_STRUCT cMonitorPara0237[] = 
{
	237,
	PRM_ATTR_SIZE(gPara.Analog4),
	0,	 	    	//min
	2000,    	//max
	0,    		  //default
	(void*)&gPara.Analog4,
//	"State",
//	"模拟量4输入(0.1V)",
};
static const PRM_DEF_STRUCT cMonitorPara0238[] = 
{
	238,
	PRM_ATTR_SIZE(gPara.Switch),
	0,	 	    	//min
	2000,    	//max
	0,    		  //default
	(void*)&gPara.Switch,
//	"State",
//	"开关输入",
};
static const PRM_DEF_STRUCT cMonitorPara0239[] = 
{
	239,
	PRM_ATTR_SIZE(gPara.Driver),
	0,	 	    	//min
	65535,    	//max
	0,    		  //default
	(void*)&gPara.Driver,
//	"State",
//	"驱动输出",
};
static const PRM_DEF_STRUCT cMonitorPara0240[] = 
{
	240,
	PRM_ATTR_SIZE(gPara.MotorEncoder),
	0,	 	    	//min
	65535,    	//max
	0,    		  //default
	(void*)&gPara.MotorEncoder,
//	"State",
//	"电机编码器输入",
};
static const PRM_DEF_STRUCT cMonitorPara0241[] = 
{
	241,
	PRM_ATTR_SIZE(gPara.DirectionEncoder1),
	0,	 	    	//min
	65535,    	//max
	0,    		  //default
	(void*)&gPara.DirectionEncoder1,
//	"State",
//	"方向盘编码器输入1",
};
static const PRM_DEF_STRUCT cMonitorPara0242[] = 
{
	242,
	PRM_ATTR_SIZE(gPara.DirectionEncoder2),
	0,	 	    	//min
	65535,    	//max
	0,    		  //default
	(void*)&gPara.DirectionEncoder2,
//	"State",
//	"方向盘编码器输入2",
};
static const PRM_DEF_STRUCT cMonitorPara0243[] = 
{
	243,
	PRM_ATTR_SIZE(gPara.KernelState),
	0,	 	    	//min
	65535,    	//max
	0,    		  //default
	(void*)&gPara.KernelState,
//	"State",
//	"控制环状态监测",
};
static const PRM_DEF_STRUCT cMonitorPara0244[] = 
{
	244,
	PRM_ATTR_SIZE(gPara.BrakePedalCmdSpd),
	0,	 	    	//min
	65535,    	//max
	0,    		  //default
	(void*)&gPara.BrakePedalCmdSpd,
//	"State",
//	"制动踏板指令(%)",
};
static const PRM_DEF_STRUCT cMonitorPara0245[] = 
{
	245,
	PRM_ATTR_SIZE(gPara.BrakePedalWipVoltage),
	0,	 	    	//min
	2000,    	//max
	0,    		  //default
	(void*)&gPara.BrakePedalWipVoltage,
//	"State",
//	"制动踏板指令电压(0.1v)",
};
static const PRM_DEF_STRUCT cMonitorPara0246[] = 
{
	246,
	PRM_ATTR_SIZE(gPara.BrakePedalHighVoltage),
	0,	 	    	//min
	2000,    	//max
	0,    		  //default
	(void*)&gPara.BrakePedalHighVoltage,
//	"State",
//	"制动踏板高端电压(0.1V)",
};

	/* Vehicle */
static const PRM_DEF_STRUCT cMonitorPara0247[] = 
{
	247,
	PRM_ATTR_SIZE(gPara.TimetoSpd1),
	0,	 	    	//min
	60000,    	//max
	0,    		  //default
	(void*)&gPara.TimetoSpd1,
//	"State",
//	"Time Capture Speed 1.unit 5ms",
};
static const PRM_DEF_STRUCT cMonitorPara0248[] = 
{
	248,
	PRM_ATTR_SIZE(gPara.TimetoSpd2),
	0,	 	    	//min
	60000,    	//max
	0,    		  //default
	(void*)&gPara.TimetoSpd2,
//	"State",
//	"Time Capture Speed 2.unit 5ms",
};
static const PRM_DEF_STRUCT cMonitorPara0249[] = 
{
	249,
	PRM_ATTR_SIZE(gPara.TimetoDist1),
	0,	 	    	//min
	60000,    	//max
	0,    		  //default
	(void*)&gPara.TimetoDist1,
//	"State",
//	"Time Capture distance1.unit 5ms",
};
static const PRM_DEF_STRUCT cMonitorPara0250[] = 
{
	250,
	PRM_ATTR_SIZE(gPara.TimetoDist2),
	0,	 	    	//min
	60000,    	//max
	0,    		  //default
	(void*)&gPara.TimetoDist2,
//	"State",
//	"Time Capture distance2.unit 5ms",
};
static const PRM_DEF_STRUCT cMonitorPara0251[] = 
{
	251,
	PRM_ATTR_SIZE(gPara.TimetoDist3),
	0,	 	    	//min
	60000,    	//max
	0,    		  //default
	(void*)&gPara.TimetoDist3,
//	"State",
//	"Time Capture distance3.unit 5ms",
};
static const PRM_DEF_STRUCT cMonitorPara0252[] = 
{
	252,
	PRM_ATTR_SIZE(gPara.AnologOut),
	0,	 	    	//min
	2000,    	//max
	0,    		  //default
	(void*)&gPara.AnologOut,
//	"State",
//	"模拟量输出(0.1V)",
};
static const PRM_DEF_STRUCT cMonitorPara0253[] = 
{
	253,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    	//min
	0,    	//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cMonitorPara0254[] = 
{
	254,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    	//min
	0,    	//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
static const PRM_DEF_STRUCT cMonitorPara0255[] = 
{
	255,
	PRM_ATTR_SIZE(gPara.NullPara),
	0,	 	    	//min
	0,    	//max
	0,    		  //default
	(void*)&gPara.NullPara,
//	"Rsv",
//	"Rsv",
};
//static 
const PRM_DEF_STRUCT* cMonitorPara_Table[] = 
{
	cMonitorPara0210, cMonitorPara0211, cMonitorPara0212, cMonitorPara0213, cMonitorPara0214, cMonitorPara0215, cMonitorPara0216, cMonitorPara0217, cMonitorPara0218, cMonitorPara0219,
	cMonitorPara0220, cMonitorPara0221, cMonitorPara0222, cMonitorPara0223, cMonitorPara0224, cMonitorPara0225, cMonitorPara0226, cMonitorPara0227, cMonitorPara0228, cMonitorPara0229,
	cMonitorPara0230, cMonitorPara0231, cMonitorPara0232, cMonitorPara0233, cMonitorPara0234, cMonitorPara0235, cMonitorPara0236, cMonitorPara0237, cMonitorPara0238, cMonitorPara0239,
	cMonitorPara0240, cMonitorPara0241, cMonitorPara0242, cMonitorPara0243, cMonitorPara0244, cMonitorPara0245, cMonitorPara0246, cMonitorPara0247, cMonitorPara0248, cMonitorPara0249,
	cMonitorPara0250, cMonitorPara0251, cMonitorPara0252, cMonitorPara0253, cMonitorPara0254, cMonitorPara0255
};
/*Hmi Monitor*/
static const PRM_DEF_STRUCT* cMoveMonitor_Table[] = 
{
	cMonitorPara0220, cMonitorPara0221, cMonitorPara0222, cMonitorPara0223, cMonitorPara0224, cMonitorPara0225, cMonitorPara0226, 
	cMonitorPara0230, cMonitorPara0231,  cMonitorPara0238,  cMonitorPara0239,  cMonitorPara0240 
};
INT8U GetPrmNoFromMoveMonitorTab(INT16U Index)
{
	if(Index >= (sizeof(cMoveMonitor_Table)/sizeof(cMoveMonitor_Table[0])))
		Index = 0;
	return cMoveMonitor_Table[Index]->PrmNo;
}
static const PRM_DEF_STRUCT* cLiftMonitor_Table[] = 
{
	cMonitorPara0220, cMonitorPara0221, cMonitorPara0222, cMonitorPara0223, cMonitorPara0224, cMonitorPara0225, cMonitorPara0226, 
	cMonitorPara0230, cMonitorPara0231,  cMonitorPara0238,  cMonitorPara0239,  cMonitorPara0240 
};
INT16U GetPrmNoFromLiftMonitorTab(INT16U Index)
{
	if(Index >= (sizeof(cLiftMonitor_Table)/sizeof(cLiftMonitor_Table[0])))
		Index = 0;
	return cLiftMonitor_Table[Index]->PrmNo;
}
static const PRM_DEF_STRUCT* cSteerMonitor_Table[] = 
{
	cMonitorPara0220, cMonitorPara0221, cMonitorPara0222, cMonitorPara0223, cMonitorPara0224, cMonitorPara0225, cMonitorPara0226,
	cMonitorPara0238,  cMonitorPara0239,  cMonitorPara0240,   cMonitorPara0241 
};
INT16U GetPrmNoFromSteerMonitorTab(INT16U Index)
{
	if(Index >= (sizeof(cSteerMonitor_Table)/sizeof(cSteerMonitor_Table[0])))
		Index = 0;
	return cSteerMonitor_Table[Index]->PrmNo;
}

/*******************************************************************************
* Name: ReadMonitorParaValByIndex
* Description: 
* Input: PrmNo-- monitor data prm No
* Output: Para val
* 
* Author:
* Date:
* Revision:
*******************************************************************************/
INT16U ReadMonitorParaValByIndex(INT16U PrmNo)
{
	INT32U Index;
	
	Index = (INT32U)PrmNo - cMonitorPara_Table[0]->PrmNo;
	
	if (Index < (sizeof(cMonitorPara_Table)/sizeof(cMonitorPara_Table[0])))
	{
		return *((INT16U*)cMonitorPara_Table[Index]->pPrmData);
	}
	else
	{
		return 0;
	}
}
/*******************************************************************************
* Name: SaveMonitorParaValByIndexToRam
* Description: 
* Input: PrmNo-- Monitor data prm No
* Output: 0: success. 1: fail
* 
* Author:
* Date:
* Revision:
*******************************************************************************/
INT16U SaveMonitorParaValByIndexToRam(INT16U PrmNo, INT16U Val)
{
	INT16U ret;
	INT32U Index;
	PRM_ATTR_STRUCT* pPrmAttr;
	PRM_DEF_STRUCT* pPrmDef;

	Index = (INT32U)PrmNo - cMonitorPara_Table[0]->PrmNo;
	if (Index >= (sizeof(cMonitorPara_Table)/sizeof(cMonitorPara_Table[0])))
	{
		SL_SET(PLC_PARA_OV_INDEX_ERR);
		return 1;
	}

	pPrmDef = (PRM_DEF_STRUCT*)(cMonitorPara_Table[Index]);
	pPrmAttr = (PRM_ATTR_STRUCT*)(&pPrmDef->PrmAttr);
	ret = 0;
	while(1)
	{
		/* if over limit */
		if (pPrmAttr->Signed == 1)
		{
			if(((INT16S)Val > pPrmDef->PrmMaxVal) || ((INT16S)Val < pPrmDef->PrmMinVal))
			{
				/* the parameter in eeprom is not in proper range */
				ret = 1;
				SL_SET(PLC_PARA_OV_LIMIT_ERR);
				break;
			}
		}
		else
		{
			if((Val > (INT16U)pPrmDef->PrmMaxVal) || (Val < (INT16U)pPrmDef->PrmMinVal))
			{
				/* the parameter in eeprom is not in proper range */
				ret = 1;
				SL_SET(PLC_PARA_OV_LIMIT_ERR);
				break;
			}
		}
		/* if not over limit, then save to ram */
		*((INT16U*)pPrmDef->pPrmData) = Val;
		break;
	}
	return ret;

}

/*******************************************************************************
* Name: InitPara
* Description: 
* 1 Check Hard Para. If checksum illegle, set Hard Para to default
* 2 Check User type. If vary , set all para  to default. not include Hard para. include hour & bat para
* 3 Set all monitor data to default 
* Input: 
* Output: 
* 
* Author:
* Date:
* Revision:
*******************************************************************************/
void InitPara(void)
{
	int index;
	INT16U sum, flag;
	PRM_ATTR_STRUCT* pPrmAttr;
	PRM_DEF_STRUCT* pPrmDef;
//	InitEEPROM();
	//Read  & check hardware para
	sum = 0;
	flag = 0;
	for (index=0; index < (sizeof(cHardPara_Table)/sizeof(cHardPara_Table[0])); index++)
	{
		flag += ReadHardParaFromEeprom(index);
		sum += *((INT16U*)cHardPara_Table[index]->pPrmData);
	}
	if (((flag | sum) & 0xffff) != 0)
	{ //Data error in eprom, set para to default value. 
		for (index=0; index < (sizeof(cHardPara_Table)/sizeof(cHardPara_Table[0])); index++)
		{
			if (SaveHardParaToEeprom(index,cHardPara_Table[index]->PrmInitVal) == 0)
				ReadHardParaFromEeprom(index);
			else
				*((INT16U*)cHardPara_Table[index]->pPrmData) = cHardPara_Table[index]->PrmInitVal;
		}
	}
	//Read eprom para volume
	if ((ReadParaFromEeprom(PARA_VOLUME_NUM) != 0)
		  || (gPara.UserType != cPara_Table[PARA_VOLUME_NUM]->PrmInitVal))
	{ //para invalid, to default val
		for(index=0;index < (sizeof(cPara_Table)/sizeof(cPara_Table[0]));index++) 
		{
			pPrmDef = (PRM_DEF_STRUCT*)cPara_Table[index];
			pPrmAttr = (PRM_ATTR_STRUCT*)(&pPrmDef->PrmAttr);
			if (pPrmAttr->Hard == 0)
			{
				if (SaveParaToEeprom(index,cPara_Table[index]->PrmInitVal) == 0)
					ReadParaFromEeprom(index);
				else
					*((INT16U*)cPara_Table[index]->pPrmData) = cPara_Table[index]->PrmInitVal;
			}
			else
			{/*Hard para keep value*/}
		}
	}
	else //para valid, read para from eprom and rom
	{
		for(index=0;index < (sizeof(cPara_Table)/sizeof(cPara_Table[0]));index++) 
		{
			ReadParaFromEeprom(index);
		}
	}
	
	for (index=0; index < (sizeof(cMonitorPara_Table)/sizeof(cMonitorPara_Table[0])); index++)
	{
		pPrmDef = (PRM_DEF_STRUCT*)(cMonitorPara_Table[index]);
		*((INT16U*)pPrmDef->pPrmData) = pPrmDef->PrmInitVal;
	}
}

