/*******************************************************************************
* Filename: LocalAi.c 	                                    	     	       *
* Description: 	LocalAi功能					           				           *
* Author:  QExpand, Lilu                                                       *
* Date: 2023/5/16															   *
* Revision:	V1.00														       *
*******************************************************************************/
#include "LocalAi.h"
#include "ErrCode.h"
#include "Log.h"
#include "Device.h"
#include "LocalDo.h"
/*******************************************************************************
* Name: int32_t i32LocalAiGet(eAiNo AiNo)
* Descriptio: 获取Ai通道数据
* Input: AiNo：Ai通道编号
* Output:  -1:    输入参数错误
		   其它值：Ai通道对应的AD值
*******************************************************************************/
int32_t i32LocalAiGet(eAiNo AiNo)
{
	int32_t res = 0;
	
	if(AiNo >= LocalAiMax)
	{
		i32LogWrite(ERR, LOG_AI, "LocalAiNo Parameter is wrong, LocalAi = %d\r\n", AiNo);
		return -1;
	}
	if (((AI_B_AI1_R == AiNo) && (0 != i32ErrCodeCheck(ErrCode71))) ||
	    ((AI_B_AI2_R == AiNo) && (0 != i32ErrCodeCheck(ErrCode72))) || 
	    ((AI_B_AI3_R == AiNo) && (0 != i32ErrCodeCheck(ErrCode73))) || 
		((AI_5V_12V_OUT1_I == AiNo) && (0 != i32ErrCodeCheck(ErrCode74))) || 
		((AI_5V_12V_OUT2_I == AiNo) && (0 != i32ErrCodeCheck(ErrCode75))))
	{
		res = 0;
	}
	else
	{
		res =  ADCValue[AiNo];
	}
#if 0	
	switch(AiNo)
	{
		case AI_B_KSI_CHECK:
			res = 0;
			break;
		case AI_B_VBUS_CHECK:
			res = 0;
			break;
		case AI_B_AI1_R:
			res = 0;
			break;
		case AI_B_AI2_R:
			res = 0;
			break;
		case AI_B_AI3_R:
			res = 0;
			break;
		case AI_5V_12V_OUT2_I:
			res = 0;
			break;
		case AI_5V_12V_OUT2_R:
			res = 0;
			break;
		case AI_5V_12V_OUT1_I:
			res = 0;
			break;
		case AI_5V_CHECK:
			res = 0;
			break;
		case AI_15V_CHECK:
			res = 0;
			break;
		case AI_5V_12V_OUT1_R:
			res = 0;
			break;
		case AI_3V3_CHECK:
			res = 0;
			break;
		case AI_CHANNEL13:
			res = 0;
			break;
		case AI_CHANNEL14:
			res = 0;
			break;	
		case AI_CHANNEL15:
			res = 0;
			break;	
		case AI_CHANNEL16:
			res = 0;
			break;			
		default:
			break;
		
	}
#endif	
	return res;		
}


#define	AI_B_KSI_CHECK_FACTOR		(3300 * (6.8 + 143) / 4096 / 6.8)		/*value = ADCvalue * (3300  * (6.8 + 143) / 4096 / 6.8)*/
#define	AI_B_VBUS_CHECK_FACTOR		(3300 * (6.8 + 143) / 4096 / 6.8)		/*value = ADCvalue * (3300  * (6.8 + 143) / 4096 / 6.8)*/
#define	AI_B_AI1_R_FACOTR			(3300.0 * 2 / 4096)						/*value = ADCvalue * (3300 * (200 + 200) / 4096 / 200)*/
#define	AI_B_AI2_R_FACOTR			(3300.0 * 2 / 4096)						/*value = ADCvalue * (3300 * (200 + 200) / 4096 / 200)*/
//#define	AI_B_AI2_R_FACOTR			(3300.0 * 282 / 4096 / 82)						/*value = ADCvalue * (3300 * (200 + 82) / 4096 / 82)*/
//#define	AI_B_AI3_R_FACOTR			(3300.0 * 2 / 4096)						/*value = ADCvalue * (3300 * (200 + 200) / 4096 / 200)*/
#define	AI_B_AI3_R_FACOTR			(3300.0 * 282 / 4096 / 82)						/*value = ADCvalue * (3300 * (200 + 82) / 4096 / 82)*/
#define	AI_5V_12V_OUT2_I_FACTOR		(3300 * 11.2 / 4096)					/*value = ADCvalue * (3300 * 11.2 / 4096)*/
#define	AI_5V_12V_OUT2_R_FACTOR		(3300.0 * 6 / 4096)						/*value = ADCvalue * (3300 * (100 + 20) / 4096 / 20)*/
#define	AI_5V_12V_OUT1_I_FACTOR		(3300 * 11.2 / 4096)					/*value = ADCvalue * (3300 * 11.2 / 4096)*/
#define	AI_5V_CHECK_FACTOR			(3300.0 * 2 / 4096)						/*value = ADCvalue * (3300 * (6.8 + 6.8) / 4096 / 6.8)*/	
#define	AI_15V_CHECK_FACTOR			(3300 / 4096 * (6.8 + 430) / 6.8)		/*value = ADCvalue * (3300 * (6.8 + 43) / 4096 / 6.8)*/
#define	AI_5V_12V_OUT1_R_FACTOR		(3300.0 * 6 / 4096)						/*value = ADCvalue * (3300 * (100 + 20) / 4096 / 20)*/
#define	AI_3V3_CHECK_FACTOR			(3300.0 * 2 / 4096)						/*value = ADCvalue * (3300 * (6.8 + 6.8) / 4096 / 6.8)*/

//23.12.14SJ
#define AI_B_AI1_CURRENT_FACOTR	(3300.0 * 2 / 4096 / 249)
#define AI_B_AI2_CURRENT_FACOTR	(3300.0 * 2 / 4096 / 249)
#define AI_B_AI3_CURRENT_FACOTR	(3300.0 * 282 / 4096 / 82 / 249)
int32_t i32LocalAiGetValue(eAiNo AiNo)
{
	uint32_t res;
	if(AiNo >= LocalAiMax)
	{
		i32LogWrite(ERR, LOG_AI, "LocalAiNo Parameter is wrong, LocalAi = %d\r\n", AiNo);
		return -1;
	}
	
	switch(AiNo)
	{	
		case AI_B_KSI_CHECK:
			res = ADCValue[AI_B_KSI_CHECK] * AI_B_KSI_CHECK_FACTOR;
			break;
		case AI_B_VBUS_CHECK:
			res = ADCValue[AI_B_VBUS_CHECK] * AI_B_VBUS_CHECK_FACTOR;
			break;
		case AI_B_AI1_R:
			if (0 != i32ErrCodeCheck(ErrCode71))
			{
				res = 0;
			}
			else
			{
				res = ADCValue[AI_B_AI1_R] * AI_B_AI1_R_FACOTR;
			}
			break;
		case AI_B_AI2_R:
			if (0 != i32ErrCodeCheck(ErrCode72))
			{
				res = 0;
			}
			else
			{
				res = ADCValue[AI_B_AI2_R] * AI_B_AI2_R_FACOTR;
			}
			break;
		case AI_B_AI3_R:
			if (0 != i32ErrCodeCheck(ErrCode73))
			{
				res = 0;
			}
			else
			{
				res = ADCValue[AI_B_AI3_R] * AI_B_AI3_R_FACOTR;
			}
			break;
		case AI_5V_12V_OUT2_I:
			if (0 != i32ErrCodeCheck(ErrCode75))
			{
				res = 0;
			}
			else
			{
				res = ADCValue[AI_5V_12V_OUT2_I] * AI_5V_12V_OUT2_I_FACTOR;
			}
			break;
		case AI_5V_12V_OUT2_R:
			res = ADCValue[AI_5V_12V_OUT2_R] * AI_5V_12V_OUT2_R_FACTOR;
			break;
		case AI_5V_12V_OUT1_I:
			if (0 != i32ErrCodeCheck(ErrCode74))
			{
				res = 0;
			}
			else
			{
				res = ADCValue[AI_5V_12V_OUT1_I] * AI_5V_12V_OUT1_I_FACTOR;
			}
			break;
		case AI_5V_CHECK:
			res = ADCValue[AI_5V_CHECK] * AI_5V_CHECK_FACTOR;
			break;
		case AI_15V_CHECK:
			res = ADCValue[AI_15V_CHECK] * AI_15V_CHECK_FACTOR;
			break;
		case AI_5V_12V_OUT1_R:
			res = ADCValue[AI_5V_12V_OUT1_R] * AI_5V_12V_OUT1_R_FACTOR;
			break;
		case AI_3V3_CHECK:
			res = ADCValue[AI_3V3_CHECK] * AI_3V3_CHECK_FACTOR;
			break;
		case AI_B_AI1_CURRENT:	//23.12.14SJ
			if (0 != i32ErrCodeCheck(ErrCode71))
			{
				res = 0;
			}
			else
			{
				i32LocalDoSet(DO_ANALOG_1, 1);
				res = ADCValue[AI_B_AI1_R] * AI_B_AI1_CURRENT_FACOTR;
			}
			break;
		case AI_B_AI2_CURRENT:
			if (0 != i32ErrCodeCheck(ErrCode72))
			{
				res = 0;
			}
			else
			{
				i32LocalDoSet(DO_ANALOG_2, 1);
				res = ADCValue[AI_B_AI2_R] * AI_B_AI2_CURRENT_FACOTR;
			}
			break;
		case AI_B_AI3_CURRENT:
			if (0 != i32ErrCodeCheck(ErrCode73))
			{
				res = 0;
			}
			else
			{
				i32LocalDoSet(DO_ANALOG_3, 1);
				res = ADCValue[AI_B_AI3_R] * AI_B_AI3_CURRENT_FACOTR;
			}
			break;
		default:
			break;	
	}

	return res;
}
 