#include "UserCanComm.h"
#include "User_CanOpenHangChaApp.h"
#include "ErrCode.h"
#include "Eeprom.h"
#include "IQmathLib.h"
#include "PropProc.h"
#include "CanRevProc.h"
#include "string.h"
#include "Para.h"
#include "WdgProc.h"
#include "math.h"
#include "Log.h"
#include "LocalAi.h"
#include "LedProc.h"
#include "AiProc.h"
#include "LocalDo.h"
#include "HourCount.h"
#include "MstSlvCom.h"

#if((USER_TYPE == USER_HANGCHA_QYDGC)||(USER_TYPE == USER_HANGCHA_PHZ))

/*******************************************************************************
* FunctionName: vGetRandPwd
* Description:  用于生成随机数密码（字母大小写、数字），无请求时只生成一次，直至下次请求
* Input: _tHANGCHADGC_PDO *pCanopenPDO 
* Output: None
*
* Author: 
* Date: 2024-01-06
* Revision: V1.0
*******************************************************************************/
void vGetRandPwd(_tHANGCHADGC_PDO *pCanopenPDO, INT8U length)
{
	INT8U i;
	static INT32U TempPwd;
	const char str[] = "xaqt";//"XAQT";
	static uint32_t u32Seed = 0;
	
	u32Seed++;
	//如果解密错误 则允许生成下一次随机密码
	if(pCanopenPDO->GeneratePwdFlag == 0)
	{
		srand(TIMER_CNT(u32Seed));
		TempPwd= (INT32U)rand() + 0x20;
		pCanopenPDO->RandPwdArr[0] = (char)((TempPwd >> 24) & 0xFF);		
		pCanopenPDO->RandPwdArr[1] = (char)((TempPwd >> 16) & 0xFF);		
		pCanopenPDO->RandPwdArr[2] = (char)((TempPwd >> 8) & 0xFF);	
		pCanopenPDO->RandPwdArr[3] = (char)(TempPwd  & 0xFF);

   for(i = 0;i < 4; i++)
	 {
		 //if (!isalnum(pCanopenPDO->RandPwdArr[i]))  //可能由于编译器问题导致部分字符识别不出
			if(  (pCanopenPDO->RandPwdArr[i] < 0x30) 
				|| ((pCanopenPDO->RandPwdArr[i] >= 0x3A) &&(pCanopenPDO->RandPwdArr[i] <= 0x40))
				|| ((pCanopenPDO->RandPwdArr[i] >= 0x5B) &&(pCanopenPDO->RandPwdArr[i] <= 0x60)) 
			  || (pCanopenPDO->RandPwdArr[i] >= 0x7A) )
		 {
			 memcpy(&pCanopenPDO->RandPwdArr[i],&str[i], sizeof(str[i]));  //生成“XAQT”
		 }
	 }

	pCanopenPDO->GeneratePwdFlag = 1;  //密码校验错误时需将该标志位清0
  }
}
/*******************************************************************************
* FunctionName: vGetReplaceChar(char *oldpwd,char *newpwd)
* Description: 该函数用于将随机密码与给定字符串,做指定替换，得到一个原始密码
* Input: char *oldpwd
* Output: char *newpwd
*
* Author: 
* Date: 2024-1-6
* Revision: V1.0
*******************************************************************************/
void vGetReplaceChar(char *oldpwd,char *newpwd)
{
	INT8U i;
	INT8U found = 0;

// 定义映射数组  加密步骤一
CharMapping mappings[] = { {'1', '\\'},	{'2', '!'},  {'3', '@'}, {'4', '#'},
													 {'5', '&'},	{'6', '%'},  {'7', '?'}, {'8', '*'},
													 {'9', '^'},	{'0', '/'},  {'d', '1'}, {'m', '1'},
												   {'o', '1'},	{'j', '2'},  {'y', '2'}, {'z', '2'},
									         {'b', '3'},	{'g', '3'},  {'r', '3'}, {'k', '4'},
												   {'t', '4'},	{'u', '4'},  {'a', '5'}, {'h', '5'},
												   {'x', '5'},	{'c', '6'},  {'p', '6'}, {'n', '6'},
												   {'f', '7'},	{'q', '7'},  {'v', '7'}, {'e', '8'},
												   {'l', '8'},	{'i', '8'},  {'w', '9'}, {'s', '9'}};

/*根据生成的随机密码在mappings的original哪个位置，然后根据映射关系，找出替代字符，大写不变*/
    while (*oldpwd)
		{
			for (i = 0; i < sizeof(mappings) / sizeof(CharMapping); ++i) 
			{
				if (*oldpwd == mappings[i].original) 
				{
					*newpwd++ = mappings[i].replacement;
					found = 1;
					break;
				}
			}
			if(!found)//如果未找到映射关系，直接赋值字符
			{
				*newpwd++ = *oldpwd;		
				found		 = 0;	
				if(*oldpwd == 0x30)  *newpwd++ = 0x2F;
			}
			// 移动到下一个字符
			++oldpwd;
    }
    // 添加字符串结束符
   //  *newpwd = '\0';
}
/*******************************************************************************
* FunctionName: vGenerateKeyPro
* Description: 根据得到的原始密码，然后进行四次加密换算
* Input: char *Original
* Output: char *CodeKey
*
* Author: 
* Date: 2024-01-11
* Revision: V1.0
*******************************************************************************/
void vGenerateKeyPro(char *Original,char *CodeKey)
{
	char TempKey[4];
	INT8U step;
	INT8U i,j;
	INT8U indexInMappings;
	char currentChar;
	CharMapping mappings2[] = {{'c', 'S'},	{'9', '5'},  {'j', 'd'}, {'^', 'f'},
														 {'p', 'v'},	{'@', 'D'},  {'#', '8'}, {'5', 'g'},
														 {'q', 't'},	{'l', '^'},  {'a', 'U'}, {'4', 'I'},
														 {'\\', '9'},	{'b', '2'},  {'m', '@'}, {'0', 'K'},
														 {'z', 'w'},	{'g', '/'},  {'&', 'P'}, {'i', 'L'},
														 {'e', 'Q'},	{'3', 'n'},  {'1', 'm'}, {'r', '%'},
														 {'s', 'Y'},	{'/', 'r'},  {'t', 'W'}, {'%', 'z'},
														 {'7', 'x'},	{'d', 'c'},  {'v', 'k'}, {'f', '!'},
														 {'*', 'V'},	{'x', 'B'},  {'o', 'E'}, {'h', 'a'},
														 {'n', '4'},	{'u', 'M'},  {'!', '3'}, {'k', '7'}};
    for (step = 0; step < 4; step++)  //四次加密
		{	
      for(i = 0; i < 4; i++)	//四位密码迭代
			{		
		//1、根据原始字符现将密码中的大写字母全部换成小写
		//2、先找出原始替换字符在mappings2中的index；
    //3、根据index映射到替换字符				
				currentChar = Original[i];
				indexInMappings = (INT8U)-1;
        if((currentChar >= 0x41) && (currentChar <= 0x5A))
				{
					currentChar += 0x20;
				}
				// 查找字符在映射表中的位置
				for (j = 0; j < sizeof(mappings2) / sizeof(CharMapping); j++) 
				{
					if (currentChar == mappings2[j].original) 
					{
						indexInMappings = j;
						break;
					}
				}

				if (indexInMappings != (INT8U)-1)
				{
					TempKey[i] = mappings2[indexInMappings].replacement;
				} 
				else 
				{
					TempKey[i] = Original[i];  // 找不到的保留原字符
				}
      }
       // 用 TempKey 作为下一轮加密的输入
      for (i = 0; i < 4; i++)
			{
				Original[i] = TempKey[i];
      }
    }
		//最后一次加密后  将 TempKey的结果拷贝到 CodeKey 中
		for(i = 0; i < 4; i++)
		{
			CodeKey[i] = TempKey[i];
		}								 

}

/*******************************************************************************
* FunctionName: iRemoteUnlockPWD
* Description:  将最终加密的密码与APP解析密码对比，一致return 1
* Input: _tHANGCHADGC_PDO  *pCanopenPDO, char *myParsepwd
* Output: char *myParsepwd
* 
* Author: 
* Date:2024-01-08
* Revision:V1.0
*******************************************************************************/
INT8U iRemoteUnlockPWD(_tHANGCHADGC_PDO  *pCanopenPDO,char *myParsepwd)
{	
	INT8U i;
	char ReceivePwd[4];

	ReceivePwd[0] = pCanopenPDO->DgnDataRxTx.DataL16 & 0xFF;
	ReceivePwd[1] = (pCanopenPDO->DgnDataRxTx.DataL16>>8)  & 0xFF;
	ReceivePwd[2] =  pCanopenPDO->DgnDataRxTx.DataH16 & 0xFF;
	ReceivePwd[3] = (pCanopenPDO->DgnDataRxTx.DataH16 >> 8) & 0xFF;	
	
  for (i = 0; i < 4; ++i)
	{
		if (ReceivePwd[i] != (unsigned char)myParsepwd[i])
		{
			pCanopenPDO->GeneratePwdFlag = 0;	
			return 0;  // 比较失败
		}
  }
    return 1;
}


void vIndexToAdress(uint32_t u32RecieveAdress,uint16_t *ad,uint16_t *fa)
{
	uint16_t u16SendAdress;
	uint16_t u16Factor;
	switch(u32RecieveAdress)
	{
		case 0x200C14:
			u16SendAdress = 16;
			u16Factor = 1;
		break;
		
		case 0x202001:
		case 0x202003:
		case 0x202004:
			u16SendAdress = 131;
			u16Factor = 2;
		break;
		case 0x202002:
			u16SendAdress = 141;
			u16Factor = 3;
		break;
		case 0x202005:
		case 0x201C4B:
			u16SendAdress = 142;
			u16Factor = 4;
		break;
		case 0x202006:
		case 0x201C57:
			u16SendAdress = 143;
			u16Factor = 5;
		break;
		case 0x202007:
			u16SendAdress = 145;
			u16Factor = 6;       //2
		break;
		case 0x202008:
			u16SendAdress = 144;
			u16Factor = 7;
		break;
		case 0x202009:
			u16SendAdress = 146;
			u16Factor = 8;
		break;
//		case 0x20200A:
//			u16SendAdress = 131;
//			u16Factor = 1;
//		break;
		case 0x20200B:
			u16SendAdress = 84;
			u16Factor = 9;
		break;
		
		case 0x20200C:
			u16SendAdress = 83;
			u16Factor = 10;
		break;
		
/*比例制动控制*/
		case 0x201F0A:
			u16SendAdress = 36;
			u16Factor = 106;
		break;
		
		case 0x201F0B:
			u16SendAdress = 101;
			u16Factor = 1;
		break;
		case 0x201F0C:
			u16SendAdress = 102;
			u16Factor = 1;
		break;
		case 0x201F0D:
			u16SendAdress = 103;
			u16Factor = 1;
		break;
		case 0x201F0E:
			u16SendAdress = 104;
			u16Factor = 1;
		break;
		case 0x201F0F:
			u16SendAdress = 105;
			u16Factor = 1;
		break;
		
		
		case 0x202102:
			u16SendAdress = 68;
			u16Factor = 11;
		break;
		case 0x202103:
			u16SendAdress = 69;
			u16Factor = 12;
		break;
		case 0x202130:
			u16SendAdress = 85;
			u16Factor = 13;
		break;
		
		case 0x202201:
		case 0x202202:	
			u16SendAdress = 132;
			u16Factor = 14;
		break;
		case 0x202205:	
			u16SendAdress = 158;
			u16Factor = 15;
		break;
		case 0x202207:	
			u16SendAdress = 160;
			u16Factor = 16;
		break;
		case 0x202208:	
			u16SendAdress = 161;
			u16Factor = 17;
		break;
		
		case 0x202209:	
			u16SendAdress = 159;
			u16Factor = 18;
		break;
		
		case 0x202801:
		case 0x202802:
			u16SendAdress = 131;
			u16Factor = 19;
		break;
		case 0x202805:
		case 0x201C4A:
			u16SendAdress = 133; 
			u16Factor = 20;
		break;
		case 0x202806:
		case 0x201C56:
			u16SendAdress = 134;
			u16Factor = 21;
		break;
		case 0x202808:
			u16SendAdress = 135;
			u16Factor = 22;
		break;
		case 0x20280B:
			u16SendAdress = 136;
			u16Factor = 23;
		break;
		
		case 0x202904:
			u16SendAdress = 5;
			u16Factor = 24;
		break;
		
		case 0x204001:
			u16SendAdress = 19;
			u16Factor = 25;
		break;
		case 0x204002:
			u16SendAdress = 20;
			u16Factor = 26;
		break;
		case 0x204003:
			u16SendAdress = 21;
			u16Factor = 27;
		break;
		case 0x204004:
			u16SendAdress = 22;
			u16Factor = 28;
		break;
		case 0x204005:
			u16SendAdress = 23;
			u16Factor = 29;
		break;
		case 0x204006:
			u16SendAdress = 24;
			u16Factor = 30;
		break;
		case 0x204007:
			u16SendAdress = 25;
			u16Factor = 31;
		break;
		case 0x204008:
			u16SendAdress = 26;
			u16Factor = 32;
		break;
		case 0x204009:
			u16SendAdress = 26;
			u16Factor = 33;
		break;
		case 0x20400A:
			u16SendAdress = 28;
			u16Factor = 34;
		break;
		case 0x20400B:
			u16SendAdress = 29;
			u16Factor = 35;
		break;
		case 0x20400C:
			u16SendAdress = 30;
			u16Factor = 36;
		break;
		case 0x20400D:
			u16SendAdress = 31;
			u16Factor = 37;
		break;
		case 0x20400E:
			u16SendAdress = 32;
			u16Factor = 38;
		break;
		case 0x20400F:
			u16SendAdress = 33;
			u16Factor = 39;
		break;
		case 0x204010:
			u16SendAdress = 34;
			u16Factor = 40;
		break;
		
		case 0x204101:
			u16SendAdress = 37;
			u16Factor = 41;
		break;
		case 0x204102:
			u16SendAdress = 38;
			u16Factor = 42;
		break;
		case 0x204103:
			u16SendAdress = 39;
			u16Factor = 43;
		break;
		case 0x204104:
			u16SendAdress = 40;
			u16Factor = 44;
		break;
		case 0x204105:
			u16SendAdress = 41;
			u16Factor = 45;
		break;
		case 0x204106:
			u16SendAdress = 3;
			u16Factor = 1;
		break;
		
		case 0x204301:
			u16SendAdress = 147;
			u16Factor = 46;
			break;
		case 0x204302:
			u16SendAdress = 148;
			u16Factor = 47;
			break;
		case 0x204303:
			u16SendAdress = 149;
			u16Factor = 48;
			break;
		case 0x204304:
			u16SendAdress = 150;
			u16Factor = 49;
			break;
		
		case 0x200A53:
			u16SendAdress = 63;
			u16Factor = 50;
			break;
		case 0x200A54:
			u16SendAdress = 64;
			u16Factor = 51;
			break;
		case 0x200A56:
			u16SendAdress = 66;
			u16Factor = 52;
			break;
		case 0x200A57:
			u16SendAdress = 67;
			u16Factor = 53;
			break;
		case 0x200A5B:
			u16SendAdress = 71;
			u16Factor = 54;
			break;
		
		case 0x200B01:
			u16SendAdress = 65;
			u16Factor = 55;
			break;
		case 0x200B02:
			u16SendAdress = 70;
			u16Factor = 1;
			break;
		case 0x200B04:
			u16SendAdress = 72;
			u16Factor = 57;
			break;
		case 0x200B05:
			u16SendAdress = 73;
			u16Factor = 56;
			break;
		case 0x200B09:
			u16SendAdress = 74;
			u16Factor = 1;
			break;
		case 0x200B07:
			u16SendAdress = 76;
			u16Factor = 58;
			break;
		case 0x200B0C:
			u16SendAdress = 81;
			u16Factor = 59;
			break;
		case 0x200B0D:
			u16SendAdress = 82;
			u16Factor = 60;
			break;
		case 0x200B0B:
			u16SendAdress = 86;
			u16Factor = 61;
			break;
		case 0x200B0A:
			u16SendAdress = 106;
			u16Factor = 62;
			break;
		
		case 0x200C02:
			u16SendAdress = 6;
			u16Factor = 63;
			break;
		case 0x200C0A:
			u16SendAdress = 7;
			u16Factor = 64;
			break;
		case 0x200C09:
			u16SendAdress = 8;
			u16Factor = 65;
			break;
		case 0x200C0B:
			u16SendAdress = 10;
			u16Factor = 66;
			break;
		case 0x200C12:
			u16SendAdress = 11;
			u16Factor = 67;
			break;
		case 0x200C0F:
			u16SendAdress = 12;
			u16Factor = 68;
			break;
		case 0x200C13:
			u16SendAdress = 13;
			u16Factor = 69;
			break;
		
		case 0x200D04:
		case 0x200D05:
			u16SendAdress = 3;
			u16Factor = 70;
			break;
		case 0x200D03:
			u16SendAdress = 8;
			u16Factor = 1;
			break;
		
		case 0x200F03:
			u16SendAdress = 14;
			u16Factor = 71;
			break;
		case 0x200F04:
			u16SendAdress = 15;
			u16Factor = 72;
			break;
		case 0x200F06:
			u16SendAdress = 17;
			u16Factor = 73;
			break;
		case 0x200F07:
			u16SendAdress = 18;
			u16Factor = 74;
			break;
		case 0x200F05:
			u16SendAdress = 35;
			u16Factor = 75;
			break;
		
		case 0x201E13:
			u16SendAdress = 94;
			u16Factor = 79;
			break;
		case 0x201E14:
			u16SendAdress = 95;
			u16Factor = 80;
			break;
		case 0x201E17:
			u16SendAdress = 98;
			u16Factor = 83;
			break;
		case 0x201E18:
			u16SendAdress = 99;
			u16Factor = 84;
			break;
		case 0x201E19:
			u16SendAdress = 100;
			u16Factor = 85;
			break;
		
		case 0x202703:
			u16SendAdress = PARA_AccAndDecTurn;
			u16Factor = 1;
			break;
		
		case 0x202B02:
		case 0x202B03:
		case 0x202B04:
			u16SendAdress = 3;
			u16Factor = 86;
			break;
		case 0x202B05:
		case 0x202B06:
		case 0x202B07:
		case 0x202B08:
		case 0x202B09:
		case 0x202B0A:
			u16SendAdress = 4;
			u16Factor = 87;
			break;
		
		case 0x202E03:
		case 0x202E04:
		case 0x202E05:
		case 0x202E06:
		case 0x202E07:
		case 0x202E08:
		case 0x202E09:
			u16SendAdress = 1;
			u16Factor = 88;
			break;
		case 0x202E0A:
		case 0x202E0B:
		case 0x202E0C:
		case 0x202E0D:
		case 0x202E0E:
		case 0x202E0F:
		case 0x202E10:
			u16SendAdress = 2;
			u16Factor = 89;
			break;
		
		case 0x201901:
			u16SendAdress = 132;
			u16Factor = 1;
			break;
		case 0x201902:
			u16SendAdress = 131;
			u16Factor = 1;
			break;
		case 0x201903:
			u16SendAdress = 4;
			u16Factor = 1;
			break;
		
		case 0x400D01:
			u16SendAdress = 0x40DC;
			u16Factor = 91;
			break;
		case 0x400D02:
			u16SendAdress = 0x40DD;
			u16Factor = 92;
			break;
		case 0x400D03:
			u16SendAdress = 0x40DF;
			u16Factor = 93;
			break;
		case 0x400D50:
			u16SendAdress = 0x40D6;
			u16Factor = 1;
			break;
		case 0x400D51:
			u16SendAdress = 0x40D8;
			u16Factor = 1;
			break;
		
		case 0x400C01:
			u16SendAdress = 0x40DE;
			u16Factor = 90;
			break;
		case 0x400C03:
			u16SendAdress = 0x40E0;
			u16Factor = 94;
			break;
		case 0x400C04:
			u16SendAdress = 0x40E1;
			u16Factor = 1;
			break;
		case 0x400C05:
			u16SendAdress = 0x40F3;
			u16Factor = 1;
			break;
		
		case 0x400111:
			u16SendAdress = 0x40F1;
			u16Factor = 1;
			break;
		case 0x400112:
			u16SendAdress = 0x40F2;
			u16Factor = 1;
			break;
		case 0x400113:
			u16SendAdress = 0x40F0;
			u16Factor = 1;
			break;
		
		case 0x400314:              // √
			u16SendAdress = 0x40E6;
			u16Factor = 1;
			break;
		case 0x40030E:              // √
			u16SendAdress = 0x40E7;
			u16Factor = 1;
			break;
		case 0x40030F:              // √
			u16SendAdress = 0x40E8;
			u16Factor = 1;
			break;
		case 0x400310:              // √
			u16SendAdress = 0x40E9;
			u16Factor = 1;
			break;
		case 0x400311:              // √
			u16SendAdress = 0x40F4;
			u16Factor = 1;
			break;
		case 0x400312:              // √
			u16SendAdress = 0x40F5;
			u16Factor = 1;
			break;
		case 0x400313:              // √
			u16SendAdress = 0x40F6;
			u16Factor = 1;
			break;
		
		case 0x202605:
			u16SendAdress = 65;
			u16Factor = 1;	
			break;
		
		case 0x201C01:
		case 0x201C02:
			u16SendAdress = 131;
			u16Factor = 1;	
			break;
		
		default:
			u16SendAdress = 0;
			u16Factor = 0;
		break;
	}
	*ad = u16SendAdress;
	*fa = u16Factor;
}


void vIndexToAdress2ECU(uint32_t u32RecieveAdress,uint16_t *ad,uint16_t *fa)
{
	uint16_t u16SendAdress;
	uint16_t u16Factor;
	switch(u32RecieveAdress)
	{
		case 0x201C03:
			u16SendAdress = 100;
			u16Factor = 1;
			break;
		case 0x201C04:
			u16SendAdress = 101;
			u16Factor = 1;
			break;
		case 0x201C05:
			u16SendAdress = 104;
			u16Factor = 1;
			break;
		case 0x201C06:
			u16SendAdress = 103;
			u16Factor = 1;
			break;
		case 0x201C07:
			u16SendAdress = 98;
			u16Factor = 97;
			break;
		case 0x201C08:
			u16SendAdress = 99;
			u16Factor = 98;
			break;
		case 0x201C09:
			u16SendAdress = 105;
			u16Factor = 105;
			break;
		case 0x201C10:
			u16SendAdress = 96;
			u16Factor = 1;
			break;
		case 0x201C11:
			u16SendAdress = 97;
			u16Factor = 1;
			break;
		case 0x201C12:
			u16SendAdress = 102;
			u16Factor = 1;
			break;
		
		case 0x201C0A:
		case 0x201C0B:
		case 0x201C0C:
		case 0x201C0D:
		case 0x201C0E:
		case 0x201C0F:
		case 0x201C13:
		case 0x201C14:
		case 0x201C15:
		case 0x201C16:
			u16SendAdress = PARA_DriverFlag;
			u16Factor = 1;
			break;
		
		case 0x201C4C:
		case 0x201C4D:
			u16SendAdress = PARA_Driver3Vol;
			u16Factor = 1;
			break;
		case 0x201C4E:
		case 0x201C4F:
			u16SendAdress = PARA_Driver4Vol;
			u16Factor = 1;
			break;
		case 0x201C50:
		case 0x201C58:
			u16SendAdress = PARA_Driver5Vol;
			u16Factor = 1;
			break;
		case 0x201C51:
		case 0x201C59:
			u16SendAdress = PARA_Driver6Vol;
			u16Factor = 1;
			break;
		case 0x201C52:
		case 0x201C5A:
			u16SendAdress = PARA_Driver7Vol;
			u16Factor = 1;
			break;
		case 0x201C53:
		case 0x201C5B:
			u16SendAdress = PARA_Driver8Vol;
			u16Factor = 1;
			break;
		case 0x201C54:
		case 0x201C5C:
			u16SendAdress = PARA_Driver9Vol;
			u16Factor = 1;
			break;
		case 0x201C55:
		case 0x201C5D:
			u16SendAdress = PARA_Driver10Vol;
			u16Factor = 1;
			break;
		
		case 0x400101:
		case 0x400102:
		case 0x400103:
		case 0x400104:
		case 0x400105:
		case 0x400106:
		case 0x400107:
		case 0x400108:
		case 0x400109:
		case 0x40010A:
		case 0x40010B:
		case 0x40010C:
		case 0x40010D:
		case 0x40010E:
		case 0x40010F:
		case 0x400110:
			u16SendAdress = PARA_Swi;
			u16Factor = 1;
			break;
			
		case 0x400301:
		case 0x400302:
		case 0x400303:
		case 0x400308:
		case 0x400309:
		case 0x40030A:
		case 0x40030B:
		case 0x40030C:
		case 0x40030D:
		case 0x400405:
			u16SendAdress = PARA_ExtSignal;
			u16Factor = 1;
			break;
		
		case 0x400401:
		case 0x400402:
			u16SendAdress = PARA_ForwardValveCurrent;
			u16Factor = 99;
			break;
		case 0x400403:
		case 0x400404:
			u16SendAdress = PARA_BackValveCurrent;
			u16Factor = 99;
			break;
		
		case 0x400201:
			u16SendAdress = PARA_Ai1;
			u16Factor = 1;
			break;
		case 0x400202:
			u16SendAdress = PARA_Ai2;
			u16Factor = 1;
			break;
		case 0x400203:
			u16SendAdress = PARA_Ai3;
			u16Factor = 1;
			break;
		case 0x400204:
			u16SendAdress = PARA_Ai4;
			u16Factor = 1;
			break;
		
		case 0x200401:
		case 0x200402:
			u16SendAdress = PARA_MotorHighSpeedDeceRate;
			u16Factor = 1;
			break;
		case 0x200403:
		case 0x200404:
			u16SendAdress = PARA_Gear1Spd;
			u16Factor = 1;
			break;
		
		case 0x20080C:
			u16SendAdress = PARA_Gear2Spd;
			u16Factor = 1;
			break;
		
		case 0x20090C:
		case 0x200501:
		case 0x200502:
			u16SendAdress = PARA_Gear3Spd;
			u16Factor = 1;
			break;
		
		case 0x200A0C:
			u16SendAdress = PARA_Gear4Spd;
			u16Factor = 1;
			break;
		
		case 0x200601:         //低电量
		case 0x200602:
			u16SendAdress = PARA_Gear4Spd;
			u16Factor = 1;
			break;
		
		case 0x400920:
		case 0x400921:
		case 0x400922:
		case 0x400923:
		case 0x400924:
		case 0x400925:
		case 0x400926:
		case 0x400927:
		case 0x400928:
		case 0x400929:
		case 0x40092A:
		case 0x40092B:
		case 0x40092C:
		case 0x40092E:
		case 0x400930:
		case 0x400931:
		case 0x400932:
		case 0x400933:
		case 0x400934:
		case 0x400935:
		case 0x400936:
			u16SendAdress = PARA_DoSwi;
			u16Factor = 1;			
			break;
		
		case 0x40092D:
			u16SendAdress = PARA_PropCurrent2;
			u16Factor = 1;	
			break;
		
		case 0x400B01:
			u16SendAdress = PARA_Ksi;
			u16Factor = 96;	
			break;
		case 0x400B02:
			u16SendAdress = PARA_Ksi;
			u16Factor = 1;	
			break;
		case 0x400B03:
			u16SendAdress = PARA_BmsSoc;
			u16Factor = 1;	
			break;
		
		case 0x400E01:
			u16SendAdress = PARA_MotorSpd;
			u16Factor = 95;	
			break;
		case 0x400E02:
			u16SendAdress = PARA_SteerAngle;
			u16Factor = 1;	
			break;
		case 0x400E03:
			u16SendAdress = PARA_ErrCode;
			u16Factor = 1;	
			break;
		case 0x400E0A:
			u16SendAdress = PARA_EcuLockState;          //上电计时
			u16Factor = 1;	
			break;
		case 0x400E0B:
			u16SendAdress = PARA_TmpLockState;       //工作计时
			u16Factor = 1;	
			break;
		case 0x400E0C:
			u16SendAdress = PARA_SelfHeartQuery;
			u16Factor = 1;	
			break;
		case 0x400E0D:                         //计时状态
			u16SendAdress = PARA_PlatfromHeartQuery;
			u16Factor = 1;	
			break;
		
		case 0x202601:
			u16SendAdress = PARA_AngleWithStartSpdPer;
			u16Factor = 1;	
			break;
		case 0x202602:
			u16SendAdress = PARA_TurnWithDecStartAngle;
			u16Factor = 1;	
			break;
		case 0x202603:
			u16SendAdress = PARA_AngleWithEndSpdPer;
			u16Factor = 1;	
			break;
		case 0x202604:
			u16SendAdress = PARA_TurnWithDecEndAngle;
			u16Factor = 1;	
			break;
		
		case 0x201E10:
			u16SendAdress = PARA_ThrottleType;
			u16Factor = 76;
			break;
		case 0x201E11:
			u16SendAdress = PARA_ThrottleFDeadZoneMinVal;
			u16Factor = 77;
			break;
		case 0x201E12:
			u16SendAdress = PARA_ThrottleFDeadZoneMaxVal;
			u16Factor = 78;
			break;
		case 0x201E15:
			u16SendAdress = PARA_ThrottleBDeadZoneMinVal;
			u16Factor = 81;
			break;
		case 0x201E16:
			u16SendAdress = PARA_ThrottleBDeadZoneMaxVal;
			u16Factor = 82;
			break;
		case 0x201E1A:
			u16SendAdress = PARA_BrakeFDeadZoneMinVal;
			u16Factor = 82;
			break;
		case 0x201E1B:
			u16SendAdress = PARA_BrakeFDeadZoneMaxVal;
			u16Factor = 82;
			break;
		case 0x201E1C:
			u16SendAdress = PARA_BrakeFMidVal;
			u16Factor = 82;
			break;
		case 0x201E1D:
			u16SendAdress = PARA_BrakeBDeadZoneMinVal;
			u16Factor = 82;
			break;
		case 0x201E1E:
			u16SendAdress = PARA_BrakeBDeadZoneMaxVal;
			u16Factor = 82;
			break;
		case 0x201E1F:
			u16SendAdress = PARA_BrakeBMidVal;
			u16Factor = 82;
			break;
		case 0x201E20:
			u16SendAdress = PARA_Analog3DeadZoneMinVal;
			u16Factor = 82;
			break;
		case 0x201E21:
			u16SendAdress = PARA_Analog3DeadZoneMaxVal;
			u16Factor = 82;
			break;
		case 0x201E22:
			u16SendAdress = PARA_Analog3MidVal;
			u16Factor = 82;
			break;
		case 0x201E23:
			u16SendAdress = PARA_Analog3DeadZoneMinVal;
			u16Factor = 82;
			break;
		case 0x201E24:
			u16SendAdress = PARA_Analog3DeadZoneMaxVal;
			u16Factor = 82;
			break;
		case 0x201E25:
			u16SendAdress = PARA_Analog3MidVal;
			u16Factor = 82;
			break;
		case 0x202B09:
			u16SendAdress = PARA_AngleValue7;
			u16Factor = 1;
			break;
		case 0x202B19:
			u16SendAdress = PARA_VehicleType;
			u16Factor = 1;
			break;
		case 0x400D04:
			u16SendAdress = PARA_PumpCmd;
			u16Factor = 100;
			break;
		case 0x202B3F:
			u16SendAdress = PARA_AngleValue4;
			u16Factor = 1;
			break;
		case 0x202B40:
			u16SendAdress = PARA_AngleValue3;
			u16Factor = 100;
			break;
		
		default:
			u16SendAdress = 0;
			u16Factor = 0;
		break;
	}
	
	*ad = u16SendAdress;
	*fa = u16Factor;
}




#endif
