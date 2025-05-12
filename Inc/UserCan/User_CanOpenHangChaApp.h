#ifndef __CANOPENHANGCHA_H
#define __CANOPENHANGCHA_H

#include "stdint.h"
#include "Userdef.h"
#include "stdlib.h"
#include "KTypedef.h"
#include "gd32f30x_timer.h"


/***** Struct define *****/
typedef struct tHCDGN_DATA
{
	uint16_t SynFlag;			 //发送/接收同步标志位（同步）
	uint16_t Q_AFlag;      //异步

	uint8_t FuncCode;           //BYTE[0]
	uint8_t SubIndex;           //BYTE[3]
	uint16_t Index;             //BYTE[2:1]
	uint16_t DataL16;           //BYTE[5:4]
	uint16_t DataH16;           //BYTE[7:6]
}tHCDGN_DATA;

typedef struct BlueSdo
{
	uint16_t SdoFlg;
	uint16_t Address;
	uint16_t SdoCs;
	uint16_t LocalData;
	uint16_t SaveData;
	uint8_t WriteReturnFlg;
	
}SdoReqCmd;

// 建立映射关系
typedef struct {
    char original;
    char replacement;
} CharMapping;

/***** Struct define *****/
typedef struct _tHANGCHADGC_PDO
{
	tHCDGN_DATA DgnDataRxTx;
	
	//DATA from App to Mcu
	uint8_t GeneratePwdFlag;
	char RandPwdArr[4];
	uint32_t IndexAddress;                 
	uint16_t Address;
	uint16_t XiShu;
	uint8_t HandshakeFlag;
  uint8_t SelectLanguage;
volatile	uint8_t SubFunID;
  size_t ErrorTotalLength;
	uint8_t ErrNum;
	uint8_t DataStateBit1;
	uint8_t DataStateBit2;
	uint8_t ErrcodeType;
	uint8_t ReqAnsyFlag;
	uint8_t OldFlag;
	tBoolean SendErrFlag;
	uint16_t ReadOriginalVal1;
	tBoolean LastErrByte;
}_tHANGCHADGC_PDO;


extern _tHANGCHADGC_PDO   HangChaDGCPdo;


void vGetRandPwd(_tHANGCHADGC_PDO *pCanopenPDO, uint8_t length);
INT8U iRemoteUnlockPWD(_tHANGCHADGC_PDO  *pCanopenPDO,char *myParsepwd);
void vGenerateKeyPro(char *Original,char *CodeKey);
void vGetReplaceChar(char *oldpwd,char *newpwd);
void vIndexToAdress(uint32_t u32RecieveAdress,uint16_t *ad,uint16_t *fa);
void vIndexToAdress2ECU(uint32_t u32RecieveAdress,uint16_t *ad,uint16_t *fa);


#endif
