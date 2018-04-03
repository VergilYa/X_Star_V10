/***********************************************************************
 * Module:  CRF.h
 * Author:  Vergil
 * Modified: 2018/03/22 13:56:15
 * Purpose: Declaration of the class CRF
 ***********************************************************************/

#if !defined(__CRFx_h)
#define __CRFx_h

#include "global.h"
#include "uartCom.h"

typedef struct _RFSysParams
{


}RFSysParams;

class CRFx
{
public:
	CRFx(char index);
	~CRFx();
	

public:
	int GetVersion();					//1.	软件版本信息（只获取，不设置）
	int GetHorL();						//2.	HorL高低站（只获取，不设置）
	int SetTXFr(int txfre);				//3.	TXFr 当前发射频率 KHz
	int GetTXFr();
	int SetRXFr(int rxfre);				//4.	RXFr 当前接收频率 KHz
	int GetRXFr();
	int SetTPST(int txpw);				//5.	TPST当前设置发信功率 0.1dbm
	int GetTPST();
	int GetTPWR();						//6.	TPWR当前实际发信功率
	int GetRECV();						//7.	RECV 当前收信电平
	int GetRSSI();						//8.	RSSI 收信指示ADC值
	int GetDETX();						//9.	DETX 发信指示ADC值
	int GetTXLO();						//10.	TXLO TXIF_LO锁定信息
	int GetRXLO();						//11.	RXLO RXIF_LO锁定信息
	int GetRFLO();						//12.	RFLO RF_LO锁定信息
	int GetVGA();						//13.	VGA电压
	int SetPAEN(int paEn);				//14.	PAEN功放使能
	int GetPAEN();
	int SetPWRUP();						//15.	功率校准,增大一个步进功率，返回寄存器值
	int SetPWRDOWN();					//16.	功率校准,减小一个步进功率，返回寄存器值
	int SetTPCO(int tpco);				//17.	TPCO发射功率补偿
	int GetTPCO();
	int SetGAIN(int gain);				//18.	GAIN接收电平补偿
	int GetGAIN();
	int GetTFMi();						//19.	TFMi最小发射频率
	int GetTFMa();						//20.	TFMa最大发射频率
	int GetRFMi();						//21.	RFMi最小接收频率
	int GetRFMa();						//22.	RFMa最大接收频率
	int GetFRIV();						//23.	FRIV收发信频率间隔
	int SetTPMi(int tpmi);				//24.	TPMi最小发信功率
	int GetTPMi();
	int SetTPMa(int tpma);				//25.	TPMa最大发信功率
	int GetTPMa();
	int SetSNum(int snum);				//26.	SNum值设置
	int GetSNum();
	int SetTCTF();						//27.	TCTF 发信校准表文件
	int GetTCTF();
	int SetRCTF();						//28.	RCTF 收信校准表文件
	int GetRCTF();
protected:
private:
	int CreatSetCmd(int cmdIndex,char *sparam,char *rparam);
	int CreatSetCmd(int cmdIndex,int sparam,char *rparam);
	int CreatGetCmd(int cmdIndex,char *gparam,char *rparam);
	int ParseRtnFun();



	HOST_COM_LAYER_ERROR_ENUM uartComOpen();
	HOST_COM_LAYER_ERROR_ENUM uartComClose();
	HOST_COM_LAYER_ERROR_ENUM uartComSend(char* pBuff, unsigned int len);
	HOST_COM_LAYER_ERROR_ENUM uartComRecv( UINT8* pBuff, UINT16 len, UINT16* byteLenRecv);

	PUART pvgUart;
};
#endif

