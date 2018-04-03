/***********************************************************************
 * Module:  CRF.cpp
 * Author:  lst
 * Modified: 2018/03/22 13:56:15
 * Purpose: Implementation of the class CRF
 ***********************************************************************/

#include "CRFx.h"
#include "CConfig.h"
#include "hostComLayerErr.h"

#define MAX_CMD_LEN 64
#define PVG_SERIAL_PORT 1
#define TAG "[CRFx]"

const char * head_cmd[] = {"SET","GET","RTN"};
const char * ctrl_cmd[] = {
	"VERS",			//1.	软件版本信息（只获取，不设置）
	"HorL",				//
	"TXFr"
	"RXFr",
	"RXFr",
	"TPWR",
	"RECV",
	"RSSI",
	"DETX",
	"TXLO",
	"RXLO",
	"RFLO",
	"VGA",
	"PAEN",
	"PWR+",
	"PWR-",
	"GAIN",
	"TFMi",
	"TFMa",
	"RFMi",
	"RFMi",
	"FRIV",
	"TPMi",
	"TPMa",
	"SNum",
	"TCTF",
	"RCTF"
};
////////////////////////////////////////////////////////////////////////
// Name:       CRFx::CRFx()
// Purpose:    Implementation of CRFx::CRFx()
// Return:     
////////////////////////////////////////////////////////////////////////

CRFx::CRFx(char index)
{
	memset(pvgUart,0,siezof(pvgUart));

	int ret;
	//init rf serial
	ret = uartComOpen();
   	if(ret!=HOST_COM_LAYER_SUCCESS_E)
   	{
   		printf("%s rf serial port open error!",TAG);
   		return;
   	}

}

////////////////////////////////////////////////////////////////////////
// Name:       CRFx::~CRFx()
// Purpose:    Implementation of CRFx::~CRFx()
// Return:     
////////////////////////////////////////////////////////////////////////

CRFx::~CRFx()
{
 	uartComClose();

}
////////////////////////////////////////////////////////////////////////
// Name:       CRFx::
// Purpose:    
// Return:     
////////////////////////////////////////////////////////////////////////
HOST_COM_LAYER_ERROR_ENUM CRFx::uartComOpen()
{
   	return pUuartOpen(&pvgUart, PVG_SERIAL_PORT);
}

HOST_COM_LAYER_ERROR_ENUM CRFx::uartComClose()
{
	return pUartClose(&pvgUart);
}

HOST_COM_LAYER_ERROR_ENUM CRFx::uartComSend(char* pBuff, unsigned int len)
{
	return pUartSend(&pvgUart, pBuff, len);
}

HOST_COM_LAYER_ERROR_ENUM CRFx::uartComRecv(char* pBuff, unsigned int *len)
{
	return pUartRecv(&pvgUart, pBuff, len);
}
////////////////////////////////////////////////////////////////////////
// Name:       CRFx::CreatSetCmd
// Purpose:    
// Return:     
////////////////////////////////////////////////////////////////////////
int CRFx::CreatSetCmd(int cmdIndex,char *sparam,char *rparam)
{
	//[head_cmd]+[空格]+[numcode]+[空格]+[ctrl_cmd]+[空格]+[parameter]+[*]
	sprintf(rparam,"%s %d %s %s*",head_cmd[0],cmdIndex,ctrl_cmd[cmdIndex],sparam);
	return 0;
}
int CRFx::CreatSetCmd(int cmdIndex,int sparam,char *rparam)
{
	//[head_cmd]+[空格]+[numcode]+[空格]+[ctrl_cmd]+[空格]+[parameter]+[*]
	sprintf(rparam,"%s %d %s %d*",head_cmd[0],cmdIndex,ctrl_cmd[cmdIndex],sparam);
	return 0;
}
////////////////////////////////////////////////////////////////////////
// Name:       CRFx::CreatGetCmd
// Purpose:    
// Return:     
////////////////////////////////////////////////////////////////////////
int CRFx::CreatGetCmd(int cmdIndex,char *gparam,char *rparam)
{
	//[head_cmd]+[空格]+[numcode]+[空格]+[ctrl_cmd]+[空格]+[parameter]+[*]
	sprintf(rparam,"%s %d %s*",head_cmd[1],cmdIndex,ctrl_cmd[cmdIndex]);
	return 0;
}

////////////////////////////////////////////////////////////////////////
// Name:       CRFx::
// Purpose:    1.	软件版本信息（只获取，不设置）
// Return:     
////////////////////////////////////////////////////////////////////////
int CRFx::GetVersion()
{
	char buf[MAX_CMD_LEN];
	CreatGetCmd(0,NULL,buf);
	uartComSend(buf,strlen(buf));

	return 0;
}
////////////////////////////////////////////////////////////////////////
// Name:       CRFx::
// Purpose:    2.	HorL高低站（只获取，不设置）
// Return:     
////////////////////////////////////////////////////////////////////////
int CRFx::GetHorL()
{
	char buf[MAX_CMD_LEN];
	CreatGetCmd(1,NULL,buf);
	uartComSend(buf,strlen(buf));

	return 0;
}
////////////////////////////////////////////////////////////////////////
// Name:       CRFx::
// Purpose:    3.	TXFr 当前发射频率 KHz
// Return:     
////////////////////////////////////////////////////////////////////////
int CRFx::SetTXFr(int txfre)
{
	char buf[MAX_CMD_LEN];
	CreatSetCmd(2,txfre,buf);
	uartComSend(buf,strlen(buf));

	return 0;
}
int CRFx::GetTXFr()
{
	char buf[MAX_CMD_LEN];
	CreatGetCmd(2,NULL,buf);
	uartComSend(buf,strlen(buf));

	return 0;
}
////////////////////////////////////////////////////////////////////////
// Name:       CRFx::
// Purpose:    4.	RXFr 当前接收频率 KHz
// Return:     
////////////////////////////////////////////////////////////////////////
int CRFx::SetRXFr(int rxfre)
{
	char buf[MAX_CMD_LEN];
	CreatSetCmd(3,rxfre,buf);
	uartComSend(buf,strlen(buf));

	return 0;
}
int CRFx::GetRXFr()
{
	char buf[MAX_CMD_LEN];
	CreatGetCmd(3,NULL,buf);
	uartComSend(buf,strlen(buf));

	return 0;
}
////////////////////////////////////////////////////////////////////////
// Name:       CRFx::
// Purpose:    5.	TPST当前设置发信功率 0.1dbm
// Return:     
////////////////////////////////////////////////////////////////////////
int CRFx::SetTPST(int txpw)
{
	char buf[MAX_CMD_LEN];
	char pbuf[MAX_CMD_LEN];
	sprintf(pbuf,"%.1f",txpw/10.0);
	CreatSetCmd(4,pbuf,buf);
	uartComSend(buf,strlen(buf));

	return 0;
}
int CRFx::GetTPST()
{
	char buf[MAX_CMD_LEN];
	CreatGetCmd(4,NULL,buf);
	uartComSend(buf,strlen(buf));

	return 0;
}
////////////////////////////////////////////////////////////////////////
// Name:       CRFx::
// Purpose:    6.	TPWR当前实际发信功率
// Return:     
////////////////////////////////////////////////////////////////////////
int CRFx::GetTPWR()
{
	char buf[MAX_CMD_LEN];
	CreatGetCmd(5,NULL,buf);
	uartComSend(buf,strlen(buf));

	return 0;
}
////////////////////////////////////////////////////////////////////////
// Name:       CRFx::
// Purpose:    7.	RECV 当前收信电平
// Return:     
////////////////////////////////////////////////////////////////////////
int CRFx::GetRECV()
{
	char buf[MAX_CMD_LEN];
	CreatGetCmd(6,NULL,buf);
	uartComSend(buf,strlen(buf));

	return 0;
}
////////////////////////////////////////////////////////////////////////
// Name:       CRFx::
// Purpose:    8.	RSSI 收信指示ADC值
// Return:     
////////////////////////////////////////////////////////////////////////
int CRFx::GetRSSI()
{
	char buf[MAX_CMD_LEN];
	CreatGetCmd(7,NULL,buf);
	uartComSend(buf,strlen(buf));

	return 0;	
}
////////////////////////////////////////////////////////////////////////
// Name:       CRFx::
// Purpose:    9.	DETX 发信指示ADC值
// Return:     
////////////////////////////////////////////////////////////////////////
int CRFx::GetDETX()
{
	char buf[MAX_CMD_LEN];
	CreatGetCmd(8,NULL,buf);
	uartComSend(buf,strlen(buf));

	return 0;	
}
////////////////////////////////////////////////////////////////////////
// Name:       CRFx::
// Purpose:    10.	TXLO TXIF_LO锁定信息
// Return:     
////////////////////////////////////////////////////////////////////////
int CRFx::GetTXLO()
{
	char buf[MAX_CMD_LEN];
	CreatGetCmd(9,NULL,buf);
	uartComSend(buf,strlen(buf));

	return 0;	
}
////////////////////////////////////////////////////////////////////////
// Name:       CRFx::
// Purpose:    11.	RXLO RXIF_LO锁定信息
// Return:     
////////////////////////////////////////////////////////////////////////
int CRFx::GetRXLO()
{
	char buf[MAX_CMD_LEN];
	CreatGetCmd(10,NULL,buf);
	uartComSend(buf,strlen(buf));

	return 0;	
}
////////////////////////////////////////////////////////////////////////
// Name:       CRFx::
// Purpose:    12.	RFLO RF_LO锁定信息
// Return:     
////////////////////////////////////////////////////////////////////////
int CRFx::GetRFLO()
{
	char buf[MAX_CMD_LEN];
	CreatGetCmd(11,NULL,buf);
	uartComSend(buf,strlen(buf));

	return 0;	
}
////////////////////////////////////////////////////////////////////////
// Name:       CRFx::
// Purpose:    13.	VGA电压
// Return:     
////////////////////////////////////////////////////////////////////////
int CRFx::GetVGA()
{
	char buf[MAX_CMD_LEN];
	CreatGetCmd(12,NULL,buf);
	uartComSend(buf,strlen(buf));

	return 0;	
}
////////////////////////////////////////////////////////////////////////
// Name:       CRFx::
// Purpose:    14.	PAEN功放使能 0:disable 1:enable
// Return:     
////////////////////////////////////////////////////////////////////////
int CRFx::SetPAEN(int paEn)
{
	char buf[MAX_CMD_LEN];
	CreatSetCmd(13,paEn,buf);
	uartComSend(buf,strlen(buf));

	return 0;
}

int CRFx::GetPAEN()
{
	char buf[MAX_CMD_LEN];
	CreatGetCmd(13,NULL,buf);
	uartComSend(buf,strlen(buf));

	return 0;	
}
////////////////////////////////////////////////////////////////////////
// Name:       CRFx::
// Purpose:    15.	功率校准,增大一个步进功率，返回寄存器值
// Return:     
////////////////////////////////////////////////////////////////////////
int CRFx::SetPWRUP()
{
	char buf[MAX_CMD_LEN];
	CreatGetCmd(14,NULL,buf);
	uartComSend(buf,strlen(buf));

	return 0;
}
////////////////////////////////////////////////////////////////////////
// Name:       CRFx::
// Purpose:    16.	功率校准,减小一个步进功率，返回寄存器值
// Return:     
////////////////////////////////////////////////////////////////////////
int CRFx::SetPWRDOWN()
{
	char buf[MAX_CMD_LEN];
	CreatGetCmd(15,NULL,buf);
	uartComSend(buf,strlen(buf));

	return 0;
}
////////////////////////////////////////////////////////////////////////
// Name:       CRFx::
// Purpose:    17.	TPCO发射功率补偿 0.1dbm
// Return:     
////////////////////////////////////////////////////////////////////////
int CRFx::SetTPCO(int tpco)
{
	char buf[MAX_CMD_LEN];
	CreatSetCmd(16,tpco,buf);
	uartComSend(buf,strlen(buf));

	return 0;
}
int CRFx::GetTPCO()
{
	char buf[MAX_CMD_LEN];
	CreatGetCmd(16,NULL,buf);
	uartComSend(buf,strlen(buf));

	return 0;	
}
////////////////////////////////////////////////////////////////////////
// Name:       CRFx::
// Purpose:    18.	GAIN接收电平补偿 dbm
// Return:     
////////////////////////////////////////////////////////////////////////
int CRFx::SetGAIN(int gain)
{
	char buf[MAX_CMD_LEN];
	CreatSetCmd(17,gain,buf);
	uartComSend(buf,strlen(buf));

	return 0;
}
int CRFx::GetGAIN()
{
	char buf[MAX_CMD_LEN];
	CreatGetCmd(17,NULL,buf);
	uartComSend(buf,strlen(buf));

	return 0;	
}
////////////////////////////////////////////////////////////////////////
// Name:       CRFx::
// Purpose:    19.	TFMi最小发射频率
// Return:     
////////////////////////////////////////////////////////////////////////
int CRFx::GetTFMi()
{
	char buf[MAX_CMD_LEN];
	CreatGetCmd(18,NULL,buf);
	uartComSend(buf,strlen(buf));

	return 0;	
}
////////////////////////////////////////////////////////////////////////
// Name:       CRFx::
// Purpose:    20.	TFMa最大发射频率
// Return:     
////////////////////////////////////////////////////////////////////////
int CRFx::GetTFMa()
{
	char buf[MAX_CMD_LEN];
	CreatGetCmd(19,NULL,buf);
	uartComSend(buf,strlen(buf));

	return 0;	
}
////////////////////////////////////////////////////////////////////////
// Name:       CRFx::
// Purpose:    21.	RFMi最小接收频率
// Return:     
////////////////////////////////////////////////////////////////////////
int CRFx::GetRFMi()
{
	char buf[MAX_CMD_LEN];
	CreatGetCmd(20,NULL,buf);
	uartComSend(buf,strlen(buf));

	return 0;	
}
////////////////////////////////////////////////////////////////////////
// Name:       CRFx::
// Purpose:    22.	RFMa最大接收频率
// Return:     
////////////////////////////////////////////////////////////////////////
int CRFx::GetRFMa()
{
	char buf[MAX_CMD_LEN];
	CreatGetCmd(21,NULL,buf);
	uartComSend(buf,strlen(buf));

	return 0;	
}
////////////////////////////////////////////////////////////////////////
// Name:       CRFx::
// Purpose:    23.	FRIV收发信频率间隔
// Return:     
////////////////////////////////////////////////////////////////////////
int CRFx::GetFRIV()
{
	char buf[MAX_CMD_LEN];
	CreatGetCmd(22,NULL,buf);
	uartComSend(buf,strlen(buf));

	return 0;	
}
////////////////////////////////////////////////////////////////////////
// Name:       CRFx::
// Purpose:    24.	TPMi最小发信功率 0.1dbm
// Return:     
////////////////////////////////////////////////////////////////////////
int CRFx::SetTPMi(int tpmi)
{
	char buf[MAX_CMD_LEN];
	char pbuf[MAX_CMD_LEN];
	sprintf(pbuf,"%.1f",tpmi/10.0);
	CreatSetCmd(23,pbuf,buf);
	uartComSend(buf,strlen(buf));

	return 0;
}
int CRFx::GetTPMi()
{
	char buf[MAX_CMD_LEN];
	CreatGetCmd(23,NULL,buf);
	uartComSend(buf,strlen(buf));

	return 0;	
}
////////////////////////////////////////////////////////////////////////
// Name:       CRFx::
// Purpose:    25.	TPMa最大发信功率
// Return:     
////////////////////////////////////////////////////////////////////////
int CRFx::SetTPMa(int tpma)
{
	char buf[MAX_CMD_LEN];
	char pbuf[MAX_CMD_LEN];
	sprintf(pbuf,"%.1f",tpma/10.0);
	CreatSetCmd(24,pbuf,buf);
	uartComSend(buf,strlen(buf));

	return 0;	
}
int CRFx::GetTPMa()
{
	char buf[MAX_CMD_LEN];
	CreatGetCmd(24,NULL,buf);
	uartComSend(buf,strlen(buf));

	return 0;	
}
////////////////////////////////////////////////////////////////////////
// Name:       CRFx::
// Purpose:    26.	SNum值设置
// Return:     
////////////////////////////////////////////////////////////////////////
int CRFx::SetSNum(int snum)
{
	char buf[MAX_CMD_LEN];
	CreatSetCmd(25,snum,buf);
	uartComSend(buf,strlen(buf));

	return 0;
}
int CRFx::GetSNum()
{
	char buf[MAX_CMD_LEN];
	CreatGetCmd(25,NULL,buf);
	uartComSend(buf,strlen(buf));	
}
////////////////////////////////////////////////////////////////////////
// Name:       CRFx::
// Purpose:    27.	TCTF 发信校准表文件
// Return:     
////////////////////////////////////////////////////////////////////////
int CRFx::SetTCTF()
{
	
}
int CRFx::GetTCTF()
{
	
}
////////////////////////////////////////////////////////////////////////
// Name:       CRFx::
// Purpose:    28.	RCTF 收信校准表文件
// Return:     
////////////////////////////////////////////////////////////////////////
int CRFx::SetRCTF()
{
	
}
int CRFx::GetRCTF()
{
	
}
