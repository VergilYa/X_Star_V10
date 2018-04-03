#ifndef _UART_COM_H_
#define _UART_COM_H_


#include "PVG_defs.h"
#endif
#include "hostComLayerErr.h"
#include "pserial.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PVG_PROTOCOL_MSG_MAX_MSG_LEN            256

typedef enum
{
    RX_STATE_IDLE_E,
	RX_STATE_FLAG_FOUND_E,
	RX_STATE_FLAG_FOUND_T_E,
	RX_STATE_FLAG_FOUND_N_E,
	RX_STATE_HEAD_FOUND_E,
	RX_STATE_CMDID_FOUND_E,
	RX_STATE_CMDNAME_FOUND_E,
    RX_STATE_IN_MSG_E,
    RX_STATE_ESC_CODE_DETECTED_E,
    RX_STATE_IGNORE_E,
	RX_STATE_FINISH
} RX_STATE_ENUM;

typedef enum{
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
}CMD_ID_ENUM;

typedef enum
{
    SET_CMD_E = 0,
    GET_CMD_E
} SorG_ENUM;


typedef struct{
    UINT16                          stuffedBuffLen;
	BYTE							pStuffedBuff[PVG_PROTOCOL_MSG_MAX_MSG_LEN];
	unsigned int					portNumber;
	PUART							uartHandle;
	RX_STATE_ENUM					msgCurrentState;
	UINT32							rxMsgLen;
	UINT8							msgId;
	UINT8							msgSorG;	//0:Set 1:Get
	BYTE							rStuffedBuff[PVG_PROTOCOL_MSG_MAX_MSG_LEN];
} UART_COM_STRUCT;

void CreatSetCmd(int cmdId,int sparam,BYTE *rparam);
void CreatGetCmd(int cmdId,char *gparam,BYTE *rparam);

HOST_COM_LAYER_ERROR_ENUM uartComInit(UART_COM_STRUCT * pComInfo);
HOST_COM_LAYER_ERROR_ENUM uartComOpen(UART_COM_STRUCT * pComInfo);
HOST_COM_LAYER_ERROR_ENUM uartComClose(UART_COM_STRUCT * pComInfo);
HOST_COM_LAYER_ERROR_ENUM uartComSend(UART_COM_STRUCT * pComInfo,  UINT8* pBuff, UINT16 len, UINT16* byteLenSend);
HOST_COM_LAYER_ERROR_ENUM uartComRecv(UART_COM_STRUCT * pComInfo, UINT8* pBuff, UINT16 len, UINT16* byteLenRecv);


#ifdef __cplusplus
}
#endif

#endif /* _UART_COM_H_ */

