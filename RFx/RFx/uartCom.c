#include "uartCom.h"
#include <stdio.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif
   
const char * head_cmd[3] = {"SET","GET","RTN"};
const char * ctrl_cmd[28] = {
	"VERS",			//1.	软件版本信息（只获取，不设置）
	"HorL",				//
	"TXFr"
	"RXFr",
	"TPST",
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
	"TPCO",
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

//Structs


// Static memebers


// Static functions

void CreatSetCmd(int cmdId,int sparam,BYTE *rparam)
{
	//[head_cmd]+[空格]+[numcode]+[空格]+[ctrl_cmd]+[空格]+[parameter]+[*]
	sprintf(rparam,"%s %d %s %d*",head_cmd[0],cmdId,ctrl_cmd[cmdId],sparam);
}

void CreatGetCmd(int cmdId,char *gparam,BYTE *rparam)
{
	//[head_cmd]+[空格]+[numcode]+[空格]+[ctrl_cmd]+[空格]+[parameter]+[*]
	sprintf(rparam,"%s %d %s*",head_cmd[1],cmdId,ctrl_cmd[cmdId]);
}

void buffCommStuffPrepare(SorG_ENUM SorG, UINT8 cmdId, BYTE *param, UART_COM_STRUCT *pUart)
{
    pUart->msgSorG = SorG;
    pUart->msgId = cmdId;
    if(SorG == SET_CMD_E)
    	CreatSetCmd(cmdId,param,pUart->pStuffedBuff)
    else
    	CreatGetCmd(cmdId,param,pUart->pStuffedBuff)
    pUart->stuffedBuffLen = strlen(pUart->pStuffedBuff);

}

// Global functions
HOST_COM_LAYER_ERROR_ENUM uartComInit(UART_COM_STRUCT * pComInfo)
{
	if (pComInfo)
	{
		(pComInfo)->msgCurrentState = RX_STATE_IDLE_E;
		(pComInfo)->rxMsgLen = 0;
		pUuartInit(&((pComInfo)->uartHandle));
	}
	else
	{
		return HOST_COM_LAYER_ERROR_E;
	}
	return HOST_COM_LAYER_SUCCESS_E;
}

HOST_COM_LAYER_ERROR_ENUM uartComOpen(UART_COM_STRUCT * pComInfo)
{
	HOST_COM_LAYER_ERROR_ENUM retVal = HOST_COM_LAYER_SUCCESS_E;

	if (pComInfo)
	{
		retVal = (HOST_COM_LAYER_ERROR_ENUM)pUuartOpen(&((pComInfo)->uartHandle), (pComInfo)->portNumber);
	}
	else
	{
		return HOST_COM_LAYER_ERROR_E;
	}

	return retVal;
}


HOST_COM_LAYER_ERROR_ENUM uartComClose(UART_COM_STRUCT * pComInfo)
{
	if (pComInfo)
	{
		pUartClose(&(pComInfo)->uartHandle);
	}
	else
	{
		return HOST_COM_LAYER_ERROR_E;
	}
	return HOST_COM_LAYER_SUCCESS_E;
}


HOST_COM_LAYER_ERROR_ENUM uartComSend(UART_COM_STRUCT * pComInfo,  UINT8* pBuff, UINT16 len, UINT16* byteLenSend)
{
//	BOOLEAN stay = TRUE;
	HOST_COM_LAYER_ERROR_ENUM retVal = HOST_COM_LAYER_SUCCESS_E;
	if (pComInfo)
	{
		(pComInfo)->rxMsgLen=0;
		(pComInfo)->msgCurrentState = RX_STATE_IDLE_E;
		retVal = (HOST_COM_LAYER_ERROR_ENUM)pUartSend( &(pComInfo)->uartHandle, (char *)((pComInfo)->pStuffedBuff),(pComInfo)->stuffedBuffLen); 
	}
	return HOST_COM_LAYER_SUCCESS_E;
}


HOST_COM_LAYER_ERROR_ENUM uartComRecv(UART_COM_STRUCT * pComInfo, UINT8* pBuff, UINT16 len, UINT16* byteLenRecv)
{
	BOOLEAN stay = TRUE;
//	HOST_COM_LAYER_ERROR_ENUM retVal = HOST_COM_LAYER_SUCCESS_E;
	unsigned int bytesRead = 0;
	char val = 0;
	*byteLenRecv = 0;
	char tmpBuf[PVG_PROTOCOL_MSG_MAX_MSG_LEN];
	int tmpInt = 0;

	if (pComInfo)
	{
		while (stay)
		{
			pUartRecv ( &(pComInfo)->uartHandle, &val, &bytesRead);	 //Todo check decrease or increase timeout!!			
			if (bytesRead==0)
			{
				return HOST_COM_LAYER_MSG_IN_PROGRESS_E;
			}
			switch ((pComInfo)->msgCurrentState)
			{
			case RX_STATE_IDLE_E:
				if (val == 'R')
				{
					(pComInfo)->msgCurrentState = RX_STATE_FLAG_FOUND_T_E;
					//memset((void *)pBuff,0x00,180);
				}
				break;
			case RX_STATE_FLAG_FOUND_T_E:
				if (val == 'T')
				{
					(pComInfo)->msgCurrentState = RX_STATE_FLAG_FOUND_N_E;
					//memset((void *)pBuff,0x00,180);
				}
				else
					(pComInfo)->msgCurrentState = RX_STATE_IDLE_E;
				break;
			case RX_STATE_FLAG_FOUND_N_E:
				if (val == 'N')
				{
					(pComInfo)->msgCurrentState = RX_STATE_HEAD_FOUND_E;
					//memset((void *)pBuff,0x00,180);
				}
				else
					(pComInfo)->msgCurrentState = RX_STATE_IDLE_E;
				break;
			case RX_STATE_HEAD_FOUND_E:
				if (val == ' ')
				{
					(pComInfo)->msgCurrentState = RX_STATE_CMDID_FOUND_E;
				}
				else
					(pComInfo)->msgCurrentState = RX_STATE_IDLE_E;
				break;
			case RX_STATE_CMDID_FOUND_E:
				if (val == ' ')
				{
					if(tmpBuf[0]==(pComInfo->cmdId+0x30))
					{
						(pComInfo)->msgCurrentState = RX_STATE_CMDNAME_FOUND_E;
						tmpInt = 0;
					}
					else
						(pComInfo)->msgCurrentState = RX_STATE_IDLE_E;
				}
				else
				{
					tmpBuf[0] = val;
				}
				break;
			case RX_STATE_IN_MSG_E:
				if (val == "*")
				{
					(pComInfo)->msgCurrentState = RX_STATE_ESC_CODE_DETECTED_E;
					stay = FALSE;
				}
				else
				{
					pBuff[(pComInfo)->rxMsgLen]= val;
					(pComInfo)->rxMsgLen++;
				}
				break;

			case RX_STATE_ESC_CODE_DETECTED_E:
				(pComInfo)->msgCurrentState = RX_STATE_IGNORE_E;
				break;

			case RX_STATE_IGNORE_E:
				// not supposed to happen, but we'll allow it for now
				break;
			case RX_STATE_FINISH:
				break;
			}

			// if message is too long, ignore it
			if ((pComInfo)->rxMsgLen > PVG_PROTOCOL_MSG_MAX_MSG_LEN)
			{
				(pComInfo)->msgCurrentState = RX_STATE_IGNORE_E;
				(pComInfo)->rxMsgLen = 0;
				*byteLenRecv=0;
				return HOST_COM_LAYER_ERROR_E;
			}

		}
	}
	else
	{
		return HOST_COM_LAYER_ERROR_E;
	}
	return HOST_COM_LAYER_SUCCESS_E;
}


#ifdef __cplusplus
}
#endif
