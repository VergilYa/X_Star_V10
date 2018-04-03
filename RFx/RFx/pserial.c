/*****************************************************
*
*	pserial.c
*/

#include "pserial.h"
#include "hostComLayerErr.h"
#include <stdio.h>


#ifdef __cplusplus
extern "C" {
#endif

/*******************************************LINUX********************************/
#include "PVG_HwLayer.h"
//#include <pthread.h>
//#include <signal.h>
#include <errno.h>



void pUartCloseX(PUART *uartHandle)
{
	pvgHwLayerUartClose(uartHandle);
}

int pUartRecvX(PUART* uartHandle, char *inBuf, unsigned int *pBytesRead)
{
	HOST_COM_LAYER_ERROR_ENUM retVal = HOST_COM_LAYER_SUCCESS_E;
 	pvgHwLayerUartByteRead(uartHandle,(BYTE *)inBuf, (UINT8*)pBytesRead);
	return retVal;
}

extern UART1_DATA_STRUCT uart1data;

int pUartSendX(PUART *uartHandle, char *lpByte, unsigned int dwBytesToWrite)
{
	HOST_COM_LAYER_ERROR_ENUM retVal = HOST_COM_LAYER_SUCCESS_E;
	unsigned int i;
	PVG_ERROR_MSG_ENUM ret=PVG_SUCCESS_MSG_E;

	while(EBUSY==pthread_mutex_lock(&(uart1data.tx_mutex)) )
		;

	for (i = 0; i < dwBytesToWrite; i++)
	{
		ret = pvgHwLayerUartByteWrite(uartHandle,lpByte[i]);
	}

	pthread_mutex_unlock(&(uart1data.tx_mutex));

	if(ret != PVG_SUCCESS_MSG_E)
		retVal = HOST_COM_LAYER_SEND_ERR_E;
	
	return retVal;
}

int pUuartOpenX(PUART * pUartHandle, unsigned int PortNumber)
{
	HOST_COM_LAYER_ERROR_ENUM retVal = HOST_COM_LAYER_SUCCESS_E;
	PVG_ERROR_MSG_ENUM ret;
	
	ret=pvgHwLayerUartOpen(pUartHandle,PortNumber);

	if(ret != PVG_SUCCESS_MSG_E)
		retVal = HOST_COM_LAYER_ERROR_E;
	return retVal;
}

int pUuartInitX(PUART *pUartHandle)
{
	return 0;
}



/*******************************************CSERIAL*************************************/

void pUartClose(PUART *uartHandle)
{
	pUartCloseX(uartHandle);
}

int pUartRecv(PUART * uartHandle, char *inBuf, unsigned int *pBytesRead)
{
	return pUartRecvX(uartHandle, inBuf, pBytesRead);
}

int pUartSend(PUART * uartHandle, char * lpByte, unsigned int dwBytesToWrite)
{
	return pUartSendX(uartHandle, lpByte, dwBytesToWrite);
}

int pUuartOpen(PUART * pUartHandle, unsigned int PortNumber)
{
	return pUuartOpenX(pUartHandle, PortNumber);
}

int pUuartInit(PUART *pUartHandle)
{
	return 0; //pUuartInitX(pUartHandle);
}

int serial_compilation()
{
	printf("__linux__ defined\n");
	printf("end of serial_compilation\n");
    return 0;
}

#ifdef __cplusplus
}
#endif 

/*****************************************************/

