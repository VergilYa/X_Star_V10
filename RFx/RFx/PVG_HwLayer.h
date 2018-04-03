#ifndef _PVG810_HW_LAYER_H_
#define _PVG810_HW_LAYER_H_

#include "PVG_Errors.h"
#include "pserial.h"
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <errno.h>


#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------
    UART functions
------------------------------------------------------*/

PVG_ERROR_MSG_ENUM pvgHwLayerUartByteWrite(PUART* pUartHandle,BYTE val); 
PVG_ERROR_MSG_ENUM pvgHwLayerUartByteRead (PUART* pUartHandle,BYTE* pVal,UINT8* isValidData);
PVG_ERROR_MSG_ENUM pvgHwLayerUartClose(PUART *pUartHandle);
PVG_ERROR_MSG_ENUM pvgHwLayerUartInit(PUART *pUartHandle);
PVG_ERROR_MSG_ENUM pvgHwLayerUartOpen(PUART * pUartHandle, unsigned int PortNumber);
int UART1RdChar(unsigned char *pChar);


void *UART1RecvThread(void *ptr);
void *UART1SendThread(void *ptr);

int UART1TxBuffRemain(void);
int UART1TxBufRead(unsigned char *pChar);

#ifdef __cplusplus
}
#endif

#endif  /*_PVG_HW_LAYER_H_*/
