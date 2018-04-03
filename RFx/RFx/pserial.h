
/*****************************************************
*
*	pserial.h
*/
#ifndef PSERIAL_H
#define PSERIAL_H
 
#ifdef __cplusplus
extern "C" {
#endif 

typedef struct{
	int      uartHandle;
} PUART;


int serial_compilation();

int pUuartOpen(PUART *pUartHandle, unsigned int PortNumber);
void pUartClose(PUART * uartHandle);
int pUartRecv (PUART * uartHandle, char *inBuf, unsigned int *pBytesRead);
int pUartSend (PUART * uartHandle, char * lpByte, unsigned int dwBytesToWrite);
int pUuartInit(PUART *pUartHandle);

int pUuartOpenX(PUART *pUartHandle, unsigned int PortNumber);
void pUartCloseX(PUART * uartHandle);
int pUartRecvX (PUART * uartHandle, char *inBuf, unsigned int *pBytesRead);
int pUartSendX (PUART * uartHandle, char * lpByte, unsigned int dwBytesToWrite,);


#ifdef __cplusplus
}
#endif 

#endif//PSERIAL_H
