/////////////////////////////////////////////////////////////////////////////////
//                                                                             //
//    DESCRIPTION:                                                             //
//                                                                             //
//    AUTHOR:                                                                  //
//                                                                             //
//    HISTORY:                                                                 //
//                                                                             //
/////////////////////////////////////////////////////////////////////////////////

#include "PVG_HwLayer.h"
#include "PVG_defs.h"
#include "string.h"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <errno.h> 

#include <ctype.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>
/////////////////////////////////////////////////////////////////////////////////
//  FUNCTION: pvgHwLayerUartByteWrite										   //
//                                                                             //
//  PARAMETERS:                                                                //
//                                                                             //
//  DESCRIPTION: write byte val						                           //
//                                                                             //
//  RETURNS: PVG_ERROR_MSG_ENUM                                             //
/////////////////////////////////////////////////////////////////////////////////
PVG_ERROR_MSG_ENUM pvgHwLayerUartByteWrite(PUART* pUartHandle,BYTE val)
{
	PVG_ERROR_MSG_ENUM retVal=PVG_SUCCESS_MSG_E;

	uart1data.UART1TxBuff[uart1data.UART1TxBuffTail] = val; 
	uart1data.UART1TxBuffTail++;
	if (uart1data.UART1TxBuffTail == UART1_TX_BUFF_SIZE)
		uart1data.UART1TxBuffTail = 0; 

	return retVal;
}
/////////////////////////////////////////////////////////////////////////////////
//  FUNCTION: pvgHwLayerUartByteRead										   //
//                                                                             //
//  PARAMETERS:                                                                //
//                                                                             //
//  DESCRIPTION: read byte val						                           //
//                                                                             //
//  RETURNS: PVG_ERROR_MSG_ENUM                                             //
/////////////////////////////////////////////////////////////////////////////////
PVG_ERROR_MSG_ENUM pvgHwLayerUartByteRead (PUART* pUartHandle,BYTE* pVal,UINT8* isValidData)
{
	PVG_ERROR_MSG_ENUM retVal=PVG_SUCCESS_MSG_E;

	*isValidData=UART1RdChar(pVal);

	return retVal;
	
}

PVG_ERROR_MSG_ENUM pvgHwLayerUartClose(PUART *pUartHandle)
{
	PVG_ERROR_MSG_ENUM retVal=PVG_SUCCESS_MSG_E;
	uart1data.UartThreadRun = 0;

	close(pUartHandle->uartHandle);

	pthread_mutex_destroy(&(uart1data.rx_mutex));
	pthread_mutex_destroy(&(uart1data.tx_mutex));
	return retVal;
}

PVG_ERROR_MSG_ENUM pvgHwLayerUartInit(PUART *pUartHandle)
{
    struct termios Opt;    //定义termios结构
	PVG_ERROR_MSG_ENUM retVal=PVG_SUCCESS_MSG_E;

    if(tcgetattr(pUartHandle->uartHandle, &Opt) != 0)
    {
    	printf("ttyS1 tcgetattr error.\n");
        retVal=PVG_ERROR_COM_PORT_BUSY_E;
        return retVal;
    }
    
	tcflush(pUartHandle->uartHandle, TCIOFLUSH);
    cfsetispeed(&Opt, B115200);
    cfsetospeed(&Opt, B115200);

	
	//Opt.c_lflag 	   = 0; 
	//Opt.c_oflag 	   = 0; 
	//Opt.c_iflag 	   = 0; 
	//8位数据位、1位停止、无校验位
	Opt.c_cflag &= ~PARENB;

	Opt.c_cflag &= ~PARENB; //清除校验位
	Opt.c_iflag &= ~INPCK; //enable parity checking	

	Opt.c_cflag &= ~CSTOPB;

	Opt.c_cflag &= ~CSIZE;
	Opt.c_cflag |= CS8;

	Opt.c_cflag |= (CLOCAL|CREAD);

	Opt.c_cc[VTIME]    = 1;   /**//*  */ 
	Opt.c_cc[VMIN]	   = 0;   /**//*  */ 

	//Raw Mode
	Opt.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*/

	Opt.c_oflag  &= ~OPOST;   /*Output*/
	Opt.c_oflag &= ~(ONLCR | OCRNL); //添加的

	Opt.c_iflag &= ~(IXON | IXOFF | IXANY);
	Opt.c_iflag &= ~(ICRNL | INLCR);

    if(tcsetattr(pUartHandle->uartHandle, TCSANOW, &Opt) != 0)
    {
    	printf("ttyS1 tcsetattr error.\n");
        retVal=PVG_ERROR_COM_PORT_BUSY_E;
        return retVal;
    }
    tcflush(pUartHandle->uartHandle, TCIOFLUSH);

	fcntl(pUartHandle->uartHandle, F_SETFL, O_NONBLOCK | FNDELAY); //无数据立即返回;  fcntl(fd, F_SETFL, 0);取消前一句功能

	return retVal;
}

#define THREAD_PRIORITY_HIGH  sched_get_priority_max(SCHED_FIFO)
#define THREAD_PRIORITY_MID   ((sched_get_priority_max(SCHED_FIFO)-sched_get_priority_min(SCHED_FIFO))/2)
#define THREAD_PRIORITY_LOW   sched_get_priority_min(SCHED_FIFO)


static int hutil_set_priority_attr(pthread_attr_t *attr, int priority)
{
    struct sched_param  schedParam;

    /* Initialize the thread attributes */
    if (pthread_attr_init(attr)) 
	{
        printf("%s():%d ERROR - Failed to initialize thread attrs\n",__FUNCTION__, __LINE__);
        return 1;
    }
	/* Force the thread to use custom scheduling attributes */
    if (pthread_attr_setinheritsched(attr, PTHREAD_EXPLICIT_SCHED)) 
	{
        printf("%s():%d ERROR - Failed to set schedule inheritance attribute\n",__FUNCTION__, __LINE__);
        return 1;
    }

	/* Set the thread to be fifo real time scheduled */
    if (pthread_attr_setschedpolicy(attr, SCHED_FIFO))  
	{
        printf("%s():%d ERROR - Failed to set FIFO scheduling policy\n",__FUNCTION__, __LINE__);
        return 1;
    }
	/* Set the thread priority */
    schedParam.sched_priority = priority;
    if (pthread_attr_setschedparam(attr, &schedParam)) 
	{
        printf("%s():%d ERROR - Failed to set scheduler parameters\n",__FUNCTION__, __LINE__);
        return 1;
    }

    return 0;
}

PVG_ERROR_MSG_ENUM pvgHwLayerUartOpen(PUART * pUartHandle, unsigned int PortNumber)
{
	int ret;
	PVG_ERROR_MSG_ENUM retVal=PVG_SUCCESS_MSG_E;


	memset((void *)&uart1data,0x00,sizeof(UART1_DATA_STRUCT));

	if ( pthread_mutex_init(&(uart1data.rx_mutex), NULL) != 0 )
	{
		printf ("Init UART1 Rx Mutex  error!\n");
		retVal=PVG_ERROR_COM_PORT_BUSY_E;
		return retVal;
	}
	if ( pthread_mutex_init(&(uart1data.tx_mutex), NULL) != 0 )
	{
		printf ("Init UART1 Tx Mutex  error!\n");
		retVal=PVG_ERROR_COM_PORT_BUSY_E;
		return retVal;
	}

	uart1data.UartThreadRun = 1;

	char szPort[20];
	pthread_attr_t attr;

    sprintf( szPort, "/dev/ttyS%d", PortNumber ) ;

	pUartHandle->uartHandle = open(szPort,O_RDWR | O_NOCTTY | O_NDELAY);
	if (-1 == pUartHandle->uartHandle)
	{ 
		printf("Can not open %s\n",szPort);
		retVal=PVG_ERROR_COM_PORT_BUSY_E;
		return retVal;
	}	

	retVal=pvgHwLayerUartInit(pUartHandle);

	//hutil_set_priority_attr(&attr,THREAD_PRIORITY_HIGH);
	
	ret=pthread_create(&(uart1data.UartThreadId),&attr,UART1SendThread,(void *)pUartHandle);
	if(ret!=0)
	{
		printf ("Create UART send pthread error!\n");
		memset((void *)uart1data.UartThreadId,0,sizeof(pthread_t));
		uart1data.UartThreadRun = 0;
		retVal=PVG_ERROR_COM_PORT_BUSY_E;
	}
	
	ret=pthread_create(&(uart1data.UartThreadId),&attr,UART1RecvThread,(void *)pUartHandle);

	if(ret!=0)
	{
		printf ("Create UART receive pthread error!\n");
		memset((void *)uart1data.UartThreadId,0,sizeof(pthread_t));
		uart1data.UartThreadRun = 0;
		retVal=PVG_ERROR_COM_PORT_BUSY_E;
	}

	return retVal;
}


int UART1RdChar(unsigned char *pChar)
{
	int valid;

	
	if (uart1data.UART1RxBuffHead != uart1data.UART1RxBuffTail) {
		while(EBUSY==pthread_mutex_lock(&(uart1data.rx_mutex)) )
			;

		*pChar = uart1data.UART1RxBuff[uart1data.UART1RxBuffHead];
        uart1data.UART1RxBuffHead++;
		uart1data.UART1RxBuffHead = uart1data.UART1RxBuffHead % UART1_RX_BUFF_SIZE;

		pthread_mutex_unlock(&(uart1data.rx_mutex));

        valid = 1;
	}
    else {
        valid = 0;
    }

	//printf("read tail:%d - head:%d\n",uart1data.UART1RxBuffTail,uart1data.UART1RxBuffHead);
    return valid;
}

//static int Rxcount=0;

void *UART1RecvThread(void *ptr)
{
	PUART *pUartHandle;

	int 	i = 0, read_byte = 0;		
	fd_set fds;  
	char read_buffer[256];
	struct timeval tv;		//select等待3秒，3秒轮询，要非阻塞就置0  
	
	pUartHandle = (PUART *)ptr;	

	
	//fcntl(pUartHandle->uartHandle, F_SETFL, 0); //无数据立即返回;	fcntl(fd, F_SETFL, 0);取消前一句功能
	while(uart1data.UartThreadRun)
	{
		FD_ZERO(&fds); 
		FD_SET(pUartHandle->uartHandle,&fds);
		
		memset(read_buffer, 0, sizeof(read_buffer));
	
	
		tv.tv_sec = 5;
		tv.tv_usec = 0;
	
		switch(select(pUartHandle->uartHandle+1,&fds,NULL,NULL,&tv))	//read, write, error fd
		{  
			case -1: 
				perror("select error.\n");
				break;		  //select error
			case 0:
				//printf("select time out.\n");
				//tcflush(pUartHandle->uartHandle, TCIFLUSH);
				break; 
			default:  
				if(FD_ISSET(pUartHandle->uartHandle,&fds))
				{  
					//tcflush(pUartHandle->uartHandle, TCIFLUSH);
					read_byte = read(pUartHandle->uartHandle,read_buffer,sizeof(read_buffer)); 
					//tcflush(pUartHandle->uartHandle, TCIFLUSH);
					//printf("read byge:%d\n",read_byte);
					while(EBUSY==pthread_mutex_lock(&(uart1data.rx_mutex)) )
						;
					
					for(i=0;i<read_byte;i++)
					{
					/*
						printf("%.2x ",read_buffer[i]);
						Rxcount++;
						if(read_buffer[i]==0x7e && Rxcount>1)
						{
							printf(":%d-%d\n",Rxcount,read_byte);
							Rxcount=0;
						}
*/
						//write(pUartHandle->uartHandle, &read_buffer[i] ,1);
						uart1data.UART1RxBuff[uart1data.UART1RxBuffTail] = read_buffer[i]; 
						uart1data.UART1RxBuffTail++;
						if (uart1data.UART1RxBuffTail == UART1_RX_BUFF_SIZE)
							uart1data.UART1RxBuffTail = 0; 
					}
					//printf("read len:%d\n",read_byte);
					
					pthread_mutex_unlock(&(uart1data.rx_mutex));
				}
				
		}
	}

	return 0;
}

void *UART1SendThread(void *ptr)
{
	PUART *pUartHandle;
    int len=0;  
    char buf[512]; 
	int i;

	pUartHandle = (PUART *)ptr;	

    while(uart1data.UartThreadRun)  
    {  
		len=UART1TxBuffRemain();
		if(len==0)
		{
			pvg810HwLayerSleep(2);
			continue;
		}

		i=UART1TxBufRead((unsigned char *)buf);
		{
			//len = write(client_fd, buf, i);
			len = write(pUartHandle->uartHandle,buf,i); 
			/*	
			if(buf[9]==0x59 || buf[13]!=0)
			{
				printf("Write %d Byte!\n",len);
				for(int j=0;j<i;j++)
					printf("%.2x ",buf[j]);
				printf("\n");
			}
	*/
	//		printf("write len:%d \n",len);
			if (-1 == len){
				if (errno == EINTR){
					continue;
				} else {
					perror("call to write!");
					break;
				}
			}
		}
    }  
    return NULL;  
}