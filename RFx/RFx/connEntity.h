#ifndef _CONN_ENTITY_H_
#define _CONN_ENTITY_H_


#include "PVG_defs.h"
#include "hostComLayerErr.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct{
	UINT16						entityindex;
	CONNECTION_COM_TYPE_ENUM	connType;
	void*						connParams;
	CEVENT						connEvent;
	CEVENT						msgClientRecvEvent;
	CONNECTION_STATUS_ENUM		connStatus;
	INT32						sleepTime;
	UINT16						recvMsgLen;
	UINT16						destinationId;
	UINT32						msgId;
} CONNECTION_COM_INFO_STRUCT;


#ifdef __cplusplus
}
#endif

#endif /* _CONN_ENTITY_H_ */
