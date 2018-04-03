#ifndef _HOST_COM_LAYER_ERR_H_
#define _HOST_COM_LAYER_ERR_H_


#include "PVG_defs.h"


#ifdef __cplusplus
extern "C" {
#endif


typedef enum
{
	HOST_COM_LAYER_SUCCESS_E,
	HOST_COM_LAYER_ERROR_E,
	HOST_COM_LAYER_PORT_BUSY_E,
	HOST_COM_LAYER_READ_ERR_E,
	HOST_COM_LAYER_READ_TIMEOUT_ERR_E,
	HOST_COM_LAYER_SEND_ERR_E,
	HOST_COM_LAYER_MSG_IN_PROGRESS_E
} HOST_COM_LAYER_ERROR_ENUM;

#define SUCCESS(val) (val == HOST_COM_LAYER_SUCCESS_E)

#ifdef __cplusplus
}
#endif

#endif /* _HOST_COM_LAYER_ERR_H_ */
