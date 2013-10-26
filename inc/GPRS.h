#ifndef __GPRS_H__
#define __GRPS_H__

#include "wip.h"

#define CONFIG_PKT_SIZE_TCP    160

typedef enum
{
	TCP_NOT_INIT,
	TCP_CANT_CONNECT,
	TCP_TRYING,
	TCP_CONNECT
} TCP_STATUS;

typedef enum
{
	TCP_CONNECTION_TYPE,
	UDP_CONNECTION_TYPE
} TCP_OR_UDP;

void StartGPRS(void);
void GetTCPStatus(TCP_STATUS *status);
int TCPTransmit(int size);
wip_in_addr_t GPRS_get_IPAddr(void);
void ClearTCPStatus(void);
TCP_OR_UDP GPRS_GetConnectionType(void);
int GPRS_HaveBearer(void);
void ConnectFTP(u8 timerid, void *context);

#endif
