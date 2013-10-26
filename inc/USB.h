#ifndef __USB_H__
#define __USB_H__

#include "SocketDescriptor.h"

typedef enum
{
	CLOSE_USB_STATE = 0,
	OPEN_USB_STATE,
	USB_FULL_INIT_STATE
} eFlowCmdState;

typedef enum
{
	NORMAL_MODE,
	WYHOOK_MODE
} USB_MODE;

void USBInterface(void);
s8 ChangeV24State(commPort_type i_comm, u8 FlowState);
void EnterAlarmMode();

#endif
