#ifndef __POWERCTRL_H__
#define __POWERCTRL_H__

#define FAST_IDLE_ACCEL_LEVEL    0x20

typedef enum
{
	FAST_IDLE_REQ_ACTIVE,
	FAST_IDLE_REQ_RELEASE
} FAST_IDLE_REQ;

typedef enum
{
	SMS,
	GPRS
} TRANSMIT_MODE;

// Number of seconds to remain awake from timed wakeup.
#define TIMED_WAKEUP_DURATION    60

void EnterSlowIdleMode(void);
void ExitSlowIdleMode(void);
int GetSlowIdleMode(void);
void InitPowerCtrl(void);
TRANSMIT_MODE PowerCtrl_GetTXMode(void);
void DisplayPowerState(void);
void PowerOffModem(u8 timerID, void *Context);

#endif
