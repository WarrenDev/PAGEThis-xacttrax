#ifndef __diagnose_h__
#define __diagnose_h__

#define DUMP_USB(x)    adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_USB, x)

typedef enum
{
	DIAGNOSE_OFF,
	DIAGNOSE_ON
} DIAGNOSE_MODE;

typedef enum
{
	LED1,
	LED2,
	LED3,
	LED4,
	LED5,
	LED6,
	VM,
	IND_OFF,
	IND_ON,
	SENSE
} INDICATOR;

typedef enum
{
	AB,
	PB,
	MS,
	AC,
	LED1_SRC,
	LED2_SRC,
	LED3_SRC,
	LED4_SRC,
	LED5_SRC,
	LED6_SRC,
	VM_SRC
} SOURCE;

void diagnose_handler(adl_atCmdPreParser_t *Cmd);
DIAGNOSE_MODE diagnose_get_mode(void);

void diagnose_activate_indicator(INDICATOR ind, SOURCE src);
void diagnose_deactivate_indicator(INDICATOR ind, SOURCE src);

INDICATOR diagnose_get(SOURCE src);
void diagnose_handler_ota(char *cmd_string);

#endif
