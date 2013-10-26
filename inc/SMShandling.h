#ifndef __H_SMSHANDLING_H__
#define __H_SMSHANDLING_H__

/******************************************************************************
 *  File       : SMShandling.h
 ***-----------------------------------------------------------------------------
 *	General description of methods to be shared by this header file
 * - fifo_put(char c)
 * - fifo_get(u8 ndx)
 * - fifo_pop()
 * - fifo_num()
 * - fifo_free()
 * - sms_service_start()
 * - sms_send()
 ***-----------------------------------------------------------------------------
 *	General description of Defines to be shared by this header file
 * - NUM_SERVER_FR (3)
 * - MAX_PHNUM_LEN (15)
 * - SMS_SIZE (160)
 ***-----------------------------------------------------------------------------
 *****************************************************************************/
#include "pistd.h"
#include "adl_global.h"

/*****************************************************************************
*
* global variables and definitions to be shared amoung all files
*
*****************************************************************************/
#define NUM_SERVER_FR       (3)     /* How many servers we can listen to */
//#define MAX_PHNUM_LEN (14) /* Number of chars in a phone num (including null) */
#define SMS_SIZE            (160)   /* chars in sms packet */

/*-----------------------------------------------------------------------------
 * TYPE DEFINITIONS
 * ------------------------------------------------------------------------------*/
/* SMS message-sending status */
typedef enum
{
	SMS_OK, SMS_TIMEOUT, SMS_WEAK_SIG, SMS_ERROR, SMS_NOSVC
} e_smsStatus;
typedef enum
{
	NO_SMS_IN, SMS_IN, SMS_IN_ERR
} e_smsRecStatus;                                               //What is this value used for?
//ascii TraceBuf[256]; // buffer for trace information

/*****************************************************************************
*
* prototypes
*
*****************************************************************************/
void CmdHandlerSMS(adl_atCmdPreParser_t *Cmd);
void fifo_put(char c);
char fifo_get(u8 ndx);
void fifo_pop(void);
u8 fifo_num(void);
u8 fifo_free(void);
bool sms_service_start(void);
UINT8 sms_send(char type);
void SMS_TX_Handler(u8 timerid, void *context);
void SMSHandling_DisplayRegStatus(void);
void SetStartupTrackingMode(int duration_remaining, BOOL enable);
void IPacketTxHandler(u8 timerid, void *context);


#define C_SIZE          160
#define S_SIZE          137
#define L_SIZE          137
#define W_SIZE          147

#define S_SIZE_GPRS     160 + 15
#define L_SIZE_GPRS     160 + 15
#define W_SIZE_GPRS     160 + 15
#define C_SIZE_GPRS     160 + 15
#define M_SIZE_GPRS     160 + 15
#define I_SIZE_GPRS     160 + 15
#define A_SIZE_GPRS		160 + 15	// AEW Jr.

#endif // #ifndef __H_SMSHANDLING_H__
