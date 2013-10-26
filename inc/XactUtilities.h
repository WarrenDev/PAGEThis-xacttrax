/*H************************************************************************
 */

/*! \file    XactUtilities.h
 *
 *   \brief   utilities
 *
 *   \details
 *
 *   \note    Other help for the reader, including references.
 *
 *
 *//*
 *
 * CHANGES :
 *
 * REF NO  DATE     WHO      DETAIL
 *         ???      PatN     First version
 * NNNNNN  DDMMMYY  Name     Name of function/item changed
 *
 *H*/

#ifndef __XACT_UTILITIES__H__
#define __XACT_UTILITIES__H__

/******************************************************************************
 *  Includes
 *****************************************************************************/
#include "pistd.h"

#define FW_MINOR_VER        0x00	// Custom PAGEThis firmware build version number - AEW
#define FW_MAJOR_VER        0x02

//#define FW_MINOR_VER        0x01
//#define FW_MAJOR_VER        0x04

#define HW_MINOR_VER        0x01
#define HW_MAJOR_VER        0x02

/***************************************************************************
 *  Globals
 ****************************************************************************/
#define RX_BUFFSIZE         (256)       /* Size of uart receive buffer */
#define SMS_TX_BUFFSIZE     (161 + 15)  /* Size of uart transmit buffer */
#define LOG_MSG_PER_SEG     (3)         /* number of sms-size messages per main flash seg */

/* fence status - device location */
typedef enum
{
	FIX_INF, FIX_OOF
} e_FenceStatus;

/******************************************************************************
 *  Prototypes
 *****************************************************************************/
void uint_2_hex_ascii(UINT32, UINT8 *, UINT8);
UINT32 hex_ascii_2_uint(UINT8 *, UINT8);

void DumpMessage(char *Msg);
void DumpMessageUSB(char *Msg, int USB);
void DisplayErrorCode(char *ErrorFunction, char *FileName, int LineNumber, int ErrorCode);

extern char g_traceBuf[256];

#define D_TRACE(str ...)          \
	{                             \
		sprintf(g_traceBuf, str); \
		TRACE((1, g_traceBuf));   \
	}
#endif // __GPF_UTILITIES__H__
