/** @addtogroup OpenAT
 *@{*/

/*H************************************************************************
 */

/*! \file    XactUtilities.c
 *
 *   \brief   Contains generic utility functions used throughout the code.
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
 *         19Jun09  AndyB    Initial version.
 * NNNNNN  DDMMMYY  Name     Name of function/item changed
 *
 *H*/

/******************************************************************************
 *  Includes
 *****************************************************************************/
#include "adl_global.h"
#include "pistd.h"
#include "XactUtilities.h"
#include "gps.h"
#include "SMShandling.h"
#include "ConfigSettings.h"
#include "status.h"

/******************************************************************************
 *  Defines/Macros
 *****************************************************************************/

/******************************************************************************
 *  Globals
 *****************************************************************************/

/** @brief hexadecimal ascii-to uint16 string converter
 *
 * @par
 * Start point of the application
 *
 * @param number
 * @param base
 * @return integer val of n-char ascii hex string.
 */
UINT32 hex_ascii_2_uint(UINT8 *number, UINT8 base)
{
	//UINT8 checker = 0x55;  /* Stack check debug variable */
	UINT8 value[8];         /* ascii chars converted to uints */
	UINT8 ndx_cntr;         /* index into value[] */
	UINT8 num_digits;       /* number of digits */
	UINT32 result;          /* return result */

	//stack_check(&checker, "soasc2int");

	/* Get the size of the buffer */
	switch (base)
	{
	case 32:
		num_digits = 8;
		break;

	case 24:
		num_digits = 6;
		break;

	case 16:
		num_digits = 4;
		break;

	case 12:
		num_digits = 3;
		break;

	case 8:
		num_digits = 2;
		break;

	default:
		DumpMessage("hex_ascii_2_uint: Error on base\r\n");
		return 0;
	}

	/* Turn the ASCII digits into integers */
	for (ndx_cntr = 0; ndx_cntr < num_digits; ndx_cntr++)
	{
		if (number[ndx_cntr] <= '9')
		{
			value[num_digits - 1 - ndx_cntr] = number[ndx_cntr] - '0';
		}
		else if (number[ndx_cntr] <= 'F')
		{
			value[num_digits - 1 - ndx_cntr] = number[ndx_cntr] - 'A' + 10;
		}
		else
		{
			value[num_digits - 1 - ndx_cntr] = number[ndx_cntr] - 'a' + 10;
		}
	}

	/* Initialize return value */
	result = 0;

	/* Calculate the value.  Notice that the case 16 falls through to the
	 * case 8 */
	result += (0xffUL & (UINT32)(value[1] * 0x10 + value[0]));

	/* add in the higher order digits if there */
	if (base >= 12)
	{
		result += (0xfffUL & (UINT32)(value[2] * 0x100));
	}
	if (base >= 16)
	{
		result += (0xffffUL & (UINT32)(value[3] * 0x1000));
	}
	if (base >= 24)
	{
		result += (0xffffffUL & (UINT32)(value[5] * 0x100000 + value[4] * 0x10000));
	}

	if (base >= 32)
	{
		result += (UINT32)(value[7] * 0x10000000 + value[6] * 0x1000000);
	}

	//stack_check(&checker, "eoasc2int");

	return result;
}

/** @brief convert int to hex ascii
 *
 * @par Converts a UINT number to an ascii string
 * There is a wavecom equivalent to this that should be used
 * for future code wm_itohexa().
 * @param number
 * @param result
 * @param base
 * @return void
 */
void uint_2_hex_ascii(UINT32 number, UINT8 *result, UINT8 base)
{
	// UINT8 checker = 0x55;  /* Stack check debug variable */
	UINT32 digits[8] =
	{
		0, 0, 0, 0, 0, 0, 0, 0
	};                                                  /* digits (hex) */
	UINT8 digit_cntr;                                   /* index into digits[] */
	UINT8 num_digits = 0;                               /* number of digits in result */

	/* Get digits. Notice that base 32 selection falls through without a break
	 * to the 16 etc.  */
	TRACE((1, "Number: %d", number));
	switch (base)
	{
	case 32:
		if (number > 0xfffffff)
		{
			digits[7] = (number / 0x10000000);
			//temp = (number / 0x10000000);
		}
		if (number > 0xffffff)
		{
			digits[6] = (number - digits[7] * 0x10000000) / 0x1000000;
		}
		if (number > 0xfffff)
		{
			digits[5] = (number - digits[7] * 0x10000000 - digits[6] *
			             0x1000000) / 0x100000;
		}
		if (number > 0xffff)
		{
			digits[4] = (number - digits[7] * 0x10000000 - digits[6] *
			             0x1000000 - digits[5] * 0x100000) / 0x10000;
		}                 /* Fall through */

	case 16:
		if (number > 0xfff)
		{
			digits[3] = (number - digits[7] * 0x10000000 - digits[6] *
			             0x1000000 - digits[5] * 0x100000 - digits[4] * 0x10000) / 0x1000;
		}
		if (number > 0xff)
		{
			digits[2] = (number - digits[7] * 0x10000000 - digits[6] *
			             0x1000000 - digits[5] * 0x100000 - digits[4] * 0x10000 -
			             digits[3] * 0x1000) / 0x100;
		}                 /* Fall through */

	case 8:
		if (number > 0xf)
		{
			digits[1] = (number - digits[7] * 0x10000000 - digits[6] *
			             0x1000000 - digits[5] * 0x100000 - digits[4] * 0x10000 -
			             digits[3] * 0x1000 - digits[2] * 0x100) / 0x10;
		}

		digits[0] = number - digits[7] * 0x10000000 - digits[6] * 0x1000000 -
		            digits[5] * 0x100000 - digits[4] * 0x10000 - digits[3] * 0x1000 -
		            digits[2] * 0x100 - digits[1] * 0x10;
	}
	/* Get the size of the buffer needed */
	switch (base)
	{
	case 32:
		num_digits = 8;
		break;

	case 16:
		num_digits = 4;
		break;

	case 8:
		num_digits = 2;
		break;
	}
	TRACE((1, "num_digits=%d", num_digits));
	/********** Convert lowest digit to hex ASCII ******************/
	for (digit_cntr = 0; digit_cntr < num_digits; digit_cntr++)
	{
		if (digits[num_digits - digit_cntr - 1] <= 9)
		{
			result[digit_cntr] = (UINT8)(digits[num_digits - digit_cntr - 1] + '0');
			TRACE((1, "Digit: %c", result[digit_cntr]));
		}
		else
		{
			result[digit_cntr] = (UINT8)(digits[num_digits - digit_cntr - 1] - 10 + 'a');
			TRACE((1, "Digit: %c", result[digit_cntr]));
		}
	}
	// TRACE((1, "Result: %s", result));

	//wm_sprintf(g_traceBuf, "Result: %s\r\n", (char*)(result));
	//DumpMessage(g_traceBuf);
	//stack_check(&checker, "eouint2");
}


/** @brief dump message
 *
 * @par
 *  Prints out the string to the AT command prompt
 * @param Msg
 * @return void
 */
void DumpMessage(char *Msg)
{
#if !defined(REGRESSION_TESTING)
#if USB_PORT_ENABLE
	adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_USB, Msg);
#else
	adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_UART1, Msg);
#endif
#endif
	TRACE((1, Msg));
}


/** @brief dump message our USB port
 *
 * @par Dump string message our the USB port
 *
 * @param Msg
 * @param USB
 * @return void
 */
void DumpMessageUSB(char *Msg, int USB)
{
	if (USB)
	{
		adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_USB, Msg);
	}
	else
	{
		adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_UART1, Msg);
	}

	TRACE((1, Msg));
}


/** @brief display error code
 *
 * @par Prints out the error message and line number
 * as an AT response
 * @param ErrorFunction
 * @param FileName
 * @param LineNumber
 * @param ErrorCode
 * @return void
 */
void DisplayErrorCode(char *ErrorFunction, char *FileName, int LineNumber, int ErrorCode)
{
	ascii ErrorMsg[128];
	wm_sprintf(ErrorMsg, "\r\n *** ERROR return from %s method:%s:%d code:%d \r\n", FileName, ErrorFunction, (int )LineNumber, (int )ErrorCode);
	DumpMessage(ErrorMsg);
	switch (ErrorCode)
	{
	case ADL_RET_ERR_PARAM:
		wm_sprintf(ErrorMsg, "\r\n     ADL_RET_ERR_PARAM\r\n");
		DumpMessage(ErrorMsg);
		break;

	case ADL_RET_ERR_UNKNOWN_HDL:
		wm_sprintf(ErrorMsg, "\r\n     ADL_RET_ERR_UNKNOWN_HDL\r\n");
		DumpMessage(ErrorMsg);
		break;

	case ADL_RET_ERR_ALREADY_SUBSCRIBED:
		wm_sprintf(ErrorMsg, "\r\n     ADL_RET_ERR_ALREADY_SUBSCRIBED\r\n");
		DumpMessage(ErrorMsg);
		break;

	case ADL_RET_ERR_NOT_SUBSCRIBED:
		wm_sprintf(ErrorMsg, "\r\n     ADL_RET_ERR_NOT_SUBSCRIBED\r\n");
		DumpMessage(ErrorMsg);
		break;

	case ADL_RET_ERR_FATAL:
		wm_sprintf(ErrorMsg, "\r\n     ADL_RET_ERR_FATAL\r\n");
		DumpMessage(ErrorMsg);
		break;

	case ADL_RET_ERR_BAD_HDL:
		wm_sprintf(ErrorMsg, "\r\n     ADL_RET_ERR_BAD_HDL\r\n");
		DumpMessage(ErrorMsg);
		break;

	case ADL_RET_ERR_BAD_STATE:
		wm_sprintf(ErrorMsg, "\r\n     ADL_RET_ERR_BAD_STATE\r\n");
		DumpMessage(ErrorMsg);
		break;

	case ADL_RET_ERR_PIN_KO:
		wm_sprintf(ErrorMsg, "\r\n     ADL_RET_ERR_PIN_KO\r\n");
		DumpMessage(ErrorMsg);
		break;

	case ADL_RET_ERR_NO_MORE_HANDLES:
		wm_sprintf(ErrorMsg, "\r\n     ADL_RET_ERR_NO_MORE_HANDLES\r\n");
		DumpMessage(ErrorMsg);
		break;

	case ADL_RET_ERR_DONE:
		wm_sprintf(ErrorMsg, "\r\n     ADL_RET_ERR_DONE\r\n");
		DumpMessage(ErrorMsg);
		break;

	case ADL_RET_ERR_OVERFLOW:
		wm_sprintf(ErrorMsg, "\r\n     ADL_RET_ERR_OVERFLOW\r\n");
		DumpMessage(ErrorMsg);
		break;

	case ADL_RET_ERR_NOT_SUPPORTED:
		wm_sprintf(ErrorMsg, "\r\n     ADL_RET_ERR_NOT_SUPPORTED\r\n");
		DumpMessage(ErrorMsg);
		break;

	case ADL_RET_ERR_NO_MORE_TIMERS:
		wm_sprintf(ErrorMsg, "\r\n     ADL_RET_ERR_NO_MORE_TIMERS\r\n");
		DumpMessage(ErrorMsg);
		break;

	case ADL_RET_ERR_NO_MORE_SEMAPHORES:
		wm_sprintf(ErrorMsg, "\r\n     ADL_RET_ERR_NO_MORE_SEMAPHORES\r\n");
		DumpMessage(ErrorMsg);
		break;

	case ADL_RET_ERR_SERVICE_LOCKED:
		wm_sprintf(ErrorMsg, "\r\n     ADL_RET_ERR_SERVICE_LOCKED\r\n");
		DumpMessage(ErrorMsg);
		break;

	case ADL_RET_ERR_SPECIFIC_BASE:
		wm_sprintf(ErrorMsg, "\r\n     ADL_RET_ERR_SPECIFIC_BASE\r\n");
		DumpMessage(ErrorMsg);
		break;

	default:
		wm_sprintf(ErrorMsg, "\r\n     unknown/undefined\r\n");
		DumpMessage(ErrorMsg);
	}
}


/*@}*/
