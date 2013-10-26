/** @addtogroup Storage
 *@{*/

/*H***************************************************************************
 */

/** @file FlashTest.c
 *
 * @brief Manage the internal flash on the WMP100.
 *
 * @note    Other help for the reader, including references.
 *
 *
 *//*
 *
 *
 *H*/
#include "adl_global.h"
#include "pistd.h"
#include "FlashTest.h"
#include "XactUtilities.h"

#define FLASH_SIZE    (4)

/** @brief Calls the adl_flhSubscribe function on the provided handle and number of objects
 *
 * @param flhHandle
 * @param numObjects
 * @return OK on success
 * @return Error Code on failure
 */
s8 FlashSubscribe(char const *const flhHandle, u16 numObjects)
{
	s8 RetCode = 0;

	RetCode = adl_flhSubscribe((char *)flhHandle, numObjects);
	if ((RetCode == OK) || (RetCode == ADL_RET_ERR_ALREADY_SUBSCRIBED))
	{
		TRACE((1, "Successfully subscribed to the flash"));
		RetCode = OK;
	}
	else
	{
		DisplayErrorCode("adl_flhSubscribe", __FILE__, __LINE__ - 2, RetCode);
	}

	return RetCode;
}


/** @brief Calls the adl_flhExist function on the provided handle and check to see if the ID exists
 *
 * @param flhHandle
 * @param flhId
 * @return < 0 on failure
 * @return >= 0 on Success
 */
s8 FlashExists(char const *const flhHandle, u16 flhId)
{
	s32 RetCode = 0;
	RetCode = adl_flhExist((char *)flhHandle, flhId);

	TRACE((1, "FlashExist "));
	TRACE((1, "Ids %x", flhId));

	if (RetCode < 0)
	{
		DisplayErrorCode("adl_flhExist", __FILE__, __LINE__ - 7, RetCode);
	}
	else
	{
		TRACE((1, "Successfully found the flash ID and its length is %d", RetCode));
	}
	return RetCode;
}


/** @brief Calls the adl_flhRead function on the provided handle and number of objects
 *
 * @param i_Handle
 * @param i_Id
 * @param i_Length
 * @param o_Data
 * @return < 0 on failure
 * @return >= 0 on Success
 */
s8 FlashRead(char const *const i_Handle, u16 i_Id, u16 i_Length, u8 *const o_Data)
{
	s8 RetCode = 0;

	RetCode = adl_flhRead((char *)i_Handle, i_Id, i_Length, o_Data);
	//wm_sprintf(g_traceBuf, "\r\nRetCode from adl_flhRead: %d\r\n", RetCode);
	//DumpMessage(g_traceBuf);
	if (RetCode < 0)
	{
		DisplayErrorCode("adl_flhRead", __FILE__, __LINE__ - 5, RetCode);
	}
	else
	{
		TRACE((1, "Successfully read from flash ID: %d", i_Id));
	}
	return RetCode;
}


/** @brief Calls the adl_flhWrite function on the provided handle
 *
 * @par description: Calls the adl_flhWrite function on the provided handle at the specified ID. The data in o_Data is the data to be written
 *
 * @param i_Handle
 * @param i_Id
 * @param i_Length
 * @param i_Data
 * @return < 0 on failure
 * @return >= 0 on Success
 */
s8 FlashWrite(char const *const i_Handle, u16 i_Id, u16 i_Length, u8 const *const i_Data)
{
	s8 RetCode = 0;

	TRACE((1, "FlashWrite %d", i_Length));

	RetCode = adl_flhWrite((char *)i_Handle, i_Id, i_Length, (u8 *)i_Data);
	//wm_sprintf(g_traceBuf, "\r\nRetCode from adl_flhWrite: %d\r\n", RetCode);
	//DumpMessage(g_traceBuf);
	if (RetCode >= 0)
	{
		TRACE((1, "Successfully wrote to the flash"));
	}
	else
	{
		DisplayErrorCode("adl_flhWrite", __FILE__, __LINE__ - 8, RetCode);
	}

	return RetCode;
}


/***************************************************************************/
/*  Function   : FlashErase                                                */
/*-------------------------------------------------------------------------*/
/*  Object     : Calls the adl_flhErase on the provided handle and ID,     */
/*				 if i_id is set to ADL_FLH_ALL_IDS, then all IDs associated*/
/*				 with the handle will be erased							   */
/*                                                                         */
/*-------------------------------------------------------------------------*/
/*  Variable Name     |IN |OUT|GLB|  Utilisation                           */
/*--------------------+---+---+---+----------------------------------------*/
/*  i_handle          | X |   |   | The ascii string of the handle         */
/*--------------------+---+---+---+----------------------------------------*/
/*  i_Id		      | X |   |   | The ID to read					       */
/*--------------------+---+---+---+----------------------------------------*/
/*	RetCode           |   | X |   | Return code from the subscribe command */
/***************************************************************************/

/** @brief Calls the adl_flhErase on the provided handle and ID
 *
 * @par description Calls the adl_flhErase on the provided handle and ID, if i_id is set to ADL_FLH_ALL_IDS, then all IDs associated with the handle will be erased
 *
 * @param i_Handle
 * @param i_id
 * @return < 0 on failure
 * @return >= 0 on Success
 */
s8 FlashErase(char const *const i_Handle, u16 i_id)
{
	s8 RetCode = -1;
	TRACE((1, "FlashErase"));

	RetCode = adl_flhErase((char *)i_Handle, i_id);
	if (RetCode < 0)
	{
		DisplayErrorCode("adl_flhExist", __FILE__, __LINE__ - 3, RetCode);
	}
	else
	{
		TRACE((1, "Successfully erased flash id: %d", i_id));
	}
	return RetCode;
}


/**@}*/
