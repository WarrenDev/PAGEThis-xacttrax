/******************************************************************************
 *
 *  wyhook_Application.h    WyHook Application Commands.
 *
 *
 ******************************************************************************
 *
 *	This file is not portable and should be used to provide project specific
 *	functionality.
 *
 ******************************************************************************
 *
 *  MODIFICATION HISTORY
 * -----------------------------------------------------------------------------
 *  Flag | Date       | Author      | Revision | Description
 * ------+------------+-------------+----------+--------------------------------
 *       | 18Aug2009  | CMurphy     |  0.1     | Creation
 * ------+------------+-------------+----------+--------------------------------
 *
 * *****************************************************************************/
#ifndef _WYHOOK_APPLICATION_H_
#define _WYHOOK_APPLICATION_H_

#include "wyhook_Message.h"

//Required Functions:

void wyhook_ProcessSubTypeApplication_Application(WYHOOK_MESSAGE_S *msg);

//Application_Commands

typedef enum tagWYHOOK_APPLICATION_APPLICATION_COMMANDS
{
	COMMAND_APPLICATION_XACT_UploadConfig = 0x00,
	COMMAND_APPLICATION_XACT_DownloadConfig = 0x01,
	COMMAND_APPLICATION_XACT_UploadFirmware = 0x02,
	COMMAND_APPLICATION_XACT_DownloadWaypoints = 0x03,
	COMMAND_APPLICATION_XACT_DisableWyhook = 0x04,
	COMMAND_APPLICATION_XACT_AssistGPS = 0x05
} WYHOOK_APPLICATION_APPLICATION_COMMANDS_T;

#endif
