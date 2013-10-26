/** this code handles over the air at command parsing and response */
// the following commands are supported

// command              response                       example
// ***************************************************************
// at+cfun              none
// at&diagnose (all)
// at+cgsn              imei                        012345678901234
// at+cimi              imsi                        310260760765891
// at+ccid              SIM ID                      +CCID: "8901260761207658918"
// at+cgmr              modem rev and software      R73a00gg.WMP100 2094168 032009 12:40
// at+wmsn              serial number               Serial Number 749430026515201
// at+csq               modem status                +CSQ: 25,0
// at+cgmm              band capability             MULTIBAND  G850  1900
// at&version           app version                 firmware version: 2.05
// at&nmea=on           160 bytes of nmea strings
// at+wimei?            IMEI                        +WIMEI: 012345678901234
// at&xact=27           device config split into 160 byte packets
// at&xact=56           none
// at&xact=57           calibration data split into 160 byte packets

#include "adl_global.h"
#include "XactUtilities.h"
#include "diagnose.h"
#include "ConfigSettings.h"
#include "Accelerometer.h"
#include "GPSInterface.h"
#include "Accelerometer.h"
#include "GPSCtrl.h"
#include "PowerCtrl.h"
#include "ConfigSettings.h"
#include "ota_at.h"
#include "Timers.h"
#include "internalflash.h"

extern ACCEL_ERROR_STATE accel_error_state;
extern ascii SMS_PHN_NUM[];
extern s8 g_smsHandle;

typedef enum
{
	//OTA_CMD_CFUN,           // done
	OTA_CMD_DIAG,           // test - maybe some more
	//OTA_CMD_CGSN,           // done
	//OTA_CMD_CIMI,           // done
	//OTA_CMD_CCID,           // done
	//OTA_CMD_CGMR,           // done
	//OTA_CMD_WMSN,           // done
	//OTA_CMD_CSQ,            // done
	//OTA_CMD_CGMM,           // done
	OTA_CMD_VERSION,        // done
	OTA_CMD_NMEA,
	//OTA_CMD_WIMEI,          // done
	OTA_CMD_XACT_8,			
	OTA_CMD_XACT_27,        // done
	OTA_CMD_XACT_38,
	OTA_CMD_XACT_54,
	OTA_CMD_XACT_56,        //done
	OTA_CMD_XACT_57,        // test
	OTA_CMD_XACT_59,
	OAT_AT_CMD,
	NUM_OTA_CMDS
} AT_COMMAND_t;

typedef struct
{
	AT_COMMAND_t command_id;
	char            *command;
} OTA_AT_CMDS_s;

static OTA_AT_CMDS_s ota_cmds[] =
{
	//{ OTA_CMD_CFUN, "AT+CFUN=1" },
	{ OTA_CMD_DIAG, "AT&DIAGNOSE=" },
	//{ OTA_CMD_CGSN, "AT+CGSN" },
	//{ OTA_CMD_CIMI, "AT+CIMI" },
	//{ OTA_CMD_CCID, "AT+CCID" },
	//{ OTA_CMD_CGMR, "AT+CGMR" },
	//{ OTA_CMD_WMSN, "AT+WMSN" },
	//{ OTA_CMD_CSQ, "AT+CSQ" },
	//{ OTA_CMD_CGMM, "AT+CGMM" },
	{ OTA_CMD_VERSION, "AT&VERSION" },
	{ OTA_CMD_NMEA, "AT&NMEA=on" },
	//{ OTA_CMD_WIMEI, "AT+WIMEI?" },
	{ OTA_CMD_XACT_8, "AT&XACT=8" },
	{ OTA_CMD_XACT_27, "AT&XACT=27" },
	{ OTA_CMD_XACT_38, "AT&XACT=38" },
	{ OTA_CMD_XACT_54, "AT&XACT=54" },
	{ OTA_CMD_XACT_56, "AT&XACT=56" },
	{ OTA_CMD_XACT_57, "AT&XACT=57" },
	{ OTA_CMD_XACT_59, "AT&XACT=59" },
	{ OAT_AT_CMD, "AT+" }
};

typedef struct
{
	char Buf[160];
	UINT8 valid;
} OTA_Tx_Buf;

#define DIAG_Q_SIZE  20
OTA_Tx_Buf OTA_Buffer[DIAG_Q_SIZE];
UINT8 g_Pos;


#define RESP_BUFFER_LENGTH    (150)   //150 reponse data + 6 byte header, makes it total of 160 bytes

static bool inprogress = false;
static s32 time[NUM_DIAG_COMMANDS];  //time in seconds


static void set_OTA_AT_inProgress(bool ip)
{
	inprogress = ip;
}


// check if a string matches the string for the current command.
// return true if it does false if it doesn't
static bool check_command(char *cmd_string, OTA_AT_CMDS_s cmd_struct)
{
	//DumpMessage("check_command\r\n");
	if (strncmp(cmd_string, cmd_struct.command, strlen(cmd_struct.command)) == 0)
	{
		return true;
	}
	else if (strncmp(cmd_string, cmd_struct.command, 3) == 0)     //Just compare first 3 characters. to check if its "AT+"
	{
		return true;
	}
	else
	{
		return false;
	}
}


static AT_COMMAND_t find_command_id(char *cmd)
{
	AT_COMMAND_t cur_cmd = 0;

	//DumpMessage("find_command_id\r\n");
	
	// find which command we are dealing with.
	while ((cur_cmd < NUM_OTA_CMDS) && (!check_command(cmd, ota_cmds[cur_cmd])))
	{
		cur_cmd++;
	}

	return cur_cmd;
}


void send_ota_response(char *resp_string)
{
	int ii;
	char TxBuff[160];
	UINT8 len=0;
	//s8 Ret;
	//UINT8 RetryCount=0;
	// first to make sure all the characters are valid. Replace with space ' ' if
	// not valid. Make sure a NULL is in byte 161
	len = strlen(resp_string);
	wm_sprintf(g_traceBuf, "len = %d\r\n", len);
	DumpMessage(g_traceBuf);
	
	memset(TxBuff, '\0', 160);
	
	//resp_string[len] = 0;

	
	if ( len > RESP_BUFFER_LENGTH)
	{
		*(resp_string + (RESP_BUFFER_LENGTH-1)) = 0x00;
		len = RESP_BUFFER_LENGTH;
	}

	for (ii = 0; ii < len; ii++)
	{
		if ((resp_string[ii] < 0x20) || (resp_string[ii] > 0x7e))
		{
			resp_string[ii] = ' ';
		}
	}

	//Send in ..D format.
	TxBuff[0] = '.';
	TxBuff[1] = '.';
	TxBuff[2] = 'P';

	//DumpMessage("Before wm_itohexa\r\n");
	wm_itohexa(&TxBuff[3], len, 3);

	//wm_sprintf(g_traceBuf, "TxBuff[3]= 0x%x, TxBuff[4]= 0x%x, TxBuff[5]= 0x%x\r\n",TxBuff[3],TxBuff[4],TxBuff[5]);
	//DumpMessage(g_traceBuf);
	
	memcpy(&TxBuff[6],resp_string,len);

	DumpMessage("Send OTA RESPONSE\r\n");
	//DumpMessage(resp_string);


	QueSMS(TxBuff);

	//adl_smsSend(g_smsHandle, "+919902000561", (char *)TxBuff, ADL_SMS_MODE_TEXT);
	DumpMessage("\r\n---------------------\r\n");

}


static char resp_buffer[RESP_BUFFER_LENGTH + 1];

/* stuff the resp buffer with the at command response */
void OTAAT_handle_at_cmd_response(char *response_string, bool isTerminal, size_t response_string_length)
{
	static int resp_buffer_offset = 0;
	// these two basically represent the same data but makes the code cleaner.
	int length_remaining = response_string_length, resp_string_offset = 0;
	int num_bytes_to_copy = 0;

	while (length_remaining)
	{
		// if we have looped through and filled up the buffer
		// and there is data in the current reponse_string still, send it out.
		if (resp_buffer_offset == RESP_BUFFER_LENGTH - 1)
		{
			DumpMessage("***Length remaining but buffer full\r\n");
			send_ota_response(resp_buffer);
			resp_buffer_offset = 0;
			// clear out the response buffer.
			memset(resp_buffer, ' ', RESP_BUFFER_LENGTH);
		}
		// check to see if we will overflow the buffer.
		if (resp_buffer_offset + length_remaining >= RESP_BUFFER_LENGTH)
		{
			num_bytes_to_copy = RESP_BUFFER_LENGTH - resp_buffer_offset - 1;
		}
		else
		{
			num_bytes_to_copy = length_remaining;
		}

		// copy the proper number of bytes into the response buffer.
		memcpy(resp_buffer + resp_buffer_offset, response_string + resp_string_offset,
		       num_bytes_to_copy);

		// adjust length remaining for the current response.
		length_remaining -= num_bytes_to_copy;
		// adjust the offset within the currecnt response string.
		resp_string_offset += num_bytes_to_copy;
		// adjust the offset within the over the air response buffer.
		resp_buffer_offset += num_bytes_to_copy;
	}

	// If this is the last response from this particular at command or
	// if the buffer is filled up, send out response ota.
	if (isTerminal || (resp_buffer_offset == RESP_BUFFER_LENGTH - 1))
	{
		send_ota_response(resp_buffer);
		resp_buffer_offset = 0;
	}

	// if this is the terminal resposne, clear the in progress flag.
	if (isTerminal)
	{
		set_OTA_AT_inProgress(false);
	}
}


static bool ota_at_handler(adl_atResponse_t *paras)
{
	wm_sprintf(g_traceBuf, "OTA AT HANDLER: IsTerminal: %d\r\n", paras->IsTerminal);
	OTAAT_handle_at_cmd_response(paras->StrData, paras->IsTerminal, paras->StrLength);

	return true;
}


void OTAAT_eval(char *cmd)
{
	// clear out the response buffer.
	memset(resp_buffer, ' ', RESP_BUFFER_LENGTH);

	wm_sprintf(g_traceBuf, "IN OTAAT_eval: CMD : %s\r\n", cmd);
	DumpMessage(g_traceBuf);

	// find out which command we want to execute.
	AT_COMMAND_t ota_at_cmd = find_command_id(cmd);

	// if the command is not found there is nothing to do.
	if (ota_at_cmd == NUM_OTA_CMDS)
	{
		send_ota_response("OTA AT COMMAND ERROR");
		return;
	}

	wm_sprintf(g_traceBuf, "** OTA AT CMD = %d\r\n", ota_at_cmd);
	DumpMessage(g_traceBuf);

	// evaluate the command
	switch (ota_at_cmd)
	{
	//case OTA_CMD_CFUN:
		//adl_atCmdCreate(ota_cmds[ota_at_cmd].command, FALSE, NULL, NULL);
		//break;

	//case OTA_CMD_CGSN:
	//case OTA_CMD_CIMI:
	//case OTA_CMD_CCID:
	//case OTA_CMD_CGMR:
	//case OTA_CMD_WMSN:
	//case OTA_CMD_CSQ:
	//case OTA_CMD_CGMM:
	//case OTA_CMD_WIMEI:
	case OAT_AT_CMD:
		set_OTA_AT_inProgress(true);
		DumpMessage(ota_cmds[ota_at_cmd].command);
		DumpMessage("\r\n");
		//adl_atCmdCreate(ota_cmds[ota_at_cmd].command, FALSE, (adl_atRspHandler_t)ota_at_handler, "*", NULL);
		adl_atCmdCreate(cmd, FALSE, (adl_atRspHandler_t)ota_at_handler, "*", NULL);
		break;

	case OTA_CMD_VERSION:
		wm_sprintf(resp_buffer, "firmware version: %u.%02u\r\n", g_config.FirmwareRevNum >> 4, g_config.FirmwareRevNum & 0xf);
		send_ota_response(resp_buffer);
		break;

	case OTA_CMD_DIAG:
		diagnose_handler_ota(strchr(cmd, '=') + 1);
		break;
	case OTA_CMD_XACT_8:
		break;
	case OTA_CMD_XACT_27:
		ConfigSettings_setOTA27(true);
		DisplayConfig();
		break;
	case OTA_CMD_XACT_38:
		ConfigSettings_setOTA38(true);
		DisplayGPSStates();
		DisplayPowerState();
		ConfigSettings_setOTA38(false);
		break;
	case OTA_CMD_XACT_54:
		{
			NMEA_ERRORS nm;
			GetNMEAError(&nm);
			wm_sprintf(resp_buffer, "RMC error: %d\r\nGSV error: %d\r\nGGA error: %d\r\n",
				           nm.RMCErrors, nm.GSVErrors, nm.GGAErrors);
			DumpMessage(resp_buffer);
			send_ota_response(resp_buffer);
		}
		break;
	case OTA_CMD_XACT_56:
		AccelCalibrate();
		wm_sprintf(resp_buffer, "Accel Caliberate Successful\r\n");
		DumpMessage(resp_buffer);
		send_ota_response(resp_buffer);
		break;
	case OTA_CMD_XACT_57:
		ConfigSettings_setOTA57(true);
		AccelReadData();
		break;
	case OTA_CMD_XACT_59:	
		wm_sprintf(resp_buffer, "Bad Threshold: %d\r\nIRQ High Slow Idle: %d\r\n", accel_error_state.threshold_bad, accel_error_state.irq_high_slow_idle);
		DumpMessage(resp_buffer);
		send_ota_response(resp_buffer);
		break;
	//case OAT_AT_CMD:
		//DumpMessage("Generic AT Command\r\n");	
		//set_OTA_AT_inProgress(true);
		//DumpMessage(ota_cmds[ota_at_cmd].command);
		//DumpMessage("\r\n");
		//adl_atCmdCreate(cmd, FALSE, (adl_atRspHandler_t)ota_at_handler, "*", NULL);
		
		//break;
	default:
		break;
	}
}


void QueSMS(char* TxBuff)
{
	wm_sprintf(g_traceBuf, "\r\n strlen(TxBuff) : %ld\r\n", strlen(TxBuff));
	DumpMessage(g_traceBuf);

	strcpy(OTA_Buffer[g_Pos].Buf, TxBuff);
	OTA_Buffer[g_Pos].valid = TRUE;

	wm_sprintf(g_traceBuf, "\r\n g_Pos : %d\r\n",g_Pos);
	DumpMessage(g_traceBuf);

	wm_sprintf(g_traceBuf, "OTA_Buffer[%d].Buf : %s\r\n",g_Pos, OTA_Buffer[g_Pos].Buf);
	DumpMessage(g_traceBuf);	

	g_Pos++;
	g_Pos %= DIAG_Q_SIZE;

	startDiagMsgHandlerTimer();


	
}
void SendOTAResponse(u8 timerID, void *Context)
{
	(void)timerID;
	(void)Context;

	BOOL SendSMS=FALSE;
	UINT8 i=0;
	
	for(i=0; i<DIAG_Q_SIZE;i++)
	{
		if(OTA_Buffer[i].valid == TRUE)
		{
			SendSMS=TRUE;
			break;
		}
	}
	
	if(SendSMS)
	{
		wm_sprintf(g_traceBuf, "Tx SMS i : %d\r\n",i);
		DumpMessage(g_traceBuf);

		s8 Ret = adl_smsSend(g_smsHandle, SMS_PHN_NUM, (char *)OTA_Buffer[i].Buf, ADL_SMS_MODE_TEXT);
		if(Ret == OK)
		{
			OTA_Buffer[i].valid = FALSE;
		}

		//startDiagMsgHandlerTimer();
	}
	else
	{
		DumpMessage("Queue is empty, stop the timer\r\n");
		stopDiagMsgHandlerTimer();
		
	}
		
	

}

void test_otaat_tmr(u8 timerID, void *Context)
{
	(void)timerID;
	(void)Context;

	static int last_val = 0;

	wm_sprintf(g_traceBuf, "TEST OTAAT %d\r\n", last_val);
	DumpMessage(g_traceBuf);

	//if (last_val != OTA_CMD_CFUN)     // skip the reset command
	//{
		//OTAAT_eval(ota_cmds[last_val].command);
	//}

	last_val++;
	if (last_val == NUM_OTA_CMDS)
	{
		last_val = 0;
	}
	else
	{
		adl_tmrSubscribe(FALSE, 20, ADL_TMR_TYPE_100MS, (adl_tmrHandler_t)(test_otaat_tmr));
	}
}


//Hanlder runs every 1 minute
void DConfig_Rpt_handler(u8 timerID, void *Context)
{
	(void)timerID;
	(void)Context;

	UINT8 i=0;
	for(i=0; i< NUM_DIAG_COMMANDS; i++)
	{
		time[i] += 60;		
		if(g_DiagConfig[i].ATCmdRepeatCount && time[i] >= g_DiagConfig[i].ATCmdRepeatInterval)
		{
			//Send out diag pkt
			OTAAT_eval(g_DiagConfig[i].ATCommand);
			g_DiagConfig[i].ATCmdRepeatCount--;
			//restart the timer
			time[i] = 0;
			//Invalidate the command if repeat count is 0
			if(g_DiagConfig[i].ATCmdRepeatCount == 0)
			{
				g_DiagConfig[i].IsCommandValid = FALSE;
			}
			//Update flash
			DiagConfigToFlash(i);
		}
	}
}

void ResetDiagTxTimerCount(UINT8 Index)
{
	time[Index] = 0;
}
