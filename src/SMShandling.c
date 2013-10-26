/** @addtogroup NetworkComm
 *@{*/

/***************************************************************************
 * @file SMShandling.c
 *****-------------------------------------------------------------------------
 *
 ****************************************************************************/

/***************************************************************************
 *  Includes
 ****************************************************************************/
#include <adl_global.h>
#include "string.h"
#include "SMShandling.h"
#include "XactUtilities.h"
#include "protocol.h"
#include "FlashTest.h"
#include "WaypointControl.h"
#include "status.h"
#include "ConfigSettings.h"
#include "GPRS.h"
#include "PowerCtrl.h"
#include "GPSCtrl.h"
#include "InternalFlash.h"
#include "gpiotest.h"
#include "Accelerometer.h"
#include "Timers.h"
#include "adc.h"

/******************************************************************************
 *  Defines
 *****************************************************************************/
#define RX_BUFFSIZE                 (256)
#define NUM_SERVER_FR               (3)
#define STD_PHNNUM_LENGTH           (10)
#define CRITICAL_RETRY_INTERVAL     60 /* the number of seconds to wait between critical retry attemps */

//#define DISABLE_SEND (1)	//Disable to send sms

/*****************************************************************************
 *   Globals
 ******************************************************************************/
s8 g_smsHandle;                          //Handle for the sms
UINT8 g_sms_tx[SMS_TX_BUFFSIZE];
UINT8 g_sms_tx_critical[SMS_TX_BUFFSIZE];                       // buffer to store the critical sos packet
BOOL WaitOn = FALSE;
WAYPOINT waypoint_data;
WAYPOINT first_waypoint_data;
static int waypoint_pkt_offset = -1;                     //keep track of offset into waypoint flash
int current_waypoint = 0;
int waypointPktNum = 0;
int g_NeedSOSAck = 0;
u8 gx_rx[RX_BUFFSIZE];                          //Buffer to store data
ascii SMS_PHN_NUM[15]; 						//Phone number of the incoming SMS.
// the status packet we want to transmit.
STATUS g_tx_status;


extern SMS_WAYPOINT_PAYLOAD sms_waypoint_data;
extern UINT8 g_rx[RX_BUFFSIZE];
extern u8 PREV_Mode;
extern ascii g_IMEI[];
extern BOOL NetworkError;
extern BOOL GPSWaitTimeout;
//extern BOOL g_SengEngineOff;
extern int ForceRoamMode;

extern volatile int g_atmel_schedule;

/***************************************************************************
 *  Prototypes
 ****************************************************************************/
extern void SimulateAccelTrigger();

void sms_service_stop(void);
void SMS_ctrl_Handler(u8 Event, u16 Nb);
bool SMS_Handler(ascii *SmsTel, ascii *SmsTimeOrLength, ascii *SmsText);
void ClearSMSHandler(u8 timerid, void *context);
void makeAllZeroWaypoint(WAYPOINT *waypoint0);

static bool CREGHandler(adl_atResponse_t *paras);

typedef enum
{
	REG_NOT = 0,
	REG_HOME,
	REG_NOT_SEARCH,
	REG_DENY,
	REG_UNKNOWN,
	REG_ROAMING
} REG_STATUS;

static REG_STATUS registration_status;

typedef struct
{
	BOOL startup_tracking_mode;
	unsigned int duration_remaining;
} TRACKING_INTERVAL_STATUS;

TRACKING_INTERVAL_STATUS tracking_interval_status;

// Check if we should create a log packet.
static bool CheckCreateLog(void)
{
	// Always create log packets for 'S' packets.
	if (g_sms_tx[PKT_TYPE] == 'S')
	{
		return TRUE;
	}
	// Create a log for a C packet if the server requested ACK.
	if ((g_sms_tx[PKT_TYPE] == 'C') && get_LastConfigSendAck())
	{
		return TRUE;
	}

	return FALSE;
}


/** @brief SMS Callback Method
 *
 * @par Reads commands from the command parser to test SMS messaging
 *
 * @note <b>REMOVE</b> this method is not called!!
 * @param Cmd
 * @return void
 */
void CmdHandlerSMS(adl_atCmdPreParser_t *Cmd)
{
	//s8 sReturn = 0;
	DumpMessage(" \r\n enter CmdHandlerSMS \r\n ");
	switch (Cmd->Type)
	{
	case ADL_CMD_TYPE_TEST:
	case ADL_CMD_TYPE_READ:
	case ADL_CMD_TYPE_ACT:
		DumpMessage("\r\n Syntax:\r\n");
		DumpMessage("         AT&SMS=1        -- Send SMS message\r\n");
		DumpMessage("         AT&SMS=2        -- End SMS service\r\n");
		break;

	case ADL_CMD_TYPE_PARA:

		if (strncmp("1", ADL_GET_PARAM(Cmd, 0), 1) == 0)
		{
			//	sReturn = sms_send();
			break;
		}

		if (strncmp("2", ADL_GET_PARAM(Cmd, 0), 1) == 0)
		{
			sms_service_stop();
			break;
		}
	}
	adl_atSendResponse(ADL_AT_RSP, "\r\n OK \r\n");
}


/** @brief Start SMS Service
 *
 * @par
 * This method start SMS handling and directs all SMS events to the SMS_ctrl_Handler
 * method
 *
 * @note why is there a return code?
 * @return TRUE
 */
bool sms_service_start()
{
	DumpMessage("\r\n Inside sms_service_start \r\n");
	TRACE((1, "Inside sms_service_start"));

	// Subscribe to the SMS Service
	g_smsHandle = adl_smsSubscribe((adl_smsHdlr_f)SMS_Handler, (adl_smsCtrlHdlr_f)SMS_ctrl_Handler, ADL_SMS_MODE_TEXT);

	switch (g_smsHandle)
	{
	case ADL_RET_ERR_PARAM:
		TRACE((1, "** ERROR adl_smsSubscribe -- ADL_RET_ERR_PARAM **"));
		break;

	case ADL_RET_ERR_SERVICE_LOCKED:
		TRACE((1, "** ERROR adl_smsSubscribe -- ADL_RET_ERR_SERVICE_LOCKED **"));
		break;

	default:
		TRACE((1, "adl_smsSubscribe successful Handle:%d", g_smsHandle));
	}

	return TRUE;
}


/** @brief Stop SMS communiation
 *
 * @return void
 */
void sms_service_stop()
{
	s8 retval;

	TRACE((1, "Inside sms_service_stop"));

	// Unsubscribe the SMS Service
	retval = adl_smsUnsubscribe(g_smsHandle);

	switch (retval)
	{
	case ADL_RET_ERR_PARAM:
		TRACE((1, "** ERROR adl_smsUnsubscribe -- ADL_RET_ERR_PARAM"));
		break;

	case ADL_RET_ERR_SERVICE_LOCKED:
		TRACE((1, "** ERROR adl_smsUnsubscribe -- ADL_RET_ERR_SERVICE_LOCKED"));
		break;

	case ADL_RET_ERR_UNKNOWN_HDL:
		TRACE((1, "** ERROR adl_smsUnsubscribe -- ADL_RET_ERR_UNKNOWN_HDL"));
		break;

	case ADL_RET_ERR_NOT_SUBSCRIBED:
		TRACE((1, "** ERROR adl_smsUnsubscribe -- ADL_RET_ERR_NOT_SUBSCRIBED"));
		break;

	case ADL_RET_ERR_BAD_STATE:
		TRACE((1, "** ERROR adl_smsUnsubscribe -- ADL_RET_ERR_BAD_STATE"));
		break;

	default:
		TRACE((1, "adl_smsUnsubscribe completed %d", retval));
	}
}


/* @brief SMS Hander method
 *
 * @par
 * This method handles all events generated by the SMS service.
 *
 * @param Event
 * @param Nb
 * @return void
 */
void SMS_ctrl_Handler(u8 Event, u16 Nb)
{
	//	s8 RetCode = 0;
	//ascii *UartCmd = "AT+IPR=57600 ";
	TRACE((1, "*****Ctrl_handler is working******"));
	DumpMessage("Inside SMS_ctrl_Handler");

	switch (Event)
	{
	case ADL_SMS_EVENT_SENDING_OK:
		DumpMessage("\r\n ADL_SMS_EVENT_SENDING_OK sms \r\n");
		break;

	case ADL_SMS_EVENT_SENDING_ERROR:
		DumpMessage("\r\n ADL_SMS_EVENT_SENDING_ERROR sms \r\n");
		wm_sprintf(g_traceBuf, "Nb = %d \r\n", Nb);
		DumpMessage(g_traceBuf);

		DumpMessage("Error sending packet, creating log packet.\r\n");
		wm_sprintf(g_traceBuf, "Num log pkt to send = %d\r\n", g_status.NumLogPktToSend);
		DumpMessage(g_traceBuf);

		if (CheckCreateLog())
		{
			if (g_status.NumLogPktToSend < MAX_NUM_LOG_PKT)
			{
				LogPktToFlash(g_status.NumLogPktToSend);
				g_status.NumLogPktToSend++;
				if (InternalFlash_WriteNonVolatileStatus() == -1)
				{
					DumpMessage("Error writing status to flash\r\n");
				}
			}
		}
		break;

	case ADL_SMS_EVENT_SENDING_MR:
		DumpMessage("\r\n ADL_SMS_EVENT_SENDING_MR sms \r\n");
		wm_sprintf(g_traceBuf, "Nb = %x \r\n", Nb);
		DumpMessage(g_traceBuf);                //Display the contents of Nb
		break;
	}
}


/***************************************************************************/
/*  Function   : SMS_handler                                          */
/*-------------------------------------------------------------------------*/
/*  Objet      : SMS SMS Service Data Handler                    */
/*                                                                         */
/*  Return     : sms_tobe_fwd to or not to forward the SMS                 */
/*                                                                         */
/*-------------------------------------------------------------------------*/
/*  Variable Name     |IN |OUT|GLB|  Utilisation                           */
/*--------------------+---+---+---+----------------------------------------*/
/*  SmsTel            |   |   |   |  SMS Data                              */
/*--------------------+---+---+---+----------------------------------------*/
/*  SmsTimeOrLength   |   |   |   |  SMS Data                              */
/*--------------------+---+---+---+----------------------------------------*/
/*  SmsText           |   |   |   |  SMS Data                              */
/*--------------------+---+---+---+----------------------------------------*/
/***************************************************************************/

/** @brief SMS SMS Service Data Handler
 *
 * @param SmsTel
 * @param SmsTimeOrLength
 * @param SmsText
 * @return TRUE message received was valid.
 * @return FALSE message received was invalid.
 */
bool SMS_Handler(ascii *SmsTel, ascii *SmsTimeOrLength, ascii *SmsText)
{
	int i;                  //iterator for for loops
	char fromBuff[15];
	bool result = false;
	adl_tmr_t   *smsClrTmr;

	//sprintf(buffer,"%d %s %s\n",strlen(SmsText),SmsTimeOrLength,(char *)SmsText);
	//DumpMessage(buffer);
	wm_sprintf     ( g_traceBuf, "SMS message received: %s\n\r", SmsText ); // Debug output - AEW Jr.
	DumpMessageUSB ( g_traceBuf,1 );

	TRACE((2, " %s Length: %s Message: %s", SmsTimeOrLength, strlen(SmsText), SmsText));
	DumpMessage("\r\n *********Inside of SMS_Handler**********");

	/* Clear out the whole buffer so we know where the incoming message ends */
	memset(g_rx, '\0', RX_BUFFSIZE);

	strcpy((char*)SMS_PHN_NUM,(char*)SmsTel);
	wm_sprintf(g_traceBuf, "SMS phone number : %s\n\r", SMS_PHN_NUM);
	DumpMessage(g_traceBuf);	

	/* If the message was good */
	if (strstr(SmsText, "..") != 0)
	{
		TRACE((1, "Message was good"));

		i = 0;
		/* Skip over a leading '1' or '+' */
		while ((SmsTel[i] == '1') || (SmsTel[i] == '+'))
		{
			i++;
		}

		{
			unsigned int Buff_ndx = 0;              //Buffer index
			for (Buff_ndx = 0; Buff_ndx < STD_PHNNUM_LENGTH; Buff_ndx++)
			{
				fromBuff[Buff_ndx] = SmsTel[Buff_ndx + i];
			}
			fromBuff[10] = '\0';

			for (Buff_ndx = 0; Buff_ndx < strlen(SmsText); Buff_ndx++)
			{
				g_rx[Buff_ndx] = SmsText[Buff_ndx];
				//SMS_PHN_NUM[Buff_ndx] = SmsText[Buff_ndx];
			}
		}

		TRACE((1, "Going to evaluate packet"));
		eval_packet();

		smsClrTmr = adl_tmrSubscribe(FALSE, 100, ADL_TMR_TYPE_100MS, ClearSMSHandler);
		result = true;
	}
	else      // Message received, but not valid - delete it
	{
		TRACE((1, "Message received, but is not valid and is going to be deleted"));

		// Increment bad access counter
		if (g_status.BadAccessCnt >= 'Z')
		{
			g_status.BadAccessCnt = 'A';
		}
		else
		{
			g_status.BadAccessCnt++;
		}
	}

	return result;
}


void ClearSMSHandler(u8 timerid, void *context)
{
	(void)timerid;
	(void)context;

	s32 sReturn = -1;
	TRACE((1, "Clearing all received read messages"));
	sReturn = adl_atCmdCreate("AT+CMGD=1,1", FALSE, NULL, NULL);
	if (sReturn != OK)
	{
		DisplayErrorCode("adl_atCmdCreate", __FILE__, __LINE__, sReturn);
	}
}


/***************************************************************************/
/*  Function   : SMS_handler                                          */
/*-------------------------------------------------------------------------*/
/*  Objet      : SMS SMS Service Data Handler                    */
/*                                                                         */
/*  Return     : sms_tobe_fwd to or not to forward the SMS                 */
/*                                                                         */
/*-------------------------------------------------------------------------*/
/*  Variable Name     |IN |OUT|GLB|  Utilisation                           */
/*--------------------+---+---+---+----------------------------------------*/
/*  SmsTel            |   |   |   |  SMS Data                              */
/*--------------------+---+---+---+----------------------------------------*/
/*  SmsTimeOrLength   |   |   |   |  SMS Data                              */
/*--------------------+---+---+---+----------------------------------------*/
/*  SmsText           |   |   |   |  SMS Data                              */
/*--------------------+---+---+---+----------------------------------------*/
/***************************************************************************/

/** @brief SMS SMS Service Data Handler
 *
 * @param timerid
 * @param context
 * @return void
 */
void SMS_TX_Handler(u8 timerid, void *context)
{
	(void)timerid;
	(void)context;

	s32 sReturn = -1;
	int i = 0;
	static adl_rtcTimeStamp_t CriticalTimeStamp;
	static adl_rtcTimeStamp_t LogSendTimeStamp;
	adl_rtcTimeStamp_t CurrentTimeStamp;
	adl_rtcTimeStamp_t DeltaTimeStamp;
	UINT32 WaypointTimeInSec;
	UINT32 amountOfTimeRequested;
	int NumWaypointInFlash;
	static int prevNeedSOSAck = 0;
	static MODES PrevServerMode = NO_CHANGE_MODE;
	int ResendCritical = 0;
	int TrackingIntervalTrig = 0;
	int LatestTimeStamp = 0;
	static adl_rtcTimeStamp_t TrackingIntervalTimeStamp;
	static adl_rtcTimeStamp_t FirstTrackingIntervalTimeStamp;
	int waypoint_done = 0;
	TCP_STATUS status;
	static UINT8 prev_waypoint_tx_result = SMS_OK;
	static int first_log = 1;
	//DumpMessage("sms tx handler\n\r");
	unsigned int tracking_interval_duration;
	int TrackingIntervalSeconds=0;
	adl_atCmdCreate("at+creg?", FALSE, (adl_atRspHandler_t)CREGHandler, "*", NULL);

//Error handling : In case of network errors
	if(NetworkError == TRUE )
	{
		GetTCPStatus(&status);
		if(status == TCP_CONNECT)
		{
			DumpMessage("Last Message was not transmitted due to network error.. Retransmitting\r\n");
			NetworkError = FALSE;
			sms_send('X');
		}
		else
		{
			DumpMessage("NetworkError : TCP not connected back yet! \r\n");
		}
	}
//End - Error handling	


	// detect the server triggered tracking mode.
	if ((PrevServerMode != g_config.Mode) && (g_config.Mode == TRACK_MODE || g_config.Mode == FULL_TRACK_MODE 
		|| g_config.Mode == LP_FULL_TRACK_MODE || g_config.Mode == ALARM_MODE))
	{
		//TrackingIntervalTrig = 1;
		GetConvertTime(&FirstTrackingIntervalTimeStamp);
		PrevServerMode = g_config.Mode;
		//Config_SetMode(TRACK_MODE, g_config.TrackingModeDuration, NULL, TRUE);
		//Could be either TRACK_MODE or FULL_TRACK_MODE
		Config_SetMode(g_config.Mode, g_config.TrackingModeDuration, NULL, TRUE);
		DumpMessage("Mode changed. FirstTrackingIntervalTimeStamp calculated again\r\n");
	}
	else
	{
		// get the current time
		GetConvertTime(&CurrentTimeStamp);

		// time difference between first tracking interval timestamp and current time.
		if ((sReturn = adl_rtcDiffTime(&CurrentTimeStamp, &FirstTrackingIntervalTimeStamp, &DeltaTimeStamp)) < 0)
		{
			DisplayErrorCode("adl_rtcConvertTime", __FILE__, __LINE__, sReturn);
		}

		{
			if (tracking_interval_status.startup_tracking_mode)
			{
				tracking_interval_duration = tracking_interval_status.duration_remaining * 10;
			}
			else
			{
				tracking_interval_duration = g_config.TrackingModeDuration * 10;
			}

			if (tracking_interval_duration != 0)         // special case, track forever.
			{           // we are done tracking now.
				//	    wm_sprintf(g_traceBuf,"Delta time stamp = %u\n", (unsigned int)DeltaTimeStamp.TimeStamp);
				//	    DumpMessage(g_traceBuf);
				if ((DeltaTimeStamp.TimeStamp >= tracking_interval_duration) && (g_config.Mode == TRACK_MODE || g_config.Mode == FULL_TRACK_MODE
					|| g_config.Mode == LP_FULL_TRACK_MODE || g_config.Mode == ALARM_MODE))
				{
					// g_config.Mode = NO_CHANGE_MODE;
					Config_SetMode(NO_CHANGE_MODE, 0, NULL, TRUE);
					PrevServerMode = g_config.Mode;
					DumpMessage("Tracking interval done\r\n");
					tracking_interval_status.startup_tracking_mode = FALSE;
				}
			}

			//	    wm_sprintf(g_traceBuf, "duration = %d\r\n", tracking_interval_duration);
			//	    DumpMessage(g_traceBuf);
		}

		// time difference between last status transmit and current time.
		if ((sReturn = adl_rtcDiffTime(&CurrentTimeStamp, &TrackingIntervalTimeStamp, &DeltaTimeStamp)) < 0)
		{
			DisplayErrorCode("adl_rtcConvertTime", __FILE__, __LINE__, sReturn);
		}

        TrackingIntervalSeconds = GetTrackingInterval();
		//if ((DeltaTimeStamp.TimeStamp >= (u32)g_config.TrackingInterval * 60) && (g_config.Mode == TRACK_MODE))
		if ((TrackingIntervalSeconds != 0) && (DeltaTimeStamp.TimeStamp >= (u32)TrackingIntervalSeconds) && (g_config.Mode == TRACK_MODE || g_config.Mode == FULL_TRACK_MODE
			|| g_config.Mode == LP_FULL_TRACK_MODE || g_config.Mode == ALARM_MODE))
		{
			if (((g_config.TrackingModeDuration * 10 - DeltaTimeStamp.TimeStamp) > 0) &&
			    (g_config.TrackingModeDuration != 0))
			{
				//Config_SetMode(TRACK_MODE, (g_config.TrackingModeDuration * 10 - DeltaTimeStamp.TimeStamp) / 10, NULL, TRUE);
				//Could be either TRACK_MODE or FULL_TRACK_MODE
				Config_SetMode(g_config.Mode, (g_config.TrackingModeDuration * 10 - DeltaTimeStamp.TimeStamp) / 10, NULL, TRUE);
			}
			else
			{
				//Config_SetMode(TRACK_MODE, 0, NULL, TRUE);
				//Could be either TRACK_MODE or FULL_TRACK_MODE
				Config_SetMode(g_config.Mode, 0, NULL, TRUE);
			}

			//Send out tracking messages : If you are not in Track Mode OR not in slow idle mode (track mode).
			if(g_config.Mode != TRACK_MODE || GetSlowIdleMode() != 1)
			{
				TrackingIntervalTrig = 1;
				DumpMessage("Tracking Interval Trig\r\n");
			}
		}
	}

	// check for critical message confirmation.
	if ((prevNeedSOSAck == 0) && (g_NeedSOSAck == 1))
	{
		// get the current time
		GetConvertTime(&CriticalTimeStamp);
	}
	if (g_NeedSOSAck && prevNeedSOSAck)
	{
		// get the current time
		GetConvertTime(&CurrentTimeStamp);

		// time difference between first tracking interval timestamp and current time.
		if ((sReturn = adl_rtcDiffTime(&CurrentTimeStamp, &CriticalTimeStamp, &DeltaTimeStamp)) < 0)
		{
			DisplayErrorCode("adl_rtcConvertTime", __FILE__, __LINE__, sReturn);
		}

		if (DeltaTimeStamp.TimeStamp >= CRITICAL_RETRY_INTERVAL)
		{
			ResendCritical = 1;
			// get the current time
			GetConvertTime(&CriticalTimeStamp);
		}
	}

	prevNeedSOSAck = g_NeedSOSAck;

	GetTCPStatus(&status);

	// see if we need to resend the critical message
	if (ResendCritical)
	{
		DumpMessage("Resending critical message\r\n");
		sms_send('R');
	}
    else if (g_atmel_schedule == 1) {
//        ascii lstr[] ="TIMER\r\n";
//        extern u8 g_fcm_Handle;

        g_atmel_schedule = 0;
//        adl_fcmSendData(g_fcm_Handle, (unsigned char *)lstr, strlen(lstr));

        sms_send('A');
    }
	//check to see if there is any alarms going off. If so send a status packet
	else if ((g_status.SystemFailure != 'N') || (g_status.SOSAlarm != 'N') ||
	         (g_status.GPSAlarm != 'N') || (g_status.BattAlarm != 'N') ||
	         (g_status.GSMAlarm != 'N') || (g_status.FenceAlarm != 'N') ||
	         (g_status.MotionAlarm != 'N') ||
	         (g_status.PowerDisconnAlarm != 'N') || (g_status.OverSpeedAlarm != 'N') ||
	         (g_config.Mode == SERVER_STATUS_REQ) ||
	         (g_config.Mode == SERVER_STATUS_REQ_V) || TrackingIntervalTrig)
	{
		//For Debug
        //Sandeep - test code
        //Display GPS and power states
		//DisplayGPSStates();
        //DisplayPowerState();

// ########################################################
// ########################################################
// ########################################################

		//Check if GPS is availabe before packing the status info.
		//Dont wait for GPS, if there is a batt alarm to be reported
		if((g_status.BattAlarm == 'N') && (g_status.SOSAlarm == 'N') && g_status.GPSStatus == 'N')
		{
			if(WaitOn == FALSE)
			{
				//start GPS if its Off
				if(GetSlowIdleMode() == 1)
			    {
			        //Restart GPS.
					DumpMessage("Forced wake up Accel active!\r\n");
		 			SimulateAccelTrigger();
			    }

				//Ensure GPS is ON.
				gps_Start();			
				//Wait for a max 2 minutes to acquire GPS before sending out the status packet.
				StartGPSAvailabilityTimer();
				WaitOn = TRUE;
				goto SKIPTX;
			}
			else if (WaitOn == TRUE && GPSWaitTimeout != TRUE)
			{
				goto SKIPTX	;							
			}
			else if (GPSWaitTimeout)
			{
				//Wait time over, send out the message with available GPS data
				DumpMessage("GPS Wait time over, send out the message anyway!\r\n");
				WaitOn = FALSE;
			}
			
		}
		//Reset the WaitOn flag if decided to transmit the packet
		if(WaitOn != FALSE)
		{
			//Ensure timer is stopped once you are out of wait.
			StopGPSAvailabilityTimer();
		}

// ########################################################
// ########################################################
// ########################################################

		GetConvertTime(&TrackingIntervalTimeStamp);
		memcpy(&g_tx_status, &g_status, sizeof (STATUS));
        
		// return if we are waiting for a vibration to finish.
		if (!ClearAlarms())
		{
			return;
		}

		sms_send('S');

		if(g_config.Mode == ALARM_MODE)
		{
			//Modem will enter alarm mode after 30 sec. 
			//30 secs time is allowed for the current transmits to be completed.
			DumpMessage("StartPowerOffTimer \r\n");
			StartPowerOffTimer();	
		}

		//Trigger the timers for transmitting I packet.
		if(g_ModeConfig.TxPattern == 0x01)
		{
			StartAnalogSensorReadTimer();
			StartIPktTxTimer();	
		}		
		//End
		
		if ((g_config.Mode == SERVER_STATUS_REQ) || (g_config.Mode == SERVER_STATUS_REQ_V))
		{
			DumpMessage("Server triggered status packet!\r\n");
			//g_config.Mode = NO_CHANGE_MODE;
			g_config.Mode = PREV_Mode;
		}

		if (TrackingIntervalTrig)
		{
			TrackingIntervalTrig = 0;
			DumpMessage("Tracking interval triggered status!\r\n");
		}
SKIPTX:	

		DumpMessage("");
	
	}
	//check to see if there are log packets that need to be sent.
	//If so send a log packet
#ifndef DISABLE_LOG_PACKET
	else if ((g_status.NumLogPktToSend > 0) && (((status == TCP_CONNECT) && (PowerCtrl_GetTXMode() == GPRS)) || ((PowerCtrl_GetTXMode() == SMS) && (Status_getGSMStatus() == GSM_WORKING) && (GetSlowIdleMode() != 1))))
	{
		adl_rtcDiffTime(&CurrentTimeStamp, &LogSendTimeStamp, &DeltaTimeStamp);

		if ((DeltaTimeStamp.TimeStamp >= 10) || first_log)
		{
			wm_sprintf(g_traceBuf, "there are log packets to send: %d\r\n", g_status.NumLogPktToSend);
			DumpMessage(g_traceBuf);
			sms_send('L');
			first_log = 0;
			GetConvertTime(&LogSendTimeStamp);
		}
	}
#endif
	//check to see if there is a request for waypoint download.
	//if so send a waypoint packet
	else if (g_config.WaypointDLData != NO_DOWNLOAD)
	{
		DumpMessage("\n\rsend waypoint data\r\n");

		switch (g_config.WaypointDLData)
		{
		case HOUR1_DATA:
			amountOfTimeRequested = SECONDS_IN_HOUR;
			break;

		case HOUR8_DATA:
			amountOfTimeRequested = 8 * SECONDS_IN_HOUR;
			break;

		case HOUR24_DATA:
			amountOfTimeRequested = 24 * SECONDS_IN_HOUR;
			break;

		case HOUR72_DATA:
			amountOfTimeRequested = 72 * SECONDS_IN_HOUR;
			break;

		case WEEK1_DATA:
			amountOfTimeRequested = SECONDS_IN_WEEK;
			break;

		default:
			amountOfTimeRequested = 0;
		}

		NumWaypointInFlash = GetNumWaypoints();
		if (prev_waypoint_tx_result == SMS_OK)
		{
			waypointPktNum += 1;
		}
		sms_waypoint_data.packet_num = waypointPktNum;
		wm_sprintf(g_traceBuf, "new waypoint packet number: %d\r\n", waypointPktNum);
		DumpMessage(g_traceBuf);

		if (ReadWaypoint(&first_waypoint_data, -1) != 0)
		{
			wm_sprintf(g_traceBuf, "waypoint read from flash failed at offset: %d\r\n", -1);
			DumpMessage(g_traceBuf);
			return;
		}

		LatestTimeStamp = wm_hexatoi((ascii *)&(first_waypoint_data.utc[0]), 8);
		DisplayWaypoint(&first_waypoint_data);
		for (i = 0; i < WAYPOINTS_PER_PKT; i++)
		{
			if (ReadWaypoint(&waypoint_data, waypoint_pkt_offset) != 0)
			{
				wm_sprintf(g_traceBuf, "waypoint read from flash failed at offset: %d\r\n", waypoint_pkt_offset);
				DumpMessage(g_traceBuf);
				waypoint_done = 1;
				break;
			}
			waypoint_pkt_offset--;

			WaypointTimeInSec = wm_hexatoi((ascii *)&(waypoint_data.utc[0]), 8);

			wm_sprintf(g_traceBuf, "waypoint time in sec: %d LatestTime in sec: %d\r\n", (int )WaypointTimeInSec, (int )LatestTimeStamp);
			DumpMessage(g_traceBuf);

			wm_sprintf(g_traceBuf, "Late - Wpt: %d, amount req: %d\r\n", (int )(LatestTimeStamp - WaypointTimeInSec), (int )amountOfTimeRequested);
			DumpMessage(g_traceBuf);

			if (((LatestTimeStamp - WaypointTimeInSec) < amountOfTimeRequested) ||
			    ((g_config.WaypointDLData == ALL_DATA) & ((waypoint_pkt_offset * -1) < (NumWaypointInFlash))))
			{
				switch (i)
				{
				case 0:
					memcpy(&sms_waypoint_data.waypoint1, &waypoint_data, NUM_WAYPOINT_BYTES);
					break;

				case 1:
					memcpy(&sms_waypoint_data.waypoint2, (u8 *)&waypoint_data, NUM_WAYPOINT_BYTES);
					break;

				case 2:
					memcpy(&sms_waypoint_data.waypoint3, (u8 *)&waypoint_data, NUM_WAYPOINT_BYTES);
					break;

				case 3:
					memcpy(&sms_waypoint_data.waypoint4, (u8 *)&waypoint_data, NUM_WAYPOINT_BYTES);
				}
			}
			else
			{
				waypoint_done = 1;
				break;
			}
		}

		WAYPOINT waypoint_all_0;

		if (i < 4)
		{
			makeAllZeroWaypoint(&waypoint_all_0);
		}

		switch (i)
		{
		case 0:
			memcpy(&sms_waypoint_data.waypoint1, (u8 *)&waypoint_all_0, NUM_WAYPOINT_BYTES);
			memcpy(&sms_waypoint_data.waypoint2, (u8 *)&waypoint_all_0, NUM_WAYPOINT_BYTES);
			memcpy(&sms_waypoint_data.waypoint3, (u8 *)&waypoint_all_0, NUM_WAYPOINT_BYTES);
			memcpy(&sms_waypoint_data.waypoint4, (u8 *)&waypoint_all_0, NUM_WAYPOINT_BYTES);
			DumpMessage("setting 4 remainder waypoints to 0\r\n");
			break;

		case 1:
			memcpy(&sms_waypoint_data.waypoint2, (u8 *)&waypoint_all_0, NUM_WAYPOINT_BYTES);
			memcpy(&sms_waypoint_data.waypoint3, (u8 *)&waypoint_all_0, NUM_WAYPOINT_BYTES);
			memcpy(&sms_waypoint_data.waypoint4, (u8 *)&waypoint_all_0, NUM_WAYPOINT_BYTES);
			DumpMessage("setting 3 remainder waypoints to 0\r\n");
			break;

		case 2:
			memcpy(&sms_waypoint_data.waypoint3, (u8 *)&waypoint_all_0, NUM_WAYPOINT_BYTES);
			memcpy(&sms_waypoint_data.waypoint4, (u8 *)&waypoint_all_0, NUM_WAYPOINT_BYTES);
			DumpMessage("setting 2 remainder waypoints to 0\r\n");
			break;

		case 3:
			memcpy(&sms_waypoint_data.waypoint4, (u8 *)&waypoint_all_0, NUM_WAYPOINT_BYTES);
			DumpMessage("setting 1 remainder waypoints to 0\r\n");
			break;
		}

		// try to send the waypoint packet.
		// if we fail, reset the pointers and it will resend next time.
		sms_waypoint_data.packet_num = current_waypoint;
		if (sms_send('W') == SMS_OK)
		{
			prev_waypoint_tx_result = SMS_OK;
			current_waypoint += 1;
			if (waypoint_done)
			{
				g_config.WaypointDLData = NO_DOWNLOAD;
				current_waypoint = 0;
				waypoint_pkt_offset = -1;
			}
		}
		else
		{
			prev_waypoint_tx_result = SMS_ERROR;
			waypoint_pkt_offset += 4;
			waypoint_done = 0;
		}
	}
}


/** @brief Create a waypoint that contains all '0'.
 *
 * @param  waypoint0
 * @return void
 */
void makeAllZeroWaypoint(WAYPOINT *waypoint0)
{
	int i;
	for (i = 0; i < 8; i++)
	{
		waypoint0->utc[i] = '0';
		waypoint0->lat[i] = '0';
		waypoint0->longi[i] = '0';
		waypoint0->speed[i] = '0';

		if (i < 2)
		{
			waypoint0->epe[i] = '0';
		}
	}
}


/* dump message debug information */
static void display_tx_debug(char type)
{
	wm_sprintf(g_traceBuf, "\r\n Sending Message type: %c\r\n", (char)type);
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf,1);

	//dump send packet to UART for debug purposes
	wm_sprintf(g_traceBuf, "\r\nPacket sent: %s\r\n", (char *)(&g_sms_tx));
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf,1);
	// send it over USB as well
	//wm_sprintf(g_traceBuf, "%s\r\n", (char *)(&g_sms_tx));
	//adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_USB, g_traceBuf);

	wm_sprintf(g_traceBuf, "tx size = %d\r\n", (int )strlen((char *)g_sms_tx));
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf,1);
}


/* check if we are romaing and not supposed to transmit */
/* return true if we should not transmit */
static bool CheckRoamingNoTransmit(void)
{
	// check if we are roaming. If so and roaming is disabled in the configuration, do not
	// transmit and indicate a transmit failure.
	if (registration_status == REG_ROAMING)
	{
		if ((g_config.SMSorGPRS == SMS_NO_ROAMING) || (g_config.SMSorGPRS == GPRS_UDP_NO_ROAMING) ||
		    (g_config.SMSorGPRS == GPRS_TCP_NO_ROAMING) || (g_config.SMSorGPRS == GPRS_UDP_FALLBACK_NO_ROAMING) ||
		    (g_config.SMSorGPRS == GPRS_TCP_FALLBACK_NO_ROAMING))
		{
			DumpMessage("Roaming -- will not transmit!\r\n");
			return true;
		}
	}

	return false;
}


/***************************************************************************/

/** @brief SMS send function
 *
 * @param type
 * @return SMS_ERROR failed to send the message.
 * @return SMS_OK message transmitted succesfully.
 */
UINT8 sms_send(char type)

{
	s32 sReturn = -1;
	UINT8 rtn_status;
	int i;
	ascii phnum_adj[MAX_PHNUM_LEN];

	// construcut the new packet.
	// If we are sending an 'R' packet resend the critical.
	if (type != 'R' && type != 'X')
	{
		make_packet(type);
	}
	else if (type == 'R')
	{
		memcpy(g_sms_tx, g_sms_tx_critical, SMS_TX_BUFFSIZE);
	}
	else
	{
		//type = 'X' is transmitted only if the previous msg transfer failed due to some n/w error. In this case we
		//need to retry transmitting the previous msg as is.
		//Since g_sms_tx doesnt get updated until a new packet is sent out we can re-transmit the previous pkt as is 
		//Dont update any new info.. send as it is...
		DumpMessage("Transmitting the old packet as it is\r\n");
	}

	display_tx_debug(type);

	do
	{
		if (CheckRoamingNoTransmit())
		{
			sReturn = -1;
			break;
		}

		// GPRS Mode. Switched from looking at the actual config to looking
		// at this GetTXMode function. This will take into account any "fallback" action
		// that results from the GPRS connection failing.
		if (PowerCtrl_GetTXMode() == GPRS)
		{
			memcpy((u8 *)(&(g_sms_tx[160])), g_IMEI, 15);
			g_sms_tx[175] = 0x00;
			DumpMessage("Doing a GPRS TX\r\n");
			if (type == 'C')
			{
				sReturn = TCPTransmit(C_SIZE_GPRS);
			}
			else if ((type == 'R'))
			{
				memset(&g_sms_tx[STATUS_PKT_SIZE], 'z', 160 - STATUS_PKT_SIZE);
				sReturn = TCPTransmit(S_SIZE_GPRS);
			}
			else if ((type == 'S'))
			{
				memset(&g_sms_tx[STATUS_PKT_SIZE], 'z', 160 - STATUS_PKT_SIZE);
				sReturn = TCPTransmit(S_SIZE_GPRS);
			}			
			else if (type == 'L')
			{
				memset(&g_sms_tx[STATUS_PKT_SIZE], 'z', 160 - STATUS_PKT_SIZE);
				sReturn = TCPTransmit(L_SIZE_GPRS);
			}
			else if (type == 'W')
			{
				memset(&g_sms_tx[WAYPOINT_PKT_SIZE], 'z', 160 - WAYPOINT_PKT_SIZE);
				sReturn = TCPTransmit(W_SIZE_GPRS);
			}
			else if (type == 'M')
			{
				memset(&g_sms_tx[MODE_CONFIG_PKT_SIZE], 'z', 160 - MODE_CONFIG_PKT_SIZE);
				sReturn = TCPTransmit(M_SIZE_GPRS);
			}
			else if (type == 'I')
			{
				memset(&g_sms_tx[IO_STATUS_PKT_SIZE], 'z', 160 - IO_STATUS_PKT_SIZE);
				sReturn = TCPTransmit(I_SIZE_GPRS);				
            } 
			else if (type == 'A')						// Alert packet - AEW Jr.
			{
                memset(&g_sms_tx[TC_ALERT_PKT_SIZE], 'z',  160 - TC_ALERT_PKT_SIZE);   /*reserved bytes*/
                sReturn = TCPTransmit(A_SIZE_GPRS);
			} else
			{
				return -1;
			}

			wm_sprintf(g_traceBuf, "\r\nGPRS Packet sent: %s\r\n", (char *)(&g_sms_tx));
			DumpMessage(g_traceBuf);

			wm_sprintf(g_traceBuf, "IMEI=%s\r\n", g_IMEI);
			DumpMessage(g_traceBuf);

			wm_sprintf(g_traceBuf, "GPRS TX Done: sReturn = %d\r\n", (int )sReturn);
			DumpMessage(g_traceBuf);
		}
		else
		{
			if (g_smsHandle < 0)
			{
				g_smsHandle = adl_smsSubscribe((adl_smsHdlr_f)SMS_Handler, (adl_smsCtrlHdlr_f)SMS_ctrl_Handler, ADL_SMS_MODE_TEXT);
			}
			else
			{
				for (i = 0; i < MAX_PHNUM_LEN; i++)
				{
					if (g_config.ServerPhoneNum[i] != '0')
					{
						break;
					}
				}
				memcpy(phnum_adj, &(g_config.ServerPhoneNum[i]), MAX_PHNUM_LEN - i);
				phnum_adj[MAX_PHNUM_LEN - i] = 0x00;

				wm_sprintf(g_traceBuf, "phone number = %s\r\n", phnum_adj);
				DumpMessage(g_traceBuf);

				//Special case, as M packet has only MODE_CONFIG_PKT_SIZE of data. So fill rest with 'z'
				if (type == 'M')
				{
					memset(&g_sms_tx[MODE_CONFIG_PKT_SIZE], 'z', 160 - MODE_CONFIG_PKT_SIZE);					
				}
				else if(type == 'I')
				{
					memset(&g_sms_tx[IO_STATUS_PKT_SIZE], 'z', 160 - IO_STATUS_PKT_SIZE);
				}
				//End - M Packet
				
				g_sms_tx[160] = 0x00;          //make sure we only try to send 160 chars.
				sReturn = adl_smsSend(g_smsHandle, phnum_adj, (char *)(&g_sms_tx), ADL_SMS_MODE_TEXT);

				if (sReturn < 0)

				{
					//store the status packet as a log packet since failed
					//to send status packet
					DisplayErrorCode("adl_smsSend", __FILE__, __LINE__, sReturn);
				}
			}
		}
	} while (0);

	// check to see if this was a critical message type.
	if ((g_sms_tx[PKT_SOS_ALARM] != 'N') && (g_config.CriticalConfirm != 0) && (type == 'S'))
	{
		DumpMessage("Critical message! need to get an ack!\r\n");
		g_NeedSOSAck = 1;
		// save the message for re-transmission if needed.
		memcpy(g_sms_tx_critical, g_sms_tx, SMS_TX_BUFFSIZE);
	}

	// failed to send the message
	if (sReturn < 0)
	{
		rtn_status = SMS_ERROR;

		//store the status packet as a log packet since failed
		//to send status packet
		if (CheckCreateLog())
		{
			if (PowerCtrl_GetTXMode() == GPRS)
			{
				TCP_STATUS status;
				GetTCPStatus(&status);
				if (status != TCP_CONNECT)
				{
					StartGPRS();
				}
			}

			DumpMessage("Error sending packet, creating log packet.\r\n");
			wm_sprintf(g_traceBuf, "Num log pkt to send = %d\r\n", g_status.NumLogPktToSend);
			DumpMessage(g_traceBuf);
			if (g_status.NumLogPktToSend < MAX_NUM_LOG_PKT)
			{
				LogPktToFlash(g_status.NumLogPktToSend);
				g_status.NumLogPktToSend++;
				if (InternalFlash_WriteNonVolatileStatus() == -1)
				{
					DumpMessage("Error writing status to flash\r\n");
				}
			}
		}
	}
	else
	{
		rtn_status = SMS_OK;
		// succesfully sent a log packet.
		if ((g_sms_tx[PKT_TYPE] == 'L') || (g_sms_tx[PKT_TYPE] == 'K'))
		{
			DumpMessage("Succesfully sent log packet\r\n");
			g_status.NumLogPktToSend--;
			if (InternalFlash_WriteNonVolatileStatus() == -1)
			{
				DumpMessage("Error writing status to flash\r\n");
			}
		}
	}

	return rtn_status;
}


void SMSHandling_DisplayRegStatus(void)
{
	wm_sprintf(g_traceBuf, "Registration status = %d\r\n", registration_status);
	DumpMessage(g_traceBuf);
}


static bool CREGHandler(adl_atResponse_t *paras)
{
	int comma_cnt = 0;
	char roaming_status;

	if (ForceRoamMode)
	{
		registration_status = REG_ROAMING;
	}
	else
	{
		if (paras->RspID == ADL_STR_CREG)
		{
			//      DumpMessage("CREG response\r\n");
			//      wm_sprintf(g_traceBuf,"%s\r\n",paras->StrData);
			//      DumpMessage(g_traceBuf);

			unsigned int ii;
			roaming_status = '?';
			for (ii = 0; ii < strlen(paras->StrData); ii++)
			{
				if (paras->StrData[ii] == ',')
				{
					comma_cnt++;
				}
				if (comma_cnt == 2)
				{
					roaming_status = paras->StrData[ii - 1];
					break;
				}
			}

			if (roaming_status == '?')
			{
				registration_status = REG_UNKNOWN;
			}
			else if (roaming_status == '5')
			{
				registration_status = REG_ROAMING;
			}
			else
			{
				registration_status = REG_HOME;            // not really true, but we'll try to transmit anyway.
			}
		}
	}

	return FALSE;
}


void SetStartupTrackingMode(int duration_remaining, BOOL enable)
{
	if (enable)
	{
		tracking_interval_status.startup_tracking_mode = TRUE;
		tracking_interval_status.duration_remaining = duration_remaining;
	}
	else
	{
		tracking_interval_status.startup_tracking_mode = FALSE;
		tracking_interval_status.duration_remaining = 0;
	}
}
/*@}*/
