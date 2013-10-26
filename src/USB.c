/** @addtogroup LED
 *@{*/

/** @file USB.c
 */
#include <adl_global.h>
#include "USB.h"
#include "pistd.h"
#include "ConfigSettings.h"
#include "WaypointControl.h"
#include "DOTAOperation.h"
#include "XactUtilities.h"
#include "wyhook_Message.h"
#include "ConfigSettings.h"
#include "protocol.h"
#include "Timers.h"
#include "SimOperations.h"
#include "adc.h"
#include "Internalflash.h"


//Debugs enabled
bool g_DebugEnable=0;


static s8 g_USBCommFCMDataMode = 0;

struct commSocket_s g_USBCommStruct;

eFlowCmdState g_USBFlowCmdState = CLOSE_USB_STATE;

void USBInterfaceCmdHandler(adl_atCmdPreParser_t *Cmd);
s8 CommInit(commPort_type i_comm);
bool evhUSBCommFcmCtrlHandler(adl_fcmEvent_e Event);
bool evhUSBCommFcmDataHandler(u16 DataSize, u8 *Data);

s32 gUSBApplicationUpdateFlag = 0; /* always 0 in this code */
bool parseSLIPPacket(u16 DataSize, u8 *Data);

USB_MODE g_wyhook_mode = NORMAL_MODE;

// rx buffer.
#define RX_BUFFSIZE    (256)

extern UINT8 g_rx[RX_BUFFSIZE];
extern BOOL NetworkError;
extern s8 g_smsHandle; 
extern AGPS_DATA g_AGPSData;

/** @brief Clear USB load file
 *
 * @par  clear download flag
 *
 * @return void
 */
void clrUSBLoadFile()
{
	D_TRACE("USB control by python\n");
	g_USBCommStruct.DownloadFlag = 0;
	SEND_OK_USB();
}


/** @brief set USB Load File flag
 *
 * @par not sure why this is here
 * @return void
 */
void setUSBLoadFile()
{
	D_TRACE("USB control by USB app\n");
	g_USBCommStruct.DownloadFlag = 1;
}


/** @brief USB Cmd Handler
 *
 * @par This method will be called on events being raised from
 * the USB subcription
 *
 * @param Params
 * @return TRUE
 */
bool USBFlowCmdHandler(adl_atResponse_t *Params)
{
	if (Params->RspID == ADL_STR_OK)
	{
		switch (g_USBFlowCmdState)
		{
		case CLOSE_USB_STATE:
			g_USBFlowCmdState = OPEN_USB_STATE;
			break;

		case OPEN_USB_STATE:
			g_USBFlowCmdState = USB_FULL_INIT_STATE;
			CommInit(USB_COM_ENUM);
			break;

		default:
			TRACE((1, "ERROR"));
			break;
		}
	}
	return TRUE;
}


/** @brief USB Interface
 *
 * @par
 * Initialize the USB interface.
 * @return void
 */
void USBInterface(void)
{
	s8 retValue = 0;

	g_USBCommStruct.portType = USB_COM_ENUM;
	g_USBCommStruct.socketFlags = 0;
	g_USBCommStruct.FCM_Handle = -1;
	g_USBCommStruct.rcvDataSize = 0;
	g_USBCommStruct.DownloadFlag = 0;

	retValue = adl_atCmdCreate("AT+WMFM=0,0,3", ADL_AT_PORT_TYPE(ADL_PORT_NONE, FALSE), USBFlowCmdHandler, "*", NULL);
	if (retValue != OK)
	{
		DisplayErrorCode("adl_atCmdCreate", __FILE__, __LINE__, retValue);
	}

	retValue = adl_atCmdCreate("AT+WMFM=0,1,3", ADL_AT_PORT_TYPE(ADL_PORT_NONE, FALSE), USBFlowCmdHandler, "*", NULL);
	if (retValue != OK)
	{
		DisplayErrorCode("adl_atCmdCreate", __FILE__, __LINE__, retValue);
	}

	retValue = adl_atCmdSubscribe("AT&USBINTERFACE", USBInterfaceCmdHandler
	                              , ADL_CMD_TYPE_ACT | ADL_CMD_TYPE_TEST | ADL_CMD_TYPE_READ | ADL_CMD_TYPE_PARA | 0x00a1);
	if (retValue != OK)
	{
		DisplayErrorCode("adl_atCmdSubscribe", __FILE__, __LINE__, retValue);
	}

	DumpMessage("USB Interface Started\n");
	g_USBFlowCmdState = USB_FULL_INIT_STATE;
	CommInit(USB_COM_ENUM);

	adl_ctxSleep(10);
	ChangeV24State(USB_COM_ENUM, ADL_FCM_V24_STATE_DATA);
}


/** @brief USB communication inialization
 *
 * @param i_comm
 * @return g_USBCommStruct.FCM_Handle
 */
s8 CommInit(commPort_type i_comm)
{
	//FIXME: remove this parameter if we aren't using it -- pjn
	(void)i_comm;

	TRACE((1, "inside CommInit\n"));

	if (adl_fcmIsAvailable(ADL_FCM_FLOW_V24_USB) == TRUE)
	{
		TRACE((1, " gFileBufferOffset Is Available\n"));
		if (g_USBCommStruct.FCM_Handle < 0)
		{
			g_USBCommStruct.FCM_Handle = adl_fcmSubscribe(ADL_PORT_USB
			                                              , (adl_fcmCtrlHdlr_f)evhUSBCommFcmCtrlHandler
			                                              , (adl_fcmDataHdlr_f)evhUSBCommFcmDataHandler);
			if (g_USBCommStruct.FCM_Handle > -1)
			{
				// we have success
				// copy passed structure to our storage
				// location
				//			g_USBCommStruct = ptrSocketStruct;
				//			g_USBCommStruct.FCM_Handle = sFcmHandle;
			}
			switch (g_USBCommStruct.FCM_Handle)
			{
			case ADL_RET_ERR_PARAM:
				TRACE((1, "ADL_RET_ERR_PARAM\n"));
				break;

			case ADL_RET_ERR_ALREADY_SUBSCRIBED:
				TRACE((1, "ADL_RET_ERR_ALREADY_SUBSCRIBED\n"));
				break;

			case ADL_RET_ERR_NOT_SUBSCRIBED:
				TRACE((1, " ADL_RET_ERR_NOT_SUBSCRIBED\n"));
				break;

			case ADL_FCM_RET_ERROR_GSM_GPRS_ALREADY_OPENNED:
				TRACE((1, " ADL_FCM_RET_ERROR_GSM_GPRS_ALREADY_OPENED\n"));
				break;

			case ADL_RET_ERR_BAD_STATE:
				TRACE((1, " ADL_RET_ERR_BAD_STATE\n"));
				break;

			case ADL_RET_ERR_SERVICE_LOCKED:
				TRACE((1, " ADL_RET_ERR_SERVICE_LOCKED\n"));
				break;

			default:
				TRACE((1, " fcm started\n"));
			}
		}
		SEND_OK_USB();
	}
	else
	{
		TRACE((1, "Uart2Init Is NOT Available\n"));
		//TRACE("%s\n",gTraceBuf);
		adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_USB, "\nERROR\n");
	}
	return g_USBCommStruct.FCM_Handle;
}


/** @brief callback method called on AT&USBINTERFACE calls
 *
 * @par
 * This method is called on every AT command AT&USBINTERFACE
 *
 * @param Cmd
 * @return void
 */
void USBInterfaceCmdHandler(adl_atCmdPreParser_t *Cmd)
{
	u8 SendOkMsg = 1;
	DumpMessage("USB Interface Cmd Handler\r\n");
	adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_USB
	                       , "USB Interface Cmd Handler\r\n");

	switch (Cmd->Type)
	{
	case ADL_CMD_TYPE_TEST:
	case ADL_CMD_TYPE_ACT:
		TRACE((1, "AT&TST=? is invalid! \n"));
		break;

	case ADL_CMD_TYPE_READ:
		TRACE((1, "\n Syntax:\n"));
		TRACE((1, "         AT&USBINTERFACE=1                   -- init USB port\n"));
		TRACE((1, "         AT&USBINTERFACE=2                   -- send at cmd \n"));
		TRACE((1, "         AT&USBINTERFACE=3                   -- file size \n"));
		TRACE((1, "         AT&USBINTERFACE=4                   -- set to AT parse mode \n"));
		TRACE((1, "         AT&USBINTERFACE=5                   -- set to data parse mode \n"));
		TRACE((1, "         AT&USBINTERFACE=6                   -- setup DA file \n"));
		TRACE((1, "         AT&USBINTERFACE=7                   -- complete application install \n"));
		TRACE((1, "         AT&USBINTERFACE=8                   -- complete application install \n"));
		TRACE((1, "         AT&USBINTERFACE=9                   -- open filename \n"));
		TRACE((1, "         AT&USBINTERFACE=10                  -- format SPI Flash \n"));
		adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_USB
		                       , "<USB> AT&USBINTERFACE=10            -- format SPI Flash </usb>\r\n");
		TRACE((1, "         AT&USBINTERFACE=11                  -- dir listing \n"));
		adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_USB
		                       , "<USB> AT&USBINTERFACE=11            -- dir listing      </usb>\r\n");
		TRACE((1, "         AT&USBINTERFACE=12                  -- InitFS \n"));
		TRACE((1, "         AT&USBINTERFACE=13,\"dirname\"        -- mkdir \n"));
		TRACE((1, "         AT&USBINTERFACE=14                  -- close Python USB \n"));
		TRACE((1, "         AT&USBINTERFACE=15                  -- close ConnectWare TCP channel\n"));
		TRACE((1, "         AT&USBINTERFACE=16,\"filename\"       -- delete file from file system\n"));
		TRACE((1, "         AT&USBINTERFACE=20,\"<rci/>\"         -- Run an RCI command\n"));
		adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_USB
		                       , "<USB> AT&USBINTERFACE=20,\"<rci/>\" -- Run command line RCI command </USB>\r\n");
		adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_USB
		                       , "<USB> AT&USBINTERFACE=22             -- Enter long xml RCI command </USB>\r\n");

		TRACE((1, "         AT&USBINTERFACE=30,GPIO,Level       -- change state of LED \n"));
		adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_USB
		                       , "AT&USBINTERFACE=30,GPIO,Level       -- change state of LED \n");
		TRACE((1, "         AT&USBINTERFACE=100                 -- test Filesystem - seek_set\n"));
		TRACE((1, "         AT&USBINTERFACE=101                 -- test Filesystem - seek_end\n"));
		TRACE((1, "         AT&USBINTERFACE=102                 -- test Filesystem - seek_current\n"));
		TRACE((1, "         AT&USBINTERFACE=109                 -- abort (intentionally crash)\n"));
		TRACE((1, "         AT&USBINTERFACE=110                 -- load updatefw.dwl from filesystem \n"));
		adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_USB
		                       , "<USB> AT&USBINTERFACE=110           -- load updatefw.dwl from filesystem </usb>\r\n");
		break;

	case ADL_CMD_TYPE_PARA:
		
		if (strncmp("115", ADL_GET_PARAM(Cmd, 0), 3) == 0)
		{
			DumpMessage("USB=115\r\n");
			g_wyhook_mode = WYHOOK_MODE;
			break;
		}

		// this will dump the waypoints over USB
		if (strncmp("114", ADL_GET_PARAM(Cmd, 0), 3) == 0)
		{
			DumpMessage("USB=114\r\n");
			DumpWaypointsUSB();
			break;
		}

		// this will display the number of waypoints in flash.
		if (strncmp("113", ADL_GET_PARAM(Cmd, 0), 3) == 0)
		{
			DumpMessage("USB=113\r\n");
			DisplayNumWaypointsUSB();
			break;
		}

		// This will display the configuration over USB
		if (strncmp("112", ADL_GET_PARAM(Cmd, 0), 3) == 0)
		{
			DumpMessage("USB=112\r\n");
			DisplayConfig();
			break;
		}

		// This will load a configuration packet.
		if (strncmp("111", ADL_GET_PARAM(Cmd, 0), 3) == 0)
		{
			DumpMessage("USB=111\r\n");
			memcpy(g_rx, ADL_GET_PARAM(Cmd, 1), strlen(ADL_GET_PARAM(Cmd, 1)));
			eval_packet();
			if (g_config.ConfigPktRxProperly != TRUE)
			{
				SendOkMsg = 0;
			}
			break;
		}

		if (strncmp("20", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			SendOkMsg = 0;
			adl_simUnsubscribe(evhSimHandler);
			stopTimersFWUpdate();

			g_USBFlowCmdState = OPEN_USB_STATE;
			setUSBLoadFile();
			if (ADL_GET_PARAM(Cmd, 1))
			{
				recordFileSize(wm_atoi(ADL_GET_PARAM(Cmd, 1)));
			}
			else
			{
				DumpMessage("NULL filesize\r\n");
				break;
			}
			StartDOTA();
			break;
		}

		if (strncmp("17", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			DumpMessage("USB=17\r\n");
			adl_simUnsubscribe(evhSimHandler);
			stopTimersFWUpdate();
			setUSBLoadFile();
			g_USBFlowCmdState = OPEN_USB_STATE;
			SEND_OK_USB();
			SendOkMsg = 0;
			break;
		}

		if (strncmp("3", ADL_GET_PARAM(Cmd, 0), 1) == 0)
		{
			DumpMessage("USB=3\r\n");
			wm_sprintf(g_traceBuf, "USB set the filesize to:%d\n", wm_atoi(ADL_GET_PARAM(Cmd, 1)));
			DumpMessage(g_traceBuf);
			recordFileSize(wm_atoi(ADL_GET_PARAM(Cmd, 1)));
			SEND_OK_USB();
			SendOkMsg = 0;
			break;
		}

		if (strncmp("6", ADL_GET_PARAM(Cmd, 0), 1) == 0)
		{
			DumpMessage("USB=6\r\n");
			gUSBApplicationUpdateFlag = 0;
			StartDOTA();
			SendOkMsg = 0;
			break;
		}
		if (strncmp("5", ADL_GET_PARAM(Cmd, 0), 1) == 0)
		{
			DumpMessage("USB=5\r\n");
			D_TRACE("place USB into Data Rx Mode\n");
			ChangeV24State(USB_COM_ENUM, ADL_FCM_V24_STATE_DATA);
			SendOkMsg = 0;
			break;
		}
		if (strncmp("7", ADL_GET_PARAM(Cmd, 0), 1) == 0)
		{
			DumpMessage("USB=7\r\n");
			break;
		}

		if (strncmp("18", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			DumpMessage("USB=18\r\n");
			break;
		}

		if(strncmp("125", ADL_GET_PARAM(Cmd, 0), 3) == 0)
		{
			DumpMessage("Setting Network Error to TRUE\r\n");
			NetworkError = TRUE;
			break;
		}

		if(strncmp("126", ADL_GET_PARAM(Cmd, 0), 3) == 0)
		{
			EnterAlarmMode();			
			break;
		}
		
		if(strncmp("127", ADL_GET_PARAM(Cmd, 0), 3) == 0)
		{
			char TimeOut[8];
			DumpMessage("Set GPS wait timeout\r\n");
			strncpy(TimeOut,ADL_GET_PARAM(Cmd, 0), 7);
			g_ModeConfig.GPSTimeout = TimeOut[3] - 0x30;			
			ModeConfigToFlash();			

			g_AGPSData.PowerOffTimeout = TimeOut[4] - 0x30;						

			//Update AGPS timeout			
			TimeOut[7] = '\0';
			g_AGPSData.WaitTimeout = atoi(&TimeOut[5]);
			AGPSDataToFlash();
			
			wm_sprintf(g_traceBuf,"AGPSTimeOut = %d, g_GPSTimeout = %d, PowerOffTimeout : %d\r\n",g_AGPSData.WaitTimeout, g_ModeConfig.GPSTimeout, g_AGPSData.PowerOffTimeout);
			DumpMessage(g_traceBuf);
			DumpMessageUSB(g_traceBuf,1);

			
			break;
		}

		if(strncmp("128", ADL_GET_PARAM(Cmd, 0), 3) == 0)
		{
			//extern BOOL g_SengEngineOff;
			//Mimic engine on.. for testing
			extern adl_rtcTime_t EngineStartTime;		
			extern adl_rtcTime_t EngineStopTime;
			adl_rtcTimeStamp_t  CurrentTimeStamp;
			s32 sReturn=-1;
			
			// Time.
			// get the current RTC time.
			if ((sReturn = adl_rtcGetTime(&EngineStartTime)) < 0) 
			{
		    	DisplayErrorCode("adl_rtcGetTime",__FILE__,__LINE__,sReturn);
		    	return;
			}
		    if ((sReturn = adl_rtcConvertTime( &EngineStartTime, &CurrentTimeStamp, ADL_RTC_CONVERT_TO_TIMESTAMP)) < 0) 
			{
				DisplayErrorCode("adl_rtcConvertTime",__FILE__,__LINE__,sReturn);
	    	}
			//wm_itohexa((ascii *)&time_buffer, CurrentTimeStamp.TimeStamp, 8);

	        wm_sprintf(g_traceBuf,"RTC TimeStamp = 0x%x\r\n",(unsigned int)CurrentTimeStamp.TimeStamp);		
			DumpMessage(g_traceBuf);
			//DumpMessageUSB(g_traceBuf,1);

	        wm_sprintf(g_traceBuf,"Date(dd:mm:yy) = %02d:%02d:%04d\r\n",EngineStartTime.Day,EngineStartTime.Month,EngineStartTime.Year);		
			DumpMessage(g_traceBuf);
			//DumpMessageUSB(g_traceBuf,1);

	        wm_sprintf(g_traceBuf,"Time(hh:mm:ss) = %02d:%02d:%02d\r\n",EngineStartTime.Hour,EngineStartTime.Minute,EngineStartTime.Second);		
			DumpMessage(g_traceBuf);

			if ((sReturn = adl_rtcGetTime(&EngineStopTime)) < 0) 
			{
		    	DisplayErrorCode("adl_rtcGetTime",__FILE__,__LINE__,sReturn);
		    	return;
			}
		    if ((sReturn = adl_rtcConvertTime( &EngineStopTime, &CurrentTimeStamp, ADL_RTC_CONVERT_TO_TIMESTAMP)) < 0) 
			{
				DisplayErrorCode("adl_rtcConvertTime",__FILE__,__LINE__,sReturn);
	    	}
			
			DumpMessage("Test : Turn Engine ON\r\n");
			//g_SengEngineOff = TRUE;
			sms_send('I');	
			break;
		}

		if(strncmp("VER", ADL_GET_PARAM(Cmd, 0), 3) == 0)
		{
			extern const ascii adl_InitApplicationName[];
			DumpMessage("Sending Sierra Firmware Version\r\n");
			adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_USB, (ascii *)adl_InitApplicationName);
			//adl_fcmSendData(g_USBCommStruct.FCM_Handle, (unsigned char*)adl_InitApplicationName, 5);
			SendOkMsg = 0;
			break;
		}

		if(strncmp("SMS", ADL_GET_PARAM(Cmd, 0), 3) == 0)
		{
			DumpMessage("Sending Code upgrade SMS\r\n");
		 	//adl_smsSend(g_smsHandle, "+15444683002", (char *)"..D457DCE4B1234000277C0", ADL_SMS_MODE_TEXT);
		 	adl_smsSend(g_smsHandle, "+15444683002", (char *)"..MY014000B0911050", ADL_SMS_MODE_TEXT);
			//SendOkMsg = 0;
			break;
		}

		if(strncmp("POLLI", ADL_GET_PARAM(Cmd, 0), 5) == 0)
		{
			DumpMessage("Sending Code upgrade SMS\r\n");
		 	//adl_smsSend(g_smsHandle, "+15444683002", (char *)"..D457DCE4B1234000277C0", ADL_SMS_MODE_TEXT);
		 	adl_smsSend(g_smsHandle, "+15444683002", (char *)"..XPOLLI106646", ADL_SMS_MODE_TEXT);
			//SendOkMsg = 0;
			break;
		}

		if(strncmp("DEBUG1", ADL_GET_PARAM(Cmd, 0), 6) == 0)
		{
			DumpMessage("Analog, digital IO debug mode enable\r\n");
			g_DebugEnable = 1;
			StartAnalogSensorReadTimer();
			break;
		}

		if(strncmp("DEBUG0", ADL_GET_PARAM(Cmd, 0), 6) == 0)
		{
			DumpMessage("Analog, digital IO debug mode disable\r\n");
			g_DebugEnable = 0;
			StopAnalogSensorReadTimer();			
			break;
		}

		if(strncmp("DCONFIG", ADL_GET_PARAM(Cmd, 0), 7) == 0)
		{
			char Command[50];
			char RxCmd[50];

			DumpMessage("Send Diag Config SMS\r\n");
			memset(Command, '\0',50);
			memset(RxCmd, '\0',50);
			
			strcpy(Command, "..D0A0560");
			memcpy(RxCmd, ADL_GET_PARAM(Cmd, 0), strlen(ADL_GET_PARAM(Cmd, 0)));
			
			strcpy(&Command[9], &RxCmd[9]);

			Command[3]=RxCmd[7];
			Command[4]=RxCmd[8];			
			
/*
			if(strlen(ADL_GET_PARAM(Cmd, 1)) > 0)
			{
				strcpy(Command,",");
				memcpy(Command,ADL_GET_PARAM(Cmd, 1), strlen(ADL_GET_PARAM(Cmd, 1)));
			}

			if(strlen(ADL_GET_PARAM(Cmd, 2)) > 0)
			{
				strcpy(Command,",");
				memcpy(Command,ADL_GET_PARAM(Cmd, 2), strlen(ADL_GET_PARAM(Cmd, 2)));
			}

			if(strlen(ADL_GET_PARAM(Cmd, 3)) > 0)
			{
				strcpy(Command,",");
				memcpy(Command,ADL_GET_PARAM(Cmd, 3), strlen(ADL_GET_PARAM(Cmd, 3)));
			}
*/
			wm_sprintf(g_traceBuf,"Command: %s\r\n",Command);
			DumpMessage(g_traceBuf);
			
			//adl_smsSend(g_smsHandle, "+15444683002", (char *)"..D0A0110AT&XACT=38", ADL_SMS_MODE_TEXT);
			adl_smsSend(g_smsHandle, "+15444683002", (char *)Command, ADL_SMS_MODE_TEXT);
			break;
		}

		if(strncmp("DIAG", ADL_GET_PARAM(Cmd, 0), 4) == 0)
		{
			DumpMessage("Test Diag Command\r\n");
			adl_smsSend(g_smsHandle, "+15444683002", "..D140260AT&DIAGNOSE=AC,VM,16", ADL_SMS_MODE_TEXT);			
			break;
		}

		wm_sprintf(g_traceBuf, "Unhandled usbinterface command=%s\r\n", ADL_GET_PARAM(Cmd, 0));
		DumpMessage(g_traceBuf);
	}

	if (SendOkMsg)
	{
		SEND_OK_USB();
	}
}

void EnterAlarmMode()
{
	adl_rtcTime_t CurrentTime;
	adl_rtcTimeStamp_t CurrentTimeStamp;
	s32 sReturn=-1;
	ascii AlarmString[20];
	ascii AlarmCommand[30];
	DumpMessage("Invoking Alarm mode\r\n");
	DumpMessageUSB("Invoking Alarm mode\r\n",1);
	// Time.
	// get the current RTC time.
	if ((sReturn = adl_rtcGetTime(&CurrentTime)) < 0) 
	{
		DisplayErrorCode("adl_rtcGetTime",__FILE__,__LINE__,sReturn);
		return;
	}
	if ((sReturn = adl_rtcConvertTime( &CurrentTime, &CurrentTimeStamp, ADL_RTC_CONVERT_TO_TIMESTAMP)) < 0) 
	{
		DisplayErrorCode("adl_rtcConvertTime",__FILE__,__LINE__,sReturn);
	}
	//wm_itohexa((ascii *)&time_buffer, CurrentTimeStamp.TimeStamp, 8);
	
	wm_sprintf(g_traceBuf,"RTC TimeStamp = 0x%x\r\n",(unsigned int)CurrentTimeStamp.TimeStamp); 	
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf,1);
	
	wm_sprintf(g_traceBuf,"Date(dd:mm:yy) = %02d:%02d:%04d\r\n",CurrentTime.Day,CurrentTime.Month,CurrentTime.Year);		
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf,1);
	
	wm_sprintf(g_traceBuf,"Time(hh:mm:ss) = %02d:%02d:%02d\r\n",CurrentTime.Hour,CurrentTime.Minute,CurrentTime.Second);		
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf,1);
	
	
	//Set Alarm - 2 Mins from current time.
	CurrentTimeStamp.TimeStamp += 120; // 2 mins from current time.
	
	if ((sReturn = adl_rtcConvertTime( &CurrentTime, &CurrentTimeStamp, ADL_RTC_CONVERT_FROM_TIMESTAMP)) < 0) 
	{
		DisplayErrorCode("adl_rtcConvertTime",__FILE__,__LINE__,sReturn);
	}
	
	wm_sprintf(g_traceBuf,"RTC Wake-up TimeStamp = 0x%x\r\n",(unsigned int)CurrentTimeStamp.TimeStamp); 	
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf,1);
	
	wm_sprintf(g_traceBuf,"Wake up Date(dd:mm:yy) = %02d:%02d:%04d\r\n",CurrentTime.Day,CurrentTime.Month,CurrentTime.Year);		
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf,1);
	
	wm_sprintf(g_traceBuf,"Wake up Time(hh:mm:ss) = %02d:%02d:%02d\r\n",CurrentTime.Hour,CurrentTime.Minute,CurrentTime.Second);		
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf,1);
	
	wm_sprintf(AlarmString,"%04d/%02d/%02d,%02d:%02d:%02d",CurrentTime.Year,CurrentTime.Month,CurrentTime.Day,CurrentTime.Hour,CurrentTime.Minute,CurrentTime.Second);		
	DumpMessage(AlarmString);			
	DumpMessage("\r\n");
	DumpMessageUSB(AlarmString,1);
	DumpMessageUSB("\r\n",1);
	
	strcpy(AlarmCommand,"AT+CALA=\"");
	strcat(AlarmCommand,&AlarmString[2]);
	strcat(AlarmCommand,"\"");
	
	DumpMessage(AlarmCommand);
	DumpMessage("\r\n");
	DumpMessageUSB(AlarmCommand,1);
	DumpMessageUSB("\r\n",1);
	
	adl_atCmdCreate(AlarmCommand, FALSE, NULL, NULL);
	adl_atCmdCreate("AT+CALA?", FALSE, NULL, NULL);
	
	DumpMessage("\r\n Going to low power Alarm mode\r\n");
	DumpMessageUSB("\r\n Going to low power Alarm mode\r\n",1);
	
	adl_atCmdCreate("AT+CPOF", FALSE, NULL, NULL);

}
/** @brief event handler from Flow Control Manager from USB port
 *
 * @param Event
 * @return TRUE
 */
bool evhUSBCommFcmCtrlHandler(adl_fcmEvent_e Event)
{
	TRACE((1, "inside USB fcm_ctrlHandler"));
	switch (Event)
	{
	case ADL_FCM_EVENT_FLOW_OPENNED:
		TRACE((1, "FCM_FLOW_OPENED"));
		g_USBCommStruct.socketFlags |= 0x02;

		/*
		 * RetCode = adl_fcmSwitchV24State( sFcmHandle
		 * , ADL_FCM_V24_STATE_DATA );
		 * if(RetCode < 0 )
		 * {
		 * DisplayErrorCode("adl_fcmSwitchV24State"
		 * ,__FILE__
		 * ,__LINE__
		 * ,RetCode);
		 * }*/
		break;

	case ADL_FCM_EVENT_FLOW_CLOSED:
		TRACE((1, "ADL_FCM_EVENT_FLOW_CLOSED"));
		g_USBCommStruct.socketFlags &= 0x01;
		adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_USB, "\nADL_FCM_EVENT_FLOW_CLOSED\n");

		break;

	case ADL_FCM_EVENT_V24_DATA_MODE_EXT:
		TRACE((1, "ADL_FCM_EVENT_V24_DATA_MODE_EXT"));
		break;

	case ADL_FCM_EVENT_V24_DATA_MODE:
		TRACE((1, "ADL_FCM_EVENT_V24_DATA_MODE"));
		g_USBCommFCMDataMode = 1;
		break;

	case ADL_FCM_EVENT_V24_AT_MODE:
		TRACE((1, "ADL_FCM_EVENT_V24_AT_MODE"));
		g_USBCommFCMDataMode = 0;
		break;

	case ADL_FCM_EVENT_V24_AT_MODE_EXT:
		DumpMessage(("ADL_FCM_EVENT_V24_AT_MODE_EXT"));
		break;

	case ADL_FCM_EVENT_RESUME:
		TRACE((1, "ADL_FCM_EVENT_RESUME"));
		g_USBCommStruct.socketFlags |= 0x02;
		break;

	case ADL_FCM_EVENT_MEM_RELEASE:
		TRACE((1, "ADL_FCM_EVENT_MEM_RELEASE"));
		break;

	case ADL_FCM_EVENT_V24_DATA_MODE_FROM_CALL:
		TRACE((1, "ADL_FCM_EVENT_V24_DATA_MODE_FROM_CALL"));
		break;

	case ADL_FCM_EVENT_V24_AT_MODE_FROM_CALL:
		TRACE((1, "ADL_FCM_EVENT_V24_AT_MODE_FROM_CALL"));
		break;
	}
	return TRUE;
}


static int usb_byte_cnt = 0;

/** @brief USB port Data Event Handler
 *
 * @par This method is called when data is received on the USB port.
 *
 * @param DataSize
 * @param Data
 * @return TRUE
 */
bool evhUSBCommFcmDataHandler(u16 DataSize, u8 *Data)
{
	usb_byte_cnt += DataSize;
	if (g_wyhook_mode == NORMAL_MODE)
	{
		if (g_USBCommStruct.DownloadFlag == 0)
		{
			// we're going to copy the recieved data to our buffer
			//
			if (g_USBCommStruct.rcvData == NULL)
			{
				g_USBCommStruct.rcvData = malloc(DataSize);
			}
			else
			{
				g_USBCommStruct.rcvData = realloc(g_USBCommStruct.rcvData, g_USBCommStruct.rcvDataSize + DataSize);
			}
			if (g_USBCommStruct.rcvData != NULL)
			{
				//TVDP: CRITICAL Syntax error in for loop.
				memcpy(g_USBCommStruct.rcvData + g_USBCommStruct.rcvDataSize, Data, DataSize);
				g_USBCommStruct.rcvDataSize += DataSize;
			}
			else
			{
				TRACE((1, "ERROR memory error %s %d\n", __FILE__, __LINE__));
			}
		}
		else
		{
			//		TRACE("rx data from USB app\n");
#ifdef DOTA_ENABLED
			if (DataSize > 0)				
			{
				ADWriteData(DataSize, Data);
			}
#endif
		}
	}
	// this is for wyhook mode.
	else
	{
		parseSLIPPacket(DataSize, Data);
	}
	return TRUE;
}


/** @brief Parse SLIP Packet
 *
 * @par
 * Convert the incoming Wyhook packet. See Wyhook documention for packet format details.
 * @param DataSize
 * @param Data
 * @return TRUE
 */
bool parseSLIPPacket(u16 DataSize, u8 *Data)
{
	static u8 rcvData[520];
	int i;

	for (i = 0; i < DataSize; i++)
	{
		if (Data[i] == 0xC0)
		{
			if (g_USBCommStruct.rcvDataSize > 0)
			{
				//parse and packup
				wyhook_ProcessPacketBuffer(g_USBCommStruct.rcvDataSize, rcvData);
				g_USBCommStruct.rcvDataSize = 0;
			}
		}
		else if (Data[i] == 0xDB)
		{
			if (Data[i + 1] == 0xDC)
			{
				rcvData[g_USBCommStruct.rcvDataSize++] = 0xC0;
				i++;
			}
			else if (Data[i + 1] == 0xDD)
			{
				rcvData[g_USBCommStruct.rcvDataSize++] = 0xDB;

				i++;
			}
		}
		else
		{
			rcvData[g_USBCommStruct.rcvDataSize++] = Data[i];
		}
		if (g_USBCommStruct.rcvDataSize >= 520)
		{
			g_USBCommStruct.rcvDataSize = 0;
		}
	}
	return TRUE;
}


#if 1

/** @brief Change USB communication flow between data and AT modes
 *
 * @par
 * This will switch the USB port between AT command mode and data mode.
 * Data mode is only used when the device is in Wyhook mode.
 * @param i_comm
 * @param FlowState
 *
 * @return RetCode
 */
s8 ChangeV24State(commPort_type i_comm, u8 FlowState)
{
	//FIXME: remove this parameter if we aren't using it -- pjn
	(void)i_comm;

	s8 RetCode = 0;

	if (FlowState == ADL_FCM_V24_STATE_DATA)
	{
		if (g_USBCommStruct.FCM_Handle > -1)
		{
			D_TRACE("using File Tx Flow\n");
			SEND_OK_USB();
			RetCode = adl_fcmSwitchV24State(g_USBCommStruct.FCM_Handle
			                                , ADL_FCM_V24_STATE_DATA);
		}
		else
		{
			D_TRACE("using Python interface\n");
			adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_USB, "\nERROR\n");
			RetCode = adl_fcmSwitchV24State(g_USBCommStruct.FCM_Handle
			                                , ADL_FCM_V24_STATE_DATA);
		}
		if (RetCode < 0)
		{
			DisplayErrorCode("adl_fcmSwitchV24State"
			                 , __FILE__
			                 , __LINE__
			                 , RetCode);
		}
		/*				D_TRACE("flow control set to ADL_FCM_V24_STATE_DATA \r\n"); */
	}
	if (FlowState == ADL_FCM_V24_STATE_AT)
	{
		if (g_USBCommStruct.FCM_Handle > -1)
		{
			D_TRACE("using File Tx Flow\n");
			DumpMessage("using File Tx Flow\n");
			//	    SEND_OK_USB();
			RetCode = adl_fcmSwitchV24State(g_USBCommStruct.FCM_Handle
			                                , ADL_FCM_V24_STATE_AT);
		}
		else
		{
			D_TRACE("using Python interface\n");
			adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_USB, "\nERROR\n");
			RetCode = adl_fcmSwitchV24State(g_USBCommStruct.FCM_Handle
			                                , ADL_FCM_V24_STATE_AT);
		}
		if (RetCode < 0)
		{
			DisplayErrorCode("adl_fcmSwitchV24State"
			                 , __FILE__
			                 , __LINE__
			                 , RetCode);
		}
		/*				D_TRACE("flow control set to ADL_FCM_V24_STATE_AT\n");*/
		//	SEND_OK_USB();
	}

	return RetCode;
}


#else
s8 ChangeV24State(commPort_type i_comm, u8 FlowState)
{
	s8 RetCode = 0;
	s8 FlowHandle = -1;

	{
		if (g_USBCommStruct.FCM_Handle > -1)
		{
			D_TRACE("using File Tx Flow\n");
			SEND_OK_USB();
			DumpMessage("using File Tx Flow\r\n");

			RetCode = adl_fcmSwitchV24State(g_USBCommStruct.FCM_Handle
			                                , ADL_FCM_V24_STATE_DATA);
		}
		else
		{
			D_TRACE("using Python interface\n");
			DumpMessage("using Python interface\r\n");
			adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_USB, "\nERROR\n");
			RetCode = adl_fcmSwitchV24State(g_USBCommStruct.FCM_Handle
			                                , ADL_FCM_V24_STATE_DATA);
		}
		if (RetCode < 0)
		{
			DisplayErrorCode("adl_fcmSwitchV24State"
			                 , __FILE__
			                 , __LINE__
			                 , RetCode);
		}
		else
		{
			DumpMessage("In data mode!\r\n");
		}
		/*				D_TRACE("flow control set to ADL_FCM_V24_STATE_DATA \r\n"); */
	}

	/*if( FlowState == ADL_FCM_V24_STATE_AT )
	 * {
	 * if( g_USBCommStruct.FCM_Handle > -1 )
	 * {
	 * D_TRACE("using File Tx Flow\n");
	 * adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_USB,"\n<usb>\r\nOK\r\n</usb>\n");
	 * RetCode = adl_fcmSwitchV24State(g_USBCommStruct.FCM_Handle
	 * , ADL_FCM_V24_STATE_AT );
	 * }
	 * else
	 * {
	 * D_TRACE("using Python interface\n");
	 * adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_USB,"\nERROR\n");
	 * RetCode = adl_fcmSwitchV24State(g_USBCommStruct.FCM_Handle
	 * , ADL_FCM_V24_STATE_AT );
	 * }
	 * if(RetCode < 0 )
	 * {
	 * DisplayErrorCode("adl_fcmSwitchV24State"
	 * ,__FILE__
	 * ,__LINE__
	 * ,RetCode);
	 * }
	 * //				D_TRACE("flow control set to ADL_FCM_V24_STATE_AT\n");
	 * adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_USB,"\n<usb>\r\nOK\r\n</usb>\n");
	 * }
	 *
	 *
	 */
	return RetCode;
}


#endif
/*@}*/
