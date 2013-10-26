/** @addtogroup GPS
 *@{ */
/** @file ublox.c */

/****************************************************************************
*
*  ublox.c
*
****************************************************************************
*
*  MODIFICATION HISTORY
* --------------------------------------------------------------------------
*  Date      | Author      | Revision | Description
* -----------+-------------+----------+-------------------------------------
*  17-Sep-09 | C. Eslinger | 1.0      | * Initial Release
* --------------------------------------------------------------------------
*
****************************************************************************
*
*  Object: Handle the retro-fitted ublox to replace eride gps
*
****************************************************************************/

#include "adl_global.h"
#include "ublox.h"
#include "wip.h"
#include "GpsInterface.h"
#include "XactUtilities.h"

#define UBLOX_BAUD_RATE         9600

#define CR                      13
#define END_OF_NMEA_MESSAGE     CR
#define STRING_TERMINATOR       '\0'

#define UBLOX_POWER_GPIO        23 //!< GPIO used to control 1.2V to nanoRide

char GPS_CONFIG_STRING[10] =
{
	0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x00, 0x04, 0x1D, 0x85
};

static u8 checkChecksum(char *theNmea);

/***************************************************************************/
/*  Local variables and prototypes                                         */
/***************************************************************************/

/** @struct gpsUblox
 *
 * @par Global status structure fo the ublox controller.
 */
struct
{
	long numSentencesProcessed;
	bool showNMEA;
	bool ubloxPowered;
	bool initialized;
	s32 gpsPowerGpioHandle;
	adl_ioDefs_t gpioConfig;
}
gpsUblox;

/* Handle returned by adl_fcmSubscribe for GPS UART */
s8 ubloxDataFcmHandle;

static bool serialInitResponseHandler(adl_atResponse_t *paras);
static void continueUbloxSerialInit(u8 id);
static void initGpsStruct(void);
static void processNmeaSentence(char *nmeaSentence);
static bool ubloxDataCtrlHandler(u8 event);
static u8 ubloxDataHandler(u16 datalength, u8 *data);
bool isValidNMEAchar(char theChar);

/***************************************************************************/
/*  Global variables and functions                                         */
/***************************************************************************/

/** @brief Init GPIO for Ublox
 *
 * @par Subscribe to the GPIO to control the Ublox power.
 * @return void
 */
void ublox_InitGPIO(void)
{
	// setup GPIO to control the power
	wip_debug("\nSetting up GPIO for ublox GPS...\n");
	gpsUblox.gpioConfig = UBLOX_POWER_GPIO |
	                      ADL_IO_GPIO |
	                      ADL_IO_DIR_OUT |
	                      ADL_IO_LEV_LOW;

	gpsUblox.gpsPowerGpioHandle = adl_ioSubscribe(1, &gpsUblox.gpioConfig, 0, 0, 0);
	wip_debug("\n GPS GPIO handle: %d\n", gpsUblox.gpsPowerGpioHandle);
	gpsUblox.ubloxPowered = FALSE;
}


/** @brief start ublox
 *
 * @par Turn on the GPS if it is off.
 * @return void
 */
void ublox_Start(void)
{
	if (gpsUblox.ubloxPowered)
	{
		wip_debug(" GPS already on.\n");
	}
	else
	{
		// init variables we want to start over each time we resume
		initGpsStruct();

		wip_debug(" Turning on ublox GPS.\n");

		// write to GPIO23 to turn on its power
		adl_ioWriteSingle(gpsUblox.gpsPowerGpioHandle,
		                  &gpsUblox.gpioConfig, TRUE);
		gpsUblox.ubloxPowered = TRUE;
	}
}


/*F***************************************************************************/

/** @brief Setup RX channel
 *
 * @par Starts the GPS after a pause or power down.
 *      Subscribes to the open at serial port.
 * @return void
 */
void ublox_SetupRXchannel(void)
{
	//Open the Data FCM flow on the DATAGPS
	ubloxDataFcmHandle = adl_fcmSubscribe((adl_fcmFlow_e)GPS_UART,
	                                      (adl_fcmCtrlHdlr_f)ubloxDataCtrlHandler,
	                                      (adl_fcmDataHdlr_f)ubloxDataHandler);
}


/*F***************************************************************************/

/** @brief Send Startup Commands
 *
 * @par At startup, send commands to put the GPS in ECO mode.
 *
 * @param timerid
 * @param context
 * @return void
 */
void ublox_SendStartupCommands(u8 timerid, void *context)
{
	(void)timerid;
	(void)context;

	static int timer_cnt = 0;
	int ret_val;

	timer_cnt++;

	if (timer_cnt < 5)
	{
		adl_tmrSubscribe(FALSE, 10, ADL_TMR_TYPE_100MS, ublox_SendStartupCommands);
	}
	else
	{
		if ((ret_val = adl_fcmSendData(ubloxDataFcmHandle, (unsigned char *)&GPS_CONFIG_STRING[0], 10)) != OK)
		{
			wip_debug("Error: Could not send GPS config string! %d\r\n", ret_val);
		}
		else
		{
			wip_debug("Send UBLOX startup command\r\n");
		}
	}
}


/*F***************************************************************************/

/** @brief Init Serial port for uBlox
 *
 * @par Initialize the serial port used to communicate with the
 * receiver-hardware.
 * @return void
 */
void ublox_InitSerialPort(void)
{
	static u8 CommandSent = 0;
	ascii ATCommand[25];

	gpsUblox.initialized = FALSE;

	switch (CommandSent)
	{
	case 0:
		// Open the NMEA through DATAGPS: The NMEA is closed by default
		wip_debug("   Sending command: AT+WMFM=0,1,2\n");
		adl_atCmdCreate("AT+WMFM=0,1,2", ADL_AT_PORT_TYPE(ADL_PORT_NONE, FALSE), (adl_atRspHandler_t)serialInitResponseHandler, "*", NULL);
		CommandSent++;
		break;

	case 1:
		/****** Initialize the serial port for GPS chipset as an FCM port ******/
		//Set the baud rate
		wm_sprintf(ATCommand, "AT+IPR=%d", UBLOX_BAUD_RATE);
		adl_atCmdCreate(ATCommand, ADL_AT_PORT_TYPE(GPS_UART, FALSE), (adl_atRspHandler_t)serialInitResponseHandler, "*", NULL);
		wip_debug("   Sending command: %s\n", ATCommand);
		CommandSent++;
		break;

	case 2:
		//Set to 8 data bits, odd parity, 2 stop bit
		adl_atCmdCreate("AT+ICF=3,0", ADL_AT_PORT_TYPE(GPS_UART, FALSE), (adl_atRspHandler_t)serialInitResponseHandler, "*", NULL);
		wip_debug("   Sending command: AT+ICF=3,0\n");
		CommandSent++;
		break;

	case 3:
		//No flow control
		adl_atCmdCreate("AT+IFC=0,0", ADL_AT_PORT_TYPE(GPS_UART, FALSE), (adl_atRspHandler_t)serialInitResponseHandler, "*", NULL);
		wip_debug("   Sending command: AT+IFC=0,0\n");
		CommandSent++;
		break;

	case 4:
		wip_debug("   GPS Serial port setup complete.");
		CommandSent = 0;
		break;
	}
}


/*F***************************************************************************/

/** @brief Stop uBlox
 *
 * @par Disables power to the ublox GPS module
 * @return void
 */
void ublox_stop(void)
{
	if (gpsUblox.ubloxPowered)
	{
		// turn power off (backup power is still on to keep ublox RAM
		adl_ioWriteSingle(gpsUblox.gpsPowerGpioHandle,
		                  &gpsUblox.gpioConfig, FALSE);

		wip_debug(" Killing power to GPS.\n");
		gpsUblox.ubloxPowered = FALSE;
	}
	else
	{
		wip_debug(" GPS power already off.\n");
	}
}


/*F***************************************************************************/

/** @brief is ubox ON
 *
 * @par ask if uBox is currently ON
 * @return TRUE ublox is on
 * @return FALSE ublox is off
 */
bool ublox_isON(void)
{
	return gpsUblox.ubloxPowered;
}


/*F***************************************************************************/

/** @brief Show Data
 *
 * @par Control the displaying of NMEA senetences out the USB.
 * Pass in TRUE for displaying sentences, FALSE for turning them off.
 * @param setting
 * @return void
 */
void ublox_ShowData(bool setting)
{
	if (!gpsUblox.ubloxPowered)
	{
		wip_debug("\n  GPS-- GPS power is currently OFF. ");
	}
	gpsUblox.showNMEA = setting;
}


/***************************************************************************/
/*  Local functions                                                        */
/***************************************************************************/

/*F***************************************************************************/

/** @brief Init Gps Structure
 *
 * @par Initializes gps structure. Sets number of sentences to 0.
 * @return void
 */
static void initGpsStruct(void)
{
	gpsUblox.numSentencesProcessed = 0;
}


/*F***************************************************************************/

/** @brief Response handler for serial port
 *
 * @par Response handler function for the AT command created using
 * adl_atCmdCreate () API.
 *
 * @return TRUE on sucess, FALSE on no subcription
 */
static bool serialInitResponseHandler(adl_atResponse_t *paras)
{
	bool bReturn = FALSE;

	if (strstr((const char *)(paras->StrData), "OK") == NULL)
	{
		wip_debug("   response: %s      (CME error 3 is OK for WMFM command.) \n", paras->StrData + 3);
	}
	adl_tmrSubscribe(FALSE, 1, ADL_TMR_TYPE_TICK, (adl_tmrHandler_t)continueUbloxSerialInit);

	return bReturn;
}


/*F***************************************************************************/

/** @brief Continue with Ublox Serial Init
 *
 * @par This function calls the ublox_InitSerialPort function which
 * sets up the serial port to initialize communication with Ublox module.
 *
 * @param id
 * @return void
 */
static void continueUbloxSerialInit(u8 id)
{
	(void)id;
	ublox_InitSerialPort();
}


/*F***************************************************************************/

/** @brief Valid NMEA char
 *
 * @par Check if the character is part of a valid NMEA string.
 * @return TRUE on success
 */
bool isValidNMEAchar(char theChar)
{
	if ((theChar >= '0') && (theChar <= '9'))
	{
		// character is number
		return TRUE;
	}
	else if ((theChar >= 'A') && (theChar <= 'Z'))
	{
		// character is Letter
		return TRUE;
	}
	else if ((theChar == '$') || (theChar == '-') || (theChar == '.') || (theChar >= '*') || (theChar >= ','))
	{
		// character is acceptable non alphanumeric
		return TRUE;
	}
	return FALSE;
}


/*F***************************************************************************/

/** @brief Data Control Handler
 *
 * @par Data Ctrl Handler monitors events on the Ublox serial port.
 * @return bReturn
 */
static bool ubloxDataCtrlHandler(u8 event)
{
	bool bReturn = TRUE;
	s8 sReturn;

	switch (event)
	{
	case ADL_FCM_EVENT_FLOW_OPENNED:
		sReturn = adl_fcmSwitchV24State(ubloxDataFcmHandle, ADL_FCM_V24_STATE_DATA);
		break;

	case ADL_FCM_EVENT_FLOW_CLOSED:
		wip_debug("   Data connection to UBLOX closed.\n");
		break;

	case ADL_FCM_EVENT_V24_DATA_MODE:           // 2
		wip_debug("   Data connection to UBLOX ready.\n");
		// set global variable to indicate GPS is ready
		gpsUblox.initialized = TRUE;
		adl_tmrSubscribe(FALSE, 1, ADL_TMR_TYPE_100MS, ublox_SendStartupCommands);

		break;

	case ADL_FCM_EVENT_V24_DATA_MODE_EXT:
	case ADL_FCM_EVENT_V24_AT_MODE:
	case ADL_FCM_EVENT_V24_AT_MODE_EXT:
	case ADL_FCM_EVENT_RESUME:
	case ADL_FCM_EVENT_MEM_RELEASE:
	case ADL_FCM_EVENT_V24_DATA_MODE_FROM_CALL:
	case ADL_FCM_EVENT_V24_AT_MODE_FROM_CALL:
		break;
	}

	return bReturn;
}


/*F***************************************************************************/

/** @brief ubox Data Handler
 *
 * @par Data received from serial port is proccessed in this data handler.
 * @return void
 */
static u8 ubloxDataHandler(u16 datalength, u8 *data)
{
	static char gpsMessage[MAX_NMEA_SENTENCE_LENGTH + 1];
	static int gpsMsgIndex = -1;
	char theNextChar;
	int index;
	int foundUBXPacket = 0;

	for (index = 0; index < datalength; index++)
	{
		theNextChar = data[index];

		if ((theNextChar != '$') && (index == 0))
		{
			//		    wip_debug("UBLOX CHAR = %x\r\n",theNextChar);
		}

		// here's a hack to look for a UBX ACK packet.
		if ((theNextChar == 0xb5) && (foundUBXPacket == 0))
		{
			wip_debug("UBLOX = %d\r\n", foundUBXPacket);
			foundUBXPacket++;
			continue;
		}
		if ((theNextChar == 0x62) && (foundUBXPacket == 1))
		{
			wip_debug("UBLOX = %d\r\n", foundUBXPacket);
			foundUBXPacket++;
			continue;
		}
		if ((theNextChar == 0x05) && (foundUBXPacket == 2))
		{
			wip_debug("UBLOX = %d\r\n", foundUBXPacket);
			foundUBXPacket++;
			continue;
		}
		if ((theNextChar == 0x00) && (foundUBXPacket == 3))
		{
			wip_debug("UBLOX = %d\r\n", foundUBXPacket);
			wip_debug("UBX PACKET: Got a NACK\r\n");
			break;
		}
		if ((theNextChar == 0x01) && (foundUBXPacket == 3))
		{
			wip_debug("UBLOX = %d\r\n", foundUBXPacket);
			wip_debug("UBX PACKEt: Got an ACK\r\n");
			break;
		}

		// no matter what, if we find a '$' we'll start over with the sentence
		if (theNextChar == '$')
		{
			gpsMsgIndex = 0;
			gpsMessage[gpsMsgIndex++] = theNextChar;
		}
		else if (gpsMsgIndex > 0)
		{
			// we've already found the absolute beginning of the NMEA
			if (gpsMsgIndex == MAX_NMEA_SENTENCE_LENGTH)
			{
				wip_debug("GPS nmea string overrun!\n");
				// pretend this is the end and process
				theNextChar = END_OF_NMEA_MESSAGE;
			}
			else if (theNextChar == END_OF_NMEA_MESSAGE)
			{
				// We've completed reading a line - now process
				gpsMessage[gpsMsgIndex++] = theNextChar;
				gpsMessage[gpsMsgIndex] = STRING_TERMINATOR;
				processNmeaSentence(gpsMessage);

				// set index to look for new NMEA ('$')
				gpsMsgIndex = -1;
				// set time of report and reset message buffer
				//gps.timeOfLastReport = GetTickMsec();
			}
			else if (isValidNMEAchar(theNextChar))
			{
				gpsMessage[gpsMsgIndex++] = theNextChar;
			}
			else
			{
				//wip_debug("!");
			}
		}
		else
		{
			// else waiting for the beginning of the NMEA sentence
			//wip_debug("~");
		}
	}     // done with UART data

	return TRUE;
}


//#define DEMO_BUILD

/*F***************************************************************************/

/** @brief process MEA Sentence
 *
 * @par Upon reception of a Nmea sentence, this is called to send the string
 * out the USB and to the rest of the application.
 * @param theNMEA
 * @return void
 */
static void processNmeaSentence(char *theNMEA)
{
	char nmeaSentence[MAX_NMEA_SENTENCE_LENGTH + 1];

	// copy to another string in case more data comes in...
	if (strlen(theNMEA) > MAX_NMEA_SENTENCE_LENGTH + 1)
	{
		wip_debug("processNmeaSentence error: String too long\r\n");
		return;
	}

	strcpy(nmeaSentence, theNMEA);

	// be sure this is a NMEA sentence before parsing it
	if (nmeaSentence[0] != '$')
	{
		// this is not a NMEA sentence--report it to user
		//wip_debug("---GPS-->%s\n",nmeaSentence);
	}
	else
	{
		if (gpsUblox.showNMEA)
		{
			//	wip_debug("---GPS-->%s\n",nmeaSentence);
			//	    wip_debug("%s\n",nmeaSentence);

			//	  wip_debug("%s\n",nmeaSentence);
			wm_sprintf(g_traceBuf, "%s\n", nmeaSentence);
			adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_USB, g_traceBuf);
			adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_UART1, g_traceBuf);
		}

		gpsUblox.numSentencesProcessed++;
		// send to original Xact app...
		if (!checkChecksum(theNMEA))
		{
#if 1
			ParseNMEA(nmeaSentence, strlen(nmeaSentence));
#else
#warning GPS STRING PARSING DISABLED!
#endif
		}
		else
		{
			NMEAFailure(nmeaSentence, strlen(nmeaSentence));
			DumpMessage("NMEA Checksum failure! Will not process string!\r\n");
		}
	}
}


/** @brief Calculate the checksum of a NMEA sentence.
 *   @param theNmea
 *   @return the calculated checksum.
 */
static u8 checkChecksum(char *theNMEA)
{
	u8 checksum = 0;
	int ii = 0;
	u8 checksum_read = 0;

	for (ii = 0; ((theNMEA[ii] != '*') && (theNMEA[ii] != 0x00)); ii++)
	{
		if (theNMEA[ii] == '$')
		{
			continue;
		}
		if (checksum == 0)
		{
			checksum = theNMEA[ii];
		}
		else
		{
			checksum ^= theNMEA[ii];
		}
	}
	checksum_read = wm_hexatoi(&theNMEA[ii + 1], 2);
	if (checksum_read != checksum)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}


/**@}*/
