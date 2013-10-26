/** @addtogroup GPS
 *@{*/

/** @file GpsInterface.c
 */

/******************************************************************************
 * @brief  This file contains all interface API to the nanoRide chip set
 * GpsInit()
 *
 *****************************************************************************/

/* -------- */
/* Includes */
/* -------- */
#include "adl_global.h"
#include "GpsInterface.h"
#include "Timers.h"
#include "XactUtilities.h"
#include "status.h"
#include "ublox.h"
#include "wip.h"
#include "XactUtilities.h"

// defines for the ported utility functions
#define SUCCESS                     0
#define NULL_CHARACTER              0
#define MAX_STR_LEN_TO_PARSE        100
#define MAX_SIZE_LAT_LONG           13 // size of string values
#define MAX_SIZE_NMEA_FIELD         13
#define NUM_GPS_SATS                50
#define MAX_NMEA_SENTENCE_LENGTH    100 // NMEA spec states 82
#define NMEA_SENTENCES_USED         5   // should match number of enums below

#define CHECKSUM_PREFIX             "*"

// RMC field indexes
#define RMC_MIN_FIELDS              13
#define RMC_UTC_TIME                2
#define RMC_FIX_STATUS              3
#define RMC_LATITUDE                4
#define RMC_N_OR_S                  5
#define RMC_LONGITUDE               6
#define RMC_W_OR_E                  7
#define RMC_SPEED                   8
#define RMC_HEADING                 9
#define RMC_DATE                    10
#define RMC_MAX_FIELD_SIZE          20 // TODO: check on this
#define ACTUAL_FIX                  'A'
#define NO_FIX                      'V'

enum gps_NMEA_t
{
	RMC=0, GSV1=1, GSV2=2, GSV3=3, GSV4=4
};                                                     // update as needed

// indicates if the last time a GGA string was encountered we got a GGA fix.
static char GotGGAFix = NO_FIX;

/* --------------- */
/* Structures      */
/* --------------- */

typedef struct
{
	BOOL dataReady;
	UINT32 timeOfLastReport;
	UINT32 numSentencesProcessed;
	//double latitude;
	//double longitude;
	unsigned char nmeaSentences[NMEA_SENTENCES_USED][MAX_NMEA_SENTENCE_LENGTH + 1];

	unsigned char latitude[MAX_SIZE_LAT_LONG];
	unsigned char longitude[MAX_SIZE_LAT_LONG];
	unsigned char eastOrWest[2];          // these are only one char, but we want to treat them as strings
	unsigned char northOrSouth[2];
	unsigned char speed[10];
	unsigned char heading[10];
	unsigned char hdop[10];
	unsigned char utcTime[10];
	unsigned char date[10];
	unsigned char satsInView;
	int satSNRs[NUM_GPS_SATS];                 // SNRs range from 0 to 99

	// double speedKnotsRMC;
	// double headingRMC;

	UINT32 ttffSecs;
	UINT32 ttffStartTime;
	UINT32 timeOfFix;
	BOOL currentlyFixed;
	BOOL afterFirstFix;

	BOOL extAntenna;
	BOOL ubloxReady;
	BOOL ubloxPowered;
} gpsState_t;

gpsState_t gps;

static void processNmeaGGA(unsigned char *nmeaSentence);
static void processNmeaRMC(unsigned char *nmeaSentence);
static void processNmeaGSV(unsigned char *nmeaSentence);

static void ClearSNRs(void);

BOOL util_GetStringField(unsigned char *sentence, int fieldIndex, int maxFieldSize, unsigned char *field);
BOOL util_IsAllDigits(unsigned char *theString);

typedef enum
{
	NO_SAT_IN_VIEW,
	SAT_IN_VIEW
} VISIBLE_SATS;

static VISIBLE_SATS visible_sats;
static void set_visible_sats(VISIBLE_SATS vs);
static VISIBLE_SATS get_visible_sats(void);

typedef struct
{
	int SNRHigh;
	int SNRLow;
} SNRStruct;

/* --------------- */
/* Global Variables*/
/* --------------- */

static int GPSInitialized = 0;
/*< global variable that indicates if the GPS initialization has completed.*/

/* -------------- */
/* Local Variables*/
/* -------------- */

#define OUTPUT_GPIO_LABEL    (ADL_IO_GPIO | 0)

char NNMEAGGAString[] = "$GPGGA,";
char NNMEAVTGString[] = "$GPVTG,";
char NNMEARMCString[] = "$GPRMC,";
char NNMEAGSVString[] = "$GPGSV,";

#define MAX_NUM_SV    (12)

//#define DISABLE_GPS_GPIO

static struct structGGAInformation gGGAInfo;
struct structVTGInformation gVTGInfo;

static NMEA_ERRORS nmea_errors =
{
	0, 0, 0
};

int g_GPSTestMode = 0;

/* -------------- */
/* Local Functions*/
/* -------------- */

extern struct structGGAInformation gGGAInfoTest;
extern struct structVTGInformation gVTGInfoTest;

/* ------------- *
* prototypes
* --------------*/
int util_NumberOfFields(unsigned char *sentence);

s32 GPIOGPSHandle;

static SNRStruct snr_struct;

/**@brief Initalise the GPS Interface
 *
 * @return TRUE
 */
s32 GpsInit(void)
{
	wip_debug(" \n Initialize GPS...\n");

	ublox_InitSerialPort();         // setup UART2
	ublox_SetupRXchannel();         // setup FCM handler

	ublox_InitGPIO();               // setup GPIO to turn on and off

	// subscribe to test AT commands
	adl_atCmdSubscribe("AT&NMEA", (adl_atCmdHandler_t)CmdHandlerNMEA,
	                   ADL_CMD_TYPE_ACT | ADL_CMD_TYPE_TEST | ADL_CMD_TYPE_READ | ADL_CMD_TYPE_PARA | 0x00C1);

	adl_atCmdSubscribe("AT&GPSON", (adl_atCmdHandler_t)CmdHandlerGPSON, ADL_CMD_TYPE_ACT);
	adl_atCmdSubscribe("AT&GPSOFF", (adl_atCmdHandler_t)CmdHandlerGPSOFF, ADL_CMD_TYPE_ACT);

	// and we'll power it up...
	ublox_Start();
	//	ublox_SendStartupCommands(); // send any needed startup commands.
	GPSInitialized = 1;

	return TRUE;
}


/** @brief Get GGA NMEA information
 *
 * @par This method return the current GGA structure information.
 *
 * @param o_GGAInfo
 * @return void
 */
void GetGGAInfo(struct structGGAInformation *o_GGAInfo)
{
	if (!g_GPSTestMode)
	{
		o_GGAInfo->Time = gGGAInfo.Time;
		o_GGAInfo->Latitude = gGGAInfo.Latitude;
		o_GGAInfo->Longitude = gGGAInfo.Longitude;
		o_GGAInfo->FixQuality = gGGAInfo.FixQuality;
		o_GGAInfo->NumberOfSatillites = gGGAInfo.NumberOfSatillites;
		o_GGAInfo->HDOP = gGGAInfo.HDOP;
		gGGAInfo.FixQuality = NO_FIX;
		TRACE((1, "Set fix quality to %d", gGGAInfo.FixQuality));
	}
	else
	{
		o_GGAInfo->Time = gGGAInfoTest.Time;
		o_GGAInfo->Latitude = gGGAInfoTest.Latitude;
		o_GGAInfo->Longitude = gGGAInfoTest.Longitude;
		o_GGAInfo->FixQuality = gGGAInfoTest.FixQuality;
		o_GGAInfo->NumberOfSatillites = gGGAInfoTest.NumberOfSatillites;
		gGGAInfoTest.FixQuality = 0;
	}
}


/** @brief gather VGT Information
 *
 * @param o_VTGInfo
 * @return void
 */
void GetVGTInfo(struct structVTGInformation *o_VTGInfo)
{
	if (!g_GPSTestMode)
	{
		o_VTGInfo->Course = gVTGInfo.Course;
		o_VTGInfo->Speed = gVTGInfo.Speed;
	}
	else
	{
		o_VTGInfo->Course = gVTGInfoTest.Course;
		o_VTGInfo->Speed = gVTGInfoTest.Speed;
	}
}


/** @brief This method return then number of locked satillites
 *
 * @par description: This method return then number of locked
 * satillites, as well as a ratio the strongest SNR, and
 * weakest SNR levels. This return strength level will be
 * the measured SNR divided by nine with the lowest level
 * being zero and the highest being nine.
 *
 * @param o_NumberOfSV
 * @param o_StrongSNR
 * @param o_WeakSNR
 * @return void
 */
void GetSV_SRN(int *o_NumberOfSV, int *o_StrongSNR, int *o_WeakSNR)
{
	/* FIXME: why are we passing in this value if we aren't using it? --pjn */
	(void)o_NumberOfSV;

	*o_StrongSNR = snr_struct.SNRHigh / 10;
	*o_WeakSNR = snr_struct.SNRLow / 10;
	ClearSNRs();
}


/** @brief This method will count checksum failures.
 *
 * @param Str
 * @param iLength
 * @return 0
 */
void NMEAFailure(char *Str, int iLength)
{
	/* FIXME: why are we passing in this value if we aren't using it? --pjn */
	(void)iLength;

	if (strncmp(Str, NNMEARMCString, strlen(NNMEARMCString)) == 0)
	{
		nmea_errors.RMCErrors++;
	}
	else if (strncmp(Str, NNMEAGSVString, strlen(NNMEAGSVString)) == 0)
	{
		nmea_errors.GSVErrors++;
	}
	else if (strncmp(Str, NNMEAGGAString, strlen(NNMEAGGAString)) == 0)
	{
		nmea_errors.GGAErrors++;
	}
}


void GetNMEAError(NMEA_ERRORS *nm)
{
	memcpy(nm, &nmea_errors, sizeof (NMEA_ERRORS));
}


/** @brief This method will parse NMEA strings
 *
 * @par This method will parse NMEA strings provid and
 * decided how to further parse them for the informaiton they have.
 *
 * @param Str
 * @param iLength
 * @return 0
 */
int ParseNMEA(char *Str, int iLength)
{
	/* FIXME: why are we passing in this value if we aren't using it? --pjn */
	(void)iLength;

	if (strncmp(Str, NNMEARMCString, strlen(NNMEARMCString)) == 0)
	{
		processNmeaRMC((unsigned char *)Str);
	}
	else if (strncmp(Str, NNMEAGSVString, strlen(NNMEAGSVString)) == 0)
	{
		processNmeaGSV((unsigned char *)Str);
	}
	else if (strncmp(Str, NNMEAGGAString, strlen(NNMEAGGAString)) == 0)
	{
		processNmeaGGA((unsigned char *)Str);
	}
	return 0;
}


/** @brief return status of GPS
 *
 * @return GPSInitialized
 */
int IsGPSInitialized(void)
{
	return GPSInitialized;
}


/** @brief Start GPS Operation
 *
 * @return void
 */
void gps_Start()
{
	if (ublox_isON())
	{
		wip_debug("\nGPS already on.\n");
	}
	else
	{
		wip_debug("\nGPS starting...\n");
		ublox_Start();
	}
}


/** @brief Stop GPS operation
 *
 * @return void
 */
void gps_Stop(void)
{
	ublox_stop();
}


/** @brief Handler for AT&NMEA commands
 *
 * @param cmdParam
 * @return void
 */
void CmdHandlerNMEA(adl_atCmdPreParser_t *cmdParam)
{
	wip_debug("Cmd handler NMEA\r\n");
	switch (cmdParam->Type)
	{
	case ADL_CMD_TYPE_PARA:
		if (strncmp("on", ADL_GET_PARAM(cmdParam, 0), 2) == 0)
		{
			ublox_ShowData(TRUE);
			break;
		}
		if (strncmp("off", ADL_GET_PARAM(cmdParam, 0), 3) == 0)
		{
			ublox_ShowData(FALSE);
			break;
		}
	}
	adl_atSendResponse(ADL_AT_RSP, "\n");
}


/** @brief Handler for AT&GPSON commands
 *
 * @param cmdParam
 * @return void
 */
void CmdHandlerGPSON(adl_atCmdPreParser_t *cmdParam)
{
	(void)cmdParam;

	if (ublox_isON())
	{
		wip_debug(" GPS already on!\n");
	}
	else
	{
		ublox_Start();
	}
	adl_atSendResponse(ADL_AT_RSP, "\n");
}


/** @brief Handler for AT&GPSOFF commands
 *
 * @param cmdParam
 * @return void
 */
void CmdHandlerGPSOFF(adl_atCmdPreParser_t *cmdParam)
{
	(void)cmdParam;

	if (ublox_isON())
	{
		gps_Stop();
	}
	else
	{
		wip_debug(" GPS already off!\n");
	}
	adl_atSendResponse(ADL_AT_RSP, "\n");
}


#define GGA_HDOP                9
#define GGA_FIX_STATUS          7
#define GGA_SATS_USED           8
#define GGA_MAX_FIELD_SIZE      20

/** @brief process NMEA GGA NMEA strings
 *
 * @param nmeaSentence
 * @return void
 */
static void processNmeaGGA(unsigned char *nmeaSentence)
{
	unsigned char theField[GGA_MAX_FIELD_SIZE];
	double HDOPFloat = 0.0;

	util_GetStringField(nmeaSentence, GGA_FIX_STATUS, GGA_MAX_FIELD_SIZE, theField);

	if (theField[0] != '0')
	{
		GotGGAFix = ACTUAL_FIX;
		util_GetStringField(nmeaSentence, GGA_HDOP, GGA_MAX_FIELD_SIZE, gps.hdop);
		wm_sprintf(g_traceBuf, "HDOP = %s\r\n", gps.hdop);
		DumpMessage(g_traceBuf);
		sscanf((char *)gps.hdop, "%lf", &HDOPFloat);
		gGGAInfo.HDOP = HDOPFloat * 10;

		wm_sprintf(g_traceBuf, "GGA HDOP=%d\r\n", gGGAInfo.HDOP);
		DumpMessage(g_traceBuf);

		util_GetStringField(nmeaSentence, GGA_SATS_USED, GGA_MAX_FIELD_SIZE, theField);
		gGGAInfo.NumberOfSatillites = atoi((char *)theField);
		wm_sprintf(g_traceBuf, "GGA SATS=%d\r\n", gGGAInfo.NumberOfSatillites);
		DumpMessage(g_traceBuf);
	}
	else
	{
		GotGGAFix = NO_FIX;
	}
}


#define SNR_0                   8
#define SNR_1                   12
#define SNR_2                   16
#define SNR_3                   20
#define GSV_MAX_FIELD_SIZE      20

static void processNmeaGSV(unsigned char *nmeaSentence)
{
	unsigned char theField[GSV_MAX_FIELD_SIZE + 1];
	int satSNR[4];
	int ii;

	VISIBLE_SATS sats_visible_this_nmea = NO_SAT_IN_VIEW;

	for (ii = 0; ii < 4; ii++)
	{
		satSNR[ii] = 0;
	}

	if (util_NumberOfFields(nmeaSentence) >= SNR_0)
	{
		if (util_GetStringField(nmeaSentence, SNR_0, GSV_MAX_FIELD_SIZE, theField))
		{
			satSNR[0] = atoi((char *)theField);
		}
	}
	if (util_NumberOfFields(nmeaSentence) >= SNR_1)
	{
		if (util_GetStringField(nmeaSentence, SNR_1, GSV_MAX_FIELD_SIZE, theField))
		{
			satSNR[1] = atoi((char *)theField);
		}
	}
	if (util_NumberOfFields(nmeaSentence) >= SNR_2)
	{
		if (util_GetStringField(nmeaSentence, SNR_2, GSV_MAX_FIELD_SIZE, theField))
		{
			satSNR[2] = atoi((char *)theField);
		}
	}
	if (util_NumberOfFields(nmeaSentence) >= SNR_3)
	{
		if (util_GetStringField(nmeaSentence, SNR_3, GSV_MAX_FIELD_SIZE, theField))
		{
			satSNR[3] = atoi((char *)theField);
		}
	}

	for (ii = 0; ii < 4; ii++)
	{
		if (satSNR[ii] > 0)
		{
			//      wm_sprintf(g_traceBuf,"SNR=%d\r\n",satSNR[ii]);
			//      DumpMessage(g_traceBuf);
			sats_visible_this_nmea = SAT_IN_VIEW;
		}

		if (satSNR[ii] >= snr_struct.SNRHigh)
		{
			snr_struct.SNRLow = snr_struct.SNRHigh;
			snr_struct.SNRHigh = satSNR[ii];
		}
		else if (satSNR[ii] > snr_struct.SNRLow)
		{
			snr_struct.SNRLow = satSNR[ii];
		}
	}

	set_visible_sats(sats_visible_this_nmea);
}


static void ClearSNRs(void)
{
	snr_struct.SNRLow = 0;
	snr_struct.SNRHigh = 0;
}


/** @brief Process RMC NMEA strings
 *
 * @param nmeaSentence
 * @return void
 */
static void processNmeaRMC(unsigned char *nmeaSentence)
{
	unsigned char theField[RMC_MAX_FIELD_SIZE + 1];
	double Lat = 0.0, Long = 0.0;
	double Degrees = 0.0, Minutes = 0.0;
	s32 iMinutes = 0;
	double Course = 0.0;
	double Speed = 0.0;

	int data_int;
	int data_1000000;

	// process as needed for this application
	if (util_NumberOfFields(nmeaSentence) < RMC_MIN_FIELDS)
	{
		return;
	}

	// save it to our struct for external reference by app
	//		strcpy((char*)temp_string,(char*)nmeaSentence);
	util_GetStringField(nmeaSentence, RMC_FIX_STATUS, RMC_MAX_FIELD_SIZE, theField);

	if ((theField[0] == ACTUAL_FIX) && (GotGGAFix == ACTUAL_FIX))
	{
		GotGGAFix = NO_FIX;

		// read in all the data we are keeping track of...
		util_GetStringField(nmeaSentence, RMC_LATITUDE, RMC_MAX_FIELD_SIZE, gps.latitude);
		util_GetStringField(nmeaSentence, RMC_LONGITUDE, RMC_MAX_FIELD_SIZE, gps.longitude);
		wm_sprintf(g_traceBuf, "NMEA sentence = %s\r\n", nmeaSentence);
		DumpMessage(g_traceBuf);
		util_GetStringField(nmeaSentence, RMC_SPEED, RMC_MAX_FIELD_SIZE, gps.speed);
		util_GetStringField(nmeaSentence, RMC_HEADING, RMC_MAX_FIELD_SIZE, gps.heading);
		util_GetStringField(nmeaSentence, RMC_UTC_TIME, RMC_MAX_FIELD_SIZE, gps.utcTime);
		util_GetStringField(nmeaSentence, RMC_DATE, RMC_MAX_FIELD_SIZE, gps.date);
		util_GetStringField(nmeaSentence, RMC_N_OR_S, RMC_MAX_FIELD_SIZE, gps.northOrSouth);
		util_GetStringField(nmeaSentence, RMC_W_OR_E, RMC_MAX_FIELD_SIZE, gps.eastOrWest);

		sscanf((char *)gps.latitude, "%lf", &Lat);
		sscanf((char *)gps.longitude, "%lf", &Long);

		data_int = (int)(Lat);
		data_1000000 = (Lat - data_int) * 1000000;
		wm_sprintf(g_traceBuf, "<Lat>%d.%d\r\n", data_int, data_1000000);
		DumpMessage(g_traceBuf);
		data_int = (int)(Long);
		data_1000000 = (Long - data_int) * 1000000;
		wm_sprintf(g_traceBuf, "<Long>%d.%d\r\n", data_int, data_1000000);
		DumpMessage(g_traceBuf);

		// alright, here comes the old code now ...
		// so here is what's going on. The reference design for eRide parsed GGA and VTG
		// strings to get the needed information. This design was ported to u-blox
		// and the e-ride nmea parsing routines stopped working. This code (above) exisited for
		// the u-blox to parse out the RMC string and it seems that all the needed data is in there.
		// so now we will stuff the old GGA and VTG data structures with the data coming from the
		// RMC parser.

		// first we will convert the Latitude ddmm.mmmm
		Degrees = Lat / 100.0;
		Minutes = Lat / 100.0 - (s32)Degrees;
		gGGAInfo.Latitude = (s32)Degrees * 1000000;

		Minutes *= 100;                                 // multiply by 100 to restore minutes section.
		Minutes /= 60;                                  // now divide by 60 to convert from minutes to decimal of a degree
		iMinutes = (s32)(Minutes * 1000000.0);          // shift decimal up

		gGGAInfo.Latitude += iMinutes;

		// now check on N or S Hemi
		if (gps.northOrSouth[0] == 'S')
		{
			// if in southern hemi mulitply by -1
			gGGAInfo.Latitude *= -1;
		}

		// now convert the Latitude (note its slightly different) dddmm.mmm
		Degrees = Long / 100.0;
		Minutes = Long / 100.0 - (s32)Degrees;
		gGGAInfo.Longitude = (s32)Degrees * 1000000;

		Minutes *= 100;         // multiply by 100 to restore minutes section.
		Minutes /= 60;          // now divide by 60 to convert from minutes to decimal of a degree
		iMinutes = (s32)(Minutes * 1000000.0);

		gGGAInfo.Longitude += iMinutes;

		// now check on E or W Hemi
		if (gps.eastOrWest[0] == 'W')
		{
			// if in western hemi mulitply by -1
			gGGAInfo.Longitude *= -1;
		}

		wm_sprintf(g_traceBuf, "----\r\nLat = %d\r\nLong=%d\r\n---", (int)gGGAInfo.Latitude, (int)gGGAInfo.Longitude);
		DumpMessage(g_traceBuf);

		// here is how to convert the speed.
		wm_sprintf(g_traceBuf, "SPEED STRING = %s\r\n", gps.speed);
		DumpMessage(g_traceBuf);
		// Scan in the variables.
		sscanf((char *)gps.heading, "%lf", &Course);
		sscanf((char *)gps.speed, "%lf", &Speed);

		Speed = Speed * 1.852;     // convert to km/hr

		wm_sprintf(g_traceBuf, "SPEED*1000 = %d\r\n", (int)(Speed * 1000));
		DumpMessage(g_traceBuf);
		FilterSpeed(&Speed);
		wm_sprintf(g_traceBuf, "SPEED FILTER*1000 = %d\r\n", (int)(Speed * 1000));
		DumpMessage(g_traceBuf);

		gVTGInfo.Course = Course;
		wm_sprintf(g_traceBuf, "COURSE STRING = %s\r\n", gps.heading);
		DumpMessage(g_traceBuf);
		wm_sprintf(g_traceBuf, "COURSE = %d\r\n", (int)gVTGInfo.Course);
		DumpMessage(g_traceBuf);

		gVTGInfo.Speed = Speed * 1000;

		// 2D/3D handling here.
		if (gGGAInfo.NumberOfSatillites > 3)
		{
			gGGAInfo.FixQuality = FIX_3D;
			DumpMessage("GOT 3D FIX!\r\n");
			SetGPSLedStatus(FIX_3D_LED);
		}
		else
		{
			gGGAInfo.FixQuality = FIX_2D;
			DumpMessage("GOT 2D FIX!\r\n");
			SetGPSLedStatus(FIX_2D_LED);
		}

		// calculate the time.
		adl_rtcTime_t temp_time;
		temp_time.Hour = (gps.utcTime[0] - 0x30) * 10 + (gps.utcTime[1] - 0x30);
		temp_time.Minute = (gps.utcTime[2] - 0x30) * 10 + (gps.utcTime[3] - 0x30);
		temp_time.Second = (gps.utcTime[4] - 0x30) * 10 + (gps.utcTime[5] - 0x30);
		temp_time.Year = 2000 + (gps.date[4] - 0x30) * 10 + (gps.date[5] - 0x30);
		temp_time.Month = (gps.date[2] - 0x30) * 10 + (gps.date[3] - 0x30);
		temp_time.Day = (gps.date[0] - 0x30) * 10 + (gps.date[1] - 0x30);
		adl_rtcTimeStamp_t temp_timestamp;
		if (OK != adl_rtcConvertTime(&temp_time, &temp_timestamp, ADL_RTC_CONVERT_TO_TIMESTAMP))
		{
			DumpMessage("Failed to convert UTC time from fix\r\n");
			gGGAInfo.Time = 0;
		}
		else
		{
			gGGAInfo.Time = temp_timestamp.TimeStamp;
		}
	}
	else
	{
		if (get_visible_sats() == SAT_IN_VIEW)
		{
			SetGPSLedStatus(SATS_VISIBLE_LED);
		}
		else
		{
			SetGPSLedStatus(FIX_BAD_LED);
		}
		gGGAInfo.FixQuality = NO_FIX;
		//gps.currentlyFixed = FALSE;
		// if we are not fixed, we don't want to save the other data
	}
}


/** @brief count number of delimited string fields
 *
 * @par determines number of fields in comma delimited string
 *
 * @param sentence
 * @return fields
 */
int util_NumberOfFields(unsigned char *sentence)
{
	int charIndex = 0;
	char theChar;
	int fields = 1;         // until null pointer, we'll always have one field, even if empty

	if (sentence == NULL)
	{
		//		LOG(LOG_ERROR,__FUNCTION__," Errror! NULL string received.");
		return SUCCESS;
	}
	else if (sentence[0] == NULL_CHARACTER)
	{
		return SUCCESS;
	}

	// else we have fields...let's go get them
	do
	{
		theChar = sentence[charIndex++];
		if (theChar == ',')
		{
			fields++;
		}
	} while ((theChar != NULL_CHARACTER) && (theChar != '\n') && (theChar != '\r'));

	//LOG(LOG_INFO,__FUNCTION__," %d fields in: %s",fields,sentence);

	return fields;
}


/** @brief get string field
 *
 * @par retrieves string field from comma deliminted string
 *
 * @param sentence
 * @param fieldIndex
 * @param maxFieldSize
 * @param field
 * @return TRUE if string found, FALSE otherwise
 */
BOOL util_GetStringField(unsigned char *sentence
                         , int fieldIndex, int maxFieldSize, unsigned char *field)
{
	unsigned char sentenceCopy[MAX_STR_LEN_TO_PARSE];
	unsigned char   *pch;
	int fieldCounter = 1;
	int charIndex = 0;

	//LOG(LOG_INFO,__FUNCTION__,"Splitting string: %s",sentence);

	if ((sentence == NULL) || (field == NULL) || (fieldIndex <= 0) || (maxFieldSize < 1))
	{
		//		LOG(LOG_ERROR,__FUNCTION__," Errror! Invalid field index or size or null pointer sent to getStringField.");
		return FALSE;
	}

	// intialize field to NULL string and length to zero
	field[0] = NULL_CHARACTER;

	if (strlen((char *)sentence) == 0)
	{
		//		LOG(LOG_WARN,__FUNCTION__,"Null string received.");
		sentenceCopy[0] = NULL_CHARACTER;
	}
	else
	{
		pch = sentence;

		// first let's copy it to a local buffer and add spaces for missing fields and get rid of CR and LF
		do
		{
			if ((charIndex == 0) && (pch[0] == ','))
			{
				// this is the first parameter and it is empty, so insert a space
				sentenceCopy[charIndex++] = ' ';
			}

			// let's get rid of the quotes and spaces from the original...
			if ((pch[0] != '"') && (pch[0] != ' '))
			{
				sentenceCopy[charIndex++] = pch[0];
			}

			pch++;

			if ((sentenceCopy[charIndex - 1] == ',') && (pch[0] == ','))
			{
				// there are two commas next to each other--add space
				sentenceCopy[charIndex++] = ' ';
			}
		} while ((pch[0] != NULL_CHARACTER) && (pch[0] != '\n') && (pch[0] != '\r'));

		// if the last actual character was a comma, then there really is another field...
		if (sentenceCopy[charIndex - 1] == ',')
		{
			sentenceCopy[charIndex++] = ' ';
		}

		// we still need to terminate this new string
		sentenceCopy[charIndex] = NULL_CHARACTER;

		//LOG(LOG_INFO,__FUNCTION__,"Splitting new string \"%s\" into tokens:",sentenceCopy);
	}

	pch = (unsigned char *)(strtok((char *)sentenceCopy, ","));

	while ((fieldCounter != fieldIndex) && (pch != NULL))
	{
		pch = (unsigned char *)(strtok((char *)NULL, ","));
		fieldCounter++;
	}

	if (pch != NULL)
	{
		// we have a field value to report
		if (strncmp((char *)pch, " ", strlen((char *)pch)) == 0)
		{
			//LOG(LOG_INFO,__FUNCTION__," Field %d is empty.",fieldIndex);
			// keep field set to null char
		}
		else
		{
			// move these chars to the requested field
			charIndex = 0;
			do
			{
				field[charIndex] = pch[charIndex];
				charIndex++;
			} while ((charIndex < (maxFieldSize - 1)) && (pch[charIndex] != ',') && (pch[charIndex] != NULL_CHARACTER));
			field[charIndex] = NULL_CHARACTER;
			//LOG(LOG_INFO,__FUNCTION__," field %d is: %s  length of %d ",fieldIndex, field, strlen((char*)field));
		}
		return TRUE;
	}
	else
	{
		//LOG(LOG_WARN,__FUNCTION__," There is no field %d",fieldIndex);
		return FALSE;
	}
}


/** @brief determine if all digits present
 *
 * @param theString
 * @return TRUE if all digits present else FALSE
 */
BOOL util_IsAllDigits(unsigned char *theString)
{
	int index = 0;
	unsigned char theDigit;

	// there must be at least one digit
	if ((theString != NULL) && (strlen((char *)(theString)) > 0))
	{
		do
		{
			theDigit = theString[index++];
			if (theDigit != NULL_CHARACTER)
			{
				if ((theDigit < '0') || (theDigit > '9'))
				{
					return FALSE;
				}
			}
		} while (theDigit != NULL_CHARACTER);

		return TRUE;
	}
	return FALSE;
}


#define MAX_SPEED       225.0   // max speed in km/hr
#define MAX_DELTA       48.0    // max change in speed in 1 second.

void FilterSpeed(double *Speed)
{
	adl_rtcTime_t CurrentTime;
	s32 sReturn = -1;
	adl_rtcTimeStamp_t TimeStamp, DeltaTimeStamp;
	int max_delta_velocity = 0;
	static double PrevSpeed = MAX_SPEED + 1.0;
	static adl_rtcTimeStamp_t PrevTimeStamp;
	int time_stamp_seconds = 0;

	// get the current time.
	if ((sReturn = adl_rtcGetTime(&CurrentTime)) < 0)
	{
		DisplayErrorCode("adl_rtcGetTime", __FILE__, __LINE__, sReturn);
	}

	// convert to time stamp.
	if ((sReturn = adl_rtcConvertTime(&CurrentTime, &TimeStamp, ADL_RTC_CONVERT_TO_TIMESTAMP)) < 0)
	{
		DisplayErrorCode("adl_rtcConvertTime", __FILE__, __LINE__, sReturn);
	}

	// First time we have evaluated the speed.
	if (PrevSpeed == (MAX_SPEED + 1.0))
	{
		// save off previous time stamp.
		memcpy(&PrevTimeStamp, &TimeStamp, sizeof (adl_rtcTimeStamp_t));
		if (*Speed > MAX_DELTA)
		{
			*Speed = MAX_DELTA;
		}
		PrevSpeed = *Speed;
		return;
	}

	// check for unrealisitic acceleration.
	if (adl_rtcDiffTime(&TimeStamp, &PrevTimeStamp, &DeltaTimeStamp) < 0)
	{
		DisplayErrorCode("adl_rtcConvertTime", __FILE__, __LINE__, -1);
	}

	if (DeltaTimeStamp.TimeStamp == 0)
	{
		time_stamp_seconds = 1;
	}
	else
	{
		time_stamp_seconds = DeltaTimeStamp.TimeStamp;
	}

	max_delta_velocity = (time_stamp_seconds * MAX_DELTA);

	if (*Speed > PrevSpeed)
	{
		if ((*Speed - PrevSpeed) > max_delta_velocity)
		{
			*Speed = PrevSpeed + max_delta_velocity;
		}
	}

	if (*Speed > MAX_SPEED)
	{
		*Speed = MAX_SPEED;
	}

	memcpy(&PrevTimeStamp, &TimeStamp, sizeof (adl_rtcTimeStamp_t));
	PrevSpeed = *Speed;
}


#define MAX_NO_SAT_STRINGS    4

static void set_visible_sats(VISIBLE_SATS vs)
{
	static int no_sat_count = 0;

	// if we see any non zero SNR enable sats visible.
	if (vs == SAT_IN_VIEW)
	{
		visible_sats = vs;
		no_sat_count = 0;
		return;
	}

	// if we don't see any sats but we previously saw some sats,
	// wait MAX_NO_SAT_STRINGS before disabling sats visible.
	if ((visible_sats == SAT_IN_VIEW) && (vs == NO_SAT_IN_VIEW))
	{
		if (no_sat_count > MAX_NO_SAT_STRINGS)
		{
			visible_sats = vs;
		}
		else
		{
			no_sat_count++;
		}
	}
}


static VISIBLE_SATS get_visible_sats(void)
{
	return visible_sats;
}


/*@}*/
