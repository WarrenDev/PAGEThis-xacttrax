/** @addtogroup GPS
 * @{ */

/*H************************************************************************
 */

/** @file    GPSCtrl.c
 *
 *   @brief   Controls the GPS.
 *
 *   @details
 *
 *   @note    Other help for the reader, including references.
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

#include <adl_global.h>
#include "ConfigSettings.h"
#include "status.h"
#include "GpsInterface.h"
#include "GPSCtrl.h"
#include "fence.h"
#include "gps.h"
#include "VibrationMotor.h"
#include "anolclient.h"
#include "WaypointControl.h"
#include "XactUtilities.h"
#include "LED.h"
#include "alarm_suppress.h"
#include "dogwalk.h"
#include "accelerometer.h"
#include "ota_at.h"
#include "GPRS.h"

GPS_POS g_current_fix;
COORD g_current_fix_32;
BOOL GPSWaitTimeout = FALSE;

extern GEOFENCE g_DogParkFence;

extern TRACKING_MODE_REQ AccelTrackingReq;

extern TRACKING_MODE_REQ TimedWakeupTrackingReq;
extern s8 ubloxDataFcmHandle;
extern AGPS_DATA g_AGPSData;

TRACKING_MODE TrackingMode = TRACKING_MODE_OFF;

/** @enum GPS_STATE
 */
typedef enum
{
	GPS_OFF,        /**< current GPS state is off */
	GPS_START,      /**< GPS is now started */
	GPS_GET_FIX,    /**< GPS is trying to get fix */
	GPS_PROC_FIX,   /**< GPS Process currently fixed */
	GPS_WAIT_NEXT_FIX, /**< GPS waiting for next fix */
} GPS_STATE;

static GPS_STATE gps_state = GPS_OFF;

static adl_rtcTimeStamp_t FixTimeStamp;
static adl_rtcTimeStamp_t TrackTimeStamp;

static int alarmGenerated = 0; /*<! inidcates if an alarm has been generated for this loss */

struct structGGAInformation g_GGAInfo;
struct structVTGInformation g_VTGInfo;

static void ProcFix(FIX_QUALITY fix_quality);
static void CheckCoord(void);

static u8 FirstDogParkFix = 1;

static int HadFix = 0;
static int AttemptedAGPSNoFix = 0;

static POWER_CYCLE_FIX_STATUS power_cycle_fix_status = NEVER_HAD_FIX;

/** @brief Display current GPS States
 *
 * @return void
 */

bool enable_ota38= false;
void ConfigSettings_setOTA38(bool ota38)
{
  enable_ota38 = ota38;
}

void DisplayGPSStates(void)
{
	ascii TxString[100];
	memset(TxString, '\0', sizeof(TxString));
	
	switch (gps_state)
	{
	case GPS_OFF:
		DumpMessage("GPS STATE = GPS_OFF\r\n");
		strcpy(TxString, "GPS STATE = GPS_OFF\r\n");
		//if(enable_ota38) send_ota_response("GPS STATE = GPS_OFF\r\n");
		break;

	case GPS_START:
		DumpMessage("GPS STATE = GPS_START\r\n");
		strcpy(TxString, "GPS STATE = GPS_START\r\n");
		//if(enable_ota38) send_ota_response("GPS STATE = GPS_START\r\n");		
		break;

	case GPS_GET_FIX:
		DumpMessage("GPS STATE = GPS_GET_FIX\r\n");
		strcpy(TxString, "GPS STATE = GPS_GET_FIX\r\n");
		//if(enable_ota38) send_ota_response("GPS STATE = GPS_GET_FIX\r\n");
		break;

	case GPS_PROC_FIX:
		DumpMessage("GPS STATE = GPS_PROC_FIX\r\n");
		strcpy(TxString, "GPS STATE = GPS_PROC_FIX\r\n");
		//if(enable_ota38) send_ota_response("GPS STATE = GPS_PROC_FIX\r\n");
		break;

	case GPS_WAIT_NEXT_FIX:
		DumpMessage("GPS STATE = WAIT_NEXT_FIX\r\n");
		strcpy(TxString, "GPS STATE = WAIT_NEXT_FIX\r\n");
		//if(enable_ota38) send_ota_response("GPS STATE = WAIT_NEXT_FIX\r\n");
		break;
	}

	switch (TrackingMode)
	{
	case TRACKING_MODE_OFF:
		DumpMessage("Tracking Mode = TRACKING_MODE_OFF\r\n");
		strcat(TxString, "Tracking Mode = TRACKING_MODE_OFF\r\n");
		//if(enable_ota38) send_ota_response("Tracking Mode = TRACKING_MODE_OFF\r\n");
		break;

	case ACCEL_TRIG_TRACKING_MODE:
		DumpMessage("Tracking Mode = ACCEL_TRIG_TRACKING_MODE\r\n");		
		strcat(TxString, "Tracking Mode = ACCEL_TRIG_TRACKING_MODE\r\n");
		//if(enable_ota38) send_ota_response("Tracking Mode = ACCEL_TRIG_TRACKING_MODE\r\n");
		break;

	case SERVER_TRIG_TRACKING_MODE:
		DumpMessage("Tracking Mode = SERVER_TRIG_TRACKING_MODE\r\n");	
		strcat(TxString, "Tracking Mode = SERVER_TRIG_TRACKING_MODE\r\n");
		//if(enable_ota38) send_ota_response("Tracking Mode = SERVER_TRIG_TRACKING_MODE\r\n");
		break;

	case DOG_PARK_TRACKING_MODE:
		DumpMessage("Tracking Mode = DOG_PARK_TRACKING_MODE\r\n");
		strcat(TxString, "Tracking Mode = DOG_PARK_TRACKING_MODE\r\n");
		//if(enable_ota38) send_ota_response("Tracking Mode = DOG_PARK_TRACKING_MODE\r\n");
		break;

	case TIMED_WAKE_TRACKING_MODE:
		DumpMessage("Tracking Mode = TIMED_WAKE_TRACKING_MODE\r\n");
		strcat(TxString, "Tracking Mode = TIMED_WAKE_TRACKING_MODE\r\n");
		//if(enable_ota38) send_ota_response("Tracking Mode = TIMED_WAKE_TRACKING_MODE\r\n");
		break;

	default:
		DumpMessage("UNKNOWN TRACKING MODE\r\n");
		strcat(TxString, "UNKNOWN TRACKING MODE\r\n");
		//if(enable_ota38) send_ota_response("UNKNOWN TRACKING MODE\r\n");
	}

	//Send response over the air..
	if(enable_ota38) send_ota_response(TxString);
}


/** @brief next fix handler
 * @param timerid
 * @param context
 *
 * @return void
 */
static u8 GPSAlarmGenerated = 0;
void NextFixHandler(u8 timerid, void *context)
{
	(void)timerid;
	(void)context;

	s32 sReturn = -1;

	char prevGPSstat;

	adl_rtcTimeStamp_t CurrentTimeStamp;
	adl_rtcTimeStamp_t DeltaTimeStamp;
	TCP_STATUS status;
	static BOOL AGPSRequested=0;
	static u8 Count=0;   //Count of how many times this function was entered.
	int ret_val;

	prevGPSstat = g_status.GPSStatus;
	TRACE((1, "Next GPS Fix timer triggered"));

	// control the tracking mode state. Process the tracking mode
	// requests from the rest of the system.
	switch (TrackingMode)
	{
	case TRACKING_MODE_OFF:
        if (GetDogParkMode() == DOGPARK_ENABLED)
		{
			TrackingMode = DOG_PARK_TRACKING_MODE;
		}
		else if (AccelTrackingReq == TRACKING_MODE_REQ_ACTIVE)
		{
			TrackingMode = ACCEL_TRIG_TRACKING_MODE;
		}

		else if (TimedWakeupTrackingReq == TRACKING_MODE_REQ_ACTIVE)
		{
			TrackingMode = TIMED_WAKE_TRACKING_MODE;
		}
		else
		{
			TrackingMode = TRACKING_MODE_OFF;
		}
		break;

	case ACCEL_TRIG_TRACKING_MODE:
        if (GetDogParkMode() == DOGPARK_ENABLED)
		{
			TrackingMode = DOG_PARK_TRACKING_MODE;
		}

		else if (AccelTrackingReq == TRACKING_MODE_REQ_RELEASE)
		{
			TrackingMode = TRACKING_MODE_OFF;
		}
		else
		{
			TrackingMode = ACCEL_TRIG_TRACKING_MODE;
		}
		break;

	case DOG_PARK_TRACKING_MODE:
        if (GetDogParkMode() == DOGPARK_DISABLED)
		{
			TrackingMode = TRACKING_MODE_OFF;
		}
		else
		{
			TrackingMode = DOG_PARK_TRACKING_MODE;
		}
		break;

	case TIMED_WAKE_TRACKING_MODE:
		if (TimedWakeupTrackingReq == TRACKING_MODE_REQ_RELEASE)
		{
			TrackingMode = TRACKING_MODE_OFF;
		}
		else
		{
			TrackingMode = TIMED_WAKE_TRACKING_MODE;
		}
		break;

	default:
		DumpMessage("Undefined tracking mode\r\n");
		TrackingMode = TRACKING_MODE_OFF;
		break;
	}

	switch (gps_state)
	{
	// the GPS should be in OFF mode while in this state.
	case GPS_OFF:
		if ((TrackingMode == ACCEL_TRIG_TRACKING_MODE) || (TrackingMode == DOG_PARK_TRACKING_MODE) ||
		    (TrackingMode == TIMED_WAKE_TRACKING_MODE))
		{
			GetConvertTime(&TrackTimeStamp);
			gps_state = GPS_START;
			DumpMessage("GPS START!\r\n");
			//Reset the flag
			AGPSRequested = 0;
		}
		else
		{
			gps_state = GPS_OFF;
		}
		break;

	// attempt to start up the GPS.
	case GPS_START:
		gps_Start();

		if (!HadFix && !AttemptedAGPSNoFix)
		{
			DumpMessage("Never had fix. Going to use AGPS!\r\n");
			//performAGPS(0.0, 0.0, 0.0, 0.0);
			AttemptedAGPSNoFix = 1;
		}
		else if (HadFix)
		{
			GetConvertTime(&CurrentTimeStamp);
			//Request AGPS is last fix is more than 2 hrs old.		
			if((CurrentTimeStamp.TimeStamp - g_AGPSData.LastReqTimeStamp) >= 2*60*60)
			{
				AGPSRequested = 0;
				wm_sprintf(g_traceBuf,"g_AGPSData.LastReqTimeStamp : 0x%x, CurrentTimeStamp: 0x%x\r\n",(unsigned int)g_AGPSData.LastReqTimeStamp,(unsigned int)CurrentTimeStamp.TimeStamp);
				DumpMessage(g_traceBuf);
				DumpMessageUSB(g_traceBuf,1);				
				DumpMessage("AGPS data is old. Request fresh data\r\n");
			}

//			if (adl_rtcDiffTime(&CurrentTimeStamp, &FixTimeStamp, &DeltaTimeStamp) < 0)
//			{
//				DisplayErrorCode("adl_rtcConvertTime", __FILE__, __LINE__, sReturn);
//			}
//			else
//			{
//				wm_sprintf(g_traceBuf,"FixTimeStamp : 0x%x, CurrentTimeStamp: 0x%x\r\n",(unsigned int)FixTimeStamp.TimeStamp,(unsigned int)CurrentTimeStamp.TimeStamp);
//				DumpMessage(g_traceBuf);
//				DumpMessageUSB(g_traceBuf,1);

				//if (DeltaTimeStamp.TimeStamp >= 60 * 60 * 2)
				//{
				//	DumpMessage("Over two hours since last fix, going to use AGPS!\r\n");
				//	performAGPS(0.0, 0.0, 0.0, 0.0);					
				//}
			//}
		}

		gps_state = GPS_GET_FIX;
		DumpMessage("GET FIX!\r\n");
		//}
		break;

	// keep on the GPS until we get a fix. probably add a timeout to this...
	case GPS_GET_FIX:

		//Increment the count to keep a track of how many times this state was entered.
		Count++;
		
		// if tracking mode has been disabled...
		if (TrackingMode == TRACKING_MODE_OFF)
		{
			gps_state = GPS_OFF;
			break;
		}

		GetGGAInfo(&g_GGAInfo);
		GetVGTInfo(&g_VTGInfo);

		//Request AGPS
		GetTCPStatus(&status);

		
		if(!AGPSRequested && status == TCP_CONNECT)
		{					
			//Get the current time
			GetConvertTime(&CurrentTimeStamp);	

			//Compare the times.
			if((CurrentTimeStamp.TimeStamp - g_AGPSData.LastReqTimeStamp) < 2*60*60)  //check for < 2 hours
			{
				wm_sprintf(g_traceBuf,"CurrentTimeStamp.TimeStamp: 0x%x, g_AGPSData.LastReqTimeStamp: 0x%x\r\n", 
										(unsigned int)CurrentTimeStamp.TimeStamp,(unsigned int)g_AGPSData.LastReqTimeStamp);
				DumpMessage(g_traceBuf);
				DumpMessageUSB(g_traceBuf,1);

				//Send the last stored AGPS data to ublox directly.
				if ((ret_val = adl_fcmSendData(ubloxDataFcmHandle, (unsigned char *)g_AGPSData.AGPSData, g_AGPSData.datalen)) != OK)
				{
					wip_debug("ERROR: Could not send data to ublox device: %d\r\n", ret_val);
					wm_sprintf(g_traceBuf,"ERROR: Could not send data to ublox device: %d\r\n", ret_val);
					DumpMessageUSB(g_traceBuf,1);						
					set_agps_status(AGPS_NOT_USED);
				}
				else
				{
					AGPSRequested = 1;
					wm_sprintf(g_traceBuf,"AGPS data sent to ublox (<2 hrs case): %ld bytes\r\n", g_AGPSData.datalen);
					DumpMessage(g_traceBuf);
					DumpMessageUSB(g_traceBuf,1);
				}
					
			}
		}

		// Check if we got a fix.
		if (g_GGAInfo.FixQuality == FIX_3D)
		{
			SetFixStatus(HAD_3D_FIX);

			alarmGenerated = 0;
			GetConvertTime(&FixTimeStamp);
			gps_state = GPS_PROC_FIX;

			if (g_config.WaypointInterval >= GPS_WAYPOINT_SHUTDOWN_INTERVAL)
			{
				gps_Stop();
			}
			// GPS working
			//g_status.GPSStatus = 'W';
			// clear alarm
			if (alarm_suppress_status(GPS_ALARM_SUP) == ALARM_EXPIRED)
			{
				g_status.GPSAlarm = 'N';
				GPSAlarmGenerated = 0;
			}
			gps_state = GPS_PROC_FIX;
			DumpMessage("GPS_PROC_FIX\r\n");
		}
		else
		{
			if (g_GGAInfo.FixQuality == FIX_2D)
			{
				DumpMessage("GOT 2D FIX -- will not use\r\n");
				
				if(status == TCP_CONNECT && !AGPSRequested)
				{					
					//Request AGPS with available 2D fix
					performAGPS((g_GGAInfo.Latitude/1000000.0), (g_GGAInfo.Longitude/1000000.0), 0.0, 0.0);
					AGPSRequested = 1;
						
				}
			}
			else     //this fucntion is called once every 925ms. so 16*925 = 15 secs
			{
				//Check for the validity of WaitTimeout
				if(g_AGPSData.WaitTimeout == 0 || g_AGPSData.WaitTimeout > 99)
					g_AGPSData.WaitTimeout = 15;    //Default to 15 if found out of range
					
				//If 2D fix not received in 15 secs then request AGPS with last valid 3D fix.				
				//Request AGPS with last GPS fix				
				if(Count > g_AGPSData.WaitTimeout && status == TCP_CONNECT && !AGPSRequested )
				{					
					DumpMessage("Request fresh AGPS data\r\n");
					performAGPS(0.0, 0.0, 0.0, 0.0);
					AGPSRequested = 1;
					Count=0;
				}
			}

			g_status.GPSStatus = 'N';
			if (prevGPSstat == 'W')
			{
				TRACE((1, "Lost GPS Signal"));
			}

			gps_state = GPS_GET_FIX;
		}
		break;

	case GPS_PROC_FIX:
		ProcFix(FIX_3D);
		// GPS working
		g_status.GPSStatus = 'W';
		DumpMessage("Got a fix1\r\n");
		adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_USB, "***GOT_FIX***\r\n");
		gps_state = GPS_WAIT_NEXT_FIX;
		HadFix = 1;
		break;

	case GPS_WAIT_NEXT_FIX:
		DumpMessage("GPS_WAIT_NEXT_FIX\r\n");
		GetConvertTime(&CurrentTimeStamp);

		// time difference between last fix and current time.
		if (adl_rtcDiffTime(&CurrentTimeStamp, &FixTimeStamp, &DeltaTimeStamp) < 0)
		{
			DisplayErrorCode("adl_rtcConvertTime", __FILE__, __LINE__, sReturn);
		}

		// If the tracking interval time has elapsed,
		// go and get another fix or if the trackind mode contoller has disabled tracking,
		// go into the off state.
		if (DeltaTimeStamp.TimeStamp >= g_config.WaypointInterval)
		{
			wm_sprintf(g_traceBuf, "currenttimestamp = 0x%x fix time stamp = 0x%x delta time stamp = 0x%x waypoint interval = %d\r\n", (unsigned int )CurrentTimeStamp.TimeStamp, (unsigned int )FixTimeStamp.TimeStamp, (unsigned int )DeltaTimeStamp.TimeStamp, (int )g_config.WaypointInterval);
			DumpMessage(g_traceBuf);
			gps_state = GPS_START;
		}
		else
		{
			if (TrackingMode == TRACKING_MODE_OFF)
			{
				gps_Stop();
				gps_state = GPS_OFF;
				FirstDogParkFix = 1;
			}
			else
			{
				gps_state = GPS_WAIT_NEXT_FIX;
			}
		}

		break;

	default:
		DumpMessage("Error: Unknown GPS state\r\n");
		break;
	}
}


/** @brief Process the 2D fix. Only called right before we go to sleep if we have no 3D fix.
 * Also will generate the GPS alarm if we go to sleep without getting a fix.
 *
 * @return void
 */
void Proc2DFix(void)
{
	if ((power_cycle_fix_status == NEVER_HAD_FIX) && (g_GGAInfo.FixQuality == FIX_2D))
	{
		ProcFix(FIX_2D);
	}
	else if (power_cycle_fix_status == NEVER_HAD_FIX)
	{
		// generate the GPS alert.
		if (((g_config.GPSAlert == GPS_ALERT_ON_OTA_AGPS_ON) ||
		     (g_config.GPSAlert == GPS_ALERT_ON_OTA_AGPS_OFF)) &&
		    (!GPSAlarmGenerated))
		{
			alarm_suppress_set_alarm_time(GPS_ALARM_SUP);
			g_status.GPSAlarm = 'Y';
			GPSAlarmGenerated = 1;
		}
	}
}


void SetFixStatus(POWER_CYCLE_FIX_STATUS status)
{
	power_cycle_fix_status = status;
}


/** @brief Process the GPS fix
 *
 * @return void
 */
static void ProcFix(FIX_QUALITY fix_quality)
{
	int sv, strong, weak;
	ascii buffer[8];
	ascii satBuf[1];
	double SpeedMPH = 0.0;
	static u8 OverSpeedAlarmGenerated = 0;
	adl_rtcTimeStamp_t GPSTimeStamp;
	adl_rtcTime_t GPSTime;
	static bool RTC_Update = 1;
	s32 sReturn=0;

	DumpMessage("processing proc fix\n\r");

	if ((TrackingMode == DOG_PARK_TRACKING_MODE) && FirstDogParkFix)
	{
		g_DogParkFence.posts[0].lat_32 = g_GGAInfo.Latitude;
		g_DogParkFence.posts[0].long_32 = g_GGAInfo.Longitude;
		// these are "dummy" posts since we only support >=2.
		// The dog park radius is handled in fence.c
		g_DogParkFence.posts[1].lat_32 = g_GGAInfo.Latitude;
		g_DogParkFence.posts[1].long_32 = g_GGAInfo.Longitude;
		Config_SetMode(DOG_PARK_MODE, 0, &g_DogParkFence, TRUE); // TODO add dog park fence.
		FirstDogParkFix = 0;
	}

	g_current_fix_32.lat_32 = g_GGAInfo.Latitude;
	g_current_fix_32.long_32 = g_GGAInfo.Longitude;

	// Signal strength.
	GetSV_SRN(&sv, &strong, &weak);
	g_status.GPSSignalStrength1 = '0' + strong;
	g_status.GPSSignalStrength2 = '0' + weak;

	wm_sprintf(g_traceBuf, "sig strong = %c sig weak = %c\r\n", g_status.GPSSignalStrength1, g_status.GPSSignalStrength2);
	DumpMessage(g_traceBuf);

	// Time.
	wm_itohexa((ascii *)&buffer, g_GGAInfo.Time, 8);
	memcpy(&g_current_fix.utc, &buffer, 8);

	if(RTC_Update == 1 && fix_quality == FIX_3D)
	{
		//Update RTC with GPS time only once on power up
		GPSTimeStamp.TimeStamp = g_GGAInfo.Time;
		GPSTimeStamp.SecondFracPart = 0;
		if ((sReturn = adl_rtcConvertTime(&GPSTime, &GPSTimeStamp, ADL_RTC_CONVERT_FROM_TIMESTAMP)) < 0)
		{
			DisplayErrorCode("adl_rtcConvertTime", __FILE__, __LINE__, sReturn);
		}	
		if(adl_rtcSetTime(&GPSTime) != OK)
			DumpMessage("Error setting RTC time\r\n");
		else
		{
			//Restart accelerometer timer, to avoid sudden slow idle mode, due to change in RTC time.
			//If RTC time jumps up suddenly and if the diff compared to old RTC is more than accel sleep duration, then
			//the device can enter slow idle mode immediately in Track Mode and Low Power full track mode.
			ActivateAccelOnPowerUp();
			wm_sprintf(g_traceBuf,"RTC time udpated with GPS value : 0x%x\r\n",(unsigned int)GPSTimeStamp.TimeStamp);
			DumpMessage(g_traceBuf);
			DumpMessageUSB(g_traceBuf,1);
		}
		RTC_Update = 0;
	}
	// Number of sats
	wm_itohexa((ascii *)&satBuf, g_GGAInfo.NumberOfSatillites, 1);
	memcpy(&g_status.NumBirds, &satBuf, 1);

	// lat and long
	wm_itohexa((ascii *)&buffer, g_GGAInfo.Latitude, 8);
	memcpy(&g_current_fix.lat, &buffer, 8);
	wm_itohexa((ascii *)&buffer, g_GGAInfo.Longitude, 8);
	memcpy(&g_current_fix.longi, &buffer, 8);
	// copy HDOP to where EPE used to be.
	wm_itohexa((ascii *)&buffer, g_GGAInfo.HDOP, 2);
	memcpy(&g_current_fix.epe, &buffer, 2);

	wm_sprintf(g_traceBuf, "HDOP: %c%c\r\n", buffer[1], buffer[0]);
	DumpMessage(g_traceBuf);

	// check the speed. The speed from VTGInfo is in km/hr
	// the speed from the config struct is in miles/hr
	SpeedMPH = (double)g_VTGInfo.Speed * KM_IN_MILES / METERS_IN_KM;

	if (!OverSpeedAlarmGenerated && (SpeedMPH >= (double)g_config.OverSpeedAlertThresh) && (g_config.OverSpeedAlertThresh != 0))
	{
		DumpMessage("over speed alarm detected\n\r");
		g_status.OverSpeedAlarm = 'Y';
		OverSpeedAlarmGenerated = 1;
	}
	else if (SpeedMPH < (double)g_config.OverSpeedAlertThresh)
	{
		DumpMessage("over speed alarm NOT detected\n\r");
		g_status.OverSpeedAlarm = 'N';
		OverSpeedAlarmGenerated = 0;
	}

	// write the waypoint fix to serial flash.
	WriteWaypointFromFix(&g_current_fix, g_VTGInfo.Speed, g_VTGInfo.Course, fix_quality);

	// do not check on the first dog park fix.
	DumpMessage("\r\nCheckCoords\r\n");
	if (((!FirstDogParkFix) && (TrackingMode == DOG_PARK_TRACKING_MODE)) ||
	    (TrackingMode != DOG_PARK_TRACKING_MODE))
	{
		CheckCoord();
	}
}


/** @brief Convert a timestamp structure to seconds
 *
 * @par
 * This reads the RTC and stores it as a adl_rtcTimeStamp_t type instead of adl_rtcTime_t type.
 * @param TimeStamp
 * @return void
 */
int GetConvertTime(adl_rtcTimeStamp_t *TimeStamp)
{
	adl_rtcTime_t CurrentTime;
	s32 sReturn = -1;

	// get the current time.
	if ((sReturn = adl_rtcGetTime(&CurrentTime)) < 0)
	{
		DisplayErrorCode("adl_rtcGetTime", __FILE__, __LINE__, sReturn);
	}

	// convert to time stamp.
	if ((sReturn = adl_rtcConvertTime(&CurrentTime, TimeStamp, ADL_RTC_CONVERT_TO_TIMESTAMP)) < 0)
	{
		DisplayErrorCode("adl_rtcConvertTime", __FILE__, __LINE__, sReturn);
	}

	return sReturn;
}


/** @brief Check the current coordinate
 *
 * @par
 * This is what triggers the geofence system. Everytime we get a fix we need to check if we are violating any geofences.
 *
 * @return void
 */
static void CheckCoord(void)
{
	UINT8 current_fence_stat;
	int fence_num;

	adl_rtcTimeStamp_t TimeStamp0, TimeStamp1;
	adl_rtcTimeStamp_t DeltaTimeStamp;

	// If we are in dog walk mode,
    if (GetDogWalkMode() == DOGWALK_ENABLED)
	{
		DumpMessage("Dog walk mode\r\n");
		return;
	}

	GetConvertTime(&TimeStamp0);
	current_fence_stat = eval_fix(&fence_num, GPS_FIX_REASON);
	GetConvertTime(&TimeStamp1);

	wm_sprintf(g_traceBuf, "Fence Number = %d\r\n", fence_num);
	DumpMessage(g_traceBuf);

	if (adl_rtcDiffTime(&TimeStamp1, &TimeStamp0, &DeltaTimeStamp) < 0)
	{
		DisplayErrorCode("adl_rtcConvertTime", __FILE__, __LINE__, -1);
	}

	wm_sprintf(g_traceBuf, "Calc time %d:%d\r\n", (int )DeltaTimeStamp.TimeStamp, (int)ADL_RTC_GET_TIMESTAMP_MS(DeltaTimeStamp));
	DumpMessage(g_traceBuf);
	wm_sprintf(g_traceBuf, "current_fence_stat = %c\r\n", current_fence_stat);
	DumpMessage(g_traceBuf);
}


/** @brief Shut down the GPS
 *
 * @return void
 */
void ShutDownGPS(void)
{
	gps_Stop();
	g_status.GPSStatus = 'N';
	gps_state = GPS_OFF;
	SetGPSLedStatus(FIX_BAD_LED);
}

void GPSTimeoutHandler(u8 timerid, void *context)
{
	(void) timerid;
	(void) context;
	
	GPSWaitTimeout = TRUE;

}


/** @}*/
