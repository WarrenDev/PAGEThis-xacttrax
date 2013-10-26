#ifndef __GPSCTRL_H__
#define __GPSCTRL_H__

#define GPS_WAYPOINT_SHUTDOWN_INTERVAL      60      /*!< the threshold for shutting down the GPS between fixes */
#define NUM_SECONDS_TO_TRACK                60 * 60 /*!< the number of seconds to stay in tracking mode if enabled by the server */

#define KM_IN_MILES                         0.6213  /*!< the number of km in a mile */
#define METERS_IN_KM                        1000

typedef enum
{
	TRACKING_MODE_REQ_ACTIVE,
	TRACKING_MODE_REQ_RELEASE
} TRACKING_MODE_REQ;

typedef enum
{
	TRACKING_MODE_OFF,
	ACCEL_TRIG_TRACKING_MODE,
	SERVER_TRIG_TRACKING_MODE,
	DOG_PARK_TRACKING_MODE,
	TIMED_WAKE_TRACKING_MODE
} TRACKING_MODE;

typedef enum
{
	NEVER_HAD_FIX,
	HAD_3D_FIX
} POWER_CYCLE_FIX_STATUS;

int GetConvertTime(adl_rtcTimeStamp_t *TimeStamp);
void DisplayGPSStates(void);
void ShutDownGPS(void);
void Proc2DFix(void);
void SetFixStatus(POWER_CYCLE_FIX_STATUS status);
void GPSTimeoutHandler(u8 timerid, void *context);
void ConfigSettings_setOTA38(bool ota38);


#endif
