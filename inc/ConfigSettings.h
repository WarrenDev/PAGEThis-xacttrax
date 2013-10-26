#ifndef __CONFIGSETTINGS__
#define __CONFIGSETTINGS__

#include "fence.h"
#include "gps.h"
#include "SMShandling.h"
#include "LED.h"

/*********** DEFINES ***********/

#define SECONDS_IN_DAY          86400
#define SECONDS_IN_HOUR         3600
#define SECONDS_IN_WEEK         604800
#define WAYPOINTS_PER_PKT       4
#define MAX_PHNUM_LEN           (15) /* Number of chars in a phone num (including null) */

#define NUM_DIAG_COMMANDS       10     

typedef enum
{
	NO_CHANGE_MODE,
	DOG_PARK_MODE,
	TRACK_MODE,
	WALK_MODE,
	SERVER_NO_SLEEP,
	SERVER_STATUS_REQ,
	SERVER_STATUS_REQ_V,
	FULL_TRACK_MODE,
	LP_FULL_TRACK_MODE,
	ALARM_MODE
} MODES;

typedef enum
{
	DISABLE_SOS,
	ENABLE_SOS,
	ACK_SOS
} SOS_CTL;

typedef enum
{
	NO_DOWNLOAD,
	HOUR1_DATA,
	HOUR8_DATA,
	HOUR24_DATA,
	HOUR72_DATA,
	WEEK1_DATA,
	ALL_DATA
} WAYPOINT_DL;

typedef enum
{
	SMS_ROAMING = 0,
	GPRS_UDP_ROAMING,
	GPRS_TCP_ROAMING,
	GPRS_UDP_FALLBACK_ROAMING,
	GPRS_TCP_FALLBACK_ROAMING,
	SMS_NO_ROAMING,
	GPRS_UDP_NO_ROAMING,
	GPRS_TCP_NO_ROAMING,
	GPRS_UDP_FALLBACK_NO_ROAMING,
	GPRS_TCP_FALLBACK_NO_ROAMING
} SMS_GPRS_MODE;

typedef enum
{
	GPS_ALERT_ON_OTA_AGPS_ON = 0,
	GPS_ALERT_ON_OTA_AGPS_OFF,
	GPS_ALERT_OFF_OTA_AGPS_ON,
	GPS_ALERT_OFF_OTA_AGPS_OFF
} GPS_ALERT_SETTING;

typedef enum
{
	BATT_ALERT_OFF_SHUTDOWN_ALERT_OFF = 0,
	BATT_LEVEL_1_SHUTDOWN_ALERT_OFF,
	BATT_LEVEL_2_SHUTDOWN_ALERT_OFF,
	BATT_LEVEL_1_SHUTDOWN_ALERT_ON,
	BATT_LEVEL_2_SHUTDOWN_ALERT_ON,
	BATT_ALERT_OFF_SHUTDOWN_ALERT_ON
} BATT_ALERT_SETTING;

/* Section of config packet structure.  See the Config Packet in the
 * protocol document for a description of the members */
typedef struct
{
	UINT8 SMSorGPRS;
	UINT32 WaypointInterval;
	UINT8 TrackingInterval;
	UINT8 MotionAlarmThresh;
	UINT8 AccelThreshWake;
	UINT32 AccelDurationWake;
	UINT32 AccelDurationSleep;
	ENDISABLE BreadCrumbMode;
	MODES Mode;
	UINT8 NumFences;
	GPS_ALERT_SETTING GPSAlert;
	UINT8 GSMAlertThresh;
	BATT_ALERT_SETTING BattAlert;
	ENDISABLE PowerDiscAlert;
	UINT8 OverSpeedAlertThresh;
	UINT8 FirmwareRevNum;
	UINT8 HardwareRevNum;
	ENDISABLE PowerDownDisable;
	UINT8 VibrationMotorPattern;
	ENDISABLE VibrationMotorDelayEn;
	ENDISABLE ServerVibMotorOverride;
	SOS_CTL SOSAlert;
	WAYPOINT_DL WaypointDLData;
	UINT8 LEDPattern;
	UINT8 LEDEn[NUM_LEDS];
	ENDISABLE ServerLEDOverride;
	UINT32 TimedWakeup;
	UINT8 FencesActive[NUM_FENCES];
	ENDISABLE InvisibleOp;
	ENDISABLE MagneticSnsEn;
	UINT8 CriticalConfirm;
	ascii ServerPhoneNum[MAX_PHNUM_LEN];
	ascii ServerIPAddr[15];
	UINT16 ServerPortNum;
	UINT32 TrackingModeDuration;
	BOOL ConfigPktRxProperly;
	BOOL MotionAlarmIndep;
} CONFIG;

extern CONFIG g_config;

typedef struct 
{
	UINT8 ATCmdRepeatCount;
	UINT8 ATCmdRepeatInterval;
	ascii ATCommand[20];   //Assuming 20 is max lenght.
	BOOL IsCommandValid;   //Required to determine number of valid commands on power up. 
}DIAG_CONFIG;

extern DIAG_CONFIG g_DiagConfig[];

typedef struct
{
	UINT8  UseCaseNum;
	UINT8  Mode;
	UINT16 AnalogThreshold;
	UINT8  SampleCount;
	UINT8  GPIO5Config;
	UINT8  GPIO7Config;
	UINT8  I2CDataLen;
	UINT8  TxPattern;
	BOOL   PktRxdProperly;
	UINT8  GPSTimeout;
}MODE_CONFIG;

typedef struct
{
	u32 LastReqTimeStamp;
	char AGPSData[5120];  //5K data
	u32 datalen;
	u8 WaitTimeout;
	u8 PowerOffTimeout;
}AGPS_DATA;

extern MODE_CONFIG g_ModeConfig;
/**** Function Prototypes **************/
extern int SetSMSorGPRS(UINT8 *value, CONFIG *configStruct);
extern int SetWaypointInterval(UINT8 *value, CONFIG *configStruct);
extern int SetTrackingInterval(UINT8 *value, CONFIG *configStruct);
extern int GetTrackingInterval();
extern int SetMotionAlarmThresh(UINT8 *value, CONFIG *configStruct);
extern int SetAccelerometerThreshWake(UINT8 *value, CONFIG *configStruct);
extern int SetAccelerometerDurWake(UINT8 *value, CONFIG *configStruct);
extern int SetAccelerometerDurSleep(UINT8 *value, CONFIG *configStruct);
extern int SetBreadCrumbMode(UINT8 *value, CONFIG *configStruct);
extern int SetMode(UINT8 *value, CONFIG *configStruct);
extern int SetFenceNumber(UINT8 *value, CONFIG *configStruct);
extern int SetGPSAlert(UINT8 *value, CONFIG *configStruct);
extern int SetGSMAlert(UINT8 *value, CONFIG *configStruct);
extern int SetBattAlert(UINT8 *value, CONFIG *configStruct);
extern int SetPowerDisconAlert(UINT8 *value, CONFIG *configStruct);
extern int SetOverSpeedAlertThresh(UINT8 *value, CONFIG *configStruct);
extern int SetPowerDownDisable(UINT8 *value, CONFIG *configStruct);
extern int SetVibrationMotorPattern(UINT8 *value, CONFIG *configStruct);
extern int SetVibrationMotorDelayEn(UINT8 *value, CONFIG *configStruct);
extern int SetSOSAlert(UINT8 *value, CONFIG *configStruct);
extern int SetWaypointDownload(UINT8 *value, CONFIG *configStruct);
extern int SetLEDPattern(UINT8 *value, CONFIG *configStruct);
extern int SetLEDEnable(UINT8 *value, CONFIG *configStruct);
extern int SetTimedWakeup(UINT8 *value, CONFIG *configStruct);
extern int SetInvisibleOp(UINT8 *value, CONFIG *configStruct);
extern int SetMagneticSNSEnable(UINT8 *value, CONFIG *configStruct);
extern int SetCriticalConfirmation(UINT8 *value, CONFIG *configStruct);
extern int SetTrackingModeDuration(UINT8 *value, CONFIG *configStruct);
extern int SetServerPhoneNum(UINT8 *value, CONFIG *configStruct);
extern int SetServerIPAddr(UINT8 *value, CONFIG *configStruct);
extern int SetServerPort(UINT8 *value, CONFIG *configStruct);
extern int SetFenceEnDisSettings(UINT8 *fence_settings, CONFIG *configStruct);
extern int WriteFenceCoord(ascii *num, CONFIG *configStruct, ascii *in_ex, ascii *lon1, ascii *lat1, ascii *lon2,
                           ascii *lat2, ascii *lon3, ascii *lat3, ascii *lon4, ascii *lat4, ascii *start_time, ascii *stop_time);

extern void SetCurrentFix(ascii *utc, ascii *lat, ascii *longi, ascii *epe);

extern void DisplayConfig(void);
void start_reset_timer(void);
void ConfigSettings_setOTA27(bool ota27);
void Config_SetMode(MODES mode, int duration, GEOFENCE *gf, BOOL write_flash);

#endif
