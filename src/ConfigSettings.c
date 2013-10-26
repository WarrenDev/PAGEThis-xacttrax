/** @addtogroup NetworkComm
 *@{*/

/*H************************************************************************
 */

/** @file    ConfigSettings.c
 *
 *   @brief   This sets configuration settings.
 *
 *   @details Configuration settings are set from the at command interface or
 *            from the server.
 *
 *   @note
 *
 *
 *//*
 *
 *H*/

#include "adl_global.h"
#include "pistd.h"
#include "XactUtilities.h"
#include "ConfigSettings.h"
#include "fence.h"
#include "FlashTest.h"
#include "gps.h"
#include "wip.h"
#include "Accelerometer.h"
#include "GPRS.h"
#include "InternalFlash.h"
#include "ota_at.h"
#include "dogwalk.h"

#define DEBUG_CONFIG_SETTINGS

CONFIG g_config;
DIAG_CONFIG g_DiagConfig[NUM_DIAG_COMMANDS];
MODE_CONFIG g_ModeConfig;
AGPS_DATA g_AGPSData;


extern GEOFENCE g_GeoFences[NUM_FENCES];

extern int g_NeedSOSAck;
static int special_start_time = 0;
static int special_stop_time = 0;

extern SMS_GPRS_MODE prev_SMSorGPRSMode;
extern u8 PREV_Mode;

void start_invisible_timer(void);
void invisible_timer(u8 timerID, void *Context);
void DisplayFence(GEOFENCE *gf);

// Use this define to control whether or not the device
// will reset when it switches communication modes.
// resetting may cause it to switch modes faster.
#define RESET_WHEN_MODE_SWITCH    1

#ifdef RESET_WHEN_MODE_SWITCH
void reset_timer(u8 timerID, void *Context);
#endif

/** @brief Set the transmission mode.
 *
 * @param value
 * @param configStruct
 * @return 1 on success
 * @return -1 on failure
 */
int SetSMSorGPRS(UINT8 *value, CONFIG *configStruct)
{
	UINT8 val = *value - 0x30;

	switch (val)
	{
	case 0:
		configStruct->SMSorGPRS = SMS_ROAMING;
		break;

	case 1:
		configStruct->SMSorGPRS = GPRS_UDP_ROAMING;
		break;

	case 2:
		configStruct->SMSorGPRS = GPRS_TCP_ROAMING;
		break;

	case 3:
		configStruct->SMSorGPRS = GPRS_UDP_FALLBACK_ROAMING;
		break;

	case 4:
		configStruct->SMSorGPRS = GPRS_TCP_FALLBACK_ROAMING;
		break;

	case 5:
		configStruct->SMSorGPRS = SMS_NO_ROAMING;
		break;

	case 6:
		configStruct->SMSorGPRS = GPRS_UDP_NO_ROAMING;
		break;

	case 7:
		configStruct->SMSorGPRS = GPRS_TCP_NO_ROAMING;
		break;

	case 8:
		configStruct->SMSorGPRS = GPRS_UDP_FALLBACK_NO_ROAMING;
		break;

	case 9:
		configStruct->SMSorGPRS = GPRS_TCP_FALLBACK_NO_ROAMING;
		break;

	default:
		wm_sprintf(g_traceBuf, "***ERROR bad SMS or GPRS config: %d\r\n", val);
		DumpMessage(g_traceBuf);
		return -1;
	}

	// if we have switched communcation modes
	// we may need to reconnect the GPRS mode.
	wm_sprintf(g_traceBuf, "new mode = %d old mode = %d\r\n", val, prev_SMSorGPRSMode);
	DumpMessage(g_traceBuf);

	if (prev_SMSorGPRSMode != val)
	{
		DumpMessage("Switching communcation modes\r\n\r\n");

#ifdef RESET_WHEN_MODE_SWITCH
		DumpMessage("Going to reset due to mode switch\r\n");
		Status_setResetReason(RESET_REASON_COMM_MODE_CHANGE);
		start_reset_timer();
#endif

		ClearTCPStatus();
		StartGPRS();
	}
	else
	{
		DumpMessage("No mode switch!\r\n");
	}

	prev_SMSorGPRSMode = val;

	return 1;
}


/** @brief Set the waypoint interval.
 *
 * @param value
 * @param configStruct
 * @return 1 on success
 * @return -1 on failure
 */
int SetWaypointInterval(UINT8 *value, CONFIG *configStruct)
{
	UINT32 waypoint_interval = hex_ascii_2_uint(value, 12);

	// There are only certain valid settings.
	if ((waypoint_interval == 1) || (waypoint_interval == 2) || (waypoint_interval == 5) ||
	    (waypoint_interval == 10) || (waypoint_interval == 15) || (waypoint_interval == 30) ||
	    (waypoint_interval == 60) || (waypoint_interval == 120) || (waypoint_interval == 300) ||
	    (waypoint_interval == 600))
	{
		configStruct->WaypointInterval = waypoint_interval;
#ifdef DEBUG_CONFIG_SETTINGS
		wm_sprintf(g_traceBuf, "waypoint inverval = %d\r\n", (int)waypoint_interval);
		DumpMessage(g_traceBuf);
#endif
		return 1;
	}
	else
	{
		DumpMessage("Invalid waypoint interval\n");
		return -1;
	}
}


/** @brief Set the tracking interval from the config packet.
 *
 * @param value
 * @param configStruct
 * @return 1 on success
 * @return -1 on failure
 */
int SetTrackingInterval(UINT8 *value, CONFIG *configStruct)
{
	wm_sprintf(g_traceBuf, "Tracking Interval Value from Config: %c\r\n", *value);
	DumpMessage(g_traceBuf);

	if(*value == '0' || *value == 'Z')
	{
		configStruct->TrackingInterval = g_config.TrackingInterval;
		DumpMessage("Tracking Interval will remain unchanged\r\n");
	}
	else	
		configStruct->TrackingInterval = *value;
	

#ifdef DEBUG_CONFIG_SETTINGS
	wm_sprintf(g_traceBuf, "Tracking Interval setting will change to: %c\r\n", configStruct->TrackingInterval);
	DumpMessage(g_traceBuf);
#endif
	
	return 1;
}

/** @brief Get the tracking interval from the config packet.
 *
 * @param void
 * @return tracking interval value on success
 * @return -1 on failure
 */
int GetTrackingInterval()
{
	int tracking_interval=10*60;

	switch (g_config.TrackingInterval)
	{
	case '0':
		//Reserved field;
		break;

	case '1':
		tracking_interval = 5;
		break;

	case '2':
		tracking_interval = 10;
		break;

	case '3':
		tracking_interval = 15;
		break;

	case '4':
		tracking_interval = 30;
		break;

	case '5':
		tracking_interval = 45;
		break;

	case '6':
		tracking_interval = 1*60;
		break;
		
	case '7':
		tracking_interval = 2*60;
		break;
	
	case '8':
		tracking_interval = 5*60;
		break;
	
	case '9':
		tracking_interval = 10*60;
		break;
	
	case 'A':
		tracking_interval = 15*60;
		break;
	
	case 'B':
		tracking_interval = 30*60;
		break;
	
	case 'C':
		tracking_interval = 45*60;
		break;
		
	case 'D' ... 'W':
		//The values from D to W increase sequentially from 1 to 20. Hence we
		//can use this formula. If the ranges is modifiled please take care of 
		//the formula accordingly.
		tracking_interval = (g_config.TrackingInterval - 'C')*60*60;
		break;
		
	case 'X':
		tracking_interval = 22*60*60;  
		break;

	case 'Y':
		tracking_interval = 24*60*60;  
		break;
		
	case 'Z':
		//Reserved Field
		break;

	default:
		DumpMessage("\r\nInvalid Tracking Interval Request\r\n");
		return -1;
	}

#ifdef DEBUG_CONFIG_SETTINGS
	//wm_sprintf(g_traceBuf, "Tracking Interval is: %d Seconds\r\n", (int)tracking_interval);
	//DumpMessage(g_traceBuf);
#endif
    	
	return tracking_interval;
}

int SetMotionAlarmThresh(UINT8 *value, CONFIG *configStruct)
{
	UINT8 val = hex_ascii_2_uint(value, 8);

#ifdef DEBUG_CONFIG_SETTINGS
	wm_sprintf(g_traceBuf, "Motion alarm threshold setting will change to: %d \r\n", (int)val & 0x7f);
	DumpMessage(g_traceBuf);
#endif
	configStruct->MotionAlarmThresh = val & 0x7f;

    if(configStruct->MotionAlarmThresh != 0 && configStruct->MotionAlarmThresh < 17)
    {
		DumpMessage("Illegal MotionAlarmThresh Value, Should be 17 at the minimum\r\n");  
		return -1;
    }

	if (val & 0x80)
	{
		configStruct->MotionAlarmIndep = FALSE;
		wm_sprintf(g_traceBuf, "Motion alarm inp = FALSE\r\n");
	}
	else
	{
		configStruct->MotionAlarmIndep = TRUE;
		wm_sprintf(g_traceBuf, "Motion alarm inp = TRUE\r\n");
	}
	DumpMessage(g_traceBuf);
	return 1;
}


/** @brief Set the Accelerometer threshold from the config packet.
 *
 * @param value
 * @param configStruct
 * @return 1 on success
 */
int SetAccelerometerThreshWake(UINT8 *value, CONFIG *configStruct)
{
	UINT8 val = hex_ascii_2_uint(value, 8);

#ifdef DEBUG_CONFIG_SETTINGS
	wm_sprintf(g_traceBuf, "Accelerometer threshold wake setting will change to: %d \r\n", (int)val);
	DumpMessage(g_traceBuf);
#endif
	configStruct->AccelThreshWake = val;

	// write the acceleromter.
	if (SetAccelTriggerThreshold(val))
	{
		DumpMessage("Error writing acceleromter!\r\n");
		return -1;
	}

	return 1;
}


/** @brief set the Accelerometer duration wake from the config packet.
 *
 * @param value
 * @param configStruct
 * @return 1 on success
 * @return -1 on failure
 */
int SetAccelerometerDurWake(UINT8 *value, CONFIG *configStruct)
{
	UINT32 val = hex_ascii_2_uint(value, 16);
	if (val <= 65535)
	{
#ifdef DEBUG_CONFIG_SETTINGS
		wm_sprintf(g_traceBuf, "Accelerometer duration wake setting will change to: %d \r\n", (int)val);
		DumpMessage(g_traceBuf);
#endif
		configStruct->AccelDurationWake = val;
		return 1;
	}
	else
	{
		DumpMessage("\r\nInvalid accelerometer duration wake setting");
		return -1;
	}
}


/** @brief Set the amount of time before the device will go to sleep from the config packet.
 *
 * @param value
 * @param configStruct
 * @return 1 on success
 * @return -1 on failure
 */
int SetAccelerometerDurSleep(UINT8 *value, CONFIG *configStruct)
{
	UINT32 val = hex_ascii_2_uint(value, 16);
	if (val <= 65535)
	{
#ifdef DEBUG_CONFIG_SETTINGS
		wm_sprintf(g_traceBuf, "Accelerometer duration sleep setting will change to: %d \r\n", (int)val);
		DumpMessage(g_traceBuf);
#endif
		configStruct->AccelDurationSleep = val;
		return 1;
	}
	else
	{
		DumpMessage("\r\nInvalid accelerometer duration sleep setting");
		return -1;
	}
}


/** @brief Set the device's breadcrumb mode from the configuration packet.
 *
 * @param value
 * @param configStruct
 * @return 1 on success
 * @return -1 on failure
 */
int SetBreadCrumbMode(UINT8 *value, CONFIG *configStruct)
{
	u8 val = *value - 0x30;
	if (val <= 8)
	{
		switch (val)
		{
		case 0:
			configStruct->BreadCrumbMode = 0;
			special_start_time = 0;
			special_stop_time = 0;
			break;

		case 1:
			configStruct->BreadCrumbMode = 1;
			special_start_time = 0;
			special_stop_time = 0;
			break;

		case 2:
			configStruct->BreadCrumbMode = 0;
			special_start_time = 1;
			special_stop_time = 0;
			break;

		case 3:
			configStruct->BreadCrumbMode = 0;
			special_start_time = 0;
			special_stop_time = 1;
			break;

		case 4:
			configStruct->BreadCrumbMode = 0;
			special_start_time = 1;
			special_stop_time = 1;
			break;

		case 5:
			configStruct->BreadCrumbMode = 1;
			special_start_time = 1;
			special_stop_time = 0;
			break;

		case 6:
			configStruct->BreadCrumbMode = 1;
			special_start_time = 0;
			special_stop_time = 1;
			break;

		case 7:
			configStruct->BreadCrumbMode = 1;
			special_start_time = 1;
			special_stop_time = 1;
			break;

		default:
			return -1;
		}

		// old setting before we had to hack it up for latin chars.
		return 1;
	}
	else
	{
		DumpMessage("\r\nInvalid bread crumb mode setting");
		return -1;
	}
}


/** @brief Set the main mode for the system.
 *
 * @param value
 * @param configStruct
 * @return 1 on success
 * @return -1 on failure
 */
int SetMode(UINT8 *value, CONFIG *configStruct)
{
	switch (*value)
	{
	case 'W':
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("Mode will change to: Walk Mode \r\n");
#endif
		configStruct->Mode = WALK_MODE;
        SetDogParkMode(DOGPARK_DISABLED);
        SetDogWalkMode(DOGWALK_ENABLED);
		Config_SetMode(WALK_MODE, 0, NULL, TRUE);
		return 1;

	case 'T':
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("Mode will change to: Track Mode \r\n");
#endif
		configStruct->Mode = TRACK_MODE;
		Config_SetMode(TRACK_MODE, 0, NULL, TRUE);      // TODO add in duration
		return 1;

	case 'F':
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("Mode will change to: Full Track Mode (Track with sleep disabled)\r\n");
#endif
		configStruct->Mode = FULL_TRACK_MODE;
		//configStruct->FullTrackModeEnabled = TRUE;
		Config_SetMode(FULL_TRACK_MODE, 0, NULL, TRUE);		// TODO add in duration
		return 1;

	case 'L':
		//Low power Full track mode supported only if Track interval is more than 10mins.
		//No power saving advantage for track itervals for less than 10 mins.
		if(configStruct->TrackingInterval >= '9')
		{
			configStruct->Mode = LP_FULL_TRACK_MODE;
			Config_SetMode(LP_FULL_TRACK_MODE, 0, NULL, TRUE); 	
		#ifdef DEBUG_CONFIG_SETTINGS
			DumpMessage("Mode will change to: Low Power Full Track Mode (Full Track with sleep Enabled)\r\n");
		#endif			
			return 1;
		}
		else
		{
		#ifdef DEBUG_CONFIG_SETTINGS
			DumpMessage("Failed to activate Low Power Full Track Mode - Tracking Interval should be >= 10mins\r\n");
		#endif
			return -1;	
		}
	case 'A':
		//Alarm mode supported only if Track interval is more than 30mins.		
		if(configStruct->TrackingInterval >= '8')
		{
			configStruct->Mode = ALARM_MODE;
			Config_SetMode(ALARM_MODE, 0, NULL, TRUE);	
		#ifdef DEBUG_CONFIG_SETTINGS
			DumpMessage("Mode will change to: Alarm Mode\r\n");
		#endif			
			return 1;
		}
		else
		{
		#ifdef DEBUG_CONFIG_SETTINGS
			DumpMessage("Failed to activate Alarm Mode - Tracking Interval should be >= 5 mins\r\n");
		#endif
			return -1;	
		}

	case 'P':
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("Mode will change to: Dog Park Mode \r\n");
#endif
		configStruct->Mode = DOG_PARK_MODE;
        SetDogParkMode(DOGPARK_ENABLED);
        SetDogWalkMode(DOGWALK_DISABLED);
		Config_SetMode(DOG_PARK_MODE, 0, NULL, TRUE);      // TODO add dog park fence.
		return 1;

	case '0':
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("Mode will stay same \r\n");
#endif
		configStruct->Mode = g_config.Mode;     //Retain the existing mode.
		//Config_SetMode(NO_CHANGE_MODE, 0, NULL, TRUE);
		return 1;

/*
	case 'Q':
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("Mode set to server no sleep \r\n");
#endif
		configStruct->Mode = SERVER_NO_SLEEP;
		return 1;
*/
	case 'S':
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("Server requesting a status packet\r\n");
#endif		
		//This is stored in order to restore the mode once the status packet is sent out.
		PREV_Mode = g_config.Mode;
		configStruct->Mode = SERVER_STATUS_REQ;
		return 1;

	case 'V':
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("Server requesting a status packet with vibrate\r\n");
#endif
		//This is stored in order to restore the mode once the status packet is sent out.
		PREV_Mode = g_config.Mode;
		configStruct->Mode = SERVER_STATUS_REQ_V;
		return 1;

	default:
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("\r\nInvalid  mode setting");
#endif
		return -1;
	}
}


/** @brief Set the current number of fences in the system.
 *
 * @param value
 * @param configStruct
 * @return 1 on success
 * @return -1 on failure
 */
int SetFenceNumber(UINT8 *value, CONFIG *configStruct)
{
	UINT8 val = hex_ascii_2_uint(value, 8);
	if (val <= NUM_FENCES)
	{
#ifdef DEBUG_CONFIG_SETTINGS
		wm_sprintf(g_traceBuf, "Number of fences setting will change to: %d \r\n", (int)val);
		DumpMessage(g_traceBuf);
#endif
		configStruct->NumFences = val;
		return 1;
	}
	else
	{
		DumpMessage("\r\nInvalid fence number setting");
		return -1;
	}
}


/** @brief Enable or disable the GPS alert based on the received configuration.
 *
 * @param value
 * @param configStruct
 * @return 1 on success
 * @return -1 on failure
 */
int SetGPSAlert(UINT8 *value, CONFIG *configStruct)
{
	UINT8 val = *value - 0x30;

	if (val <= 3)
	{
#ifdef DEBUG_CONFIG_SETTINGS
		wm_sprintf(g_traceBuf, "Setting GPSAlert to %x\r\n", val);
		DumpMessage(g_traceBuf);
#endif
		configStruct->GPSAlert = val;
		return 1;
	}
	else
	{
#ifdef DEBUG_CONFIG_SETTINGS
		wm_sprintf(g_traceBuf, "Bad GPS Alert: %x\r\n", val);
		DumpMessage(g_traceBuf);
#endif
		return -1;
	}
}


/** @brief Enable or disable the GSM alert based on the received configuration
 *
 * @param value
 * @param configStruct
 * @return 1 on success
 * @return -1 on failure
 */
int SetGSMAlert(UINT8 *value, CONFIG *configStruct)
{
	//   configStruct->GSMAlertThresh = *value - 0x30;
#ifdef DEBUG_CONFIG_SETTINGS
	wm_sprintf(g_traceBuf, "value = %c\r\n", *value);
	DumpMessage(g_traceBuf);
#endif
	if (((*value >= '0') && (*value <= '5')) || (*value == 'N'))
	{
		configStruct->GSMAlertThresh = *value;
#ifdef DEBUG_CONFIG_SETTINGS
		wm_sprintf(g_traceBuf, "GSM Alert Threshold will change to: %d \r\n", (int)configStruct->GSMAlertThresh);
		DumpMessage(g_traceBuf);
#endif
		return 1;
	}
	else
	{
		return -1;
	}
}


/** @brief Set the battery alert threshold based on the received configuration.
 *
 * @param value
 * @param configStruct
 * @return 1 on success
 * @return -1 on failure
 */
int SetBattAlert(UINT8 *value, CONFIG *configStruct)
{
	UINT8 temp_var8 = *value - 0x30;
	if (temp_var8 <= 5)
	{
#ifdef DEBUG_CONFIG_SETTINGS
		wm_sprintf(g_traceBuf, "Battery Alert threshold will change to: %d \r\n", temp_var8);
		DumpMessage(g_traceBuf);
#endif
		configStruct->BattAlert = temp_var8;
		return 1;
	}
	else
	{
		DumpMessage("\r\nInvalid battery alert threshold setting");
		return -1;
	}
}


/** @brief Enable or disable the power disconnect alert based on the recieved configuration.
 *
 * @param value
 * @param configStruct
 * @return 1 on success
 * @return -1 on failure
 */
int SetPowerDisconAlert(UINT8 *value, CONFIG *configStruct)
{
	if (*value == 'N')
	{
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("Power disconnect Alert will change to: disable \r\n");
#endif
		configStruct->PowerDiscAlert = DISABLE;
		return 1;
	}
	else if (*value == 'Y')
	{
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("Power disconnect Alert will change to: enable \r\n");
#endif
		configStruct->PowerDiscAlert = ENABLE;
		return 1;
	}
	else
	{
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("\r\nInvalid  power disconnect alert setting");
#endif
		return -1;
	}
}


/** @brief Set the overspeed threshold value based on the received configuration.
 *
 * @param value
 * @param configStruct
 * @return 1 on success
 */
int SetOverSpeedAlertThresh(UINT8 *value, CONFIG *configStruct)
{
	UINT8 temp_var8 = hex_ascii_2_uint(value, 8);

#ifdef DEBUG_CONFIG_SETTINGS
	wm_sprintf(g_traceBuf, "Over Speed Alert Threshold setting will change to: %d \r\n", (int)temp_var8);
	DumpMessage(g_traceBuf);
#endif
	configStruct->OverSpeedAlertThresh = temp_var8;
	return 1;
}


/** @brief Set the power down disable based on the received configuration.
 *
 * @param value
 * @param configStruct
 * @return 1 on success
 * @return -1 on failure
 */
int SetPowerDownDisable(UINT8 *value, CONFIG *configStruct)
{
	if (*value == 'N')
	{
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("Power Down Disable will change to: disable (normal power down) \r\n");
#endif
		configStruct->PowerDownDisable = DISABLE;
		return 1;
	}
	else if (*value == 'Y')
	{
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("Power Down Disable will change to: enable (fake power down mode)\r\n");
#endif
		configStruct->PowerDownDisable = ENABLE;
		return 1;
	}
	else
	{
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("\r\nInvalid  power down disable setting");
#endif
		return -1;
	}
}


/** @brief Activiate the vibration motor with a predefined pattern.
 *
 * @param value
 * @param configStruct
 * @return 1 on success
 * @return -1 on failure
 */
int SetVibrationMotorPattern(UINT8 *value, CONFIG *configStruct)
{
	UINT8 temp_var8 = *value - 0x30;
	if (temp_var8 == 0)
	{
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("Vibration motor pattern: no change\r\n");
#endif
		configStruct->ServerVibMotorOverride = DISABLE;
		return 1;
	}
	else if (temp_var8 <= 9)
	{
#ifdef DEBUG_CONFIG_SETTINGS
		wm_sprintf(g_traceBuf, "Vibration Motor Pattern will change to: %d \r\n", (int)temp_var8);
		DumpMessage(g_traceBuf);
#endif
		configStruct->VibrationMotorPattern = (temp_var8 - 1);
		configStruct->ServerVibMotorOverride = ENABLE;
		return 1;
	}
	else
	{
		DumpMessage("\r\nInvalid vibration motor pattern setting");
		return -1;
	}
}


/** @brief Set the vibration motor delay functionality.
 * @param value
 * @param configStruct
 * @return 1 on success
 * @return -1 on failure
 */
int SetVibrationMotorDelayEn(UINT8 *value, CONFIG *configStruct)
{
	if (*value == 'Y')
	{
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("Vibration Motor Delay will change to: enable\r\n");
#endif
		configStruct->VibrationMotorDelayEn = ENABLE;
		return 1;
	}
	else if (*value == 'N')
	{
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("Vibration Motor Delay will change to: disable\r\n");
#endif
		configStruct->VibrationMotorDelayEn = DISABLE;
		return 1;
	}
	DumpMessage("\r\nInvalid vibration motor delay setting");
	return -1;
}


/** @brief Control the SOS alert.
 *
 * @param value
 * @param configStruct
 * @return 1 on success
 * @return -1 on failure
 */
int SetSOSAlert(UINT8 *value, CONFIG *configStruct)
{
	if (*value == 'Y')
	{
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("SOS Alert will change to: enable\r\n");
#endif
		configStruct->SOSAlert = ENABLE_SOS;
		g_NeedSOSAck = 0;
		return 1;
	}
	else if (*value == 'N')
	{
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("SOS Alert will change to: disable\r\n");
#endif
		configStruct->SOSAlert = DISABLE_SOS;
		return 1;
	}
	else if (*value == 'A')
	{
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("Server Acknowledge the SOS alarm\r\n");
#endif
		g_NeedSOSAck = 0;
		configStruct->SOSAlert = ENABLE_SOS;
		configStruct->VibrationMotorPattern = configStruct->CriticalConfirm;
		return 1;
	}
	else
	{
		DumpMessage("\r\nInvalid SOS alert setting");
		return -1;
	}
}


/** @brief Initiate a waypoint download session.
 *
 * @param value
 * @param configStruct
 * @return 1 on success
 * @return -1 on failure
 */
int SetWaypointDownload(UINT8 *value, CONFIG *configStruct)
{
	int result = -1;
	switch (*value)
	{
	case '0':
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("Waypoint download will change to: Don't start download\r\n");
#endif
		configStruct->WaypointDLData = NO_DOWNLOAD;
		result = 1;
		break;

	case '1':
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("Waypoint download will change to: 1 hr of data\r\n");
#endif
		configStruct->WaypointDLData = HOUR1_DATA;
		result = 1;
		break;

	case '8':
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("Waypoint download will change to: 8 hours of data\r\n");
#endif
		configStruct->WaypointDLData = HOUR8_DATA;
		result = 1;
		break;

	case '2':
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("Waypoint download will change to: 24 hours of data\r\n");
#endif
		configStruct->WaypointDLData = HOUR24_DATA;
		result = 1;
		break;

	case '7':
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("Waypoint download will change to: 72hours of Data\r\n");
#endif
		configStruct->WaypointDLData = HOUR72_DATA;
		result = 1;
		break;

	case 'W':
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("Waypoint download will change to: 1 Weeks of Data\r\n");
#endif
		configStruct->WaypointDLData = WEEK1_DATA;
		result = 1;
		break;

	case 'A':
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("Waypoint download will change to: All Data\r\n");
#endif
		configStruct->WaypointDLData = ALL_DATA;
		result = 1;
		break;

	default:
		DumpMessage("\r\nInvalid Waypoint Download setting");
		break;
	}
	return result;
}


/** @brief Allow the server to set the LED pattern.
 *
 * @param value
 * @param configStruct
 * @return 1 on success
 * @return -1 on failure
 */
int SetLEDPattern(UINT8 *value, CONFIG *configStruct)
{
	UINT8 val = *value - 0x30;

	if (val == 0)
	{
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("LED Pattern: No Change\r\n");
#endif
		configStruct->ServerLEDOverride = DISABLE;
		return 1;
	}
	else if (val <= 9)
	{
#ifdef DEBUG_CONFIG_SETTINGS
		wm_sprintf(g_traceBuf, "LED Pattern will change to: %d \r\n", (int)val);
		DumpMessage(g_traceBuf);
#endif
		configStruct->LEDPattern = (val - 1);
		configStruct->ServerLEDOverride = ENABLE;
		return 1;
	}
	else
	{
		DumpMessage("Invalid LED Pattern setting\r\n");
		return -1;
	}
}


/** @brief Control which LEDs are enabled.
 *
 * @param value
 * @param configStruct
 * @return 1 on success
 */
int SetLEDEnable(UINT8 *value, CONFIG *configStruct)
{
	UINT8 val = hex_ascii_2_uint(value, 8);
	int i;
	int mask = 0x01;

	for (i = 0; i < NUM_LEDS; i++)
	{
		if ((val & (mask << i)) > 0)
		{
			configStruct->LEDEn[i] = 1;
		}
		else
		{
			configStruct->LEDEn[i] = 0;
		}
	}

	return 1;
}


/** @brief Set the duration before the device will experience a timed wakeup.
 *
 * @param value
 * @param configStruct
 * @return 1 on success
 * @return -1 on failure
 */
int SetTrackingModeDuration(UINT8 *value, CONFIG *configStruct)
{
	UINT32 val = hex_ascii_2_uint(value, 16);

	if (val <= SECONDS_IN_DAY)
	{
		//#ifdef DEBUG_CONFIG_SETTINGS
		wm_sprintf(g_traceBuf, "Tracking mode duration change to: %d \r\n", (int)val);
		DumpMessage(g_traceBuf);
		//#endif
		configStruct->TrackingModeDuration = val;
		return 1;
	}
	else
	{
		DumpMessage("\r\nInvalid Timed Wakeup setting");
		return -1;
	}
}


/** @brief Enable or disable invisible operation.
 *
 * @param value
 * @param configStruct
 * @return 1 on success
 * @return -1 on failure
 */
int SetInvisibleOp(UINT8 *value, CONFIG *configStruct)
{
	DumpMessage("Set Invisble Op: \r\n");
	if (*value == 'Y')
	{
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("Invisible Operation will change to: enable\r\n");
#endif
		start_invisible_timer();
		//       configStruct->InvisibleOp = ENABLE;
		return 1;
	}
	else if (*value == 'N')
	{
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("Inivisble Operation will change to: disable\r\n");
#endif
		configStruct->InvisibleOp = DISABLE;
		return 1;
	}
	else
	{
		wm_sprintf(g_traceBuf, "\r\nInvalid Invisible Operation setting: %c\r\n", *value);
		DumpMessage(g_traceBuf);
		return -1;
	}
}


/** @brief Enable or disable the magnetic sensor SOS.
 *
 * @param value
 * @param configStruct
 * @return 1 on success
 * @return -1 on failure
 */
int SetMagneticSNSEnable(UINT8 *value, CONFIG *configStruct)
{
	if (*value == 'Y')
	{
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("Magnetic Sensor will change to: enable\r\n");
#endif
		configStruct->MagneticSnsEn = ENABLE;
		return 1;
	}
	else if (*value == 'N')
	{
#ifdef DEBUG_CONFIG_SETTINGS
		DumpMessage("Magnetic Sensor will change to: disable\r\n");
#endif
		configStruct->MagneticSnsEn = DISABLE;
		return 1;
	}
	else
	{
		DumpMessage("\r\nInvalid Magnetic Sensor setting");
		return -1;
	}
}


/** @brief Enable or disable confirmation of critical messages such as SOS.
 *
 * @param value
 * @param configStruct
 * @return 1 on success
 * @return -1 on failure
 */
int SetCriticalConfirmation(UINT8 *value, CONFIG *configStruct)
{
	UINT8 val = *value - 0x30;
	if (val <= 9)
	{
#ifdef DEBUG_CONFIG_SETTINGS
		wm_sprintf(g_traceBuf, "Critical Confirmation setting will change to: %d \r\n", val);
		DumpMessage(g_traceBuf);
#endif
		configStruct->CriticalConfirm = val;
		return 1;
	}
	else
	{
		DumpMessage("\r\nInvalid Critical Confirmation setting");
		return -1;
	}
}


/** @brief Set the phone number for the SMS server.
 *
 * @param value
 * @param configStruct
 * @return 1 on success
 * @return -1 on failure
 */
int SetServerPhoneNum(UINT8 *value, CONFIG *configStruct)
{
	memcpy(configStruct->ServerPhoneNum, value, 14);
	return 1;
}


/** @brief Set the IP address for the server.
 *
 * @param value
 * @param configStruct
 * @return 1 on success
 * @return -1 on failure
 */
int SetServerIPAddr(UINT8 *value, CONFIG *configStruct)
{
	wip_in_addr_t tmp_ip;
	wip_in_addr_t final_ip = 0;

	tmp_ip = hex_ascii_2_uint(value, 32);

	// swap the bytes.
	final_ip = (tmp_ip >> 24) & 0xff;
	final_ip |= (tmp_ip >> 8) & 0xff00;
	final_ip |= (tmp_ip << 8) & 0xff0000;
	final_ip |= (tmp_ip << 24) & 0xff000000;

#ifdef DEBUG_CONFIG_SETTINGS
	wm_sprintf(g_traceBuf, "tmp_ip = %x\r\n", (unsigned int)tmp_ip);
	DumpMessage(g_traceBuf);
#endif
	if (wip_inet_ntoa(final_ip, &(configStruct->ServerIPAddr[0]), 15) == FALSE)
	{
		return -1;
	}

	return 1;
}


/** @brief Set the port for the server.
 *
 * @param value
 * @param configStruct
 * @return 1 on success
 */
int SetServerPort(UINT8 *value, CONFIG *configStruct)
{
	configStruct->ServerPortNum = hex_ascii_2_uint(value, 16);
	return 1;
}


/** @brief Enable or disable particular fences.
 *
 * @param fence_settings
 * @param configStruct
 * @return 1 on success
 */
int SetFenceEnDisSettings(UINT8 *fence_settings, CONFIG *configStruct)
{
	//   int mask = 0x00000000000001;
	int mask = 0x00000001;
	int i;
	UINT32 fence_settings_24_MSB = hex_ascii_2_uint(fence_settings, 24);
	UINT32 fence_settings_32_LSB = hex_ascii_2_uint(&fence_settings[6], 32);

	GEOFENCE tempGeoFence;

	wm_sprintf(g_traceBuf, "fence_active = %x%x\r\n", (unsigned int)fence_settings_32_LSB, (unsigned int)fence_settings_24_MSB);
	DumpMessage(g_traceBuf);

	for (i = 0; i < NUM_FENCES; i++)
	{
		memcpy(&tempGeoFence, &g_GeoFences[i], sizeof (GEOFENCE));
		if (i < 32)
		{
			if ((fence_settings_32_LSB & (mask << i)) > 0)
			{
				tempGeoFence.enable = ENABLE;
				configStruct->FencesActive[i] = 1;
			}
			else
			{
				tempGeoFence.enable = DISABLE;
				configStruct->FencesActive[i] = 0;
			}
		}
		else
		{
			if ((fence_settings_24_MSB & (mask << (i - 32))) > 0)
			{
				tempGeoFence.enable = ENABLE;
				configStruct->FencesActive[i] = 1;
			}
			else
			{
				tempGeoFence.enable = DISABLE;
				configStruct->FencesActive[i] = 0;
			}
		}

		memcpy(&g_GeoFences[i], &tempGeoFence, sizeof (GEOFENCE));
		FencesToFlash(i);
		if (i == 0)
		{
			DisplayFence(&g_GeoFences[i]);
			LoadFromFlash();
			DisplayFence(&g_GeoFences[i]);
		}
	}
	return 1;
}


/*F***************************************************************
 *
 *   NAME:    int WriteFenceCoord(ascii * num, ascii *lon1, ascii *lat1,ascii *lon2
 *                   , ascii *lat2,ascii *lon3, ascii *lat3,ascii *lon4, ascii *lat4)
 */

/*! \brief   Writes the fence coordinates specified to the fence number
 *            that is specified by the input
 *
 *   \param
 *
 * \param  num [in]                The fence number to write
 * \param  lon1 [in]               Longetude of the first coordinate
 * \param  lat1 [in]               Latitude of the first coordinate
 * \param  lon2 [in]               Longetude of the second coordinate
 * \param  lat2 [in]               Latitude of the second coordinate
 * \param  lon3 [in]               Longetude of the third coordinate
 * \param  lat3 [in]               Latitude of the third coordinate
 * \param  lon4 [in]               Longetude of the fourth coordinate
 * \param  lat4 [in]               Latitude of the fourth coordinate
 * \param  in_ex [in]              Used to be inclusive exclusive, now contains the day of week and fence direction
 *   \retval
 *      -1  Error fence not set
 *   \retval
 *      1 Success
 *   \note   none
 *//*
 *F*/

/** @brief Write Fence Corrdinates
 *
 * @par This function writes a geofence based on the provided coordinates and configuration parameters.
 *
 * @param num
 * @param configStruct
 * @param in_ex
 * @param lon1
 * @param lat1
 * @param lon2
 * @param lat2
 * @param lon3
 * @param lat3
 * @param lon4
 * @param lat4
 * @param start_time
 * @param stop_time
 *
 * @return 0
 */
int WriteFenceCoord(ascii *num, CONFIG *configStruct, ascii *in_ex, ascii *lon1,
                    ascii *lat1, ascii *lon2, ascii *lat2, ascii *lon3, ascii *lat3,
                    ascii *lon4, ascii *lat4, ascii *start_time, ascii *stop_time)
{
	GEOFENCE testGeoFence;
	int fenceNumber = 0;

	wm_sprintf(g_traceBuf, "num = %c%c\r\n", num[0], num[1]);
	DumpMessage(g_traceBuf);

	fenceNumber = wm_hexatoi(num, 2) - 1;

	/* check to see if the fence number is within bounds */
	if (fenceNumber > (NUM_FENCES - 1))
	{
		DumpMessage("Error, fence number is greater than 50\r\n");
		return -1;
	}
	else if (fenceNumber == -1)
	{
		DumpMessage("Server does not want to set any fence data\r\n");
		return 0;
	}
	else
	{
		//#ifdef DEBUG_CONFIG_SETTINGS
#if 1
		wm_sprintf(g_traceBuf, "Recieved fence number: %d\r\n", fenceNumber);
		DumpMessage(g_traceBuf);
#endif
	}

	/*	wm_sprintf(g_traceBuf,"coord length: %d\r\n",strlen(*lat1));
	 * DumpMessage(g_traceBuf);
	 * if (strlen(lat1) != 8 || strlen(lon1) != 8 ||
	 *  strlen(lat2) != 8 || strlen(lon2) != 8 ||
	 *  strlen(lat3) != 8 || strlen(lon3) != 8 ||
	 *  strlen(lat4) != 8 || strlen(lon4) != 8 )  {
	 *
	 * DumpMessage("Error, wrong length for coordinate\r\n");
	 * return -1;
	 * }*/

	if (configStruct->FencesActive[fenceNumber] == 1)
	{
		testGeoFence.enable = ENABLE;
	}
	else
	{
		testGeoFence.enable = DISABLE;
	}

	char dow_dir = (char)(*in_ex) - 0x20;

	wm_sprintf(g_traceBuf, "DOW DIR=%x\r\n", dow_dir);
	DumpMessage(g_traceBuf);
	if ((dow_dir & 0xf) > 0xB)
	{
		DumpMessage("Invalid DOW setting\r\n");
		return -1;
	}
	/* load the alarm direction */
	//	testGeoFence.alarm_dir =  ((char)(*in_ex) >> 6) & 0x3;
	char alarm_dir = (dow_dir >> 4) & 0x7;
	if (alarm_dir > 4)
	{
		DumpMessage("Invalid Alarm Direction\r\n");
		return -1;
	}
	testGeoFence.alarm_dir = alarm_dir;

	/* load the day of week information */
	//	testGeoFence.days = (char)(*in_ex) & 0xf ;
	testGeoFence.days = dow_dir & 0xf;

	if (special_start_time == 1)
	{
		testGeoFence.time.start_time = 0x5f;
	}
	else
	{
		testGeoFence.time.start_time = *(unsigned char *)start_time - 0x20;
	}

	if (special_stop_time == 1)
	{
		testGeoFence.time.stop_time = 0x5f;
	}
	else
	{
		testGeoFence.time.stop_time = *(unsigned char *)stop_time - 0x20;
	}

	//	testGeoFence.time.start_time = *(unsigned char *)start_time;
	//	testGeoFence.time.stop_time  = *(unsigned char *)stop_time;

	wm_sprintf(g_traceBuf, "alarm dir = %x\r\ndays= %x\r\nstart_time=%x\r\nstop_time=%x\r\n",
	           testGeoFence.alarm_dir,
	           testGeoFence.days,
	           testGeoFence.time.start_time,
	           testGeoFence.time.stop_time);
	DumpMessage(g_traceBuf);

	testGeoFence.posts[0].lat_32 = hex_ascii_2_uint((UINT8 *)lat1, 32);
	testGeoFence.posts[0].long_32 = hex_ascii_2_uint((UINT8 *)lon1, 32);

	testGeoFence.posts[1].lat_32 = hex_ascii_2_uint((UINT8 *)lat2, 32);

	testGeoFence.posts[1].long_32 = hex_ascii_2_uint((UINT8 *)lon2, 32);

	testGeoFence.posts[2].lat_32 = hex_ascii_2_uint((UINT8 *)lat3, 32);
	testGeoFence.posts[2].long_32 = hex_ascii_2_uint((UINT8 *)lon3, 32);
	testGeoFence.posts[3].lat_32 = hex_ascii_2_uint((UINT8 *)lat4, 32);
	testGeoFence.posts[3].long_32 = hex_ascii_2_uint((UINT8 *)lon4, 32);

#ifdef DEBUG_CONFIG_SETTINGS
	DisplayFence(&testGeoFence);
#endif

	init_fence_status(fenceNumber);

	int retcode;
	if ((retcode = FlashWrite((ascii *)g_fencesHandle, fenceNumber, sizeof (GEOFENCE), (u8 *)(&testGeoFence))) < 0)
	{
		DisplayErrorCode("FlashWrite", __FILE__, __LINE__, retcode);
		wm_sprintf(g_traceBuf, "fence number = %d", fenceNumber);
		DumpMessage(g_traceBuf);
	}

	// copy it to the global geofence structure too
	memcpy(&g_GeoFences[fenceNumber], &testGeoFence, sizeof (GEOFENCE));

	return 0;
}


/** @brief used for overriding current fix for testing
 *
 * @param utc
 * @param lat
 * @param longi
 * @param epe
 * @return void
 */
// used for overriding current fix for testing.
void SetCurrentFix(ascii *utc, ascii *lat, ascii *longi, ascii *epe)
{
	(void)utc;
	(void)lat;
	(void)longi;
	(void)epe;
}


void Config_SetMode(MODES mode, int duration, GEOFENCE *gf, BOOL write_flash)
{
	MODE_NONVOLATILE_STORAGE mvs;
	g_config.Mode = mode;
	if (write_flash)
	{
		mvs.mode = mode;
		mvs.track_dur_rem = duration;
		if (gf)
		{
			memcpy(&mvs.DogParkFence, gf, sizeof (GEOFENCE));
		}
		InternalFlash_WriteNVMode(&mvs);
	}
}


/** @brief Display a particular geofence for debug purposes.
 *
 * @param gf
 * @return void
 */
void DisplayFence(GEOFENCE *gf)
{
	int i;

	DumpMessage("\r\nDisplay Fence\r\n");
	for (i = 0; i < NUM_POSTS_PER_FENCE; i++)
	{
		wm_sprintf(g_traceBuf, "%d,%d\r\n", (int )gf->posts[i].lat_32,
		           (int )gf->posts[i].long_32);
		DumpMessage(g_traceBuf);
	}
}


static bool enable_ota27 = false;
void ConfigSettings_setOTA27(bool ota27)
{
	enable_ota27 = ota27;
}


/** @brief Display the current config for debug purposes.
 *
 * @return void
 */
void DisplayConfig(void)
{
	int i;
	wm_sprintf(g_traceBuf, "SMSorGPRS: %d\r\n", g_config.SMSorGPRS);
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	wm_sprintf(g_traceBuf, "WaypointInterval: %x\r\n", (unsigned int )g_config.WaypointInterval);
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	wm_sprintf(g_traceBuf, "TrackingInterval: %d Sec (%c)\r\n", GetTrackingInterval(), g_config.TrackingInterval);
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	wm_sprintf(g_traceBuf, "MotionAlarmThresh: %x\r\n", g_config.MotionAlarmThresh);
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	wm_sprintf(g_traceBuf, "AccelThreshWake: %x\r\n", g_config.AccelThreshWake);
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	wm_sprintf(g_traceBuf, "AccelDurationWake: %x\r\n", (unsigned int )g_config.AccelDurationWake);
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	wm_sprintf(g_traceBuf, "AccelDurationSleep: %x\r\n", (unsigned int )g_config.AccelDurationSleep);
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	wm_sprintf(g_traceBuf, "BreadCrumbMode: %d\r\n", g_config.BreadCrumbMode);
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	wm_sprintf(g_traceBuf, "Mode: %d\r\n", g_config.Mode);
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	wm_sprintf(g_traceBuf, "NumFences: %x\r\n", g_config.NumFences);
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	wm_sprintf(g_traceBuf, "GPSAlert: %d\r\n", g_config.GPSAlert);
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	wm_sprintf(g_traceBuf, "GSMAlertThresh: %d\r\n", g_config.GSMAlertThresh);
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	wm_sprintf(g_traceBuf, "BattAlert: %d\r\n", g_config.BattAlert);
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	wm_sprintf(g_traceBuf, "PowerDiscAlert: %d\r\n", g_config.PowerDiscAlert);
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	wm_sprintf(g_traceBuf, "OverSpeedAlertThresh: %d\r\n", g_config.OverSpeedAlertThresh);
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	wm_sprintf(g_traceBuf, "PowerDownDisable: %d\r\n", g_config.PowerDownDisable);
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	wm_sprintf(g_traceBuf, "VibrationMotorPattern: %d\r\n", g_config.VibrationMotorPattern);
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	wm_sprintf(g_traceBuf, "VibrationMotorDelayEn: %d\r\n", g_config.VibrationMotorDelayEn);
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	wm_sprintf(g_traceBuf, "ServerVibMotorOverride: %d\r\n", g_config.ServerVibMotorOverride);
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	wm_sprintf(g_traceBuf, "SOSAlert: %x\r\n", g_config.SOSAlert);
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	wm_sprintf(g_traceBuf, "WaypointDLData: %d\r\n", g_config.WaypointDLData);
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	wm_sprintf(g_traceBuf, "LEDPattern: %d\r\n", g_config.LEDPattern);
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	for (i = 0; i < NUM_LEDS; i++)
	{
		wm_sprintf(g_traceBuf, "LEDEn[%d]: %x\r\n", i, g_config.LEDEn[i]);
		DumpMessageUSB(g_traceBuf, 1);
		DumpMessage(g_traceBuf);
		if (enable_ota27)
		{
			OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
		}
	}
	wm_sprintf(g_traceBuf, "ServerLEDOverride: %x\r\n", g_config.ServerLEDOverride);
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	wm_sprintf(g_traceBuf, "TimedWakeup: %d\r\n", (int )g_config.TimedWakeup);
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	for (i = 0; i < NUM_FENCES; i++)
	{
		wm_sprintf(g_traceBuf, "FencesActive[%d]: %x\r\n", i, g_config.FencesActive[i]);
		DumpMessageUSB(g_traceBuf, 1);
		DumpMessage(g_traceBuf);
		if (enable_ota27)
		{
			OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
		}
	}

	wm_sprintf(g_traceBuf, "InvisibleOp: %x\r\n", g_config.InvisibleOp);
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	wm_sprintf(g_traceBuf, "MagneticSnsEn: %x\r\n", g_config.MagneticSnsEn);
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	wm_sprintf(g_traceBuf, "CriticalConfirm: %x\r\n", g_config.CriticalConfirm);
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	DumpMessageUSB("ServerPhoneNum: ", 1);
	DumpMessage("ServerPhoneNum: ");
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	for (i = 0; i < MAX_PHNUM_LEN; i++)
	{
		wm_sprintf(g_traceBuf + i, "%c", g_config.ServerPhoneNum[i]);
	}
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	DumpMessageUSB("\r\n", 1);
	DumpMessage("\r\n");

	DumpMessageUSB("ServerIPAddr: ", 1);
	DumpMessage("ServerIPAddr: ");
	for (i = 0; i < 16; i++)
	{
		wm_sprintf(g_traceBuf + i, "%c", g_config.ServerIPAddr[i]);
	}
	g_traceBuf[16] = 0x00;
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	DumpMessageUSB("\r\n", 1);
	DumpMessage("\r\n");

	DumpMessageUSB("ServerPortNum: ", 1);
	DumpMessage("ServerPortNum: ");
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response("ServerPortNum: ", false, strlen(g_traceBuf));
	}
	wm_sprintf(g_traceBuf, "%d\r\n", g_config.ServerPortNum);
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	wm_sprintf(g_traceBuf, "TrackingModeDuration: %u\r\n", (unsigned int)g_config.TrackingModeDuration);
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	wm_sprintf(g_traceBuf, "ConfigPktRxProperly: %d\r\n", g_config.ConfigPktRxProperly);
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));
	}
	wm_sprintf(g_traceBuf, "MotionAlarmIndep: %d\r\n", g_config.MotionAlarmIndep);
	DumpMessageUSB(g_traceBuf, 1);
	DumpMessage(g_traceBuf);
	if (enable_ota27)
	{
		OTAAT_handle_at_cmd_response(g_traceBuf, true, strlen(g_traceBuf));
	}
	ConfigSettings_setOTA27(false);
}


#ifdef RESET_WHEN_MODE_SWITCH
void start_reset_timer(void)
{
	if (adl_tmrSubscribe(FALSE, 50, ADL_TMR_TYPE_100MS, (adl_tmrHandler_t)reset_timer) == NULL)
	{
		DumpMessage("Could not start reset timer\r\n");
		adl_ctxSleep(ADL_TMR_MS_TO_TICK(5000));
		adl_atCmdCreate("AT+CFUN=1", FALSE, NULL, NULL);
	}
}


void reset_timer(u8 timerID, void *Context)
{
	(void)timerID;
	(void)Context;
	adl_atCmdCreate("AT+CFUN=1", FALSE, NULL, NULL);
}


void start_invisible_timer(void)
{
	if (adl_tmrSubscribe(FALSE, 50, ADL_TMR_TYPE_100MS, (adl_tmrHandler_t)invisible_timer) == NULL)
	{
		DumpMessage("Could not start invisible timer\r\n");
		g_config.InvisibleOp = ENABLE;
	}
}


void invisible_timer(u8 timerID, void *Context)
{
	(void)timerID;
	(void)Context;

	DumpMessage("Invisible Timer\r\n");
	g_config.InvisibleOp = ENABLE;
	g_config.LEDPattern = PATTERN_NORMAL;
}


#endif
/**@}*/
