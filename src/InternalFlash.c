/** @addtogroup Storage
 *@{*/

/*H***************************************************************************
 */

/** @file InternalFlash.c
 *
 * @brief Manage the internal flash on the WMP100.
 *
 * @note    Other help for the reader, including references.
 *
 *
 *//*
 *
 *
 *H*/
#include "adl_global.h"
#include "FlashTest.h"
#include "fence.h"
#include "SMShandling.h"
#include "gps.h"
#include "XactUtilities.h"
#include "ConfigSettings.h"
#include "WaypointControl.h"
#include "status.h"
#include "InternalFlash.h"
#include "dogwalk.h"

extern GEOFENCE g_GeoFences[NUM_FENCES];
extern GEOFENCE g_DogParkFence;
extern COORD g_current_fix_32;
extern UINT8 g_sms_tx[SMS_TX_BUFFSIZE];
extern UINT8 g_last_config[RX_BUFFSIZE];
extern DIAG_CONFIG g_DiagConfig[NUM_DIAG_COMMANDS];
extern UINT8 g_DiagCmdOffset;
extern AGPS_DATA g_AGPSData;

const ascii g_configStructHandle[] = "ConfigStructHandle";
const ascii g_logPacketHandle[] = "LogPacketHandle";
const ascii g_fencesHandle[] = "FencesHandle";
const ascii g_configStringHandle[] = "ConfigurationString";
const ascii g_accelCalibHandle[] = "AccelCalibHandle";
const ascii g_bandSettingHandle[] = "BandSettingHandle";
const ascii g_APNNameHandle[] = "APNName";
const ascii g_APNLoginHandle[] = "APNLogin";
const ascii g_APNPasswordHandle[] = "APNPassword";
const ascii g_NVSHandle[] = "NVSHandle";          /*non volatile status handle */
const ascii g_RebootAlarmHandle[] = "RebootAlarmHandle";
const ascii g_NVModeHandle[] = "NVModeHandle";
const ascii g_DiagCfgStructHandle[] = "DiagnosticCfgHandle";
const ascii g_ModeconfigStructHandle[] = "ModeConfigHanlde";
const ascii g_AGPSDataHandle[] = "AGPSDataHandle";

static CONFIG flashConfig;

SMS_GPRS_MODE prev_SMSorGPRSMode;

void LoadDefaultConfigStruct(void);

/** @brief Loads all values from flash, except the flash log
 *
 * @return void
 */
void LoadFromFlash()
{
	int m_count;

	MODE_NONVOLATILE_STORAGE mvs;

	// The log packet array
	FlashSubscribe((ascii *)g_logPacketHandle, MAX_NUM_LOG_PKT);

	
	//Diagnostic config struct
	FlashSubscribe((ascii *)g_DiagCfgStructHandle, NUM_DIAG_COMMANDS);

	// The configuration structure.
	FlashSubscribe((ascii *)g_configStructHandle, 1);

	// The configuration string.
	FlashSubscribe((ascii *)g_configStringHandle, 1);

	// The acceleromter calibration.
	FlashSubscribe((ascii *)g_accelCalibHandle, 3);

	//Mode config struct
	FlashSubscribe((ascii *)g_ModeconfigStructHandle, 1);

	//AGPS data struct
	FlashSubscribe((ascii *)g_AGPSDataHandle, 1);

	// Read configuration strcuture from flash.
	if (FlashRead((ascii *)g_configStructHandle, FLASH_ID, sizeof (CONFIG), (u8 *)(&g_config)) < 0)
	{
		DumpMessage("Could not read configuration from flash\r\n");
	}

	
	// Read Mode configuration strcuture from flash.
	if (FlashRead((ascii *)g_ModeconfigStructHandle, FLASH_ID, sizeof (MODE_CONFIG), (u8 *)(&g_ModeConfig)) < 0)
	{
		DumpMessage("Could not read mode configuration from flash\r\n");
	}

	// Read AGPS data strcuture from flash.
	if (FlashRead((ascii *)g_AGPSDataHandle, FLASH_ID, sizeof (AGPS_DATA), (u8 *)(&g_AGPSData)) < 0)
	{
		DumpMessage("Could not read AGPS data from flash\r\n");
	}


	for (m_count = 0; m_count < NUM_DIAG_COMMANDS; m_count++)
	{
		if (FlashRead((ascii *)g_DiagCfgStructHandle, m_count, sizeof (DIAG_CONFIG),
            (u8 *)(&g_DiagConfig[m_count])) != OK)
		{
			DumpMessage("\r\n Unable to read diag commands from the Flash \r\n");
		}

		//wm_sprintf(g_traceBuf, "g_DiagConfig[%d].ATCmdRepeatCount : %d\r\n",m_count, g_DiagConfig[m_count].ATCmdRepeatCount);
		//DumpMessage(g_traceBuf);
		//wm_sprintf(g_traceBuf, "g_DiagConfig[%d].ATCmdRepeatInterval : %d\r\n",m_count, g_DiagConfig[m_count].ATCmdRepeatInterval);
		//DumpMessage(g_traceBuf);
		//wm_sprintf(g_traceBuf, "g_DiagConfig[%d].ATCommand : %s\r\n",m_count, g_DiagConfig[m_count].ATCommand);
		//DumpMessage(g_traceBuf);
		//wm_sprintf(g_traceBuf, "g_DiagConfig[%d].IsCommandValid : %d\r\n",m_count, g_DiagConfig[m_count].IsCommandValid);
		//DumpMessage(g_traceBuf);

		//Count the number of valid commands available. Store the further commands from this index onwards.
		//If count is 10 then reset back to 0.
		if(g_DiagConfig[m_count].IsCommandValid)
			g_DiagCmdOffset++; 

		//wm_sprintf(g_traceBuf, "g_DiagCmdOffset : %d\r\n\r\n ------------------------------------------- \r\n",g_DiagCmdOffset);
		//DumpMessage(g_traceBuf);
		
	}

	prev_SMSorGPRSMode = g_config.SMSorGPRS;

	// read the last configuration string from flash
	if (FlashRead((ascii *)g_configStringHandle, FLASH_ID, 160, (u8 *)(&g_last_config)) < 0)
	{
		DumpMessage("Could not read configuration string\r\n");
		memset((u8 *)&g_last_config, 0x30, 160);
	}

	// load any reboot reason.
	g_status.RebootAlarm = InternalFlash_ReadRebootAlarm();
	wm_sprintf(g_traceBuf, "Read reboot reason = %c\r\n", g_status.RebootAlarm);
	DumpMessage(g_traceBuf);

	// Load fences from flash.
	g_status.NumFences = 0;
	for (m_count = 0; m_count < NUM_FENCES; m_count++)
	{
		if (FlashRead((ascii *)g_fencesHandle, m_count, sizeof (GEOFENCE),
            (u8 *)(&g_GeoFences[m_count])) != OK)
		{
			DumpMessage("\r\n Unable to read fences from the Flash \r\n");
		}
	}

	// load mode information
	if (InternalFlash_ReadNVMode(&mvs) == -1)
	{
		DumpMessage("Could not read nm mode\r\n");
		Config_SetMode(NO_CHANGE_MODE, 0, NULL, TRUE);
	}
	else
	{
		Config_SetMode(mvs.mode, 0, NULL, FALSE);
	}

	if (g_config.Mode == WALK_MODE)
	{
        SetDogWalkMode(DOGWALK_ENABLED);
	}

	if (g_config.Mode == DOG_PARK_MODE)
	{
        SetDogParkMode(DOGPARK_ENABLED);
		memcpy(&g_DogParkFence, &mvs.DogParkFence, sizeof (GEOFENCE));
	}
	if (mvs.track_dur_rem != 0)
	{
		SetStartupTrackingMode(mvs.track_dur_rem, TRUE);
	}
	else
	{
		SetStartupTrackingMode(0, FALSE);
	}
	DumpMessage("Init status done\r\n");
}


/** @brief Load band valid from flash
 *
 * @return The band setting in the flash or NA band if it fails.
 */
BAND_SETTING LoadBandValue(void)
{
	// attempt to subscribe to the band setting in flash.
	// if this fails, return north american band.
	if (FlashSubscribe((ascii *)g_bandSettingHandle, 1) != OK)
	{
		DumpMessage("Could not subscribe to BandSettingHandle flash\r\n");
		return BAND_DUAL_850_1900;
	}

	DumpMessage("Flash Read Band value!\r\n");

	// attempt to read the setting from the flash.
	// if this fails, return north american band.
	BAND_SETTING flash_band_setting;
	int sReturn = FlashRead((ascii *)g_bandSettingHandle, FLASH_ID,
	                        sizeof (BAND_SETTING), (u8 *)&flash_band_setting);

	if (sReturn != OK)
	{
		WriteBandValue(BAND_DUAL_850_1900);
		return BAND_DUAL_850_1900;
	}

	// make sure the value we read in is valid.
	int ii;
	for (ii = BAND_FIRST; ii < BAND_LAST; ii++)
	{
		if (flash_band_setting == ii)     // found a valid setting, return it.
		{
			return flash_band_setting;
		}
	}
	// if we get here the setting was not valid.
	// return north american band.
	return BAND_DUAL_850_1900;
}


/** @brief Write band value to the flash.
 *
 *   @param The band setting for the flash.
 */
void WriteBandValue(BAND_SETTING band_value)
{
	wm_sprintf(g_traceBuf, "band_value=%d\r\n", band_value);
	DumpMessage(g_traceBuf);
	// check if the band setting is valid.
	bool band_setting_valid = FALSE;
	int ii;
	for (ii = BAND_FIRST; ii < BAND_LAST; ii++)
	{
		if (band_value == ii)
		{
			band_setting_valid = TRUE;
			break;
		}
	}
	// if its not valid, abort the write.
	if (!band_setting_valid)
	{
		return;
	}
	// attempt to subscribe to the flash for the band setting.
	if (FlashSubscribe((ascii *)g_bandSettingHandle, 1) != OK)
	{
		return;
	}
	// write the band setting to the flash.
	FlashWrite((ascii *)g_bandSettingHandle, FLASH_ID,
	           sizeof (BAND_SETTING), (u8 *)&band_value);
}


void LoadAccelCalibration(ACCEL_CALIBRATION_DATA *cab_data)
{
	int tmp_cal[3];
	int ii, sReturn = -1;

	for (ii = 0; ii < 3; ii++)
	{
		sReturn = FlashRead((ascii *)g_accelCalibHandle, ii, sizeof (int), (u8 *)(&tmp_cal[ii]));
		if (sReturn != OK)
		{
			DumpMessage("\r\n Unable to read accel calib from flash \r\n");
			break;
		}

		wm_sprintf(g_traceBuf, "tmp_cal[%d]=%d\r\n", ii, tmp_cal[ii]);
		DumpMessage(g_traceBuf);
	}

	// This is the case where we have not calibrated the device.
	if (sReturn != OK)
	{
		cab_data->x_offset_value = 0;
		cab_data->y_offset_value = 0;
		cab_data->z_offset_value = 0;
	}
	else
	{
		cab_data->x_offset_value = tmp_cal[0];
		cab_data->y_offset_value = tmp_cal[1];
		cab_data->z_offset_value = tmp_cal[2];
	}
}


void WriteAccelCalibrationFlash(ACCEL_CALIBRATION_DATA *cab_data)
{
	int sReturn = -1;

	sReturn = FlashWrite((ascii *)g_accelCalibHandle, 0,
	                     sizeof (int), (u8 *)&(cab_data->x_offset_value));
	if (sReturn != OK)
	{
		DumpMessage("Could not write calibration data\r\n");
	}

	sReturn = FlashWrite((ascii *)g_accelCalibHandle, 1,
	                     sizeof (int), (u8 *)&(cab_data->y_offset_value));
	if (sReturn != OK)
	{
		DumpMessage("Could not write calibration data\r\n");
	}

	sReturn = FlashWrite((ascii *)g_accelCalibHandle, 2,
	                     sizeof (int), (u8 *)&(cab_data->z_offset_value));
	if (sReturn != OK)
	{
		DumpMessage("Could not write calibration data\r\n");
	}
}


/** @brief store fences to FLASH
 *
 * @par Stores the fences in the buffer (g_fences) to flash
 * @param fenceNum
 * @return void
 */
void FencesToFlash(int fenceNum)
{
	s32 sReturn = -1;
	TRACE((1, "Fence to write: %d", fenceNum));

	sReturn = FlashWrite((ascii *)g_fencesHandle, fenceNum,
	                     sizeof (GEOFENCE), (u8 *)(&g_GeoFences[fenceNum]));
	if (sReturn < 0)
	{
		DumpMessage("ERROR: Failed to write to fences to flash \r\n");
	}
}


/** @brief write config string to flash
 *
 * @param cfgstring
 * @return void
 */
void ConfigStringToFlash(u8 *cfgstring)
{
	s32 sReturn = -1;

	sReturn = FlashWrite((ascii *)g_configStringHandle, FLASH_ID,
	                     160, cfgstring);
	if (sReturn < 0)
	{
		DumpMessage("ERROR: Failed to write to fences to flash \r\n");
	}

	DumpMessage("Config string to flash\r\n");
	DumpMessage((char *)cfgstring);
}


/** @brief store log to flash
 *
 * @param logNum
 * @return void
 */
void LogPktToFlash(int logNum)
{
	s32 sReturn = -1;
	TRACE((1, "Log to write: %d", logNum));
	wm_sprintf(g_traceBuf, "Log to write: %d\r\n", logNum);
	DumpMessage(g_traceBuf);

	if (logNum > MAX_NUM_LOG_PKT)
	{
		wm_sprintf(g_traceBuf, "\r\n Log Number exceeds the max number of %d: %d \r\n", MAX_NUM_LOG_PKT, logNum);
		DumpMessage(g_traceBuf);
	}
	else
	{
		sReturn = FlashWrite((ascii *)g_logPacketHandle, logNum,
		                     (sizeof (UINT8) * SMS_TX_BUFFSIZE), (u8 *)(&g_sms_tx));
		if (sReturn < 0)
		{
			DumpMessage("ERROR: Failed to write to log  to flash \r\n");
		}
	}
}


/** @brief Read log packages from flash
 *
 * @param logNum
 * @return void
 */
void LogPktFromFlash(int logNum)
{
	s32 sReturn = -1;
	TRACE((1, "Log to read: %d", logNum));
	wm_sprintf(g_traceBuf, "Log to read: %d\r\n", logNum);
	DumpMessage(g_traceBuf);

	if (logNum > MAX_NUM_LOG_PKT)
	{
		wm_sprintf(g_traceBuf, "\r\n Log Number exceeds the max number of %d: %d \r\n", MAX_NUM_LOG_PKT, logNum);
		DumpMessage(g_traceBuf);
	}
	else
	{
		sReturn = FlashRead((ascii *)g_logPacketHandle, logNum,
		                    (sizeof (UINT8) * SMS_TX_BUFFSIZE),
		                    (u8 *)(&g_sms_tx));
		if (sReturn != OK)
		{
			DumpMessage("\r\n Unable to read log from the Flash \r\n");
		}
	}
}


/** @brief Initialize internal flash.
 * @par
 * Stores values to flash and subscribes to flash during factor initilization.
 *
 * @return void
 */
void StoreToFlash()
{
	s32 sReturn = -1;

	g_current_fix_32.lat_32 = 0x00;
	g_current_fix_32.long_32 = 0x00;

	sReturn = FlashSubscribe((ascii *)g_configStructHandle, 1);
	sReturn = FlashWrite((ascii *)g_configStructHandle, FLASH_ID, sizeof (CONFIG), (u8 *)(&g_config));

	sReturn = FlashSubscribe((ascii *)g_fencesHandle, NUM_FENCES);
}


/** @brief Dumps out the fences stored in flash to the debug port
 *
 * @return void
 */
void DumpFenceFlash()
{
	int m_fence_cnt;
	int m_post_cnt;
	s32 sReturn = -1;
	for (m_fence_cnt = 0; m_fence_cnt < NUM_FENCES; m_fence_cnt++)
	{
		sReturn = FlashRead((ascii *)g_fencesHandle, m_fence_cnt
		                    , sizeof (GEOFENCE)
		                    , (u8 *)(&g_GeoFences[m_fence_cnt]));
		if (sReturn != OK)
		{
			DumpMessage("\r\n Unable to read fences from the Flash \r\n");
		}
	}

	for (m_fence_cnt = 0; m_fence_cnt < NUM_FENCES; m_fence_cnt++)
	{
		TRACE((1, "Fence: %d", m_fence_cnt));

		wm_sprintf(g_traceBuf, "\r\nFence,enabled,dir,days,start time,stop time\r\n%d,%d,%d,%d,%d,%d\r\n", m_fence_cnt,
		           g_GeoFences[m_fence_cnt].enable, g_GeoFences[m_fence_cnt].alarm_dir,
		           g_GeoFences[m_fence_cnt].days, g_GeoFences[m_fence_cnt].time.start_time, g_GeoFences[m_fence_cnt].time.stop_time);
		DumpMessage(g_traceBuf);
		DumpMessageUSB(g_traceBuf, 1);

		for (m_post_cnt = 0; m_post_cnt < NUM_POSTS_PER_FENCE; m_post_cnt++)
		{
			wm_sprintf(g_traceBuf, "%x,%x\r\n", (unsigned int)g_GeoFences[m_fence_cnt].posts[m_post_cnt].lat_32, (unsigned int)g_GeoFences[m_fence_cnt].posts[m_post_cnt].long_32);
			DumpMessage(g_traceBuf);
			DumpMessageUSB(g_traceBuf, 1);
		}
	}
}


/** @brief load default configuration structure
 *
 * @return void
 */
void load_defaults(void)
{
	LoadDefaultConfigStruct();
}


/** @brief write configuration to FLASH
 *
 * @return void
 */
void ConfigToFlash(void)
{
	memcpy(&flashConfig, &g_config, sizeof (CONFIG));

	// Do not allow some modes to be written to flash.
	flashConfig.Mode = NO_CHANGE_MODE;
	flashConfig.WaypointDLData = NO_DOWNLOAD;

	FlashSubscribe((ascii *)g_configStructHandle, 1);
	FlashWrite((ascii *)g_configStructHandle, FLASH_ID, sizeof (CONFIG), (u8 *)(&flashConfig));
}

void DiagConfigToFlash(UINT8 CommandNum)
{
	s32 sReturn = -1;

	//FlashSubscribe((ascii *)g_DiagCfgStructHandle, 10);
	//wm_sprintf(g_traceBuf, "CommandNum : %d\r\n",CommandNum);
	//DumpMessage(g_traceBuf);
	
	sReturn = FlashWrite((ascii *)g_DiagCfgStructHandle, CommandNum, sizeof(DIAG_CONFIG), (u8 *)(&g_DiagConfig[CommandNum]));

	if (sReturn < 0)
	{
		wm_sprintf(g_traceBuf,"ERROR: Failed to write to g_DiagCfgStructHandle, sReturn:%ld", sReturn);
		DumpMessage(g_traceBuf);
	}

	DumpMessage("Diag Config stored to flash successfully\r\n");
	
}

void ModeConfigToFlash(void)
{
	s32 sReturn = -1;

	FlashSubscribe((ascii *)g_ModeconfigStructHandle, 1);
	sReturn = FlashWrite((ascii *)g_ModeconfigStructHandle, FLASH_ID, sizeof (MODE_CONFIG), (u8 *)(&g_ModeConfig));
	if (sReturn < 0)
	{
		DumpMessage("ERROR: Failed to write to g_ModeconfigStructHandle \r\n");
	}

	DumpMessage("Mode Config stored to flash successfully\r\n");

}

void AGPSDataToFlash(void)
{
	s32 sReturn = -1;

	FlashSubscribe((ascii *)g_AGPSDataHandle, 1);
	sReturn = FlashWrite((ascii *)g_AGPSDataHandle, FLASH_ID, sizeof (AGPS_DATA), (u8 *)(&g_AGPSData));
	if (sReturn < 0)
	{
		DumpMessage("ERROR: Failed to write to g_AGPSDataHandle \r\n");
	}

	DumpMessage("AGPS Data stored to flash successfully\r\n");
}



/** @brief Load default configuration to stucture
 *
 * @return void
 */
void LoadDefaultConfigStruct(void)
{
	int i;
	const char def_ip[] = "64.244.192.252";

	g_config.SMSorGPRS = 1;                 // set to GPRS mode.
	g_config.WaypointInterval = 10;
	g_config.TrackingInterval = 9;          //10 mins default. Value of 9 refers to 10 mins.
	g_config.MotionAlarmThresh = 0;
	g_config.AccelThreshWake = 0x20;
	g_config.AccelDurationWake = 100;
	g_config.AccelDurationSleep = 10000;
	g_config.BreadCrumbMode = DISABLE;
	g_config.Mode = NO_CHANGE_MODE;
	g_config.NumFences = 0;
	g_config.GPSAlert = GPS_ALERT_ON_OTA_AGPS_ON;
	g_config.GSMAlertThresh = DISABLE;
	g_config.BattAlert = BATT_LEVEL_1_SHUTDOWN_ALERT_ON;
	g_config.PowerDiscAlert = DISABLE;
	g_config.OverSpeedAlertThresh = 100;
	g_config.PowerDownDisable = DISABLE;
	g_config.VibrationMotorPattern = 0;
	g_config.VibrationMotorDelayEn = ENABLE;
	g_config.SOSAlert = ENABLE_SOS;
	g_config.WaypointDLData = NO_DOWNLOAD;
	g_config.LEDPattern = 0;
	for (i = 0; i < NUM_LEDS; i++)
	{
		g_config.LEDEn[i] = ENABLE;
	}
	g_config.TimedWakeup = 0;
	for (i = 0; i < NUM_FENCES; i++)
	{
		g_config.FencesActive[i] = 0;
	}
	g_config.InvisibleOp = DISABLE;
	g_config.MagneticSnsEn = DISABLE;
	g_config.CriticalConfirm = 0;
	for (i = 0; i < MAX_PHNUM_LEN; i++)
	{
		g_config.ServerPhoneNum[i] = '0';
	}
	memcpy(&g_config.ServerIPAddr, def_ip, 15);
	g_config.ServerPortNum = 14444;


	//Mode Config defaults.
	g_ModeConfig.UseCaseNum      = 0x00;
	g_ModeConfig.Mode            = 0x04;
	g_ModeConfig.AnalogThreshold = 0x0A;
	g_ModeConfig.SampleCount     = 0x01;
	g_ModeConfig.GPIO5Config	 = 0x01;  //Default as input
	g_ModeConfig.GPIO7Config	 = 0x01;  //Default as input
	g_ModeConfig.I2CDataLen		 = 0x01;
	g_ModeConfig.TxPattern       = 0x00;
}


/** @brief Check for factory setup
 *
 * @return 0 if factory setup required, 1 otherwise
 */
int CheckFactorySetup(void)
{
	if (CheckFactorySetupSpiFlash())
	{
		DumpMessage("Factory setup required\r\n");
		return 1;
	}
	else
	{
		DumpMessage("No factory setup required\r\n");
		return 0;
	}
}


/**@}*/
#define MAX_APN_NAME_LENGTH         100
#define MAX_APN_LOGIN_LENGTH        100
#define MAX_APN_PASSWORD_LENGTH     100

static char APNName[MAX_APN_NAME_LENGTH];
static char APNLogin[MAX_APN_LOGIN_LENGTH];
static char APNPassword[MAX_APN_PASSWORD_LENGTH];

int InternalFlash_WriteAPNName(const char *name)
{
	memset(APNName, 0x00, MAX_APN_NAME_LENGTH);
	if (name != NULL)
	{
		memcpy(APNName, name, strlen(name));
	}

	if (FlashSubscribe((ascii *)g_APNNameHandle, 1) != OK)
	{
		return -1;
	}

	if (FlashWrite((ascii *)g_APNNameHandle, FLASH_ID, MAX_APN_NAME_LENGTH, (u8 *)APNName) != OK)
	{
		return -1;
	}

	return 0;
}


int InternalFlash_LoadNonVolatileStatus(NON_VOLATILE_STATUS *nvs)
{
	if (FlashSubscribe((ascii *)g_NVSHandle, 1) != OK)
	{
		return -1;
	}

	if (FlashRead((ascii *)g_NVSHandle, FLASH_ID, sizeof (NON_VOLATILE_STATUS), (u8 *)nvs) != OK)
	{
		return -1;
	}
	return 0;
}


int InternalFlash_WriteNonVolatileStatus(void)
{
	NON_VOLATILE_STATUS nvs;

	if (FlashSubscribe((ascii *)g_NVSHandle, 1) != OK)
	{
		return -1;
	}

	nvs.NumFences = g_status.NumFences;
	nvs.RebootAlarm = g_status.RebootAlarm;
	nvs.NumLogPktToSend = g_status.NumLogPktToSend;
	nvs.LogPktToSendLatest = g_status.LogPktToSendLatest;

	if (FlashWrite((ascii *)g_NVSHandle, FLASH_ID, sizeof (NON_VOLATILE_STATUS), (u8 *)&nvs) != OK)
	{
		return -1;
	}

	return 0;
}


char * InternalFlash_GetAPNName(void)
{
	memset(APNName, 0x00, MAX_APN_NAME_LENGTH);

	if (FlashSubscribe((ascii *)g_APNNameHandle, 1) != OK)
	{
		return NULL;
	}

	if (FlashRead((ascii *)g_APNNameHandle, FLASH_ID, MAX_APN_NAME_LENGTH, (u8 *)APNName) != OK)
	{
		return NULL;
	}

	return APNName;
}


int InternalFlash_WriteAPNLogin(const char *login)
{
	memset(APNLogin, 0x00, MAX_APN_LOGIN_LENGTH);
	if (login != NULL)
	{
		memcpy(APNLogin, login, strlen(login));
	}

	if (FlashSubscribe((ascii *)g_APNLoginHandle, 1) != OK)
	{
		return -1;
	}

	if (FlashWrite((ascii *)g_APNLoginHandle, FLASH_ID, MAX_APN_LOGIN_LENGTH, (u8 *)APNLogin) != OK)
	{
		return -1;
	}

	return 0;
}


char * InternalFlash_GetAPNLogin(void)
{
	memset(APNLogin, 0x00, MAX_APN_LOGIN_LENGTH);

	if (FlashSubscribe((ascii *)g_APNLoginHandle, 1) != OK)
	{
		return NULL;
	}

	if (FlashRead((ascii *)g_APNLoginHandle, FLASH_ID, MAX_APN_LOGIN_LENGTH, (u8 *)APNLogin) != OK)
	{
		return NULL;
	}
	return APNLogin;
}


int InternalFlash_WriteAPNPassword(const char *passwd)
{
	memset(APNPassword, 0x00, MAX_APN_PASSWORD_LENGTH);

	if (passwd != NULL)
	{
		memcpy(APNPassword, passwd, strlen(passwd));
	}

	if (FlashSubscribe((ascii *)g_APNPasswordHandle, 1) != OK)
	{
		return -1;
	}

	if (FlashWrite((ascii *)g_APNPasswordHandle, FLASH_ID, MAX_APN_PASSWORD_LENGTH, (u8 *)APNPassword) != OK)
	{
		return -1;
	}

	return 0;
}


char * InternalFlash_GetAPNPassword(void)
{
	memset(APNPassword, 0x00, MAX_APN_PASSWORD_LENGTH);

	if (FlashSubscribe((ascii *)g_APNPasswordHandle, 1) != OK)
	{
		return NULL;
	}

	if (FlashRead((ascii *)g_APNPasswordHandle, FLASH_ID, MAX_APN_PASSWORD_LENGTH, (u8 *)APNPassword) != OK)
	{
		return NULL;
	}
	return APNPassword;
}


int InternalFlash_WriteRebootAlarm(char reboot_alarm)
{
	if (FlashSubscribe((ascii *)g_RebootAlarmHandle, 1) != OK)
	{
		return -1;
	}

	if (FlashWrite((ascii *)g_RebootAlarmHandle, FLASH_ID, 1, (u8 *)&reboot_alarm))
	{
		return -1;
	}
	return 0;
}


char InternalFlash_ReadRebootAlarm(void)
{
	char ret_val = 'N';

	if (FlashSubscribe((ascii *)g_RebootAlarmHandle, 1) != OK)
	{
		DumpMessage("Could not subscribe to reboot alarm handle\r\n");
		return 'N';
	}

	if (FlashRead((ascii *)g_RebootAlarmHandle, FLASH_ID, 1, (u8 *)&ret_val))
	{
		DumpMessage("Could not read reboot alarm handle\r\n");
		return 'N';
	}

	return ret_val;
}


int InternalFlash_WriteNVMode(MODE_NONVOLATILE_STORAGE *mnvs)
{
	if (FlashSubscribe((ascii *)g_NVModeHandle, 1) != OK)
	{
		return -1;
	}

	if (FlashWrite((ascii *)g_NVModeHandle, FLASH_ID, sizeof (MODE_NONVOLATILE_STORAGE), (u8 *)mnvs))
	{
		return -1;
	}

	return 0;
}


int InternalFlash_ReadNVMode(MODE_NONVOLATILE_STORAGE *mnvs)
{
	if (FlashSubscribe((ascii *)g_NVModeHandle, 1) != OK)
	{
		return -1;
	}

	if (FlashRead((ascii *)g_NVModeHandle, FLASH_ID, sizeof (MODE_NONVOLATILE_STORAGE), (u8 *)mnvs))
	{
		return -1;
	}
	return 0;
}
