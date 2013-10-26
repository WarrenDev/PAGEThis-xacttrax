#ifndef __INTERNAL_FLASH_H__
#define __INTERNAL_FLASH_H__

#include "i2c_accelerometer.h"
#include "status.h"

typedef enum
{
	BAND_FIRST = 0,
	BAND_MONO_850 = 0,
	BAND_MONO_900E,
	BAND_MONO_1800,
	BAND_MONO_1900,
	BAND_DUAL_850_1900,
	BAND_DUAL_900E_1800,
	BAND_DUAL_900E_1900,
	BAND_LAST
} BAND_SETTING;

typedef struct
{
	MODES mode;
	unsigned int track_dur_rem;
	GEOFENCE DogParkFence;
} MODE_NONVOLATILE_STORAGE;

extern void LoadFromFlash(void);
extern void FencesToFlash(int fenceNum);
extern void DumpFenceFlash(void);
extern void StoreToFlash(void);
extern void load_defaults(void);
extern void ConfigToFlash(void);
extern void ConfigStringToFlash(u8 *cfgstring);
extern void LogPktToFlash(int logNum);
extern int CheckFactorySetup(void);
extern void LogPktFromFlash(int logNum);
extern void LoadAccelCalibration(ACCEL_CALIBRATION_DATA *cab_data);
extern void WriteAccelCalibrationFlash(ACCEL_CALIBRATION_DATA *cab_data);
extern int InternalFlash_WriteAPNName(const char *name);
extern char * InternalFlash_GetAPNName(void);
extern int InternalFlash_WriteAPNLogin(const char *login);
extern char * InternalFlash_GetAPNLogin(void);
extern int InternalFlash_WriteAPNPassword(const char *passwd);
extern char * InternalFlash_GetAPNPassword(void);

extern BAND_SETTING LoadBandValue(void);
extern void WriteBandValue(BAND_SETTING band_value);
extern int InternalFlash_WriteNonVolatileStatus(void);
extern int InternalFlash_LoadNonVolatileStatus(NON_VOLATILE_STATUS *nvs);
extern int InternalFlash_WriteRebootAlarm(char reboot_alarm);
extern char InternalFlash_ReadRebootAlarm(void);
int InternalFlash_WriteNVMode(MODE_NONVOLATILE_STORAGE *mnvs);
int InternalFlash_ReadNVMode(MODE_NONVOLATILE_STORAGE *mnvs);

void DiagConfigToFlash(UINT8 CommandNum);
void ModeConfigToFlash(void);
void AGPSDataToFlash(void);

#endif
