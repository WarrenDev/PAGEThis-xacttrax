#ifndef _ACCELEROMETER_H__
#define _ACCELEROMETER_H__

#include "adl_global.h"
#include "pistd.h"

#define ACCEL_CS_GPIO    34

typedef struct
{
	int irq_high_slow_idle;
	int threshold_bad;
} ACCEL_ERROR_STATE;

void CheckAccelState(u8 timerID, void *Context);
int SetAccelTriggerThreshold(UINT8 AccelThreshWake);
void InitAccelerometer(void);
void ClearIRQ(void);
int SetAccelLevelMode(void);
int SetAccelMeasureMode(void);
void AccelCalibrate(void);
void AccelReadData(void);
void CheckAccelSlowIdle(void);
void AccelSimulateLockUp(void);
void ConfigSettings_setOTA57(bool ota57);
void SimulateAccelTrigger();
void ActivateAccelOnPowerUp();


#endif
