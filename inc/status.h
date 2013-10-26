#ifndef __STATUS__
#define __STATUS__

#include "fence.h"
#include "gps.h"
#include "SMShandling.h"
#include "LED.h"
#include "ConfigSettings.h"

#define MAX_NUM_LOG_PKT    10

/* Section of status packet structure.  See the status Packet in the
 * protocol document for a description of the members */
typedef struct
{
	UINT8 BattLevel;
	char SystemFailure;
	char GPSStatus;
	UINT8 GPSSignalStrength1;
	UINT8 GPSSignalStrength2;
	UINT8 NumBirds;
	char GSMStatus;
	char GSMSignalStrength;
	UINT8 CellID[4];
	UINT8 LocationArea[4];
	char ValidAccessCnt;
	char BadAccessCnt;
	UINT8 NumFences;
	MODES Mode;
	char SOSAlarm;
	char GPSAlarm;
	char BattAlarm;
	char GSMAlarm;
	char FenceAlarm;
	char OverSpeedAlarm;
	UINT8 CurrentFenceIn;
	char RebootAlarm;
	char MotionAlarm;
	char PowerDisconnAlarm;
	UINT8 NumLogPktToSend;
	UINT8 LogPktToSendLatest;
} STATUS;

typedef struct
{
	UINT8 NumFences;
	char RebootAlarm;
	UINT8 NumLogPktToSend;
	UINT8 LogPktToSendLatest;
} NON_VOLATILE_STATUS;

typedef enum
{
	GSM_NOT_WORKING,
	GSM_WORKING
} GSM_STATUS;

typedef enum
{
	RESET_REASON_WATCHDOG_APP_RESET = 'W',
	RESET_REASON_GSM_WATCH = 'G',
	RESET_REASON_COMM_MODE_CHANGE = 'C',
	RESET_REASON_SOFTWARE_UPDATE = 'S',
	RESET_REASON_DIAGNOSE_MODE = 'D',
	RESET_REASON_UNKNOWN = 'U',
	RESET_REASON_ACCEL = 'A'
} RESET_REASON;

extern STATUS g_status;

void InitStatus(void);
int ClearAlarms(void);
void Status_setGSMStatus(GSM_STATUS gsm_stat);
GSM_STATUS Status_getGSMStatus(void);

void Status_setResetReason(RESET_REASON);
#endif
