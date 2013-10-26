#ifndef __alarm_suppress__
#define __alarm_suppress__

typedef enum
{
	GPS_ALARM_SUP = 0,
	GSM_ALARM_SUP,
	BAT_ALARM_SUP,
	ACC_ALARM_SUP
} ALARM_SUPPRESS_TYPE;

typedef enum
{
	ALARM_EXPIRED,
	ALARM_NOT_EXPIRED,
} ALARM_EXP_STATUS;

ALARM_EXP_STATUS alarm_suppress_status(ALARM_SUPPRESS_TYPE alarm);
void alarm_suppress_set_alarm_time(ALARM_SUPPRESS_TYPE alarm);

#endif
