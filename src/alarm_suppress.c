#include <adl_global.h>
#include "alarm_suppress.h"
#include "GPSCtrl.h"

#define NUM_SUPPRESS_ALARMS         4

#define GPS_ALARM_SUPPRESS_TIME     (10 * 60)
#define GSM_ALARM_SUPPRESS_TIME     (10 * 60)
#define BAT_ALARM_SUPPRESS_TIME     (30 * 60)
#define ACC_ALARM_SUPPRESS_TIME     (2 * 60)

static adl_rtcTimeStamp_t last_alarm_times[NUM_SUPPRESS_ALARMS] = { { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 } };

static const unsigned int ALARM_SUPPRESS_TIMES[4] =
{
	GPS_ALARM_SUPPRESS_TIME,
	GSM_ALARM_SUPPRESS_TIME,
	BAT_ALARM_SUPPRESS_TIME,
	ACC_ALARM_SUPPRESS_TIME
};

static adl_rtcTimeStamp_t f_get_alarm_time(ALARM_SUPPRESS_TYPE alarm);

extern char g_traceBuf[256];

/** @brief Get the status of a paricular alarm
 *
 * @params alarm
 */
ALARM_EXP_STATUS alarm_suppress_status(ALARM_SUPPRESS_TYPE alarm)
{
	adl_rtcTimeStamp_t current_time, delta_time, alarm_time;

	alarm_time = f_get_alarm_time(alarm);

	// This is the case where we have not had this particular alarm before.
	if ((alarm_time.TimeStamp == 0) && (alarm_time.SecondFracPart == 0))
	{
		return ALARM_EXPIRED;
	}

	// get the current time.
	if (GetConvertTime(&current_time) < 0)
	{
		return ALARM_EXPIRED;
	}

	// read the time from the last battery alarm and calculate how longs its been
	if (adl_rtcDiffTime(&current_time, &alarm_time, &delta_time) < 0)
	{
		return ALARM_EXPIRED;
	}

	// if the alarm was generated long enough ago, it is expired.
	if (delta_time.TimeStamp >= ALARM_SUPPRESS_TIMES[alarm])
	{
		return ALARM_EXPIRED;
	}

	return ALARM_NOT_EXPIRED;
}


/** @brief Set the time of a particular alarm.
 *
 * @params alarm
 */
void alarm_suppress_set_alarm_time(ALARM_SUPPRESS_TYPE alarm)
{
	// set the alarm to the curren time. If this fails, set it back to the initial value.
	if (GetConvertTime(&last_alarm_times[alarm]) < 0)
	{
		last_alarm_times[alarm].TimeStamp = 0;
		last_alarm_times[alarm].SecondFracPart = 0;
	}
}


/** @brief get the time of a particular alarm.
 *
 * @params alarm
 */
static adl_rtcTimeStamp_t f_get_alarm_time(ALARM_SUPPRESS_TYPE alarm)
{
	return last_alarm_times[alarm];
}
