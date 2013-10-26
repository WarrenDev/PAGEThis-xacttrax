/** @addtogroup GPIO
 *@{*/

/*H************************************************************************
 */

/** @file    LED.c
 *
 *   @brief   Controls the LEDs.
 *
 *   @details Blinks the LEDS to indicates status and mode.
 *
 *   @note    Other help for the reader, including references.
 *
 *
 *//*
 *
 * CHANGES :
 *
 * REF NO  DATE     WHO      DETAIL
 *         19Jun09  AndyB    First version
 * NNNNNN  DDMMMYY  Name     Name of function/item changed
 *
 *H*/

#include "LED.h"
#include "pistd.h"
#include "ConfigSettings.h"
#include "status.h"
#include "diagnose.h"
#include "gpioTest.h"
#include "GPSCtrl.h"
#include "XactUtilities.h"
#include "dogwalk.h"
#include "button.h"
#include "Anolclient.h"

#define LED_TRACE_LEVEL    6 // TODO move this?
// Special modes.

extern int g_ShutDownReq;

// prototypes.
static void EvalLED(int LedNum);

static const int LEDGpio[NUM_LEDS] =
{
	POWER_LED_RED, POWER_LED_GREEN, GSM_LED_RED, GSM_LED_GREEN, GPS_LED_RED, GPS_LED_GREEN
};
static s32 LEDHandles[NUM_LEDS] =
{
	ERROR, ERROR, ERROR, ERROR, ERROR, ERROR
};

static GPS_LED_STATUS gps_led_status = FIX_BAD_LED;

// Tables to define blinking patterns. Each row represents the on/off duration for a partcular
// mode. Conditional 'Ons' are indicated by masking the upper bits with a particular pattern.
// The modes are in this order:
// 0) OFF Mode.
// 1) Normal Mode.
// 2) Walk Mode.
// 3) Park Mode.
// 4) Shutdown pattern
// 5) SOS Pattern.
static const unsigned int PowerLedRedPatterns[MAX_PATTERNS][MAX_ON_OFF_LED] =
{
	{ DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE },
	{ LED_ON_DURATION_NORMAL | POWER_STATUS_MSK, LED_OFF_DURATION_NORMAL, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE },
	{ DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE },
	{ LED_ON_DURATION_NORMAL, LED_OFF_DURATION_WALK, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE },
	{ LED_ON_DURATION_NORMAL, LED_OFF_DURATION_WALK, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE },
	{ SOS_DURATION, SOS_DURATION, SOS_DURATION | LED_OFF_MSK, SOS_DURATION, SOS_DURATION | LED_OFF_MSK, DONE, DONE, DONE, DONE, DONE }
};

static const unsigned int PowerLedGreenPatterns[MAX_PATTERNS][MAX_ON_OFF_LED] =
{
	{ DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE },
	{ LED_ON_DURATION_NORMAL | POWER_STATUS_MSK_GREEN, LED_OFF_DURATION_NORMAL, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE },
	{ LED_ON_DURATION_NORMAL, LED_OFF_DURATION_WALK, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE },
	{ LED_ON_DURATION_NORMAL, LED_OFF_DURATION_WALK, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE },
	{ LED_ON_DURATION_NORMAL, LED_OFF_DURATION_WALK, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE },
	{ DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE }
};

static const unsigned int GSMLedRedPatterns[MAX_PATTERNS][MAX_ON_OFF_LED] =
{
	{ DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE },
	{ DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE },
	{ DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE },
	{ LED_ON_DURATION_NORMAL, LED_OFF_DURATION_WALK, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE },
	{ LED_ON_DURATION_NORMAL, LED_OFF_DURATION_WALK, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE },
	{ SOS_DURATION | LED_OFF_MSK, SOS_DURATION, SOS_DURATION, SOS_DURATION, SOS_DURATION | LED_OFF_MSK, DONE, DONE, DONE, DONE, DONE }
};
static const unsigned int GSMLedGreenPatterns[MAX_PATTERNS][MAX_ON_OFF_LED] =
{
	{ DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE },
	{ LED_ON_DURATION_NORMAL | GSM_STATUS_MSK, LED_OFF_DURATION_NORMAL, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE },
	{ LED_ON_DURATION_NORMAL, LED_OFF_DURATION_WALK, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE },
	{ LED_ON_DURATION_NORMAL, LED_OFF_DURATION_WALK, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE },
	{ LED_ON_DURATION_NORMAL, LED_OFF_DURATION_WALK, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE },
	{ DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE }
};
static const unsigned int GPSLedRedPatterns[MAX_PATTERNS][MAX_ON_OFF_LED] =
{
	{ DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE },
	{ LED_ON_DURATION_NORMAL | GPS_STATUS_MSK, LED_OFF_DURATION_NORMAL, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE },
	{ DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE },
	{ LED_ON_DURATION_NORMAL, LED_OFF_DURATION_WALK, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE },
	{ LED_ON_DURATION_NORMAL, LED_OFF_DURATION_WALK, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE },
	{ SOS_DURATION | LED_OFF_MSK, SOS_DURATION, SOS_DURATION | LED_OFF_MSK, SOS_DURATION, SOS_DURATION, DONE, DONE, DONE, DONE, DONE }
};
static const unsigned int GPSLedGreenPatterns[MAX_PATTERNS][MAX_ON_OFF_LED] =
{
	{ DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE },
	{ LED_ON_DURATION_NORMAL | GPS_STATUS_MSK_GREEN, LED_OFF_DURATION_NORMAL, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE },
	{ LED_ON_DURATION_NORMAL, LED_OFF_DURATION_WALK, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE },
	{ LED_ON_DURATION_NORMAL, LED_OFF_DURATION_WALK, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE },
	{ LED_ON_DURATION_NORMAL, LED_OFF_DURATION_WALK, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE },
	{ DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE, DONE }
};

// Table for indexing the above tables.
static const unsigned int *LEDPatterns[NUM_LEDS] =
{
	(unsigned int *)PowerLedRedPatterns, (unsigned int *)PowerLedGreenPatterns,
	(unsigned int *)GSMLedRedPatterns, (unsigned int *)GSMLedGreenPatterns,
	(unsigned int *)GPSLedRedPatterns, (unsigned int *)GPSLedGreenPatterns
};

static unsigned int shutdown_trig = 0;

int check_shutdown_start(void)
{
	static int sd_started = 0;
	if (shutdown_trig != 0)
	{
		sd_started = 1;
	}
	return sd_started;
}


/** @brief   Shutoff all LED
 *
 *
 * @return void
 */
void ShutOffLED(void)
{
	int i;
	for (i = 0; i < NUM_LEDS; i++)
	{
		GpioWrite(LEDGpio[i], &LEDHandles[i], FALSE);
	}
}


void TurnOnLED(void)
{
	int i;
	for (i = 0; i < NUM_LEDS; i++)
	{
		GpioWrite(LEDGpio[i], &LEDHandles[i], TRUE);
	}
}


void TurnOnLEDNum(int lednum)
{
	GpioWrite(LEDGpio[lednum], &LEDHandles[lednum], TRUE);
}


void TurnOffLEDNum(int lednum)
{
	GpioWrite(LEDGpio[lednum], &LEDHandles[lednum], FALSE);
}


/*F***************************************************************
 *
 *   NAME:    BlinkLED
 */

/** @brief   controls all LEDs
 *
 *   @param timerid [in] u8  The timer ID
 *   @param context [in] void * The context of current timer
 *
 *
 *
 *//*
 *
 * CHANGES :
 * REF NO  DATE     WHO      DETAIL
 *         19Jun09  AndyB    First version
 * NNNNNN  DDMMMYY  Name     Name of function/item changed
 *F*/
void BlinkLED(u8 timerid, void *context)
{
	(void)timerid;
	(void)context;

	// check if we are in diagnose mode. If so, do not
	// exercise the LEDs normally.
	if (diagnose_get_mode() == DIAGNOSE_ON)
	{
		return;
	}

	if (g_config.PowerDownDisable == DISABLE)
	{
        static u32 shutdownCount = 0;
        static u32 shutdownBlock = 0;
        s32 inputButtonValue;
		// keep track of how long the button is pressed for.
		// if it exceeds 5 seconds, display the shutown LEDs.
		if (!GetSWButtonOvrd((int *)&inputButtonValue))
		{
			inputButtonValue = InputButtonActive() ? 1 : 0;
		}

		if (inputButtonValue == 0)
		{
			shutdownCount = 0;
			shutdownBlock = 0;
		}
		else 
		{
			shutdownCount++;
			if ((shutdownCount >= SHUTDOWN_THRESH) && !shutdownBlock)
			{
				DumpMessage("System shutdown requested!\r\n");
				shutdown_trig = 1;
				if (g_config.ServerLEDOverride == DISABLE)
				{
					g_config.LEDPattern = PATTERN_SHUTDOWN;
					shutdownBlock = 1;
				}
			}
		}
	}

	if ((shutdown_trig != 0) && (shutdown_trig < 100))
	{
		shutdown_trig++;
	}
	else if (shutdown_trig == 100)
	{
		shutdown_trig = 0;
	}

	if (shutdown_trig == 100)
	{
		g_ShutDownReq = 1;
		g_config.LEDPattern = PATTERN_NORMAL;
	}

    {
        static u8 SOSMode = 0;
        static adl_rtcTimeStamp_t SOSTimeStamp;
        adl_rtcTimeStamp_t CurrentTimeStamp;

        GetConvertTime(&CurrentTimeStamp);

        if ((g_config.SOSAlert == DISABLE_SOS) || (CurrentTimeStamp.TimeStamp - SOSTimeStamp.TimeStamp > SOS_BLINK_TIME))
        {
            SOSMode = 0;
        }

        // Set the patterns based on the state of the system.
        if (g_config.ServerLEDOverride == DISABLE)
        {
            if ((g_status.SOSAlarm == 'Y') || SOSMode)
            {
                g_config.LEDPattern = PATTERN_SOS;
                if (!SOSMode)
                {
                    GetConvertTime(&SOSTimeStamp);
                }
                SOSMode = 1;
            }
            else
            {
                if (g_config.LEDPattern != PATTERN_SHUTDOWN)
                {
                    if (GetDogWalkMode() == DOGWALK_ENABLED)
                    {
                        g_config.LEDPattern = PATTERN_WALK;
                    }
                    else if (GetDogParkMode() == DOGPARK_ENABLED)
                    {
                        g_config.LEDPattern = PATTERN_PARK;
                    }
                    else if (!g_config.ServerLEDOverride)
                    {
                        g_config.LEDPattern = PATTERN_NORMAL;
                    }
                }
            }
        }
    }

	// Evaluate every LED.
    {
        int i;
        for (i = 0; i < NUM_LEDS; i++)
        {
            EvalLED(i);
        }
    }
}


/*F***************************************************************
 *
 *   NAME:    EvalLED
 */

/** @brief   Evaluate the LED patterns
 *   @param LedNum [in] The LED number to evaluate.
 *
 *
 *//*
 *
 * CHANGES :
 * REF NO  DATE     WHO      DETAIL
 *         19Jun09  AndyB    First version
 * NNNNNN  DDMMMYY  Name     Name of function/item changed
 *F*/
static void EvalLED(int LedNum)
{
	static int patternPosition[NUM_LEDS] =
	{
		0, 0, 0, 0, 0, 0
	};
	static unsigned int timerCount[NUM_LEDS] =
	{
		0, 0, 0, 0, 0, 0
	};
	static UINT8 LEDPatternPrev = 0;

	unsigned int currentValue, currentValueMsked;
	unsigned int LEDStateMask;


	// Check if this LED is enabled and make sure we are not in invisible mode.
	// If its not enabled, reset everything and shut it off.

	if (!g_config.LEDEn[LedNum] || (g_config.InvisibleOp == ENABLE))  
	{
		patternPosition[LedNum] = 0;
		timerCount[LedNum] = 0;
		GpioWrite(LEDGpio[LedNum], &LEDHandles[LedNum], FALSE);
		return;
	}

	// first check if the LED pattern has changed.
	// and reset everything if it has.
	if (LEDPatternPrev != g_config.LEDPattern)
	{
        int i;
		for (i = 0; i < NUM_LEDS; i++)
		{
			patternPosition[i] = 0;
			timerCount[i] = 0;
		}
	}
	LEDPatternPrev = g_config.LEDPattern;

	// look into the LED pattern table
	//  currentValue = LEDPatterns[LedNum][g_config.LEDPattern][patternPosition[LedNum]];
	currentValue = *(LEDPatterns[LedNum] + g_config.LEDPattern * MAX_ON_OFF_LED + patternPosition[LedNum]);

	TRACE((LED_TRACE_LEVEL, "CurrentValue %x", currentValue));
	TRACE((LED_TRACE_LEVEL, "LEDPattern %x", g_config.LEDPattern));

	currentValueMsked = currentValue & ~LED_STATE_MASK;

	TRACE((LED_TRACE_LEVEL, "TimerCount %x", timerCount[LedNum]));
	TRACE((LED_TRACE_LEVEL, "value maked %x", currentValueMsked));

	// if value is set to DONE, got back to the beginning.
	if (currentValueMsked == DONE)
	{
		TRACE((LED_TRACE_LEVEL, "LED %d Done", LedNum));
		if ((LedNum >= 2) || !IsUSBConnected())
		{
			GpioWrite(LEDGpio[LedNum], &LEDHandles[LedNum], FALSE);
		}
		timerCount[LedNum] = 0;
		patternPosition[LedNum] = 0;
		// that should be all we do.
		return;
	}     // Odd number positions, shut off the LED.
	else if (patternPosition[LedNum] & 0x1)
	{
		TRACE((LED_TRACE_LEVEL, "Led %d Off patternPostion=%d", LedNum, patternPosition[LedNum]));
		if ((LedNum >= 2) || !IsUSBConnected())
		{
			GpioWrite(LEDGpio[LedNum], &LEDHandles[LedNum], FALSE);
		}
	}     // Even number positions, evaluate system state and turn on LEDs.
	else
	{
		LEDStateMask = LED_STATE_MASK & currentValue;

		switch (LEDStateMask)
		{
		case POWER_STATUS_MSK_GREEN:
			if (g_status.BattLevel >= 2)
			{
				TRACE((LED_TRACE_LEVEL, "Led %d On patternPostion=%d", LedNum, patternPosition[LedNum]));
				GpioWrite(LEDGpio[LedNum], &LEDHandles[LedNum], TRUE);
			}
			break;

		case POWER_STATUS_MSK:
			// if we are running, we have power.
			if (g_status.BattLevel <= 2)
			{
				TRACE((LED_TRACE_LEVEL, "Led %d On patternPostion=%d", LedNum, patternPosition[LedNum]));
				GpioWrite(LEDGpio[LedNum], &LEDHandles[LedNum], TRUE);
			}
			break;

		case GSM_STATUS_MSK:
			// Light up if GSM status is good.
			if (Status_getGSMStatus() == GSM_WORKING)
			{
				TRACE((LED_TRACE_LEVEL, "Led %d On patternPostion=%d", LedNum, patternPosition[LedNum]));
				GpioWrite(LEDGpio[LedNum], &LEDHandles[LedNum], TRUE);
			}
			break;

		case GPS_STATUS_MSK:     // RED GPS LED.
			// Light up if GPS status is 2D or Sats visible.
			{
				bool AGPSStatus = get_agps_status();
				//int prev_status = gps_led_status;
				//gps_led_status = FIX_3D_LED;
				//AGPSStatus = 1;
				if ((gps_led_status == FIX_2D_LED) || (gps_led_status == SATS_VISIBLE_LED) || 
					((gps_led_status == FIX_3D_LED) && !AGPSStatus))
				{
					TRACE((LED_TRACE_LEVEL, "Led %d On patternPostion=%d", LedNum, patternPosition[LedNum]));
					GpioWrite(LEDGpio[LedNum], &LEDHandles[LedNum], TRUE);
				}
				else if( (gps_led_status == FIX_3D_LED) && AGPSStatus)
				{
					DumpMessage("3D fix and AGPS ON, hence turning off red led\r\n");
					GpioWrite(LEDGpio[LedNum], &LEDHandles[LedNum], FALSE);
				}
				//gps_led_status = prev_status;
			}
			break;

		case GPS_STATUS_MSK_GREEN:     // GREEN GPS LED.
			// Light green if its fix 3d or fix 2d
			if ((gps_led_status == FIX_3D_LED))
			{
				TRACE((LED_TRACE_LEVEL, "Led %d On patternPostion=%d", LedNum, patternPosition[LedNum]));
				GpioWrite(LEDGpio[LedNum], &LEDHandles[LedNum], TRUE);
			}
			break;

		case LED_OFF_MSK:
			// no light
			TRACE((LED_TRACE_LEVEL, "Led %d On patternPostion=%d", LedNum, patternPosition[LedNum]));
			GpioWrite(LEDGpio[LedNum], &LEDHandles[LedNum], FALSE);

		default:
			// No mask, light it up.
			TRACE((LED_TRACE_LEVEL, "Led %d On patternPostion=%d", LedNum, patternPosition[LedNum]));
			GpioWrite(LEDGpio[LedNum], &LEDHandles[LedNum], TRUE);
		}
	}

	TRACE((LED_TRACE_LEVEL, "EVAL: TimerCount %x", timerCount[LedNum]));
	TRACE((LED_TRACE_LEVEL, "EVAL: value masked %x", currentValueMsked));
	// Adjust timerCount
	if (timerCount[LedNum] == currentValueMsked)
	{
		timerCount[LedNum] = 0;
		patternPosition[LedNum]++;
	}
	else
	{
		timerCount[LedNum]++;
	}
}


void SetGPSLedStatus(GPS_LED_STATUS stat)
{
	gps_led_status = stat;
}


/*@}*/
