/** @addtogroup Accel
 *@{*/

/**
 * @file VibrationMotor.c
 */

#include "VibrationMotor.h"
#include "gpioTest.h"
#include "ConfigSettings.h"
#include "status.h"
#include "diagnose.h"
#include "XactUtilities.h"
#include "Accelerometer.h"
#include "PowerCtrl.h"

FAST_IDLE_REQ VibrateFastIdleReq = FAST_IDLE_REQ_RELEASE;

//{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}

static const unsigned int VibratePatterns[MAX_PATTERNS][MAX_ON_OFF] =
{
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
	{ MD_PLS, FIVE_S, MD_PLS, FIVE_S, MD_PLS, FIVE_S, MD_PLS, FIVE_S, MD_PLS, FIVE_S, MD_PLS, FIVE_S,
	  MD_PLS, THREE_S, MD_PLS, THREE_S, MD_PLS, THREE_S, MD_PLS, THREE_S, MD_PLS, THREE_S, MD_PLS, THREE_S, MD_PLS, THREE_S,
	  MD_PLS, ONE5_S, MD_PLS, ONE5_S, MD_PLS, ONE5_S, MD_PLS, ONE5_S, MD_PLS, ONE5_S, MD_PLS, ONE5_S, MD_PLS, ONE5_S },
	{ LG_PLS, THREE_S, LG_PLS, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
	{ LG_PLS, THREE_S, LG_PLS, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
	{ LG_PLS, THREE_S, LG_PLS, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
	{ LG_PLS, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

// GPIO for motor.
static const int MOTOR_GPIO = 13;               /*!< Vibration motor GPIO */
s32 MotorGPIOHandle = ERROR;                    /*!< Motor GPIO Handle   */

int VibrateDone = 1;

/** @brief Vibration Timer handler
 *
 * @par
 * Controls the vibration motor. Based on the current system status, this timer
 * will execute one of the alarm patterns.
 * @param timerid
 * @param context
 *
 * @return void
 */
void VibrateTimer(u8 timerid, void *context)
{
	(void)timerid;
	(void)context;

	static int patternPosition = 0;
	static unsigned int timerCount = 0;
	static int SetAccelPrev = 1;
	// check for diagnose mode. if we are in diagnose mode
	// then normal operation is not performed.
	if (diagnose_get_mode() == DIAGNOSE_ON)
	{
		return;
	}

	// check if we are in "invisible" mode
	if (g_config.InvisibleOp == ENABLE)
	{
		GpioWrite(MOTOR_GPIO, &MotorGPIOHandle, FALSE);
		return;
	}

	if (g_status.SOSAlarm == 'Y')
	{
		g_config.VibrationMotorPattern = VIBRATE_SOS_PATTERN;
	}
	else if (g_config.SOSAlert == 'A')
	{
		g_config.VibrationMotorPattern = VIBRATE_SOS_ACK;
		g_config.SOSAlert = 'Y';
	}
	else if (g_config.Mode == SERVER_STATUS_REQ_V)
	{
		g_config.VibrationMotorPattern = VIBRATE_STATUS_ACK;
		DumpMessage("Server request vibrate!\r\n");
	}

	// We are done vibrating when the current vibration pattern duration is 0.
	if ((VibratePatterns[g_config.VibrationMotorPattern][patternPosition] == 0) || (patternPosition == MAX_ON_OFF))
	{
		TRACE((VIBRATION_TRACE_LEVEL, "Vibration Done"));
		timerCount = 0;
		patternPosition = 0;
		g_config.VibrationMotorPattern = 0;
		VibrateDone = 1;
		GpioWrite(MOTOR_GPIO, &MotorGPIOHandle, FALSE);
		if (SetAccelPrev == 0)
		{
			SetAccelTriggerThreshold(FAST_IDLE_ACCEL_LEVEL);
			VibrateFastIdleReq = FAST_IDLE_REQ_RELEASE;         // prevent from going to sleep while vibrating.
			SetAccelPrev = 1;
		}
		return;
	}

	if (patternPosition & 0x1)
	{
		// Odd pattern positions have motor off.
		TRACE((VIBRATION_TRACE_LEVEL, "Vibration Motor Off patternPostion=%d", patternPosition));
		GpioWrite(MOTOR_GPIO, &MotorGPIOHandle, FALSE);
	}
	else
	{
		// even positions have motor on.
		TRACE((VIBRATION_TRACE_LEVEL, "Vibration Motor On"));
		VibrateFastIdleReq = FAST_IDLE_REQ_ACTIVE;     // prevent from going to sleep while vibrating.
		SetAccelTriggerThreshold(0xff);
		GpioWrite(MOTOR_GPIO, &MotorGPIOHandle, TRUE);
		SetAccelPrev = 0;
	}

	if (timerCount == VibratePatterns[g_config.VibrationMotorPattern][patternPosition])
	{
		timerCount = 0;
		patternPosition++;
	}
	else
	{
		timerCount++;
	}
}


/** @brief enable vibration sensor
 *
 * @par
 * Writes the GPIO that is connected to the vibration motor.
 * @return void
 */
void vibrate_turn_on(void)
{
	GpioWrite(MOTOR_GPIO, &MotorGPIOHandle, TRUE);
}


/** @brief disable vibration sensor
 *
 * @par
 * Writes the GPIO that is connected to the vibration motor.
 * @return void
 */
void vibrate_turn_off(void)
{
	GpioWrite(MOTOR_GPIO, &MotorGPIOHandle, FALSE);
}


/*@}*/
