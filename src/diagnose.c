/** @addtogroup TestCode
 *@{*/
#include <adl_global.h>
#include "diagnose.h"
#include "LED.h"
#include "VibrationMotor.h"
#include "PowerCtrl.h"
#include "Accelerometer.h"
#include "XactUtilities.h"
#include "ConfigSettings.h"
#include "status.h"
#include "ota_at.h"

#define DIAGNOSE_TIME    60 * 5 /**< the number of seconds to stay in diagnose mode */

extern FAST_IDLE_REQ AccelFastIdleReq;

static INDICATOR diagnose_get_AB(void);
static INDICATOR diagnose_get_PB(void);
static INDICATOR diagnose_get_MS(void);
static INDICATOR diagnose_get_AC(void);
static INDICATOR diagnose_get_LED1(void);
static INDICATOR diagnose_get_LED2(void);
static INDICATOR diagnose_get_LED3(void);
static INDICATOR diagnose_get_LED4(void);
static INDICATOR diagnose_get_LED5(void);
static INDICATOR diagnose_get_LED6(void);
static INDICATOR diagnose_get_VM(void);

static DIAGNOSE_MODE diag_mode = DIAGNOSE_OFF;

static INDICATOR accessory_button = IND_OFF;
static INDICATOR power_button = IND_OFF;
static INDICATOR magnetic_sensor = IND_OFF;
static INDICATOR accelerometer = IND_OFF;
static INDICATOR leds[6] =
{
	IND_OFF, IND_OFF, IND_OFF,
	IND_OFF, IND_OFF, IND_OFF
};
static INDICATOR vibrate_motor = IND_OFF;

static int diagnose_proc_diagnose(char *sense_dev, char *ind_dev, char *acc_sens);
static void diagnose_check_set_prev_state(void);

void diagnose_timer(u8 timerID, void *Context);

static adl_rtcTimeStamp_t LastATTimeStamp;

/** @brief diagnose get
 *
 * @par
 * Return the current status of the indicator src
 * @return IND_OFF
 */
INDICATOR diagnose_get(SOURCE src)
{
	switch (src)
	{
	case AB:
		return diagnose_get_AB();

	case PB:
		return diagnose_get_PB();

	case MS:
		return diagnose_get_MS();

	case AC:
		return diagnose_get_AC();

	case LED1_SRC:
		return diagnose_get_LED1();

	case LED2_SRC:
		return diagnose_get_LED2();

	case LED3_SRC:
		return diagnose_get_LED3();

	case LED4_SRC:
		return diagnose_get_LED4();

	case LED5_SRC:
		return diagnose_get_LED5();

	case LED6_SRC:
		return diagnose_get_LED6();

	case VM_SRC:
		return diagnose_get_VM();

	default:
		return IND_OFF;
	}
}


/** @brief diagnose get AB
 *
 * @par
 * return the status of the accesory button.
 * @return accessory_button
 */
static INDICATOR diagnose_get_AB(void)
{
	return accessory_button;
}


/** @brief diagnose get PB
 *
 * @par
 * return the status of the power button.
 * @return power_button
 */
static INDICATOR diagnose_get_PB(void)
{
	return power_button;
}


/** @brief diagnose get MS
 *
 * @par
 * return the status of the magnetic sensor.
 * @return magnetic_sensor
 */
static INDICATOR diagnose_get_MS(void)
{
	return magnetic_sensor;
}


/** @brief diagnose get AC
 *
 * @par
 * return the status of the accelerometer.
 * @return accelerometer
 */
static INDICATOR diagnose_get_AC(void)
{
	return accelerometer;
}


/** @brief diagnose get LED1
 *
 * @par
 * return the status of LED1
 * @return LED1
 */
static INDICATOR diagnose_get_LED1(void)
{
	return leds[0];
}


/** @brief diagnose get LED2
 *
 * @par
 * return the status of LED2
 * @return LED2
 */
static INDICATOR diagnose_get_LED2(void)
{
	return leds[1];
}


/** @brief diagnose get LED3
 *
 * @par
 * return the status of LED3
 * @return LED3
 */
static INDICATOR diagnose_get_LED3(void)
{
	return leds[2];
}


/** @brief diagnose get LED4
 *
 * @par
 * return the status of LED4
 * @return LED4
 */
static INDICATOR diagnose_get_LED4(void)
{
	return leds[3];
}


/** @brief diagnose get LED5
 *
 * @par
 * return the status of LED5
 * @return LED5
 */
static INDICATOR diagnose_get_LED5(void)
{
	return leds[4];
}


/** @brief diagnose get LED6
 *
 * @par
 * return the status of LED6
 * @return LED6
 */
static INDICATOR diagnose_get_LED6(void)
{
	return leds[5];
}


/** @brief diagnose get vibrate motor.
 *
 * @par
 * return the status of vibrate motor.
 * @return vibrate motor.
 */
static INDICATOR diagnose_get_VM(void)
{
	return vibrate_motor;
}


/** @brief diagnose get mode
 *
 * @par
 * return if we are in diagnose mode.
 * @return diag_mode
 */
DIAGNOSE_MODE diagnose_get_mode(void)
{
	return diag_mode;
}


/** @brief diagnose dump usb
 *
 * @par
 * Send the status of the source out over the USB interface.
 * @param src
 * @param on_off
 * @return void
 */
static void diagnose_dump_usb(SOURCE src, int on_off)
{
	if (on_off == 0)
	{
		switch (src)
		{
		case AB:
			DUMP_USB("AB ON\r\n");
			break;

		case PB:
			DUMP_USB("PB ON\r\n");
			break;

		case MS:
			DUMP_USB("MS ON\r\n");
			break;

		case AC:
			DUMP_USB("AC ON\r\n");
			break;

		default:
			break;
		}
	}
	else
	{
		switch (src)
		{
		case AB:
			DUMP_USB("AB OFF\r\n");
			break;

		case PB:
			DUMP_USB("PB OFF\r\n");
			break;

		case MS:
			DUMP_USB("MS OFF\r\n");
			break;

		case AC:
			DUMP_USB("AC OFF\r\n");
			break;

		default:
			break;
		}
	}
}


/** @brief diagnose activate indicator
 *
 * @par
 * Turn on a particular indicator.
 * @param ind
 * @param src
 * @return void
 */
void diagnose_activate_indicator(INDICATOR ind, SOURCE src)
{
	switch (ind)
	{
	case LED1:
		TurnOnLEDNum(0);
		break;

	case LED2:
		TurnOnLEDNum(1);
		break;

	case LED3:
		TurnOnLEDNum(2);
		break;

	case LED4:
		TurnOnLEDNum(3);
		break;

	case LED5:
		TurnOnLEDNum(4);
		break;

	case LED6:
		TurnOnLEDNum(5);
		break;

	case VM:
		vibrate_turn_on();
		break;

	case IND_OFF:
		break;

	case SENSE:
		diagnose_dump_usb(src, 1);
		break;

	default:
		break;
	}
}


/** @brief diagnose deactivate indicator
 *
 * @par
 * Turn off a particular indicator.
 * @param ind
 * @param src
 * @return void
 */
void diagnose_deactivate_indicator(INDICATOR ind, SOURCE src)
{
	switch (ind)
	{
	case LED1:
		TurnOffLEDNum(0);
		break;

	case LED2:
		TurnOffLEDNum(1);
		break;

	case LED3:
		TurnOffLEDNum(2);
		break;

	case LED4:
		TurnOffLEDNum(3);
		break;

	case LED5:
		TurnOffLEDNum(4);
		break;

	case LED6:
		TurnOffLEDNum(5);
		break;

	case VM:
		vibrate_turn_off();
		break;

	case IND_OFF:
		break;

	case SENSE:
		diagnose_dump_usb(src, 0);
		break;

	default:
		break;
	}
}


/** @brief diagnose handler
 *
 * @par
 * Process the at&diagnose commands over the USB interface.
 * @param Cmd
 * @return void
 */
void diagnose_handler(adl_atCmdPreParser_t *Cmd)
{
	switch (Cmd->Type)
	{
	case ADL_CMD_TYPE_TEST:
		DUMP_USB("\r\nat&diagnose=? is invalid! \r\n");
		break;

	case ADL_CMD_TYPE_READ:
		break;

	case ADL_CMD_TYPE_ACT:
		DUMP_USB("\r\nAT&diagnose is invalid! \r\n");
		break;

	case ADL_CMD_TYPE_PARA:
		if (diagnose_proc_diagnose(ADL_GET_PARAM(Cmd, 0), ADL_GET_PARAM(Cmd, 1), ADL_GET_PARAM(Cmd, 2)) == 0)
		{
			diagnose_check_set_prev_state();
			DUMP_USB("\r\nOK\r\n");
		}
		else
		{
			DUMP_USB("\r\nDIAGNOSE ERROR\r\n");
		}
		break;
	}
}


// process an at command from the over the air interface.
// the command will come as at&diagnose=x,x,x
// just pass the x,x,x to this function
void diagnose_handler_ota(char *cmd_string)
{
#define NUM_STRINGS     (3)
#define MAX_LEN         (10)
	char params[NUM_STRINGS][MAX_LEN];
	int param_idx = 0;
	char    *begin_string = cmd_string;
	char TxStr[50];

	memset(params, 0x00, NUM_STRINGS * MAX_LEN);

	wm_sprintf(g_traceBuf, "OTA DIAGNOSE CMD HANDLER: %s\r\n", cmd_string);
	DumpMessage(g_traceBuf);

	if (cmd_string == NULL)
	{
		DumpMessage("Error: Cannot process NULL diagnose command string\r\n");
		return;
	}

	// grab individual parameters.
	do
	{
		if (*cmd_string == ',')
		{
			strncat(params[param_idx++], begin_string, (cmd_string - begin_string));
			begin_string = cmd_string + 1;
		}
	} while (*cmd_string++);

	strncat(params[param_idx], begin_string, (cmd_string - begin_string - 1));

	DumpMessage(params[0]);
	DumpMessage("\r\n");
	DumpMessage(params[1]);
	DumpMessage("\r\n");
	DumpMessage(params[2]);
	DumpMessage("\r\n");

	if (diagnose_proc_diagnose(params[0], params[1], params[2]) == -1)
	{
		wm_sprintf(TxStr,"OTA Diagnose fail\r\n");
		DumpMessage(TxStr);
		send_ota_response(TxStr);		
	}
	else
	{
		wm_sprintf(TxStr,"OTA Diagnose success\r\n");
		DumpMessage(TxStr);
		send_ota_response(TxStr);		
		diagnose_check_set_prev_state();
	}
}


/** @brief diagnose proc diagnose
 *
 * @par
 * Validate the diagnose command is correct.
 * @param sense_dev
 * @param ind_dev
 * @param acc_sens
 * @return 0
 */
static int diagnose_proc_diagnose(char *sense_dev, char *ind_dev, char *acc_sens)
{
	INDICATOR temp_ind;
	// check for reset

	if (strncmp("RESET", sense_dev, 5) == 0)
	{
		Status_setResetReason(RESET_REASON_DIAGNOSE_MODE);
		start_reset_timer();
		return 0;
	}

	// check for valid indicators.
	if      (strncmp("LED1", ind_dev, 4) == 0)
	{
		temp_ind = LED1;
	}
	else if (strncmp("LED2", ind_dev, 4) == 0)
	{
		temp_ind = LED2;
	}
	else if (strncmp("LED3", ind_dev, 4) == 0)
	{
		temp_ind = LED3;
	}
	else if (strncmp("LED4", ind_dev, 4) == 0)
	{
		temp_ind = LED4;
	}
	else if (strncmp("LED5", ind_dev, 4) == 0)
	{
		temp_ind = LED5;
	}
	else if (strncmp("LED6", ind_dev, 4) == 0)
	{
		temp_ind = LED6;
	}
	else if (strncmp("VM", ind_dev, 2) == 0)
	{
		temp_ind = VM;
	}
	else if (strncmp("OFF", ind_dev, 3) == 0)
	{
		temp_ind = IND_OFF;
	}
	else if (strncmp("ON", ind_dev, 2) == 0)
	{
		temp_ind = IND_ON;
	}
	else if (strncmp("SENSE", ind_dev, 3) == 0)
	{
		temp_ind = SENSE;
	}
	else
	{
		return -1;
	}

	// check for valid signal sources.
	if      (strncmp("AB", sense_dev, 2) == 0)
	{
		accessory_button = temp_ind;
	}
	else if (strncmp("PB", sense_dev, 2) == 0)
	{
		power_button = temp_ind;
	}
	else if (strncmp("MS", sense_dev, 2) == 0)
	{
		magnetic_sensor = temp_ind;
	}
	else if (strncmp("AC", sense_dev, 2) == 0)
	{
		accelerometer = temp_ind;
	}
	// these "signal sources" only accept IND_OFF or IND_ON indicators.
	else if ((strncmp("LED1", sense_dev, 4) == 0) && ((temp_ind == IND_OFF) || (temp_ind == IND_ON)))
	{
		leds[0] = temp_ind;
	}
	else if ((strncmp("LED2", sense_dev, 4) == 0) && ((temp_ind == IND_OFF) || (temp_ind == IND_ON)))
	{
		leds[1] = temp_ind;
	}
	else if ((strncmp("LED3", sense_dev, 4) == 0) && ((temp_ind == IND_OFF) || (temp_ind == IND_ON)))
	{
		leds[2] = temp_ind;
	}
	else if ((strncmp("LED4", sense_dev, 4) == 0) && ((temp_ind == IND_OFF) || (temp_ind == IND_ON)))
	{
		leds[3] = temp_ind;
	}
	else if ((strncmp("LED5", sense_dev, 4) == 0) && ((temp_ind == IND_OFF) || (temp_ind == IND_ON)))
	{
		leds[4] = temp_ind;
	}
	else if ((strncmp("LED6", sense_dev, 4) == 0) && ((temp_ind == IND_OFF) || (temp_ind == IND_ON)))
	{
		leds[5] = temp_ind;
	}
	else if ((strncmp("VM", sense_dev, 2) == 0) && ((temp_ind == IND_OFF) || (temp_ind == IND_ON)))
	{
		vibrate_motor = temp_ind;
	}
	else
	{
		return -1;
	}

	if ((strncmp("AC", sense_dev, 2) == 0) && (acc_sens != NULL))
	{
		SetAccelTriggerThreshold(hex_ascii_2_uint((UINT8 *)acc_sens, 8));
	}
	else if ((strncmp("AC", sense_dev, 2) == 0) && (temp_ind != IND_OFF) && (temp_ind != IND_ON))
	{
		return -1;
	}
	else if ((strncmp("AC", sense_dev, 2) == 0) && (temp_ind == IND_ON))
	{
		DUMP_USB("\r\n");
		SetAccelMeasureMode();
		AccelReadData();
	}

	return 0;
}


/** @brief diagnose check set prev state
 *
 * @par call this function whenever we recieve a valid diagnose command.
 * not needed for reboot command though.
 * @return void
 */
static void diagnose_check_set_prev_state(void)
{
	s32 sReturn;
	adl_rtcTime_t CurrentTime;

	if (diagnose_get_mode() == DIAGNOSE_OFF)
	{
		ShutOffLED();
		vibrate_turn_off();

		if (adl_tmrSubscribe(TRUE, 10, ADL_TMR_TYPE_100MS, (adl_tmrHandler_t)diagnose_timer) == NULL)
		{
			DumpMessage("Could not start diagnose timer\r\n");
		}
	}
	AccelFastIdleReq = FAST_IDLE_REQ_ACTIVE;

	// get the current RTC time.
	if ((sReturn = adl_rtcGetTime(&CurrentTime)) < 0)
	{
		DisplayErrorCode("adl_rtcGetTime", __FILE__, __LINE__, sReturn);
	}

	// Get first RTC time.
	if (adl_rtcConvertTime(&CurrentTime, &LastATTimeStamp, ADL_RTC_CONVERT_TO_TIMESTAMP) < 0)
	{
		DisplayErrorCode("adl_rtcConvertTime", __FILE__, __LINE__, sReturn);
	}

	diag_mode = DIAGNOSE_ON;
}


/** @brief diagnose timer
 *
 * @par
 * Only stay in diagnose mode for 5 minutes after the last diagnose command is received.
 * @param timerID
 * @param Context
 * @return void
 */
void diagnose_timer(u8 timerID, void *Context)
{
	(void)timerID;
	(void)Context;

	adl_rtcTimeStamp_t CurrentTimeStamp;
	adl_rtcTime_t CurrentTime;
	adl_rtcTimeStamp_t DeltaTimeStamp;
	s32 sReturn;

	if (diagnose_get_mode() == DIAGNOSE_ON)
	{
		if ((sReturn = adl_rtcGetTime(&CurrentTime)) < 0)
		{
			DisplayErrorCode("adl_rtcGetTime", __FILE__, __LINE__, sReturn);
		}

		if ((sReturn = adl_rtcConvertTime(&CurrentTime, &CurrentTimeStamp, ADL_RTC_CONVERT_TO_TIMESTAMP)) < 0)
		{
			DisplayErrorCode("adl_rtcConvertTime", __FILE__, __LINE__, sReturn);
		}

		// Calculate the time difference.
		if ((sReturn = adl_rtcDiffTime(&CurrentTimeStamp, &LastATTimeStamp, &DeltaTimeStamp)) < 0)
		{
			DisplayErrorCode("adl_rtcDiffTime", __FILE__, __LINE__, sReturn);
		}

		if (DeltaTimeStamp.TimeStamp >= DIAGNOSE_TIME)
		{
			DumpMessage("Diagnose period over, going to reset\r\n");
			DUMP_USB("Diagnose period over, going to reset\r\n");
			Status_setResetReason(RESET_REASON_DIAGNOSE_MODE);
			start_reset_timer();
		}
	}
}


/*@}*/
