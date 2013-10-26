/** @addtogroup GPIO
 *@{*/

/*H************************************************************************
 */

/*! \file    gpioTest.c
 *
 *   \brief   Reads the GPIO
 *
 *   \details Reads the GPIO such as push button and SOS button and sets system states.
 *
 *   \note    Other help for the reader, including references.
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

/* -------- */
/* Includes */
/* -------- */
#include "adl_global.h"
#include "gpioTest.h"
#include "XactUtilities.h"
#include "status.h"
#include "ConfigSettings.h"
#include "diagnose.h"
#include "airplane_mode.h"
#include "GPSCtrl.h"
#include "Traces.h"
#include "button.h"
#include "usb.h"
#include "adc.h"

DEBUG_TRACE_STORAGE;

/* --------------- */
/* Global Variables*/
/* --------------- */
#define OUTPUT_GPIO_LABEL       (ADL_IO_GPIO | 0)
#define INPUT_GPIO_LABEL        (ADL_IO_GPIO | 0)

#define SOS_BUTTON_GPIO         (20)    /*!< The GPIO number for the SOS button */
#define MAG_SOS_GPIO            (22)    /*!< The GPIO number for the output of the magnetic sensor */
#define MAG_SENSE_EN            (19)    /*!< The GPIO for enabling the magnetic sensor */
#define ON_BATT_GPIO            (21)    /*!< The GPIO for the on-bat signal */
#define USB_DET_GPIO            (33)    /*!< The GPIO for reading the USB det signal */



static s32 MagSOSHandle = ERROR;        //Handle for GPIO control
BOOL IgnAlarmGenerated = FALSE;
extern s32 GPIO_5_Handle;			


#if !defined(DIAGNOSE_MODE_DISABLED) ||  !defined(MAGNETIC_SOS_DISABLED)
static s32 MagEnHandle = ERROR;
#endif

/* --------------- */
/* Local Functions */
/* --------------- */

static s32 USBConnectedSense = 0;
static s32 OnBattSense = 1;

#define NUM_SECONDS_CREG_WD    (10 * 60)
//#define NUM_SECONDS_CREG_WD (10)


#if !defined(EVAL_GSM_WATCHDOG_DISABLED)

static void eval_gsm_watchdog(GSM_STATUS gsm_stat)
{
	static adl_rtcTimeStamp_t no_gsm_start_time;
	adl_rtcTimeStamp_t current_time_stamp, delta_time_stamp;

	typedef enum
	{
		RECORD_NOT_WORKING_TIME = 0,
		DIFF_NOT_WORKING_TIME = 1,
		GSM_WORKING_DO_NOTHING = 2,
		WAIT_FOR_RESET = 3
	} GSM_WATCH_STATE;

	static GSM_WATCH_STATE state = RECORD_NOT_WORKING_TIME;
	static GSM_WATCH_STATE prev_state = RECORD_NOT_WORKING_TIME;

	switch (state)
	{
	// wait until we get a good timestamp for begin of gsm loss
	case RECORD_NOT_WORKING_TIME:
		if (gsm_stat == GSM_WORKING)
		{
			state = GSM_WORKING_DO_NOTHING;
		}
		else if (GetConvertTime(&no_gsm_start_time) >= 0)			
		{
			wm_sprintf(g_traceBuf,"no_gsm_start_time : 0x%x\r\n", (unsigned int)no_gsm_start_time.TimeStamp);
			DumpMessage(g_traceBuf);
			state = DIFF_NOT_WORKING_TIME;
		}
		break;

	// wait until gsm has been lost for >= NUM_SECONDS_CREG_WD
	case DIFF_NOT_WORKING_TIME:
		if (gsm_stat == GSM_WORKING)
		{
			state = GSM_WORKING_DO_NOTHING;
			break;
		}
		if (GetConvertTime(&current_time_stamp) < 0)
		{
			break;
		}
		if (adl_rtcDiffTime(&current_time_stamp, &no_gsm_start_time, &delta_time_stamp) < 0)
		{
			break;
		}
		if (delta_time_stamp.TimeStamp >= NUM_SECONDS_CREG_WD)
		{
			wm_sprintf(g_traceBuf,"no_gsm_stop_time : 0x%x, Delta:%ld\r\n", (unsigned int)current_time_stamp.TimeStamp,delta_time_stamp.TimeStamp);
			DumpMessage(g_traceBuf);
			Status_setResetReason(RESET_REASON_GSM_WATCH);
			start_reset_timer();
			state = WAIT_FOR_RESET;
		}
		break;

	// wait for GSM loss.
	case GSM_WORKING_DO_NOTHING:
		if (gsm_stat == GSM_NOT_WORKING)
		{
			state = RECORD_NOT_WORKING_TIME;
		}
		break;

	case WAIT_FOR_RESET:
	default:
		break;
	}

	// print out some debug information.
	if (prev_state != state)
	{
		wm_sprintf(g_traceBuf, "eval_gsm_watchdog : %d %d\r\n", prev_state, state);
		DumpMessage(g_traceBuf);
	}
	prev_state = state;
}
#else
#warn EVAL_GSM DISABLED
static void eval_gsm_watchdog(GSM_STATUS gsm_stat)
{
    (void) gsm_stat;
    return;
}
#endif


/** @brief Reads from the GPIO that is passed into the function
 *
 *   @note   If the handle is invalid, the method will try to subscribe \
 *          to the gpio service
 *
 * @param gpioNumber Number of the GPIO to read
 * @param gpioHandle The handle of the specified GPIO
 * @return sReturn 1 High, 0 Low, <0 error
 */
s32 GpioRead(int gpioNumber, s32 *gpioHandle)
{
	s32 sReturn = -1;
	adl_ioDefs_t GpioConfig;
	adl_ioDefs_t Gpio_to_read = ADL_IO_GPIO;

	GpioConfig = gpioNumber | INPUT_GPIO_LABEL | ADL_IO_DIR_IN;

	if (*gpioHandle < 0)
	{
		*gpioHandle = adl_ioSubscribe(1, &GpioConfig, 0
		                              , 0, 0);
		if (*gpioHandle < 0)
		{
			wm_sprintf(g_traceBuf, "GPIO ERROR: GPIO Number = %d\r\n", gpioNumber);
			DumpMessage(g_traceBuf);
			DisplayErrorCode("adl_ioSubscribe"
			                 , __FILE__
			                 , __LINE__
			                 , *gpioHandle);
			sReturn = *gpioHandle;
			if (*gpioHandle == ADL_RET_ERR_DONE)
			{
				wm_sprintf(g_traceBuf, "inside ADL_RET_ERR_DONE: %x\r\n", (unsigned int)GpioConfig);
				DumpMessage(g_traceBuf);
				switch ((GpioConfig & ADL_IO_ERR_MSK))
				{
				case ADL_IO_ERR:
					wm_sprintf(g_traceBuf, "Gpio %d had and unidentified error\r\n", gpioNumber);
					DumpMessage(g_traceBuf);
					break;

				case ADL_IO_ERR_UNKWN:
					wm_sprintf(g_traceBuf, "Gpio %d is and unknown GPIO\r\n", gpioNumber);
					DumpMessage(g_traceBuf);
					break;

				case ADL_IO_ERR_USED:
					wm_sprintf(g_traceBuf, "Gpio %d is already being used\r\n", gpioNumber);
					DumpMessage(g_traceBuf);
					break;

				case ADL_IO_ERR_BADDIR:
					wm_sprintf(g_traceBuf, "Gpio %d is a bad direction\r\n", gpioNumber);
					DumpMessage(g_traceBuf);
					break;

				case ADL_IO_ERR_NIH:
					wm_sprintf(g_traceBuf, "Gpio %d is not in the handle\r\n", gpioNumber);
					DumpMessage(g_traceBuf);
					break;

				default:
					wm_sprintf(g_traceBuf, "ADL_IO_ERR_MSK produced an unknown value\r\n");
					DumpMessage(g_traceBuf);
					break;
				}
			}
		}
	}

	Gpio_to_read = ADL_IO_GPIO | gpioNumber;
	sReturn = adl_ioReadSingle(*gpioHandle, &Gpio_to_read);
	if (sReturn < 0)
	{
		DisplayErrorCode("adl_ioReadSingle", __FILE__, __LINE__, sReturn);
	}

	return sReturn;
}


/** @brief Write to GPIO
 *
 *  @par Writes to the GPIO that is passed into the function
 *
 *  @param gpioNumber
 *  @param gpioHandle
 *  @param level
 *  @return void
 */
void GpioWrite(int gpioNumber, s32 *gpioHandle, bool level)
{
	s32 sReturn = -1;
	adl_ioDefs_t GpioConfig;
	adl_ioDefs_t Gpio_to_write = ADL_IO_GPIO;
	adl_ioDefs_t Gpio_level;
	int Gpio_number = gpioNumber;
	s32 Gpio_handle = *gpioHandle;

	//Check to see which level to write to GPIO
	if (level == FALSE)
	{
		Gpio_level = ADL_IO_LEV_LOW;
	}
	else
	{
		Gpio_level = ADL_IO_LEV_HIGH;
	}

	GpioConfig = Gpio_number | OUTPUT_GPIO_LABEL
	             | ADL_IO_DIR_OUT | Gpio_level;

	//check to see if handle is valid
	if (Gpio_handle < 0)
	{
		Gpio_handle = adl_ioSubscribe(1, &GpioConfig, 0, 0, 0);
		if (Gpio_handle < 0)
		{
			DisplayErrorCode("adl_ioSubscribe"
			                 , __FILE__
			                 , __LINE__
			                 , Gpio_handle);
			sReturn = Gpio_handle;
			wm_sprintf(g_traceBuf, "GPIO number %d\r\n", gpioNumber);
			DumpMessage(g_traceBuf);
			wm_sprintf(g_traceBuf, "Error mask: %x\r\n", (unsigned int )(GpioConfig & ADL_IO_ERR_MSK));
			DumpMessage(g_traceBuf);
		}
		else
		{
			wm_sprintf(g_traceBuf, "GPIO %d is subscribed\r\n", Gpio_number);
			DumpMessage(g_traceBuf);
		}
	}
	else
	{
		Gpio_to_write = ADL_IO_GPIO | Gpio_number;

		//Write value to GPIO
		if (level == FALSE)
		{
			sReturn = adl_ioWriteSingle(Gpio_handle, &Gpio_to_write, FALSE);
			if (sReturn < 0)
			{
				DisplayErrorCode("adl_ioWriteSingle", __FILE__, __LINE__, sReturn);
			}
		}
		else
		{
			sReturn = adl_ioWriteSingle(Gpio_handle, &Gpio_to_write, TRUE);
			if (sReturn < 0)
			{
				DisplayErrorCode("adl_ioWriteSingle", __FILE__, __LINE__, sReturn);
			}
		}
	}

	//Update the handle
	*gpioHandle = Gpio_handle;
}

static void DetermineOnBatt(void)
{
    static s32 OnBattHandle = ERROR;
    OnBattSense = GpioRead(ON_BATT_GPIO, &OnBattHandle);
}


void ResetSPIFlash(void)
{
    static s32 SpiFlashResetHandle = ERROR;
    int const SPI_FLASH_RESET_PIN = 32;
	DumpMessage("\r\nReset SPI flash\r\n");
	GpioWrite(SPI_FLASH_RESET_PIN, &SpiFlashResetHandle, TRUE);
	GpioWrite(SPI_FLASH_RESET_PIN, &SpiFlashResetHandle, FALSE);
	GpioWrite(SPI_FLASH_RESET_PIN, &SpiFlashResetHandle, TRUE);
}

#if !defined(DIAGNOSE_MODE_DISABLED) || !defined(SOS_BUTTON_DISABLED)
static s32 SOSHandle = ERROR;           //Handle for GPIO control
#endif

#if !defined(DIAGNOSE_MODE_DISABLED)
static bool CheckDiagnoseMode(void)
{
    bool result = false;
    SOURCE const LEDDiagnoseSource[6] =
    {
        LED1_SRC, LED2_SRC, LED3_SRC, LED4_SRC, LED5_SRC, LED6_SRC
    };

    SOURCE const LEDDiagnoseIndicator[6] =
    {
        LED1, LED2, LED3, LED4, LED5, LED6
    };

	// check if we are in the diagnose mode. If so process
	// any associates handled by this function. Normal operation
	// will not work in this mode.
	if (diagnose_get_mode() == DIAGNOSE_ON)
	{
        int ii;
        INDICATOR tmp_ind;

		for (ii = 0; ii < 6; ii++)
		{
			if (diagnose_get(LEDDiagnoseSource[ii]) != IND_OFF)
			{
				diagnose_activate_indicator(LEDDiagnoseIndicator[ii], LEDDiagnoseSource[ii]);
			}
			else
			{
				diagnose_deactivate_indicator(LEDDiagnoseIndicator[ii], LEDDiagnoseSource[ii]);
			}
		}

		if (diagnose_get(VM_SRC) != IND_OFF)
		{
			diagnose_activate_indicator(VM, VM);
		}
		else
		{
			diagnose_deactivate_indicator(VM, VM);
		}

		tmp_ind = diagnose_get(AB);
		if (tmp_ind != IND_OFF)
		{
			if (!GpioRead(SOS_BUTTON_GPIO, &SOSHandle))
			{
				diagnose_activate_indicator(tmp_ind, AB);
			}
			else
			{
				diagnose_deactivate_indicator(tmp_ind, AB);
			}
		}

		tmp_ind = diagnose_get(PB);
		if (tmp_ind != IND_OFF)
		{
			if (InputButtonActive())
			{
				diagnose_activate_indicator(tmp_ind, PB);
			}
			else
			{
				diagnose_deactivate_indicator(tmp_ind, PB);
			}
		}

		tmp_ind = diagnose_get(MS);
		if (tmp_ind != IND_OFF)
		{
			GpioWrite(MAG_SENSE_EN, &MagEnHandle, TRUE);
			if (!GpioRead(MAG_SOS_GPIO, &MagSOSHandle))
			{
				diagnose_activate_indicator(tmp_ind, MS);
			}
			else
			{
				diagnose_deactivate_indicator(tmp_ind, MS);
			}
		}

		result = true;
	}

    return result;
}
#else
#warn DIAGNOSE MODE DISABLED
static void CheckDiagnoseMode(void)
{
    return;
}
#endif

#if defined(AIRPLANE_MODE_ENABLED)
static void ProcessAirplaneMode(void)
{
	// AB : todo fix this.
	if (!IsOnBatt())
	{
		AirplaneModeEnter();
	}
    
}
#else
static void ProcessAirplaneMode(void)
{
    return;
}
#endif

#if !defined(USB_CONNECTED_SENSE_DISABLED)

static void ProcessUSBConnectivity(void)
{
    static int USBWasConnected = 0;
    static int PowerDiscAlarmGenerated = 0;
    static s32 USBSenseHandle = ERROR;      

    USBConnectedSense = GpioRead(USB_DET_GPIO, &USBSenseHandle);
    if (USBConnectedSense)
    {
        USBWasConnected = 1;
        PowerDiscAlarmGenerated = 0;
    }
    else if (USBWasConnected)
    {
        // the USB cable was disconnected.
        if ((g_config.PowerDiscAlert == ENABLE) && !PowerDiscAlarmGenerated)
        {
            g_status.PowerDisconnAlarm = 'Y';
            PowerDiscAlarmGenerated = 1;
        }
    }
}
#else
#warn USB CONNECTED SENSE DISABLED
static void ProcessUSBConnectivity(void)
{
    return;
}
#endif

#if !defined(MAGNETIC_SOS_DISABLED)
static void ProcessMagneticSensorSOS(void)
{
    // magnetic sensor SOS control
    s32 MagSOSVal;
    static int MagneticAlarmGenerated = 0;
    static int InHolster = 0;

	// Turn on the magnetic sensor if it's enabled.
    GpioWrite(MAG_SENSE_EN, &MagEnHandle, (g_config.MagneticSnsEn == ENABLE));

    MagSOSVal = GpioRead(MAG_SOS_GPIO, &MagSOSHandle);

    if (!IsInSafeFence() && MagSOSVal && (g_config.MagneticSnsEn == ENABLE) && !MagneticAlarmGenerated && InHolster)
    {
        DumpMessage("Magnetic sensor alarm!\r\n");
        MagneticAlarmGenerated = 1;
        g_status.SOSAlarm = 'M';
        InHolster = 0;
    }
    else if (!MagSOSVal)
    {
        MagneticAlarmGenerated = 0;
    }

    // don't use the magnetic sensor value right away.
    // it seems like it takes a bit to turn on.

    {
        static int CallCnt = 0;
        if (CallCnt == 100)
        {
            if (!MagSOSVal)
            {
                InHolster = 1;
            }
        }
        else
        {
            CallCnt++;
        }
    }
}
#else
#warn MAGNETIC SOS DISABLED
static void ProcessMagneticSensorSOS(void)
{
    return;
}
#endif

#if !defined(SOS_BUTTON_DISABLED)
static void ProcessSOSButton(void)
{
    static int SOSPressed = 0;
    static adl_rtcTimeStamp_t SOSPressTimeStamp;
    static int SOSAlarmGenerated = 0;
    s32 sReturn = -1;

    if (!GpioRead(SOS_BUTTON_GPIO, &SOSHandle))
    {
        adl_rtcTime_t CurrentTime;
        adl_rtcTimeStamp_t CurrentTimeStamp;
        adl_rtcTimeStamp_t DeltaTimeStamp;

        sReturn = adl_rtcGetTime(&CurrentTime);
        ASSERT(sReturn == OK);

        sReturn = adl_rtcConvertTime(&CurrentTime, &CurrentTimeStamp, ADL_RTC_CONVERT_TO_TIMESTAMP);
        ASSERT(sReturn == OK);
        // If not pressed, get the current time and save it to the SOS press time stamp.
        if (!SOSPressed)
        {
            SOSPressTimeStamp = CurrentTimeStamp;
        }
        // if we have already been pressed, count how long.
        else
        {
            sReturn = adl_rtcDiffTime(&CurrentTimeStamp, &SOSPressTimeStamp, &DeltaTimeStamp);
            ASSERT(sReturn >= 0);

            // Generate the SOS Alarm.
            if (DeltaTimeStamp.TimeStamp >= SOS_THRESH)
            {
				//Sandeep - Added for testing. To be removed
				//static bool AlarmModeEntered = FALSE;
				//if(AlarmModeEntered == FALSE)
				//{
					//EnterAlarmMode();
					//AlarmModeEntered = TRUE;
				//}
				//End - Test				
				if ((g_config.SOSAlert != DISABLE_SOS) && !SOSAlarmGenerated)
                {
                    g_status.SOSAlarm = 'Y';
                    SOSAlarmGenerated = 1;
                }
            }
        }
        SOSPressed = 1;
    }
    else
    {
        SOSPressed = 0;
        SOSAlarmGenerated = 0;
    }
}
#else
#warn SOS BUTTON DISABLED
#endif

/** @brief event handler for GPIO
 *
 * @par The event handler for the GPIOs in GPIO read
 *
 * @param timerid
 * @param context
 * @return void
 */
void GPIOTimer(u8 timerid, void *context)
{
	(void) timerid;
	(void) context;
	// Check event

	TRACE((GPIO_TRACE_LEVEL, "\r\n enter gpio event handler \r\n"));

    if (CheckDiagnoseMode())
        return;

	eval_gsm_watchdog(Status_getGSMStatus());

	DetermineOnBatt();
    ProcessAirplaneMode();
    ProcessUSBConnectivity();
    ProcessMagneticSensorSOS();
    ProcessSOSButton();
    ProcessPowerButton();
	ProcessIginitionButton();
}


/** @brief is the USB connected
 *
 * @par Report whether or not the USB cable is connceted and charging the device.
 *
 * @return USBConnectedSense
 */
int IsUSBConnected(void)
{
	return USBConnectedSense;
}




int IsOnBatt(void)
{
	return OnBattSense;
}


/** @brief read Mag Sens
 *
 * @par
 * Determine if the magnetic sensor is currently being activated.
 * @return MagSOSVal
 */
int readMagSens(void)
{
	int MagSOSVal = GpioRead(MAG_SOS_GPIO, &MagSOSHandle);
	return MagSOSVal;
}

void InitializeUnusedGpio(void)
{
	struct
	{
		int gpio_number;
		s32 handle;
	}
	unused_pin[] =
	{
		{ 3, -1 },
		//{ 35, -1 },
		{ 45, -1 },
		//{ 46, -1 },
	};

	unsigned int i;
	size_t number_of_ios = sizeof unused_pin / sizeof unused_pin[0];

	for (i = 0; i < number_of_ios; i++)
	{
		GpioWrite(unused_pin[i].gpio_number, &unused_pin[i].handle, false);
	}
}


void TurnOffFF(void)
{
	int const FLIP_FLOP_CLEAR_GPIO = 24;
	s32 flipflop_handle = -1;

	DumpMessage("-------- Turning off FF\r\n");
	GpioWrite(FLIP_FLOP_CLEAR_GPIO, &flipflop_handle, false);
}

s32 ReadMagSensorGPIO()
{
	return (GpioRead(MAG_SOS_GPIO, &MagSOSHandle));
}


s32 ReadSOSGPIO()
{
	return (GpioRead(SOS_BUTTON_GPIO, &SOSHandle));
}


void ProcessIginitionButton(void)
{
	static int IgnPressed = 0;
	static adl_rtcTimeStamp_t IgnPressTimeStamp;
	s32 sReturn = -1;

	if (!GpioRead(GPIO_IGN_GPIO, &GPIO_5_Handle))
	{
		adl_rtcTime_t CurrentTime;
		adl_rtcTimeStamp_t CurrentTimeStamp;
		adl_rtcTimeStamp_t DeltaTimeStamp;

		sReturn = adl_rtcGetTime(&CurrentTime);
		ASSERT(sReturn == OK);

		sReturn = adl_rtcConvertTime(&CurrentTime, &CurrentTimeStamp, ADL_RTC_CONVERT_TO_TIMESTAMP);
		ASSERT(sReturn == OK);
		// If not pressed, get the current time and save it to the SOS press time stamp.
		if (!IgnPressed)
		{
			IgnPressTimeStamp = CurrentTimeStamp;
		}
		// if we have already been pressed, count how long.
		else
		{
			sReturn = adl_rtcDiffTime(&CurrentTimeStamp, &IgnPressTimeStamp, &DeltaTimeStamp);
			ASSERT(sReturn >= 0);

			// Generate the SOS Alarm.
			if (DeltaTimeStamp.TimeStamp >= IGN_THRESH)
			{
				if (!IgnAlarmGenerated)
				{
					IgnAlarmGenerated = TRUE;
					//Start Analog sampling
					StartAnalogSensorReadTimer();
				}
			}
		}
		IgnPressed = 1;
	}
	else
	{
		IgnPressed = 0;
		IgnAlarmGenerated = FALSE;
	}
}
/*@}*/
