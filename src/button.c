#include "button.h"

#include <adl_global.h>

#include "XactUtilities.h"
#include "gpioTest.h"
#include "ConfigSettings.h"
#include "Traces.h"
#include "dogwalk.h"

DEBUG_TRACE_STORAGE;

int const g_inputButton = 11;           //GPIO used to read in switch SW6
s32 g_InputButtonHandle = ERROR;        //Handle for GPIO control

typedef struct
{
	bool sw_button_ovrd_en;
	int sw_button_ovrd_value;
} SW_BUTTON_OVRD_t;
SW_BUTTON_OVRD_t sw_button_ovrd = { false, 0 };

bool GetSWButtonOvrd(int *val)
{
	*val = sw_button_ovrd.sw_button_ovrd_value;
	return sw_button_ovrd.sw_button_ovrd_en;
}


void ClrSWButtonOvrd(void)
{
	DumpMessage("SW Button Release\r\n");
	sw_button_ovrd.sw_button_ovrd_value = 0;
}


void SetSWButtonOvrd(void)
{
	DumpMessage("SW Button Press\r\n");
	sw_button_ovrd.sw_button_ovrd_en = true;
	sw_button_ovrd.sw_button_ovrd_value = 1;
}

void ProcessPowerButton(void)
{
    int prevButtonValue = 0;
    int ButtonValue = 0;

    // read the power button/mode button.
    if (!GetSWButtonOvrd(&ButtonValue))
    {
        ButtonValue = GpioRead(g_inputButton, &g_InputButtonHandle);
    }

    // First check if the power button is disabled. If its disabled don't do anything
    //FIXME: This should be encapsulated
    if (g_config.PowerDownDisable == ENABLE)
        goto error;

    // check to see if we get an event change on the power button / state change button.
    if ((ButtonValue && !prevButtonValue) || (!ButtonValue && prevButtonValue))
    {
        adl_rtcTime_t CurrentTime;
        static adl_rtcTimeStamp_t ButtonPressTimeStamp;
        s32 sReturn = -1;

        sReturn = adl_rtcGetTime(&CurrentTime);
        ASSERT_GOTO(sReturn == OK, error);


        // Button is pressed.
        if (ButtonValue)
        {
            //Get first RTC time
            sReturn = adl_rtcConvertTime(&CurrentTime, &ButtonPressTimeStamp, ADL_RTC_CONVERT_TO_TIMESTAMP);
            ASSERT_GOTO(sReturn == OK, error);
        }
        else         
        {         
            // Button is released
            adl_rtcTimeStamp_t ButtonReleaseTimeStamp;
            adl_rtcTimeStamp_t DeltaTimeStamp;
            sReturn = adl_rtcConvertTime(&CurrentTime, &ButtonReleaseTimeStamp, ADL_RTC_CONVERT_TO_TIMESTAMP);
            ASSERT_GOTO(sReturn == OK, error);

            TRACE((GPIO_TRACE_LEVEL, "Button Press Time Stamp: %x", ButtonPressTimeStamp.TimeStamp));
            TRACE((GPIO_TRACE_LEVEL, "Button Released Time Stamp: %x", ButtonReleaseTimeStamp.TimeStamp));

            TRACE((GPIO_TRACE_LEVEL, "Button Press Time Stamp frac: %x", ButtonPressTimeStamp.SecondFracPart));
            TRACE((GPIO_TRACE_LEVEL, "Button Released Time Stamp frac: %x", ButtonReleaseTimeStamp.SecondFracPart));

            sReturn = adl_rtcDiffTime(&ButtonReleaseTimeStamp, &ButtonPressTimeStamp, &DeltaTimeStamp);
            ASSERT_GOTO(sReturn >= 0, error);

            {
                int time = (int)ADL_RTC_GET_TIMESTAMP_SECONDS(DeltaTimeStamp);
                int time_ms = (int)ADL_RTC_GET_TIMESTAMP_MS(DeltaTimeStamp);

                TRACE((GPIO_TRACE_LEVEL, "Delta Time Stamp: %x", DeltaTimeStamp.TimeStamp));
                TRACE((GPIO_TRACE_LEVEL, "Button was pressed for (s): %d", time));
                wm_sprintf(g_traceBuf, "Button was pressed for (s): %d\r\n", time);
                DumpMessage(g_traceBuf);
                TRACE((GPIO_TRACE_LEVEL, "Button was pressed for (ms): %d", time_ms));

                // Adjust the system state.
                if (!check_shutdown_start())
                {
                    if (time >= SHUTDOWN_THRESH)
                    {
                        TRACE((GPIO_TRACE_LEVEL, "Shutdown system"));
						DumpMessage("Shutdown system ***\r\n");
                    }
					
					/*else if (time >= DOG_PARK_THRESH)
                    {
                        if (GetDogParkMode() == DOGPARK_ENABLED)
                        {
                            TRACE((GPIO_TRACE_LEVEL, "Deactivate dog park mode"));
                            DumpMessage("Deactivate dog park mode\r\n");

                            SetDogParkMode(DOGPARK_DISABLED);
                            Config_SetMode(NO_CHANGE_MODE, 0, NULL, TRUE);
                        }
                        else
                        {
                            TRACE((GPIO_TRACE_LEVEL, "Activate dog park mode"));
                            DumpMessage("Activate dog park mode\r\n");
                            SetDogParkMode(DOGPARK_ENABLED);
                            Config_SetMode(DOG_PARK_MODE, 0, NULL, TRUE);
                        }
                        SetDogWalkMode(DOGWALK_DISABLED);
                    }*/ 
                    /*else if (time >= DOG_WALK_THRESH)
                    {
                        if (GetDogWalkMode() == DOGWALK_ENABLED)
                        {
                            TRACE((GPIO_TRACE_LEVEL, "Deactivate dog walk mode"));
                            DumpMessage("Deactivate dog walk mode\r\n");
                            SetDogWalkMode(DOGWALK_DISABLED);
                            Config_SetMode(NO_CHANGE_MODE, 0, NULL, TRUE);
                        }
                        else
                        {
                            TRACE((GPIO_TRACE_LEVEL, "Activate dog walk mode"));
                            DumpMessage("Activate dog walk mode\r\n");
                            SetDogWalkMode(DOGWALK_ENABLED);
                            Config_SetMode(WALK_MODE, 0, NULL, TRUE);
                        }
                        SetDogWalkMode(DOGWALK_DISABLED);
                    }*/
                }
            }
        }
    }
error:
    prevButtonValue = ButtonValue;
}

bool InputButtonActive(void)
{
    return GpioRead(g_inputButton, &g_InputButtonHandle);
}
