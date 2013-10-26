

#include <adl_global.h>
#include "adc.h"
#include "ConfigSettings.h"
#include "status.h"
#include "XactUtilities.h"
#include "gpioTest.h"
#include "alarm_suppress.h"
#include "Traces.h"

DEBUG_TRACE_STORAGE;


//FIXME: externs...bad!!!!!!!!!
extern int g_ShutDownReq;

adl_tmr_t * battery_timer = NULL;
adl_tmr_t * Analog_timer = NULL;

s32 g_AvgAnalogVltg;
s32 g_BatteryVltg;
s32 g_BoardTemp;
UINT32 g_TotalRunTime=0;


UINT8 g_EngineStatus = Ign_Idle;
adl_rtcTime_t EngineStartTime;		
adl_rtcTime_t EngineStopTime;
//BOOL g_SengEngineOff = FALSE;

extern s32 GPIO_5_Handle;			
extern s32 GPIO_7_Handle;
extern bool g_DebugEnable;
extern BOOL IgnAlarmGenerated;

static void CheckTemperature(void)
{

    int temperature_c = 0;

    {
        #define TEMP_ADC (3)
        s32 adcValueTemp_mv;
        s32 result = adl_adcAnalogRead(TEMP_ADC, &adcValueTemp_mv);
        ASSERT(result == OK);

        temperature_c = (adcValueTemp_mv - 500) / 10;
        wm_sprintf(g_traceBuf, "Temp (C) = %d\r\n temp mv = %d\r\n", temperature_c, (int )adcValueTemp_mv);
        DumpMessage(g_traceBuf);
    }

    {
        static s32 ChargeEnableGPIOHandle = -1;
        int const SUSPEND_CHARGE_GPIO = 12;
        #define MIN_TEMP                10  // min temp in C
        #define MAX_TEMP                50  // max temp in C
        #define TemperatureIsInvalid(temp)    ((temp < MIN_TEMP) || (temp > MAX_TEMP))
        GpioWrite(SUSPEND_CHARGE_GPIO, &ChargeEnableGPIOHandle,
                  (bool)TemperatureIsInvalid(temperature_c));
    }
}

//TODO: move the g_status stuff to status

//static void UpdateBatteryLevel(int usb_connected)
//{
//	#define BATTERY_ADC             (0)
//
//	// The thresholds for high, medium and low battery level in mV
//	#define BATTERY_HIGH            3900
//	#define BATTERY_MID             3800
//	#define BATTERY_LOW             3700
//	#define BATTERY_DANGER          3400
//	#define USB_V_OFFSET            (0) // the number of mv to increase the battery levels for when USB is attached.
//
//    s32 adcValue_mv;
//    int high = BATTERY_HIGH;
//    int mid = BATTERY_MID;
//	static bool Shutdown_Alarm_Sent = 0;
//	
//    s32 result = adl_adcAnalogRead(BATTERY_ADC, &adcValue_mv);
//    ASSERT_GOTO(result >= 0, done);
//
//    wm_sprintf(g_traceBuf, "battery = %d\r\n", (int)adcValue_mv);
//    DumpMessage(g_traceBuf);
//	DumpMessageUSB(g_traceBuf,1);
//
//	g_BatteryVltg = adcValue_mv;
//	
//    if (usb_connected)  
//    {
//        high += USB_V_OFFSET;
//        mid += USB_V_OFFSET;
//    }
//    else
//    {
//        if (adcValue_mv <= BATTERY_DANGER)
//        {
//            DumpMessage("Battery dangerously low ... going to shutdown!\r\n");
//            g_ShutDownReq = 1;
//            if ((Shutdown_Alarm_Sent == 0) && ((g_config.BattAlert == BATT_LEVEL_1_SHUTDOWN_ALERT_ON) ||
//                    (g_config.BattAlert == BATT_LEVEL_2_SHUTDOWN_ALERT_ON) ||
//                    (g_config.BattAlert == BATT_ALERT_OFF_SHUTDOWN_ALERT_ON)))
//            {
//                DumpMessage("Shutdown Alarm!\r\n");
//                // generate the shutdown alarm.
//                g_status.BattAlarm = 'S';
//				Shutdown_Alarm_Sent = 1;
//            }
//            goto done;
//        }
//    }
//
//    if (adcValue_mv >= high)
//    {
//        g_status.BattLevel = 3;
//    }
//    else if (adcValue_mv >= mid)
//    {
//        g_status.BattLevel = 2;
//    }
//    else
//    {
//        g_status.BattLevel = 1;
//    }
//
//done:
//    return;
//}

//static void CheckBattery(void)
//{
//    UpdateBatteryLevel(IsUSBConnected());
//
//    // check if the low battery alarm is enabled.
//    if (((g_config.BattAlert != BATT_ALERT_OFF_SHUTDOWN_ALERT_OFF) &&
//                (g_config.BattAlert != BATT_ALERT_OFF_SHUTDOWN_ALERT_ON)) &&
//            (g_status.BattAlarm != 'S'))
//    {
//        u8 calc_batt_alarm_level;
//        // calculate the battery level that generates the low battery alarm.
//        switch (g_config.BattAlert)
//        {
//            case BATT_LEVEL_1_SHUTDOWN_ALERT_OFF:
//                calc_batt_alarm_level = 1;
//                break;
//
//            case BATT_LEVEL_2_SHUTDOWN_ALERT_OFF:
//                calc_batt_alarm_level = 2;
//                break;
//
//            case BATT_LEVEL_1_SHUTDOWN_ALERT_ON:
//                calc_batt_alarm_level = 1;
//                break;
//
//            case BATT_LEVEL_2_SHUTDOWN_ALERT_ON:
//                calc_batt_alarm_level = 2;
//                break;
//
//            default:
//                calc_batt_alarm_level = 1;
//        }
//
//        {
//            static int AlarmCleared = 0;
//            // set the alarm in the status structure if needed.
//            if ((g_status.BattLevel <= calc_batt_alarm_level) && (AlarmCleared))
//            {
//                g_status.BattAlarm = 'Y';
//                AlarmCleared = 0;
//                alarm_suppress_set_alarm_time(BAT_ALARM_SUP);
//            }
//            else if ((g_status.BattLevel > calc_batt_alarm_level) && (alarm_suppress_status(BAT_ALARM_SUP) == ALARM_EXPIRED))
//            {
//                g_status.BattAlarm = 'N';
//                AlarmCleared = 1;
//            }
//        }
//    }
//}

static void CheckBatteryAndTemperatureTimer(u8 timerid, void *context)
{
    (void) timerid;
    (void) context;
    ASSERT(battery_timer == NULL);
	TRACE((3, "Check Battery Timer Triggered"));
    
    CheckTemperature();
//    CheckBattery();	- AEW Jr. 
    battery_timer = adl_tmrSubscribe(FALSE, 200, ADL_TMR_TYPE_100MS, CheckBatteryAndTemperatureTimer);
}

void StartADCReadTimer(void)
{
    ASSERT(battery_timer != NULL);
    if (battery_timer != NULL)
    {
        adl_tmrUnSubscribe(battery_timer, CheckBatteryAndTemperatureTimer, ADL_TMR_TYPE_100MS);
    }
    battery_timer = adl_tmrSubscribe(FALSE, 1, ADL_TMR_TYPE_100MS, CheckBatteryAndTemperatureTimer);
	
    
}

void StopADCReadTimer(void)
{
    s32 result;

    result = adl_tmrUnSubscribe(battery_timer, CheckBatteryAndTemperatureTimer, ADL_TMR_TYPE_100MS);
    battery_timer = NULL;
}

void ReadAnalogSensorVoltage(u8 timerid, void *context)
{
	#define ANALOG_SENSOR      (1)
	#define TEMP_SENSOR        (2)
	#define IGN_ON_THRESHOLD   (4200)  //13.2 -> Max Analog value
	#define IGN_OFF_THRESHOLD  (3800)  //Proportionate to the max
	
	(void) timerid;
	(void) context;
	
	static s32 CumulativeVltg=0;  
	s32 analogValue_mv=0;
	static UINT8  count=0;
	s32 result=0;	
		
	result = adl_adcAnalogRead(TEMP_SENSOR, &analogValue_mv);
	g_BoardTemp = analogValue_mv;

	result = adl_adcAnalogRead(ANALOG_SENSOR, &analogValue_mv);
	

	CumulativeVltg += analogValue_mv;
	count++;
	if(g_EngineStatus == Ign_On)
		g_TotalRunTime++;
	
	if(count >= g_ModeConfig.SampleCount)
	{
		g_AvgAnalogVltg = CumulativeVltg/count;
		wm_sprintf(g_traceBuf, "g_AvgAnalogVltg:%ld\r\n",g_AvgAnalogVltg);
		DumpMessage(g_traceBuf);		
		DumpMessageUSB(g_traceBuf,1);
		if(IgnAlarmGenerated == TRUE && g_AvgAnalogVltg >= IGN_ON_THRESHOLD)
		{
			//Declare Engine ON
			g_EngineStatus = Ign_On;
			//Store RTC time			
			result = adl_rtcGetTime(&EngineStartTime);
			ASSERT(result == OK);
		}
		else if (IgnAlarmGenerated == TRUE && g_AvgAnalogVltg <= IGN_OFF_THRESHOLD)
		{
			//Check if engine was on 
			if(g_EngineStatus == Ign_On)
			{
				//Store RTC time				
				result = adl_rtcGetTime(&EngineStopTime);
				ASSERT(result == OK);
				//Send IO pkt 
//				g_SengEngineOff = TRUE;
				sms_send('I');
				g_TotalRunTime = 0;
				//Turn off sampling 	
				StopAnalogSensorReadTimer();				
			}
			
			//Engine off- Keep monitoring Analog vtg.
			g_EngineStatus = Ign_Off;
		}
		count = 0;
		CumulativeVltg = 0;
		
		//Disable analog sampling if Ignition IO is not Active.
		if(IgnAlarmGenerated == 0)
			StopAnalogSensorReadTimer();
	}
	
	wm_sprintf(g_traceBuf, "CumulativeVltg:%ld, g_BoardTemp:%ld, Sample Count:%d\r\n",CumulativeVltg, g_BoardTemp, g_ModeConfig.SampleCount);
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf,1);

	if(g_DebugEnable)
	{
		wm_sprintf(g_traceBuf, "analogValue_mv:%ld\r\n, GPIO5:%ld\r\n, GPIO7:%ld\r\n",analogValue_mv, GpioRead(5, &GPIO_5_Handle), GpioRead(7, &GPIO_7_Handle));
		DumpMessage(g_traceBuf);
		DumpMessageUSB(g_traceBuf,1);
		
	}


	
}

void StartAnalogSensorReadTimer()
{
	//Sample data every one second.
	Analog_timer = adl_tmrSubscribe(TRUE, 10, ADL_TMR_TYPE_100MS, ReadAnalogSensorVoltage);
	ASSERT(Analog_timer != NULL);

}

void StopAnalogSensorReadTimer()
{
    s32 result;

    result = adl_tmrUnSubscribe(Analog_timer, ReadAnalogSensorVoltage, ADL_TMR_TYPE_100MS);
    Analog_timer = NULL;

}

