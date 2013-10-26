/** @addtogroup PowerCtrl
 *@{*/

/**
 * @file PowerCtrl.c
 * @brief Power Control methods
 */

#include <adl_global.h>
#include "PowerCtrl.h"
#include "LED.h"
#include "Timers.h"
#include "gpioTest.h"
#include "GPRS.h"
#include "GpsInterface.h"
#include "GPSCtrl.h"
#include "ConfigSettings.h"
#include "SimOperations.h"
#include "diagnose.h"
#include "status.h"
#include "XactUtilities.h"
#include "Accelerometer.h"
#include "dogwalk.h"
#include "Traces.h"
#include "common.h"
#include "ota_at.h"


DEBUG_TRACE_STORAGE;

static ascii *SLOW_IDLE_MODE = "AT+W32K=1,0";
static ascii *DISABLE_UART_1 = "AT+WMFM=0,0,1";
static ascii *DISABLE_UART_2 = "AT+WMFM=0,0,2";
static ascii *DISABLE_USB = "AT+WMFM=0,0,3";

static ascii *ENABLE_UART_1 = "AT+WMFM=0,1,1";
static ascii *ENABLE_UART_2 = "AT+WMFM=0,1,2";
static ascii *ENABLE_USB = "AT+WMFM=0,1,3";
static ascii *FAST_IDLE_MODE = "AT+W32K=0";
static ascii *SHUTDOWN_STR = "AT+CPOF=1";

static int InSlowIdle = 0;

static void PowerCtrlMessageHandler(u32 MsgID, adl_ctxID_e Source, u32 Length, void *Data);
//static void SubscribeFSMTimer(void);
void SubscribeFSMTimer(void);

static void SubscribeTimedWakeUpTimer(void);
static void ShutDown(void);
static void PowerCtrl_SetTXMode(void);
static int PowerCtrl_StayFastIdle(void);
static void CheckShutDown(void);

static adl_tmr_t *pTmrPtr = NULL;

extern FAST_IDLE_REQ AccelFastIdleReq;
extern FAST_IDLE_REQ VibrateFastIdleReq;
static FAST_IDLE_REQ TimedWakeupFastIdleReq = FAST_IDLE_REQ_RELEASE;
extern FAST_IDLE_REQ PacketRXFastIdleReq;

extern bool enable_ota38;


TRACKING_MODE_REQ TimedWakeupTrackingReq = TRACKING_MODE_REQ_RELEASE;

extern int g_sim_full_init_stat;
extern UINT8 g_DogParkMode;

int g_ShutDownReq = 0;
int g_AppearPowerDown = 0;

typedef enum
{
	POWER_UP,
	WAIT_GPS,
	WAIT_TCP,
	SLOW_IDLE,
	FAST_IDLE,
	FAST_IDLE_TW_PCTL,
} POWER_STATE;

static POWER_STATE state = POWER_UP;

static TRANSMIT_MODE ActualTransmitMode = SMS;

/** @brief Enter slow idle mode
 *
 * @return void
 */
void EnterSlowIdleMode(void)
{
	s8 Ret_Value = 0;

	// Do not enter slow idle if we are vibrating.
	if (VibrateFastIdleReq == FAST_IDLE_REQ_ACTIVE)
	{
		goto done;
	}

	if (g_config.BreadCrumbMode)
	{
		g_status.MotionAlarm = 'B';
	}

	state = SLOW_IDLE;

	DumpMessage("Going into slow idle mode!!\r\n");
	DumpMessageUSB("Going into slow idle mode!!\r\n",1);

	// Write out 2D fix if we never had a 3D fix but got a 2D fix.
	Proc2DFix();

	SetAccelTriggerThreshold(g_config.AccelThreshWake);
	stopTimers();

	Ret_Value = adl_atCmdCreate(DISABLE_UART_1, FALSE, NULL, NULL);
	ASSERT(Ret_Value == OK);

	Ret_Value = adl_atCmdCreate(DISABLE_UART_2, FALSE, NULL, NULL);
	ASSERT(Ret_Value == OK);

	Ret_Value = adl_atCmdCreate(DISABLE_USB, FALSE, NULL, NULL);
	if (Ret_Value != OK)
	{
		DisplayErrorCode("adl_atCmdCreate", __FILE__, __LINE__, Ret_Value);
	}

	Ret_Value = adl_atCmdCreate(SLOW_IDLE_MODE, FALSE, NULL, NULL);
	ASSERT(Ret_Value == OK);

	ShutDownGPS();
	ShutOffLED();

	InSlowIdle = 1;

done:
    return;
}


/** @brief Exit Slow Idle Mode
 *
 * @return void
 */
void ExitSlowIdleMode(void)
{
	s8 Ret_Value = 0;
	TCP_STATUS status;

	SetFixStatus(NEVER_HAD_FIX);

	Ret_Value = adl_atCmdCreate(FAST_IDLE_MODE, FALSE, NULL, NULL);
	ASSERT(Ret_Value == OK);

	Ret_Value = adl_atCmdCreate(ENABLE_UART_1, FALSE, NULL, NULL);
	ASSERT(Ret_Value == OK);

	Ret_Value = adl_atCmdCreate(ENABLE_UART_2, FALSE, NULL, NULL);
	ASSERT(Ret_Value == OK);

	Ret_Value = adl_atCmdCreate(ENABLE_USB, FALSE, NULL, NULL);
	//ASSERT(Ret_Value == OK);

	if (g_config.MotionAlarmThresh > 0)
	{
		if (FAST_IDLE_ACCEL_LEVEL >= g_config.MotionAlarmThresh)
		{
			SetAccelTriggerThreshold(g_config.MotionAlarmThresh);
		}
		else
		{
			SetAccelTriggerThreshold(FAST_IDLE_ACCEL_LEVEL);
		}
	}
	else
	{
		SetAccelTriggerThreshold(FAST_IDLE_ACCEL_LEVEL);
	}

	startTimers();
	InSlowIdle = 0;

	// Try and start the TCP connection
	GetTCPStatus(&status);
	if ((status != TCP_CONNECT) && ((g_config.SMSorGPRS == GPRS_UDP_ROAMING) || (g_config.SMSorGPRS == GPRS_TCP_ROAMING) ||
	                                (g_config.SMSorGPRS == GPRS_UDP_FALLBACK_ROAMING) || (g_config.SMSorGPRS == GPRS_TCP_FALLBACK_ROAMING) ||
	                                (g_config.SMSorGPRS == GPRS_UDP_NO_ROAMING) || (g_config.SMSorGPRS == GPRS_TCP_NO_ROAMING) ||
	                                (g_config.SMSorGPRS == GPRS_UDP_FALLBACK_NO_ROAMING) || (g_config.SMSorGPRS == GPRS_TCP_FALLBACK_NO_ROAMING)))
	{
		StartGPRS();
	}

	if (g_config.BreadCrumbMode)
	{
		g_status.MotionAlarm = 'B';
	}

	DumpMessage("Exit slow idle\r\n");
}


/** @brief Get slow idle mode state
 *
 * @return InSlowIdle
 */
int GetSlowIdleMode(void)
{
	return InSlowIdle;
}


/** @brief Display current power state
 *
 * @par
 * Send current power state as a debug message
 * @return void
 */
void DisplayPowerState(void)
{
	switch (state)
	{
	case POWER_UP:
		DumpMessage("POWER_UP\r\n");
		DumpMessageUSB("POWER_UP\r\n", 1);
		if(enable_ota38) send_ota_response("POWER_UP");
		break;

	case WAIT_GPS:
		DumpMessage("   WAIT_GPS\r\n");
		DumpMessageUSB("   WAIT_GPS\r\n", 1);
		if(enable_ota38) send_ota_response("WAIT_GPS");
		break;

	case WAIT_TCP:
		DumpMessage("  WAIT_TCP\r\n");
		DumpMessageUSB("  WAIT_TCP\r\n", 1);
		if(enable_ota38) send_ota_response("POWER_UP");
		break;

	case SLOW_IDLE:
		DumpMessage("  SLOW_IDLE\r\n");
		DumpMessageUSB("  SLOW_IDLE\r\n", 1);
		if(enable_ota38) send_ota_response("WAIT_TCP");
		break;

	case FAST_IDLE:
		DumpMessage("  FAST_IDLE\r\n");
		DumpMessageUSB("  FAST_IDLE\r\n", 1);
		if(enable_ota38) send_ota_response("FAST_IDLE");
		break;

	case FAST_IDLE_TW_PCTL:
		DumpMessage("  FAST_IDLE_TW_PCTL\r\n");
		DumpMessageUSB("  FAST_IDLE_TW_PCTL\r\n", 1);
		if(enable_ota38) send_ota_response("FAST_IDLE_TW_PCTL");
		break;
	}
}


/** @brief Power Control timer Callback method
 *
 * @param timerID
 * @param Context
 * @return void
 */
void PowerCtrlFSM(u8 timerID, void *Context)
{
	(void)timerID;
	(void)Context;

	TCP_STATUS tcp_stat;

	//Timer expired. Set the pointer to NULL.
	pTmrPtr = NULL;

	PowerCtrl_SetTXMode();


	CheckShutDown();

	switch (state)
	{
	// this initial state when the system powers on.
	case POWER_UP:
		state = WAIT_GPS;
		SubscribeFSMTimer();
		break;

	// wait for the gps to be initalized.
	case WAIT_GPS:
		if (IsGPSInitialized() && IsSimInit())
		{
			if ((g_config.SMSorGPRS == GPRS_UDP_ROAMING) || (g_config.SMSorGPRS == GPRS_TCP_ROAMING) ||
			    (g_config.SMSorGPRS == GPRS_UDP_FALLBACK_ROAMING) || (g_config.SMSorGPRS == GPRS_TCP_FALLBACK_ROAMING) ||
			    (g_config.SMSorGPRS == GPRS_UDP_NO_ROAMING) || (g_config.SMSorGPRS == GPRS_TCP_NO_ROAMING) ||
			    (g_config.SMSorGPRS == GPRS_UDP_FALLBACK_NO_ROAMING) || (g_config.SMSorGPRS == GPRS_TCP_FALLBACK_NO_ROAMING))
			{
				state = WAIT_TCP;
				StartGPRS();
			}
			else
			{
				state = FAST_IDLE;
			}
		}
		SubscribeFSMTimer();
		break;

	case WAIT_TCP:
		GetTCPStatus(&tcp_stat);
		if (tcp_stat != TCP_NOT_INIT)
		{
			state = FAST_IDLE;
		}
		SubscribeFSMTimer();
		break;

	// wait for a qualifying event to wake from slow idle.
	case SLOW_IDLE:
		if ((AccelFastIdleReq == FAST_IDLE_REQ_ACTIVE) || (PacketRXFastIdleReq == FAST_IDLE_REQ_ACTIVE))
		{
			state = FAST_IDLE;
			ExitSlowIdleMode();
			SubscribeFSMTimer();
		}
		else if (TimedWakeupFastIdleReq == FAST_IDLE_REQ_ACTIVE)
		{
			TRACE((1, "Timed wakup event!"));
			state = FAST_IDLE_TW_PCTL;
			ExitSlowIdleMode();
			SubscribeFSMTimer();
		}
		break;

	// wait to be done in fast idle operation.
	case FAST_IDLE:
		// sms server stays in fast idle.
		if (PowerCtrl_StayFastIdle())
		{
			state = FAST_IDLE;
			//DumpMessage("call from FAST_IDLE 1\r\n");			
			SubscribeFSMTimer();
		}
		else if (AccelFastIdleReq == FAST_IDLE_REQ_RELEASE)
		{
			EnterSlowIdleMode();
		}
		else
		{
			state = FAST_IDLE;
			//DumpMessage("call from FAST_IDLE 2\r\n");
			SubscribeFSMTimer();
		}
		break;

	// fast idle mode triggered by the timed wakeup
	case FAST_IDLE_TW_PCTL:
		if (TimedWakeupFastIdleReq == FAST_IDLE_REQ_RELEASE)
		{
			EnterSlowIdleMode();
		}
		else
		{
			state = FAST_IDLE_TW_PCTL;
			SubscribeFSMTimer();
		}
		break;

	default:
		DumpMessage("Unknown power control state\r\n");
		state = FAST_IDLE;
	}
}


/** @brief Setup Power Control Message Handler
 *
 * @par Subscribe to the Power Control Message Handler
 * The power controller can be aware of accelerometer interrupts.
 *
 * @return void
 */
static void SetupPowerCtrlMsgHandler(void)
{
	adl_msgFilter_t Filter = { 0, 
                               0, 
                               ADL_MSG_ID_COMP_GREATER, 
                               ADL_CTX_HIGH_LEVEL_IRQ_HANDLER };

	s32 msgHandle = adl_msgSubscribe(&Filter, PowerCtrlMessageHandler);
    ASSERT(msgHandle >= 0);
}



/** @brief Get Power Control Tx Mode
 *
 * @return ActualTransmitMode
 */
TRANSMIT_MODE PowerCtrl_GetTXMode(void)
{
	return ActualTransmitMode;
}


/** @brief Set the transmit mode.
 *
 * @par
 * This method sets the actual tranmist mode based on the configured transmit
 * mode and the current status of GPRS. This is to support the SMS and GPRS
 * fallback functionality.
 * @return void
 */
static void PowerCtrl_SetTXMode(void)
{
	TCP_STATUS status;

	switch (g_config.SMSorGPRS)
	{
	case SMS_ROAMING:
		ActualTransmitMode = SMS;
		break;

	case GPRS_UDP_ROAMING:
		ActualTransmitMode = GPRS;
		break;

	case GPRS_TCP_ROAMING:
		ActualTransmitMode = GPRS;
		break;

	case GPRS_UDP_FALLBACK_ROAMING:
		GetTCPStatus(&status);
		if (status != TCP_CONNECT)
		{
			ActualTransmitMode = SMS;
		}
		else
		{
			ActualTransmitMode = GPRS;
		}
		break;

	case GPRS_TCP_FALLBACK_ROAMING:
		GetTCPStatus(&status);
		if (status != TCP_CONNECT)
		{
			ActualTransmitMode = SMS;
		}
		else
		{
			ActualTransmitMode = GPRS;
		}
		break;

	case SMS_NO_ROAMING:
		ActualTransmitMode = SMS;
		break;

	case GPRS_UDP_NO_ROAMING:
		ActualTransmitMode = GPRS;
		break;

	case GPRS_TCP_NO_ROAMING:
		ActualTransmitMode = GPRS;
		break;

	case GPRS_UDP_FALLBACK_NO_ROAMING:
		GetTCPStatus(&status);
		if (status != TCP_CONNECT)
		{
			ActualTransmitMode = SMS;
		}
		else
		{
			ActualTransmitMode = GPRS;
		}
		break;

	case GPRS_TCP_FALLBACK_NO_ROAMING:
		GetTCPStatus(&status);
		if (status != TCP_CONNECT)
		{
			ActualTransmitMode = SMS;
		}
		else
		{
			ActualTransmitMode = GPRS;
		}
		break;

	default:
		ActualTransmitMode = SMS;
		break;
	}

	return;
}


/** @brief Power Control Message Handler
 *
 * @par
 * This will handle messages from the accelerometer interrupt.
 * @param MsgID
 * @param Source
 * @param Length
 * @param Data
 *
 * @return void
 */
static void PowerCtrlMessageHandler(u32 MsgID, adl_ctxID_e Source, u32 Length, void *Data)
{
	(void)MsgID;
	(void)Source;
	(void)Length;
	(void)Data;

	if (state == SLOW_IDLE)
	{
		SubscribeFSMTimer();
	}
}


/** @brief Initalize the Power Control
 *
 * @return void
 */
void InitPowerCtrl(void)
{
	SetupPowerCtrlMsgHandler();
	SubscribeFSMTimer();
	SubscribeTimedWakeUpTimer();
}


/** @brief Subscribe to FSM Timer
 *
 * @par
 * This is resubscribe the state machine timer. It is called whenever we want to device to
 * stay awake. It is not called when we go into low power mode.
 * @return void
 */
//static void SubscribeFSMTimer(void)
void SubscribeFSMTimer(void)
{
	//static int count=0;
	//int sReturn;

	//This check is required as, this timer gets triggered from multiple sources. 
	//This is designed to be a single shot timer and only one instance of this should be active
	if(pTmrPtr == NULL)
	{
	
		if ((state == POWER_UP) || (state == WAIT_GPS))
		{
			if ((pTmrPtr = adl_tmrSubscribe(FALSE, 200, ADL_TMR_TYPE_100MS, PowerCtrlFSM)) == NULL)
			{
				DisplayErrorCode("adl_tmrSubscribe", __FILE__, __LINE__, (s32)(pTmrPtr));
			}
			DumpMessage("Subscribing for 20 sec\r\n");
		}
		else
		{
			if ((pTmrPtr = adl_tmrSubscribe(FALSE, 10, ADL_TMR_TYPE_TICK, PowerCtrlFSM)) == NULL)
			{
				DisplayErrorCode("adl_tmrSubscribe", __FILE__, __LINE__, (s32)(pTmrPtr));
			}
			//count++;
			//wm_sprintf(g_traceBuf, "SubscribeFSMTimer Count:%d \r\n", count);
			//DumpMessage(g_traceBuf);

		}
	}
	else
		DumpMessage("FSM Timer is already running\r\n");
}


typedef enum
{
	POWER_UP_TW,
	WAIT_DURATION,
	WAIT_FAST_IDLE,
	FAST_IDLE_TW
} TIMED_WAKEUP_STATE;

/** @brief Wake up timer callback method
 *
 * @par This state machine keeps track of the amount of time
 * between wakeups and will bring the device out of low power
 * mode when needed. This duration between wakeups is configurable.
 * @param timerID
 * @param Context
 * @return void
 */
static void TimedWakeUpTimer(u8 timerID, void *Context)
{
	(void)timerID;
	(void)Context;

	adl_rtcTimeStamp_t CurrentTimeStamp;
	adl_rtcTimeStamp_t DeltaTimeStamp;
	static adl_rtcTimeStamp_t LastWakeupTimeStamp;

	s32 sReturn = -1;

	//DumpMessage("TimedWakeUpTimer\r\n");
	static TIMED_WAKEUP_STATE tw_state = POWER_UP_TW;

	// Ping the watchdog.
	adl_wdRearmAppWd();

	// if we're in slow idle mode, do a single shot of the transmit handler.
	if (GetSlowIdleMode())
	{
		CheckAccelSlowIdle();
		singleTxHandlerTimer();
	}

	// check the receive packet fast idle request.
	if (PacketRXFastIdleReq == FAST_IDLE_REQ_ACTIVE)
	{
		//DumpMessage("call from TimedWakeUpTimer 1\r\n");
		SubscribeFSMTimer();
	}

	switch (tw_state)
	{
	// called initally at startup.
	case POWER_UP_TW:
		TRACE((1, "***TW_STATE***: POWER_UP_TW***"));
		GetConvertTime(&LastWakeupTimeStamp);
		tw_state = WAIT_DURATION;
		TimedWakeupTrackingReq = TRACKING_MODE_REQ_RELEASE;
		break;

	// waiting for the set timed duration to elapse.
	case WAIT_DURATION:
		TRACE((1, "***TW_STATE***: WAIT_DURATION***"));
		TimedWakeupTrackingReq = TRACKING_MODE_REQ_RELEASE;
		GetConvertTime(&CurrentTimeStamp);
		if ((sReturn = adl_rtcDiffTime(&CurrentTimeStamp, &LastWakeupTimeStamp, &DeltaTimeStamp)) < 0)
		{
			DisplayErrorCode("adl_rtcConvertTime", __FILE__, __LINE__, sReturn);
			break;
		}

		TRACE((1, "Current Time Stamp = %d Delta Time Stamp = %d", CurrentTimeStamp.TimeStamp, DeltaTimeStamp.TimeStamp));

		if ((DeltaTimeStamp.TimeStamp >= g_config.TimedWakeup * 10) && (g_config.TimedWakeup != 0))
		{
			SubscribeFSMTimer();
			TimedWakeupFastIdleReq = FAST_IDLE_REQ_ACTIVE;
			tw_state = WAIT_FAST_IDLE;
		}
		else
		{
			tw_state = WAIT_DURATION;
		}

		break;

	// waiting for the TD to switch fast idle mode.
	case WAIT_FAST_IDLE:
		TRACE((1, "***TW_STATE***: WAIT_FAST_IDLE***"));
		TimedWakeupTrackingReq = TRACKING_MODE_REQ_RELEASE;
		if (!GetSlowIdleMode())
		{
			tw_state = FAST_IDLE_TW;
			GetConvertTime(&LastWakeupTimeStamp);
		}
		else
		{
			tw_state = WAIT_FAST_IDLE;
		}
		break;

	// TD is in fast idle mode.
	case FAST_IDLE_TW:
		TRACE((1, "***TW_STATE***: FAST_IDLE_TW***"));
		GetConvertTime(&CurrentTimeStamp);
		if ((sReturn = adl_rtcDiffTime(&CurrentTimeStamp, &LastWakeupTimeStamp, &DeltaTimeStamp)) < 0)
		{
			DisplayErrorCode("adl_rtcConvertTime", __FILE__, __LINE__, sReturn);
			break;
		}
		TimedWakeupTrackingReq = TRACKING_MODE_REQ_ACTIVE;
		if (DeltaTimeStamp.TimeStamp >= TIMED_WAKEUP_DURATION)
		{
			GetConvertTime(&LastWakeupTimeStamp);
			TimedWakeupFastIdleReq = FAST_IDLE_REQ_RELEASE;
			tw_state = WAIT_DURATION;
		}
		else
		{
			tw_state = FAST_IDLE_TW;
		}
		break;

	default:
		DumpMessage("Unknown state\r\n");
		tw_state = WAIT_DURATION;
	}
}


/** @brief Subscribe to wakeup timer
 *
 * @return void
 */
static void SubscribeTimedWakeUpTimer(void)
{
	adl_tmr_t *pTmrPtr;
	if ((pTmrPtr = adl_tmrSubscribe(TRUE, 540, ADL_TMR_TYPE_TICK, TimedWakeUpTimer)) == NULL)
	{
		DisplayErrorCode("adl_rtcGetTime", __FILE__, __LINE__, (s32)(pTmrPtr));
	}
}


static void CheckShutDown(void)
{
	if (g_ShutDownReq && (g_status.BattAlarm != 'S'))
	{
		ShutDown();
	}
}


/** @brief Shutdown the unit
 *
 * @return void
 */
static void ShutDown(void)
{
	s8 Ret_Value = 0;
	DumpMessage("Going to shut down!\r\n");
	// issue the shutdown command.
	Ret_Value = adl_atCmdCreate(SHUTDOWN_STR, FALSE, NULL, NULL);
	if (Ret_Value != OK)
	{
		DisplayErrorCode("adl_atCmdCreate", __FILE__, __LINE__, Ret_Value);
	}
}


/** @brief Check if we should stay in Fast Idle Mode
 *
 * @return 1 stay in fast idle
 * @return 0 exit fast idle
 */
static int PowerCtrl_StayFastIdle(void)
{
	static int PacketRXAwakeCnt = 0;

	adl_rtcTimeStamp_t CurrentTimeStamp, DeltaTimeStamp;
	static adl_rtcTimeStamp_t FirstTimeStamp;
	int sReturn;

	if (g_config.Mode == SERVER_NO_SLEEP)
	{
		return 1;
	}
	if(g_config.Mode == FULL_TRACK_MODE)
	{
		return 1;
	}
	if(g_config.Mode == ALARM_MODE)
	{
		return 1;
	}
	if (GetDogWalkMode() == DOGWALK_ENABLED)
	{
		return 1;
	}
	if (GetDogParkMode() == DOGPARK_ENABLED)
	{
		return 1;
	}
	if (diagnose_get_mode() == DIAGNOSE_ON)
	{
		return 1;
	}
	//if (IsUSBConnected())
	//{
		//return 1;
	//}

	if (PacketRXFastIdleReq == FAST_IDLE_REQ_ACTIVE)
	{
		// on the first iteration get the first time stamp, otherwise get the current time stamp.
		if   (PacketRXAwakeCnt == 0)
		{
			GetConvertTime(&FirstTimeStamp);
		}
		else
		{
			GetConvertTime(&CurrentTimeStamp);
		}
		PacketRXAwakeCnt++;
		// calculuate how long we've been on.
		if ((sReturn = adl_rtcDiffTime(&CurrentTimeStamp, &FirstTimeStamp, &DeltaTimeStamp)) < 0)
		{
			DisplayErrorCode("adl_rtcConvertTime", __FILE__, __LINE__, sReturn);
			return 1;
		}

		// shut off if we've been on > 20 seconds.
		if ((PacketRXAwakeCnt > 0) && (DeltaTimeStamp.TimeStamp > 20))
		{
			PacketRXFastIdleReq = FAST_IDLE_REQ_RELEASE;
			PacketRXAwakeCnt = 0;
		}
		else
		{
			return 1;
		}             // otherwise stay on.
	}

	return 0;
}

void PowerOffModem(u8 timerID, void *Context)
{
	(void)timerID;
	(void)Context;

	adl_rtcTime_t CurrentTime;
	adl_rtcTimeStamp_t CurrentTimeStamp;
	s32 sReturn=-1;
	ascii AlarmString[20];
	ascii AlarmCommand[30];
	ascii AlarmFlush[20];
	s8 Ret;
	DumpMessage("Invoking Alarm mode\r\n");
	DumpMessageUSB("Invoking Alarm mode\r\n",1);
	
	// Time.
	// get the current RTC time.
	if ((sReturn = adl_rtcGetTime(&CurrentTime)) < 0) 
	{
    	DisplayErrorCode("adl_rtcGetTime",__FILE__,__LINE__,sReturn);
    	return;
	}
    if ((sReturn = adl_rtcConvertTime( &CurrentTime, &CurrentTimeStamp, ADL_RTC_CONVERT_TO_TIMESTAMP)) < 0) 
	{
		DisplayErrorCode("adl_rtcConvertTime",__FILE__,__LINE__,sReturn);
	}

    wm_sprintf(g_traceBuf,"RTC TimeStamp = 0x%x\r\n",(unsigned int)CurrentTimeStamp.TimeStamp);		
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf,1);

    wm_sprintf(g_traceBuf,"Date(dd:mm:yy) = %02d:%02d:%04d\r\n",CurrentTime.Day,CurrentTime.Month,CurrentTime.Year);		
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf,1);

    wm_sprintf(g_traceBuf,"Time(hh:mm:ss) = %02d:%02d:%02d\r\n",CurrentTime.Hour,CurrentTime.Minute,CurrentTime.Second);		
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf,1);


	//Set Alarm - for the tracking interval duration.
	CurrentTimeStamp.TimeStamp += GetTrackingInterval();

	if ((sReturn = adl_rtcConvertTime( &CurrentTime, &CurrentTimeStamp, ADL_RTC_CONVERT_FROM_TIMESTAMP)) < 0) 
	{
		DisplayErrorCode("adl_rtcConvertTime",__FILE__,__LINE__,sReturn);
	}

    wm_sprintf(g_traceBuf,"RTC Wake-up TimeStamp = 0x%x\r\n",(unsigned int)CurrentTimeStamp.TimeStamp);		
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf,1);

    wm_sprintf(g_traceBuf,"Wake up Date(dd:mm:yy) = %02d:%02d:%04d\r\n",CurrentTime.Day,CurrentTime.Month,CurrentTime.Year);		
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf,1);

    wm_sprintf(g_traceBuf,"Wake up Time(hh:mm:ss) = %02d:%02d:%02d\r\n",CurrentTime.Hour,CurrentTime.Minute,CurrentTime.Second);		
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf,1);

    wm_sprintf(AlarmString,"%04d/%02d/%02d,%02d:%02d:%02d",CurrentTime.Year,CurrentTime.Month,CurrentTime.Day,CurrentTime.Hour,CurrentTime.Minute,CurrentTime.Second);		
	DumpMessage(AlarmString);			
	DumpMessage("\r\n");
	DumpMessageUSB(AlarmString,1);
	DumpMessageUSB("\r\n",1);
	
	strcpy(AlarmCommand,"AT+CALA=\"");
	strcat(AlarmCommand,&AlarmString[2]);
	strcat(AlarmCommand,"\"");

	DumpMessage(AlarmCommand);
	DumpMessage("\r\n");
	DumpMessageUSB(AlarmCommand,1);
	DumpMessageUSB("\r\n",1);

	adl_atCmdCreate("AT+CALA?", FALSE, NULL, NULL);


	//Flush all the alarms before setting up wake up alarm.
	strcpy(AlarmFlush,"at+cala=\"\",xx");
	for(UINT8 i=1; i<=16; i++)
	{
		//convert integer to ascii
		if(i<=9)
		{
			AlarmFlush[11] = i+0x30;
			AlarmFlush[12] = '\0';
		}
		else
		{
			AlarmFlush[11] = (i/10)+0x30;
			AlarmFlush[12] = (i%10)+0x30;
			AlarmFlush[13] = '\0';
		}
		
		DumpMessage("\r\nFlush Command with Index:");
		DumpMessageUSB("\r\nFlush Command with Index:",1);
		DumpMessage(AlarmFlush);
		DumpMessageUSB(AlarmFlush,1);
		Ret = adl_atCmdCreate(AlarmFlush, FALSE, NULL, NULL);
		if(Ret != OK)
			DumpMessage("Alarm flush failed\r\n");		
	}
	//End - Flush alarms
		
	Ret = adl_atCmdCreate(AlarmCommand, FALSE, NULL, NULL);
	if(Ret == OK)
	{	
		DumpMessage("\r\nAlarm configured successfully\r\n");
		DumpMessageUSB("\r\nAlarm configured successfully\r\n",1);
	}
	else
	{
		DumpMessage("\r\nAlarm configuration failed\r\n");
		DumpMessageUSB("\r\nAlarm configuration failed\r\n",1);
	}

	
	DumpMessage("\r\n Going to low power Alarm mode\r\n");
	DumpMessageUSB("\r\n Going to low power Alarm mode\r\n",1);

	adl_atCmdCreate("AT+CPOF", FALSE, NULL, NULL);

}
/*@}*/
