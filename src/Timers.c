/*H************************************************************************
 */

/**
 *   @brief   Starts and stops system timers.
 *
 *   @details
 *
 *   @note    Other help for the reader, including references.
 *
 *
 *//*
 *
 * CHANGES :
 *
 * REF NO  DATE     WHO      DETAIL
 *         19Jun09  AndyB    Ported from reference design
 * NNNNNN  DDMMMYY  Name     Name of function/item changed
 *
 *H*/

/****************************************************************************
*
*  MODIFICATION HISTORY
* --------------------------------------------------------------------------
*  Flag |Date      | Author       | Revision | Description
* -----------+-------------+----------+-------------------------------------
*       |09Sep2006 | James Kilts  | 1.0      | * Initial Release
* --------------------------------------------------------------------------
*  @01  | 22Dec2006 | Biberdorf   |          | add SendUnsolicitedMsg
* --------------------------------------------------------------------------
*  @03  | 07Jan2007 | Biberdorf   |          | add gfDistance as well as
*                                            | distance calculations
* --------------------------------------------------------------------------
*  @04  | 11Jan07   | Biberdorf   |          | add Time definitions
* --------------------------------------------------------------------------
*  @05  | 01.25.07  | Biberdorf   | 2.09     |-added Voltage to flash
* --------------------------------------------------------------------------
*       |           |             |          |
* --------------------------------------------------------------------------
*
****************************************************************************/
#include "adl_global.h"


#include "XactUtilities.h"
#include "gps.h"
#include "FlashTest.h"
#include "LED.h"
#include "VibrationMotor.h"
#include "i2c.h"
#include "Accelerometer.h"
#include "status.h"
#include "ConfigSettings.h"
#include "gpioTest.h"
#include "GPSCtrl.h"
#include "SMShandling.h"
#include "alarm_suppress.h"
#include "adc.h"
#include "Timers.h"
#include "gprs.h"
#include "ota_at.h"
#include "PowerCtrl.h"
#include "internalflash.h"

//#include "pistd.h"

/****************************************************************************
*  globals
****************************************************************************/

static adl_tmr_t    *g_nextFixTimer = NULL;
static adl_tmr_t    *g_LEDTimer = NULL;
static adl_tmr_t    *g_VibrationTimer = NULL;
static adl_tmr_t    *g_startAccelStateTimer = NULL;
static adl_tmr_t    *g_GPIOTimer = NULL;
static adl_tmr_t    *g_TxHandlerTimer = NULL;
static adl_tmr_t    *g_GPSTimeoutTimer = NULL;
static adl_tmr_t    *g_DiagTimeoutTimer = NULL;
static adl_tmr_t    *g_DiagRepeatTimer = NULL;




extern bool g_gpsFirstFixFlag;
extern u16 g_BlinkReg;
extern BOOL GPSWaitTimeout;
extern AGPS_DATA g_AGPSData;

/****************************************************************************
*
*  PROTOTYPES
*
****************************************************************************/

//Battery Check Timer Functions
static void startLEDTimer(void);
static void stopLEDTimer(void);
static void startVibrationTimer(void);
static void stopVibrationTimer(void);
static void startAccelStateTimer(void);
static void stopAccelStateTimer(void);
static void startGPIOTimer(void);
static void stopGPIOTimer(void);
static void startTxHandlerTimer(void);
static void stopTxHandlerTimer(void);

/****************************************************************************
*
*  TIMER IMPLEMENTATIONS
*
****************************************************************************/

/*F***************************************************************
 *
 *   NAME:    StartTimers
 */

/** @brief   Starts all the timers for the tracking device
 *
 *//*
 *
 * CHANGES :
 * REF NO  DATE     WHO      DETAIL
 *         15Jul09  AndyB    First version
 * NNNNNN  DDMMMYY  Name     Name of function/item changed
 *F*/
void startTimers(void)
{
#if defined(BAREBONES_APPLICATION)
	startLEDTimer();
	startGPIOTimer();
#else
	startLEDTimer();
	startNextFixTimer();
	startVibrationTimer();
	startAccelStateTimer();
	startGPIOTimer();
	startTxHandlerTimer();
	StartADCReadTimer();
	startDiagMsgRepeatTimer();
#endif
}


/*F***************************************************************
 *
 *   NAME:    StopTimers
 */

/*! \brief   Stops all the timers for the tracking device
 *
 *//*
 *
 * CHANGES :
 * REF NO  DATE     WHO      DETAIL
 *         15Jul09  AndyB    First version
 * NNNNNN  DDMMMYY  Name     Name of function/item changed
 *F*/
void stopTimers(void)
{
	DumpMessage("Stop Timers\r\n");
	if (g_nextFixTimer)
	{
		stopNextFixTimer();
	}
	if (g_LEDTimer)
	{
		stopLEDTimer();
	}
	if (g_VibrationTimer)
	{
		stopVibrationTimer();
	}
	if (g_startAccelStateTimer)
	{
		stopAccelStateTimer();
	}
	if (g_GPIOTimer)
	{
		stopGPIOTimer();
	}
	if (g_TxHandlerTimer)
	{
		stopTxHandlerTimer();
	}
	if(g_DiagRepeatTimer)
	{
		stopDiagMsgRepeatTimer();
	}
	
}


void stopTimersFWUpdate(void)
{
	DumpMessage("Stop Timers FW update\r\n");
	if (g_nextFixTimer)
	{
		stopNextFixTimer();
	}
	if (g_LEDTimer)
	{
		stopLEDTimer();
	}
	if (g_VibrationTimer)
	{
		stopVibrationTimer();
	}

    StopADCReadTimer();

	if (g_startAccelStateTimer)
	{
		stopAccelStateTimer();
	}
	if (g_GPIOTimer)
	{
		stopGPIOTimer();
	}
	if (g_TxHandlerTimer)
	{
		stopTxHandlerTimer();
	}
}


/** @addtogroup GPS
 *@{*/

/** @file Timers.c
 */

/** @brief Starts the timer that checks for GPS locks
 *
 * @par
 * This method will start a periodic timer call to the handler
 * method NextFixHandler iver 50 ticks.
 *
 * @return void
 */
void startNextFixTimer()
{
	if (g_nextFixTimer != NULL)
	{
		return;
	}

	if ((g_nextFixTimer = adl_tmrSubscribe(TRUE, 50, ADL_TMR_TYPE_TICK, NextFixHandler)) == NULL)
	{
		DisplayErrorCode("adl_tmrUnSubscribe - NextFixTimer", __FILE__, __LINE__, (int )g_nextFixTimer);
	}
	else
	{
		DumpMessage("Started NextFixHandler\r\n");
	}
}


/** @brief Stops the timer that checks for GPS locks
 *
 * @return void
 */
void stopNextFixTimer()
{
	s32 sReturn;
	if ((sReturn = adl_tmrUnSubscribe(g_nextFixTimer, NextFixHandler, ADL_TMR_TYPE_TICK)) < 0)
	{
		DisplayErrorCode("adl_tmrUnSubscribe - NextFixTimer", __FILE__, __LINE__, sReturn);
	}
	else
	{
		DumpMessage("NextFixHandler stopped\r\n");
		g_nextFixTimer = NULL;
	}
}


/*@}*/

/** @addtogroup LED
 *@{*/

/** @file Timers.c
 */

/** @brief start LED Timer
 *
 * @par
 * This method will start a period timer every 100ms to call the handler function
 * BlinkLED
 *
 * @return void
 */
static void startLEDTimer()
{
	if (g_LEDTimer != NULL)
	{
		return;
	}

	if ((g_LEDTimer = adl_tmrSubscribe(TRUE, 1, ADL_TMR_TYPE_100MS, BlinkLED)) == NULL)
	{
		TRACE((1, "Could not start LED Timer"));
	}
	else
	{
		DumpMessage("Led timer started\r\n");
	}
}


/** @brief stop the LED blink timer
 *
 * @return void
 */
static void stopLEDTimer()
{
	int sReturn;
	if ((sReturn = adl_tmrUnSubscribe(g_LEDTimer, BlinkLED, ADL_TMR_TYPE_100MS)) < 0)
	{
		DisplayErrorCode("adl_tmrUnSubscribe - LedTimer", __FILE__, __LINE__, sReturn);
	}
	else
	{
		DumpMessage("LED Timer Stopped\r\n");
		g_LEDTimer = NULL;
	}
}


/*@}*/

/** @addtogroup Accel
 *@{*/

/** @file Timers.c
 */

/** @brief Start the vibration timer
 *
 * @par This method will call the hander VibrateTimer method every ms
 *
 * @return void
 */
static void startVibrationTimer()
{
	if (g_VibrationTimer != NULL)
	{
		return;
	}

	if ((g_VibrationTimer = adl_tmrSubscribe(TRUE, 1, ADL_TMR_TYPE_100MS, VibrateTimer)) == NULL)
	{
		TRACE((1, "Could not start Vibration Timer"));
	}
	else
	{
		DumpMessage("Vibration timer started\r\n");
	}
}


/** @brief Stop the vibration timer operation
 *
 * @return void
 */
static void stopVibrationTimer()
{
	int sReturn;
	if ((sReturn = adl_tmrUnSubscribe(g_VibrationTimer, VibrateTimer, ADL_TMR_TYPE_100MS)) < 0)
	{
		DisplayErrorCode("adl_tmrUnSubscribe - VibrationTimer", __FILE__, __LINE__, sReturn);
	}
	else
	{
		DumpMessage("Vibration Timer stoppped\r\n");
		g_VibrationTimer = NULL;
	}
}


/*@}*/

/** @addtogroup GPIO
 *@{*/

/** @file Timers.c
 */

/** @brief start GPIO Timer operation
 *
 * @par this method will call the GPIOTimer handler every 3 ticks
 *
 * @return void
 */
static void startGPIOTimer()
{
	if (g_GPIOTimer != NULL)
	{
		return;
	}

	g_GPIOTimer = adl_tmrSubscribe(TRUE, 3, ADL_TMR_TYPE_TICK, GPIOTimer);
}


/** @brief stop the GPIO Timer operation
 *
 * @return void
 */
static void stopGPIOTimer()
{
	int sReturn;
	if ((sReturn = adl_tmrUnSubscribe(g_GPIOTimer, GPIOTimer, ADL_TMR_TYPE_100MS)) < 0)
	{
		DisplayErrorCode("adl_tmrUnSubscribe - GPIO Timer", __FILE__, __LINE__, sReturn);
	}
	else
	{
		DumpMessage("GPIO Timer stopped\r\n");
		g_GPIOTimer = NULL;
	}
}

/*@}*/

/** @addtogroup Accel
 *@{*/

/** @brief Start Accelleration Timer
 *
 * @par this method will start a timer operation to call the CheckAccelState every 10 ticks
 *
 * @return void
 */
static void startAccelStateTimer()
{
	if (g_startAccelStateTimer != NULL)
	{
		return;
	}

	if ((g_startAccelStateTimer = adl_tmrSubscribe(TRUE, 10, ADL_TMR_TYPE_TICK, CheckAccelState)) == NULL)
	{
		DumpMessage("Could not start accel state timer\r\n");
	}
}


/** @brief stop the Acceleration timer event
 *
 * @return void
 */
static void stopAccelStateTimer()
{
	int sReturn;
	sReturn = adl_tmrUnSubscribe(g_startAccelStateTimer, CheckAccelState, ADL_TMR_TYPE_TICK);

	if (sReturn < 0)
	{
		DisplayErrorCode("adl_tmrUnSubscribe - Accel State Timer", __FILE__, __LINE__, sReturn);
	}
	else
	{
		DumpMessage("Accel State time stopped\r\n");
		g_startAccelStateTimer = NULL;
	}
}


/*@}*/

/** @addtogroup NetworkComm
 *@{*/

/** @file Timers.c
 */

/** @brief start the SMS Tx timer
 *
 * @par
 * This method starts a periodic call to the handler function SMS_TX_Handler
 * every 500 ms.
 *
 * @return void
 */
static void startTxHandlerTimer()
{
	if (g_TxHandlerTimer != NULL)
	{
		return;
	}

	if ((g_TxHandlerTimer = adl_tmrSubscribe(TRUE, 50, ADL_TMR_TYPE_100MS, SMS_TX_Handler)) == NULL)
	{
		DumpMessage("Could not start transmit handler timer\r\n");
	}
}


/** @brief one shot call to SMS Tx timer
 *
 * @par
 * This method will make a single call to the SMS handler function SMS_TX_Handler
 * in 20 ticks.
 *
 * @return void
 */
void singleTxHandlerTimer()
{
	if ((adl_tmrSubscribe(FALSE, 20, ADL_TMR_TYPE_TICK, SMS_TX_Handler)) == NULL)
	{
		DumpMessage("Could not start transmit handler timer\r\n");
	}
}


/** @brief Stop the SMS TX timer
 *
 * @par
 * Stops execution of the SMS transmit handler.
 * @return void
 */
static void stopTxHandlerTimer()
{
	int sReturn;
	sReturn = adl_tmrUnSubscribe(g_TxHandlerTimer, SMS_TX_Handler, ADL_TMR_TYPE_TICK);

	if (sReturn < 0)
	{
		DisplayErrorCode("adl_tmrUnSubscribe - TX Handler", __FILE__, __LINE__, sReturn);
	}
	else
	{
		DumpMessage("TX Handler stopped\r\n");
		g_TxHandlerTimer = NULL;
	}
}


/*@}*/

/** @addtogroup Storage
 *@{*/

/** @brief Batter Check Timer Handler
 *
 * @par
 * This method is called when the batter check timer is triggered
 *
 * @param timerid
 * @param context
 * @return void
 */
void BatteryCheckTimerHandler(u8 timerid, void *context)
{
	(void)timerid;
	(void)context;

}

void StartGPSAvailabilityTimer()
{
	//g_GPSTimeoutTimer = adl_tmrSubscribe(FALSE, 4*600, ADL_TMR_TYPE_100MS, GPSTimeoutHandler);
	//Check for the validity of timeout value.
	if(g_ModeConfig.GPSTimeout == 0 || g_ModeConfig.GPSTimeout > 9)
		g_ModeConfig.GPSTimeout = 4;   //Default to 4, if found out of range
		
	g_GPSTimeoutTimer = adl_tmrSubscribe(FALSE, g_ModeConfig.GPSTimeout*600, ADL_TMR_TYPE_100MS, GPSTimeoutHandler);
	if (g_GPSTimeoutTimer == NULL)
	{
		DumpMessage("Could not start GPSAvailabilityTimer\r\n");
		//Dont wait for time to expire as timer failed to start.
		GPSWaitTimeout = TRUE;
	}
	else
	{
		wm_sprintf(g_traceBuf,"GPSAvailabilityTimer Started : %d\r\n",g_ModeConfig.GPSTimeout);
		DumpMessage(g_traceBuf);
		GPSWaitTimeout = FALSE;
	}
}

void StopGPSAvailabilityTimer()
{
	int sReturn;
	extern BOOL WaitOn;
	sReturn = adl_tmrUnSubscribe(g_GPSTimeoutTimer, GPSTimeoutHandler, ADL_TMR_TYPE_100MS);

	if (sReturn < 0)
	{
		DisplayErrorCode("adl_tmrUnSubscribe - GPSTimeoutHandler", __FILE__, __LINE__, sReturn);
	}
	else
	{
		DumpMessage("GPSTimeoutHandler stopped\r\n");
		g_GPSTimeoutTimer = NULL;
	}
	//Reset the flag, so that the wait logic reinitiates. 
	WaitOn = FALSE;
	GPSWaitTimeout = FALSE;
	
}


void StartFTPTimer()
{
	if ((adl_tmrSubscribe(FALSE, 10*25, ADL_TMR_TYPE_100MS, ConnectFTP)) == NULL)
	{
		DumpMessage("Could not start transmit handler timer\r\n");
	}
}

//"I" packet transmit timer
void StartIPktTxTimer()
{	
	//Each sample is taken at the rate of 1 seconds. So wait 2 seconds more (to be 100% sure that you got averaged value) 
	//than the sample count time before transmitting the value.
	if ((adl_tmrSubscribe(FALSE, (g_ModeConfig.SampleCount+2)*10, ADL_TMR_TYPE_100MS, IPacketTxHandler)) == NULL)
	{
		DumpMessage("Could not start transmit handler timer\r\n");
	}

}


void startDiagMsgHandlerTimer()
{
	if (g_DiagTimeoutTimer  != NULL)
	{
		DumpMessage("startDiagMsgHandlerTimer already running\r\n");
		return;
	}

	if ((g_DiagTimeoutTimer  = adl_tmrSubscribe(TRUE, 100, ADL_TMR_TYPE_100MS, SendOTAResponse)) == NULL)
	{
		DumpMessage("Could not start diag transmit handler timer\r\n");
	}
}

void stopDiagMsgHandlerTimer()
{
	s32 sReturn = adl_tmrUnSubscribe(g_DiagTimeoutTimer, SendOTAResponse, ADL_TMR_TYPE_100MS);

	if (sReturn < 0)
	{
		DisplayErrorCode("adl_tmrUnSubscribe - g_DiagTimeoutTimer", __FILE__, __LINE__, sReturn);
	}
	else
	{
		DumpMessage("DiagMsgTimeoutHandler stopped\r\n");
		g_DiagTimeoutTimer = NULL;
	}
}

void startDiagMsgRepeatTimer()
{

	if (g_DiagRepeatTimer  != NULL)
	{
		DumpMessage("startDiagMsgHandlerTimer already running\r\n");
		return;
	}

	if ((g_DiagRepeatTimer  = adl_tmrSubscribe(TRUE, 60*10, ADL_TMR_TYPE_100MS, DConfig_Rpt_handler)) == NULL)
	{
		DumpMessage("Could not start diag transmit handler timer\r\n");
	}
}


void stopDiagMsgRepeatTimer()
{
	s32 sReturn = adl_tmrUnSubscribe(g_DiagRepeatTimer, DConfig_Rpt_handler, ADL_TMR_TYPE_100MS);

	if (sReturn < 0)
	{
		DisplayErrorCode("adl_tmrUnSubscribe - g_DiagRepeatTimer", __FILE__, __LINE__, sReturn);
	}
	else
	{
		DumpMessage("g_DiagRepeatTimer stopped\r\n");
		g_DiagRepeatTimer = NULL;
	}
}

void StartPowerOffTimer()	
{
	//30 sec timer
	//Default 30 sec timer
	if(g_AGPSData.PowerOffTimeout == 0 || g_AGPSData.PowerOffTimeout > 9)
	{
		g_AGPSData.PowerOffTimeout = 2;
		AGPSDataToFlash();
	}
	
	if ((adl_tmrSubscribe(FALSE, g_AGPSData.PowerOffTimeout*50, ADL_TMR_TYPE_100MS, PowerOffModem)) == NULL)
	{
		DumpMessage("Could not start transmit handler timer\r\n");
	}

}

/*@}*/

/*@}*/
