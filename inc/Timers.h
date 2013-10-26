/******************************************************************************
 *  File       : Timers.h
 ***-----------------------------------------------------------------------------
 *	General description of methods to be shared by this header file
 * - startGPSFixTimer
 * - stopGPSFixTimer
 * - startSendSMSTimer
 * - stopSendSMSTimer
 * - startBatteryCheckTimer
 * - stopBatteryCheckTimer
 * - startTempCheckTimer
 * - stopTempCheckTimer
 * - startOutsideFenceTimer
 * - stopOutsideFenceTimer
 ***-----------------------------------------------------------------------------
 *	General description of Defines to be shared by this header file
 * - DEFINE_NAME_1
 * - DEFINE_NAME_2
 ***-----------------------------------------------------------------------------
 *	any other info that would be useful
 * - bla bla bla
 *****************************************************************************/
#ifndef __H_TIMERS_H__
#define __H_TIMERS_H__

/******************************************************************************
 *
 * global variables and definitions to be shared amoung all files
 *
 *****************************************************************************/

extern u32 adcValue;
extern s32 adcmvValue;

/******************************************************************************
* prototypes
******************************************************************************/
//GPS Fix Timer Functions
void startNextFixTimer(void);
void stopNextFixTimer(void);

void NextFixHandler(u8 timerid, void *context);

void BatteryCheckTimerHandler(u8 timerid, void *context);

//Outside of Fence Timer Functions
void startOutsideFenceTimer(void);
void stopOutsideFenceTimer(void);
void OutsideFenceTimerHandler(u8 timerid, void *context);

void startTimers(void);
void stopTimers(void);
void singleTxHandlerTimer(void);
void startBatteryCheckTimer(void);
void stopTimersFWUpdate(void);
void StartGPSAvailabilityTimer(void);
void StopGPSAvailabilityTimer();
void StartFTPTimer();
void StartIPktTxTimer();
void startDiagMsgHandlerTimer();
void stopDiagMsgHandlerTimer();
void startDiagMsgRepeatTimer();
void stopDiagMsgRepeatTimer();
void StartPowerOffTimer();



#endif
