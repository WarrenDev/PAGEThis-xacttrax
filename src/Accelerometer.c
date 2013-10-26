/** @addtogroup Accel 
 *@{*/
/*H************************************************************************
*/
/*! \file    Accelerometer.c
*
*   \brief   Higher level acceleromter functions that interact with
*            the lower level i2c_accelerometer functions
*   \details 
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

#include "Accelerometer.h"
#include "i2c_accelerometer.h"
#include "ConfigSettings.h"
#include "PowerCtrl.h"
#include "GPSCtrl.h"
#include "status.h"
#include "diagnose.h"
#include "gpioTest.h"
#include "XactUtilities.h"
#include "InternalFlash.h"
#include "alarm_suppress.h"
#include "ota_at.h"

#define ACCEL_INT_PIN 1    // interrupt 1

#define EXTINT_TRACE_LEVEL 4 // TODO: maybe move this


#define MAX_IRQ_CLEAR_ATTEMPTS 10

s32 ExtIntHandle;
s32 g_LowExtIrqHandle;
s32 g_HighExtIrqHandle;
s32 cs_handle = -1;
s32 test_handle = -1;
s32 msgHandle = -1;

static s32 SimulateLockUp = 0;  // Used to simulate accelerometer lockup condition.


extern u8 GotBreadCrumbFix;

typedef enum {
  ACCEL_IDLE,
  ACCEL_THRESH,
  ACCEL_ACTIVE
} ACCEL_STATE;


static ACCEL_STATE accel_state = ACCEL_IDLE;
static adl_ctxID_e appContext;

typedef enum {
  POSITIVE_X,
  NEGATIVE_X,
  POSITIVE_Y,
  NEGATIVE_Y,
  POSITIVE_Z,
  NEGATIVE_Z
} ACCEL_ORIENTATION;

static adl_rtcTimeStamp_t  FirstIRQTimeStamp;

static void SetupMsgHandler (void);
static void AccelMessageHandler(u32 MsgID, adl_ctxID_e Source, u32 Length, void *Data);
static void display_orient(ACCEL_ORIENTATION accel_orient);
static void display_cab_data(ACCEL_CALIBRATION_DATA *cab_data);


static void ClearIRQTimer(u8 timerid, void *context);

static void AccelCalibrateTimer(u8 timerid, void *context);

static int abs_val(int input_val);

FAST_IDLE_REQ AccelFastIdleReq = FAST_IDLE_REQ_RELEASE;
TRACKING_MODE_REQ AccelTrackingReq = TRACKING_MODE_REQ_RELEASE;

TRACKING_MODE_REQ AccelBreadCrumbTrackingReq = TRACKING_MODE_REQ_RELEASE;

volatile int InteruptProc = 0;

void ExtIntSubscribe( void );
bool AccelInterrupt(adl_irqID_e Source,adl_irqNotificationLevel_e NotificationLevel,adl_irqEventData_t * Data);
bool AccelInterruptRead(adl_irqID_e Source, adl_irqNotificationLevel_e NotificationLevel,adl_irqEventData_t * Data ); 

void HandleInterrupt(u16 timerID, void *Context);

extern void SubscribeFSMTimer(void);


// ExtInt configuration: both edge detection without filter
adl_extintConfig_t extintConfig = { ADL_EXTINT_SENSITIVITY_RISING_EDGE
				    , ADL_EXTINT_FILTER_BYPASS_MODE 
				    ,0,0, NULL };

// sets the threshold in the accelerometer.
int SetAccelTriggerThreshold (UINT8 AccelThreshWake) {
  return naAccelerometerSetTriggerThreshold (AccelThreshWake);
}

/** @brief Set to meausure mode.
 */
int SetAccelMeasureMode(void) 
{
  return naAcclerometerSetMeasureMode();
}

/** @brief set to level detect mode.
 */
int SetAccelLevelMode(void)
{
  return naAcclerometerSetLevelMode();
}




/*F***************************************************************
*
*   NAME:    ClearIRQ
*/
/** @brief   Clear the Accelerometer interrupt.
*
* @return void
*//*
*
*F*/
static int clear_irq_cnt = 0;
static adl_tmr_t* clear_irq_tmr_t = NULL;

void ClearIRQ(void)
{
  // clear the interrupts in case the device was reset at a bad time.
  if (naAccelerometerClearTrigger() < 0) {
    TRACE((EXTINT_TRACE_LEVEL,"ReadAccelerometer: Could not clear the interrupt\n"));
  }
  
  if (clear_irq_tmr_t == NULL) {
    if (naAccelerometerCheckIRQ()) {
      clear_irq_cnt = 0;
      clear_irq_tmr_t = adl_tmrSubscribe(FALSE,1,ADL_TMR_TYPE_100MS,ClearIRQTimer);
      if (clear_irq_tmr_t == NULL) {
	wm_sprintf(g_traceBuf,"Error: Could not start Clear IRQ Timer 1");
	DumpMessage(g_traceBuf);
      }
    }
    else {
      SetAccelLevelMode();
    }
  }
}


static void ClearIRQTimer(u8 timerid, void * context)
{
  (void) timerid;
  (void) context;
  wm_sprintf(g_traceBuf,"Clear Count = %d\r\n",clear_irq_cnt);
  DumpMessage(g_traceBuf);

  if (clear_irq_cnt >= MAX_IRQ_CLEAR_ATTEMPTS)  {
    // I don't think we should hit this anymore now that the IRQ is disabled
    // until it is cleared.
  }

  if (naAccelerometerCheckIRQ()) {
    clear_irq_cnt++;
    clear_irq_tmr_t = adl_tmrSubscribe(FALSE,1,ADL_TMR_TYPE_100MS,ClearIRQTimer);
    if (clear_irq_tmr_t == NULL) {
      wm_sprintf(g_traceBuf,"Error: Could not start Clear IRQ Timer 2");
      DumpMessage(g_traceBuf);
    }
  } else {
    clear_irq_tmr_t = NULL;
    SetAccelLevelMode();
  }
  
  
}


/*F***************************************************************
*
*   NAME:    InitAccelerometer
*/
/** @brief   setup the accelerometer.
*
* @return void
*//*
*
* CHANGES :
* REF NO  DATE     WHO      DETAIL
*         15Jul09  AndyB    First version
* NNNNNN  DDMMMYY  Name     Name of function/item changed
*F*/
void InitAccelerometer (void) {
  ACCEL_CALIBRATION_DATA cab_data;

  // Subscribe to the external interrupt.
  ExtIntSubscribe();

  // Set the GPIO High for chip select signal.
  GpioWrite(ACCEL_CS_GPIO,&cs_handle,TRUE);

  // clear the interrupts in case the device was reset at a bad time.
  ClearIRQ();
#if 0
  if (naAccelerometerClearTrigger() < 0) {
    TRACE((EXTINT_TRACE_LEVEL,"ReadAccelerometer: Could not clear the interrupt\n"));
  }
#endif
  // communicate with acceleromter.
  naAccelerometerEpoch();

  // load calibtration and write it to accelerometer.
  LoadAccelCalibration(&cab_data);
  naWriteAccelData(&cab_data);

  // set to other mode.
  //  naAcceleromterSetCtrl2();

  // Set the accelerometer threshold.
  SetAccelTriggerThreshold(g_config.AccelThreshWake);

  // setup the message handler for the interrupt.
  SetupMsgHandler();

  DumpMessage("Activate Accel state machine on power up\r\n");
  //SimulateAccelTrigger();   
  ActivateAccelOnPowerUp();

  	

}

/*F***************************************************************
*
*   NAME:    AccelInterrupt
*/
/** @brief   handler for low level interrupt
*
* @param Source
* @param NotificationLevel
* @param Data 
* @return TRUE
*
*//*
*
* CHANGES :
* REF NO  DATE     WHO      DETAIL
*         15Jul09  AndyB    First version
* NNNNNN  DDMMMYY  Name     Name of function/item changed
*F*/
bool AccelInterrupt(adl_irqID_e Source
		    , adl_irqNotificationLevel_e NotificationLevel
		    , adl_irqEventData_t * Data )

{
  (void) Source;
  (void) NotificationLevel;
  (void) Data;

  // Read the input status
  adl_extintInfo_t Status, * AutoReadStatus;
  adl_extintRead ( ExtIntHandle, &Status );
	
  // Input status can also be obtained from the auto read option.
  AutoReadStatus = ( adl_extintInfo_t * ) Data->SourceData;
  TRACE((1,"Status: %x\n",Status.PinState));
  TRACE((1,"AutoReadStatus: %x\n",AutoReadStatus->PinState));

  return TRUE;
}


/*F***************************************************************
*
*   NAME:    ExtIntSubscribe
*/
/** @brief   Subscribe to external interrupt event
*
* @return void
*//*
*
* CHANGES :
* REF NO  DATE     WHO      DETAIL
*         15Jul09  AndyB    First version
* NNNNNN  DDMMMYY  Name     Name of function/item changed
*F*/
void ExtIntSubscribe( void )
{
  adl_extintCapabilities_t My_ExtInt_Cap;
  s32 sReturn=-1;

  sReturn = adl_extintGetCapabilities( &My_ExtInt_Cap);
  if(sReturn < 0)
    DisplayErrorCode("adl_extintGetCapabilities",__FILE__,__LINE__,sReturn);

  TRACE((EXTINT_TRACE_LEVEL, "******** NbExternalInterrupt = %d\n",My_ExtInt_Cap.NbExternalInterrupt)); 

#if 0
  wm_sprintf(g_traceBuf,"RisingEdgeSenstivity %d\r\nFallingEdgeSensitivity %d\r\nBothEdgeSensitivity %d\r\n LowLevelSensitivity %d\r\nHighLevelSensitivity %d\r\n supported filter :\r\n Bypass %d\r\n Stretching %d\r\nDebounce %d\r\nfilter options :\r\nMaxDebounceDuration %d ms in %d steps\r\n", 
                    My_ExtInt_Cap.NbExternalInterrupt , 
                    My_ExtInt_Cap.RisingEdgeSensitivity , 
                    My_ExtInt_Cap.FallingEdgeSensitivity , 
                    My_ExtInt_Cap.BothEdgeSensitivity , 
                    My_ExtInt_Cap.LowLevelSensitivity , 
                    My_ExtInt_Cap.HighLevelSensitivity , 
                    My_ExtInt_Cap.BypassMode , 
                    My_ExtInt_Cap.StretchingMode , 
                    My_ExtInt_Cap.DebounceMode , 
                    My_ExtInt_Cap.MaxDebounceDuration , 
                    My_ExtInt_Cap.DebounceNbStep  
	     );

  DumpMessage(g_traceBuf);
#endif

  /*---- Verify that there is an Ext Int Pin ----*/
  if(My_ExtInt_Cap.NbExternalInterrupt >=1)
    {
      g_LowExtIrqHandle = adl_irqSubscribe(AccelInterrupt,ADL_IRQ_NOTIFY_LOW_LEVEL
					   , ADL_IRQ_PRIORITY_HIGH_LEVEL, ADL_IRQ_OPTION_AUTO_READ);
      if(g_LowExtIrqHandle < 0)
	DisplayErrorCode("adl_irqSubscribe",__FILE__,__LINE__,g_LowExtIrqHandle);
      else
	TRACE((EXTINT_TRACE_LEVEL, "Successfully subscribed to the LL ExtInt Irq\n"));
		
      g_HighExtIrqHandle = adl_irqSubscribe(AccelInterruptRead,ADL_IRQ_NOTIFY_HIGH_LEVEL
						   ,ADL_IRQ_PRIORITY_LOW_LEVEL, ADL_IRQ_OPTION_AUTO_READ);
      if(g_HighExtIrqHandle < 0)
	DisplayErrorCode("adl_irqSubscribe",__FILE__,__LINE__,g_HighExtIrqHandle );
      else
	TRACE((EXTINT_TRACE_LEVEL,"Successfully subscribed to the HL ExtInt Irq\n"));

      /*---- Configure the comparator channel ----*/
      ExtIntHandle = adl_extintSubscribe( ACCEL_INT_PIN 
					  , g_LowExtIrqHandle, g_HighExtIrqHandle , &extintConfig);
      if(ExtIntHandle < 0)
	DisplayErrorCode("adl_extintSubscribe",__FILE__,__LINE__,ExtIntHandle);
      else
	TRACE((EXTINT_TRACE_LEVEL,"Successfully subscribed to the extint 2\n"));
    }
  else
    TRACE((EXTINT_TRACE_LEVEL,"NbExternalInterrupt < 1\n"));
}


/*F***************************************************************
*
*   NAME:    AccelInterruptRead
*/
/** @brief   handler for high level interrupt.
*
* @param Source 
* @param NotificationLevel
* @param Data 
* @return FALSE
*//*
*
* CHANGES :
* REF NO  DATE     WHO      DETAIL
*         15Jul09  AndyB    First version
* NNNNNN  DDMMMYY  Name     Name of function/item changed
*F*/
bool AccelInterruptRead(adl_irqID_e Source, adl_irqNotificationLevel_e NotificationLevel
			, adl_irqEventData_t * Data) 
{
  (void) Source;
  (void) NotificationLevel;
  (void) Data;

  if (SimulateLockUp == 0) 
  {
    adl_msgSend(appContext,0,0,NULL);
  }

  return FALSE;
}


/*F***************************************************************
*
*   NAME:    CheckAccelState
*/
/** @brief   keep track of how long accel is idle
*
* @param timerID
* @param Context
* @return void
*//*
*
* CHANGES :
* REF NO  DATE     WHO      DETAIL
*         15Jul09  AndyB    First version
* NNNNNN  DDMMMYY  Name     Name of function/item changed
*F*/
void CheckAccelState(u8 timerID, void * Context) {

  (void) timerID;
  (void) Context;

  adl_rtcTime_t CurrentTime;
  adl_rtcTimeStamp_t  DeltaTimeStamp;
  adl_rtcTimeStamp_t  CurrentTimeStamp;

  INDICATOR tmp_ind;
  unsigned int num_ms = 0;
  s32 sReturn = -1;
  
  // check for diagnose mode.
  if (diagnose_get_mode() == DIAGNOSE_ON) {
    
      if (accel_state == ACCEL_ACTIVE) {
	if ((sReturn = adl_rtcGetTime(&CurrentTime)) < 0) {
	  DisplayErrorCode("adl_rtcGetTime",__FILE__,__LINE__,sReturn);
	}
	// Get the current time.
	if ((sReturn = adl_rtcConvertTime( &CurrentTime, &CurrentTimeStamp, ADL_RTC_CONVERT_TO_TIMESTAMP)) < 0) {
	  DisplayErrorCode("adl_rtcConvertTime",__FILE__,__LINE__,sReturn);
	}

	// Calculate the time difference.
	if ((sReturn = adl_rtcDiffTime(&CurrentTimeStamp,&FirstIRQTimeStamp,&DeltaTimeStamp)) < 0) {
	  DisplayErrorCode("adl_rtcDiffTime",__FILE__,__LINE__,sReturn);
	}
	
	num_ms = DeltaTimeStamp.TimeStamp*1000 + (int)ADL_RTC_GET_TIMESTAMP_MS(DeltaTimeStamp);
	
	if (num_ms > 1000) {
	  tmp_ind = diagnose_get(AC);
	  if (tmp_ind != IND_OFF) diagnose_deactivate_indicator(tmp_ind,AC);
	  accel_state = ACCEL_IDLE;
	}
      }
      return;
  }

  if (accel_state == ACCEL_ACTIVE) {
    if ((sReturn = adl_rtcGetTime(&CurrentTime)) < 0) {
      DisplayErrorCode("adl_rtcGetTime",__FILE__,__LINE__,sReturn);
    }

    // Get the current time.
    if ((sReturn = adl_rtcConvertTime( &CurrentTime, &CurrentTimeStamp, ADL_RTC_CONVERT_TO_TIMESTAMP)) < 0) {
      DisplayErrorCode("adl_rtcConvertTime",__FILE__,__LINE__,sReturn);
    }

    // Calculate the time difference.
    if ((sReturn = adl_rtcDiffTime(&CurrentTimeStamp,&FirstIRQTimeStamp,&DeltaTimeStamp)) < 0) {
		wm_sprintf(g_traceBuf,"FirstIRQTimeStamp : 0x%x, CurrentTimeStamp: 0x%x\r\n",(unsigned int)FirstIRQTimeStamp.TimeStamp,(unsigned int)CurrentTimeStamp.TimeStamp);
		DumpMessage(g_traceBuf);
		DumpMessageUSB(g_traceBuf,1);
      	DisplayErrorCode("adl_rtcDiffTime",__FILE__,__LINE__,sReturn);
    }

    num_ms = DeltaTimeStamp.TimeStamp*1000 + (int)ADL_RTC_GET_TIMESTAMP_MS(DeltaTimeStamp);

	
	
    // Calculate the time difference.
    //if ((sReturn = adl_rtcDiffTime(&CurrentTimeStamp,&FirstIRQTimeStamp,&DeltaTimeStamp)) < 0) {
      //DisplayErrorCode("adl_rtcDiffTime",__FILE__,__LINE__,sReturn);
    //}

    //Accelerometer should never get released if its FULL_TRACK_MODE
    if(g_config.Mode != FULL_TRACK_MODE && g_config.Mode != ALARM_MODE)
    {
		// changed this from milliseconds to seconds by multiplying by 1000
        if (num_ms >= g_config.AccelDurationSleep*1000) {
          accel_state = ACCEL_IDLE;
          // Turn off accel triggered tracking mode.
          AccelFastIdleReq = FAST_IDLE_REQ_RELEASE;
    
          DumpMessage("Accel release\r\n!");
    
          AccelTrackingReq = TRACKING_MODE_REQ_RELEASE;
    
        }
    }

  }
}

/*F***************************************************************
*
*   NAME:    SetupMsgHandler
*/
/** @brief   setup the message handler to process accelerometer interrupts.
*
* @return void
*//*
*
* CHANGES :
* REF NO  DATE     WHO      DETAIL
*         15Jul09  AndyB    First version
* NNNNNN  DDMMMYY  Name     Name of function/item changed
*F*/
static void SetupMsgHandler (void) {
  adl_msgFilter_t Filter;

  Filter.MsgIdentifierMask = 0;
  Filter.MsgIdentifierValue = 0;
  Filter.Comparator = ADL_MSG_ID_COMP_GREATER;
  Filter.Source = ADL_CTX_HIGH_LEVEL_IRQ_HANDLER;
  appContext = adl_ctxGetID();

  msgHandle = adl_msgSubscribe(&Filter, AccelMessageHandler);
  if(msgHandle < 0 )
    {
      DisplayErrorCode("adl_msgSubscribe",__FILE__,__LINE__,msgHandle);
    }

}

//#define DISABLE_WAKE_DURATION

/*F***************************************************************
*
*   NAME:    AccelMessageHandler
*/
/** @brief   Process the accel interrupt message.
*
* @param MsgID
* @param Source
* @param Length
* @param Data
* @return void
*//*
*
* CHANGES :
* REF NO  DATE     WHO      DETAIL
*         15Jul09  AndyB    First version
* NNNNNN  DDMMMYY  Name     Name of function/item changed
*F*/
static void AccelMessageHandler(u32 MsgID, adl_ctxID_e Source, u32 Length, void *Data) {

  (void) MsgID;
  (void) Source;
  (void) Length;
  (void) Data;

  s32 sReturn = -1;
  adl_rtcTime_t CurrentTime;
#ifndef DISABLE_WAKE_DURATION
  adl_rtcTimeStamp_t  DeltaTimeStamp;
  adl_rtcTimeStamp_t  NextIRQTimeStamp;
  unsigned int num_ms;
#endif
  int accel_mag = 0;
  static int MotionAlarmTriggered;
  INDICATOR tmp_ind;


  SetAccelMeasureMode();

  //  check if we are in the diagnose mode.
  //  if we are, normal operation does not procede.
  if (diagnose_get_mode() == DIAGNOSE_ON) {

    // get the current RTC time.
    if ((sReturn = adl_rtcGetTime(&CurrentTime)) < 0) {
      DisplayErrorCode("adl_rtcGetTime",__FILE__,__LINE__,sReturn);
      return;
    }
    // Get first RTC time.
    if (adl_rtcConvertTime( &CurrentTime, &FirstIRQTimeStamp, ADL_RTC_CONVERT_TO_TIMESTAMP) < 0) {
      DisplayErrorCode("adl_rtcConvertTime",__FILE__,__LINE__,sReturn);
    }

    tmp_ind = diagnose_get(AC);
    if (tmp_ind != IND_OFF) diagnose_activate_indicator(tmp_ind,AC);

    accel_state = ACCEL_ACTIVE;

    // clear the interrupt in the acceleromter.
    ClearIRQ();
#if 0
    if (naAccelerometerClearTrigger() < 0) {
      TRACE((EXTINT_TRACE_LEVEL,"ReadAccelerometer: Could not clear the interrupt\n"));
    }

    if (naAccelerometerCheckIRQ ()) 
      {
	DumpMessage("Accel IRQ still high!\r\n");
      }
#endif
    return;
  }

  // get the current RTC time.
  if ((sReturn = adl_rtcGetTime(&CurrentTime)) < 0) {
    DisplayErrorCode("adl_rtcGetTime",__FILE__,__LINE__,sReturn);
    return;
  }

#ifndef DISABLE_WAKE_DURATION
  // process the accelerometer state.
  // record the RTC on the first interrupt.
  // verify we get another interrupt sometime after the threshold time.
  if (accel_state == ACCEL_IDLE) {
    // Get first RTC time.
    if (adl_rtcConvertTime( &CurrentTime, &FirstIRQTimeStamp, ADL_RTC_CONVERT_TO_TIMESTAMP) < 0) {
      DisplayErrorCode("adl_rtcConvertTime",__FILE__,__LINE__,sReturn);
    }
    accel_state = ACCEL_THRESH;

  }
  else if (accel_state == ACCEL_THRESH) {
    // Get the current IRQ time.
    if ((sReturn = adl_rtcConvertTime( &CurrentTime, &NextIRQTimeStamp, ADL_RTC_CONVERT_TO_TIMESTAMP)) < 0) {
      DisplayErrorCode("adl_rtcConvertTime",__FILE__,__LINE__,sReturn);
    }

    // Calculate the time difference.
    if (adl_rtcDiffTime(&NextIRQTimeStamp,&FirstIRQTimeStamp,&DeltaTimeStamp) < 0) {
      DisplayErrorCode("adl_rtcDiffTime",__FILE__,__LINE__,sReturn);
    }

    num_ms = DeltaTimeStamp.TimeStamp*1000 + (int)ADL_RTC_GET_TIMESTAMP_MS(DeltaTimeStamp);

    if (num_ms >= g_config.AccelDurationWake) {
      // Set the acceleration triggered tracking mode.
      DumpMessage("Accel active!\r\n");
      accel_state = ACCEL_ACTIVE;
      AccelFastIdleReq = FAST_IDLE_REQ_ACTIVE;

      AccelTrackingReq = TRACKING_MODE_REQ_ACTIVE;
    }
  }
  else {
    // in the active state, just record the RTC time.
    if (adl_rtcConvertTime( &CurrentTime, &FirstIRQTimeStamp, ADL_RTC_CONVERT_TO_TIMESTAMP) < 0) {
      DisplayErrorCode("adl_rtcConvertTime",__FILE__,__LINE__,sReturn);
    }
  }
#else
  if (accel_state == ACCEL_IDLE) {
    // Get first RTC time.
    if (adl_rtcConvertTime( &CurrentTime, &FirstIRQTimeStamp, ADL_RTC_CONVERT_TO_TIMESTAMP) < 0) {
      DisplayErrorCode("adl_rtcConvertTime",__FILE__,__LINE__,sReturn);
    }
    DumpMessage("Accel active!\r\n");
    accel_state = ACCEL_ACTIVE;
    AccelFastIdleReq = FAST_IDLE_REQ_ACTIVE;

    AccelTrackingReq = TRACKING_MODE_REQ_ACTIVE;
  }
  else {
    // in the active state, just record the RTC time.
    if (adl_rtcConvertTime( &CurrentTime, &FirstIRQTimeStamp, ADL_RTC_CONVERT_TO_TIMESTAMP) < 0) {
      DisplayErrorCode("adl_rtcConvertTime",__FILE__,__LINE__,sReturn);
    }
  }
#endif

  // clear the interrupt in the acceleromter.
  ClearIRQ();
#if 0
  if (naAccelerometerClearTrigger() < 0) {
    TRACE((EXTINT_TRACE_LEVEL,"ReadAccelerometer: Could not clear the interrupt\n"));
  }
  
  if (naAccelerometerCheckIRQ ()) 
  {
    DumpMessage("Accel IRQ still high!\r\n");
  }
#endif

  // check for the acceleration alarm.
  if ((accel_mag = ReadAccelerometer()) == -1) {
    DumpMessage("Read accel failed!\r\n");
    return;
  }

  if ( g_config.MotionAlarmThresh > 0) {
    int motion_alarm_thresh_mag = g_config.MotionAlarmThresh*g_config.MotionAlarmThresh;
    int fence_num;
    if ((accel_mag >=  motion_alarm_thresh_mag) && (alarm_suppress_status(ACC_ALARM_SUP)==ALARM_EXPIRED)) {
      if (g_config.MotionAlarmIndep) {
	MotionAlarmTriggered = 1;
	g_status.MotionAlarm = 'Y';
	alarm_suppress_set_alarm_time(ACC_ALARM_SUP);
	wm_sprintf(g_traceBuf,"Motion alarm generated. Threshold = %x, Mag = %x\r\n",motion_alarm_thresh_mag,accel_mag);
	DumpMessage(g_traceBuf);
      } else {
	MotionAlarmTriggered = 1;
	wm_sprintf(g_traceBuf,"Motion alarm generated -- Check fence! Threshold = %x, Mag = %x\r\n",motion_alarm_thresh_mag,accel_mag);
	DumpMessage(g_traceBuf);
	eval_fix(&fence_num,MOTION_ALARM_REASON);
	if (g_status.FenceAlarm == 'M') {
	  alarm_suppress_set_alarm_time(ACC_ALARM_SUP);
	}
      }

    }
    else if ((accel_mag < g_config.MotionAlarmThresh) && (alarm_suppress_status(ACC_ALARM_SUP)==ALARM_EXPIRED)) {
      MotionAlarmTriggered = 0;
    }
  }

}

void AccelReadData(void)
{
  ACCEL_CALIBRATION_DATA cab_data;
  naReadAccelData(&cab_data);
  display_cab_data(&cab_data);
}


static ACCEL_CALIBRATION_DATA cab_data;
static int iteration_cnt = 0;

void AccelCalibrate(void)
{

  cab_data.x_offset_value = 0;
  cab_data.y_offset_value = 0;
  cab_data.z_offset_value = 0;
  iteration_cnt = 0;

  if (adl_tmrSubscribe(FALSE,1,ADL_TMR_TYPE_100MS,AccelCalibrateTimer) == NULL)
  {
    wm_sprintf(g_traceBuf,"Error: Could not start AccelCalibrateTimer\r\n");
    DumpMessage(g_traceBuf);
  }
}

static void AccelCalibrateTimer(u8 timerid, void *context)
{
  (void) timerid;
  (void) context;

  ACCEL_ORIENTATION accel_orient;
  
  int small_pos=0,small_val=0x7fffffff;
  int ii;
  int abs_mags[3];

  int total_error = 100;

  // turn on measurement mode.
  SetAccelMeasureMode();

  // read in the x,y,z data.
  naReadAccelData(&cab_data);

  // we are dealing with 10 bit values of which 9 bits are significant.
  // 2^9 = 512 values over 8g's means that 1g is equal to 64. Find the 
  // x,y,z closest to 64. 
  abs_mags[0] = abs_val(cab_data.x_10_bit);
  abs_mags[1] = abs_val(cab_data.y_10_bit);
  abs_mags[2] = abs_val(cab_data.z_10_bit);

  for(ii=0;ii<3;ii++) {
    abs_mags[ii] = abs_val(64 - abs_mags[ii]);
    if (abs_mags[ii] < small_val) {
      small_pos = ii; 
      small_val = abs_mags[ii];
    }
  }

  switch (small_pos) 
    {
    case 0  : 
      if (cab_data.x_10_bit < 0) accel_orient = NEGATIVE_X;
      else                       accel_orient = POSITIVE_X;
      break;
    case 1  :
      if (cab_data.y_10_bit < 0) accel_orient = NEGATIVE_Y;
      else                       accel_orient = POSITIVE_Y;
      break;
    case 2  :
      if (cab_data.z_10_bit < 0) accel_orient = NEGATIVE_Z;
      else                       accel_orient = POSITIVE_Z;
      break;
    default :                    accel_orient = NEGATIVE_Z;
    }

  display_orient(accel_orient);

  switch (accel_orient) 
    {
    case POSITIVE_X : 
      cab_data.x_offset_value += 2*(64 - cab_data.x_10_bit);
      cab_data.y_offset_value += 2*(-cab_data.y_10_bit);
      cab_data.z_offset_value += 2*(-cab_data.z_10_bit);
      break;
    case NEGATIVE_X :
      cab_data.x_offset_value += 2*((-64) - cab_data.x_10_bit);
      cab_data.y_offset_value += 2*(-cab_data.y_10_bit);
      cab_data.z_offset_value += 2*(-cab_data.z_10_bit);
      break;
    case POSITIVE_Y :
      cab_data.x_offset_value += 2*(-cab_data.x_10_bit);
      cab_data.y_offset_value += 2*(64 - cab_data.y_10_bit);
      cab_data.z_offset_value += 2*(-cab_data.z_10_bit);
      break;
    case NEGATIVE_Y :
      cab_data.x_offset_value += 2*(-cab_data.x_10_bit);
      cab_data.y_offset_value += 2*((-64) - cab_data.y_10_bit);
      cab_data.z_offset_value += 2*(-cab_data.z_10_bit);
      break;
    case POSITIVE_Z :
      cab_data.x_offset_value += 2*(-cab_data.x_10_bit);
      cab_data.y_offset_value += 2*(-cab_data.y_10_bit);
      cab_data.z_offset_value += 2*(64 - cab_data.z_10_bit);
      break;
    case NEGATIVE_Z :
      cab_data.x_offset_value += 2*(-cab_data.x_10_bit);
      cab_data.y_offset_value += 2*(-cab_data.y_10_bit);
      cab_data.z_offset_value += 2*((-64) - cab_data.z_10_bit);
      break;
    }

  display_cab_data(&cab_data);
  naWriteAccelData(&cab_data);
  naReadAccelData(&cab_data);

  total_error = 0;
  switch (accel_orient) 
    {
    case POSITIVE_X :
      total_error += abs_val(64 - cab_data.x_10_bit);
      total_error += abs_val(cab_data.y_10_bit);
      total_error += abs_val(cab_data.z_10_bit);
      break;
    case NEGATIVE_X :
      total_error += abs_val(-64 - cab_data.x_10_bit);
      total_error += abs_val(cab_data.y_10_bit);
      total_error += abs_val(cab_data.z_10_bit);
      break;
    case POSITIVE_Y :
      total_error += abs_val(cab_data.x_10_bit);
      total_error += abs_val(64 - cab_data.y_10_bit);
      total_error += abs_val(cab_data.z_10_bit);
      break;
    case NEGATIVE_Y :
      total_error += abs_val(cab_data.x_10_bit);
      total_error += abs_val(-64 - cab_data.y_10_bit);
      total_error += abs_val(cab_data.z_10_bit);
      break;
    case POSITIVE_Z : 
      total_error += abs_val(cab_data.x_10_bit);
      total_error += abs_val(cab_data.y_10_bit);
      total_error += abs_val(64 - cab_data.z_10_bit);
      break;
    case NEGATIVE_Z :
      total_error += abs_val(cab_data.x_10_bit);
      total_error += abs_val(cab_data.y_10_bit);
      total_error += abs_val(-64 - cab_data.z_10_bit);
      break;
    }
  wm_sprintf(g_traceBuf,"****\r\ntotal error = %d\r\niteration=%d\r\n*****\r\n",total_error,iteration_cnt);
  DumpMessage(g_traceBuf);

  if (total_error != 0 && iteration_cnt++ < 20) {
    if (adl_tmrSubscribe(FALSE,1,ADL_TMR_TYPE_100MS,AccelCalibrateTimer) == NULL) {
	wm_sprintf(g_traceBuf,"Error: Could not start AccelCalibrateTimer\r\n");
	DumpMessage(g_traceBuf);
    }
  }
  else {
    WriteAccelCalibrationFlash(&cab_data);
  }

  // turn back on level mode.
  SetAccelLevelMode();
}

static int abs_val(int input_val)
{
  if (input_val >= 0) return input_val;
  else                return input_val*-1;
}

static void display_orient(ACCEL_ORIENTATION accel_orient)
{
  switch (accel_orient) {
  case  POSITIVE_X :
    wm_sprintf(g_traceBuf,"POSITIVE_X\r\n");
    DumpMessage(g_traceBuf);
    break;
  case  NEGATIVE_X :
    wm_sprintf(g_traceBuf,"NEGATIVE_X\r\n");
    DumpMessage(g_traceBuf);
    break;
  case  POSITIVE_Y :
    wm_sprintf(g_traceBuf,"POSITIVE_Y\r\n");
    DumpMessage(g_traceBuf);
    break;
  case  NEGATIVE_Y :
    wm_sprintf(g_traceBuf,"NEGATIVE_Y\r\n");
    DumpMessage(g_traceBuf);
    break;
  case  POSITIVE_Z :
    wm_sprintf(g_traceBuf,"POSITIVE_Z\r\n");
    DumpMessage(g_traceBuf);
    break;
  case  NEGATIVE_Z :
    wm_sprintf(g_traceBuf,"NEGATIVE_Z\r\n");
    DumpMessage(g_traceBuf);
    break;
  }
}

static bool enable_ota57 = false;
void ConfigSettings_setOTA57(bool ota57)
{
  enable_ota57 = ota57;
}

static void display_cab_data(ACCEL_CALIBRATION_DATA *cab_data)
{
    wm_sprintf(g_traceBuf,"x 10 bit: %d\r\ny 10 bit: %d\r\nz 10 bit: %d\r\nx 8 bit: %d\r\ny 8 bit: %d\r\nz 8 bit: %d\r\n",
	     cab_data->x_10_bit,
	     cab_data->y_10_bit,
	     cab_data->z_10_bit,
	     cab_data->x_8_bit,
	     cab_data->y_8_bit,
	     cab_data->z_8_bit);
    DumpMessage(g_traceBuf);
    DumpMessageUSB(g_traceBuf,1);
    if (enable_ota57) OTAAT_handle_at_cmd_response(g_traceBuf, false, strlen(g_traceBuf));

    wm_sprintf(g_traceBuf,"x calib: %d\r\ny calib: %d\r\nz calib: %d\r\n",
	       cab_data->x_offset_value,
	       cab_data->y_offset_value,
	       cab_data->z_offset_value);
    DumpMessage(g_traceBuf);
    if (enable_ota57) OTAAT_handle_at_cmd_response(g_traceBuf, true, strlen(g_traceBuf));
    ConfigSettings_setOTA57(false);

}

ACCEL_ERROR_STATE accel_error_state = {0,0};

void CheckAccelSlowIdle(void)
{
  unsigned char threshold = naAccelReadThreshold();

  if (threshold != g_config.AccelThreshWake) {
    SetAccelTriggerThreshold(g_config.AccelThreshWake);
    SetAccelLevelMode();
    accel_error_state.threshold_bad++;
    // reboot if we can't fix this.
    if (accel_error_state.threshold_bad > 1) {
      Status_setResetReason(RESET_REASON_ACCEL);
      start_reset_timer();
    }
  }

  if (naAccelerometerCheckIRQ()) {
    ClearIRQ();
    accel_error_state.irq_high_slow_idle++;
    // reboot if we can't fix this.
    if (accel_error_state.irq_high_slow_idle > 1) {
      Status_setResetReason(RESET_REASON_ACCEL);
      start_reset_timer();
    }
    SimulateLockUp = 0;
  }
}

void AccelSimulateLockUp(void)
{
  if (SimulateLockUp) SimulateLockUp = 0;
  else SimulateLockUp = 1;
}


//Sandeep Added test code

void ActivateAccelOnPowerUp()
{
	s32 sReturn = -1;
	adl_rtcTime_t CurrentTime;	
	
	DumpMessage("ActivateAccelOnPowerUp Entry\r\n");
	accel_state = ACCEL_ACTIVE;
	AccelFastIdleReq = FAST_IDLE_REQ_ACTIVE;
	AccelTrackingReq = TRACKING_MODE_REQ_ACTIVE;
	// get the current RTC time.
	if ((sReturn = adl_rtcGetTime(&CurrentTime)) < 0) {
    DisplayErrorCode("adl_rtcGetTime",__FILE__,__LINE__,sReturn);
    return;
  }
	if (adl_rtcConvertTime( &CurrentTime, &FirstIRQTimeStamp, ADL_RTC_CONVERT_TO_TIMESTAMP) < 0) {
      DisplayErrorCode("adl_rtcConvertTime",__FILE__,__LINE__,sReturn);
    }
	wm_sprintf(g_traceBuf,"FirstIRQTimeStamp : 0x%x\r\n",(unsigned int)FirstIRQTimeStamp.TimeStamp);
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf,1);

	DumpMessage("ActivateAccelOnPowerUp Exit\r\n");	
}

void SimulateAccelTrigger()
{
/*
	s32 sReturn = -1;
	adl_rtcTime_t CurrentTime;	

	DumpMessage("SimulateAccelTrigger Entry\r\n");
	accel_state = ACCEL_ACTIVE;
	AccelFastIdleReq = FAST_IDLE_REQ_ACTIVE;
	AccelTrackingReq = TRACKING_MODE_REQ_ACTIVE;

	// get the current RTC time.
	if ((sReturn = adl_rtcGetTime(&CurrentTime)) < 0) {
    DisplayErrorCode("adl_rtcGetTime",__FILE__,__LINE__,sReturn);
    return;
  }
	if (adl_rtcConvertTime( &CurrentTime, &FirstIRQTimeStamp, ADL_RTC_CONVERT_TO_TIMESTAMP) < 0) {
      DisplayErrorCode("adl_rtcConvertTime",__FILE__,__LINE__,sReturn);
    }*/
    
	DumpMessage("SimulateAccelTrigger Entry\r\n");

	ActivateAccelOnPowerUp();
    SubscribeFSMTimer();

    DumpMessage("SimulateAccelTrigger Exit\r\n");

}

/*@}*/
