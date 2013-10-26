//
// TempoCasesTask.c
//

#include "adl_global.h"
#include "TempoCasesTask.h"
#include "XactUtilities.h"
#include "COMMR572.h"
#include "status.h"
#include "SMShandling.h"
#include "alarm_suppress.h"
#include "gpioTest.h"

// Tick is 18.4 ms
#define SEC_2_TICKS( sec )		((int)((1000.0*sec)/18.4))
#define SLEEP_TIME_SEC			(3)
#define BATT_ALARM_THRESHOLD1	0x01					// 1%
#define BATT_ALARM_THRESHOLD2	0x02					// 2%
#define BATT_ALARM_THRESHOLD3	0x03					// 3%
#define BATT_ALARM_THRESHOLD5	0x05					// 5%
#define BATT_ALARM_THRESHOLD10	0x0A					// 10%
#define BATT_ALARM_THRESHOLD20	0x14					// 20%

#define	BATTALARM_THRESH20		0x01					//  Battery alarm 20% alert sent
#define	BATTALARM_THRESH10		0x02					//  Battery alarm 10% alert sent
#define	BATTALARM_THRESH5		0x04					//  Battery alarm 5% alert sent
#define	BATTALARM_THRESH3		0x08					//  Battery alarm 3% alert sent
#define	BATTALARM_THRESH2		0x10					//  Battery alarm 2% alert sent
#define	BATTALARM_THRESH1		0x20					//  Battery alarm 1% alert sent

extern STATUS g_status;

volatile u8 g_battalarm_status = 0;
u8   g_fcm_Handle;
bool g_uart_ready = false;
bool g_atmel_ready = false;
volatile int g_atmel_schedule = 0;

ascii g_alert[PARAM_STR_LEN];
unsigned int g_adCount;
unsigned char g_VoltagePct;
adl_tmr_t *pTempoCasesTmrPtr;

bool g_new_alert = false;

static void SubscribeTempoTaskTimer(void);

TC_ATMEL_STATE g_atmelState = ATMEL_READ_STATUS;

ALERT_BITS_MINUTES alert_bits_minutes[N_OF_ALERT_BIT_IDS] =
{
    // bit_id                       bit     timeout
    {SENSOR_1_ERROR,              0x0001,   10},
    {SENSOR_2_ERROR,              0x0002,   10},
    {SENSOR_DIFF_ERROR,           0x0004,   10},
    {LOW_BATTERY,                 0x0008,   0xFFFF},	// Don't report - handled differently
    {CURRENT_LIMITING,            0x0010,   10},
    {FAN_1_INTERIOR_ERROR,        0x0020,   20},
    {SENSOR_3_ERROR,              0x0040,   10},
    {SINK_OVER_TEMP,              0x0080,   10},
    {ROOM_HIGH,                   0x0100,    3},
    {FAN_2_EXTERIOR_ERROR,        0x0200,   20},
    {HUMIDITY_OUT_OF_RANGE,       0x0400,   240},
    {BOARD_SENSOR_OVERTEMP,       0x0800,  0xFFFF},   // not reported 
    {AUTOSAFE_SHUTOFF_ALERT,      0x1000,   20},
    {TEMP_SYSTEM_FAIL,            0x2000,    5},
    {IN_STANDBY,                  0x4000,  0xFFFF}    // not reported
};

unsigned int TC_packet_scheduler( void )
{
    unsigned int ret_val = 0;
    unsigned int min_timeout;
    int alert;
    int idx;

    static unsigned int sec_counter = 0;

    sec_counter += SLEEP_TIME_SEC;
    // convert alert data to integer
    alert = wm_hexatoi(g_alert, 4);
    if ( alert != 0x0000  )
    {
        // set min_timeout to max value - any alert bit will lower min_timeout
        min_timeout = 0xFFFF;
        for ( idx = 0; idx < N_OF_ALERT_BIT_IDS; idx++   )
        {
            if ( (alert_bits_minutes[idx].bit & alert) != 0  )
            {
                if ( alert_bits_minutes[idx].timeout < min_timeout )
                {
                    min_timeout = alert_bits_minutes[idx].timeout;
                }
            }
        }

        if( min_timeout != 0xFFFF )
        {
            if ( (sec_counter >= (min_timeout * 60)) || (g_new_alert == true) )
            //if ( (sec_counter >= (min_timeout * 6U)) || (g_new_alert == true) )
            {
                ret_val = 1;
                sec_counter = 0;
                g_new_alert = false;
            }
        }
    }
    return ret_val;
}

void DumpMessageTC( char *Msg )
{
    adl_atSendResponsePort(ADL_AT_RSP, TC_UART, Msg);
    adl_atSendResponsePort(ADL_AT_RSP, TC_DEBUG_UART, Msg);
}

/** @brief Tempo Cases Task timer callback method
 *
 * @par This routine performs the functions of the tempo cases task.
 * @param timerID
 * @param Context
 * @return void
 */
 static void TempoCasesTaskTimer(u8 timerID, void *Context)
{
	(void)timerID;
	(void)Context;

    ascii str[32];

	if ( g_uart_ready == true ) 
    {
        if ( g_atmel_ready == FALSE)
        {
            make_read_command(MODEL_CODE, 0, str);
        }
        else
        {
            switch (g_atmelState)
            {
                case ATMEL_READ_STATUS:
                    make_read_command(ALARM_STATUS, 0, str);
                    g_atmelState = ATMEL_STATUS_PENDING;
                    break;
                    
                case ATMEL_READ_VOLTAGE:
                    make_read_command(BATTERY_LEVEL_PERCENT, 0, str);
                    g_atmelState = ATMEL_VOLTAGE_PENDING;
                    break;
                    
                default:
                    // requires timeout counter
                    break;
            }
            // If last scheduled transmission is cleared run scheduler.
            // If not, do nothing
            if ( g_atmel_schedule == 0 )
            {
                g_atmel_schedule = TC_packet_scheduler();
            }
        }
        adl_fcmSendData(g_fcm_Handle, (unsigned char *)str, strlen(str));
    }

}

/** @brief Subscribe to Tempo Cases Task timer
 *
 * @return void
 */
static void SubscribeTempoTaskTimer(void)
{
	if ((pTempoCasesTmrPtr = adl_tmrSubscribe(TRUE, (SLEEP_TIME_SEC * 10), ADL_TMR_TYPE_100MS, TempoCasesTaskTimer)) == NULL)
	{
		DisplayErrorCode("adl_tmrSubscribe - TempoCasesTaskTimer", __FILE__, __LINE__, (s32)(pTempoCasesTmrPtr));
	}
}

void UnsubscribeTempoTaskTimer ( void )
{

	adl_tmrUnSubscribe ( pTempoCasesTmrPtr, TempoCasesTaskTimer, ADL_TMR_TYPE_100MS );
	adl_fcmUnsubscribe ( g_fcm_Handle );

	DumpMessageUSB("AEW debug - TempoCasesTask stopped\r\n", 1);
}

/** @brief Initialize the Tempo Cases Task
 *
 * @return void
 */
void TempoCasesTask( void )
{

    // Init FCM
    g_fcm_Handle = adl_fcmSubscribe((adl_fcmFlow_e)TC_UART,
                                    (adl_fcmCtrlHdlr_f)TC_CtrlHandler,
                                    (adl_fcmDataHdlr_f)TC_DataHandler);

	SubscribeTempoTaskTimer ();

	DumpMessageUSB("AEW debug - TempoCasesTask started\r\n", 1);

}

//
//
//
bool TC_CtrlHandler(u8 event)
{
    u8 sReturn = 0;
    bool bReturn = TRUE;

    switch(event) {
        case ADL_FCM_EVENT_FLOW_OPENNED:
            DumpMessageTC("TCT - ADL_FCM_EVENT_FLOW_OPENNED.\r\n");
            sReturn = adl_fcmSwitchV24State(g_fcm_Handle, ADL_FCM_V24_STATE_DATA);
            g_uart_ready = true;
            break;

        case ADL_FCM_EVENT_FLOW_CLOSED:
            DumpMessageTC("TCT - ADL_FCM_EVENT_FLOW_CLOSED.\r\n");
            break;

        case ADL_FCM_EVENT_V24_DATA_MODE:
            g_uart_ready = true;
            DumpMessageTC("TCT - ADL_FCM_EVENT_V24_DATA_MODE.\r\n");
            break;

        case ADL_FCM_EVENT_V24_DATA_MODE_EXT:
            DumpMessageTC("TCT - ADL_FCM_EVENT_V24_DATA_MODE_EXT.\r\n");
			break;
        case ADL_FCM_EVENT_V24_AT_MODE:
			DumpMessageTC("TCT - *** m10DataCtrlHandler ADL_FCM_EVENT_V24_AT_MODE ***\r\n");
			DumpMessageTC("TCT - *** Serial Port Re-init ***\r\n");
			break;
        case ADL_FCM_EVENT_V24_AT_MODE_EXT:
        case ADL_FCM_EVENT_RESUME:          
        case ADL_FCM_EVENT_MEM_RELEASE:    
        case ADL_FCM_EVENT_V24_DATA_MODE_FROM_CALL:
        case ADL_FCM_EVENT_V24_AT_MODE_FROM_CALL:
            DumpMessageTC("TCT - ADL_FCM_EVENTS.\r\n");
            break;
    }

    return bReturn;
}

u8 TC_DataHandler(u16 datalength, u8 *data)
{
    ascii str[PARAM_STR_LEN] = "";
    static ascii prev_str[PARAM_STR_LEN] = "0000";
    int i;
	char sDebug[40];

//    static unsigned char prev_VoltagePct = 0;

    char *stx_ptr;
    char *ack_ptr;

//    float fVoltage;	AEW

// Debugging with a single RS232 port
//    sprintf(str, "ECHO %s %d\r\n", data, datalength);
//    adl_fcmSendData(g_fcm_Handle, (unsigned char *)str, strlen(str));
//    adl_fcmSendData(g_fcm_Handle, data, datalength);

    memset((void *)str, 0, PARAM_STR_LEN ); 

    if (g_atmel_ready == false)
    {
        // check model number until reported
        strncpy(str, (char *)data, datalength);
        if (strstr(str, "*1658d4^") != NULLPTR)
        {
            g_atmel_ready = TRUE;
        }
    }
    else
    {
        // check for STX and ACK ?? model string is the same
        strncpy(str, (char *)data, datalength);

        switch (g_atmelState)
        {
            case ATMEL_STATUS_PENDING:
                if ( (stx_ptr = strchr(str, STX)) && (ack_ptr = strchr(str, ACK)) && (stx_ptr < ack_ptr) )
                {
                    // extract alert data, hide stx, CS and ack
                    for ( i = 0; i < ack_ptr - stx_ptr - 1 - 2; i++)
                    {
                        str[i] =  str[i+(stx_ptr - str)+1];
                    }
                    str[i] = '\0';

                    // RS232 output for debugging
//                    adl_fcmSendData(g_fcm_Handle, (u8 *)str, strlen(str));
					sprintf (sDebug, "AEW debug - Alarm [%s]\r\n", str);
					DumpMessageTC ( sDebug );

                    // Check if any alert bit is set
                    g_new_alert = false;
                    if ( strcmp(str, prev_str) != 0 )
                    {
                        strcpy(prev_str, str);
                        g_new_alert = true;
                    }
                    strcpy( g_alert, str );
                    g_atmelState = ATMEL_READ_VOLTAGE;
                }
                break;
                
            case ATMEL_VOLTAGE_PENDING:
                if ( (stx_ptr = strchr(str, STX)) && (ack_ptr = strchr(str, ACK)) && (stx_ptr < ack_ptr) )
                {
                    // extract voltage ADC counts data, hide stx, CS and ack
                    for ( i = 0; i < ack_ptr - stx_ptr - 1 - 2; i++)
                    {
                        str[i] =  str[i+(stx_ptr - str)+1];
                    }
                    str[i] = '\0';

                    // RS232 output for debugging
//                    adl_fcmSendData(g_fcm_Handle, (u8 *)str, strlen(str));
					sprintf (sDebug, "AEW debug - Battery [%s]\r\n", str);
					DumpMessageTC ( sDebug );

                    // Convert to int
                    sscanf(str, "%x", &g_adCount);

// AEW no longer needed
//                    fVoltage = ((float)g_adCount * 5.0 * 7110.0) / 2046000.0;
                    // Calculate % of charge using voltage
//                    g_VoltagePct = ((fVoltage - MIN_VOLTAGE) * 100) / (MAX_VOLTAGE - MIN_VOLTAGE);

					g_VoltagePct = (unsigned char)g_adCount;

					////////////////////////////////////////
					// Reset battery alarm alert status bits
					if ( IsUSBConnected() || (g_VoltagePct > BATT_ALARM_THRESHOLD20) ) {

						g_battalarm_status = 0;
						g_status.BattAlarm = 'N';

					} // if

					if ( g_VoltagePct <= BATT_ALARM_THRESHOLD20 && !(g_battalarm_status & BATTALARM_THRESH20) )
					{
//						g_status.BattAlarm = 'Y';
				        sms_send('A');
						g_battalarm_status |= BATTALARM_THRESH20;
					}

					if ( g_VoltagePct <= BATT_ALARM_THRESHOLD10 && !(g_battalarm_status & BATTALARM_THRESH10) )
					{
//						g_status.BattAlarm = 'Y';
				        sms_send('A');
						g_battalarm_status |= BATTALARM_THRESH10;
					}

					if ( g_VoltagePct <= BATT_ALARM_THRESHOLD5 && !(g_battalarm_status & BATTALARM_THRESH5) )
					{
//						g_status.BattAlarm = 'Y';
				        sms_send('A');
						g_battalarm_status |= BATTALARM_THRESH5;
					}

//					if ( g_VoltagePct <= BATT_ALARM_THRESHOLD3 && !(g_battalarm_status & BATTALARM_THRESH3) )
//					{
//						g_status.BattAlarm = 'Y';
//				        sms_send('A');
//						g_battalarm_status |= BATTALARM_THRESH3;
//					}

//					if ( g_VoltagePct <= BATT_ALARM_THRESHOLD2 && !(g_battalarm_status & BATTALARM_THRESH2) )
//					{
//						g_status.BattAlarm = 'Y';
//				        sms_send('A');
//						g_battalarm_status |= BATTALARM_THRESH2;
//					}

					if ( g_VoltagePct <= BATT_ALARM_THRESHOLD1 && !(g_battalarm_status & BATTALARM_THRESH1) )
					{
//						g_status.BattAlarm = 'Y';
				        sms_send('A');
						g_battalarm_status |= BATTALARM_THRESH1;
					}

					////////////////////////////////////////
                    // force transmission if change is more than 5%
//                    if ( abs(g_VoltagePct - prev_VoltagePct) > 5 )
//                    {
//                        g_new_alert = true;
//                        prev_VoltagePct = g_VoltagePct;
//                    }

					////////////////////////////////////////
					// Handle battery alarm here - not needed (AEW Jr.)
				    // set the alarm in the status structure if needed.
//					{
//						static int AlarmCleared    = 0;
//
//					    if ((g_VoltagePct <= BATT_ALARM_THRESHOLD20) && (AlarmCleared))
//						{
//					      g_status.BattAlarm = 'Y';
//					      AlarmCleared       = 0;
//					      alarm_suppress_set_alarm_time(BAT_ALARM_SUP);
//					    }
//					    else if ( (g_VoltagePct > BATT_ALARM_THRESHOLD20) && (alarm_suppress_status(BAT_ALARM_SUP)==ALARM_EXPIRED) )
//					    {
//					      g_status.BattAlarm = 'N';
//					      AlarmCleared		 = 1;
//					    }
//
//					}

                    g_atmelState = ATMEL_READ_STATUS;
                }
                break;
            default:
            {
                break;
            }
        }
    }

    return TRUE;
}

