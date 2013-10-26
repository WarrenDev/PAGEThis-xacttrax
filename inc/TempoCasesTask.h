//
// TempoCasesTask.h
//
#ifndef _TEMPO_CASES_TASK_H_
#define _TEMPO_CASES_TASK_H_

#define TC_UART			ADL_PORT_UART1
#define TC_DEBUG_UART	ADL_PORT_USB

#define PARAM_STR_LEN  (256)

#define MAX_VOLTAGE         (14.68)
#define MIN_VOLTAGE         (9.9)


extern ascii g_alert[PARAM_STR_LEN];
extern unsigned int g_adCount;
extern unsigned char g_VoltagePct;

typedef enum _TC_ATMEL_STATE {
	ATMEL_READ_STATUS = 0,
	ATMEL_STATUS_PENDING,
	ATMEL_READ_VOLTAGE,
	ATMEL_VOLTAGE_PENDING
} TC_ATMEL_STATE;

typedef enum _ALERT_BIT_ID
{
    SENSOR_1_ERROR,
    SENSOR_2_ERROR,
    SENSOR_DIFF_ERROR,
    LOW_BATTERY,
    CURRENT_LIMITING,
    FAN_1_INTERIOR_ERROR,
    SENSOR_3_ERROR,
    SINK_OVER_TEMP,
    ROOM_HIGH,
    FAN_2_EXTERIOR_ERROR, 
    HUMIDITY_OUT_OF_RANGE,
    BOARD_SENSOR_OVERTEMP,
    AUTOSAFE_SHUTOFF_ALERT,
    TEMP_SYSTEM_FAIL,
    IN_STANDBY,
    // Do not write below this comment
    N_OF_ALERT_BIT_IDS 
}
ALERT_BIT_ID;

typedef struct _ALERT_BITS_MINUTES
{
    ALERT_BIT_ID bit_id;
    unsigned int bit;
    unsigned int timeout;  // max timout is 4hrs = 240 min
}
ALERT_BITS_MINUTES;

void UnsubscribeTempoTaskTimer ( void );

void DumpMessageTC( char *Msg );
void TempoCasesTask( void );

bool TC_CtrlHandler(u8 event);
u8 TC_DataHandler(u16 datalength, u8 *data);





#endif
