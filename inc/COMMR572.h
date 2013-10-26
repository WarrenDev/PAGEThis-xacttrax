//
// COMMR572
//
#ifndef _COMMR572_H_
#define _COMMR572_H_

#include "adl_global.h"
#include "pistd.h"


#define NA 0xFF

typedef enum _COMMR572_CMND_ID
{
    MODEL_CODE,
    TEMPERATURE_INPUT_1,
    TEMPERATURE_INPUT_2,
    ALARM_STATUS,
    TEMPERATURE_INPUT_3,
    HUMIDITY_SENSOR,
    HEAT_START_TEMP,
    HEAT_END_TEMP,
    COOLING_START_TEMP,
    COOLING_END_TEMP, 
    INPUT_VOLTAGE_COUNTS,
    BATTERY_LEVEL_PERCENT,
    // Do not write below this comment
    N_OF_COMMR572_CMND_IDS 
}COMMR572_CMND_ID;

typedef struct _COMMR572_CMND
{
    COMMR572_CMND_ID cmnd_id;
    int write_command;
    int read_command;
}COMMR572_CMND;

extern char STX;
extern char ETX;
extern char ACK;

unsigned char check_sum( char *msg );
bool make_write_command(COMMR572_CMND_ID cmnd_id, int parameter, char *cmnd_buffer );
bool make_read_command(COMMR572_CMND_ID cmnd_id, int parameter, char *cmnd_buffer );


#endif  //_COMMR572_H_
