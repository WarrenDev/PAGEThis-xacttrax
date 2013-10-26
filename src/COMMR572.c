//
// COMMR572
//

#include "COMMR572.h"

char STX = 0x2A;
char ETX = 0x0D;
char ACK = 0x5E;


COMMR572_CMND commr572_cmnd[] =
{
    // ID                   write   read
    {MODEL_CODE,            NA,     0x00},
    {TEMPERATURE_INPUT_1,   NA,     0x01},
    {TEMPERATURE_INPUT_2,   NA,     0x04},
    {ALARM_STATUS,          NA,     0x03},
    {TEMPERATURE_INPUT_3,   NA,     0x0A},
    {HUMIDITY_SENSOR,       NA,     0x0B},
    {HEAT_START_TEMP,       0x1C,   0x50},
    {HEAT_END_TEMP,         0x1D,   0x51},
    {COOLING_START_TEMP,    0x1E,   0x52},
    {COOLING_END_TEMP,      0x1F,   0x53},
    {INPUT_VOLTAGE_COUNTS,  NA,     0xE0},
    {BATTERY_LEVEL_PERCENT, NA,		0x0D}
};


unsigned char check_sum( char *msg )
{
    unsigned short i;
    unsigned char cs = 0;
    for (i = 0; i < strlen(msg); i++) 
    {
        cs += msg[i];
    }
    return cs;
}

//
// NOTE: x - lower case
//
bool make_write_command(COMMR572_CMND_ID cmnd_id, int parameter, char *cmnd_buffer )
{
    char cmnd_param[16];
    int i;
    short cs;

    for (i = 0; i < N_OF_COMMR572_CMND_IDS; i++ ) 
    {
        if (commr572_cmnd[i].cmnd_id == cmnd_id) 
        {
            sprintf(cmnd_param, "%02x%04x", commr572_cmnd[i].write_command, parameter );
            break;
        }
    }

    if ( i >= N_OF_COMMR572_CMND_IDS ) 
    {
        return FALSE;
    }
    cs = check_sum( cmnd_param );
    sprintf(cmnd_buffer, "%x%s%x%x", STX, cmnd_param, cs, ETX); 

    return TRUE;
}

//
// NOT: x - lower case
//
bool make_read_command( COMMR572_CMND_ID cmnd_id, int parameter,  char *cmnd_buffer )
{
    char cmnd_param[16];
    int i;
    short cs;

    for (i = 0; i < N_OF_COMMR572_CMND_IDS; i++ ) 
    {
        if (commr572_cmnd[i].cmnd_id == cmnd_id) 
        {
            // add 4 hex digits as a command parameter
            sprintf(cmnd_param, "%02x%04x", commr572_cmnd[i].read_command, parameter );
            break;
        }
    }

    if ( i >= N_OF_COMMR572_CMND_IDS ) 
    {
        return FALSE;
    }
    cs = check_sum( cmnd_param );
    sprintf(cmnd_buffer, "%c%s%02x%c", STX, cmnd_param, cs, ETX); 

    return TRUE;
}

