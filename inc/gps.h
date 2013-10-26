#ifndef _GPS_H
#define _GPS_H

/*-----------------------------------------------------------------------------
 *   INCLUDES
 *  -----------------------------------------------------------------------------*/
#include "pistd.h"

/*--------------------------------------------------------------------------
 *  TYPE DEFINITIONS
 *  --------------------------------------------------------------------------*/

/* lat & long as 32-bit integers */
typedef struct
{
	SINT32 lat_32;
	SINT32 long_32;
} COORD;

// Inclusive or exclusive fence
typedef enum
{
	INCLUSIVE,
	EXCLUSIVE
} INEXC;

// geofence alarm directions
typedef enum
{
	SAFE_FENCE = 0,
	ALARM_LEAVE,
	ALARM_ENTER,
	ALARM_BOTH,
	ALARM_MOTION
} ALARM_DIRECTION;

// geofence active days
typedef enum
{
	ALL_DAYS = 0,
	MON_WED_FRI,
	TUES_THURS,
	MON_THRU_FRI,
	SAT_SUN,
	MON,
	TUES,
	WED,
	THURS,
	FRI,
	SAT,
	SUN
} ACTIVE_DAY;

// geofence active times
typedef struct
{
	unsigned char start_time;
	unsigned char stop_time;
} ACTIVE_TIME;

// fence enabled or disabled
typedef enum
{
	DISABLE,
	ENABLE
} ENDISABLE;

// geofence structure
typedef struct
{
	COORD posts[4];
	ALARM_DIRECTION alarm_dir;
	ACTIVE_DAY days;
	ACTIVE_TIME time;
	ENDISABLE enable;
} GEOFENCE;

/* lat & long as 16-bit integers */
typedef struct
{
	SINT16 lat_16;
	SINT16 long_16;
} COORD16;

#ifdef __GNUC__
#define BIT_PACKED_STRUCT    __attribute__ ((packed))
#else
#define BIT_PACKED_STRUCT
#endif

/* Fix data as strings of hex ascii */
struct GPS_POS_STRUCT
{
	UINT8 utc[8];
	UINT8 lat[8];
	UINT8 longi[8];
	UINT8 epe[2];     /*  */
}
BIT_PACKED_STRUCT;

typedef struct GPS_POS_STRUCT GPS_POS;

/*--------------------------------------------------------------------------
 *  FUNCTION PROTOTYPES
 *  --------------------------------------------------------------------------*/
//UINT8 gps_init(void);
UINT8 get_gps_fix(UINT16, BOOL);

/*void send_nmea(UINT8 *);
 * UINT8 sirf_out(UINT8 *, UINT16);
 * UINT8 sirf_in(UINT8, UINT8 *, UINT16, UINT16);
 */

#endif
