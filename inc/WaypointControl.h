/*H************************************************************************
 */

/*! \file    WaypointControl.h
 *
 *   \brief   Control the waypoint data to and from external spi flash.
 *
 *   \details
 *
 *   \note    For additional help, see Andy Bettino
 *
 *
 *//*
 *
 * CHANGES :
 *
 * REF NO  DATE     WHO      DETAIL
 *         06JUL09  AB         First version
 *
 *H*/

#ifndef __WAYPOINT_CONTROL_H__
#define __WAYPOINT_CONTROL_H__

#include "common.h"
#include "pistd.h"
#include "gps.h"
#include "GpsInterface.h"

typedef struct
{
	UINT8 utc[8];
	UINT8 lat[8];
	UINT8 longi[8];
	UINT8 epe[2];
	UINT8 speed[8];    // this field is now for speed and course over ground.
} HINT_STRUCT_PACKED WAYPOINT;

//#define NUM_WAYPOINT_BYTES  sizeof(WAYPOINT)

#define NUM_WAYPOINT_BYTES    (34)

// Write a waypoint to the flash.
int WriteWaypoint(WAYPOINT const *const wp);

// Read a waypoint from the flash.
int ReadWaypoint(WAYPOINT *const wp, int offset);

// Display a waypoint.
void DisplayWaypoint(const WAYPOINT *wp);

// Get number of waypoints in flash.
int GetNumWaypoints(void);

// Display the flash pointers.
void DisplayPointers(void);

// Find the start pointers in flash.
int FindFlashLocation(void);

// Display the number of waypoints over usb.
void DisplayNumWaypointsUSB(void);

// Dump the number of waypoints over usb.
void DumpWaypointsUSB(void);

// Write the current fix as a waypoint.
void WriteWaypointFromFix(GPS_POS *fix, int speed, int course_over_ground, FIX_QUALITY fix_quality);
void DisplayWaypointUSB(const WAYPOINT *wp);
void InitSpiFlash(int factory_init);
int CheckFactorySetupSpiFlash(void);
#endif
