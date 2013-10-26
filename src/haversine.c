/** @addtogroup GPS
 *@{*/

/*H************************************************************************
 */

/** @file    haversine.c
 *
 *   @brief  Implments the haversine formula for calculating distances
 *           on the surface of the earth.
 *   @details Does haversine formulu for circular fences. Also implements
 *            a cartesian approximation for other fences using standard
 *            polygon processing techniques.
 *
 *   @note    Other help for the reader, including references.
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

#include "adl_global.h"
#include <math.h>
#include "haversine.h"
#include "XactUtilities.h"

#define PI    3.14159265

extern COORD g_current_fix_32;

int Intersect(COORD *c1, COORD *c2, COORD *c3);

// return the distance between two coordinates, in yards.
int points2distance(COORD *ac, COORD *bc)
{
	double start_long = ((double)ac->long_32 / 1000000);
	double start_latt = ((double)ac->lat_32 / 1000000);
	double end_long = ((double)bc->long_32 / 1000000);
	double end_latt = ((double)bc->lat_32 / 1000000);

	start_long = start_long * PI / 180;
	start_latt = start_latt * PI / 180;
	end_long = end_long * PI / 180;
	end_latt = end_latt * PI / 180;

	double delta_latt = (end_latt - start_latt);
	double delta_long = (end_long - start_long);
	double a = pow(sin(delta_latt / 2), 2) + cos(start_latt) * cos(end_latt) * pow(sin(delta_long / 2), 2);
	double c = 2 * atan2(sqrt(a), sqrt(1 - a));

	double yard_float = 6967410.32 * c;

	int yard_int = (int)(yard_float);

	return yard_int;
}


/** @brief Intersect two coordinates
 *
 * @par Give this a polygon side, as defined by by the coordinants c2 and c3
 * the current fix is at c1 this will create a projection on the
 * eastward latitude from c1.
 *
 * @param c1
 * @param c2
 * @param c3
 * @return 0 for does not intersect,1 for does intersect
 */
int Intersect(COORD *c1, COORD *c2, COORD *c3)
{
	// first check to see if we are to the west of the polygon side and
	// if the latitude is in the range

	SINT32 north_lat;
	SINT32 south_lat;

	// figure out which one is more to the north
	if (c2->lat_32 > c3->lat_32)
	{
		north_lat = c2->lat_32;
		south_lat = c3->lat_32;
	}
	else
	{
		north_lat = c3->lat_32;
		south_lat = c2->lat_32;
	}

	// if the fix is more north than the north point or more south
	// than the south point, we cannot possibly intersect.
	if ((c1->lat_32 > north_lat) || (c1->lat_32 < south_lat))
	{
		return 0;
	}

	// do a cartesian approximation of the slope of the polygon segment.
	// this should work well except for very large polygon side lengths.
	SINT32 slope = (c2->lat_32 - c3->lat_32) / (c2->long_32 - c3->long_32);

	// find the longitude at which this fix intersects this line segment.
	// this is from the from y-y1=m(x-x1)
	// we get x = (y-y1+m*x)/m where x = longitude and y = latitude
	SINT32 long_intersect = (c1->lat_32 - c2->lat_32 + slope * c2->long_32) / slope;

	// if the fix longitude is greater than the intersection latitude, the eastward
	// project has intersected the polygon segement.
	if (c1->long_32 >= long_intersect)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}


/** @brief Check quad status.
 *
 * @par
 * Determine if we are inside a quad by counting the number of times
 *  an east-west project intersects with the boundaries.
 *
 * @param c1
 * @param c2
 * @param c3
 * @param c4
 * @return FIX_OOF if outside the fence
 * @return FIX_INF if inside the fence
 */
UINT8 quadStatus(COORD *c1, COORD *c2, COORD *c3, COORD *c4)
{
	int intersect_cnt = 0;

	if (Intersect(&g_current_fix_32, c1, c2))
	{
		intersect_cnt++;
	}
	if (Intersect(&g_current_fix_32, c2, c3))
	{
		intersect_cnt++;
	}
	if (Intersect(&g_current_fix_32, c3, c4))
	{
		intersect_cnt++;
	}
	if (Intersect(&g_current_fix_32, c4, c1))
	{
		intersect_cnt++;
	}

	wm_sprintf(g_traceBuf, "intersect cnt = %d\r\n", intersect_cnt);
	DumpMessage(g_traceBuf);

	if (intersect_cnt % 2 == 0)
	{
		return FIX_OOF;
	}
	else
	{
		return FIX_INF;
	}
}


/** @brief Check if we are inside or outside a triangle Geofence.
 *
 * @par
 * Determine if we are inside a triangle by counting the number of times
 *  an east-west project intersects with the boundaries.
 * @param c1
 * @param c2
 * @param c3
 * @return FIX_OOF if outside the fence
 * @return FIX_INF if inside the fence
 */
UINT8 triangleStatus(COORD *c1, COORD *c2, COORD *c3)
{
	int intersect_cnt = 0;

	if (Intersect(&g_current_fix_32, c1, c2))
	{
		intersect_cnt++;
	}
	if (Intersect(&g_current_fix_32, c2, c3))
	{
		intersect_cnt++;
	}
	if (Intersect(&g_current_fix_32, c3, c1))
	{
		intersect_cnt++;
	}

	if (intersect_cnt % 2 == 0)
	{
		return FIX_OOF;
	}
	else
	{
		return FIX_INF;
	}
}


/**@}*/
