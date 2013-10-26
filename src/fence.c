/** @addtogroup GPS
 *@{*/

/*H***************************************************************************
 */

/** @file	 fence.c
 *
 *	@brief	 Performs Geofence calcualtions to determine in and out of fence
 *
 *	@details Each Geofence is processed and it is determined if the tracking
 *                device's location has created any violations.
 *
 *   @note    Other help for the reader, including references.
 *
 *
 *//*
 *
 * CHANGES :
 * REF NO	DATE	WHO		DETAIL
 *		20AUG09	AB		Ported from refrence design
 *H*/

/**************************************************************************
 *   INCLUDE FILES
 ***************************************************************************/
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "pistd.h"
#include "XactUtilities.h"
#include "gps.h"
#include "fence.h"
#include "adl_global.h"
#include "ConfigSettings.h"
#include "GPSCtrl.h"
#include "haversine.h"
#include "status.h"
#include "VibrationMotor.h"
#include "GpsInterface.h"

/*-----------------------------------------------------------------------------
 *  CONSTANT & MACRO DEFINITIONS
 *  -----------------------------------------------------------------------------*/
//#define DISPLAY_TEST_INFO (1)

/*-----------------------------------------------------------------------------
 *  GLOBAL DECLARATIONS
 *  -----------------------------------------------------------------------------*/
GEOFENCE g_GeoFences[NUM_FENCES];
GEOFENCE g_DogParkFence;

extern COORD g_current_fix_32;

extern TRACKING_MODE TrackingMode;
extern int VibrateDone;

unsigned int g_in_safe_fence = 0;

/*--------------------------------------------------------------------------
 *  FUNCTION PROTOTYPES
 *  --------------------------------------------------------------------------*/
static UINT8 num_post(UINT8);
static UINT8 circle_fence_status(COORD *, UINT16, UINT32 *);

// store the status of all fences.
static UINT8 fence_status[NUM_FENCES];

extern struct structGGAInformation g_GGAInfo;

static UINT8 check_fence_active(int fence_num);
static void init_fence_array(void);
static UINT8 dog_park_eval(void);

#define MONDAY          (1)
#define TUESDAY         (2)
#define WEDNESDAY       (3)
#define THURSDAY        (4)
#define FRIDAY          (5)
#define SATURDAY        (6)
#define SUNDAY          (7)

unsigned int IsInSafeFence(void)
{
    return g_in_safe_fence;
}

/** @brief check fence active
 *
 * @par
 * Determine if the fence is active based on the configuration settings for the fence.
 * Some fences may only be active for certain days or certain times during the week.
 * @param fence_num
 * @return 0 for fence active, 1 for fence not active
 */
static UINT8 check_fence_active(int fence_num)
{
	// load the fix timestamp into the time stamp structure.
	adl_rtcTimeStamp_t fix_time_stamp;
	fix_time_stamp.TimeStamp = g_GGAInfo.Time;
	fix_time_stamp.SecondFracPart = 0;

	// convert it to a time structure
	adl_rtcTime_t fix_time_struct;

	adl_rtcConvertTime(&fix_time_struct, &fix_time_stamp, ADL_RTC_CONVERT_FROM_TIMESTAMP);

	// First check if the day is valid
	switch (g_GeoFences[fence_num].days)
	{
	case ALL_DAYS:
		break;

	case MON_WED_FRI:
		if ((fix_time_struct.WeekDay != MONDAY) && (fix_time_struct.WeekDay != WEDNESDAY) &&
		    (fix_time_struct.WeekDay != FRIDAY))
		{
			return 1;
		}
		break;

	case TUES_THURS:
		if ((fix_time_struct.WeekDay != TUESDAY) && (fix_time_struct.WeekDay != THURSDAY))
		{
			return 1;
		}
		break;

	case MON_THRU_FRI:
		if ((fix_time_struct.WeekDay == SATURDAY) || (fix_time_struct.WeekDay == SUNDAY))
		{
			return 1;
		}
		break;

	case SAT_SUN:
		if ((fix_time_struct.WeekDay != SATURDAY) && (fix_time_struct.WeekDay != SUNDAY))
		{
			return 1;
		}
		break;

	case MON:
		if (fix_time_struct.WeekDay != MONDAY)
		{
			return 1;
		}
		break;

	case TUES:
		if (fix_time_struct.WeekDay != TUESDAY)
		{
			return 1;
		}
		break;

	case WED:
		if (fix_time_struct.WeekDay != WEDNESDAY)
		{
			return 1;
		}
		break;

	case THURS:
		if (fix_time_struct.WeekDay != THURSDAY)
		{
			return 1;
		}
		break;

	case FRI:
		if (fix_time_struct.WeekDay != FRIDAY)
		{
			return 1;
		}
		break;

	case SAT:
		if (fix_time_struct.WeekDay != SATURDAY)
		{
			return 1;
		}
		break;

	case SUN:
		if (fix_time_struct.WeekDay != SUNDAY)
		{
			return 1;
		}
		break;

	default:
		return 1;
	}

	// now check if we are within the time window.
	int total_mins = (fix_time_struct.Hour * 60 + fix_time_struct.Minute) / 15;
	unsigned char current_time_15_mins = total_mins & 0xff;
	if ((current_time_15_mins >= g_GeoFences[fence_num].time.start_time) &&
	    (current_time_15_mins <= g_GeoFences[fence_num].time.stop_time))
	{
		return 0;
	}
	else
	{
		return 1;
	}
}


// init fence array if needed.
static void init_fence_array(void)
{
	static char first_eval = 1;
	if (first_eval)
	{
		int ii;
		for (ii = 0; ii < NUM_FENCES; ii++)
		{
			fence_status[ii] = 'U';
		}
		first_eval = 0;
	}
}


// check if we are in dog park mode. evaluate position if true.
static UINT8 dog_park_eval(void)
{
	UINT8 result = 'F';
	UINT16 distance;
	UINT32 how_far_out = 0xffffffff;      /* oof distance */
	if (TrackingMode == DOG_PARK_TRACKING_MODE)
	{
		DumpMessage("DOG PARK EVAL\r\n");
		distance = DOG_PARK_DISTANCE;
		if (FIX_INF == circle_fence_status(&g_DogParkFence.posts[0], distance, &how_far_out))
		{
			result = 'I';
		}

		wm_sprintf(g_traceBuf, "DOG PARK %u %u %u\r\n", (unsigned int)how_far_out, (unsigned int)g_DogParkFence.posts[0].lat_32,
		           (unsigned int)g_DogParkFence.posts[0].long_32);

		DumpMessage(g_traceBuf);
		return result;
	}
	return 0;     // 0 indiciates not in dog park mode.
}


/** @brief evaluate fix
 *
 * @par Sees if we're in or out of a fence
 * @param fence_num
 * @return 'I' = In fence, 'F' = oof, 'T' = track, 'E' = exclusive fence violate
 */
UINT8 eval_fix(int *fence_num, EVAL_REASON reason)
{
	(void)fence_num;

	UINT8 num_posts;            /* Number of fence posts in a fence */
	UINT8 fence_cntr;           /* Counts through fences */
	UINT8 result = 'F';         /* Return variable */
	UINT16 distance;            /* radius */
	UINT8 dog_park_result;

	UINT32 how_far_out = 0xffffffff;      /* oof distance */

	static int vibrate_stat = -1;
	static bool alarm_this_dog_park = false;
	// init fence array if needed.
	init_fence_array();

	// check dog park mode.
	if ((dog_park_result = dog_park_eval()) != 0)
	{
		if ((dog_park_result == 'F') && (alarm_this_dog_park == false))
		{
			g_status.FenceAlarm = 'L';
			g_status.CurrentFenceIn = 0;
			alarm_this_dog_park = true;
		}
		return dog_park_result;
	}

	alarm_this_dog_park = false;
	// AB: bad hack. please fix.
	g_in_safe_fence = 0;

	DumpMessage("Eval fix\r\n");

	/* loop through all fences */
	for (fence_cntr = 0; fence_cntr < NUM_FENCES; fence_cntr++)
	{
		result = 'O';

		/* first evaluate if we should be checking the fence */
		if (g_GeoFences[fence_cntr].enable == DISABLE)
		{
			continue;
		}

		if (check_fence_active(fence_cntr))
		{
			DumpMessage("Fence not active\r\n");
			continue;
		}

		/* Get the number of posts in this fence */
		num_posts = num_post(fence_cntr);

		/* If there's no posts in this fence, go to the next one */
		if (num_posts == 0)
		{
			TRACE((1, "No posts in fence: %d\r\n", fence_cntr));
			fence_status[fence_cntr] = 'U';
			continue;
		}

		if (num_posts == 1)     /*********** ad hoc ***************/
		{
			DumpMessage("Error - we do not support 1 post fence\r\n");
			continue;
		}
		else if (num_posts == 2)     /************* center - radius **********/
		{
			distance = points2distance(&g_GeoFences[fence_cntr].posts[0],
			                           &g_GeoFences[fence_cntr].posts[1]);

			/* If we're inside this fence - we know all we need to */
			if (FIX_INF == circle_fence_status(&g_GeoFences[fence_cntr].posts[0], distance,
			                                   &how_far_out))
			{
				result = 'I';
			}
		}
		else if (num_posts == 3)     /********************** triangle *************/
		{
			if (FIX_INF == triangleStatus(&g_GeoFences[fence_cntr].posts[0],
			                              &g_GeoFences[fence_cntr].posts[1],
			                              &g_GeoFences[fence_cntr].posts[2]))
			{
				result = 'I';
			}
		}
		else if (num_posts > 3)     /********************** polygon *************/
		{
			if (FIX_INF == quadStatus(&g_GeoFences[fence_cntr].posts[0],
			                          &g_GeoFences[fence_cntr].posts[1],
			                          &g_GeoFences[fence_cntr].posts[2],
			                          &g_GeoFences[fence_cntr].posts[3]))
			{
				result = 'I';
			}
		}

		wm_sprintf(g_traceBuf, "Fence = %d fence_status = %c\r\n", fence_cntr, result);
		DumpMessageUSB(g_traceBuf, 1);
		// Alarm generation.
		switch (g_GeoFences[fence_cntr].alarm_dir)
		{
		case SAFE_FENCE:
			g_in_safe_fence = 1;
			g_status.CurrentFenceIn = fence_cntr + 1;
			break;

		case ALARM_LEAVE:
			if ((fence_status[fence_cntr] == 'I') && (result == 'O'))
			{
				g_status.FenceAlarm = 'L';
				g_status.CurrentFenceIn = fence_cntr + 1;
			}
			else if (result == 'I')
			{
				g_status.CurrentFenceIn = fence_cntr + 1;
			}
			break;

		case ALARM_ENTER:
			if ((fence_status[fence_cntr] == 'O') && (result == 'I'))
			{
				g_status.FenceAlarm = 'E';
				g_status.CurrentFenceIn = fence_cntr + 1;
			}
			else if (result == 'I')
			{
				g_status.CurrentFenceIn = fence_cntr + 1;
			}
			break;

		case ALARM_BOTH:
			if ((fence_status[fence_cntr] == 'O') && (result == 'I'))
			{
				g_status.FenceAlarm = 'E';
				g_status.CurrentFenceIn = fence_cntr + 1;
			}
			else if ((fence_status[fence_cntr] == 'I') && (result == 'O'))
			{
				g_status.FenceAlarm = 'L';
				g_status.CurrentFenceIn = fence_cntr + 1;
			}
			else if (result == 'I')
			{
				g_status.CurrentFenceIn = fence_cntr + 1;
			}
			break;

		case ALARM_MOTION:
			if ((result == 'I') && (reason == MOTION_ALARM_REASON))
			{
				g_status.CurrentFenceIn = fence_cntr + 1;
				g_status.FenceAlarm = 'M';
			}
			break;

		default:
			wm_sprintf(g_traceBuf, "Unknown alarm direction: %x\r\n", g_GeoFences[fence_cntr].alarm_dir);
			DumpMessage(g_traceBuf);
			if (result == 'I')
			{
				g_status.CurrentFenceIn = fence_cntr + 1;
			}
		}

		// vibration warning control.
		if (vibrate_stat == -1)
		{
			if ((fence_status[fence_cntr] == 'I') && (result == 'O') &&
			    ((g_GeoFences[fence_cntr].alarm_dir == ALARM_BOTH) ||
			     (g_GeoFences[fence_cntr].alarm_dir == ALARM_LEAVE)))
			{
				vibrate_stat = fence_cntr;
				if (g_config.VibrationMotorDelayEn == ENABLE)
				{
					VibrateDone = 0;
					g_config.VibrationMotorPattern = VIBRATE_FENCE_WARNING;
				}
			}
		}
		else
		{
			// if we are in the fence that started the vibration.
			if (fence_cntr == vibrate_stat)
			{
				if (result == 'I')
				{
					g_config.VibrationMotorPattern = 0;
				}
			}
		}

		fence_status[fence_cntr] = result;
	}     /* end of for fence loop */

	return result;
}


/** @brief Gets the number of posts of a fence
 *
 * @param fence_num
 * @return post_cntr
 */
static UINT8 num_post(UINT8 fence_num)
{
	//  UINT8 checker = 0x55;
	UINT8 post_cntr;      /* Counts through post locations */

	/* Count through posts until you hit an invalid one or you hit the
	 * max */
	for (post_cntr = 0; post_cntr < NUM_POSTS_PER_FENCE; post_cntr++)
	{
		/* If its not a valid post, we're done */
		if (((g_GeoFences[fence_num].posts[post_cntr].lat_32 == 0) &&
		     (g_GeoFences[fence_num].posts[post_cntr].long_32 == 0)) ||
		    ((g_GeoFences[fence_num].posts[post_cntr].lat_32 == -1) &&
		     (g_GeoFences[fence_num].posts[post_cntr].long_32 == -1)))
		{
#if defined(OLD_CODE_THAT_PRODUCES_WARNINGS)
			(g_GeoFences[fence_num].posts[post_cntr].lat_32 == 0xffffffff &&
			 g_GeoFences[fence_num].posts[post_cntr].long_32 == 0xffffffff)) {
#endif
			break;
		}
	}
#ifdef DISPLAY_TEST_INFO
	TRACE((1, "number of posts: %d", post_cntr));
#endif
	//stack_check(&checker, "eonpost");
	return post_cntr;
}


/** @brief Sees if we're in or out of a circular fence
 *
 * @param post - center post coords
 * @param radius - radius of fence in yards
 * @param how_far_out
 * @return FIX_INF or FIX_OOF
 */
static UINT8 circle_fence_status(COORD *post, UINT16 radius, UINT32 *how_far_out)
{
	// UINT8 checker = 0x55;
	UINT32 distance;        /* Distance to center of fence from here */
	UINT16 epe;             /* Estimated error */

	/* Get the epe of the fix (in yards) */
	epe = 0;

	/* get distance (in yards) from current position to center of fence */
	distance = points2distance(post, &g_current_fix_32);

#ifdef DISPLAY_TEST_INFO
	TRACE((1, "Radius: %d", radius));
	TRACE((1, "epe: %d", epe));
	TRACE((1, "Distance: %d", distance));
#endif
	/* Return the appropriate status */
	if (distance <= (UINT32)(radius + epe))
	{
		//  stack_check(&checker, "circfnc");
		return FIX_INF;
	}
	else     /* We're outside the fence */
	{
		/* If the distance outside the fence is less than others so far */
		if ((distance - (UINT32)(radius + epe)) < *how_far_out)
		{
			/* Its the new closest fence */
			*how_far_out = distance - (UINT32)(radius + epe);
		}
		//      stack_check(&checker, "eocircfnc");
		return FIX_OOF;
	}
}


// init the fence status when fence is set from config packet.
void init_fence_status(int fence_num)
{
	fence_status[fence_num] = 'U';
}


/*@}*/
