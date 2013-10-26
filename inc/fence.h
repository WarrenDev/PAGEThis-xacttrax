#ifndef FENCE_H
#define FENCE_H

/*-----------------------------------------------------------------------------
 *   INCLUDES
 *  -----------------------------------------------------------------------------*/
#include "pistd.h"

/*-----------------------------------------------------------------------------
 *  CONSTANT & MACRO DEFINITIONS
 *  -----------------------------------------------------------------------------*/
/* Maximum number of fences */
#define NUM_FENCES              50

/* Maximum number of posts per fence */
#define NUM_POSTS_PER_FENCE     (4)

// radius of fence in dog park mode.
#define DOG_PARK_DISTANCE       328

typedef enum
{
	GPS_FIX_REASON,
	MOTION_ALARM_REASON
} EVAL_REASON;

/*--------------------------------------------------------------------------
 *  FUNCTION PROTOTYPES
 *  --------------------------------------------------------------------------*/
unsigned int IsInSafeFence(void);
UINT8 eval_fix(int *fence_num, EVAL_REASON reason);
void init_fence_status(int fence_num);

#endif
