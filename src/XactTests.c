#include <adl_global.h>

#include "ConfigSettings.h"
#include "WaypointControl.h"
#include "XactTests.h"
#include "GpsInterface.h"
#include "XactUtilities.h"

//extern UINT8  g_DogParkMode;
UINT8  g_DogParkMode;
extern CONFIG g_config;
extern int g_GPSTestMode;
static struct structGGAInformation gGGAInfoTest;
static struct structVTGInformation gVTGInfoTest;
extern char g_traceBuf[256];

void GPSWaypointTest(u8 timerID, void *Context)
{
#define NUM_WAYPOINT_TEST (1000)
  (void)timerID; (void)Context;

  // enable artificial waypoints.
  g_GPSTestMode = 1;

  static unsigned int time_val =0;
  static int num_test = 0;

  if (num_test < NUM_WAYPOINT_TEST) {
    //    wm_sprintf(g_traceBuf, "fix quality = %d\r\n", gGGAInfoTest.FixQuality);
    //    DumpMessage(g_traceBuf);
    if ( gGGAInfoTest.FixQuality == 0) {
      gGGAInfoTest.Time               = time_val++;
      gGGAInfoTest.Latitude           = 0;
      gGGAInfoTest.Longitude          = 0;
      gGGAInfoTest.FixQuality         = FIX_3D;
      gGGAInfoTest.NumberOfSatillites = 0;
      gVTGInfoTest.Course             = 0;
      gVTGInfoTest.Speed              = 0;

      wm_sprintf(g_traceBuf, "Fix %d\r\n",time_val);
      DumpMessage(g_traceBuf);
      adl_tmrSubscribe(FALSE, (g_config.WaypointInterval*10), ADL_TMR_TYPE_100MS, (adl_tmrHandler_t)(GPSWaypointTest));
      num_test++;
    } else  {
      adl_tmrSubscribe(FALSE, 1, ADL_TMR_TYPE_100MS, (adl_tmrHandler_t)(GPSWaypointTest));
    }

  }
  else if (num_test == NUM_WAYPOINT_TEST) {
    adl_tmrSubscribe(FALSE, (g_config.WaypointInterval*10), ADL_TMR_TYPE_100MS, (adl_tmrHandler_t)(GPSWaypointTest));
    num_test++;
  }
  else {
    time_val = 0;
    num_test = 0;
    WAYPOINT verify_waypoint;
    int ii,jj;
    unsigned int time_verify_val;
    for(ii=0,jj=-1;ii<NUM_WAYPOINT_TEST;ii++,jj--) {
      ReadWaypoint(&verify_waypoint, jj);
      time_verify_val = wm_hexatoi((ascii *)verify_waypoint.utc,8);
	//     wm_sprintf(g_traceBuf, "%x %x %x %x %x %x %x %x\r\n", verify_waypoint.utc[0],verify_waypoint.utc[1],verify_waypoint.utc[2],verify_waypoint.utc[3],verify_waypoint.utc[4],verify_waypoint.utc[5],verify_waypoint.utc[6],verify_waypoint.utc[7]);
	//      DumpMessage(g_traceBuf);
      wm_sprintf(g_traceBuf, "time verify val = %u\r\n", time_verify_val);
      DumpMessage(g_traceBuf);
      if ((int )time_verify_val != NUM_WAYPOINT_TEST+jj) {
	wm_sprintf(g_traceBuf, "check fail at time verify = %u\r\n", time_verify_val);
	DumpMessage(g_traceBuf);
      }
    }
  }
}


void DogParkTest(u8 timerID, void *Context)
{
  (void)timerID; (void)Context;
  typedef enum {
    GET_SOME_FIXES,
    ENABLE_DOG_PARK_MODE,
    MOVE_THE_DEVICE,
    TEST_DONE,
  } DOG_PARK_TEST_STATES;

  static DOG_PARK_TEST_STATES state;
  static unsigned int cur_lat = 100;
  static unsigned int cur_long = 100;
  static unsigned int time_val =0;
  static unsigned int fix_cnt = 0;

  // enable artificial waypoints.
  g_GPSTestMode = 1;

#define NUM_FIXES_BEFORE_PARK 3
#define NUM_TOTAL_FIXES 10
#define LAT_INCR 1000

  switch (state)
  {
  case GET_SOME_FIXES :
    if (fix_cnt >= NUM_FIXES_BEFORE_PARK)
      state = ENABLE_DOG_PARK_MODE;
    break;
  case ENABLE_DOG_PARK_MODE :
    g_DogParkMode = 1;
    state = MOVE_THE_DEVICE;
    break;
  case MOVE_THE_DEVICE :
    if (fix_cnt >= NUM_TOTAL_FIXES)
      state = TEST_DONE;
    break;
  case TEST_DONE :
  default :
    break;
  }

  if (((state == GET_SOME_FIXES) || (state == MOVE_THE_DEVICE)) && (gGGAInfoTest.FixQuality == 0))
  {  
    if (state == MOVE_THE_DEVICE)
      cur_lat += LAT_INCR;

    gGGAInfoTest.Time               = time_val++;
    gGGAInfoTest.Latitude           = cur_lat;
    gGGAInfoTest.Longitude          = cur_long;
    gGGAInfoTest.FixQuality         = FIX_3D;
    gGGAInfoTest.NumberOfSatillites = 0;
    gVTGInfoTest.Course             = 0;
    gVTGInfoTest.Speed              = 0;

    wm_sprintf(g_traceBuf, "Fix %d\r\n",time_val);
    DumpMessage(g_traceBuf);
    fix_cnt++;
  }

  if (state != TEST_DONE)
    adl_tmrSubscribe(FALSE, 10, ADL_TMR_TYPE_100MS, (adl_tmrHandler_t)(DogParkTest));
  else {
    g_GPSTestMode = 0;
    g_DogParkMode = 0;
  }
}


void DogParkTestV2(u8 timerID, void *Context)
{
  (void)timerID; (void)Context;
  typedef enum {
    GET_SOME_FIXES,
    ENABLE_DOG_PARK_MODE,
    MOVE_THE_DEVICE,
    TEST_DONE,
  } DOG_PARK_TEST_STATES;

  static DOG_PARK_TEST_STATES state;
  static unsigned int cur_lat = 0x02630EC3;
  static unsigned int cur_long = 0xFB804CE4;
  static unsigned int time_val =0;
  static unsigned int fix_cnt = 0;

  // enable artificial waypoints.
  g_GPSTestMode = 1;

#define NUM_FIXES_BEFORE_PARK 3
#define NUM_TOTAL_FIXES 10
#define LAT_INCR 1000

  switch (state)
  {
  case GET_SOME_FIXES :
    if (fix_cnt >= NUM_FIXES_BEFORE_PARK)
      state = ENABLE_DOG_PARK_MODE;
    break;
  case ENABLE_DOG_PARK_MODE :
    g_DogParkMode = 1;
    state = MOVE_THE_DEVICE;
    break;
  case MOVE_THE_DEVICE :
    if (fix_cnt >= NUM_TOTAL_FIXES)
      state = TEST_DONE;
    break;
  case TEST_DONE :
  default :
    break;
  }

  if (((state == GET_SOME_FIXES) || (state == MOVE_THE_DEVICE)) && (gGGAInfoTest.FixQuality == 0))
  {  
#if 0
    if (state == GET_SOME_FIXES && fix_cnt == NUM_FIXES_BEFORE_PARK-1) {
      cur_lat = 0x02630D3F;
      cur_long = 0xFB803F77;
    }
#endif

    if (state == MOVE_THE_DEVICE) {
      cur_lat = 0x02630D17;
      cur_long = 0xFB803F1F;
    }
    

    gGGAInfoTest.Time               = time_val++;
    gGGAInfoTest.Latitude           = cur_lat;
    gGGAInfoTest.Longitude          = cur_long;
    gGGAInfoTest.FixQuality         = FIX_3D;
    gGGAInfoTest.NumberOfSatillites = 0;
    gVTGInfoTest.Course             = 0;
    gVTGInfoTest.Speed              = 0;

    wm_sprintf(g_traceBuf, "Fix %d %u %u\r\n",time_val,cur_lat, cur_long);
    DumpMessage(g_traceBuf);
    fix_cnt++;
  }

  if (state != TEST_DONE)
    adl_tmrSubscribe(FALSE, 10, ADL_TMR_TYPE_100MS, (adl_tmrHandler_t)(DogParkTestV2));
  else {
    g_GPSTestMode = 0;
    g_DogParkMode = 0;
  }
}
