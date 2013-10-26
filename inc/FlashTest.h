/******************************************************************************
 *  File       : SourceFileName.h
 ***-----------------------------------------------------------------------------
 *	General description of methods to be shared by this header file
 * - function1
 * - function2
 ***-----------------------------------------------------------------------------
 *	General description of Defines to be shared by this header file
 * - DEFINE_NAME_1
 * - DEFINE_NAME_2
 ***-----------------------------------------------------------------------------
 *	any other info that would be useful
 * - bla bla bla
 *****************************************************************************/
#ifndef __H_FLASHTEST_H__
#define __H_FLASHTEST_H__

#include "pistd.h"

/******************************************************************************
 *
 * global variables and definitions to be shared amoung all files
 *
 *****************************************************************************/
/* Switches to define or re-define */

/*#undef LOG_FIX_AVG_NUM
 * #define LOG_FIX_AVG_NUM (100)
 * #undef START_STATE
 * #define START_STATE ('S')
 * #undef LOG_SAT_STATUS
 * #define LOG_TRK
 #undef LOG_OOF */
#define MAX_NUM_FLASH_IDS       (11)
#define FLASH_ID                (0)
#define FENCEFLASHSIZE          (sizeof (COORD) * NUM_FENCES * NUM_POSTS_PER_FENCE)

extern const ascii g_collarStatusHandle[];
extern const ascii g_timerIntervalsHandle[];
extern const ascii g_fromServerNumHandle[];
extern const ascii g_toServerNumHandle[];
extern const ascii g_xmitUtcHandle[];
extern const ascii g_configureHandle[];
extern const ascii g_minFixEpeHandle[];
extern const ascii g_minFixPostEpeHandle[];
extern const ascii g_batAlarmLevHandle[];
extern const ascii g_snModeHandle[];
extern const ascii g_checkSumHandle[];
extern const ascii g_fencesHandle[];
//extern const ascii log1handle[];
//extern const ascii log2handle[];
extern const ascii g_utcTimeHandle[];
extern ascii        *g_FlashLogBlock1[];
extern ascii        *g_FlashLogBlock2[];
extern const ascii g_lastKnownFixHandle[];

/* Status structure.  See the Status Packet in the protocol document for
 * a description of the members */

/*typedef struct
 * {
 * UINT8  batt_lev;
 * UINT8  sys_status;
 * UINT8  gps_status;
 * UINT8  gps_sig1;
 * UINT8  gps_sig2;
 * UINT8  gps_num_birds;
 * UINT8  gsm_status;
 * UINT8  gsm_sig;
 * UINT8 cell_id[4];
 * UINT8 region_id[4];
 * UINT8  valid_access;
 * UINT8  bad_access;
 * UINT8  num_fence;
 * UINT8  mode;
 * UINT8  temperature;
 * UINT8  oof_alarm;
 * UINT8  inf_alarm;
 * UINT8  batt_alarm;
 * UINT8  temp_alarm;
 * UINT8  fence_in;
 * UINT8  log_pages;
 * } COLLAR_STATUS; */

/* Section of config packet structure.  See the Config Packet in the
 * protocol document for a description of the members */

/*typedef struct
 * {
 * UINT8 re_enter_alm;
 * UINT8 over_temp;
 * UINT8 under_temp;
 * UINT8 soft_reset_dummy;
 * UINT8 hard_reset_dummy;
 * UINT8 hyst_bounds[4];
 * UINT8 near_fence_timer;
 * UINT8 fw_rev[2];
 * UINT8 hw_rev[2];
 * UINT8 clear_log_dummy;
 * UINT8 min_post_dist[2];
 * UINT8 ad_hoc_dist[4];
 * UINT8 disable_but;
 * } CONFIG; */

/* Timing intervals (in seconds) for gps fixes and gsm callins */
//typedef struct
//{
// UINT8 gps_interval;      /* minimum minutes */
//UINT8 checkin_interval;  /* minutes */
//} INTERVALS;

/* enumerated device state for indexing into interval array */
//typedef enum {MODE_SETUP, MODE_WALK, MODE_TRACK, MODE_INF, MODE_OOF,
//             MODE_END} COLLAR_STATE;

extern UINT8 g_flash_status[];

/******************************************************************************
* prototypes
******************************************************************************/

void CmdHandlerFlash(adl_atCmdPreParser_t *Cmd);
s8 FlashSubscribe(char const *const flhHandle, u16 numObjects);
s8 FlashWrite(char const *const i_Handle, u16 i_Id, u16 i_Length, u8 const *const o_Data);
s8 FlashRead(char const *const i_Handle, u16 i_Id, u16 i_Length, u8 *const o_Data);
s8 FlashExists(char const *const flhHandle, u16 flhId);
s8 FlashErase(char const *const i_Handle, u16 i_id);

#endif
