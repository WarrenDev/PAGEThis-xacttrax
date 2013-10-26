/*H************************************************************************
 */

/*! \file    VibrationMotor.h
 *
 *   \brief   Controls the vibration motor.
 *
 *   \details
 *
 *   \note    Other help for the reader, including references.
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

#ifndef __VIBRATIONMOTOR_H__
#define __VIBRATIONMOTOR_H__

#include "adl_global.h"
#include "pistd.h"

#define MAX_ON_OFF                  40  /*!< Max on/off cycles */
#define MAX_PATTERNS                9   /*!< Max number of stored patterns */

// Pulse durations
#define MD_PLS                      5   /*! <Duration of "Medium pulse", hundreds of milliseconds */
#define LG_PLS                      10  /*! < Duration of a "long pulse", hundres of milliseconds */

// Time durations.
#define FIVE_S                      50  /*!< Duration of 5 seconds, hundreds of milliseconds */
#define THREE_S                     30  /*!< Duration of 3 seconds, hundreds of milliseconds */
#define ONE5_S                      15  /*!<  Duration of 1.5 seconds, hundreds of milliseconds */

#define VIBRATION_TRACE_LEVEL       19

#define VIBRATE_FENCE_WARNING       1
#define VIBRATE_SOS_PATTERN         2
#define VIBRATE_SOS_ACK             3
#define VIBRATE_STATUS_ACK          4

extern void VibrateTimer(u8 timerid, void *context);
extern void vibrate_turn_off(void);
extern void vibrate_turn_on(void);
#endif
