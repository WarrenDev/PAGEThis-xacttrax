/*H************************************************************************
 */

/*! \file    LED.c
 *
 *   \brief   Controls the LEDs.
 *
 *   \details Blinks the LEDS to indicates status and mode.
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

#ifndef __LED_H__
#define __LED_H__

#include "adl_global.h"

#define LED_ON_INTERVAL             5   /*!< Number of 100ms intervals for the LEDs to be on*/
#define DOG_WALK_INTERVAL           30  /*!< Number of 100ms intervals for the LEDs to flash in dog walk mode */
#define DOG_PARK_INTERVAL           30  /*!< Number of 100ms intervals for the LEDs to flash in dog park mode */
#define SHUTDOWN_INTERVAL           5
#define NORMAL_MODE_INTERVAL        50  /*!< Number of 100ms intervals for the LEDs to flash in normal mode */

#define SHUTDOWN_THRESH             50  /*!< Number of 100ms before entering shutdown mode */

#define MAX_ON_OFF_LED              10
#define MAX_PATTERNS                9
#define NUM_LEDS                    6

// LED definitions. Each LED is dual green/red pack.
#define POWER_LED_RED               35  /*!< Power LED GPIO */
#define POWER_LED_GREEN             6   /*!< Power LED GPIO */
#define GSM_LED_RED                 48  /*!< GSM LED GPIO   */
#define GSM_LED_GREEN               0   /*!< GSM LED GPIO   */
#define GPS_LED_RED                 9   /*!< GPS LED GPIO   */
#define GPS_LED_GREEN               10  /*!< GPS LED GPIO   */

#define SKIP_ON                     0xffffffff

// LED patterns
#define PATTERN_OFF                 0
#define PATTERN_NORMAL              1
#define PATTERN_WALK                2
#define PATTERN_PARK                3
#define PATTERN_SHUTDOWN            4
#define PATTERN_SOS                 5

#define DONE                        0
#define SOS_DURATION                4   /*!< Number of 100ms intervals to have the LED on*/
#define LED_ON_DURATION_NORMAL      3   /*!< Number of 100ms intervals to have the LED on*/
#define LED_OFF_DURATION_NORMAL     50  /*!< Number of 100ms intervals to have the LED off*/
#define LED_OFF_DURATION_WALK       30  /*!< Number of 100ms intervals to have the LED off*/
#define LED_OFF_DURATION_PARK       30  /*!< Number of 100ms intervals to have the LED off*/

// LED masks.
#define LED_STATE_MASK              0xFF000000  /*!< Bits that tell which LED status to check */
#define POWER_STATUS_MSK            0x80000000  /*!< Check the power status before lighting the LED*/
#define GSM_STATUS_MSK              0x40000000  /*!< Check the GSM status before lighting the LED*/
#define GPS_STATUS_MSK              0x20000000  /*!< Check the GPS status before lighting the LED*/
#define LED_OFF_MSK                 0x10000000
#define POWER_STATUS_MSK_GREEN      0x08000000  /*!< Check the green power LED */
#define GPS_STATUS_MSK_GREEN        0x04000000  /*!< Check the GPS green LED */

#define SOS_BLINK_TIME              10          /*!< Number of seconds to blink the SOS LEDs for */

typedef enum
{
	DOG_WALK_STATE_LED_OFF,
	NORMAL_STATE_LED_OFF,
	NORMAL_STATE_LED_ON,
	DOG_PARK_STATE_LED_OFF,
	SHUTDOWN_STATE_LED_ON,
	SHUTDOWN_STATE_LED_OFF
} ledState;

typedef enum
{
	FIX_3D_LED,         // 3D fix.
	FIX_BAD_LED,        // no fix, no sats.
	FIX_2D_LED,         // 2D fix.
	SATS_VISIBLE_LED    // sats are visible.
} GPS_LED_STATUS;

extern void BlinkLED(u8 timerid, void *context);
extern void ShutOffLED(void);
extern void TurnOnLED(void);
extern void TurnOnLEDNum(int lednum);
extern void TurnOffLEDNum(int lednum);
extern void SetGPSLedStatus(GPS_LED_STATUS stat);
extern int check_shutdown_start(void);
#endif
