/*****************************************************************************
* Copyright (c) 2008 Digi International Inc., All Rights Reserved
*
* This software contains proprietary and confidential information of Digi
* International Inc.  By accepting transfer of this copy, Recipient agrees
* to retain this software in confidence, to prevent disclosure to others,
* and to make no use of this software other than that for which it was
* delivered.  This is an unpublished copyrighted work of Digi International
* Inc.  Except as permitted by federal law, 17 USC 117, copying is strictly
* prohibited.
*
* Restricted Rights Legend
*
* Use, duplication, or disclosure by the Government is subject to
* restrictions set forth in sub-paragraph (c)(1)(ii) of The Rights in
* Technical Data and Computer Software clause at DFARS 252.227-7031 or
* subparagraphs (c)(1) and (2) of the Commercial Computer Software -
* Restricted Rights at 48 CFR 52.227-19, as applicable.
*
* Digi International Inc. 11001 Bren Road East, Minnetonka, MN 55343
*
*****************************************************************************/

#ifndef I2C_ACCELEROMETER_H
#define I2C_ACCELEROMETER_H

#include "i2c.h"
#include "Traces.h"

#define I2C_ACCELEROMETER_MAX_TIMEOUT       0
#define I2C_ACCELEROMETER_MAX_RETRIES       0

typedef struct
{
	int x_10_bit;
	int y_10_bit;
	int z_10_bit;
	int x_8_bit;
	int y_8_bit;
	int z_8_bit;
	int x_offset_value;
	int y_offset_value;
	int z_offset_value;
} ACCEL_CALIBRATION_DATA;

/*
 * Defines the data type for the callback function which gets called when
 * the vector sum of the acceleration/g-force in X,Y and Z axes crosses the
 * registered vector sum of threshold.
 */
typedef void (*NaAccelerometerCallbackFn_t)(fixed1616 xaxis, fixed1616 yaxis,
                                            fixed1616 zaxis, unsigned long context);

/*
 * This API configures the initial settings of the accelerometer. Sets the
 * suitable mode, resolution and other device specific configuration.
 */
void naAccelerometerEpoch(void);

/*
 * This API gets the acceleration/g-force in X,Y and Z direction. Function
 * updates the current values read from the device in fixed1616 format.
 *
 * @param xPtr  Pointer to hold the X axis reading
 * @param yPtr  Pointer to hold the Y axis reading
 * @param zPtr  Pointer to hold the Z axis reading
 *
 * @return  0   Successfully read the value from the device
 * @return  -1  Failed to read data from the device or driver is busy reading
 */
int naAccelerometerGetValue(fixed1616 *xPtr, fixed1616 *yPtr, fixed1616 *zPtr);

/*
 * This API sets the trigger threshold and enables the interrupt which can
 * assert the interrupt line when one of the axis value crosses the given
 * threshold.
 *
 * @param  threshold     Trigger occurs when acceleration on any one direction
 *                       crosses this threshold value. This is a linear value.
 *                       How to calculate this threshold value?
 *                       For example if the threshold is 2.875g then this
 *                       field should have the value (2.875 * 2^16) = 0x2E000
 *
 * @return  0   Success
 * @return  -1  Failed to communicate with the device
 */
int naAccelerometerSetTriggerThreshold(char threshold);

/*
 * This API clears the interrupt and deasserts the interrupt line.
 *
 * @return  0   Success
 * @return  -1  Failed to communicate with the device
 */
int naAccelerometerClearTrigger(void);

int ReadAccelerometer(void);

int naAccelerometerCheckIRQ(void);

int naAcclerometerSetMeasureMode(void);

int naAcclerometerSetLevelMode(void);

int naReadAccelData(ACCEL_CALIBRATION_DATA *accel_calib_data);

int naWriteAccelData(ACCEL_CALIBRATION_DATA *accel_calib_data);

unsigned char naAccelReadThreshold(void);

int naAcceleromterSetCtrl2(void);

#endif /* I2C_ACCELEROMETER_H */
