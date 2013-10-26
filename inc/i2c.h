/*H***********************************************************************
 */

/*! \file    i2c.h
 *
 *   \brief   APIs for i2c calls.
 *
 *   \details These APIs are currently being used for u-blox GPS.
 *   Description of when files should be included.
 *
 *   \note    Other help for the reader, including references.
 *
 *
 *//*
 *
 * CHANGES:
 *
 * REF NO  DATE     WHO      DETAIL
 *         09OCT08  PJN         First version
 *
 *H*/

#ifndef _I2C_H_
#define _I2C_H_

/**************************************************************************
 * INCLUDE FILES
 ***************************************************************************/
/*---- system and platform files -----------------------------------------*/
#include "adl_global.h" /* Global include file for ADL APIs */

/*---- program files -----------------------------------------------------*/

/**************************************************************************
 * FILE CONTEXT
 ***************************************************************************/

/*A****************************************************************
 * NAME: Simple name for functional area served by this section
 *
 * USAGE: Description of where, how, etc. items below are used
 *
 * NOTES: Other information to help the reader.
 *
 * CHANGES :
 * REF NO  DATE      WHO     DETAIL
 *         DDMMMYY   Name    First version
 * XXNNNN  DDMMMYY   Name    Name of item changed
 *
 *A*/
/*---- context ---------------------------------------------------*/

/*---- data descriptions -------------------------------------------------*/
typedef int fixed1616;

/*---- extern data declarations ------------------------------------------*/

#define I2C_POLLGPS    (0x70475053)              // "pGPS"

/*---- extern function prototypes ----------------------------------------*/
extern int ioCallToI2CDevice(char *i_DeviceAddr, char *i_Cmd, char *i_Reg, char *i_Data, u8 *rxData);

/*A*****************************************************************
 * NAME: next functional area (as previous section)
 */

#endif /* filename_h ------ END OF FILE ------*/
