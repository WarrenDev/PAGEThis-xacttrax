/****************************************************************************
*
*  ublox.h
*
****************************************************************************
*
*  MODIFICATION HISTORY
* --------------------------------------------------------------------------
*  Date      | Author      | Revision | Description
* -----------+-------------+----------+-------------------------------------
*  17-Sep-09 | C. Eslinger | 1.0      | * Initial Release
* --------------------------------------------------------------------------
*
****************************************************************************/
#ifndef UBLOX_H
#define UBLOX_H

#define GPS_UART                    ADL_PORT_UART2
#define MAX_NMEA_SENTENCE_LENGTH    100  // NMEA spec states 82

/* -----------------------------------------------------*/
/*               Global function declarations           */
/* -----------------------------------------------------*/

void ublox_InitGPIO(void);
void ublox_InitSerialPort(void);
void ublox_SetupRXchannel(void);
void ublox_Start(void);
void ublox_Service(void);
void ublox_stop(void);
void ublox_ShowData(bool setting);
bool ublox_isON(void);

#endif   // UBLOX_H
