#ifndef __ANOL_CLIENT_H__
#define __ANOL_CLIENT_H__

typedef enum
{
	PC_USB_SRC,
	GPRS_SRC
} AGPS_REQ_SRC;

typedef enum
{
	AGPS_NOT_USED,
	AGPS_USED
} AGPS_STATUS;

void set_agps_status(AGPS_STATUS astat);
AGPS_STATUS get_agps_status(void);
void performAGPS(double lat, double lon, double alt, double accuracy);
void performColdStart(void);

#endif
