/******************************************************************************
 *  File       : GspInterface.h
 ***-----------------------------------------------------------------------------
 *  Object     :  This file contains all interface API to the nanoRide chip set
 *
 *  Contents   :  GpsInit()
 *
 *****************************************************************************/

/******************************************************************************
 * change history
 |*****************************************************************************
 |||flag | author | date   | description
 ||+-----+--------+--------+-----------------------------------------------------
 |     |        |        | create
 ||+-----+--------+--------+-----------------------------------------------------
 |     |        |        |
 ||+-----+--------+--------+-----------------------------------------------------
 |     |        |        |
 ||+-----+--------+--------+-----------------------------------------------------
 |     |        |        |
 ||+-----+--------+--------+-----------------------------------------------------
 *****************************************************************************/
#ifndef __GPS_INTERFACE__H__
#define __GPS_INTERFACE__H__

/******************************************************************************
 *  Includes
 *****************************************************************************/

/***************************************************************************
 *  Defines
 ****************************************************************************/
/* Max length of a NMEA string */
#define NMEA_STR_SZ    (130)

typedef enum
{
	FIX_NONE,
	FIX_2D,
	FIX_3D
} FIX_QUALITY;

struct structGGAInformation
{
	u32 Time;
	s32 Latitude;
	s32 Longitude;
	FIX_QUALITY FixQuality;
	u8 NumberOfSatillites;
	u8 HDOP;
};

struct structVTGInformation
{
	u32 Course;
	u32 Speed;
};

typedef struct
{
	int GGAErrors;
	int RMCErrors;
	int GSVErrors;
} NMEA_ERRORS;

/***************************************************************************
 *  Globals
 ****************************************************************************/

/******************************************************************************
 *  Prototypes
 *****************************************************************************/
s32 GpsInit(void);
s32 ParseGGA(ascii *nmeaStr, u16 len);
void GetGGAInfo(struct structGGAInformation *o_GGAInfo);
void GetSV_SRN(int *o_NumberOfSV, int *o_StrongSNR, int *o_WeakSNR);
int ParseVTGString(char *Str, int iLength);
void GetVGTInfo(struct structVTGInformation *o_VTGInfo);
int IsGPSInitialized(void);

int ParseNMEA(char *Str, int iLength);
void NMEAFailure(char *Str, int iLength);

//  **************** Added with ublox retrofit

void gps_Start(void);
void gps_Stop(void);

void CmdHandlerNMEA(adl_atCmdPreParser_t *y);
void CmdHandlerGPSON(adl_atCmdPreParser_t *y);
void CmdHandlerGPSOFF(adl_atCmdPreParser_t *y);

void GetNMEAError(NMEA_ERRORS *nm);

void FilterSpeed(double *Speed);

#endif // __GPS_INTERFACE__H__
