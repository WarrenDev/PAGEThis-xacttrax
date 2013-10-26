/** @addtogroup GPS
 *@{*/

/*H************************************************************************
 */

/*! \file    anolclient.c
 *
 *   \brief   Modified version of the assisted GPS code provided by ublox.
 *
 *   \details
 *
 *   \note    For additional help, see Andy Bettino
 *
 *
 *//*
 *
 * CHANGES :
 *
 * REF NO  DATE     WHO      DETAIL
 *         20JAN10  AB        First version
 *
 *H*/
#include <limits.h>

#include "wip.h"
#include "wip_bearer.h"
#include "adl_global.h"
#include "ubx.h"
#include "GPRS.h"
#include "anolclient.h"
#include "status.h"
#include "WaypointControl.h"
#include "XactUtilities.h"
#include "ConfigSettings.h"
#include "InternalFlash.h"
#include "GPSCtrl.h"

#define BUFFER_SIZE     4000    //!< Size of the temporary buffers
#define MAXFIELDLEN     128     //!< Maximum length for username and password

#define ANOL_UDP        0       //!< Used in getAssistNowOnlineData() to indicate UDP traffic
#define ANOL_TCP        1       //!< Used in getAssistNowOnlineData() to indicate TCP traffic

static void agps_evh(wip_event_t *ev, void *ctx);
static int getLastFixLatLong(char *lat, char *lon);

extern s8 ubloxDataFcmHandle;
extern AGPS_DATA g_AGPSData;

typedef struct REQ_s
{
	CH username[MAXFIELDLEN];       //!< Username to be sent to server
	CH password[MAXFIELDLEN];       //!< Password to be sent to server
	R8 lat;                         //!< Latitude of approximate location (in decimal degrees)
	R8 lon;                         //!< Longitude of approximate location (in decimal degrees)
	R8 alt;                         //!< Altitude of approximate location (in meters, can be zero)
	R8 accuracy;                    //!< Accuracy of lat/lon, in meters
} REQ_t;

static AGPS_STATUS agps_stat = AGPS_NOT_USED;

static CH buffer[BUFFER_SIZE];  // temporary buffer

static int useLastFix = 0;

void agps_display_float(char *buf, double value);

static wip_channel_t socket;
static unsigned int peerPort;     // UDP or TCP port to use

static CH connectTo[MAXFIELDLEN] = "agps.u-blox.com:46434";
static I proto;
static CH       *cmd;
static REQ_t    *userInfo;
static I bufferLength;
static CH peerName[256];

/** @brief get assist now online data
 *
 * @par
 * Connect to the ublox servers and download the AGPS data.
 * @return 0
 */
I getAssistNowOnlineData(void)
{
	// parse host/port string
	CH *delim = strchr(connectTo, ':');

	// split the connectTo string into host:port parts
	if (delim)
	{
		*delim = (char)0;
		strcpy(peerName, connectTo);
		peerPort = atoi(delim + 1);
		*delim = ':';
	}
	else
	{
		strcpy(peerName, connectTo);
		peerPort = 46434;
	}

	// let's just always use TCP for this.
	socket = wip_TCPClientCreate(peerName, peerPort, agps_evh, NULL);

	if (!socket)
	{
		wip_debug("AGPS: Could not create socket: %d\r\n", socket);
		set_agps_status(AGPS_NOT_USED);
	}
	else
	{
		wip_debug("AGPS: socket open!\r\n");
	}

	return 0;
}


static CH request[256]; /**< Variable to hold the request string */

/** @brief AGPSTxRequest
 *
 * @par
 * Transmit the request to the AGPS server to download the data.
 *
 * @return void
 */
void AGPSTxRequeset(void)
{
	// *****************************************************
	// Then, create the request string and send it
	// *****************************************************
	char lat_float[25];
	char lon_float[25];
	char acc_float[25];


	if (useLastFix)
	{
		if (getLastFixLatLong(lat_float, lon_float) == -1)
		{
			return;
		}
		wip_debug("lat float = %s lon float = %s\r\n", lat_float, lon_float);
		wm_sprintf(acc_float, "1000.00");
	}
	else
	{
		agps_display_float(lat_float, userInfo->lat);
		agps_display_float(lon_float, userInfo->lon);
		agps_display_float(acc_float, userInfo->accuracy);
	}

	//customization for Joe's testing only.
	//strcpy(lat_float, "40.745221");
	//strcpy(lon_float, "-73.984616\n");
	//end - customization.	
	
	wm_sprintf(request, "user=%s;pwd=%s;cmd=%s;lat=%s;lon=%s;pacc=%s",
	           userInfo->username,
	           userInfo->password,
	           cmd,
	           lat_float,
	           lon_float,
	           acc_float);

	wip_debug(request);
	wip_debug("\r\n");
	DumpMessageUSB(request,1);

	DumpMessage("Doing a TCP transmit\r\n");
	DumpMessageUSB("\r\nDoing a TCP transmit\r\n",1);

	{
		I4 totalLength = 0;
		size_t requestLength = strlen(request);

		if (requestLength > INT_MAX)
		{
			wip_debug("request length will cause an overflow in the debug code\r\n");
		}

		totalLength = wip_write(socket, request, requestLength);

		if (totalLength != (int)requestLength)
		{
			wip_debug("AGPS Error: Did not transmit all bytes: %d\r\n", totalLength);
		}
		else
		{
			wip_debug("AGPS Transmit success: %d\r\n", totalLength);
		}
	}
}


static REQ_t req =
{
	"jjesson@xacttechnology.com", "Niywc", 0.0, 0.0, 0.0, 0.0
};
/**< Login credentials for the AGPS server */

/** @brief perform Cold Start
 *
 * @par Cold start GPS communication on the uBlox
 *
 * @return void
 */
void performColdStart(void)
{
	CH ubxColdStart[] =
	{
		0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0x07, 0x02, 0x00, 0x16, 0x79
	};                                                                                                        // UBX Message to force a coldstart
	int ret_val;

	if ((ret_val = adl_fcmSendData(ubloxDataFcmHandle, (unsigned char *)ubxColdStart, sizeof (ubxColdStart))) != OK)
	{
		wip_debug("ERROR: Could not send ublox cold start: %d\r\n", ret_val);
	}
	else
	{
		wip_debug("Sent ublox cold start\r\n");
	}
}


/** @brief get last Fix Lat/Long Position
 *
 * @par get the lat and long of the last fix. write them to the
 * lat and lon buffers. They will then be suitable for using
 * in the data request.
 * @param lat
 * @param lon
 * @return void
 */
static int getLastFixLatLong(char *lat, char *lon)
{
	WAYPOINT last_waypoint;

	if (ReadWaypoint(&last_waypoint, -1))
	{
		wip_debug("Error: Could not read last fix\r\n");
		DumpMessageUSB("Error: Could not read last fix\r\n",1);
		return -1;		
	}

	char latascii[10], longascii[10];
	memcpy(latascii,last_waypoint.lat,8 );
	memcpy(longascii,last_waypoint.longi,8 );
	latascii[8] = '\0';
	longascii[8] = '\0';
	
	wm_sprintf(g_traceBuf, "Waypoint data : \r\n Latitude = %s , Longitude = %s\r\n",latascii, longascii);
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf,1);
	

	int tmp_lat_long = (int)hex_ascii_2_uint(last_waypoint.lat, 32);


	wm_sprintf(g_traceBuf, "Latitude = %d\r\n",tmp_lat_long);
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf,1);
	
	double tmp_lat_long_float = tmp_lat_long / 1000000.000;


	agps_display_float(lat, tmp_lat_long_float);


	tmp_lat_long = hex_ascii_2_uint(last_waypoint.longi, 32);

	wm_sprintf(g_traceBuf, "Longitude = %d\r\n",tmp_lat_long);
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf,1);

	tmp_lat_long_float = tmp_lat_long / 1000000.000;

	agps_display_float(lon, tmp_lat_long_float);

	
	return 0;
}


/** @brief Perform an assited GPS
 *
 * @par This function will cause the device to perform an assited GPS.
 * if lat, lon, alt and accuracy are all set to 0, the device will
 * use the last known GPS coordinates for the assited GPS.
 * otherwise overide it with known values.
 *
 * @param lat
 * @param lon
 * @param alt
 * @param accuracy
 * @return void
 */
void performAGPS(double lat, double lon, double alt, double accuracy)
{
	// check if AGPS is enabled.
	if ((g_config.GPSAlert == GPS_ALERT_ON_OTA_AGPS_OFF) ||
	    (g_config.GPSAlert == GPS_ALERT_OFF_OTA_AGPS_OFF))
	{
		wip_debug("AGPS is disabled, not going to attempt it\r\n");
		set_agps_status(AGPS_NOT_USED);
		return;
	}

	if (!GPRS_HaveBearer())
	{
		wip_debug("Error: GPRS AGPS requested and GPRS Bearer is not established.\r\n");
		set_agps_status(AGPS_NOT_USED);
		return;
	}

	int cell_id;
	int lac;
	cell_id = hex_ascii_2_uint(g_status.CellID, 16);
	lac = hex_ascii_2_uint(g_status.LocationArea, 16);

	// grab the most recent location area and cell id.
	wip_debug("Cell id = %d, location area = %d\r\n", cell_id, lac);

	wm_sprintf(g_traceBuf,"Cell id = %d, location area = %d\r\n", cell_id, lac);
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf,1);

	if ((lat == 0.0) && (lon == 0.0) && (alt == 0.0) && (accuracy == 0.0))
	{
		useLastFix = 1;
	}
	else
	{
		useLastFix = 0;
	}

	// It is important that this gives the approximate (in)accuracy
	// of the initial position, as this parameter is given back
	// to the GPS receiver as part of the UBX-AID-INI message embedded
	// in the payload from the server

	DumpMessage("AssistNow Online Request\r\n");
	DumpMessageUSB("AssistNow Online Request\r\n",1);

	//  performColdStart();

	adl_ctxSleep(ADL_TMR_MS_TO_TICK(1000));

	proto = ANOL_TCP;
	cmd = "full";
	req.lat = lat;
	req.lon = lon;
	req.alt = alt;
	req.accuracy = accuracy;
	userInfo = &req;
	bufferLength = BUFFER_SIZE;

	getAssistNowOnlineData();
}


/** @brief agps event handler
 *
 * @par
 * The WIP event handler for the AGPS session.
 * This will receive the data and put it into the proper buffer.
 * @param ev
 * @param ctx
 * @return void
 */
static void agps_evh(wip_event_t *ev, void *ctx)
{
	(void)ctx;

	static unsigned int rcv_offset = 0;
	static int totalRXLength = 0;
	static I4 expectedLength = 1e7;
	CH                  *headerEnd = NULL;
	I4 readLength = 0;
	static I4 payloadLength = 0;
	static CH           *pPayload = NULL;
	int ret_val;
	adl_rtcTimeStamp_t CurrentTimeStamp;

	switch (ev->kind)
	{
	case WIP_CEV_OPEN:
		wip_setOpts(ev->channel, WIP_COPT_RCV_LOWAT, 1, WIP_COPT_RCV_BUFSIZE, 1500, WIP_COPT_END);

		AGPSTxRequeset();
		break;

	case WIP_CEV_READ:
		DumpMessage("AGPS Some data arrived\r\n");
		DumpMessageUSB("AGPS Some data arrived\r\n",1);

		readLength = wip_read(ev->channel, buffer + rcv_offset,
		                      sizeof (buffer) - rcv_offset);

		rcv_offset += readLength;
		if (rcv_offset >= sizeof (buffer))
		{
			wip_debug("AGPS recieve buffer overflow\r\n");
			// reset our receive buffer static variables.
			totalRXLength = 0;
			expectedLength = 1e7;
			rcv_offset = 0;
			payloadLength = 0;
			pPayload = NULL;
			set_agps_status(AGPS_NOT_USED);
			break;
		}

		if ((totalRXLength < expectedLength) && (totalRXLength < BUFFER_SIZE))
		{
			if (readLength < 0)
			{
				wip_debug("error reading from socket: %i %m!\n", readLength);
				set_agps_status(AGPS_NOT_USED);
				break;
			}

			totalRXLength += readLength;

			// look for the end of the header.
			headerEnd = strstr(buffer, "\r\n\r\n");

			if (headerEnd)
			{
				CH  *pContentLength = strstr(buffer, "Content-Length");
				U4 contentLength = 0;
				if (sscanf(pContentLength, "Content-Length: %u", &contentLength) && (contentLength > 0))
				{
					expectedLength = headerEnd - buffer + contentLength + 4 /* for \r\n\r\n */;
					payloadLength = contentLength;              // store for later
					*headerEnd = (CH)0;                         // split into two strings
					pPayload = headerEnd + 4;                   // store for later
					wip_debug("setting exptected length to %i bytes\n", expectedLength);
				}
			}
		}

		if (totalRXLength >= expectedLength)
		{
			if ((totalRXLength == expectedLength) && pPayload && payloadLength)
			{
				wip_debug("AGPS: received %i bytes okay\n", totalRXLength);
				wm_sprintf(g_traceBuf,"AGPS: received %i bytes okay\n", totalRXLength);
				DumpMessageUSB(g_traceBuf,1);

				do
				{
					// If the header is Content-Type application/ubx,
					// --> we can forward to the GPS receiver
					if (strstr(buffer, "Content-Type: application/ubx"))
					{
						if (payloadLength > BUFFER_SIZE)
						{
							wip_debug("Buffer is too small (%i), received %i bytes. \n", BUFFER_SIZE, payloadLength);
							set_agps_status(AGPS_NOT_USED);
							break;
						}
						else
						{
							wip_debug("ubx data with %i bytes payload\n", payloadLength);
							// no reason to copy this... i don't think.
							//	    memcpy(buffer,pPayload,payloadLength);
						}
					}
					else
					{
						// If the header is Content type text/plain,
						// something went wrong
						if (strstr(buffer, "Content-Type: text/plain"))
						{
							wip_debug("ERROR: %s\n", pPayload);
							set_agps_status(AGPS_NOT_USED);
							break;
						}
						else
						{
							wip_debug("AGPS Unknown error on received data\r\n");
							set_agps_status(AGPS_NOT_USED);
							break;
							// Something went wrong - server does not adhere to our protocol ?!?
						}
					}

					// Transmit to the ublox.
					if ((ret_val = adl_fcmSendData(ubloxDataFcmHandle, (unsigned char *)pPayload, payloadLength)) != OK)
					{
						wip_debug("ERROR: Could not send data to ublox device: %d\r\n", ret_val);
						wm_sprintf(g_traceBuf,"ERROR: Could not send data to ublox device: %d\r\n", ret_val);
						DumpMessageUSB(g_traceBuf,1);						
						set_agps_status(AGPS_NOT_USED);
					}
					else
					{
						wip_debug("Sent AGPS data to Ublox module!\r\n");
						DumpMessageUSB("Sent AGPS data to Ublox module!\r\n",1);
						set_agps_status(AGPS_USED);
					
						memcpy(g_AGPSData.AGPSData, pPayload, payloadLength);
						g_AGPSData.AGPSData[payloadLength] = '\0';
						GetConvertTime(&CurrentTimeStamp);
						g_AGPSData.LastReqTimeStamp = CurrentTimeStamp.TimeStamp;
						g_AGPSData.datalen = payloadLength;
						wm_sprintf(g_traceBuf,"g_AGPSData.LastReqTimeStamp : 0x%x, g_AGPSData.datalen : %ld",(unsigned int)g_AGPSData.LastReqTimeStamp,g_AGPSData.datalen);
						DumpMessage(g_traceBuf);
						DumpMessageUSB(g_traceBuf,1);
						
						//Write the AGPS data to flash
						AGPSDataToFlash();
					}
				} while (0);
			}
			else
			{
				wip_debug("AGPS Error probably received the wrong number of bytes: %d %d %d\r\n", totalRXLength, pPayload, payloadLength);
				// Something went wrong - server does not adhere to our protocol? Not an AGPS Server ?!?
				set_agps_status(AGPS_NOT_USED);
			}

			// reset our receive buffer static variables.
			totalRXLength = 0;
			expectedLength = 1e7;
			rcv_offset = 0;
			payloadLength = 0;
			pPayload = NULL;
		}

		break;

	case WIP_CEV_WRITE:
		wip_debug("AGPS WIP_CEV_WRITE\r\n");
		break;

	case WIP_CEV_ERROR:
		wip_debug("WIP_CEV_ERROR\r\n");
		wip_close(ev->channel);
		break;

	case WIP_CEV_PEER_CLOSE:
		wip_debug("Connection closed by peer\r\n");
		DumpMessageUSB("Connection closed by peer\r\n",1);
		wip_close(ev->channel);
		//set_agps_status(AGPS_NOT_USED);
		break;

	default:
		wip_debug("Unknown WIP event\r\n");
		wip_close(ev->channel);
	}
}


/** @brief agps display float
 *
 * @par
 * Create an ascii representation of a floating point.
 * Doing this because displaying a floating point causes the system to reset.
 * @param buf
 * @param value
 * @return void
 */
void agps_display_float(char *buf, double value)
{
	double pos = 1000;
	int cnt;
	unsigned int tmp_val;
	int leading_zero = 1;
	int buf_pos = 0;

	if (value < 0)
	{
		*buf = '-';
		buf_pos++;
		value = value * -1;
	}

	for (cnt = 0; cnt < 9; cnt++)
	{
		if (cnt == 4)
		{
			*(buf + buf_pos++) = '.';
			continue;
		}

		tmp_val = value / pos;
		value = value - tmp_val * pos;

		if (tmp_val != 0)
		{
			leading_zero = 0;
		}

		if (!leading_zero)
		{
			*(buf + buf_pos++) = tmp_val + 0x30;
		}

		pos = pos / 10;
	}

	buf[buf_pos] = 0x00;
}


void set_agps_status(AGPS_STATUS astat)
{
	agps_stat = astat;
	wm_sprintf(g_traceBuf, "AGPS STATUS = %d\r\n", agps_stat);
	DumpMessage(g_traceBuf);
}


AGPS_STATUS get_agps_status(void)
{
	return agps_stat;
}


/*@}*/
