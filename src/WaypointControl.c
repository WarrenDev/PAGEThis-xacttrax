/** @addtogroup Storage
 *@{ */

/*H************************************************************************
 */

/*! \file    WaypointControl.c
 *
 *   \brief   Control the waypoint data to and from external spi flash.
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
 *         06JUL09  AB         First version
 *
 *H*/

/**************************************************************************
 *   INCLUDE FILES
 ***************************************************************************/
/*---- system and platform files -----------------------------------------*/
#include "adl_global.h"         /* The global include file for all of the ADL APIs */
#include "WaypointControl.h"    /* Include file for waypoint control */

#include "Traces.h"
#include "adl_shoring.h"
#include "gps.h"
#include "XactUtilities.h"
#include "anolclient.h"
#include "gpioTest.h"

#if defined(SST_FLASH)
#include "sst_flashDriver.h"
#define CHUNK_SIZE          PAGE_SIZE
#define NUMBER_OF_CHUNKS    NUMBER_OF_PAGES

#define ChunkWrite(chunk_number, buffer)    sf_writePage(chunk_number, buffer)
#define ChunkRead(chunk_number, buffer)     sf_readPage(chunk_number, buffer)
#endif

#if defined(ATMEL_FLASH)
#include "atmel_flashDriver.h"
#define CHUNK_SIZE          SECTOR_SIZE
#define NUMBER_OF_CHUNKS    NUMBER_OF_SECTORS

#define ChunkWrite(chunk_number, buffer)    sf_writeSector(chunk_number, buffer)
#define ChunkRead(chunk_number, buffer)     sf_readSector(chunk_number, buffer)

static WAYPOINT blank_waypoint_wrapped;
static WAYPOINT blank_waypoint_unwrapped;
#endif

#define FIRST_CHUNK             1 /*!< this sector number to start at */
#define NUM_WAYPOINTS           WAYPOINTS_PER_CHUNK * (NUMBER_OF_CHUNKS - FIRST_CHUNK)
#define FACTORY_INIT_MAGIC      0xdeadbeef

DEBUG_TRACE_STORAGE;

static unsigned int const WAYPOINTS_PER_CHUNK = CHUNK_SIZE / sizeof (WAYPOINT);

/*****************************************************************
 *   DATA DECLARATIONS
 ******************************************************************/
// configuration of the waypoint controller. Comes from flash.
static int current_write_chunk = FIRST_CHUNK;                       /*!< The current sector for writing */
static unsigned int waypoint_offset = 0;                            /*!< waypoint position within current sector for writing*/
static unsigned int number_of_waypoints = 0;                        /*!< Number of waypoints available */
static unsigned int oldest_chunk = FIRST_CHUNK;                     /*!< sector pointer to oldest data */
static unsigned int oldest_chunk_waypoint_offset = 0;               /*!< sector pointer to oldest data */
static bool32_t bufferHasWrapped = false;                           /*!< indicates if the buffer has wrapped */

static u8 ChunkRdBuf[CHUNK_SIZE];

#define STATIC_INIT_BLANK_8_BYTE_ARRAY(array)       array = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff }
#define STATIC_INIT_BLANK_2_BYTE_ARRAY(array)       array = { 0xff, 0xff }
static WAYPOINT const blank_waypoint =
{
	STATIC_INIT_BLANK_8_BYTE_ARRAY(.utc),
	STATIC_INIT_BLANK_8_BYTE_ARRAY(.lat),
	STATIC_INIT_BLANK_8_BYTE_ARRAY(.longi),
	STATIC_INIT_BLANK_2_BYTE_ARRAY(.epe),
	STATIC_INIT_BLANK_8_BYTE_ARRAY(.speed)
};

static char const WRAPPED_FLASH_HANDLE[] = "HasWrapped";

/*****************************************************************
 *   PUBLIC FUNCTION DEFINITIONS
 ******************************************************************/
void SpiFlashFactoryInit(void);

static void DumpWaypointsUSBTimer(void);

/*F***************************************************************
 *
 *   NAME:    int WriteWaypoint( const Waypoint * wp)
 */

/*! \brief   Write a waypoint to the serial flash.
 *
 *	\param	 wp     [in]	The waypoint to write
 *
 *   \return 0 Success
 * other Failure
 *   \note   Pay attention to how the waypoint are stored in flash... not all the flash is used.
 *           Sector size of 512 bytes and a waypoint size of 36 bytes 512/36 = 14 waypoints per flash
 *           There's an extra 8 bytes at the end of each sector.
 *//*
 *
 * CHANGES :
 * REF NO  DATE     WHO      DETAIL
 *         05JUL09  AB	    Initial version
 *F*/
int WriteWaypoint(WAYPOINT const *const wp)
{
	int sResult = SUCCESS;

	// First, read the current sector.
	if ((sResult = ChunkRead(current_write_chunk, ChunkRdBuf)) != SUCCESS)
	{
		return sResult;
	}

	wm_sprintf(g_traceBuf, "waypoint_offset = %d current_write_chunk = %d\r\n", waypoint_offset, current_write_chunk);
	DumpMessage(g_traceBuf);
	wm_sprintf(g_traceBuf, "oldest_chunk_waypoint_offset = %d oldest_chunk = %d\r\n", oldest_chunk_waypoint_offset, oldest_chunk);
	DumpMessage(g_traceBuf);

	// copy the waypoint into the read buffer.
	memcpy(ChunkRdBuf + waypoint_offset * sizeof (WAYPOINT), wp, sizeof (WAYPOINT));

	// Write it out to flash.
	if ((sResult = ChunkWrite(current_write_chunk, ChunkRdBuf)) != SUCCESS)
	{
		return sResult;
	}

	// Adjust pointers.
	if (bufferHasWrapped)
	{
		DumpMessage("Buffer wrapped!\r\n");
		if ((++oldest_chunk_waypoint_offset == WAYPOINTS_PER_CHUNK))
		{
			if ((++oldest_chunk == NUMBER_OF_CHUNKS))
			{
				oldest_chunk = FIRST_CHUNK;
			}
			oldest_chunk_waypoint_offset = 0;
		}
	}

	// we just wrapped the buffer
	if ((waypoint_offset == 0) && (current_write_chunk == FIRST_CHUNK) && (number_of_waypoints != 0))
	{
		{
			bufferHasWrapped = true;
			s32 result = adl_flhWrite(UNCONST(char *, WRAPPED_FLASH_HANDLE), 0
			                          , sizeof bufferHasWrapped, RECAST(unsigned char *, &bufferHasWrapped));
			INSIST(result == OK);
		}
	}

	if ((++waypoint_offset) == WAYPOINTS_PER_CHUNK)
	{
		if ((++current_write_chunk) == NUMBER_OF_CHUNKS)
		{
			current_write_chunk = FIRST_CHUNK;
		}
		waypoint_offset = 0;
	}

	if (number_of_waypoints < (NUM_WAYPOINTS))
	{
		number_of_waypoints++;
	}

	// write out the start pattern.

#if defined(ATMEL_FLASH)
	// First, read the current sector.
	if ((sResult = ChunkRead(current_write_chunk, ChunkRdBuf)) != SUCCESS)
	{
		return sResult;
	}

	if (bufferHasWrapped || ((waypoint_offset == 0) && (current_write_chunk == FIRST_CHUNK) && (number_of_waypoints != 0)))
	{
		// copy the waypoint into the read buffer.
		memcpy(ChunkRdBuf + waypoint_offset * sizeof (WAYPOINT), (u8 *)&blank_waypoint_wrapped, sizeof (WAYPOINT));
	}
	else
	{
		// copy the waypoint into the read buffer.
		memcpy(ChunkRdBuf + waypoint_offset * sizeof (WAYPOINT), (u8 *)&blank_waypoint_unwrapped, sizeof (WAYPOINT));
	}

	// Write it out to flash.
	if ((sResult = ChunkWrite(current_write_chunk, ChunkRdBuf)) != SUCCESS)
	{
		return sResult;
	}
#endif

#if defined(SST_FLASH)
	if (waypoint_offset == 0)
	{
		sResult = ChunkWrite(current_write_chunk, NULL);
	}
#endif

	return sResult;
}


/*F***************************************************************
 *
 *   NAME:    int ReadWaypoint(Waypoint * wp)
 */

/*! \brief   Read a waypoint to the serial flash.
 *
 *	\param	 wp     [out]	Waypoint pointer to write result to
 *  \param   offset        waypoint offset to read from. Positive is from oldest data, negatative from newest
 *
 *   \return 0 Success
 * other Failure
 *   \note   Pay attention to how the waypoint are stored in flash... not all the flash is used.
 *
 *//*
 *
 * CHANGES :
 * REF NO  DATE     WHO      DETAIL
 *         05JUL09  AB	    Initial version
 *F*/
int ReadWaypoint(WAYPOINT *const wp, int offset)
{
	int sResult = SUCCESS;
	int sectorOffset = 0;
	int waypointOffset = 0;

	// don't try to read if there's nothing to read.
	if (number_of_waypoints == 0)
	{
		return -1;
	}

	// First, calculate the sector and waypoint offset needed.
	unsigned int tmp_offset = 0;
	unsigned int waypoints_in_chunk = 0;
	unsigned int whole_Chunks = 0;
	if (offset < 0)
	{
		tmp_offset = offset * -1;
	}
	else
	{
		tmp_offset = offset;
	}

	// see if there are enough waypoints for this request.
	if (tmp_offset > number_of_waypoints)
	{
		return -1;
	}

	if (offset < 0)
	{
		DumpMessage("Negative Offset\r\n");
		tmp_offset = offset * -1;
		// this condition we read from current write pointer sector for data.
		if (tmp_offset <= waypoint_offset)
		{
			sectorOffset = current_write_chunk;
			waypointOffset = waypoint_offset - tmp_offset;
		}
		else
		{
			// Number of whole sectors offset and waypoint offset.
			whole_Chunks = tmp_offset / WAYPOINTS_PER_CHUNK;
			waypoints_in_chunk = tmp_offset % WAYPOINTS_PER_CHUNK;

			// condition where we need to go back another sector.
			if (waypoints_in_chunk > waypoint_offset)
			{
				if (current_write_chunk - whole_Chunks - 1 >= FIRST_CHUNK)
				{
					sectorOffset = current_write_chunk - whole_Chunks - 1;
				}
				else
				{
					// we have wrapped the circular buffer.
					sectorOffset = NUMBER_OF_CHUNKS - ((whole_Chunks + 1) - (current_write_chunk - FIRST_CHUNK));
				}
				waypointOffset = WAYPOINTS_PER_CHUNK - (waypoints_in_chunk - waypoint_offset);
			}
			else
			{
				if (current_write_chunk - whole_Chunks >= FIRST_CHUNK)
				{
					sectorOffset = current_write_chunk - whole_Chunks;
				}
				else
				{
					// we have wrapped the circular buffer.
					sectorOffset = NUMBER_OF_CHUNKS - (whole_Chunks - (current_write_chunk - FIRST_CHUNK));
				}
				waypointOffset = waypoint_offset - waypoints_in_chunk;
			}
		}
	}
	else
	{
		unsigned int u_offset = (unsigned int)offset;
		if (u_offset < WAYPOINTS_PER_CHUNK - oldest_chunk_waypoint_offset)
		{
			sectorOffset = oldest_chunk;
			waypointOffset = oldest_chunk_waypoint_offset + u_offset;
		}
		else
		{
			// Number of whole sectors offset and waypoint offset.
			whole_Chunks = u_offset / WAYPOINTS_PER_CHUNK;
			waypoints_in_chunk = u_offset % WAYPOINTS_PER_CHUNK;

			// check if the waypoint offset within the sector will require accessing
			// another sector past whole_Chunks
			if (waypoints_in_chunk >= (WAYPOINTS_PER_CHUNK - oldest_chunk_waypoint_offset))
			{
				// detect the wrapping of the circular buffer.
				if (oldest_chunk + whole_Chunks + 1 >= NUMBER_OF_CHUNKS)
				{
					sectorOffset = FIRST_CHUNK + (whole_Chunks + 1) - (NUMBER_OF_CHUNKS - oldest_chunk);
				}
				else
				{
					sectorOffset = oldest_chunk + whole_Chunks + 1;
				}

				waypointOffset = waypoints_in_chunk - (WAYPOINTS_PER_CHUNK - oldest_chunk_waypoint_offset);
			}
			else
			{
				// detect the wrapping of the circular buffer.
				if (oldest_chunk + whole_Chunks >= NUMBER_OF_CHUNKS)
				{
					sectorOffset = FIRST_CHUNK + (whole_Chunks) - (NUMBER_OF_CHUNKS - oldest_chunk);
				}
				else
				{
					sectorOffset = oldest_chunk + whole_Chunks;
				}

				waypointOffset = oldest_chunk_waypoint_offset + waypoints_in_chunk;
			}
		}
	}

	if ((sResult = ChunkRead(sectorOffset, ChunkRdBuf)) != SUCCESS)
	{
		return sResult;
	}

	// copy the waypoint into the read buffer.
	memcpy((u8 *)wp, ChunkRdBuf + waypointOffset * sizeof (WAYPOINT), sizeof (WAYPOINT));

	return sResult;
}


/*F***************************************************************
 *
 *   NAME:    int DisplayWaypoint( const Waypoint * wp)
 */

/*! \brief   Display a waypoint for debug
 *
 *	\param	 wp     [in]	The waypoint to display
 *
 *   \return 0 Success
 * other Failure
 *   \note
 *
 *//*
 *
 * CHANGES :
 * REF NO  DATE     WHO      DETAIL
 *         05JUL09  AB	    Initial version
 *F*/
void DisplayWaypoint(const WAYPOINT *wp)
{
	int i;
	char display_buf[9];
	unsigned int speed_cog;

	// UTC
	for (i = 0; i < 8; i++)
	{
		display_buf[i] = wp->utc[i];
	}
	display_buf[8] = 0x00;
	wm_sprintf(g_traceBuf, "UTC: %s\r\n", display_buf);
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf, 1);

	// lat
	for (i = 0; i < 8; i++)
	{
		display_buf[i] = wp->lat[i];
	}
	display_buf[8] = 0x00;
	wm_sprintf(g_traceBuf, "LAT: %s\r\n", display_buf);
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf, 1);

	// long
	for (i = 0; i < 8; i++)
	{
		display_buf[i] = wp->longi[i];
	}
	display_buf[8] = 0x00;
	wm_sprintf(g_traceBuf, "LONG: %s\r\n", display_buf);
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf, 1);

	// epe
	for (i = 0; i < 2; i++)
	{
		display_buf[i] = wp->epe[i];
	}
	display_buf[2] = 0x00;
	wm_sprintf(g_traceBuf, "EPE: %s\r\n", display_buf);
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf, 1);

	// speed and cog.
	speed_cog = hex_ascii_2_uint((UINT8 *)wp->speed, 32);
	wm_itohexa(display_buf, speed_cog >> 23, 3);
	display_buf[3] = 0;
	wm_sprintf(g_traceBuf, "COG: %s\r\n", display_buf);
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf, 1);

	wm_itohexa(display_buf, speed_cog & 0x3fffff, 6);
	display_buf[6] = 0;
	wm_sprintf(g_traceBuf, "SPEED: %s\r\n", display_buf);
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf, 1);

	if (speed_cog & 0x400000)
	{
		DumpMessage("3D FIX\r\n");
		DumpMessageUSB("3D FIX\r\n", 1);
	}
	else
	{
		DumpMessage("2D FIX\r\n");
		DumpMessageUSB("2D FIX\r\n", 1);
	}
}


int GetNumWaypoints(void)
{
	return number_of_waypoints;
}


void InitSpiFlash(int factory_init)
{
	//No need to call this code twice, since we already called it in the check --pjn
#if defined(OBSOLETE_CODE)
	if (sf_MediaInit())
	{
		DumpMessage("Fail init spi flash\r\n");
		return;
	}
	else
	{
		DumpMessage("Success init spi flash\r\n");
	}
#endif

#if defined(SST_FLASH)
	{
		s8 result = adl_flhSubscribe(UNCONST(char *, WRAPPED_FLASH_HANDLE), 1);
		INSIST((result == OK) || (result == ADL_RET_ERR_ALREADY_SUBSCRIBED));

		if (result == ADL_RET_ERR_ALREADY_SUBSCRIBED)
		{
			result = adl_flhExist(UNCONST(char *, WRAPPED_FLASH_HANDLE), 0);
			INSIST(result >= 0);

			if (result > 0)
			{
				result = adl_flhRead(UNCONST(char *, WRAPPED_FLASH_HANDLE), 0, sizeof bufferHasWrapped, RECAST(unsigned char *, &bufferHasWrapped));
				INSIST(result == OK);
			}
		}
	}
#endif

#if defined(ATMEL_FLASH)
	// setup the "blank" waypoint.
	memset((u8 *)&blank_waypoint_wrapped, 0xff, sizeof (WAYPOINT));
	memset((u8 *)&blank_waypoint_unwrapped, 0xef, sizeof (WAYPOINT));
#endif

	if (factory_init)
	{
		SpiFlashFactoryInit();
	}

	FindFlashLocation();
}


int CheckFactorySetupSpiFlash(void)
{
	int sResult;
	unsigned int first_int;

	ResetSPIFlash();

	if (sf_MediaInit(CHUNK_SIZE))
	{
		DumpMessage("Fail init spi flash\r\n");
		return 1;
	}
	else
	{
		DumpMessage("Success init spi flash\r\n");
	}

	// Try to read sector 0.
	if ((sResult = ChunkRead(0, ChunkRdBuf)) != SUCCESS)
	{
		DumpMessage("Could not read sector 0\r\n");
		return 1;
	}

	first_int = *((unsigned int *)ChunkRdBuf);
	wm_sprintf(g_traceBuf, "magic number = %x\r\n", first_int);
	DumpMessage(g_traceBuf);

	// Check for the factory number.
	if (*((unsigned int *)ChunkRdBuf) != FACTORY_INIT_MAGIC)
	{
		return 1;
	}

	return 0;
}


void SpiFlashFactoryInit(void)
{
	int sResult = 0;

	*((unsigned int *)ChunkRdBuf) = FACTORY_INIT_MAGIC;

	if ((sResult = ChunkWrite(0, ChunkRdBuf)) != SUCCESS)
	{
		return;
	}

#if defined(ATMEL_FLASH)
	if (WriteWaypoint(&blank_waypoint_unwrapped))
	{
		DumpMessage("Could not factory init spi flash\r\n");
	}
	else
	{
		DumpMessage("Factory init spi flash\r\n");
	}
#endif

#if defined(SST_FLASH)
	spiFlash_EraseChip();
	{
		bool32_t const buffer_default = false;
		s32 result = adl_flhWrite(UNCONST(char *, WRAPPED_FLASH_HANDLE), 0
		                          , sizeof bufferHasWrapped, RECAST(unsigned char *, &buffer_default));
		INSIST(result == OK);
	}
#endif
}


/*F***************************************************************
 *
 *   NAME:    FindFlashLocation
 */

/*! \brief   find where we left off in flash
 *
 *   \return 0 Success
 * other Failure
 *   \note
 *
 *//*
 *
 * CHANGES :
 * REF NO  DATE     WHO      DETAIL
 *         05JUL09  AB	    Initial version
 *F*/
int FindFlashLocation(void)
{
	unsigned int i, j;
	int sResult = SUCCESS;

	for (i = FIRST_CHUNK; i < NUMBER_OF_CHUNKS; i++)
	{
		// read the current sector
		if ((sResult = ChunkRead(i, ChunkRdBuf)) != SUCCESS)
		{
			return sResult;
		}

		// process each waypoint in the sector.
		for (j = 0; j < WAYPOINTS_PER_CHUNK; j++)
		{
#if defined(ATMEL_FLASH)
			// check if this waypoint matches the "wrapped" waypoint.
			if (!memcmp((u8 *)(ChunkRdBuf + sizeof (WAYPOINT) * j),
			            (u8 *)&blank_waypoint_wrapped, sizeof (WAYPOINT)))
			{
				current_write_chunk = i;
				waypoint_offset = j;

				// the case where we are pointing at the last waypoint and last sector
				if ((j == WAYPOINTS_PER_CHUNK - 1) && (i == NUMBER_OF_CHUNKS - 1))
				{
					// wrap it around.
					oldest_chunk = FIRST_CHUNK;
					oldest_chunk_waypoint_offset = 0;
				}
				else
				{
					if (j == WAYPOINTS_PER_CHUNK - 1)
					{
						// the next sector
						oldest_chunk = i + 1;
						oldest_chunk_waypoint_offset = 0;
					}
					else
					{
						// the current sector
						oldest_chunk = i;
						oldest_chunk_waypoint_offset = j + 1;
					}
				}

				// this is true since the wrapped pattern was encountered.
				bufferHasWrapped = true;
				number_of_waypoints = NUM_WAYPOINTS - 1;
				return SUCCESS;
			}

			if (!memcmp((u8 *)(ChunkRdBuf + sizeof (WAYPOINT) * j),
			            (u8 *)&blank_waypoint_unwrapped, sizeof (WAYPOINT)))
			{
				// This is easy in this case since we never wrapped.
				current_write_chunk = i;
				waypoint_offset = j;
				oldest_chunk = FIRST_CHUNK;
				oldest_chunk_waypoint_offset = 0;
				bufferHasWrapped = false;
				number_of_waypoints = (i - 1) * WAYPOINTS_PER_CHUNK + j;
				return SUCCESS;
			}
#endif

#if defined(SST_FLASH)
			if (!memcmp((ChunkRdBuf + sizeof (WAYPOINT) * j),
			            &blank_waypoint, sizeof (WAYPOINT)))
			{
				current_write_chunk = i;
				waypoint_offset = j;

				if (bufferHasWrapped)
				{
					// the case where we are pointing at the last waypoint and last sector
					if ((j == WAYPOINTS_PER_CHUNK - 1) && (i == NUMBER_OF_CHUNKS - 1))
					{
						oldest_chunk = FIRST_CHUNK;
						oldest_chunk_waypoint_offset = 0;
					}
					else
					{
						if (j == WAYPOINTS_PER_CHUNK - 1)
						{
							oldest_chunk = i + 1;
							oldest_chunk_waypoint_offset = 0;
						}
						else
						{
							oldest_chunk = i;
							oldest_chunk_waypoint_offset = j + 1;
						}
					}

					number_of_waypoints = NUM_WAYPOINTS - 1;
				}
				else
				{
					oldest_chunk = FIRST_CHUNK;
					oldest_chunk_waypoint_offset = 0;
					number_of_waypoints = (i - 1) * WAYPOINTS_PER_CHUNK + j;
				}

				return SUCCESS;
			}
#endif
		}
	}

	// Well, we never found a good pattern
	return -1;
}


/*F***************************************************************
 *
 *   NAME:    void DisplayPointers( void)
 */

/*! \brief   Display flash pointers for debug
 *
 *   \note
 *
 *//*
 *
 * CHANGES :
 * REF NO  DATE     WHO      DETAIL
 *         05JUL09  AB	    Initial version
 *F*/
void DisplayPointers(void)
{
	wm_sprintf(g_traceBuf, "current_write_chunk: %d\r\n", current_write_chunk);
	DumpMessage(g_traceBuf);
	wm_sprintf(g_traceBuf, "waypoint_offset: %d\r\n", waypoint_offset);
	DumpMessage(g_traceBuf);
	wm_sprintf(g_traceBuf, "oldest_chunk: %d\r\n", oldest_chunk);
	DumpMessage(g_traceBuf);
	wm_sprintf(g_traceBuf, "oldest_chunk_waypoint_offset: %d\r\n", oldest_chunk_waypoint_offset);
	DumpMessage(g_traceBuf);
	wm_sprintf(g_traceBuf, "bufferHasWrapped: %d\r\n", bufferHasWrapped);
	DumpMessage(g_traceBuf);
	wm_sprintf(g_traceBuf, "number_of_waypoints: %d\r\n", number_of_waypoints);
	DumpMessage(g_traceBuf);
	wm_sprintf(g_traceBuf, "number of waypoints: %u\r\n", NUM_WAYPOINTS);
	DumpMessage(g_traceBuf);
}


/*F***************************************************************
 *
 *   NAME:    int DisplayWaypoint( const Waypoint * wp)
 */

/*! \brief   Display a waypoint for debug
 *
 *	\param	 wp     [in]	The waypoint to display
 *
 *   \return 0 Success
 * other Failure
 *   \note
 *
 *//*
 *
 * CHANGES :
 * REF NO  DATE     WHO      DETAIL
 *         05JUL09  AB	    Initial version
 *F*/
void DisplayWaypointUSB(const WAYPOINT *wp)
{
	int i;
	char display_buf[9];

	// UTC
	for (i = 0; i < 8; i++)
	{
		display_buf[i] = wp->utc[i];
	}

	display_buf[8] = 0x00;
	wm_sprintf(g_traceBuf, "UTC: %s\r\n", display_buf);
	DumpMessageUSB(g_traceBuf, 1);

	// lat
	for (i = 0; i < 8; i++)
	{
		display_buf[i] = wp->lat[i];
	}

	display_buf[8] = 0x00;
	wm_sprintf(g_traceBuf, "LAT: %s\r\n", display_buf);
	DumpMessageUSB(g_traceBuf, 1);

	// long
	for (i = 0; i < 8; i++)
	{
		display_buf[i] = wp->longi[i];
	}

	display_buf[8] = 0x00;
	wm_sprintf(g_traceBuf, "LONG: %s\r\n", display_buf);
	DumpMessageUSB(g_traceBuf, 1);

	// epe
	for (i = 0; i < 2; i++)
	{
		display_buf[i] = wp->epe[i];
	}

	display_buf[2] = 0x00;
	wm_sprintf(g_traceBuf, "EPE: %s\r\n", display_buf);
	DumpMessageUSB(g_traceBuf, 1);

	// speed
	for (i = 0; i < 8; i++)
	{
		display_buf[i] = wp->speed[i];
	}

	display_buf[8] = 0x00;
	wm_sprintf(g_traceBuf, "SPEED: %s\r\n", display_buf);
	DumpMessageUSB(g_traceBuf, 1);
}


/** @brief Display Num Waypoint USB
 *
 * @par
 * Send the number of waypoints over the USB interface. Used for debug and for the PC loader.
 * @return void
 */
void DisplayNumWaypointsUSB(void)
{
	wm_sprintf(g_traceBuf, "%d\r\n", number_of_waypoints);
	DumpMessageUSB(g_traceBuf, 1);
}


/** @brief Dump Waypoint USB
 *
 * @par
 * Go through each waypoint and print it out over the USB interface. This is used for
 * the PC laoder to read the waypoint data out of the device.
 * @return void
 */
static unsigned int waypoint_cnt = 0;

void DumpWaypointsUSB(void)
{
	wm_sprintf(g_traceBuf, "Waypoint cnt = %d\r\n", waypoint_cnt);
	DumpMessage(g_traceBuf);

	DumpWaypointsUSBTimer();
}


/** @brief Function to dump the waypoint data over the USB interface.
 *  @par
 * Go through each waypoint and print it out over the USB interface. This is used for
 * the PC laoder to read the waypoint data out of the device.
 * @param timerid
 * @param context
 * @return void
 */
static void DumpWaypointsUSBTimer(void)
{
	int j;
	WAYPOINT tmp_wp;
	int big_loop_cnt = 0;
	unsigned int speed_cog;

	wm_sprintf(g_traceBuf, "DumpWaypointsUSB Timer: %d\r\n", waypoint_cnt);
	DumpMessage(g_traceBuf);

	for (big_loop_cnt = 0; big_loop_cnt < 500; big_loop_cnt++)
	{
		if (ReadWaypoint(&tmp_wp, waypoint_cnt) == SUCCESS)
		{
			for (j = 0; j < 8; j++)
			{
				g_traceBuf[j] = tmp_wp.utc[j];
			}

			g_traceBuf[8] = ',';

			for (j = 9; j < 17; j++)
			{
				g_traceBuf[j] = tmp_wp.lat[j - 9];
			}

			g_traceBuf[17] = ',';

			for (j = 18; j < 26; j++)
			{
				g_traceBuf[j] = tmp_wp.longi[j - 18];
			}

			g_traceBuf[26] = ',';

			for (j = 27; j < 29; j++)
			{
				g_traceBuf[j] = tmp_wp.epe[j - 27];
			}

			g_traceBuf[29] = ',';

			speed_cog = hex_ascii_2_uint((UINT8 *)tmp_wp.speed, 32);
			wm_itohexa(&g_traceBuf[30], speed_cog >> 23, 3);
			g_traceBuf[33] = ',';

			if (speed_cog & 0x400000)
			{
				g_traceBuf[34] = '1';
			}
			else
			{
				g_traceBuf[34] = '0';
			}

			g_traceBuf[35] = ',';

			if (speed_cog & 0x200000)
			{
				g_traceBuf[36] = '1';
			}
			else
			{
				g_traceBuf[36] = '0';
			}

			g_traceBuf[37] = ',';

			wm_itohexa(&g_traceBuf[38], speed_cog & 0x3fffff, 6);

			g_traceBuf[44] = '\r';
			g_traceBuf[45] = '\n';
			g_traceBuf[46] = 0x00;
			DumpMessageUSB(g_traceBuf, 1);
		}
		else
		{
			wm_sprintf(g_traceBuf, "Error reading waypoint: %d!\r\n", waypoint_cnt);
			DumpMessage(g_traceBuf);
		}

		if (waypoint_cnt < number_of_waypoints)
		{
			waypoint_cnt++;
		}
		else
		{
			waypoint_cnt = 0;
			return;
		}
	}
}


/*F***************************************************************
 *
 *   NAME:    void WriteWaypointFromFix( void)
 */

/*! \brief   Write a waypoint from a gps fix. send to flash.
 *
 *   \note
 *
 *//*
 *
 * CHANGES :
 * REF NO  DATE     WHO      DETAIL
 *         05JUL09  AB	    Initial version
 *F*/
void WriteWaypointFromFix(GPS_POS *fix, int speed, int course_over_ground, FIX_QUALITY fix_quality)
{
	WAYPOINT tmp_wp;
	int speed_cog = 0;

	memcpy(&tmp_wp.utc, fix->utc, 8);
	memcpy(&tmp_wp.lat, fix->lat, 8);
	memcpy(&tmp_wp.longi, fix->longi, 8);
	memcpy(&tmp_wp.epe, fix->epe, 2);

	// load speed and course over into speed_cog
	speed_cog = speed & 0x1fffff;

	if (fix_quality == FIX_3D)
	{
		speed_cog |= (1 << 22);      // fix quality occupies bit 22
	}
	if (get_agps_status() == AGPS_USED)
	{
		speed_cog |= (1 << 21);                     // agps occupies bit 21
	}
	speed_cog |= (course_over_ground << 23);        // course over ground occupies bits 31:23

	wm_itohexa((ascii *)&tmp_wp.speed, speed_cog, 8);

	WriteWaypoint(&tmp_wp);

	DisplayWaypoint(&tmp_wp);
}


/**@}*/
