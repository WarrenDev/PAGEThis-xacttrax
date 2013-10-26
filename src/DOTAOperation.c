/** @addtogroup OpenAT
 *@{ */
/*H****************************************************************************/

/** @file DOTAOperation.c
 *
 * @brief This file contains the APIs used to for updating to new
 * versions of the Xacttrax Project application.
 */
#include "adl_global.h"
#include "XactUtilities.h"
#include "USB.h"
#include "DOTAOperation.h"
#include "unistd.h"

char        *gFileBuffer = NULL;
int gFileSize = 0;
extern int g_DOTAOK;
static int gFileBufferOffset = 0;
static s32 gDAFileCellHandle = 0;
static s32 gADEventHandler;
static int gUSBTransferFlag = 0;
adl_tmr_t   *gUSBTimeout;

int gFileSizeDownloaded = 0;
volatile int gFileCellReady = 0;            // flag to indicate if DA write can be allowed.
// 0=not ready, 1=ready

extern s32 gUSBApplicationUpdateFlag;
extern eFlowCmdState g_USBFlowCmdState;
extern int g_DOTAUpdateMode;

void CompleteCRCCalc(char *gFileBuffer, int gFileSizeDownloaded);

void evh_ADmem(adl_adEvent_e ADevent, u32 ADProgress);

int CreateADFile(void);

static void FinishPCUpdate(void)
{
	DumpMessage("Finish update\r\n");
	adl_ctxSleep(ADL_TMR_MS_TO_TICK(5000));
	CompleteADUpdate();
}


/** @brief Record the AD File Size
 *
 * @par
 * Tell us how big the file size is going to be.
 * @return void
 */
int recordFileSize(int FileSize)
{
	gFileSize = FileSize;

	if (gUSBApplicationUpdateFlag == 0)
	{
		gFileBuffer = (char *)malloc(FileSize);
		gFileBufferOffset = 0;
	}

	return 0;
}


/** @brief Record the AD File Size OTA
 *
 * @par
 * Tell us how big the file size when doing this over the air.
 * @return void
 */
void recordFileSizeOTA(int FileSize)
{
	gFileSize = FileSize;
}


/** @brief Start DOTA operation
 *
 * @par
 * This method will subscript to the AD Events and start
 * the AD format operation.
 *
 * @return void
 */
void StartDOTA()
{
	s32 sReturn;

#ifndef BOOTSTRAP_CODE
	D_TRACE("inside StartDOTA \n");
	do
	{
		// we want to subscribe to A/D memory events
		gADEventHandler = adl_adEventSubscribe(evh_ADmem);
		if (gADEventHandler < 0)
		{
			DisplayErrorCode("adl_adEventSubscribe(evh_ADmem)", __FILE__, __LINE__, gADEventHandler);
			break;
		}

		sReturn = adl_adFormat(gADEventHandler);
		if (sReturn < 0)
		{
			DisplayErrorCode("adl_adFormat( ADEventHandle )", __FILE__, __LINE__, sReturn);
			break;
		}
	} while (0);
#endif
}


/** @brief AD memory Event Handler
 *
 * @par This is the event handler for all AD progresses events
 *
 * @param ADevent
 * @param ADProgress
 * @return void
 */
void evh_ADmem(adl_adEvent_e ADevent, u32 ADProgress)
{
	(void)ADProgress;

	s32 sReturn = 0;

	switch (ADevent)
	{
	case ADL_AD_EVENT_RECOMPACT_INIT:
	case ADL_AD_EVENT_RECOMPACT_PROGRESS:
		DumpMessage("evh admem progress\r\n");
		break;

	case ADL_AD_EVENT_FORMAT_DONE:
		DumpMessage("format done\r\n");
		D_TRACE("  *S*  Format Done!...will now recompact. \n");
		sReturn = adl_adRecompact(gADEventHandler);
		if (sReturn < 0)
		{
			DisplayErrorCode("adl_adRecompact(gADEventHandler)", __FILE__, __LINE__, sReturn);
		}
		break;

	case ADL_AD_EVENT_RECOMPACT_DONE:
		D_TRACE(" *S*  Recompact Done! will now create AD filenamen... \n");
		CreateADFile();
		//DumpMessage("ready to recieve file transmission \n");

		//	    SEND_OK_USB();
		g_DOTAOK = 1;
		ChangeV24State(USB_COM_ENUM, ADL_FCM_V24_STATE_DATA);

		break;

	case ADL_AD_EVENT_INSTALL:
		// This code should be reached but the CPU will reset before this trace
		D_TRACE(" *S*  App install called. ADL_AD_EVENT triggered. \n");
		break;

	case ADL_AD_EVENT_FORMAT_INIT:
		D_TRACE("ADL_AD_EVENT_FORMAT_INIT\n");
		break;

	case ADL_AD_EVENT_FORMAT_PROGRESS:
		D_TRACE("ADL_AD_EVENT_FORMAT_PROGRESS\n");
		break;

	default:
		DumpMessage("evh admem error\r\n");
		D_TRACE("UNHANDLED EVENT\n");
		// check for error cases here
	}
}


/** @brief Create AD File
 *
 * @par
 * Create an AD file to store updated application to.
 *
 * @return 0
 */
int CreateADFile()
{
	if (gFileSize > 0)
	{
		gDAFileCellHandle = adl_adSubscribe(1, gFileSize);

		if ((gDAFileCellHandle == ADL_RET_ERR_ALREADY_SUBSCRIBED) || (gDAFileCellHandle == 0))
		{
			D_TRACE(" gDAFileCellHandle ready\n");
			gFileCellReady = 1;
			gFileSizeDownloaded = 0;
		}
		else
		{
			DisplayErrorCode("adl_adSubscribe", __FILE__, __LINE__, gDAFileCellHandle);
		}
	}
	else
	{
		D_TRACE("gFileSize not valid, no AD File Created\n");
		gFileCellReady = -1;
	}

	return 0;
}


/** @brief Check Transfer
 *
 * @par
 * Call this in the event of a USB tranfer failure.
 * @param timerID
 * @param context
 * @return void
 */
void CheckTransfer(u16 timerID, void *context)
{
	(void)timerID;
	(void)context;

	D_TRACE("Error: Failed to transfer the file\n");
	gUSBApplicationUpdateFlag = 0;
	ChangeV24State(USB_COM_ENUM, ADL_FCM_V24_STATE_AT);
}


/** @brief AD Write Data
 *
 * @par write memory to the AD memory
 *
 * @param DataSize
 * @param Data
 * @return sReturn 0 on sucess, -1 on failure
 */
int ADWriteData(u16 DataSize, u8 *Data)
{
	s32 sReturn = -1;

	// in this code  gUSBApplicationUpdateFlag is always 0
	if (gUSBApplicationUpdateFlag == 0)
	{
		if (gFileCellReady == 0)
		{
			D_TRACE("preparing AD File for new application, please wait\n\n");
		}
		else
		{
			D_TRACE(". %d", gFileSizeDownloaded);
		}
		while (gFileCellReady == 0)
		{
			D_TRACE("+");
			adl_ctxSleep(1);
		}
		if (gFileCellReady < 0)
		{
			D_TRACE("ERROR with AD Write operaiton \n");
		}
		else
		{
			sReturn = adl_adWrite(gDAFileCellHandle, DataSize, Data);
			if (sReturn < 0)
			{
				DisplayErrorCode("adl_adWrite", __FILE__, __LINE__, sReturn);
			}
			gFileSizeDownloaded += DataSize;

			if (gFileSizeDownloaded == gFileSize)
			{
				// we're done
				wm_sprintf(g_traceBuf, " Data loadloaded %d \n", gFileSizeDownloaded);
				DumpMessage(g_traceBuf);
				if (!g_DOTAUpdateMode)
				{
					ChangeV24State(USB_COM_ENUM, ADL_FCM_V24_STATE_AT);
					SEND_OK_USB();
				    DumpMessage("OK Sent\r\n");	
					FinishPCUpdate();
				}
			}

			if (gFileSizeDownloaded > gFileSize)
			{
				// we're done
				D_TRACE(" ERROR..overload of Data %d \n", gFileSizeDownloaded - gFileSize);
				if (!g_DOTAUpdateMode)
				{
					SEND_OK_USB();
					ChangeV24State(USB_COM_ENUM, ADL_FCM_V24_STATE_AT);
				}
			}
		}
	}
	else
	{
		if (gUSBTransferFlag == 0)
		{
			gUSBTimeout = adl_tmrSubscribe(FALSE, 2000, ADL_TMR_TYPE_100MS, (adl_tmrHandler_t)CheckTransfer);
			gUSBTransferFlag++;
		}
		memcpy(gFileBuffer + gFileBufferOffset, Data, DataSize);
		gFileBufferOffset += DataSize;

		gFileSizeDownloaded += DataSize;

		if (gFileSizeDownloaded == gFileSize)
		{
			// we're done
			gUSBTransferFlag = 0;
			adl_tmrUnSubscribe(gUSBTimeout, (adl_tmrHandler_t)CheckTransfer, ADL_TMR_TYPE_100MS);
			D_TRACE(" Data loadloaded %d \n", gFileSizeDownloaded);
			ChangeV24State(USB_COM_ENUM, ADL_FCM_V24_STATE_AT);

			CompleteCRCCalc(gFileBuffer, gFileSizeDownloaded);
			// AB CRITICAL : added include.
			sReturn = write(gUSBApplicationUpdateFlag, gFileBuffer, gFileSizeDownloaded);          //TVDP: CRITICAL If this is a valid function call (write) the include is missing
			if (sReturn < 0)
			{
				DisplayErrorCode("fs_write", __FILE__, __LINE__, sReturn);
			}

			free(gFileBuffer);
			gFileBuffer = NULL;

			// we don't have filesystem support. what is going to happen with this?
			gFileSizeDownloaded = 0;
			gUSBApplicationUpdateFlag = 0;
			D_TRACE("file closed\n");
			SEND_OK_USB();
		}

		if (gFileSizeDownloaded > gFileSize)
		{
			// we're done
			D_TRACE(" ERROR..overload of Data %d \n", gFileSizeDownloaded - gFileSize);
			adl_tmrUnSubscribe(gUSBTimeout, (adl_tmrHandler_t)CheckTransfer, ADL_TMR_TYPE_100MS);

			gFileSizeDownloaded = 0;
			SEND_OK_USB();
			ChangeV24State(USB_COM_ENUM, ADL_FCM_V24_STATE_AT);
		}
	}
	return 0;
}


static int crc_tab32_init = FALSE;
static unsigned long crc_tab32[256];
#define                 P_32    0xEDB88320L

/** @brief Initalize memory for CRC calculations
 *
 * @return void
 */
static void init_crc32_tab(void)
{
	int i, j;
	unsigned long crc;

	for (i = 0; i < 256; i++)
	{
		crc = (unsigned long)i;

		for (j = 0; j < 8; j++)
		{
			if (crc & 0x00000001L)
			{
				crc = (crc >> 1) ^ P_32;
			}
			else
			{
				crc = crc >> 1;
			}
		}

		crc_tab32[i] = crc;
	}

	crc_tab32_init = TRUE;
}  /* init_crc32_tab */


/** @brief update CRC32 calculation
 *
 * @par
 * This method will update the CRC calcuation based on added
 * a new char member.
 *
 * @param crc
 * @param c
 * @return crc
 */
unsigned long update_crc_32(unsigned long crc, char c)
{
	unsigned long tmp, long_c;

	long_c = 0x000000ffL & (unsigned long)c;

	if (!crc_tab32_init)
	{
		init_crc32_tab();
	}

	tmp = crc ^ long_c;
	crc = (crc >> 8) ^ crc_tab32[tmp & 0xff];

	return crc;
}  /* update_crc_32 */


/** @brief Complete CRC Calculation
 *
 * @par This method will complete a CRC32 calculation on
 * the memory block. This is used to ensure good data integrity when
 * updating the firmware.
 *
 * @param gFileBuffer
 * @param gFileSizeDownloaded
 * @return void
 */
void CompleteCRCCalc(char *gFileBuffer, int gFileSizeDownloaded)
{
	unsigned long crc_32;
	int i = 0;
	crc_32 = 0xffffffffL;
	crc_tab32_init = FALSE;
	char Info[64];
	for (i = 0; i < gFileSizeDownloaded; i++)
	{
		crc_32 = update_crc_32(crc_32, gFileBuffer[i]);
	}

	sprintf(Info, "Size:%d CRC: %lx \n", gFileSizeDownloaded, crc_32);
	adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_USB
	                       , Info);
}


/** @brief Complete AD Update
 *
 * @par This method will complete the ADL Update operation
 * it first performs an AD Finalize of the AD memory, then
 * creates a new install point for the update memory to be
 * stored at.
 *
 * @return 0
 */
int CompleteADUpdate()
{
	s32 sReturn = -1;
	DumpMessage("Completing AD update\r\n");

	do
	{
		sReturn = adl_adFinalise(gDAFileCellHandle);
		if (sReturn < 0)
		{
			DisplayErrorCode("adl_adFinalise", __FILE__, __LINE__, sReturn);
			break;
		}

		sReturn = adl_adInstall(gDAFileCellHandle);
		if (sReturn < 0)
		{
			DisplayErrorCode("adl_adInstall", __FILE__, __LINE__, sReturn);
			break;
		}
	} while (0);

	return 0;
}


/**@}*/
