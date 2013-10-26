/** @addtogroup OpenAT
 *@{*/

/*H************************************************************************
 */

/*! \file    adl_main.c
 *
 *   \brief   Main function. Initializes the WMP and starts timers and tasks.
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
 *         19Jun09  AndyB    Initial version.
 * NNNNNN  DDMMMYY  Name     Name of function/item changed
 *
 *H*/

#include "adl_global.h"
#include "CallbackMethods.h"
#include "SMShandling.h"
#include "XactUtilities.h"
#include "SimOperations.h"
#include "FlashTest.h"
#include "gpioTest.h"
#include "Timers.h"
#include "fence.h"
#include "CommandHandlerXACT.h"
#include "ConfigSettings.h"
#include "WaypointControl.h"
#include "PowerCtrl.h"
#include "status.h"
#include "GPRS.h"
#include "wip.h"
#include "diagnose.h"
#include "InternalFlash.h"
#include "Accelerometer.h"
#include "USB.h"
#include "GpsInterface.h"
#include "i2c.h"
#include "airplane_mode.h"
#include "protocol.h"
#include "Traces.h"
#include "GPSCtrl.h"

#include "TempoCasesTask.h"

DEBUG_TRACE_STORAGE;

/******************************************************************************
 *  Mandatory variables
 *****-------------------------------------------------------------------------
 *  these are needed for Open AT Stack operation
 *****************************************************************************/
u32 const adl_InitIRQLowLevelStackSize  = kibibytes_to_bytes(4);
u32 const adl_InitIRQHighLevelStackSize = kibibytes_to_bytes(4);

/******************************************************************************
 *  Globals
 *****************************************************************************/

char g_traceBuf[256]; // buffer for trace information
char g_IMEI[15];
char const CONFIG_STR[] = "AT+WHCNF=0,0";
const ascii adl_InitApplicationName[]    = "04.08";
const ascii adl_InitCompanyName[]        = "Tempo Cases";
const ascii adl_InitApplicationVersion[] = "2.03";

//Externs
extern AGPS_DATA g_AGPSData;


static void CheckResetReason(void)
{
	// Display the reason for last reboot.
	adl_InitType_e init_type = adl_InitGetType();
	switch (init_type)
	{
	case ADL_INIT_POWER_ON:
		sprintf(g_traceBuf, "\r\nADL_INIT_POWER_ON\r\n");
		break;

	case ADL_INIT_REBOOT_FROM_EXCEPTION:
		sprintf(g_traceBuf, "\r\nADL_INIT_REBOOT_FROM_EXCEPTION\r\n");
		g_status.RebootAlarm = 'Y';
		break;

	case ADL_INIT_DOWNLOAD_SUCCESS:
		sprintf(g_traceBuf, "\r\nADL_INIT_DOWNLOAD_SUCCESS\r\n");
		break;

	case ADL_INIT_DOWNLOAD_ERROR:
		sprintf(g_traceBuf, "\r\nADL_INIT_DOWNLOAD_ERROR\r\n");
		g_status.RebootAlarm = 'Y';
		break;

	case ADL_INIT_RTC:
		sprintf(g_traceBuf, "\r\nADL_INIT_RTC\r\n");
		break;
	}
	DumpMessage(g_traceBuf);
}


static void InitializeFlashLogs(void)
{
	int cnt;
    char const *FlashLogBlock1[] =
    {
        "SMS0", "SMS1", "SMS2"
    };                                                          //Handles for the flash
    char const *FlashLogBlock2[] =
    {
        "SMS3", "SMS4", "SMS5"
    };

	for (cnt = 0; cnt < LOG_MSG_PER_SEG; cnt++)
	{
		FlashSubscribe(FlashLogBlock1[cnt], MAX_NUM_FLASH_IDS);
		FlashSubscribe(FlashLogBlock2[cnt], MAX_NUM_FLASH_IDS);
	}
}

static bool WINDHandler(adl_atUnsolicited_t *paras)
{
	(void) paras;
	return false;
}

void ClearOnOffFF(u8 timerID, void *context)
{
	(void)timerID;
	(void)context;

	TurnOffFF();
}

static void CmdHandlerVersion(adl_atCmdPreParser_t *y)
{
	(void)y;

	wm_sprintf(g_traceBuf, "firmware version: %u.%02u\r\n", g_config.FirmwareRevNum >> 4, g_config.FirmwareRevNum & 0xf);
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf, 1);
}

/** @brief Initialize many AT commands
 *
 * @par
 * Main system init. Send startup AT commands and call the init functions for all the peripherals.
 * @param Id
 * @param Context
 * @return void
 */
static void InitFunction(u8 Id, void *Context)
{
	(void)Id;
	(void)Context;

	s32 result = 0;
	int factory_setup = 0;
	static adl_rtcTimeStamp_t CurrentTimeStamp;

	DumpMessage("Init function!\r\n");
	GetConvertTime(&CurrentTimeStamp);
	wm_sprintf(g_traceBuf,"CurrentTimeStamp : 0x%x\r\n",(unsigned int)CurrentTimeStamp.TimeStamp);
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf,1);

	// disable flash led
    {
        char const DISABLE_FLASH_LED[] = "AT+WHCNF=1, 0";
        result = adl_atCmdCreate((char *)DISABLE_FLASH_LED, FALSE, NULL, NULL);
        ASSERT(result == OK);
    }

	// Set the WMP100 to Fast Idle Mode (Default) if it is in a slow idle mode
    {
        char const FAST_IDLE_MODE[] = "AT+W32K=0";
        result = adl_atCmdCreate((char *)FAST_IDLE_MODE, FALSE, NULL, NULL);
        ASSERT(result == OK);
    }

	// enable 32khz clock
    {
        char const ENABLE_32KHZ_CLK[] = "AT+WHCNF=2,1";
        result = adl_atCmdCreate((char *)ENABLE_32KHZ_CLK, FALSE, NULL, NULL);
        ASSERT(result == OK);
    }

	// Erase all read messages
    
    {
        char const CLEAR_SMS_STR[] = "AT+CMGD=1,4";
        result = adl_atCmdCreate((char *)CLEAR_SMS_STR, FALSE, NULL, NULL);
        ASSERT(result == OK);
    }

    {
        char const FLOW_CTRL_STR[] = "AT+IFC=0,0";
        result = adl_atCmdCreate((char *)FLOW_CTRL_STR, ADL_AT_PORT_TYPE(ADL_PORT_UART2, FALSE), NULL, NULL);
        ASSERT(result == OK);
    }

	// Set the band string based on the setting in the flash.
	{
		char band_string[20];
		BAND_SETTING band_value = LoadBandValue();

		wm_sprintf(band_string, "AT+WMBS=%d, 0", band_value);
		DumpMessage(band_string);

		// Set to North American Band
		result = adl_atCmdCreate((char *)band_string, FALSE, NULL, NULL);
        ASSERT(result == OK);
	}

    {
        char const NETWORK_REG_STR[] = "AT+CREG=2";
        result = adl_atCmdCreate((char *)NETWORK_REG_STR, FALSE, NULL, NULL);
        ASSERT(result == OK);
    }

    {
        char const CCED_STR[] = "AT+CCED=1";
        result = adl_atCmdCreate((char *)CCED_STR, FALSE, NULL, NULL);
        ASSERT(result == OK);
    }

	//Disable automatic time update mode
	{
        char const CTZU_STR[] = "AT+CTZU=0";
        result = adl_atCmdCreate((char *)CTZU_STR, FALSE, NULL, NULL);
        ASSERT(result == OK);
		DumpMessage("Automatic time update disabled\r\n");
    }

	// determine if we need to load factory setup.
	if ((factory_setup = CheckFactorySetup()))
	{
        int i = 0;
		//Load the defaults
		load_defaults();
		DumpMessage("Start store to flash\r\n");
		StoreToFlash();
		DumpMessage("End store to flash\r\n");
		//Initialize fences to 0 and store to flash
		for (i = 0; i < NUM_FENCES; i++)
		{
			FencesToFlash(i);
		}
	}

	{
		unsigned char flash_data[] = { 0xFF };
		unsigned char const flash_success = 0x01;
        char const FactoryDefaultHandle[] = "FactoryDefaults";
		bool initialize = false;
		s8 flash_result = -1;

		flash_result = FlashSubscribe(FactoryDefaultHandle, 1);
        ASSERT(flash_result >= 0);

		if (FlashRead(FactoryDefaultHandle, 0, sizeof flash_data, flash_data) != OK)
		{
			DumpMessage("Didn't read valid flash handle\r\n");
			initialize = true;
		}
		else
		{
			char buffer[12];
			if (flash_data[0] != flash_success)
			{
				initialize = true;
			}
			else
			{
				initialize = false;
			}

			wm_sprintf(buffer, "returned %x\r\n", initialize);
			DumpMessage(buffer);
		}

		if (initialize)
		{
			DumpMessage("Setting up initial configuration\r\n");
			SetInitialConfiguration();
			FlashWrite(FactoryDefaultHandle, 0, sizeof flash_success, &flash_success);
		}
	}

	// load settings.
	LoadFromFlash();

	InitSpiFlash(factory_setup);

	// initialize the status structure.
	InitStatus();

	// setup the accelerometer.
	InitAccelerometer();

	// start the timers.
	startTimers();

	// initialize the USB interface.
	USBInterface();

	// initialize the power control.
	InitPowerCtrl();
	
	//Initialized Pin to 2100
    {
        char const PIN_CODE[] = "2100";
        result = adl_simSubscribe(evhSimHandler, (char *)PIN_CODE);
        ASSERT(result == OK);
    }

	//Find Registration
    {
        char const CREG_IND_STR[] = "+CREG:";
        char const CCED_IND_STR[] = "+CCED:";
        char const CSQ_IND_STR[] = "+CSQ:";
        char const WIND_IND_STR[] = "+WIND:";
        result = adl_atUnSoSubscribe((char *)WIND_IND_STR, WINDHandler);
        result = adl_atUnSoSubscribe((char *)CREG_IND_STR, CREGHandler);
        result = adl_atUnSoSubscribe((char *)CCED_IND_STR, CCEDHandler);
        result = adl_atUnSoSubscribe((char *)CSQ_IND_STR, CSQHandler);
    }

	result = 0;

	// set the version.
	g_config.FirmwareRevNum = FW_MINOR_VER | (FW_MAJOR_VER << 4);
	g_config.HardwareRevNum = HW_MINOR_VER | (HW_MAJOR_VER << 4);


	wm_sprintf(g_traceBuf, "firmware version: %x\r\n", g_config.FirmwareRevNum);
	DumpMessage(g_traceBuf);

	wm_sprintf(g_traceBuf, "size of config structure: %d\r\n", (int)sizeof (CONFIG));
	DumpMessage(g_traceBuf);

	//Subscribe to sms service
	sms_service_start();

	wip_netInitOpts(WIP_NET_OPT_DEBUG_PORT, WIP_NET_DEBUG_PORT_UART1,
	                WIP_NET_OPT_END);

	// initialize the GPS.
#if !defined(DISABLE_GPS)
	GpsInit();
#endif

	if (result == 0)
	{
		adl_atCmdSubscribe("AT&XACT", CmdHandlerXACT , ADL_CMD_TYPE_ACT 
                                                    | ADL_CMD_TYPE_TEST 
                                                    | ADL_CMD_TYPE_READ 
                                                    | ADL_CMD_TYPE_PARA | 0x00C1);

		adl_atCmdSubscribe("AT&DIAGNOSE", diagnose_handler , ADL_CMD_TYPE_ACT 
                                                    | ADL_CMD_TYPE_TEST 
                                                    | ADL_CMD_TYPE_READ 
                                                    | ADL_CMD_TYPE_PARA | 0x00C1);

		adl_atCmdSubscribe("AT&VERSION", CmdHandlerVersion, ADL_CMD_TYPE_ACT);
	}

	//Display the timeout params
	wm_sprintf(g_traceBuf,"GPSTimeout:%d\r\nAGPS Timeout:%d\r\n,PowerOff Timeout:%d\r\n",g_ModeConfig.GPSTimeout,g_AGPSData.WaitTimeout,g_AGPSData.PowerOffTimeout);
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf,1);



	// turn on the watchdog timer.
	adl_wdActiveAppWd(ADL_TMR_S_TO_TICK(30));
}

/** @brief WHCNF Handler
 *
 * @par
 * Wait for WHCNF OK before continuing with initialization procedure.
 * @param paras
 * @return false
 */
static bool WHCNFHandler(adl_atResponse_t *paras)
{
	if (paras->RspID == ADL_STR_OK)
	{
        adl_tmr_t * init_tmr;
		adl_ctxSleep(1);
		DumpMessage("WHCNFHandler OK\r\n");
        
		init_tmr = adl_tmrSubscribe(FALSE, 1, ADL_TMR_TYPE_100MS, InitFunction);
        ASSERT(init_tmr != NULL);
	}
	else
	{
		// Set configuration for GPIOs
        s32 result = adl_atCmdCreate((char *)CONFIG_STR, FALSE, WHCNFHandler, "*", NULL);
        ASSERT(result == OK);
	}

	return FALSE;
}

/** @brief IMEI Handler
 *
 * @par
 * Copy the IMEI number into the global structure.
 * @param paras
 *
 * @return true
 */
static bool IMEIHandler(adl_atResponse_t *paras)
{
	DumpMessage("IMEIHandler\r\n");
	adl_ctxSleep(1);
	if (paras->StrLength >= 17)
	{
		memcpy(g_IMEI, &paras->StrData[2], 15);
		DumpMessage(paras->StrData);
	}

	return true;
}

#if defined(BAREBONES_APPLICATION)
void MainTask(void)
{
	s32 result = adl_atCmdCreate("AT+WMFM=0,1,1", FALSE, NULL, NULL);
    ASSERT(result == OK);

	adl_tmrSubscribe(FALSE, 90, ADL_TMR_TYPE_100MS, ClearOnOffFF);
	InitializeUnusedGpio();
    startTimers();
	InitPowerCtrl();
}
#else
void MainTask(void)
{
	s32 result = 0;
	//Application version number

	adl_tmrSubscribe(FALSE, 90, ADL_TMR_TYPE_100MS, ClearOnOffFF);

	// enable the serial ports
	result = adl_atCmdCreate("AT+WMFM=0,1,1", FALSE, NULL, NULL);
    ASSERT(result == OK);

	result = adl_atCmdCreate("AT+WMFM=0,1,2", FALSE, NULL, NULL);
    ASSERT(result == OK);

	result = adl_atCmdCreate("AT+WMFM=0,1,3", FALSE, NULL, NULL);
    ASSERT(result == OK);

	adl_ctxSleep(1);

	// read in the IMEI
    {
        char const CGSN_STR[] = "AT+CGSN";
        result = adl_atCmdCreate((char *)CGSN_STR, FALSE, IMEIHandler, "*", NULL);
        ASSERT(result == OK);
    }

	// make sure we are in class B mode.
    {
        char const CLASS_B_STR[] = "AT+CGCLASS=B";
        result = adl_atCmdCreate((char *)CLASS_B_STR, FALSE, NULL, NULL);
        ASSERT(result == OK);
    }

	InitializeUnusedGpio();

	CheckResetReason();

	wm_sprintf(g_traceBuf, "\r\nXACT Open AT Embedded Application %2d.%02d (%s %s)\r\n",
                        FW_MAJOR_VER, FW_MINOR_VER, __DATE__, __TIME__);
	DumpMessage(g_traceBuf);
	
	wm_sprintf(g_traceBuf, "adl_InitApplicationName: %s\r\n", adl_InitApplicationName);
	DumpMessage(g_traceBuf);

	InitializeFlashLogs();

	// Set configuration for GPIOs
	result = adl_atCmdCreate((char *)CONFIG_STR, FALSE, WHCNFHandler, "*", NULL);
    ASSERT(result == OK);
}
#endif



adl_InitTasks_t const adl_InitTasks[] =
{
    { MainTask,		  kibibytes_to_bytes(12), "main", 2 },
    { TempoCasesTask, kibibytes_to_bytes(3),  "TCT",  1 },
    { NULL, 0, NULL, 0 } 
};

/*@}*/
