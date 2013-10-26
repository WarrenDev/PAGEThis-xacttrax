/** @addtogroup TestCode
 *@{*/
#include <adl_global.h>

#include "CommandHandlerXACT.h"

#include "common.h"
#include "XactUtilities.h"
#include "ConfigSettings.h"
#include "InternalFlash.h"
#include "WaypointControl.h"
#include "GpsInterface.h"
#include "PowerCtrl.h"
#include "status.h"
#include "protocol.h"
#include "GPRS.h"
#include "anolclient.h"
#include "SMShandling.h"
#include "gpioTest.h"
#include "GPSCtrl.h"
#include "Accelerometer.h"
#include "ota_at.h"
#include "dogwalk.h"
#include "button.h"

#if defined(SST_FLASH)
#include "sst_flashDriver.h"
#define SECTOR_SIZE    1
#endif

#if defined(ATMEL_FLASH)
#include "atmel_flashDriver.h"
#endif

DEBUG_TRACE_STORAGE;

#define RX_BUFFSIZE    (256)
extern UINT8 g_rx[RX_BUFFSIZE];

extern UINT32 g_WaypointInterval;

extern volatile s32 g_VibrationPattern;
extern struct structVTGInformation gVTGInfo;

extern int g_GPSTestMode;

extern int g_ShutDownReq;

struct structGGAInformation gGGAInfoTest;
struct structVTGInformation gVTGInfoTest;

extern ACCEL_ERROR_STATE accel_error_state;

int ForceRoamMode = 0;

char tmp_str[100];

#define CFG_FIRST_PKT_SIZE      65
#define CFG_SECOND_PKT_SIZE     89

/** @brief Process commands of the form at&xact
 *
 * @param Cmd
 * @return void
 */
void CmdHandlerXACT(adl_atCmdPreParser_t *Cmd)
{
	char fence_res;
	WAYPOINT test_waypoint;
	UINT8 tmp8;
	UINT32 tmp;
	int fence_num;
	TCP_STATUS tcp_stat;
	ascii IpAddr[15];               /*IP address in ascii*/
	char tmp_nmea_string[100];
	NMEA_ERRORS nm;
	double SpeedIn;
	int SpeedIn1000;
	BAND_SETTING bandsetting;
	char            *apn_setting;
	//    int otaat_val;
	switch (Cmd->Type)
	{
	case ADL_CMD_TYPE_TEST:
		DumpMessage("\r\nAT&XACT=? is invalid! \r\n");
		break;

	case ADL_CMD_TYPE_READ:
		DumpMessage("\r\n Syntax:\r\n");
		DumpMessage("         AT&XACT=1                   -- Enable button press\r\n");
		DumpMessage("         AT&XACT=2,<type>            -- Send an SMS Message \r\n");
		DumpMessage("                                       Type: L - Log Packet\r\n");
		DumpMessage("                                             S - Status Packet\r\n");
		DumpMessage("         AT&XACT=3                   -- Initialize Fences to test fix \r\n");
		DumpMessage("         AT&XACT=4,<long32>,<lat32>  -- Test a coordinate \r\n");
		DumpMessage("         AT&XACT=5                   -- Initialize the flash objects \r\n");
		DumpMessage("         AT&XACT=6                   -- Load the values from flash \r\n");
		DumpMessage("         AT&XACT=7,<fnum>,<incexc>,<enable>,<long32>,...   --Write a fence into the flash \r\n");
		DumpMessage("         AT&XACT=8                   -- Display fences stores in flash \r\n");
		DumpMessage("         AT&XACT=9,<testnum>,<level in hex> -- Test one of the GPIO leds\r\n");
		DumpMessage("         AT&XACT=10                  -- Retrieve the RTC value \r\n");
		DumpMessage("         AT&XACT=11                  -- Dump The last known GPS location from flash \r\n");
		break;

	case ADL_CMD_TYPE_ACT:
		DumpMessage("\r\nAT&XACT is invalid! \r\n");
		break;

	case ADL_CMD_TYPE_PARA:

#if defined(SST_FLASH) && defined(REGRESSION_TESTING)
		{
			typedef enum
			{
				zero_buffer,
				b_buffer,
				inc_buffer,
				number_of_buffers
			} buffer_t;

			unsigned char test_buffers[number_of_buffers][PAGE_SIZE];

			if (strncmp("71", ADL_GET_PARAM(Cmd, 0), 2) == 0)
			{
				memset(test_buffers[zero_buffer], 0, PAGE_SIZE);
				memset(test_buffers[b_buffer], 0xb, PAGE_SIZE);

				{
					unsigned int i;
					for (i = 0; i < PAGE_SIZE; i++)
					{
						test_buffers[inc_buffer][i] = (unsigned char)i;
					}
				}
				adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_UART1, "Finished Initializing buffers\r\n");
			}

			if (strncmp("70", ADL_GET_PARAM(Cmd, 0), 2) == 0)
			{
				DumpMessage("Flash test\r\n");

				unsigned int page = 0;
				buffer_t current_buffer;

				for (current_buffer = zero_buffer; current_buffer < number_of_buffers; current_buffer++)
				{
				}
				{
					unsigned char const *current_test = test_buffers[current_buffer];
					for (page = 0; page < (NUMBER_OF_PAGES); page++)
					{
						if (page % 100 == 0)
						{
							wm_sprintf(g_traceBuf, "Page %d...\r\n", page);
							DumpMessage(g_traceBuf);
							adl_ctxSleep(1);
						}

						INSIST(sf_writePage(page, current_test) == SUCCESS);

						unsigned char buffer[PAGE_SIZE];
						INSIST(sf_readPage(page, buffer) == SUCCESS);

						if (memcmp(current_test, buffer, PAGE_SIZE) != 0)
						{
							DumpMessage("MEMCMP faile\r\n");
							ASSERT(false);
						}
					}
					adl_atSendResponsePort(ADL_AT_UNS, ADL_PORT_UART1, "Finished buffer test\r\n");
				}
			}
		}

		/* Test the Waypoint code */

		{
#define NUMBER_OF_TESTS                 kibibytes_to_bytes(1)
#define NUMBER_OF_WAYPOINTS_PER_TEST    8
			if (strncmp("72", ADL_GET_PARAM(Cmd, 0), 2) == 0)
			{
				DumpMessage("WAYPOIN TEST\r\n");
				int i;

				for (i = 0; i < NUMBER_OF_TESTS; i++)
				{
					int j;
					WAYPOINT test_waypoint[NUMBER_OF_WAYPOINTS_PER_TEST];

					for (j = 0; j < NUMBER_OF_WAYPOINTS_PER_TEST; j++)
					{
						adl_rtcTime_t curr_time;
						adl_rtcTimeStamp_t timestamp;
						s32 result = adl_rtcGetTime(&curr_time);
						ASSERT(result == OK);

						result = adl_rtcConvertTime(&curr_time, &timestamp, ADL_RTC_CONVERT_TO_TIMESTAMP);
						ASSERT(result == OK);

						{
							snprintf(RECAST(char *, test_waypoint[j].utc), sizeof test_waypoint[j].utc, "%8x", RECAST(unsigned int, timestamp.TimeStamp));
							snprintf(RECAST(char *, test_waypoint[j].lat), sizeof test_waypoint[j].lat, "%8x", RECAST(unsigned int, timestamp.SecondFracPart));
							snprintf(RECAST(char *, test_waypoint[j].longi), sizeof test_waypoint[j].longi, "%8x", RECAST(unsigned int, timestamp.TimeStamp * (i + j)));
							snprintf(RECAST(char *, test_waypoint[j].speed), sizeof test_waypoint[j].speed, "%8x", RECAST(unsigned int, timestamp.SecondFracPart * (i + j)));
							snprintf(RECAST(char *, test_waypoint[j].epe), sizeof test_waypoint[j].epe, "%2x", i);

							adl_ctxSleep(1);
							{
								WAYPOINT verify_waypoint;
								WriteWaypoint(&test_waypoint[j]);

								adl_ctxSleep(1);

								ReadWaypoint(&verify_waypoint, -1);
								ASSERT(memcmp(&test_waypoint[j], &verify_waypoint, sizeof (WAYPOINT)) == 0);
							}
						}

						adl_ctxSleep(1);
					}

					for (j = 0; j < NUMBER_OF_WAYPOINTS_PER_TEST; j++)
					{
						WAYPOINT verify_waypoint;
						ReadWaypoint(&verify_waypoint, j - NUMBER_OF_WAYPOINTS_PER_TEST);
						adl_ctxSleep(1);

						ASSERT(memcmp(&test_waypoint[j], &verify_waypoint, sizeof (WAYPOINT)) == 0);
						adl_ctxSleep(1);
					}
				}
			}
		}
#endif

		if (strncmp("91", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
#if defined(ATMEL_FLASH)
			DumpMessage("Atmel Flash\r\n");
#endif
#if defined(SST_FLASH)
			DumpMessage("SST Flash\r\n");
#endif
		}

		if (strncmp("96", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			ClrSWButtonOvrd();
		}

		if (strncmp("95", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			SetSWButtonOvrd();
		}

		if (strncmp("90", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			SetStartupTrackingMode(0, FALSE);
			break;
		}

		if (strncmp("89", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			if (InternalFlash_WriteRebootAlarm('X') != 0)
			{
				DumpMessage("Error could nto write reboot alarm\r\n");
			}
			else
			{
				DumpMessage("Success write reboot alarm\r\n");
			}

			wm_sprintf(g_traceBuf, "Read reboot alarm: %c\r\n", InternalFlash_ReadRebootAlarm());
			DumpMessage(g_traceBuf);
			break;
		}

		if (strncmp("88", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			g_config.SMSorGPRS = 1;
			break;
		}

		if (strncmp("85", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			g_config.BreadCrumbMode = 1;
			break;
		}

		if (strncmp("84", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			SetAccelerometerDurSleep((UINT8 *)ADL_GET_PARAM(Cmd, 1), &g_config);
			break;
		}

		if (strncmp("82", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			InitSpiFlash(1);
			break;
		}

		if (strncmp("81", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			if (strncmp("1", ADL_GET_PARAM(Cmd, 1), 1) == 0)
			{
				OTAAT_eval("at&diagnose=AC,LED1,20");
			}
			if (strncmp("2", ADL_GET_PARAM(Cmd, 1), 1) == 0)
			{
				OTAAT_eval("at&diagnose=PB,LED1");
			}

			break;
		}

		if (strncmp("80", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			adl_tmrSubscribe(FALSE, 20, ADL_TMR_TYPE_100MS, (adl_tmrHandler_t)(test_otaat_tmr));
			//      sscanf(ADL_GET_PARAM(Cmd,1),"%d",&otaat_val);
			//	      test_otaat(otaat_val);
			break;
		}

		if (strncmp("73", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
#ifdef SST_FLASH
			spiFlash_EraseChip();
			break;
#endif
		}

#if defined(ATMEL_FLASH)
		if (strncmp("70", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			spi_EraseChip();
			DumpMessage("Erase done\r\n");
			break;
		}
#endif

		if (strncmp("69", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			wm_sprintf(g_traceBuf, "USB connected = %d\r\n", IsUSBConnected());
			DumpMessage(g_traceBuf);
			break;
		}

		if (strncmp("68", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			// sleep for 5 seconds to allow the pc app to shutdown.
			Status_setResetReason(RESET_REASON_SOFTWARE_UPDATE);
			start_reset_timer();
			break;
		}

		if (strncmp("67", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			if (InternalFlash_WriteAPNPassword(ADL_GET_PARAM(Cmd, 1)) == 0)
			{
				wm_sprintf(g_traceBuf, "\r\nOK\r\n");
			}
			else
			{
				wm_sprintf(g_traceBuf, "\r\nERROR\r\n");
			}
			DumpMessage(g_traceBuf);
			DumpMessageUSB(g_traceBuf, 1);
			break;
		}

		if (strncmp("66", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			if (InternalFlash_WriteAPNLogin(ADL_GET_PARAM(Cmd, 1)) == 0)
			{
				wm_sprintf(g_traceBuf, "\r\nOK\r\n");
			}
			else
			{
				wm_sprintf(g_traceBuf, "\r\nERROR\r\n");
			}
			DumpMessage(g_traceBuf);
			DumpMessageUSB(g_traceBuf, 1);
			break;
		}

		if (strncmp("65", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			apn_setting = InternalFlash_GetAPNName();
			if (apn_setting == NULL)
			{
				wm_sprintf(g_traceBuf, "APN Name=\r\n");
			}
			else
			{
				wm_sprintf(g_traceBuf, "APN Name = %s\r\n", apn_setting);
			}
			DumpMessage(g_traceBuf);
			DumpMessageUSB(g_traceBuf, 1);

			apn_setting = InternalFlash_GetAPNLogin();
			if (apn_setting == NULL)
			{
				DumpMessage("Null login!\r\n");
				wm_sprintf(g_traceBuf, "APN Login =\r\n");
			}
			else
			{
				wm_sprintf(g_traceBuf, "APN Login = %s\r\n", apn_setting);
			}
			DumpMessage(g_traceBuf);
			DumpMessageUSB(g_traceBuf, 1);

			apn_setting = InternalFlash_GetAPNPassword();
			if (apn_setting == NULL)
			{
				wm_sprintf(g_traceBuf, "APN Password =\r\n");
			}
			else
			{
				wm_sprintf(g_traceBuf, "APN Password = %s\r\n", apn_setting);
			}
			DumpMessage(g_traceBuf);
			DumpMessageUSB(g_traceBuf, 1);

			wm_sprintf(g_traceBuf, "\r\nOK\r\n");
			DumpMessage(g_traceBuf);
			DumpMessageUSB(g_traceBuf, 1);
			break;
		}

		if (strncmp("64", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			if (InternalFlash_WriteAPNName(ADL_GET_PARAM(Cmd, 1)) == 0)
			{
				wm_sprintf(g_traceBuf, "\r\nOK\r\n");
			}
			else
			{
				wm_sprintf(g_traceBuf, "\r\nERROR\r\n");
			}
			DumpMessage(g_traceBuf);
			DumpMessageUSB(g_traceBuf, 1);
			break;
		}

		if (strncmp("63", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			wm_sprintf(g_traceBuf, "Band Setting: %d\r\n", LoadBandValue());
			DumpMessage(g_traceBuf);
			DumpMessageUSB(g_traceBuf, 1);
			DumpMessageUSB("\r\nOK\r\n", 1);
			break;
		}

		if (strncmp("62", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			bandsetting = *((char *)ADL_GET_PARAM(Cmd, 1)) - 0x30;
			WriteBandValue(bandsetting);
			DumpMessage("\r\nOK\r\n");
			DumpMessageUSB("\r\nOK\r\n", 1);
			break;
		}

		if (strncmp("60", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			AccelSimulateLockUp();
			break;
		}

		if (strncmp("59", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			wm_sprintf(g_traceBuf, "Bad Threshold: %d\r\nIRQ High Slow Idle: %d\r\n", accel_error_state.threshold_bad, accel_error_state.irq_high_slow_idle);
			DumpMessage(g_traceBuf);
			DumpMessageUSB(g_traceBuf, 1);
			break;
		}

		if (strncmp("58", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			naAccelerometerEpoch();
			break;
		}

		if (strncmp("57", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			AccelReadData();
			break;
		}

		if (strncmp("56", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			AccelCalibrate();
			break;
		}

		if (strncmp("55", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			sscanf(ADL_GET_PARAM(Cmd, 1), "%lf", &SpeedIn);
			SpeedIn1000 = SpeedIn * 1000;
			wm_sprintf(g_traceBuf, "Speed Filter = %d\r\n", SpeedIn1000);
			DumpMessage(g_traceBuf);
			FilterSpeed(&SpeedIn);
			SpeedIn1000 = SpeedIn * 1000;
			wm_sprintf(g_traceBuf, "Speed Filter = %d\r\n", SpeedIn1000);
			DumpMessage(g_traceBuf);
			break;
		}

		if (strncmp("54", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			GetNMEAError(&nm);
			wm_sprintf(g_traceBuf, "RMC error: %d\r\nGSV error: %d\r\nGGA error: %d\r\n",
			           nm.RMCErrors, nm.GSVErrors, nm.GGAErrors);
			DumpMessage(g_traceBuf);
			DumpMessageUSB(g_traceBuf, 1);
			break;
		}

		if (strncmp("53", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			wm_itohexa((ascii *)&tmp_nmea_string, 44981287, 8);
			tmp_nmea_string[9] = 0;
			wm_sprintf(g_traceBuf, "result = %s\r\n", tmp_nmea_string);
			DumpMessage(g_traceBuf);
			break;
		}

		if (strncmp("52", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			unsigned int ii;
			memcpy(tmp_nmea_string, ADL_GET_PARAM(Cmd, 1), strlen(ADL_GET_PARAM(Cmd, 1)));
			for (ii = 0; ii < strlen(ADL_GET_PARAM(Cmd, 1)); ii++)
			{
				if (tmp_nmea_string[ii] == '_')
				{
					tmp_nmea_string[ii] = ',';
				}
			}

			ParseNMEA(tmp_nmea_string, strlen(ADL_GET_PARAM(Cmd, 1)));

			break;
		}

		if (strncmp("51", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			performColdStart();
			break;
		}

		if (strncmp("50", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			//	performAGPS(44.979944,-93.275149,2000,1E3);
			performAGPS(0.0, 0.0, 0.0, 0.0);
			break;
		}

		if (strncmp("49", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			SMSHandling_DisplayRegStatus();
			break;
		}

		// this will clear out the TCP connect status.
		if (strncmp("48", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			ClearTCPStatus();
			break;
		}

		// This will toggle "forced" roam mode.
		if (strncmp("47", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			if (ForceRoamMode)
			{
				ForceRoamMode = 0;
				DumpMessage("Disabling forced roam mode.\r\n");
			}
			else
			{
				ForceRoamMode = 1;
				DumpMessage("Enabling forced roam mode.\r\n");
			}
			break;
		}

		if (strncmp("46", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			wm_sprintf(g_traceBuf, "IP Addr = %x\r\n", (unsigned int)GPRS_get_IPAddr());
			DumpMessage(g_traceBuf);
			wip_inet_ntoa(GPRS_get_IPAddr(), IpAddr, 15);

			wm_sprintf(g_traceBuf, "IP Addr = %s\r\n", (char *)IpAddr);
			DumpMessage(g_traceBuf);

			GetTCPStatus(&tcp_stat);

			switch (tcp_stat)
			{
			case TCP_NOT_INIT:
				DumpMessage("TCP NOT INIT\r\n");
				break;

			case TCP_CANT_CONNECT:
				DumpMessage("TCP CANT CONNECT\r\n");
				break;

			case TCP_TRYING:
				DumpMessage("TCP TRYING\r\n");
				break;

			case TCP_CONNECT:
				DumpMessage("TCP CONNECT\r\n");
				break;

			default:
				DumpMessage("Unknown TCP STATUS\r\n");
			}

			break;
		}

		if (strncmp("45", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			g_status.RebootAlarm = 'Y';
			break;
		}
		if (strncmp("44", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			//wm_sprintf(g_traceBuf, "GPS State = %d\r\n", erGetGpsState());
			//DumpMessage(g_traceBuf);
			break;
		}
		if (strncmp("43", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			wm_sprintf(g_traceBuf, "Battery stat = %d\r\n", g_status.BattLevel);
			DumpMessage(g_traceBuf);
			break;
		}

		if (strncmp("42", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			wm_sprintf(g_traceBuf, "Mag Sens Val: %d\r\n", readMagSens());
			DumpMessage(g_traceBuf);
			break;
		}

		// force into dog park mode.
		if (strncmp("41", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
            dogpark_t mode = *ADL_GET_PARAM(Cmd, 1) == '0' ? DOGPARK_DISABLED : DOGPARK_ENABLED;
            SetDogParkMode(mode);
			break;
		}
		// force into dog walk mode.
		if (strncmp("40", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
            dogwalk_t mode = *ADL_GET_PARAM(Cmd, 1) == '0' ? DOGWALK_DISABLED : DOGWALK_ENABLED;
            SetDogWalkMode(mode);
			break;
		}

		if  (strncmp("39", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			wm_sprintf(g_traceBuf, "Num waypoints = %d\r\n", GetNumWaypoints());
			DumpMessage(g_traceBuf);
			break;
		}

		// display tracking mode and gps states.
		if (strncmp("38", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			DisplayGPSStates();
			DisplayPowerState();
			break;
		}

		// stuff the fake GGA info struct
		if (strncmp("37", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			g_GPSTestMode = 1;
			break;
		}

		// stuff the fake GGA info struct
		if (strncmp("36", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			gGGAInfoTest.Time = hex_ascii_2_uint((UINT8 *)ADL_GET_PARAM(Cmd, 1), 32);
			gGGAInfoTest.Latitude = hex_ascii_2_uint((UINT8 *)ADL_GET_PARAM(Cmd, 2), 32);
			gGGAInfoTest.Longitude = hex_ascii_2_uint((UINT8 *)ADL_GET_PARAM(Cmd, 3), 32);
			gGGAInfoTest.FixQuality = 1;
			gGGAInfoTest.NumberOfSatillites = hex_ascii_2_uint((UINT8 *)ADL_GET_PARAM(Cmd, 4), 8);
			gVTGInfoTest.Course = hex_ascii_2_uint((UINT8 *)ADL_GET_PARAM(Cmd, 5), 32);
			gVTGInfoTest.Speed = hex_ascii_2_uint((UINT8 *)ADL_GET_PARAM(Cmd, 6), 32);
			adl_atSendResponsePort(ADL_AT_RSP, ADL_PORT_USB, "OK\r\n");
			break;
		}

		// Go into low power mode
		if (strncmp("35", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			EnterSlowIdleMode();
			break;
		}

#if defined(SST_FLASH)
		extern void spiFlash_ReadID();

		// Read the factory settings
		if (strncmp("34", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			spiFlash_ReadID();
			break;
		}
#endif

		// Run the find flash location and print out the location
		if (strncmp("33", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			// find the pointers in the serial flash.
			if (FindFlashLocation())
			{
				DumpMessage("Could not find flash location\r\n");
				break;
			}

			// display the pointers.
			DisplayPointers();
		}

		// set the accelerometer trigger threshold.
		if (strncmp("32", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			tmp8 = hex_ascii_2_uint((UINT8 *)ADL_GET_PARAM(Cmd, 1), 8);
			wm_sprintf(g_traceBuf, "Setting accel trigger threshold to : %x\r\n", tmp8);
			DumpMessage(g_traceBuf);
			if (SetAccelTriggerThreshold(tmp8))
			{
				DumpMessage("Trigger threshold set error\r\n");
			}
			else
			{
				DumpMessage("Trigger threshold  set success\r\n");
			}
			break;
		}

		// set an led blink pattern.
		if (strncmp("31", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			g_config.LEDPattern = (char)*ADL_GET_PARAM(Cmd, 1) - 0x30;
			g_config.ServerLEDOverride = 1;
			wm_sprintf(g_traceBuf, "Setting LED pattern to %d\r\n", g_config.LEDPattern);
			DumpMessage(g_traceBuf);
			break;
		}
		// Do a waypoint read.
		if (strncmp("30", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			tmp = hex_ascii_2_uint((UINT8 *)ADL_GET_PARAM(Cmd, 1), 32);
			wm_sprintf(g_traceBuf, "Reading waypoint offset %d\r\n", (int)tmp);
			DumpMessage(g_traceBuf);
			if (!ReadWaypoint(&test_waypoint, tmp))
			{
				DumpMessage("Read waypoint success\r\n");
				DisplayWaypoint(&test_waypoint);
				DisplayWaypointUSB(&test_waypoint);
			}
			else
			{
				DumpMessage("Read waypoint fail\r\n");
			}
			break;
		}
		// do a waypoint write.
		if (strncmp("29", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			if (Cmd->NbPara != 6)
			{
				wm_sprintf(g_traceBuf, "Error, invalid number of parameters for waypoint = %d\r\n", Cmd->NbPara);
				DumpMessage(g_traceBuf);
			}
			else
			{
				if ((strlen(ADL_GET_PARAM(Cmd, 1)) == 8) && (strlen(ADL_GET_PARAM(Cmd, 2)) == 8) && (strlen(ADL_GET_PARAM(Cmd, 3)) == 8) &&
				    (strlen(ADL_GET_PARAM(Cmd, 4)) == 2) && (strlen(ADL_GET_PARAM(Cmd, 5)) == 8))
				{
					memcpy(&(test_waypoint.utc[0]), ADL_GET_PARAM(Cmd, 1), 8);
					memcpy(&(test_waypoint.lat[0]), ADL_GET_PARAM(Cmd, 2), 8);
					memcpy(&(test_waypoint.longi[0]), ADL_GET_PARAM(Cmd, 3), 8);
					memcpy(&(test_waypoint.epe[0]), ADL_GET_PARAM(Cmd, 4), 2);
					memcpy(&(test_waypoint.speed[0]), ADL_GET_PARAM(Cmd, 5), 8);

					DisplayWaypoint(&test_waypoint);
					if (!WriteWaypoint(&test_waypoint))
					{
						DumpMessage("Wrote waypoint success\r\n");
						wm_sprintf(g_traceBuf, "Size of waypoint: %d\r\n", (int )sizeof (WAYPOINT));
						DumpMessage(g_traceBuf);
					}
					else
					{
						DumpMessage("Wrote waypoint fail\r\n");
					}
				}
				else
				{
					DumpMessage("Invalid parameter length for waypoint\r\n");
				}
			}
			break;
		}

		// load a config packet.
		if (strncmp("28", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			wm_sprintf(g_traceBuf, "length=%d\r\n", (int )strlen(ADL_GET_PARAM(Cmd, 1)));
			DumpMessage(g_traceBuf);
			wm_sprintf(g_traceBuf, "\r\nrx = %s\r\n", ADL_GET_PARAM(Cmd, 1));
			DumpMessage(g_traceBuf);

			memcpy(g_rx, ADL_GET_PARAM(Cmd, 1), strlen(ADL_GET_PARAM(Cmd, 1)));

			/* had to add some offsets to make these ascii strings */
			g_rx[PKT_FNC_START_TIME] -= 128;
			g_rx[PKT_FNC_STOP_TIME] -= 128;
			eval_packet();
			break;
		}
		if (strncmp("27", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			DisplayConfig();
			break;
		}
		if (strncmp("26", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			if (Cmd->NbPara != 6)
			{
				SetCurrentFix(ADL_GET_PARAM(Cmd, 1), ADL_GET_PARAM(Cmd, 2),
				              ADL_GET_PARAM(Cmd, 3), ADL_GET_PARAM(Cmd, 4));
			}
			else
			{
				wm_sprintf(g_traceBuf, "Error, invalid number of parameters for SetCurrentFix = %d\r\n", Cmd->NbPara);
				DumpMessage(g_traceBuf);
			}

			fence_res = eval_fix(&fence_num, GPS_FIX_REASON);
			wm_sprintf(g_traceBuf, "eval fix returns: %c\r\n", fence_res);
			DumpMessage(g_traceBuf);
			break;
		}

		if (strncmp("25", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			SetVibrationMotorPattern((UINT8 *)ADL_GET_PARAM(Cmd, 1), &g_config);
			break;
		}
		if (strncmp("23", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			//wm_sprintf(g_traceBuf,"\r\nWaypoint Interval: %d\r\n",g_WaypointInterval);
			DumpMessage(g_traceBuf);
			break;
		}
		// Set waypoint interval.
		if (strncmp("20", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			SetWaypointInterval((UINT8 *)ADL_GET_PARAM(Cmd, 1), &g_config);
			break;
		}
		// Set tracking interval.

		if (strncmp("11", ADL_GET_PARAM(Cmd, 0), 2) == 0)
		{
			//				DumpLastKnownCoord();
			break;
		}
		if (strncmp("1", ADL_GET_PARAM(Cmd, 0), 1) == 0)
		{
			DumpMessage("Xact Tracking Device\r\n");
			//				InitializeButton();
			break;
		}
		if (strncmp("2", ADL_GET_PARAM(Cmd, 0), 1) == 0)
		{
			sms_send('S');
			break;
		}
		if (strncmp("3", ADL_GET_PARAM(Cmd, 0), 1) == 0)
		{
			break;
		}
		if (strncmp("4", ADL_GET_PARAM(Cmd, 0), 1) == 0)
		{
			break;
		}
		if (strncmp("5", ADL_GET_PARAM(Cmd, 0), 1) == 0)
		{
			//				StoreToFlash();
			break;
		}
		if (strncmp("6", ADL_GET_PARAM(Cmd, 0), 1) == 0)
		{
			//				LoadFromFlash();
			break;
		}
		if (strncmp("7", ADL_GET_PARAM(Cmd, 0), 1) == 0)
		{
			TRACE((1, "Num para: %d", Cmd->NbPara));

			switch (Cmd->NbPara)
			{
			default:
				DumpMessage("WriteFenceCoord: Invalid Number of parameters\r\n");
				break;
			}
		}
		if (strncmp("8", ADL_GET_PARAM(Cmd, 0), 1) == 0)
		{
			DumpFenceFlash();
			break;
		}
		if (strncmp("9", ADL_GET_PARAM(Cmd, 0), 1) == 0)
		{
			//				TestGPIO(wm_hexatoi(ADL_GET_PARAM(Cmd,1),1), wm_hexatoi(ADL_GET_PARAM(Cmd,2),1));
			break;
		}
	}
}


/*@}*/
