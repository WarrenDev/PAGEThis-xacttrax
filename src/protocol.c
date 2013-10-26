/** @addtogroup NetworkComm
 *@{ */

/** @file    protocol.c
 *
 * @brief   parses the protocol.
 *
 *
 */

/*-----------------------------------------------------------------------------
 *  INCLUDES
 *  -----------------------------------------------------------------------------*/
#include <adl_global.h>
#include "wip.h"
#include "pistd.h"
#include "protocol.h"
#include "XactUtilities.h"
#include "gps.h"
#include "gpioTest.h"
#include "fence.h"
#include "SMShandling.h"
#include "FlashTest.h"
#include "ConfigSettings.h"
#include "status.h"
#include "InternalFlash.h"
#include "PowerCtrl.h"
#include "Accelerometer.h"
#include "TempoCasesTask.h"
#include "DOTAOperation.h"
#include "Timers.h"
#include "Anolclient.h"
#include "adc.h"
#include "ota_at.h"

/*-----------------------------------------------------------------------------
 *  CONSTANT & MACRO DEFINITIONS
 *  -----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
 *  GLOBAL DECLARATIONS
 *  -----------------------------------------------------------------------------*/
UINT8 g_rx[RX_BUFFSIZE];                            /* Buff for receiving input from GSM & GPS modules */
UINT8 g_last_config[RX_BUFFSIZE];                   /* store the last recieved configuration */
FAST_IDLE_REQ PacketRXFastIdleReq = FAST_IDLE_REQ_RELEASE;
SMS_WAYPOINT_PAYLOAD sms_waypoint_data;

static CONFIG temp_config; /* structure for holding the configuration settings until all are found to be valid */
static MODE_CONFIG temp_ModeConfig;

u8 PREV_Mode = TRACK_MODE;
ascii FTPAddress[15];

s32 GPIO_5_Handle = ERROR;			
s32 GPIO_7_Handle = ERROR;
UINT8 g_DiagCmdOffset;

struct structGGAInformation g_GGAInfo;

/*-- External variables -----------------------------------*/
extern UINT8 g_sms_tx[SMS_TX_BUFFSIZE];    /* Buff for building output packets */
extern STATUS g_tx_status;
extern ascii SMS_PHN_NUM[15];
extern char g_IMEI[15];
extern s8 g_smsHandle;
extern const ascii adl_InitApplicationName[];
extern s32 g_AvgAnalogVltg;
extern s32 g_BatteryVltg;
extern s32 g_BoardTemp;
extern DIAG_CONFIG g_DiagConfig[NUM_DIAG_COMMANDS];
extern UINT8 g_EngineStatus;
extern adl_rtcTime_t EngineStartTime;		
extern adl_rtcTime_t EngineStopTime;
extern UINT32 g_TotalRunTime;

extern u8 g_fcm_Handle;

/*-----------------------------------------------------------------------------
 *  FUNCTION PROTOTYPES
 *  -----------------------------------------------------------------------------*/
static int parse_config_pkt(UINT16);
static char ConvertBool_to_NY(bool value);
static char AddModeStatus(void);
static int check_latin_1(char test_char);
static void set_LastConfigSendAck(bool val);

/*F***************************************************************************/

/** @brief converts a bool type to a char type
 *
 * @par
 * Adds char to SMS packet
 * @param value
 * @return char 'Y' or 'N'
 */
static char ConvertBool_to_NY(bool value)
{
	if (value == TRUE)
	{
		return 'Y';
	}

	return 'N';
}


/*F***************************************************************************/

/** @brief Add Mode Status
 *
 * @par looks at the mode setting in config struct and converts
 * it to char to be added to sms packet
 *
 * @return char -> '0','P','T','W','Q','S','V', or '!'
 */
static char AddModeStatus(void)
{
	switch (g_config.Mode)
	{
	case NO_CHANGE_MODE:
		return '0';

	case DOG_PARK_MODE:
		return 'P';

	case TRACK_MODE:
		return 'T';

	case FULL_TRACK_MODE:
		return 'F';		

	case LP_FULL_TRACK_MODE:
		return 'L';	

	case ALARM_MODE:
		return 'A';
		
	case WALK_MODE:
		return 'W';

	case SERVER_NO_SLEEP:
		return 'Q';

	case SERVER_STATUS_REQ:
		return 'S';

	case SERVER_STATUS_REQ_V:
		return 'V';

	default:
		return '!';
	}
}


/*F***************************************************************************/

/** @brief make packet to send to Server
 *
 * @par Constructs a packet to send to the server.
 *
 * @param pckt_type - Type of packet to build.
 * @return void
 */
void make_packet(UINT8 pckt_type)
{
	// UINT8 checker = 0x55;
	char buffer[155];            //A buffer to store the data read in from a log ID
	WAYPOINT tmp_wp;
    ascii time_buffer[8];
    adl_rtcTime_t CurrentTime;
	adl_rtcTimeStamp_t  CurrentTimeStamp;
	s32 sReturn = -1;
	
	/* Start off with all '0's */
	memset(g_sms_tx, '0', SMS_SIZE);
	memset(buffer, '0', 155);
	//g_sms_tx[SMS_SIZE-1] = '\0';

	/* Put in start flags */
	g_sms_tx[PKT_FLAG1] = '.';
	g_sms_tx[PKT_FLAG2] = '.';

	/* Choose the type of packet requested */
	switch (pckt_type)
	{
	case 'S':          /* Status packet and Log Packet, which are the same*/
		DumpMessage("\r\n Making a status packet\r\n");

		g_sms_tx[PKT_TYPE] = 'S';

		//add battery level

		// use a 'C0' to indicate that the USB cable is plugged in
		// otherwise load the battery level percentage value - AEW Jr.
		if (IsUSBConnected())
		{
			g_sms_tx[PKT_BATT_LVL]   = 'C';
			g_sms_tx[PKT_BATT_LVL+1] = '0';
		}
		else
		{
//			g_sms_tx[PKT_BATT_LVL] = 0x30 + g_tx_status.BattLevel;
			wm_itohexa((ascii *)&g_sms_tx[PKT_BATT_LVL], g_VoltagePct, 2);
		}

		//add system failure error status
		g_sms_tx[PKT_SYS_FAIL] = g_tx_status.SystemFailure;

		//add GPS status
		g_sms_tx[PKT_GPS_STATE] = g_tx_status.GPSStatus;

		//add GPS signal strength of the highest level
		g_sms_tx[PKT_GPS_SIG1] = g_tx_status.GPSSignalStrength1;

		//add GPS signal strength of the 4th highest level
		g_sms_tx[PKT_GPS_SIG2] = g_tx_status.GPSSignalStrength2;

		//add number of GPS satellites or "birds"
		g_sms_tx[PKT_NUM_BIRDS] = g_tx_status.NumBirds;

		//add GSM status
		g_sms_tx[PKT_GSM_STATE] = g_tx_status.GSMStatus;

		//add GSM signal strength
		g_sms_tx[PKT_GSM_SIG] = g_tx_status.GSMSignalStrength;

		//add Cell ID
		memcpy(&(g_sms_tx[PKT_CELL_ID]), &(g_tx_status.CellID[0]), 4);

		//add Cell location area
		memcpy(&(g_sms_tx[PKT_LOC_AREA]), &(g_tx_status.LocationArea[0]), 4);

		//add valid access counter
		g_sms_tx[PKT_VALID_ACCESS_CNT] = g_tx_status.ValidAccessCnt;
		//add bad access counter
		g_sms_tx[PKT_BAD_ACCESS_CNT] = g_tx_status.BadAccessCnt;

		//add number of fences currently on device
		uint_2_hex_ascii(g_tx_status.NumFences,
		                 &(g_sms_tx[PKT_STATUS_NUM_FENCE]), 8);

		//add mode
		g_sms_tx[PKT_STATUS_MODE] = AddModeStatus();

		//add if SOS Alarm is going off
		g_sms_tx[PKT_SOS_ALARM] = g_tx_status.SOSAlarm;

		//add if GPS Alarm is going off
		g_sms_tx[PKT_GPS_ALARM] = g_tx_status.GPSAlarm;

		//add if Battery Alarm is going off
		g_sms_tx[PKT_BATT_ALARM] = g_tx_status.BattAlarm;

		//add if Battery Alarm is going off
		g_sms_tx[PKT_GSM_ALARM] = g_tx_status.GSMAlarm;

		//add if Fence Alarm is going off
		g_sms_tx[PKT_FENCE_ALARM] = g_tx_status.FenceAlarm;

		//add current fence device is in
		uint_2_hex_ascii(g_tx_status.CurrentFenceIn,
		                 &(g_sms_tx[PKT_CUR_FENCE]), 8);

		//add if reboot alarm is going off
		g_sms_tx[PKT_REBOOT_ALARM] = g_tx_status.RebootAlarm;

		//add if over speed alarm is going off
		g_sms_tx[PKT_OVER_SPD_ALARM] = g_tx_status.OverSpeedAlarm;

		//add if motion alarm is going off
		g_sms_tx[PKT_MOTION_ALARM] = g_tx_status.MotionAlarm;

		//add power disconnect alarm
		g_sms_tx[PKT_PWR_DISCON_ALARM] = g_tx_status.PowerDisconnAlarm;

		//add fix data 1-3
		int i,j;
		int pkt_offset = PKT_FIX1_TIME;

		for (i = -1; i >= -3; i--)
		{
			if (ReadWaypoint(&tmp_wp, i))
			{
				memset((u8 *)(&(g_sms_tx[pkt_offset])), '0', NUM_WAYPOINT_BYTES);
				DumpMessage("Not enough waypoints!\r\n");
			}
			else
			{
				memcpy((u8 *)(&(g_sms_tx[pkt_offset])), (u8 *)&tmp_wp, NUM_WAYPOINT_BYTES);
			}
			pkt_offset += NUM_WAYPOINT_BYTES;
		}
	
		// Time.
		// get the current RTC time.
		if ((sReturn = adl_rtcGetTime(&CurrentTime)) < 0) 
		{
	    	DisplayErrorCode("adl_rtcGetTime",__FILE__,__LINE__,sReturn);
	    	return;
		}
	    if ((sReturn = adl_rtcConvertTime( &CurrentTime, &CurrentTimeStamp, ADL_RTC_CONVERT_TO_TIMESTAMP)) < 0) 
		{
			DisplayErrorCode("adl_rtcConvertTime",__FILE__,__LINE__,sReturn);
    	}
		wm_itohexa((ascii *)&time_buffer, CurrentTimeStamp.TimeStamp, 8);

        wm_sprintf(g_traceBuf,"RTC TimeStamp = 0x%x\r\n",(unsigned int)CurrentTimeStamp.TimeStamp);		
		DumpMessage(g_traceBuf);

        wm_sprintf(g_traceBuf,"Date(dd:mm:yy) = %02d:%02d:%04d\r\n",CurrentTime.Day,CurrentTime.Month,CurrentTime.Year);		
		DumpMessage(g_traceBuf);

        wm_sprintf(g_traceBuf,"Time(hh:mm:ss) = %02d:%02d:%02d\r\n",CurrentTime.Hour,CurrentTime.Minute,CurrentTime.Second);		
		DumpMessage(g_traceBuf);
		
		for(i=0,j=RTC_TIME_START;i<8;i++,j++)
		{
			g_sms_tx[j] = time_buffer[i];
		}

		pkt_offset += 8;

		//Add sierra constant to the status packet.
		for(i=0,j=SIERRA_VER_START;i<5;i++,j++)
		{
			g_sms_tx[j] = adl_InitApplicationName[i];
		}
		pkt_offset += 5;

		//AGPS status
		g_sms_tx[AGPS_STAT] = 0x30 + get_agps_status();  //convert to ascii.
		pkt_offset++;
		
		// add the null char
		g_sms_tx[pkt_offset] = 0x00;
			
		// check the packet we are going to transmit.
		// if there are any characters not in the latin 1 replace with a '0'
		for (i = 0; i < pkt_offset; i++)
		{
			if (check_latin_1(g_sms_tx[i]))
			{
				g_sms_tx[i] = 'x';
			}
		}

		break;

	case 'C':          /* Configure return packet */
		g_sms_tx[PKT_TYPE] = 'C';
		g_sms_tx[PKT_CONFIG_ACK] = ConvertBool_to_NY(g_config.ConfigPktRxProperly);

		// Copy over the rest of the received packet starting at the fence number
		memcpy(&g_sms_tx[PKT_FENCE_NUM], &g_rx[PKT_FENCE_NUM], 156);

		//load firmware rev number into config packet
		wm_itohexa((ascii *)&g_sms_tx[PKT_FW_REV], g_config.FirmwareRevNum, 2);
		wm_sprintf(g_traceBuf, "firmware version: %x\r\n", g_config.FirmwareRevNum);
		DumpMessage(g_traceBuf);
		//load hardware revision number into config packet
		wm_itohexa((ascii *)&g_sms_tx[PKT_HW_REV], g_config.HardwareRevNum, 2);
		// replace value
		g_sms_tx[PKT_RESERVED] = '0';

		g_sms_tx[CONFIG_PKT_SIZE] = 0x00;

		break;
	case 'M':
		g_sms_tx[PKT_TYPE] = 'M';
		g_sms_tx[M_ACK] = (g_ModeConfig.PktRxdProperly == TRUE)?'Y':'N';
		// Copy over the rest of the received packet starting at the fence number
		memcpy(&g_sms_tx[USE_CASE], &g_rx[4], (MODE_CONFIG_PKT_SIZE-4));   //M packet is of 16 bytes
		g_sms_tx[MODE_CONFIG_PKT_SIZE] = '\0';
		
		break;
	case 'W':          /* Waypoint Data return packet */
		DumpMessage("\r\nMaking a waypoint packet to send\r\n");
		g_sms_tx[PKT_TYPE] = 'W';
		wm_sprintf((ascii *)&g_sms_tx[PKT_WAY_PKT_NUM], "%08x", sms_waypoint_data.packet_num);
		memcpy(&g_sms_tx[PKT_WAY_DATA], (u8 *)&sms_waypoint_data.waypoint1, NUM_WAYPOINT_BYTES);
		memcpy(&g_sms_tx[PKT_WAY_DATA + NUM_WAYPOINT_BYTES], (u8 *)&sms_waypoint_data.waypoint2, NUM_WAYPOINT_BYTES);
		memcpy(&g_sms_tx[PKT_WAY_DATA + 2 * NUM_WAYPOINT_BYTES], (u8 *)&sms_waypoint_data.waypoint3, NUM_WAYPOINT_BYTES);
		memcpy(&g_sms_tx[PKT_WAY_DATA + 3 * NUM_WAYPOINT_BYTES], (u8 *)&sms_waypoint_data.waypoint4, NUM_WAYPOINT_BYTES);

		//for testing only

		/* char * sms_wpt_ptr = &sms_waypoint_data;//&g_sms_tx;//&sms_waypoint_data;
		 * for(i=0;i<147;i++)
		 * {
		 * wm_sprintf(g_traceBuf,"%d:%x ", i,*(sms_wpt_ptr+i));
		 * DumpMessage(g_traceBuf);
		 * }
		 */
		//null char needs to be added at the end of the message otherwise
		//the packet could be much longer than it is suppose to be
		g_sms_tx[WAYPOINT_PKT_SIZE] = 0x00;
		break;

	case 'L':         /*Log packet */
		DumpMessage("\r\n Reading Log packetfrom internal flash to send\r\n");
		LogPktFromFlash(g_tx_status.LogPktToSendLatest);
		// Process type of log packet.
		if (g_sms_tx[PKT_TYPE] == 'S')
		{
			g_sms_tx[PKT_TYPE] = 'L';
		}
		else
		{
			g_sms_tx[PKT_TYPE] = 'K';
		}

		break;
	case 'I':
		{
			#define SOS_BUTTON_GPIO (20)
			#define MAG_SOS_GPIO    (22)
			//#define GPIO_5			(5)
			//#define GPIO_7			(7)
			
			//static s32 SOS_GPIOHandle = ERROR;
			//static s32 MagSOS_GPIOHandle = ERROR;

			UINT8 AnaBuff[20];
			
			DumpMessage("\r\n Making a IO status packet\r\n");

			g_sms_tx[PKT_TYPE] = 'I';

			//add battery level

			// use a 'C' to indicate that the USB cable is plugged int
			// otherwise put the level 1-3
			if (IsUSBConnected())
			{
				g_sms_tx[PKT_BATT_LVL] = 'C';
			}
			else
			{
				g_sms_tx[PKT_BATT_LVL] = 0x30 + g_status.BattLevel;
			}

			//add system failure error status
			g_sms_tx[PKT_SYS_FAIL] = g_status.SystemFailure;	

			// Time.
			// get the current RTC time.
			if ((sReturn = adl_rtcGetTime(&CurrentTime)) < 0) 
			{
		    	DisplayErrorCode("adl_rtcGetTime",__FILE__,__LINE__,sReturn);
		    	return;
			}
		    if ((sReturn = adl_rtcConvertTime( &CurrentTime, &CurrentTimeStamp, ADL_RTC_CONVERT_TO_TIMESTAMP)) < 0) 
			{
				DisplayErrorCode("adl_rtcConvertTime",__FILE__,__LINE__,sReturn);
	    	}
			wm_itohexa((ascii *)&time_buffer, CurrentTimeStamp.TimeStamp, 8);

	        wm_sprintf(g_traceBuf,"RTC TimeStamp = 0x%x\r\n",(unsigned int)CurrentTimeStamp.TimeStamp);		
			DumpMessage(g_traceBuf);
			//DumpMessageUSB(g_traceBuf,1);

	        wm_sprintf(g_traceBuf,"Date(dd:mm:yy) = %02d:%02d:%04d\r\n",CurrentTime.Day,CurrentTime.Month,CurrentTime.Year);		
			DumpMessage(g_traceBuf);
			//DumpMessageUSB(g_traceBuf,1);

	        wm_sprintf(g_traceBuf,"Time(hh:mm:ss) = %02d:%02d:%02d\r\n",CurrentTime.Hour,CurrentTime.Minute,CurrentTime.Second);		
			DumpMessage(g_traceBuf);
			//DumpMessageUSB(g_traceBuf,1);
			
			for(i=0,j=IO_RTC_TIME_STAMP;i<8;i++,j++)
			{
				g_sms_tx[j] = time_buffer[i];
			}

			//g_sms_tx[PANIC]           = 0x30 + GpioRead(SOS_BUTTON_GPIO, &SOS_GPIOHandle);   // Convert to ascii by adding 0x30.
			//g_sms_tx[MAGENTIC_SENSOR] = 0x30 + GpioRead(MAG_SOS_GPIO, &MagSOS_GPIOHandle);
			g_sms_tx[GPIO5_STATUS]    = 0x30 + GpioRead(GPIO_IGN_GPIO, &GPIO_5_Handle);
			g_sms_tx[GPIO7_STATUS]    = 0x30 + GpioRead(GPIO_7, &GPIO_7_Handle);
			g_sms_tx[PANIC]			  = 0x30 + ReadSOSGPIO();	
			g_sms_tx[MAGENTIC_SENSOR] = 0x30 + ReadMagSensorGPIO();

			
			//I2C sensor value
			//wm_itohexa((ascii *)AnaBuff,g_AvgAnalogVltg,4);
			g_sms_tx[I2C_SENSOR]   = '0';//AnaBuff[0];
			g_sms_tx[I2C_SENSOR+1] = '0';//AnaBuff[1];
			g_sms_tx[I2C_SENSOR+2] = '0';//AnaBuff[2];		
			g_sms_tx[I2C_SENSOR+3] = '0';//AnaBuff[3];

			//Update Analog Sensor Value.		
			wm_itohexa((ascii *)AnaBuff,g_AvgAnalogVltg,4);
			g_sms_tx[ANALOG_VALUE]   = AnaBuff[0];
			g_sms_tx[ANALOG_VALUE+1] = AnaBuff[1];
			g_sms_tx[ANALOG_VALUE+2] = AnaBuff[2];		
			g_sms_tx[ANALOG_VALUE+3] = AnaBuff[3];

		    wm_sprintf(g_traceBuf,"Analog Sensor : AnaBuff[0]:0x%x,AnaBuff[1]:0x%x,AnaBuff[2]=0x%x,AnaBuff[3]=0x%x \r\n", AnaBuff[0],AnaBuff[1],AnaBuff[2],AnaBuff[3]);
			DumpMessage(g_traceBuf);
			DumpMessageUSB(g_traceBuf,1);

			//Update Battery voltge.		
			wm_itohexa((ascii *)AnaBuff,g_BatteryVltg,4);
			g_sms_tx[BATTERY_VLTG]   = AnaBuff[0];
			g_sms_tx[BATTERY_VLTG+1] = AnaBuff[1];
			g_sms_tx[BATTERY_VLTG+2] = AnaBuff[2];		
			g_sms_tx[BATTERY_VLTG+3] = AnaBuff[3];

		    wm_sprintf(g_traceBuf,"Battery : AnaBuff[0]:0x%x,AnaBuff[1]:0x%x,AnaBuff[2]=0x%x,AnaBuff[3]=0x%x \r\n", AnaBuff[0],AnaBuff[1],AnaBuff[2],AnaBuff[3]);
			DumpMessage(g_traceBuf);
			DumpMessageUSB(g_traceBuf,1);

			//Yet to decide what is to be updated. For now fill with 0s.
			g_sms_tx[ACCELEROMETER]   = '0'; 
			g_sms_tx[ACCELEROMETER+1] = '0'; 	
			g_sms_tx[ACCELEROMETER+2] = '0'; 
			g_sms_tx[ACCELEROMETER+3] = '0'; 

			//Update Battery voltge.		
			wm_itohexa((ascii *)AnaBuff,g_BoardTemp,4);
			g_sms_tx[BOARD_TEMP]   = AnaBuff[0];
			g_sms_tx[BOARD_TEMP+1] = AnaBuff[1];
			g_sms_tx[BOARD_TEMP+2] = AnaBuff[2];		
			g_sms_tx[BOARD_TEMP+3] = AnaBuff[3];

			g_sms_tx[IGN_STATUS] = 0x30+g_EngineStatus;

			//Update run time - Time counted from voltage jump above threshold and Ignition ON.
			//wm_itohexa((ascii *)&time_buffer, CurrentTimeStamp.TimeStamp, 8);

		    if ((sReturn = adl_rtcConvertTime( &EngineStartTime, &CurrentTimeStamp, ADL_RTC_CONVERT_TO_TIMESTAMP)) < 0) 
			{
				DisplayErrorCode("adl_rtcConvertTime",__FILE__,__LINE__,sReturn);
	    	}
			wm_itohexa((ascii *)&time_buffer, CurrentTimeStamp.TimeStamp, 8);

			//Update engine on time
			for(i=0,j=ENG_ON_TIME;i<8;i++,j++)
			{
				g_sms_tx[j] = time_buffer[i];
			}

		    if ((sReturn = adl_rtcConvertTime( &EngineStopTime, &CurrentTimeStamp, ADL_RTC_CONVERT_TO_TIMESTAMP)) < 0) 
			{
				DisplayErrorCode("adl_rtcConvertTime",__FILE__,__LINE__,sReturn);
	    	}
			wm_itohexa((ascii *)&time_buffer, CurrentTimeStamp.TimeStamp, 8);
			
			//Update engine off time			
			for(i=0,j=ENG_OFF_TIME;i<8;i++,j++)
			{
				g_sms_tx[j] = time_buffer[i];
			}

			wm_itohexa((ascii *)&time_buffer, g_TotalRunTime, 8);
			for(i=0,j=RUN_TIME;i<8;i++,j++)
			{
				g_sms_tx[j] = time_buffer[i];
			}

			//make sure we transmit only I_PKT_LENGHT bytes
			g_sms_tx[I_PKT_LENGHT] = 0x00;
					
		}	
	break;
        case 'A':   /* Atmel alert packet */
        {
            static u8 cntr = 0;
            GetGGAInfo(&g_GGAInfo);

            g_sms_tx[PKT_TYPE] = 'A';
            //extern u8 g_fcm_Handle;
            
            // Fill up alert data bytes
            memcpy(&g_sms_tx[TC_ALERT_NIBBLE_3], g_alert, 4);

            // Fill up free running counter bytes
            wm_itohexa((ascii *)&g_sms_tx[TC_CNTR_NIBBLE_1], cntr, 2 );
	
            //Fill up voltage percent bytes
            wm_itohexa((ascii *)&g_sms_tx[TC_VOLTPCT_NIBBLE_1], g_VoltagePct, 2);

/*GZ added GPS status and coordinates per Gabriel's request*/
            //add GPS status
            if( g_tx_status.GPSStatus == 0){
                g_sms_tx[TC_PKT_GPS_STATE] = '0';
            }
            else{
                g_sms_tx[TC_PKT_GPS_STATE] = g_tx_status.GPSStatus;
            }

            //add GPS signal strength of the highest level
            if (g_tx_status.GPSSignalStrength1 == 0){
                g_sms_tx[TC_PKT_GPS_SIG1] = '0';
            }
            else {
                g_sms_tx[TC_PKT_GPS_SIG1] = g_tx_status.GPSSignalStrength1;
            }

            //add GPS signal strength of the 4th highest level
            if (g_tx_status.GPSSignalStrength2 == 0){
                g_sms_tx[TC_PKT_GPS_SIG2] = '0';
            }
            else {
                g_sms_tx[TC_PKT_GPS_SIG2] = g_tx_status.GPSSignalStrength2;
            }

            //add number of GPS satellites or "birds"
            if (g_tx_status.NumBirds == 0){
                g_sms_tx[TC_PKT_NUM_BIRDS] = '0';
            }
            else {
                g_sms_tx[TC_PKT_NUM_BIRDS] = g_tx_status.NumBirds;
            }
            
            /*Refresh number of visible satellites with lower level data*/
            /* Overwrites previous settings */
            g_sms_tx[TC_PKT_NUM_BIRDS] = g_GGAInfo.NumberOfSatillites + '0';
            if (g_GGAInfo.NumberOfSatillites > 9 && g_GGAInfo.NumberOfSatillites < 16)
            {
                g_sms_tx[TC_PKT_NUM_BIRDS] = g_GGAInfo.NumberOfSatillites - 10 + 'A';
            }
            else
            {    
                if(g_GGAInfo.NumberOfSatillites > 16)
                {
                    g_sms_tx[TC_PKT_NUM_BIRDS] = 'F';
                }
            }

            //add fix data 1-3
            int i, j;
            int pkt_offset = TC_PKT_FIX1_TIME;

            for (i=-1;i>=-3;i--) 
            {
                if (ReadWaypoint(&tmp_wp,i)) 
                {
                    memset((u8 *)(&(g_sms_tx[pkt_offset])),'0',NUM_WAYPOINT_BYTES);
                    DumpMessage("Not enough waypoints!\r\n");
                }
				else
				{
					memcpy((u8 *)(&(g_sms_tx[pkt_offset])),(u8 *)&tmp_wp,NUM_WAYPOINT_BYTES);
				}
                pkt_offset += NUM_WAYPOINT_BYTES;
            }

			// Time.
			// get the current RTC time.  - AEW Jr. 
			if ((sReturn = adl_rtcGetTime(&CurrentTime)) < 0) 
			{
		    	DisplayErrorCode("adl_rtcGetTime",__FILE__,__LINE__,sReturn);
		    	return;
			}
		    if ((sReturn = adl_rtcConvertTime( &CurrentTime, &CurrentTimeStamp, ADL_RTC_CONVERT_TO_TIMESTAMP)) < 0) 
			{
				DisplayErrorCode("adl_rtcConvertTime",__FILE__,__LINE__,sReturn);
	    	}
			wm_itohexa((ascii *)&time_buffer, CurrentTimeStamp.TimeStamp, 8);

	        wm_sprintf(g_traceBuf,"Alert Pkt - RTC TimeStamp = 0x%x\r\n",(unsigned int)CurrentTimeStamp.TimeStamp);		
			DumpMessage(g_traceBuf);

	        wm_sprintf(g_traceBuf,"Alert Pkt - Date(dd:mm:yy) = %02d:%02d:%04d\r\n",CurrentTime.Day,CurrentTime.Month,CurrentTime.Year);		
			DumpMessage(g_traceBuf);

	        wm_sprintf(g_traceBuf,"Alert Pkt - Time(hh:mm:ss) = %02d:%02d:%02d\r\n",CurrentTime.Hour,CurrentTime.Minute,CurrentTime.Second);		
			DumpMessage(g_traceBuf);
		
			for(i=0,j=TC_RTC_TIME_START;i<8;i++,j++)
			{
				g_sms_tx[j] = time_buffer[i];
			}

			pkt_offset += 8;

			// add the null char
            g_sms_tx[TC_ALERT_PKT_SIZE] = 0x00; 
            cntr++;          

            // RS232 output for debugging
            adl_fcmSendData(g_fcm_Handle, (u8 *)g_sms_tx, strlen((char *)g_sms_tx));

            break;
        }
	}
}


/*F***************************************************************************/

/** @brief Evaluate incoming packet
 *
 * @par Determines what kind of packet the incoming packet is and
 * calls the appropriate handlers.
 *
 * @return 1 = send ack 0 = do not send ack
 */
int eval_packet(void)
{
	UINT16 start_offset = 0;     /* Where in the buffer the packet starts */
	UINT8 good_chars;
	BOOL good_packet = FALSE;
	int send_ack = 0;
	int i;

	PacketRXFastIdleReq = FAST_IDLE_REQ_ACTIVE;

	ClearIRQ();     // clear accel IRQ.

	DumpMessage("In Function eval_packet\r\n");
	DumpMessageUSB("In Function eval_packet\r\n",1);

	TRACE((1, "Start offset: %d", start_offset));
	TRACE((1, "Packet Type: %c", g_rx[start_offset + PKT_TYPE]));

	wm_sprintf(g_traceBuf, "Packet Type: %c\n\r", g_rx[PKT_TYPE]);
	DumpMessageUSB(g_traceBuf,1);

	for(i=0;i<160;i++)
	{
		wm_sprintf(g_traceBuf, "%c", g_rx[i]);
		DumpMessageUSB(g_traceBuf,1);	
	}
	DumpMessageUSB("\r\n",1);

	/* If the packet started with enough space left to be complete */
	if (start_offset < sizeof (g_rx))
	{
		/* Handle each kind of packet */
		switch (g_rx[start_offset + PKT_TYPE])
		//switch(g_rx[PKT_TYPE])
		{
		case 'C':         /* Configure packet */

			// since we are going to stuff binary data
			// into the SMS now, lets force good_chars
			good_chars = CONFIG_PKT_SIZE;

			send_ack = parse_config_pkt(start_offset);
			if (send_ack >= 0)
			{
				good_packet = TRUE;

				DumpMessage("Config Packet was good\n\r");

				//if all settings are valid then transfer temp settings to actual settings
				memcpy(&g_config, &temp_config, sizeof (CONFIG));

				// save it in the last config area
				memcpy(g_last_config, g_rx, RX_BUFFSIZE);
				
				//Update the tracking interval value appropriately. Values 0 and F are reserved as of now.
				//Whenever 0 or F is received in config, retain the previous value. Hence g_last_config
				//must be updated accordingly. Dont store values 0 and F.
				g_last_config[PKT_TRACKING_INT] = g_config.TrackingInterval;

		
				//When MODE received from server is '0' (No Mode Change), we have to retain the previous mode, hence 
				//update Mode value in g_last_config with the current Mode value. This will hold good for all the 
				//Modes requested from server.
				g_last_config[PKT_MODE] = AddModeStatus();  
				
				// write it to flash
				ConfigStringToFlash(g_last_config);

				//set the flag is teh config structure that the packet was received properly
				g_config.ConfigPktRxProperly = TRUE;
			}
			else
			{
				//set the flag is teh config structure that the packet wasnt received properly
				g_config.ConfigPktRxProperly = FALSE;

				DumpMessage("Bad Config Packet!\r\n");
			}

			//send back config confirmation packet back to the server if requested.
			set_LastConfigSendAck(send_ack);

			if (send_ack)
			{
				sms_send('C');
			}

			// Store it in flash.
			ConfigToFlash();

			break;
		case 'X':
			{
				s32 sReturn = -1;
				DumpMessage("Processing X packet\r\n");
				DumpMessageUSB("Processing X packet\r\n",1);
				for(i=0; i<14;i++)
				{
					wm_sprintf(g_traceBuf, "g_rx[%d] : %c\r\n", i,g_rx[i]);
					DumpMessage(g_traceBuf);
				}				
				if(g_rx[3] == 'P' && g_rx[4] == 'O' && g_rx[5] == 'L' && g_rx[6] == 'L')
				{
					DumpMessage("POLL command Received\r\n");
					
					if(strncmp((char*)&g_rx[8], (char*)&g_IMEI[9], 6) == 0)
					{
						DumpMessage("Valid IMEI in the Command\r\n");
						switch (g_rx[7])
						{
							case 'S':
							case 'V':	
								//This is stored in order to restore the mode once the status packet is sent out.
								PREV_Mode = g_config.Mode;
								g_config.Mode = (g_rx[7] == 'S')? SERVER_STATUS_REQ : SERVER_STATUS_REQ_V;
								DumpMessage("Valid Status request Packet.. Sending Back Ack\r\n");

								wm_sprintf(g_traceBuf, "phone number = %s\r\n", SMS_PHN_NUM);
								DumpMessage(g_traceBuf);

								sReturn = adl_smsSend(g_smsHandle, SMS_PHN_NUM, (char *)"OK", ADL_SMS_MODE_TEXT);
								if (sReturn < 0)
									DisplayErrorCode("adl_smsSend", __FILE__, __LINE__, sReturn);
								else
									DumpMessage("ACK SMS transmitted successfully\r\n");							
								break;
							case 'I':
								DumpMessage("I packet request received\r\n");
								DumpMessageUSB("I packet request received\r\n",1);
								//reset g_AvgAnalogVltg so that fresh values are acquired
								g_AvgAnalogVltg = 0;
								//Start reading sensor value.
								if(g_ModeConfig.TxPattern == 0x00)
								{
									StartAnalogSensorReadTimer();
									StartIPktTxTimer();
								}
								break;
							default:
								DumpMessage("Invalid Status request.. Rejected\r\n");
						}
								
					}
					else
						DumpMessage("Invalid IMEI in the Command.. Rejected\r\n");															
				}
				else
				{
					DumpMessage("Invalid the Command.. Rejected\r\n");
				}									
			}
			break;
		case 'O':
			{
				UINT8 Len[9];
				UINT8 FTPAdd[9];
				UINT8 Port[5];
				UINT32 length,PortVal;
				s32 sReturn;				
				wip_in_addr_t tmp_ip;
				wip_in_addr_t final_ip = 0;
				
				
				DumpMessage("Code Upgrade Command Recieved\r\n");
				DumpMessageUSB("Code Upgrade Command Recieved\r\n",1 );

				//Get FTP address.
				memcpy(FTPAdd,&g_rx[3],8);
				tmp_ip = hex_ascii_2_uint(FTPAdd, 32);
				
				// swap the bytes.
				final_ip = (tmp_ip >> 24) & 0xff;
				final_ip |= (tmp_ip >> 8) & 0xff00;
				final_ip |= (tmp_ip << 8) & 0xff0000;
				final_ip |= (tmp_ip << 24) & 0xff000000;

				wm_sprintf(g_traceBuf, "tmp_ip = %x\r\n", (unsigned int)tmp_ip);
				DumpMessage(g_traceBuf);

				if (wip_inet_ntoa(final_ip, &FTPAddress[0], 15) == FALSE)
				{
					return -1;
				}

				wm_sprintf(g_traceBuf, "FTPAddress = %s\r\n", FTPAddress);
				DumpMessage(g_traceBuf);
				DumpMessageUSB(g_traceBuf, 1);

				//Get Port
				memcpy(Port, &g_rx[11],4);
				Port[4] = '\0';							
				PortVal = hex_ascii_2_uint(Port, 16);
				wm_sprintf(g_traceBuf, "Port = %ld\r\n",PortVal);
				DumpMessage(g_traceBuf);				
				DumpMessageUSB(g_traceBuf, 1);

				//Get lenght.

				//Len[0]=g_rx[3];Len[1]=g_rx[4];Len[2]=g_rx[5];Len[3]=g_rx[6];
				//Len[4]=g_rx[7];Len[5]=g_rx[8];Len[6]=g_rx[9];Len[7]=g_rx[10];

				memcpy(Len, &g_rx[15],8);
				Len[8] = '\0';							
				length = hex_ascii_2_uint(Len, 32);
				wm_sprintf(g_traceBuf, "length = %ld\r\n",length);
				DumpMessage(g_traceBuf);
				DumpMessageUSB(g_traceBuf, 1);

				
				DumpMessage("FTP Found DOTA\r\n");
				DumpMessageUSB("FTP Found DOTA\r\n", 1);
				stopTimersFWUpdate();
				recordFileSizeOTA(length);

				StartDOTA();

				StartFTPTimer();
				
				wm_sprintf(g_traceBuf, "phone number = %s\r\n", SMS_PHN_NUM);
				DumpMessage(g_traceBuf);

				sReturn = adl_smsSend(g_smsHandle, SMS_PHN_NUM, (char *)"OK", ADL_SMS_MODE_TEXT);				
			}	
			break;
		case 'M':
			{
				send_ack = parse_ModeConfigPacket();
				if(send_ack >= 0)
				{
					DumpMessage("Mode Config successful\r\n");
					DumpMessageUSB("Mode Config successful\r\n",1);
					memcpy(&g_ModeConfig, &temp_ModeConfig, sizeof(MODE_CONFIG));
					g_ModeConfig.PktRxdProperly = TRUE;

				}
				else
				{
					DumpMessage("Mode Config not successful\r\n");
					DumpMessageUSB("Mode Config not successful\r\n",1);
					g_ModeConfig.PktRxdProperly = FALSE;
				}

				//Store the mode cponfig to flash
				ModeConfigToFlash();

				if(send_ack)
					sms_send('M');
			}
			break;
		case 'D':
			//Evaluate Diagnostic config command.
			parse_DiagnosticPacket();
			break;
		}         /* end switch */
	}

	/* If the packet appeared to be good */
	if (good_packet == TRUE)
	{
		/* Increment good access counter */
		if (g_status.ValidAccessCnt >= 'Z')
		{
			DumpMessage("Reset valid access count\r\n");
			g_status.ValidAccessCnt = 'A';
		}
		else
		{
			g_status.ValidAccessCnt++;
		}
		TRACE((1, "good packet %x", g_tx_status.ValidAccessCnt));
	}
	else
	{
		/* Increment bad access counter */
		if (g_status.BadAccessCnt >= 'Z')
		{
			g_status.BadAccessCnt = 'A';
		}
		else
		{
			g_status.BadAccessCnt++;
		}
		TRACE((1, "bad packet %x", g_status.BadAccessCnt));
	}

	//    PacketRXFastIdleReq = FAST_IDLE_REQ_RELEASE;

	return send_ack;
}


/*F***************************************************************************/

/** @brief Parse Configuration Packet
 *
 * @par
 * Parses the configuration packet and puts the data
 * into the operating variables.
 *
 * @param start_offset - Where the 1st char of the packet is in the buffer
 * @return  >=0 on success -1 on fail due to out of bounds
 * @return  0 = do not acknowledge 1 = acknowledge
 */
static int parse_config_pkt(UINT16 start_offset)
{
	DumpMessage("\r\nParsing a Configure Packet \r\n");
	int ret_val;

	// this will cause the device to ack the config packet or not.
	// return 0 to cause ack.
	// return 1 to not ack.
	if (g_rx[start_offset + PKT_CONFIG_ACK] == 'Y')
	{
		ret_val = 1;
	}
	else if (g_rx[start_offset + PKT_CONFIG_ACK] == 'N')
	{
		ret_val = 0;
	}
	else
	{
		return -1;
	}

	/*************************************************/
	/* SMS or GPRS setting */

	{
		char buffer[24];
		int i;
		wm_sprintf(buffer, "start offset: %d\r\n", start_offset);
		DumpMessage(buffer);
		DumpMessage("start of packet: \r\n");

		for (i = 0; i < PKT_SMS_OR_GPRS; i++)
		{
			wm_sprintf(buffer, "0x%02x, \r\n", g_rx[i]);
			DumpMessage(buffer);
		}
	}

	if (SetSMSorGPRS(&(g_rx[start_offset + PKT_SMS_OR_GPRS]), &temp_config) == -1)
	{
		return -1;
	}

	/*************************************************/
	/* Set Waypoint Interval */
	if (SetWaypointInterval(&(g_rx[start_offset + PKT_WAYPOINT_INT]), &temp_config) == -1)
	{
		return -1;
	}

	/*************************************************/
	/* Set Tracking Interval */
	if (SetTrackingInterval(&(g_rx[start_offset + PKT_TRACKING_INT]), &temp_config) == -1)
	{
		return -1;
	}

	/*************************************************/
	/* Set Motion Alarm Threshold */
	if (SetMotionAlarmThresh(&(g_rx[start_offset + PKT_MOTION_ALARM_THRS]), &temp_config) == -1)
	{
		return -1;
	}

	/*************************************************/
	/* Set Accelerometer Threshold Wake */
	if (SetAccelerometerThreshWake(&(g_rx[start_offset + PKT_ACC_THRS_WAKE]), &temp_config) == -1)
	{
		return -1;
	}

	/*************************************************/
	/* Set Accelerometer Duration Wake */
	if (SetAccelerometerDurWake(&(g_rx[start_offset + PKT_ACC_DUR_WAKE]), &temp_config) == -1)
	{
		return -1;
	}

	/*************************************************/
	/* Set Accelerometer Duration Sleep */
	if (SetAccelerometerDurSleep(&(g_rx[start_offset + PKT_ACC_DUR_SLEEP]), &temp_config) == -1)
	{
		return -1;
	}

	/*************************************************/
	/* Set Bread Crumb Mode */
	if (SetBreadCrumbMode(&(g_rx[start_offset + PKT_BREAD_MODE]), &temp_config) == -1)
	{
		return -1;
	}

	/*************************************************/
	/* Set Mode */
	if (SetMode(&(g_rx[start_offset + PKT_MODE]), &temp_config) == -1)
	{
		return -1;
	}

	/*************************************************/
	/* Set Number of Fences */
	if (SetFenceNumber(&(g_rx[start_offset + PKT_NUM_FENCE]), &temp_config) == -1)
	{
		return -1;
	}

	/*************************************************/
	/* Set GPS Alert */
	if (SetGPSAlert(&g_rx[start_offset + PKT_GPS_ALERT_EN], &temp_config) == -1)
	{
		return -1;
	}

	/*************************************************/
	/* Set GSM Alert Threshold */
	if (SetGSMAlert(&g_rx[start_offset + PKT_GSM_ALERT_THRS], &temp_config) == -1)
	{
		return -1;
	}

	/*************************************************/
	/* Set Battery Alert Threshold */
	if (SetBattAlert(&g_rx[start_offset + PKT_BATT_ALERT_THRS], &temp_config) == -1)
	{
		return -1;
	}

	/*************************************************/
	/* Set Power Disconnect Alert */
	if (SetPowerDisconAlert(&g_rx[start_offset + PKT_PWR_DISCON_ALERT_EN], &temp_config) == -1)
	{
		return -1;
	}

	/*************************************************/
	/* Set Over Speed Alert Threshold */
	if (SetOverSpeedAlertThresh(&g_rx[start_offset + PKT_OVER_SPD_ALERT_THRS], &temp_config) == -1)
	{
		return -1;
	}

	/*************************************************/
	/* Set power down disable */
	if (SetPowerDownDisable(&g_rx[start_offset + PKT_PWR_DOWN_DIS], &temp_config) == -1)
	{
		return -1;
	}

	/*************************************************/
	/* Set tracking mode duration */
	if (SetTrackingModeDuration(&g_rx[start_offset + PKT_TRACKING_MODE_DURATION], &temp_config) == -1)
	{
		return -1;
	}

	/*************************************************/
	/* Set Vibration Motor Pattern */
	if (SetVibrationMotorPattern(&g_rx[start_offset + PKT_VIB_MOTOR_PATT], &temp_config) == -1)
	{
		return -1;
	}

	/*************************************************/
	/* Set Vibration Motor Delay Enable */
	if (SetVibrationMotorDelayEn(&g_rx[start_offset + PKT_VIB_MOTOR_DELAY_EN], &temp_config) == -1)
	{
		return -1;
	}

	/*************************************************/
	/* Set SOS Alert */
	if (SetSOSAlert(&g_rx[start_offset + PKT_SOS_ALERT_EN], &temp_config) == -1)
	{
		return -1;
	}

	/*************************************************/
	/* Set Waypoint Download enable */
	if (SetWaypointDownload(&g_rx[start_offset + PKT_WAYPOINT_DL], &temp_config) == -1)
	{
		return -1;
	}

	/*************************************************/
	/* Set LED Pattern */
	if (SetLEDPattern(&g_rx[start_offset + PKT_LED_PATT], &temp_config) == -1)
	{
		return -1;
	}

	/*************************************************/
	/* Set LED Enable */
	if (SetLEDEnable(&g_rx[start_offset + PKT_LED_EN], &temp_config) == -1)
	{
		return -1;
	}

	/*************************************************/
	/* Set Invisible Operation */
	if (SetInvisibleOp(&g_rx[start_offset + PKT_INVISIBLE_OP_EN], &temp_config) == -1)
	{
		return -1;
	}

	/*************************************************/
	/* Set Magnetic Enable */
	if (SetMagneticSNSEnable(&g_rx[start_offset + PKT_MAG_EN], &temp_config) == -1)
	{
		return -1;
	}

	/*************************************************/
	/* Set Critical Confirmation */
	if (SetCriticalConfirmation(&g_rx[start_offset + PKT_CRIT_CONF], &temp_config) == -1)
	{
		return -1;
	}

	/*************************************************/
	/* Set Server Phone Number */
	if (SetServerPhoneNum(&g_rx[start_offset + PKT_PHONE_NUM], &temp_config) == -1)
	{
		return -1;
	}

	/*************************************************/
	/* Set Server IP Address */
	if (SetServerIPAddr(&g_rx[start_offset + PKT_IP_ADDR], &temp_config) == -1)
	{
		return -1;
	}

	/*************************************************/
	/* Set Server Port */
	if (SetServerPort(&g_rx[start_offset + PKT_PORT_NUM], &temp_config) == -1)
	{
		return -1;
	}

	/*************************************************/
	/* Set Fence Enable\Disable  */
	if (SetFenceEnDisSettings(&g_rx[start_offset + PKT_FENCE_ACT], &temp_config) == -1)
	{
		return -1;
	}

	/*************************************************/
	/* Adjust Fence Parameters */

	wm_sprintf(g_traceBuf, "START TIME=%x STOP TIME=%x\r\n", g_rx[start_offset + PKT_FNC_START_TIME], g_rx[start_offset + PKT_FNC_STOP_TIME]);
	DumpMessage(g_traceBuf);

	if (WriteFenceCoord((ascii *)&g_rx[start_offset + PKT_FENCE_NUM], &g_config, (ascii *)&g_rx[start_offset + PKT_EXCLUSIVE],
	                    (ascii *)&g_rx[start_offset + PKT_POST1_LONG], (ascii *)&g_rx[start_offset + PKT_POST1_LAT],
	                    (ascii *)&g_rx[start_offset + PKT_POST2_LONG], (ascii *)&g_rx[start_offset + PKT_POST2_LAT],
	                    (ascii *)&g_rx[start_offset + PKT_POST3_LONG], (ascii *)&g_rx[start_offset + PKT_POST3_LAT],
	                    (ascii *)&g_rx[start_offset + PKT_POST4_LONG], (ascii *)&g_rx[start_offset + PKT_POST4_LAT],
	                    (ascii *)&g_rx[start_offset + PKT_FNC_START_TIME], (ascii *)&g_rx[start_offset + PKT_FNC_STOP_TIME]) == -1)
	{
		return -1;
	}

	// set the version again.
	temp_config.FirmwareRevNum = FW_MINOR_VER | (FW_MAJOR_VER << 4);
	temp_config.HardwareRevNum = HW_MINOR_VER | (HW_MAJOR_VER << 4);

	return ret_val;     /* packet values checked out so packet is good */
}


/*F***************************************************************************/

/** @brief Convert Temperature to unsigned int
 *
 * @par Converts a temp from GPSTracks protocol to degrees C
 *
 * @param temp_char
 * @return retTemp
 */
SINT8 temp_to_uint(UINT8 temp_char)
{
	SINT8 retTemp = 0;
	/* Set under temp alarm trigger value if server wants to */
	if ((temp_char >= '0') && (temp_char <= '9'))
	{
		retTemp = (-9 + temp_char - '0');
	}
	else if ((temp_char >= 'A') && (temp_char <= 'Z'))
	{
		retTemp = (temp_char - 'A');
	}
	else if ((temp_char >= 'a') && (temp_char <= 'z'))
	{
		retTemp = (27 + temp_char - 'a');
	}

	return retTemp;
}


/*F***************************************************************************/

/** @brief Converts a temperature in degrees C to GPSTracks protocol
 *
 * @par Converts a temperature in degrees C to GPSTracks protocol
 *
 * @param temp - temperature in degrees C
 * @return retTemp
 */
UINT8 uint_to_temp(SINT16 temp)
{
	UINT8 retTemp = 'z';

	if (temp <= -9)
	{
		retTemp = '0';
	}
	else if (temp <= 0)
	{
		retTemp = (temp + '9');
	}
	else if (temp <= 26)
	{
		retTemp = (temp + 'A' - 1);
	}
	else if (temp <= 52)
	{
		retTemp = (temp - 26 + 'a' - 1);
	}
	return retTemp;
}


/*F***************************************************************************/

/** @brief check for latin
 *
 * @par
 * Verify the character is in the latin 1 character set.
 *
 * @param test_char
 * @return 0 if ascii character, 1 otherwise
 */
static int check_latin_1(char test_char)
{
	if (test_char < 0x20)
	{
		return 1;
	}

	if ((test_char > 0x7e) && (test_char < 0xA0))
	{
		return 1;
	}

	return 0;
}


bool last_config_send_ack = FALSE;
static void set_LastConfigSendAck(bool val)
{
	last_config_send_ack = val;
}


bool get_LastConfigSendAck(void)
{
	return last_config_send_ack;
}


void SetInitialConfiguration(void)
{
	char const default_configuration[] = "..CN00100510014002800B40T002N1N002A20NN0YY003F0000FFFFFFFFFFFFFFNN0000000000022580A31FE0A1EDBP0000000000000000000000000000000000000000000000000000000000000000'Q";
	memcpy(g_rx, default_configuration, sizeof default_configuration);
	eval_packet();
}


BOOL parse_DiagnosticPacket()
{
	UINT8 Length;
	UINT8 RepeatCnt;
	UINT8 RepeatInterval;
	ascii ATCommand[20];

	DumpMessage("In parse_DiagnosticPacket\r\n");
	
	Length = hex_ascii_2_uint(&g_rx[3], 8);
	//if(Length < 8 && Length > 20)    //Considering minimum cmd length is 8 and max is 20. (AT&XACT=)
		//return FALSE;
	
	RepeatCnt = hex_ascii_2_uint(&g_rx[5], 8);
	RepeatInterval = hex_ascii_2_uint(&g_rx[7], 8);
	if(RepeatInterval < 60)   //Minimun reporting interval is 60 sec.
		return FALSE;
	
	memcpy(ATCommand,&g_rx[9],Length);
	ATCommand[Length] = '\0';


	//max count is NUM_DIAG_COMMANDS.
	g_DiagCmdOffset %= NUM_DIAG_COMMANDS;	

	g_DiagConfig[g_DiagCmdOffset].ATCmdRepeatCount    = RepeatCnt;
	g_DiagConfig[g_DiagCmdOffset].ATCmdRepeatInterval = RepeatInterval;
	strcpy(g_DiagConfig[g_DiagCmdOffset].ATCommand,ATCommand);
	g_DiagConfig[g_DiagCmdOffset].IsCommandValid = TRUE;

	wm_sprintf(g_traceBuf, "g_DiagCmdOffset : %d\r\n",g_DiagCmdOffset);
	DumpMessage(g_traceBuf);
	
	//Write commands to flash
	DiagConfigToFlash(g_DiagCmdOffset);		

	//Reset the time, whenever a new command is received
	ResetDiagTxTimerCount(g_DiagCmdOffset);

	//Increment the count
	g_DiagCmdOffset++;
		
	//Process the Command
	OTAAT_eval(ATCommand);

	return TRUE;
			
}

int parse_ModeConfigPacket()
{
	int ret_val;

	DumpMessage("\r\nParsing Mode Config Packet \r\n");
	// this will cause the device to ack the config packet or not.
	// return 0 to cause ack.
	// return 1 to not ack.
	if (g_rx[M_ACK] == 'Y')
	{
		ret_val = 1;
	}
	else if (g_rx[M_ACK] == 'N')
	{
		ret_val = 0;
	}
	else
	{
		return -1;
	}

	
	temp_ModeConfig.UseCaseNum 		= hex_ascii_2_uint(&g_rx[USE_CASE],  8);
	if(temp_ModeConfig.UseCaseNum > 0x03)
		return -1;

	temp_ModeConfig.Mode  = g_rx[MODE]; 
	
	temp_ModeConfig.AnalogThreshold = hex_ascii_2_uint(&g_rx[ANA_THRESHOLD], 16);

	temp_ModeConfig.SampleCount 		= hex_ascii_2_uint(&g_rx[SAMPLE_COUNT], 8);

	wm_sprintf(g_traceBuf,"UseCaseNum:%d, AnalogThreshold:%d,SampleCount:%d\r\n", temp_ModeConfig.UseCaseNum,temp_ModeConfig.AnalogThreshold,temp_ModeConfig.SampleCount);
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf,1);

	temp_ModeConfig.GPIO5Config 		= g_rx[GPIO_FIVE]-0x30;  //ascii to integer conversion
	if(temp_ModeConfig.GPIO5Config > 0x01)   //this value can be either 0 or 1
		return -1;
	
	temp_ModeConfig.GPIO7Config 		= g_rx[GPIO_SEVEN]-0x30;  //ascii to integer conversion
	if(temp_ModeConfig.GPIO7Config > 0x01)   //this value can be either 0 or 1
		return -1;

	temp_ModeConfig.I2CDataLen 		= hex_ascii_2_uint(&g_rx[I2C_DATA_LEN], 8);
	if(temp_ModeConfig.I2CDataLen > 0x0F)   //Max lenght supported is 15
		return -1;

	wm_sprintf(g_traceBuf,"GPIO5Config:%d, GPIO7Config:%d,I2CDataLen:%d\r\n", temp_ModeConfig.GPIO5Config,temp_ModeConfig.GPIO7Config,temp_ModeConfig.I2CDataLen);
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf,1);

	temp_ModeConfig.TxPattern  = g_rx[TX_PATTERN] - 0x30;   //ascii to integer conversion

	wm_sprintf(g_traceBuf,"TxPattern :%d\r\n", temp_ModeConfig.TxPattern);
	DumpMessage(g_traceBuf);
	DumpMessageUSB(g_traceBuf,1);

	if(temp_ModeConfig.TxPattern > 0x01)
		return -1;
	
	
	return ret_val;
	
}

void IPacketTxHandler(u8 timerid, void *context)
{
	(void) timerid;
    (void) context;
	s32 sReturn = -1;
	
	DumpMessage("Transmitting I packet\r\n");
	DumpMessageUSB("Transmitting I packet\r\n",1);
	if(g_ModeConfig.TxPattern == 0x01)
	{	
		//Transmit with available mode (GPRS/SMS)
		sms_send('I');
	}
	else
	{
		//Transmit I_PKT_LENGHT of data.
		//Tx as SMS if the incoming request is through SMS
		make_packet('I');
		sReturn = adl_smsSend(g_smsHandle, SMS_PHN_NUM, (char *)g_sms_tx, ADL_SMS_MODE_TEXT);
		if (sReturn < 0)
			DisplayErrorCode("adl_smsSend", __FILE__, __LINE__, sReturn);
		else
			DumpMessage("ACK SMS transmitted successfully\r\n");	
	}

	wm_sprintf(g_traceBuf,"%s\r\n", g_sms_tx);
	DumpMessage(g_traceBuf);
	
}

/**@}*/
