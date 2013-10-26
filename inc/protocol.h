/** @file    protocol.h
 *
 *   @brief   protocol header file
 *
 *
 */
#ifndef PROTOCOL_H
#define PROTOCOL_H

/*--------------------------------------------------------------------------
 *   INCLUDES
 *  --------------------------------------------------------------------------*/
#include "pistd.h"
#include "gps.h"
#include "WaypointControl.h"
#include "ConfigSettings.h"

/*--------------------------------------------------------------------------
 *  CONSTANT & MACRO DEFINITIONS
 *  --------------------------------------------------------------------------*/
/*Packet protocol sizes*/
#define CONFIG_PKT_SIZE                 160
#define STATUS_PKT_SIZE                 152	// was 151
#define WAYPOINT_PKT_SIZE               147
#define MODE_CONFIG_PKT_SIZE            18
#define IO_STATUS_PKT_SIZE              62

/* reserved byte 0*/
#define TC_ALERT_PKT_SIZE				125 


/* Packet protocol offsets */
#define PKT_FLAG1                       (0)
#define PKT_FLAG2                       (1)
#define PKT_TYPE                        (2)

/********  Waypoint Data packet offsets ********/
#define PKT_WAY_PKT_NUM                 (3)
#define PKT_WAY_DATA                    (11)

/********  Status packet offsets ********/
#define PKT_BATT_LVL                    (3)
#define PKT_SYS_FAIL                    (4+1)
#define PKT_GPS_STATE                   (5+1)
#define PKT_GPS_SIG1                    (6+1)
#define PKT_GPS_SIG2                    (7+1)
#define PKT_NUM_BIRDS                   (8+1)
#define PKT_GSM_STATE                   (9+1)
#define PKT_GSM_SIG                     (10+1)
#define PKT_CELL_ID                     (11+1)
#define PKT_LOC_AREA                    (15+1)
#define PKT_VALID_ACCESS_CNT            (19+1)
#define PKT_BAD_ACCESS_CNT              (20+1)
#define PKT_STATUS_NUM_FENCE            (21+1)
#define PKT_STATUS_MODE                 (23+1)
#define PKT_SOS_ALARM                   (24+1)
#define PKT_GPS_ALARM                   (25+1)
#define PKT_BATT_ALARM                  (26+1)
#define PKT_GSM_ALARM                   (27+1)
#define PKT_FENCE_ALARM                 (28+1)
#define PKT_CUR_FENCE                   (29+1)
#define PKT_REBOOT_ALARM                (31+1)
#define PKT_OVER_SPD_ALARM              (32+1)
#define PKT_MOTION_ALARM                (33+1)
#define PKT_PWR_DISCON_ALARM            (34+1)
#define PKT_FIX1_TIME                   (35+1)
#define PKT_FIX1_LAT                    (43+1)
#define PKT_FIX1_LONG                   (51+1)
#define PKT_FIX1_EPE                    (59+1)
#define PKT_FIX1_SPEED                  (61+1)
#define PKT_FIX2_TIME                   (69+1)
#define PKT_FIX2_LAT                    (77+1)
#define PKT_FIX2_LONG                   (85+1)
#define PKT_FIX2_EPE                    (93+1)
#define PKT_FIX2_SPEED                  (95+1)
#define PKT_FIX3_TIME                   (103+1)
#define PKT_FIX3_LAT                    (111+1)
#define PKT_FIX3_LONG                   (119+1)
#define PKT_FIX3_EPE                    (127+1)
#define PKT_FIX3_SPEED                  (129+1)
#define RTC_TIME_START					(137+1)
#define RTC_TIME_END					(144+1)
#define SIERRA_VER_START				(145+1)
#define SIERRA_VER_END					(149+1)
#define AGPS_STAT						(150+1)


/********  Configure packet offsets ********/
#define PKT_CONFIG_ACK                  (3)
#define PKT_FENCE_NUM                   (4)
#define PKT_SMS_OR_GPRS                 (6)
#define PKT_WAYPOINT_INT                (7)
#define PKT_TRACKING_INT                (10)
#define PKT_MOTION_ALARM_THRS           (11)
#define PKT_ACC_THRS_WAKE               (13)
#define PKT_ACC_DUR_WAKE                (15)
#define PKT_ACC_DUR_SLEEP               (19)
#define PKT_BREAD_MODE                  (23)
#define PKT_MODE                        (24)
#define PKT_NUM_FENCE                   (25)
#define PKT_GPS_ALERT_EN                (27)
#define PKT_GSM_ALERT_THRS              (28)
#define PKT_BATT_ALERT_THRS             (29)
#define PKT_PWR_DISCON_ALERT_EN         (30)
#define PKT_OVER_SPD_ALERT_THRS         (31)
#define PKT_FW_REV                      (33)
#define PKT_HW_REV                      (35)
#define PKT_RESERVED                    (37)
#define PKT_PWR_DOWN_DIS                (38)
#define PKT_TRACKING_MODE_DURATION      (46)
#define PKT_VIB_MOTOR_PATT              (39)
#define PKT_VIB_MOTOR_DELAY_EN          (40)
#define PKT_SOS_ALERT_EN                (41)
#define PKT_WAYPOINT_DL                 (42)
#define PKT_LED_PATT                    (43)
#define PKT_LED_EN                      (44)
#define PKT_TIME_WAKEUP                 (46)
#define PKT_FENCE_ACT                   (50)
#define PKT_INVISIBLE_OP_EN             (64)
#define PKT_MAG_EN                      (65)
#define PKT_CRIT_CONF                   (66)
#define PKT_PHONE_NUM                   (67)
#define PKT_IP_ADDR                     (81)
#define PKT_PORT_NUM                    (89)
#define PKT_EXCLUSIVE                   (93)
#define PKT_POST1_LAT                   (94)
#define PKT_POST1_LONG                  (102)
#define PKT_POST2_LAT                   (110)
#define PKT_POST2_LONG                  (118)
#define PKT_POST3_LAT                   (126)
#define PKT_POST3_LONG                  (134)
#define PKT_POST4_LAT                   (142)
#define PKT_POST4_LONG                  (150)
#define PKT_FNC_START_TIME              (158)
#define PKT_FNC_STOP_TIME               (159)


/************* IO Status Packet Offsets ****************/
#define IO_RTC_TIME_STAMP               (5)
#define PANIC                           (13)
#define I2C_SENSOR                      (14)
#define GPIO5_STATUS                    (18)
#define GPIO7_STATUS                    (19)
#define ANALOG_VALUE                    (20)
#define BATTERY_VLTG                    (24)
#define MAGENTIC_SENSOR                 (28)
#define ACCELEROMETER                   (29)
#define BOARD_TEMP                      (33)
#define IGN_STATUS                      (37)
#define ENG_ON_TIME                     (38)
#define ENG_OFF_TIME                    (46)      
#define RUN_TIME                        (54)   
#define I_PKT_LENGHT                    (62)   //This has to be modified if any new field is added.					



/************** M Packet Offsets ********************/
#define M_ACK                           (3)
#define USE_CASE                        (4)
#define MODE                            (6)
#define ANA_THRESHOLD                   (7)
#define SAMPLE_COUNT                    (11)
#define GPIO_FIVE                       (13)
#define GPIO_SEVEN                      (14)
#define I2C_DATA_LEN                    (15)
#define TX_PATTERN                      (17)


/*************** Atmel alert packet offsets *******************/
#define TC_ALERT_NIBBLE_3				(3)
#define TC_ALERT_NIBBLE_2				(4)
#define TC_ALERT_NIBBLE_1				(5)
#define TC_ALERT_NIBBLE_0				(6)

#define TC_CNTR_NIBBLE_1				(7)
#define TC_CNTR_NIBBLE_0				(8)

#define TC_VOLTPCT_NIBBLE_1				(9)
#define TC_VOLTPCT_NIBBLE_0				(10)

#define TC_PKT_GPS_STATE				(11)
#define TC_PKT_GPS_SIG1					(12)
#define TC_PKT_GPS_SIG2					(13)
#define TC_PKT_NUM_BIRDS				(14)
#define TC_PKT_FIX1_TIME				(15)
#define TC_RTC_TIME_START				(117)			// AEW Jr.
#define TC_RTC_TIME_END					(124)


/*--------------------------------------------------------------------------
 * TYPE DEFINITIONS
 * -------------------------------------------------------------------------*/
typedef struct
{
	int packet_num;
	WAYPOINT waypoint1;
	WAYPOINT waypoint2;
	WAYPOINT waypoint3;
	WAYPOINT waypoint4;
} SMS_WAYPOINT_PAYLOAD;

/*--------------------------------------------------------------------------
 *  FUNCTION PROTOTYPES
 *  --------------------------------------------------------------------------*/
void make_packet(UINT8 pckt_type);
int eval_packet(void);

//SINT8 temp_to_uint(UINT8);
//UINT8 uint_to_temp(SINT16);
extern UINT8 send_sms_waypoint(WAYPOINT_DL waypoint_amount);
bool get_LastConfigSendAck(void);
void SetInitialConfiguration();
BOOL parse_DiagnosticPacket();
int parse_ModeConfigPacket();
void IPacketTxHandler(u8 timerid, void *context);

#endif
