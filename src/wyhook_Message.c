/** @addtogroup TestCode
 *@{ */

/******************************************************************************
 *
 *  wyhook_Message.c     Generic WyHook Message API Layer
 *
 *
 ******************************************************************************
 *
 *	This file is 100% portable and generic and can be used in any embedded project.
 *
 ******************************************************************************
 *
 *	PORT instructions:  HOWTO port the wyhook message layer into your application.
 *
 *	1)	Add wyhook_Message.c/wyhook_Message.h to your project
 *	2)	Refer to wyhook_HAL.c and provide the same functionality in your own hardware layer functions.
 *  3)  Call wyhook_Init() once before your application's main() loop.
 *  4)  If wyhook_InWyhookMode() returns TRUE, call wyhook_Service() from in your application's main() loop.
 *
 ******************************************************************************
 *
 *  MODIFICATION HISTORY
 * -----------------------------------------------------------------------------
 *  Flag | Date       | Author      | Revision | Description
 * ------+------------+-------------+----------+--------------------------------
 *       | 18May2009  | Hendrickson |  0.1     | Creation
 * ------+------------+-------------+----------+--------------------------------
 *
 * *****************************************************************************/

#include "wyhook_Message.h"
#include "wyhook_Application.h"
#include "adl_global.h"
#include "XactUtilities.h"

/***************************************************************************/
/*  Local types                                                            */
/***************************************************************************/

/** @enum tagWYHOOK_PACKET_OFFSET_T
 * @par
 * The offset into the packet to the various peices of data.
 */
typedef enum tagWYHOOK_PACKET_OFFSET_T
{
	OFFSET_COMPONENT_TYPE_ID = 0,
	OFFSET_COMPONENT_SUBTYPE_ID = 1,
	OFFSET_COMMAND = 2,
	OFFSET_COMPONENT_NUMBER = 3,
	OFFSET_PAYLOAD = 4
} WYHOOK_PACKET_OFFSET_T;

/** @enum tagWYHOOK_PACKETIZED_DATA_OFFSET_T
 * @par
 * offset into the packet's payload for the packetized data.
 */
typedef enum tagWYHOOK_PACKETIZED_DATA_OFFSET_T
{
	OFFSET_PACKET_NUMBER = 0,
	OFFSET_TOTAL_PACKETS = 2,
	OFFSET_TOTAL_LENGTH = 4,
	OFFSET_PACKET_PAYLOAD = 6
} WYHOOK_PACKETIZED_DATA_OFFSET_T;

/** @enum tagSLIP_CHARACTERS_T
 * @par
 * Definitions of the SLIP characters used for SLIP packet encoding.
 */
typedef enum tagSLIP_CHARACTERS_T
{
	SLIP_CHAR_END = 192,
	SLIP_CHAR_ESC = 219,
	SLIP_CHAR_ESC_END = 220,
	SLIP_CHAR_ESC_ESC = 221
} SLIP_CHARACTERS_T;

/** @enum tagWYHOOK_COMPONENT_TYPE_ID_T
 * @par
 * Various types of wyhook packets defined.
 */
typedef enum tagWYHOOK_COMPONENT_TYPE_ID_T
{
	TYPE_ENTIRE_DEVICE = 0x00,
	TYPE_RADIO = 0x01,
	TYPE_MEMORY = 0x02,
	TYPE_DIGITAL_INPUT = 0x03,
	TYPE_DIGITAL_OUTPUT = 0x04,
	TYPE_ANALOG_INPUT = 0x05,
	TYPE_ANALOG_OUTPUT = 0x06,
	TYPE_APPLICATION = 0x07
} WYHOOK_COMPONENT_TYPE_ID_T;

/** @enum tagWYHOOK_ENTIRE_DEVICE_SUBTYPES_T
 * @par
 * WyHook packet subTypes for Entire Device.
 */
typedef enum tagWYHOOK_ENTIRE_DEVICE_SUBTYPES_T
{
	SUBTYPE_ENTIRE_DEVICE_EntireDevice = 0x00
} WYHOOK_ENTIRE_DEVICE_SUBTYPES_T;

/** @enum tagWYHOOK_RADIO_SUBTYPES_T
 * @par
 * WyHook packet subTypes for Radio types
 *  */
typedef enum tagWYHOOK_RADIO_SUBTYPES_T
{
	SUBTYPE_RADIO_AllRadios = 0x00,
	SUBTYPE_RADIO_ISMRadio = 0x01,
	SUBTYPE_RADIO_GSMRadio = 0x02,
	SUBTYPE_RADIO_GPSRadio = 0x03,
	SUBTYPE_RADIO_CDMARadio = 0x04,
	SUBTYPE_RADIO_PacketRadio = 0x05,
	SUBTYPE_RADIO_VoiceRadio = 0x06
} WYHOOK_RADIO_SUBTYPES_T;

/** @enum tagWYHOOK_MEMORY_SUBTYPES_T
 * @par
 * WyHook packet subTypes for memory types.
 */
typedef enum tagWYHOOK_MEMORY_SUBTYPES_T
{
	SUBTYPE_MEMORY_AllMemory = 0x00,
	SUBTYPE_MEMORY_VolatileMemory = 0x01,
	SUBTYPE_MEMORY_NonVolatileMemory = 0x02
} WYHOOK_MEMORY_SUBTYPES_T;

/** @enum tagWYHOOK_DIGITAL_INPUT_SUBTYPES_T
 * @par
 * WyHook packet subTypes for Digital Inputs
 */
typedef enum tagWYHOOK_DIGITAL_INPUT_SUBTYPES_T
{
	SUBTYPE_DIGITAL_INPUT_AllDigitalInputs = 0x00,
	SUBTYPE_DIGITAL_INPUT_DigitalSwitches = 0x01,
	SUBTYPE_DIGITAL_INPUT_DigitalGates = 0x02
} WYHOOK_DIGITAL_INPUT_SUBTYPES_T;

/** @enum tagWYHOOK_DIGITAL_OUTPUT_SUBTYPES_T
 * @par
 * WyHook packet subTypes for Digital Outputs
 */
typedef enum tagWYHOOK_DIGITAL_OUTPUT_SUBTYPES_T
{
	SUBTYPE_DIGITAL_OUTPUT_AllDigitalOutputs = 0x00,
	SUBTYPE_DIGITAL_OUTPUT_StatusLED = 0x01,
	SUBTYPE_DIGITAL_OUTPUT_Buzzer = 0x02,
	SUBTYPE_DIGITAL_OUTPUT_8SegmentDisplays = 0x03
} WYHOOK_DIGITAL_OUTPUT_SUBTYPES_T;

/** @enum tagWYHOOK_ANALOG_INPUT_SUBTYPES_T
 * @par
 * WyHook packet subTypes for Analog Inputs
 */
typedef enum tagWYHOOK_ANALOG_INPUT_SUBTYPES_T
{
	SUBTYPE_ANALOG_INPUT_AllAnalogInputs = 0x00,
	SUBTYPE_ANALOG_INPUT_AnalogToDigitalConvertor = 0x01
} WYHOOK_ANALOG_INPUT_SUBTYPES_T;

/** @enum tagWYHOOK_ANALOG_OUTPUT_SUBTYPES_T
 * @par
 * WyHook packet subTypes for Analog outputs.
 */
typedef enum tagWYHOOK_ANALOG_OUTPUT_SUBTYPES_T
{
	SUBTYPE_ANALOG_OUTPUT_AllAnalogOutputs = 0x00,
	SUBTYPE_ANALOG_OUTPUT_Speaker = 0x01,
	SUBTYPE_ANALOG_OUTPUT_Meter = 0x02,
	SUBTYPE_ANALOG_OUTPUT_StatusLight = 0x03
} WYHOOK_ANALOG_OUTPUT_SUBTYPES_T;

/** @enum tagWYHOOK_APPLICATION_SUBTYPES_T
 * @par
 * WyHook packet subTypes for the Application.
 */
typedef enum tagWYHOOK_APPLICATION_SUBTYPES_T
{
	SUBTYPE_APPLICATION_Application = 0x00
} WYHOOK_APPLICATION_SUBTYPES_T;

/** @struct tagWYHOOK_ERROR_COUNTERS_T
 * @par
 * A struct for keeping track of errors for debugging Wyhook itself.
 */
typedef struct tagWYHOOK_ERROR_COUNTERS_T
{
	unsigned char queue_overflows;
	unsigned char SLIP_protocol_errors;
	unsigned char assert_fails;
	unsigned char protocol_errors;
	unsigned char checksum_errors;
} WYHOOK_ERROR_COUNTERS_T;

/***************************************************************************/
/*  Local variables                                                        */
/***************************************************************************/
static WYHOOK_ERROR_COUNTERS_T wyhook_error_counters;

/***************************************************************************/
/*  Local function prototypes                                              */
/***************************************************************************/

/* helper functions */
static void wyhook_IncrementError(unsigned char *pErrorCounter);

/* Wyhook packet data (not SLIP) */
static WYHOOK_BOOL wyhook_IsChecksumValid(WYHOOK_MESSAGE_S *wyhook_msg);

/* Message processing */
static void wyhook_ProcessTypeApplication(WYHOOK_MESSAGE_S *msg);

/***************************************************************************/
/*  Local constants                                                        */
/***************************************************************************/

/***************************************************************************/
/*  Global variables                                                       */
/***************************************************************************/

/** @brief wyhook Increment Error
 *
 * @par Centralized wyhook related error handling.
 * Used for debugging wyhook.
 *
 * @param pErrorCounter
 * @return void
 */
static void wyhook_IncrementError(unsigned char *pErrorCounter)
{
	// make sure we don't try to increment a NULL pointer
	if (pErrorCounter)
	{
		// don't wrap back to zero
		if ((*pErrorCounter) < 0xFF)
		{
			// increment this error counter
			*pErrorCounter = (*pErrorCounter) + 1;
		}
	}
}


/** @brief wyhook Is Checksum Valid
 *
 * @par
 * Checks the WyHook checksum to ensure the packet is good.
 *
 *
 * @param wyhook_msg
 * @return  WYHOOK_TRUE or WYHOOK_FALSE
 */
static WYHOOK_BOOL wyhook_IsChecksumValid(WYHOOK_MESSAGE_S *wyhook_msg)
{
	int checksum;

	/* exit on NULL pointer */
	if (!wyhook_Assert(wyhook_msg != NULL))
	{
		DumpMessage("wyhook message is null\r\n");
		return WYHOOK_FALSE;
	}

	/* if we have a payload size, make sure our payload data pointer is not NULL */
	if (wyhook_msg->payload.size)
	{
		if (!wyhook_Assert(wyhook_msg->payload.pData != NULL))
		{
			DumpMessage("wyhook message payload is null\r\n");
			return WYHOOK_FALSE;
		}
	}

	/* calculate checksum */
	checksum = 0;
	checksum += wyhook_Checksum(&wyhook_msg->header.command, 1);
	checksum += wyhook_Checksum(&wyhook_msg->header.component_type_id, 1);
	checksum += wyhook_Checksum(&wyhook_msg->header.component_subtype_id, 1);
	checksum += wyhook_Checksum(&wyhook_msg->header.component_number, 1);
	checksum += wyhook_Checksum(wyhook_msg->payload.pData, wyhook_msg->payload.size);

	wm_sprintf(g_traceBuf, "checksum = %x\r\n", checksum);
	DumpMessage(g_traceBuf);
	checksum %= 256;

	unsigned char checksum2 = 0;
	int ii;
	checksum2 = wyhook_msg->header.command;
	checksum2 += wyhook_msg->header.component_type_id;
	checksum2 += wyhook_msg->header.component_subtype_id;
	checksum2 += wyhook_msg->header.component_number;
	for (ii = 0; ii < wyhook_msg->payload.size; ii++)
	{
		checksum2 += *(wyhook_msg->payload.pData + ii);
	}

	wm_sprintf(g_traceBuf, "payload size: %d checksum2 = %x\r\n", wyhook_msg->payload.size, checksum2);
	DumpMessage(g_traceBuf);

	wm_sprintf(g_traceBuf, "checksum2 = %x\r\n", checksum2);
	DumpMessage(g_traceBuf);

	if (checksum == wyhook_msg->checksum)
	{
		return WYHOOK_TRUE;
	}
	else
	{
		wm_sprintf(g_traceBuf, "wyhook invalid checksum -- got : %x expected: %x\r\n", checksum, wyhook_msg->checksum);
		DumpMessage(g_traceBuf);
		return WYHOOK_FALSE;
	}
}


/** @brief wyhook Process Packet Buffer
 *
 * @par DataSize, Data
 * Prepares the WyHook packet to be parsed out by the rest of the protocol
 * switches.
 *
 * @param DataSize
 * @param Data
 * @return void
 */
void wyhook_ProcessPacketBuffer(size_t DataSize, unsigned char *Data)
{
	WYHOOK_MESSAGE_S wyhook_msg;

	/* make sure packet is big enough */
	/* assumes minimum payload size of zero */
	if (DataSize < (sizeof (wyhook_msg.header) + sizeof (wyhook_msg.checksum)))
	{
		/* invalid packet size */
		wyhook_IncrementError(&wyhook_error_counters.protocol_errors);
	}
	else
	{
		/* copy the message header out of the packet buffer */
		wyhook_msg.header.component_type_id = Data[OFFSET_COMPONENT_TYPE_ID];
		wyhook_msg.header.component_subtype_id = Data[OFFSET_COMPONENT_SUBTYPE_ID];
		wyhook_msg.header.command = Data[OFFSET_COMMAND];
		wyhook_msg.header.component_number = Data[OFFSET_COMPONENT_NUMBER];

		/* determine payload size */
		wyhook_msg.payload.size = DataSize - sizeof (wyhook_msg.header) - sizeof (wyhook_msg.checksum);

		/* point to the payload */
		wyhook_msg.payload.pData = &Data[OFFSET_PAYLOAD];

		/* copy the checksum */
		wyhook_msg.checksum = Data[DataSize - 1];

		/* verify checksum */
		if (wyhook_IsChecksumValid(&wyhook_msg))
		{
			/* process the message */
			switch (wyhook_msg.header.component_type_id)
			{
			case TYPE_APPLICATION:
				wyhook_ProcessTypeApplication(&wyhook_msg);
				break;

			default:
				/* unsupported command */
				wyhook_IncrementError(&wyhook_error_counters.protocol_errors);
				break;
			}
		}
		else        /* invalid checksum */
		{
			DumpMessage("Invalid WyHook Checksum");
			wyhook_IncrementError(&wyhook_error_counters.checksum_errors);
		}
	}

	/* reset packet size */
	//wyhook_InitRxPacketBuffer();
}


/** @brief wyhook Assert
 *
 * @par
 * Allows a more compact way of calling the Wyhook Error handling.
 * @return test
 */
WYHOOK_BOOL wyhook_Assert(WYHOOK_BOOL test)
{
	if (test == WYHOOK_FALSE)
	{
		wyhook_IncrementError(&wyhook_error_counters.assert_fails);
	}
	return test;
}


/** @brief wyhook Process Type Application
 *
 * @par
 * Processes the Application Type WyHook packet and calls the correct
 * handler.
 *
 * @param msg
 * @return void
 */
static void wyhook_ProcessTypeApplication(WYHOOK_MESSAGE_S *msg)
{
	wyhook_Assert(msg != NULL);     /* validate the pointer is not NULL */

	switch (msg->header.component_subtype_id)
	{
	case SUBTYPE_APPLICATION_Application:
		wyhook_ProcessSubTypeApplication_Application(msg);
		break;

	default:
		/* unknown subtype */
		wyhook_IncrementError(&wyhook_error_counters.protocol_errors);
		break;
	}
}


/** @brief wyhook Checksum
 *
 * @par Standard Wyhook checksum algorithim to check verify integrety.
 *
 * @param buf
 * @param size
 * @return void
 */
unsigned char wyhook_Checksum(unsigned char *buf, unsigned int size)
{
	unsigned int checksum;
	unsigned int i;

	wyhook_Assert(size != 0);
	wyhook_Assert(buf != NULL);

	checksum = 0;
	for (i = 0; i < size; i++)
	{
		checksum += buf[i];
	}

	return (unsigned char)(checksum % 256);
}


/** @brief wyhook Process Packetized Data Buffer
 *
 * @par processes the WyHook payload for parsing by the passed in worker.
 *
 * @param DataSize
 * @param Data
 * @param PacketUser
 * @return void
 */
void wyhook_ProccessPacketizedDataBuffer(size_t DataSize, unsigned char *Data, void (*PacketUser)(WYHOOK_PACKETIZED_DATA_S *packet))
{
	WYHOOK_PACKETIZED_DATA_S packet;

	if (DataSize > sizeof (packet.header))
	{
		/* copy the message header out of the packet buffer */
		packet.header.packet_number = Data[OFFSET_PACKET_NUMBER];
		packet.header.total_packets = Data[OFFSET_TOTAL_PACKETS];
		packet.header.payload_length = Data[OFFSET_PACKET_PAYLOAD];

		/* point to the payload */
		packet.payload.pData = &Data[OFFSET_PACKET_PAYLOAD];

		if (PacketUser != NULL)
		{
			PacketUser(&packet);
		}
	}
}


// <><><><><><><><><><><><><>  end of wyhook_Message.c <><><><><><><><><><><><><><>

/**@}*/
