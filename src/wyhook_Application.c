/** @addtogroup TestCode
 *@{ */

/******************************************************************************
 *
 *  wyhook_Application.c    WyHook Application Commands.
 *
 *
 ******************************************************************************
 *
 *	@brief This file is not portable and should be used to provide project specific
 *	functionality.
 *
 ******************************************************************************
 *
 *  MODIFICATION HISTORY
 * -----------------------------------------------------------------------------
 *  Flag | Date       | Author      | Revision | Description
 * ------+------------+-------------+----------+--------------------------------
 *       | 18Aug2009  | CMurphy     |  0.1     | Creation
 * ------+------------+-------------+----------+--------------------------------
 *
 * *****************************************************************************/

#include "wyhook_Message.h"
#include "wyhook_Application.h"
#include "adl_global.h"
#include "XactUtilities.h"
#include "protocol.h"
#include "DOTAOperation.h"
#include "USB.h"
#include "ConfigSettings.h"
#include "anolclient.h"

// rx buffer.
#define RX_BUFFSIZE    (256)

extern UINT8 g_rx[RX_BUFFSIZE];
extern struct commSocket_s g_USBCommStruct;
extern USB_MODE g_wyhook_mode;
extern UINT8 g_last_config[RX_BUFFSIZE];
extern s8 ubloxDataFcmHandle;
extern s8 g_smsHandle;       //Handle for the sms
//XACT Functions:

void wyhook_XACTApplication_UploadConfig(WYHOOK_MESSAGE_S *msg);
void wyhook_XACTApplication_DownloadConfig(WYHOOK_MESSAGE_S *msg);
void wyhook_XACTApplication_AssistGPS(WYHOOK_MESSAGE_S *msg);
void wyhook_XACTApplication_AssistGPSWorker(WYHOOK_PACKETIZED_DATA_S *packet);

//Required Functions:

/** @brief wyhook Process Sub Type Application
 * @par
 * Once we get an application specific message, we need to figure out
 * who handles it.  And handle errors if they crop up.
 *
 * @param msg
 * @return void
 */
void wyhook_ProcessSubTypeApplication_Application(WYHOOK_MESSAGE_S *msg)
{
	wyhook_Assert(msg != NULL);

	switch (msg->header.command)
	{
	case COMMAND_APPLICATION_XACT_UploadConfig:
		wyhook_XACTApplication_UploadConfig(msg);
		break;

	case COMMAND_APPLICATION_XACT_DownloadConfig:
		wyhook_XACTApplication_DownloadConfig(msg);
		break;

	case COMMAND_APPLICATION_XACT_DisableWyhook:
		g_wyhook_mode = NORMAL_MODE;
		ChangeV24State(USB_COM_ENUM, ADL_FCM_V24_STATE_AT);
		break;

	case COMMAND_APPLICATION_XACT_AssistGPS:
		wyhook_XACTApplication_AssistGPS(msg);
		break;

	default:
		/* unknown subtype */
		//wyhook_IncrementError(&wyhook_error_counters.protocol_errors);
		break;
	}
}


/** @brief wyhook XACT Application UploadConfig
 * @par
 * Send the config sent to us off to be evaluated, and send back
 * a "Success" status.
 *
 * @param msg
 * @return void
 */
void wyhook_XACTApplication_UploadConfig(WYHOOK_MESSAGE_S *msg)
{
	unsigned char packet[] =
	{
		0xC0, 0x07, 0x00, 0x00, 0x00, 0x07, 0xC0
	};
	if (msg->payload.size > 150)
	{
		memcpy(g_rx, msg->payload.pData, msg->payload.size);
		eval_packet();
		DumpMessage("UPLOAD CONFIG!\r\n");
		adl_fcmSendData(g_USBCommStruct.FCM_Handle, packet, 7);
	}
}


/** @brief wyhook XACT Application Download Config
 * @par
 * Grab the current configuration data and send it back to the host.
 * We need to update the Firmware and the Hardware versions to make sure
 * that they are current.
 *
 * @param msg
 * @return void
 */
void wyhook_XACTApplication_DownloadConfig(WYHOOK_MESSAGE_S *msg)
{
	/* FIXME: if we aren't using the message, then don't pass it in -- pjn */
	(void)msg;

	unsigned char packet[175];
	unsigned char packet_data[] =
	{
		0x07, 0x00, 0x01, 0x00
	};

	char tmp_char;
	memcpy(packet, packet_data, 4);
	wm_itohexa((ascii *)&g_last_config[PKT_FW_REV], g_config.FirmwareRevNum, 2);
	tmp_char = g_last_config[PKT_TRACKING_MODE_DURATION];
	wm_itohexa((ascii *)&g_last_config[PKT_HW_REV], g_config.HardwareRevNum, 2);
	g_last_config[PKT_TRACKING_MODE_DURATION] = tmp_char;
	memcpy(&packet[4], g_last_config, 160);
	packet[164] = wyhook_Checksum(packet, 164);
	adl_fcmSendData(g_USBCommStruct.FCM_Handle, packet, 165);
	DumpMessage("Load Config\r\n");
	for(int i=0;i<164;i++)
	{
		wm_sprintf(g_traceBuf,"%c",g_last_config[i]);
		DumpMessage(g_traceBuf);
	}
}


/** @brief wyhook XACT Application Assist GPS
 * @par When we get a WyHook packet for doing AGPS, we need to send it
 * off to get unpacketized before it sent to be processed by the worker.
 *
 *
 * @param msg
 * @return void
 */
void wyhook_XACTApplication_AssistGPS(WYHOOK_MESSAGE_S *msg)
{
	DumpMessage("Wyhook Assit GPS called\r\n");
	wyhook_ProccessPacketizedDataBuffer(msg->payload.size, msg->payload.pData, &wyhook_XACTApplication_AssistGPSWorker);
}


/** @brief wyhook XACT Application Assist GPS Worker
 * @par
 * Sends the decoded APGS packet payload to the u-Blox module for kick starting GPS.
 *
 * @param packet The AGPS packet to be decoded and sent to the UBlox
 * @return void
 */
void wyhook_XACTApplication_AssistGPSWorker(WYHOOK_PACKETIZED_DATA_S *packet)
{
	int ret_val;
	unsigned char resp_packet[10];
	unsigned char resp_packet_data_nack[] =
	{
		0x07, 0x00, 0x05, 0x00, 0x01
	};
	unsigned char resp_packet_data_ack[] =
	{
		0x07, 0x00, 0x05, 0x00, 0x00
	};

	if (packet->header.payload_length > 0)
	{
		//Send payload_length bytes from the packet to the uBlox module.
		if ((ret_val = adl_fcmSendData(ubloxDataFcmHandle,
		                               packet->payload.pData, packet->header.payload_length)) != OK)
		{
			DumpMessage("ERROR: Could not send data to ublox device\r\n");
			memcpy(resp_packet, resp_packet_data_nack, 5);
			set_agps_status(AGPS_NOT_USED);
		}
		else
		{
			DumpMessage("Sending AGPS from wyhook .... \r\n");
			memcpy(resp_packet, resp_packet_data_ack, 5);
			set_agps_status(AGPS_USED);
		}
	}
	else
	{
		DumpMessage("AGPS wyhook: No payload!\r\n");
		memcpy(resp_packet, resp_packet_data_ack, 5);
	}

	//Send the response back to the host.
	resp_packet[5] = wyhook_Checksum(resp_packet, 5);

	adl_fcmSendData(g_USBCommStruct.FCM_Handle, resp_packet, 6);
}


/**@}*/
