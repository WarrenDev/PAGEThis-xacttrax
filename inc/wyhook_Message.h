/******************************************************************************
 *
 *  wyhook_Message.h     WyHook Message API Layer
 *
 *
 ******************************************************************************
 *
 *	This file is 100% portable and generic and can be used in any embedded project.
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

#ifndef _WYHOOK_MESSAGE_H_
#define _WYHOOK_MESSAGE_H_

#include "wm_types.h"

/***************************************************************************/
/*  Global types                                                           */
/***************************************************************************/
#ifndef NULL
#define NULL    (void *)0
#endif

typedef enum _WYHOOK_BOOL
{
	WYHOOK_FALSE = 0,
	WYHOOK_TRUE = 1
} WYHOOK_BOOL;   // Undefined size

typedef enum _WYHOOK_ERROR_T
{
	WYHOOK_ERR_NONE = 0,
	WYHOOK_ERR_PARAM = -1,
	WYHOOK_ERR_HARDWARE_FAIL = -2,
	WYHOOK_ERR_UNKNOWN = -3
} WYHOOK_ERROR_T;

typedef struct tagWYHOOK_MESSAGE_S
{
	struct
	{
		unsigned char component_type_id;
		unsigned char component_subtype_id;
		unsigned char command;
		unsigned char component_number;
	} header;
	struct
	{
		unsigned char size;
		unsigned char   *pData;
	} payload;
	unsigned char checksum;
} WYHOOK_MESSAGE_S;

typedef struct tagWYHOOK_TX_PACKET_S
{
	unsigned char size;
	unsigned char index;
	unsigned char checksum;
	WYHOOK_BOOL transmit_packet;
	unsigned char buf[513];
} WYHOOK_TX_PACKET_S;

typedef struct tagWYHOOK_PACKETIZED_DATA_S
{
	struct
	{
		unsigned short packet_number;
		unsigned short total_packets;
		unsigned short payload_length;
	} header;
	struct
	{
		unsigned char *pData;
	} payload;
} WYHOOK_PACKETIZED_DATA_S;

/***************************************************************************/
/*  Global functions                                                       */
/***************************************************************************/
extern WYHOOK_BOOL wyhook_Assert(WYHOOK_BOOL test);
void wyhook_ProcessPacketBuffer(size_t DataSize, unsigned char *Data);

void wyhook_ProccessPacketizedDataBuffer(size_t DataSize, unsigned char *Data, void (*PacketUser)(WYHOOK_PACKETIZED_DATA_S *packet));
unsigned char wyhook_Checksum(unsigned char *buf, unsigned int size);

#endif
