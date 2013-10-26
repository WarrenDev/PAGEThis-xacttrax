/*H***********************************************************************
 */

/*! \file    SocketDescriptor.h
 *
 *   \brief   Code that provides a structure for socket file descriptors
 *
 *   \details This code is being used to create a socket interface between Open AT and Python.
 *
 *   \usage   Used whenever you need to execute a file descriptor command.
 *
 *   \note    This is only loosely based on Unix File Descriptors and is incomplete.
 *
 *
 *//*
 *
 * CHANGES:
 *
 * REF NO  DATE     WHO      DETAIL
 *         02JUL08  PJN      First version
 * NNNNNN  DDMMMYY  Name     Name of function/item changed
 *
 *H*/

#ifndef _SOCKETDESCRIPTOR_H
#define _SOCKETDESCRIPTOR_H

/**************************************************************************
 * INCLUDE FILES
 ***************************************************************************/
/*---- system and platform files -----------------------------------------*/
//#include <filename.h> /* header files not defined within the program */
#include "adl_global.h"
//#include "wip.h"
/*---- program files -----------------------------------------------------*/

//#include "program.h"  /* 'global' header file for program */
//#include "filename.h" /* other files required */
//#include "socketHelper.h"
//#include "zigbeeHelper.h"
//#include "zbs_internals.h"
//#include "filedesc.h"

/**************************************************************************
 * FILE CONTEXT
 ***************************************************************************/

/*A****************************************************************
 * NAME: Simple name for functional area served by this section
 *
 * USAGE: Description of where, how, etc. items below are used
 *
 * NOTES: Other information to help the reader.
 *
 * CHANGES :
 * REF NO  DATE      WHO     DETAIL
 *         DDMMMYY   Name    First version
 * XXNNNN  DDMMMYY   Name    Name of item changed
 *
 *A*/
/*---- context ---------------------------------------------------*/

/*! \struct socketServer_s
 *   \brief The structure that contains file descriptor information */
//struct inetSocket_s{
//	wip_channel_t socketChannel;		/*!< The pointer to the channel that is represented by the fd */
//	u8 s_flags;					/*!< 8 bit register to indicate the status of the socket:\n
//										 *	 bit 7 -- 1 - Server
//										 *	          0 - Client
//										 *	 bit 6 -- 1 - nonblocking
//										 *	          0 - blocking
//										 *	 bit 5 -- 1 - peer closed
//										 *	          0 - peer not closed
//										 *	 bit 4 -- 1 - writable
//										 *	          0 - not writable
//										 *	 bit 3 -- 1 - readable
//										 *	          0 - not readable
//										 *	 bit 2 -- 1 - error
//										 *	          0 - no err
//										 *	 bit 1 -- 1 - bound
//										 *	          0 - not bound
//										 *	 bit 0 -- 1 - Is initialized
//										 *	          0 - not initialized */
//
//	struct sockaddr_in addr;
//	int addr_len;
//	s16 sock_ctx;							//Only used for server socket?
//	s32 spawnedQueue;					//!< A handle for the queue that contains the fileDescriptors
//										     of the spawned channels for server sockets  */
//	int* s_queue_mutex;
//	int* s_flags_mutex;
//};

#define SOCKET_INITIALIZED      0x01
#define SOCKET_BOUND            0x02
#define SOCKET_ERR              0x04
#define SOCKET_READABLE         0x08
#define SOCKET_WRITEABLE        0x10
#define SOCKET_PEER_CLOSED      0x20
#define SOCKET_BLOCKING         0x40
#define SOCKET_SERVER           0x80

struct zigbeeSocket_s
{
	struct zbs_context *context;
};

enum commPort_e
{
	UART1_COM_ENUM=1,
	UART2_COM_ENUM,
	USB_COM_ENUM
};

typedef enum commPort_e commPort_type;

struct commSocket_s
{
	commPort_type portType;
	u8 socketFlags;                     /*!< 8 bit register to indicate the status of the socket:\n
	                                     *	 bit 7 -- 1 -
	                                     *	          0 -
	                                     *	 bit 6 -- 1 -
	                                     *	          0 -
	                                     *	 bit 5
	                                     *	 bit 4 -- 1 -
	                                     *	          0 -
	                                     *	 bit 3 -- 1 -
	                                     *	          0 -
	                                     *	 bit 2 -- 1 - exception
	                                     *	          0 - no exception
	                                     *	 bit 1 -- 1 - writeable
	                                     *	          0 - not writeble
	                                     *	 bit 0 -- 1 - nonblocking
	                                     *	          0 - blocking */
	s8 FCM_Handle;
	char            *rcvData;
	int rcvDataSize;
	s8 DownloadFlag;
};

/*---- data descriptions -------------------------------------------------*/
//typedef struct inetSocket_s   inetSocketStruct;
//typedef struct zigbeeSocket_s zigbeeSocketStruct;

//typedef struct commSocket_s   commSocketStruct;

/*---- extern data declarations ------------------------------------------*/
//extern int gfdcount;
//extern int selectValue;	/*!< This value is used to globally store the previous value of select (Probably should be somewhere else)*/

/*---- extern function prototypes ----------------------------------------*/
//extern FunctionName( type Parm1, type Parm2 );
//int dup(int filedes);
//extern descNum_t SearchFDUsingChannel(wip_channel_t i_channel);

/*
 * //Clears the bit for the file descriptor fd in the file descriptor set fdset.
 * void FD_CLR(int fd, filedes_s *fdset);
 *
 * // Returns a non-zero value if the bit for the file descriptor fd is set in the file descriptor set by fdset, and 0 otherwise.
 * int FD_ISSET(int fd, filedes_s *fdset);
 *
 * //Sets the bit for the file descriptor fd in the file descriptor set fdset.
 * void filedes_s(int fd, filedes_s *fdset);
 *
 * //Initializes the file descriptor set fdset to have zero bits for all file descriptors.
 * void FD_ZERO(filedes_s *fdset);
 */
#endif /* FILEDESCRIPTOR_H ------ END OF FILE ------*/
