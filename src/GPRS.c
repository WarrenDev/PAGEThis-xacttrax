/** @addtogroup NetworkComm 
 *@{*/
#include <wip_bearer.h> 

#include <adl_global.h>

#include "GPRS.h"
#include "protocol.h"
#include "ConfigSettings.h"
#include "DOTAOperation.h"
#include "PowerCtrl.h"
#include "XactUtilities.h"
#include "InternalFlash.h"
#include "Timers.h"
//#include "Anolclient.h"

#define RCV_BUFFER_SIZE 10240
#define CREG_POLLING_PERIOD 20 /* in 100ms steps */
#define GPRS_PINCODE  "0000"

#define MAX_TCP_TRIES 10
#define MAX_BEARER_FAILS 3


static void evh( wip_event_t *ev, void *ctx);
static void FTPevh( wip_event_t *ev, void *ctx);


static bool poll_creg_callback(adl_atResponse_t *Rsp);
static void poll_creg( u8 Id , void * pfContext);
static void ConnectGPRS(void);
static wip_channel_t socket;
static int HaveBearer = 0;

int g_DOTAUpdateMode = 0;

static s8 gWipBearerHandle=-1;

BOOL NetworkError = FALSE;

s8 evhGPRSHandler( u16 Event, u8 CID );
static void launch_app_timer(u8 Id, void *Context);
static void  tx_timer(u8 Id, void *Context);
static int GPRSInit = 0;
void ReInitBearer(u8 timerID, void *context);
void InitBearer(void);
// the transmit buffer.
extern UINT8 g_sms_tx[];
// the receive buffer.
extern UINT8 g_rx[];
extern ascii g_IMEI[];
extern ascii FTPAddress[15];

// control if we should transmit over the tcp socket.
int g_wip_transmit_trig = 0;
int g_wip_transmit_size = 0;
int g_DOTAOK = 0;

static char rcv_buffer [RCV_BUFFER_SIZE];
static int  rcv_offset = 0;

static int TCPTryCount = 0;
static int FTPTryCount = 0;


static TCP_STATUS TCPStatus = TCP_NOT_INIT;

unsigned short int ntohs(unsigned short int netshort);
int ReverseEndian(u8 *buffer, int length);

/* bearer handle */ 
wip_bearer_t myBearer; 

static TCP_OR_UDP connection_type = TCP_CONNECTION_TYPE;

void appli_entry_point(void); 

/* return the type of connection TCP or UDP */
TCP_OR_UDP GPRS_GetConnectionType(void)
{
  return connection_type;
}

int GPRS_HaveBearer(void)
{
  return HaveBearer;
}

/* bearer events handler */ 
static int ber_fail_cnt = 0;

/** @brief Event handler for the bearer.
*
* @param br
* @param event
* @param context
* @return void
*/
void myHandler( wip_bearer_t br, s8 event, void *context) 
{ 
  (void) br;
  (void) context;

  s8 wipErrorValue = -1;
  switch( event) { 
  case WIP_BEV_IP_CONNECTED: 
    DumpMessage("WIP_BEV_IP_CONNECTED\r\n");
    GPRSInit = 1;
    HaveBearer = 1;
    appli_entry_point();
    /*IP connectivity we can start IP application from here*/ 
    break; 
  case WIP_BEV_IP_DISCONNECTED: 
    DumpMessage("WIP_BEV_IP_DISCONNECTED\r\n");
    TCPStatus = TCP_CANT_CONNECT;
    HaveBearer = 0;
    /*stop IP application*/ 
    break; 
    /* other events: */ 
  case WIP_BEV_CONN_FAILED:
    DumpMessage( " WIP_BEV_CONN_FAILED\r\n ");
    HaveBearer = 0;
    if (ber_fail_cnt == MAX_BEARER_FAILS) {
      TCPStatus = TCP_CANT_CONNECT;
      ber_fail_cnt = 0;
    }
    else {
      ber_fail_cnt++;
    }
    wip_bearerGetOpts(myBearer, WIP_BOPT_ERROR,&wipErrorValue, WIP_BOPT_END);
    switch(wipErrorValue)
      {
      case WIP_BERR_LINE_BUSY: 
	DumpMessage("WIP_BERR_LINE_BUSY\r\n");
	break;
      case WIP_BERR_NO_ANSWER: 
	DumpMessage("WIP_BERR_NO_ANSWER\r\n");
	break;
      case WIP_BERR_NO_CARRIER: 
	DumpMessage("WIP_BERR_NO_CARRIER\r\n");
	break;
      case WIP_BERR_NO_SIM: 
	DumpMessage("WIP_BERR_NO_SIM\r\n");
	break;
      case WIP_BERR_PIN_NOT_READY: 
	DumpMessage("WIP_BERR_PIN_NOT_READY\r\n");
	break;
      case WIP_BERR_GPRS_FAILED: 
	DumpMessage("WIP_BERR_GPRS_FAILED\r\n");
	break;
      case WIP_BERR_PPP_LCP_FAILED: 
	DumpMessage("WIP_BERR_LCP_FAILED\r\n");
	break;
      case WIP_BERR_PPP_AUTH_FAILED: 
	DumpMessage("WIP_BERR_PPP_AUTH_FAILED\r\n");
	break;
      case WIP_BERR_PPP_IPCP_FAILED: 
	DumpMessage("WIP_BERR_IPCP_FAILED\r\n");
	break;
      case WIP_BERR_PPP_LINK_FAILED: 
	DumpMessage("WIP_BERR_PPP_LINK_FAILED\r\n");
	break;
      case WIP_BERR_PPP_TERM_REQ:
	DumpMessage("WIP_BERR_PPP_TERM_REQ\r\n");
	break;
      case WIP_BERR_CALL_REFUSED:
	DumpMessage("WIP_BERR_CALL_REFUSED\r\n");
	break;
      default:
	DumpMessage("Unknown error\r\n");
	break;		
      }

    adl_tmrSubscribe(FALSE, 50, ADL_TMR_TYPE_100MS, ReInitBearer);

    break;
  default: 
    DumpMessage("default\r\n");
    HaveBearer = 0;
    break; 
  } 
} 
 

/** @brief Connect to GPRS 
*
* @par initialize and start GPRS bearer 
* @return ret_val
*/
int myConnectToGPRS( void ) 
{ 
  s32 ret_val;
  /* open bearer and install our event handler */ 
  if (!HaveBearer) {
    if( (ret_val = wip_bearerOpen( &myBearer, "GPRS", myHandler, NULL)) != 0) { 
      DisplayErrorCode("wip_bearerOpen",__FILE__,__LINE__,ret_val);
      /* cannot open bearer */ 
      return -1;
    } 

  char *apn_name = InternalFlash_GetAPNName();
  // set the default if NULL
  if (apn_name == NULL)  {
    InternalFlash_WriteAPNName("orbcomm.t-mobile.com");
    apn_name = InternalFlash_GetAPNName();
  }
  char *apn_login = InternalFlash_GetAPNLogin();
  // set the default if NULL
  if (apn_login == NULL) {
    InternalFlash_WriteAPNLogin("");
    apn_login = InternalFlash_GetAPNLogin();
  }
  char *apn_password = InternalFlash_GetAPNPassword();
  // set the default if NULL
  if (apn_password == NULL) {
    InternalFlash_WriteAPNPassword("");
    apn_password = InternalFlash_GetAPNPassword();
  }

  /* configure GPRS interface */ 
  if( wip_bearerSetOpts ( myBearer, 
			  WIP_BOPT_GPRS_APN, apn_name,
			  WIP_BOPT_LOGIN,    apn_login,
			  WIP_BOPT_PASSWORD, apn_password,
			  WIP_BOPT_END) != 0) 

#if 0
    if( wip_bearerSetOpts ( myBearer, 
			    WIP_BOPT_GPRS_APN, "telargo.t-mobile.com",
			    WIP_BOPT_LOGIN,    "Spe9f5d",
			    WIP_BOPT_PASSWORD, "601A4823",
			    WIP_BOPT_END) != 0) 

    if( wip_bearerSetOpts ( myBearer, 
			    WIP_BOPT_GPRS_APN, "orbcomm.t-mobile.com",
			    WIP_BOPT_LOGIN,    "",
			    WIP_BOPT_PASSWORD, "",
			    WIP_BOPT_END) != 0) 
#endif
      {
      /* cannot configure bearer */ 
      wip_bearerClose(myBearer); 
      return -2;
      } 
    /* start connection */ 
    ret_val = wip_bearerStart( myBearer);

    if(ret_val != OK && ret_val != WIP_BERR_OK_INPROGRESS) {
      DisplayErrorCode("wip_bearerStart",__FILE__,__LINE__,ret_val);
      /* cannot start bearer */ 
      wip_bearerClose( myBearer); 
      return -3;
    } 
  }
  else {
    // try to start the application level code.
    adl_tmrSubscribe( FALSE, 10, ADL_TMR_TYPE_100MS,
		      (adl_tmrHandler_t)launch_app_timer);
  }

  /* connection status will be reported to the event handler */ 
  return 0;
}


/** @brief Called once the WIP IP stack is fully initialized.
*
* @par description: Called once the WIP IP stack is fully initialized. 
* This is the starting point of user applications.
* @return void
*/
void appli_entry_point(void) {

  u16 mport = g_config.ServerPortNum;

  TCPStatus = TCP_TRYING;

  wip_debug( "[SAMPLE]: connecting to client %s:%i...\n", g_config.ServerIPAddr, g_config.ServerPortNum);

  // TCP Mode.
  if (g_config.SMSorGPRS==GPRS_TCP_ROAMING || g_config.SMSorGPRS==GPRS_TCP_FALLBACK_ROAMING ||
      g_config.SMSorGPRS==GPRS_TCP_NO_ROAMING || g_config.SMSorGPRS==GPRS_TCP_FALLBACK_NO_ROAMING) {
    
    connection_type = TCP_CONNECTION_TYPE;
    socket = wip_TCPClientCreate( g_config.ServerIPAddr, g_config.ServerPortNum, evh, NULL);


  } // UDP Mode.
  else {
    connection_type = UDP_CONNECTION_TYPE;

    // What is going on here? why does mport need to be passed by reference.
    socket = wip_UDPCreateOpts(evh,NULL,
			       WIP_COPT_PORT,&mport,
			       WIP_COPT_END);

  }

  if(!socket) {
    DisplayErrorCode("wip_TCPClientCreate",__FILE__,__LINE__,(int )socket);
  }
  else {
    TCPStatus = TCP_TRYING;
    DumpMessage("Socket open!\r\n");
    TRACE((1,"Socket opened succesfully\r\n"));
  }
}


/** @brief network to host host short
*
* @param netshort
* @return netshort
*/
unsigned short int ntohs(unsigned short int netshort)
{
	ReverseEndian((u8*)&netshort,2);
	return netshort;
}

/** @brief perform reverse endian
*
* @param buffer
* @param length
*
* @return 0
*/
int ReverseEndian(u8 *buffer, int length)
{
    int count;
    char temp;
        for(count = 0; count <length/2; count++)
        {
            temp = buffer[length - 1 - count];
            buffer[length - 1 - count] = buffer[count];
            buffer[count] = temp;
        }
        return 0;
}


/** @brief re-Initialize the Bearer 
*
* @param timerID
* @param context
*
* @return void
*/
void ReInitBearer(u8 timerID, void *context)
{
    (void) timerID;
    (void) context;
	s8 retCode = -1;

	retCode = wip_bearerClose(myBearer);
	switch(retCode)
	{
	case 0 :
	  DumpMessage("wip_bearerClose: No error\n");
	  break;
	case WIP_BERR_BAD_HDL:
	  DumpMessage("wip_bearerClose: ERROR: WIP_BERR_BAD_HDL\n");
	  break;
	case WIP_BERR_BAD_STATE:
	  DumpMessage("wip_bearerClose: ERROR: WIP_BERR_BAD_STATE\n");
	  break;
	default:
	  DumpMessage("wip_bearerClose: ERROR: Unknown Error\n");
	  break;
	}	
	InitBearer();
}

/** @brief Initialize the Bearer 
*
* @return void
*/
void InitBearer()
{
  s32 ret_val = -1;

  if( (gWipBearerHandle = wip_bearerOpen( &myBearer, "GPRS", myHandler, NULL)) != 0) { 
    if (gWipBearerHandle == WIP_BERR_ALREADY) {
      DumpMessage("Wip Bearer already open\r\n");
    } else {
      DisplayErrorCode("wip_bearerOpen",__FILE__,__LINE__,ret_val);
      return;
    }
  }


  char *apn_name = InternalFlash_GetAPNName();
  // set the default if NULL
  if (apn_name == NULL)  {
    InternalFlash_WriteAPNName("orbcomm.t-mobile.com");
    apn_name = InternalFlash_GetAPNName();
  }
  char *apn_login = InternalFlash_GetAPNLogin();
  // set the default if NULL
  if (apn_login == NULL) {
    InternalFlash_WriteAPNLogin("");
    apn_login = InternalFlash_GetAPNLogin();
  }
  char *apn_password = InternalFlash_GetAPNPassword();
  // set the default if NULL
  if (apn_password == NULL) {
    InternalFlash_WriteAPNPassword("");
    apn_password = InternalFlash_GetAPNPassword();
  }
  

    if( wip_bearerSetOpts ( myBearer, 
			    WIP_BOPT_GPRS_APN, apn_name,
			    WIP_BOPT_LOGIN,    apn_login,
			    WIP_BOPT_PASSWORD, apn_password,
			    WIP_BOPT_END) != 0) 
#if 0
  /* configure GPRS interface */ 

    if( wip_bearerSetOpts ( myBearer, 
			    WIP_BOPT_GPRS_APN, "telargo.t-mobile.com",
			    WIP_BOPT_LOGIN,    "Spe9f5d",
			    WIP_BOPT_PASSWORD, "601A4823",
			    WIP_BOPT_END) != 0) 

    if( wip_bearerSetOpts ( myBearer, 
			    WIP_BOPT_GPRS_APN, "orbcomm.t-mobile.com",
			    WIP_BOPT_LOGIN,    "",
			    WIP_BOPT_PASSWORD, "",
			    WIP_BOPT_END) != 0) 
#endif
      {
    /* cannot configure bearer */ 
	wip_bearerClose(myBearer); 
	return;
      } 

  /* start connection */ 
  ret_val = wip_bearerStart( myBearer);

  if(ret_val != OK && ret_val != WIP_BERR_OK_INPROGRESS) {
    DisplayErrorCode("wip_bearerStart",__FILE__,__LINE__,ret_val);
    /* cannot start bearer */ 
    wip_bearerClose(myBearer); 
    return;
  }

}

/** @brief Handling events happenning on the TCP client socket.
*
* @param ev
* @param ctx
* @return void
*/
static void evh( wip_event_t *ev, void *ctx) {

  (void) ctx;
  
  static int DOTASize = 0;
  static int FoundDOTA = 0;
  static int CheckedDOTA = 0;
  static int FirstDOTA = 0;
  static int DOTABytes = 0;

  switch( ev->kind) {

  case WIP_CEV_OPEN: {
    DumpMessage ("[SAMPLE] Connection established successfully\r\n");
    if (connection_type == TCP_CONNECTION_TYPE) {
      wip_setOpts( ev->channel, WIP_COPT_RCV_LOWAT, 1,WIP_COPT_RCV_BUFSIZE,10000,WIP_COPT_END);
    }
    TCPTryCount = 0;
    TCPStatus = TCP_CONNECT;

	
	//Start AGPS server
	//DumpMessage("Starting AGPS server\r\n");
	//DumpMessageUSB("Starting AGPS server\r\n",1);
	//performAGPS(0.0, 0.0, 0.0, 0.0);	
	
    break;
  }
  case WIP_CEV_READ: {
    int nread;

    DumpMessage ("[SAMPLE] Some data arrived\n");

    if (connection_type==TCP_CONNECTION_TYPE) {

      nread = wip_read( ev->channel, rcv_buffer + rcv_offset, 
			sizeof( rcv_buffer) - rcv_offset);
      if( nread < 0) { wip_debug( "[SAMPLE] read error %i\n", nread); return; }

      rcv_offset += nread;
      if( rcv_offset == sizeof( rcv_buffer)) {
	DumpMessage( "[SAMPLE] Reception capacity exceeded, won't read more\n");
      } else {
	wip_debug( "[SAMPLE] Wrote %i bytes of data from network to rcv_buffer. "
		   "%i bytes remain available in rcv_buffer\n",
		   nread, sizeof( rcv_buffer) - rcv_offset);
      }

      // check if we are going to get a DOTA packet.
      // first 3 bytes are the tag. Next 4 bytes are the size.
      if (rcv_offset >= 7 && !CheckedDOTA) {
	CheckedDOTA = 1;
	if (rcv_buffer[0]=='.' && rcv_buffer[1]=='.' && rcv_buffer[2]=='D') {
	  DumpMessage("Found DOTA\r\n");
	  FoundDOTA = 1;
	  g_DOTAUpdateMode = 1;
	  DOTASize = rcv_buffer[3] | (rcv_buffer[4] << 8) | (rcv_buffer[5] << 16) | (rcv_buffer[6] << 24);
	  FirstDOTA = 1;
	  wm_sprintf(g_traceBuf,"DOTASize=%d nread=%d\r\n",DOTASize,nread);
	  DumpMessage(g_traceBuf);
	  wm_sprintf(g_traceBuf,"%x %x %x %x\r\n",rcv_buffer[3],rcv_buffer[4],rcv_buffer[5],rcv_buffer[6]);
	  DumpMessage(g_traceBuf);
	  recordFileSizeOTA(DOTASize);
	  StartDOTA();
	  adl_tmrSubscribe( FALSE, 10, ADL_TMR_TYPE_100MS,
			    (adl_tmrHandler_t)tx_timer);
	  rcv_offset = 0;
	}
	else {
	  DumpMessage("No DOTA\r\n");
	  FoundDOTA = 0;
	}
      }

      if (!FirstDOTA) {
      
	// Load DOTA Data.
	if (FoundDOTA && CheckedDOTA) {
	  ADWriteData(nread,(u8 *)rcv_buffer);

	  DOTABytes += nread;
	  rcv_offset = 0;

	  TRACE((1,"DOTABytes = %d\r\n DOTASize = %d\r\n",DOTABytes,DOTASize));
	  DumpMessage(g_traceBuf);

	  // Done.
	  if (DOTABytes == DOTASize) {
	    DOTABytes = 0;
	    FoundDOTA = 0;
	    CheckedDOTA = 0;
	    CompleteADUpdate();
	    g_DOTAUpdateMode = 0;
	  }
	}
      }

      // Load configure Data.
      if (!FoundDOTA && CheckedDOTA) {

	wm_sprintf(g_traceBuf,"rcv offset = %d\r\n",rcv_offset);
	DumpMessage(g_traceBuf);

	// this is a complete configure packet
	if (rcv_offset == CONFIG_PKT_SIZE_TCP) {
	  rcv_offset = 0;
	  memcpy(g_rx,rcv_buffer,CONFIG_PKT_SIZE_TCP);
	  eval_packet();
	}
	else if (rcv_offset > CONFIG_PKT_SIZE_TCP) {
	  // probably got an invalid packet.
	  rcv_offset = 0;
	}
      }

      FirstDOTA = 0;
    }
    else {
      s32 nread;		
      s32 nreadable;
      u8 *rbuffer;
      ascii peer[16];      

      nreadable = ev->content.read.readable;
			
      rbuffer = (u8 *) adl_memGet( (u16) (nreadable+1));
      
      nread = wip_readOpts( socket, rbuffer, nreadable, //sizeof(buffer) - 1,
								  WIP_COPT_PEER_STRADDR, peer, sizeof( peer),
								  WIP_COPT_END);
      
      if (nread==0) {
	DumpMessage("FATAL ERROR: nread == 0!");
      }

      rbuffer[nreadable] = '\0';

      memcpy(g_rx,rbuffer,CONFIG_PKT_SIZE_TCP);
      eval_packet();
    }

    break;
  }
  case WIP_CEV_WRITE: {
    DumpMessage("WIP_CEV_WRITE\r\n");
    
    break;
  }
  case WIP_CEV_ERROR: {
    DumpMessage("WIP_CEV_ERROR\r\n");
    TRACE((1,"wip error closing socket"));
	NetworkError = TRUE;
    if (connection_type==TCP_CONNECTION_TYPE) wip_close( ev->channel); 
    
    if (TCPTryCount < MAX_TCP_TRIES) {
      appli_entry_point();   
      TCPTryCount++;
      TCPStatus = TCP_TRYING;
    }
    else {
      TCPStatus = TCP_CANT_CONNECT;
      wm_sprintf(g_traceBuf,"TCP Connect Error: Tried to connect %d times, can't connect\r\n",TCPTryCount);
      DumpMessage(g_traceBuf);
    }

    break;
  }
  case WIP_CEV_PEER_CLOSE: {
    DumpMessage( "Connection closed by peer\n");
	//NetworkError = TRUE;
    if (connection_type==TCP_CONNECTION_TYPE) wip_close( ev->channel);

    if (TCPTryCount < MAX_TCP_TRIES) {
      appli_entry_point();   
      TCPTryCount++;
      TCPStatus = TCP_TRYING;
    }
    else {
      TCPStatus = TCP_CANT_CONNECT;
    }

    break;
  }
  default : {
    DumpMessage( "Unknown WIP event\n");
	//NetworkError = TRUE;
    if (connection_type==TCP_CONNECTION_TYPE) wip_close( ev->channel);

    if (TCPTryCount < MAX_TCP_TRIES) {
      appli_entry_point();   
      TCPTryCount++;
      TCPStatus = TCP_TRYING;
    }
    else {
      TCPStatus = TCP_CANT_CONNECT;
    }
  }
  }
}


/** @brief Monitor the network registration
*
* @par description: Monitor the network registration; 
* the only way to do that is through an AT command, 
* so we send that "AT+CREG?" command. Actual reaction 
* will be performed by the callback poll_creg_callback().
*
* @param Id
* @param pfContext 
* @return void
*/
static void poll_creg( u8 Id, void *pfContext ) {
  (void) Id;
  (void) pfContext;
  adl_atCmdCreate( "AT+CREG?", FALSE, poll_creg_callback, ADL_STR_CREG, NULL);
}



/** @brief A call to "AT+CREG?" has been done
*
* @par description: A call to "AT+CREG?" has been done, 
* to check the registration status, and the answer comes 
* back to this handler. Either the registration is completed, 
* and we can actually open and start the bearer, or it 
* isn't, and we shall poll at "AT+CREG?" command again 
* through a timer.
*
* @param Rsp
* @return FALSE
*/
static bool poll_creg_callback(adl_atResponse_t *Rsp) {
    ascii *rsp;
    ascii regStateString[3];
    s32 regStateInt;
    
    TRACE (( 1, "(poll_creg_callback) Enter." ));

    rsp = (ascii *)adl_memGet(Rsp->StrLength);
    wm_strRemoveCRLF(rsp, Rsp->StrData, Rsp->StrLength);
    
    wm_strGetParameterString(regStateString, Rsp->StrData, 2);
    regStateInt = wm_atoi(regStateString);
    
    if ( 1 == regStateInt || 5 ==regStateInt) {           
      DumpMessage("(poll_creg_callback) Registered on GPRS network\r\n");
      ConnectGPRS();
    } else { 
      /* Not ready yet, we'll check again later. Set a one-off timer. */
      adl_tmrSubscribe( FALSE, CREG_POLLING_PERIOD, ADL_TMR_TYPE_100MS,
                        (adl_tmrHandler_t)poll_creg);
    }                
    return FALSE;
}

/** @brief start the GPRS communication
 *
 * @return void
 */
void StartGPRS(void) {

  s32 sReturn = -1;

  if (g_config.SMSorGPRS == SMS_ROAMING || g_config.SMSorGPRS == SMS_NO_ROAMING) return;

  sReturn = adl_gprsSubscribe(evhGPRSHandler);
  
  if( sReturn != OK )
    {
      DisplayErrorCode("adl_gprsSubscribe",__FILE__,__LINE__,sReturn);
    }
  else
    {
      DumpMessage( "Successfully subscribe to GPRS events\r\n");
    }

  if (wip_netInit() == 0)  
  {
    DumpMessage("wip_netInit Success.\r\n");
  }
  else 
  {
    DumpMessage("wip_netInit Fail.\r\n");
  }

  TCPTryCount = 0;

  if (!HaveBearer) InitBearer();
  else if (TCPStatus != TCP_CONNECT)   appli_entry_point();
}



/** @brief Connect GPRS 
 *
 * @return void
 */
static void ConnectGPRS(void) {
  int ret_val;
  if ((ret_val = myConnectToGPRS()) == 0) {
    DumpMessage("Connected to GPRS!\r\n");
	DumpMessageUSB("Connected to GPRS!\r\n",1);
  } else {
    wm_sprintf(g_traceBuf,"Failed to connect to GPRS: %d\r\n",ret_val);
    DumpMessage(g_traceBuf);
  }

  adl_tmrSubscribe( FALSE, 10, ADL_TMR_TYPE_100MS,
		    launch_app_timer);

}


/** @brief application timer
 *
 * @param Id
 * @param Context
 *
 * @return void
 */
static void  launch_app_timer(u8 Id, void *Context) {
  (void) Id;
  (void) Context;

  if (GPRSInit) {
    appli_entry_point();
  }
  else {
    adl_tmrSubscribe( FALSE, 10, ADL_TMR_TYPE_100MS,
		      launch_app_timer);
  }
}


/** @brief TX timer
 *
 * @param Id
 * @param Context
 *
 * @return void
 */
static void tx_timer(u8 Id, void *Context) {
  (void) Id;
  (void) Context;

  int nwrite;
  if (!g_DOTAOK) {
    adl_tmrSubscribe( FALSE, 10, ADL_TMR_TYPE_100MS,
		      (adl_tmrHandler_t)tx_timer);
  }
  else {
    DumpMessage("Send DOTA OK\r\n");
    nwrite = wip_write( socket, "OK",
			  2);

    if( nwrite < 0) { DisplayErrorCode("wip_write",__FILE__,__LINE__,nwrite); return; }
    g_DOTAOK = 0;
  }

}


/** @brief get GPRS IP address
 *
 * @return appIpAddr
 */
wip_in_addr_t GPRS_get_IPAddr(void)
{
  s8 RetCode;
  wip_in_addr_t appIpAddr; /*IP address in U32 format*/

  RetCode = wip_bearerGetOpts( myBearer
			       ,WIP_BOPT_IP_ADDR
			       ,&appIpAddr
			       ,WIP_BOPT_END);

  if (RetCode == 0) return appIpAddr;
  else return 0;
}

/** @brief GPRS event handler
 *
 * @param Event
 * @param CID
 * @return ADL_GPRS_FORWARD
 */
s8 evhGPRSHandler( u16 Event, u8 CID )
{
  (void) CID;
  s8 RetCode = -1;
    wip_in_addr_t appIpAddr; /*IP address in U32 format*/
  ascii IpAddr[15]; /*IP address in ascii*/
  // Trace event
  TRACE((1, "Received GPRS event\r\n"));

  switch( Event )
    {
    case ADL_GPRS_EVENT_SETUP_OK:
      TRACE((1,"ADL_GPRS_EVENT_SETUP_OK\r\n"));
      break;
    case ADL_GPRS_EVENT_ME_ATTACH:
      TRACE((1,"ADL_GPRS_EVENT_ME_ATTACH\r\n"));
      break;
    case ADL_GPRS_EVENT_ACTIVATE_OK:
      TRACE((1,"ADL_GPRS_EVENT_ACTIVATE_OK start Bearer\r\n"));

      RetCode = wip_bearerGetOpts( myBearer
				   ,WIP_BOPT_IP_ADDR
				   ,&appIpAddr
				   ,WIP_BOPT_END);
      if(RetCode == 0)
	{
	  /* Convert the address into the ascii format for printing*/
	  wip_inet_ntoa(appIpAddr , IpAddr, 15);
	  TRACE ((1,  "GPRSHandle: IP address ==>\r\n"));
#if 0
	  TRACE ((  IpAddr));
	  D_TRACE("%s\n",IpAddr);
#endif
	  wm_sprintf(g_traceBuf,"GPRSHandle: IP address ==> %x,%s\r\n",(unsigned int)appIpAddr,IpAddr);
	  DumpMessage(g_traceBuf);
	}
      else
	{
	  DisplayErrorCode("wip_bearerGetOpts"
			   ,__FILE__
			   ,__LINE__
			   ,RetCode);
	}
      break;

    case ADL_GPRS_EVENT_RING_GPRS:
      TRACE((1,"ADL_GPRS_EVENT_RING_GPRS\r\n"));
      break;
    case ADL_GPRS_EVENT_NW_CONTEXT_DEACT:
      TRACE((1,"ADL_GPRS_EVENT_NW_CONTEXT_DEACT\r\n"));
      break;
    case ADL_GPRS_EVENT_ME_CONTEXT_DEACT:
      TRACE((1,"ADL_GPRS_EVENT_ME_CONTEXT_DEACT\r\n"));
      break;
    case ADL_GPRS_EVENT_NW_DETACH:
      TRACE((1,"ADL_GPRS_EVENT_NW_DETACH\r\n"));
      break;
    case ADL_GPRS_EVENT_ME_DETACH:
      TRACE((1,"ADL_GPRS_EVENT_ME_DETACH\r\n"));
      break;
    case ADL_GPRS_EVENT_NW_CLASS_B:
      TRACE((1,"ADL_GPRS_EVENT_NW_CLASS_B\r\n"));
      break;
    case ADL_GPRS_EVENT_NW_CLASS_CG:
      TRACE((1,"ADL_GPRS_EVENT_NW_CLASS_CG\r\n"));
      break;
    case ADL_GPRS_EVENT_NW_CLASS_CC:
      TRACE((1,"ADL_GPRS_EVENT_NW_CLASS_CC\r\n"));
      break;
    case ADL_GPRS_EVENT_ME_CLASS_B:
      TRACE((1,"ADL_GPRS_EVENT_ME_CLASS_B\r\n"));
      break;
    case ADL_GPRS_EVENT_ME_CLASS_CG:
      TRACE((1,"ADL_GPRS_EVENT_ME_CLASS_CG\r\n"));
      break;
    case ADL_GPRS_EVENT_ME_CLASS_CC:
      TRACE((1,"ADL_GPRS_EVENT_ME_CLASS_CC\r\n"));
      break;
    case ADL_GPRS_EVENT_NO_CARRIER:
      TRACE((1,"ADL_GPRS_EVENT_NO_CARRIER\r\n"));
      break;
    case ADL_GPRS_EVENT_DEACTIVATE_OK:
      TRACE((1,"ADL_GPRS_EVENT_DEACTIVATE_OK\r\n"));
      break;
    case ADL_GPRS_EVENT_DEACTIVATE_OK_FROM_EXT:
      TRACE((1,"ADL_GPRS_EVENT_DEACTIVATE_OK_FROM_EXT\r\n"));
      break;
    case ADL_GPRS_EVENT_ANSWER_OK:
      TRACE((1,"ADL_GPRS_EVENT_ANSWER_OK\r\n"));
      break;
    case ADL_GPRS_EVENT_ANSWER_OK_FROM_EXT:
      TRACE((1,"ADL_GPRS_EVENT_ANSWER_OK_FROM_EXT\r\n"));
      break;
    case ADL_GPRS_EVENT_GPRS_DIAL_OK_FROM_EXT:
      TRACE((1,"ADL_GPRS_EVENT_GPRS_DIAL_OK_FROM_EXT\r\n"));
      break;
    case ADL_GPRS_EVENT_ACTIVATE_OK_FROM_EXT:
      TRACE((1,"ADL_GPRS_EVENT_ACTIVATE_OK_FROM_EXT\r\n"));
      break;
    case ADL_GPRS_EVENT_HANGUP_OK_FROM_EXT:
      TRACE((1,"ADL_GPRS_EVENT_HANGUP_OK_FROM_EXT\r\n"));
      break;
    case ADL_GPRS_EVENT_DEACTIVATE_KO:
      TRACE((1,"ADL_GPRS_EVENT_DEACTIVATE_KO\r\n"));
      break;
    case ADL_GPRS_EVENT_DEACTIVATE_KO_FROM_EXT:
      TRACE((1,"ADL_GPRS_EVENT_DEACTIVATE_KO_FROM_EXT\r\n"));
      break;
    case ADL_GPRS_EVENT_ACTIVATE_KO_FROM_EXT:
      TRACE((1,"ADL_GPRS_EVENT_ACTIVATE_KO_FROM_EXT\r\n"));
      break;
    case ADL_GPRS_EVENT_ACTIVATE_KO:
      TRACE((1,"ADL_GPRS_EVENT_ACTIVATE_KO\r\n"));
      break;
    case ADL_GPRS_EVENT_ANSWER_OK_AUTO:
      TRACE((1,"ADL_GPRS_EVENT_ANSWER_OK_AUTO\r\n"));
      break;
    case ADL_GPRS_EVENT_SETUP_KO:
      TRACE((1,"ADL_GPRS_EVENT_SETUP_KO\r\n"));
      break;
    case ADL_GPRS_EVENT_ME_UNREG:
      TRACE((1,"ADL_GPRS_EVENT_ME_UNREG\r\n"));
      break;
    case ADL_GPRS_EVENT_ME_UNREG_SEARCHING:
      TRACE((1,"ADL_GPRS_EVENT_ME_UNREG_SEARCHING\r\n"));
      break;

    default:
      TRACE((1,"undefined event happened %d\r\n"));
    }
  return ADL_GPRS_FORWARD;
}

/** @brief Get the TCP Status
*
* @param status
* @return void
*/
void GetTCPStatus(TCP_STATUS *status) {
  memcpy(status,&TCPStatus,sizeof(TCP_STATUS));
}

/** @brief Clear the TCP Status
*
* @return void
*/
void ClearTCPStatus(void)
{
  TCPStatus = TCP_TRYING;
}

/** @brief TCP Transmit 
*
* @par
* Peform a GPRS transmission - either TCP or UDP.
* The size varies based on the type of packet being transmitted - 'C','L','S', or 'W'.
* Connection type determines if we will transmit TCP or UDP.
* @param size
* @return nwrite
*
*/
int TCPTransmit(int size) {
  int nwrite;

  // If there is not a valid TCP connection, return right away.
  if (TCPStatus != TCP_CONNECT) return -1;

  // Try to send the entire packet.
  if (connection_type == TCP_CONNECTION_TYPE){
    nwrite = wip_write(socket, g_sms_tx, size);
  }
  else {
    nwrite = wip_writeOpts(socket,g_sms_tx,size,WIP_COPT_PEER_STRADDR,g_config.ServerIPAddr,
			   WIP_COPT_PEER_PORT,g_config.ServerPortNum,WIP_COPT_END);
  }

  if(nwrite != size) { 
    DisplayErrorCode("wip_write",__FILE__,__LINE__,nwrite); 
    return -1; 
  }

  return 0;
}

/*@}*/

void ConnectFTP(u8 timerid, void *context)
{
	(void) timerid;
	(void) context;
	
	//socket = wip_FTPCreateOpts("69.125.206.75", FTPevh, NULL, WIP_COPT_USER, "fox", WIP_COPT_PASSWORD, "fox", WIP_COPT_TYPE, 'A', WIP_COPT_PASSIVE, 1,WIP_COPT_END);
	socket = wip_FTPCreateOpts(FTPAddress, FTPevh, NULL, WIP_COPT_USER, "fox", WIP_COPT_PASSWORD, "fox", WIP_COPT_TYPE, 'A', WIP_COPT_PASSIVE, 1,WIP_COPT_END);
}

static void FTPevh( wip_event_t *ev, void *ctx)
{

  (void) ctx;
  
  static int DOTASize = 0;
  static int DOTABytes = 0;
  extern int gFileSize;

  wm_sprintf(g_traceBuf, "Inside FTPevh ev->kind : %d\r\n", ev->kind);
  DumpMessage(g_traceBuf);
  DumpMessageUSB(g_traceBuf, 1);

  
  switch( ev->kind) {

  case WIP_CEV_OPEN: {
    DumpMessage ("\r\n [SAMPLE] Connection established successfully\r\n");
	DumpMessage ("[SAMPLE] FTP Mode\r\n");	
	DumpMessageUSB ("[SAMPLE] FTP Mode\r\n", 1);	
	
	wip_getFile(ev->channel,"/fox/XACT.wpb.dwl",FTPevh,NULL);
	
	break;
  }
  case WIP_CEV_READ: {
	    int nread;

	    DumpMessage ("[SAMPLE] FTP data arrived\r\n");
	    DumpMessageUSB ("[SAMPLE] FTP data arrived\r\n", 1);

		static u8 *rbuffer = NULL;
		static int TotBytes=0;
		if(rbuffer == NULL)
			//rbuffer = (u8*)adl_memGet(gFileSize);
			rbuffer = (u8*)adl_memGet( RCV_BUFFER_SIZE );  // 10K is big enough to handle data

		if ( rbuffer == NULL )
			DumpMessageUSB("AEW debug - rbuffer == NULL\r\n", 1);
		else
		{
			wm_sprintf(g_traceBuf, "AEW debug - rbuffer allocated [%d]\r\n", (int)gFileSize);
			DumpMessageUSB(g_traceBuf, 1);
		}

		DumpMessage("FTP data received\r\n");
		DumpMessageUSB("FTP data received\r\n", 1);
			
		nread = wip_read( ev->channel, rbuffer,	gFileSize);
		TotBytes += nread;

		wm_sprintf(g_traceBuf, "nread = %d, TotBytes = %d\r\n", (int)nread, TotBytes);
		DumpMessage(g_traceBuf);
		DumpMessageUSB(g_traceBuf, 1);

		//for(i=0;i<nread;i++)
		//{
			//wm_sprintf(g_traceBuf, "%c ", rbuffer[i]);
			//DumpMessage(g_traceBuf);			
		//}
	
		DumpMessage("FTP Found DOTA\r\n");
		DumpMessageUSB("FTP Found DOTA\r\n", 1);
		g_DOTAUpdateMode = 1;
		DOTASize = gFileSize;
		wm_sprintf(g_traceBuf,"DOTASize=%d nread=%d\r\n",DOTASize,nread);
		DumpMessage(g_traceBuf);
		DumpMessageUSB(g_traceBuf, 1);

		// Load DOTA Data.
		DumpMessage("Start ADWriteData\r\n");
		DumpMessageUSB("FTP Found DOTA\r\n", 1);
		ADWriteData(nread,(u8 *)rbuffer);
		wm_sprintf(g_traceBuf,"DOTASize=%d nread=%d\r\n",DOTASize,nread);
		DumpMessage(g_traceBuf);
		DumpMessageUSB(g_traceBuf, 1);
		DOTABytes += nread;
		TRACE((1,"DOTABytes = %d\r\n DOTASize = %d\r\n",DOTABytes,DOTASize));
		// Done.
		if (DOTABytes == DOTASize) {
			DumpMessage("Upgrade Completed\r\n");
			DumpMessageUSB("Upgrade Completed\r\n", 1);
			DOTABytes = 0;
			CompleteADUpdate();
			adl_memRelease(rbuffer);
			rbuffer = NULL;
			g_DOTAUpdateMode = 0;
		}
		adl_ctxSleep(10);
		break;  
    }
  case WIP_CEV_WRITE: {
    DumpMessage("WIP_CEV_WRITE\r\n");
    
    break;
  }
  case WIP_CEV_ERROR: {
    DumpMessage("WIP_CEV_ERROR\r\n");
    TRACE((1,"wip error closing socket"));
	adl_ctxSleep(50);

	//NetworkError = TRUE;
    wip_close( ev->channel); 
    
    if (FTPTryCount < MAX_TCP_TRIES) {
      StartFTPTimer();   
      FTPTryCount++;
    }
    else {
      wm_sprintf(g_traceBuf,"FTP Connect Error: Tried to connect %d times, can't connect\r\n",FTPTryCount);
      DumpMessage(g_traceBuf);
    }

    break;
  }
  case WIP_CEV_PEER_CLOSE: {
    DumpMessage( "Connection closed by peer\r\n");
	adl_ctxSleep(50);
	//NetworkError = TRUE;
    wip_close( ev->channel);
    break;
  }
  case WIP_CEV_DONE :
  	DumpMessage("FTP task done\r\n");
	break;

  default : {
    DumpMessage( "Unknown WIP event\r\n");
	adl_ctxSleep(50);
	//NetworkError = TRUE;
    wip_close( ev->channel);

    if (FTPTryCount < MAX_TCP_TRIES) {
      StartFTPTimer();   
      FTPTryCount++;
    }
  }
  }
}

