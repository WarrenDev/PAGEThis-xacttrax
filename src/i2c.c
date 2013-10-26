/** @addtogroup I2C
 *@{*/
/*H****************************************************************************/
/** @file i2c.c
 *----------------------------------------------------------------------------
 *  @brief This file contains the generic I2C API calls
 *
 */
#include "adl_global.h"
//#include "appcommon.h"
#include "Traces.h"
#include "i2c_accelerometer.h"
#include "XactUtilities.h"

volatile s32	gI2CSignalFlag = 0; /* @@@TODO need to change to mutex calls */

/* GPS Constants */

static u8 gI2CDevice1 = 0x42; // uBlox
static u8 gI2CDevice2 = 0x4a; // LM73 temp sensor
static u8 gI2CDevice3 = 0x1D; // accellerometer

static s32 gI2CSemHndl = -1;

// Asynchronous reading context
typedef struct
{
    u32 ByteLength; // Byte length for the reading
    u32 Port;       // port where to send the reading result
} AsyncReadCtxt_t;

// Asynchronous Reading Context
AsyncReadCtxt_t * AsyncReadCtxt;

u8 portSetting[20] =
{
	0x0,		/* Port ID */
	0x0,		/* Reserved */
	0x0,		/* Reserved */
	0x0,		/* Reserved */
	0x84,		/* Mode, address set to 0x42 note: u32 endian reverse*/
	0x0,
	0x0,
	0x0,
	0x0,		/* Reserved */
	0x0,
	0x0,
	0x0,
	0x01,		/* inProtoMask, currently set to UBX only using u8 endian reverse*/
	0x0,
	0x01,		/* outProtoMask, currently set to UBX only */
	0x0,
	0x0,
	0x0,
	0x0,
	0x0
};

/*I2C Stuff */

int busMutex=1;
s32 i2cHandle=-1;	/* I2C Handle */
s32 pendHandle = -1;
static adl_busI2CSettings_t i2cSettings =
{
	(0x42),						/* Chip address is 0x42 (default value) */
	ADL_BUS_I2C_CLK_STD,		/* Chip uses the I2C standard clock speed */
	ADL_BUS_I2C_ADDR_7_BITS,	/* 7 bits address length */
	ADL_BUS_I2C_MASTER_MODE		/* Master Mode */
};

adl_busAccess_t AccessConfig =
{
	0,0							/* No opcode, No address */
};



s32 appBusLowIrqHandle = 0;
s32 appBusHighIrqHandle = 0;

/*Prototypes*/
int i2c_Subscribe(u8 DeviceId);
s32 i2c_write(u32 Address, u8 *Data, u32 DataSize);
s32 i2c_read(u32 Address,u8 *Data, u32 DataSize);
void StartI2CPolling(u16 timerID, void *Context);
void I2CMessageHandler(u32 MsgID, adl_ctxID_e Source, u32 Length, void *Data);

bool busHighIrqHandler(adl_irqID_e Source, adl_irqNotificationLevel_e NotificationLevel,
                                           adl_irqEventData_t *Data)
{ 
    (void) Source;
    (void) NotificationLevel;
    (void) Data;

	busMutex = 0;
	TRACE((I2C_TRACE_LEVEL,"busHirIrqHandler called"));

#ifdef REMOVE_SEM
	sReturn = adl_semProduce( gI2CSemHndl );
	if( sReturn < 0 )
	{
		TRACE((I2C_TRACE_LEVEL,"Error from semProduce"));
	}
#else
	gI2CSignalFlag = 0;
	
#endif
	return FALSE;
}

bool busLowIrqHandler(adl_irqID_e Source, adl_irqNotificationLevel_e NotificationLevel,
                                          adl_irqEventData_t *Data)
{
    (void) Source;
    (void) NotificationLevel;

    adl_busAsyncInfo_t *info = (adl_busAsyncInfo_t*)Data->SourceData;
    AsyncReadCtxt = (AsyncReadCtxt_t*)Data->Context;

    if (info->Result!=OK)
    {
       // TRACE (( drv_TraceLevel, "Wrong result : %d (expected %d)", info->Result, OK));
		TRACE ((I2C_TRACE_LEVEL, "Wrong result: %d (expected %d)\n",info->Result, OK));
    }
	TRACE ((I2C_TRACE_LEVEL, "low IRQ\n"));
	busMutex = 0;
//    return FALSE;
    return TRUE;
}

int i2c_Subscribe(u8 DeviceId)
{
	s32 sReturn=-1;
	u32 size=0;
	s32 temp;
	adl_busEvt_t busEvt;


#ifdef REMOVE_SEM
	if( gI2CSemHndl < 0 )
	{
		gI2CSemHndl = adl_semSubscribe(1);
		if( gI2CSemHndl < 0 )
		{
			//DisplayErrorCode("adl_semSubscribe",__FILE__,__LINE__,appBusLowIrqHandle );  
			return gI2CSemHndl;
		}
			sReturn = adl_semConsume(gI2CSemHndl);
	}
#endif


	if( appBusLowIrqHandle <= 0 )
	{
		TRACE((I2C_TRACE_LEVEL,"LOW LEVEL IRQ SUBSCRIBED"));
		appBusLowIrqHandle = adl_irqSubscribe ( busLowIrqHandler, ADL_IRQ_NOTIFY_LOW_LEVEL,
												 ADL_IRQ_PRIORITY_LOW_LEVEL, ADL_IRQ_OPTION_AUTO_READ  );
		if(appBusLowIrqHandle < 0)
		{
			//DisplayErrorCode("adl_irqSubscribe",__FILE__,__LINE__,appBusLowIrqHandle );  
			return appBusLowIrqHandle;
		}
	}

	if( appBusHighIrqHandle <= 0 )
	{
		TRACE((I2C_TRACE_LEVEL,"HIGH LEVEL IRQ SUBSCRIBED"));
	    appBusHighIrqHandle = adl_irqSubscribe ( busHighIrqHandler,ADL_IRQ_NOTIFY_HIGH_LEVEL,
   	                                          ADL_IRQ_PRIORITY_HIGH_LEVEL, ADL_IRQ_OPTION_AUTO_READ  );

		if(appBusHighIrqHandle < 0)
		{
			//DisplayErrorCode("adl_irqSubscribe",__FILE__,__LINE__,appBusHighIrqHandle );  
			return -1;
		}
	}

	i2cSettings.ChipAddress = DeviceId;
	
	TRACE((I2C_TRACE_LEVEL,"Device Id: %d %d",i2cSettings.ChipAddress,DeviceId));
	if( i2cHandle > -1 )
	{
		
		TRACE((I2C_TRACE_LEVEL,"need to call busUnsubscribe %d",i2cHandle));
		sReturn = adl_busUnsubscribe( i2cHandle );
		if( sReturn == 0 )
		{
			i2cHandle = -1; // we've unsubscribed from previous subscription
		}
	}
	temp = adl_busSubscribe( ADL_BUS_ID_I2C,1, &i2cSettings);
	
	TRACE((I2C_TRACE_LEVEL,"I2C Bus Subscribed Hndl:%d",temp));

	switch (temp)
	{
		case ADL_RET_ERR_PARAM:
			//printf( "Return of I2C busSubscribe is ADL_RET_ERR_PARAM\n" );
			TRACE((I2C_TRACE_LEVEL,"Return of busSubscribe is ADL_RET_ERR_PARAM"));
			break;
		case ADL_RET_ERR_ALREADY_SUBSCRIBED:
			//printf( "Return of I2C busSubscribe is ADL_RET_ERR_ALREADY_SUBSCRIBED\n" );
			TRACE((I2C_TRACE_LEVEL,"Return of busSubscribe is ADL_RET_ERR_ALREADY_SUBSCRIBED"));
			break;
		case ADL_RET_ERR_NO_MORE_HANDLES:
			//printf( "Return of I2C busSubscribe is ADL_RET_ERR_NO_MORE_HANDLES\n" );
			TRACE((I2C_TRACE_LEVEL,"Return of busSubscribe is ADL_RET_ERR_NO_MORE_HANDLES"));
			break;
		case ADL_RET_ERR_BAD_HDL:
			//printf( "Return of I2C busSubscribe is ADL_RET_ERR_BAD_HDL\n" );
			TRACE((I2C_TRACE_LEVEL,"Return of busSubscribe is ADL_RET_ERR_BAD_HDL"));
			break;
		default:
			/* SUCCESS SCENARIO */
			/*----- Set the size of address bits to 0 -----*/
			
			TRACE((I2C_TRACE_LEVEL,"adl_busSubscribe returned %d",temp));
			if(temp >= 0)
			{
				i2cHandle=temp;
				sReturn = adl_busIOCtl( i2cHandle, ADL_BUS_CMD_SET_ADD_SIZE, &size );
				if(sReturn < 0);
					//DisplayErrorCode("adl_busIOCtl",__FILE__,__LINE__,sReturn);
				else
					
				TRACE((I2C_TRACE_LEVEL,"adl_busIOCtl returned %d\n",sReturn));
				busEvt.LowLevelIrqHandle  = appBusLowIrqHandle;
				busEvt.HighLevelIrqHandle = appBusHighIrqHandle;

				sReturn = adl_busIOCtl ( i2cHandle,
									ADL_BUS_CMD_SET_ASYNC_MODE,
									&busEvt );
				if(sReturn < 0)
				{
					//DisplayErrorCode("adl_busIOCtl",__FILE__,__LINE__,sReturn);
					
					TRACE((I2C_TRACE_LEVEL,"i2cHandle: %d", i2cHandle));
					
					TRACE((I2C_TRACE_LEVEL,"busEvt.low: %d\n", busEvt.LowLevelIrqHandle));
					
					TRACE((I2C_TRACE_LEVEL,"busEvt.high: %d\n", busEvt.HighLevelIrqHandle));
					return -1;
				}

			}
			else
			{
				
				TRACE((I2C_TRACE_LEVEL,"adl_busSubscribe returned unknown error: %d",temp));
			}

		
		}
	
	TRACE((I2C_TRACE_LEVEL,"i2cHandle: %d\n", i2cHandle));
	return temp;
}

s32 SetAddressSize(u32 i_Address)
{
	s32 sReturn=-1;
	u32 size = 0;
	/*---- Set the address to read from ----*/
	if( (i2cSettings.ChipAddress == gI2CDevice1) || 
			(i2cSettings.ChipAddress == gI2CDevice2) || 
			(i2cSettings.ChipAddress == gI2CDevice3) )
	{
		size = 8;
		// lets shift the address up the MSB
		AccessConfig.Address=(i_Address << 24);
	}
	else
	{
		size = 16;
		// lets shift the address up the MSB
		AccessConfig.Address=(i_Address << 16);
	}	

	sReturn = adl_busIOCtl( i2cHandle, ADL_BUS_CMD_SET_ADD_SIZE,&size);
	if(sReturn < 0)
	{
		DisplayErrorCode("adl_busIOCtl",__FILE__,__LINE__,sReturn);
	}
	return sReturn;

}


#if defined(OLD_I2C_TIMER_HANDLER)
void timer_Handler_1(u8 id)
{
    s32 sReturn = -1;
    int i,j;
	u16 RetSize=0;
	u8 *readData= NULL;
	u8 readBuffer[24] = {0,0};

	for(i = 0; i< 1; i++)
	{
		readBuffer[0] = 0xA5;
		readBuffer[1] = 0x69;
		sReturn = i2c_read(0x000000fd,readBuffer,2);
		if(sReturn < 0)
		{
			TRACE((I2C_TRACE_LEVEL,"Failed to read to i2c\n"));
		}
		else
		{
//			while( busMutex )
//				adl_ctxSleep(1);
//			busMutex = 1;
			TRACE((I2C_TRACE_LEVEL,"Successfully read to i2c\n"));
			TRACE((I2C_TRACE_LEVEL,"Number of available bytes: %x %x\n"
					,readBuffer[0],readBuffer[1]));
			if( (readBuffer[0] != 0xFF) && (readBuffer[1] != 0xFF) )
				RetSize = ((u16)(readBuffer[0])<<8) | (u16)(readBuffer[1]);
			if( RetSize > 0 )
				break;
		}
	}
	if( RetSize < 0x1024)
	{
		readData = (u8 *)malloc(RetSize);
		sReturn = i2c_read(0x000000FF,readData,RetSize);
		if(sReturn < 0)
		{
		  TRACE((I2C_TRACE_LEVEL,"Failed to read to i2c\n"));
		}
		for(i=0,j=0;i<RetSize;i++)
		{
			TRACE((I2C_TRACE_LEVEL,"%x ",*(readData+i)));
			if(j<0x10)
				j++;
			else
			{
				TRACE((I2C_TRACE_LEVEL,"\t"));
				for(j=0;j<0x10;j++)
				{
					TRACE((I2C_TRACE_LEVEL,"%c",*(readData+i+j) ));
				}
				TRACE((I2C_TRACE_LEVEL,("\n")));
				j=0;
			}

		}
		free(readData);
		readData = NULL;
	}
    TRACE (( 1, "Bus read : %d", sReturn ));
}
#endif

s32 i2c_write(u32 Address, u8 *Data, u32 DataSize)
{
	unsigned int i;
	s32 sReturn = -1;
	u32 AddressSize = 0;

	do
	{
		for (i=0; i<DataSize; i++)
		{
			TRACE((I2C_TRACE_LEVEL,"%x,",*(Data+i)));
		}
		TRACE((I2C_TRACE_LEVEL,"\n"));
		sReturn = SetAddressSize(Address);
		if(sReturn < 0)
		{
			break;
		}
		if( (i2cSettings.ChipAddress == gI2CDevice1) || 
				(i2cSettings.ChipAddress == gI2CDevice2) || 
				(i2cSettings.ChipAddress == gI2CDevice3) )
		{
			AddressSize = 0;
			sReturn = adl_busIOCtl( i2cHandle, ADL_BUS_CMD_SET_ADD_SIZE,&AddressSize );
			TRACE((I2C_TRACE_LEVEL,"set address size to zero\n"));
		}

		sReturn = adl_busWrite(i2cHandle,&AccessConfig,DataSize,(void *)Data);
		if(sReturn < 0)
			DisplayErrorCode("adl_busWrite",__FILE__,__LINE__,sReturn);
	}while(0);
	return sReturn;
}


s32 i2c_read(u32 Address,u8 *Data, u32 DataSize)
{
	s32 sReturn = -1;

	do
	{
		sReturn = SetAddressSize(Address);
		if(sReturn < 0)
		{	
			break;
		}
		
		TRACE((I2C_TRACE_LEVEL,"calling adl_busRead %x %x\n",AccessConfig.Address,Address));
		TRACE ((I2C_TRACE_LEVEL, "call adl_busReadExt "));
		sReturn = adl_busRead(i2cHandle,&AccessConfig,DataSize,Data);
		printf("bus read return = %d\n",(int )sReturn);
#ifdef REMOVE_SEM
		sReturn = adl_semConsumeDelay( gI2CSemHndl,60);
#endif
		if( sReturn < 0 )
		  {
		    if( sReturn == -8 )
		      {
			// timeout happened, produce and continue
			TRACE((I2C_TRACE_LEVEL,"semConsumeDelay happened!!\n"));
			if( adl_semIsConsumed( gI2CSemHndl ) == TRUE )
			  {
			    sReturn = adl_semProduce( gI2CSemHndl );
			    if( sReturn < 0 )
			      DisplayErrorCode("adl_semProduce",__FILE__,__LINE__,sReturn);
			  }
		      }
		    else
		      DisplayErrorCode("adl_semConsumeDelay",__FILE__,__LINE__,sReturn);
		    printf ("I2C Return : %d\n",(int)sReturn);
		    return sReturn;
		  }
//		sReturn = adl_busReadExt(i2cHandle,&AccessConfig,DataSize,Data,AsyncReadCtxt);
//		while( busMutex )
//			adl_ctxSleep(1);
//		busMutex=1;

		TRACE ((I2C_TRACE_LEVEL, "complete adl_busReadExt "));
		if(sReturn < 0)
		{ //ADL_RET_ERR_UNKNOWN_HDL
			DisplayErrorCode("adl_busReadExt",__FILE__,__LINE__,sReturn);
			return sReturn;
		}

	}while(0);

	return sReturn;
}


void putuBoxCmd(u8 *Data,u8 Size)
{
	unsigned int i= 0;
	u8 *WriteBuf= NULL;
	u8 ck_A = 0;
	u8 ck_B = 0;
	s32 sReturn = -1;

	WriteBuf = (u8 *)malloc(Size+2);
	memcpy(WriteBuf,Data,Size);
	for(i=2; i<Size; i++)
	{
		ck_A = ck_A+Data[i];
		ck_B = ck_B + ck_A;
	}
	WriteBuf[i++] = ck_A;
	WriteBuf[i++] = ck_B;

	sReturn = i2c_write(0x000000ff,WriteBuf,(Size+2));
	if(sReturn < 0)
	{
		TRACE((I2C_TRACE_LEVEL,"Failed to write to i2c\n"));
	}
	else
	{
		TRACE((I2C_TRACE_LEVEL,"Successfully write to i2c\n"));
	}
}

u8 CmdToU8( char c)
{
	u8 RetVal = 0x0;
	switch(c)
	{
		case '0':
			RetVal = 0x0;
			break;
		case '1':
			RetVal = 0x1;
			break;
		case '2':
			RetVal = 0x2;
			break;
		case '3':
			RetVal = 0x3;
			break;
		case '4':
			RetVal = 0x4;
			break;
		case '5':
			RetVal = 0x5;
			break;
		case '6':
			RetVal = 0x6;
			break;
		case '7':
			RetVal = 0x7;
			break;
		case '8':
			RetVal = 0x8;
			break;
		case '9':
			RetVal = 0x9;
			break;
		case 'A':
		case 'a':
			RetVal = 0xa;
			break;
		case 'B':
		case 'b':
			RetVal = 0xb;
			break;
		case 'C':
		case 'c':
			RetVal = 0xc;
			break;
		case 'D':
		case 'd':
			RetVal = 0xd;
			break;
		case 'E':
		case 'e':
			RetVal = 0xe;
			break;
		case 'F':
		case 'f':
			RetVal = 0xf;
			break;
		default:
			RetVal = 0x0;
	}
	return RetVal;
}


/* The I2C device function called for accessing the Device */
int ioCallToI2CDevice( char *i_DeviceAddr, char *i_Cmd, char *i_Reg,char *i_Data, u8* rxData)
{
	s32 sReturn = 0;
	u8 DeviceAddr = 0x0;
	unsigned int iterator=0;
	int Length = 0;
	u8 ReadWriteFlag = 0;
	unsigned int ReadCnt = 0;
	u8 *Data;
	//u8 Data[2049];
	u8 ShortDeviceReg = 0;
	u16 LongDeviceReg = 0;
	u32 DeviceRegSize =0;
	u32 writeLength = 0;
	int retVal= 0;
	char Str[1024];
	int msgcnt = 0;
	do
	{
		Length = strlen(i_DeviceAddr );
		if( Length != 7 )
		{
			TRACE((I2C_TRACE_LEVEL,"Device Address must be specified as 7 bits in length"));
			
			TRACE((I2C_TRACE_LEVEL,"%d",i_DeviceAddr));
			retVal = -1;
			break;
		}

		if( *i_Cmd == 'r' || *i_Cmd == 'R' )
		{
			TRACE((I2C_TRACE_LEVEL,"we're doing a Read operation\n"));
			ReadWriteFlag = 1; // set flag to one for read
		}

		for(iterator=0; iterator<7; iterator++)
		{
			DeviceAddr = DeviceAddr<<1;
			// shift address up one bit
			if( *(i_DeviceAddr + iterator) == '1')
				DeviceAddr |= 0x1;
		}
		
		TRACE((I2C_TRACE_LEVEL,"Device Address is: %d",DeviceAddr));
		Length = strlen( i_Reg );
		
		
		TRACE((I2C_TRACE_LEVEL,"register address size: %d\n",Length));
		if( Length == 2 )
		{
			DeviceRegSize = 8;
			ShortDeviceReg = CmdToU8(*i_Reg) << 4;
			ShortDeviceReg |= CmdToU8(*(i_Reg+1));
			
			TRACE((I2C_TRACE_LEVEL,"short Device Reg: %d",ShortDeviceReg));
		}
		else
		{
			DeviceRegSize = 16;
			LongDeviceReg = (u16)(CmdToU8( *i_Reg ) << 12);
			LongDeviceReg |= (u16)(CmdToU8( *(i_Reg+1) ) << 8);
			LongDeviceReg |= (u16)(CmdToU8( *(i_Reg+2) ) << 4);
			LongDeviceReg |= (u16)(CmdToU8( *(i_Reg+3) ));
			
			TRACE((I2C_TRACE_LEVEL,"long Device Reg: %d",LongDeviceReg));
		}

		if( ReadWriteFlag )
		{
			sReturn = i2c_Subscribe( DeviceAddr );
			if( sReturn < 0 )
			{
				retVal=-1;
				break;
			}

			ReadCnt = atoi(i_Data);
			
			
			TRACE((I2C_TRACE_LEVEL,"read %d bytes from device",ReadCnt));
			//Data = (u8 *)malloc(ReadCnt);
			Data = (u8 *)adl_memGet(ReadCnt);

			if( DeviceRegSize == 8 )
				AccessConfig.Address=(ShortDeviceReg << 24);
			else
				AccessConfig.Address=(LongDeviceReg << 16);

			sReturn = adl_busIOCtl( i2cHandle, ADL_BUS_CMD_SET_ADD_SIZE,&DeviceRegSize );
			if(sReturn < 0)
			{
				//DisplayErrorCode("adl_busIOCtl",__FILE__,__LINE__,sReturn);
				retVal = -1;
				break;
			}

			gI2CSignalFlag = 1;
			sReturn = adl_busRead(i2cHandle,&AccessConfig,ReadCnt,Data);
			if( sReturn < 0 )
			{
				
				gI2CSignalFlag = 0;
				TRACE((I2C_TRACE_LEVEL,"ERROR in busRead: %d"));
				retVal = -1;
				break;
			}
		//FIXME: Changed this code for the test (Could present future watchdog issue if the wavecom fails for some reason)
		//	sReturn = adl_semConsumeDelay( gI2CSemHndl,60);
#ifdef REMOVE_SEM
			sReturn = adl_semConsume(gI2CSemHndl);
			if( sReturn < 0 )
			{
				if( sReturn == -8 )
				{
					// timeout happened, produce and continue
					
					TRACE((I2C_TRACE_LEVEL,"semConsumeDelay happened"));
					if( adl_semIsConsumed( gI2CSemHndl ) == TRUE )
					{
						sReturn = adl_semProduce( gI2CSemHndl );
						if( sReturn < 0 );
							//DisplayErrorCode("adl_semProduce",__FILE__,__LINE__,sReturn);
					}
				}
				else
					//DisplayErrorCode("adl_semConsumeDelay",__FILE__,__LINE__,sReturn);
				return -1;
			}
#else
			//gI2CSignalFlag = 1;
			while(gI2CSignalFlag )
				adl_ctxSleep(1);
#endif
			msgcnt = 0;
			for (iterator=0; iterator<ReadCnt; iterator++)
			{
				//printf("%x ",*(Data+x));
				if( (Data[iterator] > 0x1f) && (Data[iterator] < 0x7b) )
				{
					if(msgcnt < 1020 )
						Str[msgcnt++] = Data[iterator];
				}
				else
				{
					if(msgcnt > 5 )
					{
						Str[msgcnt++] = 0xA;
						Str[msgcnt++] = 0xd;
						Str[msgcnt] = 0x0;
						TRACE((I2C_TRACE_LEVEL,"%s",Str));
					}
					msgcnt = 0;

				}
				
			}
			if(msgcnt > 5 )
			{
				Str[msgcnt++] = 0xA;
				Str[msgcnt++] = 0xd;
				Str[msgcnt] = 0x0;
				TRACE((I2C_TRACE_LEVEL,"%s",Str));
				msgcnt = 0;
			}

			if(rxData != NULL)
				memcpy(rxData,Data,ReadCnt);

			//printf("\n");
		//	free(Data);
		//	Data = NULL;
			adl_memRelease(Data);
			Data = NULL;
		}
		else
		{
			sReturn = i2c_Subscribe( DeviceAddr );
			if( sReturn < 0 )
			{
				retVal = -1;
				break;
			}
			if( DeviceRegSize == 8 )
				AccessConfig.Address=(ShortDeviceReg << 24);
			else
				AccessConfig.Address=(LongDeviceReg << 16);

			sReturn = adl_busIOCtl( i2cHandle, ADL_BUS_CMD_SET_ADD_SIZE,&DeviceRegSize );
			if(sReturn < 0)
			{
				//DisplayErrorCode("adl_busIOCtl",__FILE__,__LINE__,sReturn);
				retVal = -1;
				break;
			}
			ReadCnt =strlen(i_Data);
			//Data = (u8 *)malloc(ReadCnt);
			Data = (u8 *)adl_memGet(ReadCnt);
			// AB: CRITICAL. I agree. Behavior of x may be undefined. Rewrote.
			//TVDP: CRITICAL Daring use of pointer step (x)
			if (Data == NULL)
			{
			  break;
			}

			writeLength = 0;
			iterator = 0;
			while ( iterator < ReadCnt )
			  {
			    Data[writeLength]    = CmdToU8( i_Data[iterator++] )<<4 ;
			    Data[writeLength++] |= CmdToU8( i_Data[iterator++]);
			  }

			
			TRACE((I2C_TRACE_LEVEL,"data to write"));
			for(iterator=0; iterator<writeLength; iterator++)
			{
				
				TRACE((I2C_TRACE_LEVEL,"%d",Data[iterator]));
			}

			sReturn = adl_busWrite(i2cHandle,&AccessConfig,writeLength,(void *)Data);
			if( sReturn < 0 )
			{
				
				TRACE((I2C_TRACE_LEVEL,"ERROR in adl_busWrite: %d\n",sReturn));
				retVal = -1;
				break;
			}
#ifdef REMOVE_SEM
			sReturn = adl_semConsume(gI2CSemHndl);
#endif
			//free(Data);
			//Data = NULL;
			adl_memRelease(Data);
			Data = NULL;
		}
		
	}while(0);
	return retVal;
}








/*@}*/
