/** @addtogroup Storage
 *@{*/

/*H***************************************************************************
 */

/** @file spiflash.c
 *
 * @brief Facilitate data transfer between SPI Flash and the file system
 *
 * @details Data is read and writen to the SPI Flash in sectors.
 *
 * @note    Other help for the reader, including references.
 *
 *
 *//*
 *
 * CHANGES :
 *
 * REF NO	DATE	WHO		DETAIL
 *		12JUN08	MG		First Version
 * NNNNNN	12AUG08	MG		Added Doxygen Tags
 *
 *H*/

/**************************************************************************
 *   INCLUDE FILES
 ***************************************************************************/
/*---- system and platform files -----------------------------------------*/
#include "adl_global.h" /* The global include file for all of the ADL APIs */
#include "adl_bus.h"    /* The include file required to ADL SPI bus communication */

/*---- program files -----------------------------------------------------*/
#include "atmel_flashDriver.h"  /* Include file for SPI Flash variables and Constants */
#include "Traces.h"     /* Include file for trace constants */
#include "support.h"    /* Include file for Bit Level MACROS */
#include "XactUtilities.h"

/**************************************************************************
 *   PUBLIC DECLARATIONS      Defined here, used elsewhere
 ***************************************************************************/

/**************************************************************************
 *   PRIVATE DECLARATIONS     Defined here, used only here
 ***************************************************************************/
/*---- context -----------------------------------------------------------*/
//SPI Flash OP Codes
#define CONT_ARRAY_READ (0x03)                  /**< SPI OPCODE: Continuous Array Read */
#define BLOCK_ERASE (0x50)                      /**< SPI OPCODE: Block Erase */
#define SECTOR_ERASE (0x7C)                     /**< SPI OPCODE: Sector Erase */
#define BUFFER1_WRITE (0x84)                    /**< SPI OPCODE: Buffer 1 Write */
#define BUFFER2_WRITE (0x87)                    /**< SPI OPCODE: Buffer 2 Write */
#define BUFFER1_TO_MEMORY (0x83)                /**< SPI OPCODE: Buffer 1 To Memory */
#define BUFFER2_TO_MEMORY (0x86)                /**< SPI OPCODE: Buffer 2 to Memory */
#define PAGE_TO_BUFFER1 (0x53)                  /**< SPI OPCODE: Page To Buffer 1 */
#define PAGE_TO_BUFFER2 (0x55)                  /**< SPI OPCODE: Page To Buffer 2 */
#define BUFFER1_TO_PAGE_COMPARE (0x60)          /**< SPI OPCODE: Compare Buffer 1 To Page */
#define BUFFER2_TO_PAGE_COMPARE (0x61)          /**< SPI OPCODE: Compate Buffer 2 To Page */
#define READ_MNFG_CODE (0x9F)                   /**< SPI OPCODE: Read Manufacture Code */
#define FULL_FLASH_ERASE (0xC7)                 /**< SPI OPCODE: FULL Chip Erase Code */
#define FULL_FLASH_ERASE_OPCODE (0xC794809A)    /**< SPI OPCODE: Full Chip Erase Sequence */
#define MEMORY_PAGE_READ (0xD2)                 /**< SPI OPCODE: Read Page */
#define BUFFER1_READ (0xD1)                     /**< SPI OPCODE: Read Buffer 1 */
#define BUFFER2_READ (0xD3)                     /**< SPI OPCODE: Read Buffer 2*/
#define STATUS_REGISTER_READ (0xD7)             /**< SPI OPCODE: Read Status Register */

//SPI Flash Characteristics
#define PAGECOUNT           (8192)          /**< Size of Flash in bytes */
#define PAGESIZE            (1056)          /**< Size of Flash Page in bytes on a page */
#define High                0               /**< High Sector = First on Page */
#define Low                 SectorSize      /**< Low Sector = Second on Page */

//SPI Init Flag Bit Definitions
#define SIF_SPI_INIT        0   /**< Indicates if the SPI Bus has been initialized */
#define SIF_FLASH_INIT      1   /**< Indicates if the Flash has been initialized */
#define SIF_BUFFER_INIT     2   /**< Indicates if Buffer requires initialization */

//SPI Flash Command Definitions
#define SPI_OPCODE_SIZE     8   /**< Standard SPI Opcode length in bits */
#define SPI_ADDRESS_SIZE    24  /**< Standard SPI Address length in bits */

// Retry Values
#define DELAY_READ_STATUS_CNT (0x60)    /**< Number of times to check if Flash is
	                                     * ready before operation fails. */

//#define ENTRY_TRACE(x) wm_sprintf(g_traceBuf, "%s ENTRY\r\n",x); DumpMessage(g_traceBuf);
//#define EXIT_TRACE(x)  wm_sprintf(g_traceBuf, "%s EXIT\r\n",x); DumpMessage(g_traceBuf);

#define ENTRY_TRACE(x)
#define EXIT_TRACE(x)

/*! \struct Sector_s
 *	\brief Sector structure
 *	\details Contains all the data needed to identify a sector on the flash. */
struct Sector_s
{
	u32 Page;           /*!< The Page in Flash on which the sector is located */
	u32 Location;       /*!< The Location within a Flash Page the the secor is located at */
};
typedef struct Sector_s type_Sector;    /*!< Creates a type name for Sector_s */

/*---- data declarations -------------------------------------------------*/

//SPI Flash Operation Parameter structure
const adl_busSPISettings_t spi1Settings =
{
	1,                          /**< No divider, use full clock speed */
	ADL_BUS_SPI_CLK_MODE_0,     /**< Mode 0 clock	*/
	ADL_BUS_SPI_ADDR_CS_GPIO,   /**< Use a GPIO to handle the Chip Select signal	*/
	ADL_BUS_SPI_CS_POL_LOW,     /**< Chip Select active in low state	*/
	ADL_BUS_SPI_MSB_FIRST,      /**< Data are sent MSB first	*/
//	ADL_BUS_SPI_ADDR_CS_GPIO,
	31,                         /**< Use GPIO 31 to handle Chip Select */
	ADL_BUS_SPI_LOAD_UNUSED,    /**< LOAD signal not used */
	ADL_BUS_SPI_DATA_UNIDIR,    /**< 3 Wires configuration */
	ADL_BUS_SPI_MASTER_MODE,    /**< Slave mode */
	ADL_BUS_SPI_BUSY_UNUSED     /**< BUSY signal not used */
};

Buffers ActiveBuffer;               /**< Identifies the buffer currently in use */
s32 spi1Handle;                     /**< Stores the handle to the subscribed spy bus */
u8 SPI_InitFlags = 0x00;            /**< Stored the initialization state of the SPI flash */
u32 BuffersPage[2] = { 0, 0 };          /**< Stores the number of page currently stored in Buffer 1 & 2 */
u32 SectorSize = 0;                 /**< Number if bytes per sector */
u32 SectorCount = 0;                /**< Number of sectors on flash */
u32 SectorPerPage = 0;              /**< Number of Sectors on each page of flash */
u32 ExtraPageDataStart = 0;         /**< Starting point of extra data location */
int SPI_Error = SUCCESS;            /**< SPI Driver Error Field */

/*---- function prototypes -----------------------------------------------*/
int spi_ConfigOpAddrSize(u32 OpSize, u32 AddrSize);
void GetPageFromSector(u32, type_Sector *Sector);
int spi_subSPI1(void);
int spi_WriteBuffer(Buffers, u32, char *, u32);
int spi_ReadBuffer(Buffers, u32, char *, u32);
int spi_BufferToMemory(Buffers, u32);
int spi_ReadPage(u32, u32, char *, u32);
int spi_ComparePageToBuffer(Buffers, u32);
int spi_ReadPageToBuffer(Buffers, u32);
s32 writeSPI1(u8 OpCode, s32 Address, char *Data, u32 DataSize);
s32 readSPI1(u8 OpCode, s32 Address, char *Data, u32 DataSize);
s32 FLASHReadyForCommand(void);
extern void ResetSPIFlash(void);

/*****************************************************************
 *   PUBLIC FUNCTION DEFINITIONS
 ******************************************************************/

/*F***************************************************************
 *
 *   NAME:    int sf_MediaInit( u32 SectorLength )
 */

/*! \brief Initalize the SPI FLASH module
 *
 * \par This function initializes the SPI Flash module and
 * and defines the sector size used. A sector size of
 * 512 currently be specified.
 *
 * \param SectorLength [in] The size, in bytes, of a sector.
 *
 * \return
 * <TABLE border=0>
 * <TR><TD><i>Return</i></TD><TD>Description</TD></TR>
 *  <TR><TD><i>SUCCESS</i></TD><TD>The SPI Flash Was Successfully Initialized</TD></TR>
 *  <TR><TD><i>SPI_SECTOR_OUT_OF_RANGE</i></TD><TD>The specified sector size was greater than the Flash page size (1024) </TD></TR>
 *  </TABLE>
 *
 *//*
 *
 * CHANGES :
 * REF NO  DATE     WHO      DETAIL
 *         12JUN08  MG	    Initial version
 * NNNNNN  12AUG08  MG		Added Doxygen Tags
 *F*/
int sf_MediaInit(u32 SectorLength)
{
	int sResult;

	//Only Initialize if SectorLength is valid
	if ((SectorLength < PAGESIZE) && (SectorLength > 0))
	{
		//Initialize SPI
		sResult = spi_subSPI1();

		if (sResult == SUCCESS)
		{   //SPI Bus Subscribed Successfully
			//Initialize Flash Descriptor Variables
			SectorSize = SectorLength;
			SectorPerPage = PAGESIZE / SectorSize;
			SectorCount = SectorPerPage * PAGECOUNT;
			ExtraPageDataStart = SectorLength * SectorPerPage;

			//Indicate SPI Flash Initialized Successfully
			SET_BIT(SPI_InitFlags, SIF_FLASH_INIT);

			TRACE((SPI_FLASH_STATUS_TRACE, "SPI Flash Initialized. SectorCount: %d\n", SectorCount));
		}
	}
	else
	{
		TRACE((SPI_FLASH_ERROR_TRACE, "Invalid Sector Size Specified. Size: %d", SectorLength));
		sResult = SPI_SECTOR_OUT_OF_RANGE;
	}
	SPI_Error = sResult;
	return sResult;
}


/*F***************************************************************
 *
 *   NAME:    int sf_readSector( u32 sectorNum, u8 *buf )
 */

/*! \brief   This function reads the specified sector from the SPI
 *			 Flash into buf.
 *
 *	\param	 sectorNum	u32		[in]	The sector number to read.
 *	\param	 buf		u8		[out]	Buffer pointer in which to store the sector data.
 *
 *
 *   \return
 *   <TABLE border=0>
 *       <TR>
 *           <TD><i>Return</i></TD>
 *           <TD>Description</TD>
 *       </TR>
 *       <TR>
 *           <TD><i>SUCCESS</i></TD>
 *           <TD>The read was successful</TD>
 *      </TR>
 *	   <TR>
 *           <TD><i>SPI_FLASH_NOT_INITIALIZED</i></TD>
 *           <TD>The sector could not be read. The SPI Flash was not
 *           initialized</TD>
 *      </TR>
 *   </TABLE>
 *       <TR>
 *           <TD><i>SPI_SECTOR_OUT_OF_RANGE</i></TD>
 *          <TD>The specified sector is not a valid sector number.</TD>
 *       </TR>
 *       <TR>
 *           <TD><i>SPI_READ_ERROR</i></TD>
 *           <TD>The sector could not be read. These was an error reading
 *           the data.</TD>
 *       </TR>
 *   </TABLE>
 *
 *//*
 *
 * CHANGES :
 * REF NO  DATE     WHO      DETAIL
 *         12JUN08  MG	    Initial version
 * NNNNNN  12AUG08  MG		Added Doxygen Tags
 *F*/
int sf_readSector(u32 sectorNum, u8 *buf)
{
	type_Sector sectorData;
	int sResult = SPI_UNKNOWN_ERROR;

	//Initialize Flash if not done already
	if (!(TEST_BIT(SPI_InitFlags, SIF_FLASH_INIT)))
	{
		TRACE((SPI_FLASH_DEBUG_TRACE, "Cannot Read Sector. SPI Flash has not been initialized\n"));

		sResult = SPI_FLASH_NOT_INITIALIZED;
	}
	//	else if( SectorCount==0)
	//	{
	//		TRACE((SPI_FLASH_DEBUG_TRACE,"Sector Count is Zero. Cannot Continue. Did you initialize the filesystem with a valid sector size?"));
	//	}
	else if (sectorNum > SectorCount)
	{   //Sector Number out of range
		TRACE((SPI_FLASH_DEBUG_TRACE, "Cannot Read Sector. Sector Number, %d,  out of range. Last Sector: %d\n", sectorNum, SectorCount));
		sResult = SPI_SECTOR_OUT_OF_RANGE;
	}
	else
	{
		//Get Sector Info
		GetPageFromSector(sectorNum, &sectorData);
		//Load required Page
		sResult = spi_ReadPage(sectorData.Page, sectorData.Location, (char *)buf, SectorSize);
		SPI_Error = sResult;
		if (sResult != SUCCESS)
		{
			//A Lower Level Error Occured - Report Read Error
			sResult = SPI_READ_ERROR;
		}
#ifdef DEBUG
		if (sResult == SUCCESS)
		{
			TRACE((SPI_FLASH_DEBUG_TRACE, "Sector %d Read From Page %d.", sectorNum, sectorData.Page));
		}
		else
		{
			TRACE((SPI_FLASH_DEBUG_TRACE, "ERROR*** Could not read sector &d from Page %d.", sectorNum, sectorData.Page));
		}
#endif
	}
	return sResult;
}


/*F***************************************************************
 *
 *   NAME:    int sf_writeSector( u32 sectorNum, u8 *Data )
 */

/*! \brief   This function writes the provided data to the specified sector \n
 *			 on the SPI Flash.
 *
 *	\param	 sectorNum	u32		[in]	The sector number to read.
 *	\param	 Data		u8		[in]	Pointer to the data that is to be written to the specified sector.
 *
 *
 *  \return
 *      <TABLE border=0><TR><TD><i>Return</i></TD><TD>Description</TD></TR>
 *      <TR><TD><i>SUCCESS</i></TD><TD>The write was  successful</TD></TR>
 *	   <TR><TD><i>SPI_FLASH_NOT_INITIALIZED</i></TD><TD>The sector could not be written. The SPI Flash was not initialized</TD></TR></TABLE>
 *	   <TR><TD><i>SPI_SECTOR_OUT_OF_RANGE</i></TD><TD>The specified sector is not a valid sector number.</TD></TR>
 *	   <TR><TD><i>SPI_READ_ERROR</i></TD><TD>The sector could not be written. These was an error writting the data.</TD></TR></TABLE>
 *
 *
 *   \note   Other assumptions, hints, for the reader
 *//*
 *
 * CHANGES :
 * REF NO  DATE     WHO      DETAIL
 *         12JUN08  MG	    Initial version
 * NNNNNN  12AUG08  MG		Added Doxygen Tags
 *F*/
int sf_writeSector(u32 sectorNum, u8 *Data)
{
	type_Sector sectorData;
	int sResult = SPI_UNKNOWN_ERROR;

#ifdef DEBUG
	if (sectorNum == 0)
	{
		TRACE((SPI_FLASH_DEBUG_TRACE, "Warning: Overwriting Sector 0"));
	}
#endif

	//Initialize Flash if not done already
	if (!(TEST_BIT(SPI_InitFlags, SIF_FLASH_INIT)))
	{
		TRACE((SPI_FLASH_ERROR_TRACE, "***ERROR: Cannot Write Sector. SPI Flash has not been initialized\n"));
		sResult = SPI_FLASH_NOT_INITIALIZED;
	}
	else if (sectorNum > SectorCount)
	{   //Sector Number out of range
		TRACE((SPI_FLASH_ERROR_TRACE, "***ERROR: Cannot Write Sector. Sector Number, %d,  out of range. Last Sector: %d\n", sectorNum, SectorCount));
		sResult = SPI_SECTOR_OUT_OF_RANGE;
	}
	else
	{
		//Get Sector Info
		GetPageFromSector(sectorNum, &sectorData);

		if ((sectorData.Page == BuffersPage[Buffer1]) && TEST_BIT(SPI_InitFlags, SIF_BUFFER_INIT + Buffer1))
		{   //Set Active Buffer
			ActiveBuffer = Buffer1;
			TRACE((SPI_FLASH_DEBUG_TRACE, "Buffer1 Reused"));
		}
		else if ((sectorData.Page == BuffersPage[Buffer2]) && TEST_BIT(SPI_InitFlags, SIF_BUFFER_INIT + Buffer2))
		{   //Set Active Buffer
			ActiveBuffer = Buffer2;
			TRACE((SPI_FLASH_DEBUG_TRACE, "Buffer2 Reused"));
		}
		else
		{   //No Preloaded Buffers
			//Rotate Buffers
			if (ActiveBuffer == Buffer2)
			{
				ActiveBuffer = Buffer1;
				TRACE((SPI_FLASH_DEBUG_TRACE, "Switching ActiveBuffer to Buffer 1"));
			}
			else
			{
				ActiveBuffer = Buffer2;
				TRACE((SPI_FLASH_DEBUG_TRACE, "Switching ActiveBuffer to Buffer 2"));
			}

			//Assign new page to buffer
			BuffersPage[ActiveBuffer] = sectorData.Page;

			//Load Buffer
			sResult = spi_ReadPageToBuffer(ActiveBuffer, sectorData.Page);
			//sResult = spi_ReadPageToBuffer(Buffer1, sectorData.Page);
			if (sResult != SUCCESS)
			{   //Error Reading Page to Buffer
				TRACE((SPI_FLASH_DEBUG_TRACE, "Error*** Reading Page %d to Buffer %d", sectorData.Page, Buffer1));
				return sResult;
			}

			//Indicate that the active buffer has been initialized
			SET_BIT(SPI_InitFlags, SIF_BUFFER_INIT + ActiveBuffer);
		}

		//Write data to active buffer with sector offset
		sResult = spi_WriteBuffer(ActiveBuffer, sectorData.Location, (char *)Data, SectorSize);
		if (sResult != SUCCESS)
		{   //Error Writing Data to Buffer
			TRACE((SPI_FLASH_DEBUG_TRACE, "Error*** Writing Data to Buffer %d", Buffer1));
			return sResult;
		}

		//Write Buffer to Page
		sResult = spi_BufferToMemory(ActiveBuffer, sectorData.Page);
		if (sResult != SUCCESS)
		{   //Error Writing Buffer To Memory
			TRACE((SPI_FLASH_DEBUG_TRACE, "Error*** Writing Buffer %d to Page %d", Buffer1, sectorData.Page));
			return sResult;
		}

		//Verify data written to memory is the same as that in the buffer
		sResult = spi_ComparePageToBuffer(ActiveBuffer, sectorData.Page);
#ifdef DEBUG
		if (sResult != SUCCESS)
		{   //Data Does not match
			TRACE((SPI_FLASH_DEBUG_TRACE, "Error*** Writen Data & memory do not match %d", Buffer1));
		}
		else
		{
			TRACE((SPI_FLASH_DEBUG_TRACE, "Page Data and Buffer Writen & Compared successfully"));
		}
#endif
	}
	return sResult;
}


/*F***************************************************************
 *
 *   NAME:    void GetPageFromSector( u32 sectorNum, type_Sector *FoundSector )
 */

/*! \brief   This function finds the page and location within a flash page of the provide sector number
 *
 *	\param	 sectorNum		u32			[in]	The sector number to process.
 *	\param	 FoundSector	type_Sector	[out]	The address of a the Sector structure to place the page and location information in.
 *
 *  \return void
 *//*
 *
 * CHANGES :
 * REF NO  DATE     WHO      DETAIL
 *         12JUN08  MG	    Initial version
 * NNNNNN  12AUG08  MG		Added Doxygen Tags
 *F*/
void GetPageFromSector(u32 sectorNum, type_Sector *FoundSector)
{
	//Determine if it is a High or Low Sector
	if (sectorNum & 0x1)
	{
		FoundSector->Location = Low;
	}
	else
	{
		FoundSector->Location = High;
	}

	//Divide sectors by Sectors per Page
	FoundSector->Page = (sectorNum / SectorPerPage);
}


/*F***************************************************************
 *
 *   NAME:    s32 writeSPI1( u8 OpCode,s32 Address,char *Data,u32 DataSize )
 */

/*! \brief   This function issues a write command to the SPI Flash over the SPI1 bus.
 *
 *	\param	 OpCode		u8		[in]	The opcode of the command to be sent to the SPI Flash.
 *	\param	 Address	s32		[in]	The Address to write on the SPI Flash.
 *	\param	 Data		char	[in]	The Data to write to the SPI Flash
 *	\param	 DataSize	u32		[in]	The size, in bytes, of Data.
 *
 *  \return
 *      <TABLE border=0><TR><TD><i>Return</i></TD><TD>Description</TD></TR>
 *      <TR><TD><i>SUCCESS</i></TD><TD>The write was issued successfully</TD></TR>
 *	   <TR><TD><i>Other</i></TD><TD>A error occured while attempting to write to the SPI Flash</TD></TR></TABLE>
 *
 *//*
 *
 * CHANGES :
 * REF NO  DATE     WHO      DETAIL
 *         12JUN08  MG	    Initial version
 * NNNNNN  12AUG08  MG		Added Doxygen Tags
 *F*/
s32 writeSPI1(u8 OpCode, s32 Address, char *Data, u32 DataSize)
{
	s32 sResult = SUCCESS;
	adl_busAccess_t AccessConfig =
	{
		0x0, 0x0 // Address, Op Code
	};

	TRACE((SPI_FLASH_DEBUG_TRACE, "Checking if Flash is Ready - writeSPI"));
	sResult = FLASHReadyForCommand();
	TRACE((SPI_FLASH_DEBUG_TRACE, "writeSPI - Flash Ready"));
	if (sResult != SUCCESS)
	{
		TRACE((SPI_FLASH_DEBUG_TRACE, "Error Waiting for Flash To Be Ready. Result: %d, line: %d", sResult, __LINE__));
		//D_TRACE( "Error Waiting for Flash To Be Ready." );

		//Return Error
		return sResult;
	}

	// Default Opcode & Address size to defaults
	sResult = spi_ConfigOpAddrSize(SPI_OPCODE_SIZE, SPI_ADDRESS_SIZE);
	if (sResult != SUCCESS)
	{
		//Return Error
		return sResult;
	}

	switch (OpCode)
	{
	case FULL_FLASH_ERASE:     // chip erase
		//Set Opcode & Address Sizes
		sResult = spi_ConfigOpAddrSize(SPI_OPCODE_SIZE + SPI_ADDRESS_SIZE, 0);

		AccessConfig.Opcode = FULL_FLASH_ERASE_OPCODE;
		AccessConfig.Address = 0x00000000;
		break;

	case BUFFER1_TO_MEMORY:
	case BUFFER2_TO_MEMORY:

		//Set Opcode
		AccessConfig.Opcode = (OpCode << 24);
		AccessConfig.Address = (u32)Address;

		break;

	case BUFFER1_WRITE:
	case BUFFER2_WRITE:
		//Set Opcode
		AccessConfig.Opcode = (OpCode << 24);

		//Set Address
		AccessConfig.Address = (u32)Address << 8;
		break;

	case BUFFER1_TO_PAGE_COMPARE:
	case BUFFER2_TO_PAGE_COMPARE:
		//Set Opcode
		AccessConfig.Opcode = (OpCode << 24);

		//Set Address
		AccessConfig.Address = (u32)Address << 8;

		break;

	case PAGE_TO_BUFFER1:
	case PAGE_TO_BUFFER2:                          // main memory to Buffer 1

		//Set Opcode & Address Sizes
		sResult = spi_ConfigOpAddrSize(SPI_OPCODE_SIZE + SPI_ADDRESS_SIZE, 0);

		// Set Opcode & Address
		AccessConfig.Opcode = (OpCode << 24) | (Address << 11);

		//Set Address to zero
		AccessConfig.Address = 0;

		break;

	default:

		// we did not get a valid OpCode to read FLASH with
		sResult = SPI_UNKNOWN_OPCODE;
		TRACE((SPI_FLASH_DEBUG_TRACE, "writeSPI1 Received Unknown Opcode: %d", OpCode));
		//D_TRACE( "writeSPI Received Unknown Opcode" );
	}

	sResult = adl_busWrite(spi1Handle, &AccessConfig, DataSize, Data);
	if (sResult < 0)
	{
		DisplayErrorCode("adl_busWrite", __FILE__, __LINE__, sResult);
		//D_TRACE("spi1Handle: %x, DataSize, %d\n", spi1Handle, DataSize);
	}
	return sResult;
}


/*F***************************************************************
 *
 *   NAME:    s32 readSPI1( u8 OpCode,s32 Address,char *Data,u32 DataSize )
 */

/*! \brief   This function issues a read command to the SPI Flash over the SPI1 bus.
 *
 *	\param	 OpCode		u8		[in]	The opcode of the command to be sent to the SPI Flash.
 *	\param	 Address	s32		[in]	The Address to read from on the SPI Flash.
 *	\param	 Data		char	[in]	Pointer to the location to store the data from the SPI Flash.
 *	\param	 DataSize	u32		[in]	The size, in bytes, of Data.
 *
 *  \return
 *      <TABLE border=0><TR><TD><i>Return</i></TD><TD>Description</TD></TR>
 *      <TR><TD><i>SUCCESS</i></TD><TD>The read was issued successfully</TD></TR>
 *	   <TR><TD><i>Other</i></TD><TD>A error occured while attempting to read from the SPI Flash</TD></TR></TABLE>
 *
 *//*
 *
 * CHANGES :
 * REF NO  DATE     WHO      DETAIL
 *         12JUN08  MG	    Initial version
 * NNNNNN  12AUG08  MG		Added Doxygen Tags
 *F*/
s32 readSPI1(u8 OpCode, s32 Address, char *Data, u32 DataSize)
{
	s32 sResult = 0;
	u32 length = 0;
	adl_busAccess_t AccessConfig =
	{
		0x0, 0x0 // Address, Op Code
	};

	TRACE((SPI_FLASH_DEBUG_TRACE, "Checking if Flash is Ready - readSPI"));
	sResult = FLASHReadyForCommand();
	if (sResult != SUCCESS)
	{
		TRACE((SPI_FLASH_DEBUG_TRACE, "Error Waiting for Flash To Be Ready. Result: %d, line: %d", sResult, __LINE__));
		//D_TRACE( "Error Waiting for Flash To Be Ready." );

		//Return Error
		return sResult;
	}

	// Default Opcode & Address size to defaults
	sResult = spi_ConfigOpAddrSize(SPI_OPCODE_SIZE, SPI_ADDRESS_SIZE);
	if (sResult != SUCCESS)
	{
		//Return Error
		return sResult;
	}

	switch (OpCode)
	{
	case READ_MNFG_CODE:     // Manufacture code

		// Set address size to zero (not needed for this command)
		length = 0x0;
		sResult = adl_busIOCtl(spi1Handle, ADL_BUS_CMD_SET_ADD_SIZE, (void *)&length);

		//Set Opcode
		AccessConfig.Opcode = (u32)READ_MNFG_CODE << 24;
		break;

	case CONT_ARRAY_READ:     // continous array read
		//Set Opcode
		AccessConfig.Opcode = (u32)OpCode << 24;

		//Set Address
		AccessConfig.Address = (u32)Address << 8;
		break;

	case BUFFER1_READ:
		//Set Opcode & Address Sizes
		sResult = spi_ConfigOpAddrSize(SPI_OPCODE_SIZE + SPI_ADDRESS_SIZE, 0);

		//Set Opcode to Opcode & Address
		AccessConfig.Opcode = (u32)BUFFER1_READ << 24;
		AccessConfig.Opcode |= (u32)Address;

		//Set Address To Zero
		AccessConfig.Address = 0;
		break;

	case MEMORY_PAGE_READ:
		//Set Opcode & Address Sizes
		sResult = spi_ConfigOpAddrSize(SPI_OPCODE_SIZE + SPI_ADDRESS_SIZE, 32);

		//Set Opcode to Opcode & Address
		AccessConfig.Opcode = ((u32)OpCode << 24) | (u32)Address;

		//Set Address to zero
		AccessConfig.Address = 0x00000000;
		break;

	case BUFFER2_READ:
		//Set Opcode & Address Sizes
		sResult = spi_ConfigOpAddrSize(SPI_OPCODE_SIZE + SPI_ADDRESS_SIZE, 32);

		//Set Opcode to Opcode & Address
		AccessConfig.Opcode = (u32)BUFFER2_READ << 24;
		AccessConfig.Opcode |= (u32)Address;

		//Set Address to zero
		AccessConfig.Address = 0;    //(u32)Address << 8;
		break;

	case BUFFER1_TO_PAGE_COMPARE:
	case BUFFER2_TO_PAGE_COMPARE:
		//Set Address Size to zero as it is not needed for this command
		length = 0x0;
		sResult = adl_busIOCtl(spi1Handle, ADL_BUS_CMD_SET_ADD_SIZE, (void *)&length);

		//Set Opcode
		AccessConfig.Opcode = (u32)STATUS_REGISTER_READ << 24;
		break;

	default:
		// we did not get a valid OpCode to read FLASH with
		sResult = SPI_UNKNOWN_OPCODE;
		TRACE((SPI_FLASH_DEBUG_TRACE, "readSPI1 Received Unknown Opcode: %d", OpCode));
		//D_TRACE( "readSPI Received Unknown Opcode" );
	}

	TRACE((SPI_FLASH_DEBUG_TRACE, "Sending Data on SPI Bus. OpCode: %d", OpCode));
	//sResult = adl_busRead( spi1Handle,&AccessConfig,DataSize,Data);

	if (sResult == SUCCESS)
	{   //readSPI1 Prep was successful, Send data on SPI Bus
		TRACE((SPI_FLASH_DEBUG_TRACE, "Sending Data on SPI Bus. OpCode: %d", OpCode));
		sResult = adl_busRead(spi1Handle, &AccessConfig, DataSize, Data);
	}
	return sResult;
}


/*F***************************************************************
 *
 *   NAME:    s32 FLASHReadyForCommand( void )
 */

/*! \brief   This function polls the SPI Flash to determine if it is in a ready state.
 *
 *   \return
 *      <TABLE border=0><TR><TD><i>Return</i></TD><TD>Description</TD></TR>
 *      <TR><TD><i>SUCCESS</i></TD><TD>The SPI Flash is in a ready state. </TD></TR>
 *	   <TR><TD><i>Other</i></TD><TD>The SPI Flash is in an busy state. </TD></TR></TABLE>
 *
 *
 *   \note   Other assumptions, hints, for the reader
 *//*
 *
 * CHANGES :
 * REF NO  DATE     WHO      DETAIL
 *         12JUN08  MG	    Initial version
 * NNNNNN  12AUG08  MG		Added Doxygen Tags
 *F*/
s32 FLASHReadyForCommand()
{
	s32 sResult = SUCCESS;
	char Data[2];
	int StatusReadCnt = DELAY_READ_STATUS_CNT;

	adl_busAccess_t AccessConfig =
	{
		0x0, 0x0 // Address, Op Code
	};

	ENTRY_TRACE("FlashReadyForCommand");
	// Set Opcode & Address size
	sResult = spi_ConfigOpAddrSize(SPI_OPCODE_SIZE, 0);

	if (sResult != SUCCESS)
	{
		//Error Setting OpAddr Size
		return sResult;
	}

	//Set Opcode
	AccessConfig.Opcode = (u32)STATUS_REGISTER_READ << 24;
	while (StatusReadCnt > 0)
	{
		ENTRY_TRACE("adl_busRead 1");
		sResult = adl_busRead(spi1Handle, &AccessConfig, 1, Data);
		EXIT_TRACE("adl_busRead 1");
		if (Data[0] & 0x80)
		{
			// we're not busy if bit 7 is set.
			TRACE((SPI_FLASH_DEBUG_TRACE, "SPI Bus Not Busy"));
			break;
		}
		TRACE((SPI_FLASH_DEBUG_TRACE, "SPI still busy %x", Data[0]));
		StatusReadCnt--;
	}

	if (StatusReadCnt > 0)
	{
		sResult = SUCCESS;
		TRACE((SPI_FLASH_DEBUG_TRACE, "Return OK, sResult: %x, Status: %x", sResult, StatusReadCnt));
	}
	else
	{
		sResult = SPI_UNKNOWN_ERROR;
		TRACE((SPI_FLASH_DEBUG_TRACE, "Return -1, sResult: %x, Status: %x", sResult, StatusReadCnt));

		ResetSPIFlash();
		/* temp code that should be removed */
		StatusReadCnt = 1;
		//D_TRACE("\n ERROR!! SPI timeout happend\n");
		while (StatusReadCnt)
		{
			ENTRY_TRACE("adl_busRead 2");
			sResult = adl_busRead(spi1Handle, &AccessConfig, 1, Data);
			EXIT_TRACE("adl_busRead 2");
			if (Data[0] & 0x80)
			{
				// we're not busy if bit 7 is set.
				TRACE((SPI_FLASH_DEBUG_TRACE, "SPI Bus Not Busy"));
				break;
			}
			else
			{
				StatusReadCnt++;
			}
		}
		//D_TRACE("count waiting for spi to be ready: %d\n",StatusReadCnt);
		TRACE((SPI_FLASH_DEBUG_TRACE, "SPI still busy %x", Data[0]));
	}

	EXIT_TRACE("FlashReadyForCommand");
	return sResult;
}


/*F***************************************************************
 *
 *   NAME:    int spi_subSPI1( void )
 */

/*! \brief   This function subscribes to the SPI1 bus service.
 *
 *   \return
 *      <TABLE border=0><TR><TD><i>Return</i></TD><TD>Description</TD></TR>
 *      <TR><TD><i>SUCCESS</i></TD><TD>The SPI1 bus service was successfully subscribed. </TD></TR>
 *	   <TR><TD><i>SPI_SUBSCRIBE_ERROR</i></TD><TD>An Error occured while attempting to subscribe to the service. </TD></TR></TABLE>
 *
 *
 *   \note   Other assumptions, hints, for the reader
 *//*
 *
 * CHANGES :
 * REF NO  DATE     WHO      DETAIL
 *         12JUN08  MG	    Initial version
 * NNNNNN  12AUG08  MG		Added Doxygen Tags
 *F*/
int spi_subSPI1(void)
{
	u32 length = 0;
	int sResult = SUCCESS;

	if (!(TEST_BIT(SPI_InitFlags, SIF_SPI_INIT)))
	{
		// from now on we will only use this handle to interact with the SPI port
		spi1Handle = adl_busSubscribe(ADL_BUS_ID_SPI, 1, (adl_busSPISettings_t *)(&spi1Settings));

		// make sure handle is OK...
		switch (spi1Handle)
		{
		case ADL_RET_ERR_PARAM:
			TRACE((SPI_FLASH_DEBUG_TRACE, "Return of busSubscribe is ADL_RET_ERR_PARAM"));
			break;

		case ADL_RET_ERR_ALREADY_SUBSCRIBED:
			TRACE((SPI_FLASH_DEBUG_TRACE, "Return of busSubscribe is ADL_RET_ERR_ALREADY_SUBSCRIBED"));
			break;

		case ADL_RET_ERR_NO_MORE_HANDLES:
			TRACE((SPI_FLASH_DEBUG_TRACE, "Return of busSubscribe is ADL_RET_ERR_NO_MORE_HANDLES"));
			break;

		case ADL_RET_ERR_BAD_HDL:
			TRACE((SPI_FLASH_DEBUG_TRACE, "Return of busSubscribe is ADL_RET_ERR_BAD_HDL"));
			break;

		default:
			if (spi1Handle < 0)
			{
				TRACE((SPI_FLASH_DEBUG_TRACE, "Return of SPI1 busSubscribe is UNKNOWN FAILURE\n"));
			}
			else
			{
				//Initialize sResult to SUCCESS
				sResult = SUCCESS;

				// Set data, address and opcode size (these values are hardware-specific)
				length = 8;
				sResult -= adl_busIOCtl(spi1Handle, ADL_BUS_CMD_SET_DATA_SIZE, (void *)&length);

				length = 32;
				sResult -= adl_busIOCtl(spi1Handle, ADL_BUS_CMD_SET_ADD_SIZE, (void *)&length);

				length = 8;
				sResult -= adl_busIOCtl(spi1Handle, ADL_BUS_CMD_SET_OP_SIZE, (void *)&length);

				if (sResult == SUCCESS)
				{       //SPI Bus Subscribed & Configured Correctly
					SET_BIT(SPI_InitFlags, SIF_SPI_INIT);
					TRACE((SPI_FLASH_DEBUG_TRACE, "SPI 1 bus successfully subscribed."));
				}
				else
				{       //SPI Bus was not configured correctly
					sResult = SPI_SUBSCRIBE_ERROR;

					TRACE((SPI_FLASH_DEBUG_TRACE, "Return of SPI1 busSubscribe is UNKNOWN FAILURE\n"));
				}
			}
			break;
		}
	}
	return sResult;
}


/*F***************************************************************
 *
 *   NAME:    int readMnfgCode( void )
 */

/*! \brief   This function reads the Manufacture Data of the SPI Flash outputs it through a debug message.
 *
 *   \return
 *      <TABLE border=0><TR><TD><i>Return</i></TD><TD>Description</TD></TR>
 *      <TR><TD><i>SUCCESS</i></TD><TD>The SPI Flash Manufacture Data was successfully output. </TD></TR>
 *	   <TR><TD><i>Other</i></TD><TD>An Error occured while attempting to read the SPI Flash Manufacture data. </TD></TR></TABLE>
 *
 *
 *   \note   Other assumptions, hints, for the reader
 *//*
 *
 * CHANGES :
 * REF NO  DATE     WHO      DETAIL
 *         12JUN08  MG	    Initial version
 * NNNNNN  12AUG08  MG		Added Doxygen Tags
 *F*/
int readMnfgCode()
{
	char buffer[6];
	s32 sResult = SPI_UNKNOWN_ERROR;

	//Read the manufacturing code
	sResult = readSPI1(0x9f, 0x0, (char *)buffer, 5);
#ifdef DEBUG
	if (sResult < 0)
	{
		TRACE((SPI_FLASH_ERROR_TRACE, "ERROR: Failed to read from SPI"));
	}
	else
	{
		//display the output
		buffer[5] = '\0';
		//D_TRACE("Device Data: %x %x %x %x %x",(u8)buffer[0],(u8)buffer[1],(u8)buffer[2],(u8)buffer[3],(u8)buffer[4]);
	}
#endif
	return sResult;
}


/*F***************************************************************
 *
 *   NAME:    int spi_EraseChip( void )
 */

/*! \brief   This function issues the Erase Chip command to the SPI Flash.
 *
 *   \return
 *      <TABLE border=0><TR><TD><i>Return</i></TD><TD>Description</TD></TR>
 *      <TR><TD><i>SUCCESS</i></TD><TD>The Erase Chip command was successfuly issued. </TD></TR>
 *	   <TR><TD><i>Other</i></TD><TD>An Error occured while attempting issue the Erase Chip command. </TD></TR></TABLE>
 *
 *
 *   \note   Other assumptions, hints, for the reader
 *//*
 *
 * CHANGES :
 * REF NO  DATE     WHO      DETAIL
 *         12JUN08  MG	    Initial version
 * NNNNNN  12AUG08  MG		Added Doxygen Tags
 *F*/
int spi_EraseChip()
{
	s32 sResult = SPI_UNKNOWN_ERROR;
	char *test = "0";
	sResult = writeSPI1(FULL_FLASH_ERASE, 0, test, 1);
	if (sResult == SUCCESS)
	{
		TRACE((SPI_FLASH_STATUS_TRACE, "Chip Erased Successfully"));
	}
	else
	{
		TRACE((SPI_FLASH_ERROR_TRACE, "Error occured while erasing chip, Error:%d", sResult));
	}
	return sResult;
}


/*F***************************************************************
 *
 *   NAME:    int spi_WriteBuffer( Buffers buffer, u32 offset, char *Data, u32 length )
 */

/*! \brief   This function writes data to a Buffer on the SPI Flash.
 *
 *	\param  buffer	Buffers	[in]	The SPI Flash buffer to write data to.
 *	\param  offset	u32		[in]	The offset within the buffer at which to write.
 *	\param  Data	char	[in]	The pointer to the data to write to the SPI Flash buffer.
 *	\param  length	u32		[in]	The number of bytes to write the SPI Flash buffer.
 *
 *   \return
 *      <TABLE border=0><TR><TD><i>Return</i></TD><TD>Description</TD></TR>
 *      <TR><TD><i>SUCCESS</i></TD><TD>The data was sent to the SPI Flash successfully </TD></TR>
 *	   <TR><TD><i>SPI_INVALID_PARAM</i></TD><TD>At least one of the parameters provided was invalid. </TD></TR>
 *	   <TR><TD><i>Other</i></TD><TD>An error occured while writing data to the SPI Flash Buffer </TD></TR></TABLE>
 *
 *//*
 *
 * CHANGES :
 * REF NO  DATE     WHO      DETAIL
 *         12JUN08  MG	    Initial version
 * NNNNNN  12AUG08  MG		Added Doxygen Tags
 *F*/
int spi_WriteBuffer(Buffers buffer, u32 offset, char *Data, u32 length)
{
	s32 sResult = SPI_UNKNOWN_ERROR;
	s32 sendAddress = 0x0000;

	if ((offset & 0xfffff800) != 0)
	{
		TRACE((SPI_FLASH_DEBUG_TRACE, "***ERROR: Cannon Write Buffer Invalid Address: %d", offset));
		sResult = SPI_INVALID_PARAM;
	}
	else
	{
		sendAddress = (s32)offset;

		TRACE((SPI_FLASH_DEBUG_TRACE, "Sample of Data to Be Written To Buffer %d: %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x", buffer, (u8)Data[0], (u8)Data[1], (u8)Data[2], (u8)Data[3], (u8)Data[4], (u8)Data[5], (u8)Data[6], (u8)Data[7]));
		if (buffer == Buffer1)
		{
			sResult = writeSPI1(BUFFER1_WRITE, sendAddress, Data, length);
		}
		else
		{
			sResult = writeSPI1(BUFFER2_WRITE, sendAddress, Data, length);
		}
		TRACE((SPI_FLASH_DEBUG_TRACE, "Wrote To Buffer %d at offset: %d ", buffer, offset));
	}
	return sResult;
}


/*F***************************************************************
 *
 *   NAME:    int spi_ReadBuffer( Buffers buffer, u32 offset, char *Data, u32 length )
 */

/*! \brief   This function writes data to a Buffer on the SPI Flash.
 *
 *	\param  buffer	Buffers	[in]	The SPI Flash buffer to read data from.
 *	\param  offset	u32		[in]	The offset within the buffer at which to read.
 *	\param  Data	char	[in]	The pointer at which to store the data read from the SPI Flash buffer.
 *	\param  length	u32		[in]	The number of bytes to read from the SPI Flash buffer.
 *
 *   \return
 *      <TABLE border=0><TR><TD><i>Return</i></TD><TD>Description</TD></TR>
 *      <TR><TD><i>SUCCESS</i></TD><TD>Data was read from the SPI Flash successfully </TD></TR>
 *	   <TR><TD><i>SPI_INVALID_PARAM</i></TD><TD>At least one of the parameters provided was invalid. </TD></TR>
 *	   <TR><TD><i>Other</i></TD><TD>An error occured while reading data to the SPI Flash Buffer </TD></TR></TABLE>
 *
 *   \note   Other assumptions, hints, for the reader
 *//*
 *
 * CHANGES :
 * REF NO  DATE     WHO      DETAIL
 *         12JUN08  MG	    Initial version
 * NNNNNN  12AUG08  MG		Added Doxygen Tags
 *F*/
int spi_ReadBuffer(Buffers buffer, u32 offset, char *Data, u32 length)
{
	s32 sResult = SPI_UNKNOWN_ERROR;
	s32 sendAddress = 0x0000;

	if ((offset & 0xfffff800) != 0)
	{
		TRACE((SPI_FLASH_DEBUG_TRACE, "*** ERROR: Cannot Read Buffer. Invalid Offset: %d", offset));
		sResult = SPI_INVALID_PARAM;
	}
	else
	{
		sendAddress = (s32)offset;
		if (buffer == Buffer1)
		{
			sResult = readSPI1(BUFFER1_READ, sendAddress, Data, length);
		}
		else
		{
			sResult = readSPI1(BUFFER2_READ, sendAddress, Data, length);
		}
		if (sResult < 0)
		{
			TRACE((SPI_FLASH_DEBUG_TRACE, "***ERROR: Failed to read from Buffer %d. Code: %d", sResult));
		}
		else
		{
			TRACE((SPI_FLASH_DEBUG_TRACE, "Read Buffer %d at offset: %d", buffer, sendAddress));
		}
	}
	return sResult;
}


/*F***************************************************************
 *
 *   NAME:    int spi_BufferToMemory( Buffers buffer, u32 Page )
 */

/*! \brief   This function issues the Buffer To Memory command.
 *
 *	\param  buffer	Buffers	[in]	The SPI Flash buffer to write to memory page.
 *	\param  Page	u32		[in]	The SPI Flash memory page to write the buffer data to.
 *
 *   \return
 *      <TABLE border=0><TR><TD><i>Return</i></TD><TD>Description</TD></TR>
 *      <TR><TD><i>SUCCESS</i></TD><TD>Buffer to Memory command was issued successfully </TD></TR>
 *	   <TR><TD><i>SPI_INVALID_PARAM</i></TD><TD>At least one of the parameters provided was invalid. </TD></TR>
 *	   <TR><TD><i>Other</i></TD><TD>An error occured while issuing the Buffer to Memory command </TD></TR></TABLE>
 *
 *   \note   Other assumptions, hints, for the reader
 *//*
 *
 * CHANGES :
 * REF NO  DATE     WHO      DETAIL
 *         12JUN08  MG	    Initial version
 * NNNNNN  12AUG08  MG		Added Doxygen Tags
 *F*/
int spi_BufferToMemory(Buffers buffer, u32 Page)            //Currently using the built in erase.
{
	s32 sResult = SPI_UNKNOWN_ERROR;
	s32 sendAddress;
	char test; // = 0x00000083;

	TRACE((SPI_FLASH_DEBUG_TRACE, "Writing Buffer To Memory. Buffer: %d, Page: %d", buffer, Page));

	if ((Page & 0xffffe000) != 0)
	{
		TRACE((SPI_FLASH_DEBUG_TRACE, "***ERROR: Cannot Write Buffer %d To Memory. Invalid Page: %d", buffer, Page));
		sResult = SPI_INVALID_PARAM;
	}
	else
	{
		//Construct Page Address
		sendAddress = (Page << 19);

		if (buffer == Buffer1)
		{
			sResult = writeSPI1(BUFFER1_TO_MEMORY, sendAddress, (char *)&test, 0);
		}
		else
		{
			sResult = writeSPI1(BUFFER2_TO_MEMORY, sendAddress, (char *)&test, 0);
		}
		if (sResult == SUCCESS)
		{
			TRACE((SPI_FLASH_DEBUG_TRACE, "Buffer %d Successfully Written to Page %d", buffer, Page));
		}
		else
		{
			TRACE((SPI_FLASH_DEBUG_TRACE, "***ERROR: Failed to Write Buffer: %d To Page %d. Code: %d", buffer, Page, sResult));
		}
	}
	return sResult;
}


/*F***************************************************************
 *
 *   NAME:    int spi_ReadPage( u32 page, u32 offset, char *Data, u32 length )
 */

/*! \brief   This function issues the Buffer To Memory command.
 *
 *	\param  page	u32	[in]	The SPI Flash page to read data from.
 *	\param  offset	u32		[in]	The offset within the page at which to read.
 *	\param  Data	char	[in]	The pointer the the location to store the data read from the SPI Flash page.
 *	\param  length	u32		[in]	The number of bytes to read from the SPI Flash page.
 *
 *   \return
 *      <TABLE border=0><TR><TD><i>Return</i></TD><TD>Description</TD></TR>
 *      <TR><TD><i>SUCCESS</i></TD><TD>Data was read from the SPI Flash successfully </TD></TR>
 *	   <TR><TD><i>Other</i></TD><TD>An error occured while reading data to the SPI Flash </TD></TR></TABLE>
 *
 *//*
 *
 * CHANGES :
 * REF NO  DATE     WHO      DETAIL
 *         12JUN08  MG	    Initial version
 * NNNNNN  12AUG08  MG		Added Doxygen Tags
 *F*/
int spi_ReadPage(u32 page, u32 offset, char *Data, u32 length)
{
	s32 sResult;
	s32 sendAddress;

	sResult = SPI_UNKNOWN_ERROR;
	sendAddress = (s32)page << 11;
	sendAddress |= (s32)offset;

	sResult = readSPI1(MEMORY_PAGE_READ, sendAddress, Data, (int)length);
	if (sResult == 0)
	{
		TRACE((SPI_FLASH_DEBUG_TRACE, "Page %d Read Successfully", page));
	}
	else
	{
		TRACE((SPI_FLASH_DEBUG_TRACE, "***ERROR: Failed to read Page %d. Code: %d", page, sResult));
	}
	return sResult;
}


/*F***************************************************************
 *
 *   NAME:    int spi_ComparePageToBuffer( Buffers buffer, u32 Page )
 */

/*! \brief   This function compares a SPI Flash memory page to a buffer.
 *
 *	\param  buffer	Buffers	[in]	The SPI Flash buffer to compare.
 *	\param  Page	u32		[in]	The SPI Flash page to compare.
 *
 *   \return
 *      <TABLE border=0><TR><TD><i>Return</i></TD><TD>Description</TD></TR>
 *      <TR><TD><i>SUCCESS</i></TD><TD>Data in the buffer matches the data in the </TD></TR>
 *	   <TR><TD><i>Other</i></TD><TD>The data in the buffer and page do not match or an error occurred </TD></TR></TABLE>
 *
 *   \note   Other assumptions, hints, for the reader
 *//*
 *
 * CHANGES :
 * REF NO  DATE     WHO      DETAIL
 *         12JUN08  MG	    Initial version
 * NNNNNN  12AUG08  MG		Added Doxygen Tags
 *F*/
int spi_ComparePageToBuffer(Buffers buffer, u32 Page)
{
	s32 sResult = SPI_UNKNOWN_ERROR;
	s32 sendAddress;
	char Data[1] = { 0 };
	if ((Page & 0xffffe000) != 0)
	{
		TRACE((SPI_FLASH_DEBUG_TRACE, "***ERROR: Cannot Compare Page To Buffer %d. Invalid Page: %d", buffer, Page));
		sResult = SPI_INVALID_PARAM;
	}
	else
	{
		sendAddress = (Page << 11);

		//Begin Compare
		if (buffer == Buffer1)
		{
			sResult = writeSPI1(BUFFER1_TO_PAGE_COMPARE, sendAddress, (char *)Data, 0);
		}
		else
		{
			sResult = writeSPI1(BUFFER2_TO_PAGE_COMPARE, sendAddress, (char *)Data, 0);
		}
		if (sResult == SUCCESS)
		{
			TRACE((SPI_FLASH_DEBUG_TRACE, "Compare Initiated Between Buffer %d and Page %d", buffer, Page));

			//Get Compare Result
			if (buffer == Buffer1)
			{
				sResult = readSPI1(BUFFER1_TO_PAGE_COMPARE, 0, (char *)Data, 1);
			}
			else
			{
				sResult = readSPI1(BUFFER2_TO_PAGE_COMPARE, 0, (char *)Data, 1);
			}

			if (sResult == SUCCESS)
			{
				//Compare Completed Successfully

				//Get Compare Result
				sResult = ((Data[0] & (1 << 6)) >> 6);
				TRACE((SPI_FLASH_DEBUG_TRACE, "Compare Between Buffer %d and Page %d Completed. Result: %02x", buffer, Page, sResult));
			}
			else
			{
				TRACE((SPI_FLASH_DEBUG_TRACE, "***ERROR: Compare Between Between Buffer %d and Page %d Failed. Code: %d", buffer, Page, sResult));
			}
		}
		else
		{
			TRACE((SPI_FLASH_DEBUG_TRACE, "***ERROR: Failed To Initiate Compare Between Buffer %d and Page %d", buffer, Page));
		}
	}
	return sResult;
}


/*F***************************************************************
 *
 *   NAME:    int spi_ReadPageToBuffer( Buffers buffer,u32 page )
 */

/*! \brief   This function reads a page of SPI Flash into a buffer.
 *
 *	\param  buffer	Buffers	[in]	The buffer to place page data in.
 *	\param  page	u32		[in]	The SPI Flash page to copy to buffer.
 *
 *   \return
 *      <TABLE border=0><TR><TD><i>Return</i></TD><TD>Description</TD></TR>
 *      <TR><TD><i>SUCCESS</i></TD><TD>The Read Page To Buffer command was sucessfully issued </TD></TR>
 *	   <TR><TD><i>Other</i></TD><TD>An error occured issuing the command. </TD></TR></TABLE>
 *
 *   \note   Other assumptions, hints, for the reader
 *//*
 *
 * CHANGES :
 * REF NO  DATE     WHO      DETAIL
 *         12JUN08  MG	    Initial version
 * NNNNNN  12AUG08  MG		Added Doxygen Tags
 *F*/
int spi_ReadPageToBuffer(Buffers buffer, u32 page)
{
	s32 sResult = SPI_UNKNOWN_ERROR;
	char Data[1] = { 0 };

	if (buffer == Buffer1)
	{
		sResult = writeSPI1(PAGE_TO_BUFFER1, page, (char *)&Data, 0);
	}
	else
	{
		sResult = writeSPI1(PAGE_TO_BUFFER2, page, (char *)&Data, 0);
	}

	if (sResult == SUCCESS)
	{
		TRACE((SPI_FLASH_DEBUG_TRACE, "Page Read To Buffer %d Completed Successfully. Page: %d", buffer, page));
	}
	else
	{
		TRACE((SPI_FLASH_DEBUG_TRACE, "***ERROR: Page Read To Buffer %d Failed. Page: %d Code: %d.", buffer, page, sResult));
	}
	return sResult;
}


/*F***************************************************************
 *
 *   NAME:    int spi_ConfigOpAddrSize( u32 OpSize, u32 AddrSize )
 */

/*! \brief   This function sets the legnth of a SPI bus opcode and address for SPI1.
 *
 *	\param  OpSize		u32		[in]	The size, in bits, of a SPI opcode.
 *	\param  AddrSize	u32		[in]	The size, in bits, of a SPI address.
 *
 *   \return
 *      <TABLE border=0><TR><TD><i>Return</i></TD><TD>Description</TD></TR>
 *      <TR><TD><i>SUCCESS</i></TD><TD>The Opcode and Address size was set sucessfully</TD></TR>
 *	   <TR><TD><i>SPI_Opcode_SIZE_ERROR</i></TD><TD>The Opcode size could not be set. </TD></TR>
 *      <TR><TD><i>SPI_ADDRESS_SIZE_ERROR</i></TD><TD>The Address size could not be set.</TD></TR></TABLE>
 *
 *   \note   Other assumptions, hints, for the reader
 *//*
 *
 * CHANGES :
 * REF NO  DATE     WHO      DETAIL
 *         12JUN08  MG	    Initial version
 * NNNNNN  12AUG08  MG		Added Doxygen Tags
 *F*/
int spi_ConfigOpAddrSize(u32 OpSize, u32 AddrSize)
{
	int sResult = SUCCESS;

	ENTRY_TRACE("spi_ConfigOpAddrSize");

	// Set SPI Opcode size
	if (adl_busIOCtl(spi1Handle, ADL_BUS_CMD_SET_OP_SIZE, (void *)&OpSize) != SUCCESS)
	{
		//Error occurred while setting Opcode Size
		sResult = SPI_Opcode_SIZE_ERROR;
	}

	//Set SPI Address Size
	if (adl_busIOCtl(spi1Handle, ADL_BUS_CMD_SET_ADD_SIZE, (void *)&AddrSize) != SUCCESS)
	{
		//Error occurred while setting Opcode Size
		sResult = SPI_ADDRESS_SIZE_ERROR;
	}

	EXIT_TRACE("spi_ConfigOpAddrSize");
	return sResult;
}


/**@}*/
