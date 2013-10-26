/** @addtogroup Storage
 *@{*/
#ifndef __spiflash_H_
#define __spiflash_H_

/**************************************************************************
 * FILE CONTEXT
 ***************************************************************************/

/*A****************************************************************
 * NAME:	SPI Flash API
 *
 * USAGE: These are all the variables and APIs that are used to
 *		 access the SPI Flash.
 *
 * CHANGES :
 * REF NO	DATE	WHO		DETAIL
 *			12JUN08	MG		Initial Version
 * NNNNNN	12AUG08 MG		Added Doxygen Tags
 *
 *A*/
/*---- context ---------------------------------------------------*/
//Global Constants

// RESULT CODES
#define SUCCESS                         0                           /*!< Success Value */
#define SPI_RESPONSE_BASE               (-100)                      /*!< Base from which all other Response Codes are derived */
#define SPI_UNKNOWN_ERROR               (SPI_RESPONSE_BASE - 1)     /*!< An Unknown Error Occured */
#define SPI_SECTOR_OUT_OF_RANGE         (SPI_RESPONSE_BASE - 2)     /*!< Invalid Sector oOr Size Specified */
#define SPI_FLASH_NOT_INITIALIZED       (SPI_RESPONSE_BASE - 3)     /*!< Operation Attemped Before Flash Was Initialized */
#define SPI_SUBSCRIBE_ERROR             (SPI_RESPONSE_BASE - 4)     /*!< An Error Occurred While Subscribing SPI Bus */
#define SPI_UNKNOWN_OPCODE              (SPI_RESPONSE_BASE - 5)     /*!< Invalid SPI Flash Opcode received */
#define SPI_Opcode_SIZE_ERROR           (SPI_RESPONSE_BASE - 6)     /*!< Error Setting SPI Opcode Size */
#define SPI_ADDRESS_SIZE_ERROR          (SPI_RESPONSE_BASE - 7)     /*!< Error Setting SPI Address Size */
#define SPI_INVALID_PARAM               (SPI_RESPONSE_BASE - 8)     /*!< SPI Flash Command Received an Invalid Parameter */
#define SPI_READ_ERROR                  (SPI_RESPONSE_BASE - 9)     /*!< SPI Driver Experianced A Read Error */
#define SPI_WRITE_ERROR                 (SPI_RESPONSE_BASE - 10)    /*!< SPI Driver Experianced A Write Error */

/*! \enum Buffers_e
 *	\brief Enumeration for Buffers on the SPI Flash */
enum Buffers_e
{
	Buffer1    = 0, /*!< Represents Buffer 1 on the SPI Flash */
	Buffer2    = 1  /*!< Represents Buffer 2 on the SPI Flash */
};

typedef enum Buffers_e Buffers;     /*!< Creates a type name for Buffers_e */

#define SECTOR_SIZE             512
#define NUMBER_OF_SECTORS       8192 * 2

/*---- extern function prototypes ----------------------------------------*/
extern int sf_MediaInit(u32 SectorLength);
extern int sf_readSector(u32 sectorNum, u8 *buf);
extern int sf_writeSector(u32 sectorNum, u8 *Data);

extern int spi_EraseChip(void);
#endif

/**@}*/
