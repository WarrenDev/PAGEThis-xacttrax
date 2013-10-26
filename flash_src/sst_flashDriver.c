/** @addtogroup SpiFlash
 *@{*/

/** @file sst_flashDriver.c
 */

/******************************************************************************
 * @brief  This file contains the APIs to interact with the SST SPI flash
 *
 *****************************************************************************/

#include "sst_flashDriver.h"

#include <adl_global.h>
#include "adl_shoring.h"

#include "gpioTest.h"
#include "bele.h"
#include "Traces.h"
#include "xactutilities.h"

#define OP_32_BIT(b3, b2, b1, b0)   MAKE32_4((b3), (b2), (b1), (b0))

#define OP_8_BIT(b3)                OP_32_BIT((b3), 0, 0, 0)

#define OP_READ                         OP_8_BIT(0x03)
#define OP_HI_SPEED_READ                OP_8_BIT(0x0b)
#define OP_SECTOR_ERASE                 OP_8_BIT(0x20)
#define OP_CHIP_ERASE                   OP_8_BIT(0x60)
#define OP_PAGE_PROGRAM                 OP_8_BIT(0x02)
#define OP_READ_STATUS_REGISTER         OP_8_BIT(0x05)
#define OP_WRITE_ENABLE                 OP_8_BIT(0x06)
#define OP_ENABLE_WRITE_STATUS_REGISTER OP_8_BIT(0x50)
#define OP_WRITE_STATUS_REGISTER        OP_8_BIT(0x01)
#define OP_READ_ID                      OP_8_BIT(0x90)

#define STATUS_BUSY                     BIT(0)
#define STATUS_WRITE_ENABLED            BIT(1)
#define STATUS_BLOCK_WRITE_PROTECTION_0 BIT(2)
#define STATUS_BLOCK_WRITE_PROTECTION_1 BIT(3)
#define STATUS_BLOCK_WRITE_PROTECTION_2 BIT(4)
#define STATUS_BLOCK_WRITE_PROTECTION_3 BIT(5)
#define STATUS_SECURITY_IS_STATUS       BIT(6)
#define STATUS_PROTECTION_TYPE          BIT(7)

#define STATUS_NO_WRITE_PROTECTION      ~(STATUS_BLOCK_WRITE_PROTECTION_0 | STATUS_BLOCK_WRITE_PROTECTION_1 | STATUS_BLOCK_WRITE_PROTECTION_2 | STATUS_BLOCK_WRITE_PROTECTION_3)

#define PAGE_SIZE   256
#define SECTOR_SIZE kibibytes_to_bytes(4)

#define MAX_BITS(v, n)  (((v) ~((1 << ((n) + 1)) - 1)) == 0)

#define ADDRESS_BITS    24
#define ADDRESS_PAD_RIGHT(n)    ((n) << (32 - ADDRESS_BITS))

#define FLASH_RESET_GPIO 32

DEBUG_TRACE_STORAGE;

static s32 spi_bus_handle = ADL_RET_ERR_BAD_HDL;
//static s32 spi_reset_gpio_handle = -1;

static void set_size(adl_busIoCtlCmd_e command, u32 value)
{
	s32 const result = adl_busIOCtl(spi_bus_handle, command, &value);
	ASSERT(result == OK);
}


#define VALID_BUFFER(buffer, length)    (((buffer) != NULL) && ((length) != 0))
#define NOUSE_BUFFER(buffer, length)    (((buffer) == NULL) && ((length) == 0))

static void spi_bus_read(u32 opcode, u32 address, void *buffer, size_t length)
{
	ASSERT(VALID_BUFFER(buffer, length));

	u32 address_in_msb = ADDRESS_PAD_RIGHT(address);
	{
		adl_busAccess_t access = { .Opcode = opcode, .Address = address_in_msb };
		s32 const result = adl_busRead(spi_bus_handle, &access, length, buffer);

		ASSERT(result == OK);
	}
}


static void spi_bus_write(u32 opcode, u32 address, void *buffer, size_t length)
{
	ASSERT(VALID_BUFFER(buffer, length) || NOUSE_BUFFER(buffer, length));

	u32 address_in_msb = ADDRESS_PAD_RIGHT(address);
	{
		adl_busAccess_t access = { .Opcode = opcode, .Address = address_in_msb };
		/* Thanks, Wavecom! Even it we have no data to write (just an opcode), buffer cannot be NULL. FIXME: report to Wavecom.  -ASK */
		void *buffer_hack = (buffer == NULL) ? &access : buffer;
		s32 const result = adl_busWrite(spi_bus_handle, &access, length, buffer_hack);

		ASSERT(result == OK);
	}
}


static unsigned char get_status(void)
{
	char status;

	set_size(ADL_BUS_CMD_SET_ADD_SIZE, 0);
	spi_bus_read(OP_READ_STATUS_REGISTER, 0, &status, sizeof status);

	return status;
}


static void wait_for_device_ready(void)
{
	for ( ; ;)
	{
		unsigned char const busy_mask = STATUS_BUSY;
		unsigned char const not_busy = 0x0;

		if ((get_status() & busy_mask) == not_busy)
		{
			break;
		}
	}
}


static void spiFlash_Init(void)
{
	static const adl_busSPISettings_t settings =
	{
		1,                          /* No divider, use full clock speed */
		ADL_BUS_SPI_CLK_MODE_0,     /* Mode 0 clock	*/
		ADL_BUS_SPI_ADDR_CS_GPIO,   /* Use a GPIO to handle the Chip Select signal	*/
		ADL_BUS_SPI_CS_POL_LOW,     /* Chip Select active in low state	*/
		ADL_BUS_SPI_MSB_FIRST,      /* Data are sent MSB first	*/
		31,                         /* Use GPIO 31 to handle Chip Select */
		ADL_BUS_SPI_LOAD_UNUSED,    /* LOAD signal not used */
		ADL_BUS_SPI_DATA_UNIDIR,    /* 3 Wires configuration */
		ADL_BUS_SPI_MASTER_MODE,    /* Slave mode */
		ADL_BUS_SPI_BUSY_UNUSED     /* BUSY signal not used */
	};
	s32 const handle = adl_busSubscribe(ADL_BUS_ID_SPI, 1, UNCONST(adl_busSPISettings_t *, &settings));

	ASSERT_GOTO(handle >= 0, error);

	spi_bus_handle = handle;

	/* configure the bus */
	set_size(ADL_BUS_CMD_SET_OP_SIZE, 8);
	set_size(ADL_BUS_CMD_SET_DATA_SIZE, 8);

	{
		char default_status = get_status();
		char write_status = default_status & STATUS_NO_WRITE_PROTECTION;

		set_size(ADL_BUS_CMD_SET_ADD_SIZE, 0);
		spi_bus_write(OP_ENABLE_WRITE_STATUS_REGISTER, 0, NULL, 0);
		spi_bus_write(OP_WRITE_STATUS_REGISTER, 0, &write_status, sizeof write_status);

		{
			char new_status;
			new_status = get_status();
			ASSERT((new_status & ~(STATUS_NO_WRITE_PROTECTION)) == 0);
		}
	}

error:
	return;
}


static void spiFlash_Read(unsigned long flash_byte_offset, unsigned char *const the_page, unsigned long bytes_to_read)
{
	wait_for_device_ready();

/* FIXME? The fast write supports up to 80MHz speed over the SPI bus.  I'm not sure the WMP100 can support this speed -- pjn*/
#if defined(FAST_READ)
	set_size(ADL_BUS_CMD_SET_ADD_SIZE, 32);
	spi_bus_read(OP_HI_SPEED_READ, flash_byte_offset, UNCONST(char *, the_page), bytes_to_read);
#else
	set_size(ADL_BUS_CMD_SET_ADD_SIZE, 24);
	spi_bus_read(OP_READ, flash_byte_offset, UNCONST(char *, the_page), bytes_to_read);
#endif
}


static int spiFlash_Program(unsigned long flash_byte_offset, unsigned char const *const the_page, unsigned long page_len)
{
	ASSERT(page_len == PAGE_SIZE);

	wait_for_device_ready();

	set_size(ADL_BUS_CMD_SET_ADD_SIZE, 0);
	spi_bus_write(OP_WRITE_ENABLE, 0, NULL, 0);

	set_size(ADL_BUS_CMD_SET_ADD_SIZE, 24);
	spi_bus_write(OP_PAGE_PROGRAM, flash_byte_offset, UNCONST(char *, the_page), page_len);

//#define VERIFY_WRITE
#if defined(VERIFY_WRITE)
	{
		char verify_buffer[PAGE_SIZE];

		memset(verify_buffer, 0x0, sizeof verify_buffer);

		wait_for_device_ready();

		set_size(ADL_BUS_CMD_SET_ADD_SIZE, 32);
		spi_bus_read(OP_HI_SPEED_READ, flash_byte_offset, verify_buffer, sizeof verify_buffer);

		if (memcmp(verify_buffer, the_page, PAGE_SIZE) != 0)
		{
			ASSERT(false);
		}
	}
#endif

	return 0;
}


static int spiFlash_Erase_Sector(unsigned long flash_byte_offset)
{
	ASSERT((flash_byte_offset % SECTOR_SIZE) == 0);

	wait_for_device_ready();

	set_size(ADL_BUS_CMD_SET_ADD_SIZE, 0);
	spi_bus_write(OP_WRITE_ENABLE, 0, NULL, 0);

	set_size(ADL_BUS_CMD_SET_ADD_SIZE, 24);
	spi_bus_write(OP_SECTOR_ERASE, flash_byte_offset, NULL, 0);

//#define VERIFY_ERASE_SECTOR
#if defined(VERIFY_ERASE_SECTOR)
	{
		char verify_buffer[PAGE_SIZE];
		char blank_page[PAGE_SIZE];

		memset(verify_buffer, 0x0, sizeof verify_buffer);
		memset(blank_page, 0xff, sizeof blank_page);

		wait_for_device_ready();

		set_size(ADL_BUS_CMD_SET_ADD_SIZE, 32);
		spi_bus_read(OP_HI_SPEED_READ, flash_byte_offset, verify_buffer, sizeof verify_buffer);

		if (memcmp(verify_buffer, blank_page, PAGE_SIZE) != 0)
		{
			ASSERT(false);
		}
	}
#endif

	return 0;
}


//#define VERIFY_CHIP_ERASE
int spiFlash_EraseChip(void)
{
	wait_for_device_ready();

#if defined(VERIFY_CHIP_ERASE)
	{
		char verify_buffer[PAGE_SIZE];

		memset(verify_buffer, 0x0, sizeof verify_buffer);

		set_size(ADL_BUS_CMD_SET_ADD_SIZE, 32);
		spi_bus_read(OP_HI_SPEED_READ, 0, verify_buffer, sizeof verify_buffer);

		wait_for_device_ready();
	}
#endif

	set_size(ADL_BUS_CMD_SET_ADD_SIZE, 0);
	spi_bus_write(OP_WRITE_ENABLE, 0, NULL, 0);

	set_size(ADL_BUS_CMD_SET_ADD_SIZE, 24);
	spi_bus_write(OP_CHIP_ERASE, 0, NULL, 0);

#if defined(VERIFY_CHIP_ERASE)
	{
		char verify_buffer[PAGE_SIZE];
		char blank_page[PAGE_SIZE];

		memset(verify_buffer, 0x0, sizeof verify_buffer);
		memset(blank_page, 0xff, sizeof blank_page);

		wait_for_device_ready();

		set_size(ADL_BUS_CMD_SET_ADD_SIZE, 32);
		spi_bus_read(OP_HI_SPEED_READ, 0, verify_buffer, sizeof verify_buffer);

		wait_for_device_ready();

		if (memcmp(verify_buffer, blank_page, PAGE_SIZE) != 0)
		{
			ASSERT(false);
		}
	}
#endif

	return 0;
}


void spiFlash_ReadID(void)
{
	char device_id[] = "ID";
	wait_for_device_ready();

	set_size(ADL_BUS_CMD_SET_ADD_SIZE, 0);
	spi_bus_read(OP_READ_ID, 0, &device_id, sizeof device_id);
}


/** @brief This method will initialize the SST flash part
 *
 * @par description: This method will set the SPI flash reset GPIO to inactive, and then will initialize the SPI flash by appropriately setting up the status registers to disable memory protection
 * @return SUCCESS
 */
int sf_MediaInit(u32 unused)
{
	UNUSED_PARAMETER(unused);

	DumpMessage("Entered sf_MediaInit\r\n");
/*
	adl_ioDefs_t gpio_config = FLASH_RESET_GPIO | ADL_IO_GPIO | ADL_IO_DIR_OUT | ADL_IO_LEV_HIGH;
	spi_reset_gpio_handle = adl_ioSubscribe(1, &gpio_config, 0, 0, 0);
	if(spi_reset_gpio_handle)
		DumpMessage("spi_reset_gpio_handle is not NULL\r\n");	
	else
		DumpMessage("spi_reset_gpio_handle is *NULL\r\n");	
	ASSERT(spi_reset_gpio_handle >= 0);

	{
		adl_ioDefs_t gpio_to_write = ADL_IO_GPIO | FLASH_RESET_GPIO;
		DumpMessage("Before adl_ioWriteSingle\r\n");
		s32 result = adl_ioWriteSingle(spi_reset_gpio_handle, &gpio_to_write, true);
		if(result != OK)
			DumpMessage("adl_ioWriteSingle failed\r\n");
		ASSERT(result == OK);
		DumpMessage("adl_ioWriteSingle passed\r\n");
	}
*/
	DumpMessage("Entering spiFlash_Init\r\n");	
	spiFlash_Init();
	return 0;
}


#define PAGE_TO_OFFSET(page)    (page * PAGE_SIZE)
#define PAGES_PER_SECTOR            (SECTOR_SIZE / PAGE_SIZE)

/** @brief This method is used to write an entire 4kB sector to the SST flash part
 *
 * @par description: This method is used to write entire sectors to the SST flash part.
 * This method will internally call EraseSector followed by 16 PageProgram commands.
 *
 * @param page The page number
 * @param buffer The data buffer to write
 * @return SUCCESS
 */
int sf_writePage(unsigned int page, unsigned char const *const buffer)
{
	unsigned long address = PAGE_TO_OFFSET(page);

	/* TODO: this code only works in append mode.  if we write to a page other than the
	 *       first page in the sector and then we rewrite the first page of the sector
	 *       we will see data corruption because the full sector will be erased -- pjn */
	if (page % PAGES_PER_SECTOR == 0)
	{
		spiFlash_Erase_Sector(address);
	}

	/* TODO: might be better to abstract both erase and program, but for now I will keep the method similar.  A NULL buffer indicates an erase method -- pjn*/
	if (buffer != NULL)
	{
		int result = spiFlash_Program(address, buffer, PAGE_SIZE);
		INSIST(result == OK);
	}

	return 0;
}


/** @brief This method is used to read an entire 4kB sector to the SST flash part
 *
 * @par description: This method is used to readentire sectors to the SST flash part.
 *
 *
 * @param page The page number
 * @param buffer The data buffer to read to
 * @return SUCCESS
 */
int sf_readPage(unsigned int page, unsigned char *const buffer)
{
	unsigned long address = PAGE_TO_OFFSET(page);
	spiFlash_Read(address, buffer, PAGE_SIZE);
	return 0;
}


/*@}*/
