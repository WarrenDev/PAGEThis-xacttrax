#if !defined(SST_FLASH_DRIVER_H_)
#define SST_FLASH_DRIVER_H_

#include "common.h"

#define PAGE_SIZE           256
#define MAXIMUM_ADDRESS     0x7fffff
#define NUMBER_OF_PAGES     (MAXIMUM_ADDRESS / PAGE_SIZE)
#define SUCCESS             0

void spiFlash_ReadID(void);
int spiFlash_EraseChip(void);
int sf_MediaInit(u32 unused);
int sf_writePage(unsigned int page, unsigned char const *const buffer);
int sf_readPage(unsigned int page, unsigned char *const buffer);

#endif
