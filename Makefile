ARM=true
CFLAGS=-Iinc 
SOURCES= $(wildcard  src/*.c )
ADL_SOURCES=$(wildcard _ADL/*.c) #$(wildcard filesys/*.c)

#CFLAGS+=$(NEWCFLAGS)


BUILD_FLAGS=-DOAT_BUILD
BUILD_FLAGS+=-DI2C_ACCELEROMETER_ENABLED
BUILD_FLAGS+=-DDOTA_ENABLED
## Name of output
TARGET=XACT

LDFLAGS=-lm

REGRESSION_TESTING?=false
SST_FLASH?=false
ATMEL_FLASH?=true

ifeq ($(REGRESSION_TESTING), true)
BUILD_FLAGS+=-DREGRESSION_TESTING
endif

ifeq ($(ATMEL_FLASH),true)
SST_FLASH=false
BUILD_FLAGS+=-DATMEL_FLASH
FLASH_SOURCES= flash_src/atmel_flashDriver.c
endif

ifeq ($(SST_FLASH),true)
BUILD_FLAGS+=-DSST_FLASH
FLASH_SOURCES= flash_src/sst_flashDriver.c
else
endif



## Uncomment the following to enable Plug-Ins
#WIP=5.00.2040
#SECURITY=1.00.2030
#CGPS=3.04.2020
#CGPS_VARIANT=-OPUS-III
#LUA=1.00.2010
#GR=2.03.2000
WIP=5.10.2020
OAT_API_VERSION=621
OAT_OS_VERSION=6.21.02
OAT_FIRMWARE=R73a00
OAT_IDE_VERSION=1.08.02
OAT_GCC_VERSION=4.0.1.2
GCC_VERSION=4.0.1

# Optimize for debugging
#BUILD=debug
# Optimize for Size
#BUILD=-Os

#Python doesn't work in Thumb mode for some reason.
NOTHUMB=true

#Don't build a library
LIB=

CFLAGS+=$(BUILD_FLAGS) 

SOURCES+=$(FLASH_SOURCES)

ifeq ($(ARM),true) # Environment variable set by my build environment
SOURCES+=$(ADL_SOURCES)
ifeq ($(RTE),true)
include Makefile.rte # Works
else
include Makefile.target # Works
endif

else
SOURCES+=$(LOCAL_SOURCES) # Not yet attempted
include Makefile.local # Does not exist yet - will shortly
endif

refresh:
	rm $(TARGET).elf
_POSIX/main.c:
	make -C _POSIX main.c
