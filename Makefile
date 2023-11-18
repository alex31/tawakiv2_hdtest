##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

# Compiler options here.
# -Wdouble-promotion -fno-omit-frame-pointer

DEBUG := 1
OPT_SPEED := 2
OPT_SIZE := 3

#EXECMODE := $(DEBUG)
EXECMODE := $(OPT_SPEED)
#EXECMODE := $(OPT_SIZE)

GCCVERSIONGTEQ10 := $(shell expr `arm-none-eabi-gcc -dumpversion | cut -f1 -d.` \>= 10)
GCC_DIAG =  -Werror -Wno-error=unused-variable -Wno-error=format \
	    -Wno-error=cpp -Wno-error=type-limits \
            -Wno-error=unused-function \
            -Wunused -Wpointer-arith \
            -Werror=sign-compare \
            -Wshadow -Wparentheses -fmax-errors=5 \
            -ftrack-macro-expansion=2 -Wno-error=strict-overflow -Wstrict-overflow=2 \
            -Wvla-larger-than=600 -Wduplicated-branches -Wdangling-else \
	    -Wmisleading-indentation -Wduplicated-cond -Wduplicated-branches \
            -Wlogical-op -Wformat-overflow=2

G++_DIAG =   -Wnon-virtual-dtor -Woverloaded-virtual   \
	     -Wnull-dereference

ifeq "$(GCCVERSIONGTEQ10)" "1"
    GCC_DIAG += -Wno-error=volatile 
    G++_DIAG += -Wno-volatile -Wno-error=deprecated-declarations
endif

UNUSED_DIAGS = -Wcast-align -Wsign-conversion -Wconversion

ifeq ($(EXECMODE),$(DEBUG)) 
  USE_OPT =  -O0  -ggdb3  -Wall -Wextra \
	    -falign-functions=16 -fomit-frame-pointer \
	    $(GCC_DIAG)
  USE_LTO = no
endif

ifeq ($(EXECMODE),$(OPT_SPEED)) 
  USE_OPT =  -Ofast -flto -Wall -Wextra \
         -falign-functions=16 -fomit-frame-pointer \
         -DCH_DBG_SYSTEM_STATE_CHECK=0 -DCH_DBG_ENABLE_CHECKS=0 \
         -DCH_DBG_ENABLE_ASSERTS=0 -DCH_DBG_ENABLE_STACK_CHECK=0 \
          $(GCC_DIAG)
  USE_LTO = yes
endif

ifeq ($(EXECMODE),$(OPT_SIZE)) 
  USE_OPT =  -Os -flto  -Wall -Wextra \
	    -falign-functions=16 -fomit-frame-pointer \
            --specs=nano.specs \
            -DCH_DBG_SYSTEM_STATE_CHECK=0 -DCH_DBG_ENABLE_CHECKS=0 \
            -DCH_DBG_ENABLE_ASSERTS=0 -DCH_DBG_ENABLE_STACK_CHECK=0 \
            -DCH_DBG_FILL_THREADS=0 \
	    $(GCC_DIAG)
	USE_LTO = yes
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT = -std=gnu2x
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -std=c++23 -fno-rtti -fno-exceptions -fno-threadsafe-statics $(G++_DIAG)
endif


# Enable this if you want the linker to remove unused code and data
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# If enabled, this option allows to compile the application in THUMB mode.
ifeq ($(USE_THUMB),)
  USE_THUMB = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

# If enabled, this option makes the build process faster by not compiling
# modules not used in the current configuration.
ifeq ($(USE_SMART_BUILD),)
  USE_SMART_BUILD = yes
endif

#
# Build global options
##############################################################################

##############################################################################
# Architecture or project specific options
#

# Stack size to be allocated to the Cortex-M process stack. This stack is
# the stack used by the main() thread.
ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x1000
endif

# Stack size to the allocated to the Cortex-M main/exceptions stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x1000
endif

# Enables the use of FPU (no, softfp, hard).
ifeq ($(USE_FPU),)
  USE_FPU = hard
endif

# FPU-related options.
ifeq ($(USE_FPU_OPT),)
  USE_FPU_OPT = -mfloat-abi=$(USE_FPU) -mfpu=fpv5-d16
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, sources and paths
#

# Define project name here
PROJECT = ch
BOARD = TAWAKI_V200
MCU  = cortex-m7

# Imported source files and paths
MY_DIRNAME=../../../ChibiOS_21.11_stable
ifneq "$(wildcard $(MY_DIRNAME) )" ""
   RELATIVE=../../..
else
  RELATIVE=../..
endif
CHIBIOS  := $(RELATIVE)/$(notdir $(MY_DIRNAME))
STMSRC = $(RELATIVE)/COMMON/stm
EXTSRC = $(RELATIVE)/COMMON/ext
VARIOUS = $(RELATIVE)/COMMON/various
USBD_LIB   := $(VARIOUS)/Chibios-USB-Devices/mass_storage
ETL_LIB = ../../../../etl/include
TOOLDIR    := $(VARIOUS)/TOOLS

CONFDIR    := ./cfg
BUILDDIR   := ./build
DEPDIR     := ./.dep


# Licensing files.
include $(CHIBIOS)/os/license/license.mk
# Startup files.
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32h7xx.mk
# HAL-OSAL files (optional).
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32H7xx/platform.mk
include $(CONFDIR)/board.mk
include $(CHIBIOS)/os/hal/osal/rt-nil/osal.mk
# RTOS files (optional).
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMv7-M/compilers/GCC/mk/port.mk
# Auto-build files in ./source recursively.
include $(CHIBIOS)/tools/mk/autobuild.mk
# Other files (optional).
include $(CHIBIOS)/os/various/fatfs_bindings/fatfs.mk
include $(VARIOUS)/tlsf_bku/tlsf.mk


# Define linker script file here
STARTUPLD=$(CONFDIR)/ld/
LDSCRIPT= $(STARTUPLD)/STM32H743xI_TawakiH7.ld


# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(ALLCSRC) \
       $(TLSFSRC) \
       $(CHIBIOS)/os/various/syscalls.c \
       $(EXTSRC)/BMP3XX/bmp3.c \
       $(VARIOUS)/stdutil.c \
       $(VARIOUS)/printf.c \
       $(VARIOUS)/microrl/microrlShell.c \
       $(VARIOUS)/microrl/microrl.c \
       $(VARIOUS)/leds.c \
       $(VARIOUS)/i2cMaster.c \
       $(VARIOUS)/spiPeriphIvensenseV3.c \
       $(VARIOUS)/usb_serial.c \
       $(VARIOUS)/sdLog.c \
       $(VARIOUS)/msg_queue.c \
       $(VARIOUS)/sdio.c \
       $(VARIOUS)/sdioTest.c \
       $(USBD_LIB)/usb_msd.c




# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC = $(ALLCPPSRC)

# List ASM source files here.
ASMSRC = $(ALLASMSRC)

# List ASM with preprocessor source files here.
ASMXSRC = $(ALLXASMSRC)

# Inclusion directories.
INCDIR = $(CONFDIR) $(ALLINC) $(CHIBIOS)/os/various $(VARIOUS) \
	 $(EXTLIB) $(USBD_LIB) $(TLSFINC) $(EXTSRC) $(ETL_LIB) 

#
# Project, sources and paths
##############################################################################

# Define C warning options here.
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes

# Define C++ warning options here.
CPPWARN = -Wall -Wextra -Wundef

#
# Project, target, sources and paths
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS = -DTRACE

# Define ASM defines here
UADEFS = $(UDEFS)



# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS = -lm

#
# End of user defines
##############################################################################


RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk
include $(RULESPATH)/arm-none-eabi.mk
include $(RULESPATH)/rules.mk
$(OBJS): $(CONFDIR)/board.h

$(CONFDIR)/board.h: $(CONFDIR)/board.cfg
	$(TOOLDIR)/boardGen.pl --no-pp-line $<  $@ 


stflash: all
	@echo write $(BUILDDIR)/$(PROJECT).bin to flash memory
	/usr/local/bin/st-flash write  $(BUILDDIR)/$(PROJECT).bin 0x08000000
	@echo Done

flash: all
	@echo write $(BUILDDIR)/$(PROJECT).bin to flash memory
	bmpflash  $(BUILDDIR)/$(PROJECT).elf
	@echo Done
