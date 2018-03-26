 # *******************************************************************************
# Main makefile project
# This makefile calls all the modules defined in the config.mk file
# *******************************************************************************

# *******************************************************************************
# DEFAULT DEFINITIONS
# These definitions can be overloaded from the command line
# *******************************************************************************
PROJECT		?= example
TARGET 		?= release
BOARD 		?= host
OPTIM 		?= high
ENABLE_DEP 	?= true
ARCH		?= cm4-freertos-gcc

OUTDIR		:= $(TOPDIR)build/output/


# Export them to be sure that they are available in sub-makefiles
export PROJECT
export TARGET
export BOARD
export DEFINES
export OPTIM
export ENABLE_DEP
export ARCH
export OUTDIR

# *******************************************************************************
# APPLICATION DEFINITIONS
# List of modules and extra libraries needed to generate project targets
# *******************************************************************************
LIB_UTILITIES		:= Utilities/Fonts
LIB_STM32F4XX_HAL	:= Drivers/STM32F4xx_HAL_Driver
LIB_469I_DISCO_BSP 	:= Drivers/BSP
LIB_CMSIS 			:= Drivers/CMSIS
LIB_DRIVERS			:= Drivers/Components/ft6x06 Drivers/Components/otm8009a
LIB_FREERTOS		:= Middlewares/Third_Party/FreeRTOS
LIB_FATFS			:= Middlewares/Third_Party/FatFs
LIB_USB_HOST		:= Middlewares/ST/STM32_USB_Host_Library

LIB_ALL				:= $(LIB_STM32F4XX_HAL) $(LIB_469I_DISCO_BSP) $(LIB_CMSIS) $(LIB_FREERTOS) $(LIB_DRIVERS) $(LIB_FATFS) $(LIB_USB_HOST)

# *******************************************************************************
# SERVER CONFIGURATION
# *******************************************************************************
ifeq ($(PROJECT), blinky)

DEFINES += -DUSE_HAL_DRIVER -DSTM32F469xx
GCC_PREFIX := arm-none-eabi-

APP_MODULES 	:= projects/blinky $(LIB_ALL)
APP_LIBPATH 	:= 
APP_LIBS 		:= 
APP_LDSCRIPT	:= STM32F469NIHx_FLASH.ld
endif

# *******************************************************************************
# CLIENT CONFIGURATION
# *******************************************************************************
ifeq ($(PROJECT), lcd)

DEFINES += -DDEBUG=0

APP_MODULES 	:= 
APP_LIBPATH 	:= 
APP_LIBS 		:= 

endif

# *******************************************************************************
# BUILD ENGINE
# *******************************************************************************
include build/Main.mk

#	$(call linker, $(OBJECTS), $(APP_LIBS), $(PROJECT))

clean:
	@echo "Cleaning generated files..."
	$(VERBOSE) $(RM) -rf $(OBJECTS) $(OUTDIR)/*.d $(OUTDIR)/*.gcov $(OUTDIR)/*.gcov.htm

wipe:
	@echo "Wiping output directory..."
	$(VERBOSE) $(RM) -rf $(OUTDIR)


# *******************************************************************************
# END OF MAKEFILE
# *******************************************************************************
